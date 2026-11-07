// Null space of a redundant arm, with SDL3 main callbacks.
//
// An arm with four segments holds its hand on a target. The hand has two coordinates, the arm has four joint angles,
// so there is a two dimensional family of poses with the hand in the same place. Each fixed step (60 per second) the
// arm moves by  J+ e + (I - J+ J) phi: the first part brings the hand to the target, the second part is a free wish
// phi (rest pose, stay clear of the joint limits, stay relaxed) cut down to what does not touch the hand.
// Dragging an elbow or shaking pushes the arm around inside that family while the hand stays put. Switching the
// projection off adds phi unprojected, the usual mistake: the hand gets pulled off the target.
// The math (Jacobian, pseudoinverse, projector, the three wishes) lives in logic.hpp and is covered by test.cpp.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <string>
#include <vector>

#include "hud.hpp"
#include "logic.hpp"

namespace {

constexpr int kWidth = 960;
constexpr int kHeight = 540;
constexpr float kFieldTop = 34.0F;
constexpr float kFieldBottom = 440.0F;
constexpr float kFieldRight = 600.0F;
constexpr nsp::Vec2 kBase{250.0, 250.0};  // screen position of the arm's base
constexpr double kStep = 1.0 / 60.0;
constexpr double kMaxFrameTime = 0.25;
constexpr int kMaxStepsPerFrame = 8;
constexpr std::size_t kHistory = 240;  // steps shown in the graph
constexpr std::size_t kWindow = 12;    // steps averaged for the self motion readout
constexpr std::size_t kTrail = 50;
constexpr double kErrorScale = 20.0;   // px at the full width of the error bar and the graph
constexpr double kSelfScale = 3.0;     // rad/s at the full width of the self motion bar and the graph
constexpr float kGrabRadius = 16.0F;
constexpr float kChipY = 476.0F;
constexpr float kChipH = 34.0F;
constexpr float kPanelX = 616.0F;
constexpr float kPanelW = 324.0F;
constexpr float kInnerX = kPanelX + 16.0F;
constexpr float kInnerW = kPanelW - 32.0F;
constexpr gd::Color kInkSoft{17, 24, 39, 120};

enum class Action { Off, Rest, Limits, Comfort, Project, Shake, Auto, Reset };

struct Chip {
    Action action;
    std::string label;
    SDL_FRect rect;
};

struct Sample {
    double error = 0.0;  // px
    double self = 0.0;   // rad/s
    bool mark = false;   // the user changed the second goal or the projection right before this step
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;

    nsp::Angles start{};
    nsp::Angles theta{};
    nsp::Secondary mode = nsp::Secondary::Rest;
    bool projected = true;
    bool autoTarget = false;
    nsp::Autopilot autopilot;
    nsp::Vec2 manualTarget = nsp::kStartTarget;  // math coordinates relative to the base
    bool draggingTarget = false;
    std::size_t grab = 0;  // 0 = nothing, 1 to 3 = the joint that is being dragged
    nsp::Vec2 grabPos{};
    nsp::Angles kick{};
    int kickLeft = 0;
    unsigned long long kickSeed = 0;

    double accumulator = 0.0;
    Uint64 last = 0;
    std::vector<Sample> history;
    std::deque<double> recentSelf;  // rad per step
    std::array<std::deque<nsp::Vec2>, 3> trail;
    bool markNext = false;
    long maxFrames = -1;
    long frames = 0;
};

nsp::Vec2 target(const App& app) { return app.autoTarget ? app.autopilot.target() : app.manualTarget; }

SDL_FPoint toScreen(nsp::Vec2 m) { return {static_cast<float>(kBase.x + m.x), static_cast<float>(kBase.y - m.y)}; }

nsp::Vec2 toMath(float sx, float sy) { return {static_cast<double>(sx) - kBase.x, kBase.y - static_cast<double>(sy)}; }

bool inField(float x, float y) { return x >= 0.0F && x <= kFieldRight && y >= kFieldTop && y <= kFieldBottom; }

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
        float gapBefore;
    };
    const std::array<Def, 8> defs{{{Action::Off, "1 Off", 0.0F},
                                   {Action::Rest, "2 Rest pose", 6.0F},
                                   {Action::Limits, "3 Limits", 6.0F},
                                   {Action::Comfort, "4 Comfort", 6.0F},
                                   {Action::Project, "P Project", 20.0F},
                                   {Action::Shake, "S Shake", 6.0F},
                                   {Action::Auto, "A Auto", 6.0F},
                                   {Action::Reset, "R Reset", 6.0F}}};
    float x = 24.0F;
    for (const Def& d : defs) {
        x += d.gapBefore;
        const float w = app.hud.width(d.label, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({d.action, d.label, SDL_FRect{x, kChipY, w, kChipH}});
        x += w;
    }
}

void clearReadouts(App& app) {
    app.recentSelf.clear();
    app.markNext = true;
}

void reset(App& app) {
    app.theta = app.start;
    app.mode = nsp::Secondary::Rest;
    app.projected = true;
    app.autoTarget = false;
    app.autopilot = nsp::Autopilot{};
    app.manualTarget = nsp::kStartTarget;
    app.draggingTarget = false;
    app.grab = 0;
    app.kickLeft = 0;
    app.history.clear();
    app.recentSelf.clear();
    for (auto& t : app.trail) t.clear();
    app.markNext = false;
    app.accumulator = 0.0;
    app.last = SDL_GetPerformanceCounter();
}

void setMode(App& app, nsp::Secondary m) {
    if (app.mode == m) return;
    app.mode = m;
    clearReadouts(app);
}

void apply(App& app, Action action) {
    switch (action) {
        case Action::Off: setMode(app, nsp::Secondary::Off); break;
        case Action::Rest: setMode(app, nsp::Secondary::Rest); break;
        case Action::Limits: setMode(app, nsp::Secondary::Limits); break;
        case Action::Comfort: setMode(app, nsp::Secondary::Comfort); break;
        case Action::Project:
            app.projected = !app.projected;
            clearReadouts(app);
            break;
        case Action::Shake:
            app.kick = nsp::kickDirection(++app.kickSeed);
            for (double& v : app.kick) v *= nsp::kKickGain;
            app.kickLeft = nsp::kKickSteps;
            break;
        case Action::Auto:
            app.autoTarget = !app.autoTarget;
            if (!app.autoTarget) app.manualTarget = app.autopilot.target();  // freeze where it is
            app.draggingTarget = false;
            break;
        case Action::Reset: reset(app); break;
    }
}

bool active(const App& app, Action action) {
    switch (action) {
        case Action::Off: return app.mode == nsp::Secondary::Off;
        case Action::Rest: return app.mode == nsp::Secondary::Rest;
        case Action::Limits: return app.mode == nsp::Secondary::Limits;
        case Action::Comfort: return app.mode == nsp::Secondary::Comfort;
        case Action::Project: return app.projected;
        case Action::Shake: return app.kickLeft > 0;
        case Action::Auto: return app.autoTarget;
        case Action::Reset: return false;
    }
    return false;
}

void placeTarget(App& app, float sx, float sy) {
    const float x = std::clamp(sx, 16.0F, kFieldRight - 16.0F);
    const float y = std::clamp(sy, kFieldTop + 14.0F, kFieldBottom - 14.0F);
    app.manualTarget = toMath(x, y);
    app.autoTarget = false;
}

/// The joint (1 to 3) under the pointer, or 0.
std::size_t jointAt(const App& app, float sx, float sy) {
    const auto p = nsp::joints(nsp::kLengths, app.theta);
    std::size_t best = 0;
    float bestDist = kGrabRadius;
    for (std::size_t k = 1; k <= 3; ++k) {
        const SDL_FPoint s = toScreen(p[k]);
        const float d = std::hypot(s.x - sx, s.y - sy);
        if (d <= bestDist) {
            bestDist = d;
            best = k;
        }
    }
    return best;
}

void simulate(App& app) {
    if (app.autoTarget) app.autopilot.advance(kStep);

    nsp::Angles extra{};
    if (app.kickLeft > 0) {
        extra = app.kick;
        --app.kickLeft;
    }
    if (app.grab != 0) {
        const nsp::Angles g = nsp::grabPhi(nsp::kLengths, app.theta, app.grab, app.grabPos);
        for (std::size_t i = 0; i < nsp::kJoints; ++i) extra[i] += g[i];
    }

    const nsp::StepInfo info = nsp::iterate(app.theta, nsp::kLengths, target(app), app.mode, app.projected, extra);

    const nsp::Vec2 hand = nsp::endEffector(nsp::kLengths, app.theta);
    const nsp::Vec2 tgt = target(app);
    app.history.push_back({std::hypot(tgt.x - hand.x, tgt.y - hand.y), info.self / kStep, app.markNext});
    app.markNext = false;
    if (app.history.size() > kHistory) app.history.erase(app.history.begin());
    app.recentSelf.push_back(info.self);
    if (app.recentSelf.size() > kWindow) app.recentSelf.pop_front();
    const auto p = nsp::joints(nsp::kLengths, app.theta);
    for (std::size_t k = 0; k < 3; ++k) {
        app.trail[k].push_back(p[k + 1]);
        if (app.trail[k].size() > kTrail) app.trail[k].pop_front();
    }
}

/// 1 px lines are too thin on a 960x540 canvas that is scaled up, so draw a few side by side.
void thickLine(SDL_Renderer* r, SDL_FPoint a, SDL_FPoint b, gd::Color c, int half = 1) {
    gd::setColor(r, c);
    const bool flat = std::fabs(b.x - a.x) > std::fabs(b.y - a.y);
    for (int o = -half; o <= half; ++o) {
        const auto off = static_cast<float>(o);
        SDL_RenderLine(r, a.x + (flat ? 0.0F : off), a.y + (flat ? off : 0.0F), b.x + (flat ? 0.0F : off), b.y + (flat ? off : 0.0F));
    }
}

void dashedLine(SDL_Renderer* r, SDL_FPoint a, SDL_FPoint b, gd::Color c, float dash = 8.0F, float gap = 6.0F) {
    const float len = std::hypot(b.x - a.x, b.y - a.y);
    if (len < 1.0F) return;
    const float ux = (b.x - a.x) / len;
    const float uy = (b.y - a.y) / len;
    for (float t = 0.0F; t < len; t += dash + gap) {
        const float e = std::min(t + dash, len);
        thickLine(r, {a.x + ux * t, a.y + uy * t}, {a.x + ux * e, a.y + uy * e}, c, 0);
    }
}

void dashedRing(SDL_Renderer* r, float cx, float cy, float radius, gd::Color c) {
    gd::setColor(r, c);
    const int segments = 120;
    for (int i = 0; i < segments; i += 2) {
        const float a = 6.2831853F * static_cast<float>(i) / static_cast<float>(segments);
        const float b = 6.2831853F * static_cast<float>(i + 1) / static_cast<float>(segments);
        SDL_RenderLine(r, cx + std::cos(a) * radius, cy + std::sin(a) * radius, cx + std::cos(b) * radius, cy + std::sin(b) * radius);
    }
}

const char* modeName(nsp::Secondary m) {
    switch (m) {
        case nsp::Secondary::Off: return "no second goal";
        case nsp::Secondary::Rest: return "phi: rest pose";
        case nsp::Secondary::Limits: return "phi: avoid limits";
        case nsp::Secondary::Comfort: return "phi: comfort";
    }
    return "";
}

/// Short number that stays readable from 0.001 to 400.
void formatValue(char* out, std::size_t size, double v) {
    if (v < 10.0) {
        SDL_snprintf(out, size, "%.2f", v);
    } else if (v < 100.0) {
        SDL_snprintf(out, size, "%.1f", v);
    } else {
        SDL_snprintf(out, size, "%.0f", v);
    }
}

void drawBarRow(App& app, float y, const char* label, const char* value, double fraction, gd::Color tone) {
    SDL_Renderer* r = app.renderer;
    app.hud.text(r, kInnerX, y, label, gd::kMuted);
    app.hud.text(r, kInnerX + kInnerW - app.hud.width(value, gd::Hud::Style::Bold), y, value, tone, gd::Hud::Style::Bold);
    gd::fillRect(r, kInnerX, y + 22.0F, kInnerW, 12.0F, gd::kGrid);
    gd::fillRect(r, kInnerX, y + 22.0F, kInnerW * static_cast<float>(std::clamp(fraction, 0.0, 1.0)), 12.0F, tone);
    gd::strokeRect(r, kInnerX, y + 22.0F, kInnerW, 12.0F, gd::kInk);
}

void drawField(App& app, const nsp::Vec2 tgt, const nsp::Vec2 hand) {
    SDL_Renderer* r = app.renderer;
    const SDL_Rect clip{0, static_cast<int>(kFieldTop), static_cast<int>(kFieldRight), static_cast<int>(kFieldBottom - kFieldTop)};
    SDL_SetRenderClipRect(r, &clip);

    const SDL_FPoint base = toScreen({0.0, 0.0});
    dashedRing(r, base.x, base.y, static_cast<float>(nsp::reach(nsp::kLengths)), kInkSoft);

    // Where the three inner joints have been: self motion shows up as loops while the hand sits still.
    for (const auto& t : app.trail) {
        const std::size_t count = t.size();
        for (std::size_t i = 0; i < count; ++i) {
            const float age = static_cast<float>(i + 1) / static_cast<float>(count);
            const SDL_FPoint p = toScreen(t[i]);
            gd::disc(r, p.x, p.y, 2.0F, gd::Color{234, 88, 12, static_cast<Uint8>(30.0F + 140.0F * age)});
        }
    }

    // Error to the target.
    const SDL_FPoint hp = toScreen(hand);
    const SDL_FPoint tp = toScreen(tgt);
    dashedLine(r, hp, tp, gd::kMuted);

    // The arm.
    const auto p = nsp::joints(nsp::kLengths, app.theta);
    for (std::size_t i = 0; i < nsp::kJoints; ++i) thickLine(r, toScreen(p[i]), toScreen(p[i + 1]), gd::kInk, 3);
    gd::card(r, base.x - 14.0F, base.y - 14.0F, 28.0F, 28.0F, gd::kWhite, gd::kInk);
    for (std::size_t i = 1; i < nsp::kJoints; ++i) {
        const SDL_FPoint j = toScreen(p[i]);
        const bool pastLimit = std::fabs(nsp::limitPosition(app.theta, i)) > 1.0;
        const bool grabbed = app.grab == i;
        const gd::Color ring = (pastLimit || grabbed) ? gd::kOrange : gd::kInk;
        gd::disc(r, j.x, j.y, 8.0F, grabbed ? gd::kOrange : gd::kWhite);
        gd::ring(r, j.x, j.y, 8.0F, ring);
        gd::ring(r, j.x, j.y, 7.0F, ring);
    }
    // The base joint has a limit too, shown as a ring around the base card.
    if (std::fabs(nsp::limitPosition(app.theta, 0)) > 1.0) {
        gd::ring(r, base.x, base.y, 20.0F, gd::kOrange);
        gd::ring(r, base.x, base.y, 21.0F, gd::kOrange);
    }
    gd::disc(r, hp.x, hp.y, 9.0F, gd::kTeal);
    gd::ring(r, hp.x, hp.y, 9.0F, gd::kInk);
    gd::ring(r, hp.x, hp.y, 10.0F, gd::kInk);

    // The target.
    const gd::Color tc = gd::kTeal;
    gd::ring(r, tp.x, tp.y, 13.0F, tc);
    gd::ring(r, tp.x, tp.y, 14.0F, tc);
    thickLine(r, {tp.x - 20.0F, tp.y}, {tp.x - 8.0F, tp.y}, tc, 0);
    thickLine(r, {tp.x + 8.0F, tp.y}, {tp.x + 20.0F, tp.y}, tc, 0);
    thickLine(r, {tp.x, tp.y - 20.0F}, {tp.x, tp.y - 8.0F}, tc, 0);
    thickLine(r, {tp.x, tp.y + 8.0F}, {tp.x, tp.y + 20.0F}, tc, 0);

    SDL_SetRenderClipRect(r, nullptr);
}

void drawGraph(App& app, float gx, float gy, float gw, float gh) {
    SDL_Renderer* r = app.renderer;
    gd::fillRect(r, gx, gy, gw, gh, gd::kPaper);
    const float colW = gw / static_cast<float>(kHistory);
    const std::size_t n = app.history.size();
    auto yFor = [&](double fraction) { return gy + gh - gh * static_cast<float>(std::clamp(fraction, 0.0, 1.0)); };
    for (std::size_t i = 0; i < n; ++i) {
        if (!app.history[i].mark) continue;
        const float x = gx + gw - colW * static_cast<float>(n - i);
        gd::setColor(r, gd::kGrid);
        SDL_RenderLine(r, x, gy, x, gy + gh);
    }
    for (std::size_t i = 1; i < n; ++i) {
        const float x0 = gx + gw - colW * static_cast<float>(n - i + 1);
        const float x1 = gx + gw - colW * static_cast<float>(n - i);
        const Sample& a = app.history[i - 1];
        const Sample& b = app.history[i];
        thickLine(r, {x0, yFor(a.self / kSelfScale)}, {x1, yFor(b.self / kSelfScale)}, gd::kTeal, 0);
        thickLine(r, {x0, yFor(a.error / kErrorScale)}, {x1, yFor(b.error / kErrorScale)}, gd::kOrange, 0);
    }
    gd::strokeRect(r, gx, gy, gw, gh, gd::kInk);
}

void drawGauges(App& app, float y) {
    SDL_Renderer* r = app.renderer;
    const float x0 = kInnerX + 30.0F;
    const float w = kInnerW - 30.0F;
    for (std::size_t i = 0; i < nsp::kJoints; ++i) {
        const float ry = y + static_cast<float>(i) * 19.0F;
        char name[8];
        SDL_snprintf(name, sizeof name, "J%d", static_cast<int>(i) + 1);
        app.hud.text(r, kInnerX, ry - 5.0F, name, gd::kMuted);
        gd::fillRect(r, x0, ry, w, 10.0F, gd::kGrid);
        // The zone where the limit wish does nothing.
        if (app.mode == nsp::Secondary::Limits) gd::fillRect(r, x0 + w * 0.25F, ry, w * 0.5F, 10.0F, gd::kTealSoft);
        gd::strokeRect(r, x0, ry, w, 10.0F, gd::kInk);
        const double u = nsp::limitPosition(app.theta, i);
        const float mx = x0 + w * static_cast<float>(std::clamp((u + 1.0) / 2.0, -0.04, 1.04));
        const bool past = std::fabs(u) > 1.0;
        gd::fillRect(r, mx - 3.0F, ry - 3.0F, 6.0F, 16.0F, past ? gd::kOrange : gd::kTeal);
        gd::strokeRect(r, mx - 3.0F, ry - 3.0F, 6.0F, 16.0F, gd::kInk);
    }
}

void drawPanel(App& app, double error, double selfSpeed, double meanDeg, int dims) {
    SDL_Renderer* r = app.renderer;
    gd::card(r, kPanelX, 44.0F, kPanelW, 386.0F, gd::kWhite, gd::kInk);

    char text[24];
    formatValue(text, sizeof text, error);
    drawBarRow(app, 54.0F, "hand error (px)", text, error / kErrorScale, error < 1.0 ? gd::kTeal : gd::kOrange);
    formatValue(text, sizeof text, selfSpeed);
    drawBarRow(app, 100.0F, "self motion (rad/s)", text, selfSpeed / kSelfScale, gd::kTeal);
    formatValue(text, sizeof text, meanDeg);
    drawBarRow(app, 146.0F, "mean |angle| (deg)", text, meanDeg / 90.0, gd::kTeal);

    app.hud.text(r, kInnerX, 192.0F, "history", gd::kMuted);
    app.hud.text(r, kInnerX + 84.0F, 192.0F, "error", gd::kOrange);
    app.hud.text(r, kInnerX + 144.0F, 192.0F, "self motion", gd::kTeal);
    drawGraph(app, kInnerX, 216.0F, kInnerW, 60.0F);

    app.hud.text(r, kInnerX, 286.0F, "joint inside its range", gd::kMuted);
    drawGauges(app, 318.0F);

    app.hud.text(r, kInnerX, 396.0F, app.projected ? "projection: on" : "projection: off", app.projected ? gd::kTeal : gd::kOrange, gd::Hud::Style::Bold);
    char dimsText[32];
    SDL_snprintf(dimsText, sizeof dimsText, "free dims: %d", dims);
    app.hud.text(r, kInnerX + kInnerW - app.hud.width(dimsText, gd::Hud::Style::Bold), 396.0F, dimsText, gd::kInk, gd::Hud::Style::Bold);
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    const nsp::Vec2 tgt = target(app);
    const nsp::Vec2 hand = nsp::endEffector(nsp::kLengths, app.theta);
    const double error = std::hypot(tgt.x - hand.x, tgt.y - hand.y);
    double selfSum = 0.0;
    for (const double s : app.recentSelf) selfSum += s;
    const double selfSpeed = app.recentSelf.empty() ? 0.0 : selfSum / static_cast<double>(app.recentSelf.size()) / kStep;
    const double meanDeg = nsp::meanAbsAngle(app.theta) * 180.0 / nsp::kPi;
    const int dims = nsp::nullDimensions(nsp::jacobian(nsp::kLengths, app.theta));

    gd::backdrop(r, kWidth, kHeight);
    char right[64];
    SDL_snprintf(right, sizeof right, "%s%s", modeName(app.mode), app.mode == nsp::Secondary::Off ? "" : (app.projected ? ", projected" : ", NOT projected"));
    app.hud.titleBar(r, kWidth, "[ NULL SPACE ]", right);

    drawField(app, tgt, hand);
    drawPanel(app, error, selfSpeed, meanDeg, dims);

    app.hud.text(r, 24.0F, 450.0F, "drag a joint: hand stays   drag elsewhere: move target", gd::kMuted);

    for (const Chip& c : app.chips) {
        const bool on = active(app, c.action);
        const bool naive = c.action == Action::Project && !app.projected;
        const gd::Color fill = on ? gd::kTeal : (naive ? gd::kOrange : gd::kWhite);
        const gd::Color ink = (on || naive) ? gd::kWhite : gd::kInk;
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, fill, naive ? gd::kOrange : gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 6.0F, c.label, ink, gd::Hud::Style::Bold);
    }
}

}  // namespace

SDL_AppResult SDL_AppInit(void** appstate, int argc, char** argv) {
    auto* app = new App();
    *appstate = app;
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::strcmp(argv[i], "--frames") == 0) {
            app->maxFrames = std::atol(argv[i + 1]);
        }
    }
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        SDL_Log("SDL_Init failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    if (!SDL_CreateWindowAndRenderer("redundancy-nullspace", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, gd::kLogicalPresentation);
    SDL_SetRenderVSync(app->renderer, 1);
    layoutChips(*app);
    app->start = nsp::startPose();
    app->theta = app->start;
    app->last = SDL_GetPerformanceCounter();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION:
            if (app.draggingTarget) placeTarget(app, event->motion.x, event->motion.y);
            if (app.grab != 0) app.grabPos = toMath(event->motion.x, event->motion.y);
            break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const SDL_FPoint p{event->button.x, event->button.y};
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) {
                    apply(app, c.action);
                    return SDL_APP_CONTINUE;
                }
            }
            if (!inField(p.x, p.y)) break;
            const std::size_t k = jointAt(app, p.x, p.y);
            if (k != 0) {
                app.grab = k;
                app.grabPos = toMath(p.x, p.y);
            } else {
                app.draggingTarget = true;
                placeTarget(app, p.x, p.y);
            }
            break;
        }
        // A release outside the canvas or the window never arrives here, so let go when the pointer or the focus leaves.
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event->button.button == SDL_BUTTON_LEFT) {
                app.draggingTarget = false;
                app.grab = 0;
            }
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
        case SDL_EVENT_WINDOW_FOCUS_LOST:
            app.draggingTarget = false;
            app.grab = 0;
            break;
        case SDL_EVENT_KEY_DOWN:
            // Everything here toggles or triggers and must not fire while the key is held.
            if (event->key.repeat) break;
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: apply(app, Action::Off); break;
                case SDLK_2: apply(app, Action::Rest); break;
                case SDLK_3: apply(app, Action::Limits); break;
                case SDLK_4: apply(app, Action::Comfort); break;
                case SDLK_P: apply(app, Action::Project); break;
                case SDLK_S: apply(app, Action::Shake); break;
                case SDLK_A: apply(app, Action::Auto); break;
                case SDLK_R: apply(app, Action::Reset); break;
                default: break;
            }
            break;
        default: break;
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void* appstate) {
    auto& app = *static_cast<App*>(appstate);

    const Uint64 now = SDL_GetPerformanceCounter();
    const double frameTime = std::min(static_cast<double>(now - app.last) / static_cast<double>(SDL_GetPerformanceFrequency()), kMaxFrameTime);
    app.last = now;
    app.accumulator += frameTime;
    int steps = 0;
    while (app.accumulator >= kStep && steps < kMaxStepsPerFrame) {
        simulate(app);
        app.accumulator -= kStep;
        ++steps;
    }
    if (steps == kMaxStepsPerFrame) app.accumulator = 0.0;  // too far behind: drop the rest instead of spiraling

    drawScene(app);
    SDL_RenderPresent(app.renderer);

    ++app.frames;
    return (app.maxFrames >= 0 && app.frames >= app.maxFrames) ? SDL_APP_SUCCESS : SDL_APP_CONTINUE;
}

void SDL_AppQuit(void* appstate, SDL_AppResult) {
    auto* app = static_cast<App*>(appstate);
    if (app != nullptr) {
        app->hud.destroy();
        if (app->renderer != nullptr) SDL_DestroyRenderer(app->renderer);
        if (app->window != nullptr) SDL_DestroyWindow(app->window);
        delete app;
    }
}
