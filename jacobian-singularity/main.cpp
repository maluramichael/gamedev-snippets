// Jacobian inverse kinematics near the stretched pose, with SDL3 main callbacks.
//
// An arm with three segments chases a target. Each fixed step (60 per second) one iteration of the chosen method
// turns the error into joint angle steps. The pseudoinverse divides by the smallest singular value of the Jacobian,
// which is close to 0 when the arm is nearly stretched, so a pixel of error becomes a huge swing and the arm jitters.
// Damped least squares caps that swing (at most 1 / (2 lambda) rad per pixel), the transpose never divides at all.
// The target wanders along the edge of the reach by itself (Auto), dragging in the field places it by hand.
// The math (Jacobian, the three steps, the clamp) lives in logic.hpp and is covered by test.cpp.

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
constexpr jac::Vec2 kBase{240.0, 240.0};  // screen position of the arm's base
constexpr double kStep = 1.0 / 60.0;
constexpr double kMaxFrameTime = 0.25;
constexpr int kMaxStepsPerFrame = 8;
constexpr std::size_t kHistory = 240;  // steps shown in the graph
constexpr std::size_t kPeakWindow = 12;
constexpr std::size_t kTrail = 40;
constexpr double kLambdaMin = 1.0;
constexpr double kLambdaMax = 80.0;
constexpr double kLambdaDefault = 30.0;
constexpr double kNearSingular = 10.0;  // sigma_min below this (px per rad) counts as "near singular"
constexpr float kChipY = 476.0F;
constexpr float kChipH = 34.0F;
constexpr float kPanelX = 616.0F;
constexpr float kPanelW = 324.0F;
constexpr float kInnerX = kPanelX + 16.0F;
constexpr float kInnerW = kPanelW - 32.0F;
constexpr SDL_FRect kSliderTrack{kInnerX, 342.0F, kInnerW, 6.0F};
constexpr gd::Color kInkSoft{17, 24, 39, 120};
constexpr jac::Angles kStartPose{0.16, 0.06, -0.06};

enum class Action { Transpose, PseudoInverse, Damped, Clamp, Auto, Reset };

struct Chip {
    Action action;
    std::string label;
    SDL_FRect rect;
};

struct Sample {
    double step = 0.0;
    bool mark = false;  // the user changed method, lambda or clamp right before this step
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;

    jac::Angles theta = kStartPose;
    jac::Method method = jac::Method::PseudoInverse;
    double lambda = kLambdaDefault;
    bool clamp = false;
    bool autoTarget = true;
    jac::Autopilot autopilot;
    jac::Vec2 manualTarget{200.0, 0.0};  // math coordinates relative to the base
    bool draggingTarget = false;
    bool draggingSlider = false;

    double accumulator = 0.0;
    Uint64 last = 0;
    std::vector<Sample> history;
    std::deque<double> recent;  // the last steps since the setting changed, for the peak readout
    std::deque<jac::Vec2> trail;
    bool markNext = false;
    long maxFrames = -1;
    long frames = 0;
};

jac::Vec2 target(const App& app) { return app.autoTarget ? app.autopilot.target() : app.manualTarget; }

SDL_FPoint toScreen(jac::Vec2 m) { return {static_cast<float>(kBase.x + m.x), static_cast<float>(kBase.y - m.y)}; }

jac::Vec2 toMath(float sx, float sy) { return {static_cast<double>(sx) - kBase.x, kBase.y - static_cast<double>(sy)}; }

bool inField(float x, float y) { return x >= 0.0F && x <= kFieldRight && y >= kFieldTop && y <= kFieldBottom; }

double lambdaFromSlider(float x) {
    const double t = std::clamp((static_cast<double>(x) - static_cast<double>(kSliderTrack.x)) / static_cast<double>(kSliderTrack.w), 0.0, 1.0);
    return kLambdaMin * std::pow(kLambdaMax / kLambdaMin, t);
}

float sliderFromLambda(double lambda) {
    const double t = std::log(std::clamp(lambda, kLambdaMin, kLambdaMax) / kLambdaMin) / std::log(kLambdaMax / kLambdaMin);
    return kSliderTrack.x + static_cast<float>(t) * kSliderTrack.w;
}

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
        float gapBefore;
    };
    const std::array<Def, 6> defs{{{Action::Transpose, "1 Transpose", 0.0F},
                                   {Action::PseudoInverse, "2 Pseudoinverse", 6.0F},
                                   {Action::Damped, "3 Damped", 6.0F},
                                   {Action::Clamp, "C Clamp", 20.0F},
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
    app.recent.clear();
    app.markNext = true;
    app.accumulator = 0.0;
}

void reset(App& app) {
    app.theta = kStartPose;
    app.autopilot = jac::Autopilot{};
    app.autoTarget = true;
    app.draggingTarget = false;
    app.draggingSlider = false;
    app.history.clear();
    app.recent.clear();
    app.trail.clear();
    app.markNext = false;
    app.accumulator = 0.0;
    app.last = SDL_GetPerformanceCounter();
}

void setMethod(App& app, jac::Method m) {
    if (app.method == m) return;
    app.method = m;
    clearReadouts(app);
}

void apply(App& app, Action action) {
    switch (action) {
        case Action::Transpose: setMethod(app, jac::Method::Transpose); break;
        case Action::PseudoInverse: setMethod(app, jac::Method::PseudoInverse); break;
        case Action::Damped: setMethod(app, jac::Method::Damped); break;
        case Action::Clamp:
            app.clamp = !app.clamp;
            clearReadouts(app);
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
        case Action::Transpose: return app.method == jac::Method::Transpose;
        case Action::PseudoInverse: return app.method == jac::Method::PseudoInverse;
        case Action::Damped: return app.method == jac::Method::Damped;
        case Action::Clamp: return app.clamp;
        case Action::Auto: return app.autoTarget;
        case Action::Reset: return false;
    }
    return false;
}

void setLambda(App& app, double lambda) {
    const double next = std::clamp(lambda, kLambdaMin, kLambdaMax);
    if (next == app.lambda) return;
    app.lambda = next;
    clearReadouts(app);
}

void placeTarget(App& app, float sx, float sy) {
    const float x = std::clamp(sx, 16.0F, kFieldRight - 16.0F);
    const float y = std::clamp(sy, kFieldTop + 14.0F, kFieldBottom - 14.0F);
    app.manualTarget = toMath(x, y);
    app.autoTarget = false;
}

void simulate(App& app) {
    if (app.autoTarget) app.autopilot.advance(kStep);
    const jac::Angles d = jac::iterate(app.theta, jac::kLengths, target(app), app.method, app.lambda, app.clamp ? jac::kDmax : 0.0);
    const double s = jac::norm(d);
    app.history.push_back({s, app.markNext});
    app.markNext = false;
    if (app.history.size() > kHistory) app.history.erase(app.history.begin());
    app.recent.push_back(s);
    if (app.recent.size() > kPeakWindow) app.recent.pop_front();
    app.trail.push_back(jac::endEffector(jac::kLengths, app.theta));
    if (app.trail.size() > kTrail) app.trail.pop_front();
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

const char* methodName(jac::Method m) {
    switch (m) {
        case jac::Method::Transpose: return "transpose";
        case jac::Method::PseudoInverse: return "pseudoinverse";
        case jac::Method::Damped: return "damped least squares";
    }
    return "";
}

/// Short number that stays readable from 0.001 to 400.
void formatValue(char* out, std::size_t size, double v) {
    if (v < 10.0) {
        SDL_snprintf(out, size, "%.3f", v);
    } else if (v < 100.0) {
        SDL_snprintf(out, size, "%.1f", v);
    } else {
        SDL_snprintf(out, size, "%.0f", v);
    }
}

void drawBarRow(App& app, float y, const char* label, double value, double fraction, gd::Color tone) {
    SDL_Renderer* r = app.renderer;
    app.hud.text(r, kInnerX, y, label, gd::kMuted);
    char text[24];
    formatValue(text, sizeof text, value);
    app.hud.text(r, kInnerX + kInnerW - app.hud.width(text, gd::Hud::Style::Bold), y, text, tone, gd::Hud::Style::Bold);
    gd::fillRect(r, kInnerX, y + 22.0F, kInnerW, 12.0F, gd::kGrid);
    gd::fillRect(r, kInnerX, y + 22.0F, kInnerW * static_cast<float>(std::clamp(fraction, 0.0, 1.0)), 12.0F, tone);
    gd::strokeRect(r, kInnerX, y + 22.0F, kInnerW, 12.0F, gd::kInk);
}

void drawField(App& app, const jac::Vec2 tgt, const jac::Vec2 hand, bool nearSingular) {
    SDL_Renderer* r = app.renderer;
    const SDL_Rect clip{0, static_cast<int>(kFieldTop), static_cast<int>(kFieldRight), static_cast<int>(kFieldBottom - kFieldTop)};
    SDL_SetRenderClipRect(r, &clip);

    const SDL_FPoint base = toScreen({0.0, 0.0});
    dashedRing(r, base.x, base.y, static_cast<float>(jac::reach(jac::kLengths)), kInkSoft);

    // Where the end effector has been: the jitter shows up as a scatter.
    const std::size_t trailCount = app.trail.size();
    for (std::size_t i = 0; i < trailCount; ++i) {
        const float age = static_cast<float>(i + 1) / static_cast<float>(trailCount);
        const SDL_FPoint p = toScreen(app.trail[i]);
        gd::disc(r, p.x, p.y, 2.5F, gd::Color{234, 88, 12, static_cast<Uint8>(40.0F + 150.0F * age)});
    }

    // Error to the target (dashed), and the part of it the clamp lets through.
    const SDL_FPoint hp = toScreen(hand);
    const SDL_FPoint tp = toScreen(tgt);
    dashedLine(r, hp, tp, gd::kMuted);
    if (app.clamp) {
        const jac::Vec2 e = jac::clampMag({tgt.x - hand.x, tgt.y - hand.y}, jac::kDmax);
        thickLine(r, hp, toScreen({hand.x + e.x, hand.y + e.y}), gd::kTeal, 1);
    }

    // The direction the end effector can barely move in: along the arm when it is stretched.
    {
        const jac::Vec2 w = jac::weakDirection(jac::jacobian(jac::kLengths, app.theta));
        const gd::Color c = nearSingular ? gd::kOrange : kInkSoft;
        thickLine(r, toScreen({hand.x - w.x * 34.0, hand.y - w.y * 34.0}), toScreen({hand.x + w.x * 34.0, hand.y + w.y * 34.0}), c, 1);
    }

    // The arm.
    const auto p = jac::joints(jac::kLengths, app.theta);
    for (std::size_t i = 0; i < 3; ++i) thickLine(r, toScreen(p[i]), toScreen(p[i + 1]), gd::kInk, 3);
    gd::card(r, base.x - 14.0F, base.y - 14.0F, 28.0F, 28.0F, gd::kWhite, gd::kInk);
    for (std::size_t i = 1; i < 3; ++i) {
        const SDL_FPoint j = toScreen(p[i]);
        gd::disc(r, j.x, j.y, 8.0F, gd::kWhite);
        gd::ring(r, j.x, j.y, 8.0F, gd::kInk);
        gd::ring(r, j.x, j.y, 7.0F, gd::kInk);
    }
    gd::disc(r, hp.x, hp.y, 9.0F, gd::kTeal);
    gd::ring(r, hp.x, hp.y, 9.0F, gd::kInk);
    gd::ring(r, hp.x, hp.y, 10.0F, gd::kInk);

    // The target: teal while reachable, orange beyond the reach.
    const bool beyond = jac::norm(tgt) > jac::reach(jac::kLengths);
    const gd::Color tc = beyond ? gd::kOrange : gd::kTeal;
    gd::ring(r, tp.x, tp.y, 13.0F, tc);
    gd::ring(r, tp.x, tp.y, 14.0F, tc);
    thickLine(r, {tp.x - 20.0F, tp.y}, {tp.x - 8.0F, tp.y}, tc, 0);
    thickLine(r, {tp.x + 8.0F, tp.y}, {tp.x + 20.0F, tp.y}, tc, 0);
    thickLine(r, {tp.x, tp.y - 20.0F}, {tp.x, tp.y - 8.0F}, tc, 0);
    thickLine(r, {tp.x, tp.y + 8.0F}, {tp.x, tp.y + 20.0F}, tc, 0);

    SDL_SetRenderClipRect(r, nullptr);
}

void drawPanel(App& app, double sigmaMin, double peak, double dist, bool nearSingular, bool beyond) {
    SDL_Renderer* r = app.renderer;
    gd::card(r, kPanelX, 44.0F, kPanelW, 386.0F, gd::kWhite, gd::kInk);

    const gd::Color sigmaTone = nearSingular ? gd::kOrange : gd::kTeal;
    drawBarRow(app, 56.0F, "min singular value", sigmaMin, sigmaMin / 80.0, sigmaTone);
    const gd::Color stepTone = peak > 0.5 ? gd::kOrange : gd::kTeal;
    drawBarRow(app, 106.0F, "joint step (rad, peak)", peak, peak / jac::kPi, stepTone);
    drawBarRow(app, 156.0F, "error to target (px)", dist, dist / 100.0, beyond ? gd::kOrange : gd::kTeal);

    // History of the joint step per iteration.
    app.hud.text(r, kInnerX, 204.0F, "step history (0 to pi rad)", gd::kMuted);
    const float gx = kInnerX;
    const float gy = 226.0F;
    const float gw = kInnerW;
    const float gh = 76.0F;
    gd::fillRect(r, gx, gy, gw, gh, gd::kPaper);
    gd::setColor(r, gd::kGrid);
    SDL_RenderLine(r, gx, gy + gh * 0.5F, gx + gw, gy + gh * 0.5F);
    const float colW = gw / static_cast<float>(kHistory);
    const std::size_t n = app.history.size();
    for (std::size_t i = 0; i < n; ++i) {
        const Sample& s = app.history[i];
        const float x = gx + gw - colW * static_cast<float>(n - i);
        if (s.mark) {
            gd::setColor(r, gd::kTeal);
            SDL_RenderLine(r, x, gy, x, gy + gh);
        }
        const float h = gh * static_cast<float>(std::min(s.step / jac::kPi, 1.0));
        if (h > 0.0F) gd::fillRect(r, x, gy + gh - h, std::max(colW, 1.0F), h, s.step > 0.5 ? gd::kOrange : gd::kTeal);
    }
    gd::strokeRect(r, gx, gy, gw, gh, gd::kInk);

    // Damping slider.
    const bool dampedOn = app.method == jac::Method::Damped;
    const gd::Color sliderInk = dampedOn ? gd::kInk : gd::kMuted;
    app.hud.text(r, kInnerX, 314.0F, "damping lambda  [ ]", gd::kMuted);
    char text[24];
    formatValue(text, sizeof text, app.lambda);
    app.hud.text(r, kInnerX + kInnerW - app.hud.width(text, gd::Hud::Style::Bold), 314.0F, text, sliderInk, gd::Hud::Style::Bold);
    gd::fillRect(r, kSliderTrack.x, kSliderTrack.y, kSliderTrack.w, kSliderTrack.h, gd::kGrid);
    gd::strokeRect(r, kSliderTrack.x, kSliderTrack.y, kSliderTrack.w, kSliderTrack.h, sliderInk);
    const float hx = sliderFromLambda(app.lambda);
    gd::card(r, hx - 6.0F, kSliderTrack.y - 8.0F, 12.0F, 22.0F, dampedOn ? gd::kTeal : gd::kWhite, sliderInk);
    app.hud.text(r, kInnerX, 358.0F, dampedOn ? "step <= error / (2 lambda)" : "lambda: method 3 only", gd::kMuted);

    // State.
    app.hud.text(r, kInnerX, 386.0F, nearSingular ? "pose: near singular" : "pose: regular", sigmaTone, gd::Hud::Style::Bold);
    app.hud.text(r, kInnerX, 406.0F, beyond ? "target: out of reach" : "target: in reach", beyond ? gd::kOrange : gd::kTeal, gd::Hud::Style::Bold);
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    const jac::Vec2 tgt = target(app);
    const jac::Vec2 hand = jac::endEffector(jac::kLengths, app.theta);
    const double sigmaMin = jac::singularValues(jac::jacobian(jac::kLengths, app.theta)).min;
    const bool nearSingular = sigmaMin < kNearSingular;
    const bool beyond = jac::norm(tgt) > jac::reach(jac::kLengths);
    const double dist = std::hypot(tgt.x - hand.x, tgt.y - hand.y);
    double peak = 0.0;
    for (const double s : app.recent) peak = std::max(peak, s);

    gd::backdrop(r, kWidth, kHeight);
    char right[64];
    if (app.method == jac::Method::Damped) {
        SDL_snprintf(right, sizeof right, "%s, lambda %.1f%s", methodName(app.method), app.lambda, app.clamp ? ", clamp" : "");
    } else {
        SDL_snprintf(right, sizeof right, "%s%s", methodName(app.method), app.clamp ? ", clamp" : "");
    }
    app.hud.titleBar(r, kWidth, "[ JACOBIAN SINGULARITY ]", right);

    drawField(app, tgt, hand, nearSingular);
    drawPanel(app, sigmaMin, peak, dist, nearSingular, beyond);

    char hint[96];
    SDL_snprintf(hint, sizeof hint, "drag: place target   orange bar: hardest direction   clamp: error <= %.0f px", jac::kDmax);
    app.hud.text(r, 24.0F, 450.0F, hint, gd::kMuted);

    for (const Chip& c : app.chips) {
        const bool on = active(app, c.action);
        const bool naive = c.action == Action::PseudoInverse;
        const gd::Color fill = on ? (naive ? gd::kOrange : gd::kTeal) : gd::kWhite;
        const gd::Color ink = on ? gd::kWhite : (naive ? gd::kOrange : gd::kInk);
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, fill, naive && !on ? gd::kOrange : gd::kInk);
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
    if (!SDL_CreateWindowAndRenderer("jacobian-singularity", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
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
            if (app.draggingSlider) setLambda(app, lambdaFromSlider(event->motion.x));
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
            const SDL_FRect grab{kSliderTrack.x - 10.0F, kSliderTrack.y - 12.0F, kSliderTrack.w + 20.0F, kSliderTrack.h + 24.0F};
            if (SDL_PointInRectFloat(&p, &grab)) {
                app.draggingSlider = true;
                setLambda(app, lambdaFromSlider(p.x));
            } else if (inField(p.x, p.y)) {
                app.draggingTarget = true;
                placeTarget(app, p.x, p.y);
            }
            break;
        }
        // A release outside the canvas or the window never arrives here, so let go when the pointer or the focus leaves.
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event->button.button == SDL_BUTTON_LEFT) {
                app.draggingTarget = false;
                app.draggingSlider = false;
            }
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
        case SDL_EVENT_WINDOW_FOCUS_LOST:
            app.draggingTarget = false;
            app.draggingSlider = false;
            break;
        case SDL_EVENT_KEY_DOWN:
            // Brackets scrub lambda and may repeat, everything else toggles or triggers and must not fire while held.
            if (event->key.key == SDLK_LEFTBRACKET) {
                setLambda(app, app.lambda / 1.08);
                break;
            }
            if (event->key.key == SDLK_RIGHTBRACKET) {
                setLambda(app, app.lambda * 1.08);
                break;
            }
            if (event->key.repeat) break;
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: apply(app, Action::Transpose); break;
                case SDLK_2: apply(app, Action::PseudoInverse); break;
                case SDLK_3: apply(app, Action::Damped); break;
                case SDLK_C: apply(app, Action::Clamp); break;
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
        SDL_DestroyRenderer(app->renderer);
        SDL_DestroyWindow(app->window);
        delete app;
    }
}
