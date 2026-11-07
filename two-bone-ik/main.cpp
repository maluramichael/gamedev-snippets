// Two-bone IK with SDL3 main callbacks.
//
// An arm of two segments follows the mouse without any iteration: root, joint and target are the corners of a
// triangle with the known sides d1, d2 and h (root to target), and the law of cosines gives the angles directly.
// The teal ring is every point the end can reach, |d1 - d2| <= h <= d1 + d2. The two circles (radius d1 around the root,
// radius d2 around the target) meet in the two knee positions, the second one is drawn as an open ring. Outside the
// ring the textbook formula asks for acos of a number beyond 1: "Naive" shows what that does, "Checked" stretches or
// folds the arm towards the target instead. Until the mouse moves, the target runs a path through hole, ring and the
// outside, so the first frame already shows a bent arm. The math lives in logic.hpp and is covered by test.cpp.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "hud.hpp"
#include "logic.hpp"

namespace {

constexpr int kWidth = 960;
constexpr int kHeight = 540;
constexpr float kFieldTop = 34.0F;
constexpr float kFieldBottom = 440.0F;
constexpr float kChipY = 476.0F;
constexpr float kChipH = 34.0F;
constexpr ik::Vec2 kRoot{330.0, 240.0};
constexpr double kMaxDt = 0.25;
constexpr gd::Color kMutedSoft{107, 114, 128, 130};
constexpr gd::Color kRingFill{13, 148, 136, 44};

struct Lengths {
    double d1;
    double d2;
};
constexpr Lengths kUnequal{125.0, 70.0};  // hole of radius 55 in the middle
constexpr Lengths kEqual{98.0, 98.0};     // no hole: the ring becomes a disc

enum class Action { BendLeft, BendRight, Checked, Naive, Equal, Auto };

struct Chip {
    Action action;
    std::string label;
    SDL_FRect rect;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;
    ik::Bend bend = ik::Bend::Left;
    bool naive = false;
    bool equal = false;
    bool autoPath = true;
    double phase = 0.0;
    ik::Vec2 target = {0.0, 0.0};
    ik::Pose lastValid{};  // the naive solver's last finite pose, shown faint while it returns NaN
    bool haveLast = false;
    Uint64 lastTicks = 0;
    long maxFrames = -1;
    long frames = 0;
};

Lengths lengths(const App& app) { return app.equal ? kEqual : kUnequal; }

bool inField(ik::Vec2 p) { return p.y >= static_cast<double>(kFieldTop) && p.y <= static_cast<double>(kFieldBottom); }

bool inPanel(ik::Vec2 p) { return p.x >= 660.0 && p.x <= 936.0 && p.y >= 44.0 && p.y <= 296.0; }

SDL_FPoint fp(ik::Vec2 v) { return {static_cast<float>(v.x), static_cast<float>(v.y)}; }

bool finitePose(const ik::Pose& p) { return std::isfinite(p.joint.x) && std::isfinite(p.joint.y) && std::isfinite(p.end.x) && std::isfinite(p.end.y); }

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
        float gapBefore;
    };
    const std::array<Def, 6> defs{{{Action::BendLeft, "1 Knee left", 0.0F},
                                   {Action::BendRight, "2 Knee right", 8.0F},
                                   {Action::Checked, "3 Checked", 24.0F},
                                   {Action::Naive, "4 Naive", 8.0F},
                                   {Action::Equal, "5 Equal lengths", 24.0F},
                                   {Action::Auto, "A Auto path", 8.0F}}};
    float x = 40.0F;
    for (const Def& d : defs) {
        x += d.gapBefore;
        const float w = app.hud.width(d.label, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({d.action, d.label, SDL_FRect{x, kChipY, w, kChipH}});
        x += w;
    }
}

void forgetNaivePose(App& app) { app.haveLast = false; }

void apply(App& app, Action action) {
    switch (action) {
        case Action::BendLeft: app.bend = ik::Bend::Left; break;
        case Action::BendRight: app.bend = ik::Bend::Right; break;
        case Action::Checked: app.naive = false; break;
        case Action::Naive:
            app.naive = true;
            forgetNaivePose(app);
            break;
        case Action::Equal:
            app.equal = !app.equal;
            forgetNaivePose(app);
            break;
        case Action::Auto:
            app.autoPath = !app.autoPath;
            if (app.autoPath) app.phase = 0.0;  // restart the path from its first (in range) point
            break;
    }
}

bool active(const App& app, Action action) {
    switch (action) {
        case Action::BendLeft: return app.bend == ik::Bend::Left;
        case Action::BendRight: return app.bend == ik::Bend::Right;
        case Action::Checked: return !app.naive;
        case Action::Naive: return app.naive;
        case Action::Equal: return app.equal;
        case Action::Auto: return app.autoPath;
    }
    return false;
}

void thickLine(SDL_Renderer* r, ik::Vec2 a, ik::Vec2 b, gd::Color c, int half = 0) {
    gd::setColor(r, c);
    const bool flat = std::fabs(b.x - a.x) > std::fabs(b.y - a.y);
    for (int o = -half; o <= half; ++o) {
        const auto off = static_cast<float>(o);
        SDL_RenderLine(r, static_cast<float>(a.x) + (flat ? 0.0F : off), static_cast<float>(a.y) + (flat ? off : 0.0F), static_cast<float>(b.x) + (flat ? 0.0F : off),
                       static_cast<float>(b.y) + (flat ? off : 0.0F));
    }
}

void dashedLine(SDL_Renderer* r, ik::Vec2 a, ik::Vec2 b, gd::Color c) {
    const double len = ik::length(b - a);
    if (len < 1.0) return;
    const ik::Vec2 dir = (b - a) * (1.0 / len);
    for (double s = 0.0; s < len; s += 14.0) {
        thickLine(r, a + dir * s, a + dir * std::min(s + 8.0, len), c);
    }
}

/// A limb: rounded bar with a 2 px ink outline.
void limb(SDL_Renderer* r, ik::Vec2 a, ik::Vec2 b, float width, gd::Color fill, std::uint8_t alpha = 255) {
    const double len = ik::length(b - a);
    auto bar = [&](float w, gd::Color c) {
        c.a = alpha;
        if (len > 1e-6) {
            const double nx = -(b.y - a.y) / len * static_cast<double>(w) / 2.0;
            const double ny = (b.x - a.x) / len * static_cast<double>(w) / 2.0;
            const SDL_FColor f = gd::fcolor(c);
            const SDL_Vertex v[4] = {{fp({a.x + nx, a.y + ny}), f, {0, 0}}, {fp({b.x + nx, b.y + ny}), f, {0, 0}}, {fp({b.x - nx, b.y - ny}), f, {0, 0}}, {fp({a.x - nx, a.y - ny}), f, {0, 0}}};
            const int idx[6] = {0, 1, 2, 0, 2, 3};
            SDL_RenderGeometry(r, nullptr, v, 4, idx, 6);
        }
        gd::disc(r, static_cast<float>(a.x), static_cast<float>(a.y), w / 2.0F, c, 24);
        gd::disc(r, static_cast<float>(b.x), static_cast<float>(b.y), w / 2.0F, c, 24);
    };
    bar(width + 4.0F, gd::kInk);
    bar(width, fill);
}

void drawPose(App& app, const ik::Pose& p, gd::Color upper, gd::Color lower, std::uint8_t alpha) {
    SDL_Renderer* r = app.renderer;
    limb(r, kRoot, p.joint, 16.0F, upper, alpha);
    limb(r, p.joint, p.end, 12.0F, lower, alpha);
    gd::Color white = gd::kWhite;
    white.a = alpha;
    gd::disc(r, static_cast<float>(p.joint.x), static_cast<float>(p.joint.y), 7.0F, white, 24);
    gd::ring(r, static_cast<float>(p.joint.x), static_cast<float>(p.joint.y), 7.0F, gd::kInk);
}

void drawRing(App& app, double rMin, double rMax) {
    SDL_Renderer* r = app.renderer;
    const int top = static_cast<int>(std::floor(kRoot.y - rMax));
    const int bottom = static_cast<int>(std::ceil(kRoot.y + rMax));
    for (int y = top; y <= bottom; y += 2) {
        const ik::RowSpans rs = ik::ringSpans(kRoot, rMin, rMax, static_cast<double>(y) + 1.0);
        for (int i = 0; i < rs.n; ++i) {
            const ik::Span s = rs.s[static_cast<std::size_t>(i)];
            gd::fillRect(r, static_cast<float>(s.x0), static_cast<float>(y), static_cast<float>(s.x1 - s.x0), 2.0F, kRingFill);
        }
    }
    const auto cx = static_cast<float>(kRoot.x);
    const auto cy = static_cast<float>(kRoot.y);
    gd::ring(r, cx, cy, static_cast<float>(rMax), gd::kTeal);
    gd::ring(r, cx, cy, static_cast<float>(rMax) + 1.0F, gd::kTeal);
    if (rMin > 1.0) {
        gd::ring(r, cx, cy, static_cast<float>(rMin), gd::kTeal);
        gd::ring(r, cx, cy, static_cast<float>(rMin) - 1.0F, gd::kTeal);
        app.hud.text(r, cx + static_cast<float>(rMin) + 6.0F, cy + 8.0F, "|d1-d2|", gd::kTealDark);
    }
    app.hud.text(r, cx + static_cast<float>(rMax) + 8.0F, cy + 8.0F, "d1+d2", gd::kTealDark);
}

void panelRow(App& app, float y, const char* label, const char* value, gd::Color color) {
    constexpr float kX = 660.0F;
    constexpr float kW = 276.0F;
    app.hud.text(app.renderer, kX + 14.0F, y, label, gd::kMuted);
    app.hud.text(app.renderer, kX + kW - 14.0F - app.hud.width(value, gd::Hud::Style::Bold), y, value, color, gd::Hud::Style::Bold);
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    const Lengths len = lengths(app);
    const double rMin = ik::minReach(len.d1, len.d2);
    const double rMax = ik::maxReach(len.d1, len.d2);
    const ik::Pose pose = app.naive ? ik::solveNaive(kRoot, app.target, len.d1, len.d2, app.bend) : ik::solve(kRoot, app.target, len.d1, len.d2, app.bend);
    const bool valid = finitePose(pose);
    if (app.naive && valid) {
        app.lastValid = pose;
        app.haveLast = true;
    }
    const bool inRange = pose.reach == ik::Reach::InRange;
    const gd::Color verdict = inRange ? gd::kTeal : gd::kOrange;

    gd::backdrop(r, kWidth, kHeight);
    app.hud.titleBar(r, kWidth, "[ TWO-BONE IK ]", app.naive ? "naive: acos without range check" : "law of cosines, checked");

    drawRing(app, rMin, rMax);

    // The construction: circle d1 around the root, circle d2 around the target. They meet at the knee positions.
    gd::ring(r, static_cast<float>(kRoot.x), static_cast<float>(kRoot.y), static_cast<float>(len.d1), kMutedSoft);
    gd::ring(r, static_cast<float>(app.target.x), static_cast<float>(app.target.y), static_cast<float>(len.d2), kMutedSoft);
    dashedLine(r, kRoot, app.target, kMutedSoft);

    // The other solution, as an open ring.
    if (!app.naive && inRange && ik::solutionCount(pose.h, len.d1, len.d2) != 0) {
        const ik::Pose other = ik::solve(kRoot, app.target, len.d1, len.d2, app.bend == ik::Bend::Left ? ik::Bend::Right : ik::Bend::Left);
        if (ik::length(other.joint - pose.joint) > 1.0) {
            gd::ring(r, static_cast<float>(other.joint.x), static_cast<float>(other.joint.y), 11.0F, gd::kTealDark);
            gd::ring(r, static_cast<float>(other.joint.x), static_cast<float>(other.joint.y), 12.0F, gd::kTealDark);
        }
    }

    const gd::Color upper = app.naive ? gd::kOrange : gd::kTealDark;
    const gd::Color lower = app.naive ? gd::kOrange : gd::kTeal;
    if (valid) {
        drawPose(app, pose, upper, lower, 255);
        if (!inRange) dashedLine(r, pose.end, app.target, gd::kOrange);
    } else if (app.haveLast) {
        drawPose(app, app.lastValid, upper, lower, 70);
    }

    gd::disc(r, static_cast<float>(kRoot.x), static_cast<float>(kRoot.y), 11.0F, gd::kInk, 24);
    const auto tx = static_cast<float>(app.target.x);
    const auto ty = static_cast<float>(app.target.y);
    gd::disc(r, tx, ty, 5.0F, verdict, 20);
    gd::ring(r, tx, ty, 12.0F, verdict);
    gd::ring(r, tx, ty, 13.0F, gd::kInk);

    // Readout.
    gd::card(r, 660.0F, 44.0F, 276.0F, 252.0F, gd::kWhite, gd::kInk);  // same rectangle as inPanel()
    char buf[48];
    SDL_snprintf(buf, sizeof buf, "%.0f px", pose.h);
    panelRow(app, 56.0F, "h (root-target)", buf, gd::kInk);
    SDL_snprintf(buf, sizeof buf, "%.0f..%.0f", rMin, rMax);
    panelRow(app, 82.0F, "reach", buf, gd::kInk);
    if (valid) {
        SDL_snprintf(buf, sizeof buf, "%.0f°", pose.gamma * 180.0 / 3.14159265358979323846);
    } else {
        SDL_snprintf(buf, sizeof buf, "NaN");
    }
    panelRow(app, 108.0F, "knee angle", buf, valid ? gd::kInk : gd::kOrange);
    SDL_snprintf(buf, sizeof buf, "%.2f", pose.cosAlpha);
    const bool argOk = std::fabs(pose.cosAlpha) <= 1.0;
    panelRow(app, 134.0F, "acos argument", buf, argOk ? gd::kInk : gd::kOrange);
    const int solutions = ik::solutionCount(pose.h, len.d1, len.d2);
    SDL_snprintf(buf, sizeof buf, "%s", solutions == ik::kInfinite ? "inf" : (solutions == 2 ? "2" : (solutions == 1 ? "1" : "0")));
    panelRow(app, 160.0F, "solutions", buf, solutions == 0 ? gd::kOrange : gd::kTeal);
    const char* status = "IN RANGE";
    if (app.naive && !inRange) {
        status = "OUT OF RANGE: NaN";
    } else if (pose.reach == ik::Reach::TooFar) {
        status = "TOO FAR: stretched";
    } else if (pose.reach == ik::Reach::TooClose) {
        status = "TOO CLOSE: folded";
    }
    app.hud.text(r, 674.0F, 192.0F, status, verdict, gd::Hud::Style::Bold);
    if (app.naive && !valid) app.hud.text(r, 674.0F, 216.0F, "limb is lost", gd::kOrange);
    app.hud.text(r, 674.0F, 244.0F, "h^2 = d1^2 + d2^2", gd::kMuted);
    app.hud.text(r, 674.0F, 266.0F, "    - 2*d1*d2*cos(knee)", gd::kMuted);

    app.hud.text(r, 40.0F, 446.0F, "mouse = target   teal ring = reachable   circles meet at the two knee spots", gd::kMuted);

    for (const Chip& c : app.chips) {
        const bool on = active(app, c.action);
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? gd::kTeal : gd::kWhite, gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 6.0F, c.label, on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
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
    if (!SDL_CreateWindowAndRenderer("two-bone-ik", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
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
    const Lengths len = lengths(*app);
    app->target = ik::autoTarget(kRoot, len.d1, len.d2, app->phase);
    app->lastTicks = SDL_GetTicksNS();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION: {
            const ik::Vec2 m{static_cast<double>(event->motion.x), static_cast<double>(event->motion.y)};
            if (inField(m) && !inPanel(m)) {  // the readout is not part of the field, the target never hides under it
                app.target = m;
                app.autoPath = false;  // the mouse takes over from the demo path
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const SDL_FPoint p{event->button.x, event->button.y};
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) apply(app, c.action);
            }
            break;
        }
        case SDL_EVENT_KEY_DOWN:
            if (event->key.repeat) break;  // every key here toggles, none should fire while held
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: apply(app, Action::BendLeft); break;
                case SDLK_2: apply(app, Action::BendRight); break;
                case SDLK_3: apply(app, Action::Checked); break;
                case SDLK_4: apply(app, Action::Naive); break;
                case SDLK_5: apply(app, Action::Equal); break;
                case SDLK_A: apply(app, Action::Auto); break;
                default: break;
            }
            break;
        default: break;
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void* appstate) {
    auto& app = *static_cast<App*>(appstate);
    const Uint64 now = SDL_GetTicksNS();
    const double dt = std::min(kMaxDt, static_cast<double>(now - app.lastTicks) / 1.0e9);
    app.lastTicks = now;
    if (app.autoPath) {
        const Lengths len = lengths(app);
        app.phase = ik::advancePhase(app.phase, dt);
        app.target = ik::autoTarget(kRoot, len.d1, len.d2, app.phase);
    }
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
