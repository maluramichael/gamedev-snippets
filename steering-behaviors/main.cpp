// Steering behaviors with SDL3 main callbacks.
//
// An agent chases, flees from, arrives at, wanders around or follows a path. Whatever it does, the rule underneath is
// the same: steering = desired velocity - current velocity, capped by a force limit. The green arrow is the velocity,
// the dark one the velocity the behavior wants, the orange one the difference between them. Seek keeps wanting full
// speed, so it cannot stop and circles the target; arrive asks for less speed the closer it gets.
//
// Two agents run the same behavior from the same state, A at 120 steps per second and B at 30. With "dt scaling" on
// (the force limit is a change of velocity per SECOND, a step adds limit * dt) they stay together. Switch it off and the
// limit becomes "per step": B crawls, A races ahead, and the gap number shows how far apart they are. The maths are in
// logic.hpp and covered by test.cpp.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "hud.hpp"
#include "logic.hpp"

namespace {

constexpr int kWidth = 960;
constexpr int kHeight = 540;
constexpr double kHzA = 120.0;
constexpr double kHzB = 30.0;
constexpr int kMaxStepsPerFrame = 40;
constexpr double kMaxFrameTime = 0.25;
constexpr double kTrailEvery = 0.02;  // seconds between trail points
constexpr std::size_t kTrailLength = 200;
constexpr double kVectorScale = 0.42;  // px per (px/s) when drawing velocities
constexpr float kChipY = 476.0F;
constexpr float kChipH = 34.0F;
constexpr gd::Color kInkSoft{17, 24, 39, 150};
constexpr gd::Color kTealBand{13, 148, 136, 38};
constexpr gd::Color kOrangeSoft{234, 88, 12, 70};
constexpr gd::Color kTealSoftLine{13, 148, 136, 120};

enum class Action { Seek, Flee, Arrive, Wander, FollowPath, ScaleDt, LimitForce };

struct Chip {
    Action action;
    std::string label;
    SDL_FRect rect;
};

struct Trail {
    std::vector<sb::Vec2> points;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;
    sb::Params params;
    sb::Path path;
    sb::Behavior behavior = sb::Behavior::Seek;
    bool scaleByDt = true;
    bool limitForce = true;
    sb::Agent a;  // fast: kHzA steps per second
    sb::Agent b;  // slow: kHzB steps per second
    double accA = 0.0;
    double accB = 0.0;
    double trailTimer = 0.0;
    Trail trailA;
    Trail trailB;
    sb::Vec2 target{650.0, 240.0};
    Uint64 lastTicks = 0;
    long maxFrames = -1;
    long frames = 0;
};

sb::Agent homeAgent() {
    sb::Agent agent;
    agent.pos = {200.0, 330.0};
    agent.vel = {0.0, -150.0};
    agent.heading = {0.0, -1.0};
    return agent;
}

sb::Path homePath() {
    return {{150, 150}, {430, 108}, {770, 160}, {840, 320}, {540, 395}, {240, 350}};
}

SDL_FPoint toPoint(sb::Vec2 v) { return {static_cast<float>(v.x), static_cast<float>(v.y)}; }

const char* behaviorName(sb::Behavior b) {
    switch (b) {
        case sb::Behavior::Seek: return "SEEK";
        case sb::Behavior::Flee: return "FLEE";
        case sb::Behavior::Arrive: return "ARRIVE";
        case sb::Behavior::Wander: return "WANDER";
        case sb::Behavior::Path: return "PATH";
    }
    return "";
}

bool usesTarget(sb::Behavior b) { return b == sb::Behavior::Seek || b == sb::Behavior::Flee || b == sb::Behavior::Arrive; }

/// B takes over A's state, both step clocks start at zero. Used whenever something changes, so the gap starts at 0.
void syncGhost(App& app) {
    app.b = app.a;
    app.accA = 0.0;
    app.accB = 0.0;
    app.trailTimer = 0.0;
    app.trailB = app.trailA;
}

void reset(App& app) {
    app.a = homeAgent();
    app.target = {650.0, 240.0};
    app.trailA.points.clear();
    syncGhost(app);
}

void pushTrail(Trail& trail, sb::Vec2 p) {
    trail.points.push_back(p);
    if (trail.points.size() > kTrailLength) trail.points.erase(trail.points.begin());
}

/// Runs both agents for `dt` real seconds, each in steps of its own length.
void advanceWorld(App& app, double dt) {
    dt = std::clamp(dt, 0.0, kMaxFrameTime);
    app.accA += dt;
    app.accB += dt;
    int steps = 0;
    while (app.accA >= 1.0 / kHzA && steps++ < kMaxStepsPerFrame) {
        sb::advance(app.a, app.behavior, app.target, app.params, app.path, kHzA, app.scaleByDt, app.limitForce);
        app.accA -= 1.0 / kHzA;
    }
    steps = 0;
    while (app.accB >= 1.0 / kHzB && steps++ < kMaxStepsPerFrame) {
        sb::advance(app.b, app.behavior, app.target, app.params, app.path, kHzB, app.scaleByDt, app.limitForce);
        app.accB -= 1.0 / kHzB;
    }
    app.trailTimer += dt;
    while (app.trailTimer >= kTrailEvery) {
        pushTrail(app.trailA, app.a.pos);
        pushTrail(app.trailB, app.b.pos);
        app.trailTimer -= kTrailEvery;
    }
}

void apply(App& app, Action action) {
    switch (action) {
        case Action::Seek: app.behavior = sb::Behavior::Seek; break;
        case Action::Flee: app.behavior = sb::Behavior::Flee; break;
        case Action::Arrive: app.behavior = sb::Behavior::Arrive; break;
        case Action::Wander: app.behavior = sb::Behavior::Wander; break;
        case Action::FollowPath: app.behavior = sb::Behavior::Path; break;
        case Action::ScaleDt: app.scaleByDt = !app.scaleByDt; break;
        case Action::LimitForce: app.limitForce = !app.limitForce; break;
    }
    syncGhost(app);
}

bool active(const App& app, Action action) {
    switch (action) {
        case Action::Seek: return app.behavior == sb::Behavior::Seek;
        case Action::Flee: return app.behavior == sb::Behavior::Flee;
        case Action::Arrive: return app.behavior == sb::Behavior::Arrive;
        case Action::Wander: return app.behavior == sb::Behavior::Wander;
        case Action::FollowPath: return app.behavior == sb::Behavior::Path;
        case Action::ScaleDt: return app.scaleByDt;
        case Action::LimitForce: return app.limitForce;
    }
    return false;
}

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
        float gapBefore;
    };
    const std::array<Def, 7> defs{{{Action::Seek, "1 Seek", 0.0F},
                                   {Action::Flee, "2 Flee", 8.0F},
                                   {Action::Arrive, "3 Arrive", 8.0F},
                                   {Action::Wander, "4 Wander", 8.0F},
                                   {Action::FollowPath, "5 Path", 8.0F},
                                   {Action::ScaleDt, "D dt scaling", 28.0F},
                                   {Action::LimitForce, "F force limit", 8.0F}}};
    float x = 40.0F;
    for (const Def& d : defs) {
        x += d.gapBefore;
        const float w = app.hud.width(d.label, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({d.action, d.label, SDL_FRect{x, kChipY, w, kChipH}});
        x += w;
    }
}

bool inField(const App& app, float y) { return static_cast<double>(y) >= app.params.top && static_cast<double>(y) <= app.params.bottom; }

// --- drawing helpers -----------------------------------------------------------------------------------------------

/// 1 px lines are too thin on a 960x540 canvas that is scaled up, so draw a few side by side.
void thickLine(SDL_Renderer* r, sb::Vec2 p, sb::Vec2 q, gd::Color c, int half = 1) {
    gd::setColor(r, c);
    const bool flat = std::fabs(q.x - p.x) > std::fabs(q.y - p.y);
    for (int o = -half; o <= half; ++o) {
        const auto off = static_cast<float>(o);
        SDL_RenderLine(r, static_cast<float>(p.x) + (flat ? 0.0F : off), static_cast<float>(p.y) + (flat ? off : 0.0F),
                       static_cast<float>(q.x) + (flat ? 0.0F : off), static_cast<float>(q.y) + (flat ? off : 0.0F));
    }
}

void dashedLine(SDL_Renderer* r, sb::Vec2 p, sb::Vec2 q, gd::Color c, double dash = 7.0, double gap = 5.0) {
    const double len = sb::length(q - p);
    if (len < 1e-6) return;
    const sb::Vec2 dir = (q - p) * (1.0 / len);
    for (double t = 0.0; t < len; t += dash + gap) thickLine(r, p + dir * t, p + dir * std::min(t + dash, len), c, 0);
}

void arrow(SDL_Renderer* r, sb::Vec2 from, sb::Vec2 to, gd::Color c, int half = 1) {
    const double len = sb::length(to - from);
    if (len < 2.0) return;
    thickLine(r, from, to, c, half);
    const sb::Vec2 dir = (to - from) * (1.0 / len);
    const sb::Vec2 side{-dir.y, dir.x};
    const double head = std::min(9.0, len * 0.6);
    thickLine(r, to, to - dir * head + side * (head * 0.5), c, 0);
    thickLine(r, to, to - dir * head - side * (head * 0.5), c, 0);
}

/// Dashed circle, every other segment of the ring.
void dashedRing(SDL_Renderer* r, sb::Vec2 center, double radius, gd::Color c) {
    gd::setColor(r, c);
    constexpr int kSegments = 72;
    for (int i = 0; i < kSegments; i += 2) {
        const double a0 = 2.0 * sb::kPi * i / kSegments;
        const double a1 = 2.0 * sb::kPi * (i + 1) / kSegments;
        SDL_RenderLine(r, static_cast<float>(center.x + std::cos(a0) * radius), static_cast<float>(center.y + std::sin(a0) * radius),
                       static_cast<float>(center.x + std::cos(a1) * radius), static_cast<float>(center.y + std::sin(a1) * radius));
    }
}

void drawTrail(SDL_Renderer* r, const Trail& trail, gd::Color base) {
    const std::size_t n = trail.points.size();
    for (std::size_t i = 1; i < n; ++i) {
        if (sb::distance(trail.points[i - 1], trail.points[i]) > 120.0) continue;  // the agent wrapped around the field
        gd::Color c = base;
        c.a = static_cast<Uint8>(30.0 + 150.0 * static_cast<double>(i) / static_cast<double>(n));
        thickLine(r, trail.points[i - 1], trail.points[i], c, 1);
    }
}

void drawAgent(SDL_Renderer* r, const sb::Agent& agent, gd::Color fill) {
    const sb::Vec2 h = agent.heading;
    const sb::Vec2 side{-h.y, h.x};
    const sb::Vec2 tip = agent.pos + h * 13.0;
    const sb::Vec2 left = agent.pos - h * 9.0 + side * 8.0;
    const sb::Vec2 right = agent.pos - h * 9.0 - side * 8.0;
    const SDL_FColor col = gd::fcolor(fill);
    const SDL_Vertex v[3] = {{toPoint(tip), col, {0, 0}}, {toPoint(left), col, {0, 0}}, {toPoint(right), col, {0, 0}}};
    SDL_RenderGeometry(r, nullptr, v, 3, nullptr, 0);
    thickLine(r, tip, left, gd::kInk, 0);
    thickLine(r, left, right, gd::kInk, 0);
    thickLine(r, right, tip, gd::kInk, 0);
}

void drawPath(App& app) {
    SDL_Renderer* r = app.renderer;
    const std::size_t n = app.path.size();
    const double radius = app.params.path.radius;
    for (std::size_t i = 0; i < n; ++i) {
        const sb::Vec2 p = app.path[i];
        const sb::Vec2 q = app.path[(i + 1) % n];
        const sb::Vec2 dir = sb::normalize(q - p);
        const sb::Vec2 side{-dir.y * radius, dir.x * radius};
        const SDL_FColor col = gd::fcolor(kTealBand);
        const SDL_Vertex v[4] = {{toPoint(p + side), col, {0, 0}}, {toPoint(q + side), col, {0, 0}}, {toPoint(q - side), col, {0, 0}}, {toPoint(p - side), col, {0, 0}}};
        const int idx[6] = {0, 1, 2, 0, 2, 3};
        SDL_RenderGeometry(r, nullptr, v, 4, idx, 6);
        dashedLine(r, p, q, kTealSoftLine, 10.0, 6.0);
    }
    for (const sb::Vec2& p : app.path) {
        gd::disc(r, static_cast<float>(p.x), static_cast<float>(p.y), 4.0F, gd::kTealDark);
    }
}

void drawOverlays(App& app) {
    SDL_Renderer* r = app.renderer;
    const auto tx = static_cast<float>(app.target.x);
    const auto ty = static_cast<float>(app.target.y);
    switch (app.behavior) {
        case sb::Behavior::Arrive: dashedRing(r, app.target, app.params.arriveRadius, gd::kTeal); break;
        case sb::Behavior::Flee: dashedRing(r, app.target, app.params.panicRadius, gd::kOrange); break;
        case sb::Behavior::Wander: {
            const sb::Vec2 center = app.a.pos + sb::normalize(app.a.heading) * app.params.wanderDistance;
            dashedRing(r, center, app.params.wanderRadius, gd::kTeal);
            const sb::Vec2 point = sb::wanderTarget(app.a.pos, app.a.heading, sb::wanderNoise(app.a.time) * sb::kPi, app.params.wanderDistance, app.params.wanderRadius);
            dashedLine(r, app.a.pos, center, kInkSoft, 4.0, 4.0);
            gd::disc(r, static_cast<float>(point.x), static_cast<float>(point.y), 5.0F, gd::kOrange);
            gd::ring(r, static_cast<float>(point.x), static_cast<float>(point.y), 5.0F, gd::kInk);
            break;
        }
        case sb::Behavior::Path: drawPath(app); break;
        case sb::Behavior::Seek: break;
    }
    if (usesTarget(app.behavior)) {
        gd::disc(r, tx, ty, 8.0F, gd::kOrange);
        gd::ring(r, tx, ty, 8.0F, gd::kInk);
        gd::ring(r, tx, ty, 9.0F, gd::kInk);
        thickLine(r, {app.target.x - 15.0, app.target.y}, {app.target.x + 15.0, app.target.y}, gd::kInk, 0);
        thickLine(r, {app.target.x, app.target.y - 15.0}, {app.target.x, app.target.y + 15.0}, gd::kInk, 0);
    }
}

void drawLegendSwatch(App& app, float x, float y, gd::Color c, const char* label) {
    gd::fillRect(app.renderer, x, y + 5.0F, 18.0F, 6.0F, c);
    app.hud.text(app.renderer, x + 26.0F, y, label, gd::kInk);
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    app.hud.titleBar(r, kWidth, "[ STEERING BEHAVIORS ]", app.scaleByDt ? "force * dt" : "force per frame");

    // Readout.
    gd::card(r, 24.0F, 44.0F, 340.0F, 112.0F, gd::kWhite, gd::kInk);
    app.hud.text(r, 38.0F, 52.0F, behaviorName(app.behavior), gd::kTeal, gd::Hud::Style::Large);
    char line[96];
    SDL_snprintf(line, sizeof line, "A %.0f steps/s", kHzA);
    app.hud.text(r, 38.0F, 98.0F, line, gd::kTealDark, gd::Hud::Style::Bold);
    SDL_snprintf(line, sizeof line, "B %.0f steps/s", kHzB);
    app.hud.text(r, 190.0F, 98.0F, line, gd::kOrange, gd::Hud::Style::Bold);
    if (app.scaleByDt) {
        SDL_snprintf(line, sizeof line, "gap A-B %.0f px", sb::distance(app.a.pos, app.b.pos));
    } else {
        SDL_snprintf(line, sizeof line, "gap %.0f px  speed x%.1f/x%.1f", sb::distance(app.a.pos, app.b.pos), kHzA / sb::kReferenceHz, kHzB / sb::kReferenceHz);
    }
    app.hud.text(r, 38.0F, 124.0F, line, gd::kInk, gd::Hud::Style::Bold);

    // Everything that moves stays inside the field and passes over the readout, so the agent is never hidden behind it.
    const SDL_Rect field{0, static_cast<int>(app.params.top), kWidth, static_cast<int>(app.params.bottom - app.params.top)};
    SDL_SetRenderClipRect(r, &field);
    drawOverlays(app);
    drawTrail(r, app.trailB, gd::kOrange);
    drawTrail(r, app.trailA, gd::kTeal);

    // The three vectors of agent A: where it goes, where it wants to go, and the difference that steers it.
    const sb::Vec2 desired = sb::desiredVelocity(app.behavior, app.a, app.target, app.params, app.path);
    const sb::Vec2 velTip = app.a.pos + app.a.vel * kVectorScale;
    const sb::Vec2 wishTip = app.a.pos + desired * kVectorScale;
    dashedLine(r, app.a.pos, wishTip, gd::kInk);
    arrow(r, app.a.pos, velTip, gd::kTealDark, 1);
    arrow(r, velTip, wishTip, gd::kOrange, 1);

    drawAgent(r, app.b, gd::kOrange);
    drawAgent(r, app.a, gd::kTeal);
    SDL_SetRenderClipRect(r, nullptr);

    // Legend and hint.
    drawLegendSwatch(app, 40.0F, 444.0F, gd::kTealDark, "velocity");
    drawLegendSwatch(app, 170.0F, 444.0F, gd::kInk, "desired");
    drawLegendSwatch(app, 290.0F, 444.0F, gd::kOrange, "steering = desired - velocity");
    app.hud.text(r, 620.0F, 444.0F, usesTarget(app.behavior) ? "mouse / click: target   R: reset" : "R: reset", gd::kMuted);

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
    if (!SDL_CreateWindowAndRenderer("steering-behaviors", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, SDL_LOGICAL_PRESENTATION_LETTERBOX);
    SDL_SetRenderVSync(app->renderer, 1);
    layoutChips(*app);
    app->path = homePath();
    reset(*app);
    // Run a moment ahead so the first frame already shows the swing towards the target and its trail.
    for (int i = 0; i < 70; ++i) advanceWorld(*app, 1.0 / 60.0);
    syncGhost(*app);
    app->lastTicks = SDL_GetTicksNS();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION:
            if (inField(app, event->motion.y)) app.target = {static_cast<double>(event->motion.x), static_cast<double>(event->motion.y)};
            break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const SDL_FPoint p{event->button.x, event->button.y};
            bool onChip = false;
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) {
                    apply(app, c.action);
                    onChip = true;
                }
            }
            if (!onChip && inField(app, p.y)) app.target = {static_cast<double>(p.x), static_cast<double>(p.y)};
            break;
        }
        case SDL_EVENT_KEY_DOWN:
            if (event->key.repeat) break;
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: apply(app, Action::Seek); break;
                case SDLK_2: apply(app, Action::Flee); break;
                case SDLK_3: apply(app, Action::Arrive); break;
                case SDLK_4: apply(app, Action::Wander); break;
                case SDLK_5: apply(app, Action::FollowPath); break;
                case SDLK_D: apply(app, Action::ScaleDt); break;
                case SDLK_F: apply(app, Action::LimitForce); break;
                case SDLK_R: reset(app); break;
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
    advanceWorld(app, static_cast<double>(now - app.lastTicks) * 1e-9);
    app.lastTicks = now;
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
