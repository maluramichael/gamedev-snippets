// Spider foot placement with SDL3 main callbacks.
//
// The body follows the mouse, the six feet do not: each one stays where it is in the world until the point it should
// stand on (rest point plus a bit of velocity) has moved further than the step threshold away, then it swings there.
// The gait is only a rule about who may start a swing. Tripod: two groups of three, one group in the air at a time.
// Crawl: one leg at a time. Free: whoever wants to, which makes all six lift at the same moment. The polygon of the
// planted feet is drawn against the center of the body: outside of it the body has nothing to stand on. Until the mouse
// moves, the body walks a figure eight, so the first frame already shows legs in the air. The logic is in logic.hpp and
// covered by test.cpp.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <string>
#include <vector>

#include "hud.hpp"
#include "logic.hpp"

namespace {

using spider::Vec2;

constexpr int kWidth = 960;
constexpr int kHeight = 540;
constexpr float kFieldTop = 34.0F;
constexpr float kFieldBottom = 440.0F;
constexpr float kChipY = 476.0F;
constexpr float kChipH = 34.0F;
constexpr float kPanelX = 676.0F;
constexpr float kPanelY = 44.0F;
constexpr float kPanelW = 260.0F;
constexpr float kPanelH = 226.0F;
constexpr double kStep = 1.0 / 120.0;  // fixed physics step
constexpr double kMaxFrameDt = 0.25;
constexpr int kMaxStepsPerFrame = 8;
constexpr double kWarmUp = 1.6;        // seconds simulated before the first frame, so legs are already in the air
constexpr double kFootprintLife = 4.0;
constexpr std::size_t kMaxFootprints = 96;
constexpr gd::Color kMutedSoft{107, 114, 128, 130};
constexpr gd::Color kRingSoft{107, 114, 128, 70};
constexpr gd::Color kSupportOk{13, 148, 136, 54};
constexpr gd::Color kSupportBad{234, 88, 12, 78};

enum class Action { Tripod, Crawl, Free, Lookahead, Support, Auto };

struct Chip {
    Action action;
    std::string label;
    SDL_FRect rect;
};

struct Footprint {
    Vec2 pos;
    double age;
    int group;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;
    spider::Walker walker = spider::makeWalker({0.0, 0.0}, 0.0, spider::Gait::Tripod, true);
    spider::Stats stats;
    std::deque<Footprint> prints;
    bool autoPath = true;
    bool showSupport = true;
    double phase = 0.0;
    Vec2 goal = {0.0, 0.0};
    double accumulator = 0.0;
    Uint64 lastTicks = 0;
    long maxFrames = -1;
    long frames = 0;
};

bool inField(Vec2 p) { return p.y >= static_cast<double>(kFieldTop) && p.y <= static_cast<double>(kFieldBottom); }

bool inPanel(Vec2 p) {
    return p.x >= static_cast<double>(kPanelX) && p.x <= static_cast<double>(kPanelX + kPanelW) && p.y >= static_cast<double>(kPanelY) && p.y <= static_cast<double>(kPanelY + kPanelH);
}

SDL_FPoint fp(Vec2 v) { return {static_cast<float>(v.x), static_cast<float>(v.y)}; }

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
        float gapBefore;
    };
    const std::array<Def, 6> defs{{{Action::Tripod, "1 Tripod", 0.0F},
                                   {Action::Crawl, "2 Crawl", 8.0F},
                                   {Action::Free, "3 Free", 8.0F},
                                   {Action::Lookahead, "L Lookahead", 24.0F},
                                   {Action::Support, "S Support", 8.0F},
                                   {Action::Auto, "A Auto path", 24.0F}}};
    float x = 40.0F;
    for (const Def& d : defs) {
        x += d.gapBefore;
        const float w = app.hud.width(d.label, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({d.action, d.label, SDL_FRect{x, kChipY, w, kChipH}});
        x += w;
    }
}

void resetStats(App& app) {
    app.stats = spider::Stats{};
    app.accumulator = 0.0;  // no leftover time from before the switch
}

void apply(App& app, Action action) {
    switch (action) {
        case Action::Tripod:
        case Action::Crawl:
        case Action::Free: {
            const spider::Gait g = action == Action::Tripod ? spider::Gait::Tripod : (action == Action::Crawl ? spider::Gait::Crawl : spider::Gait::Free);
            if (g != app.walker.gait) {
                app.walker.gait = g;
                resetStats(app);
            }
            break;
        }
        case Action::Lookahead:
            app.walker.lookahead = !app.walker.lookahead;
            resetStats(app);
            break;
        case Action::Support: app.showSupport = !app.showSupport; break;
        case Action::Auto:
            app.autoPath = !app.autoPath;
            if (app.autoPath) {
                app.phase = 0.0;  // restart the path from its first point, the body walks there
                resetStats(app);
            }
            break;
    }
}

bool active(const App& app, Action action) {
    switch (action) {
        case Action::Tripod: return app.walker.gait == spider::Gait::Tripod;
        case Action::Crawl: return app.walker.gait == spider::Gait::Crawl;
        case Action::Free: return app.walker.gait == spider::Gait::Free;
        case Action::Lookahead: return app.walker.lookahead;
        case Action::Support: return app.showSupport;
        case Action::Auto: return app.autoPath;
    }
    return false;
}

/// One physics step: the goal moves along the demo path if it is on, the walker steps, landings leave footprints.
void stepWorld(App& app) {
    if (app.autoPath) {
        app.phase = spider::advancePhase(app.phase, kStep);
        app.goal = spider::autoGoal(app.phase);
    }
    std::array<spider::Phase, spider::kLegs> before{};
    for (std::size_t i = 0; i < before.size(); ++i) before[i] = app.walker.legs[i].phase;
    const int started = spider::update(app.walker, app.goal, kStep);
    spider::record(app.stats, app.walker, started, kStep);
    for (std::size_t i = 0; i < before.size(); ++i) {
        const spider::Leg& leg = app.walker.legs[i];
        if (before[i] == spider::Phase::Swinging && leg.phase == spider::Phase::Planted) {
            app.prints.push_back({leg.foot, 0.0, spider::legDefs()[i].group});
            if (app.prints.size() > kMaxFootprints) app.prints.pop_front();
        }
    }
    for (Footprint& f : app.prints) f.age += kStep;
    while (!app.prints.empty() && app.prints.front().age > kFootprintLife) app.prints.pop_front();
}

void thickLine(SDL_Renderer* r, Vec2 a, Vec2 b, gd::Color c, int half = 0) {
    gd::setColor(r, c);
    const bool flat = std::fabs(b.x - a.x) > std::fabs(b.y - a.y);
    for (int o = -half; o <= half; ++o) {
        const auto off = static_cast<float>(o);
        SDL_RenderLine(r, static_cast<float>(a.x) + (flat ? 0.0F : off), static_cast<float>(a.y) + (flat ? off : 0.0F), static_cast<float>(b.x) + (flat ? 0.0F : off),
                       static_cast<float>(b.y) + (flat ? off : 0.0F));
    }
}

void dashedLine(SDL_Renderer* r, Vec2 a, Vec2 b, gd::Color c) {
    const double len = spider::length(b - a);
    if (len < 1.0) return;
    const Vec2 dir = (b - a) * (1.0 / len);
    for (double s = 0.0; s < len; s += 8.0) thickLine(r, a + dir * s, a + dir * std::min(s + 4.0, len), c);
}

/// A limb: rounded bar with a 2 px ink outline.
void limb(SDL_Renderer* r, Vec2 a, Vec2 b, float width, gd::Color fill) {
    const double len = spider::length(b - a);
    auto bar = [&](float w, gd::Color c) {
        if (len > 1e-6) {
            const double nx = -(b.y - a.y) / len * static_cast<double>(w) / 2.0;
            const double ny = (b.x - a.x) / len * static_cast<double>(w) / 2.0;
            const SDL_FColor f = gd::fcolor(c);
            const SDL_Vertex v[4] = {{fp({a.x + nx, a.y + ny}), f, {0, 0}}, {fp({b.x + nx, b.y + ny}), f, {0, 0}}, {fp({b.x - nx, b.y - ny}), f, {0, 0}}, {fp({a.x - nx, a.y - ny}), f, {0, 0}}};
            const int idx[6] = {0, 1, 2, 0, 2, 3};
            SDL_RenderGeometry(r, nullptr, v, 4, idx, 6);
        }
        gd::disc(r, static_cast<float>(a.x), static_cast<float>(a.y), w / 2.0F, c, 16);
        gd::disc(r, static_cast<float>(b.x), static_cast<float>(b.y), w / 2.0F, c, 16);
    };
    bar(width + 3.0F, gd::kInk);
    bar(width, fill);
}

void fillPolygon(SDL_Renderer* r, const std::vector<Vec2>& poly, gd::Color c) {
    if (poly.size() < 3) return;
    const SDL_FColor f = gd::fcolor(c);
    std::vector<SDL_Vertex> v;
    v.reserve(poly.size());
    for (const Vec2& p : poly) v.push_back({fp(p), f, {0, 0}});
    std::vector<int> idx;
    for (std::size_t i = 1; i + 1 < poly.size(); ++i) {
        idx.push_back(0);
        idx.push_back(static_cast<int>(i));
        idx.push_back(static_cast<int>(i + 1));
    }
    SDL_RenderGeometry(r, nullptr, v.data(), static_cast<int>(v.size()), idx.data(), static_cast<int>(idx.size()));
}

void drawFootprints(App& app) {
    SDL_Renderer* r = app.renderer;
    for (const Footprint& f : app.prints) {
        const double life = 1.0 - f.age / kFootprintLife;
        gd::Color c = f.group == 0 ? gd::kTeal : gd::kTealDark;
        c.a = static_cast<std::uint8_t>(std::clamp(life * 140.0, 0.0, 140.0));
        gd::ring(r, static_cast<float>(f.pos.x), static_cast<float>(f.pos.y), 4.0F, c, 16);
    }
}

void drawSupport(App& app, bool ok) {
    SDL_Renderer* r = app.renderer;
    const std::vector<Vec2> hull = spider::convexHull(spider::plantedFeet(app.walker));
    const gd::Color edge = ok ? gd::kTeal : gd::kOrange;
    fillPolygon(r, hull, ok ? kSupportOk : kSupportBad);
    if (hull.size() >= 2) {
        for (std::size_t i = 0; i < hull.size(); ++i) {
            const Vec2 a = hull[i];
            const Vec2 b = hull[(i + 1) % hull.size()];
            if (hull.size() == 2 && i == 1) break;
            thickLine(r, a, b, edge);
        }
    }
}

void drawTargets(App& app) {
    SDL_Renderer* r = app.renderer;
    const spider::Walker& w = app.walker;
    const double threshold = spider::activeThreshold(w);
    for (int i = 0; i < spider::kLegs; ++i) {
        const spider::Leg& leg = w.legs[static_cast<std::size_t>(i)];
        const Vec2 target = spider::stepTarget(w, i);
        const bool wants = leg.phase == spider::Phase::Planted && spider::needsStep(leg.foot, target, threshold);
        const gd::Color c = wants ? gd::kOrange : kMutedSoft;
        gd::ring(r, static_cast<float>(target.x), static_cast<float>(target.y), static_cast<float>(threshold), kRingSoft, 48);
        dashedLine(r, leg.foot, target, c);
        const auto tx = static_cast<float>(target.x);
        const auto ty = static_cast<float>(target.y);
        gd::setColor(r, c);
        SDL_RenderLine(r, tx - 5.0F, ty, tx + 5.0F, ty);
        SDL_RenderLine(r, tx, ty - 5.0F, tx, ty + 5.0F);
    }
}

void drawLegs(App& app) {
    SDL_Renderer* r = app.renderer;
    const spider::Walker& w = app.walker;
    // Shadows first, so no leg is covered by the shadow of another.
    for (int i = 0; i < spider::kLegs; ++i) {
        const spider::Leg& leg = w.legs[static_cast<std::size_t>(i)];
        if (leg.phase == spider::Phase::Swinging) gd::disc(r, static_cast<float>(leg.foot.x), static_cast<float>(leg.foot.y), 5.0F, kMutedSoft, 16);
    }
    for (int i = 0; i < spider::kLegs; ++i) {
        const auto idx = static_cast<std::size_t>(i);
        const spider::Leg& leg = w.legs[idx];
        const bool air = leg.phase == spider::Phase::Swinging;
        const double lift = spider::legLift(w, i);
        const Vec2 apparent = leg.foot + Vec2{0.0, -0.8 * lift};  // up the screen = towards the viewer
        const Vec2 outward = spider::rotate({0.0, spider::legDefs()[idx].side}, w.heading);
        const spider::LegPose pose = spider::solveLeg(spider::hipWorld(w, i), apparent, w.p.d1, w.p.d2, outward);
        limb(r, spider::hipWorld(w, i), pose.knee, 11.0F, air ? gd::kOrange : gd::kTealDark);
        limb(r, pose.knee, pose.end, 8.0F, air ? gd::kOrange : gd::kTeal);
        const float radius = 5.0F + static_cast<float>(lift) * 0.3F;
        gd::disc(r, static_cast<float>(pose.end.x), static_cast<float>(pose.end.y), radius + 1.5F, gd::kInk, 16);
        gd::disc(r, static_cast<float>(pose.end.x), static_cast<float>(pose.end.y), radius, air ? gd::kOrange : gd::kWhite, 16);
    }
}

void drawBody(App& app, bool ok) {
    SDL_Renderer* r = app.renderer;
    const spider::Walker& w = app.walker;
    const auto at = [&](double x, double y) { return spider::toWorld(w.pos, w.heading, {x, y}); };
    const Vec2 abdomen = at(-22.0, 0.0);
    const Vec2 head = at(16.0, 0.0);
    gd::disc(r, static_cast<float>(abdomen.x), static_cast<float>(abdomen.y), 27.0F, gd::kInk, 32);
    gd::disc(r, static_cast<float>(abdomen.x), static_cast<float>(abdomen.y), 24.0F, gd::kTealDark, 32);
    gd::disc(r, static_cast<float>(head.x), static_cast<float>(head.y), 20.0F, gd::kInk, 28);
    gd::disc(r, static_cast<float>(head.x), static_cast<float>(head.y), 17.0F, gd::kTeal, 28);
    for (const double side : {-1.0, 1.0}) {
        const Vec2 eye = at(25.0, side * 7.0);
        gd::disc(r, static_cast<float>(eye.x), static_cast<float>(eye.y), 4.5F, gd::kWhite, 14);
        const Vec2 pupil = at(26.8, side * 7.0);
        gd::disc(r, static_cast<float>(pupil.x), static_cast<float>(pupil.y), 2.0F, gd::kInk, 10);
    }
    // The center of mass, the point that must stay inside the support polygon.
    const auto cx = static_cast<float>(w.pos.x);
    const auto cy = static_cast<float>(w.pos.y);
    gd::disc(r, cx, cy, 5.0F, ok ? gd::kTeal : gd::kOrange, 16);
    gd::ring(r, cx, cy, 6.0F, gd::kInk, 16);
    gd::ring(r, cx, cy, 7.0F, gd::kWhite, 16);
}

void panelRow(App& app, float y, const char* label, const char* value, gd::Color color) {
    app.hud.text(app.renderer, kPanelX + 14.0F, y, label, gd::kMuted);
    app.hud.text(app.renderer, kPanelX + kPanelW - 14.0F - app.hud.width(value, gd::Hud::Style::Bold), y, value, color, gd::Hud::Style::Bold);
}

void drawPanel(App& app, bool ok) {
    SDL_Renderer* r = app.renderer;
    const spider::Walker& w = app.walker;
    gd::card(r, kPanelX, kPanelY, kPanelW, kPanelH, gd::kWhite, gd::kInk);  // same rectangle as inPanel()
    char buf[48];
    const double speed = spider::length(w.vel);
    SDL_snprintf(buf, sizeof buf, "%.0f px/s", speed);
    panelRow(app, 56.0F, "speed", buf, gd::kInk);
    const int planted = spider::plantedCount(w);
    SDL_snprintf(buf, sizeof buf, "%d / %d", planted, spider::kLegs);
    panelRow(app, 82.0F, "feet down", buf, planted >= 3 ? gd::kInk : gd::kOrange);
    SDL_snprintf(buf, sizeof buf, "%d", app.stats.minPlanted);
    panelRow(app, 108.0F, "fewest down", buf, app.stats.minPlanted >= 3 ? gd::kInk : gd::kOrange);
    panelRow(app, 134.0F, "support", ok ? "HELD" : "FALLING", ok ? gd::kTeal : gd::kOrange);
    const double unsupported = app.stats.moveTime > 0.0 ? app.stats.unsupportedTime / app.stats.moveTime : 0.0;
    SDL_snprintf(buf, sizeof buf, "%.0f%%", unsupported * 100.0);
    panelRow(app, 160.0F, "unsupported", buf, unsupported > 0.005 ? gd::kOrange : gd::kInk);
    SDL_snprintf(buf, sizeof buf, "%.0f%%", spider::dutyFactor(app.stats) * 100.0);
    panelRow(app, 186.0F, "duty factor", buf, gd::kInk);
    double stretch = 0.0;
    for (int i = 0; i < spider::kLegs; ++i) stretch = std::max(stretch, spider::length(spider::hipWorld(w, i) - w.legs[static_cast<std::size_t>(i)].foot) / spider::maxReach(w.p));
    SDL_snprintf(buf, sizeof buf, "%.0f%%", stretch * 100.0);
    panelRow(app, 212.0F, "leg stretch", buf, stretch > w.p.reachLimit - 0.02 ? gd::kOrange : gd::kInk);
    app.hud.text(r, kPanelX + 14.0F, 240.0F, "duty > 50% = walking", gd::kMuted);
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    const spider::Walker& w = app.walker;
    const bool ok = spider::supported(w);

    gd::backdrop(r, kWidth, kHeight);
    const char* rule = "tripod: one group of three in the air";
    if (w.gait == spider::Gait::Crawl) rule = "crawl: one leg in the air at a time";
    if (w.gait == spider::Gait::Free) rule = "free: every leg steps when it wants";
    app.hud.titleBar(r, kWidth, "[ SPIDER FOOT PLACEMENT ]", rule);

    // The world is cut at the field: a spider near the edge does not run over the hint line and the chips.
    const SDL_Rect field{0, static_cast<int>(kFieldTop), kWidth, static_cast<int>(kFieldBottom - kFieldTop)};
    SDL_SetRenderClipRect(r, &field);
    drawFootprints(app);
    if (app.showSupport) drawSupport(app, ok);
    drawTargets(app);
    drawLegs(app);
    drawBody(app, ok);

    // The goal.
    const auto gx = static_cast<float>(app.goal.x);
    const auto gy = static_cast<float>(app.goal.y);
    gd::ring(r, gx, gy, 6.0F, gd::kOrange, 20);
    gd::ring(r, gx, gy, 7.0F, gd::kInk, 20);
    SDL_SetRenderClipRect(r, nullptr);

    drawPanel(app, ok);

    app.hud.text(r, 40.0F, 446.0F, "mouse = goal   ring = step threshold   orange = in the air   polygon = support", gd::kMuted);

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
        if (std::strcmp(argv[i], "--frames") == 0) app->maxFrames = std::atol(argv[i + 1]);
    }
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        SDL_Log("SDL_Init failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    if (!SDL_CreateWindowAndRenderer("spider-foot-placement", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
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

    app->goal = spider::autoGoal(0.0);
    app->walker = spider::makeWalker(app->goal, spider::autoHeading(0.0), spider::Gait::Tripod, true);
    for (int k = 0; k < static_cast<int>(kWarmUp / kStep); ++k) stepWorld(*app);
    app->stats = spider::Stats{};  // the start-up transient (all feet in step) is not part of the numbers
    app->lastTicks = SDL_GetTicksNS();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION: {
            const Vec2 m{static_cast<double>(event->motion.x), static_cast<double>(event->motion.y)};
            if (inField(m) && !inPanel(m)) {  // the readout is not part of the field
                app.goal = m;
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
                case SDLK_1: apply(app, Action::Tripod); break;
                case SDLK_2: apply(app, Action::Crawl); break;
                case SDLK_3: apply(app, Action::Free); break;
                case SDLK_L: apply(app, Action::Lookahead); break;
                case SDLK_S: apply(app, Action::Support); break;
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
    const double dt = std::min(kMaxFrameDt, static_cast<double>(now - app.lastTicks) / 1.0e9);
    app.lastTicks = now;
    app.accumulator += dt;
    int steps = 0;
    while (app.accumulator >= kStep && steps < kMaxStepsPerFrame) {
        stepWorld(app);
        app.accumulator -= kStep;
        ++steps;
    }
    if (steps == kMaxStepsPerFrame) app.accumulator = 0.0;  // fell behind: drop the rest instead of spiralling
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
