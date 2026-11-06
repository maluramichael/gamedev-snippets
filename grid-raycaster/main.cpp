// Wolfenstein-style raycaster with SDL3 main callbacks.
//
// Left the map with one ray per screen column, right what those rays see: a wall column per ray, taller when closer.
// Two things are worth switching. The way a ray finds its wall: the DDA hops from grid line to grid line and can't miss
// a cell, a fixed-step march samples the ray and skips corners and reports walls late (jagged tops). And the distance
// that sets the wall height: the straight distance along the ray bends flat walls (fisheye), the distance measured
// along the view direction keeps them flat. It starts with the DDA and the fisheye so the bent far wall is the first
// thing you see. The ray casting itself lives in logic.hpp and is covered by test.cpp.

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
constexpr float kPanelY = 44.0F;
constexpr float kMapX = 24.0F;
constexpr float kCell = 20.0F;
constexpr float kViewX = 444.0F;
constexpr float kViewW = 480.0F;
constexpr float kViewH = 300.0F;
constexpr int kColumns = 120;
constexpr float kColW = kViewW / static_cast<float>(kColumns);
constexpr double kFov = 80.0 * grc::kPi / 180.0;
constexpr double kWallCells = 2.0;   // wall height in cells, the eye is in the middle
constexpr double kMoveSpeed = 3.0;   // cells per second
constexpr double kTurnSpeed = 2.0;   // radians per second
constexpr double kDragTurn = 0.008;  // radians per pixel dragged in the view
constexpr double kSwayAmplitude = 0.45;
constexpr double kSwaySpeed = 0.6;
constexpr float kGrabRadius = 14.0F;  // px around the player dot that grabs it
constexpr float kChipH = 30.0F;

enum class Method { Dda, March };
enum class Action { Dda, March, Step1, StepHalf, StepTenth, Perpendicular, Straight, Reset };
enum class Drag { None, Player, Turn };

constexpr gd::Color kWallA = gd::kTeal;
constexpr gd::Color kWallB = gd::kTealDark;

struct Chip {
    Action action;
    std::string label;
    SDL_FRect rect;
    bool naive;  // active chips of the naive variants are orange
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;

    grc::Map map = grc::defaultMap();
    grc::Camera cam{grc::startPosition(), grc::startAngle(), kFov};
    grc::Scan scan;
    std::vector<grc::Visit> visits;   // probe ray, DDA
    std::vector<grc::Vec2> samples;   // probe ray, march

    Method method = Method::Dda;
    double step = 0.5;
    bool fisheye = true;  // wall height from the straight distance

    bool idle = true;  // gentle sway until the first input
    double baseAngle = grc::startAngle();
    double swayPhase = 0.0;

    Drag drag = Drag::None;
    SDL_FPoint mouse{-100.0F, -100.0F};
    bool mouseInside = false;

    Uint64 lastTicks = 0;
    long maxFrames = -1;
    long frames = 0;
};

bool inRect(SDL_FPoint p, float x, float y, float w, float h) { return p.x >= x && p.y >= y && p.x < x + w && p.y < y + h; }

float mapW(const App& app) { return kCell * static_cast<float>(app.map.width()); }
float mapH(const App& app) { return kCell * static_cast<float>(app.map.height()); }

SDL_FPoint toScreen(grc::Vec2 p) { return {kMapX + static_cast<float>(p.x) * kCell, kPanelY + static_cast<float>(p.y) * kCell}; }

grc::Vec2 toWorld(SDL_FPoint p) { return {static_cast<double>((p.x - kMapX) / kCell), static_cast<double>((p.y - kPanelY) / kCell)}; }

void stopIdle(App& app) {
    if (!app.idle) return;
    app.idle = false;
    app.baseAngle = app.cam.angle;
}

void resetAll(App& app) {
    app.map = grc::defaultMap();
    app.cam = {grc::startPosition(), grc::startAngle(), kFov};
    app.method = Method::Dda;
    app.step = 0.5;
    app.fisheye = true;
    app.idle = true;
    app.baseAngle = grc::startAngle();
    app.swayPhase = 0.0;
    app.drag = Drag::None;
    app.lastTicks = 0;
}

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
        float x;
        float y;
        bool naive;
    };
    const float row1 = 456.0F;
    const float row2 = 496.0F;
    std::array<Def, 8> defs{{{Action::Dda, "1 DDA", 0, row1, false},
                             {Action::March, "2 Fixed step", 0, row1, true},
                             {Action::Step1, "3 Step 1.0", 0, row1, true},
                             {Action::StepHalf, "4 Step 0.5", 0, row1, true},
                             {Action::StepTenth, "5 Step 0.1", 0, row1, true},
                             {Action::Perpendicular, "6 Perpendicular distance", 0, row2, false},
                             {Action::Straight, "7 Straight distance (fisheye)", 0, row2, true},
                             {Action::Reset, "R Reset", 0, row2, false}}};
    float x = 40.0F;
    float lastY = row1;
    for (Def& d : defs) {
        if (d.y != lastY) {
            x = 40.0F;
            lastY = d.y;
        }
        const float w = app.hud.width(d.label, gd::Hud::Style::Bold) + 24.0F;
        // Wider gaps between the groups: method | step.
        const bool newGroup = d.action == Action::Step1;
        if (newGroup) x += 16.0F;
        app.chips.push_back({d.action, d.label, SDL_FRect{x, d.y, w, kChipH}, d.naive});
        x += w + 8.0F;
    }
}

void apply(App& app, Action action) {
    switch (action) {
        case Action::Dda: app.method = Method::Dda; break;
        case Action::March: app.method = Method::March; break;
        case Action::Step1: app.method = Method::March; app.step = 1.0; break;
        case Action::StepHalf: app.method = Method::March; app.step = 0.5; break;
        case Action::StepTenth: app.method = Method::March; app.step = 0.1; break;
        case Action::Perpendicular: app.fisheye = false; break;
        case Action::Straight: app.fisheye = true; break;
        case Action::Reset: resetAll(app); break;
    }
}

bool active(const App& app, Action action) {
    switch (action) {
        case Action::Dda: return app.method == Method::Dda;
        case Action::March: return app.method == Method::March;
        case Action::Step1: return app.method == Method::March && app.step == 1.0;
        case Action::StepHalf: return app.method == Method::March && app.step == 0.5;
        case Action::StepTenth: return app.method == Method::March && app.step == 0.1;
        case Action::Perpendicular: return !app.fisheye;
        case Action::Straight: return app.fisheye;
        case Action::Reset: return false;
    }
    return false;
}

/// The hit of the chosen method for one column.
const grc::Hit& chosen(const App& app, const grc::Column& c) { return app.method == Method::Dda ? c.dda : c.march; }

int probeColumn(const App& app) {
    if (app.mouseInside && inRect(app.mouse, kViewX, kPanelY, kViewW, kViewH)) {
        return std::clamp(static_cast<int>((app.mouse.x - kViewX) / kColW), 0, kColumns - 1);
    }
    return kColumns / 2;
}

void togglePicked(App& app) {
    const grc::Vec2 w = toWorld(app.mouse);
    const int cx = static_cast<int>(std::floor(w.x));
    const int cy = static_cast<int>(std::floor(w.y));
    // The border stays closed, so there is always a wall to hit.
    if (cx <= 0 || cy <= 0 || cx >= app.map.width() - 1 || cy >= app.map.height() - 1) return;
    app.map.toggle(cx, cy);
    if (!grc::canStand(app.map, app.cam.pos, grc::kPlayerRadius)) app.map.toggle(cx, cy);  // not onto the player
}

void update(App& app, double dt) {
    const bool* keys = SDL_GetKeyboardState(nullptr);
    double turn = 0.0;
    double forward = 0.0;
    double strafe = 0.0;
    if (keys[SDL_SCANCODE_LEFT] || keys[SDL_SCANCODE_Q]) turn -= 1.0;
    if (keys[SDL_SCANCODE_RIGHT] || keys[SDL_SCANCODE_E]) turn += 1.0;
    if (keys[SDL_SCANCODE_W] || keys[SDL_SCANCODE_UP]) forward += 1.0;
    if (keys[SDL_SCANCODE_S] || keys[SDL_SCANCODE_DOWN]) forward -= 1.0;
    if (keys[SDL_SCANCODE_D]) strafe += 1.0;
    if (keys[SDL_SCANCODE_A]) strafe -= 1.0;
    if (turn != 0.0 || forward != 0.0 || strafe != 0.0) stopIdle(app);

    if (app.idle) {
        app.swayPhase = std::fmod(app.swayPhase + dt * kSwaySpeed, 2.0 * grc::kPi);
        app.cam.angle = grc::normalizeAngle(app.baseAngle + kSwayAmplitude * std::sin(app.swayPhase));
    } else {
        app.cam.angle = grc::normalizeAngle(app.cam.angle + turn * kTurnSpeed * dt);
        const grc::Vec2 f = grc::viewDir(app.cam);
        const grc::Vec2 side{-f.y, f.x};
        const grc::Vec2 delta{(f.x * forward + side.x * strafe) * kMoveSpeed * dt, (f.y * forward + side.y * strafe) * kMoveSpeed * dt};
        app.cam.pos = grc::tryMove(app.map, app.cam.pos, delta, grc::kPlayerRadius);
    }

    grc::scanInto(app.scan, app.map, app.cam, kColumns, app.step);

    // The probe ray again, this time with its trail.
    app.visits.clear();
    app.samples.clear();
    const grc::Vec2 ray = app.scan.columns[static_cast<std::size_t>(probeColumn(app))].ray;
    if (app.method == Method::Dda) {
        grc::castDda(app.map, app.cam.pos, ray, &app.visits);
    } else {
        grc::castMarch(app.map, app.cam.pos, ray, app.step, 64.0, &app.samples);
    }
}

gd::Color shade(gd::Color c, double z) {
    const double f = std::clamp(1.15 - z / 14.0, 0.3, 1.0);
    return {static_cast<Uint8>(c.r * f), static_cast<Uint8>(c.g * f), static_cast<Uint8>(c.b * f), 255};
}

void drawMap(App& app) {
    SDL_Renderer* r = app.renderer;
    const float w = mapW(app);
    const float h = mapH(app);
    gd::fillRect(r, kMapX, kPanelY, w, h, gd::kWhite);

    // The cells the probe ray walked through (DDA only: the march has no such list).
    for (const grc::Visit& v : app.visits) {
        gd::fillRect(r, kMapX + static_cast<float>(v.cx) * kCell, kPanelY + static_cast<float>(v.cy) * kCell, kCell, kCell, gd::Color{13, 148, 136, 70});
    }
    gd::setColor(r, gd::kGrid);
    for (int x = 0; x <= app.map.width(); ++x) SDL_RenderLine(r, kMapX + static_cast<float>(x) * kCell, kPanelY, kMapX + static_cast<float>(x) * kCell, kPanelY + h);
    for (int y = 0; y <= app.map.height(); ++y) SDL_RenderLine(r, kMapX, kPanelY + static_cast<float>(y) * kCell, kMapX + w, kPanelY + static_cast<float>(y) * kCell);
    for (int y = 0; y < app.map.height(); ++y) {
        for (int x = 0; x < app.map.width(); ++x) {
            if (app.map.solid(x, y)) gd::fillRect(r, kMapX + static_cast<float>(x) * kCell, kPanelY + static_cast<float>(y) * kCell, kCell, kCell, gd::kInk);
        }
    }

    // One line per column. Orange where the march ended in another cell than the DDA.
    const SDL_FPoint eye = toScreen(app.cam.pos);
    for (std::size_t i = 0; i < app.scan.columns.size(); i += 2) {  // every second ray, 120 lines blur into moire
        const grc::Column& c = app.scan.columns[i];
        const grc::Hit& h2 = chosen(app, c);
        if (!h2.found) continue;
        const SDL_FPoint end = toScreen({app.cam.pos.x + c.ray.x * h2.dist, app.cam.pos.y + c.ray.y * h2.dist});
        const bool wrong = app.method == Method::March && (c.march.cx != c.dda.cx || c.march.cy != c.dda.cy);
        gd::setColor(r, wrong ? gd::Color{234, 88, 12, 200} : gd::Color{13, 148, 136, 90});
        SDL_RenderLine(r, eye.x, eye.y, end.x, end.y);
    }

    // The probe ray with its sample points.
    const grc::Column& probe = app.scan.columns[static_cast<std::size_t>(probeColumn(app))];
    const grc::Hit& ph = chosen(app, probe);
    if (ph.found) {
        const SDL_FPoint end = toScreen({app.cam.pos.x + probe.ray.x * ph.dist, app.cam.pos.y + probe.ray.y * ph.dist});
        gd::setColor(r, gd::kInk);
        SDL_RenderLine(r, eye.x, eye.y, end.x, end.y);
        for (const grc::Visit& v : app.visits) {
            const SDL_FPoint p = toScreen(v.enter);
            gd::fillRect(r, p.x - 3.0F, p.y - 3.0F, 6.0F, 6.0F, gd::kTealDark);
            gd::strokeRect(r, p.x - 3.0F, p.y - 3.0F, 6.0F, 6.0F, gd::kWhite);
        }
        for (const grc::Vec2& s : app.samples) {
            const SDL_FPoint p = toScreen(s);
            gd::fillRect(r, p.x - 1.5F, p.y - 1.5F, 3.0F, 3.0F, gd::kOrange);
        }
        gd::ring(r, end.x, end.y, 5.0F, gd::kOrange);
    }

    // The player.
    const grc::Vec2 f = grc::viewDir(app.cam);
    gd::setColor(r, gd::kInk);
    SDL_RenderLine(r, eye.x, eye.y, eye.x + static_cast<float>(f.x) * 14.0F, eye.y + static_cast<float>(f.y) * 14.0F);
    gd::disc(r, eye.x, eye.y, 5.0F, gd::kTeal);
    gd::ring(r, eye.x, eye.y, 5.0F, gd::kInk);
    gd::strokeRect(r, kMapX, kPanelY, w, h, gd::kInk);
}

void drawView(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::fillRect(r, kViewX, kPanelY, kViewW, kViewH / 2.0F, gd::Color{226, 236, 235, 255});
    gd::gradient(r, kViewX, kPanelY + kViewH / 2.0F, kViewW, kViewH / 2.0F, gd::Color{222, 219, 214, 255}, gd::Color{176, 172, 166, 255});

    const grc::Vec2 view = grc::viewDir(app.cam);
    for (std::size_t i = 0; i < app.scan.columns.size(); ++i) {
        const grc::Column& c = app.scan.columns[i];
        const grc::Hit& h = chosen(app, c);
        if (!h.found) continue;
        const double z = grc::perpendicular(h.dist, c.ray, view);
        const double depth = app.fisheye ? h.dist : z;
        const auto hpx = static_cast<float>(std::min(grc::wallHeight(depth, static_cast<double>(kViewW), kFov, kWallCells), static_cast<double>(kViewH)));
        const float x = kViewX + static_cast<float>(i) * kColW;
        gd::fillRect(r, x, kPanelY + (kViewH - hpx) / 2.0F, kColW, hpx, shade(h.side == grc::Side::Vertical ? kWallA : kWallB, z));
        if (app.method == Method::March && (c.march.cx != c.dda.cx || c.march.cy != c.dda.cy)) gd::fillRect(r, x, kPanelY, kColW, 5.0F, gd::kOrange);
    }

    const float px = kViewX + static_cast<float>(probeColumn(app)) * kColW;
    gd::strokeRect(r, px - 1.0F, kPanelY, kColW + 2.0F, kViewH, gd::kInk);
    gd::strokeRect(r, kViewX, kPanelY, kViewW, kViewH, gd::kInk);
}

void drawStats(App& app) {
    SDL_Renderer* r = app.renderer;
    using S = gd::Hud::Style;
    const bool dda = app.method == Method::Dda;
    const float y0 = 378.0F;

    gd::card(r, kMapX, y0, 400.0F, 68.0F, gd::kWhite, gd::kInk);
    char line[96];
    SDL_snprintf(line, sizeof line, "DDA    %.1f cells/ray", app.scan.ddaSteps);
    app.hud.text(r, 38.0F, y0 + 6.0F, line, dda ? gd::kTeal : gd::kMuted, dda ? S::Bold : S::Regular);
    SDL_snprintf(line, sizeof line, "March  %.1f samples/ray (step %.1f)", app.scan.marchSteps, app.step);
    app.hud.text(r, 38.0F, y0 + 26.0F, line, dda ? gd::kMuted : gd::kOrange, dda ? S::Regular : S::Bold);
    SDL_snprintf(line, sizeof line, "March misses DDA cell: %d / %d", app.scan.different, kColumns);
    app.hud.text(r, 38.0F, y0 + 46.0F, line, app.scan.different > 0 ? gd::kOrange : gd::kTeal, S::Bold);

    gd::card(r, kViewX, y0, kViewW, 68.0F, gd::kWhite, gd::kInk);
    const int col = probeColumn(app);
    const grc::Column& c = app.scan.columns[static_cast<std::size_t>(col)];
    const grc::Hit& h = chosen(app, c);
    const grc::Vec2 view = grc::viewDir(app.cam);
    const double cosA = c.ray.x * view.x + c.ray.y * view.y;
    const double off = std::acos(std::clamp(cosA, -1.0, 1.0)) * 180.0 / grc::kPi;
    SDL_snprintf(line, sizeof line, "probe ray %d: %.1f° off center, cos = %.3f", col + 1, off, cosA);
    app.hud.text(r, kViewX + 14.0F, y0 + 6.0F, line, gd::kInk);
    SDL_snprintf(line, sizeof line, "d = %.2f (along ray)   z = %.2f (depth)", h.dist, grc::perpendicular(h.dist, c.ray, view));
    app.hud.text(r, kViewX + 14.0F, y0 + 26.0F, line, gd::kInk);
    app.hud.text(r, kViewX + 14.0F, y0 + 46.0F, app.fisheye ? "wall height from d: flat walls bend" : "wall height from z: flat walls stay flat",
                 app.fisheye ? gd::kOrange : gd::kTeal, S::Bold);
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    char right[64];
    SDL_snprintf(right, sizeof right, "%s · %s", app.method == Method::Dda ? "DDA" : "fixed step", app.fisheye ? "straight distance" : "perpendicular distance");
    app.hud.titleBar(r, kWidth, "[ GRID RAYCASTER ]", right);

    drawMap(app);
    drawView(app);
    app.hud.text(r, kMapX, 350.0F, "click cell: wall on/off  drag dot: move", gd::kMuted);
    app.hud.text(r, kViewX, 350.0F, "drag: turn   W A S D move   Q E / arrows turn", gd::kMuted);
    drawStats(app);

    for (const Chip& c : app.chips) {
        const bool on = active(app, c.action);
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? (c.naive ? gd::kOrange : gd::kTeal) : gd::kWhite, gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 4.0F, c.label, on ? gd::kWhite : gd::kInk,
                     gd::Hud::Style::Bold);
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
    if (!SDL_CreateWindowAndRenderer("grid-raycaster", kWidth, kHeight, 0, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, SDL_LOGICAL_PRESENTATION_LETTERBOX);
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderVSync(app->renderer, 1);
    layoutChips(*app);
    grc::scanInto(app->scan, app->map, app->cam, kColumns, app->step);
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION:
            app.mouse = {event->motion.x, event->motion.y};
            app.mouseInside = true;
            if (app.drag == Drag::Player) {
                const grc::Vec2 target = toWorld(app.mouse);
                app.cam.pos = grc::tryMove(app.map, app.cam.pos, {target.x - app.cam.pos.x, target.y - app.cam.pos.y}, grc::kPlayerRadius);
            } else if (app.drag == Drag::Turn) {
                app.cam.angle = grc::normalizeAngle(app.cam.angle + static_cast<double>(event->motion.xrel) * kDragTurn);
            }
            break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            app.mouse = {event->button.x, event->button.y};
            app.mouseInside = true;
            bool onChip = false;
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&app.mouse, &c.rect)) {
                    apply(app, c.action);
                    onChip = true;
                    break;
                }
            }
            if (onChip) break;
            if (inRect(app.mouse, kMapX, kPanelY, mapW(app), mapH(app))) {
                const SDL_FPoint eye = toScreen(app.cam.pos);
                if (std::hypot(app.mouse.x - eye.x, app.mouse.y - eye.y) <= kGrabRadius) {
                    stopIdle(app);
                    app.drag = Drag::Player;
                } else {
                    togglePicked(app);
                }
            } else if (inRect(app.mouse, kViewX, kPanelY, kViewW, kViewH)) {
                stopIdle(app);
                app.drag = Drag::Turn;
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event->button.button == SDL_BUTTON_LEFT) app.drag = Drag::None;
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
            app.mouseInside = false;
            app.drag = Drag::None;
            break;
        case SDL_EVENT_WINDOW_FOCUS_LOST: app.drag = Drag::None; break;
        case SDL_EVENT_KEY_DOWN:
            if (event->key.repeat) break;
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: apply(app, Action::Dda); break;
                case SDLK_2: apply(app, Action::March); break;
                case SDLK_3: apply(app, Action::Step1); break;
                case SDLK_4: apply(app, Action::StepHalf); break;
                case SDLK_5: apply(app, Action::StepTenth); break;
                case SDLK_6: apply(app, Action::Perpendicular); break;
                case SDLK_7: apply(app, Action::Straight); break;
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
    const Uint64 now = SDL_GetTicks();
    const double dt = app.lastTicks == 0 ? 0.0 : std::min(static_cast<double>(now - app.lastTicks) / 1000.0, 0.25);
    app.lastTicks = now;
    update(app, dt);
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
