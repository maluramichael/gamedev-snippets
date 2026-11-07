// Possibility map with SDL3 main callbacks.
//
// A guard you cannot see is not simulated. A cloud of question marks grows over every cell where he could be, one
// cell per tick. When your view cone (or, on a noisy floor, your ears) touches a cell of the cloud, the AI has to
// decide right then: the guard appears there, or those cells are crossed off. Either way it never contradicts what you
// saw. The decision rule and everything that can go wrong live in logic.hpp, covered by test.cpp.
//
// Teal marks what the map says, orange what is wrong or different: the real guard, the branch that took the
// medipack. "Naive" turns every observed question mark into a guard, the classic bug.

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
constexpr float kCell = 30.0F;   // one cell is one square of the paper grid
constexpr float kFieldX = 30.0F;
constexpr float kFieldY = 30.0F;
constexpr double kHalfAngle = 0.55;  // radians, the cone is about 63 degrees wide
constexpr double kRange = 10.0;      // cells
constexpr double kMoveInterval = 0.11;
constexpr double kMaxFrameTime = 0.25;  // clamp: no burst of catch-up ticks after a stall
constexpr int kMaxTicksPerFrame = 6;
constexpr int kMinSpeed = 1;
constexpr int kMaxSpeed = 12;
constexpr int kDefaultSpeed = 4;
constexpr float kSliderX0 = 200.0F;
constexpr float kSliderX1 = 400.0F;
constexpr float kSliderY = 442.0F;
constexpr float kChipY = 488.0F;
constexpr float kChipH = 34.0F;
constexpr double kFlashSeconds = 0.6;

constexpr gd::Color kFog{17, 24, 39, 30};
constexpr gd::Color kTealFill{13, 148, 136, 130};
constexpr gd::Color kOrangeFill{234, 88, 12, 150};

enum class Act { Honest, Naive, Medipack, Noise, Restart };

struct Chip {
    Act act;
    std::string label;
    SDL_FRect rect;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;

    pm::Grid grid{pm::levelRows()};
    pm::Sim sim;
    pm::Cell player;
    double facing = 0.0;
    bool haveMouse = false;
    float mouseX = 0.0F;
    float mouseY = 0.0F;

    bool honest = true;
    bool medipackOn = false;
    bool noiseOn = false;
    int speed = kDefaultSpeed;
    bool draggingSlider = false;
    std::array<bool, 4> held{};  // up, left, down, right
    double moveTimer = 0.0;
    bool axisToggle = false;

    Uint64 last = 0;
    double accumulator = 0.0;
    std::vector<char> visible;
    std::vector<char> heard;

    std::string text = "look at the cloud: the AI must decide";
    float flashT = -1.0F;
    int flashCell = -1;
    std::uint64_t baseSeed = 1;  // differs per page load, R rolls again
    unsigned restarts = 0;
    long maxFrames = -1;
    long frames = 0;
};

enum Dir { Up = 0, Left = 1, Down = 2, Right = 3 };

int playerIndex(const App& app) { return app.grid.index(app.player.x, app.player.y); }

SDL_FPoint centerOf(int cell, const pm::Grid& g) {
    const pm::Cell c = g.cell(cell);
    return {kFieldX + (static_cast<float>(c.x) + 0.5F) * kCell, kFieldY + (static_cast<float>(c.y) + 0.5F) * kCell};
}

void refreshView(App& app) {
    app.sim.setPlayer(playerIndex(app));
    app.visible = pm::visibleFrom(app.grid, app.player, app.facing, kHalfAngle, kRange);
    app.heard = app.sim.heard(playerIndex(app));
}

void updateFacing(App& app) {
    if (!app.haveMouse) return;
    const SDL_FPoint p = centerOf(playerIndex(app), app.grid);
    const float dx = app.mouseX - p.x;
    const float dy = app.mouseY - p.y;
    if (dx * dx + dy * dy > 36.0F) app.facing = std::atan2(static_cast<double>(dy), static_cast<double>(dx));
}

int percent(double chance) { return static_cast<int>(std::lround(chance * 100.0)); }

void describe(App& app, const pm::Observation& o) {
    char line[96];
    if (o.pack == pm::PackOutcome::Taken) {
        SDL_snprintf(line, sizeof line, "medipack seen: gone, p %d%%", percent(o.packChance));
    } else if (o.pack == pm::PackOutcome::Kept) {
        SDL_snprintf(line, sizeof line, "medipack seen: still there, p %d%%", percent(1.0 - o.packChance));
    } else if (o.outcome == pm::Outcome::Appeared && !app.sim.honest()) {
        SDL_snprintf(line, sizeof line, "naive: %d ? seen, %d guards appear", o.seen, o.spawned);
    } else if (o.outcome == pm::Outcome::Appeared) {
        SDL_snprintf(line, sizeof line, "p %d%%: %d of %d ? seen, guard appears", percent(o.chance), o.seen, o.total);
    } else {
        SDL_snprintf(line, sizeof line, "p %d%%: %d of %d ? seen, crossed off", percent(o.chance), o.seen, o.total);
    }
    app.text = line;
}

void record(App& app, const pm::Observation& o) {
    if (o.outcome == pm::Outcome::None && o.pack == pm::PackOutcome::None) return;
    describe(app, o);
    if (o.outcome == pm::Outcome::Appeared) {
        app.flashT = 0.0F;
        app.flashCell = o.cell;
    }
}

void restart(App& app) {
    app.sim = pm::makeLevelSim(app.baseSeed + app.restarts++, app.honest);
    if (app.medipackOn) app.sim.setMedipack(app.grid.find('M'));
    app.sim.setNoise(app.noiseOn);
    app.player = app.grid.cell(app.grid.find('P'));
    app.accumulator = 0.0;
    app.moveTimer = 0.0;
    app.flashT = -1.0F;
    app.text = "look at the cloud: the AI must decide";
    updateFacing(app);
    refreshView(app);
}

void apply(App& app, Act act) {
    switch (act) {
        case Act::Honest:
        case Act::Naive:
            app.honest = act == Act::Honest;
            restart(app);  // the two rules must not share a map
            break;
        case Act::Medipack:
            app.medipackOn = !app.medipackOn;
            if (app.medipackOn) {
                app.sim.setMedipack(app.grid.find('M'));
            } else {
                app.sim.clearMedipack();
            }
            app.accumulator = 0.0;
            break;
        case Act::Noise:
            app.noiseOn = !app.noiseOn;
            app.sim.setNoise(app.noiseOn);
            app.accumulator = 0.0;
            refreshView(app);
            break;
        case Act::Restart: restart(app); break;
    }
}

bool active(const App& app, Act act) {
    switch (act) {
        case Act::Honest: return app.honest;
        case Act::Naive: return !app.honest;
        case Act::Medipack: return app.medipackOn;
        case Act::Noise: return app.noiseOn;
        case Act::Restart: return false;
    }
    return false;
}

void layoutChips(App& app) {
    struct Def {
        Act act;
        const char* label;
        float gapBefore;
    };
    const std::array<Def, 5> defs{{{Act::Honest, "1 Honest", 0.0F},
                                   {Act::Naive, "2 Naive", 8.0F},
                                   {Act::Medipack, "M Medipack", 24.0F},
                                   {Act::Noise, "F Noisy floor", 8.0F},
                                   {Act::Restart, "R Restart", 24.0F}}};
    float x = kFieldX;
    for (const Def& d : defs) {
        x += d.gapBefore;
        const float w = app.hud.width(d.label, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({d.act, d.label, SDL_FRect{x, kChipY, w, kChipH}});
        x += w;
    }
}

void stepPlayer(App& app, double dt) {
    const int dx = (app.held[Right] ? 1 : 0) - (app.held[Left] ? 1 : 0);
    const int dy = (app.held[Down] ? 1 : 0) - (app.held[Up] ? 1 : 0);
    if (dx == 0 && dy == 0) {
        app.moveTimer = 0.0;
        return;
    }
    app.moveTimer -= dt;
    if (app.moveTimer > 0.0) return;
    app.moveTimer = kMoveInterval;
    int mx = dx;
    int my = dy;
    if (dx != 0 && dy != 0) {  // both axes held: alternate, the grid has no diagonal steps
        if (app.axisToggle) {
            mx = 0;
        } else {
            my = 0;
        }
        app.axisToggle = !app.axisToggle;
    }
    app.player = pm::tryMove(app.grid, app.player, mx, my);
}

void advance(App& app, double dt) {
    stepPlayer(app, dt);
    updateFacing(app);
    refreshView(app);

    const double interval = 1.0 / static_cast<double>(app.speed);
    app.accumulator += dt;
    int ticks = 0;
    while (app.accumulator >= interval && ticks < kMaxTicksPerFrame) {
        app.sim.tick();
        record(app, app.sim.settle(app.visible, app.heard));
        app.accumulator -= interval;
        ++ticks;
    }
    if (ticks == 0) record(app, app.sim.settle(app.visible, app.heard));
    if (app.flashT >= 0.0F) {
        app.flashT += static_cast<float>(dt);
        if (app.flashT > static_cast<float>(kFlashSeconds)) app.flashT = -1.0F;
    }
}

/// 1 px lines are thin on a 960x540 canvas that is scaled up, so draw a few side by side.
void thickLine(SDL_Renderer* r, SDL_FPoint a, SDL_FPoint b, gd::Color c, int half = 1) {
    gd::setColor(r, c);
    const bool flat = std::fabs(b.x - a.x) > std::fabs(b.y - a.y);
    for (int o = -half; o <= half; ++o) {
        const auto off = static_cast<float>(o);
        SDL_RenderLine(r, a.x + (flat ? 0.0F : off), a.y + (flat ? off : 0.0F), b.x + (flat ? 0.0F : off), b.y + (flat ? off : 0.0F));
    }
}

void centerText(App& app, float cx, float y, const std::string& s, gd::Color c, gd::Hud::Style style = gd::Hud::Style::Bold) {
    app.hud.text(app.renderer, cx - app.hud.width(s, style) / 2.0F, y, s, c, style);
}

void drawMap(App& app) {
    SDL_Renderer* r = app.renderer;
    const pm::Grid& g = app.grid;
    for (int i = 0; i < g.size(); ++i) {
        const pm::Cell c = g.cell(i);
        const float x = kFieldX + static_cast<float>(c.x) * kCell;
        const float y = kFieldY + static_cast<float>(c.y) * kCell;
        if (g.wall(c.x, c.y)) {
            gd::fillRect(r, x, y, kCell, kCell, gd::kInk);
            continue;
        }
        if (app.visible[static_cast<std::size_t>(i)] == 0) gd::fillRect(r, x, y, kCell, kCell, kFog);
        if (app.noiseOn && g.noisy(c.x, c.y)) {
            const bool heard = app.heard[static_cast<std::size_t>(i)] != 0;
            gd::fillRect(r, x + 2.0F, y + 2.0F, kCell - 4.0F, kCell - 4.0F, gd::Color{234, 88, 12, static_cast<Uint8>(heard ? 70 : 28)});
            if (heard) gd::strokeRect(r, x + 2.0F, y + 2.0F, kCell - 4.0F, kCell - 4.0F, gd::kOrange);
            centerText(app, x + kCell / 2.0F, y + 4.0F, "~", heard ? gd::kOrange : gd::kMuted);
        }
    }

    // The cloud. Teal: the guard could be here. Orange: ... and he took the medipack. Both: a split cell.
    for (int i = 0; i < g.size(); ++i) {
        const std::uint8_t m = app.sim.possible()[static_cast<std::size_t>(i)];
        if (m == 0) continue;
        const pm::Cell c = g.cell(i);
        const float x = kFieldX + static_cast<float>(c.x) * kCell + 2.0F;
        const float y = kFieldY + static_cast<float>(c.y) * kCell + 2.0F;
        const float s = kCell - 4.0F;
        if (m == pm::kFree) {
            gd::fillRect(r, x, y, s, s, kTealFill);
        } else if (m == pm::kTaken) {
            gd::fillRect(r, x, y, s, s, kOrangeFill);
        } else {
            gd::fillRect(r, x, y, s / 2.0F, s, kTealFill);
            gd::fillRect(r, x + s / 2.0F, y, s / 2.0F, s, kOrangeFill);
        }
        centerText(app, x + s / 2.0F, y + 2.0F, "?", gd::kInk);
    }

    // The medipack: a card while it is there, a cross where it was.
    if (app.medipackOn) {
        const SDL_FPoint p = centerOf(app.grid.find('M'), g);
        if (app.sim.medipackGone()) {
            gd::ring(r, p.x, p.y, 11.0F, gd::kOrange, 24);
            centerText(app, p.x, p.y - 11.0F, "x", gd::kOrange);
        } else {
            gd::card(r, p.x - 10.0F, p.y - 10.0F, 20.0F, 20.0F, gd::kWhite, gd::kTealDark);
            centerText(app, p.x, p.y - 11.0F, "+", gd::kTealDark);
        }
    }

    // The view cone.
    const SDL_FPoint pc = centerOf(playerIndex(app), g);
    const float reach = static_cast<float>(kRange) * kCell;
    for (const double sign : {-1.0, 1.0}) {
        const double a = app.facing + sign * kHalfAngle;
        thickLine(r, pc, {pc.x + static_cast<float>(std::cos(a)) * reach, pc.y + static_cast<float>(std::sin(a)) * reach}, gd::Color{15, 118, 110, 120}, 0);
    }
    gd::disc(r, pc.x, pc.y, 10.0F, gd::kTealDark);
    gd::ring(r, pc.x, pc.y, 10.0F, gd::kInk);
    gd::ring(r, pc.x, pc.y, 11.0F, gd::kInk);
    thickLine(r, pc, {pc.x + static_cast<float>(std::cos(app.facing)) * 17.0F, pc.y + static_cast<float>(std::sin(app.facing)) * 17.0F}, gd::kInk, 1);

    // The real guards.
    for (const pm::Guard& guard : app.sim.guards()) {
        const SDL_FPoint p = centerOf(guard.cell, g);
        const bool seen = app.visible[static_cast<std::size_t>(guard.cell)] != 0;
        if (!seen) {  // heard only: he is behind a wall or around the corner
            gd::ring(r, p.x, p.y, 15.0F, gd::kOrange);
            gd::ring(r, p.x, p.y, 19.0F, gd::Color{234, 88, 12, 120});
        }
        gd::disc(r, p.x, p.y, 10.0F, gd::kOrange);
        gd::ring(r, p.x, p.y, 10.0F, gd::kInk);
        centerText(app, p.x, p.y - 11.0F, "G", gd::kWhite);
    }
    if (app.flashT >= 0.0F && app.flashCell >= 0) {
        const SDL_FPoint p = centerOf(app.flashCell, g);
        const float t = app.flashT / static_cast<float>(kFlashSeconds);
        gd::ring(r, p.x, p.y, 14.0F + 38.0F * t, gd::Color{234, 88, 12, static_cast<Uint8>(230.0F * (1.0F - t))});
    }
}

void drawControls(App& app) {
    SDL_Renderer* r = app.renderer;
    app.hud.text(r, kFieldX, kSliderY - 11.0F, "Q/E Speed", gd::kInk, gd::Hud::Style::Bold);
    gd::fillRect(r, kSliderX0, kSliderY - 2.0F, kSliderX1 - kSliderX0, 4.0F, gd::kGrid);
    gd::strokeRect(r, kSliderX0, kSliderY - 2.0F, kSliderX1 - kSliderX0, 4.0F, gd::kInk);
    const auto x = static_cast<float>(pm::xFromSpeed(app.speed, static_cast<double>(kSliderX0), static_cast<double>(kSliderX1), kMinSpeed, kMaxSpeed));
    gd::fillRect(r, kSliderX0, kSliderY - 2.0F, x - kSliderX0, 4.0F, gd::kTeal);
    gd::card(r, x - 6.0F, kSliderY - 10.0F, 12.0F, 20.0F, app.draggingSlider ? gd::kOrange : gd::kWhite, gd::kInk);
    char value[24];
    SDL_snprintf(value, sizeof value, "%d cells/s", app.speed);
    app.hud.text(r, kSliderX1 + 18.0F, kSliderY - 11.0F, value, gd::kInk, gd::Hud::Style::Bold);

    char stats[64];
    const auto guards = static_cast<int>(app.sim.guards().size());
    SDL_snprintf(stats, sizeof stats, "cloud %d cells   guards %d", app.sim.cells(), guards);
    app.hud.text(r, 560.0F, kSliderY - 11.0F, stats, guards > 1 ? gd::kOrange : gd::kInk, gd::Hud::Style::Bold);

    app.hud.text(r, kFieldX, 458.0F, app.text, gd::kMuted);

    // Legend, only for what is switched on.
    float lx = 560.0F;
    const float ly = 459.0F;
    gd::fillRect(r, lx, ly + 2.0F, 16.0F, 16.0F, kTealFill);
    app.hud.text(r, lx + 22.0F, ly - 1.0F, "could be here", gd::kMuted);
    lx += 22.0F + app.hud.width("could be here") + 18.0F;
    if (app.medipackOn) {
        gd::fillRect(r, lx, ly + 2.0F, 16.0F, 16.0F, kOrangeFill);
        app.hud.text(r, lx + 22.0F, ly - 1.0F, "took the pack", gd::kMuted);
    }

    for (const Chip& c : app.chips) {
        const bool on = active(app, c.act);
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? gd::kTeal : gd::kWhite, gd::kInk);
        centerText(app, c.rect.x + c.rect.w / 2.0F, c.rect.y + 6.0F, c.label, on ? gd::kWhite : gd::kInk);
    }
}

void draw(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    app.hud.titleBar(r, kWidth, "[ POSSIBILITY MAP ]", "WASD move  mouse aim  Q/E speed");
    drawMap(app);
    drawControls(app);
}

bool inSliderHit(float x, float y) { return x >= kSliderX0 - 12.0F && x <= kSliderX1 + 12.0F && y >= kSliderY - 14.0F && y <= kSliderY + 14.0F; }

void setSpeedFromX(App& app, float x) {
    app.speed = pm::speedFromX(static_cast<double>(x), static_cast<double>(kSliderX0), static_cast<double>(kSliderX1), kMinSpeed, kMaxSpeed);
}

void setHeld(App& app, SDL_Keycode key, bool down) {
    switch (key) {
        case SDLK_W:
        case SDLK_UP: app.held[Up] = down; break;
        case SDLK_A:
        case SDLK_LEFT: app.held[Left] = down; break;
        case SDLK_S:
        case SDLK_DOWN: app.held[Down] = down; break;
        case SDLK_D:
        case SDLK_RIGHT: app.held[Right] = down; break;
        default: break;
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
    if (!SDL_CreateWindowAndRenderer("possibility-map", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
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
    app->baseSeed = SDL_GetPerformanceCounter();
    restart(*app);
    app->last = SDL_GetPerformanceCounter();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    SDL_ConvertEventToRenderCoordinates(static_cast<App*>(appstate)->renderer, event);
    auto& app = *static_cast<App*>(appstate);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION:
            app.haveMouse = true;
            app.mouseX = event->motion.x;
            app.mouseY = event->motion.y;
            if (app.draggingSlider) setSpeedFromX(app, event->motion.x);
            break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const SDL_FPoint p{event->button.x, event->button.y};
            app.haveMouse = true;
            app.mouseX = p.x;
            app.mouseY = p.y;
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) apply(app, c.act);
            }
            if (inSliderHit(p.x, p.y)) {
                app.draggingSlider = true;
                setSpeedFromX(app, p.x);
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event->button.button == SDL_BUTTON_LEFT) app.draggingSlider = false;
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
            app.draggingSlider = false;
            break;
        case SDL_EVENT_WINDOW_FOCUS_LOST:
            app.draggingSlider = false;
            app.held.fill(false);  // a key released while unfocused never arrives
            break;
        case SDL_EVENT_KEY_UP: setHeld(app, event->key.key, false); break;
        case SDL_EVENT_KEY_DOWN:
            setHeld(app, event->key.key, true);
            if (event->key.repeat) break;  // everything below toggles or triggers, none of it should fire while held
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: apply(app, Act::Honest); break;
                case SDLK_2: apply(app, Act::Naive); break;
                case SDLK_M: apply(app, Act::Medipack); break;
                case SDLK_F: apply(app, Act::Noise); break;
                case SDLK_R: apply(app, Act::Restart); break;
                case SDLK_Q: app.speed = std::max(kMinSpeed, app.speed - 1); break;
                case SDLK_E: app.speed = std::min(kMaxSpeed, app.speed + 1); break;
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
    const double dt = std::min(static_cast<double>(now - app.last) / static_cast<double>(SDL_GetPerformanceFrequency()), kMaxFrameTime);
    app.last = now;
    advance(app, dt);
    draw(app);
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
