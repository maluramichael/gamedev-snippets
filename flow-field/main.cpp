// Flow fields with SDL3 main callbacks.
//
// Two ways to get 200 agents to a goal on a grid with walls. FLOW FIELD: one breadth first search starts at the goal and
// writes the distance into every cell (heat map), every cell gets an arrow that points downhill on that map, and an
// agent only reads the arrow of the cell it stands in. A* PER AGENT: every agent runs its own search from where it
// stands, and again whenever the goal or a wall changes. The card under the grid counts the cells each side expands
// after the last change. The wave that grows from the goal after a click is only a drawing: the agents use the finished
// field at once. The search, the arrows, the agent step and A* live in logic.hpp and are covered by test.cpp.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

#include "hud.hpp"
#include "logic.hpp"

namespace {

constexpr int kWidth = 960;
constexpr int kHeight = 540;
constexpr int kCols = 48;
constexpr int kRows = 16;
constexpr float kCell = 20.0F;
constexpr float kGridTop = 34.0F;
constexpr float kGridBottom = kGridTop + kCell * static_cast<float>(kRows);
constexpr int kAgentCount = 200;
constexpr double kSpeed = 8.0;         // cells per second, each agent varies +-15 percent
constexpr double kAgentRadius = 0.28;  // cells
constexpr double kMaxStep = 0.02;      // s, one agent step is at most 0.2 cells so a step never skips through a wall
constexpr double kRevealSpeed = 45.0;  // cells per second of the drawn wave
constexpr int kSpawnDistance = 20;     // arriving agents restart at least this many steps from the goal
constexpr long kStatsDelayFrames = 8;  // the A* count waits for this many quiet frames, so dragging stays light

enum class Action { ToolGoal, ToolWalls, Heat, Arrows, FollowFlow, FollowAstar, Respawn, ClearWalls, ResetMap };
enum class Tool { Goal, Walls };
enum class Follow { Flow, Astar };

struct Chip {
    Action action;
    std::string label;
    SDL_FRect rect;
};

struct Agent {
    ff::Vec2 pos{0.0, 0.0};
    double speed = 1.0;
    std::vector<ff::Cell> path;  // A* mode only
    std::size_t next = 0;
    int pathVersion = -1;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;

    ff::Grid grid;
    ff::Cell goal = ff::presetGoal();
    ff::Field field;
    std::vector<ff::Vec2> dirs;
    int version = 0;  // counts changes of goal or walls; an A* path is valid for one version
    std::vector<Agent> agents;
    ff::Rng rng{20270409U};

    Tool tool = Tool::Goal;
    Follow follow = Follow::Flow;
    bool heat = true;
    bool arrows = true;
    bool brushing = false;
    bool brushValue = true;
    ff::Vec2 brushLast{0.0, 0.0};
    double reveal = 3.0;

    bool statsDirty = true;
    long changedFrame = 0;
    int bfsWork = 0;
    ff::AstarWork astarWork;

    Uint64 lastCounter = 0;
    long maxFrames = -1;
    long frames = 0;
};

gd::Color mix(gd::Color a, gd::Color b, double t) {
    const auto m = [t](Uint8 x, Uint8 y) { return static_cast<Uint8>(std::lround(x + (y - x) * t)); };
    return gd::Color{m(a.r, b.r), m(a.g, b.g), m(a.b, b.b), 255};
}

gd::Color withAlpha(gd::Color c, double a) {
    c.a = static_cast<Uint8>(std::clamp(std::lround(a * 255.0), 0L, 255L));
    return c;
}

/// Mouse position (pixels) to cell units, may lie outside the grid.
ff::Vec2 toCellUnits(SDL_FPoint p) { return {static_cast<double>(p.x / kCell), static_cast<double>((p.y - kGridTop) / kCell)}; }

float cellX(int x) { return static_cast<float>(x) * kCell; }
float cellY(int y) { return kGridTop + static_cast<float>(y) * kCell; }

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
        float gapBefore;
        float y;
    };
    constexpr float kRow1 = 444.0F;
    constexpr float kRow2 = 482.0F;
    constexpr float kLeft = 174.0F;
    const std::array<Def, 9> defs{{{Action::ToolGoal, "1 Goal", 0.0F, kRow1},
                                   {Action::ToolWalls, "2 Walls", 8.0F, kRow1},
                                   {Action::Heat, "3 Heat", 24.0F, kRow1},
                                   {Action::Arrows, "4 Arrows", 8.0F, kRow1},
                                   {Action::ClearWalls, "C Clear walls", 24.0F, kRow1},
                                   {Action::ResetMap, "M Reset map", 8.0F, kRow1},
                                   {Action::FollowFlow, "5 Flow field", 0.0F, kRow2},
                                   {Action::FollowAstar, "6 A* per agent", 8.0F, kRow2},
                                   {Action::Respawn, "R Respawn", 24.0F, kRow2}}};
    float x = kLeft;
    float lastY = kRow1;
    for (const Def& d : defs) {
        if (d.y != lastY) {
            x = kLeft;
            lastY = d.y;
        }
        x += d.gapBefore;
        const float w = app.hud.width(d.label, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({d.action, d.label, SDL_FRect{x, d.y, w, 30.0F}});
        x += w;
    }
}

bool active(const App& app, Action action) {
    switch (action) {
        case Action::ToolGoal: return app.tool == Tool::Goal;
        case Action::ToolWalls: return app.tool == Tool::Walls;
        case Action::Heat: return app.heat;
        case Action::Arrows: return app.arrows;
        case Action::FollowFlow: return app.follow == Follow::Flow;
        case Action::FollowAstar: return app.follow == Follow::Astar;
        case Action::Respawn:
        case Action::ClearWalls:
        case Action::ResetMap: return false;
    }
    return false;
}

/// The search runs again: one BFS from the goal, one arrow per cell. Agents in A* mode notice the new version.
void recompute(App& app) {
    app.field = ff::bfs(app.grid, app.goal);
    app.dirs = ff::buildDirections(app.grid, app.field);
    ++app.version;
    app.statsDirty = true;
    app.changedFrame = app.frames;
}

void refreshStats(App& app) {
    std::vector<ff::Cell> starts;
    starts.reserve(app.agents.size());
    for (const Agent& a : app.agents) starts.push_back(ff::cellOf(a.pos));
    app.bfsWork = app.field.expanded;
    app.astarWork = ff::astarWork(app.grid, app.goal, starts);
    app.statsDirty = false;
}

void spawn(App& app, Agent& a, int minDist) {
    a.path.clear();
    a.next = 0;
    a.pathVersion = -1;
    const auto c = ff::pickSpawn(app.field, minDist, app.rng);
    if (!c) return;
    // 0.2 around the centre plus the radius stays inside the cell.
    a.pos = {c->x + 0.5 + (app.rng.uniform() - 0.5) * 0.4, c->y + 0.5 + (app.rng.uniform() - 0.5) * 0.4};
}

void respawnAll(App& app) {
    app.agents.resize(kAgentCount);
    for (Agent& a : app.agents) {
        a.speed = 0.85 + 0.3 * app.rng.uniform();
        spawn(app, a, 0);
    }
    app.statsDirty = true;
    app.changedFrame = app.frames;
}

void resetMap(App& app) {
    app.grid = ff::gridFromRows(ff::presetRows());
    app.goal = ff::presetGoal();
    app.brushing = false;
    app.reveal = 0.0;
    recompute(app);
    respawnAll(app);
}

void apply(App& app, Action action) {
    switch (action) {
        case Action::ToolGoal:
            app.tool = Tool::Goal;
            app.brushing = false;
            break;
        case Action::ToolWalls:
            app.tool = Tool::Walls;
            app.brushing = false;
            break;
        case Action::Heat: app.heat = !app.heat; break;
        case Action::Arrows: app.arrows = !app.arrows; break;
        case Action::FollowFlow:
        case Action::FollowAstar:
            app.follow = action == Action::FollowFlow ? Follow::Flow : Follow::Astar;
            for (Agent& a : app.agents) a.pathVersion = -1;
            break;
        case Action::Respawn: respawnAll(app); break;
        case Action::ClearWalls:
            std::fill(app.grid.wall.begin(), app.grid.wall.end(), static_cast<unsigned char>(0));
            app.brushing = false;
            recompute(app);
            break;
        case Action::ResetMap: resetMap(app); break;
    }
}

void setGoalAt(App& app, ff::Vec2 p) {
    const ff::Cell c = ff::cellOf(p);
    if (app.grid.blocked(c.x, c.y) || c == app.goal) return;
    app.goal = c;
    recompute(app);
}

/// Paints or erases along the way from the last mouse position, so a fast drag leaves no gaps.
void brushWalls(App& app, ff::Vec2 from, ff::Vec2 to) {
    const double len = std::hypot(to.x - from.x, to.y - from.y);
    const int n = std::max(1, static_cast<int>(std::ceil(len / 0.25)));
    bool changed = false;
    for (int i = 1; i <= n; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(n);
        const ff::Vec2 p{from.x + (to.x - from.x) * t, from.y + (to.y - from.y) * t};
        changed = ff::setWall(app.grid, ff::cellOf(p), app.brushValue, app.goal) || changed;
    }
    if (changed) recompute(app);
}

void advance(App& app, Agent& a, double h) {
    const ff::Cell c = ff::cellOf(a.pos);
    if (app.grid.blocked(c.x, c.y)) {  // a wall was painted over it
        spawn(app, a, kSpawnDistance);
        return;
    }
    if (c == app.goal) {  // arrived: restart far away, so the flow never dries up
        spawn(app, a, kSpawnDistance);
        return;
    }
    const double step = kSpeed * a.speed * h;
    if (app.follow == Follow::Flow) {
        const ff::Vec2 d = app.dirs[app.grid.index(c.x, c.y)];  // the whole "search" of this agent: one lookup
        a.pos.x += d.x * step;
        a.pos.y += d.y * step;
    } else {
        if (a.pathVersion != app.version) {
            ff::Path p = ff::astar(app.grid, c, app.goal);
            a.path = std::move(p.cells);
            a.next = a.path.size() > 1 ? 1 : a.path.size();  // index 0 is the cell it already stands in
            a.pathVersion = app.version;
        }
        if (a.next < a.path.size()) {
            const ff::Cell t = a.path[a.next];
            const double dx = t.x + 0.5 - a.pos.x;
            const double dy = t.y + 0.5 - a.pos.y;
            const double len = std::hypot(dx, dy);
            if (len < 1e-9) {
                ++a.next;
            } else {
                const double s = std::min(len, step);
                a.pos.x += dx / len * s;
                a.pos.y += dy / len * s;
                if (s >= len) ++a.next;
            }
        }
    }
    a.pos = ff::pushOutOfWalls(app.grid, a.pos, kAgentRadius);
}

void update(App& app, double dt) {
    const int steps = std::max(1, static_cast<int>(std::ceil(dt / kMaxStep)));
    const double h = dt / static_cast<double>(steps);
    for (int i = 0; i < steps; ++i) {
        for (Agent& a : app.agents) advance(app, a, h);
    }
    app.reveal = std::min(app.reveal + kRevealSpeed * dt, static_cast<double>(app.field.maxDist) + 2.0);
    if (app.statsDirty && app.frames - app.changedFrame >= kStatsDelayFrames) refreshStats(app);
}

/// 1 px lines are thin on a scaled canvas, so a second line sits next to the first.
void line2(SDL_Renderer* r, float x0, float y0, float x1, float y1, gd::Color c) {
    gd::setColor(r, c);
    const bool flat = std::fabs(x1 - x0) > std::fabs(y1 - y0);
    SDL_RenderLine(r, x0, y0, x1, y1);
    SDL_RenderLine(r, x0 + (flat ? 0.0F : 1.0F), y0 + (flat ? 1.0F : 0.0F), x1 + (flat ? 0.0F : 1.0F), y1 + (flat ? 1.0F : 0.0F));
}

void drawArrow(SDL_Renderer* r, float cx, float cy, ff::Vec2 d, gd::Color c) {
    const auto dx = static_cast<float>(d.x);
    const auto dy = static_cast<float>(d.y);
    const float tipX = cx + dx * 6.0F;
    const float tipY = cy + dy * 6.0F;
    line2(r, cx - dx * 6.0F, cy - dy * 6.0F, tipX, tipY, c);
    gd::setColor(r, c);
    SDL_RenderLine(r, tipX, tipY, tipX - dx * 4.0F - dy * 2.5F, tipY - dy * 4.0F + dx * 2.5F);
    SDL_RenderLine(r, tipX, tipY, tipX - dx * 4.0F + dy * 2.5F, tipY - dy * 4.0F - dx * 2.5F);
}

void drawGrid(App& app) {
    SDL_Renderer* r = app.renderer;
    const bool flow = app.follow == Follow::Flow;
    gd::fillRect(r, 0.0F, kGridTop, static_cast<float>(kWidth), kGridBottom - kGridTop, gd::kWhite);

    if (app.heat) {
        const double maxD = static_cast<double>(std::max(1, app.field.maxDist));
        for (int y = 0; y < kRows; ++y) {
            for (int x = 0; x < kCols; ++x) {
                const int d = app.field.at(x, y);
                if (d == ff::kUnreached || static_cast<double>(d) > app.reveal) continue;
                gd::Color col = mix(gd::kTealSoft, gd::kTealDark, 0.75 * static_cast<double>(d) / maxD);
                if ((d / 4) % 2 == 1) col = mix(col, gd::kWhite, 0.18);  // contour bands, four steps wide
                if (app.reveal - static_cast<double>(d) < 1.5) col = mix(col, gd::kOrange, 0.7);  // the front of the wave
                gd::fillRect(r, cellX(x), cellY(y), kCell, kCell, withAlpha(col, flow ? 1.0 : 0.45));
            }
        }
    }

    gd::setColor(r, gd::kGrid);
    for (int x = 0; x <= kCols; ++x) SDL_RenderLine(r, cellX(x), kGridTop, cellX(x), kGridBottom);
    for (int y = 0; y <= kRows; ++y) SDL_RenderLine(r, 0.0F, cellY(y), static_cast<float>(kWidth), cellY(y));

    for (int y = 0; y < kRows; ++y) {
        for (int x = 0; x < kCols; ++x) {
            if (app.grid.blocked(x, y)) gd::fillRect(r, cellX(x), cellY(y), kCell, kCell, gd::kInk);
        }
    }

    if (app.arrows) {
        const gd::Color col = withAlpha(gd::kInk, flow ? 0.75 : 0.25);
        for (int y = 0; y < kRows; ++y) {
            for (int x = 0; x < kCols; ++x) {
                const int d = app.field.at(x, y);
                if (d <= 0 || static_cast<double>(d) > app.reveal) continue;
                const ff::Vec2 dir = app.dirs[app.grid.index(x, y)];
                if (dir.x == 0.0 && dir.y == 0.0) continue;
                drawArrow(r, cellX(x) + kCell / 2.0F, cellY(y) + kCell / 2.0F, dir, col);
            }
        }
    }

    // The goal.
    const float gx = cellX(app.goal.x) + kCell / 2.0F;
    const float gy = cellY(app.goal.y) + kCell / 2.0F;
    gd::disc(r, gx, gy, 8.0F, gd::kOrange);
    gd::ring(r, gx, gy, 8.0F, gd::kInk);
    gd::ring(r, gx, gy, 9.0F, gd::kInk);

    // A* mode: the remaining route of the first few agents, each one has its own.
    if (!flow) {
        for (std::size_t i = 0; i < std::min<std::size_t>(5, app.agents.size()); ++i) {
            const Agent& a = app.agents[i];
            float px = static_cast<float>(a.pos.x) * kCell;
            float py = kGridTop + static_cast<float>(a.pos.y) * kCell;
            for (std::size_t k = a.next; k < a.path.size(); ++k) {
                const float qx = cellX(a.path[k].x) + kCell / 2.0F;
                const float qy = cellY(a.path[k].y) + kCell / 2.0F;
                line2(r, px, py, qx, qy, withAlpha(gd::kOrange, 0.8));
                px = qx;
                py = qy;
            }
        }
    }

    // The agents.
    for (const Agent& a : app.agents) {
        const float ax = static_cast<float>(a.pos.x) * kCell;
        const float ay = kGridTop + static_cast<float>(a.pos.y) * kCell;
        gd::fillRect(r, ax - 4.0F, ay - 4.0F, 8.0F, 8.0F, gd::kInk);
        gd::fillRect(r, ax - 3.0F, ay - 3.0F, 6.0F, 6.0F, flow ? gd::kWhite : gd::kOrange);
    }
}

void drawStats(App& app) {
    SDL_Renderer* r = app.renderer;
    constexpr float kCardY = 360.0F;
    constexpr float kBarX = 190.0F;
    constexpr float kBarMax = 380.0F;
    gd::card(r, 24.0F, kCardY, 912.0F, 74.0F, gd::kWhite, gd::kInk);
    app.hud.text(r, 38.0F, kCardY + 4.0F, "cells expanded after the last change of goal or walls", gd::kMuted);

    const long long maxWork = std::max<long long>({1, app.bfsWork, app.astarWork.expanded});
    const auto bar = [&](float y, const char* label, long long work, const std::string& value, gd::Color color, bool highlight) {
        app.hud.text(r, 38.0F, y, label, highlight ? gd::kInk : gd::kMuted, gd::Hud::Style::Bold);
        const float w = work > 0 ? std::max(3.0F, kBarMax * static_cast<float>(work) / static_cast<float>(maxWork)) : 0.0F;
        gd::fillRect(r, kBarX, y + 3.0F, w, 16.0F, color);
        gd::strokeRect(r, kBarX, y + 3.0F, w, 16.0F, gd::kInk);
        app.hud.text(r, kBarX + w + 10.0F, y, value, gd::kInk);
    };
    const bool flow = app.follow == Follow::Flow;
    bar(kCardY + 26.0F, "Flow field", app.bfsWork, ff::grouped(app.bfsWork) + " cells + " + std::to_string(app.agents.size()) + " lookups/frame", gd::kTeal, flow);
    bar(kCardY + 48.0F, "A* per agent", app.astarWork.expanded, ff::grouped(app.astarWork.expanded) + " cells in " + std::to_string(app.astarWork.searches) + " searches", gd::kOrange, !flow);
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    app.hud.titleBar(r, kWidth, "[ FLOW FIELDS ]", app.follow == Follow::Flow ? "one search from the goal, agents read arrows" : "every agent searches for itself");
    drawGrid(app);
    drawStats(app);

    app.hud.text(r, 24.0F, 444.0F + 4.0F, "Click/drag:", gd::kMuted);
    app.hud.text(r, 24.0F, 482.0F + 4.0F, "Agents follow:", gd::kMuted);
    for (const Chip& c : app.chips) {
        const bool on = active(app, c.action);
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? gd::kTeal : gd::kWhite, gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 4.0F, c.label, on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
    }
    app.hud.text(r, 24.0F, 518.0F, "1: click or drag moves the goal   2: drag paints walls, start on a wall to erase", gd::kMuted);
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
    if (!SDL_CreateWindowAndRenderer("flow-field", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, gd::kLogicalPresentation);
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderVSync(app->renderer, 1);
    layoutChips(*app);
    resetMap(*app);
    app->reveal = 3.0;
    refreshStats(*app);
    app->lastCounter = SDL_GetPerformanceCounter();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    SDL_ConvertEventToRenderCoordinates(static_cast<App*>(appstate)->renderer, event);
    auto& app = *static_cast<App*>(appstate);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION: {
            if (!app.brushing) break;
            const ff::Vec2 p = toCellUnits({event->motion.x, event->motion.y});
            if (app.tool == Tool::Goal) {
                setGoalAt(app, p);
            } else {
                brushWalls(app, app.brushLast, p);
            }
            app.brushLast = p;
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const SDL_FPoint pt{event->button.x, event->button.y};
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&pt, &c.rect)) {
                    apply(app, c.action);
                    return SDL_APP_CONTINUE;
                }
            }
            if (pt.y < kGridTop || pt.y >= kGridBottom) break;
            const ff::Vec2 p = toCellUnits(pt);
            app.brushing = true;
            app.brushLast = p;
            if (app.tool == Tool::Goal) {
                app.reveal = 0.0;  // play the wave again
                setGoalAt(app, p);
            } else {
                const ff::Cell c = ff::cellOf(p);
                app.brushValue = !app.grid.blocked(c.x, c.y);  // start on a wall: erase, otherwise paint
                brushWalls(app, p, p);
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event->button.button == SDL_BUTTON_LEFT) app.brushing = false;
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
        case SDL_EVENT_WINDOW_FOCUS_LOST: app.brushing = false; break;
        case SDL_EVENT_KEY_DOWN:
            if (event->key.repeat) break;
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: apply(app, Action::ToolGoal); break;
                case SDLK_2: apply(app, Action::ToolWalls); break;
                case SDLK_3: apply(app, Action::Heat); break;
                case SDLK_4: apply(app, Action::Arrows); break;
                case SDLK_5: apply(app, Action::FollowFlow); break;
                case SDLK_6: apply(app, Action::FollowAstar); break;
                case SDLK_R: apply(app, Action::Respawn); break;
                case SDLK_C: apply(app, Action::ClearWalls); break;
                case SDLK_M: apply(app, Action::ResetMap); break;
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
    const double dt = std::clamp(static_cast<double>(now - app.lastCounter) / static_cast<double>(SDL_GetPerformanceFrequency()), 0.0, 0.25);
    app.lastCounter = now;
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
