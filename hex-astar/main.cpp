// Hex pathfinding with SDL3 main callbacks.
//
// The same A* runs on a hex grid and on a square grid. Switch the grid and the map stays, only the neighbor function
// (6 cells instead of 4) and the distance (cube distance instead of |dx| + |dy|) change, which is what logic.hpp's
// astar() is written to show. Paint walls, forest and swamp, drag the start (A) and the goal (B), and watch which
// cells the search had to look at. "Heuristic" off makes it Dijkstra: same path, many more cells expanded.
// The search, the geometry and the picking live in logic.hpp and are covered by test.cpp.

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
constexpr float kFieldTop = 36.0F;
constexpr float kFieldBottom = 440.0F;
constexpr float kStatY = 442.0F;
constexpr float kChipY = 468.0F;
constexpr float kChipH = 34.0F;
constexpr float kHintY = 512.0F;
constexpr gd::Color kCellEdge{205, 201, 198};
constexpr gd::Color kForestFill{210, 230, 200};
constexpr gd::Color kSwampFill{226, 212, 236};
constexpr gd::Color kDigit{90, 98, 110};

enum class Action { Hex, Square, Heuristic, BrushWall, BrushForest, BrushSwamp, BrushGrass, Reset };
enum class Drag { None, Start, Goal, Paint };

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

    ha::Grid grid{ha::kCols, ha::kRows};
    ha::Topology topology = ha::Topology::Hex;
    ha::Layout layout{};
    bool heuristic = true;
    int brush = ha::kWall;
    int start = ha::kDefaultStart;
    int goal = ha::kDefaultGoal;
    ha::SearchResult star;      // with heuristic
    ha::SearchResult dijkstra;  // without, for the comparison

    Drag drag = Drag::None;
    int paintValue = ha::kWall;
    ha::Point mouse{-100.0, -100.0};
    ha::Point lastPaint{-100.0, -100.0};
    int hover = -1;

    std::vector<SDL_Vertex> verts;  // reused every frame
    std::vector<int> indices;

    long maxFrames = -1;
    long frames = 0;
};

gd::Color mix(gd::Color a, gd::Color b, float t) {
    const auto lerp = [t](Uint8 x, Uint8 y) { return static_cast<Uint8>(std::lround(static_cast<float>(x) + (static_cast<float>(y) - static_cast<float>(x)) * t)); };
    return gd::Color{lerp(a.r, b.r), lerp(a.g, b.g), lerp(a.b, b.b), 255};
}

const ha::SearchResult& shown(const App& app) { return app.heuristic ? app.star : app.dijkstra; }

void recompute(App& app) {
    app.star = ha::search(app.grid, app.topology, app.start, app.goal, true);
    app.dijkstra = ha::search(app.grid, app.topology, app.start, app.goal, false);
}

bool inField(ha::Point p) { return p.y >= static_cast<double>(kFieldTop) && p.y <= static_cast<double>(kFieldBottom); }

void updateHover(App& app) { app.hover = inField(app.mouse) ? ha::cellAt(app.layout, app.grid, app.mouse.x, app.mouse.y) : -1; }

void setTopology(App& app, ha::Topology t) {
    if (app.topology == t) return;
    app.topology = t;
    app.layout = ha::makeLayout(t, app.grid, kWidth, static_cast<double>(kFieldTop));
    app.drag = Drag::None;
    recompute(app);
    updateHover(app);
}

void resetMap(App& app) {
    ha::applyDefaultMap(app.grid);
    app.start = ha::kDefaultStart;
    app.goal = ha::kDefaultGoal;
    app.drag = Drag::None;
    recompute(app);
}

int brushValue(Action a) {
    switch (a) {
        case Action::BrushWall: return ha::kWall;
        case Action::BrushForest: return ha::kForest;
        case Action::BrushSwamp: return ha::kSwamp;
        default: return ha::kGrass;
    }
}

void apply(App& app, Action action) {
    switch (action) {
        case Action::Hex: setTopology(app, ha::Topology::Hex); break;
        case Action::Square: setTopology(app, ha::Topology::Square); break;
        case Action::Heuristic: app.heuristic = !app.heuristic; break;
        case Action::BrushWall:
        case Action::BrushForest:
        case Action::BrushSwamp:
        case Action::BrushGrass: app.brush = brushValue(action); break;
        case Action::Reset: resetMap(app); break;
    }
}

bool active(const App& app, Action action) {
    switch (action) {
        case Action::Hex: return app.topology == ha::Topology::Hex;
        case Action::Square: return app.topology == ha::Topology::Square;
        case Action::Heuristic: return app.heuristic;
        case Action::BrushWall:
        case Action::BrushForest:
        case Action::BrushSwamp:
        case Action::BrushGrass: return app.brush == brushValue(action);
        case Action::Reset: return false;
    }
    return false;
}

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
        float gapBefore;
    };
    const std::array<Def, 8> defs{{{Action::Hex, "1 Hex", 0.0F},
                                   {Action::Square, "2 Square", 8.0F},
                                   {Action::Heuristic, "H Heuristic", 24.0F},
                                   {Action::BrushWall, "W Wall", 24.0F},
                                   {Action::BrushForest, "F Forest", 8.0F},
                                   {Action::BrushSwamp, "S Swamp", 8.0F},
                                   {Action::BrushGrass, "G Grass", 8.0F},
                                   {Action::Reset, "R Reset", 24.0F}}};
    float x = 24.0F;
    for (const Def& d : defs) {
        x += d.gapBefore;
        const float w = app.hud.width(d.label, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({d.action, d.label, SDL_FRect{x, kChipY, w, kChipH}});
        x += w;
    }
}

/// Paint one cell. The start and the goal keep their cells.
void paintCell(App& app, int cell) {
    if (cell < 0 || cell == app.start || cell == app.goal || app.grid.cost(cell) == app.paintValue) return;
    app.grid.terrain[static_cast<std::size_t>(cell)] = app.paintValue;
    recompute(app);
}

/// A fast mouse skips cells between two motion events, so walk the segment in small steps.
void paintAlong(App& app, ha::Point from, ha::Point to) {
    const double length = std::hypot(to.x - from.x, to.y - from.y);
    const int steps = std::max(1, static_cast<int>(std::ceil(length / 6.0)));
    for (int i = 1; i <= steps; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(steps);
        const ha::Point p{from.x + (to.x - from.x) * t, from.y + (to.y - from.y) * t};
        if (inField(p)) paintCell(app, ha::cellAt(app.layout, app.grid, p.x, p.y));
    }
}

void moveMarker(App& app, int& marker, int other) {
    if (app.hover < 0 || app.hover == marker || app.hover == other || app.grid.cost(app.hover) == ha::kWall) return;
    marker = app.hover;
    recompute(app);
}

// ---- Drawing ----

struct Corners {
    std::array<ha::Point, 6> p{};
    int count = 0;
};

Corners cornersOf(const App& app, ha::Point c) {
    Corners out;
    if (app.topology == ha::Topology::Hex) {
        out.count = 6;
        for (int i = 0; i < 6; ++i) out.p[static_cast<std::size_t>(i)] = ha::hexCorner(c, app.layout.size, i);
    } else {
        const double h = app.layout.size / 2.0;
        out.count = 4;
        out.p = {{{c.x - h, c.y - h}, {c.x + h, c.y - h}, {c.x + h, c.y + h}, {c.x - h, c.y + h}, {0, 0}, {0, 0}}};
    }
    return out;
}

void thickLine(SDL_Renderer* r, ha::Point a, ha::Point b, gd::Color c, int half = 1) {
    gd::setColor(r, c);
    const bool flat = std::fabs(b.x - a.x) > std::fabs(b.y - a.y);
    for (int o = -half; o <= half; ++o) {
        const auto off = static_cast<float>(o);
        SDL_RenderLine(r, static_cast<float>(a.x) + (flat ? 0.0F : off), static_cast<float>(a.y) + (flat ? off : 0.0F),
                       static_cast<float>(b.x) + (flat ? 0.0F : off), static_cast<float>(b.y) + (flat ? off : 0.0F));
    }
}

void outline(SDL_Renderer* r, const Corners& k, gd::Color c, int half = 0) {
    for (int i = 0; i < k.count; ++i) {
        const ha::Point a = k.p[static_cast<std::size_t>(i)];
        const ha::Point b = k.p[static_cast<std::size_t>((i + 1) % k.count)];
        thickLine(r, a, b, c, half);
    }
}

gd::Color cellColor(const App& app, int cell) {
    const ha::SearchResult& s = shown(app);
    const int cost = app.grid.cost(cell);
    if (cost == ha::kWall) return gd::kInk;
    gd::Color base = gd::kWhite;
    if (cost == ha::kForest) base = kForestFill;
    if (cost == ha::kSwamp) base = kSwampFill;
    if (s.closed[static_cast<std::size_t>(cell)] != 0) return mix(base, gd::kTeal, 0.26F);
    if (s.costSoFar[static_cast<std::size_t>(cell)] >= 0) return mix(base, gd::kOrange, 0.36F);
    return base;
}

void drawCells(App& app) {
    app.verts.clear();
    app.indices.clear();
    const int count = app.grid.width * app.grid.height;
    for (int cell = 0; cell < count; ++cell) {
        const ha::Point c = ha::centerOf(app.layout, app.grid, cell);
        const Corners k = cornersOf(app, c);
        const SDL_FColor color = gd::fcolor(cellColor(app, cell));
        const int base = static_cast<int>(app.verts.size());
        app.verts.push_back({{static_cast<float>(c.x), static_cast<float>(c.y)}, color, {0, 0}});
        for (int i = 0; i < k.count; ++i) {
            app.verts.push_back({{static_cast<float>(k.p[static_cast<std::size_t>(i)].x), static_cast<float>(k.p[static_cast<std::size_t>(i)].y)}, color, {0, 0}});
            app.indices.insert(app.indices.end(), {base, base + 1 + i, base + 1 + (i + 1) % k.count});
        }
    }
    SDL_RenderGeometry(app.renderer, nullptr, app.verts.data(), static_cast<int>(app.verts.size()), app.indices.data(), static_cast<int>(app.indices.size()));
    for (int cell = 0; cell < count; ++cell) {
        outline(app.renderer, cornersOf(app, ha::centerOf(app.layout, app.grid, cell)), app.grid.cost(cell) == ha::kWall ? gd::kInk : kCellEdge);
    }
    char digit[8];
    for (int cell = 0; cell < count; ++cell) {
        const int cost = app.grid.cost(cell);
        if (cost != ha::kForest && cost != ha::kSwamp) continue;
        const ha::Point c = ha::centerOf(app.layout, app.grid, cell);
        SDL_snprintf(digit, sizeof digit, "%d", cost);
        app.hud.text(app.renderer, static_cast<float>(c.x) - 5.0F, static_cast<float>(c.y) - 11.0F, digit, kDigit);
    }
}

void drawMarker(App& app, int cell, gd::Color color, const char* letter) {
    const ha::Point c = ha::centerOf(app.layout, app.grid, cell);
    const auto x = static_cast<float>(c.x);
    const auto y = static_cast<float>(c.y);
    gd::disc(app.renderer, x, y, 11.0F, color);
    gd::ring(app.renderer, x, y, 11.0F, gd::kInk);
    gd::ring(app.renderer, x, y, 12.0F, gd::kInk);
    app.hud.text(app.renderer, x - 5.0F, y - 11.0F, letter, gd::kWhite, gd::Hud::Style::Bold);
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    const ha::SearchResult& s = shown(app);
    gd::backdrop(r, kWidth, kHeight);
    app.hud.titleBar(r, kWidth, "[ HEX PATHFINDING ]", app.heuristic ? "A* (heuristic on)" : "Dijkstra (heuristic off)");

    drawCells(app);

    for (std::size_t i = 1; i < s.path.size(); ++i) {
        thickLine(r, ha::centerOf(app.layout, app.grid, s.path[i - 1]), ha::centerOf(app.layout, app.grid, s.path[i]), gd::kTealDark, 1);
    }
    if (app.hover >= 0 && app.grid.cost(app.hover) != ha::kWall) {
        outline(r, cornersOf(app, ha::centerOf(app.layout, app.grid, app.hover)), gd::kInk, 1);
    }
    drawMarker(app, app.start, gd::kTeal, "A");
    drawMarker(app, app.goal, gd::kOrange, "B");

    // Numbers: both searches are computed on every change, so the comparison always matches the map.
    char line[96];
    const ha::SearchResult& other = app.heuristic ? app.dijkstra : app.star;
    const char* otherName = app.heuristic ? "Dijkstra" : "A*";
    if (s.found) {
        SDL_snprintf(line, sizeof line, "expanded %d (%s %d)  path %d  cost %d", s.expanded, otherName, other.expanded, static_cast<int>(s.path.size()) - 1, s.cost);
    } else {
        SDL_snprintf(line, sizeof line, "no path  expanded %d", s.expanded);
    }
    app.hud.text(r, 24.0F, kStatY, line, s.found ? gd::kInk : gd::kOrange, gd::Hud::Style::Bold);
    const char* readout = app.topology == ha::Topology::Hex ? "6 neighbors, dist = max(|dq|,|dr|,|ds|)" : "4 neighbors, dist = |dx|+|dy|";
    app.hud.text(r, 936.0F - app.hud.width(readout), kStatY, readout, gd::kMuted);

    for (const Chip& c : app.chips) {
        const bool on = active(app, c.action);
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? gd::kTeal : gd::kWhite, gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 6.0F, c.label, on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
    }

    // Hint line with the color legend, and what the cell under the mouse costs so far.
    const char* hint = "click/drag: paint  right: erase  drag A, B";
    app.hud.text(r, 24.0F, kHintY, hint, gd::kMuted);
    float x = 24.0F + app.hud.width(hint) + 20.0F;
    gd::fillRect(r, x, kHintY + 5.0F, 14.0F, 12.0F, mix(gd::kWhite, gd::kTeal, 0.26F));
    gd::strokeRect(r, x, kHintY + 5.0F, 14.0F, 12.0F, kCellEdge);
    app.hud.text(r, x + 20.0F, kHintY, "expanded", gd::kMuted);
    x += 20.0F + app.hud.width("expanded") + 16.0F;
    gd::fillRect(r, x, kHintY + 5.0F, 14.0F, 12.0F, mix(gd::kWhite, gd::kOrange, 0.36F));
    gd::strokeRect(r, x, kHintY + 5.0F, 14.0F, 12.0F, kCellEdge);
    app.hud.text(r, x + 20.0F, kHintY, "frontier", gd::kMuted);
    if (app.hover >= 0 && app.grid.cost(app.hover) != ha::kWall) {
        const int g = s.costSoFar[static_cast<std::size_t>(app.hover)];
        if (g < 0) {
            SDL_snprintf(line, sizeof line, "not reached");
        } else if (app.heuristic) {
            const int h = ha::distance(app.grid, app.topology, app.hover, app.goal);
            SDL_snprintf(line, sizeof line, "g %d + h %d = f %d", g, h, g + h);
        } else {
            SDL_snprintf(line, sizeof line, "g %d", g);
        }
        app.hud.text(r, 936.0F - app.hud.width(line), kHintY, line, gd::kInk, gd::Hud::Style::Bold);
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
    if (!SDL_CreateWindowAndRenderer("hex-astar", kWidth, kHeight, 0, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, SDL_LOGICAL_PRESENTATION_LETTERBOX);
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderVSync(app->renderer, 1);
    app->verts.reserve(static_cast<std::size_t>(ha::kCols * ha::kRows * 7));
    app->indices.reserve(static_cast<std::size_t>(ha::kCols * ha::kRows * 18));
    ha::applyDefaultMap(app->grid);
    app->layout = ha::makeLayout(app->topology, app->grid, kWidth, static_cast<double>(kFieldTop));
    recompute(*app);
    layoutChips(*app);
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION: {
            app.mouse = {static_cast<double>(event->motion.x), static_cast<double>(event->motion.y)};
            updateHover(app);
            if (app.drag == Drag::Start) moveMarker(app, app.start, app.goal);
            if (app.drag == Drag::Goal) moveMarker(app, app.goal, app.start);
            if (app.drag == Drag::Paint) paintAlong(app, app.lastPaint, app.mouse);
            app.lastPaint = app.mouse;
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            const SDL_FPoint p{event->button.x, event->button.y};
            app.mouse = {static_cast<double>(p.x), static_cast<double>(p.y)};
            updateHover(app);
            app.lastPaint = app.mouse;
            if (event->button.button == SDL_BUTTON_RIGHT) {
                if (app.hover >= 0) {
                    app.paintValue = ha::kGrass;
                    app.drag = Drag::Paint;
                    paintCell(app, app.hover);
                }
                break;
            }
            if (event->button.button != SDL_BUTTON_LEFT) break;
            bool onChip = false;
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) {
                    apply(app, c.action);
                    onChip = true;
                }
            }
            if (onChip || app.hover < 0) break;
            if (app.hover == app.start) {
                app.drag = Drag::Start;
            } else if (app.hover == app.goal) {
                app.drag = Drag::Goal;
            } else {
                // A click on a cell that already has this terrain removes it, so one brush draws and erases.
                app.paintValue = app.grid.cost(app.hover) == app.brush ? ha::kGrass : app.brush;
                app.drag = Drag::Paint;
                paintCell(app, app.hover);
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP: app.drag = Drag::None; break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
        case SDL_EVENT_WINDOW_FOCUS_LOST:
            app.drag = Drag::None;
            app.hover = -1;
            break;
        case SDL_EVENT_KEY_DOWN:
            if (event->key.repeat) break;
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: apply(app, Action::Hex); break;
                case SDLK_2: apply(app, Action::Square); break;
                case SDLK_H: apply(app, Action::Heuristic); break;
                case SDLK_W: apply(app, Action::BrushWall); break;
                case SDLK_F: apply(app, Action::BrushForest); break;
                case SDLK_S: apply(app, Action::BrushSwamp); break;
                case SDLK_G: apply(app, Action::BrushGrass); break;
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
