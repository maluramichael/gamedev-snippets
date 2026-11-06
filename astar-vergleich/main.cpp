// Pathfinding on a grid with SDL3 main callbacks.
//
// Four searches on the same map: breadth first, Dijkstra, greedy best first and A*. They are one loop in logic.hpp
// that only differs in what decides which waiting cell goes next, and the demo shows exactly that: teal cells are
// expanded, orange cells are waiting (the frontier), the dark line is the path found. Paint walls and mud, drag the
// start S and the goal G, and watch the counters. The map at the start has a bucket that opens towards S, which is
// where greedy search gets lost, and a mud patch on the straight line, which BFS and greedy walk right through.
// The weight slider scales the heuristic of A*: 0 is Dijkstra, 1 is plain A*, anything above 1 overestimates and
// gives up the guarantee for the cheapest path.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <string>
#include <vector>

#include "hud.hpp"
#include "logic.hpp"

namespace {

constexpr int kWidth = 960;
constexpr int kHeight = 540;
constexpr int kCols = 46;
constexpr int kRows = 18;
constexpr float kCellSize = 20.0F;
constexpr float kGridX = 20.0F;
constexpr float kGridY = 34.0F;
constexpr float kStatsY = 400.0F;
constexpr float kChipH = 32.0F;
constexpr float kRow1Y = 430.0F;
constexpr float kRow2Y = 472.0F;
constexpr float kHintY = 512.0F;
constexpr double kStepsPerSecond = 500.0;
constexpr int kMaxStepsPerFrame = 120;
constexpr double kMaxDt = 0.25;

constexpr gd::Color kClosedTint{13, 148, 136, 95};
constexpr gd::Color kOpenTint{234, 88, 12, 200};
constexpr gd::Color kMudColor{228, 208, 170};
constexpr gd::Color kMudMark{150, 112, 64};

enum class Action { Bfs, Dijkstra, Greedy, AStar, Diagonals, BrushWall, BrushMud, BrushErase, Clear, Reset, Replay };
enum class Brush { Wall, Mud, Erase };
enum class Drag { None, Start, Goal, Paint, Slider };

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
    SDL_FRect sliderTrack{};
    pf::Preset map = pf::presetMap();
    pf::Algo algo = pf::Algo::AStar;
    bool diagonals = false;
    double weight = 1.0;
    Brush brush = Brush::Wall;
    std::optional<pf::Search> search;
    std::optional<double> optimum;  // cost of the cheapest path (Dijkstra), nothing when there is no path
    double stepAccumulator = 0.0;
    Drag drag = Drag::None;
    std::uint8_t paintValue = pf::kWall;
    std::optional<pf::Cell> lastPainted;
    SDL_FPoint mouse{-100.0F, -100.0F};
    Uint64 lastTicks = 0;
    long maxFrames = -1;
    long frames = 0;
};

/// Starts a fresh search on the current map. `instant` finishes it at once (editing the map), otherwise it animates.
void restart(App& app, bool instant) {
    app.search.emplace(app.map.grid, app.map.start, app.map.goal, pf::Options{app.algo, app.diagonals, app.weight});
    app.stepAccumulator = 0.0;
    pf::Search reference(app.map.grid, app.map.start, app.map.goal, pf::Options{pf::Algo::Dijkstra, app.diagonals, 0.0});
    reference.runToEnd();
    app.optimum = reference.status() == pf::Status::Found ? std::optional<double>(pf::pathCost(app.map.grid, reference.path())) : std::nullopt;
    if (instant) app.search->runToEnd();
}

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
        float gapBefore;
        float y;
    };
    const std::array<Def, 11> defs{{{Action::Bfs, "1 BFS", 0.0F, kRow1Y},
                                    {Action::Dijkstra, "2 Dijkstra", 8.0F, kRow1Y},
                                    {Action::Greedy, "3 Greedy", 8.0F, kRow1Y},
                                    {Action::AStar, "4 A*", 8.0F, kRow1Y},
                                    {Action::Diagonals, "D Diagonals", 24.0F, kRow1Y},
                                    {Action::BrushWall, "W Wall", 0.0F, kRow2Y},
                                    {Action::BrushMud, "M Mud", 8.0F, kRow2Y},
                                    {Action::BrushErase, "E Erase", 8.0F, kRow2Y},
                                    {Action::Clear, "C Clear", 24.0F, kRow2Y},
                                    {Action::Reset, "R Reset map", 8.0F, kRow2Y},
                                    {Action::Replay, "Space Replay", 24.0F, kRow2Y}}};
    float x = 40.0F;
    float lastY = 0.0F;
    for (const Def& d : defs) {
        if (d.y != lastY) x = 40.0F;  // a new row starts at the left edge
        lastY = d.y;
        x += d.gapBefore;
        const float w = app.hud.width(d.label, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({d.action, d.label, SDL_FRect{x, d.y, w, kChipH}});
        x += w;
    }
    // The slider sits behind the diagonals chip.
    const SDL_FRect diag = app.chips[4].rect;
    app.sliderTrack = SDL_FRect{diag.x + diag.w + 24.0F + app.hud.width("Weight") + 14.0F, kRow1Y + 12.0F, 170.0F, 8.0F};
}

void setAlgo(App& app, pf::Algo algo) {
    app.algo = algo;
    restart(app, false);
}

void apply(App& app, Action action) {
    switch (action) {
        case Action::Bfs: setAlgo(app, pf::Algo::Bfs); break;
        case Action::Dijkstra: setAlgo(app, pf::Algo::Dijkstra); break;
        case Action::Greedy: setAlgo(app, pf::Algo::Greedy); break;
        case Action::AStar: setAlgo(app, pf::Algo::AStar); break;
        case Action::Diagonals:
            app.diagonals = !app.diagonals;
            restart(app, false);
            break;
        case Action::BrushWall: app.brush = Brush::Wall; break;
        case Action::BrushMud: app.brush = Brush::Mud; break;
        case Action::BrushErase: app.brush = Brush::Erase; break;
        case Action::Clear:
            app.map.grid.clear();
            restart(app, true);
            break;
        case Action::Reset:
            app.map = pf::presetMap();
            restart(app, true);
            break;
        case Action::Replay: restart(app, false); break;
    }
}

bool active(const App& app, Action action) {
    switch (action) {
        case Action::Bfs: return app.algo == pf::Algo::Bfs;
        case Action::Dijkstra: return app.algo == pf::Algo::Dijkstra;
        case Action::Greedy: return app.algo == pf::Algo::Greedy;
        case Action::AStar: return app.algo == pf::Algo::AStar;
        case Action::Diagonals: return app.diagonals;
        case Action::BrushWall: return app.brush == Brush::Wall;
        case Action::BrushMud: return app.brush == Brush::Mud;
        case Action::BrushErase: return app.brush == Brush::Erase;
        default: return false;
    }
}

std::uint8_t brushValue(Brush b) { return b == Brush::Wall ? pf::kWall : b == Brush::Mud ? pf::kMud : pf::kGrass; }

std::optional<pf::Cell> cellAt(SDL_FPoint p) {
    return pf::pickCell(static_cast<double>(p.x), static_cast<double>(p.y), static_cast<double>(kGridX), static_cast<double>(kGridY), static_cast<double>(kCellSize), kCols, kRows);
}

void setWeight(App& app, double w) {
    const double snapped = pf::snapWeight(w);
    if (snapped == app.weight) return;
    app.weight = snapped;
    if (app.algo == pf::Algo::AStar) restart(app, true);  // live while the slider moves
}

void dragSlider(App& app, float mouseX) {
    setWeight(app, pf::weightFromT(static_cast<double>((mouseX - app.sliderTrack.x) / app.sliderTrack.w)));
}

/// Paints one cell. Start and goal keep their cells free. Returns true when the map changed.
bool paintCell(App& app, pf::Cell c) {
    if (c == app.map.start || c == app.map.goal) return false;
    if (app.map.grid.at(c.x, c.y) == app.paintValue) return false;
    app.map.grid.set(c.x, c.y, app.paintValue);
    return true;
}

void paintTo(App& app, pf::Cell target) {
    bool changed = false;
    const std::vector<pf::Cell> cells = app.lastPainted ? pf::lineCells(*app.lastPainted, target) : std::vector<pf::Cell>{target};
    for (const pf::Cell& c : cells) changed = paintCell(app, c) || changed;
    app.lastPainted = target;
    if (changed) restart(app, true);
}

void moveEndpoint(App& app, pf::Cell& which, const pf::Cell& other, pf::Cell target) {
    if (target == which || target == other || !app.map.grid.passable(target.x, target.y)) return;
    which = target;
    restart(app, true);
}

void endDrag(App& app) {
    app.drag = Drag::None;
    app.lastPainted.reset();
}

/// 1 px lines are too thin on a 960x540 canvas that is scaled up, so draw a few side by side.
void thickLine(SDL_Renderer* r, float x0, float y0, float x1, float y1, gd::Color c, int half) {
    gd::setColor(r, c);
    const bool flat = std::fabs(x1 - x0) > std::fabs(y1 - y0);
    for (int o = -half; o <= half; ++o) {
        const auto off = static_cast<float>(o);
        SDL_RenderLine(r, x0 + (flat ? 0.0F : off), y0 + (flat ? off : 0.0F), x1 + (flat ? 0.0F : off), y1 + (flat ? off : 0.0F));
    }
}

SDL_FPoint centerOf(pf::Cell c) { return {kGridX + (static_cast<float>(c.x) + 0.5F) * kCellSize, kGridY + (static_cast<float>(c.y) + 0.5F) * kCellSize}; }

void drawMap(App& app) {
    SDL_Renderer* r = app.renderer;
    const float gw = kCols * kCellSize;
    const float gh = kRows * kCellSize;
    gd::fillRect(r, kGridX, kGridY, gw, gh, gd::kWhite);
    for (int x = 0; x <= kCols; ++x) gd::fillRect(r, kGridX + static_cast<float>(x) * kCellSize, kGridY, 1.0F, gh, gd::kGrid);
    for (int y = 0; y <= kRows; ++y) gd::fillRect(r, kGridX, kGridY + static_cast<float>(y) * kCellSize, gw, 1.0F, gd::kGrid);

    const pf::Search& s = *app.search;
    for (int y = 0; y < kRows; ++y) {
        for (int x = 0; x < kCols; ++x) {
            const float px = kGridX + static_cast<float>(x) * kCellSize;
            const float py = kGridY + static_cast<float>(y) * kCellSize;
            const std::uint8_t terrain = app.map.grid.at(x, y);
            if (terrain == pf::kWall) {
                gd::fillRect(r, px, py, kCellSize, kCellSize, gd::kInk);
                continue;
            }
            if (terrain == pf::kMud) {
                gd::fillRect(r, px + 1.0F, py + 1.0F, kCellSize - 1.0F, kCellSize - 1.0F, kMudColor);
                app.hud.text(r, px + 5.0F, py - 1.0F, "~", kMudMark);
            }
            const pf::CellState st = s.stateAt(x, y);
            if (st == pf::CellState::Closed) gd::fillRect(r, px + 1.0F, py + 1.0F, kCellSize - 1.0F, kCellSize - 1.0F, kClosedTint);
            if (st == pf::CellState::Open) gd::fillRect(r, px + 1.0F, py + 1.0F, kCellSize - 1.0F, kCellSize - 1.0F, kOpenTint);
        }
    }
    gd::strokeRect(r, kGridX, kGridY, gw, gh, gd::kInk);

    if (const auto last = s.lastExpanded(); last && s.status() == pf::Status::Running) {
        gd::strokeRect(r, kGridX + static_cast<float>(last->x) * kCellSize, kGridY + static_cast<float>(last->y) * kCellSize, kCellSize + 1.0F, kCellSize + 1.0F, gd::kInk);
    }

    // The path, from cell center to cell center.
    const std::vector<pf::Cell>& path = s.path();
    for (std::size_t i = 1; i < path.size(); ++i) {
        const SDL_FPoint a = centerOf(path[i - 1]);
        const SDL_FPoint b = centerOf(path[i]);
        thickLine(r, a.x, a.y, b.x, b.y, gd::kInk, 1);
    }

    // The cell under the cursor.
    if (const auto hover = cellAt(app.mouse)) {
        gd::strokeRect(r, kGridX + static_cast<float>(hover->x) * kCellSize, kGridY + static_cast<float>(hover->y) * kCellSize, kCellSize + 1.0F, kCellSize + 1.0F, gd::kTealDark);
    }

    // Start and goal on top.
    const SDL_FPoint sp = centerOf(app.map.start);
    gd::disc(r, sp.x, sp.y, 9.0F, gd::kTealDark);
    gd::ring(r, sp.x, sp.y, 9.0F, gd::kInk);
    app.hud.text(r, sp.x - 5.0F, sp.y - 11.0F, "S", gd::kWhite, gd::Hud::Style::Bold);
    const SDL_FPoint gp = centerOf(app.map.goal);
    gd::disc(r, gp.x, gp.y, 9.0F, gd::kOrange);
    gd::ring(r, gp.x, gp.y, 9.0F, gd::kInk);
    app.hud.text(r, gp.x - 5.0F, gp.y - 11.0F, "G", gd::kWhite, gd::Hud::Style::Bold);
}

std::string rule(const App& app) {
    char buf[64];
    switch (app.algo) {
        case pf::Algo::Bfs: return "BFS: first in, first out";
        case pf::Algo::Dijkstra: return "Dijkstra: lowest g first";
        case pf::Algo::Greedy: return "Greedy: lowest h first";
        case pf::Algo::AStar: SDL_snprintf(buf, sizeof buf, "A*: lowest g + %.1f h first", app.weight); return buf;
    }
    return "";
}

void drawStats(App& app) {
    SDL_Renderer* r = app.renderer;
    const pf::Search& s = *app.search;
    char head[96];
    SDL_snprintf(head, sizeof head, "expanded %zu · frontier %zu · ", s.expanded(), s.frontier());
    app.hud.text(r, kGridX, kStatsY, head, gd::kInk, gd::Hud::Style::Bold);
    const float x = kGridX + app.hud.width(head, gd::Hud::Style::Bold);

    char tail[96];
    gd::Color color = gd::kInk;
    if (s.status() == pf::Status::Running) {
        SDL_snprintf(tail, sizeof tail, "searching");
        color = gd::kMuted;
    } else if (s.status() == pf::Status::NoPath) {
        SDL_snprintf(tail, sizeof tail, "no path");
        color = gd::kOrange;
    } else {
        const double cost = pf::pathCost(app.map.grid, s.path());
        if (!app.optimum || cost <= *app.optimum + 1e-6) {
            SDL_snprintf(tail, sizeof tail, "cost %.1f (optimal)", cost);
            color = gd::kTealDark;
        } else {
            SDL_snprintf(tail, sizeof tail, "cost %.1f (optimal %.1f, +%.0f%%)", cost, *app.optimum, (cost / *app.optimum - 1.0) * 100.0);
            color = gd::kOrange;
        }
    }
    app.hud.text(r, x, kStatsY, tail, color, gd::Hud::Style::Bold);

    // Legend.
    float lx = 660.0F;
    const auto item = [&](const char* label, gd::Color fill, bool line) {
        if (line) {
            gd::fillRect(r, lx, kStatsY + 9.0F, 14.0F, 3.0F, fill);
        } else {
            gd::fillRect(r, lx, kStatsY + 4.0F, 14.0F, 14.0F, fill);
            gd::strokeRect(r, lx, kStatsY + 4.0F, 14.0F, 14.0F, gd::kInk);
        }
        app.hud.text(r, lx + 20.0F, kStatsY, label, gd::kMuted);
        lx += 20.0F + app.hud.width(label) + 16.0F;
    };
    item("expanded", gd::Color{190, 228, 224}, false);
    item("frontier", gd::Color{244, 150, 100}, false);
    item("path", gd::kInk, true);
}

void drawControls(App& app) {
    SDL_Renderer* r = app.renderer;
    for (const Chip& c : app.chips) {
        const bool on = active(app, c.action);
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? gd::kTeal : gd::kWhite, gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 5.0F, c.label, on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
    }

    // Weight slider. Only A* uses it, so the other searches show it dimmed.
    const bool live = app.algo == pf::Algo::AStar;
    const SDL_FRect t = app.sliderTrack;
    app.hud.text(r, t.x - 14.0F - app.hud.width("Weight"), t.y - 7.0F, "Weight", live ? gd::kInk : gd::kMuted, gd::Hud::Style::Bold);
    gd::fillRect(r, t.x, t.y, t.w, t.h, gd::kGrid);
    gd::strokeRect(r, t.x, t.y, t.w, t.h, live ? gd::kInk : gd::kMuted);
    const float tickX = t.x + static_cast<float>(pf::tFromWeight(1.0)) * t.w;
    gd::fillRect(r, tickX - 1.0F, t.y - 5.0F, 3.0F, t.h + 10.0F, gd::kTeal);  // 1.0 is plain A*
    const float thumbX = t.x + static_cast<float>(pf::tFromWeight(app.weight)) * t.w;
    gd::card(r, thumbX - 6.0F, t.y - 8.0F, 12.0F, t.h + 16.0F, live ? gd::kOrange : gd::kMuted, gd::kInk);
    char value[16];
    SDL_snprintf(value, sizeof value, "%.1f", app.weight);
    app.hud.text(r, t.x + t.w + 16.0F, t.y - 7.0F, value, live ? gd::kInk : gd::kMuted, gd::Hud::Style::Bold);

    app.hud.text(r, kGridX, kHintY, "paint: drag   erase: right drag   move S and G: drag them   weight: [ ]", gd::kMuted);
}

void drawScene(App& app) {
    gd::backdrop(app.renderer, kWidth, kHeight);
    const std::string title = rule(app) + (app.diagonals ? " · 8 directions" : " · 4 directions");
    app.hud.titleBar(app.renderer, kWidth, "[ PATHFINDING ]", title);
    drawMap(app);
    drawStats(app);
    drawControls(app);
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
    if (!SDL_CreateWindowAndRenderer("astar-vergleich", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
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
    restart(*app, true);  // the first frame already shows a finished search
    app->lastTicks = SDL_GetTicksNS();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION: {
            app.mouse = {event->motion.x, event->motion.y};
            const auto cell = cellAt(app.mouse);
            if (app.drag == Drag::Slider) {
                dragSlider(app, app.mouse.x);
            } else if (app.drag == Drag::Paint) {
                if (cell) {
                    paintTo(app, *cell);
                } else {
                    app.lastPainted.reset();
                }
            } else if (app.drag == Drag::Start && cell) {
                moveEndpoint(app, app.map.start, app.map.goal, *cell);
            } else if (app.drag == Drag::Goal && cell) {
                moveEndpoint(app, app.map.goal, app.map.start, *cell);
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            const SDL_FPoint p{event->button.x, event->button.y};
            app.mouse = p;
            if (app.drag != Drag::None) break;
            const bool left = event->button.button == SDL_BUTTON_LEFT;
            const bool right = event->button.button == SDL_BUTTON_RIGHT;
            if (!left && !right) break;
            if (left) {
                for (const Chip& c : app.chips) {
                    if (SDL_PointInRectFloat(&p, &c.rect)) {
                        apply(app, c.action);
                        return SDL_APP_CONTINUE;
                    }
                }
                const SDL_FRect grab{app.sliderTrack.x - 10.0F, app.sliderTrack.y - 10.0F, app.sliderTrack.w + 20.0F, app.sliderTrack.h + 20.0F};
                if (SDL_PointInRectFloat(&p, &grab)) {
                    app.drag = Drag::Slider;
                    dragSlider(app, p.x);
                    break;
                }
            }
            const auto cell = cellAt(p);
            if (!cell) break;
            if (left && *cell == app.map.start) {
                app.drag = Drag::Start;
            } else if (left && *cell == app.map.goal) {
                app.drag = Drag::Goal;
            } else {
                app.drag = Drag::Paint;
                app.paintValue = right ? pf::kGrass : brushValue(app.brush);
                paintTo(app, *cell);
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP: endDrag(app); break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
        case SDL_EVENT_WINDOW_FOCUS_LOST: endDrag(app); break;
        case SDL_EVENT_KEY_DOWN: {
            const bool fresh = !event->key.repeat;
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: if (fresh) apply(app, Action::Bfs); break;
                case SDLK_2: if (fresh) apply(app, Action::Dijkstra); break;
                case SDLK_3: if (fresh) apply(app, Action::Greedy); break;
                case SDLK_4: if (fresh) apply(app, Action::AStar); break;
                case SDLK_D: if (fresh) apply(app, Action::Diagonals); break;
                case SDLK_W: apply(app, Action::BrushWall); break;
                case SDLK_M: apply(app, Action::BrushMud); break;
                case SDLK_E: apply(app, Action::BrushErase); break;
                case SDLK_C: if (fresh) apply(app, Action::Clear); break;
                case SDLK_R: if (fresh) apply(app, Action::Reset); break;
                case SDLK_SPACE: if (fresh) apply(app, Action::Replay); break;
                case SDLK_LEFTBRACKET: setWeight(app, app.weight - pf::kWeightStep); break;  // repeat is fine here
                case SDLK_RIGHTBRACKET: setWeight(app, app.weight + pf::kWeightStep); break;
                default: break;
            }
            break;
        }
        default: break;
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void* appstate) {
    auto& app = *static_cast<App*>(appstate);
    const Uint64 now = SDL_GetTicksNS();
    const double dt = std::min(static_cast<double>(now - app.lastTicks) / 1e9, kMaxDt);
    app.lastTicks = now;
    if (app.search && app.search->status() == pf::Status::Running) {
        app.stepAccumulator += dt * kStepsPerSecond;
        int n = std::min(static_cast<int>(app.stepAccumulator), kMaxStepsPerFrame);
        app.stepAccumulator -= static_cast<double>(n);
        while (n-- > 0 && app.search->step()) {
        }
        if (app.search->status() != pf::Status::Running) app.stepAccumulator = 0.0;
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
