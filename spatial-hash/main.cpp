// Spatial hash with SDL3 main callbacks.
//
// 2000 circles bounce around and every step needs the pairs that touch. Brute force tests all n(n-1)/2 pairs, the hash
// grid only the pairs that share a cell, the quadtree only those in the same or an enclosing node. The number that
// matters is the pair-test count: it is the same on every machine. The milliseconds next to it are measured here on
// your machine and only a footnote. Hover the field to see which cell (or node) a circle falls into, drag the slider
// to change the cell size and watch the sweet spot near the circle diameter. The big circles span several cells, which
// is what the duplicate filter is for. The structures and their edge cases live in logic.hpp and are covered by test.cpp.

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

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

#include "hud.hpp"
#include "logic.hpp"

namespace {

constexpr int kWidth = 960;
constexpr int kHeight = 540;
constexpr float kStep = 1.0F / 60.0F;
constexpr double kMaxFrame = 0.25;  // seconds, a long pause (hidden tab) must not become a burst of steps
constexpr int kMaxSteps = 2;        // per frame; when brute force is too slow the simulation runs in slow motion
constexpr float kStatsY = 378.0F;
constexpr float kRow1Y = 458.0F;
constexpr float kRow2Y = 494.0F;
constexpr float kChipH = 30.0F;
constexpr float kTrackX = 108.0F;
constexpr float kTrackW = 230.0F;
constexpr int kMaxRings = 150;  // outlines drawn for the inspected cell, a huge cell would otherwise cost thousands of lines

// SDL's performance counter only ticks in whole milliseconds in the browser build, far too coarse for a step that takes
// a few milliseconds. The browser clock itself has 0.1 ms steps.
#ifdef __EMSCRIPTEN__
double nowMs() { return emscripten_get_now(); }
#else
double nowMs() { return static_cast<double>(SDL_GetPerformanceCounter()) * 1000.0 / static_cast<double>(SDL_GetPerformanceFrequency()); }
#endif

enum class Action { Brute, Grid, Quad, N500, N1000, N2000, N4000, Reset, Big, Dedup };

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
    float captionX = 0.0F;

    sh::World world;
    sh::Mode mode = sh::Mode::Grid;
    int count = 2000;
    int cell = 12;  // near the sweet spot of the start scene, see the --report of test.cpp
    bool big = true;
    bool dedup = true;

    sh::Counters counters;
    long contacts = 0;
    double stepMs = 0.0;
    bool haveStep = false;
    double fps = 0.0;
    bool haveFps = false;
    int fpsFrames = 0;
    double fpsStart = -1.0;
    double lastFrame = -1.0;
    double accumulator = 0.0;

    float mouseX = 480.0F;  // the first frame already inspects a cell
    float mouseY = 200.0F;
    bool sliderDrag = false;

    std::vector<std::uint8_t> touching;
    std::vector<float> heat;  // 1 = touched this step, fades out (see sh::stepHeat)
    std::vector<SDL_Vertex> verts;
    std::vector<int> indices;
    std::vector<int> scratch;

    long maxFrames = -1;
    long frames = 0;
};

SDL_FRect sliderRect() { return SDL_FRect{kTrackX - 8.0F, kRow2Y, kTrackW + 16.0F, kChipH}; }

bool isOn(const App& app, Action a) {
    switch (a) {
        case Action::Brute: return app.mode == sh::Mode::Brute;
        case Action::Grid: return app.mode == sh::Mode::Grid;
        case Action::Quad: return app.mode == sh::Mode::Quad;
        case Action::N500: return app.count == 500;
        case Action::N1000: return app.count == 1000;
        case Action::N2000: return app.count == 2000;
        case Action::N4000: return app.count == 4000;
        case Action::Reset: return false;
        case Action::Big: return app.big;
        case Action::Dedup: return app.dedup;
    }
    return false;
}

/// The cell size and the duplicate filter only mean something for the grid.
bool gridOnly(Action a) { return a == Action::Dedup; }
bool enabled(const App& app, Action a) { return !gridOnly(a) || app.mode == sh::Mode::Grid; }

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
        int row;
        float gapBefore;
    };
    const std::array<Def, 10> defs{{{Action::Brute, "1 Brute force", 1, 0.0F},
                                    {Action::Grid, "2 Hash grid", 1, 8.0F},
                                    {Action::Quad, "3 Quadtree", 1, 8.0F},
                                    {Action::N500, "4 500", 1, 76.0F},
                                    {Action::N1000, "5 1000", 1, 8.0F},
                                    {Action::N2000, "6 2000", 1, 8.0F},
                                    {Action::N4000, "7 4000", 1, 8.0F},
                                    {Action::Big, "B Big bodies", 2, 0.0F},
                                    {Action::Dedup, "D Dedup", 2, 8.0F},
                                    {Action::Reset, "R Reset", 2, 8.0F}}};
    float x[3] = {0.0F, 10.0F, 424.0F};
    for (const Def& d : defs) {
        x[d.row] += d.gapBefore;
        const float w = app.hud.width(d.label, gd::Hud::Style::Bold) + 24.0F;
        if (d.action == Action::N500) app.captionX = x[d.row] - 68.0F;
        app.chips.push_back({d.action, d.label, SDL_FRect{x[d.row], d.row == 1 ? kRow1Y : kRow2Y, w, kChipH}});
        x[d.row] += w;
    }
}

void resetStats(App& app) {
    app.haveStep = false;
    app.haveFps = false;
    app.fpsFrames = 0;
    app.fpsStart = -1.0;
    app.sliderDrag = false;
}

/// One fixed step, timed. Its pair count and contacts are what the panel shows.
void runStep(App& app) {
    const double t0 = nowMs();
    app.counters = app.world.step(kStep, app.mode, app.cell, app.dedup);
    const double ms = nowMs() - t0;
    app.stepMs = app.haveStep ? app.stepMs + 0.1 * (ms - app.stepMs) : ms;
    app.haveStep = true;
    app.contacts = static_cast<long>(app.world.hits().size());
    if (app.heat.size() != app.world.bodies.size()) app.heat.assign(app.world.bodies.size(), 0.0F);
    app.touching.assign(app.world.bodies.size(), 0);
    for (const sh::Pair& p : app.world.hits()) {
        app.touching[static_cast<std::size_t>(p.a)] = 1;
        app.touching[static_cast<std::size_t>(p.b)] = 1;
    }
    for (std::size_t i = 0; i < app.heat.size(); ++i) app.heat[i] = sh::stepHeat(app.heat[i], app.touching[i] != 0, kStep);
}

void restart(App& app) {
    app.world.populate(app.count, app.big, sh::kSeed);
    app.heat.assign(app.world.bodies.size(), 0.0F);
    app.accumulator = 0.0;
    resetStats(app);
    runStep(app);
}

void changeMode(App& app, sh::Mode mode) {
    app.mode = mode;
    resetStats(app);
    runStep(app);
}

void setCell(App& app, int cell) {
    cell = std::clamp(cell, sh::kMinCell, sh::kMaxCell);
    if (cell == app.cell) return;
    app.cell = cell;
    resetStats(app);
}

void apply(App& app, Action a) {
    if (!enabled(app, a)) return;
    switch (a) {
        case Action::Brute: changeMode(app, sh::Mode::Brute); break;
        case Action::Grid: changeMode(app, sh::Mode::Grid); break;
        case Action::Quad: changeMode(app, sh::Mode::Quad); break;
        case Action::N500: app.count = 500; restart(app); break;
        case Action::N1000: app.count = 1000; restart(app); break;
        case Action::N2000: app.count = 2000; restart(app); break;
        case Action::N4000: app.count = 4000; restart(app); break;
        case Action::Reset: restart(app); break;
        case Action::Big: app.big = !app.big; restart(app); break;
        case Action::Dedup:
            app.dedup = !app.dedup;
            resetStats(app);
            break;
    }
}

bool inField(const App& app, float x, float y) {
    const sh::Rect f = app.world.bounds;
    return x >= f.x0 && x < f.x1 && y >= f.y0 && y < f.y1;
}

void sliderTo(App& app, float mx) { setCell(app, sh::cellFromT((mx - kTrackX) / kTrackW)); }

// ---- drawing ----

void drawCircles(App& app) {
    app.verts.clear();
    app.indices.clear();
    SDL_Renderer* r = app.renderer;
    for (std::size_t i = 0; i < app.world.bodies.size(); ++i) {
        const sh::Body& b = app.world.bodies[i];
        const bool isBig = b.r > sh::kBigRadius;
        const int segments = isBig ? 28 : 8;
        // Orange is reserved for the naive side, so a circle that is in contact this step turns dark instead.
        const float t = app.heat[i];
        const gd::Color from = isBig ? gd::kTealSoft : gd::kTeal;
        const gd::Color to = isBig ? gd::kTealDark : gd::kInk;
        const auto mix = [t](Uint8 a, Uint8 b) { return static_cast<Uint8>(std::lround(static_cast<float>(a) + (static_cast<float>(b) - static_cast<float>(a)) * t)); };
        const gd::Color c{mix(from.r, to.r), mix(from.g, to.g), mix(from.b, to.b)};
        const SDL_FColor fc = gd::fcolor(c, isBig ? 0.9F : 0.85F);
        const int base = static_cast<int>(app.verts.size());
        app.verts.push_back({{b.x, b.y}, fc, {0, 0}});
        for (int k = 0; k < segments; ++k) {
            const float a = 6.2831853F * static_cast<float>(k) / static_cast<float>(segments);
            app.verts.push_back({{b.x + std::cos(a) * b.r, b.y + std::sin(a) * b.r}, fc, {0, 0}});
            app.indices.insert(app.indices.end(), {base, base + 1 + k, base + 1 + (k + 1) % segments});
        }
    }
    SDL_RenderGeometry(r, nullptr, app.verts.data(), static_cast<int>(app.verts.size()), app.indices.data(), static_cast<int>(app.indices.size()));
    for (const sh::Body& b : app.world.bodies) {
        if (b.r > sh::kBigRadius) gd::ring(r, b.x, b.y, b.r, gd::kInk, 32);
    }
}

void fillCell(SDL_Renderer* r, const sh::Rect& c, gd::Color fill, gd::Color edge) {
    gd::fillRect(r, c.x0, c.y0, c.x1 - c.x0, c.y1 - c.y0, fill);
    gd::strokeRect(r, c.x0, c.y0, c.x1 - c.x0, c.y1 - c.y0, edge);
}

void ringItems(App& app, const int* items, std::size_t n, gd::Color c) {
    for (std::size_t i = 0; i < n && i < static_cast<std::size_t>(kMaxRings); ++i) {
        const sh::Body& b = app.world.bodies[static_cast<std::size_t>(items[i])];
        gd::ring(app.renderer, b.x, b.y, b.r + 2.0F, c, 16);
    }
}

/// What the mouse is over, in words for the stats panel. Also draws the highlight.
std::string drawInspect(App& app) {
    SDL_Renderer* r = app.renderer;
    const sh::Rect f = app.world.bounds;
    constexpr gd::Color kSoft{13, 148, 136, 55};
    if (app.mode == sh::Mode::Grid) {
        const sh::Grid& g = app.world.grid();
        if (app.cell >= 10) {
            for (int cx = 1; cx < g.cols(); ++cx) {
                const float x = f.x0 + static_cast<float>(cx) * g.cellSize();
                gd::setColor(r, gd::Color{17, 24, 39, 30});
                SDL_RenderLine(r, x, f.y0, x, f.y1);
            }
            for (int cy = 1; cy < g.rows(); ++cy) {
                const float y = f.y0 + static_cast<float>(cy) * g.cellSize();
                gd::setColor(r, gd::Color{17, 24, 39, 30});
                SDL_RenderLine(r, f.x0, y, f.x1, y);
            }
        }
        int cx = 0;
        int cy = 0;
        if (!g.cellAt(app.mouseX, app.mouseY, cx, cy)) return "hover the field to inspect";
        fillCell(r, g.cellRect(cx, cy), kSoft, gd::kTeal);
        const std::span<const int> items = g.cellItems(cx, cy);
        ringItems(app, items.data(), items.size(), gd::kInk);
        const long k = static_cast<long>(items.size());
        return "cell: " + sh::formatInt(k) + (k == 1 ? " item, " : " items, ") + sh::formatInt(sh::brutePairs(k)) + (sh::brutePairs(k) == 1 ? " pair" : " pairs");
    }
    if (app.mode == sh::Mode::Quad) {
        const sh::Quadtree& t = app.world.tree();
        for (std::size_t n = 0; n < t.nodeCount(); ++n) {
            const sh::Quadtree::Node& node = t.node(n);
            if (node.child >= 0) continue;
            gd::strokeRect(r, node.rect.x0, node.rect.y0, node.rect.x1 - node.rect.x0, node.rect.y1 - node.rect.y0, gd::Color{17, 24, 39, 40});
        }
        const int n = inField(app, app.mouseX, app.mouseY) ? t.nodeAt(app.mouseX, app.mouseY) : -1;
        if (n < 0) return "hover the field to inspect";
        const sh::Quadtree::Node& node = t.node(static_cast<std::size_t>(n));
        fillCell(r, node.rect, kSoft, gd::kTeal);
        t.itemsOf(n, app.scratch);
        ringItems(app, app.scratch.data(), app.scratch.size(), gd::kInk);
        const long k = static_cast<long>(app.scratch.size());
        return "depth " + std::to_string(node.depth) + " node: " + sh::formatInt(k) + (k == 1 ? " item, " : " items, ") + sh::formatInt(sh::brutePairs(k)) + (sh::brutePairs(k) == 1 ? " pair" : " pairs");
    }
    // Brute force has no regions: every circle meets every other one.
    if (!inField(app, app.mouseX, app.mouseY)) return "hover the field to inspect";
    int best = -1;
    float bestGap = 4.0F;
    for (std::size_t i = 0; i < app.world.bodies.size(); ++i) {
        const sh::Body& b = app.world.bodies[i];
        const float gap = std::hypot(b.x - app.mouseX, b.y - app.mouseY) - b.r;
        if (gap < bestGap) {
            bestGap = gap;
            best = static_cast<int>(i);
        }
    }
    if (best < 0) return "no regions, all pairs";
    const sh::Body& b = app.world.bodies[static_cast<std::size_t>(best)];
    gd::ring(r, b.x, b.y, b.r + 3.0F, gd::kOrange, 24);
    return "this circle vs " + sh::formatInt(static_cast<long>(app.world.bodies.size()) - 1) + " others";
}

void drawChip(App& app, const Chip& c) {
    SDL_Renderer* r = app.renderer;
    const bool on = isOn(app, c.action);
    const bool live = enabled(app, c.action);
    const gd::Color fill = !live ? gd::kPaper : on ? gd::kTeal : gd::kWhite;
    const gd::Color edge = live ? gd::kInk : gd::kMuted;
    const gd::Color ink = !live ? gd::kMuted : on ? gd::kWhite : gd::kInk;
    gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, fill, edge);
    app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 4.0F, c.label, ink, gd::Hud::Style::Bold);
}

void drawSlider(App& app) {
    SDL_Renderer* r = app.renderer;
    const bool live = app.mode == sh::Mode::Grid;
    const gd::Color ink = live ? gd::kInk : gd::kMuted;
    app.hud.text(r, 10.0F, kRow2Y + 4.0F, "cell size", ink, gd::Hud::Style::Bold);
    const float cy = kRow2Y + kChipH / 2.0F;
    gd::fillRect(r, kTrackX, cy - 3.0F, kTrackW, 6.0F, gd::kGrid);
    gd::strokeRect(r, kTrackX, cy - 3.0F, kTrackW, 6.0F, ink);
    const float thumb = kTrackX + sh::tFromCell(app.cell) * kTrackW;
    gd::fillRect(r, kTrackX, cy - 3.0F, thumb - kTrackX, 6.0F, live ? gd::kTeal : gd::kMuted);
    gd::card(r, thumb - 6.0F, cy - 11.0F, 12.0F, 22.0F, live ? gd::kWhite : gd::kPaper, ink);
    app.hud.text(r, kTrackX + kTrackW + 14.0F, kRow2Y + 4.0F, std::to_string(app.cell) + " px", ink, gd::Hud::Style::Bold);
}

void drawStats(App& app, const std::string& inspect) {
    SDL_Renderer* r = app.renderer;
    gd::card(r, 10.0F, kStatsY, 940.0F, 70.0F, gd::kWhite, gd::kInk);
    const bool brute = app.mode == sh::Mode::Brute;
    const gd::Color tone = brute ? gd::kOrange : gd::kTeal;
    const long n = static_cast<long>(app.world.bodies.size());
    const long all = sh::brutePairs(n);

    app.hud.text(r, 20.0F, kStatsY + 4.0F, "PAIR TESTS / STEP", gd::kMuted);
    app.hud.text(r, 20.0F, kStatsY + 25.0F, sh::formatInt(app.counters.tests), tone, gd::Hud::Style::Large);

    const double share = all > 0 ? 100.0 * static_cast<double>(app.counters.tests) / static_cast<double>(all) : 0.0;
    app.hud.text(r, 290.0F, kStatsY + 4.0F, "n(n-1)/2 = " + sh::formatInt(all), gd::kMuted);
    app.hud.text(r, 290.0F, kStatsY + 26.0F, sh::formatFixed2(share) + " % of that", tone, gd::Hud::Style::Bold);
    std::string line = "contacts " + sh::formatInt(app.contacts);
    if (app.mode == sh::Mode::Grid) line += " · skipped " + sh::formatInt(app.counters.skipped);
    app.hud.text(r, 290.0F, kStatsY + 48.0F, line, gd::kInk);

    std::string structure = "no structure to build";
    if (app.mode == sh::Mode::Grid) {
        const sh::Grid& g = app.world.grid();
        structure = "cells " + std::to_string(g.cols()) + "x" + std::to_string(g.rows()) + " · entries " + sh::formatInt(static_cast<long>(g.entries()));
    } else if (app.mode == sh::Mode::Quad) {
        const sh::Quadtree& t = app.world.tree();
        structure = "nodes " + sh::formatInt(static_cast<long>(t.nodeCount())) + " · max depth " + std::to_string(t.deepest());
    }
    app.hud.text(r, 620.0F, kStatsY + 4.0F, structure, gd::kInk);
    app.hud.text(r, 620.0F, kStatsY + 26.0F, inspect, gd::kInk, gd::Hud::Style::Bold);
    std::string timing = "measuring...";
    if (app.haveStep && app.haveFps) {
        timing = "measured " + sh::formatFixed2(app.stepMs) + " ms/step, " + std::to_string(static_cast<int>(std::lround(app.fps))) + " fps";
    }
    app.hud.text(r, 620.0F, kStatsY + 48.0F, timing, gd::kMuted);
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    const char* name = app.mode == sh::Mode::Brute ? "brute force" : app.mode == sh::Mode::Grid ? "hash grid" : "quadtree";
    app.hud.titleBar(r, kWidth, "[ SPATIAL HASH ]", name);

    const sh::Rect f = app.world.bounds;
    gd::fillRect(r, f.x0, f.y0, f.x1 - f.x0, f.y1 - f.y0, gd::kWhite);
    const std::string inspect = drawInspect(app);
    drawCircles(app);
    gd::strokeRect(r, f.x0, f.y0, f.x1 - f.x0, f.y1 - f.y0, gd::kInk);
    drawStats(app, inspect);

    app.hud.text(r, app.captionX, kRow1Y + 4.0F, "bodies", gd::kMuted);
    for (const Chip& c : app.chips) drawChip(app, c);
    drawSlider(app);
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
    if (!SDL_CreateWindowAndRenderer("spatial-hash", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
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
    restart(*app);
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION:
            app.mouseX = event->motion.x;
            app.mouseY = event->motion.y;
            if (app.sliderDrag) sliderTo(app, app.mouseX);
            break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const SDL_FPoint p{event->button.x, event->button.y};
            app.mouseX = p.x;
            app.mouseY = p.y;
            bool onChip = false;
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) {
                    apply(app, c.action);
                    onChip = true;
                    break;
                }
            }
            const SDL_FRect slider = sliderRect();
            if (!onChip && app.mode == sh::Mode::Grid && SDL_PointInRectFloat(&p, &slider)) {
                app.sliderDrag = true;
                sliderTo(app, p.x);
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event->button.button == SDL_BUTTON_LEFT) app.sliderDrag = false;
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
        case SDL_EVENT_WINDOW_FOCUS_LOST: app.sliderDrag = false; break;
        case SDL_EVENT_KEY_DOWN:
            // Only the cell size keys repeat on purpose, a held toggle key must not flicker.
            if (event->key.key == SDLK_LEFT && app.mode == sh::Mode::Grid) {
                setCell(app, sh::stepCell(app.cell, -1));
            } else if (event->key.key == SDLK_RIGHT && app.mode == sh::Mode::Grid) {
                setCell(app, sh::stepCell(app.cell, 1));
            } else if (!event->key.repeat) {
                switch (event->key.key) {
                    case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                    case SDLK_1: apply(app, Action::Brute); break;
                    case SDLK_2: apply(app, Action::Grid); break;
                    case SDLK_3: apply(app, Action::Quad); break;
                    case SDLK_4: apply(app, Action::N500); break;
                    case SDLK_5: apply(app, Action::N1000); break;
                    case SDLK_6: apply(app, Action::N2000); break;
                    case SDLK_7: apply(app, Action::N4000); break;
                    case SDLK_B: apply(app, Action::Big); break;
                    case SDLK_D: apply(app, Action::Dedup); break;
                    case SDLK_R: apply(app, Action::Reset); break;
                    default: break;
                }
            }
            break;
        default: break;
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void* appstate) {
    auto& app = *static_cast<App*>(appstate);
    const double now = nowMs();
    if (app.lastFrame < 0.0) app.lastFrame = now;
    const double dt = std::min((now - app.lastFrame) / 1000.0, kMaxFrame);
    app.lastFrame = now;
    app.accumulator = std::min(app.accumulator + dt, static_cast<double>(kStep) * kMaxSteps);
    for (int steps = 0; app.accumulator >= static_cast<double>(kStep) && steps < kMaxSteps; ++steps) {
        runStep(app);
        app.accumulator -= static_cast<double>(kStep);
    }

    if (app.fpsStart < 0.0) {
        app.fpsStart = now;
        app.fpsFrames = 0;
    }
    ++app.fpsFrames;
    if (now - app.fpsStart >= 500.0) {
        app.fps = static_cast<double>(app.fpsFrames) * 1000.0 / (now - app.fpsStart);
        app.haveFps = true;
        app.fpsFrames = 0;
        app.fpsStart = now;
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
