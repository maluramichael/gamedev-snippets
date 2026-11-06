// Wave function collapse (simple tiled model) with SDL3 main callbacks.
//
// Every cell starts as "any of the 29 tiles" (shown as a washed-out blend of the tiles still possible). The solver
// repeatedly takes the undecided cell with the lowest entropy, collapses it to one tile by weight and then lets the
// consequences ripple outwards: a tile may only stay if each neighbour still has a tile that fits its edge. The orange
// frames are the cells waiting for that check. The speed chips decide how many solver steps run per second, so the
// ripple can be watched or skipped. Switch the cell order to "random" to see why the order matters: contradictions pile
// up, and the demo either restarts the whole map or undoes its last decision (backtracking).
//
// The solver, the tile edges and the rules live in logic.hpp, the tile pictures in art.hpp, both without SDL and covered
// by test.cpp. This file only draws and reads input.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "art.hpp"
#include "hud.hpp"
#include "logic.hpp"

namespace {

constexpr int kWidth = 960;
constexpr int kHeight = 540;
constexpr int kCell = 30;
constexpr int kCols = 30;
constexpr int kRows = 11;
constexpr float kFieldX = 30.0F;
constexpr float kFieldY = 34.0F;
constexpr float kStripY = 370.0F;
constexpr float kStripTile = 24.0F;
constexpr float kStripStride = 31.0F;
constexpr float kStatusY = 402.0F;
constexpr float kRowA = 428.0F;
constexpr float kRowB = 460.0F;
constexpr float kChipH = 28.0F;
constexpr float kHintY = 494.0F;
constexpr float kBenchY = 516.0F;
constexpr int kTileCount = 29;
constexpr int kBenchMaps = 40;
constexpr long kMaxStepsPerFrame = 5000;  // however fast the speed chip is, one frame never runs more than this
constexpr std::array<double, 4> kStepsPerSecond{15.0, 100.0, 600.0, 1.0e6};
constexpr gd::Color kFlashColor{234, 88, 12, 255};

enum class Action {
    RunPause,
    Step,
    NewMap,
    Speed0,
    Speed1,
    Speed2,
    Speed3,
    Compare,
    OrderEntropy,
    OrderRandom,
    OrderScanline,
    Restart,
    Backtrack,
    Weights,
    Counts
};

struct Chip {
    Action action;
    std::string labelA;
    std::string labelB;
    SDL_FRect rect;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::array<SDL_Texture*, kTileCount> textures{};
    std::array<art::Rgb, kTileCount> means{};
    std::array<wfc::TileSet, 2> sets;  // 0 tuned weights, 1 uniform
    std::unique_ptr<wfc::Solver> solver;
    std::vector<Chip> chips;
    std::vector<float> flash;  // 1 right after a cell changed, fades to 0

    bool tuned = true;
    wfc::Order order = wfc::Order::Entropy;
    wfc::Recovery recovery = wfc::Recovery::Restart;
    int speed = 2;
    bool running = true;
    bool counts = true;
    double stepBudget = 0.0;
    float hold = 0.0F;  // seconds to keep a contradiction on screen before it is handled
    std::uint64_t mapIndex = 0;
    std::string benchText;

    SDL_FPoint mouse{-100.0F, -100.0F};
    Uint64 lastTicks = 0;
    long maxFrames = -1;
    long frames = 0;
};

const wfc::TileSet& activeSet(const App& app) { return app.sets[app.tuned ? 0U : 1U]; }

std::uint64_t seedFor(std::uint64_t index) { return 0x5EED0000ULL + index * 7919ULL; }

void newMap(App& app) {
    app.solver = std::make_unique<wfc::Solver>(activeSet(app), kCols, kRows, seedFor(app.mapIndex++), app.order, app.recovery);
    app.solver->trackTouched = true;
    app.flash.assign(static_cast<std::size_t>(kCols * kRows), 0.0F);
    app.stepBudget = 0.0;
    app.hold = 0.0F;
}

/// The cell under a point of the canvas, -1 outside the map.
int cellAt(SDL_FPoint p) {
    const float fx = (p.x - kFieldX) / static_cast<float>(kCell);
    const float fy = (p.y - kFieldY) / static_cast<float>(kCell);
    if (!(fx >= 0.0F) || !(fy >= 0.0F) || fx >= static_cast<float>(kCols) || fy >= static_cast<float>(kRows)) return -1;
    return static_cast<int>(fy) * kCols + static_cast<int>(fx);
}

bool buildTextures(App& app) {
    const wfc::TileSet& set = app.sets[0];
    for (std::size_t t = 0; t < set.tiles.size(); ++t) {
        const std::vector<std::uint8_t> px = art::render(set.tiles[t], kCell, static_cast<std::uint32_t>(t) * 7U + 1U);
        app.means[t] = art::mean(px);
        SDL_Texture* tex = SDL_CreateTexture(app.renderer, SDL_PIXELFORMAT_RGBA32, SDL_TEXTUREACCESS_STATIC, kCell, kCell);
        if (tex == nullptr) {
            SDL_Log("tile texture failed: %s", SDL_GetError());
            return false;
        }
        if (!SDL_UpdateTexture(tex, nullptr, px.data(), kCell * 4)) {
            SDL_Log("tile upload failed: %s", SDL_GetError());
            SDL_DestroyTexture(tex);
            return false;
        }
        SDL_SetTextureScaleMode(tex, SDL_SCALEMODE_LINEAR);
        app.textures[t] = tex;
    }
    return true;
}

// ---- chips ----

struct Def {
    Action action;
    const char* a;
    const char* b;
    float gapBefore;
};

void layoutRow(App& app, const std::vector<Def>& defs, float y) {
    float x = kFieldX;
    for (const Def& d : defs) {
        x += d.gapBefore;
        const float w = std::max(app.hud.width(d.a, gd::Hud::Style::Bold), app.hud.width(d.b, gd::Hud::Style::Bold)) + 16.0F;
        app.chips.push_back({d.action, d.a, d.b, SDL_FRect{x, y, w, kChipH}});
        x += w;
    }
}

void layoutChips(App& app) {
    layoutRow(app,
              {{Action::RunPause, "Space Pause", "Space Run", 0.0F},
               {Action::Step, ". Step", ". Step", 6.0F},
               {Action::NewMap, "N New map", "N New map", 6.0F},
               {Action::Speed0, "1 15/s", "1 15/s", 22.0F},
               {Action::Speed1, "2 100/s", "2 100/s", 6.0F},
               {Action::Speed2, "3 600/s", "3 600/s", 6.0F},
               {Action::Speed3, "4 max", "4 max", 6.0F},
               {Action::Compare, "B Compare orders", "B Compare orders", 22.0F}},
              kRowA);
    layoutRow(app,
              {{Action::OrderEntropy, "5 Entropy", "5 Entropy", 0.0F},
               {Action::OrderRandom, "6 Random", "6 Random", 6.0F},
               {Action::OrderScanline, "7 Scanline", "7 Scanline", 6.0F},
               {Action::Restart, "8 Restart", "8 Restart", 22.0F},
               {Action::Backtrack, "9 Backtrack", "9 Backtrack", 6.0F},
               {Action::Weights, "T Tuned", "T Uniform", 22.0F},
               {Action::Counts, "C Counts", "C Counts", 6.0F}},
              kRowB);
}

const char* chipLabel(const App& app, const Chip& c) {
    switch (c.action) {
        case Action::RunPause: return app.running ? c.labelA.c_str() : c.labelB.c_str();
        case Action::Weights: return app.tuned ? c.labelA.c_str() : c.labelB.c_str();
        default: return c.labelA.c_str();
    }
}

bool chipActive(const App& app, Action a) {
    switch (a) {
        case Action::RunPause: return app.running;
        case Action::Speed0: return app.speed == 0;
        case Action::Speed1: return app.speed == 1;
        case Action::Speed2: return app.speed == 2;
        case Action::Speed3: return app.speed == 3;
        case Action::OrderEntropy: return app.order == wfc::Order::Entropy;
        case Action::OrderRandom: return app.order == wfc::Order::Random;
        case Action::OrderScanline: return app.order == wfc::Order::Scanline;
        case Action::Restart: return app.recovery == wfc::Recovery::Restart;
        case Action::Backtrack: return app.recovery == wfc::Recovery::Backtrack;
        case Action::Weights: return true;  // teal for tuned, orange for uniform
        case Action::Counts: return app.counts;
        default: return false;
    }
}

/// Orange marks the naive choices, teal the ones the post recommends.
gd::Color chipFill(const App& app, Action a) {
    if (a == Action::OrderRandom || a == Action::OrderScanline) return gd::kOrange;
    if (a == Action::Weights && !app.tuned) return gd::kOrange;
    return gd::kTeal;
}

void runBenchmark(App& app) {
    const wfc::TileSet& set = activeSet(app);
    const wfc::Bench e = wfc::benchmark(set, kCols, kRows, wfc::Order::Entropy, kBenchMaps, 5);
    const wfc::Bench r = wfc::benchmark(set, kCols, kRows, wfc::Order::Random, kBenchMaps, 5);
    const wfc::Bench s = wfc::benchmark(set, kCols, kRows, wfc::Order::Scanline, kBenchMaps, 5);
    char text[160];
    SDL_snprintf(text, sizeof text, "Contradictions in %d maps: entropy %d, random %d, scanline %d (restart strategy)", kBenchMaps,
                 e.contradictions, r.contradictions, s.contradictions);
    app.benchText = text;
}

void apply(App& app, Action a) {
    switch (a) {
        case Action::RunPause:
            app.running = !app.running;
            app.stepBudget = 0.0;
            break;
        case Action::Step:
            if (app.solver && !app.solver->finished()) {
                app.solver->step();
                app.hold = 0.0F;
            }
            break;
        case Action::NewMap: newMap(app); break;
        case Action::Speed0: app.speed = 0; break;
        case Action::Speed1: app.speed = 1; break;
        case Action::Speed2: app.speed = 2; break;
        case Action::Speed3: app.speed = 3; break;
        case Action::Compare: runBenchmark(app); break;
        case Action::OrderEntropy:
        case Action::OrderRandom:
        case Action::OrderScanline:
            app.order = a == Action::OrderEntropy ? wfc::Order::Entropy : a == Action::OrderRandom ? wfc::Order::Random : wfc::Order::Scanline;
            newMap(app);
            break;
        case Action::Restart:
        case Action::Backtrack:
            app.recovery = a == Action::Restart ? wfc::Recovery::Restart : wfc::Recovery::Backtrack;
            newMap(app);
            break;
        case Action::Weights:
            app.tuned = !app.tuned;
            app.benchText.clear();  // the numbers belong to the other weights
            newMap(app);
            break;
        case Action::Counts: app.counts = !app.counts; break;
    }
}

// ---- update ----

void update(App& app, float dt) {
    for (float& f : app.flash) f = std::max(0.0F, f - dt * 2.0F);
    wfc::Solver& s = *app.solver;
    if (app.running && !s.finished()) {
        if (app.hold > 0.0F) {
            app.hold -= dt;
        } else {
            app.stepBudget += kStepsPerSecond[static_cast<std::size_t>(app.speed)] * static_cast<double>(dt);
            const long n = static_cast<long>(std::min(app.stepBudget, static_cast<double>(kMaxStepsPerFrame)));
            app.stepBudget = std::min(app.stepBudget - static_cast<double>(n), 1.0);  // the rest does not pile up
            for (long i = 0; i < n && !s.finished(); ++i) {
                s.step();
                if (s.status == wfc::Status::Contradiction) {
                    app.hold = app.speed == 3 ? 0.2F : 0.6F;  // leave the dead end on screen for a moment
                    app.stepBudget = 0.0;
                    break;
                }
            }
        }
    }
    for (int c : s.touched) app.flash[static_cast<std::size_t>(c)] = 1.0F;
    s.touched.clear();
}

// ---- drawing ----

void drawUndecided(App& app, wfc::Mask mask, float x, float y) {
    SDL_Renderer* r = app.renderer;
    if (mask == 0) {
        gd::fillRect(r, x, y, kCell, kCell, gd::kOrange);
        gd::setColor(r, gd::kInk);
        SDL_RenderLine(r, x + 5.0F, y + 5.0F, x + kCell - 5.0F, y + kCell - 5.0F);
        SDL_RenderLine(r, x + kCell - 5.0F, y + 5.0F, x + 5.0F, y + kCell - 5.0F);
        SDL_RenderLine(r, x + 6.0F, y + 5.0F, x + kCell - 4.0F, y + kCell - 5.0F);
        SDL_RenderLine(r, x + kCell - 6.0F, y + 5.0F, x + 4.0F, y + kCell - 5.0F);
        return;
    }
    const wfc::TileSet& set = activeSet(app);
    float cr = 0.0F;
    float cg = 0.0F;
    float cb = 0.0F;
    float sum = 0.0F;
    for (wfc::Mask m = mask; m != 0; m &= m - 1) {
        const auto t = static_cast<std::size_t>(wfc::lowest(m));
        const auto w = static_cast<float>(set.tiles[t].weight);
        cr += app.means[t].r * w;
        cg += app.means[t].g * w;
        cb += app.means[t].b * w;
        sum += w;
    }
    if (sum <= 0.0F) return;
    // Blend of the possible tiles, pulled towards the paper so it reads as "not decided yet".
    const float k = 0.5F;
    const auto mix = [&](float v, float paper) { return static_cast<Uint8>(std::clamp((v / sum * (1.0F - k) + paper * k) * 255.0F, 0.0F, 255.0F)); };
    gd::fillRect(r, x, y, kCell, kCell, gd::Color{mix(cr, 0.98F), mix(cg, 0.98F), mix(cb, 0.976F)});
    gd::strokeRect(r, x, y, kCell, kCell, gd::Color{214, 211, 209, 255});
    const int n = wfc::count(mask);
    if (app.counts && n >= 2 && n <= 9) {
        char digit[2] = {static_cast<char>('0' + n), 0};
        app.hud.text(r, x + 10.0F, y + 4.0F, digit, gd::kMuted);
    }
}

void drawField(App& app, int hover) {
    SDL_Renderer* r = app.renderer;
    const wfc::Solver& s = *app.solver;
    for (int cy = 0; cy < kRows; ++cy) {
        for (int cx = 0; cx < kCols; ++cx) {
            const int c = cy * kCols + cx;
            const float x = kFieldX + static_cast<float>(cx * kCell);
            const float y = kFieldY + static_cast<float>(cy * kCell);
            const wfc::Mask m = s.domain[static_cast<std::size_t>(c)];
            if (wfc::count(m) == 1) {
                const SDL_FRect dst{x, y, static_cast<float>(kCell), static_cast<float>(kCell)};
                SDL_RenderTexture(r, app.textures[static_cast<std::size_t>(wfc::lowest(m))], nullptr, &dst);
            } else {
                drawUndecided(app, m, x, y);
            }
            const float f = app.flash[static_cast<std::size_t>(c)];
            if (f > 0.0F) {
                gd::fillRect(r, x, y, kCell, kCell, gd::Color{kFlashColor.r, kFlashColor.g, kFlashColor.b, static_cast<Uint8>(f * 110.0F)});
            }
        }
    }
    // Cells waiting for their neighbours to be checked.
    for (int c : s.queue) {
        const float x = kFieldX + static_cast<float>((c % kCols) * kCell);
        const float y = kFieldY + static_cast<float>((c / kCols) * kCell);
        gd::strokeRect(r, x + 1.0F, y + 1.0F, kCell - 2.0F, kCell - 2.0F, gd::kOrange);
        gd::strokeRect(r, x + 2.0F, y + 2.0F, kCell - 4.0F, kCell - 4.0F, gd::kOrange);
    }
    // The cell the solver will collapse next.
    if (s.status == wfc::Status::Running && s.queue.empty()) {
        const int next = s.pickCell();
        if (next >= 0) {
            const float x = kFieldX + static_cast<float>((next % kCols) * kCell);
            const float y = kFieldY + static_cast<float>((next / kCols) * kCell);
            gd::strokeRect(r, x + 1.0F, y + 1.0F, kCell - 2.0F, kCell - 2.0F, gd::kTeal);
            gd::strokeRect(r, x + 2.0F, y + 2.0F, kCell - 4.0F, kCell - 4.0F, gd::kTeal);
        }
    }
    if (hover >= 0) {
        const float x = kFieldX + static_cast<float>((hover % kCols) * kCell);
        const float y = kFieldY + static_cast<float>((hover / kCols) * kCell);
        gd::strokeRect(r, x, y, kCell, kCell, gd::kInk);
        gd::strokeRect(r, x - 1.0F, y - 1.0F, kCell + 2.0F, kCell + 2.0F, gd::kInk);
    }
    gd::strokeRect(r, kFieldX - 1.0F, kFieldY - 1.0F, static_cast<float>(kCols * kCell) + 2.0F, static_cast<float>(kRows * kCell) + 2.0F, gd::kInk);
}

/// The whole tileset in a row. Hovering a cell dims the tiles that cell can no longer be; the bar underneath is the weight.
void drawStrip(App& app, int hover) {
    SDL_Renderer* r = app.renderer;
    const wfc::TileSet& set = activeSet(app);
    double maxWeight = 0.0;
    for (const wfc::Tile& t : set.tiles) maxWeight = std::max(maxWeight, t.weight);
    wfc::Mask possible = set.all;
    if (hover >= 0) possible = app.solver->domain[static_cast<std::size_t>(hover)];
    for (std::size_t t = 0; t < set.tiles.size(); ++t) {
        const float x = kFieldX + static_cast<float>(t) * kStripStride;
        const SDL_FRect dst{x, kStripY, kStripTile, kStripTile};
        SDL_RenderTexture(r, app.textures[t], nullptr, &dst);
        const bool on = (possible >> t & 1U) != 0;
        if (!on) gd::fillRect(r, x, kStripY, kStripTile, kStripTile, gd::Color{250, 250, 249, 215});
        gd::strokeRect(r, x, kStripY, kStripTile, kStripTile, on ? gd::kInk : gd::kMuted);
        const auto w = static_cast<float>(set.tiles[t].weight / maxWeight * static_cast<double>(kStripTile));
        gd::fillRect(r, x, kStripY + kStripTile + 2.0F, std::max(1.0F, w), 3.0F, on ? gd::kTeal : gd::kGrid);
    }
}

const char* stateLabel(const App& app, gd::Color& color) {
    const wfc::Status st = app.solver->status;
    if (st == wfc::Status::Done) {
        color = gd::kTeal;
        return "DONE";
    }
    if (st == wfc::Status::GaveUp) {
        color = gd::kOrange;
        return "GAVE UP";
    }
    if (st == wfc::Status::Contradiction) {
        color = gd::kOrange;
        return "CONTRADICTION";
    }
    color = app.running ? gd::kInk : gd::kMuted;
    return app.running ? "RUNNING" : "PAUSED";
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    const wfc::Solver& s = *app.solver;
    const int hover = cellAt(app.mouse);

    gd::backdrop(r, kWidth, kHeight);
    const char* orderName = app.order == wfc::Order::Entropy ? "lowest entropy first" : app.order == wfc::Order::Random ? "random cell" : "row by row";
    char right[96];
    SDL_snprintf(right, sizeof right, "%s / %s", orderName, app.recovery == wfc::Recovery::Restart ? "restart" : "backtrack");
    app.hud.titleBar(r, kWidth, "[ WAVE FUNCTION COLLAPSE ]", right);

    drawField(app, hover);
    drawStrip(app, hover);

    char line[128];
    SDL_snprintf(line, sizeof line, "decided %d/%d  steps %ld  contradictions %d  restarts %d  backtracks %d", s.collapsedCount(), s.cells(),
                 s.counters.steps, s.counters.contradictions, s.counters.restarts, s.counters.backtracks);
    app.hud.text(r, kFieldX, kStatusY, line, gd::kInk);
    gd::Color stateColor = gd::kInk;
    const char* state = stateLabel(app, stateColor);
    app.hud.text(r, static_cast<float>(kWidth) - kFieldX - app.hud.width(state, gd::Hud::Style::Bold), kStatusY, state, stateColor, gd::Hud::Style::Bold);

    for (const Chip& c : app.chips) {
        const bool on = chipActive(app, c.action);
        const char* label = chipLabel(app, c);
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? chipFill(app, c.action) : gd::kWhite, gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 3.0F, label, on ? gd::kWhite : gd::kInk,
                     gd::Hud::Style::Bold);
    }

    char hint[160];
    if (hover >= 0) {
        const wfc::Mask m = s.domain[static_cast<std::size_t>(hover)];
        const int n = wfc::count(m);
        if (n >= 2) {
            SDL_snprintf(hint, sizeof hint, "cell %d,%d: %d of %d tiles possible, entropy %.2f. Click to collapse it.", hover % kCols, hover / kCols, n,
                         kTileCount, wfc::entropy(activeSet(app), m));
        } else if (n == 1) {
            SDL_snprintf(hint, sizeof hint, "cell %d,%d: decided, tile %d", hover % kCols, hover / kCols, wfc::lowest(m));
        } else {
            SDL_snprintf(hint, sizeof hint, "cell %d,%d: no tile fits any more", hover % kCols, hover / kCols);
        }
    } else {
        SDL_snprintf(hint, sizeof hint, "click: collapse a cell   hover: its remaining tiles   orange frame: waits to propagate");
    }
    app.hud.text(r, kFieldX, kHintY, hint, gd::kMuted);
    if (!app.benchText.empty()) app.hud.text(r, kFieldX, kBenchY, app.benchText, gd::kInk);
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
    if (!SDL_CreateWindowAndRenderer("wfc-tiles", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, SDL_LOGICAL_PRESENTATION_LETTERBOX);
    SDL_SetRenderVSync(app->renderer, 1);
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    app->sets[0] = wfc::landTileSet(true);
    app->sets[1] = wfc::landTileSet(false);
    if (app->sets[0].tiles.size() != static_cast<std::size_t>(kTileCount) || app->sets[1].tiles.size() != static_cast<std::size_t>(kTileCount)) {
        SDL_Log("tileset has %zu tiles, expected %d", app->sets[0].tiles.size(), kTileCount);
        return SDL_APP_FAILURE;
    }
    if (!buildTextures(*app)) return SDL_APP_FAILURE;
    layoutChips(*app);
    newMap(*app);
    // The first frame should already show a map under construction: run until a third of the cells are decided.
    for (int i = 0; i < 5000 && app->solver->collapsedCount() < 110 && !app->solver->finished(); ++i) app->solver->step();
    app->solver->touched.clear();
    app->lastTicks = SDL_GetTicks();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION: app.mouse = SDL_FPoint{event->motion.x, event->motion.y}; break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE: app.mouse = SDL_FPoint{-100.0F, -100.0F}; break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const SDL_FPoint p{event->button.x, event->button.y};
            app.mouse = p;
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) {
                    apply(app, c.action);
                    return SDL_APP_CONTINUE;
                }
            }
            const int cell = cellAt(p);
            if (cell >= 0 && app.solver->collapseCell(cell)) app.hold = 0.0F;
            break;
        }
        case SDL_EVENT_KEY_DOWN: {
            const bool repeat = event->key.repeat;
            if (event->key.key == SDLK_ESCAPE) return SDL_APP_SUCCESS;
            if (event->key.key == SDLK_PERIOD) {
                apply(app, Action::Step);  // holding it scrubs on purpose
                break;
            }
            if (repeat) break;
            switch (event->key.key) {
                case SDLK_SPACE: apply(app, Action::RunPause); break;
                case SDLK_N: apply(app, Action::NewMap); break;
                case SDLK_1: apply(app, Action::Speed0); break;
                case SDLK_2: apply(app, Action::Speed1); break;
                case SDLK_3: apply(app, Action::Speed2); break;
                case SDLK_4: apply(app, Action::Speed3); break;
                case SDLK_5: apply(app, Action::OrderEntropy); break;
                case SDLK_6: apply(app, Action::OrderRandom); break;
                case SDLK_7: apply(app, Action::OrderScanline); break;
                case SDLK_8: apply(app, Action::Restart); break;
                case SDLK_9: apply(app, Action::Backtrack); break;
                case SDLK_T: apply(app, Action::Weights); break;
                case SDLK_C: apply(app, Action::Counts); break;
                case SDLK_B: apply(app, Action::Compare); break;
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
    const Uint64 now = SDL_GetTicks();
    const float dt = std::min(0.25F, static_cast<float>(now - app.lastTicks) / 1000.0F);
    app.lastTicks = now;
    update(app, dt);
    drawScene(app);
    SDL_RenderPresent(app.renderer);
    ++app.frames;
    return (app.maxFrames >= 0 && app.frames >= app.maxFrames) ? SDL_APP_SUCCESS : SDL_APP_CONTINUE;
}

void SDL_AppQuit(void* appstate, SDL_AppResult) {
    auto* app = static_cast<App*>(appstate);
    if (app == nullptr) return;
    for (SDL_Texture*& t : app->textures) {
        if (t != nullptr) SDL_DestroyTexture(t);
        t = nullptr;
    }
    app->hud.destroy();
    if (app->renderer != nullptr) SDL_DestroyRenderer(app->renderer);
    if (app->window != nullptr) SDL_DestroyWindow(app->window);
    delete app;
}
