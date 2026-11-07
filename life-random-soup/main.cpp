// Random soups in the Game of Life with SDL3 main callbacks.
//
// 48 soups (16x16 cells, every cell alive with the chosen density) run side by side on an unbounded plane. A tile turns teal
// when its population has repeated for 120 generations (the soup settled into ash and gliders), grey when it died out, and the
// lifespan goes into the histogram on the right. Settled tiles are replaced by a fresh soup. The surprise is in the histogram:
// soups do not die fast, they take hundreds of generations, and the tail runs to thousands. Below it the R-pentomino and the
// acorn, five and seven cells, run for comparison and are placed against the soups.
// The world, the settle detector and the statistics live in logic.hpp and are covered by test.cpp. The demo starts 400
// generations in, so the first frame already has a histogram.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <array>
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

constexpr int kCols = 8;
constexpr int kRows = 6;
constexpr int kTileCells = 32;      // a tile shows 32x32 cells, the soup sits in the middle 16x16
constexpr int kSoupSize = 16;
constexpr int kSoupMargin = (kTileCells - kSoupSize) / 2;
constexpr float kTilePx = 64.0F;     // canvas pixels per tile, so a cell is 2 pixels
constexpr float kGridX = 24.0F;
constexpr float kGridY = 40.0F;
constexpr float kGridW = kCols * kTilePx;
constexpr float kGridH = kRows * kTilePx;

constexpr int kViewCells = 96;       // the featured pattern shows 96x96 cells, one pixel each
constexpr double kHoldSeconds = 0.7;  // a finished tile keeps its color this long before the next soup starts
constexpr int kHeadStart = 400;
constexpr int kFeaturedHeadStart = 200;
constexpr int kLong = 1000;           // "long-lived" in the display, in generations

constexpr std::array<int, 6> kDensities{20, 30, 40, 50, 60, 70};
constexpr std::array<double, 5> kRates{10.0, 30.0, 100.0, 300.0, 1000.0};
constexpr std::array<const char*, 5> kRateLabels{"10", "30", "100", "300", "1000"};
constexpr std::size_t kDefaultDensity = 3;
constexpr std::size_t kDefaultRate = 3;
constexpr double kBudgetMs = 10.0;    // simulation time per frame at most, the rest of the generations due is dropped
constexpr long kMaxGensPerFrame = 100;

constexpr float kChipH = 30.0F;
constexpr float kRowOne = 450.0F;
constexpr float kRowTwo = 488.0F;

constexpr float kPanelX = kGridX + kGridW + 16.0F;
constexpr float kPanelW = kWidth - 24.0F - kPanelX;
constexpr float kPad = 14.0F;

constexpr gd::Color kOrangeSoft{255, 237, 213, 255};
constexpr gd::Color kTileLine{214, 211, 209, 255};

constexpr std::array<const char*, 2> kPatterns{"R-PENTOMINO", "ACORN"};

enum class Kind { Density, Pattern, Rate, Pause, Step, Reseed };

struct Chip {
    Kind kind;
    std::size_t value;
    std::string label;
    SDL_FRect rect;
};

struct Tile {
    life::Run run;
    bool holding = false;
    double hold = 0.0;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    SDL_Texture* tileTex = nullptr;
    SDL_Texture* viewTex = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;
    std::array<Tile, kCols * kRows> tiles;
    life::Stats stats;
    life::Run featured;
    std::uint64_t baseSeed = 0;
    std::uint64_t soupCounter = 0;
    std::size_t densityIdx = kDefaultDensity;
    std::size_t patternIdx = 0;
    std::size_t rateIdx = kDefaultRate;
    bool paused = false;
    double acc = 0.0;
    SDL_FPoint mouse{-100.0F, -100.0F};
    Uint64 lastTicks = 0;
    long maxFrames = -1;
    long frames = 0;
};

double nowMs() {
#ifdef __EMSCRIPTEN__
    return emscripten_get_now();
#else
    return static_cast<double>(SDL_GetPerformanceCounter()) * 1000.0 / static_cast<double>(SDL_GetPerformanceFrequency());
#endif
}

bool inside(SDL_FPoint p, const SDL_FRect& r) { return SDL_PointInRectFloat(&p, &r); }

std::string percent(long part, long whole) { return whole <= 0 ? std::string("0%") : std::to_string((100 * part + whole / 2) / whole) + "%"; }

void spawn(App& app, Tile& tile) {
    ++app.soupCounter;
    tile.run = life::Run(life::randomSoup(app.baseSeed + app.soupCounter, kSoupSize, kDensities[app.densityIdx]));
    tile.holding = false;
    tile.hold = 0.0;
}

/// One generation for everything that is still running. During the head start finished tiles are replaced right away.
void stepAll(App& app, bool replaceNow) {
    for (Tile& tile : app.tiles) {
        if (!tile.holding) {
            tile.run.step();
            if (tile.run.done()) {
                app.stats.record(tile.run);
                tile.holding = true;
                tile.hold = kHoldSeconds;
            }
        }
        if (replaceNow && tile.holding) spawn(app, tile);
    }
    app.featured.step();
}

void restartFeatured(App& app, int headStart) {
    app.featured = life::Run(app.patternIdx == 0 ? life::rPentomino() : life::acorn());
    for (int g = 0; g < headStart; ++g) app.featured.step();
}

/// New soups, empty statistics, and the accumulator back to zero or the first generation comes early.
void restartSoups(App& app, int headStart) {
    app.stats.clear();
    app.soupCounter = 0;
    for (Tile& tile : app.tiles) spawn(app, tile);
    app.acc = 0.0;
    for (int g = 0; g < headStart; ++g) {
        for (Tile& tile : app.tiles) {
            if (!tile.holding) {
                tile.run.step();
                if (tile.run.done()) {
                    app.stats.record(tile.run);
                    spawn(app, tile);
                }
            }
        }
    }
}

void apply(App& app, Kind kind, std::size_t value) {
    switch (kind) {
        case Kind::Density:
            app.densityIdx = value;
            restartSoups(app, 0);
            break;
        case Kind::Pattern:
            app.patternIdx = value;
            restartFeatured(app, 0);
            break;
        case Kind::Rate:
            app.rateIdx = value;
            app.acc = 0.0;
            break;
        case Kind::Pause:
            app.paused = !app.paused;
            app.acc = 0.0;
            break;
        case Kind::Step:
            app.paused = true;
            app.acc = 0.0;
            stepAll(app, false);
            break;
        case Kind::Reseed:
            app.baseSeed += 1000000;
            restartSoups(app, 0);
            restartFeatured(app, 0);
            app.paused = false;
            break;
    }
}

bool active(const App& app, const Chip& c) {
    switch (c.kind) {
        case Kind::Density: return app.densityIdx == c.value;
        case Kind::Pattern: return app.patternIdx == c.value;
        case Kind::Rate: return app.rateIdx == c.value;
        case Kind::Pause: return app.paused;
        default: return false;
    }
}

void layoutChips(App& app) {
    const auto add = [&app](Kind kind, std::size_t value, std::string label, float& x, float y, float gap) {
        x += gap;
        const float w = app.hud.width(label, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({kind, value, std::move(label), SDL_FRect{x, y, w, kChipH}});
        x += w;
    };
    float x = 24.0F + app.hud.width("DENSITY", gd::Hud::Style::Bold);
    for (std::size_t i = 0; i < kDensities.size(); ++i) add(Kind::Density, i, std::to_string(kDensities[i]) + "%", x, kRowOne, i == 0 ? 12.0F : 8.0F);
    x += 24.0F + app.hud.width("PATTERN", gd::Hud::Style::Bold);
    for (std::size_t i = 0; i < kPatterns.size(); ++i) add(Kind::Pattern, i, kPatterns[i], x, kRowOne, i == 0 ? 12.0F : 8.0F);

    x = 24.0F + app.hud.width("SPEED", gd::Hud::Style::Bold);
    for (std::size_t i = 0; i < kRates.size(); ++i) add(Kind::Rate, i, kRateLabels[i], x, kRowTwo, i == 0 ? 12.0F : 8.0F);
    x += 12.0F + app.hud.width("gens/s");
    add(Kind::Pause, 0, "SPACE Pause", x, kRowTwo, 16.0F);
    add(Kind::Step, 0, "N Step", x, kRowTwo, 8.0F);
    add(Kind::Reseed, 0, "R New soups", x, kRowTwo, 8.0F);
}

void drawChip(App& app, const Chip& c) {
    SDL_Renderer* r = app.renderer;
    const bool on = active(app, c);
    const bool hot = inside(app.mouse, c.rect);
    gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? gd::kTeal : (hot ? gd::kTealSoft : gd::kWhite), gd::kInk);
    const float ty = c.rect.y + (c.rect.h - app.hud.lineHeight(gd::Hud::Style::Bold)) / 2.0F;
    app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, ty, c.label, on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
}

struct TileLook {
    gd::Color background;
    gd::Color cells;
};

/// White while running (orange cells once a soup is past 1000 generations), grey for dead, teal for settled, orange for long-lived.
TileLook lookOf(const life::Run& run) {
    if (!run.done()) return run.generation() >= kLong ? TileLook{gd::kWhite, gd::kOrange} : TileLook{gd::kWhite, gd::kInk};
    if (run.result() && run.population() == 0) return {gd::Color{231, 229, 228, 255}, gd::kMuted};
    if (!run.result() || run.result()->since >= kLong) return {kOrangeSoft, gd::kOrange};
    return {gd::kTealSoft, gd::kTealDark};
}

void putPixel(std::vector<Uint8>& pixels, int pitch, int x, int y, gd::Color c) {
    Uint8* d = &pixels[static_cast<std::size_t>(y) * static_cast<std::size_t>(pitch) + static_cast<std::size_t>(x) * 4];
    d[0] = c.r;
    d[1] = c.g;
    d[2] = c.b;
    d[3] = 255;
}

/// All tiles into one texture, one pixel per cell. Rows are addressed by pitch.
void uploadTiles(App& app) {
    const int w = kCols * kTileCells;
    const int h = kRows * kTileCells;
    const int pitch = w * 4;
    std::vector<Uint8> pixels(static_cast<std::size_t>(pitch) * static_cast<std::size_t>(h));
    for (int row = 0; row < kRows; ++row) {
        for (int col = 0; col < kCols; ++col) {
            const life::Run& run = app.tiles[static_cast<std::size_t>(row * kCols + col)].run;
            const TileLook look = lookOf(run);
            const int ox = col * kTileCells;
            const int oy = row * kTileCells;
            for (int y = 0; y < kTileCells; ++y) {
                for (int x = 0; x < kTileCells; ++x) putPixel(pixels, pitch, ox + x, oy + y, look.background);
            }
            for (const life::Cell& c : run.cells()) {
                const int x = c.x + kSoupMargin;
                const int y = c.y + kSoupMargin;
                if (x >= 0 && x < kTileCells && y >= 0 && y < kTileCells) putPixel(pixels, pitch, ox + x, oy + y, look.cells);
            }
        }
    }
    const SDL_Rect area{0, 0, w, h};
    SDL_UpdateTexture(app.tileTex, &area, pixels.data(), pitch);
}

void uploadFeatured(App& app) {
    const int pitch = kViewCells * 4;
    std::vector<Uint8> pixels(static_cast<std::size_t>(pitch) * static_cast<std::size_t>(kViewCells));
    const TileLook look = lookOf(app.featured);
    for (int y = 0; y < kViewCells; ++y) {
        for (int x = 0; x < kViewCells; ++x) putPixel(pixels, pitch, x, y, look.background);
    }
    const life::Cell center = life::medianCenter(app.featured.cells());  // follows the bulk, not the gliders
    for (const life::Cell& c : app.featured.cells()) {
        const int x = c.x - center.x + kViewCells / 2;
        const int y = c.y - center.y + kViewCells / 2;
        if (x >= 0 && x < kViewCells && y >= 0 && y < kViewCells) putPixel(pixels, pitch, x, y, look.cells);
    }
    const SDL_Rect area{0, 0, kViewCells, kViewCells};
    SDL_UpdateTexture(app.viewTex, &area, pixels.data(), pitch);
}

void drawTiles(App& app) {
    SDL_Renderer* r = app.renderer;
    const SDL_FRect src{0.0F, 0.0F, static_cast<float>(kCols * kTileCells), static_cast<float>(kRows * kTileCells)};
    const SDL_FRect dst{kGridX, kGridY, kGridW, kGridH};
    SDL_RenderTexture(r, app.tileTex, &src, &dst);
    const float margin = kSoupMargin * (kTilePx / kTileCells);
    const float soupPx = kSoupSize * (kTilePx / kTileCells);
    for (int row = 0; row < kRows; ++row) {
        for (int col = 0; col < kCols; ++col) {
            const float x = kGridX + static_cast<float>(col) * kTilePx;
            const float y = kGridY + static_cast<float>(row) * kTilePx;
            const Tile& tile = app.tiles[static_cast<std::size_t>(row * kCols + col)];
            if (!tile.holding) gd::strokeRect(r, x + margin, y + margin, soupPx, soupPx, kTileLine);  // where the soup started
            gd::strokeRect(r, x, y, kTilePx, kTilePx, kTileLine);
            if (tile.holding) {
                // The lifespan, on a patch of the tile's own color so it reads over the cells.
                const TileLook look = lookOf(tile.run);
                std::string label;
                if (!tile.run.result()) {
                    label = std::to_string(life::Run::kMaxGeneration) + "+";
                } else {
                    label = (tile.run.population() == 0 ? "x" : "") + std::to_string(tile.run.result()->since);
                }
                const float lw = app.hud.width(label);
                gd::fillRect(r, x + 2.0F, y + kTilePx - 24.0F, lw + 6.0F, 22.0F, look.background);
                app.hud.text(r, x + 5.0F, y + kTilePx - 24.0F, label, look.cells);
            }
        }
    }
    gd::strokeRect(r, kGridX - 1.0F, kGridY - 1.0F, kGridW + 2.0F, kGridH + 2.0F, gd::kInk);
}

void drawStatsCard(App& app) {
    SDL_Renderer* r = app.renderer;
    const life::Stats& s = app.stats;
    gd::card(r, kPanelX, 40.0F, kPanelW, 104.0F, gd::kWhite, gd::kInk);
    const float x = kPanelX + kPad;
    const float col = (kPanelW - 2.0F * kPad) / 3.0F;
    app.hud.text(r, x, 46.0F, "SOUPS", gd::kMuted, gd::Hud::Style::Bold);
    app.hud.text(r, x + col, 46.0F, "MEDIAN", gd::kMuted, gd::Hud::Style::Bold);
    app.hud.text(r, x + 2.0F * col, 46.0F, "LONGEST", gd::kMuted, gd::Hud::Style::Bold);
    app.hud.text(r, x, 66.0F, life::formatCount(s.total()), gd::kTealDark, gd::Hud::Style::Large);
    app.hud.text(r, x + col, 66.0F, s.median() < 0 ? "-" : life::formatCount(s.median()), gd::kTealDark, gd::Hud::Style::Large);
    app.hud.text(r, x + 2.0F * col, 66.0F, s.total() == 0 ? "-" : life::formatCount(s.longest()), gd::kOrange, gd::Hud::Style::Large);
    const long settled = s.dead() + s.ash();
    app.hud.text(r, x, 114.0F, "dead " + life::formatCount(s.dead()) + "  " + std::to_string(kLong) + "+ gens " + percent(s.atLeast(kLong), settled) + "  unsettled " + life::formatCount(s.capped()), gd::kInk);
}

struct Bin {
    int lo;
    int hi;
    const char* label;
};
constexpr std::array<Bin, 9> kBins{{{0, 32, "0"}, {32, 64, "32"}, {64, 128, "64"}, {128, 256, "128"}, {256, 512, "256"}, {512, 1024, "512"}, {1024, 2048, "1k"}, {2048, 4096, "2k"}, {4096, life::Stats::kCap + 1, "4k"}}};

void drawHistogram(App& app) {
    SDL_Renderer* r = app.renderer;
    const life::Stats& s = app.stats;
    const float cardY = 152.0F;
    gd::card(r, kPanelX, cardY, kPanelW, 148.0F, gd::kWhite, gd::kInk);
    const float x = kPanelX + kPad;
    app.hud.text(r, x, cardY + 6.0F, "LIFESPAN IN GENERATIONS", gd::kMuted, gd::Hud::Style::Bold);
    const float areaW = kPanelW - 2.0F * kPad;
    const float barW = areaW / static_cast<float>(kBins.size());
    const float baseY = cardY + 118.0F;
    const float maxH = 80.0F;
    long tallest = 1;
    for (const Bin& b : kBins) tallest = std::max(tallest, s.inRange(b.lo, b.hi, true) + s.inRange(b.lo, b.hi, false));
    for (std::size_t i = 0; i < kBins.size(); ++i) {
        const Bin& b = kBins[i];
        const long dead = s.inRange(b.lo, b.hi, true);
        const long ash = s.inRange(b.lo, b.hi, false);
        const float bx = x + static_cast<float>(i) * barW;
        const float ashH = std::floor(maxH * static_cast<float>(ash) / static_cast<float>(tallest));
        const float deadH = std::floor(maxH * static_cast<float>(dead) / static_cast<float>(tallest));
        const bool longLived = b.lo >= kLong;
        if (ashH > 0.0F) {
            gd::fillRect(r, bx + 3.0F, baseY - ashH, barW - 6.0F, ashH, longLived ? gd::kOrange : gd::kTeal);
            gd::strokeRect(r, bx + 3.0F, baseY - ashH, barW - 6.0F, ashH, gd::kInk);
        }
        if (deadH > 0.0F) {
            gd::fillRect(r, bx + 3.0F, baseY - ashH - deadH, barW - 6.0F, deadH, gd::kMuted);
            gd::strokeRect(r, bx + 3.0F, baseY - ashH - deadH, barW - 6.0F, deadH, gd::kInk);
        }
        app.hud.text(r, bx + 3.0F, baseY + 4.0F, b.label, gd::kMuted);
    }
    gd::fillRect(r, x, baseY, areaW, 1.0F, gd::kInk);
    char buf[48];
    SDL_snprintf(buf, sizeof buf, "tallest bar %s", life::formatCount(tallest).c_str());
    app.hud.text(r, kPanelX + kPanelW - kPad - app.hud.width(buf), cardY + 6.0F + 22.0F, buf, gd::kMuted);
}

void drawFeatured(App& app) {
    SDL_Renderer* r = app.renderer;
    const life::Stats& s = app.stats;
    const life::Run& f = app.featured;
    const float cardY = 308.0F;
    const float cardH = 116.0F;
    gd::card(r, kPanelX, cardY, kPanelW, cardH, gd::kWhite, gd::kInk);
    const float vx = kPanelX + 10.0F;
    const float vy = cardY + 10.0F;
    const SDL_FRect src{0.0F, 0.0F, static_cast<float>(kViewCells), static_cast<float>(kViewCells)};
    const SDL_FRect dst{vx, vy, static_cast<float>(kViewCells), static_cast<float>(kViewCells)};
    SDL_RenderTexture(r, app.viewTex, &src, &dst);
    gd::strokeRect(r, vx - 1.0F, vy - 1.0F, static_cast<float>(kViewCells) + 2.0F, static_cast<float>(kViewCells) + 2.0F, gd::kInk);

    const float tx = vx + static_cast<float>(kViewCells) + 14.0F;
    const std::size_t startCells = app.patternIdx == 0 ? life::rPentomino().size() : life::acorn().size();
    app.hud.text(r, tx, cardY + 6.0F, std::string(kPatterns[app.patternIdx]) + ", " + std::to_string(startCells) + " cells", gd::kTealDark, gd::Hud::Style::Bold);
    // Running: the generation counter. Settled: the lifespan, not the generation at which the detector noticed (120 later).
    if (f.settled()) {
        app.hud.text(r, tx, cardY + 28.0F, "life " + life::formatCount(f.result()->since), gd::kOrange, gd::Hud::Style::Large);
    } else {
        app.hud.text(r, tx, cardY + 28.0F, "gen " + life::formatCount(f.generation()), gd::kInk, gd::Hud::Style::Large);
    }
    app.hud.text(r, tx, cardY + 72.0F, "cells " + life::formatCount(f.population()), gd::kMuted);
    if (f.settled()) {
        const int since = f.result()->since;
        const long shorter = s.inRange(0, since, true) + s.inRange(0, since, false);
        app.hud.text(r, tx, cardY + 92.0F, "outlives " + percent(shorter, s.dead() + s.ash()) + " of soups", gd::kInk);
    } else {
        app.hud.text(r, tx, cardY + 92.0F, "median soup: " + (s.median() < 0 ? std::string("-") : life::formatCount(s.median())), gd::kMuted);
    }
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    app.hud.titleBar(r, kWidth, "[ RANDOM SOUP ]", "keys: D density, P pattern, left/right speed");

    drawTiles(app);
    drawStatsCard(app);
    drawHistogram(app);
    drawFeatured(app);

    app.hud.text(r, kGridX, kGridY + kGridH + 6.0F, "tiles: white running, teal settled, orange 1000+, grey died (x). Number = lifespan", gd::kMuted);

    const auto label = [&app, r](float x, float y, const char* s) { app.hud.text(r, x, y + (kChipH - app.hud.lineHeight(gd::Hud::Style::Bold)) / 2.0F, s, gd::kMuted, gd::Hud::Style::Bold); };
    label(24.0F, kRowOne, "DENSITY");
    label(24.0F, kRowTwo, "SPEED");
    for (const Chip& c : app.chips) {
        if (c.kind == Kind::Pattern && c.value == 0) label(c.rect.x - 12.0F - app.hud.width("PATTERN", gd::Hud::Style::Bold), kRowOne, "PATTERN");
        if (c.kind == Kind::Pause) app.hud.text(r, c.rect.x - 16.0F - app.hud.width("gens/s"), kRowTwo + (kChipH - app.hud.lineHeight()) / 2.0F, "gens/s", gd::kMuted);
    }
    for (const Chip& c : app.chips) drawChip(app, c);
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
    if (!SDL_CreateWindowAndRenderer("life-random-soup", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, gd::kLogicalPresentation);
    SDL_SetRenderVSync(app->renderer, 1);
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    app->tileTex = SDL_CreateTexture(app->renderer, SDL_PIXELFORMAT_RGBA32, SDL_TEXTUREACCESS_STATIC, kCols * kTileCells, kRows * kTileCells);
    app->viewTex = SDL_CreateTexture(app->renderer, SDL_PIXELFORMAT_RGBA32, SDL_TEXTUREACCESS_STATIC, kViewCells, kViewCells);
    if (app->tileTex == nullptr || app->viewTex == nullptr) {
        SDL_Log("texture failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetTextureScaleMode(app->tileTex, SDL_SCALEMODE_NEAREST);
    SDL_SetTextureScaleMode(app->viewTex, SDL_SCALEMODE_NEAREST);
    restartSoups(*app, kHeadStart);
    restartFeatured(*app, kFeaturedHeadStart);
    layoutChips(*app);
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
                if (inside(p, c.rect)) {
                    apply(app, c.kind, c.value);
                    break;
                }
            }
            break;
        }
        case SDL_EVENT_KEY_DOWN: {
            const SDL_Keycode key = event->key.key;
            // Everything toggles or restarts, only N is meant to be held down to walk the soups slowly.
            if (event->key.repeat && key != SDLK_N) break;
            switch (key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_SPACE: apply(app, Kind::Pause, 0); break;
                case SDLK_N: apply(app, Kind::Step, 0); break;
                case SDLK_R: apply(app, Kind::Reseed, 0); break;
                case SDLK_D: apply(app, Kind::Density, (app.densityIdx + 1) % kDensities.size()); break;
                case SDLK_P: apply(app, Kind::Pattern, (app.patternIdx + 1) % kPatterns.size()); break;
                case SDLK_LEFT:
                    if (app.rateIdx > 0) apply(app, Kind::Rate, app.rateIdx - 1);
                    break;
                case SDLK_RIGHT:
                    if (app.rateIdx + 1 < kRates.size()) apply(app, Kind::Rate, app.rateIdx + 1);
                    break;
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
    const double dt = std::min(static_cast<double>(now - app.lastTicks) / 1000.0, 0.25);
    app.lastTicks = now;
    if (!app.paused) {
        long due = std::min(life::stepsDue(app.acc, kRates[app.rateIdx], dt), kMaxGensPerFrame);
        const double start = nowMs();
        for (long i = 0; i < due; ++i) {
            stepAll(app, false);
            if (nowMs() - start > kBudgetMs) {  // too slow for this rate: drop the rest instead of falling behind more and more
                app.acc = 0.0;
                break;
            }
        }
        for (Tile& tile : app.tiles) {
            if (!tile.holding) continue;
            tile.hold -= dt;
            if (tile.hold <= 0.0) spawn(app, tile);
        }
    }
    uploadTiles(app);
    uploadFeatured(app);
    drawScene(app);
    SDL_RenderPresent(app.renderer);
    ++app.frames;
    return (app.maxFrames >= 0 && app.frames >= app.maxFrames) ? SDL_APP_SUCCESS : SDL_APP_CONTINUE;
}

void SDL_AppQuit(void* appstate, SDL_AppResult) {
    auto* app = static_cast<App*>(appstate);
    if (app != nullptr) {
        app->hud.destroy();
        if (app->tileTex != nullptr) SDL_DestroyTexture(app->tileTex);
        if (app->viewTex != nullptr) SDL_DestroyTexture(app->viewTex);
        if (app->renderer != nullptr) SDL_DestroyRenderer(app->renderer);
        if (app->window != nullptr) SDL_DestroyWindow(app->window);
        delete app;
    }
}
