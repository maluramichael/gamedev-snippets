// Wave function collapse with three levels of propagation, side by side, with SDL3 main callbacks.
//
// The same seed and the same tiles run in three maps at once. The solver always does the same thing: pick an open cell,
// decide it, and then tell the rest of the map. What differs is how much it tells.
//   no propagation   nobody is told. Every open cell still shows "any of the 29 tiles", the truth is only found when the
//                    solver stands at that cell and compares it with its decided neighbours.
//   neighbors only   the four neighbours of a decided cell drop the tiles that no longer fit, nothing travels further.
//   full cascade     a neighbour that lost tiles tells its own neighbours, until nothing changes.
// A cell with no tile left is a contradiction and the map starts over. The table counts the contradictions of 40 maps per
// combination of cell order and propagation level, measured live in small slices over many frames.
// "X-ray" marks the cells of the unpropagated map that are already lost but not noticed yet.
//
// The solver, the tile edges and the benchmark live in logic.hpp, the tile pictures in art.hpp (the same as in
// wfc-tiles), both without SDL and covered by test.cpp. This file only draws and reads input.

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
constexpr int kCell = 26;
constexpr int kCols = 10;
constexpr int kRows = 10;
constexpr int kTileCount = 29;
constexpr float kMapW = static_cast<float>(kCols * kCell);
constexpr float kMapH = static_cast<float>(kRows * kCell);
constexpr float kMapX0 = 100.0F;
constexpr float kPanelGap = 24.0F;
constexpr float kMapY = 58.0F;
constexpr float kHeadY = 34.0F;
constexpr float kStatsY = kMapY + kMapH + 4.0F;
constexpr float kRowA = 368.0F;
constexpr float kRowB = 400.0F;
constexpr float kChipH = 28.0F;
constexpr float kChipX = 24.0F;
constexpr float kTableY = 440.0F;
constexpr float kTableStride = 18.0F;
constexpr float kHintY = 516.0F;
constexpr int kBenchMaps = 40;
constexpr int kBenchCombos = 9;
constexpr int kBenchRestartCap = 100;
constexpr Uint64 kBenchBudgetMs = 8;  // per frame, so the page stays responsive while the table fills
constexpr long kMaxStepsPerFrame = 5000;
constexpr std::array<double, 4> kStepsPerSecond{15.0, 100.0, 600.0, 1.0e6};
constexpr int kPanels = 3;
constexpr std::array<wfc::Propagation, kPanels> kLevels{wfc::Propagation::Off, wfc::Propagation::Neighbors, wfc::Propagation::Full};
constexpr std::array<const char*, kPanels> kLevelNames{"NO PROPAGATION", "NEIGHBORS ONLY", "FULL CASCADE"};
constexpr std::array<const char*, 3> kOrderNames{"entropy", "scanline", "random"};
constexpr gd::Color kFlashColor{234, 88, 12, 255};

enum class Action {
    RunPause,
    Step,
    NewMap,
    Speed0,
    Speed1,
    Speed2,
    Speed3,
    OrderEntropy,
    OrderScanline,
    OrderRandom,
    XRay,
    NewSeeds
};

struct Chip {
    Action action;
    std::string labelA;
    std::string labelB;
    SDL_FRect rect;
};

struct Panel {
    std::unique_ptr<wfc::Solver> solver;
    std::vector<float> flash;  // 1 right after a cell changed, fades to 0
    float hold = 0.0F;         // seconds to keep a contradiction on screen before it is handled
    double budget = 0.0;
};

struct BenchState {
    std::array<wfc::Bench, kBenchCombos> cells{};
    int next = 0;  // maps done, combination by combination (row = order, column = level)
    int round = 0;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::array<SDL_Texture*, kTileCount> textures{};
    std::array<art::Rgb, kTileCount> means{};
    wfc::TileSet set;
    std::array<Panel, kPanels> panels;
    std::vector<Chip> chips;
    BenchState bench;

    wfc::Order order = wfc::Order::Entropy;
    int speed = 1;
    bool running = true;
    bool xray = true;
    std::uint64_t mapIndex = 0;

    SDL_FPoint mouse{-100.0F, -100.0F};
    Uint64 lastTicks = 0;
    long maxFrames = -1;
    long frames = 0;
};

float panelX(int panel) { return kMapX0 + static_cast<float>(panel) * (kMapW + kPanelGap); }

std::uint64_t seedFor(std::uint64_t index) { return 0x5EED0000ULL + index * 7919ULL; }

wfc::Limits limits() {
    wfc::Limits l;
    l.maxRestarts = kBenchRestartCap;
    return l;
}

void newMap(App& app) {
    const std::uint64_t seed = seedFor(app.mapIndex++);
    for (int p = 0; p < kPanels; ++p) {
        Panel& panel = app.panels[static_cast<std::size_t>(p)];
        panel.solver = std::make_unique<wfc::Solver>(app.set, kCols, kRows, seed, app.order, kLevels[static_cast<std::size_t>(p)], limits());
        panel.solver->trackTouched = true;
        panel.flash.assign(static_cast<std::size_t>(kCols * kRows), 0.0F);
        panel.hold = 0.0F;
        panel.budget = 0.0;
    }
}

/// Which panel and cell a point of the canvas is over, false outside every map.
bool cellAt(SDL_FPoint p, int& panel, int& cell) {
    for (int i = 0; i < kPanels; ++i) {
        const float fx = (p.x - panelX(i)) / static_cast<float>(kCell);
        const float fy = (p.y - kMapY) / static_cast<float>(kCell);
        if (!(fx >= 0.0F) || !(fy >= 0.0F) || fx >= static_cast<float>(kCols) || fy >= static_cast<float>(kRows)) continue;
        panel = i;
        cell = static_cast<int>(fy) * kCols + static_cast<int>(fx);
        return true;
    }
    return false;
}

bool buildTextures(App& app) {
    for (std::size_t t = 0; t < app.set.tiles.size(); ++t) {
        const std::vector<std::uint8_t> px = art::render(app.set.tiles[t], kCell, static_cast<std::uint32_t>(t) * 7U + 1U);
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
    float x = kChipX;
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
               {Action::Step, ". Step", ". Step", 8.0F},
               {Action::NewMap, "N New map", "N New map", 8.0F},
               {Action::Speed0, "1 15/s", "1 15/s", 24.0F},
               {Action::Speed1, "2 100/s", "2 100/s", 8.0F},
               {Action::Speed2, "3 600/s", "3 600/s", 8.0F},
               {Action::Speed3, "4 max", "4 max", 8.0F}},
              kRowA);
    layoutRow(app,
              {{Action::OrderEntropy, "5 Entropy", "5 Entropy", 0.0F},
               {Action::OrderScanline, "6 Scanline", "6 Scanline", 8.0F},
               {Action::OrderRandom, "7 Random", "7 Random", 8.0F},
               {Action::XRay, "X X-ray", "X X-ray", 24.0F},
               {Action::NewSeeds, "B New seeds", "B New seeds", 8.0F}},
              kRowB);
}

const char* chipLabel(const App& app, const Chip& c) { return c.action == Action::RunPause && !app.running ? c.labelB.c_str() : c.labelA.c_str(); }

bool chipActive(const App& app, Action a) {
    switch (a) {
        case Action::RunPause: return app.running;
        case Action::Speed0: return app.speed == 0;
        case Action::Speed1: return app.speed == 1;
        case Action::Speed2: return app.speed == 2;
        case Action::Speed3: return app.speed == 3;
        case Action::OrderEntropy: return app.order == wfc::Order::Entropy;
        case Action::OrderScanline: return app.order == wfc::Order::Scanline;
        case Action::OrderRandom: return app.order == wfc::Order::Random;
        case Action::XRay: return app.xray;
        default: return false;
    }
}

void resetBench(App& app) {
    app.bench.cells = {};
    app.bench.next = 0;
}

void apply(App& app, Action a) {
    switch (a) {
        case Action::RunPause:
            app.running = !app.running;
            for (Panel& p : app.panels) p.budget = 0.0;
            break;
        case Action::Step:
            for (Panel& p : app.panels) {
                if (p.solver && !p.solver->finished()) {
                    p.solver->step();
                    p.hold = 0.0F;
                }
            }
            break;
        case Action::NewMap: newMap(app); break;
        case Action::Speed0: app.speed = 0; break;
        case Action::Speed1: app.speed = 1; break;
        case Action::Speed2: app.speed = 2; break;
        case Action::Speed3: app.speed = 3; break;
        case Action::OrderEntropy:
        case Action::OrderScanline:
        case Action::OrderRandom:
            app.order = a == Action::OrderEntropy ? wfc::Order::Entropy : a == Action::OrderScanline ? wfc::Order::Scanline : wfc::Order::Random;
            newMap(app);
            break;
        case Action::XRay: app.xray = !app.xray; break;
        case Action::NewSeeds:
            ++app.bench.round;
            resetBench(app);
            break;
    }
}

// ---- update ----

constexpr std::array<wfc::Order, 3> kBenchOrders{wfc::Order::Entropy, wfc::Order::Scanline, wfc::Order::Random};

/// Runs maps of the table until the frame budget is spent. Maps are drawn from the same seeds as wfc::benchmark().
void advanceBench(App& app) {
    BenchState& b = app.bench;
    const Uint64 start = SDL_GetTicks();
    while (b.next < kBenchCombos * kBenchMaps && SDL_GetTicks() - start < kBenchBudgetMs) {
        const int combo = b.next / kBenchMaps;
        const int index = b.next % kBenchMaps;
        const std::uint64_t base = 5ULL + 1000ULL * static_cast<std::uint64_t>(b.round);
        wfc::benchmarkMap(b.cells[static_cast<std::size_t>(combo)], app.set, kCols, kRows, kBenchOrders[static_cast<std::size_t>(combo / 3)],
                          kLevels[static_cast<std::size_t>(combo % 3)], wfc::benchmarkSeed(base, index), limits());
        ++b.next;
    }
}

void updatePanel(const App& app, Panel& panel, float dt) {
    for (float& f : panel.flash) f = std::max(0.0F, f - dt * 2.0F);
    wfc::Solver& s = *panel.solver;
    if (app.running && !s.finished()) {
        if (panel.hold > 0.0F) {
            panel.hold -= dt;
        } else {
            panel.budget += kStepsPerSecond[static_cast<std::size_t>(app.speed)] * static_cast<double>(dt);
            const long n = static_cast<long>(std::min(panel.budget, static_cast<double>(kMaxStepsPerFrame)));
            panel.budget = std::min(panel.budget - static_cast<double>(n), 1.0);  // the rest does not pile up
            for (long i = 0; i < n && !s.finished(); ++i) {
                s.step();
                if (s.status == wfc::Status::Contradiction) {
                    panel.hold = app.speed == 3 ? 0.2F : 0.6F;  // leave the dead end on screen for a moment
                    panel.budget = 0.0;
                    break;
                }
            }
        }
    }
    for (int c : s.touched) panel.flash[static_cast<std::size_t>(c)] = 1.0F;
    s.touched.clear();
}

void update(App& app, float dt) {
    for (Panel& p : app.panels) updatePanel(app, p, dt);
    advanceBench(app);
}

// ---- drawing ----

void drawHatch(SDL_Renderer* r, float x, float y) {
    gd::fillRect(r, x, y, kCell, kCell, gd::Color{kFlashColor.r, kFlashColor.g, kFlashColor.b, 70});
    gd::setColor(r, gd::kOrange);
    for (float k = 6.0F; k < static_cast<float>(kCell) * 2.0F - 5.0F; k += 6.0F) {
        const float x0 = std::min(k, static_cast<float>(kCell));
        const float y0 = k - x0;
        const float y1 = std::min(k, static_cast<float>(kCell));
        const float x1 = k - y1;
        SDL_RenderLine(r, x + x0, y + y0, x + x1, y + y1);
    }
}

void drawUndecided(App& app, wfc::Mask mask, float x, float y, bool digit) {
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
    float cr = 0.0F;
    float cg = 0.0F;
    float cb = 0.0F;
    float sum = 0.0F;
    for (wfc::Mask m = mask; m != 0; m &= m - 1) {
        const auto t = static_cast<std::size_t>(wfc::lowest(m));
        const auto w = static_cast<float>(app.set.tiles[t].weight);
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
    if (digit && n >= 2 && n <= 9) {
        char text[2] = {static_cast<char>('0' + n), 0};
        app.hud.text(r, x + 8.0F, y + 2.0F, text, gd::kMuted);
    }
}

gd::Color stateColor(const App& app, const wfc::Solver& s) {
    if (s.status == wfc::Status::Done) return gd::kTeal;
    if (s.status == wfc::Status::GaveUp || s.status == wfc::Status::Contradiction) return gd::kOrange;
    return app.running ? gd::kInk : gd::kMuted;
}

const char* stateLabel(const App& app, const wfc::Solver& s) {
    switch (s.status) {
        case wfc::Status::Done: return "DONE";
        case wfc::Status::GaveUp: return "GAVE UP";
        case wfc::Status::Contradiction: return "DEAD END";
        case wfc::Status::Running: break;
    }
    return app.running ? "RUNNING" : "PAUSED";
}

void drawPanel(App& app, int index, int hoverPanel, int hoverCell) {
    SDL_Renderer* r = app.renderer;
    const Panel& panel = app.panels[static_cast<std::size_t>(index)];
    const wfc::Solver& s = *panel.solver;
    const float x0 = panelX(index);
    const bool off = kLevels[static_cast<std::size_t>(index)] == wfc::Propagation::Off;

    for (int cy = 0; cy < kRows; ++cy) {
        for (int cx = 0; cx < kCols; ++cx) {
            const int c = cy * kCols + cx;
            const float x = x0 + static_cast<float>(cx * kCell);
            const float y = kMapY + static_cast<float>(cy * kCell);
            const wfc::Mask m = s.domain[static_cast<std::size_t>(c)];
            if (wfc::count(m) == 1) {
                const SDL_FRect dst{x, y, static_cast<float>(kCell), static_cast<float>(kCell)};
                SDL_RenderTexture(r, app.textures[static_cast<std::size_t>(wfc::lowest(m))], nullptr, &dst);
            } else {
                drawUndecided(app, m, x, y, !off);
                if (off && app.xray && m != 0 && s.options(c) == 0) drawHatch(r, x, y);
            }
            const float f = panel.flash[static_cast<std::size_t>(c)];
            if (f > 0.0F) {
                gd::fillRect(r, x, y, kCell, kCell, gd::Color{kFlashColor.r, kFlashColor.g, kFlashColor.b, static_cast<Uint8>(f * 110.0F)});
            }
        }
    }
    // Cells waiting for their neighbours to be checked.
    for (int c : s.queue) {
        const float x = x0 + static_cast<float>((c % kCols) * kCell);
        const float y = kMapY + static_cast<float>((c / kCols) * kCell);
        gd::strokeRect(r, x + 1.0F, y + 1.0F, kCell - 2.0F, kCell - 2.0F, gd::kOrange);
        gd::strokeRect(r, x + 2.0F, y + 2.0F, kCell - 4.0F, kCell - 4.0F, gd::kOrange);
    }
    // The cell the solver will decide next.
    if (s.status == wfc::Status::Running && s.queue.empty()) {
        const int next = s.pickCell();
        if (next >= 0) {
            const float x = x0 + static_cast<float>((next % kCols) * kCell);
            const float y = kMapY + static_cast<float>((next / kCols) * kCell);
            gd::strokeRect(r, x + 1.0F, y + 1.0F, kCell - 2.0F, kCell - 2.0F, gd::kTeal);
            gd::strokeRect(r, x + 2.0F, y + 2.0F, kCell - 4.0F, kCell - 4.0F, gd::kTeal);
        }
    }
    if (hoverPanel == index && hoverCell >= 0) {
        const float x = x0 + static_cast<float>((hoverCell % kCols) * kCell);
        const float y = kMapY + static_cast<float>((hoverCell / kCols) * kCell);
        gd::strokeRect(r, x, y, kCell, kCell, gd::kInk);
        gd::strokeRect(r, x - 1.0F, y - 1.0F, kCell + 2.0F, kCell + 2.0F, gd::kInk);
    }
    const bool running = s.status == wfc::Status::Running;
    const gd::Color frame = running ? gd::kInk : (s.status == wfc::Status::Done ? gd::kTeal : gd::kOrange);
    gd::strokeRect(r, x0 - 1.0F, kMapY - 1.0F, kMapW + 2.0F, kMapH + 2.0F, frame);
    if (!running) gd::strokeRect(r, x0 - 2.0F, kMapY - 2.0F, kMapW + 4.0F, kMapH + 4.0F, frame);

    // Heading, and on the right what the map knows that the solver does not (or the size of the last ripple).
    app.hud.text(r, x0, kHeadY, kLevelNames[static_cast<std::size_t>(index)], off ? gd::kOrange : gd::kInk, gd::Hud::Style::Bold);
    char extra[32] = "";
    if (off) {
        if (app.xray) SDL_snprintf(extra, sizeof extra, "%d doomed", s.doomedCount());
    } else {
        SDL_snprintf(extra, sizeof extra, "ripple %d", s.counters.lastCascade);
    }
    app.hud.text(r, x0 + kMapW - app.hud.width(extra), kHeadY, extra, gd::kMuted);

    char line[64];
    SDL_snprintf(line, sizeof line, "%d/%d cells  steps %ld", s.collapsedCount(), s.cells(), s.counters.steps);
    app.hud.text(r, x0, kStatsY, line, gd::kInk);
    SDL_snprintf(line, sizeof line, "contradictions %d", s.counters.contradictions);
    app.hud.text(r, x0, kStatsY + 17.0F, line, s.counters.contradictions > 0 ? gd::kOrange : gd::kInk);
    const char* state = stateLabel(app, s);
    app.hud.text(r, x0 + kMapW - app.hud.width(state, gd::Hud::Style::Bold), kStatsY + 17.0F, state, stateColor(app, s), gd::Hud::Style::Bold);
}

void drawTable(App& app) {
    SDL_Renderer* r = app.renderer;
    const BenchState& b = app.bench;
    char head[160];
    SDL_snprintf(head, sizeof head, "contradictions in %d maps of %dx%d, clean = none at all, cap %d restarts%s", kBenchMaps, kCols, kRows, kBenchRestartCap,
                 b.next < kBenchCombos * kBenchMaps ? "  (measuring ...)" : "");
    app.hud.text(r, kChipX, kTableY, head, gd::kMuted);
    for (int o = 0; o < 3; ++o) {
        const float y = kTableY + kTableStride * static_cast<float>(o + 1);
        const bool current = static_cast<int>(app.order) == o;
        if (current) gd::fillRect(r, kChipX - 4.0F, y, kMapX0 + 3.0F * (kMapW + kPanelGap) - kPanelGap - kChipX + 8.0F, kTableStride - 1.0F, gd::kTealSoft);
        app.hud.text(r, kChipX, y, kOrderNames[static_cast<std::size_t>(o)], current ? gd::kTealDark : gd::kMuted, current ? gd::Hud::Style::Bold : gd::Hud::Style::Regular);
        for (int l = 0; l < kPanels; ++l) {
            const wfc::Bench& cell = b.cells[static_cast<std::size_t>(o * 3 + l)];
            char text[48];
            gd::Color color = gd::kMuted;
            if (cell.maps < kBenchMaps) {
                SDL_snprintf(text, sizeof text, "...");
            } else {
                if (cell.gaveUp > 0) SDL_snprintf(text, sizeof text, "%d (%d clean, %d capped)", cell.contradictions, cell.clean, cell.gaveUp);
                else SDL_snprintf(text, sizeof text, "%d (%d clean)", cell.contradictions, cell.clean);
                color = cell.contradictions == 0 ? gd::kTealDark : gd::kOrange;
            }
            app.hud.text(r, panelX(l) + 14.0F, y, text, color, gd::Hud::Style::Bold);
        }
    }
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    int hoverPanel = -1;
    int hoverCell = -1;
    if (!cellAt(app.mouse, hoverPanel, hoverCell)) {
        hoverPanel = -1;
        hoverCell = -1;
    }

    gd::backdrop(r, kWidth, kHeight);
    const char* orderName = app.order == wfc::Order::Entropy ? "lowest entropy first" : app.order == wfc::Order::Random ? "random cell" : "row by row";
    app.hud.titleBar(r, kWidth, "[ WFC: WHY PROPAGATION MATTERS ]", orderName);

    for (int p = 0; p < kPanels; ++p) drawPanel(app, p, hoverPanel, hoverCell);

    for (const Chip& c : app.chips) {
        const bool on = chipActive(app, c.action);
        const char* label = chipLabel(app, c);
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? gd::kTeal : gd::kWhite, gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 3.0F, label, on ? gd::kWhite : gd::kInk,
                     gd::Hud::Style::Bold);
    }

    drawTable(app);
    app.hud.text(r, kChipX, kHintY, "click a cell to decide it   orange frame: waits to propagate   hatched: lost, not noticed yet", gd::kMuted);
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
    if (!SDL_CreateWindowAndRenderer("wfc-propagation", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, gd::kLogicalPresentation);
    SDL_SetRenderVSync(app->renderer, 1);
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    app->set = wfc::landTileSet(true);
    if (app->set.tiles.size() != static_cast<std::size_t>(kTileCount)) {
        SDL_Log("tileset has %zu tiles, expected %d", app->set.tiles.size(), kTileCount);
        return SDL_APP_FAILURE;
    }
    if (!buildTextures(*app)) return SDL_APP_FAILURE;
    layoutChips(*app);
    newMap(*app);
    // The first frame should already show maps under construction: run each until about a third of the cells are decided.
    for (Panel& p : app->panels) {
        wfc::Solver& s = *p.solver;
        for (int i = 0; i < 5000 && s.collapsedCount() < 34 && s.status == wfc::Status::Running; ++i) s.step();
        s.touched.clear();
    }
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
            int panel = -1;
            int cell = -1;
            if (cellAt(p, panel, cell)) {
                Panel& pan = app.panels[static_cast<std::size_t>(panel)];
                if (pan.solver->collapseCell(cell)) pan.hold = 0.0F;
            }
            break;
        }
        case SDL_EVENT_KEY_DOWN: {
            if (event->key.key == SDLK_ESCAPE) return SDL_APP_SUCCESS;
            if (event->key.key == SDLK_PERIOD) {
                apply(app, Action::Step);  // holding it scrubs on purpose
                break;
            }
            if (event->key.repeat) break;
            switch (event->key.key) {
                case SDLK_SPACE: apply(app, Action::RunPause); break;
                case SDLK_N: apply(app, Action::NewMap); break;
                case SDLK_1: apply(app, Action::Speed0); break;
                case SDLK_2: apply(app, Action::Speed1); break;
                case SDLK_3: apply(app, Action::Speed2); break;
                case SDLK_4: apply(app, Action::Speed3); break;
                case SDLK_5: apply(app, Action::OrderEntropy); break;
                case SDLK_6: apply(app, Action::OrderScanline); break;
                case SDLK_7: apply(app, Action::OrderRandom); break;
                case SDLK_X: apply(app, Action::XRay); break;
                case SDLK_B: apply(app, Action::NewSeeds); break;
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
