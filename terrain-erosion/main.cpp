// Terrain erosion with SDL3 main callbacks.
//
// Left: a smooth heightmap from noise. Right: the same map after thousands of drops of rain. Each drop rolls
// downhill, picks up sediment where it is steep and fast, puts it down where it slows or climbs. A few recent drop
// paths are drawn in blue. The strip below is a cut through both maps along the marked row (hover to move it):
// orange where material was taken, teal where it was put down.
//
// The chips show the typical mistake next to the real thing. "Naive" digs the same amount in every step and never
// puts anything down: trenches, empty hollows, material gone (see the material figure). "Sediment" carries what it
// digs and drops it again. Thermal erosion lets slopes steeper than the talus angle slide. Lines draws contour lines,
// Diff tints the eroded map by what changed. Dragging on the eroded map rains on that spot.
//
// The drop count is a rate (drops per second) and runs through an accumulator, so a 144 Hz screen does not erode
// faster than a 60 Hz one. The simulation itself is in logic.hpp and covered by test.cpp.

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
constexpr int kN = 150;            // cells per side
constexpr int kScale = 2;          // pixels per cell
constexpr float kPanel = static_cast<float>(kN * kScale);
constexpr float kMapY = 58.0F;
constexpr std::array<float, 2> kMapX{24.0F, 340.0F};  // original, eroded
constexpr float kSliderX = 664.0F;
constexpr float kSliderW = 272.0F;
constexpr float kTrackX = 672.0F;
constexpr float kTrackW = 256.0F;
constexpr float kRowH = 40.0F;
constexpr float kChipY = 490.0F;
constexpr float kChipH = 34.0F;
constexpr float kChartX = 24.0F;
constexpr float kChartY = 392.0F;
constexpr float kChartW = 616.0F;
constexpr float kChartH = 78.0F;

constexpr int kMaxDropsPerFrame = 600;    // keeps a slow frame from snowballing
constexpr double kMaxDt = 0.25;
constexpr double kThermalPassesPerSecond = 30.0;
constexpr int kMaxThermalPerFrame = 4;
constexpr int kWarmupDrops = 6000;        // the first frame (and every mode switch) already shows erosion
constexpr double kBrushDropsPerSecond = 2400.0;  // while dragging on the eroded map
constexpr double kBrushRadiusCells = 7.0;
constexpr int kPathSlots = 24;
constexpr double kDiffScale = 0.05;       // change that gets the full color in the Diff view
constexpr double kContourInterval = 0.05;
constexpr std::uint64_t kRainSeed = 20241;

struct SliderDef {
    const char* label;
    const char* format;
    double lo, hi, step;
};
constexpr std::array<SliderDef, 6> kSliders{{{"Drops per second", "%.0f", 0.0, 12000.0, 200.0},
                                             {"Inertia", "%.2f", 0.0, 0.9, 0.05},
                                             {"Capacity", "%.1f", 1.0, 12.0, 0.5},
                                             {"Erosion", "%.2f", 0.05, 1.0, 0.05},
                                             {"Evaporation", "%.3f", 0.0, 0.1, 0.005},
                                             {"Talus angle", "%.0f°", 10.0, 60.0, 1.0}}};

enum class Action { Rain, Naive, Sediment, Thermal, Lines, Diff, NewTerrain, Reset };
enum class Drag { None, Slider, Brush };

struct Chip {
    Action action;
    std::string label;
    SDL_FRect rect;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    SDL_Texture* tex[2] = {nullptr, nullptr};
    gd::Hud hud;
    std::vector<Chip> chips;

    te::Grid original{kN, kN};
    te::Grid eroded{kN, kN};
    std::vector<double> scratch;
    te::Rng rng{kRainSeed};
    te::DropParams drop;
    te::ThermalParams thermal;
    int terrainSeed = 11;

    std::array<double, 6> sliderValue{2400.0, 0.1, 4.0, 0.3, 0.01, 40.0};
    bool rain = true;
    bool thermalOn = false;
    bool lines = false;
    bool diff = false;

    long long drops = 0;
    double dropAcc = 0.0;
    double thermalAcc = 0.0;
    double brushAcc = 0.0;
    int recordCounter = 0;
    double baseTotal = 0.0;
    double baseSteepest = 0.0;
    double baseMean = 0.0;
    double nowTotal = 0.0;
    double nowSteepest = 0.0;
    double nowMean = 0.0;

    std::array<std::vector<te::Vec2>, kPathSlots> paths;
    std::array<bool, kPathSlots> pathUsed{};
    int pathHead = 0;

    bool dirtyOriginal = true;
    bool dirtyEroded = true;
    Drag drag = Drag::None;
    int slider = 0;
    float mouseX = 0.0F;
    float mouseY = 0.0F;
    int profileRow = kN / 2;

    Uint64 lastTicks = 0;
    long maxFrames = -1;
    long frames = 0;
};

bool inRect(float x, float y, float rx, float ry, float rw, float rh) { return x >= rx && x < rx + rw && y >= ry && y < ry + rh; }

// ---------------------------------------------------------------- settings

void syncParams(App& app) {
    const auto& s = app.sliderValue;
    app.drop.inertia = s[1];
    app.drop.capacity = s[2];
    app.drop.erode = s[3];
    app.drop.evaporate = s[4];
    app.thermal.talusDeg = s[5];
}

void sliderSet(App& app, int i, double v) {
    const SliderDef& d = kSliders[static_cast<std::size_t>(i)];
    app.sliderValue[static_cast<std::size_t>(i)] = std::clamp(v, d.lo, d.hi);
    syncParams(app);
}

void sliderNudge(App& app, int i, int direction) {
    const SliderDef& d = kSliders[static_cast<std::size_t>(i)];
    sliderSet(app, i, app.sliderValue[static_cast<std::size_t>(i)] + direction * d.step);
}

void sliderFromMouse(App& app, float mx) {
    const SliderDef& d = kSliders[static_cast<std::size_t>(app.slider)];
    const double t = (static_cast<double>(mx) - static_cast<double>(kTrackX)) / static_cast<double>(kTrackW);
    sliderSet(app, app.slider, te::sliderValue(t, d.lo, d.hi, d.step));
}

float rowTop(int i) { return kMapY + kRowH * static_cast<float>(i); }

// ---------------------------------------------------------------- simulation

void recordPath(App& app, std::vector<te::Vec2>& scratchPath) {
    auto& slot = app.paths[static_cast<std::size_t>(app.pathHead)];
    slot = scratchPath;  // reuses the slot's capacity after the first few drops
    app.pathUsed[static_cast<std::size_t>(app.pathHead)] = true;
    app.pathHead = (app.pathHead + 1) % kPathSlots;
}

/// Runs `count` drops at random places (or around `cx, cy` when `local`).
void rainDrops(App& app, int count, bool local, double cx, double cy, int recordEvery) {
    static std::vector<te::Vec2> path;
    for (int i = 0; i < count; ++i) {
        double x;
        double y;
        if (local) {  // uniform in a disc
            const double a = app.rng.next() * te::kTwoPi;
            const double r = kBrushRadiusCells * std::sqrt(app.rng.next());
            x = cx + std::cos(a) * r;
            y = cy + std::sin(a) * r;
        } else {
            x = app.rng.next() * (kN - 1);
            y = app.rng.next() * (kN - 1);
        }
        const bool record = ++app.recordCounter >= recordEvery;
        if (record) app.recordCounter = 0;
        te::runDrop(app.eroded, x, y, app.rng, app.drop, record ? &path : nullptr);
        if (record) recordPath(app, path);
        ++app.drops;
    }
    app.dirtyEroded = true;
}

void refreshStats(App& app) {
    app.nowTotal = te::totalHeight(app.eroded);
    app.nowSteepest = te::steepestDeg(app.eroded, te::kRelief);
    app.nowMean = te::meanSlopeDeg(app.eroded, te::kRelief);
}

/// Back to the smooth terrain, same random sequence, so Naive and Sediment get the very same drops.
void resetErosion(App& app, int warmup) {
    app.eroded = app.original;
    app.rng = te::Rng(kRainSeed);
    app.drops = 0;
    app.dropAcc = 0.0;
    app.thermalAcc = 0.0;
    app.brushAcc = 0.0;
    app.recordCounter = 0;
    app.pathUsed.fill(false);
    app.pathHead = 0;
    app.baseTotal = te::totalHeight(app.original);
    app.baseSteepest = te::steepestDeg(app.original, te::kRelief);
    app.baseMean = te::meanSlopeDeg(app.original, te::kRelief);
    app.drag = Drag::None;
    if (warmup > 0) rainDrops(app, warmup, false, 0.0, 0.0, 400);
    refreshStats(app);
    app.dirtyEroded = true;
}

void newTerrain(App& app) {
    app.terrainSeed = app.terrainSeed % 97 + 1;
    te::generateTerrain(app.original, app.terrainSeed);
    app.dirtyOriginal = true;
    resetErosion(app, 0);
}

void apply(App& app, Action action) {
    switch (action) {
        case Action::Rain: app.rain = !app.rain; break;
        case Action::Naive:
        case Action::Sediment:
            app.drop.mode = action == Action::Naive ? te::Mode::Naive : te::Mode::Sediment;
            resetErosion(app, kWarmupDrops);
            break;
        case Action::Thermal: app.thermalOn = !app.thermalOn; break;
        case Action::Lines:
            app.lines = !app.lines;
            app.dirtyOriginal = true;
            app.dirtyEroded = true;
            break;
        case Action::Diff: app.diff = !app.diff; app.dirtyEroded = true; break;
        case Action::NewTerrain: newTerrain(app); break;
        case Action::Reset: resetErosion(app, 0); break;
    }
}

bool active(const App& app, Action action) {
    switch (action) {
        case Action::Rain: return app.rain;
        case Action::Naive: return app.drop.mode == te::Mode::Naive;
        case Action::Sediment: return app.drop.mode == te::Mode::Sediment;
        case Action::Thermal: return app.thermalOn;
        case Action::Lines: return app.lines;
        case Action::Diff: return app.diff;
        default: return false;
    }
}

void step(App& app, double dt) {
    if (app.rain && app.sliderValue[0] > 0.0) {
        app.dropAcc += app.sliderValue[0] * dt;
        const int whole = std::min(static_cast<int>(app.dropAcc), kMaxDropsPerFrame);
        app.dropAcc = std::min(app.dropAcc - whole, 1.0);  // a slow frame does not leave a backlog
        if (whole > 0) rainDrops(app, whole, false, 0.0, 0.0, 60);
    }
    if (app.drag == Drag::Brush && inRect(app.mouseX, app.mouseY, kMapX[1], kMapY, kPanel, kPanel)) {
        app.brushAcc += kBrushDropsPerSecond * dt;
        const int whole = std::min(static_cast<int>(app.brushAcc), kMaxDropsPerFrame);
        app.brushAcc = std::min(app.brushAcc - whole, 1.0);
        const double cx = (static_cast<double>(app.mouseX) - kMapX[1]) / kScale - 0.5;
        const double cy = (static_cast<double>(app.mouseY) - kMapY) / kScale - 0.5;
        if (whole > 0) rainDrops(app, whole, true, cx, cy, 4);
    }
    if (app.thermalOn) {
        app.thermalAcc += kThermalPassesPerSecond * dt;
        const int passes = std::min(static_cast<int>(app.thermalAcc), kMaxThermalPerFrame);
        app.thermalAcc = std::min(app.thermalAcc - passes, 1.0);
        for (int i = 0; i < passes; ++i) te::thermalPass(app.eroded, app.scratch, app.thermal);
        if (passes > 0) app.dirtyEroded = true;
    }
}

// ---------------------------------------------------------------- drawing the maps

void paintMap(const App& app, const te::Grid& g, const te::Grid* reference, SDL_Texture* texture) {
    void* pixels = nullptr;
    int pitch = 0;
    if (!SDL_LockTexture(texture, nullptr, &pixels, &pitch)) {
        SDL_Log("lock map failed: %s", SDL_GetError());
        return;
    }
    const auto h = [&](int x, int y) { return g.at(std::clamp(x, 0, kN - 1), std::clamp(y, 0, kN - 1)); };
    for (int y = 0; y < kN; ++y) {
        auto* row = static_cast<Uint8*>(pixels) + static_cast<std::ptrdiff_t>(y) * pitch;  // rows are `pitch` apart
        for (int x = 0; x < kN; ++x) {
            const double here = h(x, y);
            const double gx = (h(x + 1, y) - h(x - 1, y)) * 0.5 * te::kRelief;
            const double gy = (h(x, y + 1) - h(x, y - 1)) * 0.5 * te::kRelief;
            const double light = te::shadeFactor(gx, gy);
            te::Rgb c;
            if (reference != nullptr) {
                c = te::diffTint(te::shade(te::Rgb{214, 212, 206}, light), here - reference->at(x, y), kDiffScale);
            } else {
                c = te::shade(te::terrainColor(here), light);
            }
            if (app.lines && (te::isContour(here, h(x + 1, y), kContourInterval) || te::isContour(here, h(x, y + 1), kContourInterval))) c = te::shade(c, 0.62);
            Uint8* px = row + 4 * x;
            px[0] = c.r;
            px[1] = c.g;
            px[2] = c.b;
            px[3] = 255;
        }
    }
    SDL_UnlockTexture(texture);
}

void rebuild(App& app) {
    if (app.dirtyOriginal) {
        paintMap(app, app.original, nullptr, app.tex[0]);
        app.dirtyOriginal = false;
    }
    if (app.dirtyEroded) {
        paintMap(app, app.eroded, app.diff ? &app.original : nullptr, app.tex[1]);
        refreshStats(app);
        app.dirtyEroded = false;
    }
}

float cellToScreenX(int map, double cx) { return kMapX[static_cast<std::size_t>(map)] + static_cast<float>((cx + 0.5) * kScale); }
float cellToScreenY(double cy) { return kMapY + static_cast<float>((cy + 0.5) * kScale); }

void drawSliders(App& app) {
    SDL_Renderer* r = app.renderer;
    for (int i = 0; i < static_cast<int>(kSliders.size()); ++i) {
        const SliderDef& d = kSliders[static_cast<std::size_t>(i)];
        const float y = rowTop(i);
        const double v = app.sliderValue[static_cast<std::size_t>(i)];
        char value[24];
        SDL_snprintf(value, sizeof value, d.format, v);
        app.hud.text(r, kSliderX + 4.0F, y, d.label, gd::kInk, gd::Hud::Style::Bold);
        app.hud.text(r, kSliderX + kSliderW - 4.0F - app.hud.width(value, gd::Hud::Style::Bold), y, value, gd::kTealDark, gd::Hud::Style::Bold);
        const float ty = y + 29.0F;
        const float knobX = kTrackX + kTrackW * static_cast<float>(te::tOfValue(v, d.lo, d.hi));
        gd::fillRect(r, kTrackX, ty - 2.0F, kTrackW, 4.0F, gd::kGrid);
        gd::strokeRect(r, kTrackX, ty - 2.0F, kTrackW, 4.0F, gd::kMuted);
        gd::fillRect(r, kTrackX, ty - 2.0F, knobX - kTrackX, 4.0F, gd::kTeal);
        const bool held = app.drag == Drag::Slider && app.slider == i;
        gd::card(r, knobX - 6.0F, ty - 9.0F, 12.0F, 18.0F, held ? gd::kTeal : gd::kWhite, gd::kInk);
    }
}

void drawPaths(App& app) {
    SDL_Renderer* r = app.renderer;
    for (int k = 0; k < kPathSlots; ++k) {
        // The newest path is the strongest, the oldest almost gone.
        const int age = (app.pathHead - 1 - k + 2 * kPathSlots) % kPathSlots;
        const auto& path = app.paths[static_cast<std::size_t>(age)];
        if (!app.pathUsed[static_cast<std::size_t>(age)] || path.size() < 2) continue;
        const Uint8 alpha = static_cast<Uint8>(230 - 200 * k / kPathSlots);
        gd::setColor(r, gd::Color{29, 78, 216, alpha});
        for (std::size_t i = 1; i < path.size(); ++i) {
            SDL_RenderLine(r, cellToScreenX(1, path[i - 1].x), cellToScreenY(path[i - 1].y), cellToScreenX(1, path[i].x), cellToScreenY(path[i].y));
        }
    }
}

void drawProfile(App& app) {
    SDL_Renderer* r = app.renderer;
    char cap[80];
    SDL_snprintf(cap, sizeof cap, "CUT, ROW %d", app.profileRow);
    app.hud.text(r, kChartX, 366.0F, cap, gd::kInk, gd::Hud::Style::Bold);
    const char* legend = "grey: before  orange: dug  teal: filled";
    app.hud.text(r, kChartX + kChartW - app.hud.width(legend), 366.0F, legend, gd::kMuted);
    gd::fillRect(r, kChartX, kChartY, kChartW, kChartH, gd::kWhite);

    double lo = 1e9;
    double hi = -1e9;
    for (int x = 0; x < kN; ++x) {
        for (const te::Grid* g : {&app.original, &app.eroded}) {
            lo = std::min(lo, g->at(x, app.profileRow));
            hi = std::max(hi, g->at(x, app.profileRow));
        }
    }
    const double span = std::max(hi - lo, 1e-6);
    const auto sx = [&](int x) { return kChartX + kChartW * static_cast<float>(x) / (kN - 1); };
    const auto sy = [&](double v) { return kChartY + kChartH - 6.0F - (kChartH - 12.0F) * static_cast<float>((v - lo) / span); };
    const float colW = kChartW / (kN - 1);
    for (int x = 0; x < kN; ++x) {
        const double a = app.original.at(x, app.profileRow);
        const double b = app.eroded.at(x, app.profileRow);
        if (std::fabs(a - b) < 1e-9) continue;
        const float top = std::min(sy(a), sy(b));
        const float bottom = std::max(sy(a), sy(b));
        gd::fillRect(r, sx(x) - colW / 2.0F, top, colW, std::max(1.0F, bottom - top), b < a ? gd::Color{234, 88, 12, 170} : gd::Color{13, 148, 136, 170});
    }
    for (int x = 1; x < kN; ++x) {
        gd::setColor(r, gd::kMuted);
        SDL_RenderLine(r, sx(x - 1), sy(app.original.at(x - 1, app.profileRow)), sx(x), sy(app.original.at(x, app.profileRow)));
        gd::setColor(r, gd::kInk);
        SDL_RenderLine(r, sx(x - 1), sy(app.eroded.at(x - 1, app.profileRow)), sx(x), sy(app.eroded.at(x, app.profileRow)));
    }
    gd::strokeRect(r, kChartX, kChartY, kChartW, kChartH, gd::kInk);
}

void drawStats(App& app) {
    SDL_Renderer* r = app.renderer;
    const float x = kSliderX;
    const float y = 308.0F;
    gd::card(r, x, y, kSliderW, 98.0F, gd::kWhite, gd::kInk);
    char line[64];
    const std::string drops = te::groupDigits(app.drops);
    SDL_snprintf(line, sizeof line, "drops %s (%.2f/cell)", drops.c_str(), static_cast<double>(app.drops) / (kN * kN));
    if (std::strlen(line) > 25) SDL_snprintf(line, sizeof line, "drops %s", drops.c_str());  // too wide for the card
    app.hud.text(r, x + 12.0F, y + 8.0F, line, gd::kInk, gd::Hud::Style::Bold);
    const double change = app.baseTotal > 0.0 ? (app.nowTotal - app.baseTotal) / app.baseTotal * 100.0 : 0.0;
    SDL_snprintf(line, sizeof line, "material %+.1f%%", std::fabs(change) < 0.05 ? 0.0 : change);
    app.hud.text(r, x + 12.0F, y + 30.0F, line, change < -0.05 ? gd::kOrange : gd::kTealDark, gd::Hud::Style::Bold);
    SDL_snprintf(line, sizeof line, "steepest %.0f° (was %.0f°)", app.nowSteepest, app.baseSteepest);
    app.hud.text(r, x + 12.0F, y + 52.0F, line, gd::kInk);
    SDL_snprintf(line, sizeof line, "mean slope %.0f° (was %.0f°)", app.nowMean, app.baseMean);
    app.hud.text(r, x + 12.0F, y + 72.0F, line, gd::kInk);
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    app.hud.titleBar(r, kWidth, "[ TERRAIN EROSION ]", "water, sediment, talus");

    const char* names[2] = {"BEFORE", "AFTER"};
    const gd::Color nameColor[2] = {gd::kMuted, gd::kTeal};
    for (int m = 0; m < 2; ++m) {
        const float x = kMapX[static_cast<std::size_t>(m)];
        app.hud.text(r, x, 35.0F, names[m], nameColor[m], gd::Hud::Style::Bold);
        gd::fillRect(r, x + 4.0F, kMapY + 4.0F, kPanel, kPanel, gd::Color{17, 24, 39, 40});
        const SDL_FRect dst{x, kMapY, kPanel, kPanel};
        SDL_RenderTexture(r, app.tex[m], nullptr, &dst);
        gd::strokeRect(r, x, kMapY, kPanel, kPanel, gd::kInk);
        const float ry = cellToScreenY(app.profileRow);
        gd::setColor(r, gd::Color{17, 24, 39, 150});
        SDL_RenderLine(r, x, ry, x + kPanel, ry);
    }
    if (app.diff) {
        const float x = kMapX[1] + kPanel - 12.0F * 10.0F;  // "dug / filled" is 12 characters
        app.hud.text(r, x, 35.0F, "dug", gd::kOrange, gd::Hud::Style::Bold);
        app.hud.text(r, x + 40.0F, 35.0F, "/", gd::kMuted, gd::Hud::Style::Bold);
        app.hud.text(r, x + 60.0F, 35.0F, "filled", gd::kTealDark, gd::Hud::Style::Bold);
    } else {
        const bool naive = app.drop.mode == te::Mode::Naive;
        const char* right = naive ? "NAIVE: dig only" : "carry and deposit";
        app.hud.text(r, kMapX[1] + kPanel - app.hud.width(right), 35.0F, right, naive ? gd::kOrange : gd::kMuted);
    }
    drawPaths(app);
    drawProfile(app);
    drawSliders(app);
    drawStats(app);

    app.hud.text(r, kSliderX, 418.0F, "drag on AFTER: rain there", gd::kMuted);
    app.hud.text(r, kSliderX, 438.0F, "hover a map: pick the row", gd::kMuted);
    app.hud.text(r, kSliderX, 458.0F, "Up/Down: rate  Esc: quit", gd::kMuted);

    for (const Chip& c : app.chips) {
        const bool on = active(app, c.action);
        const gd::Color fill = !on ? gd::kWhite : (c.action == Action::Naive ? gd::kOrange : gd::kTeal);
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, fill, gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 6.0F, c.label, on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
    }
}

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
        float gapBefore;
    };
    const std::array<Def, 8> defs{{{Action::Rain, "Space Rain", 0.0F},
                                   {Action::Naive, "1 Naive", 24.0F},
                                   {Action::Sediment, "2 Sediment", 8.0F},
                                   {Action::Thermal, "T Thermal", 24.0F},
                                   {Action::Lines, "C Lines", 8.0F},
                                   {Action::Diff, "D Diff", 8.0F},
                                   {Action::NewTerrain, "N New", 24.0F},
                                   {Action::Reset, "R Reset", 8.0F}}};
    float x = 24.0F;
    for (const Def& d : defs) {
        x += d.gapBefore;
        const float w = app.hud.width(d.label, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({d.action, d.label, SDL_FRect{x, kChipY, w, kChipH}});
        x += w;
    }
}

void updateHover(App& app, float mx, float my) {
    for (const float x : kMapX) {
        if (inRect(mx, my, x, kMapY, kPanel, kPanel)) {
            app.profileRow = static_cast<int>(std::clamp((my - kMapY) / kScale, 0.0F, static_cast<float>(kN - 1)));
        }
    }
}

SDL_Texture* makeTexture(SDL_Renderer* renderer) {
    SDL_Texture* t = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA32, SDL_TEXTUREACCESS_STREAMING, kN, kN);
    if (t == nullptr) return nullptr;
    SDL_SetTextureScaleMode(t, SDL_SCALEMODE_LINEAR);  // 2x enlargement: smooth, the cells are not the point
    SDL_SetTextureBlendMode(t, SDL_BLENDMODE_NONE);
    return t;
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
    if (!SDL_CreateWindowAndRenderer("terrain-erosion", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, SDL_LOGICAL_PRESENTATION_LETTERBOX);
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    for (SDL_Texture*& t : app->tex) {
        t = makeTexture(app->renderer);
        if (t == nullptr) {
            SDL_Log("map texture failed: %s", SDL_GetError());
            return SDL_APP_FAILURE;
        }
    }
    SDL_SetRenderVSync(app->renderer, 1);
    layoutChips(*app);
    syncParams(*app);
    te::generateTerrain(app->original, app->terrainSeed);
    resetErosion(*app, kWarmupDrops);
    app->lastTicks = SDL_GetTicks();
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
            if (app.drag == Drag::Slider) sliderFromMouse(app, app.mouseX);
            updateHover(app, app.mouseX, app.mouseY);
            break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const float mx = event->button.x;
            const float my = event->button.y;
            app.mouseX = mx;
            app.mouseY = my;
            const SDL_FPoint p{mx, my};
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) {
                    apply(app, c.action);
                    return SDL_APP_CONTINUE;
                }
            }
            for (int i = 0; i < static_cast<int>(kSliders.size()); ++i) {
                if (inRect(mx, my, kSliderX, rowTop(i) - 4.0F, kSliderW, kRowH - 2.0F)) {
                    app.drag = Drag::Slider;
                    app.slider = i;
                    sliderFromMouse(app, mx);
                    return SDL_APP_CONTINUE;
                }
            }
            if (inRect(mx, my, kMapX[1], kMapY, kPanel, kPanel)) app.drag = Drag::Brush;
            updateHover(app, mx, my);
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event->button.button == SDL_BUTTON_LEFT) app.drag = Drag::None;
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
        case SDL_EVENT_WINDOW_FOCUS_LOST: app.drag = Drag::None; break;
        case SDL_EVENT_KEY_DOWN: {
            const bool fresh = !event->key.repeat;
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_SPACE: if (fresh) apply(app, Action::Rain); break;
                case SDLK_1: if (fresh) apply(app, Action::Naive); break;
                case SDLK_2: if (fresh) apply(app, Action::Sediment); break;
                case SDLK_T: if (fresh) apply(app, Action::Thermal); break;
                case SDLK_C: if (fresh) apply(app, Action::Lines); break;
                case SDLK_D: if (fresh) apply(app, Action::Diff); break;
                case SDLK_N: if (fresh) apply(app, Action::NewTerrain); break;
                case SDLK_R: if (fresh) apply(app, Action::Reset); break;
                case SDLK_UP: sliderNudge(app, 0, 1); break;
                case SDLK_DOWN: sliderNudge(app, 0, -1); break;
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
    const double dt = std::min(static_cast<double>(now - app.lastTicks) / 1000.0, kMaxDt);
    app.lastTicks = now;
    step(app, dt);
    rebuild(app);
    drawScene(app);
    SDL_RenderPresent(app.renderer);
    ++app.frames;
    return (app.maxFrames >= 0 && app.frames >= app.maxFrames) ? SDL_APP_SUCCESS : SDL_APP_CONTINUE;
}

void SDL_AppQuit(void* appstate, SDL_AppResult) {
    auto* app = static_cast<App*>(appstate);
    if (app != nullptr) {
        app->hud.destroy();
        for (SDL_Texture* t : app->tex) {
            if (t != nullptr) SDL_DestroyTexture(t);
        }
        if (app->renderer != nullptr) SDL_DestroyRenderer(app->renderer);
        if (app->window != nullptr) SDL_DestroyWindow(app->window);
        delete app;
    }
}
