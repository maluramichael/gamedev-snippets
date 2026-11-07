// Abelian sandpile with SDL3 main callbacks.
//
// Grains fall on a grid. A cell with 4 grains topples and hands one to each neighbor, the grains at the edge fall off. The
// first picture is 20000 grains poured into the middle. The interesting part is the order: the rows, random and reverse
// buttons visit the unstable cells differently, the intermediate pictures differ, the end picture never does. Pour with each
// order and the fingerprints in the panel match. Random drops on a filled grid show the other half: most grains change
// nothing, a few start an avalanche over half the grid. The model, the fingerprint and the histogram buckets live in
// logic.hpp and are covered by test.cpp.

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

#include "hud.hpp"
#include "logic.hpp"

namespace {

constexpr int kWidth = 960;
constexpr int kHeight = 540;
constexpr int kGrid = 129;
constexpr int kCenter = kGrid / 2;
constexpr float kCell = 3.0F;
constexpr float kMapX = 30.0F;
constexpr float kMapY = 38.0F;
constexpr float kMapSize = kCell * static_cast<float>(kGrid);
constexpr float kPanelX = 450.0F;
constexpr float kPanelRight = 930.0F;

constexpr int kPourGrains = 20000;
constexpr int kPourChunk = 125;            // grains added per frame while pouring (the end picture does not depend on the chunking)
constexpr double kPassStep = 1.0 / 24.0;   // seconds per pass while an avalanche is animated
constexpr double kHoldRate = 30.0;         // grains per second while the mouse button is held on the grid
constexpr long long kFrameBudget = 150000; // topplings per frame at most for instant drops, the rest waits for the next frame
constexpr double kRateExp = 3.7;           // the speed slider runs from 1 to about 5000 grains per second
constexpr double kSpeedStart = 0.4;
constexpr float kFlashFade = 2.0F;         // heat per second that a toppled cell loses
constexpr int kFillValue = 2;              // close to the mean height of the critical state (measured in test.cpp --survey)

constexpr float kChipH = 34.0F;
constexpr float kRow1Y = 440.0F;
constexpr float kRow2Y = 484.0F;
constexpr float kTrackX = 706.0F;
constexpr float kTrackW = 150.0F;

constexpr std::array<gd::Color, 4> kHeights{{{244, 244, 241}, {167, 243, 228}, {13, 148, 136}, {17, 24, 39}}};
constexpr gd::Color kFlash{253, 186, 116};

enum class Action { Run, Pour, Clear, Fill, Animate, OrderRows, OrderRandom, OrderReverse, SourceCenter, SourceRandom };
enum class Source { Center, Random };

struct Chip {
    Action action;
    std::string label;
    SDL_FRect rect;
};

struct PourResult {
    bool valid = false;
    std::uint32_t hash = 0;
    long long topplings = 0;
};

struct Running {
    long long size = 0;
    int passes = 0;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    SDL_Texture* texture = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;

    sp::Pile pile{kGrid, kGrid};
    sp::Rng rng{1};
    sp::Rng dropRng{2024};
    sp::Histogram hist;
    sp::Order order = sp::Order::Rows;
    Source source = Source::Random;
    bool animate = true;
    bool running = false;
    double speedT = kSpeedStart;
    double dropClock = 0.0;
    double passClock = 0.0;
    double holdClock = 0.0;

    Running current;  // the avalanche in progress (animated mode)
    long long lastSize = 0;
    int lastPasses = 0;
    long long dropped = 0;  // grains dropped by clicks and Run (a pour does not count)

    int pourLeft = 0;
    sp::Order pourOrder = sp::Order::Rows;
    std::array<PourResult, 3> pourResults{};

    std::vector<float> heat;
    std::vector<std::uint32_t> pixels;
    int lastX = -1;
    int lastY = -1;
    float lastAge = 99.0F;

    bool holding = false;
    bool dragSlider = false;
    float mouseX = -100.0F;
    float mouseY = -100.0F;

    Uint64 lastTicks = 0;
    long maxFrames = -1;
    long frames = 0;
};

std::uint32_t pack(gd::Color c) {
    // SDL_PIXELFORMAT_RGBA32 is the byte order R, G, B, A in memory on every platform.
    std::uint32_t out = 0;
    const std::uint8_t bytes[4] = {c.r, c.g, c.b, 255};
    std::memcpy(&out, bytes, sizeof out);
    return out;
}

gd::Color mix(gd::Color a, gd::Color b, float t) {
    auto lerp = [t](std::uint8_t x, std::uint8_t y) { return static_cast<std::uint8_t>(std::lround(static_cast<float>(x) + (static_cast<float>(y) - static_cast<float>(x)) * t)); };
    return gd::Color{lerp(a.r, b.r), lerp(a.g, b.g), lerp(a.b, b.b)};
}

std::size_t orderSlot(sp::Order o) { return static_cast<std::size_t>(o); }

const char* orderName(sp::Order o) { return o == sp::Order::Rows ? "rows" : o == sp::Order::Random ? "random" : "reverse"; }

double grainRate(const App& app) { return std::round(std::pow(10.0, kRateExp * app.speedT)); }

bool pouring(const App& app) { return app.pourLeft > 0; }

void clearHeat(App& app) { std::fill(app.heat.begin(), app.heat.end(), 0.0F); }

/// Every topplings pass marks the cells it visited, so the avalanche stays visible for a moment after it is over.
long long runPass(App& app) {
    const long long units = sp::pass(app.pile, app.order, app.rng);
    for (const int i : app.pile.batch) app.heat[static_cast<std::size_t>(i)] = 1.0F;
    return units;
}

void finishAvalanche(App& app) {
    app.hist.record(app.current.size);
    app.lastSize = app.current.size;
    app.lastPasses = app.current.passes;
    app.current = Running{};
    app.passClock = 0.0;
}

/// Finishes the animated avalanche in one go (used when animation is switched off or a new pour starts).
void settleNow(App& app) {
    while (!sp::stable(app.pile)) {
        app.current.size += runPass(app);
        ++app.current.passes;
    }
    if (app.current.passes > 0 || app.current.size > 0) finishAvalanche(app);
}

void dropGrain(App& app, int x, int y) {
    if (pouring(app) || !app.pile.inside(x, y)) return;
    const bool wasStable = sp::stable(app.pile);
    sp::add(app.pile, x, y, 1);
    ++app.dropped;
    app.lastX = x;
    app.lastY = y;
    app.lastAge = 0.0F;
    if (app.animate) {
        if (wasStable) app.current = Running{};
        if (sp::stable(app.pile)) finishAvalanche(app);  // nothing toppled: a size-0 avalanche
        return;
    }
    // Instant: the whole avalanche resolves inside this call. Only the newest avalanche flashes, at a few hundred grains per
    // second a flash per avalanche would paint the whole grid.
    clearHeat(app);
    if (wasStable) app.current = Running{};
    while (!sp::stable(app.pile)) {
        app.current.size += runPass(app);
        ++app.current.passes;
    }
    finishAvalanche(app);
}

void dropAuto(App& app) {
    if (app.source == Source::Center) {
        dropGrain(app, kCenter, kCenter);
    } else {
        dropGrain(app, static_cast<int>(app.dropRng.below(kGrid)), static_cast<int>(app.dropRng.below(kGrid)));
    }
}

void emptyGrid(App& app, int value) {
    sp::reset(app.pile, value);
    app.hist.clear();
    app.dropped = 0;
    app.current = Running{};
    app.lastSize = 0;
    app.lastPasses = 0;
    app.pourLeft = 0;
    app.passClock = 0.0;
    app.dropClock = 0.0;
    app.lastAge = 99.0F;
    clearHeat(app);
}

void finishPour(App& app) {
    app.pourResults[orderSlot(app.pourOrder)] = PourResult{true, sp::hash(app.pile), app.pile.topplings};
}

void startPour(App& app) {
    emptyGrid(app, 0);
    app.pourLeft = kPourGrains;
    app.pourOrder = app.order;
}

void pourStep(App& app) {
    const int n = std::min(kPourChunk, app.pourLeft);
    sp::add(app.pile, kCenter, kCenter, n);
    app.pourLeft -= n;
    sp::settle(app.pile, app.pourOrder, app.rng);
    if (app.pourLeft == 0) finishPour(app);
}

void setAnimate(App& app, bool on) {
    app.animate = on;
    if (!on) settleNow(app);
}

void apply(App& app, Action action) {
    switch (action) {
        case Action::Run: app.running = !app.running; break;
        case Action::Pour: startPour(app); break;
        case Action::Clear: emptyGrid(app, 0); break;
        case Action::Fill: emptyGrid(app, kFillValue); break;
        case Action::Animate: setAnimate(app, !app.animate); break;
        case Action::OrderRows: app.order = sp::Order::Rows; break;
        case Action::OrderRandom: app.order = sp::Order::Random; break;
        case Action::OrderReverse: app.order = sp::Order::Reverse; break;
        case Action::SourceCenter: app.source = Source::Center; break;
        case Action::SourceRandom: app.source = Source::Random; break;
    }
}

bool active(const App& app, Action action) {
    switch (action) {
        case Action::Run: return app.running;
        case Action::Pour: return pouring(app);
        case Action::Clear: return false;
        case Action::Fill: return false;
        case Action::Animate: return app.animate;
        case Action::OrderRows: return app.order == sp::Order::Rows;
        case Action::OrderRandom: return app.order == sp::Order::Random;
        case Action::OrderReverse: return app.order == sp::Order::Reverse;
        case Action::SourceCenter: return app.source == Source::Center;
        case Action::SourceRandom: return app.source == Source::Random;
    }
    return false;
}

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
        float gapBefore;
        float y;
    };
    const std::array<Def, 10> defs{{{Action::Run, "Space Run", 0.0F, kRow1Y},
                                    {Action::Pour, "P Pour 20000", 8.0F, kRow1Y},
                                    {Action::Clear, "C Clear", 8.0F, kRow1Y},
                                    {Action::Fill, "F Fill 2", 8.0F, kRow1Y},
                                    {Action::Animate, "A Animate", 8.0F, kRow1Y},
                                    {Action::OrderRows, "1 Rows", 66.0F, kRow2Y},
                                    {Action::OrderRandom, "2 Random", 8.0F, kRow2Y},
                                    {Action::OrderReverse, "3 Reverse", 8.0F, kRow2Y},
                                    {Action::SourceCenter, "4 Center", 92.0F, kRow2Y},
                                    {Action::SourceRandom, "5 Random", 8.0F, kRow2Y}}};
    float x = 24.0F;
    float lastY = 0.0F;
    for (const Def& d : defs) {
        if (d.y != lastY) x = 24.0F;
        lastY = d.y;
        x += d.gapBefore;
        const float w = app.hud.width(d.label, gd::Hud::Style::Bold) + 16.0F;
        app.chips.push_back({d.action, d.label, SDL_FRect{x, d.y, w, kChipH}});
        x += w;
    }
}

bool onSlider(float x, float y) { return x >= kTrackX - 10.0F && x <= kTrackX + kTrackW + 10.0F && y >= kRow1Y - 2.0F && y <= kRow1Y + kChipH + 2.0F; }

void dragSliderTo(App& app, float mx) { app.speedT = std::clamp(static_cast<double>((mx - kTrackX) / kTrackW), 0.0, 1.0); }

bool cellAt(float mx, float my, int& cx, int& cy) {
    const float fx = (mx - kMapX) / kCell;
    const float fy = (my - kMapY) / kCell;
    if (fx < 0.0F || fy < 0.0F || fx >= static_cast<float>(kGrid) || fy >= static_cast<float>(kGrid)) return false;
    cx = static_cast<int>(fx);
    cy = static_cast<int>(fy);
    return true;
}

void paintTexture(App& app) {
    for (int i = 0; i < kGrid * kGrid; ++i) {
        const auto k = static_cast<std::size_t>(i);
        const int c = app.pile.cells[k];
        gd::Color col = c >= sp::kLimit ? gd::kOrange : kHeights[static_cast<std::size_t>(std::max(c, 0))];
        if (app.heat[k] > 0.0F) col = mix(col, kFlash, std::min(1.0F, app.heat[k] * 0.7F));
        app.pixels[k] = pack(col);
    }
    SDL_UpdateTexture(app.texture, nullptr, app.pixels.data(), kGrid * static_cast<int>(sizeof(std::uint32_t)));
}

void update(App& app, double dt) {
    app.lastAge += static_cast<float>(dt);
    const float fade = static_cast<float>(dt) * kFlashFade;
    for (float& h : app.heat) h = h > fade ? h - fade : 0.0F;

    if (pouring(app)) {
        pourStep(app);
        return;
    }

    // Animated avalanche: one pass per tick.
    if (app.animate && !sp::stable(app.pile)) {
        app.passClock += dt;
        while (app.passClock >= kPassStep && !sp::stable(app.pile)) {
            app.passClock -= kPassStep;
            app.current.size += runPass(app);
            ++app.current.passes;
        }
        if (sp::stable(app.pile)) finishAvalanche(app);
    }

    // Held mouse button: a steady rain on one cell.
    if (app.holding) {
        app.holdClock += dt;
        int cx = 0;
        int cy = 0;
        const bool onGrid = cellAt(app.mouseX, app.mouseY, cx, cy);
        while (app.holdClock >= 1.0 / kHoldRate) {
            app.holdClock -= 1.0 / kHoldRate;
            if (onGrid) dropGrain(app, cx, cy);
        }
    }

    // Run: grains at the chosen rate. Animated, a new grain waits until the last avalanche is over.
    if (app.running) {
        app.dropClock += dt * grainRate(app);
        if (app.animate) {
            if (sp::stable(app.pile) && app.dropClock >= 1.0) {
                app.dropClock = 0.0;
                dropAuto(app);
            }
            app.dropClock = std::min(app.dropClock, 1.0);
        } else {
            const long long start = app.pile.topplings;
            while (app.dropClock >= 1.0 && app.pile.topplings - start < kFrameBudget) {
                app.dropClock -= 1.0;
                dropAuto(app);
            }
            if (app.dropClock >= 1.0) app.dropClock = 0.0;  // over budget: the rate is lower than asked, say so by dropping the backlog
        }
    }
}

void textRight(App& app, float x, float y, const std::string& s, gd::Color c, gd::Hud::Style style = gd::Hud::Style::Regular) {
    app.hud.text(app.renderer, x - app.hud.width(s, style), y, s, c, style);
}

void drawLegend(App& app) {
    SDL_Renderer* r = app.renderer;
    app.hud.text(r, kPanelX, kMapY + 1.0F, "grains per cell", gd::kMuted);
    float x = kPanelX + 170.0F;
    for (int v = 0; v < 5; ++v) {
        const gd::Color fill = v < 4 ? kHeights[static_cast<std::size_t>(v)] : gd::kOrange;
        gd::fillRect(r, x, kMapY, 22.0F, 22.0F, fill);
        gd::strokeRect(r, x, kMapY, 22.0F, 22.0F, gd::kInk);
        char label[4];
        SDL_snprintf(label, sizeof label, v < 4 ? "%d" : "%d+", v);
        const gd::Color text = (v == 0 || v == 1) ? gd::kInk : gd::kWhite;
        app.hud.text(r, x + (22.0F - app.hud.width(label, gd::Hud::Style::Bold)) / 2.0F, kMapY, label, text, gd::Hud::Style::Bold);
        x += 28.0F;
    }
    app.hud.text(r, x + 4.0F, kMapY + 1.0F, "topples", gd::kMuted);
}

void drawStats(App& app) {
    const float y0 = 66.0F;
    struct Line {
        const char* label;
        std::string value;
    };
    const bool live = !sp::stable(app.pile) && !pouring(app);
    const long long size = live ? app.current.size : app.lastSize;
    const int passes = live ? app.current.passes : app.lastPasses;
    std::string avalanche = sp::grouped(size) + " topplings, " + std::to_string(passes) + (passes == 1 ? " pass" : " passes");
    const std::array<Line, 4> lines{{{"on the grid", sp::grouped(sp::grains(app.pile))},
                                     {"fell off the edge", sp::grouped(app.pile.lost)},
                                     {"grains dropped", sp::grouped(app.dropped)},
                                     {live ? "avalanche now" : "last avalanche", avalanche}}};
    for (std::size_t i = 0; i < lines.size(); ++i) {
        const float y = y0 + 22.0F * static_cast<float>(i);
        app.hud.text(app.renderer, kPanelX, y, lines[i].label, gd::kMuted);
        textRight(app, kPanelRight, y, lines[i].value, (i == 3 && live) ? gd::kOrange : gd::kInk, gd::Hud::Style::Bold);
    }
}

void drawCompare(App& app) {
    SDL_Renderer* r = app.renderer;
    const float y0 = 166.0F;
    app.hud.text(r, kPanelX, y0, "End picture fingerprint", gd::kInk, gd::Hud::Style::Bold);
    textRight(app, kPanelRight, y0, "20,000 grains each", gd::kMuted);
    int filled = 0;
    bool same = true;
    std::uint32_t first = 0;
    for (const sp::Order o : {sp::Order::Rows, sp::Order::Random, sp::Order::Reverse}) {
        const PourResult& res = app.pourResults[orderSlot(o)];
        if (!res.valid) continue;
        if (filled == 0) first = res.hash;
        same = same && res.hash == first;
        ++filled;
    }
    int row = 0;
    for (const sp::Order o : {sp::Order::Rows, sp::Order::Random, sp::Order::Reverse}) {
        const float y = y0 + 24.0F + 22.0F * static_cast<float>(row++);
        const PourResult& res = app.pourResults[orderSlot(o)];
        const bool mine = pouring(app) && app.pourOrder == o;
        app.hud.text(r, kPanelX, y, orderName(o), mine ? gd::kOrange : gd::kInk, gd::Hud::Style::Bold);
        if (mine) {
            char line[40];
            SDL_snprintf(line, sizeof line, "pouring %d%%", 100 - 100 * app.pourLeft / kPourGrains);
            app.hud.text(r, kPanelX + 90.0F, y, line, gd::kOrange);
        } else if (res.valid) {
            app.hud.text(r, kPanelX + 90.0F, y, sp::hex8(res.hash), same ? gd::kTealDark : gd::kOrange, gd::Hud::Style::Bold);
            textRight(app, kPanelRight, y, sp::grouped(res.topplings) + " topplings", gd::kMuted);
        } else {
            app.hud.text(r, kPanelX + 90.0F, y, "not poured yet", gd::kMuted);
        }
    }
    const float vy = y0 + 24.0F + 66.0F;
    if (filled >= 2) {
        app.hud.text(r, kPanelX, vy, same ? "identical picture, identical topplings" : "DIFFERENT", same ? gd::kTealDark : gd::kOrange, gd::Hud::Style::Bold);
    } else {
        app.hud.text(r, kPanelX, vy, "pour again with another order", gd::kMuted);
    }
}

void drawHistogram(App& app) {
    SDL_Renderer* r = app.renderer;
    constexpr float kTop = 318.0F;
    constexpr float kBase = 396.0F;
    constexpr float kBarW = 22.0F;
    constexpr float kPitch = (kPanelRight - kPanelX) / static_cast<float>(sp::kBuckets);
    app.hud.text(r, kPanelX, 288.0F, "Avalanche sizes, log count", gd::kInk, gd::Hud::Style::Bold);
    textRight(app, kPanelRight, 288.0F, app.hist.drops > 0 ? "largest " + sp::grouped(app.hist.largest) : "in topplings", gd::kMuted);
    gd::fillRect(r, kPanelX, kBase, kPanelRight - kPanelX, 1.0F, gd::kInk);
    const long long peak = app.hist.peak();
    if (peak == 0) {
        const char* hint = "Fill 2, then Run: random grains";
        app.hud.text(r, kPanelX + (kPanelRight - kPanelX - app.hud.width(hint)) / 2.0F, (kTop + kBase) / 2.0F - 11.0F, hint, gd::kMuted);
    }
    for (int b = 0; b < sp::kBuckets; ++b) {
        const float x = kPanelX + kPitch * static_cast<float>(b) + (kPitch - kBarW) / 2.0F;
        const long long n = app.hist.count[static_cast<std::size_t>(b)];
        if (n > 0 && peak > 0) {
            const float h = std::max(2.0F, (kBase - kTop) * static_cast<float>(std::log10(static_cast<double>(n) + 1.0) / std::log10(static_cast<double>(peak) + 1.0)));
            gd::fillRect(r, x, kBase - h, kBarW, h, b == 0 ? gd::kTealDark : gd::kTeal);
        }
        static const char* const kLabels[sp::kBuckets] = {"0", "1", "", "4", "", "16", "", "64", "", "256", "", "1k", "", "4k", "", "16k", "", "64k+"};
        const char* label = kLabels[b];
        if (label[0] != '\0') app.hud.text(r, x + kBarW / 2.0F - app.hud.width(label) / 2.0F, kBase + 4.0F, label, gd::kMuted);
    }
}

void drawSlider(App& app) {
    SDL_Renderer* r = app.renderer;
    app.hud.text(r, kTrackX - 94.0F, kRow1Y + 6.0F, "Grains/s", gd::kInk, gd::Hud::Style::Bold);
    const float cy = kRow1Y + kChipH / 2.0F;
    const float knobX = kTrackX + kTrackW * static_cast<float>(app.speedT);
    gd::fillRect(r, kTrackX, cy - 2.0F, kTrackW, 4.0F, gd::kGrid);
    gd::strokeRect(r, kTrackX, cy - 2.0F, kTrackW, 4.0F, gd::kMuted);
    gd::fillRect(r, kTrackX, cy - 2.0F, knobX - kTrackX, 4.0F, gd::kTeal);
    gd::card(r, knobX - 6.0F, cy - 9.0F, 12.0F, 18.0F, app.dragSlider ? gd::kTeal : gd::kWhite, gd::kInk);
    app.hud.text(r, kTrackX + kTrackW + 16.0F, kRow1Y + 6.0F, sp::grouped(static_cast<long long>(grainRate(app))), gd::kTealDark, gd::Hud::Style::Bold);
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    app.hud.titleBar(r, kWidth, "[ ABELIAN SANDPILE ]", "click the grid to drop grains, hold to rain");

    const SDL_FRect dst{kMapX, kMapY, kMapSize, kMapSize};
    SDL_RenderTexture(r, app.texture, nullptr, &dst);
    gd::strokeRect(r, kMapX - 1.0F, kMapY - 1.0F, kMapSize + 2.0F, kMapSize + 2.0F, gd::kInk);

    if (app.lastAge < 0.6F && app.lastX >= 0) {
        const float cx = kMapX + (static_cast<float>(app.lastX) + 0.5F) * kCell;
        const float cy = kMapY + (static_cast<float>(app.lastY) + 0.5F) * kCell;
        gd::ring(r, cx, cy, 6.0F + 10.0F * app.lastAge, gd::kOrange);
    }
    int hx = 0;
    int hy = 0;
    if (!pouring(app) && cellAt(app.mouseX, app.mouseY, hx, hy)) {
        gd::ring(r, kMapX + (static_cast<float>(hx) + 0.5F) * kCell, kMapY + (static_cast<float>(hy) + 0.5F) * kCell, 5.0F, gd::kInk);
    }

    drawLegend(app);
    drawStats(app);
    drawCompare(app);
    drawHistogram(app);
    drawSlider(app);
    app.hud.text(r, 24.0F, kRow2Y + 6.0F, "Order", gd::kInk, gd::Hud::Style::Bold);
    app.hud.text(r, 396.0F, kRow2Y + 6.0F, "Drop at", gd::kInk, gd::Hud::Style::Bold);
    for (const Chip& c : app.chips) {
        const bool on = active(app, c.action);
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? gd::kTeal : gd::kWhite, gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 6.0F, c.label, on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
    }
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
    if (!SDL_CreateWindowAndRenderer("sandpile-avalanches", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, gd::kLogicalPresentation);
    SDL_SetRenderVSync(app->renderer, 1);
    app->texture = SDL_CreateTexture(app->renderer, SDL_PIXELFORMAT_RGBA32, SDL_TEXTUREACCESS_STREAMING, kGrid, kGrid);
    if (app->texture == nullptr) {
        SDL_Log("grid texture failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetTextureScaleMode(app->texture, SDL_SCALEMODE_NEAREST);
    app->pixels.assign(static_cast<std::size_t>(kGrid * kGrid), 0);
    app->heat.assign(static_cast<std::size_t>(kGrid * kGrid), 0.0F);
    layoutChips(*app);

    // The first picture is the finished pour, so the grid is not empty. Same steps as the Pour button, only all at once.
    startPour(*app);
    while (pouring(*app)) pourStep(*app);
    app->lastTicks = SDL_GetTicksNS();
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
            if (app.dragSlider) dragSliderTo(app, event->motion.x);
            break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const SDL_FPoint p{event->button.x, event->button.y};
            app.mouseX = p.x;
            app.mouseY = p.y;
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) {
                    apply(app, c.action);
                    return SDL_APP_CONTINUE;
                }
            }
            if (onSlider(p.x, p.y)) {
                app.dragSlider = true;
                dragSliderTo(app, p.x);
                break;
            }
            int cx = 0;
            int cy = 0;
            if (cellAt(p.x, p.y, cx, cy)) {
                dropGrain(app, cx, cy);
                app.holding = true;
                app.holdClock = 0.0;
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event->button.button == SDL_BUTTON_LEFT) {
                app.holding = false;
                app.dragSlider = false;
            }
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
        case SDL_EVENT_WINDOW_FOCUS_LOST:
            app.holding = false;
            app.dragSlider = false;
            app.mouseX = -100.0F;
            app.mouseY = -100.0F;
            break;
        case SDL_EVENT_KEY_DOWN:
            // Left and right may repeat (hold to sweep the speed), every toggle must not.
            if (event->key.key == SDLK_LEFT) {
                app.speedT = std::max(0.0, app.speedT - 0.02);
            } else if (event->key.key == SDLK_RIGHT) {
                app.speedT = std::min(1.0, app.speedT + 0.02);
            } else if (!event->key.repeat) {
                switch (event->key.key) {
                    case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                    case SDLK_SPACE: apply(app, Action::Run); break;
                    case SDLK_P: apply(app, Action::Pour); break;
                    case SDLK_C: apply(app, Action::Clear); break;
                    case SDLK_F: apply(app, Action::Fill); break;
                    case SDLK_A: apply(app, Action::Animate); break;
                    case SDLK_1: apply(app, Action::OrderRows); break;
                    case SDLK_2: apply(app, Action::OrderRandom); break;
                    case SDLK_3: apply(app, Action::OrderReverse); break;
                    case SDLK_4: apply(app, Action::SourceCenter); break;
                    case SDLK_5: apply(app, Action::SourceRandom); break;
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
    const Uint64 now = SDL_GetTicksNS();
    const double dt = std::min(0.25, static_cast<double>(now - app.lastTicks) * 1e-9);
    app.lastTicks = now;
    update(app, dt);
    paintTexture(app);
    drawScene(app);
    SDL_RenderPresent(app.renderer);
    ++app.frames;
    return (app.maxFrames >= 0 && app.frames >= app.maxFrames) ? SDL_APP_SUCCESS : SDL_APP_CONTINUE;
}

void SDL_AppQuit(void* appstate, SDL_AppResult) {
    auto* app = static_cast<App*>(appstate);
    if (app != nullptr) {
        app->hud.destroy();
        if (app->texture != nullptr) SDL_DestroyTexture(app->texture);
        if (app->renderer != nullptr) SDL_DestroyRenderer(app->renderer);
        if (app->window != nullptr) SDL_DestroyWindow(app->window);
        delete app;
    }
}
