// Town rings from bid rents with SDL3 main callbacks.
//
// Three straight bid curves over the distance to the town centre (market, house, farm) and one rule: every cell takes
// the use with the highest bid. Nobody draws rings, they fall out of the maximum, and they move when a curve moves.
// The graph on the right shows the curves for the cell under the cursor, the heavy line on top is the maximum and
// its colour is the winner. "Hand-set rings" is the naive version: radii typed in once, scaled with the town size,
// blind to slopes and river. The curve shapes are an invention for this demo. Logic and its edge cases (a tie, a use
// that never wins, a flat curve) are in logic.hpp and covered by test.cpp.

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
#include <utility>
#include <vector>

#include "hud.hpp"
#include "logic.hpp"

namespace {

constexpr int kWidth = 960;
constexpr int kHeight = 540;
constexpr int kCols = 48;
constexpr int kRows = 31;
constexpr float kCell = 12.0F;
constexpr float kMapX = 16.0F;
constexpr float kMapY = 40.0F;
constexpr float kPanelX = 616.0F;
constexpr float kPanelW = 328.0F;
constexpr float kChipY = 476.0F;
constexpr float kChipH = 34.0F;

// Graph plot area (value 0..200 over distance 0..36 cells).
constexpr float kPlotX0 = 662.0F;
constexpr float kPlotX1 = 932.0F;
constexpr float kPlotY0 = 76.0F;
constexpr float kPlotY1 = 190.0F;
constexpr double kGraphDMax = 36.0;
constexpr double kGraphVMax = 200.0;

constexpr float kSliderTop = 236.0F;
constexpr float kSliderRow = 34.0F;

constexpr double kSizeMin = 0.5;
constexpr double kSizeMax = 1.5;
constexpr double kGrowRate = 0.15;  // size units per second

constexpr gd::Color kMarketColor = gd::kOrange;
constexpr gd::Color kHouseColor = gd::kTeal;
constexpr gd::Color kFarmCurve{176, 128, 24};
constexpr gd::Color kFarmCell{232, 205, 130};
constexpr gd::Color kWaterCell{120, 170, 225};
constexpr gd::Color kWildCell{236, 234, 230};
constexpr gd::Color kInkSoft{17, 24, 39, 150};

enum class Action { HandSet, Curves, Grow, Reset };

struct Chip {
    Action action;
    std::string label;
    SDL_FRect rect;
};

struct Slider {
    const char* label;
    double brt::Params::*field;
    double lo;
    double hi;
    int decimals;
    gd::Color color;
    bool needsCurves;  // ignored by the hand-set rings
};

const std::array<Slider, 5> kSliders{{{"Town size", &brt::Params::size, kSizeMin, kSizeMax, 2, gd::kInk, false},
                                      {"River pull", &brt::Params::water, 0.0, 1.0, 2, gd::kInk, true},
                                      {"Market slope", &brt::Params::slopeMarket, 0.0, 24.0, 1, kMarketColor, true},
                                      {"House slope", &brt::Params::slopeHouse, 0.0, 8.0, 1, kHouseColor, true},
                                      {"Farm slope", &brt::Params::slopeFarm, 0.0, 2.0, 2, kFarmCurve, true}}};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;
    brt::Terrain terrain = brt::Terrain::river(kCols, kRows);
    brt::Params params;
    brt::Mode mode = brt::Mode::BidCurves;
    brt::Cell centre{20, 15};
    bool growing = false;
    double growDir = 1.0;
    std::optional<std::size_t> activeSlider;
    bool dragCentre = false;
    float mouseX = -100.0F;
    float mouseY = -100.0F;
    Uint64 lastTicks = 0;
    long maxFrames = -1;
    long frames = 0;
};

float sliderTrackY(std::size_t i) { return kSliderTop + kSliderRow * static_cast<float>(i) + 28.0F; }

void clearDrags(App& app) {
    app.activeSlider.reset();
    app.dragCentre = false;
}

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
        float gapBefore;
    };
    const std::array<Def, 4> defs{{{Action::HandSet, "1 Hand-set rings", 0.0F},
                                   {Action::Curves, "2 Bid curves", 8.0F},
                                   {Action::Grow, "G Grow", 24.0F},
                                   {Action::Reset, "R Reset", 8.0F}}};
    float x = 16.0F;
    for (const Def& d : defs) {
        x += d.gapBefore;
        const float w = app.hud.width(d.label, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({d.action, d.label, SDL_FRect{x, kChipY, w, kChipH}});
        x += w;
    }
}

void apply(App& app, Action action) {
    switch (action) {
        case Action::HandSet:
            app.mode = brt::Mode::HandSetRings;
            break;
        case Action::Curves:
            app.mode = brt::Mode::BidCurves;
            break;
        case Action::Grow:
            app.growing = !app.growing;
            break;
        case Action::Reset:
            app.params = brt::Params{};
            app.centre = {20, 15};
            app.growing = false;
            app.growDir = 1.0;
            clearDrags(app);
            break;
    }
}

bool active(const App& app, Action action) {
    switch (action) {
        case Action::HandSet: return app.mode == brt::Mode::HandSetRings;
        case Action::Curves: return app.mode == brt::Mode::BidCurves;
        case Action::Grow: return app.growing;
        case Action::Reset: return false;
    }
    return false;
}

void setSliderFromMouse(App& app, std::size_t i, float x) {
    const Slider& s = kSliders[i];
    double v = brt::valueAtX(static_cast<double>(x), static_cast<double>(kPanelX), static_cast<double>(kPanelW), s.lo, s.hi);
    const double step = s.decimals >= 2 ? 0.01 : 0.1;
    v = std::round(v / step) * step;
    app.params.*(s.field) = std::clamp(v, s.lo, s.hi);
}

std::optional<std::size_t> sliderAt(float x, float y) {
    if (x < kPanelX - 8.0F || x > kPanelX + kPanelW + 8.0F) return std::nullopt;
    for (std::size_t i = 0; i < kSliders.size(); ++i) {
        const float ty = sliderTrackY(i);
        if (y >= ty - 14.0F && y <= ty + 12.0F) return i;
    }
    return std::nullopt;
}

void moveCentre(App& app, float x, float y) {
    // Clamp into the map so a drag that leaves the map keeps the centre on its edge.
    const float cx = std::clamp(x, kMapX, kMapX + kCell * static_cast<float>(kCols) - 0.01F);
    const float cy = std::clamp(y, kMapY, kMapY + kCell * static_cast<float>(kRows) - 0.01F);
    if (const auto c = brt::cellAt(cx, cy, kMapX, kMapY, kCell, kCols, kRows)) app.centre = *c;
}

gd::Color cellColor(brt::Use u) {
    switch (u) {
        case brt::Use::Market: return kMarketColor;
        case brt::Use::House: return kHouseColor;
        case brt::Use::Farm: return kFarmCell;
        case brt::Use::Water: return kWaterCell;
        case brt::Use::Wild: return kWildCell;
    }
    return kWildCell;
}

gd::Color curveColor(brt::Use u) { return u == brt::Use::Farm ? kFarmCurve : cellColor(u); }

const char* useName(brt::Use u) {
    switch (u) {
        case brt::Use::Market: return "MARKET";
        case brt::Use::House: return "HOUSE";
        case brt::Use::Farm: return "FARM";
        case brt::Use::Water: return "WATER";
        case brt::Use::Wild: return "WILD";
    }
    return "";
}

void textRight(App& app, float xRight, float y, const char* s, gd::Color c, gd::Hud::Style style = gd::Hud::Style::Regular) {
    app.hud.text(app.renderer, xRight - app.hud.width(s, style), y, s, c, style);
}

void thickLine(SDL_Renderer* r, float x0, float y0, float x1, float y1, gd::Color c, int half) {
    gd::setColor(r, c);
    for (int o = -half; o <= half; ++o) {
        const auto off = static_cast<float>(o);
        SDL_RenderLine(r, x0, y0 + off, x1, y1 + off);
    }
}

float plotX(double d) { return kPlotX0 + static_cast<float>(d / kGraphDMax) * (kPlotX1 - kPlotX0); }
float plotY(double v) { return kPlotY1 - static_cast<float>(v / kGraphVMax) * (kPlotY1 - kPlotY0); }

void drawMap(App& app, const std::vector<brt::Use>& uses) {
    SDL_Renderer* r = app.renderer;
    for (int y = 0; y < kRows; ++y) {
        for (int x = 0; x < kCols; ++x) {
            const brt::Use u = uses[static_cast<std::size_t>(y * kCols + x)];
            gd::fillRect(r, kMapX + kCell * static_cast<float>(x), kMapY + kCell * static_cast<float>(y), kCell, kCell, cellColor(u));
        }
    }
    // Ring outlines: a line wherever two neighbouring land cells differ. The river edge needs none, it is blue already.
    for (int y = 0; y < kRows; ++y) {
        for (int x = 0; x < kCols; ++x) {
            const brt::Use u = uses[static_cast<std::size_t>(y * kCols + x)];
            if (u == brt::Use::Water) continue;
            const float px = kMapX + kCell * static_cast<float>(x);
            const float py = kMapY + kCell * static_cast<float>(y);
            if (x + 1 < kCols) {
                const brt::Use n = uses[static_cast<std::size_t>(y * kCols + x + 1)];
                if (n != u && n != brt::Use::Water) gd::fillRect(r, px + kCell - 1.0F, py, 2.0F, kCell, gd::kInk);
            }
            if (y + 1 < kRows) {
                const brt::Use n = uses[static_cast<std::size_t>((y + 1) * kCols + x)];
                if (n != u && n != brt::Use::Water) gd::fillRect(r, px, py + kCell - 1.0F, kCell, 2.0F, gd::kInk);
            }
        }
    }
    gd::strokeRect(r, kMapX, kMapY, kCell * static_cast<float>(kCols), kCell * static_cast<float>(kRows), gd::kInk);

    // Centre.
    const float cx = kMapX + kCell * (static_cast<float>(app.centre.x) + 0.5F);
    const float cy = kMapY + kCell * (static_cast<float>(app.centre.y) + 0.5F);
    gd::disc(r, cx, cy, 6.0F, gd::kWhite);
    gd::ring(r, cx, cy, 6.0F, gd::kInk);
    gd::ring(r, cx, cy, 7.0F, gd::kInk);
}

void drawGraph(App& app, std::optional<brt::Cell> hover) {
    SDL_Renderer* r = app.renderer;
    gd::card(r, kPanelX, 40.0F, kPanelW, 184.0F, gd::kWhite, gd::kInk);
    const bool curves = app.mode == brt::Mode::BidCurves;
    app.hud.text(r, kPanelX + 12.0F, 42.0F, curves ? "BID BY DISTANCE" : "HAND-SET RINGS", gd::kInk, gd::Hud::Style::Bold);

    for (const double v : {0.0, 100.0, 200.0}) {
        gd::fillRect(r, kPlotX0, plotY(v), kPlotX1 - kPlotX0, 1.0F, gd::kGrid);
        char t[8];
        SDL_snprintf(t, sizeof t, "%d", static_cast<int>(v));
        textRight(app, kPlotX0 - 6.0F, plotY(v) - 11.0F, t, gd::kMuted);
    }
    gd::fillRect(r, kPlotX0, kPlotY0, 1.0F, kPlotY1 - kPlotY0, gd::kInk);

    // Distance and river distance of the cell under the cursor, the curves shift with its river bonus.
    double dw = brt::kNoWater;
    double hoverD = -1.0;
    if (hover && !app.terrain.isWater(hover->x, hover->y)) {
        dw = app.terrain.waterDist[static_cast<std::size_t>(hover->y * kCols + hover->x)];
        hoverD = std::hypot(static_cast<double>(hover->x - app.centre.x), static_cast<double>(hover->y - app.centre.y));
    }

    if (!curves) {
        const double radii[2] = {brt::kFixedMarketRadius * app.params.size, brt::kFixedHouseRadius * app.params.size};
        for (const double rad : radii) {
            if (rad > kGraphDMax) continue;
            thickLine(r, plotX(rad), kPlotY0, plotX(rad), kPlotY1, kMarketColor, 1);
            char t[16];
            SDL_snprintf(t, sizeof t, "%.1f", rad);
            app.hud.text(r, plotX(rad) - app.hud.width(t) / 2.0F, kPlotY1 + 4.0F, t, kMarketColor);
        }
        app.hud.text(r, kPlotX0 + 14.0F, kPlotY0 + 30.0F, "radii typed in by hand", gd::kMuted);
        app.hud.text(r, kPlotX0 + 14.0F, kPlotY0 + 54.0F, "no curves, no river", gd::kMuted);
        return;
    }

    const brt::Params& p = app.params;
    constexpr int kColumns = 280;
    const float colW = (kPlotX1 - kPlotX0) / static_cast<float>(kColumns);
    // The three curves, thin and soft, only where they sit inside the plot.
    for (int c = 0; c < kColumns; ++c) {
        const double d0 = kGraphDMax * static_cast<double>(c) / kColumns;
        const double d1 = kGraphDMax * static_cast<double>(c + 1) / kColumns;
        const brt::Bids a = brt::bids(p, d0, dw);
        const brt::Bids b = brt::bids(p, d1, dw);
        const float x0 = kPlotX0 + colW * static_cast<float>(c);
        const float x1 = x0 + colW;
        auto seg = [&](double va, double vb, gd::Color col) {
            if (va < 0.0 || vb < 0.0 || va > kGraphVMax || vb > kGraphVMax) return;
            thickLine(r, x0, plotY(va), x1, plotY(vb), gd::Color{col.r, col.g, col.b, 190}, 0);
        };
        seg(a.market, b.market, kMarketColor);
        seg(a.house, b.house, kHouseColor);
        seg(a.farm, b.farm, kFarmCurve);
    }
    // The maximum, heavy, in the colour of the winner.
    for (int c = 0; c < kColumns; ++c) {
        const double d0 = kGraphDMax * static_cast<double>(c) / kColumns;
        const double d1 = kGraphDMax * static_cast<double>(c + 1) / kColumns;
        const brt::Bids a = brt::bids(p, d0, dw);
        const brt::Bids b = brt::bids(p, d1, dw);
        const double ta = std::max({a.market, a.house, a.farm});
        const double tb = std::max({b.market, b.house, b.farm});
        const brt::Use u = brt::pick(brt::bids(p, 0.5 * (d0 + d1), dw));
        if (u == brt::Use::Wild || ta > kGraphVMax || tb > kGraphVMax) continue;
        const float x0 = kPlotX0 + colW * static_cast<float>(c);
        thickLine(r, x0, plotY(ta), x0 + colW, plotY(tb), curveColor(u), 2);
    }
    // Where the winner changes: dotted line and the distance.
    float lastLabelX = -1000.0F;
    for (const brt::Boundary& bd : brt::boundaries(p, dw, kGraphDMax)) {
        const float x = plotX(bd.d);
        for (float y = kPlotY0; y < kPlotY1; y += 8.0F) gd::fillRect(r, x, y, 1.0F, 4.0F, gd::kInk);
        char t[16];
        SDL_snprintf(t, sizeof t, "%.1f", bd.d);
        const float w = app.hud.width(t);
        if (x - lastLabelX >= w + 6.0F) {
            app.hud.text(r, std::clamp(x - w / 2.0F, kPlotX0 - 4.0F, kPlotX1 - w), kPlotY1 + 4.0F, t, gd::kInk);
            lastLabelX = x;
        }
    }
    if (hoverD >= 0.0 && hoverD <= kGraphDMax) {
        const float x = plotX(hoverD);
        thickLine(r, x, kPlotY0, x, kPlotY1, kInkSoft, 0);
        const brt::Bids b = brt::bids(p, hoverD, dw);
        for (const auto& [v, col] : {std::pair<double, gd::Color>{b.market, kMarketColor}, {b.house, kHouseColor}, {b.farm, kFarmCurve}}) {
            if (v < 0.0 || v > kGraphVMax) continue;
            gd::disc(r, x, plotY(v), 4.0F, col);
            gd::ring(r, x, plotY(v), 4.0F, gd::kInk, 16);
        }
    }
    if (dw < brt::kWaterReach) {
        char t[32];
        SDL_snprintf(t, sizeof t, "river %.1f", dw);
        textRight(app, kPanelX + kPanelW - 12.0F, 42.0F, t, gd::kMuted);
    }
}

void drawSliders(App& app) {
    SDL_Renderer* r = app.renderer;
    const bool curves = app.mode == brt::Mode::BidCurves;
    for (std::size_t i = 0; i < kSliders.size(); ++i) {
        const Slider& s = kSliders[i];
        const bool dim = !curves && s.needsCurves;
        const float top = kSliderTop + kSliderRow * static_cast<float>(i);
        const float ty = sliderTrackY(i);
        const gd::Color label = dim ? gd::kMuted : s.color;
        app.hud.text(r, kPanelX, top, s.label, label, gd::Hud::Style::Bold);
        char v[16];
        SDL_snprintf(v, sizeof v, s.decimals >= 2 ? "%.2f" : "%.1f", app.params.*(s.field));
        textRight(app, kPanelX + kPanelW, top, v, dim ? gd::kMuted : gd::kInk, gd::Hud::Style::Bold);
        gd::fillRect(r, kPanelX, ty - 2.0F, kPanelW, 4.0F, dim ? gd::kGrid : gd::kInk);
        const auto kx = static_cast<float>(brt::xAtValue(app.params.*(s.field), static_cast<double>(kPanelX), static_cast<double>(kPanelW), s.lo, s.hi));
        const bool grabbed = app.activeSlider && *app.activeSlider == i;
        gd::card(r, kx - 6.0F, ty - 9.0F, 12.0F, 18.0F, grabbed ? gd::kTeal : gd::kWhite, dim ? gd::kMuted : gd::kInk);
    }
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    const bool curves = app.mode == brt::Mode::BidCurves;
    const auto uses = brt::classify(app.terrain, app.params, app.centre.x, app.centre.y, app.mode);
    const auto counts = brt::count(uses);  // recomputed every frame, so it always matches the map

    std::optional<brt::Cell> hover;
    if (!app.activeSlider) hover = brt::cellAt(static_cast<double>(app.mouseX), static_cast<double>(app.mouseY), static_cast<double>(kMapX), static_cast<double>(kMapY), static_cast<double>(kCell), kCols, kRows);

    gd::backdrop(r, kWidth, kHeight);
    app.hud.titleBar(r, kWidth, "[ BID RENT TOWN ]", curves ? "maximum of three bid curves" : "hand-set rings");

    drawMap(app, uses);
    if (hover) {
        gd::strokeRect(r, kMapX + kCell * static_cast<float>(hover->x), kMapY + kCell * static_cast<float>(hover->y), kCell, kCell, gd::kWhite);
    }
    drawGraph(app, hover);
    drawSliders(app);

    // Legend with live cell counts.
    float x = kMapX;
    const std::array<brt::Use, 4> legend{brt::Use::Market, brt::Use::House, brt::Use::Farm, brt::Use::Wild};
    for (const brt::Use u : legend) {
        char t[32];
        const char* name = u == brt::Use::Market ? "Market" : u == brt::Use::House ? "House" : u == brt::Use::Farm ? "Farm" : "Wild";
        SDL_snprintf(t, sizeof t, "%s %d", name, counts[brt::idx(u)]);
        gd::card(r, x, 421.0F, 14.0F, 14.0F, cellColor(u), gd::kInk);
        app.hud.text(r, x + 20.0F, 417.0F, t, gd::kInk, gd::Hud::Style::Bold);
        x += 20.0F + app.hud.width(t, gd::Hud::Style::Bold) + 16.0F;
    }

    // Info for the cell under the cursor, otherwise the hint.
    char line[160];
    if (hover) {
        const auto i = static_cast<std::size_t>(hover->y * kCols + hover->x);
        const double d = std::hypot(static_cast<double>(hover->x - app.centre.x), static_cast<double>(hover->y - app.centre.y));
        const brt::Use u = uses[i];
        if (u == brt::Use::Water) {
            SDL_snprintf(line, sizeof line, "cell %d,%d  d %.1f  WATER", hover->x, hover->y, d);
        } else if (curves) {
            const brt::Bids b = brt::bids(app.params, d, app.terrain.waterDist[i]);
            SDL_snprintf(line, sizeof line, "cell %d,%d  d %.1f  bids M %.0f H %.0f F %.0f  =  %s", hover->x, hover->y, d, b.market, b.house, b.farm, useName(u));
        } else {
            SDL_snprintf(line, sizeof line, "cell %d,%d  d %.1f  =  %s", hover->x, hover->y, d, useName(u));
        }
        app.hud.text(r, kMapX, 446.0F, line, gd::kInk);
    } else {
        app.hud.text(r, kMapX, 446.0F, "click or drag on the map: move the centre   drag the sliders", gd::kMuted);
    }

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
        if (std::strcmp(argv[i], "--frames") == 0) app->maxFrames = std::atol(argv[i + 1]);
    }
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        SDL_Log("SDL_Init failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    if (!SDL_CreateWindowAndRenderer("bid-rent-town", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
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
            if (app.activeSlider) setSliderFromMouse(app, *app.activeSlider, app.mouseX);
            if (app.dragCentre) moveCentre(app, app.mouseX, app.mouseY);
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
                }
            }
            if (onChip) break;
            if (const auto s = sliderAt(p.x, p.y)) {
                app.activeSlider = s;
                setSliderFromMouse(app, *s, p.x);
            } else if (brt::cellAt(static_cast<double>(p.x), static_cast<double>(p.y), static_cast<double>(kMapX), static_cast<double>(kMapY), static_cast<double>(kCell), kCols, kRows)) {
                app.dragCentre = true;
                moveCentre(app, p.x, p.y);
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event->button.button == SDL_BUTTON_LEFT) clearDrags(app);
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
        case SDL_EVENT_WINDOW_FOCUS_LOST:
            clearDrags(app);
            app.mouseX = -100.0F;
            app.mouseY = -100.0F;
            break;
        case SDL_EVENT_KEY_DOWN:
            if (event->key.repeat) break;  // every key here toggles or resets
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: apply(app, Action::HandSet); break;
                case SDLK_2: apply(app, Action::Curves); break;
                case SDLK_G: apply(app, Action::Grow); break;
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
    const Uint64 now = SDL_GetTicks();
    const double dt = std::min(0.25, static_cast<double>(now - app.lastTicks) / 1000.0);
    app.lastTicks = now;
    if (app.growing) brt::advanceSize(app.params.size, app.growDir, dt, kGrowRate, kSizeMin, kSizeMax);
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
