// Idle game economy with SDL3 main callbacks.
//
// Four buildings, each with a price that grows by a fixed factor per purchase (base * 1.15^n) and an income that grows
// by a fixed amount per building. An auto buyer plays for you and the clock runs at up to a day per second. The top
// plot is the income per second, the bottom plot the number of buildings owned, both over the time of the run, by
// default on a logarithmic time axis: with exponential prices the building count is a straight line there, ten times
// the time buys a fixed number of extra buildings. Prestige throws the run away for a permanent income multiplier; the
// new run draws the same line a fixed step higher, earlier runs stay faded so the shift is visible. The "Linear" chip
// swaps the exponential price for a linear one and the line bends upwards instead. The simulation jumps from purchase
// to purchase instead of stepping frames; the bulk-buy formula, the number formatting and the axes live in logic.hpp
// and are covered by test.cpp.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
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
constexpr double kStep = 1.0 / 60.0;  // real seconds per simulation step
constexpr int kStepEvents = 2000;     // purchases per step before the simulation lets the clock slow down
constexpr int kMaxStepsPerFrame = 5;
constexpr float kChipH = 28.0F;
constexpr float kBuyLabelX = 470.0F;
constexpr double kGrowthMin = 1.02;
constexpr double kGrowthMax = 1.30;

struct Rect {
    float x0, y0, x1, y1;
    float w() const { return x1 - x0; }
    float h() const { return y1 - y0; }
};

constexpr Rect kPlotIncome{84.0F, 92.0F, 640.0F, 254.0F};
constexpr Rect kPlotOwned{84.0F, 292.0F, 640.0F, 404.0F};
constexpr float kPanelX = 668.0F;
constexpr float kPanelW = 270.0F;
constexpr float kGrowthY = 76.0F;
constexpr float kGrowthH = 58.0F;
constexpr float kBuildY = 144.0F;
constexpr float kRowH = 44.0F;
constexpr float kRowsTop = kBuildY + 26.0F;
constexpr float kBuildH = 208.0F;
constexpr float kPrestigeY = 362.0F;
constexpr float kPrestigeH = 82.0F;
constexpr float kSliderX = kPanelX + 16.0F;
constexpr float kSliderW = kPanelW - 32.0F;
constexpr float kSliderTrackY = kGrowthY + 42.0F;

enum class Action { Mode, LogY, LogTime, Auto, Pause, Restart, Speed, Amount, Prestige };

struct Chip {
    Action action;
    int value;
    std::string label;
    SDL_FRect rect;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;

    idle::Game game;
    bool logY = true;
    bool logTime = true;
    bool paused = false;
    int speed = 3600;  // seconds of play per real second
    int amount = 1;   // manual purchase size: 1, 10, 0 = as many as possible
    double accumulator = 0.0;
    Uint64 lastTick = 0;

    SDL_FPoint mouse{-100.0F, -100.0F};
    bool dragging = false;
    long maxFrames = -1;
    long frames = 0;
};

void restartGame(App& app) {
    app.game.autoBuy = true;  // the pre-rolled scene needs the buyer, a restart with it off would show an empty plot
    app.game.restart();
    app.accumulator = 0.0;
}

// ---- chips ------------------------------------------------------------------------------------------------------

void layoutChips(App& app) {
    auto row = [&](float y, float x, auto&& build) {
        build([&](Action action, int value, std::string label, float gapBefore) {
            x += gapBefore;
            const float w = app.hud.width(label, gd::Hud::Style::Bold) + 20.0F;
            app.chips.push_back({action, value, std::move(label), SDL_FRect{x, y, w, kChipH}});
            x += w;
        });
    };
    row(38.0F, 12.0F, [](auto add) {
        add(Action::Mode, 0, "1 Exponential", 0.0F);
        add(Action::Mode, 1, "2 Linear", 6.0F);
        add(Action::LogY, 0, "L Log y", 18.0F);
        add(Action::LogTime, 0, "T Log time", 6.0F);
        add(Action::Auto, 0, "A Auto buy", 18.0F);
        add(Action::Pause, 0, "Space Pause", 6.0F);
        add(Action::Restart, 0, "R Restart", 6.0F);
    });
    const float speedX = 12.0F + app.hud.width("time speed") + 10.0F;
    row(474.0F, speedX, [](auto add) {
        add(Action::Speed, 1, "x1", 0.0F);
        add(Action::Speed, 60, "x60", 6.0F);
        add(Action::Speed, 600, "x600", 6.0F);
        add(Action::Speed, 3600, "x3600", 6.0F);
        add(Action::Speed, 86400, "x86400", 6.0F);
    });
    const float buyX = kBuyLabelX + app.hud.width("buy") + 10.0F;
    row(474.0F, buyX, [](auto add) {
        add(Action::Amount, 1, "x1", 0.0F);
        add(Action::Amount, 10, "x10", 6.0F);
        add(Action::Amount, 0, "Max", 6.0F);
    });
    const float w = app.hud.width("P Prestige", gd::Hud::Style::Bold) + 20.0F;
    app.chips.push_back({Action::Prestige, 0, "P Prestige", SDL_FRect{kPanelX + kPanelW - 14.0F - w, kPrestigeY + 48.0F, w, kChipH}});
}

bool isActive(const App& app, const Chip& c) {
    switch (c.action) {
        case Action::Mode: return (app.game.rules.mode == idle::CostMode::Linear ? 1 : 0) == c.value;
        case Action::LogY: return app.logY;
        case Action::LogTime: return app.logTime;
        case Action::Auto: return app.game.autoBuy;
        case Action::Pause: return app.paused;
        case Action::Speed: return app.speed == c.value;
        case Action::Amount: return app.amount == c.value;
        case Action::Restart:
        case Action::Prestige: return false;
    }
    return false;
}

bool isEnabled(const App& app, const Chip& c) { return c.action != Action::Prestige || idle::pointsFor(app.game.state.earned) >= 1; }

void setMode(App& app, idle::CostMode mode) {
    if (app.game.rules.mode == mode) return;
    app.game.rules.mode = mode;
    restartGame(app);
}

void apply(App& app, const Chip& c) {
    if (!isEnabled(app, c)) return;
    switch (c.action) {
        case Action::Mode: setMode(app, c.value == 1 ? idle::CostMode::Linear : idle::CostMode::Exponential); break;
        case Action::LogY: app.logY = !app.logY; break;
        case Action::LogTime: app.logTime = !app.logTime; break;
        case Action::Auto: app.game.autoBuy = !app.game.autoBuy; break;
        case Action::Pause: app.paused = !app.paused; break;
        case Action::Restart: restartGame(app); break;
        case Action::Speed: app.speed = c.value; break;
        case Action::Amount: app.amount = c.value; break;
        case Action::Prestige: app.game.prestigeNow(); break;
    }
}

void drawChip(App& app, const Chip& c) {
    SDL_Renderer* r = app.renderer;
    const bool on = isActive(app, c);
    const bool enabled = isEnabled(app, c);
    gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? gd::kTeal : gd::kWhite, enabled ? gd::kInk : gd::kMuted);
    app.hud.text(r, c.rect.x + 10.0F, c.rect.y + 3.0F, c.label, on ? gd::kWhite : enabled ? gd::kInk : gd::kMuted, gd::Hud::Style::Bold);
}

// ---- growth slider ----------------------------------------------------------------------------------------------

float sliderFraction(const App& app) { return static_cast<float>(std::clamp((app.game.rules.growth - kGrowthMin) / (kGrowthMax - kGrowthMin), 0.0, 1.0)); }

bool onSlider(SDL_FPoint p) { return p.x >= kSliderX - 8.0F && p.x <= kSliderX + kSliderW + 8.0F && p.y >= kSliderTrackY - 14.0F && p.y <= kSliderTrackY + 14.0F; }

/// Changing the growth restarts the demo: the same buildings at a different price curve are a different game.
void setGrowth(App& app, float mouseX) {
    const double t = std::clamp(static_cast<double>((mouseX - kSliderX) / kSliderW), 0.0, 1.0);
    const double g = std::round((kGrowthMin + t * (kGrowthMax - kGrowthMin)) * 100.0) / 100.0;
    if (g == app.game.rules.growth) return;
    app.game.rules.growth = g;
    restartGame(app);
}

/// The tier the auto buyer is saving for (-1 with the auto buyer off).
int plannedTier(const App& app) {
    if (!app.game.autoBuy) return -1;
    return app.game.state.plan >= 0 ? app.game.state.plan : idle::pickTarget(app.game.rules, app.game.state).tier;
}

// ---- building rows ----------------------------------------------------------------------------------------------

SDL_FRect rowRect(int i) { return SDL_FRect{kPanelX + 6.0F, kRowsTop + static_cast<float>(i) * kRowH, kPanelW - 12.0F, kRowH - 2.0F}; }

/// How many one click buys: 1, 10, or all that fit into the purse.
std::int64_t clickCount(const App& app, int tier) {
    if (app.amount > 0) return app.amount;
    const idle::State& s = app.game.state;
    return std::max<std::int64_t>(1, idle::maxAffordable(app.game.rules, idle::kTiers[static_cast<std::size_t>(tier)], s.owned[static_cast<std::size_t>(tier)], s.money));
}

// ---- plot helpers -----------------------------------------------------------------------------------------------

float yOf(const Rect& p, const idle::Axis& axis, double v) { return p.y1 - static_cast<float>(axis.map(v)) * p.h(); }

/// 1 px lines are too thin on a 960x540 canvas that is scaled up: three side by side.
void thickLine(SDL_Renderer* r, float x0, float y0, float x1, float y1, gd::Color c, int half = 1) {
    gd::setColor(r, c);
    const bool flat = std::fabs(x1 - x0) > std::fabs(y1 - y0);
    for (int o = -half; o <= half; ++o) {
        const auto off = static_cast<float>(o);
        SDL_RenderLine(r, x0 + (flat ? 0.0F : off), y0 + (flat ? off : 0.0F), x1 + (flat ? 0.0F : off), y1 + (flat ? off : 0.0F));
    }
}

struct Scale {
    idle::TimeAxis time;
    idle::Axis income;
    idle::Axis owned;
};

Scale makeScale(const App& app) {
    Scale sc;
    double tMax = 60.0;
    double maxIncome = 0.0;
    double maxOwned = 0.0;
    for (const idle::RunLog& run : app.game.history.runs) {
        for (const idle::Point& p : run) {
            tMax = std::max(tMax, p.t);
            maxIncome = std::max(maxIncome, p.income);
            maxOwned = std::max(maxOwned, p.owned);
        }
    }
    tMax = std::max(tMax, app.game.state.time);
    sc.time = idle::makeTimeAxis(tMax, app.logTime, 6);
    sc.income = app.logY ? idle::makeAxis(0.1, std::max(maxIncome, 1000.0), true, 6, 0.1) : idle::makeAxis(0.0, std::max(maxIncome, 10.0), false, 4);
    sc.owned = idle::makeAxis(0.0, std::max(maxOwned, 10.0), false, 4);
    return sc;
}

float xOf(const Rect& p, const Scale& sc, double t) { return p.x0 + static_cast<float>(sc.time.map(t)) * p.w(); }

void drawFrame(App& app, const Rect& p, const idle::Axis& axis, const Scale& sc) {
    SDL_Renderer* r = app.renderer;
    gd::card(r, p.x0, p.y0, p.w(), p.h(), gd::kWhite, gd::kInk);
    for (const double t : axis.ticks) {
        const float y = yOf(p, axis, t);
        gd::fillRect(r, p.x0, y, p.w(), 1.0F, gd::kGrid);
        const std::string s = idle::formatBig(t);
        app.hud.text(r, p.x0 - 8.0F - app.hud.width(s), y - 11.0F, s, gd::kMuted);
    }
    for (const double t : sc.time.ticks) {
        if (t > 0.0 && t < sc.time.hi - 1e-9) gd::fillRect(r, xOf(p, sc, t), p.y0, 1.0F, p.h(), gd::kGrid);
    }
    gd::strokeRect(r, p.x0, p.y0, p.w(), p.h(), gd::kInk);
}

void drawTimeLabels(App& app, const Rect& p, const Scale& sc) {
    float lastEnd = -1000.0F;
    for (const double t : sc.time.ticks) {
        const std::string s = idle::formatDuration(t);
        const float w = app.hud.width(s);
        const float x = std::clamp(xOf(p, sc, t) - w / 2.0F, 4.0F, 956.0F - w);
        if (x < lastEnd + 8.0F) continue;  // log ticks bunch up (10 h and 1 d): skip a label that would overprint the previous
        app.hud.text(app.renderer, x, p.y1 + 6.0F, s, gd::kMuted);
        lastEnd = x + w;
    }
    app.hud.text(app.renderer, p.x0 - 24.0F - app.hud.width("time"), p.y1 + 6.0F, "time", gd::kMuted);
}

/// A run as a staircase: income and the building count only change when something is bought. `tail` extends the newest run to now.
void drawRun(App& app, const Rect& p, const idle::Axis& axis, const Scale& sc, const idle::RunLog& run, double idle::Point::*field, gd::Color color, int half, bool tail) {
    if (run.empty()) return;
    std::vector<idle::Point> pts = run;
    if (tail) pts.push_back(idle::Point{std::max(app.game.state.time, run.back().t), run.back().income, run.back().owned});
    for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
        const float x0 = xOf(p, sc, pts[i].t);
        const float x1 = xOf(p, sc, pts[i + 1].t);
        const float y0 = yOf(p, axis, pts[i].*field);
        const float y1 = yOf(p, axis, pts[i + 1].*field);
        thickLine(app.renderer, x0, y0, x1, y0, color, half);
        thickLine(app.renderer, x1, y0, x1, y1, color, half);
    }
}

void legendRow(App& app, float x, float y, gd::Color color, int half, const char* text) {
    gd::fillRect(app.renderer, x, y + 10.0F - static_cast<float>(half), 18.0F, static_cast<float>(half * 2 + 1), color);
    app.hud.text(app.renderer, x + 26.0F, y, text, gd::kInk);
}

void drawPlots(App& app, const Scale& sc) {
    SDL_Renderer* r = app.renderer;
    const auto& runs = app.game.history.runs;
    const gd::Color older[2] = {gd::Color{13, 148, 136, 70}, gd::Color{234, 88, 12, 70}};
    const gd::Color newestOlder[2] = {gd::Color{13, 148, 136, 130}, gd::Color{234, 88, 12, 130}};
    const gd::Color now[2] = {gd::kTeal, gd::kOrange};
    const idle::Axis* axes[2] = {&sc.income, &sc.owned};
    const Rect* rects[2] = {&kPlotIncome, &kPlotOwned};
    double idle::Point::*fields[2] = {&idle::Point::income, &idle::Point::owned};

    app.hud.text(r, kPlotIncome.x0, 68.0F, app.logY ? "income per second (log)" : "income per second", gd::kInk, gd::Hud::Style::Bold);
    app.hud.text(r, kPlotOwned.x0, 268.0F, "buildings owned", gd::kInk, gd::Hud::Style::Bold);
    for (int k = 0; k < 2; ++k) drawFrame(app, *rects[k], *axes[k], sc);
    drawTimeLabels(app, kPlotOwned, sc);

    for (int k = 0; k < 2; ++k) {
        for (std::size_t i = 0; i + 1 < runs.size(); ++i) drawRun(app, *rects[k], *axes[k], sc, runs[i], fields[k], i + 2 == runs.size() ? newestOlder[k] : older[k], 0, false);
        drawRun(app, *rects[k], *axes[k], sc, runs.back(), fields[k], now[k], 1, true);
        if (!runs.back().empty()) {
            const idle::State& s = app.game.state;
            const double v = k == 0 ? idle::income(s) : static_cast<double>(idle::totalOwned(s));
            const float cx = xOf(*rects[k], sc, s.time);
            const float cy = yOf(*rects[k], *axes[k], v);
            gd::disc(r, cx, cy, 6.0F, gd::kWhite);
            gd::disc(r, cx, cy, 3.5F, now[k]);
            gd::ring(r, cx, cy, 6.0F, gd::kInk);
        }
    }
    legendRow(app, kPlotIncome.x1 - 270.0F, 68.0F, gd::kTeal, 1, "this run");
    legendRow(app, kPlotIncome.x1 - 150.0F, 68.0F, gd::Color{13, 148, 136, 130}, 0, "earlier runs");

    // What the log axis shows: how many buildings the last tenfold of time brought.
    const double perDecade = idle::buildingsPerDecade(runs.back(), app.game.state.time);
    if (perDecade > 0.0) {
        const std::string label = "10x time: +" + idle::formatCount(std::llround(perDecade)) + " buildings";
        app.hud.text(r, kPlotOwned.x0 + 12.0F, kPlotOwned.y0 + 8.0F, label, gd::kOrange, gd::Hud::Style::Bold);
    }
}

// ---- panel ------------------------------------------------------------------------------------------------------

void rightText(App& app, float xRight, float y, const std::string& s, gd::Color c, gd::Hud::Style style = gd::Hud::Style::Regular) {
    app.hud.text(app.renderer, xRight - app.hud.width(s, style), y, s, c, style);
}

void drawGrowth(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::card(r, kPanelX, kGrowthY, kPanelW, kGrowthH, gd::kWhite, gd::kInk);
    app.hud.text(r, kSliderX, kGrowthY + 6.0F, "growth", gd::kInk, gd::Hud::Style::Bold);
    app.hud.text(r, kSliderX + app.hud.width("growth", gd::Hud::Style::Bold) + 10.0F, kGrowthY + 6.0F, "restarts", gd::kMuted);
    rightText(app, kSliderX + kSliderW, kGrowthY + 6.0F, idle::formatBig(app.game.rules.growth), gd::kTealDark, gd::Hud::Style::Bold);
    gd::fillRect(r, kSliderX, kSliderTrackY - 2.0F, kSliderW, 5.0F, gd::Color{17, 24, 39, 60});
    const float hx = kSliderX + sliderFraction(app) * kSliderW;
    gd::fillRect(r, kSliderX, kSliderTrackY - 2.0F, hx - kSliderX, 5.0F, gd::kTeal);
    gd::card(r, hx - 6.0F, kSliderTrackY - 10.0F, 12.0F, 20.0F, app.dragging ? gd::kOrange : gd::kTeal, gd::kInk);
}

void drawBuildings(App& app) {
    SDL_Renderer* r = app.renderer;
    const idle::State& s = app.game.state;
    const idle::Rules& rules = app.game.rules;
    gd::card(r, kPanelX, kBuildY, kPanelW, kBuildH, gd::kWhite, gd::kInk);
    app.hud.text(r, kPanelX + 14.0F, kBuildY + 6.0F, "BUILDINGS", gd::kMuted);
    const std::string amountLabel = app.amount == 0 ? "buy max" : "buy " + std::to_string(app.amount);
    rightText(app, kPanelX + kPanelW - 14.0F, kBuildY + 6.0F, amountLabel, gd::kMuted);

    const int plan = plannedTier(app);
    for (int i = 0; i < idle::kTierCount; ++i) {
        const idle::Tier& t = idle::kTiers[static_cast<std::size_t>(i)];
        const std::int64_t owned = s.owned[static_cast<std::size_t>(i)];
        const std::int64_t count = clickCount(app, i);
        const double cost = idle::bulkCost(rules, t, owned, count);
        const bool can = idle::canAfford(s.money, cost);
        const SDL_FRect rect = rowRect(i);
        const bool hover = SDL_PointInRectFloat(&app.mouse, &rect);
        gd::fillRect(r, rect.x, rect.y, rect.w, rect.h, can ? gd::kTealSoft : gd::kPaper);
        gd::strokeRect(r, rect.x, rect.y, rect.w, rect.h, hover && can ? gd::kTealDark : gd::kGrid);
        if (i == plan) gd::fillRect(r, rect.x, rect.y, 4.0F, rect.h, gd::kOrange);
        app.hud.text(r, rect.x + 12.0F, rect.y + 0.0F, t.name, gd::kInk, gd::Hud::Style::Bold);
        rightText(app, rect.x + rect.w - 8.0F, rect.y + 0.0F, "x" + idle::formatCount(owned), gd::kTealDark, gd::Hud::Style::Bold);
        app.hud.text(r, rect.x + 12.0F, rect.y + 20.0F, "cost " + idle::formatBig(cost), can ? gd::kTealDark : gd::kMuted);
        rightText(app, rect.x + rect.w - 8.0F, rect.y + 20.0F, "+" + idle::formatBig(static_cast<double>(count) * t.baseIncome * idle::multiplier(s)) + "/s", gd::kMuted);
    }
}

void drawPrestige(App& app) {
    SDL_Renderer* r = app.renderer;
    const idle::State& s = app.game.state;
    const std::int64_t gain = idle::pointsFor(s.earned);
    gd::card(r, kPanelX, kPrestigeY, kPanelW, kPrestigeH, gd::kWhite, gd::kInk);
    app.hud.text(r, kPanelX + 14.0F, kPrestigeY + 8.0F, "PRESTIGE", gd::kMuted);
    rightText(app, kPanelX + kPanelW - 14.0F, kPrestigeY + 8.0F, "income x" + idle::formatBig(idle::multiplier(s)), gd::kTealDark, gd::Hud::Style::Bold);
    app.hud.text(r, kPanelX + 14.0F, kPrestigeY + 28.0F, "points " + idle::formatCount(s.points), gd::kInk);
    const std::string offer = gain >= 1 ? "+" + idle::formatCount(gain) + " pts" : "earn " + idle::formatBig(idle::kPrestigeUnit) + " first";
    rightText(app, kPanelX + kPanelW - 14.0F, kPrestigeY + 28.0F, offer, gain >= 1 ? gd::kOrange : gd::kMuted, gain >= 1 ? gd::Hud::Style::Bold : gd::Hud::Style::Regular);
    if (gain >= 1) {
        const double after = 1.0 + idle::kPointBonus * static_cast<double>(s.points + gain);
        app.hud.text(r, kPanelX + 14.0F, kPrestigeY + 54.0F, "then x" + idle::formatBig(after), gd::kMuted);
    }
}

void drawStats(App& app) {
    SDL_Renderer* r = app.renderer;
    const idle::State& s = app.game.state;
    const int plan = plannedTier(app);
    const std::string wait =
        plan < 0 ? "auto off" : idle::formatDuration(idle::waitFor(s, idle::unitCost(app.game.rules, idle::kTiers[static_cast<std::size_t>(plan)], s.owned[static_cast<std::size_t>(plan)])));
    struct Cell {
        const char* label;
        std::string value;
    };
    const Cell cells[] = {{"money", idle::formatBig(s.money)},
                          {"income", idle::formatBig(idle::income(s)) + "/s"},
                          {"run", idle::formatDuration(s.time)},
                          {"next buy", wait}};
    float x = 12.0F;
    for (const Cell& c : cells) {
        app.hud.text(r, x, 436.0F, c.label, gd::kMuted);
        x += app.hud.width(c.label) + 8.0F;
        app.hud.text(r, x, 436.0F, c.value, gd::kInk, gd::Hud::Style::Bold);
        x += app.hud.width(c.value, gd::Hud::Style::Bold) + 22.0F;
    }
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    const idle::Rules& rules = app.game.rules;
    gd::backdrop(r, kWidth, kHeight);
    const std::string cost = rules.mode == idle::CostMode::Exponential ? "cost = base × " + idle::formatBig(rules.growth) + "^n"
                                                                       : "cost = base × (1 + " + idle::formatBig(rules.growth - 1.0) + " n)";
    app.hud.titleBar(r, kWidth, "[ IDLE ECONOMY ]", cost);

    const Scale sc = makeScale(app);
    drawPlots(app, sc);
    drawGrowth(app);
    drawBuildings(app);
    drawPrestige(app);
    drawStats(app);

    app.hud.text(r, 12.0F, 455.0F, "keys: 1 2 L T A P Space R   orange bar: what the auto buyer saves for", gd::kMuted);
    app.hud.text(r, 12.0F, 482.0F, "time speed", gd::kMuted);
    app.hud.text(r, kBuyLabelX, 482.0F, "buy", gd::kMuted);
    for (const Chip& c : app.chips) drawChip(app, c);
    const std::string unit = "x" + std::to_string(app.speed) + " = " + idle::formatDuration(app.speed) + " per second";
    rightText(app, 948.0F, 482.0F, unit, gd::kMuted);
}

void stepSim(App& app) { app.game.run(static_cast<double>(app.speed) * kStep, kStepEvents); }

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
    if (!SDL_CreateWindowAndRenderer("idle-economy", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, SDL_LOGICAL_PRESENTATION_LETTERBOX);
    SDL_SetRenderVSync(app->renderer, 1);
    restartGame(*app);
    layoutChips(*app);
    app->lastTick = SDL_GetPerformanceCounter();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION:
            app.mouse = {event->motion.x, event->motion.y};
            if (app.dragging) setGrowth(app, event->motion.x);
            break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const SDL_FPoint p{event->button.x, event->button.y};
            app.mouse = p;
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) {
                    apply(app, c);
                    return SDL_APP_CONTINUE;
                }
            }
            for (int i = 0; i < idle::kTierCount; ++i) {
                const SDL_FRect rect = rowRect(i);
                if (SDL_PointInRectFloat(&p, &rect)) {
                    app.game.buyNow(i, clickCount(app, i));
                    return SDL_APP_CONTINUE;
                }
            }
            if (onSlider(p)) {
                app.dragging = true;
                setGrowth(app, p.x);
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event->button.button == SDL_BUTTON_LEFT) app.dragging = false;
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
            app.dragging = false;
            app.mouse = {-100.0F, -100.0F};
            break;
        case SDL_EVENT_WINDOW_FOCUS_LOST: app.dragging = false; break;
        case SDL_EVENT_KEY_DOWN:
            if (event->key.repeat) break;
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: setMode(app, idle::CostMode::Exponential); break;
                case SDLK_2: setMode(app, idle::CostMode::Linear); break;
                case SDLK_L: app.logY = !app.logY; break;
                case SDLK_T: app.logTime = !app.logTime; break;
                case SDLK_A: app.game.autoBuy = !app.game.autoBuy; break;
                case SDLK_P: app.game.prestigeNow(); break;
                case SDLK_SPACE: app.paused = !app.paused; break;
                case SDLK_R: restartGame(app); break;
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
    const double dt = std::min(static_cast<double>(now - app.lastTick) / static_cast<double>(SDL_GetPerformanceFrequency()), 0.25);
    app.lastTick = now;

    if (!app.paused) {
        app.accumulator += dt;
        for (int i = 0; i < kMaxStepsPerFrame && app.accumulator >= kStep; ++i) {
            app.accumulator -= kStep;
            stepSim(app);
        }
        app.accumulator = std::min(app.accumulator, kStep);  // whatever the step cap could not catch up on is dropped
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
        if (app->renderer != nullptr) SDL_DestroyRenderer(app->renderer);
        if (app->window != nullptr) SDL_DestroyWindow(app->window);
        delete app;
    }
}
