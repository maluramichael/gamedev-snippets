// Dice and damage rolls with SDL3 main callbacks.
//
// Thousands of rolls of NdS + bonus pile up as a live histogram (teal bars). The ink line on top is the exact answer,
// computed by convolving the die with itself, so you can watch the bars grow into it. Three things to see:
//   * More dice with the same range means less spread: the curve turns from a flat block into a bell.
//   * A bonus slides the curve along the axis. What +1 is worth for "need 12+" is the height of the bar just below the
//     target: 5 points everywhere on a d20, 12.5 in the middle of 3d6, almost nothing in its tails.
//   * The wrong ways to roll (orange): one flat roll over the same range keeps the mean but loses the bell, and
//     `% sides` on a toy 8 bit source makes the low faces of a d20 8 percent too frequent.
// In table mode the dice are replaced by a hand made weight table: click a bar to add weight, right click to remove it.
// The odds, the sampling and the plot geometry live in logic.hpp and are covered by test.cpp.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
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
constexpr int kPad = 10;  // room left and right of the dice range, also the largest bonus

constexpr float kPlotX = 40.0F;
constexpr float kPlotW = 880.0F;
constexpr float kPlotTop = 84.0F;
constexpr float kBase = 270.0F;  // y of the zero line
constexpr float kPlotH = kBase - kPlotTop;
constexpr float kRowY = 296.0F;  // weights (table) or the success bracket (dice) under the axis labels
constexpr float kPanelY = 322.0F;
constexpr float kPanelH = 86.0F;
constexpr float kPanelW = 280.0F;
constexpr float kRow1 = 414.0F;
constexpr float kRow2 = 448.0F;
constexpr float kRow3 = 482.0F;
constexpr float kChipH = 28.0F;

constexpr int kPerFrame = 400;
constexpr int kPreroll = 3000;  // so the very first frame already shows a histogram
constexpr std::uint64_t kMaxRolls = 1000000;

constexpr gd::Color kInkSoft{17, 24, 39, 60};
constexpr gd::Color kPlotBg{255, 255, 255, 255};

enum class Mode { Dice = 0, Table = 1 };
enum class Method { Correct = 0, Flat = 1, Modulo = 2 };
enum class Action { Dice, Sides, Bonus, Target, SetMode, SetMethod, Preset, Pause, Reset };

struct Chip {
    Action action;
    int value;
    std::string label;
    SDL_FRect rect;
};

struct Text {
    std::string text;
    float x;
    float y;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;

    Mode mode = Mode::Dice;
    Method method = Method::Correct;
    int dice = 3;
    int sides = 6;
    int bonus = 0;
    int target = 12;
    bool paused = false;
    bool scrubbing = false;  // the mouse button is held in the plot and moves the target

    std::array<dd::Table, 3> presets;
    dd::Table table;

    dd::Pcg32 rng;
    dd::Distribution base;     // the exact odds without the bonus
    dd::Distribution ideal;    // with the bonus
    dd::Distribution flat;     // one flat roll over the same range, with the bonus
    dd::Window window_;
    dd::Axis axis;
    double yMax = 0.2;
    std::optional<dd::Histogram> hist;

    std::vector<Chip> chips;
    std::vector<Text> labels;
    float bonusValueX = 0.0F;
    float targetValueX = 0.0F;

    SDL_FPoint mouse{-100.0F, -100.0F};
    long maxFrames = -1;
    long frames = 0;
};

// ---- model ----------------------------------------------------------------------------------------------------

const char* methodName(const App& app, Method m) {
    if (app.mode == Mode::Table) return m == Method::Correct ? "weighted table" : "flat, no weights";
    return m == Method::Correct ? "sum of dice" : m == Method::Flat ? "one flat roll" : "8-bit modulo";
}

int rollOnce(App& app) {
    if (app.mode == Mode::Table) {
        return app.method == Method::Correct ? app.table.pick(app.rng) : dd::rollFlat(app.rng, app.table.lo, app.table.hi());
    }
    switch (app.method) {
        case Method::Correct: return dd::rollDice(app.rng, app.dice, app.sides);
        case Method::Flat: return dd::rollFlat(app.rng, app.dice, app.dice * app.sides);
        case Method::Modulo: return dd::rollModulo8(app.rng, app.dice, app.sides);
    }
    return 0;
}

void rollSome(App& app, std::uint64_t count) {
    for (std::uint64_t i = 0; i < count && app.hist->total() < kMaxRolls; ++i) app.hist->add(rollOnce(app));
}

/// The bonus only moves things, the rolled data stays.
void refreshShift(App& app) {
    app.ideal = dd::shifted(app.base, app.bonus);
    app.flat = dd::shifted(dd::flatDistribution(app.base.lo, app.base.hi()), app.bonus);
}

/// Start the rolling over: new histogram, a first batch so there is something to look at.
void restartRolls(App& app) {
    app.hist.emplace(app.base.lo, static_cast<int>(app.base.p.size()));
    rollSome(app, kPreroll);
}

/// Everything that depends on the dice or the table: odds, axis, scale, histogram.
void rebuildModel(App& app) {
    app.base = app.mode == Mode::Dice ? dd::diceDistribution(app.dice, app.sides) : app.table.distribution();
    app.window_ = dd::windowFor(app.base.lo, app.base.hi(), kPad);
    app.axis = dd::Axis{app.window_.lo, app.window_.bins(), static_cast<double>(kPlotX), static_cast<double>(kPlotW)};
    app.target = std::clamp(app.target, app.window_.lo, app.window_.hi);
    refreshShift(app);
    const double tallest = std::max(*std::max_element(app.base.p.begin(), app.base.p.end()), 1.0 / static_cast<double>(app.base.p.size()));
    app.yMax = std::min(tallest * 1.25, 1.0);
    restartRolls(app);
}

// ---- layout ---------------------------------------------------------------------------------------------------

void addChip(App& app, Action action, int value, const std::string& label, float& x, float y, float gap) {
    const float w = app.hud.width(label, gd::Hud::Style::Bold) + 16.0F;
    app.chips.push_back({action, value, label, SDL_FRect{x, y, w, kChipH}});
    x += w + gap;
}

void addLabel(App& app, const std::string& text, float& x, float y) {
    app.labels.push_back({text, x, y + 3.0F});
    x += app.hud.width(text) + 10.0F;
}

void layoutChips(App& app) {
    app.chips.clear();
    app.labels.clear();

    float x = kPlotX;
    if (app.mode == Mode::Dice) {
        addLabel(app, "dice", x, kRow1);
        for (int n = 1; n <= 10; ++n) addChip(app, Action::Dice, n, std::to_string(n), x, kRow1, 4.0F);
        x += 20.0F;
        addLabel(app, "sides", x, kRow1);
        for (const int s : {4, 6, 8, 10, 12, 20}) addChip(app, Action::Sides, s, "d" + std::to_string(s), x, kRow1, 4.0F);
    } else {
        addLabel(app, "table", x, kRow1);
        addChip(app, Action::Preset, 0, "Flat", x, kRow1, 4.0F);
        addChip(app, Action::Preset, 1, "Ramp", x, kRow1, 4.0F);
        addChip(app, Action::Preset, 2, "Graze, hit, crit", x, kRow1, 20.0F);
        app.labels.push_back({"click a bar: weight +1, right click: -1", x, kRow1 + 3.0F});
    }

    x = kPlotX;
    addLabel(app, "bonus", x, kRow2);
    addChip(app, Action::Bonus, -1, "-", x, kRow2, 0.0F);
    app.bonusValueX = x;
    x += 60.0F;
    addChip(app, Action::Bonus, 1, "+", x, kRow2, 30.0F);
    addLabel(app, "need", x, kRow2);
    addChip(app, Action::Target, -1, "-", x, kRow2, 0.0F);
    app.targetValueX = x;
    x += 60.0F;
    addChip(app, Action::Target, 1, "+", x, kRow2, 30.0F);
    addLabel(app, "mode", x, kRow2);
    addChip(app, Action::SetMode, 0, "Dice", x, kRow2, 4.0F);
    addChip(app, Action::SetMode, 1, "Table", x, kRow2, 0.0F);

    x = kPlotX;
    addLabel(app, "roll", x, kRow3);
    const int methods = app.mode == Mode::Dice ? 3 : 2;
    for (int m = 0; m < methods; ++m) {
        const std::string label = methodName(app, static_cast<Method>(m));
        addChip(app, Action::SetMethod, m, label, x, kRow3, 4.0F);
    }
    x += 20.0F;
    addChip(app, Action::Pause, 0, "Space Pause", x, kRow3, 4.0F);
    addChip(app, Action::Reset, 0, "R Roll again", x, kRow3, 0.0F);
}

// ---- actions --------------------------------------------------------------------------------------------------

void setMode(App& app, Mode mode) {
    app.mode = mode;
    app.method = Method::Correct;
    app.scrubbing = false;
    rebuildModel(app);
    layoutChips(app);
}

void setMethod(App& app, int m) {
    const int limit = app.mode == Mode::Dice ? 3 : 2;
    app.method = static_cast<Method>(((m % limit) + limit) % limit);
    restartRolls(app);
}

void apply(App& app, Action action, int value) {
    switch (action) {
        case Action::Dice:
            app.dice = std::clamp(value, 1, 10);
            rebuildModel(app);
            break;
        case Action::Sides:
            app.sides = value;
            rebuildModel(app);
            break;
        case Action::Bonus:
            app.bonus = std::clamp(app.bonus + value, -kPad, kPad);
            refreshShift(app);
            break;
        case Action::Target: app.target = std::clamp(app.target + value, app.window_.lo, app.window_.hi); break;
        case Action::SetMode: setMode(app, value == 0 ? Mode::Dice : Mode::Table); break;
        case Action::SetMethod: setMethod(app, value); break;
        case Action::Preset:
            app.table = app.presets[static_cast<std::size_t>(value)];
            rebuildModel(app);
            break;
        case Action::Pause: app.paused = !app.paused; break;
        case Action::Reset: restartRolls(app); break;
    }
}

bool active(const App& app, const Chip& c) {
    switch (c.action) {
        case Action::Dice: return c.value == app.dice;
        case Action::Sides: return c.value == app.sides;
        case Action::SetMode: return c.value == static_cast<int>(app.mode);
        case Action::SetMethod: return c.value == static_cast<int>(app.method);
        case Action::Preset: return app.table.weights == app.presets[static_cast<std::size_t>(c.value)].weights;
        case Action::Pause: return app.paused;
        default: return false;
    }
}

/// A click on a bar of the table: add or remove one weight. Returns whether anything changed.
bool editWeight(App& app, int displayedValue, int delta) {
    const int raw = displayedValue - app.bonus;
    if (raw < app.table.lo || raw > app.table.hi()) return false;
    const auto index = static_cast<std::size_t>(raw - app.table.lo);
    if (!app.table.setWeight(static_cast<int>(index), app.table.weights[index] + delta)) return false;
    rebuildModel(app);
    return true;
}

bool inPlot(SDL_FPoint p) { return p.x >= kPlotX && p.x < kPlotX + kPlotW && p.y >= kPlotTop && p.y < kRowY + 22.0F; }

std::optional<int> hoveredValue(const App& app) {
    return inPlot(app.mouse) ? app.axis.valueAt(static_cast<double>(app.mouse.x)) : std::nullopt;
}

void scrubTo(App& app, float x) {
    const double clamped = std::clamp(static_cast<double>(x), static_cast<double>(kPlotX), static_cast<double>(kPlotX + kPlotW) - 0.01);
    if (const auto v = app.axis.valueAt(clamped)) app.target = *v;
}

// ---- drawing --------------------------------------------------------------------------------------------------

void line2(SDL_Renderer* r, float x0, float y0, float x1, float y1, gd::Color c) {
    gd::setColor(r, c);
    SDL_RenderLine(r, x0, y0, x1, y1);
    SDL_RenderLine(r, x0, y0 + 1.0F, x1, y1 + 1.0F);
}

std::string pct(double p, int decimals = 1) {
    char buf[32];
    SDL_snprintf(buf, sizeof buf, decimals == 1 ? "%.1f%%" : "%.2f%%", p * 100.0);
    return buf;
}

std::string fixed(double v, int decimals = 2) {
    char buf[32];
    SDL_snprintf(buf, sizeof buf, decimals == 1 ? "%.1f" : "%.2f", v);
    return buf;
}

void drawCentered(App& app, float cx, float y, const std::string& s, gd::Color c, gd::Hud::Style style = gd::Hud::Style::Regular) {
    app.hud.text(app.renderer, cx - app.hud.width(s, style) / 2.0F, y, s, c, style);
}

void drawPlot(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::fillRect(r, kPlotX, kPlotTop, kPlotW, kPlotH + 1.0F, kPlotBg);

    // Where the roll succeeds: everything from the target to the right.
    const auto tx = static_cast<float>(app.axis.left(app.target));
    gd::fillRect(r, tx, kPlotTop, kPlotX + kPlotW - tx, kPlotH, gd::Color{204, 251, 241, 150});

    // Horizontal guides in percent.
    const double step = dd::niceStep(static_cast<double>(kPlotH) / (app.yMax * 100.0), 30.0, 0.5);
    for (double pc = step; pc < app.yMax * 100.0; pc += step) {
        const float y = kBase - static_cast<float>(pc / 100.0 / app.yMax) * kPlotH;
        gd::fillRect(r, kPlotX, y, kPlotW, 1.0F, gd::kGrid);
        char label[16];
        if (pc == std::floor(pc)) {
            SDL_snprintf(label, sizeof label, "%d%%", static_cast<int>(pc));
        } else {
            SDL_snprintf(label, sizeof label, "%.1f%%", pc);
        }
        app.hud.text(r, kPlotX + kPlotW + 5.0F, y - 11.0F, label, gd::kMuted);
    }

    // Hover column.
    const auto hover = hoveredValue(app);
    if (hover) gd::fillRect(r, static_cast<float>(app.axis.left(*hover)), kPlotTop, static_cast<float>(app.axis.binWidth()), kPlotH, kInkSoft);

    // The rolled bars. Orange as soon as the way of rolling is a wrong one.
    const bool naive = app.method != Method::Correct;
    const gd::Color barColor = naive ? gd::kOrange : gd::kTeal;
    const float binW = static_cast<float>(app.axis.binWidth());
    const float gap = binW > 12.0F ? 3.0F : binW > 6.0F ? 2.0F : binW > 3.0F ? 1.0F : 0.0F;
    for (int v = app.window_.lo; v <= app.window_.hi; ++v) {
        const double p = app.hist->prob(v - app.bonus);
        if (p <= 0.0) continue;
        const float h = std::min(static_cast<float>(p / app.yMax), 1.0F) * kPlotH;
        gd::Color c = barColor;
        c.a = v >= app.target ? 255 : 110;
        gd::fillRect(r, static_cast<float>(app.axis.left(v)) + gap / 2.0F, kBase - h, binW - gap, h, c);
    }

    // The exact odds.
    const bool dots = binW >= 8.0F;
    float px = 0.0F;
    float py = 0.0F;
    for (int v = app.ideal.lo; v <= app.ideal.hi(); ++v) {
        const auto cx = static_cast<float>(app.axis.center(v));
        const float cy = kBase - std::min(static_cast<float>(app.ideal.at(v) / app.yMax), 1.0F) * kPlotH;
        if (v > app.ideal.lo) line2(r, px, py, cx, cy, gd::kInk);
        px = cx;
        py = cy;
        if (dots) gd::fillRect(r, cx - 2.5F, cy - 2.0F, 5.0F, 5.0F, gd::kInk);
    }

    // Axis.
    gd::fillRect(r, kPlotX, kBase, kPlotW, 2.0F, gd::kInk);
    const int labelStep = static_cast<int>(std::lround(dd::niceStep(app.axis.binWidth(), 34.0)));
    for (int v = app.window_.lo; v <= app.window_.hi; ++v) {
        if (((v % labelStep) + labelStep) % labelStep != 0) continue;
        drawCentered(app, static_cast<float>(app.axis.center(v)), kBase + 4.0F, std::to_string(v), gd::kMuted);
    }

    // The target: a dashed line on the left edge of its bar.
    for (float y = kPlotTop; y < kBase; y += 8.0F) gd::fillRect(r, tx - 1.0F, y, 3.0F, 5.0F, gd::kTealDark);
    const std::string flag = std::to_string(app.target) + "+";
    const float flagW = app.hud.width(flag, gd::Hud::Style::Bold) + 12.0F;
    const float flagX = std::min(tx + 4.0F, kPlotX + kPlotW - flagW);
    gd::card(r, flagX, kPlotTop + 4.0F, flagW, 24.0F, gd::kTealDark, gd::kInk);
    app.hud.text(r, flagX + 6.0F, kPlotTop + 5.0F, flag, gd::kWhite, gd::Hud::Style::Bold);

    gd::strokeRect(r, kPlotX, kPlotTop, kPlotW, kPlotH + 1.0F, gd::Color{17, 24, 39, 90});

    // Row under the axis: the weights in table mode, the success bracket for dice.
    if (app.mode == Mode::Table) {
        for (int v = app.table.lo; v <= app.table.hi(); ++v) {
            const int w = app.table.weights[static_cast<std::size_t>(v - app.table.lo)];
            const int shown = v + app.bonus;
            if (shown < app.window_.lo || shown > app.window_.hi) continue;
            drawCentered(app, static_cast<float>(app.axis.center(shown)), kRowY, std::to_string(w), w == 0 ? gd::kMuted : gd::kTealDark, gd::Hud::Style::Bold);
        }
    } else {
        const float y = kRowY + 2.0F;
        gd::fillRect(r, tx, y, kPlotX + kPlotW - tx, 2.0F, gd::kTealDark);
        gd::fillRect(r, tx, y - 4.0F, 2.0F, 10.0F, gd::kTealDark);
        gd::fillRect(r, kPlotX + kPlotW - 2.0F, y - 4.0F, 2.0F, 10.0F, gd::kTealDark);
        const std::string s = std::to_string(app.target) + "+ comes up " + pct(dd::atLeast(app.ideal, app.target)) + " of the time";
        const float sx = std::min(tx, kPlotX + kPlotW - app.hud.width(s));
        app.hud.text(r, sx, y + 6.0F, s, gd::kTealDark);
    }
}

void drawHeader(App& app) {
    SDL_Renderer* r = app.renderer;
    std::string title = app.mode == Mode::Dice ? dd::expression(app.dice, app.sides, app.bonus) : "Table";
    if (app.mode == Mode::Table && app.bonus != 0) title += app.bonus > 0 ? " + " + std::to_string(app.bonus) : " - " + std::to_string(-app.bonus);
    app.hud.text(r, kPlotX, 34.0F, title, gd::kInk, gd::Hud::Style::Large);

    const auto hover = hoveredValue(app);
    if (hover) {
        const int v = *hover;
        const std::string s = std::to_string(v) + ":  exact " + pct(app.ideal.at(v), 2) + "   rolled " + pct(app.hist->prob(v - app.bonus), 2);
        app.hud.text(r, kPlotX + kPlotW - app.hud.width(s), 46.0F, s, gd::kInk, gd::Hud::Style::Bold);
        return;
    }
    // Legend, right aligned: exact line, then the rolled bars with the way they were rolled.
    const bool naive = app.method != Method::Correct;
    const std::string rolled = std::string("rolled: ") + methodName(app, app.method);
    float x = kPlotX + kPlotW - app.hud.width(rolled);
    app.hud.text(r, x, 46.0F, rolled, naive ? gd::kOrange : gd::kTealDark, gd::Hud::Style::Bold);
    gd::fillRect(r, x - 24.0F, 49.0F, 16.0F, 14.0F, naive ? gd::kOrange : gd::kTeal);
    x -= 24.0F + app.hud.width("exact") + 22.0F;
    app.hud.text(r, x, 46.0F, "exact", gd::kInk);
    line2(r, x - 24.0F, 56.0F, x - 6.0F, 56.0F, gd::kInk);
}

void panel(App& app, float x, const std::string& heading, const std::array<std::string, 3>& names, const std::array<std::string, 3>& values,
           const std::array<gd::Color, 3>& colors) {
    SDL_Renderer* r = app.renderer;
    gd::card(r, x, kPanelY, kPanelW, kPanelH, gd::kWhite, gd::kInk);
    app.hud.text(r, x + 14.0F, kPanelY + 3.0F, heading, gd::kTealDark, gd::Hud::Style::Bold);
    for (std::size_t i = 0; i < 3; ++i) {
        const float y = kPanelY + 25.0F + 18.0F * static_cast<float>(i);
        app.hud.text(r, x + 14.0F, y, names[i], gd::kMuted);
        app.hud.text(r, x + 110.0F, y, values[i], colors[i]);
    }
}

void drawPanels(App& app) {
    const bool naive = app.method != Method::Correct;
    const gd::Color rolledColor = naive ? gd::kOrange : gd::kTealDark;
    const dd::Histogram& h = *app.hist;
    const bool any = h.total() > 0;
    const int rawTarget = app.target - app.bonus;

    // Mean and spread.
    {
        const double m = dd::mean(app.ideal);
        const std::string exact = fixed(m) + "  sd " + fixed(dd::stddev(app.ideal));
        const std::string rolled = any ? fixed(h.mean() + app.bonus) + "  sd " + fixed(std::sqrt(h.variance())) : "-";
        const std::string flat = fixed(dd::mean(app.flat)) + "  sd " + fixed(dd::stddev(app.flat));
        panel(app, kPlotX, "MEAN AND SPREAD", {"exact", "rolled", "flat roll"}, {exact, rolled, flat}, {gd::kInk, rolledColor, gd::kMuted});
    }
    // Chance of reaching the target.
    {
        const std::string rolled = any ? pct(h.fractionAtLeast(rawTarget)) : "-";
        panel(app, kPlotX + kPanelW + 20.0F, "CHANCE OF " + std::to_string(app.target) + "+", {"exact", "rolled", "flat roll"},
              {pct(dd::atLeast(app.ideal, app.target)), rolled, pct(dd::atLeast(app.flat, app.target))}, {gd::kInk, rolledColor, gd::kMuted});
    }
    // What one more point of bonus is worth.
    {
        const dd::WorthRange range = dd::bonusWorthRange(app.ideal);
        const std::string here = "+" + fixed(dd::bonusWorth(app.ideal, app.target) * 100.0, 1) + " points";
        const std::string spread = fixed(range.least * 100.0, 1) + " to " + fixed(range.most * 100.0, 1);
        const std::string flat = "+" + fixed(dd::bonusWorth(app.flat, app.target) * 100.0, 1) + " points";
        panel(app, kPlotX + 2.0F * (kPanelW + 20.0F), "ONE MORE BONUS POINT", {"here", "anywhere", "flat roll"}, {here, spread, flat},
              {gd::kInk, gd::kInk, gd::kMuted});
    }
}

void drawChips(App& app) {
    SDL_Renderer* r = app.renderer;
    for (const Text& t : app.labels) app.hud.text(r, t.x, t.y, t.text, gd::kMuted);
    for (const Chip& c : app.chips) {
        const bool on = active(app, c);
        const bool warn = c.action == Action::SetMethod && c.value != 0;
        const gd::Color fill = on ? (warn ? gd::kOrange : gd::kTeal) : gd::kWhite;
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, fill, gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 3.0F, c.label, on ? gd::kWhite : gd::kInk,
                     gd::Hud::Style::Bold);
    }
    const std::string bonus = app.bonus > 0 ? "+" + std::to_string(app.bonus) : std::to_string(app.bonus);
    drawCentered(app, app.bonusValueX + 30.0F, kRow2 + 3.0F, bonus, gd::kInk, gd::Hud::Style::Bold);
    drawCentered(app, app.targetValueX + 30.0F, kRow2 + 3.0F, std::to_string(app.target) + "+", gd::kInk, gd::Hud::Style::Bold);

    const char* keys = app.mode == Mode::Dice ? "keys: 1-0 dice, Left/Right bonus, Up/Down need, M method, T mode, click a bar: need" : "keys: Left/Right bonus, Up/Down need, M method, T mode, Space pause, R roll again";
    app.hud.text(r, kPlotX, 514.0F, keys, gd::kMuted);
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    std::string right = dd::formatCount(app.hist->total()) + " rolls";
    if (app.hist->total() >= kMaxRolls) right += " (max)";
    else if (app.paused) right += " (paused)";
    app.hud.titleBar(r, kWidth, "[ DICE AND DAMAGE ROLLS ]", right);
    drawHeader(app);
    drawPlot(app);
    drawPanels(app);
    drawChips(app);
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
    if (!SDL_CreateWindowAndRenderer("damage-dice", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, gd::kLogicalPresentation);
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderVSync(app->renderer, 1);
    app->presets = {dd::flatTable(), dd::rampTable(), dd::critTable()};
    app->table = app->presets[2];
    app->rng.seed(0x5eedULL, 2027ULL);
    rebuildModel(*app);
    layoutChips(*app);
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION:
            app.mouse = SDL_FPoint{event->motion.x, event->motion.y};
            if (app.scrubbing) scrubTo(app, app.mouse.x);
            break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            const SDL_FPoint p{event->button.x, event->button.y};
            app.mouse = p;
            const bool left = event->button.button == SDL_BUTTON_LEFT;
            const bool right = event->button.button == SDL_BUTTON_RIGHT;
            if (left) {
                for (const Chip& c : app.chips) {
                    if (SDL_PointInRectFloat(&p, &c.rect)) {
                        apply(app, c.action, c.value);
                        return SDL_APP_CONTINUE;  // the chip list may have been rebuilt, do not walk it any further
                    }
                }
            }
            if (!inPlot(p) || (!left && !right)) break;
            if (app.mode == Mode::Dice) {
                if (left) {
                    app.scrubbing = true;
                    scrubTo(app, p.x);
                }
            } else if (const auto v = app.axis.valueAt(static_cast<double>(p.x))) {
                editWeight(app, *v, left ? 1 : -1);
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event->button.button == SDL_BUTTON_LEFT) app.scrubbing = false;
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
            app.scrubbing = false;
            app.mouse = SDL_FPoint{-100.0F, -100.0F};
            break;
        case SDL_EVENT_WINDOW_FOCUS_LOST: app.scrubbing = false; break;
        case SDL_EVENT_KEY_DOWN: {
            const SDL_Keycode key = event->key.key;
            const bool repeat = event->key.repeat;
            if (key == SDLK_ESCAPE) return SDL_APP_SUCCESS;
            // Stepping keys may repeat, everything that toggles or restarts may not.
            if (key == SDLK_LEFT) apply(app, Action::Bonus, -1);
            else if (key == SDLK_RIGHT) apply(app, Action::Bonus, 1);
            else if (key == SDLK_DOWN) apply(app, Action::Target, -1);
            else if (key == SDLK_UP) apply(app, Action::Target, 1);
            else if (repeat) break;
            else if (key == SDLK_SPACE) apply(app, Action::Pause, 0);
            else if (key == SDLK_R) apply(app, Action::Reset, 0);
            else if (key == SDLK_M) setMethod(app, static_cast<int>(app.method) + 1);
            else if (key == SDLK_T) setMode(app, app.mode == Mode::Dice ? Mode::Table : Mode::Dice);
            else if (key >= SDLK_0 && key <= SDLK_9 && app.mode == Mode::Dice) apply(app, Action::Dice, key == SDLK_0 ? 10 : static_cast<int>(key - SDLK_0));
            break;
        }
        default: break;
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void* appstate) {
    auto& app = *static_cast<App*>(appstate);
    if (!app.paused) rollSome(app, kPerFrame);
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
