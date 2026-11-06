// XP curves with SDL3 main callbacks.
//
// How much experience does the next level cost? The top plot draws that cost (orange) and the running total (teal) for
// five curve families; two sliders change the parameters, a chip switches the y axis between linear and logarithmic
// (the only way to see an exponential curve and a linear one on the same sheet). The bottom plot turns XP into play
// time for a given XP per hour, the dashed line is what "every level takes the same time" would look like. A simulated
// player earns XP in fixed steps and climbs the curve live; hover the plots to read level, cost, total and hours.
// The curve math, the level lookup (binary search) and the formatting live in logic.hpp and are covered by test.cpp.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "hud.hpp"
#include "logic.hpp"

namespace {

constexpr int kWidth = 960;
constexpr int kHeight = 540;
constexpr double kStep = 1.0 / 60.0;  // fixed simulation step in real seconds
constexpr double kHoldAtMax = 3.0;    // real seconds the player stays at the cap before starting over
constexpr double kPulseTime = 0.7;    // real seconds a level-up impulse lasts
constexpr int kFamilies = 5;

struct Rect {
    float x0, y0, x1, y1;
    float w() const { return x1 - x0; }
    float h() const { return y1 - y0; }
};

constexpr Rect kPlotXp{84.0F, 92.0F, 640.0F, 262.0F};
constexpr Rect kPlotTime{84.0F, 298.0F, 640.0F, 416.0F};
constexpr float kPanelX = 668.0F;
constexpr float kPanelW = 270.0F;
constexpr float kChipH = 28.0F;

constexpr gd::Color kInkSoft{17, 24, 39, 120};
constexpr gd::Color kFillTeal{13, 148, 136, 45};

enum class Action { Family, LogY, Levels, Speed, Pause, Restart };
enum class Drag { None, A, B, Rate };

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

    xpc::Family family = xpc::Family::RuneScape;
    std::array<xpc::Params, kFamilies> params{};
    xpc::Curve curve{xpc::Family::RuneScape, {}};
    bool logY = false;
    int levels = 99;
    double rate = 100000.0;  // XP per hour

    int speed = 10;  // hours of play per real second
    bool paused = false;
    xpc::Player player;
    double played = 0.0;  // hours
    double pulse = 0.0;   // 1 right after a level-up, fades to 0
    double hold = 0.0;
    double accumulator = 0.0;
    Uint64 lastTick = 0;

    SDL_FPoint mouse{-100.0F, -100.0F};
    Drag drag = Drag::None;
    long maxFrames = -1;
    long frames = 0;
};

int familyIndex(xpc::Family f) { return static_cast<int>(f); }

const char* familyName(xpc::Family f) {
    switch (f) {
        case xpc::Family::Linear: return "linear";
        case xpc::Family::Polynomial: return "polynomial";
        case xpc::Family::Exponential: return "exponential";
        case xpc::Family::RuneScape: return "RuneScape";
        case xpc::Family::Table: return "table";
    }
    return "";
}

const char* formula(xpc::Family f) {
    switch (f) {
        case xpc::Family::Linear: return "step(L) = a * L";
        case xpc::Family::Polynomial: return "step(L) = a * L^b";
        case xpc::Family::Exponential: return "step(L) = a * b^L";
        case xpc::Family::RuneScape: return "total = floor(sum floor(l + 300*2^(l/7)) / 4)";
        case xpc::Family::Table: return "11 anchors, interpolated, rounded to 10";
    }
    return "";
}

const char* labelA(xpc::Family f) { return f == xpc::Family::Linear ? "a  XP per level" : f == xpc::Family::RuneScape || f == xpc::Family::Table ? "a  (fixed curve)" : "a  scale"; }

const char* labelB(xpc::Family f) { return f == xpc::Family::Polynomial ? "b  exponent" : f == xpc::Family::Exponential ? "b  growth" : f == xpc::Family::Linear ? "b  (not used)" : "b  (fixed curve)"; }

void rebuild(App& app) { app.curve = xpc::Curve(app.family, app.params[static_cast<std::size_t>(familyIndex(app.family))]); }

void restart(App& app) {
    app.player = xpc::Player{};
    app.played = 0.0;
    app.hold = 0.0;
    app.pulse = 0.0;
}

void setFamily(App& app, xpc::Family f) {
    app.family = f;
    rebuild(app);
}

// ---- chips ------------------------------------------------------------------------------------------------------

void layoutChips(App& app) {
    float x = 12.0F;
    auto add = [&](Action action, int value, std::string label, float gapBefore) {
        x += gapBefore;
        const float w = app.hud.width(label, gd::Hud::Style::Bold) + 20.0F;
        app.chips.push_back({action, value, std::move(label), SDL_FRect{x, 38.0F, w, kChipH}});
        x += w;
    };
    add(Action::Family, 0, "1 Linear", 0.0F);
    add(Action::Family, 1, "2 Poly", 6.0F);
    add(Action::Family, 2, "3 Exp", 6.0F);
    add(Action::Family, 3, "4 RuneScape", 6.0F);
    add(Action::Family, 4, "5 Table", 6.0F);
    add(Action::LogY, 0, "L Log y", 18.0F);
    x += 18.0F + app.hud.width("levels") + 8.0F;
    add(Action::Levels, 30, "30", 0.0F);
    add(Action::Levels, 60, "60", 6.0F);
    add(Action::Levels, 99, "99", 6.0F);

    x = 12.0F + app.hud.width("play speed") + 10.0F;
    auto addBottom = [&](Action action, int value, std::string label, float gapBefore) {
        x += gapBefore;
        const float w = app.hud.width(label, gd::Hud::Style::Bold) + 20.0F;
        app.chips.push_back({action, value, std::move(label), SDL_FRect{x, 474.0F, w, kChipH}});
        x += w;
    };
    addBottom(Action::Speed, 1, "x1", 0.0F);
    addBottom(Action::Speed, 10, "x10", 6.0F);
    addBottom(Action::Speed, 100, "x100", 6.0F);
    addBottom(Action::Pause, 0, "Space Pause", 18.0F);
    addBottom(Action::Restart, 0, "R Restart", 6.0F);
}

void apply(App& app, const Chip& c) {
    switch (c.action) {
        case Action::Family: setFamily(app, static_cast<xpc::Family>(c.value)); break;
        case Action::LogY: app.logY = !app.logY; break;
        case Action::Levels: app.levels = c.value; break;
        case Action::Speed: app.speed = c.value; break;
        case Action::Pause: app.paused = !app.paused; break;
        case Action::Restart: restart(app); break;
    }
}

bool isActive(const App& app, const Chip& c) {
    switch (c.action) {
        case Action::Family: return familyIndex(app.family) == c.value;
        case Action::LogY: return app.logY;
        case Action::Levels: return app.levels == c.value;
        case Action::Speed: return app.speed == c.value;
        case Action::Pause: return app.paused;
        case Action::Restart: return false;
    }
    return false;
}

// ---- sliders ----------------------------------------------------------------------------------------------------

constexpr float kSliderX = kPanelX + 16.0F;
constexpr float kSliderW = kPanelW - 32.0F;

float sliderTrackY(int row) { return 76.0F + 14.0F + static_cast<float>(row) * 44.0F + 28.0F; }

bool sliderUsed(const App& app, Drag d) {
    const xpc::ParamInfo info = xpc::paramInfo(app.family);
    return d == Drag::A ? info.a.used : d == Drag::B ? info.b.used : true;
}

double sliderFraction(const App& app, Drag d) {
    const xpc::ParamInfo info = xpc::paramInfo(app.family);
    const xpc::Params& p = app.params[static_cast<std::size_t>(familyIndex(app.family))];
    if (d == Drag::Rate) return xpc::sliderFromRate(app.rate);
    const xpc::Range& r = d == Drag::A ? info.a : info.b;
    if (!r.used || r.hi <= r.lo) return 0.0;
    return std::clamp(((d == Drag::A ? p.a : p.b) - r.lo) / (r.hi - r.lo), 0.0, 1.0);
}

void setSlider(App& app, Drag d, float mouseX) {
    const double t = std::clamp(static_cast<double>((mouseX - kSliderX) / kSliderW), 0.0, 1.0);
    if (d == Drag::Rate) {
        app.rate = xpc::roundSignificant(xpc::rateFromSlider(t), 2);
        return;
    }
    const xpc::ParamInfo info = xpc::paramInfo(app.family);
    const xpc::Range& r = d == Drag::A ? info.a : info.b;
    if (!r.used) return;
    const double unit = std::pow(10.0, r.decimals);
    const double v = std::round((r.lo + t * (r.hi - r.lo)) * unit) / unit;
    xpc::Params& p = app.params[static_cast<std::size_t>(familyIndex(app.family))];
    (d == Drag::A ? p.a : p.b) = v;
    rebuild(app);
}

Drag sliderAt(const App& app, SDL_FPoint p) {
    for (const Drag d : {Drag::A, Drag::B, Drag::Rate}) {
        const int row = d == Drag::A ? 0 : d == Drag::B ? 1 : 2;
        const float y = sliderTrackY(row);
        if (sliderUsed(app, d) && p.x >= kSliderX - 8.0F && p.x <= kSliderX + kSliderW + 8.0F && p.y >= y - 14.0F && p.y <= y + 14.0F) return d;
    }
    return Drag::None;
}

// ---- plot helpers -----------------------------------------------------------------------------------------------

float xOf(const Rect& p, double level, int n) { return p.x0 + static_cast<float>((level - 1.0) / static_cast<double>(n - 1)) * p.w(); }

float yOf(const Rect& p, const xpc::Axis& axis, double v) { return p.y1 - static_cast<float>(axis.map(v)) * p.h(); }

/// Linear interpolation between the per-level samples; `level` is 1-based and may be fractional.
double sample(const std::vector<double>& v, double level) {
    const double f = std::clamp(level - 1.0, 0.0, static_cast<double>(v.size() - 1));
    const auto i = static_cast<std::size_t>(f);
    const std::size_t j = std::min(i + 1, v.size() - 1);
    return v[i] + (v[j] - v[i]) * (f - static_cast<double>(i));
}

/// 1 px lines are too thin on a 960x540 canvas that is scaled up: three side by side.
void thickLine(SDL_Renderer* r, float x0, float y0, float x1, float y1, gd::Color c, int half = 1) {
    gd::setColor(r, c);
    const bool flat = std::fabs(x1 - x0) > std::fabs(y1 - y0);
    for (int o = -half; o <= half; ++o) {
        const auto off = static_cast<float>(o);
        SDL_RenderLine(r, x0 + (flat ? 0.0F : off), y0 + (flat ? off : 0.0F), x1 + (flat ? 0.0F : off), y1 + (flat ? off : 0.0F));
    }
}

float labelX(const App& app, const Rect& p, const std::string& s) { return p.x0 - 8.0F - app.hud.width(s); }

void drawFrame(App& app, const Rect& p, const xpc::Axis& axis, int n) {
    SDL_Renderer* r = app.renderer;
    gd::card(r, p.x0, p.y0, p.w(), p.h(), gd::kWhite, gd::kInk);
    for (const double t : axis.ticks) {
        const float y = yOf(p, axis, t);
        gd::fillRect(r, p.x0, y, p.w(), 1.0F, gd::kGrid);
        const std::string s = xpc::formatShort(t);
        app.hud.text(r, labelX(app, p, s), y - 11.0F, s, gd::kMuted);
    }
    const int every = n <= 30 ? 5 : 10;
    for (int l = 1; l <= n; l = (l == 1 ? every : l + every)) {
        gd::fillRect(r, xOf(p, l, n), p.y0, 1.0F, p.h(), gd::kGrid);
    }
    gd::strokeRect(r, p.x0, p.y0, p.w(), p.h(), gd::kInk);
}

void drawXLabels(App& app, const Rect& p, int n) {
    const int every = n <= 30 ? 5 : 10;
    auto label = [&](int l) {
        const std::string s = std::to_string(l);
        app.hud.text(app.renderer, xOf(p, l, n) - app.hud.width(s) / 2.0F, p.y1 + 6.0F, s, gd::kMuted);
    };
    for (int l = 1; l <= n; l = (l == 1 ? every : l + every)) label(l);
    if (n % every != 0) label(n);
    app.hud.text(app.renderer, p.x0 - 20.0F - app.hud.width("level"), p.y1 + 6.0F, "level", gd::kMuted);
}

void drawSeries(App& app, const Rect& p, const xpc::Axis& axis, const std::vector<double>& v, gd::Color color, int n) {
    SDL_Renderer* r = app.renderer;
    for (int l = 1; l < n; ++l) {
        thickLine(r, xOf(p, l, n), yOf(p, axis, v[static_cast<std::size_t>(l) - 1]), xOf(p, l + 1, n), yOf(p, axis, v[static_cast<std::size_t>(l)]), color);
    }
    if (n <= 30) {
        for (int l = 1; l <= n; ++l) {
            const float x = xOf(p, l, n);
            const float y = yOf(p, axis, v[static_cast<std::size_t>(l) - 1]);
            gd::fillRect(r, x - 3.0F, y - 3.0F, 7.0F, 7.0F, gd::kWhite);
            gd::strokeRect(r, x - 3.0F, y - 3.0F, 7.0F, 7.0F, color);
        }
    }
}

/// Light fill below the curve up to the player: how far along the curve the player is.
void drawProgress(App& app, const Rect& p, const xpc::Axis& axis, const std::vector<double>& v, double upTo, int n) {
    const double end = std::min(upTo, static_cast<double>(n));
    for (float x = p.x0 + 1.0F; x < xOf(p, end, n); x += 2.0F) {
        const double level = 1.0 + static_cast<double>((x - p.x0) / p.w()) * static_cast<double>(n - 1);
        const float y = yOf(p, axis, sample(v, level));
        gd::fillRect(app.renderer, x, y, 2.0F, p.y1 - y, kFillTeal);
    }
}

void legendRow(App& app, float x, float y, gd::Color color, const char* text, bool dashed = false) {
    SDL_Renderer* r = app.renderer;
    if (dashed) {
        gd::fillRect(r, x, y + 10.0F, 6.0F, 2.0F, color);
        gd::fillRect(r, x + 10.0F, y + 10.0F, 6.0F, 2.0F, color);
    } else {
        gd::fillRect(r, x, y + 8.0F, 16.0F, 5.0F, color);
    }
    app.hud.text(r, x + 24.0F, y, text, gd::kInk);
}

// ---- drawing ----------------------------------------------------------------------------------------------------

struct Data {
    std::vector<double> total, step, hoursTotal, hoursStep;
    xpc::Axis axisXp, axisTime;
};

Data collect(const App& app) {
    Data d;
    const int n = app.levels;
    for (int l = 1; l <= xpc::kMaxLevel; ++l) {
        d.total.push_back(static_cast<double>(app.curve.total(l)));
        d.step.push_back(static_cast<double>(app.curve.next(l)));
        d.hoursTotal.push_back(xpc::hoursToLevel(app.curve, l, app.rate));
        d.hoursStep.push_back(xpc::hoursForNext(app.curve, l, app.rate));
    }
    const auto lowest = [&](const std::vector<double>& v) { return *std::min_element(v.begin(), v.begin() + n); };
    const auto highest = [&](const std::vector<double>& a, const std::vector<double>& b) {
        return std::max(*std::max_element(a.begin(), a.begin() + n), *std::max_element(b.begin(), b.begin() + n));
    };
    d.axisXp = app.logY ? xpc::makeAxis(lowest(d.step), highest(d.total, d.step), true, 7, 1.0) : xpc::makeAxis(0.0, highest(d.total, d.step), false, 4);
    d.axisTime = app.logY ? xpc::makeAxis(lowest(d.hoursStep), highest(d.hoursTotal, d.hoursStep), true, 5, 1e-4)
                          : xpc::makeAxis(0.0, highest(d.hoursTotal, d.hoursStep), false, 3);
    return d;
}

/// The level under the cursor, or 0 when the cursor is not over the plots.
int hoveredLevel(const App& app) {
    const SDL_FPoint m = app.mouse;
    if (m.x < kPlotXp.x0 - 6.0F || m.x > kPlotXp.x1 + 6.0F || m.y < kPlotXp.y0 || m.y > kPlotTime.y1) return 0;
    const double t = static_cast<double>((m.x - kPlotXp.x0) / kPlotXp.w());
    return std::clamp(static_cast<int>(std::lround(1.0 + t * (app.levels - 1))), 1, app.levels);
}

void drawTooltip(App& app, int level) {
    SDL_Renderer* r = app.renderer;
    const double rate = app.rate;
    struct Row {
        const char* label;
        std::string value;
    };
    const std::array<Row, 4> rows{{{"to next", xpc::formatThousands(app.curve.next(level)) + " XP"},
                                   {"total", xpc::formatThousands(app.curve.total(level)) + " XP"},
                                   {"play time", xpc::formatHours(xpc::hoursToLevel(app.curve, level, rate))},
                                   {"this level", xpc::formatHours(xpc::hoursForNext(app.curve, level, rate))}}};
    constexpr float w = 236.0F;
    constexpr float h = 118.0F;
    float x = app.mouse.x + 16.0F;
    if (x + w > kPlotXp.x1 + 8.0F) x = app.mouse.x - 16.0F - w;  // stay over the plots, the panel on the right keeps its sliders visible
    const float y = std::clamp(app.mouse.y - h / 2.0F, 40.0F, 428.0F - h);
    gd::card(r, x, y, w, h, gd::kWhite, gd::kInk);
    char head[32];
    SDL_snprintf(head, sizeof head, "LEVEL %d", level);
    app.hud.text(r, x + 12.0F, y + 8.0F, head, gd::kTealDark, gd::Hud::Style::Bold);
    for (std::size_t i = 0; i < rows.size(); ++i) {
        const float ry = y + 34.0F + static_cast<float>(i) * 20.0F;
        app.hud.text(r, x + 12.0F, ry, rows[i].label, gd::kMuted);
        app.hud.text(r, x + w - 12.0F - app.hud.width(rows[i].value, gd::Hud::Style::Bold), ry, rows[i].value, gd::kInk, gd::Hud::Style::Bold);
    }
}

void drawPlayerMarker(App& app, const Data& d, int level, double progress) {
    SDL_Renderer* r = app.renderer;
    const double at = level + progress;
    if (at > app.levels) return;
    const float cx = xOf(kPlotXp, at, app.levels);
    const float cy1 = yOf(kPlotXp, d.axisXp, sample(d.total, at));
    const float cy2 = yOf(kPlotTime, d.axisTime, sample(d.hoursTotal, at));
    thickLine(r, cx, kPlotXp.y0, cx, kPlotTime.y1, gd::Color{234, 88, 12, 90}, 0);
    for (const float cy : {cy1, cy2}) {
        if (app.pulse > 0.0) {
            const auto a = static_cast<Uint8>(200.0 * app.pulse);
            gd::ring(r, cx, cy, 8.0F + 26.0F * static_cast<float>(1.0 - app.pulse), gd::Color{234, 88, 12, a});
            gd::ring(r, cx, cy, 9.0F + 26.0F * static_cast<float>(1.0 - app.pulse), gd::Color{234, 88, 12, a});
        }
        gd::disc(r, cx, cy, 7.0F, gd::kWhite);
        gd::disc(r, cx, cy, 4.0F, gd::kOrange);
        gd::ring(r, cx, cy, 7.0F, gd::kInk);
    }
}

void drawSlider(App& app, int row, Drag d, const char* label, const std::string& value) {
    SDL_Renderer* r = app.renderer;
    const bool used = sliderUsed(app, d);
    const float ty = sliderTrackY(row);
    const gd::Color text = used ? gd::kInk : gd::kMuted;
    app.hud.text(r, kSliderX, ty - 28.0F, label, text, gd::Hud::Style::Bold);
    app.hud.text(r, kSliderX + kSliderW - app.hud.width(value, gd::Hud::Style::Bold), ty - 28.0F, value, used ? gd::kTealDark : gd::kMuted, gd::Hud::Style::Bold);
    gd::fillRect(r, kSliderX, ty - 2.0F, kSliderW, 5.0F, used ? gd::Color{17, 24, 39, 60} : gd::Color{17, 24, 39, 25});
    if (!used) return;
    const float hx = kSliderX + static_cast<float>(sliderFraction(app, d)) * kSliderW;
    gd::fillRect(r, kSliderX, ty - 2.0F, hx - kSliderX, 5.0F, gd::kTeal);
    const bool held = app.drag == d;
    gd::card(r, hx - 6.0F, ty - 10.0F, 12.0F, 20.0F, held ? gd::kOrange : gd::kTeal, gd::kInk);
}

void drawPanel(App& app, int level, double progress) {
    SDL_Renderer* r = app.renderer;
    const xpc::ParamInfo info = xpc::paramInfo(app.family);
    const xpc::Params& p = app.params[static_cast<std::size_t>(familyIndex(app.family))];

    gd::card(r, kPanelX, 76.0F, kPanelW, 158.0F, gd::kWhite, gd::kInk);
    auto number = [](const xpc::Range& range, double v) {
        if (range.decimals == 0) return xpc::formatThousands(std::llround(v));
        char buf[32];
        std::snprintf(buf, sizeof buf, "%.*f", range.decimals, v);
        return std::string(buf);
    };
    drawSlider(app, 0, Drag::A, labelA(app.family), info.a.used ? number(info.a, p.a) : "-");
    drawSlider(app, 1, Drag::B, labelB(app.family), info.b.used ? number(info.b, p.b) : "-");
    drawSlider(app, 2, Drag::Rate, "XP per hour", xpc::formatThousands(std::llround(app.rate)));

    const float cy = 248.0F;
    const bool atMax = level >= xpc::kMaxLevel;
    gd::card(r, kPanelX, cy, kPanelW, 172.0F, gd::kWhite, gd::kInk);
    app.hud.text(r, kPanelX + 14.0F, cy + 8.0F, "PLAYER", gd::kMuted);
    char big[32];
    SDL_snprintf(big, sizeof big, "LEVEL %d", level);
    const gd::Color base = gd::kTeal;
    const auto mix = [&](Uint8 a, Uint8 b) { return static_cast<Uint8>(static_cast<double>(a) * (1.0 - app.pulse) + static_cast<double>(b) * app.pulse); };
    app.hud.text(r, kPanelX + 14.0F, cy + 28.0F, big, gd::Color{mix(base.r, gd::kOrange.r), mix(base.g, gd::kOrange.g), mix(base.b, gd::kOrange.b)}, gd::Hud::Style::Large);
    if (atMax) app.hud.text(r, kPanelX + kPanelW - 14.0F - app.hud.width("MAX", gd::Hud::Style::Bold), cy + 36.0F, "MAX", gd::kOrange, gd::Hud::Style::Bold);

    const float by = cy + 78.0F;
    const float bw = kPanelW - 28.0F;
    gd::fillRect(r, kPanelX + 14.0F, by, bw, 16.0F, gd::kWhite);
    gd::fillRect(r, kPanelX + 14.0F, by, bw * static_cast<float>(progress), 16.0F, gd::kTeal);
    gd::strokeRect(r, kPanelX + 14.0F, by, bw, 16.0F, gd::kInk);

    const std::string xp = atMax ? xpc::formatThousands(app.player.xpInt()) : xpc::formatThousands(app.player.xpInt()) + " / " + xpc::formatThousands(app.curve.total(level + 1));
    app.hud.text(r, kPanelX + 14.0F, cy + 102.0F, xp, gd::kInk);
    app.hud.text(r, kPanelX + 14.0F, cy + 126.0F, "this level takes", gd::kMuted);
    const std::string t = atMax ? "done" : xpc::formatHours(xpc::hoursForNext(app.curve, level, app.rate));
    app.hud.text(r, kPanelX + kPanelW - 14.0F - app.hud.width(t, gd::Hud::Style::Bold), cy + 126.0F, t, gd::kOrange, gd::Hud::Style::Bold);
    app.hud.text(r, kPanelX + 14.0F, cy + 148.0F, "played", gd::kMuted);
    const std::string played = xpc::formatHours(app.played);
    app.hud.text(r, kPanelX + kPanelW - 14.0F - app.hud.width(played, gd::Hud::Style::Bold), cy + 148.0F, played, gd::kInk, gd::Hud::Style::Bold);
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    const Data d = collect(app);
    const int n = app.levels;
    const int level = app.curve.levelForXp(app.player.xpInt());
    const double progress = xpc::levelProgress(app.curve, app.player.xp);
    const double at = level + progress;

    gd::backdrop(r, kWidth, kHeight);
    char right[64];
    SDL_snprintf(right, sizeof right, "%s curve", familyName(app.family));
    app.hud.titleBar(r, kWidth, "[ XP CURVES ]", right);

    for (const Chip& c : app.chips) {
        if (c.rect.y > 100.0F) continue;
        const bool on = isActive(app, c);
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? gd::kTeal : gd::kWhite, gd::kInk);
        app.hud.text(r, c.rect.x + 10.0F, c.rect.y + 3.0F, c.label, on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
    }
    app.hud.text(r, app.chips[6].rect.x - app.hud.width("levels") - 8.0F, 42.0F, "levels", gd::kMuted);

    // Top plot: XP.
    app.hud.text(r, kPlotXp.x0, 72.0F, formula(app.family), gd::kInk, gd::Hud::Style::Bold);
    drawFrame(app, kPlotXp, d.axisXp, n);
    drawProgress(app, kPlotXp, d.axisXp, d.total, at, n);
    drawSeries(app, kPlotXp, d.axisXp, d.step, gd::kOrange, n);
    drawSeries(app, kPlotXp, d.axisXp, d.total, gd::kTeal, n);
    legendRow(app, kPlotXp.x0 + 12.0F, kPlotXp.y0 + 8.0F, gd::kOrange, "XP to next level");
    legendRow(app, kPlotXp.x0 + 12.0F, kPlotXp.y0 + 30.0F, gd::kTeal, "total XP");
    if (app.logY) app.hud.text(r, kPlotXp.x1 - 8.0F - app.hud.width("log y"), kPlotXp.y1 - 24.0F, "log y", gd::kMuted);

    // Bottom plot: hours at the chosen XP per hour.
    char title[64];
    SDL_snprintf(title, sizeof title, "play time at %s XP per hour", xpc::formatThousands(std::llround(app.rate)).c_str());
    app.hud.text(r, kPlotTime.x0, 274.0F, title, gd::kInk, gd::Hud::Style::Bold);
    drawFrame(app, kPlotTime, d.axisTime, n);
    drawXLabels(app, kPlotTime, n);
    drawProgress(app, kPlotTime, d.axisTime, d.hoursTotal, at, n);
    // Dashed: the straight line "every level takes the same time" would give.
    if (!app.logY) {
        const double lastHours = d.hoursTotal[static_cast<std::size_t>(n) - 1];
        for (int i = 0; i < 4 * (n - 1); i += 6) {
            const double l0 = 1.0 + static_cast<double>(i) / 4.0;
            const double l1 = 1.0 + static_cast<double>(std::min(i + 3, 4 * (n - 1))) / 4.0;
            const auto pace = [&](double l) { return lastHours * (l - 1.0) / static_cast<double>(n - 1); };
            thickLine(r, xOf(kPlotTime, l0, n), yOf(kPlotTime, d.axisTime, pace(l0)), xOf(kPlotTime, l1, n), yOf(kPlotTime, d.axisTime, pace(l1)), gd::kMuted, 0);
        }
    }
    drawSeries(app, kPlotTime, d.axisTime, d.hoursStep, gd::kOrange, n);
    drawSeries(app, kPlotTime, d.axisTime, d.hoursTotal, gd::kTeal, n);
    legendRow(app, kPlotTime.x0 + 12.0F, kPlotTime.y0 + 6.0F, gd::kTeal, "hours to reach level");
    legendRow(app, kPlotTime.x0 + 12.0F, kPlotTime.y0 + 24.0F, gd::kOrange, "hours for this level");
    if (!app.logY) legendRow(app, kPlotTime.x0 + 12.0F, kPlotTime.y0 + 42.0F, gd::kMuted, "same time per level", true);

    drawPlayerMarker(app, d, level, progress);

    const int hover = hoveredLevel(app);
    if (hover > 0) {
        const float x = xOf(kPlotXp, hover, n);
        thickLine(r, x, kPlotXp.y0, x, kPlotXp.y1, kInkSoft, 0);
        thickLine(r, x, kPlotTime.y0, x, kPlotTime.y1, kInkSoft, 0);
        const auto idx = static_cast<std::size_t>(hover) - 1;
        struct Dot {
            const Rect* rect;
            const xpc::Axis* axis;
            double value;
            gd::Color color;
        };
        const std::array<Dot, 4> dots{{{&kPlotXp, &d.axisXp, d.total[idx], gd::kTeal},
                                       {&kPlotXp, &d.axisXp, d.step[idx], gd::kOrange},
                                       {&kPlotTime, &d.axisTime, d.hoursTotal[idx], gd::kTeal},
                                       {&kPlotTime, &d.axisTime, d.hoursStep[idx], gd::kOrange}}};
        for (const Dot& dot : dots) {
            const float y = yOf(*dot.rect, *dot.axis, dot.value);
            gd::disc(r, x, y, 5.0F, dot.color);
            gd::ring(r, x, y, 5.0F, gd::kInk);
        }
    }

    drawPanel(app, level, progress);

    app.hud.text(r, 12.0F, 482.0F, "play speed", gd::kMuted);
    for (const Chip& c : app.chips) {
        if (c.rect.y < 100.0F) continue;
        const bool on = isActive(app, c);
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? gd::kTeal : gd::kWhite, gd::kInk);
        app.hud.text(r, c.rect.x + 10.0F, c.rect.y + 3.0F, c.label, on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
    }
    app.hud.text(r, 948.0F - app.hud.width("x1 = 1 s is 1 h of play"), 482.0F, "x1 = 1 s is 1 h of play", gd::kMuted);
    app.hud.text(r, 12.0F, 448.0F, "hover: read a level   drag: sliders   keys: 1-5 L Space R", gd::kMuted);

    if (hover > 0) drawTooltip(app, hover);
}

void stepSim(App& app) {
    if (app.hold > 0.0) {
        app.hold -= kStep;
        if (app.hold <= 0.0) restart(app);
        return;
    }
    const int before = app.curve.levelForXp(app.player.xpInt());
    const double hours = static_cast<double>(app.speed) * kStep;
    const double xpBefore = app.player.xp;
    xpc::advance(app.player, app.curve, hours, app.rate);
    app.played += (app.player.xp - xpBefore) / app.rate;  // exact, also in the step that hits the cap
    const int after = app.curve.levelForXp(app.player.xpInt());
    if (after > before) app.pulse = 1.0;
    if (after >= xpc::kMaxLevel) app.hold = kHoldAtMax;
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
    if (!SDL_CreateWindowAndRenderer("xp-curves", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, gd::kLogicalPresentation);
    SDL_SetRenderVSync(app->renderer, 1);
    for (int i = 0; i < kFamilies; ++i) app->params[static_cast<std::size_t>(i)] = xpc::paramInfo(static_cast<xpc::Family>(i)).def;
    rebuild(*app);
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
            if (app.drag != Drag::None) setSlider(app, app.drag, event->motion.x);
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
            app.drag = sliderAt(app, p);
            if (app.drag != Drag::None) setSlider(app, app.drag, p.x);
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event->button.button == SDL_BUTTON_LEFT) app.drag = Drag::None;
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
        case SDL_EVENT_WINDOW_FOCUS_LOST:
            app.drag = Drag::None;
            app.mouse = {-100.0F, -100.0F};
            break;
        case SDL_EVENT_KEY_DOWN:
            if (event->key.repeat) break;  // every key here toggles or triggers, none should fire while held
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: setFamily(app, xpc::Family::Linear); break;
                case SDLK_2: setFamily(app, xpc::Family::Polynomial); break;
                case SDLK_3: setFamily(app, xpc::Family::Exponential); break;
                case SDLK_4: setFamily(app, xpc::Family::RuneScape); break;
                case SDLK_5: setFamily(app, xpc::Family::Table); break;
                case SDLK_L: app.logY = !app.logY; break;
                case SDLK_SPACE: app.paused = !app.paused; break;
                case SDLK_R: restart(app); break;
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

    app.pulse = std::max(0.0, app.pulse - dt / kPulseTime);
    if (!app.paused) {
        app.accumulator += dt;
        while (app.accumulator >= kStep) {
            app.accumulator -= kStep;
            stepSim(app);
        }
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
