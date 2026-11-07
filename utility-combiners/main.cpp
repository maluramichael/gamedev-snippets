// Utility AI: combining considerations, SDL3 main callbacks.
//
// A villager decides every tenth of a second which of four actions scores highest. Each action multiplies or averages
// a few considerations, and the combiner is what the demo lets you swap: the mean lets one zero factor hide among
// good ones, the product gives every factor a veto, bands rank survival actions above everything else. The panel on
// the right shows what each combiner would pick in the current state, so the disagreement is visible without waiting
// for the simulation to walk into it. The timeline shows the scores over the last 60 s; the strip under it is the
// chosen action, which flickers whenever two scores keep crossing. The scoring lives in logic.hpp, test.cpp covers it.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <array>
#include <cctype>
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
constexpr gd::Color kIndigo{79, 70, 229};
constexpr gd::Color kNight{17, 24, 39, 24};
constexpr gd::Color kInkSoft{17, 24, 39, 120};

// Layout of the top row: villager bars, curve panel, four action cards.
constexpr float kPanelY = 42.0F;
constexpr float kPanelH = 148.0F;
constexpr float kBarX = 124.0F;
constexpr float kBarW = 140.0F;
constexpr float kBarH = 14.0F;
constexpr float kPlotX = 306.0F;
constexpr float kPlotY = 72.0F;
constexpr float kPlotW = 164.0F;
constexpr float kPlotH = 72.0F;
constexpr float kCardX = 504.0F;
constexpr float kCardW = 100.0F;
constexpr float kCardGap = 10.0F;
// Timeline.
constexpr float kLineX = 24.0F;
constexpr float kLineY = 204.0F;
constexpr float kLineW = 720.0F;  // 600 decisions at 1.2 px each
constexpr float kLineH = 146.0F;
constexpr float kStripY = 356.0F;
constexpr float kStripH = 12.0F;
// Chips.
constexpr float kChipX = 120.0F;
constexpr float kChipH = 30.0F;
constexpr float kChipRows[3] = {402.0F, 436.0F, 470.0F};
constexpr double kInertiaOn = 0.15;

enum class Act {
    Mean, Product, Bands, Inertia, Pause, Reset,
    CurveHunger, CurveFatigue, CurveDrive, Linear, Quadratic, Logistic,
    ScenarioNight, ScenarioEager
};

struct Chip {
    Act act;
    std::string label;
    SDL_FRect rect;
};

enum class Drag { None, Need, Mid };

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;
    ua::Sim sim;
    bool paused = false;
    int curveTarget = 0;  // 0 hunger, 1 fatigue, 2 drive
    Drag drag = Drag::None;
    int dragNeed = 0;
    Uint64 lastTicks = 0;
    std::array<SDL_FPoint, ua::kHistory> points{};
    long maxFrames = -1;
    long frames = 0;
};

const char* const kNeedNames[4] = {"Hunger", "Fatigue", "Drive", "Larder"};
const char* const kActionNames[ua::kActionCount] = {"Eat", "Sleep", "Work", "Wander"};
const char* const kFactorNames[ua::kActionCount][2] = {{"hunger", "larder"}, {"tired", "night"}, {"drive", "day"}, {"idle", ""}};

gd::Color colorOf(ua::Action a) {
    switch (a) {
        case ua::Action::Eat: return gd::kOrange;
        case ua::Action::Sleep: return kIndigo;
        case ua::Action::Work: return gd::kTeal;
        case ua::Action::Wander: return gd::kMuted;
    }
    return gd::kMuted;
}

const char* combinerName(ua::Combiner c) {
    switch (c) {
        case ua::Combiner::Mean: return "mean";
        case ua::Combiner::Product: return "product";
        case ua::Combiner::Bands: return "bands";
    }
    return "";
}

double& needRef(ua::State& s, int i) {
    switch (i) {
        case 0: return s.hunger;
        case 1: return s.fatigue;
        case 2: return s.drive;
        default: return s.larder;
    }
}

ua::Curve& curveRef(ua::Settings& s, int i) {
    switch (i) {
        case 0: return s.hunger;
        case 1: return s.fatigue;
        default: return s.drive;
    }
}

const ua::Curve& curveOf(const ua::Settings& s, int i) { return curveRef(const_cast<ua::Settings&>(s), i); }

float barY(int row) { return 74.0F + 22.0F * static_cast<float>(row); }

bool inRect(const SDL_FRect& r, float x, float y) { return x >= r.x && x <= r.x + r.w && y >= r.y && y <= r.y + r.h; }

/// A fresh villager whose history is already full, so the first frame shows a whole minute.
void reset(App& app) {
    ua::Sim fresh;
    fresh.settings = app.sim.settings;
    for (std::size_t i = 0; i < ua::kHistory; ++i) fresh.tick();
    app.sim = fresh;
    app.paused = false;
    app.drag = Drag::None;
}

void loadScenario(App& app, const ua::State& state) {
    app.sim.state = state;
    app.sim.current = ua::Action::Wander;
    app.sim.pending = 0.0;
    app.paused = true;  // frozen, so the table on the right can be read
    app.drag = Drag::None;
}

void layoutChips(App& app) {
    struct Def {
        Act act;
        const char* label;
        float gapBefore;
        int row;
    };
    const std::array<Def, 14> defs{{{Act::Mean, "1 Mean", 0.0F, 0},
                                    {Act::Product, "2 Product", 8.0F, 0},
                                    {Act::Bands, "3 Bands", 8.0F, 0},
                                    {Act::Inertia, "I Inertia +0.15", 24.0F, 0},
                                    {Act::Pause, "Space Resume", 24.0F, 0},
                                    {Act::Reset, "R Reset", 8.0F, 0},
                                    {Act::CurveHunger, "H Hunger", 0.0F, 1},
                                    {Act::CurveFatigue, "F Fatigue", 8.0F, 1},
                                    {Act::CurveDrive, "D Drive", 8.0F, 1},
                                    {Act::Linear, "4 Linear", 24.0F, 1},
                                    {Act::Quadratic, "5 Quadratic", 8.0F, 1},
                                    {Act::Logistic, "6 Logistic", 8.0F, 1},
                                    {Act::ScenarioNight, "7 Night, empty larder", 0.0F, 2},
                                    {Act::ScenarioEager, "8 Eager but starving", 8.0F, 2}}};
    float x = kChipX;
    int row = 0;
    for (const Def& d : defs) {
        if (d.row != row) {
            row = d.row;
            x = kChipX;
        }
        x += d.gapBefore;
        const float w = app.hud.width(d.label, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({d.act, d.label, SDL_FRect{x, kChipRows[row], w, kChipH}});
        x += w;
    }
}

void apply(App& app, Act act) {
    app.drag = Drag::None;
    ua::Settings& s = app.sim.settings;
    switch (act) {
        case Act::Mean: s.combiner = ua::Combiner::Mean; break;
        case Act::Product: s.combiner = ua::Combiner::Product; break;
        case Act::Bands: s.combiner = ua::Combiner::Bands; break;
        case Act::Inertia: s.inertia = s.inertia > 0.0 ? 0.0 : kInertiaOn; break;
        case Act::Pause: app.paused = !app.paused; break;
        case Act::Reset: reset(app); break;
        case Act::CurveHunger: app.curveTarget = 0; break;
        case Act::CurveFatigue: app.curveTarget = 1; break;
        case Act::CurveDrive: app.curveTarget = 2; break;
        case Act::Linear: curveRef(s, app.curveTarget).type = ua::CurveType::Linear; break;
        case Act::Quadratic: curveRef(s, app.curveTarget).type = ua::CurveType::Quadratic; break;
        case Act::Logistic: curveRef(s, app.curveTarget).type = ua::CurveType::Logistic; break;
        case Act::ScenarioNight: loadScenario(app, ua::emptyLarderAtNight()); break;
        case Act::ScenarioEager: loadScenario(app, ua::eagerButStarving()); break;
    }
}

bool active(const App& app, Act act) {
    const ua::Settings& s = app.sim.settings;
    const ua::CurveType type = curveOf(s, app.curveTarget).type;
    switch (act) {
        case Act::Mean: return s.combiner == ua::Combiner::Mean;
        case Act::Product: return s.combiner == ua::Combiner::Product;
        case Act::Bands: return s.combiner == ua::Combiner::Bands;
        case Act::Inertia: return s.inertia > 0.0;
        case Act::Pause: return app.paused;
        case Act::Reset: return false;
        case Act::CurveHunger: return app.curveTarget == 0;
        case Act::CurveFatigue: return app.curveTarget == 1;
        case Act::CurveDrive: return app.curveTarget == 2;
        case Act::Linear: return type == ua::CurveType::Linear;
        case Act::Quadratic: return type == ua::CurveType::Quadratic;
        case Act::Logistic: return type == ua::CurveType::Logistic;
        case Act::ScenarioNight:
        case Act::ScenarioEager: return false;
    }
    return false;
}

std::string chipLabel(const App& app, const Chip& c) {
    if (c.act == Act::Pause) return app.paused ? "Space Resume" : "Space Pause";
    return c.label;
}

void setNeedFromX(App& app, int need, float x) {
    needRef(app.sim.state, need) = ua::clamp01(static_cast<double>((x - kBarX) / kBarW));
}

void setMidFromX(App& app, float x) {
    curveRef(app.sim.settings, app.curveTarget).mid = std::clamp(static_cast<double>((x - kPlotX) / kPlotW), 0.1, 0.9);
}

void text(App& app, float x, float y, const char* s, gd::Color c, gd::Hud::Style style = gd::Hud::Style::Regular) {
    app.hud.text(app.renderer, x, y, s, c, style);
}

void textRight(App& app, float right, float y, const char* s, gd::Color c, gd::Hud::Style style = gd::Hud::Style::Regular) {
    app.hud.text(app.renderer, right - app.hud.width(s, style), y, s, c, style);
}

void drawVillager(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::card(r, 24.0F, kPanelY, 252.0F, kPanelH, gd::kWhite, gd::kInk);
    text(app, 36.0F, 50.0F, "VILLAGER", gd::kTealDark, gd::Hud::Style::Bold);
    const ua::State& s = app.sim.state;
    const double values[5] = {s.hunger, s.fatigue, s.drive, s.larder, ua::daylight(s.clock)};
    const char* names[5] = {"Hunger", "Fatigue", "Drive", "Larder", "Daylight"};
    for (int i = 0; i < 5; ++i) {
        const float y = barY(i);
        const bool picked = i < 3 && i == app.curveTarget;
        if (picked) gd::fillRect(r, 28.0F, y - 1.0F, 4.0F, kBarH + 2.0F, gd::kTeal);
        text(app, 36.0F, y - 4.0F, names[i], picked ? gd::kTealDark : gd::kInk);
        gd::fillRect(r, kBarX, y, kBarW, kBarH, gd::kGrid);
        gd::fillRect(r, kBarX, y, kBarW * static_cast<float>(ua::clamp01(values[i])), kBarH, i == 4 ? gd::kMuted : gd::kOrange);
        gd::strokeRect(r, kBarX, y, kBarW, kBarH, gd::kInk);
    }
}

void drawCurve(App& app) {
    SDL_Renderer* r = app.renderer;
    const ua::Curve curve = curveRef(app.sim.settings, app.curveTarget);
    const double input = needRef(app.sim.state, app.curveTarget);
    gd::card(r, 292.0F, kPanelY, 196.0F, kPanelH, gd::kWhite, gd::kInk);
    char head[32];
    SDL_snprintf(head, sizeof head, "CURVE: %s", kNeedNames[app.curveTarget]);
    text(app, 304.0F, 50.0F, head, gd::kTealDark, gd::Hud::Style::Bold);

    gd::fillRect(r, kPlotX, kPlotY, kPlotW, kPlotH, gd::kPaper);
    gd::setColor(r, gd::kGrid);
    SDL_RenderLine(r, kPlotX + kPlotW / 2.0F, kPlotY, kPlotX + kPlotW / 2.0F, kPlotY + kPlotH);
    SDL_RenderLine(r, kPlotX, kPlotY + kPlotH / 2.0F, kPlotX + kPlotW, kPlotY + kPlotH / 2.0F);
    gd::setColor(r, gd::Color{107, 114, 128, 140});
    SDL_RenderLine(r, kPlotX, kPlotY + kPlotH, kPlotX + kPlotW, kPlotY);  // the straight line, for comparison
    constexpr int kSegments = 64;
    std::array<SDL_FPoint, kSegments + 1> pts{};
    for (int i = 0; i <= kSegments; ++i) {
        const double x = static_cast<double>(i) / kSegments;
        pts[static_cast<std::size_t>(i)] = {kPlotX + static_cast<float>(x) * kPlotW, kPlotY + kPlotH - static_cast<float>(ua::respond(curve, x)) * kPlotH};
    }
    gd::setColor(r, gd::kTealDark);
    for (int o = 0; o < 2; ++o) {
        std::array<SDL_FPoint, kSegments + 1> shifted = pts;
        for (SDL_FPoint& p : shifted) p.y += static_cast<float>(o);
        SDL_RenderLines(r, shifted.data(), kSegments + 1);
    }
    gd::strokeRect(r, kPlotX, kPlotY, kPlotW, kPlotH, gd::kInk);

    // Where the current need lands on the curve.
    const double out = ua::respond(curve, input);
    const float mx = kPlotX + static_cast<float>(ua::clamp01(input)) * kPlotW;
    const float my = kPlotY + kPlotH - static_cast<float>(out) * kPlotH;
    gd::setColor(r, kInkSoft);
    for (float y = my; y < kPlotY + kPlotH; y += 6.0F) SDL_RenderLine(r, mx, y, mx, std::min(y + 3.0F, kPlotY + kPlotH));
    for (float x = kPlotX; x < mx; x += 6.0F) SDL_RenderLine(r, x, my, std::min(x + 3.0F, mx), my);
    gd::disc(r, mx, my, 5.0F, gd::kOrange);
    gd::ring(r, mx, my, 5.0F, gd::kInk);

    if (curve.type == ua::CurveType::Logistic) {
        const double mid = std::clamp(curve.mid, 0.05, 0.95);
        const float hx = kPlotX + static_cast<float>(mid) * kPlotW;
        const float hy = kPlotY + kPlotH - static_cast<float>(ua::respond(curve, mid)) * kPlotH;
        gd::fillRect(r, hx - 5.0F, hy - 5.0F, 10.0F, 10.0F, app.drag == Drag::Mid ? gd::kTeal : gd::kWhite);
        gd::strokeRect(r, hx - 5.0F, hy - 5.0F, 10.0F, 10.0F, gd::kInk);
    }

    char line[48];
    SDL_snprintf(line, sizeof line, "in %.2f  out %.2f", input, out);
    text(app, 304.0F, 147.0F, line, gd::kInk);
    const char* kind = curve.type == ua::CurveType::Linear ? "Linear" : curve.type == ua::CurveType::Quadratic ? "Quadratic" : "Logistic (drag)";
    text(app, 304.0F, 166.0F, kind, gd::kMuted);
}

void drawCards(App& app, const ua::Evaluation& e, ua::Action pick) {
    SDL_Renderer* r = app.renderer;
    for (int i = 0; i < ua::kActionCount; ++i) {
        const auto a = static_cast<ua::Action>(i);
        const ua::Option& o = e[static_cast<std::size_t>(i)];
        const float x = kCardX + static_cast<float>(i) * (kCardW + kCardGap);
        const bool chosen = a == pick;
        gd::card(r, x, kPanelY, kCardW, kPanelH, chosen ? gd::kTealSoft : gd::kWhite, chosen ? gd::kTealDark : gd::kInk);
        if (chosen) gd::strokeRect(r, x + 1.0F, kPanelY + 1.0F, kCardW - 2.0F, kPanelH - 2.0F, gd::kTealDark);
        std::string name = kActionNames[i];
        for (char& ch : name) ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
        text(app, x + 8.0F, kPanelY + 6.0F, name.c_str(), colorOf(a), gd::Hud::Style::Bold);
        if (o.band == 1) {
            gd::fillRect(r, x + kCardW - 24.0F, kPanelY + 8.0F, 16.0F, 18.0F, gd::kOrange);
            text(app, x + kCardW - 21.0F, kPanelY + 6.0F, "B", gd::kWhite, gd::Hud::Style::Bold);
        }
        for (int k = 0; k < o.factors; ++k) {
            const float y0 = kPanelY + 30.0F + 36.0F * static_cast<float>(k);
            text(app, x + 8.0F, y0, kFactorNames[i][k], gd::kMuted);
            gd::fillRect(r, x + 8.0F, y0 + 22.0F, kCardW - 16.0F, 8.0F, gd::kGrid);
            gd::fillRect(r, x + 8.0F, y0 + 22.0F, (kCardW - 16.0F) * static_cast<float>(ua::clamp01(o.factor[static_cast<std::size_t>(k)])), 8.0F, colorOf(a));
            gd::strokeRect(r, x + 8.0F, y0 + 22.0F, kCardW - 16.0F, 8.0F, gd::kInk);
        }
        char score[16];
        SDL_snprintf(score, sizeof score, "%.2f", o.score);
        text(app, x + 8.0F, kPanelY + 100.0F, score, gd::kInk, gd::Hud::Style::Large);
    }
}

void drawTimeline(App& app) {
    SDL_Renderer* r = app.renderer;
    const ua::History& h = app.sim.history;
    const std::size_t n = h.size();
    constexpr float kSample = kLineW / static_cast<float>(ua::kHistory);
    const float first = static_cast<float>(ua::kHistory - n);
    const auto xOf = [&](std::size_t i) { return kLineX + (first + static_cast<float>(i)) * kSample; };
    const auto yOf = [](double v) { return kLineY + kLineH - 8.0F - static_cast<float>(ua::clamp01(v)) * (kLineH - 16.0F); };

    gd::card(r, kLineX, kLineY, kLineW, kLineH, gd::kWhite, gd::kInk);
    // Night is a grey band behind the lines.
    for (std::size_t i = 0; i < n;) {
        if (h.at(i).daylight >= 0.5F) {
            ++i;
            continue;
        }
        std::size_t j = i;
        while (j < n && h.at(j).daylight < 0.5F) ++j;
        gd::fillRect(r, xOf(i), kLineY + 1.0F, static_cast<float>(j - i) * kSample, kLineH - 2.0F, kNight);
        i = j;
    }
    gd::setColor(r, gd::kGrid);
    for (const double v : {0.25, 0.5, 0.75, 1.0}) SDL_RenderLine(r, kLineX + 1.0F, yOf(v), kLineX + kLineW - 1.0F, yOf(v));
    text(app, kLineX + 6.0F, yOf(1.0) + 2.0F, "1", gd::kMuted);
    text(app, kLineX + 6.0F, yOf(0.0) - 22.0F, "0", gd::kMuted);

    if (n >= 2) {
        for (int a = 0; a < ua::kActionCount; ++a) {
            for (std::size_t i = 0; i < n; ++i) {
                app.points[i] = {xOf(i), yOf(static_cast<double>(h.at(i).score[static_cast<std::size_t>(a)]))};
            }
            gd::setColor(r, colorOf(static_cast<ua::Action>(a)));
            for (int o = 0; o < 2; ++o) {
                if (o == 1) {
                    for (std::size_t i = 0; i < n; ++i) app.points[i].y += 1.0F;
                }
                SDL_RenderLines(r, app.points.data(), static_cast<int>(n));
            }
        }
    }

    // The chosen action, one coloured run after the other.
    gd::fillRect(r, kLineX, kStripY, kLineW, kStripH, gd::kGrid);
    for (std::size_t i = 0; i < n;) {
        std::size_t j = i;
        while (j < n && h.at(j).chosen == h.at(i).chosen) ++j;
        gd::fillRect(r, xOf(i), kStripY, static_cast<float>(j - i) * kSample, kStripH, colorOf(h.at(i).chosen));
        i = j;
    }
    gd::strokeRect(r, kLineX, kStripY, kLineW, kStripH, gd::kInk);

    // Legend between the two time labels.
    text(app, kLineX, 372.0F, "-60 s", gd::kMuted);
    textRight(app, kLineX + kLineW, 372.0F, "now", gd::kMuted);
    struct Item {
        const char* label;
        gd::Color color;
    };
    const std::array<Item, 5> items{{{"Eat", colorOf(ua::Action::Eat)}, {"Sleep", colorOf(ua::Action::Sleep)}, {"Work", colorOf(ua::Action::Work)}, {"Wander", colorOf(ua::Action::Wander)}, {"night", gd::Color{17, 24, 39, 60}}}};
    float total = 0.0F;
    for (const Item& it : items) total += app.hud.width(it.label) + 16.0F + 16.0F;
    float x = kLineX + (kLineW - total) / 2.0F;
    for (const Item& it : items) {
        gd::fillRect(r, x, 378.0F, 10.0F, 10.0F, it.color);
        text(app, x + 16.0F, 372.0F, it.label, gd::kInk);
        x += app.hud.width(it.label) + 32.0F;
    }
}

void drawNow(App& app) {
    SDL_Renderer* r = app.renderer;
    constexpr float kX = 760.0F;
    constexpr float kW = 176.0F;
    gd::card(r, kX, kLineY, kW, 170.0F, gd::kWhite, gd::kInk);
    text(app, kX + 12.0F, kLineY + 8.0F, "RIGHT NOW", gd::kTealDark, gd::Hud::Style::Bold);
    const ua::State& s = app.sim.state;
    int row = 0;
    for (const ua::Combiner c : {ua::Combiner::Mean, ua::Combiner::Product, ua::Combiner::Bands}) {
        ua::Settings probe = app.sim.settings;
        probe.combiner = c;
        const ua::Action a = ua::choose(ua::evaluate(s, probe), app.sim.current, probe.inertia);
        const float y = kLineY + 32.0F + 22.0F * static_cast<float>(row);
        const bool mine = c == app.sim.settings.combiner;
        if (mine) gd::fillRect(r, kX + 8.0F, y, kW - 16.0F, 20.0F, gd::kTeal);
        const char* label = c == ua::Combiner::Mean ? "Mean" : c == ua::Combiner::Product ? "Product" : "Bands";
        text(app, kX + 14.0F, y - 1.0F, label, mine ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
        textRight(app, kX + kW - 14.0F, y - 1.0F, kActionNames[ua::index(a)], mine ? gd::kWhite : colorOf(a), gd::Hud::Style::Bold);
        ++row;
    }
    text(app, kX + 12.0F, kLineY + 104.0F, "switches / 60 s", gd::kMuted);
    char count[16];
    SDL_snprintf(count, sizeof count, "%d", app.sim.history.switches());
    text(app, kX + 12.0F, kLineY + 122.0F, count, gd::kInk, gd::Hud::Style::Large);
}

void drawChips(App& app) {
    SDL_Renderer* r = app.renderer;
    text(app, 24.0F, kChipRows[0] + 4.0F, "Combine", gd::kMuted);
    text(app, 24.0F, kChipRows[1] + 4.0F, "Curve of", gd::kMuted);
    text(app, 24.0F, kChipRows[2] + 4.0F, "Scenario", gd::kMuted);
    for (const Chip& c : app.chips) {
        const bool on = active(app, c.act);
        const std::string label = chipLabel(app, c);
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? gd::kTeal : gd::kWhite, gd::kInk);
        text(app, c.rect.x + (c.rect.w - app.hud.width(label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 4.0F, label.c_str(), on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
    }
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    const ua::Evaluation e = ua::evaluate(app.sim.state, app.sim.settings);
    const ua::Action pick = ua::choose(e, app.sim.current, app.sim.settings.inertia);

    gd::backdrop(r, kWidth, kHeight);
    std::string right = combinerName(app.sim.settings.combiner);
    if (app.sim.settings.inertia > 0.0) right += ", inertia";
    if (app.paused) right += ", paused";
    app.hud.titleBar(r, kWidth, "[ UTILITY AI COMBINERS ]", right);

    drawVillager(app);
    drawCurve(app);
    drawCards(app, e, pick);
    drawTimeline(app);
    drawNow(app);
    drawChips(app);
    text(app, 24.0F, 506.0F, "drag a bar: set a need   B: urgent band   grey: night   chosen action = teal card", gd::kMuted);
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
    if (!SDL_CreateWindowAndRenderer("utility-combiners", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
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
    reset(*app);
    app->lastTicks = SDL_GetTicksNS();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION:
            if (app.drag == Drag::Need) setNeedFromX(app, app.dragNeed, event->motion.x);
            if (app.drag == Drag::Mid) setMidFromX(app, event->motion.x);
            break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const float x = event->button.x;
            const float y = event->button.y;
            for (const Chip& c : app.chips) {
                if (inRect(c.rect, x, y)) {
                    apply(app, c.act);
                    return SDL_APP_CONTINUE;
                }
            }
            for (int i = 0; i < 4; ++i) {
                if (inRect(SDL_FRect{kBarX, barY(i) - 3.0F, kBarW, kBarH + 6.0F}, x, y)) {
                    app.drag = Drag::Need;
                    app.dragNeed = i;
                    setNeedFromX(app, i, x);
                    return SDL_APP_CONTINUE;
                }
                if (i < 3 && inRect(SDL_FRect{30.0F, barY(i) - 3.0F, kBarX - 34.0F, kBarH + 6.0F}, x, y)) {
                    app.curveTarget = i;  // a click on the name picks the curve to edit
                    return SDL_APP_CONTINUE;
                }
            }
            if (curveRef(app.sim.settings, app.curveTarget).type == ua::CurveType::Logistic &&
                inRect(SDL_FRect{kPlotX - 8.0F, kPlotY - 8.0F, kPlotW + 16.0F, kPlotH + 16.0F}, x, y)) {
                app.drag = Drag::Mid;
                setMidFromX(app, x);
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event->button.button == SDL_BUTTON_LEFT) app.drag = Drag::None;
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
        case SDL_EVENT_WINDOW_FOCUS_LOST: app.drag = Drag::None; break;
        case SDL_EVENT_KEY_DOWN:
            if (event->key.repeat) break;  // every key toggles or triggers, none should fire while held
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: apply(app, Act::Mean); break;
                case SDLK_2: apply(app, Act::Product); break;
                case SDLK_3: apply(app, Act::Bands); break;
                case SDLK_I: apply(app, Act::Inertia); break;
                case SDLK_SPACE: apply(app, Act::Pause); break;
                case SDLK_R: apply(app, Act::Reset); break;
                case SDLK_H: apply(app, Act::CurveHunger); break;
                case SDLK_F: apply(app, Act::CurveFatigue); break;
                case SDLK_D: apply(app, Act::CurveDrive); break;
                case SDLK_4: apply(app, Act::Linear); break;
                case SDLK_5: apply(app, Act::Quadratic); break;
                case SDLK_6: apply(app, Act::Logistic); break;
                case SDLK_7: apply(app, Act::ScenarioNight); break;
                case SDLK_8: apply(app, Act::ScenarioEager); break;
                default: break;
            }
            break;
        default: break;
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void* appstate) {
    auto& app = *static_cast<App*>(appstate);
    const Uint64 now = SDL_GetTicksNS();
    const double dt = static_cast<double>(now - app.lastTicks) / 1.0e9;
    app.lastTicks = now;
    if (!app.paused) app.sim.update(dt);  // while paused the clock still runs, so resuming never bursts
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
