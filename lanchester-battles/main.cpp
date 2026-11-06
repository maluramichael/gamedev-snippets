// Lanchester battles with SDL3 main callbacks.
//
// Two armies of dots fight on the left, the differential equations of Lanchester's two laws run as dashed curves on the
// right. The dots know nothing about the laws: they have a weapon, a range, a way to pick a target and legs. Ranged
// weapons (everybody can hit everybody) follow the square law, one-on-one duels follow the linear law, and the first
// frame already shows how close the simulation gets. The chips then break the assumptions one by one (short range,
// everybody aiming at the nearest enemy, a split army) so you see where the formula stops being true and why.
// The math and the simulation live in logic.hpp (no SDL) and are covered by test.cpp.

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdarg>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "hud.hpp"
#include "logic.hpp"

namespace {

constexpr int kWidth = 960;
constexpr int kHeight = 540;
constexpr float kFieldX = 24.0F;
constexpr float kFieldY = 40.0F;
constexpr float kPlotX = 600.0F;
constexpr float kPlotY = 40.0F;
constexpr float kPlotW = 336.0F;
constexpr float kPlotH = 190.0F;
constexpr float kResultY = 238.0F;
constexpr float kResultH = 150.0F;
constexpr float kChipH = 32.0F;
constexpr std::array<float, 3> kChipRowY{398.0F, 436.0F, 474.0F};
constexpr double kStep = 1.0 / 60.0;
constexpr int kMaxStepsPerFrame = 60;
constexpr float kUnitRadius = 3.6F;
constexpr gd::Color kInkSoft{17, 24, 39, 190};
constexpr gd::Color kTealLine{15, 118, 110, 235};
constexpr gd::Color kOrangeLine{194, 65, 12, 235};

enum class Action {
    LawSquare,
    LawLinear,
    Army100v70,
    Army100v50,
    ArmySplit,
    Power1,
    Power2,
    Power4,
    Range,
    Aim,
    Speed,
    Pause,
    Restart,
};

struct Chip {
    Action action;
    SDL_FRect rect;
    std::string label;
};

struct RunStats {
    int runs = 0;
    int aWins = 0;
    double sumLeft = 0.0;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;

    // Settings behind the chips.
    lb::Law law = lb::Law::Square;
    lb::Aim aim = lb::Aim::Claim;
    bool shortRange = false;
    int army = 0;       // 0 = 100 v 70, 1 = 100 v 50, 2 = split 50+50 v 100
    int powerIndex = 0;  // B strength 1, 2, 4
    int speedIndex = 0;  // x1, x2, x4
    bool paused = false;

    lb::Params params;
    lb::Battle battle{lb::Params{}};
    lb::Outcome predicted;
    std::vector<lb::Sample> predictedCurve;
    RunStats stats;
    bool counted = false;
    // The first battle uses seed 3, whose result is close to the average (71 left, the law says 71.4), so the first
    // impression is not a lucky outlier. Every restart takes the next seed and shows the spread.
    int seedCounter = 3;
    int bAtRelease = -1;  // B alive when the reserve moved in
    double accumulator = 0.0;
    Uint64 lastTicks = 0;

    std::vector<SDL_Vertex> verts;  // reused every frame, so drawing the units allocates nothing
    std::vector<int> indices;

    long maxFrames = -1;
    long frames = 0;
};

constexpr std::array<double, 3> kPowers{1.0, 2.0, 4.0};
constexpr std::array<double, 3> kSpeeds{1.0, 2.0, 4.0};

std::string fmt(const char* format, ...) {
    char buffer[96];
    va_list args;
    va_start(args, format);
    SDL_vsnprintf(buffer, sizeof buffer, format, args);
    va_end(args);
    return buffer;
}

bool isDuels(const App& app) { return app.law == lb::Law::Linear; }

const char* aimName(lb::Aim aim) { return aim == lb::Aim::Claim ? "coordinated" : aim == lb::Aim::Nearest ? "nearest" : "random"; }

std::string chipLabel(const App& app, Action action) {
    switch (action) {
        case Action::LawSquare: return "1 Ranged = square law";
        case Action::LawLinear: return "2 Duels = linear law";
        case Action::Army100v70: return "3 100 vs 70";
        case Action::Army100v50: return "4 100 vs 50";
        case Action::ArmySplit: return "5 split 50+50 vs 100";
        case Action::Power1: return "6 ×1";
        case Action::Power2: return "7 ×2";
        case Action::Power4: return "8 ×4";
        case Action::Range: return isDuels(app) ? "W Range: melee" : app.shortRange ? "W Range: short" : "W Range: far";
        case Action::Aim: return isDuels(app) ? "A Aim: partner" : fmt("A Aim: %s", aimName(app.aim));
        case Action::Speed: return fmt("S Speed ×%d", static_cast<int>(kSpeeds[static_cast<std::size_t>(app.speedIndex)]));
        case Action::Pause: return app.paused ? "Space Resume" : "Space Pause";
        case Action::Restart: return "R Restart";
    }
    return {};
}

/// Widest text a chip can show, so the layout does not shift when a label changes.
const char* widestLabel(Action action) {
    switch (action) {
        case Action::Range: return "W Range: melee";
        case Action::Aim: return "A Aim: coordinated";
        case Action::Pause: return "Space Resume";
        default: return nullptr;
    }
}

void refreshLabels(App& app) {
    for (Chip& c : app.chips) c.label = chipLabel(app, c.action);
}

void layoutChips(App& app) {
    struct Def {
        Action action;
        int row;
        float gapBefore;
    };
    const std::array<Def, 13> defs{{{Action::LawSquare, 0, 0.0F},
                                    {Action::LawLinear, 0, 8.0F},
                                    {Action::Power1, 0, 134.0F},
                                    {Action::Power2, 0, 8.0F},
                                    {Action::Power4, 0, 8.0F},
                                    {Action::Army100v70, 1, 0.0F},
                                    {Action::Army100v50, 1, 8.0F},
                                    {Action::ArmySplit, 1, 8.0F},
                                    {Action::Range, 2, 0.0F},
                                    {Action::Aim, 2, 8.0F},
                                    {Action::Speed, 2, 24.0F},
                                    {Action::Pause, 2, 8.0F},
                                    {Action::Restart, 2, 8.0F}}};
    std::array<float, 3> x{kFieldX, kFieldX, kFieldX};
    for (const Def& d : defs) {
        const auto row = static_cast<std::size_t>(d.row);
        x[row] += d.gapBefore;
        const std::string label = chipLabel(app, d.action);
        const char* widest = widestLabel(d.action);
        const float w = app.hud.width(widest != nullptr ? widest : label, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({d.action, SDL_FRect{x[row], kChipRowY[row], w, kChipH}, label});
        x[row] += w;
    }
}

lb::Params paramsOf(const App& app) {
    lb::Params p;
    p.law = app.law;
    p.aim = app.aim;
    p.range = (app.law == lb::Law::Square && app.shortRange) ? lb::kRangeShort : lb::kRangeFar;
    p.a1 = app.army == 2 ? 50 : 100;
    p.a2 = app.army == 2 ? 50 : 0;
    p.b = app.army == 0 ? 70 : app.army == 1 ? 50 : 100;
    p.power = kPowers[static_cast<std::size_t>(app.powerIndex)];
    return p;
}

bool sameSetup(const lb::Params& a, const lb::Params& b) {
    return a.law == b.law && a.aim == b.aim && a.range == b.range && a.a1 == b.a1 && a.a2 == b.a2 && a.b == b.b && a.power == b.power;
}

/// Fresh battle from the current settings. The run statistics only start over when the settings changed.
void startBattle(App& app) {
    lb::Params p = paramsOf(app);
    if (!sameSetup(p, app.params)) app.stats = RunStats{};
    p.seed = static_cast<std::uint64_t>(app.seedCounter++);
    app.params = p;
    app.battle = lb::Battle(p);
    const lb::Setup setup = lb::setupOf(p);
    app.predicted = lb::predict(p.law, setup);
    app.predictedCurve = lb::curve(p.law, setup, 0.1, 150.0);
    if (app.predicted.winner == lb::Winner::Draw) {
        // In a tie both curves fade out for a very long time: stop drawing them once both are nearly gone.
        const double nearly = 0.04 * std::max(setup.a1 + setup.a2, setup.b);
        const auto cut = std::find_if(app.predictedCurve.begin(), app.predictedCurve.end(), [nearly](const lb::Sample& s) { return s.a < nearly && s.b < nearly; });
        if (cut != app.predictedCurve.end()) app.predictedCurve.erase(cut + 1, app.predictedCurve.end());
    }
    app.counted = false;
    app.bAtRelease = -1;
    app.accumulator = 0.0;  // no leftover time from the last battle
}

void apply(App& app, Action action) {
    switch (action) {
        case Action::LawSquare: app.law = lb::Law::Square; break;
        case Action::LawLinear: app.law = lb::Law::Linear; break;
        case Action::Army100v70: app.army = 0; break;
        case Action::Army100v50: app.army = 1; break;
        case Action::ArmySplit: app.army = 2; break;
        case Action::Power1: app.powerIndex = 0; break;
        case Action::Power2: app.powerIndex = 1; break;
        case Action::Power4: app.powerIndex = 2; break;
        case Action::Range:
            if (isDuels(app)) return;
            app.shortRange = !app.shortRange;
            break;
        case Action::Aim:
            if (isDuels(app)) return;
            app.aim = app.aim == lb::Aim::Claim ? lb::Aim::Nearest : app.aim == lb::Aim::Nearest ? lb::Aim::Random : lb::Aim::Claim;
            break;
        case Action::Speed:
            app.speedIndex = (app.speedIndex + 1) % static_cast<int>(kSpeeds.size());
            refreshLabels(app);
            return;
        case Action::Pause:
            app.paused = !app.paused;
            refreshLabels(app);
            return;
        case Action::Restart: break;
    }
    refreshLabels(app);
    startBattle(app);
}

bool active(const App& app, Action action) {
    switch (action) {
        case Action::LawSquare: return app.law == lb::Law::Square;
        case Action::LawLinear: return app.law == lb::Law::Linear;
        case Action::Army100v70: return app.army == 0;
        case Action::Army100v50: return app.army == 1;
        case Action::ArmySplit: return app.army == 2;
        case Action::Power1: return app.powerIndex == 0;
        case Action::Power2: return app.powerIndex == 1;
        case Action::Power4: return app.powerIndex == 2;
        case Action::Speed: return app.speedIndex > 0;
        case Action::Pause: return app.paused;
        case Action::Range:
        case Action::Aim:
        case Action::Restart: return false;
    }
    return false;
}

/// Range and aim are the "naive" choices when they differ from what the law assumes, so they turn orange.
bool naive(const App& app, Action action) {
    if (isDuels(app)) return false;
    return (action == Action::Range && app.shortRange) || (action == Action::Aim && app.aim != lb::Aim::Claim);
}

bool disabled(const App& app, Action action) { return isDuels(app) && (action == Action::Range || action == Action::Aim); }

// ---- drawing ----

/// 1 px lines are thin on a 960x540 canvas that is scaled up, so lines get a second pixel next to them.
void thickLine(SDL_Renderer* r, float x0, float y0, float x1, float y1, gd::Color c, bool thick = true) {
    gd::setColor(r, c);
    SDL_RenderLine(r, x0, y0, x1, y1);
    if (thick) {
        const bool flat = std::fabs(x1 - x0) > std::fabs(y1 - y0);
        SDL_RenderLine(r, x0 + (flat ? 0.0F : 1.0F), y0 + (flat ? 1.0F : 0.0F), x1 + (flat ? 0.0F : 1.0F), y1 + (flat ? 1.0F : 0.0F));
    }
}

/// All units in one geometry call: a small fan per unit, brighter in the middle.
void appendDisc(App& app, float cx, float cy, float radius, gd::Color center, gd::Color edge) {
    constexpr int kSegments = 10;
    const int base = static_cast<int>(app.verts.size());
    app.verts.push_back({{cx, cy}, gd::fcolor(center), {0, 0}});
    for (int i = 0; i <= kSegments; ++i) {
        const float a = 6.2831853F * static_cast<float>(i) / static_cast<float>(kSegments);
        app.verts.push_back({{cx + std::cos(a) * radius, cy + std::sin(a) * radius}, gd::fcolor(edge), {0, 0}});
        if (i > 0) {
            app.indices.insert(app.indices.end(), {base, base + i, base + i + 1});
        }
    }
}

std::string winnerText(lb::Outcome o) {
    if (o.winner == lb::Winner::Draw) return "tie, nobody left";
    return fmt("%s wins · %d left", o.winner == lb::Winner::A ? "A" : "B", static_cast<int>(std::lround(o.left)));
}

gd::Color winnerColor(lb::Outcome o) { return o.winner == lb::Winner::A ? gd::kTeal : o.winner == lb::Winner::B ? gd::kOrange : gd::kMuted; }

void drawField(App& app) {
    SDL_Renderer* r = app.renderer;
    const lb::Battle& battle = app.battle;
    gd::card(r, kFieldX, kFieldY, lb::kFieldW, lb::kFieldH, gd::kWhite, gd::kInk);

    // Where units fell: a faint square in their color.
    for (const lb::Unit& u : battle.units()) {
        if (u.alive) continue;
        gd::fillRect(r, kFieldX + u.pos.x - 1.5F, kFieldY + u.pos.y - 1.5F, 3.0F, 3.0F, u.side == 0 ? gd::Color{13, 148, 136, 70} : gd::Color{234, 88, 12, 70});
    }

    // Duel pairs.
    if (isDuels(app)) {
        for (std::size_t i = 0; i < battle.units().size(); ++i) {
            const lb::Unit& u = battle.units()[i];
            if (!u.alive || u.side != 0 || u.partner < 0 || !u.contact) continue;
            const lb::Unit& p = battle.units()[static_cast<std::size_t>(u.partner)];
            thickLine(r, kFieldX + u.pos.x, kFieldY + u.pos.y, kFieldX + p.pos.x, kFieldY + p.pos.y, kInkSoft, false);
        }
    }

    // Shots in the air: a short streak moving from the shooter toward its target.
    for (const lb::Shot& s : battle.shots()) {
        const lb::Unit& from = battle.units()[static_cast<std::size_t>(s.from)];
        const lb::Unit& to = battle.units()[static_cast<std::size_t>(s.to)];
        const float p = static_cast<float>(1.0 - s.left / lb::kFlight);
        const float tail = std::max(0.0F, p - 0.15F);
        const gd::Color c = from.side == 0 ? gd::Color{13, 148, 136, 150} : gd::Color{234, 88, 12, 150};
        gd::setColor(r, c);
        SDL_RenderLine(r, kFieldX + s.origin.x + (to.pos.x - s.origin.x) * tail, kFieldY + s.origin.y + (to.pos.y - s.origin.y) * tail,
                       kFieldX + s.origin.x + (to.pos.x - s.origin.x) * p, kFieldY + s.origin.y + (to.pos.y - s.origin.y) * p);
    }

    // Living units, one geometry call.
    app.verts.clear();
    app.indices.clear();
    for (const lb::Unit& u : battle.units()) {
        if (!u.alive || !u.active) continue;
        if (u.side == 0) {
            appendDisc(app, kFieldX + u.pos.x, kFieldY + u.pos.y, kUnitRadius, gd::Color{45, 190, 175}, gd::kTealDark);
        } else {
            appendDisc(app, kFieldX + u.pos.x, kFieldY + u.pos.y, kUnitRadius, gd::Color{251, 146, 60}, gd::Color{194, 65, 12});
        }
    }
    if (!app.verts.empty()) {
        SDL_RenderGeometry(r, nullptr, app.verts.data(), static_cast<int>(app.verts.size()), app.indices.data(), static_cast<int>(app.indices.size()));
    }
    // The reserve waits as rings until the first wave is dead.
    for (const lb::Unit& u : battle.units()) {
        if (u.alive && !u.active) gd::ring(r, kFieldX + u.pos.x, kFieldY + u.pos.y, kUnitRadius, gd::kTealDark, 12);
    }

    // Counters, on a white patch so a line of units running under them does not hide the numbers.
    gd::fillRect(r, kFieldX + 4.0F, kFieldY + 4.0F, 84.0F, 28.0F, gd::Color{255, 255, 255, 215});
    gd::fillRect(r, kFieldX + lb::kFieldW - 88.0F, kFieldY + 4.0F, 84.0F, 28.0F, gd::Color{255, 255, 255, 215});
    gd::fillRect(r, kFieldX + lb::kFieldW / 2.0F - 56.0F, kFieldY + 4.0F, 112.0F, 28.0F, gd::Color{255, 255, 255, 215});
    char text[48];
    SDL_snprintf(text, sizeof text, "A %d", battle.alive(0));
    app.hud.text(r, kFieldX + 10.0F, kFieldY + 8.0F, text, gd::kTeal, gd::Hud::Style::Bold);
    SDL_snprintf(text, sizeof text, "B %d", battle.alive(1));
    app.hud.text(r, kFieldX + lb::kFieldW - 10.0F - app.hud.width(text, gd::Hud::Style::Bold), kFieldY + 8.0F, text, gd::kOrange, gd::Hud::Style::Bold);
    SDL_snprintf(text, sizeof text, "%.1f s", battle.time());
    app.hud.text(r, kFieldX + (lb::kFieldW - app.hud.width(text, gd::Hud::Style::Bold)) / 2.0F, kFieldY + 8.0F, text, gd::kMuted, gd::Hud::Style::Bold);
    if (battle.finished()) {
        const char* done = "done, R fights again";
        app.hud.text(r, kFieldX + (lb::kFieldW - app.hud.width(done)) / 2.0F, kFieldY + lb::kFieldH - 26.0F, done, gd::kMuted);
    } else if (app.paused) {
        const char* msg = "paused";
        app.hud.text(r, kFieldX + (lb::kFieldW - app.hud.width(msg, gd::Hud::Style::Large)) / 2.0F, kFieldY + lb::kFieldH - 56.0F, msg, gd::kMuted, gd::Hud::Style::Large);
    }
}

/// The two lines below the field: what the rule is, and why the simulation does or does not follow the law.
void drawNotes(App& app) {
    SDL_Renderer* r = app.renderer;
    const lb::Battle& battle = app.battle;
    const float y = kFieldY + lb::kFieldH + 8.0F;
    app.hud.text(r, kFieldX, y, isDuels(app) ? "duels: one on one, the rest waits behind" : "ranged: everyone can shoot everyone", gd::kMuted);

    std::string why;
    gd::Color color = gd::kTeal;
    if (isDuels(app)) {
        const int a = battle.alive(0);
        const int b = battle.alive(1);
        const int waiting = std::abs(a - b);
        if (battle.time() < 2.0 && battle.engaged(0) < 0.01 && battle.engaged(1) < 0.01) {
            why = "first everybody walks to an opponent";
            color = gd::kOrange;
        } else {
            why = waiting > 0 ? fmt("%d of %s wait, one duel per unit", waiting, a > b ? "A" : "B") : "every unit has an opponent";
        }
    } else if (app.params.aim == lb::Aim::Nearest && battle.shotsFired() > 20) {
        why = fmt("everyone aims at the front: %d%% of shots hit corpses", static_cast<int>(std::lround(battle.wasted() * 100.0)));
        color = gd::kOrange;
    } else if (app.params.range < lb::kRangeFar) {
        // Walking into range takes a few seconds, the share is only worth showing after that.
        why = battle.time() < 4.0 ? "short range: first everybody walks into range"
                                  : fmt("short range: only %d%% of A could shoot", static_cast<int>(std::lround(battle.engaged(0) * 100.0)));
        color = gd::kOrange;
    } else if (app.params.aim == lb::Aim::Random && battle.shotsFired() > 20) {
        why = fmt("random aim: %d%% of shots hit corpses", static_cast<int>(std::lround(battle.wasted() * 100.0)));
        color = gd::kOrange;
    } else {
        why = "every shot has a target: the law fits";
    }
    app.hud.text(r, kFieldX, y + 22.0F, why, color, gd::Hud::Style::Bold);
}

void niceAxis(double maxValue, double& step) {
    static constexpr std::array<double, 7> kSteps{2.0, 5.0, 10.0, 20.0, 25.0, 50.0, 100.0};
    step = kSteps.back();
    for (double s : kSteps) {
        if (maxValue / s <= 6.0) {
            step = s;
            break;
        }
    }
}

void drawPlot(App& app) {
    SDL_Renderer* r = app.renderer;
    const lb::Battle& battle = app.battle;
    gd::card(r, kPlotX, kPlotY, kPlotW, kPlotH, gd::kWhite, gd::kInk);
    app.hud.text(r, kPlotX + 10.0F, kPlotY + 6.0F, "units alive", gd::kMuted);

    // Legend: dashed = law, solid = simulation.
    const float lx = kPlotX + kPlotW - 150.0F;
    for (float x = 0; x < 18.0F; x += 6.0F) thickLine(r, lx + x, kPlotY + 17.0F, lx + x + 3.0F, kPlotY + 17.0F, gd::kInk, false);
    app.hud.text(r, lx + 24.0F, kPlotY + 6.0F, "law", gd::kMuted);
    thickLine(r, lx + 68.0F, kPlotY + 17.0F, lx + 86.0F, kPlotY + 17.0F, gd::kInk);
    app.hud.text(r, lx + 92.0F, kPlotY + 6.0F, "sim", gd::kMuted);

    const float left = kPlotX + 40.0F;
    const float right = kPlotX + kPlotW - 14.0F;
    const float top = kPlotY + 30.0F;
    const float bottom = kPlotY + kPlotH - 24.0F;
    const double yMax = std::max<double>(app.params.a1 + app.params.a2, app.params.b) * 1.04;
    const double lawEnd = app.predictedCurve.empty() ? 1.0 : app.predictedCurve.back().t;
    const double tMax = std::max({lawEnd * 1.08, battle.time() * 1.04, 5.0});
    const auto px = [&](double t) { return left + static_cast<float>(t / tMax) * (right - left); };
    const auto py = [&](double v) { return bottom - static_cast<float>(v / yMax) * (bottom - top); };

    // Grid and labels.
    double xs = 10.0;
    niceAxis(tMax, xs);
    for (double t = 0.0; t <= tMax + 1e-9; t += xs) {
        gd::fillRect(r, px(t), top, 1.0F, bottom - top, gd::kGrid);
        const std::string label = fmt("%ds", static_cast<int>(std::lround(t)));
        const float labelX = std::min(px(t) - app.hud.width(label) / 2.0F, right - app.hud.width(label));
        app.hud.text(r, labelX, bottom + 4.0F, label, gd::kMuted);
    }
    for (int i = 0; i <= 2; ++i) {
        const double v = std::round(yMax * static_cast<double>(i) / 2.0 / 10.0) * 10.0;
        gd::fillRect(r, left, py(v), right - left, 1.0F, gd::kGrid);
        const std::string label = fmt("%d", static_cast<int>(v));
        app.hud.text(r, left - 6.0F - app.hud.width(label), py(v) - 11.0F, label, gd::kMuted);
    }
    gd::strokeRect(r, left, top, right - left, bottom - top, gd::kInk);

    // The law, dashed. The ring marks where the winner ends up.
    for (int side = 0; side < 2; ++side) {
        const gd::Color c = side == 0 ? kTealLine : kOrangeLine;
        float acc = 0.0F;
        float lastX = 0.0F;
        float lastY = 0.0F;
        bool first = true;
        for (const lb::Sample& s : app.predictedCurve) {
            const float x = px(s.t);
            const float y = py(side == 0 ? s.a : s.b);
            if (!first) {
                if (std::fmod(acc, 9.0F) < 5.0F) thickLine(r, lastX, lastY, x, y, c, false);
                acc += std::hypot(x - lastX, y - lastY);
            }
            first = false;
            lastX = x;
            lastY = y;
        }
    }
    if (app.predicted.winner != lb::Winner::Draw && !app.predictedCurve.empty()) {
        const lb::Sample& e = app.predictedCurve.back();
        const bool aWins = app.predicted.winner == lb::Winner::A;
        gd::ring(r, px(e.t), py(aWins ? e.a : e.b), 5.0F, aWins ? gd::kTealDark : kOrangeLine, 16);
    }

    // The simulation, solid.
    const std::vector<lb::Sample>& h = battle.history();
    for (std::size_t i = 1; i < h.size(); ++i) {
        thickLine(r, px(h[i - 1].t), py(h[i - 1].a), px(h[i].t), py(h[i].a), gd::kTeal);
        thickLine(r, px(h[i - 1].t), py(h[i - 1].b), px(h[i].t), py(h[i].b), gd::kOrange);
    }
}

int rounded(double v) { return static_cast<int>(std::lround(v)); }

void drawResult(App& app) {
    SDL_Renderer* r = app.renderer;
    const lb::Battle& battle = app.battle;
    gd::card(r, kPlotX, kResultY, kPlotW, kResultH, gd::kWhite, gd::kInk);
    const float x = kPlotX + 12.0F;
    float y = kResultY + 8.0F;
    constexpr float kLine = 22.0F;

    app.hud.text(r, x, y, "LAW", gd::kMuted, gd::Hud::Style::Bold);
    app.hud.text(r, x + 50.0F, y, winnerText(app.predicted), winnerColor(app.predicted), gd::Hud::Style::Bold);
    y += kLine;
    app.hud.text(r, x, y, "SIM", gd::kMuted, gd::Hud::Style::Bold);
    if (battle.finished()) {
        const lb::Outcome o = battle.outcome();
        app.hud.text(r, x + 50.0F, y, winnerText(o), winnerColor(o), gd::Hud::Style::Bold);
    } else {
        app.hud.text(r, x + 50.0F, y, "fighting ...", gd::kMuted, gd::Hud::Style::Bold);
    }
    y += kLine + 4.0F;

    // The conserved quantity, as numbers: start value and now.
    const lb::Setup setup = lb::setupOf(app.params);
    const double p = app.params.power;
    const bool square = app.params.law == lb::Law::Square;
    if (app.params.a2 > 0) {
        const lb::Outcome wave = lb::fight(app.params.law, setup.a1, setup.b, setup.alpha, setup.beta);
        const std::string law = wave.winner == lb::Winner::B ? fmt("wave 1 dies, B keeps %d (law)", rounded(wave.left)) : "wave 1 wins alone";
        app.hud.text(r, x, y, law, gd::kInk);
        y += kLine;
        const std::string now = app.bAtRelease >= 0 ? fmt("wave 2 meets %d B (sim)", app.bAtRelease) : "wave 2 waits in the back";
        app.hud.text(r, x, y, now, gd::kInk);
    } else {
        const double a0 = setup.a1;
        const double b0 = setup.b;
        const double a = battle.alive(0);
        const double b = battle.alive(1);
        const double start = square ? a0 * a0 - p * b0 * b0 : a0 - p * b0;
        const double nowValue = square ? a * a - p * b * b : a - p * b;
        const int strength = static_cast<int>(p);
        const std::string name = square ? (strength == 1 ? "A^2 - B^2" : fmt("A^2 - %dB^2", strength)) : (strength == 1 ? "A - B" : fmt("A - %dB", strength));
        app.hud.text(r, x, y, fmt("%s: %d, now %d", name.c_str(), rounded(start), rounded(nowValue)), gd::kInk);
        y += kLine;
        app.hud.text(r, x, y, "the law keeps this constant", gd::kMuted);
    }
    y += kLine;
    if (battle.time() > 0.5) {
        const std::string line = isDuels(app) ? fmt("at an opponent A %d%% B %d%%", rounded(battle.engaged(0) * 100.0), rounded(battle.engaged(1) * 100.0))
                                              : fmt("able to shoot A %d%% B %d%%", rounded(battle.engaged(0) * 100.0), rounded(battle.engaged(1) * 100.0));
        app.hud.text(r, x, y, line, gd::kMuted);
    }

    // Runs on these settings.
    if (app.stats.runs > 0) {
        const std::string runs = fmt("%d run%s, A won %d, mean left %d", app.stats.runs, app.stats.runs == 1 ? "" : "s", app.stats.aWins,
                                     rounded(app.stats.sumLeft / app.stats.runs));
        app.hud.text(r, x, kResultY + kResultH - 28.0F, runs, gd::kInk);
    }
}

void drawChips(App& app) {
    SDL_Renderer* r = app.renderer;
    for (const Chip& c : app.chips) {
        if (c.action == Action::Power1) app.hud.text(r, c.rect.x - 14.0F - app.hud.width("B strength"), c.rect.y + 5.0F, "B strength", gd::kMuted);
    }
    for (const Chip& c : app.chips) {
        const bool off = disabled(app, c.action);
        const bool on = active(app, c.action);
        const bool bad = naive(app, c.action);
        gd::Color fill = gd::kWhite;
        gd::Color ink = gd::kInk;
        if (off) {
            fill = gd::kGrid;
            ink = gd::kMuted;
        } else if (bad) {
            fill = gd::kOrange;
            ink = gd::kWhite;
        } else if (on) {
            fill = gd::kTeal;
            ink = gd::kWhite;
        }
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, fill, gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 5.0F, c.label, ink, gd::Hud::Style::Bold);
    }
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::backdrop(r, kWidth, kHeight);
    app.hud.titleBar(r, kWidth, "[ LANCHESTER BATTLES ]", isDuels(app) ? "linear law · duels" : "square law · ranged");
    drawField(app);
    drawNotes(app);
    drawPlot(app);
    drawResult(app);
    drawChips(app);
}

void countFinished(App& app) {
    if (app.counted || !app.battle.finished()) return;
    app.counted = true;
    const lb::Outcome o = app.battle.outcome();
    ++app.stats.runs;
    if (o.winner == lb::Winner::A) ++app.stats.aWins;
    app.stats.sumLeft += o.left;
}

void advance(App& app, double seconds) {
    app.accumulator += seconds;
    int steps = 0;
    while (app.accumulator >= kStep && steps < kMaxStepsPerFrame && !app.battle.finished()) {
        const bool before = app.battle.reserveReleased();
        app.battle.step(kStep);
        if (!before && app.battle.reserveReleased()) app.bAtRelease = app.battle.alive(1);
        app.accumulator -= kStep;
        ++steps;
    }
    if (steps == kMaxStepsPerFrame || app.battle.finished()) app.accumulator = 0.0;  // drop a backlog instead of spiralling
    countFinished(app);
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
    if (!SDL_CreateWindowAndRenderer("lanchester-battles", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
        SDL_Log("window/renderer failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(app->renderer, kWidth, kHeight, gd::kLogicalPresentation);
    if (!app->hud.load(app->renderer)) {
        SDL_Log("font atlas failed: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderVSync(app->renderer, 1);
    layoutChips(*app);
    startBattle(*app);
    advance(*app, 2.5);  // the first frame already has shots in the air
    app->lastTicks = SDL_GetTicksNS();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const SDL_FPoint p{event->button.x, event->button.y};
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect) && !disabled(app, c.action)) {
                    apply(app, c.action);
                    break;
                }
            }
            break;
        }
        case SDL_EVENT_KEY_DOWN: {
            if (event->key.repeat) break;
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: apply(app, Action::LawSquare); break;
                case SDLK_2: apply(app, Action::LawLinear); break;
                case SDLK_3: apply(app, Action::Army100v70); break;
                case SDLK_4: apply(app, Action::Army100v50); break;
                case SDLK_5: apply(app, Action::ArmySplit); break;
                case SDLK_6: apply(app, Action::Power1); break;
                case SDLK_7: apply(app, Action::Power2); break;
                case SDLK_8: apply(app, Action::Power4); break;
                case SDLK_W: apply(app, Action::Range); break;
                case SDLK_A: apply(app, Action::Aim); break;
                case SDLK_S: apply(app, Action::Speed); break;
                case SDLK_SPACE: apply(app, Action::Pause); break;
                case SDLK_R: apply(app, Action::Restart); break;
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
    const Uint64 now = SDL_GetTicksNS();
    double dt = static_cast<double>(now - app.lastTicks) / 1e9;
    app.lastTicks = now;
    dt = std::min(dt, 0.25);
    if (app.maxFrames >= 0) dt = 4.0 * kStep;  // smoke runs: fixed progress per frame, independent of how fast frames render
    if (!app.paused) advance(app, dt * kSpeeds[static_cast<std::size_t>(app.speedIndex)]);
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
