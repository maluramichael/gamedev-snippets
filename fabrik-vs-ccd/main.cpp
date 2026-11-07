// FABRIK vs CCD with SDL3 main callbacks.
//
// One chain of ten segments follows the target (the mouse). Every 1/60 s the chosen solver runs a few iterations from the
// pose the chain already has, which is how a game does it. Three solvers: CCD turns joint by joint from the tip to the
// root, FABRIK slides the points along lines (tip on the target, back to the root, forward again, lengths are exact by
// construction), relaxation pins both ends and lets every segment share its length error. The faint poses are the steps
// inside the last iteration. The panel counts how far the tip is from the target and, for all three solvers at once,
// how many iterations each would need to get from the start pose to this target. Until the mouse moves, the target
// jumps between spots inside the reach and beyond it, so the chain has to travel. "Stuck" sets up a two-segment chain that lies straight with the
// target on the same line, where every solver freezes. The math lives in logic.hpp and is covered by test.cpp.

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
constexpr float kFieldTop = 34.0F;
constexpr float kFieldBottom = 438.0F;
constexpr float kPanelX = 660.0F;
constexpr float kPanelY = 44.0F;
constexpr float kPanelW = 276.0F;
constexpr float kPanelH = 284.0F;
constexpr float kChipH = 28.0F;
constexpr float kChipRow1 = 468.0F;
constexpr float kChipRow2 = 502.0F;
constexpr fk::Vec2 kRoot{330.0, 236.0};
constexpr int kSegments = 10;
constexpr double kSegmentLength = 20.0;  // reach 200: the circle around the root just fits the field
constexpr double kRestBend = 0.12;       // radians per joint in the start pose, a gentle curl
constexpr double kTol = 0.5;             // px
constexpr double kTick = 1.0 / 60.0;
constexpr double kMaxDt = 0.25;
constexpr int kMaxTicksPerFrame = 5;
constexpr int kRaceCap = 500;
constexpr std::array<int, 4> kPerTick{1, 2, 4, 16};
constexpr gd::Color kRingFill{13, 148, 136, 22};

enum class Action { SetAlgo, Stuck, Reset, SetPerTick, Ghosts, Auto, Pause, Step };

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

    fk::Algo algo = fk::Algo::Ccd;
    fk::Chain chain;
    fk::Chain start;  // where the chain starts and where the iteration counts of the three solvers start from
    fk::Trace trace;
    fk::Vec2 target = {0.0, 0.0};
    int perTickIndex = 0;
    bool stuckMode = false;
    bool pinned = false;  // stuck preset: the target stays where the preset put it until the first click in the field
    bool ghosts = true;
    bool autoPath = true;
    bool paused = false;
    double phase = 0.0;
    double accumulator = 0.0;
    int used = 0;  // iterations the solver ran in the last tick
    bool stalled = false;

    std::array<fk::Result, 3> race{};
    std::array<bool, 3> raceStretched{};
    bool raceOutOfReach = false;
    fk::Vec2 raceTarget = {0.0, 0.0};
    bool raceDirty = true;

    Uint64 lastTicks = 0;
    long maxFrames = -1;
    long frames = 0;
};

SDL_FPoint fp(fk::Vec2 v) { return {static_cast<float>(v.x), static_cast<float>(v.y)}; }

bool inField(fk::Vec2 p) { return p.y >= static_cast<double>(kFieldTop) && p.y <= static_cast<double>(kFieldBottom); }

bool inPanel(fk::Vec2 p) { return p.x >= static_cast<double>(kPanelX) && p.x <= static_cast<double>(kPanelX + kPanelW) && p.y >= static_cast<double>(kPanelY) && p.y <= static_cast<double>(kPanelY + kPanelH); }

const char* algoName(fk::Algo a) {
    switch (a) {
        case fk::Algo::Ccd: return "CCD";
        case fk::Algo::Fabrik: return "FABRIK";
        case fk::Algo::Relax: return "RELAX";
    }
    return "";
}

gd::Color algoColor(fk::Algo a) {
    switch (a) {
        case fk::Algo::Ccd: return gd::kOrange;
        case fk::Algo::Fabrik: return gd::kTeal;
        case fk::Algo::Relax: return gd::kMuted;
    }
    return gd::kInk;
}

int algoIndex(fk::Algo a) { return static_cast<int>(a); }

fk::Chain makeStart(bool stuck) {
    if (stuck) return fk::makeChain(kRoot, {100.0, 100.0});  // two segments, straight along +x
    return fk::makeUniformChain(kRoot, kSegments, kSegmentLength, kRestBend);
}

fk::Vec2 clampToField(fk::Vec2 p) {
    return {std::clamp(p.x, 10.0, static_cast<double>(kPanelX) - 10.0), std::clamp(p.y, static_cast<double>(kFieldTop) + 10.0, static_cast<double>(kFieldBottom) - 10.0)};
}

void layoutChips(App& app) {
    struct Def {
        Action action;
        int value;
        const char* label;
        float gapBefore;
    };
    const std::array<Def, 5> row1{{{Action::SetAlgo, static_cast<int>(fk::Algo::Ccd), "1 CCD", 0.0F},
                                   {Action::SetAlgo, static_cast<int>(fk::Algo::Fabrik), "2 FABRIK", 8.0F},
                                   {Action::SetAlgo, static_cast<int>(fk::Algo::Relax), "3 Relaxation", 8.0F},
                                   {Action::Stuck, 0, "S Stuck", 24.0F},
                                   {Action::Reset, 0, "R Reset pose", 8.0F}}};
    float x = 40.0F;
    for (const Def& d : row1) {
        x += d.gapBefore;
        const float w = app.hud.width(d.label, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({d.action, d.value, d.label, SDL_FRect{x, kChipRow1, w, kChipH}});
        x += w;
    }
    x = 40.0F;
    const char* caption = "I  Iterations per tick";
    x += app.hud.width(caption) + 14.0F;
    const std::array<const char*, 4> counts{"1", "2", "4", "16"};
    for (std::size_t i = 0; i < counts.size(); ++i) {
        const float w = app.hud.width(counts[i], gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({Action::SetPerTick, static_cast<int>(i), counts[i], SDL_FRect{x, kChipRow2, w, kChipH}});
        x += w + 6.0F;
    }
    x += 18.0F;
    const std::array<Def, 4> row2{{{Action::Ghosts, 0, "G Ghosts", 0.0F},
                                   {Action::Auto, 0, "A Auto path", 8.0F},
                                   {Action::Pause, 0, "P Pause", 24.0F},
                                   {Action::Step, 0, "N Step", 8.0F}}};
    for (const Def& d : row2) {
        x += d.gapBefore;
        const float w = app.hud.width(d.label, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({d.action, d.value, d.label, SDL_FRect{x, kChipRow2, w, kChipH}});
        x += w;
    }
}

void runIterations(App& app, int count);

void resetSolver(App& app) {
    app.chain = app.start;
    app.trace.clear();
    app.accumulator = 0.0;
    app.used = 0;
    app.stalled = false;
    app.raceDirty = true;
}

void apply(App& app, Action action, int value) {
    switch (action) {
        case Action::SetAlgo:
            app.algo = static_cast<fk::Algo>(value);
            app.trace.clear();  // the ghosts belong to the solver that made them
            app.used = 0;
            app.stalled = false;
            app.accumulator = 0.0;
            break;
        case Action::Stuck:
            app.stuckMode = !app.stuckMode;
            app.start = makeStart(app.stuckMode);
            resetSolver(app);
            if (app.stuckMode) {
                app.autoPath = false;
                app.pinned = true;
                app.target = kRoot + fk::Vec2{-50.0, 0.0};  // on the line of the straight chain, behind the root
            } else {
                app.autoPath = true;
                app.pinned = false;
                app.phase = 0.0;
                app.target = clampToField(fk::autoTarget(kRoot, fk::reach(app.chain), app.phase));
            }
            break;
        case Action::Reset: resetSolver(app); break;
        case Action::SetPerTick:
            app.perTickIndex = std::clamp(value, 0, static_cast<int>(kPerTick.size()) - 1);
            app.accumulator = 0.0;
            break;
        case Action::Ghosts: app.ghosts = !app.ghosts; break;
        case Action::Pause:
            app.paused = !app.paused;
            app.accumulator = 0.0;
            break;
        case Action::Step:
            app.paused = true;  // stepping by hand means the clock stands still
            app.accumulator = 0.0;
            runIterations(app, 1);
            break;
        case Action::Auto:
            app.autoPath = !app.autoPath;
            if (app.autoPath) {
                app.phase = 0.0;
                app.pinned = false;
                app.accumulator = 0.0;
            }
            break;
    }
}

bool active(const App& app, const Chip& c) {
    switch (c.action) {
        case Action::SetAlgo: return app.algo == static_cast<fk::Algo>(c.value);
        case Action::Stuck: return app.stuckMode;
        case Action::Reset: return false;
        case Action::SetPerTick: return app.perTickIndex == c.value;
        case Action::Ghosts: return app.ghosts;
        case Action::Auto: return app.autoPath;
        case Action::Pause: return app.paused;
        case Action::Step: return false;
    }
    return false;
}

/// Up to `count` iterations of the chosen solver, stops early when the tip is there or the pose stands still.
/// The poses of the last iteration stay in the trace for the ghosts.
void runIterations(App& app, int count) {
    int used = 0;
    bool stalled = false;
    for (int i = 0; i < count; ++i) {
        if (fk::reached(app.chain, app.target, kTol)) break;
        const fk::Pose before = app.chain.p;
        fk::iterate(app.algo, app.chain, app.target, &app.trace);
        ++used;
        if (fk::maxShift(before, app.chain.p, app.chain.n) < fk::kStall) {
            stalled = true;
            break;
        }
    }
    app.used = used;
    app.stalled = stalled;
    if (used == 0) app.trace.clear();  // nothing left to do: no ghosts
}

/// One 1/60 s step: move the demo target, then run the iterations of this tick.
void tick(App& app) {
    if (app.autoPath) {
        app.phase = fk::advancePhase(app.phase, kTick);
        app.target = clampToField(fk::autoTarget(kRoot, fk::reach(app.chain), app.phase));
    }
    runIterations(app, kPerTick[static_cast<std::size_t>(app.perTickIndex)]);
}

/// How many iterations each solver needs from the start pose to the current target. Only when the target moved.
void updateRace(App& app) {
    if (!app.raceDirty && fk::length(app.target - app.raceTarget) < 0.25) return;
    app.raceTarget = app.target;
    app.raceDirty = false;
    for (const fk::Algo a : {fk::Algo::Ccd, fk::Algo::Fabrik, fk::Algo::Relax}) {
        fk::Chain c = app.start;
        const auto i = static_cast<std::size_t>(algoIndex(a));
        app.race[i] = fk::solve(a, c, app.target, kTol, kRaceCap);
        app.raceStretched[i] = fk::lengthError(c) > kTol;
    }
    app.raceOutOfReach = fk::bestError(app.start, app.target) > kTol;
}

void segmentLine(SDL_Renderer* r, fk::Vec2 a, fk::Vec2 b, gd::Color c) {
    gd::setColor(r, c);
    SDL_RenderLine(r, static_cast<float>(a.x), static_cast<float>(a.y), static_cast<float>(b.x), static_cast<float>(b.y));
}

void dashedLine(SDL_Renderer* r, fk::Vec2 a, fk::Vec2 b, gd::Color c) {
    const double len = fk::length(b - a);
    if (len < 1.0) return;
    const fk::Vec2 dir = (b - a) * (1.0 / len);
    for (double s = 0.0; s < len; s += 12.0) segmentLine(r, a + dir * s, a + dir * std::min(s + 6.0, len), c);
}

/// A segment: rounded bar with a 2 px ink outline.
void limb(SDL_Renderer* r, fk::Vec2 a, fk::Vec2 b, float width, gd::Color fill) {
    const double len = fk::length(b - a);
    auto bar = [&](float w, gd::Color c) {
        if (len > 1e-6) {
            const double nx = -(b.y - a.y) / len * static_cast<double>(w) / 2.0;
            const double ny = (b.x - a.x) / len * static_cast<double>(w) / 2.0;
            const SDL_FColor f = gd::fcolor(c);
            const SDL_Vertex v[4] = {{fp({a.x + nx, a.y + ny}), f, {0, 0}}, {fp({b.x + nx, b.y + ny}), f, {0, 0}}, {fp({b.x - nx, b.y - ny}), f, {0, 0}}, {fp({a.x - nx, a.y - ny}), f, {0, 0}}};
            const int idx[6] = {0, 1, 2, 0, 2, 3};
            SDL_RenderGeometry(r, nullptr, v, 4, idx, 6);
        }
        gd::disc(r, static_cast<float>(a.x), static_cast<float>(a.y), w / 2.0F, c, 16);
        gd::disc(r, static_cast<float>(b.x), static_cast<float>(b.y), w / 2.0F, c, 16);
    };
    bar(width + 4.0F, gd::kInk);
    bar(width, fill);
}

void drawChain(App& app, gd::Color fill) {
    SDL_Renderer* r = app.renderer;
    const fk::Chain& c = app.chain;
    for (int i = 0; i < c.n; ++i) {
        const auto k = static_cast<std::size_t>(i);
        const float t = c.n > 1 ? static_cast<float>(i) / static_cast<float>(c.n - 1) : 0.0F;
        limb(r, c.p[k], c.p[k + 1], 14.0F - 6.0F * t, fill);
    }
    for (int i = 1; i < c.n; ++i) {
        const fk::Vec2 p = c.p[static_cast<std::size_t>(i)];
        gd::disc(r, static_cast<float>(p.x), static_cast<float>(p.y), 3.0F, gd::kWhite, 12);
    }
}

void drawGhosts(App& app) {
    SDL_Renderer* r = app.renderer;
    gd::Color c = algoColor(app.algo);
    c.a = 150;
    for (int s = 0; s < app.trace.count; ++s) {
        const fk::Pose& pose = app.trace.poses[static_cast<std::size_t>(s)];
        for (int i = 0; i < app.chain.n; ++i) {
            const fk::Vec2 a = pose[static_cast<std::size_t>(i)];
            const fk::Vec2 b = pose[static_cast<std::size_t>(i) + 1];
            segmentLine(r, a, b, c);
        }
        for (int i = 1; i <= app.chain.n; ++i) {
            const fk::Vec2 p = pose[static_cast<std::size_t>(i)];
            gd::fillRect(r, static_cast<float>(p.x) - 1.5F, static_cast<float>(p.y) - 1.5F, 3.0F, 3.0F, c);
        }
    }
}

void drawReach(App& app) {
    SDL_Renderer* r = app.renderer;
    const auto cx = static_cast<float>(kRoot.x);
    const auto cy = static_cast<float>(kRoot.y);
    const auto radius = static_cast<float>(fk::reach(app.chain));
    gd::disc(r, cx, cy, radius, kRingFill, 64);
    gd::ring(r, cx, cy, radius, gd::kTeal, 96);
    app.hud.text(r, cx + radius * 0.7071F + 6.0F, cy - radius * 0.7071F - 18.0F, "reach", gd::kTealDark);
}

void panelRow(App& app, float y, const char* label, const char* value, gd::Color color) {
    app.hud.text(app.renderer, kPanelX + 14.0F, y, label, gd::kMuted);
    app.hud.text(app.renderer, kPanelX + kPanelW - 14.0F - app.hud.width(value, gd::Hud::Style::Bold), y, value, color, gd::Hud::Style::Bold);
}

void raceRow(App& app, float y, fk::Algo a) {
    SDL_Renderer* r = app.renderer;
    const auto i = static_cast<std::size_t>(algoIndex(a));
    const fk::Result& res = app.race[i];
    const gd::Color color = algoColor(a);
    app.hud.text(r, kPanelX + 14.0F, y, algoName(a), a == app.algo ? gd::kInk : gd::kMuted, a == app.algo ? gd::Hud::Style::Bold : gd::Hud::Style::Regular);
    char buf[24];
    const char* text = buf;
    bool bar = false;
    switch (res.outcome) {
        case fk::Outcome::Reached:
            SDL_snprintf(buf, sizeof buf, "%d", res.iterations);
            bar = true;
            break;
        case fk::Outcome::Stuck:
        case fk::Outcome::Limit:
            if (app.raceStretched[i] && app.raceOutOfReach) {
                text = "stretched";  // pinned at both ends, the segments gave way
            } else if (res.outcome == fk::Outcome::Stuck) {
                text = "stuck";
            } else {
                SDL_snprintf(buf, sizeof buf, ">%d", kRaceCap);
            }
            break;
    }
    const bool bad = res.outcome != fk::Outcome::Reached;
    app.hud.text(r, kPanelX + kPanelW - 14.0F - app.hud.width(text, gd::Hud::Style::Bold), y, text, bad ? gd::kOrange : gd::kInk, gd::Hud::Style::Bold);
    if (bar) {
        const float full = 104.0F;
        const float w = std::max(2.0F, full * static_cast<float>(std::log1p(static_cast<double>(res.iterations)) / std::log1p(static_cast<double>(kRaceCap))));
        gd::fillRect(r, kPanelX + 84.0F, y + 5.0F, full, 10.0F, gd::kGrid);
        gd::fillRect(r, kPanelX + 84.0F, y + 5.0F, w, 10.0F, color);
        gd::strokeRect(r, kPanelX + 84.0F, y + 5.0F, full, 10.0F, gd::kInk);
    }
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    const fk::Chain& c = app.chain;
    const double err = fk::tipError(c, app.target);
    const double best = fk::bestError(c, app.target);
    const double lenErr = fk::lengthError(c);
    const bool atBest = fk::reached(c, app.target, kTol);
    const bool outOfReach = best > kTol;
    const gd::Color verdict = (atBest && !outOfReach) ? gd::kTeal : gd::kOrange;
    const gd::Color color = algoColor(app.algo);

    gd::backdrop(r, kWidth, kHeight);
    char buf[64];
    SDL_snprintf(buf, sizeof buf, "%d segments x %.0f px%s", c.n, c.d[0], app.stuckMode ? ", straight" : "");
    app.hud.titleBar(r, kWidth, "[ FABRIK VS CCD ]", buf);

    drawReach(app);
    if (app.ghosts) drawGhosts(app);
    if (err > kTol) dashedLine(r, fk::tip(c), app.target, gd::kOrange);
    drawChain(app, color);

    const auto rx = static_cast<float>(kRoot.x);
    const auto ry = static_cast<float>(kRoot.y);
    gd::disc(r, rx, ry, 9.0F, gd::kInk, 20);
    const fk::Vec2 tp = fk::tip(c);
    gd::disc(r, static_cast<float>(tp.x), static_cast<float>(tp.y), 6.0F, gd::kInk, 16);
    gd::disc(r, static_cast<float>(tp.x), static_cast<float>(tp.y), 3.5F, color, 16);
    const auto tx = static_cast<float>(app.target.x);
    const auto ty = static_cast<float>(app.target.y);
    gd::disc(r, tx, ty, 4.0F, verdict, 16);
    gd::ring(r, tx, ty, 10.0F, verdict);
    gd::ring(r, tx, ty, 11.0F, gd::kInk);

    // Readout.
    gd::card(r, kPanelX, kPanelY, kPanelW, kPanelH, gd::kWhite, gd::kInk);  // same rectangle as inPanel()
    app.hud.text(r, kPanelX + 14.0F, kPanelY + 10.0F, algoName(app.algo), color, gd::Hud::Style::Bold);
    SDL_snprintf(buf, sizeof buf, "%.1f px", err);
    panelRow(app, kPanelY + 38.0F, "error", buf, atBest ? gd::kInk : gd::kOrange);
    SDL_snprintf(buf, sizeof buf, "%d of %d", app.used, kPerTick[static_cast<std::size_t>(app.perTickIndex)]);
    panelRow(app, kPanelY + 62.0F, "iterations/tick", buf, gd::kInk);
    SDL_snprintf(buf, sizeof buf, "%.0f°", fk::totalBend(c) * 180.0 / fk::kPi);
    panelRow(app, kPanelY + 86.0F, "total bend", buf, gd::kInk);
    SDL_snprintf(buf, sizeof buf, "%.1f px", lenErr);
    panelRow(app, kPanelY + 110.0F, "length error", buf, lenErr > kTol ? gd::kOrange : gd::kInk);
    const char* status = "CLOSING IN";
    if (atBest) {
        status = outOfReach ? "OUT OF REACH: straight" : "REACHED";
    } else if (outOfReach && lenErr > kTol) {
        status = "STRETCHED: lengths broken";
    } else if (app.stalled) {
        status = "STUCK: not moving";
    }
    gd::Color statusColor = gd::kInk;
    if (atBest && !outOfReach) {
        statusColor = gd::kTealDark;
    } else if (atBest || app.stalled || outOfReach) {
        statusColor = gd::kOrange;
    }
    app.hud.text(r, kPanelX + 14.0F, kPanelY + 140.0F, status, statusColor, gd::Hud::Style::Bold);

    gd::fillRect(r, kPanelX + 14.0F, kPanelY + 170.0F, kPanelW - 28.0F, 1.0F, gd::kGrid);
    app.hud.text(r, kPanelX + 14.0F, kPanelY + 180.0F, "iterations from start", gd::kMuted);
    raceRow(app, kPanelY + 206.0F, fk::Algo::Ccd);
    raceRow(app, kPanelY + 230.0F, fk::Algo::Fabrik);
    raceRow(app, kPanelY + 254.0F, fk::Algo::Relax);

    app.hud.text(r, 40.0F, 442.0F, "mouse = target   faint poses = steps of the last iteration   dashed = tip to target", gd::kMuted);

    for (const Chip& chip : app.chips) {
        const bool on = active(app, chip);
        gd::card(r, chip.rect.x, chip.rect.y, chip.rect.w, chip.rect.h, on ? gd::kTeal : gd::kWhite, gd::kInk);
        app.hud.text(r, chip.rect.x + (chip.rect.w - app.hud.width(chip.label, gd::Hud::Style::Bold)) / 2.0F, chip.rect.y + 3.0F, chip.label, on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
    }
    app.hud.text(r, 40.0F, kChipRow2 + 3.0F, "I  Iterations per tick", gd::kMuted);
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
    if (!SDL_CreateWindowAndRenderer("fabrik-vs-ccd", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
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
    app->start = makeStart(false);
    app->chain = app->start;
    app->target = clampToField(fk::autoTarget(kRoot, fk::reach(app->chain), app->phase));
    app->lastTicks = SDL_GetTicksNS();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION: {
            const fk::Vec2 m{static_cast<double>(event->motion.x), static_cast<double>(event->motion.y)};
            if (!app.pinned && inField(m) && !inPanel(m)) {  // the readout is not part of the field, the target never hides under it
                app.target = m;
                app.autoPath = false;  // the mouse takes over from the demo path
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const SDL_FPoint p{event->button.x, event->button.y};
            bool onChip = false;
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) {
                    apply(app, c.action, c.value);
                    onChip = true;
                    break;  // apply() may rebuild state, never walk on after it
                }
            }
            const fk::Vec2 m{static_cast<double>(p.x), static_cast<double>(p.y)};
            if (!onChip && inField(m) && !inPanel(m)) {
                app.pinned = false;  // the first click frees the stuck preset's target
                app.target = m;
                app.autoPath = false;
            }
            break;
        }
        case SDL_EVENT_KEY_DOWN:
            if ((event->key.key == SDLK_N || event->key.key == SDLK_SPACE) && !event->key.repeat) {
                apply(app, Action::Step, 0);
                break;
            }
            if (event->key.repeat) break;  // the other keys toggle, none should fire while held
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: apply(app, Action::SetAlgo, static_cast<int>(fk::Algo::Ccd)); break;
                case SDLK_2: apply(app, Action::SetAlgo, static_cast<int>(fk::Algo::Fabrik)); break;
                case SDLK_3: apply(app, Action::SetAlgo, static_cast<int>(fk::Algo::Relax)); break;
                case SDLK_S: apply(app, Action::Stuck, 0); break;
                case SDLK_R: apply(app, Action::Reset, 0); break;
                case SDLK_I: apply(app, Action::SetPerTick, (app.perTickIndex + 1) % static_cast<int>(kPerTick.size())); break;
                case SDLK_G: apply(app, Action::Ghosts, 0); break;
                case SDLK_A: apply(app, Action::Auto, 0); break;
                case SDLK_P: apply(app, Action::Pause, 0); break;
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
    const double dt = std::min(kMaxDt, static_cast<double>(now - app.lastTicks) / 1.0e9);
    app.lastTicks = now;
    app.accumulator = app.paused ? 0.0 : app.accumulator + dt;
    int steps = 0;
    while (app.accumulator >= kTick && steps < kMaxTicksPerFrame) {
        tick(app);
        app.accumulator -= kTick;
        ++steps;
    }
    updateRace(app);
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
