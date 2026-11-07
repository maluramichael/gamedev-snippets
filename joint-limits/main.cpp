// Joint limits with SDL3 main callbacks.
//
// An arm of four segments follows a target, and every joint has an allowed range, drawn as a wedge. Three ways to deal
// with the ranges, switchable at any time: none, solve without limits and clamp the finished pose (the tip misses,
// shown by a ghost of the unclamped pose), or clamp inside every solver step (the best pose the ranges allow). The
// colored map is the same question asked for a grid of targets: teal where the tip lands on the target, orange where
// it misses. The edges of the wedges are draggable. The math is in logic.hpp and covered by test.cpp.

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
#include <vector>

#include "hud.hpp"
#include "logic.hpp"

namespace {

constexpr int kWidth = 960;
constexpr int kHeight = 540;
constexpr float kFieldRight = 600.0F;
constexpr float kFieldTop = 34.0F;
constexpr float kFieldBottom = 420.0F;
constexpr float kMapCell = 12.0F;
constexpr int kMapCols = 50;
constexpr int kMapRows = 32;
constexpr double kMapTol = 2.0;       // px, a map cell counts as reached below this
constexpr double kSolveTol = 0.5;     // px, the solver stops below this
constexpr int kMapIterations = 40;
constexpr double kTick = 1.0 / 60.0;  // seconds per solver tick
constexpr int kSweepsPerTick = 3;
constexpr int kMaxTicksPerFrame = 15;
constexpr double kWedgeRadius = 30.0;
constexpr double kHandleRadius = 42.0;
constexpr double kHandleGrab = 12.0;
constexpr double kPinned = 0.5 * jl::kPi / 180.0;
constexpr float kChipH = 30.0F;
constexpr float kChipRow1 = 452.0F;
constexpr float kChipRow2 = 492.0F;
constexpr float kPanelX = 612.0F;

constexpr jl::Vec2 kRoot{300.0, 227.0};
const jl::Arm kArm{kRoot, {56.0, 50.0, 44.0, 38.0}};
constexpr std::array<const char*, jl::kJoints> kJointNames{"shoulder", "elbow", "wrist", "hand"};

enum class Action { ModeOff, ModeEnd, ModeStep, PresetArm, PresetStiff, ToggleMap, ToggleAuto, Reset };

struct Chip {
    Action action;
    std::string label;
    SDL_FRect rect;
};

struct Handle {
    std::size_t joint;
    bool upper;
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    gd::Hud hud;
    std::vector<Chip> chips;
    jl::Limits limits = jl::armLimits();
    int preset = 0;  // 0 arm, 1 stiff, -1 edited by hand
    jl::Mode mode = jl::Mode::End;
    jl::Angles rest = jl::restPose(jl::armLimits());
    jl::Angles unlimited = rest;  // the pose of the solver that never sees a limit
    jl::Angles limited = rest;    // the pose of the solver that clamps every step
    bool limitedStuck = false;    // its last sweep moved nothing
    jl::Vec2 target{0, 0};
    jl::Vec2 mouse{-100.0, -100.0};
    bool autoTarget = true;
    double autoPhase = 0.0;
    double accumulator = 0.0;
    Uint64 lastNs = 0;
    bool showMap = true;
    bool mapDirty = true;
    jl::Map map = jl::makeMap({0.0, static_cast<double>(kFieldTop)}, static_cast<double>(kMapCell), kMapCols, kMapRows);
    int mapReached = 0;
    int mapReachable = 0;
    std::optional<Handle> dragging;
    long maxFrames = -1;
    long frames = 0;
};

/// The pose that is on screen: what the active mode ends up with.
jl::Angles shown(const App& app) {
    switch (app.mode) {
        case jl::Mode::Off: return app.unlimited;
        case jl::Mode::End: return jl::clampAll(app.unlimited, app.limits);
        case jl::Mode::Step: return app.limited;
    }
    return app.unlimited;
}

jl::Vec2 toVec(SDL_FPoint p) { return {static_cast<double>(p.x), static_cast<double>(p.y)}; }

bool inField(jl::Vec2 p) { return p.x >= 0.0 && p.x <= static_cast<double>(kFieldRight) && p.y >= static_cast<double>(kFieldTop) && p.y <= static_cast<double>(kFieldBottom); }

SDL_FPoint pt(jl::Vec2 v) { return {static_cast<float>(v.x), static_cast<float>(v.y)}; }

void layoutChips(App& app) {
    struct Def {
        Action action;
        const char* label;
        float x;
        float y;
        float gapBefore;
    };
    const std::array<Def, 8> defs{{{Action::ModeOff, "1 Limits off", 40.0F, kChipRow1, 0.0F},
                                   {Action::ModeEnd, "2 Clamp at end", 0.0F, kChipRow1, 8.0F},
                                   {Action::ModeStep, "3 Clamp every step", 0.0F, kChipRow1, 8.0F},
                                   {Action::PresetArm, "4 Arm limits", 40.0F, kChipRow2, 0.0F},
                                   {Action::PresetStiff, "5 Stiff limits", 0.0F, kChipRow2, 8.0F},
                                   {Action::ToggleMap, "M Workspace map", 0.0F, kChipRow2, 24.0F},
                                   {Action::ToggleAuto, "A Auto target", 0.0F, kChipRow2, 8.0F},
                                   {Action::Reset, "R Reset pose", 0.0F, kChipRow2, 24.0F}}};
    float x = 0.0F;
    for (const Def& d : defs) {
        x = d.x > 0.0F ? d.x : x + d.gapBefore;
        const float w = app.hud.width(d.label, gd::Hud::Style::Bold) + 24.0F;
        app.chips.push_back({d.action, d.label, SDL_FRect{x, d.y, w, kChipH}});
        x += w;
    }
}

bool active(const App& app, Action action) {
    switch (action) {
        case Action::ModeOff: return app.mode == jl::Mode::Off;
        case Action::ModeEnd: return app.mode == jl::Mode::End;
        case Action::ModeStep: return app.mode == jl::Mode::Step;
        case Action::PresetArm: return app.preset == 0;
        case Action::PresetStiff: return app.preset == 1;
        case Action::ToggleMap: return app.showMap;
        case Action::ToggleAuto: return app.autoTarget;
        case Action::Reset: return false;
    }
    return false;
}

void setLimits(App& app, const jl::Limits& l, int preset) {
    app.limits = l;
    app.preset = preset;
    app.limited = jl::clampAll(app.limited, l);  // a limit is hard, the clamping solver is inside at once
    app.mapDirty = true;
}

void apply(App& app, Action action) {
    switch (action) {
        case Action::ModeOff:
        case Action::ModeEnd:
        case Action::ModeStep:
            app.mode = action == Action::ModeOff ? jl::Mode::Off : action == Action::ModeEnd ? jl::Mode::End : jl::Mode::Step;
            app.mapDirty = true;
            break;
        case Action::PresetArm:
            setLimits(app, jl::armLimits(), 0);
            app.rest = jl::restPose(app.limits);
            break;
        case Action::PresetStiff:
            setLimits(app, jl::stiffLimits(), 1);
            app.rest = jl::restPose(app.limits);
            break;
        case Action::ToggleMap: app.showMap = !app.showMap; break;
        case Action::ToggleAuto: app.autoTarget = !app.autoTarget; break;
        case Action::Reset:
            app.rest = jl::restPose(app.limits);
            app.unlimited = app.rest;
            app.limited = app.rest;
            app.limitedStuck = false;
            app.accumulator = 0.0;
            app.mapDirty = true;
            break;
    }
}

void tick(App& app) {
    if (app.autoTarget) {
        app.autoPhase = jl::advancePhase(app.autoPhase, kTick);
        app.target = jl::autoTarget(kRoot, jl::reachMax(kArm), app.autoPhase);
    }
    for (int i = 0; i < kSweepsPerTick; ++i) {
        const jl::Angles before = app.limited;
        jl::sweep(kArm, app.unlimited, app.target, nullptr);
        jl::sweep(kArm, app.limited, app.target, &app.limits);
        double moved = 0.0;
        for (std::size_t j = 0; j < jl::kJoints; ++j) moved = std::max(moved, std::fabs(jl::wrapAngle(app.limited[j] - before[j])));
        app.limitedStuck = moved < jl::kStall;
    }
}

void refreshMap(App& app) {
    if (!app.showMap || !app.mapDirty) return;
    jl::computeMap(app.map, app.mode, kArm, app.rest, app.limits, kSolveTol, kMapIterations);
    app.mapReached = 0;
    app.mapReachable = 0;
    for (int row = 0; row < app.map.rows; ++row) {
        for (int col = 0; col < app.map.cols; ++col) {
            const double dist = jl::length(jl::cellCenter(app.map, col, row) - kRoot);
            if (dist < jl::reachMin(kArm) || dist > jl::reachMax(kArm)) continue;
            ++app.mapReachable;
            if (static_cast<double>(app.map.error[static_cast<std::size_t>(row * app.map.cols + col)]) <= kMapTol) ++app.mapReached;
        }
    }
    app.mapDirty = false;
}

/// Direction of the segment before joint j: the reference the joint's angle is measured from.
double parentDirection(const jl::Angles& a, std::size_t j) {
    double dir = 0.0;
    for (std::size_t i = 0; i < j; ++i) dir += a[i];
    return dir;
}

jl::Vec2 handlePosition(const jl::Points& p, const jl::Angles& a, const jl::Limits& l, const Handle& h) {
    const double dir = parentDirection(a, h.joint) + (h.upper ? l.hi[h.joint] : l.lo[h.joint]);
    return p[h.joint] + jl::Vec2{std::cos(dir), std::sin(dir)} * kHandleRadius;
}

std::optional<Handle> handleAt(const App& app, jl::Vec2 m) {
    const jl::Angles a = shown(app);
    const jl::Points p = jl::forward(kArm, a);
    std::optional<Handle> best;
    double bestDist = kHandleGrab;
    for (std::size_t j = 0; j < jl::kJoints; ++j) {
        for (const bool upper : {false, true}) {
            const Handle h{j, upper};
            const double d = jl::length(handlePosition(p, a, app.limits, h) - m);
            if (d < bestDist) {
                bestDist = d;
                best = h;
            }
        }
    }
    return best;
}

void dragTo(App& app, jl::Vec2 m) {
    if (!app.dragging) return;
    const jl::Angles a = shown(app);
    const jl::Points p = jl::forward(kArm, a);
    const std::size_t j = app.dragging->joint;
    const jl::Vec2 d = m - p[j];
    if (jl::length(d) < 4.0) return;  // too close to the joint for a direction
    const double rel = jl::wrapAngle(std::atan2(d.y, d.x) - parentDirection(a, j));
    setLimits(app, jl::dragLimit(app.limits, j, app.dragging->upper, rel), -1);
}

/// 1 px lines are too thin on a 960x540 canvas that is scaled up, so draw a few side by side.
void thickLine(SDL_Renderer* r, jl::Vec2 a, jl::Vec2 b, gd::Color c, int half = 1) {
    gd::setColor(r, c);
    const bool flat = std::fabs(b.x - a.x) > std::fabs(b.y - a.y);
    for (int o = -half; o <= half; ++o) {
        const auto off = static_cast<float>(o);
        SDL_RenderLine(r, static_cast<float>(a.x) + (flat ? 0.0F : off), static_cast<float>(a.y) + (flat ? off : 0.0F),
                       static_cast<float>(b.x) + (flat ? 0.0F : off), static_cast<float>(b.y) + (flat ? off : 0.0F));
    }
}

void dashedLine(SDL_Renderer* r, jl::Vec2 a, jl::Vec2 b, gd::Color c) {
    const jl::Vec2 d = b - a;
    const double len = jl::length(d);
    if (len < 1.0) return;
    const jl::Vec2 u = d * (1.0 / len);
    for (double s = 0.0; s < len; s += 12.0) thickLine(r, a + u * s, a + u * std::min(s + 7.0, len), c, 0);
}

/// Filled pie slice from angle a0 to a1 (a0 <= a1) around (cx, cy).
void fillSector(SDL_Renderer* r, jl::Vec2 c, double radius, double a0, double a1, gd::Color color) {
    constexpr int kMaxSteps = 64;
    const int steps = std::clamp(static_cast<int>(std::ceil((a1 - a0) / 0.1)), 1, kMaxSteps);
    std::array<SDL_Vertex, kMaxSteps + 2> v{};
    std::array<int, kMaxSteps * 3> idx{};
    const SDL_FColor fc = gd::fcolor(color);
    v[0] = {pt(c), fc, {0, 0}};
    for (int i = 0; i <= steps; ++i) {
        const double a = a0 + (a1 - a0) * static_cast<double>(i) / static_cast<double>(steps);
        v[static_cast<std::size_t>(i) + 1] = {pt(c + jl::Vec2{std::cos(a), std::sin(a)} * radius), fc, {0, 0}};
        if (i < steps) {
            idx[static_cast<std::size_t>(i) * 3] = 0;
            idx[static_cast<std::size_t>(i) * 3 + 1] = i + 1;
            idx[static_cast<std::size_t>(i) * 3 + 2] = i + 2;
        }
    }
    SDL_RenderGeometry(r, nullptr, v.data(), steps + 2, idx.data(), steps * 3);
}

gd::Color errorColor(double err) {
    if (err <= kMapTol) return gd::Color{13, 148, 136, 55};
    const double k = std::min(1.0, err / 80.0);
    return gd::Color{234, 88, 12, static_cast<Uint8>(30.0 + 110.0 * k)};
}

void drawMap(App& app) {
    SDL_Renderer* r = app.renderer;
    for (int row = 0; row < app.map.rows; ++row) {
        for (int col = 0; col < app.map.cols; ++col) {
            const jl::Vec2 c = jl::cellCenter(app.map, col, row);
            const double dist = jl::length(c - kRoot);
            if (dist < jl::reachMin(kArm) || dist > jl::reachMax(kArm)) continue;
            const float err = app.map.error[static_cast<std::size_t>(row * app.map.cols + col)];
            gd::fillRect(r, static_cast<float>(c.x) - kMapCell / 2.0F, static_cast<float>(c.y) - kMapCell / 2.0F, kMapCell - 1.0F, kMapCell - 1.0F, errorColor(static_cast<double>(err)));
        }
    }
}

void drawChain(SDL_Renderer* r, const jl::Points& p, gd::Color color, int half) {
    for (std::size_t i = 0; i < jl::kJoints; ++i) thickLine(r, p[i], p[i + 1], color, half);
}

const char* modeName(jl::Mode m) {
    switch (m) {
        case jl::Mode::Off: return "limits off";
        case jl::Mode::End: return "clamp at the end";
        case jl::Mode::Step: return "clamp every step";
    }
    return "";
}

void drawPanel(App& app, const jl::Angles& a, double error) {
    SDL_Renderer* r = app.renderer;
    gd::card(r, kPanelX, 44.0F, 336.0F, 372.0F, gd::kWhite, gd::kInk);
    const float x = kPanelX + 14.0F;
    app.hud.text(r, x, 52.0F, "tip error", gd::kMuted);
    char line[96];
    SDL_snprintf(line, sizeof line, "%.1f px", error);
    app.hud.text(r, x, 72.0F, line, error <= kMapTol ? gd::kTeal : gd::kOrange, gd::Hud::Style::Large);

    for (std::size_t j = 0; j < jl::kJoints; ++j) {
        const float y = 124.0F + static_cast<float>(j) * 46.0F;
        const double deg = jl::wrapAngle(a[j]) * 180.0 / jl::kPi;
        const bool pinned = std::fabs(jl::wrapAngle(a[j] - app.limits.lo[j])) < kPinned || std::fabs(jl::wrapAngle(a[j] - app.limits.hi[j])) < kPinned;
        SDL_snprintf(line, sizeof line, "%-8s %4d°", kJointNames[j], static_cast<int>(std::lround(deg)));
        app.hud.text(r, x, y, line, gd::kInk, gd::Hud::Style::Bold);
        SDL_snprintf(line, sizeof line, "[%d, %d]", static_cast<int>(std::lround(app.limits.lo[j] * 180.0 / jl::kPi)), static_cast<int>(std::lround(app.limits.hi[j] * 180.0 / jl::kPi)));
        app.hud.text(r, kPanelX + 322.0F - app.hud.width(line), y, line, gd::kMuted);
        // The bar is the full circle from -180 to 180 degrees, the allowed range on it, the joint's angle as a tick.
        const float bx = x;
        const float bw = 308.0F;
        const float by = y + 26.0F;
        const auto at = [&](double rad) { return bx + bw * static_cast<float>((rad + jl::kPi) / (2.0 * jl::kPi)); };
        gd::fillRect(r, bx, by, bw, 9.0F, gd::kGrid);
        gd::fillRect(r, at(app.limits.lo[j]), by, std::max(2.0F, at(app.limits.hi[j]) - at(app.limits.lo[j])), 9.0F, app.mode == jl::Mode::Off ? gd::Color{107, 114, 128, 90} : gd::Color{13, 148, 136, 150});
        gd::strokeRect(r, bx, by, bw, 9.0F, gd::kInk);
        gd::fillRect(r, at(jl::wrapAngle(a[j])) - 1.5F, by - 3.0F, 3.0F, 15.0F, pinned && app.mode != jl::Mode::Off ? gd::kOrange : gd::kInk);
    }

    if (app.showMap) {
        SDL_snprintf(line, sizeof line, "reached %d of %d cells", app.mapReached, app.mapReachable);
        app.hud.text(r, x, 316.0F, line, gd::kInk, gd::Hud::Style::Bold);
        gd::fillRect(r, x, 346.0F, 14.0F, 14.0F, gd::Color{13, 148, 136, 140});
        app.hud.text(r, x + 22.0F, 343.0F, "tip on target", gd::kMuted);
        gd::fillRect(r, x + 170.0F, 346.0F, 14.0F, 14.0F, gd::Color{234, 88, 12, 150});
        app.hud.text(r, x + 192.0F, 343.0F, "missed", gd::kMuted);
    } else {
        app.hud.text(r, x, 316.0F, "map off (M)", gd::kMuted);
    }
    const char* note = app.mode == jl::Mode::Off ? "no limits: any bend goes" : app.mode == jl::Mode::End ? "solve free, then clamp: misses" : "clamp each step: allowed pose";
    const bool stuck = app.mode == jl::Mode::Step && app.limitedStuck && error > kMapTol;
    if (stuck) note = "stuck: R resets the pose";
    app.hud.text(r, x, 380.0F, note, stuck ? gd::kOrange : gd::kInk);
}

void drawScene(App& app) {
    SDL_Renderer* r = app.renderer;
    refreshMap(app);
    const jl::Angles a = shown(app);
    const jl::Points p = jl::forward(kArm, a);
    const double error = jl::length(app.target - p[jl::kJoints]);

    gd::backdrop(r, kWidth, kHeight);
    app.hud.titleBar(r, kWidth, "[ JOINT LIMITS ]", modeName(app.mode));

    if (app.showMap) drawMap(app);

    // Allowed range of every joint as a wedge on its parent segment, with a handle on each edge.
    for (std::size_t j = 0; j < jl::kJoints; ++j) {
        const double dir = parentDirection(a, j);
        const bool off = app.mode == jl::Mode::Off;
        fillSector(r, p[j], kWedgeRadius, dir + app.limits.lo[j], dir + app.limits.hi[j], off ? gd::Color{107, 114, 128, 40} : gd::Color{13, 148, 136, 70});
        gd::setColor(r, off ? gd::kMuted : gd::kTealDark);
        for (const double edge : {app.limits.lo[j], app.limits.hi[j]}) {
            const jl::Vec2 e = p[j] + jl::Vec2{std::cos(dir + edge), std::sin(dir + edge)} * kWedgeRadius;
            SDL_RenderLine(r, static_cast<float>(p[j].x), static_cast<float>(p[j].y), static_cast<float>(e.x), static_cast<float>(e.y));
        }
    }

    // The ghost: where the solver without limits put the arm, the pose that clamping at the end took apart.
    if (app.mode == jl::Mode::End) drawChain(r, jl::forward(kArm, app.unlimited), gd::Color{234, 88, 12, 170}, 1);

    drawChain(r, p, gd::kInk, 2);

    if (error > kMapTol) dashedLine(r, p[jl::kJoints], app.target, gd::kOrange);

    const std::optional<Handle> hot = app.dragging ? app.dragging : handleAt(app, app.mouse);
    for (std::size_t j = 0; j < jl::kJoints; ++j) {
        const bool pinned = std::fabs(jl::wrapAngle(a[j] - app.limits.lo[j])) < kPinned || std::fabs(jl::wrapAngle(a[j] - app.limits.hi[j])) < kPinned;
        gd::disc(r, static_cast<float>(p[j].x), static_cast<float>(p[j].y), 7.0F, pinned && app.mode != jl::Mode::Off ? gd::kOrange : gd::kWhite);
        gd::ring(r, static_cast<float>(p[j].x), static_cast<float>(p[j].y), 7.0F, gd::kInk);
        for (const bool upper : {false, true}) {
            const jl::Vec2 h = handlePosition(p, a, app.limits, {j, upper});
            const bool isHot = hot && hot->joint == j && hot->upper == upper;
            gd::fillRect(r, static_cast<float>(h.x) - 5.0F, static_cast<float>(h.y) - 5.0F, 10.0F, 10.0F, isHot ? gd::kTeal : gd::kWhite);
            gd::strokeRect(r, static_cast<float>(h.x) - 5.0F, static_cast<float>(h.y) - 5.0F, 10.0F, 10.0F, gd::kInk);
        }
    }
    gd::disc(r, static_cast<float>(p[jl::kJoints].x), static_cast<float>(p[jl::kJoints].y), 5.0F, gd::kTeal);
    gd::ring(r, static_cast<float>(p[jl::kJoints].x), static_cast<float>(p[jl::kJoints].y), 5.0F, gd::kInk);

    // The target.
    const auto tx = static_cast<float>(app.target.x);
    const auto ty = static_cast<float>(app.target.y);
    gd::ring(r, tx, ty, 11.0F, error > kMapTol ? gd::kOrange : gd::kTeal);
    gd::ring(r, tx, ty, 12.0F, error > kMapTol ? gd::kOrange : gd::kTeal);
    thickLine(r, {app.target.x - 16.0, app.target.y}, {app.target.x + 16.0, app.target.y}, gd::kInk, 0);
    thickLine(r, {app.target.x, app.target.y - 16.0}, {app.target.x, app.target.y + 16.0}, gd::kInk, 0);

    drawPanel(app, a, error);

    app.hud.text(r, 40.0F, 424.0F, "mouse: target   drag a wedge edge: change a limit", gd::kMuted);
    for (const Chip& c : app.chips) {
        const bool on = active(app, c.action);
        gd::card(r, c.rect.x, c.rect.y, c.rect.w, c.rect.h, on ? gd::kTeal : gd::kWhite, gd::kInk);
        app.hud.text(r, c.rect.x + (c.rect.w - app.hud.width(c.label, gd::Hud::Style::Bold)) / 2.0F, c.rect.y + 4.0F, c.label, on ? gd::kWhite : gd::kInk, gd::Hud::Style::Bold);
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
    if (!SDL_CreateWindowAndRenderer("joint-limits", kWidth, kHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &app->window, &app->renderer)) {
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
    // The first frame already shows an arm that has settled on the first target.
    app->target = jl::autoTarget(kRoot, jl::reachMax(kArm), 0.0);
    for (int i = 0; i < 120; ++i) {
        jl::sweep(kArm, app->unlimited, app->target, nullptr);
        jl::sweep(kArm, app->limited, app->target, &app->limits);
    }
    app->lastNs = SDL_GetTicksNS();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto& app = *static_cast<App*>(appstate);
    SDL_ConvertEventToRenderCoordinates(app.renderer, event);
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_MOUSE_MOTION:
            app.mouse = toVec({event->motion.x, event->motion.y});
            if (app.dragging) {
                dragTo(app, app.mouse);
            } else if (inField(app.mouse) && !handleAt(app, app.mouse)) {
                app.autoTarget = false;
                app.target = app.mouse;
            }
            break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button != SDL_BUTTON_LEFT) break;
            const SDL_FPoint p{event->button.x, event->button.y};
            app.mouse = toVec(p);
            bool onChip = false;
            for (const Chip& c : app.chips) {
                if (SDL_PointInRectFloat(&p, &c.rect)) {
                    apply(app, c.action);
                    onChip = true;
                }
            }
            if (onChip) break;
            if (const auto h = handleAt(app, app.mouse)) {
                app.dragging = h;
            } else if (inField(app.mouse)) {
                app.autoTarget = false;
                app.target = app.mouse;
            }
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event->button.button == SDL_BUTTON_LEFT) app.dragging.reset();
            break;
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
        case SDL_EVENT_WINDOW_FOCUS_LOST: app.dragging.reset(); break;
        case SDL_EVENT_KEY_DOWN:
            if (event->key.repeat) break;  // every key here toggles or resets, none should fire while held
            switch (event->key.key) {
                case SDLK_ESCAPE: return SDL_APP_SUCCESS;
                case SDLK_1: apply(app, Action::ModeOff); break;
                case SDLK_2: apply(app, Action::ModeEnd); break;
                case SDLK_3: apply(app, Action::ModeStep); break;
                case SDLK_4: apply(app, Action::PresetArm); break;
                case SDLK_5: apply(app, Action::PresetStiff); break;
                case SDLK_M: apply(app, Action::ToggleMap); break;
                case SDLK_A: apply(app, Action::ToggleAuto); break;
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
    const Uint64 now = SDL_GetTicksNS();
    const double dt = std::clamp(static_cast<double>(now - app.lastNs) * 1e-9, 0.0, 0.25);
    app.lastNs = now;
    app.accumulator += dt;
    for (int steps = 0; app.accumulator >= kTick && steps < kMaxTicksPerFrame; ++steps) {
        tick(app);
        app.accumulator -= kTick;
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
