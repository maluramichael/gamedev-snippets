// Joint limits for a chain: the logic of the demo, no SDL, so test.cpp can check it.
//
// An arm of kJoints segments is described by angles, not by points. Joint 0 sits on the root and its angle is absolute
// (0 points along +x, positive turns clockwise on the screen because y points down), joint i > 0 sits at the end of
// segment i - 1 and its angle is relative to that segment, 0 is straight on. Because only angles change, the segment
// lengths cannot change either. Every joint has a range [lo, hi] inside [-pi, pi], a range that wraps over +-pi is not
// representable (the elbow of a human arm is 0 to 150 degrees, no joint needs more).
//
// The solver is CCD (cyclic coordinate descent): visit the joints from the tip to the root, at each one turn the
// rest of the chain so that the tip points at the target. Limits can enter in two places:
//
//   Mode::Step  every turn is clamped to the joint's range right away, so the next joint already sees the pose that
//               the arm really has, and the solver searches inside the allowed poses.
//   Mode::End   the solver runs without limits and the finished pose is clamped. Every joint is moved by the clamp,
//               and a tip does not survive that: the answer is a pose that is allowed but aims somewhere else.
//   Mode::Off   no limits at all.
//
// Beyond the reach of the unlimited arm (sum of the lengths) the best answer is known: straight at the target, short by
// distance - reach. computeMap uses that to skip the solver there.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

namespace jl {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTiny = 1e-9;     // below this a vector has no usable direction
constexpr double kStall = 1e-12;   // a sweep that moves no angle by more than this is standing still
constexpr std::size_t kJoints = 4;

struct Vec2 {
    double x, y;
};

inline Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
inline Vec2 operator*(Vec2 a, double s) { return {a.x * s, a.y * s}; }
inline double dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
inline double cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }
inline double length(Vec2 a) { return std::sqrt(dot(a, a)); }

using Angles = std::array<double, kJoints>;
using Points = std::array<Vec2, kJoints + 1>;

struct Limits {
    Angles lo, hi;
};

struct Arm {
    Vec2 root;
    std::array<double, kJoints> len;
};

enum class Mode { Off, End, Step };

/// The same direction as `a`, written in (-pi, pi]. A NaN or infinity comes out as 0.
inline double wrapAngle(double a) {
    if (!std::isfinite(a)) return 0.0;
    double r = std::fmod(a + kPi, 2.0 * kPi);
    if (r <= 0.0) r += 2.0 * kPi;
    return r - kPi;
}

/// `a` itself if it is inside [lo, hi] (compared as directions), else the nearer border, measured around the circle.
inline double clampAngle(double a, double lo, double hi) {
    const double w = wrapAngle(a);
    if (w >= lo && w <= hi) return w;
    const double toLo = std::fabs(wrapAngle(w - lo));
    const double toHi = std::fabs(wrapAngle(w - hi));
    return toLo <= toHi ? lo : hi;
}

inline Angles clampAll(const Angles& a, const Limits& l) {
    Angles out{};
    for (std::size_t i = 0; i < kJoints; ++i) out[i] = clampAngle(a[i], l.lo[i], l.hi[i]);
    return out;
}

inline bool inLimits(const Angles& a, const Limits& l, double eps = 1e-9) {
    for (std::size_t i = 0; i < kJoints; ++i) {
        const double w = wrapAngle(a[i]);
        if (w < l.lo[i] - eps || w > l.hi[i] + eps) return false;
    }
    return true;
}

/// Joint positions p[0..kJoints]: p[0] is the root, p[kJoints] the tip.
inline Points forward(const Arm& arm, const Angles& a) {
    Points p{};
    p[0] = arm.root;
    double dir = 0.0;
    for (std::size_t i = 0; i < kJoints; ++i) {
        dir += a[i];
        p[i + 1] = p[i] + Vec2{std::cos(dir), std::sin(dir)} * arm.len[i];
    }
    return p;
}

inline Vec2 tip(const Arm& arm, const Angles& a) { return forward(arm, a)[kJoints]; }

inline double tipError(const Arm& arm, const Angles& a, Vec2 target) { return length(target - tip(arm, a)); }

/// The largest distance from the root the tip can get with no limits: all segments in a line.
inline double reachMax(const Arm& arm) {
    double sum = 0.0;
    for (const double d : arm.len) sum += d;
    return sum;
}

/// The smallest distance: 0, unless one segment is longer than all the others together.
inline double reachMin(const Arm& arm) {
    double longest = 0.0;
    for (const double d : arm.len) longest = std::max(longest, d);
    return std::max(0.0, 2.0 * longest - reachMax(arm));
}

/// One CCD sweep, tip joint first. With `limits` every joint is clamped as soon as it has turned, without it the
/// joints turn freely.
inline void sweep(const Arm& arm, Angles& a, Vec2 target, const Limits* limits) {
    Points p = forward(arm, a);
    for (std::size_t k = kJoints; k-- > 0;) {
        const Vec2 toTip = p[kJoints] - p[k];
        const Vec2 toTarget = target - p[k];
        double next = a[k];
        if (length(toTip) > kTiny && length(toTarget) > kTiny) next = wrapAngle(a[k] + std::atan2(cross(toTip, toTarget), dot(toTip, toTarget)));
        if (limits != nullptr) next = clampAngle(next, limits->lo[k], limits->hi[k]);
        const double turn = wrapAngle(next - a[k]);
        a[k] = next;
        if (turn != 0.0) {
            const double cs = std::cos(turn);
            const double sn = std::sin(turn);
            for (std::size_t i = k + 1; i <= kJoints; ++i) {
                const Vec2 v = p[i] - p[k];
                p[i] = p[k] + Vec2{cs * v.x - sn * v.y, sn * v.x + cs * v.y};
            }
        }
    }
}

struct Solve {
    int iterations = 0;
    double error = 0.0;
};

/// Sweep until the tip is within `tol` of the target, no angle moves any more (stuck, or as close as the arm gets), or
/// `maxIterations` ran out. Already there counts as solved after 0 sweeps.
inline Solve solve(const Arm& arm, Angles& a, Vec2 target, const Limits* limits, double tol, int maxIterations) {
    Solve s;
    s.error = tipError(arm, a, target);
    while (s.error > tol && s.iterations < maxIterations) {
        const Angles before = a;
        sweep(arm, a, target, limits);
        ++s.iterations;
        s.error = tipError(arm, a, target);
        double moved = 0.0;
        for (std::size_t i = 0; i < kJoints; ++i) moved = std::max(moved, std::fabs(wrapAngle(a[i] - before[i])));
        if (moved < kStall) break;
    }
    return s;
}

/// The pose a mode ends up with for one target, starting from `start`.
inline Angles pose(Mode mode, const Arm& arm, const Angles& start, Vec2 target, const Limits& limits, double tol, int maxIterations) {
    Angles a = start;
    switch (mode) {
        case Mode::Off: solve(arm, a, target, nullptr, tol, maxIterations); break;
        case Mode::End:
            solve(arm, a, target, nullptr, tol, maxIterations);
            a = clampAll(a, limits);
            break;
        case Mode::Step:
            a = clampAll(a, limits);
            solve(arm, a, target, &limits, tol, maxIterations);
            break;
    }
    return a;
}

/// Moving a border handle: the mouse at angle `rel` (relative to the parent segment, in (-pi, pi]) sets the upper or the
/// lower border of `joint`. The borders never cross, and a handle dragged over +-pi stops at pi instead of jumping to the
/// other side of the circle.
inline Limits dragLimit(Limits l, std::size_t joint, bool upper, double rel) {
    double& border = upper ? l.hi[joint] : l.lo[joint];
    if (std::fabs(rel - border) > kPi) rel = border > 0.0 ? kPi : -kPi;
    border = rel;
    if (upper) l.hi[joint] = std::clamp(l.hi[joint], l.lo[joint], kPi);
    else l.lo[joint] = std::clamp(l.lo[joint], -kPi, l.hi[joint]);
    return l;
}

/// A human-ish arm: shoulder from straight up-ish to down, elbow 0 to 150 degrees, a little play in the last two.
inline Limits armLimits() {
    constexpr double d = kPi / 180.0;
    Limits l;
    l.lo = {-110.0 * d, 0.0 * d, -60.0 * d, -45.0 * d};
    l.hi = {70.0 * d, 150.0 * d, 60.0 * d, 45.0 * d};
    return l;
}

/// Every joint only +-30 degrees.
inline Limits stiffLimits() {
    constexpr double d = kPi / 180.0;
    Limits l;
    l.lo = {-30.0 * d, -30.0 * d, -30.0 * d, -30.0 * d};
    l.hi = {30.0 * d, 30.0 * d, 30.0 * d, 30.0 * d};
    return l;
}

/// The pose the arm starts in and returns to: a gentle bend, forced into the limits.
inline Angles restPose(const Limits& l) {
    constexpr double d = kPi / 180.0;
    return clampAll({{-20.0 * d, 45.0 * d, 15.0 * d, 10.0 * d}}, l);
}

/// A grid of targets over the plane. error[row * cols + col] is the tip error for the target at the cell's centre.
struct Map {
    Vec2 origin{0, 0};  // top left corner of the cell (0, 0)
    double cell = 1.0;
    int cols = 0;
    int rows = 0;
    std::vector<float> error;
};

inline Map makeMap(Vec2 origin, double cell, int cols, int rows) {
    Map m;
    m.origin = origin;
    m.cell = cell;
    m.cols = cols;
    m.rows = rows;
    m.error.assign(static_cast<std::size_t>(cols) * static_cast<std::size_t>(rows), 0.0F);
    return m;
}

inline Vec2 cellCenter(const Map& m, int col, int row) {
    return {m.origin.x + (static_cast<double>(col) + 0.5) * m.cell, m.origin.y + (static_cast<double>(row) + 0.5) * m.cell};
}

/// Solve every cell's target from `rest` with the given mode. Beyond the reach of the unlimited arm no solver is run, the
/// answer there is known: distance - reach. A cell closer than reachMin gets the error of the closest the arm can get.
inline void computeMap(Map& m, Mode mode, const Arm& arm, const Angles& rest, const Limits& limits, double tol, int maxIterations) {
    const double maxR = reachMax(arm);
    const double minR = reachMin(arm);
    for (int row = 0; row < m.rows; ++row) {
        for (int col = 0; col < m.cols; ++col) {
            const Vec2 t = cellCenter(m, col, row);
            const double dist = length(t - arm.root);
            double err;
            if (dist > maxR) {
                err = dist - maxR;
            } else if (dist < minR) {
                err = minR - dist;
            } else {
                err = tipError(arm, pose(mode, arm, rest, t, limits, tol, maxIterations), t);
            }
            m.error[static_cast<std::size_t>(row) * static_cast<std::size_t>(m.cols) + static_cast<std::size_t>(col)] = static_cast<float>(err);
        }
    }
}

inline int reachedCells(const Map& m, double tol) {
    int n = 0;
    for (const float e : m.error) n += static_cast<double>(e) <= tol ? 1 : 0;
    return n;
}

/// Cells whose centre lies in the ring the unlimited arm can reach.
inline int reachableCells(const Map& m, const Arm& arm) {
    int n = 0;
    for (int row = 0; row < m.rows; ++row) {
        for (int col = 0; col < m.cols; ++col) {
            const double dist = length(cellCenter(m, col, row) - arm.root);
            n += (dist >= reachMin(arm) && dist <= reachMax(arm)) ? 1 : 0;
        }
    }
    return n;
}

// The demo path: the target jumps between spots around the root, one every kAutoHold seconds. Each entry is an angle in
// radians (screen coordinates) and a distance as a fraction of the reach. Some of them the limited arm cannot reach.
constexpr int kAutoSpots = 8;
constexpr double kAutoHold = 2.5;
constexpr double kAutoPeriod = kAutoHold * kAutoSpots;

inline Vec2 autoTarget(Vec2 root, double reach, double phase) {
    struct Spot {
        double angle;
        double fraction;
    };
    constexpr std::array<Spot, kAutoSpots> spots{{{-1.1, 0.8}, {0.5, 0.9}, {-2.3, 0.8}, {2.4, 0.7}, {-1.7, 0.75}, {1.3, 0.8}, {3.0, 0.9}, {-0.2, 0.6}}};
    const int index = static_cast<int>(std::min(std::max(0.0, phase / kAutoHold), static_cast<double>(kAutoSpots - 1)));  // max() also takes a NaN to 0
    const Spot s = spots[static_cast<std::size_t>(index)];
    return root + Vec2{std::cos(s.angle), std::sin(s.angle)} * (reach * s.fraction);
}

/// Phase advanced by dt seconds, kept in [0, kAutoPeriod) so it never grows.
inline double advancePhase(double phase, double dt) {
    const double next = std::fmod(phase + dt, kAutoPeriod);
    return next < 0.0 ? next + kAutoPeriod : next;
}

}  // namespace jl
