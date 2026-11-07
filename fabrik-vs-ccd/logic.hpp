// FABRIK vs CCD: the logic of the demo, no SDL, so test.cpp can check it.
//
// A chain is a list of joints p[0..n] joined by n segments of fixed length d[i] (p[i] to p[i+1]). p[0] is the root and
// stays where it is, the target t is where the tip p[n] should go. All three solvers repeat one "iteration" until the
// tip is close enough:
//
//   CCD         visit the joints from the tip to the root, at each one rotate everything behind it so that the tip
//               points at the target (an angle from a dot and a cross product). Lengths survive because it only rotates.
//   FABRIK      put the tip on the target, then walk to the root and pull each joint onto the line to its successor at
//               the right distance (backward pass), pin the root, walk to the tip doing the same (forward pass).
//               Lengths survive because every joint is placed at exactly d[i] from its neighbour.
//   Relaxation  pin the tip to the target, then walk to the root and move both ends of each segment half the length
//               error towards each other. Lengths are only approximately right until the pose has settled.
//
// A target beyond the reach (sum of the lengths) has one answer: the chain straight at the target. FABRIK gets there in
// one iteration, p(i+1) = (1 - k_i) p_i + k_i t with k_i = d_i / |t - p_i|, CCD creeps towards it. Relaxation never
// gets there: with the root and the tip both pinned, the only way to make the two ends meet is to stretch the segments.
//
// Coordinates are screen coordinates, y points down. Points that coincide have no direction: CCD skips such a joint,
// relaxation skips such a segment, FABRIK puts the point along +x.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <initializer_list>

namespace fk {

struct Vec2 {
    double x, y;
};

inline Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
inline Vec2 operator*(Vec2 a, double s) { return {a.x * s, a.y * s}; }
inline double dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
inline double cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }
inline double length(Vec2 a) { return std::sqrt(dot(a, a)); }

constexpr int kMaxSegments = 12;
constexpr double kTiny = 1e-9;          // below this a vector has no usable direction
constexpr double kStall = 1e-7;         // a pose that moves less than this in one iteration is standing still
constexpr double kPi = 3.14159265358979323846;
constexpr int kAutoSpots = 8;           // targets of the demo path
constexpr double kAutoHold = 2.0;       // seconds the target stays on each of them
constexpr double kAutoPeriod = kAutoHold * kAutoSpots;  // seconds for one round of the demo path

using Pose = std::array<Vec2, kMaxSegments + 1>;

struct Chain {
    int n = 0;                                   // number of segments, the joints are p[0..n]
    Pose p{};                                    // p[0] is the root, p[n] the tip
    std::array<double, kMaxSegments> d{};        // d[i] is the length between p[i] and p[i + 1]
};

enum class Algo { Ccd, Fabrik, Relax };

/// The poses inside one iteration: the one before it, then one after every step. Fixed storage, no allocation per frame.
struct Trace {
    static constexpr int kMax = kMaxSegments + 3;
    int count = 0;
    std::array<Pose, kMax> poses{};
    void clear() { count = 0; }
    void push(const Chain& c) {
        if (count < kMax) poses[static_cast<std::size_t>(count++)] = c.p;
    }
};

/// A straight chain from `root` along +x with the given segment lengths, each segment turned by `bend` radians against
/// the one before (positive turns towards +y, which is down on the screen). 1 to kMaxSegments segments.
inline Chain makeChain(Vec2 root, std::initializer_list<double> lengths, double bend = 0.0) {
    Chain c;
    c.p[0] = root;
    double angle = 0.0;
    for (const double len : lengths) {
        if (c.n >= kMaxSegments) break;
        const auto i = static_cast<std::size_t>(c.n);
        c.d[i] = len;
        c.p[i + 1] = c.p[i] + Vec2{std::cos(angle), std::sin(angle)} * len;
        angle += bend;
        ++c.n;
    }
    return c;
}

/// n segments of the same length.
inline Chain makeUniformChain(Vec2 root, int segments, double len, double bend = 0.0) {
    Chain c;
    c.p[0] = root;
    c.n = std::clamp(segments, 1, kMaxSegments);
    double angle = 0.0;
    for (int i = 0; i < c.n; ++i) {
        const auto k = static_cast<std::size_t>(i);
        c.d[k] = len;
        c.p[k + 1] = c.p[k] + Vec2{std::cos(angle), std::sin(angle)} * len;
        angle += bend;
    }
    return c;
}

inline Vec2 root(const Chain& c) { return c.p[0]; }
inline Vec2 tip(const Chain& c) { return c.p[static_cast<std::size_t>(c.n)]; }

/// The largest distance from the root the tip can get: all segments in a line.
inline double reach(const Chain& c) {
    double sum = 0.0;
    for (int i = 0; i < c.n; ++i) sum += c.d[static_cast<std::size_t>(i)];
    return sum;
}

/// The smallest distance from the root the tip can get: 0, unless one segment is longer than all the others together.
inline double minReach(const Chain& c) {
    double longest = 0.0;
    for (int i = 0; i < c.n; ++i) longest = std::max(longest, c.d[static_cast<std::size_t>(i)]);
    return std::max(0.0, 2.0 * longest - reach(c));
}

/// Distance from the tip to the target.
inline double tipError(const Chain& c, Vec2 target) { return length(target - tip(c)); }

/// The smallest tip error any pose can have: 0 inside the reachable ring around the root, else the distance to it.
inline double bestError(const Chain& c, Vec2 target) {
    const double dist = length(target - root(c));
    return std::max({0.0, dist - reach(c), minReach(c) - dist});
}

/// The largest deviation of a segment from its length. CCD and FABRIK keep it at rounding level, relaxation does not.
inline double lengthError(const Chain& c) {
    double worst = 0.0;
    for (int i = 0; i < c.n; ++i) {
        const auto k = static_cast<std::size_t>(i);
        worst = std::max(worst, std::fabs(length(c.p[k + 1] - c.p[k]) - c.d[k]));
    }
    return worst;
}

/// The tip is as close to the target as the chain allows, within `tol`, and no segment is off its length by more than
/// `tol` (relaxation can "reach" a far target by stretching its segments, that does not count).
inline bool reached(const Chain& c, Vec2 target, double tol) { return tipError(c, target) <= bestError(c, target) + tol && lengthError(c) <= tol; }

/// The angle at joint j (1..n-1) between segment j-1 and segment j, in (-pi, pi]. 0 is straight.
inline double bendAt(const Chain& c, int j) {
    const auto k = static_cast<std::size_t>(j);
    const Vec2 a = c.p[k] - c.p[k - 1];
    const Vec2 b = c.p[k + 1] - c.p[k];
    return std::atan2(cross(a, b), dot(a, b));
}

/// The sum of all joint bends, a measure for how curled up the chain is.
inline double totalBend(const Chain& c) {
    double sum = 0.0;
    for (int j = 1; j < c.n; ++j) sum += std::fabs(bendAt(c, j));
    return sum;
}

/// The largest distance any joint moved between two poses of n + 1 joints.
inline double maxShift(const Pose& a, const Pose& b, int n) {
    double worst = 0.0;
    for (int i = 0; i <= n; ++i) worst = std::max(worst, length(a[static_cast<std::size_t>(i)] - b[static_cast<std::size_t>(i)]));
    return worst;
}

/// One CCD sweep, tip joint first: at joint j everything behind it turns by the angle that points the tip at the target.
inline void ccdIteration(Chain& c, Vec2 target, Trace* trace = nullptr) {
    if (trace != nullptr) {
        trace->clear();
        trace->push(c);
    }
    for (int j = c.n - 1; j >= 0; --j) {
        const auto k = static_cast<std::size_t>(j);
        const Vec2 toTip = tip(c) - c.p[k];
        const Vec2 toTarget = target - c.p[k];
        if (length(toTip) > kTiny && length(toTarget) > kTiny) {
            const double angle = std::atan2(cross(toTip, toTarget), dot(toTip, toTarget));
            const double cs = std::cos(angle);
            const double sn = std::sin(angle);
            for (int i = j + 1; i <= c.n; ++i) {
                const Vec2 v = c.p[static_cast<std::size_t>(i)] - c.p[k];
                c.p[static_cast<std::size_t>(i)] = c.p[k] + Vec2{cs * v.x - sn * v.y, sn * v.x + cs * v.y};
            }
        }
        if (trace != nullptr) trace->push(c);
    }
}

/// One FABRIK iteration: backward pass from the target, forward pass from the root. A target beyond the reach is
/// answered with the straight line in one go.
inline void fabrikIteration(Chain& c, Vec2 target, Trace* trace = nullptr) {
    if (trace != nullptr) {
        trace->clear();
        trace->push(c);
    }
    const Vec2 base = root(c);
    if (length(target - base) >= reach(c)) {
        for (int i = 0; i < c.n; ++i) {
            const auto k = static_cast<std::size_t>(i);
            const Vec2 toTarget = target - c.p[k];
            const double dist = length(toTarget);  // positive unless the whole chain has no length
            c.p[k + 1] = dist > kTiny ? c.p[k] + toTarget * (c.d[k] / dist) : c.p[k];
        }
        if (trace != nullptr) trace->push(c);
        return;
    }
    // Backward: the tip on the target, every joint pulled onto the line to its successor at distance d[i].
    c.p[static_cast<std::size_t>(c.n)] = target;
    for (int i = c.n - 1; i >= 0; --i) {
        const auto k = static_cast<std::size_t>(i);
        const double r = length(c.p[k + 1] - c.p[k]);
        c.p[k] = r > kTiny ? c.p[k + 1] * (1.0 - c.d[k] / r) + c.p[k] * (c.d[k] / r) : c.p[k + 1] + Vec2{c.d[k], 0.0};
    }
    if (trace != nullptr) trace->push(c);
    // Forward: the root back where it belongs, every joint pulled onto the line to its predecessor.
    c.p[0] = base;
    for (int i = 0; i < c.n; ++i) {
        const auto k = static_cast<std::size_t>(i);
        const double r = length(c.p[k + 1] - c.p[k]);
        c.p[k + 1] = r > kTiny ? c.p[k] * (1.0 - c.d[k] / r) + c.p[k + 1] * (c.d[k] / r) : c.p[k] + Vec2{c.d[k], 0.0};
    }
    if (trace != nullptr) trace->push(c);
}

/// One relaxation pass: pin the tip to the target, then give every segment a half-half length correction, root last.
inline void relaxIteration(Chain& c, Vec2 target, Trace* trace = nullptr) {
    if (trace != nullptr) {
        trace->clear();
        trace->push(c);
    }
    c.p[static_cast<std::size_t>(c.n)] = target;
    if (trace != nullptr) trace->push(c);
    for (int i = c.n - 1; i >= 0; --i) {
        const auto k = static_cast<std::size_t>(i);
        const Vec2 span = c.p[k + 1] - c.p[k];
        const double r = length(span);
        if (r > kTiny) {
            const Vec2 dir = span * (1.0 / r);
            if (i == 0) {
                c.p[1] = c.p[0] + dir * c.d[0];  // the root stays, only its neighbour can give way
            } else {
                const double half = (r - c.d[k]) / 2.0;
                c.p[k] = c.p[k] + dir * half;
                c.p[k + 1] = c.p[k + 1] - dir * half;
            }
        }
        if (trace != nullptr) trace->push(c);
    }
}

inline void iterate(Algo algo, Chain& c, Vec2 target, Trace* trace = nullptr) {
    switch (algo) {
        case Algo::Ccd: ccdIteration(c, target, trace); break;
        case Algo::Fabrik: fabrikIteration(c, target, trace); break;
        case Algo::Relax: relaxIteration(c, target, trace); break;
    }
}

enum class Outcome { Reached, Stuck, Limit };

struct Result {
    int iterations = 0;
    Outcome outcome = Outcome::Limit;
    double error = 0.0;
};

/// Iterate until the tip is within `tol` of the best the chain can do (Reached), the pose stops moving without being
/// there (Stuck), or `maxIterations` ran out (Limit). Already there counts as Reached after 0 iterations.
inline Result solve(Algo algo, Chain& c, Vec2 target, double tol, int maxIterations) {
    Result r;
    r.error = tipError(c, target);
    if (reached(c, target, tol)) {
        r.outcome = Outcome::Reached;
        return r;
    }
    while (r.iterations < maxIterations) {
        const Pose before = c.p;
        iterate(algo, c, target);
        ++r.iterations;
        r.error = tipError(c, target);
        if (reached(c, target, tol)) {
            r.outcome = Outcome::Reached;
            return r;
        }
        if (maxShift(before, c.p, c.n) < kStall) {
            r.outcome = Outcome::Stuck;
            return r;
        }
    }
    r.outcome = Outcome::Limit;
    return r;
}

/// The demo path: the target jumps between eight spots around the root, one every kAutoHold seconds. A jump is what
/// makes the solvers visible, the chain has to travel. Three spots lie beyond the reach, the first one (phase 0) is
/// well inside it. Each entry is an angle in radians (screen coordinates) and a distance as a fraction of the reach.
inline Vec2 autoTarget(Vec2 rootPos, double reachLength, double phase) {
    struct Spot {
        double angle;
        double fraction;
    };
    constexpr std::array<Spot, kAutoSpots> spots{{{-0.6, 0.85}, {2.2, 0.7}, {-2.2, 0.95}, {0.5, 1.3}, {-1.6, 0.6}, {kPi, 1.25}, {1.2, 0.9}, {-0.2, 1.15}}};
    const int index = static_cast<int>(std::min(std::max(0.0, phase / kAutoHold), static_cast<double>(kAutoSpots - 1)));  // max() also takes a NaN to 0
    const Spot s = spots[static_cast<std::size_t>(index)];
    return rootPos + Vec2{std::cos(s.angle), std::sin(s.angle)} * (reachLength * s.fraction);
}

/// Phase advanced by dt seconds, kept in [0, kAutoPeriod) so it never grows.
inline double advancePhase(double phase, double dt) {
    const double next = std::fmod(phase + dt, kAutoPeriod);
    return next < 0.0 ? next + kAutoPeriod : next;
}

}  // namespace fk
