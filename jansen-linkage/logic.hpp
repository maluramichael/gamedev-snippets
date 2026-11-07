// Jansen's linkage as forward kinematics: the logic of the demo, no SDL, so test.cpp can check it.
//
// One crank turns. Every other joint is placed by two rod lengths alone: it is the intersection of a circle around one
// joint with a circle around another, so no joint ever gets an angle of its own. The joints are solved in a fixed order
//
//     B = circles around pivot (b) and crank tip (j)      C = pivot (d) and B (e)
//     D = pivot (c) and crank tip (k)                     E = C (f) and D (g)
//     foot = D (i) and E (h)
//
// and the foot has to follow from the crank angle alone: one degree of freedom.
//
// THE CATCH (the whole trick): two circles meet in TWO points. A joint has to pick one, and a leg with five joints can
// be assembled in 2^5 = 32 ways. circleCircle takes a `side`: +1 is the intersection to the LEFT of the line from the
// first center to the second, -1 the one on the right. The sides that make Jansen's leg are read once from a known pose
// (referenceSides) and then kept for every crank angle. Taking "the first intersection" at every joint (naiveSides)
// assembles just as well but gives a different, wrong leg.
//
// Lengths: the table "Jansen Linkage Ratios" on en.wikipedia.org/wiki/Jansen%27s_linkage, which count in units of the
// crank (15). Which rod joins which joints and the start pose come from the Wikipedia drawing
// "Strandbeest Leg Proportions" (SVG), and the distances in that drawing reproduce the table, see test.cpp.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

namespace jl {

constexpr double kTau = 6.283185307179586;

struct Vec2 {
    double x, y;
};

/// The 13 lengths of the table. a and l place the crank center relative to the pivot (frame), the other eleven are rods:
/// m is the crank, b to k connect joints.
struct Lengths {
    double a, b, c, d, e, f, g, h, i, j, k, l, m;
};

inline Lengths standardLengths() { return {38.0, 41.5, 39.3, 40.1, 55.8, 39.4, 36.7, 65.7, 49.0, 50.0, 61.9, 7.8, 15.0}; }

constexpr int kRodCount = 11;
/// One factor per rod in the order of kLinks (b c d e f g h i j k m). 1.0 is the length of the table.
using Scale = std::array<double, kRodCount>;

inline Scale unitScale() {
    Scale s;
    s.fill(1.0);
    return s;
}

/// Joints of one pose. pivot and hub are fixed to the frame, crank is the tip of the crank.
struct Pose {
    Vec2 pivot, hub, crank, b, c, d, e, foot;
};

/// One rod: its letter in the table, the two joints it connects and its length.
struct Link {
    char name;
    Vec2 Pose::*from;
    Vec2 Pose::*to;
    double Lengths::*length;
};

inline constexpr std::array<Link, kRodCount> kLinks{{
    {'b', &Pose::pivot, &Pose::b, &Lengths::b},
    {'c', &Pose::pivot, &Pose::d, &Lengths::c},
    {'d', &Pose::pivot, &Pose::c, &Lengths::d},
    {'e', &Pose::b, &Pose::c, &Lengths::e},
    {'f', &Pose::c, &Pose::e, &Lengths::f},
    {'g', &Pose::d, &Pose::e, &Lengths::g},
    {'h', &Pose::e, &Pose::foot, &Lengths::h},
    {'i', &Pose::d, &Pose::foot, &Lengths::i},
    {'j', &Pose::crank, &Pose::b, &Lengths::j},
    {'k', &Pose::crank, &Pose::d, &Lengths::k},
    {'m', &Pose::hub, &Pose::crank, &Lengths::m},
}};

inline Lengths scaled(const Scale& scale) {
    Lengths out = standardLengths();
    for (std::size_t i = 0; i < kLinks.size(); ++i) out.*kLinks[i].length *= scale[i];
    return out;
}

/// Side of the point p relative to the line c0 -> c1: +1 left, -1 right. On the line itself both intersections of two
/// circles coincide, so the answer does not matter there, +1 by convention.
inline int sideOf(Vec2 c0, Vec2 c1, Vec2 p) {
    const double cross = (c1.x - c0.x) * (p.y - c0.y) - (c1.y - c0.y) * (p.x - c0.x);
    return cross >= 0.0 ? +1 : -1;
}

/// Intersection of the circle (c0, r0) with the circle (c1, r1) on the given side (see sideOf). Empty when there is no
/// single answer: the circles are too far apart, one lies inside the other, they share a center (none or infinitely
/// many points), or a radius is negative. Touching circles give their one point.
inline std::optional<Vec2> circleCircle(Vec2 c0, double r0, Vec2 c1, double r1, int side) {
    if (r0 < 0.0 || r1 < 0.0) return std::nullopt;
    const double dx = c1.x - c0.x;
    const double dy = c1.y - c0.y;
    const double d = std::hypot(dx, dy);
    if (d == 0.0) return std::nullopt;
    if (d > r0 + r1 || d < std::fabs(r0 - r1)) return std::nullopt;
    const double along = (r0 * r0 - r1 * r1 + d * d) / (2.0 * d);  // from c0 to the chord between the two points
    // The checks above passed, so a negative value here is only rounding (touching circles): clamp it.
    const double half = std::sqrt(std::max(0.0, r0 * r0 - along * along));
    const double s = side >= 0 ? 1.0 : -1.0;
    return Vec2{c0.x + (along * dx - s * half * dy) / d, c0.y + (along * dy + s * half * dx) / d};
}

/// Which side each joint takes: B, C, D, E, foot.
using Sides = std::array<int, 5>;

inline Sides naiveSides() { return {+1, +1, +1, +1, +1}; }

/// The start pose, read off the Wikipedia drawing (origin at the pivot, y up, two decimals). The foot hangs 84 below.
inline Pose referencePose() {
    return {{0.0, 0.0}, {38.0, 7.8}, {52.68, 4.73}, {15.73, 38.41}, {-36.04, 17.58}, {8.29, -38.42}, {-23.27, -19.70}, {-9.72, -83.98}};
}

inline double referenceAngle() {
    const Pose p = referencePose();
    return std::atan2(p.crank.y - p.hub.y, p.crank.x - p.hub.x);
}

/// The sides of the reference pose, in the order and with the circle centers solve() uses.
inline Sides referenceSides() {
    const Pose p = referencePose();
    return {sideOf(p.pivot, p.crank, p.b), sideOf(p.pivot, p.b, p.c), sideOf(p.pivot, p.crank, p.d), sideOf(p.c, p.d, p.e), sideOf(p.d, p.e, p.foot)};
}

struct Solved {
    bool ok = false;
    Pose pose{};     // complete only when ok
    int failed = -1;  // first joint without a solution (0 B, 1 C, 2 D, 3 E, 4 foot), -1 when ok
};

/// All joints for one crank angle (radians, 0 = crank pointing right). Fails when some pair of circles does not meet:
/// the rods are then too short or too long for that crank angle and the linkage cannot be there.
inline Solved fail(Solved& s, int joint) {
    s.failed = joint;
    return s;
}

inline Solved solve(const Lengths& L, const Sides& sides, double angle) {
    Solved out;
    Pose& p = out.pose;
    p.pivot = {0.0, 0.0};
    p.hub = {L.a, L.l};
    p.crank = {L.a + L.m * std::cos(angle), L.l + L.m * std::sin(angle)};
    const auto b = circleCircle(p.pivot, L.b, p.crank, L.j, sides[0]);
    if (!b) return fail(out, 0);
    p.b = *b;
    const auto c = circleCircle(p.pivot, L.d, p.b, L.e, sides[1]);
    if (!c) return fail(out, 1);
    p.c = *c;
    const auto d = circleCircle(p.pivot, L.c, p.crank, L.k, sides[2]);
    if (!d) return fail(out, 2);
    p.d = *d;
    const auto e = circleCircle(p.c, L.f, p.d, L.g, sides[3]);
    if (!e) return fail(out, 3);
    p.e = *e;
    const auto foot = circleCircle(p.d, L.i, p.e, L.h, sides[4]);
    if (!foot) return fail(out, 4);
    p.foot = *foot;
    out.ok = true;
    return out;
}

inline double distance(Vec2 a, Vec2 b) { return std::hypot(a.x - b.x, a.y - b.y); }

/// Largest difference between a rod's length in the pose and its length in `L`. Zero (to rounding) for a solved pose.
inline double maxRodError(const Lengths& L, const Pose& pose) {
    double worst = 0.0;
    for (const Link& link : kLinks) worst = std::max(worst, std::fabs(distance(pose.*link.from, pose.*link.to) - L.*link.length));
    return worst;
}

inline double wrapAngle(double angle) {
    double a = std::fmod(angle, kTau);
    if (a < 0.0) a += kTau;
    return a >= kTau ? 0.0 : a;  // a tiny negative angle plus kTau rounds up to kTau itself
}

/// The foot position for `samples` crank angles spread over one turn, empty where the linkage cannot assemble.
using Path = std::vector<std::optional<Vec2>>;

inline Path footPath(const Lengths& L, const Sides& sides, int samples) {
    Path path;
    path.reserve(static_cast<std::size_t>(std::max(samples, 0)));
    for (int k = 0; k < samples; ++k) {
        const Solved s = solve(L, sides, kTau * k / samples);
        path.push_back(s.ok ? std::optional<Vec2>(s.pose.foot) : std::nullopt);
    }
    return path;
}

/// True when the crank can go all the way round: every sample assembled.
inline bool fullTurn(const Path& path) {
    return !path.empty() && std::all_of(path.begin(), path.end(), [](const std::optional<Vec2>& p) { return p.has_value(); });
}

/// The lowest y of the path, the ground the foot walks on.
inline std::optional<double> groundLevel(const Path& path) {
    std::optional<double> low;
    for (const auto& p : path) {
        if (p && (!low || p->y < *low)) low = p->y;
    }
    return low;
}

/// Share of the crank turn the foot spends strictly less than `tolerance` above the ground. Needs a full turn.
inline std::optional<double> groundContact(const Path& path, double tolerance) {
    if (!fullTurn(path)) return std::nullopt;
    const double ground = *groundLevel(path);
    const auto touching = std::count_if(path.begin(), path.end(), [&](const std::optional<Vec2>& p) { return p->y < ground + tolerance; });
    return static_cast<double>(touching) / static_cast<double>(path.size());
}

constexpr double kSubstep = 0.5 * kTau / 360.0;  // half a degree: no gap in the reachable angles can hide between two steps

/// Turns the crank by `delta` radians (either way). Stops at the last angle that still assembles and returns false when
/// it had to stop, true when it got through. `angle` is wrapped to [0, 2 pi).
inline bool advance(const Lengths& L, const Sides& sides, double& angle, double delta) {
    const int steps = std::max(1, static_cast<int>(std::ceil(std::fabs(delta) / kSubstep)));
    const double h = delta / steps;
    double a = angle;
    for (int i = 0; i < steps; ++i) {
        if (!solve(L, sides, a + h).ok) {
            angle = wrapAngle(a);
            return false;
        }
        a += h;
    }
    angle = wrapAngle(a);
    return true;
}

/// The angle closest to `angle` (either side, in half degree steps) where the linkage assembles. Empty when it assembles
/// nowhere.
inline std::optional<double> nearestReachable(const Lengths& L, const Sides& sides, double angle) {
    const double base = wrapAngle(angle);
    if (solve(L, sides, base).ok) return base;
    const int halfTurnSteps = static_cast<int>(std::lround(0.5 * kTau / kSubstep));
    for (int k = 1; k <= halfTurnSteps; ++k) {
        for (const double sign : {+1.0, -1.0}) {
            const double a = base + sign * k * kSubstep;
            if (solve(L, sides, a).ok) return wrapAngle(a);
        }
    }
    return std::nullopt;
}

}  // namespace jl
