// Two-bone IK: the logic of the demo, no SDL, so test.cpp can check it.
//
// Two segments d1 (root to joint) and d2 (joint to end) and a target at distance h from the root form a triangle with
// the known sides d1, d2 and h. The law of cosines gives its angles without any iteration:
//
//     d2^2 = d1^2 + h^2 - 2 d1 h cos(alpha)    ->  cos(alpha) = (d1^2 + h^2 - d2^2) / (2 d1 h)    angle at the root
//     h^2  = d1^2 + d2^2 - 2 d1 d2 cos(gamma)  ->  cos(gamma) = (d1^2 + d2^2 - h^2) / (2 d1 d2)   angle at the joint
//
// alpha is measured between the line root -> target and the first segment. The joint sits at
// root + dir * d1 cos(alpha) +- left * d1 sin(alpha), the plus/minus is the choice of knee side (two solutions).
//
// A triangle with these sides exists only for |d1 - d2| <= h <= d1 + d2, a ring around the root. The textbook formula
// takes acos of a number beyond [-1, 1] outside of it and returns NaN. Here the arm is stretched (too far) or folded
// (too close) straight towards the target instead, which puts the end on the reachable point nearest to the target.
//
// Coordinates are screen coordinates, y points down: "left" is the left of root -> target as seen on screen.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>

namespace ik {

struct Vec2 {
    double x, y;
};

inline Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
inline Vec2 operator*(Vec2 a, double s) { return {a.x * s, a.y * s}; }
inline double dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
inline double length(Vec2 a) { return std::sqrt(dot(a, a)); }

enum class Bend { Left, Right };
enum class Reach { InRange, TooFar, TooClose };

struct Pose {
    Vec2 joint;
    Vec2 end;
    double h;         // distance root -> target
    double cosAlpha;  // the raw argument of acos, before any clamping (beyond [-1, 1] means unreachable)
    double alpha;     // angle at the root between root -> target and the first segment, 0..pi
    double gamma;     // angle at the joint between the two segments, 0 (folded) .. pi (straight)
    Reach reach;
};

constexpr int kInfinite = -1;          // solutionCount: equal lengths and the target on the root
constexpr double kMinLength = 1e-6;    // segments shorter than this are lengthened, so no division by zero below
constexpr double kPi = 3.14159265358979323846;
constexpr double kAutoPeriod = 12.0;   // seconds for one round of the demo path

namespace detail {

inline double relEps(double d1, double d2) { return 1e-9 * (d1 + d2); }

inline Vec2 leftOf(Vec2 dir) { return {dir.y, -dir.x}; }  // y points down: (1,0) -> (0,-1) is up, the left of "east"

inline double cosAlphaRaw(double d1, double d2, double h) { return (d1 * d1 + h * h - d2 * d2) / (2.0 * d1 * h); }

inline double clampUnit(double v) { return std::clamp(v, -1.0, 1.0); }

inline Reach classify(double h, double d1, double d2) {
    const double eps = relEps(d1, d2);
    if (h > d1 + d2 + eps) return Reach::TooFar;
    if (h < std::fabs(d1 - d2) - eps) return Reach::TooClose;
    return Reach::InRange;
}

/// Direction root -> target, a fixed fallback when the target sits on the root (no direction exists there).
inline Vec2 direction(Vec2 root, Vec2 target, double h) {
    if (h < 1e-12) return {1.0, 0.0};
    return (target - root) * (1.0 / h);
}

}  // namespace detail

/// The smallest and the largest distance from the root the end can reach.
inline double minReach(double d1, double d2) { return std::fabs(d1 - d2); }
inline double maxReach(double d1, double d2) { return d1 + d2; }

/// How many poses put the end exactly on the target: 2 inside the ring, 1 on its border, 0 outside (or in the hole).
/// Equal lengths with the target on the root fold the arm onto itself in every direction: kInfinite.
inline int solutionCount(double h, double d1, double d2) {
    const double eps = detail::relEps(d1, d2);
    if (std::fabs(d1 - d2) <= eps && h <= eps) return kInfinite;
    if (h > d1 + d2 + eps || h < std::fabs(d1 - d2) - eps) return 0;
    if (std::fabs(h - (d1 + d2)) <= eps || std::fabs(h - std::fabs(d1 - d2)) <= eps) return 1;
    return 2;
}

/// The two-bone solution. Never returns NaN: out of range the arm is stretched or folded towards the target.
inline Pose solve(Vec2 root, Vec2 target, double d1, double d2, Bend bend) {
    d1 = std::max(d1, kMinLength);
    d2 = std::max(d2, kMinLength);
    const double h = length(target - root);
    const Vec2 dir = detail::direction(root, target, h);
    const Reach reach = detail::classify(h, d1, d2);
    const double raw = h >= 1e-12 ? detail::cosAlphaRaw(d1, d2, h) : 1.0;

    if (reach == Reach::TooFar) {
        const Vec2 joint = root + dir * d1;
        return {joint, joint + dir * d2, h, raw, 0.0, kPi, reach};
    }
    // Folded: the lower segment lies on the upper one. The upper one points at the target if it is the longer one,
    // away from it otherwise, so that the end lands on the line towards the target at distance |d1 - d2|.
    if (reach == Reach::TooClose || h < 1e-12) {
        const double side = d1 >= d2 ? 1.0 : -1.0;
        const Vec2 joint = root + dir * (side * d1);
        return {joint, joint - dir * (side * d2), h, raw, d1 >= d2 ? 0.0 : kPi, 0.0, reach};
    }

    // In range. The clamp only absorbs rounding on the border: cos may come out as 1.0000000000000002 there.
    const double cosAlpha = detail::clampUnit(raw);
    const double alpha = std::acos(cosAlpha);
    const double sgn = bend == Bend::Left ? 1.0 : -1.0;
    const Vec2 joint = root + dir * (d1 * cosAlpha) + detail::leftOf(dir) * (sgn * d1 * std::sin(alpha));
    // The end sits on the target; going through the joint keeps the second segment at exactly d2.
    const Vec2 toTarget = target - joint;
    const double lowerLen = length(toTarget);
    const Vec2 end = lowerLen > 0.0 ? joint + toTarget * (d2 / lowerLen) : joint + dir * d2;
    const double gamma = std::acos(detail::clampUnit((d1 * d1 + d2 * d2 - h * h) / (2.0 * d1 * d2)));
    return {joint, end, h, raw, alpha, gamma, reach};
}

/// The textbook version without any check. Identical inside the ring, NaN (or infinity) outside, in the hole and on the root.
inline Pose solveNaive(Vec2 root, Vec2 target, double d1, double d2, Bend bend) {
    const double h = length(target - root);
    const Vec2 dir = (target - root) * (1.0 / h);  // h = 0 divides by zero here, on purpose
    const double raw = detail::cosAlphaRaw(d1, d2, h);
    const double alpha = std::acos(raw);
    const double sgn = bend == Bend::Left ? 1.0 : -1.0;
    const Vec2 joint = root + dir * (d1 * std::cos(alpha)) + detail::leftOf(dir) * (sgn * d1 * std::sin(alpha));
    const Vec2 toTarget = target - joint;
    const Vec2 end = joint + toTarget * (d2 / length(toTarget));
    const double gamma = std::acos((d1 * d1 + d2 * d2 - h * h) / (2.0 * d1 * d2));
    return {joint, end, h, raw, alpha, gamma, detail::classify(h, d1, d2)};
}

struct Span {
    double x0, x1;
};

struct RowSpans {
    std::array<Span, 2> s;
    int n;
};

/// The ring (inner radius rInner, outer radius rOuter, around center) cut by the horizontal line at y: none, one or two spans.
inline RowSpans ringSpans(Vec2 center, double rInner, double rOuter, double y) {
    RowSpans out{{Span{0.0, 0.0}, Span{0.0, 0.0}}, 0};
    const double dy = std::fabs(y - center.y);
    if (dy >= rOuter) return out;
    const double outer = std::sqrt(rOuter * rOuter - dy * dy);
    if (rInner > 0.0 && dy < rInner) {
        const double inner = std::sqrt(rInner * rInner - dy * dy);
        out.s[0] = {center.x - outer, center.x - inner};
        out.s[1] = {center.x + inner, center.x + outer};
        out.n = 2;
    } else {
        out.s[0] = {center.x - outer, center.x + outer};
        out.n = 1;
    }
    return out;
}

/// Advance the phase of the demo path and keep it in [0, kAutoPeriod). Time that only grows would lose its fractions
/// after hours, a wrapped phase does not.
inline double advancePhase(double phase, double dt) {
    const double p = std::fmod(phase + dt, kAutoPeriod);
    return p < 0.0 ? p + kAutoPeriod : p;
}

/// A target path around the root that goes through the hole, the ring and the outside (needs d1 != d2 for a hole).
/// The angle turns once per period, the distance swings three times between 0.05 and 1.2 of the maximum reach.
inline Vec2 autoTarget(Vec2 root, double d1, double d2, double phase) {
    const double reach = maxReach(d1, d2);
    const double turns = 2.0 * kPi * phase / kAutoPeriod;
    const double r = reach * (0.625 + 0.575 * std::sin(3.0 * turns + 0.2));
    const double angle = turns - 0.5;
    return root + Vec2{std::cos(angle), std::sin(angle)} * r;
}

}  // namespace ik
