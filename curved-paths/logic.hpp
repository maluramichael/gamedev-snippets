// The geometry behind curved-paths, no SDL. A road is a centre line plus an offset on both sides. Two ways to make the
// centre line: a quadratic Bezier curve and a circular arc. What differs is what the road needs from it:
//   offsets    a point at a fixed distance to the left of the centre line, at every position along it
//   distances  how far along the road a position is, and which position belongs to a given distance
// A circular arc answers both in closed form. A Bezier curve answers neither exactly, so it is sampled into short
// segments, and the segments give a table from distance to the curve parameter t.
//
// Coordinates are plain doubles. "Left" is the normal (-y, x); whether that looks left or right on screen depends on
// the y axis, the logic does not care.
#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace cp {

inline constexpr double kEps = 1e-9;
inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kMaxHalfTurn = 2.6;   // rad: an arc needs the end point within this angle of the start tangent
inline constexpr double kLineAngle = 1e-4;    // rad: flatter than this is a line, the circle would be gigantic
inline constexpr double kMiterLimit = 6.0;    // longest allowed miter, as a multiple of the offset

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};

inline Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
inline Vec2 operator*(Vec2 a, double s) { return {a.x * s, a.y * s}; }
inline double dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
inline double cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }
inline double length(Vec2 v) { return std::sqrt(dot(v, v)); }
inline Vec2 leftNormal(Vec2 v) { return {-v.y, v.x}; }

/// Unit vector of `v`, or `fallback` when `v` has no direction. Every normalize in this file goes through here.
inline Vec2 normalizedOr(Vec2 v, Vec2 fallback) {
    const double len = length(v);
    return len > kEps ? v * (1.0 / len) : fallback;
}

// ---------------------------------------------------------------------------------------------------------------
// Distances to polylines, used to measure how wide a road really is.

inline double distanceToSegment(Vec2 p, Vec2 a, Vec2 b) {
    const Vec2 ab = b - a;
    const double l2 = dot(ab, ab);
    if (l2 < kEps * kEps) return length(p - a);
    const double t = std::clamp(dot(p - a, ab) / l2, 0.0, 1.0);
    return length(p - (a + ab * t));
}

inline double distanceToPolyline(Vec2 p, const std::vector<Vec2>& line) {
    if (line.empty()) return std::numeric_limits<double>::infinity();
    if (line.size() == 1) return length(p - line[0]);
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i + 1 < line.size(); ++i) best = std::min(best, distanceToSegment(p, line[i], line[i + 1]));
    return best;
}

/// The largest deviation of an edge from "exactly `halfWidth` away from the centre line", looking at the vertices and at
/// the middle of every segment (a coarse edge cuts corners between its vertices).
inline double maxWidthError(const std::vector<Vec2>& edge, const std::vector<Vec2>& centre, double halfWidth) {
    if (edge.empty() || centre.empty()) return 0.0;
    double worst = 0.0;
    for (std::size_t i = 0; i < edge.size(); ++i) {
        worst = std::max(worst, std::fabs(distanceToPolyline(edge[i], centre) - halfWidth));
        if (i + 1 < edge.size()) {
            const Vec2 mid = (edge[i] + edge[i + 1]) * 0.5;
            worst = std::max(worst, std::fabs(distanceToPolyline(mid, centre) - halfWidth));
        }
    }
    return worst;
}

// ---------------------------------------------------------------------------------------------------------------
// Quadratic Bezier curve.

struct Bezier {
    Vec2 p0, c, p1;
};

inline Vec2 bezierPoint(const Bezier& b, double t) {
    const double u = 1.0 - t;
    return b.p0 * (u * u) + b.c * (2.0 * u * t) + b.p1 * (t * t);
}

/// dB/dt. Its length is the speed of a point that moves with t at a constant rate, and that speed is not constant.
inline Vec2 bezierDerivative(const Bezier& b, double t) { return (b.c - b.p0) * (2.0 * (1.0 - t)) + (b.p1 - b.c) * (2.0 * t); }

/// Unit tangent. The derivative is zero where the control point sits on an end point, then the chord is the best guess.
inline Vec2 bezierTangent(const Bezier& b, double t) { return normalizedOr(bezierDerivative(b, t), normalizedOr(b.p1 - b.p0, {1.0, 0.0})); }

inline Vec2 bezierNormal(const Bezier& b, double t) { return leftNormal(bezierTangent(b, t)); }

/// The exact offset: a step of `d` along the normal at this very t. The result is not a Bezier curve.
inline Vec2 bezierOffsetPoint(const Bezier& b, double t, double d) { return bezierPoint(b, t) + bezierNormal(b, t) * d; }

/// The easy offset: shift the two end points along their normals and meet the shifted control lines in a corner. The
/// result is a Bezier curve again, which is why it is tempting, and it is the wrong width in between.
inline Bezier offsetControlPoints(const Bezier& b, double d) {
    const Vec2 chord = normalizedOr(b.p1 - b.p0, {1.0, 0.0});
    const Vec2 n0 = leftNormal(normalizedOr(b.c - b.p0, chord));
    const Vec2 n1 = leftNormal(normalizedOr(b.p1 - b.c, chord));
    Vec2 shift = n0;
    const Vec2 sum = n0 + n1;
    if (length(sum) > kEps) {
        const Vec2 m = normalizedOr(sum, n0);
        const double cosHalf = dot(m, n0);  // cos of half the turn at the control point
        shift = m * std::min(cosHalf > kEps ? 1.0 / cosHalf : kMiterLimit, kMiterLimit);
    }
    return {b.p0 + n0 * d, b.c + shift * d, b.p1 + n1 * d};
}

inline int clampSegments(int n) { return std::max(n, 1); }

inline std::vector<Vec2> bezierPolyline(const Bezier& b, int segments) {
    const int n = clampSegments(segments);
    std::vector<Vec2> out;
    out.reserve(static_cast<std::size_t>(n) + 1);
    for (int i = 0; i <= n; ++i) out.push_back(bezierPoint(b, static_cast<double>(i) / n));
    return out;
}

inline std::vector<Vec2> offsetPolyline(const Bezier& b, int segments, double d) {
    const int n = clampSegments(segments);
    std::vector<Vec2> out;
    out.reserve(static_cast<std::size_t>(n) + 1);
    for (int i = 0; i <= n; ++i) out.push_back(bezierOffsetPoint(b, static_cast<double>(i) / n, d));
    return out;
}

/// Cumulative length of the polyline through t = i / n. The Bezier curve has no usable closed form for this, so the
/// table is also the answer to "which t is d units along the road".
struct LengthTable {
    std::vector<double> cum;  // cum[i] = length of the curve from t = 0 to t = i / n, cum[0] = 0
    double total() const { return cum.empty() ? 0.0 : cum.back(); }
};

inline LengthTable makeLengthTable(const Bezier& b, int segments) {
    const int n = clampSegments(segments);
    LengthTable table;
    table.cum.reserve(static_cast<std::size_t>(n) + 1);
    table.cum.push_back(0.0);
    Vec2 prev = bezierPoint(b, 0.0);
    for (int i = 1; i <= n; ++i) {
        const Vec2 p = bezierPoint(b, static_cast<double>(i) / n);
        table.cum.push_back(table.cum.back() + length(p - prev));
        prev = p;
    }
    return table;
}

/// The t that lies `s` units along the road, found by binary search and linear interpolation inside the segment.
inline double tAtDistance(const LengthTable& table, double s) {
    const double total = table.total();
    if (table.cum.size() < 2 || total < kEps) return 0.0;
    s = std::clamp(s, 0.0, total);
    const auto it = std::lower_bound(table.cum.begin(), table.cum.end(), s);
    const std::size_t i = static_cast<std::size_t>(it - table.cum.begin());
    if (i == 0) return 0.0;
    const double span = table.cum[i] - table.cum[i - 1];
    const double frac = span > 0.0 ? (s - table.cum[i - 1]) / span : 0.0;
    return (static_cast<double>(i - 1) + frac) / static_cast<double>(table.cum.size() - 1);
}

struct SpeedRange {
    double min = 0.0;
    double max = 0.0;
};

/// Smallest and largest |dB/dt| over t = i / n: how unevenly a car that steps t at a constant rate drives.
inline SpeedRange bezierSpeedRange(const Bezier& b, int samples) {
    const int n = clampSegments(samples);
    SpeedRange r{std::numeric_limits<double>::infinity(), 0.0};
    for (int i = 0; i <= n; ++i) {
        const double v = length(bezierDerivative(b, static_cast<double>(i) / n));
        r.min = std::min(r.min, v);
        r.max = std::max(r.max, v);
    }
    return r;
}

// ---------------------------------------------------------------------------------------------------------------
// Circular arc (or a line, the arc with an infinite radius).

struct Arc {
    bool valid = false;      // false: no circle fits (the end point lies behind the start)
    bool line = false;       // straight; also a chord of length zero
    bool collapsed = false;  // an offset whose radius reached zero or below
    Vec2 p0, p1;
    Vec2 dir{1.0, 0.0};      // unit tangent at the start
    Vec2 center;
    double radius = 0.0;
    double startAngle = 0.0;
    double sweep = 0.0;      // signed, positive turns left
    double length = 0.0;
};

/// The arc that starts in `p0` heading along `tangent` and ends in `p1`. With phi the angle from the tangent to the
/// chord, the turn is 2 phi, the curvature is 2 sin(phi) / |chord|, and the length is radius times angle.
inline Arc makeArc(Vec2 p0, Vec2 tangent, Vec2 p1) {
    Arc a;
    a.p0 = p0;
    a.p1 = p1;
    const Vec2 d = p1 - p0;
    const double chord = length(d);
    a.dir = normalizedOr(tangent, normalizedOr(d, {1.0, 0.0}));
    if (chord < kEps) {
        a.valid = true;
        a.line = true;
        return a;
    }
    const double phi = std::atan2(cross(a.dir, d), dot(a.dir, d));
    if (std::fabs(phi) > kMaxHalfTurn) return a;
    a.valid = true;
    if (std::fabs(phi) < kLineAngle) {
        a.line = true;
        a.length = chord;
        return a;
    }
    const double kappa = 2.0 * std::sin(phi) / chord;
    a.radius = 1.0 / std::fabs(kappa);
    a.center = p0 + leftNormal(a.dir) * (1.0 / kappa);
    a.sweep = 2.0 * phi;
    const Vec2 r = p0 - a.center;
    a.startAngle = std::atan2(r.y, r.x);
    a.length = a.radius * std::fabs(a.sweep);
    return a;
}

inline Vec2 arcPoint(const Arc& a, double u) {
    if (a.line) return a.p0 + (a.p1 - a.p0) * u;
    const double angle = a.startAngle + a.sweep * u;
    return a.center + Vec2{std::cos(angle), std::sin(angle)} * a.radius;
}

inline Vec2 arcTangent(const Arc& a, double u) {
    if (a.line) return normalizedOr(a.p1 - a.p0, a.dir);
    const double angle = a.startAngle + a.sweep * u;
    const double dir = a.sweep >= 0.0 ? 1.0 : -1.0;
    return Vec2{-std::sin(angle), std::cos(angle)} * dir;
}

inline Vec2 arcNormal(const Arc& a, double u) { return leftNormal(arcTangent(a, u)); }

inline Vec2 arcOffsetPoint(const Arc& a, double u, double d) { return arcPoint(a, u) + arcNormal(a, u) * d; }

/// The offset curve of an arc is an arc around the same centre: only the radius changes. Left of a left turn is the
/// inside, so the radius shrinks; once it reaches zero the edge has collapsed into the centre.
inline Arc offsetArc(const Arc& a, double d) {
    Arc o = a;
    if (!a.valid) return o;
    if (a.line) {
        const Vec2 shift = leftNormal(arcTangent(a, 0.0)) * d;
        o.p0 = a.p0 + shift;
        o.p1 = a.p1 + shift;
        return o;
    }
    const double side = a.sweep > 0.0 ? 1.0 : -1.0;
    const double r = a.radius - side * d;
    o.collapsed = r <= kEps;
    o.radius = std::max(r, 0.0);
    o.length = o.radius * std::fabs(a.sweep);
    o.p0 = arcPoint(o, 0.0);
    o.p1 = arcPoint(o, 1.0);
    return o;
}

/// The position at distance `s` along the arc: distance divided by radius is the angle. No table.
inline Vec2 arcPointAtDistance(const Arc& a, double s) {
    const double u = a.length > kEps ? std::clamp(s / a.length, 0.0, 1.0) : 0.0;
    return arcPoint(a, u);
}

/// The distance along the arc of the point nearest to `p`: its angle round the centre times the radius, clamped to the
/// ends of the arc.
inline double arcDistanceOf(const Arc& a, Vec2 p) {
    if (a.length < kEps) return 0.0;
    if (a.line) return std::clamp(dot(p - a.p0, arcTangent(a, 0.0)), 0.0, a.length);
    const Vec2 v = p - a.center;
    if (length(v) < kEps) return 0.0;
    const double side = a.sweep >= 0.0 ? 1.0 : -1.0;
    const double twoPi = 2.0 * kPi;
    double delta = std::fmod((std::atan2(v.y, v.x) - a.startAngle) * side, twoPi);
    if (delta < 0.0) delta += twoPi;
    const double span = std::fabs(a.sweep);
    if (delta <= span) return delta * a.radius;
    return (delta - span) < (twoPi - delta) ? a.length : 0.0;  // off the end: nearer to the end or to the start
}

inline std::vector<Vec2> arcPolyline(const Arc& a, int segments) {
    if (!a.valid || a.collapsed) return {};
    const int n = clampSegments(segments);
    std::vector<Vec2> out;
    out.reserve(static_cast<std::size_t>(n) + 1);
    for (int i = 0; i <= n; ++i) out.push_back(arcPoint(a, static_cast<double>(i) / n));
    return out;
}

}  // namespace cp
