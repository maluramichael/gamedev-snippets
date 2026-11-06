// Separating axis theorem (SAT) for two convex polygons: the logic of the demo, no SDL, so test.cpp can check it.
//
// Two convex shapes do not overlap if and only if there is a line that separates them. It is enough to look at the
// normals of the edges (the "axes"): project both shapes onto an axis, which turns each into an interval. Gap between
// the intervals on ANY axis proves "no collision" and the search can stop. Overlap on EVERY axis proves "collision".
//
// The axis with the smallest overlap also gives the way out: moving B along that axis by that overlap (the minimum
// translation vector, MTV) separates the shapes with the shortest possible move. Three details decide whether the
// MTV is right:
//
//   1. The axes must be NORMALIZED. A raw axis perp(edge) is as long as the edge, and every projection onto it is
//      scaled by that length. The yes/no answer survives (the sign does not change) but the overlaps of different
//      axes are no longer comparable, so the wrong axis wins, and amount * axis is scaled by length squared.
//   2. The MTV needs a DIRECTION. An edge normal points out of its own shape. For an axis of A that is the way B has
//      to go, for an axis of B it is the way back into A. The direction is chosen per axis by the side B has to leave
//      through: the shorter way out of the interval of A.
//   3. Overlap of the intervals is not the distance to leave. When one interval lies inside the other, the shorter
//      interval's length is the "overlap" but B still has to travel past the end of A. The distance to leave is
//      min(maxA - minB, maxB - minA), which is also what gives the direction in 2.
//
// Shapes are convex polygons with area, in any winding. Units are meters, y may point either way.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace sat {

struct Vec2 {
    double x, y;
};

inline Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
inline Vec2 operator*(Vec2 a, double s) { return {a.x * s, a.y * s}; }
inline double dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
inline double cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }
inline double length(Vec2 v) { return std::hypot(v.x, v.y); }

using Polygon = std::vector<Vec2>;

/// A polygon in local coordinates around its center, turned by `angle` and moved to `pos`.
struct Shape {
    Polygon local;
    Vec2 pos;
    double angle;
};

/// Both shapes of a starting position.
struct Scene {
    Shape a, b;
};

/// Shadow of a polygon on an axis, with the corners that cast its two ends.
struct Interval {
    double min, max;
    std::size_t iMin, iMax;
};

struct Options {
    bool normalize = true;  // unit length axes. Off: the axis is perp(edge) and as long as the edge
    bool orient = true;     // choose the side B leaves through. Off: the MTV keeps the direction of the axis
};

/// What one axis says about the pair.
struct AxisResult {
    int owner;         // 0: edge of A, 1: edge of B
    std::size_t edge;  // index of the edge in its polygon, from corner `edge` to corner `edge + 1`
    Vec2 axis;         // outward normal of that edge (unit length if normalized)
    Interval a, b;     // the two shadows on `axis`
    double amount;     // > 0: depth of the overlap on this axis, <= 0: minus the gap. In units of |axis|
    Vec2 push;         // move B by this to remove the overlap on this axis: direction * amount
};

struct Result {
    bool colliding = false;
    std::vector<AxisResult> axes;  // one per direction, parallel edges are not repeated
    std::size_t best = 0;          // the axis with the smallest `amount`: the MTV axis, or the widest gap if apart
    Vec2 mtv{0.0, 0.0};            // zero when not colliding
};

inline constexpr double kTouch = 1e-9;  // overlaps up to this size count as touching
inline constexpr int kPresetCount = 3;

inline double signedArea(const Polygon& p) {
    double s = 0.0;
    for (std::size_t i = 0; i < p.size(); ++i) s += cross(p[i], p[(i + 1) % p.size()]);
    return s / 2.0;
}

/// Mean of the corners, inside every convex polygon.
inline Vec2 center(const Polygon& p) {
    if (p.empty()) return {0.0, 0.0};
    Vec2 sum{0.0, 0.0};
    for (const Vec2& v : p) sum = sum + v;
    return sum * (1.0 / static_cast<double>(p.size()));
}

inline Polygon worldVertices(const Shape& s) {
    const double c = std::cos(s.angle);
    const double sn = std::sin(s.angle);
    Polygon out;
    out.reserve(s.local.size());
    for (const Vec2& v : s.local) out.push_back({s.pos.x + v.x * c - v.y * sn, s.pos.y + v.x * sn + v.y * c});
    return out;
}

inline Polygon translated(const Polygon& p, Vec2 d) {
    Polygon out = p;
    for (Vec2& v : out) v = v + d;
    return out;
}

/// Point in a convex polygon of either winding, the border counts as inside.
inline bool contains(const Polygon& p, Vec2 q) {
    if (p.size() < 3) return false;
    bool positive = false;
    bool negative = false;
    for (std::size_t i = 0; i < p.size(); ++i) {
        const Vec2 a = p[i];
        const Vec2 b = p[(i + 1) % p.size()];
        const double side = cross(b - a, q - a);
        positive = positive || side > 0.0;
        negative = negative || side < 0.0;
    }
    return !(positive && negative);
}

/// Shadow of the polygon on the axis: the smallest and largest dot product over all corners.
inline Interval project(const Polygon& p, Vec2 axis) {
    Interval iv{dot(p.at(0), axis), dot(p.at(0), axis), 0, 0};
    for (std::size_t i = 1; i < p.size(); ++i) {
        const double d = dot(p[i], axis);
        if (d < iv.min) iv = {d, iv.max, i, iv.iMax};
        if (d > iv.max) iv = {iv.min, d, iv.iMin, i};
    }
    return iv;
}

namespace detail {

struct RawAxis {
    int owner;
    std::size_t edge;
    Vec2 axis;
};

/// Outward normals of the edges of `p`, one per direction. `known` holds the axes found so far (A's, when called for B).
inline void collectAxes(const Polygon& p, int owner, bool normalize, std::vector<RawAxis>& known) {
    const double turn = signedArea(p) >= 0.0 ? 1.0 : -1.0;  // flips the normal for clockwise polygons
    for (std::size_t i = 0; i < p.size(); ++i) {
        const Vec2 e = p[(i + 1) % p.size()] - p[i];
        const double len = length(e);
        if (len < 1e-12) continue;  // a repeated corner: no edge, no axis (and nothing to divide by)
        Vec2 n{e.y * turn, -e.x * turn};
        const Vec2 unit = n * (1.0 / len);
        const bool seen = std::any_of(known.begin(), known.end(), [&](const RawAxis& k) {
            return std::fabs(cross(unit, k.axis * (1.0 / length(k.axis)))) < 1e-9;
        });
        if (seen) continue;
        known.push_back({owner, i, normalize ? unit : n});
    }
}

}  // namespace detail

/// Runs the test on all axes of both polygons. With the default options the answer is complete and right,
/// the options exist to show what each shortcut breaks.
inline Result collide(const Polygon& a, const Polygon& b, Options opt = {}) {
    Result r;
    if (a.size() < 3 || b.size() < 3) return r;
    std::vector<detail::RawAxis> raw;
    detail::collectAxes(a, 0, opt.normalize, raw);
    detail::collectAxes(b, 1, opt.normalize, raw);
    for (const detail::RawAxis& k : raw) {
        AxisResult ar{k.owner, k.edge, k.axis, project(a, k.axis), project(b, k.axis), 0.0, {0.0, 0.0}};
        Vec2 dir = k.axis;
        if (opt.orient) {
            const double forward = ar.a.max - ar.b.min;   // B moves along +axis until it clears A
            const double backward = ar.b.max - ar.a.min;  // B moves along -axis until it clears A
            ar.amount = std::min(forward, backward);
            if (backward < forward) dir = k.axis * -1.0;
        } else {
            ar.amount = std::min(ar.a.max, ar.b.max) - std::max(ar.a.min, ar.b.min);
        }
        ar.push = dir * ar.amount;
        r.axes.push_back(ar);
    }
    if (r.axes.empty()) return r;
    for (std::size_t i = 1; i < r.axes.size(); ++i) {
        if (r.axes[i].amount < r.axes[r.best].amount) r.best = i;
    }
    // The smallest amount decides both questions: > 0 means every axis overlaps, <= 0 means this one separates.
    r.colliding = r.axes[r.best].amount > kTouch;
    if (r.colliding) r.mtv = r.axes[r.best].push;
    return r;
}

/// How deep the shapes overlap along the shortest way out, 0 if they are apart or touch. Always computed the right way.
inline double penetration(const Polygon& a, const Polygon& b) {
    const Result r = collide(a, b, Options{true, true});
    return r.colliding ? r.axes[r.best].amount : 0.0;
}

namespace detail {

inline Polygon regular(int corners, double radius, double start) {
    Polygon p;
    for (int k = 0; k < corners; ++k) {
        const double t = start + 6.283185307179586 * static_cast<double>(k) / static_cast<double>(corners);
        p.push_back({radius * std::cos(t), radius * std::sin(t)});
    }
    return p;
}

/// Moves the corners so that their mean is the origin, the pivot for dragging and turning.
inline Polygon centered(Polygon p) {
    const Vec2 c = center(p);
    for (Vec2& v : p) v = v - c;
    return p;
}

}  // namespace detail

/// Starting positions, in meters (the demo draws 40 px per meter). All overlap at the start.
inline Scene preset(int index) {
    constexpr double kHalfPi = 1.5707963267948966;
    constexpr double kSizeUp = 1.5;
    constexpr Vec2 kHome{0.8, 1.2};  // keeps the pair clear of the panels the demo draws in the corners
    Scene sc;
    switch (index) {
        case 1:
            sc = {{detail::centered({{-1.1, 0.6}, {1.1, 0.6}, {0.0, -1.0}}), {6.9, 4.0}, 0.2},
                  {detail::regular(6, 0.9, 0.0), {8.2, 4.4}, 0.3}};
            break;
        case 2:
            sc = {{{{-1.0, -0.6}, {1.0, -0.6}, {1.0, 0.6}, {-1.0, 0.6}}, {6.9, 4.1}, 0.15},
                  {{{-0.75, -0.75}, {0.75, -0.75}, {0.75, 0.75}, {-0.75, 0.75}}, {8.3, 4.6}, 0.55}};
            break;
        default:
            sc = {{{{-1.33, -0.53}, {1.33, -0.53}, {1.33, 0.53}, {-1.33, 0.53}}, {6.8, 3.8}, 0.25},
                  {detail::regular(5, 0.97, -kHalfPi), {8.2, 4.6}, 0.6}};
            break;
    }
    sc.a.pos = sc.a.pos + kHome;
    sc.b.pos = sc.b.pos + kHome;
    // Authored at 60 px per unit, the demo draws 40 px per meter. Edges between 1.6 and 4 m make the error of raw axes
    // (scaled by the edge length) easy to see; with edges around 1 m it would hide.
    for (Shape* shape : {&sc.a, &sc.b}) {
        shape->pos = shape->pos * kSizeUp;
        for (Vec2& v : shape->local) v = v * kSizeUp;
    }
    return sc;
}

}  // namespace sat
