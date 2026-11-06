// Triangle rasterizer: the logic of the demo, no SDL, so test.cpp can check it.
//
// A triangle is filled by asking every pixel whether its centre is inside. With the three edge functions
//
//     E(a, b, p) = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x)
//
// a point is inside when all three are positive (the triangle is turned so that its area is positive, so the
// interior is always on the positive side). Everything is integers: coordinates are sub-pixel units (kSub per pixel),
// the products fit into 64 bits, so "exactly on the edge" is an exact question and not a rounding accident.
//
// THE PROBLEM: a pixel centre that lies exactly ON an edge shared by two triangles. Counting it as inside (E >= 0)
// draws it twice, which shows as a bright seam with additive blending or double-drawn alpha. Counting it as outside
// (E > 0) draws it never, which leaves a hole. Both are one row of pixels along the seam.
//
// THE RULE (Fleischer and Salesin, "Accurate Polygon Scan Conversion Using Half-Open Intervals", Graphics Gems III,
// VII.6; their version works per scanline, this one per pixel centre with edge functions): a centre on an edge counts
// only when that edge is a top or a left edge. Two triangles that share an edge see it in opposite directions, and
// exactly one of the two directions is a top or left edge, so exactly one triangle owns the pixel. The same thing as
// half-open intervals: every shape owns [left, right) x [top, bottom).
//
// On a y-down screen with the triangle's area positive (clockwise on screen) the interior is below a top edge and to
// the right of a left edge. A top edge runs left to right (dy == 0, dx > 0), a left edge runs upwards (dy < 0).
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <vector>

namespace tr {

constexpr int kSub = 16;                  // sub-pixel units per pixel; a pixel centre is at kSub * x + kSub / 2
constexpr int kMaxCoord = 1 << 29;        // larger coordinates are refused: differences stay below 2^30 (fit an int), products below 2^60

struct Pt {
    int x, y;
};

struct Tri {
    Pt a, b, c;
};

enum class Rule { Inclusive, Exclusive, TopLeft };

inline long long edgeFn(Pt a, Pt b, Pt p) {
    return (static_cast<long long>(b.x) - a.x) * (static_cast<long long>(p.y) - a.y) -
           (static_cast<long long>(b.y) - a.y) * (static_cast<long long>(p.x) - a.x);
}

/// Twice the signed area of the triangle. Positive means clockwise on a y-down screen.
inline long long area2(const Tri& t) { return edgeFn(t.a, t.b, t.c); }

/// The same triangle with a positive area (two corners swapped when it is negative).
inline Tri oriented(Tri t) {
    if (area2(t) < 0) std::swap(t.b, t.c);
    return t;
}

/// Whether a pixel centre exactly on the directed edge a -> b counts for the triangle that owns this edge:
/// true for a top edge (left to right, flat) and a left edge (running upwards). Of the two directions of the same
/// edge exactly one is owned.
inline bool ownsEdge(Pt a, Pt b) {
    const int dx = b.x - a.x;
    const int dy = b.y - a.y;
    return dy < 0 || (dy == 0 && dx > 0);
}

inline bool edgeCounts(long long e, Pt a, Pt b, Rule rule) {
    switch (rule) {
        case Rule::Inclusive: return e >= 0;
        case Rule::Exclusive: return e > 0;
        case Rule::TopLeft: return e > 0 || (e == 0 && ownsEdge(a, b));
    }
    return false;
}

/// `t` must have a positive area (see oriented()).
inline bool covers(const Tri& t, Pt p, Rule rule) {
    return edgeCounts(edgeFn(t.a, t.b, p), t.a, t.b, rule) && edgeCounts(edgeFn(t.b, t.c, p), t.b, t.c, rule) &&
           edgeCounts(edgeFn(t.c, t.a, p), t.c, t.a, rule);
}

inline Pt pixelCentre(int x, int y) { return {x * kSub + kSub / 2, y * kSub + kSub / 2}; }

/// Division that rounds towards minus infinity (the % of negative numbers is not what a pixel index needs).
inline int floorDiv(int a, int b) {
    int q = a / b;
    if ((a % b != 0) && ((a < 0) != (b < 0))) --q;
    return q;
}

inline bool inRange(Pt p) { return std::abs(static_cast<long long>(p.x)) <= kMaxCoord && std::abs(static_cast<long long>(p.y)) <= kMaxCoord; }

/// Calls visit(x, y) for every pixel of a w x h grid whose centre the triangle covers under `rule`. Only the
/// triangle's bounding box is tested. A triangle with zero area, or with a coordinate beyond kMaxCoord, draws nothing.
template <class Visit>
void rasterize(const Tri& tri, Rule rule, int w, int h, Visit&& visit) {
    if (!inRange(tri.a) || !inRange(tri.b) || !inRange(tri.c)) return;
    if (area2(tri) == 0) return;
    const Tri t = oriented(tri);
    const int minX = std::min({t.a.x, t.b.x, t.c.x});
    const int maxX = std::max({t.a.x, t.b.x, t.c.x});
    const int minY = std::min({t.a.y, t.b.y, t.c.y});
    const int maxY = std::max({t.a.y, t.b.y, t.c.y});
    const int x0 = std::max(0, floorDiv(minX, kSub) - 1);
    const int x1 = std::min(w - 1, floorDiv(maxX, kSub) + 1);
    const int y0 = std::max(0, floorDiv(minY, kSub) - 1);
    const int y1 = std::min(h - 1, floorDiv(maxY, kSub) + 1);
    for (int y = y0; y <= y1; ++y) {
        for (int x = x0; x <= x1; ++x) {
            if (covers(t, pixelCentre(x, y), rule)) visit(x, y);
        }
    }
}

/// How often each pixel is drawn when all triangles are rasterized (row by row, w * h entries).
inline std::vector<int> coverage(const std::vector<Tri>& tris, Rule rule, int w, int h) {
    std::vector<int> counts(static_cast<std::size_t>(std::max(w, 0)) * static_cast<std::size_t>(std::max(h, 0)), 0);
    for (const Tri& t : tris) {
        rasterize(t, rule, w, h, [&](int x, int y) { ++counts[static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)]; });
    }
    return counts;
}

struct Tally {
    int once = 0;   // pixels drawn exactly once
    int twice = 0;  // pixels drawn two or more times (the bright seam)
    int holes = 0;  // pixels the reference draws but this one does not
};

inline Tally tally(const std::vector<int>& counts, const std::vector<int>& reference) {
    Tally t;
    const std::size_t n = std::min(counts.size(), reference.size());
    for (std::size_t i = 0; i < n; ++i) {
        if (counts[i] == 1) ++t.once;
        if (counts[i] >= 2) ++t.twice;
        if (counts[i] == 0 && reference[i] > 0) ++t.holes;
    }
    return t;
}

// A fan: one centre and a ring of corners, triangles (centre, rim[i], rim[i+1]). All triangles share the spokes.

struct Fan {
    Pt center;
    std::vector<Pt> rim;
};

inline std::vector<Tri> triangles(const Fan& fan) {
    std::vector<Tri> out;
    const std::size_t n = fan.rim.size();
    if (n < 3) return out;
    for (std::size_t i = 0; i < n; ++i) out.push_back({fan.center, fan.rim[i], fan.rim[(i + 1) % n]});
    return out;
}

/// A fan is valid when every triangle has a positive area and the ring goes exactly once around the centre. The first
/// alone is not enough: a pentagram order has five positive triangles that wind twice and overlap.
inline bool fanValid(const Fan& fan) {
    const std::size_t n = fan.rim.size();
    if (n < 3) return false;
    double turned = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        const Pt a = fan.rim[i];
        const Pt b = fan.rim[(i + 1) % n];
        if (!inRange(a) || !inRange(b) || !inRange(fan.center)) return false;
        const double ax = static_cast<double>(a.x - fan.center.x);
        const double ay = static_cast<double>(a.y - fan.center.y);
        const double bx = static_cast<double>(b.x - fan.center.x);
        const double by = static_cast<double>(b.y - fan.center.y);
        const long long cross = edgeFn(fan.center, a, b);
        if (cross <= 0) return false;
        turned += std::atan2(static_cast<double>(cross), ax * bx + ay * by);
    }
    return std::fabs(turned - 2.0 * 3.14159265358979323846) < 1e-6;
}

/// Nearest pixel centre to a position given in pixels (1.0 = one pixel).
inline Pt snapToCentre(double gx, double gy) {
    return {static_cast<int>(std::floor(gx)) * kSub + kSub / 2, static_cast<int>(std::floor(gy)) * kSub + kSub / 2};
}

/// The pixel centre of the pixel a sub-pixel position lies in.
inline Pt centreOf(Pt p) { return {floorDiv(p.x, kSub) * kSub + kSub / 2, floorDiv(p.y, kSub) * kSub + kSub / 2}; }

/// Every corner of the fan moved to its pixel centre.
inline Fan snappedFan(Fan fan) {
    fan.center = centreOf(fan.center);
    for (Pt& p : fan.rim) p = centreOf(p);
    return fan;
}

/// Nearest sub-pixel position to a position given in pixels.
inline Pt snapToSub(double gx, double gy) {
    return {static_cast<int>(std::lround(gx * kSub)), static_cast<int>(std::lround(gy * kSub))};
}

/// The start fan: an ellipse of `n` corners around the middle of a w x h grid, all on pixel centres, so a lot of
/// centres sit exactly on an edge from the first frame on.
inline Fan presetFan(int n, int w, int h) {
    Fan fan;
    const int cx = w / 2;
    const int cy = h / 2;
    fan.center = pixelCentre(cx, cy);
    const double rx = 0.39 * w;
    const double ry = 0.41 * h;
    const double tau = 2.0 * 3.14159265358979323846;
    for (int k = 0; k < n; ++k) {
        const double t = tau * k / n;
        fan.rim.push_back(snapToCentre(cx + 0.5 + rx * std::cos(t), cy + 0.5 + ry * std::sin(t)));
    }
    return fan;
}

}  // namespace tr
