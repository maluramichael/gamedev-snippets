// Point in polygon: the logic of the demo, no SDL, so test.cpp can check it.
//
// A ray is cast from the test point p to the right (+x). Every polygon edge that crosses the ray is a hit.
//
// THE RULE FOR VERTICES (the whole trick): an edge a->b crosses the ray's row only if
//
//     (a.y > p.y) != (b.y > p.y)
//
// which treats every edge as the half open interval [min y, max y). A vertex that lies exactly on the ray belongs to
// the edge that goes up from it and not to the edge that goes down to it, so a ray through a vertex is counted once
// when the polygon passes through it, and zero or two times when the polygon only touches the ray there. Both keep
// the parity right. Horizontal edges satisfy the test never, so they cannot be counted.
//
// Consequence for a test point exactly ON the boundary: edges are closed on the min x / min y side and open on the
// max x / max y side. (0,2) and (2,0) on the left and bottom side of the square (0,0)-(4,4) are inside, (4,2) and
// (2,4) on the right and top side are outside. The answer is consistent (neighbouring polygons share an edge and
// never both claim a point), but it is a convention, not geometry.
//
// Two ways to read the hits:
//   even-odd (ray casting):  odd number of hits = inside
//   nonzero  (winding number): every hit has a direction (+1 if the edge runs towards bigger y, -1 otherwise),
//                              the sum is the winding number, inside if it is not 0
// They only differ for self-overlapping polygons, e.g. the middle of a pentagram.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <optional>
#include <vector>

namespace pip {

struct Vec2 {
    double x, y;
};

using Polygon = std::vector<Vec2>;

enum class Rule { EvenOdd, NonZero };

struct Crossing {
    double x;          // where the edge meets the ray's row
    int dir;           // +1 if the edge runs towards bigger y, -1 if towards smaller y
    std::size_t edge;  // edge index: from vertex `edge` to vertex `edge + 1` (the last edge closes to vertex 0)
};

struct EdgeHit {
    std::size_t index;
    Vec2 point;       // closest point on the edge
    double distance;
};

struct Span {
    double x0, x1;  // half open: x0 <= x < x1
};

namespace detail {

inline bool spansRow(Vec2 a, Vec2 b, double y) { return (a.y > y) != (b.y > y); }

inline double xAt(Vec2 a, Vec2 b, double y) { return a.x + (y - a.y) * (b.x - a.x) / (b.y - a.y); }

inline int direction(Vec2 a, Vec2 b) { return b.y > a.y ? 1 : -1; }

inline double dist(Vec2 a, Vec2 b) { return std::hypot(a.x - b.x, a.y - b.y); }

}  // namespace detail

/// Edges hit by the ray from p to the right, sorted from left to right.
inline std::vector<Crossing> rayCrossings(const Polygon& poly, Vec2 p) {
    std::vector<Crossing> out;
    const std::size_t n = poly.size();
    if (n < 3) return out;
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2 a = poly[i];
        const Vec2 b = poly[(i + 1) % n];
        if (!detail::spansRow(a, b, p.y)) continue;
        // Which side of the edge is p on? cross / (b.y - a.y) equals (crossing x - p.x), so the sign tells whether the
        // crossing lies to the right of p, without a division and exact for integer coordinates.
        const double cross = (b.x - a.x) * (p.y - a.y) - (p.x - a.x) * (b.y - a.y);
        const bool right = b.y > a.y ? cross > 0.0 : cross < 0.0;
        if (right) out.push_back({detail::xAt(a, b, p.y), detail::direction(a, b), i});
    }
    std::sort(out.begin(), out.end(), [](const Crossing& l, const Crossing& r) { return l.x < r.x; });
    return out;
}

inline int windingNumber(const Polygon& poly, Vec2 p) {
    int w = 0;
    for (const Crossing& c : rayCrossings(poly, p)) w += c.dir;
    return w;
}

inline bool inside(const Polygon& poly, Vec2 p, Rule rule) {
    if (rule == Rule::EvenOdd) return rayCrossings(poly, p).size() % 2 == 1;
    return windingNumber(poly, p) != 0;
}

/// The filled parts of one pixel row: the same decision as `inside`, but for a whole row at once (scanline fill).
inline std::vector<Span> scanSpans(const Polygon& poly, double y, Rule rule) {
    std::vector<Crossing> all;
    const std::size_t n = poly.size();
    if (n < 3) return {};
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2 a = poly[i];
        const Vec2 b = poly[(i + 1) % n];
        if (detail::spansRow(a, b, y)) all.push_back({detail::xAt(a, b, y), detail::direction(a, b), i});
    }
    std::sort(all.begin(), all.end(), [](const Crossing& l, const Crossing& r) { return l.x < r.x; });
    std::vector<Span> spans;
    int count = 0;
    int winding = 0;
    for (std::size_t i = 0; i + 1 < all.size(); ++i) {
        ++count;
        winding += all[i].dir;
        const bool filled = rule == Rule::EvenOdd ? count % 2 == 1 : winding != 0;
        if (filled && all[i + 1].x > all[i].x) spans.push_back({all[i].x, all[i + 1].x});
    }
    return spans;
}

inline std::optional<std::size_t> nearestVertex(const Polygon& poly, Vec2 p, double maxDistance) {
    std::optional<std::size_t> best;
    double bestDistance = maxDistance;
    for (std::size_t i = 0; i < poly.size(); ++i) {
        const double d = detail::dist(poly[i], p);
        if (d <= bestDistance) {
            bestDistance = d;
            best = i;
        }
    }
    return best;
}

/// Closest edge within maxDistance (on a tie the lower index wins).
inline std::optional<EdgeHit> nearestEdge(const Polygon& poly, Vec2 p, double maxDistance) {
    std::optional<EdgeHit> best;
    const std::size_t n = poly.size();
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2 a = poly[i];
        const Vec2 b = poly[(i + 1) % n];
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        const double len2 = dx * dx + dy * dy;
        const double t = len2 > 0.0 ? std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / len2, 0.0, 1.0) : 0.0;
        const Vec2 q{a.x + t * dx, a.y + t * dy};
        const double d = detail::dist(p, q);
        if (d <= maxDistance && (!best || d < best->distance)) best = EdgeHit{i, q, d};
    }
    return best;
}

/// Inserts a vertex on edge `edge` (between vertex `edge` and the next one). Returns the new vertex index.
inline std::size_t insertVertex(Polygon& poly, std::size_t edge, Vec2 at) {
    const std::size_t index = edge + 1;
    poly.insert(poly.begin() + static_cast<std::ptrdiff_t>(index), at);
    return index;
}

/// A polygon needs three vertices, so removing the third-to-last is refused.
inline bool removeVertex(Polygon& poly, std::size_t index) {
    if (poly.size() <= 3 || index >= poly.size()) return false;
    poly.erase(poly.begin() + static_cast<std::ptrdiff_t>(index));
    return true;
}

/// Moves a ray height onto the nearest vertex row if one is within maxDistance, so the vertex case can be tried.
inline double snapToVertexRow(const Polygon& poly, double y, double maxDistance) {
    double best = y;
    double bestDistance = maxDistance;
    for (const Vec2& v : poly) {
        const double d = std::fabs(v.y - y);
        if (d <= bestDistance) {
            bestDistance = d;
            best = v.y;
        }
    }
    return best;
}

// ---- Presets (canvas coordinates of the 960x540 demo, y points down) ----

/// An irregular simple polygon with two dents, so that the ray enters and leaves it several times.
inline Polygon concavePreset() {
    return {{360, 100}, {520, 70}, {640, 130}, {600, 230}, {700, 340}, {560, 410}, {470, 300}, {400, 410}, {330, 260}};
}
inline Vec2 concaveHome() { return {500, 200}; }

/// The pentagram {5/2}: five vertices, each edge connects every second one, so the edges cross each other.
inline Polygon starPreset() {
    Polygon out;
    const double cx = 480.0;
    const double cy = 250.0;
    const double r = 165.0;
    const double pi = 3.14159265358979323846;
    for (int k = 0; k < 5; ++k) {
        const int i = (k * 2) % 5;
        const double angle = (-90.0 + 72.0 * i) * pi / 180.0;
        out.push_back({std::round(cx + r * std::cos(angle)), std::round(cy + r * std::sin(angle))});
    }
    return out;
}
inline Vec2 starHome() { return {480, 250}; }

}  // namespace pip
