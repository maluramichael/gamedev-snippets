// Hex rings, spirals, rotation and mirroring: the logic of the demo, no SDL, so test.cpp can check it.
//
// A hex is (q, r) in axial coordinates. The third cube coordinate is s = -q - r, so q + r + s is always 0. Almost
// everything here is a consequence of that:
//
//   * The six neighbors are six fixed steps (kDirections). A ring is "go to the corner, then walk each direction N times".
//   * Ring k has 6k cells, so a spiral of radius N has 1 + 6 * (1 + 2 + ... + N) = 1 + 3N(N+1) cells. No counting needed.
//   * Turning 60 degrees around the origin only permutes the cube coordinates and flips their sign. That is exact
//     integer arithmetic, no sine, no cosine, no rounding. Mirroring is a plain swap of two coordinates.
//
// "Left" means counter-clockwise on a screen where y points down and the hexes have flat tops (kDirections[0] is lower
// right, [1] upper right, [2] up, and so on, so direction i + 1 is a left turn from direction i).
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <string>
#include <vector>

namespace hrr {

struct Hex {
    int q, r;
    int s() const { return -q - r; }
    bool operator==(const Hex&) const = default;
};

inline Hex operator+(Hex a, Hex b) { return {a.q + b.q, a.r + b.r}; }
inline Hex operator-(Hex a, Hex b) { return {a.q - b.q, a.r - b.r}; }
inline Hex operator*(Hex a, int k) { return {a.q * k, a.r * k}; }

/// The six neighbors in a fixed order. Walking them in this order goes around a hex counter-clockwise on screen.
inline constexpr std::array<Hex, 6> kDirections{{{1, 0}, {1, -1}, {0, -1}, {-1, 0}, {-1, 1}, {0, 1}}};

/// Any integer is a valid direction number, 6 wraps around to 0 and -1 is 5.
inline int wrap6(int i) { return ((i % 6) + 6) % 6; }

inline Hex neighbor(Hex h, int direction) { return h + kDirections[static_cast<std::size_t>(wrap6(direction))]; }

/// Number of steps between two hexes: the largest absolute difference of the three cube coordinates.
inline int distance(Hex a, Hex b) {
    const Hex d = a - b;
    return std::max({std::abs(d.q), std::abs(d.r), std::abs(d.s())});
}

/// All cells at exactly `radius` from `center`. Radius 0 is the center itself, a negative radius is empty.
/// Cell 0 is center + direction 4 * radius, the rest follows by walking `radius` steps in each direction 0..5,
/// so cell i lies on side ringSide(i, radius).
inline std::vector<Hex> ring(Hex center, int radius) {
    if (radius < 0) return {};
    if (radius == 0) return {center};
    std::vector<Hex> out;
    out.reserve(static_cast<std::size_t>(6 * radius));
    Hex h = center + kDirections[4] * radius;
    for (int side = 0; side < 6; ++side) {
        for (int step = 0; step < radius; ++step) {
            out.push_back(h);
            h = neighbor(h, side);
        }
    }
    return out;
}

/// Which direction ring cell `index` walks in to reach the next cell. Only for radius >= 1.
inline int ringSide(int index, int radius) { return radius > 0 ? index / radius : 0; }

/// Number of cells within `radius` of a hex: the center plus ring 1..N with 6k cells each, 1 + 3N(N+1).
inline int spiralCount(int radius) { return radius < 0 ? 0 : 1 + 3 * radius * (radius + 1); }

/// The center, then ring 1, then ring 2 up to ring `radius`.
inline std::vector<Hex> spiral(Hex center, int radius) {
    if (radius < 0) return {};
    std::vector<Hex> out;
    out.reserve(static_cast<std::size_t>(spiralCount(radius)));
    for (int k = 0; k <= radius; ++k) {
        for (const Hex& h : ring(center, k)) out.push_back(h);
    }
    return out;
}

// --- Rotation and mirroring ---------------------------------------------------------------------------------------

/// A permutation of (q, r, s) with a common sign: result component i is sign * cube[from[i]].
struct Swizzle {
    std::array<int, 3> from;
    int sign;
};

/// kRotations[k] turns a hex around the origin by k * 60 degrees to the left.
inline constexpr std::array<Swizzle, 6> kRotations{{
    {{0, 1, 2}, +1},  // (q, r, s)
    {{2, 0, 1}, -1},  // (-s, -q, -r)
    {{1, 2, 0}, +1},  // (r, s, q)
    {{0, 1, 2}, -1},  // (-q, -r, -s)
    {{2, 0, 1}, +1},  // (s, q, r)
    {{1, 2, 0}, -1},  // (-r, -s, -q)
}};

enum class Axis { Q, R, S };

/// Mirror across the line of cells where the two other coordinates are equal: the named coordinate stays, the other two swap.
inline constexpr Swizzle mirrorSwizzle(Axis axis) {
    switch (axis) {
        case Axis::Q: return {{0, 2, 1}, +1};  // (q, s, r)
        case Axis::R: return {{2, 1, 0}, +1};  // (s, r, q)
        case Axis::S: return {{1, 0, 2}, +1};  // (r, q, s)
    }
    return {{0, 1, 2}, +1};
}

inline Hex apply(Swizzle w, Hex h) {
    const std::array<int, 3> cube{h.q, h.r, h.s()};
    return {w.sign * cube[static_cast<std::size_t>(w.from[0])], w.sign * cube[static_cast<std::size_t>(w.from[1])]};
}

inline std::string formula(Swizzle w) {
    const char* names[3] = {"q", "r", "s"};
    std::string out = "(";
    for (std::size_t i = 0; i < 3; ++i) {
        if (i > 0) out += ", ";
        if (w.sign < 0) out += "-";
        out += names[w.from[i]];
    }
    return out + ")";
}

/// Turn around the origin by `steps` * 60 degrees to the left, negative steps turn right. Any integer is fine.
inline Hex rotateSteps(Hex h, int steps) { return apply(kRotations[static_cast<std::size_t>(wrap6(steps))], h); }
inline Hex rotateLeft(Hex h) { return rotateSteps(h, 1); }
inline Hex rotateRight(Hex h) { return rotateSteps(h, -1); }

/// Turn around any hex: move the pivot to the origin, turn, move back.
inline Hex rotateAround(Hex h, Hex pivot, int steps) { return rotateSteps(h - pivot, steps) + pivot; }

inline Hex mirror(Hex h, Axis axis) { return apply(mirrorSwizzle(axis), h); }
inline Hex mirrorAround(Hex h, Hex pivot, Axis axis) { return mirror(h - pivot, axis) + pivot; }

inline std::string rotationFormula(int steps) { return formula(kRotations[static_cast<std::size_t>(wrap6(steps))]); }
inline std::string mirrorFormula(Axis axis) { return formula(mirrorSwizzle(axis)); }

/// The classic mistake: read (q, r) as (x, y), rotate with sine and cosine and round to the nearest integers.
/// Axial axes are 60 degrees apart, not 90, so this lands on the wrong cells. Only here to be shown next to the right way.
inline Hex naiveRotate(Hex h, int steps) {
    const double angle = -static_cast<double>(wrap6(steps)) * 3.14159265358979323846 / 3.0;
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    return {static_cast<int>(std::lround(h.q * c - h.r * s)), static_cast<int>(std::lround(h.q * s + h.r * c))};
}

/// True if both lists describe the same shape: every pair of cells has the same distance in both.
inline bool sameShape(const std::vector<Hex>& a, const std::vector<Hex>& b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i) {
        for (std::size_t j = i + 1; j < a.size(); ++j) {
            if (distance(a[i], a[j]) != distance(b[i], b[j])) return false;
        }
    }
    return true;
}

/// The little snake the rotate mode moves around. Four neighbors in a row of steps, the last one is the tip.
inline const std::vector<Hex> kShape{{1, 1}, {2, 0}, {3, 0}, {3, -1}};

// --- Pixels (flat top hexes) -------------------------------------------------------------------------------------

struct Point {
    double x, y;
};

/// Center of a hex with corner radius `size`. Flat top: corners at 0, 60, ... degrees, y points down.
inline Point toPixel(Hex h, double size) {
    return {size * 1.5 * h.q, size * (std::sqrt(3.0) / 2.0 * h.q + std::sqrt(3.0) * h.r)};
}

/// The hex under a pixel: fractional axial coordinates, then cube rounding. Rounding q, r and s one by one can break
/// q + r + s = 0, so the component with the largest rounding error is recomputed from the other two.
inline Hex fromPixel(double x, double y, double size) {
    const double q = (2.0 / 3.0 * x) / size;
    const double r = (-1.0 / 3.0 * x + std::sqrt(3.0) / 3.0 * y) / size;
    const double s = -q - r;
    double rq = std::round(q);
    double rr = std::round(r);
    const double rs = std::round(s);
    const double dq = std::fabs(rq - q);
    const double dr = std::fabs(rr - r);
    const double ds = std::fabs(rs - s);
    if (dq > dr && dq > ds) {
        rq = -rr - rs;
    } else if (dr > ds) {
        rr = -rq - rs;
    }
    return {static_cast<int>(rq), static_cast<int>(rr)};
}

}  // namespace hrr
