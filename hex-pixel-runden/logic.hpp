// From a mouse position to a hex: the logic of the demo, no SDL, so test.cpp can check it.
//
// A hex is addressed by cube coordinates (q, r, s) with q + r + s == 0. The pixel to hex direction has two steps:
//
//   1. toFractional: undo the pixel layout with the inverse of its 2x2 matrix. The result is a point in cube space
//      with fractional coordinates, still on the plane q + r + s == 0 because s is derived as -q - r.
//   2. cubeRound: find the hex that contains that point.
//
// THE TRAP in step 2: rounding q, r and s one by one does not keep the sum at 0. Each component moves by up to 0.5,
// so the rounded sum is -1, 0 or +1, and about a quarter of the plane ends up at -1 or +1. The result is not a hex.
// The fix: compare how far each component moved, take the one that moved the most and recompute it from the other
// two (q = -r - s). That component carried the biggest rounding error, so it is the one that should give.
//
// A line between two hexes uses the same idea: N = distance(a, b) samples of a straight lerp in cube space, each
// rounded. A sample can land exactly on the border of two hexes; a tiny shift of the whole line keeps that from
// depending on the tie rule.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <vector>

namespace hpr {

struct Vec2 {
    double x, y;
};

/// Cube coordinates. A real hex has q + r + s == 0; the naive rounding can produce triples that do not.
struct Cube {
    int q, r, s;
};

/// A point in cube space between the hexes.
struct Frac {
    double q, r, s;
};

struct Rect {
    double x0, y0, x1, y1;
};

inline constexpr double kSqrt3 = 1.7320508075688772;

inline bool valid(Cube c) { return c.q + c.r + c.s == 0; }
inline Cube fromAxial(int q, int r) { return {q, r, -q - r}; }

/// Hex layout: flat or pointy topped, `size` is the distance from the center to a corner, `origin` the pixel of hex (0,0,0).
struct Layout {
    bool flat;
    double size;
    Vec2 origin;
};

/// Pixel position of a point in cube space (q and r are enough, s is implied).
inline Vec2 toPixel(const Layout& l, double q, double r) {
    if (l.flat) return {l.origin.x + l.size * 1.5 * q, l.origin.y + l.size * (kSqrt3 / 2.0 * q + kSqrt3 * r)};
    return {l.origin.x + l.size * (kSqrt3 * q + kSqrt3 / 2.0 * r), l.origin.y + l.size * 1.5 * r};
}

inline Vec2 toPixel(const Layout& l, Cube h) { return toPixel(l, static_cast<double>(h.q), static_cast<double>(h.r)); }

/// Step 1: the inverse of toPixel. A layout without a size has nothing to divide by and maps everything to the origin.
inline Frac toFractional(const Layout& l, Vec2 p) {
    if (!(l.size > 0.0)) return {0.0, 0.0, 0.0};
    const double x = (p.x - l.origin.x) / l.size;
    const double y = (p.y - l.origin.y) / l.size;
    const double q = l.flat ? 2.0 / 3.0 * x : kSqrt3 / 3.0 * x - 1.0 / 3.0 * y;
    const double r = l.flat ? -1.0 / 3.0 * x + kSqrt3 / 3.0 * y : 2.0 / 3.0 * y;
    return {q, r, -q - r};
}

/// Rounds to the nearest integer; NaN becomes 0 and huge values are clamped, so the cast to int is always defined.
inline int toInt(double v) {
    if (!(v == v)) return 0;
    return static_cast<int>(std::clamp(std::round(v), -1.0e6, 1.0e6));
}

/// Everything step 2 does, so the demo can show it.
struct Rounding {
    Cube naive;       // each component rounded on its own, may not sum to 0
    Cube fixed;       // after the correction, always a hex
    double dq, dr, ds;  // how far each component moved while rounding
    int fixedAxis;    // 0, 1, 2 = q, r, s was recomputed, -1 = the naive result was already a hex
};

inline Rounding roundDetail(Frac f) {
    Rounding o{};
    o.naive = {toInt(f.q), toInt(f.r), toInt(f.s)};
    o.dq = std::fabs(static_cast<double>(o.naive.q) - f.q);
    o.dr = std::fabs(static_cast<double>(o.naive.r) - f.r);
    o.ds = std::fabs(static_cast<double>(o.naive.s) - f.s);
    o.fixed = o.naive;
    o.fixedAxis = -1;
    if (valid(o.naive)) return o;
    if (o.dq > o.dr && o.dq > o.ds) {
        o.fixed.q = -o.naive.r - o.naive.s;
        o.fixedAxis = 0;
    } else if (o.dr > o.ds) {
        o.fixed.r = -o.naive.q - o.naive.s;
        o.fixedAxis = 1;
    } else {
        o.fixed.s = -o.naive.q - o.naive.r;
        o.fixedAxis = 2;
    }
    return o;
}

/// Step 2: the hex that contains the point.
inline Cube cubeRound(Frac f) { return roundDetail(f).fixed; }

/// Both steps: the hex under a pixel.
inline Cube pickHex(const Layout& l, Vec2 p) { return cubeRound(toFractional(l, p)); }

/// Number of steps between two hexes.
inline int distance(Cube a, Cube b) { return std::max({std::abs(a.q - b.q), std::abs(a.r - b.r), std::abs(a.s - b.s)}); }

/// The six corners of a hex outline in pixels. Flat: the first corner points right, pointy: up and to the right.
inline std::array<Vec2, 6> corners(const Layout& l, Cube h) {
    const Vec2 c = toPixel(l, h);
    std::array<Vec2, 6> out{};
    for (int i = 0; i < 6; ++i) {
        const double a = (60.0 * i - (l.flat ? 0.0 : 30.0)) * 3.14159265358979323846 / 180.0;
        out[static_cast<std::size_t>(i)] = {c.x + l.size * std::cos(a), c.y + l.size * std::sin(a)};
    }
    return out;
}

/// The hex center must be at least `size` from the field border, so a whole hex fits.
inline bool inBounds(const Layout& l, const Rect& field, Cube h) {
    const Vec2 c = toPixel(l, h);
    return c.x >= field.x0 + l.size && c.x <= field.x1 - l.size && c.y >= field.y0 + l.size && c.y <= field.y1 - l.size;
}

// The playing field of the demo and its layouts.
inline constexpr Rect kField{352.0, 36.0, 948.0, 440.0};
inline constexpr double kHexSize = 26.0;
inline constexpr int kSearch = 14;  // cells() looks at q and r in (-kSearch, kSearch)

inline Layout makeLayout(bool flat) { return {flat, kHexSize, {(kField.x0 + kField.x1) / 2.0, (kField.y0 + kField.y1) / 2.0}}; }

/// All hexes of the field.
inline std::vector<Cube> cells(const Layout& l) {
    std::vector<Cube> out;
    for (int q = -kSearch + 1; q < kSearch; ++q) {
        for (int r = -kSearch + 1; r < kSearch; ++r) {
            const Cube h = fromAxial(q, r);
            if (inBounds(l, kField, h)) out.push_back(h);
        }
    }
    return out;
}

// Where the demo starts: the mouse sits close to a corner where three hexes meet, so the first frame shows a
// correction (the naive rounding gives the invalid triple (1, 0, 0)), and a line start that is in the field in both layouts.
inline constexpr Vec2 kStartMouse{668.0, 226.0};
inline constexpr Cube kStartLine{-3, 1, 2};

/// Straight interpolation in cube space.
inline Frac lerp(Frac a, Frac b, double t) { return {a.q + (b.q - a.q) * t, a.r + (b.r - a.r) * t, a.s + (b.s - a.s) * t}; }

/// The N + 1 points of the straight line from a to b (N = distance), before rounding. With `nudge` the whole line is
/// shifted by a hair (1e-6), so a sample that would sit exactly on the border of two hexes falls clearly on one side.
inline std::vector<Frac> lineSamples(Cube a, Cube b, bool nudge = true) {
    const int n = distance(a, b);
    const double e = nudge ? 1e-6 : 0.0;  // (e, e, -2e) sums to 0, the shifted line stays in the plane
    const Frac fa{a.q + e, a.r + e, a.s - 2.0 * e};
    const Frac fb{b.q + e, b.r + e, b.s - 2.0 * e};
    std::vector<Frac> out;
    out.reserve(static_cast<std::size_t>(n) + 1);
    for (int i = 0; i <= n; ++i) {
        out.push_back(n == 0 ? fa : lerp(fa, fb, static_cast<double>(i) / static_cast<double>(n)));
    }
    return out;
}

/// The hexes the line from a to b passes through, a and b included.
inline std::vector<Cube> line(Cube a, Cube b, bool nudge = true) {
    std::vector<Cube> out;
    for (const Frac& f : lineSamples(a, b, nudge)) out.push_back(cubeRound(f));
    return out;
}

}  // namespace hpr
