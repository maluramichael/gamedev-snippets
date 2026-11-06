// Terrain shaping without SDL: gradient noise, fractal sum, the monotone height curve, terraces, the ridge mask and the
// two fields (smooth and ridged) the demo reshapes. Everything that can be wrong lives here and is covered by test.cpp.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace ts {

/// Largest |gradient noise| for unit gradients: all four corner contributions line up in the middle of a cell.
inline constexpr double kNoiseMax = 0.70710678118654752440;

/// The raw sum clusters around the middle, so the demo stretches it with fixed constants (not per picture, or the
/// image would jump while panning). `--stats` in test.cpp prints the numbers these were taken from.
inline constexpr double kStretch = 1.7;
inline constexpr double kRidgeLo = 0.15;
inline constexpr double kRidgeHi = 1.05;

inline constexpr double kRidgeAmount = 0.6;  // how far a crest (ridge value 1) lifts or a crease (0) sinks the height, at full mask
inline constexpr double kRiser = 0.35;        // share of each level that is the sloping step between two terraces
inline constexpr double kValleysPower = 2.0;  // the "Valleys" preset samples x^2
inline constexpr double kSeaLevel = 0.15;
inline constexpr int kCurvePoints = 5;
inline constexpr int kHistBins = 32;

/// NaN becomes 0 (a plain std::clamp would pass it through and the int casts further down would be undefined).
inline double clamp01(double v) { return v > 0.0 ? (v < 1.0 ? v : 1.0) : 0.0; }
inline double lerp(double a, double b, double t) { return a + (b - a) * t; }

/// 6t^5 - 15t^4 + 10t^3: slope and curvature are 0 at both ends, so neighbouring cells meet without a visible grid.
inline double fade(double t) { return t * t * t * (t * (t * 6.0 - 15.0) + 10.0); }

inline double smoothstep(double e0, double e1, double x) {
    if (!(e1 > e0)) return x >= e0 ? 1.0 : 0.0;  // empty or reversed interval: a hard step, no division by zero
    const double t = clamp01((x - e0) / (e1 - e0));
    return t * t * (3.0 - 2.0 * t);
}

// ---- gradient noise --------------------------------------------------------------------------------------------

struct Vec2 {
    double x;
    double y;
};

inline std::uint32_t hash2(int ix, int iy, std::uint32_t seed) {
    std::uint32_t h = seed * 0x9E3779B1U ^ static_cast<std::uint32_t>(ix) * 0x27D4EB2DU ^ static_cast<std::uint32_t>(iy) * 0x165667B1U;
    h ^= h >> 15;
    h *= 0x2C1B3C6DU;
    h ^= h >> 12;
    h *= 0x297A2D39U;
    h ^= h >> 15;
    return h;
}

/// One of eight unit gradients. Unit length matters: it is what makes kNoiseMax the bound.
inline Vec2 gradient(std::uint32_t h) {
    constexpr double s = 0.70710678118654752440;
    static constexpr std::array<Vec2, 8> table{{{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {s, s}, {-s, s}, {s, -s}, {-s, -s}}};
    return table[h & 7U];
}

/// The four dot products of a cell (corner gradient . offset to the point) blended with the faded weights.
inline double blend(double d00, double d10, double d01, double d11, double fx, double fy) {
    const double u = fade(fx);
    const double v = fade(fy);
    return lerp(lerp(d00, d10, u), lerp(d01, d11, u), v);
}

/// Gradient (Perlin) noise, 0 on every lattice point, within +-kNoiseMax. Coordinates are clamped to +-1e6 so the
/// int cast is defined, garbage (NaN, infinity) gives 0.
inline double gradientNoise(double x, double y, std::uint32_t seed) {
    if (!std::isfinite(x) || !std::isfinite(y)) return 0.0;
    x = std::clamp(x, -1e6, 1e6);
    y = std::clamp(y, -1e6, 1e6);
    const double fx = std::floor(x);
    const double fy = std::floor(y);
    const int ix = static_cast<int>(fx);
    const int iy = static_cast<int>(fy);
    const double dx = x - fx;
    const double dy = y - fy;
    const auto dot = [&](int cx, int cy, double ox, double oy) {
        const Vec2 g = gradient(hash2(cx, cy, seed));
        return g.x * ox + g.y * oy;
    };
    return blend(dot(ix, iy, dx, dy), dot(ix + 1, iy, dx - 1.0, dy), dot(ix, iy + 1, dx, dy - 1.0), dot(ix + 1, iy + 1, dx - 1.0, dy - 1.0), dx, dy);
}

struct Octaves {
    double base;    // fractal sum, -1..1
    double ridged;  // sum of (1 - |n|)^2 over the detail octaves, 0..1: sharp crests where the noise crosses zero
};

/// Fractal sum: every octave doubles the frequency and halves the amplitude, divided by the amplitude sum so the
/// result stays in range for any octave count. Each octave gets its own seed and a shift, otherwise every octave would
/// be 0 at the origin and the origin would look special. The ridged sum uses the same noise values but leaves out the
/// first octave (unless it is the only one): the first octave shapes the big hills, and ridges taken from it would
/// sit exactly where the smooth field peaks (a peak is a large |n|, the ridge value 1 - |n| is smallest there).
inline Octaves sample(double x, double y, int octaves, std::uint32_t seed) {
    octaves = std::clamp(octaves, 1, 8);
    double freq = 1.0;
    double amp = 1.0;
    double norm = 0.0;
    double sum = 0.0;
    double ridgeSum = 0.0;
    double ridgeNorm = 0.0;
    for (int i = 0; i < octaves; ++i) {
        const double n = gradientNoise(x * freq + 37.13 * i, y * freq + 91.71 * i, seed + 0x85EBCA6BU * static_cast<std::uint32_t>(i)) / kNoiseMax;
        const double r = 1.0 - std::fabs(n);
        sum += amp * n;
        norm += amp;
        if (i > 0 || octaves == 1) {
            ridgeSum += amp * r * r;
            ridgeNorm += amp;
        }
        freq *= 2.0;
        amp *= 0.5;
    }
    return {sum / norm, ridgeSum / ridgeNorm};
}

inline double baseToHeight(double base) { return clamp01(0.5 + 0.5 * kStretch * base); }
inline double ridgeToHeight(double ridged) { return clamp01((ridged - kRidgeLo) / (kRidgeHi - kRidgeLo)); }

// ---- the height curve ------------------------------------------------------------------------------------------

using CurveY = std::array<double, kCurvePoints>;

inline CurveY linearCurve() { return {0.0, 0.25, 0.5, 0.75, 1.0}; }

inline CurveY valleysCurve() {
    CurveY y{};
    for (std::size_t i = 0; i < y.size(); ++i) y[i] = std::pow(static_cast<double>(i) / (kCurvePoints - 1), kValleysPower);
    return y;
}

inline CurveY sCurve() {
    CurveY y{};
    for (std::size_t i = 0; i < y.size(); ++i) y[i] = smoothstep(0.0, 1.0, static_cast<double>(i) / (kCurvePoints - 1));
    return y;
}

namespace detail {
inline int sign(double v) { return v > 0.0 ? 1 : (v < 0.0 ? -1 : 0); }
}  // namespace detail

/// Monotone cubic through five evenly spaced nodes (Fritsch-Carlson / PCHIP slopes). Monotone data gives a monotone
/// curve without overshoot, so a hill never turns into a dip between two nodes.
struct Curve {
    CurveY y{0.0, 0.25, 0.5, 0.75, 1.0};
    std::array<double, kCurvePoints> m{1.0, 1.0, 1.0, 1.0, 1.0};

    double operator()(double x) const {
        constexpr double h = 1.0 / (kCurvePoints - 1);
        x = clamp01(x);
        const auto k = static_cast<std::size_t>(std::min(static_cast<int>(x * (kCurvePoints - 1)), kCurvePoints - 2));  // 0..3, x in 0..1
        const double t = (x - static_cast<double>(k) * h) / h;
        const double t2 = t * t;
        const double t3 = t2 * t;
        return (2.0 * t3 - 3.0 * t2 + 1.0) * y[k] + (t3 - 2.0 * t2 + t) * h * m[k] + (-2.0 * t3 + 3.0 * t2) * y[k + 1] + (t3 - t2) * h * m[k + 1];
    }
};

inline Curve makeCurve(const CurveY& y) {
    constexpr double h = 1.0 / (kCurvePoints - 1);
    Curve c;
    c.y = y;
    std::array<double, kCurvePoints - 1> d{};
    for (std::size_t k = 0; k + 1 < y.size(); ++k) d[k] = (y[k + 1] - y[k]) / h;
    for (std::size_t k = 1; k + 1 < y.size(); ++k) {
        // A flat or turning neighbour forces slope 0; otherwise the harmonic mean keeps both sides from overshooting.
        c.m[k] = d[k - 1] * d[k] <= 0.0 ? 0.0 : 2.0 * d[k - 1] * d[k] / (d[k - 1] + d[k]);
    }
    const auto endSlope = [](double d0, double d1) {
        double m = (3.0 * d0 - d1) / 2.0;
        if (detail::sign(m) != detail::sign(d0)) return 0.0;
        if (detail::sign(d0) != detail::sign(d1) && std::fabs(m) > 3.0 * std::fabs(d0)) return 3.0 * d0;
        return m;
    };
    c.m[0] = endSlope(d[0], d[1]);
    c.m[kCurvePoints - 1] = endSlope(d[kCurvePoints - 2], d[kCurvePoints - 3]);
    return c;
}

/// Moves one node vertically. A node never passes its neighbours (that would fold the terrain over itself) and the
/// ends stay inside 0..1. Out-of-range indices and NaN change nothing.
inline void setCurvePoint(CurveY& y, int index, double value) {
    if (index < 0 || index >= kCurvePoints || std::isnan(value)) return;
    const auto i = static_cast<std::size_t>(index);
    const double lo = i == 0 ? 0.0 : y[i - 1];
    const double hi = i + 1 == y.size() ? 1.0 : y[i + 1];
    y[i] = std::clamp(value, lo, hi);
}

// ---- terraces and the mix --------------------------------------------------------------------------------------

/// Splits 0..1 into `steps` levels. Each level is a flat tread and a smoothstep riser in the middle of the step; `riser`
/// is the width of that slope as a share of the level (0 = a vertical cliff, 1 = no flat part at all). At the middle
/// of every riser the result equals the input, so the overall slope of the terrain survives.
inline double terrace(double h, int steps, double riser) {
    h = clamp01(h);
    steps = std::max(steps, 1);
    const double t = h * steps;
    const double level = std::floor(t);
    const double half = 0.5 * clamp01(riser);
    return (level + smoothstep(0.5 - half, 0.5 + half, t - level)) / steps;
}

/// 0 in the lowlands, 1 from height 0.8: the ridge detail only shows on the summits.
inline double ridgeMask(double h) { return smoothstep(0.5, 0.8, h); }

struct Shape {
    Curve curve = makeCurve(linearCurve());
    bool ridged = false;
    bool naive = false;  // ridge detail everywhere, terraces as vertical cliffs
    bool terraces = false;
    int steps = 6;
};

/// Input of the curve: the smooth field. With ridges on, (ridge - 0.5) is added on top, weighted by the mask (or
/// everywhere when naive). The ridge field is centred on 0.5, so the terrain is neither lifted nor sunk on average,
/// the zero crossings of the noise just turn into crests and the humps between them into creases.
inline double curveInput(const Shape& s, double base, double ridge) {
    base = clamp01(base);
    if (!s.ridged) return base;
    const double weight = s.naive ? 1.0 : ridgeMask(base);
    return clamp01(base + weight * (clamp01(ridge) - 0.5) * kRidgeAmount);
}

inline double shapeHeight(const Shape& s, double base, double ridge) {
    double h = s.curve(curveInput(s, base, ridge));
    if (s.terraces) h = terrace(h, s.steps, s.naive ? 0.0 : kRiser);
    return h;
}

// ---- fields -----------------------------------------------------------------------------------------------------

struct Params {
    int octaves = 5;
    std::uint32_t seed = 1;
    double offX = 0.0;   // pan, in lattice units of the first octave
    double offY = 0.0;
    double span = 4.0;   // lattice cells of the first octave across the width
};

struct Fields {
    int w = 0;
    int h = 0;
    std::vector<float> base;   // smooth field as height 0..1
    std::vector<float> ridge;  // ridged field as height 0..1
};

/// Cells are square: both axes use the width for the scale. A cell holds the sample at its centre.
inline void computeFields(Fields& f, const Params& p, int w, int h) {
    f.w = std::max(w, 0);
    f.h = std::max(h, 0);
    const auto n = static_cast<std::size_t>(f.w) * static_cast<std::size_t>(f.h);
    f.base.assign(f.w > 0 ? n : 0, 0.0F);
    f.ridge.assign(f.w > 0 ? n : 0, 0.0F);
    if (f.w == 0 || f.h == 0) {
        f.base.clear();
        f.ridge.clear();
        return;
    }
    for (int y = 0; y < f.h; ++y) {
        for (int x = 0; x < f.w; ++x) {
            const double sx = p.offX + (x + 0.5) / f.w * p.span;
            const double sy = p.offY + (y + 0.5) / f.w * p.span;
            const Octaves o = sample(sx, sy, p.octaves, p.seed);
            const auto i = static_cast<std::size_t>(y) * static_cast<std::size_t>(f.w) + static_cast<std::size_t>(x);
            f.base[i] = static_cast<float>(baseToHeight(o.base));
            f.ridge[i] = static_cast<float>(ridgeToHeight(o.ridged));
        }
    }
}

using Histogram = std::vector<int>;

inline Histogram histogram(const std::vector<float>& values, int bins) {
    bins = std::max(bins, 1);
    Histogram out(static_cast<std::size_t>(bins), 0);
    for (float v : values) {
        const double c = clamp01(static_cast<double>(v));
        const int idx = std::min(static_cast<int>(c * bins), bins - 1);
        ++out[static_cast<std::size_t>(idx)];
    }
    return out;
}

/// Applies the shape to every cell and counts the heights before the curve (`before`) and after everything (`after`).
inline void shapeFields(const Fields& f, const Shape& s, std::vector<float>& out, Histogram& before, Histogram& after) {
    out.resize(f.base.size());
    std::vector<float> in(f.base.size());
    for (std::size_t i = 0; i < f.base.size(); ++i) {
        in[i] = static_cast<float>(curveInput(s, f.base[i], f.ridge[i]));
        out[i] = static_cast<float>(shapeHeight(s, f.base[i], f.ridge[i]));
    }
    before = histogram(in, kHistBins);
    after = histogram(out, kHistBins);
}

// ---- shading -----------------------------------------------------------------------------------------------------

/// Lambert brightness 0..1 of the surface between four neighbouring heights. Light comes from the upper left
/// (screen y grows downwards), direction (-1, -1, 2) normalised, so a flat tile has brightness 2/sqrt(6).
inline double shade(double hL, double hR, double hU, double hD, double scale) {
    const double nx = -(hR - hL) * 0.5 * scale;
    const double ny = -(hD - hU) * 0.5 * scale;
    const double len = std::sqrt(nx * nx + ny * ny + 1.0);
    const double lambert = (-nx - ny + 2.0) / (len * std::sqrt(6.0));
    return clamp01(lambert);
}

}  // namespace ts
