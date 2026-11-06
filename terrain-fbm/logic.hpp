// Terrain from noise: the logic of the demo, no SDL, so test.cpp can check it.
//
// Two lattice noises and the sum of several of them (fractal Brownian motion, fbm):
//
//   value noise     one random number per lattice point, blended in between. The extrema sit ON the lattice points,
//                   so the grid shows through as blocks and diamonds.
//   gradient noise  one random direction per lattice point; each corner contributes the dot product of its direction
//                   with the offset to the sample point. The value is 0 on every lattice point and the extrema sit
//                   between them, which hides the grid much better (this is what people call Perlin noise).
//
// THE FADE CURVE: the blend weight inside a cell is not t but fade(t). With the linear t the slope jumps at every cell
// border and the grid is visible. 3t^2 - 2t^3 has zero slope at both ends. 6t^5 - 15t^4 + 10t^3 additionally has zero
// curvature there, which is what makes gradient noise look smooth.
//
// FBM: noise(x * f_i) * a_i summed over the octaves, f_i = frequency * lacunarity^i and a_i = persistence^i. The sum of
// n layers reaches up to sum(a_i), so the result has to be divided by that or it leaves [0, 1] (normalize).
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace tn {

constexpr int kMaxOctaves = 8;
constexpr int kOctaveSeedStep = 1013;  // seeds of different octaves must not coincide
constexpr double kPanLimit = 5000.0;   // pixels; keeps coordinates far away from int overflow
constexpr double kTwoPi = 6.283185307179586;
constexpr double kSqrt2 = 1.4142135623730951;

enum class Fade { Linear, Cubic, Quintic };
enum class Kind { Value, Gradient };

struct Params {
    int octaves = 5;
    double lacunarity = 2.0;
    double persistence = 0.5;
    double frequency = 3.0;
    int seed = 7;
    Fade fade = Fade::Quintic;
    bool normalize = true;
};

struct Vec2 {
    double x, y;
};

inline double applyFade(Fade f, double t) {
    switch (f) {
        case Fade::Linear: return t;
        case Fade::Cubic: return t * t * (3.0 - 2.0 * t);
        case Fade::Quintic: return t * t * t * (t * (t * 6.0 - 15.0) + 10.0);
    }
    return t;
}

inline double lerp(double a, double b, double t) { return a + (b - a) * t; }

/// Hash of a lattice point and a seed, a murmur3 finalizer over the mixed inputs.
inline std::uint32_t latticeHash(int x, int y, int seed) {
    std::uint32_t h = static_cast<std::uint32_t>(x) * 0x8da6b343U ^ static_cast<std::uint32_t>(y) * 0xd8163841U ^ static_cast<std::uint32_t>(seed) * 0xcb1ab31fU;
    h += 0x9e3779b9U;
    h ^= h >> 16;
    h *= 0x85ebca6bU;
    h ^= h >> 13;
    h *= 0xc2b2ae35U;
    h ^= h >> 16;
    return h;
}

/// Random number in [0, 1) for a lattice point (24 bits, exactly representable).
inline double latticeValue(int x, int y, int seed) { return static_cast<double>(latticeHash(x, y, seed) >> 8) / 16777216.0; }

/// Random unit vector for a lattice point.
inline Vec2 latticeGradient(int x, int y, int seed) {
    const double a = latticeValue(x, y, seed) * kTwoPi;
    return {std::cos(a), std::sin(a)};
}

/// Coordinates come from the mouse and from frequency products. NaN becomes 0, huge values are clamped, so that the
/// floor below always fits an int.
inline double sane(double v) {
    if (!(v == v)) return 0.0;
    return std::clamp(v, -1e9, 1e9);
}

struct Cell {
    int ix, iy;
    double fx, fy;
};

inline Cell cellOf(double x, double y) {
    x = sane(x);
    y = sane(y);
    const double fl = std::floor(x);
    const double gl = std::floor(y);
    return {static_cast<int>(fl), static_cast<int>(gl), x - fl, y - gl};
}

/// Value noise in [0, 1].
inline double valueNoise(double x, double y, int seed, Fade fade) {
    const Cell c = cellOf(x, y);
    const double u = applyFade(fade, c.fx);
    const double v = applyFade(fade, c.fy);
    const double top = lerp(latticeValue(c.ix, c.iy, seed), latticeValue(c.ix + 1, c.iy, seed), u);
    const double bottom = lerp(latticeValue(c.ix, c.iy + 1, seed), latticeValue(c.ix + 1, c.iy + 1, seed), u);
    return lerp(top, bottom, v);
}

/// One cell of gradient noise from the four corner gradients, (fx, fy) is the position inside the cell.
/// The result is the raw blend of the four dot products (unit gradients: at most sqrt(2)/2 in size).
inline double gradientCell(Vec2 g00, Vec2 g10, Vec2 g01, Vec2 g11, double fx, double fy, Fade fade) {
    const double n00 = g00.x * fx + g00.y * fy;
    const double n10 = g10.x * (fx - 1.0) + g10.y * fy;
    const double n01 = g01.x * fx + g01.y * (fy - 1.0);
    const double n11 = g11.x * (fx - 1.0) + g11.y * (fy - 1.0);
    const double u = applyFade(fade, fx);
    const double v = applyFade(fade, fy);
    return lerp(lerp(n00, n10, u), lerp(n01, n11, u), v);
}

/// Gradient noise scaled to [-1, 1], exactly 0 on the lattice points.
inline double gradientNoise(double x, double y, int seed, Fade fade) {
    const Cell c = cellOf(x, y);
    const double raw = gradientCell(latticeGradient(c.ix, c.iy, seed), latticeGradient(c.ix + 1, c.iy, seed), latticeGradient(c.ix, c.iy + 1, seed),
                                    latticeGradient(c.ix + 1, c.iy + 1, seed), c.fx, c.fy, fade);
    return std::clamp(raw * kSqrt2, -1.0, 1.0);
}

/// The seed of one octave. With one shared seed every octave would be a compressed copy of the first (same hash for
/// the same lattice index).
inline int octaveSeed(int seed, int octave) { return seed + octave * kOctaveSeedStep; }

inline int clampOctaves(int n) { return std::clamp(n, 1, kMaxOctaves); }

/// Persistence outside [0, 1] (or NaN) is clamped, so the amplitude sum below is at least 1 and never 0.
inline double clampPersistence(double p) { return p >= 0.0 ? (p <= 1.0 ? p : 1.0) : 0.0; }

/// Sum of the octave amplitudes 1 + p + p^2 + ..., the largest value an unnormalized fbm of [0, 1] noise can reach.
inline double amplitudeSum(int octaves, double persistence) {
    const double p = clampPersistence(persistence);
    double sum = 0.0;
    double amp = 1.0;
    for (int i = 0; i < clampOctaves(octaves); ++i) {
        sum += amp;
        amp *= p;
    }
    return sum;
}

/// fbm over any source `noise(x, y, octave)` that returns values in [0, 1]. (x, y) is in "unit" coordinates, the
/// frequency scales it. Normalized, the result stays in [0, 1].
template <class Noise>
double fbm(Noise&& noise, double x, double y, const Params& p) {
    const double persistence = clampPersistence(p.persistence);
    double frequency = p.frequency;
    double amplitude = 1.0;
    double sum = 0.0;
    for (int i = 0; i < clampOctaves(p.octaves); ++i) {
        sum += amplitude * noise(x * frequency, y * frequency, i);
        frequency *= p.lacunarity;
        amplitude *= persistence;
    }
    return p.normalize ? sum / amplitudeSum(p.octaves, p.persistence) : sum;
}

/// One layer (a single octave) of the given kind in [0, 1].
inline double layerAt(const Params& p, Kind kind, int octave, double ux, double uy) {
    double f = p.frequency;
    for (int i = 0; i < octave; ++i) f *= p.lacunarity;
    const int seed = octaveSeed(p.seed, octave);
    return kind == Kind::Value ? valueNoise(ux * f, uy * f, seed, p.fade) : 0.5 + 0.5 * gradientNoise(ux * f, uy * f, seed, p.fade);
}

/// The terrain height at unit coordinates, in [0, 1] when normalized.
inline double heightAt(const Params& p, Kind kind, double ux, double uy) {
    return fbm(
        [&](double x, double y, int octave) {
            const int seed = octaveSeed(p.seed, octave);
            return kind == Kind::Value ? valueNoise(x, y, seed, p.fade) : 0.5 + 0.5 * gradientNoise(x, y, seed, p.fade);
        },
        ux, uy, p);
}

/// Panel pixel to unit coordinate. `pan` is the view offset in pixels.
inline double unitCoord(double px, double pan, double size) { return size > 0.0 ? (px + pan) / size : 0.0; }

/// Dragging the terrain by (dx, dy) moves the view by (-dx, -dy).
inline void panBy(double& panX, double& panY, double dx, double dy) {
    panX = std::clamp(panX - dx, -kPanLimit, kPanLimit);
    panY = std::clamp(panY - dy, -kPanLimit, kPanLimit);
}

/// Slider position t in [0, 1] to a value, snapped to `step` (0 = continuous).
inline double sliderValue(double t, double lo, double hi, double step) {
    t = (t == t) ? std::clamp(t, 0.0, 1.0) : 0.0;
    double v = lo + t * (hi - lo);
    if (step > 0.0) v = lo + std::round((v - lo) / step) * step;
    return std::clamp(v, lo, hi);
}

inline double tOfValue(double v, double lo, double hi) { return hi > lo ? std::clamp((v - lo) / (hi - lo), 0.0, 1.0) : 0.0; }

struct Rgb {
    std::uint8_t r, g, b;
};

inline std::uint8_t channel(double v) { return static_cast<std::uint8_t>(std::clamp(std::lround(v), 0L, 255L)); }

/// Water, sand, grass, forest, rock, snow. Linear between the stops, clamped outside [0, 1].
inline Rgb terrainColor(double h) {
    struct Stop {
        double h;
        double r, g, b;
    };
    static constexpr Stop kRamp[] = {{0.00, 18, 62, 102},   {0.38, 38, 120, 156},  {0.44, 70, 160, 180}, {0.45, 226, 208, 156}, {0.48, 214, 198, 150}, {0.49, 118, 166, 92},
                                     {0.60, 70, 120, 72},   {0.66, 120, 112, 104}, {0.72, 170, 164, 158}, {0.80, 245, 245, 245}, {1.00, 255, 255, 255}};
    constexpr std::size_t kStops = sizeof kRamp / sizeof kRamp[0];
    h = (h == h) ? std::clamp(h, 0.0, 1.0) : 0.0;
    for (std::size_t i = 1; i < kStops; ++i) {
        if (h <= kRamp[i].h) {
            const Stop& a = kRamp[i - 1];
            const Stop& b = kRamp[i];
            const double t = (h - a.h) / (b.h - a.h);  // stops are strictly increasing
            return {channel(lerp(a.r, b.r, t)), channel(lerp(a.g, b.g, t)), channel(lerp(a.b, b.b, t))};
        }
    }
    const Stop& last = kRamp[kStops - 1];
    return {channel(last.r), channel(last.g), channel(last.b)};
}

inline Rgb grayColor(double h) {
    h = (h == h) ? std::clamp(h, 0.0, 1.0) : 0.0;
    const std::uint8_t v = channel(h * 255.0);
    return {v, v, v};
}

/// Brightness factor from the height difference across the pixel, measured from the light towards the opposite
/// side (positive = the ground rises away from the light = it faces the light = lit). 0 = flat = unchanged.
inline double shadeFactor(double slope) {
    if (!(slope == slope)) return 1.0;
    return std::clamp(1.0 + 6.0 * slope, 0.6, 1.4);
}

inline Rgb shade(Rgb c, double f) { return {channel(c.r * f), channel(c.g * f), channel(c.b * f)}; }

}  // namespace tn
