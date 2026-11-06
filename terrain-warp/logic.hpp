// Domain warping, the testable part: gradient noise, a fractal sum of it, and the warp that feeds noise with
// noise. No SDL in here. The warp is a lookup trick: the height at p is not f(p) but f(p + s * q(p)), where q is
// another noise field that says how far to look to the side. Everything stays a pure function of p and the seed.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace warp {

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};

/// The quintic 6t^5 - 15t^4 + 10t^3: slope and curvature are 0 at both ends, so cell borders do not show.
inline double fade(double t) { return t * t * t * (t * (t * 6.0 - 15.0) + 10.0); }

inline double lerp(double a, double b, double t) { return a + (b - a) * t; }

struct Cell {
    int index;
    double frac;
};

/// Lattice cell and position inside it. floor, not truncation: -0.25 is cell -1 at 0.75.
inline Cell cellOf(double v) {
    const double f = std::floor(v);
    return {static_cast<int>(f), v - f};
}

inline std::uint32_t hash(int x, int y, std::uint32_t seed) {
    std::uint32_t h = (seed * 0x9E3779B1U) ^ (static_cast<std::uint32_t>(x) * 0x85EBCA6BU) ^ (static_cast<std::uint32_t>(y) * 0xC2B2AE35U);
    h ^= h >> 16;
    h *= 0x7FEB352DU;
    h ^= h >> 15;
    h *= 0x846CA68BU;
    h ^= h >> 16;
    return h;
}

/// One of eight unit directions per lattice point.
inline Vec2 gradient(int ix, int iy, std::uint32_t seed) {
    constexpr double d = 0.70710678118654752;
    constexpr Vec2 kDirs[8] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {d, d}, {-d, d}, {d, -d}, {-d, -d}};
    return kDirs[hash(ix, iy, seed) >> 29];
}

/// The blend inside one cell: each corner contributes the dot product of its gradient with the vector from that
/// corner to the point, and the four are mixed with the fade curve.
inline double cornerBlend(Vec2 g00, Vec2 g10, Vec2 g01, Vec2 g11, double fx, double fy) {
    const double n00 = g00.x * fx + g00.y * fy;
    const double n10 = g10.x * (fx - 1.0) + g10.y * fy;
    const double n01 = g01.x * fx + g01.y * (fy - 1.0);
    const double n11 = g11.x * (fx - 1.0) + g11.y * (fy - 1.0);
    const double u = fade(fx);
    return lerp(lerp(n00, n10, u), lerp(n01, n11, u), fade(fy));
}

/// Gradient noise in [-1, 1]. With unit gradients the raw value stays within sqrt(2)/2, so scale by sqrt(2).
inline double noise(double x, double y, std::uint32_t seed) {
    constexpr double kSqrt2 = 1.4142135623730951;
    const Cell cx = cellOf(x);
    const Cell cy = cellOf(y);
    return kSqrt2 * cornerBlend(gradient(cx.index, cy.index, seed), gradient(cx.index + 1, cy.index, seed), gradient(cx.index, cy.index + 1, seed),
                                gradient(cx.index + 1, cy.index + 1, seed), cx.frac, cy.frac);
}

/// Octave `o` of the fractal sum: twice the frequency per octave, shifted so the lattice points of the octaves do not
/// all line up at the origin, and with its own seed.
inline double octaveNoise(double x, double y, std::uint32_t seed, int o) {
    const double freq = std::ldexp(1.0, o);
    return noise(x * freq + 17.31 * o, y * freq + 9.77 * o, seed + static_cast<std::uint32_t>(o) * 7919U);
}

/// Fractal sum of `octaves` noises, each with half the amplitude of the one before. Divided by the sum of the
/// amplitudes, so the result stays in [-1, 1] however many octaves there are.
inline double fbm(double x, double y, std::uint32_t seed, int octaves) {
    double sum = 0.0;
    double amplitude = 1.0;
    double total = 0.0;
    for (int o = 0; o < octaves; ++o) {
        sum += amplitude * octaveNoise(x, y, seed, o);
        total += amplitude;
        amplitude *= 0.5;
    }
    return total > 0.0 ? sum / total : 0.0;
}

/// fbm of [-1, 1] to a height in [0, 1]. The sum rarely reaches the ends, so stretch it by 1.6 around 0.5.
inline double toHeight(double v) { return std::clamp(0.5 + 0.8 * v, 0.0, 1.0); }

// Offsets that make the four warp fields different from each other (and from the terrain itself).
inline constexpr Vec2 kOffsetA{0.0, 0.0};
inline constexpr Vec2 kOffsetB{5.2, 1.3};
inline constexpr Vec2 kOffsetC{1.7, 9.2};
inline constexpr Vec2 kOffsetD{8.3, 2.8};

struct Params {
    int stages = 2;            // 0 = plain, 1 = f(p + s q), 2 = f(p + s r) with r = q(p + s q)
    double strength = 0.7;     // how far a lookup may jump, in lattice cells of the base frequency
    int octaves = 5;           // octaves of the terrain itself
    int warpOctaves = 3;       // octaves of the warp fields, fewer is enough since only the large shape matters
    std::uint32_t seed = 7;
};

/// Where the height at `p` is actually read from.
inline Vec2 warpPoint(Vec2 p, const Params& params) {
    if (params.stages <= 0 || params.strength == 0.0) return p;
    const double s = params.strength;
    const std::uint32_t seed = params.seed;
    const int o = params.warpOctaves;
    const Vec2 q{fbm(p.x + kOffsetA.x, p.y + kOffsetA.y, seed, o), fbm(p.x + kOffsetB.x, p.y + kOffsetB.y, seed, o)};
    if (params.stages == 1) return {p.x + s * q.x, p.y + s * q.y};
    const Vec2 shifted{p.x + s * q.x, p.y + s * q.y};
    const Vec2 r{fbm(shifted.x + kOffsetC.x, shifted.y + kOffsetC.y, seed, o), fbm(shifted.x + kOffsetD.x, shifted.y + kOffsetD.y, seed, o)};
    return {p.x + s * r.x, p.y + s * r.y};
}

/// Terrain height in [0, 1] at `p`.
inline double height(Vec2 p, const Params& params) {
    const Vec2 w = warpPoint(p, params);
    return toHeight(fbm(w.x, w.y, params.seed, params.octaves));
}

/// Share of the sample points (nx x ny cell centers over the rectangle) where `map` folds the plane onto itself:
/// the determinant of its Jacobian, measured with central differences, is 0 or negative. There the warped terrain shows
/// the same place twice, mirrored.
template <class Map>
double foldedFraction(Map map, double x0, double y0, double x1, double y1, int nx, int ny) {
    if (nx <= 0 || ny <= 0) return 0.0;
    const double stepX = (x1 - x0) / nx;
    const double stepY = (y1 - y0) / ny;
    const double eps = 0.25 * std::min(stepX, stepY);
    int folded = 0;
    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            const double x = x0 + (i + 0.5) * stepX;
            const double y = y0 + (j + 0.5) * stepY;
            const Vec2 xp = map({x + eps, y});
            const Vec2 xm = map({x - eps, y});
            const Vec2 yp = map({x, y + eps});
            const Vec2 ym = map({x, y - eps});
            const double det = ((xp.x - xm.x) * (yp.y - ym.y) - (yp.x - ym.x) * (xp.y - xm.y)) / (4.0 * eps * eps);
            if (det <= 0.0) ++folded;
        }
    }
    return static_cast<double>(folded) / (static_cast<double>(nx) * static_cast<double>(ny));
}

}  // namespace warp
