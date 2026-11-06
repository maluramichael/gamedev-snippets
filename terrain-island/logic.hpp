// Island from noise: the logic of the demo, no SDL, so test.cpp can check it.
//
// An island is a height map that is high in the middle and low at the border. Take ordinary fractal noise and
// pull it down towards the border with a falloff that is 0 in the middle and 1 at the edge of the map:
//
//     subtract:  height = e - strength * f
//     lerp:      height = lerp(e, 1 - f, strength)
//
// Everything below the sea level is water. The falloff f = d^k comes from a distance d in [0, 1] (circle, square or
// diamond metric, clamped to 1 so the corners of the map do not exceed it) and a curve exponent k. Too much strength
// with lerp leaves nothing but the cone 1 - f, the noise is gone: that is the typical mistake the demo shows.
//
// The noise is a small gradient noise (Perlin style) written for this demo, summed over octaves (fBm).
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace isl {

struct Vec2 {
    float x;
    float y;
};

enum class Shape { Circle, Square, Diamond };
enum class Mode { Subtract, Lerp };
enum class Band { Deep, Shallow, Beach, Grass, Forest, Rock, Snow };

inline constexpr float kFrequency = 3.0F;   // noise cells across the map
inline constexpr int kOctaves = 5;
inline constexpr float kContrast = 1.8F;    // fBm rarely reaches +-1, this stretches it around 0.5 before clamping
inline constexpr float kLacunarity = 2.0F;  // frequency factor per octave
inline constexpr float kPersistence = 0.5F; // amplitude factor per octave

struct Params {
    std::uint32_t seed = 7;
    Shape shape = Shape::Circle;
    Mode mode = Mode::Lerp;
    float strength = 0.6F;
    float exponent = 1.5F;
};

struct Heightmap {
    int n = 0;
    std::vector<float> noise;    // e, the terrain before the falloff, 0..1
    std::vector<float> falloff;  // f, 0 in the middle, 1 at the border
    std::vector<float> height;   // after combining
};

inline float lerp(float a, float b, float t) { return a * (1.0F - t) + b * t; }  // exact at t = 0 and t = 1

inline float clamp01(float v) { return std::clamp(v, 0.0F, 1.0F); }

/// 6t^5 - 15t^4 + 10t^3: slope 0 at both ends, so the cells of the grid do not show.
inline float fade(float t) { return t * t * t * (t * (t * 6.0F - 15.0F) + 10.0F); }

inline std::uint32_t hash(int ix, int iy, std::uint32_t seed) {
    std::uint32_t h = seed * 374761393U + static_cast<std::uint32_t>(ix) * 668265263U + static_cast<std::uint32_t>(iy) * 2246822519U;
    h = (h ^ (h >> 13)) * 1274126177U;
    h = (h ^ (h >> 16)) * 2246822519U;
    return h ^ (h >> 15);
}

/// A unit vector for the lattice point, one of 16 directions.
inline Vec2 gradient(int ix, int iy, std::uint32_t seed) {
    static const std::array<Vec2, 16> kDirections = [] {
        std::array<Vec2, 16> d{};
        for (std::size_t i = 0; i < d.size(); ++i) {
            const float a = 6.2831853F * static_cast<float>(i) / 16.0F;
            d[i] = {std::cos(a), std::sin(a)};
        }
        return d;
    }();
    return kDirections[(hash(ix, iy, seed) >> 8) & 15U];
}

/// Gradient noise in [-1, 1], exactly 0 on the lattice points.
inline float noise(float x, float y, std::uint32_t seed) {
    if (!std::isfinite(x) || !std::isfinite(y)) return 0.0F;
    x = std::clamp(x, -1e6F, 1e6F);  // keeps the int conversion below in range
    y = std::clamp(y, -1e6F, 1e6F);
    const float fx = std::floor(x);
    const float fy = std::floor(y);
    const int ix = static_cast<int>(fx);
    const int iy = static_cast<int>(fy);
    const float tx = x - fx;
    const float ty = y - fy;
    const auto corner = [&](int cx, int cy, float dx, float dy) {
        const Vec2 g = gradient(ix + cx, iy + cy, seed);
        return g.x * dx + g.y * dy;
    };
    const float u = fade(tx);
    const float v = fade(ty);
    const float bottom = lerp(corner(0, 0, tx, ty), corner(1, 0, tx - 1.0F, ty), u);
    const float top = lerp(corner(0, 1, tx, ty - 1.0F), corner(1, 1, tx - 1.0F, ty - 1.0F), u);
    // Unit gradients give at most sqrt(2) / 2, so sqrt(2) scales to [-1, 1].
    return std::clamp(lerp(bottom, top, v) * 1.41421356F, -1.0F, 1.0F);
}

/// Sum of octaves, each with twice the frequency and half the amplitude, divided by the amplitudes: [-1, 1].
/// Every octave is shifted, otherwise all of them are 0 at the same lattice points and the sum shows a faint grid.
inline float fbm(float x, float y, int octaves, std::uint32_t seed) {
    octaves = std::clamp(octaves, 1, 8);
    float sum = 0.0F;
    float total = 0.0F;
    float amplitude = 1.0F;
    float frequency = 1.0F;
    for (int o = 0; o < octaves; ++o) {
        const float shift = static_cast<float>(o);
        sum += amplitude * noise(x * frequency + shift * 37.13F, y * frequency + shift * 17.71F, seed);
        total += amplitude;
        amplitude *= kPersistence;
        frequency *= kLacunarity;
    }
    return sum / total;
}

/// Distance from the middle of the map, nx and ny in [-1, 1]. Clamped to 1 so the corners do not exceed the border.
inline float distance(Shape shape, float nx, float ny) {
    switch (shape) {
        case Shape::Circle: return std::min(1.0F, std::sqrt(nx * nx + ny * ny));
        case Shape::Square: return std::min(1.0F, std::max(std::fabs(nx), std::fabs(ny)));
        case Shape::Diamond: return std::min(1.0F, std::fabs(nx) + std::fabs(ny));
    }
    return 1.0F;
}

/// f = d^k: 0 in the middle, 1 at the border. A larger exponent keeps the middle flat and drops steeply at the edge.
inline float falloff(Shape shape, float nx, float ny, float exponent) {
    return std::pow(distance(shape, nx, ny), std::clamp(exponent, 0.1F, 8.0F));
}

/// e: the terrain, f: the falloff, both 0..1. The result is clamped to 0..1.
inline float combine(Mode mode, float e, float f, float strength) {
    strength = clamp01(strength);
    return clamp01(mode == Mode::Subtract ? e - strength * f : lerp(e, 1.0F - f, strength));
}

/// Center of cell i in an n x n grid, mapped to [-1, 1]. For an odd n the middle cell is exactly 0.
inline float cellToNorm(int i, int n) { return (2.0F * static_cast<float>(i) + 1.0F) / static_cast<float>(n) - 1.0F; }

/// The grid row under a screen y, clamped to the grid.
inline int rowForY(float py, float top, float size, int n) {
    if (n <= 0 || size <= 0.0F) return 0;
    const float row = std::floor((py - top) * static_cast<float>(n) / size);
    return static_cast<int>(std::clamp(row, 0.0F, static_cast<float>(n - 1)));
}

/// The value of a slider whose track starts at x0 and is w wide.
inline float sliderValue(float px, float x0, float w, float lo, float hi) {
    if (w <= 0.0F) return lo;
    return lo + (hi - lo) * std::clamp((px - x0) / w, 0.0F, 1.0F);
}

inline Heightmap build(const Params& p, int n) {
    Heightmap h;
    n = std::clamp(n, 1, 1024);
    h.n = n;
    const auto size = static_cast<std::size_t>(n) * static_cast<std::size_t>(n);
    h.noise.resize(size);
    h.falloff.resize(size);
    h.height.resize(size);
    for (int y = 0; y < n; ++y) {
        for (int x = 0; x < n; ++x) {
            const float nx = cellToNorm(x, n);
            const float ny = cellToNorm(y, n);
            const float e = clamp01(0.5F + 0.5F * kContrast * fbm((nx * 0.5F + 0.5F) * kFrequency, (ny * 0.5F + 0.5F) * kFrequency, kOctaves, p.seed));
            const float f = falloff(p.shape, nx, ny, p.exponent);
            const auto i = static_cast<std::size_t>(y * n + x);
            h.noise[i] = e;
            h.falloff[i] = f;
            h.height[i] = combine(p.mode, e, f, p.strength);
        }
    }
    return h;
}

/// Exactly at sea level counts as land, so the coast line has a beach and not a gap.
inline bool isLand(float height, float sea) { return height >= sea; }

inline float landFraction(const std::vector<float>& grid, float sea) {
    if (grid.empty()) return 0.0F;
    const auto land = std::count_if(grid.begin(), grid.end(), [sea](float v) { return isLand(v, sea); });
    return static_cast<float>(land) / static_cast<float>(grid.size());
}

/// True if any cell of the outermost ring is land: the island is cut off by the edge of the map.
inline bool touchesBorder(const std::vector<float>& grid, int n, float sea) {
    if (n < 1 || grid.size() != static_cast<std::size_t>(n) * static_cast<std::size_t>(n)) return false;
    for (int i = 0; i < n; ++i) {
        const auto top = static_cast<std::size_t>(i);
        const auto bottom = static_cast<std::size_t>((n - 1) * n + i);
        const auto left = static_cast<std::size_t>(i * n);
        const auto right = static_cast<std::size_t>(i * n + n - 1);
        if (isLand(grid[top], sea) || isLand(grid[bottom], sea) || isLand(grid[left], sea) || isLand(grid[right], sea)) return true;
    }
    return false;
}

/// Height bands relative to the sea level, so they follow the slider.
inline Band classify(float height, float sea) {
    const float above = height - sea;
    if (above < -0.12F) return Band::Deep;
    if (above < 0.0F) return Band::Shallow;
    if (above < 0.03F) return Band::Beach;
    if (above < 0.10F) return Band::Grass;
    if (above < 0.25F) return Band::Forest;
    if (above < 0.40F) return Band::Rock;
    return Band::Snow;
}

}  // namespace isl
