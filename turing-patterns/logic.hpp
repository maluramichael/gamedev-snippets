// Turing patterns: the logic of the demo, no SDL, so test.cpp can check it.
//
// Two virtual chemicals A and B live on a grid (a torus, the edges wrap). The Gray-Scott model:
//
//   dA/dt = dA * laplace(A) - A * B^2 + feed * (1 - A)
//   dB/dt = dB * laplace(B) + A * B^2 - (kill + feed) * B
//
// Diffusion spreads both out, A faster than B. The reaction A + 2B -> 3B lets B eat A and multiply. A is fed in at
// rate `feed`, B is removed at rate `kill`. That is all, and depending on two small numbers it makes spots, stripes,
// coral or cells that divide.
//
// Explicit Euler: every cell moves by dt * (rate). Two ways to get that wrong, both switchable in the demo:
//
//   * Update::InPlace writes each cell straight back into the grid. Cells after it in scan order then read new values
//     of their neighbours, a mix of two time steps. The result depends on the scan direction, so a mirror symmetric
//     start (the single seed of the mitosis preset) loses its symmetry. Update::Buffered reads only the old grid and
//     writes a second one.
//   * dt too large. The 3x3 kernel (-1 in the middle, 0.2 direct, 0.05 diagonal neighbours) has its worst eigenvalue
//     at the checkerboard: -1 - 4 * 0.2 + 4 * 0.05 = -1.6. One step multiplies a checkerboard by 1 - 1.6 * D * dt,
//     which grows once dt > 2 / (1.6 * D). With D = 1 that is dt = 1.25. `maxStableDt` computes it. The limit is the one
//     of the buffered update; in place a cell already sees the damped values of its neighbours and survives more.
//
// After each step both chemicals are clamped to [0, 1] (the true solution never leaves that range). A NaN becomes 0,
// so a bad value cannot spread over the grid.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace tp {

// Range of the feed/kill picker, the part of the plane where the interesting patterns live.
inline constexpr float kFeedMin = 0.01F;
inline constexpr float kFeedMax = 0.1F;
inline constexpr float kKillMin = 0.045F;
inline constexpr float kKillMax = 0.07F;

inline constexpr float kAdjacent = 0.2F;   // Laplace weight of the four direct neighbours
inline constexpr float kDiagonal = 0.05F;  // Laplace weight of the four diagonal neighbours
inline constexpr float kWorstEigenvalue = 1.0F + 4.0F * kAdjacent - 4.0F * kDiagonal;  // magnitude, at the checkerboard

struct Params {
    float feed = 0.055F;
    float kill = 0.062F;
    float dA = 1.0F;  // A spreads twice as fast as B, this difference is what makes patterns possible
    float dB = 0.5F;
    float dt = 1.0F;
};

enum class Update { Buffered, InPlace };

struct Field {
    int w = 0;
    int h = 0;
    std::vector<float> a;
    std::vector<float> b;
    std::vector<float> a2;  // second buffer for Update::Buffered
    std::vector<float> b2;

    Field(int width, int height) : w(std::max(width, 0)), h(std::max(height, 0)) {
        const auto n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
        a.assign(n, 1.0F);
        b.assign(n, 0.0F);
        a2.assign(n, 1.0F);
        b2.assign(n, 0.0F);
    }

    std::size_t index(int x, int y) const { return static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x); }
};

struct Preset {
    const char* name;
    float feed;
    float kill;
    int seeds;     // how many blocks of B to scatter, 1 means a single block in the middle
    int seedSize;  // side length of a block in cells
};

// Feed and kill: Sims' coral and mitosis values, the other two found by trying (see README).
inline constexpr std::array<Preset, 4> kPresets{{{"Spots", 0.022F, 0.059F, 30, 6}, {"Stripes", 0.078F, 0.061F, 30, 6}, {"Coral", 0.0545F, 0.062F, 30, 6}, {"Mitosis", 0.0367F, 0.0649F, 1, 16}}};

/// Clamp to [0, 1]; NaN becomes 0 (both comparisons are false for NaN).
inline float clamp01(float v) { return v > 0.0F ? (v < 1.0F ? v : 1.0F) : 0.0F; }

namespace detail {

inline int wrap(int v, int n) { return v < 0 ? v + n : (v >= n ? v - n : v); }

/// The 3x3 kernel at column x of a row. `up`, `mid` and `down` are the offsets of the three rows, `xm` and `xp` the
/// columns left and right of x (already wrapped).
inline float kernel(const float* v, std::size_t up, std::size_t mid, std::size_t down, std::size_t xm, std::size_t x, std::size_t xp) {
    return -v[mid + x] + kAdjacent * (v[mid + xm] + v[mid + xp] + v[up + x] + v[down + x]) +
           kDiagonal * (v[up + xm] + v[up + xp] + v[down + xm] + v[down + xp]);
}

inline std::size_t rowOffset(int y, int h, int w) { return static_cast<std::size_t>(wrap(y, h)) * static_cast<std::size_t>(w); }

inline float laplace(const float* v, int w, int h, int x, int y) {
    return kernel(v, rowOffset(y - 1, h, w), rowOffset(y, h, w), rowOffset(y + 1, h, w), static_cast<std::size_t>(wrap(x - 1, w)),
                  static_cast<std::size_t>(x), static_cast<std::size_t>(wrap(x + 1, w)));
}

/// One pass over the grid. Reads from (ra, rb) and writes to (wa, wb); for the in-place variant they are the same arrays.
/// The rows above and below are looked up once per row, only the first and last column need the wrap.
inline void pass(const Field& f, const Params& p, const float* ra, const float* rb, float* wa, float* wb) {
    const float k = p.kill + p.feed;
    const int w = f.w;
    for (int y = 0; y < f.h; ++y) {
        const std::size_t up = rowOffset(y - 1, f.h, w);
        const std::size_t mid = rowOffset(y, f.h, w);
        const std::size_t down = rowOffset(y + 1, f.h, w);
        auto cell = [&](int x, int xm, int xp) {
            const auto ux = static_cast<std::size_t>(x);
            const auto uxm = static_cast<std::size_t>(xm);
            const auto uxp = static_cast<std::size_t>(xp);
            const std::size_t i = mid + ux;
            const float a = ra[i];
            const float b = rb[i];
            const float abb = a * b * b;
            const float lapA = kernel(ra, up, mid, down, uxm, ux, uxp);
            const float lapB = kernel(rb, up, mid, down, uxm, ux, uxp);
            wa[i] = clamp01(a + p.dt * (p.dA * lapA - abb + p.feed * (1.0F - a)));
            wb[i] = clamp01(b + p.dt * (p.dB * lapB + abb - k * b));
        };
        cell(0, w - 1, w > 1 ? 1 : 0);
        for (int x = 1; x < w - 1; ++x) cell(x, x - 1, x + 1);
        if (w > 1) cell(w - 1, w - 2, 0);
    }
}

}  // namespace detail

inline float laplaceA(const Field& f, int x, int y) { return detail::laplace(f.a.data(), f.w, f.h, x, y); }
inline float laplaceB(const Field& f, int x, int y) { return detail::laplace(f.b.data(), f.w, f.h, x, y); }

/// One explicit Euler step. Buffered reads the old grid and writes the second one, then swaps.
inline void step(Field& f, const Params& p, Update mode) {
    if (f.w <= 0 || f.h <= 0) return;
    if (mode == Update::InPlace) {
        detail::pass(f, p, f.a.data(), f.b.data(), f.a.data(), f.b.data());
        return;
    }
    detail::pass(f, p, f.a.data(), f.b.data(), f.a2.data(), f.b2.data());
    f.a.swap(f.a2);
    f.b.swap(f.b2);
}

/// Largest dt for which diffusion alone does not amplify a checkerboard. The reaction has its own limit, this is the one
/// the demo can show: dt * D * 1.6 < 2.
inline float maxStableDt(const Params& p) {
    const float d = std::max(p.dA, p.dB);
    return d <= 0.0F ? std::numeric_limits<float>::infinity() : 2.0F / (kWorstEigenvalue * d);
}

/// Kill rate above which no uniform state with B in it exists for a given feed: k = sqrt(f) / 2 - f.
inline float killLimit(float feed) { return feed <= 0.0F ? 0.0F : 0.5F * std::sqrt(feed) - feed; }

struct Steady {
    float a;
    float b;
};

/// Uniform non-trivial steady states of the reaction, the roots of (f + k) B^2 - f B + f (f + k) = 0 with A = (f + k) / B.
/// Returns how many there are (0, 1 or 2) and writes them with the larger B first.
inline int steadyStates(float feed, float kill, Steady* out) {
    const double f = feed;
    const double s = f + static_cast<double>(kill);
    if (f <= 0.0 || s <= 0.0) return 0;
    const double disc = f * f - 4.0 * f * s * s;
    if (disc < 0.0) return 0;
    const double root = std::sqrt(disc);
    const double b1 = (f + root) / (2.0 * s);
    const double b2 = (f - root) / (2.0 * s);
    out[0] = {static_cast<float>(s / b1), static_cast<float>(b1)};
    if (disc == 0.0) return 1;
    out[1] = {static_cast<float>(s / b2), static_cast<float>(b2)};
    return 2;
}

/// Same small deterministic generator everywhere, so a seed always gives the same field.
inline std::uint32_t nextRandom(std::uint32_t& state) {
    state ^= state << 13;
    state ^= state >> 17;
    state ^= state << 5;
    return state;
}

/// Reset to the empty state: A = 1 everywhere, no B.
inline void clear(Field& f) {
    std::fill(f.a.begin(), f.a.end(), 1.0F);
    std::fill(f.b.begin(), f.b.end(), 0.0F);
}

/// `count` square blocks of B (and a bit less A) at random places, each `size` cells wide. Blocks are cut at the edge.
inline void seed(Field& f, std::uint32_t seedValue, int count, int size) {
    if (f.w <= 0 || f.h <= 0 || count <= 0 || size <= 0) return;
    std::uint32_t state = seedValue == 0 ? 1U : seedValue;  // xorshift must not start at 0
    for (int n = 0; n < count; ++n) {
        const int x0 = static_cast<int>(nextRandom(state) % static_cast<std::uint32_t>(f.w));
        const int y0 = static_cast<int>(nextRandom(state) % static_cast<std::uint32_t>(f.h));
        for (int y = y0; y < std::min(y0 + size, f.h); ++y) {
            for (int x = x0; x < std::min(x0 + size, f.w); ++x) {
                f.a[f.index(x, y)] = 0.5F;
                f.b[f.index(x, y)] = 1.0F;
            }
        }
    }
}

/// Start a preset: empty field, then its seeds. One seed is a block in the middle (the cell that divides), more are
/// scattered at random places.
inline void plant(Field& f, const Preset& preset, std::uint32_t seedValue) {
    clear(f);
    if (preset.seeds == 1) {
        const int x0 = std::max((f.w - preset.seedSize) / 2, 0);
        const int y0 = std::max((f.h - preset.seedSize) / 2, 0);
        for (int y = y0; y < std::min(y0 + preset.seedSize, f.h); ++y) {
            for (int x = x0; x < std::min(x0 + preset.seedSize, f.w); ++x) {
                f.a[f.index(x, y)] = 0.5F;
                f.b[f.index(x, y)] = 1.0F;
            }
        }
        return;
    }
    seed(f, seedValue, preset.seeds, preset.seedSize);
}

namespace detail {

inline void disc(Field& f, int cx, int cy, int radius, float a, float b) {
    if (radius < 0 || cx < 0 || cy < 0 || cx >= f.w || cy >= f.h) return;
    for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
            if (dx * dx + dy * dy > radius * radius) continue;
            const std::size_t i = f.index(((cx + dx) % f.w + f.w) % f.w, ((cy + dy) % f.h + f.h) % f.h);
            f.a[i] = a;
            f.b[i] = b;
        }
    }
}

}  // namespace detail

/// Brush: a disc of B around (cx, cy), wrapped around the edges like the grid itself. A centre outside the grid is ignored.
inline void paint(Field& f, int cx, int cy, int radius) { detail::disc(f, cx, cy, radius, 0.5F, 1.0F); }

/// Eraser: back to the empty state.
inline void erase(Field& f, int cx, int cy, int radius) { detail::disc(f, cx, cy, radius, 1.0F, 0.0F); }

/// Share of cells with more than `threshold` of B.
inline float coverage(const Field& f, float threshold) {
    if (f.b.empty()) return 0.0F;
    std::size_t n = 0;
    for (float v : f.b) n += v > threshold ? 1U : 0U;
    return static_cast<float>(n) / static_cast<float>(f.b.size());
}

/// Darkness of a cell for drawing, 0 = empty paper, 1 = full pattern. Sims' trick: A - B, so both chemicals show.
inline float level(float a, float b) { return clamp01(1.0F - (a - b)); }

/// The feed/kill picker: k runs along x, f along y with f growing upwards (like a chart).
struct MapRect {
    float x;
    float y;
    float w;
    float h;
};

inline void mapToParams(const MapRect& r, float px, float py, float& feed, float& kill) {
    const float u = r.w > 0.0F ? std::clamp((px - r.x) / r.w, 0.0F, 1.0F) : 0.0F;
    const float v = r.h > 0.0F ? std::clamp((py - r.y) / r.h, 0.0F, 1.0F) : 0.0F;
    kill = kKillMin + u * (kKillMax - kKillMin);
    feed = kFeedMin + (1.0F - v) * (kFeedMax - kFeedMin);
}

inline void paramsToMap(const MapRect& r, float feed, float kill, float& px, float& py) {
    const float u = std::clamp((kill - kKillMin) / (kKillMax - kKillMin), 0.0F, 1.0F);
    const float v = std::clamp((feed - kFeedMin) / (kFeedMax - kFeedMin), 0.0F, 1.0F);
    px = r.x + u * r.w;
    py = r.y + (1.0F - v) * r.h;
}

}  // namespace tp
