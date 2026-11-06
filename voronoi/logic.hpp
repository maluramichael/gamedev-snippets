// The math of the Voronoi shader, mirrored on the CPU so it can be tested without a GPU.
//
// A Voronoi cell is the set of points that are closer to one site than to any other. Per pixel the shader finds the
// nearest site by brute force (loop over all sites, keep the smallest distance, strict `<` so the smaller index wins a
// tie), remembers the second nearest, and measures the exact distance to the cell border. The Worley part replaces the
// site list by a hash: every lattice cell owns one feature point, and a pixel only has to look at the 3x3 cells around it.
// Everything here uses the same constants and the same integer hash as the GLSL in main.cpp.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace gd::voronoi {

inline constexpr float kInf = std::numeric_limits<float>::infinity();
inline constexpr float kTwoPi = 6.28318530718F;

struct Vec2 {
    float x = 0.0F;
    float y = 0.0F;
};

enum class Metric { Euclid = 0, Manhattan = 1, Chebyshev = 2 };

inline float distance(Metric metric, Vec2 a, Vec2 b) {
    const float dx = std::fabs(a.x - b.x);
    const float dy = std::fabs(a.y - b.y);
    switch (metric) {
        case Metric::Manhattan: return dx + dy;
        case Metric::Chebyshev: return std::max(dx, dy);
        case Metric::Euclid: break;
    }
    return std::sqrt(dx * dx + dy * dy);
}

/// Nearest (first) and second nearest site with their distances. `first` is -1 without sites, `second` is -1 with only one.
struct Nearest {
    int first = -1;
    int second = -1;
    float d1 = kInf;
    float d2 = kInf;
};

/// Brute force, the loop the fragment shader runs per pixel. Strict `<`: on equal distance the smaller index keeps the place.
inline Nearest nearest(const std::vector<Vec2>& sites, Vec2 p, Metric metric) {
    Nearest n;
    for (int i = 0; i < static_cast<int>(sites.size()); ++i) {
        const float d = distance(metric, sites[static_cast<std::size_t>(i)], p);
        if (d < n.d1) {
            n.second = n.first;
            n.d2 = n.d1;
            n.first = i;
            n.d1 = d;
        } else if (d < n.d2) {
            n.second = i;
            n.d2 = d;
        }
    }
    return n;
}

/// Exact Euclidean distance from `p` (inside the cell of site `first`) to the nearest cell border: the smallest distance
/// to the perpendicular bisector of `first` and any other site. The cheap (d2 - d1) / 2 is only a lower bound of this.
inline float edgeDistance(const std::vector<Vec2>& sites, int first, Vec2 p) {
    const Vec2 a = sites[static_cast<std::size_t>(first)];
    float best = kInf;
    for (int j = 0; j < static_cast<int>(sites.size()); ++j) {
        if (j == first) continue;
        const Vec2 b = sites[static_cast<std::size_t>(j)];
        const float dx = b.x - a.x;
        const float dy = b.y - a.y;
        const float length = std::sqrt(dx * dx + dy * dy);
        if (length <= 0.0F) continue;  // two sites on the same spot have no border
        const float along = ((0.5F * (a.x + b.x) - p.x) * dx + (0.5F * (a.y + b.y) - p.y) * dy) / length;
        best = std::min(best, along);
    }
    return best;
}

/// A drifting site. Fixed steps, so the same input always gives the same picture.
struct Body {
    Vec2 p;
    Vec2 v;
};

namespace detail {
/// Folds `x` into [0, size] like a ball bouncing between two walls; `flipped` says if the direction of travel is reversed.
inline float bounce(float x, float size, bool& flipped) {
    const float period = 2.0F * size;
    flipped = (static_cast<long long>(std::floor(x / size)) % 2) != 0;
    float m = std::fmod(x, period);
    if (m < 0.0F) m += period;
    return m > size ? period - m : m;
}
}  // namespace detail

/// Moves the body by `dt` and reflects it at the walls of [0, w] x [0, h].
inline void drift(Body& b, float dt, float w, float h) {
    bool fx = false, fy = false;
    b.p.x = detail::bounce(b.p.x + b.v.x * dt, w, fx);
    b.p.y = detail::bounce(b.p.y + b.v.y * dt, h, fy);
    if (fx) b.v.x = -b.v.x;
    if (fy) b.v.y = -b.v.y;
}

/// One step of Lloyd relaxation on a coarse grid. The domain is [0, gridW] x [0, gridH] with one unit per grid cell
/// (cell centers at i + 0.5). Every grid cell is given to its nearest site, each site moves to the mean of its cells.
/// A site that owns no cell stays where it is.
inline std::vector<Vec2> lloydStep(const std::vector<Vec2>& sites, int gridW, int gridH, Metric metric = Metric::Euclid) {
    struct Sum {
        double x = 0.0, y = 0.0;
        long count = 0;
    };
    std::vector<Sum> sums(sites.size());
    for (int j = 0; j < gridH; ++j) {
        for (int i = 0; i < gridW; ++i) {
            const Vec2 center{static_cast<float>(i) + 0.5F, static_cast<float>(j) + 0.5F};
            const int k = nearest(sites, center, metric).first;
            if (k < 0) continue;
            Sum& s = sums[static_cast<std::size_t>(k)];
            s.x += center.x;
            s.y += center.y;
            ++s.count;
        }
    }
    std::vector<Vec2> out = sites;
    for (std::size_t k = 0; k < sites.size(); ++k) {
        if (sums[k].count > 0) out[k] = Vec2{static_cast<float>(sums[k].x / static_cast<double>(sums[k].count)), static_cast<float>(sums[k].y / static_cast<double>(sums[k].count))};
    }
    return out;
}

/// Sum over all sites of the distance to the centroid of their cell, the number Lloyd relaxation drives to 0.
inline double centroidError(const std::vector<Vec2>& sites, int gridW, int gridH, Metric metric = Metric::Euclid) {
    const std::vector<Vec2> centroids = lloydStep(sites, gridW, gridH, metric);
    double sum = 0.0;
    for (std::size_t k = 0; k < sites.size(); ++k) sum += distance(Metric::Euclid, sites[k], centroids[k]);
    return sum;
}

// ---- Hash and cell noise (Worley) ------------------------------------------------------------------------------------

/// PCG output function as an integer hash; the GLSL in main.cpp is the same line by line (uint arithmetic wraps in both).
inline std::uint32_t pcg(std::uint32_t v) {
    const std::uint32_t s = v * 747796405U + 2891336453U;
    const std::uint32_t w = ((s >> ((s >> 28U) + 4U)) ^ s) * 277803737U;
    return (w >> 22U) ^ w;
}

/// Hash of a lattice cell (negative coordinates wrap like the GLSL `uint(int)` conversion).
inline std::uint32_t hashCell(int x, int y, std::uint32_t seed) {
    return pcg(pcg(static_cast<std::uint32_t>(x) + pcg(seed)) + static_cast<std::uint32_t>(y));
}

/// The top 24 bits as a float in [0, 1). 24 bits fit a float mantissa exactly, so the CPU and the GPU agree.
inline double unit(std::uint32_t h) { return static_cast<double>(h >> 8U) / 16777216.0; }

/// Position of the feature point inside its cell, both axes in [0, 1]. Resting place 0.15 .. 0.85 plus a swing of
/// +-0.15 with a phase of its own, so the points wander but never leave their cell (which keeps the 3x3 search valid).
inline Vec2 featureOffset(std::uint32_t h, float t) {
    const std::uint32_t h2 = pcg(h);
    const std::uint32_t h3 = pcg(h2);
    const std::uint32_t h4 = pcg(h3);
    const auto u = [](std::uint32_t v) { return static_cast<float>(unit(v)); };
    return Vec2{0.15F + 0.7F * u(h) + 0.15F * std::sin(t + kTwoPi * u(h3)), 0.15F + 0.7F * u(h2) + 0.15F * std::sin(t * 1.3F + kTwoPi * u(h4))};
}

/// F1 and F2: distance to the nearest and second nearest feature point, plus the lattice cell that owns the nearest one.
struct Worley {
    float f1 = kInf;
    float f2 = kInf;
    int cellX = 0;
    int cellY = 0;
};

/// Cell noise at `p`. `jitter` scales the offset inside the cell (0 = every feature point sits on its lattice corner),
/// `radius` is the search window: 1 means 3x3 cells (what the shader does), 2 means 5x5 (reference for the tests).
inline Worley worley(Vec2 p, float t, Metric metric, float jitter = 1.0F, int radius = 1, std::uint32_t seed = 0U) {
    const int cx = static_cast<int>(std::floor(p.x));
    const int cy = static_cast<int>(std::floor(p.y));
    Worley w;
    for (int j = -radius; j <= radius; ++j) {
        for (int i = -radius; i <= radius; ++i) {
            const int x = cx + i;
            const int y = cy + j;
            const Vec2 o = featureOffset(hashCell(x, y, seed), t);
            const Vec2 feature{static_cast<float>(x) + o.x * jitter, static_cast<float>(y) + o.y * jitter};
            const float d = distance(metric, feature, p);
            if (d < w.f1) {
                w.f2 = w.f1;
                w.f1 = d;
                w.cellX = x;
                w.cellY = y;
            } else if (d < w.f2) {
                w.f2 = d;
            }
        }
    }
    return w;
}

/// Exact Euclidean distance from `p` to the border of its Worley cell: the same bisector formula as edgeDistance, but
/// against the feature points of the 5x5 cells around the owner (a border neighbour can sit two lattice cells away).
inline float worleyEdge(Vec2 p, float t, float jitter = 1.0F, std::uint32_t seed = 0U) {
    const Worley w = worley(p, t, Metric::Euclid, jitter, 1, seed);
    const auto featureAt = [&](int x, int y) {
        const Vec2 o = featureOffset(hashCell(x, y, seed), t);
        return Vec2{static_cast<float>(x) + o.x * jitter, static_cast<float>(y) + o.y * jitter};
    };
    const Vec2 a = featureAt(w.cellX, w.cellY);
    float best = kInf;
    for (int j = -2; j <= 2; ++j) {
        for (int i = -2; i <= 2; ++i) {
            if (i == 0 && j == 0) continue;
            const Vec2 b = featureAt(w.cellX + i, w.cellY + j);
            const float dx = b.x - a.x;
            const float dy = b.y - a.y;
            const float length = std::sqrt(dx * dx + dy * dy);
            if (length <= 0.0F) continue;
            best = std::min(best, ((0.5F * (a.x + b.x) - p.x) * dx + (0.5F * (a.y + b.y) - p.y) * dy) / length);
        }
    }
    return best;
}

}  // namespace gd::voronoi
