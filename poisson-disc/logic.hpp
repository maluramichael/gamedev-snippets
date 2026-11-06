// Poisson disc sampling: the logic of the demo, no SDL, so test.cpp can check it.
//
// Goal: scatter points (trees, rocks) so that no two are closer than r, and still look random. Bridson's algorithm
// does that in time linear in the number of points:
//
//   1. A background grid with cells of r / sqrt(2). The diagonal of such a cell is exactly r, so two points in one
//      cell would be closer than r: every cell holds at most one point, and "is anything within r?" becomes a look
//      at a handful of neighbouring cells instead of a walk over all points.
//   2. An active list with the points that may still have free room around them. Start with one random point.
//   3. Take a random active point and throw up to k candidates into the ring between r and 2r around it. The first
//      candidate that is far enough from everything becomes a new point (and active). If all k fail, the active
//      point is retired.
//   4. Stop when the active list is empty.
//
// Variable radius: a DensityMap scales r from place to place (0.5 to 1). Two points then keep at least the LARGER
// of their two local radii apart, which keeps the check symmetric. The grid cell shrinks to the smallest radius
// that occurs, and the neighbour search widens to the largest one.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

namespace pd {

struct Vec2 {
    double x, y;
};

inline double distance(Vec2 a, Vec2 b) { return std::hypot(a.x - b.x, a.y - b.y); }

/// splitmix64: tiny, fast, same numbers on every platform (SDL_rand would differ and is not seedable per object).
class Rng {
public:
    explicit Rng(std::uint64_t seed) : state_(seed) {}

    std::uint64_t next() {
        state_ += 0x9E3779B97F4A7C15ULL;
        std::uint64_t z = state_;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
        return z ^ (z >> 31);
    }

    /// In [0, 1): the top 53 bits fill a double mantissa exactly.
    double uniform() { return static_cast<double>(next() >> 11) * (1.0 / 9007199254740992.0); }

    /// In [0, n), n > 0.
    std::size_t below(std::size_t n) { return std::min(n - 1, static_cast<std::size_t>(uniform() * static_cast<double>(n))); }

private:
    std::uint64_t state_;
};

/// Radius scale per place, 1 = the full radius, kMinScale = half of it (four times the points). A coarse grid of cells.
class DensityMap {
public:
    static constexpr double kMinScale = 0.5;

    DensityMap(double width, double height, double cellSize = 8.0)
        : cell_(std::max(cellSize, 1.0)),
          cols_(std::max(1, static_cast<int>(std::ceil(width / cell_)))),
          rows_(std::max(1, static_cast<int>(std::ceil(height / cell_)))),
          scale_(static_cast<std::size_t>(cols_) * static_cast<std::size_t>(rows_), 1.0) {}

    /// Scale at a position; positions outside take the value of the nearest edge cell.
    double at(Vec2 p) const {
        const int cx = std::clamp(static_cast<int>(std::floor(p.x / cell_)), 0, cols_ - 1);
        const int cy = std::clamp(static_cast<int>(std::floor(p.y / cell_)), 0, rows_ - 1);
        return scale_[index(cx, cy)];
    }

    double minScale() const { return *std::min_element(scale_.begin(), scale_.end()); }

    bool isUniform() const { return std::all_of(scale_.begin(), scale_.end(), [](double s) { return s == 1.0; }); }

    /// Pull the cells around `c` towards kMinScale, strongest in the middle, nothing at the rim of the brush.
    void paint(Vec2 c, double brush) {
        forCellsInBrush(c, brush, [](double& s, double falloff) { s = std::min(s, 1.0 - (1.0 - kMinScale) * falloff); });
    }

    /// Push the cells around `c` back to 1, fully in the middle, not at all at the rim.
    void erase(Vec2 c, double brush) {
        forCellsInBrush(c, brush, [](double& s, double falloff) {
            s += (1.0 - s) * falloff;
            if (s > 0.995) s = 1.0;  // otherwise it only ever approaches 1
        });
    }

    void clear() { std::fill(scale_.begin(), scale_.end(), 1.0); }

private:
    std::size_t index(int cx, int cy) const { return static_cast<std::size_t>(cy) * static_cast<std::size_t>(cols_) + static_cast<std::size_t>(cx); }

    template <class F>
    void forCellsInBrush(Vec2 c, double brush, F apply) {
        if (!(brush > 0.0)) return;  // also catches NaN
        const int x0 = std::max(0, static_cast<int>(std::floor((c.x - brush) / cell_)));
        const int x1 = std::min(cols_ - 1, static_cast<int>(std::floor((c.x + brush) / cell_)));
        const int y0 = std::max(0, static_cast<int>(std::floor((c.y - brush) / cell_)));
        const int y1 = std::min(rows_ - 1, static_cast<int>(std::floor((c.y + brush) / cell_)));
        for (int cy = y0; cy <= y1; ++cy) {
            for (int cx = x0; cx <= x1; ++cx) {
                const double d = distance(c, {(cx + 0.5) * cell_, (cy + 0.5) * cell_});
                if (d < brush) apply(scale_[index(cx, cy)], 1.0 - d / brush);
            }
        }
    }

    double cell_;
    int cols_;
    int rows_;
    std::vector<double> scale_;
};

struct Params {
    double width, height;
    double radius;  // minimum distance r (before the density map scales it)
    int k;          // candidates per active point before it is retired
};

struct Trial {
    Vec2 pos;
    bool accepted;
};

class Sampler {
public:
    Sampler(Params params, const DensityMap& map, std::uint64_t seed)
        : p_{std::max(params.width, 1.0), std::max(params.height, 1.0), std::max(params.radius, 1.0), std::max(params.k, 1)},
          map_(map),
          rng_(seed) {
        // The densest place sets the cell size: its diagonal is the smallest distance that can occur between points.
        cell_ = p_.radius * map_.minScale() / std::sqrt(2.0);
        cols_ = std::max(1, static_cast<int>(std::ceil(p_.width / cell_)));
        rows_ = std::max(1, static_cast<int>(std::ceil(p_.height / cell_)));
        // The sparsest place (scale 1) sets how far a neighbour can be and still matter.
        reach_ = static_cast<int>(std::ceil(p_.radius / cell_));
        grid_.assign(static_cast<std::size_t>(cols_) * static_cast<std::size_t>(rows_), -1);
        add({rng_.uniform() * p_.width, rng_.uniform() * p_.height});
    }

    /// One round: a random active point gets up to k candidates, the first that fits becomes a point, otherwise the
    /// active point is retired. Returns false when nothing is active any more.
    bool step() {
        if (active_.empty()) return false;
        lastTrials_.clear();
        const std::size_t slot = rng_.below(active_.size());
        const std::size_t a = active_[slot];
        lastActive_ = a;
        const Vec2 c = points_[a];
        const double ra = radii_[a];
        constexpr double kTwoPi = 6.283185307179586;
        for (int t = 0; t < p_.k; ++t) {
            // Uniform over the AREA of the ring [ra, 2 ra]: the radius is sqrt(ra^2 + u * (4 ra^2 - ra^2)).
            const double angle = kTwoPi * rng_.uniform();
            const double dist = ra * std::sqrt(1.0 + 3.0 * rng_.uniform());
            const Vec2 cand{c.x + std::cos(angle) * dist, c.y + std::sin(angle) * dist};
            const bool ok = fits(cand);
            ++tested_;
            lastTrials_.push_back({cand, ok});
            if (ok) {
                add(cand);  // the active point stays, it may have more room
                return true;
            }
        }
        active_[slot] = active_.back();
        active_.pop_back();
        return true;
    }

    void run() {
        while (step()) {
        }
    }

    bool done() const { return active_.empty(); }

    const std::vector<Vec2>& points() const { return points_; }
    /// The local radius of each point (r times the map value where it sits).
    const std::vector<double>& radii() const { return radii_; }
    /// Indices into points() of the points that may still have room around them.
    const std::vector<std::size_t>& active() const { return active_; }
    /// The candidates of the last step, the accepted one (if any) is the last.
    const std::vector<Trial>& lastTrials() const { return lastTrials_; }
    /// The point (index into points()) that the last step took its candidates around.
    std::size_t lastActive() const { return lastActive_; }
    std::size_t candidatesTested() const { return tested_; }

    double cellSize() const { return cell_; }
    int cols() const { return cols_; }
    int rows() const { return rows_; }
    /// Row by row, the index of the point in each cell or -1.
    const std::vector<int>& grid() const { return grid_; }

private:
    int cellX(double x) const { return std::min(cols_ - 1, static_cast<int>(x / cell_)); }
    int cellY(double y) const { return std::min(rows_ - 1, static_cast<int>(y / cell_)); }

    void add(Vec2 p) {
        const int idx = static_cast<int>(points_.size());
        points_.push_back(p);
        radii_.push_back(p_.radius * map_.at(p));
        active_.push_back(static_cast<std::size_t>(idx));
        grid_[static_cast<std::size_t>(cellY(p.y)) * static_cast<std::size_t>(cols_) + static_cast<std::size_t>(cellX(p.x))] = idx;
    }

    /// Inside the domain and at least the larger of both local radii away from every point in the cells around it.
    bool fits(Vec2 c) const {
        if (!(c.x >= 0.0 && c.x < p_.width && c.y >= 0.0 && c.y < p_.height)) return false;
        const double rc = p_.radius * map_.at(c);
        const int cx = cellX(c.x);
        const int cy = cellY(c.y);
        for (int y = std::max(0, cy - reach_); y <= std::min(rows_ - 1, cy + reach_); ++y) {
            for (int x = std::max(0, cx - reach_); x <= std::min(cols_ - 1, cx + reach_); ++x) {
                const int q = grid_[static_cast<std::size_t>(y) * static_cast<std::size_t>(cols_) + static_cast<std::size_t>(x)];
                if (q < 0) continue;
                const double need = std::max(rc, radii_[static_cast<std::size_t>(q)]);
                const double dx = points_[static_cast<std::size_t>(q)].x - c.x;
                const double dy = points_[static_cast<std::size_t>(q)].y - c.y;
                if (dx * dx + dy * dy < need * need) return false;
            }
        }
        return true;
    }

    Params p_;
    DensityMap map_;
    Rng rng_;
    double cell_ = 1.0;
    int cols_ = 1;
    int rows_ = 1;
    int reach_ = 1;
    std::vector<int> grid_;
    std::vector<Vec2> points_;
    std::vector<double> radii_;
    std::vector<std::size_t> active_;
    std::vector<Trial> lastTrials_;
    std::size_t tested_ = 0;
    std::size_t lastActive_ = 0;
};

// ---- The two things Poisson disc is compared with ----

/// Plain uniform random: every point independent of the others.
inline std::vector<Vec2> randomPoints(std::size_t n, double w, double h, Rng& rng) {
    std::vector<Vec2> out;
    out.reserve(n);
    for (std::size_t i = 0; i < n; ++i) out.push_back({rng.uniform() * w, rng.uniform() * h});
    return out;
}

/// One random point in each cell of a regular grid with about n cells, then shuffled so a growing prefix covers the
/// whole area (the demo reveals the points in this order). Cell counts are rounded, so there can be a few more or
/// fewer than n points.
inline std::vector<Vec2> jitteredGrid(std::size_t n, double w, double h, Rng& rng) {
    std::vector<Vec2> out;
    if (n == 0) return out;
    const double side = std::sqrt(w * h / static_cast<double>(n));
    const int cols = std::max(1, static_cast<int>(std::lround(w / side)));
    const int rows = std::max(1, static_cast<int>(std::lround(h / side)));
    const double cw = w / cols;
    const double ch = h / rows;
    for (int cy = 0; cy < rows; ++cy) {
        for (int cx = 0; cx < cols; ++cx) out.push_back({(cx + rng.uniform()) * cw, (cy + rng.uniform()) * ch});
    }
    for (std::size_t i = out.size(); i > 1; --i) std::swap(out[i - 1], out[rng.below(i)]);
    return out;
}

// ---- Measuring ----

struct Pair {
    std::size_t a, b;
};

/// All pairs among the first n points that are closer than the larger of their two radii. Exactly r apart is fine.
inline std::vector<Pair> closePairs(const std::vector<Vec2>& pts, const std::vector<double>& radii, std::size_t n) {
    std::vector<Pair> out;
    n = std::min({n, pts.size(), radii.size()});
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 1; j < n; ++j) {
            const double need = std::max(radii[i], radii[j]);
            if (distance(pts[i], pts[j]) < need) out.push_back({i, j});
        }
    }
    return out;
}

/// The smallest distance among the first n points in units of the larger radius of the pair (1 = just far enough).
/// Infinity when there is no pair.
inline double minSeparation(const std::vector<Vec2>& pts, const std::vector<double>& radii, std::size_t n) {
    double best = std::numeric_limits<double>::infinity();
    n = std::min({n, pts.size(), radii.size()});
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 1; j < n; ++j) {
            const double need = std::max(radii[i], radii[j]);
            if (need > 0.0) best = std::min(best, distance(pts[i], pts[j]) / need);
        }
    }
    return best;
}

// ---- Animation ----

/// How many of `total` points to show while the live sampler has `live` of the `ref` points it will end with.
/// Rounds up, so the first point shows right away. Without a reference everything shows.
inline std::size_t revealCount(std::size_t live, std::size_t total, std::size_t ref) {
    if (ref == 0 || live >= ref) return total;
    return (total * live + ref - 1) / ref;
}

/// Whole steps due after `dt` seconds at `rate` steps per second. The fraction of a step stays in `acc`. A long pause
/// counts as 0.25 s, more than maxSteps drops the rest instead of building a backlog, and rate <= 0 is paused.
inline int stepsDue(double& acc, double dt, double rate, int maxSteps) {
    if (!(rate > 0.0) || maxSteps <= 0) {
        acc = 0.0;
        return 0;
    }
    acc += std::clamp(dt, 0.0, 0.25);
    const int n = static_cast<int>(std::floor(acc * rate + 1e-9));
    if (n > maxSteps) {
        acc = 0.0;
        return maxSteps;
    }
    acc = std::max(0.0, acc - static_cast<double>(n) / rate);
    return n;
}

}  // namespace pd
