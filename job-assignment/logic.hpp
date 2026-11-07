// The testable part of the job-assignment demo, no SDL: cost matrix, greedy assignment, the Hungarian method, a
// brute-force reference, crossing detection and the layouts the demo starts from.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numeric>
#include <optional>
#include <vector>

namespace ja {

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};

inline double distance(Vec2 a, Vec2 b) { return std::hypot(a.x - b.x, a.y - b.y); }

/// costs[w][j] = what it costs worker w to do job j.
using Costs = std::vector<std::vector<double>>;
/// assignment[w] = the job worker w does. A valid assignment uses every job exactly once.
using Assignment = std::vector<int>;

/// The brute-force reference tries every permutation, n! of them, so it stops here (8! = 40320).
inline constexpr int kBruteForceMax = 8;

inline Costs costMatrix(const std::vector<Vec2>& workers, const std::vector<Vec2>& jobs) {
    Costs c(workers.size(), std::vector<double>(jobs.size()));
    for (std::size_t w = 0; w < workers.size(); ++w) {
        for (std::size_t j = 0; j < jobs.size(); ++j) c[w][j] = distance(workers[w], jobs[j]);
    }
    return c;
}

/// Square (as many jobs as workers) and every entry finite. NaN would make the Hungarian loop wait for a minimum that never comes.
inline bool isUsable(const Costs& costs) {
    for (const auto& row : costs) {
        if (row.size() != costs.size()) return false;
        for (double v : row) {
            if (!std::isfinite(v)) return false;
        }
    }
    return true;
}

/// Sum of the chosen entries; infinity for an assignment that does not fit the matrix.
inline double totalCost(const Costs& costs, const Assignment& assignment) {
    if (assignment.size() != costs.size()) return std::numeric_limits<double>::infinity();
    double sum = 0.0;
    for (std::size_t w = 0; w < assignment.size(); ++w) {
        const int j = assignment[w];
        if (j < 0 || static_cast<std::size_t>(j) >= costs.size()) return std::numeric_limits<double>::infinity();
        sum += costs[w][static_cast<std::size_t>(j)];
    }
    return sum;
}

/// 0, 1, ..., n-1 or the same backwards: the order in which the greedy method lets the workers pick.
inline std::vector<int> processingOrder(int n, bool reversed) {
    std::vector<int> order(static_cast<std::size_t>(std::max(n, 0)));
    std::iota(order.begin(), order.end(), 0);
    if (reversed) std::reverse(order.begin(), order.end());
    return order;
}

inline bool isPermutation(const std::vector<int>& order, std::size_t n) {
    if (order.size() != n) return false;
    std::vector<bool> seen(n, false);
    for (int v : order) {
        if (v < 0 || static_cast<std::size_t>(v) >= n || seen[static_cast<std::size_t>(v)]) return false;
        seen[static_cast<std::size_t>(v)] = true;
    }
    return true;
}

/// The cheapest job in `row` that is not taken yet (lowest index on a tie), n if every job is taken.
inline std::size_t cheapestFree(const std::vector<double>& row, const std::vector<char>& taken) {
    std::size_t pick = row.size();
    for (std::size_t j = 0; j < row.size(); ++j) {
        if (taken[j] == 0 && (pick == row.size() || row[j] < row[pick])) pick = j;
    }
    return pick;
}

/// Greedy: in the given order every worker takes the cheapest job that is still free (lowest index on a tie).
/// Empty for a matrix that is not square or an order that is not a permutation of the workers.
inline Assignment greedy(const Costs& costs, const std::vector<int>& order) {
    const std::size_t n = costs.size();
    if (n == 0 || !isUsable(costs) || !isPermutation(order, n)) return {};
    Assignment out(n, -1);
    std::vector<char> taken(n, 0);
    for (int w : order) {
        const std::size_t pick = cheapestFree(costs[static_cast<std::size_t>(w)], taken);
        taken[pick] = 1;
        out[static_cast<std::size_t>(w)] = static_cast<int>(pick);
    }
    return out;
}

/// The Hungarian method in its O(n^3) form with row and column potentials (u, v): each worker is added by growing an
/// alternating tree along the cheapest reduced costs until it reaches a free job, then the path is flipped.
/// Empty for a matrix that is not square or holds NaN or infinity.
inline Assignment hungarian(const Costs& costs) {
    const std::size_t n = costs.size();
    if (n == 0 || !isUsable(costs)) return {};
    constexpr double kInf = std::numeric_limits<double>::infinity();
    // Index 0 is a dummy job, rows and columns run from 1.
    std::vector<double> u(n + 1, 0.0);
    std::vector<double> v(n + 1, 0.0);
    std::vector<std::size_t> jobWorker(n + 1, 0);  // worker (1-based) that currently holds job j, 0 = free
    std::vector<std::size_t> way(n + 1, 0);        // the job before j on the alternating path
    for (std::size_t w = 1; w <= n; ++w) {
        jobWorker[0] = w;
        std::size_t j0 = 0;
        std::vector<double> minv(n + 1, kInf);
        std::vector<bool> used(n + 1, false);
        do {
            used[j0] = true;
            const std::size_t w0 = jobWorker[j0];
            double delta = kInf;
            std::size_t j1 = 0;
            for (std::size_t j = 1; j <= n; ++j) {
                if (used[j]) continue;
                const double reduced = costs[w0 - 1][j - 1] - u[w0] - v[j];
                if (reduced < minv[j]) {
                    minv[j] = reduced;
                    way[j] = j0;
                }
                if (minv[j] < delta) {
                    delta = minv[j];
                    j1 = j;
                }
            }
            for (std::size_t j = 0; j <= n; ++j) {
                if (used[j]) {
                    u[jobWorker[j]] += delta;
                    v[j] -= delta;
                } else {
                    minv[j] -= delta;
                }
            }
            j0 = j1;
        } while (jobWorker[j0] != 0);
        do {
            const std::size_t j1 = way[j0];
            jobWorker[j0] = jobWorker[j1];
            j0 = j1;
        } while (j0 != 0);
    }
    Assignment out(n, -1);
    for (std::size_t j = 1; j <= n; ++j) out[jobWorker[j] - 1] = static_cast<int>(j - 1);
    return out;
}

/// The reference: try every permutation. Empty above kBruteForceMax or for an unusable matrix.
inline Assignment bruteForce(const Costs& costs) {
    const std::size_t n = costs.size();
    if (n == 0 || n > static_cast<std::size_t>(kBruteForceMax) || !isUsable(costs)) return {};
    Assignment current(n);
    std::iota(current.begin(), current.end(), 0);
    Assignment best = current;
    double bestTotal = totalCost(costs, current);
    while (std::next_permutation(current.begin(), current.end())) {
        const double t = totalCost(costs, current);
        if (t < bestTotal) {
            bestTotal = t;
            best = current;
        }
    }
    return best;
}

struct OrderRange {
    double best = 0.0;
    double worst = 0.0;
    long orders = 0;
};

/// Total length of the greedy result over every possible order of the workers.
inline OrderRange greedyOrderRange(const Costs& costs) {
    const std::size_t n = costs.size();
    if (n == 0 || n > static_cast<std::size_t>(kBruteForceMax) || !isUsable(costs)) return {};
    std::vector<int> order = processingOrder(static_cast<int>(n), false);
    std::vector<char> taken(n);
    OrderRange r;
    r.best = std::numeric_limits<double>::infinity();
    r.worst = -std::numeric_limits<double>::infinity();
    do {
        // The same walk as greedy(), summing instead of building the assignment: 40320 orders must stay cheap in the browser.
        std::fill(taken.begin(), taken.end(), 0);
        double t = 0.0;
        for (int w : order) {
            const auto& row = costs[static_cast<std::size_t>(w)];
            const std::size_t pick = cheapestFree(row, taken);
            taken[pick] = 1;
            t += row[pick];
        }
        r.best = std::min(r.best, t);
        r.worst = std::max(r.worst, t);
        ++r.orders;
    } while (std::next_permutation(order.begin(), order.end()));
    return r;
}

// ---- crossings ----

inline double orientation(Vec2 a, Vec2 b, Vec2 c) { return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x); }

/// Proper crossing: each segment has the other's end points strictly on opposite sides. Touching and overlapping on one
/// line do not count, the demo only has to catch the X that the optimum never contains.
inline bool segmentsCross(Vec2 a, Vec2 b, Vec2 c, Vec2 d) {
    const double d1 = orientation(c, d, a);
    const double d2 = orientation(c, d, b);
    const double d3 = orientation(a, b, c);
    const double d4 = orientation(a, b, d);
    return ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0)) && ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0));
}

/// Where the worker-to-job segments of an assignment cross, one point per crossing pair. Empty for an assignment that
/// does not fit.
inline std::vector<Vec2> crossings(const std::vector<Vec2>& workers, const std::vector<Vec2>& jobs, const Assignment& assignment) {
    std::vector<Vec2> out;
    if (assignment.size() != workers.size()) return out;
    for (int j : assignment) {
        if (j < 0 || static_cast<std::size_t>(j) >= jobs.size()) return out;
    }
    for (std::size_t i = 0; i < workers.size(); ++i) {
        for (std::size_t k = i + 1; k < workers.size(); ++k) {
            const Vec2 a = workers[i];
            const Vec2 b = jobs[static_cast<std::size_t>(assignment[i])];
            const Vec2 c = workers[k];
            const Vec2 d = jobs[static_cast<std::size_t>(assignment[k])];
            if (!segmentsCross(a, b, c, d)) continue;
            // The signs of d3 and d4 differ here, so d3 - d4 is not zero.
            const double d3 = orientation(a, b, c);
            const double d4 = orientation(a, b, d);
            const double t = d3 / (d3 - d4);
            out.push_back({c.x + (d.x - c.x) * t, c.y + (d.y - c.y) * t});
        }
    }
    return out;
}

// ---- layouts ----

struct Layout {
    std::vector<Vec2> workers;
    std::vector<Vec2> jobs;
};

struct Field {
    double left, top, right, bottom;
};

inline Vec2 clampToField(Vec2 p, const Field& f) { return {std::clamp(p.x, f.left, f.right), std::clamp(p.y, f.top, f.bottom)}; }

/// A small linear congruential generator, so a seed always gives the same layout on every platform.
class Lcg {
public:
    explicit Lcg(std::uint32_t seed) : state_(seed * 2654435761U + 12345U) {}
    std::uint32_t next() {
        state_ = state_ * 1664525U + 1013904223U;
        return state_ >> 8;  // the low bits of an LCG are the weak ones
    }
    double unit() { return static_cast<double>(next()) / 16777216.0; }
    std::uint32_t below(std::uint32_t n) { return next() % n; }

private:
    std::uint32_t state_;
};

/// A point in the field at least `gap` away from every point in `placed`. After 200 tries it takes the last candidate
/// anyway, so a crowded field cannot hang the demo.
inline Vec2 freeSpot(Lcg& rng, const Field& f, const std::vector<Vec2>& placed, double gap) {
    Vec2 p{};
    for (int attempt = 0; attempt < 200; ++attempt) {
        p = {f.left + rng.unit() * (f.right - f.left), f.top + rng.unit() * (f.bottom - f.top)};
        const bool free = std::all_of(placed.begin(), placed.end(), [&](Vec2 q) { return distance(p, q) >= gap; });
        if (free) break;
    }
    return p;
}

inline Layout randomLayout(std::uint32_t seed, int n, const Field& f, double gap) {
    Layout l;
    Lcg rng(seed);
    std::vector<Vec2> placed;
    for (int i = 0; i < n; ++i) {
        l.workers.push_back(freeSpot(rng, f, placed, gap));
        placed.push_back(l.workers.back());
        l.jobs.push_back(freeSpot(rng, f, placed, gap));
        placed.push_back(l.jobs.back());
    }
    return l;
}

/// The same scene with n workers and n jobs: the first n of each stay where they are, new ones land on free spots.
inline Layout resized(const Layout& old, int n, std::uint32_t seed, const Field& f, double gap) {
    const auto count = static_cast<std::size_t>(std::max(n, 0));
    Layout l;
    l.workers.assign(old.workers.begin(), old.workers.begin() + static_cast<std::ptrdiff_t>(std::min(count, old.workers.size())));
    l.jobs.assign(old.jobs.begin(), old.jobs.begin() + static_cast<std::ptrdiff_t>(std::min(count, old.jobs.size())));
    Lcg rng(seed);
    while (l.workers.size() < count || l.jobs.size() < count) {
        std::vector<Vec2> placed = l.workers;
        placed.insert(placed.end(), l.jobs.begin(), l.jobs.end());
        auto& target = l.workers.size() <= l.jobs.size() ? l.workers : l.jobs;
        target.push_back(freeSpot(rng, f, placed, gap));
    }
    return l;
}

struct Pick {
    bool worker;
    std::size_t index;
};

/// The worker or job closest to `p` within `radius` (the radius itself still counts).
inline std::optional<Pick> pickEntity(const Layout& l, Vec2 p, double radius) {
    std::optional<Pick> best;
    double bestDist = radius;
    // Strictly closer replaces, so a dead heat goes to the first candidate, workers before jobs.
    for (std::size_t i = 0; i < l.workers.size(); ++i) {
        const double d = distance(l.workers[i], p);
        if (d <= radius && (!best || d < bestDist)) {
            bestDist = d;
            best = Pick{true, i};
        }
    }
    for (std::size_t i = 0; i < l.jobs.size(); ++i) {
        const double d = distance(l.jobs[i], p);
        if (d <= radius && (!best || d < bestDist)) {
            bestDist = d;
            best = Pick{false, i};
        }
    }
    return best;
}

inline constexpr int kInitialCount = 6;
inline constexpr std::uint32_t kInitialSeed = 154;
inline constexpr Field kPlayField{24.0, 48.0, 936.0, 358.0};
inline constexpr double kMinGap = 34.0;

/// The scene the demo opens with: the greedy result already crosses itself and is clearly longer than the optimum.
inline Layout initialLayout() { return randomLayout(kInitialSeed, kInitialCount, kPlayField, kMinGap); }

}  // namespace ja
