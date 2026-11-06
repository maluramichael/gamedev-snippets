// Caves from cellular automata: the logic of the demo, no SDL, so test.cpp can check it.
//
// 1. Fill the map with random walls (a cell is a wall if its noise value is below the density).
// 2. Repeat the 4-5 rule: a wall stays a wall if at least 4 of its 8 neighbors are walls, a floor cell becomes a wall if at
//    least 5 are. Each pass makes every cell look more like its neighbors, so the noise smooths out into blobs.
// 3. Cells outside the map count as walls. That closes the cave at the border and makes the corners fill up.
//
// TWO THINGS THAT GO WRONG:
//   * Updating the map in place. The rule must read the OLD map for every cell. If a pass writes into the map it is
//     reading, cells further down and to the right see a half-updated neighborhood and the result depends on the scan order.
//     stepBuffered is the right one, stepInPlace exists to show the difference.
//   * Believing every cave is connected. The rule only makes cells agree with their neighbors, it knows nothing about
//     paths. Regions are found with a flood fill over 4-connected floor cells (a diagonal gap between two walls is not a
//     door), and the ones that are not the largest are sealed off.
#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace ca {

struct Grid {
    int w = 0;
    int h = 0;
    std::vector<std::uint8_t> cells;  // 1 = wall, 0 = floor, row by row

    Grid(int width, int height) : w(width), h(height), cells(static_cast<std::size_t>(width * height), 0) {}

    bool inside(int x, int y) const { return x >= 0 && y >= 0 && x < w && y < h; }
    std::size_t index(int x, int y) const { return static_cast<std::size_t>(y * w + x); }
    /// Everything outside the map is wall.
    bool isWall(int x, int y) const { return !inside(x, y) || cells[index(x, y)] != 0; }
    void set(int x, int y, bool wall) { cells[index(x, y)] = wall ? 1 : 0; }
    int wallCount() const { return static_cast<int>(std::count(cells.begin(), cells.end(), std::uint8_t{1})); }
};

/// splitmix64: tiny, and the same numbers on every platform (std::uniform_real_distribution is not).
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

    /// Uniform in [0, 1): the top 53 bits fill a double exactly.
    double unit() { return static_cast<double>(next() >> 11) * (1.0 / 9007199254740992.0); }

private:
    std::uint64_t state_;
};

/// One random number per cell. Kept apart from the density, so a slider can move the threshold over the same noise.
inline std::vector<double> makeNoise(int w, int h, std::uint64_t seed) {
    Rng rng(seed);
    std::vector<double> noise(static_cast<std::size_t>(w * h));
    for (double& v : noise) v = rng.unit();
    return noise;
}

inline Grid fromNoise(int w, int h, const std::vector<double>& noise, double density) {
    Grid g(w, h);
    for (std::size_t i = 0; i < g.cells.size(); ++i) g.cells[i] = noise[i] < density ? 1 : 0;
    return g;
}

inline int wallNeighbors(const Grid& g, int x, int y) {
    int n = 0;
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            if ((dx != 0 || dy != 0) && g.isWall(x + dx, y + dy)) ++n;
        }
    }
    return n;
}

/// The 4-5 rule.
inline bool nextIsWall(bool wasWall, int walls) { return wasWall ? walls >= 4 : walls >= 5; }

/// One pass that reads the old map and writes a new one. Returns how many cells changed.
inline int stepBuffered(Grid& g) {
    Grid next = g;
    int changed = 0;
    for (int y = 0; y < g.h; ++y) {
        for (int x = 0; x < g.w; ++x) {
            const bool now = nextIsWall(g.isWall(x, y), wallNeighbors(g, x, y));
            if (now != g.isWall(x, y)) ++changed;
            next.set(x, y, now);
        }
    }
    g = std::move(next);
    return changed;
}

/// The mistake: write while reading. Later cells see the new values of the ones before them.
inline int stepInPlace(Grid& g) {
    int changed = 0;
    for (int y = 0; y < g.h; ++y) {
        for (int x = 0; x < g.w; ++x) {
            const bool now = nextIsWall(g.isWall(x, y), wallNeighbors(g, x, y));
            if (now != g.isWall(x, y)) ++changed;
            g.set(x, y, now);
        }
    }
    return changed;
}

struct Regions {
    std::vector<int> label;  // per cell: region number in scan order, -1 for walls
    std::vector<int> size;   // cells per region
    int largest = -1;        // region with the most cells, the first one on a tie, -1 if there is no floor
};

/// Connected floor areas, 4-connected (up, down, left, right). Iterative, so a big map cannot blow the stack.
inline Regions findRegions(const Grid& g) {
    Regions r;
    r.label.assign(g.cells.size(), -1);
    std::vector<std::size_t> queue;
    for (int y = 0; y < g.h; ++y) {
        for (int x = 0; x < g.w; ++x) {
            if (g.isWall(x, y) || r.label[g.index(x, y)] != -1) continue;
            const int id = static_cast<int>(r.size.size());
            int count = 0;
            queue.assign(1, g.index(x, y));
            r.label[queue[0]] = id;
            for (std::size_t head = 0; head < queue.size(); ++head) {
                const int cx = static_cast<int>(queue[head] % static_cast<std::size_t>(g.w));
                const int cy = static_cast<int>(queue[head] / static_cast<std::size_t>(g.w));
                ++count;
                constexpr int kDx[4] = {1, -1, 0, 0};
                constexpr int kDy[4] = {0, 0, 1, -1};
                for (int k = 0; k < 4; ++k) {
                    const int nx = cx + kDx[k];
                    const int ny = cy + kDy[k];
                    if (g.isWall(nx, ny) || r.label[g.index(nx, ny)] != -1) continue;
                    r.label[g.index(nx, ny)] = id;
                    queue.push_back(g.index(nx, ny));
                }
            }
            r.size.push_back(count);
        }
    }
    for (std::size_t i = 0; i < r.size.size(); ++i) {
        if (r.largest < 0 || r.size[i] > r.size[static_cast<std::size_t>(r.largest)]) r.largest = static_cast<int>(i);
    }
    return r;
}

/// Steps from (sx, sy) to every cell it can reach over floor, -1 for walls and cells it cannot reach. The flood fill of the
/// demo reveals cells in this order. A start on a wall or outside the map reaches nothing.
inline std::vector<int> floodDistances(const Grid& g, int sx, int sy) {
    std::vector<int> dist(g.cells.size(), -1);
    if (g.isWall(sx, sy)) return dist;
    std::vector<std::size_t> queue{g.index(sx, sy)};
    dist[queue[0]] = 0;
    for (std::size_t head = 0; head < queue.size(); ++head) {
        const int cx = static_cast<int>(queue[head] % static_cast<std::size_t>(g.w));
        const int cy = static_cast<int>(queue[head] / static_cast<std::size_t>(g.w));
        const int d = dist[queue[head]];
        constexpr int kDx[4] = {1, -1, 0, 0};
        constexpr int kDy[4] = {0, 0, 1, -1};
        for (int k = 0; k < 4; ++k) {
            const int nx = cx + kDx[k];
            const int ny = cy + kDy[k];
            if (g.isWall(nx, ny) || dist[g.index(nx, ny)] != -1) continue;
            dist[g.index(nx, ny)] = d + 1;
            queue.push_back(g.index(nx, ny));
        }
    }
    return dist;
}

/// Turns every region except the largest into wall. Returns how many cells that took.
inline int keepLargestRegion(Grid& g) {
    const Regions r = findRegions(g);
    int filled = 0;
    for (std::size_t i = 0; i < g.cells.size(); ++i) {
        if (r.label[i] >= 0 && r.label[i] != r.largest) {
            g.cells[i] = 1;
            ++filled;
        }
    }
    return filled;
}

struct Stats {
    int walls = 0;
    int floor = 0;
    int regions = 0;
    int largest = 0;  // cells in the biggest region
};

inline Stats measure(const Grid& g) {
    Stats s;
    s.walls = g.wallCount();
    s.floor = g.w * g.h - s.walls;
    const Regions r = findRegions(g);
    s.regions = static_cast<int>(r.size.size());
    s.largest = r.largest >= 0 ? r.size[static_cast<std::size_t>(r.largest)] : 0;
    return s;
}

enum class Update { Buffered, InPlace };

struct Result {
    Grid grid;
    bool stable = false;   // one more pass would change nothing, and so would every pass after it
    int changedPasses = 0;  // passes that changed at least one cell
};

/// The whole pipeline from noise to map: fill, `passes` times the rule, optionally drop the sealed-off caves.
/// Stops early once a pass changes nothing, and tells whether the map has settled.
inline Result generate(int w, int h, const std::vector<double>& noise, double density, int passes, Update mode, bool keepLargest) {
    Result out{fromNoise(w, h, noise, density), false, 0};
    for (int i = 0; i < passes; ++i) {
        const int changed = mode == Update::Buffered ? stepBuffered(out.grid) : stepInPlace(out.grid);
        if (changed == 0) {
            out.stable = true;
            break;
        }
        ++out.changedPasses;
    }
    if (!out.stable) {
        // Not stable yet, or exactly on the last changing pass: look one pass ahead on a copy.
        Grid probe = out.grid;
        out.stable = (mode == Update::Buffered ? stepBuffered(probe) : stepInPlace(probe)) == 0;
    }
    if (keepLargest) keepLargestRegion(out.grid);
    return out;
}

}  // namespace ca
