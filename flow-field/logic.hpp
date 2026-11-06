// Flow fields: the logic of the demo, no SDL, so test.cpp can check it.
//
// The idea in three steps:
//   1. One breadth first search starts at the GOAL and walks over the grid. Every cell gets the number of steps it takes
//      to get from there to the goal (the distance map, drawn as the heat map).
//   2. Every cell gets an arrow that points downhill on that map: the negative gradient of the distance.
//   3. An agent does not search anything. It looks up the arrow of the cell it stands in and walks that way.
//
// The search runs once per change of goal or walls, no matter how many agents there are. The alternative is A* per
// agent: the same number of searches as agents, each one starting at the agent. astar() below is that alternative, with
// expansion counters on both sides so the demo can show the difference.
//
// THE GRADIENT: for a cell with distance c the four neighbour values are read, a wall or an unreached neighbour counts
// as c + 1 ("uphill"). Then
//
//     dir = ( d(left) - d(right),  d(up) - d(down) )   normalized
//
// Neighbours differ by at most 1, so a wall (c + 1) is never lower than the opposite neighbour and no component of the
// arrow ever points into a wall. A cell in the middle of two equally long routes has d(left) == d(right) and the
// gradient cancels to zero; there the arrow falls back to the first lower neighbour (order: left, right, up, down).
//
// Coordinates: cell (x, y) covers [x, x+1) x [y, y+1) in "cell units", y grows downwards.
#pragma once

#include <algorithm>
#include <array>
#include <climits>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <queue>
#include <string>
#include <string_view>
#include <vector>

namespace ff {

struct Cell {
    int x, y;
};

inline bool operator==(Cell a, Cell b) { return a.x == b.x && a.y == b.y; }

struct Vec2 {
    double x, y;
};

inline Cell cellOf(Vec2 p) { return {static_cast<int>(std::floor(p.x)), static_cast<int>(std::floor(p.y))}; }

struct Grid {
    int w = 0;
    int h = 0;
    std::vector<unsigned char> wall;

    Grid() = default;
    Grid(int width, int height) : w(width), h(height), wall(static_cast<std::size_t>(width) * static_cast<std::size_t>(height), 0) {}

    bool inBounds(int x, int y) const { return x >= 0 && y >= 0 && x < w && y < h; }
    std::size_t index(int x, int y) const { return static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x); }
    /// The outside of the grid counts as a wall.
    bool blocked(int x, int y) const { return !inBounds(x, y) || wall[index(x, y)] != 0; }
};

/// '#' is a wall, anything else is open. All rows must have the width of the first one.
inline Grid gridFromRows(const std::vector<std::string>& rows) {
    Grid g(rows.empty() ? 0 : static_cast<int>(rows[0].size()), static_cast<int>(rows.size()));
    for (int y = 0; y < g.h; ++y) {
        for (int x = 0; x < g.w; ++x) {
            const std::string& row = rows[static_cast<std::size_t>(y)];
            g.wall[g.index(x, y)] = static_cast<std::size_t>(x) < row.size() && row[static_cast<std::size_t>(x)] == '#' ? 1 : 0;
        }
    }
    return g;
}

template <std::size_t N>
Grid gridFromRows(const std::array<std::string_view, N>& rows) {
    std::vector<std::string> copy;
    for (const std::string_view r : rows) copy.emplace_back(r);
    return gridFromRows(copy);
}

/// Changes one wall. The goal cell and anything outside the grid stay as they are. Returns true if the grid changed.
inline bool setWall(Grid& g, Cell c, bool isWall, Cell goal) {
    if (!g.inBounds(c.x, c.y) || c == goal) return false;
    unsigned char& w = g.wall[g.index(c.x, c.y)];
    const unsigned char want = isWall ? 1 : 0;
    if (w == want) return false;
    w = want;
    return true;
}

inline constexpr int kUnreached = -1;
inline constexpr std::array<Cell, 4> kNeighbors{{{-1, 0}, {1, 0}, {0, -1}, {0, 1}}};  // left, right, up, down

/// Steps to the goal for every cell, kUnreached for walls and for cells that are cut off.
struct Field {
    int w = 0;
    int h = 0;
    std::vector<int> dist;
    int expanded = 0;  // cells taken from the queue: the work of this search
    int maxDist = 0;

    int at(int x, int y) const { return x < 0 || y < 0 || x >= w || y >= h ? kUnreached : dist[static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)]; }
};

/// Breadth first search from the goal over the four neighbours.
inline Field bfs(const Grid& g, Cell goal) {
    Field f;
    f.w = g.w;
    f.h = g.h;
    f.dist.assign(static_cast<std::size_t>(g.w) * static_cast<std::size_t>(g.h), kUnreached);
    if (g.blocked(goal.x, goal.y)) return f;
    std::vector<Cell> queue;
    queue.reserve(f.dist.size());
    queue.push_back(goal);
    f.dist[g.index(goal.x, goal.y)] = 0;
    for (std::size_t head = 0; head < queue.size(); ++head) {
        const Cell c = queue[head];
        const int d = f.dist[g.index(c.x, c.y)];
        ++f.expanded;
        f.maxDist = std::max(f.maxDist, d);
        for (const Cell n : kNeighbors) {
            const int nx = c.x + n.x;
            const int ny = c.y + n.y;
            if (g.blocked(nx, ny) || f.dist[g.index(nx, ny)] != kUnreached) continue;
            f.dist[g.index(nx, ny)] = d + 1;
            queue.push_back({nx, ny});
        }
    }
    return f;
}

/// The unit vector that points downhill on the distance map, (0, 0) for walls, the goal and unreached cells.
inline Vec2 flowDirection(const Grid& g, const Field& f, int x, int y) {
    const int c = f.at(x, y);
    if (g.blocked(x, y) || c <= 0) return {0.0, 0.0};
    const auto value = [&](int nx, int ny) {
        const int d = f.at(nx, ny);
        return g.blocked(nx, ny) || d == kUnreached ? c + 1 : d;
    };
    const double gx = static_cast<double>(value(x - 1, y) - value(x + 1, y));
    const double gy = static_cast<double>(value(x, y - 1) - value(x, y + 1));
    const double len = std::hypot(gx, gy);
    if (len > 1e-9) return {gx / len, gy / len};
    int best = c;
    Vec2 dir{0.0, 0.0};
    for (const Cell n : kNeighbors) {
        const int d = value(x + n.x, y + n.y);
        if (d < best) {
            best = d;
            dir = {static_cast<double>(n.x), static_cast<double>(n.y)};
        }
    }
    return dir;
}

/// One arrow per cell, computed once per search: this is the whole "flow field". An agent only reads one entry.
inline std::vector<Vec2> buildDirections(const Grid& g, const Field& f) {
    std::vector<Vec2> out(static_cast<std::size_t>(g.w) * static_cast<std::size_t>(g.h), Vec2{0.0, 0.0});
    for (int y = 0; y < g.h; ++y) {
        for (int x = 0; x < g.w; ++x) out[g.index(x, y)] = flowDirection(g, f, x, y);
    }
    return out;
}

/// Moves a circle out of every wall cell around it (the grid border counts as wall), so agents slide along walls.
inline Vec2 pushOutOfWalls(const Grid& g, Vec2 p, double r) {
    p.x = std::clamp(p.x, 0.0, static_cast<double>(g.w));
    p.y = std::clamp(p.y, 0.0, static_cast<double>(g.h));
    const Cell home = cellOf(p);
    for (int pass = 0; pass < 2; ++pass) {  // a push out of one cell can touch the next one
        for (int j = home.y - 1; j <= home.y + 1; ++j) {
            for (int i = home.x - 1; i <= home.x + 1; ++i) {
                if (!g.blocked(i, j)) continue;
                const double nx = std::clamp(p.x, static_cast<double>(i), static_cast<double>(i + 1));
                const double ny = std::clamp(p.y, static_cast<double>(j), static_cast<double>(j + 1));
                const double dx = p.x - nx;
                const double dy = p.y - ny;
                const double d2 = dx * dx + dy * dy;
                if (d2 >= r * r) continue;
                if (d2 > 1e-18) {
                    const double d = std::sqrt(d2);
                    p.x += dx / d * (r - d);
                    p.y += dy / d * (r - d);
                    continue;
                }
                // The centre is inside the box: leave through the nearest face.
                const double left = p.x - i;
                const double right = i + 1 - p.x;
                const double top = p.y - j;
                const double bottom = j + 1 - p.y;
                const double m = std::min({left, right, top, bottom});
                if (m == left) {
                    p.x = i - r;
                } else if (m == right) {
                    p.x = i + 1 + r;
                } else if (m == top) {
                    p.y = j - r;
                } else {
                    p.y = j + 1 + r;
                }
            }
        }
    }
    return p;
}

// ---- A*, the alternative: one search per agent, from the agent ----------------------------------------------------

struct Path {
    std::vector<Cell> cells;  // start to goal, both included; empty if there is none
    int expanded = 0;         // cells taken from the queue: the work of this search
    bool found = false;
};

inline int manhattan(Cell a, Cell b) { return std::abs(a.x - b.x) + std::abs(a.y - b.y); }

/// A* on the four neighbours with the Manhattan distance as heuristic (it never overestimates, so the path is a shortest one).
/// Among equal f the cell closer to the goal goes first, which keeps the search on a straight line in open ground.
inline Path astar(const Grid& g, Cell start, Cell goal) {
    Path out;
    if (g.blocked(start.x, start.y) || g.blocked(goal.x, goal.y)) return out;
    const std::size_t n = static_cast<std::size_t>(g.w) * static_cast<std::size_t>(g.h);
    std::vector<int> cost(n, INT_MAX);
    std::vector<int> parent(n, -1);
    std::vector<unsigned char> closed(n, 0);
    struct Item {
        int f, h;
        std::size_t index;
    };
    const auto later = [](const Item& a, const Item& b) { return a.f != b.f ? a.f > b.f : a.h > b.h; };
    std::priority_queue<Item, std::vector<Item>, decltype(later)> open(later);
    const std::size_t startIndex = g.index(start.x, start.y);
    const std::size_t goalIndex = g.index(goal.x, goal.y);
    cost[startIndex] = 0;
    open.push({manhattan(start, goal), manhattan(start, goal), startIndex});
    while (!open.empty()) {
        const Item item = open.top();
        open.pop();
        if (closed[item.index] != 0) continue;  // an older, worse entry of a cell that is done
        closed[item.index] = 1;
        ++out.expanded;
        if (item.index == goalIndex) {
            for (std::size_t i = goalIndex;; i = static_cast<std::size_t>(parent[i])) {
                out.cells.push_back({static_cast<int>(i % static_cast<std::size_t>(g.w)), static_cast<int>(i / static_cast<std::size_t>(g.w))});
                if (i == startIndex) break;
            }
            std::reverse(out.cells.begin(), out.cells.end());
            out.found = true;
            return out;
        }
        const Cell c{static_cast<int>(item.index % static_cast<std::size_t>(g.w)), static_cast<int>(item.index / static_cast<std::size_t>(g.w))};
        for (const Cell d : kNeighbors) {
            const Cell next{c.x + d.x, c.y + d.y};
            if (g.blocked(next.x, next.y)) continue;
            const std::size_t ni = g.index(next.x, next.y);
            if (closed[ni] != 0) continue;
            const int ng = cost[item.index] + 1;
            if (ng >= cost[ni]) continue;
            cost[ni] = ng;
            parent[ni] = static_cast<int>(item.index);
            const int h = manhattan(next, goal);
            open.push({ng + h, h, ni});
        }
    }
    return out;
}

struct AstarWork {
    long long expanded = 0;
    int searches = 0;
};

/// What it costs when every agent searches for itself: one A* from each start cell. Agents on the same cell run the
/// same search, which is computed once and counted for each of them. Starts inside walls cannot search and are skipped.
inline AstarWork astarWork(const Grid& g, Cell goal, const std::vector<Cell>& starts) {
    AstarWork total;
    std::vector<int> memo(static_cast<std::size_t>(g.w) * static_cast<std::size_t>(g.h), -1);
    for (const Cell s : starts) {
        if (g.blocked(s.x, s.y)) continue;
        int& e = memo[g.index(s.x, s.y)];
        if (e < 0) e = astar(g, s, goal).expanded;
        total.expanded += e;
        ++total.searches;
    }
    return total;
}

// ---- Agents spawn, numbers on screen -----------------------------------------------------------------------------------

/// xorshift32, so the demo and the tests see the same numbers for the same seed.
struct Rng {
    std::uint32_t state;
    explicit Rng(std::uint32_t seed) : state(seed != 0 ? seed : 0x9E3779B9U) {}
    std::uint32_t next() {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        return state;
    }
    int below(int n) { return static_cast<int>(next() % static_cast<std::uint32_t>(n)); }  // n > 0
    double uniform() { return static_cast<double>(next()) / 4294967296.0; }
};

/// A random reachable cell that is at least `minDist` steps from the goal (never the goal itself). If the field has
/// no cell that far, the farther half of it is used, so a small or walled-in map still gets agents.
inline std::optional<Cell> pickSpawn(const Field& f, int minDist, Rng& rng) {
    const int need = std::max(1, std::min(minDist, (f.maxDist + 1) / 2));
    int count = 0;
    for (const int d : f.dist) {
        if (d >= need) ++count;
    }
    if (count == 0) return std::nullopt;
    int pick = rng.below(count);
    for (std::size_t i = 0; i < f.dist.size(); ++i) {
        if (f.dist[i] < need) continue;
        if (pick-- == 0) return Cell{static_cast<int>(i % static_cast<std::size_t>(f.w)), static_cast<int>(i / static_cast<std::size_t>(f.w))};
    }
    return std::nullopt;
}

/// 41230 -> "41,230". Built by hand, snprintf would use the separators of the system locale.
inline std::string grouped(long long n) {
    std::string digits = std::to_string(n < 0 ? -n : n);
    std::string out;
    for (std::size_t i = 0; i < digits.size(); ++i) {
        if (i > 0 && (digits.size() - i) % 3 == 0) out += ',';
        out += digits[i];
    }
    return n < 0 ? "-" + out : out;
}

// ---- The map the demo starts with -----------------------------------------------------------------------------------------

/// 48 x 16: a wall with a gap at the bottom, a wall with a gap at the top, and a cup that opens towards the agents
/// right in front of the goal. A straight line to the goal leads into the cup, the real way round it is long.
inline std::array<std::string_view, 16> presetRows() {
    return {{
        "..............#.................................",
        "..............#.................................",
        "..............#.................................",
        "..............#.................................",
        "..............#.............#....######.........",
        "..............#.............#.........#.........",
        "..............#.............#.........#.........",
        "..............#.............#.........#.........",
        "..............#.............#.........#.........",
        "..............#.............#.........#.........",
        "..............#.............#.........#.........",
        "..............#.............#.........#.........",
        "............................#....######.........",
        "............................#...................",
        "............................#...................",
        "............................#...................",
    }};
}

inline Cell presetGoal() { return {43, 8}; }

}  // namespace ff
