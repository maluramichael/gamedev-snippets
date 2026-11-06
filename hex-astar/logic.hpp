// Pathfinding on a hex grid and on a square grid, without SDL (main.cpp draws it, test.cpp checks it).
//
// The point of this file: astar() below never learns which kind of grid it runs on. It asks three questions
// (who are the neighbors, what does it cost to step there, how far is it to the goal) and the grid answers.
// For a square grid the answers are four directions and |dx| + |dy|, for a hex grid six directions and the cube
// distance. The search itself is the same code.
//
// Cells live in one array, `id = row * width + col`, for both grids, so a map survives switching between them.
// Hex rows are "odd-r": odd rows sit half a cell to the right. Hex math uses axial coordinates (q, r).
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <queue>
#include <vector>

namespace ha {

enum class Topology { Hex, Square };

// Cost of entering a cell. A wall cannot be entered.
constexpr int kWall = 0;
constexpr int kGrass = 1;
constexpr int kForest = 3;
constexpr int kSwamp = 6;

constexpr int kCols = 26;
constexpr int kRows = 13;
constexpr double kSqrt3 = 1.7320508075688772;
constexpr double kHexSize = 20.0;     // distance from the center of a hex to a corner
constexpr double kSquareSize = 30.0;  // side of a square cell

struct Axial {
    int q;
    int r;
    bool operator==(const Axial&) const = default;
};

/// Odd rows are shifted right: subtract half the row (rounded down) from the column to get q.
constexpr Axial offsetToAxial(int col, int row) { return {col - (row - (row & 1)) / 2, row}; }
constexpr int axialToCol(Axial a) { return a.q + (a.r - (a.r & 1)) / 2; }

inline constexpr std::array<Axial, 6> kHexDirections{{{1, 0}, {1, -1}, {0, -1}, {-1, 0}, {-1, 1}, {0, 1}}};

/// Steps between two hexes: the largest of the three cube differences (s = -q - r).
inline int hexDistance(Axial a, Axial b) {
    const int dq = a.q - b.q;
    const int dr = a.r - b.r;
    const int ds = -dq - dr;
    return std::max({std::abs(dq), std::abs(dr), std::abs(ds)});
}

struct Grid {
    int width;
    int height;
    std::vector<int> terrain;  // cost to enter a cell, kWall = blocked

    Grid(int w, int h) : width(w), height(h), terrain(static_cast<std::size_t>(w * h), kGrass) {}

    int id(int col, int row) const { return row * width + col; }
    int col(int node) const { return node % width; }
    int row(int node) const { return node / width; }
    bool contains(int col, int row) const { return col >= 0 && col < width && row >= 0 && row < height; }
    bool valid(int node) const { return node >= 0 && node < width * height; }
    int cost(int node) const { return terrain[static_cast<std::size_t>(node)]; }
    void set(int col, int row, int value) { terrain[static_cast<std::size_t>(id(col, row))] = value; }
};

struct Neighbors {
    std::array<int, 6> ids{};
    int count = 0;
    const int* begin() const { return ids.data(); }
    const int* end() const { return ids.data() + count; }
};

/// The six neighbors on the map (walls and cells outside left out).
inline Neighbors hexNeighbors(const Grid& g, int node) {
    Neighbors out;
    const Axial a = offsetToAxial(g.col(node), g.row(node));
    for (const Axial d : kHexDirections) {
        const Axial n{a.q + d.q, a.r + d.r};
        const int col = axialToCol(n);
        if (g.contains(col, n.r) && g.cost(g.id(col, n.r)) != kWall) out.ids[static_cast<std::size_t>(out.count++)] = g.id(col, n.r);
    }
    return out;
}

/// The four neighbors on the map (walls and cells outside left out).
inline Neighbors squareNeighbors(const Grid& g, int node) {
    static constexpr std::array<std::array<int, 2>, 4> kSteps{{{1, 0}, {0, 1}, {-1, 0}, {0, -1}}};
    Neighbors out;
    for (const auto& s : kSteps) {
        const int col = g.col(node) + s[0];
        const int row = g.row(node) + s[1];
        if (g.contains(col, row) && g.cost(g.id(col, row)) != kWall) out.ids[static_cast<std::size_t>(out.count++)] = g.id(col, row);
    }
    return out;
}

inline int hexGridDistance(const Grid& g, int a, int b) {
    return hexDistance(offsetToAxial(g.col(a), g.row(a)), offsetToAxial(g.col(b), g.row(b)));
}

inline int squareGridDistance(const Grid& g, int a, int b) { return std::abs(g.col(a) - g.col(b)) + std::abs(g.row(a) - g.row(b)); }

inline Neighbors neighborsOf(const Grid& g, Topology t, int node) { return t == Topology::Hex ? hexNeighbors(g, node) : squareNeighbors(g, node); }

inline int distance(const Grid& g, Topology t, int a, int b) { return t == Topology::Hex ? hexGridDistance(g, a, b) : squareGridDistance(g, a, b); }

struct SearchResult {
    bool found = false;
    std::vector<int> path;           // start to goal, both included; empty when not found
    int cost = 0;                    // sum of the cells entered along the path
    int expanded = 0;                // cells taken off the queue and processed
    std::vector<int> costSoFar;      // best known cost per cell, -1 = never reached
    std::vector<unsigned char> closed;  // 1 = was expanded
};

/// A*. `neighbors(node)` yields what can be reached from node, `stepCost(node)` is the price of entering it and
/// `heuristic(node)` must never overestimate the remaining cost (then the first time the goal is taken off the
/// queue, the path is the cheapest one). A heuristic that always returns 0 turns this into Dijkstra.
template <class NeighborsFn, class StepCostFn, class HeuristicFn>
SearchResult astar(int nodeCount, int start, int goal, NeighborsFn neighbors, StepCostFn stepCost, HeuristicFn heuristic) {
    SearchResult result;
    const auto n = static_cast<std::size_t>(nodeCount);
    result.costSoFar.assign(n, -1);
    result.closed.assign(n, 0);
    if (start < 0 || start >= nodeCount || goal < 0 || goal >= nodeCount) return result;

    struct Entry {
        int f;
        int g;
        int node;
    };
    // Smallest f first. Among equal f the larger g wins, that is the cell closer to the goal.
    const auto later = [](const Entry& a, const Entry& b) { return a.f != b.f ? a.f > b.f : a.g < b.g; };
    std::priority_queue<Entry, std::vector<Entry>, decltype(later)> open(later);
    std::vector<int> cameFrom(n, -1);

    result.costSoFar[static_cast<std::size_t>(start)] = 0;
    open.push({heuristic(start), 0, start});
    while (!open.empty()) {
        const Entry cur = open.top();
        open.pop();
        if (cur.g > result.costSoFar[static_cast<std::size_t>(cur.node)]) continue;  // a cheaper way was found meanwhile
        result.closed[static_cast<std::size_t>(cur.node)] = 1;
        ++result.expanded;
        if (cur.node == goal) {
            result.found = true;
            break;
        }
        for (const int next : neighbors(cur.node)) {
            const int g = cur.g + stepCost(next);
            int& known = result.costSoFar[static_cast<std::size_t>(next)];
            if (known == -1 || g < known) {
                known = g;
                cameFrom[static_cast<std::size_t>(next)] = cur.node;
                open.push({g + heuristic(next), g, next});
            }
        }
    }
    if (result.found) {
        result.cost = result.costSoFar[static_cast<std::size_t>(goal)];
        for (int at = goal; at != -1; at = cameFrom[static_cast<std::size_t>(at)]) result.path.push_back(at);
        std::reverse(result.path.begin(), result.path.end());
    }
    return result;
}

/// The search on this map. `useHeuristic = false` is Dijkstra.
inline SearchResult search(const Grid& g, Topology t, int start, int goal, bool useHeuristic) {
    if (!g.valid(start) || !g.valid(goal) || g.cost(start) == kWall || g.cost(goal) == kWall) {
        SearchResult none;  // sized like a real result so the caller can draw it
        none.costSoFar.assign(g.terrain.size(), -1);
        none.closed.assign(g.terrain.size(), 0);
        return none;
    }
    return astar(
        g.width * g.height, start, goal,
        [&](int node) { return neighborsOf(g, t, node); },
        [&](int node) { return g.cost(node); },
        [&](int node) { return useHeuristic ? distance(g, t, node, goal) : 0; });
}

// ---- Geometry: where a cell is on the 960 x 540 canvas, and which cell a point belongs to ----

struct Point {
    double x;
    double y;
};

struct Layout {
    Topology topology;
    double originX;  // left edge of the map
    double originY;  // top edge of the map
    double size;     // hex: center to corner, square: side
};

inline double cellWidth(const Layout& l) { return l.topology == Topology::Hex ? kSqrt3 * l.size : l.size; }

/// Map centered horizontally on the canvas, top edge at `top`.
inline Layout makeLayout(Topology t, const Grid& g, double canvasWidth, double top) {
    Layout l{t, 0.0, top, t == Topology::Hex ? kHexSize : kSquareSize};
    const double columns = static_cast<double>(g.width) + (t == Topology::Hex ? 0.5 : 0.0);  // odd rows stick out
    l.originX = (canvasWidth - cellWidth(l) * columns) / 2.0;
    return l;
}

inline Point centerOf(const Layout& l, const Grid& g, int node) {
    const int col = g.col(node);
    const int row = g.row(node);
    if (l.topology == Topology::Square) return {l.originX + l.size * (col + 0.5), l.originY + l.size * (row + 0.5)};
    return {l.originX + cellWidth(l) * (col + 0.5 + 0.5 * (row & 1)), l.originY + l.size * (1.0 + 1.5 * row)};
}

/// Corner i of a pointy-top hex, i = 4 is the top and 5 the upper right (y points down).
inline Point hexCorner(Point center, double size, int i) {
    const double angle = (60.0 * i + 30.0) * 3.14159265358979323846 / 180.0;
    return {center.x + size * std::cos(angle), center.y + size * std::sin(angle)};
}

/// The cell under (x, y), or -1 outside the map. For hexes the cell whose center is nearest is the right one (a hex is
/// the set of points closest to its center), then a point test against that hex rejects everything beyond the map edge.
inline int cellAt(const Layout& l, const Grid& g, double x, double y) {
    if (l.topology == Topology::Square) {
        const int col = static_cast<int>(std::floor((x - l.originX) / l.size));
        const int row = static_cast<int>(std::floor((y - l.originY) / l.size));
        return g.contains(col, row) ? g.id(col, row) : -1;
    }
    int best = -1;
    double bestSq = 1e300;
    for (int node = 0; node < g.width * g.height; ++node) {
        const Point c = centerOf(l, g, node);
        const double d = (c.x - x) * (c.x - x) + (c.y - y) * (c.y - y);
        if (d < bestSq) {
            bestSq = d;
            best = node;
        }
    }
    const Point c = centerOf(l, g, best);
    const double dx = std::fabs(x - c.x);
    const double dy = std::fabs(y - c.y);
    const bool inside = dx <= cellWidth(l) / 2.0 && dy <= l.size - dx / kSqrt3;
    return inside ? best : -1;
}

// ---- The map the demo starts with ----

constexpr int kDefaultStart = 6 * kCols + 3;   // col 3, row 6
constexpr int kDefaultGoal = 6 * kCols + 22;   // col 22, row 6

/// Two walls with gaps on opposite ends make an S-shaped way. Between them lies a swamp, near the goal a forest.
inline void applyDefaultMap(Grid& g) {
    std::fill(g.terrain.begin(), g.terrain.end(), kGrass);
    for (int row = 0; row <= 10; ++row) g.set(8, row, kWall);   // gap at the bottom
    for (int row = 2; row < g.height; ++row) g.set(17, row, kWall);  // gap at the top
    for (int col = 10; col <= 14; ++col) {
        for (int row = 4; row <= 8; ++row) g.set(col, row, kSwamp);
    }
    for (int col = 19; col <= 21; ++col) {
        for (int row = 4; row <= 9; ++row) g.set(col, row, kForest);
    }
}

}  // namespace ha
