// Pathfinding on a grid: breadth first search, Dijkstra, greedy best first and A*, all in one small class.
// No SDL in here, so test.cpp can check it on hand-computed grids.
//
// The four searches are the same loop. They differ only in the number that decides which waiting cell is taken next:
//   BFS       order of discovery            (every step counts the same, terrain is ignored)
//   Dijkstra  g                             (cost so far)
//   Greedy    h                             (estimated cost to the goal)
//   A*        g + weight * h                (weight 0 is Dijkstra, a large weight is greedy)
// Each cell is expanded at most once. That is what makes a weight above 1 give paths that are not the cheapest,
// because a cell that was closed too early is never reopened.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <optional>
#include <queue>
#include <vector>

namespace pf {

inline constexpr double kSqrt2 = 1.4142135623730951;

struct Cell {
    int x = 0;
    int y = 0;
    bool operator==(const Cell&) const = default;
};

// Terrain value = cost of entering the cell. 0 is a wall.
inline constexpr std::uint8_t kWall = 0;
inline constexpr std::uint8_t kGrass = 1;
inline constexpr std::uint8_t kMud = 5;

class Grid {
public:
    Grid(int width, int height) : w_(std::max(width, 0)), h_(std::max(height, 0)), cost_(static_cast<std::size_t>(w_) * static_cast<std::size_t>(h_), kGrass) {}

    int width() const { return w_; }
    int height() const { return h_; }
    bool contains(int x, int y) const { return x >= 0 && y >= 0 && x < w_ && y < h_; }
    /// Outside the grid counts as a wall.
    std::uint8_t at(int x, int y) const { return contains(x, y) ? cost_[index(x, y)] : kWall; }
    bool passable(int x, int y) const { return at(x, y) != kWall; }
    void set(int x, int y, std::uint8_t value) {
        if (contains(x, y)) cost_[index(x, y)] = value;
    }
    void clear() { std::fill(cost_.begin(), cost_.end(), kGrass); }
    std::size_t index(int x, int y) const { return static_cast<std::size_t>(y) * static_cast<std::size_t>(w_) + static_cast<std::size_t>(x); }

private:
    int w_;
    int h_;
    std::vector<std::uint8_t> cost_;
};

/// Estimated cost from a to b. Manhattan distance for four directions, octile distance (diagonals cost sqrt 2)
/// for eight. Both never overestimate, because the cheapest terrain costs 1 per step.
inline double heuristic(Cell a, Cell b, bool diagonals) {
    const double dx = std::abs(static_cast<double>(a.x) - static_cast<double>(b.x));
    const double dy = std::abs(static_cast<double>(a.y) - static_cast<double>(b.y));
    if (!diagonals) return dx + dy;
    return (dx + dy) + (kSqrt2 - 2.0) * std::min(dx, dy);
}

/// May the step from `from` by (dx, dy) be taken? The target must be free, and a diagonal step is not allowed to
/// cut the corner of a wall: both cells it passes between must be free as well.
inline bool canStep(const Grid& grid, Cell from, int dx, int dy) {
    if (!grid.passable(from.x, from.y)) return false;
    if (!grid.passable(from.x + dx, from.y + dy)) return false;
    if (dx != 0 && dy != 0) return grid.passable(from.x + dx, from.y) && grid.passable(from.x, from.y + dy);
    return true;
}

/// Real cost of walking along `path` (neighbouring cells): terrain of each entered cell, times sqrt 2 on a diagonal.
inline double pathCost(const Grid& grid, const std::vector<Cell>& path) {
    double total = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
        const bool diagonal = path[i].x != path[i - 1].x && path[i].y != path[i - 1].y;
        total += static_cast<double>(grid.at(path[i].x, path[i].y)) * (diagonal ? kSqrt2 : 1.0);
    }
    return total;
}

/// The cell under a pixel, or nothing outside the grid. Floors before converting, so a pixel left of the origin is
/// not truncated into cell 0.
inline std::optional<Cell> pickCell(double px, double py, double originX, double originY, double size, int width, int height) {
    const double fx = std::floor((px - originX) / size);
    const double fy = std::floor((py - originY) / size);
    if (!(fx >= 0.0 && fx < static_cast<double>(width) && fy >= 0.0 && fy < static_cast<double>(height))) return std::nullopt;
    return Cell{static_cast<int>(fx), static_cast<int>(fy)};
}

/// All cells on the line from a to b, both endpoints included (Bresenham). Paints without gaps when the mouse
/// jumps over several cells between two events.
inline std::vector<Cell> lineCells(Cell a, Cell b) {
    std::vector<Cell> out;
    const int dx = std::abs(b.x - a.x);
    const int dy = -std::abs(b.y - a.y);
    const int sx = a.x < b.x ? 1 : -1;
    const int sy = a.y < b.y ? 1 : -1;
    int err = dx + dy;
    Cell c = a;
    while (true) {
        out.push_back(c);
        if (c == b) break;
        const int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            c.x += sx;
        }
        if (e2 <= dx) {
            err += dx;
            c.y += sy;
        }
    }
    return out;
}

// The weight slider: 0 to 10 in steps of 0.5.
inline constexpr double kMaxWeight = 10.0;
inline constexpr double kWeightStep = 0.5;

inline double snapWeight(double w) {
    if (!(w > 0.0)) return 0.0;
    return std::round(std::min(w, kMaxWeight) / kWeightStep) * kWeightStep;
}
inline double weightFromT(double t) {
    if (!(t > 0.0)) return 0.0;
    return snapWeight(std::min(t, 1.0) * kMaxWeight);
}
inline double tFromWeight(double w) { return snapWeight(w) / kMaxWeight; }

enum class Algo { Bfs, Dijkstra, Greedy, AStar };
enum class Status { Running, Found, NoPath };
enum class CellState : std::uint8_t { Unseen, Open, Closed };

struct Options {
    Algo algo = Algo::AStar;
    bool diagonals = false;
    double weight = 1.0;  // only used by A*
};

class Search {
public:
    Search(const Grid& grid, Cell start, Cell goal, Options options)
        : grid_(grid),
          goal_(goal),
          options_(options),
          g_(cellCount(grid), std::numeric_limits<double>::infinity()),
          parent_(cellCount(grid), -1),
          state_(cellCount(grid), CellState::Unseen) {
        if (!grid_.passable(start.x, start.y) || !grid_.passable(goal.x, goal.y)) {
            status_ = Status::NoPath;
            return;
        }
        const std::size_t s = grid_.index(start.x, start.y);
        g_[s] = 0.0;
        state_[s] = CellState::Open;
        ++frontier_;
        push(s, start, 0.0);
    }

    /// One expansion: take the best waiting cell and look at its neighbours. Returns true while the search is still
    /// running afterwards, false once it has finished (also when it was finished before the call).
    bool step() {
        if (status_ != Status::Running) return false;
        const auto cell = popBest();
        if (!cell) {
            status_ = Status::NoPath;
            return false;
        }
        const Cell cur = *cell;
        const std::size_t ci = grid_.index(cur.x, cur.y);
        state_[ci] = CellState::Closed;
        --frontier_;
        order_.push_back(cur);
        if (cur == goal_) {
            status_ = Status::Found;
            buildPath();
            return false;
        }
        const int count = options_.diagonals ? 8 : 4;
        for (int d = 0; d < count; ++d) {
            const int dx = kDirs[d][0];
            const int dy = kDirs[d][1];
            if (!canStep(grid_, cur, dx, dy)) continue;
            const Cell next{cur.x + dx, cur.y + dy};
            const std::size_t ni = grid_.index(next.x, next.y);
            if (state_[ni] == CellState::Closed) continue;
            const bool diagonal = dx != 0 && dy != 0;
            const double terrain = options_.algo == Algo::Bfs ? 1.0 : static_cast<double>(grid_.at(next.x, next.y));
            const double newG = g_[ci] + terrain * (diagonal ? kSqrt2 : 1.0);
            const bool unseen = state_[ni] == CellState::Unseen;
            // BFS and greedy take the first way they find. Dijkstra and A* also switch to a cheaper way.
            const bool better = !unseen && newG < g_[ni] && (options_.algo == Algo::Dijkstra || options_.algo == Algo::AStar);
            if (!unseen && !better) continue;
            if (unseen) {
                state_[ni] = CellState::Open;
                ++frontier_;
            }
            g_[ni] = newG;
            parent_[ni] = static_cast<int>(ci);
            push(ni, next, newG);
        }
        return true;
    }

    void runToEnd() {
        while (step()) {
        }
    }

    Status status() const { return status_; }
    std::size_t expanded() const { return order_.size(); }
    std::size_t frontier() const { return frontier_; }
    /// Cells in the order they were expanded.
    const std::vector<Cell>& order() const { return order_; }
    /// Start to goal, empty unless the search found one.
    const std::vector<Cell>& path() const { return path_; }
    std::optional<Cell> lastExpanded() const {
        if (order_.empty()) return std::nullopt;
        return order_.back();
    }
    CellState stateAt(int x, int y) const { return grid_.contains(x, y) ? state_[grid_.index(x, y)] : CellState::Unseen; }

private:
    struct Entry {
        double key;
        double tie;
        std::uint64_t seq;
        std::size_t index;
        double g;
        bool operator>(const Entry& o) const {
            if (key != o.key) return key > o.key;
            if (tie != o.tie) return tie > o.tie;
            return seq > o.seq;
        }
    };

    static constexpr int kDirs[8][2] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}, {1, 1}, {-1, 1}, {-1, -1}, {1, -1}};

    static std::size_t cellCount(const Grid& g) { return static_cast<std::size_t>(g.width()) * static_cast<std::size_t>(g.height()); }

    void push(std::size_t index, Cell cell, double g) {
        const double h = heuristic(cell, goal_, options_.diagonals);
        double key = 0.0;
        double tie = h;  // among equal keys the cell closer to the goal goes first
        switch (options_.algo) {
            case Algo::Bfs: key = 0.0; tie = 0.0; break;  // equal keys: first come, first served
            case Algo::Dijkstra: key = g; break;
            case Algo::Greedy: key = h; tie = 0.0; break;
            case Algo::AStar: key = g + options_.weight * h; break;
        }
        queue_.push(Entry{key, tie, seq_++, index, g});
    }

    /// Best waiting cell, skipping entries that a cheaper entry for the same cell has replaced.
    std::optional<Cell> popBest() {
        while (!queue_.empty()) {
            const Entry e = queue_.top();
            queue_.pop();
            if (state_[e.index] != CellState::Open || e.g != g_[e.index]) continue;
            const int w = grid_.width();
            return Cell{static_cast<int>(e.index % static_cast<std::size_t>(w)), static_cast<int>(e.index / static_cast<std::size_t>(w))};
        }
        return std::nullopt;
    }

    void buildPath() {
        path_.clear();
        const int w = grid_.width();
        int i = static_cast<int>(grid_.index(goal_.x, goal_.y));
        while (i >= 0) {
            path_.push_back(Cell{i % w, i / w});
            i = parent_[static_cast<std::size_t>(i)];
        }
        std::reverse(path_.begin(), path_.end());
    }

    Grid grid_;
    Cell goal_;
    Options options_;
    std::vector<double> g_;
    std::vector<int> parent_;
    std::vector<CellState> state_;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<>> queue_;
    std::uint64_t seq_ = 0;
    std::size_t frontier_ = 0;
    std::vector<Cell> order_;
    std::vector<Cell> path_;
    Status status_ = Status::Running;
};

/// The map the demo starts with: a bucket that opens towards the start (the trap for greedy search) and a patch of
/// mud on the straight line (the reason BFS and greedy do not find the cheapest path).
struct Preset {
    Grid grid;
    Cell start;
    Cell goal;
};

inline Preset presetMap() {
    Preset p{Grid(46, 18), Cell{3, 9}, Cell{42, 9}};
    const auto rect = [&p](int x0, int y0, int x1, int y1, std::uint8_t value) {
        for (int y = y0; y <= y1; ++y) {
            for (int x = x0; x <= x1; ++x) p.grid.set(x, y, value);
        }
    };
    rect(30, 4, 30, 14, kWall);   // back of the bucket
    rect(22, 4, 30, 4, kWall);    // top arm
    rect(22, 14, 30, 14, kWall);  // bottom arm
    rect(10, 6, 16, 12, kMud);
    rect(8, 12, 8, 17, kWall);
    rect(38, 2, 38, 8, kWall);
    rect(34, 11, 40, 11, kWall);
    return p;
}

}  // namespace pf
