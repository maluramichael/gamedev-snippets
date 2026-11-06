// Lines and circles on a grid: the logic of the demo, no SDL, so test.cpp can check it.
//
// A cell (x, y) is the unit square centered on the integer point (x, y). A line runs from the center of one cell to
// the center of another, and there is more than one right answer to "which cells does it cover":
//
//   lerp        N = max(|dx|, |dy|) + 1 evenly spaced points on the line, each rounded to its cell. One cell per step
//               of the long axis. Rounding is done in integers, so a tie (the line crosses a cell edge exactly in the
//               middle of a step) always goes up and the result does not depend on floating point.
//   bresenham   the same count of cells, chosen by an error term instead of by rounding. On a tie it takes the cell
//               in the direction of travel, so the same line drawn from the other end can differ.
//   supercover  walks cell edge by cell edge and takes every cell the line touches. At an exact corner it either steps
//               diagonally (Corner::Diagonal) or also takes the two cells that only touch the line at that corner
//               (Corner::Both), which is what a line of sight check wants.
//
// Circles: the distance test (dx^2 + dy^2 <= r^2), the midpoint circle (one octant, mirrored eight times) and the
// per row shortcut (left and right cell of every row), which leaves gaps at the top and bottom.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <set>
#include <utility>
#include <vector>

namespace lcd {

struct Cell {
    int x, y;
    bool operator==(const Cell&) const = default;
};

using Cells = std::vector<Cell>;

enum class Corner { Diagonal, Both };

/// Floor division for b > 0 (C++ `/` truncates toward zero).
inline long long floorDiv(long long a, long long b) {
    long long q = a / b;
    if (a % b < 0) --q;
    return q;
}

/// Largest r with r * r <= n, for n >= 0.
inline long long isqrtFloor(long long n) {
    if (n <= 0) return 0;
    auto r = static_cast<long long>(std::sqrt(static_cast<double>(n)));
    while (r * r > n) --r;
    while ((r + 1) * (r + 1) <= n) ++r;
    return r;
}

inline bool sortedBefore(const Cell& a, const Cell& b) { return a.y != b.y ? a.y < b.y : a.x < b.x; }

/// Sorts row by row and drops duplicates.
inline void normalize(Cells& cells) {
    std::sort(cells.begin(), cells.end(), sortedBefore);
    cells.erase(std::unique(cells.begin(), cells.end()), cells.end());
}

/// The circle radius for a center and a point on the rim: the distance rounded to the nearest whole cell.
inline int radiusOf(Cell center, Cell rim) {
    const long long dx = rim.x - center.x, dy = rim.y - center.y;
    const long long d2 = dx * dx + dy * dy;
    if (d2 == 0) return 0;
    // round(sqrt(d2)) is the r with (r - 0.5)^2 <= d2 < (r + 0.5)^2, which for integers is r * (r - 1) < d2 <= r * (r + 1)
    long long r = isqrtFloor(d2);
    while (r * (r + 1) < d2) ++r;
    return static_cast<int>(r);
}

// ----------------------------------------------------------------------------------------------------------- lines

inline Cells lerpLine(Cell a, Cell b) {
    const long long dx = b.x - a.x, dy = b.y - a.y;
    const long long n = std::max(std::llabs(dx), std::llabs(dy));
    Cells out;
    if (n == 0) return {a};  // no division by zero for a line of length 0
    out.reserve(static_cast<std::size_t>(n) + 1);
    for (long long s = 0; s <= n; ++s) {
        // a + round(d * s / n), rounded half up, all in integers: floor((2 * d * s + n) / (2 * n))
        out.push_back({a.x + static_cast<int>(floorDiv(2 * dx * s + n, 2 * n)), a.y + static_cast<int>(floorDiv(2 * dy * s + n, 2 * n))});
    }
    return out;
}

inline Cells bresenhamLine(Cell a, Cell b) {
    const long long dx = std::llabs(static_cast<long long>(b.x) - a.x);
    const long long dy = -std::llabs(static_cast<long long>(b.y) - a.y);
    const int sx = a.x < b.x ? 1 : -1;
    const int sy = a.y < b.y ? 1 : -1;
    long long err = dx + dy;
    Cells out;
    int x = a.x, y = a.y;
    for (;;) {
        out.push_back({x, y});
        if (x == b.x && y == b.y) break;
        const long long e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y += sy;
        }
    }
    return out;
}

inline Cells supercoverLine(Cell a, Cell b, Corner corner) {
    const long long dx = b.x - a.x, dy = b.y - a.y;
    const long long nx = std::llabs(dx), ny = std::llabs(dy);
    const int sx = dx > 0 ? 1 : -1;
    const int sy = dy > 0 ? 1 : -1;
    int x = a.x, y = a.y;
    Cells out{{x, y}};
    long long ix = 0, iy = 0;
    while (ix < nx || iy < ny) {
        // Which edge does the line cross first: the vertical one after ix + 1/2 columns or the horizontal one after
        // iy + 1/2 rows? (0.5 + ix) / nx against (0.5 + iy) / ny, multiplied out so nothing is divided.
        const long long decision = (1 + 2 * ix) * ny - (1 + 2 * iy) * nx;
        if (decision == 0) {
            // exactly through a corner
            if (corner == Corner::Both) {
                out.push_back({x + sx, y});
                out.push_back({x, y + sy});
            }
            x += sx;
            y += sy;
            ++ix;
            ++iy;
        } else if (decision < 0) {
            x += sx;
            ++ix;
        } else {
            y += sy;
            ++iy;
        }
        out.push_back({x, y});
    }
    return out;
}

/// A grid of wall flags. Cells outside the grid are never walls.
class Walls {
public:
    Walls(int cols, int rows) : cols_(cols), rows_(rows), flags_(static_cast<std::size_t>(cols) * static_cast<std::size_t>(rows), 0) {}

    bool inside(Cell c) const { return c.x >= 0 && c.y >= 0 && c.x < cols_ && c.y < rows_; }
    bool at(Cell c) const { return inside(c) && flags_[index(c)] != 0; }
    void set(Cell c, bool wall) {
        if (inside(c)) flags_[index(c)] = wall ? 1 : 0;
    }
    int count() const { return static_cast<int>(std::count(flags_.begin(), flags_.end(), static_cast<char>(1))); }
    void clear() { std::fill(flags_.begin(), flags_.end(), 0); }

private:
    std::size_t index(Cell c) const { return static_cast<std::size_t>(c.y) * static_cast<std::size_t>(cols_) + static_cast<std::size_t>(c.x); }
    int cols_, rows_;
    std::vector<char> flags_;
};

/// Is the way from a to b free? Walls on the cells strictly between decide, the viewer's own cell and the target cell
/// do not count.
inline bool lineOfSight(const Cells& cells, Cell a, Cell b, const Walls& walls) {
    for (const Cell& c : cells) {
        if (c == a || c == b) continue;
        if (walls.at(c)) return false;
    }
    return true;
}

// The starting position of the demo. The line (3,12) -> (24,5) has a slope of -1/3, so it passes exactly through the
// corner between cells (4,12), (5,12), (4,11) and (5,11). lerp and Bresenham step from (4,12) to (5,11) without
// touching (5,12), supercover also takes that cell, so the wall there blocks only the supercover line of sight.
inline Cell linePresetA() { return {3, 12}; }
inline Cell linePresetB() { return {24, 5}; }
inline Walls lineWalls() {
    Walls w(28, 16);
    w.set({5, 12}, true);
    return w;
}

// --------------------------------------------------------------------------------------------------------- circles

/// Every cell with dx^2 + dy^2 <= r^2: the distance test over the bounding box.
inline Cells discDistance(Cell c, int r) {
    Cells out;
    const long long r2 = static_cast<long long>(r) * r;
    for (int y = c.y - r; y <= c.y + r; ++y) {
        for (int x = c.x - r; x <= c.x + r; ++x) {
            const long long dx = x - c.x, dy = y - c.y;
            if (dx * dx + dy * dy <= r2) out.push_back({x, y});
        }
    }
    return out;
}

/// The same disc, row by row: half the row width from the Pythagorean theorem, no distance test per cell.
inline Cells discByRows(Cell c, int r) {
    Cells out;
    const long long r2 = static_cast<long long>(r) * r;
    for (int dy = -r; dy <= r; ++dy) {
        const int half = static_cast<int>(isqrtFloor(r2 - static_cast<long long>(dy) * dy));
        for (int dx = -half; dx <= half; ++dx) out.push_back({c.x + dx, c.y + dy});
    }
    return out;
}

/// The cells (x, y) with x >= y >= 0 of the midpoint circle: the first octant, the one that is computed.
/// d is the decision value: below 0 the midpoint between the two candidates lies inside the circle, so x stays.
inline Cells midpointOctant(int r) {
    Cells out;
    int x = r, y = 0;
    long long d = 1 - static_cast<long long>(r);
    while (x >= y) {
        out.push_back({x, y});
        ++y;
        if (d < 0) {
            d += 2LL * y + 1;
        } else {
            --x;
            d += 2LL * (y - x) + 1;
        }
    }
    return out;
}

/// Mirrors the first octant into all eight and moves it to the center.
inline Cells ringMidpoint(Cell c, int r) {
    Cells out;
    for (const Cell& o : midpointOctant(r)) {
        const int x = o.x, y = o.y;
        const int pairs[8][2] = {{x, y}, {-x, y}, {x, -y}, {-x, -y}, {y, x}, {-y, x}, {y, -x}, {-y, -x}};
        for (const auto& p : pairs) out.push_back({c.x + p[0], c.y + p[1]});
    }
    normalize(out);
    return out;
}

/// The outline of the distance test disc: cells inside the disc that have a neighbour (left, right, up, down) outside.
inline Cells ringDistance(Cell c, int r) {
    const long long r2 = static_cast<long long>(r) * r;
    const auto in = [r2](long long dx, long long dy) { return dx * dx + dy * dy <= r2; };
    Cells out;
    for (int dy = -r; dy <= r; ++dy) {
        for (int dx = -r; dx <= r; ++dx) {
            if (!in(dx, dy)) continue;
            if (!in(dx - 1, dy) || !in(dx + 1, dy) || !in(dx, dy - 1) || !in(dx, dy + 1)) out.push_back({c.x + dx, c.y + dy});
        }
    }
    return out;
}

/// The tempting shortcut: for every row take the left and the right cell of the circle. Near the top and bottom the
/// circle runs flat, one row covers many columns, and those columns are missing.
inline Cells ringRowsOnly(Cell c, int r) {
    Cells out;
    const long long r2 = static_cast<long long>(r) * r;
    for (int dy = -r; dy <= r; ++dy) {
        const int half = static_cast<int>(isqrtFloor(r2 - static_cast<long long>(dy) * dy));
        out.push_back({c.x - half, c.y + dy});
        out.push_back({c.x + half, c.y + dy});
    }
    normalize(out);
    return out;
}

/// A filled disc from the midpoint ring: every row from its leftmost to its rightmost ring cell.
inline Cells discMidpoint(Cell c, int r) {
    const Cells ring = ringMidpoint(c, r);
    Cells out;
    std::size_t i = 0;
    while (i < ring.size()) {
        std::size_t j = i;
        while (j < ring.size() && ring[j].y == ring[i].y) ++j;
        // ring is sorted row by row, then by x
        for (int x = ring[i].x; x <= ring[j - 1].x; ++x) out.push_back({x, ring[i].y});
        i = j;
    }
    return out;
}

/// Is every cell reachable from every other one over the 8 neighbours? An empty set is not a shape.
inline bool connected8(const Cells& cells) {
    if (cells.empty()) return false;
    std::set<std::pair<int, int>> todo;
    for (const Cell& c : cells) todo.insert({c.x, c.y});
    std::vector<std::pair<int, int>> stack{*todo.begin()};
    todo.erase(todo.begin());
    while (!stack.empty()) {
        const auto [x, y] = stack.back();
        stack.pop_back();
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                const auto it = todo.find({x + dx, y + dy});
                if (it == todo.end()) continue;
                stack.push_back(*it);
                todo.erase(it);
            }
        }
    }
    return todo.empty();
}

/// The octant that the midpoint algorithm computes, as an offset from the center: x >= y >= 0.
inline bool inFirstOctant(int dx, int dy) { return dy >= 0 && dx >= dy; }

}  // namespace lcd
