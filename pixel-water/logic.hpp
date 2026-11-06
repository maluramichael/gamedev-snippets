// A falling sand world with water, no SDL. Every cell holds one particle (or nothing) and step() moves each
// particle at most once, using only its own neighbourhood. y grows downwards, row 0 is the top.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <utility>
#include <vector>

namespace pw {

enum class Kind : std::uint8_t { Empty, Wall, Sand, Water };

struct Cell {
    Kind kind = Kind::Empty;
    std::uint8_t tint = 0;       // fixed per particle at creation, travels with it, used for the colour variation
    std::uint32_t movedAt = 0;   // tick of the last move; a particle moves at most once per step
};

struct Counts {
    int empty = 0;
    int wall = 0;
    int sand = 0;
    int water = 0;
};

namespace detail {

inline std::uint32_t hash(int a, int b, std::uint32_t c) {
    std::uint32_t h = static_cast<std::uint32_t>(a) * 0x9E3779B1U ^ static_cast<std::uint32_t>(b) * 0x85EBCA77U ^ c * 0xC2B2AE3DU;
    h ^= h >> 15;
    h *= 0x2C1B3C6DU;
    h ^= h >> 12;
    h *= 0x297A2D39U;
    h ^= h >> 15;
    return h;
}

}  // namespace detail

class World {
public:
    World(int w, int h) : w_(w), h_(h), cells_(static_cast<std::size_t>(w) * static_cast<std::size_t>(h)) {}

    int width() const { return w_; }
    int height() const { return h_; }
    bool inside(int x, int y) const { return x >= 0 && y >= 0 && x < w_ && y < h_; }
    const Cell& cell(int x, int y) const { return cells_[index(x, y)]; }

    /// Outside the grid everything is solid, so nothing ever has to bounds-check a neighbour.
    Kind kind(int x, int y) const { return inside(x, y) ? cell(x, y).kind : Kind::Wall; }

    void set(int x, int y, Kind k) {
        if (!inside(x, y)) return;
        Cell& c = cells_[index(x, y)];
        c.kind = k;
        c.tint = k == Kind::Empty ? 0 : static_cast<std::uint8_t>(detail::hash(x, y, created_++) >> 24);
        c.movedAt = 0;
    }

    void clear() {
        for (Cell& c : cells_) c = Cell{};
    }

    Counts counts() const {
        Counts n;
        for (const Cell& c : cells_) {
            switch (c.kind) {
                case Kind::Empty: ++n.empty; break;
                case Kind::Wall: ++n.wall; break;
                case Kind::Sand: ++n.sand; break;
                case Kind::Water: ++n.water; break;
            }
        }
        return n;
    }

    std::uint32_t tick() const { return tick_; }

    /// How far water may slide sideways in one step. 1 gives slow, sticky leveling, larger values level quickly.
    int spread = 4;

    /// One simulation step. Rows are visited from the bottom to the top: a particle that falls lands in a row that
    /// was already handled, so it cannot be picked up a second time. Within a row the direction alternates, and the
    /// `movedAt` stamp covers the sideways moves, where the particle lands in a row that is still being scanned.
    void step() {
        ++tick_;
        for (int y = h_ - 1; y >= 0; --y) {
            const bool leftToRight = ((tick_ + static_cast<std::uint32_t>(y)) & 1U) == 0;
            for (int i = 0; i < w_; ++i) {
                const int x = leftToRight ? i : w_ - 1 - i;
                const Cell& c = cells_[index(x, y)];
                if (c.movedAt == tick_) continue;
                if (c.kind == Kind::Sand) {
                    stepSand(x, y);
                } else if (c.kind == Kind::Water) {
                    stepWater(x, y);
                }
            }
        }
    }

private:
    int w_, h_;
    std::vector<Cell> cells_;
    std::uint32_t tick_ = 0;
    std::uint32_t created_ = 0;

    std::size_t index(int x, int y) const { return static_cast<std::size_t>(y) * static_cast<std::size_t>(w_) + static_cast<std::size_t>(x); }

    /// Water only moves into air.
    bool isEmpty(int x, int y) const { return kind(x, y) == Kind::Empty; }

    /// Sand also sinks into water, but not into water that has already moved this step.
    bool sandCanEnter(int x, int y) const {
        const Kind k = kind(x, y);
        return k == Kind::Empty || (k == Kind::Water && cell(x, y).movedAt != tick_);
    }

    void move(int x, int y, int nx, int ny) {
        std::swap(cells_[index(x, y)], cells_[index(nx, ny)]);
        cells_[index(nx, ny)].movedAt = tick_;
        // Whatever swapped into the old spot (water, when sand sinks) has moved as well. Empty cells never carry a stamp.
        Cell& back = cells_[index(x, y)];
        back.movedAt = back.kind == Kind::Empty ? 0 : tick_;
    }

    /// 1 or -1, a different coin flip for every cell and every step, so nothing leans to one side.
    int coin(int x, int y) const { return (detail::hash(x, y, tick_) & 1U) != 0 ? 1 : -1; }

    void stepSand(int x, int y) {
        const int d = coin(x, y);
        if (sandCanEnter(x, y + 1)) {
            move(x, y, x, y + 1);
        } else if (sandCanEnter(x + d, y + 1)) {
            move(x, y, x + d, y + 1);
        } else if (sandCanEnter(x - d, y + 1)) {
            move(x, y, x - d, y + 1);
        }
    }

    /// Looks `dir` for up to `spread` free cells in this row. Returns the farthest one, or the first with air below
    /// it (a drop is worth stopping at). x itself means blocked.
    int probe(int x, int y, int dir, bool& drop) const {
        int target = x;
        drop = false;
        for (int i = 1; i <= spread; ++i) {
            const int nx = x + dir * i;
            if (!isEmpty(nx, y)) break;
            target = nx;
            if (isEmpty(nx, y + 1)) {
                drop = true;
                break;
            }
        }
        return target;
    }

    void stepWater(int x, int y) {
        const int d = coin(x, y);
        if (isEmpty(x, y + 1)) {
            move(x, y, x, y + 1);
        } else if (isEmpty(x + d, y + 1)) {
            move(x, y, x + d, y + 1);
        } else if (isEmpty(x - d, y + 1)) {
            move(x, y, x - d, y + 1);
        } else {
            bool dropA = false;
            bool dropB = false;
            const int a = probe(x, y, d, dropA);
            const int b = probe(x, y, -d, dropB);
            int target = x;
            if (dropA) {
                target = a;
            } else if (dropB) {
                target = b;
            } else if (a != x) {
                target = a;
            } else {
                target = b;
            }
            if (target != x) move(x, y, target, y);
        }
    }
};

/// Paints a filled disc (all cells with dx*dx + dy*dy <= r*r). Water and sand only go into empty cells, a wall
/// replaces whatever is there, Kind::Empty erases. `thin` writes about every third cell, so held-down painting pours
/// instead of dumping a block. Returns the number of cells written.
inline int paintDisc(World& w, int cx, int cy, int r, Kind kind, bool thin) {
    int written = 0;
    for (int y = cy - r; y <= cy + r; ++y) {
        for (int x = cx - r; x <= cx + r; ++x) {
            if (!w.inside(x, y)) continue;
            const int dx = x - cx;
            const int dy = y - cy;
            if (dx * dx + dy * dy > r * r) continue;
            if (thin && detail::hash(x, y, w.tick()) % 3U != 0U) continue;
            const Kind here = w.kind(x, y);
            const bool ok = kind == Kind::Water || kind == Kind::Sand ? here == Kind::Empty : kind == Kind::Wall ? here != Kind::Wall : here != Kind::Empty;
            if (!ok) continue;
            w.set(x, y, kind);
            ++written;
        }
    }
    return written;
}

/// A line of discs from one cell to another, so a fast mouse stroke leaves no gaps.
inline void stroke(World& w, int x0, int y0, int x1, int y1, int r, Kind kind, bool thin) {
    const int n = std::max(std::abs(x1 - x0), std::abs(y1 - y0));
    for (int i = 0; i <= n; ++i) {
        const double t = n == 0 ? 0.0 : static_cast<double>(i) / n;
        paintDisc(w, x0 + static_cast<int>(std::lround((x1 - x0) * t)), y0 + static_cast<int>(std::lround((y1 - y0) * t)), r, kind, thin);
    }
}

inline void fillRect(World& w, int x0, int y0, int x1, int y1, Kind k) {
    for (int y = y0; y <= y1; ++y)
        for (int x = x0; x <= x1; ++x) w.set(x, y, k);
}

/// The grid of the demo: 240 x 104 cells, drawn 4 x 4 pixels each.
inline constexpr int kCols = 240;
inline constexpr int kRows = 104;
/// Where the tap drips from (its left cell), directly under a stone hood.
inline constexpr int kTapX = 45;
inline constexpr int kTapY = 5;

/// The scene the demo starts with: a basin, a shelf the tap pours onto, a stone island in the basin, a hopper of sand
/// above it and two sand piles. Written for a kCols x kRows grid, a smaller grid just clips it.
inline void buildBasin(World& w) {
    fillRect(w, 0, 101, kCols - 1, 103, Kind::Wall);  // ground
    fillRect(w, 40, 3, 52, 4, Kind::Wall);            // hood over the tap
    fillRect(w, 14, 42, 98, 44, Kind::Wall);          // shelf under the tap
    fillRect(w, 14, 36, 16, 41, Kind::Wall);          // lip at the left end of the shelf
    fillRect(w, 88, 66, 90, 100, Kind::Wall);         // basin, left wall
    fillRect(w, 198, 66, 200, 100, Kind::Wall);       // basin, right wall
    fillRect(w, 140, 92, 156, 100, Kind::Wall);       // island in the basin
    fillRect(w, 130, 12, 156, 26, Kind::Sand);        // hopper of sand, falls onto the island
    for (int h = 0; h < 15; ++h) fillRect(w, 208 + h, 100 - h, 236 - h, 100 - h, Kind::Sand);        // pile right of the basin
    for (int h = 0; h < 12; ++h) fillRect(w, 30 + (3 * h) / 2, 100 - h, 70 - (3 * h) / 2, 100 - h, Kind::Sand);  // pile under the shelf
}

/// Drips `width` cells of water from (x, y) as long as the world holds fewer than `cap` water cells.
inline void pourTap(World& w, int x, int y, int width, int cap) {
    int water = w.counts().water;
    for (int i = 0; i < width && water < cap; ++i) {
        if (w.kind(x + i, y) != Kind::Empty) continue;
        w.set(x + i, y, Kind::Water);
        ++water;
    }
}

}  // namespace pw
