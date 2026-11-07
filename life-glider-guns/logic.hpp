// Conway's Game of Life: the logic of the demo, no SDL, so test.cpp can check it.
//
// A cell is alive or dead. Every generation a live cell with 2 or 3 live neighbors stays alive, a dead cell with exactly 3
// becomes alive, everything else is dead afterwards. Neighbors are the 8 cells around it.
//
// THE EDGE decides what happens at the border. On a TORUS the left edge touches the right edge and the top touches the
// bottom, so a glider that leaves on one side comes back on the opposite one. With WALLS everything beyond the border
// counts as dead, and a glider that hits one ends as a 2x2 block.
//
// AGE: a dead cell is 0, a live cell stores how many generations it has been alive (1 to 255). The rules do not need it,
// the demo uses it to color stable structures (blocks, the body of the gun) differently from things that move.
//
// COORDINATES: x grows to the right, y grows downward, like on screen. A shape is a list of cells with its top left corner at (0, 0).
// ROTATION: one step turns a shape by 90 degrees clockwise on screen. The glider in its base orientation heads southeast.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <vector>

namespace life {

constexpr int kCols = 160;
constexpr int kRows = 64;
constexpr int kMinSize = 3;  // the neighbors of a cell on a torus must be 8 different cells
constexpr int kMaxAge = 255;

struct Cell {
    int x = 0;
    int y = 0;
};
inline bool operator==(Cell a, Cell b) { return a.x == b.x && a.y == b.y; }
inline bool operator<(Cell a, Cell b) { return a.y != b.y ? a.y < b.y : a.x < b.x; }

using Shape = std::vector<Cell>;

inline int mod(int a, int n) { return ((a % n) + n) % n; }

class World {
public:
    World(int width, int height, bool torus)
        : w_(std::max(kMinSize, width)), h_(std::max(kMinSize, height)), torus_(torus),
          cur_(static_cast<std::size_t>(w_) * static_cast<std::size_t>(h_), 0), next_(cur_.size(), 0) {}

    int width() const { return w_; }
    int height() const { return h_; }
    bool torus() const { return torus_; }
    void setTorus(bool torus) { torus_ = torus; }
    long generation() const { return generation_; }

    /// Index of a cell, wrapped on a torus, -1 when it lies outside a walled world.
    int index(int x, int y) const {
        if (torus_) {
            x = mod(x, w_);
            y = mod(y, h_);
        } else if (x < 0 || y < 0 || x >= w_ || y >= h_) {
            return -1;
        }
        return y * w_ + x;
    }

    bool alive(int x, int y) const {
        const int i = index(x, y);
        return i >= 0 && cur_[static_cast<std::size_t>(i)] != 0;
    }

    /// Generations the cell has been alive, 0 for a dead cell or one outside a walled world.
    int age(int x, int y) const {
        const int i = index(x, y);
        return i < 0 ? 0 : cur_[static_cast<std::size_t>(i)];
    }

    /// Sets a cell. A cell that is already alive keeps its age. Outside a walled world nothing happens.
    void set(int x, int y, bool on) {
        const int i = index(x, y);
        if (i < 0) return;
        std::uint8_t& c = cur_[static_cast<std::size_t>(i)];
        c = on ? (c != 0 ? c : std::uint8_t{1}) : std::uint8_t{0};
    }

    /// Puts a shape with its top left corner at (ox, oy). Cells that fall outside a walled world are dropped, on a torus they wrap.
    void stamp(const Shape& shape, int ox, int oy) {
        for (const Cell& c : shape) set(ox + c.x, oy + c.y, true);
    }

    void clear() {
        std::fill(cur_.begin(), cur_.end(), std::uint8_t{0});
        generation_ = 0;
    }

    int population() const {
        return static_cast<int>(std::count_if(cur_.begin(), cur_.end(), [](std::uint8_t c) { return c != 0; }));
    }

    /// All live cells, row by row.
    Shape cells() const {
        Shape out;
        for (int y = 0; y < h_; ++y) {
            for (int x = 0; x < w_; ++x) {
                if (cur_[static_cast<std::size_t>(y * w_ + x)] != 0) out.push_back({x, y});
            }
        }
        return out;
    }

    void step() {
        for (int y = 0; y < h_; ++y) {
            for (int x = 0; x < w_; ++x) {
                int neighbors = 0;
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (dx == 0 && dy == 0) continue;
                        const int i = index(x + dx, y + dy);
                        if (i >= 0 && cur_[static_cast<std::size_t>(i)] != 0) ++neighbors;
                    }
                }
                const std::size_t here = static_cast<std::size_t>(y * w_ + x);
                const int age = cur_[here];
                if (age != 0) {
                    next_[here] = (neighbors == 2 || neighbors == 3) ? static_cast<std::uint8_t>(std::min(age + 1, kMaxAge)) : std::uint8_t{0};
                } else {
                    next_[here] = neighbors == 3 ? std::uint8_t{1} : std::uint8_t{0};
                }
            }
        }
        cur_.swap(next_);
        ++generation_;
    }

private:
    int w_;
    int h_;
    bool torus_;
    long generation_ = 0;
    std::vector<std::uint8_t> cur_;
    std::vector<std::uint8_t> next_;
};

// ---------------------------------------------------------------- shapes

inline Shape normalized(Shape s) {
    if (s.empty()) return s;
    int minX = s[0].x;
    int minY = s[0].y;
    for (const Cell& c : s) {
        minX = std::min(minX, c.x);
        minY = std::min(minY, c.y);
    }
    for (Cell& c : s) {
        c.x -= minX;
        c.y -= minY;
    }
    std::sort(s.begin(), s.end());
    return s;
}

/// One step of rotation: 90 degrees clockwise on screen (y points down), moved back to the top left corner.
inline Shape rotated90(const Shape& s) {
    Shape out;
    out.reserve(s.size());
    for (const Cell& c : s) out.push_back({-c.y, c.x});
    return normalized(std::move(out));
}

struct Extent {
    int w = 0;
    int h = 0;
};

inline Extent extent(const Shape& s) {
    Extent e;
    for (const Cell& c : s) {
        e.w = std::max(e.w, c.x + 1);
        e.h = std::max(e.h, c.y + 1);
    }
    return e;
}

enum class Kind { Glider, Block, Blinker, Gun };

inline Shape gliderBase() { return normalized({{1, 0}, {2, 1}, {0, 2}, {1, 2}, {2, 2}}); }

/// The glider has four phases. Each one is the previous one after one generation, found by running them.
inline Shape gliderPhase(int phase) {
    World w(9, 9, false);
    w.stamp(gliderBase(), 2, 2);
    for (int i = 0; i < mod(phase, 4); ++i) w.step();
    return normalized(w.cells());
}

/// The Gosper glider gun: 36 cells, 9 rows high, shoots a glider toward the southeast every 30 generations.
inline Shape gunShape() {
    static constexpr std::array<const char*, 9> kRows{{
        "........................O...........",
        "......................O.O...........",
        "............OO......OO............OO",
        "...........O...O....OO............OO",
        "OO........O.....O...OO..............",
        "OO........O...O.OO....O.O...........",
        "..........O.....O.......O...........",
        "...........O...O....................",
        "............OO......................",
    }};
    Shape s;
    for (int y = 0; y < static_cast<int>(kRows.size()); ++y) {
        for (int x = 0; kRows[static_cast<std::size_t>(y)][x] != '\0'; ++x) {
            if (kRows[static_cast<std::size_t>(y)][x] == 'O') s.push_back({x, y});
        }
    }
    return normalized(std::move(s));
}

/// A shape of the palette. `rotation` counts 90 degree steps clockwise, `phase` only matters for the glider.
inline Shape shapeOf(Kind kind, int rotation = 0, int phase = 0) {
    Shape s;
    switch (kind) {
        case Kind::Glider: s = gliderPhase(phase); break;
        case Kind::Block: s = normalized({{0, 0}, {1, 0}, {0, 1}, {1, 1}}); break;
        case Kind::Blinker: s = normalized({{0, 0}, {1, 0}, {2, 0}}); break;
        case Kind::Gun: s = gunShape(); break;
    }
    for (int i = 0; i < mod(rotation, 4); ++i) s = rotated90(s);
    return s;
}

/// Where a glider with this rotation goes in four generations: (1, 1) for the base orientation.
inline Cell gliderHeading(int rotation) {
    static constexpr std::array<Cell, 4> kHeading{{{1, 1}, {-1, 1}, {-1, -1}, {1, -1}}};
    return kHeading[static_cast<std::size_t>(mod(rotation, 4))];
}

inline const char* headingName(int rotation) {
    static constexpr std::array<const char*, 4> kNames{{"SE", "SW", "NW", "NE"}};
    return kNames[static_cast<std::size_t>(mod(rotation, 4))];
}

/// Top left corner that puts the middle of the shape on the cell (cx, cy).
inline Cell topLeftFor(const Shape& s, int cx, int cy) {
    const Extent e = extent(s);
    return {cx - e.w / 2, cy - e.h / 2};
}

// ---------------------------------------------------------------- two gliders that meet

/// A glider heading southeast and one heading northwest start 20 cells apart on the same diagonal, the second one moved by
/// (dx, dy). Where they end up depends on that offset alone. The outcomes after 150 generations were checked against a
/// separate Python implementation (see test.cpp).
struct Collision {
    int dx;
    int dy;
};

constexpr std::array<Collision, 4> kCollisions{{{0, -6}, {0, 0}, {1, 1}, {3, -2}}};

inline void loadCollision(World& w, int index, int ox, int oy) {
    const Collision& c = kCollisions[static_cast<std::size_t>(mod(index, static_cast<int>(kCollisions.size())))];
    w.stamp(shapeOf(Kind::Glider, 0, 0), ox, oy);
    w.stamp(shapeOf(Kind::Glider, 2, 0), ox + 20 + c.dx, oy + 20 + c.dy);
}

// ---------------------------------------------------------------- drawing with the mouse

/// Every cell on the straight line from a to b including both ends (Bresenham), so a fast drag leaves no gaps.
inline Shape lineCells(Cell a, Cell b) {
    Shape out;
    const int dx = std::abs(b.x - a.x);
    const int dy = -std::abs(b.y - a.y);
    const int sx = a.x < b.x ? 1 : -1;
    const int sy = a.y < b.y ? 1 : -1;
    int err = dx + dy;
    Cell p = a;
    for (;;) {
        out.push_back(p);
        if (p == b) break;
        const int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            p.x += sx;
        }
        if (e2 <= dx) {
            err += dx;
            p.y += sy;
        }
    }
    return out;
}

// ---------------------------------------------------------------- time

constexpr std::array<int, 5> kSpeeds{{5, 15, 30, 60, 120}};  // generations per second
constexpr int kMaxStepsPerFrame = 16;
constexpr double kMaxFrameTime = 0.25;

/// Turns frame time into whole generations. The remainder stays in the accumulator, a long frame is clamped and the
/// steps per frame are capped so a slow tab cannot spiral.
struct Pacer {
    double acc = 0.0;

    int due(double dt, double rate) {
        if (rate <= 0.0) return 0;
        acc += std::clamp(dt, 0.0, kMaxFrameTime);
        const double period = 1.0 / rate;
        int n = static_cast<int>(std::floor(acc / period + 1e-9));
        n = std::min(n, kMaxStepsPerFrame);
        acc = std::max(0.0, acc - n * period);
        acc = std::min(acc, period);  // what the cap left over is dropped, not saved up for later
        return n;
    }

    void reset() { acc = 0.0; }
};

// ---------------------------------------------------------------- population over time

/// The population of the last generations, oldest first.
class PopulationHistory {
public:
    explicit PopulationHistory(std::size_t capacity = 600) : capacity_(std::max<std::size_t>(capacity, 1)) {}

    void clear() { v_.clear(); }

    void push(int population) {
        v_.push_back(population);
        while (v_.size() > capacity_) v_.pop_front();
    }

    /// After an edit the newest entry follows the world instead of adding a generation.
    void replaceLast(int population) {
        if (v_.empty()) {
            push(population);
        } else {
            v_.back() = population;
        }
    }

    std::size_t size() const { return v_.size(); }
    int at(std::size_t i) const { return v_[i]; }

    /// Newest value minus the one `gens` generations earlier, nothing while the history is shorter than that.
    std::optional<int> delta(std::size_t gens) const {
        if (v_.size() <= gens) return std::nullopt;
        return v_.back() - v_[v_.size() - 1 - gens];
    }

    /// Largest value among the newest `n` entries, 0 for an empty history.
    int peak(std::size_t n) const {
        int best = 0;
        const std::size_t count = std::min(n, v_.size());
        for (std::size_t i = v_.size() - count; i < v_.size(); ++i) best = std::max(best, v_[i]);
        return best;
    }

private:
    std::size_t capacity_;
    std::deque<int> v_;
};

}  // namespace life
