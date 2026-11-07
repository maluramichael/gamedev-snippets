// Cyclic cellular automaton: the logic of the demo, no SDL, so test.cpp can check it.
//
// Every cell has one of n colors that sit on a circle: after color n-1 comes color 0 again. A cell looks at its neighbors and
// counts how many of them already have the NEXT color after its own. If there are at least `threshold` of them, the cell is
// eaten and takes that next color. Nothing else ever happens: a cell is never eaten by its own color or by the color behind it.
//
// THE GRID IS A TORUS. The cell left of the first column is the last one, same for rows. Without that the border would need a
// rule of its own, and the count of defects below would not balance.
//
// DEMONS are the places where the circle of colors winds around a point. Walk once around a 2x2 block of cells and add up the
// color steps, each one taken the short way around the circle (-n/2 .. +n/2). The sum is a multiple of n, and when it is not 0 the
// colors go around the block once or more, like the hours on a clock face, and the waves around it turn like a spiral. The sign
// tells the two turning directions apart, the demo counts any nonzero winding once. Every edge is walked once in each direction
// by the two blocks that share it, so the signed total over a whole torus is exactly 0: spirals come in pairs of opposite handedness.
// LIMIT: a step of exactly n/2 has no short way around. It is broken by the order of the colors, which keeps the total at 0, but a
// sharp front with such steps then counts as a demon too (it happens mostly with the Moore neighborhood and large n).
//
// Two typical mistakes, both switchable in the demo so they can be compared with the right thing:
//   * Updating in place. A step should read the old grid and write a new one (double buffer). Sweeping through the grid and
//     changing cells on the way lets later cells see the new colors of earlier ones, so a color wave can cross the grid in a step.
//   * Forgetting the wrap of the colors (`ring = false`): "the neighbor is exactly one higher" without the modulo. Color n-1 then
//     has no successor, the circle is a line, nothing can turn, and the grid freezes into blocks.
#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace cca {

inline constexpr int kMinColors = 3;
inline constexpr int kMaxColors = 16;

enum class Hood { VonNeumann, Moore };  // 4 neighbors (up, down, left, right) or 8 (with the diagonals)

struct Rule {
    int colors = 8;
    int threshold = 3;
    Hood hood = Hood::Moore;
    bool ring = true;  // false: color n-1 is never eaten (the mistake of forgetting the wrap)
};

/// A rectangle of cells, row by row. Width or height of 0 or less gives an empty grid.
struct Grid {
    int w = 0;
    int h = 0;
    std::vector<std::uint8_t> cells;

    Grid() = default;
    Grid(int width, int height) {
        if (width > 0 && height > 0) {
            w = width;
            h = height;
            cells.assign(static_cast<std::size_t>(width) * static_cast<std::size_t>(height), 0);
        }
    }

    bool empty() const { return cells.empty(); }
    std::size_t index(int x, int y) const { return static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x); }
    /// x and y must be inside the grid.
    std::uint8_t& at(int x, int y) { return cells[index(x, y)]; }
    std::uint8_t at(int x, int y) const { return cells[index(x, y)]; }
    /// Any x and y, wrapped around the torus. The grid must not be empty.
    std::uint8_t wrapped(int x, int y) const {
        if (x < 0 || x >= w) x = ((x % w) + w) % w;  // the common case, inside the grid, skips the modulo
        if (y < 0 || y >= h) y = ((y % h) + h) % h;
        return cells[index(x, y)];
    }

    bool operator==(const Grid& other) const { return w == other.w && h == other.h && cells == other.cells; }
};

/// The color that eats `color`: one further around the circle.
inline int successor(int color, int colors) { return color + 1 >= colors ? 0 : color + 1; }

/// The color that eats `color` under this rule, or -1 when nothing does (last color of a line).
inline int nextColor(int color, const Rule& rule) { return !rule.ring && color + 1 >= rule.colors ? -1 : successor(color, rule.colors); }

inline int neighborsOf(Hood hood) { return hood == Hood::Moore ? 8 : 4; }

/// A threshold below 1 would eat every cell for free, one above the neighbor count would never eat anything.
inline int clampThreshold(int threshold, Hood hood) { return std::clamp(threshold, 1, neighborsOf(hood)); }

inline int clampColors(int colors) { return std::clamp(colors, kMinColors, kMaxColors); }

/// How many neighbors of the cell have the color that eats it.
inline int eatenBy(const Grid& g, int x, int y, const Rule& rule) {
    static constexpr int kOffsets[8][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}, {-1, -1}, {1, -1}, {1, 1}, {-1, 1}};  // the first four are von Neumann
    const int want = nextColor(g.at(x, y), rule);
    if (want < 0) return 0;
    const int count = neighborsOf(rule.hood);
    int hits = 0;
    for (int i = 0; i < count; ++i) {
        if (g.wrapped(x + kOffsets[i][0], y + kOffsets[i][1]) == want) ++hits;
    }
    return hits;
}

/// Whether the cell changes: it has an eater at all and enough neighbors with that color.
inline bool isEaten(const Grid& g, int x, int y, const Rule& rule) { return nextColor(g.at(x, y), rule) >= 0 && eatenBy(g, x, y, rule) >= rule.threshold; }

/// One step from `in` into `out` (double buffer: every cell reads the old grid). `out` is resized to match `in`.
inline void stepInto(const Grid& in, Grid& out, const Rule& rule) {
    if (out.w != in.w || out.h != in.h) out = Grid(in.w, in.h);
    for (int y = 0; y < in.h; ++y) {
        for (int x = 0; x < in.w; ++x) {
            const int color = in.at(x, y);
            out.at(x, y) = static_cast<std::uint8_t>(isEaten(in, x, y, rule) ? nextColor(color, rule) : color);
        }
    }
}

inline Grid step(const Grid& in, const Rule& rule) {
    Grid out;
    stepInto(in, out, rule);
    return out;
}

/// The naive update: change cells while sweeping, so later cells already see the new colors of earlier ones.
inline void stepInPlace(Grid& g, const Rule& rule) {
    for (int y = 0; y < g.h; ++y) {
        for (int x = 0; x < g.w; ++x) {
            if (isEaten(g, x, y, rule)) g.at(x, y) = static_cast<std::uint8_t>(nextColor(g.at(x, y), rule));
        }
    }
}

/// splitmix64: tiny, and the same numbers on every platform (std::uniform_int_distribution is not).
inline std::uint64_t splitmix(std::uint64_t& state) {
    state += 0x9E3779B97F4A7C15ULL;
    std::uint64_t z = state;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
}

/// Every cell gets one of the colors with equal chance.
inline Grid randomGrid(int w, int h, int colors, std::uint64_t seed) {
    Grid g(w, h);
    const auto n = static_cast<std::uint64_t>(std::max(colors, 1));
    std::uint64_t state = seed;
    for (std::uint8_t& cell : g.cells) cell = static_cast<std::uint8_t>((splitmix(state) >> 33) % n);
    return g;
}

/// The step from color `from` to color `to` the short way around the circle, in -n/2 .. +n/2. A step of exactly n/2 has no short
/// way, it goes up when `from` is the smaller color and down otherwise, so that the step back is always the exact negative.
inline int wrapDiff(int from, int to, int colors) {
    if (colors <= 0) return 0;
    const int d = (((to - from) % colors) + colors) % colors;
    if (2 * d > colors) return d - colors;
    if (2 * d == colors) return from < to ? d : -d;
    return d;
}

/// Winding of the 2x2 block whose upper left cell is (x,y): the color steps around it (right, down, left, up) added up and divided
/// by n. 0 means smooth, +1 and -1 are a demon turning one way or the other.
inline int winding(const Grid& g, int x, int y, int colors) {
    if (g.empty() || colors <= 0) return 0;
    const int a = g.wrapped(x, y);
    const int b = g.wrapped(x + 1, y);
    const int c = g.wrapped(x + 1, y + 1);
    const int d = g.wrapped(x, y + 1);
    return (wrapDiff(a, b, colors) + wrapDiff(b, c, colors) + wrapDiff(c, d, colors) + wrapDiff(d, a, colors)) / colors;
}

struct Demons {
    int positive = 0;
    int negative = 0;
    int total() const { return positive + negative; }
};

/// All 2x2 blocks of the torus with a nonzero winding, counted by sign.
inline Demons countDemons(const Grid& g, int colors) {
    Demons out;
    for (int y = 0; y < g.h; ++y) {
        for (int x = 0; x < g.w; ++x) {
            const int w = winding(g, x, y, colors);
            if (w > 0) ++out.positive;
            if (w < 0) ++out.negative;
        }
    }
    return out;
}

struct DemonSite {
    int x = 0;  // upper left cell of the 2x2 block
    int y = 0;
    int sign = 0;  // +1 or -1, the sign of the winding
};

/// The same blocks as countDemons, with their positions. `out` is cleared first, so a vector can be reused every frame.
inline void collectDemons(const Grid& g, int colors, std::vector<DemonSite>& out) {
    out.clear();
    for (int y = 0; y < g.h; ++y) {
        for (int x = 0; x < g.w; ++x) {
            const int w = winding(g, x, y, colors);
            if (w != 0) out.push_back({x, y, w > 0 ? 1 : -1});
        }
    }
}

/// How many steps to take for the time that passed. `acc` carries the fraction between frames, dt is clamped to 0.25 s so a pause
/// (hidden tab) is not paid back, and `maxSteps` bounds the work of one frame, the excess is dropped.
inline int stepsDue(double& acc, double rate, double dt, int maxSteps) {
    if (rate <= 0.0) return 0;
    acc += std::clamp(dt, 0.0, 0.25) * rate;
    const int n = std::min(static_cast<int>(acc), std::max(maxSteps, 0));
    acc = std::min(acc - n, 1.0);
    return n;
}

/// A curve over an unbounded number of pushes in a bounded number of points: records every `stride`-th value and, when the
/// capacity is reached, keeps every second point and doubles the stride. The points are always the pushes at multiples of the stride.
class Series {
public:
    explicit Series(std::size_t capacity) : cap_(std::max<std::size_t>(capacity, 2)) {}

    void push(int value) {
        if (pushes_ % stride_ == 0) {
            values_.push_back(value);
            if (values_.size() >= cap_) {
                for (std::size_t i = 0; i * 2 < values_.size(); ++i) values_[i] = values_[i * 2];
                values_.resize((values_.size() + 1) / 2);
                stride_ *= 2;
            }
        }
        ++pushes_;
    }

    void clear() {
        values_.clear();
        pushes_ = 0;
        stride_ = 1;
    }

    const std::vector<int>& values() const { return values_; }
    /// Pushes between two points.
    std::size_t stride() const { return stride_; }

private:
    std::size_t cap_;
    std::vector<int> values_;
    std::size_t pushes_ = 0;
    std::size_t stride_ = 1;
};

/// Remembers the last grids and reports when the current one equals an earlier one: that is the period of the whole pattern.
class PeriodDetector {
public:
    explicit PeriodDetector(int maxPeriod) : max_(std::max(maxPeriod, 1)), ring_(static_cast<std::size_t>(max_) + 1) {}

    /// Adds the current grid. Returns the smallest p (1 .. maxPeriod) such that it equals the grid pushed p calls ago, 0 if none does.
    int push(const Grid& g) {
        int found = 0;
        for (int p = 1; p <= std::min(count_, max_) && found == 0; ++p) {
            if (ring_[slot(head_ - p)] == g) found = p;
        }
        ring_[slot(head_)] = g;
        head_ = (head_ + 1) % (max_ + 1);
        count_ = std::min(count_ + 1, max_ + 1);
        return found;
    }

    /// Forgets everything pushed so far.
    void reset() {
        head_ = 0;
        count_ = 0;
    }

private:
    std::size_t slot(int i) const { return static_cast<std::size_t>(((i % (max_ + 1)) + (max_ + 1)) % (max_ + 1)); }

    int max_;
    std::vector<Grid> ring_;
    int head_ = 0;
    int count_ = 0;
};

}  // namespace cca
