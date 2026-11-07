// Life-like cellular automata, no SDL. A rule is two sets of digits 0 to 8: Birth (a dead cell with that many live
// neighbors becomes alive) and Survive (a live cell with that many stays alive). "B3/S23" is Conway's Life. Nine birth bits and
// nine survive bits give 2^18 = 262,144 rules. The world is a torus, so a pattern that leaves one side comes back on the other.
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace lrs {

inline constexpr std::uint32_t kRuleCount = 1U << 18;

struct Rule {
    std::uint16_t birth = 0;    // bit n set: a dead cell with n live neighbors is born
    std::uint16_t survive = 0;  // bit n set: a live cell with n live neighbors survives
    bool operator==(const Rule&) const = default;
};

inline bool nextState(Rule r, bool alive, int neighbors) {
    return ((alive ? r.survive : r.birth) >> neighbors & 1U) != 0;
}

/// "B36/S23" (case does not matter, digit order and repeats do not matter). The whole string must be consumed.
inline std::optional<Rule> parseRule(std::string_view s) {
    std::size_t i = 0;
    auto section = [&](char letter) -> std::optional<std::uint16_t> {
        if (i >= s.size() || (s[i] | 0x20) != letter) return std::nullopt;
        ++i;
        std::uint16_t mask = 0;
        while (i < s.size() && s[i] >= '0' && s[i] <= '9') {
            if (s[i] > '8') return std::nullopt;
            mask = static_cast<std::uint16_t>(mask | (1U << (s[i] - '0')));
            ++i;
        }
        return mask;
    };
    const auto birth = section('b');
    if (!birth || i >= s.size() || s[i] != '/') return std::nullopt;
    ++i;
    const auto survive = section('s');
    if (!survive || i != s.size()) return std::nullopt;
    return Rule{*birth, *survive};
}

/// Canonical form: digits ascending, "B/S" for the empty rule.
inline std::string formatRule(Rule r) {
    std::string out = "B";
    for (int n = 0; n <= 8; ++n) {
        if ((r.birth >> n & 1U) != 0) out += static_cast<char>('0' + n);
    }
    out += "/S";
    for (int n = 0; n <= 8; ++n) {
        if ((r.survive >> n & 1U) != 0) out += static_cast<char>('0' + n);
    }
    return out;
}

/// Position of a rule among all 262,144: birth in the low nine bits, survive above them.
inline std::uint32_t ruleIndex(Rule r) { return static_cast<std::uint32_t>(r.birth & 0x1FFU) | (static_cast<std::uint32_t>(r.survive & 0x1FFU) << 9); }

inline Rule ruleFromIndex(std::uint32_t index) {
    return Rule{static_cast<std::uint16_t>(index & 0x1FFU), static_cast<std::uint16_t>(index >> 9 & 0x1FFU)};
}

/// 262144 -> "262,144". Built by hand because snprintf grouping depends on the locale.
inline std::string groupDigits(std::uint32_t n) {
    std::string digits = std::to_string(n);
    std::string out;
    for (std::size_t i = 0; i < digits.size(); ++i) {
        if (i > 0 && (digits.size() - i) % 3 == 0) out += ',';
        out += digits[i];
    }
    return out;
}

/// Swapping live and dead cells commutes with a step exactly when a dead cell with n live neighbors is born if and only if a live
/// cell with 8-n live neighbors would die (Day & Night, B3678/S34678, is one of these 512 rules).
inline bool selfComplementary(Rule r) {
    for (int n = 0; n <= 8; ++n) {
        for (const bool alive : {false, true}) {
            if (nextState(r, !alive, 8 - n) == !nextState(r, alive, n)) continue;
            return false;
        }
    }
    return true;
}

enum class GrowthNote { None, Blinks, Spreads, Bounded };

/// What the digits alone already prove about a finite pattern on an unbounded plane.
///   B0: every empty cell is born at once, the background flips (and flips back only with S8).
///   B1 without B0: the outermost cell of a side has a dead diagonal neighbor that sees just that one cell, so the bounding box
///      grows by one cell per side and generation.
///   none of B0..B3: a dead cell outside the bounding box touches at most three cells inside it, so nothing is ever born outside.
inline GrowthNote growthNote(Rule r) {
    if ((r.birth & 1U) != 0) return GrowthNote::Blinks;
    if ((r.birth & 2U) != 0) return GrowthNote::Spreads;
    if ((r.birth & 0xEU) == 0) return GrowthNote::Bounded;
    return GrowthNote::None;
}

struct Preset {
    const char* name;
    Rule rule;
    double soup;  // share of live cells in the random start that shows the rule at its best
};

inline constexpr std::array<Preset, 5> kPresets{{
    {"Life", Rule{1U << 3, (1U << 2) | (1U << 3)}, 0.3},
    {"HighLife", Rule{(1U << 3) | (1U << 6), (1U << 2) | (1U << 3)}, 0.3},
    {"Seeds", Rule{1U << 2, 0}, 0.3},
    // Day & Night needs about half the cells alive, or it shrinks to a few dozen blobs.
    {"Day & Night", Rule{(1U << 3) | (1U << 6) | (1U << 7) | (1U << 8), (1U << 3) | (1U << 4) | (1U << 6) | (1U << 7) | (1U << 8)}, 0.5},
    // Diamoeba flips between two outcomes around 47 percent (measured on 156x84: below it mostly dies, from 50 percent on it fills the grid).
    {"Diamoeba", Rule{(1U << 3) | (1U << 5) | (1U << 6) | (1U << 7) | (1U << 8), (1U << 5) | (1U << 6) | (1U << 7) | (1U << 8)}, 0.47},
}};

/// Name of the preset with exactly this rule, empty string otherwise.
inline const char* presetName(Rule r) {
    for (const Preset& p : kPresets) {
        if (p.rule == r) return p.name;
    }
    return "";
}

struct Cell {
    int x = 0;
    int y = 0;
    bool operator==(const Cell&) const = default;
};

struct Box {
    int minX, minY, maxX, maxY;
};

/// Torus of cells, 0 or 1 per byte. Coordinates wrap in both directions, also negative ones.
class Grid {
public:
    Grid() = default;
    Grid(int w, int h) : w_(w), h_(h), cells_(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), 0) {}

    int width() const { return w_; }
    int height() const { return h_; }
    const std::vector<std::uint8_t>& cells() const { return cells_; }

    bool alive(int x, int y) const { return cells_[index(x, y)] != 0; }
    void set(int x, int y, bool v) { cells_[index(x, y)] = v ? 1 : 0; }
    void clear() { std::fill(cells_.begin(), cells_.end(), std::uint8_t{0}); }

    int population() const { return static_cast<int>(std::count(cells_.begin(), cells_.end(), std::uint8_t{1})); }

    /// Raw row access for the stepper and the renderer.
    const std::uint8_t* row(int y) const { return cells_.data() + static_cast<std::size_t>(y) * static_cast<std::size_t>(w_); }
    std::uint8_t* row(int y) { return cells_.data() + static_cast<std::size_t>(y) * static_cast<std::size_t>(w_); }

private:
    std::size_t index(int x, int y) const {
        const int wx = ((x % w_) + w_) % w_;
        const int wy = ((y % h_) + h_) % h_;
        return static_cast<std::size_t>(wy) * static_cast<std::size_t>(w_) + static_cast<std::size_t>(wx);
    }

    int w_ = 0;
    int h_ = 0;
    std::vector<std::uint8_t> cells_;
};

/// One generation of `in` under `r`, written to `out` (resized to match).
inline void step(const Grid& in, Grid& out, Rule r) {
    const int w = in.width();
    const int h = in.height();
    if (out.width() != w || out.height() != h) out = Grid(w, h);
    for (int y = 0; y < h; ++y) {
        const std::uint8_t* up = in.row((y + h - 1) % h);
        const std::uint8_t* mid = in.row(y);
        const std::uint8_t* down = in.row((y + 1) % h);
        std::uint8_t* dst = out.row(y);
        for (int x = 0; x < w; ++x) {
            const int xm = (x + w - 1) % w;
            const int xp = (x + 1) % w;
            const int n = up[xm] + up[x] + up[xp] + mid[xm] + mid[xp] + down[xm] + down[x] + down[xp];
            dst[x] = nextState(r, mid[x] != 0, n) ? 1 : 0;
        }
    }
}

/// Bounding box of the live cells, ignoring the wrap-around. For patterns that stay away from the seam.
inline std::optional<Box> bounds(const Grid& g) {
    std::optional<Box> box;
    for (int y = 0; y < g.height(); ++y) {
        for (int x = 0; x < g.width(); ++x) {
            if (!g.alive(x, y)) continue;
            if (!box) {
                box = Box{x, y, x, y};
            } else {
                box->minX = std::min(box->minX, x);
                box->maxX = std::max(box->maxX, x);
                box->minY = std::min(box->minY, y);
                box->maxY = std::max(box->maxY, y);
            }
        }
    }
    return box;
}

/// SplitMix64: tiny, fast, the same sequence everywhere.
class Rng {
public:
    explicit Rng(std::uint64_t seed) : state_(seed) {}

    std::uint64_t next() {
        std::uint64_t z = (state_ += 0x9E3779B97F4A7C15ULL);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
        return z ^ (z >> 31);
    }

    /// True with probability p (0 never, 1 always).
    bool chance(double p) { return static_cast<double>(next() >> 11) * (1.0 / 9007199254740992.0) < p; }

private:
    std::uint64_t state_;
};

inline void fillSoup(Grid& g, double density, Rng& rng) {
    for (int y = 0; y < g.height(); ++y) {
        for (int x = 0; x < g.width(); ++x) g.set(x, y, rng.chance(density));
    }
}

/// The R-pentomino, five cells that keep Life busy for 1103 generations on an unbounded plane.
inline std::vector<std::pair<int, int>> rPentomino() { return {{1, 0}, {2, 0}, {0, 1}, {1, 1}, {1, 2}}; }

/// The HighLife replicator: a ring of 12 cells in a 5x5 box that turns into two copies of itself after 12 generations.
inline std::vector<std::pair<int, int>> replicatorRing() {
    return {{2, 0}, {3, 0}, {4, 0}, {1, 1}, {4, 1}, {0, 2}, {4, 2}, {0, 3}, {3, 3}, {0, 4}, {1, 4}, {2, 4}};
}

/// FNV-1a over the size and the cells.
inline std::uint64_t hashOf(const Grid& g) {
    std::uint64_t h = 1469598103934665603ULL;
    auto mix = [&h](std::uint64_t v) {
        h ^= v;
        h *= 1099511628211ULL;
    };
    mix(static_cast<std::uint64_t>(g.width()));
    mix(static_cast<std::uint64_t>(g.height()));
    for (const std::uint8_t c : g.cells()) mix(c);
    return h;
}

enum class Kind { Dies, Still, Periodic, Growing, Active };

struct Verdict {
    Kind kind = Kind::Active;
    int period = 0;  // generations until the exact same picture, for Still (1) and Periodic
};

/// Watches a run and names what it sees. Dies: no live cell. Still / Periodic: the exact same grid came back within kHashWindow
/// generations. Growing: at least kGrowthMinCells cells, and twice as many as kGrowthSpan generations ago. Everything else is Active. It only sees what the
/// run did so far, a long transient or a repeat with a longer period reads as Active.
class Tracker {
public:
    static constexpr std::size_t kHistory = 236;     // population samples kept (the graph shows all of them)
    static constexpr std::size_t kHashWindow = 64;   // longest repeat looked for
    static constexpr std::size_t kGrowthSpan = 40;
    static constexpr int kGrowthMinCells = 32;       // below this, doubling is noise (8 cells becoming 16 is not an explosion)

    void reset() {
        pops_.clear();
        hashes_.clear();
    }

    /// Call once for the start state and after every generation.
    void record(const Grid& g) {
        pops_.push_back(g.population());
        if (pops_.size() > kHistory) pops_.erase(pops_.begin());
        hashes_.push_back(hashOf(g));
        if (hashes_.size() > kHashWindow + 1) hashes_.erase(hashes_.begin());
    }

    const std::vector<int>& populations() const { return pops_; }

    Verdict verdict() const {
        if (pops_.empty()) return {};
        if (pops_.back() == 0) return {Kind::Dies, 0};
        const std::size_t n = hashes_.size();
        for (std::size_t k = 1; k < n; ++k) {
            if (hashes_[n - 1 - k] == hashes_[n - 1]) return {k == 1 ? Kind::Still : Kind::Periodic, static_cast<int>(k)};
        }
        if (pops_.size() > kGrowthSpan) {
            const int before = pops_[pops_.size() - 1 - kGrowthSpan];
            if (before >= kGrowthMinCells && pops_.back() >= 2 * before) return {Kind::Growing, 0};
        }
        return {};
    }

private:
    std::vector<int> pops_;
    std::vector<std::uint64_t> hashes_;
};

/// Cells on the straight line from a to b, both included (Bresenham). Used to paint without gaps when the mouse moves fast.
inline std::vector<Cell> lineCells(Cell a, Cell b) {
    std::vector<Cell> out;
    const int dx = std::abs(b.x - a.x);
    const int dy = -std::abs(b.y - a.y);
    const int sx = a.x < b.x ? 1 : -1;
    const int sy = a.y < b.y ? 1 : -1;
    int err = dx + dy;
    int x = a.x;
    int y = a.y;
    for (;;) {
        out.push_back({x, y});
        if (x == b.x && y == b.y) break;
        const int e2 = 2 * err;
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

}  // namespace lrs
