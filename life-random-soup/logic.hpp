// Conway's Game of Life on an unbounded plane, random soups and the "has it settled" detector. No SDL, covered by test.cpp.
//
// Why unbounded: a soup throws gliders that fly away forever. On a wrapped or walled grid they would hit the edge and change
// the result, and the famous numbers (R-pentomino: 1103 generations, acorn: 5206) only hold on an infinite plane. So the world
// is a list of live cells, and a step counts neighbors only around those cells.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace life {

struct Cell {
    std::int32_t x = 0;
    std::int32_t y = 0;
    bool operator==(const Cell& o) const { return x == o.x && y == o.y; }
    bool operator<(const Cell& o) const { return y != o.y ? y < o.y : x < o.x; }
};

/// Small deterministic generator (SplitMix64). std::mt19937 would do, but the distributions of the standard library differ
/// between platforms, and the demo and the tests must see the same soups everywhere.
class Rng {
public:
    explicit Rng(std::uint64_t seed) : state_(seed) {}
    std::uint64_t next() {
        std::uint64_t z = (state_ += 0x9E3779B97F4A7C15ULL);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
        return z ^ (z >> 31);
    }
    /// Uniform in [0, bound) for bound >= 1: the high 32 bits times the bound, shifted down (bias below 2^-32 per value).
    std::uint32_t below(std::uint32_t bound) { return static_cast<std::uint32_t>(((next() >> 32) * bound) >> 32); }

private:
    std::uint64_t state_;
};

/// The Game of Life with the live cells in a vector. The neighbor counts live in an open-addressing hash table that is reused
/// from step to step, a stamp per slot replaces clearing it.
class Life {
public:
    Life() = default;
    explicit Life(std::vector<Cell> cells) : cells_(std::move(cells)) {}

    const std::vector<Cell>& cells() const { return cells_; }
    int population() const { return static_cast<int>(cells_.size()); }

    void step() {
        if (cells_.empty()) return;
        reserveFor(cells_.size());
        ++epoch_;
        used_.clear();
        for (const Cell& c : cells_) {
            slot(c.x, c.y).alive = 1;
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx != 0 || dy != 0) ++slot(c.x + dx, c.y + dy).count;
                }
            }
        }
        next_.clear();
        for (const std::uint32_t index : used_) {
            const Slot& s = table_[index];
            if (s.count == 3 || (s.count == 2 && s.alive != 0)) next_.push_back(Cell{static_cast<std::int32_t>(s.key >> 32), static_cast<std::int32_t>(s.key & 0xFFFFFFFFU)});
        }
        cells_.swap(next_);
    }

private:
    struct Slot {
        std::uint64_t key = 0;
        std::uint32_t stamp = 0;
        std::uint8_t count = 0;
        std::uint8_t alive = 0;
    };

    static std::uint64_t pack(std::int32_t x, std::int32_t y) { return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32) | static_cast<std::uint32_t>(y); }

    /// A cell has at most 8 live neighbors and is a neighbor of at most 8 cells, so n live cells touch at most 9n slots.
    /// Twice that keeps the probe chains short.
    void reserveFor(std::size_t liveCells) {
        std::size_t want = 64;
        while (want < liveCells * 18) want *= 2;
        if (want > table_.size()) {
            table_.assign(want, Slot{});
            epoch_ = 0;
        }
    }

    /// The slot of a coordinate in the current step, created (zeroed) on first use.
    Slot& slot(std::int32_t x, std::int32_t y) {
        const std::uint64_t key = pack(x, y);
        const std::size_t mask = table_.size() - 1;
        std::size_t i = static_cast<std::size_t>((key * 0x9E3779B97F4A7C15ULL) >> 20) & mask;
        for (;;) {
            Slot& s = table_[i];
            if (s.stamp != epoch_) {
                s = Slot{key, epoch_, 0, 0};
                used_.push_back(static_cast<std::uint32_t>(i));
                return s;
            }
            if (s.key == key) return s;
            i = (i + 1) & mask;
        }
    }

    std::vector<Cell> cells_;
    std::vector<Cell> next_;
    std::vector<Slot> table_;
    std::vector<std::uint32_t> used_;
    std::uint32_t epoch_ = 0;
};

/// A random soup: a size x size square with every cell alive at `densityPercent` (0..100), lower left corner at (0,0).
inline std::vector<Cell> randomSoup(std::uint64_t seed, int size, int densityPercent) {
    Rng rng(seed);
    std::vector<Cell> cells;
    for (int y = 0; y < size; ++y) {
        for (int x = 0; x < size; ++x) {
            if (rng.below(100) < static_cast<std::uint32_t>(std::clamp(densityPercent, 0, 100))) cells.push_back(Cell{x, y});
        }
    }
    return cells;
}

inline std::vector<Cell> rPentomino() { return {{1, 0}, {2, 0}, {0, 1}, {1, 1}, {1, 2}}; }
inline std::vector<Cell> acorn() { return {{1, 0}, {3, 1}, {0, 2}, {1, 2}, {4, 2}, {5, 2}, {6, 2}}; }

/// The middle of a pattern for the camera: the median of the x and of the y coordinates (lower median for an even count). Unlike
/// the bounding box or the mean it stays with the bulk when a few gliders fly far away. An empty pattern is centered at (0,0).
inline Cell medianCenter(const std::vector<Cell>& cells) {
    if (cells.empty()) return Cell{0, 0};
    std::vector<std::int32_t> xs;
    std::vector<std::int32_t> ys;
    xs.reserve(cells.size());
    ys.reserve(cells.size());
    for (const Cell& c : cells) {
        xs.push_back(c.x);
        ys.push_back(c.y);
    }
    const std::size_t mid = (cells.size() - 1) / 2;
    std::nth_element(xs.begin(), xs.begin() + static_cast<std::ptrdiff_t>(mid), xs.end());
    std::nth_element(ys.begin(), ys.begin() + static_cast<std::ptrdiff_t>(mid), ys.end());
    return Cell{xs[mid], ys[mid]};
}

struct Settled {
    int period = 0;  // of the population sequence, which can be shorter than the period of the pattern
    int since = 0;   // first generation of the repeating stretch: for a soup that dies out, the generation it reaches 0
};

/// Decides from the population alone whether a run has settled: the sequence repeats with some period p <= kMaxPeriod for
/// kWindow generations in a row. Gliders keep flying and still count as settled, because a glider is always five cells.
/// This is a heuristic. A glider that will hit something far away much later is not seen, and a pattern whose parts have
/// periods with a large common multiple never settles by this measure.
class Tracker {
public:
    static constexpr int kMaxPeriod = 30;
    static constexpr int kWindow = 120;

    /// Feed the population of generation 0, 1, 2, ... in order.
    void push(int population) {
        ++gen_;
        for (int p = 1; p <= kMaxPeriod; ++p) {
            const bool same = gen_ >= p && ring_[static_cast<std::size_t>((gen_ - p) % kRing)] == population;
            run_[static_cast<std::size_t>(p)] = same ? run_[static_cast<std::size_t>(p)] + 1 : 0;
        }
        ring_[static_cast<std::size_t>(gen_ % kRing)] = population;
    }

    std::optional<Settled> settled() const {
        for (int p = 1; p <= kMaxPeriod; ++p) {
            const int run = run_[static_cast<std::size_t>(p)];
            // run counts the generations t with pop[t] == pop[t-p] up to now, so pop[t] == pop[t+p] holds from t = gen - run - p + 1.
            if (run >= kWindow) return Settled{p, gen_ - run - p + 1};
        }
        return std::nullopt;
    }

    int generation() const { return gen_; }

private:
    static constexpr int kRing = kMaxPeriod + 1;
    std::array<int, kRing> ring_{};
    std::array<int, kMaxPeriod + 1> run_{};
    int gen_ = -1;
};

/// One pattern on its way: the cells, the generation counter and the settle detector. Generation 0 is the start.
class Run {
public:
    Run() = default;
    explicit Run(std::vector<Cell> start) : life_(std::move(start)) { tracker_.push(life_.population()); }

    /// Advances one generation. A settled or capped run stays where it is.
    void step() {
        if (done()) return;
        life_.step();
        ++gen_;
        tracker_.push(life_.population());
        result_ = tracker_.settled();
    }

    bool settled() const { return result_.has_value(); }
    bool capped() const { return !result_ && gen_ >= kMaxGeneration; }
    bool done() const { return settled() || capped(); }
    const std::optional<Settled>& result() const { return result_; }
    int generation() const { return gen_; }
    int population() const { return life_.population(); }
    const std::vector<Cell>& cells() const { return life_.cells(); }

    /// A run that never settles within this many generations is given up on and counted separately.
    static constexpr int kMaxGeneration = 6000;

private:
    Life life_;
    Tracker tracker_;
    std::optional<Settled> result_;
    int gen_ = 0;
};

/// The lifespans of finished soups, counted per generation so the median is exact and the bins can be chosen when drawing.
class Stats {
public:
    static constexpr int kCap = Run::kMaxGeneration;

    void clear() { *this = Stats(); }

    void record(const Run& run) {
        ++total_;
        if (!run.result()) {
            ++capped_;
            return;
        }
        const int life = std::min(run.result()->since, kCap);
        const bool dead = run.population() == 0;
        (dead ? dead_ : ash_)[static_cast<std::size_t>(life)] += 1;
        (dead ? deadTotal_ : ashTotal_) += 1;
        longest_ = std::max(longest_, life);
    }

    long total() const { return total_; }
    long dead() const { return deadTotal_; }
    long ash() const { return ashTotal_; }
    long capped() const { return capped_; }
    int longest() const { return longest_; }

    /// Soups (dead or ash) with a lifespan in [lo, hi).
    long inRange(int lo, int hi, bool dead) const {
        const auto& counts = dead ? dead_ : ash_;
        long n = 0;
        for (int g = std::max(lo, 0); g < std::min(hi, kCap + 1); ++g) n += counts[static_cast<std::size_t>(g)];
        return n;
    }

    long atLeast(int lo) const { return inRange(lo, kCap + 1, true) + inRange(lo, kCap + 1, false); }

    /// Median lifespan of the settled soups, -1 while there are none. Lower median for an even count.
    int median() const {
        const long settledCount = deadTotal_ + ashTotal_;
        if (settledCount == 0) return -1;
        const long target = (settledCount - 1) / 2;  // 0-based rank
        long seen = 0;
        for (int g = 0; g <= kCap; ++g) {
            seen += dead_[static_cast<std::size_t>(g)] + ash_[static_cast<std::size_t>(g)];
            if (seen > target) return g;
        }
        return kCap;
    }

private:
    std::array<long, kCap + 1> dead_{};
    std::array<long, kCap + 1> ash_{};
    long total_ = 0;
    long deadTotal_ = 0;
    long ashTotal_ = 0;
    long capped_ = 0;
    int longest_ = 0;
};

/// Whole generations due after dt seconds at `rate` per second. The fraction stays in `acc`. A stall counts as 0.25 s at most.
inline long stepsDue(double& acc, double rate, double dt) {
    if (!(rate > 0.0) || !(dt > 0.0)) return 0;
    acc += rate * std::min(dt, 0.25);
    const double whole = std::floor(acc);
    acc -= whole;
    return static_cast<long>(whole);
}

inline std::string formatCount(long value) {
    const bool negative = value < 0;
    const std::string digits = std::to_string(negative ? -value : value);
    std::string out;
    for (std::size_t i = 0; i < digits.size(); ++i) {
        if (i > 0 && (digits.size() - i) % 3 == 0) out += ',';
        out += digits[i];
    }
    return negative ? "-" + out : out;
}

}  // namespace life
