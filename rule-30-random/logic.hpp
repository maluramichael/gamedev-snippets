// Rule 30 as a bit source, plus the statistics the demo shows. No SDL, covered by test.cpp.
//
// One row of the automaton is a plane of cells that starts as a single black cell. The next row is, per cell,
// left xor (center or right). BitRow does that for 64 cells per instruction, ByteRow one cell at a time, both give
// the same rows. Only the cone that the start cell can have reached is computed, outside of it every cell is white.
#pragma once

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace r30 {

/// Wolfram's rule number 30: the new cell is p xor (q or r) for left p, center q and right r.
inline constexpr bool rule30(bool p, bool q, bool r) { return p != (q || r); }

/// Row of a single-cell start, 64 cells per word. Cell x is bit (center + x), the start cell sits in the middle.
class BitRow {
public:
    /// Room for maxSteps rows after the first, the cone of the last one just fits.
    explicit BitRow(std::size_t maxSteps)
        : maxSteps_(maxSteps),
          center_(maxSteps),
          cur_((2 * maxSteps + 1 + 63) / 64 + 1, 0),
          nxt_(cur_.size(), 0) {
        setBit(cur_, center_);
    }

    std::size_t step() const { return step_; }
    std::size_t maxSteps() const { return maxSteps_; }

    /// Computes the next row. Returns false (and changes nothing) when the plane has no room left.
    bool advance() {
        if (step_ >= maxSteps_) return false;
        // The row has black cells in [center - t, center + t], the next one in one cell more on each side.
        const std::size_t lo = (center_ - step_ - 1) / 64;
        const std::size_t hi = std::min((center_ + step_ + 1) / 64, cur_.size() - 1);
        for (std::size_t k = lo; k <= hi; ++k) {
            const std::uint64_t prev = k > 0 ? cur_[k - 1] : 0;
            const std::uint64_t next = k + 1 < cur_.size() ? cur_[k + 1] : 0;
            const std::uint64_t q = cur_[k];
            const std::uint64_t p = (q << 1U) | (prev >> 63U);  // each cell sees its left neighbor
            const std::uint64_t r = (q >> 1U) | (next << 63U);  // and its right neighbor
            nxt_[k] = p ^ (q | r);
        }
        cur_.swap(nxt_);
        ++step_;
        return true;
    }

    /// Cell x of the current row, x = 0 is the start cell. Everything outside the plane is white.
    bool cell(long x) const {
        const long index = static_cast<long>(center_) + x;
        if (index < 0 || static_cast<std::size_t>(index) >= cur_.size() * 64) return false;
        const auto i = static_cast<std::size_t>(index);
        return ((cur_[i / 64] >> (i % 64)) & 1U) != 0;
    }

    std::size_t black() const {
        std::size_t n = 0;
        for (const std::uint64_t w : cur_) n += static_cast<std::size_t>(std::popcount(w));
        return n;
    }

private:
    static void setBit(std::vector<std::uint64_t>& v, std::size_t i) { v[i / 64] |= std::uint64_t{1} << (i % 64); }

    std::size_t maxSteps_;
    std::size_t center_;
    std::size_t step_ = 0;
    std::vector<std::uint64_t> cur_;
    std::vector<std::uint64_t> nxt_;
};

/// The same automaton, one byte per cell: the obvious way to write it.
class ByteRow {
public:
    explicit ByteRow(std::size_t maxSteps) : maxSteps_(maxSteps), center_(maxSteps), cur_(2 * maxSteps + 1, 0), nxt_(cur_.size(), 0) {
        cur_[center_] = 1;
    }

    std::size_t step() const { return step_; }
    std::size_t maxSteps() const { return maxSteps_; }

    bool advance() {
        if (step_ >= maxSteps_) return false;
        const std::size_t lo = center_ - step_ - 1;
        const std::size_t hi = std::min(center_ + step_ + 1, cur_.size() - 1);
        for (std::size_t i = lo; i <= hi; ++i) {
            const bool p = i > 0 && cur_[i - 1] != 0;
            const bool q = cur_[i] != 0;
            const bool r = i + 1 < cur_.size() && cur_[i + 1] != 0;
            nxt_[i] = rule30(p, q, r) ? 1 : 0;
        }
        cur_.swap(nxt_);
        ++step_;
        return true;
    }

    bool cell(long x) const {
        const long index = static_cast<long>(center_) + x;
        if (index < 0 || static_cast<std::size_t>(index) >= cur_.size()) return false;
        return cur_[static_cast<std::size_t>(index)] != 0;
    }

    std::size_t black() const {
        std::size_t n = 0;
        for (const std::uint8_t c : cur_) n += c;
        return n;
    }

private:
    std::size_t maxSteps_;
    std::size_t center_;
    std::size_t step_ = 0;
    std::vector<std::uint8_t> cur_;
    std::vector<std::uint8_t> nxt_;
};

/// The first rows of the textbook picture as a window of cells, rows x (2 * halfWidth + 1).
struct Triangle {
    std::size_t rows = 0;
    long halfWidth = 0;
    std::vector<std::uint8_t> cells;

    bool at(std::size_t t, long x) const {
        if (t >= rows || x < -halfWidth || x > halfWidth) return false;
        return cells[t * static_cast<std::size_t>(2 * halfWidth + 1) + static_cast<std::size_t>(x + halfWidth)] != 0;
    }
};

inline Triangle makeTriangle(std::size_t rows, long halfWidth) {
    Triangle tri;
    tri.rows = rows;
    tri.halfWidth = halfWidth;
    tri.cells.assign(rows * static_cast<std::size_t>(2 * halfWidth + 1), 0);
    BitRow row(rows);
    for (std::size_t t = 0; t < rows; ++t) {
        for (long x = -halfWidth; x <= halfWidth; ++x) {
            tri.cells[t * static_cast<std::size_t>(2 * halfWidth + 1) + static_cast<std::size_t>(x + halfWidth)] = row.cell(x) ? 1 : 0;
        }
        row.advance();
    }
    return tri;
}

/// The classic 32 bit LCG (Numerical Recipes constants), the same generator as in the lcg-lattice demo.
struct Lcg {
    std::uint32_t state = 1;
    std::uint32_t next() {
        state = state * 1664525U + 1013904223U;
        return state;
    }
};

enum class Source { Center, LeftSide, LcgLow, LcgHigh };
enum class Engine { PerCell, BitParallel };

/// The left-side source reads the diagonal that runs this many cells in from the left edge of the triangle.
inline constexpr std::size_t kLeftOffset = 8;

/// A stream of bits, one per call. The rule 30 sources read one cell of the current row and then advance it.
class Stream {
public:
    /// bits is how many bits will be asked for at most.
    Stream(Source source, Engine engine, std::size_t bits) : source_(source), engine_(engine) {
        if (source == Source::Center || source == Source::LeftSide) {
            if (engine == Engine::BitParallel) {
                fast_ = BitRow(bits);
            } else {
                slow_ = ByteRow(bits);
            }
        }
    }

    bool next() {
        switch (source_) {
            case Source::Center: return step(0);
            case Source::LeftSide: return step(static_cast<long>(kLeftOffset) - static_cast<long>(rows()));
            case Source::LcgLow: return (lcg_.next() & 1U) != 0;
            case Source::LcgHigh: return (lcg_.next() >> 31U) != 0;
        }
        return false;
    }

private:
    std::size_t rows() const { return engine_ == Engine::BitParallel ? fast_.step() : slow_.step(); }

    bool step(long x) {
        bool b = false;
        if (engine_ == Engine::BitParallel) {
            b = fast_.cell(x);
            fast_.advance();
        } else {
            b = slow_.cell(x);
            slow_.advance();
        }
        return b;
    }

    Source source_;
    Engine engine_;
    BitRow fast_{0};
    ByteRow slow_{0};
    Lcg lcg_;
};

/// Running count of ones and the longest run of equal bits.
struct BitStats {
    std::size_t n = 0;
    std::size_t ones = 0;
    std::size_t longest = 0;
    std::size_t run = 0;
    bool last = false;

    void push(bool b) {
        run = (n > 0 && b == last) ? run + 1 : 1;
        longest = std::max(longest, run);
        last = b;
        ones += b ? 1 : 0;
        ++n;
    }
};

inline constexpr int kMaxPeriod = 64;
inline constexpr std::size_t kWarmup = 32;  // the first bits may be a start-up transient
/// From this many bits on, "no period found" really means none up to kMaxPeriod: even period 64 had enough comparisons.
inline constexpr std::size_t kPeriodSure = kWarmup + kMaxPeriod + 3 * kMaxPeriod;

/// Finds the smallest period p <= kMaxPeriod with bit[i] == bit[i - p] for every i from the warm-up on.
/// Needs at least max(32, 3p) comparisons after the warm-up before it believes one: with fewer, a random stream would
/// show a period by luck now and then. Short streams therefore say "no period yet".
class PeriodFinder {
public:
    PeriodFinder() { lastMismatch_.fill(-1); }

    void push(bool b) {
        for (int p = 1; p <= kMaxPeriod; ++p) {
            if (n_ >= static_cast<std::size_t>(p) && (((recent_ >> (p - 1)) & 1U) != 0) != b) {
                lastMismatch_[static_cast<std::size_t>(p)] = static_cast<long>(n_);
            }
        }
        recent_ = (recent_ << 1U) | (b ? 1U : 0U);
        ++n_;
    }

    std::size_t size() const { return n_; }

    /// 0 when none was found.
    int period() const {
        for (int p = 1; p <= kMaxPeriod; ++p) {
            const auto up = static_cast<std::size_t>(p);
            const auto need = kWarmup + up + std::max<std::size_t>(32, 3 * up);
            if (n_ >= need && lastMismatch_[static_cast<std::size_t>(p)] < static_cast<long>(kWarmup) + p) return p;
        }
        return 0;
    }

private:
    std::size_t n_ = 0;
    std::uint64_t recent_ = 0;  // bit 0 is the newest bit
    std::array<long, kMaxPeriod + 1> lastMismatch_{};
};

/// 1234567 -> "1,234,567". Built by hand so the output does not depend on the locale.
inline std::string formatThousands(unsigned long long v) {
    std::string digits = std::to_string(v);
    std::string out;
    const std::size_t len = digits.size();
    for (std::size_t i = 0; i < len; ++i) {
        out += digits[i];
        const std::size_t left = len - 1 - i;
        if (left > 0 && left % 3 == 0) out += ',';
    }
    return out;
}

/// ones / n as a percentage with one decimal ("50.1"), rounded half up. 0.0 when n is 0.
inline std::string formatPercent(std::size_t ones, std::size_t n) {
    if (n == 0) return "0.0";
    const std::size_t permille = (ones * 1000 + n / 2) / n;
    return std::to_string(permille / 10) + "." + std::to_string(permille % 10);
}

/// The longest run of equal bits in n fair coin flips is typically close to log2(n). 0 for fewer than two bits.
inline int expectedLongestRun(std::size_t n) {
    if (n < 2) return 0;
    return static_cast<int>(std::lround(std::log2(static_cast<double>(n))));
}

}  // namespace r30
