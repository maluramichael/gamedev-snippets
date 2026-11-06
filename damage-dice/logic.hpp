// Dice and damage rolls: the logic of the demo, no SDL, so test.cpp can check it.
//
// Three things matter here.
//
// 1. N dice with S sides added up are not one die with a bigger range. The exact odds come from convolving the die
//    with itself N times (diceDistribution), the rolls come from N calls of the random source (rollDice). A single
//    roll over the same range (rollFlat) has the same mean but a much wider spread, and every outcome is equally likely.
//
// 2. A bonus shifts the whole distribution by a constant. What +1 is worth for "total >= target" is the chance of the
//    number just below the target (bonusWorth): the same 5 points everywhere on a d20, a lot more in the middle of 3d6
//    and almost nothing in its tails.
//
// 3. The random source decides whether the odds are the odds. `value % n` on a source with 2^k values favours the
//    small results unless n divides 2^k. uniformBelow throws away the values below 2^32 mod n instead, which is the
//    rejection method. rollModulo8 is the naive version on a toy 8 bit source, so the effect is big enough to see.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <optional>
#include <string>
#include <vector>

namespace dd {

// ---- random source ---------------------------------------------------------------------------------------------

/// PCG32 (O'Neill), small and good enough for a dice demo.
struct Pcg32 {
    std::uint64_t state = 0;
    std::uint64_t inc = 1;

    void seed(std::uint64_t initState, std::uint64_t sequence) {
        state = 0;
        inc = (sequence << 1U) | 1U;
        next();
        state += initState;
        next();
    }

    std::uint32_t next() {
        const std::uint64_t old = state;
        state = old * 6364136223846793005ULL + inc;
        const auto xorShifted = static_cast<std::uint32_t>(((old >> 18U) ^ old) >> 27U);
        const auto rot = static_cast<std::uint32_t>(old >> 59U);
        return (xorShifted >> rot) | (xorShifted << ((0U - rot) & 31U));
    }
};

/// 2^32 mod n, the number of values at the bottom of the range that a plain `% n` would use once too often.
inline std::uint32_t rejectionThreshold(std::uint32_t n) { return n == 0 ? 0U : (0U - n) % n; }

/// A uniform number in [0, n). Values below the threshold are rejected, so every remainder is equally likely.
template <class Rng>
std::uint32_t uniformBelow(Rng& rng, std::uint32_t n) {
    if (n <= 1) return 0;
    const std::uint32_t threshold = rejectionThreshold(n);
    for (;;) {
        const std::uint32_t r = rng.next();
        if (r >= threshold) return r % n;
    }
}

/// n dice with the given number of sides, added up. Sizes below 1 count as 1.
template <class Rng>
int rollDice(Rng& rng, int n, int sides) {
    n = std::max(n, 1);
    sides = std::max(sides, 1);
    int total = 0;
    for (int i = 0; i < n; ++i) total += 1 + static_cast<int>(uniformBelow(rng, static_cast<std::uint32_t>(sides)));
    return total;
}

/// The naive replacement for dice: one number, all values between lo and hi equally likely.
template <class Rng>
int rollFlat(Rng& rng, int lo, int hi) {
    if (hi <= lo) return lo;
    return lo + static_cast<int>(uniformBelow(rng, static_cast<std::uint32_t>(hi - lo + 1)));
}

/// The classic `rand() % sides` on a source that only has a byte: values 0..255. 256 is not a multiple of 20, so the
/// faces 1..16 of a d20 come up 13 times in 256 and the faces 17..20 only 12 times.
template <class Rng>
int rollModulo8(Rng& rng, int n, int sides) {
    n = std::max(n, 1);
    sides = std::max(sides, 1);
    int total = 0;
    for (int i = 0; i < n; ++i) total += 1 + static_cast<int>((rng.next() & 0xFFU) % static_cast<std::uint32_t>(sides));
    return total;
}

// ---- exact distributions ---------------------------------------------------------------------------------------

/// Probabilities for the values lo, lo + 1, ... lo + p.size() - 1.
struct Distribution {
    int lo = 0;
    std::vector<double> p{1.0};

    int hi() const { return lo + static_cast<int>(p.size()) - 1; }
    double at(int v) const {
        const int i = v - lo;
        return i < 0 || i >= static_cast<int>(p.size()) ? 0.0 : p[static_cast<std::size_t>(i)];
    }
};

/// Exact odds for n dice with `sides` sides: convolve one die into the running total n times.
inline Distribution diceDistribution(int n, int sides) {
    n = std::max(n, 1);
    sides = std::max(sides, 1);
    std::vector<double> cur{1.0};  // zero dice: the sum 0 with certainty
    for (int d = 0; d < n; ++d) {
        std::vector<double> next(cur.size() + static_cast<std::size_t>(sides) - 1, 0.0);
        for (std::size_t k = 0; k < cur.size(); ++k) {
            for (int face = 0; face < sides; ++face) next[k + static_cast<std::size_t>(face)] += cur[k] / sides;
        }
        cur = std::move(next);
    }
    return Distribution{n, std::move(cur)};
}

/// One roll over lo..hi, every value equally likely. hi below lo collapses to the single value lo.
inline Distribution flatDistribution(int lo, int hi) {
    if (hi < lo) hi = lo;
    const auto count = static_cast<std::size_t>(hi - lo + 1);
    return Distribution{lo, std::vector<double>(count, 1.0 / static_cast<double>(count))};
}

/// Odds from a weight table for the values lo, lo + 1, ... Negative weights count as 0. Without any weight the result
/// is the constant lo instead of a division by zero.
inline Distribution weightDistribution(int lo, const std::vector<int>& weights) {
    double total = 0.0;
    for (const int w : weights) total += std::max(w, 0);
    if (total <= 0.0) return Distribution{lo, {1.0}};
    Distribution d{lo, std::vector<double>(weights.size(), 0.0)};
    for (std::size_t i = 0; i < weights.size(); ++i) d.p[i] = std::max(weights[i], 0) / total;
    return d;
}

/// A constant bonus moves every outcome by the same amount.
inline Distribution shifted(const Distribution& d, int by) { return Distribution{d.lo + by, d.p}; }

inline double mean(const Distribution& d) {
    double m = 0.0;
    for (std::size_t i = 0; i < d.p.size(); ++i) m += d.p[i] * static_cast<double>(d.lo + static_cast<int>(i));
    return m;
}

inline double variance(const Distribution& d) {
    const double m = mean(d);
    double v = 0.0;
    for (std::size_t i = 0; i < d.p.size(); ++i) {
        const double dev = static_cast<double>(d.lo + static_cast<int>(i)) - m;
        v += d.p[i] * dev * dev;
    }
    return v;
}

inline double stddev(const Distribution& d) { return std::sqrt(variance(d)); }

/// P(X >= target).
inline double atLeast(const Distribution& d, int target) {
    double s = 0.0;
    for (std::size_t i = 0; i < d.p.size(); ++i) {
        if (d.lo + static_cast<int>(i) >= target) s += d.p[i];
    }
    return std::min(s, 1.0);
}

/// What one more point of bonus adds to P(X >= target): P(X + 1 >= target) - P(X >= target) = P(X = target - 1).
/// `d` already includes the bonus that is there.
inline double bonusWorth(const Distribution& d, int target) { return d.at(target - 1); }

struct WorthRange {
    double least = 0.0;  // smallest worth over all targets where +1 changes anything
    double most = 0.0;
};

inline WorthRange bonusWorthRange(const Distribution& d) {
    WorthRange r{1.0, 0.0};
    for (const double p : d.p) {
        if (p <= 0.0) continue;
        r.least = std::min(r.least, p);
        r.most = std::max(r.most, p);
    }
    if (r.most == 0.0) r.least = 0.0;
    return r;
}

// ---- weight table ----------------------------------------------------------------------------------------------

constexpr int kMaxWeight = 99;

/// A hand made loot table: weights for the values lo, lo + 1, ... The running sums make picking a binary search.
struct Table {
    int lo = 1;
    std::vector<int> weights;
    std::vector<std::uint32_t> cumulative;

    void rebuild() {
        cumulative.resize(weights.size());
        std::uint32_t run = 0;
        for (std::size_t i = 0; i < weights.size(); ++i) {
            run += static_cast<std::uint32_t>(std::max(weights[i], 0));
            cumulative[i] = run;
        }
    }

    std::uint32_t total() const { return cumulative.empty() ? 0U : cumulative.back(); }

    /// Sets one weight (clamped to 0..99). Refuses, and returns false, when the table would be left without any weight.
    bool setWeight(int index, int w) {
        if (index < 0 || index >= static_cast<int>(weights.size())) return false;
        w = std::clamp(w, 0, kMaxWeight);
        const int old = weights[static_cast<std::size_t>(index)];
        if (static_cast<std::int64_t>(total()) - old + w <= 0) return false;
        weights[static_cast<std::size_t>(index)] = w;
        rebuild();
        return true;
    }

    /// A value drawn with the odds of the weights. An all zero table gives lo.
    template <class Rng>
    int pick(Rng& rng) const {
        if (total() == 0) return lo;
        const std::uint32_t r = uniformBelow(rng, total());
        const auto it = std::upper_bound(cumulative.begin(), cumulative.end(), r);
        return lo + static_cast<int>(it - cumulative.begin());
    }

    Distribution distribution() const { return weightDistribution(lo, weights); }
    int hi() const { return lo + static_cast<int>(weights.size()) - 1; }
};

inline Table makeTable(std::vector<int> weights) {
    Table t;
    t.lo = 1;
    t.weights = std::move(weights);
    t.rebuild();
    return t;
}

/// Example weights, not taken from any game: 20 values 1..20.
inline Table flatTable() { return makeTable(std::vector<int>(20, 1)); }

inline Table rampTable() {
    std::vector<int> w(20);
    for (int i = 0; i < 20; ++i) w[static_cast<std::size_t>(i)] = 20 - i;
    return makeTable(std::move(w));
}

/// A graze (1), a normal hit (6..10) and a crit (20), nothing in between.
inline Table critTable() {
    std::vector<int> w(20, 0);
    w[0] = 2;
    for (int v = 6; v <= 10; ++v) w[static_cast<std::size_t>(v - 1)] = 8;
    w[19] = 6;
    return makeTable(std::move(w));
}

// ---- rolled results --------------------------------------------------------------------------------------------

/// Counts of the rolled values lo, lo + 1, ...
class Histogram {
public:
    Histogram(int lo, int bins) : lo_(lo), counts_(static_cast<std::size_t>(std::max(bins, 1)), 0) {}

    void reset() {
        std::fill(counts_.begin(), counts_.end(), 0ULL);
        total_ = 0;
    }

    /// False, and nothing counted, when the value lies outside the range.
    bool add(int v) {
        const int i = v - lo_;
        if (i < 0 || i >= static_cast<int>(counts_.size())) return false;
        ++counts_[static_cast<std::size_t>(i)];
        ++total_;
        return true;
    }

    std::uint64_t total() const { return total_; }
    std::uint64_t count(int v) const {
        const int i = v - lo_;
        return i < 0 || i >= static_cast<int>(counts_.size()) ? 0ULL : counts_[static_cast<std::size_t>(i)];
    }
    double prob(int v) const { return total_ == 0 ? 0.0 : static_cast<double>(count(v)) / static_cast<double>(total_); }

    double mean() const {
        if (total_ == 0) return 0.0;
        double s = 0.0;
        for (std::size_t i = 0; i < counts_.size(); ++i) s += static_cast<double>(counts_[i]) * static_cast<double>(lo_ + static_cast<int>(i));
        return s / static_cast<double>(total_);
    }

    double variance() const {
        if (total_ == 0) return 0.0;
        const double m = mean();
        double s = 0.0;
        for (std::size_t i = 0; i < counts_.size(); ++i) {
            const double dev = static_cast<double>(lo_ + static_cast<int>(i)) - m;
            s += static_cast<double>(counts_[i]) * dev * dev;
        }
        return s / static_cast<double>(total_);
    }

    double fractionAtLeast(int target) const {
        if (total_ == 0) return 0.0;
        std::uint64_t n = 0;
        for (std::size_t i = 0; i < counts_.size(); ++i) {
            if (lo_ + static_cast<int>(i) >= target) n += counts_[i];
        }
        return static_cast<double>(n) / static_cast<double>(total_);
    }

private:
    int lo_;
    std::vector<std::uint64_t> counts_;
    std::uint64_t total_ = 0;
};

// ---- text and plot geometry ------------------------------------------------------------------------------------

/// 1234567 -> "1,234,567", built by hand so the output does not depend on the locale.
inline std::string formatCount(std::uint64_t n) {
    std::string digits = std::to_string(n);
    std::string out;
    for (std::size_t i = 0; i < digits.size(); ++i) {
        if (i > 0 && (digits.size() - i) % 3 == 0) out += ',';
        out += digits[i];
    }
    return out;
}

/// "3d6", "3d6 + 2", "1d20 - 3".
inline std::string expression(int n, int sides, int bonus) {
    std::string s = std::to_string(n) + "d" + std::to_string(sides);
    if (bonus > 0) s += " + " + std::to_string(bonus);
    if (bonus < 0) s += " - " + std::to_string(-bonus);
    return s;
}

/// The visible range of values: the dice range plus some room on both sides, so a bonus can slide the curve around
/// without the axis jumping.
struct Window {
    int lo = 0;
    int hi = 0;
    int bins() const { return hi - lo + 1; }
};

inline Window windowFor(int lo, int hi, int pad) { return Window{lo - pad, hi + pad}; }

/// One bar per integer value, the first at x0, together `width` wide.
struct Axis {
    int lo = 0;
    int bins = 1;
    double x0 = 0.0;
    double width = 1.0;

    double binWidth() const { return width / static_cast<double>(bins); }
    double left(int v) const { return x0 + static_cast<double>(v - lo) * binWidth(); }
    double center(int v) const { return left(v) + binWidth() / 2.0; }
    std::optional<int> valueAt(double x) const {
        if (x < x0 || x >= x0 + width) return std::nullopt;
        return lo + static_cast<int>(std::floor((x - x0) / binWidth()));
    }
};

/// The smallest label step (0.5, 1, 2, 5, 10, ... but never below minStep) that keeps labels at least minGapPx apart.
/// When none does, the biggest one.
inline double niceStep(double unitPx, double minGapPx, double minStep = 1.0) {
    const double steps[] = {0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 25.0, 50.0, 100.0};
    double best = steps[8];
    for (const double s : steps) {
        if (s < minStep) continue;
        best = s;
        if (s * unitPx >= minGapPx) break;
    }
    return best;
}

}  // namespace dd
