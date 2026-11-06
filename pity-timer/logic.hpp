// Pity timer: the logic of the demo, no SDL, so test.cpp can check it.
//
// A drop with a 1 % rate is a coin that comes up heads with probability 0.01 on every try. The tries do not remember
// each other, so "1 %" says nothing about 100 tries: the chance to be empty-handed after 100 tries is 0.99^100, about
// 37 %. The wait for the first drop is geometrically distributed, with a long tail.
//
// Designers soften that tail by making the chance depend on the number of misses so far. Three models, all written as
// "chance of the next try, given that this is try number n since the last drop":
//
//   Random  chance(n) = p
//   Prd     chance(n) = min(1, C * n), with C picked so that the long-run rate is exactly p (the constant is found by
//           bisection: a bigger C means a bigger overall rate). The first tries are *worse* than p, later ones better.
//   Pity    chance(n) = p up to the soft start, then a straight ramp up to 1 at the hard pity (the guarantee).
//           The soft start sits at 3/4 of the hard pity, a choice of this demo, not of any game. The average rate
//           ends up above p, which is why a designer who wants p overall has to lower the base rate.
//
// From the chance function the exact wait distribution follows without any dice:
//   P(first drop on try n) = chance(n) * P(no drop in the first n-1 tries)
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace pity {

enum class Mode { Random, Prd, Pity };

/// Longest wait that is simulated or tabulated. A rate of zero never drops, this is where the loop stops.
constexpr int kMaxAttempts = 20000;

/// Pity ramps up from here: three quarters of the way to the guarantee, at least try 1.
inline int softStart(int hardPity) { return std::max(1, hardPity * 3 / 4); }

/// Overall drop rate of a PRD whose chance on try n is min(1, c * n): one divided by the expected number of tries.
/// The expectation is the sum, over every try, of the probability to get that far.
inline double prdOverallChance(double c) {
    if (c <= 0.0) return 0.0;
    if (c >= 1.0) return 1.0;
    double expected = 0.0;
    double reach = 1.0;  // probability that try n happens at all
    for (int n = 1;; ++n) {
        expected += reach;
        const double chance = c * n;
        if (chance >= 1.0 - 1e-12) break;  // the guarantee: nobody gets past this try
        reach *= 1.0 - chance;
    }
    return 1.0 / expected;
}

/// The constant c with prdOverallChance(c) == p, by bisection. c is always below p: the first tries have to be worse
/// than p to leave room for the later ones that are better.
inline double prdConstant(double p) {
    if (p <= 0.0) return 0.0;
    if (p >= 1.0) return 1.0;
    double lo = 0.0;
    double hi = p;
    for (int i = 0; i < 100; ++i) {
        const double mid = 0.5 * (lo + hi);
        if (prdOverallChance(mid) < p) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    return 0.5 * (lo + hi);
}

struct Model {
    Mode mode = Mode::Random;
    double rate = 0.0;  // the nominal drop rate p, 0..1
    int hardPity = 1;   // Pity only: the try with the guarantee
    double prdC = 0.0;  // Prd only

    /// Chance that try number `attempt` (1 = right after a drop) drops.
    double chance(int attempt) const {
        const int n = std::max(1, attempt);
        switch (mode) {
            case Mode::Random: return rate;
            case Mode::Prd: {
                const double c = prdC * n;
                return c >= 1.0 - 1e-12 ? 1.0 : c;
            }
            case Mode::Pity: {
                if (n >= hardPity) return 1.0;
                const int soft = softStart(hardPity);
                if (n < soft) return rate;
                const double ramp = static_cast<double>(n - soft + 1) / static_cast<double>(hardPity - soft + 1);
                return std::min(1.0, rate + (1.0 - rate) * ramp);
            }
        }
        return rate;
    }
};

inline Model makeModel(Mode mode, double rate, int hardPity) {
    Model m;
    m.mode = mode;
    m.rate = std::clamp(rate, 0.0, 1.0);
    m.hardPity = std::max(1, hardPity);
    m.prdC = mode == Mode::Prd ? prdConstant(m.rate) : 0.0;
    return m;
}

/// The wait distribution of a model, computed exactly. Index n is "n tries".
struct Exact {
    std::vector<double> pmf;       // pmf[n]: probability that the first drop is try n (pmf[0] = 0)
    std::vector<double> survival;  // survival[n]: probability that the first n tries all miss (survival[0] = 1)
    double mean = 0.0;             // expected number of tries until the drop

    double pmfAt(int n) const { return n >= 0 && static_cast<std::size_t>(n) < pmf.size() ? pmf[static_cast<std::size_t>(n)] : 0.0; }

    /// Probability to have no drop after n tries.
    double stillEmpty(int n) const {
        if (n <= 0) return 1.0;
        return static_cast<std::size_t>(n) < survival.size() ? survival[static_cast<std::size_t>(n)] : survival.back();
    }
};

inline Exact exact(const Model& m, int maxAttempts = kMaxAttempts) {
    Exact e;
    e.pmf.push_back(0.0);
    e.survival.push_back(1.0);
    double reach = 1.0;
    for (int n = 1; n <= maxAttempts; ++n) {
        const double c = m.chance(n);
        const double p = reach * c;
        reach *= 1.0 - c;
        e.pmf.push_back(p);
        e.survival.push_back(reach);
        e.mean += n * p;
        if (reach < 1e-13) break;  // the rest of the tail does not move any number we show
    }
    return e;
}

/// Probability per histogram bar: bar k holds the waits k*binWidth+1 .. (k+1)*binWidth.
inline std::vector<double> binProbabilities(const Exact& e, int binWidth, int bins) {
    std::vector<double> out(static_cast<std::size_t>(std::max(0, bins)), 0.0);
    for (int k = 0; k < bins; ++k) {
        for (int n = k * binWidth + 1; n <= (k + 1) * binWidth; ++n) out[static_cast<std::size_t>(k)] += e.pmfAt(n);
    }
    return out;
}

/// Histogram axis: show five times the nominal wait (1 / p), in at most 50 bars. Beyond that is "overflow".
struct Layout {
    int binWidth;
    int bins;
};

inline Layout layoutFor(double rate) {
    const double p = std::max(rate, 1e-4);  // a rate of zero must not divide by zero
    const int range = std::max(1, static_cast<int>(std::ceil(5.0 / p - 1e-9)));
    const int width = std::max(1, (range + 49) / 50);
    return {width, (range + width - 1) / width};
}

/// "1 / p" tries, the point where people expect the drop: 100 tries at 1 %.
inline int markAttempts(double rate) { return std::max(1, static_cast<int>(std::llround(1.0 / std::max(rate, 1e-4)))); }

/// SplitMix64: tiny, fixed and the same in every build, so a seed always gives the same players.
class Rng {
public:
    explicit Rng(std::uint64_t seed) : state_(seed) {}
    std::uint64_t next() {
        state_ += 0x9E3779B97F4A7C15ULL;
        std::uint64_t z = state_;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
        return z ^ (z >> 31);
    }
    /// Uniform in [0, 1): the top 53 bits.
    double uniform() { return static_cast<double>(next() >> 11) * (1.0 / 9007199254740992.0); }

private:
    std::uint64_t state_;
};

/// One player: tries until the first drop and returns how many it took (kMaxAttempts if the model never drops).
inline int rollPlayer(const Model& m, Rng& rng) {
    for (int n = 1; n <= kMaxAttempts; ++n) {
        if (rng.uniform() < m.chance(n)) return n;
    }
    return kMaxAttempts;
}

struct Summary {
    int count = 0;
    double mean = 0.0;
    int median = 0;
    int p95 = 0;
    int max = 0;
    double shareAbove = 0.0;  // share of players who needed more than `threshold` tries
};

/// Smallest value with at least q percent of the sorted values at or below it. Integer math: no rounding surprises.
inline int percentile(const std::vector<int>& sorted, int q) {
    const std::size_t n = sorted.size();
    const std::size_t index = (n * static_cast<std::size_t>(q) + 99) / 100 - 1;
    return sorted[std::min(index, n - 1)];
}

inline Summary summarize(const std::vector<int>& waits, int threshold) {
    Summary s;
    if (waits.empty()) return s;
    std::vector<int> sorted = waits;
    std::sort(sorted.begin(), sorted.end());
    double sum = 0.0;
    int above = 0;
    for (const int w : sorted) {
        sum += w;
        if (w > threshold) ++above;
    }
    s.count = static_cast<int>(sorted.size());
    s.mean = sum / s.count;
    s.median = percentile(sorted, 50);
    s.p95 = percentile(sorted, 95);
    s.max = sorted.back();
    s.shareAbove = static_cast<double>(above) / s.count;
    return s;
}

/// Counts per bar; waits beyond the last bar are returned in `overflow`.
inline std::vector<int> histogram(const std::vector<int>& waits, int binWidth, int bins, int& overflow) {
    std::vector<int> counts(static_cast<std::size_t>(std::max(0, bins)), 0);
    overflow = 0;
    for (const int w : waits) {
        const int bar = std::max(0, w - 1) / std::max(1, binWidth);
        if (bar >= bins) {
            ++overflow;
        } else {
            ++counts[static_cast<std::size_t>(bar)];
        }
    }
    return counts;
}

/// Smallest 1, 2 or 5 times a power of ten that is at least `v`: a round top for the histogram axis.
inline double niceCeil(double v) {
    if (v <= 1.0) return 1.0;
    const double magnitude = std::pow(10.0, std::floor(std::log10(v)));
    for (const double step : {1.0, 2.0, 5.0, 10.0}) {
        if (step * magnitude >= v - 1e-9) return step * magnitude;
    }
    return 10.0 * magnitude;
}

/// 10000 -> "10,000". Built by hand because printf's grouping depends on the locale.
inline std::string thousands(long value) {
    std::string digits = std::to_string(value < 0 ? -value : value);
    std::string out;
    for (std::size_t i = 0; i < digits.size(); ++i) {
        if (i > 0 && (digits.size() - i) % 3 == 0) out += ',';
        out += digits[i];
    }
    return value < 0 ? "-" + out : out;
}

}  // namespace pity
