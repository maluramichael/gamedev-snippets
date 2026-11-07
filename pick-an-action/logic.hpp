// Pick an action: the logic of the demo, no SDL, so test.cpp can check it.
//
// Scoring says how much each option is wanted. The selection rule says what to do with those scores, and it is the
// rule that gives an agent its character. The same six scores can be read four ways:
//
//   Max       always the highest score. Ties split evenly. Predictable: a robot.
//   Weighted  every option is picked with probability weight / sum of weights. Varied, but a low score is never
//             impossible, so now and then the agent does something silly.
//   Band      weighted random among the options within `band` of the best score (30 percent here, so anything under
//             70 percent of the best is out).
//   Dual      dual-utility reasoning (Kevin Dill): drop options with weight 0, keep only the highest rank, then
//             weighted random among those. Rank is an absolute class ("urgent beats everything"), the weight is the
//             relative want inside that class.
//
// "Silly" is measured: a pick is dumb if its score is under half of the best score on offer (`isDumb`).
//
// Probabilities are computed exactly (`probabilities`), sampling is a separate step (`pickFrom`), so the tests can check
// the math once and the sampler against it.
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

namespace pick {

constexpr std::size_t kActions = 6;
constexpr double kBand = 0.3;       // Band rule: options below (1 - kBand) of the best are out
constexpr double kCritical = 0.8;   // a need at or above this is urgent and lifts its action to rank 2
constexpr double kDumbShare = 0.5;  // a pick scoring under this share of the best score is dumb
constexpr double kEpsilon = 1e-12;  // float slack for ties and the band edge

enum class Rule { Max, Weighted, Band, Dual };

struct Option {
    double weight;  // how much it is wanted, 0 or less means invalid
    int rank;       // absolute class, higher beats lower
};

/// hunger, tired, bored, lonely, each 0 (fine) to 1 (desperate)
using Needs = std::array<double, 4>;

inline const char* actionName(std::size_t i) {
    static const char* const names[kActions] = {"Eat", "Snack", "Sleep", "Play", "Chat", "Watch TV"};
    return i < kActions ? names[i] : "?";
}

inline double clamp01(double v) { return std::clamp(v, 0.0, 1.0); }

/// Order: Eat, Snack, Sleep, Play, Chat, Watch TV. Score is the need itself, Snack takes the edge off hunger at 0.4 of it, Watch TV is
/// the filler with a constant 0.25. Rank 2 for an urgent need, 1 for a normal one, 0 for the filler.
inline std::vector<Option> options(const Needs& needs) {
    const double h = clamp01(needs[0]);
    const double t = clamp01(needs[1]);
    const double b = clamp01(needs[2]);
    const double l = clamp01(needs[3]);
    const auto rankOf = [](double need) { return need >= kCritical ? 2 : 1; };
    return {{h, rankOf(h)}, {0.4 * h, 1}, {t, rankOf(t)}, {b, rankOf(b)}, {l, rankOf(l)}, {0.25, 0}};
}

inline double bestWeight(const std::vector<Option>& opts) {
    double best = 0.0;
    for (const Option& o : opts) best = std::max(best, o.weight);
    return best;
}

namespace detail {

/// Scale to a sum of 1. An all zero vector stays all zero (nothing was valid).
inline void normalize(std::vector<double>& p) {
    double sum = 0.0;
    for (double x : p) sum += x;
    if (sum <= 0.0) return;
    for (double& x : p) x /= sum;
}

}  // namespace detail

/// Exact probability of picking each option under `rule`. All zeros when no option has a weight above 0.
/// `band` only matters for Rule::Band and is clamped to 0..1 (0 degenerates to Max).
inline std::vector<double> probabilities(const std::vector<Option>& opts, Rule rule, double band = kBand) {
    std::vector<double> p(opts.size(), 0.0);
    double best = 0.0;
    int topRank = std::numeric_limits<int>::min();
    for (const Option& o : opts) {
        if (o.weight <= 0.0) continue;
        best = std::max(best, o.weight);
        topRank = std::max(topRank, o.rank);
    }
    if (best <= 0.0) return p;

    const double cutoff = best * (1.0 - clamp01(band)) - kEpsilon;
    for (std::size_t i = 0; i < opts.size(); ++i) {
        const Option& o = opts[i];
        if (o.weight <= 0.0) continue;
        switch (rule) {
            case Rule::Max: p[i] = o.weight >= best - kEpsilon ? 1.0 : 0.0; break;  // ties share evenly after normalizing
            case Rule::Weighted: p[i] = o.weight; break;
            case Rule::Band: p[i] = o.weight >= cutoff ? o.weight : 0.0; break;
            case Rule::Dual: p[i] = o.rank == topRank ? o.weight : 0.0; break;
        }
    }
    detail::normalize(p);
    return p;
}

/// Walks the cumulative distribution. u in [0, 1). Options with probability 0 are skipped even when u is exactly 0, and rounding at
/// the top end falls back to the last valid option. Nothing when no option is valid.
inline std::optional<std::size_t> pickFrom(const std::vector<double>& p, double u) {
    std::optional<std::size_t> last;
    double cumulative = 0.0;
    for (std::size_t i = 0; i < p.size(); ++i) {
        if (p[i] <= 0.0) continue;
        cumulative += p[i];
        last = i;
        if (u < cumulative) return i;
    }
    return last;
}

/// A pick is dumb if it scores under half of the best score on offer. Exactly half is fine. With no score at all nothing is dumb.
inline bool isDumb(double weight, double best) { return best > 0.0 && weight < kDumbShare * best; }

/// Chance that a single pick is dumb, summed from the exact probabilities.
inline double expectedDumbRate(const std::vector<Option>& opts, const std::vector<double>& p) {
    const double best = bestWeight(opts);
    double rate = 0.0;
    for (std::size_t i = 0; i < opts.size() && i < p.size(); ++i) {
        if (isDumb(opts[i].weight, best)) rate += p[i];
    }
    return rate;
}

/// xorshift64*, own generator so every platform draws the same numbers (std distributions differ between standard libraries).
struct Rng {
    std::uint64_t state;

    /// Uniform in [0, 1) with 53 bits.
    double next() {
        if (state == 0) state = 0x9E3779B97F4A7C15ULL;  // xorshift gets stuck at 0
        state ^= state >> 12;
        state ^= state << 25;
        state ^= state >> 27;
        const std::uint64_t r = state * 0x2545F4914F6CDD1DULL;
        return static_cast<double>(r >> 11) * (1.0 / 9007199254740992.0);
    }
};

/// How often each option was picked and how many of those picks were dumb.
struct Tally {
    explicit Tally(std::size_t options) : counts(options, 0) {}

    std::vector<long> counts;
    long picks = 0;
    long dumb = 0;

    void add(std::size_t index, const std::vector<Option>& opts) {
        if (index >= counts.size() || index >= opts.size()) return;
        ++counts[index];
        ++picks;
        if (isDumb(opts[index].weight, bestWeight(opts))) ++dumb;
    }

    void clear() {
        std::fill(counts.begin(), counts.end(), 0L);
        picks = 0;
        dumb = 0;
    }

    double dumbRate() const { return picks > 0 ? static_cast<double>(dumb) / static_cast<double>(picks) : 0.0; }
};

/// Slider position: x on a track starting at `left` with `width`, mapped to 0..1 and clamped.
inline double valueFromX(float x, float left, float width) {
    if (width <= 0.0F) return 0.0;
    return clamp01(static_cast<double>((x - left) / width));
}

}  // namespace pick
