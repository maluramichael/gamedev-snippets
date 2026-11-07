// Three RimWorld storytellers on a timeline, no SDL. Everything that can be wrong lives here.
//
// Cassandra and Phoebe run on a fixed calendar: an "on" window and an "off" window alternate, the dates never move, and
// inside an "on" window a fixed number of major threats is dropped at random times (Cassandra 1 or 2, at least 1.9 days
// apart; Phoebe exactly 1). Randy has no calendar. Every 1000 ticks (1/60 of a day) he flips a weighted coin, on average one
// event every 1.35 days, and a fixed share of those events is a major threat. If 13 days pass without one, the next event
// he fires is a major threat. All numbers are from the RimWorld wiki and were not checked against the game files. The wiki
// gives the windows, the counts and the spacing but not where in the window a threat falls: uniformly is this demo's choice.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

namespace stl {

inline constexpr double kYearDays = 60.0;       // four quadrums of 15 days
inline constexpr double kChecksPerDay = 60.0;   // 60000 ticks a day, one check per 1000 ticks

/// SplitMix64: tiny, fast, and the same on every platform, so a seed tells the same story everywhere.
struct Rng {
    std::uint64_t state = 1;
    std::uint64_t next() {
        state += 0x9E3779B97F4A7C15ULL;
        std::uint64_t z = state;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
        return z ^ (z >> 31);
    }
    /// 53 random bits in [0, 1).
    double unit() { return static_cast<double>(next() >> 11) * (1.0 / 9007199254740992.0); }
};

enum class Teller { Cassandra, Phoebe, Randy };
enum class Category { Misc, ThreatBig, OrbitalVisitor, FactionArrival, ThreatSmall, ShipChunkDrop };

/// One fired event. Only Randy fires anything besides major threats in this demo.
struct Event {
    double day = 0.0;
    Category category = Category::ThreatBig;
    bool forced = false;  // Randy: fired as a threat because 13 days had passed
};

struct Window {
    double from = 0.0;
    double to = 0.0;
};

// ---- Cassandra and Phoebe: the calendar --------------------------------------------------------------------------------

struct Cycle {
    double start;       // day of the first "on" window
    double onDays;
    double offDays;
    double minSpacing;  // minimum days between two threats of one window
    int minCount;
    int maxCount;
};

inline constexpr Cycle kCassandra{11.0, 4.6, 6.0, 1.9, 1, 2};
inline constexpr Cycle kPhoebe{13.0, 8.0, 8.0, 0.0, 1, 1};

inline double period(const Cycle& c) { return c.onDays + c.offDays; }

/// All "on" windows that start before `horizon`, the last one cut off at the horizon.
inline std::vector<Window> windows(const Cycle& c, double horizon) {
    std::vector<Window> out;
    const double p = period(c);
    if (!(p > 0.0)) return out;
    for (int k = 0;; ++k) {
        const double from = c.start + static_cast<double>(k) * p;
        if (from >= horizon) break;
        out.push_back({from, std::min(from + c.onDays, horizon)});
    }
    return out;
}

/// The half open "on" window [from, to) contains `day`.
inline bool onPhase(const Cycle& c, double day) {
    const double p = period(c);
    if (!(p > 0.0) || day < c.start) return false;
    const double inCycle = std::fmod(day - c.start, p);
    return inCycle < c.onDays;
}

/// How many of `wanted` threats fit into a window of `onDays` with `spacing` days between them (never more than wanted).
inline int maxThreatsFitting(double onDays, double spacing, int wanted) {
    if (wanted <= 0) return 0;
    if (!(spacing > 0.0)) return wanted;
    if (!(onDays > 0.0)) return 1;
    const int fit = static_cast<int>(std::floor(onDays / spacing)) + 1;
    return std::min(wanted, fit);
}

/// Turns n uniform numbers in [0, 1) into n sorted offsets inside a window that are at least `spacing` apart and all within
/// [0, onDays]. Shrink the window by (n - 1) * spacing, place the points freely, then push the i-th one back by i * spacing:
/// every valid arrangement is equally likely.
inline std::vector<double> windowOffsets(int n, double onDays, double spacing, std::vector<double> units) {
    std::vector<double> out;
    if (n <= 0) return out;
    units.resize(static_cast<std::size_t>(n), 0.0);
    std::sort(units.begin(), units.end());
    const double width = onDays - static_cast<double>(n - 1) * spacing;
    for (int i = 0; i < n; ++i) out.push_back(units[static_cast<std::size_t>(i)] * width + static_cast<double>(i) * spacing);
    return out;
}

inline std::vector<Event> cycleEvents(const Cycle& c, double horizon, Rng& rng) {
    std::vector<Event> out;
    for (const Window& w : windows(c, horizon)) {
        const int span = c.maxCount - c.minCount + 1;
        int want = c.minCount + static_cast<int>(std::floor(rng.unit() * static_cast<double>(span)));
        want = std::min(want, c.maxCount);
        const int n = maxThreatsFitting(c.onDays, c.minSpacing, want);
        std::vector<double> units;
        for (int i = 0; i < n; ++i) units.push_back(rng.unit());
        for (const double offset : windowOffsets(n, c.onDays, c.minSpacing, units)) {
            const double day = w.from + offset;
            if (day < horizon) out.push_back({day, Category::ThreatBig, false});
        }
    }
    return out;
}

// ---- Randy: a coin every 1000 ticks -----------------------------------------------------------------------------------

inline constexpr double kRandyMinDays = 1.0;     // no event before day 1
inline constexpr double kRandyMtbDays = 1.35;    // mean time between events
inline constexpr int kRandyCapChecks = 780;      // 13 days without a major threat: the next event is one
inline constexpr std::array<double, 6> kRandyWeights{3.5, 1.4, 1.1, 2.4, 0.6, 0.22};
inline constexpr double kRandyWeightTotal = 3.5 + 1.4 + 1.1 + 2.4 + 0.6 + 0.22;

/// Chance that an event fires at a single check: one expected event every kRandyMtbDays * 60 checks.
inline constexpr double kRandyChance = 1.0 / (kRandyMtbDays * kChecksPerDay);

/// Maps a uniform number to an event category by the weights (Misc first). Out of range input is clamped.
inline Category pickCategory(double u) {
    double target = std::clamp(u, 0.0, 1.0) * kRandyWeightTotal;
    for (std::size_t i = 0; i < kRandyWeights.size(); ++i) {
        if (target < kRandyWeights[i]) return static_cast<Category>(i);
        target -= kRandyWeights[i];
    }
    return Category::ShipChunkDrop;
}

inline double randyThreatShare() { return kRandyWeights[static_cast<std::size_t>(Category::ThreatBig)] / kRandyWeightTotal; }

/// Mean days between major threats from the wiki's derivation. Until day 13 they arrive at rate r (events times threat share),
/// after that the next event of any kind is one: E = (1 - e^(-13 r)) / r + e^(-13 r) * 1.35.
inline double randyExpectedGap() {
    const double r = randyThreatShare() / kRandyMtbDays;
    const double cap = static_cast<double>(kRandyCapChecks) / kChecksPerDay;
    const double survive = std::exp(-cap * r);
    return (1.0 - survive) / r + survive * kRandyMtbDays;
}

inline std::vector<Event> randyEvents(double horizon, Rng& rng) {
    std::vector<Event> out;
    int lastThreat = 0;  // the clock of the 13 day rule starts at day 0
    for (int check = 1;; ++check) {
        const double day = static_cast<double>(check) / kChecksPerDay;
        if (day >= horizon) break;
        if (day < kRandyMinDays) continue;
        if (!(rng.unit() < kRandyChance)) continue;
        Event e{day, pickCategory(rng.unit()), false};
        if (check - lastThreat >= kRandyCapChecks) {
            e.category = Category::ThreatBig;
            e.forced = true;
        }
        if (e.category == Category::ThreatBig) lastThreat = check;
        out.push_back(e);
    }
    return out;
}

inline std::vector<Event> simulate(Teller t, double horizon, std::uint64_t seed) {
    Rng rng{seed};
    switch (t) {
        case Teller::Cassandra: return cycleEvents(kCassandra, horizon, rng);
        case Teller::Phoebe: return cycleEvents(kPhoebe, horizon, rng);
        case Teller::Randy: return randyEvents(horizon, rng);
    }
    return {};
}

// ---- Statistics ---------------------------------------------------------------------------------------------------------

inline std::vector<double> threatDays(const std::vector<Event>& events) {
    std::vector<double> out;
    for (const Event& e : events) {
        if (e.category == Category::ThreatBig) out.push_back(e.day);
    }
    return out;
}

/// Days between consecutive entries (input sorted).
inline std::vector<double> gaps(const std::vector<double>& days) {
    std::vector<double> out;
    for (std::size_t i = 1; i < days.size(); ++i) out.push_back(days[i] - days[i - 1]);
    return out;
}

/// Counts per bin of `binDays`; everything past the last bin lands in it, negatives in the first.
inline std::vector<int> histogram(const std::vector<double>& values, int bins, double binDays) {
    std::vector<int> out(static_cast<std::size_t>(std::max(bins, 0)), 0);
    if (out.empty() || !(binDays > 0.0)) return out;
    for (const double v : values) {
        const int idx = std::clamp(static_cast<int>(std::floor(v / binDays)), 0, bins - 1);
        ++out[static_cast<std::size_t>(idx)];
    }
    return out;
}

/// Threats per 60 day year.
inline double perYear(int count, double days) { return days > 0.0 ? static_cast<double>(count) * kYearDays / days : 0.0; }

/// How many entries lie in [from, to).
inline int countIn(const std::vector<double>& days, double from, double to) {
    int n = 0;
    for (const double d : days) n += (d >= from && d < to) ? 1 : 0;
    return n;
}

/// Gaps and rate of one storyteller over many colonies (consecutive seeds from `seedBase`).
struct Pool {
    std::vector<int> bins;
    int gapCount = 0;
    double meanGap = 0.0;
    double perYear = 0.0;
};

inline Pool pool(Teller t, int runs, double horizon, std::uint64_t seedBase, int bins, double binDays) {
    Pool p;
    std::vector<double> all;
    int threats = 0;
    for (int i = 0; i < runs; ++i) {
        const auto days = threatDays(simulate(t, horizon, seedBase + static_cast<std::uint64_t>(i)));
        threats += static_cast<int>(days.size());
        const auto g = gaps(days);
        all.insert(all.end(), g.begin(), g.end());
    }
    p.bins = histogram(all, bins, binDays);
    p.gapCount = static_cast<int>(all.size());
    double sum = 0.0;
    for (const double x : all) sum += x;
    p.meanGap = all.empty() ? 0.0 : sum / static_cast<double>(all.size());
    p.perYear = stl::perYear(threats, horizon * static_cast<double>(runs));
    return p;
}

}  // namespace stl
