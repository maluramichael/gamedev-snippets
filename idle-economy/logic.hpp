// Idle game economy without SDL: costs that grow geometrically, income that grows linearly, an auto buyer that
// jumps from purchase to purchase, prestige, and the number formatting a game with huge numbers needs.
// Covered by test.cpp. The tier prices and incomes are example values made up for this demo.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace idle {

inline constexpr int kTierCount = 4;

struct Tier {
    const char* name;
    double baseCost;    // price of the first one
    double baseIncome;  // money per second, each
};

inline constexpr std::array<Tier, kTierCount> kTiers{{{"Stand", 10.0, 0.2}, {"Shop", 100.0, 1.0}, {"Factory", 1200.0, 8.0}, {"Bank", 15000.0, 47.0}}};

inline constexpr double kStartMoney = 10.0;
inline constexpr double kPrestigeUnit = 1000.0;  // run earnings per point squared: points = floor(sqrt(earned / unit))
inline constexpr double kPointBonus = 0.1;       // every point adds 10 % of the base income
inline constexpr std::int64_t kMaxPrestigePoints = 1'000'000'000'000'000;
inline constexpr double kMoneyCap = 1e300;                // far beyond anything reachable, keeps money and earnings finite
inline constexpr std::int64_t kMaxCount = 1'000'000'000;  // owned per tier, a hard stop long before int64 matters
inline constexpr std::size_t kMaxPoints = 1024;           // history points per run before the run is thinned out
inline constexpr std::size_t kMaxRuns = 5;                // runs kept in the history

enum class CostMode { Exponential, Linear };

struct Rules {
    CostMode mode = CostMode::Exponential;
    double growth = 1.15;
};

// ---- costs ------------------------------------------------------------------------------------------------------

/// Growth of 1 or less (or NaN) means flat prices, nothing in here divides by zero because of it.
inline double growthOf(const Rules& r) { return r.growth > 1.0 ? r.growth : 1.0; }

/// Price of the next one when `owned` are already there: base * g^n, or base * (1 + n * (g - 1)) with linear costs.
inline double unitCost(const Rules& r, const Tier& t, std::int64_t owned) {
    const double n = static_cast<double>(std::max<std::int64_t>(owned, 0));
    const double g = growthOf(r);
    if (g <= 1.0) return t.baseCost;
    if (r.mode == CostMode::Exponential) return t.baseCost * std::pow(g, n);
    return t.baseCost * (1.0 + n * (g - 1.0));
}

/// Price of `count` more at once. Geometric series: base * g^n * (g^k - 1) / (g - 1). expm1 keeps it exact for small k.
inline double bulkCost(const Rules& r, const Tier& t, std::int64_t owned, std::int64_t count) {
    if (count <= 0) return 0.0;
    if (count == 1) return unitCost(r, t, owned);
    const double n = static_cast<double>(std::max<std::int64_t>(owned, 0));
    const double k = static_cast<double>(count);
    const double g = growthOf(r);
    if (g <= 1.0) return t.baseCost * k;
    if (r.mode == CostMode::Exponential) return t.baseCost * std::pow(g, n) * std::expm1(k * std::log(g)) / (g - 1.0);
    return t.baseCost * (k + (g - 1.0) * (k * n + k * (k - 1.0) / 2.0));
}

inline bool canAfford(double money, double cost) { return cost <= money; }

/// How many can be bought with `money`. The exponential case solves the series for k, then corrects the rounding
/// by checking the neighbours; the linear case is monotone, so a binary search does it.
inline std::int64_t maxAffordable(const Rules& r, const Tier& t, std::int64_t owned, double money) {
    if (!(money > 0.0) || !std::isfinite(money) || owned >= kMaxCount) return 0;
    const double unit = unitCost(r, t, owned);
    if (!(unit <= money)) return 0;  // also catches inf and NaN prices
    const std::int64_t limit = kMaxCount - std::max<std::int64_t>(owned, 0);
    const double g = growthOf(r);
    std::int64_t k = 1;
    if (g <= 1.0 || r.mode == CostMode::Exponential) {
        const double guess = g <= 1.0 ? money / unit : std::log1p(money * (g - 1.0) / unit) / std::log(g);
        k = guess >= static_cast<double>(limit) ? limit : std::max<std::int64_t>(1, static_cast<std::int64_t>(guess));
        for (int i = 0; i < 64 && k > 1 && bulkCost(r, t, owned, k) > money; ++i) --k;
        for (int i = 0; i < 64 && k < limit && bulkCost(r, t, owned, k + 1) <= money; ++i) ++k;
    } else {
        std::int64_t lo = 1;
        std::int64_t hi = limit;
        while (lo < hi) {
            const std::int64_t mid = lo + (hi - lo + 1) / 2;
            if (bulkCost(r, t, owned, mid) <= money) {
                lo = mid;
            } else {
                hi = mid - 1;
            }
        }
        k = lo;
    }
    return k;
}

// ---- state ------------------------------------------------------------------------------------------------------

struct Event {
    double time;     // seconds since the start of the run
    double income;  // per second, right after the purchase
    std::int64_t owned;  // buildings of all tiers, right after the purchase
    int tier;
};

struct State {
    std::array<std::int64_t, kTierCount> owned{};
    double money = kStartMoney;
    double time = 0.0;    // seconds in this run
    double earned = 0.0;  // money earned in this run (not what was spent)
    std::int64_t points = 0;
    int plan = -1;  // tier the auto buyer is saving for, -1 = decide next
};

inline double multiplier(const State& s) { return 1.0 + kPointBonus * static_cast<double>(s.points); }

inline double income(const State& s) {
    double sum = 0.0;
    for (int i = 0; i < kTierCount; ++i) sum += static_cast<double>(s.owned[static_cast<std::size_t>(i)]) * kTiers[static_cast<std::size_t>(i)].baseIncome;
    return sum * multiplier(s);
}

/// Prestige points a run with these earnings is worth. sqrt makes each further point cost more.
inline std::int64_t pointsFor(double earned) {
    if (!(earned > 0.0)) return 0;
    const double p = std::floor(std::sqrt(earned / kPrestigeUnit));
    return p >= static_cast<double>(kMaxPrestigePoints) ? kMaxPrestigePoints : static_cast<std::int64_t>(p);
}

/// Resets the run and keeps the points. Refuses (false) when the run is not worth a single point.
inline bool prestige(State& s) {
    const std::int64_t gain = pointsFor(s.earned);
    if (gain < 1) return false;
    const std::int64_t points = s.points + gain;
    s = State{};
    s.points = std::min(points, kMaxPrestigePoints);
    return true;
}

/// Seconds until `cost` can be paid at the current income. 0 when it can be paid now, infinity when nothing flows in.
inline double waitFor(const State& s, double cost) {
    if (canAfford(s.money, cost)) return 0.0;
    const double inc = income(s);
    return inc > 0.0 ? (cost - s.money) / inc : std::numeric_limits<double>::infinity();
}

struct Target {
    int tier;
    double cost;
    double wait;
};

/// The auto buyer's choice: the tier where waiting for the money plus the time to earn the price back is shortest.
/// Waiting for the best ratio alone would leave a poor player staring at a shop that costs a hundred times the income.
inline Target pickTarget(const Rules& r, const State& s) {
    const double mult = multiplier(s);
    auto look = [&](int i) {
        const Tier& t = kTiers[static_cast<std::size_t>(i)];
        const double cost = unitCost(r, t, s.owned[static_cast<std::size_t>(i)]);
        return Target{i, cost, waitFor(s, cost)};
    };
    auto score = [&](const Target& t) { return t.wait + t.cost / (kTiers[static_cast<std::size_t>(t.tier)].baseIncome * mult); };
    Target best = look(0);
    double bestScore = score(best);
    for (int i = 1; i < kTierCount; ++i) {
        const Target t = look(i);
        const double sc = score(t);
        if (sc < bestScore) {
            best = t;
            bestScore = sc;
        }
    }
    return best;
}

/// Buildings of all tiers together.
inline std::int64_t totalOwned(const State& s) {
    std::int64_t sum = 0;
    for (const std::int64_t n : s.owned) sum += n;
    return sum;
}

inline void record(const State& s, int tier, std::vector<Event>* log) {
    if (log != nullptr) log->push_back(Event{s.time, income(s), totalOwned(s), tier});
}

/// Buys `count` of a tier, all or nothing. Returns how many were bought (0 or `count`).
inline std::int64_t buy(const Rules& r, State& s, int tier, std::int64_t count, std::vector<Event>* log = nullptr) {
    if (tier < 0 || tier >= kTierCount || count < 1) return 0;
    std::int64_t& have = s.owned[static_cast<std::size_t>(tier)];
    if (count > kMaxCount - have) return 0;
    const double cost = bulkCost(r, kTiers[static_cast<std::size_t>(tier)], have, count);
    if (!canAfford(s.money, cost)) return 0;
    s.money = std::max(0.0, s.money - cost);
    have += count;
    s.plan = -1;
    record(s, tier, log);
    return count;
}

/// Lets `dt` seconds pass. With the auto buyer on, time jumps from purchase to purchase: income is constant in between,
/// so the moment the plan becomes affordable is a division, not a loop over frames. That makes the result independent of
/// the frame rate and cheap at any speed. At most `maxEvents` purchases per call; returns the time actually simulated.
inline double advance(const Rules& r, State& s, double dt, int maxEvents, bool autoBuy, std::vector<Event>* log) {
    if (!(dt > 0.0) || !std::isfinite(dt) || maxEvents < 1) return 0.0;
    double left = dt;
    int events = 0;
    auto elapse = [&](double seconds) {
        const double gained = income(s) * seconds;
        s.money = std::min(s.money + gained, kMoneyCap);
        s.earned = std::min(s.earned + gained, kMoneyCap);
        s.time += seconds;
    };
    while (left > 0.0) {
        if (!autoBuy) {
            elapse(left);
            left = 0.0;
            break;
        }
        if (s.plan < 0) s.plan = pickTarget(r, s).tier;
        const double cost = unitCost(r, kTiers[static_cast<std::size_t>(s.plan)], s.owned[static_cast<std::size_t>(s.plan)]);
        const double wait = waitFor(s, cost);
        if (!(wait <= left)) {
            elapse(left);
            left = 0.0;
            break;
        }
        if (events >= maxEvents) break;
        elapse(wait);
        left -= wait;
        s.money = std::max(s.money, cost);  // the division can land a hair below the price
        ++events;
        if (buy(r, s, s.plan, 1, log) == 0) {  // count limit or a price that is not a number: save the rest of the time
            elapse(left);
            left = 0.0;
        }
    }
    return dt - left;
}

// ---- history for the plots --------------------------------------------------------------------------------------

struct Point {
    double t;
    double income;
    double owned;
};

/// The points of one run. `cell` is the time resolution the run is kept at (0 = every purchase).
struct RunLog : std::vector<Point> {
    double cell = 0.0;
};

/// One list of points per run. Every purchase is a point until a run outgrows kMaxPoints; from then on points closer
/// together than `cell` seconds are merged (the newer one wins), and `cell` grows with the run, so the plot keeps the
/// same resolution everywhere on the time axis. Thinning by index would starve the stretches where purchases are rare.
struct History {
    std::vector<RunLog> runs = std::vector<RunLog>(1);

    void add(const Event& e) {
        RunLog& run = runs.back();
        const Point p{e.time, e.income, static_cast<double>(e.owned)};
        if (run.size() >= 2 && p.t - run[run.size() - 2].t < run.cell) {
            run.back() = p;
            return;
        }
        run.push_back(p);
        if (run.size() > kMaxPoints) thin(run);
    }

    void newRun() {
        runs.emplace_back();
        if (runs.size() > kMaxRuns) runs.erase(runs.begin());
    }

    void clear() { runs.assign(1, RunLog{}); }

private:
    static void thin(RunLog& run) {
        const double range = run.back().t - run.front().t;
        RunLog kept;
        kept.cell = std::max(run.cell * 2.0, range / static_cast<double>(kMaxPoints / 2));
        for (const Point& p : run) {
            if (kept.size() >= 2 && p.t - kept[kept.size() - 2].t < kept.cell) {
                kept.back() = p;
            } else {
                kept.push_back(p);
            }
        }
        if (kept.size() > kMaxPoints * 3 / 4) {  // many purchases at the same instant: the cell cannot help, halve by count
            RunLog half;
            half.cell = kept.cell;
            for (std::size_t i = 0; i < kept.size(); i += 2) half.push_back(kept[i]);
            if ((kept.size() - 1) % 2 != 0) half.push_back(kept.back());
            kept = std::move(half);
        }
        run = std::move(kept);
    }
};

/// Rules, state and history together, with the starting scene the demo shows on the first frame.
struct Game {
    Rules rules;
    State state;
    History history;
    bool autoBuy = true;

    /// Simulated seconds, purchases land in the history. Returns the time actually simulated.
    double run(double dt, int maxEvents) {
        std::vector<Event> log;
        const double used = advance(rules, state, dt, maxEvents, autoBuy, &log);
        for (const Event& e : log) history.add(e);
        return used;
    }

    std::int64_t buyNow(int tier, std::int64_t count) {
        std::vector<Event> log;
        const std::int64_t got = buy(rules, state, tier, count, &log);
        for (const Event& e : log) history.add(e);
        return got;
    }

    bool prestigeNow() {
        if (!prestige(state)) return false;
        history.newRun();
        return true;
    }

    /// A day of play, a prestige, then an hour of the second run: both curves are on screen from the start.
    /// Stops early when the event budget runs out (very flat prices make millions of purchases).
    void restart(double firstRun = 86400.0, double secondRun = 3600.0) {
        state = State{};
        history.clear();
        int budget = 200000;
        auto play = [&](double seconds) {
            double left = seconds;
            while (left > 0.0 && budget > 0) {
                const int chunk = std::min(budget, 20000);
                const double used = run(left, chunk);
                left -= used;
                budget -= chunk;
                if (!(used > 0.0)) break;
            }
        };
        play(firstRun);
        if (prestigeNow()) play(secondRun);
    }
};

// ---- number formatting ------------------------------------------------------------------------------------------
// Built from integers and strings only. snprintf with a locale would print "1,5" on a German system.

/// 1234567 -> "1,234,567".
inline std::string formatCount(std::int64_t v) {
    const bool negative = v < 0;
    // Negate in unsigned arithmetic so the smallest int64 works.
    const std::uint64_t u = negative ? ~static_cast<std::uint64_t>(v) + 1U : static_cast<std::uint64_t>(v);
    const std::string digits = std::to_string(u);
    std::string out;
    for (std::size_t i = 0; i < digits.size(); ++i) {
        if (i > 0 && (digits.size() - i) % 3 == 0) out += ',';
        out += digits[i];
    }
    return negative ? "-" + out : out;
}

/// Three significant digits at any size: 0.2, 12.34, 12,345, 1.23M, 150Dc, 1.00e36. NaN prints "-", infinity "inf".
inline std::string formatBig(double v) {
    if (std::isnan(v)) return "-";
    if (std::isinf(v)) return v < 0.0 ? "-inf" : "inf";
    if (v < 0.0) return "-" + formatBig(-v);
    if (v < 1000.0) {
        const long long cents = std::llround(v * 100.0);
        if (cents < 100000) {  // 999.996 would print as 1000.00, that case belongs to the next branch
            std::string s = std::to_string(cents / 100);
            const long long frac = cents % 100;
            if (frac != 0) {
                s += '.';
                s += static_cast<char>('0' + frac / 10);
                if (frac % 10 != 0) s += static_cast<char>('0' + frac % 10);
            }
            return s;
        }
    }
    if (v < 1e6) {
        const long long whole = std::llround(v);
        if (whole < 1000000) return formatCount(whole);
    }
    static constexpr const char* kSuffix[] = {"K", "M", "B", "T", "Qa", "Qi", "Sx", "Sp", "Oc", "No", "Dc"};
    int e = static_cast<int>(std::floor(std::log10(v)));
    double m = v / std::pow(10.0, e);
    while (m >= 10.0) {  // log10 and pow can be one ulp off at the powers of ten
        m /= 10.0;
        ++e;
    }
    while (m < 1.0) {
        m *= 10.0;
        --e;
    }
    long long r = std::llround(m * 100.0);  // 100..1000
    if (r >= 1000) {                        // 9.996 rounds up to 10.00: carry into the next group
        r = 100;
        ++e;
    }
    const std::string d = std::to_string(r);  // three digits
    if (e >= 36) return d.substr(0, 1) + "." + d.substr(1) + "e" + std::to_string(e);
    const int shift = e % 3;
    std::string out = shift == 2 ? d : shift == 1 ? d.substr(0, 2) + "." + d.substr(2) : d.substr(0, 1) + "." + d.substr(1);
    return out + kSuffix[e / 3 - 1];
}

/// 45 -> "45s", 61 -> "1m 01s", 3725 -> "1h 02m", 90000 -> "1d 01h", a year and more as "1y 35d". Infinity is "never".
inline std::string formatDuration(double seconds) {
    if (std::isnan(seconds) || seconds < 0.0) return "-";
    if (std::isinf(seconds)) return "never";
    if (seconds == 0.0) return "0s";
    if (seconds < 0.5) return "<1s";
    constexpr double kYear = 365.0 * 86400.0;
    if (seconds >= 1000.0 * kYear) return formatBig(seconds / kYear) + "y";
    const long long total = std::llround(seconds);
    auto two = [](long long v) { return (v < 10 ? "0" : "") + std::to_string(v); };
    if (total < 60) return std::to_string(total) + "s";
    if (total < 3600) return std::to_string(total / 60) + "m " + two(total % 60) + "s";
    if (total < 86400) return std::to_string(total / 3600) + "h " + two(total % 3600 / 60) + "m";
    if (total < 365 * 86400) return std::to_string(total / 86400) + "d " + two(total % 86400 / 3600) + "h";
    return std::to_string(total / (365 * 86400)) + "y " + std::to_string(total % (365 * 86400) / 86400) + "d";
}

// ---- axes -------------------------------------------------------------------------------------------------------

/// 1, 2, 5 times a power of ten, the usual choice for axis steps. `intervals` is how many steps the range should have.
inline double niceStep(double range, int intervals) {
    if (!(range > 0.0) || !std::isfinite(range) || intervals < 1) return 1.0;
    const double raw = range / static_cast<double>(intervals);
    const double magnitude = std::pow(10.0, std::floor(std::log10(raw)));
    const double frac = raw / magnitude;
    return (frac <= 1.0 ? 1.0 : frac <= 2.0 ? 2.0 : frac <= 5.0 ? 5.0 : 10.0) * magnitude;
}

/// Tick spacing for a time axis in seconds: 10 s, 5 min, 2 h, 12 h, ... the smallest that gives at most `maxTicks` steps.
inline double niceTimeStep(double range, int maxTicks) {
    if (!(range > 0.0) || !std::isfinite(range) || maxTicks < 1) return 1.0;
    static constexpr double kSteps[] = {1,    2,    5,     10,    15,    30,    60,    120,   300,   600,   900,
                                        1800, 3600, 7200,  10800, 21600, 43200, 86400, 172800, 432000, 864000};
    for (const double step : kSteps) {
        if (range / step <= static_cast<double>(maxTicks)) return step;
    }
    return niceStep(range, maxTicks);
}

/// A value axis, linear (from 0 up to a round number) or logarithmic (whole decades), with its tick values.
struct Axis {
    double lo = 0.0;
    double hi = 1.0;
    bool log = false;
    std::vector<double> ticks;

    /// Position 0..1 along the axis, clamped. Values below a log axis (zero and negatives too) land on 0.
    double map(double v) const {
        if (log) {
            if (!(v > 0.0)) return 0.0;
            return std::clamp((std::log10(v) - std::log10(lo)) / (std::log10(hi) - std::log10(lo)), 0.0, 1.0);
        }
        if (std::isnan(v)) return 0.0;
        return std::clamp((v - lo) / (hi - lo), 0.0, 1.0);
    }
};

/// Axis for data between `minV` and `maxV`. Linear: `n` is the wanted number of intervals. Logarithmic: `n` is the most
/// ticks that fit, decades are skipped evenly when there are more, `floorLo` is the smallest value the axis starts at.
inline Axis makeAxis(double minV, double maxV, bool log, int n, double floorLo = 1e-3) {
    Axis axis;
    axis.log = log;
    if (!std::isfinite(maxV)) maxV = 1.0;
    if (!log) {
        const double step = niceStep(maxV, n);
        axis.lo = 0.0;
        axis.hi = std::max(std::ceil(maxV / step - 1e-9), 1.0) * step;
        for (int i = 0; axis.lo + step * i <= axis.hi + step * 1e-9; ++i) axis.ticks.push_back(axis.lo + step * i);
        return axis;
    }
    const double low = std::max(std::isfinite(minV) ? minV : floorLo, floorLo);
    const int e0 = static_cast<int>(std::floor(std::log10(low) + 1e-9));
    int e1 = static_cast<int>(std::ceil(std::log10(std::max(maxV, low)) - 1e-9));
    if (e1 <= e0) e1 = e0 + 1;
    axis.lo = std::pow(10.0, e0);
    axis.hi = std::pow(10.0, e1);
    const int count = e1 - e0 + 1;
    const int stride = std::max(1, (count + std::max(n, 1) - 1) / std::max(n, 1));
    for (int e = e0; e <= e1; e += stride) axis.ticks.push_back(std::pow(10.0, e));
    return axis;
}

/// The time axis of the plots: linear with round steps, or logarithmic from 1 second with ticks at 1 s, 10 s, 1 min, 10 min,
/// 1 h, 10 h, 1 day, ... (a log axis cannot show 0, so everything before 1 s sits on the left edge).
struct TimeAxis {
    bool log = false;
    double hi = 60.0;
    std::vector<double> ticks;

    /// Position 0..1 along the axis, clamped.
    double map(double t) const {
        if (std::isnan(t)) return 0.0;
        if (log) return std::clamp(std::log10(std::max(t, 1.0)) / std::log10(hi), 0.0, 1.0);
        return std::clamp(t / hi, 0.0, 1.0);
    }
};

inline TimeAxis makeTimeAxis(double tMax, bool log, int maxTicks) {
    TimeAxis axis;
    axis.log = log;
    if (!std::isfinite(tMax) || tMax < 0.0) tMax = 0.0;
    if (!log) {
        const double span = std::max(tMax, 60.0);  // never narrower than a minute
        const double step = niceTimeStep(span, maxTicks);
        axis.hi = std::ceil(span / step - 1e-9) * step;
        for (double t = 0.0; t <= axis.hi + step * 1e-9; t += step) axis.ticks.push_back(t);
        return axis;
    }
    static constexpr double kTicks[] = {1.0, 10.0, 60.0, 600.0, 3600.0, 36000.0, 86400.0, 864000.0, 8640000.0, 86400000.0, 864000000.0, 31536000.0 * 10.0, 31536000.0 * 100.0};
    axis.hi = std::max(tMax, 10.0);
    for (const double t : kTicks) {
        if (t <= axis.hi) axis.ticks.push_back(t);
    }
    return axis;
}

/// Buildings the run owned at time t: the newest point at or before t (0 before the first purchase).
inline double ownedAt(const RunLog& run, double t) {
    double owned = 0.0;
    for (const Point& p : run) {
        if (p.t > t) break;
        owned = p.owned;
    }
    return owned;
}

/// How many more buildings ten times the playing time brought, measured at time t. On the exponential price curve this
/// number stays about the same decade after decade, that is the logarithm. 0 while t is below 10 s.
inline double buildingsPerDecade(const RunLog& run, double t) {
    if (!(t >= 10.0)) return 0.0;
    return ownedAt(run, t) - ownedAt(run, t / 10.0);
}

}  // namespace idle
