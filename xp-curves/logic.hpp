// XP curves: the logic of the demo, no SDL, so test.cpp can check it.
//
// A level curve is the answer to "how much experience does the next level cost?". Everything else follows from it:
//
//   step(L)   = XP to go from level L to level L+1          (what a designer tunes)
//   total(L)  = XP you need to reach level L                = step(1) + ... + step(L-1), total(1) = 0
//   level(xp) = the highest L with total(L) <= xp            (binary search over the totals)
//
// Families (a and b are the two sliders):
//   linear       step(L) = a * L
//   polynomial   step(L) = a * L^b
//   exponential  step(L) = a * b^L
//   RuneScape    points(l) = floor(l + 300 * 2^(l/7)),  total(L) = floor((points(1) + ... + points(L-1)) / 4)
//                The step is derived as total(L+1) - total(L). The 2^(l/7) makes it double about every 7 levels.
//                Checked values: level 2 = 83, level 10 = 1,154, level 50 = 101,333, level 99 = 13,034,431.
//                (Written with the level index l = L, the same as floor(L-1 + 300 * 2^((L-1)/7)) / 4 for the step
//                that leads TO level L.)
//   table        10 hand-picked step values at fixed levels, in between interpolated geometrically (so the curve is
//                smooth) and rounded to tens like a spreadsheet would.
//
// Overflow: exponential curves leave 64 bit quickly (10 * 2^99 is 6e30), so one step saturates at kMaxStep = 4e15.
// 98 steps of that still sum to 3.9e17, well below INT64_MAX = 9.2e18, so the totals never wrap.
// Every step costs at least 1 XP, which keeps the totals strictly rising (and the level lookup unambiguous) even for a = 0.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

namespace xpc {

enum class Family { Linear, Polynomial, Exponential, RuneScape, Table };

inline constexpr int kMaxLevel = 99;
inline constexpr std::int64_t kMaxStep = 4'000'000'000'000'000LL;
inline constexpr double kRateMin = 100.0;           // XP per hour, left end of the rate slider
inline constexpr double kRateMax = 10'000'000.0;    // right end

struct Params {
    double a = 0.0;
    double b = 0.0;
};

/// Slider range of one parameter. `used` is false when the family has no such parameter.
struct Range {
    double lo = 0.0;
    double hi = 0.0;
    int decimals = 0;
    bool used = false;
};

struct ParamInfo {
    Range a;
    Range b;
    Params def;
};

inline ParamInfo paramInfo(Family f) {
    switch (f) {
        case Family::Linear: return {{0.0, 5000.0, 0, true}, {}, {2500.0, 0.0}};
        case Family::Polynomial: return {{0.0, 10.0, 1, true}, {1.0, 4.0, 2, true}, {3.0, 2.6}};
        case Family::Exponential: return {{0.0, 100.0, 0, true}, {1.0, 1.25, 3, true}, {60.0, 1.1}};
        case Family::RuneScape:
        case Family::Table: return {};
    }
    return {};
}

namespace detail {

struct Anchor {
    int level;
    double step;
};

inline constexpr std::array<Anchor, 11> kTable{{{1, 100},     {10, 400},    {20, 1000},   {30, 2200},  {40, 4500},   {50, 8500},
                                                  {60, 15000}, {70, 26000}, {80, 42000}, {90, 70000}, {99, 110000}}};

/// Geometric interpolation between the table anchors, rounded to tens.
inline double tableStep(int level) {
    const auto it = std::lower_bound(kTable.begin(), kTable.end(), level, [](const Anchor& a, int l) { return a.level < l; });
    if (it == kTable.begin()) return kTable.front().step;
    if (it == kTable.end()) return kTable.back().step;
    if (it->level == level) return it->step;
    const Anchor& lo = *(it - 1);
    const double t = static_cast<double>(level - lo.level) / static_cast<double>(it->level - lo.level);
    return std::round(lo.step * std::pow(it->step / lo.step, t) / 10.0) * 10.0;
}

inline double runescapePoints(int l) { return std::floor(static_cast<double>(l) + 300.0 * std::exp2(static_cast<double>(l) / 7.0)); }

/// A raw step as a double to a legal one: NaN and tiny values cost 1, huge ones saturate.
inline std::int64_t toStep(double raw) {
    if (std::isnan(raw) || raw < 1.0) return 1;
    if (!(raw < static_cast<double>(kMaxStep))) return kMaxStep;
    return static_cast<std::int64_t>(std::llround(raw));
}

}  // namespace detail

class Curve {
public:
    /// Builds total(1..kMaxLevel+1); the extra level makes next(kMaxLevel) defined for the plot.
    Curve(Family family, Params p) : totals_(kMaxLevel + 1, 0) {
        const double a = (std::isnan(p.a) || p.a < 0.0) ? 0.0 : p.a;
        const double bPoly = (std::isnan(p.b) || p.b < 0.0) ? 0.0 : p.b;
        const double bExp = (std::isnan(p.b) || p.b < 1.0) ? 1.0 : p.b;
        if (family == Family::RuneScape) {
            std::int64_t sum = 0;
            for (int l = 1; l <= kMaxLevel; ++l) {
                sum += static_cast<std::int64_t>(detail::runescapePoints(l));
                totals_[static_cast<std::size_t>(l)] = sum / 4;
            }
            return;
        }
        for (int l = 1; l <= kMaxLevel; ++l) {
            double raw = 0.0;
            switch (family) {
                case Family::Linear: raw = a * l; break;
                case Family::Polynomial: raw = a * std::pow(static_cast<double>(l), bPoly); break;
                case Family::Exponential: raw = a * std::pow(bExp, static_cast<double>(l)); break;
                case Family::Table: raw = detail::tableStep(l); break;
                case Family::RuneScape: break;
            }
            totals_[static_cast<std::size_t>(l)] = totals_[static_cast<std::size_t>(l) - 1] + detail::toStep(raw);
        }
    }

    /// XP needed to reach `level` (1..kMaxLevel+1, anything else is clamped).
    std::int64_t total(int level) const { return totals_[index(level)]; }

    /// XP to go from `level` to the next one (1..kMaxLevel).
    std::int64_t next(int level) const {
        const int l = std::clamp(level, 1, kMaxLevel);
        return total(l + 1) - total(l);
    }

    /// The highest level whose threshold is reached, by binary search. Below 0 XP it is 1, above the cap it stays at the cap.
    int levelForXp(std::int64_t xp) const {
        const auto first = totals_.begin();
        const auto last = totals_.begin() + kMaxLevel;  // thresholds of levels 1..kMaxLevel
        const auto it = std::upper_bound(first, last, xp);
        return std::max(1, static_cast<int>(it - first));
    }

private:
    std::vector<std::int64_t> totals_;  // totals_[L-1] = total(L)

    static std::size_t index(int level) { return static_cast<std::size_t>(std::clamp(level, 1, kMaxLevel + 1) - 1); }
};

inline double hoursToLevel(const Curve& c, int level, double xpPerHour) {
    if (!(xpPerHour > 0.0)) return std::numeric_limits<double>::infinity();
    return static_cast<double>(c.total(level)) / xpPerHour;
}

inline double hoursForNext(const Curve& c, int level, double xpPerHour) {
    if (!(xpPerHour > 0.0)) return std::numeric_limits<double>::infinity();
    return static_cast<double>(c.next(level)) / xpPerHour;
}

/// The XP per hour slider is logarithmic: 0..1 maps to kRateMin..kRateMax.
inline double rateFromSlider(double t) {
    t = std::clamp(t, 0.0, 1.0);
    return kRateMin * std::pow(kRateMax / kRateMin, t);
}

inline double sliderFromRate(double rate) {
    rate = std::clamp(rate, kRateMin, kRateMax);
    return std::log(rate / kRateMin) / std::log(kRateMax / kRateMin);
}

/// Rounds to `digits` significant digits (31622 -> 32000), so a slider shows 32,000 and not 31,622.
inline double roundSignificant(double v, int digits) {
    if (!(v > 0.0)) return 0.0;
    const double scale = std::pow(10.0, std::floor(std::log10(v)) - digits + 1);
    return std::round(v / scale) * scale;
}

/// The simulated player. XP is a double so slow rates and small time steps do not get lost to rounding.
struct Player {
    double xp = 0.0;
    std::int64_t xpInt() const { return static_cast<std::int64_t>(xp); }
};

inline void advance(Player& p, const Curve& c, double hours, double xpPerHour) {
    if (!(hours > 0.0) || !(xpPerHour > 0.0)) return;
    p.xp = std::min(p.xp + hours * xpPerHour, static_cast<double>(c.total(kMaxLevel)));
}

/// Fraction (0..1) of the way from the current level to the next. At the level cap the bar is full.
inline double levelProgress(const Curve& c, double xp) {
    const int level = c.levelForXp(static_cast<std::int64_t>(std::max(xp, 0.0)));
    if (level >= kMaxLevel) return 1.0;
    return std::clamp((xp - static_cast<double>(c.total(level))) / static_cast<double>(c.next(level)), 0.0, 1.0);
}

/// 13034431 -> "13,034,431", independent of the locale (snprintf with a locale could print dots).
inline std::string formatThousands(std::int64_t v) {
    const bool negative = v < 0;
    // Negate in unsigned arithmetic so INT64_MIN works.
    std::uint64_t u = negative ? ~static_cast<std::uint64_t>(v) + 1U : static_cast<std::uint64_t>(v);
    std::string digits = std::to_string(u);
    std::string out;
    for (std::size_t i = 0; i < digits.size(); ++i) {
        if (i > 0 && (digits.size() - i) % 3 == 0) out += ',';
        out += digits[i];
    }
    return negative ? "-" + out : out;
}

namespace detail {

/// "%.1f" without a trailing ".0" (and without locale: only digits and '.').
inline std::string trimmed(double v, const char* fmt) {
    char buf[32];
    std::snprintf(buf, sizeof buf, fmt, v);
    std::string s = buf;
    if (s.find('.') != std::string::npos) {
        while (s.back() == '0') s.pop_back();
        if (s.back() == '.') s.pop_back();
    }
    return s;
}

}  // namespace detail

/// Short axis labels: 1500 -> "1.5k", 13034431 -> "13M", 0.01 -> "0.01".
inline std::string formatShort(double v) {
    if (v == 0.0) return "0";
    if (std::fabs(v) < 1.0) return detail::trimmed(v, "%.3g");
    static constexpr std::array<std::pair<double, const char*>, 4> kUnits{{{1e9, "B"}, {1e6, "M"}, {1e3, "k"}, {1.0, ""}}};
    const double mag = std::fabs(v);
    std::size_t i = 0;
    while (i + 1 < kUnits.size() && mag < kUnits[i].first) ++i;
    double scaled = std::round(mag / kUnits[i].first * 10.0) / 10.0;
    // 999.99k rounds to 1000k: that is 1M, so step up one unit.
    if (scaled >= 1000.0 && i > 0) {
        --i;
        scaled = std::round(mag / kUnits[i].first * 10.0) / 10.0;
    }
    return detail::trimmed(std::copysign(scaled, v), "%.1f") + kUnits[i].second;
}

/// Play time: "<1 min", "30 min", "12.3 h", "130 h", "1,234 h", "never".
inline std::string formatHours(double h) {
    if (std::isinf(h) || std::isnan(h)) return "never";
    if (h * 60.0 < 1.0) return "<1 min";
    const long minutes = std::lround(h * 60.0);
    if (minutes < 60) return std::to_string(minutes) + " min";
    const long tenths = std::lround(h * 10.0);
    if (tenths < 1000) {
        char buf[32];
        std::snprintf(buf, sizeof buf, "%ld.%ld h", tenths / 10, tenths % 10);
        return buf;
    }
    return formatThousands(std::llround(h)) + " h";
}

/// 1, 2, 5 times a power of ten, the usual choice for axis steps. `intervals` is how many steps the range should have.
inline double niceStep(double range, int intervals) {
    if (!(range > 0.0) || intervals < 1) return 1.0;
    const double raw = range / static_cast<double>(intervals);
    const double magnitude = std::pow(10.0, std::floor(std::log10(raw)));
    const double frac = raw / magnitude;
    return (frac <= 1.0 ? 1.0 : frac <= 2.0 ? 2.0 : frac <= 5.0 ? 5.0 : 10.0) * magnitude;
}

/// A y axis, linear (from 0 up to a round number) or logarithmic (whole decades), with its tick values.
struct Axis {
    double lo = 0.0;
    double hi = 1.0;
    bool log = false;
    std::vector<double> ticks;

    /// Position 0..1 along the axis, clamped. Values below a log axis (including 0) land on 0.
    double map(double v) const {
        if (log) {
            if (!(v > 0.0)) return 0.0;
            return std::clamp((std::log10(v) - std::log10(lo)) / (std::log10(hi) - std::log10(lo)), 0.0, 1.0);
        }
        return std::clamp((v - lo) / (hi - lo), 0.0, 1.0);
    }
};

/// Axis for data between `minV` and `maxV`. Linear: `n` is the wanted number of intervals. Logarithmic: `n` is the most
/// ticks that fit, decades are skipped evenly when there are more, and `floorLo` is the smallest value the axis starts at.
inline Axis makeAxis(double minV, double maxV, bool log, int n, double floorLo = 1e-3) {
    Axis axis;
    axis.log = log;
    if (!log) {
        const double step = niceStep(maxV, n);
        axis.lo = 0.0;
        axis.hi = std::max(std::ceil(maxV / step - 1e-9), 1.0) * step;
        for (int i = 0; axis.lo + step * i <= axis.hi + step * 1e-9; ++i) axis.ticks.push_back(axis.lo + step * i);
        return axis;
    }
    const double low = std::max(minV, floorLo);
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

}  // namespace xpc
