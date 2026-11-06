// Damage stacking: the logic of the demo, no SDL, so test.cpp can check it.
//
// Two kinds of percentage bonus look the same in a tooltip and behave very differently:
//
//   Increased  all lines share ONE pool. The pool is added up and applied once:  1 + (a + b + c) / 100
//   More       every line is its OWN factor and the factors are multiplied:      (1 + a/100) * (1 + b/100) * ...
//
//   damage = base * increasedMultiplier * moreMultiplier
//
// Two +50 % lines give x2.00 as Increased and x2.25 as More. A third +50 % adds 0.5 to the pool (x2.50, a gain of 25 %
// of the damage before it) but multiplies More again (x3.375, a gain of 50 %). So an Increased line is worth less the
// more others are already in the pool, and a More line is always worth its printed percentage. `worth` measures that.
//
// The two naive readings the demo compares against are "add every percentage into one pool" (Rule::AddAll) and "multiply
// every percentage" (Rule::MultiplyAll).
//
// Reduced and Less are simply negative percentages. Nothing may go below zero: a pool of -150 % must not turn the damage
// negative, and two factors of -150 % must not multiply into a positive number. Every pool and every factor is floored at 0.
//
// The percentages in the demo (+50 %, base damage 100) are example values, not numbers of any real game.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace ds {

enum class Bucket { Increased, More };
enum class Rule { Buckets, AddAll, MultiplyAll };

struct Mod {
    double percent = 0.0;  // +50 means "50 % increased/more", -20 means "20 % reduced/less"
    Bucket bucket = Bucket::Increased;
};

inline constexpr double kMinPercent = -50.0;  // range of the mouse wheel in the demo
inline constexpr double kMaxPercent = 200.0;
inline constexpr double kPercentStep = 5.0;

inline Bucket other(Bucket b) { return b == Bucket::Increased ? Bucket::More : Bucket::Increased; }

/// Three +50 % lines in each pool: identical numbers make the difference between the pools easy to see.
inline std::vector<Mod> defaultMods() {
    return {{50, Bucket::Increased}, {50, Bucket::Increased}, {50, Bucket::Increased}, {50, Bucket::More}, {50, Bucket::More}, {50, Bucket::More}};
}

inline constexpr std::size_t kNone = static_cast<std::size_t>(-1);

namespace detail {

inline double factor(double percent) { return std::max(0.0, 1.0 + percent / 100.0); }

/// One multiplier for the list under `rule`, leaving out the line at index `skip` (kNone leaves nothing out).
inline double multiplierWithout(const std::vector<Mod>& mods, Rule rule, std::size_t skip) {
    double pool = 0.0;     // added percentages that share the pool
    double product = 1.0;  // multiplied factors
    for (std::size_t i = 0; i < mods.size(); ++i) {
        if (i == skip) continue;
        const bool added = rule == Rule::AddAll || (rule == Rule::Buckets && mods[i].bucket == Bucket::Increased);
        if (added) {
            pool += mods[i].percent;
        } else {
            product *= factor(mods[i].percent);
        }
    }
    return std::max(0.0, 1.0 + pool / 100.0) * product;
}

}  // namespace detail

/// 1 + sum of the Increased lines, floored at 0.
inline double increasedMultiplier(const std::vector<Mod>& mods) {
    double pool = 0.0;
    for (const Mod& m : mods) {
        if (m.bucket == Bucket::Increased) pool += m.percent;
    }
    return std::max(0.0, 1.0 + pool / 100.0);
}

/// Product of the More lines, each factor floored at 0.
inline double moreMultiplier(const std::vector<Mod>& mods) {
    double product = 1.0;
    for (const Mod& m : mods) {
        if (m.bucket == Bucket::More) product *= detail::factor(m.percent);
    }
    return product;
}

inline double multiplier(const std::vector<Mod>& mods, Rule rule) { return detail::multiplierWithout(mods, rule, kNone); }

inline double damage(double base, const std::vector<Mod>& mods, Rule rule) { return base * multiplier(mods, rule); }

/// Sum of the percentages in the Increased pool (what the pool header shows).
inline double sumIncreasedPercent(const std::vector<Mod>& mods) {
    double sum = 0.0;
    for (const Mod& m : mods) {
        if (m.bucket == Bucket::Increased) sum += m.percent;
    }
    return sum;
}

/// How much the damage grows because of the line at `index`, as a fraction: with / without - 1. 0.25 means "+25 % damage".
/// The base cancels out. Empty if there is no such line or the damage without it would be zero (no ratio exists then).
inline std::optional<double> worth(const std::vector<Mod>& mods, Rule rule, std::size_t index) {
    if (index >= mods.size()) return std::nullopt;
    const double without = detail::multiplierWithout(mods, rule, index);
    if (without <= 1e-12) return std::nullopt;
    return detail::multiplierWithout(mods, rule, kNone) / without - 1.0;
}

/// Mouse wheel: `notches` steps of kPercentStep, kept inside the range of the demo.
inline double stepPercent(double percent, int notches) {
    const int n = std::clamp(notches, -1000, 1000);
    return std::clamp(percent + kPercentStep * static_cast<double>(n), kMinPercent, kMaxPercent);
}

/// Frame rate independent move of `current` towards `target`: two half steps equal one whole step. Snaps when close.
inline double smooth(double current, double target, double dt, double lambda) {
    if (dt <= 0.0) return current;
    const double next = current + (target - current) * (1.0 - std::exp(-lambda * dt));
    return std::fabs(target - next) < 1e-4 ? target : next;
}

/// Fixed decimals with ',' as thousands separator and '.' as decimal point, independent of the system locale.
/// "?" for NaN, infinity and anything of 1e12 or more.
inline std::string formatFixed(double value, int decimals) {
    if (!std::isfinite(value) || std::fabs(value) >= 1e12) return "?";
    const int d = std::clamp(decimals, 0, 4);
    long long scale = 1;
    for (int i = 0; i < d; ++i) scale *= 10;
    // |value| < 1e12 and scale <= 1e4, so the product stays below 1e16 and fits a long long.
    const long long scaled = std::llround(std::fabs(value) * static_cast<double>(scale));
    const long long whole = scaled / scale;
    const long long frac = scaled % scale;

    std::string digits = std::to_string(whole);
    std::string out;
    for (std::size_t i = 0; i < digits.size(); ++i) {
        if (i > 0 && (digits.size() - i) % 3 == 0) out += ',';
        out += digits[i];
    }
    if (d > 0) {
        std::string f = std::to_string(frac);
        out += '.';
        out += std::string(static_cast<std::size_t>(d) - f.size(), '0');
        out += f;
    }
    if (value < 0.0 && scaled != 0) out.insert(out.begin(), '-');
    return out;
}

/// "+50%", "-20%", "0%" (no sign when the shown number rounds to zero).
inline std::string formatPercent(double percent, int decimals) {
    std::string s = formatFixed(percent, decimals);
    if (s == "?") return s;
    const bool zero = s.find_first_of("123456789") == std::string::npos;
    if (!zero && s.front() != '-') s.insert(s.begin(), '+');
    return s + "%";
}

}  // namespace ds
