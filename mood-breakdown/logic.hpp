// Mood and mental breaks for a row of eight colonists, no SDL. Everything that can be wrong lives here.
//
// Each colonist has a mood bar (0..100) that chases a target. The target is the sum of a base value, a personal offset and
// the thoughts that are currently active. The bar moves toward the target by at most +12 per hour up and -8 per hour down
// (numbers from the RimWorld wiki, not checked against the game files). Below three thresholds a break becomes possible,
// at a constant rate per hour: a mean time of 4 days, 0.8 days and 0.5 days. In the Dwarf Fortress model a second,
// slow counter has to be high as well before anybody breaks. The chain reaction (neighbours who see a break lose mood)
// and all thought values are made up for the demo.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

namespace mb {

inline constexpr int kColonists = 8;
inline constexpr double kRiseRate = 12.0;       // mood points per hour, upward
inline constexpr double kFallRate = 8.0;        // mood points per hour, downward
inline constexpr double kDefaultMinor = 35.0;   // minor break threshold of a plain colonist
inline constexpr double kBreakHours = 8.0;      // how long a break lasts (demo value)
inline constexpr double kLongRise = 1.0;        // long-term stress per hour at the worst mood
inline constexpr double kLongFall = kLongRise * (43564.0 / 20160.0);  // recovery is 2.16 times faster, like the DF maxima
inline constexpr double kLongGate = 30.0;       // the long-term counter must reach this before a break is possible
inline constexpr double kDefaultBase = 32.0;

enum class Severity { None, Minor, Major, Extreme };
enum class Model { Rim, Dwarf };
enum class Thought { Corpse, BadMeal, Funeral, Witnessed, Catharsis };
enum class Event { Corpse, BadMeal, Funeral };
inline constexpr int kThoughtKinds = 5;

struct ThoughtDef {
    const char* name;
    double value;  // mood points while active
    double hours;  // how long it lasts
};

inline constexpr std::array<ThoughtDef, kThoughtKinds> kThoughts{{
    {"Corpse", -10.0, 12.0},
    {"Bad meal", -6.0, 10.0},
    {"Funeral", 12.0, 24.0},
    {"Witnessed", -14.0, 12.0},
    {"Catharsis", 40.0, 72.0},  // +40 for 3 days after a break, as the wiki says
}};

inline constexpr int index(Thought t) { return static_cast<int>(t); }

/// NaN counts as 0, so a broken input can never leak into the bar.
inline double clampMood(double x) {
    if (!(x > 0.0)) return 0.0;
    return x > 100.0 ? 100.0 : x;
}

struct Thresholds {
    double minor;
    double major;
    double extreme;
};

/// Major is 4/7 and extreme 1/7 of the minor threshold, which gives 35, 20 and 5 for the default. Minor is capped at 1..50.
inline Thresholds thresholds(double minor) {
    if (std::isnan(minor)) minor = kDefaultMinor;
    minor = std::clamp(minor, 1.0, 50.0);
    return {minor, minor * 4.0 / 7.0, minor / 7.0};
}

/// A mood exactly on a line is not below it.
inline Severity severityOf(double mood, const Thresholds& t) {
    if (mood < t.extreme) return Severity::Extreme;
    if (mood < t.major) return Severity::Major;
    if (mood < t.minor) return Severity::Minor;
    return Severity::None;
}

/// Mean time between breaks in hours at a severity. Infinite when there is no risk.
inline double mtbHours(Severity s) {
    switch (s) {
        case Severity::Minor: return 4.0 * 24.0;
        case Severity::Major: return 0.8 * 24.0;
        case Severity::Extreme: return 0.5 * 24.0;
        case Severity::None: break;
    }
    return std::numeric_limits<double>::infinity();
}

/// Chance of a break within `hours` at a constant rate of 1/MTB. Exactly additive over many small steps.
inline double breakChance(Severity s, double hours) {
    if (s == Severity::None || !(hours > 0.0)) return 0.0;
    return -std::expm1(-hours / mtbHours(s));  // mtbHours is at least 12 here
}

/// One step of the bar toward the target with the two speed limits.
inline double stepMood(double mood, double target, double hours) {
    mood = clampMood(mood);
    if (!(hours > 0.0)) return mood;
    target = clampMood(target);
    if (target > mood) return std::min(target, mood + kRiseRate * hours);
    return std::max(target, mood - kFallRate * hours);
}

/// The fast counter of the Dwarf Fortress model is just the bad end of the mood bar.
inline double shortStress(double mood) { return 100.0 - clampMood(mood); }

/// The slow counter: it climbs while short-term stress is above 50, falls while it is below, and falls faster than it climbs.
inline double stepLong(double longStress, double mood, double hours) {
    if (!(hours > 0.0)) return longStress;
    const double rate = (shortStress(mood) - 50.0) / 50.0;  // -1 .. +1
    const double perHour = rate > 0.0 ? rate * kLongRise : rate * kLongFall;
    return std::clamp(longStress + perHour * hours, 0.0, 100.0);
}

/// Which severity can actually strike. In the Dwarf Fortress model both counters must be high.
inline Severity riskOf(Model model, double mood, double longStress, const Thresholds& t) {
    const Severity s = severityOf(mood, t);
    if (model == Model::Dwarf && !(longStress >= kLongGate)) return Severity::None;
    return s;
}

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

struct Colonist {
    double trait = 0.0;                    // personal offset on the target
    double minorThreshold = kDefaultMinor;
    double mood = 45.0;
    double longStress = 0.0;
    std::array<double, kThoughtKinds> left{};  // hours left per thought, 0 = not active
    bool broken = false;
    double brokenLeft = 0.0;
    Severity severity = Severity::None;
};

struct BreakLog {
    double time = 0.0;
    int who = -1;
    Severity severity = Severity::None;
    bool chained = false;
};

inline constexpr int kLogLines = 3;

struct Sim {
    std::array<Colonist, kColonists> c{};
    double base = kDefaultBase;
    Model model = Model::Rim;
    bool chain = true;  // do neighbours lose mood when they see a break
    double clock = 0.0;
    int breaks = 0;
    int chained = 0;
    std::array<BreakLog, kLogLines> recent{};  // newest first
    Rng rng;
};

inline Sim makeSim(std::uint64_t seed) {
    Sim s;
    s.rng.state = seed;
    constexpr std::array<double, kColonists> traits{6.0, 0.0, -4.0, 2.0, -8.0, 4.0, -2.0, -6.0};
    constexpr std::array<double, kColonists> minors{35.0, 35.0, 35.0, 35.0, 35.0, 25.0, 35.0, 45.0};
    for (int i = 0; i < kColonists; ++i) {
        s.c[i].trait = traits[i];
        s.c[i].minorThreshold = minors[i];
    }
    return s;
}

inline void addThought(Colonist& c, Thought t) { c.left[index(t)] = kThoughts[index(t)].hours; }

inline double targetOf(const Sim& s, const Colonist& c) {
    double sum = s.base + c.trait;
    for (int k = 0; k < kThoughtKinds; ++k) {
        if (c.left[k] > 0.0) sum += kThoughts[k].value;
    }
    return clampMood(sum);
}

inline void applyEvent(Sim& s, Event e, int selected) {
    switch (e) {
        case Event::Corpse:
            for (Colonist& c : s.c) addThought(c, Thought::Corpse);
            break;
        case Event::Funeral:
            for (Colonist& c : s.c) addThought(c, Thought::Funeral);
            break;
        case Event::BadMeal:
            if (selected >= 0 && selected < kColonists) addThought(s.c[selected], Thought::BadMeal);
            break;
    }
}

inline void startBreak(Sim& s, int who, Severity severity) {
    if (who < 0 || who >= kColonists || s.c[who].broken) return;
    Colonist& c = s.c[who];
    const bool chained = c.left[index(Thought::Witnessed)] > 0.0;
    c.broken = true;
    c.brokenLeft = kBreakHours;
    c.severity = severity;
    ++s.breaks;
    if (chained) ++s.chained;
    for (int k = kLogLines - 1; k > 0; --k) s.recent[k] = s.recent[k - 1];
    s.recent[0] = {s.clock, who, severity, chained};
    if (!s.chain) return;
    for (const int j : {who - 1, who + 1}) {
        if (j >= 0 && j < kColonists) addThought(s.c[j], Thought::Witnessed);
    }
}

/// Advances the colony by `hours` of game time. Call it with small steps (0.05 h).
inline void step(Sim& s, double hours) {
    if (!(hours > 0.0)) return;
    s.clock += hours;
    for (int i = 0; i < kColonists; ++i) {
        Colonist& c = s.c[i];
        c.mood = stepMood(c.mood, targetOf(s, c), hours);
        c.longStress = stepLong(c.longStress, c.mood, hours);
        for (double& left : c.left) left = std::max(0.0, left - hours);
        if (c.broken) {
            c.brokenLeft -= hours;
            if (c.brokenLeft <= 0.0) {
                c.broken = false;
                c.brokenLeft = 0.0;
                c.severity = Severity::None;
                addThought(c, Thought::Catharsis);
            }
            continue;
        }
        const Severity risk = riskOf(s.model, c.mood, c.longStress, thresholds(c.minorThreshold));
        if (risk != Severity::None && s.rng.unit() < breakChance(risk, hours)) startBreak(s, i, risk);
    }
}

}  // namespace mb
