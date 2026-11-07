// Utility AI scoring for one villager, no SDL. Each action has a few considerations (a need or a fact about the world,
// mapped through a response curve to 0..1). A combiner turns them into one score per action, the highest score wins.
// The three combiners are the point of the demo: the mean lets a zero factor hide among good ones, the product gives
// every factor a veto, and bands add a class on top so a survival action can outrank any score of an ordinary one.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <utility>

namespace ua {

enum class Action { Eat, Sleep, Work, Wander };
inline constexpr int kActionCount = 4;

enum class Combiner { Mean, Product, Bands };
enum class CurveType { Linear, Quadratic, Logistic };

inline constexpr double kStep = 0.1;          // seconds between two decisions
inline constexpr double kDayLength = 40.0;    // seconds from one noon to the next
inline constexpr double kWander = 0.15;       // the fixed score of doing nothing in particular
inline constexpr double kBandEdge = 0.6;      // a survival action reaches the top band from this score on
inline constexpr double kSteepness = 10.0;    // slope of the logistic curve
inline constexpr std::size_t kHistory = 600;  // decisions kept for the timeline, 60 s

/// NaN counts as 0, so a broken input can never leak into a score.
inline double clamp01(double x) {
    if (!(x > 0.0)) return 0.0;
    return x > 1.0 ? 1.0 : x;
}

inline constexpr int index(Action a) { return static_cast<int>(a); }

/// Response curve from a raw 0..1 input to a 0..1 score. Logistic is normalised so it still starts at 0 and ends at 1,
/// `mid` is where it is steepest.
struct Curve {
    CurveType type = CurveType::Linear;
    double mid = 0.5;
};

inline double respond(const Curve& c, double x) {
    x = clamp01(x);
    switch (c.type) {
        case CurveType::Linear: return x;
        case CurveType::Quadratic: return x * x;
        case CurveType::Logistic: {
            const double m = std::clamp(c.mid, 0.05, 0.95);
            const auto sig = [](double v) { return 1.0 / (1.0 + std::exp(-v)); };
            const double lo = sig(-kSteepness * m);
            const double hi = sig(kSteepness * (1.0 - m));
            return clamp01((sig(kSteepness * (x - m)) - lo) / (hi - lo));  // hi - lo is at least 0.6
        }
    }
    return x;
}

struct State {
    double hunger = 0.3;   // 0 full, 1 starving
    double fatigue = 0.2;  // 0 rested, 1 exhausted
    double drive = 0.6;    // desire to work
    double larder = 0.5;   // food in stock
    double clock = 5.0;    // seconds, wraps at kDayLength
};

/// 1 around noon, 0 at night, smooth in between.
inline double daylight(double clock) {
    double phase = clock / kDayLength;
    phase -= std::floor(phase);
    return clamp01(0.5 + 0.8 * std::sin(6.283185307179586 * phase));
}

struct Settings {
    Combiner combiner = Combiner::Mean;
    Curve hunger;
    Curve fatigue;
    Curve drive;
    double inertia = 0.0;  // added to the score of the action that is running
};

/// Mean and Product are the two ways to fold factors into one number. Bands scores like Product, the band only
/// matters when actions are compared (see `evaluate`).
inline double combine(Combiner c, const double* factors, int n) {
    if (n <= 0) return 0.0;
    if (c == Combiner::Mean) {
        double sum = 0.0;
        for (int i = 0; i < n; ++i) sum += clamp01(factors[i]);
        return sum / n;
    }
    double product = 1.0;
    for (int i = 0; i < n; ++i) product *= clamp01(factors[i]);
    return product;
}

struct Option {
    std::array<double, 2> factor{};
    int factors = 0;
    double score = 0.0;
    int band = 0;  // 1 = urgent survival action, 0 = everything else
};
using Evaluation = std::array<Option, kActionCount>;

inline bool survival(Action a) { return a == Action::Eat || a == Action::Sleep; }

/// Eat: hunger and food in the larder. Sleep: fatigue and darkness. Work: drive and daylight. Wander: a constant.
inline Evaluation evaluate(const State& s, const Settings& settings) {
    const double day = daylight(s.clock);
    Evaluation e{};
    e[index(Action::Eat)].factor = {respond(settings.hunger, s.hunger), clamp01(s.larder)};
    e[index(Action::Sleep)].factor = {respond(settings.fatigue, s.fatigue), 1.0 - day};
    e[index(Action::Work)].factor = {respond(settings.drive, s.drive), day};
    e[index(Action::Wander)].factor = {kWander, 0.0};
    for (int i = 0; i < kActionCount; ++i) {
        Option& o = e[static_cast<std::size_t>(i)];
        o.factors = i == index(Action::Wander) ? 1 : 2;
        o.score = combine(settings.combiner, o.factor.data(), o.factors);
        const bool urgent = settings.combiner == Combiner::Bands && survival(static_cast<Action>(i)) && o.score >= kBandEdge;
        o.band = urgent ? 1 : 0;
    }
    return e;
}

/// Highest band first, then highest score. The running action gets `inertia` on top of its score, and it also wins
/// exact ties, so equal scores never make it switch.
inline Action choose(const Evaluation& e, Action current, double inertia) {
    const auto key = [&](int i) {
        const Option& o = e[static_cast<std::size_t>(i)];
        return std::pair<int, double>{o.band, o.score + (i == index(current) ? inertia : 0.0)};
    };
    int best = index(current);
    for (int i = 0; i < kActionCount; ++i) {
        if (key(i) > key(best)) best = i;
    }
    return static_cast<Action>(best);
}

/// What the villager does to its own needs while an action runs, per second.
inline void advance(State& s, Action a, double dt) {
    double dHunger = 0.03;
    double dFatigue = 0.012;
    double dDrive = 0.03;
    double dLarder = 0.0;
    switch (a) {
        case Action::Eat:
            dHunger = -0.15;
            dLarder = -0.06;
            break;
        case Action::Sleep:
            dHunger *= 0.5;
            dFatigue = -0.06;
            break;
        case Action::Work:
            dHunger += 0.02;
            dFatigue += 0.025;
            dDrive = -0.05;
            dLarder = 0.05;
            break;
        case Action::Wander: break;
    }
    s.hunger = clamp01(s.hunger + dHunger * dt);
    s.fatigue = clamp01(s.fatigue + dFatigue * dt);
    s.drive = clamp01(s.drive + dDrive * dt);
    s.larder = clamp01(s.larder + dLarder * dt);
    s.clock = std::fmod(s.clock + dt, kDayLength);
}

struct Sample {
    Action chosen = Action::Wander;
    std::array<float, kActionCount> score{};
    float daylight = 1.0F;
};

/// The last kHistory decisions, oldest first when read through `at`.
class History {
public:
    void push(const Sample& s) {
        data_[(start_ + size_) % kHistory] = s;
        if (size_ < kHistory) {
            ++size_;
        } else {
            start_ = (start_ + 1) % kHistory;
        }
    }
    std::size_t size() const { return size_; }
    const Sample& at(std::size_t i) const { return data_[(start_ + i) % kHistory]; }

    /// How often the chosen action differs from the one before, the flicker a villager shows.
    int switches() const {
        int n = 0;
        for (std::size_t i = 1; i < size_; ++i) n += at(i).chosen != at(i - 1).chosen ? 1 : 0;
        return n;
    }

private:
    std::array<Sample, kHistory> data_{};
    std::size_t start_ = 0;
    std::size_t size_ = 0;
};

/// One villager that decides every kStep seconds.
struct Sim {
    State state;
    Settings settings;
    Action current = Action::Wander;
    History history;
    double pending = 0.0;

    void tick() {
        const Evaluation e = evaluate(state, settings);
        current = choose(e, current, settings.inertia);
        Sample s;
        s.chosen = current;
        for (int i = 0; i < kActionCount; ++i) s.score[static_cast<std::size_t>(i)] = static_cast<float>(e[static_cast<std::size_t>(i)].score);
        s.daylight = static_cast<float>(daylight(state.clock));
        history.push(s);
        advance(state, current, kStep);
    }

    /// Runs as many whole steps as `dt` allows and keeps the rest. A long frame is clamped, so a hidden tab never
    /// makes the villager live through minutes at once.
    void update(double dt) {
        pending += std::clamp(dt, 0.0, 0.25);
        int steps = 0;
        while (pending >= kStep && steps < 8) {
            tick();
            pending -= kStep;
            ++steps;
        }
    }
};

/// Night, hungry, rested, the larder almost empty: only the mean still wants to eat.
inline State emptyLarderAtNight() { return State{1.0, 0.0, 1.0, 0.1, 30.0}; }
/// Noon, starving, but a drive of 1: a plain score for work beats a hungry stomach unless bands step in.
inline State eagerButStarving() { return State{0.95, 0.2, 1.0, 0.9, 10.0}; }

}  // namespace ua
