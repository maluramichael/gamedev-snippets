// Four ways to step the same equation of motion, on two systems whose true energy never changes.
//
// Orbit: a planet on a circular orbit around a fixed sun (a = -GM * p / |p|^3). Spring: a mass on a spring
// (a = -w^2 * p). Both conserve energy exactly in real life, so the energy a method reports after N steps is a direct
// measure of what the method does to the physics. No SDL in here, main.cpp only draws it.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <deque>
#include <string>

namespace im {

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};

inline Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
inline Vec2 operator*(Vec2 a, double s) { return {a.x * s, a.y * s}; }
inline double dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
inline double length(Vec2 a) { return std::sqrt(dot(a, a)); }
inline bool finite(Vec2 a) { return std::isfinite(a.x) && std::isfinite(a.y); }

enum class Method { Explicit, SemiImplicit, Verlet, RK4 };
inline constexpr std::size_t kMethods = 4;

enum class Kind { Orbit, Spring };

inline constexpr double kGM = 1.0;                // sun strength, a circular orbit of radius 1 has speed 1 and period 2 pi
inline constexpr double kOmega = 2.0;             // spring: w^2 = k / m, period pi
inline constexpr double kMinDistSq = 1e-12;       // a pass through the sun must not divide by zero
inline constexpr double kEscape = 1e4;            // farther away than this, a body counts as flown away and is frozen
inline constexpr double kTimeScale = 2.0;         // simulated seconds per real second
inline constexpr double kMaxFrameSeconds = 0.25;  // a long frame (tab in the background) must not become a burst of steps
inline constexpr int kMaxStepsPerFrame = 64;
inline constexpr double kTrailWindow = 7.0;       // seconds of orbit trail kept
inline constexpr double kPlotWindow = 20.0;       // seconds of energy history kept
inline constexpr double kPlotGap = 0.05;          // at most one energy sample per this many simulated seconds
inline constexpr double kGoneDrift = 1e6;         // drift value a flown-away body plots (the graph clamps it to the top)

inline Vec2 accel(Kind kind, Vec2 p) {
    if (kind == Kind::Spring) return p * (-kOmega * kOmega);
    const double r2 = std::max(dot(p, p), kMinDistSq);
    return p * (-kGM / (r2 * std::sqrt(r2)));
}

struct Body {
    Vec2 pos;
    Vec2 vel;   // Verlet does not read it, it is derived from the last two positions after every step
    Vec2 prev;  // Verlet only: where the body was one step ago
    bool gone = false;
};

/// Start state: the planet on a circular orbit at (1, 0), the spring stretched by 1 and at rest.
/// `prev` is the position one step back, worked out from a Taylor expansion so Verlet starts consistently.
inline Body start(Kind kind, double dt) {
    Body b;
    b.pos = {1.0, 0.0};
    b.vel = kind == Kind::Orbit ? Vec2{0.0, 1.0} : Vec2{0.0, 0.0};
    b.prev = b.pos - b.vel * dt + accel(kind, b.pos) * (0.5 * dt * dt);
    return b;
}

inline void step(Kind kind, Method method, double dt, Body& b) {
    if (b.gone || !(dt > 0.0)) return;
    switch (method) {
        case Method::Explicit: {
            // The textbook version: the new position uses the OLD velocity.
            const Vec2 a = accel(kind, b.pos);
            b.pos = b.pos + b.vel * dt;
            b.vel = b.vel + a * dt;
            break;
        }
        case Method::SemiImplicit:
            // The same two lines in the other order: the new position uses the NEW velocity.
            b.vel = b.vel + accel(kind, b.pos) * dt;
            b.pos = b.pos + b.vel * dt;
            break;
        case Method::Verlet: {
            const Vec2 next = b.pos * 2.0 - b.prev + accel(kind, b.pos) * (dt * dt);
            b.prev = b.pos;
            b.pos = next;
            // Velocity at the new position from a Taylor step back: (x - x_prev) / dt + a * dt / 2.
            b.vel = (b.pos - b.prev) * (1.0 / dt) + accel(kind, b.pos) * (0.5 * dt);
            break;
        }
        case Method::RK4: {
            const Vec2 k1x = b.vel;
            const Vec2 k1v = accel(kind, b.pos);
            const Vec2 k2x = b.vel + k1v * (0.5 * dt);
            const Vec2 k2v = accel(kind, b.pos + k1x * (0.5 * dt));
            const Vec2 k3x = b.vel + k2v * (0.5 * dt);
            const Vec2 k3v = accel(kind, b.pos + k2x * (0.5 * dt));
            const Vec2 k4x = b.vel + k3v * dt;
            const Vec2 k4v = accel(kind, b.pos + k3x * dt);
            b.pos = b.pos + (k1x + k2x * 2.0 + k3x * 2.0 + k4x) * (dt / 6.0);
            b.vel = b.vel + (k1v + k2v * 2.0 + k3v * 2.0 + k4v) * (dt / 6.0);
            break;
        }
    }
    if (!finite(b.pos) || !finite(b.vel) || length(b.pos) > kEscape) b.gone = true;
}

inline double energy(Kind kind, const Body& b) {
    const double kinetic = 0.5 * dot(b.vel, b.vel);
    if (kind == Kind::Spring) return kinetic + 0.5 * kOmega * kOmega * dot(b.pos, b.pos);
    return kinetic - kGM / std::sqrt(std::max(dot(b.pos, b.pos), kMinDistSq));
}

/// Time-stamped samples, oldest first. `minGap` thins out dense sampling, `prune` drops what left the window.
template <typename T>
struct Series {
    struct Item {
        double t;
        T value;
    };
    std::deque<Item> items;

    void push(double t, const T& value, double minGap) {
        if (!items.empty() && t - items.back().t < minGap - 1e-9) return;
        items.push_back({t, value});
    }
    void prune(double now, double window) {
        while (!items.empty() && items.front().t < now - window) items.pop_front();
    }
    void clear() { items.clear(); }
};

/// One system with all four methods side by side, same start, same dt.
struct Run {
    Kind kind = Kind::Orbit;
    double dt = 0.1;
    long steps = 0;
    double e0 = 0.0;
    std::array<Body, kMethods> body{};
    std::array<Series<Vec2>, kMethods> trail;
    std::array<Series<double>, kMethods> drift;

    double time() const { return static_cast<double>(steps) * dt; }  // multiplied, not summed: no drift in the clock

    /// Relative energy error (E - E0) / |E0|. 0 = perfect, +1 on the orbit = the planet just escaped.
    double driftOf(std::size_t m) const { return body[m].gone ? kGoneDrift : (energy(kind, body[m]) - e0) / std::fabs(e0); }

    void restart(Kind k, double newDt) {
        kind = k;
        dt = newDt;
        steps = 0;
        const Body first = start(kind, dt);
        e0 = energy(kind, first);
        for (std::size_t m = 0; m < kMethods; ++m) {
            body[m] = first;
            trail[m].clear();
            drift[m].clear();
            trail[m].push(0.0, first.pos, 0.0);
            drift[m].push(0.0, 0.0, kPlotGap);
        }
    }

    void step() {
        for (std::size_t m = 0; m < kMethods; ++m) im::step(kind, static_cast<Method>(m), dt, body[m]);
        ++steps;
        const double t = time();
        for (std::size_t m = 0; m < kMethods; ++m) {
            if (!body[m].gone) trail[m].push(t, body[m].pos, 0.0);
            drift[m].push(t, driftOf(m), kPlotGap);
            trail[m].prune(t, kTrailWindow);
            drift[m].prune(t, kPlotWindow);
        }
    }
};

/// Whole steps of `dt` that fit into the accumulator after adding `add`. The remainder stays in `acc`, the number of
/// steps is capped, and the backlog beyond the cap is dropped instead of being paid back in the next frame.
inline int takeSteps(double& acc, double add, double dt, int maxSteps) {
    if (!(dt > 0.0)) return 0;
    if (add > 0.0) acc += add;  // negative and NaN input are ignored
    const double whole = std::floor(acc / dt + 1e-9);
    if (whole < 1.0) return 0;
    acc = std::max(0.0, acc - whole * dt);
    return static_cast<int>(std::min(whole, static_cast<double>(maxSteps)));
}

/// Both systems in lockstep.
struct World {
    Run orbit;
    Run spring;
    double dt = 0.1;
    double acc = 0.0;

    void restart(double newDt) {
        dt = newDt;
        acc = 0.0;  // otherwise the first step after a restart comes too early
        orbit.restart(Kind::Orbit, dt);
        spring.restart(Kind::Spring, dt);
    }

    void stepOnce() {
        orbit.step();
        spring.step();
    }

    /// Advance by `seconds` of real time. Returns how many steps ran.
    int advance(double seconds) {
        const double clamped = std::clamp(std::isfinite(seconds) ? seconds : 0.0, 0.0, kMaxFrameSeconds);
        const int n = takeSteps(acc, clamped * kTimeScale, dt, kMaxStepsPerFrame);
        for (int i = 0; i < n; ++i) stepOnce();
        return n;
    }

    /// Run `seconds` of simulated time at once (the first frame already shows a developed state).
    void warmUp(double seconds) {
        const long n = std::lround(seconds / dt);
        for (long i = 0; i < n; ++i) stepOnce();
    }
};

/// Drift on a signed log scale for the graph: 0 -> 0, +-1 (100 %) -> +-1, each decade is the same height.
/// Below 0.01 % the scale turns linear so the zero line stays exact.
inline constexpr double kSymlogLinear = 1e-4;
inline double symlog(double drift) {
    if (std::isnan(drift)) return 0.0;
    const double scale = std::log10(1.0 + 1.0 / kSymlogLinear);
    const double f = std::min(std::log10(1.0 + std::fabs(drift) / kSymlogLinear) / scale, 1.0);
    return drift < 0.0 ? -f : f;
}

/// "+0.37%", "-12.5%", ">+1000%". Built by hand so the decimal separator never depends on the locale.
inline std::string formatDrift(double drift) {
    if (!std::isfinite(drift)) return "gone";
    const double percent = drift * 100.0;
    const double a = std::fabs(percent);
    if (a >= 1000.0) return percent > 0.0 ? ">+1000%" : "<-1000%";
    const bool twoDecimals = a < 10.0;
    const long scale = twoDecimals ? 100 : 10;
    const long scaled = std::lround(a * static_cast<double>(scale));
    const long whole = scaled / scale;
    const long frac = scaled % scale;
    std::string out = (scaled != 0 && percent < 0.0) ? "-" : "+";
    out += std::to_string(whole) + ".";
    if (twoDecimals && frac < 10) out += "0";
    out += std::to_string(frac) + "%";
    return out;
}

}  // namespace im
