// Game feel ("juice") without SDL: the small reactions that make a hit land, and the rules that keep them honest.
//
// Time lives on two clocks. REAL time keeps running during a hit stop: camera shake, the flash, the input buffer.
// GAME time is what the hit stop pauses: the swing, the dummy's squash, the sparks. `Clock` turns real time into game
// time. The camera shake uses the trauma model: hits add trauma (0..1), it drains linearly, and the offset follows
// trauma squared, so small hits stay small and stacked hits ramp up.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>

namespace jc {

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};

inline double lerp(double a, double b, double t) { return a + (b - a) * t; }
inline double clamp01(double v) { return std::clamp(v, 0.0, 1.0); }

// ---- hit stop ----

inline constexpr double kMaxStop = 0.25;  // a freeze longer than this is a hang, not a hit

struct Clock {
    double stopRemaining = 0.0;

    bool stopped() const { return stopRemaining > 0.0; }

    /// Freezes game time for `seconds`. Overlapping requests do not add up, the longer one wins.
    void requestStop(double seconds) {
        if (seconds <= 0.0) return;
        stopRemaining = std::max(stopRemaining, std::min(seconds, kMaxStop));
    }

    /// Real time passes by `realDt`, returns how much of it is game time. A freeze eats real time first.
    double advance(double realDt) {
        if (realDt <= 0.0) return 0.0;
        if (stopRemaining >= realDt) {
            stopRemaining -= realDt;
            return 0.0;
        }
        const double rest = realDt - stopRemaining;
        stopRemaining = 0.0;
        return rest;
    }
};

// ---- input ----

inline constexpr int kLightKind = 0;
inline constexpr int kHeavyKind = 1;

/// Remembers the latest press for a short while, so a click during a swing or a freeze is not lost.
struct InputBuffer {
    bool pending = false;
    double pressedAt = 0.0;
    int kind = kLightKind;

    void press(double now, int pressKind) {
        pending = true;
        pressedAt = now;
        kind = pressKind;
    }

    /// The buffered press if it is at most `window` old. Either way the buffer is empty afterwards.
    std::optional<int> take(double now, double window) {
        if (!pending) return std::nullopt;
        pending = false;
        if (now - pressedAt <= window) return kind;
        return std::nullopt;
    }
};

enum class InputPolicy { Buffered, Dropped };

/// Buffered is the right way: the freeze pauses the game, not the player. Dropped is what you get when the whole
/// loop (or the event handling) stands still, a click during the freeze is simply gone.
inline bool acceptsInput(bool stopped, InputPolicy policy) { return policy == InputPolicy::Buffered || !stopped; }

// ---- attack timing (game time) ----

enum class Phase { Idle, Windup, Recovery };

struct Swing {
    double windup = 0.0;
    double recovery = 0.0;
};

struct Attack {
    Phase phase = Phase::Idle;
    double t = 0.0;  // game seconds inside the current phase
    Swing swing{};

    bool ready() const { return phase == Phase::Idle; }

    bool start(Swing s) {
        if (!ready()) return false;
        swing = s;
        phase = Phase::Windup;
        t = 0.0;
        return true;
    }

    /// Advances by game time. True exactly once per attack: in the step that finishes the windup.
    bool advance(double dt) {
        if (phase == Phase::Idle || dt <= 0.0) return false;
        t += dt;
        bool hit = false;
        if (phase == Phase::Windup && t >= swing.windup) {
            hit = true;
            t -= swing.windup;
            phase = Phase::Recovery;
        }
        if (phase == Phase::Recovery && t >= swing.recovery) {
            phase = Phase::Idle;
            t = 0.0;
        }
        return hit;
    }

    /// 0..1 inside the current phase.
    double progress() const {
        const double length = phase == Phase::Windup ? swing.windup : swing.recovery;
        if (phase == Phase::Idle) return 0.0;
        return length > 0.0 ? clamp01(t / length) : 1.0;
    }
};

// ---- the two kinds of hit ----

struct Weight {
    double windup;
    double recovery;
    double stop;    // hit stop in seconds
    double trauma;  // added to the camera
    double squash;  // squeeze of the dummy at the hit
    double kick;    // px the dummy is pushed
    int sparks;
    int damage;
};

// One flash per hit and a hit takes windup + stop + recovery at the very least: 0.42 s and 0.64 s, below 3 per second.
inline constexpr Weight kLight{0.10, 0.26, 0.06, 0.45, 0.18, 10.0, 9, 8};
inline constexpr Weight kHeavy{0.18, 0.34, 0.12, 0.80, 0.32, 26.0, 22, 20};

inline constexpr double kOverdoStopFactor = 2.0;
inline constexpr double kRoutineTrauma = 0.5;  // overdo: even a plain swing shakes the screen

// ---- trauma and shake ----

inline constexpr double kShakeFrequency = 18.0;  // lattice points of the noise per second

inline double addTrauma(double trauma, double amount) { return std::clamp(trauma + amount, 0.0, 1.0); }

inline double decayTrauma(double trauma, double dt, double perSecond) {
    if (dt <= 0.0) return trauma;
    return std::max(0.0, trauma - perSecond * dt);
}

inline double shakeStrength(double trauma) {
    const double t = clamp01(trauma);
    return t * t;
}

inline std::uint32_t hash32(std::uint32_t x) {
    x ^= x >> 16;
    x *= 0x7FEB352DU;
    x ^= x >> 15;
    x *= 0x846CA68BU;
    x ^= x >> 16;
    return x;
}

/// Random value in [-1, 1] at an integer position, the same every time.
inline double lattice(int seed, std::int64_t i) {
    const std::uint32_t h = hash32(static_cast<std::uint32_t>(seed) * 0x9E3779B1U + hash32(static_cast<std::uint32_t>(i)));
    return static_cast<double>(h) / 4294967295.0 * 2.0 - 1.0;
}

/// Smooth 1D value noise in [-1, 1]. Smooth matters: white noise per frame looks like a broken monitor, not a shake.
inline double noise1(int seed, double x) {
    const double fl = std::floor(x);
    const double f = x - fl;
    const double s = f * f * (3.0 - 2.0 * f);
    const auto i = static_cast<std::int64_t>(fl);
    return lerp(lattice(seed, i), lattice(seed, i + 1), s);
}

inline Vec2 shakeOffset(double trauma, double time, double maxPx) {
    const double amount = shakeStrength(trauma) * maxPx;
    if (amount <= 0.0) return {};
    return {amount * noise1(1, time * kShakeFrequency), amount * noise1(2, time * kShakeFrequency)};
}

// ---- flash, squash, kick ----

/// 1 at the hit, 0 after `duration`, eased so the white drops fast and fades out softly.
inline double flashAmount(double age, double duration) {
    if (duration <= 0.0 || age < 0.0 || age >= duration) return 0.0;
    const double u = 1.0 - age / duration;
    return u * u;
}

struct Scale {
    double x = 1.0;
    double y = 1.0;
};

inline constexpr double kSquashRate = 14.0;  // decay per second
inline constexpr double kSquashOmega = 25.132741228718345;  // 2 pi * 4 Hz
inline constexpr double kSquashHalfPeriod = 0.125;          // pi / omega

/// Squeezed along x at the hit, rings back past 1 and settles. The area stays 1 (y = 1 / x).
inline Scale squash(double age, double strength) {
    const double s = std::clamp(strength, 0.0, 0.6);
    if (age < 0.0 || s == 0.0) return {};
    const double w = s * std::exp(-kSquashRate * age) * std::cos(kSquashOmega * age);
    const double x = 1.0 - w;
    return {x, 1.0 / x};
}

/// The dummy is pushed away at the hit and glides back.
inline double kick(double age, double distance) {
    if (age < 0.0) return 0.0;
    return distance * std::exp(-9.0 * age);
}

// ---- hero pose ----

inline constexpr double kRestAngle = 20.0;     // degrees from straight up, positive towards the dummy
inline constexpr double kWindupAngle = -70.0;
inline constexpr double kStrikeAngle = 110.0;
inline constexpr double kPullBack = 10.0;
inline constexpr double kLunge = 36.0;

struct Pose {
    double angle = kRestAngle;
    double dx = 0.0;
};

/// The windup raises the weapon, the first frame of the recovery is the strike, then it settles back.
inline Pose pose(Phase phase, double progress) {
    const double u = clamp01(progress);
    switch (phase) {
        case Phase::Windup: {
            const double e = 1.0 - (1.0 - u) * (1.0 - u);
            return {lerp(kRestAngle, kWindupAngle, e), lerp(0.0, -kPullBack, e)};
        }
        case Phase::Recovery: {
            const double e = u * u * (3.0 - 2.0 * u);
            return {lerp(kStrikeAngle, kRestAngle, e), kLunge * (1.0 - e)};
        }
        case Phase::Idle: break;
    }
    return {};
}

// ---- particles ----

inline constexpr std::size_t kMaxParticles = 256;

struct Rng {
    std::uint32_t state;

    /// xorshift32, [0, 1). Deterministic, so the same hit always sprays the same way in tests.
    double next() {
        if (state == 0) state = 0x9E3779B9U;
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        return static_cast<double>(state >> 8) / 16777216.0;
    }
};

struct Particle {
    Vec2 pos;
    Vec2 vel;
    double life;
    double maxLife;
};

/// Adds up to `count` particles flying around `dir` (within +-`spread` radians) at half to full `speed`.
inline void burst(std::vector<Particle>& out, Rng& rng, Vec2 origin, Vec2 dir, int count, double speed, double spread, double life) {
    const double len = std::hypot(dir.x, dir.y);
    const double base = len > 1e-9 ? std::atan2(dir.y, dir.x) : 0.0;
    for (int i = 0; i < count && out.size() < kMaxParticles; ++i) {
        const double a = base + (rng.next() * 2.0 - 1.0) * spread;
        const double v = speed * (0.5 + 0.5 * rng.next());
        const double l = life * (0.5 + 0.5 * rng.next());
        out.push_back({origin, {std::cos(a) * v, std::sin(a) * v}, l, l});
    }
}

inline void stepParticles(std::vector<Particle>& ps, double dt, double gravity) {
    if (dt <= 0.0) return;
    for (Particle& p : ps) {
        p.life -= dt;
        p.vel.y += gravity * dt;
        p.pos.x += p.vel.x * dt;
        p.pos.y += p.vel.y * dt;
    }
    ps.erase(std::remove_if(ps.begin(), ps.end(), [](const Particle& p) { return p.life <= 0.0; }), ps.end());
}

// ---- damage and the reaction switches ----

inline int takeDamage(int hp, int damage) { return std::max(0, hp - std::max(0, damage)); }

struct Effects {
    bool stop = true;
    bool shake = true;
    bool flash = true;
    bool squash = true;
    bool sparks = true;
    bool impact = true;
    bool number = true;
};

inline int effectCount(const Effects& e) {
    return int(e.stop) + int(e.shake) + int(e.flash) + int(e.squash) + int(e.sparks) + int(e.impact) + int(e.number);
}

/// Reduced motion keeps the pause and the still markers, and drops everything that shakes, flashes or flies.
inline Effects withReducedMotion(Effects e, bool reduced) {
    if (reduced) e.shake = e.flash = e.squash = e.sparks = false;
    return e;
}

struct Reaction {
    double stop = 0.0;
    double trauma = 0.0;
    bool flash = false;
    double squash = 0.0;
    double kick = 0.0;
    int sparks = 0;
    bool impact = false;
    bool number = false;
};

/// What one hit does, given the switches. The damage itself does not depend on any of them.
inline Reaction resolveHit(const Weight& w, bool overdo, const Effects& fx) {
    Reaction r;
    if (fx.stop) r.stop = overdo ? std::min(w.stop * kOverdoStopFactor, kMaxStop) : w.stop;
    if (fx.shake) r.trauma = overdo ? 1.0 : w.trauma;
    r.flash = fx.flash;
    if (fx.squash) {
        r.squash = w.squash;
        r.kick = w.kick;
    }
    if (fx.sparks) r.sparks = w.sparks;
    r.impact = fx.impact;
    r.number = fx.number;
    return r;
}

/// Trauma for a swing that hits nothing yet. Zero unless you overdo it.
inline double routineTrauma(bool overdo, const Effects& fx) { return overdo && fx.shake ? kRoutineTrauma : 0.0; }

// ---- timeline ----

struct Sample {
    double t;
    bool stopped;
};

inline void trimHistory(std::vector<Sample>& h, double now, double span) {
    const double from = now - std::max(span, 0.0);
    h.erase(h.begin(), std::find_if(h.begin(), h.end(), [from](const Sample& s) { return s.t >= from; }));
}

}  // namespace jc
