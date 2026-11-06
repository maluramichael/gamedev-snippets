// Easing and tweening: the logic of the demo, no SDL, so test.cpp can check it.
//
// An easing function bends TIME, not the path. It takes the linear progress t in [0,1] and returns how far along the
// path the object should be. The path itself is a plain lerp: position = a + (b - a) * ease(t). So f(0) = 0 and
// f(1) = 1 for every function, and the value in between may leave [0,1] (back, elastic) which overshoots the target.
//
// Every family comes in three variants:
//   In     starts slowly, ends fast          f(t)
//   Out    starts fast, ends slowly          1 - in(1 - t)            (the mirror image of In)
//   InOut  both: slow, fast, slow            in(2t) / 2, then mirrored (point symmetric around (0.5, 0.5))
// Back and elastic use their own constants for InOut (c2, c5), so their InOut is symmetric but not exactly in(2t)/2.
//
// The formulas and constants are the ones from Robert Penner's easing equations (the common set, as listed on
// easings.net). Functions expect t in [0,1], Tween::progress() guarantees that. Outside that range expo, circ and the
// springs return garbage.
//
// Tween<T> owns a value range, a duration, an easing function and a clock that you feed with the frame's delta time.
// update(dt) returns the time that was NOT used, which is what makes chaining work: a tween that ends in the middle
// of a frame hands the rest of that frame to the next one.
#pragma once

#include <algorithm>
#include <cmath>

namespace ease {

inline constexpr double kPi = 3.14159265358979323846;

// ---------------------------------------------------------------------------------------------------------------
// Easing functions
// ---------------------------------------------------------------------------------------------------------------

using Fn = double (*)(double);

namespace detail {

inline double ipow(double x, int n) {
    double r = 1.0;
    for (int i = 0; i < n; ++i) r *= x;
    return r;
}

inline double polyIn(double t, int n) { return ipow(t, n); }
inline double polyOut(double t, int n) { return 1.0 - ipow(1.0 - t, n); }
inline double polyInOut(double t, int n) {
    // 2^(n-1) * t^n for the first half, then the mirror image
    return t < 0.5 ? ipow(2.0, n - 1) * ipow(t, n) : 1.0 - ipow(-2.0 * t + 2.0, n) / 2.0;
}

inline constexpr double kBackC1 = 1.70158;           // Penner's "s": about 10 percent overshoot
inline constexpr double kBackC2 = kBackC1 * 1.525;   // stronger for the inOut version, because both halves share the swing
inline constexpr double kBackC3 = kBackC1 + 1.0;
inline constexpr double kElasticC4 = 2.0 * kPi / 3.0;  // one sine period per third of a unit
inline constexpr double kElasticC5 = 2.0 * kPi / 4.5;  // inOut: a longer period so both halves fit
inline constexpr double kBounceN1 = 7.5625;            // 2.75^2: the first parabola is exactly 1 at t = 1 / 2.75
inline constexpr double kBounceD1 = 2.75;

}  // namespace detail

inline double linear(double t) { return t; }

inline double sineIn(double t) { return 1.0 - std::cos(t * kPi / 2.0); }
inline double sineOut(double t) { return std::sin(t * kPi / 2.0); }
inline double sineInOut(double t) { return -(std::cos(kPi * t) - 1.0) / 2.0; }

inline double quadIn(double t) { return detail::polyIn(t, 2); }
inline double quadOut(double t) { return detail::polyOut(t, 2); }
inline double quadInOut(double t) { return detail::polyInOut(t, 2); }

inline double cubicIn(double t) { return detail::polyIn(t, 3); }
inline double cubicOut(double t) { return detail::polyOut(t, 3); }
inline double cubicInOut(double t) { return detail::polyInOut(t, 3); }

inline double quartIn(double t) { return detail::polyIn(t, 4); }
inline double quartOut(double t) { return detail::polyOut(t, 4); }
inline double quartInOut(double t) { return detail::polyInOut(t, 4); }

inline double quintIn(double t) { return detail::polyIn(t, 5); }
inline double quintOut(double t) { return detail::polyOut(t, 5); }
inline double quintInOut(double t) { return detail::polyInOut(t, 5); }

// 2^(10t - 10) is 0.000977 at t = 0, not 0. The special cases at the ends make f(0) = 0 and f(1) = 1 exact.
inline double expoIn(double t) { return t <= 0.0 ? 0.0 : std::pow(2.0, 10.0 * t - 10.0); }
inline double expoOut(double t) { return t >= 1.0 ? 1.0 : 1.0 - std::pow(2.0, -10.0 * t); }
inline double expoInOut(double t) {
    if (t <= 0.0) return 0.0;
    if (t >= 1.0) return 1.0;
    return t < 0.5 ? std::pow(2.0, 20.0 * t - 10.0) / 2.0 : (2.0 - std::pow(2.0, -20.0 * t + 10.0)) / 2.0;
}

// The quarter circle. The sqrt argument is clamped, rounding must never make it negative.
inline double circIn(double t) { return 1.0 - std::sqrt(std::max(0.0, 1.0 - t * t)); }
inline double circOut(double t) { return std::sqrt(std::max(0.0, 1.0 - (t - 1.0) * (t - 1.0))); }
inline double circInOut(double t) {
    return t < 0.5 ? (1.0 - std::sqrt(std::max(0.0, 1.0 - (2.0 * t) * (2.0 * t)))) / 2.0
                   : (std::sqrt(std::max(0.0, 1.0 - (-2.0 * t + 2.0) * (-2.0 * t + 2.0))) + 1.0) / 2.0;
}

inline double backIn(double t) { return detail::kBackC3 * t * t * t - detail::kBackC1 * t * t; }
inline double backOut(double t) {
    const double u = t - 1.0;
    return 1.0 + detail::kBackC3 * u * u * u + detail::kBackC1 * u * u;
}
inline double backInOut(double t) {
    constexpr double c2 = detail::kBackC2;
    if (t < 0.5) return ((2.0 * t) * (2.0 * t) * ((c2 + 1.0) * 2.0 * t - c2)) / 2.0;
    const double u = 2.0 * t - 2.0;
    return (u * u * ((c2 + 1.0) * u + c2) + 2.0) / 2.0;
}

inline double elasticIn(double t) {
    if (t <= 0.0) return 0.0;
    if (t >= 1.0) return 1.0;
    return -std::pow(2.0, 10.0 * t - 10.0) * std::sin((10.0 * t - 10.75) * detail::kElasticC4);
}
inline double elasticOut(double t) {
    if (t <= 0.0) return 0.0;
    if (t >= 1.0) return 1.0;
    return std::pow(2.0, -10.0 * t) * std::sin((10.0 * t - 0.75) * detail::kElasticC4) + 1.0;
}
inline double elasticInOut(double t) {
    if (t <= 0.0) return 0.0;
    if (t >= 1.0) return 1.0;
    const double s = std::sin((20.0 * t - 11.125) * detail::kElasticC5);
    return t < 0.5 ? -(std::pow(2.0, 20.0 * t - 10.0) * s) / 2.0 : (std::pow(2.0, -20.0 * t + 10.0) * s) / 2.0 + 1.0;
}

// Four parabolas, each one a bounce that ends lower: the ball hits the ground at 1/2.75, 2/2.75, 2.5/2.75 and 1.
inline double bounceOut(double t) {
    constexpr double n1 = detail::kBounceN1;
    constexpr double d1 = detail::kBounceD1;
    if (t < 1.0 / d1) return n1 * t * t;
    if (t < 2.0 / d1) {
        t -= 1.5 / d1;
        return n1 * t * t + 0.75;
    }
    if (t < 2.5 / d1) {
        t -= 2.25 / d1;
        return n1 * t * t + 0.9375;
    }
    t -= 2.625 / d1;
    return n1 * t * t + 0.984375;
}
inline double bounceIn(double t) { return 1.0 - bounceOut(1.0 - t); }
inline double bounceInOut(double t) { return t < 0.5 ? (1.0 - bounceOut(1.0 - 2.0 * t)) / 2.0 : (1.0 + bounceOut(2.0 * t - 1.0)) / 2.0; }

// ---------------------------------------------------------------------------------------------------------------
// Picking a function by name (the demo shows them in a grid)
// ---------------------------------------------------------------------------------------------------------------

enum class Family { Linear, Sine, Quad, Cubic, Quart, Quint, Expo, Circ, Back, Elastic, Bounce };
enum class Variant { In, Out, InOut };

inline constexpr int kFamilyCount = 11;

inline Fn lookup(Family family, Variant variant) {
    static constexpr Fn table[kFamilyCount][3] = {
        {linear, linear, linear},
        {sineIn, sineOut, sineInOut},
        {quadIn, quadOut, quadInOut},
        {cubicIn, cubicOut, cubicInOut},
        {quartIn, quartOut, quartInOut},
        {quintIn, quintOut, quintInOut},
        {expoIn, expoOut, expoInOut},
        {circIn, circOut, circInOut},
        {backIn, backOut, backInOut},
        {elasticIn, elasticOut, elasticInOut},
        {bounceIn, bounceOut, bounceInOut},
    };
    return table[static_cast<int>(family)][static_cast<int>(variant)];
}

inline double apply(Family family, Variant variant, double t) { return lookup(family, variant)(t); }

inline const char* name(Family family) {
    static constexpr const char* names[kFamilyCount] = {"linear", "sine", "quad", "cubic", "quart", "quint", "expo", "circ", "back", "elastic", "bounce"};
    return names[static_cast<int>(family)];
}

inline const char* name(Variant variant) {
    static constexpr const char* names[3] = {"in", "out", "inOut"};
    return names[static_cast<int>(variant)];
}

struct Range {
    double min = 0.0;
    double max = 1.0;
};

/// Smallest and largest value of `fn` over [0,1], sampled. Gives the plots their vertical range.
inline Range sampleRange(Fn fn, int samples = 200) {
    Range r{fn(0.0), fn(0.0)};
    for (int i = 1; i <= samples; ++i) {
        const double v = fn(static_cast<double>(i) / static_cast<double>(samples));
        r.min = std::min(r.min, v);
        r.max = std::max(r.max, v);
    }
    return r;
}

// ---------------------------------------------------------------------------------------------------------------
// Things a tween can move
// ---------------------------------------------------------------------------------------------------------------

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};
inline Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
inline Vec2 operator*(Vec2 a, double k) { return {a.x * k, a.y * k}; }

/// Color channels as doubles in 0..255, so an overshooting ease does not wrap around before it is clamped for drawing.
struct Rgb {
    double r = 0.0;
    double g = 0.0;
    double b = 0.0;
};
inline Rgb operator+(Rgb a, Rgb b) { return {a.r + b.r, a.g + b.g, a.b + b.b}; }
inline Rgb operator-(Rgb a, Rgb b) { return {a.r - b.r, a.g - b.g, a.b - b.b}; }
inline Rgb operator*(Rgb a, double k) { return {a.r * k, a.g * k, a.b * k}; }

/// a + (b - a) * k, for anything with +, - and * double. k is not clamped, the ease decides how far it may go.
template <class T>
T mix(const T& a, const T& b, double k) {
    return a + (b - a) * k;
}

// ---------------------------------------------------------------------------------------------------------------
// Tween
// ---------------------------------------------------------------------------------------------------------------

enum class Mode {
    Once,      // runs to the end and stays there
    Loop,      // jumps back to the start
    PingPong,  // runs back and forth
};

template <class T>
struct Tween {
    T from{};
    T to{};
    double duration = 1.0;  // seconds for one run from `from` to `to`
    Fn ease = linear;
    Mode mode = Mode::Once;

    Tween() = default;
    Tween(T a, T b, double seconds, Fn fn, Mode m = Mode::Once) : from(a), to(b), duration(seconds), ease(fn), mode(m) {}

    /// Advances the clock by one frame's delta time. Returns the part of `dt` that was not needed because a Once tween
    /// ended in the middle of the step (0 while it is running, Loop and PingPong never end). Negative time is ignored.
    double update(double dt) {
        if (!(dt > 0.0)) dt = 0.0;  // also catches NaN
        if (duration <= 0.0) {
            // Takes no time at all: a Once tween is done right away and passes the whole step on.
            if (mode != Mode::Once) return 0.0;
            done_ = true;
            return dt;
        }
        switch (mode) {
            case Mode::Once: {
                if (done_) return dt;
                elapsed_ += dt;
                if (elapsed_ >= duration) {
                    const double rest = elapsed_ - duration;
                    elapsed_ = duration;
                    done_ = true;
                    return rest;
                }
                return 0.0;
            }
            case Mode::Loop: elapsed_ = std::fmod(elapsed_ + dt, duration); return 0.0;
            case Mode::PingPong: elapsed_ = std::fmod(elapsed_ + dt, 2.0 * duration); return 0.0;
        }
        return 0.0;
    }

    /// Linear progress in [0,1], before the ease. A zero length tween counts as arrived.
    double progress() const {
        if (duration <= 0.0) return 1.0;
        if (mode == Mode::PingPong) return elapsed_ < duration ? elapsed_ / duration : (2.0 * duration - elapsed_) / duration;
        return std::clamp(elapsed_ / duration, 0.0, 1.0);
    }

    /// The current value: from + (to - from) * ease(progress). At the end of a Once tween this is exactly `to`.
    T value() const {
        const double p = progress();
        if (p >= 1.0 && mode == Mode::Once) return to;
        return mix(from, to, ease(p));
    }

    bool finished() const { return done_; }

    void restart() {
        elapsed_ = 0.0;
        done_ = false;
    }

private:
    double elapsed_ = 0.0;  // Once, Loop: 0..duration. PingPong: 0..2*duration
    bool done_ = false;     // only ever set for Once
};

/// Runs Once tweens one after another: each gets the time the one before it left over. A tween whose turn has not come
/// yet is not touched (not even a zero length one), so it cannot fire early. Returns the time left after the last one.
template <class... Ts>
double updateChain(double dt, Tween<Ts>&... tweens) {
    double rest = dt;
    bool blocked = false;
    auto step = [&](auto& tween) {
        if (blocked || tween.finished()) return;
        rest = tween.update(rest);
        if (!tween.finished()) blocked = true;
    };
    (step(tweens), ...);
    return rest;
}

template <class... Ts>
bool allFinished(const Tween<Ts>&... tweens) {
    return (tweens.finished() && ...);
}

}  // namespace ease
