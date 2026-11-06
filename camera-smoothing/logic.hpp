// Camera smoothing: the logic of the demo, no SDL, so test.cpp can check it.
//
// A camera that follows a player has to be smooth, but "smooth" must not depend on how many frames per second the
// machine renders. Three ways to chase a target, all called once per frame:
//
//   lerp   cam += (target - cam) * 0.1
//          The factor is a fraction PER FRAME. At 144 fps the camera closes the gap 2.4 times as often as at 60 fps.
//   exp    cam += (target - cam) * (1 - exp(-lambda * dt))
//          The factor is derived from the frame time. This is the exact solution of "move towards the target at a
//          speed proportional to the gap" for a target that holds still during the frame, so two half steps give the
//          same result as one whole step. For a target that moves while the frames get longer a small error remains,
//          because the target is only looked at once per frame.
//   spring the same idea with a velocity: a critically damped spring, closed form (Ryan Juckett's derivation).
//          It starts and stops without a jolt and never oscillates, but a camera with momentum can overshoot once.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>

namespace cs {

// ---- the scripted player ------------------------------------------------------------------------------------

/// One piece of the script: a teleport of `jump` px at the start, then `duration` seconds at `velocity` px/s.
struct Segment {
    double duration;
    double velocity;
    double jump;
};

inline constexpr double kJump = 240.0;

/// Run right, stop, run faster, stop, teleport, run back, teleport back, run back faster. The jumps and the
/// distances cancel out, so the path repeats forever and the numbers stay small.
inline constexpr std::array<Segment, 8> kScript{{
    {1.5, 300.0, 0.0},
    {1.0, 0.0, 0.0},
    {1.0, 450.0, 0.0},
    {1.0, 0.0, 0.0},
    {1.0, 0.0, kJump},
    {1.5, -300.0, 0.0},
    {1.0, 0.0, -kJump},
    {1.0, -450.0, 0.0},
}};

constexpr double scriptPeriod() {
    double sum = 0.0;
    for (const Segment& s : kScript) sum += s.duration;
    return sum;
}

inline constexpr double kPeriod = scriptPeriod();

namespace detail {

/// Time inside one period, in [0, kPeriod). The script is periodic, so only the phase matters.
inline double wrap(double t) {
    double phase = std::fmod(t, kPeriod);
    if (phase < 0.0) phase += kPeriod;
    return phase;
}

}  // namespace detail

/// Where the player is at time t seconds. A boundary between two segments belongs to the segment that starts there.
inline double playerPos(double t) {
    double phase = detail::wrap(t);
    double pos = 0.0;
    for (const Segment& s : kScript) {
        pos += s.jump;
        if (phase < s.duration) return pos + s.velocity * phase;
        pos += s.velocity * s.duration;
        phase -= s.duration;
    }
    return pos;  // only reached by rounding at the very end of the period: the net displacement is 0
}

/// The player's velocity in px/s. A teleport is not a velocity.
inline double playerVel(double t) {
    double phase = detail::wrap(t);
    for (const Segment& s : kScript) {
        if (phase < s.duration) return s.velocity;
        phase -= s.duration;
    }
    return 0.0;
}

// ---- the three cameras --------------------------------------------------------------------------------------

/// Naive: a fixed fraction of the gap per FRAME. The factor is clamped so it cannot overshoot.
inline double lerpStep(double cam, double target, double factor) {
    return cam + (target - cam) * std::clamp(factor, 0.0, 1.0);
}

/// Share of the gap that is closed in `dt` seconds at rate `lambda` (1/s): 1 - e^(-lambda dt). 0 for a rate or
/// a frame time that is not positive.
inline double expFactor(double lambda, double dt) {
    if (lambda <= 0.0 || dt <= 0.0) return 0.0;
    return 1.0 - std::exp(-lambda * dt);
}

/// The lambda that gives exactly `factor` per frame at `fps`: converts an old lerp constant. Factors of 1 and more
/// would need an infinite rate, so they are capped just below 1.
inline double lambdaForLerp(double factor, double fps) {
    if (factor <= 0.0 || fps <= 0.0) return 0.0;
    return -std::log(1.0 - std::min(factor, 0.999999)) * fps;
}

/// Frame-rate independent smoothing: cam += (target - cam) * (1 - exp(-lambda * dt)).
inline double expStep(double cam, double target, double lambda, double dt) {
    return cam + (target - cam) * expFactor(lambda, dt);
}

/// A critically damped spring, kept as position and velocity.
struct Spring {
    double pos = 0.0;
    double vel = 0.0;
};

/// Advances the spring by `dt` seconds towards `target` (held still during the step) with natural frequency
/// `omega` (rad/s). Closed form of x'' = -2 omega x' - omega^2 x with x = pos - target: x(t) = (x0 + c t) e^(-omega t)
/// where c = v0 + omega x0. Exact for any dt, so it does not matter how the time is cut into frames.
inline void springStep(Spring& s, double target, double omega, double dt) {
    if (omega <= 0.0 || dt <= 0.0) return;
    const double x0 = s.pos - target;
    const double c = s.vel + omega * x0;
    const double decay = std::exp(-omega * dt);
    s.pos = target + (x0 + c * dt) * decay;
    s.vel = (s.vel - c * omega * dt) * decay;
}

// ---- frame clock and scope ----------------------------------------------------------------------------------

/// Turns real elapsed time into a number of simulated frames of `step` seconds each, so the demo can pretend to run
/// at 15, 60 or 144 fps on any display.
struct FrameClock {
    double accumulator = 0.0;

    void reset() { accumulator = 0.0; }

    /// Frames to simulate for `realDt` real seconds. A stall is clamped to 0.25 s, at most `maxSteps` frames run and
    /// the rest of a stall is dropped instead of carried into the next frame.
    int advance(double realDt, double step, int maxSteps) {
        if (step <= 0.0 || maxSteps <= 0 || realDt <= 0.0) return 0;
        accumulator += std::min(realDt, 0.25);
        int steps = 0;
        while (accumulator >= step && steps < maxSteps) {
            accumulator -= step;
            ++steps;
        }
        if (steps == maxSteps) accumulator = 0.0;
        return steps;
    }
};

/// The last `window` seconds of one value, one sample per simulated frame.
struct Trace {
    struct Sample {
        double t;
        double v;
    };
    std::deque<Sample> samples;

    void push(double t, double v, double window) {
        samples.push_back({t, v});
        while (!samples.empty() && samples.front().t < t - window) samples.pop_front();
    }

    void clear() { samples.clear(); }
};

/// Screen x of a sample taken at time t: `now` is the right edge of the scope, `now - window` the left edge.
inline double scopeX(double t, double now, double window, double x0, double width) {
    if (window <= 0.0) return x0 + width;
    return x0 + width * (1.0 - (now - t) / window);
}

/// Screen y of a value: 0 in the middle, +range at the top, -range at the bottom, clipped beyond.
inline double scopeY(double value, double range, double y0, double height) {
    if (range <= 0.0) return y0 + height * 0.5;
    return y0 + height * 0.5 - std::clamp(value / range, -1.0, 1.0) * height * 0.5;
}

}  // namespace cs
