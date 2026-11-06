// Tests for the camera smoothing logic. Plain asserts on hand-computed cases, no framework, no SDL.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "logic.hpp"

using namespace cs;

namespace {

bool near(double a, double b, double eps = 1e-9) { return std::fabs(a - b) < eps; }

constexpr double kLambda60 = 6.321630939469577;  // -ln(0.9) * 60, the exp rate that equals lerp 0.1 at 60 fps

// --- the scripted player ---------------------------------------------------------------------------------------

void scriptHasANetDisplacementOfZero() {
    // 1.5 s at 300 + 1 s at 450 + 240 jump - 1.5 s at 300 - 240 jump - 1 s at 450 = 0, so the path repeats forever.
    assert(near(kPeriod, 9.0));
    assert(near(playerPos(0.0), 0.0));
    assert(near(playerPos(kPeriod), 0.0));
    assert(near(playerPos(kPeriod * 1000.0), 0.0));
}

void playerPositions() {
    assert(near(playerPos(1.0), 300.0));    // 1 s at 300 px/s
    assert(near(playerPos(1.5), 450.0));    // end of the first run
    assert(near(playerPos(2.0), 450.0));    // standing still
    assert(near(playerPos(3.0), 675.0));    // 0.5 s into the 450 px/s run
    assert(near(playerPos(3.5), 900.0));
    assert(near(playerPos(4.4), 900.0));
    assert(near(playerPos(4.5), 1140.0));   // teleport of 240 px at the start of the segment
    assert(near(playerPos(5.5), 1140.0));
    assert(near(playerPos(6.25), 915.0));   // 0.75 s back at 300 px/s
    assert(near(playerPos(7.0), 450.0));    // 690 after the run, then teleport back by 240
    assert(near(playerPos(8.5), 225.0));    // 0.5 s at -450 px/s
}

void playerRepeatsAndHandlesNegativeTime() {
    assert(near(playerPos(1.0 + kPeriod), playerPos(1.0)));
    assert(near(playerPos(8.5 + 7.0 * kPeriod), 225.0));
    assert(near(playerPos(-0.5), 225.0));   // same as 8.5
    assert(near(playerVel(-0.5), -450.0));
}

void playerVelocities() {
    assert(near(playerVel(0.0), 300.0));
    assert(near(playerVel(1.0), 300.0));
    assert(near(playerVel(1.5), 0.0));      // segment boundaries belong to the segment that starts there
    assert(near(playerVel(2.0), 0.0));
    assert(near(playerVel(3.0), 450.0));
    assert(near(playerVel(5.0), 0.0));      // the teleport is not a velocity
    assert(near(playerVel(6.0), -300.0));
    assert(near(playerVel(8.5), -450.0));
}

void playerOnlyJumpsAtTheTeleports() {
    // Between two samples a tiny step apart the position changes by at most speed * step, except at the two teleports.
    int jumps = 0;
    double prev = playerPos(0.0);
    for (int i = 1; i <= 9000; ++i) {
        const double now = playerPos(static_cast<double>(i) * 0.001);
        if (std::fabs(now - prev) > 1.0) ++jumps;  // 450 px/s * 1 ms = 0.45 px
        prev = now;
    }
    assert(jumps == 2);
}

// --- naive lerp ------------------------------------------------------------------------------------------------

double runConstantTarget(int fps, int mode) {
    // One second of frames chasing target 1 from 0. mode 0: lerp 0.1 per frame, mode 1: exp decay.
    const double dt = 1.0 / fps;
    double cam = 0.0;
    for (int i = 0; i < fps; ++i) {
        cam = mode == 0 ? lerpStep(cam, 1.0, 0.1) : expStep(cam, 1.0, kLambda60, dt);
    }
    return cam;
}

void lerpBasics() {
    assert(near(lerpStep(0.0, 10.0, 0.1), 1.0));
    assert(near(lerpStep(5.0, 5.0, 0.3), 5.0));
    assert(near(lerpStep(0.0, 10.0, 0.0), 0.0));
    assert(near(lerpStep(0.0, 10.0, 1.5), 10.0));    // a factor above 1 would overshoot, it is clamped
    assert(near(lerpStep(0.0, 10.0, -1.0), 0.0));
}

void lerpDependsOnTheFrameRate() {
    // 0.9^15 = 0.2058911 after 15 frames, 0.9^60 = 0.0017970 after 60 frames, 0.9^144 = 2.5e-7 after 144 frames.
    assert(near(runConstantTarget(15, 0), 0.7941089, 1e-6));
    assert(near(runConstantTarget(60, 0), 0.9982030, 1e-6));
    assert(runConstantTarget(144, 0) > 0.9999997);
    // One second later the camera is 79 percent of the way at 15 fps and 99.99997 percent at 144 fps.
    assert(runConstantTarget(15, 0) < 0.8 && runConstantTarget(144, 0) > 0.999999);
}

// --- exponential decay -----------------------------------------------------------------------------------------

void expFactorBasics() {
    assert(near(expFactor(std::log(2.0), 1.0), 0.5));      // half life: lambda * t = ln 2
    assert(near(expFactor(10.0, 0.0), 0.0));
    assert(near(expFactor(0.0, 1.0), 0.0));
    assert(near(expFactor(-3.0, 1.0), 0.0));               // a negative rate would run away, it is a no-op
    assert(near(expFactor(10.0, -1.0), 0.0));
    assert(expFactor(1000.0, 1.0) > 0.999999999 && expFactor(1000.0, 1.0) <= 1.0);
}

void lambdaMatchesALerpFactor() {
    assert(near(lambdaForLerp(0.1, 60.0), kLambda60, 1e-12));
    assert(near(expFactor(lambdaForLerp(0.1, 60.0), 1.0 / 60.0), 0.1, 1e-12));
    assert(near(expFactor(lambdaForLerp(0.5, 30.0), 1.0 / 30.0), 0.5, 1e-12));
    assert(near(lambdaForLerp(0.0, 60.0), 0.0));
    assert(near(lambdaForLerp(0.1, 0.0), 0.0));
    assert(std::isfinite(lambdaForLerp(1.0, 60.0)));        // factor 1 would be -ln(0): clamped
    assert(std::isfinite(lambdaForLerp(2.0, 60.0)));
}

void expStepBasics() {
    assert(near(expStep(0.0, 10.0, std::log(2.0), 1.0), 5.0));
    assert(near(expStep(4.0, 4.0, 5.0, 0.016), 4.0));
    assert(near(expStep(1.0, 9.0, 0.0, 0.016), 1.0));
    assert(near(expStep(1.0, 9.0, 5.0, 0.0), 1.0));
}

void expDecayIgnoresTheFrameRate() {
    // The same second of constant target gives the same camera at every frame rate: 1 - 0.9^60 = 0.9982030.
    for (const int fps : {15, 30, 60, 144, 240}) assert(near(runConstantTarget(fps, 1), 0.9982030, 1e-6));
    assert(near(runConstantTarget(15, 1), runConstantTarget(144, 1)));
    // At 60 fps the two are the same camera, because lambda was derived from the lerp factor there.
    assert(near(runConstantTarget(60, 0), runConstantTarget(60, 1), 1e-12));
}

void expSplitsIntoTwoHalfSteps() {
    const double whole = expStep(2.0, 10.0, 7.0, 0.1);
    const double half = expStep(expStep(2.0, 10.0, 7.0, 0.05), 10.0, 7.0, 0.05);
    assert(near(whole, half, 1e-12));
}

// --- lag behind a moving target ---------------------------------------------------------------------------------

double steadyLag(int fps, int mode, double speed) {
    // Target runs at `speed` px/s for 10 s, the camera starts at rest on it. Returns target - camera at the end.
    const double dt = 1.0 / fps;
    double cam = 0.0;
    Spring spring;
    double target = 0.0;
    for (int i = 0; i < fps * 10; ++i) {
        target = speed * dt * (i + 1);
        if (mode == 0) cam = lerpStep(cam, target, 0.1);
        if (mode == 1) cam = expStep(cam, target, kLambda60, dt);
        if (mode == 2) {
            springStep(spring, target, 10.0, dt);
            cam = spring.pos;
        }
    }
    return target - cam;
}

void lagOfLerpGrowsWhenFramesGetLonger() {
    // With the target one step ahead the lag is (1 - a) * speed * dt / a = 9 * speed * dt for a = 0.1.
    assert(near(steadyLag(15, 0, 300.0), 180.0, 1e-3));
    assert(near(steadyLag(60, 0, 300.0), 45.0, 1e-3));
    assert(near(steadyLag(144, 0, 300.0), 300.0 / 144.0 * 9.0, 1e-3));
}

void lagOfExpDecayBarelyMoves() {
    // speed * dt / (e^(lambda dt) - 1). At 60 fps e^(lambda dt) = 1 / 0.9, so it is 45 px, like the lerp.
    for (const int fps : {15, 60, 144}) {
        const double dt = 1.0 / fps;
        assert(near(steadyLag(fps, 1, 300.0), 300.0 * dt / (std::exp(kLambda60 * dt) - 1.0), 1e-4));
    }
    assert(near(steadyLag(60, 1, 300.0), 45.0, 1e-4));
    // Continuous limit speed / lambda = 47.5 px; the 15 fps camera is 20 percent closer, the lerp one is four times farther.
    assert(near(300.0 / kLambda60, 47.46, 0.01));
    assert(steadyLag(15, 1, 300.0) > 0.79 * 300.0 / kLambda60);
    assert(steadyLag(15, 0, 300.0) > 3.5 * steadyLag(60, 0, 300.0));
}

// --- critically damped spring -----------------------------------------------------------------------------------

void springOneStepByHand() {
    // x0 = -1 (camera 1 behind), v0 = 0, omega = 10, dt = 0.1: omega * dt = 1.
    // x = (x0 + (v0 + omega x0) dt) e^-1 = (-1 - 1) e^-1 = -0.7357589, v = -(v0 + omega x0) omega dt e^-1 = 10 e^-1.
    Spring s{0.0, 0.0};
    springStep(s, 1.0, 10.0, 0.1);
    assert(near(s.pos, 1.0 - 2.0 * std::exp(-1.0), 1e-12));
    assert(near(s.pos, 0.2642411, 1e-6));
    assert(near(s.vel, 10.0 * std::exp(-1.0), 1e-12));
    assert(near(s.vel, 3.6787944, 1e-6));
}

void springOnTargetStaysPut() {
    Spring s{3.0, 0.0};
    springStep(s, 3.0, 10.0, 0.016);
    assert(near(s.pos, 3.0) && near(s.vel, 0.0));
}

void springIgnoresTheFrameRate() {
    // Closed form after 1 s from rest towards 1: pos = 1 - (1 + omega) e^-omega = 1 - 11 e^-10, vel = omega^2 e^-omega.
    const double expectedPos = 1.0 - 11.0 * std::exp(-10.0);
    const double expectedVel = 100.0 * std::exp(-10.0);
    for (const int fps : {15, 30, 60, 144, 240}) {
        Spring s{0.0, 0.0};
        for (int i = 0; i < fps; ++i) springStep(s, 1.0, 10.0, 1.0 / fps);
        assert(near(s.pos, expectedPos, 1e-9));
        assert(near(s.vel, expectedVel, 1e-9));
    }
    assert(near(expectedPos, 0.9995006, 1e-6));
}

void springSplitsIntoTwoHalfSteps() {
    Spring whole{0.3, 2.0};
    springStep(whole, 1.0, 8.0, 0.1);
    Spring half{0.3, 2.0};
    springStep(half, 1.0, 8.0, 0.05);
    springStep(half, 1.0, 8.0, 0.05);
    assert(near(whole.pos, half.pos, 1e-12));
    assert(near(whole.vel, half.vel, 1e-12));
}

void springNeverOvershootsFromRest() {
    for (const int fps : {15, 60, 144}) {
        Spring s{0.0, 0.0};
        double prev = 0.0;
        for (int i = 0; i < fps * 3; ++i) {
            springStep(s, 1.0, 10.0, 1.0 / fps);
            assert(s.pos <= 1.0 && s.pos >= prev);
            prev = s.pos;
        }
        assert(near(s.pos, 1.0, 1e-9));
    }
}

void springWithMomentumOvershootsOnceAndComesBack() {
    // Moving towards the target faster than the spring would allow: c2 = v0 + omega x0 = 100 - 10 = 90 > 0.
    Spring s{0.0, 100.0};
    double top = 0.0;
    for (int i = 0; i < 600; ++i) {
        springStep(s, 1.0, 10.0, 1.0 / 60.0);
        top = std::max(top, s.pos);
    }
    assert(top > 1.2);
    assert(near(s.pos, 1.0, 1e-6));
}

void springGuards() {
    Spring a{1.0, 2.0};
    springStep(a, 5.0, 10.0, 0.0);
    assert(near(a.pos, 1.0) && near(a.vel, 2.0));
    springStep(a, 5.0, 10.0, -1.0);
    assert(near(a.pos, 1.0) && near(a.vel, 2.0));
    springStep(a, 5.0, 0.0, 0.1);
    assert(near(a.pos, 1.0) && near(a.vel, 2.0));
    springStep(a, 5.0, -4.0, 0.1);
    assert(near(a.pos, 1.0) && near(a.vel, 2.0));
}

void springLagOnARun() {
    // Without feed forward the spring trails a running target by about 2 speed / omega = 60 px at 300 px/s.
    // The target is held for the whole frame, which leads by half a frame on average: about speed * dt / 2 less.
    for (const int fps : {15, 60, 144}) {
        const double dt = 1.0 / fps;
        const double lag = steadyLag(fps, 2, 300.0);
        assert(std::fabs(lag - (60.0 - 300.0 * dt / 2.0)) < 0.5 * 300.0 * dt / 2.0 + 0.2);
    }
    assert(std::fabs(steadyLag(15, 2, 300.0) - steadyLag(144, 2, 300.0)) < 12.0);   // lerp: 161 px apart
    assert(std::fabs(steadyLag(15, 0, 300.0) - steadyLag(144, 0, 300.0)) > 150.0);
}

double firstFrameSpeed(int mode, int fps) {
    // Teleport of the target by 240 px, speed of the camera over the first frame in px/s.
    const double dt = 1.0 / fps;
    if (mode == 1) return (expStep(0.0, 240.0, kLambda60, dt) - 0.0) / dt;
    Spring s{0.0, 0.0};
    springStep(s, 240.0, 10.0, dt);
    return s.pos / dt;
}

void teleportHitsExpHardAndTheSpringSoftly() {
    // exp decay moves 1 - 0.9 = 10 percent of the gap at once: 24 px in one frame, 1440 px/s at 60 fps.
    assert(near(firstFrameSpeed(1, 60), 1440.0, 1e-6));
    // The spring starts with zero velocity: x = (-240 - 2400 / 60) e^(-1/6) = -237.015, so it moves 2.985 px.
    assert(near(firstFrameSpeed(0, 60) * (1.0 / 60.0), 2.985, 0.01));
    assert(firstFrameSpeed(0, 60) < firstFrameSpeed(1, 60) / 4.0);
    // Peak speed of the spring over the whole move is 240 * omega / e = 883 px/s, below where exp starts.
    Spring s{0.0, 0.0};
    double peak = 0.0;
    double prev = 0.0;
    for (int i = 0; i < 240; ++i) {
        springStep(s, 240.0, 10.0, 1.0 / 60.0);
        peak = std::max(peak, (s.pos - prev) * 60.0);
        prev = s.pos;
    }
    assert(peak < 900.0 && peak > 800.0);
    assert(peak < firstFrameSpeed(1, 60) * 0.7);
}

// --- frame clock ------------------------------------------------------------------------------------------------

void frameClockCountsWholeSteps() {
    FrameClock c;
    assert(c.advance(0.25, 0.125, 8) == 2);
    assert(near(c.accumulator, 0.0));
    assert(c.advance(0.1875, 0.125, 8) == 1);        // 0.1875 = 1.5 steps
    assert(near(c.accumulator, 0.0625));
    assert(c.advance(0.0625, 0.125, 8) == 1);        // the rest joins the next frame
    assert(near(c.accumulator, 0.0));
    assert(c.advance(0.0625, 0.125, 8) == 0);
    assert(near(c.accumulator, 0.0625));
}

void frameClockGuards() {
    FrameClock c;
    assert(c.advance(-1.0, 0.125, 8) == 0);
    assert(near(c.accumulator, 0.0));
    assert(c.advance(0.5, 0.0, 8) == 0);             // a zero step would never end
    assert(c.advance(0.5, 0.125, 0) == 0);
    // A long stall (tab in the background): dt is clamped to 0.25 s = 8 steps, the cap is 4, nothing is carried over.
    assert(c.advance(10.0, 0.03125, 4) == 4);
    assert(near(c.accumulator, 0.0));
    FrameClock d;
    assert(d.advance(10.0, 0.125, 100) == 2);        // clamped to 0.25 s
    d.advance(0.0625, 0.125, 8);
    d.reset();
    assert(near(d.accumulator, 0.0));
}

// --- traces and scope mapping -----------------------------------------------------------------------------------

void traceKeepsOnlyTheWindow() {
    Trace t;
    for (int i = 0; i <= 5; ++i) t.push(static_cast<double>(i), 10.0 * i, 2.0);
    assert(t.samples.size() == 3);                    // t = 3, 4, 5 (>= 5 - 2)
    assert(near(t.samples.front().t, 3.0) && near(t.samples.front().v, 30.0));
    assert(near(t.samples.back().t, 5.0) && near(t.samples.back().v, 50.0));
    t.clear();
    assert(t.samples.empty());
}

void scopeMapping() {
    // Window of 4 s over x = 100 .. 400: "now" is the right edge, 4 s ago the left edge, 1 s ago at x = 325.
    assert(near(scopeX(10.0, 10.0, 4.0, 100.0, 300.0), 400.0));
    assert(near(scopeX(6.0, 10.0, 4.0, 100.0, 300.0), 100.0));
    assert(near(scopeX(9.0, 10.0, 4.0, 100.0, 300.0), 325.0));
    assert(near(scopeX(10.0, 10.0, 0.0, 100.0, 300.0), 400.0));   // window 0: everything at the right edge
    // Range +-300 over y = 50 .. 150 (height 100): 0 is the middle, +300 the top, -150 three quarters down, clipped beyond.
    assert(near(scopeY(0.0, 300.0, 50.0, 100.0), 100.0));
    assert(near(scopeY(300.0, 300.0, 50.0, 100.0), 50.0));
    assert(near(scopeY(-150.0, 300.0, 50.0, 100.0), 125.0));
    assert(near(scopeY(900.0, 300.0, 50.0, 100.0), 50.0));
    assert(near(scopeY(-900.0, 300.0, 50.0, 100.0), 150.0));
    assert(near(scopeY(5.0, 0.0, 50.0, 100.0), 100.0));            // range 0: flat line in the middle
}

}  // namespace

int main() {
    scriptHasANetDisplacementOfZero();
    playerPositions();
    playerRepeatsAndHandlesNegativeTime();
    playerVelocities();
    playerOnlyJumpsAtTheTeleports();
    lerpBasics();
    lerpDependsOnTheFrameRate();
    expFactorBasics();
    lambdaMatchesALerpFactor();
    expStepBasics();
    expDecayIgnoresTheFrameRate();
    expSplitsIntoTwoHalfSteps();
    lagOfLerpGrowsWhenFramesGetLonger();
    lagOfExpDecayBarelyMoves();
    springOneStepByHand();
    springOnTargetStaysPut();
    springIgnoresTheFrameRate();
    springSplitsIntoTwoHalfSteps();
    springNeverOvershootsFromRest();
    springWithMomentumOvershootsOnceAndComesBack();
    springGuards();
    springLagOnARun();
    teleportHitsExpHardAndTheSpringSoftly();
    frameClockCountsWholeSteps();
    frameClockGuards();
    traceKeepsOnlyTheWindow();
    scopeMapping();
    std::puts("camera-smoothing logic: all asserts passed");
    return 0;
}
