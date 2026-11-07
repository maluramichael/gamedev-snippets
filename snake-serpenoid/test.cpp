// Tests for the serpenoid snake logic. Plain asserts on hand-computed cases, no framework, no SDL.
// World and body coordinates are "math" form (y up). The logic does not care which way y points on screen.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <optional>

#include "logic.hpp"

using namespace serp;

namespace {

constexpr double kPeriodOmega = kTwoPi / 2.0;  // one wave period = 2 s
constexpr double kDt = 1.0 / 240.0;            // 480 steps per period, a whole number
constexpr int kStepsPerCycle = 480;

bool near(double a, double b, double eps = 1e-9) { return std::fabs(a - b) < eps; }
bool nearVec(Vec2 a, Vec2 b, double eps = 1e-9) { return near(a.x, b.x, eps) && near(a.y, b.y, eps); }
double length(Vec2 v) { return std::hypot(v.x, v.y); }
double wrapAngle(double a) { return std::atan2(std::sin(a), std::cos(a)); }

struct Run {
    Vec2 displacement;     // center of mass, over the measured cycles
    double headingChange;  // over the measured cycles, wrapped into (-pi, pi]
    double forward;        // displacement along the mean heading, in body lengths per cycle
    double sideways;       // displacement across the mean heading, in body lengths per cycle
};

/// Runs `cycles` whole wave periods and reports what happened during the last `measured` of them. Every cycle is a whole
/// number of fixed steps, so the shape is back where it started and only the net motion is left.
Run run(double a, double b, double c, double ratio, double omega, int cycles = 4, int measured = 2) {
    const Gait g = gaitFrom(a, b, c);
    State s;
    s.pose = poseFor(shapeAt(g, 0.0, omega), {0.0, 0.0}, 0.0);
    const int cycleSteps = kStepsPerCycle;
    for (int i = 0; i < (cycles - measured) * cycleSteps; ++i) advance(s, g, omega, ratio, kDt);
    const Shape before = shapeAt(g, s.phase, omega);
    const Vec2 com0 = centerOfMass(before, s.pose);
    const double h0 = heading(before, s.pose);
    // The tail-to-head direction swings a little within a cycle, so forward and sideways are measured against its
    // average over the measured cycles, not against the instantaneous value at the start.
    double swing = 0.0;
    for (int i = 0; i < measured * cycleSteps; ++i) {
        advance(s, g, omega, ratio, kDt);
        swing += wrapAngle(heading(shapeAt(g, s.phase, omega), s.pose) - h0);
    }
    const double meanHeading = h0 + swing / (measured * cycleSteps);
    const Shape after = shapeAt(g, s.phase, omega);
    const Vec2 com1 = centerOfMass(after, s.pose);
    Run r;
    r.displacement = {com1.x - com0.x, com1.y - com0.y};
    r.headingChange = wrapAngle(heading(after, s.pose) - h0);
    const double perCycle = 1.0 / (measured * static_cast<double>(kLinks));
    r.forward = (r.displacement.x * std::cos(meanHeading) + r.displacement.y * std::sin(meanHeading)) * perCycle;
    r.sideways = (-r.displacement.x * std::sin(meanHeading) + r.displacement.y * std::cos(meanHeading)) * perCycle;
    return r;
}

void gaitMapping() {
    // Source: alpha = a sin(beta / 2), beta = b / n, gamma = -c / n, here n = 12 links.
    const Gait g = gaitFrom(1.0, kTwoPi, 0.0);
    assert(near(g.beta, kPi / 6.0));
    assert(near(g.alpha, std::sin(kPi / 12.0)));
    assert(near(g.alpha, 0.25881904510252074, 1e-12));
    assert(near(g.gamma, 0.0));
    assert(near(gaitFrom(1.0, kTwoPi, 3.0).gamma, -0.25));
    assert(near(gaitFrom(1.0, kTwoPi, -6.0).gamma, 0.5));
    assert(near(gaitFrom(0.0, kTwoPi, 0.0).alpha, 0.0));
    assert(near(gaitFrom(0.5, kTwoPi, 0.0).alpha, 0.5 * std::sin(kPi / 12.0)));
}

void jointPhaseOffsets() {
    const Gait g = gaitFrom(1.0, kTwoPi, 0.0);  // beta = pi/6, so joint 3 sits a quarter wave ahead of joint 0
    const double top = 2.0 * g.alpha;
    assert(near(jointAngle(g, 0, 0.0), 0.0));
    assert(near(jointAngle(g, 3, 0.0), top));
    assert(near(jointAngle(g, 6, 0.0), 0.0, 1e-12));
    assert(near(jointAngle(g, 9, 0.0), -top));
    // Neighbours differ by exactly beta in phase: joint j+1 at time t is joint j at the phase t + beta.
    for (int j = 0; j + 1 < kLinks - 1; ++j) {
        for (const double t : {0.0, 0.7, 2.3, 5.1}) {
            assert(near(jointAngle(g, j + 1, t), jointAngle(g, j, t + g.beta), 1e-12));
        }
    }
    // The steering offset is added to every joint, nothing else changes.
    const Gait biased = gaitFrom(1.0, kTwoPi, 3.0);
    for (int j = 0; j < kLinks - 1; ++j) assert(near(jointAngle(biased, j, 1.3) - jointAngle(g, j, 1.3), -0.25, 1e-12));
}

void jointRateIsTheDerivative() {
    const Gait g = gaitFrom(0.8, 4.0, 1.0);
    const double h = 1e-6;
    for (int j = 0; j < kLinks - 1; ++j) {
        const double numeric = (jointAngle(g, j, 1.1 + 3.0 * h) - jointAngle(g, j, 1.1 - 3.0 * h)) / (2.0 * h);
        assert(near(jointRate(g, j, 1.1, 3.0), numeric, 1e-6));
    }
}

void linkAnglesAreATravelingWave() {
    // Summing the joint angles along the chain turns the sine into a cosine wave of amplitude a, plus one term that only
    // depends on time and tilts every link by the same amount: angle_k = a (cos(t - beta/2) - cos(t + (k - 1/2) beta)).
    const double a = 0.6;
    const Gait g = gaitFrom(a, kTwoPi, 0.0);
    for (const double t : {0.0, 0.7, 2.9}) {
        const Shape s = shapeAt(g, t, kPeriodOmega);
        for (int k = 0; k < kLinks; ++k) {
            const double expected = a * (std::cos(t - g.beta / 2.0) - std::cos(t + (k - 0.5) * g.beta));
            assert(near(s.angle[static_cast<std::size_t>(k)], expected, 1e-12));
        }
    }
}

void linkLengthsStayConstant() {
    for (const double len : {1.0, 2.5}) {
        for (const double t : {0.0, 0.4, 1.7, 4.2}) {
            const Gait g = gaitFrom(1.1, kTwoPi, 1.5);
            const Shape s = shapeAt(g, t, kPeriodOmega, len);
            for (int k = 0; k < kLinks; ++k) {
                const Vec2 a = s.joint[static_cast<std::size_t>(k)];
                const Vec2 b = s.joint[static_cast<std::size_t>(k) + 1];
                assert(near(length({b.x - a.x, b.y - a.y}), len, 1e-12));
                assert(nearVec(s.center[static_cast<std::size_t>(k)], {(a.x + b.x) / 2.0, (a.y + b.y) / 2.0}, 1e-12));
            }
            // The relative angle between two links is the joint angle, by construction.
            for (int j = 0; j < kLinks - 1; ++j) {
                assert(near(s.angle[static_cast<std::size_t>(j) + 1] - s.angle[static_cast<std::size_t>(j)], jointAngle(g, j, t), 1e-12));
            }
        }
    }
}

void flatSnakeIsAStraightLine() {
    const Shape s = shapeAt(gaitFrom(0.0, kTwoPi, 0.0), 1.234, kPeriodOmega, 1.0);
    for (int k = 0; k < kLinks; ++k) {
        assert(nearVec(s.center[static_cast<std::size_t>(k)], {k + 0.5, 0.0}));
        assert(nearVec(s.centerVel[static_cast<std::size_t>(k)], {0.0, 0.0}));
    }
    assert(nearVec(s.joint[static_cast<std::size_t>(kLinks)], {12.0, 0.0}));
}

void biasAloneBendsIntoAnArc() {
    // a = 0, c = 3: gamma = -0.25 on every joint, so link k points at k * gamma and the snake rests in a circular arc.
    const Gait g = gaitFrom(0.0, kTwoPi, 3.0);
    const Shape s = shapeAt(g, 0.0, kPeriodOmega, 1.0);
    for (int k = 0; k < kLinks; ++k) assert(near(s.angle[static_cast<std::size_t>(k)], -0.25 * k));
    // Link 1 starts at (1, 0) and points at -0.25 rad, its center is half a link further.
    assert(nearVec(s.joint[1], {1.0, 0.0}));
    assert(nearVec(s.center[1], {1.0 + 0.5 * std::cos(-0.25), 0.5 * std::sin(-0.25)}));
    assert(nearVec(s.centerVel[5], {0.0, 0.0}));
}

void centerVelocityIsTheDerivative() {
    const Gait g = gaitFrom(0.9, 5.0, 0.7);
    const double omega = 2.7;
    const double h = 1e-4;
    const double phase = 0.8;
    const Shape mid = shapeAt(g, phase, omega);
    const Shape lo = shapeAt(g, phase - omega * h, omega);
    const Shape hi = shapeAt(g, phase + omega * h, omega);
    for (std::size_t k = 0; k < static_cast<std::size_t>(kLinks); ++k) {
        assert(near(mid.centerVel[k].x, (hi.center[k].x - lo.center[k].x) / (2.0 * h), 1e-5));
        assert(near(mid.centerVel[k].y, (hi.center[k].y - lo.center[k].y) / (2.0 * h), 1e-5));
        assert(near(mid.angleRate[k], (hi.angle[k] - lo.angle[k]) / (2.0 * h), 1e-5));
    }
}

Shape flatShapeWithVelocity(Vec2 uniform) {
    Shape s = shapeAt(gaitFrom(0.0, kTwoPi, 0.0), 0.0, 0.0, 1.0);
    for (Vec2& v : s.centerVel) v = uniform;
    return s;
}

void solverCancelsUniformDrift() {
    // Every link slides sideways at (0, 1) in the body frame. Moving the whole body by (0, -1) cancels it exactly,
    // there is nothing left to resist, so the answer does not depend on the friction ratio.
    for (const double ratio : {1.0, 10.0, 1000.0}) {
        const Twist t = solveTwist(flatShapeWithVelocity({0.0, 1.0}), Pose{{0.0, 0.0}, 0.0}, ratio);
        assert(nearVec(t.v, {0.0, -1.0}, 1e-9) && near(t.w, 0.0, 1e-9));
        const Twist u = solveTwist(flatShapeWithVelocity({1.0, 0.0}), Pose{{0.0, 0.0}, 0.0}, ratio);
        assert(nearVec(u.v, {-1.0, 0.0}, 1e-9) && near(u.w, 0.0, 1e-9));
        // The body frame is turned by 90 degrees: a body-frame (0, 1) is a world (-1, 0), so the world twist is (1, 0).
        const Twist r = solveTwist(flatShapeWithVelocity({0.0, 1.0}), Pose{{3.0, -2.0}, kPi / 2.0}, ratio);
        assert(nearVec(r.v, {1.0, 0.0}, 1e-9) && near(r.w, 0.0, 1e-9));
    }
}

void solverCancelsAShapeRotation() {
    // The shape spins rigidly about the body origin at 0.5 rad/s: s_k = 0.5 * (-y, x). The body must spin back at -0.5.
    for (const double ratio : {1.0, 1000.0}) {
        Shape s = shapeAt(gaitFrom(0.0, kTwoPi, 0.0), 0.0, 0.0, 1.0);
        for (std::size_t k = 0; k < static_cast<std::size_t>(kLinks); ++k) s.centerVel[k] = {-0.5 * s.center[k].y, 0.5 * s.center[k].x};
        const Twist t = solveTwist(s, Pose{{0.0, 0.0}, 0.0}, ratio);
        assert(near(t.w, -0.5, 1e-9));
        assert(nearVec(t.v, {0.0, 0.0}, 1e-9));
    }
}

void staticShapeStaysPut() {
    // Nothing moves in the body frame, so nothing moves in the world either, whatever the friction.
    const Gait still = gaitFrom(0.0, kTwoPi, 2.0);
    for (const double ratio : {1.0, 1000.0}) {
        State s;
        s.pose = Pose{{1.0, 2.0}, 0.3};
        for (int i = 0; i < 2 * kStepsPerCycle; ++i) advance(s, still, kPeriodOmega, ratio, kDt);
        assert(nearVec(s.pose.pos, {1.0, 2.0}, 1e-12) && near(s.pose.theta, 0.3, 1e-12));
    }
}

void isotropicFrictionKeepsTheCenterOfMass() {
    // With the same friction in every direction the ground cannot push the snake anywhere: the velocities of the link
    // centers sum to zero at every instant. That holds in any pose and at any phase.
    const Gait g = gaitFrom(0.9, 2.0 * kTwoPi, 0.0);
    for (const double t : {0.0, 0.31, 1.7, 4.4}) {
        const Shape s = shapeAt(g, t, kPeriodOmega);
        const Pose pose{{5.0, -3.0}, 0.9 * t};
        const Twist tw = solveTwist(s, pose, 1.0);
        Vec2 sum{0.0, 0.0};
        for (int k = 0; k < kLinks; ++k) {
            const Vec2 v = linkVelocity(s, pose, tw, k);
            sum.x += v.x;
            sum.y += v.y;
        }
        assert(nearVec(sum, {0.0, 0.0}, 1e-9));
    }
    // Integrated over whole cycles the center of mass stays put, and the snake does not turn on the spot either.
    const Run r = run(0.6, kTwoPi, 0.0, 1.0, kPeriodOmega);
    assert(length(r.displacement) < 5e-3);
    assert(std::fabs(r.headingChange) < 1e-3);
}

void wheelsPropelTheSnake() {
    const Run r = run(0.6, kTwoPi, 0.0, 1000.0, kPeriodOmega);
    std::printf("wheels (ratio 1000): %.3f body lengths per cycle forward, %.4f sideways\n", r.forward, r.sideways);
    assert(r.forward > 0.80 && r.forward < 0.95);        // measured 0.879 for this wave
    assert(std::fabs(r.sideways) < 0.01 * r.forward);    // straight: no drift across the heading
    assert(std::fabs(r.headingChange) < 1e-3);           // gamma = 0 does not turn the snake
}

void progressGrowsWithTheFrictionRatio() {
    double previous = -1.0;
    double ice = 0.0;
    double wheels = 0.0;
    for (const double ratio : {1.0, 3.0, 10.0, 30.0, 100.0, 1000.0}) {
        const Run r = run(0.6, kTwoPi, 0.0, ratio, kPeriodOmega);
        std::printf("ratio %6.0f: %.3f body lengths per cycle\n", ratio, r.forward);
        assert(r.forward > previous);
        previous = r.forward;
        if (ratio == 1.0) ice = r.forward;
        if (ratio == 1000.0) wheels = r.forward;
    }
    assert(std::fabs(ice) < 0.005 * wheels);  // equal friction: wiggling in place
}

void strideFollowsTheWavelengthNotTheBend() {
    assert(std::fabs(run(0.0, kTwoPi, 0.0, 1000.0, kPeriodOmega).forward) < 1e-9);  // no wave, no progress
    // With (almost) perfect wheels the snake advances about one wavelength per cycle, however small the bend. A bigger
    // bend makes the body path longer and costs a little stride. Measured, not derived.
    double last = 2.0;
    for (const double a : {0.1, 0.3, 0.6, 0.9, 1.2}) {
        const double f = run(a, kTwoPi, 0.0, 1e5, kPeriodOmega).forward;
        std::printf("bend a = %.1f: %.3f body lengths per cycle (ratio 1e5)\n", a, f);
        assert(f > 0.6 && f < 1.0);
        assert(f < last);
        last = f;
    }
    // At the demo's ratio of 1000 a tiny bend pushes so gently that the friction along the link already holds it back.
    const double gentle = run(0.1, kTwoPi, 0.0, 1000.0, kPeriodOmega).forward;
    std::printf("bend a = 0.1 at ratio 1000: %.3f body lengths per cycle\n", gentle);
    assert(gentle < 0.8 * run(0.1, kTwoPi, 0.0, 1e5, kPeriodOmega).forward);
    // Two waves on the body halve the wavelength, so the stride is about half.
    for (const double waves : {0.5, 1.0, 1.5, 2.0}) {
        const double f = run(0.6, kTwoPi * waves, 0.0, 1000.0, kPeriodOmega).forward;
        std::printf("%.1f waves on the body: %.3f body lengths per cycle (1 / waves = %.3f)\n", waves, f, 1.0 / waves);
        assert(f > 0.8 / waves && f < 1.0 / waves);
    }
}

void reversedWaveReversesTheSnake() {
    const Run fwd = run(0.6, kTwoPi, 0.0, 1000.0, kPeriodOmega);
    const Run back = run(0.6, kTwoPi, 0.0, 1000.0, -kPeriodOmega);
    assert(back.forward < 0.0);
    assert(near(back.forward, -fwd.forward, 0.01 * fwd.forward));
}

void steeringTurnsTheSnake() {
    const Run straight = run(0.6, kTwoPi, 0.0, 1000.0, kPeriodOmega, 4, 1);
    const Run half = run(0.6, kTwoPi, 0.5, 1000.0, kPeriodOmega, 4, 1);
    const Run one = run(0.6, kTwoPi, 1.0, 1000.0, kPeriodOmega, 4, 1);
    const Run two = run(0.6, kTwoPi, 2.0, 1000.0, kPeriodOmega, 4, 1);
    const Run minusOne = run(0.6, kTwoPi, -1.0, 1000.0, kPeriodOmega, 4, 1);
    std::printf("heading change per cycle: c=0.5 %.3f  c=1 %.3f  c=2 %.3f  c=-1 %.3f rad\n", half.headingChange, one.headingChange, two.headingChange, minusOne.headingChange);
    assert(std::fabs(straight.headingChange) < 1e-3);
    assert(one.headingChange < -0.5);                                              // c > 0 turns clockwise in y-up coordinates
    assert(minusOne.headingChange > 0.5);                                          // the mirror image
    assert(near(minusOne.headingChange, -one.headingChange, 2e-3));
    assert(near(half.headingChange * 2.0, one.headingChange, 0.02 * std::fabs(one.headingChange)));  // linear in c
    assert(near(one.headingChange * 2.0, two.headingChange, 0.02 * std::fabs(two.headingChange)));
    assert(length(one.displacement) < length(straight.displacement));              // a circle gets nowhere fast
}

void poseForPlacesTheSnake() {
    const Gait g = gaitFrom(0.8, kTwoPi, 0.4);
    const Shape s = shapeAt(g, 1.9, kPeriodOmega);
    const Pose p = poseFor(s, {100.0, -50.0}, 1.0);
    assert(near(heading(s, p), 1.0, 1e-12));
    assert(nearVec(centerOfMass(s, p), {100.0, -50.0}, 1e-9));
}

void wrapPhaseStaysInOnePeriod() {
    assert(near(wrapPhase(0.5), 0.5));
    assert(near(wrapPhase(kTwoPi + 0.5), 0.5));
    assert(near(wrapPhase(-0.5), kTwoPi - 0.5));
    assert(near(wrapPhase(0.0), 0.0));
    for (const double big : {1e7, -1e7, 123456.789}) {
        const double w = wrapPhase(big);
        assert(w >= 0.0 && w < kTwoPi);
    }
}

void badFrictionRatioIsClamped() {
    const Gait g = gaitFrom(0.6, kTwoPi, 0.0);
    const Shape s = shapeAt(g, 0.4, kPeriodOmega);
    const Pose pose{{0.0, 0.0}, 0.2};
    const Twist one = solveTwist(s, pose, 1.0);
    for (const double bad : {0.0, -5.0, 0.5}) {
        const Twist t = solveTwist(s, pose, bad);
        assert(nearVec(t.v, one.v, 1e-12) && near(t.w, one.w, 1e-12));
    }
    const Twist huge = solveTwist(s, pose, 1e30);
    assert(std::isfinite(huge.v.x) && std::isfinite(huge.v.y) && std::isfinite(huge.w));
}

void wheelsStopSideSlip() {
    // The sideways speed of the link centers is what the wheels forbid: over one cycle it is far smaller than on ice.
    const Gait g = gaitFrom(0.6, kTwoPi, 0.0);
    double iceWorst = 0.0;
    double wheelWorst = 0.0;
    for (const double ratio : {1.0, 1000.0}) {
        State s;
        s.pose = poseFor(shapeAt(g, 0.0, kPeriodOmega), {0.0, 0.0}, 0.0);
        double worst = 0.0;
        for (int i = 0; i < kStepsPerCycle; ++i) {
            const Shape sh = shapeAt(g, s.phase, kPeriodOmega);
            const Twist tw = solveTwist(sh, s.pose, ratio);
            for (int k = 0; k < kLinks; ++k) worst = std::max(worst, std::fabs(slipOf(sh, s.pose, tw, k).across));
            advance(s, g, kPeriodOmega, ratio, kDt);
        }
        (ratio == 1.0 ? iceWorst : wheelWorst) = worst;
    }
    std::printf("worst sideways slip speed: ice %.3f, wheels %.3f (link lengths per second)\n", iceWorst, wheelWorst);
    assert(wheelWorst < 0.05 * iceWorst);
    assert(iceWorst > 0.5);
}

void slipSplitsTheVelocity() {
    // A link pointing along x moving at (3, 4): 3 along, 4 across.
    Shape s = shapeAt(gaitFrom(0.0, kTwoPi, 0.0), 0.0, 0.0, 1.0);
    s.centerVel[0] = {3.0, 4.0};
    const LinkSlip sl = slipOf(s, Pose{{0.0, 0.0}, 0.0}, Twist{{0.0, 0.0}, 0.0}, 0);
    assert(near(sl.along, 3.0) && near(sl.across, 4.0));
    // Same link, body turned by 90 degrees: the link points along +y and the body-frame velocity turns with it.
    const LinkSlip turned = slipOf(s, Pose{{0.0, 0.0}, kPi / 2.0}, Twist{{0.0, 0.0}, 0.0}, 0);
    assert(near(turned.along, 3.0) && near(turned.across, 4.0));
    // A world velocity of the body adds in the world frame: +2 along x on top of 3 along, 4 across.
    const LinkSlip moving = slipOf(s, Pose{{0.0, 0.0}, 0.0}, Twist{{2.0, 0.0}, 0.0}, 0);
    assert(near(moving.along, 5.0) && near(moving.across, 4.0));
}

void speedMeterMeasuresOneWindow() {
    SpeedMeter m(2.0);
    assert(!m.displacement().has_value());
    // Moving at (3, 4) per second, sampled at 120 Hz.
    double t = 0.0;
    for (int i = 0; i <= 120; ++i) {
        t = i / 120.0;
        m.push(t, {3.0 * t, 4.0 * t});
    }
    assert(!m.displacement().has_value());  // only one second so far, the window is two
    for (int i = 121; i <= 480; ++i) {
        t = i / 120.0;
        m.push(t, {3.0 * t, 4.0 * t});
    }
    const auto d = m.displacement();
    assert(d.has_value() && nearVec(*d, {6.0, 8.0}, 1e-9));
    // Now it turns and moves at (0, 1): after one more window only that counts.
    double y = 4.0 * t;
    const double x = 3.0 * t;
    const double t0 = t;
    for (int i = 1; i <= 240; ++i) {
        t = t0 + i / 120.0;
        y = 4.0 * t0 + (t - t0);
        m.push(t, {x, y});
    }
    const auto e = m.displacement();
    assert(e.has_value() && nearVec(*e, {0.0, 2.0}, 1e-9));
    m.reset();
    assert(!m.displacement().has_value());  // a new basis starts a new measurement
}

void speedMeterScalesAWindowThatIsNotAMultipleOfTheStep() {
    SpeedMeter m(1.0);
    for (int i = 0; i <= 40; ++i) {
        const double t = 0.3 * i;  // step 0.3 s, so the sample before t - 1 is up to 0.3 s older than the window
        m.push(t, {2.0 * t, 0.0});
    }
    const auto d = m.displacement();
    assert(d.has_value() && nearVec(*d, {2.0, 0.0}, 1e-9));
}

void sliderMapping() {
    assert(near(sliderFraction(0.6, 0.0, 1.2), 0.5));
    assert(near(sliderFraction(-1.0, 0.0, 1.2), 0.0));
    assert(near(sliderFraction(9.0, 0.0, 1.2), 1.0));
    assert(near(sliderFraction(1.0, 1.0, 1.0), 0.0));  // empty range: no division by zero
    assert(near(sliderValue(100.0, 100.0, 200.0, -2.0, 2.0), -2.0));
    assert(near(sliderValue(200.0, 100.0, 200.0, -2.0, 2.0), 0.0));
    assert(near(sliderValue(300.0, 100.0, 200.0, -2.0, 2.0), 2.0));
    assert(near(sliderValue(50.0, 100.0, 200.0, -2.0, 2.0), -2.0));   // left of the track clamps
    assert(near(sliderValue(900.0, 100.0, 200.0, -2.0, 2.0), 2.0));   // right of the track clamps
    assert(near(sliderValue(150.0, 100.0, 0.0, -2.0, 2.0), -2.0));    // zero-width track
}

}  // namespace

int main() {
    std::setvbuf(stdout, nullptr, _IONBF, 0);  // keep the measurements visible when an assert aborts
    gaitMapping();
    jointPhaseOffsets();
    jointRateIsTheDerivative();
    linkLengthsStayConstant();
    linkAnglesAreATravelingWave();
    flatSnakeIsAStraightLine();
    biasAloneBendsIntoAnArc();
    centerVelocityIsTheDerivative();
    solverCancelsUniformDrift();
    solverCancelsAShapeRotation();
    staticShapeStaysPut();
    isotropicFrictionKeepsTheCenterOfMass();
    wheelsPropelTheSnake();
    progressGrowsWithTheFrictionRatio();
    strideFollowsTheWavelengthNotTheBend();
    reversedWaveReversesTheSnake();
    steeringTurnsTheSnake();
    poseForPlacesTheSnake();
    wrapPhaseStaysInOnePeriod();
    badFrictionRatioIsClamped();
    wheelsStopSideSlip();
    slipSplitsTheVelocity();
    speedMeterMeasuresOneWindow();
    speedMeterScalesAWindowThatIsNotAMultipleOfTheStep();
    sliderMapping();
    std::puts("snake-serpenoid logic: all tests passed");
    return 0;
}
