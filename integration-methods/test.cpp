// Plain-assert tests for the integration methods. Expected values are worked out by hand in the comment above them.
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>

#include "logic.hpp"

namespace {

int failures = 0;

#define CHECK(cond)                                                     \
    do {                                                                \
        if (!(cond)) {                                                  \
            std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); \
            ++failures;                                                 \
        }                                                               \
    } while (false)

bool near(double a, double b, double eps = 1e-9) { return std::fabs(a - b) <= eps; }

using namespace im;

constexpr double kPi = 3.14159265358979323846;

Body stepped(Kind kind, Method method, double dt, long n) {
    Body b = start(kind, dt);
    for (long i = 0; i < n; ++i) step(kind, method, dt, b);
    return b;
}

// ---- start state and forces ----

void startState() {
    const Body o = start(Kind::Orbit, 0.1);
    CHECK(near(o.pos.x, 1.0) && near(o.pos.y, 0.0));
    CHECK(near(o.vel.x, 0.0) && near(o.vel.y, 1.0));
    // Verlet needs the position one step back: pos - vel*dt + a*dt^2/2 = (1,0) - (0,0.1) + (-1,0)*0.005 = (0.995, -0.1).
    CHECK(near(o.prev.x, 0.995) && near(o.prev.y, -0.1));
    // Circular orbit: E = v^2/2 - GM/r = 0.5 - 1.
    CHECK(near(energy(Kind::Orbit, o), -0.5));

    const Body s = start(Kind::Spring, 0.1);
    // a = -4 at x = 1, so prev = 1 - 0 + 0.5 * (-4) * 0.01 = 0.98.
    CHECK(near(s.prev.x, 0.98));
    // E = 0.5 * w^2 * x^2 = 0.5 * 4 * 1.
    CHECK(near(energy(Kind::Spring, s), 2.0));
}

void forces() {
    // Orbit: a = -GM p / |p|^3. At (2,0): -1/4. At (0,0.5): -0.5 / 0.125 = -4 (pointing at the sun).
    CHECK(near(accel(Kind::Orbit, {2.0, 0.0}).x, -0.25));
    CHECK(near(accel(Kind::Orbit, {0.0, 0.5}).y, -4.0));
    // Spring: a = -w^2 p = -4 p.
    CHECK(near(accel(Kind::Spring, {1.0, 0.0}).x, -4.0));
    CHECK(near(accel(Kind::Spring, {-0.5, 0.0}).x, 2.0));
    // Right on the sun there is no force and no NaN.
    const Vec2 a = accel(Kind::Orbit, {0.0, 0.0});
    CHECK(finite(a) && a.x == 0.0 && a.y == 0.0);
}

// ---- one step, by hand ----

void oneStepOnTheSpring() {
    // x = 1, v = 0, dt = 0.1, a = -4.
    const Body e = stepped(Kind::Spring, Method::Explicit, 0.1, 1);
    CHECK(near(e.pos.x, 1.0) && near(e.vel.x, -0.4));  // position moved with the old velocity 0
    const Body s = stepped(Kind::Spring, Method::SemiImplicit, 0.1, 1);
    CHECK(near(s.vel.x, -0.4) && near(s.pos.x, 0.96));  // v = -0.4 first, then x = 1 - 0.4 * 0.1
    // Verlet: next = 2*1 - 0.98 + (-4)*0.01 = 0.98. Velocity from the Taylor step back: (0.98-1)/0.1 + 0.5*(-3.92)*0.1 = -0.396.
    const Body v = stepped(Kind::Spring, Method::Verlet, 0.1, 1);
    CHECK(near(v.pos.x, 0.98) && near(v.vel.x, -0.396) && near(v.prev.x, 1.0));
    // RK4: k1x=0 k1v=-4 | k2x=-0.2 k2v=-4 | k3x=-0.2 k3v=-3.96 | k4x=-0.396 k4v=-3.92.
    // x = 1 + 0.1/6 * (0 - 0.4 - 0.4 - 0.396) = 0.98006667, v = 0.1/6 * (-4 - 8 - 7.92 - 3.92) = -0.39733333.
    const Body r = stepped(Kind::Spring, Method::RK4, 0.1, 1);
    CHECK(near(r.pos.x, 0.98006666667, 1e-9) && near(r.vel.x, -0.39733333333, 1e-9));
    // The exact answer is cos(0.2) = 0.9800666 and -2 sin(0.2) = -0.3973387: RK4 is within 6e-6, the others are not.
    CHECK(near(r.pos.x, std::cos(0.2), 1e-6));
    CHECK(!near(e.pos.x, std::cos(0.2), 1e-3) && !near(s.pos.x, std::cos(0.2), 1e-3));
}

void oneStepOnTheOrbit() {
    // (1,0), v = (0,1), a = (-1,0), dt = 0.1.
    const Body e = stepped(Kind::Orbit, Method::Explicit, 0.1, 1);
    CHECK(near(e.pos.x, 1.0) && near(e.pos.y, 0.1));
    CHECK(near(e.vel.x, -0.1) && near(e.vel.y, 1.0));
    const Body s = stepped(Kind::Orbit, Method::SemiImplicit, 0.1, 1);
    CHECK(near(s.vel.x, -0.1) && near(s.vel.y, 1.0));
    CHECK(near(s.pos.x, 0.99) && near(s.pos.y, 0.1));  // x = 1 + (-0.1) * 0.1
    // Explicit after one step: |p|^2 = 1.01, |v|^2 = 1.01. E = 0.505 - 1/sqrt(1.01) = 0.505 - 0.99503719 = -0.49003719.
    // Drift = (E - E0) / |E0| = (-0.49003719 + 0.5) / 0.5 = 0.01992562.
    Run run;
    run.restart(Kind::Orbit, 0.1);
    run.step();
    CHECK(near(run.driftOf(0), 0.01992562, 1e-7));
}

// ---- what the methods do to the energy ----

void explicitPumpsEnergyOnTheSpring() {
    // One explicit step multiplies the spring energy by exactly 1 + (w dt)^2 = 1.04 at dt = 0.1:
    // E' = E + w^2 dt^2 E. After n steps the drift is 1.04^n - 1, whatever the start.
    Run run;
    run.restart(Kind::Spring, 0.1);
    for (int n = 1; n <= 40; ++n) {
        run.step();
        CHECK(near(run.driftOf(0), std::pow(1.04, n) - 1.0, 1e-9 * std::pow(1.04, n)));
    }
    CHECK(run.driftOf(0) > 3.8);  // 1.04^40 = 4.80
}

void semiImplicitStaysBounded() {
    // Semi-implicit Euler conserves v^2 + w^2 x^2 - w^2 dt x v exactly. From that, with s = w dt:
    // E/E0 lies between 1 - s/(2+s) and 1 + s/(2-s), so the drift stays inside [-x/(1+x), +x/(1-x)] with x = s/2.
    // At dt = 0.1: s = 0.2, x = 0.1, so [-0.0909, +0.1111]. Forever, not just for a while.
    const double dt = 0.1;
    const double x = kOmega * dt / 2.0;
    const double lo = -x / (1.0 + x);
    const double hi = x / (1.0 - x);
    Run run;
    run.restart(Kind::Spring, dt);
    double seenLo = 0.0;
    double seenHi = 0.0;
    for (int i = 0; i < 6000; ++i) {
        run.step();
        const double d = run.driftOf(1);
        CHECK(d >= lo - 1e-9 && d <= hi + 1e-9);
        seenLo = std::min(seenLo, d);
        seenHi = std::max(seenHi, d);
    }
    CHECK(seenHi > 0.95 * hi && seenLo < 0.95 * lo);  // the bound is not slack, the energy really swings that far
    // Explicit is far outside the same band after the same time (it is gone, in fact).
    CHECK(run.driftOf(0) > 1000.0);
}

void verletAndRk4KeepTheOrbit() {
    // One period of the circular orbit at dt = 2 pi / 628, all start at (1,0).
    const double dt = 2.0 * kPi / 628.0;
    const Body e = stepped(Kind::Orbit, Method::Explicit, dt, 628);
    const Body s = stepped(Kind::Orbit, Method::SemiImplicit, dt, 628);
    const Body v = stepped(Kind::Orbit, Method::Verlet, dt, 628);
    const Body r = stepped(Kind::Orbit, Method::RK4, dt, 628);
    CHECK(length(e.pos) > 1.1 && length(e.pos) < 1.15);  // explicit: one lap and the radius is 12 % larger
    CHECK(near(length(s.pos), 1.0, 1e-5));
    CHECK(near(length(v.pos), 1.0, 1e-5));
    CHECK(near(r.pos.x, 1.0, 1e-7) && near(r.pos.y, 0.0, 1e-7));  // back at the start
}

double orbitPeriod(Method method, double dt) {
    // Time between the first and the eleventh upward crossing of y = 0 on the right side, linearly interpolated.
    Body b = start(Kind::Orbit, dt);
    int crossings = 0;
    double first = 0.0;
    double last = 0.0;
    for (long i = 1; i < 200000 && crossings < 11; ++i) {
        const double y0 = b.pos.y;
        step(Kind::Orbit, method, dt, b);
        if (y0 < 0.0 && b.pos.y >= 0.0 && b.pos.x > 0.0) {
            const double t = (static_cast<double>(i) - b.pos.y / (b.pos.y - y0)) * dt;
            if (crossings == 0) first = t;
            last = t;
            ++crossings;
        }
    }
    return crossings == 11 ? (last - first) / 10.0 : -1.0;
}

void orbitPeriods() {
    // The true period of a circular orbit of radius 1 with GM = 1 is 2 pi.
    const double rk4 = orbitPeriod(Method::RK4, 0.1);
    const double verlet = orbitPeriod(Method::Verlet, 0.1);
    const double semi = orbitPeriod(Method::SemiImplicit, 0.1);
    CHECK(near(rk4, 2.0 * kPi, 1e-3));
    CHECK(verlet > 6.29 && verlet < 6.32);  // a little slow
    CHECK(semi > 6.31 && semi < 6.34);      // a little slower
    CHECK(std::fabs(rk4 - 2.0 * kPi) < std::fabs(verlet - 2.0 * kPi));
    CHECK(std::fabs(verlet - 2.0 * kPi) < std::fabs(semi - 2.0 * kPi));
}

void orderOfAccuracy() {
    // Error of x(1) against cos(2) on the spring. Halving dt divides the error by 2^order:
    // explicit and semi-implicit are first order (2), Verlet second (4), RK4 fourth (16).
    const double expected[] = {2.0, 2.0, 4.0, 16.0};
    const double tolerance[] = {0.2, 0.2, 0.2, 2.0};
    for (std::size_t m = 0; m < kMethods; ++m) {
        const auto method = static_cast<Method>(m);
        const double coarse = std::fabs(stepped(Kind::Spring, method, 0.02, 50).pos.x - std::cos(2.0));
        const double fine = std::fabs(stepped(Kind::Spring, method, 0.01, 100).pos.x - std::cos(2.0));
        CHECK(coarse > fine);
        CHECK(std::fabs(coarse / fine - expected[m]) < tolerance[m]);
    }
}

void verletRunsBackwards() {
    // Swap the last two positions and the same rule retraces the path. Explicit Euler cannot do that.
    const double dt = 0.1;
    Body v = start(Kind::Orbit, dt);
    const Body first = v;
    for (int i = 0; i < 500; ++i) step(Kind::Orbit, Method::Verlet, dt, v);
    std::swap(v.pos, v.prev);
    for (int i = 0; i < 500; ++i) step(Kind::Orbit, Method::Verlet, dt, v);
    CHECK(near(v.pos.x, first.prev.x, 1e-9) && near(v.pos.y, first.prev.y, 1e-9));

    Body e = start(Kind::Orbit, dt);
    for (int i = 0; i < 500; ++i) step(Kind::Orbit, Method::Explicit, dt, e);
    CHECK(energy(Kind::Orbit, e) > -0.3);  // and its energy has grown from -0.5 towards an escape
}

// ---- guards ----

void flownAwayIsFrozen() {
    // Explicit on the spring at dt = 0.4 multiplies the amplitude by sqrt(1.64) = 1.28 per step: beyond 1e4 after 38 steps.
    Body b = start(Kind::Spring, 0.4);
    long i = 0;
    while (!b.gone && i < 1000) {
        step(Kind::Spring, Method::Explicit, 0.4, b);
        ++i;
    }
    CHECK(b.gone && i < 60 && finite(b.pos));
    const Vec2 frozen = b.pos;
    step(Kind::Spring, Method::Explicit, 0.4, b);
    CHECK(b.pos.x == frozen.x && b.pos.y == frozen.y);

    Run run;
    run.restart(Kind::Spring, 0.4);
    for (int k = 0; k < 100; ++k) run.step();
    CHECK(run.body[0].gone && run.driftOf(0) == kGoneDrift);
    CHECK(!run.body[3].gone);
}

void closePassStaysFinite() {
    for (std::size_t m = 0; m < kMethods; ++m) {
        Body b;
        b.pos = {1e-9, 0.0};
        b.vel = {0.0, 1.0};
        b.prev = b.pos;
        for (int i = 0; i < 5; ++i) step(Kind::Orbit, static_cast<Method>(m), 0.1, b);
        CHECK(finite(b.pos) && finite(b.vel));
    }
}

void badStepSizeDoesNothing() {
    Body b = start(Kind::Spring, 0.1);
    for (double dt : {0.0, -0.1, std::numeric_limits<double>::quiet_NaN()}) {
        for (std::size_t m = 0; m < kMethods; ++m) {
            Body c = b;
            step(Kind::Spring, static_cast<Method>(m), dt, c);
            CHECK(c.pos.x == b.pos.x && c.vel.x == b.vel.x && !c.gone);
        }
    }
}

// ---- samples, clock, plot scale ----

void seriesThinsAndPrunes() {
    Series<double> s;
    s.push(0.0, 1.0, 0.05);
    s.push(0.02, 2.0, 0.05);  // too close to the last sample
    s.push(0.05, 3.0, 0.05);  // exactly the gap: kept
    CHECK(s.items.size() == 2 && near(s.items.back().value, 3.0));
    s.push(0.06, 4.0, 0.0);  // gap 0 keeps everything
    CHECK(s.items.size() == 3);
    s.prune(10.0, 9.96);  // keeps t >= 0.04
    CHECK(s.items.size() == 2 && near(s.items.front().t, 0.05));
    s.prune(100.0, 1.0);
    CHECK(s.items.empty());
}

void runRecordsAWindow() {
    // dt = 0.02 and a sample gap of 0.05: samples at 0, 0.06, 0.12 ... 0.96 after one second (50 steps) = 17.
    Run run;
    run.restart(Kind::Spring, 0.02);
    CHECK(run.drift[0].items.size() == 1);
    for (int i = 0; i < 50; ++i) run.step();
    CHECK(near(run.time(), 1.0, 1e-12));
    CHECK(run.drift[3].items.size() == 17);
    CHECK(run.trail[3].items.size() == 51);  // the trail keeps every step
    // After 30 s at dt = 0.1 only the last 20 s of energy and the last 7 s of trail are left.
    run.restart(Kind::Orbit, 0.1);
    for (int i = 0; i < 300; ++i) run.step();
    CHECK(run.drift[3].items.front().t >= 10.0 - 1e-9 && run.drift[3].items.size() == 201);
    CHECK(run.trail[3].items.front().t >= 23.0 - 1e-9 && run.trail[3].items.size() == 71);
    // A restart clears everything.
    run.restart(Kind::Orbit, 0.1);
    CHECK(run.steps == 0 && run.trail[2].items.size() == 1 && run.drift[2].items.size() == 1 && near(run.driftOf(2), 0.0));
}

void stepsFromTheAccumulator() {
    double acc = 0.0;
    CHECK(takeSteps(acc, 0.25, 0.1, 64) == 2 && near(acc, 0.05));  // 2.5 steps: two now, half a step stays
    CHECK(takeSteps(acc, 0.25, 0.1, 64) == 3 && near(acc, 0.0));   // 0.30 / 0.1 is 2.9999999999999996 in floating point, still 3
    CHECK(takeSteps(acc, 0.04, 0.1, 64) == 0 && near(acc, 0.04));  // less than a step: nothing yet
    acc = 0.0;
    CHECK(takeSteps(acc, 1000.0, 0.1, 64) == 64);                   // capped
    CHECK(acc >= 0.0 && acc < 0.1);                                 // and the backlog is gone, only the fraction stays
    acc = 0.03;
    CHECK(takeSteps(acc, -5.0, 0.1, 64) == 0 && near(acc, 0.03));  // negative input ignored
    CHECK(takeSteps(acc, std::numeric_limits<double>::quiet_NaN(), 0.1, 64) == 0 && near(acc, 0.03));
    CHECK(takeSteps(acc, 1.0, 0.0, 64) == 0);                       // dt <= 0 never divides
    CHECK(takeSteps(acc, 1.0, -0.1, 64) == 0);
}

void worldRunsBothSystemsInLockstep() {
    World w;
    w.restart(0.1);
    // 0.1 s of frame time at 2x is 0.2 s of simulation = two steps of 0.1.
    CHECK(w.advance(0.1) == 2);
    CHECK(w.orbit.steps == 2 && w.spring.steps == 2);
    // A frame of 10 s (tab in the background) counts as 0.25 s: 0.5 s of simulation = 5 steps.
    CHECK(w.advance(10.0) == 5);
    CHECK(w.orbit.steps == 7 && w.spring.steps == 7);
    CHECK(w.advance(-1.0) == 0 && w.advance(std::numeric_limits<double>::quiet_NaN()) == 0);
    // A restart drops the leftover time, so the first step after it is not early.
    CHECK(w.advance(0.025) == 0);  // 0.05 s of simulation, half a step stays in the accumulator
    w.restart(0.1);
    CHECK(near(w.acc, 0.0) && w.orbit.steps == 0);
    CHECK(w.advance(0.025) == 0);
    // Warm-up runs simulated time at once: 6 s at dt = 0.1 is 60 steps, the clock reads 6.
    w.restart(0.1);
    w.warmUp(6.0);
    CHECK(w.orbit.steps == 60 && near(w.orbit.time(), 6.0, 1e-12));
}

void symlogScale() {
    CHECK(symlog(0.0) == 0.0);
    CHECK(near(symlog(1.0), 1.0) && near(symlog(-1.0), -1.0));
    CHECK(symlog(50.0) == 1.0 && symlog(-50.0) == -1.0);  // beyond 100 % the line sits on the edge
    CHECK(symlog(kGoneDrift) == 1.0);
    // Each decade is one quarter of the height: 1 % = log10(1 + 100) / log10(10001) = 2.00432 / 4.00004 = 0.50106.
    CHECK(near(symlog(0.01), 0.50106, 1e-4));
    CHECK(near(symlog(0.1), 0.75, 1e-3));
    CHECK(near(symlog(-0.01), -symlog(0.01)));
    CHECK(symlog(0.001) > symlog(0.0005) && symlog(0.0005) > symlog(0.0));  // strictly increasing
    CHECK(symlog(std::numeric_limits<double>::quiet_NaN()) == 0.0);
    CHECK(symlog(std::numeric_limits<double>::infinity()) == 1.0);
}

void formatting() {
    CHECK(formatDrift(0.0) == "+0.00%");
    CHECK(formatDrift(-1e-9) == "+0.00%");  // no minus sign in front of a zero
    CHECK(formatDrift(0.00123) == "+0.12%");
    CHECK(formatDrift(-0.0538) == "-5.38%");
    CHECK(formatDrift(0.45) == "+45.0%");
    CHECK(formatDrift(-0.9935) == "-99.4%");
    CHECK(formatDrift(9.5) == "+950.0%");
    CHECK(formatDrift(12.0) == ">+1000%");
    CHECK(formatDrift(-12.0) == "<-1000%");
    CHECK(formatDrift(0.0999) == "+9.99%");
    CHECK(formatDrift(std::numeric_limits<double>::quiet_NaN()) == "gone");
    CHECK(formatDrift(std::numeric_limits<double>::infinity()) == "gone");
}

}  // namespace

int main() {
    startState();
    forces();
    oneStepOnTheSpring();
    oneStepOnTheOrbit();
    explicitPumpsEnergyOnTheSpring();
    semiImplicitStaysBounded();
    verletAndRk4KeepTheOrbit();
    orbitPeriods();
    orderOfAccuracy();
    verletRunsBackwards();
    flownAwayIsFrozen();
    closePassStaysFinite();
    badStepSizeDoesNothing();
    seriesThinsAndPrunes();
    runRecordsAWindow();
    stepsFromTheAccumulator();
    worldRunsBothSystemsInLockstep();
    symlogScale();
    formatting();
    if (failures != 0) {
        std::printf("%d check(s) failed\n", failures);
        return 1;
    }
    std::printf("all integration-methods checks passed\n");
    return 0;
}
