// Plain-assert tests for the n-body logic. Every expected value is worked out by hand in the comment above it.
// Where a value is only bounded (energy drift of a method), the bound comes from the error order of the method
// and carries a wide margin, because FMA and ARM round differently from x86.
#include <cmath>
#include <cstdio>
#include <limits>

#include "logic.hpp"

namespace {

int failures = 0;

#define CHECK(cond)                                                                \
    do {                                                                           \
        if (!(cond)) {                                                             \
            std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);            \
            ++failures;                                                            \
        }                                                                          \
    } while (false)

bool near(double a, double b, double eps = 1e-9) { return std::fabs(a - b) <= eps; }

using namespace nb;

System twoEqual(double dist) {
    System s;
    s.add(Body{{-dist / 2.0, 0.0}, {0.0, 0.0}, 1.0});
    s.add(Body{{dist / 2.0, 0.0}, {0.0, 0.0}, 1.0});
    return s;
}

// ---- forces ----

void accelerationsTwoBodies() {
    // G = 1, m = 1 each, distance 1: |a| = G m / r^2 = 1, each pulled towards the other.
    System s = twoEqual(1.0);
    Accel a;
    accelerations(s, a);
    CHECK(near(a[0].x, 1.0) && near(a[0].y, 0.0));
    CHECK(near(a[1].x, -1.0) && near(a[1].y, 0.0));

    // Masses 1 and 3, distance 2 on the x axis: body 0 feels G*3/4 = 0.75, body 1 feels G*1/4 = 0.25 (opposite sign).
    System t;
    t.add(Body{{0.0, 0.0}, {0.0, 0.0}, 1.0});
    t.add(Body{{2.0, 0.0}, {0.0, 0.0}, 3.0});
    accelerations(t, a);
    CHECK(near(a[0].x, 0.75));
    CHECK(near(a[1].x, -0.25));

    // 3-4-5 diagonal: distance 5, m = 1 and 1: |a| = 1/25 = 0.04, direction (0.6, 0.8).
    System d;
    d.add(Body{{0.0, 0.0}, {0.0, 0.0}, 1.0});
    d.add(Body{{3.0, 4.0}, {0.0, 0.0}, 1.0});
    accelerations(d, a);
    CHECK(near(a[0].x, 0.024) && near(a[0].y, 0.032));
}

void softening() {
    // Plummer softening: a = G m d / (d^2 + eps^2)^1.5. d = 1, eps = 1: 1 / 2^1.5 = 0.353553.
    System s = twoEqual(1.0);
    s.softening = 1.0;
    Accel a;
    accelerations(s, a);
    CHECK(near(a[0].x, 0.3535533906, 1e-9));
    // Potential: -G m1 m2 / sqrt(d^2 + eps^2) = -1 / sqrt(2).
    CHECK(near(potentialEnergy(s), -0.7071067812, 1e-9));
    // eps = 0 gives the textbook values back.
    s.softening = 0.0;
    CHECK(near(potentialEnergy(s), -1.0));

    // Near collision, distance 0.001. Naive: 1 / d^2 = 1e6. Softened with eps = 0.05:
    // d^2 + eps^2 = 0.002501, ^1.5 = 1.25075e-4, a = 0.001 / 1.25075e-4 = 7.9952.
    System n = twoEqual(0.001);
    accelerations(n, a);
    CHECK(near(a[0].x, 1.0e6, 1.0));
    n.softening = 0.05;
    accelerations(n, a);
    CHECK(near(a[0].x, 7.9952, 1e-3));

    // The softened force never exceeds its maximum 2 / (3 sqrt 3 eps^2) = 0.3849 / eps^2, reached at d = eps / sqrt 2.
    const double eps = 0.05;
    const double bound = 2.0 / (3.0 * std::sqrt(3.0) * eps * eps);  // 153.96
    double peak = 0.0;
    for (int i = 1; i <= 2000; ++i) {
        System p = twoEqual(0.001 * static_cast<double>(i));
        p.softening = eps;
        accelerations(p, a);
        peak = std::fmax(peak, a[0].x);
    }
    CHECK(peak <= bound * (1.0 + 1e-9));
    CHECK(peak >= bound * 0.99);  // the bound is reached, not just kept

    // Bodies on the same spot: the direction is undefined, the force is zero, never NaN.
    System same = twoEqual(0.0);
    accelerations(same, a);
    CHECK(std::isfinite(a[0].x) && a[0].x == 0.0 && a[0].y == 0.0);
    same.softening = 0.1;
    accelerations(same, a);
    CHECK(a[0].x == 0.0 && a[1].x == 0.0);
}

// ---- single steps ----

void eulerStep() {
    // Both at rest, distance 1, a = +-1, dt = 0.1. Explicit Euler moves with the OLD velocity (0): positions stay.
    System s = twoEqual(1.0);
    step(s, Method::Euler, 0.1);
    CHECK(near(s.bodies[0].pos.x, -0.5) && near(s.bodies[1].pos.x, 0.5));
    CHECK(near(s.bodies[0].vel.x, 0.1) && near(s.bodies[1].vel.x, -0.1));

    // With a sideways velocity (0, 1) the position uses it: y = 0.1. The pull is still +x: vx = 0.1.
    System t = twoEqual(1.0);
    t.bodies[0].vel = {0.0, 1.0};
    step(t, Method::Euler, 0.1);
    CHECK(near(t.bodies[0].pos.y, 0.1));
    CHECK(near(t.bodies[0].vel.x, 0.1));
}

void leapfrogStep() {
    // Kick-drift-kick, dt = 0.1, start at rest, distance 1:
    //   half kick  v = 0 + 1 * 0.05 = 0.05
    //   drift      x0 = -0.5 + 0.05 * 0.1 = -0.495, x1 = 0.495, distance 0.99
    //   half kick  a = 1 / 0.99^2 = 1.0203040, v = 0.05 + 1.0203040 * 0.05 = 0.1010152
    System s = twoEqual(1.0);
    step(s, Method::Leapfrog, 0.1);
    CHECK(near(s.bodies[0].pos.x, -0.495, 1e-12));
    CHECK(near(s.bodies[0].vel.x, 0.1010152, 1e-6));
    CHECK(near(s.bodies[1].vel.x, -0.1010152, 1e-6));
}

void freeFlight() {
    // A lone body feels nothing: every method moves it on a straight line, (1,2) + (3,-1) * 0.5 = (2.5, 1.5).
    for (Method m : {Method::Euler, Method::Leapfrog, Method::Rk4}) {
        System s;
        s.add(Body{{1.0, 2.0}, {3.0, -1.0}, 5.0});
        step(s, m, 0.5);
        CHECK(near(s.bodies[0].pos.x, 2.5) && near(s.bodies[0].pos.y, 1.5));
        CHECK(near(s.bodies[0].vel.x, 3.0) && near(s.bodies[0].vel.y, -1.0));
    }
}

// ---- conserved quantities ----

void energyAndMomentum() {
    // Mass 2 at speed 3: K = 0.5 * 2 * 9 = 9.
    System k;
    k.add(Body{{0.0, 0.0}, {3.0, 0.0}, 2.0});
    CHECK(near(kineticEnergy(k), 9.0));
    CHECK(near(potentialEnergy(k), 0.0));  // a single body has no pairs

    // Burrau's problem, masses 3, 4, 5 on a 3-4-5 triangle, at rest: the pair distances are 5, 4 and 3, so
    // U = -(3*4/5 + 3*5/4 + 4*5/3) = -(2.4 + 3.75 + 6.6667) = -769/60 = -12.81667.
    const System p = makeScene(Scene::Pythagoras);
    CHECK(p.count == 3);
    CHECK(near(kineticEnergy(p), 0.0));
    CHECK(near(totalEnergy(p), -769.0 / 60.0, 1e-9));
    // Center of mass: x = (3*1 + 4*(-2) + 5*1)/12 = 0, y = (3*3 + 4*(-1) + 5*(-1))/12 = 0.
    const Vec2 c = centerOfMass(p);
    CHECK(near(c.x, 0.0) && near(c.y, 0.0));

    // Momentum is conserved by every method, even Euler: the pair forces are equal and opposite. It starts nonzero.
    for (Method m : {Method::Euler, Method::Leapfrog, Method::Rk4}) {
        for (double eps : {0.0, 0.1}) {
            System s;
            s.softening = eps;
            s.add(Body{{0.0, 0.0}, {0.3, -0.1}, 1.0});
            s.add(Body{{1.0, 0.2}, {0.0, 0.4}, 2.5});
            s.add(Body{{-0.8, 1.1}, {-0.2, 0.0}, 0.7});
            s.add(Body{{0.4, -1.3}, {0.1, 0.1}, 1.8});
            s.add(Body{{2.0, 2.0}, {-0.1, -0.2}, 0.4});
            const Vec2 p0 = momentum(s);
            for (int i = 0; i < 1000; ++i) step(s, m, 0.002);
            const Vec2 p1 = momentum(s);
            CHECK(near(p1.x, p0.x, 1e-9) && near(p1.y, p0.y, 1e-9));
            CHECK(allFinite(s));
        }
    }
}

// Two equal masses on a circle: separation d = 1, omega^2 = G (m1 + m2) / d^3 = 2, each body moves at
// v = omega d / 2 = sqrt(2) / 2 = 0.7071 around the middle.
System circularPair() {
    System s;
    const double v = std::sqrt(2.0) / 2.0;
    s.add(Body{{-0.5, 0.0}, {0.0, -v}, 1.0});
    s.add(Body{{0.5, 0.0}, {0.0, v}, 1.0});
    return s;
}

void integratorsOnACircularOrbit() {
    const double period = 2.0 * 3.14159265358979323846 / std::sqrt(2.0);  // 4.44288
    const double dt = 0.01;
    const int steps = static_cast<int>(std::lround(period / dt));  // 444

    // E0 = K + U = 2 * 0.5 * 0.5 - 1 = -0.5.
    CHECK(near(totalEnergy(circularPair()), -0.5, 1e-12));

    // Explicit Euler adds about 0.5 * m * a^2 * dt^2 = 5e-5 of kinetic energy per body and step (the kick is
    // perpendicular to the velocity on a circle), so after one period the energy has risen by at least
    // 444 * 2 * 5e-5 = 0.044, which is 8.9 % of |E0|. 5 % is the checked bound.
    System e = circularPair();
    const double e0 = totalEnergy(e);
    for (int i = 0; i < steps; ++i) step(e, Method::Euler, dt);
    CHECK(relativeDrift(totalEnergy(e), e0) > 0.05);

    // Leapfrog error is O(dt^2): (omega dt)^2 / 12 is about 2e-5 here, bounded and oscillating. Bound 1e-3.
    System l = circularPair();
    double worst = 0.0;
    for (int i = 0; i < steps; ++i) {
        step(l, Method::Leapfrog, dt);
        worst = std::fmax(worst, std::fabs(relativeDrift(totalEnergy(l), e0)));
    }
    CHECK(worst < 1e-3);
    // And it is back where it started after one period: the phase error is small, the orbit did not grow.
    CHECK(near(l.bodies[0].pos.x, -0.5, 5e-3) && near(l.bodies[0].pos.y, 0.0, 5e-3));

    // RK4 error is O(dt^4) per orbit: far below leapfrog on a single period. Bound 1e-5.
    System r = circularPair();
    for (int i = 0; i < steps; ++i) step(r, Method::Rk4, dt);
    CHECK(std::fabs(relativeDrift(totalEnergy(r), e0)) < 1e-5);
}

void softenedCoreCrossing() {
    // Two unit masses at rest, distance 1, eps = 0.2. They fall through each other's softened core.
    // E0 = -1 / sqrt(1 + 0.04) = -0.98058. At d = 0 the potential is -1 / 0.2 = -5, so K = 5 - 0.98058 = 4.01942
    // and each body (K = 2 * 0.5 * v^2) moves at v = sqrt(4.01942) = 2.00485.
    System s = twoEqual(1.0);
    s.softening = 0.2;
    const double e0 = totalEnergy(s);
    CHECK(near(e0, -0.9805806757, 1e-9));
    double maxSpeed = 0.0;
    double minDist = 10.0;
    double worst = 0.0;
    for (int i = 0; i < 3000; ++i) {
        step(s, Method::Leapfrog, 0.002);
        maxSpeed = std::fmax(maxSpeed, length(s.bodies[0].vel));
        minDist = std::fmin(minDist, minDistance(s));
        worst = std::fmax(worst, std::fabs(relativeDrift(totalEnergy(s), e0)));
    }
    CHECK(maxSpeed > 1.995 && maxSpeed < 2.0055);
    CHECK(minDist < 0.01);   // really went through the middle
    // And the books balance with the softened potential. Leapfrog error scales with (v dt / eps)^2 = (2 * 0.002 / 0.2)^2
    // = 4e-4 here; the bound leaves a factor of 5.
    CHECK(worst < 2e-3);

    // The same start without softening: the pull at the last sample before the crossing is huge, and the bodies
    // are flung out far faster than the energy budget allows (the exact answer stays below 2.0 at distance 0.2).
    System n = twoEqual(1.0);
    double naiveMax = 0.0;
    for (int i = 0; i < 3000; ++i) {
        step(n, Method::Leapfrog, 0.002);
        naiveMax = std::fmax(naiveMax, length(n.bodies[0].vel));
    }
    CHECK(naiveMax > 4.0);

    // Measuring with the wrong potential is a bug of its own: the softened run, judged with eps = 0, looks like
    // it drifts although the integrator is fine.
    System m = twoEqual(1.0);
    m.softening = 0.2;
    for (int i = 0; i < 300; ++i) step(m, Method::Leapfrog, 0.002);
    System wrong = m;
    wrong.softening = 0.0;
    CHECK(std::fabs(totalEnergy(wrong) - totalEnergy(m)) > 0.05);
}

// ---- scenes ----

void figureEight() {
    // Chenciner and Montgomery (2000): x1 = -x2 = (0.97000436, -0.24308753), x3 = 0, v3 = (-0.93240737, -0.86473146),
    // v1 = v2 = -v3 / 2, unit masses, G = 1, period 6.32591398.
    const System f = makeScene(Scene::Figure8);
    CHECK(f.count == 3);
    CHECK(near(f.bodies[0].pos.x, 0.97000436, 1e-12) && near(f.bodies[0].pos.y, -0.24308753, 1e-12));
    CHECK(near(f.bodies[1].pos.x, -0.97000436, 1e-12));
    CHECK(near(f.bodies[2].vel.x, -0.93240737, 1e-12));
    CHECK(near(f.bodies[0].vel.x, 0.466203685, 1e-12) && near(f.bodies[1].vel.y, 0.43236573, 1e-12));
    // Distances: r13 = r23 = |x1| = 0.99999..., r12 = 2: U = -(1 + 1 + 0.5) = -2.5. K = 0.5 * (v3^2 + 2 * v3^2 / 4)
    // = 0.75 * v3^2 = 0.75 * 1.61701 = 1.21276, so E = -1.2872.
    CHECK(near(totalEnergy(f), -1.2872, 1e-3));
    CHECK(near(momentum(f).x, 0.0, 1e-12) && near(momentum(f).y, 0.0, 1e-12));
    CHECK(near(centerOfMass(f).x, 0.0, 1e-12));
    CHECK(info(Scene::Figure8).softening > 0.0);

    // One period of leapfrog with 2000 steps: all three bodies come back to their start (tolerance 2e-3,
    // two orders above the expected O(dt^2) error and the 8 digit rounding of the initial values).
    const double period = 6.32591398;
    System l = f;
    for (int i = 0; i < 2000; ++i) step(l, Method::Leapfrog, period / 2000.0);
    for (std::size_t i = 0; i < 3; ++i) {
        CHECK(near(l.bodies[i].pos.x, f.bodies[i].pos.x, 2e-3) && near(l.bodies[i].pos.y, f.bodies[i].pos.y, 2e-3));
    }

    // Explicit Euler with the same step does not close the loop: the energy has grown by more than 1 %.
    System e = f;
    for (int i = 0; i < 2000; ++i) step(e, Method::Euler, period / 2000.0);
    CHECK(relativeDrift(totalEnergy(e), totalEnergy(f)) > 0.01);
}

void planetsAndCluster() {
    // Planets: the innermost orbits the sun at r = 0.35 with the circular speed sqrt(G M / r) = sqrt(1 / 0.35) = 1.690309.
    // The frame is the one with zero total momentum, so only the velocity RELATIVE to the sun is exact.
    const System p = makeScene(Scene::Planets);
    CHECK(p.count == 5);
    CHECK(near(p.bodies[0].mass, 1.0));
    const Vec2 rel = p.bodies[1].vel - p.bodies[0].vel;
    const Vec2 off = p.bodies[1].pos - p.bodies[0].pos;
    CHECK(near(length(off), 0.35, 1e-12));
    CHECK(near(length(rel), 1.6903085094570331, 1e-9));
    CHECK(near(dot(rel, off), 0.0, 1e-12));  // tangential
    CHECK(near(momentum(p).x, 0.0, 1e-12) && near(momentum(p).y, 0.0, 1e-12));
    CHECK(near(centerOfMass(p).x, 0.0, 1e-12) && near(centerOfMass(p).y, 0.0, 1e-12));

    // Cluster: same seed, same cluster. Centered and virial ratio K/|U| as requested.
    const System a = makeScene(Scene::Cluster);
    const System b = makeScene(Scene::Cluster);
    CHECK(a.count == 14);
    CHECK(a.bodies[5].pos.x == b.bodies[5].pos.x && a.bodies[5].vel.y == b.bodies[5].vel.y);
    CHECK(near(momentum(a).x, 0.0, 1e-12) && near(momentum(a).y, 0.0, 1e-12));
    CHECK(near(centerOfMass(a).x, 0.0, 1e-12) && near(centerOfMass(a).y, 0.0, 1e-12));
    CHECK(near(kineticEnergy(a) / std::fabs(potentialEnergy(a)), 0.2, 1e-9));
    for (std::size_t i = 0; i < a.count; ++i) CHECK(length(a.bodies[i].pos) < 1.5);

    // Every scene is a finite, bounded, positive-mass system.
    for (int i = 0; i < kSceneCount; ++i) {
        const System s = makeScene(static_cast<Scene>(i));
        CHECK(allFinite(s) && s.count >= 3 && s.count + 4 <= kMaxBodies);
        for (std::size_t k = 0; k < s.count; ++k) CHECK(s.bodies[k].mass > 0.0);
        CHECK(info(static_cast<Scene>(i)).pxPerUnit > 0.0 && info(static_cast<Scene>(i)).timeScale > 0.0);
        CHECK(info(static_cast<Scene>(i)).defaultStep >= 0 && info(static_cast<Scene>(i)).defaultStep < static_cast<int>(kStepSizes.size()));
    }
}

void virialScaling() {
    // Two unit masses at distance 1: U = -1. Both move at 0.5 (opposite): K = 2 * 0.5 * 0.25 = 0.25, ratio 0.25.
    System s = twoEqual(1.0);
    s.bodies[0].vel = {0.0, 0.5};
    s.bodies[1].vel = {0.0, -0.5};
    scaleToVirialRatio(s, 0.5);  // K must double: speeds grow by sqrt 2
    CHECK(near(kineticEnergy(s), 0.5));
    CHECK(near(s.bodies[0].vel.y, 0.5 * std::sqrt(2.0)));

    // Nothing moves: there is nothing to scale, no NaN from 0/0. A single body has no potential: also untouched.
    System rest = twoEqual(1.0);
    scaleToVirialRatio(rest, 0.5);
    CHECK(allFinite(rest) && near(kineticEnergy(rest), 0.0));
    System lone;
    lone.add(Body{{0.0, 0.0}, {1.0, 0.0}, 1.0});
    scaleToVirialRatio(lone, 0.5);
    CHECK(near(lone.bodies[0].vel.x, 1.0));
}

// ---- helpers ----

void systemHelpers() {
    // Closest pair of (0,0), (3,4), (10,10) is the first two: 5.
    System s;
    s.add(Body{{0.0, 0.0}, {0.0, 0.0}, 1.0});
    s.add(Body{{3.0, 4.0}, {0.0, 0.0}, 1.0});
    s.add(Body{{10.0, 10.0}, {0.0, 0.0}, 1.0});
    CHECK(near(minDistance(s), 5.0));
    System one;
    one.add(Body{{0.0, 0.0}, {0.0, 0.0}, 1.0});
    CHECK(std::isinf(minDistance(one)));
    CHECK(std::isinf(minDistance(System{})));

    // Capacity and bad input.
    System full;
    for (std::size_t i = 0; i < kMaxBodies; ++i) CHECK(full.add(Body{{static_cast<double>(i), 0.0}, {0.0, 0.0}, 1.0}));
    CHECK(!full.add(Body{{99.0, 0.0}, {0.0, 0.0}, 1.0}));
    CHECK(full.count == kMaxBodies);
    System bad;
    CHECK(!bad.add(Body{{0.0, 0.0}, {0.0, 0.0}, 0.0}));
    CHECK(!bad.add(Body{{0.0, 0.0}, {0.0, 0.0}, -1.0}));
    CHECK(!bad.add(Body{{std::nan(""), 0.0}, {0.0, 0.0}, 1.0}));
    CHECK(!bad.add(Body{{0.0, 0.0}, {std::numeric_limits<double>::infinity(), 0.0}, 1.0}));
    CHECK(bad.count == 0);

    // allFinite sees NaN and infinity in any field.
    System f = twoEqual(1.0);
    CHECK(allFinite(f));
    f.bodies[1].vel.y = std::numeric_limits<double>::infinity();
    CHECK(!allFinite(f));
    f = twoEqual(1.0);
    f.bodies[0].pos.x = std::nan("");
    CHECK(!allFinite(f));
}

void driftAndScale() {
    // (e - e0) / |e0|: -0.9 against -1 is +10 %, -3 against -2 is -50 %. e0 = 0 must not divide: plain difference.
    CHECK(near(relativeDrift(-0.9, -1.0), 0.1));
    CHECK(near(relativeDrift(-3.0, -2.0), -0.5));
    CHECK(near(relativeDrift(0.5, 0.0), 0.5));
    CHECK(near(relativeDrift(-1.0, -1.0), 0.0));

    // symlog(x, c) = sign(x) * log10(1 + |x| / c): 0 -> 0, 1e-3 with c = 1e-6 -> log10(1001), odd, monotonic.
    CHECK(symlog(0.0, 1e-6) == 0.0);
    CHECK(near(symlog(1e-3, 1e-6), std::log10(1001.0), 1e-12));
    CHECK(near(symlog(-1.0, 1e-6), -std::log10(1000001.0), 1e-12));
    CHECK(symlog(2.0, 1e-6) > symlog(1.0, 1e-6));
    CHECK(near(symlog(-0.37, 1e-6), -symlog(0.37, 1e-6), 1e-12));
    CHECK(symlog(std::nan(""), 1e-6) == 0.0);
    CHECK(std::isfinite(symlog(std::numeric_limits<double>::infinity(), 1e-6)));
    CHECK(symlog(std::numeric_limits<double>::infinity(), 1e-6) > 100.0);
}

void edgePoints() {
    // Half sizes 100 x 50 around the center. The ray leaves the rectangle where it first reaches an edge.
    auto p = edgePoint(100.0, 50.0, 1.0, 0.0);
    CHECK(near(p.x, 100.0) && near(p.y, 0.0));
    p = edgePoint(100.0, 50.0, 0.0, -1.0);
    CHECK(near(p.x, 0.0) && near(p.y, -50.0));
    p = edgePoint(100.0, 50.0, 1.0, 2.0);  // steeper than the diagonal: top edge first, t = 25 -> (25, 50)
    CHECK(near(p.x, 25.0) && near(p.y, 50.0));
    p = edgePoint(100.0, 50.0, 4.0, 1.0);  // shallow: right edge first, t = 25 -> (100, 25)
    CHECK(near(p.x, 100.0) && near(p.y, 25.0));
    p = edgePoint(100.0, 50.0, -3.0, 3.0);  // 45 degrees: top edge at x = -50
    CHECK(near(p.x, -50.0) && near(p.y, 50.0));
    p = edgePoint(100.0, 50.0, 0.0, 0.0);  // no direction: the center, no division by zero
    CHECK(near(p.x, 0.0) && near(p.y, 0.0));
}

void randomNumbers() {
    Rng a{12345};
    Rng b{12345};
    bool same = true;
    bool inRange = true;
    double sum = 0.0;
    for (int i = 0; i < 10000; ++i) {
        const double x = a.uniform();
        same = same && x == b.uniform();
        inRange = inRange && x >= 0.0 && x < 1.0;
        sum += x;
    }
    CHECK(same);
    CHECK(inRange);
    CHECK(near(sum / 10000.0, 0.5, 0.02));  // a uniform mean, standard error 0.003
    Rng zero{0};                            // seed 0 would stick xorshift at 0: it must still produce numbers
    CHECK(zero.uniform() != zero.uniform());
}

}  // namespace

int main() {
    accelerationsTwoBodies();
    softening();
    eulerStep();
    leapfrogStep();
    freeFlight();
    energyAndMomentum();
    integratorsOnACircularOrbit();
    softenedCoreCrossing();
    figureEight();
    planetsAndCluster();
    virialScaling();
    systemHelpers();
    driftAndScale();
    edgePoints();
    randomNumbers();
    if (failures == 0) std::printf("all n-body checks passed\n");
    return failures == 0 ? 0 : 1;
}
