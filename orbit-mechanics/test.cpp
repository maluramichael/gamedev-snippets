// Tests for the orbit logic. Plain asserts on hand-computed cases, no framework, no SDL.
// Units are free: most cases use mu = 1 and radius 1, so a circular orbit has period 2 pi and speed 1.
#undef NDEBUG
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>

#include "logic.hpp"

using namespace orb;

namespace {

constexpr double kPi = 3.14159265358979323846;

bool near(double a, double b, double tol = 1e-9) { return std::fabs(a - b) <= tol; }
bool nearState(const State& a, const State& b, double tol) {
    return near(a.pos.x, b.pos.x, tol) && near(a.pos.y, b.pos.y, tol) && near(a.vel.x, b.vel.x, tol) && near(a.vel.y, b.vel.y, tol);
}

/// An independent reference: classic RK4 with a tiny step. The Kepler propagator must agree with it.
State rk4(State s, double mu, double h, long steps) {
    for (long i = 0; i < steps; ++i) {
        const Vec2 k1v = acceleration(s.pos, mu);
        const Vec2 k1x = s.vel;
        const Vec2 k2v = acceleration(s.pos + k1x * (h / 2), mu);
        const Vec2 k2x = s.vel + k1v * (h / 2);
        const Vec2 k3v = acceleration(s.pos + k2x * (h / 2), mu);
        const Vec2 k3x = s.vel + k2v * (h / 2);
        const Vec2 k4v = acceleration(s.pos + k3x * h, mu);
        const Vec2 k4x = s.vel + k3v * h;
        s.pos = s.pos + (k1x + k2x * 2 + k3x * 2 + k4x) * (h / 6);
        s.vel = s.vel + (k1v + k2v * 2 + k3v * 2 + k4v) * (h / 6);
    }
    return s;
}

void gravity() {
    const Vec2 a1 = acceleration({2, 0}, 4.0);  // -mu * r / |r|^3 = -4 * 2 / 8
    assert(near(a1.x, -1.0) && near(a1.y, 0.0));
    const Vec2 a2 = acceleration({0, -1}, 1.0);  // pulls back up
    assert(near(a2.x, 0.0) && near(a2.y, 1.0));
    const Vec2 a3 = acceleration({3, 4}, 125.0);  // |r| = 5, |r|^3 = 125
    assert(near(a3.x, -3.0) && near(a3.y, -4.0));
    // Sitting on the sun must not produce NaN or infinity.
    const Vec2 a0 = acceleration({0, 0}, 1.0);
    assert(std::isfinite(a0.x) && std::isfinite(a0.y));
    // Twice as far, a quarter of the pull (inverse square).
    assert(near(length(acceleration({1, 0}, 1.0)), 4.0 * length(acceleration({2, 0}, 1.0))));
}

void energyAndMomentum() {
    const State circle{{1, 0}, {0, 1}};
    assert(near(energy(circle, 1.0), -0.5));  // 1/2 - 1
    assert(near(angularMomentum(circle), 1.0));
    const State apo{{1, 0}, {0, 0.5}};
    assert(near(energy(apo, 1.0), -0.875));  // 0.125 - 1
    assert(near(angularMomentum(apo), 0.5));
    const State back{{1, 0}, {0, -0.5}};
    assert(near(angularMomentum(back), -0.5));  // the other way round
    assert(near(energy({{2, 0}, {0, 0}}, 1.0), -0.5));  // at rest: only the potential
}

void explicitEulerStep() {
    // r = (1,0), v = (0,1), mu = 1, h = 0.1. Both updates use the OLD values:
    // pos = (1, 0.1), vel = (0,1) + 0.1 * (-1, 0) = (-0.1, 1).
    const State s = stepExplicitEuler({{1, 0}, {0, 1}}, 1.0, 0.1);
    assert(near(s.pos.x, 1.0) && near(s.pos.y, 0.1));
    assert(near(s.vel.x, -0.1) && near(s.vel.y, 1.0));
    // The energy goes UP: 1/2 * 1.01 - 1 / sqrt(1.01) = -0.49003719...
    assert(near(energy(s, 1.0), -0.49003719, 1e-7));
    assert(energy(s, 1.0) > -0.5);
    // Angular momentum grows by exactly 1 + h^2 mu / r^3 per step: 1.01 here.
    assert(near(angularMomentum(s), 1.01));
    // Second case: r = (2,0), v = (0,0.5), h = 0.2: factor 1 + 0.04 / 8 = 1.005 on L = 1.
    const State t = stepExplicitEuler({{2, 0}, {0, 0.5}}, 1.0, 0.2);
    assert(near(angularMomentum(t), 1.005));
}

void semiImplicitStep() {
    // Same start, but the velocity is updated first and the NEW velocity moves the body:
    // vel = (-0.1, 1), pos = (1,0) + 0.1 * (-0.1, 1) = (0.99, 0.1).
    const State s = stepSemiImplicit({{1, 0}, {0, 1}}, 1.0, 0.1);
    assert(near(s.vel.x, -0.1) && near(s.vel.y, 1.0));
    assert(near(s.pos.x, 0.99) && near(s.pos.y, 0.1));
    // 0.99 * 1 - 0.1 * (-0.1) = 1: the angular momentum is kept exactly, unlike the explicit step above.
    assert(near(angularMomentum(s), 1.0, 1e-15));
    const State t = stepSemiImplicit({{2, 0}, {0, 0.5}}, 1.0, 0.2);
    assert(near(angularMomentum(t), 1.0, 1e-15));
}

void verletStep() {
    // x' = x + v h + a h^2 / 2 = (0.995, 0.1); a' = -x' / |x'|^3 = (-0.994963, -0.0999963);
    // v' = v + (a + a') h / 2 = (-0.0997481, 0.9950002).
    const State s = stepVelocityVerlet({{1, 0}, {0, 1}}, 1.0, 0.1);
    assert(near(s.pos.x, 0.995) && near(s.pos.y, 0.1));
    assert(near(s.vel.x, -0.0997481, 1e-6) && near(s.vel.y, 0.9950002, 1e-6));
}

void integratorsOverManyOrbits() {
    // 100 circular orbits with 200 steps each. The explicit step pumps energy in, the other two do not.
    const double h = 2.0 * kPi / 200.0;
    State e{{1, 0}, {0, 1}};
    State si = e;
    State v = e;
    double maxE = 0.0;
    double maxSi = 0.0;
    double maxV = 0.0;
    double maxRadiusE = 0.0;
    double minRadiusSi = 9.0;
    double maxRadiusSi = 0.0;
    for (int i = 0; i < 20000; ++i) {
        e = stepExplicitEuler(e, 1.0, h);
        si = stepSemiImplicit(si, 1.0, h);
        v = stepVelocityVerlet(v, 1.0, h);
        maxE = std::max(maxE, std::fabs(relativeDrift(energy(e, 1.0), -0.5)));
        maxSi = std::max(maxSi, std::fabs(relativeDrift(energy(si, 1.0), -0.5)));
        maxV = std::max(maxV, std::fabs(relativeDrift(energy(v, 1.0), -0.5)));
        maxRadiusE = std::max(maxRadiusE, length(e.pos));
        minRadiusSi = std::min(minRadiusSi, length(si.pos));
        maxRadiusSi = std::max(maxRadiusSi, length(si.pos));
    }
    assert(maxE > 0.3);  // explicit Euler: the orbit has blown up (measured 0.80)
    assert(maxRadiusE > 3.0);
    assert(maxSi < 0.01);  // semi-implicit: bounded wobble (measured 0.001)
    assert(minRadiusSi > 0.97 && maxRadiusSi < 1.03);
    assert(maxV < 1e-4);  // Verlet: (measured 2e-7)
    // The semi-implicit step keeps L to rounding error, however eccentric the orbit.
    const State apo = apsisState(1.0, 0.75, 0.0, 1.0);
    State s = apo;
    for (int i = 0; i < 20000; ++i) s = stepSemiImplicit(s, 1.0, 0.01);
    assert(near(angularMomentum(s), angularMomentum(apo), 1e-9));
}

void drift() {
    assert(near(relativeDrift(1.1, 1.0), 0.1));
    assert(near(relativeDrift(-0.45, -0.5), 0.1));  // (-0.45 + 0.5) / 0.5: relative to the size of the reference
    assert(near(relativeDrift(0.25, 0.0), 0.25));   // no reference to divide by: absolute
}

void percentText() {
    assert(formatPercent(0.1234) == "+12.3%");
    assert(formatPercent(-0.5) == "-50.0%");
    assert(formatPercent(0.0003) == "0.0%");  // below 0.05 %: no sign, no "-0.0%"
    assert(formatPercent(-0.0003) == "0.0%");
    assert(formatPercent(12.0) == ">999%");
    assert(formatPercent(-12.0) == "<-999%");
    assert(formatPercent(std::numeric_limits<double>::quiet_NaN()) == "n/a");
}

void stumpff() {
    assert(near(stumpffC(0.0), 0.5) && near(stumpffS(0.0), 1.0 / 6.0));
    // z = pi^2: C = (1 - cos pi) / pi^2 = 2 / pi^2, S = (pi - sin pi) / pi^3 = 1 / pi^2.
    assert(near(stumpffC(kPi * kPi), 2.0 / (kPi * kPi)));
    assert(near(stumpffS(kPi * kPi), 1.0 / (kPi * kPi)));
    // z = -1: C = cosh 1 - 1, S = sinh 1 - 1.
    assert(near(stumpffC(-1.0), std::cosh(1.0) - 1.0));
    assert(near(stumpffS(-1.0), std::sinh(1.0) - 1.0));
    // Continuous across the series switch at tiny z.
    assert(near(stumpffC(1e-9), 0.5, 1e-8) && near(stumpffS(-1e-9), 1.0 / 6.0, 1e-8));
}

void circularKepler() {
    const State start{{1, 0}, {0, 1}};
    const double T = 2.0 * kPi;
    assert(nearState(propagate(start, 1.0, 0.0), start, 1e-12));
    assert(nearState(propagate(start, 1.0, T / 4), {{0, 1}, {-1, 0}}, 1e-9));
    assert(nearState(propagate(start, 1.0, T / 2), {{-1, 0}, {0, -1}}, 1e-9));
    assert(nearState(propagate(start, 1.0, T), start, 1e-9));
    // Many orbits later, still the same point (time is reduced modulo the period).
    assert(nearState(propagate(start, 1.0, 1000.0 * T + T / 4), {{0, 1}, {-1, 0}}, 1e-6));
    // A negative time is not supported and leaves the state alone.
    assert(nearState(propagate(start, 1.0, -1.0), start, 0.0));
}

void eccentricKepler() {
    // Start at the far point: r = 1, v = 0.5 -> e = 0.75, a = 4/7, periapsis 1/7, T = 2 pi a^1.5.
    const State start{{1, 0}, {0, 0.5}};
    const double a = 4.0 / 7.0;
    const double T = 2.0 * kPi * std::pow(a, 1.5);
    const auto c = conicFromState(start, 1.0);
    assert(c && c->bound);
    assert(near(c->e, 0.75) && near(c->p, 0.25) && near(c->rPeri, 1.0 / 7.0) && near(c->rApo, 1.0) && near(c->period, T));
    assert(near(c->energy, -0.875) && near(c->L, 0.5));
    // Half a period later it is at the near point, moving fast: speed L / r = 0.5 / (1/7) = 3.5.
    assert(nearState(propagate(start, 1.0, T / 2), {{-1.0 / 7.0, 0}, {0, -3.5}}, 1e-9));
    assert(nearState(propagate(start, 1.0, T), start, 1e-9));
    // The other way round: mirrored in y.
    assert(nearState(propagate({{1, 0}, {0, -0.5}}, 1.0, T / 2), {{-1.0 / 7.0, 0}, {0, 3.5}}, 1e-9));
    // Kepler's second law: near the far point the body is slow. After a quarter of the period it has only moved about
    // 22 degrees away from the start. Independent check through Kepler's equation E - e sin E = M with M = 270 degrees:
    // E = 4.0990786, true anomaly 3.5290109, angle from the start 0.3874183 rad.
    const State q = propagate(start, 1.0, T / 4);
    assert(q.pos.y > 0.0 && q.pos.x > 0.0 && std::atan2(q.pos.y, q.pos.x) < kPi / 4);
    assert(near(std::atan2(q.pos.y, q.pos.x), 0.3874183, 1e-6));
    // The way back is the mirror image: three quarters of the period is the same point below the axis.
    const State q3 = propagate(start, 1.0, 3.0 * T / 4);
    assert(near(q3.pos.x, q.pos.x, 1e-9) && near(q3.pos.y, -q.pos.y, 1e-9));
}

void againstRk4() {
    // Bound orbit that starts somewhere in the middle, moving inward.
    const State b{{1.5, 0}, {-0.3, 0.5}};
    for (const double t : {2.0, 5.0, 12.0}) {
        assert(nearState(propagate(b, 1.0, t), rk4(b, 1.0, 1e-3, static_cast<long>(t / 1e-3)), 2e-6));
    }
    // Retrograde bound orbit.
    const State r{{0.0, -1.2}, {-0.4, 0.3}};  // angular momentum -0.48
    assert(nearState(propagate(r, 1.0, 9.0), rk4(r, 1.0, 1e-3, 9000), 2e-6));
    // Hyperbola that passes its periapsis (about 0.28 from the sun): incoming, energy 0.41 - 1/3.04 = 0.08 > 0.
    const State h{{3, 0.5}, {-0.9, 0.1}};
    assert(energy(h, 1.0) > 0.0);
    assert(nearState(propagate(h, 1.0, 6.0), rk4(h, 1.0, 5e-4, 12000), 2e-6));
    // Hyperbola that is already leaving.
    const State out{{1, 0}, {0, 1.5}};
    assert(energy(out, 1.0) > 0.0);
    assert(nearState(propagate(out, 1.0, 3.0), rk4(out, 1.0, 1e-3, 3000), 2e-6));
    // Another mass: the units of mu and distance are free.
    const State big{{130, 0}, {0, 40}};
    assert(nearState(propagate(big, 602300.0, 7.0), rk4(big, 602300.0, 1e-3, 7000), 1e-5));
}

void parabola() {
    // Exactly the escape speed sqrt(2 mu / r): e = 1, the case where a and the period do not exist.
    const State start{{1, 0}, {0, std::sqrt(2.0)}};
    const auto c = conicFromState(start, 1.0);
    assert(c && !c->bound);
    assert(near(c->e, 1.0, 1e-9) && near(c->rPeri, 1.0));  // p = L^2 / mu = 2, so 2 / (1 + 1)
    assert(std::isinf(c->period) && std::isinf(c->rApo));
    assert(nearState(propagate(start, 1.0, 2.0), rk4(start, 1.0, 1e-3, 2000), 1e-6));
}

void invariantsOfTheExactSolution() {
    const State starts[] = {{{1, 0}, {0, 0.5}}, {{1.5, 0.2}, {-0.3, 0.5}}, {{3, 0.5}, {-0.9, 0.1}}, {{1, 0}, {0, 1}}};
    for (const State& s : starts) {
        const double e0 = energy(s, 1.0);
        const double l0 = angularMomentum(s);
        for (const double t : {0.3, 1.0, 4.0, 17.5}) {
            const State p = propagate(s, 1.0, t);
            assert(near(energy(p, 1.0), e0, 1e-9));
            assert(near(angularMomentum(p), l0, 1e-9));
        }
    }
}

void degenerate() {
    // Falling straight at the sun: no ellipse to speak of (L = 0).
    assert(!conicFromState({{1, 0}, {-1, 0}}, 1.0));
    assert(!conicFromState({{1, 0}, {0, 0}}, 1.0));
    assert(!conicFromState({{0, 0}, {0, 1}}, 1.0));
    assert(!conicFromState({{1, 0}, {0, 1}}, 0.0));
    // Propagating from a broken state returns it unchanged instead of NaN.
    const State broken{{0, 0}, {0, 1}};
    assert(nearState(propagate(broken, 1.0, 1.0), broken, 0.0));
}

void orbitShape() {
    // Ellipse: every point has distance sum 2a to the two foci, the second focus sits at -2 a e (cos w, sin w).
    const State start{{1, 0}, {0, 0.5}};
    const auto c = *conicFromState(start, 1.0);
    const double a = 4.0 / 7.0;
    const Vec2 f2{-2.0 * a * c.e * std::cos(c.omega), -2.0 * a * c.e * std::sin(c.omega)};
    const auto pts = conicPoints(c, 120, 100.0);
    assert(pts.size() == 121);
    double minR = 9.0;
    double maxR = 0.0;
    for (const Vec2& p : pts) {
        assert(near(length(p) + length(p - f2), 2.0 * a, 1e-9));
        minR = std::min(minR, length(p));
        maxR = std::max(maxR, length(p));
    }
    assert(near(minR, 1.0 / 7.0, 1e-9) && near(maxR, 1.0, 1e-9));
    assert(near(pts.front().x, pts.back().x, 1e-9) && near(pts.front().y, pts.back().y, 1e-9));  // closed
    // The middle sample has true anomaly 0 = the near point; the next one is already past it, in the direction of
    // motion (the start moves towards +y, so at the near point on the left it moves down).
    assert(near(pts[60].x, -1.0 / 7.0, 1e-9));
    assert(pts[61].y < 0.0);
    const auto back = *conicFromState({{1, 0}, {0, -0.5}}, 1.0);
    assert(conicPoints(back, 120, 100.0)[61].y > 0.0);  // reversed orbit, reversed direction

    // Hyperbola: |P - F2| - |P| = 2 a_h, F2 = 2 a_h e (cos w, sin w), cut off at the requested radius.
    const auto h = *conicFromState({{1, 0}, {0, 1.5}}, 1.0);
    const double ah = h.p / (h.e * h.e - 1.0);
    const Vec2 g2{2.0 * ah * h.e * std::cos(h.omega), 2.0 * ah * h.e * std::sin(h.omega)};
    const auto hp = conicPoints(h, 80, 20.0);
    assert(hp.size() == 81);
    for (const Vec2& p : hp) {
        assert(length(p) <= 20.0 + 1e-9);
        assert(near(length(p - g2) - length(p), 2.0 * ah, 1e-8));
    }
    assert(near(length(hp.front()), 20.0, 1e-6) && near(length(hp.back()), 20.0, 1e-6));  // runs out to the cut-off on both sides
}

void areaLaw() {
    // Equal times, equal areas: eight sectors of T/8 around a strongly eccentric orbit all sweep L/2 * T/8.
    const State start{{1, 0}, {0, 0.5}};
    const double a = 4.0 / 7.0;
    const double T = 2.0 * kPi * std::pow(a, 1.5);
    const double expected = 0.5 * 0.5 * (T / 8.0);
    double total = 0.0;
    for (int k = 0; k < 8; ++k) {
        const double area = sweptArea(start, 1.0, T * k / 8.0, T * (k + 1) / 8.0, 2000);
        assert(near(area, expected, expected * 2e-3));
        total += area;
    }
    // All eight together are the whole ellipse: pi a b with b = a sqrt(1 - e^2).
    assert(near(total, kPi * a * a * std::sqrt(1.0 - 0.75 * 0.75), total * 2e-3));
    // A quarter of a unit circle is pi / 4.
    assert(near(sweptArea({{1, 0}, {0, 1}}, 1.0, 0.0, kPi / 2, 400), kPi / 4, 1e-5));
}

void startStates() {
    // Counterclockwise in the maths sense: at (1,0) the velocity points to +y.
    const State c = circularState({1, 0}, 1.0);
    assert(near(c.vel.x, 0.0) && near(c.vel.y, 1.0));
    const State d = circularState({0, 4}, 4.0);  // speed sqrt(mu / r) = 1, tangent at the top is (-1, 0)
    assert(near(d.vel.x, -1.0) && near(d.vel.y, 0.0));
    assert(near(conicFromState(d, 4.0)->e, 0.0, 1e-9));
    // A circular start is a circle whatever the angle.
    const State e = circularState({3, -4}, 10.0);
    assert(near(conicFromState(e, 10.0)->e, 0.0, 1e-9));
    assert(near(length(e.vel), std::sqrt(10.0 / 5.0)));

    // apsisState: far point at angle 0 with e = 0.75 and mu = 1 has speed 0.5, as in the cases above.
    const State a = apsisState(1.0, 0.75, 0.0, 1.0);
    assert(near(a.pos.x, 1.0) && near(a.pos.y, 0.0) && near(a.vel.x, 0.0) && near(a.vel.y, 0.5));
    const auto ca = *conicFromState(apsisState(170.0, 0.5, 2.0, 602300.0), 602300.0);
    assert(near(ca.e, 0.5, 1e-9) && near(ca.rApo, 170.0, 1e-6) && near(ca.rPeri, 170.0 / 3.0, 1e-6));

    // A drag vector is a velocity: scaled, nothing else.
    const State l = launchState({10, 20}, {30, -40}, 0.5);
    assert(near(l.pos.x, 10.0) && near(l.pos.y, 20.0) && near(l.vel.x, 15.0) && near(l.vel.y, -20.0));
}

void usable() {
    assert(isUsable({{1, 0}, {0, 1}}, 0.5, 10.0));
    assert(!isUsable({{0.1, 0}, {0, 1}}, 0.5, 10.0));  // too close to the sun
    assert(!isUsable({{11, 0}, {0, 1}}, 0.5, 10.0));   // out of range
    assert(!isUsable({{std::numeric_limits<double>::quiet_NaN(), 0}, {0, 1}}, 0.5, 10.0));
    assert(!isUsable({{1, 0}, {std::numeric_limits<double>::infinity(), 1}}, 0.5, 10.0));
    assert(isUsable({{0.5, 0}, {0, 1}}, 0.5, 10.0));  // limits are included
}

}  // namespace

int main() {
    gravity();
    energyAndMomentum();
    explicitEulerStep();
    semiImplicitStep();
    verletStep();
    integratorsOverManyOrbits();
    drift();
    percentText();
    stumpff();
    circularKepler();
    eccentricKepler();
    againstRk4();
    parabola();
    invariantsOfTheExactSolution();
    degenerate();
    orbitShape();
    areaLaw();
    startStates();
    usable();
    std::puts("orbit-mechanics logic: all asserts passed");
    return 0;
}
