// Orbit mechanics: the logic of the demo, no SDL, so test.cpp can check it.
//
// A body moves in the pull of one fixed sun, a = -mu * r / |r|^3. Three numerical steps and one exact answer:
//
//   explicit Euler    pos += vel * h;   vel += acc(old pos) * h        energy and angular momentum grow every step
//   semi-implicit     vel += acc * h;   pos += NEW vel * h             same cost, swapped order, keeps L exactly
//   velocity Verlet   pos += vel*h + acc*h*h/2; vel += (acc + acc') * h/2
//   Kepler            propagate(): universal variables, no step size, no drift
//
// Why explicit Euler spirals outward: it moves along the tangent of the orbit and only then turns the velocity. The
// tangent always lies outside the circle, so every step lands a little too far out. The angular momentum is multiplied
// by exactly 1 + h^2 mu / r^3 per step (test.cpp checks this with hand-computed numbers). Swapping the two lines
// removes that: L' = (r + h v') x v' = r x v' = r x (v + h a) = r x v, because a points along r.
//
// The exact orbit is propagated with the universal variable method (Stumpff functions), which handles ellipses,
// parabolas and hyperbolas with the same code. F(chi) is strictly increasing (dF/dchi is the distance r > 0), so a
// bracketed Newton iteration always finds the single root.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace orb {

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kInf = std::numeric_limits<double>::infinity();

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};

inline Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
inline Vec2 operator*(Vec2 a, double k) { return {a.x * k, a.y * k}; }
inline double dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
inline double cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }
inline double length(Vec2 a) { return std::sqrt(dot(a, a)); }

struct State {
    Vec2 pos;
    Vec2 vel;
};

/// Pull of a sun at the origin. Sitting exactly on it gives zero instead of infinity.
inline Vec2 acceleration(Vec2 pos, double mu) {
    const double r2 = dot(pos, pos);
    if (r2 < 1e-24) return {0.0, 0.0};
    return pos * (-mu / (r2 * std::sqrt(r2)));
}

inline State stepExplicitEuler(State s, double mu, double h) {
    const Vec2 a = acceleration(s.pos, mu);
    return {s.pos + s.vel * h, s.vel + a * h};
}

inline State stepSemiImplicit(State s, double mu, double h) {
    const Vec2 v = s.vel + acceleration(s.pos, mu) * h;
    return {s.pos + v * h, v};
}

inline State stepVelocityVerlet(State s, double mu, double h) {
    const Vec2 a0 = acceleration(s.pos, mu);
    const Vec2 p = s.pos + s.vel * h + a0 * (0.5 * h * h);
    const Vec2 a1 = acceleration(p, mu);
    return {p, s.vel + (a0 + a1) * (0.5 * h)};
}

/// Specific energy (per unit mass): kinetic plus potential. Constant on a real orbit.
inline double energy(const State& s, double mu) { return 0.5 * dot(s.vel, s.vel) - mu / length(s.pos); }

/// Specific angular momentum (z component). Constant on a real orbit; the sign is the direction of travel.
inline double angularMomentum(const State& s) { return cross(s.pos, s.vel); }

/// (value - reference) relative to the size of the reference, absolute when the reference is zero.
inline double relativeDrift(double value, double reference) {
    const double scale = std::fabs(reference);
    return scale > 1e-300 ? (value - reference) / scale : value - reference;
}

/// "+12.3%", "-0.5%", "0.0%" below 0.05 %, ">999%" and "<-999%" beyond, "n/a" for NaN.
inline std::string formatPercent(double drift) {
    if (std::isnan(drift)) return "n/a";
    const double pct = drift * 100.0;
    if (pct > 999.0) return ">999%";
    if (pct < -999.0) return "<-999%";
    if (std::fabs(pct) < 0.05) return "0.0%";
    char buf[24];
    std::snprintf(buf, sizeof buf, "%+.1f%%", pct);
    return buf;
}

/// Stumpff functions C(z) = (1 - cos sqrt z) / z and S(z) = (sqrt z - sin sqrt z) / sqrt z^3, continued to z < 0 with
/// cosh and sinh. Near zero the closed forms cancel, so a series takes over.
inline double stumpffC(double z) {
    if (std::fabs(z) < 1e-2) {
        return 1.0 / 2.0 - z / 24.0 + z * z / 720.0 - z * z * z / 40320.0 + z * z * z * z / 3628800.0;
    }
    if (z > 0.0) return (1.0 - std::cos(std::sqrt(z))) / z;
    return (std::cosh(std::sqrt(-z)) - 1.0) / -z;
}

inline double stumpffS(double z) {
    if (std::fabs(z) < 1e-2) {
        return 1.0 / 6.0 - z / 120.0 + z * z / 5040.0 - z * z * z / 362880.0 + z * z * z * z / 39916800.0;
    }
    if (z > 0.0) {
        const double s = std::sqrt(z);
        return (s - std::sin(s)) / (s * s * s);
    }
    const double s = std::sqrt(-z);
    return (std::sinh(s) - s) / (s * s * s);
}

/// The exact state `dt` after `s0` (dt >= 0; anything else, a broken state or mu <= 0 returns s0 unchanged).
/// For an ellipse the time is reduced modulo the period first, so even hour-long runs stay precise.
inline State propagate(const State& s0, double mu, double dt) {
    const double r0 = length(s0.pos);
    if (!(dt > 0.0) || !(mu > 0.0) || !(r0 > 1e-12)) return s0;
    const double sqmu = std::sqrt(mu);
    const double alpha = 2.0 / r0 - dot(s0.vel, s0.vel) / mu;  // 1 / semi-major axis, negative for hyperbolas
    const double rv = dot(s0.pos, s0.vel) / sqmu;
    double t = dt;
    if (alpha > 1e-12) {
        t = std::fmod(dt, 2.0 * kPi / (sqmu * alpha * std::sqrt(alpha)));
        if (t <= 0.0) return s0;
    }

    // F(chi) = sqrt(mu) * time needed to reach chi, minus the time we want. dF/dchi is the distance r(chi) > 0.
    const auto F = [&](double x) {
        const double z = alpha * x * x;
        return rv * x * x * stumpffC(z) + (1.0 - alpha * r0) * x * x * x * stumpffS(z) + r0 * x - sqmu * t;
    };
    const auto dF = [&](double x) {
        const double z = alpha * x * x;
        return x * x * stumpffC(z) + rv * x * (1.0 - z * stumpffS(z)) + r0 * (1.0 - z * stumpffC(z));
    };
    double lo = 0.0;
    double hi = std::max(1e-9, 2.0 * sqmu * t / r0);
    for (int i = 0; i < 200 && F(hi) < 0.0; ++i) hi *= 2.0;
    double x = std::clamp(sqmu * t / r0, lo, hi);
    for (int i = 0; i < 100; ++i) {
        const double f = F(x);
        if (f < 0.0) {
            lo = x;
        } else {
            hi = x;
        }
        double next = x - f / dF(x);
        if (!(next > lo && next < hi)) next = 0.5 * (lo + hi);
        const bool done = std::fabs(next - x) <= 1e-15 * std::max(1.0, std::fabs(x));
        x = next;
        if (done) break;
    }

    const double z = alpha * x * x;
    const double c = stumpffC(z);
    const double s = stumpffS(z);
    const double f = 1.0 - x * x * c / r0;
    const double g = t - x * x * x * s / sqmu;
    const Vec2 pos = s0.pos * f + s0.vel * g;
    const double r = length(pos);
    const double fdot = sqmu / (r * r0) * x * (z * s - 1.0);
    const double gdot = 1.0 - x * x * c / r;
    return {pos, s0.pos * fdot + s0.vel * gdot};
}

/// The shape of an orbit, for drawing and for the numbers on screen. Positions on it follow
/// r(nu) = p / (1 + e cos nu) at the angle omega + dir * nu from the sun.
struct Conic {
    bool bound = false;     // ellipse (or circle); false for parabola and hyperbola
    double p = 0.0;         // semi-latus rectum, L^2 / mu
    double e = 0.0;         // eccentricity
    double omega = 0.0;     // direction of the near point
    double dir = 1.0;       // +1 counterclockwise in maths orientation, -1 the other way
    double rPeri = 0.0;     // distance at the near point
    double rApo = kInf;     // distance at the far point (infinite when not bound)
    double period = kInf;   // infinite when not bound
    double energy = 0.0;
    double L = 0.0;
};

/// The orbit through a state. Nothing for a fall straight at the sun (L = 0), a sun on top of the body or mu <= 0.
inline std::optional<Conic> conicFromState(const State& s, double mu) {
    const double r = length(s.pos);
    const double speed = length(s.vel);
    if (!(mu > 0.0) || !(r > 1e-12) || !(speed > 0.0)) return std::nullopt;
    const double L = cross(s.pos, s.vel);
    if (std::fabs(L) <= 1e-9 * r * speed) return std::nullopt;
    Conic c;
    c.L = L;
    c.dir = L > 0.0 ? 1.0 : -1.0;
    c.p = L * L / mu;
    const Vec2 ev = (s.pos * (speed * speed - mu / r) - s.vel * dot(s.pos, s.vel)) * (1.0 / mu);
    c.e = length(ev);
    c.omega = c.e > 1e-9 ? std::atan2(ev.y, ev.x) : std::atan2(s.pos.y, s.pos.x);  // a circle has no near point: use the body
    c.energy = energy(s, mu);
    c.bound = c.e < 1.0 - 1e-9;
    c.rPeri = c.p / (1.0 + c.e);
    if (c.bound) {
        const double a = c.p / (1.0 - c.e * c.e);
        c.rApo = c.p / (1.0 - c.e);
        c.period = 2.0 * kPi * std::sqrt(a * a * a / mu);
    }
    return c;
}

/// n + 1 points along the orbit, starting and ending at the far end of an ellipse (closed), or running from the cut-off
/// radius on one side through the near point to the cut-off on the other for an open orbit.
inline std::vector<Vec2> conicPoints(const Conic& c, int n, double maxRadius) {
    double nuMax = kPi;
    if (!c.bound) {
        const double cosCut = (c.p / maxRadius - 1.0) / c.e;  // where r(nu) = maxRadius
        nuMax = cosCut >= 1.0 ? 0.0 : std::acos(cosCut);
    }
    std::vector<Vec2> out;
    out.reserve(static_cast<std::size_t>(std::max(n, 0)) + 1);
    for (int i = 0; i <= n; ++i) {
        const double nu = -nuMax + 2.0 * nuMax * i / std::max(n, 1);
        const double r = c.p / (1.0 + c.e * std::cos(nu));
        const double angle = c.omega + c.dir * nu;
        out.push_back({r * std::cos(angle), r * std::sin(angle)});
    }
    return out;
}

/// Area swept out between the sun and the exact orbit from t0 to t1 (sum of `steps` thin triangles).
/// Kepler's second law: this is L / 2 * (t1 - t0) whatever the part of the orbit.
inline double sweptArea(const State& s0, double mu, double t0, double t1, int steps) {
    if (steps < 1 || !(t1 > t0)) return 0.0;
    Vec2 prev = propagate(s0, mu, t0).pos;
    double area = 0.0;
    for (int i = 1; i <= steps; ++i) {
        const Vec2 cur = propagate(s0, mu, t0 + (t1 - t0) * i / steps).pos;
        area += 0.5 * std::fabs(cross(prev, cur));
        prev = cur;
    }
    return area;
}

/// Speed for a circular orbit at `pos`, travelling counterclockwise in maths orientation.
inline State circularState(Vec2 pos, double mu) {
    const double r = length(pos);
    if (!(r > 1e-12) || !(mu > 0.0)) return {pos, {0.0, 0.0}};
    return {pos, Vec2{-pos.y, pos.x} * (std::sqrt(mu / r) / r)};
}

/// Start at the far point of an orbit with eccentricity e, at distance rApo in direction `angle`, counterclockwise.
inline State apsisState(double rApo, double e, double angle, double mu) {
    const double ecc = std::clamp(e, 0.0, 0.999);
    const double a = rApo / (1.0 + ecc);
    const double speed = std::sqrt(mu / a * (1.0 - ecc) / (1.0 + ecc));
    return {{rApo * std::cos(angle), rApo * std::sin(angle)}, {-speed * std::sin(angle), speed * std::cos(angle)}};
}

/// A drag with the mouse is a velocity: the vector scaled by `speedPerPx`.
inline State launchState(Vec2 start, Vec2 drag, double speedPerPx) { return {start, drag * speedPerPx}; }

/// Finite and between minR and maxR from the sun. Anything else is a body that fell in or flew off.
inline bool isUsable(const State& s, double minR, double maxR) {
    if (!std::isfinite(s.pos.x) || !std::isfinite(s.pos.y) || !std::isfinite(s.vel.x) || !std::isfinite(s.vel.y)) return false;
    const double r = length(s.pos);
    return r >= minR && r <= maxR;
}

}  // namespace orb
