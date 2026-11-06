// Hohmann transfer: the logic of the demo, no SDL, so test.cpp can check it.
//
// A ship circles a planet on a low orbit (radius r1) and wants to meet a target on a higher one (radius r2).
// The cheapest way with two short burns is half of an ellipse that touches both circles:
//
//   burn 1  at the low orbit, along the direction of travel:    dv1 = v1 * (sqrt(2 r2 / (r1 + r2)) - 1)
//   coast   half an ellipse with semi-major axis a = (r1 + r2) / 2:   T = pi * sqrt(a^3 / mu)
//   burn 2  at the far point, again along the direction of travel:   dv2 = v2 * (1 - sqrt(2 r1 / (r1 + r2)))
//
// where v = sqrt(mu / r) is the speed on a circle. Arriving on the right orbit is not enough, the target has to be
// there too. While the ship flies for T the target moves by n2 * T (n2 = sqrt(mu / r2^3)), and it has to end up exactly
// opposite the burn point, so at the moment of the burn it must lead the ship by pi - n2 * T. That angle is the phase
// angle; the burn waits until the two orbits have turned into it (the "window"). A burn toward the target at any other
// moment ends on a different orbit, not at the target.
//
// The exact orbit is propagated with the universal variable method (Stumpff functions), a copy of the one in
// orbit-mechanics, so burns and coasting have no step size and no drift.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

namespace hoh {

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kInf = std::numeric_limits<double>::infinity();
inline constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
inline constexpr double kAutoDelay = 0.6;  // the naive autopilot waits this long (seconds) before it fires

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

/// The direction of `a` with length 1; a vector too short to have a direction gives zero, never NaN.
inline Vec2 unit(Vec2 a) {
    const double len = length(a);
    return len > 1e-12 ? a * (1.0 / len) : Vec2{0.0, 0.0};
}

struct State {
    Vec2 pos;
    Vec2 vel;
};

/// Into (-pi, pi]. A NaN stays a NaN.
inline double wrapPi(double a) {
    double x = std::fmod(a, 2.0 * kPi);
    if (x > kPi) {
        x -= 2.0 * kPi;
    } else if (x <= -kPi) {
        x += 2.0 * kPi;
    }
    return x;
}

/// Into [0, 2 pi). A tiny negative angle that rounds up to 2 pi becomes 0.
inline double wrap2Pi(double a) {
    double x = std::fmod(a, 2.0 * kPi);
    if (x < 0.0) x += 2.0 * kPi;
    if (x >= 2.0 * kPi) x = 0.0;
    return x;
}

/// The angle from direction `a` to direction `b` as seen from the origin: positive counterclockwise, in (-pi, pi].
inline double phaseBetween(Vec2 a, Vec2 b) { return wrapPi(std::atan2(cross(a, b), dot(a, b))); }  // atan2 may give -pi for exactly opposite

// ------------------------------------------------------------------------------------------------ closed forms

inline double circularSpeed(double mu, double r) { return (mu > 0.0 && r > 0.0) ? std::sqrt(mu / r) : 0.0; }

/// Angular speed on a circle (radians per second).
inline double meanMotion(double mu, double r) { return (mu > 0.0 && r > 0.0) ? std::sqrt(mu / (r * r * r)) : 0.0; }

/// First burn, along the direction of travel. Negative when r2 < r1 (a retrograde burn to go down). Zero for bad input.
inline double hohmannDv1(double mu, double r1, double r2) {
    if (!(mu > 0.0) || !(r1 > 0.0) || !(r2 > 0.0)) return 0.0;
    return circularSpeed(mu, r1) * (std::sqrt(2.0 * r2 / (r1 + r2)) - 1.0);
}

/// Second burn, at the other end of the ellipse, also along the direction of travel.
inline double hohmannDv2(double mu, double r1, double r2) {
    if (!(mu > 0.0) || !(r1 > 0.0) || !(r2 > 0.0)) return 0.0;
    return circularSpeed(mu, r2) * (1.0 - std::sqrt(2.0 * r1 / (r1 + r2)));
}

inline double hohmannTotalDv(double mu, double r1, double r2) { return std::fabs(hohmannDv1(mu, r1, r2)) + std::fabs(hohmannDv2(mu, r1, r2)); }

/// Flight time of the half ellipse.
inline double hohmannTime(double mu, double r1, double r2) {
    if (!(mu > 0.0) || !(r1 > 0.0) || !(r2 > 0.0)) return 0.0;
    const double a = 0.5 * (r1 + r2);
    return kPi * std::sqrt(a * a * a / mu);
}

/// How far the target has to be ahead of the ship (counterclockwise, in (-pi, pi]) when the first burn starts. While the
/// ship flies half a turn the target covers n2 * T, and it must end up opposite the burn point.
inline double hohmannPhase(double mu, double r1, double r2) {
    if (!(mu > 0.0) || !(r1 > 0.0) || !(r2 > 0.0)) return 0.0;
    return wrapPi(kPi - meanMotion(mu, r2) * hohmannTime(mu, r1, r2));
}

/// Seconds until the phase angle (target ahead of ship, `phaseNow`) equals `needed`, for a ship turning at n1 and a target
/// turning at n2. The ship gains (n1 - n2) radians per second; infinite when both turn at the same speed.
inline double timeToWindow(double phaseNow, double needed, double n1, double n2) {
    const double w = n1 - n2;
    if (std::fabs(w) < 1e-12) return kInf;
    return w > 0.0 ? wrap2Pi(phaseNow - needed) / w : wrap2Pi(needed - phaseNow) / -w;
}

/// The phase angle to start with so that the window opens in `seconds`.
inline double phaseForWait(double mu, double r1, double r2, double seconds) {
    const double w = meanMotion(mu, r1) - meanMotion(mu, r2);
    return wrapPi(hohmannPhase(mu, r1, r2) + w * seconds);
}

// ------------------------------------------------------------------------------------------------ exact orbits

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

    // F(chi) = sqrt(mu) * time needed to reach chi, minus the time we want. dF/dchi is the distance r(chi) > 0,
    // so F is strictly increasing and a bracketed Newton iteration finds the single root.
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

/// A circular orbit at radius `r` and polar angle `angle`, travelling counterclockwise.
inline State circularState(double r, double angle, double mu) {
    const double v = circularSpeed(mu, r);
    return {{r * std::cos(angle), r * std::sin(angle)}, {-v * std::sin(angle), v * std::cos(angle)}};
}

/// The shape of an orbit, for drawing and for the numbers on screen. Positions on it follow
/// r(nu) = p / (1 + e cos nu) at the angle omega + dir * nu from the planet.
struct Conic {
    bool bound = false;    // ellipse (or circle); false for parabola and hyperbola
    double p = 0.0;        // semi-latus rectum, L^2 / mu
    double e = 0.0;        // eccentricity
    double omega = 0.0;    // direction of the near point
    double dir = 1.0;      // +1 counterclockwise, -1 the other way
    double rPeri = 0.0;    // distance at the near point
    double rApo = kInf;    // distance at the far point (infinite when not bound)
    double period = kInf;  // infinite when not bound
};

/// The orbit through a state. Nothing for a fall straight at the planet (L = 0), a body on top of it or mu <= 0.
inline std::optional<Conic> conicFromState(const State& s, double mu) {
    const double r = length(s.pos);
    const double speed = length(s.vel);
    if (!(mu > 0.0) || !(r > 1e-12) || !(speed > 0.0)) return std::nullopt;
    const double L = cross(s.pos, s.vel);
    if (std::fabs(L) <= 1e-9 * r * speed) return std::nullopt;
    Conic c;
    c.dir = L > 0.0 ? 1.0 : -1.0;
    c.p = L * L / mu;
    const Vec2 ev = (s.pos * (speed * speed - mu / r) - s.vel * dot(s.pos, s.vel)) * (1.0 / mu);
    c.e = length(ev);
    c.omega = c.e > 1e-9 ? std::atan2(ev.y, ev.x) : std::atan2(s.pos.y, s.pos.x);  // a circle has no near point: use the body
    c.bound = c.e < 1.0 - 1e-9;
    c.rPeri = c.p / (1.0 + c.e);
    if (c.bound) {
        const double a = c.p / (1.0 - c.e * c.e);
        c.rApo = c.p / (1.0 - c.e);
        c.period = 2.0 * kPi * std::sqrt(a * a * a / mu);
    }
    return c;
}

/// The point of the orbit at true anomaly `nu` (0 = near point, pi = far point).
inline Vec2 conicPoint(const Conic& c, double nu) {
    const double r = c.p / (1.0 + c.e * std::cos(nu));
    const double angle = c.omega + c.dir * nu;
    return {r * std::cos(angle), r * std::sin(angle)};
}

/// n + 1 points along the orbit, starting and ending at the far end of an ellipse (closed), or running from the cut-off
/// radius on one side through the near point to the cut-off on the other for an open orbit.
inline std::vector<Vec2> conicPoints(const Conic& c, int n, double maxRadius) {
    double nuMax = kPi;
    if (!c.bound) {
        const double cosCut = (c.p / maxRadius - 1.0) / c.e;  // where r(nu) = maxRadius
        nuMax = cosCut >= 1.0 ? 0.0 : std::acos(std::max(cosCut, -1.0));
    }
    std::vector<Vec2> out;
    out.reserve(static_cast<std::size_t>(std::max(n, 0)) + 1);
    for (int i = 0; i <= n; ++i) {
        out.push_back(conicPoint(c, -nuMax + 2.0 * nuMax * i / std::max(n, 1)));
    }
    return out;
}

/// The two ends of an ellipse.
enum class Apsis { Near, Far };

/// Seconds until the body reaches the near or far point of its ellipse (0 at the point itself). Nothing for a circle (every
/// point is as far as any other), an open orbit, a fall straight down or mu <= 0.
inline std::optional<double> timeToApsis(const State& s, double mu, Apsis which) {
    const auto c = conicFromState(s, mu);
    if (!c || !c->bound || c->e < 1e-6) return std::nullopt;
    const double r = length(s.pos);
    const double cosNu = std::clamp((c->p / r - 1.0) / c->e, -1.0, 1.0);
    double nu = std::acos(cosNu);
    if (dot(s.pos, s.vel) < 0.0) nu = 2.0 * kPi - nu;  // on the way in
    const double eccentric = 2.0 * std::atan2(std::sqrt(1.0 - c->e) * std::sin(nu / 2.0), std::sqrt(1.0 + c->e) * std::cos(nu / 2.0));
    const double mean = wrap2Pi(eccentric - c->e * std::sin(eccentric));
    const double a = c->p / (1.0 - c->e * c->e);
    const double n = std::sqrt(mu / (a * a * a));
    const double angle = which == Apsis::Far ? (mean <= kPi ? kPi - mean : 3.0 * kPi - mean) : (mean > 0.0 ? 2.0 * kPi - mean : 0.0);
    return angle / n;
}

// ------------------------------------------------------------------------------------------------ burns and the planet

/// A burn of `dv` along the direction of travel (negative = against it). A ship at rest has no such direction: unchanged.
inline State progradeBurn(State s, double dv) {
    s.vel = s.vel + unit(s.vel) * dv;
    return s;
}

/// A burn of `dv` toward a point. A target on the ship has no direction: unchanged.
inline State aimBurn(State s, Vec2 target, double dv) {
    s.vel = s.vel + unit(target - s.pos) * dv;
    return s;
}

/// Does the segment from a to b come closer than `radius` to `center`? (Touching does not count.) A radius of 0 never hits.
inline bool segmentHitsDisc(Vec2 a, Vec2 b, Vec2 center, double radius) {
    if (!(radius > 0.0)) return false;
    const Vec2 d = b - a;
    const double len2 = dot(d, d);
    if (len2 < 1e-300) return length(a - center) < radius;
    const double t = std::clamp(dot(center - a, d) / len2, 0.0, 1.0);
    return length(a + d * t - center) < radius;
}

// ------------------------------------------------------------------------------------------------ the mission

/// How a burn is aimed: along the direction of travel (the textbook way) or at the target (the way it looks right).
enum class Aim { Prograde, AtTarget };

struct Scenario {
    double mu = 1.0;
    double r1 = 1.0;            // start orbit
    double r2 = 4.0;            // target orbit
    double planetRadius = 0.0;  // a ship that touches this is lost; 0 switches the check off
};

/// One run: a ship that starts on the circle r1 at angle 0 and a target on the circle r2, both counterclockwise.
/// Burn 1 is planned for dv1, burn 2 for dv2 (Hohmann sizes), whatever the aim.
struct Mission {
    Scenario sc;
    Aim aim = Aim::Prograde;
    State ship;
    double targetAngle0 = 0.0;
    double time = 0.0;
    int burns = 0;  // 0 before the first burn, 1 on the way, 2 when both are done
    double dvSpent = 0.0;
    bool crashed = false;
    double missAtBurn2 = kNaN;  // distance between ship and target at the second burn
    Vec2 lastBurnPos;           // where, in which direction (unit vector of the velocity change) and how big the last burn was
    Vec2 lastBurnDir;
    double lastBurnDv = 0.0;
    std::array<double, 2> burnTime{kNaN, kNaN};
};

/// The ship on the start circle at angle 0, the target `targetPhase` radians ahead of it.
inline Mission startMission(const Scenario& sc, Aim aim, double targetPhase) {
    Mission m;
    m.sc = sc;
    m.aim = aim;
    m.ship = circularState(sc.r1, 0.0, sc.mu);
    m.targetAngle0 = targetPhase;
    return m;
}

inline State targetState(const Mission& m) {
    const double n2 = meanMotion(m.sc.mu, m.sc.r2);
    return circularState(m.sc.r2, m.targetAngle0 + n2 * m.time, m.sc.mu);
}

/// The end of the transfer ellipse where the second burn happens: the far point going up, the near point going down.
inline Apsis arrivalApsis(const Mission& m) { return m.sc.r2 >= m.sc.r1 ? Apsis::Far : Apsis::Near; }

/// How far the target is ahead of the ship now, in (-pi, pi].
inline double phaseNow(const Mission& m) { return phaseBetween(m.ship.pos, targetState(m).pos); }

/// How far the phase angle is from the one the transfer needs: positive = the window is still to come (for a ship on
/// the low orbit), zero = burn now.
inline double windowError(const Mission& m) { return wrapPi(phaseNow(m) - hohmannPhase(m.sc.mu, m.sc.r1, m.sc.r2)); }

/// The size of the next burn (0 when both are done).
inline double plannedDv(const Mission& m) {
    if (m.burns == 0) return hohmannDv1(m.sc.mu, m.sc.r1, m.sc.r2);
    if (m.burns == 1) return hohmannDv2(m.sc.mu, m.sc.r1, m.sc.r2);
    return 0.0;
}

/// Fires the next planned burn now. False when there is none left or the ship is lost.
inline bool burn(Mission& m) {
    if (m.crashed || m.burns >= 2) return false;
    const double dv = plannedDv(m);
    const Vec2 velBefore = m.ship.vel;
    m.ship = m.aim == Aim::Prograde ? progradeBurn(m.ship, dv) : aimBurn(m.ship, targetState(m).pos, dv);
    m.lastBurnPos = m.ship.pos;
    m.lastBurnDir = unit(m.ship.vel - velBefore);
    m.lastBurnDv = dv;
    m.dvSpent += std::fabs(dv);
    m.burnTime[static_cast<std::size_t>(m.burns)] = m.time;
    ++m.burns;
    if (m.burns == 2) m.missAtBurn2 = length(m.ship.pos - targetState(m).pos);
    return true;
}

/// Seconds until the autopilot would fire next, infinite when it has nothing left to do. The textbook aim waits for the
/// window and for the end of the ellipse; the naive aim fires once after a short delay and then leaves the ship alone.
inline double autoEventTime(const Mission& m) {
    if (m.crashed) return kInf;
    if (m.aim == Aim::AtTarget) return m.burns == 0 ? std::max(0.0, kAutoDelay - m.time) : kInf;
    if (m.burns == 0) {
        const double t = timeToWindow(phaseNow(m), hohmannPhase(m.sc.mu, m.sc.r1, m.sc.r2), meanMotion(m.sc.mu, m.sc.r1),
                                      meanMotion(m.sc.mu, m.sc.r2));
        return std::max(0.0, t);
    }
    if (m.burns == 1) {
        const auto t = timeToApsis(m.ship, m.sc.mu, arrivalApsis(m));
        return t ? std::max(0.0, *t) : kInf;
    }
    return kInf;
}

/// Moves the clock and the ship by `step` seconds (the target follows from the clock). With a planet the ship flies in
/// pieces that turn it by about 0.05 rad each, so a straight chord is a fair stand-in for the curve when it is checked
/// against the planet. A ship that touches the planet stays where it died.
inline void coast(Mission& m, double step) {
    m.time += step;
    if (m.crashed) return;
    if (!(m.sc.planetRadius > 0.0)) {
        m.ship = propagate(m.ship, m.sc.mu, step);
        return;
    }
    double left = step;
    while (left > 0.0) {
        const double speed = length(m.ship.vel);
        const double piece = speed > 1e-12 ? 0.05 * length(m.ship.pos) / speed : left;
        const double h = std::min(left, std::max(piece, step * 1e-4));
        const State next = propagate(m.ship, m.sc.mu, h);
        if (segmentHitsDisc(m.ship.pos, next.pos, {0.0, 0.0}, m.sc.planetRadius) || length(next.pos) < m.sc.planetRadius) {
            m.crashed = true;
            return;
        }
        m.ship = next;
        left -= h;
    }
}

/// Advances the mission by `dt` seconds. With the autopilot on, the burns happen at their exact moment inside the step.
inline void advance(Mission& m, double dt, bool autopilot) {
    if (!(dt > 0.0)) return;
    double remaining = dt;
    while (remaining > 0.0) {
        const double event = autopilot ? autoEventTime(m) : kInf;
        if (event <= remaining) {
            coast(m, event);
            remaining -= event;
            if (!burn(m)) break;  // cannot happen (no event without a burn left), but never loop on it
        } else {
            coast(m, remaining);
            break;
        }
    }
}

/// Is the burn about to be right? First burn: the phase angle is within `tolAngle` of the one the transfer needs. Second
/// burn: the ship is within `tolTime` seconds of the far point (before or after). Only for the textbook aim.
inline bool burnReady(const Mission& m, double tolAngle, double tolTime) {
    if (m.crashed || m.aim != Aim::Prograde) return false;
    if (m.burns == 0) return std::fabs(windowError(m)) <= tolAngle;
    if (m.burns == 1) {
        const auto c = conicFromState(m.ship, m.sc.mu);
        const auto t = timeToApsis(m.ship, m.sc.mu, arrivalApsis(m));
        return c && t && (*t <= tolTime || *t >= c->period - tolTime);
    }
    return false;
}

/// Both burns done, the ship ended up closer than `missTol` to the target, on a nearly circular orbit.
inline bool rendezvous(const Mission& m, double missTol) {
    if (m.crashed || m.burns < 2 || !(m.missAtBurn2 < missTol)) return false;
    const auto c = conicFromState(m.ship, m.sc.mu);
    return c && c->bound && c->e < 0.1;
}

}  // namespace hoh
