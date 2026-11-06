// Tests for the gravity assist logic. Plain asserts on hand-computed cases, no framework, no SDL.
// Units are pixels and seconds. The planet moves along +x at planetSpeed, the probe feels only the planet.
#undef NDEBUG
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

#include "logic.hpp"

using namespace ga;

namespace {

bool near(double a, double b, double tol = 1e-9) { return std::fabs(a - b) <= tol; }
bool nearVec(Vec2 a, Vec2 b, double tol = 1e-9) { return near(a.x, b.x, tol) && near(a.y, b.y, tol); }

World plainWorld(double mu, double planetSpeed) {
    World w;
    w.mu = mu;
    w.planetSpeed = planetSpeed;
    w.softening = 0.0;
    return w;
}

Flight fly(const World& w, const Launch& l, const Limits& lim = {}) {
    Flight f;
    simulate(w, worldState(w, l), f, lim);
    return f;
}

void vectors() {
    assert(near(length({3, 4}), 5.0));
    assert(nearVec(normalized({3, 4}), {0.6, 0.8}));
    // The zero vector must not turn into NaN.
    assert(nearVec(normalized({0, 0}), {0, 0}));
    assert(nearVec(rotated({1, 0}, kPi / 2), {0, 1}, 1e-12));
    assert(nearVec(rotated({0, -60}, kPi / 2), {60, 0}, 1e-12));
    assert(near(signedAngle({1, 0}, {0, 1}), kPi / 2));
    assert(near(signedAngle({0, 1}, {1, 0}), -kPi / 2));
    assert(near(signedAngle({2, 0}, {5, 0}), 0.0));
    assert(near(std::fabs(signedAngle({1, 0}, {-1, 0})), kPi));
    // No direction, no angle.
    assert(near(signedAngle({0, 0}, {1, 0}), 0.0));
    assert(near(signedAngle({1, 0}, {0, 0}), 0.0));
}

void planetMotion() {
    World w = plainWorld(1.0, 60.0);
    w.meetTime = 2.6;
    w.meetPoint = {10, 20};
    assert(nearVec(planetVelocity(w), {60, 0}));
    assert(nearVec(planetPosition(w, 2.6), {10, 20}));
    assert(nearVec(planetPosition(w, 0.0), {10 - 156, 20}, 1e-9));
    assert(nearVec(planetPosition(w, 3.6), {70, 20}, 1e-9));
    // A planet that stands still stays where the others meet it.
    w.planetSpeed = 0.0;
    assert(nearVec(planetPosition(w, 0.0), {10, 20}));
    assert(nearVec(planetPosition(w, 99.0), {10, 20}));
}

void gravity() {
    World w = plainWorld(4.0, 0.0);
    w.meetPoint = {0, 0};
    const Vec2 a1 = acceleration(w, {2, 0}, 0.0);  // -mu * r / |r|^3 = -4 * 2 / 8
    assert(nearVec(a1, {-1.0, 0.0}));
    w.mu = 1.0;
    assert(nearVec(acceleration(w, {0, -1}, 0.0), {0.0, 1.0}));  // pulls back toward the planet
    w.mu = 125.0;
    assert(nearVec(acceleration(w, {3, 4}, 0.0), {-3.0, -4.0}));  // |r| = 5, |r|^3 = 125
    // Inverse square: twice as far, a quarter of the pull.
    w.mu = 1.0;
    assert(near(length(acceleration(w, {1, 0}, 0.0)), 4.0 * length(acceleration(w, {2, 0}, 0.0))));
    // Sitting exactly on the planet must not produce NaN or infinity, with and without softening.
    const Vec2 a0 = acceleration(w, {0, 0}, 0.0);
    assert(std::isfinite(a0.x) && std::isfinite(a0.y));
    w.softening = 2.0;
    const Vec2 as = acceleration(w, {0, 0}, 0.0);
    assert(near(as.x, 0.0) && near(as.y, 0.0));
    // With softening the pull at r = 2 is mu * r / (r^2 + eps^2)^1.5 = 2 / 8^1.5.
    assert(near(length(acceleration(w, {2, 0}, 0.0)), 2.0 / std::pow(8.0, 1.5)));
    // The planet at the meeting time is where the pull points, not at the origin.
    w.softening = 0.0;
    w.planetSpeed = 10.0;
    w.meetTime = 1.0;
    w.meetPoint = {100, 0};
    const Vec2 a = acceleration(w, {100, 5}, 1.0);  // directly above the planet at t = 1
    assert(near(a.x, 0.0) && a.y < 0.0);
}

void launchConversion() {
    World w = plainWorld(1.0, 60.0);
    w.meetTime = 2.6;
    w.meetPoint = {0, 0};
    const Launch l{{-34, 150}, {0, -60}};
    const State s = worldState(w, l);
    assert(nearVec(s.pos, {-156 - 34, 150}, 1e-9));  // planet at t = 0 is at -156
    assert(nearVec(s.vel, {60, -60}));               // sun frame velocity = planet velocity + relative velocity
    const Launch back = relativeLaunch(w, s.pos, s.vel);
    assert(nearVec(back.rel, l.rel) && nearVec(back.u, l.u));
    // With a standing planet the two frames are the same.
    w.planetSpeed = 0.0;
    const State still = worldState(w, l);
    assert(nearVec(still.vel, l.u) && nearVec(still.pos, l.rel));
}

void startZone() {
    const World w = plainWorld(1.0, 0.0);  // zone 100, margin 4
    assert(startAllowed(w, {200, 0}));
    assert(!startAllowed(w, {100, 0}));
    assert(!startAllowed(w, {0, 0}));
    assert(!startAllowed(w, {104, 0}));  // the margin is excluded
    assert(startAllowed(w, {104.1, 0}));
    assert(startAllowed(w, {-80, 80}));  // 113 away
}

void straightLine() {
    // No mass, no pull: the probe keeps its velocity (a straight line, sampled every step).
    World w = plainWorld(0.0, 0.0);
    w.meetPoint = {-2000, 0};
    Limits lim;
    lim.maxTime = 1.0;
    Flight f;
    simulate(w, State{{10, 20}, {30, -40}}, f, lim);
    assert(f.samples.size() == 201);  // t = 0 and 200 steps of 0.005
    assert(near(f.samples.front().t, 0.0));
    assert(near(f.samples.back().t, 1.0, 1e-9));
    assert(nearVec(f.samples.back().pos, {40, -20}, 1e-9));
    assert(nearVec(f.samples.back().vel, {30, -40}, 1e-9));
    assert(f.outcome == Outcome::Miss);
    assert(f.enter < 0 && f.exit < 0);
    assert(!analyze(w, f).valid);
}

void leavesTheArena() {
    // A probe far away and flying farther away stops early, there is nothing left to see.
    World w = plainWorld(90000.0, 60.0);
    const Flight f = fly(w, Launch{{0, 1000}, {0, 100}});
    assert(f.outcome == Outcome::Miss);
    assert(f.samples.back().t < 5.0);
    assert(duration(f) == f.samples.back().t);
}

void sampling() {
    World w = plainWorld(0.0, 0.0);
    w.meetPoint = {-2000, 0};
    Limits lim;
    lim.maxTime = 1.0;
    Flight f;
    simulate(w, State{{0, 0}, {100, 0}}, f, lim);
    assert(nearVec(sampleAt(f, 0.0).pos, {0, 0}));
    // Between two samples the position is interpolated: 0.0075 s is a step and a half.
    const Sample half = sampleAt(f, 0.0075);
    assert(near(half.t, 0.0075, 1e-12));
    assert(nearVec(half.pos, {0.75, 0}, 1e-9));
    assert(nearVec(sampleAt(f, 0.5).pos, {50, 0}, 1e-9));
    // Out of range clamps to the first and last sample.
    assert(nearVec(sampleAt(f, -3.0).pos, {0, 0}));
    assert(nearVec(sampleAt(f, 50.0).pos, f.samples.back().pos));
    // An empty flight returns a zero sample instead of crashing.
    Flight empty;
    assert(nearVec(sampleAt(empty, 1.0).pos, {0, 0}) && near(duration(empty), 0.0));
}

void zoneCrossing() {
    World w = plainWorld(1.0, 0.0);
    w.meetPoint = {0, 0};
    const Sample a{1.0, {-101, 0}, {10, 0}};
    const Sample b{2.0, {-99, 0}, {20, 0}};
    const Sample c = crossing(w, a, b);  // the zone edge is at x = -100, half way
    assert(near(c.t, 1.5) && nearVec(c.pos, {-100, 0}) && nearVec(c.vel, {15, 0}));
    // The same pair the other way round (leaving) crosses at the same place.
    const Sample d = crossing(w, b, a);
    assert(near(d.t, 1.5) && nearVec(d.pos, {-100, 0}));
    // Two samples at the same distance do not divide by zero.
    const Sample e = crossing(w, Sample{0.0, {-100, 0}, {1, 0}}, Sample{1.0, {100, 0}, {2, 0}});
    assert(std::isfinite(e.t) && std::isfinite(e.pos.x));
}

/// Speed at the zone edge from the energy of the planet frame: u^2 = u0^2 + 2 mu (1/D - 1/r0).
void energyAtTheZoneEdge() {
    const World w = plainWorld(90000.0, 0.0);
    const Launch l{{290, 40}, {-55, 0}};
    const Flight f = fly(w, l);
    const Encounter e = analyze(w, f);
    assert(f.outcome == Outcome::Flyby && e.valid);
    const double r0 = length(l.rel);
    const double expected = std::sqrt(55.0 * 55.0 + 2.0 * w.mu * (1.0 / w.zone - 1.0 / r0));
    assert(near(e.planetIn, expected, 0.1));  // about 64.89
    assert(near(e.planetOut, expected, 0.1));
    // A planet that stands still: the sun frame is the planet frame, so no gain at all.
    assert(near(e.sunIn, e.planetIn, 1e-9) && near(e.sunOut, e.planetOut, 1e-9));
    assert(near(e.gain, 0.0, 0.1));
}

void angularMomentum() {
    // With a standing planet the pull is central, so rel x v is constant along the whole flight.
    const World w = plainWorld(90000.0, 0.0);
    const Launch l{{290, 40}, {-55, 0}};
    const Flight f = fly(w, l);
    const double l0 = cross(l.rel, l.u);  // 290 * 0 - 40 * (-55) = 2200
    assert(near(l0, 2200.0));
    for (std::size_t i = 0; i < f.samples.size(); i += 50) {
        const Sample& s = f.samples[i];
        const Vec2 rel = s.pos - planetPosition(w, s.t);
        assert(near(cross(rel, s.vel), l0, 2.0));
    }
}

/// The exact turn between the velocities at the two crossings of a circle of radius D, from the hyperbola:
/// e = sqrt(1 + (b uInf^2 / mu)^2), p = (b uInf)^2 / mu, cos(theta_D) = (p / D - 1) / e, tan(gamma) = e sin / (1 + e cos),
/// turn = 2 (theta_D - gamma). For D toward infinity this is the textbook 2 asin(1/e).
double exactTurn(double mu, double uInf, double b, double d) {
    const double e = std::sqrt(1.0 + (b * uInf * uInf / mu) * (b * uInf * uInf / mu));
    const double p = b * b * uInf * uInf / mu;
    const double theta = std::acos((p / d - 1.0) / e);
    const double gamma = std::atan2(e * std::sin(theta), 1.0 + e * std::cos(theta));
    return 2.0 * (theta - gamma);
}

void turnAgainstTheHyperbola() {
    World w = plainWorld(90000.0, 0.0);
    w.zone = 4000.0;
    Limits lim;
    lim.maxTime = 300.0;
    lim.tail = 1.0;
    lim.bound = 1e9;
    const double u0 = 100.0;
    const double r0 = std::sqrt(4500.0 * 4500.0 + 50.0 * 50.0);
    const double uInf = std::sqrt(u0 * u0 - 2.0 * w.mu / r0);  // energy: the speed far away, before the pull is felt
    const Flight f = fly(w, Launch{{-4500, 50}, {u0, 0}}, lim);
    const Encounter e = analyze(w, f);
    assert(e.valid && f.outcome == Outcome::Flyby);
    // The impact parameter follows from the angular momentum: L = |rel x u| = 5000 = b uInf.
    const double b = std::fabs(cross({-4500, 50}, {u0, 0})) / uInf;
    const double expected = exactTurn(w.mu, uInf, b, w.zone);
    assert(near(std::fabs(e.turn), expected, 0.002));  // radians, about 0.1 degrees
    // For a huge zone this is the textbook 2 atan(mu / (b uInf^2)); here the zone is large enough to land within a degree.
    assert(near(std::fabs(e.turn), 2.0 * std::atan(w.mu / (b * uInf * uInf)), 0.02));
}

void gainFromTheMovingPlanet() {
    const World w = plainWorld(90000.0, 60.0);
    const Launch l{{290, 40}, {-55, 0}};
    const Flight f = fly(w, l);
    const Encounter e = analyze(w, f);
    assert(f.outcome == Outcome::Flyby && e.valid);
    // Planet frame: the speed comes back (the pull is conservative), the direction does not.
    assert(near(e.planetIn, e.planetOut, 0.15));
    assert(std::fabs(e.turn) > 1.0);
    // Sun frame: v = V + u, so |v_out|^2 - |v_in|^2 = 2 V.(u_out - u_in) + |u_out|^2 - |u_in|^2 (exact algebra).
    const Vec2 v = planetVelocity(w);
    const double lhs = dot(e.vOut, e.vOut) - dot(e.vIn, e.vIn);
    const double rhs = 2.0 * dot(v, e.uOut - e.uIn) + dot(e.uOut, e.uOut) - dot(e.uIn, e.uIn);
    assert(near(lhs, rhs, 1e-6));
    assert(nearVec(e.vIn, v + e.uIn, 1e-9) && nearVec(e.vOut, v + e.uOut, 1e-9));
    assert(near(e.gain, e.sunOut - e.sunIn, 1e-12));
    assert(e.gain > 50.0);  // from nearly standing still to over 70
    assert(e.tIn < e.tOut);
}

void textbookNinetyDegrees() {
    // The textbook case: planet frame speed v, turned by 90 degrees, planet moves at v. The probe goes from sqrt(2) v to 2 v.
    const Vec2 planet{60, 0};
    const Vec2 uIn{0, -60};
    const Vec2 uOut = rotated(uIn, kPi / 2);
    assert(nearVec(uOut, {60, 0}, 1e-9));
    assert(near(length(planet + uIn), 60.0 * std::sqrt(2.0), 1e-9));  // 84.85
    assert(near(length(planet + uOut), 120.0, 1e-9));
    assert(near(length(planet + uOut) - length(planet + uIn), 60.0 * (2.0 - std::sqrt(2.0)), 1e-9));  // 35.15, about 0.59 v
}

void gainIsAtMostTwicePlanetSpeed() {
    // Whatever the turn angle, the speed in the sun frame grows by at most 2 V (a U-turn against the planet's motion).
    const Vec2 planet{60, 0};
    const Vec2 uIn{-100, 0};
    double best = -1e9;
    double bestAngle = 0.0;
    for (int i = 0; i < 3600; ++i) {
        const double angle = kPi * 2.0 * i / 3600.0;
        const double gain = length(planet + rotated(uIn, angle)) - length(planet + uIn);
        if (gain > best) {
            best = gain;
            bestAngle = angle;
        }
    }
    assert(near(best, 120.0, 1e-9));           // 160 - 40
    assert(near(bestAngle, kPi, 1e-9));        // the full U-turn
    // Without a turn there is no gain, and braking works the other way round (turning the other side of the circle).
    assert(near(length(planet + rotated(uIn, 0.0)) - length(planet + uIn), 0.0, 1e-12));
}

void outcomes() {
    const World w = plainWorld(90000.0, 0.0);
    // Straight at the planet: crash, and the flight ends there.
    const Flight crash = fly(w, Launch{{0, 200}, {0, -100}});
    assert(crash.outcome == Outcome::Crash);
    assert(crash.closest < w.planetRadius);
    assert(!analyze(w, crash).valid);
    assert(crash.samples.back().t < 3.0);
    // Far past the planet and leaving: a miss without a zone crossing.
    const Flight miss = fly(w, Launch{{500, 300}, {40, 20}});
    assert(miss.outcome == Outcome::Miss && miss.enter < 0);
    assert(miss.closest > 400.0);
    // Too slow to leave: the energy at the zone edge is negative, the probe stays bound and the flight runs to the time limit.
    // Hand check: L = 150 * 20 = 3000, E = 200 - 600 = -400, apsides r = 75 and 150 (400 r^2 - 90000 r + 4.5e6 = 0).
    const Flight bound = fly(w, Launch{{150, 0}, {0, -20}});
    assert(bound.outcome == Outcome::Bound);
    assert(bound.enter > 0 && !analyze(w, bound).valid);
    assert(near(bound.closest, 75.0, 0.5));
    assert(near(bound.samples.back().t, 30.0, 0.01));
}

void presets() {
    for (int mass = 0; mass < static_cast<int>(kMasses.size()); ++mass) {
        for (int speed = 0; speed < static_cast<int>(kSpeeds.size()); ++speed) {
            World w;
            w.mu = kMasses[static_cast<std::size_t>(mass)];
            w.planetSpeed = kSpeeds[static_cast<std::size_t>(speed)];
            for (int p = 0; p < kPresetCount; ++p) {
                const Flight f = fly(w, preset(p));
                const Encounter e = analyze(w, f);
                // Every preset is a clean flyby for every mass and planet speed, and the planet frame never changes with the speed.
                assert(f.outcome == Outcome::Flyby && e.valid);
                assert(near(e.planetIn, e.planetOut, 0.3));
                if (w.planetSpeed == 0.0) assert(near(e.gain, 0.0, 0.3));
            }
        }
    }
    // The three headline numbers at medium mass and planet speed 60 (values from the prototype, within 2 px/s).
    World w;
    const Encounter behind = analyze(w, fly(w, preset(0)));
    const Encounter front = analyze(w, fly(w, preset(1)));
    const Encounter rest = analyze(w, fly(w, preset(2)));
    assert(behind.gain > 33.0 && behind.gain < 37.0);   // about +35
    assert(front.gain < -70.0 && front.gain > -76.0);   // about -73
    assert(rest.gain > 68.0 && rest.gain < 73.0);       // about +70
    assert(behind.sunIn < 92.0 && behind.sunOut > 120.0);
    assert(rest.sunIn < 10.0);  // starts almost at rest in the sun frame
    // Passing behind the planet speeds the probe up, passing in front slows it down: the sign of the turn flips.
    assert(behind.turn * front.turn < 0.0);
}

void diagramFit() {
    struct Case {
        Vec2 planet;
        double u;
    };
    const Case cases[] = {{{60, 0}, 60}, {{0, 0}, 60}, {{90, 0}, 10}, {{60, 0}, 0}, {{0, 0}, 0}, {{30, 0}, 70}};
    for (const Case& c : cases) {
        const DiagramFit fit = fitDiagram(c.planet, c.u, 260.0, 150.0, 8.0);
        assert(std::isfinite(fit.scale) && fit.scale > 0.0);
        // The origin, the two extreme points of the circle in x and in y all land inside the area minus the margin.
        const Vec2 pts[] = {{0, 0}, c.planet + Vec2{c.u, 0}, c.planet + Vec2{-c.u, 0}, c.planet + Vec2{0, c.u}, c.planet + Vec2{0, -c.u}};
        for (const Vec2& p : pts) {
            const Vec2 m = fit.map(p);
            assert(m.x >= 8.0 - 1e-9 && m.x <= 260.0 - 8.0 + 1e-9);
            assert(m.y >= 8.0 - 1e-9 && m.y <= 150.0 - 8.0 + 1e-9);
        }
    }
    // Hand case: the circle around (60, 0) with radius 60 spans x in [0, 120] and y in [-60, 60], a 120 x 120 box.
    // Available area 244 x 134 gives scale 134 / 120 = 1.11667, centred horizontally.
    const DiagramFit fit = fitDiagram({60, 0}, 60.0, 260.0, 150.0, 8.0);
    assert(near(fit.scale, 134.0 / 120.0, 1e-9));
    assert(near(fit.map({0, 0}).x, 8.0 + (244.0 - 120.0 * fit.scale) / 2.0, 1e-9));
    assert(near(fit.map({60, -60}).y, 8.0, 1e-9));
}

}  // namespace

int main() {
    vectors();
    planetMotion();
    gravity();
    launchConversion();
    startZone();
    straightLine();
    leavesTheArena();
    sampling();
    zoneCrossing();
    energyAtTheZoneEdge();
    angularMomentum();
    turnAgainstTheHyperbola();
    gainFromTheMovingPlanet();
    textbookNinetyDegrees();
    gainIsAtMostTwicePlanetSpeed();
    outcomes();
    presets();
    diagramFit();
    std::puts("gravity-assist logic: all asserts passed");
    return 0;
}
