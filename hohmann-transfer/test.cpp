// Tests for the Hohmann transfer logic. Plain asserts on hand-computed cases, no framework, no SDL.
// Units are free: most cases use mu = 1 and a start radius of 1, so the start orbit has speed 1 and period 2 pi.
// The main case is r1 = 1, r2 = 4 (a ratio of four, so the numbers come out round):
//   transfer ellipse a = 2.5, speed at the near point sqrt(2 * 4 / 5) = 1.26491, at the far point 1.26491 / 4 = 0.316228
//   dv1 = 0.264911, dv2 = 0.5 - 0.316228 = 0.183772, flight time pi * sqrt(2.5^3) = 12.41824
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdio>

#include "logic.hpp"

using namespace hoh;

namespace {

bool near(double a, double b, double tol = 1e-9) { return std::fabs(a - b) <= tol; }

void angleHelpers() {
    assert(near(wrapPi(0.0), 0.0));
    assert(near(wrapPi(kPi), kPi));          // the half-open interval keeps +pi
    assert(near(wrapPi(-kPi), kPi));         // and never returns -pi
    assert(near(std::fabs(wrapPi(3.0 * kPi)), kPi));  // odd multiples of pi land on the edge, whichever side rounding picks
    assert(near(wrapPi(2.0 * kPi + 0.5), 0.5));
    assert(near(wrapPi(-2.0 * kPi - 0.5), -0.5));
    assert(near(wrap2Pi(-0.5), 2.0 * kPi - 0.5));
    assert(near(wrap2Pi(2.0 * kPi), 0.0));
    assert(near(wrap2Pi(7.0), 7.0 - 2.0 * kPi));
    assert(near(wrap2Pi(0.0), 0.0));
    // A NaN stays a NaN instead of looping or turning into a number.
    assert(std::isnan(wrapPi(std::nan(""))));

    assert(near(phaseBetween({1, 0}, {0, 3}), kPi / 2.0));    // target a quarter turn ahead
    assert(near(phaseBetween({1, 0}, {0, -3}), -kPi / 2.0));  // and behind
    assert(near(phaseBetween({0, 2}, {0, -5}), kPi));         // opposite
    assert(near(phaseBetween({1, 1}, {2, 2}), 0.0));          // same direction, other distance
    assert(near(phaseBetween({-1, 0}, {0, -1}), kPi / 2.0));  // from the left to the bottom is counterclockwise
}

void vectorHelpers() {
    const Vec2 u = unit({3, 4});
    assert(near(u.x, 0.6) && near(u.y, 0.8));
    const Vec2 z = unit({0, 0});  // no direction: zero, not NaN
    assert(z.x == 0.0 && z.y == 0.0);
    const Vec2 tiny = unit({1e-20, 0});
    assert(std::isfinite(tiny.x) && std::isfinite(tiny.y));
}

void closedForms() {
    assert(near(circularSpeed(1.0, 1.0), 1.0));
    assert(near(circularSpeed(4.0, 4.0), 1.0));  // sqrt(4 / 4)
    assert(near(circularSpeed(1.0, 4.0), 0.5));
    assert(near(meanMotion(1.0, 4.0), 0.125));   // sqrt(1 / 64)
    assert(near(meanMotion(1.0, 1.0), 1.0));

    assert(near(hohmannDv1(1.0, 1.0, 4.0), 0.26491106406735176, 1e-12));  // sqrt(1.6) - 1
    assert(near(hohmannDv2(1.0, 1.0, 4.0), 0.18377223398316206, 1e-12));  // 0.5 * (1 - sqrt(0.4))
    assert(near(hohmannTime(1.0, 1.0, 4.0), 12.418235332245125, 1e-9));   // pi * sqrt(15.625)
    assert(near(hohmannTotalDv(1.0, 1.0, 4.0), 0.26491106406735176 + 0.18377223398316206, 1e-12));
    // Scaling the mass of the sun by 4 doubles every speed and halves every time.
    assert(near(hohmannDv1(4.0, 1.0, 4.0), 2.0 * hohmannDv1(1.0, 1.0, 4.0), 1e-12));
    assert(near(hohmannTime(4.0, 1.0, 4.0), 0.5 * hohmannTime(1.0, 1.0, 4.0), 1e-9));

    // Going down is the same trip backwards: both burns point against the motion, same sizes in reverse order.
    assert(hohmannDv1(1.0, 4.0, 1.0) < 0.0 && hohmannDv2(1.0, 4.0, 1.0) < 0.0);
    assert(near(hohmannDv1(1.0, 4.0, 1.0), -0.18377223398316206, 1e-12));  // 0.5 * (sqrt(0.4) - 1)
    assert(near(hohmannDv2(1.0, 4.0, 1.0), -0.26491106406735176, 1e-12));
    assert(near(hohmannTotalDv(1.0, 4.0, 1.0), hohmannTotalDv(1.0, 1.0, 4.0), 1e-12));
    assert(near(hohmannTime(1.0, 4.0, 1.0), hohmannTime(1.0, 1.0, 4.0), 1e-12));

    // Same orbit: nothing to do.
    assert(near(hohmannDv1(1.0, 2.0, 2.0), 0.0));
    assert(near(hohmannDv2(1.0, 2.0, 2.0), 0.0));
    // Nonsense input gives zero instead of NaN.
    assert(hohmannDv1(0.0, 1.0, 4.0) == 0.0);
    assert(hohmannDv1(1.0, 0.0, 4.0) == 0.0);
    assert(hohmannDv2(1.0, 1.0, -4.0) == 0.0);
    assert(hohmannTime(-1.0, 1.0, 4.0) == 0.0);
    assert(hohmannPhase(1.0, 0.0, 4.0) == 0.0);
}

void phaseAngle() {
    // r2 = 4: n2 = 0.125, flight time 12.418235, the target covers 1.552279 rad meanwhile, so it must lead by pi - 1.552279.
    assert(near(hohmannPhase(1.0, 1.0, 4.0), 1.5893132370591525, 1e-9));
    // Earth to Mars: ratio 1.524. The textbook value is about 44 degrees.
    const double mars = hohmannPhase(1.0, 1.0, 1.524) * 180.0 / kPi;
    assert(mars > 44.2 && mars < 44.5);
    // Ratio 2: 63.09 degrees (pi - n2 * T with n2 = 2^-1.5, T = pi * 1.5^1.5).
    assert(near(hohmannPhase(1.0, 1.0, 2.0) * 180.0 / kPi, 63.08657048910078, 1e-7));
    // Down from 4 to 1 the target is faster and has to trail: a negative lead, -171.5 degrees.
    const double down = hohmannPhase(1.0, 4.0, 1.0);
    assert(down < 0.0 && near(down, wrapPi(kPi - 12.418235332245125), 1e-9));
    assert(near(down * 180.0 / kPi, -171.5, 0.1));
    // Always inside the half-open interval.
    assert(hohmannPhase(1.0, 1.0, 100.0) <= kPi && hohmannPhase(1.0, 1.0, 100.0) > -kPi);
}

void waiting() {
    const double needed = hohmannPhase(1.0, 1.0, 4.0);  // 1.5893132
    // n1 = 1, n2 = 0.125, so the ship gains 0.875 rad per time unit on the target.
    // Target opposite the ship (pi): pi - 1.5893132 = 1.5522795 rad to catch up.
    assert(near(timeToWindow(kPi, needed, 1.0, 0.125), 1.5522795 / 0.875, 1e-6));
    // Window just passed (target 1 rad ahead, needed 1.589): it takes nearly a whole synodic period of 2 pi / 0.875 = 7.18.
    assert(near(timeToWindow(1.0, needed, 1.0, 0.125), (2.0 * kPi - (needed - 1.0)) / 0.875, 1e-9));
    // Right now.
    assert(near(timeToWindow(needed, needed, 1.0, 0.125), 0.0, 1e-12));
    // A target 1 rad behind the ship has to be caught from the other side: 2 pi - 1 - 1.589 rad of gain.
    assert(near(timeToWindow(-1.0, needed, 1.0, 0.125), (2.0 * kPi - 1.0 - needed) / 0.875, 1e-9));
    // Same orbit, same speed: the phase never changes, the window never comes.
    assert(std::isinf(timeToWindow(1.0, 0.5, 1.0, 1.0)));
    // Down: the target is faster, the phase grows. n1 = 0.125, n2 = 1, target 0 rad ahead, needed -171.5 degrees = 3.2898 rad mod 2 pi.
    const double needDown = hohmannPhase(1.0, 4.0, 1.0);
    assert(near(timeToWindow(0.0, needDown, 0.125, 1.0), wrap2Pi(needDown - 0.0) / 0.875, 1e-9));
    // And the pair always agrees with phaseForWait.
    for (const double seconds : {0.0, 0.5, 2.5, 7.0}) {
        const double start = phaseForWait(1.0, 1.0, 4.0, seconds);
        assert(near(timeToWindow(start, needed, 1.0, 0.125), seconds, 1e-9));
    }
    for (const double seconds : {0.5, 2.5}) {
        const double start = phaseForWait(1.0, 4.0, 1.0, seconds);
        assert(near(timeToWindow(start, needDown, 0.125, 1.0), seconds, 1e-9));
    }
}

void burnsAndGeometry() {
    const State s{{1, 0}, {0, 1}};
    const State up = progradeBurn(s, 0.5);
    assert(near(up.vel.x, 0.0) && near(up.vel.y, 1.5) && near(up.pos.x, 1.0));
    const State back = progradeBurn(s, -0.25);  // negative = retrograde
    assert(near(back.vel.y, 0.75));
    const State diag = progradeBurn({{0, 0}, {3, 4}}, 5.0);  // along (0.6, 0.8): adds (3, 4)
    assert(near(diag.vel.x, 6.0) && near(diag.vel.y, 8.0));
    const State drift = progradeBurn({{1, 0}, {0, 0}}, 1.0);  // no velocity, no prograde: unchanged
    assert(drift.vel.x == 0.0 && drift.vel.y == 0.0);

    const State aimed = aimBurn(s, {1, 5}, 2.0);  // straight up from (1, 0): +2 in y
    assert(near(aimed.vel.x, 0.0) && near(aimed.vel.y, 3.0));
    const State sideways = aimBurn(s, {-3, 0}, 1.0);  // toward the sun side
    assert(near(sideways.vel.x, -1.0) && near(sideways.vel.y, 1.0));
    const State onTop = aimBurn(s, {1, 0}, 1.0);  // target on the ship: no direction
    assert(near(onTop.vel.x, 0.0) && near(onTop.vel.y, 1.0));

    // The disc test: a chord through, a tangent touch (not a hit), a miss, an end inside, a segment that stops short.
    assert(segmentHitsDisc({-2, 0}, {2, 0}, {0, 0}, 1.0));
    assert(!segmentHitsDisc({-2, 1}, {2, 1}, {0, 0}, 1.0));
    assert(!segmentHitsDisc({-2, 2}, {2, 2}, {0, 0}, 1.0));
    assert(segmentHitsDisc({0.5, 0}, {5, 0}, {0, 0}, 1.0));
    assert(!segmentHitsDisc({-5, 0}, {-2, 0}, {0, 0}, 1.0));
    assert(segmentHitsDisc({0.2, 0.2}, {0.2, 0.2}, {0, 0}, 1.0));   // a point inside
    assert(!segmentHitsDisc({3, 0}, {3, 0}, {0, 0}, 1.0));          // a point outside
    assert(!segmentHitsDisc({-2, 0}, {2, 0}, {0, 0}, 0.0));         // no disc
    assert(segmentHitsDisc({-2, 0.9}, {2, 0.9}, {0, 0}, 1.0));      // a shallow chord
    assert(segmentHitsDisc({2, 2}, {2, -2}, {2, 0.5}, 0.25));       // disc away from the origin
}

void ellipseOfTheTransfer() {
    // Burn at (1, 0) with dv1: the orbit is the transfer ellipse, near point at 1, far point at 4.
    const State burned = progradeBurn(circularState(1.0, 0.0, 1.0), hohmannDv1(1.0, 1.0, 4.0));
    const auto c = conicFromState(burned, 1.0);
    assert(c.has_value());
    assert(c->bound);
    assert(near(c->rPeri, 1.0, 1e-9));
    assert(near(c->rApo, 4.0, 1e-9));
    assert(near(c->e, 0.6, 1e-9));  // (4 - 1) / (4 + 1)
    assert(near(c->period, 2.0 * hohmannTime(1.0, 1.0, 4.0), 1e-9));
    const Vec2 peri = conicPoint(*c, 0.0);
    const Vec2 apo = conicPoint(*c, kPi);
    assert(near(peri.x, 1.0, 1e-9) && near(peri.y, 0.0, 1e-9));
    assert(near(apo.x, -4.0, 1e-9) && near(apo.y, 0.0, 1e-9));

    // Half an orbit later (the exact propagation) the ship is at (-4, 0) with the far point speed 0.316228.
    const State far = propagate(burned, 1.0, hohmannTime(1.0, 1.0, 4.0));
    assert(near(far.pos.x, -4.0, 1e-8) && near(far.pos.y, 0.0, 1e-8));
    assert(near(length(far.vel), 0.31622776601683794, 1e-9));
    // The second burn adds exactly hohmannDv2 and the orbit is circular at 4.
    const State circ = progradeBurn(far, hohmannDv2(1.0, 1.0, 4.0));
    const auto c2 = conicFromState(circ, 1.0);
    assert(c2.has_value() && c2->e < 1e-8 && near(c2->rPeri, 4.0, 1e-8));

    // The curve of a start orbit with a different direction of travel and the drawing points stay on the conic.
    const auto pts = conicPoints(*c, 40, 100.0);
    assert(pts.size() == 41);
    for (const Vec2& p : pts) {
        const double r = length(p);
        assert(r >= 1.0 - 1e-9 && r <= 4.0 + 1e-9);
    }
}

void apsisTimes() {
    const double tH = hohmannTime(1.0, 1.0, 4.0);
    State s = progradeBurn(circularState(1.0, 0.0, 1.0), hohmannDv1(1.0, 1.0, 4.0));
    // At the near point the far point is half a period away.
    assert(near(*timeToApsis(s, 1.0, Apsis::Far), tH, 1e-8));
    // A quarter of the way, half the time is left; symmetric about the near point, it is the same after passing the far point.
    const State mid = propagate(s, 1.0, 0.5 * tH);
    assert(near(*timeToApsis(mid, 1.0, Apsis::Far), 0.5 * tH, 1e-8));
    // 1.5 tH after the near point the far point has been passed at tH, the next one is at 3 tH.
    const State past = propagate(s, 1.0, 1.5 * tH);
    assert(near(*timeToApsis(past, 1.0, Apsis::Far), 1.5 * tH, 1e-8));
    // Just before the far point it is just about to arrive.
    const State almost = propagate(s, 1.0, tH - 0.001);
    assert(near(*timeToApsis(almost, 1.0, Apsis::Far), 0.001, 1e-6));
    // A circle has no far point; so does a body that never comes back.
    assert(!timeToApsis(circularState(1.0, 0.7, 1.0), 1.0, Apsis::Far).has_value());
    assert(!timeToApsis(State{{1, 0}, {0, 2}}, 1.0, Apsis::Far).has_value());   // faster than escape speed 1.414
    assert(!timeToApsis(State{{1, 0}, {0, 0}}, 1.0, Apsis::Far).has_value());   // standing still: straight fall, no orbit
    assert(!timeToApsis(State{{1, 0}, {0, 1}}, 0.0, Apsis::Far).has_value());   // no sun
    // The orbit does not have to start on the x axis: a burn at 2 rad gives the same time.
    const State turned = progradeBurn(circularState(1.0, 2.0, 1.0), hohmannDv1(1.0, 1.0, 4.0));
    assert(near(*timeToApsis(turned, 1.0, Apsis::Far), tH, 1e-8));

    // Going down it is the other end that matters. Burn against the motion at 4: that is the far point of the ellipse,
    // the near point (radius 1) is half a period away, and a quarter of a period later the next far point is 1.5 half-periods off.
    const State down = progradeBurn(circularState(4.0, 0.0, 1.0), hohmannDv1(1.0, 4.0, 1.0));
    assert(near(*timeToApsis(down, 1.0, Apsis::Near), tH, 1e-8));
    const State downMid = propagate(down, 1.0, 0.5 * tH);
    assert(near(*timeToApsis(downMid, 1.0, Apsis::Near), 0.5 * tH, 1e-8));
    assert(near(*timeToApsis(downMid, 1.0, Apsis::Far), 1.5 * tH, 1e-8));  // it left the far point a quarter period ago
    // At the near point itself: zero. A hair before it: the hair.
    const State atNear = propagate(down, 1.0, tH);
    assert(near(atNear.pos.x, -1.0, 1e-8));
    assert(*timeToApsis(propagate(down, 1.0, tH - 0.002), 1.0, Apsis::Near) < 0.0021);
    assert(near(*timeToApsis(propagate(down, 1.0, tH - 0.002), 1.0, Apsis::Near), 0.002, 1e-7));
}

/// Everything the autopilot does, from the first frame to the second burn, in steps of 10 ms.
Mission flyAutopilot(Scenario sc, Aim aim, double phase, double dt, double limit) {
    Mission m = startMission(sc, aim, phase);
    for (double t = 0.0; t < limit && m.burns < 2; t += dt) advance(m, dt, true);
    return m;
}

void missionBasics() {
    const Scenario sc{1.0, 1.0, 4.0, 0.1};
    Mission m = startMission(sc, Aim::Prograde, 2.0);
    assert(m.burns == 0 && m.dvSpent == 0.0 && !m.crashed && near(m.time, 0.0));
    assert(near(length(m.ship.pos), 1.0) && near(length(m.ship.vel), 1.0));
    assert(near(m.ship.pos.x, 1.0) && near(m.ship.pos.y, 0.0));
    const State t = targetState(m);
    assert(near(length(t.pos), 4.0) && near(length(t.vel), 0.5));
    assert(near(t.pos.x, 4.0 * std::cos(2.0)) && near(t.pos.y, 4.0 * std::sin(2.0)));
    assert(near(phaseNow(m), 2.0));
    assert(near(plannedDv(m), hohmannDv1(1.0, 1.0, 4.0), 1e-12));
    assert(near(windowError(m), 2.0 - hohmannPhase(1.0, 1.0, 4.0), 1e-12));

    // Time moves the target by n2 * t, and the ship keeps its circle (no autopilot, no burn).
    advance(m, 4.0, false);
    assert(near(m.time, 4.0));
    assert(near(phaseNow(m), wrapPi(0.125 * 4.0 + 2.0 - 4.0), 1e-9));
    assert(near(length(m.ship.pos), 1.0, 1e-9));
    // Zero, negative and NaN steps do nothing.
    const double before = m.time;
    advance(m, 0.0, false);
    advance(m, -1.0, false);
    advance(m, std::nan(""), false);
    assert(near(m.time, before));

    // Splitting the time into small steps gives the same ship as one big one.
    Mission one = startMission(sc, Aim::Prograde, 2.0);
    Mission many = startMission(sc, Aim::Prograde, 2.0);
    advance(one, 3.0, false);
    for (int i = 0; i < 300; ++i) advance(many, 0.01, false);
    assert(near(one.ship.pos.x, many.ship.pos.x, 1e-9) && near(one.ship.pos.y, many.ship.pos.y, 1e-9));
    assert(near(one.time, many.time, 1e-9));
}

void manualBurns() {
    const Scenario sc{1.0, 1.0, 4.0, 0.1};
    Mission m = startMission(sc, Aim::Prograde, 1.5893132370591525);
    assert(burn(m));
    assert(m.burns == 1 && near(m.dvSpent, 0.26491106406735176, 1e-12));
    assert(near(length(m.ship.vel), 1.2649110640673518, 1e-12));
    assert(near(m.lastBurnDv, 0.26491106406735176, 1e-12));
    assert(near(m.lastBurnDir.x, 0.0, 1e-12) && near(m.lastBurnDir.y, 1.0, 1e-12));  // along the direction of travel (up)
    assert(near(m.lastBurnPos.x, 1.0) && near(m.lastBurnPos.y, 0.0));
    assert(near(plannedDv(m), 0.18377223398316206, 1e-12));
    assert(std::isnan(m.missAtBurn2));
    advance(m, hohmannTime(1.0, 1.0, 4.0), false);
    assert(burn(m));
    assert(m.burns == 2 && near(m.dvSpent, 0.44868330, 1e-7));
    // The ship arrives where the target is: the phase was right.
    assert(m.missAtBurn2 < 1e-6);
    assert(rendezvous(m, 0.01));
    assert(near(plannedDv(m), 0.0));
    // A third burn does nothing.
    const State kept = m.ship;
    assert(!burn(m));
    assert(m.burns == 2 && near(m.ship.vel.x, kept.vel.x) && near(m.dvSpent, 0.44868330, 1e-7));
    assert(!rendezvous(m, 0.0));  // a tolerance of zero is never met

    // Too early by a quarter of a radian: the ship reaches the far point, the target is not there.
    Mission early = startMission(sc, Aim::Prograde, 1.5893132370591525 + 0.25);
    burn(early);
    advance(early, hohmannTime(1.0, 1.0, 4.0), false);
    burn(early);
    // The target is 0.25 rad further around than the ship, which is 4 * 0.25 = 1 unit of arc, so the distance is about 1.
    assert(early.missAtBurn2 > 0.9 && early.missAtBurn2 < 1.1);
    assert(!rendezvous(early, 0.5));
    assert(rendezvous(early, 1.2));

    // The second burn at the wrong place leaves an eccentric orbit and that is not a rendezvous either.
    Mission wrong = startMission(sc, Aim::Prograde, 1.5893132370591525);
    burn(wrong);
    advance(wrong, 0.4 * hohmannTime(1.0, 1.0, 4.0), false);
    burn(wrong);
    assert(!rendezvous(wrong, 100.0));
}

void autopilot() {
    const Scenario sc{1.0, 1.0, 4.0, 0.1};
    // The window is 2.5 time units away. The first burn must happen exactly then, even with a coarse step of 0.3.
    Mission m = startMission(sc, Aim::Prograde, phaseForWait(1.0, 1.0, 4.0, 2.5));
    assert(near(autoEventTime(m), 2.5, 1e-9));
    while (m.burns < 1 && m.time < 10.0) advance(m, 0.3, true);
    assert(m.burns == 1);
    assert(near(m.burnTime[0], 2.5, 1e-9));
    assert(near(m.dvSpent, 0.26491106406735176, 1e-12));
    // The rest of that 0.3 step was already flown after the burn, so the far point is T minus that much away.
    assert(near(autoEventTime(m) + (m.time - m.burnTime[0]), hohmannTime(1.0, 1.0, 4.0), 1e-7));
    while (m.burns < 2 && m.time < 30.0) advance(m, 0.3, true);
    assert(m.burns == 2);
    assert(near(m.burnTime[1] - m.burnTime[0], hohmannTime(1.0, 1.0, 4.0), 1e-7));
    assert(m.missAtBurn2 < 1e-6);
    assert(rendezvous(m, 0.001));
    assert(near(m.dvSpent, hohmannTotalDv(1.0, 1.0, 4.0), 1e-9));
    assert(std::isinf(autoEventTime(m)));

    // One huge step covers both burns and still lands.
    Mission big = startMission(sc, Aim::Prograde, phaseForWait(1.0, 1.0, 4.0, 2.5));
    advance(big, 100.0, true);
    assert(big.burns == 2 && rendezvous(big, 0.001));
    assert(near(big.time, 100.0));

    // Window right now: the burn happens at time 0.
    Mission now = startMission(sc, Aim::Prograde, hohmannPhase(1.0, 1.0, 4.0));
    advance(now, 0.01, true);
    assert(now.burns == 1 && near(now.burnTime[0], 0.0, 1e-9));

    // Without the autopilot nothing happens by itself.
    Mission idle = startMission(sc, Aim::Prograde, phaseForWait(1.0, 1.0, 4.0, 2.5));
    advance(idle, 50.0, false);
    assert(idle.burns == 0);

    // Different ratios, different start phases, down and up, fine and coarse steps: always a rendezvous.
    const double ratios[] = {1.2, 1.524, 2.0, 4.0, 9.0, 0.5, 0.25};
    const double phases[] = {0.0, 1.0, 2.5, -2.0, 3.1};
    for (const double ratio : ratios) {
        for (const double phase : phases) {
            for (const double dt : {0.016, 0.25}) {
                const Scenario s{1.0, 1.0, ratio, 0.01};
                const Mission r = flyAutopilot(s, Aim::Prograde, phase, dt, 400.0);
                assert(r.burns == 2);
                assert(!r.crashed);
                assert(rendezvous(r, 1e-4 * ratio));
                assert(near(r.dvSpent, hohmannTotalDv(1.0, 1.0, ratio), 1e-9));
            }
        }
    }
}

void naiveAim() {
    const Scenario sc{1.0, 1.0, 4.0, 0.1};
    // Target a quarter turn ahead, at (0, 4). Point at it from (1, 0): direction (-1, 4) / sqrt(17).
    Mission m = startMission(sc, Aim::AtTarget, kPi / 2.0);
    assert(burn(m));
    assert(near(m.ship.vel.x, -0.06425037050261523, 1e-12));
    assert(near(m.ship.vel.y, 1.257001482010461, 1e-12));
    assert(near(m.lastBurnDir.x, -1.0 / std::sqrt(17.0), 1e-12) && near(m.lastBurnDir.y, 4.0 / std::sqrt(17.0), 1e-12));  // toward the target
    // Same dv as the real first burn, but the energy is lower: the orbit is smaller and never gets to 4.
    // From v^2 = 1.58418: E = -0.20791, a = 2.40489, L = 1.257001, e = 0.585648, apoapsis = a (1 + e) = 3.81331.
    const auto c = conicFromState(m.ship, 1.0);
    assert(c.has_value() && c->bound);
    assert(near(c->e, 0.5856481847593448, 1e-9));
    assert(near(c->rApo, 3.8133119432789297, 1e-9));
    assert(near(c->rPeri, 0.9964711850733151, 1e-9));
    assert(c->rApo < 3.9);

    // Whatever happens next, the ship is not at the target when it comes to the far point.
    advance(m, *timeToApsis(m.ship, 1.0, Apsis::Far), false);
    assert(burn(m));
    // It sits at radius 3.8133 and the target at 4, so they are at least 0.1867 apart whatever the angle (triangle
    // inequality). The run itself gives 0.92: the ship comes up short and in the wrong place.
    const double gap = length(m.ship.pos - targetState(m).pos);
    assert(gap >= 4.0 - 3.8133119432789297 - 1e-9);
    assert(near(gap, 0.919, 0.005));
    assert(!rendezvous(m, 0.5));

    // Naive autopilot: fires after the delay and then does nothing more.
    Mission a = startMission(sc, Aim::AtTarget, 2.0);
    assert(near(autoEventTime(a), kAutoDelay, 1e-12));
    advance(a, 0.5, true);
    assert(a.burns == 0);
    advance(a, 0.2, true);
    assert(a.burns == 1 && near(a.burnTime[0], kAutoDelay, 1e-9));
    assert(std::isinf(autoEventTime(a)));
    advance(a, 100.0, true);
    assert(a.burns == 1);

    // The worst case: a target on the other side, the burn points back toward the sun and the orbit shrinks below the start.
    Mission back = startMission(sc, Aim::AtTarget, kPi);
    burn(back);  // direction (-5, 0): all of dv1 pushes inward
    const auto cb = conicFromState(back.ship, 1.0);
    assert(cb.has_value());
    // A radial kick on a circle (L stays 1, p = 1) gives e = dv1 exactly: the orbit is now between 1 / (1 + e) = 0.7906
    // and 1 / (1 - e) = 1.3604, nowhere near the target at 4.
    assert(near(cb->e, 0.26491106406735176, 1e-9));
    assert(near(cb->rPeri, 0.7905694150420949, 1e-9));
    assert(near(cb->rApo, 1.3603796100280632, 1e-9));
}

void crashing() {
    const Scenario sc{1.0, 1.0, 4.0, 0.5};
    Mission m = startMission(sc, Aim::Prograde, 2.0);
    m.ship = State{{1, 0}, {0, 0.2}};  // almost no sideways speed: falls to a periapsis of 0.02, deep inside the disc
    for (int i = 0; i < 300 && !m.crashed; ++i) advance(m, 0.01, false);
    assert(m.crashed);
    // From now on the ship stays where it died, and the clock and the target go on.
    const State dead = m.ship;
    const double t = m.time;
    advance(m, 1.0, false);
    assert(near(m.ship.pos.x, dead.pos.x) && near(m.ship.pos.y, dead.pos.y));
    assert(near(m.time, t + 1.0));
    assert(!burn(m));
    assert(std::isinf(autoEventTime(m)));

    // The same fall with a planet of radius 0.01 (smaller than the periapsis 0.02) is survived.
    Mission small = startMission(Scenario{1.0, 1.0, 4.0, 0.01}, Aim::Prograde, 2.0);
    small.ship = State{{1, 0}, {0, 0.2}};
    for (int i = 0; i < 2500; ++i) advance(small, 0.001, false);  // 2.5 s: past the periapsis at 1.15 s
    assert(!small.crashed);

    // A planet radius of 0 turns the check off.
    Mission off = startMission(Scenario{1.0, 1.0, 4.0, 0.0}, Aim::Prograde, 2.0);
    off.ship = State{{1, 0}, {0, 0.2}};
    for (int i = 0; i < 300; ++i) advance(off, 0.01, false);
    assert(!off.crashed);

    // The real transfer never touches a planet of radius 0.5.
    const Mission ok = flyAutopilot(sc, Aim::Prograde, 2.0, 0.01, 100.0);
    assert(!ok.crashed && ok.burns == 2);
}

void readiness() {
    const Scenario sc{1.0, 1.0, 4.0, 0.1};
    const double tol = 3.0 * kPi / 180.0;
    Mission m = startMission(sc, Aim::Prograde, hohmannPhase(1.0, 1.0, 4.0));
    assert(burnReady(m, tol, 0.2));
    Mission off = startMission(sc, Aim::Prograde, hohmannPhase(1.0, 1.0, 4.0) + 0.1);  // 5.7 degrees off
    assert(!burnReady(off, tol, 0.2));
    Mission edge = startMission(sc, Aim::Prograde, hohmannPhase(1.0, 1.0, 4.0) - 0.04);
    assert(burnReady(edge, tol, 0.2));
    // At the near point the far point is half an orbit away: not yet. At the far point: yes.
    burn(m);
    assert(!burnReady(m, tol, 0.2));
    advance(m, hohmannTime(1.0, 1.0, 4.0) - 0.1, false);
    assert(burnReady(m, tol, 0.2));
    advance(m, 0.1 + 0.15, false);  // a bit past it
    assert(burnReady(m, tol, 0.2));
    advance(m, 1.0, false);  // well past
    assert(!burnReady(m, tol, 0.2));
    // After both burns there is nothing to be ready for; the naive aim has no window either.
    burn(m);
    assert(!burnReady(m, tol, 0.2));
    Mission naive = startMission(sc, Aim::AtTarget, hohmannPhase(1.0, 1.0, 4.0));
    assert(!burnReady(naive, tol, 0.2));
}

}  // namespace

int main() {
    angleHelpers();
    vectorHelpers();
    closedForms();
    phaseAngle();
    waiting();
    burnsAndGeometry();
    ellipseOfTheTransfer();
    apsisTimes();
    missionBasics();
    manualBurns();
    autopilot();
    naiveAim();
    crashing();
    readiness();
    std::puts("hohmann-transfer logic: all tests passed");
    return 0;
}
