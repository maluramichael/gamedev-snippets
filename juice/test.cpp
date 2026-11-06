// Plain-assert tests for the juice logic. Every expected value is worked out by hand in the comment above it.
#include <cmath>
#include <cstdio>
#include <vector>

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

using namespace jc;

// ---- hit stop clock ----

void stopHoldsGameTime() {
    // 0.0625 s freeze (exact in binary). Real 0.03125 s pass: nothing for the game, 0.03125 s of freeze left.
    Clock c;
    c.requestStop(0.0625);
    CHECK(c.stopped());
    CHECK(near(c.advance(0.03125), 0.0));
    CHECK(near(c.stopRemaining, 0.03125));
    // Another 0.0625 real: 0.03125 finish the freeze, the other 0.03125 are game time.
    CHECK(near(c.advance(0.0625), 0.03125));
    CHECK(!c.stopped());
    // Without a freeze game time equals real time.
    CHECK(near(c.advance(0.016), 0.016));
}

void stopEdges() {
    Clock c;
    CHECK(near(c.advance(0.0), 0.0));
    CHECK(near(c.advance(-1.0), 0.0));  // negative time never runs the game backwards
    c.requestStop(0.0);                  // zero and negative requests do nothing
    c.requestStop(-0.5);
    CHECK(!c.stopped());
    c.requestStop(0.5);                  // capped, a bug must not freeze the game for good
    CHECK(near(c.stopRemaining, kMaxStop));
    // Exactly as much real time as freeze left: freeze ends, no game time.
    Clock d;
    d.requestStop(0.125);
    CHECK(near(d.advance(0.125), 0.0));
    CHECK(!d.stopped());
}

void stopsDoNotStack() {
    // Two hits in a row: the longer remaining freeze wins, they do not add up to 0.18.
    Clock c;
    c.requestStop(0.125);
    c.advance(0.0625);                   // 0.0625 left
    c.requestStop(0.0625);               // shorter than what is left: unchanged
    CHECK(near(c.stopRemaining, 0.0625));
    c.requestStop(0.125);                // longer: replaces
    CHECK(near(c.stopRemaining, 0.125));
}

// ---- input buffer ----

void bufferKeepsAPress() {
    InputBuffer b;
    CHECK(!b.take(1.0, 0.25).has_value());  // nothing pressed
    b.press(1.0, kHeavyKind);
    auto k = b.take(1.125, 0.25);           // 0.125 s later, inside the window
    CHECK(k.has_value() && *k == kHeavyKind);
    CHECK(!b.take(1.125, 0.25).has_value());  // taken once
}

void bufferExpires() {
    InputBuffer b;
    b.press(1.0, kLightKind);
    CHECK(!b.take(1.5, 0.25).has_value());  // 0.5 s is too old
    CHECK(!b.take(1.0, 0.25).has_value());  // and it is gone, not just hidden
    b.press(1.0, kLightKind);
    CHECK(b.take(1.25, 0.25).has_value());  // exactly at the window edge still counts
}

void bufferKeepsTheNewestPress() {
    InputBuffer b;
    b.press(1.0, kLightKind);
    b.press(1.125, kHeavyKind);
    auto k = b.take(1.25, 0.25);
    CHECK(k.has_value() && *k == kHeavyKind);
}

void inputPolicy() {
    CHECK(acceptsInput(false, InputPolicy::Dropped));
    CHECK(!acceptsInput(true, InputPolicy::Dropped));  // the naive freeze eats the click
    CHECK(acceptsInput(true, InputPolicy::Buffered));  // hit stop only pauses the game, not the input
    CHECK(acceptsInput(false, InputPolicy::Buffered));
}

// ---- attack timing ----

void attackHitsOnce() {
    // windup 0.125, recovery 0.25 (exact in binary).
    Attack a;
    CHECK(a.ready());
    CHECK(!a.advance(1.0));  // idle never hits
    a.start({0.125, 0.25});
    CHECK(!a.ready());
    CHECK(a.phase == Phase::Windup);
    CHECK(!a.advance(0.0625));
    CHECK(a.advance(0.0625 + 0.03125));  // crosses 0.125: the hit, 0.03125 into the recovery
    CHECK(a.phase == Phase::Recovery);
    CHECK(near(a.t, 0.03125));
    CHECK(!a.advance(0.125));
    CHECK(!a.ready());
    CHECK(!a.advance(0.125));  // 0.03125 + 0.25 = 0.28125 >= 0.25: ready again
    CHECK(a.ready());
    CHECK(!a.advance(1.0));    // and no second hit
}

void attackBigStep() {
    // One huge step goes through windup and recovery, still exactly one hit.
    Attack a;
    a.start({0.125, 0.25});
    CHECK(a.advance(10.0));
    CHECK(a.ready());
}

void attackEdges() {
    Attack a;
    a.start({0.125, 0.25});
    CHECK(!a.advance(0.0));    // no game time passes during a freeze
    CHECK(!a.advance(-1.0));
    CHECK(a.phase == Phase::Windup && near(a.t, 0.0));
    CHECK(!a.start({0.5, 0.5}));  // a running attack cannot be restarted
    CHECK(near(a.swing.windup, 0.125));
    Attack z;
    z.start({0.0, 0.0});          // zero windup: hits on the very next step
    CHECK(z.advance(0.001));
    CHECK(z.ready());
}

void flashRateStaysBelowThreeHertz() {
    // One flash per hit, and a hit cannot come faster than windup + stop + recovery (also with the doubled overdo stop).
    for (const Weight& w : {kLight, kHeavy}) {
        const double cycle = w.windup + w.stop + w.recovery;
        CHECK(1.0 / cycle < 3.0);
        CHECK(1.0 / (w.windup + std::min(w.stop * kOverdoStopFactor, kMaxStop) + w.recovery) < 3.0);
    }
    CHECK(kHeavy.stop > kLight.stop && kHeavy.trauma > kLight.trauma && kHeavy.damage > kLight.damage);
}

// ---- trauma and shake ----

void traumaAddsAndClamps() {
    CHECK(near(addTrauma(0.25, 0.5), 0.75));
    CHECK(near(addTrauma(0.75, 0.5), 1.0));   // never above 1
    CHECK(near(addTrauma(0.5, -2.0), 0.0));   // never below 0
}

void traumaDecays() {
    // 1.0 at 2 per second: after 0.25 s it is 0.5, after 0.5 s it is 0 and stays there.
    CHECK(near(decayTrauma(1.0, 0.25, 2.0), 0.5));
    CHECK(near(decayTrauma(1.0, 0.5, 2.0), 0.0));
    CHECK(near(decayTrauma(0.25, 10.0, 2.0), 0.0));
    CHECK(near(decayTrauma(0.5, 0.0, 2.0), 0.5));   // no time, no change
    CHECK(near(decayTrauma(0.5, -1.0, 2.0), 0.5));  // negative time must not add trauma
}

void shakeStrengthIsSquared() {
    CHECK(near(shakeStrength(0.0), 0.0));
    CHECK(near(shakeStrength(0.5), 0.25));  // half the trauma is a quarter of the shake
    CHECK(near(shakeStrength(1.0), 1.0));
    CHECK(near(shakeStrength(2.0), 1.0));   // out of range input is clamped
    CHECK(near(shakeStrength(-1.0), 0.0));
}

void noiseIsSmoothAndBounded() {
    for (int i = 0; i < 400; ++i) {
        const double x = 0.37 * i;
        const double n = noise1(7, x);
        CHECK(n >= -1.0 && n <= 1.0);
        CHECK(std::fabs(noise1(7, x + 0.001) - n) < 0.01);  // no jumps between neighbours
    }
    CHECK(near(noise1(3, 4.25), noise1(3, 4.25)));    // deterministic
    CHECK(!near(noise1(3, 4.0), noise1(4, 4.0)));     // seeds give different curves
    // A negative time is still defined and bounded.
    CHECK(std::fabs(noise1(1, -12.5)) <= 1.0);
}

void shakeOffsetScalesWithTrauma() {
    const Vec2 none = shakeOffset(0.0, 3.3, 24.0);
    CHECK(near(none.x, 0.0) && near(none.y, 0.0));  // no trauma, no shake, exactly
    // Trauma 0.5 -> strength 0.25 -> at most 6 px of 24.
    double biggest = 0.0;
    for (int i = 0; i < 500; ++i) {
        const Vec2 o = shakeOffset(0.5, 0.01 * i, 24.0);
        biggest = std::max({biggest, std::fabs(o.x), std::fabs(o.y)});
        CHECK(std::fabs(o.x) <= 6.0 + 1e-9 && std::fabs(o.y) <= 6.0 + 1e-9);
    }
    CHECK(biggest > 1.0);  // and it actually moves
    // Full trauma reaches further than a quarter of it.
    double big = 0.0;
    for (int i = 0; i < 500; ++i) big = std::max(big, std::fabs(shakeOffset(1.0, 0.01 * i, 24.0).x));
    CHECK(big > 6.0 && big <= 24.0 + 1e-9);
}

// ---- flash, squash, kick ----

void flashFades() {
    CHECK(near(flashAmount(0.0, 0.125), 1.0));
    CHECK(near(flashAmount(0.0625, 0.125), 0.25));  // (1 - 0.5)^2
    CHECK(near(flashAmount(0.125, 0.125), 0.0));
    CHECK(near(flashAmount(1.0, 0.125), 0.0));
    CHECK(near(flashAmount(-0.1, 0.125), 0.0));     // before the hit nothing
    CHECK(near(flashAmount(0.0, 0.0), 0.0));        // no duration, no division by zero
}

void squashShape() {
    // At the hit the dummy is as squeezed as asked: strength 0.25 -> width 0.75, height 1/0.75.
    const Scale s0 = squash(0.0, 0.25);
    CHECK(near(s0.x, 0.75) && near(s0.y, 1.0 / 0.75));
    CHECK(near(s0.x * s0.y, 1.0));  // area stays the same
    // Half a period later it overshoots to the other side (wider than 1), but less.
    const Scale mid = squash(kSquashHalfPeriod, 0.25);
    CHECK(mid.x > 1.0 && mid.x < 1.25);
    // After a second it is at rest, and before the hit it is untouched.
    CHECK(near(squash(1.0, 0.25).x, 1.0, 1e-4));
    CHECK(near(squash(-0.2, 0.25).x, 1.0));
    CHECK(near(squash(0.0, 0.0).x, 1.0));
    // Absurd strength is clamped, so the dummy can never get zero or negative width.
    CHECK(squash(0.0, 5.0).x > 0.3);
}

void kickGlidesBack() {
    CHECK(near(kick(0.0, 20.0), 20.0));
    CHECK(kick(0.1, 20.0) < kick(0.05, 20.0));
    CHECK(near(kick(2.0, 20.0), 0.0, 1e-6));
    CHECK(near(kick(-1.0, 20.0), 0.0));
}

// ---- pose ----

void poseKeyFrames() {
    CHECK(near(pose(Phase::Idle, 0.5).angle, kRestAngle) && near(pose(Phase::Idle, 0.5).dx, 0.0));
    CHECK(near(pose(Phase::Windup, 0.0).angle, kRestAngle));
    CHECK(near(pose(Phase::Windup, 1.0).angle, kWindupAngle) && near(pose(Phase::Windup, 1.0).dx, -kPullBack));
    // The strike pose appears in the very first frame of the recovery: that is the frame the hit stop holds.
    CHECK(near(pose(Phase::Recovery, 0.0).angle, kStrikeAngle) && near(pose(Phase::Recovery, 0.0).dx, kLunge));
    CHECK(near(pose(Phase::Recovery, 1.0).angle, kRestAngle) && near(pose(Phase::Recovery, 1.0).dx, 0.0));
    CHECK(near(pose(Phase::Recovery, 7.0).angle, kRestAngle));  // progress is clamped
    CHECK(near(pose(Phase::Windup, -3.0).angle, kRestAngle));
}

// ---- particles ----

void rngIsDeterministic() {
    Rng a{12345};
    Rng b{12345};
    for (int i = 0; i < 50; ++i) {
        const double x = a.next();
        CHECK(near(x, b.next()));
        CHECK(x >= 0.0 && x < 1.0);
    }
    Rng zero{0};  // a zero seed would stay zero forever with xorshift
    CHECK(zero.next() != zero.next() || zero.next() != zero.next());
}

void burstCountAndCone() {
    Rng rng{99};
    std::vector<Particle> ps;
    burst(ps, rng, {10.0, 20.0}, {0.0, -2.0}, 40, 100.0, 0.5, 0.5);
    CHECK(ps.size() == 40);
    for (const Particle& p : ps) {
        CHECK(near(p.pos.x, 10.0) && near(p.pos.y, 20.0));
        const double speed = std::hypot(p.vel.x, p.vel.y);
        CHECK(speed >= 50.0 - 1e-9 && speed <= 100.0 + 1e-9);        // half to full speed
        CHECK(-p.vel.y / speed >= std::cos(0.5) - 1e-9);              // inside the cone around "up" (y points down)
        CHECK(p.life > 0.0 && p.life <= 0.5);
    }
}

void burstEdges() {
    Rng rng{1};
    std::vector<Particle> ps;
    burst(ps, rng, {0, 0}, {0.0, 0.0}, 3, 10.0, 0.0, 1.0);  // zero direction falls back to +x, zero spread = straight
    CHECK(ps.size() == 3);
    for (const Particle& p : ps) CHECK(p.vel.x > 0.0 && near(p.vel.y, 0.0, 1e-9));
    burst(ps, rng, {0, 0}, {1, 0}, 1000, 10.0, 1.0, 1.0);   // never more than the cap
    CHECK(ps.size() == kMaxParticles);
    std::vector<Particle> none;
    burst(none, rng, {0, 0}, {1, 0}, -4, 10.0, 1.0, 1.0);
    CHECK(none.empty());
}

void particlesStepAndDie() {
    std::vector<Particle> ps{{{0.0, 0.0}, {10.0, 0.0}, 1.0, 1.0}, {{5.0, 5.0}, {0.0, 0.0}, 0.05, 1.0}};
    // dt 0.1, gravity 100: vy = 0 + 100*0.1 = 10, y = 10*0.1 = 1, x = 10*0.1 = 1. The second one is dead after 0.1 s.
    stepParticles(ps, 0.1, 100.0);
    CHECK(ps.size() == 1);
    CHECK(near(ps[0].vel.y, 10.0) && near(ps[0].pos.x, 1.0) && near(ps[0].pos.y, 1.0));
    CHECK(near(ps[0].life, 0.9));
    stepParticles(ps, 0.0, 100.0);  // no time, no change
    CHECK(ps.size() == 1 && near(ps[0].pos.y, 1.0));
    stepParticles(ps, 5.0, 100.0);  // everything old goes away
    CHECK(ps.empty());
}

// ---- damage and reaction switches ----

void damageClamps() {
    CHECK(takeDamage(100, 8) == 92);
    CHECK(takeDamage(5, 8) == 0);
    CHECK(takeDamage(5, -3) == 5);  // negative damage does not heal
    CHECK(takeDamage(0, 8) == 0);
}

void reactionFollowsSwitches() {
    Effects off;
    off.stop = off.shake = off.flash = off.squash = off.sparks = off.impact = off.number = false;
    const Reaction none = resolveHit(kHeavy, false, off);
    CHECK(near(none.stop, 0.0) && near(none.trauma, 0.0) && !none.flash && near(none.squash, 0.0) && near(none.kick, 0.0));
    CHECK(none.sparks == 0 && !none.impact && !none.number);
    CHECK(effectCount(off) == 0);

    const Effects on;  // everything on by default
    CHECK(effectCount(on) == 7);
    const Reaction light = resolveHit(kLight, false, on);
    CHECK(near(light.stop, kLight.stop) && near(light.trauma, kLight.trauma) && light.flash);
    CHECK(near(light.squash, kLight.squash) && near(light.kick, kLight.kick) && light.sparks == kLight.sparks);
    CHECK(light.impact && light.number);

    // One switch changes exactly one reaction.
    Effects noStop = on;
    noStop.stop = false;
    const Reaction r = resolveHit(kLight, false, noStop);
    CHECK(near(r.stop, 0.0) && near(r.trauma, kLight.trauma) && r.flash);
}

void overdoShakesEverything() {
    const Effects on;
    const Reaction fine = resolveHit(kHeavy, false, on);
    const Reaction over = resolveHit(kHeavy, true, on);
    CHECK(over.trauma > fine.trauma && near(over.trauma, 1.0));
    CHECK(near(over.stop, std::min(kHeavy.stop * kOverdoStopFactor, kMaxStop)));
    // The routine swing only shakes in overdo mode and only with the shake switch on.
    CHECK(near(routineTrauma(false, on), 0.0));
    CHECK(near(routineTrauma(true, on), kRoutineTrauma));
    Effects noShake = on;
    noShake.shake = false;
    CHECK(near(routineTrauma(true, noShake), 0.0));
    CHECK(near(resolveHit(kHeavy, true, noShake).trauma, 0.0));
}

void reducedMotionTurnsMotionOff() {
    const Effects on;
    const Effects calm = withReducedMotion(on, true);
    CHECK(!calm.shake && !calm.flash && !calm.squash && !calm.sparks);
    CHECK(calm.stop && calm.impact && calm.number);  // a pause and a still marker are no motion
    const Effects same = withReducedMotion(on, false);
    CHECK(effectCount(same) == 7);
    // The reduction never switches anything on.
    Effects off;
    off.stop = off.shake = off.flash = off.squash = off.sparks = off.impact = off.number = false;
    CHECK(effectCount(withReducedMotion(off, true)) == 0);
}

void historyKeepsTwoSeconds() {
    std::vector<Sample> h;
    for (int i = 0; i <= 10; ++i) h.push_back({0.5 * i, false});  // 0 .. 5 s
    trimHistory(h, 5.0, 2.0);
    CHECK(!h.empty());
    CHECK(near(h.front().t, 3.0));   // 5 - 2 = 3 is kept, older goes
    CHECK(near(h.back().t, 5.0));
    std::vector<Sample> empty;
    trimHistory(empty, 5.0, 2.0);
    CHECK(empty.empty());
    trimHistory(h, 5.0, -1.0);       // a negative span keeps nothing newer than "now"
    CHECK(h.size() <= 1);
}

}  // namespace

int main() {
    stopHoldsGameTime();
    stopEdges();
    stopsDoNotStack();
    bufferKeepsAPress();
    bufferExpires();
    bufferKeepsTheNewestPress();
    inputPolicy();
    attackHitsOnce();
    attackBigStep();
    attackEdges();
    flashRateStaysBelowThreeHertz();
    traumaAddsAndClamps();
    traumaDecays();
    shakeStrengthIsSquared();
    noiseIsSmoothAndBounded();
    shakeOffsetScalesWithTrauma();
    flashFades();
    squashShape();
    kickGlidesBack();
    poseKeyFrames();
    rngIsDeterministic();
    burstCountAndCone();
    burstEdges();
    particlesStepAndDie();
    damageClamps();
    reactionFollowsSwitches();
    overdoShakesEverything();
    reducedMotionTurnsMotionOff();
    historyKeepsTwoSeconds();
    if (failures != 0) {
        std::printf("%d check(s) failed\n", failures);
        return 1;
    }
    std::printf("all checks passed\n");
    return 0;
}
