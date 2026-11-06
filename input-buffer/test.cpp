// Tests for the coyote time and input buffer logic. Plain asserts on hand-computed cases, no framework, no SDL.
//
// Time is counted in ticks of 1/120 s. The stepping loop is: the world says whether the player is grounded, the
// controller decides whether a jump fires, the world moves. "Tick k" below is the k-th call of that loop, from 0.
// y points down, y = 0 is the top line of the stones.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "logic.hpp"

using namespace ib;

namespace {

bool near(double a, double b, double eps = 1e-6) { return std::fabs(a - b) < eps; }

/// One controller plus its parameters, fed by hand.
struct Rig {
    Controller c;
    Params p;
    Outcome tick(bool grounded, bool press) { return c.step(grounded, press, p); }
};

/// Runs `n` ticks without any input and returns the last outcome.
Outcome idle(Rig& r, bool grounded, int n) {
    Outcome o;
    for (int i = 0; i < n; ++i) o = r.tick(grounded, false);
    return o;
}

void conversions() {
    assert(ticksFromMs(0) == 0);
    assert(ticksFromMs(50) == 6);
    assert(ticksFromMs(100) == 12);
    assert(ticksFromMs(150) == 18);
    assert(ticksFromMs(200) == 24);
    assert(msFromTicks(12) == 100);
    assert(msFromTicks(1) == 8);   // 8.33 ms
    assert(msFromTicks(5) == 42);  // 41.67 ms
    assert(msFromTicks(0) == 0);
}

void pressOnTheGroundJumpsAtOnce() {
    Rig r;
    const Outcome o = r.tick(true, true);
    assert(o.jump == Jump::Ground);
    assert(o.pressAge == 0);
    assert(o.canJump);
    assert(!o.lost);
}

void noGroundNoJump() {
    // The ground check of the buffer: a press in the air must never turn into a jump on its own, however often it is
    // repeated, as long as the player never touches the ground and never was grounded.
    Rig r;
    for (int i = 0; i < 300; ++i) {
        const Outcome o = r.tick(false, true);
        assert(o.jump == Jump::None);
        assert(!o.canJump);
    }
}

void coyoteWindowBoundary() {
    Rig r;  // coyote 12, buffer 12
    r.tick(true, false);                   // the last grounded tick
    idle(r, false, 11);                    // air ticks 1..11
    const Outcome o = r.tick(false, true); // air tick 12: exactly 12 ticks after the ground
    assert(o.jump == Jump::Coyote);
    assert(o.sinceGround == 12);
    assert(o.pressAge == 0);

    Rig late;
    late.tick(true, false);
    idle(late, false, 12);                    // air ticks 1..12
    const Outcome l = late.tick(false, true);  // air tick 13: one too many
    assert(l.jump == Jump::None);
    assert(!l.canJump);
    assert(l.sinceGround == 13);
}

void coyoteZeroMeansGroundOnly() {
    Rig r;
    r.p.coyote = 0;
    r.tick(true, false);
    const Outcome air = r.tick(false, true);  // first tick after leaving the ground
    assert(air.jump == Jump::None);
    assert(!air.canJump);
    Rig g;
    g.p.coyote = 0;
    assert(g.tick(true, true).jump == Jump::Ground);
}

void bufferWindowBoundary() {
    Rig r;  // buffer 12
    r.tick(false, true);   // tick 0: press in the air
    idle(r, false, 11);    // ticks 1..11, nothing is lost yet
    const Outcome o = r.tick(true, false);  // tick 12: landed, press is exactly 12 ticks old
    assert(o.jump == Jump::Ground);
    assert(o.pressAge == 12);

    Rig late;
    Outcome o2 = late.tick(false, true);  // tick 0
    assert(!o2.lost);
    for (int i = 1; i < 12; ++i) {
        o2 = late.tick(false, false);
        assert(!o2.lost);
    }
    o2 = late.tick(false, false);  // tick 12: still in the air, the press expires now
    assert(o2.lost);
    assert(o2.jump == Jump::None);
    assert(late.tick(true, false).jump == Jump::None);  // tick 13: landing is one tick too late
}

void bufferZeroLosesAtOnce() {
    Rig r;
    r.p.buffer = 0;
    const Outcome o = r.tick(false, true);
    assert(o.lost);
    assert(o.jump == Jump::None);
    Rig g;
    g.p.buffer = 0;
    assert(g.tick(true, true).jump == Jump::Ground);  // pressed exactly on the ground still works
}

void aPressFiresOnlyOnce() {
    Rig r;
    assert(r.tick(true, true).jump == Jump::Ground);
    for (int i = 0; i < 40; ++i) assert(r.tick(false, false).jump == Jump::None);
    assert(!r.c.pending());
}

void resetOnJumpPreventsTheDoubleJump() {
    Rig on;  // consume = true
    assert(on.tick(true, true).jump == Jump::Ground);
    idle(on, false, 3);
    assert(on.tick(false, true).jump == Jump::None);  // a second tap 4 ticks later: coyote was used up
    for (int i = 0; i < 20; ++i) assert(on.tick(false, false).jump == Jump::None);

    Rig off;  // the bug: the coyote timer keeps running from the old ground contact
    off.p.consume = false;
    assert(off.tick(true, true).jump == Jump::Ground);
    idle(off, false, 3);
    const Outcome o = off.tick(false, true);
    assert(o.jump == Jump::Air);
    assert(o.jumpsBefore == 1);
    assert(o.sinceGround == 4);
    // The window still closes: a tap 14 ticks after the ground does nothing even with the bug.
    Rig closed;
    closed.p.consume = false;
    closed.tick(true, true);
    idle(closed, false, 12);
    assert(closed.tick(false, true).jump == Jump::None);
}

void landingStartsOver() {
    Rig r;
    r.p.consume = false;
    r.tick(true, true);
    idle(r, false, 3);
    assert(r.tick(false, true).jump == Jump::Air);
    r.tick(true, false);  // landed
    idle(r, false, 20);
    assert(r.tick(true, true).jump == Jump::Ground);  // not Air: the jump counter was cleared
}

void aNewPressReplacesTheOldOne() {
    Rig r;
    r.tick(false, true);                       // tick 0
    idle(r, false, 4);                         // ticks 1..4
    const Outcome second = r.tick(false, true);  // tick 5: replaces the first press
    assert(second.replaced);
    idle(r, false, 11);                        // ticks 6..16
    const Outcome o = r.tick(true, false);     // tick 17: 12 ticks after the second press, 17 after the first
    assert(o.jump == Jump::Ground);
    assert(o.pressAge == 12);
}

void resetClearsEverything() {
    Rig r;
    r.tick(false, true);
    r.c.reset();
    assert(!r.c.pending());
    assert(r.tick(true, false).jump == Jump::None);  // the old press is gone
}

void levelLayout() {
    assert(groundAt(0.0) == Ground::Long);
    assert(groundAt(149.9) == Ground::Long);
    assert(groundAt(150.0) == Ground::None);
    assert(groundAt(229.9) == Ground::None);
    assert(groundAt(230.0) == Ground::Short);
    assert(groundAt(309.9) == Ground::Short);
    assert(groundAt(310.0) == Ground::None);
    assert(groundAt(385.9) == Ground::None);
    assert(groundAt(386.0) == Ground::Long);  // the pattern repeats
    assert(groundAt(386.0 + 230.0) == Ground::Short);
    assert(groundAt(-1.0) == Ground::None);   // negative x wraps from the end of the period
    assert(cycleOf(385.9) == 0);
    assert(cycleOf(386.0) == 1);
}

void jumpArc() {
    // Hand-computed: a tick changes vy by g*dt = 15 px/s and y by vy/120. After n ticks of a jump from the ground
    // y = (-560 n + 7.5 n (n + 1)) / 120. The first n with y >= 0 is 74 (y = 185/120), n = 73 gives -365/120.
    World w;
    w.x = 21.0;
    w.step(true);
    int ticks = 1;
    double top = w.y;
    while (!w.onGround && ticks < 200) {
        w.step(false);
        ++ticks;
        top = std::min(top, w.y);
    }
    assert(ticks == 74);
    assert(near(w.y, 0.0));
    assert(near(top, -10175.0 / 120.0, 1e-6));  // apex after 37 ticks: (-20720 + 7.5 * 37 * 38) / 120
    assert(near(w.x, 21.0 + 74.0 * kRunSpeed / kTickRate, 1e-6));
}

void landingPrediction() {
    World w;
    w.x = 21.0;
    assert(ticksToLanding(0.0, -kJumpSpeed) == 74);
    w.step(true);
    for (int i = 1; i < 10; ++i) w.step(false);  // 10 ticks into the jump
    assert(ticksToLanding(w.y, w.vy) == 64);
    for (int i = 0; i < 63; ++i) w.step(false);
    assert(!w.onGround);
    assert(ticksToLanding(w.y, w.vy) == 1);
    w.step(false);
    assert(w.onGround);
    // A rising player that never comes down within the limit reports -1.
    assert(ticksToLanding(0.0, -kJumpSpeed, 10) == -1);
}

void walkingOffTheLedge() {
    World w;
    w.x = 21.0;  // the ledge is at 150, one tick is 5/3 px
    for (int i = 1; i <= 77; ++i) {
        w.step(false);
        assert(w.onGround);  // 21 + 77 * 5/3 = 149.33
    }
    w.step(false);  // 78th step: 151.0, over the gap
    assert(!w.onGround);
    assert(near(w.y, 0.0));  // walking off does not drop the player in the same tick
}

void fallingIntoThePit() {
    // After walking off, y = n (n + 1) / 16 after n more ticks: 87.9 at n = 37, 92.6 at n = 38. The pit is 90 deep.
    World w;
    w.x = 21.0;
    for (int i = 0; i < 78; ++i) w.step(false);
    for (int n = 1; n <= 37; ++n) assert(!w.step(false));
    assert(near(w.y, 37.0 * 38.0 / 16.0, 1e-6));
    assert(w.step(false));  // n = 38
    assert(w.falls == 1);
    assert(w.onGround);
    assert(near(w.x, kSpawnX));
    assert(near(w.y, 0.0));
}

/// Steps a Sim for `ticks` ticks, pressing on the given ticks, and returns whether anything died.
struct Script {
    long a = -1, b = -1, c = -1;
    bool at(long t) const { return t == a || t == b || t == c; }
};

void lateJumpLandsOnTheShortStone() {
    // The player walks off the ledge during tick 77, so tick 77 is the last grounded tick. A press at tick 82 is
    // 5 ticks after it. The jump lands 74 ticks later: the landing step is tick 82 + 73 = 155 at
    // x = 21 + 156 * 5/3 = 281.0, on the short stone [230, 310).
    Sim s;
    s.botOn = false;
    s.world.x = 21.0;
    const Script script{82};
    for (long t = 0; t <= 155; ++t) {
        assert(!s.tickOnce(script.at(t)));
        if (t == 77) assert(s.world.onGround == false);  // walked off during this tick
    }
    assert(s.world.onGround);
    assert(near(s.world.x, 281.0));
    assert(groundAt(s.world.x) == Ground::Short);

    const auto& presses = s.hist.presses();
    assert(presses.size() == 1);
    assert(presses[0].tick == 82);
    assert(presses[0].state == PressRec::State::Fired);
    assert(presses[0].kind == Jump::Coyote);
    assert(presses[0].sinceGround == 5);
    assert(s.hist.stats().jumps == 1);
    assert(s.hist.stats().saved == 1);

    // The recorded window: grounded up to tick 77, open for 12 more ticks, used up by the jump at 82.
    assert(s.hist.at(77)->grounded && s.hist.at(77)->open);
    assert(!s.hist.at(78)->grounded && s.hist.at(78)->open);
    assert(s.hist.at(82)->jump == Jump::Coyote && s.hist.at(82)->open);
    assert(!s.hist.at(83)->open);  // consumed, so the bar ends here
}

void windowOpenRecordWithoutAJump() {
    Sim s;
    s.botOn = false;
    s.world.x = 21.0;
    for (long t = 0; t < 91; ++t) s.tickOnce(false);
    assert(s.hist.at(89)->open);   // 12 ticks after tick 77
    assert(!s.hist.at(90)->open);  // 13 ticks
}

void tooLateFallsAndIsReportedAsLate() {
    // Press at tick 90: 13 ticks after the ground, one outside the window. It waits 12 ticks and is lost at tick 102.
    // The player falls from tick 78 on and dies at n = 38, tick 115.
    Sim s;
    s.botOn = false;
    s.world.x = 21.0;
    const Script script{90};
    for (long t = 0; t < 115; ++t) assert(!s.tickOnce(script.at(t)));
    assert(s.tickOnce(false));  // tick 115
    assert(s.hist.stats().falls == 1);
    assert(s.hist.stats().jumps == 0);
    assert(s.hist.stats().lost == 1);
    const PressRec& r = s.hist.presses()[0];
    assert(r.state == PressRec::State::Lost);
    assert(r.endTick == 102);
    const Miss m = classifyMiss(r);
    assert(m.kind == Lost::Late);
    assert(m.ticks == 13);
}

void bufferedPressSurvivesTheLanding() {
    // Jump at tick 0 from x = 21. It lands at tick 73 (grounded from tick 74) at x = 144.33, still on the long
    // stone. A press at tick 67 is 7 ticks old when the player is grounded again.
    Sim s;
    s.botOn = false;
    s.world.x = 21.0;
    const Script script{0, 67};
    for (long t = 0; t <= 74; ++t) s.tickOnce(script.at(t));
    const auto& presses = s.hist.presses();
    assert(presses.size() == 2);
    assert(presses[1].state == PressRec::State::Fired);
    assert(presses[1].endTick == 74);
    assert(presses[1].age == 7);
    assert(presses[1].kind == Jump::Ground);
    assert(s.hist.stats().jumps == 2);
}

void bufferTooShortLosesTheEarlyPress() {
    Sim s;
    s.botOn = false;
    s.world.x = 21.0;
    s.params.buffer = 6;
    const Script script{0, 67};
    for (long t = 0; t <= 74; ++t) s.tickOnce(script.at(t));
    const PressRec& r = s.hist.presses()[1];
    assert(r.state == PressRec::State::Lost);
    assert(r.endTick == 73);  // 6 ticks after the press, the player is still in the air
    assert(r.nextGround == 74);
    const Miss m = classifyMiss(r);
    assert(m.kind == Lost::Early);
    assert(m.ticks == 7);  // pressed 7 ticks before landing
    assert(s.hist.stats().jumps == 1);
    assert(s.hist.stats().lost == 1);  // counted once the landing showed that it was a miss
}

void doubleTapBug() {
    // Jump at tick 0, tap again at tick 5. With the timer reset the second tap is lost, with the bug it is a jump
    // in mid-air that sends the player much higher.
    int lostOn = -1;
    auto run = [&lostOn](bool consume, double& apex, int& airJumps) {
        Sim s;
        s.botOn = false;
        s.world.x = 21.0;
        s.params.consume = consume;
        const Script script{0, 5};
        apex = 0.0;
        for (long t = 0; t < 80; ++t) {
            s.tickOnce(script.at(t));
            apex = std::min(apex, s.world.y);
        }
        airJumps = s.hist.stats().airJumps;
        if (consume) lostOn = s.hist.stats().lost;
        return classifyMiss(s.hist.presses()[1]);
    };
    double apexOn = 0.0;
    double apexOff = 0.0;
    int airOn = -1;
    int airOff = -1;
    const Miss lostTap = run(true, apexOn, airOn);
    run(false, apexOff, airOff);
    assert(airOn == 0);
    assert(airOff == 1);
    assert(lostOn == 0);  // a tap in the air is not a miss
    assert(apexOn > -86.0 && apexOn < -84.0);  // the normal apex is -84.79
    assert(apexOff < -100.0);                  // second jump from y = -21.5: about -106
    assert(lostTap.kind == Lost::Extra);       // the tap was lost, no landing right after it
}

void sloppyBotIsStarvedWithoutWindows() {
    Sim s;
    s.params.coyote = 0;
    s.params.buffer = 0;
    for (long t = 0; t < 600; ++t) s.tickOnce(false);
    assert(s.hist.stats().jumps == 0);   // every ledge press is at least 2 ticks late
    assert(s.hist.stats().falls >= 4);
    assert(s.hist.stats().lost >= 4);
}

void sloppyBotFirstCycle() {
    // Start at x = 21. Ledge crossed during tick 77. The bot's first late press is 2 ticks after the ground (tick 79),
    // the jump lands during tick 152 at x = 276 on the short stone. Its first early press is 4 ticks before the
    // landing: at tick 149, firing at tick 153 with age 4.
    Sim s;
    for (long t = 0; t < 230; ++t) s.tickOnce(false);
    const auto& p = s.hist.presses();
    assert(p.size() >= 2);
    assert(p[0].tick == 79 && p[0].kind == Jump::Coyote && p[0].sinceGround == 2 && p[0].endTick == 79);
    assert(p[1].tick == 149 && p[1].kind == Jump::Ground && p[1].age == 4 && p[1].endTick == 153);
    assert(s.hist.stats().falls == 0);
}

void windowsOfTwoHundredMsCarryTheBotForever() {
    // The bot is off by at most 19 ticks (158 ms). With both windows at 24 ticks (200 ms) nothing is ever lost,
    // with no windows it never gets anywhere.
    Sim s;
    s.params.coyote = 24;
    s.params.buffer = 24;
    for (long t = 0; t < 6000; ++t) s.tickOnce(false);
    assert(s.hist.stats().falls == 0);
    assert(s.hist.stats().lost == 0);
    assert(s.hist.stats().jumps > 40);
    assert(s.world.x > 5.0 * kPeriod);
}

void windowsOfOneHundredMsLoseSomeInputs() {
    Sim s;  // defaults: 12 and 12 ticks
    for (long t = 0; t < 6000; ++t) s.tickOnce(false);
    assert(s.hist.stats().falls > 0);   // late = 16 and early = 19 are outside
    assert(s.hist.stats().jumps > 20);  // but most of the course works
}

void historyRingAndPruning() {
    Sim s;
    for (long t = 0; t < 5000; ++t) s.tickOnce(false);
    assert(s.hist.at(4999) != nullptr);
    assert(s.hist.at(4999 - 400) != nullptr);
    assert(s.hist.at(0) == nullptr);  // long gone from the ring
    assert(s.hist.at(5000) == nullptr);
    for (const PressRec& r : s.hist.presses()) assert(5000 - r.tick <= History::kKeep + 1);
}

void restartClearsTheRun() {
    Sim s;
    for (long t = 0; t < 400; ++t) s.tickOnce(false);
    s.restart();
    assert(s.tick == 0);
    assert(s.hist.presses().empty());
    assert(s.hist.stats().jumps == 0);
    assert(near(s.world.x, kSpawnX));
    assert(!s.ctrl.pending());
}

}  // namespace

int main() {
    conversions();
    pressOnTheGroundJumpsAtOnce();
    noGroundNoJump();
    coyoteWindowBoundary();
    coyoteZeroMeansGroundOnly();
    bufferWindowBoundary();
    bufferZeroLosesAtOnce();
    aPressFiresOnlyOnce();
    resetOnJumpPreventsTheDoubleJump();
    landingStartsOver();
    aNewPressReplacesTheOldOne();
    resetClearsEverything();
    levelLayout();
    jumpArc();
    landingPrediction();
    walkingOffTheLedge();
    fallingIntoThePit();
    lateJumpLandsOnTheShortStone();
    windowOpenRecordWithoutAJump();
    tooLateFallsAndIsReportedAsLate();
    bufferedPressSurvivesTheLanding();
    bufferTooShortLosesTheEarlyPress();
    doubleTapBug();
    sloppyBotIsStarvedWithoutWindows();
    sloppyBotFirstCycle();
    windowsOfTwoHundredMsCarryTheBotForever();
    windowsOfOneHundredMsLoseSomeInputs();
    historyRingAndPruning();
    restartClearsTheRun();
    std::puts("input-buffer logic: all asserts passed");
    return 0;
}
