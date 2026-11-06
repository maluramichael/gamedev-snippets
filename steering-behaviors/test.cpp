// Tests for the steering behaviors logic. Plain asserts on hand-computed cases, no framework, no SDL.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "logic.hpp"

using namespace sb;

namespace {

bool near(double a, double b, double eps = 1e-9) { return std::fabs(a - b) < eps; }
bool near(Vec2 a, Vec2 b, double eps = 1e-9) { return near(a.x, b.x, eps) && near(a.y, b.y, eps); }

// A world without walls, so wrapping cannot get in the way of the physics tests.
Params openWorld() {
    Params p;
    p.left = -1e9;
    p.top = -1e9;
    p.right = 1e9;
    p.bottom = 1e9;
    return p;
}

void vectorBasics() {
    assert(near(length({3, 4}), 5.0));
    assert(near(normalize({3, 4}), {0.6, 0.8}));
    assert(near(normalize({0, 0}), {0, 0}));  // no division by zero, no NaN
    assert(near(truncate({3, 4}, 10.0), {3, 4}));  // shorter than the limit: untouched
    assert(near(truncate({3, 4}, 2.5), {1.5, 2.0}));  // longer: same direction, length 2.5
    assert(near(truncate({3, 4}, 0.0), {0, 0}));
    assert(near(truncate({0, 0}, 1.0), {0, 0}));
}

void seekPointsAtTheTarget() {
    // Target 3 right, 4 down: unit direction (0.6, 0.8), full speed.
    assert(near(seekDesired({0, 0}, {3, 4}, 100.0), {60, 80}));
    assert(near(seekDesired({10, 10}, {7, 6}, 100.0), {-60, -80}));
    // Standing on the target there is no direction: zero, not NaN.
    assert(near(seekDesired({5, 5}, {5, 5}, 100.0), {0, 0}));
}

void fleeIsTheMirror() {
    // Inside the panic radius: straight away from the target at full speed.
    assert(near(fleeDesired({0, 0}, {3, 4}, {1, 0}, 100.0, 10.0), {-60, -80}));
    // Exactly on the radius counts as safe, just inside it does not.
    assert(near(fleeDesired({0, 0}, {10, 0}, {1, 0}, 100.0, 10.0), {0, 0}));
    assert(near(fleeDesired({0, 0}, {9.9, 0}, {1, 0}, 100.0, 10.0), {-100, 0}));
    assert(near(fleeDesired({0, 0}, {50, 0}, {1, 0}, 100.0, 10.0), {0, 0}));
    // On top of the target: run along the current heading instead of standing still.
    assert(near(fleeDesired({2, 2}, {2, 2}, {0, -1}, 100.0, 10.0), {0, -100}));
}

void arriveRampsDownInsideTheRadius() {
    const double speed = 100.0;
    const double radius = 50.0;
    assert(near(arriveDesired({0, 0}, {100, 0}, speed, radius), {100, 0}));  // far away: full speed
    assert(near(arriveDesired({0, 0}, {50, 0}, speed, radius), {100, 0}));   // exactly on the radius: still full
    assert(near(arriveDesired({0, 0}, {25, 0}, speed, radius), {50, 0}));    // half way in: half speed
    assert(near(arriveDesired({0, 0}, {0, 10}, speed, radius), {0, 20}));    // 10 of 50: one fifth
    assert(near(arriveDesired({3, 3}, {3, 3}, speed, radius), {0, 0}));      // there: stop
    assert(near(arriveDesired({0, 0}, {-12.5, 0}, speed, radius), {-25, 0}));
}

void forceIsLimitedPerSecondAndScaledByDt() {
    Params p = openWorld();
    p.maxSpeed = 1000.0;
    p.maxForce = 50.0;
    // From rest towards (100,0): the missing velocity is 100, but a step of 0.5 s may only add 50 * 0.5 = 25.
    Agent a;
    applySteering(a, {100, 0}, p, 0.5, true);
    assert(near(a.vel, {25, 0}));
    assert(near(a.pos, {12.5, 0}));  // moves with the new velocity: 25 * 0.5
    // Two steps of 0.25 s add the same velocity as one of 0.5 s (12.5 each).
    Agent b;
    applySteering(b, {100, 0}, p, 0.25, true);
    applySteering(b, {100, 0}, p, 0.25, true);
    assert(near(b.vel, {25, 0}));
    // Without the limit the velocity jumps to the wish at once.
    Agent c;
    applySteering(c, {100, 0}, p, 0.5, false);
    assert(near(c.vel, {100, 0}));
    // A wish below the cap is reached exactly, no overshoot.
    Agent d;
    applySteering(d, {10, 0}, p, 0.5, true);
    assert(near(d.vel, {10, 0}));
    // Speed is capped at maxSpeed even when the wish is higher.
    Params q = openWorld();
    q.maxSpeed = 30.0;
    Agent e;
    applySteering(e, {300, 400}, q, 1.0, false);
    assert(near(length(e.vel), 30.0));
    assert(near(e.vel, {18, 24}));
}

void wrapAroundTheField() {
    Params p;
    p.left = 0;
    p.top = 10;
    p.right = 100;
    p.bottom = 60;
    assert(near(wrap({-5, 20}, p), {95, 20}));
    assert(near(wrap({105, 20}, p), {5, 20}));
    assert(near(wrap({50, 5}, p), {50, 55}));
    assert(near(wrap({50, 65}, p), {50, 15}));
    assert(near(wrap({50, 30}, p), {50, 30}));
    assert(near(wrap({0, 10}, p), {0, 10}));  // the left/top edge belongs to the field
    assert(near(wrap({100, 60}, p), {0, 10}));  // the right/bottom edge does not
    // An agent leaving through the right edge comes back in on the left.
    Params w = p;
    w.maxSpeed = 1000.0;
    Agent a;
    a.pos = {98, 30};
    a.vel = {100, 0};
    applySteering(a, {100, 0}, w, 0.1, true);  // moves 10 px
    assert(near(a.pos, {8, 30}));
}

void wanderNoiseIsSmoothAndBounded() {
    double prev = wanderNoise(0.0);
    for (int i = 1; i <= 20000; ++i) {
        const double t = i * 0.01;  // 200 s
        const double n = wanderNoise(t);
        assert(n >= -1.0 && n <= 1.0);
        assert(std::fabs(n - prev) < 0.06);  // no jumps between neighbours 10 ms apart
        prev = n;
    }
    assert(near(wanderNoise(12.34), wanderNoise(12.34)));  // a pure function of time
}

void wanderTargetSitsOnTheCircleAheadOfTheAgent() {
    const Vec2 pos{10, 20};
    const Vec2 heading{1, 0};
    // Angle 0: the point on the circle straight ahead, distance + radius away.
    assert(near(wanderTarget(pos, heading, 0.0, 80.0, 30.0), {120, 20}));
    // A quarter turn: the circle center is (90,20), the point is 30 px to the side of it (y grows downwards in the demo).
    assert(near(wanderTarget(pos, heading, kPi / 2, 80.0, 30.0), {90, 50}, 1e-9));
    // Heading up: ahead is up.
    assert(near(wanderTarget(pos, {0, -1}, 0.0, 80.0, 30.0), {10, -90}));
    // Whatever the angle: the target is on the circle.
    for (int i = 0; i < 16; ++i) {
        const double angle = i * kPi / 8;
        const Vec2 t = wanderTarget(pos, heading, angle, 80.0, 30.0);
        assert(near(distance(t, {90, 20}), 30.0, 1e-9));
    }
}

const Path kSquare{{0, 0}, {100, 0}, {100, 100}, {0, 100}};

void closestPointOnAClosedPath() {
    PathHit h = closestOnPath(kSquare, {50, -20});
    assert(near(h.point, {50, 0}) && h.segment == 0 && near(h.distance, 20.0));
    h = closestOnPath(kSquare, {110, 50});
    assert(near(h.point, {100, 50}) && h.segment == 1 && near(h.distance, 10.0));
    // The closing segment from the last point back to the first is part of the path.
    h = closestOnPath(kSquare, {-7, 40});
    assert(near(h.point, {0, 40}) && h.segment == 3 && near(h.distance, 7.0));
    // Beyond the end of a segment the closest point is its end point, not the extended line.
    h = closestOnPath(kSquare, {150, -50});
    assert(near(h.point, {100, 0}) && near(h.distance, std::sqrt(5000.0)));
    // Right on the path: distance 0.
    h = closestOnPath(kSquare, {30, 0});
    assert(near(h.point, {30, 0}) && near(h.distance, 0.0));
}

void closestPointSurvivesDegenerateSegments() {
    const Path twice{{0, 0}, {0, 0}, {10, 0}};  // a zero length first segment
    const PathHit h = closestOnPath(twice, {4, 3});
    assert(near(h.point, {4, 0}) && near(h.distance, 3.0));
    const Path single{{5, 5}};  // one point is not a path
    assert(!closestOnPath(single, {0, 0}).valid);
    assert(!closestOnPath({}, {0, 0}).valid);
    assert(closestOnPath(twice, {4, 3}).valid);
    assert(near(pathDesired(single, {0, 0}, {1, 0}, PathParams{}, 100.0), {0, 0}));
    assert(near(pathDesired({}, {0, 0}, {1, 0}, PathParams{}, 100.0), {0, 0}));
}

void pointAheadWalksAroundCorners() {
    // 10 px before the corner (100,0), 30 px ahead: 10 to the corner, 20 down the next segment.
    PathHit h = closestOnPath(kSquare, {90, 0});
    assert(near(pointAhead(kSquare, h, 30.0), {100, 20}));
    // Within a segment.
    assert(near(pointAhead(kSquare, h, 5.0), {95, 0}));
    // From the last segment it wraps to the first: (0,90) + 100 -> 90 up to (0,0), 10 more along segment 0.
    h = closestOnPath(kSquare, {0, 90});
    assert(h.segment == 3);
    assert(near(pointAhead(kSquare, h, 100.0), {10, 0}));
    // Further than one lap is fine too: a lap is 400, so 410 is 10 px down from (0,90).
    assert(near(pointAhead(kSquare, h, 410.0), {0, 80}));
    // Distance 0 is the point itself.
    assert(near(pointAhead(kSquare, h, 0.0), {0, 90}));
}

void pathFollowingOnlyActsOutsideThePath() {
    PathParams pp;
    pp.radius = 10.0;
    pp.lookahead = 0.1;
    pp.lead = 20.0;
    // Inside the corridor: predicted (30,3) is 3 px from the path, keep going along the current heading at full speed.
    assert(near(pathDesired(kSquare, {20, 3}, {100, 0}, pp, 100.0), {100, 0}));
    // Outside: predicted (30,30), closest (30,0), target 20 ahead = (50,0), from (20,30) that is direction (30,-30).
    const Vec2 d = pathDesired(kSquare, {20, 30}, {100, 0}, pp, 100.0);
    assert(near(d, {100 * std::sqrt(0.5), -100 * std::sqrt(0.5)}, 1e-9));
    // Standing still inside the corridor must not mean standing still forever: it picks a direction along the path.
    const Vec2 s = pathDesired(kSquare, {20, 3}, {0, 0}, pp, 100.0);
    assert(length(s) > 99.0);
    assert(s.x > 0.0);  // the path runs to the right here
}

// --- whole behaviors, simulated -------------------------------------------------------------------------------

struct Run {
    double minDistance = 1e18;  // closest approach after the warm up
    double finalDistance = 0;
    double finalSpeed = 0;
    double slowest = 1e18;      // lowest speed after the warm up
    double fastest = 0;         // highest speed after the warm up
    double farthest = 0;        // largest distance to the target after the warm up
    Agent agent;
};

Run simulate(Behavior b, double hz, bool scale, bool limit, double seconds, double warmUp, const Params& p, Vec2 target, Agent start) {
    Run r;
    r.agent = start;
    const int steps = static_cast<int>(seconds * hz);
    for (int i = 0; i < steps; ++i) {
        advance(r.agent, b, target, p, kNoPath, hz, scale, limit);
        if (static_cast<double>(i) / hz >= warmUp) {
            r.minDistance = std::min(r.minDistance, distance(r.agent.pos, target));
            r.slowest = std::min(r.slowest, length(r.agent.vel));
            r.fastest = std::max(r.fastest, length(r.agent.vel));
            r.farthest = std::max(r.farthest, distance(r.agent.pos, target));
        }
    }
    r.finalDistance = distance(r.agent.pos, target);
    r.finalSpeed = length(r.agent.vel);
    return r;
}

Agent startAgent() {
    Agent a;
    a.pos = {0, 0};
    a.vel = {0, -150};
    a.heading = {0, -1};
    return a;
}

void seekNeverSettles() {
    const Params p = openWorld();
    const Vec2 target{300, 0};
    for (const double hz : {30.0, 120.0}) {
        const Run r = simulate(Behavior::Seek, hz, true, true, 20.0, 4.0, p, target, startAgent());
        std::printf("seek %3.0f Hz: min dist %.1f farthest %.1f slowest %.1f fastest %.1f\n", hz, r.minDistance, r.farthest, r.slowest, r.fastest);
        // Long after the first pass it still shoots through the target at nearly full speed...
        assert(r.fastest > 0.8 * p.maxSpeed);
        // ...and swings out again each time instead of coming to rest on it: it never stays within 30 px of the target.
        assert(r.farthest > 30.0);
    }
}

void arriveSettles() {
    const Params p = openWorld();
    const Vec2 target{300, 0};
    for (const double hz : {30.0, 120.0}) {
        const Run r = simulate(Behavior::Arrive, hz, true, true, 14.0, 0.0, p, target, startAgent());
        std::printf("arrive %3.0f Hz: final dist %.3f speed %.3f\n", hz, r.finalDistance, r.finalSpeed);
        assert(r.finalDistance < 1.0);
        assert(r.finalSpeed < 1.0);
    }
}

void seekWithoutForceLimitJitters() {
    const Params p = openWorld();
    // No cap: the velocity snaps to the wish, the agent reaches the target and keeps flipping around it at full speed.
    const Run r = simulate(Behavior::Seek, 60.0, true, false, 6.0, 3.0, p, {300, 0}, startAgent());
    assert(r.finalDistance < p.maxSpeed / 60.0 * 2.0);  // never further than a step or two from the target
    assert(r.slowest > 0.99 * p.maxSpeed);  // and still at full speed: it never brakes, it just flips around
}

void dtScalingKeepsTheSpeedAtAnyFrameRate() {
    const Params p = openWorld();
    const Vec2 target{300, 0};
    // Same wish, same two seconds, 30 versus 120 steps per second.
    const Run fast = simulate(Behavior::Seek, 120.0, true, true, 2.0, 99.0, p, target, startAgent());
    const Run slow = simulate(Behavior::Seek, 30.0, true, true, 2.0, 99.0, p, target, startAgent());
    const double gapScaled = distance(fast.agent.pos, slow.agent.pos);
    const Run fastN = simulate(Behavior::Seek, 120.0, false, true, 2.0, 99.0, p, target, startAgent());
    const Run slowN = simulate(Behavior::Seek, 30.0, false, true, 2.0, 99.0, p, target, startAgent());
    const double gapNaive = distance(fastN.agent.pos, slowN.agent.pos);
    std::printf("gap after 2 s: scaled %.1f px, per frame %.1f px\n", gapScaled, gapNaive);
    assert(gapScaled < 15.0);
    assert(gapNaive > 8.0 * gapScaled);
    assert(gapNaive > 100.0);
    // The per-frame version at 60 steps per second is the reference it was tuned for: same as the scaled one.
    const Run sixtyN = simulate(Behavior::Seek, 60.0, false, true, 2.0, 99.0, p, target, startAgent());
    const Run sixtyS = simulate(Behavior::Seek, 60.0, true, true, 2.0, 99.0, p, target, startAgent());
    assert(near(sixtyN.agent.pos, sixtyS.agent.pos, 1e-6));
}

void wanderDoesNotDependOnTheStepRate() {
    const Params p = openWorld();
    const Run fast = simulate(Behavior::Wander, 120.0, true, true, 4.0, 99.0, p, {0, 0}, startAgent());
    const Run slow = simulate(Behavior::Wander, 30.0, true, true, 4.0, 99.0, p, {0, 0}, startAgent());
    std::printf("wander gap after 4 s: %.1f px\n", distance(fast.agent.pos, slow.agent.pos));
    assert(distance(fast.agent.pos, slow.agent.pos) < 40.0);
    assert(near(fast.agent.time, 4.0, 1e-9) && near(slow.agent.time, 4.0, 1e-9));
}

void fleeStopsOutsideThePanicRadius() {
    const Params p = openWorld();
    Agent a;
    a.pos = {0, 0};
    a.vel = {0, 0};
    const Run r = simulate(Behavior::Flee, 60.0, true, true, 12.0, 0.0, p, {50, 0}, a);
    // It ran away to the left, left the panic radius and braked to a halt there.
    assert(r.finalDistance >= p.panicRadius);
    assert(r.finalSpeed < 1.0);
    assert(r.agent.pos.x < 0.0);
}

void pathFollowingStaysNearTheLoop() {
    Params p = openWorld();
    Path loop{{0, 0}, {400, 0}, {400, 300}, {0, 300}};
    Agent a;
    a.pos = {0, 0};
    a.vel = {p.maxSpeed, 0};
    a.heading = {1, 0};
    double worst = 0;
    for (int i = 0; i < 60 * 40; ++i) {
        advance(a, Behavior::Path, {0, 0}, p, loop, 60.0, true, true);
        if (i > 60 * 5) worst = std::max(worst, closestOnPath(loop, a.pos).distance);
    }
    std::printf("path: worst distance from the loop %.1f px\n", worst);
    assert(worst < 3.0 * p.path.radius);
}

}  // namespace

int main() {
    std::setvbuf(stdout, nullptr, _IONBF, 0);  // keep the printed numbers when an assert aborts
    vectorBasics();
    seekPointsAtTheTarget();
    fleeIsTheMirror();
    arriveRampsDownInsideTheRadius();
    forceIsLimitedPerSecondAndScaledByDt();
    wrapAroundTheField();
    wanderNoiseIsSmoothAndBounded();
    wanderTargetSitsOnTheCircleAheadOfTheAgent();
    closestPointOnAClosedPath();
    closestPointSurvivesDegenerateSegments();
    pointAheadWalksAroundCorners();
    pathFollowingOnlyActsOutsideThePath();
    seekNeverSettles();
    arriveSettles();
    seekWithoutForceLimitJitters();
    dtScalingKeepsTheSpeedAtAnyFrameRate();
    wanderDoesNotDependOnTheStepRate();
    fleeStopsOutsideThePanicRadius();
    pathFollowingStaysNearTheLoop();
    std::puts("steering-behaviors: all tests passed");
    return EXIT_SUCCESS;
}
