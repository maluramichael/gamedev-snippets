// Tests for the spider foot placement logic. Plain asserts on hand-computed cases, no framework, no SDL.
// Body-local axes: x points forward, y to the right (screen coordinates, y down). Legs are numbered
// 0 FL, 1 FR, 2 ML, 3 MR, 4 RL, 5 RR; tripod group A = {0, 3, 4}, group B = {1, 2, 5}.
#undef NDEBUG
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "logic.hpp"

using namespace spider;

namespace {

constexpr double kPi = 3.14159265358979323846;

bool near(double a, double b, double eps = 1e-9) { return std::fabs(a - b) < eps; }
bool near(Vec2 a, Vec2 b, double eps = 1e-9) { return near(a.x, b.x, eps) && near(a.y, b.y, eps); }
bool finite(Vec2 v) { return std::isfinite(v.x) && std::isfinite(v.y); }

// A walker at the origin facing +x with the parameters spelled out, so the hand-computed numbers below do not move
// when the tuning of the demo does.
Walker plain(Gait gait, bool lookahead = true) {
    Walker w = makeWalker({0.0, 0.0}, 0.0, gait, lookahead);
    w.p.d1 = 54.0;
    w.p.d2 = 70.0;
    w.p.stepThreshold = 50.0;
    w.p.settleThreshold = 5.0;
    w.p.settleSpeed = 11.0;
    w.p.lookahead = 0.2;
    w.p.maxAhead = 25.0;
    w.p.swingTime = 0.12;
    w.p.liftHeight = 14.0;
    return w;
}

// Put a foot `back` behind its step target (the body moves along +x, so "behind" is -x).
void footBehindTarget(Walker& w, int leg, double back) { w.legs[static_cast<std::size_t>(leg)].foot = stepTarget(w, leg) - Vec2{back, 0.0}; }

int count(const std::array<bool, kLegs>& a) {
    int n = 0;
    for (const bool b : a) n += b ? 1 : 0;
    return n;
}

// The 3-4-5 triangle: d1 = 3, d2 = 4, foot at distance 5 gives cos(alpha) = (9 + 25 - 16) / 30 = 0.6, sin = 0.8, the knee
// is at 1.8 along the line and 2.4 to the side. The "outward" vector picks the side.
void legIkThreeFourFive() {
    const Vec2 hip{0, 0};
    const Vec2 foot{5, 0};
    const LegPose down = solveLeg(hip, foot, 3, 4, {0, 1});
    const LegPose up = solveLeg(hip, foot, 3, 4, {0, -1});
    assert(near(down.knee, {1.8, 2.4}));
    assert(near(up.knee, {1.8, -2.4}));
    assert(near(down.end, foot) && near(up.end, foot));

    // Rotated and shifted: foot straight "down" the screen, knee pushed towards +x.
    const LegPose r = solveLeg({10, 10}, {10, 15}, 3, 4, {1, 0});
    assert(near(r.knee, {12.4, 11.8}));
    assert(near(length(r.knee - Vec2{10, 10}), 3.0));
    assert(near(length(r.end - r.knee), 4.0));
    // An outward vector at an angle still chooses by the sign of the dot product: (1, 1) prefers the lower knee here.
    assert(near(solveLeg(hip, foot, 3, 4, {1, 1}).knee, {1.8, 2.4}));
}

void legIkOutOfRange() {
    // Too far: stretched straight towards the foot, the end stops at d1 + d2.
    const LegPose far = solveLeg({0, 0}, {10, 0}, 3, 4, {0, 1});
    assert(near(far.knee, {3, 0}) && near(far.end, {7, 0}));
    // Too close: folded, the end sits at |d1 - d2| from the hip, no NaN.
    const LegPose close = solveLeg({0, 0}, {0.5, 0}, 3, 4, {0, 1});
    assert(finite(close.knee) && finite(close.end));
    assert(near(length(close.knee), 3.0));
    assert(near(length(close.end), 1.0));
    // On the hip: still finite.
    const LegPose on = solveLeg({2, 2}, {2, 2}, 3, 4, {0, 1});
    assert(finite(on.knee) && finite(on.end));
}

void clampsToReach() {
    assert(near(clampToReach({0, 0}, {6, 8}, 5.0), {3, 4}));
    assert(near(clampToReach({0, 0}, {3, 4}, 5.0), {3, 4}));  // exactly on the border: untouched
    assert(near(clampToReach({0, 0}, {1, 1}, 5.0), {1, 1}));
    assert(near(clampToReach({1, 1}, {1, 1}, 5.0), {1, 1}));  // on the hip: no division by zero
    assert(near(clampToReach({10, 0}, {10, -20}, 5.0), {10, -5}));
}

// The step rule compares with "greater than": a foot exactly on the threshold stays.
void stepRuleIsStrict() {
    assert(!needsStep({3, 4}, {0, 0}, 5.0));
    assert(needsStep({3, 4}, {0, 0}, 4.99));
    assert(!needsStep({0, 0}, {0, 0}, 0.0));
    assert(needsStep({0.001, 0}, {0, 0}, 0.0));
}

void hullAndInside() {
    const std::vector<Vec2> square{{0, 0}, {4, 0}, {4, 4}, {0, 4}, {2, 2}};
    const std::vector<Vec2> hull = convexHull(square);
    assert(hull.size() == 4);  // the interior point is dropped
    assert(insideHull(hull, {2, 2}));
    assert(!insideHull(hull, {5, 2}));
    assert(!insideHull(hull, {2, -0.001}));
    assert(insideHull(hull, {4, 2}));  // on an edge counts as inside
    assert(insideHull(hull, {0, 0}));  // so does a corner

    // Orientation of the input must not matter.
    const std::vector<Vec2> reversed{{0, 4}, {4, 4}, {4, 0}, {0, 0}};
    assert(insideHull(convexHull(reversed), {2, 2}));

    // Collinear points and fewer than three points have no area.
    assert(!insideHull(convexHull({{0, 0}, {1, 1}, {2, 2}}), {1, 1}));
    assert(!insideHull(convexHull({{0, 0}, {1, 1}}), {0.5, 0.5}));
    assert(!insideHull(convexHull({}), {0, 0}));
    assert(!insideHull(convexHull({{0, 0}, {0, 0}, {0, 0}}), {0, 0}));

    // Triangle with the hypotenuse x + y = 1.
    const std::vector<Vec2> tri = convexHull({{-1, -1}, {2, -1}, {-1, 2}});
    assert(insideHull(tri, {0, 0}));
    assert(insideHull(tri, {0.5, 0.5}));  // on the hypotenuse
    assert(!insideHull(tri, {0.6, 0.6}));
}

void worldTransform() {
    // Heading 90 degrees turns local (20, -12) into (12, 20): x' = x cos - y sin = 12, y' = x sin + y cos = 20.
    const Vec2 p = toWorld({100, 50}, kPi / 2.0, {20, -12});
    assert(near(p, {112, 70}));
    assert(near(toWorld({3, 4}, 0.0, {1, 2}), {4, 6}));
    // The same for the leg tables: leg 0 (front left) hip is (20, -12).
    Walker w = plain(Gait::Tripod);
    w.pos = {100, 50};
    w.heading = kPi / 2.0;
    assert(near(hipWorld(w, 0), {112, 70}));
}

void supportInNeutralStance() {
    Walker w = plain(Gait::Tripod);
    const auto setPlanted = [&](bool a, bool b) {
        for (int i = 0; i < kLegs; ++i) {
            const bool planted = legDefs()[static_cast<std::size_t>(i)].group == 0 ? a : b;
            w.legs[static_cast<std::size_t>(i)].phase = planted ? Phase::Planted : Phase::Swinging;
        }
    };
    setPlanted(true, true);
    assert(plantedCount(w) == 6 && supported(w));
    // Either tripod alone holds the center: FL (56,-82), MR (4,96), RL (-60,-86) is a triangle around the origin
    // (at y = 0 it spans x = -29.8 .. 32.0, a hand count), and the mirror image for the other group.
    setPlanted(true, false);
    assert(plantedCount(w) == 3 && supported(w));
    setPlanted(false, true);
    assert(plantedCount(w) == 3 && supported(w));
    setPlanted(false, false);
    assert(plantedCount(w) == 0 && !supported(w));
    // Two feet, or three on one side only, cannot hold the center.
    setPlanted(false, true);
    w.legs[1].phase = Phase::Swinging;
    assert(plantedCount(w) == 2 && !supported(w));
    setPlanted(false, false);
    for (const int i : {0, 2, 4}) w.legs[static_cast<std::size_t>(i)].phase = Phase::Planted;  // all on the left
    assert(plantedCount(w) == 3 && !supported(w));
    // Same checks with the body moved and turned.
    w.pos = {300, 120};
    w.heading = 1.0;
    for (auto& l : w.legs) l.foot = {0, 0};
    setPlanted(true, true);
    for (int i = 0; i < kLegs; ++i) w.legs[static_cast<std::size_t>(i)].foot = restWorld(w, i);
    assert(supported(w));
}

void stepTargets() {
    Walker w = plain(Gait::Tripod);
    const Vec2 rest0 = restWorld(w, 0);
    assert(near(rest0, {56, -82}));
    w.vel = {150, 0};  // 150 * 0.2 = 30, above maxAhead 25: clamped
    assert(near(stepTarget(w, 0), {81, -82}));
    w.vel = {60, 0};  // 12
    assert(near(stepTarget(w, 0), {68, -82}));
    w.vel = {0, 0};
    assert(near(stepTarget(w, 0), rest0));
    w.vel = {0, -60};  // 12 to the side, the clamp works on the length
    assert(near(stepTarget(w, 0), {56, -94}));
    w.vel = {150, 0};
    w.lookahead = false;
    assert(near(stepTarget(w, 0), rest0));
}

void tripodChoosesAGroup() {
    Walker w = plain(Gait::Tripod);
    w.vel = {100, 0};  // threshold 50
    // Only leg 0 (group A) is beyond the threshold: the whole group A starts, group B stays.
    footBehindTarget(w, 0, 56.0);
    auto s = chooseSteps(w);
    assert(s[0] && s[3] && s[4] && !s[1] && !s[2] && !s[5]);
    // 50 exactly is not beyond.
    footBehindTarget(w, 0, 50.0);
    assert(count(chooseSteps(w)) == 0);
    // Nothing needs a step.
    for (int i = 0; i < kLegs; ++i) footBehindTarget(w, i, 40.0);
    assert(count(chooseSteps(w)) == 0);
    // A needs a step, but a leg of group B is in the air: nobody starts.
    footBehindTarget(w, 0, 56.0);
    w.legs[1].phase = Phase::Swinging;
    assert(count(chooseSteps(w)) == 0);
    w.legs[1].phase = Phase::Planted;
    // Both groups need a step: the one that is further behind goes first.
    footBehindTarget(w, 0, 56.0);
    footBehindTarget(w, 5, 70.0);
    s = chooseSteps(w);
    assert(s[1] && s[2] && s[5] && !s[0] && !s[3] && !s[4]);
    // Slow body: the settle threshold (5) applies below settleSpeed (11), a 7 px offset is enough.
    Walker slow = plain(Gait::Tripod);
    slow.vel = {5, 0};
    slow.lookahead = false;
    slow.legs[2].foot = restWorld(slow, 2) + Vec2{7, 0};
    s = chooseSteps(slow);
    assert(s[1] && s[2] && s[5] && count(s) == 3);
    slow.legs[2].foot = restWorld(slow, 2) + Vec2{4.9, 0};
    assert(count(chooseSteps(slow)) == 0);
}

void crawlMovesOneLeg() {
    Walker w = plain(Gait::Crawl);
    w.vel = {100, 0};
    footBehindTarget(w, 0, 56.0);
    footBehindTarget(w, 5, 70.0);
    auto s = chooseSteps(w);
    assert(count(s) == 1 && s[5]);  // the leg that is further behind
    w.legs[2].phase = Phase::Swinging;
    assert(count(chooseSteps(w)) == 0);  // one leg is up already
}

void freeMovesAnyLeg() {
    Walker w = plain(Gait::Free);
    w.vel = {100, 0};
    footBehindTarget(w, 0, 56.0);
    footBehindTarget(w, 5, 70.0);
    footBehindTarget(w, 3, 30.0);
    w.legs[2].phase = Phase::Swinging;
    footBehindTarget(w, 2, 80.0);  // in the air already: never restarted
    auto s = chooseSteps(w);
    assert(s[0] && s[5] && !s[3] && !s[2] && count(s) == 2);
}

// A swing interpolates from where the foot left to the step target; the middle of a swing is the middle of the way
// and the highest point of the lift.
void swingMotion() {
    Walker w = plain(Gait::Tripod);  // standing: vel 0, target = rest point
    Leg& leg = w.legs[0];
    leg.start = restWorld(w, 0) - Vec2{20, 0};
    leg.foot = leg.start;
    leg.phase = Phase::Swinging;
    leg.t = 0.0;
    assert(near(legLift(w, 0), 0.0));
    update(w, w.pos, 0.06);  // half of swingTime (0.12)
    assert(w.legs[0].phase == Phase::Swinging);
    assert(near(w.legs[0].t, 0.5));
    assert(near(w.legs[0].foot, {46, -82}));  // smoothstep(0.5) = 0.5
    assert(near(legLift(w, 0), 14.0));        // sin(pi * 0.5) * liftHeight
    update(w, w.pos, 0.06);
    assert(w.legs[0].phase == Phase::Planted);
    assert(near(w.legs[0].foot, {56, -82}));
    assert(near(legLift(w, 0), 0.0));
    // The other feet never moved.
    for (int i = 1; i < kLegs; ++i) assert(near(w.legs[static_cast<std::size_t>(i)].foot, restWorld(w, i)));
}

void brakeStopsAtTheReachLimit() {
    Walker w = plain(Gait::Free);  // limit = 0.97 * 124 = 120.28
    for (auto& l : w.legs) l.phase = Phase::Swinging;
    w.legs[0].phase = Phase::Planted;
    const Vec2 hip = hipWorld(w, 0);  // (20, -12)
    w.legs[0].foot = hip - Vec2{112, 0};
    // Moving forward by 10 would make the distance 122: the longest allowed share f solves 112 + 10 f = 120.28.
    assert(near(brakeFactor(w, {10, 0}, 0.0), 0.828, 1e-4));
    assert(near(brakeFactor(w, {-10, 0}, 0.0), 1.0));  // towards the foot: free
    assert(near(brakeFactor(w, {1, 0}, 0.0), 1.0));    // 113 is still fine
    // Already beyond the limit (distance 122): moving further away is stopped, moving closer is allowed.
    w.legs[0].foot = hip - Vec2{122, 0};
    assert(near(brakeFactor(w, {10, 0}, 0.0), 0.0, 1e-4));
    assert(near(brakeFactor(w, {-10, 0}, 0.0), 1.0));
    // Swinging legs do not hold the body back.
    w.legs[0].phase = Phase::Swinging;
    assert(near(brakeFactor(w, {10, 0}, 0.0), 1.0));
}

void angles() {
    assert(near(angleDiff(350.0 * kPi / 180.0, 10.0 * kPi / 180.0), 20.0 * kPi / 180.0));
    assert(near(angleDiff(10.0 * kPi / 180.0, 350.0 * kPi / 180.0), -20.0 * kPi / 180.0));
    assert(near(angleDiff(1.0, 1.0), 0.0));
    assert(std::fabs(angleDiff(0.0, 3.0 * kPi)) <= kPi + 1e-12);
}

void statsAccumulate() {
    Walker w = plain(Gait::Tripod);
    Stats s;
    w.vel = {100, 0};
    record(s, w, 0, 0.5);  // six feet down for half a second while moving
    assert(near(s.moveTime, 0.5) && near(dutyFactor(s), 1.0));
    assert(s.minPlanted == 6 && s.steps == 0 && near(s.unsupportedTime, 0.0));
    for (const int i : {1, 2, 5}) w.legs[static_cast<std::size_t>(i)].phase = Phase::Swinging;
    record(s, w, 3, 0.5);  // group B in the air: three of six feet for half a second
    // stance time: 6 * 0.5 + 3 * 0.5 = 4.5 over 6 legs * 1.0 s
    assert(near(dutyFactor(s), 0.75));
    assert(s.minPlanted == 3 && s.steps == 3 && near(s.moveTime, 1.0));
    assert(near(s.unsupportedTime, 0.0));  // tripod A holds the center
    for (auto& l : w.legs) l.phase = Phase::Swinging;
    record(s, w, 0, 0.25);
    assert(s.minPlanted == 0 && near(s.unsupportedTime, 0.25));
    assert(near(dutyFactor(s), 4.5 / 7.5));  // stance stays 4.5, total 6 * 1.25
    // Standing still does not count (speed below the moving limit).
    w.vel = {5, 0};
    record(s, w, 2, 10.0);
    assert(near(s.moveTime, 1.25) && s.steps == 3);
    // Empty stats do not divide by zero.
    assert(near(dutyFactor(Stats{}), 0.0));
    // Reset forgets everything, minPlanted starts at the leg count again.
    s = Stats{};
    assert(s.minPlanted == kLegs && s.steps == 0);
}

void demoPath() {
    assert(near(advancePhase(kAutoPeriod - 0.5, 1.0), 0.5));
    assert(advancePhase(0.0, -0.5) >= 0.0 && advancePhase(0.0, -0.5) < kAutoPeriod);
    assert(near(advancePhase(3.0, 0.0), 3.0));
    // A quarter period: sin(pi/2) = 1 in x, sin(pi) = 0 in y.
    const Vec2 g = autoGoal(kAutoPeriod / 4.0);
    assert(near(g.x, kPathCenter.x + kPathHalfWidth, 1e-9));
    assert(near(g.y, kPathCenter.y, 1e-9));
    // Heading at phase 0: x' = 200, y' = 2 * 80 = 160 (the common factor 2 pi / P drops out), a quarter period: x' = 0, y' = -160.
    assert(near(autoHeading(0.0), std::atan2(160.0, 200.0)));
    assert(near(autoHeading(kAutoPeriod / 4.0), std::atan2(-160.0, 0.0), 1e-9));
    for (double t = 0.0; t < kAutoPeriod; t += 0.37) {
        const Vec2 q = autoGoal(t);
        assert(std::fabs(q.x - kPathCenter.x) <= kPathHalfWidth + 1e-9);
        assert(std::fabs(q.y - kPathCenter.y) <= kPathHalfHeight + 1e-9);
    }
}

// ---- simulations -----------------------------------------------------------------------------------------------

struct Result {
    double maxStretch = 0.0;    // largest hip-foot distance seen
    int maxSwinging = 0;
    int minPlanted = kLegs;
    int unsupportedTicks = 0;
    double lastUnsupported = 0.0;  // time of the last unsupported tick
    int bothGroupsInAir = 0;    // ticks with a swinging leg in group A and one in group B
    int movedPlanted = 0;       // ticks in which a planted foot changed its world position
    int zeroPlantedTicks = 0;
    double distance = 0.0;
    double time = 0.0;
    Stats stats;
};

// Runs the walker towards a goal function for `seconds` at 120 Hz and checks the invariants on every tick.
template <class Goal>
Result run(Walker& w, double seconds, Goal goal) {
    Result r;
    const double dt = 1.0 / 120.0;
    const int ticks = static_cast<int>(seconds / dt);
    double phase = 0.0;
    for (int k = 0; k < ticks; ++k) {
        const Walker before = w;
        phase = advancePhase(phase, dt);
        const int started = update(w, goal(phase, w), dt);
        record(r.stats, w, started, dt);
        r.distance += length(w.pos - before.pos);
        r.time += dt;
        int swinging = 0;
        int inA = 0;
        int inB = 0;
        for (int i = 0; i < kLegs; ++i) {
            const auto idx = static_cast<std::size_t>(i);
            const Leg& leg = w.legs[idx];
            r.maxStretch = std::max(r.maxStretch, length(hipWorld(w, i) - leg.foot));
            if (leg.phase == Phase::Swinging) {
                ++swinging;
                (legDefs()[idx].group == 0 ? inA : inB) += 1;
            }
            if (before.legs[idx].phase == Phase::Planted && (leg.foot.x != before.legs[idx].foot.x || leg.foot.y != before.legs[idx].foot.y)) ++r.movedPlanted;
        }
        r.maxSwinging = std::max(r.maxSwinging, swinging);
        r.minPlanted = std::min(r.minPlanted, kLegs - swinging);
        if (swinging == kLegs) ++r.zeroPlantedTicks;
        if (inA > 0 && inB > 0) ++r.bothGroupsInAir;
        if (!supported(w)) {
            ++r.unsupportedTicks;
            r.lastUnsupported = r.time;
        }
    }
    return r;
}

Vec2 pathGoal(double phase, const Walker&) { return autoGoal(phase); }

void tripodNeverLosesSupport() {
    Walker w = makeWalker(autoGoal(0.0), autoHeading(0.0), Gait::Tripod, true);
    const Result r = run(w, 40.0, pathGoal);
    const double reach = maxReach(w.p);
    assert(r.distance > 600.0);                       // it really walked
    assert(r.stats.steps > 40);
    assert(r.bothGroupsInAir == 0);                   // the rule: one group in the air at a time
    assert(r.minPlanted >= 3);                        // so three feet are always down
    assert(r.unsupportedTicks <= 12 && r.lastUnsupported < 1.0);  // and they hold the center, but for the start: all feet are in step then
    assert(r.maxStretch <= reach + 1e-9);             // no leg longer than d1 + d2
    assert(r.movedPlanted == 0);                      // a planted foot never changes its world position
    assert(r.maxSwinging == 3);
}

void crawlOneAtATime() {
    Walker w = makeWalker(autoGoal(0.0), autoHeading(0.0), Gait::Crawl, true);
    const Result r = run(w, 40.0, pathGoal);
    assert(r.maxSwinging == 1);
    assert(r.minPlanted == 5);
    assert(r.maxStretch <= maxReach(w.p) + 1e-9);
    assert(r.movedPlanted == 0);
    assert(r.stats.steps > 20);
}

// Without a rule the feet lift together: they all drift by the same amount, so they all cross the threshold at the
// same moment. In the air there is nothing to hold the body.
void freeLiftsEverything() {
    Walker w = makeWalker(autoGoal(0.0), autoHeading(0.0), Gait::Free, true);
    const Result r = run(w, 40.0, pathGoal);
    assert(r.zeroPlantedTicks > 0);
    assert(r.minPlanted == 0);
    assert(r.unsupportedTicks > 0);
    assert(r.maxStretch <= maxReach(w.p) + 1e-9);
    assert(r.movedPlanted == 0);
}

struct Straight {
    double speed;             // px/s, measured from 4 s to 10 s
    double duty;
    double unsupported;       // share of the moving time
    double maxStretch;
    int minPlanted;
};

// A straight walk towards a far goal for 10 s, the numbers of the last 6 s (the feet are in step by then).
Straight straight(Gait gait, bool lookahead) {
    Walker w = makeWalker({50, 240}, 0.0, gait, lookahead);
    const double dt = 1.0 / 120.0;
    Stats stats;
    Vec2 from{0, 0};
    Straight s{0.0, 0.0, 0.0, 0.0, kLegs};
    for (int k = 0; k < 1200; ++k) {
        if (k == 480) {
            from = w.pos;
            stats = Stats{};
        }
        const int started = update(w, {5000, 240}, dt);
        if (k >= 480) {
            record(stats, w, started, dt);
            for (int i = 0; i < kLegs; ++i) s.maxStretch = std::max(s.maxStretch, length(hipWorld(w, i) - w.legs[static_cast<std::size_t>(i)].foot));
        }
    }
    s.speed = length(w.pos - from) / 6.0;
    s.duty = dutyFactor(stats);
    s.unsupported = stats.unsupportedTime / stats.moveTime;
    s.minPlanted = stats.minPlanted;
    return s;
}

// The same straight walk for every gait, with and without look-ahead. These are measurements of the demo's own
// parameters (195 px/s top speed, step threshold 50 px, swing 0.12 s), not general numbers.
void gaitsOnAStraightWalk() {
    const Straight tri = straight(Gait::Tripod, true);
    const Straight crawl = straight(Gait::Crawl, true);
    const Straight free = straight(Gait::Free, true);
    const Straight triNo = straight(Gait::Tripod, false);
    const double reach = maxReach(Params{});

    // Tripod reaches the top speed, always has three feet down and holds the center the whole time.
    assert(tri.speed > 0.97 * Params{}.maxSpeed);
    assert(tri.minPlanted == 3);
    assert(near(tri.unsupported, 0.0));
    assert(tri.duty > 0.5);  // walking in the biomechanics sense
    // Free is as fast and has moments without any foot on the ground.
    assert(free.speed > 0.97 * Params{}.maxSpeed);
    assert(free.minPlanted == 0);
    assert(free.unsupported > 0.2);
    // Crawl keeps five feet down but is slower, and five feet are no guarantee: the center still leaves their polygon
    // now and then, because all the feet lag behind.
    assert(crawl.minPlanted == 5);
    assert(crawl.speed < 0.75 * tri.speed);
    assert(crawl.unsupported > 0.01 && crawl.unsupported < 0.5 * free.unsupported);
    assert(crawl.duty > tri.duty);
    // No look-ahead: the foot lands on the rest point and the whole stride is behind it, the leg stretches to the
    // limit, the body has to slow down and the center falls behind the feet.
    assert(triNo.speed < 0.85 * tri.speed);
    assert(triNo.unsupported > 0.1);
    assert(triNo.maxStretch > 0.95 * reach && triNo.maxStretch <= reach);
    assert(tri.maxStretch < 0.95 * reach);
    for (const Straight* s : {&tri, &crawl, &free, &triNo}) assert(s->maxStretch <= reach + 1e-9);
    std::printf("straight walk, last 6 s: speed px/s, duty, unsupported, max stretch (of %.0f), fewest feet down\n", reach);
    std::printf("  tripod            %4.0f  %3.0f%%  %3.0f%%  %5.1f  %d\n", tri.speed, tri.duty * 100, tri.unsupported * 100, tri.maxStretch, tri.minPlanted);
    std::printf("  crawl             %4.0f  %3.0f%%  %3.0f%%  %5.1f  %d\n", crawl.speed, crawl.duty * 100, crawl.unsupported * 100, crawl.maxStretch, crawl.minPlanted);
    std::printf("  free              %4.0f  %3.0f%%  %3.0f%%  %5.1f  %d\n", free.speed, free.duty * 100, free.unsupported * 100, free.maxStretch, free.minPlanted);
    std::printf("  tripod, no lookah %4.0f  %3.0f%%  %3.0f%%  %5.1f  %d\n", triNo.speed, triNo.duty * 100, triNo.unsupported * 100, triNo.maxStretch, triNo.minPlanted);
}

void settlesWhenStopped() {
    Walker w = makeWalker({100, 240}, 0.0, Gait::Tripod, true);
    const Vec2 goal{400, 240};
    run(w, 3.0, [&](double, const Walker&) { return goal; });
    assert(length(w.pos - goal) < 60.0);
    const Vec2 stop = w.pos;
    run(w, 2.0, [&](double, const Walker&) { return stop; });
    assert(length(w.vel) < 1.0);
    for (int i = 0; i < kLegs; ++i) {
        assert(w.legs[static_cast<std::size_t>(i)].phase == Phase::Planted);
        assert(length(w.legs[static_cast<std::size_t>(i)].foot - restWorld(w, i)) <= w.p.settleThreshold + 1e-9);
    }
}

void turnsTowardsTheGoal() {
    Walker w = makeWalker({400, 400}, 0.0, Gait::Tripod, true);
    const Vec2 goal{400, 100};
    run(w, 4.0, [&](double, const Walker&) { return goal; });
    assert(near(w.heading, -kPi / 2.0, 0.05));  // facing up the screen
    assert(length(w.pos - goal) < 10.0);
}

}  // namespace

int main() {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    legIkThreeFourFive();
    legIkOutOfRange();
    clampsToReach();
    stepRuleIsStrict();
    hullAndInside();
    worldTransform();
    supportInNeutralStance();
    stepTargets();
    tripodChoosesAGroup();
    crawlMovesOneLeg();
    freeMovesAnyLeg();
    swingMotion();
    brakeStopsAtTheReachLimit();
    angles();
    statsAccumulate();
    demoPath();
    tripodNeverLosesSupport();
    crawlOneAtATime();
    freeLiftsEverything();
    gaitsOnAStraightWalk();
    settlesWhenStopped();
    turnsTowardsTheGoal();
    std::puts("spider-foot-placement logic: all asserts passed");
    return 0;
}
