// Tests for the FABRIK vs CCD logic. Plain asserts on hand-computed cases, no framework, no SDL.
// Coordinates are screen coordinates (y points down). The measurements at the end print their numbers, the post quotes them.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "logic.hpp"

using namespace fk;

namespace {

bool near(double a, double b, double eps = 1e-9) { return std::fabs(a - b) <= eps; }
bool near(Vec2 a, Vec2 b, double eps = 1e-9) { return near(a.x, b.x, eps) && near(a.y, b.y, eps); }
bool finite(Vec2 v) { return std::isfinite(v.x) && std::isfinite(v.y); }
bool finite(const Chain& c) {
    for (int i = 0; i <= c.n; ++i) {
        if (!finite(c.p[static_cast<std::size_t>(i)])) return false;
    }
    return true;
}

constexpr Algo kAll[] = {Algo::Ccd, Algo::Fabrik, Algo::Relax};

const char* name(Algo a) { return a == Algo::Ccd ? "CCD" : (a == Algo::Fabrik ? "FABRIK" : "Relax"); }

// ---------------------------------------------------------------------------------------------------------------------
// The chain itself

void chainBasics() {
    const Chain c = makeChain({10, 20}, {5, 5, 5});
    assert(c.n == 3);
    assert(near(c.p[0], {10, 20}) && near(c.p[1], {15, 20}) && near(c.p[3], {25, 20}));
    assert(near(reach(c), 15.0) && near(minReach(c), 0.0));
    assert(near(lengthError(c), 0.0));

    // Unequal lengths leave a hole: 3 and 4 can fold to 1 but not to 0 (the longest is longer than the rest).
    const Chain u = makeChain({0, 0}, {3, 4});
    assert(near(reach(u), 7.0) && near(minReach(u), 1.0));
    // Equal lengths, and a long chain with short segments, can fold onto the root.
    assert(near(minReach(makeChain({0, 0}, {2, 2})), 0.0));
    assert(near(minReach(makeChain({0, 0}, {5, 1, 1})), 3.0));  // 2 * 5 - 7

    // A quarter turn per segment: (0,0) -> (1,0) -> (1,1), the second segment points down on the screen.
    const Chain b = makeChain({0, 0}, {1, 1}, kPi / 2.0);
    assert(near(b.p[1], {1, 0}) && near(b.p[2], {1, 1}));
    assert(near(bendAt(b, 1), kPi / 2.0));
    assert(near(totalBend(b), kPi / 2.0));
    assert(near(totalBend(makeUniformChain({0, 0}, 4, 2.0, 0.5)), 1.5));  // joints 1, 2, 3 bend 0.5 each
    assert(near(totalBend(makeUniformChain({0, 0}, 4, 2.0, -0.5)), 1.5));
    assert(near(totalBend(makeUniformChain({0, 0}, 4, 2.0, 0.0)), 0.0));

    assert(makeUniformChain({0, 0}, 99, 1.0).n == kMaxSegments);
    assert(makeUniformChain({0, 0}, 0, 1.0).n == 1);
    assert(makeChain({0, 0}, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}).n == kMaxSegments);
}

void errors() {
    const Chain c = makeUniformChain({0, 0}, 2, 1.0);  // reach 2, minReach 0
    assert(near(bestError(c, {1, 1}), 0.0));           // inside the disc
    assert(near(bestError(c, {2, 0}), 0.0));           // on the border
    assert(near(bestError(c, {3, 4}), 3.0));           // 5 - 2
    assert(near(bestError(c, {0, 0}), 0.0));           // equal lengths fold onto the root
    assert(near(tipError(c, {2, 3}), 3.0));            // the tip sits at (2,0)

    const Chain h = makeChain({0, 0}, {3, 4});  // hole of radius 1
    assert(near(bestError(h, {0.4, 0}), 0.6));
    assert(near(bestError(h, {0, 1}), 0.0));
    assert(near(bestError(h, {0, 8}), 1.0));

    assert(reached(c, {1.995, 0}, 0.01));
    assert(!reached(c, {1.9, 0}, 0.01));
    // Target (6,0) is out of reach, the best possible error is 4 and the straight chain (tip at (2,0)) has exactly that.
    assert(near(tipError(c, {6, 0}), 4.0));
    assert(reached(c, {6, 0}, 1e-9));
    const Chain bent = makeUniformChain({0, 0}, 2, 1.0, kPi / 2.0);  // tip at (1,1): 5.099 away
    assert(!reached(bent, {6, 0}, 0.01));
}

// ---------------------------------------------------------------------------------------------------------------------
// CCD

// Two segments of length 1, straight along +x, target (1, 1). The elbow (1,0) sees the tip at (1,0) and the target at
// (0,1): +90 degrees, the tip lands on (1,1). The root then sees tip and target in the same direction: no turn.
void ccdByHand() {
    Chain c = makeUniformChain({0, 0}, 2, 1.0);
    Trace t;
    ccdIteration(c, {1, 1}, &t);
    assert(near(c.p[1], {1, 0}) && near(c.p[2], {1, 1}));
    assert(near(root(c), {0, 0}));
    assert(t.count == 3);  // before, after joint 1, after joint 0
    assert(near(t.poses[0][2], {2, 0}));
    assert(near(t.poses[1][2], {1, 1}) && near(t.poses[1][1], {1, 0}));
    assert(near(t.poses[2][2], {1, 1}));

    // Target (0, 2), the straight chain would reach it upright. Elbow (1,0): tip direction (1,0), target direction
    // (-1,2), the tip swings to (1,0) + (-1,2)/sqrt(5) = (0.5528, 0.8944). The root then turns that tip onto the y axis:
    // |tip| = sqrt(2 - 2/sqrt(5)) = 1.05146 and the elbow follows to (0.85065, 0.52573). One sweep is not enough,
    // 0.9485 are left.
    Chain d = makeUniformChain({0, 0}, 2, 1.0);
    ccdIteration(d, {0, 2});
    assert(near(d.p[2], {0.0, 1.05146}, 1e-4));
    assert(near(d.p[1], {0.85065, 0.52573}, 1e-4));
    assert(near(tipError(d, {0, 2}), 0.94854, 1e-4));
    assert(near(lengthError(d), 0.0, 1e-12));
}

// Straight chain, target on the same line behind the root. The elbow turns the last segment by 180 degrees, which puts
// the tip on the root. The root joint then sees a tip with no direction and does nothing. The next sweep changes
// nothing either: Juckett's local minimum for CCD.
void ccdFrozenOnTheAxis() {
    Chain c = makeUniformChain({0, 0}, 2, 1.0);
    const Vec2 target{-0.5, 0.0};
    ccdIteration(c, target);
    assert(near(c.p[1], {1, 0}));
    assert(near(c.p[2], {0, 0}, 1e-9));
    const Pose before = c.p;
    ccdIteration(c, target);
    assert(maxShift(before, c.p, c.n) < 1e-9);
    assert(near(tipError(c, target), 0.5, 1e-9));
    assert(bestError(c, target) == 0.0);  // reachable: the elbow could go up or down, so this is a real failure

    Chain s = makeUniformChain({0, 0}, 2, 1.0);
    const Result r = solve(Algo::Ccd, s, target, 0.001, 100);
    assert(r.outcome == Outcome::Stuck);
    assert(r.iterations <= 3);
    assert(near(r.error, 0.5, 1e-9));
}

// ---------------------------------------------------------------------------------------------------------------------
// FABRIK

// Two segments of length 1, straight, target (0, 1.5).
// Backward: the tip goes to the target. Joint 1 is at distance sqrt(1 + 2.25) = sqrt(13)/2 from it, so k = 2/sqrt(13)
// = 0.5547 and p1 = (1 - k) * (0, 1.5) + k * (1, 0) = (0.5547, 0.66795). The root is pulled along (-0.0841, -0.1013).
// Forward: root back to the origin, p1 = 0.63888, 0.76931, tip = (-0.01935, 1.52213), 0.0294 from the target.
void fabrikByHand() {
    Chain c = makeUniformChain({0, 0}, 2, 1.0);
    Trace t;
    fabrikIteration(c, {0, 1.5}, &t);
    assert(t.count == 3);  // before, after the backward pass, after the forward pass
    assert(near(t.poses[0][1], {1, 0}));
    assert(near(t.poses[1][2], {0, 1.5}));
    assert(near(t.poses[1][1], {0.55470, 0.66795}, 1e-4));
    assert(near(t.poses[1][0], {-0.08414, -0.10133}, 1e-4));
    assert(near(c.p[0], {0, 0}, 0.0));  // bit-exact: the root is pinned
    assert(near(c.p[1], {0.63888, 0.76931}, 1e-4));
    assert(near(c.p[2], {-0.01935, 1.52213}, 1e-4));
    assert(near(tipError(c, {0, 1.5}), 0.02940, 1e-4));
    assert(near(lengthError(c), 0.0, 1e-12));
}

// Beyond the reach there is one answer and FABRIK writes it down in a single iteration, from any pose:
// k_i = d_i / |t - p_i| puts every joint on the line to the target.
void fabrikStretchesInOneStep() {
    for (const double bend : {0.0, 0.3, -1.2}) {
        Chain c = makeUniformChain({0, 0}, 3, 1.0, bend);
        Trace t;
        fabrikIteration(c, {6, 8}, &t);  // distance 10, reach 3, direction (0.6, 0.8)
        for (int i = 0; i <= 3; ++i) assert(near(c.p[static_cast<std::size_t>(i)], Vec2{0.6, 0.8} * static_cast<double>(i), 1e-12));
        assert(t.count == 2);
        assert(near(tipError(c, {6, 8}), 7.0, 1e-12));
        Chain again = c;
        const Result r = solve(Algo::Fabrik, again, {6, 8}, 0.01, 50);
        assert(r.outcome == Outcome::Reached && r.iterations == 0);  // the straight pose is already the best there is
    }
    Chain c = makeUniformChain({0, 0}, 3, 1.0, 0.3);
    const Result r = solve(Algo::Fabrik, c, {6, 8}, 0.01, 50);
    assert(r.outcome == Outcome::Reached && r.iterations == 1);
    // Exactly at the reach: the same branch, the tip lands on the target.
    Chain e = makeUniformChain({0, 0}, 3, 1.0, 0.3);
    fabrikIteration(e, {0, 3});
    assert(near(tip(e), {0, 3}, 1e-12));
}

// ---------------------------------------------------------------------------------------------------------------------
// Relaxation

// Same case as FABRIK: two segments of length 1, straight, target (0, 1.5).
// Pin the tip to (0, 1.5). Segment 1 (joint 1 to tip) is sqrt(3.25) = 1.80278 long instead of 1, the error 0.80278 is
// shared: each end moves 0.40139 towards the other. Joint 1 goes to (0.77735, 0.33398), the tip to (0.22265, 1.16603).
// Segment 0 is the root segment, only joint 1 moves: onto the circle of radius 1, (0.91879, 0.39474). The tip stays
// 0.40139 short of the target and segment 1 is no longer exactly 1 long.
void relaxByHand() {
    Chain c = makeUniformChain({0, 0}, 2, 1.0);
    Trace t;
    relaxIteration(c, {0, 1.5}, &t);
    assert(t.count == 4);  // before, tip pinned, after segment 1, after segment 0
    assert(near(t.poses[1][2], {0, 1.5}));
    assert(near(t.poses[2][1], {0.77735, 0.33398}, 1e-4));
    assert(near(t.poses[2][2], {0.22265, 1.16603}, 1e-4));
    assert(near(c.p[0], {0, 0}, 0.0));
    assert(near(c.p[1], {0.91879, 0.39474}, 1e-4));
    assert(near(c.p[2], {0.22265, 1.16603}, 1e-4));
    assert(near(tipError(c, {0, 1.5}), 0.40139, 1e-4));
    assert(near(lengthError(c), 0.03898, 1e-4));  // segment 1: |(0.22265, 1.16603) - (0.91879, 0.39474)| = 1.03898
    assert(near(length(c.p[1]), 1.0, 1e-12));  // the root segment is exact by construction
}

// ---------------------------------------------------------------------------------------------------------------------
// All three on many targets

struct Stats {
    double mean = 0.0;
    int worst = 0;
    int failed = 0;
    int count = 0;
};

// 10 unit segments from the rest pose (bend 0.12 per joint), 9 radii x 24 angles inside the reach, or 3 x 24 beyond it.
Stats measure(Algo algo, bool inside, double tol, int cap, double* maxLengthError = nullptr) {
    Stats s;
    const Chain rest = makeUniformChain({0, 0}, 10, 1.0, 0.12);
    const double L = reach(rest);
    const double radii[] = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9};
    const double outside[] = {1.1, 1.3, 1.6};
    const int rc = inside ? 9 : 3;
    long sum = 0;
    for (int ri = 0; ri < rc; ++ri) {
        const double r = (inside ? radii[ri] : outside[ri]) * L;
        for (int a = 0; a < 24; ++a) {
            const double ang = 2.0 * kPi * a / 24.0 + 0.1;
            const Vec2 target{std::cos(ang) * r, std::sin(ang) * r};
            Chain c = rest;
            const Result res = solve(algo, c, target, tol, cap);
            assert(finite(c));
            assert(near(c.p[0], {0, 0}, 0.0));
            if (res.outcome != Outcome::Reached) ++s.failed;
            s.worst = std::max(s.worst, res.iterations);
            sum += res.iterations;
            ++s.count;
            if (maxLengthError != nullptr) *maxLengthError = std::max(*maxLengthError, lengthError(c));
        }
    }
    s.mean = static_cast<double>(sum) / s.count;
    return s;
}

void everyIterationKeepsTheRules() {
    // CCD and FABRIK keep the root and all segment lengths in every single iteration, not just at the end.
    const Chain rest = makeUniformChain({3, -2}, 10, 1.0, 0.12);
    for (const Algo algo : {Algo::Ccd, Algo::Fabrik}) {
        for (const Vec2 target : {Vec2{-5, 4}, Vec2{9, 1}, Vec2{40, 40}, Vec2{3, -2}, Vec2{3, -1.5}, Vec2{-100, 0.001}}) {
            Chain c = rest;
            for (int i = 0; i < 60; ++i) {
                iterate(algo, c, target);
                assert(finite(c));
                assert(near(c.p[0], {3, -2}, 0.0));
                assert(lengthError(c) < 1e-9);
            }
        }
    }
    // Relaxation keeps the root too, but lengths only after it settled.
    Chain r = rest;
    iterate(Algo::Relax, r, {-5, 4});
    assert(near(r.p[0], {3, -2}, 0.0));
    assert(lengthError(r) > 1.0);
    for (int i = 0; i < 400; ++i) iterate(Algo::Relax, r, {-5, 4});
    assert(near(r.p[0], {3, -2}, 0.0));
    assert(lengthError(r) < 0.05);
}

// A target beyond the reach: relaxation pins the root and the tip and has no other way to make them meet than to
// stretch the segments. It settles with the tip near the target and every segment too long. CCD and FABRIK cannot do that.
void relaxStretchesBeyondTheReach() {
    const Vec2 target{40, 40};  // 10 unit segments from (3,-2): distance 56.0, reach 10
    Chain r = makeUniformChain({3, -2}, 10, 1.0, 0.12);
    const Result res = solve(Algo::Relax, r, target, 0.01, 5000);
    std::printf("relax beyond the reach: %s after %d iterations, length error %.2f per segment, tip error %.2f (best for a rigid chain: %.2f)\n",
                res.outcome == Outcome::Stuck ? "settled" : "other", res.iterations, lengthError(r), tipError(r, target), bestError(r, target));
    assert(res.outcome == Outcome::Stuck);
    assert(lengthError(r) > 4.0);
    assert(tipError(r, target) < 0.5 * bestError(r, target));
    assert(!reached(r, target, 0.01));
    for (const Algo algo : {Algo::Ccd, Algo::Fabrik}) {
        Chain c = makeUniformChain({3, -2}, 10, 1.0, 0.12);
        for (int i = 0; i < 100; ++i) iterate(algo, c, target);
        assert(lengthError(c) < 1e-9);
    }
}

void reachableTargets() {
    double lenErr[3] = {0, 0, 0};
    Stats s[3];
    int k = 0;
    for (const Algo algo : kAll) {
        s[k] = measure(algo, true, 0.01, 20000, &lenErr[k]);
        std::printf("reachable   %-6s targets %d  mean %.1f iterations  worst %d  not reached %d  max length error %.4f\n", name(algo), s[k].count, s[k].mean, s[k].worst, s[k].failed,
                    lenErr[k]);
        assert(s[k].failed == 0);
        ++k;
    }
    assert(s[1].mean * 3.0 < s[0].mean);  // FABRIK needs far fewer sweeps than CCD ...
    assert(s[0].mean < s[2].mean);        // ... and CCD fewer than relaxation
    assert(lenErr[0] < 1e-9 && lenErr[1] < 1e-9);
    assert(lenErr[2] > 1e-5);             // relaxation stops with a visible length error
}

void unreachableTargets() {
    Stats s[3];
    int k = 0;
    for (const Algo algo : kAll) {
        s[k] = measure(algo, false, 0.01, 20000);
        std::printf("unreachable %-6s targets %d  mean %.1f iterations  worst %d  not reached %d\n", name(algo), s[k].count, s[k].mean, s[k].worst, s[k].failed);
        ++k;
    }
    assert(s[0].failed == 0 && s[1].failed == 0);
    assert(s[1].worst == 1 && near(s[1].mean, 1.0));  // FABRIK: always one
    assert(s[0].mean > 20.0 * s[1].mean);              // CCD: dozens
    assert(s[2].failed == s[2].count);                 // relaxation never ends with valid lengths
}

// Beyond the reach CCD and FABRIK end with all joints on the line root -> target.
void unreachableEndsStraight() {
    const Vec2 target{-30.0, 40.0};  // distance 50, reach 10
    const Vec2 dir{-0.6, 0.8};
    for (const Algo algo : {Algo::Ccd, Algo::Fabrik}) {
        Chain c = makeUniformChain({0, 0}, 10, 1.0, 0.12);
        const Result r = solve(algo, c, target, 0.001, 20000);
        assert(r.outcome == Outcome::Reached);
        for (int i = 1; i <= c.n; ++i) {
            const Vec2 p = c.p[static_cast<std::size_t>(i)];
            assert(std::fabs(cross(dir, p)) < 0.05);  // distance from the line
            assert(dot(dir, p) > 0.0);
        }
        assert(near(length(tip(c)), 10.0, 0.01));
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// Curling: what CCD does to a chain that has to bend

double signedBendDegrees(const Chain& c, int j) { return bendAt(c, j) * 180.0 / kPi; }

void ccdCurlsUp() {
    // A straight chain of 10 unit segments, target straight above at 7.5 (reach 10).
    const Chain straight = makeUniformChain({0, 0}, 10, 1.0);
    const Vec2 target{0, -7.5};
    Chain ccd = straight;
    Chain fab = straight;
    ccdIteration(ccd, target);
    fabrikIteration(fab, target);
    int ccdMax = 1;
    int fabMax = 1;
    for (int j = 1; j < 10; ++j) {
        if (std::fabs(bendAt(ccd, j)) > std::fabs(bendAt(ccd, ccdMax))) ccdMax = j;
        if (std::fabs(bendAt(fab, j)) > std::fabs(bendAt(fab, fabMax))) fabMax = j;
    }
    std::printf("first iteration, bend at the joint next to the tip: CCD %.1f deg, FABRIK %.1f deg; largest bend: CCD at joint %d (%.1f deg), FABRIK at joint %d (%.1f deg)\n",
                signedBendDegrees(ccd, 9), signedBendDegrees(fab, 9), ccdMax, signedBendDegrees(ccd, ccdMax), fabMax, signedBendDegrees(fab, fabMax));
    assert(ccdMax == 9);                  // CCD puts the biggest bend at the joint it visits first
    assert(fabMax != 9);                  // FABRIK spreads it out
    assert(std::fabs(bendAt(ccd, 9)) > 2.0 * std::fabs(bendAt(fab, fabMax)));

    // Converged: how curled is the pose?
    for (const Vec2 t : {Vec2{0, -7.5}, Vec2{-6, -3}}) {
        Chain a = straight;
        Chain b = straight;
        assert(solve(Algo::Ccd, a, t, 0.01, 20000).outcome == Outcome::Reached);
        assert(solve(Algo::Fabrik, b, t, 0.01, 20000).outcome == Outcome::Reached);
        std::printf("converged on (%.1f, %.1f): total bend CCD %.0f deg, FABRIK %.0f deg\n", t.x, t.y, totalBend(a) * 180.0 / kPi, totalBend(b) * 180.0 / kPi);
        assert(totalBend(a) > 2.0 * totalBend(b));
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// The stuck preset: two segments, straight, the target on the same line

void stuckPreset() {
    const Vec2 axis{-0.5, 0.0};  // the chain is 1 + 1 long and straight along +x, the target sits behind the root
    for (const Algo algo : kAll) {
        Chain c = makeUniformChain({0, 0}, 2, 1.0);
        const Result r = solve(algo, c, axis, 0.001, 1000);
        std::printf("stuck %-6s on the axis: %s after %d iterations, error %.3f\n", name(algo), r.outcome == Outcome::Stuck ? "stuck" : "other", r.iterations, r.error);
        assert(r.outcome == Outcome::Stuck);
        assert(r.error > 0.2);
        assert(bestError(c, axis) == 0.0);
    }
    // Nudged off the axis by 0.01 (1 px of a 100 px segment): every method gets out.
    int iters[3] = {0, 0, 0};
    int k = 0;
    for (const Algo algo : kAll) {
        Chain c = makeUniformChain({0, 0}, 2, 1.0);
        const Result r = solve(algo, c, {-0.5, 0.01}, 0.001, 1000);
        std::printf("stuck %-6s nudged by 0.01: %s after %d iterations, error %.4f\n", name(algo), r.outcome == Outcome::Reached ? "reached" : "not reached", r.iterations, r.error);
        assert(r.outcome == Outcome::Reached);
        iters[k++] = r.iterations;
    }
    assert(iters[0] <= 5);   // CCD jumps out at once ...
    assert(iters[1] > 10);   // ... FABRIK and relaxation need many sweeps to creep out
    assert(iters[2] > 10);
}

// ---------------------------------------------------------------------------------------------------------------------
// Edges

void degenerateInput() {
    // All joints on one point: no segment has a direction. No NaN, whatever the solver.
    for (const Algo algo : kAll) {
        Chain c = makeUniformChain({1, 1}, 4, 1.0);
        for (int i = 0; i <= 4; ++i) c.p[static_cast<std::size_t>(i)] = {1, 1};
        for (const Vec2 target : {Vec2{3, 1}, Vec2{1, 1}, Vec2{1, 1.5}, Vec2{-1e6, 3e5}}) {
            Chain w = c;
            for (int i = 0; i < 30; ++i) iterate(algo, w, target);
            assert(finite(w));
            assert(near(w.p[0], {1, 1}, 0.0));
        }
    }
    // The target on the root and on a joint.
    for (const Algo algo : kAll) {
        Chain c = makeUniformChain({0, 0}, 5, 1.0, 0.2);
        const Vec2 onJoint = c.p[2];
        for (int i = 0; i < 30; ++i) iterate(algo, c, onJoint);
        assert(finite(c));
        for (const Vec2 target : {Vec2{0, 0}, Vec2{1e-12, 0}}) {
            Chain w = c;
            for (int i = 0; i < 30; ++i) iterate(algo, w, target);
            assert(finite(w));
        }
    }
    // A chain without any length: the target is out of reach for every solver, still no NaN.
    for (const Algo algo : kAll) {
        Chain z = makeChain({2, 2}, {0.0, 0.0});
        for (const Vec2 target : {Vec2{2, 2}, Vec2{5, 1}}) {
            for (int i = 0; i < 5; ++i) iterate(algo, z, target);
            assert(finite(z));
        }
    }
    // FABRIK folds a chain onto the root without a NaN: target on the root, equal lengths, minReach 0.
    Chain f = makeUniformChain({0, 0}, 3, 1.0, 0.3);
    const Result r = solve(Algo::Fabrik, f, {0, 0}, 0.01, 2000);
    assert(finite(f) && r.outcome == Outcome::Reached);
}

void solveStopsWhereItShould() {
    Chain c = makeUniformChain({0, 0}, 10, 1.0, 0.12);
    const Result limit = solve(Algo::Ccd, c, {-4, 6}, 0.001, 3);
    assert(limit.outcome == Outcome::Limit && limit.iterations == 3);
    assert(near(limit.error, tipError(c, {-4, 6})));

    Chain there = makeUniformChain({0, 0}, 2, 1.0);
    const Result zero = solve(Algo::Fabrik, there, tip(there), 0.001, 10);
    assert(zero.outcome == Outcome::Reached && zero.iterations == 0);

    // A target in the hole of an unequal chain: the nearest point of the ring is the answer.
    Chain h = makeChain({0, 0}, {3, 4});
    const Result inHole = solve(Algo::Fabrik, h, {0.0, 0.0}, 0.01, 200);
    assert(finite(h));
    assert(inHole.outcome == Outcome::Reached && near(inHole.error, 1.0, 0.01));
}

void maxShiftChecks() {
    Chain a = makeUniformChain({0, 0}, 3, 1.0);
    Chain b = a;
    assert(maxShift(a.p, b.p, 3) == 0.0);
    b.p[2] = b.p[2] + Vec2{0.0, 0.5};
    assert(near(maxShift(a.p, b.p, 3), 0.5));
    assert(near(maxShift(a.p, b.p, 1), 0.0));  // only the first two joints are compared
}

void traceSizes() {
    const Chain rest = makeUniformChain({0, 0}, 10, 1.0, 0.12);
    Trace t;
    Chain a = rest;
    ccdIteration(a, {-4, 6}, &t);
    assert(t.count == 11);  // before + one per joint
    assert(near(t.poses[0][5], rest.p[5], 0.0));
    for (int i = 0; i <= 10; ++i) assert(near(t.poses[10][static_cast<std::size_t>(i)], a.p[static_cast<std::size_t>(i)], 0.0));
    Chain b = rest;
    fabrikIteration(b, {-4, 6}, &t);
    assert(t.count == 3);
    Chain c = rest;
    relaxIteration(c, {-4, 6}, &t);
    assert(t.count == 12);  // before, pinned, one per segment
    Chain big = makeUniformChain({0, 0}, kMaxSegments, 1.0, 0.1);
    relaxIteration(big, {-4, 6}, &t);
    assert(t.count == kMaxSegments + 2 && t.count <= Trace::kMax);
    // A second call starts a fresh trace.
    fabrikIteration(big, {-4, 6}, &t);
    assert(t.count == 3);
}

// ---------------------------------------------------------------------------------------------------------------------
// The demo path

void autoPath() {
    const Vec2 rootPos{100, 200};
    const double L = 200.0;
    // The spots, as (angle, fraction of the reach): the target sits on each for kAutoHold seconds, then jumps.
    const double spots[kAutoSpots][2] = {{-0.6, 0.85}, {2.2, 0.7}, {-2.2, 0.95}, {0.5, 1.3}, {-1.6, 0.6}, {kPi, 1.25}, {1.2, 0.9}, {-0.2, 1.15}};
    int inside = 0;
    int outside = 0;
    for (int i = 0; i < kAutoSpots; ++i) {
        const Vec2 expected = rootPos + Vec2{std::cos(spots[i][0]), std::sin(spots[i][0])} * (L * spots[i][1]);
        const double start = i * kAutoHold;
        for (const double offset : {0.0, 0.5 * kAutoHold, kAutoHold - 1e-6}) {
            assert(near(autoTarget(rootPos, L, start + offset), expected, 1e-9));  // it does not move while it holds
        }
        const double r = length(expected - rootPos);
        (r < L ? inside : outside)++;
    }
    assert(inside == 5 && outside == 3);
    assert(length(autoTarget(rootPos, L, 0.0) - rootPos) < L);  // the first frame is reachable
    assert(!near(autoTarget(rootPos, L, kAutoHold - 1e-6), autoTarget(rootPos, L, kAutoHold), 1.0));  // and then it jumps
    assert(near(autoTarget(rootPos, L, kAutoPeriod - 1e-9), autoTarget(rootPos, L, kAutoPeriod - 0.5 * kAutoHold), 1e-9));
    assert(near(kAutoPeriod, kAutoSpots * kAutoHold));
    // Phases outside the period, from rounding or a caller that forgot to wrap, stay on the first and the last spot.
    assert(near(autoTarget(rootPos, L, -0.1), autoTarget(rootPos, L, 0.0)));
    assert(near(autoTarget(rootPos, L, std::nan("")), autoTarget(rootPos, L, 0.0)));
    assert(near(autoTarget(rootPos, L, 1e300), autoTarget(rootPos, L, kAutoPeriod - 1e-9)));
    assert(near(autoTarget(rootPos, L, kAutoPeriod + 5.0), autoTarget(rootPos, L, kAutoPeriod - 1e-9)));
}

void phaseWraps() {
    assert(near(advancePhase(15.99, 0.02), 0.01, 1e-9));
    assert(near(advancePhase(0.0, 0.25), 0.25));
    const double big = advancePhase(3.0, 27.0);  // dt is clamped by the caller, the function must not trust that
    assert(big >= 0.0 && big < kAutoPeriod && near(big, 14.0, 1e-9));
    assert(advancePhase(5.0, 0.0) == 5.0);
}

}  // namespace

int main() {
    chainBasics();
    errors();
    ccdByHand();
    ccdFrozenOnTheAxis();
    fabrikByHand();
    fabrikStretchesInOneStep();
    relaxByHand();
    everyIterationKeepsTheRules();
    relaxStretchesBeyondTheReach();
    reachableTargets();
    unreachableTargets();
    unreachableEndsStraight();
    ccdCurlsUp();
    stuckPreset();
    degenerateInput();
    solveStopsWhereItShould();
    maxShiftChecks();
    traceSizes();
    autoPath();
    phaseWraps();
    std::puts("fabrik-vs-ccd logic: all asserts passed");
    return 0;
}
