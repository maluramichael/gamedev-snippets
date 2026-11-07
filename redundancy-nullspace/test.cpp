// Tests for the null space IK logic. Plain asserts on hand-computed cases, no framework, no SDL.
// Math coordinates throughout (y points up). The pose kBent with the lengths kHandLen keeps the hand calculations short,
// the demo arm (nsp::kLengths) is used where a property has to hold for the real thing.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "logic.hpp"

using namespace nsp;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr Lengths kHandLen{100.0, 85.0, 65.0, 50.0};
constexpr Lengths kUnit{1.0, 1.0, 1.0, 1.0};

bool near(double a, double b, double tol = 1e-9) { return std::fabs(a - b) < tol; }

bool nearVec(Vec2 a, Vec2 b, double tol = 1e-9) { return near(a.x, b.x, tol) && near(a.y, b.y, tol); }

// The pose [0, 90 deg, -90 deg, 0] of kHandLen: the first segment points right, the second up, the last two right again.
// Joints at (100,0), (100,85), (165,85), the hand at (215,85).
const Angles kBent{0.0, kPi / 2.0, -kPi / 2.0, 0.0};

// Small deterministic generator, so the "random" poses are the same on every machine.
struct Lcg {
    unsigned long long s = 12345ULL;
    double next() {  // [0, 1)
        s = s * 6364136223846793005ULL + 1442695040888963407ULL;
        return static_cast<double>(s >> 11) / 9007199254740992.0;
    }
    double range(double lo, double hi) { return lo + (hi - lo) * next(); }
};

Angles randomPose(Lcg& rng) {
    Angles t{};
    for (std::size_t i = 0; i < kJoints; ++i) t[i] = rng.range(kLimitLo[i], kLimitHi[i]);
    return t;
}

double handError(const Angles& theta, Vec2 target) {
    const Vec2 p = endEffector(kLengths, theta);
    return std::hypot(target.x - p.x, target.y - p.y);
}

double dot(const Angles& a, const Angles& b) {
    double s = 0.0;
    for (std::size_t i = 0; i < kJoints; ++i) s += a[i] * b[i];
    return s;
}

struct Run {
    double maxError = 0.0;
    double finalError = 0.0;
    double maxJointChange = 0.0;  // largest change of any joint angle compared to the start
    Angles end{};
};

/// The hand sits on the start target, the arm runs `steps` iterations with one secondary goal.
Run runFromStart(Secondary mode, bool projected, int steps) {
    Angles theta = startPose();
    const Angles start = theta;
    Run r;
    for (int i = 0; i < steps; ++i) {
        iterate(theta, kLengths, kStartTarget, mode, projected, Angles{});
        r.finalError = handError(theta, kStartTarget);
        r.maxError = std::max(r.maxError, r.finalError);
    }
    for (std::size_t i = 0; i < kJoints; ++i) r.maxJointChange = std::max(r.maxJointChange, std::fabs(theta[i] - start[i]));
    r.end = theta;
    return r;
}

void forwardKinematics() {
    assert(nearVec(endEffector(kUnit, {0, 0, 0, 0}), {4, 0}));
    assert(nearVec(endEffector(kUnit, {kPi / 2, 0, 0, 0}), {0, 4}, 1e-12));
    // Joint positions: base at the origin, then one per segment end. The angles add up along the chain.
    const auto p = joints(kHandLen, kBent);
    assert(nearVec(p[0], {0, 0}) && nearVec(p[1], {100, 0}, 1e-12) && nearVec(p[2], {100, 85}, 1e-12));
    assert(nearVec(p[3], {165, 85}, 1e-12) && nearVec(p[4], {215, 85}, 1e-12));
    assert(near(reach(kLengths), 260.0));
    assert(nearVec(endEffector(kLengths, {0, 0, 0, 0}), {260, 0}));
}

void jacobianByHand() {
    // Column i is the perpendicular to the line from joint i to the hand: (-dy, dx).
    const Jacobian j = jacobian(kHandLen, kBent);
    assert(nearVec(j.col[0], {-85, 215}, 1e-9) && nearVec(j.col[1], {-85, 115}, 1e-9));
    assert(nearVec(j.col[2], {0, 115}, 1e-9) && nearVec(j.col[3], {0, 50}, 1e-9));
    // Jacobian of the second joint (100,85): only joints 0 and 1 are in front of it.
    const Jacobian e = jacobianAt(kHandLen, kBent, 2);
    assert(nearVec(e.col[0], {-85, 100}, 1e-9) && nearVec(e.col[1], {-85, 0}, 1e-9));
    assert(nearVec(e.col[2], {0, 0}) && nearVec(e.col[3], {0, 0}));
    // Straight arm along +x: every column is (0, distance to the hand), J has rank 1.
    const Jacobian s = jacobian(kUnit, {0, 0, 0, 0});
    assert(nearVec(s.col[0], {0, 4}) && nearVec(s.col[1], {0, 3}) && nearVec(s.col[2], {0, 2}) && nearVec(s.col[3], {0, 1}));
}

void jacobianMatchesFiniteDifferences() {
    Lcg rng;
    for (int n = 0; n < 200; ++n) {
        const Angles t = randomPose(rng);
        for (std::size_t k = 1; k <= kJoints; ++k) {
            const Jacobian a = jacobianAt(kLengths, t, k);
            const Jacobian b = numericJacobianAt(kLengths, t, k);
            for (std::size_t i = 0; i < kJoints; ++i) assert(nearVec(a.col[i], b.col[i], 1e-5));
        }
    }
}

void matrixInverse() {
    Mat2 inv;
    assert(inverse(Mat2{2, 0, 0, 4}, inv));
    assert(near(inv.a, 0.5) && near(inv.d, 0.25) && near(inv.b, 0.0) && near(inv.c, 0.0));
    assert(inverse(Mat2{1, 2, 3, 4}, inv));
    assert(near(inv.a, -2.0) && near(inv.b, 1.0) && near(inv.c, 1.5) && near(inv.d, -0.5));
    assert(!inverse(Mat2{1, 2, 2, 4}, inv));  // det 0
    assert(!inverse(Mat2{0, 0, 0, 0}, inv));
    // The guard is relative: a small but well conditioned matrix still inverts.
    assert(inverse(Mat2{1e-6, 0, 0, 1e-6}, inv));
}

void singularValuesAndDimensions() {
    // Straight arm of the demo: J = columns (0, 260) (0, 180) (0, 110) (0, 50), sigma_max = sqrt(114600), sigma_min = 0.
    const Jacobian straight = jacobian(kLengths, {0, 0, 0, 0});
    const SingularValues s = singularValues(straight);
    assert(near(s.max, std::sqrt(114600.0), 1e-9) && near(s.min, 0.0, 1e-9));
    assert(nullDimensions(straight) == 3);
    // A bent arm has rank 2: two free directions for four joints.
    const Jacobian bent = jacobian(kHandLen, kBent);
    assert(nullDimensions(bent) == 2);
    // Hand check: J J^T = [[14450, -28050], [-28050, 75175]], det = 299476250, trace = 89625.
    const Mat2 g = gram(bent);
    assert(near(g.a, 14450.0) && near(g.b, -28050.0) && near(g.c, -28050.0) && near(g.d, 75175.0));
    const SingularValues sb = singularValues(bent);
    assert(near(sb.max * sb.max * sb.min * sb.min, 299476250.0, 1e-3));
    assert(near(sb.max * sb.max + sb.min * sb.min, 89625.0, 1e-6));
}

void pseudoInverseByHand() {
    const Jacobian j = jacobian(kHandLen, kBent);
    const Vec2 e{10.0, 20.0};
    const Angles d = pinvTimes(j, e);
    // J d = e: the step does what it promises.
    assert(nearVec(times(j, d), e, 1e-9));
    // And it is the shortest such step: it has no component along the null vector n = (1, -1, 0, -2).
    // (J n = (-85 + 85, 215 - 115 + 0 - 100) = (0, 0), checked by hand.)
    const Angles n{1.0, -1.0, 0.0, -2.0};
    assert(nearVec(times(j, n), {0.0, 0.0}, 1e-9));
    assert(near(dot(d, n), 0.0, 1e-9));
}

void dampedStepByHand() {
    // Straight demo arm, error (0, 10), lambda 1: J J^T + I = diag(1, 114601), so d_i = col_i.y * 10 / 114601.
    const Jacobian j = jacobian(kLengths, {0, 0, 0, 0});
    const Angles d = pinvTimes(j, {0.0, 10.0}, 1.0);
    assert(near(d[0], 2600.0 / 114601.0, 1e-12) && near(d[1], 1800.0 / 114601.0, 1e-12));
    assert(near(d[2], 1100.0 / 114601.0, 1e-12) && near(d[3], 500.0 / 114601.0, 1e-12));
    // Along the arm there is no step to take: nothing in J points that way.
    const Angles along = pinvTimes(j, {10.0, 0.0}, 1.0);
    assert(norm(along) == 0.0);
    // Exactly singular without damping: no inverse, so the zero step instead of a division by 0.
    assert(norm(pinvTimes(j, {0.0, 10.0}, 0.0)) == 0.0);
}

void primaryDamping() {
    assert(near(primaryLambda(100.0), 0.0) && near(primaryLambda(kSigmaFade), 0.0));
    assert(near(primaryLambda(0.0), kSigmaFade) && near(primaryLambda(kSigmaFade / 2.0), kSigmaFade / 2.0));
}

void projectorByHand() {
    const Jacobian j = jacobian(kHandLen, kBent);
    const Angles n{1.0, -1.0, 0.0, -2.0};
    // A null vector goes through the projector untouched.
    const Angles pn = project(j, n);
    for (std::size_t i = 0; i < kJoints; ++i) assert(near(pn[i], n[i], 1e-5));
    // A vector from the row space (J^T w) is removed completely.
    const Angles row = transposeTimes(j, {3.0, -2.0});
    assert(norm(row) > 100.0);
    assert(norm(project(j, row)) < 1e-4);
    // Projecting twice changes nothing, and the projector is symmetric: <P a, b> = <a, P b>.
    const Angles a{0.3, -1.1, 0.7, 0.2};
    const Angles b{-0.5, 0.4, 0.9, -0.8};
    const Angles pa = project(j, a);
    const Angles ppa = project(j, pa);
    for (std::size_t i = 0; i < kJoints; ++i) assert(near(pa[i], ppa[i], 1e-5));
    assert(near(dot(pa, b), dot(a, project(j, b)), 1e-5));
    // The part that stays is never longer than the wish.
    assert(norm(pa) <= norm(a) + 1e-9);
}

void projectedWishDoesNotMoveTheHand() {
    // J (I - J+ J) phi = 0 for random poses and random wishes. The projector is damped by 0.01, which leaves at most
    // lambda^2 / sigma_min = 2e-5 per unit of phi at sigma_min = 5. A raw wish moves the hand by tens of pixels.
    Lcg rng;
    int checked = 0;
    double rawMove = 0.0;
    for (int n = 0; n < 600; ++n) {
        const Angles t = randomPose(rng);
        const Jacobian j = jacobian(kLengths, t);
        if (singularValues(j).min < 5.0) continue;  // the check below needs a well conditioned J
        Angles phi{};
        for (double& v : phi) v = rng.range(-1.0, 1.0);
        assert(norm(times(j, project(j, phi))) < 1e-4);
        rawMove += norm(times(j, phi));
        ++checked;
    }
    assert(checked > 300);
    assert(rawMove / checked > 30.0);
    // At the exactly stretched pose J has rank 1, the projector still removes the one direction the hand can move in.
    const Jacobian straight = jacobian(kLengths, {0, 0, 0, 0});
    assert(norm(times(straight, project(straight, {1.0, 2.0, 3.0, 4.0}))) < 1e-4);
}

void nullStepKeepsTheHand() {
    // Second order only: a pure null space step of the demo pose moves the hand by under a pixel, the raw step by many.
    const Angles start = startPose();
    Angles t = start;
    const Vec2 before = endEffector(kLengths, t);
    const StepInfo info = iterate(t, kLengths, before, Secondary::Rest, true, Angles{});
    assert(info.self > 0.05);
    assert(near(info.step, info.self, 1e-6));  // the whole step is self motion
    const Vec2 after = endEffector(kLengths, t);
    assert(std::hypot(after.x - before.x, after.y - before.y) < 0.5);

    Angles u = start;
    iterate(u, kLengths, before, Secondary::Rest, false, Angles{});
    const Vec2 naive = endEffector(kLengths, u);
    assert(std::hypot(naive.x - before.x, naive.y - before.y) > 5.0);
}

void primaryStepReachesTheError() {
    // 5 px to the right with no second goal: the error after one step is the second order remainder only.
    Angles t = startPose();
    const Vec2 h = endEffector(kLengths, t);
    const Vec2 target{h.x + 5.0, h.y};
    iterate(t, kLengths, target, Secondary::Off, true, Angles{});
    assert(handError(t, target) < 0.5);
    // Without a second goal and without error nothing moves at all.
    Angles s = startPose();
    const Angles before = s;
    const StepInfo info = iterate(s, kLengths, h, Secondary::Off, true, Angles{});
    assert(info.step < 1e-9 && info.self < 1e-9);
    for (std::size_t i = 0; i < kJoints; ++i) assert(near(s[i], before[i], 1e-9));
}

void clampAndWrap() {
    assert(nearVec(clampMag({3.0, 4.0}, 10.0), {3.0, 4.0}));
    assert(nearVec(clampMag({30.0, 40.0}, 5.0), {3.0, 4.0}));
    assert(nearVec(clampMag({3.0, 4.0}, 0.0), {0.0, 0.0}));
    assert(nearVec(clampMag({0.0, 0.0}, 5.0), {0.0, 0.0}));
    const Angles c = clampNorm({3.0, 4.0, 0.0, 0.0}, 1.0);
    assert(near(c[0], 0.6) && near(c[1], 0.8) && near(norm(c), 1.0));
    assert(norm(clampNorm({3.0, 4.0, 0.0, 0.0}, 0.0)) == 0.0);
    assert(near(clampNorm({0.3, 0.0, 0.0, 0.0}, 1.0)[0], 0.3));
    assert(near(wrapAngle(3.0 * kPi), kPi, 1e-12) || near(wrapAngle(3.0 * kPi), -kPi, 1e-12));
    assert(near(wrapAngle(0.5), 0.5) && near(wrapAngle(2.0 * kPi + 0.5), 0.5, 1e-12));
    assert(near(wrapAngle(-2.0 * kPi - 0.5), -0.5, 1e-12));
}

void secondaryGoalsByHand() {
    // Rest: nothing at the rest pose, 5 % of the way back otherwise.
    assert(norm(phiRest(kRest)) == 0.0);
    Angles off = kRest;
    off[0] += 1.0;
    off[2] -= 0.4;
    const Angles r = phiRest(off);
    assert(near(r[0], -0.05) && near(r[1], 0.0) && near(r[2], 0.02) && near(r[3], 0.0));

    // Limit positions: joint 0 spans [-0.5, 2.5], middle 1.0, half range 1.5.
    assert(near(limitPosition({1.0, 1.3, -1.2, 0.0}, 0), 0.0));
    assert(near(limitPosition({2.5, 0.0, 0.0, 1.5}, 0), 1.0) && near(limitPosition({2.5, 0.0, 0.0, 1.5}, 1), -1.0));
    assert(near(limitPosition({2.5, 0.0, 0.0, 1.5}, 2), 1.0) && near(limitPosition({2.5, 0.0, 0.0, 1.5}, 3), 1.0));
    assert(near(worstLimitUse({1.0, 1.3, -1.2, 0.0}), 0.0));
    assert(near(worstLimitUse({1.0, 1.3, -1.2, 1.5}), 1.0));
    assert(near(worstLimitUse({3.25, 1.3, -1.2, 0.0}), 1.5));  // beyond the limit counts more than 1

    // Limits: dead zone in the middle, nothing exactly at half way, then a push back that grows with the use.
    assert(norm(phiLimits({1.0, 1.3, -1.2, 0.0})) == 0.0);
    assert(norm(phiLimits({1.0 + 0.75, 1.3, -1.2, 0.0})) == 0.0);  // u = 0.5 exactly
    const Angles atHi = phiLimits({2.5, 1.3, -1.2, 0.0});           // u = 1: push 1, gain 0.08 * half range 1.5
    assert(near(atHi[0], -0.12) && near(atHi[1], 0.0));
    const Angles atLo = phiLimits({1.0, 0.0, -1.2, 0.0});           // joint 1: u = -1, half range 1.3
    assert(near(atLo[1], 0.08 * 1.3));
    const Angles beyond = phiLimits({4.0, 1.3, -1.2, 0.0});         // u = 2: push (2 - 0.5) / 0.5 = 3
    assert(near(beyond[0], -0.08 * 1.5 * 3.0));
    const Angles quarter = phiLimits({1.0 + 1.5 * 0.75, 1.3, -1.2, 0.0});  // u = 0.75: push 0.5
    assert(near(quarter[0], -0.08 * 1.5 * 0.5));

    // Comfort: zero at 0, pulls towards 0 from both sides with the same size, never more than the gain.
    assert(norm(phiComfort({0, 0, 0, 0})) == 0.0);
    const Angles c = phiComfort({1.0, -1.0, 0.5, -0.05});
    assert(near(c[0], -0.03 / std::hypot(1.0, 0.05), 1e-12) && near(c[1], -c[0]));
    assert(near(c[3], 0.03 / std::sqrt(2.0), 1e-12));  // the corner: angle -0.05 = the softening, 1/sqrt(2) of the gain
    assert(std::fabs(phiComfort({100.0, 0, 0, 0})[0]) < 0.03);
    assert(near(meanAbsAngle({1.0, -1.0, 0.5, -0.5}), 0.75));

    assert(norm(phiFor(Secondary::Off, off)) == 0.0);
    assert(near(phiFor(Secondary::Rest, off)[0], r[0]));
}

void startPoseIsOnTheTarget() {
    const Angles s = startPose();
    assert(handError(s, kStartTarget) < 1e-6);
    // It is bent far from the rest pose and outside a limit, so every second goal has something to do right away.
    Angles d{};
    for (std::size_t i = 0; i < kJoints; ++i) d[i] = s[i] - kRest[i];
    assert(norm(d) > 2.0);
    assert(worstLimitUse(s) > 1.0);
}

void offKeepsTheArmWhereItIs() {
    // No second goal: the arm stays in whatever self-motion state it is in, whatever happens.
    const Run r = runFromStart(Secondary::Off, true, 300);
    assert(r.maxJointChange < 1e-9);
}

void restPullsToTheRestPose() {
    const Run r = runFromStart(Secondary::Rest, true, 300);
    // The hand stays on the target the whole time (under half a pixel), the joints move by over a radian.
    assert(r.maxError < 0.5);
    assert(r.finalError < 1e-3);
    assert(r.maxJointChange > 1.0);
    Angles d{};
    Angles d0{};
    const Angles s = startPose();
    for (std::size_t i = 0; i < kJoints; ++i) {
        d[i] = r.end[i] - kRest[i];
        d0[i] = s[i] - kRest[i];
    }
    assert(norm(d) < 0.6 * norm(d0));
}

void forgettingTheProjectionPullsTheHandAway() {
    const Run good = runFromStart(Secondary::Rest, true, 300);
    const Run naive = runFromStart(Secondary::Rest, false, 300);
    assert(good.finalError < 1e-3);
    assert(naive.finalError > 3.0);
    assert(naive.maxError > 5.0);
}

void limitsMovesAJointBackIntoItsRange() {
    const double before = worstLimitUse(startPose());
    const Run r = runFromStart(Secondary::Limits, true, 300);
    assert(before > 1.0);
    assert(worstLimitUse(r.end) < 0.7);
    assert(r.maxError < 0.5 && r.finalError < 1e-3);
}

void comfortLowersTheMeanAngle() {
    const double before = meanAbsAngle(startPose());
    const Run r = runFromStart(Secondary::Comfort, true, 300);
    assert(meanAbsAngle(r.end) < 0.8 * before);
    assert(r.maxError < 0.5 && r.finalError < 1e-3);
}

void shakeMovesTheArmNotTheHand() {
    const Angles k = kickDirection(1);
    assert(near(norm(k), 1.0, 1e-12));
    assert(norm(kickDirection(2)) > 0.99 && dot(kickDirection(1), kickDirection(2)) < 0.999);
    assert(near(dot(k, kickDirection(1)), 1.0, 1e-12));  // same seed, same direction
    Angles extra = k;
    for (double& v : extra) v *= kKickGain;
    Angles t = startPose();
    const Angles before = t;
    double maxError = 0.0;
    for (int i = 0; i < kKickSteps; ++i) {
        iterate(t, kLengths, kStartTarget, Secondary::Off, true, extra);
        maxError = std::max(maxError, handError(t, kStartTarget));
    }
    double change = 0.0;
    for (std::size_t i = 0; i < kJoints; ++i) change = std::max(change, std::fabs(t[i] - before[i]));
    assert(change > 0.5);
    assert(maxError < 0.5);
}

void grabbingAnElbowKeepsTheHand() {
    Angles t = startPose();
    const auto p = joints(kLengths, t);
    const Vec2 to{p[2].x, p[2].y + 30.0};
    const double before = std::hypot(to.x - p[2].x, to.y - p[2].y);
    double maxError = 0.0;
    for (int i = 0; i < 90; ++i) {
        iterate(t, kLengths, kStartTarget, Secondary::Off, true, grabPhi(kLengths, t, 2, to));
        maxError = std::max(maxError, handError(t, kStartTarget));
    }
    const auto q = joints(kLengths, t);
    assert(before == 30.0);
    assert(std::hypot(to.x - q[2].x, to.y - q[2].y) < 8.0);
    assert(maxError < 1.0);
    // Joint 1 sits on a circle around the base, the grab cannot take it off the circle.
    assert(near(norm(q[1]), kLengths[0], 1e-9));
}

void stretchedArmStaysFinite() {
    // Target a bit beyond the reach: the arm is at its singularity, nothing may blow up.
    Angles t{};
    const Vec2 target{262.0, 10.0};
    const double start = handError(t, target);
    for (int i = 0; i < 200; ++i) {
        const StepInfo info = iterate(t, kLengths, target, Secondary::Rest, true, Angles{});
        assert(std::isfinite(info.step) && info.step <= kMaxJointStep + 1e-12);
        for (const double v : t) assert(std::isfinite(v) && std::fabs(v) <= kPi + 1e-12);
    }
    assert(handError(t, target) < start);
}

void autopilotStaysInRange() {
    Autopilot a;
    double lo = 1e9;
    double hi = 0.0;
    for (int i = 0; i < 4000; ++i) {
        a.advance(0.05);
        assert(a.phase >= 0.0 && a.phase < 2.0 * kPi);
        lo = std::min(lo, norm(a.target()));
        hi = std::max(hi, norm(a.target()));
    }
    assert(hi < 0.85 * reach(kLengths));  // never near the stretched pose
    assert(lo > 30.0);
    const double before = a.phase;
    a.advance(-1.0);
    assert(a.phase == before);
    // Same period after a very long run.
    Autopilot b;
    for (int i = 0; i < 2000000; ++i) b.advance(1.0 / 60.0);
    assert(b.phase >= 0.0 && b.phase < 2.0 * kPi);
}

}  // namespace

int main() {
    forwardKinematics();
    jacobianByHand();
    jacobianMatchesFiniteDifferences();
    matrixInverse();
    singularValuesAndDimensions();
    pseudoInverseByHand();
    dampedStepByHand();
    primaryDamping();
    projectorByHand();
    projectedWishDoesNotMoveTheHand();
    nullStepKeepsTheHand();
    primaryStepReachesTheError();
    clampAndWrap();
    secondaryGoalsByHand();
    startPoseIsOnTheTarget();
    offKeepsTheArmWhereItIs();
    restPullsToTheRestPose();
    forgettingTheProjectionPullsTheHandAway();
    limitsMovesAJointBackIntoItsRange();
    comfortLowersTheMeanAngle();
    shakeMovesTheArmNotTheHand();
    grabbingAnElbowKeepsTheHand();
    stretchedArmStaysFinite();
    autopilotStaysInRange();
    std::puts("redundancy-nullspace logic: all asserts passed");
    return 0;
}
