// Tests for the Jacobian IK logic. Plain asserts on hand-computed cases, no framework, no SDL.
// Math coordinates throughout (y points up). Unit lengths {1, 1, 1} keep the hand calculations short, the demo arm
// (jac::kLengths) is used where a property has to hold for the real thing.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "logic.hpp"

using namespace jac;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr Lengths kUnit{1.0, 1.0, 1.0};

bool near(double a, double b, double tol = 1e-9) { return std::fabs(a - b) < tol; }

bool nearVec(Vec2 a, Vec2 b, double tol = 1e-9) { return near(a.x, b.x, tol) && near(a.y, b.y, tol); }

// The pose [0, 90 deg, 0] of the unit arm: the first segment points right, the other two point up. End at (1, 2).
const Angles kBent{0.0, kPi / 2.0, 0.0};

// Small deterministic generator, so the "random" poses are the same on every machine.
struct Lcg {
    unsigned long long s = 12345ULL;
    double next() {  // [0, 1)
        s = s * 6364136223846793005ULL + 1442695040888963407ULL;
        return static_cast<double>(s >> 11) / 9007199254740992.0;
    }
    double range(double lo, double hi) { return lo + (hi - lo) * next(); }
};

Vec2 errorTo(const Lengths& len, const Angles& theta, Vec2 target) {
    const Vec2 p = endEffector(len, theta);
    return {target.x - p.x, target.y - p.y};
}

void forwardKinematics() {
    assert(nearVec(endEffector(kUnit, {0, 0, 0}), {3, 0}));
    assert(nearVec(endEffector(kUnit, {kPi / 2, 0, 0}), {0, 3}, 1e-12));
    assert(nearVec(endEffector(kUnit, kBent), {1, 2}, 1e-12));
    // Joint positions: base at the origin, then one per segment end. The angles add up along the chain.
    const auto j = joints(kUnit, kBent);
    assert(nearVec(j[0], {0, 0}) && nearVec(j[1], {1, 0}, 1e-12) && nearVec(j[2], {1, 1}, 1e-12) && nearVec(j[3], {1, 2}, 1e-12));
    assert(near(reach(kUnit), 3.0));
    assert(near(reach(kLengths), 250.0));
    // A segment of length 0 adds nothing.
    assert(nearVec(endEffector({1.0, 0.0, 1.0}, {0, 1.0, -1.0}), {2, 0}));
}

void jacobianByHand() {
    // Straight arm along +x, end at (3,0): every column is (0, distance from joint to end), so J has rank 1.
    const Jacobian straight = jacobian(kUnit, {0, 0, 0});
    assert(nearVec(straight.col[0], {0, 3}) && nearVec(straight.col[1], {0, 2}) && nearVec(straight.col[2], {0, 1}));
    // Straight up, end at (0,3): the columns turn to (-distance, 0).
    const Jacobian up = jacobian(kUnit, {kPi / 2, 0, 0});
    assert(nearVec(up.col[0], {-3, 0}, 1e-12) && nearVec(up.col[1], {-2, 0}, 1e-12) && nearVec(up.col[2], {-1, 0}, 1e-12));
    // Bent pose: joints (0,0), (1,0), (1,1), end (1,2). Column = perpendicular of (end - joint) = (-dy, dx).
    const Jacobian bent = jacobian(kUnit, kBent);
    assert(nearVec(bent.col[0], {-2, 1}, 1e-12) && nearVec(bent.col[1], {-2, 0}, 1e-12) && nearVec(bent.col[2], {-1, 0}, 1e-12));
}

void jacobianMatchesFiniteDifferences() {
    Lcg rng;
    for (int i = 0; i < 200; ++i) {
        const Angles theta{rng.range(-kPi, kPi), rng.range(-kPi, kPi), rng.range(-kPi, kPi)};
        const Jacobian a = jacobian(kLengths, theta);
        const Jacobian n = numericJacobian(kLengths, theta);
        for (std::size_t c = 0; c < 3; ++c) {
            assert(nearVec(a.col[c], n.col[c], 1e-5));
        }
    }
    // The check has teeth: a Jacobian with a swapped column does not match.
    const Angles theta{0.4, -0.9, 1.3};
    Jacobian wrong = jacobian(kLengths, theta);
    std::swap(wrong.col[0], wrong.col[2]);
    assert(!nearVec(wrong.col[0], numericJacobian(kLengths, theta).col[0], 1e-3));
}

void matrixInverse() {
    // [[9,-2],[-2,1]] has det 5 and inverse 1/5 [[1,2],[2,9]].
    const auto inv = inverse({9, -2, -2, 1});
    assert(inv.has_value());
    assert(near(inv->a, 0.2) && near(inv->b, 0.4) && near(inv->c, 0.4) && near(inv->d, 1.8));
    // Singular: a zero row, and a row that is twice the other.
    assert(!inverse({0, 0, 0, 14}).has_value());
    assert(!inverse({1, 2, 2, 4}).has_value());
    assert(!inverse({0, 0, 0, 0}).has_value());
    // The guard is relative: a tiny but well conditioned matrix is still invertible, a huge nearly singular one is not.
    assert(inverse({1e-6, 0, 0, 1e-6}).has_value());
    assert(!inverse({1e6, 1e6, 1e6, 1e6 * (1.0 + 1e-14)}).has_value());
    // Just regular, not singular, near the edge of the guard: still returns something finite.
    const auto thin = inverse({1.0, 0.0, 0.0, 1e-4});
    assert(thin.has_value() && near(thin->d, 1e4, 1e-6));
}

void gramMatrixAndSingularValues() {
    const Jacobian bent = jacobian(kUnit, kBent);
    const Mat2 g = gram(bent);  // J J^T = [[9,-2],[-2,1]]
    assert(near(g.a, 9, 1e-12) && near(g.b, -2, 1e-12) && near(g.c, -2, 1e-12) && near(g.d, 1, 1e-12));
    // Eigenvalues of that matrix: 5 +- sqrt(20), the singular values are their square roots.
    const SingularValues sv = singularValues(bent);
    assert(near(sv.max, std::sqrt(5.0 + std::sqrt(20.0)), 1e-9));
    assert(near(sv.min, std::sqrt(5.0 - std::sqrt(20.0)), 1e-9));
    // Straight arm: rank 1, JJ^T = [[0,0],[0,14]], so sigma_min is exactly 0 and sigma_max = sqrt(14).
    const SingularValues flat = singularValues(jacobian(kUnit, {0, 0, 0}));
    assert(flat.min == 0.0 && near(flat.max, std::sqrt(14.0), 1e-12));
    // sigma_min is a product-stable value: tiny bend, tiny sigma_min, no negative square root.
    const SingularValues almost = singularValues(jacobian(kLengths, {0.0, 1e-7, 0.0}));
    assert(almost.min >= 0.0 && almost.min < 1e-4 && almost.max > 100.0);
}

void weakDirectionIsRadialWhenStretched() {
    // At the stretched pose pointing along +x the hard direction of the end effector is +x (or -x): sideways is easy.
    const Vec2 w = weakDirection(jacobian(kLengths, {0.0, 0.01, 0.0}));
    assert(near(std::hypot(w.x, w.y), 1.0, 1e-12));
    assert(std::fabs(w.x) > 0.999);
    // And it is an eigenvector with the smallest singular value: J^T w has length sigma_min.
    const Jacobian j = jacobian(kLengths, {0.3, -0.5, 0.8});
    const Vec2 u = weakDirection(j);
    double sx = 0.0;
    for (const Vec2& c : j.col) sx += (c.x * u.x + c.y * u.y) * (c.x * u.x + c.y * u.y);
    assert(near(std::sqrt(sx), singularValues(j).min, 1e-9));
}

void transposeStepByHand() {
    // Bent pose, e = (1,0): J^T e = (-2,-2,-1), J J^T e = (9,-2), alpha = <e,JJ^T e> / <JJ^T e,JJ^T e> = 9/85.
    const Jacobian j = jacobian(kUnit, kBent);
    const Angles d = stepTranspose(j, {1, 0});
    const double a = 9.0 / 85.0;
    assert(near(d[0], -2 * a, 1e-12) && near(d[1], -2 * a, 1e-12) && near(d[2], -1 * a, 1e-12));
    // At a singular direction J^T e = 0, nothing to divide: the step is 0, not NaN.
    const Jacobian flat = jacobian(kUnit, {0, 0, 0});
    const Angles z = stepTranspose(flat, {1, 0});
    assert(z[0] == 0.0 && z[1] == 0.0 && z[2] == 0.0);
    // e = 0 gives no step.
    const Angles zero = stepTranspose(j, {0, 0});
    assert(zero[0] == 0.0 && zero[1] == 0.0 && zero[2] == 0.0);
}

void pseudoInverseStepByHand() {
    const Jacobian j = jacobian(kUnit, kBent);
    const Angles d = stepPseudoInverse(j, {1, 0});
    assert(near(d[0], 0.0, 1e-12) && near(d[1], -0.4, 1e-12) && near(d[2], -0.2, 1e-12));
    // It really solves J d = e.
    double ex = 0.0;
    double ey = 0.0;
    for (std::size_t i = 0; i < 3; ++i) {
        ex += j.col[i].x * d[i];
        ey += j.col[i].y * d[i];
    }
    assert(near(ex, 1.0, 1e-12) && near(ey, 0.0, 1e-12));
    // Exactly singular pose: the formula has no answer, the guard returns 0 instead of NaN or inf.
    const Angles s = stepPseudoInverse(jacobian(kUnit, {0, 0, 0}), {0, 1});
    assert(s[0] == 0.0 && s[1] == 0.0 && s[2] == 0.0);
}

void dampedStepByHand() {
    const Jacobian j = jacobian(kUnit, kBent);
    // lambda = 1: (JJ^T + I) = [[10,-2],[-2,2]], inverse 1/16 [[2,2],[2,10]], applied to (1,0) = (1/8, 1/8), J^T of that.
    const Angles d = stepDamped(j, {1, 0}, 1.0);
    assert(near(d[0], -0.125, 1e-12) && near(d[1], -0.25, 1e-12) && near(d[2], -0.125, 1e-12));
    // Straight arm, e = (0,1), lambda = 1: (JJ^T + I) = diag(1,15), step = (3,2,1) / 15. Damping makes it defined.
    const Angles s = stepDamped(jacobian(kUnit, {0, 0, 0}), {0, 1}, 1.0);
    assert(near(s[0], 3.0 / 15.0, 1e-12) && near(s[1], 2.0 / 15.0, 1e-12) && near(s[2], 1.0 / 15.0, 1e-12));
    // The radial direction at the straight arm is impossible: no motion, for every lambda.
    const Angles r = stepDamped(jacobian(kUnit, {0, 0, 0}), {1, 0}, 1.0);
    assert(near(r[0], 0.0, 1e-12) && near(r[1], 0.0, 1e-12) && near(r[2], 0.0, 1e-12));
    // Limits: small lambda approaches the pseudoinverse, large lambda approaches J^T e / lambda^2.
    const Angles pinv = stepPseudoInverse(j, {1, 0});
    const Angles tiny = stepDamped(j, {1, 0}, 1e-6);
    for (std::size_t i = 0; i < 3; ++i) assert(near(tiny[i], pinv[i], 1e-6));
    const Angles big = stepDamped(j, {1, 0}, 1e3);
    assert(near(big[0], -2e-6, 1e-9) && near(big[1], -2e-6, 1e-9) && near(big[2], -1e-6, 1e-9));
}

void clampMagnitude() {
    assert(nearVec(clampMag({3, 4}, 10.0), {3, 4}));      // shorter than d: unchanged
    assert(nearVec(clampMag({3, 4}, 5.0), {3, 4}));       // exactly d: unchanged
    assert(nearVec(clampMag({3, 4}, 2.5), {1.5, 2.0}));   // longer: same direction, length d
    assert(nearVec(clampMag({-3, 4}, 2.5), {-1.5, 2.0}));
    assert(nearVec(clampMag({0, 0}, 2.5), {0, 0}));       // zero vector: no division by zero
    assert(nearVec(clampMag({3, 4}, 0.0), {0, 0}));       // nothing may move
    assert(nearVec(clampMag({3, 4}, -1.0), {0, 0}));
}

void wrapping() {
    assert(near(wrapAngle(0.5), 0.5));
    assert(near(wrapAngle(7.0), 7.0 - 2.0 * kPi));
    assert(near(wrapAngle(-7.0), -7.0 + 2.0 * kPi));
    assert(near(wrapAngle(2.0 * kPi), 0.0));
    assert(std::fabs(wrapAngle(1e6)) <= kPi + 1e-12);
    // Wrapping never changes the pose.
    assert(nearVec(endEffector(kLengths, {7.0, -9.5, 20.0}), endEffector(kLengths, {wrapAngle(7.0), wrapAngle(-9.5), wrapAngle(20.0)}), 1e-9));
}

void transposeShrinksTheErrorInTheUsualCase() {
    // Reachable target, one transpose step at a time: the error shrinks every step, quickly at first.
    Angles theta{0.3, 0.4, -0.2};
    const Vec2 target{150.0, 100.0};
    double prev = norm(errorTo(kLengths, theta, target));
    const double start = prev;
    for (int i = 0; i < 15; ++i) {
        iterate(theta, kLengths, target, Method::Transpose, 0.0, 0.0);
        const double now = norm(errorTo(kLengths, theta, target));
        assert(now < prev);
        prev = now;
    }
    assert(prev < 0.01 * start);
    // In general it is not monotone: alpha is right for the linearized arm, and a big step on the real, curved arm can
    // overshoot. Count the runs (reachable targets, bent start poses, 30 steps) in which the error grows at least once.
    Lcg rng;
    int rises = 0;
    const int runs = 200;
    for (int k = 0; k < runs; ++k) {
        Angles t{rng.range(-1.5, 1.5), rng.range(0.3, 1.5) * (rng.next() < 0.5 ? -1 : 1), rng.range(0.3, 1.5) * (rng.next() < 0.5 ? -1 : 1)};
        const double r = rng.range(40.0, 200.0);
        const double a = rng.range(-kPi, kPi);
        const Vec2 tg{r * std::cos(a), r * std::sin(a)};
        double before = norm(errorTo(kLengths, t, tg));
        bool rose = false;
        for (int i = 0; i < 30; ++i) {
            iterate(t, kLengths, tg, Method::Transpose, 0.0, 0.0);
            const double after = norm(errorTo(kLengths, t, tg));
            if (after > before + 1e-9) rose = true;
            before = after;
        }
        if (rose) ++rises;
    }
    std::printf("transpose runs in which the error grows at least once: %d of %d\n", rises, runs);
    assert(rises > 0 && rises < runs / 5);
}

void iterateAppliesAndReports() {
    // iterate() returns the step it applied and moves the angles by exactly that.
    Angles theta{0.3, 0.4, -0.2};
    const Angles before = theta;
    const Angles d = iterate(theta, kLengths, {150.0, 100.0}, Method::Damped, 20.0, 0.0);
    for (std::size_t i = 0; i < 3; ++i) assert(near(wrapAngle(theta[i] - before[i]), wrapAngle(d[i]), 1e-12));
    assert(norm(d) > 0.0);
    // Target on the end effector: no error, no step, for every method.
    for (Method m : {Method::Transpose, Method::PseudoInverse, Method::Damped}) {
        Angles t{0.3, 0.4, -0.2};
        const Vec2 here = endEffector(kLengths, t);
        const Angles z = iterate(t, kLengths, here, m, 20.0, 0.0);
        assert(norm(z) == 0.0);
    }
    // The clamp shortens the error, so the step: far target, clamp 10 px against none.
    Angles a{0.3, 0.4, -0.2};
    Angles b = a;
    const double free = norm(iterate(a, kLengths, {-200.0, 150.0}, Method::Damped, 20.0, 0.0));
    const double clamped = norm(iterate(b, kLengths, {-200.0, 150.0}, Method::Damped, 20.0, 10.0));
    assert(clamped < free);
}

void nearTheSingularityPseudoInverseExplodes() {
    // The arm almost stretched along +x, bent by 0.01 rad at the second joint. One pixel of error along the weak (radial)
    // direction. The pseudoinverse needs 1 / sigma_min radians for it, damping with lambda = 20 needs at most 1 / (2 lambda).
    const Angles theta{0.0, 0.01, 0.0};
    const Jacobian j = jacobian(kLengths, theta);
    const SingularValues sv = singularValues(j);
    assert(sv.min > 0.5 && sv.min < 0.6);
    std::printf("bend 0.01 rad: sigma_min %.4f, sigma_max %.1f\n", sv.min, sv.max);
    const Vec2 e = weakDirection(j);
    const double pinv = norm(stepPseudoInverse(j, e));
    const double dls = norm(stepDamped(j, e, 20.0));
    assert(near(pinv, 1.0 / sv.min, 1e-9));
    assert(pinv > 1.8 && pinv < 1.9);  // more than a hundred degrees for one pixel
    assert(dls < 1.0 / (2.0 * 20.0));
    assert(dls < 0.002);
    assert(pinv > 1000.0 * dls);
    // Same pixel, but along the easy direction (sideways): both methods agree it is cheap.
    const Vec2 side{-e.y, e.x};
    assert(norm(stepPseudoInverse(j, side)) < 0.01);
    // Away from the singularity the damping costs little: bend 1 rad, same pixel in the weak direction.
    const Jacobian far = jacobian(kLengths, {0.0, 1.0, 0.0});
    const Vec2 ef = weakDirection(far);
    assert(norm(stepPseudoInverse(far, ef)) < 0.02);
}

void dampedStepIsBounded() {
    // sigma / (sigma^2 + lambda^2) <= 1 / (2 lambda), so |step| <= |e| / (2 lambda) for every pose and every error.
    Lcg rng;
    for (const double lambda : {2.0, 10.0, 20.0, 40.0}) {
        for (int i = 0; i < 400; ++i) {
            const Angles theta{rng.range(-kPi, kPi), rng.range(-0.3, 0.3) + (rng.next() < 0.5 ? 0.0 : rng.range(-kPi, kPi)), rng.range(-kPi, kPi)};
            const Vec2 e{rng.range(-80.0, 80.0), rng.range(-80.0, 80.0)};
            const double bound = std::hypot(e.x, e.y) / (2.0 * lambda);
            assert(norm(stepDamped(jacobian(kLengths, theta), e, lambda)) <= bound + 1e-9);
        }
    }
}

void pseudoInverseIsUnboundedNearSingularity() {
    // The pseudoinverse needs 1 / sigma_min radians for one pixel along the weak direction, and sigma_min shrinks in
    // proportion to the bend: ten times less bend, ten times more radians. The guard only kicks in far below that.
    const double bends[] = {1e-2, 1e-3, 1e-4};
    double previous = 0.0;
    for (const double bend : bends) {
        const Jacobian j = jacobian(kLengths, {0.0, bend, 0.0});
        const double step = norm(stepPseudoInverse(j, weakDirection(j)));
        std::printf("bend %.0e rad: pseudoinverse needs %.1f rad for 1 px, damped 20 needs %.5f rad\n", bend, step,
                    norm(stepDamped(j, weakDirection(j), 20.0)));
        if (previous > 0.0) assert(step > 9.0 * previous && step < 11.0 * previous);
        previous = step;
    }
    assert(previous > 100.0);
}

void unreachableTargetSettles() {
    // Target 20 px beyond the reach (270 px from a base with reach 250), start from a bent pose, 300 steps.
    const Vec2 target{270.0, 0.0};
    struct Result {
        double lateMean;
        double lateMax;
        double dist;
    };
    const auto run = [&](Method m, double lambda, double dmax) {
        Angles theta{0.3, -0.4, 0.3};
        Result r{0.0, 0.0, 0.0};
        for (int i = 0; i < 300; ++i) {
            const double s = norm(iterate(theta, kLengths, target, m, lambda, dmax));
            if (i >= 240) {
                r.lateMean += s / 60.0;
                r.lateMax = std::max(r.lateMax, s);
            }
        }
        const Vec2 e = errorTo(kLengths, theta, target);
        r.dist = std::hypot(e.x, e.y);
        return r;
    };
    const Result tr = run(Method::Transpose, 0.0, 0.0);
    const Result pinv = run(Method::PseudoInverse, 0.0, 0.0);
    const Result d5 = run(Method::Damped, 5.0, 0.0);
    const Result d20 = run(Method::Damped, 20.0, 0.0);
    const Result d40 = run(Method::Damped, 40.0, 0.0);
    const Result d40c = run(Method::Damped, 40.0, kDmax);
    const Result pinvc = run(Method::PseudoInverse, 0.0, kDmax);
    std::printf("unreachable target at 270 px (reach 250), steps 240..299:\n");
    std::printf("  transpose        mean %.4f rad  max %.4f  distance %.3f px\n", tr.lateMean, tr.lateMax, tr.dist);
    std::printf("  pseudoinverse    mean %.4f rad  max %.4f  distance %.3f px\n", pinv.lateMean, pinv.lateMax, pinv.dist);
    std::printf("  pinv + clamp     mean %.4f rad  max %.4f  distance %.3f px\n", pinvc.lateMean, pinvc.lateMax, pinvc.dist);
    std::printf("  damped 5         mean %.4f rad  max %.4f  distance %.3f px\n", d5.lateMean, d5.lateMax, d5.dist);
    std::printf("  damped 20        mean %.4f rad  max %.4f  distance %.3f px\n", d20.lateMean, d20.lateMax, d20.dist);
    std::printf("  damped 40        mean %.4f rad  max %.4f  distance %.3f px\n", d40.lateMean, d40.lateMax, d40.dist);
    std::printf("  damped 40+clamp  mean %.4f rad  max %.4f  distance %.3f px\n", d40c.lateMean, d40c.lateMax, d40c.dist);
    // Transposed: quiet and almost optimal (the closest possible distance is 20 px).
    assert(tr.lateMax < 0.01 && tr.dist > 19.99 && tr.dist < 20.1);
    // Pseudoinverse: keeps swinging by whole radians and the end effector flies far from the target.
    assert(pinv.lateMean > 1.0);
    // Damping with too little lambda also keeps swinging, enough lambda settles at the optimum.
    assert(d5.lateMean > 0.5);
    assert(d20.lateMean > 0.3);
    assert(d40.lateMax < 1e-3 && d40.dist > 19.99 && d40.dist < 20.1);
    assert(d40c.lateMax < 1e-3 && d40c.dist > 19.99 && d40c.dist < 20.1);
}

void transposeIsSlowNearTheStretch() {
    // Reachable by 1 px (249 of 250), 120 steps: pseudoinverse and damping are on the target, the transpose is still 2 px away.
    const Vec2 target{249.0, 0.0};
    const auto dist = [&](Method m) {
        Angles theta{0.3, -0.4, 0.3};
        for (int i = 0; i < 120; ++i) iterate(theta, kLengths, target, m, 20.0, 0.0);
        const Vec2 e = errorTo(kLengths, theta, target);
        return std::hypot(e.x, e.y);
    };
    const double tr = dist(Method::Transpose);
    const double pinv = dist(Method::PseudoInverse);
    const double dls = dist(Method::Damped);
    std::printf("target 1 px inside the reach, distance after 120 steps: transpose %.3f, pinv %.6f, damped 20 %.6f\n", tr, pinv, dls);
    assert(tr > 1.5);
    assert(pinv < 0.01);
    assert(dls < 0.01);
}

void clampAgainstNoClampOverManyTargets() {
    // 100 unreachable targets (1.02 to 1.3 times the reach), random start poses, 300 steps. A run "jitters" when
    // some step in the last 100 is bigger than 0.05 rad.
    const auto jittering = [](Method m, double lambda, double dmax) {
        Lcg rng;
        int count = 0;
        for (int k = 0; k < 100; ++k) {
            const double r = reach(kLengths) * rng.range(1.02, 1.3);
            const double a = rng.range(-1.2, 1.2);
            const Vec2 target{r * std::cos(a), r * std::sin(a)};
            Angles theta{rng.range(-1.0, 1.0), rng.range(-1.0, 1.0), rng.range(-1.0, 1.0)};
            double late = 0.0;
            for (int i = 0; i < 300; ++i) {
                const double s = norm(iterate(theta, kLengths, target, m, lambda, dmax));
                if (i >= 200) late = std::max(late, s);
            }
            if (late > 0.05) ++count;
        }
        return count;
    };
    const int pinv = jittering(Method::PseudoInverse, 0.0, 0.0);
    const int d20 = jittering(Method::Damped, 20.0, 0.0);
    const int d40 = jittering(Method::Damped, 40.0, 0.0);
    const int d40c = jittering(Method::Damped, 40.0, kDmax);
    std::printf("jittering runs of 100 unreachable targets: pinv %d, damped 20 %d, damped 40 %d, damped 40 + clamp %d\n", pinv, d20, d40, d40c);
    assert(pinv >= 95);
    assert(d20 >= 50);
    assert(d40 >= 10);  // damping alone is not enough for the far targets
    assert(d40c == 0);  // with the clamp it is
}

void autopilotStaysInRange() {
    Autopilot ap;
    const double len = reach(kLengths);
    // The first target is outside the reach, so the first frame already shows the trouble.
    const Vec2 first = ap.target();
    assert(std::hypot(first.x, first.y) > len && std::hypot(first.x, first.y) < 1.08 * len + 1e-9);
    double lo = 1e9;
    double hi = 0.0;
    double ymax = 0.0;
    for (int i = 0; i < 60 * 600; ++i) {
        ap.advance(1.0 / 60.0);
        const Vec2 t = ap.target();
        const double r = std::hypot(t.x, t.y);
        lo = std::min(lo, r);
        hi = std::max(hi, r);
        ymax = std::max(ymax, std::fabs(t.y));
        assert(t.x > 0.0);
    }
    assert(lo > 0.81 * len && lo < 0.83 * len);   // the path dips well inside the reach ...
    assert(hi > 1.07 * len && hi < 1.081 * len);  // ... and leaves it
    assert(ymax < 0.6 * 1.08 * len);              // fits the field
    // The phases never leave one period, however long it runs.
    Autopilot long_run;
    for (int i = 0; i < 1000000; ++i) long_run.advance(1.0 / 60.0);
    assert(long_run.phaseA >= 0.0 && long_run.phaseA < 2.0 * kPi && long_run.phaseB >= 0.0 && long_run.phaseB < 2.0 * kPi);
    // A zero or negative dt moves nothing.
    Autopilot still;
    const Vec2 before = still.target();
    still.advance(0.0);
    still.advance(-1.0);
    assert(before.x == still.target().x && before.y == still.target().y);
}

void stepFor() {
    const Jacobian j = jacobian(kUnit, kBent);
    const Vec2 e{1, 0};
    const Angles a = stepFor(Method::Transpose, j, e, 7.0);
    const Angles b = stepTranspose(j, e);
    const Angles c = stepFor(Method::PseudoInverse, j, e, 7.0);
    const Angles d = stepFor(Method::Damped, j, e, 7.0);
    const Angles dd = stepDamped(j, e, 7.0);
    for (std::size_t i = 0; i < 3; ++i) assert(a[i] == b[i] && d[i] == dd[i]);
    assert(near(c[1], -0.4, 1e-12));  // lambda is ignored for the pseudoinverse
}

}  // namespace

int main() {
    forwardKinematics();
    jacobianByHand();
    jacobianMatchesFiniteDifferences();
    matrixInverse();
    gramMatrixAndSingularValues();
    weakDirectionIsRadialWhenStretched();
    transposeStepByHand();
    pseudoInverseStepByHand();
    dampedStepByHand();
    clampMagnitude();
    wrapping();
    transposeShrinksTheErrorInTheUsualCase();
    iterateAppliesAndReports();
    nearTheSingularityPseudoInverseExplodes();
    dampedStepIsBounded();
    pseudoInverseIsUnboundedNearSingularity();
    unreachableTargetSettles();
    transposeIsSlowNearTheStretch();
    clampAgainstNoClampOverManyTargets();
    autopilotStaysInRange();
    stepFor();
    std::puts("jacobian-singularity logic: all asserts passed");
    return 0;
}
