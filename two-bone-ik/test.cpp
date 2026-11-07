// Tests for the two-bone IK logic. Plain asserts on hand-computed cases, no framework, no SDL.
// Coordinates are screen coordinates (y points down). "Left" is the left of the direction root -> target as seen on screen.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "logic.hpp"

using namespace ik;

namespace {

constexpr double kPi = 3.14159265358979323846;

bool near(double a, double b, double eps = 1e-9) { return std::fabs(a - b) < eps; }
bool near(Vec2 a, Vec2 b, double eps = 1e-9) { return near(a.x, b.x, eps) && near(a.y, b.y, eps); }
bool finite(Vec2 v) { return std::isfinite(v.x) && std::isfinite(v.y); }

// The 3-4-5 triangle: d1 = 3, d2 = 4, h = 5. cos(alpha) = (9 + 25 - 16) / 30 = 0.6, sin(alpha) = 0.8,
// so the joint is at (1.8, +-2.4), and the elbow angle is 90 degrees because 3^2 + 4^2 = 5^2.
void threeFourFive() {
    const Vec2 root{0, 0};
    const Vec2 target{5, 0};
    const Pose left = solve(root, target, 3, 4, Bend::Left);
    const Pose right = solve(root, target, 3, 4, Bend::Right);
    assert(near(left.joint, {1.8, -2.4}));  // left of "towards +x" on a y-down screen is up
    assert(near(right.joint, {1.8, 2.4}));
    assert(near(left.end, target) && near(right.end, target));
    assert(left.reach == Reach::InRange && right.reach == Reach::InRange);
    assert(near(left.h, 5.0));
    assert(near(left.cosAlpha, 0.6));
    assert(near(left.alpha, std::acos(0.6)));
    assert(near(left.gamma, kPi / 2.0));
    assert(solutionCount(5, 3, 4) == 2);
}

// For every target inside the ring and both bends: the end sits on the target, the segments keep their length.
void lengthsAndReachInsideTheRing() {
    const double lengths[][2] = {{3, 4}, {4, 3}, {125, 70}, {70, 125}, {100, 100}, {1, 10}, {10, 1}};
    for (const auto& l : lengths) {
        const double d1 = l[0];
        const double d2 = l[1];
        const Vec2 root{40, -25};
        int tested = 0;
        for (int a = 0; a < 24; ++a) {
            for (int k = 1; k <= 9; ++k) {
                // 9 radii strictly between min and max reach (equal lengths: min is 0, k = 1 is still above it)
                const double r = minReach(d1, d2) + (maxReach(d1, d2) - minReach(d1, d2)) * k / 10.0;
                const double ang = 2.0 * kPi * a / 24.0 + 0.1;
                const Vec2 target = root + Vec2{std::cos(ang), std::sin(ang)} * r;
                for (const Bend bend : {Bend::Left, Bend::Right}) {
                    const Pose p = solve(root, target, d1, d2, bend);
                    assert(p.reach == Reach::InRange);
                    assert(near(p.end, target, 1e-8));
                    assert(near(length(p.joint - root), d1, 1e-8));
                    assert(near(length(p.end - p.joint), d2, 1e-8));
                    assert(near(p.h, r, 1e-8));
                    ++tested;
                }
            }
        }
        assert(tested == 24 * 9 * 2);
    }
}

// The two bends are mirror images across the line root -> target: same distance to the line, opposite sides,
// and different from each other everywhere except on the border of the ring.
void bendsAreMirrorImages() {
    const Vec2 root{0, 0};
    const Vec2 target{6, 8};  // h = 10, direction (0.6, 0.8)
    const Pose l = solve(root, target, 7, 5, Bend::Left);
    const Pose r = solve(root, target, 7, 5, Bend::Right);
    const Vec2 dir{0.6, 0.8};
    // signed distance to the line: cross(dir, p) = dir.x * p.y - dir.y * p.x, positive is right of dir on a y-down screen
    const double cl = dir.x * l.joint.y - dir.y * l.joint.x;
    const double cr = dir.x * r.joint.y - dir.y * r.joint.x;
    assert(cl < 0.0 && cr > 0.0);  // left is negative, right is positive
    assert(near(cl, -cr));
    assert(length(l.joint - r.joint) > 1.0);
    // along the line both joints are at the same place: d1 * cos(alpha) = (49 + 100 - 25) / 20 = 6.2
    assert(near(dot(l.joint, dir), 6.2));
    assert(near(dot(r.joint, dir), 6.2));
    // the lower segment, seen from the target, ends on the root side as well (mirror: |joint - root| equal)
    assert(near(length(l.joint - root), length(r.joint - root)));
}

// h = d1 + d2: a fully stretched arm, one solution, still in range.
void borderOfFullStretch() {
    const Vec2 root{10, 10};
    const Vec2 target{10, 17};  // straight down, h = 7 = 3 + 4
    for (const Bend bend : {Bend::Left, Bend::Right}) {
        const Pose p = solve(root, target, 3, 4, bend);
        assert(p.reach == Reach::InRange);
        assert(near(p.end, target, 1e-9));
        assert(near(p.joint, {10, 13}, 1e-9));  // on the line, 3 from the root
        assert(near(p.alpha, 0.0, 1e-7));
        assert(near(p.gamma, kPi, 1e-7));
    }
    assert(solutionCount(7, 3, 4) == 1);
    // a hair beyond is already "too far", and the pose is the same stretched one
    const Pose beyond = solve(root, {10, 17.001}, 3, 4, Bend::Left);
    assert(beyond.reach == Reach::TooFar);
    assert(solutionCount(7.001, 3, 4) == 0);
}

// h = |d1 - d2|: folded, the lower segment lies on top of the upper one, one solution.
void borderOfFullFold() {
    const Vec2 root{0, 0};
    // d1 = 4, d2 = 3, target 1 to the right: joint at (4, 0), end back at (1, 0)
    const Pose a = solve(root, {1, 0}, 4, 3, Bend::Left);
    assert(a.reach == Reach::InRange);
    assert(near(a.joint, {4, 0}, 1e-7));
    assert(near(a.end, {1, 0}, 1e-7));
    assert(near(a.gamma, 0.0, 1e-6));
    assert(solutionCount(1, 4, 3) == 1);
    // d1 = 3, d2 = 4, same target: the upper segment points away from the target, joint at (-3, 0), end at (1, 0)
    const Pose b = solve(root, {1, 0}, 3, 4, Bend::Right);
    assert(b.reach == Reach::InRange);
    assert(near(b.joint, {-3, 0}, 1e-7));
    assert(near(b.end, {1, 0}, 1e-7));
    assert(near(b.alpha, kPi, 1e-6));
    assert(solutionCount(1, 3, 4) == 1);
}

// h = 0 divides by zero in the textbook formula. Unequal lengths: the target sits in the hole, nothing is reachable.
// Equal lengths: the arm folds onto itself, every direction works, we report "infinite" and draw one.
void targetOnTheRoot() {
    const Vec2 root{5, 5};
    const Pose p = solve(root, root, 4, 3, Bend::Left);
    assert(finite(p.joint) && finite(p.end));
    assert(p.reach == Reach::TooClose);
    assert(near(length(p.joint - root), 4.0));
    assert(near(length(p.end - root), 1.0));  // as close as the ring allows: |d1 - d2|
    assert(solutionCount(0, 4, 3) == 0);

    const Pose q = solve(root, root, 3, 3, Bend::Right);
    assert(finite(q.joint) && finite(q.end));
    assert(q.reach == Reach::InRange);
    assert(near(q.end, root));
    assert(near(length(q.joint - root), 3.0));
    assert(solutionCount(0, 3, 3) == kInfinite);
    assert(solutionCount(0.5, 3, 3) == 2);
}

// Far outside: the arm is stretched straight at the target, the end sits at d1 + d2 on that line.
void tooFarStretches() {
    const Vec2 root{0, 0};
    const Vec2 target{60, 80};  // h = 100, direction (0.6, 0.8)
    for (const Bend bend : {Bend::Left, Bend::Right}) {
        const Pose p = solve(root, target, 30, 40, bend);
        assert(p.reach == Reach::TooFar);
        assert(near(p.joint, {18, 24}));  // 30 along the direction
        assert(near(p.end, {42, 56}));    // 70 along the direction
        assert(near(p.gamma, kPi));
        assert(near(p.h, 100.0));
    }
    // it is the nearest reachable point: the distance left over is h - (d1 + d2)
    const Pose p = solve(root, target, 30, 40, Bend::Left);
    assert(near(length(target - p.end), 30.0));
    assert(solutionCount(100, 30, 40) == 0);
}

// Inside the hole: folded towards the target, the end sits at |d1 - d2| on the line to the target.
void tooCloseFolds() {
    const Vec2 root{0, 0};
    const Vec2 target{0, 1};  // h = 1, hole radius is |5 - 2| = 3
    const Pose p = solve(root, target, 5, 2, Bend::Left);
    assert(p.reach == Reach::TooClose);
    assert(near(p.joint, {0, 5}));
    assert(near(p.end, {0, 3}));
    assert(near(length(target - p.end), 2.0));  // nearest ring point: min reach - h
    assert(near(p.gamma, 0.0));
    // d1 shorter than d2: the upper segment points away, the lower one reaches back past the root
    const Pose q = solve(root, target, 2, 5, Bend::Left);
    assert(q.reach == Reach::TooClose);
    assert(near(q.joint, {0, -2}));
    assert(near(q.end, {0, 3}));
    assert(solutionCount(1, 5, 2) == 0);
}

// The unclamped textbook version: identical inside the ring, NaN outside and in the hole.
void naiveBreaksOutsideTheRing() {
    const Vec2 root{0, 0};
    for (const Bend bend : {Bend::Left, Bend::Right}) {
        const Pose good = solve(root, {5, 0}, 3, 4, bend);
        const Pose naive = solveNaive(root, {5, 0}, 3, 4, bend);
        assert(near(naive.joint, good.joint) && near(naive.end, good.end));
        assert(naive.reach == Reach::InRange);
    }
    const Pose far = solveNaive(root, {30, 0}, 3, 4, Bend::Left);  // cos(alpha) = (9 + 900 - 16) / 180 = 4.96
    assert(near(far.cosAlpha, 893.0 / 180.0));
    assert(std::isnan(far.alpha) && !finite(far.joint) && !finite(far.end));
    assert(far.reach == Reach::TooFar);
    const Pose hole = solveNaive(root, {1, 0}, 5, 2, Bend::Left);  // cos(alpha) = (25 + 1 - 4) / 10 = 2.2
    assert(std::isnan(hole.alpha) && !finite(hole.joint));
    assert(hole.reach == Reach::TooClose);
    const Pose zero = solveNaive(root, root, 4, 3, Bend::Left);  // division by zero
    assert(!finite(zero.joint));
    // the safe version never produces a NaN, wherever the target is
    for (const Vec2 t : {Vec2{30, 0}, Vec2{1, 0}, Vec2{0, 0}, Vec2{-1e6, 3e5}, Vec2{1e-12, 0}}) {
        const Pose p = solve(root, t, 4, 3, Bend::Left);
        assert(finite(p.joint) && finite(p.end) && std::isfinite(p.alpha) && std::isfinite(p.gamma));
    }
}

// Rounding: for d1 = 0.1, d2 = 0.2 and h = 0.1 + 0.2 the cosine comes out a hair above 1 in double precision,
// so even a target exactly on the border breaks the unclamped formula. The clamped one has to survive it.
void roundingOnTheBorder() {
    const double d1 = 0.1;
    const double d2 = 0.2;
    const double h = d1 + d2;
    const double c = (d1 * d1 + h * h - d2 * d2) / (2.0 * d1 * h);
    assert(c > 1.0 && c < 1.0 + 1e-12);
    const Pose p = solve({0, 0}, {h, 0}, d1, d2, Bend::Left);
    assert(finite(p.joint) && finite(p.end));
    assert(near(p.end, {h, 0}, 1e-9));
    assert(near(length(p.joint), d1, 1e-9));
}

// Degenerate segment lengths must not produce NaN either.
void degenerateLengths() {
    const Pose p = solve({0, 0}, {5, 0}, 0.0, 3.0, Bend::Left);
    assert(finite(p.joint) && finite(p.end));
    const Pose q = solve({0, 0}, {5, 0}, -2.0, 3.0, Bend::Right);
    assert(finite(q.joint) && finite(q.end));
}

// The ring as horizontal spans: center (0,0), radii 3 and 5.
void ringSpansPerRow() {
    const Vec2 c{0, 0};
    const RowSpans mid = ringSpans(c, 3, 5, 0);  // through the center: two bars, [-5,-3] and [3,5]
    assert(mid.n == 2);
    assert(near(mid.s[0].x0, -5) && near(mid.s[0].x1, -3));
    assert(near(mid.s[1].x0, 3) && near(mid.s[1].x1, 5));
    const RowSpans low = ringSpans(c, 3, 5, 4);  // below the hole: sqrt(25 - 16) = 3, one span [-3,3]
    assert(low.n == 1);
    assert(near(low.s[0].x0, -3) && near(low.s[0].x1, 3));
    const RowSpans tangent = ringSpans(c, 3, 5, 3);  // touches the hole at one point: one span, sqrt(25 - 9) = 4
    assert(tangent.n == 1);
    assert(near(tangent.s[0].x0, -4) && near(tangent.s[0].x1, 4));
    assert(ringSpans(c, 3, 5, 5).n == 0);  // the top of the outer circle has no width
    assert(ringSpans(c, 3, 5, -6).n == 0);
    const RowSpans disc = ringSpans({10, 20}, 0, 5, 20);  // no hole: one span around the center x
    assert(disc.n == 1 && near(disc.s[0].x0, 5) && near(disc.s[0].x1, 15));
}

// The auto path sweeps through hole, ring and the outside, and loops without a jump.
void autoPath() {
    const Vec2 root{100, 200};
    const double d1 = 125;
    const double d2 = 70;
    bool sawHole = false;
    bool sawOutside = false;
    bool sawRing = false;
    for (int i = 0; i < 1200; ++i) {
        const double phase = kAutoPeriod * i / 1200.0;
        const double r = length(autoTarget(root, d1, d2, phase) - root);
        assert(r <= 1.2 * maxReach(d1, d2) + 1e-9);
        sawHole = sawHole || r < minReach(d1, d2);
        sawOutside = sawOutside || r > maxReach(d1, d2);
        sawRing = sawRing || (r > minReach(d1, d2) && r < maxReach(d1, d2));
    }
    assert(sawHole && sawOutside && sawRing);
    assert(near(autoTarget(root, d1, d2, kAutoPeriod - 1e-9), autoTarget(root, d1, d2, 0.0), 1e-6));
    // the first frame (phase 0) is inside the ring, so it shows a bent arm
    const Pose first = solve(root, autoTarget(root, d1, d2, 0.0), d1, d2, Bend::Left);
    assert(first.reach == Reach::InRange);
}

void phaseWraps() {
    assert(near(advancePhase(11.99, 0.02), 0.01, 1e-9));
    assert(near(advancePhase(0.0, 0.25), 0.25));
    const double big = advancePhase(3.0, 25.0);  // dt is clamped by the caller, but the function must not trust that
    assert(big >= 0.0 && big < kAutoPeriod && near(big, 4.0, 1e-9));
    assert(advancePhase(5.0, 0.0) == 5.0);
}

}  // namespace

int main() {
    threeFourFive();
    lengthsAndReachInsideTheRing();
    bendsAreMirrorImages();
    borderOfFullStretch();
    borderOfFullFold();
    targetOnTheRoot();
    tooFarStretches();
    tooCloseFolds();
    naiveBreaksOutsideTheRing();
    roundingOnTheBorder();
    degenerateLengths();
    ringSpansPerRow();
    autoPath();
    phaseWraps();
    std::puts("two-bone-ik logic: all asserts passed");
    return 0;
}
