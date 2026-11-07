// Tests for the joint-limits logic. Plain asserts on hand-computed cases, no framework, no SDL.
// Coordinates are screen coordinates (y points down), angles in radians, positive turns clockwise on the screen.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "logic.hpp"

using namespace jl;

namespace {

constexpr double kDeg = kPi / 180.0;

bool near(double a, double b, double eps = 1e-9) { return std::fabs(a - b) < eps; }
bool near(Vec2 a, Vec2 b, double eps = 1e-9) { return near(a.x, b.x, eps) && near(a.y, b.y, eps); }

// Segment lengths 4, 3, 2, 1: the sums 4, 7, 9, 10 make every forward-kinematics case easy to do by hand.
const Arm kArm{{0, 0}, {4, 3, 2, 1}};

// Elbow 0..150 degrees, the first joint free, the last two stiff.
Limits elbowOnly() {
    Limits l;
    l.lo = {-kPi, 0.0, 0.0, 0.0};
    l.hi = {kPi, 150.0 * kDeg, 0.0, 0.0};
    return l;
}

// A bendable but bounded arm: every joint has a range of its own.
Limits bounded() {
    Limits l;
    l.lo = {-110.0 * kDeg, 0.0, -60.0 * kDeg, -45.0 * kDeg};
    l.hi = {70.0 * kDeg, 150.0 * kDeg, 60.0 * kDeg, 45.0 * kDeg};
    return l;
}

// Deterministic numbers in [0, 1): a 64-bit LCG, top 53 bits.
struct Lcg {
    std::uint64_t state = 12345;
    double next() {
        state = state * 6364136223846793005ULL + 1442695040888963407ULL;
        return static_cast<double>(state >> 11) / 9007199254740992.0;
    }
};

Angles randomInside(const Limits& l, Lcg& rng) {
    Angles a{};
    for (std::size_t i = 0; i < kJoints; ++i) a[i] = l.lo[i] + (l.hi[i] - l.lo[i]) * rng.next();
    return a;
}

void wrapping() {
    assert(near(wrapAngle(0.0), 0.0));
    assert(near(wrapAngle(1.5 * kPi), -0.5 * kPi));
    assert(near(wrapAngle(-1.5 * kPi), 0.5 * kPi));
    assert(near(wrapAngle(2.0 * kPi), 0.0));
    assert(near(wrapAngle(kPi), kPi));    // the range is (-pi, pi], pi stays
    assert(near(wrapAngle(-kPi), kPi));   // and -pi is the same direction, it comes out as +pi
    assert(near(wrapAngle(7.0 * kPi), kPi));
}

void clampingOneAngle() {
    const double lo = 0.0;
    const double hi = 150.0 * kDeg;
    assert(near(clampAngle(75.0 * kDeg, lo, hi), 75.0 * kDeg));  // inside stays
    assert(near(clampAngle(0.0, lo, hi), 0.0));                  // on the border stays
    assert(near(clampAngle(150.0 * kDeg, lo, hi), hi));
    assert(near(clampAngle(160.0 * kDeg, lo, hi), hi));          // just above
    assert(near(clampAngle(-10.0 * kDeg, lo, hi), lo));          // just below
    // -170 degrees is 190 degrees: 40 past the upper border, 170 past the lower one.
    assert(near(clampAngle(-170.0 * kDeg, lo, hi), hi));
    // -30 degrees is 30 below the lower border and 180 away from the upper one.
    assert(near(clampAngle(-30.0 * kDeg, lo, hi), lo));
    // An angle given as 2 pi + 10 degrees is inside after wrapping.
    assert(near(clampAngle(2.0 * kPi + 10.0 * kDeg, lo, hi), 10.0 * kDeg));
    // A range of width 0 pins the joint.
    assert(near(clampAngle(1.0, 0.25, 0.25), 0.25));
    assert(near(clampAngle(-3.0, 0.25, 0.25), 0.25));
}

void clampingAllAngles() {
    const Limits l = bounded();
    const Angles in{{0.2, 1.0, -0.5, 0.3}};
    assert(inLimits(in, l));
    const Angles same = clampAll(in, l);
    for (std::size_t i = 0; i < kJoints; ++i) assert(near(same[i], in[i]));
    const Angles out{{2.0, -0.4, 1.5, -1.0}};
    assert(!inLimits(out, l));
    const Angles c = clampAll(out, l);
    assert(near(c[0], 70.0 * kDeg));
    assert(near(c[1], 0.0));
    assert(near(c[2], 60.0 * kDeg));
    assert(near(c[3], -45.0 * kDeg));
    assert(inLimits(c, l));
}

void forwardKinematics() {
    {
        const Points p = forward(kArm, {{0, 0, 0, 0}});
        assert(near(p[0], {0, 0}) && near(p[1], {4, 0}) && near(p[2], {7, 0}) && near(p[3], {9, 0}) && near(p[4], {10, 0}));
    }
    {
        // The first angle is absolute: 90 degrees points down the screen.
        const Points p = forward(kArm, {{90.0 * kDeg, 0, 0, 0}});
        assert(near(p[1], {0, 4}) && near(p[2], {0, 7}) && near(p[3], {0, 9}) && near(p[4], {0, 10}));
    }
    {
        // The others are relative to the segment before: an elbow of 90 degrees turns the rest down.
        const Points p = forward(kArm, {{0, 90.0 * kDeg, 0, 0}});
        assert(near(p[1], {4, 0}) && near(p[2], {4, 3}) && near(p[3], {4, 5}) && near(p[4], {4, 6}));
    }
    {
        // Elbow +90, wrist -90: the last two segments point right again.
        const Points p = forward(kArm, {{0, 90.0 * kDeg, -90.0 * kDeg, 0}});
        assert(near(p[2], {4, 3}) && near(p[3], {6, 3}) && near(p[4], {7, 3}));
    }
    {
        // The root can sit anywhere.
        const Arm moved{{100, 50}, {4, 3, 2, 1}};
        assert(near(tip(moved, {{0, 0, 0, 0}}), {110, 50}));
    }
}

void reaches() {
    assert(near(reachMax(kArm), 10.0));
    assert(near(reachMin(kArm), 0.0));  // 2 * 4 - 10 < 0
    const Arm longFirst{{0, 0}, {10, 1, 1, 1}};  // the first segment is longer than the rest together: a hole of 10 - 3 = 7
    assert(near(reachMax(longFirst), 13.0));
    assert(near(reachMin(longFirst), 7.0));
}

void freeSolveReachesAndKeepsLengths() {
    Angles a{{0.3, -0.5, 0.4, 0.2}};
    const Vec2 target{5, 3};  // 5.83 from the root, well inside the reach of 10
    const Solve s = solve(kArm, a, target, nullptr, 1e-6, 200);
    assert(s.error < 1e-6);
    assert(near(tipError(kArm, a, target), s.error));
    const Points p = forward(kArm, a);
    for (std::size_t i = 0; i < kJoints; ++i) {
        const Vec2 d{p[i + 1].x - p[i].x, p[i + 1].y - p[i].y};
        assert(near(std::sqrt(d.x * d.x + d.y * d.y), kArm.len[i], 1e-12));  // angles cannot change a length
    }
    assert(near(p[0], {0, 0}));  // the root stays
}

void freeSolveBeyondTheReach() {
    Angles a{{0.3, -0.5, 0.4, 0.2}};
    const Vec2 target{50, 0};
    const Solve s = solve(kArm, a, target, nullptr, 1e-6, 200);
    // The best the arm can do: straight at the target, the tip at distance 10, 40 short of it.
    assert(near(s.error, 40.0, 1e-6));
}

void targetOnTheRoot() {
    // Nothing to aim at: the angles must stay finite and the solver must not loop forever.
    Angles a{{0.3, 0.5, 0.4, 0.2}};
    const Solve s = solve(kArm, a, {0, 0}, nullptr, 1e-6, 200);
    for (const double v : a) assert(std::isfinite(v));
    assert(std::isfinite(s.error));
    const Limits l = bounded();
    Angles b{{0.3, 0.5, 0.4, 0.2}};
    solve(kArm, b, {0, 0}, &l, 1e-6, 200);
    for (const double v : b) assert(std::isfinite(v));
}

void singleSweepClampsEveryJoint() {
    // One sweep with limits: every angle inside after it, even though the free sweep would leave them.
    const Limits l = bounded();
    Angles a{{0.0, 0.3, 0.0, 0.0}};
    sweep(kArm, a, {-8, -3}, &l);
    assert(inLimits(a, l));
    Angles f{{0.0, 0.3, 0.0, 0.0}};
    sweep(kArm, f, {-8, -3}, nullptr);
    assert(!inLimits(f, l));  // the same sweep without limits leaves the box (the shoulder alone turns about 200 degrees)
}

void limitsHoldAtTheEnd() {
    // For a ring of targets inside and outside the reach: the clamped solver never leaves its box.
    const Limits l = bounded();
    int n = 0;
    for (int ring = 0; ring < 4; ++ring) {
        const double radius = 2.0 + 3.0 * ring;
        for (int k = 0; k < 36; ++k) {
            const double ang = 2.0 * kPi * k / 36.0;
            Angles a = clampAll({{0.0, 0.5, 0.0, 0.0}}, l);
            solve(kArm, a, {radius * std::cos(ang), radius * std::sin(ang)}, &l, 1e-6, 100);
            assert(inLimits(a, l));
            assert(std::isfinite(tipError(kArm, a, {radius * std::cos(ang), radius * std::sin(ang)})));
            ++n;
        }
    }
    assert(n == 144);
}

// The elbow only bends clockwise (0..150 degrees), the wrist joints are pinned at 0. A target 8 from the root on the left: the
// arm starts bent the allowed way, but the solver without limits settles on a pose with the elbow bent the forbidden way (and
// uses the wrist joints too). Clamping that pose at the end moves every joint and the tip misses. Clamping every step gives
// a pose that is allowed and on the target.
void clampAtTheEndMissesTheStepClampDoesNot() {
    const Limits l = elbowOnly();
    const Vec2 target{-8.0, 0.0};
    const Angles start{{-60.0 * kDeg, 45.0 * kDeg, 0.0, 0.0}};
    assert(inLimits(start, l));

    Angles unlimited = start;
    solve(kArm, unlimited, target, nullptr, 1e-9, 300);
    assert(tipError(kArm, unlimited, target) < 1e-6);
    assert(unlimited[1] < 0.0);  // the precondition of the story: the free solution bends the forbidden way

    const Angles end = clampAll(unlimited, l);
    assert(inLimits(end, l));
    assert(tipError(kArm, end, target) > 1.0);  // far off (3.09 when measured)

    Angles step = start;
    solve(kArm, step, target, &l, 1e-9, 300);
    assert(inLimits(step, l));
    assert(tipError(kArm, step, target) < 1e-6);
}

// A straight arm and a target on its line, nearer than the tip: at the joints 0 to 2 the tip and the target are in one
// line, so those turns are 0. Without limits the last joint still escapes, the target lies behind it and it turns by 180
// degrees. With the wrist joints pinned at 0 nothing may turn, the arm stays straight and 2 short (10 - 8). This is the local
// minimum of clamping every step: a pose that reaches exists, the solver just does not find it.
void straightArmStaysStraight() {
    const Limits l = elbowOnly();
    const Vec2 target{8.0, 0.0};
    Angles unlimited{{0, 0, 0, 0}};
    assert(solve(kArm, unlimited, target, nullptr, 1e-9, 100).error < 1e-6);
    Angles clamped{{0, 0, 0, 0}};
    const Solve b = solve(kArm, clamped, target, &l, 1e-9, 100);
    assert(near(b.error, 2.0));
    assert(b.iterations == 1);  // one sweep that moves nothing, then it knows it is standing still
    // Bending first does not help: the elbow wants to swing the forbidden way, is stopped at 0 and the arm ends straight again.
    Angles bent{{0.3, 1.0, 0, 0}};
    const Solve c = solve(kArm, bent, target, &l, 1e-9, 300);
    assert(near(c.error, 2.0));
    assert(bent[1] == 0.0);
    // Yet a pose inside the limits that reaches the target exists: elbow acos(0.25) = 75.52 degrees, shoulder -phi with
    // cos(phi) = 5.5 / 8 (the 4 + 6 cos(beta) = 5.5 part of the triangle with the forearm 3 + 2 + 1 = 6 long).
    const double beta = std::acos(0.25);
    const double phi = std::atan2(6.0 * std::sin(beta), 4.0 + 6.0 * std::cos(beta));
    const Angles reaching{{-phi, beta, 0.0, 0.0}};
    assert(inLimits(reaching, l));
    assert(tipError(kArm, reaching, target) < 1e-9);
}

// The same miss with the numbers worked out by hand. Both wrist joints are pinned at 0, so the arm is 4 + 6 long. The mirrored
// solution (elbow -75.52 degrees) clamped to 0 is a straight arm of length 10 along the shoulder angle. The shoulder was
// at target angle + phi with cos(phi) = 5.5 / 8 (the 4 + 6 cos(beta) = 5.5 part of the triangle), so the straight tip
// misses by sqrt(10^2 + 8^2 - 2 * 10 * 8 * 0.6875) = sqrt(54).
void handComputedMiss() {
    const Limits l = elbowOnly();
    const double beta = std::acos(0.25);  // elbow of the mirrored solution, 75.52 degrees
    const double phi = std::atan2(6.0 * std::sin(-beta), 4.0 + 6.0 * std::cos(-beta));
    assert(near(std::cos(phi), 5.5 / 8.0, 1e-12));
    const double targetAngle = 0.0;
    const Vec2 target{8.0, 0.0};
    const Angles mirrored{{targetAngle - phi, -beta, 0.0, 0.0}};
    assert(tipError(kArm, mirrored, target) < 1e-9);  // the forbidden pose does reach
    const Angles clamped = clampAll(mirrored, l);
    assert(near(clamped[1], 0.0));
    assert(near(tipError(kArm, clamped, target), std::sqrt(54.0), 1e-9));
}

// Over a set of targets the per-step clamp is never worse than the end clamp, and the sum shows the gap.
void stepClampBeatsEndClamp() {
    const Limits l = bounded();
    const Angles rest = clampAll({{-0.3, 0.7, 0.0, 0.0}}, l);
    double sumStep = 0.0;
    double sumEnd = 0.0;
    int worse = 0;
    int n = 0;
    for (int ring = 1; ring <= 3; ++ring) {
        const double radius = 2.5 * ring;
        for (int k = 0; k < 36; ++k) {
            const double ang = 2.0 * kPi * k / 36.0;
            const Vec2 t{radius * std::cos(ang), radius * std::sin(ang)};
            const Angles step = pose(Mode::Step, kArm, rest, t, l, 1e-6, 100);
            const Angles end = pose(Mode::End, kArm, rest, t, l, 1e-6, 100);
            const double es = tipError(kArm, step, t);
            const double ee = tipError(kArm, end, t);
            sumStep += es;
            sumEnd += ee;
            if (es > ee + 1e-6) ++worse;
            ++n;
        }
    }
    std::printf("step vs end over %d targets: mean error %.3f vs %.3f, step worse in %d\n", n, sumStep / n, sumEnd / n, worse);
    assert(sumStep < sumEnd);
}

// Targets that the arm can reach inside its box (made with forward kinematics from random poses inside the box): the clamped solver
// finds almost all of them from the rest pose. "Almost": CCD can get stuck, the count is printed.
void reachableTargetsAreFound() {
    const Limits l = bounded();
    const Angles rest = clampAll({{-0.3, 0.7, 0.0, 0.0}}, l);
    Lcg rng;
    int found = 0;
    int foundEnd = 0;
    const int total = 500;
    for (int i = 0; i < total; ++i) {
        const Angles truth = randomInside(l, rng);
        const Vec2 t = tip(kArm, truth);
        const Angles step = pose(Mode::Step, kArm, rest, t, l, 1e-3, 200);
        assert(inLimits(step, l));
        if (tipError(kArm, step, t) < 1e-2) ++found;
        const Angles end = pose(Mode::End, kArm, rest, t, l, 1e-3, 200);
        if (tipError(kArm, end, t) < 1e-2) ++foundEnd;
    }
    std::printf("targets inside the box reached from rest: step %d of %d, end %d of %d\n", found, total, foundEnd, total);
    assert(found > foundEnd);
}

void offModeIgnoresTheBox() {
    const Limits l = bounded();
    const Angles rest = clampAll({{-0.3, 0.7, 0.0, 0.0}}, l);
    const Vec2 t{-6, 2};
    const Angles off = pose(Mode::Off, kArm, rest, t, l, 1e-9, 300);
    assert(tipError(kArm, off, t) < 1e-6);
    // The first joint of the result is outside its range [-110, 70] degrees: the target is on the other side.
    assert(!inLimits(off, l));
}

void mapCells() {
    Map m = makeMap({-10, -10}, 2.0, 10, 10);  // 10 x 10 cells of 2, covering [-10, 10] on both axes
    assert(m.cols == 10 && m.rows == 10 && m.error.size() == 100);
    assert(near(cellCenter(m, 0, 0), {-9, -9}));
    assert(near(cellCenter(m, 9, 9), {9, 9}));
    assert(near(cellCenter(m, 5, 2), {1, -5}));
    const Limits l = bounded();
    const Angles rest = clampAll({{-0.3, 0.7, 0.0, 0.0}}, l);
    computeMap(m, Mode::Off, kArm, rest, l, 1e-3, 100);
    // Centre (1, 1) is 1.41 from the root, inside the reach: reached. Centre (9, 9) is 12.7 from the root: 2.73 short of 10.
    assert(m.error[static_cast<std::size_t>(5 * 10 + 5)] < 1e-3);
    assert(near(m.error[static_cast<std::size_t>(9 * 10 + 9)], std::sqrt(162.0) - 10.0, 1e-3));
    const int off = reachedCells(m, 1e-2);
    const int possible = reachableCells(m, kArm);
    // Without limits every cell up to 80% of the reach is found; close to 10 the arm is nearly straight and CCD creeps, a
    // few cells are not reached within 100 sweeps.
    for (int row = 0; row < m.rows; ++row) {
        for (int col = 0; col < m.cols; ++col) {
            const double dist = length(cellCenter(m, col, row));
            const float e = m.error[static_cast<std::size_t>(row * m.cols + col)];
            if (dist <= 8.0) assert(e < 1e-2);
            if (dist > 10.0) assert(e > 1e-2);
        }
    }
    assert(off <= possible);
    computeMap(m, Mode::Step, kArm, rest, l, 1e-3, 100);
    const int step = reachedCells(m, 1e-2);
    computeMap(m, Mode::End, kArm, rest, l, 1e-3, 100);
    const int end = reachedCells(m, 1e-2);
    std::printf("map cells within the reach %d, reached: off %d, step %d, end %d\n", possible, off, step, end);
    assert(step < off);   // the box takes away part of the workspace
    assert(end < step);   // and clamping at the end loses more of it
}

void draggingBorders() {
    const Limits l = armLimits();  // shoulder [-110, 70], elbow [0, 150]
    {
        const Limits d = dragLimit(l, 1, true, 100.0 * kDeg);  // upper border of the elbow to 100 degrees
        assert(near(d.hi[1], 100.0 * kDeg) && near(d.lo[1], 0.0));
    }
    {
        const Limits d = dragLimit(l, 1, true, -20.0 * kDeg);  // pulled below the lower border: the range closes, never flips
        assert(near(d.hi[1], d.lo[1]) && near(d.lo[1], 0.0));
    }
    {
        const Limits d = dragLimit(l, 1, false, 170.0 * kDeg);  // lower border pushed over the upper one (150)
        assert(near(d.lo[1], 150.0 * kDeg) && near(d.hi[1], 150.0 * kDeg));
    }
    {
        // The upper border of the shoulder (70) dragged round the back, the mouse angle jumps from 179 to -179 degrees: it stops at pi.
        const Limits d = dragLimit(l, 0, true, -179.0 * kDeg);
        assert(near(d.hi[0], kPi) && near(d.lo[0], -110.0 * kDeg));
        const Limits e = dragLimit(l, 0, false, 179.0 * kDeg);  // the lower border (-110) the other way round
        assert(near(e.lo[0], -kPi) && near(e.hi[0], 70.0 * kDeg));
    }
    {
        const Limits d = dragLimit(l, 2, true, 3.0);  // other joints stay as they were
        assert(near(d.hi[2], 3.0) && near(d.hi[1], l.hi[1]) && near(d.lo[2], l.lo[2]));
    }
}

void presets() {
    for (const Limits& l : {armLimits(), stiffLimits()}) {
        for (std::size_t i = 0; i < kJoints; ++i) {
            assert(l.lo[i] <= l.hi[i]);
            assert(l.lo[i] >= -kPi && l.hi[i] <= kPi);
        }
        assert(inLimits(restPose(l), l));
    }
    const Angles rest = restPose(armLimits());
    assert(near(rest[0], -20.0 * kDeg) && near(rest[1], 45.0 * kDeg) && near(rest[2], 15.0 * kDeg) && near(rest[3], 10.0 * kDeg));
    const Angles stiff = restPose(stiffLimits());  // 45 and 15 degrees do not fit into +-30, the elbow is held at 30
    assert(near(stiff[1], 30.0 * kDeg) && near(stiff[2], 15.0 * kDeg));
}

void demoPath() {
    const Vec2 root{100, 200};
    const Vec2 first = autoTarget(root, 10.0, 0.0);
    assert(near(first, {100 + 8.0 * std::cos(-1.1), 200 + 8.0 * std::sin(-1.1)}));
    assert(near(autoTarget(root, 10.0, kAutoHold - 0.001), first));  // still on the first spot
    assert(!near(autoTarget(root, 10.0, kAutoHold + 0.001), first));  // the second one
    assert(near(autoTarget(root, 10.0, -5.0), first));                // before the start: the first
    assert(near(autoTarget(root, 10.0, std::nan("")), first));        // garbage: the first
    assert(near(autoTarget(root, 10.0, 1e9), autoTarget(root, 10.0, kAutoPeriod - 0.001)));  // past the end: the last
    assert(near(advancePhase(19.9, 0.2), 0.1));
    assert(near(advancePhase(0.5, -1.0), kAutoPeriod - 0.5));
    double phase = 0.0;
    for (int i = 0; i < 100000; ++i) {
        phase = advancePhase(phase, 1.0 / 60.0);
        assert(phase >= 0.0 && phase < kAutoPeriod);
    }
}

}  // namespace

int main() {
    std::setvbuf(stdout, nullptr, _IONBF, 0);  // keep the printed numbers when an assert aborts
    wrapping();
    clampingOneAngle();
    clampingAllAngles();
    forwardKinematics();
    reaches();
    freeSolveReachesAndKeepsLengths();
    freeSolveBeyondTheReach();
    targetOnTheRoot();
    singleSweepClampsEveryJoint();
    limitsHoldAtTheEnd();
    clampAtTheEndMissesTheStepClampDoesNot();
    handComputedMiss();
    straightArmStaysStraight();
    stepClampBeatsEndClamp();
    reachableTargetsAreFound();
    offModeIgnoresTheBox();
    mapCells();
    draggingBorders();
    presets();
    demoPath();
    std::printf("joint-limits: all tests passed\n");
    return 0;
}
