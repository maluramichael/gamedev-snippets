// Tests for the Jansen linkage logic. Plain asserts on hand-computed cases, no framework, no SDL.
// Coordinates are "math" form: x to the right, y up. The pivot is the origin.
#undef NDEBUG
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <optional>

#include "logic.hpp"

using namespace jl;

namespace {

constexpr double kTwoPi = 6.283185307179586;

bool near(double a, double b, double eps = 1e-9) { return std::fabs(a - b) < eps; }
bool near(Vec2 a, Vec2 b, double eps = 1e-9) { return near(a.x, b.x, eps) && near(a.y, b.y, eps); }
double dist(Vec2 a, Vec2 b) { return std::hypot(a.x - b.x, a.y - b.y); }

void circleCircleHandComputed() {
    // 3-4-5 triangles: circles r=5 around (0,0) and (6,0) meet at (3,4) and (3,-4).
    // Side +1 is the point to the LEFT of the line from the first to the second center.
    auto p = circleCircle({0, 0}, 5, {6, 0}, 5, +1);
    assert(p && near(*p, {3, 4}));
    p = circleCircle({0, 0}, 5, {6, 0}, 5, -1);
    assert(p && near(*p, {3, -4}));
    // Swap the centers and "left" swaps with them.
    p = circleCircle({6, 0}, 5, {0, 0}, 5, +1);
    assert(p && near(*p, {3, -4}));
    // 5-12-13 and 9-12-15 around (0,0) r=13 and (14,0) r=15: they meet at (5, 12).
    p = circleCircle({0, 0}, 13, {14, 0}, 15, +1);
    assert(p && near(*p, {5, 12}));
    // Not axis aligned: r=5 around (0,0) and (4.8,3.6), distance 6. Left of the line is (0,5), right is (4.8,-1.4).
    p = circleCircle({0, 0}, 5, {4.8, 3.6}, 5, +1);
    assert(p && near(*p, {0, 5}));
    p = circleCircle({0, 0}, 5, {4.8, 3.6}, 5, -1);
    assert(p && near(*p, {4.8, -1.4}));
    // Both results lie on both circles.
    assert(near(dist(*p, {0, 0}), 5.0) && near(dist(*p, {4.8, 3.6}), 5.0));
    // Offset centers: (1,1) r=5 and (1,9) r=5, distance 8, the points are (1 -+ 3, 5). Left of an upward line is -x.
    p = circleCircle({1, 1}, 5, {1, 9}, 5, +1);
    assert(p && near(*p, {-2, 5}));
    p = circleCircle({1, 1}, 5, {1, 9}, 5, -1);
    assert(p && near(*p, {4, 5}));
}

void circleCircleTouching() {
    // Touching from outside: one point, both sides agree, and no NaN from a tiny negative h^2.
    for (int side : {+1, -1}) {
        const auto p = circleCircle({0, 0}, 1, {2, 0}, 1, side);
        assert(p && near(*p, {1, 0}));
    }
    // Touching from inside: r=3 around (0,0) and r=2 around (1,0) meet at (3,0).
    const auto q = circleCircle({0, 0}, 3, {1, 0}, 2, +1);
    assert(q && near(*q, {3, 0}));
    // A radius of 0 is a point: it lies on the other circle exactly when the distance equals that radius.
    const auto r = circleCircle({0, 0}, 0, {3, 0}, 3, +1);
    assert(r && near(*r, {0, 0}));
}

void circleCircleFailures() {
    assert(!circleCircle({0, 0}, 1, {3, 0}, 1, +1));  // too far apart: d = 3 > r0 + r1 = 2
    assert(!circleCircle({0, 0}, 1, {2.0000001, 0}, 1, +1));  // just too far
    assert(!circleCircle({0, 0}, 5, {1, 0}, 1, +1));  // one inside the other: d = 1 < |r0 - r1| = 4
    assert(!circleCircle({1, 0}, 1, {0, 0}, 5, -1));
    assert(!circleCircle({2, 2}, 3, {2, 2}, 3, +1));  // same center, same radius: infinitely many points, none is "the" answer
    assert(!circleCircle({2, 2}, 3, {2, 2}, 4, +1));  // same center, different radius: none
    assert(!circleCircle({0, 0}, -1, {1, 0}, 1, +1));  // negative radius is not a circle
    assert(!circleCircle({0, 0}, 1, {1, 0}, -1, +1));
}

void sideOfHandComputed() {
    assert(sideOf({0, 0}, {6, 0}, {3, 4}) == +1);
    assert(sideOf({0, 0}, {6, 0}, {3, -4}) == -1);
    assert(sideOf({6, 0}, {0, 0}, {3, 4}) == -1);  // direction reversed, the same point is on the right now
    assert(sideOf({0, 0}, {6, 0}, {3, 0}) == +1);  // on the line both answers coincide, +1 by convention
    // sideOf and circleCircle agree: the point you get for a side is on that side.
    for (int side : {+1, -1}) {
        const auto p = circleCircle({0, 0}, 5, {4.8, 3.6}, 5, side);
        assert(p && sideOf({0, 0}, {4.8, 3.6}, *p) == side);
    }
}

void wrapping() {
    assert(near(wrapAngle(0.0), 0.0));
    assert(near(wrapAngle(kTwoPi), 0.0));
    assert(near(wrapAngle(-0.5), kTwoPi - 0.5));
    assert(near(wrapAngle(3 * kTwoPi + 1.0), 1.0));
    assert(wrapAngle(-1e-18) < kTwoPi);  // never returns 2 pi itself
    assert(wrapAngle(-1e-18) >= 0.0);
}

void lengthsMatchTheWikipediaTable() {
    // Segment lengths of Jansen's linkage, table "Jansen Linkage Ratios" on en.wikipedia.org/wiki/Jansen%27s_linkage.
    const Lengths L = standardLengths();
    const std::array<double, 13> table{38, 41.5, 39.3, 40.1, 55.8, 39.4, 36.7, 65.7, 49, 50, 61.9, 7.8, 15};
    const std::array<double, 13> mine{L.a, L.b, L.c, L.d, L.e, L.f, L.g, L.h, L.i, L.j, L.k, L.l, L.m};
    for (std::size_t i = 0; i < table.size(); ++i) assert(near(table[i], mine[i]));
}

void linkTableIsAlphabetical() {
    // The chips and the scale array use this order.
    const char names[] = "bcdefghijkm";
    for (std::size_t i = 0; i < kLinks.size(); ++i) assert(kLinks[i].name == names[i]);
    assert(static_cast<int>(kLinks.size()) == kRodCount);
    // Every link points to a different pair of joints.
    for (std::size_t i = 0; i < kLinks.size(); ++i) {
        for (std::size_t j = i + 1; j < kLinks.size(); ++j) {
            const bool same = kLinks[i].from == kLinks[j].from && kLinks[i].to == kLinks[j].to;
            assert(!same);
        }
    }
}

void referencePoseMatchesTheDrawing() {
    // The pose read off the SVG "Strandbeest Leg Proportions" (two decimals). Its distances reproduce the table,
    // which is a second, independent check of both the table and of which joints each rod connects.
    const Lengths L = standardLengths();
    const Pose ref = referencePose();
    for (const Link& link : kLinks) {
        assert(near(dist(ref.*link.from, ref.*link.to), L.*link.length, 0.03));
    }
    // The frame: the crank center sits 38 right of and 7.8 above the pivot.
    assert(near(ref.pivot, {0, 0}) && near(ref.hub, {38, 7.8}, 1e-9));
    assert(maxRodError(L, ref) < 0.03);
}

void solveReproducesTheReferencePose() {
    const Lengths L = standardLengths();
    const Pose ref = referencePose();
    const Solved s = solve(L, referenceSides(), referenceAngle());
    assert(s.ok && s.failed == -1);
    assert(near(s.pose.pivot, ref.pivot) && near(s.pose.hub, ref.hub));
    assert(near(s.pose.crank, ref.crank, 0.01));
    assert(near(s.pose.b, ref.b, 0.03));
    assert(near(s.pose.c, ref.c, 0.03));
    assert(near(s.pose.d, ref.d, 0.03));
    assert(near(s.pose.e, ref.e, 0.03));
    assert(near(s.pose.foot, ref.foot, 0.05));
    // Hand check of the first joint: B is 41.5 from the pivot and 50 from the crank tip.
    assert(near(dist(s.pose.b, s.pose.pivot), 41.5) && near(dist(s.pose.b, s.pose.crank), 50.0));
}

void crankAngleMovesTheCrankTip() {
    const Lengths L = standardLengths();
    const Sides sides = referenceSides();
    // Angle 0: the tip is 15 to the right of the hub. Angle pi/2: 15 above it.
    const Solved s0 = solve(L, sides, 0.0);
    assert(s0.ok && near(s0.pose.crank, {38 + 15, 7.8}));
    const Solved s1 = solve(L, sides, kTwoPi / 4);
    assert(s1.ok && near(s1.pose.crank, {38, 7.8 + 15}));
}

void everyRodKeepsItsLengthAtEveryAngle() {
    const Lengths L = standardLengths();
    const Sides sides = referenceSides();
    for (int k = 0; k < 720; ++k) {
        const Solved s = solve(L, sides, kTwoPi * k / 720.0);
        assert(s.ok);
        assert(maxRodError(L, s.pose) < 1e-9);
        // And per link, not just the maximum: this is the "11 distances" check.
        for (const Link& link : kLinks) assert(near(dist(s.pose.*link.from, s.pose.*link.to), L.*link.length));
    }
}

void rodsKeepTheirLengthAfterScaling() {
    // Scaling a rod changes the target, the solver must still hit it wherever it can assemble.
    Scale scale = unitScale();
    scale[7] = 1.1;  // rod 'i'
    scale[3] = 0.95;  // rod 'e'
    const Lengths L = scaled(scale);
    assert(near(L.i, 49 * 1.1) && near(L.e, 55.8 * 0.95) && near(L.h, 65.7));
    int solved = 0;
    for (int k = 0; k < 360; ++k) {
        const Solved s = solve(L, referenceSides(), kTwoPi * k / 360.0);
        if (!s.ok) continue;
        ++solved;
        assert(maxRodError(L, s.pose) < 1e-9);
    }
    assert(solved == 360);
}

void pathIsClosed() {
    const Lengths L = standardLengths();
    const Sides sides = referenceSides();
    for (double start : {0.0, 1.0, 2.5}) {
        const Solved a = solve(L, sides, start);
        const Solved b = solve(L, sides, start + kTwoPi);
        assert(a.ok && b.ok);
        assert(near(a.pose.foot, b.pose.foot) && near(a.pose.e, b.pose.e) && near(a.pose.c, b.pose.c));
    }
    // The first and the last sample of a path are one step apart, not a jump.
    const Path path = footPath(L, sides, 360);
    assert(path.size() == 360 && fullTurn(path));
    assert(dist(*path.front(), *path.back()) < 1.0);
}

void footStaysTheLowestJoint() {
    // In the walking pose the foot hangs under everything else, all the way round.
    const Lengths L = standardLengths();
    for (int k = 0; k < 360; ++k) {
        const Solved s = solve(L, referenceSides(), kTwoPi * k / 360.0);
        assert(s.ok);
        for (const Vec2 q : {s.pose.pivot, s.pose.crank, s.pose.b, s.pose.c, s.pose.d, s.pose.e}) assert(s.pose.foot.y < q.y);
    }
}

void failuresNameTheJoint() {
    // Make exactly one rod absurd and check which joint reports it. The joints are solved in the order B C D E foot.
    const struct {
        char rod;
        int joint;
    } cases[] = {{'b', 0}, {'e', 1}, {'c', 2}, {'f', 3}, {'h', 4}};
    for (const auto& c : cases) {
        Lengths L = standardLengths();
        for (const Link& link : kLinks) {
            if (link.name == c.rod) L.*link.length = 1.0;
        }
        const Solved s = solve(L, referenceSides(), referenceAngle());
        assert(!s.ok);
        assert(s.failed == c.joint);
    }
}

void thirtyTwoAssemblies() {
    // Every joint has two intersection points, so there are 2^5 = 32 ways to put the legs together. Measured:
    // all 32 assemble in the start pose, 16 of them also survive a whole revolution, 16 do not.
    const Lengths L = standardLengths();
    int atStart = 0;
    int fullRevolution = 0;
    for (int bits = 0; bits < 32; ++bits) {
        Sides sides{};
        for (int j = 0; j < 5; ++j) sides[static_cast<std::size_t>(j)] = (bits >> j) & 1 ? +1 : -1;
        if (solve(L, sides, referenceAngle()).ok) ++atStart;
        if (fullTurn(footPath(L, sides, 360))) ++fullRevolution;
    }
    assert(atStart == 32);
    assert(fullRevolution == 16);
}

void referenceSidesAreTheOnlyLegThatWalks() {
    // The naive choice (first intersection for every joint) assembles too, but with D and E on the wrong side:
    // the foot ends up near the body instead of 84 below the pivot.
    const Lengths L = standardLengths();
    const Solved good = solve(L, referenceSides(), referenceAngle());
    const Solved naive = solve(L, naiveSides(), referenceAngle());
    assert(good.ok && naive.ok);
    assert(dist(good.pose.foot, naive.pose.foot) > 50.0);
    assert(good.pose.foot.y < -80.0 && naive.pose.foot.y > -20.0);
    // They agree on B and C (same sides there) and differ on D.
    assert(near(good.pose.b, naive.pose.b) && near(good.pose.c, naive.pose.c));
    assert(dist(good.pose.d, naive.pose.d) > 10.0);
}

void groundContact() {
    const Lengths L = standardLengths();
    const Path path = footPath(L, referenceSides(), 360);
    const auto level = groundLevel(path);
    assert(level);
    // The lowest point of the path: measured about -84.03 (the pose of the drawing has the foot at -83.98).
    assert(*level < -83.9 && *level > -84.2);
    const auto share = groundContact(path, 1.0);
    assert(share);
    assert(*share > 0.42 && *share < 0.44);  // measured 155 of 360 samples
    // A tolerance of 0 counts nothing (strictly below the ground level never happens), a huge one counts everything.
    assert(near(*groundContact(path, 0.0), 0.0));
    assert(near(*groundContact(path, 1000.0), 1.0));
    // The more tolerance, the more contact.
    assert(*groundContact(path, 0.5) < *groundContact(path, 2.0));
}

void pathWithGapsHasNoStats() {
    Scale scale = unitScale();
    scale[1] = 1.2;  // rod 'c' +20 %: the crank can no longer turn all the way round
    const Lengths L = scaled(scale);
    const Path path = footPath(L, referenceSides(), 360);
    assert(!fullTurn(path));
    assert(!groundContact(path, 1.0));
    assert(!groundLevel(Path{}));
    const auto reachable = std::count_if(path.begin(), path.end(), [](const auto& p) { return p.has_value(); });
    assert(reachable > 100 && reachable < 360);  // measured 220 of 360
}

void advanceStopsAtTheLimit() {
    const Lengths L = standardLengths();
    double angle = 1.0;
    assert(advance(L, referenceSides(), angle, 3 * kTwoPi));  // standard rods: any distance is fine
    assert(near(angle, wrapAngle(1.0 + 3 * kTwoPi), 1e-6));
    angle = 1.0;
    assert(advance(L, referenceSides(), angle, -kTwoPi));
    assert(near(angle, 1.0, 1e-6));
    assert(advance(L, referenceSides(), angle, 0.0));

    // With rod 'c' 20 % longer the crank rocks: a turn is cut short, the angle stays where it still assembles.
    Scale scale = unitScale();
    scale[1] = 1.2;
    const Lengths jammed = scaled(scale);
    const auto start = nearestReachable(jammed, referenceSides(), 0.0);
    assert(start);
    angle = *start;
    assert(!advance(jammed, referenceSides(), angle, kTwoPi));
    assert(solve(jammed, referenceSides(), angle).ok);
    // Turning back from the stop works, going on does not.
    double again = angle;
    assert(!advance(jammed, referenceSides(), again, 0.05));
    assert(near(again, angle, 0.01));
    assert(advance(jammed, referenceSides(), again, -0.05));
}

void nearestReachableFindsAValidAngle() {
    const Lengths L = standardLengths();
    // Everything is reachable: the angle comes back unchanged (wrapped).
    const auto same = nearestReachable(L, referenceSides(), 2.0);
    assert(same && near(*same, 2.0));
    const auto wrapped = nearestReachable(L, referenceSides(), 2.0 + kTwoPi);
    assert(wrapped && near(*wrapped, 2.0, 1e-6));

    Scale scale = unitScale();
    scale[1] = 1.2;
    const Lengths jammed = scaled(scale);
    // Find an angle that does not assemble, then ask for the nearest one that does.
    double bad = -1.0;
    for (int k = 0; k < 360 && bad < 0; ++k) {
        const double a = kTwoPi * k / 360.0;
        if (!solve(jammed, referenceSides(), a).ok) bad = a;
    }
    assert(bad >= 0.0);
    const auto found = nearestReachable(jammed, referenceSides(), bad);
    assert(found && solve(jammed, referenceSides(), *found).ok);
    // Nothing assembles at all: no answer.
    Lengths broken = standardLengths();
    broken.b = 1.0;
    assert(!nearestReachable(broken, referenceSides(), 0.0));
}

void footRodsNeverJam() {
    // Rods h and i only place the foot, no other joint hangs on them: the crank turns all the way round for any length.
    for (std::size_t index : {std::size_t{6}, std::size_t{7}}) {
        assert(kLinks[index].name == 'h' || kLinks[index].name == 'i');
        for (double factor : {0.8, 0.9, 1.1, 1.2}) {
            Scale scale = unitScale();
            scale[index] = factor;
            assert(fullTurn(footPath(scaled(scale), referenceSides(), 360)));
        }
    }
    // Rod h 9 percent longer: the path gets taller and the foot spends less time on the ground (what the demo shows).
    Scale scale = unitScale();
    scale[6] = 1.09;
    const Path path = footPath(scaled(scale), referenceSides(), 360);
    const Path base = footPath(standardLengths(), referenceSides(), 360);
    assert(fullTurn(path) && fullTurn(base));
    double lowest = 1e9, highest = -1e9, baseLowest = 1e9, baseHighest = -1e9;
    for (const auto& p : path) lowest = std::min(lowest, p->y), highest = std::max(highest, p->y);
    for (const auto& p : base) baseLowest = std::min(baseLowest, p->y), baseHighest = std::max(baseHighest, p->y);
    assert(baseHighest - baseLowest > 21.5 && baseHighest - baseLowest < 22.5);  // measured 22.46, the demo shows 22
    assert(highest - lowest > 33.5 && highest - lowest < 34.5);                    // measured 34 (rounded), the demo shows 34
    assert(*groundContact(path, 1.0) < *groundContact(base, 1.0));
}

void scaleIsPerRod() {
    Scale scale = unitScale();
    for (double s : scale) assert(near(s, 1.0));
    const Lengths base = standardLengths();
    const Lengths same = scaled(scale);
    for (const Link& link : kLinks) assert(near(base.*link.length, same.*link.length));
    scale[0] = 0.8;  // 'b'
    const Lengths L = scaled(scale);
    assert(near(L.b, 41.5 * 0.8) && near(L.c, 39.3) && near(L.m, 15.0));
    assert(near(L.a, 38.0) && near(L.l, 7.8));  // the frame never scales
}

}  // namespace

int main() {
    circleCircleHandComputed();
    circleCircleTouching();
    circleCircleFailures();
    sideOfHandComputed();
    wrapping();
    lengthsMatchTheWikipediaTable();
    linkTableIsAlphabetical();
    referencePoseMatchesTheDrawing();
    solveReproducesTheReferencePose();
    crankAngleMovesTheCrankTip();
    everyRodKeepsItsLengthAtEveryAngle();
    rodsKeepTheirLengthAfterScaling();
    pathIsClosed();
    footStaysTheLowestJoint();
    failuresNameTheJoint();
    thirtyTwoAssemblies();
    referenceSidesAreTheOnlyLegThatWalks();
    groundContact();
    pathWithGapsHasNoStats();
    advanceStopsAtTheLimit();
    nearestReachableFindsAValidAngle();
    footRodsNeverJam();
    scaleIsPerRod();
    std::puts("jansen-linkage logic: all asserts passed");
    return 0;
}
