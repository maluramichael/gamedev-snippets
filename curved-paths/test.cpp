// Tests for the curved-paths logic. Plain asserts on hand-computed cases, no framework, no SDL.
// Coordinates are written in "math" form (y up, left turn = counter-clockwise); the logic does not care how it is drawn.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "logic.hpp"

using namespace cp;

namespace {

bool near(double a, double b, double tol = 1e-9) { return std::fabs(a - b) < tol; }
bool nearV(Vec2 a, Vec2 b, double tol = 1e-9) { return near(a.x, b.x, tol) && near(a.y, b.y, tol); }
bool finite(Vec2 v) { return std::isfinite(v.x) && std::isfinite(v.y); }

// Right angle, heading east then north: control polygon (0,0) (10,0) (10,10).
const Bezier kCorner{{0, 0}, {10, 0}, {10, 10}};
// Parabola y = x - x^2/4 for x in 0..4: (0,0) (2,2) (4,0) gives x = 4t and y = 4t(1-t).
const Bezier kParabola{{0, 0}, {2, 2}, {4, 0}};
// Control point on the start: B(t) = (10 t^2, 0), a straight road that starts slowly.
const Bezier kSlowStart{{0, 0}, {0, 0}, {10, 0}};

void vectorBasics() {
    assert(near(dot({1, 2}, {3, 4}), 11.0));
    assert(near(cross({1, 0}, {0, 1}), 1.0));
    assert(nearV(leftNormal({1, 0}), {0, 1}));
    assert(nearV(leftNormal({0, 1}), {-1, 0}));
    assert(nearV(normalizedOr({3, 4}, {1, 0}), {0.6, 0.8}));
    assert(nearV(normalizedOr({0, 0}, {0, 1}), {0, 1}));  // a zero vector has no direction: the fallback is used
}

void bezierPointsAndTangents() {
    assert(nearV(bezierPoint(kParabola, 0.0), {0, 0}));
    assert(nearV(bezierPoint(kParabola, 0.5), {2, 1}));  // (0 + 2*2 + 4) / 4, (0 + 2*2 + 0) / 4
    assert(nearV(bezierPoint(kParabola, 1.0), {4, 0}));
    assert(nearV(bezierDerivative(kParabola, 0.0), {4, 4}));  // 2 (C - P0)
    assert(nearV(bezierDerivative(kParabola, 1.0), {4, -4}));  // 2 (P1 - C)
    assert(nearV(bezierDerivative(kParabola, 0.5), {4, 0}));
    assert(nearV(bezierTangent(kParabola, 0.5), {1, 0}));
    assert(nearV(bezierNormal(kParabola, 0.5), {0, 1}));  // left of travel
    // The derivative is zero at t = 0 when the control point sits on the start: fall back to the chord.
    assert(nearV(bezierDerivative(kSlowStart, 0.0), {0, 0}));
    assert(nearV(bezierTangent(kSlowStart, 0.0), {1, 0}));
    // All three points equal: no direction at all, still finite.
    const Bezier dot3{{5, 5}, {5, 5}, {5, 5}};
    assert(finite(bezierTangent(dot3, 0.5)));
    assert(finite(bezierOffsetPoint(dot3, 0.5, 2.0)));
}

void controlPointOffsetOfACorner() {
    // Offset 1 to the left (the inner side of the left turn): the end points move along their own normals, the control
    // point is the corner where the two shifted control lines meet.
    const Bezier o = offsetControlPoints(kCorner, 1.0);
    assert(nearV(o.p0, {0, 1}));
    assert(nearV(o.p1, {9, 10}));
    assert(nearV(o.c, {9, 1}));
    const Bezier right = offsetControlPoints(kCorner, -1.0);
    assert(nearV(right.p0, {0, -1}));
    assert(nearV(right.p1, {11, 10}));
    assert(nearV(right.c, {11, -1}));
}

void controlPointOffsetEdgeCases() {
    // Straight road, three points on a line: the miter is undefined, the control point just moves sideways.
    const Bezier line{{0, 0}, {5, 0}, {10, 0}};
    const Bezier o = offsetControlPoints(line, 2.0);
    assert(nearV(o.p0, {0, 2}) && nearV(o.c, {5, 2}) && nearV(o.p1, {10, 2}));
    // A U-turn (end direction opposite to the start direction) must not blow up either.
    const Bezier u{{0, 0}, {10, 0}, {0, 0.001}};
    assert(finite(offsetControlPoints(u, 1.0).c));
    // The sharp corner would put the control point very far away; the miter is capped at 6 times the offset.
    assert(length(offsetControlPoints(u, 1.0).c - u.c) <= 6.0 + 1e-9);
    assert(length(offsetControlPoints(u, 1.0).c - u.c) > 5.0);
    // Everything on one point.
    const Bezier dot3{{1, 1}, {1, 1}, {1, 1}};
    const Bezier od = offsetControlPoints(dot3, 3.0);
    assert(finite(od.p0) && finite(od.c) && finite(od.p1));
}

void edgeWidthOfTheNaiveOffset() {
    // At t = 0.5 the corner curve is at (7.5, 2.5) with tangent (1,1)/sqrt2. The true offset moves exactly 1 along the
    // normal (-1,1)/sqrt2. The shifted control points give 0.25*(0,1) + 0.5*(9,1) + 0.25*(9,10) = (6.75, 3.25), which is
    // 0.75*sqrt2 = 1.0607 away: the road is 6 percent too wide in the middle.
    const Vec2 centre = bezierPoint(kCorner, 0.5);
    assert(nearV(centre, {7.5, 2.5}));
    const Vec2 truth = bezierOffsetPoint(kCorner, 0.5, 1.0);
    assert(near(length(truth - centre), 1.0));
    const Vec2 naive = bezierPoint(offsetControlPoints(kCorner, 1.0), 0.5);
    assert(nearV(naive, {6.75, 3.25}));
    assert(near(length(naive - centre), 0.75 * std::sqrt(2.0)));
}

void segmentAndPolylineDistance() {
    assert(near(distanceToSegment({5, 3}, {0, 0}, {10, 0}), 3.0));
    assert(near(distanceToSegment({-3, 4}, {0, 0}, {10, 0}), 5.0));  // beyond the start: distance to the end point
    assert(near(distanceToSegment({13, 4}, {0, 0}, {10, 0}), 5.0));
    assert(near(distanceToSegment({5, 6}, {2, 2}, {2, 2}), 5.0));  // zero-length segment
    const std::vector<Vec2> l{{0, 0}, {10, 0}, {10, 10}};
    assert(near(distanceToPolyline({12, 5}, l), 2.0));
    assert(near(distanceToPolyline({5, 1}, l), 1.0));
    const std::vector<Vec2> single{{1, 1}};
    assert(near(distanceToPolyline({4, 5}, single), 5.0));
}

void widthErrorOfEdges() {
    const std::vector<Vec2> centre = bezierPolyline(kCorner, 256);
    // The true offset stays 1 away from the road everywhere (vertices and the middle of the segments).
    assert(maxWidthError(offsetPolyline(kCorner, 256, 1.0), centre, 1.0) < 5e-3);
    // The shifted control points are exactly 1 away at both ends and 1.0607 in the middle: about 0.06 too wide.
    const double naive = maxWidthError(bezierPolyline(offsetControlPoints(kCorner, 1.0), 64), centre, 1.0);
    assert(naive > 0.05 && naive < 0.07);
    // A coarse edge (4 segments) cuts corners: its segment middles are off the true offset.
    assert(maxWidthError(offsetPolyline(kCorner, 4, 1.0), centre, 1.0) > 0.05);
    // Empty input is no error.
    assert(near(maxWidthError({}, centre, 1.0), 0.0));
}

void polylineLength() {
    // Chord of the parabola: 4. Two segments: (0,0) (2,1) (4,0) is 2*sqrt5.
    assert(near(makeLengthTable(kParabola, 1).total(), 4.0));
    assert(near(makeLengthTable(kParabola, 2).total(), 2.0 * std::sqrt(5.0)));
    // Many segments: 2 (sqrt2 + asinh 1), the integral of sqrt(1 + (1 - x/2)^2) over 0..4.
    const double exact = 2.0 * (std::sqrt(2.0) + std::asinh(1.0));
    assert(near(exact, 4.5911743, 1e-6));
    assert(near(makeLengthTable(kParabola, 1024).total(), exact, 1e-4));
    // A polyline never overshoots: fewer segments, shorter length.
    assert(makeLengthTable(kParabola, 4).total() < makeLengthTable(kParabola, 16).total());
    assert(makeLengthTable(kParabola, 16).total() < exact);
    // Zero segments are not possible: clamped to one.
    assert(near(makeLengthTable(kParabola, 0).total(), 4.0));
}

void distanceToParameter() {
    // x = 10 t^2: after distance s the parameter is sqrt(s / 10).
    const LengthTable fine = makeLengthTable(kSlowStart, 1024);
    assert(near(fine.total(), 10.0, 1e-9));
    assert(near(tAtDistance(fine, 2.5), 0.5, 1e-4));
    assert(near(tAtDistance(fine, 0.0), 0.0));
    assert(near(tAtDistance(fine, 10.0), 1.0));
    assert(near(tAtDistance(fine, -1.0), 0.0));  // clamped
    assert(near(tAtDistance(fine, 99.0), 1.0));
    // Table with 4 segments: t = 0, .25, .5, .75, 1 at distances 0, .625, 2.5, 5.625, 10. On a table point the answer
    // is exact, between two points it is interpolated linearly: distance 1.5625 sits half way between .625 and 2.5 in
    // distance terms, so t = 0.375, while the true value is sqrt(0.15625) = 0.3953.
    const LengthTable coarse = makeLengthTable(kSlowStart, 4);
    assert(near(coarse.cum[2], 2.5) && near(coarse.cum[3], 5.625));
    assert(near(tAtDistance(coarse, 2.5), 0.5));
    assert(near(tAtDistance(coarse, 1.5625), 0.375));
    assert(near(std::sqrt(0.15625), 0.3953, 1e-4));
    // Nothing to measure: a curve that is a single point has length zero, t stays 0 and nothing is divided.
    const Bezier dot3{{2, 2}, {2, 2}, {2, 2}};
    const LengthTable none = makeLengthTable(dot3, 8);
    assert(near(none.total(), 0.0));
    assert(near(tAtDistance(none, 1.0), 0.0));
}

void evenStepsInDistanceNotInT() {
    // Eight equal steps in t on x = 10 t^2 cover 10 (2i+1) / 64: from 0.156 to 2.34, a factor of 15.
    double first = 0.0;
    double last = 0.0;
    for (int i = 0; i < 8; ++i) {
        const double a = static_cast<double>(i) / 8.0;
        const double b = static_cast<double>(i + 1) / 8.0;
        const double step = length(bezierPoint(kSlowStart, b) - bezierPoint(kSlowStart, a));
        if (i == 0) first = step;
        if (i == 7) last = step;
    }
    assert(near(first, 10.0 / 64.0));
    assert(near(last, 150.0 / 64.0));
    assert(near(last / first, 15.0));
    // Eight equal steps in distance through the table stay equal to within one percent.
    const LengthTable table = makeLengthTable(kSlowStart, 512);
    for (int i = 0; i < 8; ++i) {
        const double a = tAtDistance(table, table.total() * i / 8.0);
        const double b = tAtDistance(table, table.total() * (i + 1) / 8.0);
        const double step = length(bezierPoint(kSlowStart, b) - bezierPoint(kSlowStart, a));
        assert(near(step, 10.0 / 8.0, 0.0125));
    }
}

void speedRange() {
    // |B'(t)| on x = 10 t^2 is 20 t: from 0 to 20.
    const SpeedRange s = bezierSpeedRange(kSlowStart, 64);
    assert(near(s.min, 0.0));
    assert(near(s.max, 20.0));
    // A circle-like symmetric curve keeps the speed within the control polygon lengths: parabola 5.66 .. 4.
    const SpeedRange p = bezierSpeedRange(kParabola, 64);
    assert(near(p.max, 4.0 * std::sqrt(2.0)));  // at the ends |(4,4)|
    assert(near(p.min, 4.0));  // in the middle (4,0)
}

void arcQuarterTurn() {
    // From (0,0) heading east to (1,1): half of the turn angle is 45 degrees, so a quarter circle of radius 1 around (0,1).
    const Arc a = makeArc({0, 0}, {1, 0}, {1, 1});
    assert(a.valid && !a.line);
    assert(near(a.radius, 1.0));
    assert(nearV(a.center, {0, 1}));
    assert(near(a.sweep, kPi / 2.0));
    assert(near(a.length, kPi / 2.0));
    assert(nearV(arcPoint(a, 0.0), {0, 0}));
    assert(nearV(arcPoint(a, 1.0), {1, 1}));
    assert(nearV(arcPoint(a, 0.5), {std::sqrt(0.5), 1.0 - std::sqrt(0.5)}));
    assert(nearV(arcTangent(a, 0.0), {1, 0}));
    assert(nearV(arcTangent(a, 1.0), {0, 1}));
    assert(nearV(arcNormal(a, 0.0), {0, 1}));  // left of travel, towards the centre
    // A tangent that is not unit length gives the same arc.
    const Arc b = makeArc({0, 0}, {7, 0}, {1, 1});
    assert(near(b.radius, 1.0) && nearV(b.center, {0, 1}));
}

void arcOtherSides() {
    // Mirror image, a right turn: clockwise, centre on the right.
    const Arc cw = makeArc({0, 0}, {1, 0}, {1, -1});
    assert(cw.valid && near(cw.radius, 1.0));
    assert(nearV(cw.center, {0, -1}));
    assert(near(cw.sweep, -kPi / 2.0));
    assert(nearV(arcPoint(cw, 1.0), {1, -1}));
    assert(nearV(arcTangent(cw, 1.0), {0, -1}));
    // Half circle: heading north, end point 2 to the right. Radius 1, clockwise, length pi.
    const Arc half = makeArc({0, 0}, {0, 1}, {2, 0});
    assert(half.valid && near(half.radius, 1.0) && near(half.sweep, -kPi));
    assert(nearV(half.center, {1, 0}));
    assert(near(half.length, kPi));
    assert(nearV(arcPoint(half, 1.0), {2, 0}, 1e-9));
    // The arc really passes through the end point for an arbitrary case, with the right start tangent.
    const Arc any = makeArc({3, -2}, {0.3, 0.9}, {-4, 5});
    assert(any.valid && !any.line);
    assert(nearV(arcPoint(any, 1.0), {-4, 5}, 1e-9));
    assert(nearV(arcTangent(any, 0.0), normalizedOr({0.3, 0.9}, {1, 0}), 1e-9));
    assert(near(length(arcPoint(any, 0.37) - any.center), any.radius, 1e-9));
}

void arcDegenerateCases() {
    // End point straight ahead: a line, not an arc with an infinite radius.
    const Arc ahead = makeArc({0, 0}, {1, 0}, {5, 0});
    assert(ahead.valid && ahead.line);
    assert(near(ahead.length, 5.0));
    assert(nearV(arcPoint(ahead, 0.5), {2.5, 0}));
    assert(nearV(arcTangent(ahead, 0.5), {1, 0}));
    // So nearly ahead that the circle would be huge: still a line, nothing exploding.
    const Arc nearly = makeArc({0, 0}, {1, 0}, {500, 0.00001});
    assert(nearly.valid && nearly.line && near(nearly.length, 500.0, 1e-3));
    // No tangent at all: the chord decides, which is a line.
    const Arc noDir = makeArc({0, 0}, {0, 0}, {3, 4});
    assert(noDir.valid && noDir.line && near(noDir.length, 5.0));
    // Start equal to end: a point.
    const Arc same = makeArc({2, 2}, {1, 0}, {2, 2});
    assert(same.valid && same.line && near(same.length, 0.0));
    assert(finite(arcPoint(same, 0.5)) && finite(arcTangent(same, 0.5)));
    assert(near(arcDistanceOf(same, {9, 9}), 0.0));
    // End point behind the start: no circle fits.
    assert(!makeArc({0, 0}, {1, 0}, {-3, 0}).valid);
    assert(!makeArc({0, 0}, {1, 0}, {-3, 0.5}).valid);
    // The limit: a turn of 2 * 2.6 rad is still fine, one just past it is not.
    assert(makeArc({0, 0}, {1, 0}, {std::cos(2.5), std::sin(2.5)}).valid);
    assert(!makeArc({0, 0}, {1, 0}, {std::cos(2.7), std::sin(2.7)}).valid);
}

void arcOffsets() {
    const Arc ccw = makeArc({0, 0}, {1, 0}, {1, 1});  // left turn, radius 1
    // Left of a left turn is the inside: the radius shrinks. The result is again an arc around the same centre.
    const Arc inner = offsetArc(ccw, 0.25);
    assert(!inner.collapsed && near(inner.radius, 0.75) && nearV(inner.center, ccw.center));
    assert(near(inner.length, 0.75 * kPi / 2.0));
    const Arc outer = offsetArc(ccw, -0.25);
    assert(near(outer.radius, 1.25) && near(outer.length, 1.25 * kPi / 2.0));
    // The offset point is the offset curve at the same u.
    assert(nearV(arcOffsetPoint(ccw, 0.3, 0.25), arcPoint(inner, 0.3)));
    assert(nearV(arcOffsetPoint(ccw, 0.3, -0.25), arcPoint(outer, 0.3)));
    assert(nearV(arcPoint(inner, 0.0), {0, 0.25}));
    // A right turn is the other way round: left is the outside.
    const Arc cw = makeArc({0, 0}, {1, 0}, {1, -1});
    assert(near(offsetArc(cw, 0.25).radius, 1.25));
    assert(near(offsetArc(cw, -0.25).radius, 0.75));
    // As wide as the radius or wider: the inner edge collapses, no negative radius.
    assert(offsetArc(ccw, 1.0).collapsed);
    assert(offsetArc(ccw, 5.0).collapsed && near(offsetArc(ccw, 5.0).radius, 0.0));
    assert(!offsetArc(ccw, 0.99).collapsed);
    // Lines move sideways.
    const Arc line = makeArc({0, 0}, {1, 0}, {4, 0});
    const Arc moved = offsetArc(line, 2.0);
    assert(nearV(moved.p0, {0, 2}) && nearV(moved.p1, {4, 2}) && near(moved.length, 4.0));
    // The road width stays exact: every point of both edges is half a width from the centre line.
    const std::vector<Vec2> centre = arcPolyline(ccw, 512);
    assert(maxWidthError(arcPolyline(offsetArc(ccw, 0.25), 128), centre, 0.25) < 1e-3);
    assert(maxWidthError(arcPolyline(offsetArc(ccw, -0.25), 128), centre, 0.25) < 1e-3);
}

void arcDistances() {
    const Arc a = makeArc({0, 0}, {1, 0}, {1, 1});
    // Distance to position and back.
    assert(nearV(arcPointAtDistance(a, 0.0), {0, 0}));
    assert(nearV(arcPointAtDistance(a, a.length), {1, 1}));
    assert(nearV(arcPointAtDistance(a, a.length / 2.0), arcPoint(a, 0.5)));
    assert(nearV(arcPointAtDistance(a, -3.0), {0, 0}));  // clamped
    assert(nearV(arcPointAtDistance(a, 99.0), {1, 1}));
    // Position to distance: the point at 45 degrees round the circle is pi/4 along (radius 1).
    assert(near(arcDistanceOf(a, {std::sqrt(0.5), 1.0 - std::sqrt(0.5)}), kPi / 4.0));
    // Off the curve the nearest angle counts: radially outside gives the same distance.
    assert(near(arcDistanceOf(a, {2.0 * std::sqrt(0.5), 1.0 - 2.0 * std::sqrt(0.5)}), kPi / 4.0));
    // Past either end the nearest end wins.
    assert(near(arcDistanceOf(a, {1.0, 1.0 + 0.5}), a.length));
    assert(near(arcDistanceOf(a, {-0.5, 0.2}), 0.0));
    // The centre has no angle: the start.
    assert(near(arcDistanceOf(a, a.center), 0.0));
    // Equal steps in the angle are equal steps in distance on an arc (this is the whole point of the arc): a chord of
    // a circle with the same angle step is always the same length.
    double lo = 1e9;
    double hi = 0.0;
    for (int i = 0; i < 16; ++i) {
        const double c = length(arcPoint(a, (i + 1) / 16.0) - arcPoint(a, i / 16.0));
        lo = std::min(lo, c);
        hi = std::max(hi, c);
    }
    assert(hi - lo < 1e-12);
    // Lines.
    const Arc line = makeArc({0, 0}, {1, 0}, {10, 0});
    assert(near(arcDistanceOf(line, {4, 3}), 4.0));
    assert(near(arcDistanceOf(line, {-4, 3}), 0.0));
    assert(near(arcDistanceOf(line, {40, 3}), 10.0));
    assert(nearV(arcPointAtDistance(line, 2.5), {2.5, 0}));
}

void polylinesHaveTheRequestedCount() {
    assert(bezierPolyline(kParabola, 8).size() == 9);
    assert(bezierPolyline(kParabola, 0).size() == 2);  // at least one segment
    assert(offsetPolyline(kParabola, 8, 1.0).size() == 9);
    assert(arcPolyline(makeArc({0, 0}, {1, 0}, {1, 1}), 10).size() == 11);
    assert(arcPolyline(makeArc({0, 0}, {1, 0}, {-3, 0}), 10).empty());  // invalid arc: nothing to draw
    assert(arcPolyline(offsetArc(makeArc({0, 0}, {1, 0}, {1, 1}), 2.0), 10).empty());  // collapsed
}

}  // namespace

int main() {
    vectorBasics();
    bezierPointsAndTangents();
    controlPointOffsetOfACorner();
    controlPointOffsetEdgeCases();
    edgeWidthOfTheNaiveOffset();
    segmentAndPolylineDistance();
    widthErrorOfEdges();
    polylineLength();
    distanceToParameter();
    evenStepsInDistanceNotInT();
    speedRange();
    arcQuarterTurn();
    arcOtherSides();
    arcDegenerateCases();
    arcOffsets();
    arcDistances();
    polylinesHaveTheRequestedCount();
    std::puts("curved-paths logic: all tests passed");
    return EXIT_SUCCESS;
}
