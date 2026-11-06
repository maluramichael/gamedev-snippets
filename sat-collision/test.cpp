// Tests for the SAT logic. Plain asserts on hand-computed cases, no framework, no SDL.
// Coordinates are "math" form (y up), the logic does not care which way y points.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "logic.hpp"

using namespace sat;

namespace {

bool near(double a, double b, double eps = 1e-9) { return std::fabs(a - b) < eps; }
bool nearVec(Vec2 a, Vec2 b, double eps = 1e-9) { return near(a.x, b.x, eps) && near(a.y, b.y, eps); }

/// Axis-aligned box as a polygon, counter-clockwise.
Polygon box(double x0, double y0, double x1, double y1) { return {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}}; }

Polygon reversed(Polygon p) {
    Polygon out(p.rbegin(), p.rend());
    return out;
}

Polygon rotated(const Polygon& p, double angle) {
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    Polygon out;
    for (const Vec2& v : p) out.push_back({v.x * c - v.y * s, v.x * s + v.y * c});
    return out;
}

const Options kRight{true, true};
const Options kRaw{false, true};       // axes keep the edge length
const Options kUnoriented{true, false};  // MTV keeps the direction of the axis

void projectionOfASquare() {
    const Polygon sq = box(0, 0, 4, 4);
    const Interval x = project(sq, {1, 0});
    assert(near(x.min, 0) && near(x.max, 4));
    // The diagonal (1,1)/sqrt2: corners (0,0) and (4,4) are the extremes, 4*sqrt2 apart.
    const double r = std::sqrt(0.5);
    const Interval d = project(sq, {r, r});
    assert(near(d.min, 0) && near(d.max, 4 * std::sqrt(2.0)));
    assert(d.iMin == 0 && d.iMax == 2);
    // A longer axis scales the projection (this is what an unnormalized axis does).
    const Interval twice = project(sq, {2, 0});
    assert(near(twice.min, 0) && near(twice.max, 8));
}

void overlappingSquares() {
    // B reaches 1 into A on x and 3 on y. Pushing B +x by 1 separates them.
    const Result r = collide(box(0, 0, 4, 4), box(3, 1, 7, 5), kRight);
    assert(r.colliding);
    assert(r.axes.size() == 2);  // opposite edges are parallel, one axis per direction
    assert(near(r.axes[r.best].amount, 1.0));
    assert(nearVec(r.mtv, {1, 0}));
    // Every axis reports overlap (amount > 0), otherwise it would have proven a gap.
    for (const AxisResult& a : r.axes) assert(a.amount > 0.0);
}

void mtvPointsFromAToB() {
    // B sits to the left of A and reaches 2 into it: B has to move LEFT.
    const Polygon a = box(0, 0, 4, 4);
    const Polygon b = box(-2, 1, 2, 5);
    const Result right = collide(a, b, kRight);
    assert(right.colliding && nearVec(right.mtv, {-2, 0}));
    // Without the side choice the MTV keeps the direction of A's right edge normal: it pushes B INTO A.
    const Result wrong = collide(a, b, kUnoriented);
    assert(wrong.colliding && nearVec(wrong.mtv, {2, 0}));
    assert(penetration(a, translated(b, right.mtv)) == 0.0);
    assert(penetration(a, translated(b, wrong.mtv)) > 2.9);  // B at x 0..4 now, still 3 deep on y
}

void mtvIsMirroredWhenTheShapesSwap() {
    const Polygon a = box(0, 0, 4, 4);
    const Polygon b = box(3, 1, 7, 5);
    assert(nearVec(collide(a, b, kRight).mtv, {1, 0}));
    assert(nearVec(collide(b, a, kRight).mtv, {-1, 0}));
}

void separatedBoxes() {
    const Result r = collide(box(0, 0, 4, 4), box(5, 0, 9, 4), kRight);
    assert(!r.colliding);
    assert(nearVec(r.mtv, {0, 0}));
    // The x axis (second one: A's bottom edge gives y first) is the separating one, the gap is 1.
    assert(r.best == 1);
    assert(near(r.axes[r.best].amount, -1.0));
    assert(near(r.axes[0].amount, 4.0));  // y axis: both span 0..4, push 4 either way
    assert(!collide(box(0, 0, 4, 4), box(5, 0, 9, 4), kRaw).colliding);
    assert(!collide(box(0, 0, 4, 4), box(5, 0, 9, 4), kUnoriented).colliding);
}

void touchingIsNotColliding() {
    const Result r = collide(box(0, 0, 4, 4), box(4, 0, 8, 4), kRight);
    assert(!r.colliding);
    assert(near(r.axes[r.best].amount, 0.0));
    // Pushed by a hair they do collide.
    assert(collide(box(0, 0, 4, 4), box(3.999, 0, 7.999, 4), kRight).colliding);
}

void containment() {
    // B lies inside A. The intervals overlap by 2 (B's width) but B has to travel 6 to leave: 4 to the edge of A
    // plus its own width of 2. Overlap alone is not the push distance.
    const Polygon a = box(0, 0, 10, 10);
    const Polygon b = box(4, 4, 6, 6);
    const Result r = collide(a, b, kRight);
    assert(r.colliding);
    assert(near(r.axes[r.best].amount, 6.0));
    assert(near(length(r.mtv), 6.0));
    assert(penetration(a, translated(b, r.mtv)) == 0.0);
    // The plain interval overlap (2) pushes B only to the middle of A's edge: still inside.
    const Result naive = collide(a, b, kUnoriented);
    assert(near(length(naive.mtv), 2.0));
    assert(penetration(a, translated(b, naive.mtv)) > 0.0);
}

void rotatedDiamond() {
    // B is a diamond with its tip (3.5,2) reaching 0.5 into A's right edge x = 4.
    const Polygon a = box(0, 0, 4, 4);
    const Polygon b{{7.5, 2}, {5.5, 4}, {3.5, 2}, {5.5, 0}};
    const Result r = collide(a, b, kRight);
    assert(r.colliding);
    assert(r.axes.size() == 4);  // A: y and x, B: its two diagonals
    assert(r.best == 1);         // A's right edge, the x axis
    assert(nearVec(r.mtv, {0.5, 0}));
    // On the diagonal (1,1)/sqrt2 A spans 0..5.657 and B 3.889..6.718, so B has to move 1.768 = 1.25 * sqrt(2).
    // The other diagonal gives the same number from the other side (B has to move along -axis there).
    assert(near(r.axes[2].amount, 1.25 * std::sqrt(2.0)));
    assert(near(r.axes[3].amount, 1.25 * std::sqrt(2.0)));
    assert(r.axes[2].owner == 1 && r.axes[3].owner == 1);
    assert(penetration(a, translated(b, r.mtv)) == 0.0);
}

void unnormalizedAxes() {
    // A is 8 x 2, B is 4 x 2 and reaches 2 into A on x and 1 on y. The right push is 1 along y.
    const Polygon a = box(0, 0, 8, 2);
    const Polygon b = box(6, 1, 10, 3);
    const Result right = collide(a, b, kRight);
    assert(right.colliding && nearVec(right.mtv, {0, 1}));

    // Axes taken as perp(edge) have the length of the edge: 8 for the bottom edge, 2 for the right edge.
    const Result raw = collide(a, b, kRaw);
    assert(raw.colliding);  // the yes/no answer survives, only the numbers are scaled
    assert(raw.axes.size() == 2);
    assert(near(length(raw.axes[0].axis), 8.0) && near(length(raw.axes[1].axis), 2.0));
    assert(near(raw.axes[0].amount, 8.0));  // 1 * 8
    assert(near(raw.axes[1].amount, 4.0));  // 2 * 2
    // The x axis now looks like the smaller overlap, so it is chosen, and amount * axis = 4 * (2,0) = (8,0).
    assert(raw.best == 1);
    assert(nearVec(raw.mtv, {8, 0}));
    assert(near(length(raw.mtv), 8.0));  // 8 units instead of 1 along y
    assert(penetration(a, translated(b, raw.mtv)) == 0.0);  // it does resolve, 8 times too far and the wrong way
}

void rotationDoesNotChangeTheDepth() {
    const Polygon a = box(0, 0, 4, 4);
    const Polygon b{{7.5, 2}, {5.5, 4}, {3.5, 2}, {5.5, 0}};
    const Result base = collide(a, b, kRight);
    for (const double angle : {0.7, 2.0, -1.3}) {
        const Result r = collide(rotated(a, angle), rotated(b, angle), kRight);
        assert(r.colliding);
        assert(near(length(r.mtv), length(base.mtv)));
        // The MTV turns with the scene.
        assert(nearVec(r.mtv, rotated({base.mtv}, angle)[0]));
    }
}

void windingDoesNotMatter() {
    const Polygon a = box(0, 0, 4, 4);
    const Polygon b = box(-2, 1, 2, 5);
    // Clockwise polygons have the same outward normals.
    assert(nearVec(collide(reversed(a), b, kRight).mtv, collide(a, b, kRight).mtv));
    assert(nearVec(collide(a, reversed(b), kRight).mtv, collide(a, b, kRight).mtv));
    assert(near(signedArea(a), 16.0));
    assert(near(signedArea(reversed(a)), -16.0));
}

void degenerateInput() {
    const Polygon a = box(0, 0, 4, 4);
    // Fewer than three corners is not a shape.
    assert(!collide(a, Polygon{{1, 1}, {2, 2}}, kRight).colliding);
    assert(collide(Polygon{}, a, kRight).axes.empty());
    // A repeated corner makes a zero length edge: no axis, no NaN, same answer as without.
    const Polygon withTwin{{0, 0}, {0, 0}, {4, 0}, {4, 4}, {0, 4}};
    const Result r = collide(withTwin, box(3, 1, 7, 5), kRight);
    assert(r.colliding && r.axes.size() == 2);
    assert(nearVec(r.mtv, {1, 0}));
    assert(!std::isnan(r.mtv.x) && !std::isnan(r.mtv.y));
}

void pointInConvex() {
    const Polygon sq = box(0, 0, 4, 4);
    assert(contains(sq, {2, 2}));
    assert(contains(reversed(sq), {2, 2}));
    assert(!contains(sq, {5, 2}));
    assert(!contains(sq, {2, -0.1}));
    assert(contains(sq, {4, 2}));  // the border counts
    const Polygon tri{{0, 0}, {4, 0}, {0, 4}};
    assert(contains(tri, {1, 1}));
    assert(!contains(tri, {3, 3}));  // inside the bounding box, outside the hypotenuse
}

void shapeTransform() {
    // A unit square around the origin, turned by 90 degrees and moved to (10, 20).
    const Shape s{{{-1, -1}, {1, -1}, {1, 1}, {-1, 1}}, {10, 20}, 1.5707963267948966};
    const Polygon w = worldVertices(s);
    assert(w.size() == 4);
    assert(nearVec(w[0], {11, 19}, 1e-9));  // (-1,-1) turned by 90 degrees is (1,-1)
    assert(nearVec(center(w), {10, 20}, 1e-9));
}

void presetsOpenOnSomethingInteresting() {
    for (int i = 0; i < kPresetCount; ++i) {
        const Scene sc = preset(i);
        const Polygon a = worldVertices(sc.a);
        const Polygon b = worldVertices(sc.b);
        assert(a.size() >= 3 && b.size() >= 3);
        assert(collide(a, b, kRight).colliding);
        // The yes/no answer is the same for every setting, only the push differs.
        assert(collide(a, b, kRaw).colliding && collide(a, b, kUnoriented).colliding);
    }
    // The first preset shows both mistakes at once: the right push comes from an axis of B, the raw axes choose another
    // axis, and the unoriented MTV points the wrong way.
    const Scene sc = preset(0);
    const Polygon a = worldVertices(sc.a);
    const Polygon b = worldVertices(sc.b);
    const Result right = collide(a, b, kRight);
    assert(right.axes[right.best].owner == 1);
    assert(length(right.mtv) > 0.3 && length(right.mtv) < 1.5);
    const Result raw = collide(a, b, kRaw);
    assert(raw.axes[raw.best].owner != right.axes[right.best].owner || raw.axes[raw.best].edge != right.axes[right.best].edge);
    assert(dot(collide(a, b, kUnoriented).mtv, right.mtv) < 0.0);
    assert(penetration(a, translated(b, right.mtv)) == 0.0);
    assert(penetration(a, translated(b, collide(a, b, kUnoriented).mtv)) > 0.0);
}

}  // namespace

int main() {
    projectionOfASquare();
    overlappingSquares();
    mtvPointsFromAToB();
    mtvIsMirroredWhenTheShapesSwap();
    separatedBoxes();
    touchingIsNotColliding();
    containment();
    rotatedDiamond();
    unnormalizedAxes();
    rotationDoesNotChangeTheDepth();
    windingDoesNotMatter();
    degenerateInput();
    pointInConvex();
    shapeTransform();
    presetsOpenOnSomethingInteresting();
    std::puts("sat-collision logic: all asserts passed");
    return 0;
}
