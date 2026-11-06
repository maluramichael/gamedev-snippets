// Tests for the hex ring / spiral / rotation logic. Plain asserts on hand-computed cases, no framework, no SDL.
// Axial coordinates are (q, r), the third cube coordinate is s = -q - r.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdio>
#include <set>
#include <utility>
#include <vector>

#include "logic.hpp"

using namespace hrr;

namespace {

bool near(double a, double b) { return std::fabs(a - b) < 1e-9; }

std::set<std::pair<int, int>> asSet(const std::vector<Hex>& v) {
    std::set<std::pair<int, int>> out;
    for (const Hex& h : v) out.insert({h.q, h.r});
    return out;
}

void cubeAndDistance() {
    const Hex h{3, -1};
    assert(h.s() == -2);
    assert(h.q + h.r + h.s() == 0);
    assert(distance({0, 0}, {3, -1}) == 3);  // cube (3,-1,-2): largest absolute value is 3
    assert(distance({0, 0}, {1, 1}) == 2);   // cube (1,1,-2): not 1, the sign matters
    assert(distance({0, 0}, {1, -1}) == 1);  // a neighbor
    assert(distance({2, -3}, {-1, 1}) == 4);  // difference (3,-4,1)
    assert(distance({5, 5}, {5, 5}) == 0);
    assert(distance({-2, 4}, {1, 0}) == distance({1, 0}, {-2, 4}));
}

void directionsAreNeighbors() {
    for (int i = 0; i < 6; ++i) {
        assert(distance({0, 0}, kDirections[static_cast<std::size_t>(i)]) == 1);
        // the six directions are distinct and opposite ones cancel
        assert(kDirections[static_cast<std::size_t>(i)] + kDirections[static_cast<std::size_t>((i + 3) % 6)] == Hex({0, 0}));
    }
    assert(neighbor({2, 2}, 0) == Hex({3, 2}));
    assert(neighbor({2, 2}, 6) == neighbor({2, 2}, 0));   // wraps
    assert(neighbor({2, 2}, -1) == neighbor({2, 2}, 5));  // negative wraps too
}

void ringOfRadiusTwo() {
    // Start = center + dir4 * 2 = (-2, 2), then 2 steps each along dir 0..5.
    const std::vector<Hex> expected{{-2, 2}, {-1, 2}, {0, 2},  {1, 1},  {2, 0},  {2, -1},
                                    {2, -2}, {1, -2}, {0, -2}, {-1, -1}, {-2, 0}, {-2, 1}};
    const auto ring2 = ring({0, 0}, 2);
    assert(ring2.size() == 12);
    for (std::size_t i = 0; i < expected.size(); ++i) assert(ring2[i] == expected[i]);
}

void ringShape() {
    assert(ring({4, -3}, 0) == std::vector<Hex>({{4, -3}}));  // radius 0 is the center itself, not an empty ring
    assert(ring({0, 0}, -1).empty());
    for (int n = 1; n <= 8; ++n) {
        const auto cells = ring({3, -2}, n);
        assert(static_cast<int>(cells.size()) == 6 * n);
        assert(asSet(cells).size() == cells.size());  // no cell twice
        for (const Hex& h : cells) assert(distance({3, -2}, h) == n);
        // Consecutive cells are neighbors, and the walk closes back to the start.
        for (std::size_t i = 0; i < cells.size(); ++i) assert(distance(cells[i], cells[(i + 1) % cells.size()]) == 1);
        // The step from cell i to cell i+1 is the direction of its side.
        for (std::size_t i = 0; i < cells.size(); ++i) {
            const int side = ringSide(static_cast<int>(i), n);
            assert(cells[(i + 1) % cells.size()] == cells[i] + kDirections[static_cast<std::size_t>(side)]);
        }
    }
    assert(ringSide(0, 2) == 0 && ringSide(1, 2) == 0 && ringSide(2, 2) == 1 && ringSide(11, 2) == 5);
    assert(ringSide(5, 3) == 1 && ringSide(17, 3) == 5);
}

void spiralCounts() {
    // Hand-computed: 1, 7, 19, 37, 61, 91.
    const int expected[] = {1, 7, 19, 37, 61, 91};
    for (int n = 0; n <= 5; ++n) {
        assert(spiralCount(n) == expected[n]);
        assert(static_cast<int>(spiral({0, 0}, n).size()) == expected[n]);
    }
    assert(spiralCount(-1) == 0);
    assert(spiral({0, 0}, -1).empty());
    // The formula and the generator agree far beyond what the demo shows, and the cells are exactly the disc.
    for (int n = 0; n <= 12; ++n) {
        const auto cells = spiral({-1, 2}, n);
        assert(static_cast<int>(cells.size()) == spiralCount(n));
        assert(asSet(cells).size() == cells.size());
        for (const Hex& h : cells) assert(distance({-1, 2}, h) <= n);
        // Everything within distance n is there: count by brute force over a bounding box.
        int inside = 0;
        for (int q = -1 - n; q <= -1 + n; ++q)
            for (int r = 2 - n; r <= 2 + n; ++r)
                if (distance({-1, 2}, {q, r}) <= n) ++inside;
        assert(inside == spiralCount(n));
    }
    // Order: center first, then ring 1 cells, then ring 2 cells.
    const auto s2 = spiral({0, 0}, 2);
    assert(s2[0] == Hex({0, 0}));
    for (std::size_t i = 1; i <= 6; ++i) assert(distance({0, 0}, s2[i]) == 1);
    for (std::size_t i = 7; i <= 18; ++i) assert(distance({0, 0}, s2[i]) == 2);
    assert(s2[1] == Hex({-1, 1}));  // ring 1 starts at dir4
}

void rotationsOfTheDirections() {
    // One step to the left turns direction i into direction i + 1, six steps are a full turn.
    for (int i = 0; i < 6; ++i) {
        const Hex d = kDirections[static_cast<std::size_t>(i)];
        assert(rotateSteps(d, 1) == kDirections[static_cast<std::size_t>((i + 1) % 6)]);
        assert(rotateSteps(d, 2) == kDirections[static_cast<std::size_t>((i + 2) % 6)]);
        assert(rotateSteps(d, 3) == Hex({-d.q, -d.r}));
        assert(rotateSteps(d, -1) == kDirections[static_cast<std::size_t>((i + 5) % 6)]);
    }
}

void rotationHandComputed() {
    const Hex tip{3, -1};  // cube (3,-1,-2)
    assert(rotateSteps(tip, 0) == tip);
    assert(rotateSteps(tip, 1) == Hex({2, -3}));    // (-s,-q,-r) = (2,-3,1)
    assert(rotateSteps(tip, 2) == Hex({-1, -2}));   // (r,s,q)    = (-1,-2,3)
    assert(rotateSteps(tip, 3) == Hex({-3, 1}));    // (-q,-r,-s) = (-3,1,2)
    assert(rotateSteps(tip, 4) == Hex({-2, 3}));    // (s,q,r)    = (-2,3,-1)
    assert(rotateSteps(tip, 5) == Hex({1, 2}));     // (-r,-s,-q) = (1,2,-3)
    assert(rotateSteps(tip, 6) == tip);
    assert(rotateSteps(tip, -1) == rotateSteps(tip, 5));
    assert(rotateSteps(tip, -7) == rotateSteps(tip, 5));
    assert(rotateLeft(tip) == rotateSteps(tip, 1));
    assert(rotateRight(tip) == rotateSteps(tip, 5));
    assert(rotateRight(rotateLeft(tip)) == tip);
}

void rotationKeepsTheCubeInvariant() {
    for (int q = -6; q <= 6; ++q) {
        for (int r = -6; r <= 6; ++r) {
            const Hex h{q, r};
            for (int k = -7; k <= 7; ++k) {
                const Hex t = rotateSteps(h, k);
                assert(t.q + t.r + t.s() == 0);
                assert(distance({0, 0}, t) == distance({0, 0}, h));
            }
            assert(rotateSteps(rotateSteps(h, 2), 1) == rotateSteps(h, 3));
        }
    }
}

void rotationAroundAPivot() {
    const Hex pivot{2, 1};
    assert(rotateAround(pivot, pivot, 1) == pivot);  // the pivot stays
    // Neighbor of the pivot: (3,1) is dir 0 away, one step left makes it dir 1 away: (3,0).
    assert(rotateAround({3, 1}, pivot, 1) == Hex({3, 0}));
    // Tip (3,-1) relative to the pivot (2,1) is (1,-2), cube (1,-2,1). One step left: (-s,-q,-r) = (-1,-1,2), plus the pivot.
    assert(rotateAround({3, -1}, pivot, 1) == Hex({1, 0}));  // (-1,-1) + pivot
    // Distance to the pivot is kept for every step, a full turn is the identity.
    for (int k = 0; k < 6; ++k) assert(distance(rotateAround({3, -1}, pivot, k), pivot) == distance({3, -1}, pivot));
    assert(rotateAround({3, -1}, pivot, 6) == Hex({3, -1}));
    // The six rotations of one cell are exactly the six corners of the ring around the pivot.
    const auto corners = ring(pivot, 2);
    for (int k = 0; k < 6; ++k) {
        const Hex p = rotateAround({4, 1}, pivot, k);  // (4,1) is 2 away from the pivot in dir 0
        bool found = false;
        for (std::size_t i = 0; i < corners.size(); i += 2) found = found || corners[i] == p;
        assert(found);
    }
}

void mirrors() {
    const Hex tip{3, -1};  // cube (3,-1,-2)
    assert(mirror(tip, Axis::Q) == Hex({3, -2}));   // (q,s,r)
    assert(mirror(tip, Axis::R) == Hex({-2, -1}));  // (s,r,q)
    assert(mirror(tip, Axis::S) == Hex({-1, 3}));   // (r,q,s)
    for (const Axis a : {Axis::Q, Axis::R, Axis::S}) {
        assert(mirror(mirror(tip, a), a) == tip);  // twice is the identity
        for (int q = -4; q <= 4; ++q)
            for (int r = -4; r <= 4; ++r) {
                const Hex m = mirror({q, r}, a);
                assert(distance({0, 0}, m) == distance({0, 0}, {q, r}));
                assert(m.q + m.r + m.s() == 0);
            }
    }
    // Cells on the axis do not move: q stays when q is the axis name, mirror Q fixes r == s.
    assert(mirror({4, -2}, Axis::Q) == Hex({4, -2}));  // cube (4,-2,-2): r == s
    assert(mirror({-2, 4}, Axis::R) == Hex({-2, 4}));  // cube (-2,4,-2): q == s
    assert(mirror({-2, -2}, Axis::S) == Hex({-2, -2}));  // cube (-2,-2,4): q == r
    assert(mirrorAround({3, -1}, {2, 1}, Axis::Q) == Hex({3, 2}));  // relative (1,-2,1) -> (1,1,-2), plus the pivot (2,1)
    // A mirror is not a rotation: it flips the walking order of a neighbor ring.
    assert(mirror(kDirections[1], Axis::Q) == kDirections[0]);
}

void naiveRotationIsWrong() {
    // q and r treated as x and y, rotated by cos/sin and rounded. (2,0) turned by -60 degrees is (1,-1.732) -> (1,-2).
    assert(naiveRotate({2, 0}, 0) == Hex({2, 0}));
    assert(naiveRotate({2, 0}, 1) == Hex({1, -2}));
    assert(rotateSteps({2, 0}, 1) == Hex({2, -2}));  // the right answer is on the ring and a corner of it
    // The demo shape breaks: two neighbors end up two apart.
    const std::vector<Hex> shape = kShape;
    std::vector<Hex> bad;
    std::vector<Hex> good;
    for (const Hex& h : shape) {
        bad.push_back(naiveRotate(h, 1));
        good.push_back(rotateSteps(h, 1));
    }
    assert(bad[0] == Hex({1, 0}) && bad[1] == Hex({1, -2}) && bad[2] == Hex({2, -3}) && bad[3] == Hex({1, -3}));
    assert(distance(shape[0], shape[1]) == 1);
    assert(distance(bad[0], bad[1]) == 2);
    assert(sameShape(shape, shape));
    assert(sameShape(shape, good));
    assert(!sameShape(shape, bad));
}

void sameShapeDetectsChanges() {
    const std::vector<Hex> a{{0, 0}, {1, 0}, {1, -1}};
    std::vector<Hex> moved = a;
    for (Hex& h : moved) h = h + Hex({3, 2});
    assert(sameShape(a, moved));  // translation keeps all distances
    std::vector<Hex> longer = a;
    longer[2] = {2, -2};
    assert(!sameShape(a, longer));
    assert(!sameShape(a, std::vector<Hex>{{0, 0}, {1, 0}}));  // different length
    assert(sameShape({}, {}));
}

void pixelRoundTrip() {
    constexpr double size = 19.0;
    for (int q = -7; q <= 7; ++q) {
        for (int r = -7; r <= 7; ++r) {
            const Hex h{q, r};
            const Point p = toPixel(h, size);
            assert(fromPixel(p.x, p.y, size) == h);
            // A point well inside the hex (0.6 of the corner radius) still belongs to it, in every direction.
            for (int k = 0; k < 6; ++k) {
                const double a = 3.14159265358979 / 3.0 * k + 0.2;
                assert(fromPixel(p.x + 0.6 * size * std::cos(a), p.y + 0.6 * size * std::sin(a), size) == h);
            }
        }
    }
    // Hand-computed: with size 1 the neighbor (1,0) sits at (1.5, 0.866...), the border to (0,0) at (0.75, 0.433).
    const Point n = toPixel({1, 0}, 1.0);
    assert(near(n.x, 1.5) && near(n.y, std::sqrt(3.0) / 2.0));
    assert(fromPixel(0.7, 0.4, 1.0) == Hex({0, 0}));
    assert(fromPixel(0.8, 0.45, 1.0) == Hex({1, 0}));
    assert(near(toPixel({0, 1}, 2.0).y, 2.0 * std::sqrt(3.0)));
    assert(near(toPixel({0, 1}, 2.0).x, 0.0));
}

void rotationTableText() {
    // The text shown on screen comes from the same table the code uses.
    assert(rotationFormula(0) == "(q, r, s)");
    assert(rotationFormula(1) == "(-s, -q, -r)");
    assert(rotationFormula(2) == "(r, s, q)");
    assert(rotationFormula(3) == "(-q, -r, -s)");
    assert(rotationFormula(4) == "(s, q, r)");
    assert(rotationFormula(5) == "(-r, -s, -q)");
    assert(rotationFormula(-1) == rotationFormula(5));
    assert(rotationFormula(7) == rotationFormula(1));
    assert(mirrorFormula(Axis::Q) == "(q, s, r)");
    assert(mirrorFormula(Axis::R) == "(s, r, q)");
    assert(mirrorFormula(Axis::S) == "(r, q, s)");
}

}  // namespace

int main() {
    cubeAndDistance();
    directionsAreNeighbors();
    ringOfRadiusTwo();
    ringShape();
    spiralCounts();
    rotationsOfTheDirections();
    rotationHandComputed();
    rotationKeepsTheCubeInvariant();
    rotationAroundAPivot();
    mirrors();
    naiveRotationIsWrong();
    sameShapeDetectsChanges();
    pixelRoundTrip();
    rotationTableText();
    std::puts("hex-ring-rotation logic: all asserts passed");
    return 0;
}
