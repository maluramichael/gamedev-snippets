// Tests for the CPU mirror of the Voronoi shader math in logic.hpp. Hand-computed cases first, then the properties the demo relies on.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "logic.hpp"

namespace {

int failures = 0;

#define CHECK(cond)                                                                    \
    do {                                                                               \
        if (!(cond)) {                                                                 \
            std::printf("FAILED %s:%d: %s\n", __FILE__, __LINE__, #cond);              \
            ++failures;                                                                \
        }                                                                              \
    } while (0)

bool near(double a, double b, double eps = 1e-5) { return std::fabs(a - b) <= eps; }

using namespace gd::voronoi;

// Metrics on the 3-4-5 triangle: the vector (3, 4) has length 5, Manhattan 7, Chebyshev 4.
void metricsOnA345Triangle() {
    const Vec2 a{0.0F, 0.0F}, b{3.0F, 4.0F};
    CHECK(near(distance(Metric::Euclid, a, b), 5.0));
    CHECK(near(distance(Metric::Manhattan, a, b), 7.0));
    CHECK(near(distance(Metric::Chebyshev, a, b), 4.0));
    CHECK(near(distance(Metric::Manhattan, b, a), 7.0));  // symmetric
    CHECK(near(distance(Metric::Euclid, a, a), 0.0));
}

// Sites at (0,0), (10,0), (0,10). Points are checked by hand: the nearest one is obvious.
void nearestOnHandPickedPoints() {
    const std::vector<Vec2> s{{0.0F, 0.0F}, {10.0F, 0.0F}, {0.0F, 10.0F}};
    const Nearest a = nearest(s, {1.0F, 1.0F}, Metric::Euclid);
    CHECK(a.first == 0);
    CHECK(near(a.d1, std::sqrt(2.0)));
    CHECK(a.second >= 1);
    CHECK(near(a.d2, std::sqrt(81.0 + 1.0)));  // (10,0) and (0,10) are both sqrt(82) away: the smaller index is the second
    CHECK(a.second == 1);
    CHECK(nearest(s, {9.0F, 1.0F}, Metric::Euclid).first == 1);
    CHECK(nearest(s, {1.0F, 9.0F}, Metric::Euclid).first == 2);
    // The bisector of sites 1 and 2 is the line y = x (through the origin at 45 degrees).
    CHECK(nearest(s, {8.0F, 8.5F}, Metric::Euclid).first == 2);
    CHECK(nearest(s, {8.5F, 8.0F}, Metric::Euclid).first == 1);
}

// A point exactly between two sites: same distance to both, and the smaller index wins (strict <, like in the shader).
void tieGoesToTheSmallerIndex() {
    const std::vector<Vec2> s{{0.0F, 0.0F}, {4.0F, 0.0F}};
    const Nearest n = nearest(s, {2.0F, 0.0F}, Metric::Euclid);
    CHECK(near(n.d1, 2.0));
    CHECK(near(n.d2, 2.0));
    CHECK(n.first == 0);
    CHECK(n.second == 1);
    // Same with the order of the sites swapped: now index 0 is the right-hand site.
    const std::vector<Vec2> t{{4.0F, 0.0F}, {0.0F, 0.0F}};
    CHECK(nearest(t, {2.0F, 0.0F}, Metric::Euclid).first == 0);
    // Every point on the perpendicular bisector has equal distance to both (here x = 2, any y).
    for (const float y : {-7.0F, 0.0F, 3.5F, 100.0F}) {
        const Nearest m = nearest(s, {2.0F, y}, Metric::Euclid);
        CHECK(near(m.d1, m.d2, 1e-4));
    }
}

// With two sites nothing is left for "second": it stays at -1 and d2 is infinite. With one site likewise.
void degenerateSiteCounts() {
    const Nearest none = nearest({}, {1.0F, 1.0F}, Metric::Euclid);
    CHECK(none.first == -1);
    const Nearest one = nearest(std::vector<Vec2>{{5.0F, 5.0F}}, {1.0F, 1.0F}, Metric::Euclid);
    CHECK(one.first == 0);
    CHECK(one.second == -1);
    CHECK(std::isinf(one.d2));
}

// The metric changes the cells. Sites A = (0,0) and B = (4,3), the point P = (4.4, -1.9):
//   Euclid:    A = sqrt(19.36 + 3.61) = 4.793, B = sqrt(0.16 + 24.01) = 4.916 -> A
//   Manhattan: A = 6.3,                         B = 0.4 + 4.9 = 5.3          -> B
//   Chebyshev: A = 4.4,                         B = 4.9                      -> A
void metricsDisagreeSomewhere() {
    const std::vector<Vec2> s{{0.0F, 0.0F}, {4.0F, 3.0F}};
    // P = (3, -1) is clearly in A's cell for every metric: Euclid 3.162 against 4.123, Manhattan 4 against 5.
    CHECK(nearest(s, {3.0F, -1.0F}, Metric::Euclid).first == 0);
    CHECK(nearest(s, {3.0F, -1.0F}, Metric::Manhattan).first == 0);
    const Vec2 p{4.4F, -1.9F};
    CHECK(nearest(s, p, Metric::Euclid).first == 0);
    CHECK(nearest(s, p, Metric::Manhattan).first == 1);
    CHECK(nearest(s, p, Metric::Chebyshev).first == 0);
    CHECK(near(nearest(s, p, Metric::Manhattan).d1, 5.3));
}

// Exact distance to the nearest cell border. Two sites (0,0) and (4,0): the border is x = 2.
void edgeDistanceIsTheDistanceToTheBisector() {
    const std::vector<Vec2> s{{0.0F, 0.0F}, {4.0F, 0.0F}};
    CHECK(near(edgeDistance(s, 0, {1.0F, 0.0F}), 1.0));
    CHECK(near(edgeDistance(s, 0, {1.0F, 50.0F}), 1.0));  // along the border the distance does not change
    CHECK(near(edgeDistance(s, 0, {2.0F, 7.0F}), 0.0));   // on the bisector
    CHECK(near(edgeDistance(s, 1, {3.0F, 0.0F}), 1.0));   // seen from the other cell
    // Three sites: the nearest border wins. A = (0,0), B = (4,0), C = (0,6): borders x = 2 (to B) and y = 3 (to C).
    const std::vector<Vec2> t{{0.0F, 0.0F}, {4.0F, 0.0F}, {0.0F, 6.0F}};
    CHECK(near(edgeDistance(t, 0, {1.0F, 1.0F}), 1.0));
    CHECK(near(edgeDistance(t, 0, {1.5F, 2.0F}), 0.5));
    CHECK(near(edgeDistance(t, 0, {0.5F, 2.75F}), 0.25));
}

// The cheap version (d2 - d1) / 2 is only a lower bound: right on the axis through both sites it is exact, off the axis it is too small.
void halfDifferenceIsOnlyALowerBound() {
    const std::vector<Vec2> s{{0.0F, 0.0F}, {4.0F, 0.0F}};
    const Nearest on = nearest(s, {1.0F, 0.0F}, Metric::Euclid);
    CHECK(near((on.d2 - on.d1) / 2.0, edgeDistance(s, 0, {1.0F, 0.0F})));  // (3 - 1) / 2 = 1
    const Nearest off = nearest(s, {1.0F, 3.0F}, Metric::Euclid);
    CHECK(near(off.d1, std::sqrt(10.0)));
    CHECK(near(off.d2, std::sqrt(18.0)));
    CHECK(near((off.d2 - off.d1) / 2.0, (std::sqrt(18.0) - std::sqrt(10.0)) / 2.0));
    CHECK((off.d2 - off.d1) / 2.0 < edgeDistance(s, 0, {1.0F, 3.0F}) - 0.4);  // about 0.54 against 1.0
}

// Drifting sites bounce off the walls of the rectangle [0, w] x [0, h].
void driftReflectsAtTheWalls() {
    Body b{{9.0F, 5.0F}, {4.0F, 0.0F}};
    drift(b, 1.0F, 10.0F, 10.0F);  // would be at 13, the wall is at 10: ends at 7 and flies back
    CHECK(near(b.p.x, 7.0));
    CHECK(near(b.v.x, -4.0));
    CHECK(near(b.p.y, 5.0));
    Body c{{1.0F, 1.0F}, {-3.0F, -2.0F}};
    drift(c, 1.0F, 10.0F, 10.0F);  // x: -2 -> 2, y: -1 -> 1
    CHECK(near(c.p.x, 2.0));
    CHECK(near(c.p.y, 1.0));
    CHECK(near(c.v.x, 3.0));
    CHECK(near(c.v.y, 2.0));
    Body d{{5.0F, 5.0F}, {1.0F, 1.0F}};
    drift(d, 2.0F, 10.0F, 10.0F);  // free flight
    CHECK(near(d.p.x, 7.0));
    CHECK(near(d.v.x, 1.0));
    // A huge step still ends up inside.
    Body e{{5.0F, 5.0F}, {1000.0F, -777.0F}};
    drift(e, 1.0F, 10.0F, 10.0F);
    CHECK(e.p.x >= 0.0F && e.p.x <= 10.0F && e.p.y >= 0.0F && e.p.y <= 10.0F);
}

// Lloyd relaxation on a grid where every cell is one unit; the domain is [0, w] x [0, h] and cell centers sit at i + 0.5.
void lloydSingleSiteStaysInTheMiddle() {
    const std::vector<Vec2> s{{4.5F, 4.5F}};
    const std::vector<Vec2> r = lloydStep(s, 9, 9);
    CHECK(r.size() == 1);
    CHECK(near(r[0].x, 4.5));
    CHECK(near(r[0].y, 4.5));
    // A single site anywhere moves to the middle of the whole domain in one step.
    const std::vector<Vec2> t = lloydStep(std::vector<Vec2>{{1.0F, 8.0F}}, 10, 6);
    CHECK(near(t[0].x, 5.0));
    CHECK(near(t[0].y, 3.0));
}

// Domain 10 x 10, sites at (4,5) and (6,5). The bisector is x = 5, the left cell has centers 0.5 .. 4.5 (mean 2.5),
// the right one 5.5 .. 9.5 (mean 7.5), rows 0.5 .. 9.5 (mean 5). So the sites move apart to (2.5,5) and (7.5,5).
void lloydTwoSitesMoveApart() {
    const std::vector<Vec2> s{{4.0F, 5.0F}, {6.0F, 5.0F}};
    const std::vector<Vec2> r = lloydStep(s, 10, 10);
    CHECK(near(r[0].x, 2.5));
    CHECK(near(r[0].y, 5.0));
    CHECK(near(r[1].x, 7.5));
    CHECK(near(r[1].y, 5.0));
    CHECK(near(r[1].x - r[0].x, 5.0));
    CHECK(r[1].x - r[0].x > s[1].x - s[0].x);
    // That is a fixed point: the second step changes nothing.
    const std::vector<Vec2> r2 = lloydStep(r, 10, 10);
    CHECK(near(r2[0].x, 2.5) && near(r2[1].x, 7.5));
    // The distance of each site to its cell centroid: 1.5 + 1.5 before, 0 after.
    CHECK(near(centroidError(s, 10, 10), 3.0));
    CHECK(near(centroidError(r, 10, 10), 0.0));
}

// Random-ish sites: the sum of the centroid deviations goes down over a few steps.
void lloydReducesTheCentroidError() {
    std::vector<Vec2> s;
    std::uint32_t seed = 7U;
    for (int i = 0; i < 12; ++i) {
        const float x = static_cast<float>(unit(pcg(seed++))) * 60.0F;
        const float y = static_cast<float>(unit(pcg(seed++))) * 34.0F;
        s.push_back({x, y});
    }
    const double before = centroidError(s, 60, 34);
    std::vector<Vec2> cur = s;
    for (int i = 0; i < 5; ++i) cur = lloydStep(cur, 60, 34);
    const double after = centroidError(cur, 60, 34);
    CHECK(after < before * 0.5);
    for (const Vec2& p : cur) CHECK(p.x >= 0.0F && p.x <= 60.0F && p.y >= 0.0F && p.y <= 34.0F);
}

// A site that owns no grid cell has no centroid and stays where it is.
void lloydSiteWithoutCellStays() {
    const std::vector<Vec2> s{{1.0F, 1.0F}, {1.0F, 1.0F}};  // identical: the second never wins a tie
    const std::vector<Vec2> r = lloydStep(s, 4, 4);
    CHECK(near(r[0].x, 2.0) && near(r[0].y, 2.0));
    CHECK(near(r[1].x, 1.0) && near(r[1].y, 1.0));
}

// Hash: values fixed with an independent implementation (Python, same constants), so the GPU and the CPU can agree.
void hashGoldenValues() {
    CHECK(pcg(0U) == 129708002U);
    CHECK(pcg(1U) == 2831084092U);
    CHECK(hashCell(0, 0, 0U) == 2145236065U);
    CHECK(hashCell(-1, 2, 7U) == 1383149818U);
    CHECK(hashCell(3, 5, 0U) == 3892713224U);
    CHECK(near(unit(hashCell(0, 0, 0U)), 8379828.0 / 16777216.0, 1e-9));
    CHECK(hashCell(1, 2, 3U) == hashCell(1, 2, 3U));  // deterministic
    CHECK(hashCell(1, 2, 3U) != hashCell(2, 1, 3U));  // x and y are not interchangeable
    CHECK(hashCell(1, 2, 3U) != hashCell(1, 2, 4U));  // the seed matters
}

void unitStaysInHalfOpenInterval() {
    CHECK(unit(0U) == 0.0);
    CHECK(unit(0xFFFFFFFFU) < 1.0);
    for (std::uint32_t i = 0; i < 2000; ++i) {
        const double u = unit(pcg(i));
        CHECK(u >= 0.0 && u < 1.0);
    }
}

// With jitter 0 every feature point sits on its cell corner, so the cell noise is a plain grid and everything is hand-computable.
void worleyOnARegularGrid() {
    const Worley w0 = worley({0.0F, 0.0F}, 0.0F, Metric::Euclid, 0.0F);
    CHECK(near(w0.f1, 0.0));
    CHECK(near(w0.f2, 1.0));
    CHECK(near(w0.f2 - w0.f1, 1.0));
    const Worley w1 = worley({3.0F, -2.0F}, 0.0F, Metric::Euclid, 0.0F);  // any lattice point, negative coordinates too
    CHECK(near(w1.f1, 0.0));
    CHECK(near(w1.f2, 1.0));
    const Worley w2 = worley({0.25F, 0.0F}, 0.0F, Metric::Euclid, 0.0F);
    CHECK(near(w2.f1, 0.25));
    CHECK(near(w2.f2, 0.75));
    CHECK(near(w2.f2 - w2.f1, 0.5));
    const Worley w3 = worley({0.5F, 0.0F}, 0.0F, Metric::Euclid, 0.0F);  // halfway between two lattice points: a cell border
    CHECK(near(w3.f2 - w3.f1, 0.0));
    const Worley w4 = worley({0.5F, 0.5F}, 0.0F, Metric::Euclid, 0.0F);  // the center of a square: four equal corners
    CHECK(near(w4.f1, std::sqrt(0.5)));
    CHECK(near(w4.f2, std::sqrt(0.5)));
    const Worley m = worley({0.5F, 0.5F}, 0.0F, Metric::Manhattan, 0.0F);
    CHECK(near(m.f1, 1.0));
    const Worley c = worley({0.5F, 0.5F}, 0.0F, Metric::Chebyshev, 0.0F);
    CHECK(near(c.f1, 0.5));
    // The id of the nearest feature point is the lattice cell (here 0,0 for p inside [0,0.5)).
    CHECK(w2.cellX == 0 && w2.cellY == 0);
    const Worley w5 = worley({0.75F, 0.25F}, 0.0F, Metric::Euclid, 0.0F);
    CHECK(w5.cellX == 1 && w5.cellY == 0);
}

// With real jitter: F1 never exceeds the diagonal of one cell, F2 >= F1, and the result is deterministic.
void worleyBounds() {
    for (int i = 0; i < 4000; ++i) {
        const float x = (static_cast<float>(unit(pcg(static_cast<std::uint32_t>(i) * 2U))) - 0.5F) * 40.0F;
        const float y = (static_cast<float>(unit(pcg(static_cast<std::uint32_t>(i) * 2U + 1U))) - 0.5F) * 40.0F;
        const float t = static_cast<float>(i % 17) * 0.7F;
        const Worley a = worley({x, y}, t, Metric::Euclid);
        CHECK(a.f1 >= 0.0F && a.f1 <= std::sqrt(2.0F) + 1e-6F);
        CHECK(a.f2 >= a.f1);
        const Worley again = worley({x, y}, t, Metric::Euclid);
        CHECK(a.f1 == again.f1 && a.f2 == again.f2 && a.cellX == again.cellX && a.cellY == again.cellY);
        CHECK(worley({x, y}, t, Metric::Chebyshev).f1 <= 1.0F + 1e-6F);
        CHECK(worley({x, y}, t, Metric::Manhattan).f1 <= 2.0F + 1e-6F);
    }
}

// Why 3x3 cells are enough: compare with a 5x5 search over many points. The 3x3 result is wrong only very rarely.
void worleyThreeByThreeIsAlmostAlwaysExact() {
    int wrong = 0;
    const int n = 20000;
    for (int i = 0; i < n; ++i) {
        const float x = (static_cast<float>(unit(pcg(static_cast<std::uint32_t>(i) * 2U + 100000U))) - 0.5F) * 60.0F;
        const float y = (static_cast<float>(unit(pcg(static_cast<std::uint32_t>(i) * 2U + 100001U))) - 0.5F) * 60.0F;
        const Worley fast = worley({x, y}, 0.0F, Metric::Euclid, 1.0F, 1);
        const Worley slow = worley({x, y}, 0.0F, Metric::Euclid, 1.0F, 2);
        if (std::fabs(fast.f1 - slow.f1) > 1e-6F) ++wrong;
    }
    std::printf("3x3 against 5x5: F1 differs in %d of %d points\n", wrong, n);
    CHECK(wrong * 100 < n);  // less than one percent
}

// For Manhattan and Chebyshev the "bisector" can be an area: sites on an exact diagonal have the same Manhattan distance
// to every point beyond them. That is why those metrics show dark patches when the borders are drawn from (d2 - d1).
void manhattanTiesCanFillAnArea() {
    const std::vector<Vec2> s{{0.0F, 0.0F}, {4.0F, 4.0F}};
    for (const Vec2 p : {Vec2{-1.0F, 5.0F}, Vec2{-3.0F, 6.5F}, Vec2{0.0F, 20.0F}, Vec2{-10.0F, 4.0F}}) {
        const Nearest n = nearest(s, p, Metric::Manhattan);
        CHECK(near(n.d1, n.d2));
    }
    const Nearest e = nearest(s, {-10.0F, 4.0F}, Metric::Euclid);  // with the Euclidean metric the same point is clearly in one cell
    CHECK(e.d2 - e.d1 > 3.0F);
}

// Worley edge distance on the regular lattice (jitter 0): feature points on the integer corners, A = (0,0).
void worleyEdgeOnARegularGrid() {
    CHECK(near(worleyEdge({0.25F, 0.0F}, 0.0F, 0.0F), 0.25));  // border x = 0.5
    CHECK(near(worleyEdge({0.3F, 0.1F}, 0.0F, 0.0F), 0.2));    // x = 0.5 is 0.2 away, y = 0.5 is 0.4 away, the diagonal one 0.424
    CHECK(near(worleyEdge({0.5F, 0.2F}, 0.0F, 0.0F), 0.0));    // on the border between (0,0) and (1,0), tie goes to the first
}

void worleyEdgeIsAtLeastHalfTheDifference() {
    for (int i = 0; i < 2000; ++i) {
        const float x = (static_cast<float>(unit(pcg(static_cast<std::uint32_t>(i) * 2U + 500000U))) - 0.5F) * 30.0F;
        const float y = (static_cast<float>(unit(pcg(static_cast<std::uint32_t>(i) * 2U + 500001U))) - 0.5F) * 30.0F;
        const Worley w = worley({x, y}, 1.0F, Metric::Euclid);
        const float exact = worleyEdge({x, y}, 1.0F);
        CHECK(exact >= (w.f2 - w.f1) / 2.0F - 1e-4F);
        CHECK(exact >= -1e-4F);
    }
}

}  // namespace

int main() {
    metricsOnA345Triangle();
    nearestOnHandPickedPoints();
    tieGoesToTheSmallerIndex();
    degenerateSiteCounts();
    metricsDisagreeSomewhere();
    edgeDistanceIsTheDistanceToTheBisector();
    halfDifferenceIsOnlyALowerBound();
    driftReflectsAtTheWalls();
    lloydSingleSiteStaysInTheMiddle();
    lloydTwoSitesMoveApart();
    lloydReducesTheCentroidError();
    lloydSiteWithoutCellStays();
    hashGoldenValues();
    unitStaysInHalfOpenInterval();
    worleyOnARegularGrid();
    worleyBounds();
    worleyThreeByThreeIsAlmostAlwaysExact();
    manhattanTiesCanFillAnArea();
    worleyEdgeOnARegularGrid();
    worleyEdgeIsAtLeastHalfTheDifference();
    if (failures != 0) {
        std::printf("%d check(s) failed\n", failures);
        return EXIT_FAILURE;
    }
    std::printf("all checks passed\n");
    return EXIT_SUCCESS;
}
