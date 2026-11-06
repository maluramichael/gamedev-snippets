// Tests for the Poisson disc logic. Plain asserts on hand-computed cases and on properties of the finished
// sample sets, no framework, no SDL.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <vector>

#include "logic.hpp"

using namespace pd;

namespace {

bool near(double a, double b, double eps = 1e-9) { return std::fabs(a - b) < eps; }

DensityMap flat(double w, double h) { return DensityMap(w, h); }

std::vector<Vec2> sample(double w, double h, double r, int k, std::uint64_t seed) {
    Sampler s(Params{w, h, r, k}, flat(w, h), seed);
    s.run();
    return s.points();
}

// ---- Rng ----

void rngMatchesSplitmix64() {
    // Reference values of splitmix64 started at state 0.
    Rng r(0);
    assert(r.next() == 0xE220A8397B1DCDAFULL);
    assert(r.next() == 0x6E789E6AA1B965F4ULL);
    assert(r.next() == 0x06C45D188009454FULL);
}

void rngUniformStaysInRange() {
    Rng r(42);
    double lo = 1.0;
    double hi = 0.0;
    for (int i = 0; i < 10000; ++i) {
        const double u = r.uniform();
        assert(u >= 0.0 && u < 1.0);
        lo = std::min(lo, u);
        hi = std::max(hi, u);
    }
    assert(lo < 0.01 && hi > 0.99);  // it really spreads over the whole range
}

void rngSeedsDiffer() {
    Rng a(1);
    Rng b(2);
    assert(a.next() != b.next());
    Rng c(1);
    Rng d(1);
    for (int i = 0; i < 10; ++i) assert(c.next() == d.next());
}

// ---- Density map ----

void mapStartsFlat() {
    DensityMap m(80, 40, 8.0);
    assert(m.isUniform());
    assert(near(m.minScale(), 1.0));
    assert(near(m.at({10, 10}), 1.0));
    assert(near(m.at({-50, -50}), 1.0));  // outside clamps to the edge cell
    assert(near(m.at({500, 500}), 1.0));
}

void mapPaintAndErase() {
    DensityMap m(80, 80, 8.0);
    // (4,4) is the centre of cell (0,0): distance 0 means full strength, scale kMinScale.
    m.paint({4, 4}, 20.0);
    assert(!m.isUniform());
    assert(near(m.at({4, 4}), DensityMap::kMinScale));
    assert(near(m.minScale(), DensityMap::kMinScale));
    // Cell (1,0) has its centre at (12,4): distance 8 of 20, falloff 0.6, scale 1 - 0.5 * 0.6 = 0.7.
    assert(near(m.at({12, 4}), 0.7));
    // Far away nothing changed.
    assert(near(m.at({70, 70}), 1.0));
    // Painting never makes a cell denser than the floor, painting twice changes nothing.
    m.paint({4, 4}, 20.0);
    assert(near(m.at({4, 4}), DensityMap::kMinScale));
    // Erasing at full strength restores the cell, at partial strength it moves halfway: cell (1,0) 0.7 -> 0.7 + 0.3 * 0.6.
    m.erase({4, 4}, 20.0);
    assert(near(m.at({4, 4}), 1.0));
    assert(near(m.at({12, 4}), 0.7 + 0.3 * 0.6));
    m.clear();
    assert(m.isUniform());
}

void mapOutsideBrushUntouched() {
    DensityMap m(80, 80, 8.0);
    m.paint({4, 4}, 8.0);  // cell (1,0) is exactly 8 away: falloff 0, no change
    assert(near(m.at({12, 4}), 1.0));
    m.paint({4, 4}, 0.0);  // a brush without size changes nothing and must not divide by zero
    assert(near(m.at({4, 4}), DensityMap::kMinScale));
}

// ---- Sampler ----

void gridCellsHoldOnePoint() {
    // Cell size is r / sqrt(2): the diagonal of a cell is exactly r, so two points in one cell would be closer than r.
    Sampler s(Params{100, 50, 10.0, 30}, flat(100, 50), 1);
    assert(near(s.cellSize(), 10.0 / std::sqrt(2.0)));
    assert(near(s.cellSize() * std::sqrt(2.0), 10.0));
    assert(s.cols() == 15);  // 100 / 7.07 = 14.14
    assert(s.rows() == 8);   // 50 / 7.07 = 7.07
    assert(s.grid().size() == 15U * 8U);
}

void startsWithOneActivePoint() {
    Sampler s(Params{200, 100, 10.0, 30}, flat(200, 100), 5);
    assert(s.points().size() == 1);
    assert(s.active().size() == 1);
    assert(!s.done());
    assert(s.candidatesTested() == 0);
    const Vec2 p = s.points()[0];
    assert(p.x >= 0 && p.x < 200 && p.y >= 0 && p.y < 100);
}

void everyStepEitherAddsOrRetires() {
    Sampler s(Params{200, 150, 12.0, 8}, flat(200, 150), 3);
    int guard = 0;
    while (!s.done() && guard++ < 100000) {
        const std::size_t points = s.points().size();
        const std::size_t active = s.active().size();
        const std::size_t tested = s.candidatesTested();
        assert(s.step());
        const auto& trials = s.lastTrials();
        assert(!trials.empty() && trials.size() <= 8);
        assert(s.candidatesTested() == tested + trials.size());
        // Every candidate lies in the ring between r and 2r around the point the step worked on.
        const Vec2 parent = s.points()[s.lastActive()];
        const double ra = s.radii()[s.lastActive()];
        for (const Trial& t : trials) {
            assert(distance(parent, t.pos) >= ra - 1e-9 && distance(parent, t.pos) <= 2.0 * ra + 1e-9);
        }
        if (s.points().size() == points + 1) {
            // Success: the new point joins the active list, the old one stays. Only the last trial is accepted.
            assert(s.active().size() == active + 1);
            assert(trials.back().accepted);
            for (std::size_t i = 0; i + 1 < trials.size(); ++i) assert(!trials[i].accepted);
        } else {
            // Failure: exactly k rejected trials and the active point is gone.
            assert(s.points().size() == points);
            assert(s.active().size() == active - 1);
            assert(trials.size() == 8);
            for (const Trial& t : trials) assert(!t.accepted);
        }
    }
    assert(s.done());
    assert(!s.step());  // nothing left to do
}

void noTwoPointsCloserThanR() {
    for (std::uint64_t seed : {1ULL, 2ULL, 99ULL}) {
        for (int k : {1, 5, 30}) {
            const auto pts = sample(300, 200, 14.0, k, seed);
            for (std::size_t i = 0; i < pts.size(); ++i) {
                assert(pts[i].x >= 0 && pts[i].x < 300 && pts[i].y >= 0 && pts[i].y < 200);
                for (std::size_t j = i + 1; j < pts.size(); ++j) assert(distance(pts[i], pts[j]) >= 14.0 - 1e-9);
            }
        }
    }
}

void gridLookupMatchesBruteForce() {
    // Every point sits in exactly the grid cell its coordinates belong to, and the grid holds nothing else.
    Sampler s(Params{250, 180, 11.0, 30}, flat(250, 180), 8);
    s.run();
    int filled = 0;
    for (int c : s.grid()) filled += c >= 0 ? 1 : 0;
    assert(static_cast<std::size_t>(filled) == s.points().size());  // one point per cell, none lost
    for (std::size_t i = 0; i < s.points().size(); ++i) {
        const int cx = static_cast<int>(s.points()[i].x / s.cellSize());
        const int cy = static_cast<int>(s.points()[i].y / s.cellSize());
        assert(s.grid()[static_cast<std::size_t>(cy * s.cols() + cx)] == static_cast<int>(i));
    }
}

void noBigHolesLeft() {
    // With k = 30 the finished set leaves no gap of 2r or more: a probe that far from every point would have been
    // found by the candidates of its neighbours. (Checked on a lattice of probes, fixed seeds, so deterministic.)
    for (std::uint64_t seed : {4ULL, 5ULL}) {
        const auto pts = sample(300, 200, 14.0, 30, seed);
        for (double y = 1.0; y < 200.0; y += 4.0) {
            for (double x = 1.0; x < 300.0; x += 4.0) {
                double best = std::numeric_limits<double>::infinity();
                for (const Vec2& p : pts) best = std::min(best, distance(p, {x, y}));
                assert(best < 2.0 * 14.0);
            }
        }
    }
}

void lowKStopsEarly() {
    // k = 1 gives up on a point after one try, so the set stays much sparser than with k = 30.
    const auto sparse = sample(300, 200, 14.0, 1, 6);
    const auto dense = sample(300, 200, 14.0, 30, 6);
    assert(sparse.size() * 10 < dense.size() * 8);
    assert(sparse.size() >= 2);
}

void workGrowsLinearly() {
    // Each step adds a point or retires one (see above), so at most 2n - 1 steps with at most k candidates each.
    for (int k : {1, 10, 30}) {
        Sampler s(Params{320, 240, 13.0, k}, flat(320, 240), 11);
        s.run();
        const std::size_t n = s.points().size();
        assert(s.candidatesTested() <= static_cast<std::size_t>(k) * (2 * n - 1));
        assert(s.candidatesTested() >= n - 1);  // every point except the first needed at least one candidate
    }
    // Four times the area gives about four times the points (the density stays the same), not sixteen times.
    const std::size_t small = sample(200, 150, 12.0, 30, 2).size();
    const std::size_t big = sample(400, 300, 12.0, 30, 2).size();
    assert(big > small * 32 / 10 && big < small * 48 / 10);
}

void tinyDomainKeepsOnlyTheSeed() {
    // 1 x 1 with r = 10: every candidate is 10 to 20 away from the seed and outside, so k rejected tries end it.
    Sampler s(Params{1, 1, 10.0, 7}, flat(1, 1), 1);
    s.run();
    assert(s.points().size() == 1);
    assert(s.candidatesTested() == 7);
    assert(s.done());
}

void paramsAreClamped() {
    Sampler s(Params{50, 50, -3.0, 0}, flat(50, 50), 1);  // radius < 1 and k < 1 are lifted to 1
    assert(s.cellSize() > 0.0);
    int guard = 0;
    while (s.step() && guard++ < 1000000) {
    }
    assert(s.done());
    assert(s.candidatesTested() >= 1);
}

void deterministicPerSeed() {
    const auto a = sample(200, 120, 12.0, 30, 17);
    const auto b = sample(200, 120, 12.0, 30, 17);
    const auto c = sample(200, 120, 12.0, 30, 18);
    assert(a.size() == b.size());
    for (std::size_t i = 0; i < a.size(); ++i) assert(a[i].x == b[i].x && a[i].y == b[i].y);
    assert(a[0].x != c[0].x || a[0].y != c[0].y);
}

// ---- Variable radius ----

void denseMapMeansMorePoints() {
    DensityMap m(200, 200, 8.0);
    m.paint({100, 100}, 2000.0);  // the whole map ends up near the floor, 0.5 to 0.55
    assert(m.minScale() < 0.55);
    Sampler dense(Params{200, 200, 20.0, 30}, m, 1);
    dense.run();
    const std::size_t flatCount = sample(200, 200, 20.0, 30, 1).size();
    // Half the radius means about four times the points; allow for the edges and the weaker falloff at the rim.
    assert(dense.points().size() > flatCount * 25 / 10);
}

void paintedSpotIsDenserThanTheRest() {
    DensityMap m(200, 200, 8.0);
    m.paint({100, 100}, 60.0);
    Sampler s(Params{200, 200, 20.0, 30}, m, 3);
    s.run();
    int center = 0;
    int corner = 0;
    for (const Vec2& p : s.points()) {
        if (distance(p, {100, 100}) < 25.0) ++center;
        if (distance(p, {30, 30}) < 25.0) ++corner;
    }
    // Same area, so the counts compare directly. The corner is untouched (r = 20), the middle is down to r = 10.
    assert(center > corner * 18 / 10);
}

void variableRadiusKeepsPairDistance() {
    // Each pair keeps at least the larger of its two radii apart. The radius of a point is the map value where it sits.
    DensityMap m(200, 200, 8.0);
    m.paint({60, 60}, 70.0);
    m.paint({150, 140}, 50.0);
    Sampler s(Params{200, 200, 18.0, 30}, m, 9);
    s.run();
    const auto& pts = s.points();
    const auto& radii = s.radii();
    assert(pts.size() == radii.size());
    for (std::size_t i = 0; i < pts.size(); ++i) {
        assert(near(radii[i], 18.0 * m.at(pts[i])));
        for (std::size_t j = i + 1; j < pts.size(); ++j) {
            assert(distance(pts[i], pts[j]) >= std::max(radii[i], radii[j]) - 1e-9);
        }
    }
}

// ---- Comparison helpers ----

void closePairsByHand() {
    const std::vector<Vec2> pts{{0, 0}, {3, 0}, {10, 0}};
    const std::vector<double> r5{5, 5, 5};
    const auto pairs = closePairs(pts, r5, 3);
    assert(pairs.size() == 1 && pairs[0].a == 0 && pairs[0].b == 1);
    assert(near(minSeparation(pts, r5, 3), 3.0 / 5.0));
    // Only the first two points count when n = 2, and a single point has no pair.
    assert(closePairs(pts, r5, 2).size() == 1);
    assert(closePairs(pts, r5, 1).empty());
    assert(closePairs(pts, r5, 0).empty());
    assert(std::isinf(minSeparation(pts, r5, 1)));
}

void touchingIsNotTooClose() {
    const std::vector<Vec2> pts{{0, 0}, {5, 0}};
    const std::vector<double> r{5, 5};
    assert(closePairs(pts, r, 2).empty());  // exactly r apart is allowed
    assert(near(minSeparation(pts, r, 2), 1.0));
    const std::vector<Vec2> nearly{{0, 0}, {4.999, 0}};
    assert(closePairs(nearly, r, 2).size() == 1);
}

void closePairsUseTheLargerRadius() {
    const std::vector<Vec2> pts{{0, 0}, {4, 0}};
    assert(closePairs(pts, std::vector<double>{5, 2}, 2).size() == 1);  // max(5, 2) = 5 > 4
    assert(closePairs(pts, std::vector<double>{2, 2}, 2).empty());
    assert(near(minSeparation(pts, std::vector<double>{5, 2}, 2), 4.0 / 5.0));
}

void naiveGeneratorsAreWorseThanPoisson() {
    const double w = 300;
    const double h = 300;
    const double r = 14;
    const auto poisson = sample(w, h, r, 30, 1);
    const std::size_t n = poisson.size();
    const std::vector<double> radii(n, r);
    assert(closePairs(poisson, radii, n).empty());
    assert(minSeparation(poisson, radii, n) >= 1.0 - 1e-9);
    Rng rng(1);
    const auto random = randomPoints(n, w, h, rng);
    assert(random.size() == n);
    assert(!closePairs(random, radii, n).empty());
    assert(minSeparation(random, radii, n) < 0.5);
    const auto jittered = jitteredGrid(n, w, h, rng);
    assert(!closePairs(jittered, radii, jittered.size()).empty());
}

void randomPointsStayInside() {
    Rng rng(3);
    for (const Vec2& p : randomPoints(500, 120, 80, rng)) assert(p.x >= 0 && p.x < 120 && p.y >= 0 && p.y < 80);
    assert(randomPoints(0, 10, 10, rng).empty());
}

void jitteredGridCountsAndCells() {
    // 100 points on 100 x 100: cell side sqrt(10000 / 100) = 10, a 10 x 10 grid with exactly one point per cell.
    Rng rng(5);
    const auto g = jitteredGrid(100, 100, 100, rng);
    assert(g.size() == 100);
    std::vector<int> perCell(100, 0);
    for (const Vec2& p : g) {
        assert(p.x >= 0 && p.x < 100 && p.y >= 0 && p.y < 100);
        ++perCell[static_cast<std::size_t>(static_cast<int>(p.y / 10.0) * 10 + static_cast<int>(p.x / 10.0))];
    }
    for (int c : perCell) assert(c == 1);
    // 100 points on 200 x 100: side sqrt(20000 / 100) = 14.14, round(200 / 14.14) = 14 by round(7.07) = 7 cells: 98 points.
    Rng rng2(5);
    assert(jitteredGrid(100, 200, 100, rng2).size() == 98);
    // The order is shuffled: the first ten points do not all come from the first grid row.
    int firstRow = 0;
    for (std::size_t i = 0; i < 10; ++i) firstRow += g[i].y < 10.0 ? 1 : 0;
    assert(firstRow < 10);
    Rng rng3(5);
    assert(jitteredGrid(0, 100, 100, rng3).empty());
}

// ---- Animation helpers ----

void revealFollowsTheLiveSampler() {
    assert(revealCount(0, 300, 200) == 0);
    assert(revealCount(100, 300, 200) == 150);  // half as far as the reference: half of the points
    assert(revealCount(200, 300, 200) == 300);
    assert(revealCount(250, 300, 200) == 300);  // never more than there are
    assert(revealCount(1, 300, 200) == 2);      // rounds up, 1.5 -> 2, so something shows from the first point
    assert(revealCount(5, 300, 0) == 300);      // no reference: show everything
}

void stepsDueKeepsTheRemainder() {
    double acc = 0.0;
    assert(stepsDue(acc, 0.05, 10.0, 100) == 0);  // half a step so far
    assert(near(acc, 0.05));
    assert(stepsDue(acc, 0.05, 10.0, 100) == 1);  // 0.1 s = one step
    assert(near(acc, 0.0, 1e-9));
    acc = 0.0;
    assert(stepsDue(acc, 0.25, 10.0, 100) == 2);  // 2.5 steps: two now, the half stays
    assert(near(acc, 0.05));
    assert(stepsDue(acc, 0.25, 10.0, 100) == 3);  // 0.3 s
    assert(near(acc, 0.0, 1e-9));
}

void stepsDueClampsAndCaps() {
    double acc = 0.0;
    assert(stepsDue(acc, 100.0, 10.0, 1000) == 2);  // a long pause counts as 0.25 s
    acc = 0.0;
    assert(stepsDue(acc, 0.25, 1000.0, 100) == 100);  // capped, the backlog is dropped
    assert(near(acc, 0.0));
    acc = 0.0;
    assert(stepsDue(acc, -1.0, 10.0, 100) == 0);  // a clock that went backwards gives nothing
    assert(near(acc, 0.0));
    assert(stepsDue(acc, 0.1, 0.0, 100) == 0);  // rate 0 means paused
}

}  // namespace

int main() {
    rngMatchesSplitmix64();
    rngUniformStaysInRange();
    rngSeedsDiffer();
    mapStartsFlat();
    mapPaintAndErase();
    mapOutsideBrushUntouched();
    gridCellsHoldOnePoint();
    startsWithOneActivePoint();
    everyStepEitherAddsOrRetires();
    noTwoPointsCloserThanR();
    gridLookupMatchesBruteForce();
    noBigHolesLeft();
    lowKStopsEarly();
    workGrowsLinearly();
    tinyDomainKeepsOnlyTheSeed();
    paramsAreClamped();
    deterministicPerSeed();
    denseMapMeansMorePoints();
    paintedSpotIsDenserThanTheRest();
    variableRadiusKeepsPairDistance();
    closePairsByHand();
    touchingIsNotTooClose();
    closePairsUseTheLargerRadius();
    naiveGeneratorsAreWorseThanPoisson();
    randomPointsStayInside();
    jitteredGridCountsAndCells();
    revealFollowsTheLiveSampler();
    stepsDueKeepsTheRemainder();
    stepsDueClampsAndCaps();
    std::puts("poisson-disc logic: all asserts passed");
    return 0;
}
