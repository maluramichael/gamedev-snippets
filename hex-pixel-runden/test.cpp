// Tests for the pixel to hex logic. Plain asserts on hand-computed cases, no framework, no SDL.
#undef NDEBUG
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

#include "logic.hpp"

using namespace hpr;

namespace {

bool near(double a, double b, double eps = 1e-9) { return std::fabs(a - b) < eps; }
bool same(Cube a, Cube b) { return a.q == b.q && a.r == b.r && a.s == b.s; }
bool nearPoint(Vec2 a, Vec2 b) { return near(a.x, b.x, 1e-9) && near(a.y, b.y, 1e-9); }

constexpr double kSqrt3 = 1.7320508075688772;

// Size 10 at the origin: every pixel below is computed by hand from x = 10 * sqrt(3) * (q + r / 2), y = 15 * r (pointy)
// and x = 15 * q, y = 10 * (sqrt(3) / 2 * q + sqrt(3) * r) (flat).
const Layout kPointy{false, 10.0, {0.0, 0.0}};
const Layout kFlat{true, 10.0, {0.0, 0.0}};

void hexToPixel() {
    assert(nearPoint(toPixel(kPointy, {0, 0, 0}), {0.0, 0.0}));
    assert(nearPoint(toPixel(kPointy, {1, 0, -1}), {10.0 * kSqrt3, 0.0}));
    assert(nearPoint(toPixel(kPointy, {0, 1, -1}), {5.0 * kSqrt3, 15.0}));  // down and to the right
    assert(nearPoint(toPixel(kPointy, {-1, 1, 0}), {-5.0 * kSqrt3, 15.0}));  // down and to the left
    assert(nearPoint(toPixel(kFlat, {1, 0, -1}), {15.0, 5.0 * kSqrt3}));
    assert(nearPoint(toPixel(kFlat, {0, 1, -1}), {0.0, 10.0 * kSqrt3}));
    // The origin moves everything.
    const Layout shifted{false, 10.0, {100.0, 50.0}};
    assert(nearPoint(toPixel(shifted, {0, 1, -1}), {100.0 + 5.0 * kSqrt3, 65.0}));
}

void pixelToFractional() {
    const Frac a = toFractional(kPointy, {5.0 * kSqrt3, 15.0});  // the center of hex (0, 1, -1)
    assert(near(a.q, 0.0) && near(a.r, 1.0) && near(a.s, -1.0));
    const Frac b = toFractional(kFlat, {15.0, 5.0 * kSqrt3});  // the center of hex (1, 0, -1)
    assert(near(b.q, 1.0) && near(b.r, 0.0) && near(b.s, -1.0));
    // A pixel half way between two centers lands half way between their coordinates.
    const Frac c = toFractional(kPointy, {2.5 * kSqrt3, 0.0});  // between (0,0,0) and (1,0,-1)
    assert(near(c.q, 0.25) && near(c.r, 0.0) && near(c.s, -0.25));
    // The third component is always derived, so the sum cannot drift.
    const Frac d = toFractional(kFlat, {37.3, -81.9});
    assert(near(d.q + d.r + d.s, 0.0));
    // A layout without a size cannot divide: the answer is the origin hex, not NaN.
    const Frac z = toFractional(Layout{false, 0.0, {0.0, 0.0}}, {5.0, 5.0});
    assert(near(z.q, 0.0) && near(z.r, 0.0) && near(z.s, 0.0));
}

void roundTripOfCenters() {
    for (const Layout& layout : {kPointy, kFlat}) {
        for (int q = -6; q <= 6; ++q) {
            for (int r = -6; r <= 6; ++r) {
                const Cube h = fromAxial(q, r);
                assert(same(pickHex(layout, toPixel(layout, h)), h));
            }
        }
    }
}

void roundingWithoutCorrection() {
    // (1.2, -0.4, -0.8) rounds to (1, 0, -1) which already sums to 0: nothing to fix.
    const Rounding o = roundDetail({1.2, -0.4, -0.8});
    assert(same(o.naive, {1, 0, -1}) && same(o.fixed, {1, 0, -1}));
    assert(o.fixedAxis == -1);
    assert(valid(o.naive));
    assert(near(o.dq, 0.2) && near(o.dr, 0.4) && near(o.ds, 0.2));
}

void roundingFixesTheLargestError() {
    // (0.4, 0.3, -0.7): naive (0, 0, -1) sums to -1. Errors 0.4, 0.3, 0.3: q is off the most, q = -r - s = 1.
    const Rounding a = roundDetail({0.4, 0.3, -0.7});
    assert(same(a.naive, {0, 0, -1}) && !valid(a.naive));
    assert(a.fixedAxis == 0);
    assert(same(a.fixed, {1, 0, -1}));
    assert(near(a.dq, 0.4));
    // The mirror image: (-0.4, -0.3, 0.7), naive (0, 0, 1) sums to 1, q = -r - s = -1.
    const Rounding b = roundDetail({-0.4, -0.3, 0.7});
    assert(same(b.naive, {0, 0, 1}) && !valid(b.naive));
    assert(b.fixedAxis == 0 && same(b.fixed, {-1, 0, 1}));
    // r is the largest error: (0.3, 0.4, -0.7), naive (0, 0, -1), r = -q - s = 1.
    const Rounding c = roundDetail({0.3, 0.4, -0.7});
    assert(c.fixedAxis == 1 && same(c.fixed, {0, 1, -1}));
    // (0.6, 0.6, -1.2): naive (1, 1, -1) sums to 1 with errors 0.4, 0.4, 0.2. q and r tie, the tie goes to r.
    const Rounding d = roundDetail({0.6, 0.6, -1.2});
    assert(same(d.naive, {1, 1, -1}) && d.fixedAxis == 1 && same(d.fixed, {1, 0, -1}));
    // s only wins when it is strictly the largest: (0.3, 0.3, -0.6) rounds to (0, 0, -1), errors 0.3, 0.3, 0.4.
    const Rounding e = roundDetail({0.3, 0.3, -0.6});
    assert(same(e.naive, {0, 0, -1}) && e.fixedAxis == 2 && same(e.fixed, {0, 0, 0}));
}

void roundingTies() {
    // (0.5, 0.5, -1): naive (1, 1, -1) sums to 1, errors 0.5, 0.5, 0. q and r tie, the rule picks r.
    const Rounding o = roundDetail({0.5, 0.5, -1.0});
    assert(same(o.naive, {1, 1, -1}));
    assert(o.fixedAxis == 1 && same(o.fixed, {1, 0, -1}));
    // Both candidates are equally close, so the answer is a convention but must be a real hex.
    assert(valid(o.fixed));
}

void roundingNeverLeavesTheHexGrid() {
    // Extreme and broken input is clamped instead of overflowing the int cast.
    const double inf = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (const Frac f : {Frac{inf, -inf, 0.0}, Frac{nan, nan, nan}, Frac{1e300, -1e300, 0.0}}) {
        assert(valid(cubeRound(f)));
    }
    assert(same(cubeRound({nan, nan, nan}), {0, 0, 0}));
}

void naiveRoundingFailsSometimes() {
    // Walk over a grid of pixels. The corrected result is always a hex, the naive one is not.
    int invalid = 0;
    int total = 0;
    for (const Layout& layout : {kPointy, kFlat}) {
        for (double y = -60.0; y <= 60.0; y += 1.7) {
            for (double x = -60.0; x <= 60.0; x += 1.7) {
                const Rounding o = roundDetail(toFractional(layout, {x, y}));
                assert(valid(o.fixed));
                if (valid(o.naive)) {
                    assert(o.fixedAxis == -1 && same(o.naive, o.fixed));
                } else {
                    assert(o.fixedAxis >= 0);
                    ++invalid;
                }
                ++total;
            }
        }
    }
    // The rounded fractions a, b, -a-b sum to 0 only if |a + b| < 0.5, where a and b are the offsets to the nearest
    // integers. Over the unit square that fails on two triangles of 1/8 each: a quarter of the plane.
    const double share = static_cast<double>(invalid) / static_cast<double>(total);
    assert(share > 0.2 && share < 0.3);
}

void pickedHexIsTheNearestCenter() {
    // Brute force: the hex whose center is closest in pixels wins. Deterministic pseudo random points, no ties in practice.
    unsigned long long seed = 12345;
    auto next = [&seed]() {
        seed = seed * 6364136223846793005ULL + 1442695040888963407ULL;
        return static_cast<double>(seed >> 11) / 9007199254740992.0;  // [0, 1)
    };
    for (const Layout& layout : {kPointy, kFlat}) {
        for (int i = 0; i < 4000; ++i) {
            const Vec2 p{next() * 200.0 - 100.0, next() * 200.0 - 100.0};
            Cube best{0, 0, 0};
            double bestDist = 1e18;
            for (int q = -12; q <= 12; ++q) {
                for (int r = -12; r <= 12; ++r) {
                    const Vec2 c = toPixel(layout, fromAxial(q, r));
                    const double d = (c.x - p.x) * (c.x - p.x) + (c.y - p.y) * (c.y - p.y);
                    if (d < bestDist) {
                        bestDist = d;
                        best = fromAxial(q, r);
                    }
                }
            }
            assert(same(pickHex(layout, p), best));
        }
    }
}

void hexDistance() {
    assert(distance({0, 0, 0}, {0, 0, 0}) == 0);
    assert(distance({0, 0, 0}, {3, -1, -2}) == 3);
    assert(distance({2, -3, 1}, {-1, 1, 0}) == 4);  // differences 3, -4, 1
    assert(distance({-1, 1, 0}, {2, -3, 1}) == 4);  // symmetric
    assert(distance({0, 0, 0}, {1, 0, -1}) == 1);   // neighbors
}

void cornersOfAHex() {
    // Flat: the first corner points right, pointy: the first corner is up and to the right.
    const auto flat = corners(kFlat, {0, 0, 0});
    assert(nearPoint(flat[0], {10.0, 0.0}));
    assert(nearPoint(flat[3], {-10.0, 0.0}));
    const auto pointy = corners(kPointy, {0, 0, 0});
    assert(nearPoint(pointy[0], {5.0 * kSqrt3, -5.0}));
    assert(nearPoint(pointy[2], {0.0, 10.0}));  // the corner at the bottom (y points down)
    for (const Layout& layout : {kPointy, kFlat}) {
        const Vec2 c = toPixel(layout, {1, -1, 0});
        const auto k = corners(layout, {1, -1, 0});
        for (int i = 0; i < 6; ++i) {
            // Regular: every corner is `size` from the center and from the next corner.
            assert(near(std::hypot(k[i].x - c.x, k[i].y - c.y), 10.0));
            const Vec2 n = k[(i + 1) % 6];
            assert(near(std::hypot(k[i].x - n.x, k[i].y - n.y), 10.0));
        }
        // Neighbors share exactly two corners, which ties the pixel formula to the outline.
        const auto other = corners(layout, {2, -1, -1});
        int shared = 0;
        for (const Vec2& a : k) {
            for (const Vec2& b : other) {
                if (nearPoint(a, b)) ++shared;
            }
        }
        assert(shared == 2);
        // A hex two steps away shares nothing.
        const auto distant = corners(layout, {3, -1, -2});
        for (const Vec2& a : k) {
            for (const Vec2& b : distant) assert(!nearPoint(a, b));
        }
    }
}

void insideTheField() {
    const Rect field{100.0, 50.0, 300.0, 250.0};
    const Layout layout{false, 20.0, {200.0, 150.0}};
    assert(inBounds(layout, field, {0, 0, 0}));
    // The center must be at least `size` away from the border so the whole hex fits.
    const Layout onEdge{false, 20.0, {120.0, 150.0}};  // center x = 120 = left edge 100 + size
    assert(inBounds(onEdge, field, {0, 0, 0}));
    const Layout justOutside{false, 20.0, {119.0, 150.0}};
    assert(!inBounds(justOutside, field, {0, 0, 0}));
    const Layout top{false, 20.0, {200.0, 70.0}};  // top edge 50 + size
    assert(inBounds(top, field, {0, 0, 0}));
    assert(!inBounds(Layout{false, 20.0, {200.0, 69.0}}, field, {0, 0, 0}));
    // Far away hexes are out. With 20 px hexes at x spacing 34.64 the hex (4, 0) is at x = 338.6 > 280.
    assert(!inBounds(layout, field, {4, 0, -4}));
    assert(inBounds(layout, field, {2, 0, -2}));  // x = 269.3
}

void cellsCoverTheWholeField() {
    // The search window in cells() must be wide enough: nothing is cut off at its edge.
    for (const bool flat : {false, true}) {
        const Layout layout = makeLayout(flat);
        const std::vector<Cube> all = cells(layout);
        assert(all.size() > 60);
        for (const Cube& c : all) {
            assert(valid(c));
            assert(inBounds(layout, kField, c));
            assert(std::abs(c.q) < kSearch - 1 && std::abs(c.r) < kSearch - 1);  // the window has room to spare
        }
        assert(std::find_if(all.begin(), all.end(), [](Cube c) { return same(c, {0, 0, 0}); }) != all.end());
    }
}

void lineAlongAStraightDirection() {
    // Four steps along (+1, -1, 0): every sample sits exactly on a hex center.
    const auto l = line({0, 0, 0}, {4, -4, 0});
    assert(l.size() == 5);
    for (int i = 0; i <= 4; ++i) assert(same(l[static_cast<std::size_t>(i)], {i, -i, 0}));
    // A start equal to the end is one hex.
    const auto one = line({2, -1, -1}, {2, -1, -1});
    assert(one.size() == 1 && same(one[0], {2, -1, -1}));
}

void lineHandComputed() {
    // From (0, 0, 0) to (3, -1, -2), N = 3. Sample 1 at t = 1/3 is (1, -1/3, -2/3) which rounds to (1, 0, -1),
    // sample 2 at t = 2/3 is (2, -2/3, -4/3) which rounds to (2, -1, -1).
    const auto l = line({0, 0, 0}, {3, -1, -2});
    assert(l.size() == 4);
    assert(same(l[0], {0, 0, 0}) && same(l[1], {1, 0, -1}) && same(l[2], {2, -1, -1}) && same(l[3], {3, -1, -2}));
    const auto samples = lineSamples({0, 0, 0}, {3, -1, -2}, false);
    assert(samples.size() == 4);
    assert(near(samples[1].q, 1.0) && near(samples[1].r, -1.0 / 3.0) && near(samples[1].s, -2.0 / 3.0));
    assert(near(samples[3].q, 3.0) && near(samples[3].r, -1.0) && near(samples[3].s, -2.0));
}

void lineNudge() {
    // From (0, 0, 0) to (2, -1, -1) the middle sample is exactly (1, -0.5, -0.5): on the edge between (1, -1, 0)
    // and (1, 0, -1). Without the nudge the tie rule decides, with the nudge the line moves a hair off the edge
    // and the answer no longer depends on the tie rule.
    const Cube a{0, 0, 0};
    const Cube b{2, -1, -1};
    const auto plain = line(a, b, false);
    assert(plain.size() == 3 && same(plain[1], {1, -1, 0}));
    const auto nudged = line(a, b, true);
    assert(nudged.size() == 3 && same(nudged[1], {1, 0, -1}));
    // And walking it backwards gives the same middle hex.
    const auto back = line(b, a, true);
    assert(same(back[1], {1, 0, -1}));
}

void lineProperties() {
    const Cube starts[] = {{0, 0, 0}, {-3, 1, 2}, {2, -5, 3}, {4, 4, -8}};
    const Cube ends[] = {{3, -1, -2}, {5, -5, 0}, {-4, 6, -2}, {1, 2, -3}, {-3, 1, 2}};
    for (const bool nudge : {false, true}) {
        for (const Cube& a : starts) {
            for (const Cube& b : ends) {
                const auto l = line(a, b, nudge);
                assert(static_cast<int>(l.size()) == distance(a, b) + 1);
                assert(same(l.front(), a) && same(l.back(), b));
                for (std::size_t i = 0; i < l.size(); ++i) {
                    assert(valid(l[i]));
                    if (i > 0) assert(distance(l[i - 1], l[i]) == 1);  // a line of neighbors, no gaps, no jumps
                }
            }
        }
    }
}

void lineSamplesCanFailNaively() {
    // (0, 0, 0) to (3, 3, -6) takes N = 6 steps. Sample 1 at t = 1/6 is (0.5, 0.5, -1): the naive rounding gives
    // (1, 1, -1) which sums to 1, so it is no hex at all.
    const auto samples = lineSamples({0, 0, 0}, {3, 3, -6}, false);
    assert(samples.size() == 7);
    assert(near(samples[1].q, 0.5) && near(samples[1].r, 0.5) && near(samples[1].s, -1.0));
    assert(!valid(roundDetail(samples[1]).naive));
    assert(valid(cubeRound(samples[1])));
}

void startState() {
    // The first frame is meant to show a correction, in the pointy layout the demo starts in.
    const Rounding o = roundDetail(toFractional(makeLayout(false), kStartMouse));
    assert(!valid(o.naive));
    assert(o.fixedAxis == 0);
    assert(same(o.fixed, {0, 0, 0}));
    assert(inBounds(makeLayout(false), kField, o.fixed));
    // The line start is a hex of the grid in both layouts.
    assert(inBounds(makeLayout(false), kField, kStartLine));
    assert(inBounds(makeLayout(true), kField, kStartLine));
}

}  // namespace

int main() {
    hexToPixel();
    pixelToFractional();
    roundTripOfCenters();
    roundingWithoutCorrection();
    roundingFixesTheLargestError();
    roundingTies();
    roundingNeverLeavesTheHexGrid();
    naiveRoundingFailsSometimes();
    pickedHexIsTheNearestCenter();
    hexDistance();
    cornersOfAHex();
    insideTheField();
    cellsCoverTheWholeField();
    lineAlongAStraightDirection();
    lineHandComputed();
    lineNudge();
    lineProperties();
    lineSamplesCanFailNaively();
    startState();
    std::puts("hex-pixel-runden logic: all asserts passed");
    return 0;
}
