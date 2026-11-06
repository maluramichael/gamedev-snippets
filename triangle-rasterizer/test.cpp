// Tests for the triangle rasterizer logic. Plain asserts on hand-computed cases, no framework, no SDL.
//
// Units: a pixel is kSub = 16 sub-pixel units wide, its centre is at 16 * x + 8. Pixel (x, y) therefore has its centre
// at (x + 0.5, y + 0.5) in pixel units. y points down, like the screen.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <numeric>
#include <set>
#include <utility>
#include <vector>

#include "logic.hpp"

using namespace tr;

namespace {

/// Pixel-unit coordinates (may be fractional, multiples of 1/16) to sub-pixel units.
Pt px(double x, double y) { return {static_cast<int>(std::lround(x * kSub)), static_cast<int>(std::lround(y * kSub))}; }

std::set<std::pair<int, int>> pixelsOf(const Tri& t, Rule rule, int w = 16, int h = 16) {
    std::set<std::pair<int, int>> out;
    rasterize(t, rule, w, h, [&](int x, int y) { out.insert({x, y}); });
    return out;
}

/// Small deterministic generator, so the property test is the same on every platform.
struct Lcg {
    std::uint64_t s;
    int next(int n) {
        s = s * 6364136223846793005ULL + 1442695040888963407ULL;
        return static_cast<int>((s >> 33) % static_cast<std::uint64_t>(n));
    }
};

void edgeFunctionSigns() {
    // Edge (0,0)->(4,0): p below it (y grows down) is on the positive side, p above on the negative side, p on it is 0.
    assert(edgeFn({0, 0}, {4, 0}, {1, 2}) == 8);
    assert(edgeFn({0, 0}, {4, 0}, {1, -2}) == -8);
    assert(edgeFn({0, 0}, {4, 0}, {3, 0}) == 0);
    // The value is twice the signed area of (a, b, p): a 4 by 3 right triangle gives 12.
    assert(edgeFn({0, 0}, {4, 0}, {0, 3}) == 12);
    // Swapping the edge direction flips the sign.
    assert(edgeFn({4, 0}, {0, 0}, {1, 2}) == -8);
    // Big coordinates stay exact (this overflows a 32 bit int).
    assert(edgeFn({0, 0}, {1 << 20, 0}, {0, 1 << 20}) == (1LL << 40));
}

void orientation() {
    const Tri cw{{0, 0}, {64, 0}, {0, 64}};  // positive area2 (clockwise on a y-down screen)
    assert(area2(cw) == 64 * 64);
    const Tri flipped{cw.a, cw.c, cw.b};
    assert(area2(flipped) == -64 * 64);
    const Tri fixed = oriented(flipped);
    assert(area2(fixed) == 64 * 64);
    assert(area2(oriented(cw)) == 64 * 64);
    assert(area2(Tri{{0, 0}, {32, 32}, {64, 64}}) == 0);
}

void ownership() {
    // Top edge: runs left to right with the interior below it. Owned.
    assert(ownsEdge({0, 0}, {64, 0}));
    // Bottom edge: runs right to left. Not owned.
    assert(!ownsEdge({64, 64}, {0, 64}));
    // Left edge: runs upward. Owned. Right edge: runs downward. Not owned.
    assert(ownsEdge({0, 64}, {0, 0}));
    assert(!ownsEdge({64, 0}, {64, 64}));
    // Slanted edges follow dy.
    assert(ownsEdge({10, 50}, {40, 10}));
    assert(!ownsEdge({10, 10}, {40, 50}));
    // The two directions of any edge: exactly one of them is owned. That is the whole trick.
    Lcg rng{7};
    for (int i = 0; i < 2000; ++i) {
        const Pt a{rng.next(9) * 8, rng.next(9) * 8};
        const Pt b{rng.next(9) * 8, rng.next(9) * 8};
        if (a.x == b.x && a.y == b.y) continue;
        assert(ownsEdge(a, b) != ownsEdge(b, a));
    }
}

void singleTriangleByHand() {
    // Corners on pixel corners: (0,0) (4,0) (0,4). Pixel (x,y) has its centre at (x+.5, y+.5). The top and left edge pass
    // between the centres, but the hypotenuse x + y = 4 runs through the centres of the pixels with x + y == 3
    // (x + .5 + y + .5 = 4). Inside strictly: x + y <= 2, 3 + 2 + 1 = 6 pixels. With the hypotenuse: 4 more.
    const Tri a{px(0, 0), px(4, 0), px(0, 4)};
    for (Rule rule : {Rule::Exclusive, Rule::TopLeft}) {  // the hypotenuse is a bottom-right edge, not owned
        const auto s = pixelsOf(a, rule);
        assert(s.size() == 6);
        assert(s.count({0, 0}) && s.count({2, 0}) && s.count({0, 2}) && s.count({1, 1}));
        assert(!s.count({3, 0}) && !s.count({2, 1}) && !s.count({0, 3}));
    }
    const auto all = pixelsOf(a, Rule::Inclusive);
    assert(all.size() == 10);
    assert(all.count({3, 0}) && all.count({2, 1}) && all.count({1, 2}) && all.count({0, 3}));
    assert(!all.count({4, 0}) && !all.count({2, 2}));

    // The same triangle moved by half a pixel: (.5,.5) (4.5,.5) (.5,4.5). Now the top edge runs through the centres of
    // row 0, the left edge through the centres of column 0 and the hypotenuse through the centres with x + y = 4.
    const Tri b{px(0.5, 0.5), px(4.5, 0.5), px(0.5, 4.5)};
    // Inclusive: x >= 0, y >= 0, x + y <= 4: 5 + 4 + 3 + 2 + 1 = 15.
    const auto inc = pixelsOf(b, Rule::Inclusive);
    assert(inc.size() == 15);
    assert(inc.count({0, 0}) && inc.count({4, 0}) && inc.count({0, 4}) && inc.count({2, 2}));
    // Exclusive: x >= 1, y >= 1, x + y <= 3: (1,1) (2,1) (1,2).
    const auto exc = pixelsOf(b, Rule::Exclusive);
    assert(exc.size() == 3);
    assert(exc.count({1, 1}) && exc.count({2, 1}) && exc.count({1, 2}));
    // Top-left: owns the top and the left edge, not the hypotenuse: x >= 0, y >= 0, x + y <= 3: 4 + 3 + 2 + 1 = 10.
    const auto tl = pixelsOf(b, Rule::TopLeft);
    assert(tl.size() == 10);
    assert(tl.count({0, 0}) && tl.count({3, 0}) && tl.count({0, 3}));
    assert(!tl.count({4, 0}) && !tl.count({2, 2}) && !tl.count({0, 4}));
}

void windingDoesNotMatter() {
    const Tri b{px(0.5, 0.5), px(4.5, 0.5), px(0.5, 4.5)};
    const Tri flipped{b.a, b.c, b.b};
    for (Rule rule : {Rule::Inclusive, Rule::Exclusive, Rule::TopLeft}) {
        assert(pixelsOf(b, rule) == pixelsOf(flipped, rule));
    }
}

void squareSplitByADiagonal() {
    // Square (0,0)-(4,4) split along (0,0)-(4,4). The 4 pixels with x == y have their centre on the diagonal.
    const Tri lower{px(0, 0), px(4, 0), px(4, 4)};
    const Tri upper{px(0, 0), px(4, 4), px(0, 4)};
    const std::vector<Tri> tris{lower, upper};

    const auto tl = coverage(tris, Rule::TopLeft, 8, 8);
    for (int y = 0; y < 8; ++y) {
        for (int x = 0; x < 8; ++x) assert(tl[static_cast<std::size_t>(y * 8 + x)] == (x < 4 && y < 4 ? 1 : 0));
    }
    const auto inc = coverage(tris, Rule::Inclusive, 8, 8);
    const auto exc = coverage(tris, Rule::Exclusive, 8, 8);
    for (int y = 0; y < 4; ++y) {
        for (int x = 0; x < 4; ++x) {
            assert(inc[static_cast<std::size_t>(y * 8 + x)] == (x == y ? 2 : 1));
            assert(exc[static_cast<std::size_t>(y * 8 + x)] == (x == y ? 0 : 1));
        }
    }
    const Tally t = tally(inc, tl);
    assert(t.once == 12 && t.twice == 4 && t.holes == 0);
    const Tally e = tally(exc, tl);
    assert(e.once == 12 && e.twice == 0 && e.holes == 4);
    const Tally good = tally(tl, tl);
    assert(good.once == 16 && good.twice == 0 && good.holes == 0);
}

void fanOfASquare() {
    // Square (0,0)-(4,4) with the centre (2,2): four triangles. The spokes run along both diagonals, so the 8 pixels on
    // them have their centre on a shared edge. 16 pixels in all.
    Fan fan{px(2, 2), {px(0, 0), px(4, 0), px(4, 4), px(0, 4)}};
    assert(fanValid(fan));
    const auto tris = triangles(fan);
    assert(tris.size() == 4);
    const auto tl = coverage(tris, Rule::TopLeft, 8, 8);
    const auto inc = coverage(tris, Rule::Inclusive, 8, 8);
    const auto exc = coverage(tris, Rule::Exclusive, 8, 8);
    const Tally ti = tally(inc, tl);
    assert(ti.once == 8 && ti.twice == 8 && ti.holes == 0);
    const Tally te = tally(exc, tl);
    assert(te.once == 8 && te.twice == 0 && te.holes == 8);
    const Tally tt = tally(tl, tl);
    assert(tt.once == 16 && tt.twice == 0 && tt.holes == 0);
    // Total draws: 24 inclusive, 8 exclusive, 16 top-left.
    assert(std::accumulate(inc.begin(), inc.end(), 0) == 24);
    assert(std::accumulate(exc.begin(), exc.end(), 0) == 8);
    assert(std::accumulate(tl.begin(), tl.end(), 0) == 16);
}

void degenerateAndClipped() {
    // A sliver with all three corners on one line covers nothing, in every rule.
    for (Rule rule : {Rule::Inclusive, Rule::Exclusive, Rule::TopLeft}) {
        assert(pixelsOf(Tri{px(0, 0), px(2, 2), px(4, 4)}, rule).empty());
        assert(pixelsOf(Tri{px(1, 1), px(1, 1), px(1, 1)}, rule).empty());
        assert(pixelsOf(Tri{px(0, 1), px(5, 1), px(2, 1)}, rule).empty());
    }
    // A triangle that sticks out of the 6 x 6 grid on every side only reports pixels inside the grid.
    const Tri big{px(-20, -20), px(60, -20), px(-20, 60)};
    int count = 0;
    rasterize(big, Rule::TopLeft, 6, 6, [&](int x, int y) {
        assert(x >= 0 && x < 6 && y >= 0 && y < 6);
        ++count;
    });
    assert(count == 36);
    // Completely outside, to the left and above.
    assert(pixelsOf(Tri{px(-9, 1), px(-3, 1), px(-3, 5)}, Rule::TopLeft, 6, 6).empty());
    assert(pixelsOf(Tri{px(1, -9), px(5, -9), px(3, -3)}, Rule::TopLeft, 6, 6).empty());
    // Coordinates beyond the supported range are refused instead of overflowing the edge function.
    const Tri huge{{0, 0}, {kMaxCoord + 1, 0}, {0, kMaxCoord + 1}};
    assert(pixelsOf(huge, Rule::TopLeft).empty());
    const Tri limit{{0, 0}, {kMaxCoord, 0}, {0, kMaxCoord}};
    assert(pixelsOf(limit, Rule::TopLeft, 4, 4).size() == 16);
}

/// Splits a rectangle into triangles in a random way: first by a diagonal, then repeatedly one triangle by a point in
/// its inside (3 triangles) or by a point on one of its edges (2 triangles, which leaves a T-junction on the
/// neighbour). All corners sit on a half-pixel lattice, so pixel centres on edges are everywhere.
std::vector<Tri> randomSplit(Lcg& rng, Pt lo, Pt hi, int splits) {
    std::vector<Tri> tris;
    if (rng.next(2) == 0) {
        tris.push_back({lo, {hi.x, lo.y}, hi});
        tris.push_back({lo, hi, {lo.x, hi.y}});
    } else {
        tris.push_back({lo, {hi.x, lo.y}, {lo.x, hi.y}});
        tris.push_back({{hi.x, lo.y}, hi, {lo.x, hi.y}});
    }
    for (int i = 0; i < splits; ++i) {
        const std::size_t at = static_cast<std::size_t>(rng.next(static_cast<int>(tris.size())));
        const Tri t = oriented(tris[at]);
        if (rng.next(2) == 0) {
            // Inside point on the half-pixel lattice, strictly inside.
            const int minX = std::min({t.a.x, t.b.x, t.c.x});
            const int maxX = std::max({t.a.x, t.b.x, t.c.x});
            const int minY = std::min({t.a.y, t.b.y, t.c.y});
            const int maxY = std::max({t.a.y, t.b.y, t.c.y});
            for (int attempt = 0; attempt < 40; ++attempt) {
                const Pt p{minX + rng.next((maxX - minX) / 8 + 1) * 8, minY + rng.next((maxY - minY) / 8 + 1) * 8};
                if (edgeFn(t.a, t.b, p) > 0 && edgeFn(t.b, t.c, p) > 0 && edgeFn(t.c, t.a, p) > 0) {
                    tris[at] = {t.a, t.b, p};
                    tris.push_back({t.b, t.c, p});
                    tris.push_back({t.c, t.a, p});
                    break;
                }
            }
        } else {
            const Pt e[3] = {t.a, t.b, t.c};
            const int k = rng.next(3);
            const Pt a = e[k];
            const Pt b = e[(k + 1) % 3];
            const Pt c = e[(k + 2) % 3];
            const int g = std::gcd(std::abs(b.x - a.x), std::abs(b.y - a.y));
            if (g < 2) continue;
            const int step = 1 + rng.next(g - 1);
            const Pt m{a.x + (b.x - a.x) / g * step, a.y + (b.y - a.y) / g * step};
            tris[at] = {a, m, c};
            tris.push_back({m, b, c});
        }
    }
    return tris;
}

void propertyEveryPixelExactlyOnce() {
    // The claim of the half-open rule: any partition of a rectangle into triangles covers each pixel exactly once, and
    // that pixel is exactly the one whose centre lies in [x0, x1) x [y0, y1).
    constexpr int kW = 24;
    constexpr int kH = 18;
    Lcg rng{2024};
    long inclusiveDoubles = 0;
    long exclusiveHoles = 0;
    long trianglesTotal = 0;
    for (int round = 0; round < 400; ++round) {
        const int x0 = rng.next(8) * 8;
        const int y0 = rng.next(8) * 8;
        const Pt lo{x0, y0};
        const Pt hi{x0 + (1 + rng.next(14)) * 16 + rng.next(2) * 8, y0 + (1 + rng.next(10)) * 16 + rng.next(2) * 8};
        const auto tris = randomSplit(rng, lo, hi, rng.next(14));
        trianglesTotal += static_cast<long>(tris.size());
        const auto tl = coverage(tris, Rule::TopLeft, kW, kH);
        for (int y = 0; y < kH; ++y) {
            for (int x = 0; x < kW; ++x) {
                const int cx = x * kSub + kSub / 2;
                const int cy = y * kSub + kSub / 2;
                const bool in = cx >= lo.x && cx < hi.x && cy >= lo.y && cy < hi.y;
                assert(tl[static_cast<std::size_t>(y * kW + x)] == (in ? 1 : 0));
            }
        }
        const Tally ti = tally(coverage(tris, Rule::Inclusive, kW, kH), tl);
        const Tally te = tally(coverage(tris, Rule::Exclusive, kW, kH), tl);
        assert(ti.holes == 0);  // inclusive never leaves a hole ...
        assert(te.twice == 0);  // ... and exclusive never draws twice, the two naive rules fail in opposite ways
        inclusiveDoubles += ti.twice;
        exclusiveHoles += te.holes;
    }
    // The generator is nasty enough that both naive rules do break (otherwise the test above proves nothing).
    assert(trianglesTotal > 1500);
    assert(inclusiveDoubles > 500);
    assert(exclusiveHoles > 500);
}

void tallyCounts() {
    const std::vector<int> ref{1, 1, 1, 0, 1};
    const Tally t = tally({1, 2, 0, 0, 3}, ref);
    assert(t.once == 1 && t.twice == 2 && t.holes == 1);  // index 1 and 4 drawn 2 and 3 times, index 2 is a hole
    // Pixels outside the reference are not holes, even when nothing draws them.
    assert(tally({0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}).holes == 0);
}

void fanValidity() {
    // Regular hexagon: valid.
    Fan hex{px(10, 10), {px(16, 10), px(13, 15), px(7, 15), px(4, 10), px(7, 5), px(13, 5)}};
    assert(fanValid(hex));
    assert(triangles(hex).size() == 6);
    // The centre moved to (13,10) is still strictly inside the hexagon. On the corner (16,10) two triangles have area 0.
    Fan onRim = hex;
    onRim.center = px(13, 10);
    assert(fanValid(onRim));
    onRim.center = px(16, 10);
    assert(!fanValid(onRim));
    // One rim point pushed across the centre: the triangle flips.
    Fan flipped = hex;
    flipped.rim[3] = px(14, 10);
    assert(!fanValid(flipped));
    // A pentagram order winds twice around the centre: every triangle is positive, the sum of angles is 720 degrees.
    Fan star{px(10, 10), {}};
    const double pi = 3.14159265358979323846;
    for (int k = 0; k < 5; ++k) {
        const double a = 2.0 * pi * 2.0 * k / 5.0;
        star.rim.push_back(px(10 + 6 * std::cos(a), 10 + 6 * std::sin(a)));
    }
    assert(!fanValid(star));
    // Fewer than three rim points is no fan.
    assert(!fanValid(Fan{px(1, 1), {px(2, 2), px(3, 1)}}));
    // Replacing a rim point by a valid one keeps it valid.
    Fan moved = hex;
    moved.rim[0] = px(17, 11);
    assert(fanValid(moved));
}

void presets() {
    for (int n = 3; n <= 12; ++n) {
        const Fan f = presetFan(n, 36, 22);
        assert(static_cast<int>(f.rim.size()) == n);
        assert(fanValid(f));
        const auto tl = coverage(triangles(f), Rule::TopLeft, 36, 22);
        assert(std::accumulate(tl.begin(), tl.end(), 0) > 0);
        for (int c : tl) assert(c <= 1);  // a valid fan never overlaps itself
        // Everything stays inside the grid.
        for (const Pt& p : f.rim) assert(p.x >= 0 && p.x <= 36 * kSub && p.y >= 0 && p.y <= 22 * kSub);
        // Preset corners are pixel centres, so the demo starts with ties everywhere.
        for (const Pt& p : f.rim) assert(p.x % kSub == kSub / 2 && p.y % kSub == kSub / 2);
        assert(f.center.x % kSub == kSub / 2 && f.center.y % kSub == kSub / 2);
    }
}

void fanNeverOverlapsOrLeavesHolesWhenTopLeft() {
    // Random valid fans: top-left draws no pixel twice, and the covered area equals the triangles' own pixel sets.
    Lcg rng{99};
    int checked = 0;
    for (int round = 0; round < 300; ++round) {
        Fan f = presetFan(3 + rng.next(8), 36, 22);
        for (std::size_t i = 0; i < f.rim.size(); ++i) {
            Fan trial = f;
            trial.rim[i] = {rng.next(36 * 2) * 8, rng.next(22 * 2) * 8};
            if (fanValid(trial)) f = trial;
        }
        const auto tris = triangles(f);
        const auto tl = coverage(tris, Rule::TopLeft, 36, 22);
        for (int c : tl) assert(c <= 1);
        // Hole-free: every pixel whose centre lies inside the polygon (strictly, away from all edges) is drawn once.
        for (int y = 0; y < 22; ++y) {
            for (int x = 0; x < 36; ++x) {
                const Pt c = pixelCentre(x, y);
                bool strictlyIn = false;
                for (const Tri& t : tris) {
                    const Tri o = oriented(t);
                    if (edgeFn(o.a, o.b, c) > 0 && edgeFn(o.b, o.c, c) > 0 && edgeFn(o.c, o.a, c) > 0) strictlyIn = true;
                }
                if (strictlyIn) assert(tl[static_cast<std::size_t>(y * 36 + x)] == 1);
            }
        }
        ++checked;
    }
    assert(checked == 300);
}

void snapping() {
    // To the nearest pixel centre: pixel (3, 5) owns grid positions [3, 4) x [5, 6).
    assert(snapToCentre(3.2, 5.9).x == 3 * kSub + kSub / 2);
    assert(snapToCentre(3.2, 5.9).y == 5 * kSub + kSub / 2);
    assert(snapToCentre(0.0, 0.0).x == kSub / 2);
    assert(snapToCentre(3.999, 5.0).x == 3 * kSub + kSub / 2);
    // Negative positions floor instead of truncating towards zero.
    assert(snapToCentre(-0.2, -0.2).x == -kSub / 2);
    // Free: nearest 1/16 pixel.
    assert(snapToSub(1.0, 2.0).x == 16 && snapToSub(1.0, 2.0).y == 32);
    assert(snapToSub(1.03, 2.49).x == 16 && snapToSub(1.03, 2.49).y == 40);
    assert(snapToSub(1.04, 2.0).x == 17);
}

void snappingAFan() {
    Fan fan{px(2.3, 2.9), {px(0.1, 0.1), px(3.9, 0.5), px(3.5, 3.5), px(-0.2, 3.9)}};
    const Fan s = snappedFan(fan);
    assert(s.center.x == 2 * kSub + 8 && s.center.y == 2 * kSub + 8);
    assert(s.rim[0].x == 8 && s.rim[0].y == 8);
    assert(s.rim[1].x == 3 * kSub + 8 && s.rim[1].y == 8);
    assert(s.rim[2].x == 3 * kSub + 8 && s.rim[2].y == 3 * kSub + 8);
    assert(s.rim[3].x == -8 && s.rim[3].y == 3 * kSub + 8);  // -0.2 lies in pixel -1
    assert(centreOf({16, 15}).x == 24 && centreOf({16, 15}).y == 8);
}

void pixelCentreAndFloorDiv() {
    assert(pixelCentre(0, 0).x == 8 && pixelCentre(0, 0).y == 8);
    assert(pixelCentre(3, 2).x == 56 && pixelCentre(3, 2).y == 40);
    assert(floorDiv(17, 16) == 1 && floorDiv(16, 16) == 1 && floorDiv(15, 16) == 0);
    assert(floorDiv(-1, 16) == -1 && floorDiv(-16, 16) == -1 && floorDiv(-17, 16) == -2);
}

}  // namespace

int main() {
    edgeFunctionSigns();
    orientation();
    ownership();
    singleTriangleByHand();
    windingDoesNotMatter();
    squareSplitByADiagonal();
    fanOfASquare();
    degenerateAndClipped();
    propertyEveryPixelExactlyOnce();
    tallyCounts();
    fanValidity();
    presets();
    fanNeverOverlapsOrLeavesHolesWhenTopLeft();
    snapping();
    snappingAFan();
    pixelCentreAndFloorDiv();
    std::puts("triangle-rasterizer logic: all asserts passed");
    return 0;
}
