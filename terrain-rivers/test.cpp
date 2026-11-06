// Tests for the terrain-rivers logic. Plain asserts on hand-computed cases, no framework, no SDL.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <vector>

#include "logic.hpp"

using namespace rv;

namespace {

bool near(double a, double b, double eps = 1e-9) { return std::fabs(a - b) < eps; }

// A 7 x 3 grid. The whole border is an outlet (it drains off the map). Only the row y = 1, x = 1..5 is interior:
//
//   border: 1.0 everywhere, except the left cell (0,1) = 0.3
//   interior x = 1..5:  0.6  0.2  0.4  0.2  0.9
//
// Cell index = y * 7 + x, so the interior row is 8..12 and the low outlet (0,1) is 7.
constexpr int kW = 7;
constexpr int kH = 3;
constexpr double kSea = 0.0;  // everything is land

std::vector<double> basinRow() {
    std::vector<double> h(kW * kH, 1.0);
    h[7] = 0.3;
    h[8] = 0.6;
    h[9] = 0.2;
    h[10] = 0.4;
    h[11] = 0.2;
    h[12] = 0.9;
    return h;
}

void outletsAreSeaAndBorder() {
    // 3 x 3, middle row: sea (0.1), land (0.5), land (0.5); sea level 0.3.
    const std::vector<double> h{0.9, 0.9, 0.9, 0.1, 0.5, 0.5, 0.9, 0.9, 0.9};
    const auto o = outletMask(h, 3, 3, 0.3);
    for (int i = 0; i < 9; ++i) assert(o[static_cast<std::size_t>(i)] == (i != 4));  // only the center is not an outlet
    // Raise the sea above the center: now it is an outlet, too.
    assert(outletMask(h, 3, 3, 0.6)[4]);
    // Inland sea cell (not on the border) is an outlet even if no ocean reaches it.
    std::vector<double> big(25, 0.8);
    big[12] = 0.05;
    assert(outletMask(big, 5, 5, 0.3)[12]);
    assert(!outletMask(big, 5, 5, 0.3)[6]);
}

void fillRaisesBasinsToTheirPourPoint() {
    const auto h = basinRow();
    const auto outlet = outletMask(h, kW, kH, kSea);
    const double eps = 1e-6;
    const auto filled = fillSinks(h, kW, kH, outlet, eps);
    // The outlet row keeps its heights.
    assert(near(filled[7], 0.3));
    // x = 1 (0.6) is higher than the outlet next to it, so it is untouched.
    assert(near(filled[8], 0.6));
    // x = 2, 3, 4 lie in a basin whose lowest way out is over x = 1 (0.6): they rise to just above 0.6, one eps per step.
    assert(near(filled[9], 0.6 + eps));
    assert(near(filled[10], 0.6 + 2 * eps));
    assert(near(filled[11], 0.6 + 3 * eps));
    // x = 5 (0.9) is higher than the water next to it, so it is untouched.
    assert(near(filled[12], 0.9));
    // Filling never lowers a cell.
    for (std::size_t i = 0; i < h.size(); ++i) assert(filled[i] >= h[i]);
}

void lakeMaskMarksTheRaisedCells() {
    const auto h = basinRow();
    const auto outlet = outletMask(h, kW, kH, kSea);
    const auto filled = fillSinks(h, kW, kH, outlet, 1e-6);
    const auto lake = lakeMask(h, filled, outlet, kLakeDepth);
    int count = 0;
    for (std::size_t i = 0; i < lake.size(); ++i) {
        const bool expected = i == 9 || i == 10 || i == 11;
        assert(static_cast<bool>(lake[i]) == expected);
        count += lake[i] ? 1 : 0;
    }
    assert(count == 3);
    assert(countComponents(lake, kW, kH) == 1);
}

void componentsCountSeparateLakes() {
    // 5 x 3 mask: two single cells in the corners (they touch nothing) and a diagonal pair in the middle (8-connected).
    std::vector<char> m(15, 0);
    m[0] = 1;                    // (0,0)
    m[14] = 1;                   // (4,2)
    m[1 * 5 + 2] = 1;            // (2,1)
    m[2 * 5 + 3] = 1;            // (3,2) touches (4,2) and (2,1) diagonally
    assert(countComponents(m, 5, 3) == 2);  // {(0,0)} and the diagonal chain of three
    assert(countComponents(std::vector<char>(15, 0), 5, 3) == 0);
    assert(countComponents(std::vector<char>(15, 1), 5, 3) == 1);
}

void rawFlowEndsInPits() {
    const auto h = basinRow();
    const auto outlet = outletMask(h, kW, kH, kSea);
    const auto dir = flowDirections(h, kW, kH, outlet);
    // Border cells are outlets, they have no direction.
    for (int i = 0; i < kW * kH; ++i) {
        if (outlet[static_cast<std::size_t>(i)]) assert(dir[static_cast<std::size_t>(i)] == -1);
    }
    // x = 2 (0.2) and x = 4 (0.2) have no lower neighbor: pits.
    assert(dir[9] == -1);
    assert(dir[11] == -1);
    assert(countPits(dir, outlet) == 2);
    // x = 1 (0.6): the outlet at 0.3 drops 0.3 over distance 1, the pit x = 2 drops 0.4. Steepest wins: into the pit.
    assert(dir[8] == directionIndex(1, 0));
    // x = 3 (0.4): x = 2 and x = 4 are both 0.2 lower, the tie goes to the first neighbor in scan order (left).
    assert(dir[10] == directionIndex(-1, 0));
    // x = 5 (0.9) drops 0.7 into x = 4.
    assert(dir[12] == directionIndex(-1, 0));
}

void rawAccumulation() {
    const auto h = basinRow();
    const auto outlet = outletMask(h, kW, kH, kSea);
    const auto dir = flowDirections(h, kW, kH, outlet);
    const auto acc = accumulate(h, kW, dir, rainOf(h, kSea));
    assert(acc[12] == 1);  // x = 5 only has its own rain
    assert(acc[11] == 2);  // pit x = 4: itself + x = 5
    assert(acc[8] == 1);
    assert(acc[10] == 1);
    assert(acc[9] == 3);   // pit x = 2: itself + x = 1 + x = 3
    // Rain is conserved: every land cell is counted exactly once at an end of its path (an outlet or a pit).
    long total = 0;
    for (std::size_t i = 0; i < acc.size(); ++i) {
        if (dir[i] == -1) total += acc[i];
    }
    assert(total == kW * kH);
}

void filledFlowReachesTheOutlet() {
    const auto h = basinRow();
    const auto outlet = outletMask(h, kW, kH, kSea);
    const auto filled = fillSinks(h, kW, kH, outlet, 1e-6);
    const auto dir = flowDirections(filled, kW, kH, outlet);
    assert(countPits(dir, outlet) == 0);
    // The whole row drains left, one cell at a time, into the outlet (0,1).
    assert(dir[12] == directionIndex(-1, 0));
    assert(dir[11] == directionIndex(-1, 0));
    assert(dir[10] == directionIndex(-1, 0));
    assert(dir[9] == directionIndex(-1, 0));
    assert(dir[8] == directionIndex(-1, 0));
    const auto acc = accumulate(filled, kW, dir, rainOf(h, kSea));
    assert(acc[12] == 1);
    assert(acc[11] == 2);
    assert(acc[10] == 3);
    assert(acc[9] == 4);
    assert(acc[8] == 5);
    assert(acc[7] == 6);  // the outlet: its own rain plus the five cells of the row
    const auto path = tracePath(dir, kW, 12);
    assert(path.size() == 6);  // 12, 11, 10, 9, 8, 7
    assert(path.front() == 12 && path.back() == 7);
    // The same start in the raw terrain stops in the pit after two cells.
    const auto rawDir = flowDirections(h, kW, kH, outlet);
    const auto rawPath = tracePath(rawDir, kW, 12);
    assert(rawPath.size() == 2);
    assert(rawPath.back() == 11);
}

void diagonalsAreFartherAway() {
    // 3 x 3 land, center 0.5 is the only interior cell. Orthogonal neighbor above: 0.5 - 0.2 = 0.3 (drop 0.2).
    // Diagonal neighbor top-left: 0.5 - 0.27 = 0.23 (drop 0.27, but 0.27 / sqrt(2) = 0.191 < 0.2). The orthogonal one wins.
    std::vector<double> h(9, 0.9);
    h[4] = 0.5;
    h[1] = 0.3;
    h[0] = 0.23;
    const auto outlet = outletMask(h, 3, 3, kSea);
    const auto dir = flowDirections(h, 3, 3, outlet);
    assert(dir[4] == directionIndex(0, -1));
    // Make the diagonal drop 0.3: 0.3 / sqrt(2) = 0.212 > 0.2, now the diagonal wins.
    h[0] = 0.2;
    assert(flowDirections(h, 3, 3, outlet)[4] == directionIndex(-1, -1));
}

void directionTable() {
    assert(directionIndex(-1, -1) == 0);
    assert(directionIndex(1, 1) == 7);
    assert(directionIndex(0, 0) == -1);
    assert(directionIndex(2, 0) == -1);
    for (int d = 0; d < 8; ++d) assert(directionIndex(kDx[static_cast<std::size_t>(d)], kDy[static_cast<std::size_t>(d)]) == d);
    assert(near(stepLength(0), std::sqrt(2.0)));
    assert(near(stepLength(1), 1.0));
}

void riverRule() {
    assert(!isRiver(9, 10));
    assert(isRiver(10, 10));
    assert(isRiver(11, 10));
    assert(isRiver(5, 0));  // a threshold below 1 is clamped to 1, no division by zero
    assert(!isRiver(0, 0));
    assert(near(riverWidth(5, 10), 0.0));
    assert(near(riverWidth(10, 10), 1.0));
    assert(near(riverWidth(20, 10), 1.75));  // 1 + 0.75 * log2(2)
    assert(near(riverWidth(40, 10), 2.5));
    assert(near(riverWidth(100000, 10), 4.0));  // capped
    assert(riverWidth(10, 0) >= 0.0);  // no division by zero
    assert(riverWidth(10, -3) >= 0.0);
}

void thresholdSlider() {
    assert(thresholdFromT(0.0) == 4);
    assert(thresholdFromT(1.0) == 400);
    assert(thresholdFromT(0.5) == 40);  // geometric middle of 4 and 400
    assert(thresholdFromT(-3.0) == 4);
    assert(thresholdFromT(7.0) == 400);
    for (const int v : {4, 10, 25, 40, 100, 400}) assert(std::abs(thresholdFromT(tFromThreshold(v)) - v) <= 1);
    assert(near(tFromThreshold(4), 0.0));
    assert(near(tFromThreshold(400), 1.0));
    assert(near(tFromThreshold(1), 0.0));  // below the range clamps
    assert(near(tFromThreshold(100000), 1.0));
}

void linearSlider() {
    assert(near(stepValue(0.5, 0.1, 0.5, 0.01), 0.3));
    assert(near(stepValue(0.0, 0.1, 0.5, 0.01), 0.1));
    assert(near(stepValue(1.0, 0.1, 0.5, 0.01), 0.5));
    assert(near(stepValue(-4.0, 0.1, 0.5, 0.01), 0.1));
    assert(near(stepValue(9.0, 0.1, 0.5, 0.01), 0.5));
    assert(near(stepValue(0.5, 0.0, 1.0, 0.0), 0.5));  // step 0 means no snapping
    assert(near(stepValue(0.5, 2.0, 2.0, 0.1), 2.0));  // empty range
    assert(near(stepValue(0.44, 0.0, 1.0, 0.1), 0.4));
}

void cellLookup() {
    // Map drawn at (24, 44), 4 px per cell, 160 x 100 cells.
    const auto at = [](double x, double y) { return cellAt(x, y, 24.0, 44.0, 4.0, 160, 100); };
    assert(at(24.0, 44.0) == std::optional<int>(0));
    assert(at(27.9, 47.9) == std::optional<int>(0));
    assert(at(28.0, 44.0) == std::optional<int>(1));
    assert(at(24.0, 48.0) == std::optional<int>(160));
    assert(at(24.0 + 640.0 - 0.01, 44.0 + 400.0 - 0.01) == std::optional<int>(160 * 100 - 1));
    assert(!at(24.0 + 640.0, 50.0));
    assert(!at(30.0, 44.0 + 400.0));
    assert(!at(23.99, 50.0));
    assert(!at(30.0, 43.99));
    assert(!at(1e18, 1e18));
    assert(!cellAt(0, 0, 0, 0, 0.0, 10, 10));  // zero cell size
}

void colorsAndShade() {
    const Rgb sea = terrainColor(0.1, 0.3);
    const Rgb land = terrainColor(0.4, 0.3);
    assert(sea.b > sea.r);   // water is bluish
    assert(land.g > land.b); // low land is greenish
    const Rgb deep = terrainColor(0.0, 0.3);
    assert(deep.r <= sea.r && deep.g <= sea.g);  // deeper sea is darker
    const Rgb peak = terrainColor(1.0, 0.3);
    assert(peak.r > 200 && peak.g > 200 && peak.b > 200);  // white tops
    // Out of range heights do not crash or wrap.
    (void)terrainColor(-5.0, 0.3);
    (void)terrainColor(5.0, 0.3);
    // Hillshade: flat is 1, a slope facing the light (north-west) is brighter, one facing away is darker.
    std::vector<double> flat(9, 0.5);
    assert(near(hillshade(flat, 3, 3, 1, 1), 1.0));
    std::vector<double> eastUp{0.4, 0.5, 0.6, 0.4, 0.5, 0.6, 0.4, 0.5, 0.6};  // rises toward east: faces west
    assert(hillshade(eastUp, 3, 3, 1, 1) > 1.0);
    std::vector<double> westUp{0.6, 0.5, 0.4, 0.6, 0.5, 0.4, 0.6, 0.5, 0.4};
    assert(hillshade(westUp, 3, 3, 1, 1) < 1.0);
    std::vector<double> steep{0.0, 0.5, 5.0, 0.0, 0.5, 5.0, 0.0, 0.5, 5.0};
    assert(hillshade(steep, 3, 3, 1, 1) <= 1.4 && hillshade(steep, 3, 3, 1, 1) >= 0.6);  // clamped
    // Edge cells use the cell itself instead of reading outside.
    assert(near(hillshade(flat, 3, 3, 0, 0), 1.0));
    assert(near(hillshade(flat, 3, 3, 2, 2), 1.0));
}

void terrainBasics() {
    const auto a = makeTerrain(160, 100, 7);
    const auto b = makeTerrain(160, 100, 7);
    const auto c = makeTerrain(160, 100, 8);
    assert(a.size() == 16000);
    assert(a == b);  // deterministic
    assert(a != c);  // another seed, another land
    for (const double v : a) assert(v >= 0.0 && v <= 1.0);
    // The island shape: the border is below the default sea level, the middle has land.
    for (int x = 0; x < 160; ++x) {
        assert(a[static_cast<std::size_t>(x)] <= kDefaultSea);
        assert(a[static_cast<std::size_t>(99 * 160 + x)] <= kDefaultSea);
    }
    for (int y = 0; y < 100; ++y) {
        assert(a[static_cast<std::size_t>(y * 160)] <= kDefaultSea);
        assert(a[static_cast<std::size_t>(y * 160 + 159)] <= kDefaultSea);
    }
    int land = 0;
    for (const double v : a) land += v > kDefaultSea ? 1 : 0;
    assert(land > 16000 / 5 && land < 16000 * 4 / 5);
    // A higher sea level leaves less land.
    int landHigh = 0;
    for (const double v : a) landHigh += v > 0.5 ? 1 : 0;
    assert(landHigh < land);
    assert(makeTerrain(0, 10, 1).empty());
    assert(makeTerrain(10, -2, 1).empty());
}

/// The property that makes the whole thing work, on a real map: after filling, every land cell has a path to an
/// outlet. Before, the pits stop the water.
void everyDropReachesTheSeaAfterFilling() {
    const int w = 160;
    const int h = 100;
    for (const unsigned seed : {1u, 7u, 42u}) {
        const auto heights = makeTerrain(w, h, seed);
        const auto outlet = outletMask(heights, w, h, kDefaultSea);
        const auto rawDir = flowDirections(heights, w, h, outlet);
        const int rawPits = countPits(rawDir, outlet);
        assert(rawPits > 0);  // noise terrain has depressions, that is the whole point of the demo
        const auto filled = fillSinks(heights, w, h, outlet, kEps);
        const auto dir = flowDirections(filled, w, h, outlet);
        assert(countPits(dir, outlet) == 0);
        for (int i = 0; i < w * h; ++i) {
            if (outlet[static_cast<std::size_t>(i)]) continue;
            const auto path = tracePath(dir, w, i);
            assert(!path.empty());
            assert(path.size() <= static_cast<std::size_t>(w * h));
            assert(outlet[static_cast<std::size_t>(path.back())]);  // the drop ends at an outlet
            // The surface falls along the path, strictly.
            for (std::size_t k = 1; k < path.size(); ++k) {
                assert(filled[static_cast<std::size_t>(path[k])] < filled[static_cast<std::size_t>(path[k - 1])]);
            }
        }
        // Rain is conserved: the sum at the ends of all paths equals the number of land cells.
        const auto rain = rainOf(heights, kDefaultSea);
        const auto acc = accumulate(filled, w, dir, rain);
        long landCells = 0;
        long total = 0;
        for (int i = 0; i < w * h; ++i) {
            landCells += rain[static_cast<std::size_t>(i)];
            if (dir[static_cast<std::size_t>(i)] == -1) total += acc[static_cast<std::size_t>(i)];
        }
        assert(total == landCells);
        // Filling creates lakes where there were pits, and only raises cells.
        const auto lake = lakeMask(heights, filled, outlet, kLakeDepth);
        assert(countComponents(lake, w, h) > 0);
        for (std::size_t i = 0; i < heights.size(); ++i) assert(filled[i] >= heights[i]);
        std::printf("seed %u: %d raw pits, %d lakes\n", seed, rawPits, countComponents(lake, w, h));
    }
}

void worldSummary() {
    const World world = buildWorld(kW, kH, basinRow(), kSea);
    assert(world.rawPits == 2);
    assert(world.lakeCount == 1);
    assert(world.lakeCells == 3);
    assert(world.rawMaxAcc == 3);
    assert(world.maxAcc == 6);
    assert(countPits(world.dir, world.outlet) == 0);
    // Sea level above the whole row turns the row into outlets: no pits, no lakes, nothing flows.
    const World flooded = buildWorld(kW, kH, basinRow(), 0.95);
    assert(flooded.rawPits == 0);
    assert(flooded.lakeCount == 0);
    assert(flooded.maxAcc <= 1);
}

void deadEndsAndDemoDrop() {
    const auto h = basinRow();
    const World world = buildWorld(kW, kH, h, kSea);
    // Raw: pit 9 collected 3 cells, pit 11 collected 2.
    assert(countDeadEnds(world.rawDir, world.outlet, world.rawAcc, 2) == 2);
    assert(countDeadEnds(world.rawDir, world.outlet, world.rawAcc, 3) == 1);
    assert(countDeadEnds(world.rawDir, world.outlet, world.rawAcc, 4) == 0);
    assert(countDeadEnds(world.dir, world.outlet, world.acc, 1) == 0);  // after filling nothing ends inland
    // Longest raw path into a pit is 2 cells, first such cell is 8 (8 -> 9).
    assert(pickDemoDrop(world) == 8);
    // No land at all: nothing to pick.
    assert(pickDemoDrop(buildWorld(kW, kH, basinRow(), 5.0)) == -1);
    // On a real map the pick is a land cell that does end in a pit.
    const World big = buildWorld(160, 100, makeTerrain(160, 100, 7), kDefaultSea);
    const int pick = pickDemoDrop(big);
    assert(pick >= 0 && big.outlet[static_cast<std::size_t>(pick)] == 0);
    const auto path = tracePath(big.rawDir, 160, pick);
    assert(path.size() > 5);
    assert(big.outlet[static_cast<std::size_t>(path.back())] == 0);
}

void sliderStateNeverLeavesTheRange() {
    assert(clampSea(-1.0) == kSeaMin);
    assert(clampSea(9.0) == kSeaMax);
    assert(near(clampSea(0.33), 0.33));
}

}  // namespace

int main() {
    outletsAreSeaAndBorder();
    fillRaisesBasinsToTheirPourPoint();
    lakeMaskMarksTheRaisedCells();
    componentsCountSeparateLakes();
    rawFlowEndsInPits();
    rawAccumulation();
    filledFlowReachesTheOutlet();
    diagonalsAreFartherAway();
    directionTable();
    riverRule();
    thresholdSlider();
    linearSlider();
    cellLookup();
    colorsAndShade();
    terrainBasics();
    everyDropReachesTheSeaAfterFilling();
    worldSummary();
    deadEndsAndDemoDrop();
    sliderStateNeverLeavesTheRange();
    std::puts("terrain-rivers logic: all tests passed");
    return 0;
}
