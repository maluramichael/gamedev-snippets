// Tests for the bid-rent town logic. Plain asserts on hand-computed cases, no framework, no SDL.
// Defaults: size 1, slopes farm 0.6 / house 4 / market 12, intercepts 20 / 60 / 100 (times size for house and market).
#undef NDEBUG
#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "logic.hpp"

using namespace brt;

namespace {

bool near(double a, double b, double eps = 1e-6) { return std::fabs(a - b) < eps; }

Params defaults() { return Params{}; }

void bidsOnTheDefaultCurves() {
    const Params p = defaults();
    const Bids b0 = bids(p, 0.0, kNoWater);
    assert(near(b0.farm, 20.0) && near(b0.house, 60.0) && near(b0.market, 100.0));
    const Bids b5 = bids(p, 5.0, kNoWater);
    assert(near(b5.farm, 17.0) && near(b5.house, 40.0) && near(b5.market, 40.0));
}

void ringsAlongARay() {
    const Params p = defaults();
    for (int d = 0; d <= 5; ++d) assert(useAt(p, d, kNoWater) == Use::Market);
    assert(useAt(p, 6.0, kNoWater) == Use::House);  // market 28, house 36
    assert(useAt(p, 11.0, kNoWater) == Use::House);  // house 16, farm 13.4
    assert(useAt(p, 12.0, kNoWater) == Use::Farm);  // house 12, farm 12.8
    assert(useAt(p, 33.0, kNoWater) == Use::Farm);  // farm 0.2
    assert(useAt(p, 34.0, kNoWater) == Use::Wild);  // farm -0.4, the others are lower still
}

void tieGoesToTheDenserUse() {
    const Params p = defaults();
    // d = 5: market and house both bid exactly 40. The denser use wins, so a tie never flickers.
    assert(useAt(p, 5.0, kNoWater) == Use::Market);
    // d = 4.999 and 5.001 sit on the two sides of the tie.
    assert(useAt(p, 4.999, kNoWater) == Use::Market);
    assert(useAt(p, 5.001, kNoWater) == Use::House);
}

void nothingAboveZeroIsWild() {
    Params p = defaults();
    p.slopeFarm = 0.5;  // 20 - 0.5 * 40 = 0 exactly
    assert(useAt(p, 39.0, kNoWater) == Use::Farm);
    assert(useAt(p, 40.0, kNoWater) == Use::Wild);  // a bid of exactly 0 is not worth building on
}

void boundariesOfTheDefaults() {
    const auto b = boundaries(defaults(), kNoWater, 36.0);
    assert(b.size() == 3);
    assert(b[0].from == Use::Market && b[0].to == Use::House && near(b[0].d, 5.0, 1e-4));
    assert(b[1].from == Use::House && b[1].to == Use::Farm && near(b[1].d, 40.0 / 3.4, 1e-4));
    assert(b[2].from == Use::Farm && b[2].to == Use::Wild && near(b[2].d, 100.0 / 3.0, 1e-4));
}

void growthPushesTheRingsOut() {
    Params p = defaults();
    p.size = 1.5;  // market 150 - 12 d, house 90 - 4 d
    auto b = boundaries(p, kNoWater, 36.0);
    assert(b.size() == 3);
    assert(near(b[0].d, 60.0 / 8.0, 1e-4));  // 7.5
    assert(near(b[1].d, 70.0 / 3.4, 1e-4));  // 20.588
    p.size = 0.5;  // market 50 - 12 d, house 30 - 4 d
    b = boundaries(p, kNoWater, 36.0);
    assert(b.size() == 3);
    assert(near(b[0].d, 20.0 / 8.0, 1e-4));  // 2.5
    assert(near(b[1].d, 10.0 / 3.4, 1e-4));  // 2.941
    // Over the whole range the first two rings only ever move outward.
    double lastMarket = 0.0;
    double lastHouse = 0.0;
    for (int i = 0; i <= 10; ++i) {
        p.size = 0.5 + 0.1 * i;
        const auto e = boundaries(p, kNoWater, 36.0);
        assert(e.size() == 3);
        assert(e[0].d >= lastMarket - 1e-9 && e[1].d >= lastHouse - 1e-9);
        lastMarket = e[0].d;
        lastHouse = e[1].d;
    }
}

void aSmallTownHasNoHouses() {
    Params p = defaults();
    p.size = 0.3;  // house starts at 18, below the farm's 20, so it is never the maximum
    const auto b = boundaries(p, kNoWater, 36.0);
    assert(b.size() == 2);
    assert(b[0].from == Use::Market && b[0].to == Use::Farm && near(b[0].d, 10.0 / 11.4, 1e-4));
    assert(b[1].from == Use::Farm && b[1].to == Use::Wild);
}

void slopesDecideTheOrder() {
    Params p = defaults();
    p.slopeMarket = 4.0;  // flatter than the house curve and higher: house never wins
    p.slopeHouse = 12.0;
    const auto b = boundaries(p, kNoWater, 36.0);
    assert(b.size() == 2);
    assert(b[0].from == Use::Market && b[0].to == Use::Farm && near(b[0].d, 80.0 / 3.4, 1e-4));
    Params flat = defaults();
    flat.slopeMarket = 0.0;  // a market curve with no slope swallows everything
    assert(useAt(flat, 35.0, kNoWater) == Use::Market);
    assert(boundaries(flat, kNoWater, 36.0).empty());
}

void waterLiftsTheBank() {
    Params p = defaults();
    p.water = 1.0;
    // dw = 1: bonus 60 * (1 - 1/6) = 50, the market gets all of it, the house half.
    const Bids b = bids(p, 7.0, 1.0);
    assert(near(b.market, 16.0 + 50.0));
    assert(near(b.house, 32.0 + 25.0));
    assert(near(b.farm, 15.8));  // farms get nothing from the river
    assert(useAt(p, 7.0, 1.0) == Use::Market);
    assert(useAt(p, 7.0, kNoWater) == Use::House);  // same distance, no river: house
    p.water = 0.0;
    assert(useAt(p, 7.0, 1.0) == Use::House);  // slider at zero: no effect
    p.water = 1.0;
    const Bids edge = bids(p, 7.0, kWaterReach);  // the reach ends exactly here
    assert(near(edge.market, 16.0) && near(edge.house, 32.0));
    const Bids far = bids(p, 7.0, 50.0);
    assert(near(far.market, 16.0) && near(far.house, 32.0));
}

void terrainDistances() {
    // 5 x 1, water in column 0.
    Terrain line = Terrain::fromMask(5, 1, {1, 0, 0, 0, 0});
    for (int x = 0; x < 5; ++x) assert(near(line.waterDist[static_cast<std::size_t>(x)], static_cast<double>(x)));
    assert(line.isWater(0, 0) && !line.isWater(1, 0));
    // 3 x 3, water in the top left corner: (2,2) is sqrt(8) away, (1,0) is 1.
    Terrain corner = Terrain::fromMask(3, 3, {1, 0, 0, 0, 0, 0, 0, 0, 0});
    assert(near(corner.waterDist[2 * 3 + 2], std::sqrt(8.0)));
    assert(near(corner.waterDist[1], 1.0));
    // No water at all: every distance is "none".
    Terrain dry = Terrain::fromMask(2, 2, {0, 0, 0, 0});
    for (double d : dry.waterDist) assert(d >= kNoWater);
    // The generated river puts water in every row and keeps the map's edges dry enough to be farmland.
    const Terrain river = Terrain::river(48, 31);
    for (int y = 0; y < 31; ++y) {
        int n = 0;
        for (int x = 0; x < 48; ++x) n += river.isWater(x, y) ? 1 : 0;
        assert(n >= 1 && n <= 3);
    }
    assert(!river.isWater(0, 15) && !river.isWater(47, 15));
}

void classifyACorridor() {
    // 14 x 1 map, no water, centre at column 0: distances 0..13.
    Terrain t = Terrain::fromMask(14, 1, std::vector<unsigned char>(14, 0));
    const Params p = defaults();
    auto uses = classify(t, p, 0, 0, Mode::BidCurves);
    auto n = count(uses);
    assert(n[idx(Use::Market)] == 6 && n[idx(Use::House)] == 6 && n[idx(Use::Farm)] == 2);
    assert(n[idx(Use::Wild)] == 0 && n[idx(Use::Water)] == 0);
    // Centre at the other end: the mirror image.
    uses = classify(t, p, 13, 0, Mode::BidCurves);
    n = count(uses);
    assert(n[idx(Use::Market)] == 6 && n[idx(Use::House)] == 6 && n[idx(Use::Farm)] == 2);
    assert(uses[13] == Use::Market && uses[0] == Use::Farm);
    // Water in column 3 is water, whatever the curves say.
    Terrain wet = Terrain::fromMask(14, 1, {0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
    Params dry = p;
    dry.water = 0.0;
    uses = classify(wet, dry, 0, 0, Mode::BidCurves);
    assert(uses[3] == Use::Water);
    n = count(uses);
    assert(n[idx(Use::Water)] == 1 && n[idx(Use::Market)] == 5);
}

void handSetRings() {
    assert(useFixed(1.0, 4.9) == Use::Market);
    assert(useFixed(1.0, 5.0) == Use::House);  // radius is exclusive
    assert(useFixed(1.0, 11.9) == Use::House);
    assert(useFixed(1.0, 12.0) == Use::Farm);
    assert(useFixed(1.0, 400.0) == Use::Farm);  // no wild ground out there
    assert(useFixed(2.0, 9.9) == Use::Market);  // the rings scale with the size
    assert(useFixed(2.0, 23.9) == Use::House);
}

void handSetRingsMissWhatTheCurvesSee() {
    Params p = defaults();
    p.size = 0.5;
    // d = 4: the curves say farm (house 14 against farm 17.6), the hand-set rings still draw a house.
    assert(useAt(p, 4.0, kNoWater) == Use::Farm);
    assert(useFixed(0.5, 4.0) == Use::House);
    p.size = 1.5;
    // d = 19: house 14 against farm 8.6 for the curves, the fixed outer ring ends at 18.
    assert(useAt(p, 19.0, kNoWater) == Use::House);
    assert(useFixed(1.5, 19.0) == Use::Farm);
    // The fixed rings do not know about water.
    Terrain t = Terrain::fromMask(14, 1, std::vector<unsigned char>(14, 0));
    p = defaults();
    p.water = 1.0;
    const auto uses = classify(t, p, 0, 0, Mode::HandSetRings);
    assert(count(uses)[idx(Use::Market)] == 5);  // d = 0..4, the radius is exclusive
}

void sliderMapping() {
    assert(near(valueAtX(100.0, 100.0, 200.0, 0.0, 10.0), 0.0));
    assert(near(valueAtX(300.0, 100.0, 200.0, 0.0, 10.0), 10.0));
    assert(near(valueAtX(200.0, 100.0, 200.0, 0.0, 10.0), 5.0));
    assert(near(valueAtX(-50.0, 100.0, 200.0, 2.0, 4.0), 2.0));  // clamped left
    assert(near(valueAtX(900.0, 100.0, 200.0, 2.0, 4.0), 4.0));  // clamped right
    assert(near(valueAtX(150.0, 100.0, 0.0, 2.0, 4.0), 2.0));  // zero width track
    assert(near(xAtValue(5.0, 100.0, 200.0, 0.0, 10.0), 200.0));
    assert(near(xAtValue(99.0, 100.0, 200.0, 0.0, 10.0), 300.0));  // clamped
    assert(near(xAtValue(1.0, 100.0, 200.0, 3.0, 3.0), 100.0));  // lo == hi
}

void growBouncesInsideItsRange() {
    double size = 1.4;
    double dir = 1.0;
    advanceSize(size, dir, 0.5, 0.1, 0.5, 1.5);  // +0.05
    assert(near(size, 1.45) && dir == 1.0);
    advanceSize(size, dir, 2.0, 0.1, 0.5, 1.5);  // would be 1.65: clamp and turn around
    assert(near(size, 1.5) && dir == -1.0);
    advanceSize(size, dir, 1.0, 0.1, 0.5, 1.5);
    assert(near(size, 1.4));
    advanceSize(size, dir, 0.0, 0.1, 0.5, 1.5);  // no time, no change
    assert(near(size, 1.4));
    size = 0.52;
    dir = -1.0;
    advanceSize(size, dir, 100.0, 0.1, 0.5, 1.5);  // a huge step never leaves the range
    assert(near(size, 0.5) && dir == 1.0);
}

void cellUnderThePointer() {
    // Map at (16, 40), 12 px cells, 48 x 31.
    auto c = cellAt(16.0, 40.0, 16.0, 40.0, 12.0, 48, 31);
    assert(c && c->x == 0 && c->y == 0);  // the top left corner belongs to the first cell
    c = cellAt(27.9, 51.9, 16.0, 40.0, 12.0, 48, 31);
    assert(c && c->x == 0 && c->y == 0);
    c = cellAt(28.0, 52.0, 16.0, 40.0, 12.0, 48, 31);
    assert(c && c->x == 1 && c->y == 1);
    c = cellAt(16.0 + 48 * 12.0 - 0.1, 40.0 + 31 * 12.0 - 0.1, 16.0, 40.0, 12.0, 48, 31);
    assert(c && c->x == 47 && c->y == 30);
    assert(!cellAt(16.0 + 48 * 12.0, 100.0, 16.0, 40.0, 12.0, 48, 31));  // right edge is outside
    assert(!cellAt(15.9, 100.0, 16.0, 40.0, 12.0, 48, 31));
    assert(!cellAt(100.0, 39.9, 16.0, 40.0, 12.0, 48, 31));
    assert(!cellAt(100.0, 100.0, 16.0, 40.0, 0.0, 48, 31));  // a zero cell size finds nothing
}

}  // namespace

int main() {
    bidsOnTheDefaultCurves();
    ringsAlongARay();
    tieGoesToTheDenserUse();
    nothingAboveZeroIsWild();
    boundariesOfTheDefaults();
    growthPushesTheRingsOut();
    aSmallTownHasNoHouses();
    slopesDecideTheOrder();
    waterLiftsTheBank();
    terrainDistances();
    classifyACorridor();
    handSetRings();
    handSetRingsMissWhatTheCurvesSee();
    sliderMapping();
    growBouncesInsideItsRange();
    cellUnderThePointer();
    std::puts("bid-rent-town logic: all tests passed");
    return 0;
}
