// Tests for the falling-sand water logic. Plain asserts on hand-computed cases, no framework, no SDL.
// Coordinates: x to the right, y DOWN (row 0 is the top), like the screen.
#undef NDEBUG
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <utility>
#include <vector>

#include "logic.hpp"

using namespace pw;

namespace {

void wall(World& w, int x0, int y0, int x1, int y1) {
    for (int y = y0; y <= y1; ++y)
        for (int x = x0; x <= x1; ++x) w.set(x, y, Kind::Wall);
}

void fill(World& w, int x0, int y0, int x1, int y1, Kind k) {
    for (int y = y0; y <= y1; ++y)
        for (int x = x0; x <= x1; ++x) w.set(x, y, k);
}

void steps(World& w, int n) {
    for (int i = 0; i < n; ++i) w.step();
}

std::vector<std::pair<int, int>> where(const World& w, Kind k) {
    std::vector<std::pair<int, int>> out;
    for (int y = 0; y < w.height(); ++y)
        for (int x = 0; x < w.width(); ++x)
            if (w.kind(x, y) == k) out.emplace_back(x, y);
    return out;
}

int column(const World& w, int x, Kind k) {
    int n = 0;
    for (int y = 0; y < w.height(); ++y) n += w.kind(x, y) == k ? 1 : 0;
    return n;
}

bool sameWorld(const World& a, const World& b) {
    for (int y = 0; y < a.height(); ++y)
        for (int x = 0; x < a.width(); ++x)
            if (a.cell(x, y).kind != b.cell(x, y).kind || a.cell(x, y).tint != b.cell(x, y).tint) return false;
    return true;
}

// Small deterministic generator for the scattered-world tests.
struct Lcg {
    std::uint32_t s;
    std::uint32_t next() { return s = s * 1664525U + 1013904223U; }
    int below(int n) { return static_cast<int>((next() >> 8) % static_cast<std::uint32_t>(n)); }
};

/// A 14 wide basin with walls left, right and below. The inside is x 1..12, the floor row is the last one.
World basin(int height) {
    World w(14, height);
    wall(w, 0, 0, 0, height - 1);
    wall(w, 13, 0, 13, height - 1);
    wall(w, 0, height - 1, 13, height - 1);
    return w;
}

bool levelAt(const World& w, int height) {
    for (int x = 1; x <= 12; ++x)
        if (column(w, x, Kind::Water) != height) return false;
    return true;
}

int stepsUntilLevel(World& w, int height, int limit) {
    for (int i = 0; i < limit; ++i) {
        if (levelAt(w, height)) return i;
        w.step();
    }
    return levelAt(w, height) ? limit : -1;
}

void fallsOneRowPerStep() {
    World w(11, 20);
    w.set(5, 0, Kind::Water);
    w.step();
    assert(w.kind(5, 1) == Kind::Water && w.kind(5, 0) == Kind::Empty);  // exactly one row, not the whole column
    steps(w, 4);
    assert(w.kind(5, 5) == Kind::Water);
    steps(w, 40);
    assert(where(w, Kind::Water).size() == 1);  // the bottom edge acts as a floor, the cell may slide along it
    assert(where(w, Kind::Water)[0].second == 19);
}

void sandFallsOneRowPerStep() {
    World w(11, 20);
    w.set(5, 0, Kind::Sand);
    w.step();
    assert(w.kind(5, 1) == Kind::Sand);
    steps(w, 3);
    assert(w.kind(5, 4) == Kind::Sand);
}

void aFallingColumnMovesAsOneTrain() {
    // Scanning bottom to top lets the whole column advance one row per step. A top to bottom scan would pick the
    // top cell up again after moving it, or leave gaps in the column.
    World w(11, 20);
    fill(w, 5, 0, 5, 2, Kind::Water);
    w.step();
    assert(w.kind(5, 0) == Kind::Empty);
    assert(w.kind(5, 1) == Kind::Water && w.kind(5, 2) == Kind::Water && w.kind(5, 3) == Kind::Water);
}

void restsOnWallAndFloor() {
    World w(11, 10);
    wall(w, 0, 9, 10, 9);
    wall(w, 4, 8, 4, 8);
    wall(w, 6, 8, 6, 8);
    w.set(5, 8, Kind::Water);  // in a one cell slot
    steps(w, 100);
    assert(w.kind(5, 8) == Kind::Water);

    World s(11, 10);
    wall(s, 0, 9, 10, 9);
    s.set(5, 8, Kind::Sand);  // sand on a flat floor with air beside it: diagonals are walls, so it stays
    steps(s, 100);
    assert(s.kind(5, 8) == Kind::Sand);

    World o(21, 10);
    wall(o, 0, 9, 20, 9);
    o.set(10, 8, Kind::Water);  // open floor: it may wander sideways but never leaves the floor row
    steps(o, 50);
    assert(where(o, Kind::Water).size() == 1);
    assert(where(o, Kind::Water)[0].second == 8);
}

void sandSlidesDownASlope() {
    // A single sand cell on top of a one cell pillar slides off diagonally and ends on the floor next to it.
    World w(11, 10);
    wall(w, 0, 9, 10, 9);
    wall(w, 5, 8, 5, 8);
    w.set(5, 7, Kind::Sand);
    steps(w, 10);
    const auto sand = where(w, Kind::Sand);
    assert(sand.size() == 1);
    assert(sand[0].second == 8 && (sand[0].first == 4 || sand[0].first == 6));
}

void waterLevelsOutInAFlatBasin() {
    World w = basin(30);
    fill(w, 1, 23, 4, 28, Kind::Water);  // 4 x 6 = 24 cells piled on the left, the flat level is 2 rows
    const int n = stepsUntilLevel(w, 2, 3000);
    assert(n >= 0);
    assert(w.counts().water == 24);
    std::printf("basin levelled after %d steps\n", n);
}

void flowRangeSpeedsUpLeveling() {
    World fast = basin(30);
    World slow = basin(30);
    fast.spread = 4;
    slow.spread = 1;
    fill(fast, 1, 23, 4, 28, Kind::Water);
    fill(slow, 1, 23, 4, 28, Kind::Water);
    const int a = stepsUntilLevel(fast, 2, 6000);
    const int b = stepsUntilLevel(slow, 2, 6000);
    std::printf("levelled after %d steps with range 4, %d with range 1 (-1: not within 6000)\n", a, b);
    assert(a >= 0);
    assert(b == -1 || b > a);
}

void everyParticleSurvives() {
    World w(60, 40);
    Lcg rng{12345};
    wall(w, 0, 39, 59, 39);
    for (int i = 0; i < 40; ++i) w.set(rng.below(60), 10 + rng.below(28), Kind::Wall);
    for (int i = 0; i < 400; ++i) w.set(rng.below(60), rng.below(30), Kind::Water);
    for (int i = 0; i < 300; ++i) w.set(rng.below(60), rng.below(30), Kind::Sand);
    const Counts before = w.counts();
    for (int round = 0; round < 40; ++round) {
        steps(w, 25);  // 1000 steps in total
        const Counts now = w.counts();
        assert(now.water == before.water);
        assert(now.sand == before.sand);
        assert(now.wall == before.wall);
        assert(now.empty == before.empty);
    }
    assert(before.water > 0 && before.sand > 0);
}

void sandSinksThroughWater() {
    World w = basin(24);
    fill(w, 1, 14, 12, 22, Kind::Water);  // 12 x 9 pool
    fill(w, 3, 8, 8, 13, Kind::Sand);     // sand block hovering above it, falls in
    const Counts before = w.counts();
    steps(w, 400);
    assert(w.counts().water == before.water && w.counts().sand == before.sand);
    // In every column all water lies above all sand.
    for (int x = 1; x <= 12; ++x) {
        int lowestWater = -1;
        int highestSand = 1000;
        for (int y = 0; y < w.height(); ++y) {
            if (w.kind(x, y) == Kind::Water) lowestWater = y;
            if (w.kind(x, y) == Kind::Sand) highestSand = std::min(highestSand, y);
        }
        assert(lowestWater < highestSand);
    }
    // And nothing floats: no sand cell has water directly below it.
    for (const auto& [x, y] : where(w, Kind::Sand)) assert(w.kind(x, y + 1) != Kind::Water);
}

void edgesAreSolid() {
    World w(6, 6);
    assert(w.kind(-1, 0) == Kind::Wall && w.kind(0, -1) == Kind::Wall);
    assert(w.kind(6, 0) == Kind::Wall && w.kind(0, 6) == Kind::Wall);
    w.set(-1, 3, Kind::Water);
    w.set(6, 3, Kind::Water);
    w.set(3, 9, Kind::Sand);
    assert(w.counts().water == 0 && w.counts().sand == 0);

    // A completely mixed, completely full world and an almost full one: no write outside, nothing lost.
    fill(w, 0, 0, 5, 5, Kind::Water);
    fill(w, 0, 0, 5, 1, Kind::Sand);
    w.set(2, 5, Kind::Empty);
    const Counts before = w.counts();
    steps(w, 200);
    assert(w.counts().water == before.water && w.counts().sand == before.sand);

    // Water and sand against the left and right edge of a wide world.
    World e(8, 12);
    fill(e, 0, 0, 0, 5, Kind::Water);
    fill(e, 7, 0, 7, 5, Kind::Sand);
    steps(e, 300);
    assert(e.counts().water == 6 && e.counts().sand == 6);
}

void sameStartSameResult() {
    auto build = [] {
        World w(40, 30);
        wall(w, 0, 29, 39, 29);
        wall(w, 10, 20, 30, 21);
        fill(w, 12, 2, 20, 10, Kind::Water);
        fill(w, 22, 2, 28, 8, Kind::Sand);
        return w;
    };
    World a = build();
    World b = build();
    for (int i = 0; i < 500; ++i) {
        a.step();
        b.step();
    }
    assert(sameWorld(a, b));
}

void sandPileDoesNotDrift() {
    // 200 sand cells fall as a column onto a wide floor. The scan and the diagonal choice alternate, so the pile
    // does not lean to one side: its centre of mass stays within one cell of the drop column.
    World w(101, 60);
    wall(w, 0, 59, 100, 59);
    fill(w, 50, 0, 50, 49, Kind::Sand);
    fill(w, 51, 0, 51, 49, Kind::Sand);
    fill(w, 49, 0, 49, 49, Kind::Sand);
    fill(w, 52, 0, 52, 49, Kind::Sand);
    steps(w, 1200);
    const auto sand = where(w, Kind::Sand);
    assert(sand.size() == 200);
    double sum = 0;
    int lo = 1000;
    int hi = -1;
    for (const auto& [x, y] : sand) {
        (void)y;
        sum += x;
        lo = std::min(lo, x);
        hi = std::max(hi, x);
    }
    const double centre = sum / static_cast<double>(sand.size());
    std::printf("sand pile centre %.2f, columns %d..%d\n", centre, lo, hi);
    assert(centre > 49.5 && centre < 51.5);  // dropped around x = 50.5
    assert(hi - lo >= 10);                    // it did spread
}

void brushCounts() {
    World w(40, 40);
    assert(paintDisc(w, 20, 20, 1, Kind::Water, false) == 5);   // plus shape
    assert(w.counts().water == 5);
    assert(paintDisc(w, 20, 20, 1, Kind::Water, false) == 0);   // water only goes into empty cells
    World s(40, 40);
    assert(paintDisc(s, 20, 20, 3, Kind::Sand, false) == 29);
    assert(paintDisc(s, 20, 20, 6, Kind::Wall, false) == 113);
    assert(s.counts().wall == 113 && s.counts().sand == 0);  // a wall replaces what is there
    assert(paintDisc(s, 20, 20, 3, Kind::Empty, false) == 29);
    assert(s.counts().wall == 113 - 29);

    // Partly outside: only the inside counts, nothing crashes.
    World c(10, 10);
    assert(paintDisc(c, 0, 0, 3, Kind::Water, false) == 11);  // one quarter of the 29 cell disc, axes included

    // Thin mode writes only a part of the disc.
    World t(40, 40);
    const int n = paintDisc(t, 20, 20, 6, Kind::Water, true);
    assert(n > 0 && n < 113);
    assert(n == t.counts().water);
}

void strokeCoversTheLine() {
    World w(60, 20);
    stroke(w, 5, 10, 50, 10, 1, Kind::Wall, false);
    for (int x = 5; x <= 50; ++x) assert(w.kind(x, 10) == Kind::Wall);
    assert(w.kind(4, 10) == Kind::Empty || w.kind(4, 10) == Kind::Wall);  // the disc end cap may reach one cell further
    assert(w.kind(30, 12) == Kind::Empty);
}

void clearEmptiesTheWorld() {
    World w(10, 10);
    fill(w, 0, 0, 9, 9, Kind::Water);
    w.clear();
    assert(w.counts().empty == 100);
}

void startSceneIsBuilt() {
    World w(kCols, kRows);
    buildBasin(w);
    const Counts c = w.counts();
    assert(c.water == 0);
    assert(c.wall == 1382);  // ground 720, hood 26, shelf 255, lip 18, basin walls 210, island 153
    assert(c.sand > 800 && c.sand < 1000);
    assert(w.kind(0, kRows - 1) == Kind::Wall && w.kind(kCols - 1, kRows - 1) == Kind::Wall);  // ground
    assert(w.kind(kTapX, kTapY) == Kind::Empty);  // the tap needs room under its hood
    World tiny(10, 10);  // a grid that is too small is clipped, not a crash
    buildBasin(tiny);
    assert(tiny.counts().empty + tiny.counts().wall + tiny.counts().sand == 100);
}

void tapPoursUntilTheCap() {
    World w(kCols, kRows);
    buildBasin(w);
    const int sand = w.counts().sand;
    int maxWater = 0;
    for (int i = 0; i < 1500; ++i) {
        pourTap(w, kTapX, kTapY, 2, 500);
        w.step();
        maxWater = std::max(maxWater, w.counts().water);
    }
    assert(w.counts().water == 500);  // reached, and
    assert(maxWater == 500);          // never exceeded
    assert(w.counts().sand == sand);
}

}  // namespace

int main() {
    fallsOneRowPerStep();
    sandFallsOneRowPerStep();
    aFallingColumnMovesAsOneTrain();
    restsOnWallAndFloor();
    sandSlidesDownASlope();
    waterLevelsOutInAFlatBasin();
    flowRangeSpeedsUpLeveling();
    everyParticleSurvives();
    sandSinksThroughWater();
    edgesAreSolid();
    sameStartSameResult();
    sandPileDoesNotDrift();
    brushCounts();
    strokeCoversTheLine();
    clearEmptiesTheWorld();
    startSceneIsBuilt();
    tapPoursUntilTheCap();
    std::puts("all logic tests passed");
    return 0;
}
