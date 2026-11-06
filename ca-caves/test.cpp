// Tests for the cave logic. Plain asserts on hand-computed cases, no framework, no SDL.
// Maps are written as text rows: '#' is a wall, '.' is floor. (0,0) is the top left cell.
// `test --survey` additionally prints a table over many seeds (numbers for the write-up, not part of ctest).
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "logic.hpp"

using namespace ca;

namespace {

Grid parse(const std::vector<std::string>& rows) {
    Grid g(static_cast<int>(rows[0].size()), static_cast<int>(rows.size()));
    for (int y = 0; y < g.h; ++y) {
        for (int x = 0; x < g.w; ++x) g.set(x, y, rows[static_cast<std::size_t>(y)][static_cast<std::size_t>(x)] == '#');
    }
    return g;
}

std::vector<std::string> dump(const Grid& g) {
    std::vector<std::string> rows;
    for (int y = 0; y < g.h; ++y) {
        std::string row;
        for (int x = 0; x < g.w; ++x) row += g.isWall(x, y) ? '#' : '.';
        rows.push_back(row);
    }
    return rows;
}

Grid mirrored(const Grid& g) {
    Grid out(g.w, g.h);
    for (int y = 0; y < g.h; ++y) {
        for (int x = 0; x < g.w; ++x) out.set(g.w - 1 - x, y, g.isWall(x, y));
    }
    return out;
}

void rngIsReproducible() {
    // splitmix64 with seed 0 starts with these two values (reference test vector of the algorithm).
    Rng a(0);
    assert(a.next() == 0xE220A8397B1DCDAFULL);
    assert(a.next() == 0x6E789E6AA1B965F4ULL);
    Rng b(42);
    Rng c(42);
    Rng d(43);
    bool differs = false;
    for (int i = 0; i < 20; ++i) {
        const std::uint64_t v = b.next();
        assert(v == c.next());
        if (v != d.next()) differs = true;
    }
    assert(differs);
}

void unitStaysInRange() {
    Rng r(7);
    double sum = 0.0;
    constexpr int kDraws = 100000;
    for (int i = 0; i < kDraws; ++i) {
        const double u = r.unit();
        assert(u >= 0.0 && u < 1.0);
        sum += u;
    }
    assert(std::fabs(sum / kDraws - 0.5) < 0.01);
}

void noiseThreshold() {
    // A cell is a wall if its noise is strictly below the density.
    const std::vector<double> noise{0.1, 0.5, 0.9, 0.5};
    const Grid g = fromNoise(2, 2, noise, 0.5);
    assert(g.isWall(0, 0));
    assert(!g.isWall(1, 0));  // exactly 0.5 is not below 0.5
    assert(!g.isWall(0, 1));
    assert(!g.isWall(1, 1));
    assert(fromNoise(2, 2, noise, 0.0).wallCount() == 0);
    assert(fromNoise(2, 2, noise, 1.0).wallCount() == 4);
    // The same noise at a higher density only ever adds walls.
    assert(fromNoise(2, 2, noise, 0.95).wallCount() == 4);
    assert(fromNoise(2, 2, noise, 0.6).wallCount() == 3);
}

void noiseSize() {
    assert(makeNoise(5, 4, 1).size() == 20);
    assert(makeNoise(5, 4, 1) == makeNoise(5, 4, 1));
    assert(makeNoise(5, 4, 1) != makeNoise(5, 4, 2));
}

void outsideCountsAsWall() {
    const Grid g = parse({"...", "...", "..."});
    assert(!g.isWall(1, 1));
    assert(g.isWall(-1, 1));
    assert(g.isWall(1, -1));
    assert(g.isWall(3, 1));
    assert(g.isWall(1, 3));
    assert(wallNeighbors(g, 1, 1) == 0);  // the middle sees only floor
    assert(wallNeighbors(g, 0, 0) == 5);  // a corner: 5 of its 8 neighbors lie outside
    assert(wallNeighbors(g, 1, 0) == 3);  // an edge cell: 3 outside
    const Grid solid = parse({"###", "###", "###"});
    assert(wallNeighbors(solid, 1, 1) == 8);
    assert(wallNeighbors(solid, 0, 0) == 8);
    // One pass on the empty map: only the corners reach 5 wall neighbors.
    Grid empty = g;
    stepBuffered(empty);
    assert((dump(empty) == std::vector<std::string>{"#.#", "...", "#.#"}));
}

void theFourFiveRule() {
    // A wall survives with 4 or more wall neighbors, a floor cell turns into a wall with 5 or more.
    assert(!nextIsWall(true, 3));
    assert(nextIsWall(true, 4));
    assert(!nextIsWall(false, 4));
    assert(nextIsWall(false, 5));
    assert(!nextIsWall(true, 0));
    assert(nextIsWall(true, 8));
    assert(nextIsWall(false, 8));
    // Same thing as "at least 5 walls in the 3x3 block, the cell itself included".
    for (int n = 0; n <= 8; ++n) {
        assert(nextIsWall(true, n) == (n + 1 >= 5));
        assert(nextIsWall(false, n) == (n >= 5));
    }
}

void loneWallVanishes() {
    Grid g = parse({".......", ".......", ".......", "...#...", ".......", ".......", "......."});
    const int changed = stepBuffered(g);
    // The wall in the middle has 0 neighbors and goes. The four map corners have 5 outside neighbors and become walls.
    assert(g.wallCount() == 4);
    assert(!g.isWall(3, 3));
    assert(g.isWall(0, 0) && g.isWall(6, 0) && g.isWall(0, 6) && g.isWall(6, 6));
    assert(changed == 5);
}

void blockErodesFromTheCorners() {
    Grid g = parse({".........", ".........", ".........", "...###...", "...###...", "...###...", ".........", ".........", "........."});
    // Pass 1: block corners have 3 wall neighbors (< 4) and go, edge cells have 5 and stay, the center has 8.
    // The four map corners turn into walls (5 outside neighbors).
    assert(stepBuffered(g) == 8);
    assert(g.wallCount() == 9);
    assert(!g.isWall(3, 3) && !g.isWall(5, 3) && !g.isWall(3, 5) && !g.isWall(5, 5));
    assert(g.isWall(4, 3) && g.isWall(3, 4) && g.isWall(4, 4) && g.isWall(5, 4) && g.isWall(4, 5));
    // Pass 2: the arms of the plus have 3 wall neighbors and go, the middle keeps its 4 arms.
    assert(stepBuffered(g) == 4);
    assert(g.wallCount() == 5);
    assert(g.isWall(4, 4));
    // Pass 3: the last cell has no neighbors left.
    assert(stepBuffered(g) == 1);
    assert(g.wallCount() == 4);
    assert(!g.isWall(4, 4));
    // Pass 4: nothing changes anymore (the map corners keep their 5 outside neighbors).
    assert(stepBuffered(g) == 0);
    assert(g.wallCount() == 4);
}

void straightWallsStayPut() {
    const std::vector<std::string> rows{"####....", "####....", "####....", "####....", "####...."};
    Grid g = parse(rows);
    stepBuffered(g);
    const auto after = dump(g);
    // Away from the map border (rows 1 to 3, columns 1 to 6) a straight edge is a fixed point of the rule:
    // the last wall column has 5 wall neighbors, the first floor column has 3.
    for (int y = 1; y <= 3; ++y) {
        for (int x = 1; x <= 6; ++x) assert(after[static_cast<std::size_t>(y)][static_cast<std::size_t>(x)] == rows[static_cast<std::size_t>(y)][static_cast<std::size_t>(x)]);
    }
}

void bufferedAndInPlaceDiffer() {
    // One wall at (2,3) in an otherwise empty 4x4 map.
    const std::vector<std::string> start{"....", "....", "....", "..#."};
    // Buffered: every cell looks at the OLD map. Only the four corners (5 outside neighbors) become walls,
    // the wall itself has 3 outside neighbors and goes.
    Grid b = parse(start);
    stepBuffered(b);
    assert((dump(b) == std::vector<std::string>{"#..#", "....", "....", "#..#"}));
    // In place, scanning row by row: (0,3) becomes a wall first. Then (1,3) already counts that new wall plus the old wall at (2,3)
    // and its 3 outside neighbors: 5, so it becomes a wall too. (2,3) sees the new wall at (1,3) and stays. The row fills up.
    Grid p = parse(start);
    stepInPlace(p);
    assert((dump(p) == std::vector<std::string>{"#..#", "....", "....", "####"}));
}

void bufferedIsMirrorSymmetric() {
    // Buffered passes do not care in which order the cells are visited, so mirroring before or after gives the same map.
    // The in-place pass visits left to right, so mirroring changes the result.
    int inPlaceBroken = 0;
    for (std::uint64_t seed = 1; seed <= 40; ++seed) {
        const Grid g = fromNoise(20, 12, makeNoise(20, 12, seed), 0.45);
        Grid a = mirrored(g);
        stepBuffered(a);
        Grid b = g;
        stepBuffered(b);
        assert(dump(a) == dump(mirrored(b)));
        Grid c = mirrored(g);
        stepInPlace(c);
        Grid d = g;
        stepInPlace(d);
        if (dump(c) != dump(mirrored(d))) ++inPlaceBroken;
    }
    assert(inPlaceBroken > 0);
}

void regionsAreFourConnected() {
    const Grid g = parse({"..#..", "..#..", "#####", "....."});
    const Regions r = findRegions(g);
    assert(r.size.size() == 3);
    assert(r.label[0] == 0 && r.label[1] == 0);               // top left block
    assert(r.label[3] == 1 && r.label[4] == 1);               // top right block
    assert(r.label[2] == -1);                                  // wall
    assert(r.label[15] == 2 && r.label[19] == 2);              // bottom row
    assert(r.size[0] == 4 && r.size[1] == 4 && r.size[2] == 5);
    assert(r.largest == 2);
    // Two floor cells that only touch at a corner are two regions: a player cannot squeeze between the walls.
    const Regions d = findRegions(parse({".#", "#."}));
    assert(d.size.size() == 2);
    assert(d.label[0] != d.label[3]);
}

void regionsOfNothing() {
    const Regions solid = findRegions(parse({"##", "##"}));
    assert(solid.size.empty());
    assert(solid.largest == -1);
    const Regions open = findRegions(parse({"...", "..."}));
    assert(open.size.size() == 1 && open.size[0] == 6 && open.largest == 0);
}

void floodDistancesGoAround() {
    const Grid g = parse({".....", ".###.", "....."});
    const std::vector<int> d = floodDistances(g, 0, 0);
    auto at = [&](int x, int y) { return d[static_cast<std::size_t>(y * g.w + x)]; };
    assert(at(0, 0) == 0);
    assert(at(4, 0) == 4);
    assert(at(0, 2) == 2);
    assert(at(2, 2) == 4);
    assert(at(4, 2) == 6);   // 4 along the top and 2 down, or 2 down and 4 along the bottom
    assert(at(2, 1) == -1);  // wall
    // Starting on a wall or outside the map reaches nothing.
    for (int v : floodDistances(g, 2, 1)) assert(v == -1);
    for (int v : floodDistances(g, -1, 0)) assert(v == -1);
    for (int v : floodDistances(g, 0, 9)) assert(v == -1);
}

void floodStopsAtWalls() {
    const Grid g = parse({"..#..", "..#..", "#####", "....."});
    const std::vector<int> d = floodDistances(g, 0, 0);
    int reached = 0;
    for (int v : d) {
        if (v >= 0) ++reached;
    }
    assert(reached == 4);
    assert(d[3] == -1 && d[15] == -1);
}

void keepLargestFillsTheRest() {
    Grid g = parse({"..#..", "..#..", "#####", "....."});
    assert(keepLargestRegion(g) == 8);
    assert((dump(g) == std::vector<std::string>{"#####", "#####", "#####", "....."}));
    assert(keepLargestRegion(g) == 0);  // a second call has nothing left to remove
    Grid solid = parse({"##", "##"});
    assert(keepLargestRegion(solid) == 0);
    // On a tie the first region (scan order) wins.
    Grid tie = parse({".#."});
    assert(keepLargestRegion(tie) == 1);
    assert((dump(tie) == std::vector<std::string>{".##"}));
}

void measureCounts() {
    const Grid g = parse({"..#..", "..#..", "#####", "....."});
    const Stats s = measure(g);
    assert(s.walls == 7 && s.floor == 13);
    assert(s.walls + s.floor == g.w * g.h);
    assert(s.regions == 3);
    assert(s.largest == 5);
}

void generatePipeline() {
    const std::vector<double> noise = makeNoise(30, 20, 5);
    // No passes: the pure random fill.
    const Result zero = generate(30, 20, noise, 0.45, 0, Update::Buffered, false);
    assert(dump(zero.grid) == dump(fromNoise(30, 20, noise, 0.45)));
    assert(!zero.stable);
    assert(zero.changedPasses == 0);
    // Passes are the same as stepping by hand.
    Grid by = fromNoise(30, 20, noise, 0.45);
    for (int i = 0; i < 3; ++i) stepBuffered(by);
    assert(dump(generate(30, 20, noise, 0.45, 3, Update::Buffered, false).grid) == dump(by));
    Grid ip = fromNoise(30, 20, noise, 0.45);
    for (int i = 0; i < 3; ++i) stepInPlace(ip);
    assert(dump(generate(30, 20, noise, 0.45, 3, Update::InPlace, false).grid) == dump(ip));
    // Keep largest leaves at most one region.
    const Result kept = generate(30, 20, noise, 0.45, 4, Update::Buffered, true);
    assert(measure(kept.grid).regions <= 1);
    // Far more passes than needed: the map is stable and does not change anymore.
    const Result late = generate(30, 20, noise, 0.45, 60, Update::Buffered, false);
    assert(late.stable);
    Grid walk = fromNoise(30, 20, noise, 0.45);
    int walked = 0;
    while (stepBuffered(walk) != 0) ++walked;
    assert(late.changedPasses == walked);
    assert(walked > 0 && walked < 60);
    assert(dump(late.grid) == dump(walk));
    // Stable is reported on the last pass that changed something, not one pass later.
    const Result exact = generate(30, 20, noise, 0.45, walked, Update::Buffered, false);
    assert(exact.stable && exact.changedPasses == walked);
    const Result early = generate(30, 20, noise, 0.45, walked - 1, Update::Buffered, false);
    assert(!early.stable && early.changedPasses == walked - 1);
    assert(dump(late.grid) == dump(generate(30, 20, noise, 0.45, 61, Update::Buffered, false).grid));
}

void noiseTurnsIntoCaves() {
    // Pure noise has a lot of tiny pockets. A few passes merge them: fewer regions, and the largest one holds most of the floor.
    int fewer = 0;
    for (std::uint64_t seed = 1; seed <= 20; ++seed) {
        const std::vector<double> noise = makeNoise(150, 62, seed);
        const Stats before = measure(generate(150, 62, noise, 0.45, 0, Update::Buffered, false).grid);
        const Stats after = measure(generate(150, 62, noise, 0.45, 5, Update::Buffered, false).grid);
        if (after.regions < before.regions) ++fewer;
        assert(after.regions * 4 < before.regions);  // not a single map keeps more than a quarter of its pockets
    }
    assert(fewer == 20);
}

void survey() {
    constexpr int kW = 150;
    constexpr int kH = 62;
    constexpr int kSeeds = 200;
    std::puts("density  walls%  regions  largest%-of-floor  maps-with-1-region%  passes-to-stable  maps-with-seal-off%");
    for (const int step : {38, 40, 42, 44, 45, 46, 48, 50, 52, 54, 56, 60}) {
        const double density = step / 100.0;
        double walls = 0.0;
        double regions = 0.0;
        double share = 0.0;
        int single = 0;
        double passes = 0.0;
        int sealed = 0;
        for (std::uint64_t seed = 1; seed <= kSeeds; ++seed) {
            const std::vector<double> noise = makeNoise(kW, kH, seed);
            Grid g = fromNoise(kW, kH, noise, density);
            int n = 0;
            while (n < 100 && stepBuffered(g) != 0) ++n;
            passes += n;
            const Stats s = measure(g);
            walls += 100.0 * s.walls / (kW * kH);
            regions += s.regions;
            share += s.floor > 0 ? 100.0 * s.largest / s.floor : 0.0;
            if (s.regions == 1) ++single;
            if (s.regions > 1) ++sealed;
        }
        std::printf("%.2f     %5.1f   %6.1f   %6.1f             %5.1f               %5.1f             %5.1f\n", density, walls / kSeeds, regions / kSeeds, share / kSeeds, 100.0 * single / kSeeds, passes / kSeeds,
                    100.0 * sealed / kSeeds);
    }
    std::puts("\ndensity  pass:regions");
    for (const int step : {40, 45, 50, 55}) {
        const double density = step / 100.0;
        std::printf("%.2f    ", density);
        for (int pass = 0; pass <= 8; ++pass) {
            double regions = 0.0;
            for (std::uint64_t seed = 1; seed <= kSeeds; ++seed) {
                regions += measure(generate(kW, kH, makeNoise(kW, kH, seed), density, pass, Update::Buffered, false).grid).regions;
            }
            std::printf(" %d:%.1f", pass, regions / kSeeds);
        }
        std::puts("");
    }
    std::puts("\nbuffered vs in place at 0.45, 5 passes (avg over seeds): walls% / regions");
    for (const Update mode : {Update::Buffered, Update::InPlace}) {
        double walls = 0.0;
        double regions = 0.0;
        for (std::uint64_t seed = 1; seed <= kSeeds; ++seed) {
            const Stats s = measure(generate(kW, kH, makeNoise(kW, kH, seed), 0.45, 5, mode, false).grid);
            walls += 100.0 * s.walls / (kW * kH);
            regions += s.regions;
        }
        std::printf("%s: %.1f / %.1f\n", mode == Update::Buffered ? "buffered" : "in place", walls / kSeeds, regions / kSeeds);
    }
    std::puts("\nseeds with a good first picture at 0.45, 4 passes (regions, largest share):");
    for (std::uint64_t seed = 1; seed <= 60; ++seed) {
        const Stats s = measure(generate(kW, kH, makeNoise(kW, kH, seed), 0.45, 4, Update::Buffered, false).grid);
        std::printf("seed %2llu: floor %d regions %d largest %d\n", static_cast<unsigned long long>(seed), s.floor, s.regions, s.largest);
    }
}

}  // namespace

int main(int argc, char** argv) {
    rngIsReproducible();
    unitStaysInRange();
    noiseThreshold();
    noiseSize();
    outsideCountsAsWall();
    theFourFiveRule();
    loneWallVanishes();
    blockErodesFromTheCorners();
    straightWallsStayPut();
    bufferedAndInPlaceDiffer();
    bufferedIsMirrorSymmetric();
    regionsAreFourConnected();
    regionsOfNothing();
    floodDistancesGoAround();
    floodStopsAtWalls();
    keepLargestFillsTheRest();
    measureCounts();
    generatePipeline();
    noiseTurnsIntoCaves();
    if (argc > 1 && std::strcmp(argv[1], "--survey") == 0) survey();
    std::puts("ca-caves logic: all asserts passed");
    return 0;
}
