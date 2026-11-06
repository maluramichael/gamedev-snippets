// Tests for the wave function collapse logic. Plain asserts on hand-computed cases, no framework, no SDL.
// Tile edges are written N, E, S, W (clockwise from the top), labels are plain numbers.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "art.hpp"
#include "logic.hpp"

using namespace wfc;

namespace {

bool near(double a, double b, double eps = 1e-9) { return std::fabs(a - b) < eps; }

TileSet makeSet(std::vector<Tile> tiles) {
    TileSet s;
    s.tiles = std::move(tiles);
    const bool ok = s.build();
    assert(ok);
    (void)ok;
    return s;
}

/// 0 grass, 1 road east-west, 2 road north-south. Edge label 1 = road.
TileSet roadSet() {
    return makeSet({{{0, 0, 0, 0}, 1.0}, {{0, 1, 0, 1}, 1.0}, {{1, 0, 1, 0}, 1.0}});
}

void compatTable() {
    const TileSet s = roadSet();
    assert(s.all == 0b111);
    // East of the grass tile (label 0): tiles whose west edge is 0 are grass (0) and the vertical road (2).
    assert(s.compat[East][0] == 0b101);
    // East of the horizontal road (label 1): only the horizontal road has a road on its west edge.
    assert(s.compat[East][1] == 0b010);
    assert(s.compat[East][2] == 0b101);
    // West is the mirror image.
    assert(s.compat[West][0] == 0b101);
    assert(s.compat[West][1] == 0b010);
    // North of the vertical road (label 1): tiles whose south edge is 1, only the vertical road.
    assert(s.compat[North][2] == 0b100);
    assert(s.compat[North][0] == 0b011);
    assert(s.compat[South][2] == 0b100);
    assert(s.compat[South][1] == 0b011);
}

void buildRejectsBadSets() {
    TileSet empty;
    assert(!empty.build());
    TileSet zero;
    zero.tiles = {{{0, 0, 0, 0}, 0.0}};
    assert(!zero.build());
    TileSet negative;
    negative.tiles = {{{0, 0, 0, 0}, -1.0}};
    assert(!negative.build());
    TileSet big;
    for (int i = 0; i < kMaxTiles + 1; ++i) big.tiles.push_back({{0, 0, 0, 0}, 1.0});
    assert(!big.build());
    TileSet full;
    for (int i = 0; i < kMaxTiles; ++i) full.tiles.push_back({{0, 0, 0, 0}, 1.0});
    assert(full.build());
    assert(full.all == ~Mask{0});
}

void entropyValues() {
    const TileSet s = roadSet();
    assert(near(entropy(s, 0), 0.0));
    assert(near(entropy(s, 0b100), 0.0));          // one tile: nothing left to decide
    assert(near(entropy(s, 0b011), std::log(2.0)));  // two equal tiles: one coin flip
    assert(near(entropy(s, 0b111), std::log(3.0)));
    // Weights 3 and 1: ln 4 - 3 ln 3 / 4 = 0.5623351446
    const TileSet w = makeSet({{{0, 0, 0, 0}, 3.0}, {{0, 0, 0, 0}, 1.0}});
    assert(near(entropy(w, 0b11), 0.5623351446, 1e-9));
    // Scaling all weights changes nothing.
    const TileSet w2 = makeSet({{{0, 0, 0, 0}, 30.0}, {{0, 0, 0, 0}, 10.0}});
    assert(near(entropy(w2, 0b11), entropy(w, 0b11)));
}

void weightedPick() {
    const TileSet w = makeSet({{{0, 0, 0, 0}, 3.0}, {{0, 0, 0, 0}, 1.0}});
    assert(pickWeighted(w, 0b11, 0.0) == 0);
    assert(pickWeighted(w, 0b11, 0.7499) == 0);
    assert(pickWeighted(w, 0b11, 0.75) == 1);  // 0.75 * 4 = 3 = the end of the first tile
    assert(pickWeighted(w, 0b11, 0.9999) == 1);
    assert(pickWeighted(w, 0b11, 1.0) == 1);   // rounding at the top end still lands on a tile
    assert(pickWeighted(w, 0b10, 0.0) == 1);   // only the second tile is left
    assert(pickWeighted(w, 0b10, 0.5) == 1);
    assert(pickWeighted(w, 0, 0.5) == -1);
    // With the first tile banned the weights of the others are what counts: 1 and 1.
    const TileSet t = makeSet({{{0, 0, 0, 0}, 3.0}, {{0, 0, 0, 0}, 1.0}, {{0, 0, 0, 0}, 1.0}});
    assert(pickWeighted(t, 0b110, 0.49) == 1);
    assert(pickWeighted(t, 0b110, 0.5) == 2);
}

void rngRange() {
    Rng a(42);
    Rng b(42);
    Rng c(43);
    bool differs = false;
    for (int i = 0; i < 1000; ++i) {
        const double x = a.uniform();
        assert(x >= 0.0 && x < 1.0);
        assert(x == b.uniform());
        differs = differs || x != c.uniform();
    }
    assert(differs);
}

void propagationOnALine() {
    const TileSet s = roadSet();
    // 3 cells in a row, nothing decided. Horizontal neighbours only, so nothing can be pruned at the start.
    Solver v(s, 3, 1, 1, Order::Scanline, Recovery::Restart);
    assert(v.domain == std::vector<Mask>({0b111, 0b111, 0b111}));
    // The middle cell is the horizontal road. Its neighbours must have a road on the touching edge too.
    v.domain[1] = 0b010;
    v.queue.push_back(1);
    v.queued[1] = 1;
    const long steps = v.run(100);
    // Pop of cell 1 changes both neighbours, pop of cell 0, pop of cell 2 (no change), then nothing is left: done.
    assert(steps == 4);
    assert(v.status == Status::Done);
    assert(v.domain == std::vector<Mask>({0b010, 0b010, 0b010}));
    assert(v.valid());
    assert(v.counters.contradictions == 0);

    // With the grass tile in the middle the neighbours may be grass or the vertical road.
    Solver g(s, 3, 1, 1, Order::Scanline, Recovery::Restart);
    g.domain[1] = 0b001;
    g.queue.push_back(1);
    g.queued[1] = 1;
    g.step();
    assert(g.domain[0] == 0b101);
    assert(g.domain[2] == 0b101);
    assert(g.status == Status::Running);
}

void contradictionIsDetected() {
    const TileSet s = roadSet();
    Solver v(s, 3, 1, 1, Order::Scanline, Recovery::Restart);
    // The horizontal road next to the vertical road cannot happen: no tile fits between them.
    v.domain[0] = 0b010;
    v.domain[2] = 0b100;
    v.queue.push_back(0);
    v.queued[0] = 1;
    v.queue.push_back(2);
    v.queued[2] = 1;
    v.step();  // cell 0 limits cell 1 to the horizontal road and puts it in the queue behind cell 2
    assert(v.domain[1] == 0b010);
    assert(v.queue.size() == 2);
    v.step();  // cell 2 (vertical road) wants a neighbour with grass or a vertical road on its west side: none is left
    assert(v.domain[1] == 0);
    assert(v.status == Status::Contradiction);
    assert(v.badCell == 1);
    assert(v.counters.contradictions == 1);
    assert(v.queue.empty());  // cell 1 was still waiting, the pending work is dropped
    assert(v.queued == std::vector<char>({0, 0, 0}));
}

/// Two tiles that only combine with themselves: X (west/east label 0) and Y (west/east label 1).
/// X is so much heavier that it is picked every time, unless something forbids it.
TileSet trapSet() { return makeSet({{{0, 0, 0, 0}, 1e9}, {{0, 1, 0, 1}, 1.0}}); }

void backtrackUndoesTheLastDecision() {
    const TileSet s = trapSet();
    // The last cell was forced to Y without propagating. The first collapse picks X and runs into it.
    Solver v(s, 3, 1, 5, Order::Entropy, Recovery::Backtrack);
    v.domain[2] = 0b10;
    int guard = 0;
    while (v.status == Status::Running && guard++ < 50) v.step();
    assert(v.status == Status::Contradiction);
    assert(v.stack.size() == 1);
    assert(v.stack[0].tile == 0);  // the heavy tile X was the decision
    const int cell = v.stack[0].cell;
    assert(cell == 0 || cell == 1);
    v.step();  // the backtrack
    assert(v.status == Status::Running);
    assert(v.counters.backtracks == 1);
    assert(v.counters.restarts == 0);
    assert(v.stack.empty());
    assert(v.domain[static_cast<std::size_t>(cell)] == 0b10);  // X is banned on that cell, Y is what is left
    assert(v.domain[2] == 0b10);                                // the forced cell is still there
    v.run(100);
    assert(v.status == Status::Done);
    assert(v.valid());
    assert(v.domain == std::vector<Mask>({0b10, 0b10, 0b10}));
    assert(v.counters.contradictions == 1);
    assert(v.counters.backtracks == 1);

    // The restart strategy loses the forced cell and starts over, so X wins everywhere.
    Solver r(s, 3, 1, 5, Order::Entropy, Recovery::Restart);
    r.domain[2] = 0b10;
    r.run(100);
    assert(r.status == Status::Done);
    assert(r.valid());
    assert(r.counters.restarts == 1);
    assert(r.counters.backtracks == 0);
    assert(r.domain == std::vector<Mask>({0b01, 0b01, 0b01}));
}

void backtrackKeepsPendingPropagation() {
    // A click while the queue is not empty: the snapshot has to hold the queue, otherwise the restore forgets work.
    const TileSet s = roadSet();
    Solver v(s, 4, 1, 3, Order::Scanline, Recovery::Backtrack);
    v.domain[0] = 0b010;
    v.queue.push_back(0);
    v.queued[0] = 1;
    assert(v.collapseCell(3));
    assert(v.stack.size() == 1);
    assert(v.stack[0].queue.size() == 1 && v.stack[0].queue[0] == 0);
    assert(v.stack[0].cell == 3);
}

void giveUpAfterTheLimit() {
    // One tile whose south edge (1) matches no north edge (0): even a 1x2 map cannot work.
    const TileSet s = makeSet({{{0, 0, 1, 0}, 1.0}});
    for (Recovery mode : {Recovery::Restart, Recovery::Backtrack}) {
        Solver v(s, 3, 3, 9, Order::Entropy, mode);
        assert(v.status == Status::Contradiction);  // found by the initial propagation
        const long taken = v.run(v.maxSteps());
        assert(v.status == Status::GaveUp);
        assert(v.finished());
        assert(v.counters.restarts == v.limits.maxRestarts);
        assert(v.counters.contradictions == v.limits.maxRestarts + 1);
        assert(taken == v.limits.maxRestarts + 1);  // one recovery step per contradiction
        assert(v.counters.steps <= v.maxSteps());
        assert(v.step() == Status::GaveUp);  // stays there
        assert(v.counters.steps == taken);
    }
    Limits few;
    few.maxRestarts = 3;
    Solver f(s, 2, 2, 1, Order::Random, Recovery::Restart, few);
    f.run(1000);
    assert(f.status == Status::GaveUp);
    assert(f.counters.restarts == 3);
    assert(f.counters.steps == 4);
}

void runStopsAtTheBudget() {
    const TileSet s = landTileSet(true);
    Solver v(s, 30, 11, 1, Order::Entropy, Recovery::Restart);
    assert(v.run(0) == 0);
    assert(v.run(10) == 10);
    assert(v.counters.steps == 10);
    assert(!v.finished());
}

void pickOrders() {
    const TileSet s = roadSet();
    // Cells: decided, 3 tiles, 2 tiles, 3 tiles.
    Solver sc(s, 4, 1, 1, Order::Scanline, Recovery::Restart);
    sc.domain = {0b001, 0b111, 0b011, 0b111};
    assert(sc.pickCell() == 1);
    Solver en(s, 4, 1, 1, Order::Entropy, Recovery::Restart);
    en.domain = {0b001, 0b111, 0b011, 0b111};
    assert(en.pickCell() == 2);  // two tiles are less open than three
    Solver rd(s, 4, 1, 1, Order::Random, Recovery::Restart);
    rd.domain = {0b001, 0b111, 0b011, 0b111};
    int expect = 1;
    for (int i : {2, 3}) {
        if (rd.noise[static_cast<std::size_t>(i)] < rd.noise[static_cast<std::size_t>(expect)]) expect = i;
    }
    assert(rd.pickCell() == expect);
    sc.domain = {0b001, 0b010, 0b100, 0b001};
    assert(sc.pickCell() == -1);
}

void entropyUsesTheWeights() {
    // Same number of tiles in both cells, but the first has one very likely tile, so it is nearly decided.
    const TileSet s = makeSet({{{0, 0, 0, 0}, 10.0}, {{0, 0, 0, 0}, 0.1}, {{0, 0, 0, 0}, 1.0}});
    Solver v(s, 2, 1, 1, Order::Entropy, Recovery::Restart);
    v.domain = {0b110, 0b011};  // {0.1, 1} and {10, 0.1}
    assert(entropy(s, 0b011) < entropy(s, 0b110));
    assert(v.pickCell() == 1);
}

void collapsedCountAndValid() {
    const TileSet s = roadSet();
    Solver v(s, 2, 2, 1, Order::Scanline, Recovery::Restart);
    assert(v.collapsedCount() == 0);
    assert(!v.valid());
    v.domain = {0b001, 0b001, 0b001, 0b010};
    assert(v.collapsedCount() == 4);
    assert(!v.valid());  // bottom left grass (east edge 0) meets the road piece (west edge 1)
    v.domain = {0b001, 0b001, 0b001, 0b001};
    assert(v.valid());
    v.domain = {0b010, 0b010, 0b001, 0b001};
    assert(v.valid());  // road pair on top: 1 meets 1, their south edges (0) meet the grass north edges (0)
    v.domain = {0b010, 0b001, 0b001, 0b001};
    assert(!v.valid());  // road east edge 1 against grass west edge 0
    v.domain = {0b100, 0b001, 0b001, 0b001};
    assert(!v.valid());  // vertical road above grass: south edge 1 against north edge 0, all horizontal pairs fit
    v.domain = {0b011, 0b001, 0b001, 0b001};
    assert(!v.valid());  // not decided
}

void collapseCellRules() {
    const TileSet s = roadSet();
    Solver v(s, 3, 1, 1, Order::Scanline, Recovery::Restart);
    assert(!v.collapseCell(-1));
    assert(!v.collapseCell(3));
    assert(v.collapseCell(1));
    assert(count(v.domain[1]) == 1);
    assert(!v.collapseCell(1));  // already decided
    assert(!v.queue.empty());
    v.run(100);
    assert(v.status == Status::Done);
    assert(!v.collapseCell(0));  // finished
}

void landSetShape() {
    const TileSet u = landTileSet(false);
    const TileSet t = landTileSet(true);
    assert(u.tiles.size() == 29 && t.tiles.size() == 29);
    for (const Tile& tile : u.tiles) assert(tile.weight == 1.0);
    // Tile 0 is all grass, tile 15 is all water, the last two are the bridges.
    for (int e : t.tiles[0].edge) assert(e == Grass);
    for (int e : t.tiles[15].edge) assert(e == Water);
    assert((t.tiles[27].edge == std::array<int, 4>{Water, Road, Water, Road}));
    assert((t.tiles[28].edge == std::array<int, 4>{Road, Water, Road, Water}));
    int ground = 0;
    int roads = 0;
    int bridges = 0;
    for (const Tile& tile : t.tiles) {
        int r = 0;
        int w = 0;
        for (int e : tile.edge) {
            r += e == Road ? 1 : 0;
            w += e == Water ? 1 : 0;
        }
        if (r == 0) ++ground;
        else if (w == 0) ++roads;
        else ++bridges;
        assert(r != 1);  // no dead ends
    }
    assert(ground == 16 && roads == 11 && bridges == 2);
    // Grass is the most likely tile, a road crossing the least.
    for (std::size_t i = 1; i < t.tiles.size(); ++i) assert(t.tiles[0].weight > t.tiles[i].weight);
    // The compat relation is symmetric: u next to t on side s means t next to u on the opposite side.
    for (int side = 0; side < kSides; ++side) {
        for (std::size_t a = 0; a < t.tiles.size(); ++a) {
            assert(t.compat[static_cast<std::size_t>(side)][a] != 0);
            for (std::size_t b = 0; b < t.tiles.size(); ++b) {
                const bool ab = (t.compat[static_cast<std::size_t>(side)][a] >> b & 1U) != 0;
                const bool ba = (t.compat[static_cast<std::size_t>(opposite(side))][b] >> a & 1U) != 0;
                assert(ab == ba);
            }
        }
    }
    // East of the east-west bridge only tiles with a road on their west edge fit: 7 of the 11 road tiles
    // (masks 8 to 15 with at least two roads) and the bridge itself.
    assert(count(t.compat[East][27]) == 8);
    assert((t.compat[East][27] >> 27 & 1U) != 0);
    assert((t.compat[East][27] >> 28 & 1U) == 0);
}

void landSetEverySideHasSupport() {
    const TileSet t = landTileSet(true);
    Solver v(t, 30, 11, 1, Order::Entropy, Recovery::Restart);
    for (Mask m : v.domain) assert(m == t.all);  // the initial propagation prunes nothing
    assert(v.counters.steps == 0 && v.counters.contradictions == 0);
}

void manyMapsEndAndAreValid() {
    for (bool tuned : {true, false}) {
        const TileSet set = landTileSet(tuned);
        for (Order order : {Order::Entropy, Order::Random, Order::Scanline}) {
            for (Recovery mode : {Recovery::Restart, Recovery::Backtrack}) {
                for (std::uint64_t seed = 1; seed <= 12; ++seed) {
                    Solver v(set, 30, 11, seed * 977, order, mode);
                    v.run(v.maxSteps());
                    assert(v.finished());  // the loop ends, whatever the order does
                    assert(v.counters.steps <= v.maxSteps());
                    assert(v.counters.restarts <= v.limits.maxRestarts);
                    if (v.status == Status::Done) assert(v.valid());
                    else assert(v.status == Status::GaveUp);
                }
            }
        }
    }
}

void sameSeedSameMap() {
    const TileSet set = landTileSet(true);
    Solver a(set, 30, 11, 123, Order::Entropy, Recovery::Restart);
    Solver b(set, 30, 11, 123, Order::Entropy, Recovery::Restart);
    Solver c(set, 30, 11, 124, Order::Entropy, Recovery::Restart);
    a.run(a.maxSteps());
    b.run(b.maxSteps());
    c.run(c.maxSteps());
    assert(a.domain == b.domain);
    assert(a.counters.steps == b.counters.steps);
    assert(a.domain != c.domain);
    // reset() with the same seed is the same map again.
    a.reset(123);
    assert(a.counters.steps == 0);
    a.run(a.maxSteps());
    assert(a.domain == b.domain);
}

double grassShare(bool tuned) {
    const TileSet set = landTileSet(tuned);
    long grass = 0;
    long cells = 0;
    for (std::uint64_t seed = 1; seed <= 20; ++seed) {
        Solver v(set, 30, 11, seed * 31, Order::Entropy, Recovery::Restart);
        v.run(v.maxSteps());
        if (v.status != Status::Done) continue;
        for (Mask m : v.domain) {
            grass += m == bit(0) ? 1 : 0;
            ++cells;
        }
    }
    assert(cells > 0);
    return static_cast<double>(grass) / static_cast<double>(cells);
}

void weightsShapeTheMap() {
    const double tuned = grassShare(true);
    const double uniform = grassShare(false);
    std::fprintf(stderr, "grass share: tuned %.3f, uniform %.3f\n", tuned, uniform);
    assert(tuned > 0.3);
    assert(uniform < 0.15);
}

void scanlineNeverGetsStuckOnThisSet() {
    // Scanline order decides a cell when only its north and west neighbours exist. The land set has a tile for every
    // pair (tile above, tile left), so that order cannot run into a contradiction. Not true for every tileset.
    const TileSet t = landTileSet(true);
    for (std::size_t above = 0; above < t.tiles.size(); ++above) {
        for (std::size_t left = 0; left < t.tiles.size(); ++left) {
            assert((t.compat[South][above] & t.compat[East][left]) != 0);
        }
    }
    // A set without that property: X is all 0, Y is all 1, so below an X and right of a Y there is no tile.
    const TileSet bad = makeSet({{{0, 0, 0, 0}, 1.0}, {{1, 1, 1, 1}, 1.0}});
    assert(bad.compat[South][0] == 0b01 && bad.compat[East][1] == 0b10);
    assert((bad.compat[South][0] & bad.compat[East][1]) == 0);
}

void orderMatters() {
    const TileSet set = landTileSet(true);
    const Bench entropy = benchmark(set, 30, 11, Order::Entropy, 60, 5);
    const Bench random = benchmark(set, 30, 11, Order::Random, 60, 5);
    const Bench scan = benchmark(set, 30, 11, Order::Scanline, 60, 5);
    std::fprintf(stderr, "contradictions per 60 maps: entropy %d, random %d, scanline %d (gave up %d, %d, %d)\n", entropy.contradictions,
                 random.contradictions, scan.contradictions, entropy.gaveUp, random.gaveUp, scan.gaveUp);
    assert(entropy.maps == 60 && random.maps == 60);
    assert(random.contradictions > 20 * (entropy.contradictions + 1));  // a random cell is the naive way and it hurts
    assert(scan.contradictions == 0);                                    // see scanlineNeverGetsStuckOnThisSet
    assert(random.gaveUp == 0);                                          // but forty restarts are always enough here
    assert(random.steps > entropy.steps);                                // and the restarts cost steps
}

// ---- tile art ----

bool sameColor(art::Rgb a, art::Rgb b) { return a.r == b.r && a.g == b.g && a.b == b.b; }

void kindsAtKnownPoints() {
    using art::Kind;
    const Tile grass{{Grass, Grass, Grass, Grass}, 1.0};
    const Tile water{{Water, Water, Water, Water}, 1.0};
    const Tile coast{{Water, Grass, Grass, Grass}, 1.0};  // water on the north edge
    const Tile roadEW{{Grass, Road, Grass, Road}, 1.0};
    const Tile bridgeEW{{Water, Road, Water, Road}, 1.0};
    assert(art::kindAt(grass, 0.5, 0.5) == Kind::Grass);
    assert(art::kindAt(water, 0.5, 0.5) == Kind::Water);
    assert(art::kindAt(water, 0.02, 0.02) == Kind::Water);  // the corners of an all-water tile are water as well
    assert(art::kindAt(water, 0.98, 0.97) == Kind::Water);
    assert(art::kindAt(coast, 0.5, 0.2) == Kind::Water);    // lens depth in the middle is 0.54
    assert(art::kindAt(coast, 0.5, 0.58) == Kind::Sand);    // 0.04 beyond the lens, inside the rim of 0.09
    assert(art::kindAt(coast, 0.5, 0.7) == Kind::Grass);
    assert(art::kindAt(coast, 0.0, 0.3) == Kind::Grass);    // the lens is 0 wide at the ends of the edge
    assert(art::kindAt(coast, 1.0, 0.3) == Kind::Grass);
    assert(art::kindAt(roadEW, 0.5, 0.5) == Kind::Road);
    assert(art::kindAt(roadEW, 0.5, 0.66) == Kind::RoadRim);  // 0.16 from the middle: past 0.14, inside 0.19
    assert(art::kindAt(roadEW, 0.5, 0.75) == Kind::Grass);
    assert(art::kindAt(roadEW, 0.05, 0.5) == Kind::Road);     // all the way to the edge
    assert(art::kindAt(roadEW, 0.5, 0.0) == Kind::Grass);     // nothing on the north edge
    assert(art::kindAt(bridgeEW, 0.55, 0.5) == Kind::Wood);   // 0.55 * 8 = 4.4, fractional part 0.4
    assert(art::kindAt(bridgeEW, 0.5, 0.5) == Kind::Plank);   // 0.5 * 8 = 4.0, a plank line
    assert(art::kindAt(bridgeEW, 0.05, 0.5) == Kind::Road);   // the ends look like the road next to it
    assert(art::kindAt(bridgeEW, 0.5, 0.1) == Kind::Water);   // river above and below
}

void touchingTilesLookContinuous() {
    const TileSet t = landTileSet(true);
    int pairs = 0;
    for (int side = 0; side < kSides; ++side) {
        for (std::size_t a = 0; a < t.tiles.size(); ++a) {
            for (std::size_t b = 0; b < t.tiles.size(); ++b) {
                if ((t.compat[static_cast<std::size_t>(side)][a] >> b & 1U) == 0) continue;
                ++pairs;
                for (int i = 1; i < 20; ++i) {
                    const double k = i / 20.0;
                    double ua = 0, va = 0, ub = 0, vb = 0;
                    switch (side) {
                        case North: ua = k; va = 0.0; ub = k; vb = 1.0; break;
                        case East: ua = 1.0; va = k; ub = 0.0; vb = k; break;
                        case South: ua = k; va = 1.0; ub = k; vb = 0.0; break;
                        default: ua = 0.0; va = k; ub = 1.0; vb = k; break;
                    }
                    assert(sameColor(art::sample(t.tiles[a], ua, va), art::sample(t.tiles[b], ub, vb)));
                }
            }
        }
    }
    assert(pairs > 1000);
}

void renderedTile() {
    const TileSet t = landTileSet(true);
    const auto px = art::render(t.tiles[0], 30, 0);
    assert(px.size() == 30U * 30U * 4U);
    for (std::size_t i = 3; i < px.size(); i += 4) assert(px[i] == 255);
    const art::Rgb g = art::mean(px);
    assert(near(g.r, art::kGrass.r, 0.05) && near(g.g, art::kGrass.g, 0.05) && near(g.b, art::kGrass.b, 0.05));
    const art::Rgb w = art::mean(art::render(t.tiles[15], 30, 1));
    assert(w.b > w.r && w.b > 0.7F);
    assert(art::render(t.tiles[0], 30, 0) == art::render(t.tiles[0], 30, 0));  // same salt, same specks
    assert(art::mean({}).g > 0.0F);                                             // empty input does not divide by zero
}

}  // namespace

int main() {
    compatTable();
    buildRejectsBadSets();
    entropyValues();
    weightedPick();
    rngRange();
    propagationOnALine();
    contradictionIsDetected();
    backtrackUndoesTheLastDecision();
    backtrackKeepsPendingPropagation();
    giveUpAfterTheLimit();
    runStopsAtTheBudget();
    pickOrders();
    entropyUsesTheWeights();
    collapsedCountAndValid();
    collapseCellRules();
    landSetShape();
    landSetEverySideHasSupport();
    manyMapsEndAndAreValid();
    sameSeedSameMap();
    weightsShapeTheMap();
    scanlineNeverGetsStuckOnThisSet();
    orderMatters();
    kindsAtKnownPoints();
    touchingTilesLookContinuous();
    renderedTile();
    std::puts("wfc-tiles logic: all asserts passed");
    return 0;
}
