// Tests for the propagation levels of the wave function collapse solver. Plain asserts on hand-computed cases, no SDL.
// Tile edges are written N, E, S, W (clockwise from the top), labels are plain numbers.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

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
TileSet roadSet() { return makeSet({{{0, 0, 0, 0}, 1.0}, {{0, 1, 0, 1}, 1.0}, {{1, 0, 1, 0}, 1.0}}); }

constexpr Propagation kLevels[] = {Propagation::Off, Propagation::Neighbors, Propagation::Full};
constexpr Order kOrders[] = {Order::Entropy, Order::Scanline, Order::Random};

void compatTable() {
    const TileSet s = roadSet();
    assert(s.all == 0b111);
    assert(s.compat[East][0] == 0b101);  // east of grass: west edge 0 = grass and the vertical road
    assert(s.compat[East][1] == 0b010);  // east of the horizontal road: only the horizontal road
    assert(s.compat[East][2] == 0b101);
    assert(s.compat[West][1] == 0b010);
    assert(s.compat[North][2] == 0b100);  // north of the vertical road: south edge 1, only the vertical road
    assert(s.compat[North][0] == 0b011);
}

void buildRejectsBadSets() {
    TileSet empty;
    assert(!empty.build());
    TileSet zero;
    zero.tiles = {{{0, 0, 0, 0}, 0.0}};
    assert(!zero.build());
    TileSet big;
    for (int i = 0; i < kMaxTiles + 1; ++i) big.tiles.push_back({{0, 0, 0, 0}, 1.0});
    assert(!big.build());
}

void entropyAndPick() {
    const TileSet s = roadSet();
    assert(near(entropy(s, 0), 0.0));
    assert(near(entropy(s, 0b100), 0.0));
    assert(near(entropy(s, 0b011), std::log(2.0)));
    const TileSet w = makeSet({{{0, 0, 0, 0}, 3.0}, {{0, 0, 0, 0}, 1.0}});
    assert(near(entropy(w, 0b11), 0.5623351446, 1e-9));  // ln 4 - 3 ln 3 / 4
    assert(pickWeighted(w, 0b11, 0.7499) == 0);
    assert(pickWeighted(w, 0b11, 0.75) == 1);  // 0.75 * 4 = 3 = the end of the first tile
    assert(pickWeighted(w, 0b11, 1.0) == 1);
    assert(pickWeighted(w, 0, 0.5) == -1);
}

void rngRange() {
    Rng a(42);
    Rng b(42);
    for (int i = 0; i < 1000; ++i) {
        const double x = a.uniform();
        assert(x >= 0.0 && x < 1.0);
        assert(x == b.uniform());
    }
}

/// One road tile in the first cell of a row of five. How far the news travels is what the three levels differ in.
void rowFull() {
    const TileSet s = roadSet();
    Solver v(s, 5, 1, 1, Order::Scanline, Propagation::Full);
    assert(v.assign(0, 1));
    assert(v.queue.size() == 1);
    // Five pops (cell 0 narrows cell 1, cell 1 narrows cell 2, ... cell 4 changes nothing), then the Done step.
    const long steps = v.run(100);
    assert(steps == 6);
    assert(v.status == Status::Done);
    assert(v.domain == std::vector<Mask>({0b010, 0b010, 0b010, 0b010, 0b010}));
    assert(v.valid());
    assert(v.consistent());
    assert(v.counters.narrowed == 4);
    assert(v.counters.decisions == 1);
    assert(v.counters.contradictions == 0);
}

void rowNeighbors() {
    const TileSet s = roadSet();
    Solver v(s, 5, 1, 1, Order::Scanline, Propagation::Neighbors);
    assert(v.assign(0, 1));
    assert(v.queue.empty());  // nothing travels on
    assert(v.domain == std::vector<Mask>({0b010, 0b010, 0b111, 0b111, 0b111}));
    assert(v.counters.narrowed == 1);
    assert(!v.consistent());  // grass in cell 2 has no support in cell 1 any more
    assert(v.options(2) == 0b111);
}

void rowOff() {
    const TileSet s = roadSet();
    Solver v(s, 5, 1, 1, Order::Scanline, Propagation::Off);
    assert(v.assign(0, 1));
    assert(v.domain == std::vector<Mask>({0b010, 0b111, 0b111, 0b111, 0b111}));
    assert(v.counters.narrowed == 0);
    // The stored set of cell 1 still says "anything". Only looking at the decided neighbour reveals what is left.
    assert(v.options(1) == 0b010);
    assert(v.options(2) == 0b111);
    // Off decides one cell per step, nothing is ever pending: five decisions and the Done step.
    assert(v.run(100) == 5);
    assert(v.status == Status::Done);
    assert(v.valid());
    assert(v.counters.decisions == 5);
    assert(v.counters.narrowed == 0);
}

/// Road in the first cell, grass in the last, nothing decided in between: no tile fits cell 1 (the road wants a road
/// on its east side, the grass on the other side wants grass). Who notices, and when, is the difference.
void sandwichOff() {
    const TileSet s = roadSet();
    Solver v(s, 3, 1, 1, Order::Scanline, Propagation::Off);
    assert(v.assign(0, 1));
    assert(v.assign(2, 0));  // nobody stops this, cell 2 only looks at decided neighbours
    assert(v.status == Status::Running);
    assert(v.domain[1] == 0b111);  // the map still believes cell 1 can be anything
    assert(v.options(1) == 0);
    assert(v.doomedCount() == 1);
    v.step();  // the solver gets to cell 1 and finds out
    assert(v.status == Status::Contradiction);
    assert(v.badCell == 1);
    assert(v.domain[1] == 0);
    assert(v.counters.contradictions == 1);
    assert(v.counters.steps == 1);
    assert(v.doomedCount() == 0);
    v.step();  // the recovery
    assert(v.status == Status::Running);
    assert(v.counters.restarts == 1);
    assert(v.domain == std::vector<Mask>({0b111, 0b111, 0b111}));
}

void sandwichNeighbors() {
    const TileSet s = roadSet();
    Solver v(s, 3, 1, 1, Order::Scanline, Propagation::Neighbors);
    assert(v.assign(0, 1));
    assert(v.domain[1] == 0b010);
    // The grass in cell 2 empties cell 1 in the same moment: the neighbour is narrowed, not asked later.
    assert(v.assign(2, 0));
    assert(v.status == Status::Contradiction);
    assert(v.badCell == 1);
    assert(v.domain[1] == 0);
    assert(v.counters.contradictions == 1);
    assert(v.counters.steps == 0);  // assign() is a click, not a solver step
    assert(v.doomedCount() == 0);
}

void sandwichFull() {
    const TileSet s = roadSet();
    Solver v(s, 3, 1, 1, Order::Scanline, Propagation::Full);
    assert(v.assign(0, 1));
    while (!v.queue.empty()) v.step();
    // The road has already pushed its news to the far end: the grass is not an option for cell 2 any more.
    assert(v.domain == std::vector<Mask>({0b010, 0b010, 0b010}));
    assert(!v.assign(2, 0));
    assert(v.status == Status::Running);
    assert(v.counters.contradictions == 0);
    assert(v.step() == Status::Done);
    assert(v.valid());
}

void collapseCellRules() {
    const TileSet s = roadSet();
    for (Propagation level : kLevels) {
        Solver v(s, 3, 1, 1, Order::Scanline, level);
        assert(!v.collapseCell(-1));
        assert(!v.collapseCell(3));
        assert(!v.assign(1, 3));   // there is no tile 3
        assert(!v.assign(1, -1));
        assert(v.collapseCell(1));
        assert(count(v.domain[1]) == 1);
        assert(!v.collapseCell(1));  // already decided
        v.run(100);
        assert(v.status == Status::Done);
        assert(!v.collapseCell(0));  // finished
    }
    // Clicking a cell that is already doomed shows the contradiction.
    Solver off(s, 3, 1, 1, Order::Scanline, Propagation::Off);
    assert(off.assign(0, 1));
    assert(off.assign(2, 0));
    assert(off.collapseCell(1));
    assert(off.status == Status::Contradiction);
    assert(off.badCell == 1);
    assert(!off.collapseCell(0));  // nothing is decided while the contradiction is on screen
}

/// Without propagation every open cell still holds all tiles, so the entropy has nothing to measure: the order
/// "lowest entropy" is the order "random" (the tie-break noise decides in both).
void entropyIsBlindWithoutPropagation() {
    const TileSet set = landTileSet(true);
    for (std::uint64_t seed = 1; seed <= 20; ++seed) {
        Solver e(set, 12, 8, seed, Order::Entropy, Propagation::Off);
        Solver r(set, 12, 8, seed, Order::Random, Propagation::Off);
        assert(e.pickCell() == r.pickCell());
        e.run(e.maxSteps());
        r.run(r.maxSteps());
        assert(e.domain == r.domain);
        assert(e.counters.steps == r.counters.steps);
        assert(e.counters.contradictions == r.counters.contradictions);
    }
    // With propagation the sets differ and the entropy order starts to mean something.
    Solver f(set, 12, 8, 3, Order::Entropy, Propagation::Full);
    assert(f.assign(40, 0));
    while (!f.queue.empty()) f.step();
    const int picked = f.pickCell();
    assert(picked >= 0);
    assert(count(f.domain[static_cast<std::size_t>(picked)]) < count(set.all));  // a cell the decision has narrowed
}

void pickOrders() {
    const TileSet s = roadSet();
    // Cells: decided, 3 tiles, 2 tiles, 3 tiles.
    for (Propagation level : {Propagation::Neighbors, Propagation::Full}) {
        Solver sc(s, 4, 1, 1, Order::Scanline, level);
        sc.domain = {0b001, 0b111, 0b011, 0b111};
        assert(sc.pickCell() == 1);
        Solver en(s, 4, 1, 1, Order::Entropy, level);
        en.domain = {0b001, 0b111, 0b011, 0b111};
        assert(en.pickCell() == 2);  // two tiles are less open than three
        sc.domain = {0b001, 0b010, 0b100, 0b001};
        assert(sc.pickCell() == -1);
    }
    Solver rd(s, 4, 1, 1, Order::Random, Propagation::Full);
    rd.domain = {0b001, 0b111, 0b011, 0b111};
    int expect = 1;
    for (int i : {2, 3}) {
        if (rd.noise[static_cast<std::size_t>(i)] < rd.noise[static_cast<std::size_t>(expect)]) expect = i;
    }
    assert(rd.pickCell() == expect);
    // Off + scanline walks the cells in reading order and ignores everything else.
    Solver off(s, 4, 1, 1, Order::Scanline, Propagation::Off);
    off.domain = {0b001, 0b111, 0b111, 0b111};
    assert(off.pickCell() == 1);
}

void giveUpAfterTheLimit() {
    // Two tiles (so every cell is open) whose south edge (1) matches no north edge (0): a map with two rows is impossible.
    const TileSet s = makeSet({{{0, 0, 1, 0}, 1.0}, {{0, 0, 1, 0}, 2.0}});
    {
        Solver v(s, 3, 3, 9, Order::Entropy, Propagation::Full);
        assert(v.status == Status::Contradiction);  // found by the initial propagation
        const long taken = v.run(v.maxSteps());
        assert(v.status == Status::GaveUp);
        assert(v.counters.restarts == v.limits.maxRestarts);
        assert(v.counters.contradictions == v.limits.maxRestarts + 1);
        assert(taken == v.limits.maxRestarts + 1);  // one recovery step per contradiction
        assert(v.step() == Status::GaveUp);         // stays there
        assert(v.counters.steps == taken);
    }
    Limits few;
    few.maxRestarts = 3;
    {
        // Neighbors: the first decision empties the cell below it, step 1 decides and fails, step 2 recovers.
        Solver v(s, 2, 2, 1, Order::Scanline, Propagation::Neighbors, few);
        v.run(1000);
        assert(v.status == Status::GaveUp);
        assert(v.counters.restarts == 3);
        assert(v.counters.contradictions == 4);
        assert(v.counters.steps == 8);
    }
    {
        // Off, reading order: decide cells 0 and 1, the stand at cell 2 (below cell 0) fails. 3 steps and a recovery.
        Solver v(s, 2, 2, 1, Order::Scanline, Propagation::Off, few);
        v.run(1000);
        assert(v.status == Status::GaveUp);
        assert(v.counters.restarts == 3);
        assert(v.counters.contradictions == 4);
        assert(v.counters.steps == 16);
    }
}

void runStopsAtTheBudget() {
    const TileSet s = landTileSet(true);
    Solver v(s, 10, 10, 1, Order::Entropy, Propagation::Full);
    assert(v.run(0) == 0);
    assert(v.run(10) == 10);
    assert(v.counters.steps == 10);
    assert(!v.finished());
}

/// Whatever the level and the order do: a set is never empty without the solver saying so, a finished map is valid, and
/// with the full cascade every quiet moment is arc consistent.
void invariantsOnManyMaps() {
    for (bool tuned : {true, false}) {
        const TileSet set = landTileSet(tuned);
        for (Propagation level : kLevels) {
            for (Order order : kOrders) {
                for (std::uint64_t seed = 1; seed <= 12; ++seed) {
                    Solver v(set, 10, 10, seed * 977, order, level);
                    const long cap = v.maxSteps();
                    while (!v.finished()) {
                        v.step();
                        assert(v.counters.steps <= cap);
                        int empty = 0;
                        for (int i = 0; i < v.cells(); ++i) {
                            if (v.domain[static_cast<std::size_t>(i)] == 0) {
                                ++empty;
                                assert((v.status == Status::Contradiction || v.status == Status::GaveUp) && i == v.badCell);
                            }
                        }
                        assert(empty == (v.status == Status::Contradiction ? 1 : 0) || v.status == Status::GaveUp);
                        if (level == Propagation::Full && v.status == Status::Running && v.queue.empty()) assert(v.consistent());
                    }
                    if (v.status == Status::Done) assert(v.valid());
                    else assert(v.status == Status::GaveUp);
                    assert(v.counters.restarts <= v.limits.maxRestarts);
                    if (level == Propagation::Off) assert(v.counters.narrowed == 0);
                }
            }
        }
    }
}

/// Without propagation open cells that can take no tile any more do exist, unnoticed (that is what the demo shows).
void doomedCellsExistWithoutPropagation() {
    const TileSet set = landTileSet(true);
    int offSeen = 0;
    for (std::uint64_t seed = 1; seed <= 30; ++seed) {
        Solver v(set, 10, 10, seed * 31, Order::Random, Propagation::Off);
        while (!v.finished()) {
            v.step();
            if (v.doomedCount() > 0) ++offSeen;
        }
    }
    assert(offSeen > 0);
}

void sameSeedSameMap() {
    const TileSet set = landTileSet(true);
    for (Propagation level : kLevels) {
        Solver a(set, 10, 10, 123, Order::Entropy, level);
        Solver b(set, 10, 10, 123, Order::Entropy, level);
        Solver c(set, 10, 10, 124, Order::Entropy, level);
        a.run(a.maxSteps());
        b.run(b.maxSteps());
        c.run(c.maxSteps());
        assert(a.domain == b.domain);
        assert(a.counters.steps == b.counters.steps);
        assert(a.domain != c.domain);
        a.reset(123);
        assert(a.counters.steps == 0);
        a.run(a.maxSteps());
        assert(a.domain == b.domain);
        assert(a.counters.steps == b.counters.steps);
    }
}

void landSetShape() {
    const TileSet t = landTileSet(true);
    const TileSet u = landTileSet(false);
    assert(t.tiles.size() == 29 && u.tiles.size() == 29);
    for (const Tile& tile : u.tiles) assert(tile.weight == 1.0);
    for (std::size_t i = 1; i < t.tiles.size(); ++i) assert(t.tiles[0].weight > t.tiles[i].weight);
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
    assert(count(t.compat[East][27]) == 8);  // east of the east-west bridge: 7 road tiles with a west road, plus the bridge
}

void benchmarkIsDeterministic() {
    const TileSet set = landTileSet(true);
    for (Propagation level : kLevels) {
        const Bench a = benchmark(set, 10, 10, Order::Random, level, 10, 5);
        const Bench b = benchmark(set, 10, 10, Order::Random, level, 10, 5);
        assert(a.maps == 10 && b.maps == 10);
        assert(a.contradictions == b.contradictions);
        assert(a.steps == b.steps);
        assert(a.gaveUp == b.gaveUp);
        assert(a.clean <= a.maps);
        assert(a.clean + a.gaveUp <= a.maps);
    }
    // A map that cannot work gives up at the cap, and the contradictions are what the cap allows.
    const TileSet bad = makeSet({{{0, 0, 1, 0}, 1.0}, {{0, 0, 1, 0}, 2.0}});
    Limits lim;
    lim.maxRestarts = 2;
    const Bench b = benchmark(bad, 3, 3, Order::Scanline, Propagation::Full, 4, 1, lim);
    assert(b.maps == 4 && b.gaveUp == 4 && b.clean == 0);
    assert(b.contradictions == 4 * 3);
}

/// Reading order only ever looks at the cell's north and west neighbour (the others are still open). The land set has a tile
/// for every pair of labels on those two edges, so a cell can always be filled: no propagation needed, no contradiction.
void readingOrderNeedsNoPropagationOnTheLandSet() {
    const TileSet set = landTileSet(true);
    for (int north = Grass; north <= Road; ++north) {
        for (int west = Grass; west <= Road; ++west) {
            bool found = false;
            for (const Tile& t : set.tiles) found = found || (t.edge[North] == north && t.edge[West] == west);
            assert(found);
        }
    }
    for (std::uint64_t seed = 1; seed <= 40; ++seed) {
        Solver v(set, 10, 10, seed * 13, Order::Scanline, Propagation::Off);
        v.run(v.maxSteps());
        assert(v.status == Status::Done && v.valid());
        assert(v.counters.contradictions == 0);
        assert(v.counters.decisions == 100);
    }
    // A set without that completeness has to pay: in the road set no tile has a road on its north and its west edge, so a
    // cell with the horizontal road to its west and the vertical road to its north has nothing left.
    const TileSet roads = roadSet();
    int contradictions = 0;
    for (std::uint64_t seed = 1; seed <= 40; ++seed) {
        Solver v(roads, 6, 6, seed, Order::Scanline, Propagation::Off);
        v.run(v.maxSteps());
        contradictions += v.counters.contradictions;
    }
    assert(contradictions > 0);
}

/// The claims of the post, measured: 5 seed sets of 40 maps of 10x10, restarts capped at 100. The numbers are what this
/// tileset does, not a law. Generous bounds, the exact counts depend on the rounding of the platform.
void measuredClaims() {
    const TileSet set = landTileSet(true);
    Limits cap;
    cap.maxRestarts = 100;
    auto total = [&](Order order, Propagation level) {
        Bench sum;
        for (int k = 0; k < 5; ++k) {
            const Bench b = benchmark(set, 10, 10, order, level, 40, 5 + 1000ULL * static_cast<std::uint64_t>(k), cap);
            sum.contradictions += b.contradictions;
            sum.maps += b.maps;
            sum.clean += b.clean;
        }
        return sum;
    };
    for (Order order : kOrders) {
        const Bench off = total(order, Propagation::Off);
        const Bench neigh = total(order, Propagation::Neighbors);
        const Bench full = total(order, Propagation::Full);
        assert(off.maps == 200);
        // Without propagation never fewer contradictions than with it.
        assert(off.contradictions >= full.contradictions);
        assert(neigh.contradictions >= full.contradictions);
    }
    const Bench fullEntropy = total(Order::Entropy, Propagation::Full);
    const Bench fullScan = total(Order::Scanline, Propagation::Full);
    const Bench fullRandom = total(Order::Random, Propagation::Full);
    assert(fullEntropy.contradictions == 0 && fullEntropy.clean == 200);
    assert(fullScan.contradictions == 0 && fullScan.clean == 200);
    assert(fullRandom.contradictions > 0 && fullRandom.contradictions < 400);  // random is the only order that still trips
    // Entropy and random without propagation are the same thing, and far worse than with it.
    const Bench offEntropy = total(Order::Entropy, Propagation::Off);
    const Bench offRandom = total(Order::Random, Propagation::Off);
    assert(offEntropy.contradictions == offRandom.contradictions);
    assert(offEntropy.contradictions > 20 * (fullRandom.contradictions + 1));
    // The half measure helps entropy a lot but is not zero, and it does nothing for the random order.
    const Bench neighEntropy = total(Order::Entropy, Propagation::Neighbors);
    const Bench neighRandom = total(Order::Random, Propagation::Neighbors);
    assert(neighEntropy.contradictions > 0 && neighEntropy.contradictions * 4 < offEntropy.contradictions);
    assert(neighRandom.contradictions * 2 > offRandom.contradictions);
}

/// What "nobody noticed" costs: without propagation the solver keeps deciding cells after a cell has lost its last
/// tile. Measured as the decisions between the first moment an open cell has no option left and the contradiction, over
/// 40 maps of 10x10 in random order (the entropy order is the same thing without propagation). Measured: 23.7 on
/// average over 1109 contradictions, of about 57 decisions per attempt.
void decisionsAfterTheMapWasLost() {
    const TileSet set = landTileSet(true);
    Limits cap;
    cap.maxRestarts = 100;
    long blind = 0;
    long attemptDecisions = 0;
    int contradictions = 0;
    for (int i = 0; i < 40; ++i) {
        Solver v(set, 10, 10, benchmarkSeed(5, i), Order::Random, Propagation::Off, cap);
        long firstDoomed = -1;
        long attemptStart = 0;
        while (!v.finished()) {
            const int before = v.counters.contradictions;
            v.step();
            if (v.counters.contradictions > before) {
                ++contradictions;
                assert(firstDoomed >= 0);  // a cell is lost before it is found out
                blind += v.counters.decisions - firstDoomed;
                attemptDecisions += v.counters.decisions - attemptStart;
                firstDoomed = -1;
            }
            if (v.status != Status::Running) continue;
            if (firstDoomed < 0 && v.doomedCount() > 0) firstDoomed = v.counters.decisions;
            if (v.collapsedCount() == 0) attemptStart = v.counters.decisions;
        }
    }
    assert(contradictions > 500);
    const double avgBlind = static_cast<double>(blind) / contradictions;
    const double avgAttempt = static_cast<double>(attemptDecisions) / contradictions;
    assert(avgBlind > 15.0 && avgBlind < 35.0);
    assert(avgAttempt > 45.0 && avgAttempt < 70.0);
    assert(avgBlind < avgAttempt);
}

/// The price of the cascade: in reading order, where it buys nothing on this set, maps from the same seeds cost about 4.6
/// times the steps (101 against 468 per map). Without propagation a map is one step per cell and the Done step.
void cascadeCostsSteps() {
    const TileSet set = landTileSet(true);
    const Bench off = benchmark(set, 10, 10, Order::Scanline, Propagation::Off, 40, 5);
    const Bench full = benchmark(set, 10, 10, Order::Scanline, Propagation::Full, 40, 5);
    assert(off.steps == 40 * 101);
    assert(off.contradictions == 0 && full.contradictions == 0);
    assert(full.steps > 3 * off.steps && full.steps < 7 * off.steps);
}

/// Why the half measure fails: a neighbour that is cut down to one tile counts as decided, but it does not tell its own
/// neighbours. sandwichNeighbors() shows one such case by hand. If those cells do pass the news on (the test variant), entropy
/// and reading order have no contradictions left in 200 maps, so the silence is the whole cause there. The random order
/// needs more: cells with two or three tiles left have to pass on their news as well, which only the full cascade does.
void silentCellsExplainTheHalfMeasure() {
    const TileSet set = landTileSet(true);
    Limits cap;
    cap.maxRestarts = 100;
    auto total = [&](Order order, bool speak) {
        long sum = 0;
        int clean = 0;
        for (int k = 0; k < 5; ++k) {
            for (int i = 0; i < 40; ++i) {
                Solver v(set, 10, 10, benchmarkSeed(5 + 1000ULL * static_cast<std::uint64_t>(k), i), order, Propagation::Neighbors, cap);
                v.singletonsSpeak = speak;
                v.reset(benchmarkSeed(5 + 1000ULL * static_cast<std::uint64_t>(k), i));
                v.run(v.maxSteps());
                sum += v.counters.contradictions;
                clean += v.status == Status::Done && v.counters.contradictions == 0 ? 1 : 0;
            }
        }
        return std::pair<long, int>{sum, clean};
    };
    assert(total(Order::Entropy, false).first > 100);
    assert(total(Order::Scanline, false).first > 100);
    assert(total(Order::Entropy, true) == (std::pair<long, int>{0, 200}));
    assert(total(Order::Scanline, true) == (std::pair<long, int>{0, 200}));
    assert(total(Order::Random, true).first > 1000);  // measured 6101, the full cascade has 78
}

}  // namespace

int main() {
    compatTable();
    buildRejectsBadSets();
    entropyAndPick();
    rngRange();
    rowFull();
    rowNeighbors();
    rowOff();
    sandwichOff();
    sandwichNeighbors();
    sandwichFull();
    collapseCellRules();
    entropyIsBlindWithoutPropagation();
    pickOrders();
    giveUpAfterTheLimit();
    runStopsAtTheBudget();
    invariantsOnManyMaps();
    doomedCellsExistWithoutPropagation();
    sameSeedSameMap();
    landSetShape();
    benchmarkIsDeterministic();
    readingOrderNeedsNoPropagationOnTheLandSet();
    measuredClaims();
    decisionsAfterTheMapWasLost();
    cascadeCostsSteps();
    silentCellsExplainTheHalfMeasure();
    std::puts("wfc-propagation: all logic tests passed");
    return 0;
}
