// Tests for the 3D wave function collapse logic. Plain asserts on hand-computed cases, no framework, no SDL, no GL.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <set>
#include <vector>

#include "castle.hpp"
#include "logic.hpp"
#include "mesh.hpp"

using namespace w3;

namespace {

bool near(double a, double b, double eps = 1e-9) { return std::fabs(a - b) < eps; }
bool nearf(float a, float b, float eps = 1e-5F) { return std::fabs(a - b) < eps; }

/// Three tiles 0, 1, 2 that stack like a staircase: sideways only equal tiles touch, upwards a tile may carry itself or
/// the next one (b is a or a + 1).
bool stairFits(int a, int dir, int b) {
    if (dir == PosY) return b == a || b == a + 1;
    if (dir == NegY) return a == b || a == b + 1;
    return a == b;
}

TileSet stairSetWith(TileSet::EdgeFn edge, std::vector<double> weights) {
    TileSet s;
    const bool ok = s.build(std::move(weights), stairFits, edge);
    assert(ok);
    (void)ok;
    return s;
}
TileSet stairSet() { return stairSetWith([](int, int) { return true; }, {1.0, 1.0, 1.0}); }
TileSet stairSet(TileSet::EdgeFn edge) { return stairSetWith(std::move(edge), {1.0, 1.0, 1.0}); }
TileSet stairSetW(std::vector<double> weights) { return stairSetWith([](int, int) { return true; }, std::move(weights)); }

// ---------------------------------------------------------------------------------------------- the solver

void directions() {
    // Six neighbours, in pairs, and the opposite is the same index with the lowest bit flipped.
    assert(kOffsets.size() == 6);
    for (int d = 0; d < kDirCount; ++d) {
        const Int3 a = kOffsets[static_cast<std::size_t>(d)];
        const Int3 b = kOffsets[static_cast<std::size_t>(opposite(d))];
        assert(a.x == -b.x && a.y == -b.y && a.z == -b.z);
        assert(std::abs(a.x) + std::abs(a.y) + std::abs(a.z) == 1);
        assert(opposite(opposite(d)) == d);
    }
    assert(opposite(PosX) == NegX && opposite(PosY) == NegY && opposite(NegZ) == PosZ);
    assert(isHorizontal(PosX) && isHorizontal(NegZ) && !isHorizontal(PosY) && !isHorizontal(NegY));
}

void compatTable() {
    const TileSet s = stairSet();
    assert(s.size() == 3 && s.all == 0b111);
    assert(s.fits(PosX, 1) == 0b010);  // sideways: only the same tile
    assert(s.fits(NegZ, 2) == 0b100);
    assert(s.fits(PosY, 0) == 0b011);  // above 0: 0 or 1
    assert(s.fits(PosY, 2) == 0b100);  // above 2: only 2 (there is no tile 3)
    assert(s.fits(NegY, 1) == 0b011);  // below 1: 0 or 1
    assert(s.fits(NegY, 0) == 0b001);
    assert(s.symmetric());
    // A rule that only works one way round is not symmetric.
    TileSet bad;
    const bool ok = bad.build({1.0, 1.0}, [](int a, int dir, int b) { return dir == PosX ? b >= a : b == a; }, [](int, int) { return true; });
    assert(ok && !bad.symmetric());
}

void buildRejectsBadSets() {
    TileSet s;
    const TileSet::FitFn fit = [](int, int, int) { return true; };
    const TileSet::EdgeFn edge = [](int, int) { return true; };
    assert(!s.build({}, fit, edge));
    assert(!s.build({0.0}, fit, edge));
    assert(!s.build({1.0, -1.0}, fit, edge));
    assert(!s.build(std::vector<double>(kMaxTiles + 1, 1.0), fit, edge));
    assert(s.build(std::vector<double>(kMaxTiles, 1.0), fit, edge));
    assert(s.all == ~Mask{0});
}

void entropyValues() {
    const TileSet s = stairSet();
    assert(near(entropy(s, 0), 0.0));
    assert(near(entropy(s, 0b100), 0.0));  // one tile: nothing left to decide
    assert(near(entropy(s, 0b011), std::log(2.0)));
    assert(near(entropy(s, 0b111), std::log(3.0)));
    // Weights 3 and 1: ln 4 - 3 ln 3 / 4 = 0.5623351446
    TileSet w;
    assert(w.build({3.0, 1.0}, stairFits, [](int, int) { return true; }));
    assert(near(entropy(w, 0b11), 0.5623351446, 1e-9));
    TileSet w2;
    assert(w2.build({30.0, 10.0}, stairFits, [](int, int) { return true; }));
    assert(near(entropy(w2, 0b11), entropy(w, 0b11)));  // scaling all weights changes nothing
}

void solidShareValues() {
    TileSet s = stairSetW({3.0, 1.0, 4.0});
    s.solid = {0, 1, 1};
    assert(near(solidShare(s, 0b111), 5.0 / 8.0));
    assert(near(solidShare(s, 0b001), 0.0));
    assert(near(solidShare(s, 0b110), 1.0));
    assert(near(solidShare(s, 0), 0.0));
}

void weightedPick() {
    TileSet w;
    assert(w.build({3.0, 1.0}, stairFits, [](int, int) { return true; }));
    assert(pickWeighted(w, 0b11, 0.0) == 0);
    assert(pickWeighted(w, 0b11, 0.7499) == 0);
    assert(pickWeighted(w, 0b11, 0.75) == 1);  // 0.75 * 4 = 3 = the end of the first tile
    assert(pickWeighted(w, 0b11, 1.0) == 1);   // rounding at the top end still lands on a tile
    assert(pickWeighted(w, 0b10, 0.0) == 1);
    assert(pickWeighted(w, 0, 0.5) == -1);
    TileSet t;
    assert(t.build({3.0, 1.0, 1.0}, stairFits, [](int, int) { return true; }));
    assert(pickWeighted(t, 0b110, 0.49) == 1);  // with the first tile banned the weights 1 and 1 count
    assert(pickWeighted(t, 0b110, 0.5) == 2);
}

void rngRange() {
    Rng a(42), b(42), c(43);
    bool differs = false;
    for (int i = 0; i < 1000; ++i) {
        const double x = a.uniform();
        assert(x >= 0.0 && x < 1.0);
        assert(x == b.uniform());
        differs = differs || x != c.uniform();
    }
    assert(differs);
}

void indexing() {
    const TileSet s = stairSet();
    Solver v(s, 3, 2, 4, 1, Order::Layers, Recovery::Restart);
    assert(v.cells() == 24);
    assert(v.index(0, 0, 0) == 0 && v.index(2, 0, 0) == 2 && v.index(0, 0, 1) == 3 && v.index(0, 1, 0) == 12 && v.index(2, 1, 3) == 23);
    for (int i = 0; i < v.cells(); ++i) {
        const Int3 c = v.coords(i);
        assert(v.index(c.x, c.y, c.z) == i);
        assert(v.inside(c.x, c.y, c.z));
    }
    assert(!v.inside(-1, 0, 0) && !v.inside(3, 0, 0) && !v.inside(0, 2, 0) && !v.inside(0, 0, 4) && !v.inside(0, -1, 0));
}

void propagationUpAndDown() {
    const TileSet s = stairSet();
    // A column of three cells, nothing decided. The middle cell is forced to be tile 2.
    Solver v(s, 1, 3, 1, 1, Order::Layers, Recovery::Restart);
    assert(v.domain == std::vector<Mask>({0b111, 0b111, 0b111}));
    v.domain[1] = 0b100;
    v.queue.push_back(1);
    v.queued[1] = 1;
    // Pop of the middle cell narrows both neighbours, then the two pops find nothing more to change.
    v.step();
    assert(v.domain == std::vector<Mask>({0b110, 0b100, 0b100}));  // below tile 2 sits 1 or 2; above it only 2
    assert(v.queue.size() == 2);
    v.step();
    v.step();
    assert(v.queue.empty());
    assert(v.counters.steps == 3);
    assert(v.status == Status::Running);
    assert(v.counters.contradictions == 0);
}

void borderRules() {
    // Tile 0 may stand on the ground only, tiles 1 and 2 may be at the top. Sideways everything may touch the border.
    const TileSet s = stairSet([](int a, int dir) { return dir == NegY ? a <= 1 : dir == PosY ? a >= 1 : true; });
    Solver v(s, 1, 2, 1, 1, Order::Layers, Recovery::Restart);
    // Bottom {0,1}, top {1,2}; then propagation: above 0 or 1 may be 0,1,2 -> {1,2} stays; below {1,2} may be 0,1,2 -> {0,1} stays.
    assert(v.domain == std::vector<Mask>({0b011, 0b110}));
    assert(v.status == Status::Running);

    // Only tile 1 may touch the +x border: the east cell is 1, and so is its western neighbour (sideways only equal tiles).
    const TileSet e = stairSet([](int a, int dir) { return dir == PosX ? a == 1 : true; });
    Solver w(e, 2, 1, 1, 1, Order::Layers, Recovery::Restart);
    assert(w.domain == std::vector<Mask>({0b010, 0b010}));
    assert(w.step() == Status::Done);
    assert(w.valid());
}

void impossibleSetGivesUp() {
    // Tile 0 on the ground and tile 2 at the top never fit: 2 may not sit on 0 (b == a or a + 1 only).
    const TileSet s = stairSet([](int a, int dir) { return dir == NegY ? a == 0 : dir == PosY ? a == 2 : true; });
    Solver v(s, 1, 2, 1, 1, Order::Entropy, Recovery::Restart);
    assert(v.status == Status::Contradiction);
    assert(v.counters.contradictions == 1);
    const long taken = v.run(v.maxSteps());
    // 40 restarts, each one contradicts again; the 41st recovery gives up.
    assert(v.status == Status::GaveUp);
    assert(taken == 41 && v.counters.steps == 41);
    assert(v.counters.restarts == 40);
    assert(v.counters.contradictions == 41);
    assert(taken <= v.maxSteps());
}

void collapseAndClick() {
    const TileSet s = stairSet();
    Solver v(s, 2, 1, 2, 5, Order::Entropy, Recovery::Restart);
    assert(v.collapseCell(0));
    assert(count(v.at(0)) == 1);
    assert(!v.collapseCell(0));   // already decided
    assert(!v.collapseCell(-1));
    assert(!v.collapseCell(99));
    v.run(v.maxSteps());
    assert(v.status == Status::Done && v.valid());
    assert(!v.collapseCell(1));  // not while finished
}

void pickOrder() {
    const TileSet s = stairSetW({5.0, 1.0, 1.0});
    // Cells 0 and 1 side by side. Cell 0 is {0,1} (two tiles, weights 5 and 1), cell 1 is {1,2} (weights 1 and 1).
    Solver v(s, 2, 1, 1, 3, Order::Entropy, Recovery::Restart);
    v.domain = {0b011, 0b110};
    assert(entropy(s, 0b011) < entropy(s, 0b110));
    assert(v.pickCell() == 0);  // the unevenly weighted cell is nearly decided already
    v.domain[0] = 0b001;        // decided cells are never picked
    assert(v.pickCell() == 1);
    v.domain[1] = 0b010;
    assert(v.pickCell() == -1);

    // Layers: a high cell with low entropy still comes after a low cell with high entropy.
    Solver l(s, 1, 2, 1, 3, Order::Layers, Recovery::Restart);
    l.domain = {0b111, 0b011};
    assert(l.pickCell() == 0);
    Solver e(s, 1, 2, 1, 3, Order::Entropy, Recovery::Restart);
    e.domain = {0b111, 0b011};
    assert(e.pickCell() == 1);
}

void touchedAndQueueConsistency() {
    // The snapshot has to carry the queue: after every step the flags say exactly what the queue holds.
    const CastleSet c = makeCastle();
    Solver v(c.set, 8, 5, 8, 11, Order::Random, Recovery::Backtrack);
    v.trackTouched = true;
    long taken = 0;
    while (!v.finished() && taken < v.maxSteps()) {
        v.step();
        ++taken;
        std::vector<char> expect(v.queued.size(), 0);
        for (int cell : v.queue) {
            assert(expect[static_cast<std::size_t>(cell)] == 0);  // no cell twice
            expect[static_cast<std::size_t>(cell)] = 1;
        }
        assert(expect == v.queued);
    }
    assert(v.finished());
    assert(!v.touched.empty());
    // Whatever the final state, every touched index is a real cell.
    for (int cell : v.touched) assert(cell >= 0 && cell < v.cells());
}

// ---------------------------------------------------------------------------------------------- the castle

void castleTileSet() {
    const CastleSet c = makeCastle();
    assert(c.tiles.size() == 34 && c.set.size() == 34);
    assert(c.set.symmetric());
    assert(c.find(Kind::House) >= 0 && c.find(Kind::HouseUpper) >= 0 && c.find(Kind::Post, 0b0101) >= 0 && c.find(Kind::Tower, 15) >= 0 && c.find(Kind::Tower, 16) == -1);
    // A wall with a single arm is a dead end: not in the set, unless asked for.
    assert(c.find(Kind::Post, 0b0001) == -1 && c.find(Kind::Tower, 0b1000) == -1);
    Weights open;
    open.deadEnds = true;
    assert(makeCastle(open).tiles.size() == 42);
    int solid = 0;
    for (int t = 0; t < c.set.size(); ++t) solid += c.set.isSolid(t) ? 1 : 0;
    assert(solid == 31);  // 34 minus the three kinds of air
    for (int t = 0; t < c.set.size(); ++t) assert(c.set.weight[static_cast<std::size_t>(t)] > 0.0);
}

Mask maskOf(const CastleSet& c, Kind kind) {
    Mask m = 0;
    for (int t = 0; t < c.set.size(); ++t) {
        if (c.tile(t).kind == kind) m |= bit(t);
    }
    return m;
}

void castleSidewaysRules() {
    const CastleSet c = makeCastle();
    const int straight = c.find(Kind::Post, 0b0011);  // arms towards +x and -x
    // East of it: a tile with an arm towards -x (bit 1). That is a pillar or a tower with that bit set: 7 patterns each
    // (8 have the bit, one of them is the dead end that is not in the set); no house, no air.
    const Mask east = c.set.fits(PosX, straight);
    int armed = 0;
    for (int t = 0; t < c.set.size(); ++t) {
        const bool inSet = (east >> t & 1) != 0;
        const bool wants = hasArms(c.tile(t).kind) && (c.tile(t).arms & 0b0010) != 0;
        assert(inSet == wants);
        armed += inSet ? 1 : 0;
    }
    assert(armed == 14);
    assert((east & maskOf(c, Kind::House)) == 0);
    // North of it (-z) there is no arm, so anything without an arm towards +z fits, and nothing with one.
    const Mask north = c.set.fits(NegZ, straight);
    assert((north >> c.find(Kind::House) & 1) != 0);
    assert((north >> c.find(Kind::AirSky) & 1) != 0);
    assert((north >> c.find(Kind::Post, 0b0000) & 1) != 0);
    assert((north >> c.find(Kind::Post, 0b0011) & 1) != 0);
    assert((north >> c.find(Kind::Post, 0b0101) & 1) == 0);  // an arm towards +z would poke into the arm-less side
    // A pillar without arms has a house or a tree next to it on every side.
    const int lone = c.find(Kind::Post, 0);
    for (int d : {PosX, NegX, PosZ, NegZ}) {
        assert((c.set.fits(d, lone) >> c.find(Kind::Tree) & 1) != 0);
        assert((c.set.fits(d, lone) >> c.find(Kind::House) & 1) != 0);
    }
}

void castleVerticalRules() {
    const CastleSet c = makeCastle();
    // Above a house, on the ground floor or higher: another floor or one of the two roofs.
    const Mask floorOrRoof = maskOf(c, Kind::HouseUpper) | maskOf(c, Kind::RoofX) | maskOf(c, Kind::RoofZ);
    assert(c.set.fits(PosY, c.find(Kind::House)) == floorOrRoof);
    assert(c.set.fits(PosY, c.find(Kind::HouseUpper)) == floorOrRoof);
    // Below an upper floor: any house. Below the ground floor: nothing (it stands on the ground).
    assert(c.set.fits(NegY, c.find(Kind::HouseUpper)) == (maskOf(c, Kind::House) | maskOf(c, Kind::HouseUpper)));
    assert(c.set.fits(NegY, c.find(Kind::House)) == 0);
    // Above any air there is only air that sits over air.
    assert(c.set.fits(PosY, c.find(Kind::AirGround)) == maskOf(c, Kind::AirSky));
    assert(c.set.fits(PosY, c.find(Kind::AirBuilt)) == maskOf(c, Kind::AirSky));
    assert(c.set.fits(PosY, c.find(Kind::AirSky)) == maskOf(c, Kind::AirSky));
    // A pillar carries a pillar (12 patterns) or lets the air in, a tower carries a tower or the roof.
    assert(count(c.set.fits(PosY, c.find(Kind::Post, 0))) == 13);
    assert(count(c.set.fits(PosY, c.find(Kind::Tower, 5))) == 14);
    assert((c.set.fits(PosY, c.find(Kind::Tower, 5)) >> c.find(Kind::TowerRoof) & 1) != 0);
    assert((c.set.fits(PosY, c.find(Kind::Post, 5)) >> c.find(Kind::TowerRoof) & 1) == 0);
    // Below: the roof of a tower sits on a tower and on nothing else.
    assert(c.set.fits(NegY, c.find(Kind::TowerRoof)) == maskOf(c, Kind::Tower));
    // Below a tree: only the ground, which is not a tile, so nothing; the tree is only legal in the lowest layer.
    assert(c.set.fits(NegY, c.find(Kind::Tree)) == 0);
    // Pillars stand on pillars: the pillar below can have any arm pattern.
    assert(c.set.fits(NegY, c.find(Kind::Post, 3)) == maskOf(c, Kind::Post));
}

void castleBorderRules() {
    const CastleSet c = makeCastle();
    // Sides: no arm may point out of the grid.
    for (int d : {PosX, NegX, PosZ, NegZ}) {
        for (int t = 0; t < c.set.size(); ++t) assert(((c.set.edgeOk[static_cast<std::size_t>(d)] >> t & 1) != 0) == !armTowards(c.tile(t), d));
    }
    // Ground: 28 tiles can stand there (lawn, 2 trees, house, 12 pillars, 12 towers). Air over air cannot.
    assert(count(c.set.edgeOk[NegY]) == 28);
    assert((c.set.edgeOk[NegY] >> c.find(Kind::AirSky) & 1) == 0);
    assert((c.set.edgeOk[NegY] >> c.find(Kind::RoofX) & 1) == 0);
    // Top: everything that does not need something above it; a house, on any floor, does.
    assert(count(c.set.edgeOk[PosY]) == 32);
    assert((c.set.edgeOk[PosY] >> c.find(Kind::House) & 1) == 0);
    assert((c.set.edgeOk[PosY] >> c.find(Kind::HouseUpper) & 1) == 0);
    assert((c.set.edgeOk[PosY] >> c.find(Kind::RoofZ) & 1) != 0);
}

/// Checks a finished world against the rules themselves (castleFits, castleEdge), not against the tables the solver used.
bool worldIsLegal(const CastleSet& c, const Solver& s) {
    for (int i = 0; i < s.cells(); ++i) {
        if (count(s.at(i)) != 1) return false;
        const CastleTile& a = c.tile(lowest(s.at(i)));
        const Int3 p = s.coords(i);
        for (int d = 0; d < kDirCount; ++d) {
            const Int3 o = kOffsets[static_cast<std::size_t>(d)];
            if (!s.inside(p.x + o.x, p.y + o.y, p.z + o.z)) {
                if (!castleEdge(a, d)) return false;
                continue;
            }
            const CastleTile& b = c.tile(lowest(s.at(s.index(p.x + o.x, p.y + o.y, p.z + o.z))));
            if (!castleFits(a, d, b)) return false;
        }
    }
    return true;
}

void castleWorldsAreLegal() {
    const CastleSet c = makeCastle();
    for (std::uint64_t seed = 1; seed <= 6; ++seed) {
        Solver s(c.set, 12, 6, 12, seed, Order::Entropy, Recovery::Backtrack);
        s.run(s.maxSteps());
        assert(s.status == Status::Done);
        assert(s.valid());
        assert(worldIsLegal(c, s));
        // The pieces really are where the rules say: a house has a house or a roof above it, never air.
        for (int i = 0; i < s.cells(); ++i) {
            const Int3 p = s.coords(i);
            const Kind k = c.tile(lowest(s.at(i))).kind;
            if (k == Kind::House || k == Kind::HouseUpper) {
                assert(p.y + 1 < s.height);
                const Kind up = c.tile(lowest(s.at(s.index(p.x, p.y + 1, p.z)))).kind;
                assert(up == Kind::HouseUpper || up == Kind::RoofX || up == Kind::RoofZ);
            }
            if (k == Kind::HouseUpper) {
                const Kind down = c.tile(lowest(s.at(s.index(p.x, p.y - 1, p.z)))).kind;
                assert(down == Kind::House || down == Kind::HouseUpper);
            }
            if (p.y == 0) assert(standsOnGround(k));
        }
    }
}

void castleBenchmark() {
    const CastleSet c = makeCastle();
    // 10 worlds of the demo's size with every order and both recoveries. None may give up or break a rule.
    for (const Order order : {Order::Entropy, Order::Layers, Order::Random}) {
        for (const Recovery rec : {Recovery::Restart, Recovery::Backtrack}) {
            const Bench b = benchmark(c.set, 12, 6, 12, order, rec, 10, 100);
            assert(b.worlds == 10);
            assert(b.gaveUp == 0);
            assert(b.invalid == 0);
            const double fill = static_cast<double>(b.solidCells) / static_cast<double>(b.cells);
            std::printf("  order %d recovery %d: %d conflicts, %ld steps, %.1f%% solid\n", static_cast<int>(order), static_cast<int>(rec), b.contradictions, b.steps, fill * 100.0);
            std::fflush(stdout);
            assert(fill > 0.10);  // not an empty lawn
            if (order != Order::Random) assert(fill < 0.60);  // not a solid block either (random order has no sense of density)
        }
    }
}

// ---------------------------------------------------------------------------------------------- time and numbers

void stepClock() {
    StepClock c;
    assert(c.take(0.25, 10.0, 100) == 2);  // 2.5 steps: two now, half a step stays
    assert(near(c.acc, 0.5));
    assert(c.take(0.05, 10.0, 100) == 1);  // 0.5 + 0.5 = 1
    assert(near(c.acc, 0.0));
    assert(c.take(10.0, 10.0, 100) == 2);  // a stalled frame is clamped to 0.25 s, not 100 steps
    StepClock d;
    assert(d.take(0.1, 1000.0, 7) == 7);   // cap per frame
    assert(d.acc < 1.0);                   // what the frame could not afford is dropped, not saved up
    assert(d.take(0.0, 10.0, 7) == 0);
    assert(d.take(0.1, 0.0, 7) == 0);
    assert(d.take(-1.0, 10.0, 7) == 0);
    assert(d.take(0.1, 10.0, 0) == 0);
    StepClock e;
    e.take(0.05, 10.0, 100);
    e.reset();
    assert(near(e.acc, 0.0));
}

void smallHelpers() {
    assert(nearf(decay(1.0F, 3.0F, 0.0F), 1.0F));
    assert(nearf(decay(1.0F, 3.0F, 0.1F), std::exp(-0.3F)));
    // Two half frames equal one whole frame.
    assert(nearf(decay(decay(1.0F, 3.0F, 0.05F), 3.0F, 0.05F), decay(1.0F, 3.0F, 0.1F)));
    assert(nearf(decay(1.0F, 3.0F, 99.0F), std::exp(-0.75F)));  // clamped to a quarter second

    assert(nearf(easeOutBack(0.0F), 0.0F));
    assert(nearf(easeOutBack(1.0F), 1.0F));
    assert(easeOutBack(0.8F) > 1.0F);  // overshoots on the way, then settles
    assert(nearf(easeOutBack(-3.0F), 0.0F) && nearf(easeOutBack(7.0F), 1.0F));

    assert(groupDigits(0) == "0");
    assert(groupDigits(999) == "999");
    assert(groupDigits(1000) == "1,000");
    assert(groupDigits(1234567) == "1,234,567");
    assert(groupDigits(-12345) == "-12,345");
}

// ---------------------------------------------------------------------------------------------- camera and picking

void matrices() {
    // lookAt: the eye goes to the origin, the target straight ahead (-z) at its distance.
    const Vec3 eye{3.0F, 4.0F, 5.0F}, target{0.0F, 1.0F, 0.0F};
    const Mat4 view = lookAt(eye, target, {0.0F, 1.0F, 0.0F});
    const Vec4 e = transform(view, eye);
    assert(nearf(e.x, 0.0F) && nearf(e.y, 0.0F) && nearf(e.z, 0.0F));
    const Vec4 t = transform(view, target);
    assert(nearf(t.x, 0.0F, 1e-4F) && nearf(t.y, 0.0F, 1e-4F) && nearf(t.z, -length(target - eye), 1e-4F));

    // perspective: the near plane maps to -1, the far plane to +1, on the axis.
    const Mat4 p = perspective(0.8F, 1.5F, 1.0F, 50.0F);
    const Vec4 n = transform(p, {0.0F, 0.0F, -1.0F});
    const Vec4 f = transform(p, {0.0F, 0.0F, -50.0F});
    assert(nearf(n.z / n.w, -1.0F, 1e-4F) && nearf(f.z / f.w, 1.0F, 1e-4F));

    // ortho: the box corners map to the cube corners.
    const Mat4 o = ortho(10.0F, 5.0F, 1.0F, 31.0F);
    const Vec4 c1 = transform(o, {10.0F, 5.0F, -1.0F});
    const Vec4 c2 = transform(o, {-10.0F, -5.0F, -31.0F});
    assert(nearf(c1.x, 1.0F) && nearf(c1.y, 1.0F) && nearf(c1.z, -1.0F));
    assert(nearf(c2.x, -1.0F) && nearf(c2.y, -1.0F) && nearf(c2.z, 1.0F));

    // The zero vector stays zero instead of becoming NaN.
    const Vec3 z = normalize({0.0F, 0.0F, 0.0F});
    assert(z.x == 0.0F && z.y == 0.0F && z.z == 0.0F);
}

void orbitCamera() {
    OrbitCamera cam;
    assert(nearf(length(cam.eye() - cam.target), cam.distance, 1e-4F));
    const Basis b = cam.basis();
    assert(nearf(length(b.right), 1.0F) && nearf(length(b.up), 1.0F) && nearf(length(b.forward), 1.0F));
    assert(nearf(dot(b.right, b.up), 0.0F) && nearf(dot(b.right, b.forward), 0.0F) && nearf(dot(b.up, b.forward), 0.0F));
    cam.orbit(0.0F, 100.0F);
    assert(nearf(cam.pitch, kMaxPitch));
    cam.orbit(0.0F, -100.0F);
    assert(nearf(cam.pitch, kMinPitch));
    cam.zoom(100.0F);
    assert(nearf(cam.distance, kMinDistance));
    cam.zoom(-100.0F);
    assert(nearf(cam.distance, kMaxDistance));
    cam.yaw = 0.0F;
    cam.pitch = 0.0F;
    cam.distance = 10.0F;
    cam.target = {0.0F, 0.0F, 0.0F};
    assert(nearf(cam.eye().x, 0.0F) && nearf(cam.eye().y, 0.0F) && nearf(cam.eye().z, 10.0F));  // yaw 0 looks along -z
}

void rays() {
    OrbitCamera cam;
    cam.yaw = 0.0F;
    cam.pitch = 0.0F;
    cam.distance = 10.0F;
    cam.target = {0.0F, 0.0F, 0.0F};
    const Ray centre = rayThrough(cam, 0.5F, 0.5F, 16.0F / 9.0F);
    assert(nearf(centre.dir.x, 0.0F) && nearf(centre.dir.y, 0.0F) && nearf(centre.dir.z, -1.0F));
    // The right edge, vertically centred: the angle to the axis has tan = tan(fov/2) * aspect.
    const float aspect = 16.0F / 9.0F;
    const Ray right = rayThrough(cam, 1.0F, 0.5F, aspect);
    const float tr = std::tan(kFovY / 2.0F) * aspect;
    assert(nearf(right.dir.x, tr / std::sqrt(1.0F + tr * tr), 1e-5F));
    assert(right.dir.x > 0.0F);
    const Ray top = rayThrough(cam, 0.5F, 0.0F, aspect);  // v = 0 is the top of the canvas
    assert(top.dir.y > 0.0F && nearf(top.dir.x, 0.0F));

    // Round trip with the shifted picture: project a world point to the canvas, shoot the ray through that canvas point,
    // and the ray has to pass through the point again. Without the shift parameter it would miss by about half a unit.
    OrbitCamera other;
    other.yaw = 1.1F;
    other.pitch = 0.7F;
    other.distance = 20.0F;
    const Vec3 world{2.0F, 1.0F, -3.0F};
    const Vec4 clip = transform(shiftedPerspective(kFovY, aspect, kNear, kFar, kViewShift) * other.view(), world);
    const float u = (clip.x / clip.w + 1.0F) / 2.0F;
    const float v = (1.0F - clip.y / clip.w) / 2.0F;
    const auto missBy = [&](float shift) {
        const Ray ray = rayThrough(other, u, v, aspect, shift);
        return length(cross(ray.dir, world - ray.origin));  // distance of the point from the ray (dir has unit length)
    };
    assert(missBy(kViewShift) < 1e-3F);
    assert(missBy(0.0F) > 0.3F);
    // The shift moves the picture up: the target, which sat in the middle, now appears above the middle of the canvas.
    const Vec4 centre4 = transform(shiftedPerspective(kFovY, aspect, kNear, kFar, kViewShift) * other.view(), other.target);
    assert(nearf(centre4.y / centre4.w, kViewShift, 1e-4F));
}

void rayBoxCases() {
    const Vec3 lo{0.0F, 0.0F, 0.0F}, hi{1.0F, 1.0F, 1.0F};
    float t = -1.0F;
    assert(rayBox({{-5.0F, 0.5F, 0.5F}, {1.0F, 0.0F, 0.0F}}, lo, hi, t) && nearf(t, 5.0F));
    assert(!rayBox({{-5.0F, 0.5F, 0.5F}, {-1.0F, 0.0F, 0.0F}}, lo, hi, t));  // pointing away
    assert(!rayBox({{-5.0F, 2.0F, 0.5F}, {1.0F, 0.0F, 0.0F}}, lo, hi, t));   // parallel to the slab y, above it
    assert(!rayBox({{-5.0F, 0.5F, 3.0F}, {1.0F, 0.0F, 0.0F}}, lo, hi, t));   // parallel to the slab z, beside it
    assert(rayBox({{0.5F, 0.5F, 0.5F}, {0.0F, 1.0F, 0.0F}}, lo, hi, t) && nearf(t, 0.0F));  // origin inside
    assert(rayBox({{0.5F, 5.0F, 0.5F}, {0.0F, -1.0F, 0.0F}}, lo, hi, t) && nearf(t, 4.0F));
    // A diagonal ray that passes the box at the corner of the slabs misses.
    const Vec3 d = normalize({1.0F, 1.0F, 0.0F});
    assert(!rayBox({{-3.0F, 0.5F, 0.5F}, d}, lo, hi, t));  // reaches y = 3.5 when x = 0
    assert(rayBox({{-0.5F, -0.25F, 0.5F}, d}, lo, hi, t) && nearf(t, 0.5F * std::sqrt(2.0F), 1e-5F));
}

void picking() {
    const TileSet s = stairSet();
    // 2x1x2 cells: cell (x, z) occupies x in [x - 1, x], z in [z - 1, z] (the grid is centred on the origin).
    Solver v(s, 2, 1, 2, 1, Order::Entropy, Recovery::Restart);
    // With everything undecided, every cell can be hit.
    assert(v.pickCell() >= 0);
    assert(pickUndecided(v, {{-0.5F, 5.0F, -0.5F}, {0.0F, -1.0F, 0.0F}}) == v.index(0, 0, 0));
    assert(pickUndecided(v, {{0.5F, 5.0F, 0.5F}, {0.0F, -1.0F, 0.0F}}) == v.index(1, 0, 1));
    assert(pickUndecided(v, {{0.5F, 5.0F, 5.0F}, {0.0F, -1.0F, 0.0F}}) == -1);  // beside the grid
    // From the side, along x at z = -0.5: the near cell wins.
    assert(pickUndecided(v, {{-5.0F, 0.5F, -0.5F}, {1.0F, 0.0F, 0.0F}}) == v.index(0, 0, 0));
    v.domain[static_cast<std::size_t>(v.index(0, 0, 0))] = 0b001;  // decided cells are skipped
    assert(pickUndecided(v, {{-5.0F, 0.5F, -0.5F}, {1.0F, 0.0F, 0.0F}}) == v.index(1, 0, 0));
    assert(pickUndecided(v, {{-0.5F, 5.0F, -0.5F}, {0.0F, -1.0F, 0.0F}}) == -1);
}

void worldSpace() {
    const Vec3 lo = cellMin(12, 12, 0, 0, 0);
    assert(nearf(lo.x, -6.0F) && nearf(lo.y, 0.0F) && nearf(lo.z, -6.0F));
    const Vec3 hi = cellMin(12, 12, 11, 5, 11);
    assert(nearf(hi.x, 5.0F) && nearf(hi.y, 5.0F));
    const Vec3 base = cellBase(12, 12, 0, 0, 0);
    assert(nearf(base.x, -5.5F) && nearf(base.z, -5.5F) && nearf(base.y, 0.0F));
    // An odd size is centred too: cells -1.5 .. 1.5, the middle cell sits on the origin.
    const Vec3 mid = cellBase(3, 3, 1, 0, 1);
    assert(nearf(mid.x, 0.0F) && nearf(mid.z, 0.0F));
}

// ---------------------------------------------------------------------------------------------- chips

void chips() {
    const ChipLayout l = chipLayout();
    assert(l.chips.size() == static_cast<std::size_t>(Chip::Count));  // every chip exactly once
    std::set<int> ids;
    for (const ChipRect& c : l.chips) {
        ids.insert(static_cast<int>(c.id));
        assert(c.x >= 0.0F && c.x + c.w <= kCanvasWidth - 8.0F);
        assert(c.y >= l.panelTop && c.y + c.h <= kCanvasHeight);
        assert(c.w >= chipWidth("") && c.h == kChipHeight);
    }
    assert(ids.size() == l.chips.size());
    // No two chips overlap, not even by a pixel.
    for (std::size_t i = 0; i < l.chips.size(); ++i) {
        for (std::size_t j = i + 1; j < l.chips.size(); ++j) {
            const ChipRect &a = l.chips[i], &b = l.chips[j];
            const bool apart = a.x + a.w <= b.x || b.x + b.w <= a.x || a.y + a.h <= b.y || b.y + b.h <= a.y;
            assert(apart);
        }
    }
    // The label of a group ends before the first chip to its right in the same row.
    for (const GroupLabel& g : l.labels) {
        const float end = g.x + static_cast<float>(std::char_traits<char>::length(g.text)) * kCharWidth;
        float nextX = 1e9F;
        for (const ChipRect& c : l.chips) {
            if (c.y == g.y - 4.0F && c.x >= g.x) nextX = std::min(nextX, c.x);
        }
        assert(nextX < 1e9F && end + 4.0F <= nextX);
    }
    // Hit test: inside, on the edge (left/top inclusive, right/bottom exclusive) and outside.
    const ChipRect first = l.chips.front();
    assert(hitChip(l.chips, first.x + 1.0F, first.y + 1.0F) == &l.chips.front());
    assert(hitChip(l.chips, first.x, first.y) == &l.chips.front());
    assert(hitChip(l.chips, first.x + first.w, first.y + first.h) != &l.chips.front());
    assert(hitChip(l.chips, 3.0F, 3.0F) == nullptr);
    assert(chipWidth("new") == 3.0F * kCharWidth + 2.0F * kChipPad);
}

// ---------------------------------------------------------------------------------------------- the pictures

struct Box {
    float lo[3] = {1e9F, 1e9F, 1e9F};
    float hi[3] = {-1e9F, -1e9F, -1e9F};
    void add(const Vertex& v) {
        const float p[3] = {v.px, v.py, v.pz};
        for (int i = 0; i < 3; ++i) {
            lo[i] = std::min(lo[i], p[i]);
            hi[i] = std::max(hi[i], p[i]);
        }
    }
};

Box bounds(const Verts& v) {
    Box b;
    for (const Vertex& p : v) b.add(p);
    return b;
}

void meshesAreWellFormed() {
    const CastleSet c = makeCastle();
    for (int t = 0; t < c.set.size(); ++t) {
        const CastleTile& tile = c.tile(t);
        const Verts v = buildTile(tile);
        assert(v.size() % 3 == 0);
        assert(v.empty() == isAir(tile.kind));  // air has nothing to draw, everything else has
        for (const Vertex& p : v) {
            assert(std::isfinite(p.px) && std::isfinite(p.py) && std::isfinite(p.pz));
            // Inside the cell, with a hair of tolerance for the float math.
            assert(p.px >= -0.5F - 1e-5F && p.px <= 0.5F + 1e-5F);
            assert(p.pz >= -0.5F - 1e-5F && p.pz <= 0.5F + 1e-5F);
            assert(p.py >= -1e-5F && p.py <= 1.0F + 1e-5F);
            assert(nearf(length({p.nx, p.ny, p.nz}), 1.0F, 1e-4F));
            assert(p.r >= 0.0F && p.r <= 1.0F && p.g >= 0.0F && p.g <= 1.0F && p.b >= 0.0F && p.b <= 1.0F);
        }
        // The stored normal is the one of the triangle as it is wound, and every triangle has area.
        for (std::size_t i = 0; i + 2 < v.size(); i += 3) {
            const Vec3 a{v[i].px, v[i].py, v[i].pz}, b{v[i + 1].px, v[i + 1].py, v[i + 1].pz}, d{v[i + 2].px, v[i + 2].py, v[i + 2].pz};
            const Vec3 n = cross(b - a, d - a);
            assert(length(n) > 1e-8F);
            assert(dot(normalize(n), {v[i].nx, v[i].ny, v[i].nz}) > 0.999F);
        }
        // The battlements may rise 0.12 above the cell and nowhere else.
        const Verts cap = buildCap(tile);
        assert(cap.empty() == !hasArms(tile.kind));
        for (const Vertex& p : cap) {
            assert(p.px >= -0.5F - 1e-5F && p.px <= 0.5F + 1e-5F && p.pz >= -0.5F - 1e-5F && p.pz <= 0.5F + 1e-5F);
            assert(p.py >= 1.0F - 1e-5F && p.py <= 1.0F + kMerlonHeight + 1e-5F);
        }
    }
}

void armsMeetOnTheFace() {
    const CastleSet c = makeCastle();
    // An arm towards a direction reaches exactly the face of the cell (0.5), and no arm means nothing gets there.
    for (const Kind kind : {Kind::Post, Kind::Tower}) {
        for (int arms = 0; arms < 16; ++arms) {
            const Verts v = buildTile({kind, arms});
            float reach[4] = {0.0F, 0.0F, 0.0F, 0.0F};  // how far the mesh reaches towards +x, -x, +z, -z
            for (const Vertex& p : v) {
                reach[0] = std::max(reach[0], p.px);
                reach[1] = std::max(reach[1], -p.px);
                reach[2] = std::max(reach[2], p.pz);
                reach[3] = std::max(reach[3], -p.pz);
            }
            for (int i = 0; i < 4; ++i) {
                const bool arm = (arms >> i & 1) != 0;
                if (arm) assert(nearf(reach[i], 0.5F));
                else assert(reach[i] < 0.46F);
            }
        }
    }
    // The pieces of tiles that fit sideways meet: both sides have an arm of the same width at the face.
    assert(kArmHalf > 0.0F && kArmHalf < kPostHalf);
}

void roofsTurn() {
    const Verts x = buildTile({Kind::RoofX, 0});
    const Verts z = buildTile({Kind::RoofZ, 0});
    assert(x.size() == z.size() && !x.empty());
    const Box bx = bounds(x), bz = bounds(z);
    // The ridge of RoofX runs along x (long in x, eaves reach +-0.5 in z); RoofZ is the same turned by a quarter.
    assert(nearf(bx.hi[2], 0.5F) && bx.hi[0] < 0.47F);
    assert(nearf(bz.hi[0], 0.5F) && bz.hi[2] < 0.47F);
    assert(nearf(bx.hi[1], bz.hi[1]));
    for (const Vertex& p : z) assert(nearf(length({p.nx, p.ny, p.nz}), 1.0F, 1e-4F));
}

void slabAndCube() {
    const Verts slab = buildSlab(12, 12);
    assert(!slab.empty() && slab.size() % 3 == 0);
    const Box b = bounds(slab);
    assert(nearf(b.hi[1], 0.0F) && b.lo[1] < -0.5F);
    assert(nearf(b.hi[0], 6.45F) && nearf(b.lo[0], -6.45F) && nearf(b.hi[2], 6.45F));
    int up = 0;
    for (const Vertex& p : slab) up += p.ny > 0.99F ? 1 : 0;
    assert(up >= 12 * 12 * 6);  // the grass: two triangles per cell
    const Verts cube = buildCube();
    assert(cube.size() == 36);
    const Box cb = bounds(cube);
    for (int i = 0; i < 3; ++i) assert(nearf(cb.lo[i], -0.5F) && nearf(cb.hi[i], 0.5F));
}

}  // namespace

int main() {
    directions();
    compatTable();
    buildRejectsBadSets();
    entropyValues();
    solidShareValues();
    weightedPick();
    rngRange();
    indexing();
    propagationUpAndDown();
    borderRules();
    impossibleSetGivesUp();
    collapseAndClick();
    pickOrder();
    touchedAndQueueConsistency();
    castleTileSet();
    castleSidewaysRules();
    castleVerticalRules();
    castleBorderRules();
    castleWorldsAreLegal();
    castleBenchmark();
    stepClock();
    smallHelpers();
    matrices();
    orbitCamera();
    rays();
    rayBoxCases();
    picking();
    worldSpace();
    chips();
    meshesAreWellFormed();
    armsMeetOnTheFace();
    roofsTurn();
    slabAndCube();
    std::puts("wfc-3d logic: all tests passed");
    return 0;
}
