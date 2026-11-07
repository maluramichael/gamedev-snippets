// Tests for the cyclic cellular automaton. Plain asserts on hand-computed cases, no framework, no SDL.
// The period checks at the end run the real thing on a 64x64 torus and were first measured with a separate throwaway
// program, so they are not just this code agreeing with itself.
#undef NDEBUG
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <utility>
#include <vector>

#include "logic.hpp"

using namespace cca;

namespace {

Grid make(int w, int h, const std::vector<int>& values) {
    Grid g(w, h);
    assert(values.size() == static_cast<std::size_t>(w * h));
    for (int i = 0; i < w * h; ++i) g.cells[static_cast<std::size_t>(i)] = static_cast<std::uint8_t>(values[static_cast<std::size_t>(i)]);
    return g;
}

std::vector<int> values(const Grid& g) { return std::vector<int>(g.cells.begin(), g.cells.end()); }

void successorWraps() {
    assert(successor(0, 3) == 1);
    assert(successor(1, 3) == 2);
    assert(successor(2, 3) == 0);  // the last color is eaten by color 0
    assert(successor(15, 16) == 0);
}

void neighborCounts() {
    assert(neighborsOf(Hood::VonNeumann) == 4);
    assert(neighborsOf(Hood::Moore) == 8);
    // The threshold cannot exceed the neighbors that exist, and it is at least 1 (0 would eat every cell for free).
    assert(clampThreshold(9, Hood::Moore) == 8);
    assert(clampThreshold(5, Hood::VonNeumann) == 4);
    assert(clampThreshold(0, Hood::Moore) == 1);
    assert(clampThreshold(3, Hood::Moore) == 3);
}

void onlyTheSuccessorEats() {
    // 3x3 torus, n = 3, the center is color 0. Von Neumann neighbors are up, down, left, right.
    const Rule rule{3, 1, Hood::VonNeumann};
    // All four neighbors have color 1 = successor of 0: eaten.
    assert(eatenBy(make(3, 3, {2, 1, 2, 1, 0, 1, 2, 1, 2}), 1, 1, rule) == 4);
    // All four neighbors have color 2 = the color BEHIND 0: nobody eats it.
    assert(eatenBy(make(3, 3, {1, 2, 1, 2, 0, 2, 1, 2, 1}), 1, 1, rule) == 0);
    // All four have its own color: nobody eats it.
    assert(eatenBy(make(3, 3, {1, 0, 1, 0, 0, 0, 1, 0, 1}), 1, 1, rule) == 0);
    // Von Neumann ignores the diagonals even when they hold the successor.
    assert(eatenBy(make(3, 3, {1, 2, 1, 2, 0, 2, 1, 2, 1}), 1, 1, rule) == 0);
    // Moore counts them: four diagonal ones of color 1 and four edge ones of color 2.
    assert(eatenBy(make(3, 3, {1, 2, 1, 2, 0, 2, 1, 2, 1}), 1, 1, Rule{3, 1, Hood::Moore}) == 4);
    // The step follows: only the cell with enough eaters changes.
    const Grid after = step(make(3, 3, {2, 1, 2, 1, 0, 1, 2, 1, 2}), rule);
    assert(after.at(1, 1) == 1);
}

void thresholdDecides() {
    // 3x3 torus, n = 3, the center is 0 and exactly two of its four von Neumann neighbors are 1 (up and left).
    const Grid g = make(3, 3, {2, 1, 2, 1, 0, 2, 2, 2, 2});
    assert(eatenBy(g, 1, 1, Rule{3, 1, Hood::VonNeumann}) == 2);
    assert(step(g, Rule{3, 1, Hood::VonNeumann}).at(1, 1) == 1);
    assert(step(g, Rule{3, 2, Hood::VonNeumann}).at(1, 1) == 1);  // equal to the threshold counts
    assert(step(g, Rule{3, 3, Hood::VonNeumann}).at(1, 1) == 0);  // one short: stays
}

void colorWrapsInAStep() {
    // Color 2 of 3 is eaten by color 0 and becomes 0.
    const Grid g = make(3, 3, {0, 0, 0, 0, 2, 0, 0, 0, 0});
    assert(step(g, Rule{3, 1, Hood::VonNeumann}).at(1, 1) == 0);
    // Neighbors of the same cell stay: their successor is 1, and nobody around has a 1.
    assert(step(g, Rule{3, 1, Hood::VonNeumann}).at(0, 1) == 0);
}

void edgesWrapAround() {
    // 4x1 torus (the up and down neighbors are the cell itself, which is never its own successor, so only left and right count).
    // Cell 0 has color 0 and its LEFT neighbor, cell 3, wraps around from the other end.
    const Rule rule{3, 1, Hood::VonNeumann};
    const Grid g = make(4, 1, {0, 0, 0, 1});
    const Grid after = step(g, rule);
    assert(after.at(0, 0) == 1);  // eaten by cell 3 across the border
    assert(after.at(1, 0) == 0);
    assert(after.at(2, 0) == 1);  // eaten by its right neighbor
    assert(after.at(3, 0) == 1);  // color 1 needs a 2, there is none
}

void bufferedAgainstInPlace() {
    // Row [0,1,0,0] with n = 3, threshold 1, von Neumann (again only left and right count).
    // Double buffer: every cell reads the OLD row. cell 0 sees a 1 on its right, cell 1 sees no 2, cell 2 sees a 1 on its left,
    // cell 3 sees only zeros: [1,1,1,0].
    const Rule rule{3, 1, Hood::VonNeumann};
    const Grid g = make(4, 1, {0, 1, 0, 0});
    assert((values(step(g, rule)) == std::vector<int>{1, 1, 1, 0}));
    // In place: the sweep reads cells that already changed. Cell 3 now sees the new 1 of cell 2 and the new 1 of cell 0
    // across the border and is eaten as well: [1,1,1,1]. A wave runs through the whole row within one step.
    Grid h = g;
    stepInPlace(h, rule);
    assert((values(h) == std::vector<int>{1, 1, 1, 1}));
}

void fourCellDemonTurns() {
    // 2x2 torus with the four colors in a ring: 0 1 / 3 2. Every cell has its two neighbors twice (the torus wraps onto itself),
    // so the cell with color c sees the color c+1 twice, and every cell advances. The ring turns: 1 2 / 0 3.
    const Rule rule{4, 1, Hood::VonNeumann};
    const Grid g = make(2, 2, {0, 1, 3, 2});
    const Grid next = step(g, rule);
    assert((values(next) == std::vector<int>{1, 2, 0, 3}));
    // Four steps later every cell is back where it started: period 4.
    Grid t = g;
    for (int i = 0; i < 4; ++i) t = step(t, rule);
    assert(t == g);
    // And not earlier.
    t = g;
    for (int i = 0; i < 3; ++i) {
        t = step(t, rule);
        assert(!(t == g));
    }
}

void windingOfAPlaquette() {
    // One plaquette goes (x,y) -> (x+1,y) -> (x+1,y+1) -> (x,y+1) -> back. Colors 0 1 / 3 2 read 0,1,2,3: +1 per edge, sum 4 = n.
    const Grid ring = make(2, 2, {0, 1, 3, 2});
    assert(winding(ring, 0, 0, 4) == 1);
    // The same colors the other way around, 0 3 / 1 2, wind the other way.
    const Grid back = make(2, 2, {0, 3, 1, 2});
    assert(winding(back, 0, 0, 4) == -1);
    // A flat block, a ramp (0 1 / 0 1) and a step (0 0 / 1 1) have no winding.
    assert(winding(make(2, 2, {2, 2, 2, 2}), 0, 0, 4) == 0);
    assert(winding(make(2, 2, {0, 1, 0, 1}), 0, 0, 4) == 0);
    assert(winding(make(2, 2, {0, 0, 1, 1}), 0, 0, 4) == 0);
    // n = 3: 0 1 / 0 2 reads 0,1,2,0, steps +1 +1 +1 +0 = 3 = n, one winding. The wrap from 2 to 0 is a +1 step, not a -2.
    assert(winding(make(2, 2, {0, 1, 0, 2}), 0, 0, 3) == 1);
    // A jump of exactly n/2 has no direction of its own, the order of the colors decides (smaller to larger is +n/2). The plaquette
    // 0 2 / 2 0 with n = 4 reads 0,2,0,2: +2, -2, +2, -2. Sum 0.
    assert(winding(make(2, 2, {0, 2, 2, 0}), 0, 0, 4) == 0);
}

void demonCountOnAFlatField() {
    Grid flat(8, 8);
    for (auto& c : flat.cells) c = 2;
    const Demons d = countDemons(flat, 5);
    assert(d.positive == 0 && d.negative == 0);
    assert(d.total() == 0);
    // A smooth gradient along x on a ring of width 6 with n = 6 has steps of +1 and one wrap of +1 (5 to 0): no plaquette winds.
    Grid ramp(6, 6);
    for (int y = 0; y < 6; ++y)
        for (int x = 0; x < 6; ++x) ramp.at(x, y) = static_cast<std::uint8_t>(x);
    assert(countDemons(ramp, 6).total() == 0);
}

void demonCountOnOneVortex() {
    // 4x4 torus, n = 4, all 0 except three cells (rows are y, columns are x):
    //   0 0 0 0
    //   0 0 1 0
    //   0 3 2 0
    //   0 0 0 0
    // Plaquette (1,1) reads 0,1,2,3 and winds once. Plaquette (1,2) below it reads 3,2,0,0: -1, then 2 to 0 which is a tie and
    // counts -2 by the order rule, then 0, then 0 to 3 which is -1: sum -4, winding -1. Every other plaquette sums to 0.
    Grid g(4, 4);
    g.at(2, 1) = 1;
    g.at(2, 2) = 2;
    g.at(1, 2) = 3;
    assert(winding(g, 1, 1, 4) == 1);
    assert(winding(g, 1, 2, 4) == -1);
    assert(winding(g, 2, 1, 4) == 0);
    assert(winding(g, 0, 0, 4) == 0);
    const Demons d = countDemons(g, 4);
    assert(d.positive == 1);
    assert(d.negative == 1);
    assert(d.total() == 2);
}

void torusHasNoNetCharge() {
    // Every edge of the loop is walked once in each direction by the two plaquettes that share it, so the signed sum is exactly 0
    // on a torus for ANY field. Checked on random fields for several color counts, including even ones with their n/2 ties.
    for (int colors : {3, 4, 5, 8, 11, 16}) {
        for (std::uint64_t seed = 1; seed <= 5; ++seed) {
            const Grid g = randomGrid(20, 14, colors, seed);
            const Demons d = countDemons(g, colors);
            assert(d.positive == d.negative);
            assert(d.total() > 0);  // white noise is full of them
        }
    }
}

void randomGridUsesAllColors() {
    const Grid g = randomGrid(64, 64, 8, 42);
    std::vector<int> seen(8, 0);
    for (const std::uint8_t c : g.cells) {
        assert(c < 8);
        ++seen[c];
    }
    for (int count : seen) assert(count > 0);
    // Same seed, same grid, a different seed a different one.
    assert(randomGrid(64, 64, 8, 42) == g);
    assert(!(randomGrid(64, 64, 8, 43) == g));
}

void degenerateSizes() {
    assert(Grid(0, 5).cells.empty());
    assert(Grid(-3, 5).cells.empty());
    // A grid without cells steps to a grid without cells instead of dividing by zero.
    assert(step(Grid(0, 0), Rule{3, 1, Hood::Moore}).cells.empty());
    assert(countDemons(Grid(0, 0), 3).total() == 0);
    // A 1x1 torus is its own neighbor, which is never the successor: nothing moves.
    Grid one(1, 1);
    one.at(0, 0) = 1;
    assert(step(one, Rule{3, 1, Hood::Moore}).at(0, 0) == 1);
}

void periodDetector() {
    // Four steps of the demon ring repeat the start: the detector reports 4 after the fifth grid.
    const Rule rule{4, 1, Hood::VonNeumann};
    PeriodDetector detector(16);
    Grid g = make(2, 2, {0, 1, 3, 2});
    int found = 0;
    for (int i = 0; i < 5; ++i) {
        found = detector.push(g);
        if (i < 4) assert(found == 0);
        g = step(g, rule);
    }
    assert(found == 4);
    // A grid that stays put has period 1.
    PeriodDetector still(16);
    const Grid flat = make(2, 2, {1, 1, 1, 1});
    assert(still.push(flat) == 0);
    assert(still.push(flat) == 1);
    // Reset forgets the past.
    still.reset();
    assert(still.push(flat) == 0);
}

void lineModeNeverEatsTheLastColor() {
    const Rule line{3, 1, Hood::VonNeumann, false};
    const Rule ring{3, 1, Hood::VonNeumann, true};
    assert(nextColor(1, line) == 2);
    assert(nextColor(2, line) == -1);  // nothing comes after the last color of a line
    assert(nextColor(2, ring) == 0);
    // 3x3 torus, the center is color 2 and all four neighbors are 0. On the ring 0 eats 2, on the line it does not.
    const Grid g = make(3, 3, {1, 0, 1, 0, 2, 0, 1, 0, 1});
    assert(eatenBy(g, 1, 1, ring) == 4);
    assert(eatenBy(g, 1, 1, line) == 0);
    assert(step(g, ring).at(1, 1) == 0);
    assert(step(g, line).at(1, 1) == 2);
    // Everything below the last color still works on a line: a 0 next to a 1 is eaten. Here the 1 at (1,0) and the 1 at (0,1) touch the 0 at (1,1).
    const Grid h = make(3, 3, {2, 1, 2, 1, 0, 2, 2, 2, 2});
    assert(step(h, line).at(1, 1) == 1);
}

void collectingDemons() {
    // The grid of demonCountOnOneVortex: the ring at (1,1) and its partner at (1,2), found in reading order.
    Grid g(4, 4);
    g.at(2, 1) = 1;
    g.at(2, 2) = 2;
    g.at(1, 2) = 3;
    std::vector<DemonSite> sites{{9, 9, 9}};  // old content must go
    collectDemons(g, 4, sites);
    assert(sites.size() == 2);
    assert(sites[0].x == 1 && sites[0].y == 1 && sites[0].sign == 1);
    assert(sites[1].x == 1 && sites[1].y == 2 && sites[1].sign == -1);
    collectDemons(Grid(4, 4), 4, sites);
    assert(sites.empty());
}

void stepsFromTime() {
    double acc = 0.0;
    // 4 steps per second and a frame of 0.125 s: half a step per frame, so every second frame takes one.
    assert(stepsDue(acc, 4.0, 0.125, 100) == 0);
    assert(acc == 0.5);
    assert(stepsDue(acc, 4.0, 0.125, 100) == 1);
    assert(acc == 0.0);
    // A long pause (10 s) counts as 0.25 s: 1 step at 4 per second, not 40.
    acc = 0.0;
    assert(stepsDue(acc, 4.0, 10.0, 100) == 1);
    // The work of one frame is bounded and the excess is dropped instead of piling up.
    acc = 0.0;
    assert(stepsDue(acc, 120.0, 0.25, 8) == 8);
    assert(acc <= 1.0);
    // Negative time and a rate of 0 give nothing and leave the accumulator alone.
    acc = 0.25;
    assert(stepsDue(acc, 4.0, -1.0, 100) == 0);
    assert(acc == 0.25);
    assert(stepsDue(acc, 0.0, 0.1, 100) == 0);
    assert(acc == 0.25);
}

void seriesKeepsEverySecondPoint() {
    Series s(4);
    for (int i = 0; i < 4; ++i) s.push(i);  // 0 1 2 3 reaches the capacity: keep every second point
    assert((s.values() == std::vector<int>{0, 2}));
    assert(s.stride() == 2);
    s.push(4);  // push number 4 is a multiple of the stride
    assert((s.values() == std::vector<int>{0, 2, 4}));
    s.push(5);  // not one
    assert((s.values() == std::vector<int>{0, 2, 4}));
    s.push(6);  // full again: 0 2 4 6 becomes 0 4
    assert((s.values() == std::vector<int>{0, 4}));
    assert(s.stride() == 4);
    s.clear();
    assert(s.values().empty());
    assert(s.stride() == 1);
    s.push(7);
    assert((s.values() == std::vector<int>{7}));
}

/// Runs `settle` steps from random noise on a torus, then reports the period the detector sees in the following 80 steps.
int settledPeriod(const Rule& rule, int size, std::uint64_t seed, int settle) {
    Grid g = randomGrid(size, size, rule.colors, seed);
    Grid next;
    for (int i = 0; i < settle; ++i) {
        stepInto(g, next, rule);
        std::swap(g, next);
    }
    PeriodDetector detector(32);
    int period = 0;
    for (int i = 0; i < 80; ++i) {
        period = detector.push(g);
        stepInto(g, next, rule);
        std::swap(g, next);
    }
    return period;
}

void wholeGridRepeatsAfterNSteps() {
    // Measured on a 64x64 torus from the noise of seed 7 (and for the other seeds 1..8, see the notes in the README): with threshold 1
    // every cell ends up taking one color per step, so the whole grid repeats after exactly n steps.
    for (int n : {3, 4, 5, 8}) assert(settledPeriod(Rule{n, 1, Hood::VonNeumann}, 64, 7, 800) == n);
    for (int n : {3, 4, 5, 8}) assert(settledPeriod(Rule{n, 1, Hood::Moore}, 64, 7, 800) == n);
    // The exception: for an odd n above 5 on the von Neumann neighborhood the period was n + 1 in 7 of 8 seeds (seed 3 gave n).
    assert(settledPeriod(Rule{7, 1, Hood::VonNeumann}, 64, 7, 800) == 8);
    assert(settledPeriod(Rule{7, 1, Hood::VonNeumann}, 64, 3, 800) == 7);
}

void demonsFallToTheSpirals() {
    // 64x64, n = 8, threshold 1, von Neumann, seed 7: 1266 demon blocks in the noise, 78 after the spirals have settled.
    const Rule rule{8, 1, Hood::VonNeumann};
    Grid g = randomGrid(64, 64, 8, 7);
    const Demons start = countDemons(g, 8);
    assert(start.total() == 1266);
    Grid next;
    for (int i = 0; i < 800; ++i) {
        stepInto(g, next, rule);
        std::swap(g, next);
    }
    const Demons end = countDemons(g, 8);
    assert(end.total() == 78);
    // Spirals come in pairs of opposite handedness, at every moment.
    assert(start.positive == start.negative);
    assert(end.positive == end.negative);
}

Grid mirroredLeftRight(const Grid& g) {
    Grid out(g.w, g.h);
    for (int y = 0; y < g.h; ++y)
        for (int x = 0; x < g.w; ++x) out.at(g.w - 1 - x, y) = g.at(x, y);
    return out;
}

void bufferedIsSymmetricInPlaceIsNot() {
    // A double-buffered step treats left and right alike: stepping a mirrored grid gives the mirror of the stepped grid.
    // The in-place sweep runs from the top left to the bottom right, so cells see new colors only from the left and from above,
    // and mirroring changes the outcome. Checked on the noise of seed 5 (16x16, n = 8, von Neumann, threshold 1).
    const Rule rule{8, 1, Hood::VonNeumann};
    const Grid g = randomGrid(16, 16, 8, 5);
    assert(step(mirroredLeftRight(g), rule) == mirroredLeftRight(step(g, rule)));
    Grid a = g;
    stepInPlace(a, rule);
    Grid b = mirroredLeftRight(g);
    stepInPlace(b, rule);
    assert(!(b == mirroredLeftRight(a)));
}

void highThresholdFreezesTheNoise() {
    // Four of four neighbors must carry the next color: in noise that practically never happens, nothing moves.
    const Rule rule{8, 4, Hood::VonNeumann};
    const Grid start = randomGrid(64, 64, 8, 7);
    Grid g = start;
    Grid next;
    for (int i = 0; i < 200; ++i) {
        stepInto(g, next, rule);
        std::swap(g, next);
    }
    assert(countDemons(g, 8).total() == countDemons(start, 8).total());
    assert(step(g, rule) == g);
}

void lineModeFreezesIntoTopBlocks() {
    // Without the wrap nothing can turn. From the same noise (64x64, n = 8, von Neumann, threshold 1, seed 7) the grid freezes, and
    // most cells end up in the last color, which can only be left by being eaten.
    const Rule rule{8, 1, Hood::VonNeumann, false};
    Grid g = randomGrid(64, 64, 8, 7);
    Grid next;
    for (int i = 0; i < 3000; ++i) {
        stepInto(g, next, rule);
        std::swap(g, next);
    }
    assert(step(g, rule) == g);
    int top = 0;
    for (const std::uint8_t c : g.cells) top += c == 7 ? 1 : 0;
    assert(top == 2942);
}

}  // namespace

int main() {
    successorWraps();
    neighborCounts();
    onlyTheSuccessorEats();
    thresholdDecides();
    colorWrapsInAStep();
    edgesWrapAround();
    bufferedAgainstInPlace();
    fourCellDemonTurns();
    windingOfAPlaquette();
    demonCountOnAFlatField();
    demonCountOnOneVortex();
    torusHasNoNetCharge();
    randomGridUsesAllColors();
    degenerateSizes();
    periodDetector();
    lineModeNeverEatsTheLastColor();
    collectingDemons();
    stepsFromTime();
    seriesKeepsEverySecondPoint();
    wholeGridRepeatsAfterNSteps();
    demonsFallToTheSpirals();
    bufferedIsSymmetricInPlaceIsNot();
    highThresholdFreezesTheNoise();
    lineModeFreezesIntoTopBlocks();
    std::puts("cyclic-ca-spirals: all tests passed");
    return 0;
}
