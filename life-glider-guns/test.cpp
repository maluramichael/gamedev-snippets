// Tests for the Game of Life logic. Plain asserts on hand-computed cases, no framework, no SDL.
// Numbers marked "(reference)" come from an independent Python implementation of the same rules (a set of live cells on an
// unbounded plane, with a modulo version for the torus), so they are a second opinion and not just this code agreeing with itself.
#undef NDEBUG
#include <algorithm>
#include <array>
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <iterator>
#include <string>
#include <vector>

#include "logic.hpp"

using namespace life;

namespace {

Shape sorted(Shape s) {
    std::sort(s.begin(), s.end());
    return s;
}

Shape moved(const Shape& s, int dx, int dy) {
    Shape out;
    for (const Cell& c : s) out.push_back({c.x + dx, c.y + dy});
    return sorted(std::move(out));
}

void run(World& w, int generations) {
    for (int i = 0; i < generations; ++i) w.step();
}

void worldBasics() {
    World tiny(1, 1, true);
    assert(tiny.width() == kMinSize && tiny.height() == kMinSize);  // a torus needs 8 different neighbors
    World w(10, 8, false);
    assert(w.width() == 10 && w.height() == 8);
    assert(w.population() == 0 && w.generation() == 0);
    assert(!w.alive(-1, 0) && !w.alive(10, 0) && !w.alive(0, 8));  // outside a walled world nothing is alive
    w.set(-1, 0, true);                                            // and setting there does nothing
    w.set(10, 3, true);
    assert(w.population() == 0);
    w.set(3, 3, true);
    assert(w.alive(3, 3) && w.population() == 1 && w.age(3, 3) == 1);
    w.set(3, 3, false);
    assert(!w.alive(3, 3) && w.age(3, 3) == 0);
    assert(w.age(-5, -5) == 0);

    World t(10, 8, true);
    t.set(-1, 0, true);  // wraps to the right edge
    assert(t.alive(9, 0) && t.alive(-1, 0) && t.alive(19, 0));
    t.set(0, 8, true);  // wraps to the top edge
    assert(t.alive(0, 0));
    assert(t.population() == 2);
    t.setTorus(false);
    assert(!t.alive(-1, 0) && t.alive(9, 0));

    const Shape cells = t.cells();
    assert(cells.size() == 2 && cells[0] == (Cell{0, 0}) && cells[1] == (Cell{9, 0}));  // row by row
    t.clear();
    assert(t.population() == 0 && t.generation() == 0);
}

void stampingAndEdges() {
    World wall(10, 10, false);
    wall.stamp(shapeOf(Kind::Block), 8, 8);  // fits exactly
    assert(wall.population() == 4);
    wall.clear();
    wall.stamp(shapeOf(Kind::Block), 9, 9);  // only one cell is inside
    assert(wall.population() == 1 && wall.alive(9, 9));
    wall.clear();
    wall.stamp(shapeOf(Kind::Block), -1, -1);  // and here again only one, the corner (0, 0)
    assert(wall.population() == 1 && wall.alive(0, 0));

    World torus(10, 10, true);
    torus.stamp(shapeOf(Kind::Block), 9, 9);  // all four corners
    assert(torus.population() == 4);
    assert(torus.alive(9, 9) && torus.alive(0, 9) && torus.alive(9, 0) && torus.alive(0, 0));
}

void ageOfCells() {
    World w(10, 10, false);
    w.stamp(shapeOf(Kind::Block), 4, 4);
    assert(w.age(4, 4) == 1);
    run(w, 5);
    assert(w.age(4, 4) == 6 && w.age(5, 5) == 6);  // a survivor gets one older each generation
    w.set(4, 4, true);                             // setting a live cell does not reset its age
    assert(w.age(4, 4) == 6);
    run(w, 400);
    assert(w.age(4, 4) == kMaxAge);  // saturates instead of wrapping around
    World b(10, 10, false);
    b.stamp(shapeOf(Kind::Blinker), 3, 4);
    b.step();
    assert(b.age(4, 4) == 2);                      // the middle survives
    assert(b.age(4, 3) == 1 && b.age(4, 5) == 1);  // the ends are newborn
}

void stillLifeAndOscillator() {
    World w(12, 12, false);
    w.stamp(shapeOf(Kind::Block), 5, 5);
    const Shape start = w.cells();
    for (int i = 0; i < 10; ++i) {
        w.step();
        assert(w.cells() == start);
    }
    assert(w.generation() == 10);

    // The blinker has period 2: horizontal, vertical, horizontal again.
    World b(12, 12, false);
    b.stamp(shapeOf(Kind::Blinker), 4, 5);
    const Shape horizontal = b.cells();
    assert(horizontal == (Shape{{4, 5}, {5, 5}, {6, 5}}));
    b.step();
    assert(b.cells() == (Shape{{5, 4}, {5, 5}, {5, 6}}));
    b.step();
    assert(b.cells() == horizontal);
    b.step();
    assert(b.population() == 3);

    // Three in a row dying out, two cells alone die, a lone cell dies: underpopulation.
    World d(8, 8, false);
    d.set(1, 1, true);
    d.set(2, 1, true);
    d.step();
    assert(d.population() == 0);
}

void shapes() {
    assert(shapeOf(Kind::Glider).size() == 5 && shapeOf(Kind::Block).size() == 4 && shapeOf(Kind::Blinker).size() == 3 &&
           shapeOf(Kind::Gun).size() == 36);
    const Extent g = extent(shapeOf(Kind::Glider));
    assert(g.w == 3 && g.h == 3);
    const Extent gun = extent(shapeOf(Kind::Gun));
    assert(gun.w == 36 && gun.h == 9);
    const Extent gunTurned = extent(shapeOf(Kind::Gun, 1));
    assert(gunTurned.w == 9 && gunTurned.h == 36);
    assert(extent(shapeOf(Kind::Blinker)).w == 3 && extent(shapeOf(Kind::Blinker)).h == 1);
    assert(extent(shapeOf(Kind::Blinker, 1)).w == 1 && extent(shapeOf(Kind::Blinker, 1)).h == 3);
    assert(shapeOf(Kind::Blinker, 1) == (Shape{{0, 0}, {0, 1}, {0, 2}}));  // (x, y) -> (-y, x), moved back to the corner

    // Every shape sits at the top left corner, sorted row by row.
    for (Kind k : {Kind::Glider, Kind::Block, Kind::Blinker, Kind::Gun}) {
        for (int r = 0; r < 4; ++r) {
            const Shape s = shapeOf(k, r, 1);
            int minX = 1000;
            int minY = 1000;
            for (const Cell& c : s) {
                minX = std::min(minX, c.x);
                minY = std::min(minY, c.y);
            }
            assert(minX == 0 && minY == 0);
            assert(std::is_sorted(s.begin(), s.end()));
        }
    }
    // Four turns are no turn, a negative count turns the other way, the glider phase wraps after four.
    assert(shapeOf(Kind::Gun, 4) == shapeOf(Kind::Gun, 0));
    assert(shapeOf(Kind::Gun, -1) == shapeOf(Kind::Gun, 3));
    assert(shapeOf(Kind::Glider, 0, 5) == shapeOf(Kind::Glider, 0, 1));
    assert(shapeOf(Kind::Glider, 0, -1) == shapeOf(Kind::Glider, 0, 3));
    assert(shapeOf(Kind::Gun, 2) != shapeOf(Kind::Gun, 0));

    // The first two phases of the glider, by hand. Phase 0:   .O.   Phase 1:  O.O
    //                                                          ..O            .OO
    //                                                          OOO            .O.
    assert(gliderPhase(0) == (Shape{{1, 0}, {2, 1}, {0, 2}, {1, 2}, {2, 2}}));
    assert(gliderPhase(1) == (Shape{{0, 0}, {2, 0}, {1, 1}, {2, 1}, {1, 2}}));
    assert(gliderPhase(0) != gliderPhase(2));  // phase 2 is the mirror image of phase 0, not the same picture
    assert(gliderPhase(4) == gliderPhase(0));  // after four generations the same picture, moved by (1, 1)

    // Centering: the middle of the shape lands on the cell.
    assert(topLeftFor(shapeOf(Kind::Glider), 10, 10) == (Cell{9, 9}));
    assert(topLeftFor(shapeOf(Kind::Block), 10, 10) == (Cell{9, 9}));
    assert(topLeftFor(shapeOf(Kind::Blinker), 10, 10) == (Cell{9, 10}));
    assert(topLeftFor(shapeOf(Kind::Gun), 80, 32) == (Cell{62, 28}));
}

void gliderMovesDiagonally() {
    // 4 generations move a glider by one cell diagonally, in all four directions and all four phases.
    for (int rotation = 0; rotation < 4; ++rotation) {
        for (int phase = 0; phase < 4; ++phase) {
            World w(30, 30, false);
            const Shape start = shapeOf(Kind::Glider, rotation, phase);
            w.stamp(start, 12, 12);
            const Shape at0 = w.cells();
            w.step();
            assert(w.population() == 5);
            assert(w.cells() != moved(start, 12, 12));  // after one generation the picture is another phase
            run(w, 3);
            const Cell h = gliderHeading(rotation);
            assert(w.cells() == moved(at0, h.x, h.y));
            run(w, 36);  // ten times as far
            assert(w.cells() == moved(at0, 10 * h.x, 10 * h.y));
        }
    }
    assert(gliderHeading(0) == (Cell{1, 1}) && gliderHeading(1) == (Cell{-1, 1}));
    assert(gliderHeading(2) == (Cell{-1, -1}) && gliderHeading(3) == (Cell{1, -1}));
    assert(gliderHeading(4) == gliderHeading(0) && gliderHeading(-1) == gliderHeading(3));
    assert(std::string(headingName(0)) == "SE" && std::string(headingName(3)) == "NE");
}

void gliderWrapsAroundTheTorus() {
    // On an 8x8 torus a glider is back after 4 * 8 generations.
    World w(8, 8, true);
    w.stamp(shapeOf(Kind::Glider), 2, 2);
    const Shape start = w.cells();
    run(w, 4);
    Shape expected;
    for (const Cell& c : start) expected.push_back({c.x + 1, c.y + 1});
    assert(w.cells() == sorted(expected));
    run(w, 28);
    assert(w.generation() == 32 && w.cells() == start);
    run(w, 4);
    assert(w.cells() != start);

    // On 8x12 it needs the least common multiple of both sizes: 4 * 24 = 96 generations.
    World r(8, 12, true);
    r.stamp(shapeOf(Kind::Glider), 2, 2);
    const Shape startR = r.cells();
    run(r, 32);
    assert(r.cells() != startR);  // moved by (8, 8) = (0, 8) on this torus
    run(r, 16);
    assert(r.cells() != startR);  // (12, 12) = (4, 0)
    run(r, 48);
    assert(r.generation() == 96 && r.cells() == startR);

    // On the 160x64 field of the demo: 4 * lcm(160, 64) = 4 * 320 = 1280 generations. Half of that it is 32 rows down, not back.
    World big(kCols, kRows, true);
    big.stamp(shapeOf(Kind::Glider), 70, 20);
    const Shape startBig = big.cells();
    run(big, 640);
    assert(big.cells() != startBig);  // (160, 160) = (0, 32)
    run(big, 640);
    assert(big.generation() == 1280 && big.cells() == startBig);

    // Crossing the corner changes nothing about the glider: it is always 5 cells.
    World c(8, 8, true);
    c.stamp(shapeOf(Kind::Glider), 6, 6);
    for (int i = 0; i < 40; ++i) {
        c.step();
        assert(c.population() == 5);
    }
}

void wallsTurnGlidersIntoBlocks() {
    // (reference) a glider that hits a wall ends as a block: all 16 combinations of direction and phase.
    for (int rotation = 0; rotation < 4; ++rotation) {
        for (int phase = 0; phase < 4; ++phase) {
            World w(20, 20, false);
            w.stamp(shapeOf(Kind::Glider, rotation, phase), 8, 8);
            run(w, 200);
            assert(w.population() == 4);
            assert(normalized(w.cells()) == shapeOf(Kind::Block));
            run(w, 1);
            assert(w.population() == 4);
        }
    }
}

void torusEdgeConnectsOppositeSides() {
    // A vertical blinker on the left edge. With a torus it grows a horizontal one across the edge,
    // with walls the part beyond the edge is missing and it dies out after two generations.
    World torus(10, 10, true);
    torus.stamp(Shape{{0, 4}, {0, 5}, {0, 6}}, 0, 0);
    torus.step();
    assert(torus.cells() == (Shape{{0, 5}, {1, 5}, {9, 5}}));  // x = 9 is the cell left of x = 0

    World wall(10, 10, false);
    wall.stamp(Shape{{0, 4}, {0, 5}, {0, 6}}, 0, 0);
    wall.step();
    assert(wall.cells() == (Shape{{0, 5}, {1, 5}}));
    wall.step();
    assert(wall.population() == 0);

    // The same at the top edge.
    World top(10, 10, true);
    top.stamp(Shape{{4, 0}, {5, 0}, {6, 0}}, 0, 0);
    top.step();
    assert(top.cells() == (Shape{{5, 0}, {5, 1}, {5, 9}}));

    // A block over the corner of the torus is still a block, with walls its four corner cells are alone and die.
    World corner(10, 10, true);
    corner.stamp(shapeOf(Kind::Block), 9, 9);
    const Shape start = corner.cells();
    run(corner, 6);
    assert(corner.cells() == start);
    World dead(10, 10, false);
    for (const Cell& c : start) dead.set(c.x, c.y, true);
    dead.step();
    assert(dead.population() == 0);
}

void gunShootsAGliderEvery30Generations() {
    // (reference) population of the gun in generations 0 to 30 on an unbounded plane.
    constexpr std::array<int, 31> kPops{{36, 39, 43, 48, 51, 44, 51, 48, 61, 42, 48, 50, 54, 55, 56, 42,
                                         44, 47, 53, 54, 54, 54, 49, 60, 43, 50, 47, 47, 50, 48, 41}};
    World w(kCols, kRows, false);
    w.stamp(shapeOf(Kind::Gun), 4, 4);
    const Shape gun = w.cells();
    assert(gun.size() == 36);
    for (int g = 0; g <= 30; ++g) {
        assert(w.population() == kPops[static_cast<std::size_t>(g)]);
        if (g < 30) w.step();
    }

    // Generation 30: the gun is exactly as it was, plus one glider, so +5 cells.
    Shape extra;
    const Shape now = w.cells();
    std::set_difference(now.begin(), now.end(), gun.begin(), gun.end(), std::back_inserter(extra));
    assert(extra.size() == 5);
    assert(std::includes(now.begin(), now.end(), gun.begin(), gun.end()));
    bool isGlider = false;
    for (int phase = 0; phase < 4; ++phase) isGlider = isGlider || normalized(extra) == shapeOf(Kind::Glider, 0, phase);
    assert(isGlider);
    assert(extra[0].x > 4 + 20 && extra[0].y > 4 + 8);  // and it left the gun toward the southeast

    // From here on every 30 generations it is 5 more, while the gun shows the same picture.
    for (int k = 2; k <= 6; ++k) {
        run(w, 30);
        assert(w.population() == 36 + 5 * k);
        const Shape cells = w.cells();
        assert(std::includes(cells.begin(), cells.end(), gun.begin(), gun.end()));
    }
    assert(w.generation() == 180);
}

void gunIsTheSameInEveryDirection() {
    // Turned by 90 degrees the gun shoots in another direction and counts the same: 36, 41, 46 at generations 0, 30, 60.
    for (int rotation = 0; rotation < 4; ++rotation) {
        World w(100, 100, false);
        const Shape s = shapeOf(Kind::Gun, rotation);
        const Cell tl = topLeftFor(s, 50, 50);
        w.stamp(s, tl.x, tl.y);
        assert(w.population() == 36);
        run(w, 30);
        assert(w.population() == 41);
        run(w, 30);
        assert(w.population() == 46);
    }
}

void wallsStopTheGrowthTorusDoesNot() {
    // (reference) the gun at (4, 4) on 160x64. With walls the stream ends in blocks at the border and the population
    // settles into a cycle, on the torus the stream wraps around and it keeps growing.
    World wall(kCols, kRows, false);
    World torus(kCols, kRows, true);
    wall.stamp(shapeOf(Kind::Gun), 4, 4);
    torus.stamp(shapeOf(Kind::Gun), 4, 4);
    run(wall, 100);
    run(torus, 100);
    assert(wall.population() == 63 && torus.population() == 63);  // same until the first glider reaches the border
    run(wall, 200);
    run(torus, 200);
    assert(wall.population() == 75 && torus.population() == 86);  // generation 300
    run(wall, 300);
    run(torus, 300);
    assert(wall.population() == 75 && torus.population() == 136);  // generation 600
    run(wall, 600);
    run(torus, 600);
    assert(wall.population() == 75 && torus.population() == 236);  // generation 1200
}

int populationAfter(int collision, int ox, int oy, int generations) {
    World w(120, 120, false);
    loadCollision(w, collision, ox, oy);
    assert(w.population() == 10);
    run(w, generations);
    return w.population();
}

void gliderCollisions() {
    // (reference) two gliders, same direction of travel, only the offset differs. After 150 generations:
    assert(populationAfter(0, 30, 30, 150) == 10);  // offset (0, -6): they miss each other, both still fly
    assert(populationAfter(1, 30, 30, 150) == 0);   // offset (0, 0): both gone
    assert(populationAfter(2, 30, 30, 150) == 4);   // offset (1, 1): one block is left
    assert(populationAfter(3, 30, 30, 150) == 16);  // offset (3, -2): four blocks
    // The outcome depends on the offset only, not on where it happens, and nothing more happens afterwards.
    assert(populationAfter(2, 25, 41, 150) == 4);
    assert(populationAfter(3, 41, 25, 150) == 16);
    assert(populationAfter(2, 30, 30, 160) == 4);
    assert(populationAfter(3, 30, 30, 160) == 16);
    assert(populationAfter(1, 30, 30, 160) == 0);
    // The index wraps.
    assert(populationAfter(6, 30, 30, 150) == 4 && populationAfter(-3, 30, 30, 150) == 0);

    // Before they meet nothing differs: 10 cells for the first generations in all four cases.
    for (int i = 0; i < 4; ++i) assert(populationAfter(i, 30, 30, 20) == 10);
}

void drawingLines() {
    assert(lineCells({3, 4}, {3, 4}) == (Shape{{3, 4}}));
    assert(lineCells({0, 0}, {3, 0}) == (Shape{{0, 0}, {1, 0}, {2, 0}, {3, 0}}));
    assert(lineCells({0, 0}, {2, 2}) == (Shape{{0, 0}, {1, 1}, {2, 2}}));
    assert(lineCells({2, 0}, {0, 0}) == (Shape{{2, 0}, {1, 0}, {0, 0}}));  // from right to left, in that order
    assert(lineCells({0, 0}, {0, -2}) == (Shape{{0, 0}, {0, -1}, {0, -2}}));
    // Steep line, calculated by hand: dx = 2, dy = 5, six cells, one per row it passes through.
    assert(lineCells({0, 0}, {2, 5}) == (Shape{{0, 0}, {0, 1}, {1, 2}, {1, 3}, {2, 4}, {2, 5}}));
    // Both ends are always on the line. In the other direction a tie may be broken the other way, so only the count is the same.
    Shape forward = lineCells({1, 7}, {9, 2});
    Shape backward = lineCells({9, 2}, {1, 7});
    assert(forward.size() == 9 && backward.size() == 9);  // max(dx, dy) + 1
    assert(forward.front() == (Cell{1, 7}) && forward.back() == (Cell{9, 2}));
    assert(backward.front() == (Cell{9, 2}) && backward.back() == (Cell{1, 7}));
    // Neighbors on the line touch, so a fast drag leaves no gap.
    for (std::size_t i = 1; i < forward.size(); ++i) {
        assert(std::abs(forward[i].x - forward[i - 1].x) <= 1 && std::abs(forward[i].y - forward[i - 1].y) <= 1);
    }
}

void pacing() {
    Pacer p;
    assert(p.due(1.0 / 60.0, 30.0) == 0);  // half a generation
    assert(p.due(1.0 / 60.0, 30.0) == 1);  // the second half completes it
    assert(std::abs(p.acc) < 1e-6);
    // The remainder stays: 1.5 generations are one now and a half left.
    Pacer q;
    assert(q.due(0.05, 30.0) == 1);
    assert(std::abs(q.acc - 1.0 / 60.0) < 1e-9);
    assert(q.due(0.05, 30.0) == 2);
    // A long frame is clamped to a quarter second: 7 generations at 30 per second.
    Pacer slow;
    assert(slow.due(5.0, 30.0) == 7);
    // At 120 per second a quarter second is 30 generations, the cap is 16 and the rest is dropped.
    Pacer fast;
    assert(fast.due(1.0, 120.0) == kMaxStepsPerFrame);
    assert(fast.acc <= 1.0 / 120.0 + 1e-12);
    assert(fast.due(0.0, 120.0) <= 1);
    // No rate, no steps. Negative time is nothing.
    Pacer none;
    assert(none.due(0.1, 0.0) == 0 && none.due(0.1, -3.0) == 0);
    assert(none.due(-1.0, 30.0) == 0 && none.acc == 0.0);
    q.reset();
    assert(q.acc == 0.0);
    // Over a second of 60 fps frames at every speed the generations add up (a little less, the last one may be unfinished).
    for (int rate : kSpeeds) {
        Pacer r;
        int total = 0;
        for (int i = 0; i < 60; ++i) total += r.due(1.0 / 60.0, rate);
        assert(total >= rate - 1 && total <= rate);
    }
}

void populationHistory() {
    PopulationHistory h;
    assert(h.size() == 0 && h.peak(10) == 0 && !h.delta(1).has_value() && !h.delta(0).has_value());
    for (int i = 0; i <= 40; ++i) h.push(i);
    assert(h.size() == 41 && h.at(0) == 0 && h.at(40) == 40);
    assert(h.delta(30).value() == 30);  // 40 - 10
    assert(h.delta(40).value() == 40);  // all the way back
    assert(!h.delta(41).has_value());   // not that many generations yet
    assert(h.delta(0).value() == 0);
    assert(h.peak(5) == 40 && h.peak(1000) == 40);
    h.replaceLast(7);
    assert(h.size() == 41 && h.at(40) == 7 && h.peak(5) == 39);
    h.clear();
    assert(h.size() == 0);
    h.replaceLast(9);  // on an empty history it adds the first entry
    assert(h.size() == 1 && h.at(0) == 9);

    PopulationHistory small(10);
    for (int i = 0; i < 25; ++i) small.push(i);
    assert(small.size() == 10 && small.at(0) == 15 && small.at(9) == 24);
    assert(PopulationHistory(0).size() == 0);  // a capacity of 0 still keeps one entry
    PopulationHistory one(0);
    one.push(3);
    one.push(4);
    assert(one.size() == 1 && one.at(0) == 4);

    // What the demo shows: the gun at 30 generations is +5 against the start.
    World w(kCols, kRows, false);
    w.stamp(shapeOf(Kind::Gun), 4, 4);
    PopulationHistory hist;
    hist.push(w.population());
    for (int i = 0; i < 90; ++i) {
        w.step();
        hist.push(w.population());
    }
    assert(hist.delta(30).value() == 5 && hist.delta(60).value() == 10 && hist.delta(90).value() == 15);
    // The gun swings between 36 and 61 cells inside one period, but 30 generations apart it is always the same picture.
    for (int i = 1; i <= 60; ++i) {
        w.step();
        hist.push(w.population());
        assert(hist.delta(30).value() == 5);
    }
}

}  // namespace

int main() {
    worldBasics();
    stampingAndEdges();
    ageOfCells();
    stillLifeAndOscillator();
    shapes();
    gliderMovesDiagonally();
    gliderWrapsAroundTheTorus();
    wallsTurnGlidersIntoBlocks();
    torusEdgeConnectsOppositeSides();
    gunShootsAGliderEvery30Generations();
    gunIsTheSameInEveryDirection();
    wallsStopTheGrowthTorusDoesNot();
    gliderCollisions();
    drawingLines();
    pacing();
    populationHistory();
    std::puts("life-glider-guns logic: all asserts passed");
    return 0;
}
