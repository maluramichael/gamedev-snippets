// Tests for Langton's ant. Plain asserts on hand-computed cases, no framework, no SDL.
// Numbers marked "(reference)" come from an independent Python implementation of the same rules (dict of cells on an
// unbounded plane, no torus), so they are a second opinion and not just this code agreeing with itself.
#undef NDEBUG
#include <algorithm>
#include <cassert>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

#include "logic.hpp"

using namespace ant;

namespace {

World make(const char* rule, int size) {
    const auto parsed = parseRule(rule);
    assert(parsed.has_value());
    return World(*parsed, size);
}

void parsing() {
    assert(parseRule("RL").has_value());
    assert(parseRule("LLRRRLRLRLLR").has_value());
    assert(parseRule("RL")->colors() == 2);
    assert(parseRule("RLLR")->colors() == 4);
    assert(parseRule("RLLR")->turnsRight(0));
    assert(!parseRule("RLLR")->turnsRight(1));
    assert(!parseRule("RLLR")->turnsRight(2));
    assert(parseRule("RLLR")->turnsRight(3));
    // Empty, a single color, other letters, lower case and too many colors are no rules.
    assert(!parseRule("").has_value());
    assert(!parseRule("R").has_value());
    assert(!parseRule("RX").has_value());
    assert(!parseRule("rl").has_value());
    assert(!parseRule("R L").has_value());
    assert(parseRule("RLRLRLRLRLRLRLRL").has_value());    // 16 colors
    assert(!parseRule("RLRLRLRLRLRLRLRLR").has_value());  // 17
}

void firstSteps() {
    // Size 8, so the start cell is (4,4), the ant faces up (y shrinks upward). White turns right, black turns left.
    World w = make("RL", 8);
    assert(w.steps() == 0 && w.antX() == 4 && w.antY() == 4 && w.antDir() == 0 && w.colored() == 0);
    w.step();  // white at (4,4): turn right, flip, move to (5,4)
    assert(w.colorAt(4, 4) == 1 && w.antX() == 5 && w.antY() == 4 && w.antDir() == 1);
    w.step();  // (5,4) white: facing down, to (5,5)
    assert(w.antX() == 5 && w.antY() == 5 && w.antDir() == 2);
    w.step();  // (5,5): facing left, to (4,5)
    assert(w.antX() == 4 && w.antY() == 5 && w.antDir() == 3);
    w.step();  // (4,5): facing up, back to (4,4), which is black now. A 2x2 black square.
    assert(w.antX() == 4 && w.antY() == 4 && w.antDir() == 0 && w.colored() == 4);
    w.step();  // black: turn left (up becomes left), flip to white, move to (3,4)
    assert(w.colorAt(4, 4) == 0 && w.antX() == 3 && w.antY() == 4 && w.antDir() == 3 && w.colored() == 3);
    w.step();  // (3,4) white: turn right (left becomes up), move to (3,3)
    assert(w.colorAt(3, 4) == 1 && w.antX() == 3 && w.antY() == 3 && w.antDir() == 0 && w.colored() == 4);
    assert(w.steps() == 6);
}

void wrapping() {
    // Size 2: the start is (1,1). Every move leaves the 2x2 grid on one side and comes back on the other.
    World w = make("RL", 2);
    w.step();  // right: x 1 -> 2 -> 0
    assert(w.antX() == 0 && w.antY() == 1);
    w.step();  // down: y 1 -> 2 -> 0
    assert(w.antX() == 0 && w.antY() == 0);
    w.step();  // left: x 0 -> -1 -> 1
    assert(w.antX() == 1 && w.antY() == 0);
    w.step();  // up: y 0 -> -1 -> 1, onto the black start cell
    assert(w.antX() == 1 && w.antY() == 1 && w.antDir() == 0 && w.colored() == 4);
    w.step();  // black: turn left, flip, move left to x = 0
    assert(w.antX() == 0 && w.antY() == 1 && w.antDir() == 3 && w.colored() == 3);
}

void manyColors() {
    // RLLR: colors 0 and 3 turn right, 1 and 2 turn left, and a visit adds one color (modulo 4).
    World w = make("RLLR", 8);
    w.run(4);  // the same 2x2 walk as the classic ant, the four cells now have color 1
    assert(w.antX() == 4 && w.antY() == 4 && w.antDir() == 0 && w.colorAt(4, 4) == 1);
    w.step();  // color 1 turns left: up becomes left. The start cell becomes color 2.
    assert(w.colorAt(4, 4) == 2 && w.antX() == 3 && w.antY() == 4 && w.antDir() == 3);
    // Colors wrap around: with two colors and both turning right the ant circles a 2x2 square, a cell visited twice is white again.
    World loop = make("RR", 8);
    loop.run(4);  // around the 2x2 square once: all four cells color 1
    assert(loop.colorAt(4, 4) == 1 && loop.colorAt(5, 4) == 1 && loop.colorAt(5, 5) == 1 && loop.colorAt(4, 5) == 1);
    loop.run(4);  // a second time: all four back to 0
    assert(loop.colorAt(4, 4) == 0 && loop.colorAt(5, 4) == 0 && loop.colored() == 0 && loop.antX() == 4 && loop.antY() == 4);
}

void cellCounts() {
    // (reference) colored cells of the classic ant on the unbounded plane
    World w = make("RL", 256);
    w.run(1000);
    assert(w.colored() == 118);
    w.run(9000);
    assert(w.colored() == 720);
}

void highwayStart() {
    World w = make("RL", 256);
    for (int i = 0; i < 10392; ++i) {  // no false alarm before: not a single step of the first 10392 reports one
        w.step();
        assert(!w.highway().has_value());
    }
    assert(w.phase() == Phase::Chaos);
    w.step();  // a highway is only recognized after three full periods: 9977 + 4 * 104 = 10393
    const auto h = w.highway();
    assert(h.has_value());
    assert(h->period == 104);
    assert(h->onset == 9977);  // (reference) the turns repeat with period 104 from step 9977 on, and not before
    assert(h->dx == -2 && h->dy == 2);
    assert(w.phase() == Phase::Highway);
    // It keeps going: one period later the ant stands exactly (-2, +2) away, with the same heading.
    w.run(1607);  // step 12000
    assert(w.highway().has_value() && w.highway()->onset == 9977);
    const int x0 = w.antX();
    const int y0 = w.antY();
    const int d0 = w.antDir();
    w.run(104);
    assert(w.antX() == x0 - 2 && w.antY() == y0 + 2 && w.antDir() == d0);
    w.run(104);
    assert(w.antX() == x0 - 4 && w.antY() == y0 + 4 && w.antDir() == d0);
}

void torusBreaksHighway() {
    // (reference) on a 100x100 torus the highway runs into its own earlier mess: recognized at 10393, gone at 14209,
    // back at 16343, gone again at 17649.
    World w = make("RL", 100);
    w.run(10392);
    assert(!w.highway().has_value());
    w.step();
    assert(w.highway().has_value());
    w.run(14208 - 10393);
    assert(w.steps() == 14208 && w.highway().has_value());
    w.step();
    assert(w.steps() == 14209 && !w.highway().has_value());
    w.run(16342 - 14209);
    assert(!w.highway().has_value());
    w.step();
    assert(w.steps() == 16343 && w.highway().has_value());
    w.run(17648 - 16343);
    assert(w.highway().has_value());
    w.step();
    assert(w.steps() == 17649 && !w.highway().has_value());
}

void torusSizes() {
    // (reference) on 200x200 the first highway has room for 9000 more steps: recognized at 10393, gone at 19409.
    World big = make("RL", 200);
    big.run(19408);
    assert(big.highway().has_value() && big.highway()->onset == 9977);
    big.step();
    assert(big.steps() == 19409 && !big.highway().has_value());
    // (reference) on 50x50 the highway never gets going: the trail meets itself after 25 cells.
    World small = make("RL", 50);
    for (long i = 0; i < 60000; ++i) {
        small.step();
        assert(!small.highway().has_value());
    }
}

void squareGrowth() {
    // (reference) LRRRRRLLR (nine colors) grows into a nearly full square: after 100000 steps 16974 cells are colored and
    // they span 132 x 132 cells, 97.4 percent of that square.
    World w = make("LRRRRRLLR", 256);
    w.run(100000);
    assert(w.colored() == 16974);
    int x0 = 256, x1 = -1, y0 = 256, y1 = -1;
    for (int y = 0; y < 256; ++y) {
        for (int x = 0; x < 256; ++x) {
            if (w.colorAt(x, y) == 0) continue;
            x0 = std::min(x0, x);
            x1 = std::max(x1, x);
            y0 = std::min(y0, y);
            y1 = std::max(y1, y);
        }
    }
    assert(x1 - x0 + 1 == 132 && y1 - y0 + 1 == 132);
}

void closedLoopIsNoHighway() {
    // Every turn is R: the ant circles a 2x2 square forever. Its turns repeat with period 1 (and 4, 8, ...), but it
    // does not get anywhere, so it is no highway.
    World w = make("RR", 16);
    w.run(5000);
    assert(!w.highway().has_value());
    assert(w.phase() != Phase::Highway);
}

std::vector<long> symmetricSteps(const char* rule, long upTo, int size) {
    World w = make(rule, size);
    std::vector<long> steps;
    for (long i = 1; i <= upTo; ++i) {
        w.step();
        if (w.symmetries() != 0) steps.push_back(w.steps());
    }
    return steps;
}

void symmetry() {
    // (reference) the classic ant is point- or mirror-symmetric around its start at exactly these steps, up to step 12500.
    const std::vector<long> expected{1, 2, 4, 5, 8, 9, 96, 97, 184, 185, 368, 369, 384, 385, 472, 473};
    assert(symmetricSteps("RL", 12500, 256) == expected);

    World w = make("RL", 256);
    assert(w.symmetries() == (kRot180 | kFlipX | kFlipY | kDiag | kAnti | kRot90));  // an empty field is symmetric in every way
    w.step();  // one black cell at (c,c) is symmetric in every way
    assert(w.symmetries() == (kRot180 | kFlipX | kFlipY | kDiag | kAnti | kRot90));
    w.step();  // two black cells in a row: only the flip across the row remains
    assert(w.symmetries() == kFlipY);
    w.step();
    assert(w.symmetries() == 0);
    w.step();  // step 4: three black cells (corner), symmetric across the diagonal
    assert(w.symmetries() == kDiag);
    w.run(4);  // step 8: six cells around the white start cell: the diagonal, the anti-diagonal and the half turn
    assert(w.symmetries() == (kDiag | kAnti | kRot180));
    w.run(88);  // step 96: only the half turn is left
    assert(w.steps() == 96 && w.symmetries() == kRot180);
    w.run(377);  // step 473
    assert(w.steps() == 473 && w.symmetries() == kRot180);
    w.step();
    assert(w.symmetries() == 0);
    assert(w.lastSymmetric() == 473);
}

void symmetryOtherRules() {
    // (reference) symmetric steps within the first 6000 steps, and the last one
    struct Case {
        const char* rule;
        std::size_t count;
        long last;
    };
    const Case cases[] = {{"RLR", 14, 617}, {"LLRR", 66, 833}, {"RLLR", 34, 417}, {"LRRRRRLLR", 10, 73}, {"LLRRRLRLRLLR", 20, 81}};
    for (const Case& c : cases) {
        const auto s = symmetricSteps(c.rule, 6000, 256);
        assert(s.size() == c.count);
        assert(s.back() == c.last);
    }
}

void phases() {
    World w = make("RL", 256);
    assert(w.phase() == Phase::Symmetry);  // nothing has happened yet
    w.run(729);  // 473 + 256: still within the gap since the last symmetric step
    assert(w.phase() == Phase::Symmetry);
    w.step();
    assert(w.steps() == 730 && w.phase() == Phase::Chaos);
    w.run(4270);
    assert(w.phase() == Phase::Chaos);
    w.run(5392);  // step 10392
    assert(w.steps() == 10392 && w.phase() == Phase::Chaos);
    w.step();
    assert(w.phase() == Phase::Highway);
}

void phaseLog() {
    // One entry per 64 steps: entry i is the phase after step 64 * (i + 1).
    World w = make("RL", 256);
    w.run(10432);
    const auto& log = w.phaseLog();
    assert(log.size() == 163);
    assert(log[0] == Phase::Symmetry);    // step 64
    assert(log[10] == Phase::Symmetry);   // step 704: 231 steps after the last symmetric one
    assert(log[11] == Phase::Chaos);      // step 768
    assert(log[161] == Phase::Chaos);     // step 10368
    assert(log[162] == Phase::Highway);   // step 10432
    // The log stops at kLogBuckets entries.
    w.run(30000);
    assert(w.phaseLog().size() == kLogBuckets);
}

void noFalseHighway() {
    // None of the other rules gets a highway with a period up to 128 within 30000 steps. The detector must not invent one.
    for (const char* rule : {"RLR", "LLRR", "RLLR", "LRRRRRLLR"}) {
        World w = make(rule, 256);
        for (long i = 0; i < 30000; ++i) {
            w.step();
            assert(!w.highway().has_value());
        }
    }
}

void counts() {
    assert(formatCount(0) == "0");
    assert(formatCount(7) == "7");
    assert(formatCount(999) == "999");
    assert(formatCount(1000) == "1,000");
    assert(formatCount(12345) == "12,345");
    assert(formatCount(100000) == "100,000");
    assert(formatCount(1234567) == "1,234,567");
    assert(formatCount(-1500) == "-1,500");
}

void clock() {
    double acc = 0.0;
    assert(stepsDue(acc, 10.0, 0.25) == 2);  // 2.5 steps
    assert(acc == 0.5);
    assert(stepsDue(acc, 10.0, 0.25) == 3);  // 0.5 + 2.5
    assert(acc == 0.0);
    assert(stepsDue(acc, 0.0, 0.25) == 0);
    assert(stepsDue(acc, 100.0, -1.0) == 0);
    assert(stepsDue(acc, 100.0, 10.0) == 25);  // a long stall counts as 0.25 s only
    acc = 0.0;
    long total = 0;
    for (int i = 0; i < 60; ++i) total += stepsDue(acc, 1.0, 1.0 / 64.0);  // 1 step per second at 64 fps: a step every 64 frames
    assert(total == 0);
    for (int i = 0; i < 4; ++i) total += stepsDue(acc, 1.0, 1.0 / 64.0);
    assert(total == 1);
}

}  // namespace

int main() {
    parsing();
    firstSteps();
    wrapping();
    manyColors();
    cellCounts();
    highwayStart();
    torusBreaksHighway();
    torusSizes();
    squareGrowth();
    closedLoopIsNoHighway();
    symmetry();
    symmetryOtherRules();
    phases();
    phaseLog();
    noFalseHighway();
    counts();
    clock();
    std::puts("langtons-ant: all tests passed");
    return 0;
}
