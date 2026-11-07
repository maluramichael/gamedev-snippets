// Tests for the Wireworld logic and circuits. Plain asserts on hand-computed cases, no framework, no SDL.
#undef NDEBUG
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <string_view>
#include <vector>

#include "circuits.hpp"
#include "logic.hpp"

using namespace ww;

namespace {

const Circuit& circuit(std::string_view id) {
    for (const Circuit& c : circuits()) {
        if (c.id == id) return c;
    }
    std::puts("unknown circuit");
    std::abort();
}

// The demo's field is 60 x 33, presets are centred with rounding down: ox = (60 - width) / 2, oy = (33 - height) / 2.
constexpr int kCols = 60;
constexpr int kRows = 33;

/// A straight wire of `n` cells in row 1 of an (n) x 3 grid.
Grid wireRow(int n) {
    Grid g(n, 3);
    for (int x = 0; x < n; ++x) g.set(x, 1, Cell::Wire);
    return g;
}

int countHeads(const Grid& g) { return static_cast<int>(g.count(Cell::Head)); }

void transitions() {
    assert(nextState(Cell::Empty, 3, Rule::Wireworld) == Cell::Empty);
    assert(nextState(Cell::Head, 0, Rule::Wireworld) == Cell::Tail);
    assert(nextState(Cell::Tail, 8, Rule::Wireworld) == Cell::Wire);
    assert(nextState(Cell::Wire, 0, Rule::Wireworld) == Cell::Wire);
    assert(nextState(Cell::Wire, 1, Rule::Wireworld) == Cell::Head);
    assert(nextState(Cell::Wire, 2, Rule::Wireworld) == Cell::Head);
    assert(nextState(Cell::Wire, 3, Rule::Wireworld) == Cell::Wire);  // the blocking that makes diodes and gates possible
    assert(nextState(Cell::Wire, 8, Rule::Wireworld) == Cell::Wire);
    assert(nextState(Cell::Wire, 3, Rule::AnyHead) == Cell::Head);
    assert(nextState(Cell::Wire, 0, Rule::AnyHead) == Cell::Wire);
    assert(nextState(Cell::Empty, 5, Rule::AnyHead) == Cell::Empty);
}

void neighbourCount() {
    Grid g(3, 3);
    for (int y = 0; y < 3; ++y) {
        for (int x = 0; x < 3; ++x) g.set(x, y, Cell::Head);
    }
    assert(g.headNeighbors(1, 1) == 8);
    assert(g.headNeighbors(0, 0) == 3);  // a corner has three neighbours
    assert(g.headNeighbors(1, 0) == 5);  // an edge cell has five
    g.set(0, 0, Cell::Tail);
    g.set(2, 2, Cell::Wire);
    assert(g.headNeighbors(1, 1) == 6);  // tails and wires are not heads
    // Outside is empty, and writing outside is ignored.
    assert(g.at(-1, 0) == Cell::Empty && g.at(3, 3) == Cell::Empty);
    g.set(5, 5, Cell::Head);
    assert(countHeads(g) == 7);
}

void electronRunsOneCellPerStep() {
    Grid g = wireRow(16);
    g.set(2, 1, Cell::Tail);
    g.set(3, 1, Cell::Head);
    for (int n = 1; n <= 11; ++n) {
        g.step(Rule::Wireworld);
        assert(g.at(3 + n, 1) == Cell::Head);
        assert(g.at(2 + n, 1) == Cell::Tail);
        assert(countHeads(g) == 1 && g.count(Cell::Tail) == 1);
    }
    // Step 12 puts the head on the last cell (x = 15), step 13 leaves only the tail there, step 14 leaves nothing.
    g.step(Rule::Wireworld);
    assert(g.at(15, 1) == Cell::Head && countHeads(g) == 1);
    g.step(Rule::Wireworld);
    assert(countHeads(g) == 0 && g.at(15, 1) == Cell::Tail && !g.quiet());
    g.step(Rule::Wireworld);
    assert(g.quiet());
}

void headTailWireCycle() {
    // A lone head with a wire neighbour: head, tail, wire, and the wire is ready again.
    Grid g(3, 1);
    g.set(0, 0, Cell::Head);
    g.set(1, 0, Cell::Wire);
    g.set(2, 0, Cell::Wire);
    g.step(Rule::Wireworld);
    assert(g.at(0, 0) == Cell::Tail && g.at(1, 0) == Cell::Head && g.at(2, 0) == Cell::Wire);
    g.step(Rule::Wireworld);
    assert(g.at(0, 0) == Cell::Wire && g.at(1, 0) == Cell::Tail && g.at(2, 0) == Cell::Head);
    g.step(Rule::Wireworld);
    assert(g.at(0, 0) == Cell::Wire && g.at(1, 0) == Cell::Wire && g.at(2, 0) == Cell::Tail);
}

void threeHeadsBlock() {
    // Wire in the middle, heads around it. 1 and 2 heads fire, 3 do not.
    auto centreAfter = [](int heads) {
        Grid g(3, 3);
        g.set(1, 1, Cell::Wire);
        const int xs[3] = {0, 2, 0};
        const int ys[3] = {0, 0, 2};
        for (int i = 0; i < heads; ++i) g.set(xs[i], ys[i], Cell::Head);
        g.step(Rule::Wireworld);
        return g.at(1, 1);
    };
    assert(centreAfter(0) == Cell::Wire);
    assert(centreAfter(1) == Cell::Head);
    assert(centreAfter(2) == Cell::Head);
    assert(centreAfter(3) == Cell::Wire);
}

void cornersCutThePath() {
    // A right-angle corner is not 9 steps long but 8: the diagonal neighbour (5,1) fires together with (5,0).
    Grid g(7, 7);
    for (int x = 0; x <= 5; ++x) g.set(x, 0, Cell::Wire);
    for (int y = 1; y <= 5; ++y) g.set(5, y, Cell::Wire);
    g.set(0, 0, Cell::Tail);
    g.set(1, 0, Cell::Head);
    for (int n = 1; n <= 3; ++n) g.step(Rule::Wireworld);
    assert(g.at(4, 0) == Cell::Head && countHeads(g) == 1);
    g.step(Rule::Wireworld);  // step 4: two heads at the corner
    assert(g.at(5, 0) == Cell::Head && g.at(5, 1) == Cell::Head && countHeads(g) == 2);
    for (int n = 5; n <= 7; ++n) {
        g.step(Rule::Wireworld);
        assert(g.at(5, n - 3) == Cell::Head && countHeads(g) == 1);
    }
    g.step(Rule::Wireworld);  // step 8
    assert(g.at(5, 5) == Cell::Head);
}

void inPlaceDependsOnDirection() {
    // Row-major scan, one buffer. An electron going right sees its own tail first and dies, one going left is untouched.
    Grid right = wireRow(12);
    right.set(2, 1, Cell::Tail);
    right.set(3, 1, Cell::Head);
    right.step(Rule::InPlace);
    assert(right.at(3, 1) == Cell::Tail && countHeads(right) == 0);

    Grid left = wireRow(12);
    left.set(9, 1, Cell::Tail);
    left.set(8, 1, Cell::Head);
    for (int n = 1; n <= 8; ++n) {
        left.step(Rule::InPlace);
        assert(left.at(8 - n, 1) == Cell::Head && countHeads(left) == 1);
    }

    // The same by rows: down dies, up works.
    Grid down(3, 12);
    Grid up(3, 12);
    for (int y = 0; y < 12; ++y) {
        down.set(1, y, Cell::Wire);
        up.set(1, y, Cell::Wire);
    }
    down.set(1, 2, Cell::Tail);
    down.set(1, 3, Cell::Head);
    down.step(Rule::InPlace);
    assert(countHeads(down) == 0);
    up.set(1, 9, Cell::Tail);
    up.set(1, 8, Cell::Head);
    up.step(Rule::InPlace);
    assert(up.at(1, 7) == Cell::Head && countHeads(up) == 1);

    // The correct rule runs both ways the same.
    Grid ok = wireRow(12);
    ok.set(2, 1, Cell::Tail);
    ok.set(3, 1, Cell::Head);
    for (int n = 1; n <= 5; ++n) ok.step(Rule::Wireworld);
    assert(ok.at(8, 1) == Cell::Head);
}

void paintingAndLines() {
    Grid g(10, 10);
    g.paint(2, 2, Cell::Head);  // no head on an empty cell
    assert(g.at(2, 2) == Cell::Empty);
    g.paint(2, 2, Cell::Wire);
    g.paint(2, 2, Cell::Head);
    assert(g.at(2, 2) == Cell::Head);
    g.paint(2, 2, Cell::Empty);
    assert(g.at(2, 2) == Cell::Empty);

    // (0,0) to (5,2): six cells, one per column, every step a neighbour of the last.
    Grid l(8, 8);
    l.line(0, 0, 5, 2, Cell::Wire);
    assert(l.count(Cell::Wire) == 6);
    int prevY = 0;
    for (int x = 0; x <= 5; ++x) {
        int hits = 0;
        int hitY = -1;
        for (int y = 0; y < 8; ++y) {
            if (l.at(x, y) == Cell::Wire) {
                ++hits;
                hitY = y;
            }
        }
        assert(hits == 1 && std::abs(hitY - prevY) <= 1);
        prevY = hitY;
    }
    assert(l.at(0, 0) == Cell::Wire && l.at(5, 2) == Cell::Wire);

    // A single point and a backwards line work too.
    Grid p(4, 4);
    p.line(2, 2, 2, 2, Cell::Wire);
    assert(p.count(Cell::Wire) == 1);
    p.line(3, 3, 0, 3, Cell::Wire);
    assert(p.count(Cell::Wire) == 5);
}

void quenching() {
    Grid g = wireRow(8);
    g.set(2, 1, Cell::Tail);
    g.set(3, 1, Cell::Head);
    g.quench();
    assert(g.quiet() && g.count(Cell::Wire) == 8);
}

void placingResolvesPorts() {
    const Circuit& c = circuit("halfadder");
    assert(artWidth(c) == 37 && artHeight(c) == 16);
    const Placed p = place(c, kCols, kRows);
    const int ox = (kCols - 37) / 2;  // 11
    const int oy = (kRows - 16) / 2;  // 8
    assert(ox == 11 && oy == 8);
    // Input A: 'a' is at art (2,0), 'c' at (0,8). Input B: 'b' at (2,2), 'd' at (0,14). Scan order is row by row.
    assert(p.ports.heads.size() == 2 && p.ports.heads[0].size() == 2 && p.ports.heads[1].size() == 2);
    assert(p.ports.heads[0][0].x == ox + 2 && p.ports.heads[0][0].y == oy + 0);
    assert(p.ports.heads[0][1].x == ox + 0 && p.ports.heads[0][1].y == oy + 8);
    assert(p.ports.heads[1][0].x == ox + 2 && p.ports.heads[1][0].y == oy + 2);
    assert(p.ports.heads[1][1].x == ox + 0 && p.ports.heads[1][1].y == oy + 14);
    // Probes: '1' = SUM at art (36,1), '2' = CARRY at art (36,6).
    assert(p.ports.probes.size() == 2);
    assert(p.ports.probes[0].x == ox + 36 && p.ports.probes[0].y == oy + 1);
    assert(p.ports.probes[1].x == ox + 36 && p.ports.probes[1].y == oy + 6);
    assert(p.grid.at(p.ports.probes[0].x, p.ports.probes[0].y) == Cell::Wire);
    assert(p.grid.count(Cell::Head) == 0 && p.grid.count(Cell::Tail) == 0);

    // The diode's tail partners: the clock has none, a letter pair would put a tail next to the head.
    const Circuit& clock = circuit("clock");
    const Placed q = place(clock, kCols, kRows);
    assert(q.grid.count(Cell::Head) == 1 && q.grid.count(Cell::Tail) == 1);  // the electron drawn into the art
    assert(q.ports.probes.size() == 1 && q.ports.heads.empty());
}

void injectionSkipsMissingWire() {
    const Circuit& c = circuit("or");
    Placed p = place(c, kCols, kRows);
    const Port a = p.ports.heads[0][0];
    p.grid.set(a.x, a.y, Cell::Empty);  // the user erased the input cell
    inject(p.grid, p.ports, 3U);
    assert(p.grid.at(a.x, a.y) == Cell::Empty);  // nothing appears out of thin air
    const Port b = p.ports.heads[1][0];
    assert(p.grid.at(b.x, b.y) == Cell::Head);   // the intact input still fires
    assert(countHeads(p.grid) == 1);
}

void clockPeriod() {
    const Circuit& c = circuit("clock");
    // The ring: top row 8 cells, bottom row 8, left side 3, right side 3 (the corners are cut diagonally) = 22 cells.
    int ring = 0;
    for (std::size_t y = 6; y < c.art.size(); ++y) {
        for (const char ch : c.art[y]) {
            if (ch == '#' || ch == 'H' || ch == 'T') ++ring;
        }
    }
    assert(ring == 22);
    const Placed p = place(c, kCols, kRows);
    assert(measurePeriod(p.grid, p.ports.probes[0], 400, Rule::Wireworld) == 22);

    // After exactly one lap the ring looks as it did (the tap on the top row does not disturb it).
    const int ox = (kCols - 12) / 2;
    const int oy = (kRows - 11) / 2;
    Grid g = p.grid;
    for (int t = 0; t < 22; ++t) g.step(Rule::Wireworld);
    for (int y = 6; y < 11; ++y) {
        for (int x = 0; x < 12; ++x) assert(g.at(ox + x, oy + y) == p.grid.at(ox + x, oy + y));
    }
    // Not 21, not 23.
    Grid h = p.grid;
    for (int t = 0; t < 21; ++t) h.step(Rule::Wireworld);
    bool same = true;
    for (int y = 6; y < 11; ++y) {
        for (int x = 0; x < 12; ++x) same = same && h.at(ox + x, oy + y) == p.grid.at(ox + x, oy + y);
    }
    assert(!same);
    // Under the in-place bug the electron dies and the clock stops.
    assert(measurePeriod(p.grid, p.ports.probes[0], 400, Rule::InPlace) == 0);
}

void diodeIsOneWay() {
    const Circuit& c = circuit("diode");
    const Placed p = place(c, kCols, kRows);
    // Forward wire (top): left end a, right end probe 1. Reverse wire (bottom): left end c, right end probe 2.
    const Port topLeft = p.ports.heads[0][0];
    const Port bottomLeft = p.ports.heads[0][1];
    const Port topRight = p.ports.probes[0];
    const Port bottomRight = p.ports.probes[1];

    // Left to right: the top diode passes, the turned-around bottom diode blocks.
    const RunResult r = runInputs(p.grid, p.ports, 1U, c.period, Rule::Wireworld);
    assert(r.pulses.size() == 2 && r.pulses[0] == 1 && r.pulses[1] == 0 && r.quiet);

    // Right to left: head one cell inside the right end, tail on the end cell. Now the bottom diode passes, the top one blocks.
    auto fromRight = [&](Port right, Port left) {
        Grid g = p.grid;
        g.quench();
        g.set(right.x - 1, right.y, Cell::Head);
        g.set(right.x, right.y, Cell::Tail);
        int atLeft = 0;
        for (int t = 0; t < 60; ++t) {
            g.step(Rule::Wireworld);
            if (g.at(left.x, left.y) == Cell::Head) ++atLeft;
        }
        return atLeft;
    };
    assert(fromRight(topRight, topLeft) == 0);
    assert(fromRight(bottomRight, bottomLeft) == 1);

    // The hole costs no time: the electron covers the 36 cells from the left end to probe 1 in 36 steps.
    Grid g = p.grid;
    g.quench();
    inject(g, p.ports, 1U);
    int first = -1;
    for (int t = 1; t <= 60 && first < 0; ++t) {
        g.step(Rule::Wireworld);
        if (g.at(topRight.x, topRight.y) == Cell::Head) first = t;
    }
    assert(first == 36);
}

void orGate() {
    const Circuit& c = circuit("or");
    const Placed p = place(c, kCols, kRows);
    const std::vector<Row> t = truthTable(p.grid, c, p.ports, Rule::Wireworld);
    assert(t.size() == 4);
    const unsigned wantOut[4] = {0, 1, 1, 1};
    for (unsigned i = 0; i < 4; ++i) {
        assert(t[i].in == i && t[i].out == wantOut[i] && t[i].want == wantOut[i] && t[i].ok() && t[i].quiet);
    }
    // Both inputs together give ONE pulse, not two (the junction cell sees two heads and fires once).
    assert(runInputs(p.grid, p.ports, 3U, c.period, Rule::Wireworld).pulses[0] == 1);

    // Backflow: when A fires, the junction also lights up B's wire backwards. With the diodes that electron is swallowed
    // before it reaches B's input cell, without them it runs all the way back.
    const Port bInput = p.ports.heads[1][0];
    auto reachesB = [&](const Placed& pl, Port b) {
        Grid g = pl.grid;
        g.quench();
        inject(g, pl.ports, 1U);  // A only
        int n = 0;
        for (int s = 0; s < 80; ++s) {
            g.step(Rule::Wireworld);
            if (g.at(b.x, b.y) == Cell::Head) ++n;
        }
        return n;
    };
    assert(reachesB(p, bInput) == 0);

    // The same junction without diodes (hand-drawn here): A at row 0, B at row 6, junction (13,3).
    const std::vector<std::string> bare = {
        "a##########",
        "           #",
        "            #",
        "             #################1",
        "            #",
        "           #",
        "b##########",
    };
    Circuit plain;
    plain.art = bare;
    plain.inputs = {{"A", "a"}, {"B", "b"}};
    plain.outputNames = {"OR"};
    plain.expect = expectOr;
    plain.period = 44;
    const Placed q = place(plain, kCols, kRows);
    assert(reachesB(q, q.ports.heads[1][0]) == 1);
    // Fed together it still works as an OR: synchronous inputs do not need the diodes.
    const std::vector<Row> tq = truthTable(q.grid, plain, q.ports, Rule::Wireworld);
    for (unsigned i = 0; i < 4; ++i) assert(tq[i].ok());
}

void xorGate() {
    const Circuit& c = circuit("xor");
    const Placed p = place(c, kCols, kRows);
    const std::vector<Row> t = truthTable(p.grid, c, p.ports, Rule::Wireworld);
    assert(t.size() == 4);
    const unsigned wantOut[4] = {0, 1, 1, 0};
    for (unsigned i = 0; i < 4; ++i) assert(t[i].out == wantOut[i] && t[i].ok() && t[i].quiet);

    // The cancellation needs the electrons to arrive on the same step: one step late and both get through as one pulse.
    auto pulsesWhenBLate = [&](int late) {
        Grid g = p.grid;
        g.quench();
        int n = 0;
        for (int s = 0; s < 80; ++s) {
            if (s == 0) {
                for (const Port& q : p.ports.heads[0]) g.set(q.x, q.y, Cell::Head);
            }
            if (s == late) {
                for (const Port& q : p.ports.heads[1]) g.set(q.x, q.y, Cell::Head);
            }
            g.step(Rule::Wireworld);
            if (g.at(p.ports.probes[0].x, p.ports.probes[0].y) == Cell::Head) ++n;
        }
        return n;
    };
    assert(pulsesWhenBLate(0) == 0);
    assert(pulsesWhenBLate(1) == 1);
    assert(pulsesWhenBLate(2) == 1);
}

void halfAdder() {
    const Circuit& c = circuit("halfadder");
    const Placed p = place(c, kCols, kRows);
    const std::vector<Row> t = truthTable(p.grid, c, p.ports, Rule::Wireworld);
    assert(t.size() == 4);
    // 0+0 = 00, 1+0 = 01, 0+1 = 01, 1+1 = 10 (bit 0 SUM, bit 1 CARRY), so the output bits read as the binary sum.
    const unsigned wantOut[4] = {0, 1, 1, 2};
    for (unsigned i = 0; i < 4; ++i) {
        assert(t[i].in == i && t[i].out == wantOut[i] && t[i].ok() && t[i].clean && t[i].quiet);
        unsigned ones = (i & 1U) + ((i >> 1U) & 1U);
        assert(t[i].out == ones);
    }
    // Every combination is over well before the period ends: the last electron leaves the SUM probe at step 34 and the
    // CARRY probe at step 36.
    for (unsigned in = 0; in < 4; ++in) {
        Grid g = p.grid;
        g.quench();
        inject(g, p.ports, in);
        int lastSum = 0;
        int lastCarry = 0;
        for (int s = 1; s <= c.period; ++s) {
            g.step(Rule::Wireworld);
            if (g.at(p.ports.probes[0].x, p.ports.probes[0].y) == Cell::Head) lastSum = s;
            if (g.at(p.ports.probes[1].x, p.ports.probes[1].y) == Cell::Head) lastCarry = s;
        }
        assert(lastSum == (((in & 1U) ^ ((in >> 1U) & 1U)) != 0U ? 34 : 0));
        assert(lastCarry == (in == 3U ? 36 : 0));
    }
}

void bugRulesBreakTheGates() {
    const Circuit& hc = circuit("halfadder");
    const Placed hp = place(hc, kCols, kRows);
    auto okRows = [](const std::vector<Row>& rows) {
        int n = 0;
        for (const Row& r : rows) n += r.ok() ? 1 : 0;
        return n;
    };
    // The correct rule: 4 of 4. Any-head: only the 0+0 row survives. In place: the same.
    assert(okRows(truthTable(hp.grid, hc, hp.ports, Rule::Wireworld)) == 4);
    assert(okRows(truthTable(hp.grid, hc, hp.ports, Rule::AnyHead)) == 1);
    assert(okRows(truthTable(hp.grid, hc, hp.ports, Rule::InPlace)) == 1);

    // Any head: nothing is ever blocked. The XOR lets 1+1 through, the turned-around diode lets the electron pass.
    const Circuit& xc = circuit("xor");
    const Placed xp = place(xc, kCols, kRows);
    const std::vector<Row> x = truthTable(xp.grid, xc, xp.ports, Rule::AnyHead);
    assert(x[3].out == 1 && !x[3].ok() && x[0].ok() && x[1].ok() && x[2].ok());
    const Circuit& dc = circuit("diode");
    const Placed dp = place(dc, kCols, kRows);
    const std::vector<Row> d = truthTable(dp.grid, dc, dp.ports, Rule::AnyHead);
    assert(d[1].out == 3 && !d[1].ok());

    // In place: the electrons going right die on the first wire cell, so nothing arrives at any probe.
    const std::vector<Row> dIn = truthTable(dp.grid, dc, dp.ports, Rule::InPlace);
    assert(dIn[1].out == 0 && !dIn[1].ok());
    // The OR gate has no blocking in it and survives the any-head bug for all four rows.
    const Circuit& oc = circuit("or");
    const Placed op = place(oc, kCols, kRows);
    for (const Row& r : truthTable(op.grid, oc, op.ports, Rule::AnyHead)) assert(r.ok());
}

void benchInjectsEveryPeriod() {
    const Circuit& c = circuit("halfadder");
    Placed p = place(c, kCols, kRows);
    Machine m(std::move(p.grid), std::move(p.ports), c.period);

    // A only: a SUM pulse after 34 steps, then one more every 44 steps, never a carry.
    m.bits = 1U;
    std::vector<long> sum;
    long carries = 0;
    long lastCarry = -1;
    for (int t = 0; t < 3 * c.period; ++t) {
        const long before = m.lastPulse[0];
        m.tick();
        if (m.lastPulse[0] != before) sum.push_back(m.generation);
        if (m.lastPulse[1] != lastCarry) {
            ++carries;
            lastCarry = m.lastPulse[1];
        }
    }
    assert(sum.size() == 3);
    assert(sum[0] == 34 && sum[1] == 34 + 44 && sum[2] == 34 + 88);
    assert(carries == 0 && m.lastPulse[1] == -1);

    // Both on: carry only, 36 steps in, and the switches act on the next round, not in the middle of one.
    Placed q = place(c, kCols, kRows);
    Machine n(std::move(q.grid), std::move(q.ports), c.period);
    n.bits = 3U;
    for (int t = 0; t < 40; ++t) n.tick();
    assert(n.lastPulse[1] == 36 && n.lastPulse[0] == -1);
    assert(n.grid.quiet());
    n.bits = 0U;  // switched off after the round is over: nothing is injected any more
    for (int t = 0; t < 2 * c.period; ++t) n.tick();
    assert(n.lastPulse[1] == 36);
}

void benchStartAndRestart() {
    const Circuit& c = circuit("halfadder");
    Placed p = place(c, kCols, kRows);
    Machine m(std::move(p.grid), std::move(p.ports), c.period);
    m.bits = 3U;
    m.startRound();  // electrons are in the circuit before the first tick
    assert(countHeads(m.grid) == 4);  // A and B each feed two input cells
    for (int t = 0; t < 36; ++t) m.tick();
    assert(m.lastPulse[1] == 36 && m.lastPulse[0] == -1);  // the same timing as when the first tick injects
    // The second round starts exactly `period` steps after the first: the first injection was at step 0, this one at step 44.
    for (int t = 36; t < 44; ++t) m.tick();
    assert(countHeads(m.grid) == 0);
    m.tick();  // step 45 (the injection happened at the start of this tick, then one step)
    assert(countHeads(m.grid) == 4);

    // restartRound: electrons and lamps gone, the next tick injects right away.
    m.restartRound();
    assert(m.grid.quiet() && m.lastPulse[0] == -1 && m.lastPulse[1] == -1 && m.phase == 0);
    m.tick();
    assert(countHeads(m.grid) == 4);

    // A circuit without bench ignores startRound.
    const Circuit& clock = circuit("clock");
    Placed q = place(clock, kCols, kRows);
    Machine k(std::move(q.grid), std::move(q.ports), clock.period);
    k.startRound();
    assert(k.grid.count(Cell::Head) == 1 && k.phase == 0);
}

void freeDrawingHasNoBench() {
    Machine m(Grid(10, 10), Ports{}, 0);
    m.bits = 7U;
    for (int t = 0; t < 5; ++t) m.tick();
    assert(m.generation == 5 && m.grid.quiet());
    const Circuit& clock = circuit("clock");
    assert(truthTable(Grid(4, 4), clock, Ports{}, Rule::Wireworld).empty());  // no expectation, no table
}

}  // namespace

int main() {
    transitions();
    neighbourCount();
    electronRunsOneCellPerStep();
    headTailWireCycle();
    threeHeadsBlock();
    cornersCutThePath();
    inPlaceDependsOnDirection();
    paintingAndLines();
    quenching();
    placingResolvesPorts();
    injectionSkipsMissingWire();
    clockPeriod();
    diodeIsOneWay();
    orGate();
    xorGate();
    halfAdder();
    bugRulesBreakTheGates();
    benchInjectsEveryPeriod();
    benchStartAndRestart();
    freeDrawingHasNoBench();
    std::puts("wireworld-gates logic: all asserts passed");
    return 0;
}
