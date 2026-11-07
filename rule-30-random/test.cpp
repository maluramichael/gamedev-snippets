// Tests for the rule 30 logic. Plain asserts on hand-computed cases, no framework, no SDL.
// The row strings of the first rows are worked out by hand from the rule table, the long runs (100,000 bits) are
// checked against numbers that were computed independently with Python big integers.
#undef NDEBUG
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "logic.hpp"

using namespace r30;

namespace {

/// Cells -half..half of the row as a string of 0 and 1.
template <class Row>
std::string rowString(const Row& row, long half) {
    std::string s;
    for (long x = -half; x <= half; ++x) s += row.cell(x) ? '1' : '0';
    return s;
}

template <class Row>
std::string firstBlackCounts(std::size_t rows) {
    Row row(rows);
    std::string out;
    for (std::size_t t = 0; t < rows; ++t) {
        out += std::to_string(row.black()) + ",";
        row.advance();
    }
    return out;
}

void ruleTable() {
    // Wolfram's rule number 30 is 00011110 read over the neighborhoods 111, 110, ..., 000.
    for (int p = 0; p < 2; ++p) {
        for (int q = 0; q < 2; ++q) {
            for (int r = 0; r < 2; ++r) {
                const int index = p * 4 + q * 2 + r;
                assert(rule30(p != 0, q != 0, r != 0) == (((30 >> index) & 1) != 0));
            }
        }
    }
    assert(!rule30(false, false, false));  // an empty neighborhood stays empty
    assert(rule30(true, false, false));    // 100 -> 1
    assert(!rule30(true, true, true));     // 111 -> 0
}

template <class Row>
void rowsByHand() {
    // Worked out by hand: row 1 is 111. Row 2 at x = -2 sees 001 -> 1, x = -1 sees 011 -> 1, x = 0 sees 111 -> 0,
    // x = 1 sees 110 -> 0, x = 2 sees 100 -> 1. Row 3 continues the same way.
    Row row(10);
    assert(rowString(row, 3) == "0001000");
    row.advance();
    assert(rowString(row, 3) == "0011100");
    row.advance();
    assert(rowString(row, 3) == "0110010");  // 1 1 0 0 1 inside x = -2..2
    row.advance();
    assert(rowString(row, 3) == "1101111");
    assert(row.step() == 3);
}

template <class Row>
void blackCountsAreKnown() {
    // Number of black cells per row of the textbook picture, checked against the hand-worked rows above.
    assert(firstBlackCounts<Row>(18) == "1,3,3,6,4,9,5,12,7,12,11,14,12,19,13,22,15,19,");
}

void bitParallelMatchesPerCell() {
    // 300 steps cross four 64 bit words on each side, so every shift carry between words is exercised.
    BitRow bits(300);
    ByteRow bytes(300);
    for (int t = 0; t <= 300; ++t) {
        for (long x = -305; x <= 305; ++x) assert(bits.cell(x) == bytes.cell(x));
        assert(bits.black() == bytes.black());
        assert(bits.step() == bytes.step());
        if (t < 300) {
            assert(bits.advance());
            assert(bytes.advance());
        }
    }
}

template <class Row>
void stepLimit() {
    Row row(5);
    for (int i = 0; i < 5; ++i) assert(row.advance());
    assert(row.step() == 5);
    const std::string before = rowString(row, 6);
    const std::size_t black = row.black();
    assert(!row.advance());  // the plane has no room for a sixth step
    assert(!row.advance());
    assert(row.step() == 5);
    assert(rowString(row, 6) == before);
    assert(row.black() == black);
}

template <class Row>
void outsideTheCone() {
    Row row(100);
    for (int i = 0; i < 10; ++i) row.advance();
    assert(row.cell(10) && row.cell(-10));  // the two edge cells of a row are always black
    assert(!row.cell(11));
    assert(!row.cell(-11));
    assert(!row.cell(1000000));
    assert(!row.cell(-1000000));
    Row fresh(100);
    assert(fresh.cell(0) && !fresh.cell(1) && !fresh.cell(-1));
}

void edgesAreAlwaysBlack() {
    BitRow row(600);
    for (int t = 0; t <= 600; ++t) {
        assert(row.cell(-t) && row.cell(t));
        row.advance();
    }
}

void bitsOfTheCenterColumn() {
    Stream s(Source::Center, Engine::BitParallel, 64);
    std::string bits;
    for (int i = 0; i < 32; ++i) bits += s.next() ? '1' : '0';
    assert(bits.substr(0, 4) == "1101");  // rows 0 to 3 by hand: 1, 1, 0, 1
    assert(bits == "11011100110001011001001110101110");
}

void hundredThousandBits() {
    constexpr std::size_t kBits = 100000;
    Stream s(Source::Center, Engine::BitParallel, kBits);
    BitStats stats;
    BitStats first1000;
    for (std::size_t i = 0; i < kBits; ++i) {
        const bool b = s.next();
        stats.push(b);
        if (i < 1000) first1000.push(b);
    }
    assert(stats.n == kBits);
    assert(stats.ones == 50098);
    assert(stats.longest == 21);
    assert(first1000.ones == 481);  // Wolfram tallies 481 black and 519 white cells in the first 1000
}

void streamEnginesAgree() {
    for (const Source src : {Source::Center, Source::LeftSide}) {
        Stream fast(src, Engine::BitParallel, 2000);
        Stream slow(src, Engine::PerCell, 2000);
        for (int i = 0; i < 2000; ++i) assert(fast.next() == slow.next());
    }
    // The LCG sources do not depend on the engine.
    Stream a(Source::LcgLow, Engine::BitParallel, 100);
    Stream b(Source::LcgLow, Engine::PerCell, 100);
    for (int i = 0; i < 100; ++i) assert(a.next() == b.next());
}

void leftSideIsRegular() {
    // Diagonal k counts k cells in from the left edge of the triangle. The cell of row t is x = k - t.
    constexpr std::size_t kRows = 500;
    BitRow row(kRows);
    std::string d2;
    std::string d3;
    std::string d8;
    for (std::size_t t = 0; t < kRows; ++t) {
        if (t >= 2) d2 += row.cell(2 - static_cast<long>(t)) ? '1' : '0';
        if (t >= 3) d3 += row.cell(3 - static_cast<long>(t)) ? '1' : '0';
        if (t >= 8) d8 += row.cell(8 - static_cast<long>(t)) ? '1' : '0';
        row.advance();
    }
    assert(d2 == std::string(d2.size(), '0'));
    assert(d3.substr(0, 8) == "10101010");
    for (std::size_t i = 0; i < d3.size(); ++i) assert(d3[i] == (i % 2 == 0 ? '1' : '0'));
    for (std::size_t i = 0; i < d8.size(); ++i) assert(d8[i] == "1100"[i % 4]);

    // The stream of diagonal 8: exactly balanced, longest run 2, repeats every 4 bits.
    Stream s(Source::LeftSide, Engine::BitParallel, 2000);
    BitStats stats;
    PeriodFinder pf;
    for (std::size_t i = 0; i < 2000; ++i) {
        const bool b = s.next();
        if (i >= kLeftOffset) stats.push(b);  // the diagonal only exists from row k on
        pf.push(b);
    }
    assert(stats.n == 2000 - kLeftOffset);
    assert(stats.ones == (2000 - kLeftOffset) / 2);  // 1992 bits of 1100 repeating
    assert(stats.longest == 2);
    assert(pf.period() == 4);
}

void triangleSnapshot() {
    const Triangle tri = makeTriangle(18, 20);
    assert(tri.rows == 18 && tri.halfWidth == 20 && tri.cells.size() == 18U * 41U);
    const int expected[18] = {1, 3, 3, 6, 4, 9, 5, 12, 7, 12, 11, 14, 12, 19, 13, 22, 15, 19};
    for (std::size_t t = 0; t < 18; ++t) {
        int black = 0;
        for (long x = -20; x <= 20; ++x) black += tri.at(t, x) ? 1 : 0;
        assert(black == expected[t]);
    }
    assert(tri.at(0, 0) && !tri.at(0, 1));
    assert(!tri.at(0, 21));    // outside the stored window
    assert(!tri.at(99, 0));    // outside the stored rows
}

void bitStats() {
    BitStats s;
    assert(s.n == 0 && s.ones == 0 && s.longest == 0);
    for (const char c : std::string("0011100")) s.push(c == '1');
    assert(s.n == 7);
    assert(s.ones == 3);
    assert(s.longest == 3);
    BitStats one;
    one.push(true);
    assert(one.longest == 1 && one.ones == 1);
    BitStats same;
    for (int i = 0; i < 10; ++i) same.push(false);
    assert(same.longest == 10 && same.ones == 0);
    BitStats alt;
    for (int i = 0; i < 10; ++i) alt.push(i % 2 == 0);
    assert(alt.longest == 1 && alt.ones == 5);
    BitStats late;  // the longest run is not at the end
    for (const char c : std::string("1111010")) late.push(c == '1');
    assert(late.longest == 4);
}

int periodOf(const std::vector<bool>& bits) {
    PeriodFinder pf;
    for (const bool b : bits) pf.push(b);
    return pf.period();
}

void periodFinder() {
    assert(periodOf({}) == 0);
    assert(kPeriodSure == 288);  // 32 warm-up + 64 period + 192 comparisons for the longest period
    // Exactly at that length the longest period is accepted, one bit earlier it is not.
    std::vector<bool> p64edge;
    for (std::size_t i = 0; i < kPeriodSure; ++i) p64edge.push_back((i % 64) < 5 || (i % 64) == 40);
    assert(periodOf(p64edge) == 64);
    p64edge.pop_back();
    assert(periodOf(p64edge) == 0);
    assert(periodOf(std::vector<bool>(300, true)) == 1);
    std::vector<bool> alt;
    for (int i = 0; i < 300; ++i) alt.push_back(i % 2 == 0);
    assert(periodOf(alt) == 2);
    std::vector<bool> three;
    for (int i = 0; i < 300; ++i) three.push_back(i % 3 == 0);
    assert(periodOf(three) == 3);
    // Too little data to be sure: warm-up (32) plus the period plus 32 comparisons, so 65 bits for period 1.
    assert(periodOf(std::vector<bool>(64, true)) == 0);
    assert(periodOf(std::vector<bool>(65, true)) == 1);
    // A messy start inside the warm-up does not matter, 1100 repeating after it gives 4 (and not 2).
    std::vector<bool> late;
    for (int i = 0; i < 20; ++i) late.push_back((i * 7 + 3) % 5 < 2);
    for (int i = 20; i < 400; ++i) late.push_back("1100"[i % 4] == '1');
    assert(periodOf(late) == 4);
    // A single flip after the warm-up breaks every period for good.
    std::vector<bool> glitch(1100, true);
    glitch[1000] = false;
    assert(periodOf(glitch) == 0);
    // A single flip inside the warm-up does not.
    std::vector<bool> early(300, true);
    early[10] = false;
    assert(periodOf(early) == 1);
    // The longest period it looks for.
    std::vector<bool> p64;
    for (int i = 0; i < 1000; ++i) p64.push_back((i % 64) < 5 || (i % 64) == 40);
    assert(periodOf(p64) == 64);
    std::vector<bool> p65;
    for (int i = 0; i < 1000; ++i) p65.push_back((i % 65) < 5 || (i % 65) == 40);
    assert(periodOf(p65) == 0);
}

void randomLookingStreamsHaveNoPeriod() {
    // Checked at many lengths, a coincidence at one length would show up here.
    Stream center(Source::Center, Engine::BitParallel, 5000);
    Stream high(Source::LcgHigh, Engine::BitParallel, 5000);
    PeriodFinder a;
    PeriodFinder b;
    for (int i = 0; i < 5000; ++i) {
        a.push(center.next());
        b.push(high.next());
        assert(a.period() == 0);
        assert(b.period() == 0);
    }
}

void lcg() {
    // 1664525 * 1 + 1013904223 = 1015568748, the rest worked out with 32 bit wrap-around.
    Lcg g;
    const std::uint32_t expected[5] = {1015568748U, 1586005467U, 2165703038U, 3027450565U, 217083232U};
    for (const std::uint32_t e : expected) assert(g.next() == e);
    // The multiplier and the increment are odd, so the lowest bit flips on every call: 0 1 0 1 0.
    Stream low(Source::LcgLow, Engine::BitParallel, 2000);
    for (int i = 0; i < 2000; ++i) assert(low.next() == (i % 2 == 1));
    // Bit 31 of the first five outputs: 0 0 1 1 0.
    Stream high(Source::LcgHigh, Engine::BitParallel, 10);
    const bool expectedHigh[5] = {false, false, true, true, false};
    for (const bool e : expectedHigh) assert(high.next() == e);
    // Perfectly balanced and runs of 1: exactly what the longest-run count is meant to catch.
    Stream again(Source::LcgLow, Engine::BitParallel, 1000);
    BitStats stats;
    PeriodFinder pf;
    for (int i = 0; i < 1000; ++i) {
        const bool b = again.next();
        stats.push(b);
        pf.push(b);
    }
    assert(stats.ones == 500 && stats.longest == 1 && pf.period() == 2);
}

void formatting() {
    assert(formatThousands(0) == "0");
    assert(formatThousands(7) == "7");
    assert(formatThousands(999) == "999");
    assert(formatThousands(1000) == "1,000");
    assert(formatThousands(65536) == "65,536");
    assert(formatThousands(1234567) == "1,234,567");
    assert(formatPercent(50098, 100000) == "50.1");  // 500.98 per mille rounds to 501
    assert(formatPercent(1, 2) == "50.0");
    assert(formatPercent(3, 3) == "100.0");
    assert(formatPercent(0, 7) == "0.0");
    assert(formatPercent(0, 0) == "0.0");  // no bits yet, no division by zero
    assert(expectedLongestRun(65536) == 16);
    assert(expectedLongestRun(1024) == 10);
    assert(expectedLongestRun(100000) == 17);  // log2 = 16.6
    assert(expectedLongestRun(0) == 0);
    assert(expectedLongestRun(1) == 0);
}

}  // namespace

int main() {
    ruleTable();
    rowsByHand<BitRow>();
    rowsByHand<ByteRow>();
    blackCountsAreKnown<BitRow>();
    blackCountsAreKnown<ByteRow>();
    bitParallelMatchesPerCell();
    stepLimit<BitRow>();
    stepLimit<ByteRow>();
    outsideTheCone<BitRow>();
    outsideTheCone<ByteRow>();
    edgesAreAlwaysBlack();
    bitsOfTheCenterColumn();
    hundredThousandBits();
    streamEnginesAgree();
    leftSideIsRegular();
    triangleSnapshot();
    bitStats();
    periodFinder();
    randomLookingStreamsHaveNoPeriod();
    lcg();
    formatting();
    std::puts("rule-30-random logic: all asserts passed");
    return 0;
}
