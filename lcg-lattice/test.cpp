// Tests for the lcg-lattice logic. Plain asserts, no framework, no SDL.
//
// Hand-computed cases come first (the tiny LCG is small enough to do on paper). The published sequences of the real
// generators (C standard sample rand(), Microsoft rand(), minstd, xorshift32, PCG32) pin the implementations down
// to the bit; each of them was compared against a reference value from its own documentation or from the book.
#undef NDEBUG
#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <set>
#include <vector>

#include "logic.hpp"

using namespace lcg;

namespace {

bool near(double a, double b) { return std::fabs(a - b) < 1e-12; }

std::vector<std::uint32_t> outputs(Kind kind, std::uint32_t seed, std::size_t n) { return draw(make(kind, seed), n); }

std::vector<std::uint8_t> bits(Kind kind, std::uint32_t seed, std::size_t n, int k) {
    return bitSequence(outputs(kind, seed, n), k);
}

void tinyByHand() {
    // x -> 65 x + 1 mod 4096. From 1: 66, then 66 * 65 + 1 = 4291 = 4096 + 195, then 195 * 65 + 1 = 12676 = 3 * 4096 + 388.
    Rng r = make(Kind::Tiny, 1);
    assert(next(r) == 66);
    assert(next(r) == 195);
    assert(next(r) == 388);
    // From 0 the first value is just the increment.
    Rng z = make(Kind::Tiny, 0);
    assert(next(z) == 1);
    assert(next(z) == 66);
    assert(outBits(Kind::Tiny) == 12);
}

void tinyFullPeriod() {
    // a - 1 = 64 is a multiple of 4 and c is odd, so the generator visits all 4096 values before it repeats.
    const auto v = outputs(Kind::Tiny, 0, 4096);
    const std::set<std::uint32_t> distinct(v.begin(), v.end());
    assert(distinct.size() == 4096);
    const auto w = outputs(Kind::Tiny, 0, 8192);
    for (std::size_t i = 0; i < 4096; ++i) assert(w[i] == w[i + 4096]);
    // Perfectly uniform over one period: 4096 values in 256 bins of 16 values each, chi-square is exactly 0.
    assert(near(chiSquare(Kind::Tiny, v, 256), 0.0));
}

void tinyLivesOnLines() {
    // Every pair (x, y) satisfies y = 65 x + 1 - 4096 k for an integer k = floor((65 x + 1) / 4096). The largest x is
    // 4095, so k runs 0..64: all 4096 points sit on exactly 65 parallel lines.
    const auto v = outputs(Kind::Tiny, 0, 4097);
    std::set<std::uint32_t> lines;
    for (std::size_t i = 0; i + 1 < v.size(); ++i) {
        const std::uint32_t k = (65 * v[i] + 1) / 4096;
        assert(65 * v[i] + 1 - 4096 * k == v[i + 1]);
        lines.insert(k);
    }
    assert(lines.size() == 65);
}

void lowBitsRepeatFast() {
    // Bit k of a power-of-two LCG with a = 1 mod 4 and odd c repeats every 2^(k+1) draws (Knuth, and the book says the
    // same). Bit 0 flips on every call.
    const auto outs = outputs(Kind::Tiny, 5, 4096);
    for (int k = 0; k <= 9; ++k) {
        assert(smallestPeriod(bitSequence(outs, k), 1024) == (std::size_t{1} << (k + 1)));
    }
    // Bit 10 repeats only every 2048 draws, past the search limit of 1024.
    assert(smallestPeriod(bitSequence(outs, 10), 1024) == 0);
    const auto b0 = bitSequence(outs, 0);
    assert(near(sameAsPrevious(b0), 0.0));
    assert(longestRun(b0) == 1);
}

void diyKeepsTheWholeState() {
    // x -> 1664525 x + 1013904223 mod 2^32, returned as it is. From 0 the first value is the increment.
    Rng r = make(Kind::Diy32, 0);
    assert(next(r) == 1013904223U);
    assert(outBits(Kind::Diy32) == 32);
    const auto outs = outputs(Kind::Diy32, 7, 4096);
    assert(near(sameAsPrevious(bitSequence(outs, 0)), 0.0));
    assert(smallestPeriod(bitSequence(outs, 1), 1024) == 4);  // 1664524 is a multiple of 4
    assert(smallestPeriod(bitSequence(outs, 2), 1024) == 8);
    assert(smallestPeriod(bitSequence(outs, 31), 1024) == 0);
    // x % 6 only looks at x % 2 and x % 3, so with an alternating parity a die roll alternates odd and even forever.
    for (std::size_t i = 0; i + 1 < outs.size(); ++i) assert(outs[i] % 6 % 2 != outs[i + 1] % 6 % 2);
}

void realGeneratorsMatchTheirReferences() {
    // C standard sample rand(), seed 1: the first value is (1103515245 + 12345) >> 16 = 1103527590 >> 16 = 16838.
    const std::vector<std::uint32_t> ansi{16838, 5758, 10113, 17515, 31051};
    assert(outputs(Kind::Ansi, 1, ansi.size()) == ansi);
    assert(outBits(Kind::Ansi) == 15);
    // Microsoft rand(), seed 1: (214013 + 2531011) >> 16 = 2745024 >> 16 = 41.
    const std::vector<std::uint32_t> msvc{41, 18467, 6334, 26500, 19169};
    assert(outputs(Kind::Msvc, 1, msvc.size()) == msvc);
    // Park-Miller minstd: 16807, then 16807^2 = 282475249, then 1622650073. The C++ standard names the 10000th value.
    const std::vector<std::uint32_t> pm{16807, 282475249, 1622650073};
    assert(outputs(Kind::ParkMiller, 1, pm.size()) == pm);
    assert(outputs(Kind::ParkMiller, 1, 10000).back() == 1043618065U);
    // xorshift32 from 1: 1 ^ (1 << 13) = 8193, 8193 ^ (8193 >> 17) = 8193, 8193 ^ (8193 << 5) = 270369.
    const std::vector<std::uint32_t> xs{270369, 67634689, 2647435461U, 307599695};
    assert(outputs(Kind::Xorshift32, 1, xs.size()) == xs);
    // PCG32 reference demo, initstate 42, initseq 54.
    const std::vector<std::uint32_t> pcg{0xa15c02b7U, 0x7b47f409U, 0xba1d3330U, 0x83d2f293U, 0xbfa4784bU, 0xcbed606eU};
    assert(outputs(Kind::Pcg32, 42, pcg.size()) == pcg);
}

void theShiftHidesTheLowBits() {
    // rand() returns state >> 16, so output bit 0 is state bit 16, which repeats every 2^17 draws instead of every 2.
    // That is the whole trick of the C library: it throws the bad bits away. 15 bits are left.
    constexpr std::size_t kPeriod = std::size_t{1} << 17;
    for (const Kind kind : {Kind::Ansi, Kind::Msvc}) {
        const auto b0 = bits(kind, 1, 3 * kPeriod, 0);
        assert(smallestPeriod(b0, kPeriod) == kPeriod);
        const auto head = std::vector<std::uint8_t>(b0.begin(), b0.begin() + 100000);
        assert(std::fabs(sameAsPrevious(head) - 0.5) < 0.02);
    }
}

void wolfensteinTable() {
    // The table printed in the Black Book: 256 entries, "no 1s and two 2s" (Carmack's remark there).
    int ones = 0;
    int twos = 0;
    for (const std::uint8_t v : kWolfTable) {
        if (v == 1) ++ones;
        if (v == 2) ++twos;
    }
    assert(kWolfTable.size() == 256);
    assert(ones == 0 && twos == 2);
    assert(kWolfTable[0] == 0 && kWolfTable[1] == 8 && kWolfTable[2] == 109 && kWolfTable[255] == 249);
    // US_RndT increments the index first, so from index 0 the first value is table[1].
    Rng r = make(Kind::Wolf, 0);
    assert(next(r) == 8);
    assert(next(r) == 109);
    // After 256 calls the index is back at 0 and the whole sequence repeats.
    const auto outs = outputs(Kind::Wolf, 0, 512);
    for (std::size_t i = 0; i < 256; ++i) assert(outs[i] == outs[i + 256]);
    assert(outBits(Kind::Wolf) == 8);
    // Every bit of the output repeats within 256 flips.
    const auto periods = bitPeriods(outs, 8, 1024);
    for (int k = 0; k < 8; ++k) {
        assert(periods[static_cast<std::size_t>(k)] != 0);
        assert(256 % periods[static_cast<std::size_t>(k)] == 0);
    }
    assert(periods[8] == 0);
}

void goodGeneratorsHaveNoBitPeriods() {
    for (const Kind kind : {Kind::Xorshift32, Kind::Pcg32, Kind::ParkMiller, Kind::Ansi, Kind::Msvc}) {
        const auto outs = outputs(kind, 1, 4096);
        const auto periods = bitPeriods(outs, outBits(kind), 1024);
        for (const std::size_t p : periods) assert(p == 0);
    }
    // The tiny LCG is the contrast: bits 0..9 all repeat within 1024 draws, bits 10 and 11 do not.
    const auto periods = bitPeriods(outputs(Kind::Tiny, 5, 4096), 12, 1024);
    for (std::size_t k = 0; k < 10; ++k) assert(periods[k] == (std::size_t{1} << (k + 1)));
    assert(periods[10] == 0 && periods[11] == 0);
    assert(periods[12] == 0);  // beyond the output width stays 0
}

void oneDimensionalUniformityMissesTheLattice() {
    // Over 256 bins and 256000 draws the expected count is 1000 per bin; chi-square has mean 255 and sd about 23.
    // The bound 400 is 6 sd away, so a healthy generator passes with a wide margin. The weak generators pass as well:
    // a histogram cannot see the lattice, that needs the pairs.
    for (const Kind kind : {Kind::Xorshift32, Kind::Pcg32, Kind::ParkMiller, Kind::Ansi, Kind::Msvc, Kind::Diy32}) {
        const auto outs = outputs(kind, 1, 256000);
        const double chi = chiSquare(kind, outs, 256);
        assert(chi < 400.0);
    }
}

void fairCoinStatistics() {
    for (const Kind kind : {Kind::Xorshift32, Kind::Pcg32, Kind::ParkMiller}) {
        for (const int k : {0, outBits(kind) - 1}) {
            const auto seq = bits(kind, 1, 100000, k);
            assert(std::fabs(sameAsPrevious(seq) - 0.5) < 0.01);
            const std::size_t run = longestRun(seq);
            assert(run >= 12 && run <= 24);  // log2(100000) is 16.6
        }
    }
}

void statisticHelpers() {
    assert(smallestPeriod({}, 8) == 0);
    assert(smallestPeriod({0, 1, 0, 1, 0, 1, 0, 1, 0, 1}, 0) == 0);
    assert(smallestPeriod({0, 1, 0, 1, 0, 1, 0, 1, 0, 1}, 8) == 2);
    assert(smallestPeriod({1, 1, 1, 1, 1, 1}, 8) == 1);
    assert(smallestPeriod({0, 0, 1, 0, 0, 1, 0, 0, 1}, 8) == 3);
    assert(smallestPeriod({0, 1, 0, 1, 0, 1, 0, 0}, 8) == 0);  // one flipped value at the end breaks every period
    assert(smallestPeriod({0, 1, 1}, 8) == 0);                 // only periods up to half the length can be judged
    assert(near(sameAsPrevious({0, 1, 0, 1}), 0.0));
    assert(near(sameAsPrevious({1, 1, 1, 1}), 1.0));
    assert(near(sameAsPrevious({0, 0, 1, 1}), 2.0 / 3.0));
    assert(near(sameAsPrevious({1}), 0.0));
    assert(near(sameAsPrevious({}), 0.0));
    assert(longestRun({}) == 0);
    assert(longestRun({1}) == 1);
    assert(longestRun({0, 0, 1, 1, 1, 0}) == 3);
    assert(near(chiSquare(Kind::Tiny, {}, 4), 0.0));
    assert(near(chiSquare(Kind::Tiny, {1, 2}, 0), 0.0));
    // Eight values below 8 / 4096 fall into the lower of two bins: counts 8 and 0, expected 4 each,
    // chi-square = 4^2 / 4 + 4^2 / 4 = 8.
    assert(near(chiSquare(Kind::Tiny, {0, 1, 2, 3, 4, 5, 6, 7}, 2), 8.0));
    // Bit access, including the guard for bit indices past the word.
    assert(bitOf(0b1010U, 0) == 0 && bitOf(0b1010U, 1) == 1 && bitOf(0b1010U, 3) == 1);
    assert(bitOf(0x80000000U, 31) == 1);
    assert(bitOf(0xffffffffU, 32) == 0);
    assert(bitOf(0xffffffffU, -1) == 0);
}

void unitStaysBelowOne() {
    assert(near(unit(Kind::Tiny, 2048), 0.5));
    assert(unit(Kind::Tiny, 4095) < 1.0);
    assert(unit(Kind::Ansi, 32767) < 1.0);
    assert(unit(Kind::ParkMiller, kParkMillerM - 1) < 1.0);
    assert(unit(Kind::ParkMiller, 1) > 0.0);
    assert(unit(Kind::Xorshift32, 0xffffffffU) < 1.0);
    assert(unit(Kind::Pcg32, 0) >= 0.0);
    assert(unit(Kind::Wolf, 255) < 1.0);
}

void seedsAreSafe() {
    // xorshift32 never leaves 0, the multiplicative LCG never leaves 0 either: both must refuse a zero seed.
    Rng x = make(Kind::Xorshift32, 0);
    assert(next(x) != 0);
    for (const std::uint32_t seed : {0U, 1U, 2147483646U, 2147483647U, 0xffffffffU}) {
        for (const std::uint32_t v : outputs(Kind::ParkMiller, seed, 1000)) assert(v >= 1 && v < kParkMillerM);
    }
    // A seed larger than the state is reduced, never out of range.
    for (const std::uint32_t v : outputs(Kind::Tiny, 0xffffffffU, 100)) assert(v < 4096);
    for (const std::uint32_t v : outputs(Kind::Msvc, 0xffffffffU, 100)) assert(v < 32768);
    for (const std::uint32_t v : outputs(Kind::Wolf, 0xffffffffU, 600)) assert(v <= 255);
    // Different seeds give different streams.
    assert(outputs(Kind::Pcg32, 1, 4) != outputs(Kind::Pcg32, 2, 4));
}

void seedNumbers() {
    assert(seedFor(0) == 1U);
    std::set<std::uint32_t> seeds;
    for (std::uint32_t n = 0; n < 1000; ++n) seeds.insert(seedFor(n));
    assert(seeds.size() == 1000);
}

void pairsAreConsecutive() {
    // From seed 0 the tiny LCG yields 1, 66, 4291 mod 4096 = 195, ...: the first points are (1, 66) and (66, 195), scaled to 0..1.
    const auto p = collectPairs(make(Kind::Tiny, 0), 1.0, 100, 1000);
    assert(p.size() == 100);
    assert(near(p[0].x, 1.0 / 4096.0) && near(p[0].y, 66.0 / 4096.0));
    assert(near(p[1].x, 66.0 / 4096.0) && near(p[1].y, 195.0 / 4096.0));
    for (const Point& q : p) assert(q.x >= 0.0 && q.x < 1.0 && q.y >= 0.0 && q.y < 1.0);
}

void pairsStopAfterOneCycle() {
    // One full cycle of the tiny LCG is 4096 pairs, the closing one (last value, first value) included.
    // Ask for far more than exist: the search stops by itself instead of repeating points.
    const auto all = collectPairs(make(Kind::Tiny, 0), 1.0, 100000, 1000000);
    assert(all.size() == 4096);
    std::set<std::pair<double, double>> distinct;
    for (const Point& q : all) distinct.insert({q.x, q.y});
    assert(distinct.size() == 4096);
    // The Wolfenstein table has 256 values and so 256 pairs.
    assert(collectPairs(make(Kind::Wolf, 0), 1.0, 100000, 1000000).size() == 256);
}

void zoomKeepsOnlyTheStrip() {
    // x < 0.001 means output < 4.096 for the tiny LCG: the values 0..4, each once per cycle, with y = 65 x + 1.
    const auto strip = collectPairs(make(Kind::Tiny, 0), 0.001, 100000, 1000000);
    assert(strip.size() == 5);
    std::set<long> ys;
    for (const Point& q : strip) {
        assert(q.x < 0.001);
        ys.insert(std::lround(q.y * 4096.0));
    }
    assert((ys == std::set<long>{1, 66, 131, 196, 261}));
}

void dieRolls() {
    // Top bits scaled to 0..5: 0 maps to 0, the largest output to 5, the middle of the range to 3 (2^31 * 6 / 2^32 = 3).
    assert(scaled(0, 32, 6) == 0);
    assert(scaled(0xffffffffU, 32, 6) == 5);
    assert(scaled(0x80000000U, 32, 6) == 3);
    assert(scaled(4095, 12, 6) == 5);  // 4095 * 6 = 24570, and 24570 >> 12 = 5
    assert(scaled(2048, 12, 6) == 3);
    assert(scaled(12345, 32, 1) == 0);
    assert(scaled(12345, 32, 0) == 0);
    assert(scaled(12345, 0, 6) == 0);
    assert(scaled(12345, 33, 6) == 0);
    assert(scaled(0xffffffffU, 8, 6) == 5);  // an output wider than the stated bits is clamped, never out of range
    // Both ways give a fair histogram over many rolls (chi-square with 5 degrees of freedom, 20.5 is the 0.1 % point), ...
    const auto outs = outputs(Kind::Diy32, 1, 60000);
    std::array<int, 6> naive{};
    std::array<int, 6> top{};
    int naiveFlips = 0;
    int topFlips = 0;
    for (std::size_t i = 0; i < outs.size(); ++i) {
        const std::uint32_t a = outs[i] % 6;
        const std::uint32_t b = scaled(outs[i], 32, 6);
        assert(a < 6 && b < 6);
        ++naive[a];
        ++top[b];
        if (i > 0) {
            naiveFlips += (a % 2 != outs[i - 1] % 6 % 2) ? 1 : 0;
            topFlips += (b % 2 != scaled(outs[i - 1], 32, 6) % 2) ? 1 : 0;
        }
    }
    for (const auto* faces : {&naive, &top}) {
        double chi = 0.0;
        for (const int c : *faces) chi += (c - 10000.0) * (c - 10000.0) / 10000.0;
        assert(chi < 20.5);
    }
    // ... but the naive roll alternates between odd and even on every single step, the scaled one flips about half of the time.
    assert(naiveFlips == 59999);
    assert(topFlips > 28000 && topFlips < 32000);
}

void statePairsAreExact() {
    // Tiny, strip x < 0.001: the states 0..4, the same five dots as the long way round.
    const auto direct = statePairs(Kind::Tiny, 0.001, 100, 7);
    assert(direct.size() == 5);
    for (std::size_t s = 0; s < 5; ++s) {
        assert(near(direct[s].x, static_cast<double>(s) / 4096.0));
        assert(near(direct[s].y, static_cast<double>(65 * s + 1) / 4096.0));
    }
    // The whole square of the tiny LCG is its 4096 states, with no repeats.
    assert(statePairs(Kind::Tiny, 1.0, 100000, 7).size() == 4096);
    // The 32 bit LCG: states below 1e-5 * 2^32 = 42949.67, so 0..42949 (ceil gives the exclusive end 42950).
    // Every dot satisfies y * 2^32 = (1664525 x + 1013904223) mod 2^32 exactly.
    const auto diy = statePairs(Kind::Diy32, 1e-5, 500, 1);
    assert(diy.size() == 500);
    for (const Point& p : diy) {
        assert(p.x < 1e-5);
        const auto s = static_cast<std::uint64_t>(std::llround(p.x * 4294967296.0));
        const auto want = static_cast<std::uint32_t>(static_cast<std::uint32_t>(s) * 1664525U + 1013904223U);
        assert(std::llround(p.y * 4294967296.0) == static_cast<long long>(want));
    }
    // Park-Miller: y * m = 16807 s mod m, and the state 0 is not part of the set.
    for (const Point& p : statePairs(Kind::ParkMiller, 1e-4, 500, 1)) {
        const auto s = static_cast<std::uint64_t>(std::llround(p.x * 2147483647.0));
        assert(s >= 1);
        assert(std::llround(p.y * 2147483647.0) == static_cast<long long>(16807ULL * s % 2147483647ULL));
    }
    // Same seed, same dots; another seed, other dots.
    assert(statePairs(Kind::Diy32, 1e-5, 50, 1)[0].x == statePairs(Kind::Diy32, 1e-5, 50, 1)[0].x);
    assert(statePairs(Kind::Diy32, 1e-5, 50, 1)[0].x != statePairs(Kind::Diy32, 1e-5, 50, 2)[0].x);
    // Edges: no strip, no dots, and generators that hide their state have no direct form.
    assert(statePairs(Kind::Diy32, 0.0, 50, 1).empty());
    assert(statePairs(Kind::Diy32, -1.0, 50, 1).empty());
    assert(statePairs(Kind::Diy32, 1.0, 0, 1).empty());
    assert(statePairs(Kind::Ansi, 1.0, 50, 1).empty());
    assert(statePairs(Kind::Pcg32, 1.0, 50, 1).empty());
    assert(statePairs(Kind::Msvc, 1.0, 50, 1).empty());
    assert(statePairs(Kind::Wolf, 1.0, 50, 1).empty());
    assert(statePairs(Kind::Xorshift32, 1.0, 50, 1).empty());  // scanned instead, see outputIsState
    assert(statePairs(Kind::ParkMiller, 1e-12, 50, 1).empty());  // 1e-12 * m < 1: state 0 only, which is not a state
    assert(outputIsState(Kind::Tiny) && outputIsState(Kind::Diy32) && outputIsState(Kind::ParkMiller));
    assert(!outputIsState(Kind::Ansi) && !outputIsState(Kind::Pcg32) && !outputIsState(Kind::Xorshift32));
}

void pairLimits() {
    assert(collectPairs(make(Kind::Pcg32, 1), 1.0, 50, 1000000).size() == 50);
    assert(collectPairs(make(Kind::Pcg32, 1), 1.0, 50, 0).empty());
    assert(collectPairs(make(Kind::Pcg32, 1), 0.0, 50, 1000).empty());
    assert(collectPairs(make(Kind::Pcg32, 1), -1.0, 50, 1000).empty());
    assert(collectPairs(make(Kind::Pcg32, 1), 1.0, 0, 1000).empty());
    // A tight strip with a small budget returns what it found, never more than the budget allows.
    const auto few = collectPairs(make(Kind::Pcg32, 1), 1e-6, 50, 1000);
    assert(few.size() <= 1000);
}

}  // namespace

int main() {
    tinyByHand();
    tinyFullPeriod();
    tinyLivesOnLines();
    lowBitsRepeatFast();
    diyKeepsTheWholeState();
    realGeneratorsMatchTheirReferences();
    theShiftHidesTheLowBits();
    wolfensteinTable();
    goodGeneratorsHaveNoBitPeriods();
    oneDimensionalUniformityMissesTheLattice();
    fairCoinStatistics();
    statisticHelpers();
    unitStaysBelowOne();
    seedsAreSafe();
    seedNumbers();
    pairsAreConsecutive();
    pairsStopAfterOneCycle();
    zoomKeepsOnlyTheStrip();
    pairLimits();
    statePairsAreExact();
    dieRolls();
    std::puts("lcg-lattice logic: all tests passed");
    return 0;
}
