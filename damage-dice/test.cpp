// Tests for the dice logic. Plain asserts on hand-computed cases, no framework, no SDL.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "logic.hpp"

using namespace dd;

namespace {

bool near(double a, double b, double eps = 1e-9) { return std::fabs(a - b) < eps; }

/// A random source that hands out prepared values, so a test can force exact branches. at() throws when it runs dry.
struct Script {
    std::vector<std::uint32_t> values;
    std::size_t next_ = 0;
    std::uint32_t next() { return values.at(next_++); }
};

double sum(const Distribution& d) {
    double s = 0.0;
    for (const double p : d.p) s += p;
    return s;
}

void generator() {
    // Reference values of the PCG32 demo program: pcg32_srandom_r(&rng, 42, 54).
    Pcg32 rng;
    rng.seed(42, 54);
    assert(rng.next() == 0xa15c02b7U);
    assert(rng.next() == 0x7b47f409U);
    assert(rng.next() == 0xba1d3330U);
    assert(rng.next() == 0x83d2f293U);
    assert(rng.next() == 0xbfa4784bU);
    assert(rng.next() == 0xcbed606eU);
    // Same seed, same stream. Another stream differs.
    Pcg32 a, b, c;
    a.seed(7, 1);
    b.seed(7, 1);
    c.seed(7, 2);
    bool differs = false;
    for (int i = 0; i < 8; ++i) {
        const std::uint32_t x = a.next();
        assert(x == b.next());
        differs = differs || x != c.next();
    }
    assert(differs);
}

void rejectionThresholds() {
    // 2^32 mod n, written out: 2^32 = 4294967296.
    assert(rejectionThreshold(1) == 0);
    assert(rejectionThreshold(2) == 0);
    assert(rejectionThreshold(8) == 0);
    assert(rejectionThreshold(3) == 1);   // 4294967295 = 3 * 1431655765
    assert(rejectionThreshold(6) == 4);   // 4294967292 = 6 * 715827882
    assert(rejectionThreshold(7) == 4);   // 2^3 = 1 mod 7, so 2^32 = 2^30 * 4 = 4 mod 7
    assert(rejectionThreshold(20) == 16);
    assert(rejectionThreshold(0) == 0);   // guard, not a real die
}

void uniformBelowRejects() {
    // n = 6: values below 4 are thrown away, the first kept value is reduced modulo 6.
    Script s{{3, 4, 10}};
    assert(uniformBelow(s, 6) == 4);
    assert(s.next_ == 2);
    // Nothing to draw for a die with one face, or fewer.
    Script none{{}};
    assert(uniformBelow(none, 1) == 0);
    assert(uniformBelow(none, 0) == 0);
    // A power of two never rejects.
    Script p{{0, 13}};
    assert(uniformBelow(p, 8) == 0);
    assert(uniformBelow(p, 8) == 5);
}

void diceCounts() {
    const Distribution d6 = diceDistribution(1, 6);
    assert(d6.lo == 1 && d6.hi() == 6 && d6.p.size() == 6);
    for (int v = 1; v <= 6; ++v) assert(near(d6.at(v), 1.0 / 6.0));

    // 2d4: sums 2..8 occur 1, 2, 3, 4, 3, 2, 1 times out of 16.
    const Distribution d4 = diceDistribution(2, 4);
    assert(d4.lo == 2 && d4.hi() == 8);
    const int counts[] = {1, 2, 3, 4, 3, 2, 1};
    for (int i = 0; i < 7; ++i) assert(near(d4.at(2 + i), counts[i] / 16.0));

    // 2d6: 36 outcomes, six of them make a 7.
    const Distribution two = diceDistribution(2, 6);
    assert(two.lo == 2 && two.hi() == 12);
    assert(near(two.at(7), 6.0 / 36.0));
    assert(near(two.at(6), 5.0 / 36.0));
    assert(near(two.at(2), 1.0 / 36.0) && near(two.at(12), 1.0 / 36.0));

    // 3d6: 216 outcomes. 10 and 11 make 27, 9 and 12 make 25, the ends make 1.
    const Distribution three = diceDistribution(3, 6);
    assert(three.lo == 3 && three.hi() == 18 && three.p.size() == 16);
    assert(near(three.at(3), 1.0 / 216.0) && near(three.at(18), 1.0 / 216.0));
    assert(near(three.at(10), 27.0 / 216.0) && near(three.at(11), 27.0 / 216.0));
    assert(near(three.at(9), 25.0 / 216.0) && near(three.at(12), 25.0 / 216.0));
    assert(near(sum(three), 1.0));
    // Outside the range there is nothing.
    assert(three.at(2) == 0.0 && three.at(19) == 0.0 && three.at(-50) == 0.0);

    // Degenerate dice: a die with one face is a constant, nonsense sizes are clamped to one.
    const Distribution fixed = diceDistribution(3, 1);
    assert(fixed.lo == 3 && fixed.p.size() == 1 && near(fixed.p[0], 1.0));
    const Distribution clamped = diceDistribution(0, 0);
    assert(clamped.lo == 1 && clamped.p.size() == 1 && near(clamped.p[0], 1.0));

    // The biggest case the demo offers still adds up: 10d20 runs from 10 to 200.
    const Distribution big = diceDistribution(10, 20);
    assert(big.lo == 10 && big.hi() == 200 && big.p.size() == 191);
    assert(near(sum(big), 1.0, 1e-9));
}

void meanAndSpread() {
    // Mean N * (S + 1) / 2, variance N * (S^2 - 1) / 12.
    const Distribution three = diceDistribution(3, 6);
    assert(near(mean(three), 10.5));
    assert(near(variance(three), 105.0 / 12.0));  // 3 * (36 - 1) / 12
    const Distribution d20 = diceDistribution(1, 20);
    assert(near(mean(d20), 10.5));
    assert(near(variance(d20), 399.0 / 12.0));
    assert(near(stddev(d20), std::sqrt(33.25)));
    const Distribution big = diceDistribution(10, 20);
    assert(near(mean(big), 105.0, 1e-7));
    assert(near(variance(big), 332.5, 1e-6));
    // More dice with the same range means less spread: 1d12 vs 2d6 vs 3d4 vs 4d3 vs 6d2 all reach about 12.
    assert(stddev(diceDistribution(2, 6)) < stddev(diceDistribution(1, 12)));
    assert(stddev(diceDistribution(6, 2)) < stddev(diceDistribution(2, 6)));
}

void flatAndWeights() {
    // One roll over 3..18: 16 values, each 1/16, variance (16^2 - 1) / 12.
    const Distribution flat = flatDistribution(3, 18);
    assert(flat.lo == 3 && flat.p.size() == 16);
    assert(near(flat.at(3), 0.0625) && near(flat.at(18), 0.0625));
    assert(near(mean(flat), 10.5));
    assert(near(variance(flat), 255.0 / 12.0));
    // Same mean as 3d6 but a much wider spread: that is the point of the demo.
    assert(stddev(flat) > 1.5 * stddev(diceDistribution(3, 6)));
    const Distribution swapped = flatDistribution(5, 2);  // empty range collapses to its first value
    assert(swapped.lo == 5 && swapped.p.size() == 1 && near(swapped.p[0], 1.0));

    // Weights 2, 0, 6 on the values 1, 2, 3.
    const Distribution w = weightDistribution(1, {2, 0, 6});
    assert(w.lo == 1 && w.p.size() == 3);
    assert(near(w.at(1), 0.25) && near(w.at(2), 0.0) && near(w.at(3), 0.75));
    // Negative weights count as zero, a table without any weight is a constant instead of a division by zero.
    const Distribution neg = weightDistribution(1, {-3, 2});
    assert(near(neg.at(1), 0.0) && near(neg.at(2), 1.0));
    const Distribution empty = weightDistribution(4, {0, 0, 0});
    assert(empty.lo == 4 && empty.p.size() == 1 && near(empty.p[0], 1.0));
    const Distribution none = weightDistribution(4, {});
    assert(none.lo == 4 && none.p.size() == 1 && near(none.p[0], 1.0));
}

void successChance() {
    const Distribution three = diceDistribution(3, 6);
    assert(near(atLeast(three, 12), 81.0 / 216.0));  // 25 + 21 + 15 + 10 + 6 + 3 + 1
    assert(near(atLeast(three, 3), 1.0));
    assert(near(atLeast(three, -100), 1.0));
    assert(near(atLeast(three, 4), 215.0 / 216.0));
    assert(near(atLeast(three, 18), 1.0 / 216.0));
    assert(near(atLeast(three, 19), 0.0));
    assert(near(atLeast(diceDistribution(1, 20), 11), 0.5));

    // A bonus is a shift: 3d6 + 2 reaches 14 exactly as often as 3d6 reaches 12.
    const Distribution plus2 = shifted(three, 2);
    assert(plus2.lo == 5 && plus2.hi() == 20);
    assert(near(atLeast(plus2, 14), atLeast(three, 12)));
    assert(near(mean(plus2), 12.5) && near(variance(plus2), variance(three)));
    const Distribution minus3 = shifted(three, -3);
    assert(minus3.lo == 0 && near(atLeast(minus3, 1), 1.0 - 1.0 / 216.0));
}

void bonusIsWorth() {
    // d20: a +1 is worth 5 points for every target from 2 to 21, nothing below and above.
    const Distribution d20 = diceDistribution(1, 20);
    assert(near(bonusWorth(d20, 1), 0.0));
    for (int t = 2; t <= 21; ++t) assert(near(bonusWorth(d20, t), 0.05));
    assert(near(bonusWorth(d20, 22), 0.0));
    // 3d6: 27/216 = 12.5 points in the middle, 6/216 at 17, 1/216 at 19, none at 20.
    const Distribution three = diceDistribution(3, 6);
    assert(near(bonusWorth(three, 11), 27.0 / 216.0));
    assert(near(bonusWorth(three, 17), 6.0 / 216.0));
    assert(near(bonusWorth(three, 19), 1.0 / 216.0));
    assert(near(bonusWorth(three, 20), 0.0));
    assert(near(bonusWorth(three, 3), 0.0));
    // It is the difference of the success chance with and without the extra point.
    for (int t = -2; t <= 25; ++t) {
        assert(near(bonusWorth(three, t), atLeast(shifted(three, 1), t) - atLeast(three, t)));
    }
    // Range over all targets that move at all.
    const WorthRange flat = bonusWorthRange(d20);
    assert(near(flat.least, 0.05) && near(flat.most, 0.05));
    const WorthRange bell = bonusWorthRange(three);
    assert(near(bell.least, 1.0 / 216.0) && near(bell.most, 27.0 / 216.0));
    const WorthRange table = bonusWorthRange(weightDistribution(1, {2, 0, 6}));
    assert(near(table.least, 0.25) && near(table.most, 0.75));
}

void rolling() {
    // Three d6 from prepared values: 6 % 6 -> 1, 11 % 6 -> 6, 13 % 6 -> 2.
    Script s{{6, 11, 13}};
    assert(rollDice(s, 3, 6) == 9);
    // Flat over 3..18 draws one value out of 16 (no rejection: 16 divides 2^32).
    Script f{{5, 15, 16}};
    assert(rollFlat(f, 3, 18) == 8);
    assert(rollFlat(f, 3, 18) == 18);
    assert(rollFlat(f, 3, 18) == 3);
    Script none{{}};
    assert(rollFlat(none, 7, 7) == 7);
    assert(rollFlat(none, 9, 2) == 9);

    // Only the lowest byte counts for the 8 bit source: 0x113 -> 0x13 = 19 -> face 20.
    Script m{{0x113, 255}};
    assert(rollModulo8(m, 2, 20) == 20 + 16);  // 255 % 20 = 15 -> face 16
}

void moduloBias() {
    // Run every byte through d20: 256 = 12 * 20 + 16, so the faces 1..16 come up 13 times and 17..20 only 12 times.
    int face20[21] = {};
    int face8[9] = {};
    int face6[7] = {};
    for (std::uint32_t b = 0; b < 256; ++b) {
        Script s{{b}};
        ++face20[rollModulo8(s, 1, 20)];
        Script t{{b}};
        ++face8[rollModulo8(t, 1, 8)];
        Script u{{b}};
        ++face6[rollModulo8(u, 1, 6)];
    }
    for (int f = 1; f <= 16; ++f) assert(face20[f] == 13);
    for (int f = 17; f <= 20; ++f) assert(face20[f] == 12);
    for (int f = 1; f <= 8; ++f) assert(face8[f] == 32);  // a power of two divides 256: no bias at all
    for (int f = 1; f <= 4; ++f) assert(face6[f] == 43);   // 256 = 42 * 6 + 4
    for (int f = 5; f <= 6; ++f) assert(face6[f] == 42);
}

void tables() {
    Table t;
    t.lo = 1;
    t.weights = {2, 0, 6};
    t.rebuild();
    assert(t.total() == 8);
    assert((t.cumulative == std::vector<std::uint32_t>{2, 2, 8}));
    // pick: 0 and 1 land in the first slot, 2 to 7 in the third, the zero weight is never chosen.
    const std::uint32_t raw[] = {0, 1, 2, 7, 8};
    const int expect[] = {1, 1, 3, 3, 1};
    for (int i = 0; i < 5; ++i) {
        Script s{{raw[i]}};
        assert(t.pick(s) == expect[i]);
    }

    assert(t.setWeight(1, 5));
    assert(t.total() == 13);
    assert((t.cumulative == std::vector<std::uint32_t>{2, 7, 13}));
    // Weights are clamped to 0..99.
    assert(t.setWeight(0, 500) && t.weights[0] == 99);
    assert(t.setWeight(0, -4) && t.weights[0] == 0);
    // The last weight cannot go: a table with total 0 has nothing to pick.
    assert(t.setWeight(1, 0));
    assert(!t.setWeight(2, 0));
    assert(t.weights[2] == 6 && t.total() == 6);
    assert(!t.setWeight(3, 1));
    assert(!t.setWeight(-1, 1));
    assert(t.total() == 6);
}

void presets() {
    const Table flat = flatTable();
    assert(flat.lo == 1 && flat.weights.size() == 20 && flat.total() == 20);
    const Table ramp = rampTable();
    assert(ramp.weights.front() == 20 && ramp.weights.back() == 1 && ramp.total() == 210);
    const Table crit = critTable();
    assert(crit.weights.size() == 20 && crit.total() == 48);
    assert(crit.weights[0] == 2);                                   // value 1: a graze
    for (int v = 6; v <= 10; ++v) assert(crit.weights[v - 1] == 8);  // 6..10: the normal hit
    assert(crit.weights[19] == 6);                                  // value 20: the crit
    assert(crit.weights[1] == 0 && crit.weights[10] == 0 && crit.weights[18] == 0);
}

void histogram() {
    Histogram h(1, 6);
    assert(h.total() == 0);
    // Nothing rolled yet: no division by zero.
    assert(near(h.mean(), 0.0) && near(h.variance(), 0.0) && near(h.prob(3), 0.0) && near(h.fractionAtLeast(1), 0.0));
    assert(!h.add(0) && !h.add(7) && !h.add(-3));
    assert(h.total() == 0);
    assert(h.add(1) && h.add(2) && h.add(3));
    assert(h.total() == 3 && h.count(2) == 1);
    assert(near(h.mean(), 2.0));
    assert(near(h.variance(), 2.0 / 3.0));  // ((-1)^2 + 0 + 1^2) / 3
    h.add(3);
    assert(near(h.prob(3), 0.5));
    assert(near(h.fractionAtLeast(3), 0.5));
    assert(near(h.fractionAtLeast(1), 1.0));
    assert(near(h.fractionAtLeast(4), 0.0));
    assert(near(h.fractionAtLeast(-5), 1.0));
    assert(near(h.prob(99), 0.0));
    h.reset();
    assert(h.total() == 0 && h.count(3) == 0);
}

void statistics() {
    // 200,000 rolls of 3d6 with a fixed seed. The tolerances are about five standard errors.
    Pcg32 rng;
    rng.seed(2024, 11);
    const Distribution ideal = diceDistribution(3, 6);
    Histogram dice(ideal.lo, static_cast<int>(ideal.p.size()));
    Histogram flat(ideal.lo, static_cast<int>(ideal.p.size()));
    for (int i = 0; i < 200000; ++i) {
        dice.add(rollDice(rng, 3, 6));
        flat.add(rollFlat(rng, 3, 18));
    }
    assert(near(dice.mean(), 10.5, 0.05));
    assert(near(dice.variance(), 8.75, 0.15));
    assert(near(dice.fractionAtLeast(12), 0.375, 0.01));
    assert(near(dice.prob(10), 27.0 / 216.0, 0.01));
    // The flat roll has the same mean and a far wider spread, and passes the target more often.
    assert(near(flat.mean(), 10.5, 0.1));
    assert(near(flat.variance(), 21.25, 0.3));
    assert(near(flat.fractionAtLeast(12), 7.0 / 16.0, 0.01));

    // Modulo on a byte: the faces 1..16 of a d20 are 8 percent too frequent, 13/256 against 12/256.
    Histogram d20(1, 20);
    for (int i = 0; i < 300000; ++i) d20.add(rollModulo8(rng, 1, 20));
    assert(near(d20.prob(1), 13.0 / 256.0, 0.002));
    assert(near(d20.prob(20), 12.0 / 256.0, 0.002));
}

void formatting() {
    assert(formatCount(0) == "0");
    assert(formatCount(7) == "7");
    assert(formatCount(999) == "999");
    assert(formatCount(1000) == "1,000");
    assert(formatCount(12345) == "12,345");
    assert(formatCount(100000) == "100,000");
    assert(formatCount(1000000) == "1,000,000");
    assert(formatCount(1234567890123ULL) == "1,234,567,890,123");

    assert(expression(3, 6, 0) == "3d6");
    assert(expression(3, 6, 2) == "3d6 + 2");
    assert(expression(1, 20, -3) == "1d20 - 3");
    assert(expression(10, 20, 10) == "10d20 + 10");
}

void axisAndTicks() {
    const Window w = windowFor(3, 18, 10);
    assert(w.lo == -7 && w.hi == 28 && w.bins() == 36);
    const Window big = windowFor(10, 200, 10);
    assert(big.lo == 0 && big.hi == 210 && big.bins() == 211);

    const Axis a{-9, 40, 40.0, 880.0};
    assert(near(a.binWidth(), 22.0));
    assert(near(a.left(-9), 40.0) && near(a.left(0), 238.0) && near(a.left(31), 920.0));
    assert(near(a.center(1), 40.0 + 10 * 22.0 + 11.0));
    assert(*a.valueAt(40.0) == -9);
    assert(*a.valueAt(61.99) == -9);
    assert(*a.valueAt(62.0) == -8);
    assert(*a.valueAt(919.99) == 30);
    assert(!a.valueAt(920.0));
    assert(!a.valueAt(39.9));
    assert(!a.valueAt(-100.0));

    // Label step: the smallest one that keeps the labels 36 px apart.
    assert(near(niceStep(22.0, 36.0), 2.0));
    assert(near(niceStep(4.2, 36.0), 10.0));
    assert(near(niceStep(40.0, 36.0), 1.0));
    assert(near(niceStep(0.5, 36.0), 100.0));
    assert(near(niceStep(0.001, 36.0), 100.0));  // nothing fits: the biggest step
    assert(near(niceStep(80.0, 36.0, 0.5), 0.5));
    assert(near(niceStep(80.0, 36.0), 1.0));
}

}  // namespace

int main() {
    generator();
    rejectionThresholds();
    uniformBelowRejects();
    diceCounts();
    meanAndSpread();
    flatAndWeights();
    successChance();
    bonusIsWorth();
    rolling();
    moduloBias();
    tables();
    presets();
    histogram();
    statistics();
    formatting();
    axisAndTicks();
    std::puts("damage-dice logic: all checks passed");
    return 0;
}
