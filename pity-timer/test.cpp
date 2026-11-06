// Tests for the pity-timer logic. Plain asserts on hand-computed cases, no framework, no SDL.
// Statistical checks use a fixed seed and wide tolerances (several standard deviations), never exact equality.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>

#include "logic.hpp"

using namespace pity;

namespace {

bool near(double a, double b, double eps = 1e-9) { return std::fabs(a - b) < eps; }

// 0.99^100, worked out once with a calculator: the chance that 1 % per try leaves you empty after 100 tries.
constexpr double kEmptyAfter100 = 0.36603234127;

void randomModel() {
    const Model m = makeModel(Mode::Random, 0.01, 100);
    assert(near(m.chance(1), 0.01));
    assert(near(m.chance(57), 0.01));
    assert(near(m.chance(5000), 0.01));
    const Exact e = exact(m);
    assert(near(e.pmf[1], 0.01));
    assert(near(e.pmf[2], 0.99 * 0.01));
    assert(near(e.stillEmpty(0), 1.0));
    assert(near(e.stillEmpty(1), 0.99));
    assert(near(e.stillEmpty(100), kEmptyAfter100, 1e-9));
    assert(near(e.mean, 100.0, 1e-6));  // 1 / p
    // Nothing special happens at 100: after 100 tries a player is no closer to a drop than after 1.
    assert(near(e.stillEmpty(200), kEmptyAfter100 * kEmptyAfter100, 1e-9));
}

void halfChance() {
    const Exact e = exact(makeModel(Mode::Random, 0.5, 100));
    assert(near(e.pmf[1], 0.5));
    assert(near(e.pmf[2], 0.25));
    assert(near(e.pmf[3], 0.125));
    assert(near(e.stillEmpty(3), 0.125));
    assert(near(e.mean, 2.0, 1e-9));
}

void clampedInputs() {
    assert(near(makeModel(Mode::Random, 2.0, 10).chance(1), 1.0));
    assert(near(makeModel(Mode::Random, -1.0, 10).chance(1), 0.0));
    assert(near(makeModel(Mode::Random, 1.0, 10).chance(1), 1.0));
    assert(makeModel(Mode::Pity, 0.1, 0).hardPity == 1);  // a guarantee at try 1 at the earliest
    assert(near(makeModel(Mode::Pity, 0.1, 0).chance(1), 1.0));
    assert(near(makeModel(Mode::Prd, 0.0, 10).chance(7), 0.0));
    assert(near(makeModel(Mode::Prd, 1.0, 10).chance(1), 1.0));
}

void softStartValues() {
    assert(softStart(100) == 75);
    assert(softStart(4) == 3);
    assert(softStart(3) == 2);
    assert(softStart(2) == 1);
    assert(softStart(1) == 1);
}

void pityHandComputed() {
    // Base 10 %, hard pity 4: soft pity starts at 3. Chance at try n:
    //   1: 0.1   2: 0.1   3: 0.1 + 0.9 * 1/2 = 0.55   4: 1 (guarantee)   5+: 1
    const Model m = makeModel(Mode::Pity, 0.1, 4);
    assert(near(m.chance(1), 0.1));
    assert(near(m.chance(2), 0.1));
    assert(near(m.chance(3), 0.55));
    assert(near(m.chance(4), 1.0));
    assert(near(m.chance(5), 1.0));
    const Exact e = exact(m);
    assert(near(e.pmf[1], 0.1));
    assert(near(e.pmf[2], 0.09));
    assert(near(e.pmf[3], 0.81 * 0.55));         // 0.4455
    assert(near(e.pmf[4], 0.81 * 0.45));         // 0.3645
    assert(near(e.pmfAt(5), 0.0));                 // nobody gets past the guarantee
    assert(near(e.mean, 0.1 + 0.18 + 3 * 0.4455 + 4 * 0.3645));  // 3.0745
    assert(near(e.mean, 3.0745));
    assert(near(e.stillEmpty(4), 0.0));
}

void pityRaisesTheRealRate() {
    // The same nominal 1 % with a guarantee at 100: the average wait is far below 100.
    const Exact e = exact(makeModel(Mode::Pity, 0.01, 100));
    assert(e.mean < 80.0);
    assert(e.stillEmpty(100) == 0.0);
    assert(near(e.stillEmpty(74), std::pow(0.99, 74), 1e-9));  // identical to plain random until the soft start
}

void prdConstants() {
    // C = 0.5: chances 0.5, 1.0. Expected tries 1 + 0.5 = 1.5, so the overall rate is 2/3.
    assert(near(prdOverallChance(0.5), 2.0 / 3.0));
    // C = 1/3: chances 1/3, 2/3, 1. Expected tries 1 + 2/3 + (2/3)(1/3) = 17/9, overall rate 9/17.
    assert(near(prdOverallChance(1.0 / 3.0), 9.0 / 17.0));
    assert(near(prdOverallChance(1.0), 1.0));
    assert(near(prdConstant(2.0 / 3.0), 0.5, 1e-9));
    assert(near(prdConstant(9.0 / 17.0), 1.0 / 3.0, 1e-9));
    assert(near(prdConstant(1.0), 1.0));
    assert(near(prdConstant(0.0), 0.0));
    for (const double p : {0.005, 0.01, 0.05, 0.25, 0.5, 0.9}) {
        const double c = prdConstant(p);
        assert(near(prdOverallChance(c), p, 1e-9));
        assert(c > 0.0 && c < p);  // it has to start lower than the target to average out at it
    }
    assert(prdConstant(0.25) < prdConstant(0.5));
}

void prdModel() {
    const Model m = makeModel(Mode::Prd, 0.25, 100);
    assert(near(m.chance(1), m.prdC));
    assert(near(m.chance(3), 3.0 * m.prdC));
    assert(near(m.chance(1000), 1.0));
    const Exact e = exact(m);
    assert(near(e.mean, 4.0, 1e-6));  // the point of PRD: the long-run rate is exactly the nominal one
    const int cap = static_cast<int>(std::ceil(1.0 / m.prdC));
    assert(e.pmfAt(cap) > 0.0);
    assert(near(e.stillEmpty(cap), 0.0));
    // The famous quirk: the very first try is much worse than the nominal rate.
    assert(m.chance(1) < 0.25 / 2.0);
    // And the price: at the nominal wait (100 tries at 1 %) more players are still empty than with plain random
    // (45 % against 37 %). PRD narrows the spread around the mean, it does not move the mean.
    const double empty = exact(makeModel(Mode::Prd, 0.01, 100)).stillEmpty(100);
    assert(empty > 0.44 && empty < 0.46);
}

void distributionsSumToOne() {
    for (const Mode mode : {Mode::Random, Mode::Prd, Mode::Pity}) {
        for (const double rate : {0.005, 0.01, 0.2}) {
            const Exact e = exact(makeModel(mode, rate, 50));
            double total = 0.0;
            for (const double p : e.pmf) total += p;
            assert(near(total + e.stillEmpty(static_cast<int>(e.pmf.size()) - 1), 1.0, 1e-9));
            assert(near(e.pmf[0], 0.0));
        }
    }
}

void bins() {
    // p = 0.5, two tries per bin: [1,2] = 0.5 + 0.25, [3,4] = 0.125 + 0.0625.
    const Exact e = exact(makeModel(Mode::Random, 0.5, 100));
    const auto b = binProbabilities(e, 2, 2);
    assert(b.size() == 2);
    assert(near(b[0], 0.75));
    assert(near(b[1], 0.1875));
    // A bin range longer than the table is padded with zeros, not read out of bounds.
    const Exact pity = exact(makeModel(Mode::Pity, 0.1, 4));
    const auto c = binProbabilities(pity, 10, 3);
    assert(c.size() == 3);
    assert(near(c[0], 1.0));
    assert(near(c[1], 0.0) && near(c[2], 0.0));
}

void layoutValues() {
    const Layout a = layoutFor(0.01);  // 5 / p = 500 tries shown, 10 per bar
    assert(a.binWidth == 10 && a.bins == 50);
    const Layout b = layoutFor(0.2);  // 25 tries, one per bar
    assert(b.binWidth == 1 && b.bins == 25);
    const Layout c = layoutFor(0.005);
    assert(c.binWidth == 20 && c.bins == 50);
    const Layout d = layoutFor(0.03);  // 5 / 0.03 = 166.67 -> 167, 4 per bar
    assert(d.binWidth == 4 && d.bins == 42);
    assert(layoutFor(1.0).binWidth == 1 && layoutFor(1.0).bins == 5);
    assert(layoutFor(0.0).bins >= 1);  // never zero bars
}

void marks() {
    assert(markAttempts(0.01) == 100);
    assert(markAttempts(0.03) == 33);
    assert(markAttempts(0.2) == 5);
    assert(markAttempts(1.0) == 1);
    assert(markAttempts(0.0) >= 1);
}

void rngBasics() {
    Rng a(42);
    Rng b(42);
    Rng c(43);
    bool differs = false;
    for (int i = 0; i < 100; ++i) {
        const double x = a.uniform();
        assert(x >= 0.0 && x < 1.0);
        assert(x == b.uniform());
        if (x != c.uniform()) differs = true;
    }
    assert(differs);
    Rng d(7);
    double sum = 0.0;
    for (int i = 0; i < 100000; ++i) sum += d.uniform();
    assert(std::fabs(sum / 100000.0 - 0.5) < 0.01);
}

void rollBounds() {
    Rng rng(1);
    const Model sure = makeModel(Mode::Random, 1.0, 10);
    for (int i = 0; i < 100; ++i) assert(rollPlayer(sure, rng) == 1);
    const Model pity = makeModel(Mode::Pity, 0.1, 4);
    int seenMax = 0;
    for (int i = 0; i < 20000; ++i) {
        const int w = rollPlayer(pity, rng);
        assert(w >= 1 && w <= 4);
        seenMax = std::max(seenMax, w);
    }
    assert(seenMax == 4);
    const Model prd = makeModel(Mode::Prd, 0.25, 100);
    const int cap = static_cast<int>(std::ceil(1.0 / prd.prdC));
    for (int i = 0; i < 20000; ++i) assert(rollPlayer(prd, rng) <= cap);
    // A rate of zero can never drop. The roll must stop at the cap instead of looping for ever.
    assert(rollPlayer(makeModel(Mode::Random, 0.0, 10), rng) == kMaxAttempts);
}

std::vector<int> simulate(const Model& m, int players, std::uint64_t seed) {
    Rng rng(seed);
    std::vector<int> waits;
    for (int i = 0; i < players; ++i) waits.push_back(rollPlayer(m, rng));
    return waits;
}

void simulationMatchesTheMath() {
    // 10,000 players at 1 %: about 36.6 % still have nothing after 100 tries (sigma of that share is 0.5 points).
    const auto waits = simulate(makeModel(Mode::Random, 0.01, 100), 10000, 2024);
    const Summary s = summarize(waits, 100);
    assert(s.count == 10000);
    assert(std::fabs(s.shareAbove - kEmptyAfter100) < 0.03);
    assert(std::fabs(s.mean - 100.0) < 6.0);  // sigma of the mean is 1
    // PRD: the mean wait is the nominal one, even though the shape of the distribution changed.
    const Model prd = makeModel(Mode::Prd, 0.01, 100);
    const auto prdWaits = simulate(prd, 10000, 2025);
    const Summary sp = summarize(prdWaits, 100);
    assert(std::fabs(sp.mean - 100.0) < 3.0);
    // What PRD removes is the tail, not the share at the mean: 3x the nominal wait is rare for PRD (exact: below
    // 0.1 %), but 0.99^300, about 4.9 %, for plain random.
    assert(exact(prd).stillEmpty(300) < 0.001);
    assert(summarize(prdWaits, 300).shareAbove < 0.01);
    assert(summarize(simulate(makeModel(Mode::Random, 0.01, 100), 10000, 2027), 300).shareAbove > 0.03);
    // Pity: same check against its own exact mean.
    const Model pity = makeModel(Mode::Pity, 0.01, 100);
    const Summary sy = summarize(simulate(pity, 10000, 2026), 100);
    assert(std::fabs(sy.mean - exact(pity).mean) < 2.0);
    assert(sy.max <= 100);
    assert(sy.shareAbove == 0.0);
}

void summaryHandComputed() {
    const std::vector<int> waits{10, 1, 2, 9, 3, 8, 4, 7, 5, 6};  // unsorted on purpose
    const Summary s = summarize(waits, 5);
    assert(s.count == 10);
    assert(near(s.mean, 5.5));
    assert(s.median == 5);  // sorted[ceil(0.5 * 10) - 1]
    assert(s.p95 == 10);    // sorted[ceil(0.95 * 10) - 1]
    assert(s.max == 10);
    assert(near(s.shareAbove, 0.5));  // 6..10 are above 5
    const Summary none = summarize({}, 5);
    assert(none.count == 0 && near(none.mean, 0.0) && none.max == 0 && near(none.shareAbove, 0.0));
    const Summary one = summarize({7}, 7);
    assert(one.median == 7 && one.p95 == 7 && near(one.shareAbove, 0.0));  // 7 is not above 7
}

void histogramHandComputed() {
    const std::vector<int> waits{1, 2, 3, 4, 5, 11};
    int overflow = -1;
    const auto h = histogram(waits, 2, 3, overflow);  // bars [1,2] [3,4] [5,6]
    assert(h.size() == 3);
    assert(h[0] == 2 && h[1] == 2 && h[2] == 1);
    assert(overflow == 1);
    const auto z = histogram({}, 2, 3, overflow);
    assert(z.size() == 3 && z[0] == 0 && overflow == 0);
}

void niceCeilValues() {
    assert(near(niceCeil(0.0), 1.0));
    assert(near(niceCeil(1.0), 1.0));
    assert(near(niceCeil(1.2), 2.0));
    assert(near(niceCeil(2.0), 2.0));
    assert(near(niceCeil(3.0), 5.0));
    assert(near(niceCeil(5.1), 10.0));
    assert(near(niceCeil(100.0), 100.0));
    assert(near(niceCeil(100.5), 200.0));
    assert(near(niceCeil(870.0), 1000.0));
    assert(near(niceCeil(1000.0), 1000.0));
    assert(near(niceCeil(2100.0), 5000.0));
}

void thousandsSeparator() {
    assert(thousands(0) == "0");
    assert(thousands(999) == "999");
    assert(thousands(1000) == "1,000");
    assert(thousands(10000) == "10,000");
    assert(thousands(1234567) == "1,234,567");
    assert(thousands(-1234) == "-1,234");
}

}  // namespace

int main() {
    randomModel();
    halfChance();
    clampedInputs();
    softStartValues();
    pityHandComputed();
    pityRaisesTheRealRate();
    prdConstants();
    prdModel();
    distributionsSumToOne();
    bins();
    layoutValues();
    marks();
    rngBasics();
    rollBounds();
    simulationMatchesTheMath();
    summaryHandComputed();
    histogramHandComputed();
    niceCeilValues();
    thousandsSeparator();
    std::puts("pity-timer logic: all asserts passed");
    return 0;
}
