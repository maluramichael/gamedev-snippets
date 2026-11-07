// Tests for the pick-an-action logic. Plain asserts on hand-computed cases, no framework, no SDL.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>

#include "logic.hpp"

using namespace pick;

namespace {

bool near(double a, double b, double eps = 1e-9) { return std::fabs(a - b) < eps; }

// Action order: 0 Eat, 1 Snack, 2 Sleep, 3 Play, 4 Chat, 5 Watch TV.
// Needs order: hunger, tired, bored, lonely.

void optionsFromNeeds() {
    const auto o = options({0.9, 0.5, 0.6, 0.3});
    assert(o.size() == kActions);
    assert(near(o[0].weight, 0.9) && o[0].rank == 2);   // hunger 0.9 is critical
    assert(near(o[1].weight, 0.36) && o[1].rank == 1);  // snack is 0.4 of the hunger and never critical
    assert(near(o[2].weight, 0.5) && o[2].rank == 1);
    assert(near(o[3].weight, 0.6) && o[3].rank == 1);
    assert(near(o[4].weight, 0.3) && o[4].rank == 1);
    assert(near(o[5].weight, 0.25) && o[5].rank == 0);  // the filler: constant weight, lowest rank
}

void criticalThreshold() {
    assert(options({0.8, 0, 0, 0})[0].rank == 2);   // the threshold itself counts as critical
    assert(options({0.79, 0, 0, 0})[0].rank == 1);
    assert(options({0, 0.8, 0, 0})[2].rank == 2);
    assert(options({0, 0, 0.8, 0})[3].rank == 2);
    assert(options({0, 0, 0, 0.8})[4].rank == 2);
}

void needsAreClamped() {
    const auto o = options({1.7, -0.4, 0.0, 0.0});
    assert(near(o[0].weight, 1.0));  // clamped to 1
    assert(near(o[2].weight, 0.0));  // clamped to 0
}

void maxTakesTheBest() {
    const auto p = probabilities(options({0.9, 0.5, 0.6, 0.3}), Rule::Max);
    assert(near(p[0], 1.0));
    for (std::size_t i = 1; i < p.size(); ++i) assert(near(p[i], 0.0));
}

void maxSplitsTies() {
    // Sleep and Play both 0.5, Eat 0.2: a tie must not favour the lower index.
    const auto p = probabilities(options({0.2, 0.5, 0.5, 0.1}), Rule::Max);
    assert(near(p[2], 0.5) && near(p[3], 0.5));
    assert(near(p[0], 0.0) && near(p[5], 0.0));
}

void weightedIsProportional() {
    const auto p = probabilities(options({0.9, 0.5, 0.6, 0.3}), Rule::Weighted);
    const double sum = 0.9 + 0.36 + 0.5 + 0.6 + 0.3 + 0.25;  // 2.91
    assert(near(p[0], 0.9 / sum));
    assert(near(p[1], 0.36 / sum));
    assert(near(p[5], 0.25 / sum));
    double total = 0;
    for (double x : p) total += x;
    assert(near(total, 1.0));
}

void weightedLetsTheSatedEat() {
    // Hunger 0.1, everything else 0: Eat still has a chance, it just is small.
    const auto p = probabilities(options({0.1, 0, 0, 0}), Rule::Weighted);
    assert(p[0] > 0.0);
    assert(near(p[0], 0.1 / (0.1 + 0.04 + 0.25)));
}

void bandKeepsOnlyThePlausible() {
    // Best is Eat 0.9, cut-off 0.9 * 0.7 = 0.63: Play 0.6 is out.
    const auto p = probabilities(options({0.9, 0.5, 0.6, 0.3}), Rule::Band, 0.3);
    assert(near(p[0], 1.0));
    // Play 0.7 clears the cut-off: the two share by weight.
    const auto q = probabilities(options({0.9, 0.5, 0.7, 0.3}), Rule::Band, 0.3);
    assert(near(q[0], 0.9 / 1.6) && near(q[3], 0.7 / 1.6));
    assert(near(q[2], 0.0));
}

void bandBoundaryIsInside() {
    // Exactly 70 percent of the best stays in, with a hair of tolerance for the float product.
    const auto p = probabilities({{1.0, 1}, {0.7, 1}, {0.69, 1}}, Rule::Band, 0.3);
    assert(near(p[0], 1.0 / 1.7) && near(p[1], 0.7 / 1.7));
    assert(near(p[2], 0.0));
}

void bandWidthZeroIsMax() {
    const auto p = probabilities({{1.0, 1}, {0.99, 1}}, Rule::Band, 0.0);
    assert(near(p[0], 1.0) && near(p[1], 0.0));
}

void dualRankBeatsWeight() {
    // A tiny weight at rank 2 wins over a huge weight at rank 1, every time.
    const std::vector<Option> o{{0.01, 2}, {1.0, 1}};
    const auto p = probabilities(o, Rule::Dual);
    assert(near(p[0], 1.0) && near(p[1], 0.0));
    for (int i = 0; i < 1000; ++i) {
        const auto choice = pickFrom(p, static_cast<double>(i) / 1000.0);
        assert(choice && *choice == 0);
    }
}

void dualStarvingNeverWatchesTv() {
    const auto p = probabilities(options({0.95, 0.55, 0.5, 0.3}), Rule::Dual);
    assert(near(p[0], 1.0));
    assert(near(p[5], 0.0));
}

void dualWeightsWithinTheRank() {
    // Eat 0.85 and Play 0.9 are both critical: they share 0.85 : 0.9, everything else is out.
    const auto p = probabilities(options({0.85, 0.15, 0.9, 0.4}), Rule::Dual);
    assert(near(p[0], 0.85 / 1.75));
    assert(near(p[3], 0.9 / 1.75));
    assert(near(p[2], 0.0) && near(p[4], 0.0) && near(p[5], 0.0));  // Sleep 0.15, Chat 0.4 and TV are rank 1 or 0
}

void dualZeroWeightFallsThrough() {
    // The rank 2 option has weight 0: it is invalid and the next rank decides.
    const std::vector<Option> o{{0.0, 2}, {0.5, 1}, {0.5, 1}, {0.9, 0}};
    const auto p = probabilities(o, Rule::Dual);
    assert(near(p[0], 0.0) && near(p[1], 0.5) && near(p[2], 0.5) && near(p[3], 0.0));
}

void dualFillerOnlyWhenNothingElse() {
    const auto p = probabilities(options({0, 0, 0, 0}), Rule::Dual);
    assert(near(p[5], 1.0));  // only the filler has a weight
}

void zeroWeightIsNeverPicked() {
    const std::vector<Option> o{{0.0, 1}, {0.5, 1}, {0.5, 1}};
    for (Rule rule : {Rule::Max, Rule::Weighted, Rule::Band, Rule::Dual}) {
        const auto p = probabilities(o, rule);
        assert(near(p[0], 0.0));
        for (int i = 0; i < 1000; ++i) {
            const auto choice = pickFrom(p, static_cast<double>(i) / 1000.0);
            assert(choice && *choice != 0);
        }
    }
}

void nothingValidMeansNoPick() {
    const std::vector<Option> o{{0.0, 1}, {0.0, 2}};
    for (Rule rule : {Rule::Max, Rule::Weighted, Rule::Band, Rule::Dual}) {
        const auto p = probabilities(o, rule);
        assert(near(p[0], 0.0) && near(p[1], 0.0));
        assert(!pickFrom(p, 0.3));
    }
    assert(probabilities({}, Rule::Dual).empty());
}

void negativeWeightCountsAsZero() {
    const auto p = probabilities({{-1.0, 1}, {0.5, 1}}, Rule::Weighted);
    assert(near(p[0], 0.0) && near(p[1], 1.0));
}

void pickFromWalksTheCdf() {
    const std::vector<double> p{0.2, 0.3, 0.5};
    assert(*pickFrom(p, 0.0) == 0);
    assert(*pickFrom(p, 0.19) == 0);
    assert(*pickFrom(p, 0.2) == 1);  // the cumulative border belongs to the next option
    assert(*pickFrom(p, 0.49) == 1);
    assert(*pickFrom(p, 0.5) == 2);
    assert(*pickFrom(p, 0.999) == 2);
}

void pickFromSkipsZeroAtTheEdges() {
    assert(*pickFrom({0.0, 0.5, 0.5}, 0.0) == 1);                // u = 0 must not land on a zero entry
    assert(*pickFrom({0.5, 0.5, 0.0}, 0.9999999999999) == 1);    // rounding at the top must not either
    assert(*pickFrom({0.3, 0.3, 0.3, 0.0}, 0.99999999) == 2);    // sums to 0.9: the tail falls back to the last valid option
}

void dumbnessThreshold() {
    assert(isDumb(0.4, 1.0));
    assert(!isDumb(0.5, 1.0));   // exactly half is not dumb
    assert(!isDumb(0.9, 1.0));
    assert(!isDumb(0.0, 0.0));   // nothing is dumb when nothing has a score
}

void expectedDumbRates() {
    const auto o = options({0.9, 0.5, 0.6, 0.3});
    // Under 0.45: Snack 0.36, Chat 0.3, TV 0.25.
    assert(near(expectedDumbRate(o, probabilities(o, Rule::Weighted)), 0.91 / 2.91));
    assert(near(expectedDumbRate(o, probabilities(o, Rule::Max)), 0.0));
    assert(near(expectedDumbRate(o, probabilities(o, Rule::Band)), 0.0));
    assert(near(expectedDumbRate(o, probabilities(o, Rule::Dual)), 0.0));
    // No critical need: the rank 1 group is 0.7, 0.28, 0.4, 0.6, 0.2 (sum 2.18), under 0.35 are Snack and Chat.
    const auto n = options({0.7, 0.4, 0.6, 0.2});
    assert(near(expectedDumbRate(n, probabilities(n, Rule::Dual)), 0.48 / 2.18));
}

void rngRange() {
    Rng rng{12345};
    for (int i = 0; i < 100000; ++i) {
        const double u = rng.next();
        assert(u >= 0.0 && u < 1.0);
    }
    Rng a{7};
    Rng b{7};
    for (int i = 0; i < 100; ++i) assert(a.next() == b.next());  // deterministic
    Rng zero{0};
    assert(zero.next() != zero.next());                           // a zero seed must not get stuck
}

void sampledDistributionMatchesTheMath() {
    // 20000 picks against the exact probabilities. A 5 sigma bound at p = 0.5 is about 0.018, so 0.02 is safe for any seed.
    const auto o = options({0.9, 0.5, 0.6, 0.3});
    for (Rule rule : {Rule::Weighted, Rule::Band, Rule::Dual, Rule::Max}) {
        const auto p = probabilities(o, rule);
        Rng rng{99};
        Tally tally(o.size());
        constexpr int kN = 20000;
        for (int i = 0; i < kN; ++i) tally.add(*pickFrom(p, rng.next()), o);
        assert(tally.picks == kN);
        for (std::size_t i = 0; i < p.size(); ++i) {
            assert(std::fabs(static_cast<double>(tally.counts[i]) / kN - p[i]) < 0.02);
        }
    }
}

void tallyCountsDumbPicks() {
    const auto o = options({0.9, 0.5, 0.6, 0.3});
    Tally tally(o.size());
    tally.add(0, o);  // Eat 0.9: fine
    tally.add(5, o);  // TV 0.25 is under 0.45: dumb
    tally.add(5, o);
    tally.add(3, o);  // Play 0.6: fine
    assert(tally.picks == 4 && tally.dumb == 2);
    assert(near(tally.dumbRate(), 0.5));
    tally.clear();
    assert(tally.picks == 0 && tally.dumb == 0 && tally.counts[5] == 0);
    assert(near(tally.dumbRate(), 0.0));  // no division by zero on an empty tally
}

void sliderMapping() {
    assert(near(valueFromX(100.0F, 100.0F, 200.0F), 0.0));
    assert(near(valueFromX(300.0F, 100.0F, 200.0F), 1.0));
    assert(near(valueFromX(200.0F, 100.0F, 200.0F), 0.5));
    assert(near(valueFromX(50.0F, 100.0F, 200.0F), 0.0));    // left of the track clamps
    assert(near(valueFromX(900.0F, 100.0F, 200.0F), 1.0));   // right of the track clamps
    assert(near(valueFromX(150.0F, 100.0F, 0.0F), 0.0));     // a zero width must not divide
}

}  // namespace

int main() {
    optionsFromNeeds();
    criticalThreshold();
    needsAreClamped();
    maxTakesTheBest();
    maxSplitsTies();
    weightedIsProportional();
    weightedLetsTheSatedEat();
    bandKeepsOnlyThePlausible();
    bandBoundaryIsInside();
    bandWidthZeroIsMax();
    dualRankBeatsWeight();
    dualStarvingNeverWatchesTv();
    dualWeightsWithinTheRank();
    dualZeroWeightFallsThrough();
    dualFillerOnlyWhenNothingElse();
    zeroWeightIsNeverPicked();
    nothingValidMeansNoPick();
    negativeWeightCountsAsZero();
    pickFromWalksTheCdf();
    pickFromSkipsZeroAtTheEdges();
    dumbnessThreshold();
    expectedDumbRates();
    rngRange();
    sampledDistributionMatchesTheMath();
    tallyCountsDumbPicks();
    sliderMapping();
    std::puts("pick-an-action: all logic tests passed");
    return 0;
}
