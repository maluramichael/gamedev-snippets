// Tests for the mood and breakdown logic. Plain asserts on hand-computed cases, no framework, no SDL.
// The statistical tests use a fixed seed and a wide tolerance, so they are deterministic and not flaky.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>

#include "logic.hpp"

using namespace mb;

namespace {

bool near(double a, double b, double eps = 1e-9) { return std::fabs(a - b) < eps; }

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

void moodRate() {
    // The bar rises by at most 12 per hour and falls by at most 8 per hour.
    assert(near(stepMood(20, 80, 1.0), 32.0));
    assert(near(stepMood(50, 0, 1.0), 42.0));
    assert(near(stepMood(20, 80, 0.5), 26.0));
    assert(near(stepMood(50, 0, 0.5), 46.0));
    // Close to the target it lands on it, no overshoot in either direction.
    assert(near(stepMood(20, 25, 1.0), 25.0));
    assert(near(stepMood(50, 47, 1.0), 47.0));
    assert(near(stepMood(40, 40, 3.0), 40.0));
    // No time, no movement. Negative time is ignored too.
    assert(near(stepMood(40, 90, 0.0), 40.0));
    assert(near(stepMood(40, 90, -1.0), 40.0));
    assert(near(stepMood(40, 0, kNan), 40.0));
    // Over a long time it reaches the target and stays there.
    assert(near(stepMood(0, 100, 100.0), 100.0));
    assert(near(stepMood(100, 0, 100.0), 0.0));
    // Ten small steps equal one big one.
    double m = 20.0;
    for (int i = 0; i < 10; ++i) m = stepMood(m, 80.0, 0.1);
    assert(near(m, 32.0));
    m = 50.0;
    for (int i = 0; i < 10; ++i) m = stepMood(m, 0.0, 0.1);
    assert(near(m, 42.0));
    // Up is faster than down.
    assert(stepMood(50, 100, 1.0) - 50.0 > 50.0 - stepMood(50, 0, 1.0));
    // A target outside 0..100 is clamped, a broken target never leaks into the bar.
    assert(near(stepMood(10, -50, 10.0), 0.0));
    assert(near(stepMood(90, 500, 10.0), 100.0));
    assert(std::isfinite(stepMood(40, kNan, 1.0)));
}

void thresholdsFromBase() {
    const Thresholds t = thresholds(35.0);
    assert(near(t.minor, 35.0) && near(t.major, 20.0) && near(t.extreme, 5.0));
    const Thresholds s = thresholds(28.0);  // 4/7 and 1/7 of the minor threshold
    assert(near(s.minor, 28.0) && near(s.major, 16.0) && near(s.extreme, 4.0));
    // The minor threshold is capped between 1 and 50.
    assert(near(thresholds(0.0).minor, 1.0));
    assert(near(thresholds(-30.0).minor, 1.0));
    assert(near(thresholds(80.0).minor, 50.0));
    assert(near(thresholds(80.0).major, 50.0 * 4.0 / 7.0));
    assert(near(thresholds(1.0).extreme, 1.0 / 7.0));
    assert(near(thresholds(kNan).minor, 35.0));  // a broken value falls back to the default
}

void severityBands() {
    const Thresholds t = thresholds(35.0);
    assert(severityOf(80.0, t) == Severity::None);
    assert(severityOf(35.0, t) == Severity::None);  // exactly on the line is not below it
    assert(severityOf(34.99, t) == Severity::Minor);
    assert(severityOf(20.0, t) == Severity::Minor);
    assert(severityOf(19.99, t) == Severity::Major);
    assert(severityOf(5.0, t) == Severity::Major);
    assert(severityOf(4.99, t) == Severity::Extreme);
    assert(severityOf(0.0, t) == Severity::Extreme);
    assert(severityOf(kNan, t) == Severity::None);
    // The same mood is calmer for a colonist with a lower minor threshold.
    assert(severityOf(30.0, thresholds(25.0)) == Severity::None);
    assert(severityOf(30.0, thresholds(45.0)) == Severity::Minor);
}

void breakChances() {
    assert(near(mtbHours(Severity::Minor), 96.0));
    assert(near(mtbHours(Severity::Major), 19.2));
    assert(near(mtbHours(Severity::Extreme), 12.0));
    assert(std::isinf(mtbHours(Severity::None)));
    // After exactly one mean time the chance is 1 - 1/e.
    assert(near(breakChance(Severity::Minor, 96.0), 1.0 - std::exp(-1.0)));
    assert(near(breakChance(Severity::Extreme, 12.0), 1.0 - std::exp(-1.0)));
    assert(near(breakChance(Severity::Minor, 1.0), 1.0 - std::exp(-1.0 / 96.0)));
    assert(near(breakChance(Severity::Major, 0.05), 1.0 - std::exp(-0.05 / 19.2)));
    // No risk without a severity, no time or negative time. Never above 1.
    assert(breakChance(Severity::None, 5.0) == 0.0);
    assert(breakChance(Severity::Extreme, 0.0) == 0.0);
    assert(breakChance(Severity::Extreme, -2.0) == 0.0);
    assert(breakChance(Severity::Extreme, kNan) == 0.0);
    assert(breakChance(Severity::Extreme, 1.0e6) <= 1.0);
    // Worse severity, higher chance for the same time.
    assert(breakChance(Severity::Minor, 1.0) < breakChance(Severity::Major, 1.0));
    assert(breakChance(Severity::Major, 1.0) < breakChance(Severity::Extreme, 1.0));
}

double fractionBroken(Severity sev, double totalHours, double dt, int trials, std::uint64_t seed) {
    Rng rng{seed};
    int hit = 0;
    const int steps = static_cast<int>(std::lround(totalHours / dt));
    for (int t = 0; t < trials; ++t) {
        for (int i = 0; i < steps; ++i) {
            if (rng.unit() < breakChance(sev, dt)) {
                ++hit;
                break;
            }
        }
    }
    return static_cast<double>(hit) / trials;
}

void breakRateIsMeasuredNotAssumed() {
    // After one mean time, 1 - 1/e = 63.2 percent of the trials must have broken, whatever the severity.
    const double expected = 1.0 - std::exp(-1.0);
    for (const Severity s : {Severity::Minor, Severity::Major, Severity::Extreme}) {
        const double f = fractionBroken(s, mtbHours(s), 0.5, 3000, 12345);
        std::printf("measured: %s, share broken after one mean time = %.3f (expected %.3f)\n", s == Severity::Minor ? "minor" : s == Severity::Major ? "major" : "extreme", f, expected);
        assert(std::fabs(f - expected) < 0.04);
    }
    // Half a mean time: 1 - exp(-0.5) = 39.3 percent. Shows the step size does not matter.
    const double half = fractionBroken(Severity::Major, 0.5 * mtbHours(Severity::Major), 0.05, 3000, 777);
    assert(std::fabs(half - (1.0 - std::exp(-0.5))) < 0.04);
}

void longTermStress() {
    assert(near(shortStress(0.0), 100.0));
    assert(near(shortStress(100.0), 0.0));
    assert(near(shortStress(35.0), 65.0));
    assert(near(shortStress(kNan), 100.0));
    // At mood 0 the counter climbs by kLongRise per hour, at mood 50 it stands still.
    assert(near(stepLong(0.0, 0.0, 10.0), 10.0 * kLongRise));
    assert(near(stepLong(30.0, 50.0, 10.0), 30.0));
    // Recovery is faster than the build-up, by the ratio of the two Dwarf Fortress maxima.
    assert(near(kLongFall / kLongRise, 43564.0 / 20160.0));
    assert(near(stepLong(50.0, 100.0, 1.0), 50.0 - kLongFall));
    // Mood 25 means short-term stress 75, half way between neutral and the top: half the maximal rate.
    assert(near(stepLong(0.0, 25.0, 1.0), 0.5 * kLongRise));
    // Clamped at both ends.
    assert(near(stepLong(0.0, 100.0, 5.0), 0.0));
    assert(near(stepLong(99.5, 0.0, 10.0), 100.0));
    // No time, no change.
    assert(near(stepLong(12.0, 0.0, 0.0), 12.0));
    assert(near(stepLong(12.0, 0.0, -3.0), 12.0));
}

void twoCounterCondition() {
    const Thresholds t = thresholds(35.0);
    // RimWorld: the mood alone decides.
    assert(riskOf(Model::Rim, 30.0, 0.0, t) == Severity::Minor);
    assert(riskOf(Model::Rim, 40.0, 100.0, t) == Severity::None);
    // Dwarf Fortress: both counters must be high. Low mood with a fresh long-term counter is safe.
    assert(riskOf(Model::Dwarf, 30.0, 0.0, t) == Severity::None);
    assert(riskOf(Model::Dwarf, 30.0, kLongGate - 0.01, t) == Severity::None);
    assert(riskOf(Model::Dwarf, 30.0, kLongGate, t) == Severity::Minor);
    assert(riskOf(Model::Dwarf, 30.0, 90.0, t) == Severity::Minor);
    // A high long-term counter does nothing while the mood is fine.
    assert(riskOf(Model::Dwarf, 40.0, 90.0, t) == Severity::None);
    // The severity still comes from the mood.
    assert(riskOf(Model::Dwarf, 3.0, 50.0, t) == Severity::Extreme);
}

Sim calm(std::uint64_t seed = 1) {
    Sim s = makeSim(seed);
    s.base = 90.0;
    for (Colonist& c : s.c) c.mood = 80.0;
    return s;
}

void thoughtsMoveTheTarget() {
    Sim s = makeSim(1);
    s.base = 32.0;
    Colonist c;
    c.trait = 0.0;
    assert(near(targetOf(s, c), 32.0));
    addThought(c, Thought::Corpse);
    assert(near(targetOf(s, c), 22.0));  // 32 - 10
    addThought(c, Thought::Funeral);
    assert(near(targetOf(s, c), 34.0));  // 32 - 10 + 12
    c.trait = -8.0;
    assert(near(targetOf(s, c), 26.0));
    // Adding the same thought again refreshes it, it does not stack.
    Colonist d;
    addThought(d, Thought::Corpse);
    d.left[static_cast<int>(Thought::Corpse)] = 3.0;
    addThought(d, Thought::Corpse);
    assert(near(d.left[static_cast<int>(Thought::Corpse)], 12.0));
    assert(near(targetOf(s, d), 22.0));
    // The target is clamped to the bar.
    s.base = 5.0;
    Colonist e;
    addThought(e, Thought::Corpse);
    assert(near(targetOf(s, e), 0.0));
    s.base = 90.0;
    Colonist f;
    addThought(f, Thought::Catharsis);
    assert(near(targetOf(s, f), 100.0));
}

void thoughtsExpire() {
    Sim s = calm();
    applyEvent(s, Event::Corpse, 0);
    const int corpse = static_cast<int>(Thought::Corpse);
    step(s, 11.9);
    assert(s.c[0].left[corpse] > 0.05);
    step(s, 0.1);
    assert(s.c[0].left[corpse] < 1e-9);
    assert(near(targetOf(s, s.c[0]), 90.0 + s.c[0].trait));
}

void eventsHaveAScope() {
    Sim s = calm();
    applyEvent(s, Event::BadMeal, 2);
    const int meal = static_cast<int>(Thought::BadMeal);
    for (int i = 0; i < kColonists; ++i) assert((s.c[i].left[meal] > 0.0) == (i == 2));
    Sim t = calm();
    applyEvent(t, Event::Corpse, 5);
    const int corpse = static_cast<int>(Thought::Corpse);
    for (int i = 0; i < kColonists; ++i) assert(t.c[i].left[corpse] > 0.0);
    Sim u = calm();
    applyEvent(u, Event::Funeral, 0);
    const int funeral = static_cast<int>(Thought::Funeral);
    for (int i = 0; i < kColonists; ++i) assert(u.c[i].left[funeral] > 0.0);
    // A selection outside the row does nothing instead of writing out of bounds.
    Sim v = calm();
    applyEvent(v, Event::BadMeal, -1);
    applyEvent(v, Event::BadMeal, kColonists);
    for (int i = 0; i < kColonists; ++i) assert(v.c[i].left[meal] == 0.0);
}

void aBreakDragsTheNeighboursDown() {
    Sim s = calm();
    const int witnessed = static_cast<int>(Thought::Witnessed);
    startBreak(s, 3, Severity::Minor);
    assert(s.c[3].broken && s.c[3].severity == Severity::Minor);
    assert(s.breaks == 1 && s.chained == 0);
    for (int i = 0; i < kColonists; ++i) assert((s.c[i].left[witnessed] > 0.0) == (i == 2 || i == 4));
    // The witness lowers the target by 14, from 90 + trait.
    assert(near(targetOf(s, s.c[2]), 90.0 + s.c[2].trait - 14.0));
    assert(near(targetOf(s, s.c[1]), 90.0 + s.c[1].trait));
    // Breaking a second time while broken changes nothing.
    startBreak(s, 3, Severity::Extreme);
    assert(s.breaks == 1 && s.c[3].severity == Severity::Minor);
    // The ends of the row have only one neighbour.
    Sim a = calm();
    startBreak(a, 0, Severity::Major);
    for (int i = 0; i < kColonists; ++i) assert((a.c[i].left[witnessed] > 0.0) == (i == 1));
    Sim b = calm();
    startBreak(b, kColonists - 1, Severity::Major);
    for (int i = 0; i < kColonists; ++i) assert((b.c[i].left[witnessed] > 0.0) == (i == kColonists - 2));
    // Out of range is ignored.
    Sim c = calm();
    startBreak(c, -1, Severity::Minor);
    startBreak(c, kColonists, Severity::Minor);
    assert(c.breaks == 0);
    // Without the chain the neighbours stay untouched.
    Sim d = calm();
    d.chain = false;
    startBreak(d, 3, Severity::Minor);
    for (int i = 0; i < kColonists; ++i) assert(d.c[i].left[witnessed] == 0.0);
    // A break that starts while a witness thought is active counts as chained.
    Sim e = calm();
    startBreak(e, 3, Severity::Minor);
    startBreak(e, 2, Severity::Minor);
    assert(e.breaks == 2 && e.chained == 1);
    assert(e.recent[0].who == 2 && e.recent[0].chained && e.recent[1].who == 3 && !e.recent[1].chained);
}

void aBreakEndsWithCatharsis() {
    Sim s = calm();
    startBreak(s, 3, Severity::Minor);
    step(s, kBreakHours - 0.1);
    assert(s.c[3].broken);
    step(s, 0.2);
    assert(!s.c[3].broken && s.c[3].severity == Severity::None);
    // Catharsis lasts three days and lifts the target by 40.
    assert(near(s.c[3].left[static_cast<int>(Thought::Catharsis)], 72.0));
    assert(s.breaks == 1);  // the calm colony does not break again
}

void noNewBreakWhileBroken() {
    // Mood 0 would be extreme, but one colonist can only break once at a time.
    Sim s = makeSim(5);
    s.base = 0.0;
    for (Colonist& c : s.c) c.mood = 0.0;
    startBreak(s, 4, Severity::Extreme);
    const int before = s.breaks;
    for (int i = 0; i < 100; ++i) {
        step(s, 0.05);
        assert(s.c[4].broken);
    }
    assert(s.breaks >= before);
}

void simulationKeepsItsInvariants() {
    Sim s = makeSim(42);
    s.base = 20.0;
    Rng pick{99};
    for (int i = 0; i < 60000; ++i) {
        if (i % 40 == 0) applyEvent(s, static_cast<Event>(static_cast<int>(pick.unit() * 3.0) % 3), static_cast<int>(pick.unit() * 8.0) % 8);
        if (i == 30000) s.model = Model::Dwarf;
        step(s, 0.05);
        for (const Colonist& c : s.c) {
            assert(std::isfinite(c.mood) && c.mood >= 0.0 && c.mood <= 100.0);
            assert(std::isfinite(c.longStress) && c.longStress >= 0.0 && c.longStress <= 100.0);
            for (const double left : c.left) assert(left >= 0.0 && left <= 72.0 + 1e-9);
            assert(c.broken == (c.brokenLeft > 0.0));
        }
    }
    assert(s.breaks > 0);
    assert(s.chained <= s.breaks);
    assert(near(s.clock, 3000.0, 1e-6));
}

void sameSeedSameStory() {
    Sim a = makeSim(7);
    Sim b = makeSim(7);
    Sim c = makeSim(8);
    a.base = b.base = c.base = 20.0;
    for (int i = 0; i < 10000; ++i) {
        step(a, 0.05);
        step(b, 0.05);
        step(c, 0.05);
    }
    bool differs = false;
    for (int i = 0; i < kColonists; ++i) {
        assert(a.c[i].mood == b.c[i].mood && a.c[i].broken == b.c[i].broken);
        differs = differs || a.c[i].mood != c.c[i].mood;
    }
    assert(a.breaks == b.breaks);
    assert(differs);
}

struct Tally {
    double breaks = 0.0;
    double chained = 0.0;
};

Tally meanBreaks(Model model, bool chain, double hours, int seeds) {
    Tally t;
    for (int seed = 1; seed <= seeds; ++seed) {
        Sim s = makeSim(static_cast<std::uint64_t>(seed));
        s.model = model;
        s.chain = chain;
        const int steps = static_cast<int>(std::lround(hours / 0.05));
        for (int i = 0; i < steps; ++i) step(s, 0.05);
        t.breaks += s.breaks;
        t.chained += s.chained;
    }
    t.breaks /= seeds;
    t.chained /= seeds;
    return t;
}

void theChainAndTheGateChangeTheOutcome() {
    constexpr int kSeeds = 300;
    constexpr double kHours = 120.0;
    const Tally withChain = meanBreaks(Model::Rim, true, kHours, kSeeds);
    const Tally noChain = meanBreaks(Model::Rim, false, kHours, kSeeds);
    const Tally dwarf = meanBreaks(Model::Dwarf, true, kHours, kSeeds);
    std::printf("measured: default colony, %d seeds, %.0f hours, mean breaks: rim with chain %.2f (%.2f chained), rim without chain %.2f, dwarf with chain %.2f\n", kSeeds, kHours, withChain.breaks, withChain.chained, noChain.breaks, dwarf.breaks);
    assert(noChain.chained == 0.0);
    assert(withChain.breaks > noChain.breaks);
    assert(withChain.chained > 0.0);
    assert(dwarf.breaks < withChain.breaks);
}

double meanBreaksAtBase(double base, double hours, int seeds, Model model) {
    double sum = 0.0;
    for (int seed = 1; seed <= seeds; ++seed) {
        Sim s = makeSim(static_cast<std::uint64_t>(seed));
        s.base = base;
        s.model = model;
        const int steps = static_cast<int>(std::lround(hours / 0.05));
        for (int i = 0; i < steps; ++i) step(s, 0.05);
        sum += s.breaks;
    }
    return sum / seeds;
}

void lowerBaseMeansMoreBreaks() {
    // Same colony, only the base mood differs. The count must never go up with a higher base, and it jumps when the
    // targets cross the lines.
    double previous = 1.0e9;
    for (const double base : {22.0, 32.0, 42.0, 52.0}) {
        const double mean = meanBreaksAtBase(base, 120.0, 200, Model::Rim);
        std::printf("measured: base %.0f, 120 hours, mean breaks %.2f\n", base, mean);
        assert(mean <= previous);
        previous = mean;
    }
    assert(meanBreaksAtBase(52.0, 120.0, 200, Model::Rim) < 0.2);
    assert(meanBreaksAtBase(22.0, 120.0, 200, Model::Rim) > 4.0);
}

void aBadDayDoesNotBreakTheDwarvesYet() {
    // One corpse for everybody. The long-term counter climbs by at most kLongRise per hour, so in the first 24 hours it
    // cannot reach the gate of 30 and the Dwarf Fortress model produces no break at all. The RimWorld model does.
    double rim = 0.0;
    double dwarf = 0.0;
    constexpr int kSeeds = 200;
    for (int seed = 1; seed <= kSeeds; ++seed) {
        for (const Model model : {Model::Rim, Model::Dwarf}) {
            Sim s = makeSim(static_cast<std::uint64_t>(seed));
            s.model = model;
            applyEvent(s, Event::Corpse, 0);
            for (int i = 0; i < 480; ++i) step(s, 0.05);
            (model == Model::Rim ? rim : dwarf) += s.breaks;
        }
    }
    std::printf("measured: corpse for everybody, first 24 hours, mean breaks: rim %.2f, dwarf %.2f\n", rim / kSeeds, dwarf / kSeeds);
    assert(dwarf == 0.0);
    assert(rim > 0.0);
}

}  // namespace

int main() {
    moodRate();
    thresholdsFromBase();
    severityBands();
    breakChances();
    breakRateIsMeasuredNotAssumed();
    longTermStress();
    twoCounterCondition();
    thoughtsMoveTheTarget();
    thoughtsExpire();
    eventsHaveAScope();
    aBreakDragsTheNeighboursDown();
    aBreakEndsWithCatharsis();
    noNewBreakWhileBroken();
    simulationKeepsItsInvariants();
    sameSeedSameStory();
    theChainAndTheGateChangeTheOutcome();
    lowerBaseMeansMoreBreaks();
    aBadDayDoesNotBreakTheDwarvesYet();
    std::puts("mood-breakdown: all tests passed");
    return 0;
}
