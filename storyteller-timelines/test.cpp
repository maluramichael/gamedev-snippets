// Tests for the storyteller logic. Plain asserts on hand-computed cases, no framework, no SDL.
// The numbers (cycle lengths, Randy's weights) are the ones from the RimWorld wiki, not checked against the game files,
// so the tests check that the model does what the wiki describes, not that the wiki is right.
#undef NDEBUG
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "logic.hpp"

using namespace stl;

namespace {

bool near(double a, double b, double eps = 1e-9) { return std::fabs(a - b) < eps; }

void cassandraWindowsMatchTheWikiTable() {
    // Wiki table: on at 11..15.6, 21.6..26.2, 32.2..36.8, 42.8..47.4, 53.4..58, every further one +10.6.
    const auto w = windows(kCassandra, 200.0);
    assert(w.size() == 18);  // starts 11 + 10.6 k stay below day 200 for k = 0..17
    assert(near(w[0].from, 11.0) && near(w[0].to, 15.6));
    assert(near(w[1].from, 21.6) && near(w[1].to, 26.2));
    assert(near(w[2].from, 32.2) && near(w[2].to, 36.8));
    assert(near(w[3].from, 42.8) && near(w[3].to, 47.4));
    assert(near(w[4].from, 53.4) && near(w[4].to, 58.0));
    assert(near(w[17].from, 11.0 + 17 * 10.6, 1e-9));
}

void phoebeWindowsMatchTheWikiTable() {
    const auto w = windows(kPhoebe, 200.0);
    assert(w.size() == 12);  // 13 + 16 k < 200 for k = 0..11
    assert(near(w[0].from, 13.0) && near(w[0].to, 21.0));
    assert(near(w[1].from, 29.0) && near(w[1].to, 37.0));
    assert(near(w[2].from, 45.0) && near(w[2].to, 53.0));
    assert(near(w[11].from, 189.0) && near(w[11].to, 197.0));
}

void windowsAreClippedAtTheHorizon() {
    // Cassandra's third window is 32.2..36.8: a horizon at 35 cuts it, a horizon at 32.2 leaves it out.
    const auto cut = windows(kCassandra, 35.0);
    assert(cut.size() == 3);
    assert(near(cut[2].from, 32.2) && near(cut[2].to, 35.0));
    assert(windows(kCassandra, 32.2).size() == 2);
    assert(windows(kCassandra, 11.0).empty());  // starts exactly at the horizon: nothing
    assert(windows(kCassandra, 0.0).empty());
    assert(windows(kCassandra, -5.0).empty());
}

void onPhaseEdges() {
    assert(!onPhase(kCassandra, 10.99));
    assert(onPhase(kCassandra, 11.0));
    assert(onPhase(kCassandra, 15.59));
    assert(!onPhase(kCassandra, 15.6));  // the window is half open
    assert(!onPhase(kCassandra, 20.0));
    assert(onPhase(kCassandra, 21.6 + 1e-9));
    assert(!onPhase(kCassandra, 0.0));
    assert(!onPhase(kCassandra, -3.0));
    assert(onPhase(kPhoebe, 13.0));
    assert(!onPhase(kPhoebe, 21.0));
    assert(!onPhase(kPhoebe, 28.99));
    assert(onPhase(kPhoebe, 29.0));
    assert(near(period(kCassandra), 10.6) && near(period(kPhoebe), 16.0));
}

void placementByHand() {
    // One threat: the unit scales over the whole window. 0.25 of 8 days is day 2.
    auto one = windowOffsets(1, 8.0, 0.0, {0.25});
    assert(one.size() == 1 && near(one[0], 2.0));
    // Two threats in 4.6 days with 1.9 spacing: width 4.6 - 1.9 = 2.7, the second is pushed by 1.9.
    // Units come in any order and are sorted: {0.5, 0.0} -> offsets {0, 0.5 * 2.7 + 1.9}.
    auto two = windowOffsets(2, 4.6, 1.9, {0.5, 0.0});
    assert(two.size() == 2 && near(two[0], 0.0) && near(two[1], 1.35 + 1.9));
    // Both at the extremes: the earliest possible pair and the latest possible pair.
    auto early = windowOffsets(2, 4.6, 1.9, {0.0, 0.0});
    assert(near(early[0], 0.0) && near(early[1], 1.9));
    auto late = windowOffsets(2, 4.6, 1.9, {1.0, 1.0});
    assert(near(late[0], 2.7) && near(late[1], 4.6));
}

void fittingCount() {
    assert(maxThreatsFitting(4.6, 1.9, 1) == 1);
    assert(maxThreatsFitting(4.6, 1.9, 2) == 2);
    assert(maxThreatsFitting(4.6, 1.9, 3) == 3);  // 0, 1.9, 3.8
    assert(maxThreatsFitting(4.6, 1.9, 4) == 3);  // 5.7 would not fit
    assert(maxThreatsFitting(1.0, 1.9, 2) == 1);  // window shorter than the spacing
    assert(maxThreatsFitting(8.0, 0.0, 5) == 5);  // no spacing: everything fits
    assert(maxThreatsFitting(0.0, 1.9, 2) == 1);
    assert(maxThreatsFitting(4.6, 1.9, 0) == 0);
}

void cassandraEveryThreatInsideAWindow() {
    int ones = 0;
    int twos = 0;
    for (std::uint64_t seed = 1; seed <= 300; ++seed) {
        const auto ev = simulate(Teller::Cassandra, 200.0, seed);
        const auto days = threatDays(ev);
        const auto w = windows(kCassandra, 200.0);
        for (const double d : days) assert(d >= 11.0 && d < 200.0);
        for (const Window& win : w) {
            int n = 0;
            double prev = -1e9;
            for (const double d : days) {
                if (d >= win.from && d < win.to) {
                    ++n;
                    assert(d - prev >= 1.9 - 1e-9);  // minimum spacing inside a window
                    prev = d;
                }
            }
            assert(n >= 1 && n <= 2);
            (n == 1 ? ones : twos)++;
        }
        // Nothing outside the windows: the counts add up to the total.
        int inWindows = 0;
        for (const double d : days) {
            for (const Window& win : w) inWindows += (d >= win.from && d < win.to) ? 1 : 0;
        }
        assert(inWindows == static_cast<int>(days.size()));
    }
    // 1 or 2 threats with the same chance: about half each (5400 windows, a few percent of slack).
    const double share = static_cast<double>(twos) / static_cast<double>(ones + twos);
    assert(share > 0.46 && share < 0.54);
}

void cassandraGapsStayInTheirBand() {
    // Within a window the gap is at least 1.9 and at most 4.6. Between windows: the end of one window to the start of the
    // next is 6 days of silence, so the gap is at least 6, and at most 4.6 + 6 + 4.6 = 15.2.
    double lo = 1e9;
    double hi = 0.0;
    for (std::uint64_t seed = 1; seed <= 300; ++seed) {
        for (const double g : gaps(threatDays(simulate(Teller::Cassandra, 200.0, seed)))) {
            lo = std::min(lo, g);
            hi = std::max(hi, g);
            assert(g >= 1.9 - 1e-9);
            assert(g <= 15.2 + 1e-9);
            assert(g <= 4.6 || g >= 6.0 - 1e-9);  // no gap between 4.6 and 6: that would cross a quiet phase shorter than 6 days
        }
    }
    assert(lo < 2.1);   // the spacing is reached
    assert(hi > 14.5);  // so is the far end
}

void phoebeOneThreatPerWindow() {
    for (std::uint64_t seed = 1; seed <= 300; ++seed) {
        const auto days = threatDays(simulate(Teller::Phoebe, 200.0, seed));
        const auto w = windows(kPhoebe, 200.0);
        assert(days.size() == w.size());
        for (std::size_t i = 0; i < w.size(); ++i) assert(days[i] >= w[i].from && days[i] < w[i].to);
        for (const double g : gaps(days)) assert(g > 8.0 - 1e-9 && g < 24.0 + 1e-9);  // wiki: anywhere from 8 to 24 days
    }
}

void yearlyRates() {
    // Long runs, so the start and the cut-off at the horizon do not matter. Cassandra: 1.5 threats per 10.6 days = 8.49 a year.
    // Phoebe: 1 per 16 days = 3.75 a year. The wiki says about 8.5 for Cassandra.
    const double days = 300000.0;
    const double cass = perYear(static_cast<int>(threatDays(simulate(Teller::Cassandra, days, 7)).size()), days);
    const double phoebe = perYear(static_cast<int>(threatDays(simulate(Teller::Phoebe, days, 7)).size()), days);
    assert(std::fabs(cass - 60.0 * 1.5 / 10.6) < 0.1);
    assert(std::fabs(phoebe - 3.75) < 0.01);
}

void categoryBoundaries() {
    // Weights 3.5 1.4 1.1 2.4 0.6 0.22, total 9.22. Cumulative edges: 3.5 4.9 6.0 8.4 9.0 9.22.
    assert(near(kRandyWeightTotal, 9.22));
    assert(pickCategory(0.0) == Category::Misc);
    assert(pickCategory(3.4 / 9.22) == Category::Misc);
    assert(pickCategory(3.6 / 9.22) == Category::ThreatBig);
    assert(pickCategory(4.8 / 9.22) == Category::ThreatBig);
    assert(pickCategory(5.0 / 9.22) == Category::OrbitalVisitor);
    assert(pickCategory(6.1 / 9.22) == Category::FactionArrival);
    assert(pickCategory(8.5 / 9.22) == Category::ThreatSmall);
    assert(pickCategory(9.1 / 9.22) == Category::ShipChunkDrop);
    assert(pickCategory(0.999999999) == Category::ShipChunkDrop);
    assert(pickCategory(1.0) == Category::ShipChunkDrop);  // clamped, never out of range
    assert(pickCategory(-0.5) == Category::Misc);
}

void randyAnalyticMean() {
    // Without the 13 day rule a major threat comes every 1.35 * 9.22 / 1.4 = 8.89 days. The rule shortens the mean to 7.14.
    assert(near(randyThreatShare(), 1.4 / 9.22));
    assert(std::fabs(randyExpectedGap() - 7.14) < 0.005);
    assert(std::fabs(60.0 / randyExpectedGap() - 8.40) < 0.01);  // about as many a year as Cassandra's 8.49
}

void randyMeasuredMean() {
    const auto ev = simulate(Teller::Randy, 400000.0, 11);
    const auto days = threatDays(ev);
    const auto g = gaps(days);
    double sum = 0.0;
    for (const double x : g) sum += x;
    const double mean = sum / static_cast<double>(g.size());
    assert(std::fabs(mean - randyExpectedGap()) < 0.1);
    assert(std::fabs(perYear(static_cast<int>(days.size()), 400000.0) - 60.0 / randyExpectedGap()) < 0.15);
    // One event every 1.35 days, 1.4 / 9.22 of them major threats (about 15 percent): with forced ones a bit more.
    const double eventsPerDay = static_cast<double>(ev.size()) / 400000.0;
    assert(std::fabs(eventsPerDay - 1.0 / 1.35) < 0.02);
}

void randyForcedRule() {
    int forced = 0;
    for (std::uint64_t seed = 1; seed <= 200; ++seed) {
        const auto ev = simulate(Teller::Randy, 200.0, seed);
        double prev = 0.0;  // the clock of the rule starts at day 0
        for (const Event& e : ev) {
            assert(e.day >= 1.0 - 1e-9 && e.day < 200.0);  // nothing before minDaysPassed = 1
            if (e.category != Category::ThreatBig) {
                assert(!e.forced);
                continue;
            }
            // Forced exactly when 13 days have passed since the previous major threat, never otherwise.
            const bool overdue = std::llround((e.day - prev) * 60.0) >= 780;
            assert(e.forced == overdue);
            forced += e.forced ? 1 : 0;
            prev = e.day;
        }
        // The rule also bounds the silence from the other side: the last event before a long gap cannot be skipped.
        for (const Event& e : ev) assert(e.forced ? e.category == Category::ThreatBig : true);
    }
    assert(forced > 0);  // it does happen, about a fifth of all threats
}

void randyClumpsTheOthersDoNot() {
    // The point of the demo: same average, different shape. Randy has gaps shorter than 1.9 days (Cassandra's minimum
    // spacing, Phoebe never) and gaps longer than 15.2 days (Cassandra's maximum).
    int randy = 0;
    int randyShort = 0;
    int randyLong = 0;
    for (std::uint64_t seed = 1; seed <= 300; ++seed) {
        for (const double g : gaps(threatDays(simulate(Teller::Randy, 200.0, seed)))) {
            ++randy;
            randyShort += g < 1.9 ? 1 : 0;
            randyLong += g > 15.2 ? 1 : 0;
        }
    }
    const double shortShare = static_cast<double>(randyShort) / static_cast<double>(randy);
    const double longShare = static_cast<double>(randyLong) / static_cast<double>(randy);
    assert(shortShare > 0.15 && shortShare < 0.25);  // exponential: 1 - exp(-1.9 / 8.89) = 0.19
    assert(longShare > 0.02 && longShare < 0.08);    // 0.23 * exp(-2.2 / 1.35) = 0.045
}

void sameSeedSameStory() {
    for (const Teller t : {Teller::Cassandra, Teller::Phoebe, Teller::Randy}) {
        const auto a = simulate(t, 200.0, 42);
        const auto b = simulate(t, 200.0, 42);
        const auto c = simulate(t, 200.0, 43);
        assert(a.size() == b.size());
        for (std::size_t i = 0; i < a.size(); ++i) assert(a[i].day == b[i].day && a[i].forced == b[i].forced);
        bool differs = a.size() != c.size();
        for (std::size_t i = 0; !differs && i < a.size(); ++i) differs = a[i].day != c[i].day;
        assert(differs);
    }
}

void rngRange() {
    Rng r{5};
    double lo = 1.0;
    double hi = 0.0;
    for (int i = 0; i < 100000; ++i) {
        const double u = r.unit();
        assert(u >= 0.0 && u < 1.0);
        lo = std::min(lo, u);
        hi = std::max(hi, u);
    }
    assert(lo < 0.001 && hi > 0.999);
}

void gapsAndHistogram() {
    assert(gaps({}).empty());
    assert(gaps({5.0}).empty());
    const auto g = gaps({1.0, 3.5, 4.0, 10.0});
    assert(g.size() == 3 && near(g[0], 2.5) && near(g[1], 0.5) && near(g[2], 6.0));
    // 1 day bins, 5 bins: the last one collects everything from 4 on.
    const auto h = histogram({0.5, 1.0, 1.99, 2.0, 4.0, 29.9, -1.0}, 5, 1.0);
    assert(h.size() == 5);
    assert(h[0] == 2);  // 0.5 and the negative one (clamped)
    assert(h[1] == 2);  // 1.0 and 1.99
    assert(h[2] == 1);  // 2.0
    assert(h[3] == 0);
    assert(h[4] == 2);  // 4.0 and 29.9
    const auto wide = histogram({0.5, 1.9, 2.0}, 3, 2.0);
    assert(wide[0] == 2 && wide[1] == 1 && wide[2] == 0);
    // Degenerate input gives zeros instead of a division by zero.
    const auto zero = histogram({1.0, 2.0}, 4, 0.0);
    assert(zero.size() == 4 && zero[0] == 0 && zero[3] == 0);
    assert(histogram({1.0}, 0, 1.0).empty());
}

void yearCounting() {
    assert(near(perYear(19, 200.0), 5.7));
    assert(near(perYear(0, 200.0), 0.0));
    assert(near(perYear(10, 60.0), 10.0));
    assert(near(perYear(10, 0.0), 0.0));  // no division by zero
    assert(near(perYear(10, -4.0), 0.0));
    const std::vector<double> days{0.5, 59.99, 60.0, 119.9, 120.0};
    assert(countIn(days, 0.0, 60.0) == 2);
    assert(countIn(days, 60.0, 120.0) == 2);
    assert(countIn(days, 120.0, 180.0) == 1);
    assert(countIn(days, 180.0, 200.0) == 0);
    assert(countIn(days, 5.0, 5.0) == 0);
}

void poolMatchesTheSingleRuns() {
    // The pool of a storyteller is built from runs with consecutive seeds, so it must equal doing the same by hand.
    const Pool p = pool(Teller::Phoebe, 3, 200.0, 100, 30, 1.0);
    std::vector<double> all;
    for (std::uint64_t s = 100; s < 103; ++s) {
        const auto g = gaps(threatDays(simulate(Teller::Phoebe, 200.0, s)));
        all.insert(all.end(), g.begin(), g.end());
    }
    assert(p.gapCount == static_cast<int>(all.size()));
    assert(all.size() == 3 * 11);  // 12 threats a run, 11 gaps
    assert(p.bins == histogram(all, 30, 1.0));
    double sum = 0.0;
    for (const double x : all) sum += x;
    assert(near(p.meanGap, sum / static_cast<double>(all.size())));
    assert(near(p.perYear, perYear(3 * 12, 3 * 200.0)));
    assert(near(p.perYear, 3.6));  // 12 in 200 days
    // Phoebe never produces a gap under 8 days: the first eight bins are empty.
    for (int i = 0; i < 8; ++i) assert(p.bins[static_cast<std::size_t>(i)] == 0);
    // No runs, no division by zero.
    const Pool none = pool(Teller::Randy, 0, 200.0, 1, 30, 1.0);
    assert(none.gapCount == 0 && near(none.meanGap, 0.0) && near(none.perYear, 0.0));
}

}  // namespace

int main() {
    cassandraWindowsMatchTheWikiTable();
    phoebeWindowsMatchTheWikiTable();
    windowsAreClippedAtTheHorizon();
    onPhaseEdges();
    placementByHand();
    fittingCount();
    cassandraEveryThreatInsideAWindow();
    cassandraGapsStayInTheirBand();
    phoebeOneThreatPerWindow();
    yearlyRates();
    categoryBoundaries();
    randyAnalyticMean();
    randyMeasuredMean();
    randyForcedRule();
    randyClumpsTheOthersDoNot();
    sameSeedSameStory();
    rngRange();
    gapsAndHistogram();
    yearCounting();
    poolMatchesTheSingleRuns();
    std::puts("storyteller-timelines logic: all asserts passed");
    return 0;
}
