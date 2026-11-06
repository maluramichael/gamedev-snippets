// Tests for the idle economy logic. Plain asserts on hand-computed cases, no framework, no SDL.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

#include "logic.hpp"

using namespace idle;

namespace {

bool near(double a, double b, double eps = 1e-9) { return std::fabs(a - b) <= eps * std::max(1.0, std::fabs(b)); }

constexpr double kInf = std::numeric_limits<double>::infinity();

void costs() {
    const Rules exp;  // exponential, growth 1.15
    const Tier& stand = kTiers[0];
    assert(unitCost(exp, stand, 0) == 10.0);
    assert(near(unitCost(exp, stand, 1), 11.5));
    assert(near(unitCost(exp, stand, 2), 13.225));
    assert(near(unitCost(exp, stand, 10), 40.455577357079));  // 10 * 1.15^10
    assert(near(unitCost(exp, kTiers[1], 0), 100.0));

    // Linear costs have the same first step but grow by a fixed amount: base * (1 + n * (g - 1)).
    const Rules lin{CostMode::Linear, 1.15};
    assert(near(unitCost(lin, stand, 0), 10.0));
    assert(near(unitCost(lin, stand, 1), 11.5));
    assert(near(unitCost(lin, stand, 10), 25.0));
    assert(near(unitCost(lin, stand, 100), 160.0));

    // A growth of 1 or less is no growth: every purchase costs the base price.
    const Rules flat{CostMode::Exponential, 1.0};
    const Rules below{CostMode::Exponential, 0.5};
    assert(unitCost(flat, stand, 500) == 10.0);
    assert(unitCost(below, stand, 500) == 10.0);
    assert(unitCost(Rules{CostMode::Linear, 0.5}, stand, 500) == 10.0);
    assert(unitCost(Rules{CostMode::Linear, std::numeric_limits<double>::quiet_NaN()}, stand, 5) == 10.0);
    assert(unitCost(exp, stand, -4) == 10.0);  // a negative count is not a thing
}

void overflow() {
    const Rules exp;
    // Exponential costs leave double range without a warning: pow gives inf, and inf can never be afforded.
    assert(std::isinf(unitCost(exp, kTiers[0], 6000)));
    assert(maxAffordable(exp, kTiers[0], 6000, 1e300) == 0);
    assert(!canAfford(1e300, unitCost(exp, kTiers[0], 6000)));

    // The same cost as a 64 bit integer breaks much earlier: 10 * 1.15^n passes 2^63 between n = 295 and n = 296.
    const double int64Limit = 9223372036854775808.0;  // 2^63
    assert(unitCost(exp, kTiers[0], 295) < int64Limit);
    assert(unitCost(exp, kTiers[0], 296) > int64Limit);

    // A huge fortune still gives a finite, consistent answer.
    const std::int64_t k = maxAffordable(exp, kTiers[0], 0, 1e300);
    assert(k > 4000 && k < 6000);
    assert(bulkCost(exp, kTiers[0], 0, k) <= 1e300);
    assert(bulkCost(exp, kTiers[0], 0, k + 1) > 1e300);
}

void bulk() {
    const Rules exp;
    const Rules lin{CostMode::Linear, 1.15};
    const Tier& stand = kTiers[0];
    assert(bulkCost(exp, stand, 0, 0) == 0.0);
    assert(bulkCost(exp, stand, 0, 1) == 10.0);  // exactly the unit cost, no rounding noise
    assert(near(bulkCost(exp, stand, 0, 3), 34.725));  // 10 + 11.5 + 13.225
    assert(near(bulkCost(exp, stand, 2, 2), 28.43375));  // 13.225 + 15.20875
    assert(near(bulkCost(lin, stand, 2, 3), 43.5));      // 13 + 14.5 + 16
    assert(near(bulkCost(Rules{CostMode::Exponential, 1.0}, stand, 7, 5), 50.0));
    assert(bulkCost(exp, stand, 0, -3) == 0.0);

    assert(maxAffordable(exp, stand, 0, 9.99) == 0);
    assert(maxAffordable(exp, stand, 0, 10.0) == 1);
    assert(maxAffordable(exp, stand, 0, 34.72) == 2);
    assert(maxAffordable(exp, stand, 0, 34.73) == 3);
    assert(maxAffordable(lin, stand, 2, 43.49) == 2);
    assert(maxAffordable(lin, stand, 2, 43.51) == 3);
    assert(maxAffordable(Rules{CostMode::Exponential, 1.0}, stand, 7, 55.0) == 5);

    // Closed form against a plain loop, just inside and just outside every total.
    for (const Rules& rules : {exp, lin, Rules{CostMode::Exponential, 1.05}, Rules{CostMode::Exponential, 1.3}}) {
        for (const Tier& tier : kTiers) {
            for (const std::int64_t owned : {0, 1, 7, 40}) {
                double total = 0.0;
                for (std::int64_t k = 1; k <= 60; ++k) {
                    total += unitCost(rules, tier, owned + k - 1);
                    assert(maxAffordable(rules, tier, owned, total * 1.0000001) == k);
                    assert(maxAffordable(rules, tier, owned, total * 0.9999999) == k - 1);
                }
            }
        }
    }

    // Garbage in, nothing out.
    assert(maxAffordable(exp, stand, 0, -5.0) == 0);
    assert(maxAffordable(exp, stand, 0, std::numeric_limits<double>::quiet_NaN()) == 0);
    assert(maxAffordable(exp, stand, 0, kInf) == 0);
    // Linear costs with an absurd fortune stop at the count limit instead of running away.
    assert(maxAffordable(lin, stand, 0, 1e300) == kMaxCount);
}

void incomeAndPrestige() {
    State s;
    assert(near(multiplier(s), 1.0));
    s.owned = {3, 1, 0, 0};
    assert(near(income(s), 1.6));  // 3 * 0.2 + 1 * 1
    s.points = 10;
    assert(near(multiplier(s), 2.0));
    assert(near(income(s), 3.2));
    s.points = 12;
    assert(near(multiplier(s), 2.2));

    assert(pointsFor(0.0) == 0);
    assert(pointsFor(999.0) == 0);  // sqrt(0.999)
    assert(pointsFor(1000.0) == 1);
    assert(pointsFor(3999.0) == 1);
    assert(pointsFor(4000.0) == 2);
    assert(pointsFor(1e6) == 31);  // sqrt(1000) = 31.6
    assert(pointsFor(1e12) == 31622);
    assert(pointsFor(-5.0) == 0);
    assert(pointsFor(std::numeric_limits<double>::quiet_NaN()) == 0);
    assert(pointsFor(kInf) == kMaxPrestigePoints);
    assert(pointsFor(1e300) == kMaxPrestigePoints);  // sqrt(1e297) does not fit in an int64

    State a;
    a.owned = {5, 2, 1, 0};
    a.money = 123.0;
    a.time = 800.0;
    a.earned = 500.0;
    assert(!prestige(a));  // not worth a point yet, nothing changes
    assert(a.points == 0 && a.owned[0] == 5 && a.earned == 500.0);

    a.earned = 4000.0;
    assert(prestige(a));
    assert(a.points == 2);
    assert(a.owned == (std::array<std::int64_t, kTierCount>{0, 0, 0, 0}));
    assert(a.money == kStartMoney && a.time == 0.0 && a.earned == 0.0);
    assert(near(multiplier(a), 1.2));
    a.earned = 9000.0;
    assert(prestige(a));
    assert(a.points == 2 + 3);  // points add up over the runs
}

void targets() {
    const Rules rules;
    State s;
    // Fresh start: only the first stand is affordable, everything else waits for income that does not exist.
    Target t = pickTarget(rules, s);
    assert(t.tier == 0 && t.wait == 0.0 && t.cost == 10.0);

    // One stand, empty purse: the next stand (11.5) is 57.5 s away at 0.2 per second and pays back in 57.5 s more.
    s.owned = {1, 0, 0, 0};
    s.money = 0.0;
    t = pickTarget(rules, s);
    assert(t.tier == 0 && near(t.wait, 57.5) && near(t.cost, 11.5));

    // 30 stands cost 662.12 each by now (payback 3310 s). A shop for 100 pays back in 100 s, so it wins.
    s.owned = {30, 0, 0, 0};
    s.money = 0.0;
    t = pickTarget(rules, s);
    assert(t.tier == 1 && near(t.cost, 100.0) && near(t.wait, 100.0 / 6.0));
    s.money = 1e9;
    t = pickTarget(rules, s);
    assert(t.tier == 1 && t.wait == 0.0);

    // The multiplier makes everything pay back faster but does not change the order: still the shop.
    s.points = 10;
    s.money = 0.0;
    t = pickTarget(rules, s);
    assert(t.tier == 1 && near(t.wait, 100.0 / 12.0));  // income 6 * 2

    // waitFor: instant when affordable, never when there is no income.
    State e;
    e.money = 5.0;
    assert(waitFor(e, 10.0) == std::numeric_limits<double>::infinity());
    assert(waitFor(e, 5.0) == 0.0);
    assert(waitFor(e, 1.0) == 0.0);
    e.owned = {1, 0, 0, 0};
    assert(near(waitFor(e, 6.0), 5.0));  // 1 missing at 0.2 per second
}

void buying() {
    const Rules rules;
    State s;
    assert(buy(rules, s, 1, 1) == 0);  // 10 < 100
    assert(s.money == 10.0 && s.owned[1] == 0);
    assert(buy(rules, s, 0, 1) == 1);
    assert(s.money == 0.0 && s.owned[0] == 1);
    assert(buy(rules, s, 0, 0) == 0);
    assert(buy(rules, s, 0, -2) == 0);
    assert(buy(rules, s, 9, 1) == 0);  // no such tier
    assert(buy(rules, s, -1, 1) == 0);

    s.money = 34.73;
    s.owned = {0, 0, 0, 0};
    assert(buy(rules, s, 0, 4) == 0);  // all or nothing, 4 stands cost 49.9
    assert(buy(rules, s, 0, 3) == 3);
    assert(s.owned[0] == 3 && near(s.money, 0.005, 1e-6));

    // The count limit holds even with money to spare.
    State rich;
    rich.money = 1e300;
    rich.owned[0] = kMaxCount;
    assert(buy(Rules{CostMode::Linear, 1.15}, rich, 0, 1) == 0);
}

void advanceByHand() {
    const Rules rules;
    std::vector<Event> log;
    State s;
    // 10 money buys the first stand at t = 0. 0.2/s: stand 2 (11.5) at 57.5 s, stand 3 (13.225 at 0.4/s) 33.0625 s later.
    const double used = advance(rules, s, 100.0, 1000, true, &log);
    assert(near(used, 100.0));
    assert(s.owned[0] == 3 && s.owned[1] == 0);
    assert(near(s.time, 100.0));
    // 0.6/s for the last 9.4375 s.
    assert(near(s.money, 5.6625));
    assert(near(s.earned, 30.3875));  // 11.5 + 13.225 + 5.6625
    assert(log.size() == 3);
    assert(near(log[0].time, 0.0) && near(log[0].income, 0.2) && log[0].owned == 1 && log[0].tier == 0);
    assert(near(log[1].time, 57.5) && near(log[1].income, 0.4) && log[1].owned == 2);
    assert(near(log[2].time, 90.5625) && near(log[2].income, 0.6) && log[2].owned == 3);

    // Without the auto buyer money just piles up.
    State idle;
    idle.owned = {1, 0, 0, 0};
    idle.money = 0.0;
    assert(near(advance(rules, idle, 10.0, 1000, false, nullptr), 10.0));
    assert(near(idle.money, 2.0) && near(idle.earned, 2.0) && near(idle.time, 10.0));

    // The event cap stops the simulation early and reports how far it got.
    State capped;
    const double got = advance(rules, capped, 100.0, 2, true, nullptr);
    assert(near(got, 57.5));
    assert(capped.owned[0] == 2 && near(capped.time, 57.5));

    // Nonsense time steps do nothing.
    State n;
    assert(advance(rules, n, 0.0, 10, true, nullptr) == 0.0);
    assert(advance(rules, n, -5.0, 10, true, nullptr) == 0.0);
    assert(advance(rules, n, std::numeric_limits<double>::quiet_NaN(), 10, true, nullptr) == 0.0);
    assert(advance(rules, n, 5.0, 0, true, nullptr) == 0.0);
    assert(n.time == 0.0 && n.owned[0] == 0);
}

void advanceLongRuns() {
    const Rules rules;
    // Reference values from an independent run of the same rules (a short Python model): one hour of play.
    State s;
    assert(near(advance(rules, s, 3600.0, 100000, true, nullptr), 3600.0));
    assert((s.owned == std::array<std::int64_t, kTierCount>{32, 27, 24, 18}));
    assert(near(income(s), 1071.4));
    assert(near(s.earned, 1570074.51, 1e-6));
    assert(pointsFor(s.earned) == 39);

    // Prestige turns that into x4.9 and the next 20 minutes reach further than the first hour did.
    assert(prestige(s));
    assert(s.points == 39 && near(multiplier(s), 4.9));
    advance(rules, s, 1200.0, 100000, true, nullptr);
    assert((s.owned == std::array<std::int64_t, kTierCount>{40, 35, 32, 26}));
    assert(near(income(s), 7452.9));

    // Linear costs keep buying: after the same hour there are far more shops and factories, and the income is higher
    // than the exponential run's, because nothing ever gets expensive.
    State lin;
    advance(Rules{CostMode::Linear, 1.15}, lin, 3600.0, 100000, true, nullptr);
    assert((lin.owned == std::array<std::int64_t, kTierCount>{271, 132, 86, 36}));
    assert(income(lin) > 2.0 * 1071.4);
}

void sliceInvariance() {
    // The result must not depend on how the time is cut into frames.
    const Rules rules;
    State whole;
    advance(rules, whole, 3600.0, 100000, true, nullptr);
    State sliced;
    for (int i = 0; i < 36000; ++i) advance(rules, sliced, 0.1, 100000, true, nullptr);
    assert(whole.owned == sliced.owned);
    assert(near(whole.time, sliced.time, 1e-6));
    assert(near(whole.money, sliced.money, 1e-6));
    assert(near(whole.earned, sliced.earned, 1e-6));

    // 60 fps with one huge speed: same again.
    State big;
    for (int i = 0; i < 600; ++i) advance(rules, big, 6.0, 100000, true, nullptr);
    assert(whole.owned == big.owned);
}

void finiteAtAnyScale() {
    // An absurd stretch of time: the purse and the earnings stay finite and capped, nothing turns into inf or NaN.
    const Rules rules;
    State s;
    s.owned = {kMaxCount, kMaxCount, kMaxCount, kMaxCount};
    s.points = kMaxPrestigePoints;
    assert(near(advance(rules, s, 1e295, 10, true, nullptr), 1e295));
    assert(std::isfinite(s.money) && s.money <= kMoneyCap);
    assert(std::isfinite(s.earned) && s.earned <= kMoneyCap);
    assert(pointsFor(s.earned) > 0 && pointsFor(s.earned) <= kMaxPrestigePoints);
    assert(prestige(s));
    assert(s.points == kMaxPrestigePoints);  // the point cap holds across prestiges
    assert(formatBig(income(s)).size() > 0);
}

void noStallOrNegativeMoney() {
    // A long run with prestiges: the purse never goes below zero, time always arrives, events stay bounded.
    for (const Rules& rules : {Rules{}, Rules{CostMode::Linear, 1.15}, Rules{CostMode::Exponential, 1.02}, Rules{CostMode::Exponential, 1.3}}) {
        State s;
        std::vector<Event> log;
        for (int run = 0; run < 4; ++run) {
            double left = 86400.0;
            int guard = 0;
            while (left > 0.0 && guard++ < 2000) {
                const double used = advance(rules, s, std::min(left, 60.0), 5000, true, &log);
                assert(s.money >= 0.0);
                assert(std::isfinite(s.money) && std::isfinite(s.earned));
                if (!(used > 0.0)) break;
                left -= used;
            }
            assert(left <= 0.0 || guard >= 2000);
            prestige(s);
        }
        assert(!log.empty());
        for (std::size_t i = 1; i < log.size(); ++i) assert(log[i].time >= 0.0);
    }
}

void game() {
    // A short scene: an hour, a prestige worth 39 points, twenty minutes of the second run.
    Game g;
    g.restart(3600.0, 1200.0);
    assert(g.state.points == 39);
    assert((g.state.owned == std::array<std::int64_t, kTierCount>{40, 35, 32, 26}));
    assert(near(g.state.time, 1200.0));
    assert(g.history.runs.size() == 2);
    assert(!g.history.runs[0].empty() && !g.history.runs[1].empty());
    // 101 buildings after the first hour, 133 after twenty minutes with the bonus.
    assert(g.history.runs[0].back().owned == 101.0);
    assert(g.history.runs[1].back().owned == 133.0);
    assert(g.history.runs[1].back().income > g.history.runs[0].back().income);
    // Within a run, income and the building count never drop.
    for (const RunLog& run : g.history.runs) {
        for (std::size_t i = 1; i < run.size(); ++i) {
            assert(run[i].t >= run[i - 1].t);
            assert(run[i].income >= run[i - 1].income);
            assert(run[i].owned >= run[i - 1].owned);
        }
    }

    // The scene the demo starts with: a day, a prestige worth 465 points (x47.5), then one hour of the new run, which
    // already owns more buildings than the first run did after the whole day.
    Game day;
    day.restart();
    assert(day.state.points == 465);
    assert((day.state.owned == std::array<std::int64_t, kTierCount>{73, 68, 65, 59}));
    assert(near(day.state.time, 3600.0));
    assert(day.history.runs[0].back().owned == 241.0);
    assert(day.history.runs[1].back().owned == 265.0);
    assert(near(income(day.state), 160341.0, 1e-5));

    // Manual play lands in the history as well.
    Game m;
    assert(!m.prestigeNow());  // nothing earned yet
    assert(m.buyNow(1, 1) == 0);
    assert(m.buyNow(0, 1) == 1);
    assert(m.history.runs[0].size() == 1 && m.state.owned[0] == 1);
    m.autoBuy = false;
    assert(near(m.run(100.0, 10), 100.0));  // 20 earned, nothing bought
    assert(m.state.owned[0] == 1 && near(m.state.money, 20.0));
    assert(m.history.runs[0].size() == 1);

    // Flat prices make millions of purchases: the restart gives up at its event budget instead of hanging.
    Game flat;
    flat.rules.growth = 1.0;
    flat.restart();
    assert(flat.state.time > 0.0 || flat.state.points > 0);
}

void history() {
    History h;
    assert(h.runs.size() == 1);
    for (int i = 0; i < 5000; ++i) h.add(Event{static_cast<double>(i), 1.0 + i, 2 + i, 0});
    assert(h.runs[0].size() <= kMaxPoints);
    assert(h.runs[0].size() > kMaxPoints / 4);  // thinned, not emptied
    assert(near(h.runs[0].front().t, 0.0));
    assert(near(h.runs[0].back().t, 4999.0));  // the newest point is never thinned away
    assert(near(h.runs[0].back().income, 5000.0));
    for (std::size_t i = 1; i < h.runs[0].size(); ++i) assert(h.runs[0][i].t > h.runs[0][i - 1].t);

    for (int i = 0; i < 8; ++i) h.newRun();
    assert(h.runs.size() == kMaxRuns);  // the oldest runs fall off
    assert(h.runs.back().empty());
    h.add(Event{3.0, 7.0, 9, 1});
    assert(h.runs.back().size() == 1 && h.runs.back()[0].income == 7.0 && h.runs.back()[0].owned == 9.0);
    h.clear();
    assert(h.runs.size() == 1 && h.runs[0].empty());
}

/// Income the plot shows at time t: the newest point at or before t (the step function the plot draws).
double shown(const RunLog& run, double t) {
    double v = 0.0;
    for (const Point& p : run) {
        if (p.t <= t) v = p.income;
    }
    return v;
}

void historyKeepsSparseStretches() {
    // 100 purchases 40 s apart, then 4900 purchases every half second. Thinning by index would leave the slow start
    // with a handful of points and a long flat line; thinning by time keeps every one of the slow purchases.
    History h;
    double t = 0.0;
    for (int i = 0; i < 100; ++i, t += 40.0) h.add(Event{t, 1.0 + i, 1, 0});
    for (int i = 100; i < 5000; ++i, t += 0.5) h.add(Event{t, 1.0 + i, 1, 0});
    const RunLog& run = h.runs[0];
    assert(run.size() <= kMaxPoints);
    for (int i = 0; i < 100; ++i) assert(shown(run, 40.0 * i) == 1.0 + i);
    // The fast stretch is at worst one cell behind the exact step function: cell seconds hold at most 2 * cell + 1 purchases.
    assert(run.cell > 0.0 && run.cell < 40.0);
    for (double q = 4000.0; q < t; q += 37.0) {
        const double exact = 101.0 + std::floor((q - 4000.0) / 0.5);
        assert(std::fabs(shown(run, q) - exact) <= 2.0 * run.cell + 2.0);
    }

    // Many purchases at one instant cannot be merged by time, the count limit still holds.
    History same;
    for (int i = 0; i < 5000; ++i) same.add(Event{0.0, 1.0 + i, 1, 0});
    assert(same.runs[0].size() <= kMaxPoints);
    assert(same.runs[0].back().income == 5000.0);
}

void bigNumbers() {
    assert(formatBig(0.0) == "0");
    assert(formatBig(0.2) == "0.2");
    assert(formatBig(1.5) == "1.5");
    assert(formatBig(0.01) == "0.01");
    assert(formatBig(0.004) == "0");
    assert(formatBig(12.34) == "12.34");
    assert(formatBig(999.0) == "999");
    assert(formatBig(999.994) == "999.99");
    assert(formatBig(999.996) == "1,000");  // rounds up into the next group
    assert(formatBig(1000.0) == "1,000");
    assert(formatBig(12345.0) == "12,345");
    assert(formatBig(999999.4) == "999,999");
    assert(formatBig(999999.6) == "1.00M");
    assert(formatBig(1e6) == "1.00M");
    assert(formatBig(1234567.0) == "1.23M");
    assert(formatBig(12345678.0) == "12.3M");
    assert(formatBig(123456789.0) == "123M");
    assert(formatBig(999600000.0) == "1.00B");  // 999.6M rounds to 1.00B
    assert(formatBig(1e9) == "1.00B");
    assert(formatBig(1e12) == "1.00T");
    assert(formatBig(1.23456e15) == "1.23Qa");
    assert(formatBig(1e18) == "1.00Qi");
    assert(formatBig(1e21) == "1.00Sx");
    assert(formatBig(9.999e20) == "1.00Sx");  // carry from 999.9Qi
    assert(formatBig(1e24) == "1.00Sp");
    assert(formatBig(1e27) == "1.00Oc");
    assert(formatBig(1e30) == "1.00No");
    assert(formatBig(1e33) == "1.00Dc");
    assert(formatBig(9.999e32) == "1.00Dc");
    assert(formatBig(1.5e35) == "150Dc");
    assert(formatBig(9.996e35) == "1.00e36");  // the carry leaves the suffix table
    assert(formatBig(1e36) == "1.00e36");
    assert(formatBig(1.5e100) == "1.50e100");
    assert(formatBig(1e300) == "1.00e300");
    assert(formatBig(std::numeric_limits<double>::max()) == "1.80e308");
    assert(formatBig(-1234.0) == "-1,234");
    assert(formatBig(-5e9) == "-5.00B");
    assert(formatBig(kInf) == "inf");
    assert(formatBig(-kInf) == "-inf");
    assert(formatBig(std::numeric_limits<double>::quiet_NaN()) == "-");
    assert(formatBig(std::numeric_limits<double>::denorm_min()) == "0");

    // Each power of ten from 1e3 to 1e36 starts a group of its own, whatever the last bit of pow() says.
    const char* suffixes[] = {"K", "M", "B", "T", "Qa", "Qi", "Sx", "Sp", "Oc", "No", "Dc"};
    for (int e = 6; e <= 33; ++e) {
        const std::string expected = "1.00" + std::string(suffixes[e / 3 - 1]);
        const double v = std::pow(10.0, e);
        if (e % 3 == 0) assert(formatBig(v) == expected);
    }
    // Digits: 3 significant ones, whichever group.
    assert(formatBig(1.5e6) == "1.50M");
    assert(formatBig(15e6) == "15.0M");
    assert(formatBig(150e6) == "150M");
}

void thousands() {
    assert(formatCount(0) == "0");
    assert(formatCount(999) == "999");
    assert(formatCount(1000) == "1,000");
    assert(formatCount(1234567) == "1,234,567");
    assert(formatCount(-1234567) == "-1,234,567");
    assert(formatCount(std::numeric_limits<std::int64_t>::max()) == "9,223,372,036,854,775,807");
    assert(formatCount(std::numeric_limits<std::int64_t>::min()) == "-9,223,372,036,854,775,808");
}

void durations() {
    assert(formatDuration(0.0) == "0s");
    assert(formatDuration(0.4) == "<1s");
    assert(formatDuration(1.0) == "1s");
    assert(formatDuration(45.0) == "45s");
    assert(formatDuration(59.4) == "59s");
    assert(formatDuration(59.6) == "1m 00s");
    assert(formatDuration(61.0) == "1m 01s");
    assert(formatDuration(3599.4) == "59m 59s");
    assert(formatDuration(3599.6) == "1h 00m");
    assert(formatDuration(3600.0) == "1h 00m");
    assert(formatDuration(3725.0) == "1h 02m");
    assert(formatDuration(86399.0) == "23h 59m");
    assert(formatDuration(90000.0) == "1d 01h");
    assert(formatDuration(364.0 * 86400.0) == "364d 00h");
    assert(formatDuration(365.0 * 86400.0) == "1y 0d");
    assert(formatDuration(400.0 * 86400.0) == "1y 35d");
    assert(formatDuration(1e12) == "31,710y");
    assert(formatDuration(1e30) == "31.7Sxy");
    assert(formatDuration(kInf) == "never");
    assert(formatDuration(-3.0) == "-");
    assert(formatDuration(std::numeric_limits<double>::quiet_NaN()) == "-");
}

void axes() {
    const Axis lin = makeAxis(0.0, 87.0, false, 5);
    assert(near(lin.lo, 0.0) && near(lin.hi, 100.0));
    assert(lin.ticks.size() == 6);  // 0, 20, ... 100
    assert(near(lin.map(50.0), 0.5));
    assert(near(lin.map(-5.0), 0.0) && near(lin.map(1e9), 1.0));

    const Axis log = makeAxis(1.0, 1000.0, true, 8);
    assert(log.ticks.size() == 4);  // 1, 10, 100, 1000
    assert(near(log.lo, 1.0) && near(log.hi, 1000.0));
    assert(near(log.map(10.0), 1.0 / 3.0));
    assert(near(log.map(100.0), 2.0 / 3.0));
    assert(log.map(0.0) == 0.0 && log.map(-3.0) == 0.0);  // below a log axis, zero included, sits on the floor
    assert(log.map(kInf) == 1.0);

    const Axis many = makeAxis(1.0, 1e12, true, 5);  // 13 decades with 5 labels: every third
    assert(many.ticks.size() <= 5);
    assert(near(many.ticks[0], 1.0));

    const Axis flat = makeAxis(50.0, 50.0, true, 6);
    assert(flat.hi > flat.lo);
    const Axis floored = makeAxis(0.0, 90.0, true, 6, 0.1);
    assert(near(floored.lo, 0.1));
    const Axis empty = makeAxis(0.0, 0.0, false, 4);
    assert(empty.hi > empty.lo);

    // Time ticks.
    assert(near(niceTimeStep(600.0, 6), 120.0));
    assert(near(niceTimeStep(3600.0, 6), 600.0));
    assert(near(niceTimeStep(90000.0, 6), 21600.0));
    assert(near(niceTimeStep(40.0, 6), 10.0));
    assert(near(niceTimeStep(0.0, 6), 1.0));
    assert(near(niceTimeStep(kInf, 6), 1.0));
    assert(niceTimeStep(1e9, 5) * 5.0 >= 1e9);  // beyond the table it still produces a step that fits
}

void timeAxes() {
    const TimeAxis lin = makeTimeAxis(3000.0, false, 6);  // 3000 s: steps of 10 minutes
    assert(near(lin.hi, 3000.0) && lin.ticks.size() == 6 && near(lin.ticks[1], 600.0));
    assert(near(lin.map(1500.0), 0.5) && lin.map(-1.0) == 0.0 && lin.map(1e9) == 1.0);
    const TimeAxis tiny = makeTimeAxis(0.0, false, 6);  // nothing happened yet, still a usable minute
    assert(tiny.hi >= 60.0 && tiny.ticks.size() >= 2);

    const TimeAxis log = makeTimeAxis(4000.0, true, 6);
    assert(near(log.hi, 4000.0));
    assert((log.ticks == std::vector<double>{1.0, 10.0, 60.0, 600.0, 3600.0}));
    assert(near(log.map(1.0), 0.0) && near(log.map(4000.0), 1.0));
    assert(near(log.map(100.0), std::log10(100.0) / std::log10(4000.0)));
    assert(log.map(0.0) == 0.0 && log.map(0.3) == 0.0 && log.map(-5.0) == 0.0);  // before 1 s is the left edge
    assert(log.map(kInf) == 1.0 && log.map(std::numeric_limits<double>::quiet_NaN()) == 0.0);
    const TimeAxis shortLog = makeTimeAxis(2.0, true, 6);  // never narrower than one decade
    assert(near(shortLog.hi, 10.0) && shortLog.ticks.size() == 2);
    const TimeAxis longLog = makeTimeAxis(86400.0 * 400.0, true, 6);
    assert(longLog.ticks.size() >= 8 && longLog.ticks.back() <= longLog.hi);
}

void decades() {
    RunLog run;
    for (const Point& p : {Point{0.0, 1.0, 1.0}, Point{10.0, 2.0, 5.0}, Point{100.0, 3.0, 12.0}, Point{1000.0, 4.0, 20.0}}) run.push_back(p);
    assert(ownedAt(run, -1.0) == 0.0);
    assert(ownedAt(run, 0.0) == 1.0);
    assert(ownedAt(run, 9.99) == 1.0);
    assert(ownedAt(run, 10.0) == 5.0);  // the purchase at exactly t counts
    assert(ownedAt(run, 50.0) == 5.0);
    assert(ownedAt(run, 1e9) == 20.0);
    assert(ownedAt(RunLog{}, 5.0) == 0.0);
    assert(buildingsPerDecade(run, 100.0) == 12.0 - 5.0);
    assert(buildingsPerDecade(run, 1000.0) == 20.0 - 12.0);
    assert(buildingsPerDecade(run, 10.0) == 5.0 - 1.0);  // at t = 10 the earlier point is t = 1
    assert(buildingsPerDecade(run, 9.0) == 0.0);          // too early for a decade
    assert(buildingsPerDecade(run, std::numeric_limits<double>::quiet_NaN()) == 0.0);

    // The price curve's logarithm: from day one on, every tenfold time adds about the same number of buildings. Measured
    // by this simulation (4 tiers, growth 1.15): 89, 76, 75 and 72 more per decade from 1e4 s to 1e8 s.
    Game g;
    double at[4] = {};
    const double marks[4] = {1e4, 1e5, 1e6, 1e7};
    double clock = 0.0;
    for (int i = 0; i < 4; ++i) {
        while (clock < marks[i]) {
            const double used = g.run(std::min(marks[i] - clock, 3600.0), 5000);
            assert(used > 0.0);
            clock += used;
        }
        at[i] = static_cast<double>(totalOwned(g.state));
    }
    assert(near(at[0], 157.0) && near(at[1], 246.0) && near(at[2], 322.0) && near(at[3], 397.0));
    const double d2 = at[2] - at[1];
    const double d3 = at[3] - at[2];
    assert(d2 > 60.0 && d2 < 90.0 && d3 > 60.0 && d3 < 90.0);
    assert(std::fabs(d3 - d2) < 0.15 * d2);
    // A steeper price curve gives fewer buildings per decade, a flatter one more (about 40, 76 and 160 for 1.30, 1.15, 1.07).
    auto perDecade = [](double growth) {
        Game x;
        x.rules.growth = growth;
        double c = 0.0;
        double n5 = 0.0;
        double n6 = 0.0;
        for (const double m : {1e5, 1e6}) {
            while (c < m) c += x.run(std::min(m - c, 3600.0), 5000);
            (m == 1e5 ? n5 : n6) = static_cast<double>(totalOwned(x.state));
        }
        return n6 - n5;
    };
    assert(perDecade(1.30) < perDecade(1.15) && perDecade(1.15) < perDecade(1.07));
    assert(perDecade(1.30) > 30.0 && perDecade(1.30) < 50.0 && perDecade(1.07) > 140.0 && perDecade(1.07) < 180.0);
}

}  // namespace

int main() {
    costs();
    overflow();
    bulk();
    incomeAndPrestige();
    targets();
    buying();
    advanceByHand();
    advanceLongRuns();
    sliceInvariance();
    finiteAtAnyScale();
    noStallOrNegativeMoney();
    history();
    historyKeepsSparseStretches();
    game();
    bigNumbers();
    thousands();
    durations();
    axes();
    timeAxes();
    decades();
    std::puts("idle-economy logic: all asserts passed");
    return 0;
}
