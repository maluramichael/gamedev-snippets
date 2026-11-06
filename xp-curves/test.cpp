// Tests for the XP curve logic. Plain asserts on hand-computed cases, no framework, no SDL.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <string>

#include "logic.hpp"

using namespace xpc;

namespace {

bool near(double a, double b, double eps = 1e-9) { return std::fabs(a - b) < eps; }

constexpr Family kAll[] = {Family::Linear, Family::Polynomial, Family::Exponential, Family::RuneScape, Family::Table};

void linearFamily() {
    // xp(L) = a*L is the cost of the step L -> L+1: 10, 20, 30, ... so total(L) = 10 * L * (L-1) / 2.
    const Curve c(Family::Linear, {10.0, 0.0});
    assert(c.total(1) == 0);
    assert(c.total(2) == 10);
    assert(c.total(3) == 30);
    assert(c.total(4) == 60);
    assert(c.total(5) == 100);
    assert(c.next(1) == 10);
    assert(c.next(98) == 980);
    assert(c.total(99) == 48510);  // 10 * 99 * 98 / 2
}

void polynomialFamily() {
    // xp(L) = 2 * L^2: 2, 8, 18, 32 -> totals 0, 2, 10, 28, 60. total(10) = 2 * (1 + 4 + ... + 81) = 570.
    const Curve c(Family::Polynomial, {2.0, 2.0});
    assert(c.total(2) == 2);
    assert(c.total(3) == 10);
    assert(c.total(4) == 28);
    assert(c.total(5) == 60);
    assert(c.total(10) == 570);
    // b = 0 makes every step cost a.
    const Curve flat(Family::Polynomial, {5.0, 0.0});
    assert(flat.next(1) == 5 && flat.next(50) == 5);
    // A negative exponent is not a growing curve, it is treated as 0.
    const Curve neg(Family::Polynomial, {5.0, -2.0});
    assert(neg.next(50) == 5);
}

void exponentialFamily() {
    // xp(L) = 10 * 2^L: 20, 40, 80, 160 -> totals 0, 20, 60, 140, 300.
    const Curve c(Family::Exponential, {10.0, 2.0});
    assert(c.next(1) == 20);
    assert(c.total(2) == 20);
    assert(c.total(3) == 60);
    assert(c.total(4) == 140);
    assert(c.total(5) == 300);
    assert(c.total(10) == 10 * ((1 << 10) - 2));
    // b <= 1 would shrink the steps, it is treated as 1 (constant steps of a).
    const Curve one(Family::Exponential, {7.0, 1.0});
    const Curve below(Family::Exponential, {7.0, 0.5});
    assert(one.total(11) == 70);
    assert(below.total(11) == 70);
    assert(below.next(99) == 7);
}

void runescape() {
    // points(l) = floor(l + 300 * 2^(l/7)), total(L) = floor(sum of points(l) for l < L / 4).
    // Hand check of the first step: 1 + 300 * 2^(1/7) = 1 + 331.2 = 332.2 -> 332, 332 / 4 = 83.
    const Curve c(Family::RuneScape, {});
    assert(c.total(1) == 0);
    assert(c.total(2) == 83);
    assert(c.total(3) == 174);
    assert(c.total(4) == 276);
    assert(c.total(5) == 388);
    assert(c.total(10) == 1154);
    assert(c.total(11) == 1358);
    assert(c.total(50) == 101333);
    assert(c.total(99) == 13034431);
    assert(c.next(1) == 83);
    assert(c.next(2) == 91);
    assert(c.next(49) == 9612);
    assert(c.next(98) == 1228825);
    // The step doubles about every 7 levels (it is 2^(1/7) = +10.4 percent per level).
    const double ratio = static_cast<double>(c.next(91)) / static_cast<double>(c.next(84));
    assert(ratio > 1.95 && ratio < 2.05);
}

void tableFamily() {
    const Curve c(Family::Table, {});
    // The anchors are exact.
    assert(c.next(1) == 100);
    assert(c.next(10) == 400);
    assert(c.next(20) == 1000);
    assert(c.next(99) == 110000);
    // Halfway between 400 and 1000 (geometric): 400 * sqrt(2.5) = 632.46, rounded to tens.
    assert(c.next(15) == 630);
    assert(c.total(2) == 100);
}

void monotonic() {
    const Params sets[] = {{0.0, 0.0}, {1.0, 1.0}, {10.0, 2.0}, {3.0, 2.6}, {60.0, 1.1}, {10.0, 2.0}, {1e9, 9.0}};
    for (const Family f : kAll) {
        for (const Params p : sets) {
            const Curve c(f, p);
            assert(c.total(1) == 0);
            for (int l = 1; l < kMaxLevel; ++l) {
                assert(c.next(l) >= 1);              // every level costs something, so totals strictly rise
                assert(c.total(l + 1) > c.total(l));
            }
        }
    }
    // The cost of a step never falls for the growing families.
    for (const Family f : kAll) {
        const Curve c(f, paramInfo(f).def);
        for (int l = 2; l <= kMaxLevel; ++l) assert(c.next(l) >= c.next(l - 1));
    }
}

void parameterEdgeCases() {
    // a = 0 would make every level free (and totals flat): each level still costs 1.
    const Curve zero(Family::Linear, {0.0, 0.0});
    assert(zero.next(1) == 1 && zero.next(99) == 1);
    assert(zero.total(99) == 98);
    // NaN and infinity must not poison the totals.
    const Curve nan(Family::Polynomial, {std::numeric_limits<double>::quiet_NaN(), 2.0});
    assert(nan.next(5) == 1);
    const Curve inf(Family::Linear, {std::numeric_limits<double>::infinity(), 0.0});
    assert(inf.next(5) == kMaxStep);
    // 10 * 2^99 does not fit in 64 bit: the step saturates, the sum still fits (99 * 4e15 < 9.2e18).
    const Curve huge(Family::Exponential, {10.0, 2.0});
    assert(huge.next(99) == kMaxStep);
    assert(huge.total(99) > 0);
    assert(huge.total(100) > huge.total(99));
    assert(huge.total(100) <= 99 * kMaxStep);
}

void levelLookup() {
    const Curve lin(Family::Linear, {10.0, 0.0});  // totals 0, 10, 30, 60, 100
    assert(lin.levelForXp(0) == 1);
    assert(lin.levelForXp(9) == 1);
    assert(lin.levelForXp(10) == 2);  // exactly on the threshold is the new level
    assert(lin.levelForXp(29) == 2);
    assert(lin.levelForXp(30) == 3);
    assert(lin.levelForXp(59) == 3);
    assert(lin.levelForXp(60) == 4);
    assert(lin.levelForXp(-5) == 1);
    assert(lin.levelForXp(48509) == 98);
    assert(lin.levelForXp(48510) == 99);
    assert(lin.levelForXp(std::numeric_limits<std::int64_t>::max()) == kMaxLevel);

    const Curve rs(Family::RuneScape, {});
    assert(rs.levelForXp(82) == 1);
    assert(rs.levelForXp(83) == 2);
    assert(rs.levelForXp(1153) == 9);
    assert(rs.levelForXp(1154) == 10);
    assert(rs.levelForXp(13034430) == 98);
    assert(rs.levelForXp(13034431) == 99);
    assert(rs.levelForXp(200000000) == 99);  // beyond the cap stays at the cap

    // The binary search agrees with every threshold of every family.
    for (const Family f : kAll) {
        const Curve c(f, paramInfo(f).def);
        for (int l = 1; l <= kMaxLevel; ++l) {
            assert(c.levelForXp(c.total(l)) == l);
            if (l > 1) assert(c.levelForXp(c.total(l) - 1) == l - 1);
        }
    }
}

void hours() {
    const Curve rs(Family::RuneScape, {});
    assert(near(hoursToLevel(rs, 10, 1154.0), 1.0));
    assert(near(hoursToLevel(rs, 1, 1154.0), 0.0));
    assert(near(hoursForNext(rs, 1, 83.0), 1.0));
    assert(near(hoursForNext(rs, 2, 182.0), 0.5));  // 91 / 182
    assert(std::isinf(hoursToLevel(rs, 10, 0.0)));
    assert(std::isinf(hoursForNext(rs, 10, -3.0)));
}

void rateSlider() {
    assert(near(rateFromSlider(0.0), kRateMin));
    assert(near(rateFromSlider(1.0), kRateMax));
    assert(near(rateFromSlider(0.5), 31622.776601683796, 1e-6));  // 100 * 10^2.5
    assert(near(rateFromSlider(-3.0), kRateMin));
    assert(near(rateFromSlider(7.0), kRateMax));
    assert(near(sliderFromRate(kRateMin), 0.0));
    assert(near(sliderFromRate(kRateMax), 1.0));
    assert(near(sliderFromRate(1.0), 0.0));  // below the range clamps
    for (const double t : {0.1, 0.37, 0.8}) assert(near(sliderFromRate(rateFromSlider(t)), t, 1e-12));
}

void significant() {
    assert(near(roundSignificant(31622.776601683796, 2), 32000.0));
    assert(near(roundSignificant(123456.0, 3), 123000.0));
    assert(near(roundSignificant(99.5, 2), 100.0));
    assert(near(roundSignificant(7.0, 2), 7.0));
    assert(near(roundSignificant(0.0, 2), 0.0));
}

void player() {
    const Curve rs(Family::RuneScape, {});
    Player p;
    assert(rs.levelForXp(p.xpInt()) == 1);
    advance(p, rs, 1.0, 83.0);  // one hour at 83 xp/h
    assert(near(p.xp, 83.0));
    assert(rs.levelForXp(p.xpInt()) == 2);
    p.xp = 128.5;  // 45.5 into the 91 xp of level 2
    assert(near(levelProgress(rs, p.xp), 0.5));
    assert(near(levelProgress(rs, 83.0), 0.0));
    assert(near(levelProgress(rs, 0.0), 0.0));
    advance(p, rs, 1e9, 1e7);  // way past the cap
    assert(near(p.xp, 13034431.0));
    assert(rs.levelForXp(p.xpInt()) == kMaxLevel);
    assert(near(levelProgress(rs, p.xp), 1.0));
    advance(p, rs, 1.0, 0.0);  // no rate, no change
    assert(near(p.xp, 13034431.0));
}

void thousands() {
    assert(formatThousands(0) == "0");
    assert(formatThousands(7) == "7");
    assert(formatThousands(999) == "999");
    assert(formatThousands(1000) == "1,000");
    assert(formatThousands(83) == "83");
    assert(formatThousands(101333) == "101,333");
    assert(formatThousands(13034431) == "13,034,431");
    assert(formatThousands(-1234567) == "-1,234,567");
    assert(formatThousands(std::numeric_limits<std::int64_t>::max()) == "9,223,372,036,854,775,807");
    assert(formatThousands(std::numeric_limits<std::int64_t>::min()) == "-9,223,372,036,854,775,808");
}

void shortAndHours() {
    assert(formatShort(0.0) == "0");
    assert(formatShort(5.0) == "5");
    assert(formatShort(20.0) == "20");
    assert(formatShort(999.0) == "999");
    assert(formatShort(1000.0) == "1k");
    assert(formatShort(1500.0) == "1.5k");
    assert(formatShort(50000.0) == "50k");
    assert(formatShort(5e6) == "5M");
    assert(formatShort(13034431.0) == "13M");
    assert(formatShort(2.5e9) == "2.5B");
    assert(formatShort(0.1) == "0.1");
    assert(formatShort(0.01) == "0.01");
    assert(formatShort(0.001) == "0.001");
    assert(formatShort(0.5) == "0.5");
    assert(formatShort(999999.0) == "1M");  // 1000k rolls over to the next unit
    assert(formatShort(-1500.0) == "-1.5k");

    assert(formatHours(std::numeric_limits<double>::infinity()) == "never");
    assert(formatHours(0.0) == "<1 min");
    assert(formatHours(0.5) == "30 min");
    assert(formatHours(0.0083) == "<1 min");
    assert(formatHours(0.999) == "1.0 h");  // 59.94 min rounds up to an hour
    assert(formatHours(1.0) == "1.0 h");
    assert(formatHours(12.34) == "12.3 h");
    assert(formatHours(99.96) == "100 h");
    assert(formatHours(130.2) == "130 h");
    assert(formatHours(1234.0) == "1,234 h");
}

void axes() {
    assert(near(niceStep(100.0, 4), 50.0));            // raw 25 -> 5 * 10
    assert(near(niceStep(100.0, 5), 20.0));            // raw 20 -> 2 * 10
    assert(near(niceStep(13034431.0, 5), 5e6));        // raw 2.6e6 -> 5 * 1e6
    assert(near(niceStep(0.0, 5), 1.0));
    assert(near(niceStep(10.0, 5), 2.0));              // raw 2 stays 2

    const Axis lin = makeAxis(0.0, 100.0, false, 5);
    assert(near(lin.lo, 0.0) && near(lin.hi, 100.0));
    assert(lin.ticks.size() == 6);                      // 0, 20, ..., 100
    assert(near(lin.ticks[1], 20.0));
    assert(near(lin.map(50.0), 0.5));
    assert(near(lin.map(-4.0), 0.0) && near(lin.map(500.0), 1.0));

    const Axis big = makeAxis(0.0, 13034431.0, false, 5);
    assert(near(big.hi, 15e6));
    assert(big.ticks.size() == 4);                      // 0, 5M, 10M, 15M

    const Axis lg = makeAxis(83.0, 13034431.0, true, 6);
    assert(near(lg.lo, 10.0) && near(lg.hi, 1e8));
    assert(near(lg.map(1000.0), 2.0 / 7.0));
    assert(near(lg.map(10.0), 0.0) && near(lg.map(1e8), 1.0));
    assert(near(lg.map(1.0), 0.0));                     // below the range clamps
    assert(lg.ticks.size() == 4);                       // 8 decades would not fit: every 2nd one, 1e1 1e3 1e5 1e7
    assert(near(lg.ticks[1], 1000.0) && near(lg.ticks[3], 1e7));
    const Axis few = makeAxis(1.0, 1000.0, true, 6);
    assert(few.ticks.size() == 4);                      // 1, 10, 100, 1000
    assert(near(few.lo, 1.0) && near(few.hi, 1000.0));
    const Axis flat = makeAxis(50.0, 50.0, true, 6);    // a single value still gets one decade
    assert(flat.hi > flat.lo);
    const Axis floored = makeAxis(0.0, 90.0, true, 6, 0.01);
    assert(near(floored.lo, 0.01));                     // zero data cannot go below the floor
}

}  // namespace

int main() {
    linearFamily();
    polynomialFamily();
    exponentialFamily();
    runescape();
    tableFamily();
    monotonic();
    parameterEdgeCases();
    levelLookup();
    hours();
    rateSlider();
    significant();
    player();
    thousands();
    shortAndHours();
    axes();
    std::puts("xp-curves logic: all asserts passed");
    return 0;
}
