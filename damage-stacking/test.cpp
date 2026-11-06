// Tests for the damage-stacking logic. Plain asserts on hand-computed cases, no framework, no SDL.
// "Increased" modifiers are added into one pool, "More" modifiers are multiplied one by one.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "logic.hpp"

using namespace ds;

namespace {

bool near(double a, double b) { return std::fabs(a - b) < 1e-9; }

const Bucket kInc = Bucket::Increased;
const Bucket kMore = Bucket::More;

void emptyListChangesNothing() {
    const std::vector<Mod> none;
    for (const Rule rule : {Rule::Buckets, Rule::AddAll, Rule::MultiplyAll}) {
        assert(near(multiplier(none, rule), 1.0));
        assert(near(damage(100.0, none, rule), 100.0));
    }
}

void twoMoreBeatTwoIncreased() {
    // The whole point of the demo: the same two +50 % lines.
    const std::vector<Mod> more{{50, kMore}, {50, kMore}};
    const std::vector<Mod> inc{{50, kInc}, {50, kInc}};
    assert(near(moreMultiplier(more), 2.25));  // 1.5 * 1.5
    assert(near(damage(100.0, more, Rule::Buckets), 225.0));
    assert(near(increasedMultiplier(inc), 2.0));  // 1 + 0.5 + 0.5
    assert(near(damage(100.0, inc, Rule::Buckets), 200.0));
    // A pool does not touch the other pool.
    assert(near(increasedMultiplier(more), 1.0));
    assert(near(moreMultiplier(inc), 1.0));
}

void mixedBuckets() {
    const std::vector<Mod> mods{{50, kInc}, {30, kInc}, {50, kMore}, {20, kMore}};
    assert(near(increasedMultiplier(mods), 1.8));
    assert(near(moreMultiplier(mods), 1.8));  // 1.5 * 1.2
    assert(near(multiplier(mods, Rule::Buckets), 3.24));
    assert(near(damage(100.0, mods, Rule::Buckets), 324.0));
    assert(near(multiplier(mods, Rule::AddAll), 2.5));         // 1 + 0.5 + 0.3 + 0.5 + 0.2
    assert(near(multiplier(mods, Rule::MultiplyAll), 3.51));   // 1.5 * 1.3 * 1.5 * 1.2
}

void orderDoesNotMatter() {
    std::vector<Mod> mods{{50, kInc}, {30, kMore}, {20, kInc}, {-10, kMore}, {40, kMore}};
    std::vector<Mod> reversed(mods.rbegin(), mods.rend());
    for (const Rule rule : {Rule::Buckets, Rule::AddAll, Rule::MultiplyAll}) {
        assert(near(multiplier(mods, rule), multiplier(reversed, rule)));
    }
}

void naiveRulesIgnoreBuckets() {
    const std::vector<Mod> a{{50, kInc}, {50, kMore}};
    const std::vector<Mod> b{{50, kMore}, {50, kMore}};
    assert(near(multiplier(a, Rule::Buckets), 2.25));  // inc 1.5 * more 1.5
    assert(near(multiplier(b, Rule::Buckets), 2.25));
    assert(near(multiplier(a, Rule::AddAll), 2.0));
    assert(near(multiplier(a, Rule::AddAll), multiplier(b, Rule::AddAll)));
    assert(near(multiplier(a, Rule::MultiplyAll), 2.25));
    assert(near(multiplier(a, Rule::MultiplyAll), multiplier(b, Rule::MultiplyAll)));
    // The buckets matter when the same lines sit in the same pool.
    const std::vector<Mod> c{{50, kInc}, {50, kInc}};
    assert(near(multiplier(c, Rule::Buckets), 2.0));
}

void reducedAndLess() {
    assert(near(increasedMultiplier({{50, kInc}, {-20, kInc}}), 1.3));
    assert(near(moreMultiplier({{-50, kMore}}), 0.5));
    assert(near(moreMultiplier({{-50, kMore}, {100, kMore}}), 1.0));  // 0.5 * 2
}

void nothingGoesBelowZero() {
    assert(near(increasedMultiplier({{-150, kInc}}), 0.0));  // not -0.5
    assert(near(increasedMultiplier({{-100, kInc}}), 0.0));
    // Two negative factors must not turn into a positive product: (-0.5) * (-0.5) = 0.25 would be wrong.
    assert(near(moreMultiplier({{-150, kMore}, {-150, kMore}}), 0.0));
    assert(near(damage(100.0, {{-150, kMore}, {-150, kMore}}, Rule::Buckets), 0.0));
    assert(near(multiplier({{-150, kInc}, {-150, kMore}}, Rule::MultiplyAll), 0.0));
    assert(near(multiplier({{-60, kInc}, {-60, kMore}}, Rule::AddAll), 0.0));  // 1 - 1.2
}

void worthOfIncreasedShrinks() {
    // One +50 % alone is worth its 50 %.
    assert(near(*worth({{50, kInc}}, Rule::Buckets, 0), 0.5));
    // Two in the pool: 2.0 with, 1.5 without, 2.0 / 1.5 - 1 = 1/3.
    assert(near(*worth({{50, kInc}, {50, kInc}}, Rule::Buckets, 1), 1.0 / 3.0));
    // Four in the pool: 3.0 / 2.5 - 1 = 0.2.
    const std::vector<Mod> four{{50, kInc}, {50, kInc}, {50, kInc}, {50, kInc}};
    assert(near(*worth(four, Rule::Buckets, 2), 0.2));
    // More keeps its value however many others there are: 2.25 / 1.5 - 1 = 0.5.
    assert(near(*worth({{50, kMore}, {50, kMore}}, Rule::Buckets, 0), 0.5));
    // n other +50 % in the pool: 0.5 / (1 + 0.5 n), strictly falling, More stays at 0.5.
    double last = 1.0;
    for (int n = 0; n <= 10; ++n) {
        std::vector<Mod> inc(static_cast<std::size_t>(n) + 1, Mod{50, kInc});
        std::vector<Mod> more(static_cast<std::size_t>(n) + 1, Mod{50, kMore});
        const double w = *worth(inc, Rule::Buckets, 0);
        assert(near(w, 0.5 / (1.0 + 0.5 * n)));
        assert(w < last);
        last = w;
        assert(near(*worth(more, Rule::Buckets, 0), 0.5));
    }
    assert(near(*worth({{50, kInc}, {50, kInc}}, Rule::Buckets, 0), *worth({{50, kInc}, {50, kInc}}, Rule::Buckets, 1)));
}

void worthByRule() {
    const std::vector<Mod> mods{{50, kInc}, {50, kMore}, {30, kMore}};
    // MultiplyAll: every modifier is worth exactly its percent.
    assert(near(*worth(mods, Rule::MultiplyAll, 0), 0.5));
    assert(near(*worth(mods, Rule::MultiplyAll, 2), 0.3));
    // AddAll: 1 + 1.3 = 2.3 with, 1.8 without the first: 2.3 / 1.8 - 1.
    assert(near(*worth(mods, Rule::AddAll, 0), 2.3 / 1.8 - 1.0));
    // Buckets: the lone Increased line is worth its 50 %, the More lines their own.
    assert(near(*worth(mods, Rule::Buckets, 0), 0.5));
    assert(near(*worth(mods, Rule::Buckets, 1), 0.5));
}

void worthEdgeCases() {
    // Negative modifier: 1.3 with, 1.5 without.
    assert(near(*worth({{50, kInc}, {-20, kInc}}, Rule::Buckets, 1), 1.3 / 1.5 - 1.0));
    // A modifier that zeroes everything costs everything.
    assert(near(*worth({{-100, kInc}}, Rule::Buckets, 0), -1.0));
    // Without it the damage would be zero: the ratio does not exist.
    assert(!worth({{100, kInc}, {-90, kInc}, {-90, kInc}}, Rule::Buckets, 0).has_value());
    // Out of range.
    assert(!worth({{50, kInc}}, Rule::Buckets, 1).has_value());
    assert(!worth({}, Rule::Buckets, 0).has_value());
}

void headlineStart() {
    const std::vector<Mod> start = defaultMods();
    assert(start.size() == 6);
    assert(near(sumIncreasedPercent(start), 150.0));
    assert(near(increasedMultiplier(start), 2.5));
    assert(near(moreMultiplier(start), 3.375));
    assert(near(damage(100.0, start, Rule::Buckets), 843.75));
    // Each Increased line is worth 25 % (2.5 / 2.0 - 1), each More line 50 %.
    assert(near(*worth(start, Rule::Buckets, 0), 0.25));
    assert(near(*worth(start, Rule::Buckets, 3), 0.5));
    assert(other(kInc) == kMore && other(kMore) == kInc);
}

void sumOfIncreased() {
    assert(near(sumIncreasedPercent({{50, kInc}, {50, kMore}, {-10, kInc}}), 40.0));
    assert(near(sumIncreasedPercent({}), 0.0));
}

void percentSteps() {
    assert(near(stepPercent(50.0, 1), 55.0));
    assert(near(stepPercent(50.0, -3), 35.0));
    assert(near(stepPercent(50.0, 0), 50.0));
    assert(near(stepPercent(195.0, 3), kMaxPercent));
    assert(near(stepPercent(-45.0, -5), kMinPercent));
    assert(near(stepPercent(0.0, 1000000), kMaxPercent));
    assert(near(stepPercent(0.0, -1000000), kMinPercent));
}

void smoothing() {
    assert(near(smooth(3.0, 10.0, 0.0, 12.0), 3.0));
    assert(near(smooth(3.0, 10.0, -1.0, 12.0), 3.0));  // time never runs backwards
    assert(near(smooth(0.0, 1.0, 0.1, 10.0), 1.0 - std::exp(-1.0)));
    assert(near(smooth(0.0, 1.0, 100.0, 10.0), 1.0));
    // Framerate independent: two half steps land where one full step does.
    const double half = smooth(smooth(0.0, 1.0, 0.05, 10.0), 1.0, 0.05, 10.0);
    assert(std::fabs(half - smooth(0.0, 1.0, 0.1, 10.0)) < 1e-12);
    // Close enough snaps, so the animation really ends.
    assert(smooth(0.99999, 1.0, 0.001, 1.0) == 1.0);
    // Works downwards too.
    assert(smooth(10.0, 0.0, 0.1, 10.0) < 10.0);
}

void fixedFormatting() {
    assert(formatFixed(0.0, 0) == "0");
    assert(formatFixed(999.4, 0) == "999");
    assert(formatFixed(2.5, 0) == "3");
    assert(formatFixed(843.75, 0) == "844");
    assert(formatFixed(1000.0, 0) == "1,000");
    assert(formatFixed(1234567.891, 2) == "1,234,567.89");
    assert(formatFixed(2.25, 2) == "2.25");
    assert(formatFixed(3.375, 2) == "3.38");
    assert(formatFixed(0.999, 2) == "1.00");
    assert(formatFixed(5.0, 2) == "5.00");
    assert(formatFixed(0.05, 2) == "0.05");
    assert(formatFixed(-1.5, 1) == "-1.5");
    assert(formatFixed(-0.001, 2) == "0.00");  // no minus sign in front of a zero
    assert(formatFixed(-1234.0, 0) == "-1,234");
    assert(formatFixed(1e13, 0) == "?");
    assert(formatFixed(std::nan(""), 0) == "?");
    assert(formatFixed(HUGE_VAL, 2) == "?");
}

void percentFormatting() {
    assert(formatPercent(50.0, 0) == "+50%");
    assert(formatPercent(-20.0, 0) == "-20%");
    assert(formatPercent(0.0, 0) == "0%");
    assert(formatPercent(0.01, 0) == "0%");  // rounds to zero, so no sign
    assert(formatPercent(27.777, 1) == "+27.8%");
    assert(formatPercent(-13.333, 1) == "-13.3%");
    assert(formatPercent(1234.0, 0) == "+1,234%");
    assert(formatPercent(HUGE_VAL, 0) == "?");
}

}  // namespace

int main() {
    emptyListChangesNothing();
    twoMoreBeatTwoIncreased();
    mixedBuckets();
    orderDoesNotMatter();
    naiveRulesIgnoreBuckets();
    reducedAndLess();
    nothingGoesBelowZero();
    worthOfIncreasedShrinks();
    worthByRule();
    worthEdgeCases();
    headlineStart();
    sumOfIncreased();
    percentSteps();
    smoothing();
    fixedFormatting();
    percentFormatting();
    std::puts("damage-stacking logic: all tests passed");
    return 0;
}
