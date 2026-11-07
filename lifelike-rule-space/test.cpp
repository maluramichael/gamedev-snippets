// Tests for the life-like rule space logic. Plain asserts on hand-computed cases, no framework, no SDL.
// Cells are written as (x, y) with y pointing down, like the screen. Most cases use a 64x64 torus with the pattern far from
// the seam, so the wrap-around never matters; the seam gets its own case.
#undef NDEBUG
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "logic.hpp"

using namespace lrs;

namespace {

using Cells = std::vector<std::pair<int, int>>;

Rule rule(const char* s) {
    const auto r = parseRule(s);
    assert(r.has_value());
    return *r;
}

Grid gridWith(int w, int h, const Cells& cells) {
    Grid g(w, h);
    for (const auto& [x, y] : cells) g.set(x, y, true);
    return g;
}

Cells cellsOf(const Grid& g) {
    Cells out;
    for (int y = 0; y < g.height(); ++y) {
        for (int x = 0; x < g.width(); ++x) {
            if (g.alive(x, y)) out.emplace_back(x, y);
        }
    }
    return out;  // row-major, so already sorted by y then x
}

Cells sortedByY(Cells c) {
    std::sort(c.begin(), c.end(), [](const auto& a, const auto& b) { return a.second != b.second ? a.second < b.second : a.first < b.first; });
    return c;
}

Grid advance(Grid g, Rule r, int generations) {
    Grid next(g.width(), g.height());
    for (int i = 0; i < generations; ++i) {
        step(g, next, r);
        std::swap(g, next);
    }
    return g;
}

Grid inverted(const Grid& g) {
    Grid out(g.width(), g.height());
    for (int y = 0; y < g.height(); ++y) {
        for (int x = 0; x < g.width(); ++x) out.set(x, y, !g.alive(x, y));
    }
    return out;
}

bool sameCells(const Grid& a, const Grid& b) { return a.width() == b.width() && a.height() == b.height() && a.cells() == b.cells(); }

void parsing() {
    assert((rule("B3/S23") == Rule{1U << 3, (1U << 2) | (1U << 3)}));
    assert((rule("B36/S23") == Rule{(1U << 3) | (1U << 6), (1U << 2) | (1U << 3)}));
    assert((rule("B2/S") == Rule{1U << 2, 0}));
    assert((rule("b3/s23") == rule("B3/S23")));  // lower case is fine
    assert((rule("B/S") == Rule{0, 0}));         // a rule where nothing is ever born and nothing survives
    assert((rule("B3678/S34678") == Rule{(1U << 3) | (1U << 6) | (1U << 7) | (1U << 8), (1U << 3) | (1U << 4) | (1U << 6) | (1U << 7) | (1U << 8)}));
    assert((rule("B012345678/S012345678") == Rule{0x1FF, 0x1FF}));
    assert((rule("B63/S32") == rule("B36/S23")));    // digit order does not matter
    assert((rule("B33/S2233") == rule("B3/S23")));   // repeated digits collapse
    assert((rule("B0/S8") == Rule{1U << 0, 1U << 8}));

    for (const char* bad : {"", "B3", "B3/", "S23/B3", "B3S23", "B3/S23x", " B3/S23", "B3/S23 ", "B9/S23", "B3/S9", "B3/S2a", "23/3",
                            "B3/S23/", "B-3/S2", "B3//S23", "/", "B3/B3", "BS", "B3/Sx"}) {
        assert(!parseRule(bad).has_value());
    }
}

void formatting() {
    assert(formatRule(rule("B63/S32")) == "B36/S23");
    assert(formatRule(Rule{}) == "B/S");
    assert(formatRule(rule("B2/S")) == "B2/S");
    assert(formatRule(Rule{0x1FF, 0x1FF}) == "B012345678/S012345678");
    assert(formatRule(rule("b3678/s34678")) == "B3678/S34678");
}

void everyRuleRoundTrips() {
    assert(kRuleCount == 262144);
    assert(kRuleCount == (1U << 18));  // 9 birth bits and 9 survive bits
    for (std::uint32_t i = 0; i < kRuleCount; ++i) {
        const Rule r = ruleFromIndex(i);
        assert(ruleIndex(r) == i);
        const auto back = parseRule(formatRule(r));
        assert(back.has_value() && *back == r);
    }
    // The index is birth in the low nine bits, survive above: B0/S is 1, B/S0 is 512, Life is 8 + 12 * 512.
    assert(ruleIndex(rule("B0/S")) == 1);
    assert(ruleIndex(rule("B/S0")) == 512);
    assert(ruleIndex(rule("B3/S23")) == 8 + 12 * 512);
    assert(ruleIndex(Rule{0x1FF, 0x1FF}) == kRuleCount - 1);
}

void grouping() {
    assert(groupDigits(0) == "0");
    assert(groupDigits(7) == "7");
    assert(groupDigits(999) == "999");
    assert(groupDigits(1000) == "1,000");
    assert(groupDigits(262144) == "262,144");
    assert(groupDigits(1234567) == "1,234,567");
}

void nextStateTable() {
    const Rule life = rule("B3/S23");
    assert(nextState(life, false, 3));
    assert(!nextState(life, false, 2));
    assert(!nextState(life, false, 4));
    assert(!nextState(life, false, 6));
    assert(!nextState(life, true, 1));
    assert(nextState(life, true, 2));
    assert(nextState(life, true, 3));
    assert(!nextState(life, true, 4));
    // HighLife differs from Life in exactly one place: a dead cell with six neighbors is born.
    const Rule highLife = rule("B36/S23");
    assert(nextState(highLife, false, 6));
    for (int n = 0; n <= 8; ++n) {
        if (n == 6) continue;
        assert(nextState(highLife, false, n) == nextState(life, false, n));
    }
    for (int n = 0; n <= 8; ++n) assert(nextState(highLife, true, n) == nextState(life, true, n));
    // Seeds: nothing survives.
    const Rule seeds = rule("B2/S");
    for (int n = 0; n <= 8; ++n) assert(!nextState(seeds, true, n));
    assert(nextState(seeds, false, 2));
    assert(!nextState(seeds, false, 3));
}

void lifeBlinker() {
    const Cells horizontal{{5, 5}, {6, 5}, {7, 5}};
    const Cells vertical{{6, 4}, {6, 5}, {6, 6}};
    const Grid g = gridWith(64, 64, horizontal);
    assert(cellsOf(advance(g, rule("B3/S23"), 1)) == sortedByY(vertical));
    assert(cellsOf(advance(g, rule("B3/S23"), 2)) == sortedByY(horizontal));
    assert(sameCells(advance(g, rule("B3/S23"), 40), g));
    // The same three cells in Seeds do something else entirely: everything dies, the cells above and below the middle appear.
    assert(!sameCells(advance(g, rule("B2/S"), 2), g));
}

void lifeBlockAndGlider() {
    const Grid block = gridWith(64, 64, {{10, 10}, {11, 10}, {10, 11}, {11, 11}});
    assert(sameCells(advance(block, rule("B3/S23"), 1), block));
    const Cells glider{{11, 10}, {12, 11}, {10, 12}, {11, 12}, {12, 12}};
    const Cells moved{{12, 11}, {13, 12}, {11, 13}, {12, 13}, {13, 13}};  // one cell right and one down after four generations
    assert(cellsOf(advance(gridWith(64, 64, glider), rule("B3/S23"), 4)) == sortedByY(moved));
}

void wrapAroundTheSeam() {
    // A blinker across the left and right edge of a 64 wide torus: (63,5), (0,5), (1,5) turns into a column at x = 0.
    const Grid g = gridWith(64, 64, {{63, 5}, {0, 5}, {1, 5}});
    assert(cellsOf(advance(g, rule("B3/S23"), 1)) == sortedByY(Cells{{0, 4}, {0, 5}, {0, 6}}));
    // The same across the top and bottom edge.
    const Grid v = gridWith(64, 64, {{5, 63}, {5, 0}, {5, 1}});
    assert(cellsOf(advance(v, rule("B3/S23"), 1)) == sortedByY(Cells{{4, 0}, {5, 0}, {6, 0}}));
}

void seedsPair() {
    // Two neighbors side by side: both die, and the four cells above and below that touch both of them are born.
    const Grid g = gridWith(64, 64, {{10, 10}, {11, 10}});
    assert(cellsOf(advance(g, rule("B2/S"), 1)) == sortedByY(Cells{{10, 9}, {11, 9}, {10, 11}, {11, 11}}));
}

void birthWithoutSurvival() {
    // A row of three in B3/S: the cell above and below the middle one sees all three cells, the outer cells are not enough.
    const Grid line = gridWith(64, 64, {{10, 10}, {11, 10}, {12, 10}});
    assert(cellsOf(advance(line, rule("B3/S"), 1)) == sortedByY(Cells{{11, 9}, {11, 11}}));
    // With B4 instead of B3 nothing is born and the row just dies.
    assert(advance(line, rule("B4/S"), 1).population() == 0);
}

void singleCellUnderB1() {
    const Grid dot = gridWith(64, 64, {{20, 20}});
    const Grid ring = advance(dot, rule("B1/S"), 1);
    assert(ring.population() == 8);
    assert(!ring.alive(20, 20));
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) assert(ring.alive(20 + dx, 20 + dy) == (dx != 0 || dy != 0));
    }
}

void boundsOfPatterns() {
    assert(!bounds(Grid(8, 8)).has_value());
    const auto b = bounds(gridWith(16, 16, {{3, 4}, {9, 4}, {5, 11}}));
    assert(b.has_value());
    assert(b->minX == 3 && b->maxX == 9 && b->minY == 4 && b->maxY == 11);
}

Grid randomPatch(Rng& rng, int w, int h, int x0, int y0, int size, double density) {
    Grid g(w, h);
    for (int y = 0; y < size; ++y) {
        for (int x = 0; x < size; ++x) g.set(x0 + x, y0 + y, rng.chance(density));
    }
    g.set(x0 + size / 2, y0 + size / 2, true);  // never empty
    return g;
}

void b1SpreadsInEveryDirection() {
    // The leftmost cell of the top row has a dead neighbor diagonally above-left that sees only that one cell, so with B1 it is
    // born. The same holds at every side. With B1 and no B0 the bounding box therefore grows by exactly one cell per side and
    // generation, whatever the other digits say.
    Rng rng(7);
    for (std::uint16_t s = 0; s < 512; ++s) {
        Rule r{static_cast<std::uint16_t>(1U << 1), s};
        r.birth |= static_cast<std::uint16_t>(rng.next() & 0x1FC);  // random B2..B8
        assert((r.birth & 1U) == 0);
        Grid g = randomPatch(rng, 64, 64, 29, 29, 6, 0.5);
        Grid next(64, 64);
        for (int gen = 0; gen < 20; ++gen) {
            const auto before = bounds(g);
            step(g, next, r);
            std::swap(g, next);
            const auto after = bounds(g);
            assert(before.has_value() && after.has_value());
            assert(after->minX == before->minX - 1 && after->maxX == before->maxX + 1);
            assert(after->minY == before->minY - 1 && after->maxY == before->maxY + 1);
        }
    }
}

void noSmallBirthDigitsMeansNoGrowth() {
    // A dead cell next to the outside of a bounding box touches at most three cells of the box (the cell straight across an edge,
    // and its two diagonal neighbors). So without B0 to B3 nothing can be born outside the box: the pattern never leaves it.
    Rng rng(11);
    for (std::uint16_t s = 0; s < 512; ++s) {
        Rule r{static_cast<std::uint16_t>(rng.next() & 0x1F0), s};  // B4..B8 only
        assert((r.birth & 0xFU) == 0);
        Grid g = randomPatch(rng, 64, 64, 29, 29, 8, 0.8);
        const auto start = bounds(g);
        assert(start.has_value());
        Grid next(64, 64);
        for (int gen = 0; gen < 30; ++gen) {
            step(g, next, r);
            std::swap(g, next);
            const auto now = bounds(g);
            if (!now) break;
            assert(now->minX >= start->minX && now->maxX <= start->maxX);
            assert(now->minY >= start->minY && now->maxY <= start->maxY);
        }
    }
    // Control: Life has B3, and a glider does leave its first bounding box.
    Grid glider = gridWith(64, 64, {{11, 10}, {12, 11}, {10, 12}, {11, 12}, {12, 12}});
    const auto first = bounds(glider);
    glider = advance(glider, rule("B3/S23"), 8);
    const auto later = bounds(glider);
    assert(first && later && later->minX > first->minX && later->maxX > first->maxX);
}

void growthNotes() {
    assert(growthNote(rule("B3/S23")) == GrowthNote::None);
    assert(growthNote(rule("B2/S")) == GrowthNote::None);
    assert(growthNote(rule("B36/S23")) == GrowthNote::None);
    assert(growthNote(rule("B1/S")) == GrowthNote::Spreads);
    assert(growthNote(rule("B13/S12")) == GrowthNote::Spreads);
    assert(growthNote(rule("B0/S")) == GrowthNote::Blinks);
    assert(growthNote(rule("B01/S")) == GrowthNote::Blinks);  // B0 wins: the background flips before anything spreads
    assert(growthNote(rule("B4/S")) == GrowthNote::Bounded);
    assert(growthNote(rule("B45678/S5678")) == GrowthNote::Bounded);
    assert(growthNote(rule("B/S")) == GrowthNote::Bounded);
}

void dayAndNightIsSelfComplementary() {
    // B3678/S34678: a dead cell with n live neighbors is born exactly when a live cell with 8-n live neighbors would die.
    assert(selfComplementary(rule("B3678/S34678")));
    assert(!selfComplementary(rule("B3/S23")));
    assert(!selfComplementary(rule("B2/S")));
    int count = 0;
    for (std::uint32_t i = 0; i < kRuleCount; ++i) {
        if (selfComplementary(ruleFromIndex(i))) ++count;
    }
    assert(count == 512);  // the birth set is free (2^9), the survive set follows from it

    Rng rng(3);
    Grid g(40, 40);
    fillSoup(g, 0.4, rng);
    Grid a(40, 40);
    Grid b(40, 40);
    // Swapping living and dead before or after a step gives the same picture, for Day & Night but not for Life.
    const Rule dn = rule("B3678/S34678");
    step(g, a, dn);
    step(inverted(g), b, dn);
    assert(sameCells(inverted(a), b));
    const Rule life = rule("B3/S23");
    step(g, a, life);
    step(inverted(g), b, life);
    assert(!sameCells(inverted(a), b));
    // And every one of the 512 self-complementary rules behaves like that.
    int checked = 0;
    for (std::uint32_t i = 0; i < kRuleCount; ++i) {
        const Rule r = ruleFromIndex(i);
        if (!selfComplementary(r)) continue;
        step(g, a, r);
        step(inverted(g), b, r);
        assert(sameCells(inverted(a), b));
        ++checked;
    }
    assert(checked == 512);
}

Cells shifted(const Cells& c, int dx, int dy) {
    Cells out;
    for (const auto& [x, y] : c) out.emplace_back(x + dx, y + dy);
    return out;
}

void highLifeReplicator() {
    // The 12-cell ring below, run in B36/S23 on a torus big enough that it never meets itself: after 12 generations there are
    // exactly two copies of it, shifted by (-2, -2) and (2, 2), so four cells apart on the diagonal and none left where it started. I found it by brute force over all 5x5 patterns:
    // it is the only one there whose generation 12 is two disjoint copies of itself.
    const Cells ring{{2, 0}, {3, 0}, {4, 0}, {1, 1}, {4, 1}, {0, 2}, {4, 2}, {0, 3}, {3, 3}, {0, 4}, {1, 4}, {2, 4}};
    assert(ring.size() == 12);
    const Rule highLife = rule("B36/S23");
    Grid g = gridWith(96, 96, shifted(ring, 40, 40));
    Cells both = shifted(ring, 38, 38);
    for (const auto& c : shifted(ring, 42, 42)) both.push_back(c);
    const Grid twoCopies = gridWith(96, 96, both);
    assert(twoCopies.population() == 24);  // the copies do not overlap

    Grid next(96, 96);
    std::vector<int> populations;
    for (int gen = 1; gen <= 48; ++gen) {
        step(g, next, highLife);
        std::swap(g, next);
        if (gen == 12) assert(sameCells(g, twoCopies));
        if (gen % 12 == 0) populations.push_back(g.population());
    }
    // Copies at 12, 24, 36 and 48: 2, 2, 4, 2. Where two copies grow into each other they cancel, which is Rule 90 on the
    // row of copies (one copy: 1, 2, 2, 4, 2 along the rows).
    assert((populations == std::vector<int>{24, 24, 48, 24}));

    // In Life the same cells do something else: 32 cells after 12 generations, not two copies.
    const Grid life = advance(gridWith(96, 96, shifted(ring, 40, 40)), rule("B3/S23"), 12);
    assert(life.population() == 32);
    assert(!sameCells(life, twoCopies));
}

void presets() {
    assert(kPresets.size() == 5);
    assert(formatRule(kPresets[0].rule) == "B3/S23");
    assert(formatRule(kPresets[1].rule) == "B36/S23");
    assert(formatRule(kPresets[2].rule) == "B2/S");
    assert(formatRule(kPresets[3].rule) == "B3678/S34678");
    assert(formatRule(kPresets[4].rule) == "B35678/S5678");
    for (const Preset& p : kPresets) assert(p.soup > 0.0 && p.soup < 1.0);
    assert(kPresets[3].soup == 0.5);  // Day & Night
    assert(std::string(presetName(rule("B3/S23"))) == "Life");
    assert(std::string(presetName(rule("B36/S23"))) == "HighLife");
    assert(std::string(presetName(rule("B3678/S34678"))) == "Day & Night");
    assert(std::string(presetName(rule("B3/S234"))).empty());  // not a preset
}

void soupAndHash() {
    Rng rng(1);
    Grid g(100, 100);
    fillSoup(g, 0.0, rng);
    assert(g.population() == 0);
    fillSoup(g, 1.0, rng);
    assert(g.population() == 10000);
    fillSoup(g, 0.3, rng);
    assert(g.population() > 2700 && g.population() < 3300);

    Grid a = gridWith(16, 16, {{1, 1}, {2, 3}});
    Grid b = gridWith(16, 16, {{1, 1}, {2, 3}});
    assert(hashOf(a) == hashOf(b));
    b.set(5, 5, true);
    assert(hashOf(a) != hashOf(b));
    // Same cells on a different size must not collide trivially.
    assert(hashOf(gridWith(8, 4, {{1, 1}})) != hashOf(gridWith(4, 8, {{1, 1}})));
}

void rngIsDeterministic() {
    Rng a(42);
    Rng b(42);
    for (int i = 0; i < 100; ++i) assert(a.next() == b.next());
    Rng c(43);
    Rng d(42);
    assert(c.next() != d.next());
    Rng e(5);
    for (int i = 0; i < 1000; ++i) {
        assert(!e.chance(0.0));
        assert(e.chance(1.0));
    }
}

void verdicts() {
    const Rule life = rule("B3/S23");
    {
        Tracker t;
        Grid g(32, 32);
        t.record(g);
        assert(t.verdict().kind == Kind::Dies);
    }
    {
        Tracker t;
        Grid g = gridWith(32, 32, {{4, 4}, {5, 4}, {4, 5}, {5, 5}});
        t.record(g);
        assert(t.verdict().kind != Kind::Still);  // a single sample proves nothing yet
        g = advance(g, life, 1);
        t.record(g);
        assert(t.verdict().kind == Kind::Still);
        assert(t.verdict().period == 1);
    }
    {
        Tracker t;
        Grid g = gridWith(32, 32, {{5, 5}, {6, 5}, {7, 5}});
        t.record(g);
        for (int i = 0; i < 3; ++i) {
            g = advance(g, life, 1);
            t.record(g);
        }
        assert(t.verdict().kind == Kind::Periodic);
        assert(t.verdict().period == 2);
    }
    {
        // A glider on a 16x16 torus is back after 4 * 16 = 64 generations, the longest repeat the tracker looks for.
        Tracker t;
        Grid g = gridWith(16, 16, {{1, 0}, {2, 1}, {0, 2}, {1, 2}, {2, 2}});
        t.record(g);
        for (int i = 0; i < 63; ++i) {
            g = advance(g, life, 1);
            t.record(g);
            assert(t.verdict().kind != Kind::Periodic);
        }
        g = advance(g, life, 1);
        t.record(g);
        assert(t.verdict().kind == Kind::Periodic && t.verdict().period == 64);
    }
    {
        // On 17x17 it takes 68 generations: out of range, so the tracker says Active instead of guessing.
        Tracker t;
        Grid g = gridWith(17, 17, {{1, 0}, {2, 1}, {0, 2}, {1, 2}, {2, 2}});
        t.record(g);
        for (int i = 0; i < 70; ++i) {
            g = advance(g, life, 1);
            t.record(g);
        }
        assert(t.verdict().kind == Kind::Active);
    }
    {
        // B1/S12345678 from one cell: cells never die once they have a neighbor, the pattern grows every generation.
        Tracker t;
        const Rule fill = rule("B1/S12345678");
        Grid g = gridWith(200, 200, {{100, 100}});
        t.record(g);
        Grid next(200, 200);
        for (int i = 0; i < 60; ++i) {
            step(g, next, fill);
            std::swap(g, next);
            t.record(g);
        }
        assert(t.verdict().kind == Kind::Growing);
    }
    {
        // Doubling from a handful of cells is no explosion: 8 cells for 40 generations, then 16, stays Active.
        Tracker t;
        const Grid small = gridWith(64, 64, {{1, 1}, {3, 1}, {5, 1}, {7, 1}, {9, 1}, {11, 1}, {13, 1}, {15, 1}});
        Grid doubled = small;
        for (int x = 17; x < 33; x += 2) doubled.set(x, 1, true);
        assert(small.population() == 8 && doubled.population() == 16);
        for (int i = 0; i < 41; ++i) t.record(small);
        t.record(doubled);
        assert(t.verdict().kind == Kind::Active);
        // The same doubling from 40 to 80 cells is.
        Tracker u;
        Grid forty(128, 4);
        for (int x = 0; x < 80; x += 2) forty.set(x, 1, true);
        Grid eighty = forty;
        for (int x = 80; x < 160; x += 2) eighty.set(x % 128, 3, true);
        assert(forty.population() == 40 && eighty.population() == 80);
        for (int i = 0; i < 41; ++i) u.record(forty);
        u.record(eighty);
        assert(u.verdict().kind == Kind::Growing);
    }
    {
        // A Life soup after ten generations is neither dead nor repeating nor growing.
        Rng rng(2);
        Tracker t;
        Grid g(64, 64);
        fillSoup(g, 0.3, rng);
        t.record(g);
        Grid next(64, 64);
        for (int i = 0; i < 10; ++i) {
            step(g, next, life);
            std::swap(g, next);
            t.record(g);
        }
        assert(t.verdict().kind == Kind::Active);
    }
    {
        // reset() forgets everything, and the history never grows past its cap.
        Tracker t;
        Grid g = gridWith(8, 8, {{1, 1}});
        for (int i = 0; i < 1000; ++i) t.record(g);
        assert(t.populations().size() == Tracker::kHistory);
        t.reset();
        assert(t.populations().empty());
        assert(t.verdict().kind == Kind::Active);
    }
}

void lines() {
    auto line = lineCells({0, 0}, {4, 0});
    assert((line == std::vector<Cell>{{0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}}));
    assert((lineCells({2, 2}, {2, 2}) == std::vector<Cell>{{2, 2}}));
    assert((lineCells({3, 0}, {0, 0}) == std::vector<Cell>{{3, 0}, {2, 0}, {1, 0}, {0, 0}}));
    assert((lineCells({0, 0}, {0, 3}) == std::vector<Cell>{{0, 0}, {0, 1}, {0, 2}, {0, 3}}));
    assert((lineCells({0, 0}, {3, 3}) == std::vector<Cell>{{0, 0}, {1, 1}, {2, 2}, {3, 3}}));
    // Any line has max(|dx|, |dy|) + 1 cells, starts and ends where asked, and never skips a cell.
    for (int dx = -7; dx <= 7; ++dx) {
        for (int dy = -7; dy <= 7; ++dy) {
            const auto cells = lineCells({10, 10}, {10 + dx, 10 + dy});
            assert(cells.size() == static_cast<std::size_t>(std::max(std::abs(dx), std::abs(dy))) + 1);
            assert((cells.front() == Cell{10, 10}));
            assert((cells.back() == Cell{10 + dx, 10 + dy}));
            for (std::size_t i = 1; i < cells.size(); ++i) {
                assert(std::abs(cells[i].x - cells[i - 1].x) <= 1 && std::abs(cells[i].y - cells[i - 1].y) <= 1);
                assert(cells[i] != cells[i - 1]);
            }
        }
    }
}

void gridEdges() {
    Grid g(8, 4);
    assert(g.width() == 8 && g.height() == 4);
    g.set(-1, -1, true);  // wraps to (7, 3)
    assert(g.alive(7, 3));
    assert(g.alive(-1, -1));
    assert(g.alive(15, 7));
    assert(g.population() == 1);
    g.clear();
    assert(g.population() == 0);
    // A grid so small that a cell is its own neighbor on several sides still gives the answer of the torus rule: on a 1x1 torus
    // the single cell sees itself eight times, so with S8 it survives and without it dies.
    Grid one(1, 1);
    one.set(0, 0, true);
    Grid out(1, 1);
    step(one, out, rule("B/S8"));
    assert(out.alive(0, 0));
    step(one, out, rule("B/S"));
    assert(!out.alive(0, 0));
}

void pentomino() {
    // The R-pentomino has five cells, written out here once so the demo and the test agree.
    const auto cells = rPentomino();
    assert(cells.size() == 5);
    const std::set<std::pair<int, int>> expect{{1, 0}, {2, 0}, {0, 1}, {1, 1}, {1, 2}};
    const std::set<std::pair<int, int>> got(cells.begin(), cells.end());
    assert(got == expect);
}

}  // namespace

int main() {
    parsing();
    formatting();
    everyRuleRoundTrips();
    grouping();
    nextStateTable();
    lifeBlinker();
    lifeBlockAndGlider();
    wrapAroundTheSeam();
    seedsPair();
    birthWithoutSurvival();
    singleCellUnderB1();
    boundsOfPatterns();
    b1SpreadsInEveryDirection();
    noSmallBirthDigitsMeansNoGrowth();
    growthNotes();
    dayAndNightIsSelfComplementary();
    highLifeReplicator();
    presets();
    soupAndHash();
    rngIsDeterministic();
    verdicts();
    lines();
    gridEdges();
    pentomino();
    std::puts("lifelike-rule-space logic: all asserts passed");
    return 0;
}
