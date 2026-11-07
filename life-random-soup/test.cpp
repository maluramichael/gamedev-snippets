// Tests for the random soup demo. Plain asserts on hand-computed cases, no framework, no SDL.
// Numbers marked "(reference)" come from an independent Python implementation (set of cells, Counter for the neighbor counts,
// the settle rule written straight from its definition instead of with run counters, same SplitMix64 for the soups), so they
// are a second opinion and not just this code agreeing with itself.
// Run it with --stats to print the lifespan table of the first 2000 soups instead (see README).
#undef NDEBUG
#include <algorithm>
#include <array>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "logic.hpp"

using namespace life;

namespace {

std::vector<Cell> sorted(std::vector<Cell> cells) {
    std::sort(cells.begin(), cells.end());
    return cells;
}

Life stepped(std::vector<Cell> cells, int generations) {
    Life life(std::move(cells));
    for (int i = 0; i < generations; ++i) life.step();
    return life;
}

std::vector<Cell> shifted(const std::vector<Cell>& cells, int dx, int dy) {
    std::vector<Cell> out;
    for (const Cell& c : cells) out.push_back(Cell{c.x + dx, c.y + dy});
    return out;
}

void rng() {
    // SplitMix64 with seed 0 starts with this value (the published reference output).
    Rng zero(0);
    assert(zero.next() == 0xE220A8397B1DCDAFULL);
    Rng a(42);
    Rng b(42);
    for (int i = 0; i < 100; ++i) assert(a.next() == b.next());
    Rng c(7);
    for (int i = 0; i < 1000; ++i) {
        assert(c.below(1) == 0);
        assert(c.below(100) < 100);
    }
}

void soupGeneration() {
    // Density 0 is empty, 100 is full, anything between lies in the square and is reproducible.
    assert(randomSoup(1, 16, 0).empty());
    assert(randomSoup(1, 16, 100).size() == 256);
    assert(randomSoup(1, 0, 50).empty());
    assert(randomSoup(1, 16, -5).empty());       // clamped, no wrap-around
    assert(randomSoup(1, 16, 250).size() == 256);
    assert(randomSoup(5, 16, 50) == randomSoup(5, 16, 50));
    assert(randomSoup(5, 16, 50) != randomSoup(6, 16, 50));
    for (const Cell& c : randomSoup(3, 16, 50)) assert(c.x >= 0 && c.x < 16 && c.y >= 0 && c.y < 16);
    // (reference) population of the first generation, 16x16 at 50 percent.
    assert(randomSoup(1, 16, 50).size() == 135);
    assert(randomSoup(2, 16, 50).size() == 130);
    assert(randomSoup(3, 16, 50).size() == 132);
    assert(randomSoup(5, 16, 50).size() == 126);
}

void smallPatterns() {
    assert(stepped({}, 5).population() == 0);
    assert(stepped({{0, 0}}, 1).population() == 0);                 // loneliness
    assert(stepped({{0, 0}, {1, 0}}, 1).population() == 0);         // two neighbors are still too few
    const std::vector<Cell> block{{0, 0}, {1, 0}, {0, 1}, {1, 1}};
    assert(sorted(stepped(block, 1).cells()) == sorted(block));    // each has exactly 3 neighbors
    assert(sorted(stepped(block, 7).cells()) == sorted(block));

    const std::vector<Cell> blinker{{0, 0}, {1, 0}, {2, 0}};
    const std::vector<Cell> upright{{1, -1}, {1, 0}, {1, 1}};
    assert(sorted(stepped(blinker, 1).cells()) == sorted(upright));  // the two ends have 1 neighbor and die, (1,+-1) have 3 and are born
    assert(sorted(stepped(blinker, 2).cells()) == sorted(blinker));

    // The 3x3 square: corners keep 3 neighbors, edge middles (5) and the center (8) die, the four cells next to the middles
    // of the sides see exactly three squares and are born. 4 + 4 = 8 cells.
    std::vector<Cell> square;
    for (int y = 0; y < 3; ++y) {
        for (int x = 0; x < 3; ++x) square.push_back(Cell{x, y});
    }
    const std::vector<Cell> octagon{{0, 0}, {2, 0}, {0, 2}, {2, 2}, {1, -1}, {-1, 1}, {3, 1}, {1, 3}};
    assert(sorted(stepped(square, 1).cells()) == sorted(octagon));

    // The beacon alternates between 8 and 6 cells: the touching corners die, then come back.
    const std::vector<Cell> beacon{{0, 0}, {1, 0}, {0, 1}, {1, 1}, {2, 2}, {3, 2}, {2, 3}, {3, 3}};
    assert(stepped(beacon, 1).population() == 6);
    assert(stepped(beacon, 2).population() == 8);
    assert(sorted(stepped(beacon, 2).cells()) == sorted(beacon));
}

void glider() {
    const std::vector<Cell> g{{1, 0}, {2, 1}, {0, 2}, {1, 2}, {2, 2}};
    // After 4 generations the glider is the same shape one cell right and one down.
    assert(sorted(stepped(g, 4).cells()) == sorted(shifted(g, 1, 1)));
    assert(sorted(stepped(g, 40).cells()) == sorted(shifted(g, 10, 10)));
    // The rules do not care where the origin is, negative coordinates and a far start included.
    assert(sorted(stepped(shifted(g, -50, -30), 4).cells()) == sorted(shifted(g, -49, -29)));
    assert(sorted(stepped(shifted(g, -1000000, 2000000), 400).cells()) == sorted(shifted(g, -1000000 + 100, 2000000 + 100)));
    // A glider is five cells in every phase.
    Life life(g);
    for (int i = 0; i < 20; ++i) {
        assert(life.population() == 5);
        life.step();
    }
}

void rPentominoByHand() {
    // Generation 1 and 2 by hand: 6 cells, then 7 cells.
    const std::vector<Cell> gen1{{0, 0}, {1, 0}, {2, 0}, {0, 1}, {0, 2}, {1, 2}};
    assert(sorted(stepped(rPentomino(), 1).cells()) == sorted(gen1));
    const std::vector<Cell> gen2{{0, 0}, {1, 0}, {0, 2}, {1, 2}, {2, 1}, {-1, 1}, {1, -1}};
    assert(sorted(stepped(rPentomino(), 2).cells()) == sorted(gen2));
    // (reference) the first populations.
    const int expected[] = {5, 6, 7, 9, 8, 9, 12, 11, 18, 11};
    Life life(rPentomino());
    for (const int pop : expected) {
        assert(life.population() == pop);
        life.step();
    }
}

// Reference implementation: a dense array, so a different data structure and a different way to count neighbors. The field is
// big enough that nothing reaches the edge in the generations used here.
std::vector<Cell> denseRun(const std::vector<Cell>& start, int generations, int field) {
    const int offset = field / 2 - 8;
    std::vector<unsigned char> now(static_cast<std::size_t>(field) * static_cast<std::size_t>(field), 0);
    auto at = [field](std::vector<unsigned char>& v, int x, int y) -> unsigned char& { return v[static_cast<std::size_t>(y) * static_cast<std::size_t>(field) + static_cast<std::size_t>(x)]; };
    for (const Cell& c : start) at(now, c.x + offset, c.y + offset) = 1;
    std::vector<unsigned char> next(now.size(), 0);
    for (int g = 0; g < generations; ++g) {
        for (int y = 1; y < field - 1; ++y) {
            for (int x = 1; x < field - 1; ++x) {
                int n = 0;
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) n += (dx != 0 || dy != 0) ? at(now, x + dx, y + dy) : 0;
                }
                at(next, x, y) = (n == 3 || (n == 2 && at(now, x, y) != 0)) ? 1 : 0;
            }
        }
        now.swap(next);
    }
    std::vector<Cell> out;
    for (int y = 0; y < field; ++y) {
        for (int x = 0; x < field; ++x) {
            if (at(now, x, y) != 0) out.push_back(Cell{x - offset, y - offset});
        }
    }
    return out;
}

void againstDenseReference() {
    for (std::uint64_t seed = 1; seed <= 6; ++seed) {
        const auto soup = randomSoup(seed, 16, 50);
        const Life life = stepped(soup, 120);
        assert(sorted(life.cells()) == sorted(denseRun(soup, 120, 384)));
    }
    // A sparse soup, where cells die out and come back, and a full one that dies in the first generation.
    assert(sorted(stepped(randomSoup(4, 16, 10), 200).cells()) == sorted(denseRun(randomSoup(4, 16, 10), 200, 384)));
    assert(sorted(stepped(randomSoup(9, 16, 100), 30).cells()) == sorted(denseRun(randomSoup(9, 16, 100), 30, 384)));
}

void tracker() {
    // Constant population: the repeat is 120 generations long once generation 120 is in (t = 1..120), starting at generation 0.
    Tracker t;
    for (int g = 0; g < 120; ++g) {
        t.push(5);
        assert(!t.settled());
    }
    t.push(5);
    assert(t.generation() == 120);
    assert(t.settled().has_value());
    assert(t.settled()->period == 1 && t.settled()->since == 0);

    // 5, 6, 7 and then 9 forever: the repeat starts at generation 3, and is noticed 120 generations later.
    Tracker u;
    u.push(5);
    u.push(6);
    u.push(7);
    int pushed = 3;
    while (!u.settled()) {
        u.push(9);
        ++pushed;
        assert(pushed < 1000);
    }
    assert(u.generation() == 3 + 120);
    assert(u.settled()->period == 1 && u.settled()->since == 3);

    // Alternating 8, 6: period 2 from the start (the beacon).
    Tracker v;
    for (int g = 0; g < 200; ++g) v.push(g % 2 == 0 ? 8 : 6);
    assert(v.settled().has_value());
    assert(v.settled()->period == 2 && v.settled()->since == 0);

    // One value that breaks the repeat resets the count.
    Tracker w;
    for (int g = 0; g < 119; ++g) w.push(4);
    w.push(7);
    for (int g = 0; g < 100; ++g) w.push(4);
    assert(!w.settled());
    for (int g = 0; g < 40; ++g) w.push(4);
    assert(w.settled().has_value() && w.settled()->period == 1 && w.settled()->since == 120);

    // A sequence that never repeats (strictly growing) never settles. Period 30 is the longest that is found, 31 is not.
    Tracker grow;
    for (int g = 0; g < 500; ++g) grow.push(g);
    assert(!grow.settled());
    Tracker p30;
    Tracker p31;
    for (int g = 0; g < 600; ++g) {
        p30.push(100 + g % 30);
        p31.push(100 + g % 31);
    }
    assert(p30.settled().has_value() && p30.settled()->period == 30 && p30.settled()->since == 0);
    assert(!p31.settled());

    // A dying soup: 5, 2, 0, 0, ... settles at period 1 and the lifespan is the generation it reached 0.
    Tracker dead;
    dead.push(5);
    dead.push(2);
    for (int g = 0; g < 200; ++g) dead.push(0);
    assert(dead.settled().has_value() && dead.settled()->period == 1 && dead.settled()->since == 2);
}

Run runUntilDone(std::vector<Cell> start) {
    Run run(std::move(start));
    while (!run.done()) run.step();
    return run;
}

void runs() {
    const Run blinker = runUntilDone({{0, 0}, {1, 0}, {2, 0}});
    assert(blinker.settled() && blinker.result()->since == 0 && blinker.result()->period == 1 && blinker.population() == 3);
    assert(blinker.generation() == 120);

    const Run beacon = runUntilDone({{0, 0}, {1, 0}, {0, 1}, {1, 1}, {2, 2}, {3, 2}, {2, 3}, {3, 3}});
    assert(beacon.settled() && beacon.result()->period == 2 && beacon.result()->since == 0);

    const Run lonely = runUntilDone({{4, 4}});
    assert(lonely.settled() && lonely.population() == 0 && lonely.result()->since == 1);

    // Done means done: more steps change nothing.
    Run again = blinker;
    again.step();
    assert(again.generation() == blinker.generation());

    // (reference) R-pentomino: settles at generation 1103 with 116 cells, found 120 generations later. The population is
    // constant from there on: six gliders, blinkers, blocks and so on.
    const Run r = runUntilDone(rPentomino());
    assert(r.settled());
    assert(r.result()->since == 1103 && r.result()->period == 1);
    assert(r.generation() == 1103 + 120);
    assert(r.population() == 116);

    // (reference) acorn: 5206 generations, 633 cells.
    const Run a = runUntilDone(acorn());
    assert(a.settled());
    assert(a.result()->since == 5206 && a.result()->period == 1);
    assert(a.generation() == 5206 + 120);
    assert(a.population() == 633);
}

void gun() {
    // The Gosper glider gun never settles: its population grows by one glider every 30 generations. It runs into the cap.
    const std::vector<Cell> gosper{{1, 5},  {1, 6},  {2, 5},  {2, 6},  {11, 5}, {11, 6}, {11, 7}, {12, 4}, {12, 8}, {13, 3}, {13, 9}, {14, 3}, {14, 9},
                                   {15, 6}, {16, 4}, {16, 8}, {17, 5}, {17, 6}, {17, 7}, {18, 6}, {21, 3}, {21, 4}, {21, 5}, {22, 3}, {22, 4}, {22, 5},
                                   {23, 2}, {23, 6}, {25, 1}, {25, 2}, {25, 6}, {25, 7}, {35, 3}, {35, 4}, {36, 3}, {36, 4}};
    assert(gosper.size() == 36);
    // (reference) 36 cells, 41 after one glider is out (generation 30), 46 after two, 86 at generation 300.
    Life life(gosper);
    int pops[301];
    for (int g = 0; g <= 300; ++g) {
        pops[g] = life.population();
        life.step();
    }
    assert(pops[0] == 36 && pops[30] == 41 && pops[60] == 46 && pops[90] == 51 && pops[300] == 86);
    const Run run = runUntilDone(gosper);
    assert(!run.settled() && run.capped());
    assert(run.generation() == Run::kMaxGeneration);
    assert(!run.result().has_value());
}

struct Expected {
    int since;
    int pop;  // population when the run is declared settled
};

void checkSoups(int density, std::uint64_t firstSeed, const std::vector<Expected>& expected) {
    for (std::size_t i = 0; i < expected.size(); ++i) {
        const Run run = runUntilDone(randomSoup(firstSeed + i, 16, density));
        assert(run.settled());
        assert(run.result()->since == expected[i].since);
        assert(run.result()->period == 1);
        assert(run.population() == expected[i].pop);
    }
}

void soupsAgainstReference() {
    // (reference) 16x16 at 50 percent, seeds 1 to 30: lifespan and final population.
    checkSoups(50, 1, {{165, 15},  {81, 39},   {720, 144}, {1703, 124}, {2592, 622}, {42, 20},  {286, 112}, {892, 102},  {178, 35}, {634, 146},
                       {140, 27},  {1232, 144}, {307, 54},  {122, 17},   {410, 41},   {92, 37},  {546, 171}, {2295, 276}, {389, 54}, {41, 14},
                       {407, 50},  {2200, 224}, {257, 88},  {609, 102},  {1390, 433}, {214, 34}, {581, 151}, {199, 55},   {203, 63}, {252, 35}});
    // (reference) at 10 percent many soups die out (population 0). Seed 4 starts as 31 scattered cells and ends at the same
    // numbers as the R-pentomino: lifespan 1103, 116 cells.
    checkSoups(10, 1, {{4, 6}, {1, 7}, {5, 3}, {1103, 116}, {1, 0}, {6, 0}, {2, 0}, {2, 0}, {4, 0}, {2, 6}, {1, 0}, {6, 0}});
    // (reference) at 70 percent the crowd dies back before anything grows.
    checkSoups(70, 1, {{138, 17}, {31, 25}, {335, 54}, {137, 61}, {1950, 312}, {63, 19}, {81, 39}, {66, 43}});
}

void stats() {
    Stats s;
    assert(s.median() == -1 && s.total() == 0 && s.longest() == 0);
    // The 12 soups at 10 percent, seeds 1 to 12: lifespans 4 1 5 1103 1 6 2 2 4 2 1 6, seven of them dead.
    for (std::uint64_t seed = 1; seed <= 12; ++seed) s.record(runUntilDone(randomSoup(seed, 16, 10)));
    assert(s.total() == 12);
    assert(s.dead() == 7 && s.ash() == 5 && s.capped() == 0);
    assert(s.longest() == 1103);
    // Sorted: 1 1 1 2 2 2 4 4 5 6 6 1103, the lower median is the sixth value.
    assert(s.median() == 2);
    assert(s.atLeast(1000) == 1);
    assert(s.atLeast(1104) == 0);
    assert(s.inRange(0, 2, true) == 2);   // dead with a lifespan of 1: seeds 5 and 11
    assert(s.inRange(0, 2, false) == 1);  // ash with a lifespan of 1: seed 2
    assert(s.inRange(2, 1104, false) == 4);
}

void camera() {
    assert(medianCenter({}) == (Cell{0, 0}));
    assert(medianCenter({{7, -3}}) == (Cell{7, -3}));
    // Two cells: the lower median.
    assert(medianCenter({{4, 10}, {2, 20}}) == (Cell{2, 10}));
    // The R-pentomino: x = 1 2 0 1 1, y = 0 0 1 1 2, both medians are 1.
    assert(medianCenter(rPentomino()) == (Cell{1, 1}));
    // Four cells at the origin and one glider 1000 cells away: the camera stays with the four, the mean would not.
    assert(medianCenter({{0, 0}, {1, 0}, {2, 0}, {3, 0}, {1000, 1000}}) == (Cell{2, 0}));
    assert(medianCenter({{-5, -5}, {-4, -4}, {-3, -3}, {-2, -2}, {900, -900}}) == (Cell{-3, -4}));
}

void rate() {
    double acc = 0.0;
    assert(stepsDue(acc, 10.0, 0.05) == 0);   // half a generation, kept
    assert(stepsDue(acc, 10.0, 0.05) == 1);   // now a whole one
    assert(stepsDue(acc, 0.0, 1.0) == 0);
    assert(stepsDue(acc, 10.0, -1.0) == 0);
    acc = 0.0;
    assert(stepsDue(acc, 100.0, 5.0) == 25);  // a stall counts as 0.25 s
    assert(formatCount(0) == "0" && formatCount(1234) == "1,234" && formatCount(1234567) == "1,234,567" && formatCount(-1200) == "-1,200");
}

/// --stats: lifespan table of the first 2000 soups at 16x16 and 50 percent, binned by powers of two, for the post.
void printStats() {
    for (const int density : {50, 30, 70}) {
        Stats s;
        for (std::uint64_t seed = 1; seed <= 2000; ++seed) s.record(runUntilDone(randomSoup(seed, 16, density)));
        std::printf("density %d%%: %ld soups, dead %ld, ash %ld, no settle by %d: %ld, median %d, longest %d\n", density, s.total(), s.dead(), s.ash(),
                    Run::kMaxGeneration, s.capped(), s.median(), s.longest());
        int lo = 0;
        for (const int hi : {16, 32, 64, 128, 256, 512, 1024, 2048, 4096, Stats::kCap + 1}) {
            std::printf("  [%4d, %4d)  dead %5ld  ash %5ld\n", lo, hi, s.inRange(lo, hi, true), s.inRange(lo, hi, false));
            lo = hi;
        }
        std::printf("  at least 1000 generations: %ld\n", s.atLeast(1000));
        std::printf("  at least 1103 (R-pentomino): %ld, at least 5206 (acorn): %ld\n", s.atLeast(1103), s.atLeast(5206));
    }

    // How often does "settled" turn out wrong? Run the first 300 soups at 50 percent for 2000 more generations after the
    // detector said settled, and count those whose population stops repeating with the same period.
    int checked = 0;
    int changed = 0;
    for (std::uint64_t seed = 1; seed <= 300; ++seed) {
        const Run run = runUntilDone(randomSoup(seed, 16, 50));
        if (!run.settled()) continue;
        ++checked;
        const int period = run.result()->period;
        std::vector<int> pops{run.population()};
        Life life(run.cells());
        bool broke = false;
        for (int g = 1; g <= 2000 && !broke; ++g) {
            life.step();
            pops.push_back(life.population());
            if (g >= period && pops[static_cast<std::size_t>(g)] != pops[static_cast<std::size_t>(g - period)]) broke = true;
        }
        if (broke) ++changed;
    }
    std::printf("settled soups checked: %d, population changed within the next 2000 generations: %d\n", checked, changed);
}

}  // namespace

int main(int argc, char** argv) {
    if (argc > 1 && std::strcmp(argv[1], "--stats") == 0) {
        printStats();
        return 0;
    }
    rng();
    soupGeneration();
    smallPatterns();
    glider();
    rPentominoByHand();
    againstDenseReference();
    tracker();
    runs();
    gun();
    soupsAgainstReference();
    stats();
    camera();
    rate();
    std::puts("life-random-soup: all tests passed");
    return 0;
}
