// Tests for the sandpile logic. Plain asserts on hand-computed cases, no framework, no SDL.
// `test --survey` only prints the numbers the write-up quotes (not part of ctest).
#undef NDEBUG
#include <algorithm>
#include <array>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <utility>
#include <vector>

#include "logic.hpp"

using namespace sp;

namespace {

const std::array<std::pair<int, int>, 8> kOnes{{{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {2, 0}, {-2, 0}, {0, 2}, {0, -2}}};
const std::array<std::pair<int, int>, 4> kTwos{{{1, 1}, {1, -1}, {-1, 1}, {-1, -1}}};

int cell(const Pile& p, int x, int y) { return p.cells[static_cast<std::size_t>(p.idx(x, y))]; }

Pile scatter(int size, int count, std::uint64_t seed) {
    Pile p(size, size);
    Rng rng(seed);
    for (int i = 0; i < count; ++i) {
        add(p, static_cast<int>(rng.below(static_cast<std::uint32_t>(size))), static_cast<int>(rng.below(static_cast<std::uint32_t>(size))), 1);
    }
    return p;
}

bool sameResult(const Pile& a, const Pile& b) { return a.cells == b.cells && a.lost == b.lost && a.topplings == b.topplings; }

void threeGrainsStay() {
    Pile p(5, 5);
    add(p, 2, 2, 3);
    assert(stable(p));
    Rng rng(1);
    const Avalanche a = settle(p, Order::Rows, rng);
    assert(a.topplings == 0 && a.passes == 0);
    assert(cell(p, 2, 2) == 3);
}

void fourGrainsTopple() {
    Pile p(5, 5);
    add(p, 2, 2, 4);
    assert(!stable(p));
    Rng rng(1);
    const Avalanche a = settle(p, Order::Rows, rng);
    assert(a.topplings == 1 && a.passes == 1);
    assert(cell(p, 2, 2) == 0);
    assert(cell(p, 2, 1) == 1 && cell(p, 1, 2) == 1 && cell(p, 3, 2) == 1 && cell(p, 2, 3) == 1);
    assert(grains(p) == 4 && p.lost == 0 && p.total == 4 && p.topplings == 1);
    assert(stable(p));
}

void aCornerLosesTwo() {
    Pile p(5, 5);
    add(p, 0, 0, 4);
    Rng rng(1);
    settle(p, Order::Rows, rng);
    assert(cell(p, 0, 0) == 0 && cell(p, 1, 0) == 1 && cell(p, 0, 1) == 1);
    assert(p.lost == 2);
    assert(grains(p) == 2);
    Pile q(5, 5);
    add(q, 4, 4, 4);  // the opposite corner behaves the same
    settle(q, Order::Reverse, rng);
    assert(q.lost == 2 && cell(q, 3, 4) == 1 && cell(q, 4, 3) == 1);
}

void anEdgeLosesOne() {
    Pile p(5, 5);
    add(p, 2, 0, 4);
    Rng rng(1);
    settle(p, Order::Rows, rng);
    assert(cell(p, 1, 0) == 1 && cell(p, 3, 0) == 1 && cell(p, 2, 1) == 1 && cell(p, 2, 0) == 0);
    assert(p.lost == 1);
    assert(grains(p) == 3);
}

void aLoneCellLosesAllFour() {
    Pile p(1, 1);
    add(p, 0, 0, 4);
    Rng rng(1);
    const Avalanche a = settle(p, Order::Rows, rng);
    assert(a.topplings == 1 && p.lost == 4 && grains(p) == 0);
}

void sixteenGrainsAtTheCenter() {
    // Hand computed on an infinite grid: the middle topples 4 times, its neighbors once each, the middle once more.
    // 0 1 2 1 0 .. final: middle 0, the 4 neighbors 1, the 4 diagonals 2, the 4 cells two steps away on the axes 1.
    for (const Order order : {Order::Rows, Order::Reverse, Order::Random}) {
        Pile p(9, 9);
        add(p, 4, 4, 16);
        Rng rng(5);
        const Avalanche a = settle(p, order, rng);
        assert(a.topplings == 9);
        assert(a.passes == 3);  // pass 1: the middle (4 times), pass 2: its four neighbors, pass 3: the middle once more
        assert(cell(p, 4, 4) == 0);
        for (const auto& [dx, dy] : kOnes) assert(cell(p, 4 + dx, 4 + dy) == 1);
        for (const auto& [dx, dy] : kTwos) assert(cell(p, 4 + dx, 4 + dy) == 2);
        assert(grains(p) == 16 && p.lost == 0);
    }
}

void theOrderChangesTheMiddleNotTheEnd() {
    // Two neighbors, X = (5,5) with 4 grains and Y = (6,5) with 7, on an 11 x 11 grid.
    // Rows: X first. X gives Y a grain (8), then Y topples twice and gives X 2. After one pass: X = 2, Y = 0, stable.
    // Reverse: Y first. Y topples once (3), X gets one (5), X topples once and gives Y one (4). After one pass: X = 1, Y = 4, not stable.
    // The second pass of the reverse order topples Y once more and lands on exactly the same picture.
    auto make = [] {
        Pile p(11, 11);
        add(p, 5, 5, 4);
        add(p, 6, 5, 7);
        return p;
    };
    Rng rng(3);
    Pile rows = make();
    assert(pass(rows, Order::Rows, rng) == 3);
    assert(cell(rows, 5, 5) == 2 && cell(rows, 6, 5) == 0);
    assert(stable(rows));

    Pile rev = make();
    assert(pass(rev, Order::Reverse, rng) == 2);
    assert(cell(rev, 5, 5) == 1 && cell(rev, 6, 5) == 4);
    assert(!stable(rev));
    assert(pass(rev, Order::Reverse, rng) == 1);
    assert(stable(rev));

    assert(rows.cells == rev.cells);
    assert(rows.topplings == 3 && rev.topplings == 3);
    // the shared end picture: X 2, Y 0, west, north, south of X one each, east of Y, north of Y, south of Y two each
    assert(cell(rows, 4, 5) == 1 && cell(rows, 5, 4) == 1 && cell(rows, 5, 6) == 1);
    assert(cell(rows, 7, 5) == 2 && cell(rows, 6, 4) == 2 && cell(rows, 6, 6) == 2);
    assert(grains(rows) == 11);
}

void fourOrdersGiveTheSameGrid() {
    for (const std::uint64_t seed : {1ULL, 2ULL, 3ULL}) {
        Pile base = scatter(15, 400, seed);
        Pile rows = base;
        Pile rev = base;
        Pile r1 = base;
        Pile r2 = base;
        Rng g1(11);
        Rng g2(12345);
        Rng g3(7);
        settle(rows, Order::Rows, g3);
        settle(rev, Order::Reverse, g3);
        settle(r1, Order::Random, g1);
        settle(r2, Order::Random, g2);
        assert(stable(rows) && stable(rev) && stable(r1) && stable(r2));
        assert(sameResult(rows, rev) && sameResult(rows, r1) && sameResult(rows, r2));
        assert(rows.lost > 0);  // the edge really was involved
        assert(grains(rows) + rows.lost == 400);
    }
    Pile heap(15, 15);
    add(heap, 7, 7, 300);
    Pile other = heap;
    Rng g(9);
    settle(heap, Order::Rows, g);
    settle(other, Order::Random, g);
    assert(sameResult(heap, other));
}

void chunksDoNotMatter() {
    // 500 grains in the middle of a 31 x 31 grid: one go, chunks of 37, one grain at a time, all with a different order.
    Rng rng(4);
    Pile whole(31, 31);
    add(whole, 15, 15, 500);
    settle(whole, Order::Rows, rng);
    Pile chunks(31, 31);
    for (int left = 500; left > 0; left -= 37) {
        add(chunks, 15, 15, std::min(37, left));
        settle(chunks, Order::Reverse, rng);
    }
    Pile single(31, 31);
    for (int i = 0; i < 500; ++i) {
        add(single, 15, 15, 1);
        settle(single, Order::Random, rng);
    }
    assert(whole.cells == chunks.cells && whole.cells == single.cells);
    assert(whole.lost == chunks.lost && whole.lost == single.lost);
    assert(whole.topplings == chunks.topplings && whole.topplings == single.topplings);
}

void theInteriorKeepsEveryGrain() {
    Pile p(21, 21);
    add(p, 10, 10, 100);
    Rng rng(1);
    settle(p, Order::Rows, rng);
    assert(p.lost == 0);
    assert(grains(p) == 100 && p.total == 100);
}

void grainsAreNeverCreatedOrDestroyed() {
    Pile p(21, 21);
    Rng rng(8);
    for (int i = 0; i < 3000; ++i) {
        add(p, 10, 10, 1);
        settle(p, Order::Random, rng);
        assert(grains(p) + p.lost == p.total);
    }
    assert(p.lost > 0);
    assert(p.total == 3000);
}

void growingPileIsMirrorSymmetric() {
    Pile p(65, 65);
    add(p, 32, 32, 3000);
    Rng rng(2);
    settle(p, Order::Random, rng);
    for (int y = 0; y < 65; ++y) {
        for (int x = 0; x < 65; ++x) {
            assert(cell(p, x, y) == cell(p, y, x));
            assert(cell(p, x, y) == cell(p, 64 - x, y));
            assert(cell(p, x, y) == cell(p, x, 64 - y));
        }
    }
    assert(p.lost == 0);
}

void resetPlacesGrainsEverywhere() {
    Pile p(4, 4);
    add(p, 0, 0, 4);
    reset(p, 2);
    assert(p.lost == 0 && p.topplings == 0 && p.total == 32 && grains(p) == 32);
    assert(stable(p));
    for (const int c : p.cells) assert(c == 2);
    add(p, 1, 1, 2);  // 2 + 2 = 4 topples, the neighbors go to 3, the sum stays
    Rng rng(1);
    const Avalanche a = settle(p, Order::Rows, rng);
    assert(a.topplings == 1 && cell(p, 1, 1) == 0 && cell(p, 2, 1) == 3 && cell(p, 1, 2) == 3 && cell(p, 0, 1) == 3 && cell(p, 1, 0) == 3);
    assert(grains(p) == 34 && p.total == 34);
}

void dropsOutsideAreIgnored() {
    Pile p(3, 3);
    add(p, -1, 0, 5);
    add(p, 0, 3, 5);
    add(p, 3, 0, 5);
    add(p, 1, 1, 0);
    assert(grains(p) == 0 && p.total == 0 && stable(p));
}

void orderingOfCells() {
    std::vector<int> v{9, 2, 7, 4, 5, 1, 8};
    Rng rng(1);
    orderCells(v, Order::Rows, rng);
    assert((v == std::vector<int>{1, 2, 4, 5, 7, 8, 9}));
    orderCells(v, Order::Reverse, rng);
    assert((v == std::vector<int>{9, 8, 7, 5, 4, 2, 1}));
    bool movedOnce = false;
    for (std::uint64_t seed = 1; seed <= 20; ++seed) {
        std::vector<int> w{1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
        Rng g(seed);
        orderCells(w, Order::Random, g);
        std::vector<int> sorted = w;
        std::sort(sorted.begin(), sorted.end());
        assert((sorted == std::vector<int>{1, 2, 3, 4, 5, 6, 7, 8, 9, 10}));  // still the same cells
        if (!std::is_sorted(w.begin(), w.end())) movedOnce = true;
    }
    assert(movedOnce);
    std::vector<int> none;
    orderCells(none, Order::Random, rng);
    assert(none.empty());
}

void rngMatchesTheReferenceVector() {
    // splitmix64 with seed 0, reference outputs computed independently in Python.
    Rng rng(0);
    assert(rng.next() == 0xE220A8397B1DCDAFULL);
    assert(rng.next() == 0x6E789E6AA1B965F4ULL);
    assert(rng.next() == 0x06C45D188009454FULL);
}

void belowStaysInRange() {
    Rng rng(99);
    assert(rng.below(0) == 0);
    assert(rng.below(1) == 0);
    for (const std::uint32_t n : {2U, 3U, 7U, 129U}) {
        std::vector<int> seen(n, 0);
        for (int i = 0; i < 5000; ++i) {
            const std::uint32_t v = rng.below(n);
            assert(v < n);
            ++seen[v];
        }
        for (const int s : seen) assert(s > 0);  // every residue shows up
    }
    Rng a(5);
    Rng b(5);
    for (int i = 0; i < 10; ++i) assert(a.next() == b.next());
}

void bucketsAreOctaves() {
    assert(bucketOf(0) == 0);
    assert(bucketOf(1) == 1);
    assert(bucketOf(2) == 2 && bucketOf(3) == 2);
    assert(bucketOf(4) == 3 && bucketOf(7) == 3);
    assert(bucketOf(8) == 4);
    assert(bucketOf(1023) == 10 && bucketOf(1024) == 11);
    assert(bucketOf(65535) == 16);
    assert(bucketOf(65536) == kBuckets - 1);
    assert(bucketOf(1LL << 40) == kBuckets - 1);  // the last bucket collects everything above
    assert(bucketOf(-5) == 0);
}

void histogramCounts() {
    Histogram h;
    assert(h.peak() == 0);
    for (const long long s : {0LL, 0LL, 0LL, 1LL, 3LL, 2LL, 100LL, 100000LL}) h.record(s);
    assert(h.drops == 8);
    assert(h.count[0] == 3 && h.count[1] == 1 && h.count[2] == 2 && h.count[7] == 1 && h.count[kBuckets - 1] == 1);
    assert(h.largest == 100000);
    assert(h.peak() == 3);
}

void groupedNumbers() {
    assert(grouped(0) == "0");
    assert(grouped(7) == "7");
    assert(grouped(999) == "999");
    assert(grouped(1000) == "1,000");
    assert(grouped(20000) == "20,000");
    assert(grouped(100000) == "100,000");
    assert(grouped(1234567) == "1,234,567");
    assert(grouped(-1234) == "-1,234");
    assert(grouped(-12) == "-12");
}

void hexIsEightDigits() {
    assert(hex8(0) == "00000000");
    assert(hex8(255) == "000000FF");
    assert(hex8(0xDEADBEEFU) == "DEADBEEF");
}

void hashIsFnvOverTheCells() {
    // FNV-1a 32 over the cells as four little-endian bytes each, reference values computed independently in Python.
    Pile one(1, 1);
    assert(hash(one) == 0x4B95F515U);
    Pile two(2, 1);
    two.cells = {1, 2};
    assert(hash(two) == 0xDED89066U);
    Pile four(2, 2);
    four.cells = {3, 0, 1, 2};
    assert(hash(four) == 0x5ACD7C45U);
    four.cells[3] = 3;
    assert(hash(four) != 0x5ACD7C45U);
}

void survey() {
    Rng rng(1);
    std::puts("20000 grains in the middle of a 129 x 129 grid, per order:");
    std::puts("order    topplings  lost  grains-left  hash      radius(max |dx| of a grain)  cells>0  mean-height");
    for (const Order order : {Order::Rows, Order::Reverse, Order::Random}) {
        Pile p(129, 129);
        for (int left = 20000; left > 0; left -= 250) {
            add(p, 64, 64, std::min(250, left));
            settle(p, order, rng);
        }
        int radius = 0;
        long long occupied = 0;
        for (int y = 0; y < 129; ++y) {
            for (int x = 0; x < 129; ++x) {
                if (cell(p, x, y) > 0) {
                    ++occupied;
                    radius = std::max(radius, std::max(std::abs(x - 64), std::abs(y - 64)));
                }
            }
        }
        const char* name = order == Order::Rows ? "rows" : order == Order::Reverse ? "reverse" : "random";
        std::printf("%-8s %lld  %lld  %lld  %s  %d  %lld  %.4f\n", name, p.topplings, p.lost, grains(p), hex8(hash(p)).c_str(), radius, occupied,
                    static_cast<double>(grains(p)) / static_cast<double>(occupied));
    }
    std::puts("");
    std::puts("random single-grain drops on an initially empty 129 x 129 grid, avalanche size = topplings per grain:");
    Pile p(129, 129);
    Rng drop(2024);
    Histogram h;
    constexpr long kWarmup = 100000;
    constexpr long kMeasured = 200000;
    long long sumTopplings = 0;
    for (long i = 0; i < kWarmup + kMeasured; ++i) {
        add(p, static_cast<int>(drop.below(129)), static_cast<int>(drop.below(129)), 1);
        const long long before = p.topplings;
        settle(p, Order::Random, rng);
        if (i >= kWarmup) {
            h.record(p.topplings - before);
            sumTopplings += p.topplings - before;
        }
        if (i == 20000 - 1 || i == 40000 - 1 || i == kWarmup - 1) std::printf("after %ld grains: mean height %.4f, lost %lld\n", i + 1, static_cast<double>(grains(p)) / (129.0 * 129.0), p.lost);
    }
    std::printf("measured drops %lld, mean avalanche %.1f topplings, largest %lld, mean height %.4f\n", h.drops, static_cast<double>(sumTopplings) / static_cast<double>(h.drops), h.largest,
                static_cast<double>(grains(p)) / (129.0 * 129.0));
    std::puts("bucket  from..to            count   share");
    for (int b = 0; b < kBuckets; ++b) {
        const long long lo = b == 0 ? 0 : (1LL << (b - 1));
        const long long hi = b == 0 ? 0 : (1LL << b) - 1;
        std::printf("%2d  %7lld..%-7lld  %8lld  %.5f\n", b, lo, hi, h.count[static_cast<std::size_t>(b)], static_cast<double>(h.count[static_cast<std::size_t>(b)]) / static_cast<double>(h.drops));
    }
}

void theTwentyThousandGrainPile() {
    Rng rng(6);
    Pile rows(129, 129);
    Pile rev(129, 129);
    for (int left = 20000; left > 0; left -= 250) {
        add(rows, 64, 64, std::min(250, left));
        settle(rows, Order::Rows, rng);
    }
    add(rev, 64, 64, 20000);  // all at once, the other direction
    settle(rev, Order::Reverse, rng);
    assert(rows.cells == rev.cells);
    assert(rows.topplings == rev.topplings);
    assert(hash(rows) == hash(rev));
    assert(rows.lost == 0);  // the pile fits into the grid
    assert(grains(rows) == 20000);
    for (int y = 0; y < 129; ++y) {
        for (int x = 0; x < 129; ++x) {
            assert(cell(rows, x, y) == cell(rows, y, x));
            assert(cell(rows, x, y) == cell(rows, 128 - x, y));
        }
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc > 1 && std::strcmp(argv[1], "--survey") == 0) {
        survey();
        return 0;
    }
    threeGrainsStay();
    fourGrainsTopple();
    aCornerLosesTwo();
    anEdgeLosesOne();
    aLoneCellLosesAllFour();
    sixteenGrainsAtTheCenter();
    theOrderChangesTheMiddleNotTheEnd();
    fourOrdersGiveTheSameGrid();
    chunksDoNotMatter();
    theInteriorKeepsEveryGrain();
    grainsAreNeverCreatedOrDestroyed();
    growingPileIsMirrorSymmetric();
    resetPlacesGrainsEverywhere();
    dropsOutsideAreIgnored();
    orderingOfCells();
    rngMatchesTheReferenceVector();
    belowStaysInRange();
    bucketsAreOctaves();
    histogramCounts();
    groupedNumbers();
    hexIsEightDigits();
    hashIsFnvOverTheCells();
    theTwentyThousandGrainPile();
    std::puts("sandpile-avalanches logic: all asserts passed");
    return 0;
}
