// Tests for the FizzleFade logic. Plain asserts on hand-computed cases and on the property that matters:
// every pixel comes up exactly once. No framework, no SDL.
#undef NDEBUG
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "logic.hpp"

using namespace ff;

namespace {

/// Runs the whole fade and counts how often each pixel was produced.
std::vector<int> visitCounts(int w, int h, Fizzle& f) {
    std::vector<int> count(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), 0);
    while (!f.done()) {
        const Step s = f.next();
        if (s.visible) {
            assert(s.x >= 0 && s.x < w && s.y >= 0 && s.y < h);
            ++count[static_cast<std::size_t>(s.y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(s.x)];
        }
    }
    return count;
}

void bitWidthValues() {
    assert(bitWidth(0) == 0);
    assert(bitWidth(1) == 1);
    assert(bitWidth(2) == 2);
    assert(bitWidth(3) == 2);
    assert(bitWidth(255) == 8);
    assert(bitWidth(256) == 9);
    assert(bitWidth(399) == 9);
}

void registerStepByHand() {
    // 4 bits, mask 1100 (taps 4 and 3): 1 -> 1100 -> 0110 -> 0011 -> 1101 ...
    assert(lfsrNext(0x1, 0xC) == 0xC);   // a 1 falls out, so the mask is XORed in
    assert(lfsrNext(0xC, 0xC) == 0x6);   // a 0 falls out, plain shift
    assert(lfsrNext(0x6, 0xC) == 0x3);
    assert(lfsrNext(0x3, 0xC) == 0xD);   // 0011 >> 1 = 0001, XOR 1100 = 1101
}

void periodByHand() {
    // The book's four-bit example with a bad mask: 1, A, 5, 8, 4, 2 and back to 1, six values instead of fifteen.
    assert(period(4, 0xA) == 6);
    // The good mask visits all fifteen non-zero states.
    assert(period(4, 0xC) == 15);
    // A mask without the top tap leaves the cycle through 1 (here: 0001 -> 0000 and stuck), reported as 0.
    assert(period(4, 0x0) == 0);
}

void everyMaskHasMaximumPeriod() {
    for (int bits = kMinBits; bits <= kMaxBits; ++bits) {
        const uint32_t mask = tapMask(bits);
        assert(mask != 0);
        assert((mask >> (bits - 1)) == 1u);  // top tap set, nothing above the register
        const uint64_t expected = (uint64_t{1} << bits) - 1;
        assert(period(bits, mask) == expected);
        // Period 2^n - 1 inside 2^n - 1 non-zero states means no state repeats. Check it directly as well.
        std::vector<bool> seen(std::size_t{1} << bits, false);
        uint32_t s = 1;
        for (uint64_t i = 0; i < expected; ++i) {
            assert(s != 0 && s < (uint32_t{1} << bits));
            assert(!seen[s]);
            seen[s] = true;
            s = lfsrNext(s, mask);
        }
        assert(s == 1);
    }
    assert(tapMask(1) == 0);
    assert(tapMask(kMaxBits + 1) == 0);
    assert(tapMask(17) == 0x12000);  // the Wolfenstein 3D register
}

void supportedSizes() {
    assert(Fizzle::supported(320, 200));
    assert(Fizzle::supported(1, 1));
    assert(!Fizzle::supported(0, 10));
    assert(!Fizzle::supported(10, 0));
    assert(!Fizzle::supported(-3, 10));
    assert(!Fizzle::supported(4096, 4096));  // 12 + 13 bits, more than the largest mask
    assert(Fizzle::supported(1000, 1000));
    assert(!Fizzle(0, 5).valid());
    assert(Fizzle(0, 5).done());
}

void registerWidths() {
    // x gets bitWidth(w - 1) bits, y gets bitWidth(h) bits because the stored row is y + 1.
    assert(Fizzle(400, 240).bits() == 17);  // 9 + 8, exactly Wolfenstein 3D
    assert(Fizzle(400, 240).mask() == 0x12000);
    assert(Fizzle(320, 200).bits() == 17);
    assert(Fizzle(100, 60).bits() == 13);   // 7 + 6
    assert(Fizzle(200, 120).bits() == 15);  // 8 + 7
    assert(Fizzle(1, 1).bits() == 2);       // 1 + 1
    assert(Fizzle(400, 240).length() == 131071);
}

void firstPixelsByHand() {
    // 400x240: state 1 is x = 0, row field 1, so pixel (0, 0). Then 0x12000 and five more shifts have an empty row
    // field (it would be y = -1) and are skipped. 0x480 is the first state with a row field: 0x80 -> y = 127, x = 4.
    Fizzle f(400, 240);
    Step a = f.next();
    assert(a.visible && a.x == 0 && a.y == 0);
    for (int i = 0; i < 6; ++i) {
        const Step skip = f.next();
        assert(!skip.visible);
    }
    Step b = f.next();
    assert(b.visible && b.x == 4 && b.y == 127);
    assert(f.steps() == 8);
    assert(f.skipped() == 6);
    assert(f.emitted() == 2);

    // 100x60 (13 bits, mask 0x100D): state 1 gives (0,0), the next state is the mask 0x100D itself:
    // row field 0x0D = 13 -> y = 12, x = 0x100D >> 6 = 64.
    Fizzle g(100, 60);
    a = g.next();
    assert(a.visible && a.x == 0 && a.y == 0);
    b = g.next();
    assert(b.visible && b.x == 64 && b.y == 12);
}

void tinyScreen() {
    // 1x1: two bits, states 1 -> 3 -> 2 -> 1. Only state 1 maps to the pixel, the others are outside.
    Fizzle f(1, 1);
    const Step a = f.next();
    assert(a.visible && a.x == 0 && a.y == 0);
    assert(!f.done());
    assert(!f.next().visible);
    assert(!f.next().visible);
    assert(f.done());
    assert(f.steps() == 3 && f.emitted() == 1 && f.skipped() == 2);
}

void everyPixelExactlyOnce() {
    const int sizes[][2] = {{1, 1},   {1, 9},   {9, 1},    {7, 5},      {100, 60},  {200, 120}, {320, 200},
                            {400, 240}, {255, 255}, {256, 256}, {257, 31}, {64, 64}, {1000, 100}};
    for (const auto& size : sizes) {
        const int w = size[0];
        const int h = size[1];
        Fizzle f(w, h);
        assert(f.valid());
        const std::vector<int> count = visitCounts(w, h, f);
        for (int c : count) assert(c == 1);
        assert(f.emitted() == static_cast<uint64_t>(w) * static_cast<uint64_t>(h));
        assert(f.steps() == f.length());  // the register ran one full cycle, no more, no less
        assert(f.skipped() == f.length() - f.emitted());
        assert(f.done());
        // Asking again changes nothing.
        const Step after = f.next();
        assert(!after.visible);
        assert(f.steps() == f.length());
    }
}

void orderLooksRandomNotLinear() {
    // The first 100 produced pixels of a 200x120 screen must not all sit in one row or walk along a line.
    Fizzle f(200, 120);
    int sameRowAsPrevious = 0;
    int previousY = -1;
    int produced = 0;
    while (produced < 100) {
        const Step s = f.next();
        if (!s.visible) continue;
        if (s.y == previousY) ++sameRowAsPrevious;
        previousY = s.y;
        ++produced;
    }
    assert(sameRowAsPrevious < 10);
}

void rngRange() {
    Rng r(1);
    assert(r.below(0) == 0);
    for (int i = 0; i < 1000; ++i) assert(r.below(1) == 0);
    bool seen[7] = {};
    for (int i = 0; i < 1000; ++i) {
        const uint32_t v = r.below(7);
        assert(v < 7);
        seen[v] = true;
    }
    for (bool s : seen) assert(s);  // every value 0..6 shows up within 1000 draws
    Rng zero(0);                    // a zero seed must not lock the generator
    assert(zero.next() != zero.next());
    Rng a(42);
    Rng b(42);
    for (int i = 0; i < 20; ++i) assert(a.next() == b.next());  // deterministic per seed
}

void naiveCoversButWastes() {
    const int w = 20;
    const int h = 12;
    NaiveFade n(w, h, 7);
    uint64_t newly = 0;
    while (!n.done()) {
        if (n.next().visible) ++newly;
        assert(n.tries() < 1000000);
    }
    assert(newly == 240);
    assert(n.covered() == 240);
    assert(n.tries() == n.covered() + n.wasted());
    // Collecting 240 coupons takes about 240 * (ln 240 + 0.58) = 1450 draws. Wide bounds, the seed is fixed anyway.
    assert(n.tries() > 2 * 240);
    assert(n.tries() < 30 * 240);
    assert(n.wasted() > 0);
    // Finished: no more work.
    const uint64_t before = n.tries();
    assert(!n.next().visible);
    assert(n.tries() == before);

    NaiveFade one(1, 1, 3);
    assert(one.next().visible);
    assert(one.done());
    assert(one.wasted() == 0);
}

void naiveSlowsDown() {
    // The last tenth of the pixels costs far more tries than the first tenth.
    NaiveFade n(40, 25, 11);
    const uint32_t total = 1000;
    uint64_t triesAtTenth = 0;
    uint64_t triesAtNinetyPercent = 0;
    while (!n.done()) {
        n.next();
        if (triesAtTenth == 0 && n.covered() >= total / 10) triesAtTenth = n.tries();
        if (triesAtNinetyPercent == 0 && n.covered() >= total * 9 / 10) triesAtNinetyPercent = n.tries();
    }
    const uint64_t firstTenth = triesAtTenth;
    const uint64_t lastTenth = n.tries() - triesAtNinetyPercent;
    assert(firstTenth < 200);
    assert(lastTenth > 3 * firstTenth);  // about 105 tries for the first 100 pixels, about 520 for the last 100
}

void formatting() {
    assert(groupDigits(0) == "0");
    assert(groupDigits(7) == "7");
    assert(groupDigits(999) == "999");
    assert(groupDigits(1000) == "1,000");
    assert(groupDigits(131071) == "131,071");
    assert(groupDigits(1234567) == "1,234,567");
    assert(groupDigits(1000000) == "1,000,000");
    assert(percentTenths(0, 100) == 0);
    assert(percentTenths(1, 3) == 333);
    assert(percentTenths(1, 2) == 500);
    assert(percentTenths(999, 1000) == 999);
    assert(percentTenths(100, 100) == 1000);
    assert(percentTenths(5, 0) == 0);  // nothing to divide by
    assert(formatTenths(0) == "0.0");
    assert(formatTenths(1000) == "100.0");
    assert(formatTenths(713) == "71.3");
    assert(formatTenths(5) == "0.5");
}

}  // namespace

int main() {
    bitWidthValues();
    registerStepByHand();
    periodByHand();
    everyMaskHasMaximumPeriod();
    supportedSizes();
    registerWidths();
    firstPixelsByHand();
    tinyScreen();
    everyPixelExactlyOnce();
    orderLooksRandomNotLinear();
    rngRange();
    naiveCoversButWastes();
    naiveSlowsDown();
    formatting();
    std::puts("fizzle-fade logic: all tests passed");
    return 0;
}
