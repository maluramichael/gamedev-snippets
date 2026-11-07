// Tests for the elementary cellular automaton logic. Plain asserts on hand-computed cases, no framework, no SDL.
// Rows are written as strings of 0 and 1 to keep the expected values readable.
#undef NDEBUG
#include <algorithm>
#include <bit>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <set>
#include <string>
#include <vector>

#include "logic.hpp"

using namespace eca;

namespace {

Row row(const std::string& s) {
    Row r;
    for (char c : s) r.push_back(c == '1' ? 1 : 0);
    return r;
}

std::string text(const Row& r) {
    std::string s;
    for (std::uint8_t v : r) s += v ? '1' : '0';
    return s;
}

/// The cells x0..x1 (inclusive) of a row as text, to compare the part of a row that matters.
std::string slice(const Row& r, int x0, int x1) { return text(Row(r.begin() + x0, r.begin() + x1 + 1)); }

int popcount(const Row& r) { return static_cast<int>(std::count(r.begin(), r.end(), std::uint8_t{1})); }

void ruleTable() {
    // 30 = 0b00011110: 111 -> 0, 110 -> 0, 101 -> 0, 100 -> 1, 011 -> 1, 010 -> 1, 001 -> 1, 000 -> 0
    const int expect30[8] = {0, 1, 1, 1, 1, 0, 0, 0};  // indexed by the neighborhood read as a number, 0 (000) to 7 (111)
    for (int n = 0; n < 8; ++n) assert(ruleBit(30, n) == expect30[n]);
    // 110 = 0b01101110
    const int expect110[8] = {0, 1, 1, 1, 0, 1, 1, 0};
    for (int n = 0; n < 8; ++n) assert(ruleBit(110, n) == expect110[n]);
    assert(ruleBit(0, 5) == 0);
    assert(ruleBit(255, 0) == 1);
    assert(neighborhood(1, 1, 1) == 7);
    assert(neighborhood(1, 0, 0) == 4);
    assert(neighborhood(0, 0, 1) == 1);
    assert(neighborhood(0, 1, 0) == 2);
    assert(bits(30) == "00011110");
    assert(bits(110) == "01101110");
    assert(bits(0) == "00000000");
    assert(bits(255) == "11111111");
}

void editingBits() {
    assert(toggleBit(0, 0) == 1);
    assert(toggleBit(0, 7) == 128);
    assert(toggleBit(110, 0) == 111);
    assert(toggleBit(110, 1) == 108);
    assert(toggleBit(toggleBit(110, 4), 4) == 110);
    assert(toggleBit(110, 8) == 110);  // out of range: unchanged
    assert(toggleBit(110, -1) == 110);
    assert(nextRule(255, 1) == 0);  // wraps
    assert(nextRule(0, -1) == 255);
    assert(nextRule(110, 1) == 111);
    assert(nextRule(110, -10) == 100);
}

void seeds() {
    assert(text(singleSeed(5)) == "00100");
    assert(text(singleSeed(6)) == "000100");  // even width: the cell right of the middle
    assert(text(singleSeed(1)) == "1");
    assert(singleSeed(0).empty());
    assert(randomSeed(64, 7) == randomSeed(64, 7));  // same seed, same row
    assert(randomSeed(64, 7) != randomSeed(64, 8));
    assert(popcount(randomSeed(1000, 1)) > 400 && popcount(randomSeed(1000, 1)) < 600);  // about half, wide margin
    assert(randomSeed(0, 1).empty());
}

void stepAndWrap() {
    // Width 5, one cell at the left edge. Rule 90 is XOR of both neighbors, so the new cells sit at x = 1 and, wrapped, at x = 4.
    assert(text(step(90, row("10000"))) == "01001");
    // 204 copies the cell itself, 170 takes the right neighbor, 240 the left one: shifts show the wrap too.
    assert(text(step(204, row("10110"))) == "10110");
    assert(text(step(170, row("1000"))) == "0001");  // cell 3 reads cell 0 across the edge
    assert(text(step(240, row("1000"))) == "0100");
    assert(text(step(240, row("0001"))) == "1000");  // cell 0 reads cell 3
    // Tiny widths: a single cell is its own left and right neighbor, two cells see each other on both sides.
    assert(text(step(255, row("1"))) == "1");  // 111 -> 1
    assert(text(step(30, row("1"))) == "0");   // 111 -> 0
    assert(text(step(129, row("0"))) == "1");  // 000 -> 1 (129 = bits 0 and 7)
    assert(text(step(30, row("10"))) == "10");  // cell 0 sees 010 -> 1, cell 1 sees 101 -> 0
    assert(text(step(90, row("10"))) == "00");
    assert(step(110, Row{}).empty());  // no cells, no modulo by zero
}

void rule90IsSierpinski() {
    // Rule 90 is XOR of the two neighbors, so a single cell grows Pascal's triangle mod 2. Width 21, the seed sits at x = 10.
    const auto rows = evolve(90, singleSeed(21), 8);
    assert(rows.size() == 8);
    assert(slice(rows[0], 9, 11) == "010");
    assert(slice(rows[1], 8, 12) == "01010");
    assert(slice(rows[2], 7, 13) == "0100010");        // C(2,k) = 1 2 1: only the outer two are odd
    assert(slice(rows[3], 6, 14) == "010101010");      // C(3,k) = 1 3 3 1: all odd
    assert(slice(rows[4], 5, 15) == "01000000010");    // C(4,k) = 1 4 6 4 1: only the outer two
    assert(slice(rows[5], 4, 16) == "0101000001010");  // C(5,k) = 1 5 10 10 5 1: 1 5 . . 5 1
}

void rule90CountsLiveCells() {
    // Row n of Pascal's triangle has 2^popcount(n) odd entries (Lucas), so row n of rule 90 has that many live cells.
    const auto rows = evolve(90, singleSeed(81), 34);  // wide enough that nothing wraps within 33 rows
    for (int n = 0; n < 34; ++n) assert(popcount(rows[n]) == (1 << std::popcount(static_cast<unsigned>(n))));
    assert(popcount(rows[31]) == 32);
    assert(popcount(rows[32]) == 2);
}

void rule30() {
    const auto rows = evolve(30, singleSeed(21), 5);
    assert(slice(rows[1], 9, 11) == "111");
    assert(slice(rows[2], 8, 12) == "11001");
    assert(slice(rows[3], 7, 13) == "1101111");
    assert(slice(rows[4], 6, 14) == "110010001");
}

void rule110() {
    const auto rows = evolve(110, singleSeed(21), 5);
    assert(slice(rows[1], 8, 11) == "0110");  // grows to the left only: 100 -> 0
    assert(slice(rows[2], 7, 11) == "01110");
    assert(slice(rows[3], 6, 11) == "011010");
    assert(slice(rows[4], 5, 11) == "0111110");
}

void rule184Traffic() {
    // 184: a 1 moves right if the cell ahead is 0, otherwise it waits. Ring of four.
    assert(text(step(184, row("1100"))) == "1010");  // the front car moves, the one behind is blocked for a step
    assert(text(step(184, row("1010"))) == "0101");  // free road: everybody moves
    // Cars are conserved on a ring, for any start row.
    Row r = randomSeed(64, 3);
    const int cars = popcount(r);
    for (int i = 0; i < 100; ++i) {
        r = step(184, r);
        assert(popcount(r) == cars);
    }
}

void evolveShape() {
    assert(evolve(30, singleSeed(7), 0).empty());
    assert(evolve(30, singleSeed(7), -3).empty());
    const auto one = evolve(30, singleSeed(7), 1);
    assert(one.size() == 1 && one[0] == singleSeed(7));  // row 0 is the seed itself
    const auto rows = evolve(30, singleSeed(7), 4);
    assert(rows.size() == 4);
    for (const Row& r : rows) assert(r.size() == 7);
    assert(rows[1] == step(30, rows[0]));
    assert(rows[3] == step(30, rows[2]));
    const auto empty = evolve(30, Row{}, 3);
    assert(empty.size() == 3 && empty[2].empty());
}

void mirrorComplementOf110() {
    // The four faces of rule 110, worked out by hand from the rule table.
    assert(mirror(110) == 124);
    assert(complement(110) == 137);
    assert(mirrorComplement(110) == 193);
    const auto v = variants(110);
    assert(v[0] == 110 && v[1] == 124 && v[2] == 137 && v[3] == 193);
    // Other hand-computed ones.
    assert(mirror(30) == 86 && complement(30) == 135 && mirrorComplement(30) == 149);
    assert(mirror(184) == 226 && complement(184) == 226 && mirrorComplement(184) == 184);
    assert(mirror(90) == 90 && complement(90) == 165 && mirrorComplement(90) == 165);
    assert(mirror(2) == 16 && complement(2) == 191 && mirrorComplement(2) == 247);
    assert(mirror(170) == 240 && complement(170) == 170 && mirrorComplement(170) == 240);
    assert(mirror(204) == 204 && complement(204) == 204);  // the identity rule has no preferred side or color
    assert(complement(0) == 255 && complement(255) == 0 && mirror(0) == 0 && mirror(255) == 255);
}

void transformsAreInvolutions() {
    for (int i = 0; i < 256; ++i) {
        const auto r = static_cast<Rule>(i);
        assert(mirror(mirror(r)) == r);
        assert(complement(complement(r)) == r);
        assert(mirror(complement(r)) == complement(mirror(r)));
        assert(mirrorComplement(r) == mirror(complement(r)));
    }
}

/// The real reason the transforms are what they are: evolving the transformed rule from the transformed seed gives the
/// transformed picture. Checked on all 256 rules over a ring, so the wrap-around is part of it.
void transformsMatchTheirPictures() {
    for (int i = 0; i < 256; ++i) {
        const auto r = static_cast<Rule>(i);
        const Row seed = randomSeed(17, 100 + static_cast<std::uint64_t>(i));
        const auto base = evolve(r, seed, 12);
        const auto mir = evolve(mirror(r), mirrorRow(seed), 12);
        const auto com = evolve(complement(r), complementRow(seed), 12);
        const auto both = evolve(mirrorComplement(r), complementRow(mirrorRow(seed)), 12);
        for (std::size_t t = 0; t < base.size(); ++t) {
            assert(mir[t] == mirrorRow(base[t]));
            assert(com[t] == complementRow(base[t]));
            assert(both[t] == complementRow(mirrorRow(base[t])));
        }
    }
    assert(text(mirrorRow(row("11010"))) == "01011");
    assert(text(complementRow(row("11010"))) == "00101");
}

void worlds() {
    // Burnside: 256 rules, 64 fixed by the mirror, 16 by the complement, 16 by both together, (256 + 64 + 16 + 16) / 4 = 88 worlds.
    int fixedMirror = 0;
    int fixedComplement = 0;
    int fixedBoth = 0;
    int size1 = 0;
    int size2 = 0;
    int size4 = 0;
    std::set<Rule> canonicals;
    std::set<int> indices;
    for (int i = 0; i < 256; ++i) {
        const auto r = static_cast<Rule>(i);
        fixedMirror += mirror(r) == r;
        fixedComplement += complement(r) == r;
        fixedBoth += mirrorComplement(r) == r;
        const auto w = world(r);
        assert(std::is_sorted(w.begin(), w.end()));
        assert(std::find(w.begin(), w.end(), r) != w.end());
        for (Rule m : w) assert(world(m) == w && worldIndex(m) == worldIndex(r) && canonical(m) == canonical(r));
        assert(canonical(r) == w.front());
        if (w.front() == r) {
            canonicals.insert(r);
            (w.size() == 1 ? size1 : w.size() == 2 ? size2 : size4) += 1;
        }
        indices.insert(worldIndex(r));
    }
    assert(fixedMirror == 64);
    assert(fixedComplement == 16);
    assert(fixedBoth == 16);
    assert(canonicals.size() == 88);
    assert(worldCount() == 88);
    assert((256 + fixedMirror + fixedComplement + fixedBoth) / 4 == 88);
    // Orbit sizes: 8 rules are fixed by everything, the other mirror-symmetric ones pair up, and so on (8 + 36 + 44 = 88).
    assert(size1 == 8 && size2 == 36 && size4 == 44);
    assert(8 * 1 + 36 * 2 + 44 * 4 == 256);
    // Indices run 1 to 88 without gaps.
    assert(indices.size() == 88 && *indices.begin() == 1 && *indices.rbegin() == 88);
    assert(worldIndex(0) == 1 && worldIndex(255) == 1);
    assert(worldIndex(1) == 2 && worldIndex(127) == 2);
    assert(worldIndex(2) == 3 && worldIndex(16) == 3 && worldIndex(191) == 3 && worldIndex(247) == 3);
    const std::vector<Rule> w110 = world(110);
    assert((w110 == std::vector<Rule>{110, 124, 137, 193}));
    assert((world(90) == std::vector<Rule>{90, 165}));
    assert((world(184) == std::vector<Rule>{184, 226}));
    assert((world(0) == std::vector<Rule>{0, 255}));
    assert((world(204) == std::vector<Rule>{204}));
    assert((world(2) == std::vector<Rule>{2, 16, 191, 247}));
}

}  // namespace

int main() {
    ruleTable();
    editingBits();
    seeds();
    stepAndWrap();
    rule90IsSierpinski();
    rule90CountsLiveCells();
    rule30();
    rule110();
    rule184Traffic();
    evolveShape();
    mirrorComplementOf110();
    transformsAreInvolutions();
    transformsMatchTheirPictures();
    worlds();
    std::puts("elementary-rules logic: all asserts passed");
    return 0;
}
