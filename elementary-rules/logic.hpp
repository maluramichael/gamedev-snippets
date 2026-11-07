// Elementary cellular automata: the logic of the demo, no SDL, so test.cpp can check it.
//
// A row of cells, each 0 or 1. Every step a cell looks at itself and its two neighbors, three bits that read as a number
// from 0 (000) to 7 (111), and the rule says what the cell becomes. That is a table with 8 entries, so a rule is 8 bits,
// and read as one number it is 0 to 255: bit n of the rule number is the answer for neighborhood n (Wolfram's code).
// Each new row is the next step, so the rows stacked below each other are time.
//
// THE ROW IS A RING. The cell left of the first one is the last one. Without that the edges would need an invented rule of
// their own, and the symmetries below would only hold away from the border.
//
// THE 256 RULES ARE NOT 256 DIFFERENT WORLDS. Two relabelings turn a rule into another one that draws the same picture:
//   * mirror:     swap left and right (neighborhood abc becomes cba). The new rule evolved from the mirrored row is the
//                 mirror image of the old picture.
//   * complement: swap 0 and 1 everywhere (neighborhood n becomes 7 - n, and the answer is flipped).
// Both together give four rules per world, fewer when a rule is its own mirror or complement. 256 rules fall into 88 worlds.
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace eca {

using Rule = std::uint8_t;
using Row = std::vector<std::uint8_t>;  // 0 or 1 per cell

/// Left, center and right cell (each 0 or 1) as the number 0..7 that picks a bit of the rule.
inline int neighborhood(int left, int center, int right) { return (left << 2) | (center << 1) | right; }

/// What the rule says for neighborhood n (0..7).
inline int ruleBit(Rule rule, int n) { return (rule >> (n & 7)) & 1; }

/// The 8 bits of the rule, most significant first, so the first one answers 111 and the last one 000 like in the demo.
inline std::string bits(Rule rule) {
    std::string s;
    for (int n = 7; n >= 0; --n) s += ruleBit(rule, n) != 0 ? '1' : '0';
    return s;
}

/// Flips the answer for neighborhood n. Out of range leaves the rule alone.
inline Rule toggleBit(Rule rule, int n) { return n < 0 || n > 7 ? rule : static_cast<Rule>(rule ^ (1U << n)); }

/// The rule number plus delta, wrapping around at 256 (255 + 1 is 0).
inline Rule nextRule(Rule rule, int delta) { return static_cast<Rule>((rule + delta) & 255); }

/// A single live cell in the middle (the right one of the two middle cells for an even width).
inline Row singleSeed(int width) {
    Row row(static_cast<std::size_t>(std::max(width, 0)), 0);
    if (!row.empty()) row[row.size() / 2] = 1;
    return row;
}

/// splitmix64: tiny, and the same numbers on every platform (std::uniform_real_distribution is not).
inline std::uint64_t splitmix(std::uint64_t& state) {
    state += 0x9E3779B97F4A7C15ULL;
    std::uint64_t z = state;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
}

/// Every cell is 0 or 1 with equal chance.
inline Row randomSeed(int width, std::uint64_t seed) {
    Row row(static_cast<std::size_t>(std::max(width, 0)), 0);
    std::uint64_t state = seed;
    for (std::uint8_t& cell : row) cell = static_cast<std::uint8_t>((splitmix(state) >> 40) & 1U);
    return row;
}

/// One step on a ring. Any nonzero cell counts as 1.
inline Row step(Rule rule, const Row& row) {
    const std::size_t w = row.size();
    Row next(w, 0);
    for (std::size_t x = 0; x < w; ++x) {
        const int left = row[(x + w - 1) % w] != 0 ? 1 : 0;
        const int center = row[x] != 0 ? 1 : 0;
        const int right = row[(x + 1) % w] != 0 ? 1 : 0;
        next[x] = static_cast<std::uint8_t>(ruleBit(rule, neighborhood(left, center, right)));
    }
    return next;
}

/// The seed and the rows that follow from it, `rows` in total (row 0 is the seed itself).
inline std::vector<Row> evolve(Rule rule, const Row& seed, int rows) {
    std::vector<Row> out;
    if (rows <= 0) return out;
    out.reserve(static_cast<std::size_t>(rows));
    out.push_back(seed);
    for (int i = 1; i < rows; ++i) {
        Row next = step(rule, out.back());
        out.push_back(std::move(next));
    }
    return out;
}

inline Row mirrorRow(const Row& row) { return Row(row.rbegin(), row.rend()); }

inline Row complementRow(const Row& row) {
    Row out(row.size(), 0);
    for (std::size_t i = 0; i < row.size(); ++i) out[i] = row[i] != 0 ? 0 : 1;
    return out;
}

/// The rule that sees left and right swapped: neighborhood abc of the new rule answers what cba answered.
inline Rule mirror(Rule rule) {
    unsigned out = 0;
    for (int n = 0; n < 8; ++n) {
        const int swapped = ((n & 1) << 2) | (n & 2) | ((n >> 2) & 1);
        out |= static_cast<unsigned>(ruleBit(rule, swapped)) << n;
    }
    return static_cast<Rule>(out);
}

/// The rule that sees 0 and 1 swapped: neighborhood n answers the opposite of what 7 - n (all bits flipped) answered.
inline Rule complement(Rule rule) {
    unsigned out = 0;
    for (int n = 0; n < 8; ++n) out |= static_cast<unsigned>(1 - ruleBit(rule, 7 - n)) << n;
    return static_cast<Rule>(out);
}

inline Rule mirrorComplement(Rule rule) { return mirror(complement(rule)); }

/// The rule itself, its mirror, its complement and both together, in that order. Entries repeat when a rule is its own
/// mirror or complement.
inline std::array<Rule, 4> variants(Rule rule) { return {rule, mirror(rule), complement(rule), mirrorComplement(rule)}; }

/// The smallest rule number of the world a rule belongs to.
inline Rule canonical(Rule rule) {
    const auto v = variants(rule);
    return *std::min_element(v.begin(), v.end());
}

/// All rules of the same world, ascending, without repeats (1, 2 or 4 of them).
inline std::vector<Rule> world(Rule rule) {
    const auto v = variants(rule);
    std::vector<Rule> out(v.begin(), v.end());
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

/// Number of worlds, counted by walking all 256 rules (88).
inline int worldCount() {
    int count = 0;
    for (int i = 0; i < 256; ++i) count += canonical(static_cast<Rule>(i)) == i ? 1 : 0;
    return count;
}

/// 1-based number of a rule's world when the worlds are sorted by their smallest rule.
inline int worldIndex(Rule rule) {
    const Rule head = canonical(rule);
    int index = 1;
    for (int i = 0; i < head; ++i) index += canonical(static_cast<Rule>(i)) == i ? 1 : 0;
    return index;
}

}  // namespace eca
