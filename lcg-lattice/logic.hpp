// Linear congruential generators and what their output looks like. No SDL, covered by test.cpp.
//
// Eight generators behind one interface: next() returns the output word, outBits() says how many of its low bits are
// meaningful, unit() maps it to [0, 1). The statistics below are what the demo shows: the pairs (x_n, x_n+1), the
// repeat period of a single output bit, how often a bit equals the previous one, and the longest run.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace lcg {

enum class Kind { Tiny, Diy32, Ansi, Msvc, ParkMiller, Xorshift32, Pcg32, Wolf };
inline constexpr int kKindCount = 8;

inline constexpr std::uint32_t kParkMillerM = 2147483647U;  // 2^31 - 1

/// The 256 byte table of Wolfenstein 3D (Game Engine Black Book, chapter 4.12). Its own author said it is not even a shuffle.
inline constexpr std::array<std::uint8_t, 256> kWolfTable{
    0, 8, 109, 220, 222, 241, 149, 107, 75, 248, 254, 140, 16, 66, 74, 21,
    211, 47, 80, 242, 154, 27, 205, 128, 161, 89, 77, 36, 95, 110, 85, 48,
    212, 140, 211, 249, 22, 79, 200, 50, 28, 188, 52, 140, 202, 120, 68, 145,
    62, 70, 184, 190, 91, 197, 152, 224, 149, 104, 25, 178, 252, 182, 202, 182,
    141, 197, 4, 81, 181, 242, 145, 42, 39, 227, 156, 198, 225, 193, 219, 93,
    122, 175, 249, 0, 175, 143, 70, 239, 46, 246, 163, 53, 163, 109, 168, 135,
    2, 235, 25, 92, 20, 145, 138, 77, 69, 166, 78, 176, 173, 212, 166, 113,
    94, 161, 41, 50, 239, 49, 111, 164, 70, 60, 2, 37, 171, 75, 136, 156,
    11, 56, 42, 146, 138, 229, 73, 146, 77, 61, 98, 196, 135, 106, 63, 197,
    195, 86, 96, 203, 113, 101, 170, 247, 181, 113, 80, 250, 108, 7, 255, 237,
    129, 226, 79, 107, 112, 166, 103, 241, 24, 223, 239, 120, 198, 58, 60, 82,
    128, 3, 184, 66, 143, 224, 145, 224, 81, 206, 163, 45, 63, 90, 168, 114,
    59, 33, 159, 95, 28, 139, 123, 98, 125, 196, 15, 70, 194, 253, 54, 14,
    109, 226, 71, 17, 161, 93, 186, 87, 244, 138, 20, 52, 123, 251, 26, 36,
    17, 46, 52, 231, 232, 76, 31, 221, 84, 37, 216, 165, 212, 106, 197, 242,
    98, 43, 39, 175, 254, 145, 190, 84, 118, 222, 187, 136, 120, 163, 236, 249,
};

struct Rng {
    Kind kind = Kind::Tiny;
    std::uint64_t state = 0;
    std::uint64_t inc = 0;  // PCG stream, unused by the others
};

struct Point {
    double x = 0.0;
    double y = 0.0;
};

namespace detail {

inline std::uint32_t pcgStep(Rng& r) {
    const std::uint64_t old = r.state;
    r.state = old * 6364136223846793005ULL + r.inc;
    const auto xorShifted = static_cast<std::uint32_t>(((old >> 18U) ^ old) >> 27U);
    const auto rot = static_cast<std::uint32_t>(old >> 59U);
    return (xorShifted >> rot) | (xorShifted << ((0U - rot) & 31U));
}

}  // namespace detail

/// A generator in its start state. Every seed is valid: generators that must not start at 0 get a nonzero state.
inline Rng make(Kind kind, std::uint32_t seed) {
    Rng r;
    r.kind = kind;
    switch (kind) {
        case Kind::Tiny: r.state = seed & 4095U; break;
        case Kind::Diy32: r.state = seed; break;
        case Kind::Ansi: r.state = seed; break;
        case Kind::Msvc: r.state = seed & 0x7fffffffU; break;
        case Kind::ParkMiller: r.state = seed == 0 ? 1U : (seed - 1U) % (kParkMillerM - 1U) + 1U; break;  // 1 .. m - 1
        case Kind::Xorshift32: r.state = seed == 0 ? 1U : seed; break;
        case Kind::Pcg32:
            // pcg32_srandom_r(seed, 54): two steps with the seed added in between.
            r.state = 0;
            r.inc = (54ULL << 1U) | 1ULL;
            detail::pcgStep(r);
            r.state += seed;
            detail::pcgStep(r);
            break;
        case Kind::Wolf: r.state = seed & 255U; break;
    }
    return r;
}

/// Seed number n of the demo's "new seed" button. Number 0 is the default seed 1; the odd multiplier makes the
/// mapping a bijection on 32 bits, so different numbers never give the same seed.
inline std::uint32_t seedFor(std::uint32_t n) { return n * 2654435761U + 1U; }

/// Number of meaningful low bits of the output word.
inline int outBits(Kind kind) {
    switch (kind) {
        case Kind::Tiny: return 12;
        case Kind::Diy32: return 32;
        case Kind::Ansi: return 15;
        case Kind::Msvc: return 15;
        case Kind::ParkMiller: return 31;
        case Kind::Xorshift32: return 32;
        case Kind::Pcg32: return 32;
        case Kind::Wolf: return 8;
    }
    return 0;
}

/// The next output word.
inline std::uint32_t next(Rng& r) {
    switch (r.kind) {
        case Kind::Tiny:
            r.state = (65U * r.state + 1U) & 4095U;
            return static_cast<std::uint32_t>(r.state);
        case Kind::Diy32: {
            // The whole state is the output, low bits included: the classic mistake.
            const auto s = static_cast<std::uint32_t>(r.state) * 1664525U + 1013904223U;
            r.state = s;
            return s;
        }
        case Kind::Ansi: {
            // The portable rand() of the C standard: state * 1103515245 + 12345, return bits 16..30.
            const auto s = static_cast<std::uint32_t>(r.state) * 1103515245U + 12345U;
            r.state = s;
            return (s >> 16U) & 0x7fffU;
        }
        case Kind::Msvc:
            r.state = (214013ULL * r.state + 2531011ULL) & 0x7fffffffULL;
            return static_cast<std::uint32_t>(r.state >> 16U);
        case Kind::ParkMiller:
            r.state = (16807ULL * r.state) % kParkMillerM;
            return static_cast<std::uint32_t>(r.state);
        case Kind::Xorshift32: {
            auto x = static_cast<std::uint32_t>(r.state);
            x ^= x << 13U;
            x ^= x >> 17U;
            x ^= x << 5U;
            r.state = x;
            return x;
        }
        case Kind::Pcg32: return detail::pcgStep(r);
        case Kind::Wolf:
            r.state = (r.state + 1U) & 255U;
            return kWolfTable[static_cast<std::size_t>(r.state)];
    }
    return 0;
}

/// Output word as a number in [0, 1). Park-Miller divides by its modulus, the others by 2^bits.
inline double unit(Kind kind, std::uint32_t out) {
    if (kind == Kind::ParkMiller) return static_cast<double>(out) / static_cast<double>(kParkMillerM);
    return std::ldexp(static_cast<double>(out), -outBits(kind));
}

/// Bit k of an output word, 0 for k outside 0..31.
inline std::uint32_t bitOf(std::uint32_t out, int k) {
    if (k < 0 || k > 31) return 0;
    return (out >> static_cast<unsigned>(k)) & 1U;
}

/// n outputs from a copy of the generator.
inline std::vector<std::uint32_t> draw(Rng r, std::size_t n) {
    std::vector<std::uint32_t> out;
    out.reserve(n);
    for (std::size_t i = 0; i < n; ++i) out.push_back(next(r));
    return out;
}

inline std::vector<std::uint8_t> bitSequence(const std::vector<std::uint32_t>& outs, int k) {
    std::vector<std::uint8_t> seq;
    seq.reserve(outs.size());
    for (const std::uint32_t v : outs) seq.push_back(static_cast<std::uint8_t>(bitOf(v, k)));
    return seq;
}

/// Share of positions where the value equals its predecessor. A fair coin gives 0.5, a strictly alternating bit 0.
inline double sameAsPrevious(const std::vector<std::uint8_t>& seq) {
    if (seq.size() < 2) return 0.0;
    std::size_t same = 0;
    for (std::size_t i = 1; i < seq.size(); ++i) same += seq[i] == seq[i - 1] ? 1U : 0U;
    return static_cast<double>(same) / static_cast<double>(seq.size() - 1);
}

inline std::size_t longestRun(const std::vector<std::uint8_t>& seq) {
    std::size_t best = 0;
    std::size_t run = 0;
    for (std::size_t i = 0; i < seq.size(); ++i) {
        run = (i > 0 && seq[i] == seq[i - 1]) ? run + 1 : 1;
        best = std::max(best, run);
    }
    return best;
}

/// A number in 0..n-1 from the top bits of the output instead of `x % n`: the high bits are the good ones. `out` must be
/// below 2^bits. The result is clamped to n - 1, 0 for n == 0 or an unusable bit count.
inline std::uint32_t scaled(std::uint32_t out, int bits, std::uint32_t n) {
    if (n == 0 || bits <= 0 || bits > 32) return 0;
    const auto v = static_cast<std::uint32_t>((static_cast<std::uint64_t>(out) * n) >> static_cast<unsigned>(bits));
    return std::min(v, n - 1U);
}

/// Smallest p in 1..maxPeriod with seq[i] == seq[i + p] for every i, or 0 if there is none. Only periods up to half the
/// length are judged, so at least half the sequence is compared.
inline std::size_t smallestPeriod(const std::vector<std::uint8_t>& seq, std::size_t maxPeriod) {
    const std::size_t limit = std::min(maxPeriod, seq.size() / 2);
    for (std::size_t p = 1; p <= limit; ++p) {
        bool same = true;
        for (std::size_t i = 0; same && i + p < seq.size(); ++i) same = seq[i] == seq[i + p];
        if (same) return p;
    }
    return 0;
}

/// Pearson chi-square of the outputs, mapped to [0, 1) and counted in equal bins. 0 for no data.
inline double chiSquare(Kind kind, const std::vector<std::uint32_t>& outs, std::size_t bins) {
    if (outs.empty() || bins == 0) return 0.0;
    std::vector<std::size_t> count(bins, 0);
    for (const std::uint32_t v : outs) {
        const auto b = static_cast<std::size_t>(unit(kind, v) * static_cast<double>(bins));
        ++count[std::min(b, bins - 1)];
    }
    const double expected = static_cast<double>(outs.size()) / static_cast<double>(bins);
    double chi = 0.0;
    for (const std::size_t c : count) {
        const double d = static_cast<double>(c) - expected;
        chi += d * d / expected;
    }
    return chi;
}

/// Repeat period of every output bit (0 = none up to maxPeriod), judged on the given outputs.
inline std::array<std::size_t, 32> bitPeriods(const std::vector<std::uint32_t>& outs, int bits, std::size_t maxPeriod) {
    std::array<std::size_t, 32> periods{};
    for (int k = 0; k < bits && k < 32; ++k) periods[static_cast<std::size_t>(k)] = smallestPeriod(bitSequence(outs, k), maxPeriod);
    return periods;
}

/// Consecutive pairs (x_n, x_n+1) as points in the unit square, keeping those with x_n < xMax. Stops at maxPoints, after
/// `budget` draws, or after one full cycle of the generator, whichever comes first.
inline std::vector<Point> collectPairs(Rng r, double xMax, std::size_t maxPoints, std::size_t budget) {
    std::vector<Point> points;
    if (xMax <= 0.0 || maxPoints == 0 || budget == 0) return points;
    const Kind kind = r.kind;
    double prev = unit(kind, next(r));
    const std::uint64_t cycleState = r.state;  // the state after the first draw: seeing it again closes the cycle
    for (std::size_t i = 0; i < budget && points.size() < maxPoints; ++i) {
        const double cur = unit(kind, next(r));
        if (prev < xMax) points.push_back({prev, cur});
        prev = cur;
        if (r.state == cycleState) break;
    }
    return points;
}

/// True when the output word is the whole state of an LCG, so the pairs of a narrow strip can be written down directly.
/// (xorshift32 returns its state too, but near state 0 it is a special case that no real run visits often: it is scanned.)
inline bool outputIsState(Kind kind) {
    return kind == Kind::Tiny || kind == Kind::Diy32 || kind == Kind::ParkMiller;
}

namespace detail {

/// A small hash-based random stream for picking states (not one of the generators under test).
inline std::uint32_t pick(std::uint32_t& s) {
    s += 0x9e3779b9U;
    std::uint32_t z = s;
    z = (z ^ (z >> 16U)) * 0x85ebca6bU;
    z = (z ^ (z >> 13U)) * 0xc2b2ae35U;
    return z ^ (z >> 16U);
}

}  // namespace detail

/// Pairs (s, next(s)) for states s whose output lies below xMax. A very narrow strip is hit by only one run in a million
/// draws, but for a generator whose output is its state the pairs can be written down directly: this takes up to `count`
/// random states of the strip (all of them if there are fewer), the same dots a very long run would have produced.
/// Empty for generators whose output is not the state, for xMax <= 0 and for count 0.
inline std::vector<Point> statePairs(Kind kind, double xMax, std::size_t count, std::uint32_t seed) {
    std::vector<Point> points;
    if (!outputIsState(kind) || xMax <= 0.0 || count == 0) return points;
    // Valid states are lo .. lo + range - 1, and the output of state s is s itself.
    const std::uint64_t lo = kind == Kind::ParkMiller ? 1U : 0U;  // the multiplicative LCG never leaves 0, so 0 is no state
    std::uint64_t full = 0;  // number of different output values 2^bits, or m for Park-Miller
    switch (kind) {
        case Kind::Tiny: full = 4096; break;
        case Kind::ParkMiller: full = kParkMillerM; break;
        default: full = std::uint64_t{1} << 32U; break;
    }
    const double scaled = std::ceil(xMax * static_cast<double>(full));
    const std::uint64_t hi = scaled >= static_cast<double>(full) ? full : static_cast<std::uint64_t>(scaled);  // exclusive
    if (hi <= lo) return points;
    const std::uint64_t span = hi - lo;
    auto add = [&](std::uint64_t s) {
        Rng r;
        r.kind = kind;
        r.state = s;
        const std::uint32_t out = next(r);
        points.push_back({unit(kind, static_cast<std::uint32_t>(s)), unit(kind, out)});
    };
    if (span <= count) {
        for (std::uint64_t s = lo; s < hi; ++s) add(s);
        return points;
    }
    std::uint32_t stream = seed;
    for (std::size_t i = 0; i < count; ++i) add(lo + ((static_cast<std::uint64_t>(detail::pick(stream)) * span) >> 32U));
    return points;
}

}  // namespace lcg
