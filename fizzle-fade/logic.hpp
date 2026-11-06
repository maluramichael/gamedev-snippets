// FizzleFade logic, no SDL.
//
// The screen transition from Wolfenstein 3D: every pixel changes exactly once, in a scrambled order, at constant
// speed. The trick is a linear feedback shift register (LFSR). A register of n bits that is shifted right, with a
// mask XORed in whenever a 1 falls out, runs through all 2^n - 1 non-zero values before it repeats. Cut the register
// into an x part and a y part and every pixel comes up exactly once, with no memory for "already done".
//
// NaiveFade is the version everybody writes first: pick a random pixel, check a flag, try again if it is taken.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace ff {

constexpr int kMinBits = 2;
constexpr int kMaxBits = 24;

/// One pixel position the fade produced, or a state that fell outside the screen (visible == false).
struct Step {
    int x = 0;
    int y = 0;
    bool visible = false;
};

/// Number of bits needed to write n: bitWidth(0) = 0, bitWidth(255) = 8, bitWidth(256) = 9.
constexpr int bitWidth(uint32_t n) {
    int bits = 0;
    while (n != 0) {
        ++bits;
        n >>= 1;
    }
    return bits;
}

namespace detail {
/// Tap positions are counted from 1 (the top bit of an n-bit register is tap n).
constexpr uint32_t tap(int position) { return uint32_t{1} << (position - 1); }
}  // namespace detail

/// Mask for the right-shifting (Galois) register of the given width that gives the maximum period 2^bits - 1.
/// 0 for widths outside kMinBits..kMaxBits. The test walks every one of these masks through its full cycle.
constexpr uint32_t tapMask(int bits) {
    using detail::tap;
    switch (bits) {
        case 2: return tap(2) | tap(1);
        case 3: return tap(3) | tap(2);
        case 4: return tap(4) | tap(3);
        case 5: return tap(5) | tap(3);
        case 6: return tap(6) | tap(5);
        case 7: return tap(7) | tap(6);
        case 8: return tap(8) | tap(6) | tap(5) | tap(4);
        case 9: return tap(9) | tap(5);
        case 10: return tap(10) | tap(7);
        case 11: return tap(11) | tap(9);
        case 12: return tap(12) | tap(6) | tap(4) | tap(1);
        case 13: return tap(13) | tap(4) | tap(3) | tap(1);
        case 14: return tap(14) | tap(5) | tap(3) | tap(1);
        case 15: return tap(15) | tap(14);
        case 16: return tap(16) | tap(15) | tap(13) | tap(4);
        case 17: return tap(17) | tap(14);  // 0x12000, the register of Wolfenstein 3D
        case 18: return tap(18) | tap(11);
        case 19: return tap(19) | tap(6) | tap(2) | tap(1);
        case 20: return tap(20) | tap(17);
        case 21: return tap(21) | tap(19);
        case 22: return tap(22) | tap(21);
        case 23: return tap(23) | tap(18);
        case 24: return tap(24) | tap(23) | tap(22) | tap(17);
        default: return 0;
    }
}

/// One step of the register: shift right, and if a 1 fell out, XOR the mask in. No branch needed.
constexpr uint32_t lfsrNext(uint32_t state, uint32_t mask) { return (state >> 1) ^ ((uint32_t{0} - (state & 1U)) & mask); }

/// Steps until the register, started at 1, is back at 1. 0 if it never gets back (it runs into the dead state 0, or
/// a bad mask sends it into a cycle that does not contain 1). A good mask gives 2^bits - 1.
inline uint64_t period(int bits, uint32_t mask) {
    const uint64_t limit = uint64_t{1} << bits;
    uint32_t state = 1;
    for (uint64_t i = 1; i <= limit; ++i) {
        state = lfsrNext(state, mask);
        if (state == 0) return 0;
        if (state == 1) return i;
    }
    return 0;
}

/// The fade by register. The register holds (x << ybits) | (y + 1): the +1 keeps the pixel (0, 0) away from the dead
/// state 0, so all w * h pixels have a code that the register can reach. States that decode to a column or row outside
/// the screen are skipped, they cost one shift and no pixel.
class Fizzle {
public:
    /// True when a register of at most kMaxBits bits can hold both coordinates.
    static bool supported(int w, int h) {
        if (w < 1 || h < 1) return false;
        return widthBits(w) + bitWidth(static_cast<uint32_t>(h)) <= kMaxBits;
    }

    Fizzle(int w, int h) : w_(w), h_(h) {
        if (!supported(w, h)) {
            done_ = true;
            return;
        }
        valid_ = true;
        ybits_ = bitWidth(static_cast<uint32_t>(h));
        bits_ = widthBits(w) + ybits_;
        mask_ = tapMask(bits_);
        length_ = (uint64_t{1} << bits_) - 1;
    }

    bool valid() const { return valid_; }
    bool done() const { return done_; }
    int bits() const { return bits_; }
    uint32_t mask() const { return mask_; }
    /// Number of states in one full cycle, 2^bits - 1.
    uint64_t length() const { return length_; }
    uint64_t steps() const { return steps_; }
    uint64_t skipped() const { return skipped_; }
    uint64_t emitted() const { return emitted_; }

    /// Reads the current register as a pixel, then advances it once. After the full cycle the fade is done.
    Step next() {
        if (done_) return {};
        Step out;
        const uint32_t x = state_ >> ybits_;
        const uint32_t row = state_ & ((uint32_t{1} << ybits_) - 1U);
        if (row >= 1 && row <= static_cast<uint32_t>(h_) && x < static_cast<uint32_t>(w_)) {
            out = {static_cast<int>(x), static_cast<int>(row) - 1, true};
            ++emitted_;
        } else {
            ++skipped_;
        }
        ++steps_;
        state_ = lfsrNext(state_, mask_);
        if (state_ == 1) done_ = true;
        return out;
    }

private:
    static int widthBits(int w) {
        const int bits = bitWidth(static_cast<uint32_t>(w - 1));
        return bits < 1 ? 1 : bits;
    }

    int w_ = 0;
    int h_ = 0;
    int ybits_ = 0;
    int bits_ = 0;
    uint32_t mask_ = 0;
    uint32_t state_ = 1;
    uint64_t length_ = 0;
    uint64_t steps_ = 0;
    uint64_t skipped_ = 0;
    uint64_t emitted_ = 0;
    bool valid_ = false;
    bool done_ = false;
};

/// xorshift64*, small and good enough for picking pixels. A zero seed is replaced, zero would lock the generator.
class Rng {
public:
    explicit Rng(uint64_t seed) : s_(seed != 0 ? seed : 0x9E3779B97F4A7C15ULL) {}

    uint32_t next() {
        s_ ^= s_ >> 12;
        s_ ^= s_ << 25;
        s_ ^= s_ >> 27;
        return static_cast<uint32_t>((s_ * 0x2545F4914F6CDD1DULL) >> 32);
    }

    /// Uniform in [0, n), 0 for n == 0. Multiply and shift instead of a modulo, so no bias towards small values.
    uint32_t below(uint32_t n) { return static_cast<uint32_t>((static_cast<uint64_t>(next()) * n) >> 32); }

private:
    uint64_t s_;
};

/// The obvious version: draw a random pixel, look at the flag, draw again if it was taken. Every try counts, a try on
/// a pixel that is already done is wasted. Needs one flag per pixel.
class NaiveFade {
public:
    NaiveFade(int w, int h, uint64_t seed)
        : w_(w < 1 ? 1 : w), h_(h < 1 ? 1 : h), filled_(static_cast<std::size_t>(w_) * static_cast<std::size_t>(h_), 0), rng_(seed) {}

    bool done() const { return covered_ == filled_.size(); }
    uint64_t tries() const { return tries_; }
    uint64_t wasted() const { return wasted_; }
    uint32_t covered() const { return covered_; }
    std::size_t flagCount() const { return filled_.size(); }

    /// One try. visible is true when the drawn pixel was still open.
    Step next() {
        if (done()) return {};
        const int x = static_cast<int>(rng_.below(static_cast<uint32_t>(w_)));
        const int y = static_cast<int>(rng_.below(static_cast<uint32_t>(h_)));
        ++tries_;
        uint8_t& flag = filled_[static_cast<std::size_t>(y) * static_cast<std::size_t>(w_) + static_cast<std::size_t>(x)];
        if (flag != 0) {
            ++wasted_;
            return {x, y, false};
        }
        flag = 1;
        ++covered_;
        return {x, y, true};
    }

private:
    int w_;
    int h_;
    std::vector<uint8_t> filled_;
    Rng rng_;
    uint64_t tries_ = 0;
    uint64_t wasted_ = 0;
    uint32_t covered_ = 0;
};

/// 1234567 -> "1,234,567", built by hand so the output does not depend on the locale.
inline std::string groupDigits(uint64_t value) {
    const std::string plain = std::to_string(value);
    std::string out;
    const std::size_t n = plain.size();
    for (std::size_t i = 0; i < n; ++i) {
        out += plain[i];
        const std::size_t left = n - 1 - i;
        if (left > 0 && left % 3 == 0) out += ',';
    }
    return out;
}

/// part / total in tenths of a percent, rounded down. 0 when total is 0.
inline uint32_t percentTenths(uint64_t part, uint64_t total) {
    if (total == 0) return 0;
    return static_cast<uint32_t>(part * 1000 / total);
}

/// 713 -> "71.3", 1000 -> "100.0".
inline std::string formatTenths(uint32_t tenths) { return std::to_string(tenths / 10) + "." + std::to_string(tenths % 10); }

}  // namespace ff
