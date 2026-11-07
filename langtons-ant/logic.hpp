// Langton's ant: the logic of the demo, no SDL, so test.cpp can check it.
//
// An ant on a grid of colored cells. Every step it looks at the color under it, turns (the rule says left or right for each
// color), changes that cell to the next color and moves one cell forward. With two colors (white: turn right, black: turn left)
// that is Langton's ant. A rule is written as the letters in color order: "RL" is the classic ant, "RLLR" has four colors.
//
// THE GRID IS A TORUS: leaving on one side means coming back on the opposite one. Starting in the middle, the ant behaves
// like on an unbounded plane until its trail reaches the edge.
//
// HEADING: 0 = up (y shrinks), 1 = right, 2 = down, 3 = left. y grows downward, like on screen.
//
// THREE PHASES, found by measuring and not by a formula:
//   * symmetry: the colors are the same under a point or mirror reflection around the start cell (kRot180 and so on).
//               A state counts as symmetric if it was symmetric within the last kSymmetryGap steps.
//   * chaos:    nothing like that.
//   * highway:  the turns repeat with a period p (up to kLongestPeriod) for at least three periods and kMinRun steps, and one period moves
//               the ant by a fixed offset without turning it around in the end. The classic ant gets one with p = 104
//               that starts at step 9977 and moves (-2, +2) per period. It is recognized 4 periods later (3 repeats need
//               3 * 104 matching turns after the first period), and the reported onset is where the repetition began.
//
// SYMMETRY WITHOUT LOOKING AT THE WHOLE GRID: each reflection T keeps a counter of cells whose color differs from the color
// at T(cell). A step changes exactly one cell, so only two terms of that sum change and the update is O(1). The field is
// symmetric under T when the counter is zero.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace ant {

constexpr int kMaxColors = 16;
constexpr int kLongestPeriod = 128;  // the detector looks for highways with a period up to this
constexpr int kPeriodsNeeded = 3;
// A repetition must also be at least this long. Short staircases happen all the time: the classic ant draws a six-step
// one (period 2) at step 279, and RLLR walks a 98-step diagonal (period 4) around step 22761. Neither is a highway.
constexpr int kMinRun = 150;
constexpr long kSymmetryGap = 256;
constexpr long kBucket = 64;                // steps per entry of the phase log
constexpr std::size_t kLogBuckets = 384;    // entries kept, 384 * 64 = 24576 steps

enum Symmetry : unsigned { kRot180 = 1, kFlipX = 2, kFlipY = 4, kDiag = 8, kAnti = 16, kRot90 = 32 };
constexpr int kSymmetryCount = 6;

enum class Phase : std::uint8_t { Symmetry, Chaos, Highway };

struct Rule {
    std::string letters;  // 'L' or 'R' per color
    int colors() const { return static_cast<int>(letters.size()); }
    bool turnsRight(int color) const { return letters[static_cast<std::size_t>(color % colors())] == 'R'; }
};

/// 2 to kMaxColors letters, each L or R. Anything else is no rule.
inline std::optional<Rule> parseRule(std::string_view text) {
    if (text.size() < 2 || text.size() > static_cast<std::size_t>(kMaxColors)) return std::nullopt;
    for (char c : text) {
        if (c != 'L' && c != 'R') return std::nullopt;
    }
    return Rule{std::string(text)};
}

struct Highway {
    int period;
    long onset;  // step where the repetition began
    int dx;      // offset of the ant per period
    int dy;
};

class World {
public:
    /// size is clamped to 2..1024. The ant starts in the middle, facing up, on an empty grid.
    World(Rule rule, int size) : rule_(std::move(rule)), size_(std::clamp(size, 2, 1024)) {
        cells_.assign(static_cast<std::size_t>(size_) * static_cast<std::size_t>(size_), 0);
        ax_ = ay_ = size_ / 2;
        posRing_[0] = Pos{};
    }

    void step() {
        const std::size_t idx = static_cast<std::size_t>(ay_) * static_cast<std::size_t>(size_) + static_cast<std::size_t>(ax_);
        const int color = cells_[idx];
        const bool right = rule_.turnsRight(color);
        const int next = (color + 1) % rule_.colors();

        std::array<long, kSymmetryCount> before{};
        for (int t = 0; t < kSymmetryCount; ++t) before[static_cast<std::size_t>(t)] = pairMismatches(t, ax_, ay_);
        cells_[idx] = static_cast<std::uint8_t>(next);
        for (int t = 0; t < kSymmetryCount; ++t) mismatch_[static_cast<std::size_t>(t)] += pairMismatches(t, ax_, ay_) - before[static_cast<std::size_t>(t)];
        if (color == 0 && next != 0) ++colored_;
        if (color != 0 && next == 0) --colored_;

        dir_ = (dir_ + (right ? 1 : 3)) & 3;
        rot_ += right ? 1 : -1;
        static constexpr int kDx[4] = {0, 1, 0, -1};
        static constexpr int kDy[4] = {-1, 0, 1, 0};
        ux_ += kDx[dir_];
        uy_ += kDy[dir_];
        ax_ = wrap(ax_ + kDx[dir_]);
        ay_ = wrap(ay_ + kDy[dir_]);
        ++steps_;

        track(right);
        if (symmetries() != 0) lastSymmetric_ = steps_;
        if (steps_ % kBucket == 0 && log_.size() < kLogBuckets) log_.push_back(phase());
    }

    void run(long n) {
        for (long i = 0; i < n; ++i) step();
    }

    long steps() const { return steps_; }
    int size() const { return size_; }
    int antX() const { return ax_; }
    int antY() const { return ay_; }
    int antDir() const { return dir_; }
    const Rule& rule() const { return rule_; }
    /// Cells that are not color 0.
    long colored() const { return colored_; }
    int colorAt(int x, int y) const { return cells_[index(wrap(x), wrap(y))]; }
    /// Row-major colors, size * size entries.
    const std::vector<std::uint8_t>& cells() const { return cells_; }
    /// Bit set of Symmetry values the colors have right now. An empty grid has all of them.
    unsigned symmetries() const {
        unsigned bits = 0;
        for (int t = 0; t < kSymmetryCount; ++t) {
            if (mismatch_[static_cast<std::size_t>(t)] == 0) bits |= 1U << t;
        }
        return bits;
    }
    /// The last step (0 = start) where the field was symmetric.
    long lastSymmetric() const { return lastSymmetric_; }

    /// The smallest period with a repetition of at least three periods that moves the ant without ending up turned.
    std::optional<Highway> highway() const {
        for (int p = 2; p <= kLongestPeriod; ++p) {
            const int run = run_[static_cast<std::size_t>(p)];
            if (run < std::max(kPeriodsNeeded * p, kMinRun)) continue;
            const Pos& a = posRing_[static_cast<std::size_t>((steps_ - p) & kRingMask)];
            const Pos& b = posRing_[static_cast<std::size_t>(steps_ & kRingMask)];
            const long dx = b.x - a.x;
            const long dy = b.y - a.y;
            const long turned = ((b.rot - a.rot) % 4 + 4) % 4;
            if (turned != 0 || (dx == 0 && dy == 0)) continue;
            return Highway{p, steps_ - run - p, static_cast<int>(dx), static_cast<int>(dy)};
        }
        return std::nullopt;
    }

    Phase phase() const {
        if (highway().has_value()) return Phase::Highway;
        return steps_ - lastSymmetric_ <= kSymmetryGap ? Phase::Symmetry : Phase::Chaos;
    }

    /// Entry i is the phase after step kBucket * (i + 1), at most kLogBuckets entries.
    const std::vector<Phase>& phaseLog() const { return log_; }

private:
    struct Pos {
        long x = 0;  // unwrapped: the torus does not fold the ant's path back
        long y = 0;
        long rot = 0;  // right turns minus left turns
    };

    static constexpr long kRingMask = 255;  // ring buffers hold 256 entries, more than kLongestPeriod + 1

    int wrap(int v) const { return ((v % size_) + size_) % size_; }
    std::size_t index(int x, int y) const { return static_cast<std::size_t>(y) * static_cast<std::size_t>(size_) + static_cast<std::size_t>(x); }
    int at(int x, int y) const { return cells_[index(x, y)]; }

    /// Where reflection t sends the cell (x, y). All reflections are around the start cell (size / 2, size / 2).
    std::pair<int, int> image(int t, int x, int y, bool inverse) const {
        const int c = size_ / 2;
        const int dx = x - c;
        const int dy = y - c;
        int fx = 0;
        int fy = 0;
        switch (t) {
            case 0: fx = c - dx; fy = c - dy; break;                                    // kRot180
            case 1: fx = c - dx; fy = c + dy; break;                                    // kFlipX
            case 2: fx = c + dx; fy = c - dy; break;                                    // kFlipY
            case 3: fx = c + dy; fy = c + dx; break;                                    // kDiag
            case 4: fx = c - dy; fy = c - dx; break;                                    // kAnti
            default: inverse ? (fx = c + dy, fy = c - dx) : (fx = c - dy, fy = c + dx); break;  // kRot90 and its inverse
        }
        return {wrap(fx), wrap(fy)};
    }

    /// The two terms of the mismatch sum that contain cell (x, y): its own color against its image, and the color of the
    /// cell that is sent onto it against its color.
    long pairMismatches(int t, int x, int y) const {
        const auto [fx, fy] = image(t, x, y, false);
        const auto [bx, by] = image(t, x, y, true);
        return (at(x, y) != at(fx, fy) ? 1 : 0) + (at(bx, by) != at(x, y) ? 1 : 0);
    }

    /// Feeds the turn of the step that just ended into the period detector.
    void track(bool right) {
        const long n = steps_;  // turns so far
        turnRing_[static_cast<std::size_t>((n - 1) & kRingMask)] = right ? 1 : 0;
        posRing_[static_cast<std::size_t>(n & kRingMask)] = Pos{ux_, uy_, rot_};
        for (int p = 1; p <= kLongestPeriod; ++p) {
            auto& run = run_[static_cast<std::size_t>(p)];
            if (n - 1 < p) {
                run = 0;
                continue;
            }
            const bool same = turnRing_[static_cast<std::size_t>((n - 1) & kRingMask)] == turnRing_[static_cast<std::size_t>((n - 1 - p) & kRingMask)];
            run = same ? static_cast<std::uint16_t>(std::min<int>(run + 1, 65535)) : static_cast<std::uint16_t>(0);
        }
    }

    Rule rule_;
    int size_;
    std::vector<std::uint8_t> cells_;
    int ax_ = 0;
    int ay_ = 0;
    int dir_ = 0;
    long ux_ = 0;
    long uy_ = 0;
    long rot_ = 0;
    long steps_ = 0;
    long colored_ = 0;
    long lastSymmetric_ = 0;
    std::array<long, kSymmetryCount> mismatch_{};
    std::array<std::uint8_t, 256> turnRing_{};
    std::array<Pos, 256> posRing_{};
    std::array<std::uint16_t, kLongestPeriod + 1> run_{};
    std::vector<Phase> log_;
};

/// 12345 as "12,345". English grouping on every system, snprintf would follow the locale.
inline std::string formatCount(long value) {
    const bool negative = value < 0;
    std::string digits = std::to_string(negative ? -value : value);
    std::string out;
    for (std::size_t i = 0; i < digits.size(); ++i) {
        if (i > 0 && (digits.size() - i) % 3 == 0) out += ',';
        out += digits[i];
    }
    return negative ? "-" + out : out;
}

/// Whole steps due after dt seconds at `rate` steps per second. The fraction stays in `acc`, so a slow rate still gets its
/// steps. A stall counts as 0.25 s at most, otherwise the first frame after a pause would run a burst of thousands of steps.
inline long stepsDue(double& acc, double rate, double dt) {
    if (!(rate > 0.0) || !(dt > 0.0)) return 0;
    acc += rate * std::min(dt, 0.25);
    const double whole = std::floor(acc);
    acc -= whole;
    return static_cast<long>(whole);
}

}  // namespace ant
