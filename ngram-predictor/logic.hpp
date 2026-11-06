// N-gram prediction for rock paper scissors, no SDL.
//
// The model counts every run of N moves it has seen (N = 1 to 4). To predict, it takes the last N-1 moves as the
// window, looks at all runs that start with that window and picks the move that completed them most often.
// Following Game AI Pro ch. 48 every possible run gets one free occurrence (counts + 1), so an unseen move never has
// probability 0. On top of that a window that was never seen falls back to a shorter one (backoff), which is
// this demo's own addition: with N = 4 the first moves would otherwise be a coin flip.
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

namespace ngp {

enum Move : int { Rock = 0, Paper = 1, Scissors = 2 };

constexpr int kMoves = 3;
constexpr int kMaxOrder = 4;

/// The move that beats `m`.
constexpr Move counter(Move m) { return static_cast<Move>((static_cast<int>(m) + 1) % kMoves); }

/// +1 if `a` beats `b`, -1 if `b` beats `a`, 0 for a draw.
constexpr int outcome(Move a, Move b) {
    const int d = (static_cast<int>(a) - static_cast<int>(b) + kMoves) % kMoves;
    return d == 0 ? 0 : (d == 1 ? 1 : -1);
}

constexpr char letter(Move m) { return m == Rock ? 'R' : (m == Paper ? 'P' : 'S'); }

constexpr int pow3(int n) {
    int v = 1;
    for (int i = 0; i < n; ++i) v *= kMoves;
    return v;
}

/// A window of `order - 1` moves as letters, "PS" for the code of paper then scissors. Order 1 has no window.
inline std::string windowLetters(int order, int window) {
    std::string s(static_cast<std::size_t>(std::max(order - 1, 0)), '?');
    for (std::size_t i = s.size(); i-- > 0;) {
        s[i] = letter(static_cast<Move>(window % kMoves));
        window /= kMoves;
    }
    return s;
}

struct Prediction {
    Move move = Rock;                  // the move expected from the player
    int usedOrder = 1;                 // the order that answered, lower than asked when the window was unseen
    std::array<std::uint32_t, 3> counts{};  // how often each move followed the window
    std::array<double, 3> prob{};      // (count + 1) / (total + 3), the free occurrence of the book
    int tied = 1;                      // how many moves share the highest count
};

class Model {
public:
    struct Row {
        int window = 0;
        std::array<std::uint32_t, 3> counts{};
        std::uint32_t total = 0;
        bool current = false;  // the window the next prediction will use
    };

    Model() {
        for (int n = 1; n <= kMaxOrder; ++n) counts_[static_cast<std::size_t>(n)].assign(static_cast<std::size_t>(pow3(n)), 0);
    }

    /// Learn one move: every run of 1 to 4 moves that ends with it gets one more occurrence.
    void observe(Move m) {
        recent_.push_back(m);
        if (recent_.size() > static_cast<std::size_t>(kMaxOrder)) recent_.pop_front();
        ++rounds_;
        for (int n = 1; n <= kMaxOrder; ++n) {
            if (recent_.size() >= static_cast<std::size_t>(n)) ++counts_[static_cast<std::size_t>(n)][static_cast<std::size_t>(foldLast(n))];
        }
    }

    /// Expected next move of the player. `tieSeed` picks among moves with the same count, so the same seed gives
    /// the same answer and different seeds spread the guesses.
    Prediction predict(int order, unsigned tieSeed) const {
        order = std::clamp(order, 1, kMaxOrder);
        // Window of order - 1 moves needs that much history, otherwise start lower.
        int k = std::min<int>(order, static_cast<int>(recent_.size()) + 1);
        for (;; --k) {
            const std::array<std::uint32_t, 3> c = countsFor(k, foldLast(k - 1));
            const std::uint32_t total = c[0] + c[1] + c[2];
            if (total > 0 || k == 1) return finish(k, c, total, tieSeed);
        }
    }

    /// Rows of the count table for one order: the current window first (if there is enough history), then the
    /// windows seen most often. At most `limit` rows.
    std::vector<Row> rows(int order, std::size_t limit) const {
        order = std::clamp(order, 1, kMaxOrder);
        std::vector<Row> out;
        std::vector<Row> seen;
        const bool haveWindow = recent_.size() >= static_cast<std::size_t>(order - 1);
        const int current = haveWindow ? foldLast(order - 1) : -1;
        for (int w = 0; w < pow3(order - 1); ++w) {
            Row row;
            row.window = w;
            row.counts = countsFor(order, w);
            row.total = row.counts[0] + row.counts[1] + row.counts[2];
            row.current = w == current;
            if (row.current) {
                out.push_back(row);
            } else if (row.total > 0) {
                seen.push_back(row);
            }
        }
        std::sort(seen.begin(), seen.end(), [](const Row& a, const Row& b) { return a.total != b.total ? a.total > b.total : a.window < b.window; });
        for (const Row& row : seen) out.push_back(row);
        if (out.size() > limit) out.resize(limit);
        return out;
    }

    std::size_t rounds() const { return rounds_; }
    const std::deque<Move>& recent() const { return recent_; }

private:
    /// Base-3 code of the last `n` moves (0 for n = 0). Needs n <= recent_.size().
    int foldLast(int n) const {
        int code = 0;
        for (std::size_t i = recent_.size() - static_cast<std::size_t>(n); i < recent_.size(); ++i) code = code * kMoves + static_cast<int>(recent_[i]);
        return code;
    }

    std::array<std::uint32_t, 3> countsFor(int order, int window) const {
        const auto& table = counts_[static_cast<std::size_t>(order)];
        const auto base = static_cast<std::size_t>(window) * kMoves;
        return {table[base], table[base + 1], table[base + 2]};
    }

    static Prediction finish(int usedOrder, const std::array<std::uint32_t, 3>& c, std::uint32_t total, unsigned tieSeed) {
        Prediction p;
        p.usedOrder = usedOrder;
        p.counts = c;
        const std::uint32_t best = std::max({c[0], c[1], c[2]});
        std::array<int, 3> candidates{};
        int n = 0;
        for (int i = 0; i < kMoves; ++i) {
            p.prob[static_cast<std::size_t>(i)] = static_cast<double>(c[static_cast<std::size_t>(i)] + 1) / static_cast<double>(total + kMoves);
            if (c[static_cast<std::size_t>(i)] == best) candidates[static_cast<std::size_t>(n++)] = i;
        }
        p.tied = n;
        p.move = static_cast<Move>(candidates[tieSeed % static_cast<unsigned>(n)]);
        return p;
    }

    std::array<std::vector<std::uint32_t>, kMaxOrder + 1> counts_;  // index = order, one slot per run
    std::deque<Move> recent_;                                       // the last kMaxOrder moves, enough to form any window
    std::size_t rounds_ = 0;
};

/// Hit rate of the predictions: over everything so far, over the last 20 rounds, and as a curve for the chart.
class Tracker {
public:
    static constexpr int kRolling = 20;
    static constexpr int kCurve = 100;

    void record(bool hit) {
        ++rounds_;
        if (hit) ++hits_;
        recent_.push_back(hit);
        if (hit) ++recentHits_;
        if (recent_.size() > static_cast<std::size_t>(kRolling)) {
            if (recent_.front()) --recentHits_;
            recent_.pop_front();
        }
        curve_.push_back(rolling());
        if (curve_.size() > static_cast<std::size_t>(kCurve)) curve_.pop_front();
    }

    void clear() { *this = Tracker(); }

    std::size_t rounds() const { return rounds_; }
    std::size_t hits() const { return hits_; }
    double cumulative() const { return rounds_ == 0 ? 0.0 : static_cast<double>(hits_) / static_cast<double>(rounds_); }
    double rolling() const { return recent_.empty() ? 0.0 : static_cast<double>(recentHits_) / static_cast<double>(recent_.size()); }
    const std::deque<double>& curve() const { return curve_; }

private:
    std::size_t rounds_ = 0;
    std::size_t hits_ = 0;
    std::deque<bool> recent_;
    int recentHits_ = 0;
    std::deque<double> curve_;
};

/// Small xorshift generator, so a run is reproducible and the tests do not depend on the platform's rand().
class Rng {
public:
    explicit Rng(std::uint32_t seed = 1) : s_(seed == 0 ? 0x9E3779B9U : seed) {}

    std::uint32_t nextU32() {
        s_ ^= s_ << 13;
        s_ ^= s_ >> 17;
        s_ ^= s_ << 5;
        return s_;
    }

    /// Uniform in [0, 1).
    double nextFloat() { return static_cast<double>(nextU32() >> 8) / 16777216.0; }

private:
    std::uint32_t s_;
};

/// The move the AI plays. A roll below the mistake rate means it ignores its prediction and plays `random`.
constexpr Move aiMove(Move predicted, double mistakeRate, double roll, Move random) { return roll < mistakeRate ? random : counter(predicted); }

/// Half rock, three tenths paper, two tenths scissors, for a roll in [0, 1).
constexpr Move biasedMove(double roll) { return roll < 0.5 ? Rock : (roll < 0.8 ? Paper : Scissors); }

enum class Habit { Loop, Biased, Cycle };

/// A simulated player with a habit: a loop of four moves, a cycle of three, or a fixed bias. Every round it slips
/// with probability `slip` and plays a random move instead; the loop keeps its place when it slips.
struct SimPlayer {
    Habit habit = Habit::Loop;
    double slip = 0.12;
    std::size_t step = 0;

    Move next(Rng& rng) {
        static constexpr Move kLoop[4] = {Rock, Rock, Paper, Scissors};
        static constexpr Move kCycle[3] = {Rock, Paper, Scissors};
        const double slipRoll = rng.nextFloat();
        const std::uint32_t random = rng.nextU32();
        const double biasRoll = rng.nextFloat();
        Move m = Rock;
        switch (habit) {
            case Habit::Loop: m = kLoop[step % 4]; break;
            case Habit::Cycle: m = kCycle[step % 3]; break;
            case Habit::Biased: m = biasedMove(biasRoll); break;
        }
        ++step;
        return slipRoll < slip ? static_cast<Move>(random % static_cast<std::uint32_t>(kMoves)) : m;
    }
};

struct RoundResult {
    Prediction prediction;
    Move player = Rock;
    Move ai = Rock;
    int outcome = 0;      // seen from the player: +1 player wins, -1 AI wins
    bool hit = false;     // the prediction was right
    bool slipped = false; // the AI ignored its prediction this round
};

struct Score {
    std::size_t player = 0;
    std::size_t draws = 0;
    std::size_t ai = 0;
};

/// One game: the AI predicts the player's move, answers with its counter (or slips), then learns what was played.
struct Game {
    explicit Game(std::uint32_t seed = 1) : rng(seed) {}

    RoundResult play(Move player) {
        RoundResult r;
        r.player = player;
        // Always draw the same three numbers so a run stays reproducible whatever the settings are.
        r.prediction = model.predict(order, rng.nextU32());
        const double roll = rng.nextFloat();
        const auto random = static_cast<Move>(rng.nextU32() % static_cast<std::uint32_t>(kMoves));
        r.slipped = roll < mistake;
        r.ai = aiMove(r.prediction.move, mistake, roll, random);
        r.outcome = ngp::outcome(player, r.ai);
        r.hit = r.prediction.move == player;
        tracker.record(r.hit);
        if (r.outcome > 0) {
            ++score.player;
        } else if (r.outcome < 0) {
            ++score.ai;
        } else {
            ++score.draws;
        }
        model.observe(player);
        return r;
    }

    /// Forget everything learned; order, mistake rate and the random stream stay.
    void reset() {
        model = Model();
        resetStats();
    }

    /// Start a fresh hit rate and score but keep what the model has learned. The numbers describe one setting
    /// (order, player), so they restart when it changes.
    void resetStats() {
        tracker.clear();
        score = Score();
    }

    Model model;
    Tracker tracker;
    Score score;
    Rng rng;
    int order = 3;
    double mistake = 0.0;
};

}  // namespace ngp
