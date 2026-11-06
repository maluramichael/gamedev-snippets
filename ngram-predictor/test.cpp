// Tests for the n-gram predictor logic. Plain asserts on hand-computed cases, no framework, no SDL.
// Moves are written R (rock), P (paper), S (scissors); R P R P R P is a sequence of six moves.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "logic.hpp"

using namespace ngp;

namespace {

bool near(double a, double b) { return std::fabs(a - b) < 1e-9; }

constexpr Move R = Rock;
constexpr Move P = Paper;
constexpr Move S = Scissors;

Model learned(const std::vector<Move>& moves) {
    Model m;
    for (Move mv : moves) m.observe(mv);
    return m;
}

void rulesOfTheGame() {
    // paper covers rock, scissors cut paper, rock blunts scissors
    assert(outcome(P, R) == 1);
    assert(outcome(S, P) == 1);
    assert(outcome(R, S) == 1);
    assert(outcome(R, P) == -1);
    assert(outcome(P, S) == -1);
    assert(outcome(S, R) == -1);
    assert(outcome(R, R) == 0 && outcome(P, P) == 0 && outcome(S, S) == 0);
    // the counter move always beats the move it answers
    for (int i = 0; i < kMoves; ++i) {
        const auto m = static_cast<Move>(i);
        assert(outcome(counter(m), m) == 1);
    }
    assert(counter(R) == P && counter(P) == S && counter(S) == R);
}

void emptyModelGuessesUniformly() {
    const Model m;
    for (int order = 1; order <= kMaxOrder; ++order) {
        const Prediction p = m.predict(order, 0);
        assert(p.usedOrder == 1);  // nothing seen: only the unigram can answer
        assert(p.tied == 3);
        for (int i = 0; i < kMoves; ++i) assert(near(p.prob[static_cast<std::size_t>(i)], 1.0 / 3.0));
    }
    // every tie-break seed picks one of three moves, and the seeds reach all of them
    assert(m.predict(3, 0).move == R);
    assert(m.predict(3, 1).move == P);
    assert(m.predict(3, 2).move == S);
    assert(m.predict(3, 3).move == R);
}

void bookExampleRRPRR() {
    // Game AI Pro ch. 48: "R, R, L, R, R" with L played as paper here.
    const Model m = learned({R, R, P, R, R});
    // unigram counts: R 4, P 1
    const Prediction one = m.predict(1, 0);
    assert(one.counts[0] == 4 && one.counts[1] == 1 && one.counts[2] == 0);
    assert(one.move == R && one.usedOrder == 1);
    // bigram: window R, patterns RR (2) and RP (1)
    const Prediction two = m.predict(2, 0);
    assert(two.usedOrder == 2);
    assert(two.counts[0] == 2 && two.counts[1] == 1 && two.counts[2] == 0);
    assert(two.move == R);
    // trigram: window RR, only RRP was seen
    const Prediction three = m.predict(3, 0);
    assert(three.usedOrder == 3);
    assert(three.counts[0] == 0 && three.counts[1] == 1 && three.counts[2] == 0);
    assert(three.move == P);
    // 4-gram: window PRR was never seen, so the prediction falls back to the trigram
    const Prediction four = m.predict(4, 0);
    assert(four.usedOrder == 3);
    assert(four.move == P);
}

void freeOccurrenceProbabilities() {
    // Game AI Pro 48.3.3: after R P R P R P a trigram says 60 percent R, 20 percent P, 20 percent S.
    const Model m = learned({R, P, R, P, R, P});
    const Prediction p = m.predict(3, 0);
    assert(p.usedOrder == 3);
    assert(p.counts[0] == 2 && p.counts[1] == 0 && p.counts[2] == 0);  // window R P was followed by R twice
    assert(near(p.prob[0], 3.0 / 5.0));
    assert(near(p.prob[1], 1.0 / 5.0));
    assert(near(p.prob[2], 1.0 / 5.0));
    assert(near(p.prob[0] + p.prob[1] + p.prob[2], 1.0));
    assert(p.move == R && p.tied == 1);
}

void tieBreakOnlyAmongTheBest() {
    // bigram window R is followed by R twice and by P once: a clear winner, the seed does not matter
    const Model m = learned({R, R, R, P, R});
    for (unsigned seed = 0; seed < 5; ++seed) {
        const Prediction p = m.predict(2, seed);
        assert(p.counts[0] == 2 && p.counts[1] == 1);
        assert(p.tied == 1 && p.move == R);
    }
    // unigram counts R 1, P 1, S 0: two candidates, scissors is never picked
    const Model t = learned({R, P});
    assert(t.predict(1, 0).tied == 2);
    assert(t.predict(1, 0).move == R);
    assert(t.predict(1, 1).move == P);
    assert(t.predict(1, 2).move == R);
    assert(t.predict(1, 3).move == P);
}

void historyIsBounded() {
    Model m;
    for (int i = 0; i < 1000; ++i) m.observe(static_cast<Move>(i % kMoves));
    assert(m.rounds() == 1000);
    assert(m.recent().size() <= static_cast<std::size_t>(kMaxOrder));
    // every move completed one unigram pattern
    std::uint32_t total = 0;
    for (std::uint32_t c : m.predict(1, 0).counts) total += c;
    assert(total == 1000);
    // 1000 moves complete 997 patterns of length 4
    std::uint32_t patterns = 0;
    for (const Model::Row& row : m.rows(4, 100)) patterns += row.total;
    assert(patterns == 997);
}

// Plays the sequence against a model of the given order and counts how often the prediction was right,
// starting at round `from`.
int hitsFrom(const std::vector<Move>& seq, int order, std::size_t from) {
    Model m;
    int hits = 0;
    for (std::size_t i = 0; i < seq.size(); ++i) {
        if (i >= from && m.predict(order, 0).move == seq[i]) ++hits;
        m.observe(seq[i]);
    }
    return hits;
}

void loopNeedsOrderThree() {
    std::vector<Move> seq;
    for (int i = 0; i < 10; ++i) seq.insert(seq.end(), {R, R, P, S});
    // rounds 8 to 39 are eight full periods
    assert(hitsFrom(seq, 3, 8) == 32);  // window of two moves is unambiguous: RR->P, RP->S, PS->R, SR->R
    assert(hitsFrom(seq, 4, 8) == 32);
    // the bigram sees "R" followed by R or P and can only pick one: it gets the R after S and the first R
    // (a tie between R and P goes to R: right on R after S, R after R and S after P, wrong on P after R)
    assert(hitsFrom(seq, 2, 8) == 24);
    // raw frequency says R every time and is right on the two Rs of each period
    assert(hitsFrom(seq, 1, 8) == 16);
}

void cycleNeedsOrderTwo() {
    std::vector<Move> seq;
    for (int i = 0; i < 12; ++i) seq.insert(seq.end(), {R, P, S});
    // 36 moves, rounds 4 to 35 are all predicted
    assert(hitsFrom(seq, 2, 4) == 32);
    // raw frequency: counts are level after every third move, a tie goes to R, which is right once per cycle
    assert(hitsFrom(seq, 1, 4) == 10);
}

void rollingHitRate() {
    Tracker t;
    assert(t.rounds() == 0 && near(t.cumulative(), 0.0) && near(t.rolling(), 0.0));  // no division by zero
    for (int i = 0; i < 10; ++i) t.record(false);
    for (int i = 0; i < 20; ++i) t.record(true);
    assert(t.rounds() == 30 && t.hits() == 20);
    assert(near(t.cumulative(), 20.0 / 30.0));
    assert(near(t.rolling(), 1.0));  // the last 20 rounds were all hits
    t.record(false);
    assert(near(t.rolling(), 19.0 / 20.0));
    // the curve keeps one value per round and stays bounded
    for (int i = 0; i < 500; ++i) t.record(i % 2 == 0);
    assert(t.curve().size() == static_cast<std::size_t>(Tracker::kCurve));
    assert(near(t.rolling(), 0.5));
    t.clear();
    assert(t.rounds() == 0 && t.curve().empty() && near(t.rolling(), 0.0));
}

void rowsOfTheTable() {
    // moves: R P R R P S  -> trigram patterns RPR, PRR, RRP, RPS
    const Model m = learned({R, P, R, R, P, S});
    // order 1: a single row for the empty window
    const auto one = m.rows(1, 6);
    assert(one.size() == 1 && one[0].current && one[0].total == 6);
    // order 3: current window is the last two moves P S, never seen as a window; it is listed first anyway
    const auto rows = m.rows(3, 6);
    assert(!rows.empty() && rows[0].current);
    assert(rows[0].window == 1 * 3 + 2 && rows[0].total == 0);
    // seen windows follow, most seen first: RP appeared twice (RPR, RPS), then PR (1) and RR (1) by code
    assert(rows.size() == 4);
    assert(rows[1].window == 0 * 3 + 1 && rows[1].total == 2);
    assert(rows[2].window == 0 * 3 + 0 && rows[2].total == 1);  // RR has the smaller code than PR
    assert(rows[3].window == 1 * 3 + 0 && rows[3].total == 1);
    // the limit cuts the list but keeps the current window
    const auto cut = m.rows(3, 2);
    assert(cut.size() == 2 && cut[0].current && cut[1].window == 0 * 3 + 1);
    // too little history: no current window at all
    const Model young = learned({R});
    for (const Model::Row& row : young.rows(3, 6)) assert(!row.current);
    assert(young.rows(3, 6).empty());
    // letters of a window
    assert(windowLetters(3, 1 * 3 + 2) == "PS");
    assert(windowLetters(2, 2) == "S");
    assert(windowLetters(1, 0).empty());
}

void aiMistakes() {
    // a roll below the rate is a mistake, so a rate of 0 never errs and a rate of 1 always does
    assert(aiMove(R, 0.0, 0.0, S) == P);
    assert(aiMove(R, 0.0, 0.999, S) == P);
    assert(aiMove(R, 1.0, 0.999, S) == S);
    assert(aiMove(R, 0.25, 0.2499, S) == S);
    assert(aiMove(R, 0.25, 0.25, S) == P);  // boundary: equal to the rate is not a mistake
}

void randomNumbers() {
    Rng a(42);
    Rng b(42);
    for (int i = 0; i < 50; ++i) assert(a.nextU32() == b.nextU32());
    Rng zero(0);  // a zero seed must not lock the generator
    assert(zero.nextU32() != zero.nextU32());
    Rng c(7);
    double sum = 0.0;
    for (int i = 0; i < 2000; ++i) {
        const double f = c.nextFloat();
        assert(f >= 0.0 && f < 1.0);
        sum += f;
    }
    assert(sum / 2000.0 > 0.45 && sum / 2000.0 < 0.55);
}

void simulatedPlayers() {
    Rng rng(1);
    SimPlayer loop{Habit::Loop, 0.0};
    const Move expect[] = {R, R, P, S, R, R, P, S, R};
    for (Move e : expect) assert(loop.next(rng) == e);
    SimPlayer cycle{Habit::Cycle, 0.0};
    const Move cyc[] = {R, P, S, R, P, S, R};
    for (Move e : cyc) assert(cycle.next(rng) == e);
    // biased: half rock, three tenths paper, two tenths scissors (thresholds checked directly)
    assert(biasedMove(0.0) == R && biasedMove(0.4999) == R);
    assert(biasedMove(0.5) == P && biasedMove(0.7999) == P);
    assert(biasedMove(0.8) == S && biasedMove(0.9999) == S);
    // slips: with a slip rate of 1 the loop is gone, with 0 it is exact (checked above); the step still advances
    SimPlayer slippy{Habit::Loop, 1.0};
    int differs = 0;
    Rng rng2(5);
    const Move loopSeq[] = {R, R, P, S};
    for (int i = 0; i < 200; ++i) {
        if (slippy.next(rng2) != loopSeq[i % 4]) ++differs;
    }
    assert(differs > 80 && differs < 160);  // random moves match the loop one time in three
    // over many draws the biased player shows its distribution
    SimPlayer biased{Habit::Biased, 0.0};
    Rng rng3(9);
    int counts[3] = {0, 0, 0};
    for (int i = 0; i < 6000; ++i) ++counts[biased.next(rng3)];
    assert(counts[0] > 2700 && counts[0] < 3300);
    assert(counts[1] > 1500 && counts[1] < 2100);
    assert(counts[2] > 900 && counts[2] < 1500);
}

void gameRound() {
    Game g(11);
    g.order = 2;
    g.mistake = 0.0;
    SimPlayer cycle{Habit::Cycle, 0.0};
    Rng unused(0);
    int aiWins = 0;
    for (int i = 0; i < 30; ++i) {
        const RoundResult r = g.play(cycle.next(unused));
        assert(r.hit == (r.prediction.move == r.player));
        assert(r.outcome == outcome(r.player, r.ai));
        assert(!r.slipped);
        // from the fifth round on every transition of the cycle has been seen once
        if (i >= 4) {
            assert(r.hit);
            assert(r.outcome == -1);
            ++aiWins;
        }
    }
    assert(aiWins == 26);
    assert(g.tracker.rounds() == 30);
    assert(g.score.player + g.score.draws + g.score.ai == 30);
    assert(g.score.ai >= 26);
    // new statistics keep the learned counts: the cycle is still known
    g.resetStats();
    assert(g.tracker.rounds() == 0 && g.tracker.curve().empty());
    assert(g.score.player + g.score.draws + g.score.ai == 0);
    assert(g.model.rounds() == 30 && g.model.predict(2, 0).tied == 1);
    // settings survive a reset, the learned counts do not
    g.reset();
    assert(g.order == 2 && g.tracker.rounds() == 0 && g.model.rounds() == 0);
    assert(g.score.player + g.score.draws + g.score.ai == 0);
    assert(g.model.predict(2, 0).tied == 3);
}

void gameIsDeterministic() {
    auto run = [] {
        Game g(99);
        g.order = 3;
        g.mistake = 0.25;
        SimPlayer loop{Habit::Loop, 0.12};
        Rng input(3);
        std::string log;
        for (int i = 0; i < 60; ++i) {
            const RoundResult r = g.play(loop.next(input));
            log += static_cast<char>('0' + r.ai);
            log += static_cast<char>('0' + r.player);
        }
        return log;
    };
    assert(run() == run());
}

void mistakesLowerTheAiWinRate() {
    auto aiWins = [](double mistake) {
        Game g(21);
        g.order = 3;
        g.mistake = mistake;
        SimPlayer loop{Habit::Loop, 0.0};
        Rng unused(0);
        int wins = 0;
        for (int i = 0; i < 400; ++i) {
            if (g.play(loop.next(unused)).outcome == -1) ++wins;
        }
        return wins;
    };
    const int sharp = aiWins(0.0);
    const int sloppy = aiWins(0.5);
    assert(sharp > 380);  // learned after a handful of rounds
    assert(sloppy < sharp - 80);
}

}  // namespace

int main() {
    rulesOfTheGame();
    emptyModelGuessesUniformly();
    bookExampleRRPRR();
    freeOccurrenceProbabilities();
    tieBreakOnlyAmongTheBest();
    historyIsBounded();
    loopNeedsOrderThree();
    cycleNeedsOrderTwo();
    rollingHitRate();
    rowsOfTheTable();
    aiMistakes();
    randomNumbers();
    simulatedPlayers();
    gameRound();
    gameIsDeterministic();
    mistakesLowerTheAiWinRate();
    std::puts("ngram-predictor logic: all asserts passed");
    return 0;
}
