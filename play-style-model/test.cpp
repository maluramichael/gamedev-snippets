// Tests for the play style model. Plain asserts on hand-computed cases, no framework, no SDL.
// Style order everywhere: Fighter, Tactician, Storyteller, Role-player, Power gamer.
// The first two numbers come from "Techniques for AI-Driven Experience Management in Interactive Narratives"
// (Game AI Pro 2, chapter 42): player (F 0.9, M 0.2, S 0.1, T 0.4, P 0.3), Grendel scores 0.81, the fairy 0.09.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdio>

#include "logic.hpp"

using namespace psm;

namespace {

constexpr int kGrendel = 0;
constexpr int kFairy = 1;
constexpr int kAmbush = 2;
constexpr int kBall = 3;
constexpr int kLoot = 4;
constexpr int kFight = 0;
constexpr int kStory = 2;

bool near(double a, double b) { return std::fabs(a - b) < 1e-9; }

Profile afterMoves(Profile p, int move, int times) {
    for (int i = 0; i < times; ++i) p = learn(p, kMovesTable[static_cast<std::size_t>(move)].update);
    return p;
}

void chapterNumbers() {
    // F 0.9 x 0.9 = 0.81 and S 0.1 x 0.9 = 0.09, everything else multiplies with 0.
    assert(near(dot(kChapterProfile, kCandidatesTable[kGrendel].fit), 0.81));
    assert(near(dot(kChapterProfile, kCandidatesTable[kFairy].fit), 0.09));
    const Scores s = scoreByProfile(kChapterProfile);
    assert(near(s[kGrendel], 0.81) && near(s[kFairy], 0.09));
}

void dotBasics() {
    const Profile a{1, 0, 0, 0, 0};
    const Profile b{0, 1, 0, 0, 0};
    assert(near(dot(a, b), 0.0));  // orthogonal
    assert(near(dot(a, a), 1.0));
    assert(near(dot(Profile{}, kChapterProfile), 0.0));
    assert(near(dot(Profile{1, 2, 3, 4, 5}, Profile{1, 1, 1, 1, 1}), 15.0));
}

void tableShape() {
    // The chapter's two events are pure, the others are mixed, every annotation sums to 0.9 so that no event wins by mass alone.
    const Profile grendel{0.9, 0, 0, 0, 0};
    const Profile fairy{0, 0, 0.9, 0, 0};
    for (int i = 0; i < kStyles; ++i) {
        assert(near(kCandidatesTable[kGrendel].fit[static_cast<std::size_t>(i)], grendel[static_cast<std::size_t>(i)]));
        assert(near(kCandidatesTable[kFairy].fit[static_cast<std::size_t>(i)], fairy[static_cast<std::size_t>(i)]));
    }
    for (const Candidate& c : kCandidatesTable) {
        double sum = 0;
        for (double v : c.fit) sum += v;
        assert(near(sum, 0.9));
    }
    // Move i belongs to style i: its own component is the biggest, 0.3 (the chapter raises the fighter by 0.3 per kill).
    for (std::size_t i = 0; i < kMovesTable.size(); ++i) {
        assert(near(kMovesTable[i].update[i], 0.3));
        for (std::size_t j = 0; j < kStyles; ++j) assert(kMovesTable[i].update[j] <= kMovesTable[i].update[i] + 1e-12);
    }
}

void learning() {
    const Profile fight = kMovesTable[kFight].update;  // F 0.3, P 0.1
    const Profile once = learn(Profile{}, fight);
    assert(near(once[0], 0.3) && near(once[4], 0.1) && near(once[1], 0.0));
    // Clamped to 1, three fights from 0.9 do not run away.
    const Profile many = afterMoves(kChapterProfile, kFight, 5);
    assert(near(many[0], 1.0));
    assert(near(many[4], 0.8));  // 0.3 + 5 x 0.1, not clamped yet
    // A negative update clamps at 0.
    assert(near(learn(Profile{0.1, 0, 0, 0, 0}, Profile{-0.5, 0, 0, 0, 0})[0], 0.0));
    // The argument stays untouched (by value).
    Profile p{0.2, 0.2, 0.2, 0.2, 0.2};
    (void)learn(p, fight);
    assert(near(p[0], 0.2));
}

void forgetting() {
    const Profile p{0.8, 0.4, 0.0, 0.2, 1.0};
    const Profile half = forget(p, kNeutral, 6.0, 6.0);
    for (std::size_t i = 0; i < kStyles; ++i) assert(near(half[i], p[i] / 2.0));
    const Profile quarter = forget(p, kNeutral, 6.0, 12.0);
    assert(near(quarter[0], 0.2) && near(quarter[4], 0.25));
    // dt = 0 and a negative dt change nothing.
    for (std::size_t i = 0; i < kStyles; ++i) {
        assert(near(forget(p, kNeutral, 6.0, 0.0)[i], p[i]));
        assert(near(forget(p, kNeutral, 6.0, -1.0)[i], p[i]));
    }
    // Two half steps are one step: the result does not depend on the frame rate.
    const Profile two = forget(forget(p, kNeutral, 6.0, 1.5), kNeutral, 6.0, 1.5);
    const Profile one = forget(p, kNeutral, 6.0, 3.0);
    for (std::size_t i = 0; i < kStyles; ++i) assert(near(two[i], one[i]));
    // A neutral value other than 0 pulls from both sides.
    const Profile mid{0.5, 0.5, 0.5, 0.5, 0.5};
    const Profile pulled = forget(Profile{1.0, 0.0, 0.5, 0.5, 0.5}, mid, 6.0, 6.0);
    assert(near(pulled[0], 0.75) && near(pulled[1], 0.25) && near(pulled[2], 0.5));
    // A zero or negative half-life is "forget at once", not a division by zero.
    const Profile gone = forget(p, kNeutral, 0.0, 1.0);
    for (double v : gone) assert(near(v, 0.0));
    // After a long time everything is neutral.
    const Profile late = forget(p, mid, 6.0, 600.0);
    for (double v : late) assert(near(v, 0.5));
}

void choosing() {
    // The chapter's player meets Grendel.
    Choice c = choose(scoreByProfile(kChapterProfile));
    assert(c.index == kGrendel);
    assert(near(c.best, 0.81));
    // Runner-up is the ambush: T 0.6 x 0.4 + F 0.3 x 0.9 = 0.51, margin 0.30.
    assert(c.runnerUp == kAmbush);
    assert(near(c.margin, 0.81 - 0.51));
}

void storyTurnsItAround() {
    // Story moves raise S by 0.3 and the role-player by 0.1. After two the fairy still loses (0.63 against 0.81),
    // after three S is clamped at 1.0 and the fairy wins with 0.9.
    const Profile two = afterMoves(kChapterProfile, kStory, 2);
    assert(near(two[2], 0.7));
    assert(near(scoreByProfile(two)[kFairy], 0.63));
    assert(choose(scoreByProfile(two)).index == kGrendel);
    const Profile three = afterMoves(kChapterProfile, kStory, 3);
    assert(near(three[2], 1.0) && near(three[3], 0.5));
    const Scores s = scoreByProfile(three);
    assert(near(s[kFairy], 0.9) && near(s[kGrendel], 0.81));
    const Choice c = choose(s);
    assert(c.index == kFairy);
    assert(c.runnerUp == kGrendel);
    assert(near(c.margin, 0.09));
    // The mixed events are behind: ball 0.6 x 0.5 + 0.3 x 1.0 = 0.6.
    assert(near(s[kBall], 0.6));
    assert(near(s[kLoot], 0.6 * 0.3 + 0.3 * 0.4));  // P 0.6 x 0.3, T 0.3 x 0.4
}

void lastMoveIsNaive() {
    // One story move on a fighter: the naive rule jumps to the fairy, the profile still says Grendel.
    const Profile p = afterMoves(kChapterProfile, kStory, 1);
    assert(choose(scoreByProfile(p)).index == kGrendel);
    const Scores naive = scoreByLastMove(kStory);
    assert(near(naive[kFairy], 0.9) && near(naive[kGrendel], 0.0));
    assert(choose(naive).index == kFairy);
    // The naive rule looks at the move only: fights give Grendel, whatever the profile is.
    assert(choose(scoreByLastMove(kFight)).index == kGrendel);
    // No move yet, no pick. An invalid move index too.
    assert(choose(scoreByLastMove(-1)).index == -1);
    assert(choose(scoreByLastMove(99)).index == -1);
}

void forgettingFlipsFaster() {
    // Without forgetting the chapter's fighter needs three story moves. After two half-lives of forgetting
    // the profile is (0.225, 0.1, 0.025, 0.05, 0.075), one story move gives S 0.325 and R 0.15:
    // fairy 0.2925 against Grendel 0.2025.
    const Profile old = forget(kChapterProfile, kNeutral, 6.0, 12.0);
    assert(near(old[0], 0.225) && near(old[2], 0.025));
    assert(choose(scoreByProfile(old)).index == kGrendel);  // forgetting alone keeps the ranking, it scales all values
    const Profile now = afterMoves(old, kStory, 1);
    const Scores s = scoreByProfile(now);
    assert(near(s[kFairy], 0.2925) && near(s[kGrendel], 0.2025));
    assert(choose(s).index == kFairy);
    // Without forgetting the same single move changes nothing.
    assert(choose(scoreByProfile(afterMoves(kChapterProfile, kStory, 1))).index == kGrendel);
}

void tiesAndNoSignal() {
    // No signal at all: nothing is picked, no first-in-list default.
    const Choice none = choose(scoreByProfile(Profile{}));
    assert(none.index == -1 && none.runnerUp == -1 && near(none.margin, 0.0));
    // A real tie: the lower index wins, the margin is 0 and the other one is the runner-up.
    const Choice tie = choose(Scores{0.5, 0.5, 0.1, 0.0, 0.0});
    assert(tie.index == 0 && tie.runnerUp == 1 && near(tie.margin, 0.0));
    // A single positive entry has no runner-up score above 0: margin = best.
    const Choice solo = choose(Scores{0.0, 0.0, 0.7, 0.0, 0.0});
    assert(solo.index == 2 && near(solo.margin, 0.7));
    // Negative scores are not a signal.
    assert(choose(Scores{-0.2, -0.1, 0.0, 0.0, 0.0}).index == -1);
}

void explaining() {
    // Chapter player, ambush (T 0.6, F 0.3): F contributes 0.27, T 0.24, sorted by size.
    const auto t = topTerms(kChapterProfile, kCandidatesTable[kAmbush].fit, 3);
    assert(t.size() == 2);  // zero contributions are left out
    assert(t[0].style == 0 && near(t[0].part, 0.27));
    assert(t[1].style == 1 && near(t[1].part, 0.24));
    // The cut is applied after sorting.
    const auto one = topTerms(kChapterProfile, kCandidatesTable[kAmbush].fit, 1);
    assert(one.size() == 1 && one[0].style == 0);
    // Grendel is a single term, the sum of terms equals the dot product.
    const auto g = topTerms(kChapterProfile, kCandidatesTable[kGrendel].fit, 5);
    assert(g.size() == 1 && near(g[0].part, 0.81));
    assert(topTerms(Profile{}, kCandidatesTable[kGrendel].fit, 5).empty());
    assert(topTerms(kChapterProfile, kCandidatesTable[kGrendel].fit, 0).empty());
}

}  // namespace

int main() {
    chapterNumbers();
    dotBasics();
    tableShape();
    learning();
    forgetting();
    choosing();
    storyTurnsItAround();
    lastMoveIsNaive();
    forgettingFlipsFaster();
    tiesAndNoSignal();
    explaining();
    std::puts("play-style-model: all tests passed");
    return 0;
}
