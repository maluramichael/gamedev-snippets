// Play style model, no SDL. The player is a vector of five numbers (how much they act as a fighter, tactician, storyteller,
// role-player, power gamer). Every move adds a small update vector, the game scores each candidate event with a dot product
// against that vector and shows the best one. Idea and the example numbers: "Techniques for AI-Driven Experience Management
// in Interactive Narratives", Game AI Pro 2, chapter 42.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

namespace psm {

inline constexpr int kStyles = 5;
inline constexpr int kMoves = 5;
inline constexpr int kCandidates = 5;

using Profile = std::array<double, kStyles>;
using Scores = std::array<double, kCandidates>;

// Order everywhere: Fighter, Tactician, Storyteller, Role-player, Power gamer. Move i belongs to style i.
inline constexpr std::array<const char*, kStyles> kStyleNames{"Fighter", "Tactician", "Storyteller", "Role-player", "Power gamer"};
inline constexpr std::array<char, kStyles> kStyleLetters{'F', 'T', 'S', 'R', 'P'};

struct Move {
    const char* name;
    Profile update;  // added to the profile when the player does this
};

struct Candidate {
    const char* name;
    Profile fit;  // how well the event suits each style, every annotation sums to 0.9
};

inline constexpr std::array<Move, kMoves> kMovesTable{{
    {"Fight", {0.3, 0.0, 0.0, 0.0, 0.1}},
    {"Tactics", {0.0, 0.3, 0.0, 0.0, 0.1}},
    {"Story", {0.0, 0.0, 0.3, 0.1, 0.0}},
    {"Roleplay", {0.0, 0.0, 0.1, 0.3, 0.0}},
    {"Power-up", {0.0, 0.1, 0.0, 0.0, 0.3}},
}};

// The first two are the chapter's events. The others are mixed so that the list has more than one right answer.
inline constexpr std::array<Candidate, kCandidates> kCandidatesTable{{
    {"Grendel attacks", {0.9, 0.0, 0.0, 0.0, 0.0}},
    {"Magic fairy", {0.0, 0.0, 0.9, 0.0, 0.0}},
    {"Ambush at the ford", {0.3, 0.6, 0.0, 0.0, 0.0}},
    {"Masked ball", {0.0, 0.0, 0.3, 0.6, 0.0}},
    {"Rare loot cache", {0.0, 0.3, 0.0, 0.0, 0.6}},
}};

// The player from the chapter: F 0.9, M 0.2, S 0.1, T 0.4, P 0.3 (in our order F, T, S, R, P).
inline constexpr Profile kChapterProfile{0.9, 0.4, 0.1, 0.2, 0.3};
inline constexpr Profile kNeutral{0.0, 0.0, 0.0, 0.0, 0.0};
inline constexpr double kForgetHalfLife = 6.0;  // seconds until a profile value is halfway back to neutral

inline double dot(const Profile& a, const Profile& b) {
    double sum = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i) sum += a[i] * b[i];
    return sum;
}

/// Adds a move's update to the profile and keeps every value in [0, 1].
inline Profile learn(Profile profile, const Profile& update) {
    for (std::size_t i = 0; i < profile.size(); ++i) profile[i] = std::clamp(profile[i] + update[i], 0.0, 1.0);
    return profile;
}

/// Moves every value toward its neutral value. The remaining distance halves every `halfLife` seconds, so two steps of
/// dt/2 give the same result as one step of dt and the frame rate does not matter. halfLife <= 0 forgets at once,
/// dt <= 0 changes nothing.
inline Profile forget(Profile profile, const Profile& neutral, double halfLife, double dt) {
    if (dt <= 0.0) return profile;
    const double keep = halfLife <= 0.0 ? 0.0 : std::exp2(-dt / halfLife);
    for (std::size_t i = 0; i < profile.size(); ++i) profile[i] = neutral[i] + (profile[i] - neutral[i]) * keep;
    return profile;
}

/// The chapter's rule: score = profile . annotation, one number per candidate.
inline Scores scoreByProfile(const Profile& profile) {
    Scores s{};
    for (std::size_t i = 0; i < s.size(); ++i) s[i] = dot(profile, kCandidatesTable[i].fit);
    return s;
}

/// The naive rule: only the last move counts, an event scores by how well it fits the style of that move.
inline Scores scoreByLastMove(int move) {
    Scores s{};
    if (move < 0 || move >= kStyles) return s;
    for (std::size_t i = 0; i < s.size(); ++i) s[i] = kCandidatesTable[i].fit[static_cast<std::size_t>(move)];
    return s;
}

struct Choice {
    int index = -1;     // best candidate, -1 when no score is above 0 (no signal)
    int runnerUp = -1;  // second best, -1 when there is none
    double best = 0.0;
    double margin = 0.0;  // best minus the runner-up's score
};

/// The highest score wins, ties go to the lower index. Scores of 0 or less are no signal, so nothing is picked.
inline Choice choose(const Scores& scores) {
    Choice c;
    double best = 0.0;
    for (std::size_t i = 0; i < scores.size(); ++i) {
        if (scores[i] > best + 1e-12) {
            best = scores[i];
            c.index = static_cast<int>(i);
        }
    }
    if (c.index < 0) return c;
    c.best = best;
    double second = 0.0;
    for (std::size_t i = 0; i < scores.size(); ++i) {
        if (static_cast<int>(i) == c.index) continue;
        if (c.runnerUp < 0 || scores[i] > second + 1e-12) {
            second = scores[i];
            c.runnerUp = static_cast<int>(i);
        }
    }
    c.margin = best - std::max(second, 0.0);
    return c;
}

struct Term {
    int style;
    double part;  // profile[style] * fit[style]
};

/// The dot product written out: the biggest non-zero terms, largest first, at most n.
inline std::vector<Term> topTerms(const Profile& profile, const Profile& fit, std::size_t n) {
    std::vector<Term> terms;
    for (std::size_t i = 0; i < profile.size(); ++i) {
        const double part = profile[i] * fit[i];
        if (part > 1e-12) terms.push_back({static_cast<int>(i), part});
    }
    std::stable_sort(terms.begin(), terms.end(), [](const Term& a, const Term& b) { return a.part > b.part; });
    if (terms.size() > n) terms.resize(n);
    return terms;
}

}  // namespace psm
