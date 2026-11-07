// Petri-net arbiter for one shared, non-shareable role (a mounted gun), no SDL.
//
// Part 1 is a generic Petri net: places hold tokens, a transition fires when every input place has at least the arc weight in
// tokens, and firing moves tokens from the inputs to the outputs. Part 2 is the net of one arbiter (Sergio Ocio Barriales,
// "Petri Nets and AI Arbitration", Game AI Pro 3, chapter 29), part 3 selects WHO gets the gun when the net fires "assign",
// and part 4 is the little world around it: agents that walk to the gun, die, and hand it on.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace pa {

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};

inline double dist(Vec2 a, Vec2 b) { return std::hypot(a.x - b.x, a.y - b.y); }

// ---------------------------------------------------------------------------------------------------------------------
// Part 1: the net. A place may appear at most once in the `in` list of a transition (and once in `out`).

struct Arc {
    std::size_t place;
    int weight;
};

struct Transition {
    std::vector<Arc> in;
    std::vector<Arc> out;
};

struct Net {
    std::vector<int> marking;  // tokens per place
    std::vector<Transition> transitions;
};

namespace detail {
inline bool enabledAt(const std::vector<int>& marking, const Transition& t) {
    for (const Arc& a : t.in) {
        if (marking[a.place] < a.weight) return false;
    }
    return true;
}

inline void apply(std::vector<int>& marking, const Transition& t) {
    for (const Arc& a : t.in) marking[a.place] -= a.weight;
    for (const Arc& a : t.out) marking[a.place] += a.weight;
}
}  // namespace detail

/// A transition with no input place is a source and always enabled.
inline bool enabled(const Net& net, std::size_t t) { return detail::enabledAt(net.marking, net.transitions[t]); }

/// Consumes the input tokens and produces the output tokens. Refuses (and changes nothing) when the transition is not enabled.
inline bool fire(Net& net, std::size_t t) {
    if (!enabled(net, t)) return false;
    detail::apply(net.marking, net.transitions[t]);
    return true;
}

/// Two enabled transitions are in conflict when firing one disables the other: they fight over the same tokens.
inline bool inConflict(const Net& net, std::size_t a, std::size_t b) {
    if (a == b || !enabled(net, a) || !enabled(net, b)) return false;
    std::vector<int> after = net.marking;
    detail::apply(after, net.transitions[a]);
    return !detail::enabledAt(after, net.transitions[b]);
}

// ---------------------------------------------------------------------------------------------------------------------
// Part 2: the arbiter net.
//
//   Spawned -> [Register] -> ReadyToAssign -(n)-> [Start] -(n)-> ReadyToUse -> [Assign] -> GunBusy -> [Incapacitated] -> GunAvailable
//                                                                   |            ^                                           |
//                                                                [Dropout]       +---------------- GunAvailable -------------+

enum Place : std::size_t { Spawned, ReadyToAssign, ReadyToUse, GunBusy, GunAvailable, kPlaceCount };
enum Trans : std::size_t { Register, Start, Assign, Incapacitated, Dropout, kTransCount };

inline constexpr int kMinAgents = 2;
inline constexpr int kMaxAgents = 10;

/// `n` agents will register. Start waits for all of them (arc weight n), Assign needs an agent AND the gun.
inline Net makeArbiterNet(int n) {
    n = std::max(n, 1);
    Net net;
    net.marking.assign(kPlaceCount, 0);
    net.marking[Spawned] = n;
    net.marking[GunAvailable] = 1;
    net.transitions.resize(kTransCount);
    net.transitions[Register] = {{{Spawned, 1}}, {{ReadyToAssign, 1}}};
    net.transitions[Start] = {{{ReadyToAssign, n}}, {{ReadyToUse, n}}};
    net.transitions[Assign] = {{{ReadyToUse, 1}, {GunAvailable, 1}}, {{GunBusy, 1}}};
    net.transitions[Incapacitated] = {{{GunBusy, 1}}, {{GunAvailable, 1}}};
    net.transitions[Dropout] = {{{ReadyToUse, 1}}, {}};
    return net;
}

// ---------------------------------------------------------------------------------------------------------------------
// Part 3: who gets the gun.

enum class Policy { Fifo, Nearest, Score };

struct Candidate {
    std::size_t id;
    int order;  // registration order, 0 registered first
    Vec2 pos;
    double accuracy;  // 0..1
};

inline constexpr double kRange = 640.0;  // distance at which closeness reaches 0
inline constexpr double kWeightCloseness = 0.6;
inline constexpr double kWeightAccuracy = 0.4;

/// Closeness (1 at the gun, 0 at kRange or farther) and accuracy, weighted 0.6 : 0.4. Higher is better.
inline double score(Vec2 pos, double accuracy, Vec2 gun) {
    const double closeness = std::clamp(1.0 - dist(pos, gun) / kRange, 0.0, 1.0);
    return kWeightCloseness * closeness + kWeightAccuracy * std::clamp(accuracy, 0.0, 1.0);
}

/// Index into `list` of the chosen candidate, nothing for an empty list. Ties go to the lower registration order.
inline std::optional<std::size_t> pick(Policy policy, const std::vector<Candidate>& list, Vec2 gun) {
    std::optional<std::size_t> best;
    double bestKey = 0.0;  // smaller is better
    for (std::size_t i = 0; i < list.size(); ++i) {
        const Candidate& c = list[i];
        double key = 0.0;
        switch (policy) {
            case Policy::Fifo: key = static_cast<double>(c.order); break;
            case Policy::Nearest: key = dist(c.pos, gun); break;
            case Policy::Score: key = -score(c.pos, c.accuracy, gun); break;
        }
        if (!best || key < bestKey || (key == bestKey && c.order < list[*best].order)) {
            best = i;
            bestKey = key;
        }
    }
    return best;
}

// ---------------------------------------------------------------------------------------------------------------------
// Part 4: the world.

inline constexpr double kFieldLeft = 40.0;
inline constexpr double kFieldRight = 720.0;
inline constexpr double kFieldTop = 64.0;
inline constexpr double kFieldBottom = 236.0;
inline constexpr double kMinSeparation = 56.0;
inline constexpr double kAccuracyMin = 0.3;
inline constexpr double kSpeed = 140.0;         // px per second
inline constexpr double kTick = 0.15;           // seconds per transition firing
inline constexpr double kManTime = 1.4;         // seconds at the gun before the enemy hits the gunner (auto-kill)
inline constexpr double kRestartDelay = 1.2;    // seconds the empty field stays before the next round
inline constexpr double kMaxDt = 0.25;

/// The round the demo starts with. With six agents its layout has three different answers (FIFO sends agent 1 over 680 px, Nearest
/// agent 6 from 84 px, Score agent 3), so the first frame already shows the problem. Found by scanning rounds, checked in test.cpp.
inline constexpr std::uint32_t kShowcaseRound = 6;

/// Small deterministic generator, the same layouts on every machine.
struct Rng {
    std::uint32_t state;
    double next() {
        state = state * 1664525U + 1013904223U;
        return static_cast<double>(state >> 8) / 16777216.0;  // 24 bits, [0, 1)
    }
};

enum class State { Spawning, Registered, Waiting, Walking, Manning, Dead };

struct Agent {
    Vec2 pos;
    double accuracy = 1.0;
    State state = State::Spawning;
};

/// `n` agents (clamped to kMinAgents..kMaxAgents) at random spots, at least kMinSeparation apart. Agent i registers i-th.
inline std::vector<Agent> makeAgents(std::uint32_t round, int n) {
    n = std::clamp(n, kMinAgents, kMaxAgents);
    Rng rng{round * 2654435761U + 12345U};
    for (int i = 0; i < 3; ++i) (void)rng.next();
    std::vector<Agent> out;
    for (int i = 0; i < n; ++i) {
        Vec2 p;
        for (int tries = 0; tries < 60; ++tries) {
            p = {kFieldLeft + rng.next() * (kFieldRight - kFieldLeft), kFieldTop + rng.next() * (kFieldBottom - kFieldTop)};
            const bool apart = std::all_of(out.begin(), out.end(), [&](const Agent& o) { return dist(o.pos, p) >= kMinSeparation; });
            if (apart) break;
        }
        out.push_back(Agent{p, kAccuracyMin + rng.next() * (1.0 - kAccuracyMin), State::Spawning});
    }
    return out;
}

struct Stats {
    int assignments = 0;
    double sumDistance = 0.0;  // distance gunner to gun at the moment of the assignment
    double sumArrival = 0.0;   // seconds the gunner needs to get there
};

inline double meanDistance(const Stats& s) { return s.assignments > 0 ? s.sumDistance / s.assignments : 0.0; }
inline double meanArrival(const Stats& s) { return s.assignments > 0 ? s.sumArrival / s.assignments : 0.0; }

struct FireEvent {
    std::size_t transition;
    double time;  // Sim::clock when it fired
    int agent;    // the agent concerned, -1 for none
};

struct Sim {
    Vec2 gun{800.0, 150.0};
    Policy policy = Policy::Fifo;
    int count = 6;
    std::uint32_t round = 0;
    std::vector<Agent> agents;
    Net net = makeArbiterNet(1);
    int gunner = -1;
    bool autoKill = true;
    Stats stats;
    std::vector<FireEvent> events;  // recent firings, older than a second are dropped
    double clock = 0.0;
    double tickAcc = 0.0;
    double mannedFor = 0.0;
    double restartIn = -1.0;
};

/// Starts a round with the given agents (all must be Spawning). An empty list is ignored.
inline void beginRound(Sim& s, std::vector<Agent> agents) {
    if (agents.empty()) return;
    s.agents = std::move(agents);
    s.net = makeArbiterNet(static_cast<int>(s.agents.size()));
    s.gunner = -1;
    s.events.clear();
    s.tickAcc = 0.0;
    s.mannedFor = 0.0;
    s.restartIn = -1.0;
}

inline void nextRound(Sim& s) {
    ++s.round;
    beginRound(s, makeAgents(s.round, s.count));
}

inline Sim makeSim(Policy policy, int count, std::uint32_t round = 0) {
    Sim s;
    s.policy = policy;
    s.count = std::clamp(count, kMinAgents, kMaxAgents);
    s.round = round;
    beginRound(s, makeAgents(round, s.count));
    return s;
}

/// The statistics describe one policy, so a change starts them over. The same policy changes nothing.
inline void setPolicy(Sim& s, Policy policy) {
    if (policy == s.policy) return;
    s.policy = policy;
    s.stats = {};
}

/// A new number of agents also starts a new round, because the net's arc weight depends on it.
inline void setCount(Sim& s, int count) {
    count = std::clamp(count, kMinAgents, kMaxAgents);
    if (count == s.count) return;
    s.count = count;
    s.stats = {};
    nextRound(s);
}

inline void record(Sim& s, std::size_t transition, int agent) { s.events.push_back({transition, s.clock, agent}); }

/// Only agents in the pool (Waiting) or at the gun (Walking, Manning) can be hit. Returns whether somebody died.
inline bool kill(Sim& s, std::size_t id) {
    if (id >= s.agents.size()) return false;
    Agent& a = s.agents[id];
    if (a.state == State::Waiting) {
        if (!fire(s.net, Dropout)) return false;
        record(s, Dropout, static_cast<int>(id));
    } else if (a.state == State::Walking || a.state == State::Manning) {
        if (!fire(s.net, Incapacitated)) return false;
        record(s, Incapacitated, static_cast<int>(id));
        s.gunner = -1;
    } else {
        return false;
    }
    a.state = State::Dead;
    return true;
}

inline bool killGunner(Sim& s) { return s.gunner >= 0 && kill(s, static_cast<std::size_t>(s.gunner)); }

namespace detail {
/// Fires at most one automatic transition per tick. Register comes first, then Start, then Assign.
inline void stepNet(Sim& s) {
    if (enabled(s.net, Register)) {
        for (std::size_t i = 0; i < s.agents.size(); ++i) {
            if (s.agents[i].state != State::Spawning) continue;
            fire(s.net, Register);
            s.agents[i].state = State::Registered;
            record(s, Register, static_cast<int>(i));
            return;
        }
    } else if (enabled(s.net, Start)) {
        fire(s.net, Start);
        for (Agent& a : s.agents) {
            if (a.state == State::Registered) a.state = State::Waiting;
        }
        record(s, Start, -1);
    } else if (enabled(s.net, Assign)) {
        std::vector<Candidate> waiting;
        for (std::size_t i = 0; i < s.agents.size(); ++i) {
            if (s.agents[i].state == State::Waiting) waiting.push_back({i, static_cast<int>(i), s.agents[i].pos, s.agents[i].accuracy});
        }
        const auto chosen = pick(s.policy, waiting, s.gun);
        if (!chosen) return;
        const std::size_t id = waiting[*chosen].id;
        fire(s.net, Assign);
        const double d = dist(s.agents[id].pos, s.gun);
        ++s.stats.assignments;
        s.stats.sumDistance += d;
        s.stats.sumArrival += d / kSpeed;
        s.agents[id].state = State::Walking;
        s.gunner = static_cast<int>(id);
        s.mannedFor = 0.0;
        record(s, Assign, static_cast<int>(id));
    }
}
}  // namespace detail

inline void advance(Sim& s, double dt) {
    dt = std::clamp(dt, 0.0, kMaxDt);
    s.clock += dt;
    s.events.erase(s.events.begin(), std::find_if(s.events.begin(), s.events.end(), [&](const FireEvent& e) { return e.time >= s.clock - 1.0; }));

    if (s.gunner >= 0) {
        Agent& g = s.agents[static_cast<std::size_t>(s.gunner)];
        if (g.state == State::Walking) {
            const double d = dist(g.pos, s.gun);
            const double step = kSpeed * dt;
            if (d <= step) {
                g.pos = s.gun;
                g.state = State::Manning;
                s.mannedFor = 0.0;
            } else {
                const double k = step / d;  // d > step >= 0, so d > 0
                g.pos = {g.pos.x + (s.gun.x - g.pos.x) * k, g.pos.y + (s.gun.y - g.pos.y) * k};
            }
        } else if (g.state == State::Manning) {
            s.mannedFor += dt;
            if (s.autoKill && s.mannedFor >= kManTime) killGunner(s);
        }
    }

    s.tickAcc += dt;
    for (int steps = 0; steps < 16 && s.tickAcc + 1e-9 >= kTick; ++steps) {
        s.tickAcc -= kTick;
        detail::stepNet(s);
    }
    s.tickAcc = std::max(s.tickAcc, 0.0);

    const bool anyAlive = std::any_of(s.agents.begin(), s.agents.end(), [](const Agent& a) { return a.state != State::Dead; });
    if (!anyAlive && s.restartIn < 0.0) s.restartIn = kRestartDelay;
    if (s.restartIn >= 0.0) {
        s.restartIn -= dt;
        if (s.restartIn <= 0.0) nextRound(s);
    }
}

}  // namespace pa
