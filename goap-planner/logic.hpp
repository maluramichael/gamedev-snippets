// GOAP logic without SDL: a world of a few variables, actions with preconditions, effects and costs, and two A* searches
// over it. The forward search walks through complete world states from the start, the backward search starts from the
// goal and works with partial states (only the variables that still matter). Both find the cheapest plan.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <queue>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace goap {

inline constexpr int kMaxVars = 16;
inline constexpr int kAny = -1;  // "does not matter", only used in partial states of the backward search

struct State {
    std::array<std::int8_t, kMaxVars> v{};
    bool operator==(const State& o) const { return v == o.v; }
};

struct Cond {
    int var;
    int value;
};

enum class Kind { Walk, PickUp, OpenDoor, KickDoor, PickLock, BreakWindow, OpenWindow, Enter, Climb, Attack, Loot };

struct Action {
    std::string name;
    Kind kind;
    int cost;
    std::vector<Cond> pre;
    std::vector<Cond> eff;
    int from = -1;  // places, for the animation: where the action starts and where it ends
    int to = -1;
};

struct Problem {
    int numVars = 0;
    std::array<int, kMaxVars> radix{};  // number of values of each variable
    State start;
    std::vector<Cond> goal;
    std::vector<Action> actions;
};

struct Plan {
    bool found = false;
    int cost = 0;
    std::vector<int> steps;  // indices into Problem::actions, in execution order
    long expanded = 0;       // nodes taken out of the open list and expanded
    long generated = 0;      // nodes put into the open list
};

inline bool applicable(const State& s, const Action& a) {
    for (const Cond& c : a.pre) {
        if (s.v[static_cast<std::size_t>(c.var)] != c.value) return false;
    }
    return true;
}

inline State apply(State s, const Action& a) {
    for (const Cond& c : a.eff) s.v[static_cast<std::size_t>(c.var)] = static_cast<std::int8_t>(c.value);
    return s;
}

inline bool goalMet(const State& s, const std::vector<Cond>& goal) {
    for (const Cond& c : goal) {
        if (s.v[static_cast<std::size_t>(c.var)] != c.value) return false;
    }
    return true;
}

namespace detail {

/// Smallest action cost. A goal that is not met yet needs at least one more action, so this is an admissible and
/// consistent estimate for both searches (A* never has to reopen a node).
inline int minCost(const Problem& p) {
    int best = 0;
    for (const Action& a : p.actions) {
        if (best == 0 || a.cost < best) best = a.cost;
    }
    return best;
}

/// One number for a state; every variable has radix + 1 digits because the backward search also stores "any".
inline std::uint64_t encode(const State& s, const Problem& p) {
    std::uint64_t key = 0;
    for (int i = 0; i < p.numVars; ++i) {
        key = key * static_cast<std::uint64_t>(p.radix[static_cast<std::size_t>(i)] + 1) + static_cast<std::uint64_t>(s.v[static_cast<std::size_t>(i)] + 1);
    }
    return key;
}

struct Node {
    State s;
    int g = 0;
    std::uint64_t parent = 0;
    int action = -1;  // the action on the edge from the parent, -1 at the root
    bool closed = false;
};

using Nodes = std::unordered_map<std::uint64_t, Node>;

struct Entry {
    int f;
    int negG;  // among equal f the deeper node first, it is closer to the goal
    long seq;
    std::uint64_t key;
    int g;
    bool operator>(const Entry& o) const { return std::tie(f, negG, seq) > std::tie(o.f, o.negG, o.seq); }
};

using Open = std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>>;

/// Undo `a` on the partial state `g`: what must hold before `a` runs so that `g` holds after it. False when `a` is no use
/// for `g` (it sets nothing `g` asks for) or contradicts it.
inline bool regress(const State& g, const Action& a, State& out) {
    bool relevant = false;
    for (const Cond& e : a.eff) {
        const int want = g.v[static_cast<std::size_t>(e.var)];
        if (want == e.value) {
            relevant = true;
        } else if (want != kAny) {
            return false;
        }
    }
    if (!relevant) return false;
    out = g;
    for (const Cond& e : a.eff) out.v[static_cast<std::size_t>(e.var)] = kAny;
    for (const Cond& c : a.pre) {
        auto& slot = out.v[static_cast<std::size_t>(c.var)];
        if (slot != kAny && slot != c.value) return false;
        slot = static_cast<std::int8_t>(c.value);
    }
    return true;
}

/// Shared A* loop. `neighbors(state, visit)` calls visit(nextState, actionIndex, cost) for every edge, `isGoal` ends the
/// search, `h` is the estimate. `reverse` says whether the path is read from the goal end (backward search).
template <class Neighbors, class IsGoal, class H>
Plan astar(const Problem& p, const State& root, bool reverse, Neighbors neighbors, IsGoal isGoal, H h) {
    Plan plan;
    Nodes nodes;
    Open open;
    long seq = 0;
    const std::uint64_t rootKey = encode(root, p);
    nodes[rootKey] = Node{root, 0, rootKey, -1, false};
    open.push({h(root), 0, seq++, rootKey, 0});
    ++plan.generated;
    while (!open.empty()) {
        const Entry e = open.top();
        open.pop();
        Node& n = nodes[e.key];
        if (n.closed || e.g != n.g) continue;  // an older, worse entry of a node that was reached cheaper since
        n.closed = true;
        ++plan.expanded;
        if (isGoal(n.s)) {
            plan.found = true;
            plan.cost = n.g;
            for (std::uint64_t k = e.key; nodes[k].action >= 0; k = nodes[k].parent) plan.steps.push_back(nodes[k].action);
            if (!reverse) std::reverse(plan.steps.begin(), plan.steps.end());
            return plan;
        }
        const State from = n.s;
        const int fromG = n.g;
        neighbors(from, [&](const State& next, int action, int cost) {
            const std::uint64_t key = encode(next, p);
            const int g = fromG + cost;
            auto it = nodes.find(key);
            if (it != nodes.end() && it->second.g <= g) return;
            nodes[key] = Node{next, g, e.key, action, false};
            open.push({g + h(next), -g, seq++, key, g});
            ++plan.generated;
        });
    }
    return plan;
}

}  // namespace detail

/// A* over complete world states, from the start to any state that meets the goal. Edges are the applicable actions.
inline Plan planForward(const Problem& p) {
    const int step = detail::minCost(p);
    return detail::astar(
        p, p.start, false,
        [&](const State& s, auto&& visit) {
            for (std::size_t i = 0; i < p.actions.size(); ++i) {
                const Action& a = p.actions[i];
                if (!applicable(s, a)) continue;
                const State next = apply(s, a);
                if (next == s) continue;  // changes nothing
                visit(next, static_cast<int>(i), a.cost);
            }
        },
        [&](const State& s) { return goalMet(s, p.goal); }, [&](const State& s) { return goalMet(s, p.goal) ? 0 : step; });
}

/// A* from the goal backwards over partial states: each step asks what must hold before an action that helps. Variables
/// nobody asks about stay "any", so objects that have nothing to do with the goal never enter the search state.
inline Plan planBackward(const Problem& p) {
    const int step = detail::minCost(p);
    State goal;
    goal.v.fill(static_cast<std::int8_t>(kAny));
    for (const Cond& c : p.goal) goal.v[static_cast<std::size_t>(c.var)] = static_cast<std::int8_t>(c.value);
    const auto atStart = [&](const State& s) {
        for (int i = 0; i < p.numVars; ++i) {
            const int want = s.v[static_cast<std::size_t>(i)];
            if (want != kAny && want != p.start.v[static_cast<std::size_t>(i)]) return false;
        }
        return true;
    };
    return detail::astar(
        p, goal, true,
        [&](const State& g, auto&& visit) {
            for (std::size_t i = 0; i < p.actions.size(); ++i) {
                State before;
                if (!detail::regress(g, p.actions[i], before) || before == g) continue;
                visit(before, static_cast<int>(i), p.actions[i].cost);
            }
        },
        atStart, [&](const State& s) { return atStart(s) ? 0 : step; });
}

// ---- the mini world -------------------------------------------------------------------------------------------

enum class Role { Soldier, Assassin, Rat };
enum class Door { Free, Locked, Barricaded };

inline constexpr int kMaxCrates = 9;
inline constexpr int kVarLoc = 0;
inline constexpr int kVarWeapon = 1;
inline constexpr int kVarDoor = 2;
inline constexpr int kVarWindow = 3;
inline constexpr int kVarDead = 4;
inline constexpr int kVarLooted = 5;  // one bool per crate from here on

inline constexpr int kStart = 0;
inline constexpr int kLocker = 1;
inline constexpr int kDoor = 2;
inline constexpr int kWindow = 3;
inline constexpr int kRoom = 4;
inline constexpr int kFirstCrate = 5;

struct World {
    Role role = Role::Soldier;
    Door door = Door::Free;
    int crates = 0;
};

struct Pos {
    double x;
    double y;
};

inline Pos placePos(int place) {
    static constexpr std::array<Pos, 5 + kMaxCrates> kPos{{{70, 330},
                                                           {250, 360},
                                                           {530, 215},
                                                           {720, 410},
                                                           {760, 200},
                                                           {340, 300},
                                                           {390, 300},
                                                           {440, 300},
                                                           {340, 350},
                                                           {390, 350},
                                                           {440, 350},
                                                           {340, 400},
                                                           {390, 400},
                                                           {440, 400}}};
    return kPos[static_cast<std::size_t>(place)];
}

inline int walkCost(int a, int b) {
    const Pos p = placePos(a);
    const Pos q = placePos(b);
    const long c = std::lround(std::hypot(q.x - p.x, q.y - p.y) / 60.0);
    return c < 1 ? 1 : static_cast<int>(c);
}

inline std::string placeName(int place) {
    static const char* const kNames[] = {"Start", "Locker", "Door", "Window", "Room"};
    if (place < kFirstCrate) return kNames[place];
    return "Crate " + std::to_string(place - kFirstCrate + 1);
}

inline State startState() { return State{}; }

inline Problem buildProblem(const World& w, const State& start) {
    Problem p;
    p.start = start;
    p.numVars = kVarLooted + w.crates;
    p.radix.fill(2);
    p.radix[kVarLoc] = kFirstCrate + w.crates;
    p.goal = {{kVarDead, 1}};
    auto add = [&p](std::string name, Kind kind, int cost, std::vector<Cond> pre, std::vector<Cond> eff, int from = -1, int to = -1) {
        p.actions.push_back({std::move(name), kind, cost, std::move(pre), std::move(eff), from, to});
    };

    // Every place outside the room can be walked to from every other one, the cost is the distance.
    std::vector<int> outside{kStart, kLocker, kDoor, kWindow};
    for (int i = 0; i < w.crates; ++i) outside.push_back(kFirstCrate + i);
    for (int a : outside) {
        for (int b : outside) {
            if (a != b) add("go " + placeName(a) + ">" + placeName(b), Kind::Walk, walkCost(a, b), {{kVarLoc, a}}, {{kVarLoc, b}}, a, b);
        }
    }
    // The distractors: crates that can be looted, which changes nothing the goal cares about.
    for (int i = 0; i < w.crates; ++i) {
        add("loot " + placeName(kFirstCrate + i), Kind::Loot, 1, {{kVarLoc, kFirstCrate + i}}, {{kVarLooted + i, 1}});
    }

    const bool free = w.door == Door::Free;
    const bool passable = w.door != Door::Barricaded;
    const std::vector<Cond> atDoor{{kVarLoc, kDoor}};
    const std::vector<Cond> atWindow{{kVarLoc, kWindow}};
    const std::vector<Cond> openDoor{{kVarLoc, kDoor}, {kVarDoor, 1}};
    const std::vector<Cond> openWindow{{kVarLoc, kWindow}, {kVarWindow, 1}};
    switch (w.role) {
        case Role::Soldier:
            add("pick up rifle", Kind::PickUp, 1, {{kVarLoc, kLocker}}, {{kVarWeapon, 1}});
            if (free) add("open door", Kind::OpenDoor, 1, atDoor, {{kVarDoor, 1}});
            if (passable) add("kick door", Kind::KickDoor, 4, atDoor, {{kVarDoor, 1}});
            add("walk in", Kind::Enter, 1, openDoor, {{kVarLoc, kRoom}}, kDoor, kRoom);
            add("break window", Kind::BreakWindow, 3, atWindow, {{kVarWindow, 1}});
            add("climb in", Kind::Climb, 2, openWindow, {{kVarLoc, kRoom}}, kWindow, kRoom);
            add("shoot", Kind::Attack, 1, {{kVarLoc, kRoom}, {kVarWeapon, 1}}, {{kVarDead, 1}});
            break;
        case Role::Assassin:
            if (free) add("open door", Kind::OpenDoor, 1, atDoor, {{kVarDoor, 1}});
            if (passable) add("pick lock", Kind::PickLock, 2, atDoor, {{kVarDoor, 1}});
            add("walk in", Kind::Enter, 1, openDoor, {{kVarLoc, kRoom}}, kDoor, kRoom);
            add("open window", Kind::OpenWindow, 1, atWindow, {{kVarWindow, 1}});
            add("climb in", Kind::Climb, 2, openWindow, {{kVarLoc, kRoom}}, kWindow, kRoom);
            add("stab", Kind::Attack, 1, {{kVarLoc, kRoom}}, {{kVarDead, 1}});
            break;
        case Role::Rat:
            if (free) add("open door", Kind::OpenDoor, 1, atDoor, {{kVarDoor, 1}});
            add("walk in", Kind::Enter, 1, openDoor, {{kVarLoc, kRoom}}, kDoor, kRoom);
            add("bite", Kind::Attack, 1, {{kVarLoc, kRoom}}, {{kVarDead, 1}});
            break;
    }
    return p;
}

inline int findAction(const Problem& p, const std::string& name) {
    for (std::size_t i = 0; i < p.actions.size(); ++i) {
        if (p.actions[i].name == name) return static_cast<int>(i);
    }
    return -1;
}

}  // namespace goap
