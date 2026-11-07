// Tests for the GOAP logic. Plain asserts on hand-computed cases, no framework, no SDL.
#undef NDEBUG
#include <array>
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <string>
#include <vector>

#include "logic.hpp"

using namespace goap;

namespace {

std::vector<std::string> names(const Problem& p, const Plan& plan) {
    std::vector<std::string> out;
    for (int i : plan.steps) out.push_back(p.actions[static_cast<std::size_t>(i)].name);
    return out;
}

using Names = std::vector<std::string>;

Problem world(Role role, Door door, int crates = 0) { return buildProblem(World{role, door, crates}, startState()); }

// A tiny hand-made problem: two bool variables x and y, goal y = 1.
Problem tiny(int costShortcut, bool withX) {
    Problem p;
    p.numVars = 2;
    p.radix = {2, 2};
    p.goal = {{1, 1}};
    if (withX) p.actions.push_back({"a", Kind::Walk, 1, {}, {{0, 1}}});                // makes x
    p.actions.push_back({"b", Kind::Walk, 1, {{0, 1}}, {{1, 1}}});                     // needs x, makes y
    p.actions.push_back({"c", Kind::Walk, costShortcut, {}, {{1, 1}}});                // makes y directly
    p.actions.push_back({"d", Kind::Walk, 1, {}, {{1, 0}}});                           // undoes y, never useful
    return p;
}

void tinyProblems() {
    // a + b cost 2, the shortcut c costs 3: the two-step plan wins.
    Problem p = tiny(3, true);
    for (const Plan& plan : {planForward(p), planBackward(p)}) {
        assert(plan.found && plan.cost == 2);
        assert((names(p, plan) == Names{"a", "b"}));
    }
    // Shortcut costs 1: the one-step plan wins.
    p = tiny(1, true);
    for (const Plan& plan : {planForward(p), planBackward(p)}) {
        assert(plan.found && plan.cost == 1);
        assert((names(p, plan) == Names{"c"}));
    }
    // Same cost: either is fine, but the cost is 2 and the steps must be a valid plan, checked below by simulation.
    p = tiny(2, true);
    assert(planForward(p).cost == 2 && planBackward(p).cost == 2);
    // Without a, b can never run (x stays 0), and c is the only way: cost 3.
    p = tiny(3, false);
    assert(planForward(p).cost == 3 && planBackward(p).cost == 3);
    // No action produces y at all: no plan, and the search ends instead of looping.
    Problem none;
    none.numVars = 2;
    none.radix = {2, 2};
    none.goal = {{1, 1}};
    none.actions.push_back({"a", Kind::Walk, 1, {}, {{0, 1}}});
    none.actions.push_back({"b", Kind::Walk, 1, {{0, 1}}, {{0, 0}}});
    assert(!planForward(none).found);
    assert(!planBackward(none).found);
    assert(planForward(none).expanded == 2);  // states (0,0) and (1,0), nothing else is reachable
    // Precondition that nothing satisfies: b needs x = 1 but nobody sets x.
    Problem blocked;
    blocked.numVars = 2;
    blocked.radix = {2, 2};
    blocked.goal = {{1, 1}};
    blocked.actions.push_back({"b", Kind::Walk, 1, {{0, 1}}, {{1, 1}}});
    assert(!planForward(blocked).found && !planBackward(blocked).found);
    // The goal already holds: empty plan, cost 0.
    blocked.start.v[1] = 1;
    const Plan done = planForward(blocked);
    assert(done.found && done.cost == 0 && done.steps.empty() && done.expanded == 1);
    const Plan doneB = planBackward(blocked);
    assert(doneB.found && doneB.cost == 0 && doneB.steps.empty());
}

void walkCosts() {
    // Straight-line distance / 60, rounded, at least 1. Distances worked out from the place coordinates.
    assert(walkCost(kStart, kLocker) == 3);   // (180, 30) -> 182.5
    assert(walkCost(kLocker, kDoor) == 5);    // (280, 145) -> 315.3
    assert(walkCost(kStart, kDoor) == 8);     // (460, 115) -> 474.2
    assert(walkCost(kStart, kWindow) == 11);  // (650, 80) -> 654.9
    assert(walkCost(kLocker, kWindow) == 8);  // (470, 50) -> 472.7
    assert(walkCost(kDoor, kWindow) == 5);    // (190, 195) -> 272.2, 4.54 rounds up
    assert(walkCost(kStart, kStart) == 1);    // never free
    assert(walkCost(kStart, kDoor) == walkCost(kDoor, kStart));
}

void cheapestPlans() {
    struct Case {
        Role role;
        Door door;
        int cost;
        Names steps;
    };
    const std::vector<Case> cases{
        {Role::Soldier, Door::Free, 12, {"go Start>Locker", "pick up rifle", "go Locker>Door", "open door", "walk in", "shoot"}},
        {Role::Soldier, Door::Locked, 15, {"go Start>Locker", "pick up rifle", "go Locker>Door", "kick door", "walk in", "shoot"}},
        {Role::Soldier, Door::Barricaded, 18, {"go Start>Locker", "pick up rifle", "go Locker>Window", "break window", "climb in", "shoot"}},
        {Role::Assassin, Door::Free, 11, {"go Start>Door", "open door", "walk in", "stab"}},
        {Role::Assassin, Door::Locked, 12, {"go Start>Door", "pick lock", "walk in", "stab"}},
        {Role::Assassin, Door::Barricaded, 15, {"go Start>Window", "open window", "climb in", "stab"}},
        {Role::Rat, Door::Free, 11, {"go Start>Door", "open door", "walk in", "bite"}},
    };
    for (const Case& c : cases) {
        const Problem p = world(c.role, c.door);
        const Plan f = planForward(p);
        const Plan b = planBackward(p);
        assert(f.found && f.cost == c.cost);
        assert(b.found && b.cost == c.cost);
        assert(names(p, f) == c.steps);
    }
}

void noPlanWithoutPrecondition() {
    // The rat can open a door but not kick it, pick it, or use the window.
    for (Door d : {Door::Locked, Door::Barricaded}) {
        const Problem p = world(Role::Rat, d);
        assert(!planForward(p).found);
        assert(!planBackward(p).found);
    }
    // The soldier without a way to the weapon cannot shoot: drop the pick-up action and the plan is gone.
    Problem p = world(Role::Soldier, Door::Free);
    p.actions.erase(p.actions.begin() + findAction(p, "pick up rifle"));
    assert(!planForward(p).found);
    assert(!planBackward(p).found);
}

State at(int place, bool weapon) {
    State s = startState();
    s.v[kVarLoc] = static_cast<std::int8_t>(place);
    s.v[kVarWeapon] = weapon ? 1 : 0;
    return s;
}

void replanAfterBlockade() {
    // The soldier stands at the door with the rifle. Door locked: kick it.
    Problem p = buildProblem(World{Role::Soldier, Door::Locked, 0}, at(kDoor, true));
    Plan plan = planForward(p);
    assert(plan.found && plan.cost == 6);  // kick 4 + walk in 1 + shoot 1
    assert((names(p, plan) == Names{"kick door", "walk in", "shoot"}));
    // Barricaded while he stands there: around to the window, 5 + 3 + 2 + 1.
    p = buildProblem(World{Role::Soldier, Door::Barricaded, 0}, at(kDoor, true));
    plan = planForward(p);
    assert(plan.found && plan.cost == 11);
    assert((names(p, plan) == Names{"go Door>Window", "break window", "climb in", "shoot"}));
    assert(planBackward(p).cost == 11);
    // The door is already open (state), the barricade no longer matters for walking in.
    State open = at(kDoor, true);
    open.v[kVarDoor] = 1;
    p = buildProblem(World{Role::Soldier, Door::Barricaded, 0}, open);
    plan = planForward(p);
    assert(plan.found && plan.cost == 2);
    assert((names(p, plan) == Names{"walk in", "shoot"}));
    // The rat at the door with a locked door has no plan, with the door free it does.
    p = buildProblem(World{Role::Rat, Door::Locked, 0}, at(kDoor, false));
    assert(!planForward(p).found);
    p = buildProblem(World{Role::Rat, Door::Free, 0}, at(kDoor, false));
    assert(planForward(p).cost == 3);
}

void scriptLookup() {
    const Problem freeDoor = world(Role::Soldier, Door::Free);
    const Problem locked = world(Role::Soldier, Door::Locked);
    assert(findAction(freeDoor, "open door") >= 0);
    assert(findAction(locked, "open door") == -1);  // the scripted step does not exist any more
    assert(findAction(locked, "kick door") >= 0);
    assert(findAction(world(Role::Rat, Door::Locked), "kick door") == -1);
    assert(findAction(world(Role::Assassin, Door::Barricaded), "pick lock") == -1);
    assert(findAction(freeDoor, "no such action") == -1);
}

// Independent oracle: sweep over all known states and relax until nothing improves (Bellman-Ford style).
int oracleCost(const Problem& p) {
    std::map<std::array<std::int8_t, kMaxVars>, int> dist;
    dist[p.start.v] = 0;
    bool changed = true;
    while (changed) {
        changed = false;
        const auto snapshot = dist;
        for (const auto& [key, d] : snapshot) {
            State s;
            s.v = key;
            for (const Action& a : p.actions) {
                if (!applicable(s, a)) continue;
                const State n = apply(s, a);
                const auto it = dist.find(n.v);
                if (it == dist.end() || d + a.cost < it->second) {
                    dist[n.v] = d + a.cost;
                    changed = true;
                }
            }
        }
    }
    int best = -1;
    for (const auto& [key, d] : dist) {
        State s;
        s.v = key;
        if (goalMet(s, p.goal) && (best < 0 || d < best)) best = d;
    }
    return best;
}

bool validPlan(const Problem& p, const Plan& plan) {
    State s = p.start;
    int sum = 0;
    for (int i : plan.steps) {
        const Action& a = p.actions[static_cast<std::size_t>(i)];
        if (!applicable(s, a)) return false;
        s = apply(s, a);
        sum += a.cost;
    }
    return goalMet(s, p.goal) && sum == plan.cost;
}

void searchesAgreeWithOracle() {
    int worlds = 0;
    for (Role role : {Role::Soldier, Role::Assassin, Role::Rat}) {
        for (Door door : {Door::Free, Door::Locked, Door::Barricaded}) {
            for (int crates = 0; crates <= 2; ++crates) {
                const Problem p = world(role, door, crates);
                const int want = oracleCost(p);
                const Plan f = planForward(p);
                const Plan b = planBackward(p);
                assert(f.found == (want >= 0) && b.found == (want >= 0));
                if (want >= 0) {
                    assert(f.cost == want && b.cost == want);
                    assert(validPlan(p, f) && validPlan(p, b));
                }
                ++worlds;
            }
        }
    }
    assert(worlds == 27);
}

void distractorsBlowUpTheForwardSearch() {
    const Problem p0 = world(Role::Soldier, Door::Free, 0);
    const Problem p9 = world(Role::Soldier, Door::Free, kMaxCrates);
    const Plan f0 = planForward(p0);
    const Plan f9 = planForward(p9);
    const Plan b0 = planBackward(p0);
    const Plan b9 = planBackward(p9);
    for (Door d : {Door::Free, Door::Barricaded}) {
        for (int n : {0, 3, 6, 9}) {
            const Problem p = world(Role::Soldier, d, n);
            const Plan f = planForward(p);
            const Plan b = planBackward(p);
            std::printf("soldier door %d crates %d: cost %d forward %ld backward %ld\n", static_cast<int>(d), n, f.cost, f.expanded, b.expanded);
        }
    }
    std::printf("forward expanded: 0 crates %ld, 9 crates %ld; backward: 0 crates %ld, 9 crates %ld\n", f0.expanded, f9.expanded, b0.expanded, b9.expanded);
    std::fflush(stdout);
    // Same plan, same cost: the crates do not matter for the result.
    assert(f0.cost == 12 && f9.cost == 12 && b0.cost == 12 && b9.cost == 12);
    assert(names(p0, f0) == names(p9, f9));
    assert(f9.expanded > 20 * f0.expanded);
    assert(b9.expanded < f9.expanded / 10);
}

}  // namespace

int main() {
    tinyProblems();
    walkCosts();
    cheapestPlans();
    noPlanWithoutPrecondition();
    replanAfterBlockade();
    scriptLookup();
    searchesAgreeWithOracle();
    distractorsBlowUpTheForwardSearch();
    std::puts("goap-planner logic: all tests passed");
    return 0;
}
