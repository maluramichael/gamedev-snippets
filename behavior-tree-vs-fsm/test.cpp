// Tests for the behavior-tree-vs-fsm logic. Plain asserts on hand-computed cases, no framework, no SDL.
// Time steps are 0.125 s (exact in binary), so the walking and timer numbers below are exact.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <set>
#include <source_location>
#include <string>
#include <tuple>
#include <vector>

#include "logic.hpp"

using namespace gbt;

namespace {

constexpr double kDt = 0.125;

bool near(double a, double b, double eps = 1e-6) { return std::fabs(a - b) < eps; }

const Features kNone{};
const Features kAll{true, true, true};

// ---- behavior tree nodes -------------------------------------------------------------------------------------------

/// A leaf that returns a scripted status per call (the last one repeats) and counts calls and aborts.
struct Probe {
    std::vector<Status> script;
    std::size_t pos = 0;
    int calls = 0;
    int aborts = 0;
};

std::unique_ptr<Node> probeLeaf(Probe& p) {
    return std::make_unique<Leaf>(
        "probe",
        [&p](Ctx&, double&) {
            ++p.calls;
            const Status s = p.script[std::min(p.pos, p.script.size() - 1)];
            ++p.pos;
            return s;
        },
        [&p] { ++p.aborts; });
}

struct Rig {
    Body body;
    Ctx ctx{body, 900.0, kDt, 0};
    Status tick(Node& n) {
        ++ctx.tickId;
        return n.tick(ctx);
    }
};

constexpr Status S = Status::Success;
constexpr Status F = Status::Failure;
constexpr Status R = Status::Running;

void sequenceResults() {
    Rig rig;
    {
        Probe a{{S}}, b{{S}};
        Sequence seq("seq");
        seq.add(probeLeaf(a));
        seq.add(probeLeaf(b));
        assert(rig.tick(seq) == S);
        assert(a.calls == 1 && b.calls == 1);
    }
    {
        Probe a{{S}}, b{{F}};
        Sequence seq("seq");
        seq.add(probeLeaf(a));
        seq.add(probeLeaf(b));
        assert(rig.tick(seq) == F);
    }
    {
        Probe a{{F}}, b{{S}};  // a failing child stops the sequence, b is never asked
        Sequence seq("seq");
        seq.add(probeLeaf(a));
        seq.add(probeLeaf(b));
        assert(rig.tick(seq) == F);
        assert(b.calls == 0);
    }
    {
        // A running child is resumed directly: the first child is not asked again until the sequence is done.
        Probe a{{S}}, b{{R, R, S}};
        Sequence seq("seq");
        seq.add(probeLeaf(a));
        seq.add(probeLeaf(b));
        assert(rig.tick(seq) == R);
        assert(rig.tick(seq) == R);
        assert(a.calls == 1 && b.calls == 2);
        assert(rig.tick(seq) == S);
        assert(a.calls == 1 && b.calls == 3);
        assert(rig.tick(seq) == S);  // next round starts at the first child again
        assert(a.calls == 2 && b.calls == 4);
    }
    {
        Sequence empty("empty");  // nothing to do counts as done
        assert(rig.tick(empty) == S);
    }
}

void selectorResults() {
    Rig rig;
    {
        Probe a{{F}}, b{{S}};
        Selector sel("sel", false);
        sel.add(probeLeaf(a));
        sel.add(probeLeaf(b));
        assert(rig.tick(sel) == S);
    }
    {
        Probe a{{F}}, b{{F}};
        Selector sel("sel", false);
        sel.add(probeLeaf(a));
        sel.add(probeLeaf(b));
        assert(rig.tick(sel) == F);
    }
    {
        Probe a{{S}}, b{{S}};  // the first success wins, b is never asked
        Selector sel("sel", true);
        sel.add(probeLeaf(a));
        sel.add(probeLeaf(b));
        assert(rig.tick(sel) == S);
        assert(b.calls == 0);
    }
    {
        Selector empty("empty", false);  // no child, no success
        assert(rig.tick(empty) == F);
    }
}

void passiveSelectorKeepsTheRunningChild() {
    Rig rig;
    Probe a{{F, F, S}}, b{{R, R, S}};
    Selector sel("sel", false);
    sel.add(probeLeaf(a));
    sel.add(probeLeaf(b));
    assert(rig.tick(sel) == R);
    assert(rig.tick(sel) == R);
    assert(rig.tick(sel) == S);  // b finishes, a was asked once and never again
    assert(a.calls == 1 && b.calls == 3);
    assert(b.aborts == 0);
}

void activeSelectorChecksFromTheTop() {
    Rig rig;
    Probe a{{F, F, S}}, b{{R}};
    Selector sel("sel", true);
    sel.add(probeLeaf(a));
    sel.add(probeLeaf(b));
    assert(rig.tick(sel) == R);  // a fails, b runs
    assert(rig.tick(sel) == R);  // a is asked again, still failing
    assert(a.calls == 2 && b.calls == 2);
    assert(rig.tick(sel) == S);  // a succeeds now and cuts b off
    assert(a.calls == 3 && b.calls == 2);
    assert(b.aborts == 1);
    assert(sel.currentIndex() == 0);
}

void abortResetsSequenceProgress() {
    Rig rig;
    Probe a{{F, S, F}}, x{{S}}, y{{R}};
    Selector sel("sel", true);
    sel.add(probeLeaf(a));
    auto seq = std::make_unique<Sequence>("seq");
    seq->add(probeLeaf(x));
    seq->add(probeLeaf(y));
    sel.add(std::move(seq));
    assert(rig.tick(sel) == R);  // a fails, the sequence runs x then y
    assert(x.calls == 1 && y.calls == 1);
    assert(rig.tick(sel) == S);  // a wins, the sequence is aborted while y runs
    assert(y.aborts == 1);
    assert(rig.tick(sel) == R);  // a fails again: the sequence starts over at x, not at y
    assert(x.calls == 2 && y.calls == 2);
}

void abortingAFinishedNodeDoesNothing() {
    Rig rig;
    Probe a{{S}};
    auto leaf = probeLeaf(a);
    assert(rig.tick(*leaf) == S);
    leaf->abort();
    assert(a.aborts == 0);
}

// ---- walking, senses, timers ---------------------------------------------------------------------------------------

void walking() {
    Body b;
    b.x = 0;
    assert(!walkTo(b, 100, 80, 3, kDt));
    assert(near(b.x, 10) && b.facing > 0);

    b.x = 100;  // already there
    assert(walkTo(b, 100, 80, 3, kDt));
    assert(near(b.x, 100));

    b.x = 0;  // stops at the stop distance, not on the target
    assert(walkTo(b, 100, 150, 38, 1.0));
    assert(near(b.x, 62));

    b.x = 99.5;  // never overshoots
    assert(walkTo(b, 100, 80, 0, 1.0));
    assert(near(b.x, 100));

    b.x = 100;  // to the left
    assert(!walkTo(b, 0, 80, 3, kDt));
    assert(near(b.x, 90) && b.facing < 0);

    b.x = 0;  // dt 0 does not move
    assert(!walkTo(b, 100, 80, 3, 0.0));
    assert(near(b.x, 0));
}

void senses() {
    Body b;
    b.x = 100;
    assert(perceive(b, 270.0).sees);    // exactly 170 away
    assert(!perceive(b, 270.5).sees);
    assert(perceive(b, 144.0).inRange);  // exactly 44 away
    assert(!perceive(b, 144.5).inRange);
    assert(perceive(b, -70.0).sees);     // left side works the same

    b.hp = 30.0;
    assert(!perceive(b, 900.0).lowHp);   // 30 is not low yet
    b.hp = 29.9;
    assert(perceive(b, 900.0).lowHp);
    b.hp = 99.9;
    assert(!perceive(b, 900.0).healed);
    b.hp = 100.0;
    assert(perceive(b, 900.0).healed);
}

void seeingRemembersAndCancelsNoise() {
    Body b;
    b.x = 100;
    b.hasNoise = true;
    sense(b, 900.0);  // too far, nothing changes
    assert(!b.hasLead && b.hasNoise);
    sense(b, 200.0);
    assert(b.hasLead && near(b.leadX, 200.0));
    assert(!b.hasNoise);  // the guard sees the cause, the noise is explained
}

void hearing() {
    Body b;
    b.x = 100;
    makeNoise(b, 430.0);  // exactly 330 away
    assert(b.hasNoise && near(b.noiseX, 430.0));
    Body c;
    c.x = 100;
    makeNoise(c, 430.5);
    assert(!c.hasNoise);
}

// ---- structure: how much the change costs --------------------------------------------------------------------------

Features subset(int mask) { return Features{(mask & 1) != 0, (mask & 2) != 0, (mask & 4) != 0}; }

int bits(int mask) { return (mask & 1) + ((mask >> 1) & 1) + ((mask >> 2) & 1); }

void edgeCounts() {
    // 6 edges for the four base states. Each requirement that has to reach every state adds one edge more than the
    // one before: 6, 11, 17, 24.
    const int expected[4] = {6, 11, 17, 24};
    for (int mask = 0; mask < 8; ++mask) {
        assert(static_cast<int>(buildEdges(subset(mask), false).size()) == expected[bits(mask)]);
    }
    assert(buildEdges(kNone, false).size() == 6);
    assert(buildEdges(kAll, true).size() == 24);  // the history marker changes where an edge goes, not how many there are
}

void stateCounts() {
    assert(stateCount(kNone) == 4);
    assert(stateCount(Features{true, false, false}) == 5);
    assert(stateCount(kAll) == 7);
}

void nodeCounts() {
    assert(countNodes(*buildTree(kNone, true)) == 12);
    assert(countNodes(*buildTree(subset(1), true)) == 16);  // noise: 4 nodes
    assert(countNodes(*buildTree(subset(2), true)) == 16);  // low health: 4 nodes
    assert(countNodes(*buildTree(subset(4), true)) == 15);  // radio: 3 nodes
    assert(countNodes(*buildTree(subset(3), true)) == 20);
    assert(countNodes(*buildTree(subset(5), true)) == 19);
    assert(countNodes(*buildTree(subset(6), true)) == 19);
    assert(countNodes(*buildTree(kAll, true)) == 23);
    assert(countNodes(*buildTree(kAll, false)) == 23);  // passive or active is the same tree
}

void edgesAreWellFormed() {
    for (int mask = 0; mask < 8; ++mask) {
        const Features f = subset(mask);
        const auto edges = buildEdges(f, false);
        std::set<std::tuple<int, int, int>> seen;
        std::set<State> withExit;
        for (const Edge& e : edges) {
            assert(stateExists(e.from, f) && stateExists(e.to, f));
            assert(e.from != e.to);
            assert(seen.insert({static_cast<int>(e.from), static_cast<int>(e.to), static_cast<int>(e.when)}).second);
            withExit.insert(e.from);
        }
        // no dead ends: every state has a way out
        for (State s : kAllStates) {
            if (stateExists(s, f)) assert(withExit.count(s) == 1);
        }
    }
}

void changeCosts() {
    // Noise: a new state that Patrol and Search can enter.
    Cost c = changeCost(kNone, Features{true, false, false});
    assert(c.fsmEdges == 5 && c.fsmStatesEdited == 2);
    assert(c.btNodes == 4 && c.btPlaces == 1);

    // Low health reaches the four base states.
    c = changeCost(kNone, Features{false, true, false});
    assert(c.fsmEdges == 5 && c.fsmStatesEdited == 4);
    assert(c.btNodes == 4 && c.btPlaces == 1);

    // With noise present it also has to reach Inspect.
    c = changeCost(Features{true, false, false}, Features{true, true, false});
    assert(c.fsmEdges == 6 && c.fsmStatesEdited == 5);

    // Radio call as the last one: every other state, Heal and Inspect included.
    c = changeCost(Features{true, true, false}, kAll);
    assert(c.fsmEdges == 7 && c.fsmStatesEdited == 6);
    assert(c.btNodes == 3 && c.btPlaces == 1);

    // Taking it away again is the same change with the opposite sign.
    c = changeCost(kAll, Features{true, true, false});
    assert(c.fsmEdges == -7 && c.fsmStatesEdited == 6);
    assert(c.btNodes == -3 && c.btPlaces == 1);

    c = changeCost(kAll, kAll);
    assert(c.fsmEdges == 0 && c.fsmStatesEdited == 0 && c.btNodes == 0 && c.btPlaces == 0);
}

// ---- behavior, both brains on the same inputs ---------------------------------------------------------------------

template <class Brain>
void run(Brain& brain, int ticks, double playerX) {
    for (int i = 0; i < ticks; ++i) brain.tick(playerX, kDt);
}

struct Pair {
    FsmBrain fsm;
    BtBrain bt;
    explicit Pair(const Features& f, bool history = false, bool active = true) {
        fsm.configure(f, history);
        bt.configure(f, active);
    }
    void run(int ticks, double playerX) {
        ::run(fsm, ticks, playerX);
        ::run(bt, ticks, playerX);
    }
    void both(State s, std::source_location at = std::source_location::current()) const {
        if (fsm.state != s || bt.state != s) {
            std::fprintf(stderr, "line %u: expected %s, fsm is %s, bt is %s\n", static_cast<unsigned>(at.line()), name(s), name(fsm.state), name(bt.state));
        }
        assert(fsm.state == s);
        assert(bt.state == s);
    }
};

void patrolling() {
    Pair p(kNone);
    p.run(8, 900.0);  // one second at 80 px/s from x = 60 towards 260
    p.both(State::Patrol);
    assert(near(p.fsm.body.x, 140.0) && near(p.bt.body.x, 140.0));

    // Reaching the waypoint turns to the next one.
    Pair q(kNone);
    q.fsm.body.x = q.bt.body.x = 255.0;
    q.run(1, 900.0);
    assert(q.fsm.body.waypoint == 2 && q.bt.body.waypoint == 2);
}

void chaseThenAttack() {
    Pair p(kNone);
    p.run(1, 200.0);  // the player is 140 away, inside the sight
    p.both(State::Chase);
    p.run(5, 200.0);  // 6 ticks at 18.75 px in total, the guard stops 38 from the player
    assert(near(p.fsm.body.x, 162.0) && near(p.bt.body.x, 162.0));
    assert(p.fsm.state == State::Chase && p.bt.state == State::Chase);
    p.run(1, 200.0);
    p.both(State::Attack);
    p.run(3, 200.0);  // a strike takes 0.5 s = 4 ticks
    assert(p.fsm.body.strikes == 1 && p.bt.body.strikes == 1);
    p.run(4, 200.0);
    assert(p.fsm.body.strikes == 2 && p.bt.body.strikes == 2);
}

void losingThePlayerEndsInPatrol() {
    Pair p(kNone);
    p.run(10, 200.0);
    p.both(State::Attack);
    p.run(24, 900.0);  // three seconds later the guard still looks around where it last saw the player
    p.both(State::Search);
    p.run(40, 900.0);
    p.both(State::Patrol);
}

void passiveSelectorIsDeaf() {
    BtBrain active, passive;
    active.configure(kNone, true);
    passive.configure(kNone, false);
    for (BtBrain* bt : {&active, &passive}) {
        bt->body.x = 100.0;
        bt->body.hasLead = true;
        bt->body.leadX = 600.0;
    }
    run(active, 1, 900.0);  // both walk towards the old sighting
    run(passive, 1, 900.0);
    assert(active.state == State::Search && passive.state == State::Search);

    run(active, 1, 200.0);  // the player steps into the sight, 90 away
    run(passive, 1, 200.0);
    assert(active.state == State::Chase);  // looked at the top again, saw the player
    assert(passive.state == State::Search);  // still busy with its search branch

    run(active, 15, 200.0);
    run(passive, 15, 200.0);  // 2 s after the sighting
    assert(active.state == State::Attack);
    assert(passive.state == State::Search);
    run(passive, 14, 200.0);  // 30 ticks since the player appeared, the branch is still going
    assert(passive.state == State::Search);
    run(passive, 4, 200.0);  // the branch ends after 32 ticks (87 px of walking, then 3 s of looking)
    assert(passive.state == State::Attack);
}

void fsmReactsOnEveryTickToo() {
    FsmBrain fsm;
    fsm.configure(kNone, false);
    fsm.state = State::Search;
    fsm.body.x = 100.0;
    fsm.body.hasLead = true;
    fsm.body.leadX = 600.0;
    run(fsm, 1, 900.0);
    assert(fsm.state == State::Search);
    run(fsm, 1, 200.0);
    assert(fsm.state == State::Chase);
}

void lowHealthHealsFromAnywhere() {
    const Features f{false, true, false};
    Pair p(f);
    p.fsm.body.x = p.bt.body.x = 100.0;
    p.fsm.body.hp = p.bt.body.hp = 20.0;
    p.run(1, 900.0);
    p.both(State::Heal);
    assert(near(p.fsm.body.x, 118.75) && near(p.bt.body.x, 118.75));  // runs at 150 px/s

    // Standing at the medkit (player far away): 30 hp per second, and it keeps healing past the threshold.
    Pair q(f);
    q.fsm.body.x = q.bt.body.x = kMedkitX;
    q.fsm.body.hp = q.bt.body.hp = 20.0;
    q.run(10, 300.0);
    assert(near(q.fsm.body.hp, 57.5) && near(q.bt.body.hp, 57.5));
    q.both(State::Heal);
    q.run(14, 300.0);  // full after 22 ticks, back to patrol one tick later
    assert(near(q.fsm.body.hp, 100.0) && near(q.bt.body.hp, 100.0));
    q.both(State::Patrol);

    // Without the requirement the same guard just carries on.
    Pair off(kNone);
    off.fsm.body.hp = off.bt.body.hp = 20.0;
    off.run(5, 900.0);
    off.both(State::Patrol);
    assert(near(off.fsm.body.hp, 20.0));
}

void lowHealthBeatsAttack() {
    const Features f{false, true, false};
    Pair p(f);
    p.run(10, 200.0);
    p.both(State::Attack);
    p.fsm.body.hp = p.bt.body.hp = 10.0;
    p.run(1, 200.0);
    p.both(State::Heal);
}

void noiseIsInvestigated() {
    const Features f{true, false, false};
    Pair p(f);
    p.fsm.body.x = p.bt.body.x = 100.0;
    makeNoise(p.fsm.body, 400.0);
    makeNoise(p.bt.body, 400.0);
    p.run(1, 900.0);
    p.both(State::Inspect);
    p.run(19, 900.0);
    p.both(State::Inspect);  // still walking there
    assert(p.fsm.body.x > 250.0 && p.bt.body.x > 250.0);
    p.run(25, 900.0);  // arrives after 30 ticks, listens 1.5 s, then patrols on
    p.both(State::Patrol);

    // Without the requirement the guard does not care.
    Pair off(kNone);
    off.fsm.body.x = off.bt.body.x = 100.0;
    makeNoise(off.fsm.body, 400.0);
    makeNoise(off.bt.body, 400.0);
    off.run(5, 900.0);
    off.both(State::Patrol);
}

void radioRingInterruptsEverything() {
    const Features f{false, false, true};
    Pair p(f);
    p.run(10, 200.0);
    p.both(State::Attack);
    p.fsm.body.radioRinging = p.bt.body.radioRinging = true;
    p.run(1, 200.0);
    p.both(State::Radio);
    p.run(4, 200.0);
    p.both(State::Radio);
}

void historyRemembersWhereTheCallCame() {
    const Features f{false, false, true};
    Pair withHistory(f, true);
    Pair without(f, false);
    for (FsmBrain* fsm : {&withHistory.fsm, &without.fsm}) {
        fsm->state = State::Search;
        fsm->body.x = 100.0;
        fsm->body.hasLead = true;
        fsm->body.leadX = 500.0;
        fsm->body.radioRinging = true;
    }
    withHistory.bt.body = withHistory.fsm.body;
    withHistory.bt.body.radioRinging = true;
    withHistory.run(1, 900.0);
    without.run(1, 900.0);
    assert(withHistory.fsm.state == State::Radio && without.fsm.state == State::Radio);
    assert(withHistory.fsm.remembered == State::Search);
    withHistory.run(4, 900.0);
    assert(withHistory.fsm.state == State::Radio && withHistory.bt.state == State::Radio);
    withHistory.run(17, 900.0);  // 2.5 s of talking, one tick to leave
    without.run(21, 900.0);
    assert(withHistory.fsm.state == State::Search);  // back where it was
    assert(without.fsm.state == State::Patrol);      // the plain FSM only knows the way to Patrol
    assert(withHistory.bt.state == State::Search);   // the tree needs no marker: it looks at the top again
}

void takingAFeatureAwayFallsBack() {
    FsmBrain fsm;
    fsm.configure(kAll, false);
    fsm.state = State::Heal;
    fsm.configure(Features{true, false, true}, false);
    assert(fsm.state == State::Patrol);
    fsm.state = State::Inspect;
    fsm.configure(kNone, false);
    assert(fsm.state == State::Patrol);
    fsm.state = State::Chase;
    fsm.configure(kAll, false);
    assert(fsm.state == State::Chase);  // a state that still exists is kept
}

void historyTargetThatVanishedIsPatrol() {
    FsmBrain fsm;
    fsm.configure(kAll, true);
    fsm.remembered = State::Heal;
    fsm.configure(Features{true, false, true}, true);
    assert(fsm.remembered == State::Patrol);
}

// ---- drawing helper ------------------------------------------------------------------------------------------------

void boxEdge() {
    Pt p = boxEdgePoint({0, 0}, {1, 0}, 38, 13);
    assert(near(p.x, 38) && near(p.y, 0));
    p = boxEdgePoint({10, 10}, {0, -1}, 38, 13);
    assert(near(p.x, 10) && near(p.y, -3));
    const double d = std::sqrt(0.5);
    p = boxEdgePoint({0, 0}, {d, d}, 38, 13);  // 45 degrees leaves through the top or bottom, the box is flat
    assert(near(p.x, 13) && near(p.y, 13));
    p = boxEdgePoint({5, 7}, {0, 0}, 38, 13);  // no direction: the center, not a division by zero
    assert(near(p.x, 5) && near(p.y, 7));
    p = boxEdgePoint({0, 0}, {1, 0}, 38, 13, {0, 5});  // a ray next to the middle line leaves on the same side
    assert(near(p.x, 38) && near(p.y, 5));
    p = boxEdgePoint({0, 0}, {0, 1}, 38, 13, {5, 0});
    assert(near(p.x, 5) && near(p.y, 13));
    p = boxEdgePoint({0, 0}, {-1, 0}, 38, 13, {0, -5});
    assert(near(p.x, -38) && near(p.y, -5));
    p = boxEdgePoint({0, 0}, {d, d}, 38, 13, {0, -5});  // diagonal from above the middle: 18 px of height left, 18 px along y
    assert(near(p.x, 18) && near(p.y, 13));
}

void names() {
    assert(std::string(name(State::Inspect)) == "Inspect");
    assert(std::string(whenLabel(When::LostSight)) == "lost sight");
}

}  // namespace

int main() {
    sequenceResults();
    selectorResults();
    passiveSelectorKeepsTheRunningChild();
    activeSelectorChecksFromTheTop();
    abortResetsSequenceProgress();
    abortingAFinishedNodeDoesNothing();
    walking();
    senses();
    seeingRemembersAndCancelsNoise();
    hearing();
    edgeCounts();
    stateCounts();
    nodeCounts();
    edgesAreWellFormed();
    changeCosts();
    patrolling();
    chaseThenAttack();
    losingThePlayerEndsInPatrol();
    passiveSelectorIsDeaf();
    fsmReactsOnEveryTickToo();
    lowHealthHealsFromAnywhere();
    lowHealthBeatsAttack();
    noiseIsInvestigated();
    radioRingInterruptsEverything();
    historyRemembersWhereTheCallCame();
    takingAFeatureAwayFallsBack();
    historyTargetThatVanishedIsPatrol();
    boxEdge();
    names();
    std::puts("behavior-tree-vs-fsm logic: all asserts passed");
    return 0;
}
