// Tests for the Petri-net arbiter logic. Plain asserts on hand-computed cases, no framework, no SDL.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "logic.hpp"

using namespace pa;

namespace {

bool near(double a, double b) { return std::fabs(a - b) < 1e-9; }

Net tiny(std::vector<int> marking, std::vector<Transition> transitions) { return Net{std::move(marking), std::move(transitions)}; }

void firingNeedsATokenInEveryInputPlace() {
    // p0 and p1 feed t0, which produces into p2.
    Net net = tiny({1, 0, 0}, {Transition{{{0, 1}, {1, 1}}, {{2, 1}}}});
    assert(!enabled(net, 0));  // p1 is empty
    assert(!fire(net, 0));
    assert((net.marking == std::vector<int>{1, 0, 0}));  // a refused firing changes nothing
    net.marking[1] = 1;
    assert(enabled(net, 0));
    assert(fire(net, 0));
    assert((net.marking == std::vector<int>{0, 0, 1}));
    assert(!enabled(net, 0));
}

void arcWeightsConsumeAndProduceSeveralTokens() {
    Net net = tiny({2, 0}, {Transition{{{0, 3}}, {{1, 2}}}});
    assert(!enabled(net, 0));  // two tokens, three needed
    net.marking[0] = 3;
    assert(fire(net, 0));
    assert((net.marking == std::vector<int>{0, 2}));
    net.marking[0] = 7;
    assert(fire(net, 0));
    assert((net.marking == std::vector<int>{4, 4}));  // 7 - 3 = 4 left, 2 + 2 = 4 produced
}

void sourceAndSinkTransitions() {
    Net net = tiny({0, 2}, {Transition{{}, {{0, 1}}}, Transition{{{1, 1}}, {}}});
    assert(enabled(net, 0));  // no input place: always enabled
    assert(fire(net, 0) && fire(net, 0));
    assert(net.marking[0] == 2);
    assert(fire(net, 1));  // sink: consumes and produces nothing
    assert(net.marking[1] == 1);
    assert(fire(net, 1));
    assert(!enabled(net, 1));
}

void conflictMeansFiringOneDisablesTheOther() {
    // t0 and t1 both take the single token of p0.
    Net net = tiny({1, 0, 0}, {Transition{{{0, 1}}, {{1, 1}}}, Transition{{{0, 1}}, {{2, 1}}}});
    assert(inConflict(net, 0, 1));
    assert(inConflict(net, 1, 0));
    assert(!inConflict(net, 0, 0));  // a transition does not conflict with itself
    net.marking[0] = 2;
    assert(!inConflict(net, 0, 1));  // enough tokens for both, in either order
    net.marking[0] = 0;
    assert(!inConflict(net, 0, 1));  // neither is enabled
    // Disjoint inputs never conflict.
    Net apart = tiny({1, 1}, {Transition{{{0, 1}}, {}}, Transition{{{1, 1}}, {}}});
    assert(!inConflict(apart, 0, 1));
}

void inConflictLeavesTheNetAlone() {
    Net net = tiny({1, 0, 0}, {Transition{{{0, 1}}, {{1, 1}}}, Transition{{{0, 1}}, {{2, 1}}}});
    (void)inConflict(net, 0, 1);
    assert((net.marking == std::vector<int>{1, 0, 0}));
}

void arbiterNetStartsWithTheGunFree() {
    const Net net = makeArbiterNet(3);
    assert((net.marking == std::vector<int>{3, 0, 0, 0, 1}));  // Spawned, ReadyToAssign, ReadyToUse, GunBusy, GunAvailable
    assert(enabled(net, Register));
    assert(!enabled(net, Start));
    assert(!enabled(net, Assign));  // nobody is ready
    assert(!enabled(net, Incapacitated));
    assert(!enabled(net, Dropout));
}

void startNeedsAllAgentsRegistered() {
    Net net = makeArbiterNet(3);
    assert(fire(net, Register) && fire(net, Register));
    assert(!enabled(net, Start));  // 2 of 3
    assert(fire(net, Register));
    assert(!enabled(net, Register));  // everybody registered
    assert(enabled(net, Start));
    assert(fire(net, Start));
    assert((net.marking == std::vector<int>{0, 0, 3, 0, 1}));  // all three moved at once
}

void theGunIsNeverGivenTwice() {
    Net net = makeArbiterNet(3);
    for (int i = 0; i < 3; ++i) fire(net, Register);
    fire(net, Start);
    assert(fire(net, Assign));
    assert((net.marking == std::vector<int>{0, 0, 2, 1, 0}));
    assert(!enabled(net, Assign));  // two agents ready, but the gun is busy
    assert(!fire(net, Assign));
    assert(fire(net, Incapacitated));  // the gunner dies
    assert((net.marking == std::vector<int>{0, 0, 2, 0, 1}));
    assert(enabled(net, Assign));
    assert(fire(net, Assign));
    assert((net.marking == std::vector<int>{0, 0, 1, 1, 0}));
}

void assignAndDropoutCompeteForTheLastToken() {
    Net net = makeArbiterNet(1);
    fire(net, Register);
    fire(net, Start);
    assert(inConflict(net, Assign, Dropout));  // one token in ReadyToUse, both want it
    Net more = makeArbiterNet(2);
    fire(more, Register);
    fire(more, Register);
    fire(more, Start);
    assert(!inConflict(more, Assign, Dropout));  // two tokens: both can happen
}

Candidate cand(std::size_t id, int order, double x, double y, double accuracy) { return Candidate{id, order, Vec2{x, y}, accuracy}; }

void fifoTakesTheLowestOrderNotTheFirstInTheList() {
    const Vec2 gun{0, 0};
    const std::vector<Candidate> list{cand(7, 2, 10, 0, 0.5), cand(8, 0, 500, 0, 0.5), cand(9, 1, 20, 0, 0.5)};
    const auto i = pick(Policy::Fifo, list, gun);
    assert(i && list[*i].id == 8);  // order 0, although it is the farthest
}

void nearestTakesTheSmallestDistance() {
    const Vec2 gun{0, 0};
    const std::vector<Candidate> list{cand(1, 0, 100, 0, 1.0), cand(2, 1, 30, 40, 0.1), cand(3, 2, 0, -60, 1.0)};
    const auto i = pick(Policy::Nearest, list, gun);
    assert(i && list[*i].id == 2);  // distance 50 (3-4-5), the others 100 and 60
    // Equal distance: the earlier order wins.
    const std::vector<Candidate> tie{cand(1, 3, 60, 0, 0.5), cand(2, 1, 0, 60, 0.5)};
    const auto t = pick(Policy::Nearest, tie, gun);
    assert(t && tie[*t].id == 2);
}

void scoreWeighsDistanceAndAccuracy() {
    const Vec2 gun{0, 0};
    // A: closeness 1 - 100/640 = 0.84375, score 0.6 * 0.84375 + 0.4 * 0.2 = 0.58625
    // B: closeness 1 - 200/640 = 0.6875,  score 0.6 * 0.6875 + 0.4 * 1.0 = 0.8125
    assert(near(score({100, 0}, 0.2, gun), 0.58625));
    assert(near(score({200, 0}, 1.0, gun), 0.8125));
    const std::vector<Candidate> list{cand(1, 0, 100, 0, 0.2), cand(2, 1, 200, 0, 1.0)};
    const auto s = pick(Policy::Score, list, gun);
    assert(s && list[*s].id == 2);  // the better shot wins
    const auto n = pick(Policy::Nearest, list, gun);
    assert(n && list[*n].id == 1);  // the closer one wins here, so the two policies disagree
}

void scoreClampsItsInputs() {
    const Vec2 gun{0, 0};
    assert(near(score({1000, 0}, 0.5, gun), 0.2));  // far beyond the range: closeness 0, only accuracy counts
    assert(near(score({0, 0}, 2.0, gun), 1.0));     // accuracy above 1 counts as 1
    assert(near(score({0, 0}, -1.0, gun), 0.6));    // below 0 counts as 0
}

void pickOfNobodyIsNothing() {
    const std::vector<Candidate> none;
    for (const Policy p : {Policy::Fifo, Policy::Nearest, Policy::Score}) assert(!pick(p, none, Vec2{0, 0}));
}

void statsMeansAreZeroWithoutData() {
    const Stats s;
    assert(near(meanDistance(s), 0.0) && near(meanArrival(s), 0.0));
    Stats t;
    t.assignments = 2;
    t.sumDistance = 300.0;
    t.sumArrival = 3.0;
    assert(near(meanDistance(t), 150.0) && near(meanArrival(t), 1.5));
}

void roundsAreDeterministicAndInsideTheField() {
    const std::vector<Agent> a = makeAgents(5, kMaxAgents);
    const std::vector<Agent> b = makeAgents(5, kMaxAgents);
    const std::vector<Agent> c = makeAgents(6, kMaxAgents);
    assert(a.size() == static_cast<std::size_t>(kMaxAgents));
    bool differs = false;
    for (std::size_t i = 0; i < a.size(); ++i) {
        assert(near(a[i].pos.x, b[i].pos.x) && near(a[i].pos.y, b[i].pos.y) && near(a[i].accuracy, b[i].accuracy));
        differs = differs || !near(a[i].pos.x, c[i].pos.x);
    }
    assert(differs);  // another round, another layout
    for (std::uint32_t round = 0; round < 200; ++round) {
        const std::vector<Agent> agents = makeAgents(round, kMaxAgents);
        for (std::size_t i = 0; i < agents.size(); ++i) {
            const Agent& g = agents[i];
            assert(g.pos.x >= kFieldLeft && g.pos.x <= kFieldRight && g.pos.y >= kFieldTop && g.pos.y <= kFieldBottom);
            assert(g.accuracy >= kAccuracyMin && g.accuracy <= 1.0);
            assert(g.state == State::Spawning);
            for (std::size_t j = 0; j < i; ++j) assert(dist(g.pos, agents[j].pos) >= kMinSeparation);
        }
    }
}

void theShowcaseRoundHasThreeDifferentAnswers() {
    const std::vector<Agent> agents = makeAgents(kShowcaseRound, 6);
    std::vector<Candidate> list;
    for (std::size_t i = 0; i < agents.size(); ++i) list.push_back(Candidate{i, static_cast<int>(i), agents[i].pos, agents[i].accuracy});
    const Vec2 gun = Sim{}.gun;
    const auto f = pick(Policy::Fifo, list, gun);
    const auto n = pick(Policy::Nearest, list, gun);
    const auto s = pick(Policy::Score, list, gun);
    assert(f && n && s);
    assert(*f == 0 && dist(list[*f].pos, gun) > 650.0);  // the first to register is far away
    assert(*n == 5 && dist(list[*n].pos, gun) < 100.0);
    assert(*s == 2);
    assert(makeSim(Policy::Fifo, 6, kShowcaseRound).round == kShowcaseRound);
}

void countIsClamped() {
    assert(makeAgents(0, 0).size() == static_cast<std::size_t>(kMinAgents));
    assert(makeAgents(0, 99).size() == static_cast<std::size_t>(kMaxAgents));
    assert(makeAgents(0, 4).size() == 4U);
}

std::vector<Agent> twoAgents() {
    // Agent 0 registers first and is 700 px from the gun at (800, 150); agent 1 is 100 px away.
    return {Agent{{100, 150}, 0.5, State::Spawning}, Agent{{700, 150}, 0.5, State::Spawning}};
}

void runFor(Sim& sim, double seconds) {
    for (double t = 0.0; t < seconds - 1e-9; t += 0.05) advance(sim, 0.05);
}

void fifoSendsTheFarAgentNearestTheClose() {
    Sim fifo = makeSim(Policy::Fifo, 2);
    beginRound(fifo, twoAgents());
    runFor(fifo, 0.7);  // register, register, start, assign = 4 ticks of 0.15 s
    assert(fifo.gunner == 0);
    assert(fifo.agents[0].state == State::Walking);
    assert(fifo.agents[1].state == State::Waiting);
    assert(fifo.stats.assignments == 1);
    assert(near(meanDistance(fifo.stats), 700.0));
    assert(near(meanArrival(fifo.stats), 700.0 / kSpeed));  // 5 s

    Sim nearest = makeSim(Policy::Nearest, 2);
    beginRound(nearest, twoAgents());
    runFor(nearest, 0.7);
    assert(nearest.gunner == 1);
    assert(near(meanDistance(nearest.stats), 100.0));
    assert(near(meanArrival(nearest.stats), 100.0 / kSpeed));
}

void theNetNeedsOneTickPerFiring() {
    Sim sim = makeSim(Policy::Nearest, 2);
    beginRound(sim, twoAgents());
    advance(sim, kTick * 0.5);
    assert(sim.events.empty());  // not a full tick yet
    advance(sim, kTick * 0.6);
    assert(sim.events.size() == 1 && sim.events[0].transition == Register);
    assert(sim.agents[0].state == State::Registered && sim.agents[1].state == State::Spawning);
}

void theGunnerWalksThenMansTheGun() {
    Sim sim = makeSim(Policy::Nearest, 2);
    beginRound(sim, twoAgents());
    runFor(sim, 0.7);
    assert(sim.agents[1].state == State::Walking);
    runFor(sim, 0.5);  // 100 px at 140 px/s needs 0.71 s in total, 0.5 s are not enough
    assert(sim.agents[1].state == State::Walking);
    runFor(sim, 0.4);
    assert(sim.agents[1].state == State::Manning);
    assert(near(dist(sim.agents[1].pos, sim.gun), 0.0));  // arrives exactly, no overshoot
}

void killingTheGunnerHandsTheGunOver() {
    Sim sim = makeSim(Policy::Nearest, 2);
    sim.autoKill = false;
    beginRound(sim, twoAgents());
    runFor(sim, 0.7);
    assert(sim.gunner == 1);
    assert(killGunner(sim));
    assert(sim.agents[1].state == State::Dead && sim.gunner == -1);
    assert(sim.net.marking[GunBusy] == 0 && sim.net.marking[GunAvailable] == 1);
    assert(!killGunner(sim));  // nobody mans it now
    runFor(sim, kTick + 0.05);
    assert(sim.gunner == 0);  // the only one left gets it
    assert(sim.stats.assignments == 2);
}

void killingAWaitingAgentRemovesItsToken() {
    Sim sim = makeSim(Policy::Fifo, 2);
    sim.autoKill = false;
    beginRound(sim, twoAgents());
    runFor(sim, 0.7);
    assert(sim.net.marking[ReadyToUse] == 1);
    assert(kill(sim, 1));  // the one still waiting
    assert(sim.net.marking[ReadyToUse] == 0);
    assert(sim.gunner == 0);  // the gunner is untouched
    assert(!kill(sim, 1));    // already dead
    assert(!kill(sim, 5));    // no such agent
}

void agentsStillRegisteringCannotBeKilled() {
    Sim sim = makeSim(Policy::Fifo, 2);
    beginRound(sim, twoAgents());
    assert(!kill(sim, 0));
    advance(sim, kTick + 0.01);
    assert(sim.agents[0].state == State::Registered);
    assert(!kill(sim, 0));
}

void aRoundEndsAndAnotherStarts() {
    Sim sim = makeSim(Policy::Nearest, 2);
    sim.autoKill = false;
    beginRound(sim, twoAgents());
    runFor(sim, 0.7);
    assert(killGunner(sim));
    runFor(sim, kTick + 0.05);
    assert(killGunner(sim));
    assert(sim.net.marking[GunAvailable] == 1 && sim.net.marking[GunBusy] == 0);
    const std::uint32_t round = sim.round;
    runFor(sim, kRestartDelay - 0.2);
    assert(sim.round == round);  // still showing the empty field
    runFor(sim, 0.5);
    assert(sim.round == round + 1);
    assert(sim.agents.size() == 2U);
    assert(sim.agents[0].state != State::Dead && sim.agents[1].state != State::Dead);
}

void autoKillShootsTheGunnerAfterItManned() {
    Sim sim = makeSim(Policy::Nearest, 2);
    sim.autoKill = true;
    beginRound(sim, twoAgents());
    runFor(sim, 0.7 + 100.0 / kSpeed + 0.1);  // arrived
    assert(sim.agents[1].state == State::Manning);
    runFor(sim, kManTime - 0.4);
    assert(sim.agents[1].state == State::Manning);
    runFor(sim, 0.8);
    assert(sim.agents[1].state == State::Dead);
    sim.autoKill = false;
    Sim quiet = makeSim(Policy::Nearest, 2);
    quiet.autoKill = false;
    beginRound(quiet, twoAgents());
    runFor(quiet, 10.0);
    assert(quiet.agents[1].state == State::Manning);  // nobody shoots without auto-kill
}

void changingPolicyOrCountResetsTheStats() {
    Sim sim = makeSim(Policy::Fifo, 2);
    beginRound(sim, twoAgents());
    runFor(sim, 0.7);
    assert(sim.stats.assignments == 1);
    setPolicy(sim, Policy::Fifo);
    assert(sim.stats.assignments == 1);  // same policy: nothing changes
    setPolicy(sim, Policy::Score);
    assert(sim.stats.assignments == 0 && near(sim.stats.sumDistance, 0.0));
    assert(sim.policy == Policy::Score);
    runFor(sim, 0.5);
    setCount(sim, 5);
    assert(sim.count == 5 && sim.agents.size() == 5U && sim.stats.assignments == 0);
    setCount(sim, 50);
    assert(sim.count == kMaxAgents);
}

void invariantsHoldUnderAKillStorm() {
    for (const Policy policy : {Policy::Fifo, Policy::Nearest, Policy::Score}) {
        for (const int n : {2, 5, kMaxAgents}) {
            Sim sim = makeSim(policy, n, 100);
            sim.autoKill = false;
            nextRound(sim);
            Rng rng{static_cast<std::uint32_t>(n * 31 + static_cast<int>(policy))};
            long assignments = 0;
            for (int step = 0; step < 6000; ++step) {
                if (rng.next() < 0.04) {
                    const auto id = static_cast<std::size_t>(rng.next() * static_cast<double>(sim.agents.size()));
                    (void)kill(sim, std::min(id, sim.agents.size() - 1));
                }
                if (rng.next() < 0.01) (void)killGunner(sim);
                advance(sim, 0.03);
                int spawning = 0, registered = 0, waiting = 0, manned = 0, dead = 0;
                for (const Agent& a : sim.agents) {
                    spawning += a.state == State::Spawning;
                    registered += a.state == State::Registered;
                    waiting += a.state == State::Waiting;
                    manned += a.state == State::Walking || a.state == State::Manning;
                    dead += a.state == State::Dead;
                }
                const std::vector<int>& m = sim.net.marking;
                assert(m[Spawned] == spawning && m[ReadyToAssign] == registered && m[ReadyToUse] == waiting);
                assert(m[GunBusy] == manned && manned <= 1);                // the role is never given twice
                assert(m[GunBusy] + m[GunAvailable] == 1);                  // exactly one gun token in the net
                assert(spawning + registered + waiting + manned + dead == static_cast<int>(sim.agents.size()));
                assert(sim.gunner == -1 ? manned == 0 : manned == 1);
                if (sim.gunner >= 0) {
                    const State st = sim.agents[static_cast<std::size_t>(sim.gunner)].state;
                    assert(st == State::Walking || st == State::Manning);
                }
                for (const Transition& t : sim.net.transitions) {
                    for (const Arc& a : t.in) assert(m[a.place] >= 0);
                }
                assignments = sim.stats.assignments;
            }
            assert(assignments > 0);
        }
    }
}

void measureFifoAgainstNearest() {
    // 10000 random layouts of six agents, the gun as in the demo. Prints what the post quotes.
    constexpr int kScenes = 10000;
    constexpr int kAgents = 6;
    double sum[3] = {0, 0, 0};
    int worst = 0;
    for (int scene = 0; scene < kScenes; ++scene) {
        const std::vector<Agent> agents = makeAgents(static_cast<std::uint32_t>(scene), kAgents);
        std::vector<Candidate> list;
        for (std::size_t i = 0; i < agents.size(); ++i) list.push_back(Candidate{i, static_cast<int>(i), agents[i].pos, agents[i].accuracy});
        const Vec2 gun = Sim{}.gun;
        double d[3];
        int k = 0;
        for (const Policy p : {Policy::Fifo, Policy::Nearest, Policy::Score}) {
            const auto i = pick(p, list, gun);
            assert(i);
            d[k] = dist(list[*i].pos, gun);
            sum[k] += d[k];
            ++k;
        }
        assert(d[1] <= d[0] + 1e-9 && d[1] <= d[2] + 1e-9);  // nearest is by definition never farther
        if (d[0] > d[1] * 2.0) ++worst;
    }
    const double fifo = sum[0] / kScenes;
    const double nearest = sum[1] / kScenes;
    const double scored = sum[2] / kScenes;
    std::printf("mean distance of the gunner over %d layouts of %d agents: fifo %.1f, nearest %.1f, score %.1f; fifo more than twice as far as nearest in %.1f%%\n", kScenes, kAgents, fifo, nearest, scored, 100.0 * worst / kScenes);
    assert(fifo > nearest * 1.5);   // FIFO is clearly worse, whatever the exact numbers on another CPU
    assert(scored < fifo);
    assert(scored >= nearest);
}

}  // namespace

int main() {
    firingNeedsATokenInEveryInputPlace();
    arcWeightsConsumeAndProduceSeveralTokens();
    sourceAndSinkTransitions();
    conflictMeansFiringOneDisablesTheOther();
    inConflictLeavesTheNetAlone();
    arbiterNetStartsWithTheGunFree();
    startNeedsAllAgentsRegistered();
    theGunIsNeverGivenTwice();
    assignAndDropoutCompeteForTheLastToken();
    fifoTakesTheLowestOrderNotTheFirstInTheList();
    nearestTakesTheSmallestDistance();
    scoreWeighsDistanceAndAccuracy();
    scoreClampsItsInputs();
    pickOfNobodyIsNothing();
    statsMeansAreZeroWithoutData();
    roundsAreDeterministicAndInsideTheField();
    theShowcaseRoundHasThreeDifferentAnswers();
    countIsClamped();
    fifoSendsTheFarAgentNearestTheClose();
    theNetNeedsOneTickPerFiring();
    theGunnerWalksThenMansTheGun();
    killingTheGunnerHandsTheGunOver();
    killingAWaitingAgentRemovesItsToken();
    agentsStillRegisteringCannotBeKilled();
    aRoundEndsAndAnotherStarts();
    autoKillShootsTheGunnerAfterItManned();
    changingPolicyOrCountResetsTheStats();
    invariantsHoldUnderAKillStorm();
    measureFifoAgainstNearest();
    std::puts("petri-arbiter logic: all tests passed");
    return 0;
}
