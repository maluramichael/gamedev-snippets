// Tests for the possibility-map logic. Plain asserts on hand-computed cases, no framework, no SDL.
//
// Patterns for one-row grids use one letter per cell: '.' not possible, 'o' possible (medipack not taken),
// 'x' possible (medipack taken), 'b' both. 'M' in the map text marks the medipack, 'P' the player, 'G' the guard post.
#undef NDEBUG
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>
#include <string>
#include <vector>

#include "logic.hpp"

using namespace pm;

namespace {

constexpr double kPi4 = kPi / 4.0;

bool near(double a, double b, double tol = 1e-9) { return std::fabs(a - b) < tol; }

std::vector<char> none(const Grid& g) { return std::vector<char>(static_cast<std::size_t>(g.size()), 0); }

std::vector<char> only(const Grid& g, std::initializer_list<int> cells) {
    std::vector<char> m = none(g);
    for (const int c : cells) m[static_cast<std::size_t>(c)] = 1;
    return m;
}

std::string pattern(const Sim& s) {
    std::string out;
    for (const std::uint8_t m : s.possible()) {
        out += m == 0 ? '.' : m == kFree ? 'o' : m == kTaken ? 'x' : 'b';
    }
    return out;
}

Sim corridor(const std::string& row, std::uint64_t seed = 1, bool honest = true) { return Sim(Grid({row}), {}, seed, honest); }

bool anyIn(const Sim& s, const std::vector<char>& mask) {
    for (std::size_t i = 0; i < mask.size(); ++i) {
        if (mask[i] != 0 && s.possible()[i] != 0) return true;
    }
    return false;
}

void parsingAndDistances() {
    const Grid g({"..#..", "....."});
    assert(g.width() == 5 && g.height() == 2 && g.size() == 10);
    assert(g.wall(2, 0) && g.floor(0, 0) && g.wall(-1, 0) && g.wall(5, 1) && g.wall(0, 2));  // outside counts as wall
    const std::vector<int> d = distancesFrom(g, g.index(0, 0));
    const std::vector<int> expected{0, 1, -1, 5, 6, 1, 2, 3, 4, 5};
    assert(d == expected);
    const Grid sealed({".#."});
    assert(distancesFrom(sealed, 0)[2] == -1);  // unreachable stays -1
    assert(distancesFrom(sealed, 1)[0] == -1);  // a wall is no source
}

void stepTowardFollowsTheDistanceField() {
    const Grid g({".....", "..#.."});
    const std::vector<int> d = distancesFrom(g, g.index(4, 1));
    int at = g.index(0, 0);
    int steps = 0;
    while (at != g.index(4, 1) && steps < 20) {
        const int next = stepToward(g, d, at);
        assert(next != at);
        assert(d[static_cast<std::size_t>(next)] == d[static_cast<std::size_t>(at)] - 1);
        at = next;
        ++steps;
    }
    assert(steps == d[static_cast<std::size_t>(g.index(0, 0))] && steps == 5);
    assert(stepToward(g, d, g.index(4, 1)) == g.index(4, 1));  // already there
    assert(stepToward(g, d, g.index(2, 1)) == g.index(2, 1));  // a wall cell has no way out
}

void spreadOnACorridor() {
    Sim s = corridor(".........");
    s.seed(4, 0);
    assert(pattern(s) == "....o....");
    s.tick();
    assert(pattern(s) == "...ooo...");  // one cell per tick in each direction, the old cell stays possible
    s.tick();
    assert(pattern(s) == "..ooooo..");
    s.tick();
    s.tick();
    assert(pattern(s) == "ooooooooo");
    s.tick();
    assert(pattern(s) == "ooooooooo");  // nothing left to grow into, nothing shrinks
}

void spreadStopsAtWalls() {
    Sim s = corridor("..#..");
    s.seed(0, 0);
    for (int i = 0; i < 10; ++i) s.tick();
    assert(pattern(s) == "oo...");  // the wall cell shows as "." too
}

void spreadIsFourWay() {
    const Grid open({".....", ".....", ".....", ".....", "....."});
    Sim s(open, {}, 1);
    s.seed(open.index(2, 2), 0);
    s.tick();
    assert(s.cells() == 5);  // plus shape, no diagonals
    assert(s.possible()[static_cast<std::size_t>(open.index(1, 1))] == 0);
    s.tick();
    assert(s.cells() == 13);  // diamond of radius 2: 1 + 4 + 8
    assert(s.possible()[static_cast<std::size_t>(open.index(0, 0))] == 0);
    assert(s.possible()[static_cast<std::size_t>(open.index(0, 2))] != 0);
    s.seed(open.index(2, 2), 0);
    Sim ball(open, {}, 1);
    ball.seed(open.index(2, 2), 2);
    assert(ball.cells() == 13);  // seeding a radius is the same as spreading that many ticks
}

void nobodyStandsWhereThePlayerStands() {
    // In a one-cell corridor the player is a plug: the cloud cannot get past him.
    Sim s = corridor(".........");
    s.setPlayer(3);
    s.seed(0, 0);
    for (int i = 0; i < 12; ++i) s.tick();
    assert(pattern(s) == "ooo......");
    // Whatever was marked in his cell is crossed off without a decision.
    Sim t = corridor(".........");
    t.seed(4, 8);
    t.setPlayer(4);
    const Observation obs = t.settle(none(t.grid()), none(t.grid()));
    assert(obs.outcome == Outcome::None && t.possible()[4] == 0 && t.cells() == 8);
    // A guard on his rounds stops in front of him.
    Sim g(Grid({"....."}), {4}, 1);
    g.addGuard(0);
    g.setPlayer(2);
    for (int i = 0; i < 6; ++i) g.tick();
    assert(g.guards()[0].cell == 1);
    g.setPlayer(-1);
    g.tick();
    assert(g.guards()[0].cell == 2);
}

void guardNeverLeavesItsOwnCloud() {
    // Whatever step a patrolling guard takes, the cell it reaches is inside what the cloud would have spread to.
    const Grid g(levelRows());
    const std::vector<int> patrol = levelPatrol(g);
    Sim sim(g, patrol, 1);
    sim.addGuard(g.find('G'));
    std::vector<char> visited(static_cast<std::size_t>(g.size()), 0);
    for (int i = 0; i < 400; ++i) {
        const int before = sim.guards()[0].cell;
        sim.tick();
        const int after = sim.guards()[0].cell;
        Sim ref(g, patrol, 1);
        ref.seed(before, 0);
        ref.tick();
        assert(ref.possible()[static_cast<std::size_t>(after)] != 0);
        visited[static_cast<std::size_t>(after)] = 1;
    }
    for (const int w : patrol) assert(visited[static_cast<std::size_t>(w)] != 0);  // the loop really is walked
}

void chanceRule() {
    assert(!appears(0, 5, 0.0));        // nothing observed, nothing to decide
    assert(!appears(0, 5, 0.99));
    assert(appears(5, 5, 0.999999));    // everything observed: the only possibility left, no choice
    assert(appears(5, 5, 0.0));
    assert(appears(1, 4, 0.24));        // 1 of 4 states: rolls below 0.25 appear
    assert(!appears(1, 4, 0.25));       // 0.25 * 4 is exactly 1, not below
    assert(appears(2, 4, 0.49));
    assert(!appears(2, 4, 0.5));
    assert(!appears(1, 0, 0.0));        // empty map
}

void observationForcesADecision() {
    // Nine possible cells, the cone touches cells 0 to 2.
    int appeared = 0;
    int cleared = 0;
    for (std::uint64_t seed = 1; seed <= 300; ++seed) {
        Sim s = corridor(".........", seed);
        s.seed(4, 4);
        assert(pattern(s) == "ooooooooo");
        const Observation obs = s.settle(only(s.grid(), {0, 1, 2}), none(s.grid()));
        assert(obs.seen == 3 && obs.total == 9 && near(obs.chance, 1.0 / 3.0, 1e-12));
        assert(!anyIn(s, only(s.grid(), {0, 1, 2})));  // whatever the AI chose, the seen cells are no longer possible
        if (obs.outcome == Outcome::Appeared) {
            ++appeared;
            assert(s.guards().size() == 1 && s.guards()[0].cell <= 2);
            assert(s.cells() == 0);  // one guard exists, so nowhere else is possible
        } else {
            ++cleared;
            assert(obs.outcome == Outcome::Cleared);
            assert(s.guards().empty());
            assert(pattern(s) == "...oooooo");  // the rest of the map is untouched
        }
    }
    assert(appeared > 0 && cleared > 0);
    assert(appeared + cleared == 300);
}

void forcedWhenEverythingIsSeen() {
    for (std::uint64_t seed = 1; seed <= 50; ++seed) {
        Sim s = corridor("....", seed);
        s.seed(1, 3);
        const Observation obs = s.settle(only(s.grid(), {0, 1, 2, 3}), none(s.grid()));
        assert(obs.outcome == Outcome::Appeared && near(obs.chance, 1.0, 1e-12));
        assert(s.guards().size() == 1 && s.cells() == 0);
    }
}

void nothingSeenNothingHappens() {
    Sim s = corridor(".........", 3);
    s.seed(7, 0);
    const Observation obs = s.settle(only(s.grid(), {0, 1, 2}), none(s.grid()));
    assert(obs.outcome == Outcome::None && obs.seen == 0);
    assert(pattern(s) == ".......o.");
    assert(s.guards().empty());
}

void aGuardOutOfSightBecomesACloudAgain() {
    Sim s = corridor(".........", 1);
    s.addGuard(4);
    // The guard is in view: it stays a real guard, the map stays empty.
    s.settle(only(s.grid(), {3, 4, 5}), none(s.grid()));
    assert(s.guards().size() == 1 && s.cells() == 0);
    // The cone turns away: the guard dissolves into exactly the cell it stood on.
    s.settle(only(s.grid(), {0, 1}), none(s.grid()));
    assert(s.guards().empty());
    assert(pattern(s) == "....o....");
}

void neverTwoInstancesOfTheSameGuard() {
    // Stress: random player, random cone, random options. After every settle either one real guard exists
    // and the map is empty, or no guard exists and the map is not empty. And what is observed is never possible.
    const Grid g(levelRows());
    const std::vector<int> patrol = levelPatrol(g);
    const std::vector<int> floors = [&] {
        std::vector<int> out;
        for (int i = 0; i < g.size(); ++i) {
            if (g.floor(g.cell(i).x, g.cell(i).y)) out.push_back(i);
        }
        return out;
    }();
    long settles = 0;
    long appearances = 0;
    long packDecisions = 0;
    for (std::uint64_t seed = 1; seed <= 120; ++seed) {
        Sim sim = makeLevelSim(seed, true);
        Rng dice(seed * 7919);
        sim.setNoise(seed % 2 == 0);
        if (seed % 3 != 0) sim.setMedipack(g.find('M'));
        for (int step = 0; step < 300; ++step) {
            const int at = floors[static_cast<std::size_t>(dice.next() * static_cast<double>(floors.size()))];
            const double facing = dice.next() * 2.0 * kPi;
            const std::vector<char> vis = visibleFrom(g, g.cell(at), facing, 0.55, 10.0);
            const std::vector<char> heard = sim.heard(at);
            sim.setPlayer(at);
            sim.tick();
            const Observation obs = sim.settle(vis, heard);
            ++settles;
            appearances += obs.outcome == Outcome::Appeared ? 1 : 0;
            packDecisions += obs.pack != PackOutcome::None ? 1 : 0;
            assert(sim.guards().size() <= 1);
            assert(sim.guards().empty() == (sim.cells() > 0));
            assert(sim.possible()[static_cast<std::size_t>(at)] == 0);
            for (std::size_t i = 0; i < vis.size(); ++i) {
                if (vis[i] != 0 || heard[i] != 0) {
                    if (!sim.guards().empty() && static_cast<std::size_t>(sim.guards()[0].cell) == i) continue;
                    assert(sim.possible()[i] == 0);
                }
            }
            if (sim.medipackGone()) {
                for (const std::uint8_t m : sim.possible()) assert((m & kFree) == 0);  // once it is gone, nobody can have left it
            }
            if (sim.medipack() < 0) {
                for (const std::uint8_t m : sim.possible()) assert((m & kTaken) == 0);
            }
        }
    }
    assert(appearances > 100 && packDecisions > 10);  // the stress run really exercised the decisions
    std::printf("stress: %ld settles, %ld appearances, %ld medipack decisions\n", settles, appearances, packDecisions);
}

void naiveSpawnsOneGuardPerCell() {
    Sim s = corridor(".........", 1, false);
    s.seed(4, 4);
    const Observation obs = s.settle(only(s.grid(), {0, 1, 2}), none(s.grid()));
    assert(obs.outcome == Outcome::Appeared && obs.spawned == 3);
    assert(s.guards().size() == 3);  // three instances of the one guard
    assert(s.cells() == 6);          // and the map still claims he is somewhere else
}

void oddsStayConsistentWhileSweeping() {
    // Ten possible cells. The cone sees 2, then 3 more. A guard at a uniformly random cell is seen with
    // 5/10. The AI's two rolls (2/10, then 3/8 of what is left) have to add up to exactly that.
    int seen = 0;
    constexpr int kTrials = 20000;
    for (std::uint64_t seed = 1; seed <= kTrials; ++seed) {
        Sim s = corridor("..........", seed);
        s.seed(4, 9);
        const Observation a = s.settle(only(s.grid(), {0, 1}), none(s.grid()));
        assert(near(a.chance, 0.2, 1e-12));
        const Observation b = s.settle(only(s.grid(), {2, 3, 4}), none(s.grid()));
        if (a.outcome == Outcome::Appeared) {
            ++seen;
        } else {
            assert(near(b.chance, 3.0 / 8.0, 1e-12));
            seen += b.outcome == Outcome::Appeared ? 1 : 0;
        }
    }
    const double rate = static_cast<double>(seen) / kTrials;
    std::printf("sweep: guard seen in %.4f of %d trials (expected 0.5)\n", rate, kTrials);
    assert(std::fabs(rate - 0.5) < 0.02);
}

void medipackForksTheCloud() {
    Sim s(Grid({"........."}), {}, 1);
    s.setMedipack(4);
    s.seed(0, 0);
    for (int i = 0; i < 3; ++i) s.tick();
    assert(pattern(s) == "oooo.....");  // the cloud has not reached the pack yet
    s.tick();
    assert(pattern(s) == "oooob....");  // it arrives: one branch took the pack, one did not
    s.tick();
    assert(pattern(s) == "ooobbb...");  // the branch that took it spreads on from the pack in both directions
    s.tick();
    assert(pattern(s) == "oobbbbb..");
    assert(!s.medipackGone());
}

void medipackDecisionWhenSeen() {
    int taken = 0;
    constexpr int kTrials = 6000;
    for (std::uint64_t seed = 1; seed <= kTrials; ++seed) {
        Sim s(Grid({"........."}), {}, seed);
        s.setMedipack(4);
        s.seed(0, 0);
        for (int i = 0; i < 6; ++i) s.tick();
        assert(pattern(s) == "oobbbbb..");
        // States: free in cells 0..6 (7), taken in cells 2..6 (5). Only the pack cell is in view.
        const Observation obs = s.settle(only(s.grid(), {4}), none(s.grid()));
        assert(obs.pack != PackOutcome::None && near(obs.packChance, 5.0 / 12.0, 1e-12));
        if (obs.pack == PackOutcome::Taken) {
            ++taken;
            assert(s.medipackGone());
            for (const std::uint8_t m : s.possible()) assert((m & kFree) == 0);
        } else {
            // Seen present. If the guard then appears on the pack cell, he picks it up.
            assert(s.medipackGone() == !s.guards().empty());
            for (const std::uint8_t m : s.possible()) assert((m & kTaken) == 0);
        }
        assert(s.possible()[4] == 0);
    }
    const double rate = static_cast<double>(taken) / kTrials;
    std::printf("medipack: taken in %.4f of %d trials (expected %.4f)\n", rate, kTrials, 5.0 / 12.0);
    assert(std::fabs(rate - 5.0 / 12.0) < 0.03);
}

void medipackNeverDecidedWithoutAFork() {
    // Free states only: the pack was never reachable, so looking at it decides nothing and it stays.
    Sim s(Grid({"........."}), {}, 1);
    s.setMedipack(4);
    s.seed(0, 2);
    const Observation obs = s.settle(only(s.grid(), {4}), none(s.grid()));
    assert(obs.pack == PackOutcome::None && !s.medipackGone());
    assert(pattern(s) == "ooo......");
}

void nobodyTakesAPackYouAreLookingAt() {
    // The pack cell is in view, so a guard that steps onto it is an observation, not a silent pickup.
    int appeared = 0;
    for (std::uint64_t seed = 1; seed <= 200; ++seed) {
        Sim s(Grid({"........."}), {}, seed);
        s.setMedipack(4);
        s.seed(3, 0);
        s.settle(only(s.grid(), {4}), none(s.grid()));  // the pack is in view from now on
        s.tick();
        assert(pattern(s) == "..ooo....");  // no taken branch at the pack
        const Observation obs = s.settle(only(s.grid(), {4}), none(s.grid()));
        assert(s.possible()[4] == 0);
        if (obs.outcome == Outcome::Appeared) {
            ++appeared;
            assert(s.guards()[0].cell == 4 && s.medipackGone());  // he stands on it: now it is gone
        } else {
            assert(!s.medipackGone());
        }
    }
    assert(appeared > 0 && appeared < 200);
}

void switchingTheMedipackOff() {
    Sim s(Grid({"........."}), {}, 1);
    s.setMedipack(4);
    s.seed(0, 0);
    for (int i = 0; i < 6; ++i) s.tick();
    s.clearMedipack();
    assert(s.medipack() == -1 && !s.medipackGone());
    assert(pattern(s) == "ooooooo..");
}

void noiseBlocksTheCloudWhileYouListen() {
    // A noisy tile at cell 4, the player at cell 0 (4 steps away, within earshot). The cloud starts at cell 8 and
    // can never get past the noisy tile unnoticed: reaching it is an observation, so the AI either lets the guard
    // appear there or crosses the tile off.
    int appeared = 0;
    for (std::uint64_t seed = 1; seed <= 200; ++seed) {
        Sim s(Grid({"....,...."}), {}, seed);
        s.setNoise(true);
        s.seed(8, 0);
        assert(s.heard(0) == only(s.grid(), {4}));
        for (int i = 0; i < 12; ++i) {
            s.tick();
            const Observation obs = s.settle(none(s.grid()), s.heard(0));
            assert(!anyIn(s, only(s.grid(), {0, 1, 2, 3, 4})));
            if (obs.outcome == Outcome::Appeared) {
                assert(s.guards()[0].cell == 4);  // heard on the tile, and he stays real while you can hear him
                ++appeared;
                break;
            }
        }
    }
    assert(appeared > 0);
    // With the noisy floor off the same cloud simply flows through.
    Sim quiet(Grid({"....,...."}), {}, 1);
    quiet.seed(8, 0);
    for (int i = 0; i < 12; ++i) {
        quiet.tick();
        quiet.settle(none(quiet.grid()), quiet.heard(0));
    }
    assert(pattern(quiet) == "ooooooooo");
}

void earshotIsPathDistance() {
    Sim far(Grid({"....................,......"}), {}, 1);
    far.setNoise(true);
    assert(far.heard(0) == none(far.grid()));  // 20 steps away, earshot is 8
    Sim edge(Grid({"........,....."}), {}, 1);
    edge.setNoise(true);
    assert(edge.heard(0) == only(edge.grid(), {8}));  // exactly 8 steps: heard
    assert(edge.heard(9) == only(edge.grid(), {8}));
    edge.setNoise(false);
    assert(edge.heard(0) == none(edge.grid()));
    // Around the wall it is 5 steps, not the 4.1 of the straight line.
    Sim bend(Grid({"..#..", "....,"}), {}, 1);
    bend.setNoise(true);
    bend.setEarshot(4);
    assert(bend.heard(0) == none(bend.grid()));
    bend.setEarshot(5);
    assert(bend.heard(0) == only(bend.grid(), {9}));
}

void visibilityCone() {
    const Grid open({"...........", "...........", "...........", "...........", "...........", "...........", "...........", "...........", "...........", "...........", "..........."});
    const Cell eye{5, 5};
    auto vis = [&](const Grid& g, double facing, int x, int y) { return visibleFrom(g, eye, facing, kPi4, 5.0)[static_cast<std::size_t>(g.index(x, y))] != 0; };
    assert(vis(open, 0, 5, 5));    // own cell
    assert(vis(open, 0, 8, 5));    // straight ahead
    assert(vis(open, 0, 10, 5));   // exactly at range
    assert(vis(open, 0, 9, 7));    // 26.6 degrees off the axis
    assert(vis(open, 0, 8, 8));    // exactly on the edge of the cone: 45 degrees counts
    assert(!vis(open, 0, 7, 8));   // 56.3 degrees
    assert(!vis(open, 0, 9, 9));   // on the 45 degree edge but 5.66 away, beyond range 5
    assert(!vis(open, 0, 5, 2));   // 90 degrees
    assert(!vis(open, 0, 2, 5));   // behind
    assert(vis(open, kPi / 2, 5, 8) && !vis(open, kPi / 2, 8, 5));   // facing south (y grows downwards)
    assert(vis(open, kPi, 2, 5));
    assert(vis(open, kPi, 2, 4));  // angle -161.6 degrees seen from facing 180 degrees: 18.4 apart across the wrap
    assert(!vis(open, kPi, 8, 5));

    const Grid walled({"...........", "...........", "...........", "...........", "...........", ".......#...", "...........", "...........", "...........", "...........", "..........."});
    assert(vis(walled, 0, 6, 5));
    assert(!vis(walled, 0, 7, 5));   // the wall itself is not a floor cell
    assert(!vis(walled, 0, 8, 5));   // behind the wall
    assert(!vis(walled, 0, 10, 5));
    assert(vis(walled, 0, 8, 6));    // the line to it slips past the wall (6,5) (7,6)
    assert(vis(walled, 0, 8, 4));
    const Grid diagonal({"...........", "...........", "...........", "...........", "...........", "...........", "......#....", "...........", "...........", "...........", "..........."});
    assert(!visibleFrom(diagonal, eye, kPi / 4.0, 0.2, 5.0)[static_cast<std::size_t>(diagonal.index(7, 7))]);  // the diagonal line goes through the corner cell
    assert(visibleFrom(diagonal, eye, kPi / 4.0, 0.2, 5.0)[static_cast<std::size_t>(diagonal.index(6, 5))] == 0);  // 45 degrees off the axis, outside 11.5 degrees
}

void levelIsPlayable() {
    const std::vector<std::string> rows = levelRows();
    assert(rows.size() == 13);
    for (const std::string& r : rows) assert(r.size() == 30);
    const Grid g(rows);
    const int player = g.find('P');
    const int post = g.find('G');
    const int pack = g.find('M');
    assert(player == g.index(6, 7) && post == g.index(24, 2) && pack == g.index(16, 7));
    assert(g.noisy(20, 5) && g.noisy(20, 4));
    const std::vector<int> dist = distancesFrom(g, player);
    for (int i = 0; i < g.size(); ++i) {
        if (g.floor(g.cell(i).x, g.cell(i).y)) assert(dist[static_cast<std::size_t>(i)] >= 0);  // no closed-off cell
    }
    for (const int w : levelPatrol(g)) {
        assert(g.floor(g.cell(w).x, g.cell(w).y) && dist[static_cast<std::size_t>(w)] >= 0);
    }
    assert(dist[static_cast<std::size_t>(post)] >= 0);
    // The noisy door is the only way out of the guard's room: with it closed the post cannot reach the player.
    std::vector<std::string> shut = rows;
    shut[5][20] = '#';
    const Grid closed(shut);
    assert(distancesFrom(closed, closed.find('G'))[static_cast<std::size_t>(closed.find('P'))] == -1);
    // The level sim starts with a cloud that has not reached the player's view yet.
    const Sim sim = makeLevelSim(1, true);
    assert(sim.cells() > 20);
    const std::vector<char> vis = visibleFrom(g, g.cell(player), 0.0, 0.55, 10.0);
    assert(!anyIn(sim, vis));
}

void timeToTheFirstSighting() {
    // The start scene with the player standing still: how long until the guard shows up.
    long total = 0;
    long worst = 0;
    constexpr int kSeeds = 200;
    const Grid g(levelRows());
    const std::vector<char> vis = visibleFrom(g, g.cell(g.find('P')), 0.0, 0.55, 10.0);
    for (std::uint64_t seed = 1; seed <= kSeeds; ++seed) {
        Sim sim = makeLevelSim(seed, true);
        long ticks = 0;
        while (sim.guards().empty() && ticks < 5000) {
            sim.tick();
            sim.settle(vis, none(g));
            ++ticks;
        }
        assert(!sim.guards().empty());  // the chance is above zero whenever the front is in view
        total += ticks;
        worst = std::max(worst, ticks);
    }
    std::printf("start scene: guard appears after %.1f ticks on average, worst %ld (%d seeds, one tick = one cell)\n", static_cast<double>(total) / kSeeds, worst, kSeeds);
}

void sliderMapping() {
    assert(speedFromX(200.0, 200.0, 400.0, 1, 12) == 1);
    assert(speedFromX(400.0, 200.0, 400.0, 1, 12) == 12);
    assert(speedFromX(50.0, 200.0, 400.0, 1, 12) == 1);
    assert(speedFromX(900.0, 200.0, 400.0, 1, 12) == 12);
    assert(speedFromX(300.0, 200.0, 400.0, 1, 12) == 7);  // 6.5 rounds away from zero
    for (int s = 1; s <= 12; ++s) assert(speedFromX(xFromSpeed(s, 200.0, 400.0, 1, 12), 200.0, 400.0, 1, 12) == s);
    assert(near(xFromSpeed(1, 200.0, 400.0, 1, 12), 200.0) && near(xFromSpeed(12, 200.0, 400.0, 1, 12), 400.0));
    assert(near(xFromSpeed(7, 200.0, 400.0, 1, 12), 200.0 + 200.0 * 6.0 / 11.0));
}

void movingOnTheGrid() {
    const Grid g({"..#", "..."});
    assert(tryMove(g, {0, 0}, 1, 0).x == 1);
    assert(tryMove(g, {1, 0}, 1, 0).x == 1);   // wall
    assert(tryMove(g, {0, 0}, -1, 0).x == 0);  // outside
    assert(tryMove(g, {0, 0}, 0, -1).y == 0);
    assert(tryMove(g, {1, 0}, 0, 1).y == 1);
    assert(tryMove(g, {1, 1}, 0, 1).y == 1);
}

void rngIsDeterministic() {
    Rng a(42);
    Rng b(42);
    for (int i = 0; i < 100; ++i) {
        const double x = a.next();
        assert(x == b.next() && x >= 0.0 && x < 1.0);
    }
    assert(Rng(1).next() != Rng(2).next());
}

}  // namespace

int main() {
    parsingAndDistances();
    stepTowardFollowsTheDistanceField();
    spreadOnACorridor();
    spreadStopsAtWalls();
    spreadIsFourWay();
    nobodyStandsWhereThePlayerStands();
    guardNeverLeavesItsOwnCloud();
    chanceRule();
    observationForcesADecision();
    forcedWhenEverythingIsSeen();
    nothingSeenNothingHappens();
    aGuardOutOfSightBecomesACloudAgain();
    neverTwoInstancesOfTheSameGuard();
    naiveSpawnsOneGuardPerCell();
    oddsStayConsistentWhileSweeping();
    medipackForksTheCloud();
    medipackDecisionWhenSeen();
    medipackNeverDecidedWithoutAFork();
    nobodyTakesAPackYouAreLookingAt();
    switchingTheMedipackOff();
    noiseBlocksTheCloudWhileYouListen();
    earshotIsPathDistance();
    visibilityCone();
    levelIsPlayable();
    timeToTheFirstSighting();
    sliderMapping();
    movingOnTheGrid();
    rngIsDeterministic();
    std::puts("possibility-map: all tests passed");
    return 0;
}
