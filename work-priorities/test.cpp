// Tests for the work-priorities logic. Plain asserts on hand-computed cases, no framework, no SDL.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>

#include "logic.hpp"

using namespace wp;

namespace {

bool near(double a, double b, double eps = 1e-9) { return std::fabs(a - b) < eps; }

Grid uniform(int build, int haul, int cook) {
    Grid g{};
    for (Row& row : g) row = {build, haul, cook};
    return g;
}

void claim(std::vector<Task>& tasks, int i) { tasks[static_cast<std::size_t>(i)].claimed = true; }

// Run a hand-built world until it reports finished.
void runToEnd(World& w) {
    for (int i = 0; i < 100000 && !w.finished; ++i) step(w, kStep);
    assert(w.finished);
}

World lonelyWorld(const std::vector<Task>& tasks, Vec2 start, Vec2 stock) {
    World w;
    w.tasks = tasks;
    w.stock = stock;
    w.grid = {Row{1, 1, 1}, Row{0, 0, 0}, Row{0, 0, 0}};  // only colonist 0 works
    for (Colonist& c : w.colonists) c.pos = start;
    return w;
}

void smallestNumberFirst() {
    const std::vector<Task> tasks{makeTask(Kind::Build, {10, 0}), makeTask(Kind::Haul, {500, 0}), makeTask(Kind::Cook, {20, 0})};
    // Haul has the smallest number, so it wins although it is the farthest.
    assert(pickTask(tasks, Row{2, 1, 3}, {0, 0}, Rule::Priority) == 1);
    assert(pickTask(tasks, Row{3, 2, 1}, {0, 0}, Rule::Priority) == 2);
    assert(pickTask(tasks, Row{1, 4, 4}, {0, 0}, Rule::Priority) == 0);
}

void tieGoesToTheLeftColumn() {
    // Haul is much closer, but Build is further left and has the same number.
    const std::vector<Task> tasks{makeTask(Kind::Build, {900, 0}), makeTask(Kind::Haul, {1, 0}), makeTask(Kind::Cook, {2, 0})};
    assert(pickTask(tasks, Row{2, 2, 0}, {0, 0}, Rule::Priority) == 0);
    assert(pickTask(tasks, Row{0, 3, 3}, {0, 0}, Rule::Priority) == 1);  // Haul beats Cook on the tie
    assert(pickTask(tasks, Row{4, 4, 4}, {0, 0}, Rule::Priority) == 0);
}

void emptyCellNever() {
    const std::vector<Task> tasks{makeTask(Kind::Build, {10, 0}), makeTask(Kind::Haul, {20, 0}), makeTask(Kind::Cook, {30, 0})};
    assert(pickTask(tasks, Row{0, 0, 4}, {0, 0}, Rule::Priority) == 2);  // the lowest priority still beats nothing
    assert(pickTask(tasks, Row{0, 0, 0}, {0, 0}, Rule::Priority) == -1);
    assert(pickTask(tasks, Row{0, 0, 0}, {0, 0}, Rule::Score) == -1);
    const std::vector<Task> onlyHaul{makeTask(Kind::Haul, {1, 0})};
    assert(pickTask(onlyHaul, Row{1, 0, 1}, {0, 0}, Rule::Priority) == -1);  // work exists, but this colonist may not do it
    assert(pickTask(onlyHaul, Row{1, 0, 1}, {0, 0}, Rule::Score) == -1);
    assert(pickTask({}, Row{1, 1, 1}, {0, 0}, Rule::Priority) == -1);
}

void oneLevelBeforeTheNext() {
    std::vector<Task> tasks{makeTask(Kind::Haul, {10, 0}), makeTask(Kind::Haul, {20, 0}), makeTask(Kind::Build, {5, 0})};
    const Row row{2, 1, 3};
    int a = pickTask(tasks, row, {0, 0}, Rule::Priority);
    assert(a == 0);
    claim(tasks, a);
    a = pickTask(tasks, row, {0, 0}, Rule::Priority);
    assert(a == 1);  // still hauling, although the wall is 5 px away
    claim(tasks, a);
    a = pickTask(tasks, row, {0, 0}, Rule::Priority);
    assert(a == 2);  // only now the next level
    claim(tasks, a);
    assert(pickTask(tasks, row, {0, 0}, Rule::Priority) == -1);
}

void doneTasksAreSkipped() {
    std::vector<Task> tasks{makeTask(Kind::Haul, {10, 0}), makeTask(Kind::Haul, {20, 0})};
    tasks[0].done = true;
    assert(pickTask(tasks, Row{1, 1, 1}, {0, 0}, Rule::Priority) == 1);
    tasks[1].done = true;
    assert(pickTask(tasks, Row{1, 1, 1}, {0, 0}, Rule::Priority) == -1);
}

void nearestWithinAKind() {
    const std::vector<Task> tasks{makeTask(Kind::Haul, {0, 0}), makeTask(Kind::Haul, {150, 0}), makeTask(Kind::Haul, {300, 0})};
    assert(pickTask(tasks, Row{1, 1, 1}, {100, 0}, Rule::Priority) == 1);  // 100, 50, 200
    assert(pickTask(tasks, Row{1, 1, 1}, {290, 0}, Rule::Priority) == 2);
    const std::vector<Task> twins{makeTask(Kind::Haul, {90, 0}), makeTask(Kind::Haul, {110, 0})};
    assert(pickTask(twins, Row{1, 1, 1}, {100, 0}, Rule::Priority) == 0);  // both 10 away, the first one wins
    assert(pickTask(twins, Row{1, 1, 1}, {100, 0}, Rule::Score) == 0);
}

void scoreTradesDistanceForPriority() {
    // Haul 1 far away, Build 2 right here.
    std::vector<Task> far{makeTask(Kind::Haul, {1000, 0}), makeTask(Kind::Build, {10, 0})};
    const Row row{2, 1, 3};
    assert(pickTask(far, row, {0, 0}, Rule::Priority) == 0);
    assert(pickTask(far, row, {0, 0}, Rule::Score) == 1);  // 10 + 150 = 160 beats 1000
    // Haul 1 at 100: score 100 beats 10 + 150.
    std::vector<Task> mid{makeTask(Kind::Haul, {100, 0}), makeTask(Kind::Build, {10, 0})};
    assert(pickTask(mid, row, {0, 0}, Rule::Score) == 0);
    // Priority 4 costs three steps: 3 * 150 = 450.
    std::vector<Task> four{makeTask(Kind::Haul, {449, 0}), makeTask(Kind::Build, {0, 0})};
    assert(pickTask(four, Row{4, 1, 1}, {0, 0}, Rule::Score) == 0);  // 449 < 0 + 450
    std::vector<Task> five{makeTask(Kind::Haul, {451, 0}), makeTask(Kind::Build, {0, 0})};
    assert(pickTask(five, Row{4, 1, 1}, {0, 0}, Rule::Score) == 1);  // 451 > 450
}

void scoreTies() {
    // Equal score 150: the smaller number wins even though the task is in the right column.
    const std::vector<Task> a{makeTask(Kind::Build, {0, 0}), makeTask(Kind::Haul, {150, 0})};
    assert(pickTask(a, Row{2, 1, 3}, {0, 0}, Rule::Score) == 1);
    // Equal score and equal number: the left column wins.
    const std::vector<Task> b{makeTask(Kind::Haul, {100, 0}), makeTask(Kind::Build, {0, 100})};
    assert(pickTask(b, Row{1, 1, 1}, {0, 0}, Rule::Score) == 1);
    // Score keeps the empty cell out, however close the task is.
    const std::vector<Task> c{makeTask(Kind::Haul, {1, 0}), makeTask(Kind::Build, {900, 0})};
    assert(pickTask(c, Row{3, 0, 3}, {0, 0}, Rule::Score) == 1);
}

void cyclingACell() {
    assert(nextPriority(0, 1) == 1);
    assert(nextPriority(3, 1) == 4);
    assert(nextPriority(4, 1) == 0);  // after 4 comes the empty cell
    assert(nextPriority(0, -1) == 4);
    assert(nextPriority(1, -1) == 0);
}

void stockSlotsDiffer() {
    for (int i = 0; i < 6; ++i) {
        assert(dist(stockSlot(i), {0, 0}) < 30.0);
        for (int j = i + 1; j < 6; ++j) assert(dist(stockSlot(i), stockSlot(j)) > 5.0);
    }
}

void haulTripCountsBothLegs() {
    // Colonist at the origin, stone 60 away, stock another 40 beyond it: 60 + 40 = 100 px at 80 px/s.
    World w = lonelyWorld({makeTask(Kind::Haul, {0, 60})}, {0, 0}, {0, 100});
    runToEnd(w);
    const Result r = summarize(w);
    assert(near(r.walked, 100.0, 1e-6));
    assert(std::fabs(r.seconds - 1.25) < 0.05);
    assert(r.undone == 0);
    assert(r.doneOrder.size() == 1 && r.doneOrder[0] == Kind::Haul);
    assert(w.colonists[1].walked == 0.0 && w.colonists[2].walked == 0.0);
    assert(dist(w.tasks[0].pos, w.stock) < 30.0);  // the stone lies in the stock now
}

void buildWalksThenWorks() {
    // 80 px away: one second of walking, three seconds of work.
    World w = lonelyWorld({makeTask(Kind::Build, {80, 0})}, {0, 0}, {500, 500});
    runToEnd(w);
    const Result r = summarize(w);
    assert(near(r.walked, 80.0, 1e-6));
    assert(std::fabs(r.seconds - 4.0) < 0.05);
    // Standing next to it costs the work time only.
    World v = lonelyWorld({makeTask(Kind::Cook, {0, 0})}, {0, 0}, {500, 500});
    runToEnd(v);
    assert(near(summarize(v).walked, 0.0));
    assert(std::fabs(summarize(v).seconds - kCookSeconds) < 0.05);
}

void nobodyIsStuck() {
    // Cook is empty for everybody: the three meals stay undone and the run still ends.
    const Result r = run(uniform(1, 1, 0), Rule::Priority);
    assert(r.undone == 3);
    assert(r.doneOrder.size() == 11);
    assert(r.seconds < kMaxSeconds);
    const Result s = run(uniform(1, 1, 0), Rule::Score);
    assert(s.undone == 3 && s.doneOrder.size() == 11);
    // Nobody may do anything.
    const Result none = run(uniform(0, 0, 0), Rule::Priority);
    assert(none.undone == 14 && none.walked == 0.0 && none.doneOrder.empty());
}

void claimOrderOfTheScenario() {
    // Haul 1: all six stones are claimed before a single wall or meal.
    World h = makeWorld(uniform(2, 1, 3), Rule::Priority);
    runToEnd(h);
    assert(h.claimOrder.size() == 14);
    for (std::size_t i = 0; i < 6; ++i) assert(h.claimOrder[i] == Kind::Haul);
    // Build 1: the five walls first.
    World b = makeWorld(uniform(1, 2, 3), Rule::Priority);
    runToEnd(b);
    for (std::size_t i = 0; i < 5; ++i) assert(b.claimOrder[i] == Kind::Build);
    // Equal numbers: the left column goes first, so walls, then stones, then meals.
    World e = makeWorld(uniform(1, 1, 1), Rule::Priority);
    runToEnd(e);
    for (std::size_t i = 0; i < 5; ++i) assert(e.claimOrder[i] == Kind::Build);
    for (std::size_t i = 5; i < 11; ++i) assert(e.claimOrder[i] == Kind::Haul);
}

void scenarioFinishesAndIsDeterministic() {
    for (const Rule rule : {Rule::Priority, Rule::Score}) {
        const Result a = run(uniform(2, 1, 3), rule);
        const Result b = run(uniform(2, 1, 3), rule);
        assert(a.undone == 0 && a.doneOrder.size() == 14);
        assert(a.walked == b.walked && a.seconds == b.seconds);
        // Every stone travels from its spot to the stock at least once, so that is a lower bound.
        const World w = makeWorld(uniform(2, 1, 3), rule);
        double bound = 0.0;
        for (const Task& t : w.tasks) {
            if (t.kind == Kind::Haul) bound += dist(t.pos, w.stock);
        }
        assert(a.walked > bound);
    }
}

}  // namespace

int main() {
    smallestNumberFirst();
    tieGoesToTheLeftColumn();
    emptyCellNever();
    oneLevelBeforeTheNext();
    doneTasksAreSkipped();
    nearestWithinAKind();
    scoreTradesDistanceForPriority();
    scoreTies();
    cyclingACell();
    stockSlotsDiffer();
    haulTripCountsBothLegs();
    buildWalksThenWorks();
    nobodyIsStuck();
    claimOrderOfTheScenario();
    scenarioFinishesAndIsDeterministic();
    std::puts("work-priorities logic: all asserts passed");
    return 0;
}
