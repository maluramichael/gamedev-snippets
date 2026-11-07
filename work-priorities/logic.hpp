// Work priorities as in RimWorld, without SDL.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

namespace wp {

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};

inline double dist(Vec2 a, Vec2 b) { return std::hypot(a.x - b.x, a.y - b.y); }

enum class Kind : int { Build = 0, Haul = 1, Cook = 2 };
inline constexpr int kKinds = 3;
inline constexpr int kColonists = 3;

using Row = std::array<int, kKinds>;
using Grid = std::array<Row, kColonists>;

enum class Rule { Priority, Score };

inline constexpr double kSpeed = 80.0;
inline constexpr double kStep = 1.0 / 60.0;
inline constexpr double kScoreStep = 150.0;
inline constexpr double kBuildSeconds = 3.0;
inline constexpr double kCookSeconds = 2.5;
inline constexpr double kMaxSeconds = 600.0;

struct Task {
    Kind kind = Kind::Build;
    Vec2 pos;
    double work = 0.0;
    double left = 0.0;
    bool claimed = false;
    bool done = false;
};

inline Task makeTask(Kind kind, Vec2 pos) {
    const double work = kind == Kind::Build ? kBuildSeconds : kind == Kind::Cook ? kCookSeconds : 0.0;
    return Task{kind, pos, work, work, false, false};
}

inline int nextPriority(int p, int dir) { return ((p + dir) % 5 + 5) % 5; }

/// Where the n-th delivered stone lies, relative to the stock's center: a small grid of three columns.
inline Vec2 stockSlot(int n) { return {(n % 3 - 1) * 18.0, ((n / 3) % 3 - 1) * 18.0}; }

/// Sort key of a task for one colonist, compared field by field (smaller is better).
struct Key {
    std::array<double, 4> v{};
};

inline bool better(const Key& a, const Key& b) {
    for (std::size_t i = 0; i < a.v.size(); ++i) {
        if (std::fabs(a.v[i] - b.v[i]) > 1e-9) return a.v[i] < b.v[i];
    }
    return false;  // a full tie keeps the task that came first
}

/// The task a colonist standing at `from` takes next, or -1. A cell of 0 (empty) forbids the work type, claimed and
/// finished tasks are not available. `Priority`: smallest number, then the left column, then the nearest task.
/// `Score`: walking distance plus kScoreStep per priority step above 1, then number and column.
inline int pickTask(const std::vector<Task>& tasks, const Row& row, Vec2 from, Rule rule) {
    int best = -1;
    Key bestKey;
    for (std::size_t i = 0; i < tasks.size(); ++i) {
        const Task& t = tasks[i];
        if (t.claimed || t.done) continue;
        const int p = row[static_cast<std::size_t>(t.kind)];
        if (p < 1) continue;
        const double d = dist(from, t.pos);
        const double kind = static_cast<double>(t.kind);
        const Key key = rule == Rule::Priority ? Key{{static_cast<double>(p), kind, d, 0.0}}
                                               : Key{{d + kScoreStep * (p - 1), static_cast<double>(p), kind, d}};
        if (best < 0 || better(key, bestKey)) {
            best = static_cast<int>(i);
            bestKey = key;
        }
    }
    return best;
}

enum class Phase { Idle, ToTask, Working, Carrying };

struct Colonist {
    Vec2 pos;
    Phase phase = Phase::Idle;
    int task = -1;
    double walked = 0.0;
};

struct World {
    std::vector<Task> tasks;
    std::array<Colonist, kColonists> colonists;
    Grid grid{};
    Rule rule = Rule::Priority;
    Vec2 stock;
    double time = 0.0;
    double lastDone = 0.0;
    bool finished = false;
    int dropped = 0;
    std::vector<Kind> claimOrder;
    std::vector<Kind> doneOrder;
};

/// The fixed scenario: a wall to build at the top left, stones lying around it, a stove at the top right, and the
/// stock in the bottom right corner. Tasks are listed in column order.
inline World makeWorld(const Grid& grid, Rule rule) {
    World w;
    w.grid = grid;
    w.rule = rule;
    w.stock = {560.0, 340.0};
    for (int i = 0; i < 5; ++i) w.tasks.push_back(makeTask(Kind::Build, {70.0 + 40.0 * i, 90.0}));
    for (const Vec2 p : {Vec2{90, 150}, Vec2{150, 170}, Vec2{210, 140}, Vec2{120, 230}, Vec2{250, 200}, Vec2{60, 190}}) {
        w.tasks.push_back(makeTask(Kind::Haul, p));
    }
    for (int i = 0; i < 3; ++i) w.tasks.push_back(makeTask(Kind::Cook, {470.0, 110.0}));
    w.colonists[0].pos = {330.0, 230.0};
    w.colonists[1].pos = {360.0, 250.0};
    w.colonists[2].pos = {390.0, 230.0};
    return w;
}

inline void finishTask(World& w, Colonist& c) {
    Task& t = w.tasks[static_cast<std::size_t>(c.task)];
    t.done = true;
    w.doneOrder.push_back(t.kind);
    w.lastDone = w.time;
    c.task = -1;
    c.phase = Phase::Idle;
}

/// Moves `c` toward `target` by at most `reach`, counts what it walked, true on arrival.
inline bool walk(Colonist& c, Vec2 target, double reach) {
    const double d = dist(c.pos, target);
    const double m = std::min(d, reach);
    if (d > 0.0) {
        c.pos.x += (target.x - c.pos.x) / d * m;
        c.pos.y += (target.y - c.pos.y) / d * m;
    }
    c.walked += m;
    return m >= d;
}

/// One fixed step. Idle colonists pick in order (Ann first) and claim at once, so two never take the same task.
inline void step(World& w, double dt) {
    if (w.finished) return;
    w.time += dt;
    for (int i = 0; i < kColonists; ++i) {
        Colonist& c = w.colonists[static_cast<std::size_t>(i)];
        if (c.phase == Phase::Idle) {
            const int t = pickTask(w.tasks, w.grid[static_cast<std::size_t>(i)], c.pos, w.rule);
            if (t >= 0) {
                w.tasks[static_cast<std::size_t>(t)].claimed = true;
                w.claimOrder.push_back(w.tasks[static_cast<std::size_t>(t)].kind);
                c.task = t;
                c.phase = Phase::ToTask;
            }
        }
        if (c.phase == Phase::ToTask) {
            Task& t = w.tasks[static_cast<std::size_t>(c.task)];
            if (walk(c, t.pos, kSpeed * dt)) c.phase = t.kind == Kind::Haul ? Phase::Carrying : Phase::Working;
        } else if (c.phase == Phase::Carrying) {
            if (walk(c, w.stock, kSpeed * dt)) {
                w.tasks[static_cast<std::size_t>(c.task)].pos = {w.stock.x + stockSlot(w.dropped).x, w.stock.y + stockSlot(w.dropped).y};
                ++w.dropped;
                finishTask(w, c);
            }
        } else if (c.phase == Phase::Working) {
            Task& t = w.tasks[static_cast<std::size_t>(c.task)];
            t.left -= dt;
            if (t.left <= 0.0) {
                t.left = 0.0;
                finishTask(w, c);
            }
        }
    }
    // Finished when nobody is busy and nobody could take anything that is left.
    bool busy = false;
    for (int i = 0; i < kColonists; ++i) {
        const Colonist& c = w.colonists[static_cast<std::size_t>(i)];
        busy = busy || c.phase != Phase::Idle || pickTask(w.tasks, w.grid[static_cast<std::size_t>(i)], c.pos, w.rule) >= 0;
    }
    w.finished = !busy;
}

struct Result {
    double walked = 0.0;
    double seconds = 0.0;
    int undone = 0;
    std::vector<Kind> doneOrder;
};

inline Result summarize(const World& w) {
    Result r;
    for (const Colonist& c : w.colonists) r.walked += c.walked;
    r.seconds = w.lastDone;
    for (const Task& t : w.tasks) r.undone += t.done ? 0 : 1;
    r.doneOrder = w.doneOrder;
    return r;
}

/// The whole scenario to the end, for comparing grids and rules without watching.
inline Result run(const Grid& grid, Rule rule) {
    World w = makeWorld(grid, rule);
    const int maxSteps = static_cast<int>(kMaxSeconds / kStep);
    for (int i = 0; i < maxSteps && !w.finished; ++i) step(w, kStep);
    return summarize(w);
}

}  // namespace wp
