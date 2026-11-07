// Logic of the behavior-tree-vs-fsm demo, no SDL. Two guards do the same job on a 1D corridor: patrol, chase, attack,
// search, and (as optional requirements) check a noise, heal at a medkit, take a radio call. One guard is driven by a
// state machine (a table of edges), the other by a behavior tree (selector, sequence, leaves). Both read the same senses
// and use the same walking and timer helpers, so the only difference is how the decision is made.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace gbt {

inline constexpr double kArena = 920.0;       // corridor length
inline constexpr double kSight = 170.0;       // sees the player within this distance
inline constexpr double kHearing = 330.0;     // hears a noise within this distance
inline constexpr double kStrikeRange = 44.0;
inline constexpr double kChaseStop = 38.0;    // chasing stops this far from the player, inside the strike range
inline constexpr double kArrive = 3.0;        // close enough to a target position
inline constexpr double kWalk = 80.0;         // px/s
inline constexpr double kRun = 150.0;
inline constexpr double kStrikeTime = 0.5;
inline constexpr double kLookTime = 3.0;      // looking around at the last sighting
inline constexpr double kListenTime = 1.5;    // listening at the spot of a noise
inline constexpr double kRadioTime = 2.5;
inline constexpr double kMaxHp = 100.0;
inline constexpr double kLowHp = 30.0;
inline constexpr double kHealRate = 30.0;     // hp/s at the medkit
inline constexpr double kMedkitX = 880.0;
inline constexpr std::array<double, 4> kWaypoints{60.0, 260.0, 460.0, 660.0};

/// What a guard is doing. Doubles as the state of the state machine and as the label of a branch of the tree.
enum class State { Patrol, Chase, Search, Attack, Inspect, Heal, Radio };
inline constexpr std::array<State, 7> kAllStates{State::Patrol, State::Chase, State::Search, State::Attack,
                                                 State::Inspect, State::Heal, State::Radio};

inline const char* name(State s) {
    switch (s) {
        case State::Patrol: return "Patrol";
        case State::Chase: return "Chase";
        case State::Search: return "Search";
        case State::Attack: return "Attack";
        case State::Inspect: return "Inspect";
        case State::Heal: return "Heal";
        case State::Radio: return "Radio";
    }
    return "?";
}

/// The optional requirements. Each one adds a state (Inspect, Heal, Radio) and something that has to reach the others.
struct Features {
    bool noise = false;
    bool lowHealth = false;
    bool radio = false;
    bool operator==(const Features&) const = default;
};

inline bool stateExists(State s, const Features& f) {
    switch (s) {
        case State::Inspect: return f.noise;
        case State::Heal: return f.lowHealth;
        case State::Radio: return f.radio;
        default: return true;
    }
}

inline int stateCount(const Features& f) {
    int n = 0;
    for (State s : kAllStates) n += stateExists(s, f) ? 1 : 0;
    return n;
}

// ---- the guard's body and senses -----------------------------------------------------------------------------------

struct Body {
    double x = kWaypoints[0];
    double hp = kMaxHp;
    std::size_t waypoint = 1;  // the next waypoint to walk to
    bool hasLead = false;      // knows where the player was last seen
    double leadX = 0.0;
    bool hasNoise = false;
    double noiseX = 0.0;
    bool radioRinging = false;
    double facing = 1.0;
    int strikes = 0;
    double flash = 0.0;        // seconds left of the strike flash, only for drawing
};

struct Percept {
    bool sees = false;
    bool inRange = false;
    bool heardNoise = false;
    bool hasLead = false;
    bool lowHp = false;
    bool healed = false;
    bool ringing = false;
};

inline Percept perceive(const Body& b, double playerX) {
    const double d = std::fabs(playerX - b.x);
    return Percept{d <= kSight, d <= kStrikeRange, b.hasNoise, b.hasLead, b.hp < kLowHp, b.hp >= kMaxHp, b.radioRinging};
}

/// Updates what the guard knows. Whoever sees the player remembers where, and a noise that has a visible cause is no mystery.
inline void sense(Body& b, double playerX) {
    if (std::fabs(playerX - b.x) <= kSight) {
        b.hasLead = true;
        b.leadX = playerX;
        b.hasNoise = false;
    }
}

inline void makeNoise(Body& b, double x) {
    if (std::fabs(x - b.x) <= kHearing) {
        b.hasNoise = true;
        b.noiseX = x;
    }
}

/// Moves towards `target` and stops `stop` px short of it. Returns true once it is there.
inline bool walkTo(Body& b, double target, double speed, double stop, double dt) {
    const double dx = target - b.x;
    const double dist = std::fabs(dx);
    if (dist <= stop) return true;
    b.facing = dx > 0.0 ? 1.0 : -1.0;
    b.x += b.facing * std::min(speed * dt, dist - stop);
    return std::fabs(target - b.x) <= stop + 1e-9;
}

inline bool healStep(Body& b, double dt) {
    b.hp = std::min(kMaxHp, b.hp + kHealRate * dt);
    return b.hp >= kMaxHp;
}

inline void strikeDone(Body& b) {
    ++b.strikes;
    b.flash = 0.25;
}

inline void patrolStep(Body& b, double dt, bool& arrived) {
    arrived = walkTo(b, kWaypoints[b.waypoint], kWalk, kArrive, dt);
    if (arrived) b.waypoint = (b.waypoint + 1) % kWaypoints.size();
}

// ---- behavior tree --------------------------------------------------------------------------------------------------

enum class Status { Success, Failure, Running };

struct Ctx {
    Body& body;
    double playerX;
    double dt;
    unsigned long tickId;
};

class Node {
public:
    std::string label;
    State branch = State::Patrol;  // for the children of the root: the state this branch stands for
    Status last = Status::Failure;
    unsigned long lastTick = 0;    // tickId of the last tick, 0 = never ticked
    bool running = false;
    std::vector<std::unique_ptr<Node>> kids;

    explicit Node(std::string l) : label(std::move(l)) {}
    virtual ~Node() = default;

    void add(std::unique_ptr<Node> kid) { kids.push_back(std::move(kid)); }

    Status tick(Ctx& c) {
        if (!running) reset();
        last = update(c);
        lastTick = c.tickId;
        running = last == Status::Running;
        return last;
    }

    /// Cuts a running node off. Its children and progress are cleaned up, a finished node is left alone.
    void abort() {
        if (!running) return;
        for (auto& kid : kids) kid->abort();
        aborted();
        running = false;
        reset();
    }

protected:
    virtual Status update(Ctx& c) = 0;
    virtual void reset() {}
    virtual void aborted() {}
};

/// A condition or an action. `fn` gets a timer that starts at 0 whenever the leaf starts over.
class Leaf : public Node {
public:
    using Fn = std::function<Status(Ctx&, double&)>;
    Leaf(std::string l, Fn fn, std::function<void()> onAbort = {}) : Node(std::move(l)), fn_(std::move(fn)), onAbort_(std::move(onAbort)) {}

protected:
    Status update(Ctx& c) override { return fn_(c, t_); }
    void reset() override { t_ = 0.0; }
    void aborted() override {
        if (onAbort_) onAbort_();
    }

private:
    Fn fn_;
    std::function<void()> onAbort_;
    double t_ = 0.0;
};

/// Runs the children in order, stops at the first failure. A running child is resumed directly next tick, the
/// children before it are not asked again (this is how the Behavior Tree Starter Kit does it).
class Sequence : public Node {
public:
    explicit Sequence(std::string l) : Node(std::move(l)) {}

protected:
    Status update(Ctx& c) override {
        while (current_ < kids.size()) {
            const Status s = kids[current_]->tick(c);
            if (s != Status::Success) return s;
            ++current_;
        }
        return Status::Success;
    }
    void reset() override { current_ = 0; }

private:
    std::size_t current_ = 0;
};

/// Tries the children in order until one does not fail. Passive: a running child stays in charge until it is done.
/// Active: every tick starts from the top again, and a higher child that now takes over aborts the running one.
class Selector : public Node {
public:
    Selector(std::string l, bool active) : Node(std::move(l)), active_(active) {}

    bool active() const { return active_; }
    std::size_t currentIndex() const { return current_; }
    const Node* currentChild() const { return current_ < kids.size() ? kids[current_].get() : nullptr; }

protected:
    Status update(Ctx& c) override {
        const bool wasRunning = running;
        const std::size_t prev = current_;
        std::size_t i = (active_ || !wasRunning) ? 0 : current_;
        for (; i < kids.size(); ++i) {
            const Status s = kids[i]->tick(c);
            if (s == Status::Failure) continue;
            if (active_ && wasRunning && i != prev) kids[prev]->abort();
            current_ = i;
            return s;
        }
        return Status::Failure;
    }
    void reset() override { current_ = 0; }

private:
    bool active_;
    std::size_t current_ = 0;
};

namespace detail {

inline void append(std::vector<std::unique_ptr<Node>>&) {}

template <class... Rest>
void append(std::vector<std::unique_ptr<Node>>& v, std::unique_ptr<Node> first, Rest... rest) {
    v.push_back(std::move(first));
    append(v, std::move(rest)...);
}

inline std::unique_ptr<Node> cond(std::string label, std::function<bool(const Ctx&)> test) {
    return std::make_unique<Leaf>(std::move(label), [test = std::move(test)](Ctx& c, double&) { return test(c) ? Status::Success : Status::Failure; });
}

inline std::unique_ptr<Node> action(std::string label, Leaf::Fn fn) { return std::make_unique<Leaf>(std::move(label), std::move(fn)); }

template <class... Kids>
std::unique_ptr<Node> sequence(std::string label, Kids... kids) {
    auto seq = std::make_unique<Sequence>(std::move(label));
    append(seq->kids, std::move(kids)...);
    return seq;
}

/// A timed action: runs for `seconds`, then calls `done` once and succeeds.
inline std::unique_ptr<Node> timed(std::string label, double seconds, std::function<void(Body&)> done) {
    return action(std::move(label), [seconds, done = std::move(done)](Ctx& c, double& t) {
        t += c.dt;
        if (t < seconds) return Status::Running;
        done(c.body);
        return Status::Success;
    });
}

}  // namespace detail

/// The guard's tree. Priority from left to right: radio, heal, attack, chase, noise, search, patrol.
inline std::unique_ptr<Node> buildTree(const Features& f, bool active) {
    using namespace detail;
    auto root = std::make_unique<Selector>(active ? "ACTIVE" : "PASSIVE", active);
    auto put = [&root](std::unique_ptr<Node> branch, State s) {
        branch->branch = s;
        root->add(std::move(branch));
    };
    if (f.radio) {
        put(sequence("Radio", cond("ringing?", [](const Ctx& c) { return c.body.radioRinging; }),
                     timed("talk", kRadioTime, [](Body& b) { b.radioRinging = false; })),
            State::Radio);
    }
    if (f.lowHealth) {
        put(sequence("Heal", cond("hp low?", [](const Ctx& c) { return perceive(c.body, c.playerX).lowHp; }),
                     action("medkit", [](Ctx& c, double&) { return walkTo(c.body, kMedkitX, kRun, kArrive, c.dt) ? Status::Success : Status::Running; }),
                     action("heal", [](Ctx& c, double&) { return healStep(c.body, c.dt) ? Status::Success : Status::Running; })),
            State::Heal);
    }
    put(sequence("Attack", cond("close?", [](const Ctx& c) { return perceive(c.body, c.playerX).inRange; }),
                 timed("strike", kStrikeTime, [](Body& b) { strikeDone(b); })),
        State::Attack);
    put(sequence("Chase", cond("sees?", [](const Ctx& c) { return perceive(c.body, c.playerX).sees; }),
                 action("chase", [](Ctx& c, double&) { return walkTo(c.body, c.body.leadX, kRun, kChaseStop, c.dt) ? Status::Success : Status::Running; })),
        State::Chase);
    if (f.noise) {
        put(sequence("Noise", cond("noise?", [](const Ctx& c) { return c.body.hasNoise; }),
                     action("go to", [](Ctx& c, double&) { return walkTo(c.body, c.body.noiseX, kWalk, kArrive, c.dt) ? Status::Success : Status::Running; }),
                     timed("listen", kListenTime, [](Body& b) { b.hasNoise = false; })),
            State::Inspect);
    }
    put(sequence("Search", cond("lastpos?", [](const Ctx& c) { return c.body.hasLead; }),
                 action("go to", [](Ctx& c, double&) { return walkTo(c.body, c.body.leadX, kWalk, kArrive, c.dt) ? Status::Success : Status::Running; }),
                 timed("look", kLookTime, [](Body& b) { b.hasLead = false; })),
        State::Search);
    put(action("patrol",
               [](Ctx& c, double&) {
                   bool arrived = false;
                   patrolStep(c.body, c.dt, arrived);
                   return arrived ? Status::Success : Status::Running;
               }),
        State::Patrol);
    return root;
}

inline int countNodes(const Node& n) {
    int total = 1;
    for (const auto& kid : n.kids) total += countNodes(*kid);
    return total;
}

class BtBrain {
public:
    Body body;
    State state = State::Patrol;
    unsigned long tickId = 0;
    std::unique_ptr<Node> root;

    BtBrain() { configure(Features{}, true); }

    /// Builds the tree anew. The body stays, the progress of running nodes does not.
    void configure(const Features& f, bool active) { root = buildTree(f, active); }

    void tick(double playerX, double dt) {
        sense(body, playerX);
        Ctx c{body, playerX, dt, ++tickId};
        root->tick(c);
        if (const Node* current = static_cast<Selector&>(*root).currentChild()) state = current->branch;
        body.flash = std::max(0.0, body.flash - dt);
    }
};

// ---- finite state machine ------------------------------------------------------------------------------------------

/// The condition of an edge.
enum class When { Sees, LostSight, InRange, OutOfRange, LeadGone, HasNoise, NoiseDoneLead, NoiseDoneNoLead, LowHp, Healed, Ringing, RadioDone };

inline const char* whenLabel(When w) {
    switch (w) {
        case When::Sees: return "sees player";
        case When::LostSight: return "lost sight";
        case When::InRange: return "in range";
        case When::OutOfRange: return "out of range";
        case When::LeadGone: return "nothing found";
        case When::HasNoise: return "heard noise";
        case When::NoiseDoneLead:
        case When::NoiseDoneNoLead: return "noise checked";
        case When::LowHp: return "hp low";
        case When::Healed: return "healed";
        case When::Ringing: return "radio rings";
        case When::RadioDone: return "call over";
    }
    return "?";
}

/// Which requirement an edge belongs to (the demo colors them).
enum class Group { Base, Noise, Heal, Radio };

struct Edge {
    State from;
    State to;
    When when;
    Group group;
    bool viaHistory = false;  // goes back to the state the interrupt came from, `to` is then only the fallback
};

inline bool holds(When w, const Percept& p, double stateTime) {
    switch (w) {
        case When::Sees: return p.sees;
        case When::LostSight: return !p.sees;
        case When::InRange: return p.inRange;
        case When::OutOfRange: return !p.inRange;
        case When::LeadGone: return !p.hasLead;
        case When::HasNoise: return p.heardNoise;
        case When::NoiseDoneLead: return !p.heardNoise && p.hasLead;
        case When::NoiseDoneNoLead: return !p.heardNoise && !p.hasLead;
        case When::LowHp: return p.lowHp;
        case When::Healed: return p.healed;
        case When::Ringing: return p.ringing;
        case When::RadioDone: return stateTime >= kRadioTime;
    }
    return false;
}

/// All edges, in priority order: for one state the first edge that holds fires. A requirement that has to work from
/// anywhere (radio, heal) brings one edge per existing state, plus the way back.
inline std::vector<Edge> buildEdges(const Features& f, bool history) {
    std::vector<Edge> e;
    if (f.radio) {
        for (State s : kAllStates) {
            if (s != State::Radio && stateExists(s, f)) e.push_back({s, State::Radio, When::Ringing, Group::Radio});
        }
    }
    if (f.lowHealth) {
        for (State s : kAllStates) {
            if (s != State::Heal && s != State::Radio && stateExists(s, f)) e.push_back({s, State::Heal, When::LowHp, Group::Heal});
        }
    }
    e.push_back({State::Patrol, State::Chase, When::Sees, Group::Base});
    e.push_back({State::Chase, State::Attack, When::InRange, Group::Base});
    e.push_back({State::Chase, State::Search, When::LostSight, Group::Base});
    e.push_back({State::Attack, State::Chase, When::OutOfRange, Group::Base});
    e.push_back({State::Search, State::Chase, When::Sees, Group::Base});
    e.push_back({State::Search, State::Patrol, When::LeadGone, Group::Base});
    if (f.noise) {
        e.push_back({State::Patrol, State::Inspect, When::HasNoise, Group::Noise});
        e.push_back({State::Search, State::Inspect, When::HasNoise, Group::Noise});
        e.push_back({State::Inspect, State::Chase, When::Sees, Group::Noise});
        e.push_back({State::Inspect, State::Search, When::NoiseDoneLead, Group::Noise});
        e.push_back({State::Inspect, State::Patrol, When::NoiseDoneNoLead, Group::Noise});
    }
    if (f.lowHealth) e.push_back({State::Heal, State::Patrol, When::Healed, Group::Heal});
    if (f.radio) e.push_back({State::Radio, State::Patrol, When::RadioDone, Group::Radio, history});
    return e;
}

class FsmBrain {
public:
    Body body;
    State state = State::Patrol;
    State remembered = State::Patrol;  // the history marker: where the last radio call came from
    std::vector<Edge> edges = buildEdges(Features{}, false);
    double time = 0.0;                 // in the current state
    bool arrived = false;
    int fired = -1;                    // index of the edge taken in the last tick, -1 = none

    /// Rebuilds the edge table. The body stays, and so does the state if it still exists.
    void configure(const Features& f, bool history) {
        features_ = f;
        edges = buildEdges(f, history);
        if (!stateExists(state, f)) restart(State::Patrol);
        if (!stateExists(remembered, f)) remembered = State::Patrol;
        fired = -1;
    }

    void tick(double playerX, double dt) {
        sense(body, playerX);
        const Percept p = perceive(body, playerX);
        fired = -1;
        for (std::size_t i = 0; i < edges.size(); ++i) {
            const Edge& e = edges[i];
            if (e.from != state || !holds(e.when, p, time)) continue;
            if (e.to == State::Radio) remembered = e.from;
            if (state == State::Radio) body.radioRinging = false;
            const State target = e.viaHistory ? remembered : e.to;
            restart(stateExists(target, features_) ? target : State::Patrol);
            fired = static_cast<int>(i);
            break;
        }
        run(dt);
        body.flash = std::max(0.0, body.flash - dt);
    }

private:
    Features features_;

    void restart(State s) {
        state = s;
        time = 0.0;
        arrived = false;
    }

    void run(double dt) {
        switch (state) {
            case State::Patrol: patrolStep(body, dt, arrived); break;
            case State::Chase: walkTo(body, body.leadX, kRun, kChaseStop, dt); break;
            case State::Search:
                if (!arrived) arrived = walkTo(body, body.leadX, kWalk, kArrive, dt);
                if (arrived) {
                    time += dt;
                    if (time >= kLookTime) body.hasLead = false;
                }
                break;
            case State::Attack:
                time += dt;
                if (time >= kStrikeTime) {
                    time = 0.0;
                    strikeDone(body);
                }
                break;
            case State::Inspect:
                if (!arrived) arrived = walkTo(body, body.noiseX, kWalk, kArrive, dt);
                if (arrived) {
                    time += dt;
                    if (time >= kListenTime) body.hasNoise = false;
                }
                break;
            case State::Heal:
                if (!arrived) arrived = walkTo(body, kMedkitX, kRun, kArrive, dt);
                if (arrived) healStep(body, dt);
                break;
            case State::Radio: time += dt; break;
        }
    }
};

// ---- what a change costs -------------------------------------------------------------------------------------------

struct Cost {
    int fsmEdges = 0;         // edges more (less when negative)
    int fsmStatesEdited = 0;  // existing states whose list of exits changed
    int btNodes = 0;
    int btPlaces = 0;         // places in the tree that change: one insert into the root's list
};

inline Cost changeCost(const Features& before, const Features& after) {
    Cost c;
    const auto eb = buildEdges(before, false);
    const auto ea = buildEdges(after, false);
    c.fsmEdges = static_cast<int>(ea.size()) - static_cast<int>(eb.size());
    for (State s : kAllStates) {
        if (!stateExists(s, before) || !stateExists(s, after)) continue;
        const auto exits = [s](const std::vector<Edge>& edges) {
            return std::count_if(edges.begin(), edges.end(), [s](const Edge& e) { return e.from == s; });
        };
        if (exits(eb) != exits(ea)) ++c.fsmStatesEdited;
    }
    c.btNodes = countNodes(*buildTree(after, true)) - countNodes(*buildTree(before, true));
    c.btPlaces = before == after ? 0 : 1;
    return c;
}

// ---- drawing helper ------------------------------------------------------------------------------------------------

struct Pt {
    double x;
    double y;
};

/// Where a ray leaves a box (center, half sizes hw and hh) in direction `dir`. The ray starts at the center plus
/// `offset`, which must lie inside the box (a line drawn next to the middle line). A zero direction gives the start.
inline Pt boxEdgePoint(Pt center, Pt dir, double hw, double hh, Pt offset = {0.0, 0.0}) {
    const Pt origin{center.x + offset.x, center.y + offset.y};
    const double ax = std::fabs(dir.x);
    const double ay = std::fabs(dir.y);
    if (ax < 1e-12 && ay < 1e-12) return origin;
    const double tx = ax < 1e-12 ? 1e300 : (dir.x > 0.0 ? hw - offset.x : hw + offset.x) / ax;
    const double ty = ay < 1e-12 ? 1e300 : (dir.y > 0.0 ? hh - offset.y : hh + offset.y) / ay;
    const double t = std::min(tx, ty);
    return {origin.x + dir.x * t, origin.y + dir.y * t};
}

}  // namespace gbt
