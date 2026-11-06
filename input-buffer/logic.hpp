// Coyote time and input buffer, without SDL.
//
// Everything runs in fixed ticks of 1/120 s, so a window of "100 ms" is exactly 12 ticks and the result does not
// depend on the frame rate (main.cpp feeds the ticks from an accumulator). One tick is:
//
//   1. the world says whether the player is grounded,
//   2. the Controller decides whether a jump fires (coyote time and input buffer live here),
//   3. the world moves.
//
// y points down, y = 0 is the top line of the stones. Nothing here is drawn; main.cpp only reads the state.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>

namespace ib {

constexpr int kTickRate = 120;
constexpr double kDt = 1.0 / kTickRate;
constexpr int kNever = 1000;  // "a long time ago" for the timers

constexpr int ticksFromMs(int ms) { return (ms * kTickRate + 500) / 1000; }
constexpr int msFromTicks(int ticks) { return (ticks * 1000 + kTickRate / 2) / kTickRate; }

// ---------------------------------------------------------------------------------------------------------------
// The two windows
// ---------------------------------------------------------------------------------------------------------------

struct Params {
    int coyote = 12;      // ticks after the last grounded tick in which a jump still works
    int buffer = 12;      // ticks a press waits for the player to be able to jump
    bool consume = true;  // reset the coyote timer when a jump fires (false is the double jump bug)
};

enum class Jump { None, Ground, Coyote, Air };

struct Outcome {
    Jump jump = Jump::None;
    int pressAge = 0;       // ticks the fired press had waited
    bool lost = false;      // a pending press ran out of buffer
    bool replaced = false;  // a new press overwrote a pending one
    bool canJump = false;   // grounded or inside the coyote window (before this tick's jump used it up)
    int sinceGround = 0;    // ticks since the last grounded tick
    int jumpsBefore = 0;    // jumps since the last grounded tick, before this tick
};

class Controller {
public:
    /// One tick. `grounded` is the state at the start of the tick, `pressed` is a jump press that happened in it.
    Outcome step(bool grounded, bool pressed, const Params& p) {
        Outcome o;
        if (grounded) {
            sinceGround_ = 0;
            jumps_ = 0;
        } else {
            sinceGround_ = std::min(sinceGround_ + 1, kNever);
        }
        if (pressed) {
            o.replaced = pending_;
            pending_ = true;
            pressAge_ = 0;
        } else if (pending_) {
            ++pressAge_;
        }
        o.canJump = sinceGround_ <= p.coyote;
        o.sinceGround = sinceGround_;
        o.jumpsBefore = jumps_;
        if (pending_) {
            if (o.canJump) {
                o.jump = grounded ? Jump::Ground : (jumps_ == 0 ? Jump::Coyote : Jump::Air);
                o.pressAge = pressAge_;
                pending_ = false;
                ++jumps_;
                if (p.consume) sinceGround_ = kNever;
            } else if (pressAge_ >= p.buffer) {
                o.lost = true;
                pending_ = false;
            }
        }
        return o;
    }

    void reset() { *this = Controller{}; }

    bool pending() const { return pending_; }
    int pressAge() const { return pressAge_; }
    int sinceGround() const { return sinceGround_; }
    int jumps() const { return jumps_; }

private:
    int sinceGround_ = kNever;
    int pressAge_ = 0;
    int jumps_ = 0;
    bool pending_ = false;
};

// ---------------------------------------------------------------------------------------------------------------
// The level and the player
// ---------------------------------------------------------------------------------------------------------------

constexpr double kRunSpeed = 200.0;  // px/s, the player always runs to the right
constexpr double kGravity = 1800.0;
constexpr double kJumpSpeed = 560.0;
constexpr double kPitDepth = 90.0;  // falling this far below the stones respawns
constexpr double kSpawnX = 21.0;    // offset of the respawn point in its period

// One period: a long stone, a gap, a short stone, a gap. A jump covers about 123 px, so the short stone can only
// be reached by jumping at the long stone's edge and the next long stone only by jumping right after landing.
constexpr double kPeriod = 386.0;
constexpr double kLongLen = 150.0;
constexpr double kShortStart = 230.0;
constexpr double kShortLen = 80.0;

enum class Ground { None, Long, Short };

inline double wrapPeriod(double x) {
    const double r = std::fmod(x, kPeriod);
    return r < 0.0 ? r + kPeriod : r;
}

inline Ground groundAt(double x) {
    const double r = wrapPeriod(x);
    if (r < kLongLen) return Ground::Long;
    if (r >= kShortStart && r < kShortStart + kShortLen) return Ground::Short;
    return Ground::None;
}

inline long cycleOf(double x) { return static_cast<long>(std::floor(x / kPeriod)); }

/// Ticks until a body at (y, vy) crosses y = 0 downwards, counting the integration steps of World::step. -1 if it
/// does not within `maxTicks`.
inline int ticksToLanding(double y, double vy, int maxTicks = 240) {
    for (int n = 1; n <= maxTicks; ++n) {
        const double before = y;
        vy += kGravity * kDt;
        y += vy * kDt;
        if (vy > 0.0 && before <= 0.0 && y >= 0.0) return n;
    }
    return -1;
}

struct World {
    double x = kSpawnX;
    double y = 0.0;
    double vy = 0.0;
    bool onGround = true;
    int falls = 0;

    void respawn() {
        x = static_cast<double>(cycleOf(x)) * kPeriod + kSpawnX;
        y = 0.0;
        vy = 0.0;
        onGround = true;
    }

    /// One tick of movement. Returns true if the player fell into a pit and was put back on the long stone.
    bool step(bool jump) {
        if (jump) {
            vy = -kJumpSpeed;
            onGround = false;
        }
        x += kRunSpeed * kDt;
        if (onGround) {
            if (groundAt(x) == Ground::None) onGround = false;  // walked off, falling starts next tick
            return false;
        }
        const double before = y;
        vy += kGravity * kDt;
        y += vy * kDt;
        if (vy > 0.0 && before <= 0.0 && y >= 0.0 && groundAt(x) != Ground::None) {
            y = 0.0;
            vy = 0.0;
            onGround = true;
            return false;
        }
        if (y > kPitDepth) {
            ++falls;
            respawn();
            return true;
        }
        return false;
    }
};

// ---------------------------------------------------------------------------------------------------------------
// A sloppy tester: always a little too early or too late
// ---------------------------------------------------------------------------------------------------------------

/// Presses jump too late at the ledge of the long stone (so it needs coyote time) and too early before landing on the
/// short stone (so it needs the buffer). The offsets cycle through fixed lists in ticks, so a run is reproducible.
/// Every few ledges it also taps a second time, nervously, which is what exposes the double jump bug.
class Bot {
public:
    static constexpr int kCount = 6;
    static constexpr std::array<int, kCount> kLate{2, 5, 8, 4, 16, 10};   // ticks after the ledge
    static constexpr std::array<int, kCount> kEarly{4, 11, 7, 19, 3, 9};  // ticks before landing
    static constexpr int kTapIndex = 3;                                    // this ledge press is followed by a tap
    static constexpr int kTapDelay = 5;

    /// Called once per tick before the controller. Returns whether the bot presses jump in this tick.
    bool decide(const World& w, const Controller& c) {
        bool press = false;
        if (tapIn_ > 0 && --tapIn_ == 0) press = true;
        if (w.onGround) {
            ledgeDone_ = false;
            flightDone_ = false;
            return press;
        }
        if (c.jumps() == 0) {
            if (!ledgeDone_ && c.sinceGround() + 1 >= kLate[static_cast<std::size_t>(ledgePresses_ % kCount)]) {
                ledgeDone_ = true;
                if (ledgePresses_ % kCount == kTapIndex) tapIn_ = kTapDelay;
                ++ledgePresses_;
                press = true;
            }
        } else if (!flightDone_) {
            const int n = ticksToLanding(w.y, w.vy);
            const bool shortStone = n > 0 && groundAt(w.x + kRunSpeed * kDt * n) == Ground::Short;
            if (shortStone && n <= kEarly[static_cast<std::size_t>(flightPresses_ % kCount)]) {
                flightDone_ = true;
                ++flightPresses_;
                press = true;
            }
        }
        return press;
    }

    /// After a respawn: forget the current attempt, keep the position in the offset lists.
    void onRespawn() {
        ledgeDone_ = false;
        flightDone_ = false;
        tapIn_ = 0;
    }

private:
    int ledgePresses_ = 0;
    int flightPresses_ = 0;
    int tapIn_ = 0;
    bool ledgeDone_ = false;
    bool flightDone_ = false;
};

// ---------------------------------------------------------------------------------------------------------------
// What the timeline shows
// ---------------------------------------------------------------------------------------------------------------

struct TickRec {
    long tick = -1;
    bool grounded = false;
    bool open = false;  // grounded or inside the coyote window
    Jump jump = Jump::None;
};

struct PressRec {
    enum class State { Pending, Fired, Lost };
    long tick = 0;
    State state = State::Pending;
    long endTick = 0;
    Jump kind = Jump::None;  // for fired presses
    int age = 0;             // for fired presses: ticks it waited
    bool groundedAtPress = false;
    int sinceGround = 0;
    int jumpsBefore = 0;
    long nextGround = -1;  // first grounded tick after a lost press, -2 if the player died before landing
    bool counted = false;  // already counted in Stats::lost
};

enum class Lost { Early, Late, Extra };

struct Miss {
    Lost kind = Lost::Extra;
    int ticks = 0;
};

constexpr int kEarlyReach = 40;  // a lost press that is followed by a landing within this many ticks was too early
constexpr int kLateReach = 60;

/// Why a lost press was lost. Early: the landing came after the buffer ran out. Late: it was pressed after the
/// ground was gone and the coyote window had closed. Extra: a tap in the air that had nothing to wait for.
inline Miss classifyMiss(const PressRec& r) {
    if (r.nextGround >= 0 && r.nextGround - r.tick <= kEarlyReach) return {Lost::Early, static_cast<int>(r.nextGround - r.tick)};
    if (r.jumpsBefore == 0 && !r.groundedAtPress && r.sinceGround <= kLateReach) return {Lost::Late, r.sinceGround};
    return {Lost::Extra, 0};
}

struct Stats {
    int jumps = 0;
    int saved = 0;     // jumps that only worked because of coyote time or the buffer
    int lost = 0;      // inputs that were too early or too late (taps in the air do not count)
    int falls = 0;
    int airJumps = 0;  // jumps in mid-air, only possible with the bug
};

class History {
public:
    static constexpr int kRing = 512;
    static constexpr long kKeep = 480;

    void clear() { *this = History{}; }

    void record(long tick, bool grounded, bool pressed, const Outcome& o) {
        ring_[static_cast<std::size_t>(tick % kRing)] = TickRec{tick, grounded, o.canJump, o.jump};
        if (grounded) {
            for (PressRec& r : presses_) {
                if (r.state == PressRec::State::Lost && r.nextGround == -1) {
                    r.nextGround = tick;
                    countIfMiss(r);
                }
            }
        }
        if (o.replaced) resolvePending(tick, PressRec::State::Lost, Jump::None, 0);
        if (pressed) {
            PressRec r;
            r.tick = tick;
            r.groundedAtPress = grounded;
            r.sinceGround = o.sinceGround;
            r.jumpsBefore = o.jumpsBefore;
            presses_.push_back(r);
        }
        if (o.jump != Jump::None) {
            resolvePending(tick, PressRec::State::Fired, o.jump, o.pressAge);
            ++stats_.jumps;
            if (o.jump == Jump::Coyote || o.pressAge > 0) ++stats_.saved;
            if (o.jump == Jump::Air) ++stats_.airJumps;
        } else if (o.lost) {
            resolvePending(tick, PressRec::State::Lost, Jump::None, 0);
        }
        while (!presses_.empty() && presses_.front().tick < tick - kKeep) presses_.pop_front();
    }

    /// The player fell: a press that was still waiting is void, and no landing follows the lost ones.
    void onDeath(long tick) {
        ++stats_.falls;
        falls_.push_back(tick);
        while (!falls_.empty() && falls_.front() < tick - kKeep) falls_.pop_front();
        if (!presses_.empty() && presses_.back().state == PressRec::State::Pending) presses_.pop_back();
        for (PressRec& r : presses_) {
            if (r.nextGround == -1) r.nextGround = -2;
        }
    }

    const TickRec* at(long tick) const {
        if (tick < 0) return nullptr;
        const TickRec& t = ring_[static_cast<std::size_t>(tick % kRing)];
        return t.tick == tick ? &t : nullptr;
    }

    const std::deque<PressRec>& presses() const { return presses_; }
    const std::deque<long>& falls() const { return falls_; }
    const Stats& stats() const { return stats_; }

    /// The newest press that has an outcome.
    const PressRec* lastResolved() const {
        for (auto it = presses_.rbegin(); it != presses_.rend(); ++it) {
            if (it->state != PressRec::State::Pending) return &*it;
        }
        return nullptr;
    }

private:
    std::array<TickRec, kRing> ring_{};
    std::deque<PressRec> presses_;
    std::deque<long> falls_;  // ticks in which the player fell into a pit
    Stats stats_;

    void resolvePending(long tick, PressRec::State state, Jump kind, int age) {
        if (presses_.empty() || presses_.back().state != PressRec::State::Pending) return;
        PressRec& r = presses_.back();
        r.state = state;
        r.endTick = tick;
        r.kind = kind;
        r.age = age;
        if (state == PressRec::State::Lost) countIfMiss(r);
    }

    void countIfMiss(PressRec& r) {
        if (r.counted) return;
        const Lost kind = classifyMiss(r).kind;
        if (kind == Lost::Early || kind == Lost::Late) {
            r.counted = true;
            ++stats_.lost;
        }
    }
};

// ---------------------------------------------------------------------------------------------------------------
// Everything together
// ---------------------------------------------------------------------------------------------------------------

struct Sim {
    World world;
    Controller ctrl;
    Bot bot;
    History hist;
    Params params;
    bool botOn = true;
    long tick = 0;

    /// One fixed tick. `humanPress` is a press by the player; the bot adds its own when it is on. Returns true if
    /// the player fell into a pit in this tick.
    bool tickOnce(bool humanPress) {
        const bool botPress = botOn && bot.decide(world, ctrl);
        const bool grounded = world.onGround;
        const Outcome o = ctrl.step(grounded, humanPress || botPress, params);
        hist.record(tick, grounded, humanPress || botPress, o);
        const bool died = world.step(o.jump != Jump::None);
        if (died) {
            ctrl.reset();
            bot.onRespawn();
            hist.onDeath(tick);
        }
        ++tick;
        return died;
    }

    /// Back to the start; windows and the bot/you switch stay as they are.
    void restart() {
        world = World{};
        ctrl.reset();
        bot = Bot{};
        hist.clear();
        tick = 0;
    }
};

}  // namespace ib
