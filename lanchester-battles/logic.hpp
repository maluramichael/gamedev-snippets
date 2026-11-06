// Lanchester battles: the math and the agent simulation, without SDL, so test.cpp can run it natively.
//
// Two armies A and B. alpha is how fast one unit of A kills units of B (kills per second while it can fire),
// beta the same for B. Lanchester's two laws differ in how many units can fight at the same moment:
//
//   Square law (everybody can hit everybody):  dA/dt = -beta*B,  dB/dt = -alpha*A,  alpha*A^2 - beta*B^2 is constant.
//   Linear law (one-on-one duels):             dA/dt = -beta*m,  dB/dt = -alpha*m,  alpha*A   - beta*B   is constant,
//                                              m = min(A, B) is the number of duels running at once.
//
// The constant tells who wins and with how much: the survivor keeps sqrt(k/alpha) (square) or k/alpha (linear).
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace lb {

enum class Law { Square, Linear };
enum class Winner { A, B, Draw };

struct Outcome {
    Winner winner = Winner::Draw;
    double left = 0.0;  // size of the winning army when the last enemy falls
};

/// A fights B. `a2` is a reserve of A that joins only after the first group of A is dead ("divide and lose").
struct Setup {
    double a1 = 0.0;
    double a2 = 0.0;
    double b = 0.0;
    double alpha = 1.0;
    double beta = 1.0;
};

struct Sample {
    double t = 0.0;
    double a = 0.0;  // all of A that is alive, reserve included
    double b = 0.0;
};

/// Result of one engagement of `a` against `b`, straight from the conserved quantity.
inline Outcome fight(Law law, double a, double b, double alpha, double beta) {
    a = std::max(a, 0.0);
    b = std::max(b, 0.0);
    alpha = std::max(alpha, 0.0);
    beta = std::max(beta, 0.0);
    if (a == 0.0 && b == 0.0) return {Winner::Draw, 0.0};
    if (b == 0.0) return {Winner::A, a};
    if (a == 0.0) return {Winner::B, b};
    if (alpha == 0.0 && beta == 0.0) return {Winner::Draw, 0.0};  // nobody can hurt anybody
    const double n = law == Law::Square ? 2.0 : 1.0;
    const double strengthA = alpha * std::pow(a, n);
    const double strengthB = beta * std::pow(b, n);
    const double k = strengthA - strengthB;
    if (std::fabs(k) <= 1e-12 * (strengthA + strengthB)) return {Winner::Draw, 0.0};
    if (k > 0.0) return {Winner::A, std::pow(k / alpha, 1.0 / n)};
    return {Winner::B, std::pow(-k / beta, 1.0 / n)};
}

/// Same with a reserve: the second group of A only starts once the first one is gone.
inline Outcome predict(Law law, const Setup& s) {
    const double reserve = std::max(s.a2, 0.0);
    const Outcome first = fight(law, s.a1, s.b, s.alpha, s.beta);
    if (first.winner == Winner::A) return {Winner::A, first.left + reserve};
    if (first.winner == Winner::Draw) return reserve > 0.0 ? Outcome{Winner::A, reserve} : first;
    if (reserve <= 0.0) return first;
    return fight(law, reserve, first.left, s.alpha, s.beta);
}

namespace detail {

struct Pair {
    double a;
    double b;
};

inline Pair slope(Law law, double a, double b, double alpha, double beta) {
    a = std::max(a, 0.0);
    b = std::max(b, 0.0);
    if (law == Law::Square) return {-beta * b, -alpha * a};
    const double m = std::min(a, b);
    return {-beta * m, -alpha * m};
}

}  // namespace detail

/// The differential equations integrated with RK4, sampled every `dt` seconds. Ends at `tMax`, or when a side drops to
/// half a unit (the last sample is the interpolated moment it does, that side at exactly 0.5; a reserve of A moves up
/// instead of ending the curve).
inline std::vector<Sample> curve(Law law, const Setup& s, double dt = 0.05, double tMax = 300.0) {
    constexpr double kDead = 0.5;
    double a = std::max(s.a1, 0.0);
    double reserve = std::max(s.a2, 0.0);
    double b = std::max(s.b, 0.0);
    double t = 0.0;
    std::vector<Sample> out;
    out.push_back({t, a + reserve, b});
    if ((s.alpha <= 0.0 && s.beta <= 0.0) || dt <= 0.0) return out;
    while (t < tMax) {
        if (a <= kDead && reserve > kDead) {  // the first group is dead, the reserve takes the field
            a = reserve;
            reserve = 0.0;
        }
        if (a <= kDead || b <= kDead) break;
        const detail::Pair k1 = detail::slope(law, a, b, s.alpha, s.beta);
        const detail::Pair k2 = detail::slope(law, a + 0.5 * dt * k1.a, b + 0.5 * dt * k1.b, s.alpha, s.beta);
        const detail::Pair k3 = detail::slope(law, a + 0.5 * dt * k2.a, b + 0.5 * dt * k2.b, s.alpha, s.beta);
        const detail::Pair k4 = detail::slope(law, a + dt * k3.a, b + dt * k3.b, s.alpha, s.beta);
        const double na = a + dt / 6.0 * (k1.a + 2.0 * k2.a + 2.0 * k3.a + k4.a);
        const double nb = b + dt / 6.0 * (k1.b + 2.0 * k2.b + 2.0 * k3.b + k4.b);
        // Whoever falls to kDead first cuts the step short. a > kDead here, so the divisions are safe.
        const double fa = na < kDead ? (a - kDead) / (a - na) : 1.0;
        const double fb = nb < kDead ? (b - kDead) / (b - nb) : 1.0;
        const double f = std::min(fa, fb);
        t += f * dt;
        a = fa == f && fa < 1.0 ? kDead : a + f * (na - a);
        b = fb == f && fb < 1.0 ? kDead : b + f * (nb - b);
        out.push_back({t, a + reserve, b});
    }
    return out;
}

// ---------------------------------------------------------------------------------------------------------------
// Agent simulation: the same two armies as dots on a field. Nothing in here knows the laws; it only has weapons,
// range, targets and movement. Comparing its result with fight() is the whole point of the demo.
// ---------------------------------------------------------------------------------------------------------------

constexpr float kFieldW = 560.0F;
constexpr float kFieldH = 300.0F;
constexpr float kRangeFar = 5000.0F;  // everybody reaches everybody
constexpr float kRangeShort = 70.0F;

// Ranged units: one shot every 1.6 s, 8 percent of the shots kill. So one unit kills 0.05 enemies per second.
constexpr double kRangedInterval = 1.6;
constexpr double kRangedHit = 0.08;
// Melee units strike faster, otherwise a duel battle takes minutes: 0.8 s and 20 percent, 0.25 kills per second.
constexpr double kDuelInterval = 0.8;
constexpr double kDuelHit = 0.2;
constexpr double kFlight = 0.30;  // a shot needs this long to arrive, and it lands even if the shooter died meanwhile
constexpr float kSpeed = 55.0F;   // px per second
constexpr float kContact = 13.0F;  // melee reach
constexpr float kQueue = 26.0F;    // waiting units stop this far from the nearest enemy
constexpr float kSeparation = 9.0F;
constexpr double kMaxTime = 150.0;
constexpr float kSpacing = 15.0F;
constexpr int kMaxRows = 14;
constexpr float kFrontA = 190.0F;
constexpr float kFrontB = 370.0F;

enum class Aim {
    Claim,    // the enemy with the fewest shots already on the way, nearest first
    Nearest,  // nearest enemy, however many others picked it as well
    Random,   // any enemy in range
};

struct Params {
    Law law = Law::Square;  // Square = ranged weapons, Linear = melee duels
    Aim aim = Aim::Claim;
    float range = kRangeFar;
    int a1 = 100;
    int a2 = 0;        // reserve of A, waits behind until the first group is dead
    int b = 70;
    double power = 1.0;  // one unit of B is this many times faster than one unit of A
    std::uint64_t seed = 1;
};

inline double alphaOf(Law law) { return law == Law::Square ? kRangedHit / kRangedInterval : kDuelHit / kDuelInterval; }

/// The numbers the laws need for these parameters.
inline Setup setupOf(const Params& p) {
    const double alpha = alphaOf(p.law);
    return {static_cast<double>(p.a1), static_cast<double>(p.a2), static_cast<double>(p.b), alpha, alpha * p.power};
}

struct Vec2 {
    float x = 0.0F;
    float y = 0.0F;
};

/// Small deterministic generator (SplitMix64), so a seed gives the same battle on every machine.
class Rng {
public:
    explicit Rng(std::uint64_t seed) : state_(seed) {}
    std::uint64_t next() {
        state_ += 0x9E3779B97F4A7C15ULL;
        std::uint64_t z = state_;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
        return z ^ (z >> 31);
    }
    /// In [0, 1).
    double uniform() { return static_cast<double>(next() >> 11) * (1.0 / 9007199254740992.0); }

private:
    std::uint64_t state_;
};

/// Positions of `count` units in columns, front column first. `dir` is -1 for an army that stands left of `frontX`
/// and extends to the left, +1 for the right side.
inline std::vector<Vec2> block(int count, float frontX, int dir) {
    std::vector<Vec2> out;
    if (count <= 0) return out;
    const int cols = (count + kMaxRows - 1) / kMaxRows;
    const int perCol = (count + cols - 1) / cols;
    int placed = 0;
    for (int c = 0; c < cols && placed < count; ++c) {
        const int inCol = std::min(perCol, count - placed);
        for (int r = 0; r < inCol; ++r) {
            const float y = kFieldH / 2.0F + (static_cast<float>(r) - static_cast<float>(inCol - 1) / 2.0F) * kSpacing;
            out.push_back({frontX + static_cast<float>(dir * c) * kSpacing, y});
        }
        placed += inCol;
    }
    return out;
}

/// Number of columns `block(count, ...)` uses.
inline int columns(int count) { return count <= 0 ? 0 : (count + kMaxRows - 1) / kMaxRows; }

struct Unit {
    Vec2 pos;
    int side = 0;  // 0 = A, 1 = B
    int group = 0;  // 1 = A's reserve
    bool alive = true;
    bool active = true;
    double cooldown = 0.0;
    int partner = -1;  // duel opponent
    bool contact = false;  // standing at the duel partner
};

struct Shot {
    int from = 0;
    int to = 0;
    Vec2 origin;
    double left = 0.0;  // seconds until it arrives
    bool hit = false;
};

class Battle {
public:
    explicit Battle(const Params& p) : p_(p), rng_(p.seed * 7919ULL + 17ULL) {
        const auto frontsA = block(p_.a1, kFrontA, -1);
        const int reserveGap = columns(p_.a1) + 1;
        const auto reserve = block(p_.a2, kFrontA - static_cast<float>(reserveGap) * kSpacing, -1);
        const auto frontsB = block(p_.b, kFrontB, +1);
        for (const Vec2& v : frontsA) add(v, 0, 0, true);
        for (const Vec2& v : reserve) add(v, 0, 1, false);
        for (const Vec2& v : frontsB) add(v, 1, 0, true);
        claims_.assign(units_.size(), 0);
        history_.push_back({0.0, static_cast<double>(alive_[0]), static_cast<double>(alive_[1])});
    }

    const Params& params() const { return p_; }
    double time() const { return time_; }
    const std::vector<Unit>& units() const { return units_; }
    const std::vector<Shot>& shots() const { return shots_; }
    /// Alive units over time (A with its reserve), one sample every 0.1 s.
    const std::vector<Sample>& history() const { return history_; }
    int alive(int side) const { return alive_[static_cast<std::size_t>(side)]; }
    bool reserveReleased() const { return released_; }
    bool timedOut() const { return time_ >= kMaxTime; }
    bool finished() const { return timedOut() || ((alive_[0] == 0 || alive_[1] == 0) && shots_.empty()); }

    Outcome outcome() const {
        if (alive_[0] > 0 && alive_[1] == 0) return {Winner::A, static_cast<double>(alive_[0])};
        if (alive_[1] > 0 && alive_[0] == 0) return {Winner::B, static_cast<double>(alive_[1])};
        return {};
    }

    /// Share of the time that units of `side` had an enemy to hit (in range, or standing at a partner).
    double engaged(int side) const {
        const auto s = static_cast<std::size_t>(side);
        return unitTime_[s] > 0.0 ? engagedTime_[s] / unitTime_[s] : 1.0;
    }
    /// Share of the fired shots that arrived at a target that was already dead.
    double wasted() const { return fired_ > 0 ? static_cast<double>(atDead_) / static_cast<double>(fired_) : 0.0; }
    int shotsFired() const { return fired_; }

    void step(double dt) {
        if (dt <= 0.0 || finished()) return;
        time_ += dt;
        if (!released_ && p_.a2 > 0 && activeAlive(0) == 0) release();
        const std::size_t n = units_.size();
        for (std::size_t k = 0; k < n; ++k) {
            const std::size_t i = (k + offset_) % n;  // rotating start so nobody is always first to shoot
            Unit& u = units_[i];
            if (!u.alive || !u.active) continue;
            const bool busy = p_.law == Law::Square ? stepRanged(i, dt) : stepDuel(i, dt);
            const auto s = static_cast<std::size_t>(u.side);
            unitTime_[s] += dt;
            if (busy) engagedTime_[s] += dt;
        }
        offset_ = (offset_ + 7) % std::max<std::size_t>(n, 1);
        resolveShots(dt);
        while (time_ + 1e-9 >= nextSample_) {
            history_.push_back({nextSample_, static_cast<double>(alive_[0]), static_cast<double>(alive_[1])});
            nextSample_ += 0.1;
        }
    }

private:
    void add(Vec2 pos, int side, int group, bool active) {
        Unit u;
        u.pos = pos;
        u.side = side;
        u.group = group;
        u.active = active;
        u.cooldown = rng_.uniform() * intervalOf(side);
        units_.push_back(u);
        ++alive_[static_cast<std::size_t>(side)];
    }

    double intervalOf(int side) const {
        const double base = p_.law == Law::Square ? kRangedInterval : kDuelInterval;
        return side == 1 ? base / std::max(p_.power, 1e-6) : base;
    }
    double hitOf() const { return p_.law == Law::Square ? kRangedHit : kDuelHit; }

    int activeAlive(int side) const {
        int n = 0;
        for (const Unit& u : units_) n += (u.side == side && u.alive && u.active) ? 1 : 0;
        return n;
    }

    void release() {
        for (Unit& u : units_) {
            if (u.group == 1) u.active = true;
        }
        released_ = true;
    }

    void kill(int i) {
        Unit& u = units_[static_cast<std::size_t>(i)];
        if (!u.alive) return;
        u.alive = false;
        --alive_[static_cast<std::size_t>(u.side)];
        if (u.partner >= 0) {
            units_[static_cast<std::size_t>(u.partner)].partner = -1;
            u.partner = -1;
        }
    }

    static float dist2(Vec2 a, Vec2 b) {
        const float dx = a.x - b.x;
        const float dy = a.y - b.y;
        return dx * dx + dy * dy;
    }

    /// Walk up to `maxStep` toward `to`, pushed apart from friends that are too close. The push closes 30 times the
    /// overlap per second (capped at all of it), like the walking it scales with `dt`, so the step length hardly matters.
    void move(std::size_t i, Vec2 to, float maxStep, double dt) {
        Unit& u = units_[i];
        const float dx = to.x - u.pos.x;
        const float dy = to.y - u.pos.y;
        const float d = std::sqrt(dx * dx + dy * dy);
        if (d < 1e-4F || maxStep <= 0.0F) return;
        const float step = std::min(maxStep, d);
        float px = u.pos.x + dx / d * step;
        float py = u.pos.y + dy / d * step;
        const auto relax = static_cast<float>(std::min(1.0, 30.0 * dt));
        for (std::size_t j = 0; j < units_.size(); ++j) {
            const Unit& o = units_[j];
            if (j == i || !o.alive || !o.active || o.side != u.side) continue;
            const float ox = px - o.pos.x;
            const float oy = py - o.pos.y;
            const float od2 = ox * ox + oy * oy;
            if (od2 < kSeparation * kSeparation && od2 > 1e-6F) {
                const float od = std::sqrt(od2);
                const float push = (kSeparation - od) * relax;
                px += ox / od * push;
                py += oy / od * push;
            }
        }
        u.pos.x = std::clamp(px, 4.0F, kFieldW - 4.0F);
        u.pos.y = std::clamp(py, 4.0F, kFieldH - 4.0F);
    }

    /// Ranged weapons. True while the unit has somebody in range.
    bool stepRanged(std::size_t i, double dt) {
        Unit& u = units_[i];
        int nearest = -1;
        float best = 1e30F;
        for (std::size_t j = 0; j < units_.size(); ++j) {
            const Unit& e = units_[j];
            if (e.side == u.side || !e.alive || !e.active) continue;
            const float d2 = dist2(u.pos, e.pos);
            if (d2 < best) {
                best = d2;
                nearest = static_cast<int>(j);
            }
        }
        if (nearest < 0) return false;
        const float dist = std::sqrt(best);
        if (dist > p_.range) {
            move(i, units_[static_cast<std::size_t>(nearest)].pos, std::min(kSpeed * static_cast<float>(dt), dist - p_.range * 0.97F), dt);
            return false;
        }
        u.cooldown -= dt;
        if (u.cooldown <= 0.0) {
            const int target = pickTarget(i, nearest);
            if (target >= 0) {
                fire(i, target);
                u.cooldown += intervalOf(u.side);
            } else {
                u.cooldown = 0.0;  // nothing in range right now
            }
        }
        return true;
    }

    int pickTarget(std::size_t i, int nearest) {
        const Unit& u = units_[i];
        if (p_.aim == Aim::Nearest) return nearest;
        int pick = -1;
        float best = 1e30F;
        int seen = 0;
        const float r2 = p_.range * p_.range;
        for (std::size_t j = 0; j < units_.size(); ++j) {
            const Unit& e = units_[j];
            if (e.side == u.side || !e.alive || !e.active) continue;
            const float d2 = dist2(u.pos, e.pos);
            if (d2 > r2) continue;
            if (p_.aim == Aim::Claim) {
                const bool fewer = pick < 0 || claims_[j] < claims_[static_cast<std::size_t>(pick)];
                const bool tie = pick >= 0 && claims_[j] == claims_[static_cast<std::size_t>(pick)] && d2 < best;
                if (fewer || tie) {
                    best = d2;
                    pick = static_cast<int>(j);
                }
            } else {
                ++seen;  // reservoir sampling: each candidate ends up picked with probability 1/seen
                if (rng_.uniform() * seen < 1.0) pick = static_cast<int>(j);
            }
        }
        return pick;
    }

    void fire(std::size_t from, int to) {
        shots_.push_back({static_cast<int>(from), to, units_[from].pos, kFlight, rng_.uniform() < hitOf()});
        ++claims_[static_cast<std::size_t>(to)];
        ++fired_;
    }

    void resolveShots(double dt) {
        std::size_t keep = 0;
        for (std::size_t k = 0; k < shots_.size(); ++k) {
            Shot s = shots_[k];
            s.left -= dt;
            if (s.left > 0.0) {
                shots_[keep++] = s;
                continue;
            }
            --claims_[static_cast<std::size_t>(s.to)];
            if (!units_[static_cast<std::size_t>(s.to)].alive) {
                ++atDead_;
            } else if (s.hit) {
                kill(s.to);
            }
        }
        shots_.resize(keep);
    }

    /// Melee: pair up one on one, walk to the partner, trade blows. True while standing at the partner.
    bool stepDuel(std::size_t i, double dt) {
        Unit& u = units_[i];
        if (u.partner < 0) {
            u.contact = false;
            int free = -1;
            float freeD = 1e30F;
            int any = -1;
            float anyD = 1e30F;
            for (std::size_t j = 0; j < units_.size(); ++j) {
                const Unit& e = units_[j];
                if (e.side == u.side || !e.alive || !e.active) continue;
                const float d2 = dist2(u.pos, e.pos);
                if (d2 < anyD) {
                    anyD = d2;
                    any = static_cast<int>(j);
                }
                if (e.partner < 0 && d2 < freeD) {
                    freeD = d2;
                    free = static_cast<int>(j);
                }
            }
            if (free >= 0) {
                u.partner = free;
                units_[static_cast<std::size_t>(free)].partner = static_cast<int>(i);
            } else {
                if (any >= 0) {
                    const float d = std::sqrt(anyD);
                    move(i, units_[static_cast<std::size_t>(any)].pos, std::min(kSpeed * static_cast<float>(dt), d - kQueue), dt);
                }
                return false;
            }
        }
        const int partner = u.partner;
        const Unit& p = units_[static_cast<std::size_t>(partner)];
        const float d = std::sqrt(dist2(u.pos, p.pos));
        if (d > kContact) {
            // Walk until clearly inside the reach, so rounding cannot leave a pair standing 13.0000001 px apart.
            move(i, p.pos, std::min(kSpeed * static_cast<float>(dt), d - kContact * 0.9F), dt);
            u.contact = false;
            return false;
        }
        if (!u.contact) {
            // A fresh duel starts with a random swing phase for both. Without this the unit that walks up to a
            // waiting partner always strikes first and the reserve side would win more duels than the law says.
            u.contact = true;
            u.cooldown = rng_.uniform() * intervalOf(u.side);
        }
        u.cooldown -= dt;
        if (u.cooldown <= 0.0) {
            u.cooldown += intervalOf(u.side);
            if (rng_.uniform() < hitOf()) kill(partner);
        }
        return true;
    }

    Params p_;
    Rng rng_;
    std::vector<Unit> units_;
    std::vector<Shot> shots_;
    std::vector<int> claims_;  // shots in the air per target
    std::vector<Sample> history_;
    std::array<int, 2> alive_{0, 0};
    std::array<double, 2> unitTime_{0.0, 0.0};
    std::array<double, 2> engagedTime_{0.0, 0.0};
    double time_ = 0.0;
    double nextSample_ = 0.1;
    std::size_t offset_ = 0;
    bool released_ = false;
    int fired_ = 0;
    int atDead_ = 0;
};

}  // namespace lb
