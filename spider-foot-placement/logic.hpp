// Spider foot placement: the logic of the demo, no SDL, so test.cpp can check it.
//
// The body is driven (it follows a goal), the feet are not. Every foot stands still in the world until the point it
// should stand on has moved too far away, then it swings to that point. "The point it should stand on" is the leg's
// rest point on the body plus the body's velocity times a short look-ahead, so the foot lands a bit in front of where
// the body is heading. "Too far" is the distance between that target and the foot, compared with a threshold.
//
// The gait is only a rule about who may start a swing: alternating tripod (two groups of three legs, one group in the
// air at a time), crawl (one leg at a time) or free (everyone who wants to). Nothing else differs.
//
// A planted foot never moves by itself. The body is the one that gives way: if a move would stretch a planted leg beyond
// 97 percent of d1 + d2, the move is cut short (brakeFactor). The legs are drawn with a two-segment IK solved by the
// law of cosines, the knee bent outward.
//
// Body-local axes: x forward, y to the right. Coordinates are screen coordinates, y points down.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace spider {

struct Vec2 {
    double x, y;
};

inline Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
inline Vec2 operator*(Vec2 a, double s) { return {a.x * s, a.y * s}; }
inline double dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
inline double cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }
inline double length(Vec2 a) { return std::sqrt(dot(a, a)); }

constexpr double kPi = 3.14159265358979323846;
constexpr int kLegs = 6;
constexpr double kMovingSpeed = 28.0;  // px/s: below this the body counts as standing for the statistics
constexpr double kAutoPeriod = 12.0;   // seconds for one round of the demo path
constexpr Vec2 kPathCenter{320.0, 240.0};
constexpr double kPathHalfWidth = 200.0;
constexpr double kPathHalfHeight = 80.0;

enum class Gait { Tripod, Crawl, Free };
enum class Phase { Planted, Swinging };

struct Params {
    double d1 = 54.0;               // hip to knee
    double d2 = 70.0;               // knee to foot
    double stepThreshold = 50.0;    // a step is wanted when the foot is further than this from its target
    double settleThreshold = 5.0;   // the same while the body (almost) stands: feet shuffle to their rest points
    double settleSpeed = 11.0;      // px/s below which the settle threshold applies
    double lookahead = 0.2;         // seconds of velocity added to the rest point
    double maxAhead = 25.0;         // ... but never more than this many px
    double swingTime = 0.12;        // seconds one swing takes
    double liftHeight = 14.0;       // px of height at the top of a swing (drawn as size and shadow)
    double reachLimit = 0.97;       // a planted leg may stretch to this share of d1 + d2
    double maxSpeed = 195.0;        // px/s
    double accel = 980.0;           // px/s^2
    double arrive = 3.0;            // 1/s: speed = distance * arrive near the goal
    double stopRadius = 6.0;        // px around the goal where the body stops
    double turnRate = 6.0;          // rad/s
    double turnMinSpeed = 14.0;     // px/s below which the body keeps its heading
};

inline double maxReach(const Params& p) { return p.d1 + p.d2; }

struct LegDef {
    Vec2 hip;   // body-local
    Vec2 rest;  // body-local: where the foot likes to stand
    int group;  // tripod group 0 or 1
    double side;  // -1 left, +1 right (outward direction in body-local y)
};

// 0 FL, 1 FR, 2 ML, 3 MR, 4 RL, 5 RR. Group 0 = {FL, MR, RL}, group 1 = {FR, ML, RR}: on each side the groups alternate.
inline constexpr std::array<LegDef, kLegs> kLegDefs{{{{20.0, -12.0}, {56.0, -82.0}, 0, -1.0},
                                                      {{20.0, 12.0}, {56.0, 82.0}, 1, 1.0},
                                                      {{4.0, -16.0}, {4.0, -96.0}, 1, -1.0},
                                                      {{4.0, 16.0}, {4.0, 96.0}, 0, 1.0},
                                                      {{-20.0, -16.0}, {-60.0, -86.0}, 0, -1.0},
                                                      {{-20.0, 16.0}, {-60.0, 86.0}, 1, 1.0}}};
inline const std::array<LegDef, kLegs>& legDefs() { return kLegDefs; }

struct Leg {
    Vec2 foot;
    Phase phase;
    double t;     // swing progress 0..1
    Vec2 start;   // where the swing started
};

struct Walker {
    Vec2 pos;
    double heading;  // radians, 0 = facing +x
    Vec2 vel;
    std::array<Leg, kLegs> legs;
    Gait gait;
    bool lookahead;
    Params p;
};

// ---- geometry ---------------------------------------------------------------------------------------------------

inline Vec2 rotate(Vec2 v, double angle) {
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    return {v.x * c - v.y * s, v.x * s + v.y * c};
}

inline Vec2 toWorld(Vec2 pos, double heading, Vec2 local) { return pos + rotate(local, heading); }

inline Vec2 hipWorld(const Walker& w, int i) { return toWorld(w.pos, w.heading, legDefs()[static_cast<std::size_t>(i)].hip); }
inline Vec2 restWorld(const Walker& w, int i) { return toWorld(w.pos, w.heading, legDefs()[static_cast<std::size_t>(i)].rest); }

/// Wraps an angle difference into (-pi, pi], so the body always turns the short way.
inline double angleDiff(double from, double to) {
    double d = std::fmod(to - from, 2.0 * kPi);
    if (d > kPi) d -= 2.0 * kPi;
    if (d <= -kPi) d += 2.0 * kPi;
    return d;
}

inline Vec2 clampLength(Vec2 v, double maxLen) {
    const double len = length(v);
    return len > maxLen && len > 0.0 ? v * (maxLen / len) : v;
}

/// The foot pulled back onto the circle of radius maxDist around the hip, if it is outside.
inline Vec2 clampToReach(Vec2 hip, Vec2 foot, double maxDist) {
    const double d = length(foot - hip);
    if (d <= maxDist || d < 1e-12) return foot;
    return hip + (foot - hip) * (maxDist / d);
}

// ---- two-segment IK (the law of cosines, see the two-bone-ik demo) ---------------------------------------------

struct LegPose {
    Vec2 knee;
    Vec2 end;
};

/// Hip to knee is d1, knee to foot d2. Of the two knee positions the one that lies further along `outward` is taken.
/// Out of range the leg is stretched (too far) or folded (too close) towards the foot instead of returning NaN.
inline LegPose solveLeg(Vec2 hip, Vec2 foot, double d1, double d2, Vec2 outward) {
    const double h = length(foot - hip);
    const Vec2 dir = h < 1e-12 ? Vec2{1.0, 0.0} : (foot - hip) * (1.0 / h);
    const double eps = 1e-9 * (d1 + d2);
    if (h > d1 + d2 + eps) {
        const Vec2 knee = hip + dir * d1;
        return {knee, knee + dir * d2};
    }
    if (h < std::fabs(d1 - d2) - eps || h < 1e-12) {
        const double side = d1 >= d2 ? 1.0 : -1.0;
        const Vec2 knee = hip + dir * (side * d1);
        return {knee, knee - dir * (side * d2)};
    }
    const double cosA = std::clamp((d1 * d1 + h * h - d2 * d2) / (2.0 * d1 * h), -1.0, 1.0);
    const double sinA = std::sqrt(1.0 - cosA * cosA);
    const Vec2 base = hip + dir * (d1 * cosA);
    const Vec2 perp{-dir.y, dir.x};
    const Vec2 a = base + perp * (d1 * sinA);
    const Vec2 b = base - perp * (d1 * sinA);
    return {dot(a - hip, outward) >= dot(b - hip, outward) ? a : b, foot};
}

// ---- support polygon --------------------------------------------------------------------------------------------

/// Convex hull (monotone chain), counter-clockwise in math axes, without collinear points. Fewer than three
/// distinct non-collinear points give a degenerate hull of fewer than three vertices.
inline std::vector<Vec2> convexHull(std::vector<Vec2> pts) {
    std::sort(pts.begin(), pts.end(), [](Vec2 a, Vec2 b) { return a.x < b.x || (a.x == b.x && a.y < b.y); });
    pts.erase(std::unique(pts.begin(), pts.end(), [](Vec2 a, Vec2 b) { return a.x == b.x && a.y == b.y; }), pts.end());
    if (pts.size() < 3) return pts;
    std::vector<Vec2> hull(2 * pts.size());
    std::size_t k = 0;
    for (std::size_t i = 0; i < pts.size(); ++i) {
        while (k >= 2 && cross(hull[k - 1] - hull[k - 2], pts[i] - hull[k - 2]) <= 0.0) --k;
        hull[k++] = pts[i];
    }
    const std::size_t lower = k + 1;
    for (std::size_t i = pts.size() - 1; i-- > 0;) {
        while (k >= lower && cross(hull[k - 1] - hull[k - 2], pts[i] - hull[k - 2]) <= 0.0) --k;
        hull[k++] = pts[i];
    }
    hull.resize(k - 1);
    return hull;
}

/// Inside or on the border of a convex hull from convexHull(). A hull without area contains nothing.
inline bool insideHull(const std::vector<Vec2>& hull, Vec2 p) {
    if (hull.size() < 3) return false;
    for (std::size_t i = 0; i < hull.size(); ++i) {
        const Vec2 a = hull[i];
        const Vec2 b = hull[(i + 1) % hull.size()];
        if (cross(b - a, p - a) < -1e-9) return false;
    }
    return true;
}

inline int plantedCount(const Walker& w) {
    int n = 0;
    for (const Leg& l : w.legs) n += l.phase == Phase::Planted ? 1 : 0;
    return n;
}

inline std::vector<Vec2> plantedFeet(const Walker& w) {
    std::vector<Vec2> feet;
    for (const Leg& l : w.legs) {
        if (l.phase == Phase::Planted) feet.push_back(l.foot);
    }
    return feet;
}

/// Static stability: the center of the body is inside the polygon spanned by the planted feet.
inline bool supported(const Walker& w) { return insideHull(convexHull(plantedFeet(w)), w.pos); }

// ---- the step rule ----------------------------------------------------------------------------------------------

inline Walker makeWalker(Vec2 pos, double heading, Gait gait, bool lookahead) {
    Walker w{pos, heading, {0.0, 0.0}, {}, gait, lookahead, Params{}};
    for (int i = 0; i < kLegs; ++i) {
        const Vec2 rest = restWorld(w, i);
        w.legs[static_cast<std::size_t>(i)] = Leg{rest, Phase::Planted, 0.0, rest};
    }
    return w;
}

/// Where leg i wants to stand: its rest point plus the body velocity times the look-ahead, at most maxAhead long.
inline Vec2 stepTarget(const Walker& w, int i) {
    const Vec2 rest = restWorld(w, i);
    if (!w.lookahead) return rest;
    return rest + clampLength(w.vel * w.p.lookahead, w.p.maxAhead);
}

/// Strictly greater: a foot exactly on the threshold stays.
inline bool needsStep(Vec2 foot, Vec2 target, double threshold) { return length(foot - target) > threshold; }

inline double activeThreshold(const Walker& w) { return length(w.vel) < w.p.settleSpeed ? w.p.settleThreshold : w.p.stepThreshold; }

/// Which planted legs start a swing now. This is the whole gait.
///   Tripod: when a leg of a group wants a step and no leg of the other group is in the air, all planted legs of the
///           group go together (if both groups want one, the one further behind goes first).
///   Crawl:  when nobody is in the air, the leg that is furthest behind goes.
///   Free:   every planted leg that wants a step goes.
inline std::array<bool, kLegs> chooseSteps(const Walker& w) {
    std::array<bool, kLegs> go{};
    const double threshold = activeThreshold(w);
    std::array<double, kLegs> behind{};  // distance foot -> target for planted legs, 0 otherwise
    std::array<bool, kLegs> wants{};
    int swinging = 0;
    std::array<bool, 2> airborne{false, false};
    std::array<double, 2> groupWant{0.0, 0.0};
    for (int i = 0; i < kLegs; ++i) {
        const auto idx = static_cast<std::size_t>(i);
        const int g = legDefs()[idx].group;
        if (w.legs[idx].phase == Phase::Swinging) {
            ++swinging;
            airborne[static_cast<std::size_t>(g)] = true;
            continue;
        }
        const double d = length(w.legs[idx].foot - stepTarget(w, i));
        behind[idx] = d;
        wants[idx] = needsStep(w.legs[idx].foot, stepTarget(w, i), threshold);
        if (wants[idx]) groupWant[static_cast<std::size_t>(g)] = std::max(groupWant[static_cast<std::size_t>(g)], d);
    }

    switch (w.gait) {
        case Gait::Free:
            for (std::size_t i = 0; i < kLegs; ++i) go[i] = wants[i];
            break;
        case Gait::Crawl: {
            if (swinging > 0) break;
            int best = -1;
            for (int i = 0; i < kLegs; ++i) {
                const auto idx = static_cast<std::size_t>(i);
                if (wants[idx] && (best < 0 || behind[idx] > behind[static_cast<std::size_t>(best)])) best = i;
            }
            if (best >= 0) go[static_cast<std::size_t>(best)] = true;
            break;
        }
        case Gait::Tripod: {
            const bool can0 = groupWant[0] > 0.0 && !airborne[1];
            const bool can1 = groupWant[1] > 0.0 && !airborne[0];
            int group = -1;
            if (can0 && can1) {
                group = groupWant[1] > groupWant[0] ? 1 : 0;
            } else if (can0) {
                group = 0;
            } else if (can1) {
                group = 1;
            }
            if (group >= 0) {
                for (int i = 0; i < kLegs; ++i) {
                    const auto idx = static_cast<std::size_t>(i);
                    if (legDefs()[idx].group == group && w.legs[idx].phase == Phase::Planted) go[idx] = true;
                }
            }
            break;
        }
    }
    return go;
}

// ---- moving the body --------------------------------------------------------------------------------------------

/// The largest share f in [0, 1] of a body move (step, heading change) that keeps every planted leg within the reach
/// limit. A leg that is beyond the limit already only blocks moves that stretch it further.
inline double brakeFactor(const Walker& w, Vec2 step, double headingStep) {
    const double limit = w.p.reachLimit * maxReach(w.p);
    const auto ok = [&](double f) {
        for (int i = 0; i < kLegs; ++i) {
            const auto idx = static_cast<std::size_t>(i);
            if (w.legs[idx].phase != Phase::Planted) continue;
            const Vec2 foot = w.legs[idx].foot;
            const double bound = std::max(limit, length(hipWorld(w, i) - foot));
            const Vec2 hip = toWorld(w.pos + step * f, w.heading + headingStep * f, legDefs()[idx].hip);
            if (length(hip - foot) > bound + 1e-9) return false;
        }
        return true;
    };
    if (ok(1.0)) return 1.0;
    double lo = 0.0;
    double hi = 1.0;
    for (int k = 0; k < 24; ++k) {
        const double mid = 0.5 * (lo + hi);
        (ok(mid) ? lo : hi) = mid;
    }
    return lo;
}

inline double smoothstep(double t) {
    t = std::clamp(t, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

/// Height of the foot above the ground: 0 while planted, a half sine over the swing.
inline double legLift(const Walker& w, int i) {
    const Leg& l = w.legs[static_cast<std::size_t>(i)];
    return l.phase == Phase::Swinging ? std::sin(kPi * std::clamp(l.t, 0.0, 1.0)) * w.p.liftHeight : 0.0;
}

inline Vec2 approach(Vec2 v, Vec2 wanted, double maxDelta) {
    const Vec2 d = wanted - v;
    const double len = length(d);
    return len <= maxDelta || len == 0.0 ? wanted : v + d * (maxDelta / len);
}

/// One fixed step: the body moves towards the goal (cut short by the reach limit), swinging feet advance and land,
/// then new swings start. Returns how many legs started a swing.
inline int update(Walker& w, Vec2 goal, double dt) {
    const Params& p = w.p;
    const Vec2 toGoal = goal - w.pos;
    const double dist = length(toGoal);
    Vec2 wanted{0.0, 0.0};
    if (dist > p.stopRadius) wanted = toGoal * (std::min(p.maxSpeed, dist * p.arrive) / dist);
    w.vel = approach(w.vel, wanted, p.accel * dt);

    const Vec2 step = w.vel * dt;
    double headingStep = 0.0;
    if (length(w.vel) > p.turnMinSpeed) {
        const double lim = p.turnRate * dt;
        headingStep = std::clamp(angleDiff(w.heading, std::atan2(w.vel.y, w.vel.x)), -lim, lim);
    }
    const double f = brakeFactor(w, step, headingStep);
    w.pos = w.pos + step * f;
    w.heading = std::remainder(w.heading + headingStep * f, 2.0 * kPi);
    w.vel = w.vel * f;

    for (int i = 0; i < kLegs; ++i) {
        Leg& leg = w.legs[static_cast<std::size_t>(i)];
        if (leg.phase != Phase::Swinging) continue;
        leg.t += dt / p.swingTime;
        const Vec2 target = stepTarget(w, i);
        if (leg.t >= 1.0 - 1e-12) {
            leg.foot = target;
            leg.phase = Phase::Planted;
            leg.t = 0.0;
        } else {
            leg.foot = leg.start + (target - leg.start) * smoothstep(leg.t);
        }
    }

    int started = 0;
    const std::array<bool, kLegs> go = chooseSteps(w);
    for (int i = 0; i < kLegs; ++i) {
        const auto idx = static_cast<std::size_t>(i);
        if (!go[idx]) continue;
        w.legs[idx].phase = Phase::Swinging;
        w.legs[idx].t = 0.0;
        w.legs[idx].start = w.legs[idx].foot;
        ++started;
    }

    // Safety net: a foot is never further from its hip than the leg is long. The brake makes this a no-op for planted feet.
    for (int i = 0; i < kLegs; ++i) {
        Leg& leg = w.legs[static_cast<std::size_t>(i)];
        leg.foot = clampToReach(hipWorld(w, i), leg.foot, maxReach(p));
    }
    return started;
}

// ---- statistics -------------------------------------------------------------------------------------------------

/// Counted only while the body moves. Reset (Stats{}) whenever the gait or the look-ahead changes.
struct Stats {
    double moveTime = 0.0;
    double stanceTime = 0.0;       // summed over all legs
    double unsupportedTime = 0.0;  // center outside the polygon of the planted feet (or fewer than three feet)
    int steps = 0;                 // swings started, summed over all legs
    int minPlanted = kLegs;
};

inline void record(Stats& s, const Walker& w, int started, double dt) {
    if (length(w.vel) <= kMovingSpeed) return;
    s.moveTime += dt;
    const int planted = plantedCount(w);
    s.stanceTime += planted * dt;
    s.steps += started;
    s.minPlanted = std::min(s.minPlanted, planted);
    if (!supported(w)) s.unsupportedTime += dt;
}

/// Share of the time an average foot is on the ground. Above 0.5 is what biomechanics calls walking.
inline double dutyFactor(const Stats& s) { return s.moveTime > 0.0 ? s.stanceTime / (kLegs * s.moveTime) : 0.0; }

// ---- demo path --------------------------------------------------------------------------------------------------

/// Keeps the phase in [0, kAutoPeriod): a time that only grows would lose its fractions after hours.
inline double advancePhase(double phase, double dt) {
    const double p = std::fmod(phase + dt, kAutoPeriod);
    return p < 0.0 ? p + kAutoPeriod : p;
}

/// A figure eight: x swings once per period, y twice.
inline Vec2 autoGoal(double phase) {
    const double a = 2.0 * kPi * phase / kAutoPeriod;
    return {kPathCenter.x + kPathHalfWidth * std::sin(a), kPathCenter.y + kPathHalfHeight * std::sin(2.0 * a)};
}

/// The direction of travel on the path at a phase (the derivative of autoGoal), for a start that already faces forward.
inline double autoHeading(double phase) {
    const double a = 2.0 * kPi * phase / kAutoPeriod;
    return std::atan2(2.0 * kPathHalfHeight * std::cos(2.0 * a), kPathHalfWidth * std::cos(a));
}

}  // namespace spider
