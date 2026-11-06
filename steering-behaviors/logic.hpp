// Steering behaviors: the logic of the demo, no SDL, so test.cpp can check it.
//
// After Craig Reynolds, "Steering Behaviors For Autonomous Characters" (GDC 1999), written down again in my own words.
//
// Every behavior answers one question: which velocity do I want right now (the "desired velocity")? The steering is
// the same for all of them:
//
//     steering = desired velocity - current velocity
//
// That is the velocity that is missing. A real vehicle cannot add it all at once, so the change per step is capped:
// the force limit. The cap is a change of velocity PER SECOND (maxForce), so one step of dt seconds may add at most
// maxForce * dt. Leave the dt out and the agent is tuned for exactly one frame rate: at twice the steps per second it
// accelerates and moves twice as fast. `scaleByDt = false` keeps that mistake around on purpose, tuned for 60 steps
// per second, so the demo can show it next to the right version.
//
// Position and velocity are in pixels and pixels per second, y grows downwards like on screen.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace sb {

constexpr double kPi = 3.14159265358979323846;
/// The step rate the "per frame" version is tuned for.
constexpr double kReferenceHz = 60.0;

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};

inline Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
inline Vec2 operator*(Vec2 a, double s) { return {a.x * s, a.y * s}; }
inline double length(Vec2 v) { return std::sqrt(v.x * v.x + v.y * v.y); }
inline double distance(Vec2 a, Vec2 b) { return length(a - b); }

/// Unit vector, or zero for a zero vector (no division by zero, no NaN).
inline Vec2 normalize(Vec2 v) {
    const double len = length(v);
    return len > 1e-9 ? v * (1.0 / len) : Vec2{0.0, 0.0};
}

/// Same direction, length at most `max`.
inline Vec2 truncate(Vec2 v, double max) {
    if (max <= 0.0) return {0.0, 0.0};
    const double len = length(v);
    return len > max ? v * (max / len) : v;
}

struct PathParams {
    double radius = 26.0;     // half width of the corridor around the path
    double lookahead = 0.45;  // seconds: the check is made for where the agent will be, not where it is
    double lead = 45.0;       // how far ahead on the path the agent aims once it left the corridor
};

struct Params {
    double maxSpeed = 200.0;      // px/s
    double maxForce = 200.0;      // px/s per second: the most velocity the steering may add in a second
    double arriveRadius = 130.0;  // arrive starts to slow down this far from the target
    double panicRadius = 220.0;   // flee only reacts inside this distance
    double wanderDistance = 90.0; // wander: the circle sits this far ahead of the agent
    double wanderRadius = 45.0;   // and has this radius
    PathParams path;
    // The field. Agents that leave it on one side come back on the other.
    double left = 0.0;
    double top = 34.0;
    double right = 960.0;
    double bottom = 430.0;
};

struct Agent {
    Vec2 pos;
    Vec2 vel;
    Vec2 heading{1.0, 0.0};  // direction of the last non-zero velocity
    double time = 0.0;       // seconds this agent has been simulated
};

enum class Behavior { Seek, Flee, Arrive, Wander, Path };

using Path = std::vector<Vec2>;
inline const Path kNoPath{};

// --- the desired velocities ----------------------------------------------------------------------------------------

/// Straight at the target at full speed. Never slows down, so it cannot stop on the target.
inline Vec2 seekDesired(Vec2 pos, Vec2 target, double maxSpeed) { return normalize(target - pos) * maxSpeed; }

/// Away from the target at full speed, but only inside the panic radius. Outside it the wish is zero and the agent
/// brakes. On top of the target there is no "away", so it keeps its heading.
inline Vec2 fleeDesired(Vec2 pos, Vec2 target, Vec2 heading, double maxSpeed, double panicRadius) {
    const Vec2 away = pos - target;
    const double dist = length(away);
    if (dist >= panicRadius) return {0.0, 0.0};
    if (dist < 1e-9) return normalize(heading) * maxSpeed;
    return away * (maxSpeed / dist);
}

/// Like seek, but the wished speed falls linearly with the distance inside `slowRadius` and is zero on the target.
inline Vec2 arriveDesired(Vec2 pos, Vec2 target, double maxSpeed, double slowRadius) {
    const Vec2 toTarget = target - pos;
    const double dist = length(toTarget);
    if (dist < 1e-9) return {0.0, 0.0};
    const double speed = dist >= slowRadius ? maxSpeed : maxSpeed * (dist / slowRadius);
    return toTarget * (speed / dist);
}

/// Smooth noise in [-1, 1], a pure function of time. The same time gives the same wish at any step rate.
inline double wanderNoise(double t) { return 0.6 * std::sin(0.9 * t) + 0.3 * std::sin(2.3 * t + 1.7) + 0.1 * std::sin(5.1 * t + 0.4); }

/// A point on a circle of radius `radius` whose center is `distance` ahead of the agent. `angle` (radians) turns
/// the point around the circle, starting straight ahead.
inline Vec2 wanderTarget(Vec2 pos, Vec2 heading, double angle, double distance, double radius) {
    const Vec2 h = normalize(heading);
    const Vec2 center = pos + h * distance;
    const Vec2 dir{h.x * std::cos(angle) - h.y * std::sin(angle), h.x * std::sin(angle) + h.y * std::cos(angle)};
    return center + dir * radius;
}

struct PathHit {
    Vec2 point;               // closest point on the path
    std::size_t segment = 0;  // it lies on the segment from path[segment] to path[segment + 1] (the last one closes the loop)
    double distance = 0.0;    // from the query point to `point`
    bool valid = false;       // false for a path with fewer than two points
};

/// Closest point on a closed path (the last point connects back to the first).
inline PathHit closestOnPath(const Path& path, Vec2 p) {
    PathHit best;
    const std::size_t n = path.size();
    if (n < 2) return best;
    best.distance = 1e300;
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2 a = path[i];
        const Vec2 b = path[(i + 1) % n];
        const Vec2 ab = b - a;
        const double len2 = ab.x * ab.x + ab.y * ab.y;
        // Project onto the segment and clamp to its ends. A zero length segment is just its point.
        double t = len2 > 1e-18 ? ((p.x - a.x) * ab.x + (p.y - a.y) * ab.y) / len2 : 0.0;
        t = std::clamp(t, 0.0, 1.0);
        const Vec2 q = a + ab * t;
        const double d = distance(p, q);
        if (d < best.distance) best = {q, i, d, true};
    }
    return best;
}

/// The point `dist` px further along the path (in path direction) from a hit, around corners and past the closing segment.
inline Vec2 pointAhead(const Path& path, const PathHit& hit, double dist) {
    const std::size_t n = path.size();
    if (!hit.valid || n < 2) return hit.point;
    double lap = 0.0;
    for (std::size_t i = 0; i < n; ++i) lap += distance(path[i], path[(i + 1) % n]);
    if (lap <= 1e-9) return hit.point;
    double remaining = std::fmod(std::max(dist, 0.0), lap);
    Vec2 cur = hit.point;
    std::size_t i = hit.segment;
    for (std::size_t guard = 0; guard <= n + 1; ++guard) {
        const Vec2 end = path[(i + 1) % n];
        const double len = distance(cur, end);
        if (remaining <= len) return cur + normalize(end - cur) * remaining;
        remaining -= len;
        cur = end;
        i = (i + 1) % n;
    }
    return cur;
}

/// Path following: look where the agent will be soon. Inside the corridor around the path, keep going. Outside, aim at
/// a point a bit further along the path. An agent that stands still has no direction to keep, so it always aims.
inline Vec2 pathDesired(const Path& path, Vec2 pos, Vec2 vel, const PathParams& pp, double maxSpeed) {
    const Vec2 predicted = pos + vel * pp.lookahead;
    const PathHit hit = closestOnPath(path, predicted);
    if (!hit.valid) return {0.0, 0.0};
    if (hit.distance <= pp.radius && length(vel) > 1.0) return normalize(vel) * maxSpeed;
    return seekDesired(pos, pointAhead(path, hit, pp.lead), maxSpeed);
}

inline Vec2 desiredVelocity(Behavior b, const Agent& a, Vec2 target, const Params& p, const Path& path) {
    switch (b) {
        case Behavior::Seek: return seekDesired(a.pos, target, p.maxSpeed);
        case Behavior::Flee: return fleeDesired(a.pos, target, a.heading, p.maxSpeed, p.panicRadius);
        case Behavior::Arrive: return arriveDesired(a.pos, target, p.maxSpeed, p.arriveRadius);
        case Behavior::Wander:
            return seekDesired(a.pos, wanderTarget(a.pos, a.heading, wanderNoise(a.time) * kPi, p.wanderDistance, p.wanderRadius), p.maxSpeed);
        case Behavior::Path: return pathDesired(path, a.pos, a.vel, p.path, p.maxSpeed);
    }
    return {0.0, 0.0};
}

// --- stepping ------------------------------------------------------------------------------------------------------

/// Wraps a point into [left, right) x [top, bottom).
inline Vec2 wrap(Vec2 p, const Params& f) {
    const double w = f.right - f.left;
    const double h = f.bottom - f.top;
    if (w > 0.0) p.x = f.left + std::fmod(std::fmod(p.x - f.left, w) + w, w);
    if (h > 0.0) p.y = f.top + std::fmod(std::fmod(p.y - f.top, h) + h, h);
    return p;
}

/// One step of `dt` seconds: add the missing velocity (capped if `limitForce`), cap the speed, move, wrap.
inline void applySteering(Agent& a, Vec2 desired, const Params& p, double dt, bool limitForce) {
    Vec2 steer = desired - a.vel;
    if (limitForce) steer = truncate(steer, p.maxForce * dt);
    a.vel = truncate(a.vel + steer, p.maxSpeed);
    a.pos = wrap(a.pos + a.vel * dt, p);
    if (length(a.vel) > 1e-3) a.heading = normalize(a.vel);
}

/// One simulation step at `hz` steps per second. With `scaleByDt` the maths use the real step length 1/hz, without it
/// they use 1/60 s whatever the rate is. The clock (`time`) always runs in real seconds.
inline void advance(Agent& a, Behavior b, Vec2 target, const Params& p, const Path& path, double hz, bool scaleByDt, bool limitForce) {
    const Vec2 desired = desiredVelocity(b, a, target, p, path);
    applySteering(a, desired, p, scaleByDt ? 1.0 / hz : 1.0 / kReferenceHz, limitForce);
    a.time += 1.0 / hz;
}

}  // namespace sb
