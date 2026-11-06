// Verlet cloth and ropes without SDL: points, distance links and the solver that repeatedly pushes links back to their
// rest length. Everything that can be wrong is here and covered by test.cpp.
//
// A point stores where it is (pos) and where it was one step ago (prev). The velocity is not stored anywhere, it is
// pos - prev. That is why moving a point by hand or pushing it out of a link also changes its speed, for free.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

namespace vc {

struct Vec2 {
    float x = 0.0F;
    float y = 0.0F;
};

inline Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
inline Vec2 operator*(Vec2 a, float s) { return {a.x * s, a.y * s}; }
inline float dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
inline float length(Vec2 a) { return std::sqrt(dot(a, a)); }

/// Shortens `v` to at most `maxLen`. A zero vector stays zero.
inline Vec2 clampLength(Vec2 v, float maxLen) {
    const float len = length(v);
    const float limit = std::max(maxLen, 0.0F);
    if (len <= limit || len < 1e-9F) return v;
    return v * (limit / len);
}

struct Point {
    Vec2 pos;
    Vec2 prev;            // position one step ago, velocity = pos - prev (Verlet)
    Vec2 vel;             // velocity in px/s, only used by the "stored velocity" integrator
    float invMass = 1.0F;
    bool pinned = false;  // fixed in space
    bool held = false;    // carried by the mouse
    float inv() const { return (pinned || held) ? 0.0F : invMass; }
};

struct Link {
    int a = 0;
    int b = 0;
    float rest = 1.0F;
    bool alive = true;
};

struct World {
    std::vector<Point> pts;
    std::vector<Link> links;
    int clothCols = 0;  // the cloth grid, if any, always comes first in pts and links
    int clothRows = 0;
    float minX = -1.0e9F;
    float maxX = 1.0e9F;
    float floorY = 1.0e9F;
};

struct Params {
    int iterations = 12;     // how often the links are pushed back per step
    float wind = 0.0F;       // 0..1
    bool verlet = true;      // false: velocity is stored per point and positions are corrected behind its back
    float tear = 0.0F;       // a link breaks when longer than rest * tear, 0 = never
    float gravity = 900.0F;  // px/s^2
    float damping = 0.999F;  // per step
};

inline constexpr int kMaxIterations = 60;
inline constexpr float kFloorFriction = 0.5F;
inline constexpr float kWindStrength = 600.0F;  // px/s^2 at full strength and gust 1

/// Gust of wind at time `t` (seconds) and horizontal position `x`. The phases are reduced to one period in double, so
/// the gusts stay smooth after hours, where a float time has a resolution of tenths of a second.
inline Vec2 windAccel(double t, float x, float strength) {
    constexpr double kTwoPi = 6.283185307179586;
    const double slow = std::fmod(t + static_cast<double>(x) * 0.012, kTwoPi);
    const double fast = std::fmod(3.0 * t - static_cast<double>(x) * 0.02, kTwoPi);
    const double gust = 0.55 + 0.45 * std::sin(slow) + 0.2 * std::sin(fast);
    const float ax = static_cast<float>(gust) * kWindStrength * strength;
    return {ax, -0.15F * ax};
}

/// One integration step of a single point. Pinned and held points stay where they are.
inline void integrate(Point& p, Vec2 acc, const Params& prm, float dt) {
    if (p.inv() == 0.0F) return;
    if (prm.verlet) {
        // pos - prev is the velocity of the last step.
        const Vec2 v = (p.pos - p.prev) * prm.damping;
        p.prev = p.pos;
        p.pos = p.pos + v + acc * (dt * dt);
    } else {
        p.vel = (p.vel + acc * dt) * prm.damping;
        p.prev = p.pos;
        p.pos = p.pos + p.vel * dt;
    }
}

/// Pushes the two ends of a link towards its rest length, the lighter end moves more. Nothing happens when both are
/// fixed or when the points coincide (no direction to push in).
inline void satisfy(Point& a, Point& b, float rest) {
    const float w = a.inv() + b.inv();
    const Vec2 d = b.pos - a.pos;
    const float len = length(d);
    if (w <= 0.0F || len < 1e-6F) return;
    const float k = (len - rest) / (len * w);
    a.pos = a.pos + d * (k * a.inv());
    b.pos = b.pos - d * (k * b.inv());
}

/// The mouse carries a point: it jumps to the target and keeps the speed of that jump (at most `maxMove` per step), so
/// letting go does not freeze it.
inline void moveHeld(Point& p, Vec2 target, float maxMove, float dt) {
    const Vec2 old = p.pos;
    p.pos = target;
    const Vec2 d = clampLength(p.pos - old, maxMove);
    p.prev = p.pos - d;
    p.vel = dt > 0.0F ? d * (1.0F / dt) : Vec2{};
}

/// Walls and floor. The speed against them is gone, along the floor it halves.
inline void collide(World& w) {
    for (Point& p : w.pts) {
        if (p.pinned || p.held) continue;
        if (p.pos.x < w.minX) {
            p.pos.x = w.minX;
            p.prev.x = w.minX;
            p.vel.x = 0.0F;
        }
        if (p.pos.x > w.maxX) {
            p.pos.x = w.maxX;
            p.prev.x = w.maxX;
            p.vel.x = 0.0F;
        }
        if (p.pos.y > w.floorY) {
            p.pos.y = w.floorY;
            p.prev.y = w.floorY;
            p.vel.y = 0.0F;
            p.prev.x = p.pos.x - (p.pos.x - p.prev.x) * kFloorFriction;
            p.vel.x *= kFloorFriction;
        }
    }
}

/// One simulation step: forces and integration, then `iterations` rounds over all links, then walls.
inline void step(World& w, const Params& prm, double time, float dt) {
    for (Point& p : w.pts) {
        const Vec2 acc = Vec2{0.0F, prm.gravity} + windAccel(time, p.pos.x, prm.wind);
        integrate(p, acc, prm, dt);
    }
    const int rounds = std::clamp(prm.iterations, 1, kMaxIterations);
    for (int k = 0; k < rounds; ++k) {
        for (Link& l : w.links) {
            if (!l.alive) continue;
            Point& a = w.pts[static_cast<std::size_t>(l.a)];
            Point& b = w.pts[static_cast<std::size_t>(l.b)];
            if (prm.tear > 0.0F && length(b.pos - a.pos) > l.rest * prm.tear) {
                l.alive = false;
                continue;
            }
            satisfy(a, b, l.rest);
        }
    }
    collide(w);
}

/// Distance from point `p` to the segment a-b. A segment of length 0 is a point.
inline float segmentPointDistance(Vec2 p, Vec2 a, Vec2 b) {
    const Vec2 ab = b - a;
    const float len2 = dot(ab, ab);
    const float t = len2 > 0.0F ? std::clamp(dot(p - a, ab) / len2, 0.0F, 1.0F) : 0.0F;
    return length(p - (a + ab * t));
}

namespace detail {
inline float cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }
}  // namespace detail

/// Smallest distance between the segments a-b and c-d, 0 when they cross or touch.
inline float segmentsDistance(Vec2 a, Vec2 b, Vec2 c, Vec2 d) {
    const Vec2 r = b - a;
    const Vec2 s = d - c;
    const float denom = detail::cross(r, s);
    if (std::fabs(denom) > 1e-9F) {
        const float t = detail::cross(c - a, s) / denom;
        const float u = detail::cross(c - a, r) / denom;
        if (t >= 0.0F && t <= 1.0F && u >= 0.0F && u <= 1.0F) return 0.0F;
    }
    // No proper crossing (parallel, collinear, apart, or one of them is a point): the closest approach is at an end.
    return std::min({segmentPointDistance(a, c, d), segmentPointDistance(b, c, d), segmentPointDistance(c, a, b), segmentPointDistance(d, a, b)});
}

/// Index of the point closest to `p` within `maxDist`, the lower index wins a tie.
inline std::optional<std::size_t> nearestPoint(const World& w, Vec2 p, float maxDist) {
    std::optional<std::size_t> best;
    float bestD2 = maxDist * maxDist;
    for (std::size_t i = 0; i < w.pts.size(); ++i) {
        const Vec2 d = w.pts[i].pos - p;
        const float d2 = dot(d, d);
        if (d2 <= bestD2 && (!best || d2 < bestD2)) {
            best = i;
            bestD2 = d2;
        }
    }
    return best;
}

/// Cuts every live link that the swipe from-to touches within `radius`. A click is a swipe of length 0. Returns the count.
inline int cutAlong(World& w, Vec2 from, Vec2 to, float radius) {
    int cut = 0;
    for (Link& l : w.links) {
        if (!l.alive) continue;
        const Vec2 a = w.pts[static_cast<std::size_t>(l.a)].pos;
        const Vec2 b = w.pts[static_cast<std::size_t>(l.b)].pos;
        if (segmentsDistance(a, b, from, to) <= radius) {
            l.alive = false;
            ++cut;
        }
    }
    return cut;
}

/// Pins a point where it is, or lets it go again. Either way it loses its speed.
inline void togglePin(World& w, std::size_t i) {
    if (i >= w.pts.size()) return;
    Point& p = w.pts[i];
    p.pinned = !p.pinned;
    p.prev = p.pos;
    p.vel = {};
}

struct Strain {
    float average = 0.0F;  // mean of |length / rest - 1| over the live links
    float maximum = 0.0F;
    int alive = 0;
};

inline Strain strain(const World& w) {
    Strain s;
    float sum = 0.0F;
    for (const Link& l : w.links) {
        if (!l.alive || l.rest <= 0.0F) continue;
        const float e = std::fabs(length(w.pts[static_cast<std::size_t>(l.b)].pos - w.pts[static_cast<std::size_t>(l.a)].pos) / l.rest - 1.0F);
        sum += e;
        s.maximum = std::max(s.maximum, e);
        ++s.alive;
    }
    if (s.alive > 0) s.average = sum / static_cast<float>(s.alive);
    return s;
}

/// Slider helpers: mouse x on a track starting at `x` with width `w` to a value in [lo, hi], and back.
inline float sliderValue(float mouseX, float x, float w, float lo, float hi) {
    if (w <= 0.0F) return lo;
    return lo + (hi - lo) * std::clamp((mouseX - x) / w, 0.0F, 1.0F);
}

inline float sliderFraction(float value, float lo, float hi) {
    if (hi <= lo) return 0.0F;
    return std::clamp((value - lo) / (hi - lo), 0.0F, 1.0F);
}

// Links of the cloth grid: all horizontal ones row by row, then all vertical ones row by row.
inline int hLinkIndex(int cols, int i, int j) { return j * (cols - 1) + i; }
inline int vLinkIndex(int cols, int rows, int i, int j) { return (cols - 1) * rows + j * cols + i; }

/// A grid of cols x rows points `spacing` apart, starting at (x0, y0), with the top row pinned at every `pinEvery`-th
/// point and at the last one. Call it on an empty world, the index helpers above assume the cloth comes first.
inline void addCloth(World& w, int cols, int rows, float spacing, float x0, float y0, int pinEvery) {
    w.clothCols = cols;
    w.clothRows = rows;
    const int base = static_cast<int>(w.pts.size());
    for (int j = 0; j < rows; ++j) {
        for (int i = 0; i < cols; ++i) {
            Point p;
            p.pos = {x0 + spacing * static_cast<float>(i), y0 + spacing * static_cast<float>(j)};
            p.prev = p.pos;
            p.pinned = j == 0 && (i == cols - 1 || (pinEvery > 0 && i % pinEvery == 0));
            w.pts.push_back(p);
        }
    }
    for (int j = 0; j < rows; ++j) {
        for (int i = 0; i + 1 < cols; ++i) w.links.push_back(Link{base + j * cols + i, base + j * cols + i + 1, spacing, true});
    }
    for (int j = 0; j + 1 < rows; ++j) {
        for (int i = 0; i < cols; ++i) w.links.push_back(Link{base + j * cols + i, base + (j + 1) * cols + i, spacing, true});
    }
}

/// A rope of n points on the straight line from-to. `slack` > 1 makes the links longer than the span, so it sags.
/// `lastInvMass` below 1 hangs a weight on the end.
inline void addRope(World& w, Vec2 from, Vec2 to, int n, float slack, bool pinFirst, bool pinLast, float lastInvMass) {
    if (n < 2) return;
    const int base = static_cast<int>(w.pts.size());
    const float rest = length(to - from) / static_cast<float>(n - 1) * slack;
    for (int k = 0; k < n; ++k) {
        const float t = static_cast<float>(k) / static_cast<float>(n - 1);
        Point p;
        p.pos = from + (to - from) * t;
        p.prev = p.pos;
        p.pinned = (k == 0 && pinFirst) || (k == n - 1 && pinLast);
        if (k == n - 1) p.invMass = lastInvMass;
        w.pts.push_back(p);
        if (k > 0) w.links.push_back(Link{base + k - 1, base + k, rest, true});
    }
}

/// The scene of the demo: a cloth, a rope, a rope with a weight, and a hammock between two posts.
inline World makeScene() {
    World w;
    w.minX = 0.0F;
    w.maxX = 960.0F;
    w.floorY = 404.0F;
    addCloth(w, 31, 20, 14.0F, 60.0F, 64.0F, 5);
    addRope(w, {610.0F, 64.0F}, {610.0F, 302.0F}, 18, 1.0F, true, false, 1.0F);
    addRope(w, {690.0F, 64.0F}, {690.0F, 302.0F}, 18, 1.0F, true, false, 0.1F);
    addRope(w, {770.0F, 90.0F}, {930.0F, 90.0F}, 16, 1.15F, true, true, 1.0F);
    return w;
}

}  // namespace vc
