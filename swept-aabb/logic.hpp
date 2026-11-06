// Swept AABB: when, within one frame, does a moving box hit a still box, and from which side?
//
// No SDL in here, everything that can be wrong is covered by test.cpp. All numbers are in lane coordinates:
// pixels, y pointing down, one frame is one call of step(), a velocity is "pixels per frame".
#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <span>
#include <vector>

namespace sw {

struct Vec2 {
    float x = 0.0F;
    float y = 0.0F;
};

inline Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
inline Vec2 operator*(Vec2 a, float s) { return {a.x * s, a.y * s}; }
inline float dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
inline float length(Vec2 a) { return std::sqrt(dot(a, a)); }

/// Axis-aligned box by its corners, min is the top left corner.
struct Aabb {
    Vec2 min;
    Vec2 max;
};

inline Vec2 center(const Aabb& b) { return {(b.min.x + b.max.x) * 0.5F, (b.min.y + b.max.y) * 0.5F}; }
inline Aabb translated(const Aabb& b, Vec2 d) { return {b.min + d, b.max + d}; }

/// The test most code starts with. Touching edges do not count as overlap.
inline bool overlaps(const Aabb& a, const Aabb& b) {
    return a.min.x < b.max.x && a.max.x > b.min.x && a.min.y < b.max.y && a.max.y > b.min.y;
}

struct Hit {
    bool hit = false;
    float t = 1.0F;   // fraction of the frame's motion that is free, 0 = blocked at once, 1 = reaches the wall at the very end
    Vec2 normal;      // unit vector along one axis, pointing from the wall back at the box
};

namespace detail {

struct Slab {
    float entry;
    float exit;
};

/// When does a box [bMin,bMax] moving with speed v on this axis start and stop overlapping [sMin,sMax]?
inline Slab slab(float bMin, float bMax, float sMin, float sMax, float v) {
    constexpr float kInf = std::numeric_limits<float>::infinity();
    if (v == 0.0F) {
        // No motion on this axis: the boxes either overlap here for all time or never. Touching is not overlapping.
        if (bMax <= sMin || bMin >= sMax) return {kInf, -kInf};
        return {-kInf, kInf};
    }
    if (v > 0.0F) return {(sMin - bMax) / v, (sMax - bMin) / v};
    return {(sMax - bMin) / v, (sMin - bMax) / v};
}

}  // namespace detail

/// Moves `box` by the whole of `v` and reports the first moment it touches `obstacle`.
/// The boxes overlap exactly while both axes overlap, so the time of impact is the LAST of the two entry times, and
/// the axis that enters last is the side that was hit. A box that starts out overlapping, moves away, stands still
/// or just grazes a corner has no hit.
inline Hit sweep(const Aabb& box, Vec2 v, const Aabb& obstacle) {
    const detail::Slab sx = detail::slab(box.min.x, box.max.x, obstacle.min.x, obstacle.max.x, v.x);
    const detail::Slab sy = detail::slab(box.min.y, box.max.y, obstacle.min.y, obstacle.max.y, v.y);
    const float entry = std::max(sx.entry, sy.entry);
    const float exit = std::min(sx.exit, sy.exit);

    Hit h;
    if (entry < 0.0F || entry > 1.0F || entry >= exit) return h;
    h.hit = true;
    h.t = entry;
    if (sx.entry >= sy.entry) {
        h.normal = {v.x > 0.0F ? -1.0F : 1.0F, 0.0F};
    } else {
        h.normal = {0.0F, v.y > 0.0F ? -1.0F : 1.0F};
    }
    return h;
}

/// After a hit the box is parked this far in front of the wall. Without it, rounding can leave it a hair inside, and
/// then the next sweep starts out overlapping and finds nothing.
inline constexpr float kSkin = 0.02F;

/// Moves a box by v through a set of obstacles: advance to the nearest hit, then either stop there or slide along
/// the wall with the rest of the motion. Three passes are enough to go around a corner, what is left after that is dropped.
inline Aabb moveSwept(Aabb box, Vec2 v, std::span<const Aabb> obstacles, bool slide) {
    Vec2 remaining = v;
    for (int pass = 0; pass < 3; ++pass) {
        if (remaining.x == 0.0F && remaining.y == 0.0F) break;
        Hit best;
        for (const Aabb& o : obstacles) {
            const Hit h = sweep(box, remaining, o);
            if (h.hit && (!best.hit || h.t < best.t)) best = h;
        }
        if (!best.hit) {
            box = translated(box, remaining);
            break;
        }
        box = translated(box, remaining * best.t + best.normal * kSkin);
        if (!slide) break;
        remaining = remaining * (1.0F - best.t);
        remaining = remaining - best.normal * dot(remaining, best.normal);  // keep only the part along the wall
    }
    return box;
}

/// The naive way: move, then look whether the new position overlaps. If it does, the move is refused.
struct NaiveStep {
    Aabb box;
    bool blocked = false;   // a move was refused
    bool missed = false;    // the sweep says the box crossed the wall this frame, the overlap test did not notice
    float shortBy = 0.0F;   // distance along the motion that the sweep would still have covered before the wall
};

inline NaiveStep naiveStep(const Aabb& box, Vec2 v, const Aabb& wall, bool slide) {
    NaiveStep out;
    out.box = box;
    const Hit truth = sweep(box, v, wall);
    if (!slide) {
        const Aabb moved = translated(box, v);
        if (overlaps(moved, wall)) {
            out.blocked = true;
        } else {
            out.box = moved;
        }
    } else {
        // Each axis on its own, the usual way to get sliding out of an overlap test.
        const Aabb movedX = translated(out.box, {v.x, 0.0F});
        if (overlaps(movedX, wall)) {
            out.blocked = true;
        } else {
            out.box = movedX;
        }
        const Aabb movedY = translated(out.box, {0.0F, v.y});
        if (overlaps(movedY, wall)) {
            out.blocked = true;
        } else {
            out.box = movedY;
        }
    }
    out.missed = !out.blocked && truth.hit;
    out.shortBy = out.blocked && truth.hit ? length(v) * truth.t : 0.0F;
    return out;
}

// ---- the scene of the demo: two lanes, same wall, same box ----

inline constexpr float kFieldW = 960.0F;
inline constexpr float kLaneH = 190.0F;
inline constexpr float kBoxSize = 36.0F;
inline constexpr float kStartX = 60.0F;     // left edge of the box at the start
inline constexpr float kWallX = 590.0F;
inline constexpr float kWallTop = 35.0F;
inline constexpr float kWallBottom = 155.0F;
inline constexpr int kMaxFrames = 80;

inline Aabb wallBox(float thickness) { return {{kWallX, kWallTop}, {kWallX + thickness, kWallBottom}}; }

inline Aabb startBox(float centerY) {
    return {{kStartX, centerY - kBoxSize * 0.5F}, {kStartX + kBoxSize, centerY + kBoxSize * 0.5F}};
}

/// Direction in degrees, 0 = right, positive = down (y points down).
inline Vec2 velocity(float speed, float degrees) {
    const float rad = degrees * 3.14159265F / 180.0F;
    return {speed * std::cos(rad), speed * std::sin(rad)};
}

enum class Mode { Naive, Sweep };

/// First frame in which the sweep reports a hit: the frame the demo draws the time of impact for.
struct KeyFrame {
    bool valid = false;
    Aabb from;
    Vec2 v;
    Hit hit;
};

inline Aabb impactBox(const KeyFrame& k) { return translated(k.from, k.v * k.hit.t); }

/// One box on its way through the scene, advanced one frame at a time.
struct Run {
    Mode mode = Mode::Naive;
    Aabb box;
    std::vector<Aabb> trail;  // the box after every frame, starting position first
    int frame = 0;
    int still = 0;            // consecutive frames without movement
    bool done = false;
    bool blocked = false;     // naive: a move was refused in the last frame
    bool missed = false;      // naive: at some frame the box crossed the wall without an overlap
    float shortBy = 0.0F;
    KeyFrame key;
};

inline Run startRun(Mode mode, const Aabb& box) {
    Run run;
    run.mode = mode;
    run.box = box;
    run.trail.push_back(box);
    return run;
}

inline void step(Run& run, Vec2 v, const Aabb& wall, bool slide) {
    if (run.done) return;
    const Aabb from = run.box;
    const Hit truth = sweep(from, v, wall);
    if (truth.hit && !run.key.valid) run.key = KeyFrame{true, from, v, truth};

    if (run.mode == Mode::Naive) {
        const NaiveStep n = naiveStep(from, v, wall, slide);
        run.box = n.box;
        run.blocked = n.blocked;
        run.shortBy = n.shortBy;
        if (n.missed) run.missed = true;
    } else {
        run.box = moveSwept(from, v, std::span<const Aabb>(&wall, 1), slide);
    }

    ++run.frame;
    run.trail.push_back(run.box);
    const Vec2 moved = center(run.box) - center(from);
    run.still = length(moved) < 0.01F ? run.still + 1 : 0;
    const bool outside = run.box.min.x > kFieldW || run.box.max.x < 0.0F || run.box.min.y > kLaneH || run.box.max.y < 0.0F;
    if (run.still >= 2 || outside || run.frame >= kMaxFrames) run.done = true;
}

}  // namespace sw
