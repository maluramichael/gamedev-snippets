// The maths of the starfield demo, no SDL, covered by test.cpp.
//
// Camera space: x right, y down (like the screen), z into the scene, the camera at the origin looking along +z.
// Flying with velocity v moves every star by -v per second. A star at (x, y, z) lands at centre + focal * (x, y) / z.
//
// The world is a periodic box: a star that leaves it on one side comes back on the other, and a star that flies
// past the camera (z < 0) comes back at the far end with new x and y. That keeps the density in the view volume
// constant, also while steering sideways, and needs no special case for stars entering from the edge.
//
// The layered version (the naive one) lives at the bottom: every star has a fixed screen position and one of three
// layers, each with its own pixel speed. No depth, so no perspective.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace sf {

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

/// focal in px, near and far in world units. Nothing at z <= near is drawn.
struct Lens {
    double focal;
    double near;
    double far;
};

/// Half extents in x and y and the depth of the periodic world box (z runs from 0 to depth).
struct Box {
    double halfX;
    double halfY;
    double depth;
};

constexpr double kMinDepth = 1e-4;  // the only protection the unguarded divide gets: never exactly zero

// ------------------------------------------------------------------ projection

/// Perspective projection. A point at or behind the near plane is not drawn: no divide at all.
inline std::optional<Vec2> project(Vec3 p, Vec2 centre, const Lens& lens) {
    if (p.z <= lens.near) return std::nullopt;
    return Vec2{centre.x + lens.focal * p.x / p.z, centre.y + lens.focal * p.y / p.z};
}

/// The bug: divide by z and trust it. Just in front of the camera the result shoots off to thousands of pixels,
/// behind it the image is mirrored. Only an exact zero is kept away from the divide.
inline Vec2 projectUnguarded(Vec3 p, Vec2 centre, const Lens& lens) {
    const double d = std::fabs(p.z) < kMinDepth ? kMinDepth : p.z;
    return {centre.x + lens.focal * p.x / d, centre.y + lens.focal * p.y / d};
}

/// Where the flight direction points on the screen, the point every star flows away from.
/// Without forward speed there is none.
inline std::optional<Vec2> vanishingPoint(Vec3 v, Vec2 centre, double focal) {
    if (v.z <= 1e-9) return std::nullopt;
    return Vec2{centre.x + focal * v.x / v.z, centre.y + focal * v.y / v.z};
}

/// The velocity with forward speed `speed` whose vanishing point is `target`.
inline Vec3 velocityToward(Vec2 target, Vec2 centre, double focal, double speed) {
    return {(target.x - centre.x) / focal * speed, (target.y - centre.y) / focal * speed, speed};
}

struct Streak {
    Vec2 head;  // where the star is now
    Vec2 tail;  // where it was `exposure` seconds ago
};

/// The streak of a moving star: its position now and its position `exposure` seconds ago, both projected.
/// The star moves by -v, so the earlier position is p + v * exposure.
inline std::optional<Streak> streak(Vec3 p, Vec3 vel, double exposure, Vec2 centre, const Lens& lens) {
    const auto head = project(p, centre, lens);
    const auto tail = project({p.x + vel.x * exposure, p.y + vel.y * exposure, p.z + vel.z * exposure}, centre, lens);
    if (!head || !tail) return std::nullopt;
    return Streak{*head, *tail};
}

// ------------------------------------------------------------------ the world

class Rng {
public:
    explicit Rng(std::uint64_t seed) : s_(seed != 0 ? seed : 0x9E3779B97F4A7C15ULL) {}

    std::uint64_t next() {
        s_ ^= s_ >> 12;
        s_ ^= s_ << 25;
        s_ ^= s_ >> 27;
        return s_ * 0x2545F4914F6CDD1DULL;
    }

    /// Uniform in [0, 1).
    double uniform() { return static_cast<double>(next() >> 11) * (1.0 / 9007199254740992.0); }

    double range(double lo, double hi) { return lo + (hi - lo) * uniform(); }

private:
    std::uint64_t s_;
};

/// Folds v into [lo, lo + len). The upper edge is not part of the range.
inline double wrapInto(double v, double lo, double len) {
    double r = v - lo;
    r -= std::floor(r / len) * len;
    if (r >= len || r < 0.0) r = 0.0;  // rounding of a tiny negative value can give exactly len
    return lo + r;
}

constexpr int kTints = 5;  // blue white, white, pale yellow, orange, faint red

struct Star {
    Vec3 p;
    float mag;  // intrinsic brightness, 0 to 1
    std::uint8_t tint;
};

/// Far: a star passing the camera comes back at the far end (the right way).
/// Anywhere: it gets a new random depth, which makes stars appear out of nowhere in the middle of the view.
enum class Respawn { Far, Anywhere };

/// Moves a star for dt seconds of flight with velocity vel. Returns how often it wrapped in depth.
inline int advance(Star& s, Vec3 vel, double dt, const Box& box, Respawn mode, Rng& rng) {
    double x = s.p.x - vel.x * dt;
    double y = s.p.y - vel.y * dt;
    const double z = s.p.z - vel.z * dt;
    const double laps = std::floor(z / box.depth);
    x = wrapInto(x, -box.halfX, 2.0 * box.halfX);
    y = wrapInto(y, -box.halfY, 2.0 * box.halfY);
    if (laps == 0.0) {
        s.p = {x, y, z};
        return 0;
    }
    // Passed the camera (or, flying backwards, the far end): fresh x and y over the whole slab.
    s.p.x = rng.range(-box.halfX, box.halfX);
    s.p.y = rng.range(-box.halfY, box.halfY);
    s.p.z = mode == Respawn::Far ? wrapInto(z, 0.0, box.depth) : rng.range(0.0, box.depth);
    return static_cast<int>(std::fabs(laps));
}

inline std::vector<Star> makeStars(std::size_t count, const Box& box, Rng& rng) {
    std::vector<Star> out;
    out.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        const double u = rng.uniform();
        Star s;
        s.p = {rng.range(-box.halfX, box.halfX), rng.range(-box.halfY, box.halfY), rng.range(0.0, box.depth)};
        s.mag = static_cast<float>(0.3 + 0.7 * u * u);
        s.tint = static_cast<std::uint8_t>(std::min(kTints - 1, static_cast<int>(rng.uniform() * kTints)));
        out.push_back(s);
    }
    return out;
}

/// Is the point inside the pyramid the window sees? half is half the window in px.
inline bool inFrustum(Vec3 p, Vec2 half, const Lens& lens) {
    if (p.z <= lens.near || p.z > lens.far) return false;
    return std::fabs(p.x) <= half.x * p.z / lens.focal && std::fabs(p.y) <= half.y * p.z / lens.focal;
}

inline double smoothstep(double t) {
    t = std::clamp(t, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

/// 0 at the near plane, 1 from three times the near distance, back to 0 over the last quarter before the far plane.
/// Stars fade in and out instead of popping.
inline double depthFade(double z, const Lens& lens) {
    if (z <= lens.near || z >= lens.far) return 0.0;
    const double in = smoothstep((z - lens.near) / (2.0 * lens.near));
    const double out = smoothstep((lens.far - z) / (0.25 * lens.far));
    return in * out;
}

constexpr double kStarRadius = 0.0012;  // world units of a star, its size on the screen is focal * radius / z
constexpr double kMaxStarPx = 3.2;
constexpr double kMinStarPx = 0.5;

/// Apparent radius in px. Depth at or behind the near plane counts as the near plane, so the divide is safe.
inline double apparentRadius(double z, const Lens& lens) {
    const double d = std::max(z, lens.near);
    return std::clamp(kStarRadius * lens.focal / d, kMinStarPx, kMaxStarPx);
}

// ------------------------------------------------------------------ steering and smoothing

/// Frame-rate independent smoothing: two steps of dt/2 give exactly one step of dt.
inline double approach(double current, double target, double lambda, double dt) {
    return current + (target - current) * (1.0 - std::exp(-lambda * dt));
}

/// Keeps the steering target inside the window, at most `fraction` of the way from the centre to each edge.
inline Vec2 clampHeading(Vec2 target, Vec2 centre, Vec2 half, double fraction) {
    return {centre.x + std::clamp(target.x - centre.x, -fraction * half.x, fraction * half.x),
            centre.y + std::clamp(target.y - centre.y, -fraction * half.y, fraction * half.y)};
}

/// Slow Lissajous drift of the heading (offset from the centre in px), so the first frames already steer.
inline Vec2 autopilot(double t, Vec2 amplitude) {
    return {amplitude.x * std::sin(0.37 * t), amplitude.y * std::sin(0.53 * t + 1.0)};
}

// ------------------------------------------------------------------ the layered version (naive)

constexpr int kFlatLayerCount = 3;

struct FlatLayer {
    double speed;     // px per second away from the centre, the same for every star of the layer
    double parallax;  // share of the steering offset this layer follows
    float size;       // px
    float brightness;
};

constexpr FlatLayer kFlatLayers[kFlatLayerCount] = {{30.0, 0.2, 1.0F, 0.45F}, {90.0, 0.5, 1.6F, 0.7F}, {240.0, 1.0, 2.4F, 1.0F}};
constexpr double kFlatSpawnRadius = 40.0;
constexpr double kFlatMargin = 8.0;

struct FlatStar {
    double angle;
    double radius;  // px from the centre
    int layer;
    float mag;
    std::uint8_t tint;
};

/// Screen position: out from the centre along the angle, shifted by the layer's share of the steering offset.
inline Vec2 flatPosition(const FlatStar& s, Vec2 centre, Vec2 steer) {
    const double k = kFlatLayers[s.layer].parallax;
    return {centre.x + std::cos(s.angle) * s.radius + steer.x * k, centre.y + std::sin(s.angle) * s.radius + steer.y * k};
}

/// Moves a star outwards at its layer's speed. Leaving the window sends it back to the middle: a new angle and a
/// radius uniform over the area of a small disc (sqrt of the random number, see the post on discs). True if it did.
inline bool advanceFlat(FlatStar& s, double dt, Vec2 centre, Vec2 steer, Vec2 half, Rng& rng) {
    s.radius += kFlatLayers[s.layer].speed * dt;
    const Vec2 p = flatPosition(s, centre, steer);
    if (std::fabs(p.x - centre.x) <= half.x + kFlatMargin && std::fabs(p.y - centre.y) <= half.y + kFlatMargin) return false;
    s.angle = rng.uniform() * 6.283185307179586;
    s.radius = kFlatSpawnRadius * std::sqrt(rng.uniform());
    return true;
}

/// Stars spread uniformly over the window, the larger share on the slow layers.
inline std::vector<FlatStar> makeFlatStars(std::size_t count, Vec2 half, Rng& rng) {
    std::vector<FlatStar> out;
    out.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        const double x = rng.range(-half.x, half.x);
        const double y = rng.range(-half.y, half.y);
        const double u = rng.uniform();
        FlatStar s;
        s.angle = std::atan2(y, x);
        s.radius = std::hypot(x, y);
        s.layer = u < 0.55 ? 0 : (u < 0.85 ? 1 : 2);
        s.mag = static_cast<float>(0.5 + 0.5 * rng.uniform());
        s.tint = static_cast<std::uint8_t>(std::min(kTints - 1, static_cast<int>(rng.uniform() * kTints)));
        out.push_back(s);
    }
    return out;
}

}  // namespace sf
