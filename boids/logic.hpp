// The testable part of the boids demo: the three rules, a predator, a wrap-around world and a uniform grid for the
// neighbor search. No SDL in here, test.cpp covers it with hand-computed cases.
//
// Each rule produces a steering force the way a steering behavior does: desired velocity minus current velocity,
// cut to a maximum force. The sum of the three, weighted by the sliders, is the acceleration of a bird.
// The world is a torus: leaving on the right comes back on the left, and distances take the short way around.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace boids {

struct Vec2 {
    float x = 0.0F;
    float y = 0.0F;
};

inline Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
inline Vec2 operator*(Vec2 a, float s) { return {a.x * s, a.y * s}; }
inline Vec2& operator+=(Vec2& a, Vec2 b) { a = a + b; return a; }
inline Vec2& operator-=(Vec2& a, Vec2 b) { a = a - b; return a; }

inline float lengthSq(Vec2 v) { return v.x * v.x + v.y * v.y; }
inline float length(Vec2 v) { return std::sqrt(lengthSq(v)); }

/// Unit vector, or zero for a (nearly) zero vector instead of a division by zero.
inline Vec2 normalized(Vec2 v) {
    const float len = length(v);
    return len > 1e-6F ? v * (1.0F / len) : Vec2{};
}

/// `v` cut to at most `maxLength`; a limit of 0 or less gives the zero vector.
inline Vec2 limited(Vec2 v, float maxLength) {
    if (!(maxLength > 0.0F)) return {};
    const float len = length(v);
    if (len <= maxLength) return v;
    return v * (maxLength / len);
}

/// `v` moved into [0, size). Rounding can turn a tiny negative number into exactly `size`, that counts as 0.
inline float wrapCoord(float v, float size) {
    float r = std::fmod(v, size);
    if (r < 0.0F) r += size;
    if (r >= size) r = 0.0F;
    return r;
}

/// Signed shortest way from `from` to `to` on a circle of length `size`, both inside [0, size).
/// Exactly half the circle stays as it is, both ways are equally long.
inline float wrapDelta(float from, float to, float size) {
    float d = to - from;
    const float half = size * 0.5F;
    if (d > half) d -= size;
    else if (d < -half) d += size;
    return d;
}

struct World {
    float width = 960.0F;
    float height = 370.0F;
};

struct Params {
    float perception = 50.0F;        // neighbors are the birds closer than this
    float separationRadius = 14.0F;  // birds closer than this push each other away
    float maxSpeed = 130.0F;         // px/s
    float minSpeed = 70.0F;          // birds never hover
    float maxForce = 260.0F;         // px/s^2 per rule
    float fearRadius = 90.0F;        // the predator scares everything closer than this
    float fearBoost = 4.0F;          // fleeing may use this many times maxForce
    float separation = 2.0F;         // slider weights
    float alignment = 1.0F;
    float cohesion = 1.5F;
};

struct Predator {
    Vec2 pos;
    bool active = false;
};

struct Bird {
    Vec2 pos;
    Vec2 vel;
    bool scared = false;
};

/// What one bird has seen of the others so far.
struct Sums {
    Vec2 away;        // sum of "away from the neighbor, weighted 1/distance" over the crowding ones
    Vec2 vel;         // sum of the neighbors' velocities
    Vec2 offset;      // sum of the vectors to the neighbors
    int count = 0;    // neighbors within perception
    int crowd = 0;    // neighbors within the separation radius
};

/// Adds `other` to what `me` has seen, if it is a neighbor. Strict `<`: a bird exactly on the radius does not count.
inline void accumulate(Sums& s, const Bird& me, const Bird& other, const World& w, const Params& p) {
    const Vec2 d{wrapDelta(me.pos.x, other.pos.x, w.width), wrapDelta(me.pos.y, other.pos.y, w.height)};
    const float distSq = lengthSq(d);
    if (!(distSq < p.perception * p.perception)) return;
    s.vel += other.vel;
    s.offset += d;
    ++s.count;
    // Two birds on the very same spot have no direction to part in; their different velocities part them soon enough.
    if (distSq < p.separationRadius * p.separationRadius && distSq > 1e-12F) {
        s.away -= d * (1.0F / distSq);
        ++s.crowd;
    }
}

/// The steering force: velocity wanted minus velocity had, cut to `maxForce`. No wish (zero vector) means no force.
inline Vec2 steerToward(Vec2 desired, Vec2 vel, float maxForce) {
    if (lengthSq(desired) == 0.0F) return {};
    return limited(desired - vel, maxForce);
}

/// Separation, alignment and cohesion, each weighted by its slider.
inline Vec2 flockSteering(const Bird& me, const Sums& s, const Params& p) {
    Vec2 total;
    if (s.crowd > 0) {
        total += steerToward(normalized(s.away) * p.maxSpeed, me.vel, p.maxForce) * p.separation;
    }
    if (s.count > 0) {
        total += steerToward(normalized(s.vel) * p.maxSpeed, me.vel, p.maxForce) * p.alignment;
        // Towards the center of the neighbors, fading out inside half the perception so a bird in the middle of
        // its group does not jitter around a center that is almost on top of it.
        const Vec2 center = s.offset * (1.0F / static_cast<float>(s.count));
        const float dist = length(center);
        const float fade = std::min(1.0F, dist / (p.perception * 0.5F));
        total += steerToward(normalized(center) * p.maxSpeed, me.vel, p.maxForce) * (p.cohesion * fade);
    }
    return total;
}

/// Closer than the fear radius to an active predator.
inline bool inFear(const Bird& me, const Predator& pr, const World& w, const Params& p) {
    if (!pr.active) return false;
    const Vec2 d{wrapDelta(me.pos.x, pr.pos.x, w.width), wrapDelta(me.pos.y, pr.pos.y, w.height)};
    return lengthSq(d) < p.fearRadius * p.fearRadius;
}

/// Flee at full speed, harder the closer the predator is: zero at the edge of the fear radius, full at distance 0.
inline Vec2 fleeSteering(const Bird& me, const Predator& pr, const World& w, const Params& p) {
    if (!inFear(me, pr, w, p)) return {};
    const Vec2 d{wrapDelta(me.pos.x, pr.pos.x, w.width), wrapDelta(me.pos.y, pr.pos.y, w.height)};
    const float dist = length(d);
    Vec2 away = dist > 1e-6F ? d * (-1.0F / dist) : normalized(me.vel);
    if (lengthSq(away) == 0.0F) away = {1.0F, 0.0F};  // on top of the predator and standing still: any way will do
    const float strength = 1.0F - dist / p.fearRadius;
    return steerToward(away * p.maxSpeed, me.vel, p.maxForce * p.fearBoost) * strength;
}

/// Speed forced into [minSpeed, maxSpeed]; a bird at rest gets a direction to the right.
inline Vec2 clampSpeed(Vec2 vel, const Params& p) {
    const float len = length(vel);
    if (len > p.maxSpeed) return vel * (p.maxSpeed / len);
    if (len < p.minSpeed) {
        if (len < 1e-6F) return {p.minSpeed, 0.0F};
        return vel * (p.minSpeed / len);
    }
    return vel;
}

enum class Search { Grid, Brute };

/// Uniform grid over the torus. Cells are never smaller than the perception, so every neighbor of a bird lies in
/// its own cell or one of the eight around it (wrapping at the edges). Built by counting sort: no allocation per bird.
class Grid {
public:
    void build(const std::vector<Bird>& birds, const World& world, float cellMin) {
        const float width = world.width > 0.0F ? world.width : 1.0F;
        const float height = world.height > 0.0F ? world.height : 1.0F;
        const float m = cellMin > 1.0F ? cellMin : 1.0F;
        // Clamped as a float first: a huge world over a tiny cell must not become an out-of-range int cast.
        cols_ = static_cast<int>(std::clamp(std::floor(width / m), 1.0F, static_cast<float>(kMaxCells)));
        rows_ = static_cast<int>(std::clamp(std::floor(height / m), 1.0F, static_cast<float>(kMaxCells)));
        cellW_ = width / static_cast<float>(cols_);
        cellH_ = height / static_cast<float>(rows_);
        const std::size_t cells = static_cast<std::size_t>(cols_) * static_cast<std::size_t>(rows_);
        start_.assign(cells + 1, 0);
        for (const Bird& b : birds) ++start_[cellOf(b.pos) + 1];
        for (std::size_t c = 0; c < cells; ++c) start_[c + 1] += start_[c];
        cursor_.assign(start_.begin(), start_.end() - 1);
        items_.resize(birds.size());
        for (std::size_t i = 0; i < birds.size(); ++i) items_[static_cast<std::size_t>(cursor_[cellOf(birds[i].pos)]++)] = static_cast<int>(i);
    }

    int cols() const { return cols_; }
    int rows() const { return rows_; }
    float cellWidth() const { return cellW_; }
    float cellHeight() const { return cellH_; }

    int cellX(float x) const { return index(x / cellW_, cols_); }
    int cellY(float y) const { return index(y / cellH_, rows_); }

    /// Calls `f(birdIndex)` for every bird in the 3x3 block of cells around `pos`, the bird itself included.
    /// Each cell is visited once, also when the grid has fewer than three columns or rows.
    template <class F>
    void forEachCandidate(Vec2 pos, F&& f) const {
        int xs[3];
        int ys[3];
        const int nx = block(cellX(pos.x), cols_, xs);
        const int ny = block(cellY(pos.y), rows_, ys);
        for (int iy = 0; iy < ny; ++iy) {
            for (int ix = 0; ix < nx; ++ix) {
                const std::size_t cell = static_cast<std::size_t>(ys[iy]) * static_cast<std::size_t>(cols_) + static_cast<std::size_t>(xs[ix]);
                for (int k = start_[cell]; k < start_[cell + 1]; ++k) f(items_[static_cast<std::size_t>(k)]);
            }
        }
    }

    /// The distinct cell columns (or rows) of the block around `center`, wrapped; returns how many.
    static int block(int center, int n, int out[3]) {
        int count = 0;
        for (int d = -1; d <= 1; ++d) {
            const int v = (center + d + n) % n;
            bool seen = false;
            for (int k = 0; k < count; ++k) seen = seen || out[k] == v;
            if (!seen) out[count++] = v;
        }
        return count;
    }

private:
    static constexpr int kMaxCells = 1024;

    /// Cell index from a position in cell units. Clamped, so the edge value and rounding never leave the grid;
    /// NaN lands in cell 0 instead of becoming an undefined cast.
    static int index(float f, int n) {
        if (!(f > 0.0F)) return 0;
        if (f >= static_cast<float>(n)) return n - 1;
        return static_cast<int>(f);
    }

    std::size_t cellOf(Vec2 p) const { return static_cast<std::size_t>(cellY(p.y)) * static_cast<std::size_t>(cols_) + static_cast<std::size_t>(cellX(p.x)); }

    int cols_ = 1;
    int rows_ = 1;
    float cellW_ = 1.0F;
    float cellH_ = 1.0F;
    std::vector<int> start_;
    std::vector<int> cursor_;
    std::vector<int> items_;
};

class Flock {
public:
    static constexpr std::size_t kMaxBirds = 5000;

    explicit Flock(World world) : world_(world) {}

    const World& world() const { return world_; }
    std::vector<Bird>& birds() { return birds_; }
    const std::vector<Bird>& birds() const { return birds_; }
    const std::vector<Vec2>& accel() const { return accel_; }
    const Grid& grid() const { return grid_; }
    /// Distance tests of the last acceleration pass: one per (bird, other bird) pair that was actually looked at.
    long long lastTests() const { return tests_; }

    /// `count` birds (0 up to kMaxBirds) at random spots with random headings. Same seed, same flock.
    void reset(int count, std::uint32_t seed, const Params& p) {
        const std::size_t n = count < 0 ? 0 : std::min(static_cast<std::size_t>(count), kMaxBirds);
        rng_ = seed * 2654435761U + 0x9E3779B9U;
        if (rng_ == 0) rng_ = 1;
        for (int i = 0; i < 4; ++i) random01(rng_);
        birds_.clear();
        grow(n, p);
        tests_ = 0;
    }

    /// Changes the number of birds and keeps the ones that stay: fewer cuts the end off, more adds random newcomers.
    void setCount(int count, const Params& p) {
        const std::size_t n = count < 0 ? 0 : std::min(static_cast<std::size_t>(count), kMaxBirds);
        if (n < birds_.size()) birds_.resize(n);
        else grow(n, p);
        accel_.assign(birds_.size(), Vec2{});
    }

    /// Fills accel() from the current state and sets every bird's `scared` flag. Returns the number of distance tests.
    long long computeAccelerations(const Params& p, const Predator& predator, Search search) {
        accel_.assign(birds_.size(), Vec2{});
        long long tests = 0;
        if (search == Search::Grid) grid_.build(birds_, world_, p.perception);
        const float maxTotal = p.maxForce * p.fearBoost;  // fleeing needs more room than the flock rules
        for (std::size_t i = 0; i < birds_.size(); ++i) {
            Sums s;
            if (search == Search::Brute) {
                for (std::size_t j = 0; j < birds_.size(); ++j) {
                    if (j == i) continue;
                    ++tests;
                    accumulate(s, birds_[i], birds_[j], world_, p);
                }
            } else {
                grid_.forEachCandidate(birds_[i].pos, [&](int j) {
                    if (static_cast<std::size_t>(j) == i) return;
                    ++tests;
                    accumulate(s, birds_[i], birds_[static_cast<std::size_t>(j)], world_, p);
                });
            }
            birds_[i].scared = inFear(birds_[i], predator, world_, p);
            accel_[i] = limited(flockSteering(birds_[i], s, p) + fleeSteering(birds_[i], predator, world_, p), maxTotal);
        }
        tests_ = tests;
        return tests;
    }

    /// One simulation step of `dt` seconds. All accelerations come from the state before the step.
    void step(float dt, const Params& p, const Predator& predator, Search search) {
        computeAccelerations(p, predator, search);
        for (std::size_t i = 0; i < birds_.size(); ++i) {
            Bird& b = birds_[i];
            b.vel = clampSpeed(b.vel + accel_[i] * dt, p);
            b.pos = {wrapCoord(b.pos.x + b.vel.x * dt, world_.width), wrapCoord(b.pos.y + b.vel.y * dt, world_.height)};
        }
    }

    /// 0 when the headings cancel out (random), 1 when everyone flies the same way.
    float polarization() const {
        if (birds_.empty()) return 0.0F;
        Vec2 sum;
        for (const Bird& b : birds_) sum += normalized(b.vel);
        return length(sum) / static_cast<float>(birds_.size());
    }

private:
    void grow(std::size_t n, const Params& p) {
        birds_.reserve(n);
        while (birds_.size() < n) {
            Bird b;
            b.pos = {wrapCoord(random01(rng_) * world_.width, world_.width), wrapCoord(random01(rng_) * world_.height, world_.height)};
            const float angle = 6.2831853F * random01(rng_);
            const float speed = p.minSpeed + (p.maxSpeed - p.minSpeed) * random01(rng_);
            b.vel = {std::cos(angle) * speed, std::sin(angle) * speed};
            birds_.push_back(b);
        }
        accel_.assign(birds_.size(), Vec2{});
    }

    /// xorshift32, mapped to [0, 1).
    static float random01(std::uint32_t& state) {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        return static_cast<float>(state >> 8) * (1.0F / 16777216.0F);
    }

    World world_;
    std::vector<Bird> birds_;
    std::vector<Vec2> accel_;
    Grid grid_;
    std::uint32_t rng_ = 1;
    long long tests_ = 0;
};

/// 1234567 -> "1,234,567". Built by hand, printf would use the locale of the machine.
inline std::string formatCount(long long value) {
    unsigned long long u = value < 0 ? 0ULL - static_cast<unsigned long long>(value) : static_cast<unsigned long long>(value);
    std::string digits = std::to_string(u);
    std::string out;
    for (std::size_t i = 0; i < digits.size(); ++i) {
        if (i > 0 && (digits.size() - i) % 3 == 0) out += ',';
        out += digits[i];
    }
    return value < 0 ? "-" + out : out;
}

/// Slider value for a mouse x on a track starting at `trackX`, `trackW` wide. Outside the track it clamps.
inline float sliderValue(float mouseX, float trackX, float trackW, float lo, float hi) {
    if (!(trackW > 0.0F)) return lo;
    const float t = std::clamp((mouseX - trackX) / trackW, 0.0F, 1.0F);
    return lo + (hi - lo) * t;
}

/// Where a value sits on the track as 0..1.
inline float sliderFraction(float value, float lo, float hi) {
    if (!(hi > lo)) return 0.0F;
    return std::clamp((value - lo) / (hi - lo), 0.0F, 1.0F);
}

}  // namespace boids
