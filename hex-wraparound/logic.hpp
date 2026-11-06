// Hex map wraparound without SDL. A hexagon-shaped map of radius N has 3N(N+1)+1 tiles. Copies of it, moved by the six
// "mirror centers" (2N+1, -N, -N-1) turned in 60 degree steps, cover the plane without gaps or overlap, so leaving the
// map on one side is the same as entering the next copy, and subtracting that copy's center brings the tile back
// onto the map. Everything that can be wrong lives here and is checked by test.cpp. Pixel positions are for a
// pointy hex of "size" 1 (center to corner), y points down (screen space).
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <set>
#include <tuple>
#include <vector>

namespace hexwrap {

struct Cube {
    int q, r, s;
};
struct Point {
    double x, y;
};

constexpr double kSqrt3 = 1.7320508075688772;
constexpr double kPi = 3.14159265358979323846;

/// Cube from the two stored axes, the third one follows: s = -q - r.
inline Cube make(int q, int r) { return {q, r, -q - r}; }
inline bool operator==(Cube a, Cube b) { return a.q == b.q && a.r == b.r && a.s == b.s; }
inline Cube operator+(Cube a, Cube b) { return {a.q + b.q, a.r + b.r, a.s + b.s}; }
inline Cube operator-(Cube a, Cube b) { return {a.q - b.q, a.r - b.r, a.s - b.s}; }
inline Cube operator-(Cube a) { return {-a.q, -a.r, -a.s}; }

/// Steps from the origin: the largest absolute cube coordinate.
inline int length(Cube c) { return std::max({std::abs(c.q), std::abs(c.r), std::abs(c.s)}); }
inline int distance(Cube a, Cube b) { return length(a - b); }

// ---- directions and rotation ------------------------------------------------------------------------------------

constexpr int kDirections = 6;

/// Pointy layout on a y-down screen: E, NE, NW, W, SW, SE. Each is one 60 degree turn (counterclockwise) after the previous.
constexpr Cube kStep[kDirections] = {{1, 0, -1}, {1, -1, 0}, {0, -1, 1}, {-1, 0, 1}, {-1, 1, 0}, {0, 1, -1}};

inline int wrapIndex(int i) { return ((i % kDirections) + kDirections) % kDirections; }
inline Cube direction(int i) { return kStep[wrapIndex(i)]; }
inline int opposite(int d) { return wrapIndex(d + 3); }

/// Turn by 60 degrees counterclockwise on screen: (q, r, s) becomes (-s, -q, -r).
inline Cube rotateCcw(Cube c) { return {-c.s, -c.q, -c.r}; }

// ---- the map and its copies -------------------------------------------------------------------------------------

/// 1 + 6 + 12 + ... + 6N tiles.
inline int tileCount(int radius) { return 3 * radius * (radius + 1) + 1; }

/// Center of copy k (0..5) of the map: the first one is (2N+1, -N, -N-1), every next one is turned 60 degrees.
inline Cube mirrorCenter(int radius, int k) {
    Cube c{2 * radius + 1, -radius, -radius - 1};
    for (int i = 0; i < wrapIndex(k); ++i) c = rotateCcw(c);
    return c;
}

/// -1: the tile is in none of the seven maps (the origin map and its six neighbors), 0: on the map, 1..6: in the copy
/// around mirrorCenter(k - 1).
inline int copyIndex(Cube h, int radius) {
    if (length(h) <= radius) return 0;
    for (int k = 0; k < 6; ++k) {
        if (distance(h, mirrorCenter(radius, k)) <= radius) return k + 1;
    }
    return -1;
}

/// The tile on the map that a tile in one of the six neighbor copies stands for: subtract that copy's center.
/// A tile on the map stays as it is, and so does one that lies in none of the seven (the caller has to avoid those).
inline Cube wrap(Cube h, int radius) {
    const int copy = copyIndex(h, radius);
    return copy <= 0 ? h : h - mirrorCenter(radius, copy - 1);
}

/// wrap() for the one step that matters in a game: a tile just outside the map. The 6(N+1) tiles of that ring are
/// worked out once, a step is then an addition and, only when it leaves the map, one table lookup.
class WrapTable {
public:
    explicit WrapTable(int radius) : radius_(radius), side_(2 * radius + 3), to_(static_cast<std::size_t>(side_ * side_)) {
        for (int q = -radius - 1; q <= radius + 1; ++q) {
            for (int r = -radius - 1; r <= radius + 1; ++r) {
                const Cube c = make(q, r);
                if (length(c) == radius + 1) to_[slot(c)] = wrap(c, radius);
            }
        }
    }

    /// Destination of a tile in the ring just outside the map.
    Cube lookup(Cube ring) const { return to_[slot(ring)]; }

    /// One step from a tile on the map, wrapped. Precondition: `from` is on the map and `dir` is 0..5.
    Cube step(Cube from, int dir) const {
        const Cube next = from + direction(dir);
        return length(next) <= radius_ ? next : lookup(next);
    }

private:
    std::size_t slot(Cube c) const { return static_cast<std::size_t>((c.q + radius_ + 1) * side_ + (c.r + radius_ + 1)); }

    int radius_;
    int side_;
    std::vector<Cube> to_;
};

/// The tempting version: step off the map and appear on the tile opposite the one you left. Inside the map it is a
/// normal step.
inline Cube naiveStep(Cube from, int dir, int radius) {
    const Cube next = from + direction(dir);
    return length(next) <= radius ? next : -from;
}

/// How many different tiles the six neighbors of a tile are. `stepFn(tile, dir)` does one step.
template <class StepFn>
int distinctNeighbors(Cube tile, StepFn stepFn) {
    std::set<std::tuple<int, int, int>> seen;
    for (int d = 0; d < kDirections; ++d) {
        const Cube t = stepFn(tile, d);
        seen.insert({t.q, t.r, t.s});
    }
    return static_cast<int>(seen.size());
}

// ---- drawing ----------------------------------------------------------------------------------------------------

/// Center of a pointy hex, size 1: x = sqrt(3) (q + r / 2), y = 3/2 r.
inline Point center(Cube c) { return {kSqrt3 * (c.q + c.r / 2.0), 1.5 * c.r}; }

/// Corner i (0..5) relative to the center, size 1: corner 0 is up-right (30 degrees above east), then clockwise.
inline Point corner(int i) {
    const double angle = (60.0 * i - 30.0) * kPi / 180.0;
    return {std::cos(angle), std::sin(angle)};
}

/// First corner of the side that faces direction d (E, NE, NW, W, SW, SE): corners a and a + 1.
inline int sideCorner(int d) { return wrapIndex(6 - d); }

/// Is a point (relative to a hex center, size 1) inside that hex? Three pairs of parallel sides, each pair is a strip
/// of half width sqrt(3)/2 around the center. Pointy sides face 0, 60 and 120 degrees.
inline bool insideHex(Point p) {
    for (int k = 0; k < 3; ++k) {
        const double angle = 60.0 * k * kPi / 180.0;
        if (std::fabs(p.x * std::cos(angle) + p.y * std::sin(angle)) > kSqrt3 / 2.0) return false;
    }
    return true;
}

}  // namespace hexwrap
