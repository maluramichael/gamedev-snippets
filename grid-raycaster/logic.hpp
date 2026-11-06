// The logic of the grid raycaster, no SDL. Everything that can be wrong lives here and is covered by test.cpp.
//
// The world is a grid of square cells, each either wall or floor. One ray per screen column answers "which wall cell
// do I hit first, and how far away is it". Two ways to answer that are kept side by side:
//   castDda    jumps from grid line to grid line (Amanatides and Woo, the idea behind Wolfenstein 3D's inner loop).
//              It visits exactly the cells the ray passes through, so it can neither miss one nor stop short.
//   castMarch  walks along the ray in fixed steps and looks at the cell under each sample. Simple, and wrong in
//              two ways: a step that is too long jumps over a corner, and the hit is reported up to one step late.
//
// Coordinates: x grows to the right, y grows downwards, cell (cx, cy) covers x in [cx, cx+1) and y in [cy, cy+1).
// Everything outside the map counts as wall, so no ray can leave the world.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace grc {

constexpr double kPi = 3.14159265358979323846;
constexpr double kNearClip = 0.05;        // closer than this, a wall is drawn as if it were this far away
constexpr double kPlayerRadius = 0.25;    // cells
constexpr double kMinStep = 1e-4;        // smallest march step, so a bad value cannot loop for ever
constexpr long kMaxSamples = 2'000'000;   // hard limit of samples for one march
constexpr double kCoordLimit = 1e6;       // origins farther out than this are not worth a ray

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};

/// Which kind of grid line the ray crossed last. Vertical lines are x = const, so a Vertical hit means the ray ran
/// into a wall face that looks east or west.
enum class Side { Vertical, Horizontal };

class Map {
public:
    Map(int w, int h) : w_(std::max(w, 0)), h_(std::max(h, 0)), cells_(static_cast<std::size_t>(w_) * static_cast<std::size_t>(h_), 0) {}

    /// '#' is a wall, anything else floor. Short rows are padded with floor.
    static Map fromRows(const std::vector<std::string>& rows) {
        std::size_t w = 0;
        for (const std::string& r : rows) w = std::max(w, r.size());
        Map m(static_cast<int>(w), static_cast<int>(rows.size()));
        for (std::size_t y = 0; y < rows.size(); ++y) {
            for (std::size_t x = 0; x < rows[y].size(); ++x) m.set(static_cast<int>(x), static_cast<int>(y), rows[y][x] == '#');
        }
        return m;
    }

    int width() const { return w_; }
    int height() const { return h_; }
    bool contains(int x, int y) const { return x >= 0 && y >= 0 && x < w_ && y < h_; }
    bool solid(int x, int y) const { return !contains(x, y) || cells_[index(x, y)] != 0; }
    void set(int x, int y, bool wall) {
        if (contains(x, y)) cells_[index(x, y)] = wall ? 1 : 0;
    }
    void toggle(int x, int y) {
        if (contains(x, y)) cells_[index(x, y)] ^= 1;
    }

private:
    std::size_t index(int x, int y) const { return static_cast<std::size_t>(y) * static_cast<std::size_t>(w_) + static_cast<std::size_t>(x); }
    int w_;
    int h_;
    std::vector<unsigned char> cells_;
};

/// A cell the DDA entered, with the point where the ray crossed into it. For drawing the trail.
struct Visit {
    int cx = 0;
    int cy = 0;
    Vec2 enter;
};

struct Hit {
    bool found = false;
    int cx = 0;
    int cy = 0;
    double dist = 0.0;  // straight-line distance from the origin along the ray
    Side side = Side::Vertical;
    int steps = 0;      // cells entered (DDA) or samples taken (march)
};

namespace detail {

inline bool usable(Vec2 origin, Vec2 dir, double& len) {
    len = std::hypot(dir.x, dir.y);
    return len > 0.0 && std::isfinite(len) && std::isfinite(origin.x) && std::isfinite(origin.y) && std::fabs(origin.x) <= kCoordLimit &&
           std::fabs(origin.y) <= kCoordLimit;
}

inline int cellOf(double v) { return static_cast<int>(std::floor(v)); }

}  // namespace detail

/// Walks the grid cell by cell. `tMaxX`/`tMaxY` hold the distance along the ray to the next vertical/horizontal grid
/// line, `deltaX`/`deltaY` the distance between two such lines. The smaller one says which line is crossed next, so the
/// ray visits every cell it touches and nothing else. A crossing of both lines at the same moment (a ray through a
/// lattice point) steps in x first, which keeps every move 4-connected.
inline Hit castDda(const Map& map, Vec2 origin, Vec2 dir, std::vector<Visit>* visits = nullptr) {
    Hit hit;
    double len = 0.0;
    if (!detail::usable(origin, dir, len)) return hit;
    int cx = detail::cellOf(origin.x);
    int cy = detail::cellOf(origin.y);
    if (map.solid(cx, cy)) {
        hit.found = true;
        hit.cx = cx;
        hit.cy = cy;
        return hit;
    }
    const Vec2 d{dir.x / len, dir.y / len};
    constexpr double kInf = std::numeric_limits<double>::infinity();
    const int stepX = d.x < 0.0 ? -1 : 1;
    const int stepY = d.y < 0.0 ? -1 : 1;
    const double deltaX = d.x == 0.0 ? kInf : 1.0 / std::fabs(d.x);
    const double deltaY = d.y == 0.0 ? kInf : 1.0 / std::fabs(d.y);
    double tMaxX = d.x == 0.0 ? kInf : (d.x > 0.0 ? static_cast<double>(cx + 1) - origin.x : origin.x - static_cast<double>(cx)) * deltaX;
    double tMaxY = d.y == 0.0 ? kInf : (d.y > 0.0 ? static_cast<double>(cy + 1) - origin.y : origin.y - static_cast<double>(cy)) * deltaY;

    // A ray crosses at most width + height grid lines before it leaves the map, and outside everything is wall.
    const int limit = map.width() + map.height() + 4;
    for (int i = 0; i < limit; ++i) {
        double t = 0.0;
        if (tMaxX <= tMaxY) {
            cx += stepX;
            t = tMaxX;
            tMaxX += deltaX;
            hit.side = Side::Vertical;
        } else {
            cy += stepY;
            t = tMaxY;
            tMaxY += deltaY;
            hit.side = Side::Horizontal;
        }
        ++hit.steps;
        if (visits != nullptr) visits->push_back({cx, cy, {origin.x + d.x * t, origin.y + d.y * t}});
        if (map.solid(cx, cy)) {
            hit.found = true;
            hit.cx = cx;
            hit.cy = cy;
            hit.dist = t;
            return hit;
        }
    }
    return hit;
}

/// Samples the ray every `step` units and returns the first sample that lies in a wall. The sample positions are
/// k * step, not a running sum, so rounding errors do not pile up.
inline Hit castMarch(const Map& map, Vec2 origin, Vec2 dir, double step, double maxDist = 64.0, std::vector<Vec2>* samples = nullptr) {
    Hit hit;
    double len = 0.0;
    if (!detail::usable(origin, dir, len)) return hit;
    int prevX = detail::cellOf(origin.x);
    int prevY = detail::cellOf(origin.y);
    if (map.solid(prevX, prevY)) {
        hit.found = true;
        hit.cx = prevX;
        hit.cy = prevY;
        return hit;
    }
    if (!(step >= kMinStep)) step = kMinStep;
    const Vec2 d{dir.x / len, dir.y / len};
    for (long k = 1; k <= kMaxSamples; ++k) {
        const double t = static_cast<double>(k) * step;
        if (t > maxDist) break;
        const Vec2 p{origin.x + d.x * t, origin.y + d.y * t};
        const int cx = detail::cellOf(p.x);
        const int cy = detail::cellOf(p.y);
        ++hit.steps;
        if (samples != nullptr) samples->push_back(p);
        if (map.solid(cx, cy)) {
            hit.found = true;
            hit.cx = cx;
            hit.cy = cy;
            hit.dist = t;
            hit.side = cx != prevX ? Side::Vertical : Side::Horizontal;
            return hit;
        }
        prevX = cx;
        prevY = cy;
    }
    return hit;
}

struct Camera {
    Vec2 pos;
    double angle = 0.0;  // radians, 0 looks along +x, positive turns clockwise on screen (y points down)
    double fov = 1.4;    // horizontal field of view, radians
};

inline Vec2 viewDir(const Camera& cam) { return {std::cos(cam.angle), std::sin(cam.angle)}; }

inline double halfTan(double fov) { return std::tan(std::clamp(fov, 0.1, kPi - 0.1) / 2.0); }

/// Unit direction of the ray through the middle of screen column `column`. The columns are spaced evenly on a flat
/// projection plane in front of the eye, not by equal angles, which is what a flat screen needs. Positive camera x
/// is the right-hand side of the picture.
inline Vec2 columnRay(const Camera& cam, int column, int columns) {
    const int n = std::max(columns, 1);
    const double camX = 2.0 * (static_cast<double>(column) + 0.5) / static_cast<double>(n) - 1.0;
    const Vec2 dir = viewDir(cam);
    const double half = halfTan(cam.fov);
    const Vec2 ray{dir.x - dir.y * half * camX, dir.y + dir.x * half * camX};
    const double len = std::hypot(ray.x, ray.y);
    return {ray.x / len, ray.y / len};
}

/// The distance from the eye to the wall plane, measured along the view direction instead of along the ray.
/// Both vectors have length 1: dist * cos(angle between them).
inline double perpendicular(double dist, Vec2 ray, Vec2 view) { return dist * (ray.x * view.x + ray.y * view.y); }

/// Height in pixels of a wall `cells` high at depth `z`: the focal length (in pixels) times the height, divided by the
/// depth.
inline double wallHeight(double z, double viewWidth, double fov, double cells = 1.0) {
    return cells * viewWidth / (2.0 * halfTan(fov)) / std::max(z, kNearClip);
}

inline bool canStand(const Map& map, Vec2 p, double radius) {
    for (const double dx : {-radius, radius}) {
        for (const double dy : {-radius, radius}) {
            if (map.solid(detail::cellOf(p.x + dx), detail::cellOf(p.y + dy))) return false;
        }
    }
    return true;
}

/// Moves by `delta`, one axis at a time, so a body that runs into a wall slides along it.
inline Vec2 tryMove(const Map& map, Vec2 p, Vec2 delta, double radius) {
    if (canStand(map, {p.x + delta.x, p.y}, radius)) p.x += delta.x;
    if (canStand(map, {p.x, p.y + delta.y}, radius)) p.y += delta.y;
    return p;
}

inline double normalizeAngle(double a) {
    if (!std::isfinite(a)) return 0.0;
    a = std::fmod(a, 2.0 * kPi);
    return a < 0.0 ? a + 2.0 * kPi : a;
}

/// One screen column with the answer of both methods.
struct Column {
    Vec2 ray;
    Hit dda;
    Hit march;
};

struct Scan {
    std::vector<Column> columns;
    double ddaSteps = 0.0;    // average cells entered per ray
    double marchSteps = 0.0;  // average samples per ray
    int different = 0;        // rays where the march ends in another cell than the DDA (or finds nothing)
};

/// Fills `s` for a view `columns` wide, reusing its storage (the demo calls this every frame).
inline void scanInto(Scan& s, const Map& map, const Camera& cam, int columns, double step) {
    const int n = std::max(columns, 0);
    s.columns.resize(static_cast<std::size_t>(n));
    s.ddaSteps = 0.0;
    s.marchSteps = 0.0;
    s.different = 0;
    long ddaTotal = 0;
    long marchTotal = 0;
    for (int i = 0; i < n; ++i) {
        Column& c = s.columns[static_cast<std::size_t>(i)];
        c.ray = columnRay(cam, i, n);
        c.dda = castDda(map, cam.pos, c.ray);
        c.march = castMarch(map, cam.pos, c.ray, step);
        ddaTotal += c.dda.steps;
        marchTotal += c.march.steps;
        if (c.dda.found != c.march.found || (c.dda.found && (c.dda.cx != c.march.cx || c.dda.cy != c.march.cy))) ++s.different;
    }
    if (n > 0) {
        s.ddaSteps = static_cast<double>(ddaTotal) / n;
        s.marchSteps = static_cast<double>(marchTotal) / n;
    }
}

inline Scan scan(const Map& map, const Camera& cam, int columns, double step) {
    Scan s;
    scanInto(s, map, cam, columns, step);
    return s;
}

inline Map defaultMap() {
    return Map::fromRows({
        "####################",
        "#..................#",
        "#.......#..........#",
        "#.......#..........#",
        "#...#...#.....##...#",
        "#.......#.....##...#",
        "#.......#..........#",
        "#.......#..........#",
        "#.......#......#...#",
        "#.......#..#....#..#",
        "#...#...#........#.#",
        "#.................##",
        "#..................#",
        "#..................#",
        "####################",
    });
}

inline Vec2 startPosition() { return {3.5, 7.5}; }
inline double startAngle() { return 0.0; }

}  // namespace grc
