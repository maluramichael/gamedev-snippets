// Logic of the hex range and sight demo. No SDL, so test.cpp can check it on its own.
//
// Hexes are pointy-top and use axial coordinates (q, r). The map is a rectangle of `rows` rows and `cols` columns
// in which every other row sticks out half a hex ("odd-r"), stored by (row, column) but addressed by (q, r).
//
// Range: a loop over q and r (hexesInRange) knows nothing about walls. A flood fill (floodFill) spreads layer by
// layer from the unit and does not step into walls, so it only reaches what can be walked to.
//
// Sight: three answers to "can the unit see that hex?". hexLine walks the hexes of a straight line in cube
// coordinates, Center tests the exact segment from center to center against the wall hexagons, Corners also
// tries the six corners of the target. A wall only blocks when the line goes through its inside, touching a
// corner or sliding along an edge is not blocking.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace hx {

inline constexpr double kSqrt3 = 1.7320508075688772;

struct Hex {
    int q = 0;
    int r = 0;
    constexpr Hex() = default;
    constexpr Hex(int qq, int rr) : q(qq), r(rr) {}
};
constexpr bool operator==(Hex a, Hex b) { return a.q == b.q && a.r == b.r; }
constexpr bool operator!=(Hex a, Hex b) { return !(a == b); }

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};

/// The six neighbors. Index i matches kNormals[i].
inline constexpr std::array<Hex, 6> kDirections{{{1, 0}, {1, -1}, {0, -1}, {-1, 0}, {-1, 1}, {0, 1}}};

/// Unit vectors from a hex center to the middle of each of its six edges, same order as kDirections.
inline constexpr std::array<Vec2, 6> kNormals{{{1.0, 0.0}, {0.5, -0.8660254037844386}, {-0.5, -0.8660254037844386}, {-1.0, 0.0}, {-0.5, 0.8660254037844386}, {0.5, 0.8660254037844386}}};

/// Unit vectors from a hex center to its six corners (-30, 30, 90, 150, 210, 270 degrees, y down).
inline constexpr std::array<Vec2, 6> kCornerDirs{{{0.8660254037844386, -0.5}, {0.8660254037844386, 0.5}, {0.0, 1.0}, {-0.8660254037844386, 0.5}, {-0.8660254037844386, -0.5}, {0.0, -1.0}}};

constexpr int iabs(int v) { return v < 0 ? -v : v; }

constexpr Hex neighbor(Hex h, int dir) { return {h.q + kDirections[static_cast<std::size_t>(dir)].q, h.r + kDirections[static_cast<std::size_t>(dir)].r}; }

/// Number of steps between two hexes on an empty grid.
constexpr int distance(Hex a, Hex b) {
    const int dq = a.q - b.q;
    const int dr = a.r - b.r;
    return (iabs(dq) + iabs(dr) + iabs(dq + dr)) / 2;
}

// ---- pixels -----------------------------------------------------------------------------------------------------

struct Layout {
    double size = 20.0;  // distance from the center to a corner
    Vec2 origin{};       // pixel position of hex (0,0)
};

inline Vec2 toPixel(const Layout& l, Hex h) {
    return {l.origin.x + l.size * kSqrt3 * (h.q + h.r / 2.0), l.origin.y + l.size * 1.5 * h.r};
}

inline Vec2 corner(const Layout& l, Hex h, int i) {
    const Vec2 c = toPixel(l, h);
    const Vec2 d = kCornerDirs[static_cast<std::size_t>(i)];
    return {c.x + l.size * d.x, c.y + l.size * d.y};
}

/// Rounds fractional axial coordinates to the nearest hex: round all three cube axes, then fix the one that was
/// rounded the furthest so that q + r + s = 0 holds again. Anything that is not finite becomes (0,0), huge values
/// are clamped, so the cast to int is always defined.
inline Hex roundHex(double q, double r) {
    if (!std::isfinite(q) || !std::isfinite(r)) return {0, 0};
    constexpr double kLimit = 1e6;
    q = std::clamp(q, -kLimit, kLimit);
    r = std::clamp(r, -kLimit, kLimit);
    const double s = -q - r;
    double rq = std::round(q);
    double rr = std::round(r);
    const double rs = std::round(s);
    const double dq = std::fabs(rq - q);
    const double dr = std::fabs(rr - r);
    const double ds = std::fabs(rs - s);
    if (dq > dr && dq > ds) {
        rq = -rr - rs;
    } else if (dr > ds) {
        rr = -rq - rs;
    }
    return {static_cast<int>(rq), static_cast<int>(rr)};
}

inline Hex fromPixel(const Layout& l, Vec2 p) {
    const double x = (p.x - l.origin.x) / l.size;
    const double y = (p.y - l.origin.y) / l.size;
    return roundHex(kSqrt3 / 3.0 * x - y / 3.0, 2.0 / 3.0 * y);
}

// ---- the map ----------------------------------------------------------------------------------------------------

class Map {
public:
    Map(int cols, int rows) : cols_(cols), rows_(rows), wall_(static_cast<std::size_t>(cols) * static_cast<std::size_t>(rows), 0) {}

    int cols() const { return cols_; }
    int rows() const { return rows_; }
    std::size_t size() const { return wall_.size(); }

    bool contains(Hex h) const {
        if (h.r < 0 || h.r >= rows_) return false;
        const int col = h.q + (h.r >> 1);
        return col >= 0 && col < cols_;
    }

    /// Position in the flat storage. Only for hexes the map contains.
    std::size_t index(Hex h) const { return static_cast<std::size_t>(h.r) * static_cast<std::size_t>(cols_) + static_cast<std::size_t>(h.q + (h.r >> 1)); }

    /// The hex in a given row and column of the rectangle (0 <= row < rows, 0 <= col < cols).
    Hex hexAt(int row, int col) const { return {col - (row >> 1), row}; }
    Hex hexAtIndex(std::size_t i) const { return hexAt(static_cast<int>(i / static_cast<std::size_t>(cols_)), static_cast<int>(i % static_cast<std::size_t>(cols_))); }

    bool wall(Hex h) const { return contains(h) && wall_[index(h)] != 0; }
    void setWall(Hex h, bool on) {
        if (contains(h)) wall_[index(h)] = on ? 1 : 0;
    }
    void clearWalls() { std::fill(wall_.begin(), wall_.end(), static_cast<unsigned char>(0)); }

    std::vector<Hex> walls() const {
        std::vector<Hex> out;
        for (std::size_t i = 0; i < wall_.size(); ++i) {
            if (wall_[i] != 0) out.push_back(hexAtIndex(i));
        }
        return out;
    }

private:
    int cols_;
    int rows_;
    std::vector<unsigned char> wall_;
};

// ---- range ------------------------------------------------------------------------------------------------------

/// Every hex of the map within n steps, found by looping over q and r. Walls play no role here.
inline std::vector<Hex> hexesInRange(const Map& map, Hex center, int n) {
    std::vector<Hex> out;
    for (int dq = -n; dq <= n; ++dq) {
        for (int dr = std::max(-n, -dq - n); dr <= std::min(n, -dq + n); ++dr) {
            const Hex h{center.q + dq, center.r + dr};
            if (map.contains(h)) out.push_back(h);
        }
    }
    return out;
}

struct Reach {
    std::vector<int> cost;                 // per map index, -1 = not reached
    std::vector<std::vector<Hex>> layers;  // layers[k] = hexes first reached after k steps
};

/// Breadth first search by layers: every step costs one point, walls are never entered. An empty `layers` means
/// there was nothing to fill (start outside the map, on a wall, or a negative budget).
inline Reach floodFill(const Map& map, Hex start, int points) {
    Reach out;
    out.cost.assign(map.size(), -1);
    if (points < 0 || !map.contains(start) || map.wall(start)) return out;
    out.cost[map.index(start)] = 0;
    out.layers.push_back({start});
    for (int step = 1; step <= points; ++step) {
        std::vector<Hex> next;
        for (const Hex& h : out.layers.back()) {
            for (int d = 0; d < 6; ++d) {
                const Hex n = neighbor(h, d);
                if (map.contains(n) && !map.wall(n) && out.cost[map.index(n)] < 0) {
                    out.cost[map.index(n)] = step;
                    next.push_back(n);
                }
            }
        }
        if (next.empty()) break;
        out.layers.push_back(std::move(next));
    }
    return out;
}

// ---- sight ------------------------------------------------------------------------------------------------------

/// The hexes on the straight line from a to b, both included. Cube coordinates are interpolated and rounded. A step
/// that lands exactly between two hexes is a tie, the small constant nudge decides it the same way every time.
inline std::vector<Hex> hexLine(Hex a, Hex b) {
    const int n = distance(a, b);
    std::vector<Hex> out;
    if (n == 0) return {a};
    out.reserve(static_cast<std::size_t>(n) + 1);
    for (int i = 0; i <= n; ++i) {
        const double t = static_cast<double>(i) / n;
        out.push_back(roundHex(a.q + (b.q - a.q) * t + 1e-6, a.r + (b.r - a.r) * t + 2e-6));
    }
    return out;
}

/// Does the segment a-b go through the inside of the hexagon at h? Clips the segment against the six edge planes
/// (Cyrus-Beck). A segment that only touches a corner, ends on the edge or runs along an edge does not count.
inline bool segmentCrossesHex(const Layout& l, Hex h, Vec2 a, Vec2 b) {
    const Vec2 c = toPixel(l, h);
    const double apothem = l.size * kSqrt3 / 2.0;
    const double eps = 1e-9 * l.size;
    const Vec2 d{b.x - a.x, b.y - a.y};
    const double length = std::hypot(d.x, d.y);
    double tIn = 0.0;
    double tOut = 1.0;
    for (const Vec2& n : kNormals) {
        const double dist = n.x * (a.x - c.x) + n.y * (a.y - c.y) - apothem;  // below 0: a is inside this edge
        const double slope = n.x * d.x + n.y * d.y;                           // below 0: the segment moves inwards
        if (std::fabs(slope) <= 1e-9 * length) {
            if (dist > -eps) return false;  // parallel to this edge and not strictly inside
            continue;
        }
        const double t = -dist / slope;
        if (slope < 0.0) {
            tIn = std::max(tIn, t);
        } else {
            tOut = std::min(tOut, t);
        }
        if (tIn >= tOut) return false;
    }
    return tOut - tIn > 1e-9;
}

/// The first wall (in list order) the segment a-b goes through, ignoring the two hexes `from` and `to`.
inline std::optional<Hex> blocker(const Layout& l, const std::vector<Hex>& walls, Hex from, Hex to, Vec2 a, Vec2 b) {
    for (const Hex& w : walls) {
        if (w == from || w == to) continue;
        if (segmentCrossesHex(l, w, a, b)) return w;
    }
    return std::nullopt;
}

enum class Sight { HexLine, Center, Corners };

/// Can `from` see `to`? `walls` is the list of wall hexes. A wall that is the target is seen (its face), `from`
/// never blocks itself.
inline bool sees(Sight mode, const Map& map, const Layout& l, const std::vector<Hex>& walls, Hex from, Hex to) {
    if (from == to) return true;
    if (mode == Sight::HexLine) {
        const std::vector<Hex> line = hexLine(from, to);
        for (std::size_t i = 1; i + 1 < line.size(); ++i) {
            if (map.wall(line[i])) return false;
        }
        return true;
    }
    const Vec2 a = toPixel(l, from);
    if (!blocker(l, walls, from, to, a, toPixel(l, to))) return true;
    if (mode == Sight::Center) return false;
    for (int i = 0; i < 6; ++i) {
        if (!blocker(l, walls, from, to, a, corner(l, to, i))) return true;
    }
    return false;
}

enum class Seen : unsigned char { Outside, Hidden, Visible };

struct SightField {
    std::vector<Seen> seen;                  // per map index
    std::vector<unsigned char> differs;      // 1 where the answer is not the one of the center ray
};

/// Sight of every hex within `radius` steps of `from`, with the chosen method. `differs` compares with the center ray.
inline SightField computeSight(const Map& map, const Layout& l, Hex from, int radius, Sight mode) {
    SightField out;
    out.seen.assign(map.size(), Seen::Outside);
    out.differs.assign(map.size(), 0);
    if (!map.contains(from)) return out;
    const std::vector<Hex> walls = map.walls();
    for (const Hex& h : hexesInRange(map, from, radius)) {
        const bool visible = sees(mode, map, l, walls, from, h);
        out.seen[map.index(h)] = visible ? Seen::Visible : Seen::Hidden;
        if (mode != Sight::Center && visible != sees(Sight::Center, map, l, walls, from, h)) out.differs[map.index(h)] = 1;
    }
    return out;
}

// ---- start layout -----------------------------------------------------------------------------------------------

inline constexpr int kPresetCols = 19;
inline constexpr int kPresetRows = 13;

/// The layout the demo starts with: '#' is a wall, '@' the unit. A long wall with a gap at the bottom cuts off the hexes
/// behind it, a few single pillars and a short wall make shadows.
inline std::vector<std::string> presetRows() {
    return {
        "...................",
        ".......#...........",
        ".......#....#......",
        ".......#.....#.....",
        ".......#...........",
        ".......#...........",
        ".....@.#...........",
        ".......#...........",
        ".......#.....##....",
        ".......#...........",
        ".......#...........",
        "..###..............",
        "...................",
    };
}

/// Sets the walls from the rows (row i = map row i, character j = column j) and returns the position of '@'.
/// Rejects a layout of the wrong size or without a unit and then leaves the map as it was.
inline std::optional<Hex> loadLayout(Map& map, const std::vector<std::string>& rows) {
    if (static_cast<int>(rows.size()) != map.rows()) return std::nullopt;
    std::optional<Hex> unit;
    for (const std::string& row : rows) {
        if (static_cast<int>(row.size()) != map.cols()) return std::nullopt;
    }
    for (int r = 0; r < map.rows(); ++r) {
        for (int c = 0; c < map.cols(); ++c) {
            if (rows[static_cast<std::size_t>(r)][static_cast<std::size_t>(c)] == '@' && !unit) unit = map.hexAt(r, c);
        }
    }
    if (!unit) return std::nullopt;
    map.clearWalls();
    for (int r = 0; r < map.rows(); ++r) {
        for (int c = 0; c < map.cols(); ++c) map.setWall(map.hexAt(r, c), rows[static_cast<std::size_t>(r)][static_cast<std::size_t>(c)] == '#');
    }
    return unit;
}

}  // namespace hx
