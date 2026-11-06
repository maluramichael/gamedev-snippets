// Hex grid coordinates without SDL: axial and cube, the four offset conventions, doubled coordinates, distance,
// neighbors, and the formulas that put a hex on the screen. Everything that can be wrong lives here and is checked by
// test.cpp. y points down (screen space), all pixel positions are for a hex of "size" 1 (center to corner).
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <optional>
#include <vector>

namespace hex {

enum class Orientation { Pointy, Flat };  // pointy: a corner points up, flat: a side is on top
enum class Parity { Odd, Even };          // which rows (pointy) or columns (flat) are pushed half a hex over

struct Layout {
    Orientation orientation;
    Parity parity;
};

struct Axial {
    int q, r;
};
struct Cube {
    int q, r, s;
};
struct Offset {
    int col, row;
};
struct Doubled {
    int col, row;
};
struct Point {
    double x, y;
};
struct Bounds {
    double minX, maxX, minY, maxY;
};
struct Board {
    int cols, rows;
};

constexpr double kSqrt3 = 1.7320508075688772;

// ---- axial <-> cube ---------------------------------------------------------------------------------------------

/// The three cube axes always add up to zero, so the third one carries no information: s = -q - r.
inline Cube toCube(Axial a) { return {a.q, a.r, -a.q - a.r}; }
inline Axial toAxial(Cube c) { return {c.q, c.r}; }

// ---- offset <-> axial -------------------------------------------------------------------------------------------
// "x & 1" is the parity bit, it also works for negative numbers (two's complement). The division is always of an even
// number, so it is exact.

inline int parityBit(int x) { return x & 1; }

inline Axial toAxial(Layout l, Offset o) {
    if (l.orientation == Orientation::Pointy) {
        const int shift = l.parity == Parity::Odd ? o.row - parityBit(o.row) : o.row + parityBit(o.row);
        return {o.col - shift / 2, o.row};
    }
    const int shift = l.parity == Parity::Odd ? o.col - parityBit(o.col) : o.col + parityBit(o.col);
    return {o.col, o.row - shift / 2};
}

inline Offset toOffset(Layout l, Axial a) {
    if (l.orientation == Orientation::Pointy) {
        const int shift = l.parity == Parity::Odd ? a.r - parityBit(a.r) : a.r + parityBit(a.r);
        return {a.q + shift / 2, a.r};
    }
    const int shift = l.parity == Parity::Odd ? a.q - parityBit(a.q) : a.q + parityBit(a.q);
    return {a.q, a.r + shift / 2};
}

// ---- doubled ----------------------------------------------------------------------------------------------------
// Pointy hexes double the column (every second column does not exist), flat hexes double the row.

inline Doubled toDoubled(Orientation o, Axial a) {
    return o == Orientation::Pointy ? Doubled{2 * a.q + a.r, a.r} : Doubled{a.q, 2 * a.r + a.q};
}

/// Only cells where column + row is even exist.
inline bool isDoubled(Orientation, Doubled d) { return ((d.col + d.row) & 1) == 0; }

inline Axial fromDoubled(Orientation o, Doubled d) {
    return o == Orientation::Pointy ? Axial{(d.col - d.row) / 2, d.row} : Axial{d.col, (d.row - d.col) / 2};
}

// ---- distance ---------------------------------------------------------------------------------------------------

/// Number of neighbor steps between two hexes: the largest absolute difference of the three cube axes.
inline int cubeDistance(Cube a, Cube b) { return std::max({std::abs(a.q - b.q), std::abs(a.r - b.r), std::abs(a.s - b.s)}); }

/// Same number straight from axial, s is rebuilt from the other two: |dq| + |dq + dr| + |dr|, halved.
inline int axialDistance(Axial a, Axial b) {
    const int dq = a.q - b.q;
    const int dr = a.r - b.r;
    return (std::abs(dq) + std::abs(dq + dr) + std::abs(dr)) / 2;
}

/// The tempting wrong answer: count columns and rows of the offset grid. It is off whenever a row is pushed over.
inline int naiveOffsetDistance(Offset a, Offset b) { return std::abs(a.col - b.col) + std::abs(a.row - b.row); }

// ---- neighbors --------------------------------------------------------------------------------------------------

constexpr int kDirections = 6;

/// Six steps, same order for both orientations. Pointy: E NE NW W SW SE. Flat: SE NE N NW SW S.
constexpr Axial kAxialDirections[kDirections] = {{1, 0}, {1, -1}, {0, -1}, {-1, 0}, {-1, 1}, {0, 1}};

inline int wrapDirection(int d) { return ((d % kDirections) + kDirections) % kDirections; }

inline Axial neighbor(Axial a, int direction) {
    const Axial step = kAxialDirections[wrapDirection(direction)];
    return {a.q + step.q, a.r + step.r};
}

/// Neighbor straight in offset coordinates. The step depends on whether the row (pointy) or column (flat) is one of the
/// pushed ones, so there are two tables. Written for odd-r and odd-q, even-r and even-q use the other table.
inline Offset neighborOffset(Layout l, Offset o, int direction) {
    // [pushed or not][direction] -> {dcol, drow}
    constexpr int pointy[2][kDirections][2] = {
        {{1, 0}, {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}, {0, 1}},  // row not pushed right
        {{1, 0}, {1, -1}, {0, -1}, {-1, 0}, {0, 1}, {1, 1}},    // row pushed right
    };
    constexpr int flat[2][kDirections][2] = {
        {{1, 0}, {1, -1}, {0, -1}, {-1, -1}, {-1, 0}, {0, 1}},  // column not pushed down
        {{1, 1}, {1, 0}, {0, -1}, {-1, 0}, {-1, 1}, {0, 1}},    // column pushed down
    };
    const int line = l.orientation == Orientation::Pointy ? parityBit(o.row) : parityBit(o.col);
    const int pushed = l.parity == Parity::Odd ? line : 1 - line;
    const int d = wrapDirection(direction);
    const int* step = l.orientation == Orientation::Pointy ? pointy[pushed][d] : flat[pushed][d];
    return {o.col + step[0], o.row + step[1]};
}

// ---- keys and screen directions ---------------------------------------------------------------------------------

enum class Screen { UpLeft, Up, UpRight, Left, Right, DownLeft, Down, DownRight };
enum class Arrow { Left, Right, Up, Down };

/// Axial direction index of the neighbor that lies in that screen direction, or -1 if this orientation has none
/// (a pointy hex has no neighbor straight up, a flat one none straight left).
inline int directionOnScreen(Orientation o, Screen s) {
    if (o == Orientation::Pointy) {
        switch (s) {
            case Screen::Right: return 0;
            case Screen::UpRight: return 1;
            case Screen::UpLeft: return 2;
            case Screen::Left: return 3;
            case Screen::DownLeft: return 4;
            case Screen::DownRight: return 5;
            default: return -1;
        }
    }
    switch (s) {
        case Screen::DownRight: return 0;
        case Screen::UpRight: return 1;
        case Screen::Up: return 2;
        case Screen::UpLeft: return 3;
        case Screen::DownLeft: return 4;
        case Screen::Down: return 5;
        default: return -1;
    }
}

/// An arrow key has to pick one of six neighbors. Where two neighbors share a side, the upper or right-hand one wins
/// and Shift takes the other.
inline Screen arrowToScreen(Orientation o, Arrow a, bool shift) {
    if (o == Orientation::Pointy) {
        switch (a) {
            case Arrow::Left: return Screen::Left;
            case Arrow::Right: return Screen::Right;
            case Arrow::Up: return shift ? Screen::UpLeft : Screen::UpRight;
            case Arrow::Down: return shift ? Screen::DownLeft : Screen::DownRight;
        }
    }
    switch (a) {
        case Arrow::Up: return Screen::Up;
        case Arrow::Down: return Screen::Down;
        case Arrow::Left: return shift ? Screen::DownLeft : Screen::UpLeft;
        case Arrow::Right: return shift ? Screen::DownRight : Screen::UpRight;
    }
    return Screen::Up;
}

// ---- drawing ----------------------------------------------------------------------------------------------------

/// Center of a hex on the screen, size 1. Pointy: x = sqrt(3) (q + r / 2), y = 3/2 r. Flat: x = 3/2 q, y = sqrt(3) (r + q / 2).
inline Point center(Orientation o, Axial a) {
    const auto q = static_cast<double>(a.q);
    const auto r = static_cast<double>(a.r);
    if (o == Orientation::Pointy) return {kSqrt3 * (q + r / 2.0), 1.5 * r};
    return {1.5 * q, kSqrt3 * (r + q / 2.0)};
}

/// Corner i (0..5) relative to the center, size 1. Pointy starts 30 degrees above the right (up-right), flat points right.
inline Point corner(Orientation o, int i) {
    const double degrees = 60.0 * static_cast<double>(i) - (o == Orientation::Pointy ? 30.0 : 0.0);
    const double angle = degrees * 3.14159265358979323846 / 180.0;
    return {std::cos(angle), std::sin(angle)};
}

// ---- board ------------------------------------------------------------------------------------------------------

/// The board is a rectangle in offset coordinates, so which hexes it contains depends on the convention.
inline bool inBoard(Layout l, Board b, Axial a) {
    const Offset o = toOffset(l, a);
    return o.col >= 0 && o.col < b.cols && o.row >= 0 && o.row < b.rows;
}

/// All cells row by row (row 0 first, then column 0 to the right).
inline std::vector<Axial> boardCells(Layout l, Board b) {
    std::vector<Axial> cells;
    for (int row = 0; row < b.rows; ++row) {
        for (int col = 0; col < b.cols; ++col) cells.push_back(toAxial(l, {col, row}));
    }
    return cells;
}

/// Pixel extent (size 1) of all cells including their outline, to center the board.
inline Bounds boardBounds(Layout l, const std::vector<Axial>& cells) {
    const double halfW = l.orientation == Orientation::Pointy ? kSqrt3 / 2.0 : 1.0;
    const double halfH = l.orientation == Orientation::Pointy ? 1.0 : kSqrt3 / 2.0;
    Bounds b{1e9, -1e9, 1e9, -1e9};
    for (const Axial& a : cells) {
        const Point c = center(l.orientation, a);
        b.minX = std::min(b.minX, c.x - halfW);
        b.maxX = std::max(b.maxX, c.x + halfW);
        b.minY = std::min(b.minY, c.y - halfH);
        b.maxY = std::max(b.maxY, c.y + halfH);
    }
    return b;
}

/// Is a point (relative to a hex center, size 1) inside that hex? Three pairs of parallel sides, each pair is a strip of
/// half width sqrt(3)/2 around the center. Pointy sides face 0, 60 and 120 degrees, flat sides 30, 90 and 150.
inline bool insideHex(Orientation o, Point p) {
    const double offset = o == Orientation::Pointy ? 0.0 : 30.0;
    for (int k = 0; k < 3; ++k) {
        const double angle = (offset + 60.0 * static_cast<double>(k)) * 3.14159265358979323846 / 180.0;
        if (std::fabs(p.x * std::cos(angle) + p.y * std::sin(angle)) > kSqrt3 / 2.0) return false;
    }
    return true;
}

/// Which cell is under the point? The cheap way: the nearest center, then check the point really lies in that hex.
/// A linear scan is fine for a few dozen cells, the next post replaces it with a formula.
inline std::optional<std::size_t> pickCell(Orientation o, const std::vector<Axial>& cells, Point p) {
    std::optional<std::size_t> best;
    double bestDistance = 1e300;
    for (std::size_t i = 0; i < cells.size(); ++i) {
        const Point c = center(o, cells[i]);
        const double d = (p.x - c.x) * (p.x - c.x) + (p.y - c.y) * (p.y - c.y);
        if (d < bestDistance) {
            bestDistance = d;
            best = i;
        }
    }
    if (!best) return std::nullopt;
    const Point c = center(o, cells[*best]);
    if (!insideHex(o, {p.x - c.x, p.y - c.y})) return std::nullopt;
    return best;
}

}  // namespace hex
