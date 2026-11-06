// Autotiling logic without SDL: neighbour masks, the corner rule that turns 256 masks into 47 tiles, the 16-tile
// dual grid and the pieces (rectangles) a tile is drawn from. main.cpp only turns pieces into SDL rectangles, so
// the picture on screen is exactly the picture test.cpp checks.
//
// Mask layout, clockwise from north (a bit is set when that neighbour has the same material as the cell):
//   NW=128  N=1   NE=2
//    W=64  cell   E=4
//   SW=32   S=16  SE=8
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <initializer_list>
#include <string_view>
#include <vector>

namespace at {

inline constexpr int N = 1;
inline constexpr int NE = 2;
inline constexpr int E = 4;
inline constexpr int SE = 8;
inline constexpr int S = 16;
inline constexpr int SW = 32;
inline constexpr int W = 64;
inline constexpr int NW = 128;

/// FourBit: only N E S W, 16 tiles, no inner corners. EightBitRaw: all 8 neighbours, 256 tiles. EightBit47: the same
/// picture, but a corner only counts when both of its edge neighbours are set, which leaves 47 distinct tiles.
/// DualGrid: tiles sit on the grid vertices and look at the 4 cells around them, 16 tiles.
enum class Mode { FourBit, EightBitRaw, EightBit47, DualGrid };

/// Body is the filled inside, Edge the outline (drawn on top of Body).
enum class Kind : std::uint8_t { Body, Edge };

struct Piece {
    int x;
    int y;
    int w;
    int h;
    Kind kind;
};

struct Cell {
    int x;
    int y;
};

/// Material 0 is empty, anything else is solid. Reads outside the map give 0, writes outside are refused.
struct Grid {
    int w = 0;
    int h = 0;
    std::vector<std::uint8_t> cells;

    Grid(int width, int height) : w(width), h(height), cells(static_cast<std::size_t>(std::max(0, width) * std::max(0, height)), 0) {}

    bool inside(int x, int y) const { return x >= 0 && y >= 0 && x < w && y < h; }
    std::uint8_t at(int x, int y) const { return inside(x, y) ? cells[static_cast<std::size_t>(y * w + x)] : std::uint8_t{0}; }
    bool set(int x, int y, std::uint8_t material) {
        if (!inside(x, y)) return false;
        cells[static_cast<std::size_t>(y * w + x)] = material;
        return true;
    }
};

/// '#' wall (1), 'g' grass (2), anything else empty. Rows may differ in length, missing cells stay empty.
inline Grid parseMap(std::initializer_list<std::string_view> rows) {
    std::size_t width = 0;
    for (std::string_view r : rows) width = std::max(width, r.size());
    Grid g(static_cast<int>(width), static_cast<int>(rows.size()));
    int y = 0;
    for (std::string_view r : rows) {
        for (std::size_t x = 0; x < r.size(); ++x) {
            const std::uint8_t m = r[x] == '#' ? 1 : (r[x] == 'g' ? 2 : 0);
            g.set(static_cast<int>(x), y, m);
        }
        ++y;
    }
    return g;
}

// ---- masks and the corner rule -------------------------------------------------------------------------------

/// Clears every corner bit whose two edge neighbours are not both set. A diagonal neighbour can only change how the
/// tile looks when both edges next to it are filled (then it decides whether there is an inner corner).
inline int reduceCorners(int mask) {
    int out = mask & (N | E | S | W);
    if ((mask & NE) != 0 && (mask & N) != 0 && (mask & E) != 0) out |= NE;
    if ((mask & SE) != 0 && (mask & S) != 0 && (mask & E) != 0) out |= SE;
    if ((mask & SW) != 0 && (mask & S) != 0 && (mask & W) != 0) out |= SW;
    if ((mask & NW) != 0 && (mask & N) != 0 && (mask & W) != 0) out |= NW;
    return out;
}

namespace detail {
struct BlobTables {
    std::array<int, 256> indexOf{};  // reduced mask -> 0..46 (-1 for masks that are not reduced)
    std::array<int, 47> maskOf{};
    BlobTables() {
        indexOf.fill(-1);
        int next = 0;
        for (int m = 0; m < 256; ++m) {
            if (reduceCorners(m) == m) {
                indexOf[static_cast<std::size_t>(m)] = next;
                maskOf[static_cast<std::size_t>(next)] = m;
                ++next;
            }
        }
    }
};
inline const BlobTables& blobTables() {
    static const BlobTables tables;
    return tables;
}
}  // namespace detail

/// Tile number 0..46 of a mask (any raw mask, the corner rule is applied first). Numbered by ascending reduced mask.
inline int blobIndex(int mask) { return detail::blobTables().indexOf[static_cast<std::size_t>(reduceCorners(mask & 255))]; }

/// The reduced mask a tile number stands for, the inverse of blobIndex.
inline int blobMask(int index) { return detail::blobTables().maskOf[static_cast<std::size_t>(index)]; }

/// 4-bit number 0..15: N=1 E=2 S=4 W=8, corners ignored.
inline int fourBitIndex(int mask) {
    return ((mask & N) != 0 ? 1 : 0) | ((mask & E) != 0 ? 2 : 0) | ((mask & S) != 0 ? 4 : 0) | ((mask & W) != 0 ? 8 : 0);
}

/// Inner corners of a tile: both edge neighbours set but the diagonal one missing. The 4-bit mode cannot see these.
inline int innerCorners(int mask) {
    const auto corner = [mask](int edgeA, int edgeB, int diagonal) { return (mask & edgeA) != 0 && (mask & edgeB) != 0 && (mask & diagonal) == 0 ? 1 : 0; };
    return corner(N, E, NE) + corner(S, E, SE) + corner(S, W, SW) + corner(N, W, NW);
}

/// Neighbours of cell (x, y) that have the given material, as an 8-bit mask. Outside the map counts as empty.
inline int cellMask(const Grid& g, int x, int y, std::uint8_t material) {
    struct Step {
        int bit;
        int dx;
        int dy;
    };
    static constexpr std::array<Step, 8> kSteps{{{N, 0, -1}, {NE, 1, -1}, {E, 1, 0}, {SE, 1, 1}, {S, 0, 1}, {SW, -1, 1}, {W, -1, 0}, {NW, -1, -1}}};
    int mask = 0;
    for (const Step& s : kSteps) {
        if (g.at(x + s.dx, y + s.dy) == material) mask |= s.bit;
    }
    return mask;
}

// Dual grid: vertex (vx, vy) is the corner between cells (vx-1, vy-1) (vx, vy-1) (vx-1, vy) (vx, vy).
inline constexpr int kDualNW = 1;
inline constexpr int kDualNE = 2;
inline constexpr int kDualSE = 4;
inline constexpr int kDualSW = 8;

inline int dualMask(const Grid& g, int vx, int vy, std::uint8_t material) {
    return (g.at(vx - 1, vy - 1) == material ? kDualNW : 0) | (g.at(vx, vy - 1) == material ? kDualNE : 0) |
           (g.at(vx, vy) == material ? kDualSE : 0) | (g.at(vx - 1, vy) == material ? kDualSW : 0);
}

/// Inner corners in the whole map (what the 4-bit mode would draw wrong).
inline int mapInnerCorners(const Grid& g) {
    int n = 0;
    for (int y = 0; y < g.h; ++y) {
        for (int x = 0; x < g.w; ++x) {
            const std::uint8_t material = g.at(x, y);
            if (material != 0) n += innerCorners(cellMask(g, x, y, material));
        }
    }
    return n;
}

inline int tileCount(Mode mode) {
    switch (mode) {
        case Mode::FourBit: return 16;
        case Mode::EightBitRaw: return 256;
        case Mode::EightBit47: return 47;
        case Mode::DualGrid: return 16;
    }
    return 0;
}

/// The tile a cell with this 8-bit mask uses in a per-cell mode.
inline int cellTileIndex(Mode mode, int mask) {
    switch (mode) {
        case Mode::FourBit: return fourBitIndex(mask);
        case Mode::EightBitRaw: return mask & 255;
        case Mode::EightBit47: return blobIndex(mask);
        case Mode::DualGrid: return 0;
    }
    return 0;
}

// ---- drawing: a tile is a few rectangles -----------------------------------------------------------------------

/// Outline thickness for a tile of s pixels (s should be even).
inline int edgeWidth(int s) { return std::max(1, s / 6); }

namespace detail {

enum class Corner { NW, NE, SE, SW };

/// One quarter (q x q at ox, oy) of a filled cell. `corner` is the corner of the CELL this quarter touches. v and h are
/// the cell's vertical and horizontal neighbours on that side, d the diagonal one. A missing neighbour gets an outline
/// strip on that side, both neighbours present but the diagonal missing gets a small notch in the corner.
template <class F>
void quarter(Corner corner, int ox, int oy, int q, int border, bool v, bool h, bool d, F&& fn) {
    const bool top = corner == Corner::NW || corner == Corner::NE;
    const bool left = corner == Corner::NW || corner == Corner::SW;
    fn(Piece{ox, oy, q, q, Kind::Body});
    if (!v) fn(Piece{ox, top ? oy : oy + q - border, q, border, Kind::Edge});
    if (!h) fn(Piece{left ? ox : ox + q - border, oy, border, q, Kind::Edge});
    if (v && h && !d) fn(Piece{left ? ox : ox + q - border, top ? oy : oy + q - border, border, border, Kind::Edge});
}

/// The 8-bit mask a tile number of a per-cell mode stands for. FourBit sets all corners, so it never gets a notch.
inline int tileMask8(Mode mode, int index) {
    switch (mode) {
        case Mode::FourBit:
            return ((index & 1) != 0 ? N : 0) | ((index & 2) != 0 ? E : 0) | ((index & 4) != 0 ? S : 0) | ((index & 8) != 0 ? W : 0) | NE | SE | SW | NW;
        case Mode::EightBitRaw: return index & 255;
        case Mode::EightBit47: return blobMask(index);
        case Mode::DualGrid: return 0;
    }
    return 0;
}

}  // namespace detail

/// Calls fn(Piece) for every rectangle of one tile whose top-left pixel is (ox, oy), s pixels wide and high. Per-cell
/// tiles are the cell itself; a dual-grid tile is centred on a vertex and holds a quarter of each of the 4 cells.
template <class F>
void tilePieces(Mode mode, int index, int ox, int oy, int s, F&& fn) {
    using detail::Corner;
    const int q = s / 2;
    const int b = edgeWidth(s);
    if (mode == Mode::DualGrid) {
        const bool nw = (index & kDualNW) != 0;
        const bool ne = (index & kDualNE) != 0;
        const bool se = (index & kDualSE) != 0;
        const bool sw = (index & kDualSW) != 0;
        // The quarter at the tile's NW is the SE corner of the NW cell, and so on: the cell's corner points at the vertex.
        if (nw) detail::quarter(Corner::SE, ox, oy, q, b, sw, ne, se, fn);
        if (ne) detail::quarter(Corner::SW, ox + q, oy, q, b, se, nw, sw, fn);
        if (se) detail::quarter(Corner::NW, ox + q, oy + q, q, b, ne, sw, nw, fn);
        if (sw) detail::quarter(Corner::NE, ox, oy + q, q, b, nw, se, ne, fn);
        return;
    }
    const int m = detail::tileMask8(mode, index);
    detail::quarter(Corner::NW, ox, oy, q, b, (m & N) != 0, (m & W) != 0, (m & NW) != 0, fn);
    detail::quarter(Corner::NE, ox + q, oy, q, b, (m & N) != 0, (m & E) != 0, (m & NE) != 0, fn);
    detail::quarter(Corner::SE, ox + q, oy + q, q, b, (m & S) != 0, (m & E) != 0, (m & SE) != 0, fn);
    detail::quarter(Corner::SW, ox, oy + q, q, b, (m & S) != 0, (m & W) != 0, (m & SW) != 0, fn);
}

/// Calls fn(Piece, material) for the whole map with cells of s x s pixels, map origin at (0, 0). Each material is
/// tiled on its own: a neighbour of another material counts as empty.
template <class F>
void mapPieces(const Grid& g, Mode mode, int s, F&& fn) {
    if (mode == Mode::DualGrid) {
        const int q = s / 2;
        for (int vy = 0; vy <= g.h; ++vy) {
            for (int vx = 0; vx <= g.w; ++vx) {
                for (std::uint8_t material = 1; material <= 2; ++material) {
                    const int m = dualMask(g, vx, vy, material);
                    if (m == 0) continue;
                    tilePieces(mode, m, vx * s - q, vy * s - q, s, [&](const Piece& p) { fn(p, static_cast<int>(material)); });
                }
            }
        }
        return;
    }
    for (int y = 0; y < g.h; ++y) {
        for (int x = 0; x < g.w; ++x) {
            const std::uint8_t material = g.at(x, y);
            if (material == 0) continue;
            const int index = cellTileIndex(mode, cellMask(g, x, y, material));
            tilePieces(mode, index, x * s, y * s, s, [&](const Piece& p) { fn(p, static_cast<int>(material)); });
        }
    }
}

/// How often each tile number is used by the map (size tileCount(mode)). The empty dual tile 0 is never counted.
inline std::vector<int> tileUsage(const Grid& g, Mode mode) {
    std::vector<int> use(static_cast<std::size_t>(tileCount(mode)), 0);
    if (mode == Mode::DualGrid) {
        for (int vy = 0; vy <= g.h; ++vy) {
            for (int vx = 0; vx <= g.w; ++vx) {
                for (std::uint8_t material = 1; material <= 2; ++material) {
                    const int m = dualMask(g, vx, vy, material);
                    if (m != 0) ++use[static_cast<std::size_t>(m)];
                }
            }
        }
        return use;
    }
    for (int y = 0; y < g.h; ++y) {
        for (int x = 0; x < g.w; ++x) {
            const std::uint8_t material = g.at(x, y);
            if (material != 0) ++use[static_cast<std::size_t>(cellTileIndex(mode, cellMask(g, x, y, material)))];
        }
    }
    return use;
}

/// All cells on the line from a to b, both ends included (Bresenham), so a fast mouse stroke leaves no gaps.
inline std::vector<Cell> lineCells(Cell a, Cell b) {
    std::vector<Cell> out;
    const int dx = std::abs(b.x - a.x);
    const int dy = -std::abs(b.y - a.y);
    const int sx = a.x < b.x ? 1 : -1;
    const int sy = a.y < b.y ? 1 : -1;
    int err = dx + dy;
    int x = a.x;
    int y = a.y;
    for (;;) {
        out.push_back({x, y});
        if (x == b.x && y == b.y) break;
        const int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y += sy;
        }
    }
    return out;
}

}  // namespace at
