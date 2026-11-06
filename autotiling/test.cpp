// Tests for the autotiling logic. Plain asserts on hand-computed cases, no framework, no SDL.
// Bit layout of the 8-bit mask, clockwise from north: N=1 NE=2 E=4 SE=8 S=16 SW=32 W=64 NW=128.
#undef NDEBUG
#include <algorithm>
#include <array>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <set>
#include <vector>

#include "logic.hpp"

using namespace at;

namespace {

/// Per-pixel picture of a set of pieces: (material << 2) | (kind + 1), 0 = nothing. Later pieces overwrite earlier ones.
struct Raster {
    int w;
    int h;
    std::vector<int> px;
    Raster(int width, int height) : w(width), h(height), px(static_cast<std::size_t>(width * height), 0) {}
    void put(const Piece& p, int material) {
        assert(p.w > 0 && p.h > 0);
        assert(p.x >= 0 && p.y >= 0 && p.x + p.w <= w && p.y + p.h <= h);  // no piece may leave the picture
        for (int y = p.y; y < p.y + p.h; ++y) {
            for (int x = p.x; x < p.x + p.w; ++x) {
                px[static_cast<std::size_t>(y * w + x)] = (material << 2) | (static_cast<int>(p.kind) + 1);
            }
        }
    }
    int count(Kind k) const {
        int n = 0;
        for (int v : px) n += (v & 3) == static_cast<int>(k) + 1 ? 1 : 0;
        return n;
    }
};

Raster tileRaster(Mode mode, int index, int s) {
    Raster r(s, s);
    tilePieces(mode, index, 0, 0, s, [&](const Piece& p) { r.put(p, 1); });
    return r;
}

Raster mapRaster(const Grid& g, Mode mode, int s) {
    Raster r(g.w * s, g.h * s);
    mapPieces(g, mode, s, [&](const Piece& p, int material) { r.put(p, material); });
    return r;
}

void maskConstants() {
    assert(N == 1 && NE == 2 && E == 4 && SE == 8 && S == 16 && SW == 32 && W == 64 && NW == 128);
}

void cornerRule() {
    assert(reduceCorners(0) == 0);
    assert(reduceCorners(255) == 255);
    assert(reduceCorners(NE) == 0);                  // a corner alone counts for nothing
    assert(reduceCorners(N | NE) == N);              // E is missing
    assert(reduceCorners(N | E) == (N | E));
    assert(reduceCorners(N | E | NE) == (N | E | NE));  // both edges set: the corner stays
    assert(reduceCorners(E | SE) == E);
    assert(reduceCorners(E | SE | S) == (E | SE | S));
    assert(reduceCorners(NE | SE | SW | NW) == 0);   // four corners, no edge
    assert(reduceCorners(N | E | S | W | NE) == (N | E | S | W | NE));
    assert(reduceCorners(255 - SE) == 255 - SE);     // all edges set: every corner is meaningful
}

void fortySeven() {
    std::set<int> raw;
    std::set<int> reduced;
    std::array<int, 256> classSize{};
    for (int m = 0; m < 256; ++m) {
        raw.insert(m);
        reduced.insert(reduceCorners(m));
        ++classSize[static_cast<std::size_t>(reduceCorners(m))];
    }
    assert(raw.size() == 256);
    assert(reduced.size() == 47);  // the number the whole demo is about
    assert(classSize[0] == 16);    // no edge: the four corner bits are free, 2^4 raw masks collapse into one tile
    assert(classSize[255] == 1);   // every neighbour set: nothing to collapse
    assert(classSize[N | E] == 8);       // N and E set: only NE is meaningful, the other 3 corners are free (2^3)
    assert(classSize[N | E | NE] == 8);  // same with NE set
}

void indexTable() {
    std::set<int> seen;
    for (int m = 0; m < 256; ++m) {
        const int i = blobIndex(m);
        assert(i >= 0 && i < 47);
        seen.insert(i);
        assert(blobMask(i) == reduceCorners(m));  // index -> mask is the inverse
        assert(blobIndex(reduceCorners(m)) == i);
    }
    assert(seen.size() == 47);
    assert(blobIndex(0) == 0);
    assert(blobIndex(NE) == 0);   // a lone corner is the isolated tile
    assert(blobIndex(255) == 46); // largest reduced mask gets the last index
    assert(blobIndex(N) == 1);    // numbering follows the reduced mask value: 0, 1, 4, 5, 7, ...
    assert(blobIndex(E) == 2);
    assert(blobIndex(N | E) == 3);
    assert(blobIndex(N | E | NE) == 4);
}

void neighbourMask() {
    // Every raw mask must survive a round trip through a 3x3 map.
    const Cell offset[8] = {{1, 0}, {2, 0}, {2, 1}, {2, 2}, {1, 2}, {0, 2}, {0, 1}, {0, 0}};  // N NE E SE S SW W NW
    for (int m = 0; m < 256; ++m) {
        Grid g(3, 3);
        g.set(1, 1, 1);
        for (int b = 0; b < 8; ++b) {
            if (m & (1 << b)) g.set(offset[b].x, offset[b].y, 1);
        }
        assert(cellMask(g, 1, 1, 1) == m);
    }
    // Other materials and the outside of the map never count.
    Grid g(3, 3);
    g.set(1, 1, 1);
    g.set(1, 0, 2);  // north is grass, not wall
    g.set(1, 2, 1);  // south is wall
    assert(cellMask(g, 1, 1, 1) == S);
    Grid one(1, 1);
    one.set(0, 0, 1);
    assert(cellMask(one, 0, 0, 1) == 0);
    Grid two(2, 2);
    for (int y = 0; y < 2; ++y) for (int x = 0; x < 2; ++x) two.set(x, y, 1);
    assert(cellMask(two, 0, 0, 1) == (E | SE | S));  // top-left cell of a 2x2 block
    assert(cellMask(two, 1, 1, 1) == (N | NW | W));  // bottom-right cell
}

void fourBit() {
    assert(fourBitIndex(0) == 0);
    assert(fourBitIndex(N) == 1);
    assert(fourBitIndex(E) == 2);
    assert(fourBitIndex(S) == 4);
    assert(fourBitIndex(W) == 8);
    assert(fourBitIndex(N | E | S | W) == 15);
    assert(fourBitIndex(255) == 15);  // corners are ignored
    assert(fourBitIndex(NE | SE | SW | NW) == 0);
    std::set<int> seen;
    for (int m = 0; m < 256; ++m) seen.insert(fourBitIndex(m));
    assert(seen.size() == 16);
    assert(tileCount(Mode::FourBit) == 16);
    assert(tileCount(Mode::EightBitRaw) == 256);
    assert(tileCount(Mode::EightBit47) == 47);
    assert(tileCount(Mode::DualGrid) == 16);
}

void dualMasks() {
    // Bits: NW=1, NE=2, SE=4, SW=8 (the four cells around a grid vertex).
    Grid g(2, 2);
    g.set(0, 0, 1);
    assert(dualMask(g, 1, 1, 1) == 1);  // cell (0,0) is the NW cell of vertex (1,1)
    assert(dualMask(g, 0, 0, 1) == 4);  // ... the SE cell of vertex (0,0)
    assert(dualMask(g, 1, 0, 1) == 8);  // ... the SW cell of vertex (1,0)
    assert(dualMask(g, 0, 1, 1) == 2);  // ... the NE cell of vertex (0,1)
    assert(dualMask(g, 2, 2, 1) == 0);
    assert(dualMask(g, 1, 1, 2) == 0);  // another material does not count
    for (int combo = 0; combo < 16; ++combo) {
        Grid q(2, 2);
        if (combo & 1) q.set(0, 0, 1);
        if (combo & 2) q.set(1, 0, 1);
        if (combo & 4) q.set(1, 1, 1);
        if (combo & 8) q.set(0, 1, 1);
        assert(dualMask(q, 1, 1, 1) == combo);  // 16 fillings, 16 distinct tiles
    }
    Grid full(2, 2);
    for (int y = 0; y < 2; ++y) for (int x = 0; x < 2; ++x) full.set(x, y, 1);
    assert(dualMask(full, 1, 1, 1) == 15);
    assert(dualMask(full, 0, 0, 1) == 4);
}

void singleTilePixels() {
    // s = 12: quadrant 6, border 2.
    assert(edgeWidth(12) == 2);
    {
        const Raster r = tileRaster(Mode::EightBit47, blobIndex(0), 12);  // isolated: a 2 px frame around 8x8 body
        assert(r.count(Kind::Edge) == 144 - 8 * 8);
        assert(r.count(Kind::Body) == 64);
    }
    {
        const Raster r = tileRaster(Mode::EightBit47, blobIndex(255), 12);  // surrounded: all body
        assert(r.count(Kind::Edge) == 0);
        assert(r.count(Kind::Body) == 144);
    }
    {
        const Raster r = tileRaster(Mode::EightBit47, blobIndex(255 - SE), 12);  // inner corner: one 2x2 notch
        assert(r.count(Kind::Edge) == 4);
        assert(r.count(Kind::Body) == 140);
    }
    {
        const Raster r = tileRaster(Mode::EightBit47, blobIndex(N | E | S | W), 12);  // four notches
        assert(r.count(Kind::Edge) == 16);
    }
    {
        const Raster r = tileRaster(Mode::FourBit, 15, 12);  // 4-bit never draws a notch
        assert(r.count(Kind::Edge) == 0);
    }
    {
        const Raster r = tileRaster(Mode::EightBitRaw, 255 - SE, 12);  // raw index 247 looks like the 47 tile
        assert(r.count(Kind::Edge) == 4);
    }
    {
        const Raster r = tileRaster(Mode::DualGrid, 15, 12);  // four set cells around the vertex: solid
        assert(r.count(Kind::Edge) == 0);
        assert(r.count(Kind::Body) == 144);
    }
    {
        const Raster r = tileRaster(Mode::DualGrid, 0, 12);
        assert(r.count(Kind::Body) == 0 && r.count(Kind::Edge) == 0);
    }
    {
        // Only the NW cell: a single 6x6 quadrant, edges toward the other three cells (right and bottom sides, 2 px each).
        const Raster r = tileRaster(Mode::DualGrid, 1, 12);
        assert(r.count(Kind::Body) + r.count(Kind::Edge) == 36);
        assert(r.count(Kind::Edge) == 2 * 6 + 2 * 6 - 2 * 2);
    }
}

void sameMapEveryWay() {
    // The 47-tile, the raw 256-tile and the 16-tile dual grid render must agree pixel by pixel. Random maps, 3 materials.
    std::uint32_t seed = 12345;
    auto next = [&seed]() {
        seed = seed * 1664525U + 1013904223U;
        return (seed >> 16) & 0xFFU;
    };
    for (int round = 0; round < 20; ++round) {
        Grid g(7, 6);
        for (int y = 0; y < g.h; ++y) {
            for (int x = 0; x < g.w; ++x) {
                const unsigned v = next() % 5;
                g.set(x, y, static_cast<std::uint8_t>(v < 2 ? 0 : (v < 4 ? 1 : 2)));
            }
        }
        const Raster a = mapRaster(g, Mode::EightBit47, 12);
        const Raster b = mapRaster(g, Mode::EightBitRaw, 12);
        const Raster c = mapRaster(g, Mode::DualGrid, 12);
        assert(a.px == b.px);
        assert(a.px == c.px);
    }
}

void fourBitLosesTheInnerCorner() {
    // L shape: (0,0) (1,0) (0,1) set, (1,1) empty. Cell (0,0) has E and S but not SE: one notch of 2x2 px.
    const Grid g = parseMap({"##", "#."});
    assert(g.at(0, 0) == 1 && g.at(1, 1) == 0);
    const Raster full = mapRaster(g, Mode::EightBit47, 12);
    const Raster four = mapRaster(g, Mode::FourBit, 12);
    int diff = 0;
    for (std::size_t i = 0; i < full.px.size(); ++i) diff += full.px[i] != four.px[i] ? 1 : 0;
    assert(diff == 4);
    assert(full.count(Kind::Edge) == four.count(Kind::Edge) + 4);
}

void innerCornerCount() {
    assert(innerCorners(0) == 0);
    assert(innerCorners(255) == 0);
    assert(innerCorners(255 - SE) == 1);
    assert(innerCorners(N | E) == 1);           // N and E set, NE missing
    assert(innerCorners(N | NE) == 0);          // E is missing: that is an outline, not an inner corner
    assert(innerCorners(N | E | S | W) == 4);
    assert(innerCorners(N | E | S | W | NE | SW) == 2);
    assert(mapInnerCorners(parseMap({"##", "#."})) == 1);
    assert(mapInnerCorners(parseMap({"##", "##"})) == 0);
    assert(mapInnerCorners(parseMap({"###", "#.#", "###"})) == 4);  // a ring has 4 inner corners, one per corner of the hole
}

void usage() {
    const Grid g = parseMap({"##", "#."});
    // 47 tiles: (0,0) mask E|S -> 20, (1,0) mask W only -> 64 (SW is cut by the corner rule), (0,1) mask N only -> 1 (NE too).
    const std::vector<int> u47 = tileUsage(g, Mode::EightBit47);
    assert(static_cast<int>(u47.size()) == 47);
    assert(u47[static_cast<std::size_t>(blobIndex(E | S))] == 1);
    assert(u47[static_cast<std::size_t>(blobIndex(W))] == 1);
    assert(u47[static_cast<std::size_t>(blobIndex(N))] == 1);
    assert(std::count_if(u47.begin(), u47.end(), [](int c) { return c > 0; }) == 3);
    // Raw: the same cells keep their raw masks: E|S = 20, W|SW = 96, N|NE = 3.
    const std::vector<int> raw = tileUsage(g, Mode::EightBitRaw);
    assert(static_cast<int>(raw.size()) == 256);
    assert(raw[E | S] == 1 && raw[W | SW] == 1 && raw[N | NE] == 1);
    // Dual: vertices (0,0)=4 (1,0)=12 (2,0)=8 (0,1)=6 (1,1)=11 (2,1)=1 (0,2)=2 (1,2)=1, tile 1 twice, empty tile 0 not counted.
    const std::vector<int> dual = tileUsage(g, Mode::DualGrid);
    assert(static_cast<int>(dual.size()) == 16);
    assert(dual[0] == 0);
    assert(dual[1] == 2);
    assert(dual[4] == 1 && dual[12] == 1 && dual[8] == 1 && dual[6] == 1 && dual[11] == 1 && dual[2] == 1);
    assert(std::count_if(dual.begin(), dual.end(), [](int c) { return c > 0; }) == 7);
}

void parsing() {
    const Grid g = parseMap({"#g.", ".#g"});
    assert(g.w == 3 && g.h == 2);
    assert(g.at(0, 0) == 1 && g.at(1, 0) == 2 && g.at(2, 0) == 0);
    assert(g.at(0, 1) == 0 && g.at(1, 1) == 1 && g.at(2, 1) == 2);
    assert(g.at(-1, 0) == 0 && g.at(3, 0) == 0 && g.at(0, 2) == 0);  // outside reads as empty
    Grid h(2, 2);
    assert(!h.set(2, 0, 1) && !h.set(0, -1, 1));  // writes outside are refused
    assert(h.set(1, 1, 2) && h.at(1, 1) == 2);
}

void lines() {
    const std::vector<Cell> a = lineCells({0, 0}, {3, 1});
    assert(a.size() == 4);
    assert(a[0].x == 0 && a[0].y == 0);
    assert(a[1].x == 1 && a[1].y == 0);
    assert(a[2].x == 2 && a[2].y == 1);
    assert(a[3].x == 3 && a[3].y == 1);
    const std::vector<Cell> single = lineCells({4, 4}, {4, 4});
    assert(single.size() == 1 && single[0].x == 4 && single[0].y == 4);
    const std::vector<Cell> up = lineCells({2, 5}, {2, 2});
    assert(up.size() == 4 && up[0].y == 5 && up[3].y == 2);
    const std::vector<Cell> back = lineCells({3, 1}, {0, 0});
    assert(back.size() == 4 && back[0].x == 3 && back[3].x == 0);
}

}  // namespace

int main() {
    maskConstants();
    cornerRule();
    fortySeven();
    indexTable();
    neighbourMask();
    fourBit();
    dualMasks();
    singleTilePixels();
    sameMapEveryWay();
    fourBitLosesTheInnerCorner();
    innerCornerCount();
    usage();
    parsing();
    lines();
    std::puts("autotiling logic: all tests passed");
    return 0;
}
