// Tests for the terrain mesh maths in logic.hpp: the noise, the height field, the chunk meshes (topology, winding, skirts),
// the seams between chunks, the LOD cracks and how deep a skirt has to be, plus the camera. Hand-computed cases first,
// then properties over the whole world. The rendering itself needs a browser.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <set>
#include <vector>

#include "logic.hpp"

namespace {

int failures = 0;

#define CHECK(cond)                                                       \
    do {                                                                  \
        if (!(cond)) {                                                    \
            std::printf("FAILED %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            ++failures;                                                   \
        }                                                                 \
    } while (0)

bool near(double a, double b, double eps = 1e-4) { return std::fabs(a - b) <= eps; }
bool near(const tmesh::Vec3& a, const tmesh::Vec3& b, double eps = 1e-4) { return near(a.x, b.x, eps) && near(a.y, b.y, eps) && near(a.z, b.z, eps); }

constexpr double kPi = 3.14159265358979323846;

using tmesh::Coords;
using tmesh::Layout;
using tmesh::Vec3;

// ---------------------------------------------------------------------------------------------- helpers

Vec3 vertexPos(const tmesh::ChunkMesh& m, std::uint32_t v) {
    const std::size_t o = static_cast<std::size_t>(v) * tmesh::kVertexFloats;
    return {m.vertices[o], m.vertices[o + 1], m.vertices[o + 2]};
}
Vec3 vertexNormal(const tmesh::ChunkMesh& m, std::uint32_t v) {
    const std::size_t o = static_cast<std::size_t>(v) * tmesh::kVertexFloats;
    return {m.vertices[o + 3], m.vertices[o + 4], m.vertices[o + 5]};
}
float vertexFlag(const tmesh::ChunkMesh& m, std::uint32_t v) { return m.vertices[static_cast<std::size_t>(v) * tmesh::kVertexFloats + 6]; }

/// Vertex of the surface grid (column i along x, row j along z).
std::uint32_t surfaceIndex(const tmesh::ChunkMesh& m, int i, int j) { return static_cast<std::uint32_t>(j * (m.cells + 1) + i); }

Vec3 triangleNormal(const tmesh::ChunkMesh& m, std::size_t t) {
    const Vec3 a = vertexPos(m, m.indices[t * 3]);
    const Vec3 b = vertexPos(m, m.indices[t * 3 + 1]);
    const Vec3 c = vertexPos(m, m.indices[t * 3 + 2]);
    return tmesh::cross(b - a, c - a);
}

// ---------------------------------------------------------------------------------------------- noise

// Without a fade the value at a lattice point is the lattice value itself, so value noise must hit it exactly.
// In the middle of a cell the four corners weigh the same: (a + b + c + d) / 4.
void valueNoiseByHand() {
    for (int x = -3; x <= 3; ++x) {
        for (int z = -3; z <= 3; ++z) {
            const float v = tmesh::lattice(x, z, 7U);
            CHECK(v >= 0.0F && v < 1.0F);
            CHECK(near(tmesh::valueNoise(static_cast<float>(x), static_cast<float>(z), 7U), v, 1e-6));
        }
    }
    const float a = tmesh::lattice(2, -5, 3U), b = tmesh::lattice(3, -5, 3U), c = tmesh::lattice(2, -4, 3U), d = tmesh::lattice(3, -4, 3U);
    CHECK(near(tmesh::valueNoise(2.5F, -4.5F, 3U), (a + b + c + d) / 4.0F, 1e-6));
    // the same on the negative side of the origin (floor, not truncation)
    const float e = tmesh::lattice(-1, -1, 3U), f = tmesh::lattice(0, -1, 3U), g = tmesh::lattice(-1, 0, 3U), h = tmesh::lattice(0, 0, 3U);
    CHECK(near(tmesh::valueNoise(-0.5F, -0.5F, 3U), (e + f + g + h) / 4.0F, 1e-6));
}

void latticeIsDeterministicButNotConstant() {
    CHECK(tmesh::lattice(10, 20, 1U) == tmesh::lattice(10, 20, 1U));
    CHECK(tmesh::lattice(10, 20, 1U) != tmesh::lattice(10, 20, 2U));  // the seed matters
    CHECK(tmesh::lattice(10, 20, 1U) != tmesh::lattice(20, 10, 1U));  // x and z are not interchangeable
    std::set<float> seen;
    for (int i = 0; i < 100; ++i) seen.insert(tmesh::lattice(i, i * 7, 1U));
    CHECK(seen.size() > 95);  // 100 lattice points, essentially 100 different values
    double sum = 0.0;
    for (int i = 0; i < 4000; ++i) sum += tmesh::lattice(i % 64, i / 64, 5U);
    CHECK(near(sum / 4000.0, 0.5, 0.03));
}

// The quintic fade has slope 0 at both ends, so cell borders do not show as creases.
void fadeCurve() {
    CHECK(near(tmesh::fade(0.0F), 0.0));
    CHECK(near(tmesh::fade(1.0F), 1.0));
    CHECK(near(tmesh::fade(0.5F), 0.5));
    CHECK(near(tmesh::fade(0.25F), 0.103515625, 1e-6));  // 6/1024 - 15/256 + 10/64
    CHECK(tmesh::fade(0.001F) < 1e-7F);                  // flat start: 10 t^3
    CHECK(tmesh::fade(0.999F) > 1.0F - 1e-6F);
    float last = -1.0F;
    for (int i = 0; i <= 100; ++i) {
        const float v = tmesh::fade(static_cast<float>(i) / 100.0F);
        CHECK(v >= last);
        last = v;
    }
}

void valueNoiseIsContinuous() {
    for (int i = 0; i < 400; ++i) {
        const float x = -20.0F + static_cast<float>(i) * 0.1037F;
        const float z = 13.0F - static_cast<float>(i) * 0.0713F;
        CHECK(std::fabs(tmesh::valueNoise(x + 0.001F, z, 9U) - tmesh::valueNoise(x, z, 9U)) < 0.004F);
        CHECK(std::fabs(tmesh::valueNoise(x, z + 0.001F, 9U) - tmesh::valueNoise(x, z, 9U)) < 0.004F);
    }
}

// Junk coordinates must not crash or leave [0, 1]: the lattice lookup casts to int.
void noiseSurvivesJunk() {
    const float inf = std::numeric_limits<float>::infinity();
    for (const float v : {std::nanf(""), inf, -inf, 1.0e30F, -1.0e30F, 3.0e9F}) {
        const float n = tmesh::valueNoise(v, 5.0F, 7U);
        CHECK(n >= 0.0F && n <= 1.0F);
        const float m = tmesh::valueNoise(2.0F, v, 7U);
        CHECK(m >= 0.0F && m <= 1.0F);
        const float h = tmesh::heightAt(v, v);
        CHECK(h >= 0.0F && h <= tmesh::kHeightScale);
    }
    CHECK(tmesh::Layout{0}.chunkOf(10.0F) == 0);  // an invalid layout answers instead of dividing by zero
    CHECK(tmesh::Layout{32}.chunkOf(std::nanf("")) == 0);
}

void fbmAndHeightRange() {
    float lo = 1.0F, hi = 0.0F, hLo = 1e9F, hHi = -1e9F;
    for (int i = 0; i < 3000; ++i) {
        const float x = -300.0F + static_cast<float>(i % 60) * 10.0F;
        const float z = -300.0F + static_cast<float>(i / 60) * 12.5F;
        const float f = tmesh::fbm(x, z);
        CHECK(f >= 0.0F && f <= 1.0F);
        lo = std::min(lo, f);
        hi = std::max(hi, f);
        const float h = tmesh::heightAt(x, z);
        CHECK(h >= 0.0F && h <= tmesh::kHeightScale);
        hLo = std::min(hLo, h);
        hHi = std::max(hHi, h);
    }
    CHECK(hi - lo > 0.3F);              // the field is not flat
    CHECK(hHi > 0.5F * tmesh::kHeightScale);  // there are real hills
    CHECK(hLo < 0.1F * tmesh::kHeightScale);  // and real lowlands
}

// shape(): 0 up to kShapeLow, 1 from kShapeHigh on, in between t^1.5 with t the way from low to high.
void heightShaping() {
    const float lo = tmesh::kShapeLow, hi = tmesh::kShapeHigh;
    CHECK(near(tmesh::shape(0.0F), 0.0));
    CHECK(near(tmesh::shape(lo), 0.0));
    CHECK(near(tmesh::shape((lo + hi) / 2.0F), std::pow(0.5, 1.5), 1e-5));  // halfway: 0.353553
    CHECK(near(tmesh::shape(lo + 0.25F * (hi - lo)), 0.125, 1e-5));        // a quarter of the way: 0.25^1.5
    CHECK(near(tmesh::shape(hi), 1.0));
    CHECK(near(tmesh::shape(1.0F), 1.0));
    CHECK(near(tmesh::shape(-4.0F), 0.0));
    float last = -1.0F;
    for (int i = 0; i <= 100; ++i) {
        const float v = tmesh::shape(static_cast<float>(i) / 100.0F);
        CHECK(v >= last);
        last = v;
    }
}

// ---------------------------------------------------------------------------------------------- normals

// Central differences: n = normalize(hL - hR, 2 eps, hD - hU).
void normalByHand() {
    CHECK(near(tmesh::normalFromHeights(5.0F, 5.0F, 5.0F, 5.0F, 1.0F), Vec3{0.0F, 1.0F, 0.0F}));
    const float s = 1.0F / std::sqrt(2.0F);
    // the height grows by 1 per metre along +x: the surface leans towards -x
    CHECK(near(tmesh::normalFromHeights(-1.0F, 1.0F, 0.0F, 0.0F, 1.0F), Vec3{-s, s, 0.0F}));
    // along +z
    CHECK(near(tmesh::normalFromHeights(0.0F, 0.0F, -1.0F, 1.0F, 1.0F), Vec3{0.0F, s, -s}));
    // a bigger step between the samples makes the same height difference a gentler slope
    const Vec3 gentle = tmesh::normalFromHeights(-1.0F, 1.0F, 0.0F, 0.0F, 4.0F);
    CHECK(gentle.y > s);
    CHECK(near(tmesh::length(gentle), 1.0));
    // a cliff stays a unit vector
    CHECK(near(tmesh::length(tmesh::normalFromHeights(-900.0F, 900.0F, 40.0F, -40.0F, 2.0F)), 1.0));
}

// ---------------------------------------------------------------------------------------------- layout

void layoutNumbers() {
    CHECK(Layout{16}.chunksPerSide() == 16);
    CHECK(Layout{32}.chunksPerSide() == 8);
    CHECK(Layout{64}.chunksPerSide() == 4);
    CHECK(near(Layout{32}.chunkSize(), 64.0));
    CHECK(near(tmesh::kWorldSize, 512.0));
    CHECK(near(Layout{32}.origin(0), -256.0));
    CHECK(near(Layout{32}.origin(7) + Layout{32}.chunkSize(), 256.0));  // the chunks tile the world exactly
    CHECK(near(Layout{64}.origin(2), 0.0));

    CHECK(Layout{16}.valid() && Layout{32}.valid() && Layout{64}.valid());
    CHECK(!Layout{24}.valid());   // does not divide the world
    CHECK(!Layout{0}.valid());
    CHECK(!Layout{-32}.valid());
    CHECK(!Layout{8}.valid());    // LOD 2 would need 2 cells per chunk edge: too few for the demo
    CHECK(!Layout{512}.valid());

    // which chunk holds a world coordinate: the border belongs to the chunk on its right, outside clamps
    const Layout l{32};
    CHECK(l.chunkOf(-256.0F) == 0);
    CHECK(l.chunkOf(-192.1F) == 0);
    CHECK(l.chunkOf(-192.0F) == 1);
    CHECK(l.chunkOf(0.0F) == 4);
    CHECK(l.chunkOf(255.9F) == 7);
    CHECK(l.chunkOf(256.0F) == 7);
    CHECK(l.chunkOf(-9999.0F) == 0);
    CHECK(l.chunkOf(9999.0F) == 7);
}

void strides() {
    CHECK(tmesh::lodStride(0) == 1);
    CHECK(tmesh::lodStride(1) == 2);
    CHECK(tmesh::lodStride(2) == 4);
    CHECK(tmesh::lodStride(-1) == 1);  // out of range clamps
    CHECK(tmesh::lodStride(9) == 4);
}

// World mode: the height is the one global function. Local mode: the chunk position is forgotten.
void heightModes() {
    const Layout l{32};
    CHECK(tmesh::terrainHeight(l, Coords::World, 3, 5, 10.0F, 20.0F) == tmesh::heightAt(l.origin(3) + 10.0F, l.origin(5) + 20.0F));
    CHECK(tmesh::terrainHeight(l, Coords::Local, 3, 5, 10.0F, 20.0F) == tmesh::heightAt(10.0F, 20.0F));
    CHECK(tmesh::terrainHeight(l, Coords::Local, 0, 0, 10.0F, 20.0F) == tmesh::terrainHeight(l, Coords::Local, 7, 2, 10.0F, 20.0F));
    // in the world there is only one answer at a world point, whichever chunk asks
    CHECK(tmesh::terrainHeight(l, Coords::World, 0, 0, 64.0F, 10.0F) == tmesh::terrainHeight(l, Coords::World, 1, 0, 0.0F, 10.0F));
    // the height under a world point (camera clamp) uses the chunk that holds the point
    CHECK(tmesh::surfaceHeight(l, Coords::World, 12.5F, -40.0F) == tmesh::heightAt(12.5F, -40.0F));
    CHECK(tmesh::surfaceHeight(l, Coords::Local, -192.0F + 5.0F, -256.0F + 7.0F) == tmesh::heightAt(5.0F, 7.0F));
}

// ---------------------------------------------------------------------------------------------- mesh topology

void meshCounts() {
    // n = cells per edge at that LOD: 16 / 1, 16 / 2, 16 / 4 for a 16-cell chunk
    const struct {
        int lod, n;
    } cases[] = {{0, 16}, {1, 8}, {2, 4}};
    for (const auto& c : cases) {
        const tmesh::ChunkMesh plain = tmesh::buildChunk(Layout{16}, Coords::World, false, 3, 4, c.lod);
        CHECK(plain.cells == c.n);
        CHECK(plain.vertexCount() == (c.n + 1) * (c.n + 1));
        CHECK(plain.indices.size() == static_cast<std::size_t>(2 * c.n * c.n * 3));
        CHECK(plain.surfaceTriangles == 2 * c.n * c.n);

        const tmesh::ChunkMesh skirted = tmesh::buildChunk(Layout{16}, Coords::World, true, 3, 4, c.lod);
        CHECK(skirted.vertexCount() == (c.n + 1) * (c.n + 1) + 4 * (c.n + 1));
        CHECK(skirted.indices.size() == static_cast<std::size_t>((2 * c.n * c.n + 8 * c.n) * 3));
        CHECK(skirted.surfaceTriangles == 2 * c.n * c.n);
        CHECK(tmesh::triangleCount(Layout{16}, c.lod, true) == 2 * c.n * c.n + 8 * c.n);
        CHECK(tmesh::triangleCount(Layout{16}, c.lod, false) == 2 * c.n * c.n);
        for (const std::uint32_t i : skirted.indices) CHECK(i < static_cast<std::uint32_t>(skirted.vertexCount()));
        for (const std::uint32_t i : plain.indices) CHECK(i < static_cast<std::uint32_t>(plain.vertexCount()));
        CHECK(skirted.vertices.size() == static_cast<std::size_t>(skirted.vertexCount()) * tmesh::kVertexFloats);
    }
    // by hand: 16 cells -> 512 triangles + 128 skirt triangles; LOD 1 -> 128 + 64; LOD 2 -> 32 + 32
    CHECK(tmesh::triangleCount(Layout{16}, 0, true) == 640);
    CHECK(tmesh::triangleCount(Layout{16}, 1, true) == 192);
    CHECK(tmesh::triangleCount(Layout{16}, 2, true) == 64);
    // 16-bit-sized chunk: the biggest one is 65 x 65 + 4 x 65 vertices
    CHECK(tmesh::buildChunk(Layout{64}, Coords::World, true, 0, 0, 0).vertexCount() == 65 * 65 + 4 * 65);
}

void meshRejectsBadInput() {
    CHECK(tmesh::buildChunk(Layout{32}, Coords::World, true, -1, 0, 0).vertexCount() == 0);
    CHECK(tmesh::buildChunk(Layout{32}, Coords::World, true, 0, 8, 0).vertexCount() == 0);   // 8 chunks per side: 0..7
    CHECK(tmesh::buildChunk(Layout{32}, Coords::World, true, 0, 0, 3).vertexCount() == 0);   // only LOD 0..2
    CHECK(tmesh::buildChunk(Layout{32}, Coords::World, true, 0, 0, -1).vertexCount() == 0);
    CHECK(tmesh::buildChunk(Layout{24}, Coords::World, true, 0, 0, 0).vertexCount() == 0);  // invalid layout
    CHECK(tmesh::buildChunk(Layout{32}, Coords::World, true, 0, 0, 0).vertexCount() > 0);
    CHECK(tmesh::buildChunk(Layout{32}, Coords::World, true, 7, 7, 2).vertexCount() > 0);
    CHECK(tmesh::triangleCount(Layout{24}, 0, true) == 0);
    CHECK(tmesh::triangleCount(Layout{32}, 5, true) == 0);
}

void meshPositions() {
    const Layout l{32};
    const tmesh::ChunkMesh m = tmesh::buildChunk(l, Coords::World, true, 2, 5, 1);  // 16 cells of 4 m
    CHECK(m.cells == 16);
    // corner (0, 0) sits at the chunk origin, the last vertex at origin + chunk size
    const Vec3 first = vertexPos(m, surfaceIndex(m, 0, 0));
    const Vec3 last = vertexPos(m, surfaceIndex(m, 16, 16));
    CHECK(near(first.x, l.origin(2)) && near(first.z, l.origin(5)));
    CHECK(near(last.x, l.origin(2) + l.chunkSize()) && near(last.z, l.origin(5) + l.chunkSize()));
    // x grows with the column, z with the row, in steps of stride * cell size = 4 m
    CHECK(near(vertexPos(m, surfaceIndex(m, 3, 0)).x - first.x, 12.0));
    CHECK(near(vertexPos(m, surfaceIndex(m, 0, 3)).z - first.z, 12.0));
    // the height is the height function at that very point
    for (int j = 0; j <= 16; j += 4) {
        for (int i = 0; i <= 16; i += 4) {
            const Vec3 p = vertexPos(m, surfaceIndex(m, i, j));
            CHECK(p.y == tmesh::heightAt(p.x, p.z));
            CHECK(vertexFlag(m, surfaceIndex(m, i, j)) == 0.0F);
        }
    }
    // normals are unit vectors pointing up
    for (std::uint32_t v = 0; v < static_cast<std::uint32_t>(m.vertexCount()); ++v) {
        CHECK(near(tmesh::length(vertexNormal(m, v)), 1.0, 1e-4));
        CHECK(vertexNormal(m, v).y > 0.0F);
    }
}

void meshWinding() {
    for (const Coords mode : {Coords::World, Coords::Local}) {
        for (int lod = 0; lod < tmesh::kLodLevels; ++lod) {
            const tmesh::ChunkMesh m = tmesh::buildChunk(Layout{16}, mode, true, 5, 9, lod);
            const Vec3 centre{m.minX + (m.maxX - m.minX) / 2.0F, 0.0F, m.minZ + (m.maxZ - m.minZ) / 2.0F};
            for (std::size_t t = 0; t < m.indices.size() / 3; ++t) {
                const Vec3 n = triangleNormal(m, t);
                CHECK(tmesh::length(n) > 1e-6F);  // no degenerate triangle
                if (static_cast<int>(t) < m.surfaceTriangles) {
                    CHECK(n.y > 0.0F);  // the top surface faces up (a height field never folds over)
                } else {
                    // a skirt faces away from the chunk, and it is (nearly) vertical
                    const Vec3 a = vertexPos(m, m.indices[t * 3]);
                    const Vec3 b = vertexPos(m, m.indices[t * 3 + 1]);
                    const Vec3 c = vertexPos(m, m.indices[t * 3 + 2]);
                    const Vec3 mid = (a + b + c) * (1.0F / 3.0F);
                    const Vec3 out{mid.x - centre.x, 0.0F, mid.z - centre.z};
                    CHECK(tmesh::dot(n, out) > 0.0F);
                    CHECK(std::fabs(n.y) < 0.3F * tmesh::length(n));
                }
            }
        }
    }
}

// Skirt vertices repeat the edge vertex, hang kSkirtDepth below it, keep its normal and are flagged.
void skirtVertices() {
    const tmesh::ChunkMesh m = tmesh::buildChunk(Layout{16}, Coords::World, true, 1, 1, 0);
    const int n = m.cells;
    const std::uint32_t base = static_cast<std::uint32_t>((n + 1) * (n + 1));
    for (int side = 0; side < 4; ++side) {
        for (int k = 0; k <= n; ++k) {
            const std::uint32_t skirt = base + static_cast<std::uint32_t>(side * (n + 1) + k);
            std::uint32_t top = 0;
            if (side == 0) top = surfaceIndex(m, k, 0);
            if (side == 1) top = surfaceIndex(m, k, n);
            if (side == 2) top = surfaceIndex(m, 0, k);
            if (side == 3) top = surfaceIndex(m, n, k);
            const Vec3 a = vertexPos(m, top), b = vertexPos(m, skirt);
            CHECK(near(a.x, b.x) && near(a.z, b.z));
            CHECK(near(a.y - b.y, tmesh::kSkirtDepth));
            CHECK(near(vertexNormal(m, top), vertexNormal(m, skirt)));
            CHECK(vertexFlag(m, skirt) == 1.0F);
        }
    }
    // no skirt, no extra vertices and nothing flagged
    const tmesh::ChunkMesh plain = tmesh::buildChunk(Layout{16}, Coords::World, false, 1, 1, 0);
    for (std::uint32_t v = 0; v < static_cast<std::uint32_t>(plain.vertexCount()); ++v) CHECK(vertexFlag(plain, v) == 0.0F);
}

// Every grid edge once: n (n + 1) horizontal, n (n + 1) vertical, n n diagonal; a skirt side adds 3 n + 1.
void uniqueEdges() {
    for (int lod = 0; lod < tmesh::kLodLevels; ++lod) {
        const int n = 16 / tmesh::lodStride(lod);
        const tmesh::ChunkMesh plain = tmesh::buildChunk(Layout{16}, Coords::World, false, 0, 0, lod);
        const std::vector<std::uint32_t> e = tmesh::uniqueEdges(plain.indices);
        CHECK(e.size() == static_cast<std::size_t>((3 * n * n + 2 * n) * 2));
        const tmesh::ChunkMesh skirted = tmesh::buildChunk(Layout{16}, Coords::World, true, 0, 0, lod);
        CHECK(tmesh::uniqueEdges(skirted.indices).size() == static_cast<std::size_t>((3 * n * n + 2 * n + 4 * (3 * n + 1)) * 2));
    }
    // a single triangle has three edges, a shared edge counts once whatever the direction
    CHECK(tmesh::uniqueEdges({0, 1, 2}).size() == 6);
    CHECK(tmesh::uniqueEdges({0, 1, 2, 2, 1, 3}).size() == 10);
    CHECK(tmesh::uniqueEdges({}).empty());
}

// ---------------------------------------------------------------------------------------------- seams

// World coordinates: both chunks sample the same function at the same world point, so the border vertices are the same
// vertices, positions and normals, for neighbours in x and in z.
void worldCoordinatesHaveNoSeams() {
    const Layout l{16};
    const tmesh::ChunkMesh left = tmesh::buildChunk(l, Coords::World, false, 4, 6, 0);
    const tmesh::ChunkMesh right = tmesh::buildChunk(l, Coords::World, false, 5, 6, 0);
    const tmesh::ChunkMesh above = tmesh::buildChunk(l, Coords::World, false, 4, 5, 0);
    const int n = left.cells;
    for (int k = 0; k <= n; ++k) {
        CHECK(near(vertexPos(left, surfaceIndex(left, n, k)), vertexPos(right, surfaceIndex(right, 0, k)), 0.0));
        CHECK(near(vertexNormal(left, surfaceIndex(left, n, k)), vertexNormal(right, surfaceIndex(right, 0, k)), 0.0));
        CHECK(near(vertexPos(left, surfaceIndex(left, k, 0)), vertexPos(above, surfaceIndex(above, k, n)), 0.0));
        CHECK(near(vertexNormal(left, surfaceIndex(left, k, 0)), vertexNormal(above, surfaceIndex(above, k, n)), 0.0));
    }
}

// Local coordinates: every chunk is the same piece of terrain, copied, and the border does not match.
void localCoordinatesTileAndTear() {
    const Layout l{64};  // 128 m per chunk: the noise changes a lot over that distance
    const tmesh::ChunkMesh a = tmesh::buildChunk(l, Coords::Local, false, 2, 3, 0);
    const tmesh::ChunkMesh b = tmesh::buildChunk(l, Coords::Local, false, 0, 1, 0);
    const int n = a.cells;
    for (int j = 0; j <= n; ++j) {
        for (int i = 0; i <= n; ++i) CHECK(vertexPos(a, surfaceIndex(a, i, j)).y == vertexPos(b, surfaceIndex(b, i, j)).y);
    }
    // neighbours: the right edge of one is not the left edge of the next
    const tmesh::ChunkMesh right = tmesh::buildChunk(l, Coords::Local, false, 3, 3, 0);
    float worst = 0.0F;
    int torn = 0;
    for (int k = 0; k <= n; ++k) {
        const float d = std::fabs(vertexPos(a, surfaceIndex(a, n, k)).y - vertexPos(right, surfaceIndex(right, 0, k)).y);
        worst = std::max(worst, d);
        if (d > 0.5F) ++torn;
    }
    CHECK(worst > 5.0F);  // a visible step, metres high
    CHECK(torn > n / 2);
    // the positions still line up, only the heights disagree
    CHECK(near(vertexPos(a, surfaceIndex(a, n, 4)).x, vertexPos(right, surfaceIndex(right, 0, 4)).x));
}

// ---------------------------------------------------------------------------------------------- LOD

void lodChoice() {
    CHECK(tmesh::lodForDistance(0.0F, 100.0F) == 0);
    CHECK(tmesh::lodForDistance(99.9F, 100.0F) == 0);
    CHECK(tmesh::lodForDistance(100.0F, 100.0F) == 1);  // the threshold itself already belongs to the coarser level
    CHECK(tmesh::lodForDistance(199.9F, 100.0F) == 1);
    CHECK(tmesh::lodForDistance(200.0F, 100.0F) == 2);
    CHECK(tmesh::lodForDistance(1e9F, 100.0F) == 2);
    CHECK(tmesh::lodForDistance(-5.0F, 100.0F) == 0);
    CHECK(tmesh::lodForDistance(0.0F, 0.0F) == 0);  // a zero distance setting must not push the camera's own chunk to LOD 2
    CHECK(tmesh::lodForDistance(50.0F, 0.0F) == 2);
    CHECK(tmesh::lodForDistance(std::nanf(""), 100.0F) == 0);
}

void distanceToChunk() {
    const Layout l{32};  // chunk (0, 0) spans -256..-192 in x and z
    CHECK(near(tmesh::distanceToChunk(l, 0, 0, -220.0F, -250.0F), 0.0));        // inside
    CHECK(near(tmesh::distanceToChunk(l, 0, 0, -192.0F, -192.0F), 0.0));        // on the corner
    CHECK(near(tmesh::distanceToChunk(l, 0, 0, -182.0F, -220.0F), 10.0));       // 10 m beside the right edge
    CHECK(near(tmesh::distanceToChunk(l, 0, 0, -220.0F, -300.0F), 44.0));       // 44 m above the top edge
    CHECK(near(tmesh::distanceToChunk(l, 0, 0, -189.0F, -188.0F), 5.0));        // 3-4-5 from the corner
    CHECK(near(tmesh::distanceToChunk(l, 1, 1, -189.0F, -188.0F), 0.0));        // the neighbour that holds the point
}

void lodPlan() {
    const Layout l{32};
    const tmesh::LodPlan near1 = tmesh::planLods(l, 0.0F, 0.0F, 100.0F);
    CHECK(near1.lods.size() == 64);
    CHECK(near1.count[0] + near1.count[1] + near1.count[2] == 64);
    CHECK(near1.lods[static_cast<std::size_t>(l.chunkOf(0.0F) * 8 + l.chunkOf(0.0F))] == 0);  // under the camera
    CHECK(near1.count[0] > 0 && near1.count[1] > 0 && near1.count[2] > 0);

    // a longer LOD distance never makes any chunk coarser, and it never has fewer triangles to draw
    const tmesh::LodPlan far1 = tmesh::planLods(l, 0.0F, 0.0F, 300.0F);
    for (std::size_t i = 0; i < near1.lods.size(); ++i) CHECK(far1.lods[i] <= near1.lods[i]);
    CHECK(tmesh::planTriangles(l, far1, true) >= tmesh::planTriangles(l, near1, true));
    // the worst case is every chunk at LOD 0
    const long all = 64L * tmesh::triangleCount(l, 0, true);
    CHECK(tmesh::planTriangles(l, far1, true) <= all);
    CHECK(tmesh::planTriangles(l, near1, true) < all / 2);
    // camera far outside: everything is far away
    const tmesh::LodPlan outside = tmesh::planLods(l, 5000.0F, 5000.0F, 100.0F);
    CHECK(outside.count[2] == 64);
    // by hand: a 32-cell chunk at LOD 2 has 8 x 8 cells = 128 triangles + 4 skirt sides of 16 = 64
    CHECK(tmesh::planTriangles(l, outside, true) == 64L * 192L);
    CHECK(tmesh::planTriangles(l, outside, false) == 64L * 128L);
}

// The coarse chunk's edge only keeps every stride-th vertex of the fine edge, so those vertices are shared, the ones in
// between lie off the coarse line: that is the crack. Gap = vertical distance between fine vertex and coarse polyline.
float maxCrackGap(const Layout& l, int fineStride, int coarseStride) {
    float worst = 0.0F;
    const int cells = tmesh::kWorldCells;
    for (int border = 0; border <= l.chunksPerSide(); ++border) {
        const float x = tmesh::kWorldSize * -0.5F + static_cast<float>(border * l.chunkCells) * tmesh::kCellSize;
        for (int k = 0; k < cells; ++k) {
            const float z = tmesh::kWorldSize * -0.5F + static_cast<float>(k) * tmesh::kCellSize;
            const int seg = k / coarseStride;
            const float z0 = tmesh::kWorldSize * -0.5F + static_cast<float>(seg * coarseStride) * tmesh::kCellSize;
            const float z1 = z0 + static_cast<float>(coarseStride) * tmesh::kCellSize;
            const float t = (z - z0) / (z1 - z0);
            const float line = tmesh::heightAt(x, z0) + (tmesh::heightAt(x, z1) - tmesh::heightAt(x, z0)) * t;
            worst = std::max(worst, std::fabs(line - tmesh::heightAt(x, z)));
        }
        (void)fineStride;
    }
    return worst;
}

void lodCracksExistAndSkirtsCoverThem() {
    const Layout l{16};  // the most borders: the worst case of all layouts
    const float g1 = maxCrackGap(l, 1, 1);
    const float g2 = maxCrackGap(l, 1, 2);
    const float g4 = maxCrackGap(l, 1, 4);
    CHECK(g1 == 0.0F);          // same LOD on both sides: the edges are identical, no crack
    CHECK(g2 > 0.05F);          // LOD 1 next to LOD 0 leaves a gap
    CHECK(g4 > g2);             // and the coarser the neighbour, the wider
    CHECK(g4 < tmesh::kSkirtDepth);  // a skirt of kSkirtDepth reaches below the worst gap in this world
    CHECK(g4 > 0.2F * tmesh::kSkirtDepth);  // and it is not wildly oversized
    std::printf("crack gap  lod0|lod1: %.2f m   lod0|lod2: %.2f m   skirt depth %.1f m\n", static_cast<double>(g2), static_cast<double>(g4), static_cast<double>(tmesh::kSkirtDepth));

    // the shared vertices really are shared: every second vertex of a fine edge is a vertex of the coarse edge
    const Layout l16{16};
    const tmesh::ChunkMesh fine = tmesh::buildChunk(l16, Coords::World, false, 4, 4, 0);
    const tmesh::ChunkMesh coarse = tmesh::buildChunk(l16, Coords::World, false, 5, 4, 1);
    for (int k = 0; k <= coarse.cells; ++k) {
        CHECK(near(vertexPos(fine, surfaceIndex(fine, fine.cells, 2 * k)), vertexPos(coarse, surfaceIndex(coarse, 0, k)), 0.0));
    }
    // the normals at the shared vertices agree too, so the light does not jump at the LOD border either
    for (int k = 0; k <= coarse.cells; ++k) {
        CHECK(near(vertexNormal(fine, surfaceIndex(fine, fine.cells, 2 * k)), vertexNormal(coarse, surfaceIndex(coarse, 0, k)), 0.0));
    }
    // and an odd fine vertex is not on the coarse line
    const float mid = 0.5F * (vertexPos(coarse, surfaceIndex(coarse, 0, 3)).y + vertexPos(coarse, surfaceIndex(coarse, 0, 4)).y);
    CHECK(std::fabs(vertexPos(fine, surfaceIndex(fine, fine.cells, 7)).y - mid) > 1e-3F);
}

// ---------------------------------------------------------------------------------------------- camera

void cameraVectors() {
    tmesh::FlyCamera c;
    c.yaw = 0.0F;
    c.pitch = 0.0F;
    CHECK(near(c.forward(), Vec3{0.0F, 0.0F, -1.0F}));
    CHECK(near(c.right(), Vec3{1.0F, 0.0F, 0.0F}));
    c.yaw = static_cast<float>(kPi / 2.0);
    CHECK(near(c.forward(), Vec3{1.0F, 0.0F, 0.0F}));
    CHECK(near(c.right(), Vec3{0.0F, 0.0F, 1.0F}));
    c.pitch = static_cast<float>(kPi / 6.0);
    CHECK(near(c.forward().y, 0.5));
    CHECK(near(tmesh::length(c.forward()), 1.0));
    CHECK(near(c.right().y, 0.0));
}

void cameraMoves() {
    tmesh::FlyCamera c;
    c.pos = {10.0F, 20.0F, 30.0F};
    c.yaw = 0.0F;
    c.pitch = 0.0F;
    c.move(5.0F, 0.0F, 0.0F);
    CHECK(near(c.pos, Vec3{10.0F, 20.0F, 25.0F}));
    c.move(0.0F, 2.0F, 0.0F);
    CHECK(near(c.pos, Vec3{12.0F, 20.0F, 25.0F}));
    c.move(0.0F, 0.0F, 3.0F);
    CHECK(near(c.pos, Vec3{12.0F, 23.0F, 25.0F}));
    // looking up and moving forward climbs
    c.pitch = 0.5F;
    const float y = c.pos.y;
    c.move(10.0F, 0.0F, 0.0F);
    CHECK(c.pos.y > y);

    // pitch stops short of straight up and down (no flip), yaw may turn forever
    c.look(0.0F, 100.0F);
    CHECK(c.pitch < static_cast<float>(kPi / 2.0) && c.pitch > 1.4F);
    c.look(0.0F, -1000.0F);
    CHECK(c.pitch > -static_cast<float>(kPi / 2.0) && c.pitch < -1.4F);
    c.yaw = 0.0F;
    c.look(7.0F, 0.0F);
    CHECK(near(c.yaw, 7.0));
}

void cameraStaysAboveGroundAndInWorld() {
    tmesh::FlyCamera c;
    c.pos = {0.0F, 3.0F, 0.0F};
    tmesh::keepAbove(c, 50.0F, 4.0F);
    CHECK(near(c.pos.y, 54.0));      // pushed up to ground + clearance
    tmesh::keepAbove(c, 10.0F, 4.0F);
    CHECK(near(c.pos.y, 54.0));      // already higher: untouched
    c.pos = {-900.0F, 80.0F, 900.0F};
    tmesh::keepInside(c, 20.0F);
    CHECK(near(c.pos.x, -256.0 - 20.0) && near(c.pos.z, 256.0 + 20.0));
    c.pos = {5.0F, 80.0F, -7.0F};
    tmesh::keepInside(c, 20.0F);
    CHECK(near(c.pos, Vec3{5.0F, 80.0F, -7.0F}));
}

void matrices() {
    // lookAt puts the eye at the origin and the target on the -z axis
    const tmesh::Mat4 v = tmesh::lookAt({3.0F, 4.0F, 5.0F}, {3.0F, 4.0F, -5.0F}, {0.0F, 1.0F, 0.0F});
    const tmesh::Vec4 eye = v * tmesh::Vec4{3.0F, 4.0F, 5.0F, 1.0F};
    CHECK(near(eye.x, 0.0) && near(eye.y, 0.0) && near(eye.z, 0.0));
    const tmesh::Vec4 target = v * tmesh::Vec4{3.0F, 4.0F, -5.0F, 1.0F};
    CHECK(near(target.x, 0.0) && near(target.y, 0.0) && near(target.z, -10.0));

    // perspective: the near plane maps to depth -1, the far plane to +1 (after the divide)
    const tmesh::Mat4 p = tmesh::perspective(1.0F, 16.0F / 9.0F, 0.5F, 1500.0F);
    const tmesh::Vec4 n = p * tmesh::Vec4{0.0F, 0.0F, -0.5F, 1.0F};
    const tmesh::Vec4 f = p * tmesh::Vec4{0.0F, 0.0F, -1500.0F, 1.0F};
    CHECK(near(n.z / n.w, -1.0, 1e-4));
    CHECK(near(f.z / f.w, 1.0, 1e-4));
    // a point at the top edge of the view (tan(fov/2) above the axis) lands on y = +1
    const float top = std::tan(0.5F) * 10.0F;
    const tmesh::Vec4 t = p * tmesh::Vec4{0.0F, top, -10.0F, 1.0F};
    CHECK(near(t.y / t.w, 1.0, 1e-4));
}

// ---------------------------------------------------------------------------------------------- time and path

void pathClock() {
    tmesh::PathClock c;
    CHECK(near(c.advance(0.1, 1.0), 0.1));
    CHECK(near(c.angle, 0.1));
    // a stall is clamped, the path does not jump
    c.angle = 0.0;
    CHECK(near(c.advance(30.0, 1.0), 0.25));
    CHECK(near(c.angle, 0.25));
    // junk and zero add nothing
    c.angle = 1.0;
    CHECK(c.advance(0.0, 1.0) == 0.0);
    CHECK(c.advance(-1.0, 1.0) == 0.0);
    CHECK(c.advance(std::nan(""), 1.0) == 0.0);
    CHECK(near(c.angle, 1.0));
    // the angle wraps into [0, 2 pi) in double and stays precise after hours
    c.angle = 2.0 * kPi - 0.05;
    c.advance(0.1, 1.0);
    CHECK(near(c.angle, 0.05, 1e-9));
    for (int i = 0; i < 2000000; ++i) c.advance(0.016, 0.07);
    CHECK(c.angle >= 0.0 && c.angle < 2.0 * kPi);
    // 2,000,000 steps * 0.016 s * 0.07 rad/s = 2240 rad = 356.5 turns + 0.05 start -> compare with fmod of the exact sum
    CHECK(near(c.angle, std::fmod(0.05 + 2000000.0 * 0.016 * 0.07, 2.0 * kPi), 1e-4));
}

void pathPosition() {
    // angle 0: on the +x axis, looking at the middle, so the camera faces -x
    const tmesh::PathPose a = tmesh::pathPose(0.0, 150.0F);
    CHECK(near(a.x, 150.0) && near(a.z, 0.0));
    const tmesh::FlyCamera ca{{0.0F, 0.0F, 0.0F}, a.yaw, 0.0F};
    CHECK(near(ca.forward(), Vec3{-1.0F, 0.0F, 0.0F}, 1e-5));
    // a quarter turn later: on +z, facing -z
    const tmesh::PathPose b = tmesh::pathPose(kPi / 2.0, 150.0F);
    CHECK(near(b.x, 0.0, 1e-4) && near(b.z, 150.0, 1e-4));
    const tmesh::FlyCamera cb{{0.0F, 0.0F, 0.0F}, b.yaw, 0.0F};
    CHECK(near(cb.forward(), Vec3{0.0F, 0.0F, -1.0F}, 1e-5));
    // everywhere on the circle: at the radius, and the view direction points at the middle
    for (int i = 0; i < 100; ++i) {
        const tmesh::PathPose p = tmesh::pathPose(static_cast<double>(i) * 0.0628, 150.0F);
        CHECK(near(std::sqrt(p.x * p.x + p.z * p.z), 150.0, 1e-3));
        const tmesh::FlyCamera c{{p.x, 0.0F, p.z}, p.yaw, 0.0F};
        CHECK(near(tmesh::dot(c.forward(), Vec3{-p.x / 150.0F, 0.0F, -p.z / 150.0F}), 1.0, 1e-4));
    }
}

void smoothing() {
    CHECK(near(tmesh::smoothFactor(3.0F, 0.0F), 0.0));
    CHECK(near(tmesh::smoothFactor(3.0F, 1.0F), 1.0 - std::exp(-3.0), 1e-6));
    CHECK(tmesh::smoothFactor(3.0F, 1000.0F) <= 1.0F);
    CHECK(tmesh::smoothFactor(3.0F, -1.0F) == 0.0F);
    // two half steps give the same as one whole step: that is what makes it frame-rate independent
    const float half = tmesh::smoothFactor(2.0F, 0.05F);
    CHECK(near(1.0F - (1.0F - half) * (1.0F - half), tmesh::smoothFactor(2.0F, 0.1F), 1e-6));
}

}  // namespace

int main() {
    valueNoiseByHand();
    latticeIsDeterministicButNotConstant();
    fadeCurve();
    valueNoiseIsContinuous();
    noiseSurvivesJunk();
    fbmAndHeightRange();
    heightShaping();
    normalByHand();
    layoutNumbers();
    strides();
    heightModes();
    meshCounts();
    meshRejectsBadInput();
    meshPositions();
    meshWinding();
    skirtVertices();
    uniqueEdges();
    worldCoordinatesHaveNoSeams();
    localCoordinatesTileAndTear();
    lodChoice();
    distanceToChunk();
    lodPlan();
    lodCracksExistAndSkirtsCoverThem();
    cameraVectors();
    cameraMoves();
    cameraStaysAboveGroundAndInWorld();
    matrices();
    pathClock();
    pathPosition();
    smoothing();
    if (failures != 0) {
        std::printf("%d check(s) failed\n", failures);
        return EXIT_FAILURE;
    }
    std::printf("all checks passed\n");
    return EXIT_SUCCESS;
}
