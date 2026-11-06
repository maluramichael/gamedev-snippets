// Chunked terrain without SDL and GL, so test.cpp can check it. main.cpp only uploads what this file builds.
//
// The terrain is one height function over the whole world, h(x, z), made of seven octaves of value noise. The world is
// cut into square chunks of 16, 32 or 64 cells (a cell is 2 m). Each chunk becomes a grid mesh. Three things decide
// whether the chunks fit together:
//
//   1. Coordinates. A chunk has to ask h for the WORLD position of its vertices. Ask with chunk-local coordinates and every
//      chunk is the same hill, copied, with a step at every border (Coords::Local is that mistake, kept to be shown).
//   2. Normals. They come from h itself, sampled one finest cell to each side, even beyond the chunk. A chunk that only
//      looked at its own vertices could not light its border correctly. Both neighbours get the identical vector.
//   3. LOD. A far chunk keeps every 2nd or 4th vertex (stride 1, 2, 4). Its border is then a straight line between kept
//      vertices, while the neighbour at a finer level follows the real terrain between them: a crack. A skirt (a wall of
//      kSkirtDepth metres hanging down from the border) closes it without touching the neighbour.
//
// Conventions: right-handed, y up, matrices row-major m[row][col] with column vectors (v' = M * v). The camera looks
// along -z at yaw 0.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace tmesh {

inline constexpr float kCellSize = 2.0F;        // metres between the vertices of the finest level
inline constexpr int kWorldCells = 256;         // the world is 256 x 256 cells
inline constexpr float kWorldSize = static_cast<float>(kWorldCells) * kCellSize;  // 512 m
inline constexpr float kHeightScale = 90.0F;    // highest possible point, metres
inline constexpr float kSkirtDepth = 7.0F;      // how far a skirt hangs below its border, metres
inline constexpr int kLodLevels = 3;            // strides 1, 2, 4
inline constexpr std::size_t kVertexFloats = 7;  // x y z, nx ny nz, skirt flag (0 = terrain, 1 = bottom of a skirt)

// ---------------------------------------------------------------------------------------------- vectors

struct Vec3 {
    float x = 0.0F, y = 0.0F, z = 0.0F;
};

inline Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator*(Vec3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
inline float dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(Vec3 a, Vec3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline float length(Vec3 a) { return std::sqrt(dot(a, a)); }
inline Vec3 normalize(Vec3 a) {
    const float l = length(a);
    return l > 0.0F ? a * (1.0F / l) : a;
}

// ---------------------------------------------------------------------------------------------- noise

/// A hash of a lattice point and a seed (integer mixing, no tables), the same on every platform.
inline std::uint32_t hash2(int x, int z, std::uint32_t seed) {
    std::uint32_t h = static_cast<std::uint32_t>(x) * 374761393U + static_cast<std::uint32_t>(z) * 668265263U + seed * 2246822519U;
    h = (h ^ (h >> 13)) * 1274126177U;
    return h ^ (h >> 16);
}

/// The random value at a lattice point, in [0, 1).
inline float lattice(int x, int z, std::uint32_t seed) { return static_cast<float>(hash2(x, z, seed) >> 8) / 16777216.0F; }

/// Quintic ease, slope 0 at both ends, so the cell borders of the lattice do not show as creases.
inline float fade(float t) { return t * t * t * (t * (t * 6.0F - 15.0F) + 10.0F); }

inline float lerp(float a, float b, float t) { return a + (b - a) * t; }

/// Smooth value noise in [0, 1]: the four lattice values around the point, blended with the fade.
inline float valueNoise(float x, float z, std::uint32_t seed) {
    // The cast to int below is only defined for values that fit: junk (NaN, inf, absurd distances) becomes a finite point.
    x = std::isfinite(x) ? std::clamp(x, -1.0e6F, 1.0e6F) : 0.0F;
    z = std::isfinite(z) ? std::clamp(z, -1.0e6F, 1.0e6F) : 0.0F;
    const float fx = std::floor(x), fz = std::floor(z);
    const int ix = static_cast<int>(fx), iz = static_cast<int>(fz);
    const float u = fade(x - fx), v = fade(z - fz);
    const float top = lerp(lattice(ix, iz, seed), lattice(ix + 1, iz, seed), u);
    const float bottom = lerp(lattice(ix, iz + 1, seed), lattice(ix + 1, iz + 1, seed), u);
    return lerp(top, bottom, v);
}

inline constexpr int kOctaves = 7;
inline constexpr float kBaseWavelength = 240.0F;  // metres of the biggest hills
inline constexpr float kGain = 0.55F;
inline constexpr std::uint32_t kSeed = 1337U;

/// Fractal sum of kOctaves octaves, each twice as fine and kGain times as strong, normalised back to [0, 1].
inline float fbm(float x, float z) {
    float sum = 0.0F, amplitude = 1.0F, norm = 0.0F, frequency = 1.0F / kBaseWavelength;
    for (int o = 0; o < kOctaves; ++o) {
        sum += amplitude * valueNoise(x * frequency, z * frequency, kSeed + static_cast<std::uint32_t>(o) * 101U);
        norm += amplitude;
        amplitude *= kGain;
        frequency *= 2.0F;
    }
    return sum / norm;
}

inline constexpr float kShapeLow = 0.22F;   // fbm values below this are lowland at height 0
inline constexpr float kShapeHigh = 0.72F;  // and above this the highest peaks

/// Stretches the narrow middle of fbm (it rarely reaches 0 or 1) over 0..1 and bends it so valleys are wide, peaks narrow.
inline float shape(float f) {
    const float t = std::clamp((f - kShapeLow) / (kShapeHigh - kShapeLow), 0.0F, 1.0F);
    return std::pow(t, 1.5F);
}

/// The terrain height at a world position, metres, 0 .. kHeightScale.
inline float heightAt(float x, float z) { return kHeightScale * shape(fbm(x, z)); }

/// Unit normal from the heights one step to each side (central differences). Slope 1 in +x leans the normal to -x.
inline Vec3 normalFromHeights(float hNegX, float hPosX, float hNegZ, float hPosZ, float step) {
    return normalize({hNegX - hPosX, 2.0F * step, hNegZ - hPosZ});
}

// ---------------------------------------------------------------------------------------------- layout

/// Where the chunk coordinates come from: the right way (one world function) or the mistake (every chunk its own origin).
enum class Coords { World, Local };

struct Layout {
    int chunkCells = 32;  // cells per chunk edge at the finest level

    bool valid() const { return chunkCells >= 16 && chunkCells <= 64 && kWorldCells % chunkCells == 0; }
    int chunksPerSide() const { return kWorldCells / std::max(chunkCells, 1); }
    float chunkSize() const { return static_cast<float>(chunkCells) * kCellSize; }
    /// World coordinate of the low edge of chunk column / row c.
    float origin(int c) const { return -kWorldSize / 2.0F + static_cast<float>(c) * chunkSize(); }
    /// The chunk column / row that holds a world coordinate. A border belongs to the chunk after it, outside clamps.
    int chunkOf(float w) const {
        if (!(chunkSize() > 0.0F)) return 0;
        const float rel = (w - origin(0)) / chunkSize();
        if (!(rel > 0.0F)) return 0;  // also catches NaN
        return static_cast<int>(std::min(std::floor(rel), static_cast<float>(chunksPerSide() - 1)));
    }
};

inline int lodStride(int lod) { return 1 << std::clamp(lod, 0, kLodLevels - 1); }

/// Height at a position inside a chunk, local metres from the chunk's low corner. Coords::Local forgets which chunk it is.
inline float terrainHeight(const Layout& l, Coords mode, int cx, int cz, float lx, float lz) {
    return mode == Coords::World ? heightAt(l.origin(cx) + lx, l.origin(cz) + lz) : heightAt(lx, lz);
}

/// Height of the drawn terrain under a world position (for keeping the camera above it).
inline float surfaceHeight(const Layout& l, Coords mode, float wx, float wz) {
    if (mode == Coords::World) return heightAt(wx, wz);
    const int cx = l.chunkOf(wx), cz = l.chunkOf(wz);
    return heightAt(wx - l.origin(cx), wz - l.origin(cz));
}

// ---------------------------------------------------------------------------------------------- chunk mesh

struct ChunkMesh {
    std::vector<float> vertices;          // kVertexFloats per vertex
    std::vector<std::uint32_t> indices;   // triangles: first the surface, then the skirts
    int cells = 0;                        // cells per edge at this LOD
    int surfaceTriangles = 0;
    float minX = 0.0F, maxX = 0.0F, minZ = 0.0F, maxZ = 0.0F;

    int vertexCount() const { return static_cast<int>(vertices.size() / kVertexFloats); }
};

inline int triangleCount(const Layout& l, int lod, bool skirts) {
    if (!l.valid() || lod < 0 || lod >= kLodLevels) return 0;
    const int n = l.chunkCells / lodStride(lod);
    return 2 * n * n + (skirts ? 8 * n : 0);
}

/// Builds chunk (cx, cz) at a LOD: a (n + 1) x (n + 1) grid, n = cells / stride, row after row (index = j * (n + 1) + i),
/// two triangles per cell counter-clockwise seen from above. With skirts, four more rows of n + 1 vertices follow (one per
/// side: z low, z high, x low, x high) that repeat the border vertex kSkirtDepth lower, and 2 n triangles per side.
inline ChunkMesh buildChunk(const Layout& l, Coords mode, bool skirts, int cx, int cz, int lod) {
    ChunkMesh m;
    if (!l.valid() || lod < 0 || lod >= kLodLevels) return m;
    const int per = l.chunksPerSide();
    if (cx < 0 || cz < 0 || cx >= per || cz >= per) return m;

    const int stride = lodStride(lod);
    const int n = l.chunkCells / stride;
    const float step = static_cast<float>(stride) * kCellSize;
    m.cells = n;
    m.minX = l.origin(cx);
    m.minZ = l.origin(cz);
    m.maxX = m.minX + l.chunkSize();
    m.maxZ = m.minZ + l.chunkSize();
    const std::size_t surfaceVertices = static_cast<std::size_t>(n + 1) * static_cast<std::size_t>(n + 1);
    m.vertices.reserve((surfaceVertices + (skirts ? 4U * static_cast<std::size_t>(n + 1) : 0U)) * kVertexFloats);

    const auto push = [&m](Vec3 p, Vec3 nrm, float flag) { m.vertices.insert(m.vertices.end(), {p.x, p.y, p.z, nrm.x, nrm.y, nrm.z, flag}); };
    const auto at = [n](int i, int j) { return static_cast<std::uint32_t>(j * (n + 1) + i); };

    for (int j = 0; j <= n; ++j) {
        for (int i = 0; i <= n; ++i) {
            const float lx = static_cast<float>(i) * step, lz = static_cast<float>(j) * step;
            const float h = terrainHeight(l, mode, cx, cz, lx, lz);
            // The normal looks one FINEST cell to each side, whatever the LOD, and may leave the chunk: both chunks that
            // share this border point compute exactly the same vector.
            const Vec3 nrm = normalFromHeights(terrainHeight(l, mode, cx, cz, lx - kCellSize, lz), terrainHeight(l, mode, cx, cz, lx + kCellSize, lz),
                                               terrainHeight(l, mode, cx, cz, lx, lz - kCellSize), terrainHeight(l, mode, cx, cz, lx, lz + kCellSize), kCellSize);
            push({m.minX + lx, h, m.minZ + lz}, nrm, 0.0F);
        }
    }
    for (int j = 0; j < n; ++j) {
        for (int i = 0; i < n; ++i) {
            const std::uint32_t i0 = at(i, j), i1 = at(i + 1, j), i2 = at(i, j + 1), i3 = at(i + 1, j + 1);
            m.indices.insert(m.indices.end(), {i0, i2, i1, i1, i2, i3});
        }
    }
    m.surfaceTriangles = 2 * n * n;
    if (!skirts) return m;

    // Skirt rows. The top of side s at position k is a surface vertex, the bottom a new vertex below it.
    const auto top = [&](int side, int k) {
        switch (side) {
            case 0: return at(k, 0);
            case 1: return at(k, n);
            case 2: return at(0, k);
            default: return at(n, k);
        }
    };
    const std::uint32_t base = static_cast<std::uint32_t>(surfaceVertices);
    for (int side = 0; side < 4; ++side) {
        for (int k = 0; k <= n; ++k) {
            const std::size_t o = static_cast<std::size_t>(top(side, k)) * kVertexFloats;
            const Vec3 p{m.vertices[o], m.vertices[o + 1] - kSkirtDepth, m.vertices[o + 2]};
            push(p, {m.vertices[o + 3], m.vertices[o + 4], m.vertices[o + 5]}, 1.0F);
        }
    }
    for (int side = 0; side < 4; ++side) {
        const bool flip = side == 1 || side == 2;  // the winding that faces outwards depends on the side
        for (int k = 0; k < n; ++k) {
            const std::uint32_t a0 = top(side, k), a1 = top(side, k + 1);
            const std::uint32_t b0 = base + static_cast<std::uint32_t>(side * (n + 1) + k), b1 = b0 + 1;
            if (!flip) {
                m.indices.insert(m.indices.end(), {a0, a1, b0, a1, b1, b0});
            } else {
                m.indices.insert(m.indices.end(), {a1, a0, b0, a1, b0, b1});
            }
        }
    }
    return m;
}

/// Every edge of a triangle list once, as pairs (a, b) with a < b, for the wireframe.
inline std::vector<std::uint32_t> uniqueEdges(const std::vector<std::uint32_t>& indices) {
    std::vector<std::pair<std::uint32_t, std::uint32_t>> edges;
    edges.reserve(indices.size());
    for (std::size_t t = 0; t + 2 < indices.size(); t += 3) {
        for (std::size_t e = 0; e < 3; ++e) {
            const std::uint32_t a = indices[t + e], b = indices[t + (e + 1) % 3];
            edges.emplace_back(std::min(a, b), std::max(a, b));
        }
    }
    std::sort(edges.begin(), edges.end());
    edges.erase(std::unique(edges.begin(), edges.end()), edges.end());
    std::vector<std::uint32_t> out;
    out.reserve(edges.size() * 2);
    for (const auto& e : edges) out.insert(out.end(), {e.first, e.second});
    return out;
}

// ---------------------------------------------------------------------------------------------- LOD

/// 0 near, 1 beyond lodDistance, 2 beyond twice that. The threshold itself belongs to the coarser level.
inline int lodForDistance(float distance, float lodDistance) {
    const float base = std::max(lodDistance, 1.0F);
    if (distance >= 2.0F * base) return 2;
    if (distance >= base) return 1;
    return 0;  // also for negative and NaN distances
}

/// Horizontal distance from a point to the nearest point of a chunk, 0 inside. Nearest, not centre: the chunk the camera
/// stands next to must be fine even when its centre is a chunk size away.
inline float distanceToChunk(const Layout& l, int cx, int cz, float camX, float camZ) {
    const float x0 = l.origin(cx), z0 = l.origin(cz);
    const float dx = std::max({x0 - camX, 0.0F, camX - (x0 + l.chunkSize())});
    const float dz = std::max({z0 - camZ, 0.0F, camZ - (z0 + l.chunkSize())});
    return std::sqrt(dx * dx + dz * dz);
}

struct LodPlan {
    std::vector<int> lods;  // per chunk, row after row (index = cz * chunksPerSide + cx)
    int count[kLodLevels] = {0, 0, 0};
};

inline LodPlan planLods(const Layout& l, float camX, float camZ, float lodDistance) {
    LodPlan plan;
    const int per = l.chunksPerSide();
    plan.lods.reserve(static_cast<std::size_t>(per) * static_cast<std::size_t>(per));
    for (int cz = 0; cz < per; ++cz) {
        for (int cx = 0; cx < per; ++cx) {
            const int lod = lodForDistance(distanceToChunk(l, cx, cz, camX, camZ), lodDistance);
            plan.lods.push_back(lod);
            ++plan.count[lod];
        }
    }
    return plan;
}

inline long planTriangles(const Layout& l, const LodPlan& plan, bool skirts) {
    long sum = 0;
    for (const int lod : plan.lods) sum += triangleCount(l, lod, skirts);
    return sum;
}

// ---------------------------------------------------------------------------------------------- matrices and camera

struct Vec4 {
    float x = 0.0F, y = 0.0F, z = 0.0F, w = 0.0F;
};

struct Mat4 {
    float m[4][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
};

inline Mat4 operator*(const Mat4& a, const Mat4& b) {
    Mat4 r;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            float sum = 0.0F;
            for (int k = 0; k < 4; ++k) sum += a.m[i][k] * b.m[k][j];
            r.m[i][j] = sum;
        }
    }
    return r;
}

inline Vec4 operator*(const Mat4& a, Vec4 v) {
    return {a.m[0][0] * v.x + a.m[0][1] * v.y + a.m[0][2] * v.z + a.m[0][3] * v.w, a.m[1][0] * v.x + a.m[1][1] * v.y + a.m[1][2] * v.z + a.m[1][3] * v.w,
            a.m[2][0] * v.x + a.m[2][1] * v.y + a.m[2][2] * v.z + a.m[2][3] * v.w, a.m[3][0] * v.x + a.m[3][1] * v.y + a.m[3][2] * v.z + a.m[3][3] * v.w};
}

inline Mat4 perspective(float fovY, float aspect, float zNear, float zFar) {
    const float f = 1.0F / std::tan(fovY / 2.0F);
    Mat4 r;
    r.m[0][0] = f / aspect;
    r.m[1][1] = f;
    r.m[2][2] = (zFar + zNear) / (zNear - zFar);
    r.m[2][3] = 2.0F * zFar * zNear / (zNear - zFar);
    r.m[3][2] = -1.0F;
    r.m[3][3] = 0.0F;
    return r;
}

inline Mat4 lookAt(Vec3 eye, Vec3 target, Vec3 up) {
    const Vec3 f = normalize(target - eye);
    const Vec3 s = normalize(cross(f, up));
    const Vec3 u = cross(s, f);
    Mat4 r;
    r.m[0][0] = s.x; r.m[0][1] = s.y; r.m[0][2] = s.z; r.m[0][3] = -dot(s, eye);
    r.m[1][0] = u.x; r.m[1][1] = u.y; r.m[1][2] = u.z; r.m[1][3] = -dot(u, eye);
    r.m[2][0] = -f.x; r.m[2][1] = -f.y; r.m[2][2] = -f.z; r.m[2][3] = dot(f, eye);
    return r;
}

inline constexpr float kFovY = 1.0F;
inline constexpr float kNear = 0.5F;
inline constexpr float kFar = 1800.0F;
inline constexpr float kMaxPitch = 1.5F;  // just short of straight up (pi/2 = 1.5708), so forward never lines up with up

struct FlyCamera {
    Vec3 pos{0.0F, 120.0F, 0.0F};
    float yaw = 0.0F;    // 0 looks along -z, pi/2 along +x
    float pitch = 0.0F;  // positive looks up

    Vec3 forward() const { return {std::sin(yaw) * std::cos(pitch), std::sin(pitch), -std::cos(yaw) * std::cos(pitch)}; }
    Vec3 right() const { return {std::cos(yaw), 0.0F, std::sin(yaw)}; }
    Mat4 view() const { return lookAt(pos, pos + forward(), {0.0F, 1.0F, 0.0F}); }

    void look(float dYaw, float dPitch) {
        if (!std::isfinite(dYaw) || !std::isfinite(dPitch)) return;
        yaw += dYaw;
        pitch = std::clamp(pitch + dPitch, -kMaxPitch, kMaxPitch);
    }
    /// Metres along the view direction, to the right, and straight up.
    void move(float ahead, float sideways, float up) {
        if (!std::isfinite(ahead) || !std::isfinite(sideways) || !std::isfinite(up)) return;
        pos = pos + forward() * ahead + right() * sideways + Vec3{0.0F, up, 0.0F};
    }
};

/// Lifts the camera to at least `clearance` metres above the ground.
inline void keepAbove(FlyCamera& c, float ground, float clearance) { c.pos.y = std::max(c.pos.y, ground + clearance); }

/// Keeps the camera within the world plus a margin.
inline void keepInside(FlyCamera& c, float margin) {
    const float half = kWorldSize / 2.0F + margin;
    c.pos.x = std::clamp(c.pos.x, -half, half);
    c.pos.z = std::clamp(c.pos.z, -half, half);
}

// ---------------------------------------------------------------------------------------------- the automatic flight

inline constexpr double kTwoPi = 2.0 * 3.14159265358979323846;

/// The angle on the flight circle. Kept in double and wrapped into [0, 2 pi) on every step, so it stays exact after hours.
/// A stall (background tab) is clamped to a quarter second instead of jumping the camera ahead.
struct PathClock {
    static constexpr double kMaxFrame = 0.25;
    double angle = 0.0;

    /// Returns the angle that was added.
    double advance(double frameSeconds, double radiansPerSecond) {
        if (!(frameSeconds > 0.0)) return 0.0;
        const double added = std::min(frameSeconds, kMaxFrame) * radiansPerSecond;
        angle = std::fmod(angle + added, kTwoPi);
        if (angle < 0.0) angle += kTwoPi;
        return added;
    }
};

struct PathPose {
    float x, z, yaw;
};

/// A counter-clockwise circle around the middle of the world; the camera looks at the middle, so near chunks fill the
/// foreground and the coarse ones sit behind them.
inline PathPose pathPose(double angle, float radius) {
    const double halfPi = kTwoPi / 4.0;
    return {radius * static_cast<float>(std::cos(angle)), radius * static_cast<float>(std::sin(angle)), static_cast<float>(angle - halfPi)};
}

/// Blend factor for "move a part of the way to the target": 1 - exp(-lambda dt). Frame-rate independent.
inline float smoothFactor(float lambda, float dt) {
    if (!(dt > 0.0F)) return 0.0F;
    return 1.0F - std::exp(-lambda * dt);
}

}  // namespace tmesh
