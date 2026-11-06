// The pictures of the tiles: a few boxes, prisms and cones per tile, flat shaded, built once from the rules of the tile.
// No GL in here, so test.cpp can check that every piece stays inside its cell and that the wall arms of two neighbours
// meet exactly on the shared face.
//
// Local coordinates of a tile: x and z from -0.5 to 0.5 around the middle of the cell, y from 0 (floor) to 1 (ceiling).
// Everything is a triangle list with a stored normal and a color per vertex (flat shading needs a vertex per face).
#pragma once

#include <cmath>
#include <vector>

#include "castle.hpp"
#include "logic.hpp"

namespace w3 {

struct Rgb {
    float r = 1.0F, g = 1.0F, b = 1.0F;
};
constexpr Rgb rgb(unsigned hex) { return {static_cast<float>((hex >> 16) & 0xFFU) / 255.0F, static_cast<float>((hex >> 8) & 0xFFU) / 255.0F, static_cast<float>(hex & 0xFFU) / 255.0F}; }

struct Vertex {
    float px, py, pz;
    float nx, ny, nz;
    float r, g, b;
};
using Verts = std::vector<Vertex>;
constexpr int kFloatsPerVertex = 9;

inline constexpr float kPi = 3.14159265358979F;

/// Collects triangles. Every solid is convex, so the builder is told one point inside it (`center`) and flips any triangle
/// that would face inwards. That keeps the winding of the corner lists below from mattering.
class MeshBuilder {
public:
    Verts v;
    Vec3 center{};

    void tri(Vec3 a, Vec3 b, Vec3 c, Rgb col) {
        Vec3 n = cross(b - a, c - a);
        if (length(n) < 1e-9F) return;  // a sliver without area has no normal
        n = normalize(n);
        const Vec3 g = (a + b + c) * (1.0F / 3.0F);
        if (dot(n, g - center) < 0.0F) {
            std::swap(b, c);
            n = n * -1.0F;
        }
        for (const Vec3& p : {a, b, c}) v.push_back({p.x, p.y, p.z, n.x, n.y, n.z, col.r, col.g, col.b});
    }
    void quad(Vec3 a, Vec3 b, Vec3 c, Vec3 d, Rgb col) {
        tri(a, b, c, col);
        tri(a, c, d, col);
    }

    void box(Vec3 lo, Vec3 hi, Rgb side, Rgb top) {
        center = (lo + hi) * 0.5F;
        const Vec3 p[8] = {{lo.x, lo.y, lo.z}, {hi.x, lo.y, lo.z}, {hi.x, lo.y, hi.z}, {lo.x, lo.y, hi.z},
                           {lo.x, hi.y, lo.z}, {hi.x, hi.y, lo.z}, {hi.x, hi.y, hi.z}, {lo.x, hi.y, hi.z}};
        quad(p[4], p[5], p[6], p[7], top);   // +y
        quad(p[0], p[1], p[2], p[3], side);  // -y
        quad(p[1], p[2], p[6], p[5], side);  // +x
        quad(p[0], p[3], p[7], p[4], side);  // -x
        quad(p[3], p[2], p[6], p[7], side);  // +z
        quad(p[0], p[1], p[5], p[4], side);  // -z
    }

    /// A regular n-gon prism around the y axis. The facets alternate between two colors for the faceted look.
    void prism(int n, float cx, float cz, float radius, float y0, float y1, float rotation, Rgb a, Rgb b, Rgb top) {
        center = {cx, (y0 + y1) / 2.0F, cz};
        for (int k = 0; k < n; ++k) {
            const Vec3 p0 = ring(n, k, cx, cz, radius, rotation);
            const Vec3 p1 = ring(n, k + 1, cx, cz, radius, rotation);
            quad({p0.x, y0, p0.z}, {p1.x, y0, p1.z}, {p1.x, y1, p1.z}, {p0.x, y1, p0.z}, (k & 1) != 0 ? b : a);
            tri({cx, y1, cz}, {p0.x, y1, p0.z}, {p1.x, y1, p1.z}, top);
            tri({cx, y0, cz}, {p0.x, y0, p0.z}, {p1.x, y0, p1.z}, a);
        }
    }

    /// A cone (n facets, alternating colors) from a ring at y0 up to the apex.
    void cone(int n, float cx, float cz, float radius, float y0, float apexY, float rotation, Rgb a, Rgb b) {
        center = {cx, y0 + (apexY - y0) * 0.3F, cz};
        for (int k = 0; k < n; ++k) {
            const Vec3 p0 = ring(n, k, cx, cz, radius, rotation);
            const Vec3 p1 = ring(n, k + 1, cx, cz, radius, rotation);
            tri({p0.x, y0, p0.z}, {p1.x, y0, p1.z}, {cx, apexY, cz}, (k & 1) != 0 ? b : a);
            tri({cx, y0, cz}, {p0.x, y0, p0.z}, {p1.x, y0, p1.z}, a);
        }
    }

    /// A faceted ball (a few rings of quads, triangles at the poles).
    void ball(float cx, float cy, float cz, float radius, int segments, int rings, Rgb a, Rgb b) {
        center = {cx, cy, cz};
        for (int i = 0; i < rings; ++i) {
            const float t0 = kPi * static_cast<float>(i) / static_cast<float>(rings);
            const float t1 = kPi * static_cast<float>(i + 1) / static_cast<float>(rings);
            for (int k = 0; k < segments; ++k) {
                const float s0 = 2.0F * kPi * static_cast<float>(k) / static_cast<float>(segments);
                const float s1 = 2.0F * kPi * static_cast<float>(k + 1) / static_cast<float>(segments);
                const auto at = [&](float t, float s) { return Vec3{cx + radius * std::sin(t) * std::cos(s), cy + radius * std::cos(t), cz + radius * std::sin(t) * std::sin(s)}; };
                const Rgb col = ((i + k) & 1) != 0 ? b : a;
                if (i == 0) tri(at(t0, s0), at(t1, s0), at(t1, s1), col);
                else if (i == rings - 1) tri(at(t0, s0), at(t1, s0), at(t0, s1), col);
                else quad(at(t0, s0), at(t1, s0), at(t1, s1), at(t0, s1), col);
            }
        }
    }

    /// A gable roof with its ridge along x: triangles at both ends, two slopes, a floor.
    void gable(float x0, float x1, float halfWidth, float y0, float ridgeY, Rgb slopeA, Rgb slopeB, Rgb end) {
        center = {(x0 + x1) / 2.0F, y0 + (ridgeY - y0) * 0.3F, 0.0F};
        tri({x0, y0, -halfWidth}, {x0, y0, halfWidth}, {x0, ridgeY, 0.0F}, end);
        tri({x1, y0, -halfWidth}, {x1, y0, halfWidth}, {x1, ridgeY, 0.0F}, end);
        quad({x0, y0, -halfWidth}, {x1, y0, -halfWidth}, {x1, ridgeY, 0.0F}, {x0, ridgeY, 0.0F}, slopeA);
        quad({x0, y0, halfWidth}, {x1, y0, halfWidth}, {x1, ridgeY, 0.0F}, {x0, ridgeY, 0.0F}, slopeB);
        quad({x0, y0, -halfWidth}, {x1, y0, -halfWidth}, {x1, y0, halfWidth}, {x0, y0, halfWidth}, slopeB);
    }

private:
    static Vec3 ring(int n, int k, float cx, float cz, float radius, float rotation) {
        const float a = rotation + 2.0F * kPi * static_cast<float>(k) / static_cast<float>(n);
        return {cx + radius * std::cos(a), 0.0F, cz + radius * std::sin(a)};
    }
};

// ---------------------------------------------------------------------------------------------- palette

namespace pal {
inline constexpr Rgb stone = rgb(0xDAD4C6);
inline constexpr Rgb stoneTop = rgb(0xEAE5D8);
inline constexpr Rgb stoneDark = rgb(0xB7AFA0);
inline constexpr Rgb tower = rgb(0xD3CBBC);
inline constexpr Rgb towerAlt = rgb(0xC8BFAE);
inline constexpr Rgb slit = rgb(0x39434F);
inline constexpr Rgb plaster = rgb(0xF4ECD6);
inline constexpr Rgb plasterDark = rgb(0xE4D9BC);
inline constexpr Rgb timber = rgb(0x8A6A4A);
inline constexpr Rgb window = rgb(0x3F5568);
inline constexpr Rgb roofA = rgb(0xE2663C);
inline constexpr Rgb roofB = rgb(0xCB5430);
inline constexpr Rgb teal = rgb(0x0D9488);
inline constexpr Rgb tealDark = rgb(0x0F766E);
inline constexpr Rgb trunk = rgb(0x8A6642);
inline constexpr Rgb leafA = rgb(0x62B26B);
inline constexpr Rgb leafB = rgb(0x4F9E5E);
inline constexpr Rgb pineA = rgb(0x3E8A58);
inline constexpr Rgb pineB = rgb(0x2F7050);
inline constexpr Rgb grassA = rgb(0xAAD68F);
inline constexpr Rgb grassB = rgb(0x9DCB83);
inline constexpr Rgb grassEdge = rgb(0x86B96C);
inline constexpr Rgb soil = rgb(0x8F6E54);
inline constexpr Rgb soilDark = rgb(0x7A5B45);
}  // namespace pal

// ---------------------------------------------------------------------------------------------- tiles

inline constexpr float kPostHalf = 0.17F;    // half width of a pillar
inline constexpr float kArmHalf = 0.12F;     // half thickness of a wall arm
inline constexpr float kTowerRadius = 0.40F;  // circumscribed radius of the octagonal tower
inline constexpr float kOctagon = kPi / 8.0F;  // 22.5 degrees: turns the octagon so that flats face north, east, south, west
inline constexpr float kMerlonHeight = 0.12F;

/// Unit vector of an arm direction in the xz plane (x, z).
inline void armVector(int dir, float& dx, float& dz) {
    dx = static_cast<float>(kOffsets[static_cast<std::size_t>(dir)].x);
    dz = static_cast<float>(kOffsets[static_cast<std::size_t>(dir)].z);
}

/// A box that runs from `from` to `to` along an arm direction, `half` wide, from y0 to y1.
inline void armBox(MeshBuilder& m, int dir, float from, float to, float half, float y0, float y1, Rgb side, Rgb top) {
    float dx = 0.0F, dz = 0.0F;
    armVector(dir, dx, dz);
    const float ax = dx * from, az = dz * from, bx = dx * to, bz = dz * to;
    const float px = std::fabs(dz) * half, pz = std::fabs(dx) * half;  // the width runs across the arm
    m.box({std::min(ax, bx) - px, y0, std::min(az, bz) - pz}, {std::max(ax, bx) + px, y1, std::max(az, bz) + pz}, side, top);
}

inline void addArms(MeshBuilder& m, int arms, float from) {
    for (int i = 0; i < 4; ++i) {
        if ((arms >> i & 1) != 0) armBox(m, kArmDirs[static_cast<std::size_t>(i)], from, 0.5F, kArmHalf, 0.0F, 1.0F, pal::stone, pal::stoneTop);
    }
}

inline Verts rotateY90(Verts in) {
    for (Vertex& p : in) {
        const float x = p.px, z = p.pz, nx = p.nx, nz = p.nz;
        p.px = z;  // (x, z) -> (z, -x): a quarter turn around y
        p.pz = -x;
        p.nx = nz;
        p.nz = -nx;
    }
    return in;
}

/// Small dark box on the face of a wall: a window or an arrow slit. `dir` is the side it looks to, `distance` where the
/// wall surface is. It sticks out 0.014 and sinks in 0.004, so it never z-fights with the wall.
inline void addSlot(MeshBuilder& m, int dir, float distance, float halfWidth, float y0, float y1, Rgb col) {
    armBox(m, dir, distance - 0.004F, distance + 0.014F, halfWidth, y0, y1, col, col);
}

inline Verts buildTile(const CastleTile& t) {
    MeshBuilder m;
    switch (t.kind) {
        case Kind::AirGround:
        case Kind::AirBuilt:
        case Kind::AirSky: break;
        case Kind::Tree:
            m.box({-0.045F, 0.0F, -0.045F}, {0.045F, 0.32F, 0.045F}, pal::trunk, pal::trunk);
            m.ball(0.0F, 0.54F, 0.0F, 0.27F, 7, 4, pal::leafA, pal::leafB);
            break;
        case Kind::Pine:
            m.box({-0.04F, 0.0F, -0.04F}, {0.04F, 0.16F, 0.04F}, pal::trunk, pal::trunk);
            m.cone(8, 0.0F, 0.0F, 0.31F, 0.10F, 0.52F, 0.0F, pal::pineA, pal::pineB);
            m.cone(8, 0.0F, 0.0F, 0.25F, 0.34F, 0.76F, 0.2F, pal::pineB, pal::pineA);
            m.cone(8, 0.0F, 0.0F, 0.18F, 0.58F, 0.98F, 0.4F, pal::pineA, pal::pineB);
            break;
        case Kind::House:
        case Kind::HouseUpper: {
            const float floor = t.kind == Kind::House ? 0.10F : 0.0F;  // the stone plinth is only under the ground floor
            if (t.kind == Kind::House) m.box({-0.42F, 0.0F, -0.42F}, {0.42F, floor, 0.42F}, pal::stoneDark, pal::stone);
            m.box({-0.40F, floor, -0.40F}, {0.40F, 1.0F, 0.40F}, pal::plaster, pal::plasterDark);
            for (const float sx : {-1.0F, 1.0F}) {
                for (const float sz : {-1.0F, 1.0F}) {
                    m.box({sx * 0.40F - 0.03F, floor, sz * 0.40F - 0.03F}, {sx * 0.40F + 0.03F, 1.0F, sz * 0.40F + 0.03F}, pal::timber, pal::timber);
                }
            }
            for (const int dir : {PosX, NegX, PosZ, NegZ}) addSlot(m, dir, 0.40F, 0.09F, 0.46F, 0.72F, pal::window);
            break;
        }
        case Kind::RoofX: m.gable(-0.46F, 0.46F, 0.5F, 0.0F, 0.62F, pal::roofA, pal::roofB, pal::plasterDark); break;
        case Kind::RoofZ: return rotateY90(buildTile({Kind::RoofX, 0}));
        case Kind::Post:
            m.box({-kPostHalf, 0.0F, -kPostHalf}, {kPostHalf, 1.0F, kPostHalf}, pal::stoneDark, pal::stoneTop);
            addArms(m, t.arms, kPostHalf - 0.01F);
            break;
        case Kind::Tower: {
            m.prism(8, 0.0F, 0.0F, kTowerRadius, 0.0F, 1.0F, kOctagon, pal::tower, pal::towerAlt, pal::stoneTop);
            m.prism(8, 0.0F, 0.0F, kTowerRadius + 0.04F, 0.0F, 0.08F, kOctagon, pal::stoneDark, pal::stoneDark, pal::stoneDark);
            m.prism(8, 0.0F, 0.0F, kTowerRadius + 0.04F, 0.92F, 1.0F, kOctagon, pal::stoneDark, pal::stoneDark, pal::stoneTop);
            addArms(m, t.arms, 0.30F);
            const float apothem = kTowerRadius * std::cos(kOctagon);
            for (int i = 0; i < 4; ++i) {
                if ((t.arms >> i & 1) == 0) addSlot(m, kArmDirs[static_cast<std::size_t>(i)], apothem, 0.025F, 0.45F, 0.72F, pal::slit);
            }
            break;
        }
        case Kind::TowerRoof:
            m.prism(8, 0.0F, 0.0F, kTowerRadius + 0.04F, 0.0F, 0.07F, kOctagon, pal::tealDark, pal::tealDark, pal::tealDark);
            m.cone(8, 0.0F, 0.0F, 0.49F, 0.07F, 0.98F, kOctagon, pal::teal, pal::tealDark);
            break;
    }
    return std::move(m.v);
}

/// The battlements: small blocks that stand on top of a pillar or tower (and its arms) when nothing is above it. They
/// reach up to 0.12 above the cell, that is the only thing in the demo that leaves its cell.
inline Verts buildCap(const CastleTile& t) {
    MeshBuilder m;
    if (t.kind == Kind::Post) {
        for (const float sx : {-1.0F, 1.0F}) {
            for (const float sz : {-1.0F, 1.0F}) {
                m.box({sx * 0.115F - 0.05F, 1.0F, sz * 0.115F - 0.05F}, {sx * 0.115F + 0.05F, 1.0F + kMerlonHeight, sz * 0.115F + 0.05F}, pal::stone, pal::stoneTop);
            }
        }
        for (int i = 0; i < 4; ++i) {
            if ((t.arms >> i & 1) != 0) armBox(m, kArmDirs[static_cast<std::size_t>(i)], 0.28F, 0.40F, kArmHalf, 1.0F, 1.0F + kMerlonHeight, pal::stone, pal::stoneTop);
        }
    } else if (t.kind == Kind::Tower) {
        for (int k = 0; k < 8; ++k) {
            const float a = 2.0F * kPi * static_cast<float>(k) / 8.0F;
            const float cx = 0.325F * std::cos(a), cz = 0.325F * std::sin(a);
            m.box({cx - 0.055F, 1.0F, cz - 0.055F}, {cx + 0.055F, 1.0F + kMerlonHeight, cz + 0.055F}, pal::stone, pal::stoneTop);
        }
    }
    return std::move(m.v);
}

/// The island the world stands on: a slab of grass (checkered per cell) with a soil edge, y = 0 is the grass.
inline Verts buildSlab(int w, int d) {
    constexpr float margin = 0.45F;
    constexpr float thick = 0.8F;
    const float hx = static_cast<float>(w) / 2.0F, hz = static_cast<float>(d) / 2.0F;
    const float ox = hx + margin, oz = hz + margin;
    MeshBuilder m;
    m.center = {0.0F, -thick / 2.0F, 0.0F};
    for (int z = 0; z < d; ++z) {
        for (int x = 0; x < w; ++x) {
            const Vec3 lo = cellMin(w, d, x, 0, z);
            m.quad({lo.x, 0.0F, lo.z}, {lo.x + 1.0F, 0.0F, lo.z}, {lo.x + 1.0F, 0.0F, lo.z + 1.0F}, {lo.x, 0.0F, lo.z + 1.0F}, ((x + z) & 1) != 0 ? pal::grassB : pal::grassA);
        }
    }
    const Rgb rim = pal::grassB;
    m.quad({-ox, 0.0F, -oz}, {ox, 0.0F, -oz}, {ox, 0.0F, -hz}, {-ox, 0.0F, -hz}, rim);
    m.quad({-ox, 0.0F, hz}, {ox, 0.0F, hz}, {ox, 0.0F, oz}, {-ox, 0.0F, oz}, rim);
    m.quad({-ox, 0.0F, -hz}, {-hx, 0.0F, -hz}, {-hx, 0.0F, hz}, {-ox, 0.0F, hz}, rim);
    m.quad({hx, 0.0F, -hz}, {ox, 0.0F, -hz}, {ox, 0.0F, hz}, {hx, 0.0F, hz}, rim);
    const float bands[4] = {0.0F, -0.10F, -0.45F, -thick};
    const Rgb colors[3] = {pal::grassEdge, pal::soil, pal::soilDark};
    for (int i = 0; i < 3; ++i) {
        const float y1 = bands[i], y0 = bands[i + 1];
        m.quad({-ox, y0, oz}, {ox, y0, oz}, {ox, y1, oz}, {-ox, y1, oz}, colors[i]);    // +z
        m.quad({-ox, y0, -oz}, {ox, y0, -oz}, {ox, y1, -oz}, {-ox, y1, -oz}, colors[i]);  // -z
        m.quad({ox, y0, -oz}, {ox, y0, oz}, {ox, y1, oz}, {ox, y1, -oz}, colors[i]);    // +x
        m.quad({-ox, y0, -oz}, {-ox, y0, oz}, {-ox, y1, oz}, {-ox, y1, -oz}, colors[i]);  // -x
    }
    m.quad({-ox, -thick, -oz}, {ox, -thick, -oz}, {ox, -thick, oz}, {-ox, -thick, oz}, pal::soilDark);
    return std::move(m.v);
}

/// A cube from -0.5 to 0.5 on all axes: the ghost of an undecided cell.
inline Verts buildCube() {
    MeshBuilder m;
    m.box({-0.5F, -0.5F, -0.5F}, {0.5F, 0.5F, 0.5F}, {}, {});
    return std::move(m.v);
}

}  // namespace w3
