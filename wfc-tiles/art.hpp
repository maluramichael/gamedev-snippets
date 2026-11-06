// The tile pictures, drawn by code from the edge labels of a tile (no image files). No SDL, so test.cpp can check that
// tiles which are allowed to touch also look continuous along the shared edge.
//
// A tile is a unit square, u to the right and v down. Water, sand and roads are all derived from the four edge labels:
//
//   Water on an edge pushes a flat lens into the tile: the depth is 0.54 * (1 - |t|^4) at the position t along that
//   edge (-1 .. 1). The lens is 0 at both ends of the edge, so the corners belong to the grass edges next to it, and
//   full depth in the middle, so two water edges that face each other meet in the middle of the tile. A thin sand rim
//   follows the lens.
//   Road on an edge draws a band from the middle of that edge to the tile center. All bands of a tile meet there.
//   A bridge is a road across two water edges: the band is wood and gets plank lines.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "logic.hpp"

namespace art {

struct Rgb {
    float r, g, b;
};

enum class Kind { Grass, Sand, Water, Road, RoadRim, Wood, WoodRim, Plank };

inline constexpr Rgb kGrass{0.588F, 0.776F, 0.463F};
inline constexpr Rgb kSand{0.925F, 0.863F, 0.627F};
inline constexpr Rgb kWater{0.361F, 0.627F, 0.839F};
inline constexpr Rgb kRoad{0.808F, 0.682F, 0.486F};
inline constexpr Rgb kRoadRim{0.553F, 0.431F, 0.275F};
inline constexpr Rgb kWood{0.690F, 0.490F, 0.306F};
inline constexpr Rgb kWoodRim{0.357F, 0.247F, 0.153F};
inline constexpr Rgb kPlank{0.510F, 0.353F, 0.208F};

inline Rgb colorOf(Kind k) {
    switch (k) {
        case Kind::Grass: return kGrass;
        case Kind::Sand: return kSand;
        case Kind::Water: return kWater;
        case Kind::Road: return kRoad;
        case Kind::RoadRim: return kRoadRim;
        case Kind::Wood: return kWood;
        case Kind::WoodRim: return kWoodRim;
        case Kind::Plank: return kPlank;
    }
    return kGrass;
}

constexpr double kLensDepth = 0.54;
constexpr double kSandRim = 0.09;
constexpr double kRoadHalf = 0.14;  // half width of the road surface
constexpr double kRimHalf = 0.19;   // half width including the dark rim

/// 1 in the middle of an edge, 0 at both ends, nearly flat in between.
inline double lensProfile(double t) {
    const double a = t * t;
    return std::max(0.0, 1.0 - a * a);
}

/// Distance and position along the edge `side` for the point (u, v).
inline void edgeFrame(int side, double u, double v, double& dist, double& lateral) {
    switch (side) {
        case 0: dist = v; lateral = (u - 0.5) * 2.0; break;
        case 1: dist = 1.0 - u; lateral = (v - 0.5) * 2.0; break;
        case 2: dist = 1.0 - v; lateral = (u - 0.5) * 2.0; break;
        default: dist = u; lateral = (v - 0.5) * 2.0; break;
    }
}

/// Distance from (u, v) to the segment from the middle of edge `side` to the tile center.
inline double roadDistance(int side, double u, double v) {
    static constexpr std::array<double, 4> px{0.5, 1.0, 0.5, 0.0};
    static constexpr std::array<double, 4> py{0.0, 0.5, 1.0, 0.5};
    const double ax = px[static_cast<std::size_t>(side)];
    const double ay = py[static_cast<std::size_t>(side)];
    const double dx = 0.5 - ax;
    const double dy = 0.5 - ay;
    const double along = std::clamp(((u - ax) * dx + (v - ay) * dy) / (dx * dx + dy * dy), 0.0, 1.0);
    const double cx = ax + dx * along;
    const double cy = ay + dy * along;
    return std::hypot(u - cx, v - cy);
}

/// What the tile shows at (u, v), both in [0, 1].
inline Kind kindAt(const wfc::Tile& tile, double u, double v) {
    // Ground first: water, sand or grass.
    Kind ground = Kind::Grass;
    int waterEdges = 0;
    int roadEdges = 0;
    for (int s = 0; s < wfc::kSides; ++s) {
        const int label = tile.edge[static_cast<std::size_t>(s)];
        waterEdges += label == wfc::Water ? 1 : 0;
        roadEdges += label == wfc::Road ? 1 : 0;
    }
    for (int s = 0; s < wfc::kSides && ground != Kind::Water; ++s) {
        if (tile.edge[static_cast<std::size_t>(s)] != wfc::Water) continue;
        double dist = 0.0;
        double lateral = 0.0;
        edgeFrame(s, u, v, dist, lateral);
        const double p = lensProfile(lateral);
        const double f = kLensDepth * p - dist;
        if (f > 0.0) {
            ground = Kind::Water;
            continue;
        }
        if (f > -kSandRim * p) ground = Kind::Sand;
    }
    if (roadEdges == 0) return ground;

    // Road on top. A bridge has road on two opposite edges and water on the other two.
    const bool bridge = waterEdges > 0;
    double best = 1e9;
    for (int s = 0; s < wfc::kSides; ++s) {
        if (tile.edge[static_cast<std::size_t>(s)] != wfc::Road) continue;
        best = std::min(best, roadDistance(s, u, v));
    }
    if (best > kRimHalf) return ground;
    // Position along the travel direction of a bridge (0 .. 1 over the tile). Both bridge ends look like the road.
    const bool horizontal = tile.edge[1] == wfc::Road;
    const double travel = horizontal ? u : v;
    const bool deck = bridge && travel > 0.15 && travel < 0.85;
    if (best > kRoadHalf) return deck ? Kind::WoodRim : Kind::RoadRim;
    if (!deck) return Kind::Road;
    const double phase = travel * 8.0 - std::floor(travel * 8.0);
    return phase < 0.1 ? Kind::Plank : Kind::Wood;
}

inline Rgb sample(const wfc::Tile& tile, double u, double v) { return colorOf(kindAt(tile, u, v)); }

inline std::uint32_t hash(std::uint32_t x, std::uint32_t y, std::uint32_t salt) {
    std::uint32_t h = x * 374761393U + y * 668265263U + salt * 2246822519U;
    h = (h ^ (h >> 13)) * 1274126177U;
    return h ^ (h >> 16);
}

/// size x size RGBA bytes (byte order r, g, b, a), 3x3 samples per pixel so the edges are smooth. Pixels that lie fully on
/// grass or water get a few darker or lighter specks (same for the same salt).
inline std::vector<std::uint8_t> render(const wfc::Tile& tile, int size, std::uint32_t salt) {
    std::vector<std::uint8_t> px(static_cast<std::size_t>(size) * static_cast<std::size_t>(size) * 4U, 255);
    constexpr int kSub = 3;
    for (int y = 0; y < size; ++y) {
        for (int x = 0; x < size; ++x) {
            float r = 0.0F;
            float g = 0.0F;
            float b = 0.0F;
            int grass = 0;
            int water = 0;
            for (int j = 0; j < kSub; ++j) {
                for (int i = 0; i < kSub; ++i) {
                    const double u = (x + (i + 0.5) / kSub) / size;
                    const double v = (y + (j + 0.5) / kSub) / size;
                    const Kind k = kindAt(tile, u, v);
                    grass += k == Kind::Grass ? 1 : 0;
                    water += k == Kind::Water ? 1 : 0;
                    const Rgb c = colorOf(k);
                    r += c.r;
                    g += c.g;
                    b += c.b;
                }
            }
            constexpr float n = static_cast<float>(kSub * kSub);
            r /= n;
            g /= n;
            b /= n;
            const std::uint32_t h = hash(static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y), salt) % 100U;
            if (grass == kSub * kSub && h < 6U) {
                r *= 0.88F;
                g *= 0.9F;
                b *= 0.88F;
            } else if (water == kSub * kSub && h < 4U) {
                r = std::min(1.0F, r * 1.18F);
                g = std::min(1.0F, g * 1.12F);
                b = std::min(1.0F, b * 1.06F);
            }
            const std::size_t o = (static_cast<std::size_t>(y) * static_cast<std::size_t>(size) + static_cast<std::size_t>(x)) * 4U;
            px[o + 0] = static_cast<std::uint8_t>(std::clamp(r, 0.0F, 1.0F) * 255.0F + 0.5F);
            px[o + 1] = static_cast<std::uint8_t>(std::clamp(g, 0.0F, 1.0F) * 255.0F + 0.5F);
            px[o + 2] = static_cast<std::uint8_t>(std::clamp(b, 0.0F, 1.0F) * 255.0F + 0.5F);
        }
    }
    return px;
}

/// Mean color of a rendered tile, for the washed-out preview of an undecided cell.
inline Rgb mean(const std::vector<std::uint8_t>& px) {
    double r = 0.0;
    double g = 0.0;
    double b = 0.0;
    const std::size_t n = px.size() / 4U;
    for (std::size_t i = 0; i < n; ++i) {
        r += px[i * 4U + 0];
        g += px[i * 4U + 1];
        b += px[i * 4U + 2];
    }
    if (n == 0) return kGrass;
    const double k = 1.0 / (255.0 * static_cast<double>(n));
    return {static_cast<float>(r * k), static_cast<float>(g * k), static_cast<float>(b * k)};
}

}  // namespace art
