// Logic of the biomes and hillshade demo, no SDL, covered by test.cpp.
//
// Two noise maps (height and moisture) plus a temperature that falls with height pick a cell in a 4x4 Whittaker
// style table. The color between table cells is interpolated, so biomes fade into each other instead of ending on a
// hard edge. Hillshade (Esri's formula) lights the height map from a sun at a given azimuth and altitude.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace bio {

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kSeaLevel = 0.40625;  // 13/32, exact in float so a stored height compares equal
inline constexpr double kLapse = 1.2;         // temperature lost between the shore and the highest peak
inline constexpr double kShoreWidth = 0.025;  // height above the sea that still counts as beach
inline constexpr double kZFactor = 24.0;      // height 0..1 versus a cell of width 1, exaggerates the relief
inline constexpr double kShadeGain = 0.9;     // how strongly the hillshade darkens and brightens the color
inline constexpr double kShadeMin = 0.3;
inline constexpr double kShadeMax = 1.35;
inline constexpr int kCols = 4;  // moisture: dry to wet
inline constexpr int kRows = 4;  // temperature: cold to hot

struct Rgb {
    float r, g, b;
};

inline Rgb lerp(Rgb a, Rgb b, float t) { return {a.r + (b.r - a.r) * t, a.g + (b.g - a.g) * t, a.b + (b.b - a.b) * t}; }

inline Rgb scale(Rgb c, double k) {
    const auto f = static_cast<float>(k);
    return {c.r * f, c.g * f, c.b * f};
}

constexpr Rgb rgb(int r, int g, int b) { return {static_cast<float>(r) / 255.0F, static_cast<float>(g) / 255.0F, static_cast<float>(b) / 255.0F}; }

inline constexpr Rgb kSand = rgb(222, 200, 140);
inline constexpr Rgb kShallow = rgb(74, 160, 190);
inline constexpr Rgb kDeep = rgb(26, 70, 128);

inline double clamp01(double v) { return std::clamp(v, 0.0, 1.0); }

inline double smoothstep(double t) {
    t = clamp01(t);
    return t * t * (3.0 - 2.0 * t);
}

// ---------------------------------------------------------------------------------------------------------------
// Gradient noise, written for this demo (no dependency on another demo).

inline double fade(double t) { return t * t * t * (t * (t * 6.0 - 15.0) + 10.0); }

inline std::uint32_t hash(int x, int y, std::uint32_t seed) {
    std::uint32_t h = seed ^ (static_cast<std::uint32_t>(x) * 0x27D4EB2DU) ^ (static_cast<std::uint32_t>(y) * 0x165667B1U);
    h ^= h >> 15;
    h *= 0x2C1B3C6DU;
    h ^= h >> 12;
    h *= 0x297A2D39U;
    h ^= h >> 15;
    return h;
}

/// Dot product of the lattice gradient at (ix, iy) with the offset (dx, dy) from that corner.
inline double cornerDot(int ix, int iy, double dx, double dy, std::uint32_t seed) {
    constexpr double r = 0.70710678118654752;
    constexpr double dirs[8][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {r, r}, {-r, r}, {r, -r}, {-r, -r}};
    const auto& d = dirs[hash(ix, iy, seed) & 7U];
    return d[0] * dx + d[1] * dy;
}

/// Perlin style noise in [-1, 1], exactly 0 on the lattice points.
inline double noise2(double x, double y, std::uint32_t seed) {
    const double fx0 = std::floor(x);
    const double fy0 = std::floor(y);
    const int x0 = static_cast<int>(fx0);
    const int y0 = static_cast<int>(fy0);
    const double fx = x - fx0;
    const double fy = y - fy0;
    const double u = fade(fx);
    const double v = fade(fy);
    const double n00 = cornerDot(x0, y0, fx, fy, seed);
    const double n10 = cornerDot(x0 + 1, y0, fx - 1.0, fy, seed);
    const double n01 = cornerDot(x0, y0 + 1, fx, fy - 1.0, seed);
    const double n11 = cornerDot(x0 + 1, y0 + 1, fx - 1.0, fy - 1.0, seed);
    const double a = n00 + (n10 - n00) * u;
    const double b = n01 + (n11 - n01) * u;
    // Unit gradients give at most sqrt(2)/2, scaling brings the range to [-1, 1].
    return std::clamp((a + (b - a) * v) * 1.4142135623730951, -1.0, 1.0);
}

/// Fractal noise in [0, 1]: each octave doubles the frequency and halves the weight.
inline double fbm(double x, double y, std::uint32_t seed, int octaves) {
    double sum = 0.0;
    double total = 0.0;
    double amp = 1.0;
    double freq = 1.0;
    for (int i = 0; i < octaves; ++i) {
        sum += amp * noise2(x * freq, y * freq, seed + static_cast<std::uint32_t>(i) * 101U);
        total += amp;
        amp *= 0.5;
        freq *= 2.0;
    }
    return total > 0.0 ? clamp01(0.5 + 0.5 * sum / total) : 0.5;
}

// ---------------------------------------------------------------------------------------------------------------
// Biome table

struct Biome {
    const char* name;
    Rgb color;
};

/// Index = row * kCols + column. Row 0 is cold, row 3 is hot, column 0 is dry, column 3 is wet.
inline const std::array<Biome, 16>& table() {
    static const std::array<Biome, 16> t = {{
        {"Snow", rgb(240, 245, 250)},        {"Tundra", rgb(176, 190, 178)},   {"Taiga", rgb(70, 120, 110)},      {"Spruce", rgb(40, 92, 84)},
        {"Steppe", rgb(200, 190, 120)},      {"Meadow", rgb(150, 190, 100)},   {"Forest", rgb(62, 138, 74)},      {"Dark forest", rgb(34, 104, 62)},
        {"Shrubland", rgb(214, 172, 96)},    {"Prairie", rgb(186, 196, 80)},   {"Woodland", rgb(120, 168, 60)},   {"Swamp", rgb(70, 122, 92)},
        {"Desert", rgb(232, 190, 110)},      {"Savanna", rgb(214, 178, 70)},   {"Dry forest", rgb(150, 160, 48)}, {"Rainforest", rgb(16, 110, 56)},
    }};
    return t;
}

/// The table cell a moisture and temperature value falls into (values outside [0, 1] clamp to the outer cells).
inline int nearestCell(double moisture, double temp) {
    const int col = std::clamp(static_cast<int>(std::floor(clamp01(moisture) * kCols)), 0, kCols - 1);
    const int row = std::clamp(static_cast<int>(std::floor(clamp01(temp) * kRows)), 0, kRows - 1);
    return row * kCols + col;
}

/// Weight of the upper neighbor when f in [0, 1] runs from one cell center to the next. Width 0 is a hard step at
/// the border between the two cells, width 1 a smoothstep over the whole way, in between the plateaus stay flat.
inline double bandBlend(double f, double width) {
    if (width <= 1e-9) return f < 0.5 ? 0.0 : 1.0;
    const double lo = 0.5 - width / 2.0;
    return smoothstep((f - lo) / width);
}

/// Color for a moisture and temperature in [0, 1]. The colors of the four surrounding cell centers are mixed
/// bilinearly, with `bandBlend` shaping the mix per axis. That is what turns table cells into soft transitions.
inline Rgb biomeColor(double moisture, double temp, double width) {
    const auto& t = table();
    const double u = clamp01(moisture) * kCols - 0.5;
    const double v = clamp01(temp) * kRows - 0.5;
    const double cu = std::floor(u);
    const double cv = std::floor(v);
    const int i0 = static_cast<int>(cu);
    const int j0 = static_cast<int>(cv);
    const auto cell = [&t](int col, int row) {
        return t[static_cast<std::size_t>(std::clamp(row, 0, kRows - 1) * kCols + std::clamp(col, 0, kCols - 1))].color;
    };
    const auto fu = static_cast<float>(bandBlend(u - cu, width));
    const auto fv = static_cast<float>(bandBlend(v - cv, width));
    const Rgb top = lerp(cell(i0, j0), cell(i0 + 1, j0), fu);
    const Rgb bottom = lerp(cell(i0, j0 + 1), cell(i0 + 1, j0 + 1), fu);
    return lerp(top, bottom, fv);
}

/// Temperature in [0, 1]: `base` at the shore, falling by `kLapse` up to the highest peak. Under water it stays at base.
inline double temperature(double h, double base) {
    const double elevation = clamp01((h - kSeaLevel) / (1.0 - kSeaLevel));
    return clamp01(base - kLapse * elevation);
}

// ---------------------------------------------------------------------------------------------------------------
// Hillshade, after Esri "How Hillshade works". Heights are the 3x3 neighborhood a b c / d e f / g h i, row 0 is north.

inline double landHeight(double h) { return std::max(h, kSeaLevel); }

struct Gradient {
    double dzdx, dzdy;
};

/// Horn's 3x3 gradient: the weighted difference of the east and west columns (and south and north rows).
inline Gradient horn(const double z[9], double cell) {
    return {((z[2] + 2.0 * z[5] + z[8]) - (z[0] + 2.0 * z[3] + z[6])) / (8.0 * cell),
            ((z[6] + 2.0 * z[7] + z[8]) - (z[0] + 2.0 * z[1] + z[2])) / (8.0 * cell)};
}

inline double slopeOf(Gradient g, double zFactor) { return std::atan(zFactor * std::hypot(g.dzdx, g.dzdy)); }

/// Direction the surface faces (downhill), as a math angle: 0 = east, pi/2 = north, pi = west.
inline double aspectOf(Gradient g) { return std::atan2(g.dzdy, -g.dzdx); }

inline double degToRad(double d) { return d * kPi / 180.0; }

/// Hillshade of flat ground, which only depends on the sun's altitude: cos(zenith).
inline double flatShade(double altitudeDeg) { return std::cos(degToRad(90.0 - altitudeDeg)); }

/// 0..1 brightness of a surface, 255 * (cos(zenith) cos(slope) + sin(zenith) sin(slope) cos(azimuth - aspect)) / 255.
/// The azimuth is a compass angle (0 = north, 90 = east) and is turned into a math angle first.
inline double shade(double slope, double aspect, double azimuthDeg, double altitudeDeg) {
    const double zenith = degToRad(90.0 - altitudeDeg);
    const double azimuthMath = degToRad(360.0 - azimuthDeg + 90.0);
    const double v = std::cos(zenith) * std::cos(slope) + std::sin(zenith) * std::sin(slope) * std::cos(azimuthMath - aspect);
    return clamp01(v);
}

/// Brightness multiplier for a color: 1 on flat ground whatever the sun's altitude, above 1 on lit slopes.
inline double shadeFactor(double shadeValue, double altitudeDeg) {
    return std::clamp(1.0 + kShadeGain * (shadeValue - flatShade(altitudeDeg)), kShadeMin, kShadeMax);
}

// ---------------------------------------------------------------------------------------------------------------
// The world: two noise maps plus the slope and aspect of the terrain.

struct World {
    int w = 0;
    int h = 0;
    std::uint32_t seed = 0;
    std::vector<float> height;
    std::vector<float> moisture;
    std::vector<float> slope;
    std::vector<float> aspect;
};

/// Slope and aspect of every cell from `height`. The sea is flat (heights below the sea level count as the sea level)
/// and a neighbor outside the map is replaced by the edge cell.
inline void computeRelief(World& world) {
    const std::size_t n = static_cast<std::size_t>(world.w) * static_cast<std::size_t>(world.h);
    world.slope.assign(n, 0.0F);
    world.aspect.assign(n, 0.0F);
    for (int y = 0; y < world.h; ++y) {
        for (int x = 0; x < world.w; ++x) {
            double z[9];
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    const int sx = std::clamp(x + dx, 0, world.w - 1);
                    const int sy = std::clamp(y + dy, 0, world.h - 1);
                    z[(dy + 1) * 3 + (dx + 1)] = landHeight(world.height[static_cast<std::size_t>(sy * world.w + sx)]);
                }
            }
            const Gradient g = horn(z, 1.0);
            const auto i = static_cast<std::size_t>(y * world.w + x);
            world.slope[i] = static_cast<float>(slopeOf(g, kZFactor));
            world.aspect[i] = static_cast<float>(aspectOf(g));
        }
    }
}

/// Height map and an independent moisture map from two seeds, stretched so both use the whole range 0..1.
inline void generate(World& world, int width, int height, std::uint32_t seed) {
    world.w = width;
    world.h = height;
    world.seed = seed;
    const std::size_t n = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    world.height.assign(n, 0.0F);
    world.moisture.assign(n, 0.0F);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const double nx = (x + 0.5) / height;
            const double ny = (y + 0.5) / height;
            const double hRaw = fbm(nx * 3.0 + 17.3, ny * 3.0 + 9.1, seed, 5);
            const double mRaw = fbm(nx * 2.4 + 41.7, ny * 2.4 + 3.9, seed ^ 0x9E3779B9U, 4);
            const auto i = static_cast<std::size_t>(y * width + x);
            world.height[i] = static_cast<float>(clamp01((hRaw - 0.5) * 1.9 + 0.5));
            world.moisture[i] = static_cast<float>(clamp01((mRaw - 0.5) * 2.2 + 0.5));
        }
    }
    computeRelief(world);
}

// ---------------------------------------------------------------------------------------------------------------
// Classification

enum class Classify { HeightOnly, HeightMoisture };

struct Params {
    Classify mode = Classify::HeightMoisture;
    bool soft = true;            // false: hard edges (width 0)
    double width = 0.7;          // transition width in 0..1 when soft
    double moistureOffset = 0.0;  // shifts the whole moisture map wetter or drier
    double tempBase = 0.85;       // temperature at the shore
};

inline double effectiveWidth(const Params& p) { return p.soft ? p.width : 0.0; }

/// Moisture that is really looked up: the map plus the offset, or a constant 0.5 when only the height counts.
inline double effectiveMoisture(double m, const Params& p) { return clamp01((p.mode == Classify::HeightOnly ? 0.5 : m) + p.moistureOffset); }

/// Table cell of a land cell, -1 for water.
inline int cellIndex(double h, double m, const Params& p) {
    if (h < kSeaLevel) return -1;
    return nearestCell(effectiveMoisture(m, p), temperature(h, p.tempBase));
}

/// Color of a cell without lighting: water by depth, a sand band at the shore, otherwise the table.
inline Rgb cellColor(double h, double m, const Params& p) {
    if (h < kSeaLevel) {
        const double depth = (kSeaLevel - h) / kSeaLevel;
        return lerp(kShallow, kDeep, static_cast<float>(smoothstep(depth * 2.0)));
    }
    const Rgb land = biomeColor(effectiveMoisture(m, p), temperature(h, p.tempBase), effectiveWidth(p));
    const double s = (h - kSeaLevel) / kShoreWidth;
    const double sand = p.soft ? 1.0 - smoothstep(s) : (s < 1.0 ? 1.0 : 0.0);
    return lerp(land, kSand, static_cast<float>(sand));
}

/// The land cell closest to (cx, cy), used for a probe that starts on land. Falls back to (cx, cy) on a map without land.
inline std::size_t nearestLandCell(const World& world, int cx, int cy, double minHeight) {
    std::size_t best = static_cast<std::size_t>(std::clamp(cy, 0, std::max(world.h - 1, 0)) * world.w + std::clamp(cx, 0, std::max(world.w - 1, 0)));
    double bestDist = 1e18;
    for (int y = 0; y < world.h; ++y) {
        for (int x = 0; x < world.w; ++x) {
            const auto i = static_cast<std::size_t>(y * world.w + x);
            if (static_cast<double>(world.height[i]) < minHeight) continue;
            const double d = double(x - cx) * (x - cx) + double(y - cy) * (y - cy);
            if (d < bestDist) {
                bestDist = d;
                best = i;
            }
        }
    }
    return best;
}

inline int popcount16(std::uint16_t mask) {
    int n = 0;
    for (; mask != 0; mask = static_cast<std::uint16_t>(mask >> 1)) n += mask & 1U;
    return n;
}

/// How many of the 16 table cells show up on the land of this map.
inline int countBiomes(const World& world, const Params& p) {
    std::uint16_t mask = 0;
    for (std::size_t i = 0; i < world.height.size(); ++i) {
        const int c = cellIndex(world.height[i], world.moisture[i], p);
        if (c >= 0) mask = static_cast<std::uint16_t>(mask | (1U << c));
    }
    return popcount16(mask);
}

}  // namespace bio
