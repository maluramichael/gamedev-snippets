// The logic of terrain-rivers, no SDL: a noise island, depression filling (Priority-Flood), D8 flow directions and
// flow accumulation. Rivers are the cells where enough rain has collected, lakes are the cells the fill raised.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <queue>
#include <utility>
#include <vector>

namespace rv {

inline constexpr double kEps = 1e-6;         // how much higher than the cell it drains into a flooded cell is put
inline constexpr double kLakeDepth = 1e-5;   // a cell counts as lake when the fill raised it by more than this
inline constexpr double kDefaultSea = 0.30;
inline constexpr double kSeaMin = 0.10;
inline constexpr double kSeaMax = 0.50;
inline constexpr int kThresholdMin = 4;
inline constexpr int kThresholdMax = 400;
inline constexpr double kPi = 3.14159265358979323846;

// The eight neighbors in scan order (row by row). The order matters: on a tie the first one wins.
inline constexpr std::array<int, 8> kDx{-1, 0, 1, -1, 1, -1, 0, 1};
inline constexpr std::array<int, 8> kDy{-1, -1, -1, 0, 0, 1, 1, 1};

/// Index 0..7 of the neighbor at offset (dx, dy), or -1 if that is not one of the eight.
inline int directionIndex(int dx, int dy) {
    for (int d = 0; d < 8; ++d) {
        if (kDx[static_cast<std::size_t>(d)] == dx && kDy[static_cast<std::size_t>(d)] == dy) return d;
    }
    return -1;
}

/// Distance to the neighbor: diagonals are sqrt(2) away, which is why a steeper-looking diagonal can still lose.
inline double stepLength(int d) {
    if (d < 0 || d > 7) return 1.0;
    return (kDx[static_cast<std::size_t>(d)] != 0 && kDy[static_cast<std::size_t>(d)] != 0) ? std::sqrt(2.0) : 1.0;
}

inline double clampSea(double v) { return std::clamp(v, kSeaMin, kSeaMax); }

// --- terrain -------------------------------------------------------------------------------------------------------

inline std::uint32_t hash2(int x, int y, std::uint32_t seed) {
    std::uint32_t h = seed * 374761393U + static_cast<std::uint32_t>(x) * 668265263U + static_cast<std::uint32_t>(y) * 2246822519U;
    h ^= h >> 13;
    h *= 1274126177U;
    h ^= h >> 16;
    return h;
}

inline double fade(double t) { return t * t * t * (t * (t * 6.0 - 15.0) + 10.0); }

/// Gradient noise, one random direction per lattice point. Range about -0.7 .. 0.7.
inline double gradientNoise(double x, double y, std::uint32_t seed) {
    const double fx = std::floor(x);
    const double fy = std::floor(y);
    const int ix = static_cast<int>(fx);
    const int iy = static_cast<int>(fy);
    const double tx = x - fx;
    const double ty = y - fy;
    const auto corner = [&](int cx, int cy, double ox, double oy) {
        const double angle = static_cast<double>(hash2(cx, cy, seed)) / 4294967296.0 * 2.0 * kPi;
        return std::cos(angle) * ox + std::sin(angle) * oy;
    };
    const double a = corner(ix, iy, tx, ty);
    const double b = corner(ix + 1, iy, tx - 1.0, ty);
    const double c = corner(ix, iy + 1, tx, ty - 1.0);
    const double d = corner(ix + 1, iy + 1, tx - 1.0, ty - 1.0);
    const double u = fade(tx);
    const double v = fade(ty);
    return (a + (b - a) * u) + ((c + (d - c) * u) - (a + (b - a) * u)) * v;
}

/// Four octaves of gradient noise, scaled into 0..1.
inline double fbm(double x, double y, std::uint32_t seed) {
    double sum = 0.0;
    double amp = 1.0;
    double total = 0.0;
    double freq = 1.0;
    for (int o = 0; o < 4; ++o) {
        sum += amp * gradientNoise(x * freq, y * freq, seed + static_cast<std::uint32_t>(o) * 101U);
        total += amp;
        amp *= 0.5;
        freq *= 2.0;
    }
    return std::clamp(sum / total / 0.7071 * 0.5 + 0.5, 0.0, 1.0);
}

inline double smoothstep(double a, double b, double x) {
    const double t = std::clamp((x - a) / (b - a), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

/// An island: noise plus a cone, faded to nothing at the border. Every border cell ends up below the default sea level.
inline std::vector<double> makeTerrain(int w, int h, unsigned seed) {
    if (w <= 0 || h <= 0) return {};
    std::vector<double> out(static_cast<std::size_t>(w) * static_cast<std::size_t>(h));
    const double cx = w / 2.0;
    const double cy = h / 2.0;
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            const double dx = (x + 0.5 - cx) / cx;
            const double dy = (y + 0.5 - cy) / cy;
            const double d = std::sqrt(dx * dx + dy * dy);
            const double n = fbm(x * 0.035 + 11.3, y * 0.035 + 7.9, seed);
            const double cone = 1.0 - std::min(d, 1.0);
            const double edge = 1.0 - smoothstep(0.86, 0.99, d);
            out[static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)] =
                std::clamp((0.55 * n + 0.55 * cone - 0.03) * edge, 0.0, 1.0);
        }
    }
    return out;
}

// --- water ---------------------------------------------------------------------------------------------------------

/// Cells where water leaves the map: everything at or below sea level (also inland) and the border of the grid.
inline std::vector<char> outletMask(const std::vector<double>& heights, int w, int h, double sea) {
    std::vector<char> out(heights.size(), 0);
    if (w <= 0 || h <= 0 || heights.size() != static_cast<std::size_t>(w) * static_cast<std::size_t>(h)) return out;
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            const std::size_t i = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
            out[i] = (heights[i] <= sea || x == 0 || y == 0 || x == w - 1 || y == h - 1) ? 1 : 0;
        }
    }
    return out;
}

/// Priority-Flood: start at the outlets and always take the lowest cell of the frontier. A cell found from a cell at
/// level L cannot be lower than L, otherwise water could not leave it, so it is raised to L plus a tiny eps. The eps
/// gives every flooded cell a strictly lower neighbor (the one it was found from), so no flat area is left without a
/// direction. Cells that already drain downhill keep their height.
inline std::vector<double> fillSinks(const std::vector<double>& heights, int w, int h, const std::vector<char>& outlet, double eps) {
    std::vector<double> filled = heights;
    const std::size_t count = static_cast<std::size_t>(std::max(w, 0)) * static_cast<std::size_t>(std::max(h, 0));
    if (w <= 0 || h <= 0 || heights.size() != count || outlet.size() != count) return filled;
    struct Item {
        double key;
        std::uint64_t order;
        int index;
        bool operator>(const Item& o) const { return key != o.key ? key > o.key : order > o.order; }
    };
    std::priority_queue<Item, std::vector<Item>, std::greater<>> frontier;
    std::vector<char> closed(count, 0);
    std::uint64_t order = 0;
    for (std::size_t i = 0; i < count; ++i) {
        if (outlet[i] != 0) {
            closed[i] = 1;
            frontier.push({filled[i], order++, static_cast<int>(i)});
        }
    }
    while (!frontier.empty()) {
        const Item c = frontier.top();
        frontier.pop();
        const int cx = c.index % w;
        const int cy = c.index / w;
        for (int d = 0; d < 8; ++d) {
            const int nx = cx + kDx[static_cast<std::size_t>(d)];
            const int ny = cy + kDy[static_cast<std::size_t>(d)];
            if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
            const std::size_t n = static_cast<std::size_t>(ny) * static_cast<std::size_t>(w) + static_cast<std::size_t>(nx);
            if (closed[n] != 0) continue;
            closed[n] = 1;
            filled[n] = std::max(heights[n], filled[static_cast<std::size_t>(c.index)] + eps);
            frontier.push({filled[n], order++, static_cast<int>(n)});
        }
    }
    return filled;
}

/// Land cells the fill raised noticeably: the lakes.
inline std::vector<char> lakeMask(const std::vector<double>& heights, const std::vector<double>& filled, const std::vector<char>& outlet, double minDepth) {
    std::vector<char> out(heights.size(), 0);
    if (filled.size() != heights.size() || outlet.size() != heights.size()) return out;
    for (std::size_t i = 0; i < heights.size(); ++i) out[i] = (outlet[i] == 0 && filled[i] - heights[i] > minDepth) ? 1 : 0;
    return out;
}

/// Number of connected groups of set cells (diagonal neighbors connect).
inline int countComponents(const std::vector<char>& mask, int w, int h) {
    if (w <= 0 || h <= 0 || mask.size() != static_cast<std::size_t>(w) * static_cast<std::size_t>(h)) return 0;
    std::vector<char> seen(mask.size(), 0);
    std::vector<int> stack;
    int groups = 0;
    for (std::size_t start = 0; start < mask.size(); ++start) {
        if (mask[start] == 0 || seen[start] != 0) continue;
        ++groups;
        seen[start] = 1;
        stack.push_back(static_cast<int>(start));
        while (!stack.empty()) {
            const int c = stack.back();
            stack.pop_back();
            for (int d = 0; d < 8; ++d) {
                const int nx = c % w + kDx[static_cast<std::size_t>(d)];
                const int ny = c / w + kDy[static_cast<std::size_t>(d)];
                if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
                const std::size_t n = static_cast<std::size_t>(ny) * static_cast<std::size_t>(w) + static_cast<std::size_t>(nx);
                if (mask[n] == 0 || seen[n] != 0) continue;
                seen[n] = 1;
                stack.push_back(static_cast<int>(n));
            }
        }
    }
    return groups;
}

/// D8: every cell drains into the neighbor with the steepest drop (drop divided by distance). -1 for outlets and for
/// cells without a lower neighbor (pits). On a tie the first neighbor in scan order wins.
inline std::vector<int> flowDirections(const std::vector<double>& surface, int w, int h, const std::vector<char>& outlet) {
    std::vector<int> dir(surface.size(), -1);
    if (w <= 0 || h <= 0 || surface.size() != static_cast<std::size_t>(w) * static_cast<std::size_t>(h) || outlet.size() != surface.size()) return dir;
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            const std::size_t i = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
            if (outlet[i] != 0) continue;
            double best = 0.0;
            for (int d = 0; d < 8; ++d) {
                const int nx = x + kDx[static_cast<std::size_t>(d)];
                const int ny = y + kDy[static_cast<std::size_t>(d)];
                if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
                const double drop = surface[i] - surface[static_cast<std::size_t>(ny) * static_cast<std::size_t>(w) + static_cast<std::size_t>(nx)];
                if (drop <= 0.0) continue;
                const double slope = drop / stepLength(d);
                if (slope > best) {
                    best = slope;
                    dir[i] = d;
                }
            }
        }
    }
    return dir;
}

/// Land cells that are not outlets and have no way down.
inline int countPits(const std::vector<int>& dir, const std::vector<char>& outlet) {
    int pits = 0;
    for (std::size_t i = 0; i < dir.size() && i < outlet.size(); ++i) pits += (outlet[i] == 0 && dir[i] == -1) ? 1 : 0;
    return pits;
}

/// One unit of rain per land cell.
inline std::vector<int> rainOf(const std::vector<double>& heights, double sea) {
    std::vector<int> rain(heights.size(), 0);
    for (std::size_t i = 0; i < heights.size(); ++i) rain[i] = heights[i] > sea ? 1 : 0;
    return rain;
}

/// Flow accumulation: how many cells drain through each cell. Process the cells from high to low on the same surface
/// the directions were taken from, then every cell is final before it passes its total on.
inline std::vector<int> accumulate(const std::vector<double>& surface, int w, const std::vector<int>& dir, const std::vector<int>& rain) {
    std::vector<int> acc = rain;
    if (w <= 0 || dir.size() != surface.size() || rain.size() != surface.size()) return acc;
    std::vector<int> order(surface.size());
    for (std::size_t i = 0; i < order.size(); ++i) order[i] = static_cast<int>(i);
    std::stable_sort(order.begin(), order.end(), [&](int a, int b) { return surface[static_cast<std::size_t>(a)] > surface[static_cast<std::size_t>(b)]; });
    for (const int i : order) {
        const int d = dir[static_cast<std::size_t>(i)];
        if (d < 0) continue;
        const std::ptrdiff_t to = static_cast<std::ptrdiff_t>(i) + static_cast<std::ptrdiff_t>(kDy[static_cast<std::size_t>(d)]) * w + kDx[static_cast<std::size_t>(d)];
        acc[static_cast<std::size_t>(to)] += acc[static_cast<std::size_t>(i)];
    }
    return acc;
}

/// The cells a raindrop visits from `start` to where it stops (an outlet or a pit), both ends included.
inline std::vector<int> tracePath(const std::vector<int>& dir, int w, int start) {
    std::vector<int> path;
    if (w <= 0 || start < 0 || static_cast<std::size_t>(start) >= dir.size()) return path;
    int cur = start;
    while (path.size() <= dir.size()) {
        path.push_back(cur);
        const int d = dir[static_cast<std::size_t>(cur)];
        if (d < 0) break;
        cur += kDy[static_cast<std::size_t>(d)] * w + kDx[static_cast<std::size_t>(d)];
    }
    return path;
}

inline bool isRiver(int acc, int threshold) { return acc >= std::max(threshold, 1); }

/// Line width in px: 1 at the threshold, growing with every doubling of the catchment, capped at 4. 0 below the threshold.
inline double riverWidth(int acc, int threshold) {
    const int t = std::max(threshold, 1);
    if (acc < t) return 0.0;
    return std::min(1.0 + 0.75 * std::log2(static_cast<double>(acc) / t), 4.0);
}

struct World {
    int w = 0;
    int h = 0;
    double sea = kDefaultSea;
    std::vector<double> heights;
    std::vector<double> filled;
    std::vector<char> outlet;
    std::vector<char> lake;
    std::vector<int> rawDir, rawAcc;  // directions and accumulation on the terrain as it is
    std::vector<int> dir, acc;        // the same after filling the depressions
    int rawPits = 0;
    int lakeCount = 0;
    int lakeCells = 0;
    int rawMaxAcc = 0;
    int maxAcc = 0;
};

inline World buildWorld(int w, int h, std::vector<double> heights, double sea) {
    World world;
    world.w = w;
    world.h = h;
    world.sea = sea;
    world.heights = std::move(heights);
    world.outlet = outletMask(world.heights, w, h, sea);
    const std::vector<int> rain = rainOf(world.heights, sea);
    world.rawDir = flowDirections(world.heights, w, h, world.outlet);
    world.rawAcc = accumulate(world.heights, w, world.rawDir, rain);
    world.filled = fillSinks(world.heights, w, h, world.outlet, kEps);
    world.dir = flowDirections(world.filled, w, h, world.outlet);
    world.acc = accumulate(world.filled, w, world.dir, rain);
    world.lake = lakeMask(world.heights, world.filled, world.outlet, kLakeDepth);
    world.rawPits = countPits(world.rawDir, world.outlet);
    world.lakeCount = countComponents(world.lake, w, h);
    for (const char c : world.lake) world.lakeCells += c != 0 ? 1 : 0;
    for (const int a : world.rawAcc) world.rawMaxAcc = std::max(world.rawMaxAcc, a);
    for (const int a : world.acc) world.maxAcc = std::max(world.maxAcc, a);
    return world;
}

/// Rivers that stop in the middle of the land: pits that collected at least `threshold` cells.
inline int countDeadEnds(const std::vector<int>& dir, const std::vector<char>& outlet, const std::vector<int>& acc, int threshold) {
    int ends = 0;
    for (std::size_t i = 0; i < dir.size() && i < outlet.size() && i < acc.size(); ++i) {
        ends += (outlet[i] == 0 && dir[i] == -1 && isRiver(acc[i], threshold)) ? 1 : 0;
    }
    return ends;
}

/// The raindrop the demo starts with: the land cell whose path on the raw terrain is longest before it gets stuck in a
/// pit (first one on a tie). Without any pit the highest land cell. -1 if there is no land.
inline int pickDemoDrop(const World& world) {
    int best = -1;
    std::size_t bestLen = 0;
    int highest = -1;
    double highestHeight = -1.0;
    for (std::size_t i = 0; i < world.heights.size(); ++i) {
        if (world.outlet[i] != 0) continue;
        if (world.heights[i] > highestHeight) {
            highestHeight = world.heights[i];
            highest = static_cast<int>(i);
        }
        const std::vector<int> path = tracePath(world.rawDir, world.w, static_cast<int>(i));
        const auto end = static_cast<std::size_t>(path.back());
        if (world.outlet[end] == 0 && path.size() > bestLen) {
            bestLen = path.size();
            best = static_cast<int>(i);
        }
    }
    return best >= 0 ? best : highest;
}

// --- UI mapping ----------------------------------------------------------------------------------------------------

/// Threshold slider: geometric between 4 and 400 cells, because the interesting values are small.
inline int thresholdFromT(double t) {
    const double c = std::clamp(t, 0.0, 1.0);
    const double ratio = static_cast<double>(kThresholdMax) / kThresholdMin;
    return static_cast<int>(std::lround(kThresholdMin * std::pow(ratio, c)));
}

inline double tFromThreshold(int threshold) {
    const int c = std::clamp(threshold, kThresholdMin, kThresholdMax);
    const double ratio = static_cast<double>(kThresholdMax) / kThresholdMin;
    return std::log(static_cast<double>(c) / kThresholdMin) / std::log(ratio);
}

/// Linear slider value for t in 0..1, snapped to `step` (0 = no snapping).
inline double stepValue(double t, double lo, double hi, double step) {
    if (!(hi > lo)) return lo;
    double v = lo + std::clamp(t, 0.0, 1.0) * (hi - lo);
    if (step > 0.0) v = lo + std::round((v - lo) / step) * step;
    return std::clamp(v, lo, hi);
}

/// The cell under a screen point, if the point is on the map. Map drawn at (ox, oy), `cell` px per cell.
inline std::optional<int> cellAt(double x, double y, double ox, double oy, double cell, int w, int h) {
    if (!(cell > 0.0) || w <= 0 || h <= 0) return std::nullopt;
    const double fx = (x - ox) / cell;
    const double fy = (y - oy) / cell;
    if (!(fx >= 0.0 && fy >= 0.0 && fx < w && fy < h)) return std::nullopt;
    return static_cast<int>(std::floor(fy)) * w + static_cast<int>(std::floor(fx));
}

// --- colors --------------------------------------------------------------------------------------------------------

struct Rgb {
    std::uint8_t r, g, b;
};

inline Rgb mix(Rgb a, Rgb b, double t) {
    const double c = std::clamp(t, 0.0, 1.0);
    const auto m = [&](std::uint8_t p, std::uint8_t q) { return static_cast<std::uint8_t>(std::lround(p + (q - p) * c)); };
    return {m(a.r, b.r), m(a.g, b.g), m(a.b, b.b)};
}

/// Sea in two blues by depth, land from light green over brown to white tops.
inline Rgb terrainColor(double height, double sea) {
    const double hgt = std::clamp(height, 0.0, 1.0);
    if (hgt <= sea) {
        const double t = sea > 0.0 ? hgt / sea : 1.0;
        return mix({88, 150, 178}, {150, 205, 218}, t);
    }
    const double u = sea < 1.0 ? (hgt - sea) / (1.0 - sea) : 0.0;
    constexpr Rgb low{176, 206, 140};
    constexpr Rgb mid{140, 170, 108};
    constexpr Rgb high{178, 160, 118};
    constexpr Rgb rock{214, 206, 190};
    constexpr Rgb snow{250, 250, 248};
    if (u < 0.35) return mix(low, mid, u / 0.35);
    if (u < 0.60) return mix(mid, high, (u - 0.35) / 0.25);
    if (u < 0.85) return mix(high, rock, (u - 0.60) / 0.25);
    return mix(rock, snow, (u - 0.85) / 0.15);
}

/// Brightness factor from the slope, light from the north-west. Flat is 1. Edge cells use themselves for the missing neighbor.
inline double hillshade(const std::vector<double>& heights, int w, int h, int x, int y) {
    const auto at = [&](int px, int py) {
        px = std::clamp(px, 0, w - 1);
        py = std::clamp(py, 0, h - 1);
        return heights[static_cast<std::size_t>(py) * static_cast<std::size_t>(w) + static_cast<std::size_t>(px)];
    };
    const double dx = at(x + 1, y) - at(x - 1, y);
    const double dy = at(x, y + 1) - at(x, y - 1);
    return std::clamp(1.0 + 5.0 * (dx + dy), 0.6, 1.4);
}

}  // namespace rv
