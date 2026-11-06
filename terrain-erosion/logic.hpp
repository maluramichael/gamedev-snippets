// Terrain erosion: the logic of the demo, no SDL, so test.cpp can check it.
//
// A heightmap from noise is smooth everywhere. Real mountains are not: water has carved valleys into them and
// rock has slid down the steep parts. Two small simulations bring that back.
//
// HYDRAULIC (drops). Many drops, one at a time. A drop starts at a random point and rolls downhill:
//   direction = old direction * inertia - gradient * (1 - inertia)       inertia makes it carry on over small bumps
//   capacity  = max(-dh, minSlope) * speed * water * capacityFactor      how much sediment it can carry
// Per step it compares what it carries with that capacity. Carrying less: it digs, but never deeper than the step
// it just walked down. Carrying more, or walking uphill: it puts material down. Material taken from the ground goes
// into the drop and material put down comes out of the drop, so nothing is lost (the test checks the sum). When the
// map ends or the steps are used up, the rest of its load settles where it is.
// NAIVE drop (the mistake): digs the same small amount in every step, never puts anything down, stops dead as soon
// as the ground rises. It makes trenches, leaves the hollows empty and the material is gone.
//
// THERMAL (talus). Loose material cannot lie steeper than its talus angle. Every cell compares itself with its
// eight neighbours; where the slope is steeper than allowed, it hands over part of the excess, shared out among
// those neighbours. The deltas are collected first and applied afterwards, so the order of the cells does not matter.
//
// Heights are stored in 0..1. kRelief says how many cells high 1.0 is; it only matters where slopes in degrees
// are computed.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace te {

constexpr double kRelief = 60.0;      // a height of 1.0 stands for this many cells (vertical exaggeration of slopes)
constexpr int kBrushRadius = 3;       // digging is spread over a disc of this radius, a single cell would carve needles
constexpr double kNaiveCut = 0.02;    // what the naive drop digs per step, times the erosion setting
constexpr int kMaxOctaves = 8;
constexpr int kMaxSteps = 200;        // upper bound for the life of a drop
constexpr double kTwoPi = 6.283185307179586;
constexpr double kPi = 3.141592653589793;
constexpr double kEpsilon = 1e-12;

struct Vec2 {
    double x, y;
};

/// A rectangular grid of doubles, row-major.
struct Grid {
    int w = 0;
    int h = 0;
    std::vector<double> v;

    Grid() = default;
    Grid(int width, int height) : w(std::max(0, width)), h(std::max(0, height)), v(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), 0.0) {}

    double& at(int x, int y) { return v[static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)]; }
    double at(int x, int y) const { return v[static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)]; }
    bool inside(int x, int y) const { return x >= 0 && y >= 0 && x < w && y < h; }
};

inline double totalHeight(const Grid& g) {
    double sum = 0.0;
    for (double v : g.v) sum += v;
    return sum;
}

/// NaN becomes `fallback`, everything else is clamped to [lo, hi].
inline double clampSane(double v, double lo, double hi, double fallback) {
    if (!(v == v)) return fallback;
    return std::clamp(v, lo, hi);
}

// ---------------------------------------------------------------- random numbers

/// splitmix64, small and good enough for scattering drops. Same seed, same sequence on every platform.
struct Rng {
    std::uint64_t s;
    explicit Rng(std::uint64_t seed) : s(seed) {}

    std::uint64_t nextBits() {
        s += 0x9e3779b97f4a7c15ULL;
        std::uint64_t z = s;
        z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
        z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
        return z ^ (z >> 31);
    }

    /// In [0, 1).
    double next() { return static_cast<double>(nextBits() >> 11) / 9007199254740992.0; }
};

// ---------------------------------------------------------------- noise

inline std::uint32_t latticeHash(int x, int y, int seed) {
    std::uint32_t h = static_cast<std::uint32_t>(x) * 0x8da6b343U ^ static_cast<std::uint32_t>(y) * 0xd8163841U ^ static_cast<std::uint32_t>(seed) * 0xcb1ab31fU;
    h += 0x9e3779b9U;
    h ^= h >> 16;
    h *= 0x85ebca6bU;
    h ^= h >> 13;
    h *= 0xc2b2ae35U;
    h ^= h >> 16;
    return h;
}

inline double fade(double t) { return t * t * t * (t * (t * 6.0 - 15.0) + 10.0); }
inline double lerp(double a, double b, double t) { return a + (b - a) * t; }

/// Gradient noise in [-1, 1], exactly 0 on the lattice points. Non-finite and huge inputs are clamped.
inline double gradientNoise(double x, double y, int seed) {
    x = clampSane(x, -1e9, 1e9, 0.0);
    y = clampSane(y, -1e9, 1e9, 0.0);
    const double fx0 = std::floor(x);
    const double fy0 = std::floor(y);
    const int ix = static_cast<int>(fx0);
    const int iy = static_cast<int>(fy0);
    const double fx = x - fx0;
    const double fy = y - fy0;
    const auto dot = [&](int cx, int cy, double dx, double dy) {
        const double a = static_cast<double>(latticeHash(cx, cy, seed) >> 8) / 16777216.0 * kTwoPi;
        return std::cos(a) * dx + std::sin(a) * dy;
    };
    const double u = fade(fx);
    const double v = fade(fy);
    const double top = lerp(dot(ix, iy, fx, fy), dot(ix + 1, iy, fx - 1.0, fy), u);
    const double bottom = lerp(dot(ix, iy + 1, fx, fy - 1.0), dot(ix + 1, iy + 1, fx - 1.0, fy - 1.0), u);
    return std::clamp(lerp(top, bottom, v) * 1.4142135623730951, -1.0, 1.0);  // unit gradients reach sqrt(2)/2 at most
}

/// Sum of octaves (each twice the frequency and `persistence` times the amplitude of the one before), divided by the
/// sum of the amplitudes, so the result stays in [0, 1]. (x, y) in unit coordinates.
inline double fbm(double x, double y, int seed, int octaves, double frequency, double persistence = 0.5) {
    const int n = std::clamp(octaves, 1, kMaxOctaves);
    const double decay = clampSane(persistence, 0.0, 1.0, 0.5);
    double sum = 0.0;
    double norm = 0.0;
    double amplitude = 1.0;
    double f = frequency;
    for (int i = 0; i < n; ++i) {
        sum += amplitude * (0.5 + 0.5 * gradientNoise(x * f, y * f, seed + i * 1013));  // another seed per octave
        norm += amplitude;
        amplitude *= decay;
        f *= 2.0;
    }
    return sum / norm;
}

/// Stretches the grid to exactly 0..1. A constant grid becomes all 0.
inline void normalizeToUnit(Grid& g) {
    if (g.v.empty()) return;
    const auto [lo, hi] = std::minmax_element(g.v.begin(), g.v.end());
    const double low = *lo;
    const double span = *hi - low;
    for (double& v : g.v) v = span > kEpsilon ? (v - low) / span : 0.0;
}

/// The smooth starting terrain: fbm of gradient noise, stretched to the full range.
inline void generateTerrain(Grid& g, int seed) {
    constexpr int kOctaves = 5;
    constexpr double kFrequency = 2.0;
    constexpr double kPersistence = 0.45;  // a little less fine detail than 0.5, so the valleys have room to show
    for (int y = 0; y < g.h; ++y) {
        for (int x = 0; x < g.w; ++x) {
            g.at(x, y) = fbm(static_cast<double>(x) / g.w, static_cast<double>(y) / g.h, seed, kOctaves, kFrequency, kPersistence);
        }
    }
    normalizeToUnit(g);
}

// ---------------------------------------------------------------- sampling and spreading

struct Sample {
    double h, gx, gy;
};

/// Bilinear height and gradient at (x, y). The position is clamped so that the whole cell lies on the grid.
inline Sample sampleAt(const Grid& g, double x, double y) {
    if (g.w < 2 || g.h < 2) return {0.0, 0.0, 0.0};
    x = clampSane(x, 0.0, g.w - 1.0 - 1e-9, 0.0);
    y = clampSane(y, 0.0, g.h - 1.0 - 1e-9, 0.0);
    const int ix = static_cast<int>(x);
    const int iy = static_cast<int>(y);
    const double fx = x - ix;
    const double fy = y - iy;
    const double h00 = g.at(ix, iy);
    const double h10 = g.at(ix + 1, iy);
    const double h01 = g.at(ix, iy + 1);
    const double h11 = g.at(ix + 1, iy + 1);
    return {h00 * (1.0 - fx) * (1.0 - fy) + h10 * fx * (1.0 - fy) + h01 * (1.0 - fx) * fy + h11 * fx * fy, (h10 - h00) * (1.0 - fy) + (h11 - h01) * fy,
            (h01 - h00) * (1.0 - fx) + (h11 - h10) * fx};
}

/// Adds `amount` to the four cells around (x, y), weighted by distance. Positions outside are clamped onto the grid.
inline void addBilinear(Grid& g, double x, double y, double amount) {
    if (g.w == 0 || g.h == 0) return;
    x = clampSane(x, 0.0, g.w - 1.0, 0.0);
    y = clampSane(y, 0.0, g.h - 1.0, 0.0);
    const int x0 = std::min(static_cast<int>(x), g.w - 1);
    const int y0 = std::min(static_cast<int>(y), g.h - 1);
    const int x1 = std::min(x0 + 1, g.w - 1);
    const int y1 = std::min(y0 + 1, g.h - 1);
    const double fx = x - x0;
    const double fy = y - y0;
    g.at(x0, y0) += amount * (1.0 - fx) * (1.0 - fy);
    g.at(x1, y0) += amount * fx * (1.0 - fy);
    g.at(x0, y1) += amount * (1.0 - fx) * fy;
    g.at(x1, y1) += amount * fx * fy;
}

/// Adds `amount` (negative digs) around (x, y), weight max(0, 1 - d / radius) per cell. The weights are normalized
/// over the cells that exist, so the full amount is handed out even at the edge of the map.
inline void addBrush(Grid& g, double x, double y, double amount) {
    if (g.w == 0 || g.h == 0) return;
    x = clampSane(x, 0.0, g.w - 1.0, 0.0);
    y = clampSane(y, 0.0, g.h - 1.0, 0.0);
    const int xa = std::max(0, static_cast<int>(std::floor(x)) - kBrushRadius);
    const int xb = std::min(g.w - 1, static_cast<int>(std::ceil(x)) + kBrushRadius);
    const int ya = std::max(0, static_cast<int>(std::floor(y)) - kBrushRadius);
    const int yb = std::min(g.h - 1, static_cast<int>(std::ceil(y)) + kBrushRadius);
    // Window of at most (2 * radius + 2) cells per side: weights are worked out once and used twice.
    constexpr int kSide = 2 * kBrushRadius + 2;
    double weights[kSide][kSide];
    double total = 0.0;
    for (int cy = ya; cy <= yb; ++cy) {
        for (int cx = xa; cx <= xb; ++cx) {
            const double ddx = cx - x;
            const double ddy = cy - y;
            const double w = std::max(0.0, 1.0 - std::sqrt(ddx * ddx + ddy * ddy) / kBrushRadius);
            weights[cy - ya][cx - xa] = w;
            total += w;
        }
    }
    if (total < kEpsilon) {
        addBilinear(g, x, y, amount);
        return;
    }
    for (int cy = ya; cy <= yb; ++cy) {
        for (int cx = xa; cx <= xb; ++cx) g.at(cx, cy) += amount * weights[cy - ya][cx - xa] / total;
    }
}

// ---------------------------------------------------------------- one drop

enum class Mode { Naive, Sediment };

struct DropParams {
    Mode mode = Mode::Sediment;
    double inertia = 0.1;    // 0 = follows the gradient exactly, 1 = never turns
    double capacity = 4.0;   // sediment capacity factor
    double erode = 0.3;      // share of the free capacity that is dug up per step
    double deposit = 0.3;    // share of the excess load that is put down per step
    double evaporate = 0.01; // share of the water that is gone after each step
    double gravity = 4.0;    // height lost turns into speed
    double minSlope = 0.01;  // on flat ground a drop still has a little capacity
    int maxSteps = 40;
};

struct DropResult {
    int steps = 0;
};

struct Change {
    double erode = 0.0;
    double deposit = 0.0;
};

inline double capacityOf(double dh, double speed, double water, const DropParams& p) {
    const double slope = std::max(-dh, p.minSlope);
    return std::max(0.0, slope * speed * water * p.capacity);
}

/// What one step does: dig up or put down (never both). `dh` is the height change of the step (negative = downhill).
inline Change stepChange(double sediment, double capacity, double dh, const DropParams& p) {
    const double depositRate = clampSane(p.deposit, 0.0, 1.0, 0.0);
    const double erodeRate = clampSane(p.erode, 0.0, 1.0, 0.0);
    if (dh > 0.0) return {0.0, std::min(dh, sediment)};  // uphill: fill the step, but never more than it carries
    if (sediment > capacity) return {0.0, (sediment - capacity) * depositRate};
    return {std::max(0.0, std::min((capacity - sediment) * erodeRate, -dh)), 0.0};  // never dig deeper than the step
}

/// Runs one drop from (x, y). With `path` the visited positions are recorded (start included). The sum of all heights
/// stays the same in Sediment mode; Naive mode removes material.
inline DropResult runDrop(Grid& g, double x, double y, Rng& rng, const DropParams& params, std::vector<Vec2>* path) {
    DropResult result;
    if (path != nullptr) path->clear();
    if (g.w < 2 || g.h < 2) return result;
    DropParams p = params;
    p.inertia = clampSane(p.inertia, 0.0, 1.0, 0.0);
    p.evaporate = clampSane(p.evaporate, 0.0, 1.0, 0.0);
    p.capacity = clampSane(p.capacity, 0.0, 1e6, 0.0);
    p.gravity = clampSane(p.gravity, 0.0, 1e6, 0.0);
    p.minSlope = clampSane(p.minSlope, 0.0, 1.0, 0.0);
    const int maxSteps = std::clamp(p.maxSteps, 0, kMaxSteps);
    const double xMax = g.w - 1.0 - 1e-9;
    const double yMax = g.h - 1.0 - 1e-9;
    x = clampSane(x, 0.0, xMax, 0.0);
    y = clampSane(y, 0.0, yMax, 0.0);
    double dx = 0.0;
    double dy = 0.0;
    double speed = 1.0;
    double water = 1.0;
    double sediment = 0.0;
    if (path != nullptr) path->push_back({x, y});

    for (int step = 0; step < maxSteps; ++step) {
        const Sample here = sampleAt(g, x, y);
        dx = dx * p.inertia - here.gx * (1.0 - p.inertia);
        dy = dy * p.inertia - here.gy * (1.0 - p.inertia);
        const double len = std::hypot(dx, dy);
        if (len < kEpsilon) {  // flat ground, no direction: pick one
            const double a = rng.next() * kTwoPi;
            dx = std::cos(a);
            dy = std::sin(a);
        } else {
            dx /= len;
            dy /= len;
        }
        const double nx = x + dx;
        const double ny = y + dy;
        if (!(nx >= 0.0 && nx <= xMax && ny >= 0.0 && ny <= yMax)) break;  // left the map
        const double dh = sampleAt(g, nx, ny).h - here.h;

        if (p.mode == Mode::Naive) {
            if (dh > 0.0) break;  // the ground rises: the naive drop is stuck in its hollow
            addBrush(g, x, y, -clampSane(p.erode, 0.0, 1.0, 0.0) * kNaiveCut);
        } else {
            const Change c = stepChange(sediment, capacityOf(dh, speed, water, p), dh, p);
            if (c.erode > 0.0) addBrush(g, x, y, -c.erode);
            if (c.deposit > 0.0) addBilinear(g, x, y, c.deposit);
            sediment += c.erode - c.deposit;
        }
        speed = std::sqrt(std::max(0.0, speed * speed - dh * p.gravity));
        water *= 1.0 - p.evaporate;
        x = nx;
        y = ny;
        ++result.steps;
        if (path != nullptr) path->push_back({x, y});
    }
    if (p.mode == Mode::Sediment && sediment > 0.0) addBilinear(g, x, y, sediment);  // the water is gone, the load stays
    return result;
}

// ---------------------------------------------------------------- thermal erosion

struct ThermalParams {
    double talusDeg = 40.0;   // steepest slope loose material can keep
    double relief = kRelief;  // how many cells high 1.0 is
    double rate = 0.5;        // share of the largest excess that is moved per pass
};

/// One pass over the grid; returns the amount of material that was moved. `delta` is scratch space (reused).
inline double thermalPass(Grid& g, std::vector<double>& delta, const ThermalParams& p) {
    if (g.v.empty()) return 0.0;
    const double angle = clampSane(p.talusDeg, 1.0, 89.0, 40.0) * kPi / 180.0;
    const double relief = std::max(p.relief, kEpsilon);
    const double allowed = std::tan(angle) / relief;  // height difference per cell of distance
    const double rate = clampSane(p.rate, 0.0, 1.0, 0.5);
    delta.assign(g.v.size(), 0.0);
    double moved = 0.0;
    for (int y = 0; y < g.h; ++y) {
        for (int x = 0; x < g.w; ++x) {
            const double here = g.at(x, y);
            double excess[8] = {};
            int nbX[8] = {};
            int nbY[8] = {};
            int count = 0;
            double sum = 0.0;
            double largest = 0.0;
            for (int oy = -1; oy <= 1; ++oy) {
                for (int ox = -1; ox <= 1; ++ox) {
                    if ((ox == 0 && oy == 0) || !g.inside(x + ox, y + oy)) continue;
                    const double dist = (ox != 0 && oy != 0) ? 1.4142135623730951 : 1.0;
                    const double ex = here - g.at(x + ox, y + oy) - allowed * dist;
                    if (ex > 1e-9) {
                        excess[count] = ex;
                        nbX[count] = x + ox;
                        nbY[count] = y + oy;
                        ++count;
                        sum += ex;
                        largest = std::max(largest, ex);
                    }
                }
            }
            if (count == 0) continue;
            const double give = rate * largest;
            for (int i = 0; i < count; ++i) {
                const double share = give * excess[i] / sum;
                delta[static_cast<std::size_t>(nbY[i]) * static_cast<std::size_t>(g.w) + static_cast<std::size_t>(nbX[i])] += share;
                delta[static_cast<std::size_t>(y) * static_cast<std::size_t>(g.w) + static_cast<std::size_t>(x)] -= share;
            }
            moved += give;
        }
    }
    for (std::size_t i = 0; i < g.v.size(); ++i) g.v[i] += delta[i];
    return moved;
}

// ---------------------------------------------------------------- statistics

/// Mean slope in degrees over the interior cells (central differences).
inline double meanSlopeDeg(const Grid& g, double relief) {
    if (g.w < 3 || g.h < 3) return 0.0;
    double sum = 0.0;
    for (int y = 1; y < g.h - 1; ++y) {
        for (int x = 1; x < g.w - 1; ++x) {
            const double gx = (g.at(x + 1, y) - g.at(x - 1, y)) * 0.5 * relief;
            const double gy = (g.at(x, y + 1) - g.at(x, y - 1)) * 0.5 * relief;
            sum += std::atan(std::hypot(gx, gy));
        }
    }
    return sum / static_cast<double>((g.w - 2) * (g.h - 2)) * 180.0 / kPi;
}

/// The steepest slope between any cell and one of its eight neighbours, in degrees (the angle thermal erosion limits).
inline double steepestDeg(const Grid& g, double relief) {
    double best = 0.0;
    for (int y = 0; y < g.h; ++y) {
        for (int x = 0; x < g.w; ++x) {
            for (int oy = -1; oy <= 1; ++oy) {
                for (int ox = -1; ox <= 1; ++ox) {
                    if ((ox == 0 && oy == 0) || !g.inside(x + ox, y + oy)) continue;
                    const double dist = (ox != 0 && oy != 0) ? 1.4142135623730951 : 1.0;
                    best = std::max(best, std::fabs(g.at(x, y) - g.at(x + ox, y + oy)) * relief / dist);
                }
            }
        }
    }
    return std::atan(best) * 180.0 / kPi;
}

// ---------------------------------------------------------------- colors

struct Rgb {
    std::uint8_t r, g, b;
};

inline constexpr Rgb kOrangeRgb{234, 88, 12};
inline constexpr Rgb kTealRgb{13, 148, 136};

inline std::uint8_t channel(double v) { return static_cast<std::uint8_t>(std::clamp(std::lround(v), 0L, 255L)); }

/// Lowland, hills, rock, snow. Linear between the stops, clamped outside [0, 1].
inline Rgb terrainColor(double h) {
    struct Stop {
        double h, r, g, b;
    };
    static constexpr Stop kRamp[] = {{0.00, 70, 112, 78}, {0.25, 116, 154, 84}, {0.50, 168, 156, 108}, {0.72, 132, 122, 112}, {0.88, 206, 204, 202}, {1.00, 250, 250, 250}};
    constexpr std::size_t kStops = sizeof kRamp / sizeof kRamp[0];
    h = clampSane(h, 0.0, 1.0, 0.0);
    for (std::size_t i = 1; i < kStops; ++i) {
        if (h <= kRamp[i].h) {
            const Stop& a = kRamp[i - 1];
            const Stop& b = kRamp[i];
            const double t = (h - a.h) / (b.h - a.h);  // the stops are strictly increasing
            return {channel(lerp(a.r, b.r, t)), channel(lerp(a.g, b.g, t)), channel(lerp(a.b, b.b, t))};
        }
    }
    return {channel(kRamp[kStops - 1].r), channel(kRamp[kStops - 1].g), channel(kRamp[kStops - 1].b)};
}

/// Brightness from the slope with the light in the upper left: ground that rises towards the lower right faces the
/// light. gx and gy are slopes in world units (height per cell times relief).
inline double shadeFactor(double gx, double gy) {
    if (!(gx == gx) || !(gy == gy)) return 1.0;
    return std::clamp(1.0 + 0.4 * (gx + gy) * 0.5, 0.45, 1.5);
}

inline Rgb shade(Rgb c, double f) { return {channel(c.r * f), channel(c.g * f), channel(c.b * f)}; }

/// True when a contour line runs between two neighbouring heights: floor(a / interval) differs from floor(b / interval).
inline bool isContour(double a, double b, double interval) {
    if (!(interval > 0.0) || !(a == a) || !(b == b)) return false;
    return std::floor(a / interval) != std::floor(b / interval);
}

/// Tints `base` towards orange where material was taken (delta < 0) and towards teal where material was put down.
/// `scale` is the change that gets the full color.
inline Rgb diffTint(Rgb base, double delta, double scale) {
    if (!(delta == delta) || delta == 0.0) return base;
    const double t = scale > 0.0 ? std::min(std::fabs(delta) / scale, 1.0) : 1.0;
    const Rgb target = delta < 0.0 ? kOrangeRgb : kTealRgb;
    return {channel(lerp(base.r, target.r, t)), channel(lerp(base.g, target.g, t)), channel(lerp(base.b, target.b, t))};
}

// ---------------------------------------------------------------- numbers for the screen

/// 1234567 -> "1,234,567". Built by hand: snprintf separators depend on the locale.
inline std::string groupDigits(long long n) {
    const bool negative = n < 0;
    std::string digits = std::to_string(n);
    if (negative) digits.erase(0, 1);
    std::string out;
    for (std::size_t i = 0; i < digits.size(); ++i) {
        if (i > 0 && (digits.size() - i) % 3 == 0) out += ',';
        out += digits[i];
    }
    return negative ? "-" + out : out;
}

/// Slider position t in [0, 1] to a value, snapped to `step` (0 = continuous).
inline double sliderValue(double t, double lo, double hi, double step) {
    t = clampSane(t, 0.0, 1.0, 0.0);
    double v = lo + t * (hi - lo);
    if (step > 0.0) v = lo + std::round((v - lo) / step) * step;
    return std::clamp(v, std::min(lo, hi), std::max(lo, hi));
}

inline double tOfValue(double v, double lo, double hi) { return hi > lo ? std::clamp((v - lo) / (hi - lo), 0.0, 1.0) : 0.0; }

}  // namespace te
