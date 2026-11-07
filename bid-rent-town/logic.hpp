// The logic of the bid-rent town, no SDL. Every land use has a bid curve over the distance to the centre
// (a straight line, high near the centre, falling), and each cell takes the use whose bid is highest. Rings
// are what that maximum looks like on a map. The curve shapes and all numbers are made up for the demo.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

namespace brt {

enum class Use : unsigned char { Wild, Farm, House, Market, Water };
enum class Mode { BidCurves, HandSetRings };

constexpr std::size_t idx(Use u) { return static_cast<std::size_t>(u); }

constexpr double kNoWater = 1e9;       // "no water anywhere": distance used when the map has no river
constexpr double kMarketBase = 100.0;  // bid at the centre, times size
constexpr double kHouseBase = 60.0;    // bid at the centre, times size
constexpr double kFarmBase = 20.0;     // farmland does not care how big the town is
constexpr double kWaterBonus = 60.0;   // bid added to a market right at the water (house gets half)
constexpr double kWaterReach = 6.0;    // cells: the bonus fades to zero this far from the river
constexpr double kFixedMarketRadius = 5.0;  // hand-set rings, measured once at size 1 on the default curves
constexpr double kFixedHouseRadius = 12.0;

struct Params {
    double size = 1.0;  // town size: multiplies the house and market curves (more people, more demand)
    double slopeFarm = 0.6;
    double slopeHouse = 4.0;
    double slopeMarket = 12.0;
    double water = 0.6;  // 0..1, how much the river adds
};

struct Bids {
    double farm = 0.0;
    double house = 0.0;
    double market = 0.0;
};

/// Bids of the three uses at distance d (cells) from the centre and dw from the nearest water.
inline Bids bids(const Params& p, double d, double dw) {
    const double reach = dw >= kWaterReach ? 0.0 : 1.0 - dw / kWaterReach;  // 1 at the water, 0 at the edge of its reach
    const double bonus = kWaterBonus * p.water * std::max(0.0, reach);
    return {kFarmBase - p.slopeFarm * d, kHouseBase * p.size - p.slopeHouse * d + 0.5 * bonus,
            kMarketBase * p.size - p.slopeMarket * d + bonus};
}

/// The maximum wins. A tie goes to the denser use (market, then house), so a cell never flickers between two.
/// A best bid of zero or less is not worth building on.
inline Use pick(const Bids& b) {
    Use best = Use::Market;
    double top = b.market;
    if (b.house > top) {
        best = Use::House;
        top = b.house;
    }
    if (b.farm > top) {
        best = Use::Farm;
        top = b.farm;
    }
    return top > 0.0 ? best : Use::Wild;
}

inline Use useAt(const Params& p, double d, double dw) { return pick(bids(p, d, dw)); }

/// The naive version: rings placed by hand, scaled with the town size, deaf to slopes and river.
inline Use useFixed(double size, double d) {
    if (d < kFixedMarketRadius * size) return Use::Market;
    if (d < kFixedHouseRadius * size) return Use::House;
    return Use::Farm;
}

struct Boundary {
    double d;
    Use from;
    Use to;
};

/// Distances at which the winning use changes along a ray, found by scanning in steps of 0.01 and then bisecting.
inline std::vector<Boundary> boundaries(const Params& p, double dw, double dMax) {
    constexpr double kStep = 0.01;
    std::vector<Boundary> out;
    Use last = useAt(p, 0.0, dw);
    for (int i = 1; static_cast<double>(i) * kStep <= dMax; ++i) {
        const double hi0 = static_cast<double>(i) * kStep;
        const Use now = useAt(p, hi0, dw);
        if (now == last) continue;
        double lo = hi0 - kStep;
        double hi = hi0;
        for (int k = 0; k < 40; ++k) {
            const double mid = 0.5 * (lo + hi);
            (useAt(p, mid, dw) == last ? lo : hi) = mid;
        }
        out.push_back({0.5 * (lo + hi), last, now});
        last = now;
    }
    return out;
}

struct Cell {
    int x;
    int y;
};

/// The cell under a pixel, or nothing outside the map. The left and top edges belong to the cell, the right and bottom ones do not.
inline std::optional<Cell> cellAt(double px, double py, double ox, double oy, double cell, int cols, int rows) {
    if (!(cell > 0.0)) return std::nullopt;
    const double fx = (px - ox) / cell;
    const double fy = (py - oy) / cell;
    if (!(fx >= 0.0 && fx < static_cast<double>(cols) && fy >= 0.0 && fy < static_cast<double>(rows))) return std::nullopt;
    return Cell{static_cast<int>(fx), static_cast<int>(fy)};
}

struct Terrain {
    int cols = 0;
    int rows = 0;
    std::vector<unsigned char> mask;  // 1 = water
    std::vector<double> waterDist;    // cells to the nearest water cell, kNoWater without any

    bool isWater(int x, int y) const { return mask[static_cast<std::size_t>(y * cols + x)] != 0; }

    static Terrain fromMask(int cols, int rows, std::vector<unsigned char> mask) {
        Terrain t;
        t.cols = cols;
        t.rows = rows;
        t.mask = std::move(mask);
        t.waterDist.assign(t.mask.size(), kNoWater);
        std::vector<Cell> water;
        for (int y = 0; y < rows; ++y) {
            for (int x = 0; x < cols; ++x) {
                if (t.isWater(x, y)) water.push_back({x, y});
            }
        }
        for (int y = 0; y < rows; ++y) {
            for (int x = 0; x < cols; ++x) {
                double best = kNoWater;
                for (const Cell& w : water) best = std::min(best, std::hypot(static_cast<double>(x - w.x), static_cast<double>(y - w.y)));
                t.waterDist[static_cast<std::size_t>(y * cols + x)] = best;
            }
        }
        return t;
    }

    /// A river running top to bottom, two cells wide, bending a little, a little past the middle.
    static Terrain river(int cols, int rows) {
        std::vector<unsigned char> mask(static_cast<std::size_t>(cols * rows), 0);
        for (int y = 0; y < rows; ++y) {
            const double rx = 0.55 * static_cast<double>(cols) + 1.8 * std::sin(0.45 * static_cast<double>(y));
            for (int x = 0; x < cols; ++x) {
                if (std::fabs(static_cast<double>(x) + 0.5 - rx) < 1.0) mask[static_cast<std::size_t>(y * cols + x)] = 1;
            }
        }
        return fromMask(cols, rows, std::move(mask));
    }
};

/// Land use of every cell for a town centred on (cx, cy). Water stays water in both modes.
inline std::vector<Use> classify(const Terrain& t, const Params& p, int cx, int cy, Mode mode) {
    std::vector<Use> out(t.mask.size(), Use::Wild);
    for (int y = 0; y < t.rows; ++y) {
        for (int x = 0; x < t.cols; ++x) {
            const auto i = static_cast<std::size_t>(y * t.cols + x);
            if (t.mask[i] != 0) {
                out[i] = Use::Water;
                continue;
            }
            const double d = std::hypot(static_cast<double>(x - cx), static_cast<double>(y - cy));
            out[i] = mode == Mode::BidCurves ? useAt(p, d, t.waterDist[i]) : useFixed(p.size, d);
        }
    }
    return out;
}

inline std::array<int, 5> count(const std::vector<Use>& uses) {
    std::array<int, 5> n{};
    for (Use u : uses) ++n[idx(u)];
    return n;
}

/// Slider track: pixel x to value, clamped to the track. A track without width gives the lower end.
inline double valueAtX(double x, double x0, double width, double lo, double hi) {
    if (!(width > 0.0)) return lo;
    const double t = std::clamp((x - x0) / width, 0.0, 1.0);
    return lo + t * (hi - lo);
}

/// Value to pixel x on a slider track, clamped. An empty value range puts the knob at the left end.
inline double xAtValue(double v, double x0, double width, double lo, double hi) {
    if (!(hi > lo)) return x0;
    const double t = std::clamp((v - lo) / (hi - lo), 0.0, 1.0);
    return x0 + t * width;
}

/// The town grows and shrinks between lo and hi: clamp at the ends and turn around, however large dt is.
inline void advanceSize(double& size, double& dir, double dt, double rate, double lo, double hi) {
    size += dir * rate * dt;
    if (size >= hi) {
        size = hi;
        dir = -1.0;
    } else if (size <= lo) {
        size = lo;
        dir = 1.0;
    }
}

}  // namespace brt
