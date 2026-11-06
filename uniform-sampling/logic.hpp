// Uniform random points in a disc, a triangle and on a sphere, plus the naive versions that clump.
//
// The rule behind all of them: a mapping from two uniform numbers (u, v) to a shape is only uniform if it keeps equal
// areas equal. The naive versions map equal steps of a parameter (radius, distance from the apex, polar angle) to
// equal steps on the shape, but the area behind such a step is not equal: a ring near the center is small, a ring
// near the rim is big. The fix is to make the parameter proportional to the square root of the area (disc, triangle)
// or to use the height (sphere, Archimedes' hat-box theorem).
//
// Sources: Greg Turk, "Generating Random Points in Triangles", Graphics Gems I, and Peter Shirley, "Nonuniform
// Random Point Sets via Warping", Graphics Gems III.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace us {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

/// PCG32 (O'Neill), small, fast and the same on every platform, so a seed always gives the same cloud.
class Rng {
public:
    explicit Rng(std::uint64_t seed, std::uint64_t stream = 54) {
        inc_ = (stream << 1U) | 1U;
        next();
        state_ += seed;
        next();
    }

    std::uint32_t next() {
        const std::uint64_t old = state_;
        state_ = old * 6364136223846793005ULL + inc_;
        const auto xorshifted = static_cast<std::uint32_t>(((old >> 18U) ^ old) >> 27U);
        const auto rot = static_cast<std::uint32_t>(old >> 59U);
        return (xorshifted >> rot) | (xorshifted << ((~rot + 1U) & 31U));
    }

    /// 53 random bits as a double in [0, 1).
    double uniform() {
        const std::uint32_t a = next() >> 5U;
        const std::uint32_t b = next() >> 6U;
        return (static_cast<double>(a) * 67108864.0 + static_cast<double>(b)) / 9007199254740992.0;
    }

private:
    std::uint64_t state_ = 0;
    std::uint64_t inc_ = 0;
};

enum class Shape { Disc, Triangle, Sphere };
/// Fold is the second correct way to fill a triangle, for the other shapes it means Uniform.
enum class Method { Naive, Uniform, Fold };

// ---- Disc (unit radius) ----

/// Radius and angle both uniform: every ring gets the same share of the points, but the inner rings have much less area.
inline Vec2 discNaive(double u, double v) {
    const double r = u;
    const double a = kTwoPi * v;
    return {r * std::cos(a), r * std::sin(a)};
}

/// The share of the area inside radius r is r^2, so r = sqrt(u) puts a fraction u of the points inside r.
inline Vec2 discUniform(double u, double v) {
    const double r = std::sqrt(u);
    const double a = kTwoPi * v;
    return {r * std::cos(a), r * std::sin(a)};
}

/// A ring between rMin and rMax around the center, for example enemies that spawn around the player. The area inside
/// radius r grows with r^2, so it is r^2 that gets interpolated, not r. rMin = 0 is the full disc.
inline Vec2 annulusUniform(double rMin, double rMax, double u, double v) {
    const double r = std::sqrt(rMin * rMin + u * (rMax * rMax - rMin * rMin));
    const double a = kTwoPi * v;
    return {r * std::cos(a), r * std::sin(a)};
}

// ---- Triangle (barycentric weights a, b, c of the corners A, B, C, returned as x, y, z) ----

/// t picks a segment parallel to BC and s a point on it. Equal steps of t are equal steps of distance from A, but the
/// segments get longer the further they are from A, so the points pile up at the apex.
inline Vec3 triangleNaive(double s, double t) { return {1.0 - t, (1.0 - s) * t, s * t}; }

/// Turk's method 1: the root of t makes the share of the area between A and the segment equal to t.
inline Vec3 triangleUniform(double s, double t) {
    const double q = std::sqrt(t);
    return {1.0 - q, (1.0 - s) * q, s * q};
}

/// Turk's method 2: (s, t) is uniform in a parallelogram made of two triangles, a draw in the wrong half is reflected
/// about the center of the parallelogram into the right one. No root, but it only works for triangles.
inline Vec3 triangleFold(double s, double t) {
    if (s + t > 1.0) {
        s = 1.0 - s;
        t = 1.0 - t;
    }
    return {1.0 - s - t, s, t};
}

/// Barycentric weights to a point in the plane.
inline Vec2 triangleToPlane(Vec2 a, Vec2 b, Vec2 c, Vec3 w) { return {w.x * a.x + w.y * b.x + w.z * c.x, w.x * a.y + w.y * b.y + w.z * c.y}; }

// ---- Sphere (unit radius, the poles are on the z axis) ----

/// The polar angle uniform: the rings of latitude near the poles are tiny, but get as many points as the equator.
inline Vec3 sphereNaive(double u, double v) {
    const double theta = kPi * u;
    const double phi = kTwoPi * v;
    const double rho = std::sin(theta);
    return {rho * std::cos(phi), rho * std::sin(phi), std::cos(theta)};
}

/// Archimedes: the area of a zone between two heights depends only on the distance of the heights, not on where it
/// is. So z itself is uniform in [-1, 1].
inline Vec3 sphereUniform(double u, double v) {
    const double z = 1.0 - 2.0 * u;
    const double phi = kTwoPi * v;
    const double rho = std::sqrt(std::max(0.0, 1.0 - z * z));
    return {rho * std::cos(phi), rho * std::sin(phi), z};
}

/// One sample of the shape. Disc: (x, y, 0). Triangle: barycentric weights. Sphere: (x, y, z).
inline Vec3 samplePoint(Shape shape, Method method, double u, double v) {
    switch (shape) {
        case Shape::Disc: {
            const Vec2 p = method == Method::Naive ? discNaive(u, v) : discUniform(u, v);
            return {p.x, p.y, 0.0};
        }
        case Shape::Triangle:
            return method == Method::Naive ? triangleNaive(u, v) : method == Method::Fold ? triangleFold(u, v) : triangleUniform(u, v);
        case Shape::Sphere: return method == Method::Naive ? sphereNaive(u, v) : sphereUniform(u, v);
    }
    return {};
}

// ---- Measuring: bins of equal area ----

/// Where a point lies on a scale in which equal steps are equal areas: 0 at the center (disc), the apex (triangle) or
/// the south pole (sphere), 1 at the rim, the base or the north pole.
inline double areaFraction(Shape shape, Vec3 p) {
    switch (shape) {
        case Shape::Disc: return p.x * p.x + p.y * p.y;
        case Shape::Triangle: {
            const double h = std::clamp(1.0 - p.x, 0.0, 1.0);  // distance from the apex, 1 on the base
            return h * h;
        }
        case Shape::Sphere: return (1.0 + p.z) / 2.0;
    }
    return 0.0;
}

/// Bin of a fraction in [0, 1]. Values outside (rounding) and NaN end up in the first or last bin.
inline int binIndex(double fraction, int bins) {
    if (bins <= 0 || !(fraction > 0.0)) return 0;
    const double scaled = std::min(fraction, 1.0) * static_cast<double>(bins);
    return std::min(static_cast<int>(scaled), bins - 1);  // scaled <= bins, so the cast is safe
}

struct Cloud {
    std::vector<Vec3> points;
    std::vector<int> bins;  // counts per band of equal area
};

/// `count` points. The same seed gives the same (u, v) sequence for every method, so naive and correct are fed
/// identical random numbers and only the mapping differs.
inline Cloud generate(Shape shape, Method method, int count, std::uint64_t seed, int bins) {
    Cloud cloud;
    const int n = std::max(count, 0);
    cloud.points.reserve(static_cast<std::size_t>(n));
    cloud.bins.assign(static_cast<std::size_t>(std::max(bins, 0)), 0);
    Rng rng(seed);
    for (int i = 0; i < n; ++i) {
        const double u = rng.uniform();
        const double v = rng.uniform();
        const Vec3 p = samplePoint(shape, method, u, v);
        cloud.points.push_back(p);
        if (bins > 0) ++cloud.bins[static_cast<std::size_t>(binIndex(areaFraction(shape, p), bins))];
    }
    return cloud;
}

/// Smallest and largest bin as a multiple of the expected count (1.0 = perfectly flat).
struct Spread {
    double minRatio = 0.0;
    double maxRatio = 0.0;
};

inline Spread spread(const std::vector<int>& bins) {
    long long total = 0;
    for (int c : bins) total += c;
    if (bins.empty() || total <= 0) return {};
    const double expected = static_cast<double>(total) / static_cast<double>(bins.size());
    const auto [lo, hi] = std::minmax_element(bins.begin(), bins.end());
    return {static_cast<double>(*lo) / expected, static_cast<double>(*hi) / expected};
}

// ---- Viewing the sphere ----

/// Orthographic view of a point: x right, y up, depth < 0 faces the viewer. `yaw` spins about the pole axis, `pitch`
/// tips the north pole towards the viewer (pi/2 looks straight down on it).
struct View {
    double x = 0.0;
    double y = 0.0;
    double depth = 0.0;
};

inline View project(Vec3 p, double yaw, double pitch) {
    const double x1 = p.x * std::cos(yaw) - p.y * std::sin(yaw);
    const double y1 = p.x * std::sin(yaw) + p.y * std::cos(yaw);
    const double y2 = y1 * std::cos(pitch) - p.z * std::sin(pitch);
    const double z2 = y1 * std::sin(pitch) + p.z * std::cos(pitch);
    return {x1, z2, y2};
}

/// Angle in [0, 2 pi), so a spin that runs for hours does not lose precision.
inline double wrapAngle(double a) {
    double r = std::fmod(a, kTwoPi);
    if (r < 0.0) r += kTwoPi;
    return r >= kTwoPi ? 0.0 : r;
}

/// 1234567 -> "1,234,567". Built by hand, snprintf with a locale would print a dot or a comma depending on the system.
inline std::string withThousands(long long n) {
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

}  // namespace us
