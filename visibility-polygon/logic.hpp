// Logic of the visibility demo, no SDL. Everything that can be wrong lives here and is covered by test.cpp.
//
// Walls are line segments, the light is a point. The visibility polygon is built by casting rays from the light
// and connecting the hits in angle order. The only rays that matter are the ones that point at a wall end: between
// two such angles the nearest wall does not change, so the hit points between them lie on one straight wall.
// A ray that goes exactly through a wall end is not enough, the polygon needs the point *behind* the end as well.
// That is what the two rays at +-offset are for.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

namespace vis {

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kTwoPi = 2.0 * kPi;
/// Slack on the position along a wall, so that a ray through a shared wall end hits one of the two walls
/// even when rounding puts it a hair outside both.
inline constexpr double kEndSlack = 1e-9;

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};

inline Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
inline bool operator==(Vec2 a, Vec2 b) { return a.x == b.x && a.y == b.y; }
inline bool operator<(Vec2 a, Vec2 b) { return a.x != b.x ? a.x < b.x : a.y < b.y; }
inline double cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }
inline double length(Vec2 a) { return std::sqrt(a.x * a.x + a.y * a.y); }

struct Segment {
    Vec2 a;
    Vec2 b;
};

struct Hit {
    double t = 0.0;  // distance along the ray, in lengths of the direction vector
    Vec2 point;
};

/// Nearest wall hit by the ray `origin + t * dir`, t > 0. A wall parallel to the ray (also a collinear one) is skipped:
/// its ends get rays of their own. A hit exactly on a wall end counts.
inline std::optional<Hit> castRay(Vec2 origin, Vec2 dir, const std::vector<Segment>& walls) {
    const double dirLen = length(dir);
    std::optional<Hit> best;
    for (const Segment& w : walls) {
        const Vec2 e = w.b - w.a;
        const double denom = cross(dir, e);
        if (std::fabs(denom) <= 1e-12 * dirLen * length(e)) continue;  // parallel, or a zero length wall or direction
        const Vec2 ao = w.a - origin;
        const double t = cross(ao, e) / denom;
        const double u = cross(ao, dir) / denom;  // position along the wall, 0 at a, 1 at b
        if (t <= 1e-9 || u < -kEndSlack || u > 1.0 + kEndSlack) continue;
        if (!best || t < best->t) best = Hit{t, {origin.x + dir.x * t, origin.y + dir.y * t}};
    }
    return best;
}

enum class Method {
    CornerRays,    // the right one: a ray at every wall end and one just before and after it
    ExactCorners,  // the naive shortcut: only the ray through the end, the point behind it is missing
    Uniform,       // N evenly spaced rays, ignores the walls
};

enum class RayKind { Corner, Before, After, Uniform, ConeEdge };

struct Options {
    Method method = Method::CornerRays;
    int uniformCount = 90;
    double offset = 1e-4;       // radians, distance of the Before/After rays from the one through the corner
    double aim = 0.0;           // direction of the flashlight, radians
    double halfAngle = kPi;     // half the opening of the cone; pi or more is the full circle
};

struct Ray {
    double angle = 0.0;  // absolute, radians
    RayKind kind = RayKind::Corner;
    Vec2 point;          // where it ends
    double dist = 0.0;
};

struct Candidate {
    double rel = 0.0;  // angle relative to the aim, in (-pi, pi]
    RayKind kind = RayKind::Corner;
};

/// Output buffers, reused from frame to frame (clear keeps the capacity).
struct Result {
    std::vector<Ray> rays;
    /// The fan around the light. A cone starts with the light itself (rimBegin 1), the full circle is closed on its own.
    std::vector<Vec2> polygon;
    bool full = true;
    std::size_t rimBegin = 0;
    std::vector<Candidate> candidates;  // scratch
    std::vector<Vec2> corners;          // scratch
};

inline double wrapAngle(double a) {
    double r = std::remainder(a, kTwoPi);  // [-pi, pi]
    if (r <= -kPi) r += kTwoPi;
    return r;
}

inline void compute(Vec2 origin, const std::vector<Segment>& walls, const Options& opt, Result& out) {
    out.rays.clear();
    out.polygon.clear();
    out.candidates.clear();
    out.corners.clear();
    out.full = opt.halfAngle >= kPi - 1e-12;
    out.rimBegin = out.full ? 0 : 1;
    const double half = std::min(opt.halfAngle, kPi);
    const double aim = out.full ? 0.0 : opt.aim;
    auto inCone = [&](double rel) { return out.full || std::fabs(rel) <= half + 1e-12; };
    auto add = [&](double rel, RayKind kind) {
        const double r = wrapAngle(rel);
        if (inCone(r)) out.candidates.push_back({r, kind});
    };

    if (opt.method == Method::Uniform) {
        const int n = std::clamp(opt.uniformCount, 3, 4096);
        for (int i = 0; i < n; ++i) add(-kPi + kTwoPi * static_cast<double>(i) / static_cast<double>(n) - aim, RayKind::Uniform);
    } else {
        for (const Segment& w : walls) {
            out.corners.push_back(w.a);
            out.corners.push_back(w.b);
        }
        std::sort(out.corners.begin(), out.corners.end());
        out.corners.erase(std::unique(out.corners.begin(), out.corners.end()), out.corners.end());
        for (const Vec2& c : out.corners) {
            const Vec2 d = c - origin;
            if (length(d) < 1e-9) continue;  // the light sits on this end: it has no direction
            const double rel = std::atan2(d.y, d.x) - aim;
            if (opt.method == Method::CornerRays) add(rel - opt.offset, RayKind::Before);
            add(rel, RayKind::Corner);
            if (opt.method == Method::CornerRays) add(rel + opt.offset, RayKind::After);
        }
    }
    if (!out.full) {
        add(-half, RayKind::ConeEdge);
        add(half, RayKind::ConeEdge);
    }

    std::stable_sort(out.candidates.begin(), out.candidates.end(), [](const Candidate& a, const Candidate& b) { return a.rel < b.rel; });
    // Two rays at the same angle (a cone edge on a corner) are one ray.
    out.candidates.erase(std::unique(out.candidates.begin(), out.candidates.end(),
                                     [](const Candidate& a, const Candidate& b) { return std::fabs(a.rel - b.rel) < 1e-12; }),
                         out.candidates.end());

    if (!out.full) out.polygon.push_back(origin);
    for (const Candidate& c : out.candidates) {
        const double angle = aim + c.rel;
        const auto hit = castRay(origin, {std::cos(angle), std::sin(angle)}, walls);
        if (!hit) continue;  // the light is outside the walls
        out.rays.push_back({angle, c.kind, hit->point, hit->t});
        out.polygon.push_back(hit->point);
    }
    if (out.rays.empty()) out.polygon.clear();
}

/// Even-odd point in polygon test (see the point-in-polygon demo). The division is safe: it only runs for edges
/// whose ends lie on different sides of the row.
inline bool contains(const std::vector<Vec2>& poly, Vec2 p) {
    bool in = false;
    const std::size_t n = poly.size();
    for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
        const Vec2 a = poly[j];
        const Vec2 b = poly[i];
        if ((a.y > p.y) != (b.y > p.y) && p.x < a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y)) in = !in;
    }
    return in;
}

inline double area(const std::vector<Vec2>& poly) {
    double sum = 0.0;
    const std::size_t n = poly.size();
    for (std::size_t i = 0, j = n - 1; i < n; j = i++) sum += cross(poly[j], poly[i]);
    return std::fabs(sum) / 2.0;
}

/// The direct answer to "can the light see p": no wall properly crosses the segment from the light to p.
/// Touching a wall end does not block.
inline bool lineOfSight(Vec2 from, Vec2 to, const std::vector<Segment>& walls) {
    const Vec2 d = to - from;
    for (const Segment& w : walls) {
        const Vec2 e = w.b - w.a;
        const double s1 = cross(e, from - w.a);
        const double s2 = cross(e, to - w.a);
        const double s3 = cross(d, w.a - from);
        const double s4 = cross(d, w.b - from);
        if (s1 * s2 < 0.0 && s3 * s4 < 0.0) return false;
    }
    return true;
}

// ---- building and editing walls ------------------------------------------------------------------------------------

inline void addBox(std::vector<Segment>& walls, double x, double y, double w, double h) {
    walls.push_back({{x, y}, {x + w, y}});
    walls.push_back({{x + w, y}, {x + w, y + h}});
    walls.push_back({{x + w, y + h}, {x, y + h}});
    walls.push_back({{x, y + h}, {x, y}});
}

inline void addLoop(std::vector<Segment>& walls, const std::vector<Vec2>& points) {
    for (std::size_t i = 0; i < points.size(); ++i) walls.push_back({points[i], points[(i + 1) % points.size()]});
}

/// The wall end closest to p within `radius`, looking only at the first `movable` walls.
inline std::optional<Vec2> nearestCorner(const std::vector<Segment>& walls, std::size_t movable, Vec2 p, double radius) {
    std::optional<Vec2> best;
    double bestDist = radius;
    for (std::size_t i = 0; i < std::min(movable, walls.size()); ++i) {
        for (const Vec2 c : {walls[i].a, walls[i].b}) {
            const double d = length(c - p);
            if (d <= bestDist) {
                bestDist = d;
                best = c;
            }
        }
    }
    return best;
}

/// Moves every wall end that sits exactly at `from` to `to`, so a box stays closed while you drag a corner.
inline std::size_t moveCorner(std::vector<Segment>& walls, Vec2 from, Vec2 to) {
    std::size_t moved = 0;
    for (Segment& w : walls) {
        if (w.a == from) {
            w.a = to;
            ++moved;
        }
        if (w.b == from) {
            w.b = to;
            ++moved;
        }
    }
    return moved;
}

}  // namespace vis
