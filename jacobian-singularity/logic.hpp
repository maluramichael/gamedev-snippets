// Jacobian inverse kinematics for a planar arm with three segments, without SDL.
//
// The arm has three joint angles and the end effector has two coordinates, so the Jacobian J is a 2x3 matrix:
// column i says how the end effector moves per radian of joint i. Every method below turns the error e (target minus
// end effector) into joint steps d_theta with J d_theta ~ e. They differ in how they invert J, which is the whole story
// near the stretched pose: there J loses a direction (the arm cannot get longer), its smallest singular value goes to 0
// and everything that divides by it explodes.
//
// Math coordinates (y points up), base at the origin, pixels and radians. The Jacobian is analytic; numericJacobian
// is only there for the test to compare against.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>

namespace jac {

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};

using Angles = std::array<double, 3>;
using Lengths = std::array<double, 3>;

/// 2x3 Jacobian stored by columns: col[i] = d(end effector) / d(theta_i).
struct Jacobian {
    std::array<Vec2, 3> col{};
};

/// [[a, b], [c, d]]
struct Mat2 {
    double a = 0.0;
    double b = 0.0;
    double c = 0.0;
    double d = 0.0;
};

struct SingularValues {
    double max = 0.0;
    double min = 0.0;
};

enum class Method { Transpose, PseudoInverse, Damped };

inline constexpr double kPi = 3.14159265358979323846;

/// The demo arm. Reach is 250 px.
inline constexpr Lengths kLengths{100.0, 85.0, 65.0};

/// ClampMag limit of Buss: about half the length of a typical segment.
inline constexpr double kDmax = 0.5 * (kLengths[0] + kLengths[1] + kLengths[2]) / 3.0;

inline double norm(Vec2 v) { return std::hypot(v.x, v.y); }

inline double norm(const Angles& a) { return std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]); }

inline double reach(const Lengths& len) { return len[0] + len[1] + len[2]; }

/// Maps any angle to [-pi, pi]. Keeps the angles small however long the arm thrashes about.
inline double wrapAngle(double a) { return std::remainder(a, 2.0 * kPi); }

/// Base, then the end of each segment. The angles add up along the chain.
inline std::array<Vec2, 4> joints(const Lengths& len, const Angles& theta) {
    std::array<Vec2, 4> p{};
    double a = 0.0;
    for (std::size_t i = 0; i < 3; ++i) {
        a += theta[i];
        p[i + 1] = {p[i].x + len[i] * std::cos(a), p[i].y + len[i] * std::sin(a)};
    }
    return p;
}

inline Vec2 endEffector(const Lengths& len, const Angles& theta) { return joints(len, theta)[3]; }

/// Rotating joint i by a small angle moves the end effector perpendicular to the line from joint i to the end effector.
inline Jacobian jacobian(const Lengths& len, const Angles& theta) {
    const auto p = joints(len, theta);
    Jacobian j;
    for (std::size_t i = 0; i < 3; ++i) {
        const double dx = p[3].x - p[i].x;
        const double dy = p[3].y - p[i].y;
        j.col[i] = {-dy, dx};
    }
    return j;
}

/// Central differences, for the test only.
inline Jacobian numericJacobian(const Lengths& len, const Angles& theta, double h = 1e-6) {
    Jacobian j;
    for (std::size_t i = 0; i < 3; ++i) {
        Angles up = theta;
        Angles down = theta;
        up[i] += h;
        down[i] -= h;
        const Vec2 a = endEffector(len, up);
        const Vec2 b = endEffector(len, down);
        j.col[i] = {(a.x - b.x) / (2.0 * h), (a.y - b.y) / (2.0 * h)};
    }
    return j;
}

/// J J^T, a symmetric 2x2 matrix.
inline Mat2 gram(const Jacobian& j) {
    Mat2 g;
    for (const Vec2& c : j.col) {
        g.a += c.x * c.x;
        g.b += c.x * c.y;
        g.d += c.y * c.y;
    }
    g.c = g.b;
    return g;
}

/// Inverse of a 2x2 matrix, or nothing if it is (nearly) singular. The guard is relative to the matrix size, so a small
/// well-conditioned matrix still inverts and a big nearly singular one does not.
inline std::optional<Mat2> inverse(const Mat2& m) {
    const double det = m.a * m.d - m.b * m.c;
    const double scale = m.a + m.d;
    if (std::fabs(det) <= 1e-12 * scale * scale) return std::nullopt;
    return Mat2{m.d / det, -m.b / det, -m.c / det, m.a / det};
}

/// Singular values of J are the square roots of the eigenvalues of J J^T. The small eigenvalue comes from
/// det / larger eigenvalue, which stays accurate when it is tiny (the difference of the two roots would lose it).
inline SingularValues singularValues(const Jacobian& j) {
    const Mat2 g = gram(j);
    const double tr = g.a + g.d;
    const double det = g.a * g.d - g.b * g.b;
    const double disc = std::sqrt(std::max(0.0, tr * tr / 4.0 - det));
    const double big = tr / 2.0 + disc;
    const double small = big > 0.0 ? std::max(0.0, det / big) : 0.0;
    return {std::sqrt(big), std::sqrt(small)};
}

/// Unit direction in which the end effector is hardest to move (eigenvector of J J^T for the smaller eigenvalue).
/// At the stretched pose that is the direction along the arm.
inline Vec2 weakDirection(const Jacobian& j) {
    const Mat2 g = gram(j);
    const double s = singularValues(j).min;
    const double lambda = s * s;
    // (g - lambda I) u = 0: the rows are (a - l, b) and (b, d - l), u is perpendicular to the longer one.
    const Vec2 r1{g.a - lambda, g.b};
    const Vec2 r2{g.b, g.d - lambda};
    const Vec2 r = norm(r1) >= norm(r2) ? r1 : r2;
    const double n = norm(r);
    if (n < 1e-300) return {1.0, 0.0};  // J J^T is a multiple of the identity, every direction is equally hard
    return {-r.y / n, r.x / n};
}

/// Buss' ClampMag: a vector longer than d is shortened to length d, its direction stays.
inline Vec2 clampMag(Vec2 w, double d) {
    const double n = norm(w);
    if (d <= 0.0) return {0.0, 0.0};
    if (n <= d) return w;
    return {w.x * d / n, w.y * d / n};
}

namespace detail {

inline Angles transposeTimes(const Jacobian& j, Vec2 f) {
    return {j.col[0].x * f.x + j.col[0].y * f.y, j.col[1].x * f.x + j.col[1].y * f.y, j.col[2].x * f.x + j.col[2].y * f.y};
}

inline Vec2 apply(const Mat2& m, Vec2 v) { return {m.a * v.x + m.b * v.y, m.c * v.x + m.d * v.y}; }

}  // namespace detail

/// d_theta = alpha J^T e with alpha = <e, J J^T e> / <J J^T e, J J^T e>, the step length that makes J d_theta the best
/// multiple of J J^T e for the error e. No inverse, so no explosion, but a stretched arm barely moves.
inline Angles stepTranspose(const Jacobian& j, Vec2 e) {
    const Vec2 u = detail::apply(gram(j), e);
    const double den = u.x * u.x + u.y * u.y;
    if (den <= 1e-18) return {0.0, 0.0, 0.0};
    const double alpha = (e.x * u.x + e.y * u.y) / den;
    const Angles t = detail::transposeTimes(j, e);
    return {alpha * t[0], alpha * t[1], alpha * t[2]};
}

/// d_theta = J^T (J J^T)^-1 e. Where J J^T is singular the formula has no answer and the step is 0.
inline Angles stepPseudoInverse(const Jacobian& j, Vec2 e) {
    const auto inv = inverse(gram(j));
    if (!inv) return {0.0, 0.0, 0.0};
    return detail::transposeTimes(j, detail::apply(*inv, e));
}

/// d_theta = J^T (J J^T + lambda^2 I)^-1 e. The smallest singular value s contributes s / (s^2 + lambda^2) instead of
/// 1 / s, which never exceeds 1 / (2 lambda).
inline Angles stepDamped(const Jacobian& j, Vec2 e, double lambda) {
    Mat2 g = gram(j);
    g.a += lambda * lambda;
    g.d += lambda * lambda;
    const auto inv = inverse(g);
    if (!inv) return {0.0, 0.0, 0.0};
    return detail::transposeTimes(j, detail::apply(*inv, e));
}

inline Angles stepFor(Method m, const Jacobian& j, Vec2 e, double lambda) {
    switch (m) {
        case Method::Transpose: return stepTranspose(j, e);
        case Method::PseudoInverse: return stepPseudoInverse(j, e);
        case Method::Damped: return stepDamped(j, e, lambda);
    }
    return {0.0, 0.0, 0.0};
}

/// One iteration towards `target` (relative to the base): error, optional clamp (dmax > 0), step, add to the angles.
/// Returns the step that was applied.
inline Angles iterate(Angles& theta, const Lengths& len, Vec2 target, Method m, double lambda, double dmax) {
    const Vec2 p = endEffector(len, theta);
    Vec2 e{target.x - p.x, target.y - p.y};
    if (dmax > 0.0) e = clampMag(e, dmax);
    const Angles d = stepFor(m, jacobian(len, theta), e, lambda);
    for (std::size_t i = 0; i < 3; ++i) theta[i] = wrapAngle(theta[i] + d[i]);
    return d;
}

/// A target that wanders along the edge of the reach: the angle swings slowly, the distance breathes between 0.82 and
/// 1.08 times the reach. Both phases are kept in one period so they stay precise after hours.
struct Autopilot {
    double phaseA = 0.3;  // swing
    double phaseB = 1.0;  // distance, starts at 1.06 times the reach

    Vec2 target() const {
        const double angle = 0.55 * std::sin(phaseA);
        const double r = reach(kLengths) * (0.95 + 0.13 * std::sin(phaseB));
        return {r * std::cos(angle), r * std::sin(angle)};
    }

    void advance(double dt) {
        if (dt <= 0.0) return;
        phaseA = wrapPhase(phaseA + 0.55 * dt);
        phaseB = wrapPhase(phaseB + 0.9 * dt);
    }

private:
    static double wrapPhase(double p) {
        const double q = std::fmod(p, 2.0 * kPi);
        return q < 0.0 ? q + 2.0 * kPi : q;
    }
};

}  // namespace jac
