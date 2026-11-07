// Redundant inverse kinematics for a planar arm with four segments, without SDL.
//
// The hand has two coordinates but the arm has four joint angles, so the Jacobian J is a 2x4 matrix and two
// directions of joint space do not move the hand at all: the null space of J. Whatever joint step we add inside it
// leaves the hand where it is, to first order. Buss writes it as
//
//     d_theta = J+ e + (I - J+ J) phi
//
// J+ e brings the hand to the target (the shortest joint step that does it), the projector (I - J+ J) cuts a free
// wish phi down to the part that does not touch the hand. phi is the second goal: stay near a rest pose, keep away
// from the joint limits, stay relaxed.
//
// Math coordinates (y points up), base at the origin, pixels and radians.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace nsp {

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};

inline constexpr std::size_t kJoints = 4;

using Angles = std::array<double, kJoints>;
using Lengths = std::array<double, kJoints>;

/// 2x4 Jacobian stored by columns: col[i] = d(point) / d(theta_i).
struct Jacobian {
    std::array<Vec2, kJoints> col{};
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

/// What the free wish phi asks for.
enum class Secondary { Off, Rest, Limits, Comfort };

inline constexpr double kPi = 3.14159265358979323846;

/// The demo arm. Reach is 260 px.
inline constexpr Lengths kLengths{80.0, 70.0, 60.0, 50.0};

/// Longest error the primary step may act on, so a far target does not ask for a huge joint step.
inline constexpr double kDmax = 40.0;

/// Longest joint step per iteration (norm, radians). Only matters near the stretched pose.
inline constexpr double kMaxJointStep = 0.35;

/// Below this smallest singular value (px per rad) the primary step is damped, see primaryLambda.
inline constexpr double kSigmaFade = 8.0;

/// Damping of the projector. Tiny: it only keeps the 2x2 inverse alive when J has rank 1.
inline constexpr double kProjectionLambda = 0.01;

inline constexpr Angles kRest{0.6, 0.9, -0.7, -0.3};
inline constexpr Angles kLimitLo{-0.5, 0.0, -2.4, -1.5};
inline constexpr Angles kLimitHi{2.5, 2.6, 0.0, 1.5};

inline constexpr double kRestGain = 0.05;      // fraction of the way to the rest pose per step
inline constexpr double kLimitGain = 0.08;     // fraction of the half range per step at a limit
inline constexpr double kComfortGain = 0.03;   // rad per step towards 0
inline constexpr double kComfortSoft = 0.05;   // rad, rounds the corner of |angle| at 0
inline constexpr double kLimitDeadZone = 0.5;  // the limit push starts at half the half range
inline constexpr double kKickGain = 0.12;     // rad per step of the shake, along a random unit vector
inline constexpr int kKickSteps = 40;
inline constexpr double kGrabGain = 0.3;
inline constexpr double kGrabLambda = 8.0;
inline constexpr double kGrabMax = 60.0;

inline double norm(Vec2 v) { return std::hypot(v.x, v.y); }

inline double norm(const Angles& a) {
    double s = 0.0;
    for (const double v : a) s += v * v;
    return std::sqrt(s);
}

inline double reach(const Lengths& len) { return len[0] + len[1] + len[2] + len[3]; }

/// Maps any angle to [-pi, pi]. Keeps the angles small however long the arm moves about.
inline double wrapAngle(double a) { return std::remainder(a, 2.0 * kPi); }

/// Base, then the end of each segment. The angles add up along the chain.
inline std::array<Vec2, kJoints + 1> joints(const Lengths& len, const Angles& theta) {
    std::array<Vec2, kJoints + 1> p{};
    double a = 0.0;
    for (std::size_t i = 0; i < kJoints; ++i) {
        a += theta[i];
        p[i + 1] = {p[i].x + len[i] * std::cos(a), p[i].y + len[i] * std::sin(a)};
    }
    return p;
}

inline Vec2 endEffector(const Lengths& len, const Angles& theta) { return joints(len, theta)[kJoints]; }

/// Jacobian of the position of joint k (1 to 4, 4 is the hand). Rotating joint i moves that point perpendicular to the
/// line from joint i to it. Joints from k on sit behind the point and do not move it: their columns are 0.
inline Jacobian jacobianAt(const Lengths& len, const Angles& theta, std::size_t k) {
    const auto p = joints(len, theta);
    Jacobian j;
    for (std::size_t i = 0; i < k && i < kJoints; ++i) {
        const double dx = p[k].x - p[i].x;
        const double dy = p[k].y - p[i].y;
        j.col[i] = {-dy, dx};
    }
    return j;
}

inline Jacobian jacobian(const Lengths& len, const Angles& theta) { return jacobianAt(len, theta, kJoints); }

/// Central differences, for the test only.
inline Jacobian numericJacobianAt(const Lengths& len, const Angles& theta, std::size_t k, double h = 1e-6) {
    Jacobian j;
    for (std::size_t i = 0; i < kJoints; ++i) {
        Angles up = theta;
        Angles down = theta;
        up[i] += h;
        down[i] -= h;
        const Vec2 a = joints(len, up)[k];
        const Vec2 b = joints(len, down)[k];
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

/// Inverse of a 2x2 matrix, or false if it is (nearly) singular. The guard is relative to the matrix size.
inline bool inverse(const Mat2& m, Mat2& out) {
    const double det = m.a * m.d - m.b * m.c;
    const double scale = m.a + m.d;
    if (std::fabs(det) <= 1e-12 * scale * scale) return false;
    out = Mat2{m.d / det, -m.b / det, -m.c / det, m.a / det};
    return true;
}

/// Singular values of J are the square roots of the eigenvalues of J J^T. The small one comes from det / larger
/// eigenvalue, which stays accurate when it is tiny.
inline SingularValues singularValues(const Jacobian& j) {
    const Mat2 g = gram(j);
    const double tr = g.a + g.d;
    const double det = g.a * g.d - g.b * g.b;
    const double disc = std::sqrt(std::max(0.0, tr * tr / 4.0 - det));
    const double big = tr / 2.0 + disc;
    const double small = big > 0.0 ? std::max(0.0, det / big) : 0.0;
    return {std::sqrt(big), std::sqrt(small)};
}

/// Number of free directions in joint space: 4 joints minus the rank of J (2, or 1 when the arm is stretched or folded).
inline int nullDimensions(const Jacobian& j) {
    const SingularValues s = singularValues(j);
    const int rank = s.min > 1.0 ? 2 : (s.max > 1.0 ? 1 : 0);
    return static_cast<int>(kJoints) - rank;
}

inline Vec2 clampMag(Vec2 w, double d) {
    const double n = norm(w);
    if (d <= 0.0) return {0.0, 0.0};
    if (n <= d) return w;
    return {w.x * d / n, w.y * d / n};
}

inline Angles clampNorm(const Angles& a, double d) {
    const double n = norm(a);
    if (d <= 0.0) return Angles{};
    if (n <= d) return a;
    Angles out = a;
    for (double& v : out) v *= d / n;
    return out;
}

/// J v: the hand motion that a joint step v causes (to first order).
inline Vec2 times(const Jacobian& j, const Angles& v) {
    Vec2 out;
    for (std::size_t i = 0; i < kJoints; ++i) {
        out.x += j.col[i].x * v[i];
        out.y += j.col[i].y * v[i];
    }
    return out;
}

/// J^T f
inline Angles transposeTimes(const Jacobian& j, Vec2 f) {
    Angles out{};
    for (std::size_t i = 0; i < kJoints; ++i) out[i] = j.col[i].x * f.x + j.col[i].y * f.y;
    return out;
}

inline Vec2 apply(const Mat2& m, Vec2 v) { return {m.a * v.x + m.b * v.y, m.c * v.x + m.d * v.y}; }

/// J^T (J J^T + lambda^2 I)^-1 v. With lambda = 0 that is the pseudoinverse J+ applied to v. Where the 2x2 matrix has
/// no inverse the answer is the zero step.
inline Angles pinvTimes(const Jacobian& j, Vec2 v, double lambda = 0.0) {
    Mat2 g = gram(j);
    g.a += lambda * lambda;
    g.d += lambda * lambda;
    Mat2 inv;
    if (!inverse(g, inv)) return Angles{};
    return transposeTimes(j, apply(inv, v));
}

/// (I - J+ J) phi: the part of phi that does not move the hand. Static pose, so the hand error stays at first order 0.
inline Angles project(const Jacobian& j, const Angles& phi) {
    const Angles back = pinvTimes(j, times(j, phi), kProjectionLambda);
    Angles out{};
    for (std::size_t i = 0; i < kJoints; ++i) out[i] = phi[i] - back[i];
    return out;
}

/// Damping for the primary step: none while the arm is well bent, growing to kSigmaFade at a singularity, where the
/// plain pseudoinverse would divide by about 0 (see the post on the Jacobian and singularities).
inline double primaryLambda(double sigmaMin) { return std::max(0.0, kSigmaFade - sigmaMin); }

/// Position of joint i inside its range: -1 at the lower limit, +1 at the upper one, 0 in the middle.
inline double limitPosition(const Angles& theta, std::size_t i) {
    const double mid = 0.5 * (kLimitLo[i] + kLimitHi[i]);
    const double half = 0.5 * (kLimitHi[i] - kLimitLo[i]);
    return (theta[i] - mid) / half;
}

/// Largest |limitPosition| of all joints. 1 or more means a joint sits at or beyond its limit.
inline double worstLimitUse(const Angles& theta) {
    double worst = 0.0;
    for (std::size_t i = 0; i < kJoints; ++i) worst = std::max(worst, std::fabs(limitPosition(theta, i)));
    return worst;
}

/// Zucconi's torsion penalty: the mean of the absolute joint angles.
inline double meanAbsAngle(const Angles& theta) {
    double s = 0.0;
    for (const double v : theta) s += std::fabs(v);
    return s / static_cast<double>(kJoints);
}

/// Pulls every joint a fraction of the way to the rest pose.
inline Angles phiRest(const Angles& theta) {
    Angles out{};
    for (std::size_t i = 0; i < kJoints; ++i) out[i] = kRestGain * (kRest[i] - theta[i]);
    return out;
}

/// Pushes a joint back once it is further than half of its half range from the middle of its range. Inside that zone
/// it does nothing, the push grows linearly and is 1 half range times the gain at the limit.
inline Angles phiLimits(const Angles& theta) {
    Angles out{};
    for (std::size_t i = 0; i < kJoints; ++i) {
        const double u = limitPosition(theta, i);
        const double half = 0.5 * (kLimitHi[i] - kLimitLo[i]);
        const double push = std::max(0.0, (std::fabs(u) - kLimitDeadZone) / (1.0 - kLimitDeadZone));
        out[i] = -kLimitGain * half * std::copysign(push, u);
    }
    return out;
}

/// Gradient step on the mean of |angle|, with the corner at 0 rounded off so the arm does not chatter there.
inline Angles phiComfort(const Angles& theta) {
    Angles out{};
    for (std::size_t i = 0; i < kJoints; ++i) out[i] = -kComfortGain * theta[i] / std::hypot(theta[i], kComfortSoft);
    return out;
}

inline Angles phiFor(Secondary s, const Angles& theta) {
    switch (s) {
        case Secondary::Off: return Angles{};
        case Secondary::Rest: return phiRest(theta);
        case Secondary::Limits: return phiLimits(theta);
        case Secondary::Comfort: return phiComfort(theta);
    }
    return Angles{};
}

/// A wish that pulls joint k (1 to 3) towards `to` (relative to the base): a damped pseudoinverse of that joint's own
/// Jacobian, a fraction of the way each step. Fed through the projector the hand stays where it is.
inline Angles grabPhi(const Lengths& len, const Angles& theta, std::size_t k, Vec2 to) {
    const auto p = joints(len, theta);
    const Vec2 e = clampMag({to.x - p[k].x, to.y - p[k].y}, kGrabMax);
    Angles out = pinvTimes(jacobianAt(len, theta, k), e, kGrabLambda);
    for (double& v : out) v *= kGrabGain;
    return out;
}

/// A random unit vector in joint space, the same for the same seed on every machine.
inline Angles kickDirection(unsigned long long seed) {
    unsigned long long s = seed * 6364136223846793005ULL + 1442695040888963407ULL;
    Angles out{};
    for (double& v : out) {
        s = s * 6364136223846793005ULL + 1442695040888963407ULL;
        v = static_cast<double>(s >> 11) / 4503599627370496.0 - 1.0;  // [-1, 1)
    }
    const double n = norm(out);
    if (n < 1e-9) return {1.0, 0.0, 0.0, 0.0};
    for (double& v : out) v /= n;
    return out;
}

struct StepInfo {
    double step = 0.0;  // norm of the joint step
    double self = 0.0;  // norm of the part of it that leaves the hand where it is
};

/// One iteration towards `target` (relative to the base):
///   primary   = J+ e        brings the hand to the target
///   secondary = P phi       the free wish, projected into the null space (or raw when `projected` is false)
/// `extra` is added to the wish (the shake and the grab of a joint). Returns the step that was applied.
inline StepInfo iterate(Angles& theta, const Lengths& len, Vec2 target, Secondary mode, bool projected, const Angles& extra) {
    const Jacobian j = jacobian(len, theta);
    const Vec2 hand = endEffector(len, theta);
    const Vec2 e = clampMag({target.x - hand.x, target.y - hand.y}, kDmax);
    const Angles primary = pinvTimes(j, e, primaryLambda(singularValues(j).min));
    Angles phi = phiFor(mode, theta);
    for (std::size_t i = 0; i < kJoints; ++i) phi[i] += extra[i];
    const Angles secondary = projected ? project(j, phi) : phi;
    Angles d{};
    for (std::size_t i = 0; i < kJoints; ++i) d[i] = primary[i] + secondary[i];
    d = clampNorm(d, kMaxJointStep);
    const double self = norm(project(j, d));
    for (std::size_t i = 0; i < kJoints; ++i) theta[i] = wrapAngle(theta[i] + d[i]);
    return {norm(d), self};
}

/// A pose with the hand on `target`, found by plain iteration without a second goal.
inline Angles settle(Angles theta, const Lengths& len, Vec2 target, int steps) {
    for (int i = 0; i < steps; ++i) iterate(theta, len, target, Secondary::Off, true, Angles{});
    return theta;
}

inline constexpr Angles kSeed{1.4, 1.6, -2.0, 1.2};
inline constexpr Vec2 kStartTarget{150.0, 60.0};

/// Where the demo starts: a bent arm with its hand on the start target, far from the rest pose.
inline Angles startPose() { return settle(kSeed, kLengths, kStartTarget, 600); }

/// A target that wanders on a small ellipse well inside the reach (never near the stretched pose). The phase is kept in
/// one period so it stays precise after hours.
struct Autopilot {
    double phase = 0.0;

    Vec2 target() const { return {130.0 + 80.0 * std::cos(phase), 20.0 + 70.0 * std::sin(phase)}; }

    void advance(double dt) {
        if (dt <= 0.0) return;
        const double q = std::fmod(phase + 0.5 * dt, 2.0 * kPi);
        phase = q < 0.0 ? q + 2.0 * kPi : q;
    }
};

}  // namespace nsp
