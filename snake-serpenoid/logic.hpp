// Serpenoid snake without SDL.
//
// Hirose's serpenoid curve moves a chain of links with one sine wave: joint i bends by
//     phi_i(t) = 2 alpha sin(omega t + (i - 1) beta) + gamma            (Colorado et al. 2010, eq. 1)
// with alpha = a sin(beta / 2), beta = b / n and gamma = -c / n (eq. 2). a is the bend, b how many wave periods fit
// into the body (b = 2 pi is one), c the steering bias. Nothing in that formula says where the snake goes. The shape
// is all there is: the whole body pose (x, y, heading) is an unknown that the ground decides.
//
// The ground is modelled as viscous friction at the center of every link, with a coefficient C_N across the link and
// C_T along it (ratio = C_N / C_T). The body moves so that the total friction force and torque are zero, which is a
// 3x3 linear least-squares problem per step: minimise sum_k C_T (v_k . t_k)^2 + C_N (v_k . n_k)^2 over the body
// velocity, where v_k is the velocity of link center k (body velocity plus the velocity of the shape change).
//  * ratio = 1: the same friction in every direction, the ground cannot push the snake anywhere, the center of mass
//    stays put and the snake wiggles in place.
//  * ratio large: across-the-link slipping is (almost) forbidden, like Hirose's passive wheels. The wave is turned
//    into forward motion.
// Link 0 is the tail end, link kLinks-1 the head. With this index order the wave runs toward link 0 and the snake
// crawls the other way.
//
// Math coordinates (y up), angles in radians, link length 1 unless stated. Body frame: origin at the start of link 0,
// link 0 pointing along +x.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <deque>
#include <optional>

namespace serp {

inline constexpr int kLinks = 12;
inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kTwoPi = 2.0 * kPi;
/// Friction ratios outside [1, kMaxRatio] are clamped. Below 1 the "across" direction would be the slippery one, above
/// 1e6 the system gets numerically singular (a straight snake can slide along its own axis for free).
inline constexpr double kMaxRatio = 1e6;

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};

inline Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
inline Vec2 operator*(Vec2 a, double s) { return {a.x * s, a.y * s}; }

inline Vec2 rotate(Vec2 v, double angle) {
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    return {c * v.x - s * v.y, s * v.x + c * v.y};
}

// ---- The wave -------------------------------------------------------------------------------------------------------

struct Gait {
    double alpha = 0.0;
    double beta = 0.0;
    double gamma = 0.0;
};

inline Gait gaitFrom(double a, double b, double c, int links = kLinks) {
    const double beta = b / static_cast<double>(links);
    return {a * std::sin(beta / 2.0), beta, -c / static_cast<double>(links)};
}

/// Angle of joint `joint` (0-based, the source's i - 1) between link `joint` and link `joint + 1`.
inline double jointAngle(const Gait& g, int joint, double phase) {
    return 2.0 * g.alpha * std::sin(phase + static_cast<double>(joint) * g.beta) + g.gamma;
}

inline double jointRate(const Gait& g, int joint, double phase, double omega) {
    return 2.0 * g.alpha * omega * std::cos(phase + static_cast<double>(joint) * g.beta);
}

/// Wave phase kept in [0, 2 pi): the phase grows forever, sin() does not care, but a wrapped double keeps its precision.
inline double wrapPhase(double phase) {
    double w = std::fmod(phase, kTwoPi);
    if (w < 0.0) w += kTwoPi;
    return w >= kTwoPi ? 0.0 : w;
}

// ---- The body ---------------------------------------------------------------------------------------------------------

/// The snake in its own frame at one moment, plus how every link center moves because the shape changes.
struct Shape {
    std::array<Vec2, kLinks + 1> joint{};  // joint[k] starts link k, joint[kLinks] ends the head link
    std::array<Vec2, kLinks> center{};
    std::array<Vec2, kLinks> centerVel{};  // body-frame velocity of each center from the joint motion alone
    std::array<double, kLinks> angle{};    // orientation of each link in the body frame
    std::array<double, kLinks> angleRate{};
};

inline Shape shapeAt(const Gait& g, double phase, double omega, double linkLength = 1.0) {
    Shape s;
    double psi = 0.0;
    double dpsi = 0.0;
    Vec2 jointVel{0.0, 0.0};
    s.joint[0] = {0.0, 0.0};
    for (int k = 0; k < kLinks; ++k) {
        const auto i = static_cast<std::size_t>(k);
        s.angle[i] = psi;
        s.angleRate[i] = dpsi;
        const Vec2 along{std::cos(psi), std::sin(psi)};
        const Vec2 across{-std::sin(psi), std::cos(psi)};
        s.center[i] = s.joint[i] + along * (linkLength / 2.0);
        s.centerVel[i] = jointVel + across * (linkLength / 2.0 * dpsi);
        s.joint[i + 1] = s.joint[i] + along * linkLength;
        jointVel = jointVel + across * (linkLength * dpsi);
        if (k < kLinks - 1) {
            psi += jointAngle(g, k, phase);
            dpsi += jointRate(g, k, phase, omega);
        }
    }
    return s;
}

struct Pose {
    Vec2 pos;  // world position of the body-frame origin
    double theta = 0.0;
};

struct Twist {
    Vec2 v;  // world velocity of the body-frame origin
    double w = 0.0;
};

inline Vec2 toWorld(const Pose& pose, Vec2 local) { return pose.pos + rotate(local, pose.theta); }

inline Vec2 centerOfMass(const Shape& s, const Pose& pose) {
    Vec2 sum{0.0, 0.0};
    for (const Vec2& c : s.center) sum = sum + toWorld(pose, c);
    return sum * (1.0 / static_cast<double>(kLinks));
}

/// Direction from the tail link to the head link.
inline double heading(const Shape& s, const Pose& pose) {
    const Vec2 d = toWorld(pose, s.center[kLinks - 1]) - toWorld(pose, s.center[0]);
    return std::atan2(d.y, d.x);
}

/// The pose that puts the snake's center of mass at `com` with the given heading.
inline Pose poseFor(const Shape& s, Vec2 com, double wantedHeading) {
    const Vec2 d = s.center[kLinks - 1] - s.center[0];
    Pose pose{{0.0, 0.0}, wantedHeading - std::atan2(d.y, d.x)};
    pose.pos = com - centerOfMass(s, pose);
    return pose;
}

/// World velocity of a link center: body velocity, plus the body turning about its origin, plus the shape moving.
inline Vec2 linkVelocity(const Shape& s, const Pose& pose, const Twist& tw, int link) {
    const auto i = static_cast<std::size_t>(link);
    const Vec2 p = rotate(s.center[i], pose.theta);
    const Vec2 shape = rotate(s.centerVel[i], pose.theta);
    return tw.v + Vec2{-tw.w * p.y, tw.w * p.x} + shape;
}

struct LinkSlip {
    double along = 0.0;   // speed of the link center along the link, what a wheel allows
    double across = 0.0;  // speed across the link, what a wheel forbids
};

inline LinkSlip slipOf(const Shape& s, const Pose& pose, const Twist& tw, int link) {
    const Vec2 v = linkVelocity(s, pose, tw, link);
    const double a = pose.theta + s.angle[static_cast<std::size_t>(link)];
    return {v.x * std::cos(a) + v.y * std::sin(a), -v.x * std::sin(a) + v.y * std::cos(a)};
}

// ---- The ground ---------------------------------------------------------------------------------------------------------

namespace detail {

using Mat3 = std::array<std::array<double, 3>, 3>;
using Vec3 = std::array<double, 3>;

/// Solves m q = r by Gaussian elimination with partial pivoting. A singular matrix gives q = 0.
inline Vec3 solve3(Mat3 m, Vec3 r) {
    for (std::size_t col = 0; col < 3; ++col) {
        std::size_t pivot = col;
        for (std::size_t row = col + 1; row < 3; ++row) {
            if (std::fabs(m[row][col]) > std::fabs(m[pivot][col])) pivot = row;
        }
        std::swap(m[col], m[pivot]);
        std::swap(r[col], r[pivot]);
        const double d = m[col][col];
        if (std::fabs(d) < 1e-15) return {0.0, 0.0, 0.0};
        for (std::size_t row = col + 1; row < 3; ++row) {
            const double f = m[row][col] / d;
            for (std::size_t k = col; k < 3; ++k) m[row][k] -= f * m[col][k];
            r[row] -= f * r[col];
        }
    }
    Vec3 q{};
    for (std::size_t i = 3; i-- > 0;) {
        double sum = r[i];
        for (std::size_t k = i + 1; k < 3; ++k) sum -= m[i][k] * q[k];
        q[i] = sum / m[i][i];
    }
    return q;
}

}  // namespace detail

/// The body velocity at which total friction force and torque vanish. `ratio` = C_N / C_T, clamped to [1, kMaxRatio].
inline Twist solveTwist(const Shape& s, const Pose& pose, double ratio) {
    if (!(ratio >= 1.0)) ratio = 1.0;  // also catches NaN
    ratio = std::min(ratio, kMaxRatio);
    const double cn = 1.0;
    const double ct = 1.0 / ratio;
    detail::Mat3 a{};
    detail::Vec3 b{};
    for (std::size_t k = 0; k < static_cast<std::size_t>(kLinks); ++k) {
        const Vec2 p = rotate(s.center[k], pose.theta);
        const Vec2 shape = rotate(s.centerVel[k], pose.theta);
        const double ang = pose.theta + s.angle[k];
        const Vec2 t{std::cos(ang), std::sin(ang)};
        const Vec2 n{-t.y, t.x};
        // Velocity of center k: (ux - w py + sx, uy + w px + sy). Its t and n components are linear in (ux, uy, w).
        const detail::Vec3 rowT{t.x, t.y, p.x * t.y - p.y * t.x};
        const detail::Vec3 rowN{n.x, n.y, p.x * n.y - p.y * n.x};
        const double shapeT = shape.x * t.x + shape.y * t.y;
        const double shapeN = shape.x * n.x + shape.y * n.y;
        for (std::size_t i = 0; i < 3; ++i) {
            for (std::size_t j = 0; j < 3; ++j) a[i][j] += ct * rowT[i] * rowT[j] + cn * rowN[i] * rowN[j];
            b[i] -= ct * rowT[i] * shapeT + cn * rowN[i] * shapeN;
        }
    }
    const detail::Vec3 q = detail::solve3(a, b);
    return Twist{{q[0], q[1]}, q[2]};
}

// ---- Time ---------------------------------------------------------------------------------------------------------------

struct State {
    Pose pose;
    double phase = 0.0;  // wave phase omega t, wrapped
    double time = 0.0;
};

/// One step of length dt, midpoint rule: the body velocity is evaluated again half a step ahead.
inline void advance(State& st, const Gait& g, double omega, double ratio, double dt, double linkLength = 1.0) {
    const Twist k1 = solveTwist(shapeAt(g, st.phase, omega, linkLength), st.pose, ratio);
    const Pose mid{st.pose.pos + k1.v * (dt / 2.0), st.pose.theta + k1.w * (dt / 2.0)};
    const Twist k2 = solveTwist(shapeAt(g, st.phase + omega * dt / 2.0, omega, linkLength), mid, ratio);
    st.pose.pos = st.pose.pos + k2.v * dt;
    st.pose.theta += k2.w * dt;
    st.phase = wrapPhase(st.phase + omega * dt);
    st.time += dt;
}

/// How far a point moved over the last `window` seconds of simulated time. Stays empty until a full window has been
/// seen, so a fresh or reset meter never shows a half-measured number.
class SpeedMeter {
public:
    explicit SpeedMeter(double window) : window_(window) {}

    void reset() { samples_.clear(); }

    void push(double t, Vec2 pos) {
        samples_.push_back({t, pos});
        // Keep exactly one sample at or before t - window, it is the start of the window.
        while (samples_.size() >= 2 && samples_[1].t <= t - window_) samples_.pop_front();
    }

    std::optional<Vec2> displacement() const {
        if (samples_.size() < 2) return std::nullopt;
        const Sample& first = samples_.front();
        const Sample& last = samples_.back();
        const double span = last.t - first.t;
        if (span < window_ - 1e-9 || span <= 0.0) return std::nullopt;
        return (last.pos - first.pos) * (window_ / span);
    }

private:
    struct Sample {
        double t;
        Vec2 pos;
    };
    double window_;
    std::deque<Sample> samples_;
};

// ---- Sliders ------------------------------------------------------------------------------------------------------------

inline double sliderFraction(double value, double lo, double hi) {
    if (hi <= lo) return 0.0;
    return std::clamp((value - lo) / (hi - lo), 0.0, 1.0);
}

inline double sliderValue(double mouseX, double trackX, double trackW, double lo, double hi) {
    if (trackW <= 0.0) return lo;
    return lo + std::clamp((mouseX - trackX) / trackW, 0.0, 1.0) * (hi - lo);
}

}  // namespace serp
