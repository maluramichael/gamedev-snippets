// The physics of n-body-gravity, no SDL: forces with optional Plummer softening, three integrators, the conserved
// quantities, the start scenes and a few helpers for the picture. Covered by test.cpp.
//
// Units are chosen so that G = 1. Positions, masses and times are plain numbers; main.cpp decides how many pixels
// one unit is. Everything is double: the point of the demo is the error of the METHOD, so the rounding of the
// number type should not get in the way.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace nb {

constexpr double kG = 1.0;
constexpr std::size_t kMaxBodies = 24;
// Pairs closer than this (squared) exert no force. Only reachable with eps = 0, and it keeps 1 / r^3 finite.
constexpr double kMinDistSq = 1e-24;

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};

inline Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
inline Vec2 operator*(Vec2 a, double s) { return {a.x * s, a.y * s}; }
inline Vec2& operator+=(Vec2& a, Vec2 b) { a.x += b.x; a.y += b.y; return a; }
inline Vec2& operator-=(Vec2& a, Vec2 b) { a.x -= b.x; a.y -= b.y; return a; }
inline double dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
inline double length(Vec2 a) { return std::sqrt(dot(a, a)); }

struct Body {
    Vec2 pos;
    Vec2 vel;
    double mass = 1.0;
};

enum class Method { Euler, Leapfrog, Rk4 };

using Accel = std::array<Vec2, kMaxBodies>;

struct System {
    std::array<Body, kMaxBodies> bodies{};
    std::size_t count = 0;
    double softening = 0.0;  // Plummer length eps, 0 = textbook gravity

    /// Appends a body. Refuses a full system, a mass that is not positive and any NaN or infinity.
    bool add(const Body& b) {
        const bool finite = std::isfinite(b.pos.x) && std::isfinite(b.pos.y) && std::isfinite(b.vel.x) && std::isfinite(b.vel.y) &&
                            std::isfinite(b.mass);
        if (count >= kMaxBodies || !finite || b.mass <= 0.0) return false;
        bodies[count++] = b;
        return true;
    }
};

// ---- forces ----

namespace detail {

/// Acceleration of every body for the given positions. The pair force is symmetric, so momentum is conserved by
/// construction, whatever integrator calls this. Plummer softening: a = G m d / (d^2 + eps^2)^1.5.
inline void accelerationsAt(const std::array<Vec2, kMaxBodies>& pos, const std::array<Body, kMaxBodies>& bodies, std::size_t n, double eps,
                            Accel& out) {
    for (std::size_t i = 0; i < n; ++i) out[i] = {};
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 1; j < n; ++j) {
            const Vec2 d = pos[j] - pos[i];
            const double r2 = dot(d, d) + eps * eps;
            if (!(r2 > kMinDistSq)) continue;  // same spot without softening: direction undefined, no force
            const double inv = kG / (r2 * std::sqrt(r2));
            out[i] += d * (bodies[j].mass * inv);
            out[j] -= d * (bodies[i].mass * inv);
        }
    }
}

}  // namespace detail

inline void accelerations(const System& s, Accel& out) {
    std::array<Vec2, kMaxBodies> pos{};
    for (std::size_t i = 0; i < s.count; ++i) pos[i] = s.bodies[i].pos;
    detail::accelerationsAt(pos, s.bodies, s.count, s.softening, out);
}

// ---- integrators ----

inline void stepEuler(System& s, double dt) {
    Accel a;
    accelerations(s, a);
    for (std::size_t i = 0; i < s.count; ++i) {
        // Explicit Euler: both updates read the state at the START of the step. The position uses the old velocity.
        s.bodies[i].pos += s.bodies[i].vel * dt;
        s.bodies[i].vel += a[i] * dt;
    }
}

/// Kick-drift-kick leapfrog: second order, symplectic, time-reversible.
inline void stepLeapfrog(System& s, double dt) {
    Accel a;
    accelerations(s, a);
    for (std::size_t i = 0; i < s.count; ++i) {
        s.bodies[i].vel += a[i] * (0.5 * dt);
        s.bodies[i].pos += s.bodies[i].vel * dt;
    }
    accelerations(s, a);
    for (std::size_t i = 0; i < s.count; ++i) s.bodies[i].vel += a[i] * (0.5 * dt);
}

/// Classic Runge-Kutta of fourth order: four force evaluations per step, not symplectic.
inline void stepRk4(System& s, double dt) {
    const std::size_t n = s.count;
    std::array<Vec2, kMaxBodies> x0{};
    std::array<Vec2, kMaxBodies> v0{};
    for (std::size_t i = 0; i < n; ++i) {
        x0[i] = s.bodies[i].pos;
        v0[i] = s.bodies[i].vel;
    }
    std::array<Vec2, kMaxBodies> xs{};
    std::array<Vec2, kMaxBodies> k1x{}, k2x{}, k3x{}, k4x{};
    Accel k1v, k2v, k3v, k4v;

    for (std::size_t i = 0; i < n; ++i) k1x[i] = v0[i];
    detail::accelerationsAt(x0, s.bodies, n, s.softening, k1v);

    for (std::size_t i = 0; i < n; ++i) {
        xs[i] = x0[i] + k1x[i] * (0.5 * dt);
        k2x[i] = v0[i] + k1v[i] * (0.5 * dt);
    }
    detail::accelerationsAt(xs, s.bodies, n, s.softening, k2v);

    for (std::size_t i = 0; i < n; ++i) {
        xs[i] = x0[i] + k2x[i] * (0.5 * dt);
        k3x[i] = v0[i] + k2v[i] * (0.5 * dt);
    }
    detail::accelerationsAt(xs, s.bodies, n, s.softening, k3v);

    for (std::size_t i = 0; i < n; ++i) {
        xs[i] = x0[i] + k3x[i] * dt;
        k4x[i] = v0[i] + k3v[i] * dt;
    }
    detail::accelerationsAt(xs, s.bodies, n, s.softening, k4v);

    for (std::size_t i = 0; i < n; ++i) {
        s.bodies[i].pos = x0[i] + (k1x[i] + k2x[i] * 2.0 + k3x[i] * 2.0 + k4x[i]) * (dt / 6.0);
        s.bodies[i].vel = v0[i] + (k1v[i] + k2v[i] * 2.0 + k3v[i] * 2.0 + k4v[i]) * (dt / 6.0);
    }
}

inline void step(System& s, Method m, double dt) {
    switch (m) {
        case Method::Euler: stepEuler(s, dt); break;
        case Method::Leapfrog: stepLeapfrog(s, dt); break;
        case Method::Rk4: stepRk4(s, dt); break;
    }
}

// ---- conserved quantities and measurements ----

inline double kineticEnergy(const System& s) {
    double k = 0.0;
    for (std::size_t i = 0; i < s.count; ++i) k += 0.5 * s.bodies[i].mass * dot(s.bodies[i].vel, s.bodies[i].vel);
    return k;
}

/// Potential of the SAME force law the integrators use: with softening, -G m1 m2 / sqrt(d^2 + eps^2). Judging a
/// softened run with the textbook potential would show a drift that is not there.
inline double potentialEnergy(const System& s) {
    double u = 0.0;
    for (std::size_t i = 0; i < s.count; ++i) {
        for (std::size_t j = i + 1; j < s.count; ++j) {
            const Vec2 d = s.bodies[j].pos - s.bodies[i].pos;
            const double r2 = dot(d, d) + s.softening * s.softening;
            if (!(r2 > kMinDistSq)) continue;
            u -= kG * s.bodies[i].mass * s.bodies[j].mass / std::sqrt(r2);
        }
    }
    return u;
}

inline double totalEnergy(const System& s) { return kineticEnergy(s) + potentialEnergy(s); }

inline Vec2 momentum(const System& s) {
    Vec2 p;
    for (std::size_t i = 0; i < s.count; ++i) p += s.bodies[i].vel * s.bodies[i].mass;
    return p;
}

inline double totalMass(const System& s) {
    double m = 0.0;
    for (std::size_t i = 0; i < s.count; ++i) m += s.bodies[i].mass;
    return m;
}

inline Vec2 centerOfMass(const System& s) {
    const double m = totalMass(s);
    if (!(m > 0.0)) return {};
    Vec2 c;
    for (std::size_t i = 0; i < s.count; ++i) c += s.bodies[i].pos * s.bodies[i].mass;
    return c * (1.0 / m);
}

/// Smallest distance between any two bodies, infinity for fewer than two.
inline double minDistance(const System& s) {
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < s.count; ++i) {
        for (std::size_t j = i + 1; j < s.count; ++j) best = std::fmin(best, length(s.bodies[j].pos - s.bodies[i].pos));
    }
    return best;
}

inline bool allFinite(const System& s) {
    for (std::size_t i = 0; i < s.count; ++i) {
        const Body& b = s.bodies[i];
        if (!std::isfinite(b.pos.x) || !std::isfinite(b.pos.y) || !std::isfinite(b.vel.x) || !std::isfinite(b.vel.y)) return false;
    }
    return true;
}

/// (e - e0) / |e0|. An initial energy of exactly zero has no scale to divide by: plain difference then.
inline double relativeDrift(double e, double e0) {
    const double scale = std::fabs(e0);
    return (e - e0) / (scale > 1e-12 ? scale : 1.0);
}

/// Signed logarithm: 0 stays 0, +-c * 9 maps to +-1, a drift of 1e-9 and one of 1e+3 fit on the same axis.
inline double symlog(double x, double linThreshold) {
    if (std::isnan(x)) return 0.0;
    const double mag = std::min(std::fabs(x), 1e300);
    return std::copysign(std::log10(1.0 + mag / linThreshold), x);
}

/// Where a ray from the middle of a rectangle (half sizes hw, hh) in direction (dx, dy) leaves it.
inline Vec2 edgePoint(double hw, double hh, double dx, double dy) {
    const double ax = std::fabs(dx);
    const double ay = std::fabs(dy);
    if (ax <= 0.0 && ay <= 0.0) return {};
    // Scale so that the larger relative extent reaches exactly 1; compare dx/hw with dy/hh without dividing by 0.
    const double tx = ax > 0.0 ? hw / ax : std::numeric_limits<double>::infinity();
    const double ty = ay > 0.0 ? hh / ay : std::numeric_limits<double>::infinity();
    const double t = std::min(tx, ty);
    return {dx * t, dy * t};
}

// ---- random numbers ----

/// xorshift64*: tiny, deterministic everywhere (std::uniform_real_distribution may differ between standard libraries).
struct Rng {
    std::uint64_t state;

    std::uint64_t next() {
        if (state == 0) state = 0x9E3779B97F4A7C15ULL;  // xorshift sticks at 0
        state ^= state >> 12;
        state ^= state << 25;
        state ^= state >> 27;
        return state * 2685821657736338717ULL;
    }
    /// [0, 1) with 53 random bits.
    double uniform() { return static_cast<double>(next() >> 11) * (1.0 / 9007199254740992.0); }
};

// ---- scenes ----

enum class Scene { Figure8, Planets, Cluster, Pythagoras };
constexpr int kSceneCount = 4;
// Fixed step sizes the chips offer. The close encounters of the pythagoras scene need the smallest.
constexpr std::array<double, 3> kStepSizes{0.05, 0.01, 0.002};

struct SceneInfo {
    const char* label;
    double pxPerUnit;       // zoom
    double timeScale;       // simulated seconds per real second
    double trailSeconds;    // how far back the trail reaches
    double softening;       // eps used when the softening switch is on
    double spawnMass;       // mass of a body placed with the mouse
    double bodyScale;       // pixel radius of the heaviest body of the scene
    double prerollSeconds;  // simulated before the first frame so the trails are already there
    int defaultStep;        // index into kStepSizes the scene starts with
};

inline const SceneInfo& info(Scene s) {
    static constexpr SceneInfo kInfos[kSceneCount] = {
        {"figure 8", 170.0, 1.5, 5.0, 0.05, 0.1, 11.0, 4.0, 1},
        {"planets", 110.0, 1.5, 6.0, 0.02, 0.004, 13.0, 4.0, 1},
        {"cluster", 190.0, 0.8, 3.0, 0.05, 0.05, 7.0, 1.0, 1},
        {"pythagoras", 60.0, 2.0, 6.0, 0.1, 1.0, 9.0, 3.0, 2},
    };
    return kInfos[static_cast<int>(s)];
}

/// Moves the system into the frame where the center of mass sits at the origin and the total momentum is zero.
inline void centerSystem(System& s) {
    const double m = totalMass(s);
    if (!(m > 0.0)) return;
    const Vec2 c = centerOfMass(s);
    const Vec2 v = momentum(s) * (1.0 / m);
    for (std::size_t i = 0; i < s.count; ++i) {
        s.bodies[i].pos -= c;
        s.bodies[i].vel -= v;
    }
}

/// Scales all velocities so that K / |U| equals `ratio` (0.5 is virial equilibrium). Untouched if nothing moves or
/// there is no potential to compare with.
inline void scaleToVirialRatio(System& s, double ratio) {
    const double k = kineticEnergy(s);
    const double u = std::fabs(potentialEnergy(s));
    if (!(k > 0.0) || !(u > 0.0) || !(ratio > 0.0)) return;
    const double f = std::sqrt(ratio * u / k);
    for (std::size_t i = 0; i < s.count; ++i) s.bodies[i].vel = s.bodies[i].vel * f;
}

inline System makeScene(Scene which) {
    System s;
    switch (which) {
        case Scene::Figure8: {
            // Chenciner and Montgomery 2000: three unit masses chase each other along one figure-eight curve.
            const Vec2 p{0.97000436, -0.24308753};
            const Vec2 v3{-0.93240737, -0.86473146};
            s.add(Body{p, v3 * -0.5, 1.0});
            s.add(Body{p * -1.0, v3 * -0.5, 1.0});
            s.add(Body{{0.0, 0.0}, v3, 1.0});
            break;
        }
        case Scene::Planets: {
            // A sun of mass 1 and four toy planets on circular orbits v = sqrt(G M / r), phases spread around the
            // circle. The planets are far lighter than the sun, so it hardly moves, and they sit far enough apart
            // to stay out of each other's way for as long as you watch.
            s.add(Body{{0.0, 0.0}, {0.0, 0.0}, 1.0});
            struct P { double r, phi, mass; };
            constexpr P kPlanets[] = {{0.35, 0.0, 0.002}, {0.62, 2.1, 0.002}, {1.05, 4.0, 0.012}, {1.75, 5.3, 0.004}};
            for (const P& p : kPlanets) {
                const double v = std::sqrt(kG * 1.0 / p.r);
                s.add(Body{{p.r * std::cos(p.phi), p.r * std::sin(p.phi)}, {-v * std::sin(p.phi), v * std::cos(p.phi)}, p.mass});
            }
            centerSystem(s);
            break;
        }
        case Scene::Cluster: {
            // 14 equal masses (total 1) thrown into a disc of radius 1 with small random velocities: it collapses,
            // and the bodies have many close encounters. Fixed seed, so every visitor sees the same cluster.
            Rng rng{20261106};
            constexpr int kN = 14;
            for (int i = 0; i < kN; ++i) {
                Vec2 p;
                do {
                    p = {rng.uniform() * 2.0 - 1.0, rng.uniform() * 2.0 - 1.0};
                } while (dot(p, p) > 1.0);
                s.add(Body{p, {rng.uniform() * 2.0 - 1.0, rng.uniform() * 2.0 - 1.0}, 1.0 / kN});
            }
            centerSystem(s);
            scaleToVirialRatio(s, 0.2);
            break;
        }
        case Scene::Pythagoras: {
            // Burrau 1913: masses 3, 4, 5 at rest on the corners of a 3-4-5 triangle. Each body sits opposite the
            // side whose length equals its mass: the "3" is opposite the side of length 3, and so on.
            s.add(Body{{1.0, 3.0}, {0.0, 0.0}, 3.0});
            s.add(Body{{-2.0, -1.0}, {0.0, 0.0}, 4.0});
            s.add(Body{{1.0, -1.0}, {0.0, 0.0}, 5.0});
            break;
        }
    }
    return s;
}

}  // namespace nb
