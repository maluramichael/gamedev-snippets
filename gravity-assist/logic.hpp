// The maths of a gravity assist, no SDL.
//
// The sun is not simulated. Over the few seconds of an encounter its pull on the probe and on the planet is the same,
// so it cancels and the planet simply flies in a straight line at planetSpeed through the "sun frame" (the frame in
// which the sun stands still). The probe is a test mass: it feels the planet, the planet does not feel it.
//
// Everything is described in the planet frame and then boosted: a Launch is a start position and a velocity measured
// from the planet. The sun frame sees the same flight with the planet's velocity added to the probe, so changing
// planetSpeed changes only what the sun sees, never what the planet sees.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace ga {

constexpr double kPi = 3.14159265358979323846;

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};

inline Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
inline Vec2 operator*(Vec2 a, double s) { return {a.x * s, a.y * s}; }
inline double dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
inline double cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }
inline double length(Vec2 a) { return std::hypot(a.x, a.y); }

/// Unit vector, the zero vector for a vector that has no direction.
inline Vec2 normalized(Vec2 a) {
    const double l = length(a);
    return l > 1e-12 ? a * (1.0 / l) : Vec2{};
}

inline Vec2 rotated(Vec2 a, double angle) {
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    return {a.x * c - a.y * s, a.x * s + a.y * c};
}

/// Signed angle from a to b in (-pi, pi]; 0 when one of them has no direction.
inline double signedAngle(Vec2 a, Vec2 b) {
    if (length(a) < 1e-12 || length(b) < 1e-12) return 0.0;
    return std::atan2(cross(a, b), dot(a, b));
}

// ----------------------------------------------------------------------------------------------------------- the world

struct World {
    double mu = 90000.0;         // gravitational parameter of the planet (px^3 / s^2)
    double planetSpeed = 60.0;   // the planet flies along +x at this speed in the sun frame (px / s)
    double meetTime = 2.6;       // the planet is at meetPoint at this time, whatever its speed
    Vec2 meetPoint{0.0, 0.0};
    double zone = 100.0;         // radius of the circle where the speeds are read off (px)
    double planetRadius = 8.0;   // closer than this the probe has crashed
    double softening = 1.5;      // Plummer softening, keeps 1 / r^2 finite
};

constexpr std::array<double, 3> kMasses{45000.0, 90000.0, 180000.0};
constexpr std::array<double, 4> kSpeeds{0.0, 30.0, 60.0, 90.0};
constexpr int kPresetCount = 3;
constexpr double kStartMargin = 4.0;  // a launch must start this far outside the zone

inline Vec2 planetVelocity(const World& w) { return {w.planetSpeed, 0.0}; }

/// Where the planet is at time t in the sun frame. Choosing the position by the meeting time keeps an encounter at the
/// same place when the speed changes.
inline Vec2 planetPosition(const World& w, double t) { return w.meetPoint + planetVelocity(w) * (t - w.meetTime); }

/// Pull of the planet on a probe at `pos` at time t: -mu d / (|d|^2 + eps^2)^1.5.
inline Vec2 acceleration(const World& w, Vec2 pos, double t) {
    const Vec2 d = pos - planetPosition(w, t);
    const double r2 = d.x * d.x + d.y * d.y + w.softening * w.softening;
    if (r2 <= 0.0) return {};
    return d * (-w.mu / (r2 * std::sqrt(r2)));
}

struct State {
    Vec2 pos;
    Vec2 vel;
};

/// A launch in the planet frame: start relative to the planet at t = 0 and velocity relative to the planet.
struct Launch {
    Vec2 rel;
    Vec2 u;
};

inline State worldState(const World& w, const Launch& l) { return {planetPosition(w, 0.0) + l.rel, planetVelocity(w) + l.u}; }

inline Launch relativeLaunch(const World& w, Vec2 pos, Vec2 vel) { return {pos - planetPosition(w, 0.0), vel - planetVelocity(w)}; }

inline bool startAllowed(const World& w, Vec2 rel) { return length(rel) > w.zone + kStartMargin; }

/// 0: pass behind the planet (speeds up), 1: pass in front (slows down), 2: start almost at rest and let the planet carry it.
inline Launch preset(int index) {
    switch (index) {
        case 0: return {{-34.0, 150.0}, {0.0, -60.0}};
        case 1: return {{34.0, 150.0}, {0.0, -60.0}};
        default: return {{290.0, 40.0}, {-55.0, 0.0}};
    }
}

// --------------------------------------------------------------------------------------------------------- the flight

struct Limits {
    double step = 0.005;     // fixed time step (s)
    double maxTime = 30.0;   // flights never run longer than this
    double tail = 4.5;       // how long a flight goes on after it left the zone
    double bound = 900.0;    // a probe farther from the origin than this, and moving away from everything, is gone
};

struct Sample {
    double t = 0.0;
    Vec2 pos;
    Vec2 vel;
};

enum class Outcome { Flyby, Miss, Crash, Bound };

struct Flight {
    std::vector<Sample> samples;  // sample i belongs to t = i * step
    double step = 0.005;
    Outcome outcome = Outcome::Miss;
    double closest = std::numeric_limits<double>::infinity();  // smallest distance to the planet
    long enter = -1;                                           // first sample inside the zone
    long exit = -1;                                            // first sample outside the zone after that
};

inline double duration(const Flight& f) { return f.samples.empty() ? 0.0 : f.samples.back().t; }

/// Velocity Verlet with the planet's position looked up at the new time. The whole flight is computed in one go: the
/// prediction line and the animation are the same data, so they cannot disagree. The planet keeps a constant velocity,
/// so the planet frame is inertial and the energy of the probe in it is conserved.
inline void simulate(const World& w, const State& start, Flight& f, const Limits& lim = {}) {
    f.samples.clear();
    f.step = lim.step;
    f.outcome = Outcome::Miss;
    f.enter = -1;
    f.exit = -1;
    State s = start;
    f.samples.push_back({0.0, s.pos, s.vel});
    f.closest = length(s.pos - planetPosition(w, 0.0));
    const Vec2 planetV = planetVelocity(w);
    // Total energy per mass in the planet frame at the zone edge decides whether the probe can ever leave.
    auto bindingEnergy = [&](Vec2 pos, Vec2 vel, double t) {
        const Vec2 d = pos - planetPosition(w, t);
        const Vec2 u = vel - planetV;
        return 0.5 * dot(u, u) - w.mu / std::sqrt(dot(d, d) + w.softening * w.softening + 1e-12);
    };
    bool bound = false;
    long lastStep = std::lround(lim.maxTime / lim.step);
    if (f.closest <= w.zone) {
        f.enter = 0;
        bound = bindingEnergy(s.pos, s.vel, 0.0) < 0.0;
    }
    Vec2 a = acceleration(w, s.pos, 0.0);
    for (long i = 1; i <= lastStep; ++i) {
        const double h = lim.step;
        const double t = static_cast<double>(i) * h;
        s.pos = s.pos + s.vel * h + a * (0.5 * h * h);
        const Vec2 next = acceleration(w, s.pos, t);
        s.vel = s.vel + (a + next) * (0.5 * h);
        a = next;
        f.samples.push_back({t, s.pos, s.vel});
        const double d = length(s.pos - planetPosition(w, t));
        f.closest = std::min(f.closest, d);
        if (d < w.planetRadius) {
            f.outcome = Outcome::Crash;
            return;
        }
        if (f.enter < 0 && d <= w.zone) {
            f.enter = i;
            bound = bindingEnergy(s.pos, s.vel, t) < 0.0;
        } else if (f.enter >= 0 && f.exit < 0 && d > w.zone && !bound) {
            f.exit = i;
            lastStep = std::min(lastStep, i + std::lround(lim.tail / lim.step));
        }
        // Not yet near the planet and flying away from the origin and from the planet: nothing more will happen.
        if (f.enter < 0 && length(s.pos) > lim.bound && dot(s.pos, s.vel) > 0.0 && dot(s.pos - planetPosition(w, t), s.vel - planetV) > 0.0) break;
    }
    f.outcome = f.exit >= 0 ? Outcome::Flyby : (f.enter >= 0 ? Outcome::Bound : Outcome::Miss);
}

/// The sample where the path between a and b crosses the zone circle (linear in the distance, which is plenty at 5 ms).
inline Sample crossing(const World& w, const Sample& a, const Sample& b) {
    const double da = length(a.pos - planetPosition(w, a.t));
    const double db = length(b.pos - planetPosition(w, b.t));
    const double span = da - db;
    const double f = std::fabs(span) < 1e-12 ? 0.5 : std::clamp((da - w.zone) / span, 0.0, 1.0);
    return {a.t + (b.t - a.t) * f, a.pos + (b.pos - a.pos) * f, a.vel + (b.vel - a.vel) * f};
}

/// Position and velocity at time t, linear between the two nearest samples, clamped to the flight.
inline Sample sampleAt(const Flight& f, double t) {
    if (f.samples.empty()) return {};
    if (t <= 0.0) return f.samples.front();
    if (t >= f.samples.back().t || f.samples.size() < 2) return f.samples.back();
    const double x = t / f.step;
    const long i = std::clamp(static_cast<long>(std::floor(x)), 0L, static_cast<long>(f.samples.size()) - 2);
    const double frac = std::clamp(x - static_cast<double>(i), 0.0, 1.0);
    const Sample& a = f.samples[static_cast<std::size_t>(i)];
    const Sample& b = f.samples[static_cast<std::size_t>(i) + 1];
    return {t, a.pos + (b.pos - a.pos) * frac, a.vel + (b.vel - a.vel) * frac};
}

// --------------------------------------------------------------------------------------------------------- the numbers

/// What the probe had when it entered the zone and when it left it, seen from both frames.
struct Encounter {
    bool valid = false;  // entered and left the zone
    double tIn = 0.0;
    double tOut = 0.0;
    Vec2 vIn, vOut;      // sun frame
    Vec2 uIn, uOut;      // planet frame
    double sunIn = 0.0;
    double sunOut = 0.0;
    double planetIn = 0.0;
    double planetOut = 0.0;
    double gain = 0.0;   // sunOut - sunIn
    double turn = 0.0;   // signed angle from uIn to uOut in radians
};

inline Encounter analyze(const World& w, const Flight& f) {
    Encounter e;
    if (f.enter < 0 || f.exit < 1 || f.exit >= static_cast<long>(f.samples.size())) return e;
    const auto at = [&](long i) -> const Sample& { return f.samples[static_cast<std::size_t>(i)]; };
    const Sample in = f.enter == 0 ? at(0) : crossing(w, at(f.enter - 1), at(f.enter));
    const Sample out = crossing(w, at(f.exit - 1), at(f.exit));
    const Vec2 planet = planetVelocity(w);
    e.valid = true;
    e.tIn = in.t;
    e.tOut = out.t;
    e.vIn = in.vel;
    e.vOut = out.vel;
    e.uIn = in.vel - planet;
    e.uOut = out.vel - planet;
    e.sunIn = length(e.vIn);
    e.sunOut = length(e.vOut);
    e.planetIn = length(e.uIn);
    e.planetOut = length(e.uOut);
    e.gain = e.sunOut - e.sunIn;
    e.turn = signedAngle(e.uIn, e.uOut);
    return e;
}

// ------------------------------------------------------------------------------------------------ the velocity diagram

/// Fits the velocity diagram into a box: the origin (speed 0), the planet's velocity and the circle of radius uLen around
/// its tip (every velocity the probe can have after a turn of any angle). map() gives box coordinates.
struct DiagramFit {
    double scale = 1.0;
    Vec2 origin;
    Vec2 map(Vec2 p) const { return origin + p * scale; }
};

inline DiagramFit fitDiagram(Vec2 planet, double uLen, double width, double height, double margin) {
    const double minX = std::min(0.0, planet.x - uLen);
    const double maxX = std::max(0.0, planet.x + uLen);
    const double minY = std::min(0.0, planet.y - uLen);
    const double maxY = std::max(0.0, planet.y + uLen);
    const double w = maxX - minX;
    const double h = maxY - minY;
    const double availW = std::max(1.0, width - 2.0 * margin);
    const double availH = std::max(1.0, height - 2.0 * margin);
    double scale = std::numeric_limits<double>::infinity();
    if (w > 1e-9) scale = std::min(scale, availW / w);
    if (h > 1e-9) scale = std::min(scale, availH / h);
    if (!std::isfinite(scale) || scale <= 0.0) scale = 1.0;
    DiagramFit fit;
    fit.scale = scale;
    fit.origin = {margin + (availW - w * scale) / 2.0 - minX * scale, margin + (availH - h * scale) / 2.0 - minY * scale};
    return fit;
}

}  // namespace ga
