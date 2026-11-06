// Gerstner waves without SDL and GL, so test.cpp can check them. The vertex shader in main.cpp runs the same formulas.
//
// A rest point (x, z) of the water surface is pushed around by every wave i with direction D, wave number k, amplitude A
// and steepness Q. With theta = k * dot(D, (x, z)) - omega * t + phase:
//
//     X = x + sum( Q A D.x cos(theta) )      horizontal shift: points bunch up under the crest, crests get sharp
//     Y =     sum(   A     sin(theta) )      height
//     Z = z + sum( Q A D.z cos(theta) )
//
// Deep water disperses: omega = sqrt(g k), so long waves run faster (c = omega / k = sqrt(g / k)).
// The tangents are the exact derivatives of that map, no finite differences:
//
//     Tx = d(X,Y,Z)/dx = (1 - sum(S Dx Dx sin), sum(k A Dx cos), -sum(S Dx Dz sin))      with S = Q k A
//     Tz = d(X,Y,Z)/dz = (-sum(S Dx Dz sin), sum(k A Dz cos), 1 - sum(S Dz Dz sin))
//     N  = normalize(cross(Tz, Tx))
//
// N.y before normalising is the Jacobian J = det(d(X,Z)/d(x,z)). It is 1 on flat water, falls where the surface is
// squeezed together (the foam) and goes negative once the surface folds over itself. sum(S) <= 1 keeps J >= 0.
//
// Conventions: right-handed, y up, matrices row-major m[row][col] with column vectors (v' = M * v).
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace gw {

inline constexpr float kGravity = 9.81F;
inline constexpr double kTwoPi = 2.0 * 3.14159265358979323846;
inline constexpr int kMaxWaves = 8;  // the shader's uniform arrays have this size

// ---------------------------------------------------------------------------------------------- vectors

struct Vec3 {
    float x = 0.0F, y = 0.0F, z = 0.0F;
};

inline Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator*(Vec3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
inline float dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(Vec3 a, Vec3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline float length(Vec3 a) { return std::sqrt(dot(a, a)); }
inline Vec3 normalize(Vec3 a) {
    const float l = length(a);
    return l > 0.0F ? a * (1.0F / l) : a;
}

// ---------------------------------------------------------------------------------------------- one wave

struct Wave {
    float dirX = 1.0F, dirZ = 0.0F;  // unit vector the crests travel along (in the xz plane)
    float wavelength = 10.0F;        // metres
    float amplitude = 0.5F;          // metres
    float steepness = 0.0F;          // Q: how far points are shifted sideways; 0 = plain sine wave
    float phase = 0.0F;              // start offset in radians so the waves do not line up at t = 0
};
using Waves = std::vector<Wave>;

inline float wavenumber(const Wave& w) { return w.wavelength > 0.0F ? static_cast<float>(kTwoPi) / w.wavelength : 0.0F; }

/// Deep water dispersion: angular frequency from wave number.
inline float dispersion(float k) { return std::sqrt(kGravity * std::max(k, 0.0F)); }
inline float omega(const Wave& w) { return dispersion(wavenumber(w)); }
inline double period(const Wave& w) { return omega(w) > 0.0F ? kTwoPi / static_cast<double>(omega(w)) : 0.0; }
inline float phaseSpeed(const Wave& w) { return wavenumber(w) > 0.0F ? omega(w) / wavenumber(w) : 0.0F; }

/// omega * t reduced to one turn, in double. Float loses the fraction after a few hours of running time, so the
/// CPU does this one multiplication and the shader only ever sees an angle between 0 and 2 pi.
inline double timePhase(const Wave& w, double t) {
    double p = std::fmod(static_cast<double>(omega(w)) * t, kTwoPi);
    if (p < 0.0) p += kTwoPi;
    return p;
}

/// Q * k * A: the share of the steepness budget this wave uses.
inline float sharpness(const Wave& w) { return w.steepness * wavenumber(w) * w.amplitude; }
inline float steepnessSum(const Waves& waves) {
    float sum = 0.0F;
    for (const Wave& w : waves) sum += sharpness(w);
    return sum;
}
/// sum(Q k A) <= 1 guarantees that the surface never folds over itself.
inline bool steepnessOk(const Waves& waves) { return steepnessSum(waves) <= 1.0F + 1e-6F; }
inline float maxHeight(const Waves& waves) {
    float sum = 0.0F;
    for (const Wave& w : waves) sum += w.amplitude;
    return sum;
}

// ---------------------------------------------------------------------------------------------- the surface

struct Frame {
    Vec3 position;   // where the rest point (x, z) is at time t
    Vec3 tangentX;   // d position / d x
    Vec3 tangentZ;   // d position / d z
    Vec3 rawNormal;  // cross(tangentZ, tangentX), not normalised; its y is the Jacobian
    Vec3 normal;     // unit length
    float jacobian = 1.0F;
};

inline Frame frame(const Waves& waves, float x, float z, double t) {
    Frame f;
    f.position = {x, 0.0F, z};
    f.tangentX = {1.0F, 0.0F, 0.0F};
    f.tangentZ = {0.0F, 0.0F, 1.0F};
    for (const Wave& w : waves) {
        const float k = wavenumber(w);
        const float theta = k * (w.dirX * x + w.dirZ * z) - static_cast<float>(timePhase(w, t)) + w.phase;
        const float s = std::sin(theta);
        const float c = std::cos(theta);
        const float qa = w.steepness * w.amplitude;
        f.position.x += qa * w.dirX * c;
        f.position.y += w.amplitude * s;
        f.position.z += qa * w.dirZ * c;
        const float ka = k * w.amplitude;
        const float sharp = w.steepness * ka;
        f.tangentX.x -= sharp * w.dirX * w.dirX * s;
        f.tangentX.y += ka * w.dirX * c;
        f.tangentX.z -= sharp * w.dirX * w.dirZ * s;
        f.tangentZ.x -= sharp * w.dirX * w.dirZ * s;
        f.tangentZ.y += ka * w.dirZ * c;
        f.tangentZ.z -= sharp * w.dirZ * w.dirZ * s;
    }
    f.rawNormal = cross(f.tangentZ, f.tangentX);
    f.normal = normalize(f.rawNormal);
    f.jacobian = f.tangentX.x * f.tangentZ.z - f.tangentZ.x * f.tangentX.z;
    return f;
}

inline Vec3 displace(const Waves& waves, float x, float z, double t) { return frame(waves, x, z, t).position; }
inline Vec3 normal(const Waves& waves, float x, float z, double t) { return frame(waves, x, z, t).normal; }
inline float jacobian(const Waves& waves, float x, float z, double t) { return frame(waves, x, z, t).jacobian; }

// ---------------------------------------------------------------------------------------------- floating things

struct Rest {
    float x = 0.0F, z = 0.0F;
};

/// The rest point whose displaced position is the given world position (x, z). A ship sits at a fixed world position,
/// but the formula is written in rest coordinates, so we have to invert the horizontal shift. Newton's method on the
/// 2x2 map converges in a few steps; the Jacobian of that map is exactly the J from above.
inline Rest restPoint(const Waves& waves, float worldX, float worldZ, double t, int iterations = 8) {
    Rest r{worldX, worldZ};
    for (int i = 0; i < iterations; ++i) {
        const Frame f = frame(waves, r.x, r.z, t);
        const float ex = f.position.x - worldX;
        const float ez = f.position.z - worldZ;
        if (std::fabs(ex) < 1e-6F && std::fabs(ez) < 1e-6F) break;
        if (std::fabs(f.jacobian) < 1e-3F) break;  // right at a fold there is no unique answer
        r.x -= (f.tangentZ.z * ex - f.tangentZ.x * ez) / f.jacobian;
        r.z -= (f.tangentX.x * ez - f.tangentX.z * ex) / f.jacobian;
    }
    return r;
}

struct BuoyPose {
    Vec3 position;
    Vec3 up;
};

/// A buoy moored at a fixed world position (x, z): it rides on the surface point above it and leans with the normal.
inline BuoyPose buoy(const Waves& waves, float worldX, float worldZ, double t) {
    const Rest r = restPoint(waves, worldX, worldZ, t);
    const Frame f = frame(waves, r.x, r.z, t);
    return {f.position, f.normal};
}

// ---------------------------------------------------------------------------------------------- presets and clock

struct Preset {
    float amplitude;  // scales all amplitudes
    float steepness;  // Q of every wave
};
// The base set has sum(k A) = 0.663, so the budget sum(Q k A) <= 1 allows Q = 1.5 at amplitude 1 but only 0.75 at 2.
inline constexpr Preset kCalm{0.25F, 1.0F};
inline constexpr Preset kBreeze{1.0F, 0.85F};
inline constexpr Preset kStorm{2.0F, 0.72F};

inline Preset mix(const Preset& a, const Preset& b, float s) { return {a.amplitude + (b.amplitude - a.amplitude) * s, a.steepness + (b.steepness - a.steepness) * s}; }

/// Six waves from different directions, wavelengths 22 m down to 3 m.
inline Waves makeWaves(const Preset& p) {
    struct Base {
        float angleDeg, wavelength, amplitude, phase;
    };
    static constexpr Base kBase[] = {{20.0F, 22.0F, 0.55F, 0.0F}, {-15.0F, 14.0F, 0.30F, 1.3F}, {50.0F, 9.0F, 0.18F, 2.1F},
                                     {-55.0F, 6.0F, 0.10F, 4.0F}, {5.0F, 4.0F, 0.05F, 0.7F},   {80.0F, 3.0F, 0.03F, 5.2F}};
    Waves out;
    for (const Base& b : kBase) {
        const float a = b.angleDeg * 3.14159265F / 180.0F;
        out.push_back(Wave{std::sin(a), std::cos(a), b.wavelength, b.amplitude * p.amplitude, p.steepness, b.phase});
    }
    return out;
}

/// Wave time from accumulated frame times. One value per frame goes to the GPU and to the CPU side (buoy), so both
/// see the same water. A stall (tab in the background, breakpoint) is clamped instead of jumping the waves ahead.
struct WaveClock {
    static constexpr double kMaxFrame = 0.25;
    double time = 0.0;

    /// Returns the time that was added.
    double advance(double frameSeconds, bool paused) {
        if (paused || !(frameSeconds > 0.0)) return 0.0;
        const double dt = std::min(frameSeconds, kMaxFrame);
        time += dt;
        return dt;
    }
};

// ---------------------------------------------------------------------------------------------- meshes

/// Rest positions of an n x n vertex grid centred on the origin, `size` metres wide: x0 z0 x1 z0 ... row by row.
inline std::vector<float> gridPositions(int n, float size) {
    std::vector<float> out;
    if (n < 1) return out;
    out.reserve(static_cast<std::size_t>(n) * static_cast<std::size_t>(n) * 2U);
    for (int j = 0; j < n; ++j) {
        for (int i = 0; i < n; ++i) {
            const float u = n > 1 ? static_cast<float>(i) / static_cast<float>(n - 1) : 0.5F;
            const float v = n > 1 ? static_cast<float>(j) / static_cast<float>(n - 1) : 0.5F;
            out.push_back((u - 0.5F) * size);
            out.push_back((v - 0.5F) * size);
        }
    }
    return out;
}

/// Two triangles per cell, counter clockwise seen from above. 16 bit indices: n * n must stay below 65536.
inline std::vector<std::uint16_t> gridTriangles(int n) {
    std::vector<std::uint16_t> out;
    if (n < 2) return out;
    for (int j = 0; j + 1 < n; ++j) {
        for (int i = 0; i + 1 < n; ++i) {
            const auto i0 = static_cast<std::uint16_t>(j * n + i);
            const auto i1 = static_cast<std::uint16_t>(i0 + 1);
            const auto i2 = static_cast<std::uint16_t>(i0 + n);
            const auto i3 = static_cast<std::uint16_t>(i2 + 1);
            out.insert(out.end(), {i0, i2, i1, i1, i2, i3});
        }
    }
    return out;
}

/// Every grid edge once, for the wireframe chip.
inline std::vector<std::uint16_t> gridLines(int n) {
    std::vector<std::uint16_t> out;
    if (n < 2) return out;
    for (int j = 0; j < n; ++j) {
        for (int i = 0; i < n; ++i) {
            const auto a = static_cast<std::uint16_t>(j * n + i);
            if (i + 1 < n) out.insert(out.end(), {a, static_cast<std::uint16_t>(a + 1)});
            if (j + 1 < n) out.insert(out.end(), {a, static_cast<std::uint16_t>(a + n)});
        }
    }
    return out;
}

struct Mesh {
    std::vector<Vec3> v;
    std::vector<std::uint16_t> idx;
};

/// The buoy: a lathe of a profile (radius, height) around the y axis. y = 0 is the waterline, the hull reaches below it.
inline Mesh buoyMesh(int segments) {
    static constexpr float kProfile[][2] = {{0.0F, -0.42F}, {0.22F, -0.38F}, {0.42F, -0.22F}, {0.52F, 0.0F}, {0.52F, 0.18F},
                                            {0.42F, 0.36F}, {0.20F, 0.50F},  {0.07F, 0.56F},  {0.07F, 1.45F}, {0.0F, 1.62F}};
    constexpr int rings = static_cast<int>(sizeof kProfile / sizeof kProfile[0]);
    Mesh m;
    if (segments < 3) return m;
    for (const auto& p : kProfile) {
        for (int j = 0; j < segments; ++j) {
            const float a = static_cast<float>(kTwoPi) * static_cast<float>(j) / static_cast<float>(segments);
            m.v.push_back({p[0] * std::cos(a), p[1], p[0] * std::sin(a)});
        }
    }
    for (int i = 0; i + 1 < rings; ++i) {
        for (int j = 0; j < segments; ++j) {
            const int j2 = (j + 1) % segments;
            const auto a = static_cast<std::uint16_t>(i * segments + j);
            const auto b = static_cast<std::uint16_t>(i * segments + j2);
            const auto c = static_cast<std::uint16_t>((i + 1) * segments + j);
            const auto d = static_cast<std::uint16_t>((i + 1) * segments + j2);
            m.idx.insert(m.idx.end(), {a, c, b, c, d, b});
        }
    }
    return m;
}

// ---------------------------------------------------------------------------------------------- matrices and camera

struct Vec4 {
    float x = 0.0F, y = 0.0F, z = 0.0F, w = 0.0F;
};

struct Mat4 {
    float m[4][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
};

inline Mat4 operator*(const Mat4& a, const Mat4& b) {
    Mat4 r;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            float sum = 0.0F;
            for (int k = 0; k < 4; ++k) sum += a.m[i][k] * b.m[k][j];
            r.m[i][j] = sum;
        }
    }
    return r;
}

inline Vec4 operator*(const Mat4& a, Vec4 v) {
    return {a.m[0][0] * v.x + a.m[0][1] * v.y + a.m[0][2] * v.z + a.m[0][3] * v.w, a.m[1][0] * v.x + a.m[1][1] * v.y + a.m[1][2] * v.z + a.m[1][3] * v.w,
            a.m[2][0] * v.x + a.m[2][1] * v.y + a.m[2][2] * v.z + a.m[2][3] * v.w, a.m[3][0] * v.x + a.m[3][1] * v.y + a.m[3][2] * v.z + a.m[3][3] * v.w};
}

inline Mat4 translate(Vec3 t) {
    Mat4 r;
    r.m[0][3] = t.x;
    r.m[1][3] = t.y;
    r.m[2][3] = t.z;
    return r;
}

inline Mat4 scale(float s) {
    Mat4 r;
    r.m[0][0] = r.m[1][1] = r.m[2][2] = s;
    return r;
}

/// Rotation that turns +y into the unit vector `up` by the shortest way (Rodrigues' formula).
inline Mat4 alignUp(Vec3 up) {
    Mat4 r;
    const float c = up.y;
    if (c > 0.999999F) return r;
    if (c < -0.999999F) {  // upside down: half a turn around x
        r.m[1][1] = -1.0F;
        r.m[2][2] = -1.0F;
        return r;
    }
    const Vec3 v{up.z, 0.0F, -up.x};  // cross((0,1,0), up)
    const float k = 1.0F / (1.0F + c);
    const float vv[3] = {v.x, v.y, v.z};
    const float skew[3][3] = {{0, -vv[2], vv[1]}, {vv[2], 0, -vv[0]}, {-vv[1], vv[0], 0}};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            float sq = 0.0F;
            for (int l = 0; l < 3; ++l) sq += skew[i][l] * skew[l][j];
            r.m[i][j] = (i == j ? 1.0F : 0.0F) + skew[i][j] + sq * k;
        }
    }
    return r;
}

inline Mat4 perspective(float fovY, float aspect, float zNear, float zFar) {
    const float f = 1.0F / std::tan(fovY / 2.0F);
    Mat4 r;
    r.m[0][0] = f / aspect;
    r.m[1][1] = f;
    r.m[2][2] = (zFar + zNear) / (zNear - zFar);
    r.m[2][3] = 2.0F * zFar * zNear / (zNear - zFar);
    r.m[3][2] = -1.0F;
    r.m[3][3] = 0.0F;
    return r;
}

inline Mat4 lookAt(Vec3 eye, Vec3 target, Vec3 up) {
    const Vec3 f = normalize(target - eye);
    const Vec3 s = normalize(cross(f, up));
    const Vec3 u = cross(s, f);
    Mat4 r;
    r.m[0][0] = s.x; r.m[0][1] = s.y; r.m[0][2] = s.z; r.m[0][3] = -dot(s, eye);
    r.m[1][0] = u.x; r.m[1][1] = u.y; r.m[1][2] = u.z; r.m[1][3] = -dot(u, eye);
    r.m[2][0] = -f.x; r.m[2][1] = -f.y; r.m[2][2] = -f.z; r.m[2][3] = dot(f, eye);
    return r;
}

inline constexpr float kFovY = 0.8F;
inline constexpr float kNear = 0.3F;
inline constexpr float kFar = 400.0F;
inline constexpr float kMinDistance = 7.0F;
inline constexpr float kMaxDistance = 48.0F;
inline constexpr float kMaxPitch = 1.45F;
inline constexpr float kMinEyeHeight = 3.0F;  // above the mean water level; the storm crests reach 2.6 m

struct Basis {
    Vec3 right, up, forward;
};

struct OrbitCamera {
    float yaw = 0.35F;
    float pitch = 0.2F;
    float distance = 24.0F;
    Vec3 target{0.0F, 0.3F, 0.0F};

    /// Lowest pitch at this distance that keeps the eye above kMinEyeHeight, so the camera never dives into a wave.
    float minPitch() const { return std::asin(std::clamp((kMinEyeHeight - target.y) / distance, 0.0F, 0.95F)); }

    Vec3 eye() const { return target + Vec3{distance * std::cos(pitch) * std::sin(yaw), distance * std::sin(pitch), distance * std::cos(pitch) * std::cos(yaw)}; }
    Mat4 view() const { return lookAt(eye(), target, {0.0F, 1.0F, 0.0F}); }
    Basis basis() const {
        const Vec3 f = normalize(target - eye());
        const Vec3 r = normalize(cross(f, {0.0F, 1.0F, 0.0F}));
        return {r, cross(r, f), f};
    }

    void orbit(float dYaw, float dPitch) {
        yaw += dYaw;
        pitch = std::clamp(pitch + dPitch, minPitch(), kMaxPitch);
    }
    /// One wheel notch moves the camera 10 % closer (negative notches move away).
    void zoom(float notches) {
        distance = std::clamp(distance * std::pow(0.9F, std::clamp(notches, -50.0F, 50.0F)), kMinDistance, kMaxDistance);
        orbit(0.0F, 0.0F);  // closer means the minimum pitch gets steeper
    }
};

}  // namespace gw
