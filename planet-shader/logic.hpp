// The planet shader's math without SDL and GL, so test.cpp can check it. The fragment shader in main.cpp repeats these
// functions in GLSL (hash, noise, fbm, mapping, ray, glow, bands). The constants below are pasted into the shader source
// at startup, so numbers cannot drift apart; the code itself has to be kept in sync by hand.
//
// A planet is a ball of radius 1 at the origin. Every pixel shoots a ray from the camera. If it hits the ball, the hit point
// (a unit vector, the direction from the centre) is turned into the planet's own frame (spin and axis tilt) and fed to a
// noise function. That is the whole trick: the noise is 3D, so every point of the surface has a value and neighbouring
// points have neighbouring values. The naive way maps longitude and latitude to a flat noise texture, which breaks at the
// seam (longitude +-pi) and at the poles; Mapping::LatLong reproduces exactly that so the demo can show it.
//
// Conventions: right-handed, y up, camera looks down -z at yaw 0. Screen y points down.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace gd::planet {

inline constexpr float kPi = 3.14159265358979323846F;
inline constexpr double kTwoPi = 2.0 * 3.14159265358979323846;

// ---------------------------------------------------------------------------------------------- shared constants

inline constexpr float kBaseFreq = 1.5F;     // noise cells per radian of arc on the first octave
inline constexpr int kMaxOctaves = 8;
inline constexpr float kSliceZ = 3.10F;      // naive mapping: the flat map is a slice through 3D noise at this z (picked so that the seam cuts through land)
inline constexpr float kHeightGain = 1.2F;   // elevation = 0.5 + gain * fbm; fbm spreads about 0.15 at 8 octaves, so elevation sits mostly in 0.2 .. 0.8
inline constexpr float kTilt = 0.41F;        // axis tilt in radians
inline constexpr float kAtmoRadius = 1.08F;  // top of the glow shell, in planet radii
inline constexpr int kLatticeOffset = 4096;  // keeps lattice coordinates positive, so no negative-to-unsigned casts

// Bands of the land, as fractions of the height above sea level (see landHeight), and the depth of the shallows.
inline constexpr float kDeepDepth = 0.12F;
inline constexpr float kBeachTop = 0.03F;
inline constexpr float kLowTop = 0.30F;
inline constexpr float kHighTop = 0.50F;
inline constexpr float kRockTop = 0.70F;

// Camera.
inline constexpr float kTanHalfFov = 0.45F;
inline constexpr float kAspect = 16.0F / 9.0F;
inline constexpr float kMaxPitch = 1.4F;
inline constexpr float kMinDistance = 1.6F;
inline constexpr float kMaxDistance = 8.0F;
inline constexpr float kCanvasW = 960.0F;
inline constexpr float kCanvasH = 540.0F;
inline constexpr float kCenterY = 241.0F;   // canvas row of the planet's centre: the middle of the free band between title bar and chips

// ---------------------------------------------------------------------------------------------- vectors

struct Vec3 {
    float x = 0.0F, y = 0.0F, z = 0.0F;
};

inline constexpr Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline constexpr Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline constexpr Vec3 operator*(Vec3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
inline constexpr float dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline constexpr Vec3 cross(Vec3 a, Vec3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline float length(Vec3 a) { return std::sqrt(dot(a, a)); }
inline Vec3 normalize(Vec3 a) {
    const float l = length(a);
    return l > 0.0F ? a * (1.0F / l) : a;
}

inline float clamp01(float v) { return std::clamp(v, 0.0F, 1.0F); }

inline float smoothstep(float a, float b, float x) {
    const float t = clamp01((x - a) / (b - a));
    return t * t * (3.0F - 2.0F * t);
}

// ---------------------------------------------------------------------------------------------- hash and gradient noise

/// 32-bit integer hash (xorshift-multiply). Unsigned arithmetic wraps the same way in C++ and GLSL, so the shader and this
/// header produce identical bits. hash32(0) is 0, which is why lattice coordinates are shifted by kLatticeOffset.
inline constexpr std::uint32_t hash32(std::uint32_t x) {
    x ^= x >> 16U;
    x *= 0x7feb352dU;
    x ^= x >> 15U;
    x *= 0x846ca68bU;
    x ^= x >> 16U;
    return x;
}

/// Hash of one lattice corner. Nested so that the corners of a cell can share the inner hashes (see noise3).
inline constexpr std::uint32_t hashCell(int x, int y, int z) {
    return hash32(static_cast<std::uint32_t>(x) + hash32(static_cast<std::uint32_t>(y) + hash32(static_cast<std::uint32_t>(z))));
}

/// Perlin's quintic: 6t^5 - 15t^4 + 10t^3. Flat at both ends in the first and second derivative, so cell borders leave no crease.
inline constexpr float fade(float t) { return t * t * t * (t * (t * 6.0F - 15.0F) + 10.0F); }

/// Dot product of the offset (x, y, z) with one of 12 edge directions of a cube, picked by the low 4 bits of the hash
/// (Perlin's improved noise). 16 values, four directions twice.
inline constexpr float gradient(std::uint32_t h, float x, float y, float z) {
    const std::uint32_t k = h & 15U;
    const float u = k < 8U ? x : y;
    const float v = k < 4U ? y : ((k == 12U || k == 14U) ? x : z);
    return (((k & 1U) == 0U) ? u : -u) + (((k & 2U) == 0U) ? v : -v);
}

/// 3D gradient noise in about [-1, 1], exactly 0 on every lattice point. Works for |coordinates| below kLatticeOffset.
inline float noise3(Vec3 p) {
    const float fx = std::floor(p.x), fy = std::floor(p.y), fz = std::floor(p.z);
    const float x = p.x - fx, y = p.y - fy, z = p.z - fz;
    const auto xi = static_cast<std::uint32_t>(static_cast<int>(fx) + kLatticeOffset);
    const auto yi = static_cast<std::uint32_t>(static_cast<int>(fy) + kLatticeOffset);
    const auto zi = static_cast<std::uint32_t>(static_cast<int>(fz) + kLatticeOffset);
    // Same as hashCell for the 8 corners, but the inner hashes are computed once: 2 + 4 + 8 = 14 instead of 24.
    const std::uint32_t z0 = hash32(zi), z1 = hash32(zi + 1U);
    const std::uint32_t y00 = hash32(yi + z0), y10 = hash32(yi + 1U + z0), y01 = hash32(yi + z1), y11 = hash32(yi + 1U + z1);
    const float a = gradient(hash32(xi + y00), x, y, z);
    const float b = gradient(hash32(xi + 1U + y00), x - 1.0F, y, z);
    const float c = gradient(hash32(xi + y10), x, y - 1.0F, z);
    const float d = gradient(hash32(xi + 1U + y10), x - 1.0F, y - 1.0F, z);
    const float e = gradient(hash32(xi + y01), x, y, z - 1.0F);
    const float f = gradient(hash32(xi + 1U + y01), x - 1.0F, y, z - 1.0F);
    const float g = gradient(hash32(xi + y11), x, y - 1.0F, z - 1.0F);
    const float h = gradient(hash32(xi + 1U + y11), x - 1.0F, y - 1.0F, z - 1.0F);
    const float u = fade(x), v = fade(y), w = fade(z);
    auto lerp = [](float p0, float p1, float t) { return p0 + (p1 - p0) * t; };
    return lerp(lerp(lerp(a, b, u), lerp(c, d, u), v), lerp(lerp(e, f, u), lerp(g, h, u), v), w);
}

// ---------------------------------------------------------------------------------------------- fbm

/// Next octave: the point is turned by a fixed rotation (so the lattices of the octaves do not line up), scaled by the
/// lacunarity 2 and shifted (so the lattice zero is not at the same place in every octave).
inline Vec3 octaveStep(Vec3 p) {
    return {2.0F * (0.80F * p.y + 0.60F * p.z) + 1.37F, 2.0F * (-0.80F * p.x + 0.36F * p.y - 0.48F * p.z) + 2.11F,
            2.0F * (-0.60F * p.x - 0.48F * p.y + 0.64F * p.z) + 0.73F};
}

/// Weight of octave i when `lod` octaves are in: 1 for the full ones, a fraction for the one that fades in, 0 after.
inline float octaveWeight(int i, float lod) { return clamp01(lod - static_cast<float>(i)); }

/// Fractal sum of `octaves` noise layers with halving amplitude, divided by the sum of all amplitudes so the result stays
/// in [-1, 1] whatever the octave count. Octaves beyond `lod` are faded out (see lodOctaves) but still count in the divisor,
/// so zooming out only makes the planet calmer and does not move the coastline.
inline float fbm(Vec3 p, int octaves, float lod) {
    octaves = std::clamp(octaves, 0, kMaxOctaves);
    float sum = 0.0F, norm = 0.0F, amplitude = 1.0F;
    for (int i = 0; i < octaves; ++i) {
        const float w = octaveWeight(i, lod);
        if (w > 0.0F) sum += w * amplitude * noise3(p);
        norm += amplitude;
        amplitude *= 0.5F;
        p = octaveStep(p);
    }
    return norm > 0.0F ? sum / norm : 0.0F;
}

/// How many octaves a pixel can show. An octave is useful while its cells are at least two pixels wide
/// (1 / (kBaseFreq * 2^i) >= 2 * pixelWorld); more would only shimmer. Returned as a float: the last octave fades in.
inline float lodOctaves(float pixelWorld) {
    if (!(pixelWorld > 0.0F)) return static_cast<float>(kMaxOctaves);
    const float m = std::log2(1.0F / (2.0F * kBaseFreq * pixelWorld));
    return std::clamp(m + 1.0F, 0.0F, static_cast<float>(kMaxOctaves));
}

// ---------------------------------------------------------------------------------------------- mapping and elevation

enum class Mapping { Sphere, LatLong };

/// Where in noise space does a point of the planet (unit vector, planet frame) look up its value?
/// Sphere: the point itself, scaled. LatLong: longitude and latitude as flat coordinates, which is what a 2D noise texture
/// wrapped around a ball does. The scale is the same at the equator so both look alike there.
inline Vec3 noiseCoord(Vec3 u, Mapping mapping) {
    if (mapping == Mapping::Sphere) return u * kBaseFreq;
    const float lon = std::atan2(u.z, u.x);  // -pi .. pi, the jump is at -x
    const float lat = std::asin(std::clamp(u.y, -1.0F, 1.0F));
    return Vec3{lon, lat, kSliceZ} * kBaseFreq;
}

/// Height of the surface at a point, about 0..1 (a bell curve around 0.5).
inline float elevation(Vec3 u, Mapping mapping, int octaves, float lod) {
    return 0.5F + kHeightGain * fbm(noiseCoord(u, mapping), octaves, lod);
}

/// A point on the seam (longitude pi) at a given latitude, planet frame.
inline Vec3 seamPoint(float lat) { return {-std::cos(lat), std::sin(lat), 0.0F}; }

/// Evenly spread points on the sphere (golden angle spiral). i in [0, n).
inline Vec3 fibonacciPoint(int i, int n) {
    const float y = 1.0F - 2.0F * (static_cast<float>(i) + 0.5F) / static_cast<float>(n);
    const float r = std::sqrt(std::max(0.0F, 1.0F - y * y));
    const float phi = static_cast<float>(i) * 2.39996323F;  // pi * (3 - sqrt(5))
    return {r * std::cos(phi), y, r * std::sin(phi)};
}

/// The value with `share` of the numbers strictly below it. share <= 0 gives the minimum, share >= 1 something above the maximum.
inline float quantileLevel(std::vector<float> values, float share) {
    if (values.empty()) return 0.0F;
    std::sort(values.begin(), values.end());
    const float scaled = std::max(share, 0.0F) * static_cast<float>(values.size());
    const auto index = static_cast<std::size_t>(scaled);
    if (index >= values.size()) return std::nextafter(values.back(), 1e30F);
    return values[index];
}

/// Sea level that puts `share` of the planet under water, measured on 6000 evenly spread points.
inline float seaLevelFor(float share, Mapping mapping, int octaves) {
    constexpr int kSamples = 6000;
    std::vector<float> e(kSamples);
    for (int i = 0; i < kSamples; ++i) e[static_cast<std::size_t>(i)] = elevation(fibonacciPoint(i, kSamples), mapping, octaves, static_cast<float>(kMaxOctaves));
    return quantileLevel(std::move(e), share);
}

// ---------------------------------------------------------------------------------------------- colour bands

enum class Band { Deep, Shallow, Beach, Lowland, Highland, Rock, Snow, Ice };

/// Height above the sea as 0..1 of the room that is left up to the top (0 at the coast, 1 at the highest possible peak).
inline float landHeight(float e, float sea) { return clamp01((e - sea) / (1.0F - sea)); }

/// The hard version of the colour bands. The shader blends between them; the thresholds are the same.
/// `y` is the planet-frame y of the point (sin of the latitude), `iceLatitude` the |y| from which the cap starts.
inline Band bandOf(float e, float sea, float y, float iceLatitude) {
    if (std::fabs(y) >= iceLatitude) return Band::Ice;
    if (e < sea - kDeepDepth) return Band::Deep;
    if (e < sea) return Band::Shallow;
    const float t = landHeight(e, sea);
    if (t < kBeachTop) return Band::Beach;
    if (t < kLowTop) return Band::Lowland;
    if (t < kHighTop) return Band::Highland;
    if (t < kRockTop) return Band::Rock;
    return Band::Snow;
}

struct Preset {
    const char* name;
    float oceanShare;                // how much of the surface is below the sea (or lava) level
    std::array<Vec3, 8> palette;     // sRGB: deep, shallow, beach, lowland wet, lowland dry, rock, snow, polar ice
    float iceLatitude;               // |y| where the polar caps start
    float clouds;                    // 0..1 cover
    Vec3 cloudColor;
    Vec3 atmosphere;                 // glow colour
    float atmosphereStrength;
    float gloss;                     // sun glint on the sea, 0 = matte
    float lava;                      // 1: the lowest part glows by itself
    float cities;                    // 1: lights on the night side
};

inline constexpr std::array<Preset, 4> kPresets{{
    {"Earth", 0.70F,
     {{{0.03F, 0.11F, 0.31F}, {0.08F, 0.38F, 0.58F}, {0.80F, 0.74F, 0.52F}, {0.16F, 0.40F, 0.13F}, {0.55F, 0.50F, 0.28F}, {0.38F, 0.34F, 0.30F},
       {0.95F, 0.96F, 0.98F}, {0.92F, 0.96F, 1.00F}}},
     0.86F, 0.50F, {1.00F, 1.00F, 1.00F}, {0.30F, 0.55F, 1.00F}, 1.0F, 1.0F, 0.0F, 1.0F},
    {"Desert", 0.35F,
     {{{0.16F, 0.07F, 0.06F}, {0.30F, 0.13F, 0.09F}, {0.55F, 0.30F, 0.18F}, {0.66F, 0.34F, 0.19F}, {0.78F, 0.50F, 0.30F}, {0.42F, 0.24F, 0.18F},
       {0.82F, 0.66F, 0.55F}, {0.94F, 0.94F, 0.97F}}},
     0.93F, 0.0F, {0.85F, 0.65F, 0.50F}, {0.90F, 0.60F, 0.42F}, 0.45F, 0.0F, 0.0F, 0.0F},
    {"Ice", 0.50F,
     {{{0.22F, 0.38F, 0.58F}, {0.50F, 0.72F, 0.88F}, {0.85F, 0.93F, 0.98F}, {0.78F, 0.88F, 0.95F}, {0.90F, 0.95F, 0.99F}, {0.40F, 0.50F, 0.62F},
       {1.00F, 1.00F, 1.00F}, {1.00F, 1.00F, 1.00F}}},
     0.35F, 0.25F, {0.95F, 0.98F, 1.00F}, {0.55F, 0.78F, 1.00F}, 0.55F, 0.5F, 0.0F, 0.0F},
    {"Lava", 0.30F,
     {{{1.00F, 0.55F, 0.08F}, {0.85F, 0.25F, 0.03F}, {0.12F, 0.07F, 0.06F}, {0.17F, 0.11F, 0.09F}, {0.24F, 0.16F, 0.12F}, {0.30F, 0.22F, 0.19F},
       {0.52F, 0.48F, 0.46F}, {0.50F, 0.46F, 0.45F}}},
     1.0F, 0.35F, {0.30F, 0.24F, 0.22F}, {1.00F, 0.45F, 0.18F}, 0.60F, 0.0F, 1.0F, 0.0F},
}};

// ---------------------------------------------------------------------------------------------- rays

struct Hit {
    bool hit = false;
    float t = 0.0F;     // nearest intersection in front of the origin (the exit point if the origin is inside)
    float tFar = 0.0F;  // the other one
};

/// Ray against a sphere at the origin. `rd` must be a unit vector.
inline Hit raySphere(Vec3 ro, Vec3 rd, float radius) {
    const float b = dot(ro, rd);
    const float c = dot(ro, ro) - radius * radius;
    const float h = b * b - c;
    if (h < 0.0F) return {};
    const float s = std::sqrt(h);
    const float t0 = -b - s, t1 = -b + s;
    if (t1 < 0.0F) return {};
    return t0 >= 0.0F ? Hit{true, t0, t1} : Hit{true, t1, t1};
}

struct Closest {
    float distance;  // from the origin to the ray's line
    float t;         // where on the line (negative: behind the start)
};

inline Closest closestApproach(Vec3 ro, Vec3 rd) {
    const float b = dot(ro, rd);
    return {std::sqrt(std::max(dot(ro, ro) - b * b, 0.0F)), -b};
}

/// How much of a pixel the planet covers, from how far the ray passes the surface: 0.5 on the silhouette, 1 half a pixel
/// inside, 0 half a pixel outside. `pixelWorld` is the size of a pixel at the planet. This is the whole anti-aliasing.
inline float edgeCoverage(float rmin, float pixelWorld) {
    if (!(pixelWorld > 0.0F)) return rmin < 1.0F ? 1.0F : 0.0F;
    return clamp01((1.0F - rmin) / pixelWorld + 0.5F);
}

// ---------------------------------------------------------------------------------------------- camera

struct Camera {
    Vec3 pos, fwd, right, up;
};

inline float clampPitch(float pitch) { return std::clamp(pitch, -kMaxPitch, kMaxPitch); }
inline float clampDistance(float d) { return std::clamp(d, kMinDistance, kMaxDistance); }

/// Camera on a sphere around the planet, looking at its centre.
inline Camera makeCamera(float yaw, float pitch, float distance) {
    pitch = clampPitch(pitch);
    distance = clampDistance(distance);
    const Vec3 dir{std::sin(yaw) * std::cos(pitch), std::sin(pitch), std::cos(yaw) * std::cos(pitch)};
    Camera c;
    c.pos = dir * distance;
    c.fwd = dir * -1.0F;
    c.right = normalize(cross(c.fwd, Vec3{0.0F, 1.0F, 0.0F}));
    c.up = cross(c.right, c.fwd);
    return c;
}

/// Direction of the ray through a canvas pixel (960x540, y down). The view axis passes through (480, kCenterY).
inline Vec3 pixelRay(const Camera& c, float px, float py) {
    const float ndcX = px / (kCanvasW * 0.5F) - 1.0F;
    const float ndcY = (kCenterY - py) / (kCanvasH * 0.5F);
    return normalize(c.fwd + c.right * (ndcX * kTanHalfFov * kAspect) + c.up * (ndcY * kTanHalfFov));
}

struct Screen {
    float x, y;
    bool inFront;
};

/// Canvas pixel of a world point, the inverse of pixelRay.
inline Screen project(const Camera& c, Vec3 world) {
    const Vec3 v = world - c.pos;
    const float z = dot(v, c.fwd);
    if (z <= 1e-6F) return {0.0F, 0.0F, false};
    const float ndcX = dot(v, c.right) / (z * kTanHalfFov * kAspect);
    const float ndcY = dot(v, c.up) / (z * kTanHalfFov);
    return {(ndcX + 1.0F) * (kCanvasW * 0.5F), kCenterY - ndcY * (kCanvasH * 0.5F), true};
}

/// Is this point of the planet surface (world frame, unit vector) on the side that faces the camera?
inline bool facesCamera(const Camera& c, Vec3 surfacePoint) { return dot(surfacePoint, c.pos - surfacePoint) > 0.0F; }

// ---------------------------------------------------------------------------------------------- planet frame and time

/// Planet frame to world: spin about the planet's own y axis first, then lean the whole thing by the tilt about z.
inline Vec3 toWorld(Vec3 local, float spin, float tilt) {
    const float cs = std::cos(spin), ss = std::sin(spin);
    const Vec3 s{local.x * cs + local.z * ss, local.y, -local.x * ss + local.z * cs};
    const float ct = std::cos(tilt), st = std::sin(tilt);
    return {s.x * ct - s.y * st, s.x * st + s.y * ct, s.z};
}

/// World to planet frame, the inverse of toWorld.
inline Vec3 toLocal(Vec3 world, float spin, float tilt) {
    const float ct = std::cos(tilt), st = std::sin(tilt);
    const Vec3 t{world.x * ct + world.y * st, -world.x * st + world.y * ct, world.z};
    const float cs = std::cos(spin), ss = std::sin(spin);
    return {t.x * cs - t.z * ss, t.y, t.x * ss + t.z * cs};
}

/// Angle after dt seconds at `rate` rad/s, kept in [0, 2 pi) in double. Float cannot hold a growing angle: after an hour of
/// running the fraction has only a few digits left and the rotation stutters. The shader only ever gets the reduced angle.
inline double advanceAngle(double angle, double rate, double dt) {
    double a = std::fmod(angle + rate * dt, kTwoPi);
    if (a < 0.0) a += kTwoPi;
    return a;
}

/// Spin that turns the seam of the naive mapping (longitude pi) towards a camera at the given yaw.
inline float seamFacingSpin(float yaw, float tilt) { return std::atan2(std::cos(tilt) * std::cos(yaw), -std::sin(yaw)); }

// ---------------------------------------------------------------------------------------------- light and atmosphere

/// How much daylight a surface gets from the cosine between its normal and the sun. Wider than a hard max(0, n.l): the
/// terminator is a soft band, like the real one where the atmosphere scatters light around the corner.
inline float dayFactor(float ndl) { return smoothstep(-0.1F, 0.25F, ndl); }

/// Glow of the atmosphere for a ray that misses the planet, from the distance of the ray to the centre: 1 on the limb,
/// 0 at the top of the shell, falling with the cube.
inline float glowOutside(float rmin) {
    const float a = clamp01((rmin - 1.0F) / (kAtmoRadius - 1.0F));
    const float k = 1.0F - a;
    return k * k * k;
}

/// Glow on the disc from the cosine between the view ray and the surface normal: 1 at the rim, 0 looking straight down.
inline float glowInside(float cosView) {
    const float k = 1.0F - clamp01(cosView);
    return k * k * k;
}

/// The same for a ray that hits the planet, given the ray's distance to the centre (cos = sqrt(1 - rmin^2)).
inline float glowOnDisc(float rmin) { return glowInside(std::sqrt(std::max(0.0F, 1.0F - rmin * rmin))); }

}  // namespace gd::planet
