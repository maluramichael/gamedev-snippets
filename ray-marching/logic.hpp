// The CPU mirror of the ray marcher's shader math, no SDL and no GL so it can be tested natively.
//
// Sphere tracing walks a ray through a signed distance field (SDF): at the current point the SDF says how far the nearest
// surface is, so the ray can safely jump exactly that far without hitting anything, and repeats until the distance is
// below a threshold (hit), the ray is too far away (miss) or the step budget is spent (exhausted). Everything else comes
// from the same SDF: the normal is its gradient, shadows and ambient occlusion are a few extra lookups along a ray.
//
// main.cpp's fragment shader repeats these functions in GLSL with the same constants. The scene() function and the
// numbers in it have to stay in sync with the shader's map().
#pragma once

#include <algorithm>
#include <cmath>

namespace gd::march {

struct Vec3 {
    float x = 0.0F, y = 0.0F, z = 0.0F;
};

inline Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator*(Vec3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
inline float dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline float length(Vec3 a) { return std::sqrt(dot(a, a)); }
inline Vec3 normalize(Vec3 a) { return a * (1.0F / length(a)); }

// ---- distance functions: the exact distance to the shape, negative inside ----

inline float sdSphere(Vec3 p, float radius) { return length(p) - radius; }

/// Box centered at the origin with half extents `b`. Outside it is the length of the vector to the nearest point on the box
/// (face, edge or corner), inside it is the depth to the nearest face.
inline float sdBox(Vec3 p, Vec3 b) {
    const Vec3 q{std::fabs(p.x) - b.x, std::fabs(p.y) - b.y, std::fabs(p.z) - b.z};
    const Vec3 outside{std::fmax(q.x, 0.0F), std::fmax(q.y, 0.0F), std::fmax(q.z, 0.0F)};
    return length(outside) + std::fmin(std::fmax(q.x, std::fmax(q.y, q.z)), 0.0F);
}

/// A box with rounded edges: shrink the box by r, then inflate the distance by r.
inline float sdRoundBox(Vec3 p, Vec3 b, float r) { return sdBox(p, b) - r; }

/// Torus around the y axis: ring radius `ring` in the xz plane, tube radius `tube`.
inline float sdTorus(Vec3 p, float ring, float tube) {
    const float qx = std::sqrt(p.x * p.x + p.z * p.z) - ring;
    return std::sqrt(qx * qx + p.y * p.y) - tube;
}

/// The floor y = 0.
inline float sdPlane(Vec3 p) { return p.y; }

/// Polynomial smooth minimum. Equals min(a, b) when the values are further apart than k, dips below it by up to k/4 when
/// they are close. k <= 0 is the plain minimum.
inline float smin(float a, float b, float k) {
    if (k <= 0.0F) return std::fmin(a, b);
    const float h = std::fmax(k - std::fabs(a - b), 0.0F) / k;
    return std::fmin(a, b) - h * h * k * 0.25F;
}

inline Vec3 rotY(Vec3 p, float a) {
    const float c = std::cos(a), s = std::sin(a);
    return {c * p.x + s * p.z, p.y, -s * p.x + c * p.z};
}

inline Vec3 rotX(Vec3 p, float a) {
    const float c = std::cos(a), s = std::sin(a);
    return {p.x, c * p.y - s * p.z, s * p.y + c * p.z};
}

// ---- the scene ----

struct Shapes {
    float sphere, box, torus;
};

/// Distances to the three moving shapes at time `t` (seconds). They drift into each other and apart again.
inline Shapes shapes(Vec3 p, float t) {
    const Vec3 sphereAt{-0.9F + 0.9F * std::sin(0.8F * t), 0.8F + 0.35F * std::sin(1.3F * t + 1.0F), 0.2F * std::sin(0.6F * t)};
    const Vec3 boxAt{0.5F + 0.9F * std::cos(0.7F * t), 0.75F + 0.3F * std::sin(1.1F * t), 0.6F * std::cos(0.5F * t)};
    const Vec3 torusAt{0.2F * std::sin(0.5F * t), 1.2F + 0.25F * std::sin(0.9F * t + 2.0F), -0.3F + 0.5F * std::sin(0.4F * t)};
    return {sdSphere(p - sphereAt, 0.65F), sdRoundBox(rotY(p - boxAt, 0.6F * t), {0.4F, 0.3F, 0.4F}, 0.1F),
            sdTorus(rotX(p - torusAt, 0.6F * t), 0.75F, 0.2F)};
}

/// The whole scene: the floor, and the three shapes merged with a smooth union of strength `k`.
inline float scene(Vec3 p, float t, float k) {
    const Shapes s = shapes(p, t);
    return std::fmin(sdPlane(p), smin(smin(s.sphere, s.box, k), s.torus, k));
}

// ---- sphere tracing ----

struct Hit {
    bool hit = false;        // the distance dropped below the threshold
    bool exhausted = false;  // no hit, not out of range either: ran out of steps (artifacts, dark in the heatmap)
    float t = 0.0F;          // distance along the ray where it ended
    int steps = 0;           // SDF evaluations
};

/// Marches from `ro` along the unit vector `rd`. A hit is d < eps + epsSlope * t: with epsSlope > 0 the threshold grows with
/// the distance, which is roughly the size of a pixel there.
template <class Sdf>
Hit march(Vec3 ro, Vec3 rd, const Sdf& sdf, int maxSteps, float eps = 0.001F, float tmax = 30.0F, float epsSlope = 0.0F) {
    Hit h;
    float t = 0.0F;
    for (int i = 0; i < maxSteps; ++i) {
        const float d = sdf(ro + rd * t);
        h.steps = i + 1;
        if (d < eps + epsSlope * t) {
            h.hit = true;
            h.t = t;
            return h;
        }
        t += d;
        if (t > tmax) {
            h.t = t;
            return h;
        }
    }
    h.t = t;
    h.exhausted = true;
    return h;
}

/// Surface normal as the gradient of the SDF, four taps on the corners of a tetrahedron instead of six along the axes.
template <class Sdf>
Vec3 normalAt(Vec3 p, const Sdf& sdf, float eps = 0.001F) {
    constexpr float kScale = 0.5773F;
    const Vec3 a{1, -1, -1}, b{-1, -1, 1}, c{-1, 1, -1}, d{1, 1, 1};
    const float h = kScale * eps;
    const Vec3 n = a * sdf(p + a * h) + b * sdf(p + b * h) + c * sdf(p + c * h) + d * sdf(p + d * h);
    return normalize(n);
}

/// Soft shadow towards the light: march from the surface to the light and remember the smallest ratio of distance to
/// the field to distance travelled. A ray that skims an object gets a value between 0 (blocked) and 1 (free).
template <class Sdf>
float softShadow(Vec3 ro, Vec3 rd, const Sdf& sdf, float hardness = 8.0F, int maxSteps = 40, float tmax = 12.0F) {
    float res = 1.0F;
    float t = 0.02F;
    for (int i = 0; i < maxSteps; ++i) {
        const float d = sdf(ro + rd * t);
        res = std::fmin(res, hardness * d / t);
        t += std::clamp(d, 0.02F, 0.5F);
        if (res < 0.001F || t > tmax) break;
    }
    res = std::clamp(res, 0.0F, 1.0F);
    return res * res * (3.0F - 2.0F * res);
}

/// Ambient occlusion: five taps along the normal. In the open each tap is exactly as far from the surface as from the
/// origin point (d == h); the shortfall (h - d) is how much geometry crowds in.
template <class Sdf>
float ambientOcclusion(Vec3 p, Vec3 n, const Sdf& sdf) {
    float occ = 0.0F;
    float weight = 1.0F;
    for (int i = 0; i < 5; ++i) {
        const float h = 0.01F + 0.12F * static_cast<float>(i) / 4.0F;
        occ += (h - sdf(p + n * h)) * weight;
        weight *= 0.95F;
    }
    return std::clamp(1.0F - 3.0F * occ, 0.0F, 1.0F);
}

// ---- heatmap ----

inline constexpr float kHeatSteps = 48.0F;  // step count where the heatmap reaches its last color

/// Step count to color: paper at 0, teal at half, orange at kHeatSteps and above.
inline Vec3 heat(float steps) {
    const float x = std::clamp(steps / kHeatSteps, 0.0F, 1.0F) * 2.0F;
    const Vec3 paper{0.98F, 0.98F, 0.97F}, teal{0.051F, 0.580F, 0.533F}, orange{0.918F, 0.345F, 0.047F};
    return x < 1.0F ? paper + (teal - paper) * x : teal + (orange - teal) * (x - 1.0F);
}

}  // namespace gd::march
