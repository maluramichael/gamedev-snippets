// The math of a small ray tracer. No SDL, so test.cpp can check it with hand-computed numbers.
//
// One primary ray per pixel goes out of the camera. The nearest thing it hits (three spheres, one floor plane) gets a
// color: diffuse light from a point light, optionally a shadow ray towards the light, optionally a reflected ray that
// is traced the same way. Every ray that is shot is counted, that count is what the demo shows as rays per second.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace rt {

struct Vec3 {
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
};

inline Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator*(Vec3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
inline Vec3 operator*(Vec3 a, Vec3 b) { return {a.x * b.x, a.y * b.y, a.z * b.z}; }
inline float dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline float length(Vec3 a) { return std::sqrt(dot(a, a)); }

inline Vec3 normalize(Vec3 a) {
    const float len = length(a);
    return len > 0.0F ? a * (1.0F / len) : Vec3{};
}

inline Vec3 lerp(Vec3 a, Vec3 b, float t) { return a + (b - a) * t; }

/// Mirror direction `d` at a surface with the unit normal `n`: d - 2 (d.n) n.
inline Vec3 reflect(Vec3 d, Vec3 n) { return d - n * (2.0F * dot(d, n)); }

struct Ray {
    Vec3 origin;
    Vec3 dir;  // unit length for rays that come from the camera, reflect() keeps it that way
};

struct Sphere {
    Vec3 center;
    float radius;
    Vec3 color;
    float reflectivity;  // 0 = matt, 1 = perfect mirror
};

inline constexpr float kInf = 1e30F;

/// Nearest hit with t in (tMin, tMax). Solves |o + t d - c|^2 = r^2, a quadratic in t. From inside the sphere the
/// near root is negative and the far root is the way out.
inline bool hitSphere(const Ray& ray, const Sphere& s, float tMin, float tMax, float& t) {
    const Vec3 oc = ray.origin - s.center;
    const float a = dot(ray.dir, ray.dir);
    const float halfB = dot(oc, ray.dir);
    const float c = dot(oc, oc) - s.radius * s.radius;
    const float disc = halfB * halfB - a * c;
    if (disc < 0.0F) return false;  // the line misses the sphere
    const float root = std::sqrt(disc);
    float hit = (-halfB - root) / a;
    if (hit <= tMin || hit >= tMax) {
        hit = (-halfB + root) / a;
        if (hit <= tMin || hit >= tMax) return false;
    }
    t = hit;
    return true;
}

/// Horizontal plane y = planeY. A ray parallel to it (dir.y == 0) never hits.
inline bool hitPlane(const Ray& ray, float planeY, float tMin, float tMax, float& t) {
    if (std::fabs(ray.dir.y) < 1e-8F) return false;
    const float hit = (planeY - ray.origin.y) / ray.dir.y;
    if (hit <= tMin || hit >= tMax) return false;
    t = hit;
    return true;
}

inline constexpr float kTileSize = 1.5F;

/// 0 or 1: which of the two tile colors lies at (x, z). floor() keeps the pattern going across zero.
inline int checkerIndex(float x, float z) {
    const int sum = static_cast<int>(std::floor(x / kTileSize)) + static_cast<int>(std::floor(z / kTileSize));
    return sum & 1;  // two's complement: -1 & 1 == 1, so negative tiles alternate as well
}

struct Camera {
    Vec3 pos{0.0F, 2.2F, -6.0F};
    float tanHalfFov = 0.38F;  // about 41.6 degrees vertical field of view
    float shift = 0.137F;      // moves the horizon up in the image without tilting the camera (horizon at 32% from the top)
    float aspect = 16.0F / 9.0F; // only used by project(), primaryRay() takes it from the image size
};

/// Ray through the continuous image position (u, v), both 0..1 with (0, 0) top left.
inline Ray rayThrough(const Camera& cam, float u, float v, float aspect) {
    const float sx = (2.0F * u - 1.0F) * aspect * cam.tanHalfFov;
    const float sy = (1.0F - 2.0F * v) * cam.tanHalfFov - cam.shift;
    return {cam.pos, normalize(Vec3{sx, sy, 1.0F})};
}

/// Primary ray through the center of pixel (px, py) of a w x h image.
inline Ray primaryRay(const Camera& cam, int px, int py, int w, int h) {
    return rayThrough(cam, (static_cast<float>(px) + 0.5F) / static_cast<float>(w), (static_cast<float>(py) + 0.5F) / static_cast<float>(h),
                      static_cast<float>(w) / static_cast<float>(h));
}

/// Where a world point lands in the image (u, v in 0..1). False for points behind the camera.
inline bool project(const Camera& cam, Vec3 point, float& u, float& v) {
    const Vec3 rel = point - cam.pos;
    if (rel.z <= 1e-4F) return false;
    const float sx = rel.x / rel.z;
    const float sy = rel.y / rel.z + cam.shift;
    u = (sx / (cam.aspect * cam.tanHalfFov) + 1.0F) * 0.5F;
    v = (1.0F - sy / cam.tanHalfFov) * 0.5F;
    return true;
}

inline constexpr Vec3 kSkyHorizon{0.86F, 0.97F, 0.95F};
inline constexpr Vec3 kSkyZenith{0.18F, 0.62F, 0.68F};
inline constexpr Vec3 kTileA{0.93F, 0.93F, 0.90F};
inline constexpr Vec3 kTileB{0.30F, 0.40F, 0.44F};

/// Gradient from the horizon color up to the zenith color; looking below the horizon keeps the horizon color.
inline Vec3 skyColor(Vec3 dir) { return lerp(kSkyHorizon, kSkyZenith, std::clamp(dir.y, 0.0F, 1.0F)); }

inline constexpr float kAmbient = 0.2F;
inline constexpr float kMinT = 1e-4F;        // rays ignore hits closer than this
inline constexpr float kShadowBias = 1e-3F;  // see occluded()
inline constexpr float kFogStart = 12.0F;    // the floor fades into the horizon color between these distances
inline constexpr float kFogEnd = 60.0F;

/// Does anything stand between `point` and the light? The shadow ray starts at `point` lifted by `bias` along the
/// normal. Without that lift, rounding errors leave the start a hair below the surface it was computed on, the ray
/// "hits" that surface right at t = 0 and the point shadows itself: the surface is covered in speckles ("shadow acne").
inline bool occluded(Vec3 point, Vec3 normal, Vec3 light, const Sphere* spheres, int count, float bias) {
    const Vec3 origin = point + normal * bias;
    const Vec3 toLight = light - origin;
    const float dist = length(toLight);
    const Ray shadowRay{origin, toLight * (1.0F / dist)};
    float t = 0.0F;
    for (int i = 0; i < count; ++i) {
        if (hitSphere(shadowRay, spheres[i], 0.0F, dist, t)) return true;  // only hits before the light count
    }
    return false;
}

struct Scene {
    std::vector<Sphere> spheres;
    Vec3 light{-3.0F, 3.0F, -1.0F};
    bool floor = true;
    float floorY = 0.0F;
    float floorReflectivity = 0.0F;  // a polished floor mirrors a bit of what stands on it
};

struct Options {
    bool shadows = true;
    bool reflections = true;
    int maxDepth = 3;  // reflection bounces; 0 means "no reflection ray at all"
};

/// The demo scene: a matt teal ball, a matt orange ball and a mirror ball on a slightly reflective checkerboard.
inline Scene demoScene() {
    Scene s;
    s.floorReflectivity = 0.2F;
    s.spheres.push_back({{0.0F, 1.2F, 2.6F}, 1.2F, {0.08F, 0.09F, 0.11F}, 0.85F});   // mirror
    s.spheres.push_back({{-2.5F, 0.85F, 2.0F}, 0.85F, {0.05F, 0.58F, 0.53F}, 0.0F}); // teal
    s.spheres.push_back({{2.4F, 0.75F, 1.6F}, 0.75F, {0.92F, 0.35F, 0.05F}, 0.0F});  // orange
    return s;
}

/// Color seen along `ray`. Counts every ray it shoots (this one, shadow rays, reflection rays) in `rays`.
inline Vec3 trace(const Scene& scene, const Ray& ray, int depth, const Options& opt, std::uint64_t& rays) {
    ++rays;
    float best = kInf;
    int sphereIndex = -1;
    bool floorHit = false;
    float t = 0.0F;
    for (int i = 0; i < static_cast<int>(scene.spheres.size()); ++i) {
        if (hitSphere(ray, scene.spheres[static_cast<std::size_t>(i)], kMinT, best, t)) {
            best = t;
            sphereIndex = i;
        }
    }
    if (scene.floor && hitPlane(ray, scene.floorY, kMinT, best, t)) {
        best = t;
        floorHit = true;
        sphereIndex = -1;
    }
    if (sphereIndex < 0 && !floorHit) return skyColor(ray.dir);

    const Vec3 point = ray.origin + ray.dir * best;
    Vec3 normal{0.0F, 1.0F, 0.0F};
    Vec3 albedo;
    float reflectivity = 0.0F;
    if (floorHit) {
        albedo = checkerIndex(point.x, point.z) == 0 ? kTileA : kTileB;
        reflectivity = scene.floorReflectivity;
    } else {
        const Sphere& s = scene.spheres[static_cast<std::size_t>(sphereIndex)];
        normal = (point - s.center) * (1.0F / s.radius);
        albedo = s.color;
        reflectivity = s.reflectivity;
    }

    // Diffuse: the more the surface faces the light, the brighter. Facing away, only the ambient term is left, and
    // there is no point in a shadow ray then.
    const Vec3 toLight = normalize(scene.light - point);
    const float facing = std::max(0.0F, dot(normal, toLight));
    float lit = 1.0F;
    if (opt.shadows && facing > 0.0F) {
        ++rays;
        if (occluded(point, normal, scene.light, scene.spheres.data(), static_cast<int>(scene.spheres.size()), kShadowBias)) lit = 0.0F;
    }
    Vec3 color = albedo * (kAmbient + (1.0F - kAmbient) * facing * lit);

    if (opt.reflections && reflectivity > 0.0F && depth < opt.maxDepth) {
        const Ray bounce{point + normal * kShadowBias, normalize(reflect(ray.dir, normal))};
        color = lerp(color, trace(scene, bounce, depth + 1, opt, rays), reflectivity);
    }
    if (floorHit) color = lerp(color, kSkyHorizon, std::clamp((best - kFogStart) / (kFogEnd - kFogStart), 0.0F, 1.0F));
    return color;
}

/// 0xAARRGGBB, opaque, each channel rounded from 0..1.
inline std::uint32_t pack(Vec3 c) {
    const auto byte = [](float v) { return static_cast<std::uint32_t>(std::lround(std::clamp(v, 0.0F, 1.0F) * 255.0F)); };
    return 0xFF000000U | (byte(c.x) << 16) | (byte(c.y) << 8) | byte(c.z);
}

/// Renders rows [y0, y1) of a w x h image into `pixels` (row-major, w per row).
inline void renderRows(const Scene& scene, const Camera& cam, const Options& opt, int w, int h, int y0, int y1, std::uint32_t* pixels,
                       std::uint64_t& rays) {
    for (int y = y0; y < y1; ++y) {
        for (int x = 0; x < w; ++x) {
            pixels[y * w + x] = pack(trace(scene, primaryRay(cam, x, y, w, h), 0, opt, rays));
        }
    }
}

inline constexpr float kLightHeight = 3.0F;
inline constexpr float kLightXMax = 7.0F;
inline constexpr float kLightZMin = -3.0F;
inline constexpr float kLightZMax = 12.0F;

/// Light position for a mouse at image position (u, v): it hangs kLightHeight above the floor point under the cursor.
/// Above the horizon there is no floor under the cursor, then the light goes to the far end.
inline Vec3 lightFromMouse(const Camera& cam, float u, float v, float aspect) {
    const Ray ray = rayThrough(cam, u, v, aspect);
    float t = 0.0F;
    float x = 0.0F;
    float z = kLightZMax;
    if (hitPlane(ray, 0.0F, 0.0F, kInf, t)) {
        const Vec3 p = ray.origin + ray.dir * t;
        x = p.x;
        z = p.z;
    }
    return {std::clamp(x, -kLightXMax, kLightXMax), kLightHeight, std::clamp(z, kLightZMin, kLightZMax)};
}

}  // namespace rt
