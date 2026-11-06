// The math behind picking, without SDL so test.cpp can check it: 4x4 matrices, pixel -> world ray, ray tests
// (sphere, box with the slab method, plane), nearest hit, orbit camera and the two little meshes the demo draws.
//
// Conventions: right-handed, y up, column vectors (v' = M * v), matrices stored row-major as m[row][col].
// Clip space is OpenGL style: z in [-w, w] and the near plane is z + w = 0. Pixels have y pointing down.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

namespace pk {

inline constexpr float kFovY = 0.8F;  // vertical field of view in radians (about 46 degrees)
inline constexpr float kNear = 0.1F;
inline constexpr float kFar = 100.0F;
inline constexpr float kMinPitch = 0.09F;
inline constexpr float kMaxPitch = 1.48F;
inline constexpr float kMinDistance = 6.0F;
inline constexpr float kMaxDistance = 14.0F;

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

struct Vec4 {
    float x = 0.0F, y = 0.0F, z = 0.0F, w = 0.0F;
};

// ---------------------------------------------------------------------------------------------- matrices

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
    return {a.m[0][0] * v.x + a.m[0][1] * v.y + a.m[0][2] * v.z + a.m[0][3] * v.w,
            a.m[1][0] * v.x + a.m[1][1] * v.y + a.m[1][2] * v.z + a.m[1][3] * v.w,
            a.m[2][0] * v.x + a.m[2][1] * v.y + a.m[2][2] * v.z + a.m[2][3] * v.w,
            a.m[3][0] * v.x + a.m[3][1] * v.y + a.m[3][2] * v.z + a.m[3][3] * v.w};
}

/// Maps the view frustum to the clip cube. Camera looks down -z, depth ends up in [-1, 1] after the divide by w.
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

/// World -> view: puts the eye at the origin looking down -z.
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

/// General inverse (Gauss-Jordan with partial pivoting, in double). A singular matrix gives the identity.
inline Mat4 inverse(const Mat4& in) {
    double a[4][8];
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            a[i][j] = static_cast<double>(in.m[i][j]);
            a[i][j + 4] = i == j ? 1.0 : 0.0;
        }
    }
    for (int col = 0; col < 4; ++col) {
        int pivot = col;
        for (int row = col + 1; row < 4; ++row) {
            if (std::fabs(a[row][col]) > std::fabs(a[pivot][col])) pivot = row;
        }
        if (std::fabs(a[pivot][col]) < 1e-12) return Mat4{};
        for (int j = 0; j < 8; ++j) std::swap(a[col][j], a[pivot][j]);
        const double d = a[col][col];
        for (int j = 0; j < 8; ++j) a[col][j] /= d;
        for (int row = 0; row < 4; ++row) {
            if (row == col) continue;
            const double factor = a[row][col];
            for (int j = 0; j < 8; ++j) a[row][j] -= factor * a[col][j];
        }
    }
    Mat4 r;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) r.m[i][j] = static_cast<float>(a[i][j + 4]);
    }
    return r;
}

// ---------------------------------------------------------------------------------------------- pixels <-> world

struct Viewport {
    float x, y, w, h;
};

/// World point -> (pixel x, pixel y, NDC depth). Nothing if the point is behind the camera.
inline std::optional<Vec3> project(const Mat4& viewProj, const Viewport& vp, Vec3 world) {
    const Vec4 c = viewProj * Vec4{world.x, world.y, world.z, 1.0F};
    if (c.w <= 0.0F) return std::nullopt;
    const float nx = c.x / c.w;
    const float ny = c.y / c.w;
    return Vec3{vp.x + (nx * 0.5F + 0.5F) * vp.w, vp.y + (1.0F - (ny * 0.5F + 0.5F)) * vp.h, c.z / c.w};
}

/// Pixel plus an NDC depth (-1 near plane, +1 far plane) -> world point. The inverse of project().
inline Vec3 unproject(const Mat4& invViewProj, const Viewport& vp, float px, float py, float ndcZ) {
    const float nx = (px - vp.x) / vp.w * 2.0F - 1.0F;
    const float ny = 1.0F - (py - vp.y) / vp.h * 2.0F;
    const Vec4 c = invViewProj * Vec4{nx, ny, ndcZ, 1.0F};
    return {c.x / c.w, c.y / c.w, c.z / c.w};
}

struct Ray {
    Vec3 origin;
    Vec3 dir;  // unit length
};

/// The ray through a pixel: unproject it once on the near and once on the far plane, the direction is the
/// difference. Both points lie on the ray from the eye, so the eye is the origin and t is a real distance.
inline Ray pixelToRay(const Mat4& invViewProj, const Viewport& vp, float px, float py, Vec3 eye) {
    const Vec3 nearPoint = unproject(invViewProj, vp, px, py, -1.0F);
    const Vec3 farPoint = unproject(invViewProj, vp, px, py, 1.0F);
    return Ray{eye, normalize(farPoint - nearPoint)};
}

// ---------------------------------------------------------------------------------------------- ray tests
// All of them return the smallest t >= 0 with origin + t * dir on the shape, or nothing. A ray that starts inside
// a solid reports t = 0. The direction has to be a unit vector.

inline std::optional<float> raySphere(const Ray& ray, Vec3 center, float radius) {
    const Vec3 oc = ray.origin - center;
    const float a = dot(ray.dir, ray.dir);
    const float halfB = dot(oc, ray.dir);
    const float c = dot(oc, oc) - radius * radius;
    const float disc = halfB * halfB - a * c;
    if (disc < 0.0F) return std::nullopt;
    const float root = std::sqrt(disc);
    const float far = (-halfB + root) / a;
    if (far < 0.0F) return std::nullopt;  // the sphere is behind the origin
    return std::max((-halfB - root) / a, 0.0F);
}

/// Slab method: a box is the overlap of three slabs (one per axis). The ray is inside a slab for an interval of t,
/// it hits the box where all three intervals overlap. An axis the ray runs parallel to has no interval: the origin
/// is either inside that slab (ignore the axis) or outside (miss).
inline std::optional<float> rayAabb(const Ray& ray, Vec3 lo, Vec3 hi) {
    const float o[3] = {ray.origin.x, ray.origin.y, ray.origin.z};
    const float d[3] = {ray.dir.x, ray.dir.y, ray.dir.z};
    const float mn[3] = {lo.x, lo.y, lo.z};
    const float mx[3] = {hi.x, hi.y, hi.z};
    float tMin = 0.0F;  // starting at 0 also covers "origin inside" and cuts off everything behind the origin
    float tMax = INFINITY;
    for (int i = 0; i < 3; ++i) {
        if (std::fabs(d[i]) < 1e-8F) {
            if (o[i] < mn[i] || o[i] > mx[i]) return std::nullopt;
            continue;
        }
        float t1 = (mn[i] - o[i]) / d[i];
        float t2 = (mx[i] - o[i]) / d[i];
        if (t1 > t2) std::swap(t1, t2);
        tMin = std::max(tMin, t1);
        tMax = std::min(tMax, t2);
        if (tMin > tMax) return std::nullopt;
    }
    return tMin;
}

inline std::optional<float> rayPlane(const Ray& ray, Vec3 point, Vec3 normal) {
    const float denom = dot(normal, ray.dir);
    if (std::fabs(denom) < 1e-8F) return std::nullopt;
    const float t = dot(normal, point - ray.origin) / denom;
    if (t < 0.0F) return std::nullopt;
    return t;
}

// ---------------------------------------------------------------------------------------------- scene and picking

struct Object {
    enum class Kind { Sphere, Box };
    Kind kind;
    Vec3 center;
    Vec3 half;  // box: half extents; sphere: half.x is the radius
    const char* name;

    float radius() const { return half.x; }
};

inline Vec3 boundsMin(const Object& o) {
    return o.kind == Object::Kind::Sphere ? o.center - Vec3{o.half.x, o.half.x, o.half.x} : o.center - o.half;
}
inline Vec3 boundsMax(const Object& o) {
    return o.kind == Object::Kind::Sphere ? o.center + Vec3{o.half.x, o.half.x, o.half.x} : o.center + o.half;
}

inline std::optional<float> intersect(const Ray& ray, const Object& o) {
    return o.kind == Object::Kind::Sphere ? raySphere(ray, o.center, o.radius()) : rayAabb(ray, boundsMin(o), boundsMax(o));
}

struct Pick {
    int object = -1;     // index into the scene, -1 for the floor
    bool floor = false;  // the nearest thing is the floor plane (y = 0)
    float t = 0.0F;
};

/// Tests every object and the floor plane and keeps the nearest hit. Objects win a tie against the floor.
inline std::optional<Pick> pick(const std::vector<Object>& objects, const Ray& ray) {
    std::optional<Pick> best;
    for (std::size_t i = 0; i < objects.size(); ++i) {
        const auto t = intersect(ray, objects[i]);
        if (t && (!best || *t < best->t)) best = Pick{static_cast<int>(i), false, *t};
    }
    const auto floorT = rayPlane(ray, {0, 0, 0}, {0, 1, 0});
    if (floorT && (!best || *floorT < best->t)) best = Pick{-1, true, *floorT};
    return best;
}

/// Do two objects penetrate each other? Touching (a ball resting on a box) does not count.
inline bool overlap(const Object& a, const Object& b) {
    constexpr float eps = 1e-4F;
    const auto closestOnBox = [](const Object& box, Vec3 p) {
        const Vec3 lo = boundsMin(box);
        const Vec3 hi = boundsMax(box);
        return Vec3{std::clamp(p.x, lo.x, hi.x), std::clamp(p.y, lo.y, hi.y), std::clamp(p.z, lo.z, hi.z)};
    };
    if (a.kind == Object::Kind::Sphere && b.kind == Object::Kind::Sphere) {
        return length(a.center - b.center) < a.radius() + b.radius() - eps;
    }
    if (a.kind == Object::Kind::Sphere) return length(a.center - closestOnBox(b, a.center)) < a.radius() - eps;
    if (b.kind == Object::Kind::Sphere) return length(b.center - closestOnBox(a, b.center)) < b.radius() - eps;
    const Vec3 lo = boundsMin(a), hi = boundsMax(a), lo2 = boundsMin(b), hi2 = boundsMax(b);
    return lo.x < hi2.x - eps && hi.x > lo2.x + eps && lo.y < hi2.y - eps && hi.y > lo2.y + eps && lo.z < hi2.z - eps && hi.z > lo2.z + eps;
}

/// Seven objects on the floor; the pearl sits on the tower, so a ray from the side can hit either of the two.
inline std::vector<Object> defaultScene() {
    using K = Object::Kind;
    return {
        {K::Box, {-3.2F, 0.8F, -1.2F}, {0.8F, 0.8F, 0.8F}, "Cube"},
        {K::Box, {2.6F, 1.3F, -2.4F}, {0.7F, 1.3F, 0.7F}, "Tower"},
        {K::Box, {0.6F, 0.4F, 2.6F}, {1.3F, 0.4F, 0.8F}, "Slab"},
        {K::Sphere, {-0.6F, 0.9F, -0.4F}, {0.9F, 0.0F, 0.0F}, "Ball"},
        {K::Sphere, {3.3F, 0.7F, 1.3F}, {0.7F, 0.0F, 0.0F}, "Orb"},
        {K::Sphere, {2.6F, 3.0F, -2.4F}, {0.4F, 0.0F, 0.0F}, "Pearl"},
        {K::Sphere, {-3.4F, 0.55F, 2.4F}, {0.55F, 0.0F, 0.0F}, "Marble"},
    };
}

// ---------------------------------------------------------------------------------------------- camera

struct OrbitCamera {
    float yaw = 0.62F;
    float pitch = 0.5F;
    float distance = 11.0F;
    Vec3 target{0.0F, 0.6F, 0.0F};

    Vec3 eye() const {
        return target + Vec3{distance * std::cos(pitch) * std::sin(yaw), distance * std::sin(pitch), distance * std::cos(pitch) * std::cos(yaw)};
    }
    Mat4 view() const { return lookAt(eye(), target, {0.0F, 1.0F, 0.0F}); }

    void orbit(float dYaw, float dPitch) {
        yaw += dYaw;
        pitch = std::clamp(pitch + dPitch, kMinPitch, kMaxPitch);
    }
    /// One wheel notch moves the camera 10 % closer (negative notches move away).
    void zoom(float notches) { distance = std::clamp(distance * std::pow(0.9F, notches), kMinDistance, kMaxDistance); }
};

// ---------------------------------------------------------------------------------------------- meshes

using Tri = std::array<int, 3>;

struct Mesh {
    std::vector<Vec3> v;
    std::vector<Tri> t;  // counter-clockwise seen from outside
};

/// Cube from -1 to 1 with 8 shared corners.
inline Mesh unitBox() {
    Mesh m;
    for (int i = 0; i < 8; ++i) m.v.push_back({(i & 1) ? 1.0F : -1.0F, (i & 2) ? 1.0F : -1.0F, (i & 4) ? 1.0F : -1.0F});
    const int quads[6][4] = {{1, 3, 7, 5}, {0, 4, 6, 2}, {2, 6, 7, 3}, {0, 1, 5, 4}, {4, 5, 7, 6}, {0, 2, 3, 1}};
    for (const auto& q : quads) {
        m.t.push_back({q[0], q[1], q[2]});
        m.t.push_back({q[0], q[2], q[3]});
    }
    return m;
}

/// Unit sphere from an icosahedron: every level splits each triangle into four and pushes the new corners onto the
/// sphere. Level 2 has 320 triangles.
inline Mesh icosphere(int level) {
    const float g = (1.0F + std::sqrt(5.0F)) / 2.0F;
    Mesh m;
    const Vec3 base[12] = {{-1, g, 0}, {1, g, 0}, {-1, -g, 0}, {1, -g, 0}, {0, -1, g}, {0, 1, g},
                           {0, -1, -g}, {0, 1, -g}, {g, 0, -1}, {g, 0, 1}, {-g, 0, -1}, {-g, 0, 1}};
    for (const Vec3 p : base) m.v.push_back(normalize(p));
    m.t = {{0, 11, 5}, {0, 5, 1}, {0, 1, 7}, {0, 7, 10}, {0, 10, 11}, {1, 5, 9}, {5, 11, 4}, {11, 10, 2}, {10, 7, 6}, {7, 1, 8},
           {3, 9, 4}, {3, 4, 2}, {3, 2, 6}, {3, 6, 8}, {3, 8, 9}, {4, 9, 5}, {2, 4, 11}, {6, 2, 10}, {8, 6, 7}, {9, 8, 1}};
    for (int l = 0; l < level; ++l) {
        std::vector<std::pair<std::uint64_t, int>> cache;  // edge key -> midpoint index
        const auto midpoint = [&](int a, int b) {
            const std::uint64_t key = (static_cast<std::uint64_t>(std::min(a, b)) << 32) | static_cast<std::uint64_t>(std::max(a, b));
            for (const auto& [k, index] : cache) {
                if (k == key) return index;
            }
            const Vec3 mid = normalize((m.v[static_cast<std::size_t>(a)] + m.v[static_cast<std::size_t>(b)]) * 0.5F);
            m.v.push_back(mid);
            cache.emplace_back(key, static_cast<int>(m.v.size()) - 1);
            return static_cast<int>(m.v.size()) - 1;
        };
        std::vector<Tri> next;
        for (const Tri& tri : m.t) {
            const int ab = midpoint(tri[0], tri[1]);
            const int bc = midpoint(tri[1], tri[2]);
            const int ca = midpoint(tri[2], tri[0]);
            next.push_back({tri[0], ab, ca});
            next.push_back({tri[1], bc, ab});
            next.push_back({tri[2], ca, bc});
            next.push_back({ab, bc, ca});
        }
        m.t = std::move(next);
    }
    return m;
}

/// Seen from the eye, is the triangle facing us? Uses the corners as they are in world space.
inline bool frontFacing(const Mesh& world, const Tri& tri, Vec3 eye) {
    const Vec3 a = world.v[static_cast<std::size_t>(tri[0])];
    const Vec3 b = world.v[static_cast<std::size_t>(tri[1])];
    const Vec3 c = world.v[static_cast<std::size_t>(tri[2])];
    return dot(cross(b - a, c - a), eye - a) > 0.0F;
}

/// Outline edges of a convex mesh: the edges between a front-facing and a back-facing triangle.
inline std::vector<std::pair<int, int>> silhouette(const Mesh& world, Vec3 eye) {
    struct Edge {
        std::uint64_t key;
        int a, b;
        bool front;
    };
    std::vector<Edge> edges;
    edges.reserve(world.t.size() * 3);
    for (const Tri& tri : world.t) {
        const bool front = frontFacing(world, tri, eye);
        for (int k = 0; k < 3; ++k) {
            const int a = tri[static_cast<std::size_t>(k)];
            const int b = tri[static_cast<std::size_t>((k + 1) % 3)];
            edges.push_back({(static_cast<std::uint64_t>(std::min(a, b)) << 32) | static_cast<std::uint64_t>(std::max(a, b)), std::min(a, b), std::max(a, b), front});
        }
    }
    std::sort(edges.begin(), edges.end(), [](const Edge& l, const Edge& r) { return l.key < r.key; });
    std::vector<std::pair<int, int>> out;
    for (std::size_t i = 0; i + 1 < edges.size(); ++i) {
        if (edges[i].key == edges[i + 1].key && edges[i].front != edges[i + 1].front) out.emplace_back(edges[i].a, edges[i].b);
    }
    return out;
}

// ---------------------------------------------------------------------------------------------- clipping

/// Cuts a polyline (2 points) or a convex polygon (3 or more) at the near plane in clip space (z + w >= 0).
/// Without this, anything that reaches behind the camera gets a negative w and lands mirrored on screen.
inline std::vector<Vec4> clipNear(const std::vector<Vec4>& in) {
    const auto dist = [](const Vec4& p) { return p.z + p.w; };
    const auto cut = [&](const Vec4& a, const Vec4& b) {
        const float s = dist(a) / (dist(a) - dist(b));
        return Vec4{a.x + (b.x - a.x) * s, a.y + (b.y - a.y) * s, a.z + (b.z - a.z) * s, a.w + (b.w - a.w) * s};
    };
    std::vector<Vec4> out;
    const std::size_t n = in.size();
    const std::size_t edges = n == 2 ? 1 : n;
    for (std::size_t i = 0; i < edges; ++i) {
        const Vec4& a = in[i];
        const Vec4& b = in[(i + 1) % n];
        const bool aIn = dist(a) >= 0.0F;
        const bool bIn = dist(b) >= 0.0F;
        if (aIn) out.push_back(a);
        if (aIn != bIn) out.push_back(cut(a, b));
        if (n == 2 && i + 1 == edges && bIn) out.push_back(b);
    }
    return out.size() < 2 ? std::vector<Vec4>{} : out;
}

}  // namespace pk
