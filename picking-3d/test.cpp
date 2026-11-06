// Hand-computed cases for the picking math. Plain checks instead of assert(), so a Release build (NDEBUG) still tests.
#include <cmath>
#include <cstdio>
#include <vector>

#include "logic.hpp"

using namespace pk;

namespace {

int g_failures = 0;

#define CHECK(cond)                                                    \
    do {                                                               \
        if (!(cond)) {                                                 \
            std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); \
            ++g_failures;                                              \
        }                                                              \
    } while (0)

bool near(float a, float b, float eps = 1e-3F) { return std::fabs(a - b) <= eps; }
bool near(Vec3 a, Vec3 b, float eps = 1e-3F) { return near(a.x, b.x, eps) && near(a.y, b.y, eps) && near(a.z, b.z, eps); }

constexpr float kPi = 3.14159265F;
const Viewport kVp{0.0F, 0.0F, 100.0F, 100.0F};

// Camera on +z looking at the origin, 90 degree field of view, square viewport: x/z at distance 1 maps to +-1 in NDC.
Mat4 squareViewProj() { return perspective(kPi / 2.0F, 1.0F, 0.1F, 100.0F) * lookAt({0, 0, 5}, {0, 0, 0}, {0, 1, 0}); }

void matrices() {
    const Mat4 vp = squareViewProj();
    // the target lands in the middle of the viewport, NDC depth 480.5 / 499.5
    const auto c = project(vp, kVp, {0, 0, 0});
    CHECK(c.has_value());
    CHECK(near(c->x, 50.0F) && near(c->y, 50.0F));
    CHECK(near(c->z, 480.5F / 499.5F, 1e-4F));
    // one unit to the right at distance 5: NDC x = 1/5, pixel = (0.2 * 0.5 + 0.5) * 100 = 60
    const auto r = project(vp, kVp, {1, 0, 0});
    CHECK(r.has_value() && near(r->x, 60.0F) && near(r->y, 50.0F));
    // up is smaller y in pixels
    const auto u = project(vp, kVp, {0, 1, 0});
    CHECK(u.has_value() && near(u->x, 50.0F) && near(u->y, 40.0F));
    // behind the camera: no pixel
    CHECK(!project(vp, kVp, {0, 0, 8}).has_value());
    // the viewport offset is added
    const auto o = project(vp, Viewport{10.0F, 30.0F, 100.0F, 100.0F}, {0, 0, 0});
    CHECK(o.has_value() && near(o->x, 60.0F) && near(o->y, 80.0F));
    // inverse * matrix = identity
    const Mat4 m = vp;
    const Mat4 id = inverse(m) * m;
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) CHECK(near(id.m[row][col], row == col ? 1.0F : 0.0F, 1e-3F));
    }
}

void roundTrip() {
    OrbitCamera cam;
    cam.yaw = 0.9F;
    cam.pitch = 0.4F;
    cam.distance = 9.0F;
    const Viewport vp{0.0F, 30.0F, 692.0F, 510.0F};
    const Mat4 viewProj = perspective(0.9F, vp.w / vp.h, 0.1F, 100.0F) * cam.view();
    const Mat4 inv = inverse(viewProj);
    const Vec3 points[] = {{0, 0, 0}, {1.5F, 0.5F, -2.0F}, {-3, 1, 2}, {2, 3, 1}, {0, 0.6F, 0}};
    for (const Vec3 p : points) {
        const auto px = project(viewProj, vp, p);
        CHECK(px.has_value());
        if (px) CHECK(near(unproject(inv, vp, px->x, px->y, px->z), p, 2e-3F));
    }
}

void pixelRays() {
    // 90 degree camera on +z: the top-left pixel looks along (-1, 1, -1) / sqrt 3
    const Mat4 vp = squareViewProj();
    const Ray corner = pixelToRay(inverse(vp), kVp, 0.0F, 0.0F, {0, 0, 5});
    const float s = 1.0F / std::sqrt(3.0F);
    CHECK(near(corner.origin, {0, 0, 5}));
    CHECK(near(corner.dir, {-s, s, -s}));
    CHECK(near(length(corner.dir), 1.0F));

    // the middle of the viewport looks exactly along the view direction, for any orbit
    OrbitCamera cam;
    cam.yaw = -1.1F;
    cam.pitch = 0.7F;
    cam.distance = 7.0F;
    const Viewport big{0.0F, 30.0F, 692.0F, 510.0F};
    const Mat4 viewProj = perspective(0.9F, big.w / big.h, 0.1F, 100.0F) * cam.view();
    const Ray mid = pixelToRay(inverse(viewProj), big, big.x + big.w / 2.0F, big.y + big.h / 2.0F, cam.eye());
    CHECK(near(mid.dir, normalize(cam.target - cam.eye()), 1e-4F));
    CHECK(near(mid.origin, cam.eye()));
}

void sphere() {
    const Ray down{{0, 0, 10}, {0, 0, -1}};
    CHECK(raySphere(down, {0, 0, 0}, 1.0F) && near(*raySphere(down, {0, 0, 0}, 1.0F), 9.0F));
    // tangent: discriminant 0, touches at z = 0
    const auto tangent = raySphere(down, {1, 0, 0}, 1.0F);
    CHECK(tangent && near(*tangent, 10.0F));
    CHECK(!raySphere(down, {1.5F, 0, 0}, 1.0F));
    // origin inside: hit at the origin itself
    const Ray inside{{0, 0, 0.5F}, {0, 0, -1}};
    CHECK(raySphere(inside, {0, 0, 0}, 1.0F) && near(*raySphere(inside, {0, 0, 0}, 1.0F), 0.0F));
    // sphere behind the origin: the line hits it, the ray does not
    CHECK(!raySphere(Ray{{0, 0, -10}, {0, 0, -1}}, {0, 0, 0}, 1.0F));
    // off-axis hit: center (0, 3, 0), r 5, ray along x from (-10, 0, 0): hits at x = -4 -> t = 6
    const auto off = raySphere(Ray{{-10, 0, 0}, {1, 0, 0}}, {0, 3, 0}, 5.0F);
    CHECK(off && near(*off, 6.0F));
}

void aabb() {
    const Vec3 lo{-1, -1, -1};
    const Vec3 hi{1, 1, 1};
    const auto front = rayAabb(Ray{{0, 0, 10}, {0, 0, -1}}, lo, hi);
    CHECK(front && near(*front, 9.0F));
    const auto offset = rayAabb(Ray{{0.5F, 0.5F, 10}, {0, 0, -1}}, lo, hi);
    CHECK(offset && near(*offset, 9.0F));
    CHECK(!rayAabb(Ray{{1.5F, 0, 10}, {0, 0, -1}}, lo, hi));
    // parallel to an axis, inside the slab of that axis: the other slabs decide
    const auto along = rayAabb(Ray{{-5, 0.5F, 0}, {1, 0, 0}}, lo, hi);
    CHECK(along && near(*along, 4.0F));
    // parallel and outside the slab: miss no matter what the other axes say
    CHECK(!rayAabb(Ray{{-5, 1.5F, 0}, {1, 0, 0}}, lo, hi));
    CHECK(!rayAabb(Ray{{-5, -1.5F, 0}, {1, 0, 0}}, lo, hi));
    // parallel exactly on the boundary plane counts as inside
    const auto edge = rayAabb(Ray{{-5, 1, 0}, {1, 0, 0}}, lo, hi);
    CHECK(edge && near(*edge, 4.0F));
    // start inside: t = 0
    const auto in = rayAabb(Ray{{0, 0, 0}, {1, 0, 0}}, lo, hi);
    CHECK(in && near(*in, 0.0F));
    // box behind the origin
    CHECK(!rayAabb(Ray{{5, 0, 0}, {1, 0, 0}}, lo, hi));
    // diagonal ray into the corner (-1,-1,-1): t = 4 * sqrt 3
    const float s = 1.0F / std::sqrt(3.0F);
    const auto diag = rayAabb(Ray{{-5, -5, -5}, {s, s, s}}, lo, hi);
    CHECK(diag && near(*diag, 4.0F * std::sqrt(3.0F)));
    // diagonal ray that passes the box on the outside
    CHECK(!rayAabb(Ray{{-5, 4, 0}, {s, s, s}}, lo, hi));
    // off-center box: [2,4] x [0,1] x [0,1], ray along +x at y = z = 0.5 from the origin
    const auto moved = rayAabb(Ray{{0, 0.5F, 0.5F}, {1, 0, 0}}, {2, 0, 0}, {4, 1, 1});
    CHECK(moved && near(*moved, 2.0F));
}

void plane() {
    const Vec3 p{0, 0, 0};
    const Vec3 n{0, 1, 0};
    const auto straight = rayPlane(Ray{{0, 5, 0}, {0, -1, 0}}, p, n);
    CHECK(straight && near(*straight, 5.0F));
    const float s = 1.0F / std::sqrt(2.0F);
    const auto slanted = rayPlane(Ray{{0, 5, 0}, {s, -s, 0}}, p, n);
    CHECK(slanted && near(*slanted, 5.0F * std::sqrt(2.0F)));
    CHECK(!rayPlane(Ray{{0, 5, 0}, {1, 0, 0}}, p, n));    // parallel
    CHECK(!rayPlane(Ray{{0, 5, 0}, {0, 1, 0}}, p, n));    // pointing away
    const auto on = rayPlane(Ray{{3, 0, 3}, {0, -1, 0}}, p, n);
    CHECK(on && near(*on, 0.0F));                          // starts on the plane
    const auto below = rayPlane(Ray{{0, -2, 0}, {0, 1, 0}}, p, n);
    CHECK(below && near(*below, 2.0F));                    // from below, the normal does not matter
}

void picking() {
    // far one first in the list: the nearest hit has to win, not the first
    const std::vector<Object> objects{
        {Object::Kind::Sphere, {0, 0, -5}, {1, 0, 0}, "far"},
        {Object::Kind::Sphere, {0, 0, 0}, {1, 0, 0}, "near"},
        {Object::Kind::Box, {4, 0, 0}, {1, 1, 1}, "side"},
    };
    const auto hit = pick(objects, Ray{{0, 0, 10}, {0, 0, -1}});
    CHECK(hit && hit->object == 1 && !hit->floor && near(hit->t, 9.0F));
    const auto box = pick(objects, Ray{{4, 0, 10}, {0, 0, -1}});
    CHECK(box && box->object == 2 && near(box->t, 9.0F));
    // the ray misses everything but flies into the floor
    const auto floor = pick(objects, Ray{{8, 5, 0}, {0, -1, 0}});
    CHECK(floor && floor->floor && floor->object == -1 && near(floor->t, 5.0F));
    // an object in front of the floor spot wins over the floor behind it
    const auto over = pick(objects, Ray{{0, 5, 0}, {0, -1, 0}});
    CHECK(over && over->object == 1 && near(over->t, 4.0F));
    // looking at the sky: nothing
    CHECK(!pick(objects, Ray{{8, 5, 0}, {0, 1, 0}}));
    CHECK(!pick({}, Ray{{0, 1, 0}, {1, 0, 0}}));
}

void pipeline() {
    // the full chain: world -> pixel -> ray -> pick. The pixel of an object's center must pick that object
    // (or something in front of it).
    const std::vector<Object> scene = defaultScene();
    CHECK(scene.size() >= 5 && scene.size() <= 8);
    OrbitCamera cam;
    const Viewport vp{0.0F, 30.0F, 692.0F, 510.0F};
    const Mat4 viewProj = perspective(kFovY, vp.w / vp.h, kNear, kFar) * cam.view();
    const Mat4 inv = inverse(viewProj);
    for (std::size_t i = 0; i < scene.size(); ++i) {
        const auto px = project(viewProj, vp, scene[i].center);
        CHECK(px.has_value());
        if (!px) continue;
        const Ray ray = pixelToRay(inv, vp, px->x, px->y, cam.eye());
        const auto hit = pick(scene, ray);
        CHECK(hit.has_value() && !hit->floor);
        if (hit) {
            const float toCenter = length(scene[i].center - cam.eye());
            CHECK(hit->object == static_cast<int>(i) || hit->t < toCenter);
        }
    }
    // the objects do not overlap each other (needed for the depth sort by object)
    for (std::size_t i = 0; i < scene.size(); ++i) {
        for (std::size_t j = i + 1; j < scene.size(); ++j) CHECK(!overlap(scene[i], scene[j]));
    }
    // and they all sit on or above the floor
    for (const Object& o : scene) CHECK(boundsMin(o).y >= -1e-4F);
}

void camera() {
    OrbitCamera cam;
    cam.target = {0, 0, 0};
    cam.yaw = 0.0F;
    cam.pitch = 0.0F;
    cam.distance = 10.0F;
    CHECK(near(cam.eye(), {0, 0, 10}));
    cam.yaw = kPi / 2.0F;
    CHECK(near(cam.eye(), {10, 0, 0}));
    cam.yaw = 0.0F;
    cam.pitch = kPi / 4.0F;
    CHECK(near(cam.eye(), {0, 10.0F * std::sin(kPi / 4.0F), 10.0F * std::cos(kPi / 4.0F)}));
    // pitch is clamped so the camera never flips over the pole
    cam.orbit(0.0F, 100.0F);
    CHECK(near(cam.pitch, kMaxPitch));
    cam.orbit(0.0F, -100.0F);
    CHECK(near(cam.pitch, kMinPitch));
    // zoom shrinks the distance by 10 % per notch and stays inside the limits
    cam.distance = 10.0F;
    cam.zoom(1.0F);
    CHECK(near(cam.distance, 9.0F));
    cam.zoom(-1.0F);
    CHECK(near(cam.distance, 10.0F));
    cam.zoom(1000.0F);
    CHECK(near(cam.distance, kMinDistance));
    cam.zoom(-1000.0F);
    CHECK(near(cam.distance, kMaxDistance));
}

void meshes() {
    const Mesh box = unitBox();
    CHECK(box.v.size() == 8 && box.t.size() == 12);
    for (const auto& tri : box.t) {
        const Vec3 a = box.v[static_cast<std::size_t>(tri[0])];
        const Vec3 b = box.v[static_cast<std::size_t>(tri[1])];
        const Vec3 c = box.v[static_cast<std::size_t>(tri[2])];
        const Vec3 centroid = (a + b + c) * (1.0F / 3.0F);
        CHECK(dot(cross(b - a, c - a), centroid) > 0.0F);  // counter-clockwise seen from outside
    }
    const Mesh ico = icosphere(2);
    CHECK(ico.t.size() == 320 && ico.v.size() == 162);
    for (const Vec3 v : ico.v) CHECK(near(length(v), 1.0F, 1e-4F));
    for (const auto& tri : ico.t) {
        const Vec3 a = ico.v[static_cast<std::size_t>(tri[0])];
        const Vec3 b = ico.v[static_cast<std::size_t>(tri[1])];
        const Vec3 c = ico.v[static_cast<std::size_t>(tri[2])];
        CHECK(dot(cross(b - a, c - a), a + b + c) > 0.0F);
    }
}

void silhouettes() {
    const Mesh box = unitBox();
    // straight onto one face: the outline is the 4 sides of that face, its diagonal is no silhouette
    CHECK(silhouette(box, {0, 0, 10}).size() == 4);
    // onto a corner: the classic hexagon
    CHECK(silhouette(box, {10, 10, 10}).size() == 6);
    // a sphere: one closed loop, so every vertex on it has exactly two silhouette edges
    const Mesh ico = icosphere(2);
    const auto edges = silhouette(ico, {3, 2, 7});
    CHECK(edges.size() > 8);
    std::vector<int> degree(ico.v.size(), 0);
    for (const auto& e : edges) {
        ++degree[static_cast<std::size_t>(e.first)];
        ++degree[static_cast<std::size_t>(e.second)];
    }
    for (const int d : degree) CHECK(d == 0 || d == 2);
}

void clipping() {
    // clip space z + w >= 0 is the near plane. A segment from in front of the camera to behind it gets cut.
    const Mat4 vp = squareViewProj();
    const Vec4 front = vp * Vec4{0, 0, 0, 1};
    const Vec4 behind = vp * Vec4{0, 0, 9, 1};  // eye is at z = 5
    const auto cut = clipNear({front, behind});
    CHECK(cut.size() == 2);
    if (cut.size() == 2) {
        CHECK(near(cut[0].z + cut[0].w, 0.0F, 1e-3F) || near(cut[1].z + cut[1].w, 0.0F, 1e-3F));
        CHECK(cut[0].w > 0.0F && cut[1].w > 0.0F);
    }
    CHECK(clipNear({behind, vp * Vec4{1, 0, 10, 1}}).empty());
    CHECK(clipNear({front, vp * Vec4{1, 0, 0, 1}}).size() == 2);
    // a triangle with one vertex behind the camera becomes a quad
    CHECK(clipNear({front, vp * Vec4{1, 0, 0, 1}, behind}).size() == 4);
}

}  // namespace

int main() {
    matrices();
    roundTrip();
    pixelRays();
    sphere();
    aabb();
    plane();
    picking();
    pipeline();
    camera();
    meshes();
    silhouettes();
    clipping();
    if (g_failures != 0) {
        std::printf("%d check(s) failed\n", g_failures);
        return 1;
    }
    std::printf("picking-3d logic: all checks passed\n");
    return 0;
}
