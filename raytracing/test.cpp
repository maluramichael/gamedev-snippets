// Hand-computed checks for the ray tracer math. No SDL. Release builds define NDEBUG, so no assert() here:
// CHECK always runs and the program exits with 1 on the first failed batch.

#include <cmath>
#include <cstdint>
#include <cstdio>

#include "logic.hpp"

namespace {

int failures = 0;

#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            std::printf("FAILED line %d: %s\n", __LINE__, #cond);            \
            ++failures;                                                      \
        }                                                                    \
    } while (0)

bool near(float a, float b, float eps = 1e-4F) { return std::fabs(a - b) <= eps; }
bool near(rt::Vec3 a, rt::Vec3 b, float eps = 1e-4F) { return near(a.x, b.x, eps) && near(a.y, b.y, eps) && near(a.z, b.z, eps); }

constexpr float kFar = 1e30F;

void vectors() {
    const rt::Vec3 a{1, 2, 3};
    const rt::Vec3 b{4, -5, 6};
    CHECK(near(rt::dot(a, b), 4 - 10 + 18));
    CHECK(near(rt::length(rt::Vec3{3, 4, 12}), 13.0F));
    CHECK(near(rt::normalize(rt::Vec3{0, 0, 7}), rt::Vec3{0, 0, 1}));
    CHECK(near(rt::length(rt::normalize(rt::Vec3{1, 2, 3})), 1.0F));
    CHECK(near(a + b, rt::Vec3{5, -3, 9}));
    CHECK(near(a - b, rt::Vec3{-3, 7, -3}));
    CHECK(near(a * 2.0F, rt::Vec3{2, 4, 6}));
    CHECK(near(rt::lerp(rt::Vec3{0, 0, 0}, rt::Vec3{10, 20, 30}, 0.25F), rt::Vec3{2.5F, 5, 7.5F}));
    // a zero vector must not turn into NaN
    const rt::Vec3 z = rt::normalize(rt::Vec3{0, 0, 0});
    CHECK(near(z, rt::Vec3{0, 0, 0}));
}

void reflection() {
    // falling straight down onto a floor bounces straight up
    CHECK(near(rt::reflect(rt::Vec3{0, -1, 0}, rt::Vec3{0, 1, 0}), rt::Vec3{0, 1, 0}));
    // 45 degrees: (1,-1,0) off a floor becomes (1,1,0)
    CHECK(near(rt::reflect(rt::Vec3{1, -1, 0}, rt::Vec3{0, 1, 0}), rt::Vec3{1, 1, 0}));
    // head on to a wall facing -z goes straight back
    CHECK(near(rt::reflect(rt::Vec3{0, 0, 1}, rt::Vec3{0, 0, -1}), rt::Vec3{0, 0, -1}));
    // grazing along the surface does not change
    CHECK(near(rt::reflect(rt::Vec3{1, 0, 0}, rt::Vec3{0, 1, 0}), rt::Vec3{1, 0, 0}));
    // reflection keeps the length
    const rt::Vec3 d = rt::normalize(rt::Vec3{0.3F, -0.8F, 0.5F});
    CHECK(near(rt::length(rt::reflect(d, rt::Vec3{0, 1, 0})), 1.0F));
}

void raySphere() {
    const rt::Sphere s{{0, 0, 5}, 1.0F, {1, 1, 1}, 0.0F};
    float t = 0;

    // straight on: front surface at z = 4
    CHECK(rt::hitSphere({{0, 0, 0}, {0, 0, 1}}, s, 0.0F, kFar, t));
    CHECK(near(t, 4.0F));

    // from the inside: origin in the center, the far surface is at t = radius
    CHECK(rt::hitSphere({{0, 0, 5}, {1, 0, 0}}, s, 0.0F, kFar, t));
    CHECK(near(t, 1.0F));

    // grazes the top: tangent ray at y = 1 has a discriminant of 0 (counts as a hit, t = 5)
    CHECK(rt::hitSphere({{0, 1, 0}, {0, 0, 1}}, s, 0.0F, kFar, t));
    CHECK(near(t, 5.0F, 1e-3F));

    // just misses: y = 1.001
    CHECK(!rt::hitSphere({{0, 1.001F, 0}, {0, 0, 1}}, s, 0.0F, kFar, t));

    // sphere behind the origin
    CHECK(!rt::hitSphere({{0, 0, 0}, {0, 0, -1}}, s, 0.0F, kFar, t));

    // tMax cuts the ray off before the surface
    CHECK(!rt::hitSphere({{0, 0, 0}, {0, 0, 1}}, s, 0.0F, 3.5F, t));
    CHECK(rt::hitSphere({{0, 0, 0}, {0, 0, 1}}, s, 0.0F, 4.5F, t));

    // off center: ray at x = 0.6 hits at z = 5 - sqrt(1 - 0.36) = 4.2
    CHECK(rt::hitSphere({{0.6F, 0, 0}, {0, 0, 1}}, s, 0.0F, kFar, t));
    CHECK(near(t, 4.2F));

    // tMin skips the near root and returns the far one
    CHECK(rt::hitSphere({{0, 0, 0}, {0, 0, 1}}, s, 4.5F, kFar, t));
    CHECK(near(t, 6.0F));
}

void rayPlane() {
    float t = 0;
    // from (0,2,0) down to y = 0: t = 2
    CHECK(rt::hitPlane({{0, 2, 0}, {0, -1, 0}}, 0.0F, 0.0F, kFar, t));
    CHECK(near(t, 2.0F));
    // slanted: direction (0,-1,1) is not unit length, t scales with it: reaches y = 0 at t = 2
    CHECK(rt::hitPlane({{0, 2, 0}, {0, -1, 1}}, 0.0F, 0.0F, kFar, t));
    CHECK(near(t, 2.0F));
    // plane at y = 1: from y = 3 going down, t = 2
    CHECK(rt::hitPlane({{5, 3, 5}, {0, -1, 0}}, 1.0F, 0.0F, kFar, t));
    CHECK(near(t, 2.0F));
    // parallel to the plane never hits, even exactly on it
    CHECK(!rt::hitPlane({{0, 2, 0}, {1, 0, 0}}, 0.0F, 0.0F, kFar, t));
    CHECK(!rt::hitPlane({{0, 0, 0}, {1, 0, 0}}, 0.0F, 0.0F, kFar, t));
    // pointing away from the plane: negative t is no hit
    CHECK(!rt::hitPlane({{0, 2, 0}, {0, 1, 0}}, 0.0F, 0.0F, kFar, t));
    // tMax
    CHECK(!rt::hitPlane({{0, 2, 0}, {0, -1, 0}}, 0.0F, 0.0F, 1.5F, t));
}

void checkerboard() {
    // tile (0,0) is the first color, neighbours alternate
    // (tiles are kTileSize = 1.5 wide: 0 to 1.5 is the first tile, 1.5 to 3 the second)
    CHECK(rt::checkerIndex(0.5F, 0.5F) == 0);
    CHECK(rt::checkerIndex(1.4F, 1.4F) == 0);  // still inside the first tile
    CHECK(rt::checkerIndex(2.0F, 0.5F) == 1);
    CHECK(rt::checkerIndex(0.5F, 2.0F) == 1);
    CHECK(rt::checkerIndex(2.0F, 2.0F) == 0);
    // negative coordinates continue the pattern without a double wide tile at 0
    CHECK(rt::checkerIndex(-0.5F, 0.5F) == 1);
    CHECK(rt::checkerIndex(-0.5F, -0.5F) == 0);
    CHECK(rt::checkerIndex(-2.0F, 0.5F) == 0);
    CHECK(rt::checkerIndex(-0.001F, 0.5F) == 1);
}

void camera() {
    rt::Camera cam;
    cam.pos = {0, 1, -5};
    cam.tanHalfFov = 1.0F;  // 90 degrees vertical
    cam.shift = 0.0F;
    // the center of a 2x2 image is the corner between all four pixels; for an odd size the middle pixel looks straight ahead
    const rt::Ray mid = rt::primaryRay(cam, 1, 1, 3, 3);
    CHECK(near(mid.origin, cam.pos));
    CHECK(near(mid.dir, rt::Vec3{0, 0, 1}));
    // top-left pixel of 3x3, aspect 1: (-2/3, +2/3, 1) normalized
    const rt::Ray tl = rt::primaryRay(cam, 0, 0, 3, 3);
    CHECK(near(tl.dir, rt::normalize(rt::Vec3{-2.0F / 3.0F, 2.0F / 3.0F, 1.0F})));
    // bottom-right mirrors it
    const rt::Ray br = rt::primaryRay(cam, 2, 2, 3, 3);
    CHECK(near(br.dir, rt::normalize(rt::Vec3{2.0F / 3.0F, -2.0F / 3.0F, 1.0F})));
    // wide image: horizontal extent is scaled by the aspect ratio (4:1 -> x reaches 4 * 0.75 = 3 at the first pixel of 4)
    const rt::Ray wide = rt::primaryRay(cam, 0, 0, 4, 1);
    CHECK(near(wide.dir.x / wide.dir.z, -0.75F * 4.0F));
    // shift moves the horizon: with shift 0.5 the middle pixel looks 0.5 down
    cam.shift = 0.5F;
    CHECK(near(rt::primaryRay(cam, 1, 1, 3, 3).dir, rt::normalize(rt::Vec3{0, -0.5F, 1})));
    // project() is the inverse of primaryRay() for the pixel centers (it needs the aspect ratio of the image)
    cam.aspect = 1.0F;
    float u = 0;
    float v = 0;
    CHECK(rt::project(cam, cam.pos + rt::primaryRay(cam, 0, 0, 3, 3).dir * 7.0F, u, v));
    CHECK(near(u, 1.0F / 6.0F) && near(v, 1.0F / 6.0F));
    CHECK(rt::project(cam, cam.pos + rt::primaryRay(cam, 2, 1, 3, 3).dir * 3.0F, u, v));
    CHECK(near(u, 5.0F / 6.0F) && near(v, 0.5F));
    // behind the camera nothing projects
    CHECK(!rt::project(cam, rt::Vec3{0, 1, -9}, u, v));
}

void sky() {
    CHECK(near(rt::skyColor(rt::Vec3{0, 1, 0}), rt::kSkyZenith));
    CHECK(near(rt::skyColor(rt::Vec3{0, 0, 1}), rt::kSkyHorizon));
    CHECK(near(rt::skyColor(rt::Vec3{0, -1, 0}), rt::kSkyHorizon));  // below the horizon stays horizon colored
    const rt::Vec3 mid = rt::skyColor(rt::normalize(rt::Vec3{0, 1, 1}));
    CHECK(mid.x > rt::kSkyZenith.x && mid.x < rt::kSkyHorizon.x);
}

rt::Scene emptyScene() {
    rt::Scene s;
    s.spheres.clear();
    s.floor = false;
    s.light = {3, 0, 0};
    return s;
}

void shading() {
    // matt sphere at z = 5, light at (3,0,0). Hit point (0,0,4), normal (0,0,-1), direction to the light
    // (3,0,-4)/5, so n.l = 0.8 and the brightness is ambient + (1 - ambient) * 0.8 = 0.2 + 0.8 * 0.8 = 0.84.
    rt::Scene s = emptyScene();
    s.spheres.push_back({{0, 0, 5}, 1.0F, {1.0F, 0.5F, 0.25F}, 0.0F});
    rt::Options o;
    o.shadows = false;
    o.reflections = false;
    std::uint64_t rays = 0;
    const rt::Vec3 c = rt::trace(s, {{0, 0, 0}, {0, 0, 1}}, 0, o, rays);
    CHECK(near(c, rt::Vec3{0.84F, 0.42F, 0.21F}, 1e-3F));
    CHECK(rays == 1);

    // a miss returns the sky and costs one ray
    rays = 0;
    CHECK(near(rt::trace(s, {{0, 0, 0}, {0, 1, 0}}, 0, o, rays), rt::kSkyZenith));
    CHECK(rays == 1);

    // light behind the surface: only ambient, and no shadow ray is wasted on it
    s.light = {0, 0, 20};
    o.shadows = true;
    rays = 0;
    const rt::Vec3 dark = rt::trace(s, {{0, 0, 0}, {0, 0, 1}}, 0, o, rays);
    CHECK(near(dark, rt::Vec3{0.2F, 0.1F, 0.05F}, 1e-3F));
    CHECK(rays == 1);
}

void shadows() {
    // light straight above a small sphere, big floor-less blocker in between
    rt::Scene s = emptyScene();
    s.spheres.push_back({{0, 0, 0}, 1.0F, {1, 1, 1}, 0.0F});    // receiver
    s.spheres.push_back({{0, 3, 0}, 0.5F, {1, 1, 1}, 0.0F});    // blocker
    s.light = {0, 6, 0};
    rt::Options o;
    o.reflections = false;
    std::uint64_t rays = 0;

    // looking straight down at the top of the receiver from the side is awkward, so shoot a ray that hits the
    // receiver at (0,1,0): origin (0,2,0) is inside the blocker's shadow cone, the blocker is at y 2.5..3.5
    const rt::Ray down{{0, 2.0F, 0}, {0, -1, 0}};
    o.shadows = true;
    const rt::Vec3 shadowed = rt::trace(s, down, 0, o, rays);
    CHECK(near(shadowed, rt::Vec3{0.2F, 0.2F, 0.2F}, 1e-3F));  // ambient only
    CHECK(rays == 2);                                         // primary + shadow ray

    rays = 0;
    o.shadows = false;
    const rt::Vec3 flat = rt::trace(s, down, 0, o, rays);
    CHECK(near(flat, rt::Vec3{1, 1, 1}, 1e-3F));  // n.l = 1, fully lit without shadow rays
    CHECK(rays == 1);

    // a point that sees the light: same receiver, light moved to the side of the blocker
    s.light = {6, 2, 0};
    o.shadows = true;
    rays = 0;
    const rt::Ray side{{4.0F, 0.0F, 0}, {-1, 0, 0}};  // hits the receiver at (1,0,0), facing +x, light is on that side
    const rt::Vec3 lit = rt::trace(s, side, 0, o, rays);
    CHECK(lit.x > 0.9F);
    CHECK(rays == 2);
}

void shadowAcne() {
    // A point that floating point rounding left 1e-4 UNDER the surface of a unit sphere at the origin.
    const rt::Sphere sphere{{0, 0, 0}, 1.0F, {1, 1, 1}, 0.0F};
    const rt::Vec3 normal{0, 1, 0};
    const rt::Vec3 inside = rt::Vec3{0, 1.0F - 1e-4F, 0};
    const rt::Vec3 light{0, 5, 0};
    const rt::Sphere spheres[] = {sphere};

    // Without a bias the shadow ray starts inside the sphere, finds the surface 1e-4 ahead and calls it a blocker.
    CHECK(rt::occluded(inside, normal, light, spheres, 1, 0.0F));
    // With the bias the origin is lifted along the normal and the surface is not found.
    CHECK(!rt::occluded(inside, normal, light, spheres, 1, rt::kShadowBias));
    // A real blocker is still found with the bias.
    const rt::Sphere blocker{{0, 3, 0}, 0.5F, {1, 1, 1}, 0.0F};
    const rt::Sphere both[] = {sphere, blocker};
    CHECK(rt::occluded(inside, normal, light, both, 2, rt::kShadowBias));
    // A blocker BEHIND the light does not count (tMax is the distance to the light).
    const rt::Sphere behindLight{{0, 8, 0}, 0.5F, {1, 1, 1}, 0.0F};
    const rt::Sphere beyond[] = {sphere, behindLight};
    CHECK(!rt::occluded(inside, normal, light, beyond, 2, rt::kShadowBias));
}

void floorAndFog() {
    rt::Scene s = emptyScene();
    s.floor = true;
    s.floorY = 0.0F;
    s.light = {0, 10, 0};
    rt::Options o;
    o.shadows = false;
    o.reflections = false;
    std::uint64_t rays = 0;
    // straight down onto tile (0,0): first tile color, n.l = 1 (light right above), so brightness is 1
    const rt::Vec3 c = rt::trace(s, {{0.5F, 2.0F, 0.5F}, {0, -1, 0}}, 0, o, rays);
    CHECK(near(c, rt::kTileA, 0.01F));
    const rt::Vec3 d = rt::trace(s, {{2.0F, 2.0F, 0.5F}, {0, -1, 0}}, 0, o, rays);
    CHECK(near(d, rt::kTileB, 0.01F));
    // far away the floor fades to the horizon color: at t >= fogEnd it is exactly the sky at the horizon
    const rt::Vec3 far = rt::trace(s, {{0.5F, 1.0F, 0.5F}, rt::normalize(rt::Vec3{0, -1.0F / 200.0F, 1})}, 0, o, rays);
    CHECK(near(far, rt::kSkyHorizon, 1e-3F));
}

void floorReflection() {
    // a polished floor: straight down onto tile A with the light right above, the bounce goes straight up into the sky
    rt::Scene s = emptyScene();
    s.floor = true;
    s.floorReflectivity = 0.2F;
    s.light = {0.5F, 10, 0.5F};
    rt::Options o;
    o.shadows = false;
    o.reflections = true;
    std::uint64_t rays = 0;
    const rt::Vec3 c = rt::trace(s, {{0.5F, 2.0F, 0.5F}, {0, -1, 0}}, 0, o, rays);
    CHECK(near(c, rt::lerp(rt::kTileA, rt::kSkyZenith, 0.2F), 1e-3F));
    CHECK(rays == 2);
    // with reflections off the same pixel costs one ray and shows the plain tile
    o.reflections = false;
    rays = 0;
    CHECK(near(rt::trace(s, {{0.5F, 2.0F, 0.5F}, {0, -1, 0}}, 0, o, rays), rt::kTileA, 1e-3F));
    CHECK(rays == 1);
}

void reflectionDepth() {
    // two mirror balls facing each other, the ray starts between them and bounces along the z axis for ever.
    rt::Scene s = emptyScene();
    s.spheres.push_back({{0, 0, 5}, 1.0F, {1, 1, 1}, 1.0F});
    s.spheres.push_back({{0, 0, -5}, 1.0F, {1, 1, 1}, 1.0F});
    s.light = {0, 100, 0};
    rt::Options o;
    o.shadows = false;
    o.reflections = true;
    o.maxDepth = 3;
    std::uint64_t rays = 0;
    rt::trace(s, {{0, 0, 0}, {0, 0, 1}}, 0, o, rays);
    CHECK(rays == 4);  // depth 0, 1, 2, 3 and then it stops

    o.maxDepth = 0;
    rays = 0;
    rt::trace(s, {{0, 0, 0}, {0, 0, 1}}, 0, o, rays);
    CHECK(rays == 1);

    // reflections off: a mirror ball is just its base color, one ray
    o.reflections = false;
    o.maxDepth = 3;
    rays = 0;
    rt::trace(s, {{0, 0, 0}, {0, 0, 1}}, 0, o, rays);
    CHECK(rays == 1);
}

void mirrorShowsSky() {
    // mirror ball under the open sky: the ray goes in at the equator and the reflection leaves straight back,
    // so we see whatever is behind the camera. Looking along +z at a ball at z = 5 gives a reflection along -z: the horizon.
    rt::Scene s = emptyScene();
    s.spheres.push_back({{0, 0, 5}, 1.0F, {0, 0, 0}, 1.0F});
    s.light = {0, 100, 0};
    rt::Options o;
    o.shadows = false;
    o.reflections = true;
    std::uint64_t rays = 0;
    const rt::Vec3 c = rt::trace(s, {{0, 0, 0}, {0, 0, 1}}, 0, o, rays);
    CHECK(near(c, rt::kSkyHorizon, 1e-3F));  // base color black, reflectivity 1
    CHECK(rays == 2);
}

void packing() {
    CHECK(rt::pack(rt::Vec3{1, 1, 1}) == 0xFFFFFFFFU);
    CHECK(rt::pack(rt::Vec3{0, 0, 0}) == 0xFF000000U);
    CHECK(rt::pack(rt::Vec3{1, 0.5F, 0}) == 0xFFFF8000U);  // 0.5 * 255 = 127.5 rounds to 128
    CHECK(rt::pack(rt::Vec3{2, -1, 0}) == 0xFFFF0000U);    // out of range values are clamped
}

void lightFollowsMouse() {
    const rt::Camera cam;  // default camera
    const float aspect = 16.0F / 9.0F;
    // The floor point (1, 0, 4) seen on screen: pointing at it puts the light 3 above it.
    float u = 0;
    float v = 0;
    CHECK(rt::project(cam, rt::Vec3{1, 0, 4}, u, v));
    const rt::Vec3 l = rt::lightFromMouse(cam, u, v, aspect);
    CHECK(near(l.x, 1.0F, 1e-3F));
    CHECK(near(l.y, rt::kLightHeight));
    CHECK(near(l.z, 4.0F, 1e-3F));
    // above the horizon (the sky) there is no floor: the light goes to the far end, x follows the mouse
    const rt::Vec3 far = rt::lightFromMouse(cam, 0.5F, 0.05F, aspect);
    CHECK(near(far.z, rt::kLightZMax));
    CHECK(near(far.x, 0.0F, 1e-3F));
    // far to the left on the floor is clamped to the box
    const rt::Vec3 left = rt::lightFromMouse(cam, 0.0F, 0.6F, aspect);
    CHECK(left.x >= -rt::kLightXMax - 1e-3F && left.z <= rt::kLightZMax);
    CHECK(left.x < 0.0F);
}

}  // namespace

int main() {
    packing();
    lightFollowsMouse();
    vectors();
    reflection();
    raySphere();
    rayPlane();
    checkerboard();
    camera();
    sky();
    shading();
    shadows();
    shadowAcne();
    floorAndFog();
    floorReflection();
    reflectionDepth();
    mirrorShowsSky();
    if (failures != 0) {
        std::printf("%d check(s) failed\n", failures);
        return 1;
    }
    std::printf("all raytracing checks passed\n");
    return 0;
}
