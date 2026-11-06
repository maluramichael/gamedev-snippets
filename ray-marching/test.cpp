// Tests for the CPU mirror of the shader math in logic.hpp. Hand-computed cases first, then the properties the demo relies on.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include "logic.hpp"

namespace {

int failures = 0;

#define CHECK(cond)                                                                    \
    do {                                                                               \
        if (!(cond)) {                                                                 \
            std::printf("FAILED %s:%d: %s\n", __FILE__, __LINE__, #cond);              \
            ++failures;                                                                \
        }                                                                              \
    } while (0)

bool near(double a, double b, double eps = 1e-5) { return std::fabs(a - b) <= eps; }

using namespace gd::march;

void sphereDistances() {
    CHECK(near(sdSphere({3, 0, 0}, 1.0F), 2.0));    // outside: distance to the centre minus the radius
    CHECK(near(sdSphere({0, 0, 0}, 1.0F), -1.0));   // inside: negative, the depth below the surface
    CHECK(near(sdSphere({0, -1, 0}, 1.0F), 0.0));   // on the surface
    CHECK(near(sdSphere({1, 2, 2}, 1.0F), 2.0));    // |(1,2,2)| = 3
}

void boxDistances() {
    const Vec3 b{1, 1, 1};
    CHECK(near(sdBox({3, 0, 0}, b), 2.0));                    // in front of a face
    CHECK(near(sdBox({2, 2, 0}, b), std::sqrt(2.0)));         // beside an edge
    CHECK(near(sdBox({2, 2, 2}, b), std::sqrt(3.0)));         // beside a corner
    CHECK(near(sdBox({0, 0, 0}, b), -1.0));                   // centre: depth to the nearest face
    CHECK(near(sdBox({0.5F, 0, 0}, b), -0.5));
    CHECK(near(sdBox({1, 0.3F, -0.2F}, b), 0.0));             // on a face
    CHECK(near(sdRoundBox({3, 0, 0}, b, 0.25F), 1.75));       // rounding eats 0.25 from the outside
}

void torusDistances() {
    // Ring radius 2 in the xz plane, tube radius 0.5.
    CHECK(near(sdTorus({2, 0, 0}, 2.0F, 0.5F), -0.5));        // on the core circle: the full tube radius inside
    CHECK(near(sdTorus({3, 0, 0}, 2.0F, 0.5F), 0.5));         // 1 outside the core, tube radius 0.5 -> 0.5
    CHECK(near(sdTorus({2.5F, 0, 0}, 2.0F, 0.5F), 0.0));      // on the surface
    CHECK(near(sdTorus({0, 0, 0}, 2.0F, 0.5F), 1.5));         // the hole: 2 to the ring minus 0.5
    CHECK(near(sdTorus({0, 1, 0}, 2.0F, 0.5F), std::sqrt(5.0) - 0.5));
}

void smoothUnion() {
    CHECK(near(smin(1.0F, 1.0F, 1.0F), 0.75));                // equal inputs: the maximum dip, k/4 below the minimum
    CHECK(near(smin(0.0F, 2.0F, 1.0F), 0.0));                 // further apart than k: plain min
    CHECK(near(smin(0.5F, 0.0F, 1.0F), -0.0625));            // |a-b| = 0.5, h = 0.5 -> 0 - 0.5 * 0.5 * 1 / 4
    CHECK(near(smin(2.0F, 3.0F, 0.0F), 2.0));                 // k = 0 degrades to min instead of dividing by zero
    CHECK(near(smin(1.0F, 2.0F, 0.5F), smin(2.0F, 1.0F, 0.5F)));  // symmetric
    // Bounds on a grid: never above min, never more than k/4 below it.
    for (float k : {0.1F, 0.3F, 0.8F}) {
        for (float a = -2.0F; a <= 2.0F; a += 0.07F) {
            for (float b = -2.0F; b <= 2.0F; b += 0.07F) {
                const float m = std::fmin(a, b);
                const float s = smin(a, b, k);
                CHECK(s <= m + 1e-6F);
                CHECK(s >= m - k / 4.0F - 1e-6F);
            }
        }
    }
}

void rotations() {
    const Vec3 r = rotY({1, 0, 0}, 3.14159265F / 2.0F);       // a quarter turn around y moves +x to -z
    CHECK(near(r.x, 0.0, 1e-5) && near(r.y, 0.0) && near(r.z, -1.0, 1e-5));
    const Vec3 q = rotX({0, 1, 0}, 3.14159265F / 2.0F);       // a quarter turn around x moves +y to +z
    CHECK(near(q.x, 0.0) && near(q.y, 0.0, 1e-5) && near(q.z, 1.0, 1e-5));
}

// A unit sphere at z = 5 as a plain SDF.
const auto kSphereAt5 = [](Vec3 p) { return sdSphere(p - Vec3{0, 0, 5}, 1.0F); };

void marchHitsSphere() {
    const Hit h = march({0, 0, 0}, {0, 0, 1}, kSphereAt5, 64);
    CHECK(h.hit);
    CHECK(near(h.t, 4.0, 2e-3));    // the front of the sphere is at z = 4
    CHECK(h.steps == 2);            // head on with an exact SDF: first step jumps the whole 4, second sees d < eps
}

void marchGrazingTakesMoreSteps() {
    const Hit head = march({0, 0, 0}, {0, 0, 1}, kSphereAt5, 128);
    // Slightly inside the silhouette: the ray skims the sphere, every step is tiny.
    const Hit graze = march({0.995F, 0, 0}, {0, 0, 1}, kSphereAt5, 128);
    CHECK(graze.hit);
    CHECK(graze.steps > head.steps + 5);
    CHECK(graze.steps <= 128);
    // The same ray with a low limit runs out of steps before it arrives: not a hit, flagged as exhausted.
    const Hit cut = march({0.995F, 0, 0}, {0, 0, 1}, kSphereAt5, 6);
    CHECK(!cut.hit);
    CHECK(cut.exhausted);
    CHECK(cut.steps == 6);
}

void marchMisses() {
    const Hit h = march({1.5F, 0, 0}, {0, 0, 1}, kSphereAt5, 64);   // flies past at distance 1.5, 0.5 clearance
    CHECK(!h.hit);
    CHECK(!h.exhausted);            // ended by leaving tmax, not by running out of steps
    CHECK(h.t > 30.0F);
}

void marchPlane() {
    const auto plane = [](Vec3 p) { return sdPlane(p); };
    // Parallel to the floor: the distance stays 1, every step advances 1, tmax ends it.
    const Hit parallel = march({0, 1, 0}, {0, 0, 1}, plane, 64, 0.001F, 20.0F);
    CHECK(!parallel.hit);
    CHECK(!parallel.exhausted);
    CHECK(parallel.steps >= 20 && parallel.steps <= 22);
    // Parallel with a too small step budget: exhausted, which is what the heatmap shows as dark pixels.
    const Hit shortRun = march({0, 1, 0}, {0, 0, 1}, plane, 8, 0.001F, 20.0F);
    CHECK(!shortRun.hit && shortRun.exhausted && shortRun.steps == 8);
    // 45 degrees down from height 1: the height shrinks by a factor 1 - 1/sqrt(2) = 0.2929 per step.
    const float s = 1.0F / std::sqrt(2.0F);
    const Hit slant = march({0, 1, 0}, {0, -s, s}, plane, 64);
    CHECK(slant.hit);
    CHECK(slant.steps >= 6 && slant.steps <= 9);   // 1 * 0.2929^n < 0.001 at n = 5.6
    // With only 2 steps it cannot get there.
    CHECK(!march({0, 1, 0}, {0, -s, s}, plane, 2).hit);
}

void epsilonGrowsWithDistance() {
    // A shallow ray towards the floor creeps in: each step only closes ~10% of the remaining height. A threshold that grows
    // with the distance (about the size of a pixel there) stops it much earlier than a tiny constant one.
    const auto plane = [](Vec3 p) { return sdPlane(p); };
    const Vec3 rd = normalize({0, -0.1F, 1});
    const Hit tight = march({0, 0.3F, 0}, rd, plane, 256, 0.0001F, 100.0F, 0.0F);
    const Hit loose = march({0, 0.3F, 0}, rd, plane, 256, 0.0001F, 100.0F, 0.002F);
    CHECK(tight.hit);
    CHECK(loose.hit);
    CHECK(loose.steps < tight.steps);
}

void normalOfSphereIsRadial() {
    const Vec3 c{1, 2, 3};
    const auto sdf = [c](Vec3 p) { return sdSphere(p - c, 1.5F); };
    for (Vec3 dir : {Vec3{1, 0, 0}, Vec3{0, 1, 0}, Vec3{0, 0, -1}, Vec3{0.6F, 0.8F, 0}, Vec3{-0.36F, 0.48F, 0.8F}}) {
        const Vec3 p = c + dir * 1.5F;
        const Vec3 n = normalAt(p, sdf);
        CHECK(near(length(n), 1.0, 1e-4));
        CHECK(near(n.x, dir.x, 2e-3) && near(n.y, dir.y, 2e-3) && near(n.z, dir.z, 2e-3));
    }
    // The normal of a floor is straight up.
    const Vec3 n = normalAt({3, 0, -2}, [](Vec3 p) { return sdPlane(p); });
    CHECK(near(n.x, 0.0, 1e-4) && near(n.y, 1.0, 1e-4) && near(n.z, 0.0, 1e-4));
}

void shadowTests() {
    const auto sdf = [](Vec3 p) { return sdSphere(p - Vec3{0, 3, 0}, 1.0F); };
    // Straight up from below the sphere: blocked.
    CHECK(softShadow({0, 0, 0}, {0, 1, 0}, sdf) == 0.0F);
    // Straight up far beside it: free.
    CHECK(softShadow({6, 0, 0}, {0, 1, 0}, sdf) == 1.0F);
    // Passing close beside it: a penumbra strictly in between, and darker the closer it passes.
    const float near1 = softShadow({1.3F, 0, 0}, {0, 1, 0}, sdf);
    const float near2 = softShadow({1.8F, 0, 0}, {0, 1, 0}, sdf);
    CHECK(near1 > 0.0F && near1 < 1.0F);
    CHECK(near2 > near1);
}

void aoTests() {
    // A unit sphere resting on the floor.
    const auto sdf = [](Vec3 p) { return std::fmin(sdPlane(p), sdSphere(p - Vec3{0, 1, 0}, 1.0F)); };
    const float far = ambientOcclusion({4, 0, 0}, {0, 1, 0}, sdf);
    const float close = ambientOcclusion({0.6F, 0, 0}, {0, 1, 0}, sdf);
    CHECK(near(far, 1.0));      // open floor: every tap along the normal is exactly as far as it is high
    CHECK(close < 1.0F);        // next to the sphere the taps sit closer to it than to the floor
    CHECK(close > 0.0F);
    const float contact = ambientOcclusion({0.05F, 0, 0}, {0, 1, 0}, sdf);
    CHECK(contact < close);     // darker the closer to the contact point
}

float sceneAt(Vec3 p, float t, float k) { return scene(p, t, k); }

void sceneBlend() {
    // k = 0 is the plain minimum of the shapes, k > 0 only ever pulls the surface outward (smaller values), by at most k/2.
    std::uint32_t rng = 7;
    const auto rnd = [&rng]() {
        rng ^= rng << 13;
        rng ^= rng >> 17;
        rng ^= rng << 5;
        return static_cast<float>(rng >> 8) / 16777216.0F;
    };
    for (int i = 0; i < 4000; ++i) {
        const Vec3 p{rnd() * 6.0F - 3.0F, rnd() * 3.0F, rnd() * 6.0F - 3.0F};
        const float t = rnd() * 20.0F;
        const float hard = sceneAt(p, t, 0.0F);
        for (float k : {0.3F, 0.8F}) {
            const float soft = sceneAt(p, t, k);
            CHECK(soft <= hard + 1e-5F);
            CHECK(soft >= hard - k / 2.0F - 1e-5F);
        }
    }
}

// The reason sphere tracing is safe: a step of d never leaves the free space only if the SDF never overestimates the
// distance, i.e. it is 1-Lipschitz. The scene has to keep that promise, also with the smooth union and the rotations.
void sceneIsLipschitz() {
    std::uint32_t rng = 99;
    const auto rnd = [&rng]() {
        rng ^= rng << 13;
        rng ^= rng >> 17;
        rng ^= rng << 5;
        return static_cast<float>(rng >> 8) / 16777216.0F;
    };
    for (float k : {0.0F, 0.3F, 0.8F}) {
        for (int i = 0; i < 4000; ++i) {
            const Vec3 a{rnd() * 6.0F - 3.0F, rnd() * 3.0F, rnd() * 6.0F - 3.0F};
            const Vec3 b = a + Vec3{rnd() - 0.5F, rnd() - 0.5F, rnd() - 0.5F} * 0.4F;
            const float t = rnd() * 20.0F;
            CHECK(std::fabs(scene(a, t, k) - scene(b, t, k)) <= length(a - b) * 1.001F + 1e-5F);
        }
    }
}

void sceneMarch() {
    // At time 0 the sphere (radius 0.65) sits at x = -0.9, y = 0.8 + 0.35 * sin(1) = 1.0945, z = 0. A ray at that x and y,
    // fired along +z from z = -6, meets its front at t = 5.35 without blending. The smooth union only pulls the surface
    // outward, so with k = 0.3 the hit comes a little earlier, never later.
    const Vec3 ro{-0.9F, 1.0945F, -6};
    const Hit h = march(ro, {0, 0, 1}, [](Vec3 p) { return scene(p, 0.0F, 0.3F); }, 128);
    CHECK(h.hit);
    CHECK(h.t > 4.5F && h.t <= 5.36F);
    CHECK(march(ro, {0, 0, 1}, [](Vec3 p) { return scene(p, 0.0F, 0.0F); }, 128).t > 5.34F);
    // Looking at the sky (up, away from the floor) nothing is hit.
    const Hit sky = march({0, 1.2F, -6}, normalize({0, 1, 1}), [](Vec3 p) { return scene(p, 0.0F, 0.5F); }, 128);
    CHECK(!sky.hit);
}

void heatRamp() {
    const Vec3 paper = heat(0.0F);
    CHECK(near(paper.x, 0.98) && near(paper.y, 0.98) && near(paper.z, 0.97));
    const Vec3 mid = heat(kHeatSteps / 2.0F);                // teal
    CHECK(near(mid.x, 0.051) && near(mid.y, 0.580) && near(mid.z, 0.533));
    const Vec3 top = heat(kHeatSteps);                       // orange
    CHECK(near(top.x, 0.918) && near(top.y, 0.345) && near(top.z, 0.047));
    const Vec3 over = heat(kHeatSteps * 5.0F);               // clamped
    CHECK(near(over.x, top.x) && near(over.y, top.y) && near(over.z, top.z));
    const Vec3 neg = heat(-3.0F);
    CHECK(near(neg.x, paper.x));
    const Vec3 quarter = heat(kHeatSteps / 4.0F);            // halfway paper -> teal
    CHECK(near(quarter.x, (0.98 + 0.051) / 2.0) && near(quarter.y, (0.98 + 0.580) / 2.0));
}

}  // namespace

int main() {
    sphereDistances();
    boxDistances();
    torusDistances();
    smoothUnion();
    rotations();
    marchHitsSphere();
    marchGrazingTakesMoreSteps();
    marchMisses();
    marchPlane();
    epsilonGrowsWithDistance();
    normalOfSphereIsRadial();
    shadowTests();
    aoTests();
    sceneBlend();
    sceneIsLipschitz();
    sceneMarch();
    heatRamp();
    if (failures == 0) std::printf("all ray-marching tests passed\n");
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
