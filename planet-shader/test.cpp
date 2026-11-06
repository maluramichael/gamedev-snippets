// Tests for the planet math in logic.hpp. The fragment shader in main.cpp repeats these formulas, so what is checked here
// is what the GPU draws. Expected values are worked out by hand; the hash and noise values were computed with an
// independent Python implementation (arbitrary precision integers) of the same algorithm.
#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string_view>
#include <vector>

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

using namespace gd::planet;

// Small deterministic generator for sample points, independent of the code under test.
struct Lcg {
    std::uint64_t s = 88172645463325252ULL;
    float next() {  // [0, 1)
        s = s * 6364136223846793005ULL + 1442695040888963407ULL;
        return static_cast<float>(s >> 40) / 16777216.0F;
    }
};

// ---------------------------------------------------------------------------------------------- hash and noise

void hashValues() {
    CHECK(hash32(0U) == 0U);  // xorshift-multiply keeps zero at zero, which is why the lattice is shifted by 4096
    CHECK(hash32(1U) == 1753845952U);
    CHECK(hash32(2U) == 3507691905U);
    CHECK(hash32(3U) == 1408362973U);
    CHECK(hashCell(1, 2, 3) == 2902294105U);
    CHECK(hashCell(4096, 4096, 4096) == 184345924U);
    CHECK(hashCell(4097, 4096, 4096) == 2610484970U);
    CHECK(hashCell(4097, 4096, 4096) != hashCell(4096, 4097, 4096));  // x, y and z do not commute
    CHECK(hashCell(4096, 4097, 4096) != hashCell(4096, 4096, 4097));
}

void hashAvalanche() {
    // Flipping one input bit should flip about half of the 32 output bits.
    double total = 0.0;
    for (std::uint32_t i = 0; i < 4096U; ++i) total += std::popcount(hash32(i) ^ hash32(i ^ 1U));
    const double mean = total / 4096.0;
    CHECK(mean > 14.0 && mean < 18.0);
}

void fadeCurve() {
    CHECK(near(fade(0.0F), 0.0));
    CHECK(near(fade(1.0F), 1.0));
    CHECK(near(fade(0.5F), 0.5));
    CHECK(near(fade(0.25F), 0.103515625));  // 6/1024 - 15/256 + 10/64
    for (float t = 0.0F; t <= 1.0F; t += 0.05F) CHECK(near(fade(t) + fade(1.0F - t), 1.0, 1e-5));
    CHECK(fade(0.001F) < 1e-8F);  // flat at the ends: the second derivative is zero there too, no creases at cell borders
}

void gradientTable() {
    // The 12 edge directions of Perlin's improved noise, picked by the low 4 bits of the hash.
    const float x = 0.3F, y = 0.5F, z = 0.9F;
    CHECK(near(gradient(0U, x, y, z), 0.8));    // x + y
    CHECK(near(gradient(1U, x, y, z), 0.2));    // -x + y
    CHECK(near(gradient(2U, x, y, z), -0.2));   // x - y
    CHECK(near(gradient(3U, x, y, z), -0.8));   // -x - y
    CHECK(near(gradient(4U, x, y, z), 1.2));    // x + z
    CHECK(near(gradient(5U, x, y, z), 0.6));    // -x + z
    CHECK(near(gradient(8U, x, y, z), 1.4));    // y + z
    CHECK(near(gradient(12U, x, y, z), 0.8));   // y + x
    CHECK(near(gradient(13U, x, y, z), 0.4));   // -y + z
    CHECK(near(gradient(14U, x, y, z), 0.2));   // y - x
    CHECK(near(gradient(15U, x, y, z), -1.4));  // -y - z
    CHECK(near(gradient(0x1230U + 4U, x, y, z), gradient(4U, x, y, z)));  // only the low 4 bits count
}

void noiseValues() {
    // Gradient noise is exactly zero on every lattice point, also with negative coordinates.
    CHECK(noise3({0, 0, 0}) == 0.0F);
    CHECK(near(noise3({3, -2, 5}), 0.0, 1e-7));
    CHECK(near(noise3({-7, 4, -1}), 0.0, 1e-7));
    // Worked out with the independent implementation.
    CHECK(near(noise3({0.5F, 0, 0}), 0.25, 1e-6));
    CHECK(near(noise3({0.25F, 0.75F, -0.5F}), -0.27952098846435547, 1e-5));
    CHECK(near(noise3({-1.3F, 2.6F, 0.4F}), -0.2799629063208958, 1e-5));
    CHECK(near(noise3({10.5F, -3.25F, 7.75F}), -0.0723576545715332, 1e-5));
}

float noiseReference(Vec3 p) {
    // The plain textbook version: eight separate hashCell calls and a nested lerp, no shared sub-hashes.
    const float fx = std::floor(p.x), fy = std::floor(p.y), fz = std::floor(p.z);
    const float x = p.x - fx, y = p.y - fy, z = p.z - fz;
    const int xi = static_cast<int>(fx) + kLatticeOffset, yi = static_cast<int>(fy) + kLatticeOffset, zi = static_cast<int>(fz) + kLatticeOffset;
    auto corner = [&](int dx, int dy, int dz) { return gradient(hashCell(xi + dx, yi + dy, zi + dz), x - static_cast<float>(dx), y - static_cast<float>(dy), z - static_cast<float>(dz)); };
    const float u = fade(x), v = fade(y), w = fade(z);
    auto lerp = [](float a, float b, float t) { return a + (b - a) * t; };
    return lerp(lerp(lerp(corner(0, 0, 0), corner(1, 0, 0), u), lerp(corner(0, 1, 0), corner(1, 1, 0), u), v),
                lerp(lerp(corner(0, 0, 1), corner(1, 0, 1), u), lerp(corner(0, 1, 1), corner(1, 1, 1), u), v), w);
}

void noiseMatchesReference() {
    Lcg rng;
    for (int i = 0; i < 2000; ++i) {
        const Vec3 p{rng.next() * 40.0F - 20.0F, rng.next() * 40.0F - 20.0F, rng.next() * 40.0F - 20.0F};
        CHECK(near(noise3(p), noiseReference(p), 1e-6));
    }
}

void noiseRangeAndSmoothness() {
    Lcg rng;
    float lo = 0.0F, hi = 0.0F;
    double sum = 0.0;
    constexpr int kSamples = 20000;
    for (int i = 0; i < kSamples; ++i) {
        const Vec3 p{rng.next() * 100.0F - 50.0F, rng.next() * 100.0F - 50.0F, rng.next() * 100.0F - 50.0F};
        const float n = noise3(p);
        lo = std::min(lo, n);
        hi = std::max(hi, n);
        sum += n;
        // Lipschitz: a step of 0.001 can never change the value by more than a few thousandths.
        CHECK(std::fabs(noise3(p + Vec3{0.001F, 0.0F, 0.0F}) - n) < 0.01F);
    }
    CHECK(hi <= 1.0001F && lo >= -1.0001F);
    CHECK(hi > 0.6F && lo < -0.6F);                 // it really uses the range
    CHECK(std::fabs(sum / kSamples) < 0.02);        // and averages out to zero
}

void noiseIsContinuousAcrossZero() {
    // The lattice is shifted by an offset instead of casting negative numbers, so nothing special happens at 0.
    for (const float eps : {1e-3F, 1e-4F}) {
        CHECK(std::fabs(noise3({eps, 0.3F, 0.4F}) - noise3({-eps, 0.3F, 0.4F})) < 0.01F);
        CHECK(std::fabs(noise3({0.3F, eps, 0.4F}) - noise3({0.3F, -eps, 0.4F})) < 0.01F);
        CHECK(std::fabs(noise3({0.3F, 0.4F, eps}) - noise3({0.3F, 0.4F, -eps})) < 0.01F);
    }
}

// ---------------------------------------------------------------------------------------------- fbm

void octaveStepIsARotationTimesTwo() {
    const Vec3 base = octaveStep({0, 0, 0});
    const Vec3 a = octaveStep({1, 0, 0}) - base;
    const Vec3 b = octaveStep({0, 1, 0}) - base;
    const Vec3 c = octaveStep({0, 0, 1}) - base;
    CHECK(near(length(a), 2.0) && near(length(b), 2.0) && near(length(c), 2.0));  // lacunarity 2, no stretching
    CHECK(near(dot(a, b), 0.0) && near(dot(a, c), 0.0) && near(dot(b, c), 0.0));  // the octaves are turned, not sheared
    const Vec3 p{0.3F, -1.2F, 2.0F};
    CHECK(near(length(octaveStep(p) - base), 2.0 * length(p)));
    CHECK(length(base) > 0.1F);  // the shift keeps the zero of the lattice from lining up in every octave
}

void octaveWeights() {
    CHECK(near(octaveWeight(0, 3.5F), 1.0));
    CHECK(near(octaveWeight(2, 3.5F), 1.0));
    CHECK(near(octaveWeight(3, 3.5F), 0.5));  // the last one fades in
    CHECK(near(octaveWeight(4, 3.5F), 0.0));
    CHECK(near(octaveWeight(0, -2.0F), 0.0));
}

void fbmSums() {
    const Vec3 p{0.7F, 1.9F, -0.4F};
    CHECK(near(fbm(p, 1, 8.0F), noise3(p), 1e-6));  // one octave is the plain noise
    const Vec3 p1 = octaveStep(p), p2 = octaveStep(p1), p3 = octaveStep(p2);
    const float n0 = noise3(p), n1 = noise3(p1), n2 = noise3(p2), n3 = noise3(p3);
    const double norm = 1.0 + 0.5 + 0.25 + 0.125;  // the amplitudes add up to 1.875 and the result is divided by that
    CHECK(near(fbm(p, 4, 8.0F), (n0 + 0.5 * n1 + 0.25 * n2 + 0.125 * n3) / norm, 1e-6));
    // Detail fade: lod 1 keeps only the first octave but still divides by the full sum, so the result just gets calmer.
    CHECK(near(fbm(p, 4, 1.0F), n0 / norm, 1e-6));
    CHECK(near(fbm(p, 4, 1.5F), (n0 + 0.25 * n1) / norm, 1e-6));  // octave 1 at half weight
    CHECK(fbm(p, 4, 0.0F) == 0.0F);
    CHECK(near(fbm(p, kMaxOctaves + 5, 99.0F), fbm(p, kMaxOctaves, 99.0F)));  // the cap
}

void fbmStaysInRange() {
    Lcg rng;
    for (int i = 0; i < 3000; ++i) {
        const Vec3 p{rng.next() * 20.0F - 10.0F, rng.next() * 20.0F - 10.0F, rng.next() * 20.0F - 10.0F};
        CHECK(std::fabs(fbm(p, 8, 8.0F)) <= 1.0001F);
    }
}

void lodFromPixelSize() {
    // An octave is shown while its cells are at least two pixels wide: 1 / (kBaseFreq * 2^i) >= 2 * pixel.
    const float pixel = 1.0F / (2.0F * kBaseFreq * 32.0F);  // octave 5 has exactly two-pixel cells
    CHECK(near(lodOctaves(pixel), 6.0));                     // octaves 0..5 are fully in, so the count is 6
    CHECK(near(lodOctaves(pixel * 2.0F), 5.0));              // twice the pixel size drops one octave
    CHECK(lodOctaves(1e-9F) == static_cast<float>(kMaxOctaves));
    CHECK(lodOctaves(10.0F) == 0.0F);
    CHECK(lodOctaves(0.0F) == static_cast<float>(kMaxOctaves));
}

// ---------------------------------------------------------------------------------------------- mapping

void mappingCoordinates() {
    const Vec3 s = noiseCoord({0, 1, 0}, Mapping::Sphere);
    CHECK(near(s.x, 0.0) && near(s.y, kBaseFreq) && near(s.z, 0.0));
    const Vec3 l = noiseCoord({1, 0, 0}, Mapping::LatLong);  // lon 0, lat 0
    CHECK(near(l.x, 0.0) && near(l.y, 0.0) && near(l.z, kBaseFreq * kSliceZ));
    const Vec3 north = noiseCoord({0, 1, 0}, Mapping::LatLong);  // lat = pi / 2
    CHECK(near(north.y, kBaseFreq * kPi / 2.0F));
    const Vec3 quarter = noiseCoord({0, 0, 1}, Mapping::LatLong);  // lon = pi / 2 at the equator
    CHECK(near(quarter.x, kBaseFreq * kPi / 2.0F) && near(quarter.y, 0.0));
}

void seamIsAJumpOnlyInTheNaiveMapping() {
    // Two points 0.0002 apart on the surface, on either side of the meridian at longitude +-pi.
    const Vec3 a = normalize({-1.0F, 0.0F, 0.0001F});
    const Vec3 b = normalize({-1.0F, 0.0F, -0.0001F});
    const float sphereJump = std::fabs(noiseCoord(a, Mapping::Sphere).x - noiseCoord(b, Mapping::Sphere).x);
    const float naiveJump = std::fabs(noiseCoord(a, Mapping::LatLong).x - noiseCoord(b, Mapping::LatLong).x);
    CHECK(sphereJump < 1e-3F);
    CHECK(naiveJump > 2.0F * kPi * kBaseFreq - 0.01F);  // a whole turn of longitude in noise space
    CHECK(near(noiseCoord(seamPoint(0.3F), Mapping::LatLong).x, kPi * kBaseFreq, 1e-4));  // the seam sits at lon = pi
}

void seamShowsInTheHeight() {
    // Walk along the seam on both sides and compare the heights of the neighbours: smooth on the sphere, a cliff in lat/long.
    double sphereSum = 0.0, naiveSum = 0.0;
    constexpr int kPairs = 400;
    for (int i = 0; i < kPairs; ++i) {
        const float lat = -1.2F + 2.4F * (static_cast<float>(i) + 0.5F) / kPairs;
        const Vec3 a = normalize({-std::cos(lat), std::sin(lat), 0.0002F});
        const Vec3 b = normalize({-std::cos(lat), std::sin(lat), -0.0002F});
        sphereSum += std::fabs(elevation(a, Mapping::Sphere, 8, 8.0F) - elevation(b, Mapping::Sphere, 8, 8.0F));
        naiveSum += std::fabs(elevation(a, Mapping::LatLong, 8, 8.0F) - elevation(b, Mapping::LatLong, 8, 8.0F));
    }
    CHECK(sphereSum / kPairs < 0.005);
    CHECK(naiveSum / kPairs > 0.05);
}

void poleIsPinchedInTheNaiveMapping() {
    // Right next to the pole, longitude is meaningless: the points are 0.0014 apart but a quarter turn apart in lon.
    const Vec3 a = normalize({0.0007F, 1.0F, 0.0F});
    const Vec3 b = normalize({0.0F, 1.0F, 0.0007F});
    const Vec3 ca = noiseCoord(a, Mapping::LatLong), cb = noiseCoord(b, Mapping::LatLong);
    CHECK(near(std::fabs(ca.x - cb.x), kBaseFreq * kPi / 2.0F, 1e-3));
    const Vec3 sa = noiseCoord(a, Mapping::Sphere), sb = noiseCoord(b, Mapping::Sphere);
    CHECK(length(sa - sb) < 0.01F);
}

// ---------------------------------------------------------------------------------------------- sampling and sea level

void fibonacciSphere() {
    constexpr int n = 1000;
    double sumY = 0.0;
    int north = 0;
    for (int i = 0; i < n; ++i) {
        const Vec3 p = fibonacciPoint(i, n);
        CHECK(near(length(p), 1.0, 1e-5));
        sumY += p.y;
        if (p.y > 0.0F) ++north;
    }
    CHECK(near(fibonacciPoint(0, 100).y, 0.99, 1e-6));   // 1 - 2 * 0.5 / 100
    CHECK(near(fibonacciPoint(99, 100).y, -0.99, 1e-6));
    CHECK(std::fabs(sumY / n) < 1e-3);
    CHECK(north == n / 2);
}

void quantiles() {
    std::vector<float> v{5.0F, 1.0F, 4.0F, 2.0F, 3.0F};
    CHECK(near(quantileLevel(v, 0.6F), 4.0));  // 3 of 5 values (1, 2, 3) lie below 4
    CHECK(near(quantileLevel(v, 0.0F), 1.0));  // nothing below the minimum
    const float all = quantileLevel(v, 1.0F);
    CHECK(all > 5.0F);                         // everything below
    CHECK(quantileLevel({}, 0.5F) == 0.0F);    // empty input does not crash
}

void seaLevelGivesTheWantedShare() {
    // Level from one sample set, share measured on a different one.
    for (const Mapping m : {Mapping::Sphere, Mapping::LatLong}) {
        for (const int octaves : {1, 4, 8}) {
            const float level = seaLevelFor(0.7F, m, octaves);
            int below = 0;
            constexpr int n = 5003;
            for (int i = 0; i < n; ++i) below += elevation(fibonacciPoint(i, n), m, octaves, 8.0F) < level ? 1 : 0;
            const double share = static_cast<double>(below) / n;
            CHECK(share > 0.67 && share < 0.73);
        }
    }
    CHECK(seaLevelFor(0.3F, Mapping::Sphere, 8) < seaLevelFor(0.7F, Mapping::Sphere, 8));  // more ocean, higher sea
}

void elevationDistribution() {
    for (const Mapping m : {Mapping::Sphere, Mapping::LatLong}) {
        double sum = 0.0, sq = 0.0;
        int outside = 0;
        constexpr int n = 4000;
        for (int i = 0; i < n; ++i) {
            const float e = elevation(fibonacciPoint(i, n), m, 8, 8.0F);
            sum += e;
            sq += static_cast<double>(e) * e;
            if (e < 0.0F || e > 1.0F) ++outside;
        }
        const double mean = sum / n;
        const double sd = std::sqrt(sq / n - mean * mean);
        CHECK(mean > 0.45 && mean < 0.55);
        CHECK(sd > 0.10 && sd < 0.25);
        CHECK(outside < n / 100);  // the 0..1 scale of the colour bands is not wasted
    }
}

// ---------------------------------------------------------------------------------------------- rays and camera

void sphereIntersections() {
    Hit h = raySphere({0, 0, 5}, {0, 0, -1}, 1.0F);
    CHECK(h.hit && near(h.t, 4.0) && near(h.tFar, 6.0));
    h = raySphere({0, 1, 5}, {0, 0, -1}, 1.0F);  // grazes the top
    CHECK(h.hit && near(h.t, 5.0) && near(h.tFar, 5.0));
    h = raySphere({0, 1.5F, 5}, {0, 0, -1}, 1.0F);
    CHECK(!h.hit);
    h = raySphere({0, 0, -5}, {0, 0, -1}, 1.0F);  // sphere behind the ray
    CHECK(!h.hit);
    h = raySphere({0, 0, 0}, {0, 0, -1}, 1.0F);   // origin inside: the way out is the only hit
    CHECK(h.hit && near(h.t, 1.0) && near(h.tFar, 1.0));
    h = raySphere({3, 0, 0}, {-1, 0, 0}, 2.0F);   // radius is used, not assumed to be 1
    CHECK(h.hit && near(h.t, 1.0) && near(h.tFar, 5.0));
}

void closestApproachDistance() {
    const Closest c = closestApproach({0, 1.5F, 5}, {0, 0, -1});
    CHECK(near(c.distance, 1.5) && near(c.t, 5.0));
    const Closest d = closestApproach({0, 0, 5}, {0, 0, -1});
    CHECK(near(d.distance, 0.0) && near(d.t, 5.0));
    const Closest e = closestApproach({3, 4, 0}, {1, 0, 0});  // the line runs from the origin's point of view backwards too
    CHECK(near(e.distance, 4.0) && near(e.t, -3.0));
}

void edgeCoverageRamp() {
    const float px = 0.01F;
    CHECK(near(edgeCoverage(1.0F, px), 0.5));                  // the silhouette runs through the pixel centre
    CHECK(near(edgeCoverage(1.0F - 0.5F * px, px), 1.0));      // a half pixel inside: fully covered
    CHECK(near(edgeCoverage(1.0F + 0.5F * px, px), 0.0));
    CHECK(near(edgeCoverage(1.0F - 0.25F * px, px), 0.75));
    CHECK(edgeCoverage(0.0F, px) == 1.0F);
    CHECK(edgeCoverage(3.0F, px) == 0.0F);
    CHECK(edgeCoverage(0.5F, 0.0F) == 1.0F && edgeCoverage(1.5F, 0.0F) == 0.0F);  // zero-size pixel: a hard edge, no division by zero
}

void cameraBasis() {
    Camera c = makeCamera(0.0F, 0.0F, 3.0F);
    CHECK(near(c.pos.x, 0.0) && near(c.pos.y, 0.0) && near(c.pos.z, 3.0));
    CHECK(near(c.fwd.z, -1.0) && near(c.right.x, 1.0) && near(c.up.y, 1.0));
    c = makeCamera(kPi / 2.0F, 0.0F, 3.0F);  // on the +x axis, looking back at the origin
    CHECK(near(c.pos.x, 3.0, 1e-5) && near(c.pos.z, 0.0, 1e-5));
    CHECK(near(c.fwd.x, -1.0, 1e-5));
    CHECK(near(c.right.z, -1.0, 1e-5));
    c = makeCamera(0.0F, 0.5F, 4.0F);  // looking down at the planet from above the equator
    CHECK(near(length(c.fwd), 1.0) && near(length(c.right), 1.0) && near(length(c.up), 1.0));
    CHECK(near(dot(c.fwd, c.right), 0.0) && near(dot(c.fwd, c.up), 0.0) && near(dot(c.right, c.up), 0.0));
    CHECK(near(length(c.pos), 4.0));
    CHECK(c.pos.y > 1.9F);           // above the equator plane
    CHECK(c.up.y > 0.0F);            // the camera is never upside down
}

void pitchAndZoomLimits() {
    CHECK(near(clampPitch(0.3F), 0.3));
    CHECK(near(clampPitch(5.0F), kMaxPitch));
    CHECK(near(clampPitch(-5.0F), -kMaxPitch));
    CHECK(kMaxPitch < kPi / 2.0F);   // never straight up: the basis would degenerate
    CHECK(near(clampDistance(0.1F), kMinDistance));
    CHECK(near(clampDistance(100.0F), kMaxDistance));
    CHECK(kMinDistance > 1.0F);      // the camera stays outside the planet
}

void pixelRays() {
    const Camera c = makeCamera(0.0F, 0.0F, 3.4F);
    const Vec3 mid = pixelRay(c, 480.0F, kCenterY);  // the view axis runs through the centre of the free band, not of the canvas
    CHECK(near(mid.x, 0.0) && near(mid.y, 0.0) && near(mid.z, -1.0));
    const Vec3 right = pixelRay(c, 960.0F, kCenterY);  // right edge: tan = kTanHalfFov * 16 / 9
    CHECK(near(right.x / -right.z, kTanHalfFov * 16.0F / 9.0F));
    const Vec3 top = pixelRay(c, 480.0F, 0.0F);        // 241 px above the axis, 270 px = kTanHalfFov
    CHECK(near(top.y / -top.z, kTanHalfFov * kCenterY / 270.0F));
    // Screen y points down: above the axis the ray looks up, below it looks down.
    CHECK(pixelRay(c, 480.0F, 100.0F).y > 0.0F);
    CHECK(pixelRay(c, 480.0F, 400.0F).y < 0.0F);
}

void planetSizeOnScreen() {
    // Tangent ray at distance 3.4: tan = 1 / sqrt(3.4^2 - 1) = 0.3077, in pixels 0.3077 / 0.45 * 270 = 184.6.
    const Camera c = makeCamera(0.0F, 0.0F, 3.4F);
    CHECK(raySphere(c.pos, pixelRay(c, 480.0F + 183.0F, kCenterY), 1.0F).hit);
    CHECK(!raySphere(c.pos, pixelRay(c, 480.0F + 187.0F, kCenterY), 1.0F).hit);
    const Hit centre = raySphere(c.pos, pixelRay(c, 480.0F, kCenterY), 1.0F);
    CHECK(centre.hit && near(centre.t, 2.4, 1e-4));
}

void projectionRoundTrip() {
    const Camera c = makeCamera(0.7F, 0.3F, 3.0F);
    struct Pixel {
        int x, y;
    };
    for (const Pixel px : {Pixel{480, 241}, Pixel{400, 200}, Pixel{520, 300}, Pixel{450, 270}}) {
        const Vec3 rd = pixelRay(c, static_cast<float>(px.x), static_cast<float>(px.y));
        const Hit h = raySphere(c.pos, rd, 1.0F);
        CHECK(h.hit);
        const Vec3 world = c.pos + rd * h.t;
        const Screen s = project(c, world);
        CHECK(s.inFront);
        CHECK(near(s.x, px.x, 1e-2) && near(s.y, px.y, 1e-2));
        CHECK(facesCamera(c, world));
    }
    const Screen behind = project(c, c.pos - c.fwd * 2.0F);
    CHECK(!behind.inFront);
    CHECK(facesCamera(c, c.pos * (1.0F / length(c.pos))));    // the point under the camera
    CHECK(!facesCamera(c, c.pos * (-1.0F / length(c.pos))));  // the far side
}

// ---------------------------------------------------------------------------------------------- planet frame, time

void planetFrame() {
    // A quarter turn about +y takes +x to -z (right-handed), like the camera and the rest of the series.
    const Vec3 w = toWorld({1, 0, 0}, kPi / 2.0F, 0.0F);
    CHECK(near(w.x, 0.0) && near(w.y, 0.0) && near(w.z, -1.0));
    const Vec3 l = toLocal({0, 0, -1}, kPi / 2.0F, 0.0F);
    CHECK(near(l.x, 1.0) && near(l.z, 0.0));
    // The axis does not depend on the spin and leans by the tilt: (-sin t, cos t, 0).
    for (const float spin : {0.0F, 1.0F, 4.0F}) {
        const Vec3 axis = toWorld({0, 1, 0}, spin, 0.4F);
        CHECK(near(axis.x, -std::sin(0.4F)) && near(axis.y, std::cos(0.4F)) && near(axis.z, 0.0));
    }
    Lcg rng;
    for (int i = 0; i < 200; ++i) {
        const Vec3 p = normalize({rng.next() - 0.5F, rng.next() - 0.5F, rng.next() - 0.5F});
        const float spin = rng.next() * 6.28F;
        const Vec3 back = toLocal(toWorld(p, spin, 0.41F), spin, 0.41F);
        CHECK(near(back.x, p.x, 1e-5) && near(back.y, p.y, 1e-5) && near(back.z, p.z, 1e-5));
    }
}

void angleAdvance() {
    CHECK(near(advanceAngle(0.0, 1.0, 0.5), 0.5));
    CHECK(near(advanceAngle(6.0, 1.0, 1.0), 7.0 - kTwoPi, 1e-12));  // wraps
    CHECK(near(advanceAngle(0.1, -1.0, 0.5), 0.1 - 0.5 + kTwoPi, 1e-12));
    // Ten million frames of 1/60 s at one turn per minute: the angle stays in [0, 2 pi) and agrees with the exact value.
    const double rate = kTwoPi / 60.0;
    const double dt = 1.0 / 60.0;
    double angle = 0.0;
    bool inRange = true;
    constexpr long kFrames = 10'000'000L;
    for (long i = 0; i < kFrames; ++i) {
        angle = advanceAngle(angle, rate, dt);
        if (angle < 0.0 || angle >= kTwoPi) inRange = false;
    }
    CHECK(inRange);
    const double exact = std::fmod(rate * dt * static_cast<double>(kFrames), kTwoPi);
    double diff = std::fabs(angle - exact);
    diff = std::min(diff, kTwoPi - diff);
    CHECK(diff < 1e-5);
    // The same sum kept as a growing float loses the fraction: that is why the angle is reduced on the CPU in double.
    float naive = 0.0F;
    for (long i = 0; i < 1'000'000L; ++i) naive += static_cast<float>(rate * dt);
    const double naiveExact = std::fmod(rate * dt * 1'000'000.0, kTwoPi);
    double naiveDiff = std::fabs(std::fmod(static_cast<double>(naive), kTwoPi) - naiveExact);
    naiveDiff = std::min(naiveDiff, kTwoPi - naiveDiff);
    CHECK(naiveDiff > 0.01);
}

void seamFacing() {
    for (const float yaw : {0.0F, 1.0F, -2.0F, 3.0F, 0.55F}) {
        const float spin = seamFacingSpin(yaw, 0.41F);
        const Vec3 w = toWorld(seamPoint(0.0F), spin, 0.41F);
        const Vec3 horizontal = normalize({w.x, 0.0F, w.z});
        CHECK(near(horizontal.x, std::sin(yaw), 1e-4) && near(horizontal.z, std::cos(yaw), 1e-4));
        CHECK(facesCamera(makeCamera(yaw, 0.0F, 3.4F), w));
    }
}

// ---------------------------------------------------------------------------------------------- light, atmosphere, colours

void daylight() {
    CHECK(near(dayFactor(-0.1F), 0.0));
    CHECK(near(dayFactor(0.25F), 1.0));
    CHECK(near(dayFactor(0.075F), 0.5));      // halfway between the two edges
    CHECK(near(dayFactor(0.0F), 0.198, 1e-3));  // t = 0.2857: 3t^2 - 2t^3
    CHECK(dayFactor(-1.0F) == 0.0F && dayFactor(1.0F) == 1.0F);
    for (float x = -0.5F; x < 0.5F; x += 0.01F) CHECK(dayFactor(x + 0.01F) >= dayFactor(x));
}

void atmosphereGlow() {
    CHECK(near(glowOutside(1.0F), 1.0));
    CHECK(near(glowOutside(kAtmoRadius), 0.0));
    CHECK(near(glowOutside(1.0F + 0.5F * (kAtmoRadius - 1.0F)), 0.125));  // (1 - 0.5)^3
    CHECK(glowOutside(5.0F) == 0.0F);
    CHECK(near(glowInside(0.0F), 1.0));
    CHECK(near(glowInside(0.5F), 0.125));
    CHECK(near(glowInside(1.0F), 0.0));
    // The edge of the disc: seen from outside and from inside the glow is the same, so the limb has no seam.
    CHECK(near(glowOutside(1.0F), glowOnDisc(1.0F)));
    CHECK(near(glowOnDisc(0.0F), 0.0));  // looking straight down
    for (float r = 1.0F; r < kAtmoRadius; r += 0.002F) CHECK(glowOutside(r + 0.002F) <= glowOutside(r));
    for (float r = 0.0F; r < 0.99F; r += 0.01F) CHECK(glowOnDisc(r + 0.01F) >= glowOnDisc(r));
}

void bands() {
    const float sea = 0.5F;
    const float noIce = 2.0F;  // |y| never reaches it
    CHECK(near(landHeight(0.5F, sea), 0.0));
    CHECK(near(landHeight(1.0F, sea), 1.0));
    CHECK(near(landHeight(0.75F, sea), 0.5));
    CHECK(landHeight(0.2F, sea) == 0.0F);  // below the sea there is no land height
    CHECK(bandOf(0.2F, sea, 0.0F, noIce) == Band::Deep);
    CHECK(bandOf(sea - kDeepDepth - 0.001F, sea, 0.0F, noIce) == Band::Deep);
    CHECK(bandOf(sea - kDeepDepth + 0.001F, sea, 0.0F, noIce) == Band::Shallow);
    CHECK(bandOf(sea - 0.001F, sea, 0.0F, noIce) == Band::Shallow);
    CHECK(bandOf(sea, sea, 0.0F, noIce) == Band::Beach);  // exactly at the level counts as land
    CHECK(bandOf(sea + 0.5F * kBeachTop * (1.0F - sea), sea, 0.0F, noIce) == Band::Beach);
    CHECK(bandOf(sea + 0.5F * (kLowTop + kBeachTop) * (1.0F - sea), sea, 0.0F, noIce) == Band::Lowland);
    CHECK(bandOf(sea + 0.5F * (kHighTop + kLowTop) * (1.0F - sea), sea, 0.0F, noIce) == Band::Highland);
    CHECK(bandOf(sea + 0.5F * (kRockTop + kHighTop) * (1.0F - sea), sea, 0.0F, noIce) == Band::Rock);
    CHECK(bandOf(0.999F, sea, 0.0F, noIce) == Band::Snow);
    // Polar caps override everything below them, ocean as well.
    CHECK(bandOf(0.2F, sea, 0.9F, 0.8F) == Band::Ice);
    CHECK(bandOf(0.7F, sea, -0.9F, 0.8F) == Band::Ice);  // south pole: absolute latitude
    CHECK(bandOf(0.7F, sea, 0.7F, 0.8F) != Band::Ice);
    CHECK(kBeachTop < kLowTop && kLowTop < kHighTop && kHighTop < kRockTop && kRockTop < 1.0F);
}

void presets() {
    CHECK(kPresets.size() == 4);
    for (std::size_t i = 0; i < kPresets.size(); ++i) {
        const Preset& p = kPresets[i];
        CHECK(p.name != nullptr && p.name[0] != '\0');
        for (std::size_t j = i + 1; j < kPresets.size(); ++j) CHECK(std::string_view(p.name) != std::string_view(kPresets[j].name));
        for (const Vec3& c : p.palette) CHECK(c.x >= 0.0F && c.x <= 1.0F && c.y >= 0.0F && c.y <= 1.0F && c.z >= 0.0F && c.z <= 1.0F);
        for (const Vec3 c : {p.atmosphere, p.cloudColor}) CHECK(c.x >= 0.0F && c.x <= 1.0F && c.y >= 0.0F && c.y <= 1.0F && c.z >= 0.0F && c.z <= 1.0F);
        CHECK(p.oceanShare > 0.0F && p.oceanShare < 1.0F);
        CHECK(p.iceLatitude > 0.0F && p.iceLatitude <= 1.0F);
        CHECK(p.clouds >= 0.0F && p.clouds <= 1.0F);
        CHECK(p.atmosphereStrength >= 0.0F && p.atmosphereStrength <= 1.5F);
        CHECK(p.gloss >= 0.0F && p.gloss <= 1.0F);
        CHECK(p.lava == 0.0F || p.lava == 1.0F);
        CHECK(p.cities == 0.0F || p.cities == 1.0F);
    }
    CHECK(kPresets[0].cities == 1.0F);     // Earth has city lights...
    CHECK(kPresets[3].lava == 1.0F);       // ...and the last one glows
    CHECK(kPresets[3].cities == 0.0F);
}

}  // namespace

int main() {
    hashValues();
    hashAvalanche();
    fadeCurve();
    gradientTable();
    noiseValues();
    noiseMatchesReference();
    noiseRangeAndSmoothness();
    noiseIsContinuousAcrossZero();
    octaveStepIsARotationTimesTwo();
    octaveWeights();
    fbmSums();
    fbmStaysInRange();
    lodFromPixelSize();
    mappingCoordinates();
    seamIsAJumpOnlyInTheNaiveMapping();
    seamShowsInTheHeight();
    poleIsPinchedInTheNaiveMapping();
    fibonacciSphere();
    quantiles();
    seaLevelGivesTheWantedShare();
    elevationDistribution();
    sphereIntersections();
    closestApproachDistance();
    edgeCoverageRamp();
    cameraBasis();
    pitchAndZoomLimits();
    pixelRays();
    planetSizeOnScreen();
    projectionRoundTrip();
    planetFrame();
    angleAdvance();
    seamFacing();
    daylight();
    atmosphereGlow();
    bands();
    presets();
    if (failures == 0) std::printf("all planet-shader tests passed\n");
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
