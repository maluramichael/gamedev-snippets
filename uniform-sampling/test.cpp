// Tests for the uniform-sampling logic. Plain asserts on hand-computed cases, no framework, no SDL.
// The statistical tests use a fixed seed and a fixed count, so they are deterministic; the tolerances are wide
// enough (several standard deviations) that a different libm could not flip them.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <vector>

#include "logic.hpp"

using namespace us;

namespace {

bool near(double a, double b, double eps = 1e-9) { return std::fabs(a - b) < eps; }

void rngMatchesReference() {
    // First outputs of the PCG32 reference demo (pcg32_srandom_r(&rng, 42, 54)).
    Rng rng(42, 54);
    assert(rng.next() == 0xa15c02b7U);
    assert(rng.next() == 0x7b47f409U);
    assert(rng.next() == 0xba1d3330U);
    assert(rng.next() == 0x83d2f293U);
    assert(rng.next() == 0xbfa4784bU);
    assert(rng.next() == 0xcbed606eU);
}

void rngUniformRange() {
    Rng rng(7);
    double sum = 0.0;
    for (int i = 0; i < 100000; ++i) {
        const double x = rng.uniform();
        assert(x >= 0.0 && x < 1.0);
        sum += x;
    }
    assert(near(sum / 100000.0, 0.5, 0.01));  // sd of the mean is 0.0009
}

void discMapping() {
    // A quarter of the draws must land inside half the radius (a quarter of the area), so u = 0.25 gives r = 0.5.
    const Vec2 good = discUniform(0.25, 0.0);
    assert(near(good.x, 0.5) && near(good.y, 0.0));
    // The naive version takes u as the radius itself.
    const Vec2 bad = discNaive(0.25, 0.0);
    assert(near(bad.x, 0.25) && near(bad.y, 0.0));
    // v is the angle: a quarter turn points up.
    const Vec2 up = discUniform(1.0, 0.25);
    assert(near(up.x, 0.0) && near(up.y, 1.0));
    // u = 0 is the center whatever the angle is.
    const Vec2 center = discUniform(0.0, 0.3);
    assert(near(center.x, 0.0) && near(center.y, 0.0));
}

void annulusMapping() {
    // Between r = 1 and r = 3 the area is 8 pi. Half of it is reached at r^2 = 1 + 4 = 5.
    const Vec2 mid = annulusUniform(1.0, 3.0, 0.5, 0.0);
    assert(near(mid.x, std::sqrt(5.0)) && near(mid.y, 0.0));
    // Halfway in r (r = 2) would only have 3/8 of the area behind it, which is what an interpolated radius gets wrong.
    assert(!near(mid.x, 2.0, 0.1));
    const Vec2 inner = annulusUniform(1.0, 3.0, 0.0, 0.25);
    assert(near(inner.x, 0.0) && near(inner.y, 1.0));
    const Vec2 outer = annulusUniform(1.0, 3.0, 1.0, 0.0);
    assert(near(outer.x, 3.0));
    // rMin = 0 is the plain disc, rMin = rMax is a circle.
    const Vec2 full = annulusUniform(0.0, 2.0, 0.25, 0.0);
    assert(near(full.x, 2.0 * discUniform(0.25, 0.0).x));
    const Vec2 circle = annulusUniform(2.0, 2.0, 0.7, 0.0);
    assert(near(circle.x, 2.0));
    // Statistics: every point stays in the ring and the half area radius splits the draws in half.
    Rng rng(9);
    int below = 0;
    for (int i = 0; i < 20000; ++i) {
        const Vec2 p = annulusUniform(1.0, 3.0, rng.uniform(), rng.uniform());
        const double r = std::hypot(p.x, p.y);
        assert(r >= 1.0 - 1e-12 && r <= 3.0 + 1e-12);
        if (r < std::sqrt(5.0)) ++below;
    }
    assert(std::abs(below - 10000) < 400);  // sd of the count is 71
}

void triangleWeights() {
    // s = 0.5, t = 0.25: sqrt(t) = 0.5, so a = 0.5, b = 0.5 * 0.5, c = 0.5 * 0.5.
    const Vec3 u = triangleUniform(0.5, 0.25);
    assert(near(u.x, 0.5) && near(u.y, 0.25) && near(u.z, 0.25));
    // Without the root the same draw sits much closer to the apex A: a = 1 - t.
    const Vec3 n = triangleNaive(0.5, 0.25);
    assert(near(n.x, 0.75) && near(n.y, 0.125) && near(n.z, 0.125));
    // Corners: t = 0 is A, (s = 0, t = 1) is B, (s = 1, t = 1) is C.
    const Vec3 a = triangleUniform(0.3, 0.0);
    assert(near(a.x, 1.0) && near(a.y, 0.0) && near(a.z, 0.0));
    const Vec3 b = triangleUniform(0.0, 1.0);
    assert(near(b.x, 0.0) && near(b.y, 1.0) && near(b.z, 0.0));
    const Vec3 c = triangleUniform(1.0, 1.0);
    assert(near(c.x, 0.0) && near(c.y, 0.0) && near(c.z, 1.0));
}

void triangleFoldMapping() {
    // s + t = 1.25 > 1 folds to (0.25, 0.5): a = 0.25, b = 0.25, c = 0.5.
    const Vec3 f = triangleFold(0.75, 0.5);
    assert(near(f.x, 0.25) && near(f.y, 0.25) && near(f.z, 0.5));
    // The mirror draw (0.25, 0.5) lies in the lower half already and stays as it is: the same point.
    const Vec3 g = triangleFold(0.25, 0.5);
    assert(near(g.x, 0.25) && near(g.y, 0.25) && near(g.z, 0.5));
    // s + t = 1 exactly is on the edge BC and is not folded.
    const Vec3 e = triangleFold(0.5, 0.5);
    assert(near(e.x, 0.0) && near(e.y, 0.5) && near(e.z, 0.5));
}

void triangleToPlaneMapping() {
    const Vec2 p = triangleToPlane({0, 0}, {4, 0}, {0, 4}, {0.5, 0.25, 0.25});
    assert(near(p.x, 1.0) && near(p.y, 1.0));
    // A degenerate (collinear) triangle has no area, every point lies on the line, nothing divides.
    const Vec2 d = triangleToPlane({0, 0}, {2, 2}, {4, 4}, {0.2, 0.3, 0.5});
    assert(near(d.x, d.y));
}

void sphereMapping() {
    // z = 1 - 2u is the whole trick: u = 1/3 gives z = 1/3, not the angle's cosine.
    const Vec3 good = sphereUniform(1.0 / 3.0, 0.0);
    assert(near(good.z, 1.0 / 3.0));
    assert(near(good.x, std::sqrt(1.0 - 1.0 / 9.0)) && near(good.y, 0.0));
    // The naive version takes u as the polar angle: u = 1/3 is 60 degrees, cos = 0.5.
    const Vec3 bad = sphereNaive(1.0 / 3.0, 0.0);
    assert(near(bad.z, 0.5));
    assert(near(bad.x, std::sqrt(0.75)));
    // The poles: u = 0 is north, u = 1 is south, both methods.
    assert(near(sphereUniform(0.0, 0.7).z, 1.0));
    assert(near(sphereUniform(1.0, 0.7).z, -1.0));
    assert(near(sphereNaive(0.0, 0.7).z, 1.0));
    assert(near(sphereNaive(1.0, 0.7).z, -1.0));
}

void binning() {
    assert(binIndex(0.0, 4) == 0);
    assert(binIndex(0.25, 4) == 1);  // exactly on a border belongs to the upper bin
    assert(binIndex(0.35, 10) == 3);
    assert(binIndex(0.999999, 10) == 9);
    assert(binIndex(1.0, 4) == 3);   // the outer rim is the last bin, not one past it
    assert(binIndex(1.5, 4) == 3);   // rounding can push a fraction above 1
    assert(binIndex(-0.1, 4) == 0);
    assert(binIndex(std::numeric_limits<double>::quiet_NaN(), 4) == 0);
    assert(binIndex(0.5, 0) == 0);   // no bins at all must not crash
}

void areaFractions() {
    assert(near(areaFraction(Shape::Disc, {0.5, 0.0, 0.0}), 0.25));
    assert(near(areaFraction(Shape::Disc, {0.0, 1.0, 0.0}), 1.0));
    // Triangle: the share of the area between the apex and the parallel through the point is (1 - a)^2.
    assert(near(areaFraction(Shape::Triangle, {0.5, 0.25, 0.25}), 0.25));
    assert(near(areaFraction(Shape::Triangle, {1.0, 0.0, 0.0}), 0.0));
    assert(near(areaFraction(Shape::Triangle, {-1e-15, 0.5, 0.5}), 1.0, 1e-9));  // rounding below zero is clamped
    // Sphere: Archimedes, the area below height z is (1 + z) / 2 of the total.
    assert(near(areaFraction(Shape::Sphere, {0.0, 0.0, -1.0}), 0.0));
    assert(near(areaFraction(Shape::Sphere, {1.0, 0.0, 0.0}), 0.5));
    assert(near(areaFraction(Shape::Sphere, {0.0, 0.0, 1.0}), 1.0));
}

void spreadOfBins() {
    const Spread s = spread({10, 20, 30, 40});  // total 100, expected 25 each
    assert(near(s.minRatio, 0.4) && near(s.maxRatio, 1.6));
    const Spread flat = spread({5, 5, 5});
    assert(near(flat.minRatio, 1.0) && near(flat.maxRatio, 1.0));
    const Spread none = spread({});
    assert(near(none.minRatio, 0.0) && near(none.maxRatio, 0.0));
    const Spread zeros = spread({0, 0, 0});  // no division by a zero total
    assert(near(zeros.minRatio, 0.0) && near(zeros.maxRatio, 0.0));
}

constexpr int kCount = 100000;
constexpr int kBins = 10;

void generateBasics() {
    const Cloud a = generate(Shape::Disc, Method::Uniform, 1000, 5, kBins);
    assert(a.points.size() == 1000);
    assert(a.bins.size() == static_cast<std::size_t>(kBins));
    int total = 0;
    for (int c : a.bins) total += c;
    assert(total == 1000);
    // Same seed, same cloud; another seed, another cloud.
    const Cloud b = generate(Shape::Disc, Method::Uniform, 1000, 5, kBins);
    assert(a.points[17].x == b.points[17].x && a.points[999].y == b.points[999].y);
    const Cloud c = generate(Shape::Disc, Method::Uniform, 1000, 6, kBins);
    assert(a.points[17].x != c.points[17].x);
    // Same seed and shape: naive and correct are fed the same random numbers, only the mapping differs.
    const Cloud n = generate(Shape::Disc, Method::Naive, 1000, 5, kBins);
    const double rGood = std::hypot(a.points[17].x, a.points[17].y);  // sqrt(u)
    const double rBad = std::hypot(n.points[17].x, n.points[17].y);   // u
    assert(near(rBad, rGood * rGood, 1e-9));
    // Nothing requested, nothing returned.
    const Cloud empty = generate(Shape::Disc, Method::Uniform, 0, 5, kBins);
    assert(empty.points.empty());
    const Cloud noBins = generate(Shape::Disc, Method::Uniform, 10, 5, 0);
    assert(noBins.points.size() == 10 && noBins.bins.empty());
    assert(generate(Shape::Disc, Method::Uniform, -5, 5, kBins).points.empty());
}

void expectFlat(const Cloud& cloud) {
    const double expected = static_cast<double>(kCount) / kBins;
    for (int c : cloud.bins) assert(std::fabs(c - expected) < 0.04 * expected);  // sd of a bin is 1%
}

void uniformMethodsAreFlat() {
    expectFlat(generate(Shape::Disc, Method::Uniform, kCount, 11, kBins));
    expectFlat(generate(Shape::Triangle, Method::Uniform, kCount, 11, kBins));
    expectFlat(generate(Shape::Triangle, Method::Fold, kCount, 11, kBins));
    expectFlat(generate(Shape::Sphere, Method::Uniform, kCount, 11, kBins));
}

void naiveMethodsClump() {
    // Disc and triangle: the first band has area 1/10 but gets P(u^2 < 0.1) = sqrt(0.1) = 31.6 % of the draws.
    for (Shape shape : {Shape::Disc, Shape::Triangle}) {
        const Cloud cloud = generate(shape, Method::Naive, kCount, 11, kBins);
        const Spread s = spread(cloud.bins);
        assert(near(s.maxRatio, 10.0 * std::sqrt(0.1), 0.1));
        assert(cloud.bins[0] == static_cast<int>(std::lround(s.maxRatio * kCount / kBins)));
        assert(s.minRatio < 0.6);  // the outer ring gets 1 - sqrt(0.9) = 5.1 % of the draws
        assert(near(s.minRatio, 10.0 * (1.0 - std::sqrt(0.9)), 0.1));
    }
    // Sphere: the polar cap with fraction below 0.1 holds the draws with cos^2(pi u / 2) < 0.1, which is u > 0.795,
    // so 20.5 % instead of 10 %. Both caps clump the same way.
    const Cloud sphere = generate(Shape::Sphere, Method::Naive, kCount, 11, kBins);
    const Spread s = spread(sphere.bins);
    const double capShare = 1.0 - 2.0 * std::acos(std::sqrt(0.1)) / kPi;
    assert(near(s.maxRatio, 10.0 * capShare, 0.1));
    assert(near(static_cast<double>(sphere.bins[0]) / sphere.bins[kBins - 1], 1.0, 0.1));
}

void shapesStayInside() {
    const Cloud disc = generate(Shape::Disc, Method::Uniform, 10000, 3, kBins);
    for (const Vec3& p : disc.points) assert(p.x * p.x + p.y * p.y <= 1.0 + 1e-12);
    for (Method m : {Method::Naive, Method::Uniform, Method::Fold}) {
        const Cloud tri = generate(Shape::Triangle, m, 10000, 3, kBins);
        for (const Vec3& p : tri.points) {
            assert(p.x >= -1e-12 && p.y >= -1e-12 && p.z >= -1e-12);
            assert(near(p.x + p.y + p.z, 1.0, 1e-12));
        }
    }
    const Cloud sph = generate(Shape::Sphere, Method::Uniform, 10000, 3, kBins);
    for (const Vec3& p : sph.points) assert(near(std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z), 1.0, 1e-12));
}

void means() {
    // Uniform triangle: every corner gets an average weight of 1/3. Naive: a = 1 - t averages 1/2, the cloud hangs on A.
    auto meanA = [](Method m) {
        const Cloud cloud = generate(Shape::Triangle, m, kCount, 21, kBins);
        double sum = 0.0;
        for (const Vec3& p : cloud.points) sum += p.x;
        return sum / kCount;
    };
    assert(near(meanA(Method::Uniform), 1.0 / 3.0, 0.005));
    assert(near(meanA(Method::Fold), 1.0 / 3.0, 0.005));
    assert(near(meanA(Method::Naive), 0.5, 0.005));
    // Disc: E[r^2] = 1/2 for the uniform disc, 1/3 for r = u.
    auto meanR2 = [](Method m) {
        const Cloud cloud = generate(Shape::Disc, m, kCount, 21, kBins);
        double sum = 0.0;
        for (const Vec3& p : cloud.points) sum += p.x * p.x + p.y * p.y;
        return sum / kCount;
    };
    assert(near(meanR2(Method::Uniform), 0.5, 0.005));
    assert(near(meanR2(Method::Naive), 1.0 / 3.0, 0.005));
    // Sphere: E[z^2] = 1/3 for the uniform sphere, 1/2 for z = cos(pi u).
    auto meanZ2 = [](Method m) {
        const Cloud cloud = generate(Shape::Sphere, m, kCount, 21, kBins);
        double sum = 0.0;
        for (const Vec3& p : cloud.points) sum += p.z * p.z;
        return sum / kCount;
    };
    assert(near(meanZ2(Method::Uniform), 1.0 / 3.0, 0.005));
    assert(near(meanZ2(Method::Naive), 0.5, 0.005));
}

void projection() {
    // The north pole tipped fully towards the viewer sits in the middle of the screen and faces us.
    const View top = project({0, 0, 1}, 0.0, kPi / 2.0);
    assert(near(top.x, 0.0) && near(top.y, 0.0) && near(top.depth, -1.0));
    // Without a tilt the pole is on the rim, up, and neither in front nor behind.
    const View rim = project({0, 0, 1}, 0.0, 0.0);
    assert(near(rim.x, 0.0) && near(rim.y, 1.0) && near(rim.depth, 0.0));
    // The point (0, -1, 0) looks straight at the viewer.
    const View front = project({0, -1, 0}, 0.0, 0.0);
    assert(near(front.x, 0.0) && near(front.y, 0.0) && near(front.depth, -1.0));
    // A quarter turn about the pole axis moves (1, 0, 0) to (0, 1, 0), which is the back side.
    const View back = project({1, 0, 0}, kPi / 2.0, 0.0);
    assert(near(back.x, 0.0) && near(back.y, 0.0) && near(back.depth, 1.0));
    // A turn about the pole axis never changes the height on screen when the tilt is 0.
    assert(near(project({0.3, 0.4, 0.5}, 1.234, 0.0).y, 0.5));
}

void angleWrapping() {
    assert(near(wrapAngle(0.0), 0.0));
    assert(near(wrapAngle(kTwoPi + 1.0), 1.0));
    assert(near(wrapAngle(-kPi / 2.0), 1.5 * kPi));
    assert(wrapAngle(-1e-20) < kTwoPi);  // a tiny negative angle must not round up to a full turn
    assert(wrapAngle(-1e-20) >= 0.0);
    assert(near(wrapAngle(100.0 * kTwoPi + 0.5), 0.5, 1e-9));
}

void thousands() {
    assert(withThousands(0) == "0");
    assert(withThousands(999) == "999");
    assert(withThousands(1000) == "1,000");
    assert(withThousands(20000) == "20,000");
    assert(withThousands(1234567) == "1,234,567");
    assert(withThousands(-1234) == "-1,234");
    assert(withThousands(-5) == "-5");
}

}  // namespace

int main() {
    rngMatchesReference();
    rngUniformRange();
    discMapping();
    annulusMapping();
    triangleWeights();
    triangleFoldMapping();
    triangleToPlaneMapping();
    sphereMapping();
    binning();
    areaFractions();
    spreadOfBins();
    generateBasics();
    uniformMethodsAreFlat();
    naiveMethodsClump();
    shapesStayInside();
    means();
    projection();
    angleWrapping();
    thousands();
    std::puts("uniform-sampling: all tests passed");
    return 0;
}
