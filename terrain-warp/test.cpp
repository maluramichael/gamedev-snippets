// Tests for the domain warping logic. Plain asserts on hand-computed cases, no framework, no SDL.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdio>

#include "logic.hpp"

using namespace warp;

namespace {

bool near(double a, double b, double eps = 1e-9) { return std::fabs(a - b) < eps; }

void fadeCurve() {
    assert(fade(0.0) == 0.0);
    assert(fade(1.0) == 1.0);
    assert(near(fade(0.5), 0.5));
    // 6t^5 - 15t^4 + 10t^3 at t = 1/4: 6/1024 - 15/256 + 10/64 = 6.625 / 64
    assert(near(fade(0.25), 0.103515625));
    assert(near(fade(0.25) + fade(0.75), 1.0));  // symmetric around the middle
    double last = -1.0;
    for (int i = 0; i <= 100; ++i) {
        const double v = fade(i / 100.0);
        assert(v > last);  // strictly increasing, so no cell border can fold the lookup back
        last = v;
    }
}

void cellSplit() {
    const Cell a = cellOf(2.75);
    assert(a.index == 2 && near(a.frac, 0.75));
    const Cell b = cellOf(-0.25);  // floor, not truncation: -1 and 0.75
    assert(b.index == -1 && near(b.frac, 0.75));
    const Cell c = cellOf(-1.0);
    assert(c.index == -1 && c.frac == 0.0);
    const Cell d = cellOf(0.0);
    assert(d.index == 0 && d.frac == 0.0);
}

void cornerBlending() {
    const Vec2 ex{1.0, 0.0};
    const Vec2 ey{0.0, 1.0};
    const Vec2 zero{0.0, 0.0};
    // All four gradients point in +x: the corner values are fx, fx-1, fx, fx-1, so the blend is fx - fade(fx).
    assert(near(cornerBlend(ex, ex, ex, ex, 0.25, 0.6), 0.25 - 0.103515625));
    assert(near(cornerBlend(ex, ex, ex, ex, 0.5, 0.1), 0.0));
    assert(near(cornerBlend(ex, ex, ex, ex, 0.0, 0.7), 0.0));  // left edge of the cell
    assert(near(cornerBlend(ex, ex, ex, ex, 1.0, 0.7), 0.0));  // right edge, the next cell's left edge
    // Same with +y gradients, mirrored: fy - fade(fy), independent of fx.
    assert(near(cornerBlend(ey, ey, ey, ey, 0.9, 0.25), 0.25 - 0.103515625));
    // Only the top left corner has a gradient (not a unit vector, the blend does not care): at the middle
    // n00 = 1*0.5, the rest is 0, top = 0.25, bottom = 0, result = 0.125.
    assert(near(cornerBlend(ex, zero, zero, zero, 0.5, 0.5), 0.125));
    // Opposite gradients on the left and right: positive in the middle on the left half is negative on the right.
    assert(cornerBlend(ex, ex, ex, ex, 0.25, 0.5) > 0.0);
    assert(cornerBlend(ex, ex, ex, ex, 0.75, 0.5) < 0.0);
}

void noiseProperties() {
    for (int seed = 0; seed < 3; ++seed) {
        for (int ix = -3; ix <= 3; ++ix) {
            for (int iy = -3; iy <= 3; ++iy) {
                assert(noise(ix, iy, static_cast<std::uint32_t>(seed)) == 0.0);  // gradient noise is 0 on every lattice point
            }
        }
    }
    assert(noise(0.4, 0.7, 1) == noise(0.4, 0.7, 1));  // deterministic
    int differing = 0;
    double maxAbs = 0.0;
    for (int i = 0; i < 200; ++i) {
        for (int j = 0; j < 200; ++j) {
            const double x = -5.0 + i * 0.0517;
            const double y = -5.0 + j * 0.0493;
            const double a = noise(x, y, 1);
            maxAbs = std::fmax(maxAbs, std::fabs(a));
            assert(std::isfinite(a) && std::fabs(a) <= 1.0 + 1e-9);
            if (a != noise(x, y, 2)) ++differing;
        }
    }
    assert(differing > 30000);  // another seed gives another landscape
    assert(maxAbs > 0.6);       // and the range is actually used, not just bounded
    // Continuous: a step of 1e-4 may not change the value by more than 2e-3 (slope below 20), also across cell borders.
    for (int i = 0; i < 2000; ++i) {
        const double x = -4.0 + i * 0.0041;
        assert(std::fabs(noise(x + 1e-4, 0.37, 1) - noise(x, 0.37, 1)) < 2e-3);
    }
}

void fbmSum() {
    // One octave is the noise itself, two octaves are (n0 + n1 / 2) / (1 + 1/2).
    assert(near(fbm(0.3, 0.8, 5, 1), octaveNoise(0.3, 0.8, 5, 0)));
    const double two = (octaveNoise(0.3, 0.8, 5, 0) + 0.5 * octaveNoise(0.3, 0.8, 5, 1)) / 1.5;
    assert(near(fbm(0.3, 0.8, 5, 2), two));
    assert(fbm(0.3, 0.8, 5, 0) == 0.0);  // no octaves, no noise (and no division by zero)
    assert(fbm(0.3, 0.8, 5, -4) == 0.0);
    for (int i = 0; i < 100; ++i) {
        for (int j = 0; j < 100; ++j) {
            assert(std::fabs(fbm(i * 0.173, j * 0.211, 9, 6)) <= 1.0 + 1e-9);  // the normalization keeps it in [-1, 1]
        }
    }
}

void heightMapping() {
    assert(near(toHeight(0.0), 0.5));
    assert(toHeight(-1.0) == 0.0);
    assert(toHeight(1.0) == 1.0);
    assert(toHeight(5.0) == 1.0);
    assert(toHeight(-5.0) == 0.0);
    assert(toHeight(0.1) > toHeight(0.05));
}

Params stage(int stages, double strength) {
    Params p;
    p.stages = stages;
    p.strength = strength;
    return p;
}

void warpPoints() {
    const Vec2 p{1.37, 0.82};
    // No stage or no strength leaves the point alone, exactly.
    for (int stages = 0; stages <= 2; ++stages) {
        const Vec2 w = warpPoint(p, stage(stages, 0.0));
        assert(w.x == p.x && w.y == p.y);
    }
    const Vec2 w0 = warpPoint(p, stage(0, 3.0));
    assert(w0.x == p.x && w0.y == p.y);

    // One stage: p + s * q(p), with q = (fbm(p + A), fbm(p + B)).
    const Params one = stage(1, 1.5);
    const Vec2 w1 = warpPoint(p, one);
    const double qx = fbm(p.x + kOffsetA.x, p.y + kOffsetA.y, one.seed, one.warpOctaves);
    const double qy = fbm(p.x + kOffsetB.x, p.y + kOffsetB.y, one.seed, one.warpOctaves);
    assert(near(w1.x, p.x + 1.5 * qx) && near(w1.y, p.y + 1.5 * qy));

    // Two stages: p + s * r, with r = (fbm(p + s q + C), fbm(p + s q + D)).
    const Params two = stage(2, 1.5);
    const Vec2 w2 = warpPoint(p, two);
    const double sx = p.x + 1.5 * qx;
    const double sy = p.y + 1.5 * qy;
    const double rx = fbm(sx + kOffsetC.x, sy + kOffsetC.y, two.seed, two.warpOctaves);
    const double ry = fbm(sx + kOffsetD.x, sy + kOffsetD.y, two.seed, two.warpOctaves);
    assert(near(w2.x, p.x + 1.5 * rx) && near(w2.y, p.y + 1.5 * ry));
    assert(std::fabs(w2.x - w1.x) + std::fabs(w2.y - w1.y) > 1e-6);  // the second stage really changes something

    // Because |fbm| <= 1, a point moves at most `strength` along each axis, in both stages.
    for (int stages = 1; stages <= 2; ++stages) {
        for (int i = 0; i < 60; ++i) {
            for (int j = 0; j < 60; ++j) {
                const Vec2 q{i * 0.29 - 3.0, j * 0.31 - 3.0};
                const Vec2 w = warpPoint(q, stage(stages, 2.5));
                assert(std::fabs(w.x - q.x) <= 2.5 + 1e-9 && std::fabs(w.y - q.y) <= 2.5 + 1e-9);
            }
        }
    }
}

void heights() {
    const Vec2 p{0.9, 1.4};
    // Without strength the warped height is the plain height.
    for (int stages = 0; stages <= 2; ++stages) {
        assert(height(p, stage(stages, 0.0)) == height(p, stage(0, 0.0)));
    }
    assert(height(p, stage(2, 1.5)) != height(p, stage(0, 1.5)));
    for (int i = 0; i < 80; ++i) {
        for (int j = 0; j < 80; ++j) {
            const double h = height({i * 0.11, j * 0.13}, stage(2, 3.0));
            assert(h >= 0.0 && h <= 1.0);
        }
    }
    Params other = stage(2, 1.5);
    other.seed = 99;
    assert(height(p, other) != height(p, stage(2, 1.5)));
}

void foldDetection() {
    constexpr double kTwoPi = 6.283185307179586;
    const auto identity = [](Vec2 p) { return p; };
    const auto mirror = [](Vec2 p) { return Vec2{-p.x, p.y}; };
    const auto collapse = [](Vec2 p) { return Vec2{0.0 * p.x, p.y}; };
    const auto scale = [](Vec2 p) { return Vec2{2.0 * p.x, 2.0 * p.y}; };
    assert(foldedFraction(identity, 0, 0, 3, 3, 20, 20) == 0.0);
    assert(foldedFraction(scale, 0, 0, 3, 3, 20, 20) == 0.0);
    assert(foldedFraction(mirror, 0, 0, 3, 3, 20, 20) == 1.0);    // determinant -1 everywhere
    assert(foldedFraction(collapse, 0, 0, 3, 3, 20, 20) == 1.0);  // determinant 0 counts: the map squashes the area to a line

    // x' = x + s sin(x) has determinant 1 + s cos(x). For s = 2 that is <= 0 where cos(x) <= -1/2, one third of a period.
    const auto sine2 = [](Vec2 p) { return Vec2{p.x + 2.0 * std::sin(p.x), p.y}; };
    assert(near(foldedFraction(sine2, 0.0, 0.0, kTwoPi, 1.0, 600, 4), 1.0 / 3.0, 0.01));
    // For s = 0.5 the determinant stays at 0.5 or more: never folded.
    const auto sine05 = [](Vec2 p) { return Vec2{p.x + 0.5 * std::sin(p.x), p.y}; };
    assert(foldedFraction(sine05, 0.0, 0.0, kTwoPi, 1.0, 600, 4) == 0.0);
    // s = 1 only touches 0 at x = pi: with sample centers that never sit exactly on it, nothing is folded.
    const auto sine1 = [](Vec2 p) { return Vec2{p.x + std::sin(p.x), p.y}; };
    assert(foldedFraction(sine1, 0.0, 0.0, kTwoPi, 1.0, 600, 4) < 0.01);
    // No samples, no fraction (and no division by zero).
    assert(foldedFraction(mirror, 0, 0, 3, 3, 0, 0) == 0.0);
}

void foldGrowsWithStrength() {
    const auto fraction = [](int stages, double strength) {
        const Params p = stage(stages, strength);
        return foldedFraction([&p](Vec2 q) { return warpPoint(q, p); }, 0.0, 0.0, 4.8, 3.0, 64, 40);
    };
    assert(fraction(2, 0.0) == 0.0);
    assert(fraction(0, 5.0) == 0.0);  // no stage, no warp, whatever the strength
    const double mild = fraction(2, 0.3);
    const double strong = fraction(2, 6.0);
    assert(mild < strong);
    assert(strong > 0.1);  // a strong warp overlaps itself over a large part of the map
}

}  // namespace

int main() {
    fadeCurve();
    cellSplit();
    cornerBlending();
    noiseProperties();
    fbmSum();
    heightMapping();
    warpPoints();
    heights();
    foldDetection();
    foldGrowsWithStrength();
    std::puts("terrain-warp logic: all tests passed");
    return 0;
}
