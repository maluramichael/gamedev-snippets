// Tests for the terrain-layers logic. Plain asserts on hand-computed cases, no framework, no SDL.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "logic.hpp"

using namespace tl;

namespace {

bool near(double a, double b, double eps = 1e-5) { return std::fabs(a - b) < eps; }

void smoothstepEdges() {
    assert(near(smoothstep(0.2F, 0.6F, 0.0F), 0.0));
    assert(near(smoothstep(0.2F, 0.6F, 0.2F), 0.0));
    assert(near(smoothstep(0.2F, 0.6F, 0.4F), 0.5));  // t = 0.5 -> 3/4 - 2/8 = 0.5
    assert(near(smoothstep(0.2F, 0.6F, 0.6F), 1.0));
    assert(near(smoothstep(0.2F, 0.6F, 9.0F), 1.0));
    // t = 0.25 -> 3 * 0.0625 - 2 * 0.015625 = 0.15625
    assert(near(smoothstep(0.0F, 4.0F, 1.0F), 0.15625));
    // zero width is a hard step, no division by zero
    assert(near(smoothstep(0.5F, 0.5F, 0.49F), 0.0));
    assert(near(smoothstep(0.5F, 0.5F, 0.5F), 1.0));
    assert(near(smoothstep(0.5F, 0.4F, 0.9F), 1.0));  // reversed edges act like a step as well
    assert(near(smoothstep(0.5F, 0.4F, 0.1F), 0.0));
}

void fadeCurve() {
    // 6t^5 - 15t^4 + 10t^3
    assert(near(fade(0.0F), 0.0));
    assert(near(fade(1.0F), 1.0));
    assert(near(fade(0.5F), 0.5));
    assert(near(fade(0.1F), 0.00856));
    assert(near(fade(0.9F), 1.0 - 0.00856));
    // flat at both ends: the slope of the fade is 0 there, which is what hides the grid
    assert(fade(0.01F) < 0.0001F);
    assert(fade(0.99F) > 0.9999F);
}

void gradientDirections() {
    const float k = 0.70710678F;
    // dx = 0.5, dy = 0.25 against the 8 unit directions
    assert(near(gradientDot(0, 0.5F, 0.25F), 0.5));
    assert(near(gradientDot(1, 0.5F, 0.25F), -0.5));
    assert(near(gradientDot(2, 0.5F, 0.25F), 0.25));
    assert(near(gradientDot(3, 0.5F, 0.25F), -0.25));
    assert(near(gradientDot(4, 0.5F, 0.25F), 0.75 * k));
    assert(near(gradientDot(5, 0.5F, 0.25F), 0.25 * k));
    assert(near(gradientDot(6, 0.5F, 0.25F), -0.25 * k));
    assert(near(gradientDot(7, 0.5F, 0.25F), -0.75 * k));
    // only the low three bits pick the direction
    assert(near(gradientDot(8, 0.5F, 0.25F), 0.5));
    assert(near(gradientDot(0xFFFFFFFFu, 0.5F, 0.25F), gradientDot(7, 0.5F, 0.25F)));
}

void perlinProperties() {
    // exactly zero on every lattice point, also for negative coordinates
    for (int x = -3; x <= 3; ++x) {
        for (int y = -3; y <= 3; ++y) {
            assert(near(perlin(static_cast<float>(x), static_cast<float>(y), 5), 0.0));
        }
    }
    // bounded by sqrt(2)/2 (unit gradients in 2D), so noise01 stays in [0, 1] without clamping
    float lo = 1.0F;
    float hi = -1.0F;
    double sum = 0.0;
    int n = 0;
    for (int i = 0; i < 400; ++i) {
        for (int j = 0; j < 400; ++j) {
            const float p = perlin(static_cast<float>(i) * 0.0731F - 8.0F, static_cast<float>(j) * 0.0617F - 8.0F, 11);
            lo = std::min(lo, p);
            hi = std::max(hi, p);
            sum += static_cast<double>(p);
            ++n;
        }
    }
    assert(lo >= -0.70711F && hi <= 0.70711F);
    assert(lo < -0.3F && hi > 0.3F);  // and it does use a good part of the range
    assert(std::fabs(sum / n) < 0.05);
    // deterministic, seed changes the picture
    assert(perlin(1.37F, 2.91F, 3) == perlin(1.37F, 2.91F, 3));
    int differing = 0;
    for (int i = 0; i < 50; ++i) {
        const float x = 0.37F + static_cast<float>(i) * 0.41F;
        if (std::fabs(perlin(x, 1.21F, 3) - perlin(x, 1.21F, 4)) > 1e-4F) ++differing;
    }
    assert(differing > 40);
    // smooth: a tiny step changes the value by a tiny amount (no cell seams)
    for (int i = 0; i < 2000; ++i) {
        const float x = static_cast<float>(i) * 0.0123F;
        assert(std::fabs(perlin(x + 0.001F, 0.77F, 2) - perlin(x, 0.77F, 2)) < 0.01F);
    }
    // non-finite or huge input must not reach the int cast
    assert(near(perlin(INFINITY, 1.0F, 1), 0.0));
    assert(near(perlin(NAN, 1.0F, 1), 0.0));
    (void)perlin(1e30F, -1e30F, 1);
    const float v = noise01(1.37F, 2.91F, 3);
    assert(v >= 0.0F && v <= 1.0F);
    assert(near(noise01(2.0F, 3.0F, 3), 0.5));  // lattice point: perlin 0 -> exactly the middle
}

void fbmNormalization() {
    const auto one = [](float, float, int) { return 1.0F; };
    const auto half = [](float, float, int) { return 0.5F; };
    // amplitudes are normalized: constant noise stays constant, however many octaves
    assert(near(fbm(one, 0.3F, 0.4F, 1, 2.0F, 0.5F), 1.0));
    assert(near(fbm(one, 0.3F, 0.4F, 6, 2.0F, 0.5F), 1.0));
    assert(near(fbm(half, 0.3F, 0.4F, 6, 2.0F, 0.7F), 0.5));
    // no octaves: flat middle value, not 0 and not NaN
    assert(near(fbm(one, 0.3F, 0.4F, 0, 2.0F, 0.5F), 0.5));
    assert(near(fbm(one, 0.3F, 0.4F, -2, 2.0F, 0.5F), 0.5));
    // weights: octave 0 has amplitude 1, octave 1 has 0.5 -> (1 * 1 + 0 * 0.5) / 1.5
    const auto onlyFirst = [](float, float, int o) { return o == 0 ? 1.0F : 0.0F; };
    assert(near(fbm(onlyFirst, 0.0F, 0.0F, 2, 2.0F, 0.5F), 1.0 / 1.5));
    // frequency: octave 1 sees x * lacunarity. With noise = x at x = 0.25: (0.25 + 0.5 * 0.5) / 1.5
    const auto ident = [](float x, float, int) { return x; };
    assert(near(fbm(ident, 0.25F, 0.0F, 2, 2.0F, 0.5F), 0.5 / 1.5));
    // three octaves, lacunarity 3, gain 0.5: (0.1 + 0.5 * 0.3 + 0.25 * 0.9) / 1.75
    assert(near(fbm(ident, 0.1F, 0.0F, 3, 3.0F, 0.5F), (0.1 + 0.15 + 0.225) / 1.75));
}

void ridgeShapes() {
    assert(near(ridge(0.5F, 1.0F), 1.0));  // the crest sits where the noise crosses the middle
    assert(near(ridge(0.0F, 1.0F), 0.0));
    assert(near(ridge(1.0F, 1.0F), 0.0));
    assert(near(ridge(0.25F, 1.0F), 0.5));
    assert(near(ridge(0.75F, 1.0F), 0.5));
    assert(near(ridge(0.25F, 2.0F), 0.0));  // stretch narrows the crest, the flanks fall off sooner
    assert(near(ridge(0.4F, 2.0F), 0.6));
    assert(near(ridge(0.5F, 3.0F), 1.0));
    // ridged fbm squares the ridge and normalizes like fbm
    const auto mid = [](float, float, int) { return 0.5F; };
    const auto quarter = [](float, float, int) { return 0.25F; };
    assert(near(ridged(mid, 0.0F, 0.0F, 4, 2.0F, 0.5F, 1.0F), 1.0));
    assert(near(ridged(quarter, 0.0F, 0.0F, 4, 2.0F, 0.5F, 1.0F), 0.25));  // 0.5 squared
    assert(near(ridged(quarter, 0.0F, 0.0F, 0, 2.0F, 0.5F, 1.0F), 0.0));
}

void contrastCurve() {
    assert(near(contrast(0.5F, 3.0F), 0.5));
    assert(near(contrast(0.6F, 2.0F), 0.7));
    assert(near(contrast(0.4F, 2.0F), 0.3));
    assert(near(contrast(0.9F, 4.0F), 1.0));  // clamped
    assert(near(contrast(0.1F, 4.0F), 0.0));
    assert(near(contrast(0.8F, 1.0F), 0.8));
}

void maskAndBlend() {
    // mask: 0.5 exactly at the threshold, whatever the softness
    assert(near(maskOf(0.55F, 0.55F, 0.1F), 0.5));
    assert(near(maskOf(0.45F, 0.55F, 0.1F), 0.0));
    assert(near(maskOf(0.65F, 0.55F, 0.1F), 1.0));
    assert(near(maskOf(0.549F, 0.55F, 0.0F), 0.0));  // softness 0 is a hard step
    assert(near(maskOf(0.55F, 0.55F, 0.0F), 1.0));

    // Hand-computed with base = 0.1 + 0.5 c, mountains 0.4 m, detail 0.08 (d - 0.5), threshold 0.55, softness 0.1.
    // c = 0.8 (mask 1), m = 0.5, d = 0.5 (no detail):
    assert(near(blend(Mode::Add, 0.8F, 0.5F, 0.5F, 0.55F, 0.1F), 0.1 + 0.4 + 0.2));       // 0.7
    assert(near(blend(Mode::Multiply, 0.8F, 0.5F, 0.5F, 0.55F, 0.1F), 0.5 * 1.2));         // 0.6
    assert(near(blend(Mode::Mask, 0.8F, 0.5F, 0.5F, 0.55F, 0.1F), 0.7));                   // mask 1 = same as add
    assert(near(blend(Mode::Lerp, 0.8F, 0.5F, 0.5F, 0.55F, 0.1F), 0.7));                   // mask 1 = the mountain terrain
    // c = 0.3 (mask 0): lowland. Add still raises mountains, mask and lerp do not.
    assert(near(blend(Mode::Add, 0.3F, 0.5F, 0.5F, 0.55F, 0.1F), 0.1 + 0.15 + 0.2));        // 0.45
    assert(near(blend(Mode::Multiply, 0.3F, 0.5F, 0.5F, 0.55F, 0.1F), 0.25 * 1.2));         // 0.30
    assert(near(blend(Mode::Mask, 0.3F, 0.5F, 0.5F, 0.55F, 0.1F), 0.25));                   // base only
    assert(near(blend(Mode::Lerp, 0.3F, 0.5F, 0.5F, 0.55F, 0.1F), 0.26 + 0.25 * 0.3));      // plains 0.335
    // c = threshold: mask 0.5. Mask adds half the mountain, lerp is the mean of both terrains.
    assert(near(blend(Mode::Mask, 0.55F, 1.0F, 0.5F, 0.55F, 0.1F), 0.1 + 0.275 + 0.2));     // 0.575
    assert(near(blend(Mode::Lerp, 0.55F, 1.0F, 0.5F, 0.55F, 0.1F), 0.5 * (0.26 + 0.1375) + 0.5 * (0.375 + 0.4)));
    // detail: +-0.04 at d = 1 / 0
    assert(near(blend(Mode::Add, 0.8F, 0.5F, 1.0F, 0.55F, 0.1F), 0.74));
    assert(near(blend(Mode::Add, 0.8F, 0.5F, 0.0F, 0.55F, 0.1F), 0.66));
    // the result never leaves [0, 1]
    assert(near(blend(Mode::Add, 1.0F, 1.0F, 1.0F, 0.55F, 0.1F), 1.0));       // 1.04 clamped
    assert(near(blend(Mode::Multiply, 1.0F, 1.0F, 1.0F, 0.55F, 0.1F), 1.0));  // 0.96 + 0.04
    assert(near(blend(Mode::Lerp, 0.0F, 0.0F, 0.0F, 0.55F, 0.1F), 0.22));     // 0.26 - 0.04
    assert(near(blend(Mode::Mask, 0.0F, 0.0F, 0.0F, 0.55F, 0.1F), 0.06));     // 0.1 - 0.04
}

void layerSampling() {
    Layers a;
    fillLayers(a, 32, 20, 7, 0.0F, 0.0F, 3.0F);
    assert(a.w == 32 && a.h == 20);
    assert(a.c.size() == 640 && a.m.size() == 640 && a.d.size() == 640);
    float cLo = 1.0F;
    float cHi = 0.0F;
    float mHi = 0.0F;
    for (std::size_t i = 0; i < a.c.size(); ++i) {
        assert(a.c[i] >= 0.0F && a.c[i] <= 1.0F);
        assert(a.m[i] >= 0.0F && a.m[i] <= 1.0F);
        assert(a.d[i] >= 0.0F && a.d[i] <= 1.0F);
        cLo = std::min(cLo, a.c[i]);
        cHi = std::max(cHi, a.c[i]);
        mHi = std::max(mHi, a.m[i]);
    }
    assert(cHi - cLo > 0.3F);  // the continent layer really varies
    assert(mHi > 0.3F);
    // panning by exactly one cell shifts the grid by one column
    const float cell = 3.0F / 32.0F;
    Layers b;
    fillLayers(b, 32, 20, 7, cell, 0.0F, 3.0F);
    for (int y = 0; y < 20; ++y) {
        for (int x = 0; x + 1 < 32; ++x) {
            const auto at = [&](const Layers& l, const std::vector<float>& v, int xx) { return v[static_cast<std::size_t>(y * l.w + xx)]; };
            assert(near(at(b, b.c, x), at(a, a.c, x + 1), 2e-3));
            assert(near(at(b, b.m, x), at(a, a.m, x + 1), 2e-3));
            assert(near(at(b, b.d, x), at(a, a.d, x + 1), 2e-3));
        }
    }
    // another seed is another map
    Layers c;
    fillLayers(c, 32, 20, 8, 0.0F, 0.0F, 3.0F);
    int differing = 0;
    for (std::size_t i = 0; i < a.c.size(); ++i) {
        if (std::fabs(a.c[i] - c.c[i]) > 0.01F) ++differing;
    }
    assert(differing > 300);
    // degenerate sizes give empty layers instead of garbage
    Layers e;
    fillLayers(e, 0, 20, 7, 0.0F, 0.0F, 3.0F);
    assert(e.c.empty());
    fillLayers(e, 4, -1, 7, 0.0F, 0.0F, 3.0F);
    assert(e.c.empty());
    // a zero or negative span must not divide by zero downstream
    fillLayers(e, 4, 4, 7, 0.0F, 0.0F, 0.0F);
    assert(e.c.size() == 16);
}

void panLimit() {
    assert(near(clampPan(5.0F), 5.0));
    assert(near(clampPan(kMaxPan + 10.0F), kMaxPan));
    assert(near(clampPan(-kMaxPan - 10.0F), -kMaxPan));
    assert(near(clampPan(NAN), 0.0));
}

Layers tinyLayers() {
    Layers l;
    l.w = 3;
    l.h = 2;
    l.c = {0.3F, 0.8F, 0.55F, 0.3F, 0.8F, 0.55F};
    l.m = {0.5F, 0.5F, 1.0F, 0.5F, 0.5F, 1.0F};
    l.d = {0.5F, 0.5F, 0.5F, 1.0F, 1.0F, 0.0F};
    return l;
}

void layerToggles() {
    const Layers l = tinyLayers();
    Field f;
    blendField(l, Enabled{true, true, true}, Mode::Add, 0.55F, 0.1F, f);
    assert(f.w == 3 && f.h == 2 && f.height.size() == 6 && f.mask.size() == 6 && f.cont.size() == 6);
    assert(near(f.height[0], 0.45));
    assert(near(f.height[1], 0.7));
    assert(near(f.height[3], 0.49));  // c 0.3, m 0.5, d 1.0: 0.45 + 0.04
    assert(near(f.height[5], 0.1 + 0.275 + 0.4 - 0.04));
    assert(near(f.mask[0], 0.0) && near(f.mask[1], 1.0) && near(f.mask[2], 0.5));
    assert(near(f.cont[1], 0.8));

    // continent off: flat 0.5 (the mask is then the same everywhere)
    blendField(l, Enabled{false, true, true}, Mode::Mask, 0.55F, 0.1F, f);
    for (std::size_t i = 0; i < 6; ++i) {
        assert(near(f.cont[i], 0.5));
        assert(near(f.mask[i], maskOf(0.5F, 0.55F, 0.1F)));
    }
    // mountains off: no mountain term at all
    blendField(l, Enabled{true, false, true}, Mode::Add, 0.55F, 0.1F, f);
    assert(near(f.height[1], 0.1 + 0.4));
    assert(near(f.height[2], 0.1 + 0.275));
    // detail off: d counts as 0.5, no offset
    blendField(l, Enabled{true, true, false}, Mode::Add, 0.55F, 0.1F, f);
    assert(near(f.height[3], 0.45));
    assert(near(f.height[5], 0.1 + 0.275 + 0.4));
    // all off: the same everywhere
    blendField(l, Enabled{false, false, false}, Mode::Add, 0.55F, 0.1F, f);
    for (std::size_t i = 0; i < 6; ++i) assert(near(f.height[i], 0.35));
    // mismatched input is refused instead of read out of bounds
    Layers bad = l;
    bad.m.pop_back();
    blendField(bad, Enabled{}, Mode::Add, 0.55F, 0.1F, f);
    assert(f.height.empty() && f.w == 0);
}

Field rampField() {
    // one row of land that rises by 0.02 per cell on the left and 0.06 per cell on the right, rows above and below are flat copies
    Field f;
    f.w = 6;
    f.h = 3;
    const float row[6] = {0.40F, 0.42F, 0.44F, 0.50F, 0.56F, 0.62F};
    const float mask[6] = {0.0F, 0.0F, 0.0F, 1.0F, 1.0F, 1.0F};
    for (int y = 0; y < 3; ++y) {
        for (int x = 0; x < 6; ++x) {
            f.height.push_back(row[x]);
            f.mask.push_back(mask[x]);
            f.cont.push_back(0.5F);
        }
    }
    return f;
}

void slopeAndRelief() {
    const Field f = rampField();
    // central differences: (h[x+1] - h[x-1]) / 2
    assert(near(slopeAt(f, 1, 1), 0.02));
    assert(near(slopeAt(f, 2, 1), 0.04));
    assert(near(slopeAt(f, 3, 1), 0.06));
    assert(near(slopeAt(f, 4, 1), 0.06));
    // edges use the cell itself instead of reading outside
    assert(near(slopeAt(f, 0, 1), 0.01));  // (0.42 - 0.40) / 2
    assert(near(slopeAt(f, 5, 0), 0.03));  // (0.62 - 0.56) / 2

    // interior cells x = 1..4 on row y = 1: mask off at x = 1, 2, on at x = 3, 4
    Relief r = reliefStats(f);
    assert(r.nLow == 2 && r.nHigh == 2);
    assert(near(r.lowland, 0.03));
    assert(near(r.highland, 0.06));
    assert(near(r.ratio(), 2.0));

    // the transition band (mask between 0.05 and 0.95) counts for neither side
    Field t = f;
    t.mask[static_cast<std::size_t>(1 * 6 + 2)] = 0.5F;
    t.mask[static_cast<std::size_t>(1 * 6 + 3)] = 0.96F;  // just on the highland side
    t.mask[static_cast<std::size_t>(1 * 6 + 4)] = 0.94F;  // just inside the band
    r = reliefStats(t);
    assert(r.nLow == 1 && r.nHigh == 1);
    assert(near(r.lowland, 0.02) && near(r.highland, 0.06));

    // water is not land: sink cell x = 2 below the sea, it leaves the statistics
    Field w = f;
    w.height[static_cast<std::size_t>(1 * 6 + 2)] = 0.1F;
    r = reliefStats(w);
    assert(r.nLow == 1);
    assert(near(r.lowland, 0.15));  // cell x = 1 now sees 0.40 on the left and 0.10 on the right: |0.10 - 0.40| / 2

    // everything on one side: the ratio is unknown (0), not NaN or inf
    Field allOn = f;
    for (float& m : allOn.mask) m = 1.0F;
    r = reliefStats(allOn);
    assert(r.nLow == 0 && r.nHigh == 4);
    assert(near(r.ratio(), 0.0));
    Field allOff = f;
    for (float& m : allOff.mask) m = 0.0F;
    r = reliefStats(allOff);
    assert(r.nHigh == 0 && r.nLow == 4);
    assert(near(r.ratio(), 0.0));
    // a perfectly flat lowland would divide by zero: the ratio stays 0
    Field flat = f;
    for (float& v : flat.height) v = 0.4F;
    r = reliefStats(flat);
    assert(r.nLow == 2 && r.nHigh == 2);
    assert(near(r.lowland, 0.0));
    assert(near(r.ratio(), 0.0));
    // too small for an interior
    Field tiny;
    tiny.w = 2;
    tiny.h = 2;
    tiny.height.assign(4, 0.5F);
    tiny.mask.assign(4, 0.0F);
    tiny.cont.assign(4, 0.5F);
    r = reliefStats(tiny);
    assert(r.nLow == 0 && r.nHigh == 0);
    // data that does not match the size is refused
    Field bad = f;
    bad.mask.pop_back();
    r = reliefStats(bad);
    assert(r.nLow == 0 && r.nHigh == 0);
}

void shading() {
    // light from the upper left; flat ground is the reference 1.0
    assert(near(shadeFromSlope(0.0F, 0.0F), 1.0));
    // rising to the east: the surface faces west, towards the light: n = (-1, 0, 1) / sqrt 2
    // (0.5 + 0.70711) / 1.41421 / 0.70711 = 1.20711
    assert(near(shadeFromSlope(1.0F, 0.0F), 1.20711, 1e-4));
    // rising to the west: n = (1, 0, 1) / sqrt 2 -> (-0.5 + 0.70711) / 1.41421 / 0.70711 = 0.20711
    assert(near(shadeFromSlope(-1.0F, 0.0F), 0.20711, 1e-4));
    // north and south behave the same way (screen y points down, light comes from the top)
    assert(near(shadeFromSlope(0.0F, 1.0F), 1.20711, 1e-4));
    assert(near(shadeFromSlope(0.0F, -1.0F), 0.20711, 1e-4));

    // on the field: a plain ramp rising east is brighter than flat, rising west is darker
    const Field f = rampField();
    assert(shadeAt(f, 2, 1, 10.0F) > 1.0F);
    Field flat = f;
    for (float& v : flat.height) v = 0.5F;
    assert(near(shadeAt(flat, 2, 1, 10.0F), 1.0));
    Field mirrored = f;
    for (int y = 0; y < 3; ++y) {
        for (int x = 0; x < 6; ++x) mirrored.height[static_cast<std::size_t>(y * 6 + x)] = f.height[static_cast<std::size_t>(y * 6 + 5 - x)];
    }
    assert(shadeAt(mirrored, 3, 1, 10.0F) < 1.0F);
}

void colors() {
    const Rgb deep = terrainColor(0.0F);
    assert(deep.b > deep.r && deep.b > 100);
    const Rgb shore = terrainColor(kSea - 0.001F);
    assert(shore.b > shore.r);  // still water
    const Rgb sand = terrainColor(kSea);
    assert(sand.r > sand.b);  // land begins with sand, a visible step at the coast
    const Rgb snow = terrainColor(1.0F);
    assert(snow.r >= 245 && snow.g >= 245 && snow.b >= 245);
    const Rgb grass = terrainColor(0.45F);
    assert(grass.g > grass.r && grass.g > grass.b);
    // out of range heights clamp to the ends
    const Rgb below = terrainColor(-3.0F);
    assert(below.r == deep.r && below.g == deep.g && below.b == deep.b);
    const Rgb above = terrainColor(9.0F);
    assert(above.r == snow.r && above.g == snow.g && above.b == snow.b);

    const Rgb calm = reliefColor(0.0F);
    const Rgb steep = reliefColor(1.0F);
    assert(calm.r == 231 && calm.g == 229 && calm.b == 228);
    assert(steep.r == 234 && steep.g == 88 && steep.b == 12);
    const Rgb halfway = reliefColor(0.5F * kReliefFull);
    assert(halfway.r > 225 && halfway.r < 240 && halfway.g > 150 && halfway.g < 160);  // (229 + 88) / 2 = 158

    const Rgb g = heightColor(0.8F);
    assert(g.r == g.g && g.g == g.b);
    assert(g.r == 204);
    const Rgb sea = heightColor(0.1F);
    assert(sea.b > sea.r);
}

void downsampling() {
    const std::vector<float> src = {1, 3, 5, 7,   //
                                    1, 3, 5, 7};
    const Grid g = downsample(src, 4, 2, 2);
    assert(g.w == 2 && g.h == 1);
    assert(near(g.v[0], 2.0) && near(g.v[1], 6.0));
    const Grid same = downsample(src, 4, 2, 1);
    assert(same.w == 4 && same.h == 2 && near(same.v[3], 7.0));
    // sizes that do not divide are cut off, factor 0 and bad sizes give an empty grid
    const Grid cut = downsample(src, 4, 2, 3);
    assert(cut.w == 1 && cut.h == 0 && cut.v.empty());
    assert(downsample(src, 4, 2, 0).v.empty());
    assert(downsample(src, 5, 2, 1).v.empty());  // size does not match the data
}

void sliders() {
    assert(near(sliderValue(100.0F, 100.0F, 300.0F, 0.0F, 1.0F), 0.0));
    assert(near(sliderValue(300.0F, 100.0F, 300.0F, 0.0F, 1.0F), 1.0));
    assert(near(sliderValue(200.0F, 100.0F, 300.0F, 0.0F, 1.0F), 0.5));
    assert(near(sliderValue(150.0F, 100.0F, 300.0F, 0.2F, 0.6F), 0.3));
    assert(near(sliderValue(0.0F, 100.0F, 300.0F, 0.2F, 0.6F), 0.2));    // left of the track clamps
    assert(near(sliderValue(999.0F, 100.0F, 300.0F, 0.2F, 0.6F), 0.6));  // right of the track clamps
    assert(near(sliderValue(5.0F, 100.0F, 100.0F, 0.2F, 0.6F), 0.2));    // zero width track
    assert(near(sliderPos(0.3F, 100.0F, 300.0F, 0.2F, 0.6F), 150.0));
    assert(near(sliderPos(9.0F, 100.0F, 300.0F, 0.2F, 0.6F), 300.0));
    assert(near(sliderPos(0.3F, 100.0F, 300.0F, 0.5F, 0.5F), 100.0));    // empty range
}

// The point of the demo, with real noise: relief on the lowlands versus the highlands. Adding gives both about the same
// relief, the mask takes the mountains away from the lowlands.
void maskConcentratesRelief() {
    // the demo's own grid and defaults
    Layers layers;
    fillLayers(layers, 288, 192, kDefaultSeed, 0.0F, 0.0F, kViewSpan);
    Field add;
    Field masked;
    blendField(layers, Enabled{}, Mode::Add, kDefaultThreshold, kDefaultSoftness, add);
    blendField(layers, Enabled{}, Mode::Mask, kDefaultThreshold, kDefaultSoftness, masked);
    const Relief ra = reliefStats(add);
    const Relief rm = reliefStats(masked);
    std::printf("add:  lowland %.4f (%d)  highland %.4f (%d)  ratio %.2f\n", ra.lowland, ra.nLow, ra.highland, ra.nHigh, ra.ratio());
    std::printf("mask: lowland %.4f (%d)  highland %.4f (%d)  ratio %.2f\n", rm.lowland, rm.nLow, rm.highland, rm.nHigh, rm.ratio());
    assert(ra.nLow > 500 && ra.nHigh > 500 && rm.nLow > 500 && rm.nHigh > 500);
    assert(ra.ratio() < 1.6);
    assert(rm.ratio() > 2.0);
    assert(rm.ratio() > ra.ratio() * 1.5);
    assert(rm.lowland < ra.lowland);
}

}  // namespace

int main() {
    smoothstepEdges();
    fadeCurve();
    gradientDirections();
    perlinProperties();
    fbmNormalization();
    ridgeShapes();
    contrastCurve();
    maskAndBlend();
    layerSampling();
    panLimit();
    layerToggles();
    slopeAndRelief();
    shading();
    colors();
    downsampling();
    sliders();
    maskConcentratesRelief();
    std::puts("terrain-layers logic: all asserts passed");
    return 0;
}
