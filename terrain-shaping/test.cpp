// Tests for the terrain-shaping logic. Plain asserts on hand-computed cases first, then the properties the demo relies on.
// Run with --stats to print the noise statistics the stretch constants in logic.hpp were derived from.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "logic.hpp"

namespace {

int failures = 0;

#define CHECK(cond)                                                       \
    do {                                                                  \
        if (!(cond)) {                                                    \
            std::printf("FAILED %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            ++failures;                                                   \
        }                                                                 \
    } while (0)

bool near(double a, double b, double eps = 1e-9) { return std::fabs(a - b) <= eps; }

using namespace ts;

// fade(t) = 6t^5 - 15t^4 + 10t^3: 0 at 0, 1 at 1, exactly 1/2 in the middle, flat at both ends.
void fadeCurve() {
    CHECK(near(fade(0.0), 0.0));
    CHECK(near(fade(0.5), 0.5));
    CHECK(near(fade(1.0), 1.0));
    CHECK(near(fade(0.25), 0.103515625));  // t^3 = 1/64, t (6t - 15) + 10 = 6.625, product 0.103515625
    CHECK(fade(0.001) < 1.1e-8);           // slope 0 at the ends: 10 t^3
    CHECK(fade(0.999) > 1.0 - 1.1e-8);
    for (int i = 0; i < 100; ++i) CHECK(fade(i / 100.0) < fade((i + 1) / 100.0));
}

// Four corner dot products blended with the faded weights. All inputs by hand.
void blendCases() {
    CHECK(near(blend(1, 2, 3, 4, 0.5, 0.5), 2.5));  // lerp(lerp(1,2,.5), lerp(3,4,.5), .5) = (1.5 + 3.5) / 2
    CHECK(near(blend(1, 2, 3, 4, 0.0, 0.0), 1.0));  // on the corner: only that corner counts
    CHECK(near(blend(1, 2, 3, 4, 1.0, 0.0), 2.0));
    CHECK(near(blend(1, 2, 3, 4, 0.0, 1.0), 3.0));
    CHECK(near(blend(1, 2, 3, 4, 1.0, 1.0), 4.0));
    CHECK(near(blend(1, 2, 3, 4, 0.5, 0.0), 1.5));  // along the bottom edge only d00 and d10 matter
    CHECK(near(blend(0, 0, 0, 0, 0.3, 0.7), 0.0));
}

void smoothstepCases() {
    CHECK(near(smoothstep(0.0, 1.0, 0.0), 0.0));
    CHECK(near(smoothstep(0.0, 1.0, 0.5), 0.5));
    CHECK(near(smoothstep(0.0, 1.0, 1.0), 1.0));
    CHECK(near(smoothstep(0.3, 0.7, 0.2), 0.0));  // clamped below
    CHECK(near(smoothstep(0.3, 0.7, 0.9), 1.0));  // clamped above
    CHECK(near(smoothstep(0.3, 0.7, 0.5), 0.5));
    CHECK(near(smoothstep(0.5, 0.5, 0.4), 0.0));  // empty interval: a hard step, no division by zero
    CHECK(near(smoothstep(0.5, 0.5, 0.5), 1.0));
    CHECK(near(smoothstep(0.7, 0.3, 0.8), 1.0));  // reversed interval counts as a step at e0
    CHECK(near(smoothstep(0.7, 0.3, 0.6), 0.0));
}

// Gradient noise is exactly 0 on every lattice point (all four dot products have a zero offset vector there).
void noiseOnLattice() {
    for (int ix = -3; ix <= 3; ++ix) {
        for (int iy = -3; iy <= 3; ++iy) CHECK(near(gradientNoise(ix, iy, 7), 0.0));
    }
    CHECK(near(gradientNoise(-12.0, 40.0, 99), 0.0));
}

void noiseProperties() {
    // Deterministic, and a different seed gives a different field.
    CHECK(gradientNoise(1.37, 2.61, 5) == gradientNoise(1.37, 2.61, 5));
    int differing = 0;
    for (int i = 0; i < 50; ++i) {
        if (gradientNoise(0.37 + i * 0.5, 0.61 + i * 0.3, 1) != gradientNoise(0.37 + i * 0.5, 0.61 + i * 0.3, 2)) ++differing;
    }
    CHECK(differing > 40);

    // Never leaves +-sqrt(2)/2 and does not stay near zero. Negative coordinates take the same path.
    double lo = 1.0;
    double hi = -1.0;
    for (int i = -200; i <= 200; ++i) {
        for (int j = -200; j <= 200; ++j) {
            const double n = gradientNoise(i * 0.113 + 0.0071, j * 0.097 + 0.0043, 3);
            lo = std::min(lo, n);
            hi = std::max(hi, n);
        }
    }
    CHECK(lo >= -kNoiseMax && hi <= kNoiseMax);
    CHECK(lo < -0.4 && hi > 0.4);

    // Continuous: a tiny step gives a tiny change, also across the lattice lines and across zero.
    for (double x : {-1.0, -1e-9, 0.0, 0.5, 1.0, 2.0}) {
        CHECK(std::fabs(gradientNoise(x + 1e-7, 0.3, 4) - gradientNoise(x, 0.3, 4)) < 1e-6);
        CHECK(std::fabs(gradientNoise(0.3, x + 1e-7, 4) - gradientNoise(0.3, x, 4)) < 1e-6);
    }
}

void noiseRejectsGarbage() {
    CHECK(gradientNoise(std::nan(""), 1.0, 1) == 0.0);
    CHECK(gradientNoise(1.0, HUGE_VAL, 1) == 0.0);
    // Far outside the supported range the coordinates are clamped instead of overflowing the int cast.
    const double far = gradientNoise(1e300, -1e300, 1);
    CHECK(std::isfinite(far) && std::fabs(far) <= kNoiseMax);
}

void fbmCases() {
    // One octave at the origin: noise is 0 there, so base is 0 and the ridge 1 - |0| squared is 1.
    const Octaves o0 = sample(0.0, 0.0, 1, 1);
    CHECK(near(o0.base, 0.0));
    CHECK(near(o0.ridged, 1.0));
    // One octave away from the lattice equals the noise scaled to [-1, 1].
    const Octaves o1 = sample(0.37, 0.61, 1, 1);
    CHECK(near(o1.base, gradientNoise(0.37, 0.61, 1) / kNoiseMax));
    const double r = 1.0 - std::fabs(o1.base);
    CHECK(near(o1.ridged, r * r));
    // From two octaves on the ridged field leaves out the first octave: it is the one that shapes the big hills, and
    // ridges taken from it would sit exactly where the smooth field peaks, sinking every summit. With two octaves the
    // ridged value is that of the second octave alone, (1 - |n|)^2 with the shifted coordinates and the octave's own seed.
    const double n1 = gradientNoise(0.37 * 2.0 + 37.13, 0.61 * 2.0 + 91.71, 1U + 0x85EBCA6BU) / kNoiseMax;
    CHECK(near(sample(0.37, 0.61, 2, 1).ridged, (1.0 - std::fabs(n1)) * (1.0 - std::fabs(n1))));
    CHECK(near(sample(0.0, 0.0, 2, 1).base, gradientNoise(37.13, 91.71, 1U + 0x85EBCA6BU) / kNoiseMax * 0.5 / 1.5));  // base still counts octave 0
    // Octave count is clamped to 1..8.
    CHECK(sample(0.37, 0.61, 0, 1).base == sample(0.37, 0.61, 1, 1).base);
    CHECK(sample(0.37, 0.61, -5, 1).base == sample(0.37, 0.61, 1, 1).base);
    CHECK(sample(0.37, 0.61, 100, 1).base == sample(0.37, 0.61, 8, 1).base);
    // More octaves change the value (the detail is added), and all results stay in range.
    CHECK(sample(0.37, 0.61, 5, 1).base != sample(0.37, 0.61, 1, 1).base);
    for (int oct = 1; oct <= 8; ++oct) {
        for (int i = 0; i < 300; ++i) {
            const Octaves s = sample(i * 0.173 - 20.0, i * 0.071 + 3.0, oct, 9);
            CHECK(s.base >= -1.0 && s.base <= 1.0);
            CHECK(s.ridged >= 0.0 && s.ridged <= 1.0);
        }
    }
    // The octave shift keeps all octaves from being 0 at the same spot: the origin is no longer special.
    CHECK(std::fabs(sample(0.0, 0.0, 5, 1).base) > 1e-6);
}

void heightMapping() {
    CHECK(near(baseToHeight(0.0), 0.5));
    CHECK(near(baseToHeight(1.0), 1.0));  // clamped
    CHECK(near(baseToHeight(-1.0), 0.0));
    CHECK(near(baseToHeight(-100.0), 0.0));
    CHECK(near(baseToHeight(0.25), 0.5 + 0.5 * kStretch * 0.25));
    CHECK(near(ridgeToHeight(kRidgeLo), 0.0));
    CHECK(near(ridgeToHeight(kRidgeHi), 1.0));
    CHECK(near(ridgeToHeight(-3.0), 0.0));
    CHECK(near(ridgeToHeight(3.0), 1.0));
    CHECK(near(ridgeToHeight(0.5 * (kRidgeLo + kRidgeHi)), 0.5));
}

CurveY ys(double a, double b, double c, double d, double e) { return CurveY{a, b, c, d, e}; }

void curveNodesAndLinear() {
    const Curve lin = makeCurve(linearCurve());
    for (int i = 0; i <= 100; ++i) CHECK(near(lin(i / 100.0), i / 100.0));
    // The curve passes exactly through its nodes, whatever the (monotone) values are.
    const CurveY y = ys(0.0, 0.1, 0.15, 0.8, 1.0);
    const Curve c = makeCurve(y);
    for (int k = 0; k < kCurvePoints; ++k) CHECK(near(c(k * 0.25), y[static_cast<std::size_t>(k)]));
    // Out of range or garbage input is clamped, not extrapolated.
    CHECK(near(c(-3.0), 0.0));
    CHECK(near(c(7.0), 1.0));
    CHECK(near(c(std::nan("")), 0.0));
}

// Values 0, 0, 0, 1, 1 are flat on both sides and jump between x = 0.5 and 0.75. All four inner slopes are 0
// (a zero secant next to a non-zero one gives slope 0), so the middle piece is the Hermite S: at its midpoint exactly 1/2.
void curveHandComputed() {
    const Curve c = makeCurve(ys(0, 0, 0, 1, 1));
    CHECK(near(c(0.375), 0.0));
    CHECK(near(c(0.625), 0.5));
    CHECK(near(c(0.5), 0.0));
    CHECK(near(c(0.75), 1.0));
    // A quarter of the way into the jump: t = 0.25, 3t^2 - 2t^3 = 0.15625.
    CHECK(near(c(0.5625), 0.15625));
}

void curveStaysMonotone() {
    const std::array<CurveY, 4> shapes{{ys(0, 0, 0, 1, 1), ys(0, 0.02, 0.03, 0.04, 1), ys(0, 0.96, 0.97, 0.98, 1), ys(0.3, 0.3, 0.5, 0.5, 0.9)}};
    for (const CurveY& y : shapes) {
        const Curve c = makeCurve(y);
        double prev = c(0.0);
        for (int i = 1; i <= 2000; ++i) {
            const double v = c(i / 2000.0);
            CHECK(v >= prev - 1e-12);
            CHECK(v >= y[0] - 1e-12 && v <= y[4] + 1e-12);
            prev = v;
        }
    }
}

void curvePresets() {
    CHECK(near(valleysCurve()[2], std::pow(0.5, kValleysPower)));
    CHECK(near(valleysCurve()[0], 0.0) && near(valleysCurve()[4], 1.0));
    CHECK(near(sCurve()[2], 0.5));
    CHECK(near(sCurve()[1], 0.15625));  // smoothstep(0.25) = 3/16 - 2/64
    CHECK(near(sCurve()[3], 0.84375));
    // Valleys samples x^2: the middle node sits at 0.25, a quarter of the way at 1/16. The curve passes through its nodes.
    const Curve v = makeCurve(valleysCurve());
    CHECK(near(v(0.5), 0.25));
    CHECK(near(v(0.25), 0.0625));
    CHECK(near(v(0.75), 0.5625));
    CHECK(near(v(1.0), 1.0));
    CHECK(v(0.6) < 0.6 && v(0.9) < 0.9);  // below the straight line everywhere between the ends
}

void curveEditing() {
    CurveY y = linearCurve();
    // A node cannot cross its neighbours: the curve would fold the terrain over.
    setCurvePoint(y, 2, 0.9);
    CHECK(near(y[2], y[3]));  // clamped to the right neighbour 0.75
    setCurvePoint(y, 2, 0.0);
    CHECK(near(y[2], y[1]));  // and to the left one 0.25
    setCurvePoint(y, 2, 0.6);
    CHECK(near(y[2], 0.6));
    setCurvePoint(y, 0, -1.0);  // the end points stay inside 0..1
    CHECK(near(y[0], 0.0));
    setCurvePoint(y, 0, 0.9);
    CHECK(near(y[0], y[1]));
    setCurvePoint(y, 4, 2.0);
    CHECK(near(y[4], 1.0));
    setCurvePoint(y, 4, 0.0);
    CHECK(near(y[4], y[3]));
    setCurvePoint(y, 9, 0.5);  // an index outside the array changes nothing
    setCurvePoint(y, -1, 0.5);
    CHECK(near(y[4], y[3]));
    setCurvePoint(y, 1, std::nan(""));  // garbage changes nothing either
    CHECK(near(y[1], 0.25));
}

// Four steps, riser 0.4: each level is a flat tread for f < 0.3, the slope between f = 0.3 and 0.7, a flat tread above.
void terraceCases() {
    constexpr double riser = 0.4;  // riser from f = 0.3 to 0.7
    CHECK(near(terrace(0.0, 4, riser), 0.0));
    CHECK(near(terrace(1.0, 4, riser), 1.0));
    CHECK(near(terrace(0.30, 4, riser), 0.25));   // t = 1.2: tread
    CHECK(near(terrace(0.375, 4, riser), 0.375));  // t = 1.5: middle of the riser, still the identity
    CHECK(near(terrace(0.45, 4, riser), 0.5));     // t = 1.8: tread of the next level
    CHECK(near(terrace(0.8, 4, riser), 0.75));     // t = 3.2: tread
    // Smooth: halfway up a riser (f = 0.4) lies between its two levels, strictly.
    const double mid = terrace(0.35, 4, riser);  // t = 1.4, f = 0.4 -> smoothstep(0.3, 0.7, 0.4) = 0.15625
    CHECK(near(mid, (1.0 + 0.15625) / 4.0));
    // Naive: riser 0 is a hard step, at the middle of each level.
    CHECK(near(terrace(0.30, 4, 0.0), 0.25));
    CHECK(near(terrace(0.45, 4, 0.0), 0.5));
    CHECK(near(terrace(0.375, 4, 0.0), 0.5));
    CHECK(near(terrace(0.374, 4, 0.0), 0.25));
    CHECK(near(terrace(1.0, 4, 0.0), 1.0));
    // Step counts below 1 count as 1, inputs outside 0..1 are clamped.
    CHECK(near(terrace(0.5, 0, riser), terrace(0.5, 1, riser)));
    CHECK(near(terrace(0.5, 1, 1.0), 0.5));
    CHECK(near(terrace(-4.0, 4, riser), 0.0));
    CHECK(near(terrace(4.0, 4, riser), 1.0));
    CHECK(near(terrace(std::nan(""), 4, riser), 0.0));
    // Monotone and inside 0..1 for every step count, and the soft version has no jumps.
    for (int steps = 1; steps <= 12; ++steps) {
        double prev = terrace(0.0, steps, riser);
        for (int i = 1; i <= 3000; ++i) {
            const double v = terrace(i / 3000.0, steps, riser);
            CHECK(v >= prev - 1e-12 && v <= 1.0 + 1e-12);
            CHECK(v - prev < 0.01);  // steepest part: slope 1.5 / riser per unit of f, f moves steps / 3000 per sample, divided by steps
            prev = v;
        }
    }
}

void shapePipeline() {
    Shape s;
    s.curve = makeCurve(linearCurve());
    // Everything off: the base height passes through unchanged.
    CHECK(near(shapeHeight(s, 0.4, 0.9), 0.4));
    CHECK(near(shapeHeight(s, 0.9, 0.1), 0.9));
    // The ridge mask is 0 up to height 0.5, 1 from 0.8 and 1/2 in between.
    CHECK(near(ridgeMask(0.3), 0.0));
    CHECK(near(ridgeMask(0.5), 0.0));
    CHECK(near(ridgeMask(0.65), 0.5));
    CHECK(near(ridgeMask(0.8), 1.0));
    CHECK(near(ridgeMask(0.95), 1.0));
    // Ridged (right way): the ridge detail is added around its mean 0.5, weighted by the mask. Lowlands keep the base.
    s.ridged = true;
    CHECK(near(shapeHeight(s, 0.4, 0.9), 0.4));
    CHECK(near(shapeHeight(s, 0.5, 0.0), 0.5));                       // mask is still 0 at 0.5
    CHECK(near(shapeHeight(s, 0.8, 0.3), 0.8 + (0.3 - 0.5) * kRidgeAmount));  // 0.68: a flank between two crests sinks
    CHECK(near(shapeHeight(s, 0.8, 0.5), 0.8));                       // a ridge value of 0.5 changes nothing
    CHECK(near(shapeHeight(s, 0.65, 0.3), 0.65 + 0.5 * (0.3 - 0.5) * kRidgeAmount));  // half mask: 0.59
    CHECK(near(shapeHeight(s, 0.95, 1.0), 1.0));                      // crest on a high summit: clamped, not above 1
    CHECK(near(shapeHeight(s, 0.85, 0.0), 0.85 - 0.5 * kRidgeAmount));  // deepest crease: 0.55
    // Naive: the same detail everywhere, also in the valley.
    s.naive = true;
    CHECK(near(shapeHeight(s, 0.4, 0.9), 0.4 + 0.4 * kRidgeAmount));  // 0.64
    CHECK(near(shapeHeight(s, 0.1, 0.0), 0.0));                       // clamped at the bottom: 0.1 - 0.3
    s.ridged = false;
    CHECK(near(shapeHeight(s, 0.4, 0.9), 0.4));  // naive alone changes nothing without a feature
    // The curve is applied after the mix, terraces after the curve.
    Shape v;
    v.curve = makeCurve(valleysCurve());
    CHECK(near(shapeHeight(v, 0.5, 0.0), v.curve(0.5)));
    v.terraces = true;
    v.steps = 4;
    CHECK(near(shapeHeight(v, 0.5, 0.0), terrace(v.curve(0.5), 4, kRiser)));
    v.naive = true;
    CHECK(near(shapeHeight(v, 0.5, 0.0), terrace(v.curve(0.5), 4, 0.0)));
    // Out-of-range input is clamped before the curve (a negative base would make pow() return NaN in a naive version).
    CHECK(near(shapeHeight(v, -0.5, 0.0), terrace(v.curve(0.0), 4, 0.0)));
    CHECK(std::isfinite(shapeHeight(v, std::nan(""), 0.0)));
}

void histogramCases() {
    const std::vector<float> values{0.0F, 0.1F, 0.5F, 1.0F, 1.0F};
    const auto h = histogram(values, 4);
    CHECK(h.size() == 4);
    CHECK(h[0] == 2);  // 0.0 and 0.1 in [0, 0.25)
    CHECK(h[1] == 0);
    CHECK(h[2] == 1);  // 0.5 is the lower edge of [0.5, 0.75)
    CHECK(h[3] == 2);  // 1.0 belongs to the last bin
    const auto g = histogram(std::vector<float>{-3.0F, 9.0F, std::nanf("")}, 4);
    CHECK(g[0] == 2 && g[3] == 1);  // outside values and NaN are clamped into the outer bins
    CHECK(histogram(values, 0).size() == 1);  // at least one bin
    int total = 0;
    for (int c : histogram(values, 7)) total += c;
    CHECK(total == 5);
}

// Light from the upper left with z = 2/sqrt(6): a flat tile has that brightness.
void shading() {
    const double flat = shade(0.5, 0.5, 0.5, 0.5, 40.0);
    CHECK(near(flat, 2.0 / std::sqrt(6.0)));
    // A rise of 0.02 across the tile at scale 40 tilts the normal by 0.4. Rising to the right tilts it to the left,
    // towards the light: brighter than flat. (2.4 / (sqrt(1.16) * sqrt(6)) = 0.909)
    CHECK(near(shade(0.49, 0.51, 0.5, 0.5, 40.0), 2.4 / (std::sqrt(1.16) * std::sqrt(6.0))));
    CHECK(shade(0.49, 0.51, 0.5, 0.5, 40.0) > flat);
    CHECK(shade(0.51, 0.49, 0.5, 0.5, 40.0) < flat);  // rising to the left: faces away
    CHECK(shade(0.5, 0.5, 0.49, 0.51, 40.0) > flat);  // rising downwards (y grows downwards): faces up, towards the light
    CHECK(shade(0.5, 0.5, 0.51, 0.49, 40.0) < flat);
    for (double a : {0.0, 0.3, 1.0}) {
        for (double b : {0.0, 0.5, 1.0}) {
            const double s = shade(a, b, b, a, 400.0);
            CHECK(s >= 0.0 && s <= 1.0);
        }
    }
    CHECK(near(shade(0.1, 0.9, 0.2, 0.7, 0.0), flat));  // scale 0: always flat
}

void fieldsAndShaping() {
    Params p;
    p.octaves = 4;
    p.seed = 3;
    Fields f;
    computeFields(f, p, 16, 12);
    CHECK(f.w == 16 && f.h == 12);
    CHECK(f.base.size() == 192 && f.ridge.size() == 192);
    for (std::size_t i = 0; i < f.base.size(); ++i) {
        CHECK(f.base[i] >= 0.0F && f.base[i] <= 1.0F);
        CHECK(f.ridge[i] >= 0.0F && f.ridge[i] <= 1.0F);
    }
    // A cell's value is the sample at its centre: spot check cell (5, 7).
    const double cx = p.offX + (5 + 0.5) / 16.0 * p.span;
    const double cy = p.offY + (7 + 0.5) / 16.0 * p.span;
    CHECK(near(f.base[7 * 16 + 5], baseToHeight(sample(cx, cy, 4, 3).base), 1e-6));
    // Panning by exactly one cell moves the field by one cell.
    Params q = p;
    q.offX += p.span / 16.0;
    Fields g;
    computeFields(g, q, 16, 12);
    for (int y = 0; y < 12; ++y) {
        for (int x = 0; x + 1 < 16; ++x) CHECK(near(g.base[static_cast<std::size_t>(y * 16 + x)], f.base[static_cast<std::size_t>(y * 16 + x + 1)], 1e-5));
    }

    Shape s;
    s.curve = makeCurve(valleysCurve());
    std::vector<float> out;
    Histogram before;
    Histogram after;
    shapeFields(f, s, out, before, after);
    CHECK(out.size() == f.base.size());
    int nb = 0;
    int na = 0;
    for (int c : before) nb += c;
    for (int c : after) na += c;
    CHECK(nb == 192 && na == 192);
    for (std::size_t i = 0; i < out.size(); ++i) CHECK(near(out[i], static_cast<double>(static_cast<float>(shapeHeight(s, f.base[i], f.ridge[i]))), 1e-6));
    // The valleys curve lowers the average height.
    double sumIn = 0.0;
    double sumOut = 0.0;
    for (std::size_t i = 0; i < out.size(); ++i) {
        sumIn += f.base[i];
        sumOut += out[i];
    }
    CHECK(sumOut < sumIn);

    // Degenerate sizes do not crash and give empty fields.
    Fields e;
    computeFields(e, p, 0, 5);
    CHECK(e.base.empty() && e.ridge.empty());
}

// How much of the 0..1 range the fixed stretch constants really use.
void rangeUsage() {
    std::vector<double> base;
    std::vector<double> ridge;
    for (int i = 0; i < 400; ++i) {
        for (int j = 0; j < 400; ++j) {
            const Octaves s = sample(i * 0.093 - 17.0, j * 0.091 + 5.0, 5, 11);
            base.push_back(baseToHeight(s.base));
            ridge.push_back(ridgeToHeight(s.ridged));
        }
    }
    std::sort(base.begin(), base.end());
    std::sort(ridge.begin(), ridge.end());
    const auto pct = [](const std::vector<double>& v, double p) { return v[static_cast<std::size_t>(p * static_cast<double>(v.size() - 1))]; };
    // Fixed constants, so the picture does not jump while panning: most pixels use the range, almost none clip.
    CHECK(pct(base, 0.02) < 0.2 && pct(base, 0.98) > 0.8);
    CHECK(pct(base, 0.001) > 0.0 && pct(base, 0.999) < 1.0);
    CHECK(pct(ridge, 0.02) < 0.3 && pct(ridge, 0.98) > 0.75);
    CHECK(pct(ridge, 0.001) > 0.0 && pct(ridge, 0.999) < 1.0);
    double mb = 0.0;
    double mr = 0.0;
    for (double v : base) mb += v;
    for (double v : ridge) mr += v;
    mb /= static_cast<double>(base.size());
    mr /= static_cast<double>(ridge.size());
    CHECK(std::fabs(mb - 0.5) < 0.03);
    CHECK(std::fabs(mr - 0.5) < 0.06);  // the ridge field is centred on 0.5, so adding (ridge - 0.5) does not lift or sink the terrain
}

void printStats() {
    for (int oct : {1, 5, 8}) {
        std::vector<double> base;
        std::vector<double> ridge;
        double bmin = 1e9;
        double bmax = -1e9;
        for (int i = 0; i < 1000; ++i) {
            for (int j = 0; j < 1000; ++j) {
                const Octaves s = sample(i * 0.0379 - 17.0, j * 0.0371 + 5.0, oct, 11);
                base.push_back(s.base);
                ridge.push_back(s.ridged);
                bmin = std::min(bmin, s.base);
                bmax = std::max(bmax, s.base);
            }
        }
        std::sort(base.begin(), base.end());
        std::sort(ridge.begin(), ridge.end());
        const auto pct = [](const std::vector<double>& v, double p) { return v[static_cast<std::size_t>(p * static_cast<double>(v.size() - 1))]; };
        double mb = 0.0;
        double mr = 0.0;
        for (double v : base) mb += v;
        for (double v : ridge) mr += v;
        std::printf("octaves %d  base: min %.3f p1 %.3f p5 %.3f p50 %.3f p95 %.3f p99 %.3f max %.3f mean %.3f\n", oct, bmin, pct(base, 0.01), pct(base, 0.05), pct(base, 0.5), pct(base, 0.95), pct(base, 0.99), bmax, mb / 1e6);
        std::printf("            ridged: min %.3f p1 %.3f p5 %.3f p50 %.3f p95 %.3f p99 %.3f max %.3f mean %.3f\n", pct(ridge, 0.0), pct(ridge, 0.01), pct(ridge, 0.05), pct(ridge, 0.5), pct(ridge, 0.95), pct(ridge, 0.99), pct(ridge, 1.0), mr / 1e6);
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc > 1 && std::strcmp(argv[1], "--stats") == 0) {
        printStats();
        return 0;
    }
    fadeCurve();
    blendCases();
    smoothstepCases();
    noiseOnLattice();
    noiseProperties();
    noiseRejectsGarbage();
    fbmCases();
    heightMapping();
    curveNodesAndLinear();
    curveHandComputed();
    curveStaysMonotone();
    curvePresets();
    curveEditing();
    terraceCases();
    shapePipeline();
    histogramCases();
    shading();
    fieldsAndShaping();
    rangeUsage();
    if (failures != 0) {
        std::printf("%d check(s) failed\n", failures);
        return EXIT_FAILURE;
    }
    std::printf("terrain-shaping logic: all checks passed\n");
    return EXIT_SUCCESS;
}
