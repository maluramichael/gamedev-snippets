// Tests for the gamma / linear light logic. Plain asserts on hand-computed cases, no framework, no SDL.
// Reference values (0.21404114, 0.73535698, ...) come from the sRGB formula worked out independently of logic.hpp.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>

#include "logic.hpp"

using namespace gam;

namespace {

bool near(double a, double b, double eps = 1e-7) { return std::fabs(a - b) < eps; }
bool nearRgb(Rgb a, Rgb b, double eps = 1e-7) { return near(a.r, b.r, eps) && near(a.g, b.g, eps) && near(a.b, b.b, eps); }

constexpr Rgb kBlack{0, 0, 0};
constexpr Rgb kWhite{1, 1, 1};
constexpr Rgb kRed{1, 0, 0};
constexpr Rgb kGreen{0, 1, 0};
const Rgb kTeal{13.0 / 255.0, 148.0 / 255.0, 136.0 / 255.0};
const Rgb kOrange{234.0 / 255.0, 88.0 / 255.0, 12.0 / 255.0};

void curveEndpoints() {
    assert(near(decode(0.0), 0.0, 1e-12));
    assert(near(decode(1.0), 1.0, 1e-12));
    assert(near(encode(0.0), 0.0, 1e-12));
    assert(near(encode(1.0), 1.0, 1e-12));
}

void curveClampsItsInput() {
    assert(decode(-0.5) == 0.0);
    assert(decode(2.0) == 1.0);
    assert(encode(-1.0) == 0.0);
    assert(encode(5.0) == 1.0);
}

void knownValues() {
    // ((0.5 + 0.055) / 1.055) ^ 2.4 = 0.21404114: half the signal carries about a fifth of the light.
    assert(near(decode(0.5), 0.21404114));
    assert(near(decode(128.0 / 255.0), 0.21586050));
    // The other way round: half the light needs the code value 0.73535698, which is 188 of 255 (187.516 rounded).
    assert(near(encode(0.5), 0.73535698));
    assert(toByte(encode(0.5)) == 188);
    // Quarter and three quarters of the light.
    assert(near(encode(0.25), 0.53709873));
    assert(near(encode(0.75), 0.88082502));
}

void linearSegmentNearBlack() {
    // Below 0.04045 the curve is the straight line s / 12.92 (and l * 12.92 on the way back).
    assert(near(decode(0.02), 0.02 / 12.92, 1e-12));
    assert(near(decode(0.04045), 0.0031308049536, 1e-12));
    assert(near(encode(0.0031308), 0.040449936, 1e-12));
    assert(near(encode(0.001), 0.01292, 1e-12));
    // No jump where the two pieces meet: a step of 1e-7 in light moves the code by about 12.92 * 1e-7 = 1.3e-6.
    assert(std::fabs(decode(0.04045) - decode(0.04046)) < 1e-6);
    assert(std::fabs(encode(0.0031308) - encode(0.0031309)) < 2e-6);
}

void bytes() {
    assert(toByte(0.0) == 0);
    assert(toByte(1.0) == 255);
    assert(toByte(-1.0) == 0);
    assert(toByte(2.0) == 255);
    assert(toByte(0.5) == 128);  // 127.5 rounds up
    assert(toByte(1.0 / 255.0) == 1);
}

void roundTrip() {
    // Every 8 bit value survives decode, encode and back to 8 bit.
    double previous = -1.0;
    for (int b = 0; b < 256; ++b) {
        const double lin = decode(b / 255.0);
        assert(lin > previous);  // strictly increasing
        previous = lin;
        assert(toByte(encode(lin)) == b);
    }
    // And the float round trip is exact enough in both pieces of the curve.
    for (const double x : {0.0005, 0.001, 0.0031308, 0.01, 0.1, 0.5, 0.9}) {
        assert(near(decode(encode(x)), x, 1e-12));
    }
}

void blackWhiteMidpoint() {
    // The classic: 50 % between black and white.
    const Rgb naive = mixSrgb(kBlack, kWhite, 0.5);
    const Rgb right = mixLinear(kBlack, kWhite, 0.5);
    assert(nearRgb(naive, {0.5, 0.5, 0.5}));
    assert(toByte(naive.r) == 128);
    assert(nearRgb(right, {0.73535698, 0.73535698, 0.73535698}));
    assert(toByte(right.r) == 188);
    // Light actually leaving the pixels: 21 % for the naive gray, 50 % for the right one.
    assert(near(luminance(decode(naive)), 0.21404114));
    assert(near(luminance(decode(right)), 0.5));
}

void mixEndpointsAndClamp() {
    assert(nearRgb(mixLinear(kTeal, kOrange, 0.0), kTeal, 1e-12));
    assert(nearRgb(mixLinear(kTeal, kOrange, 1.0), kOrange, 1e-12));
    assert(nearRgb(mixSrgb(kTeal, kOrange, 0.0), kTeal, 1e-12));
    assert(nearRgb(mixSrgb(kTeal, kOrange, 1.0), kOrange, 1e-12));
    // t outside 0..1 is clamped, not extrapolated.
    assert(nearRgb(mixLinear(kTeal, kOrange, -3.0), kTeal, 1e-12));
    assert(nearRgb(mixLinear(kTeal, kOrange, 7.0), kOrange, 1e-12));
    assert(nearRgb(mixSrgb(kBlack, kWhite, 9.0), kWhite, 1e-12));
}

void redGreen() {
    const Rgb naive = mixSrgb(kRed, kGreen, 0.5);
    const Rgb right = mixLinear(kRed, kGreen, 0.5);
    assert(nearRgb(naive, {0.5, 0.5, 0.0}));
    assert(nearRgb(right, {0.73535698, 0.73535698, 0.0}));
    // Rec. 709 weights: 0.5 * 0.2126 + 0.5 * 0.7152 = 0.4639 is what two half lights add up to.
    assert(near(luminance(decode(right)), 0.4639));
    assert(near(luminance(decode(naive)), 0.19858737));
}

void mixingLinearIsAddingLight() {
    // The property that makes the linear mix right: its light is the weighted sum of the light of A and B.
    const double ya = luminance(decode(kTeal));
    const double yb = luminance(decode(kOrange));
    for (const double t : {0.0, 0.25, 0.5, 0.8, 1.0}) {
        const double expected = (1.0 - t) * ya + t * yb;
        assert(near(luminance(decode(mixLinear(kTeal, kOrange, t))), expected, 1e-9));
    }
    // The naive mix breaks it: at the middle it is 0.1770 where 0.2377 is expected.
    assert(near(luminance(decode(mixSrgb(kTeal, kOrange, 0.5))), 0.17699205, 1e-6));
    assert(near(0.5 * (ya + yb), 0.23770682, 1e-6));
    assert(std::fabs(luminance(decode(mixSrgb(kTeal, kOrange, 0.5))) - 0.5 * (ya + yb)) > 0.05);
    // Bytes of the teal/orange middle, none of them close to a .5 rounding edge.
    const Rgb n = mixSrgb(kTeal, kOrange, 0.5);
    const Rgb l = mixLinear(kTeal, kOrange, 0.5);
    assert(toByte(l.r) == 172 && toByte(l.g) == 123 && toByte(l.b) == 99);
    assert(toByte(n.g) == 118 && toByte(n.b) == 74);
}

void luminanceWeights() {
    assert(near(luminance(kWhite), 1.0, 1e-12));
    assert(near(luminance(kBlack), 0.0, 1e-12));
    assert(near(luminance(kRed), 0.2126, 1e-12));
    assert(near(luminance(kGreen), 0.7152, 1e-12));
    assert(near(luminance({0, 0, 1}), 0.0722, 1e-12));
}

void pipelines() {
    // Gray texture 0.5 (code value 128), half the light.
    assert(near(shade(Pipeline::Naive, 0.5, 0.5), 0.25));
    assert(near(shade(Pipeline::Correct, 0.5, 0.5), 0.36078021));
    assert(near(shade(Pipeline::RawAlbedo, 0.5, 0.5), 0.53709873));
    assert(near(shade(Pipeline::EncodedTwice, 0.5, 0.5), 0.63487476));
    // Naive is darkest, the doubled correction brightest.
    assert(shade(Pipeline::Naive, 0.5, 0.5) < shade(Pipeline::Correct, 0.5, 0.5));
    assert(shade(Pipeline::Correct, 0.5, 0.5) < shade(Pipeline::RawAlbedo, 0.5, 0.5));
    assert(shade(Pipeline::RawAlbedo, 0.5, 0.5) < shade(Pipeline::EncodedTwice, 0.5, 0.5));
}

void pipelinesAtFullAndNoLight() {
    // Full light: naive and correct both give the texture back, the two mistakes brighten it.
    for (const double a : {0.0, 0.2, 0.5, 0.8, 1.0}) {
        assert(near(shade(Pipeline::Naive, a, 1.0), a, 1e-12));
        assert(near(shade(Pipeline::Correct, a, 1.0), a, 1e-12));
    }
    assert(near(shade(Pipeline::RawAlbedo, 0.5, 1.0), 0.73535698));
    assert(near(shade(Pipeline::EncodedTwice, 0.5, 1.0), 0.73535698));
    // No light is black in every pipeline, white and black textures keep their ends.
    for (const Pipeline p : {Pipeline::Naive, Pipeline::Correct, Pipeline::RawAlbedo, Pipeline::EncodedTwice}) {
        assert(shade(p, 0.5, 0.0) == 0.0);
        assert(shade(p, 0.0, 1.0) == 0.0);
        assert(near(shade(p, 1.0, 1.0), 1.0, 1e-12));
    }
    // Too much light clamps, negative light is none.
    assert(near(shade(Pipeline::Correct, 1.0, 2.0), 1.0, 1e-12));
    assert(shade(Pipeline::Naive, 0.5, 4.0) == 1.0);
    assert(shade(Pipeline::Correct, 0.5, -1.0) == 0.0);
}

void vectors() {
    const Vec3 zero = normalize({0, 0, 0});
    assert(near(zero.x, 0.0, 1e-12) && near(zero.y, 0.0, 1e-12) && near(zero.z, 1.0, 1e-12));  // fallback, no NaN
    const Vec3 v = normalize({3, 0, 4});
    assert(near(v.x, 0.6, 1e-12) && near(v.y, 0.0, 1e-12) && near(v.z, 0.8, 1e-12));
}

void lambertTerm() {
    const Vec3 n{0, 0, 1};
    assert(near(lambert(n, {0, 0, 1}, 0.1), 1.0, 1e-12));    // facing the light: ambient + the rest
    assert(near(lambert(n, {0, 0, -1}, 0.1), 0.1, 1e-12));   // turned away: ambient only
    assert(near(lambert(n, {1, 0, 0}, 0.1), 0.1, 1e-12));    // edge on
    assert(near(lambert(n, {0.6, 0, 0.8}, 0.1), 0.1 + 0.9 * 0.8, 1e-12));
    assert(near(lambert(n, {0, 0, 1}, 0.0), 1.0, 1e-12));
    assert(near(lambert(n, {0, 0, -1}, 0.0), 0.0, 1e-12));
}

void sphereSurface() {
    Vec3 n{9, 9, 9};
    assert(sphereNormal(0, 0, 10, n));
    assert(near(n.x, 0.0, 1e-12) && near(n.y, 0.0, 1e-12) && near(n.z, 1.0, 1e-12));
    assert(sphereNormal(6, 0, 10, n));
    assert(near(n.x, 0.6, 1e-12) && near(n.z, 0.8, 1e-12));
    assert(sphereNormal(0, 6, 10, n));  // y points up
    assert(near(n.y, 0.6, 1e-12) && near(n.z, 0.8, 1e-12));
    assert(sphereNormal(10, 0, 10, n));  // the rim itself still belongs to the sphere
    assert(near(n.x, 1.0, 1e-12) && near(n.z, 0.0, 1e-12));
    assert(!sphereNormal(8, 8, 10, n));
    assert(!sphereNormal(10.1, 0, 10, n));
    assert(!sphereNormal(0, 0, 0, n));   // no radius, no sphere (and no division by zero)
    assert(!sphereNormal(0, 0, -5, n));
}

void rimCoverage() {
    assert(near(coverage(0.0, 10.0), 1.0, 1e-12));
    assert(near(coverage(9.5, 10.0), 1.0, 1e-12));
    assert(near(coverage(10.0, 10.0), 0.5, 1e-12));
    assert(near(coverage(10.25, 10.0), 0.25, 1e-12));
    assert(near(coverage(10.5, 10.0), 0.0, 1e-12));
    assert(near(coverage(30.0, 10.0), 0.0, 1e-12));
}

void checker() {
    constexpr double kS = 0.6427876097;  // sin 40 degrees
    constexpr double kC = 0.7660444431;  // cos 40 degrees
    assert(checkerCell({0, 0, 1}, 6));          // straight ahead: cell (0, 0), even
    assert(!checkerCell({kS, 0, kC}, 6));       // 40 degrees right: cell (1, 0), odd
    assert(!checkerCell({-0.1736481777, 0, 0.9848077530}, 6));  // 10 degrees left: cell (-1, 0), odd (floor, not truncation)
    assert(!checkerCell({0, kS, kC}, 6));       // 40 degrees up: cell (0, 1)
    assert(!checkerCell({0.5, 0.4, 0.7681145748}, 6));  // longitude 33 degrees, latitude 23.6: cell (1, 0)
    assert(checkerCell({0.5, 0.6, 0.6244997998}, 6));   // longitude 38.7, latitude 36.9: cell (1, 1), even
    assert(checkerCell({0, 0, 1}, 0));          // no cells: plain
    (void)checkerCell({0, 1.0000001, 0}, 6);    // rounding slightly above 1 must not produce NaN
}

void lightDirection() {
    const Vec3 mid = lightFromMouse(480, 340);
    assert(near(mid.x, 0.0, 1e-12) && near(mid.y, 0.0, 1e-12) && near(mid.z, 1.0, 1e-12));
    const Vec3 right = lightFromMouse(780, 340);  // raw (1, 0, 0.6) over sqrt(1.36)
    assert(near(right.x, 0.857493, 1e-6) && near(right.y, 0.0, 1e-12) && near(right.z, 0.514496, 1e-6));
    const Vec3 up = lightFromMouse(480, 160);  // screen y grows downwards, the light goes up
    assert(near(up.x, 0.0, 1e-12) && near(up.y, 0.857493, 1e-6) && near(up.z, 0.514496, 1e-6));
    for (double x = -400.0; x < 1400.0; x += 97.0) {
        for (double y = -300.0; y < 900.0; y += 61.0) {
            const Vec3 l = lightFromMouse(x, y);
            assert(near(l.x * l.x + l.y * l.y + l.z * l.z, 1.0, 1e-9));
            assert(l.z > 0.0);  // always in front of the sphere
        }
    }
}

void sliderPosition() {
    // A bar from x = 24, 536 px wide: the left edge is 0, the middle 0.5, the right edge 1, outside is clamped.
    assert(near(sliderT(24.0, 24.0, 536.0), 0.0, 1e-12));
    assert(near(sliderT(292.0, 24.0, 536.0), 0.5, 1e-12));
    assert(near(sliderT(560.0, 24.0, 536.0), 1.0, 1e-12));
    assert(near(sliderT(-100.0, 24.0, 536.0), 0.0, 1e-12));
    assert(near(sliderT(900.0, 24.0, 536.0), 1.0, 1e-12));
    // A bar without width cannot be divided by; it reads as 0.
    assert(sliderT(100.0, 24.0, 0.0) == 0.0);
    assert(sliderT(100.0, 24.0, -5.0) == 0.0);
}

}  // namespace

int main() {
    curveEndpoints();
    curveClampsItsInput();
    knownValues();
    linearSegmentNearBlack();
    bytes();
    roundTrip();
    blackWhiteMidpoint();
    mixEndpointsAndClamp();
    redGreen();
    mixingLinearIsAddingLight();
    luminanceWeights();
    pipelines();
    pipelinesAtFullAndNoLight();
    vectors();
    lambertTerm();
    sphereSurface();
    rimCoverage();
    checker();
    lightDirection();
    sliderPosition();
    std::puts("gamma-linear logic: all asserts passed");
    return 0;
}
