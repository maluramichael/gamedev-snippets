// Tests for the ordered-dithering logic. Plain asserts on hand-computed cases, no framework, no SDL.
// Threshold maps are checked against the published index matrices (Bayer, 2x2 / 4x4 / 8x8), the rest against
// small flat images where the number of white pixels follows from the matrix by hand.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "logic.hpp"

using namespace od;

namespace {

bool near(double a, double b, double eps = 1e-9) { return std::fabs(a - b) < eps; }

Image flat(int w, int h, Rgb c) {
    Image img;
    img.w = w;
    img.h = h;
    img.px.assign(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), c);
    return img;
}

/// Number of pixels in a mono result that came out white.
int whites(const Image& img) {
    int n = 0;
    for (const Rgb& p : img.px) n += p.r > 0.5 ? 1 : 0;
    return n;
}

double meanLinear(const Image& img) {
    double sum = 0.0;
    for (const Rgb& p : img.px) sum += srgbToLinear(p.r);
    return sum / static_cast<double>(img.px.size());
}

constexpr Palette kMono{2, true};
constexpr Palette kFour{4, true};
constexpr Palette kRgb8{2, false};

void transferCurve() {
    assert(near(srgbToLinear(0.0), 0.0));
    assert(near(srgbToLinear(1.0), 1.0));
    assert(near(srgbToLinear(0.5), 0.2140411, 1e-6));          // the well known mid gray
    assert(near(srgbToLinear(0.04045), 0.04045 / 12.92));      // end of the linear toe
    assert(near(linearToSrgb(0.0031308), 0.0031308 * 12.92));  // same point from the other side
    for (double v = 0.0; v <= 1.0; v += 0.05) assert(near(linearToSrgb(srgbToLinear(v)), v, 1e-9));
    for (double v = 0.0; v < 1.0; v += 0.01) assert(srgbToLinear(v) < srgbToLinear(v + 0.01));
    // Out of range and NaN are clamped, not propagated.
    assert(near(srgbToLinear(-0.5), 0.0));
    assert(near(srgbToLinear(2.0), 1.0));
    assert(near(srgbToLinear(std::nan("")), 0.0));
    assert(near(linearToSrgb(7.0), 1.0));
}

void bayerMatrices() {
    // 2x2: [0 2; 3 1]
    const int m2[2][2] = {{0, 2}, {3, 1}};
    for (int y = 0; y < 2; ++y)
        for (int x = 0; x < 2; ++x) assert(bayerIndex(2, x, y) == m2[y][x]);
    // 4x4: rows written out from the reference matrix
    const int m4[4][4] = {{0, 8, 2, 10}, {12, 4, 14, 6}, {3, 11, 1, 9}, {15, 7, 13, 5}};
    for (int y = 0; y < 4; ++y)
        for (int x = 0; x < 4; ++x) assert(bayerIndex(4, x, y) == m4[y][x]);
    // 8x8: first, second and last row of the reference matrix
    const int row0[8] = {0, 32, 8, 40, 2, 34, 10, 42};
    const int row1[8] = {48, 16, 56, 24, 50, 18, 58, 26};
    const int row7[8] = {63, 31, 55, 23, 61, 29, 53, 21};
    for (int x = 0; x < 8; ++x) {
        assert(bayerIndex(8, x, 0) == row0[x]);
        assert(bayerIndex(8, x, 1) == row1[x]);
        assert(bayerIndex(8, x, 7) == row7[x]);
    }
    // 1x1 is the seed of the recursion.
    assert(bayerIndex(1, 5, 9) == 0);
}

void bayerIsAPermutation() {
    for (int n : {2, 4, 8, 16}) {
        std::vector<int> seen(static_cast<std::size_t>(n * n), 0);
        for (int y = 0; y < n; ++y)
            for (int x = 0; x < n; ++x) {
                const int i = bayerIndex(n, x, y);
                assert(i >= 0 && i < n * n);
                ++seen[static_cast<std::size_t>(i)];
            }
        for (int c : seen) assert(c == 1);
    }
}

void bayerWrapsAndRejects() {
    assert(bayerIndex(4, 4, 0) == bayerIndex(4, 0, 0));    // tiles to the right
    assert(bayerIndex(4, 5, 7) == bayerIndex(4, 1, 3));    // and diagonally
    assert(bayerIndex(4, -1, 0) == bayerIndex(4, 3, 0));   // negative coordinates wrap too
    assert(bayerIndex(4, -1, 0) == 10);                    // (3,0) in the table above
    assert(bayerIndex(3, 1, 1) == 0);                      // not a power of two: no matrix
    assert(bayerIndex(0, 1, 1) == 0);
    assert(bayerIndex(-4, 1, 1) == 0);
}

void thresholdsAreCentered() {
    // (index + 0.5) / n^2, so a value of k/n^2 beats exactly k cells.
    assert(near(threshold(Pattern::Bayer2, 0, 0), 0.5 / 4.0));
    assert(near(threshold(Pattern::Bayer2, 1, 0), 2.5 / 4.0));
    assert(near(threshold(Pattern::Bayer4, 3, 0), 10.5 / 16.0));
    assert(near(threshold(Pattern::Bayer8, 1, 0), 32.5 / 64.0));
    assert(near(threshold(Pattern::Off, 3, 3), 0.5));
    for (Pattern p : {Pattern::Bayer2, Pattern::Bayer4, Pattern::Bayer8, Pattern::Noise})
        for (int y = 0; y < 20; ++y)
            for (int x = 0; x < 20; ++x) {
                const double t = threshold(p, x, y);
                assert(t > 0.0 && t < 1.0);  // strictly inside, so 0 never turns on and 1 never turns off
            }
    assert(patternSize(Pattern::Off) == 0 && patternSize(Pattern::Bayer2) == 2);
    assert(patternSize(Pattern::Bayer4) == 4 && patternSize(Pattern::Bayer8) == 8 && patternSize(Pattern::Noise) == 0);
}

void noiseIsFixedAndUniformEnough() {
    assert(noiseThreshold(7, 9) == noiseThreshold(7, 9));  // same pixel, same value: no flicker
    assert(noiseThreshold(7, 9) != noiseThreshold(9, 7));
    double sum = 0.0;
    int below = 0;
    for (int y = 0; y < 64; ++y)
        for (int x = 0; x < 64; ++x) {
            const double t = noiseThreshold(x, y);
            sum += t;
            below += t < 0.25 ? 1 : 0;
        }
    assert(near(sum / 4096.0, 0.5, 0.03));
    assert(std::abs(below - 1024) < 150);  // about a quarter lies under 0.25
}

void quantizeChannelEdges() {
    // One channel, two levels, threshold 0.5 (no matrix): rounding, ties go down.
    assert(near(quantizeChannel(0.49, 2, 0.5, Space::Srgb), 0.0));
    assert(near(quantizeChannel(0.51, 2, 0.5, Space::Srgb), 1.0));
    assert(near(quantizeChannel(0.5, 2, 0.5, Space::Srgb), 0.0));
    // In linear light the halfway point moves: 0.7 is 0.448 linear (black), 0.8 is 0.605 (white).
    assert(near(quantizeChannel(0.7, 2, 0.5, Space::Linear), 0.0));
    assert(near(quantizeChannel(0.8, 2, 0.5, Space::Linear), 1.0));
    // The ends never flip, whatever the threshold.
    for (Space s : {Space::Srgb, Space::Linear}) {
        assert(near(quantizeChannel(0.0, 2, 0.001, s), 0.0));
        assert(near(quantizeChannel(1.0, 2, 0.999, s), 1.0));
        assert(near(quantizeChannel(-3.0, 4, 0.5, s), 0.0));
        assert(near(quantizeChannel(3.0, 4, 0.5, s), 1.0));
    }
    // Four levels sit at 0, 1/3, 2/3, 1. Between two of them the choice is made inside that interval only.
    assert(near(quantizeChannel(0.5, 4, 0.4, Space::Srgb), 2.0 / 3.0));  // t = 0.5 in [1/3, 2/3], 0.5 > 0.4
    assert(near(quantizeChannel(0.5, 4, 0.6, Space::Srgb), 1.0 / 3.0));
    assert(near(quantizeChannel(0.2, 4, 0.1, Space::Srgb), 1.0 / 3.0));  // t = 0.6 in [0, 1/3]
    assert(near(quantizeChannel(0.9, 4, 0.9, Space::Srgb), 2.0 / 3.0));  // t = 0.7 in [2/3, 1], 0.7 < 0.9
    // Fewer than two levels is no palette: black.
    assert(near(quantizeChannel(0.8, 1, 0.5, Space::Srgb), 0.0));
    assert(near(quantizeChannel(0.8, 0, 0.5, Space::Srgb), 0.0));
}

void bayer2x2PlacesPixels() {
    // 25 percent (v = 0.3): only the cell with index 0 (top left) is white.
    const Image a = dither(flat(2, 2, {0.3, 0.3, 0.3}), kMono, Pattern::Bayer2, Space::Srgb);
    assert(a.px[0].r == 1.0 && a.px[1].r == 0.0 && a.px[2].r == 0.0 && a.px[3].r == 0.0);
    // 75 percent (v = 0.7): indices 0, 1, 2 are white, index 3 (bottom left) stays black.
    const Image b = dither(flat(2, 2, {0.7, 0.7, 0.7}), kMono, Pattern::Bayer2, Space::Srgb);
    assert(b.px[0].r == 1.0 && b.px[1].r == 1.0 && b.px[2].r == 0.0 && b.px[3].r == 1.0);
    // Half: two of four.
    assert(whites(dither(flat(2, 2, {0.5, 0.5, 0.5}), kMono, Pattern::Bayer2, Space::Srgb)) == 2);
    // Pure black and white stay pure.
    assert(whites(dither(flat(2, 2, {0.0, 0.0, 0.0}), kMono, Pattern::Bayer2, Space::Srgb)) == 0);
    assert(whites(dither(flat(2, 2, {1.0, 1.0, 1.0}), kMono, Pattern::Bayer2, Space::Srgb)) == 4);
}

void everyTileHasTheSameShare() {
    // k/16 of a 4x4 tile is white for a value of k/16, in every tile of a bigger image.
    for (int k = 0; k <= 16; ++k) {
        const double v = k / 16.0;
        const Image out = dither(flat(8, 8, {v, v, v}), kMono, Pattern::Bayer4, Space::Srgb);
        assert(whites(out) == 4 * k);  // four tiles
    }
    // The same for 8x8: 20/64 of the pixels.
    const double v = 20.0 / 64.0;
    assert(whites(dither(flat(8, 8, {v, v, v}), kMono, Pattern::Bayer8, Space::Srgb)) == 20);
}

void linearLightKeepsTheBrightness() {
    // Flat mid gray 0.5 is 0.214 in linear light. Bayer 4x4 in linear space: thresholds (j + 0.5) / 16 below 0.214
    // are j = 0, 1, 2, so 3 of 16 pixels are white. In sRGB space it is 8 of 16 and the area comes out too bright.
    const Image mid = flat(4, 4, {0.5, 0.5, 0.5});
    const Image right = dither(mid, kMono, Pattern::Bayer4, Space::Linear);
    const Image naive = dither(mid, kMono, Pattern::Bayer4, Space::Srgb);
    assert(whites(right) == 3);
    assert(whites(naive) == 8);
    assert(near(meanLinear(right), 3.0 / 16.0));
    assert(near(meanLinear(naive), 0.5));
    // The correct one is within half a matrix step of the real brightness, the naive one is far off.
    const double target = srgbToLinear(0.5);
    assert(std::fabs(meanLinear(right) - target) <= 0.5 / 16.0);
    assert(std::fabs(meanLinear(naive) - target) > 0.25);
    // Black and white are the same in both spaces.
    for (double v : {0.0, 1.0}) {
        const Image f = flat(4, 4, {v, v, v});
        assert(whites(dither(f, kMono, Pattern::Bayer4, Space::Linear)) == whites(dither(f, kMono, Pattern::Bayer4, Space::Srgb)));
    }
}

void linearLightHoldsForEveryMatrixAndPalette() {
    // For many flat values: mean linear light of the output stays within half a matrix step (0.5 / n^2) of the target.
    for (Pattern p : {Pattern::Bayer2, Pattern::Bayer4, Pattern::Bayer8}) {
        const int n = patternSize(p);
        const double tolerance = 0.5 / static_cast<double>(n * n) + 1e-9;
        for (double v = 0.0; v <= 1.0; v += 0.0625) {
            const Image mono = dither(flat(8, 8, {v, v, v}), kMono, p, Space::Linear);
            assert(std::fabs(meanLinear(mono) - srgbToLinear(v)) <= tolerance);
        }
    }
    // Four grays: the choice happens between the two neighbouring palette entries. The widest gap in linear light is
    // 1 - 0.402 = 0.598 (top interval), so the error is at most 0.598 * 0.5 / 64.
    for (double v = 0.0; v <= 1.0; v += 0.0625) {
        const Image four = dither(flat(8, 8, {v, v, v}), kFour, Pattern::Bayer8, Space::Linear);
        assert(std::fabs(meanLinear(four) - srgbToLinear(v)) <= 0.005);
        for (const Rgb& px : four.px) {  // and only palette entries show up
            const double k = px.r * 3.0;
            assert(near(k, std::round(k)));
        }
    }
}

void fourLevelsMixTwoNeighbours() {
    // 0.5 sits halfway between 1/3 and 2/3 (sRGB space): Bayer 2x2 gives two of each.
    const Image out = dither(flat(2, 2, {0.5, 0.5, 0.5}), kFour, Pattern::Bayer2, Space::Srgb);
    int low = 0;
    int high = 0;
    for (const Rgb& px : out.px) {
        if (near(px.r, 1.0 / 3.0)) ++low;
        if (near(px.r, 2.0 / 3.0)) ++high;
    }
    assert(low == 2 && high == 2);
}

void offIsPlainRounding() {
    const Image ramp = [] {
        Image img = flat(5, 1, {0, 0, 0});
        for (int x = 0; x < 5; ++x) img.px[static_cast<std::size_t>(x)] = {x / 4.0, x / 4.0, x / 4.0};
        return img;
    }();
    const Image out = dither(ramp, kMono, Pattern::Off, Space::Srgb);
    assert(out.px[0].r == 0.0 && out.px[1].r == 0.0 && out.px[2].r == 0.0 && out.px[3].r == 1.0 && out.px[4].r == 1.0);
}

void grayPaletteUsesLuminance() {
    // Pure green is bright (0.7152 in linear light), pure blue is dark (0.0722). Gray conversion happens in linear light.
    const Image g = dither(flat(1, 1, {0.0, 1.0, 0.0}), kMono, Pattern::Off, Space::Linear);
    const Image b = dither(flat(1, 1, {0.0, 0.0, 1.0}), kMono, Pattern::Off, Space::Linear);
    assert(g.px[0].r == 1.0 && g.px[0].g == 1.0 && g.px[0].b == 1.0);
    assert(b.px[0].r == 0.0 && b.px[0].g == 0.0 && b.px[0].b == 0.0);
    assert(near(luminance({1, 1, 1}), 1.0));
    assert(near(luminance({0, 0, 0}), 0.0));
    assert(near(luminance({0, 1, 0}), 0.7152));
    assert(near(luminance({0, 0, 1}), 0.0722));
}

void colorPaletteKeepsChannelsApart() {
    // Orange-ish (1, 0.5, 0) with 2 levels per channel and no matrix: red on, green rounds down, blue off.
    const Image o = dither(flat(1, 1, {1.0, 0.4, 0.0}), kRgb8, Pattern::Off, Space::Srgb);
    assert(o.px[0].r == 1.0 && o.px[0].g == 0.0 && o.px[0].b == 0.0);
    // Same pixel with a 2x2 matrix: the green channel flips on in the cells whose threshold is below 0.4.
    const Image m = dither(flat(2, 2, {1.0, 0.4, 0.0}), kRgb8, Pattern::Bayer2, Space::Srgb);
    int greenOn = 0;
    for (const Rgb& px : m.px) {
        assert(px.r == 1.0 && px.b == 0.0);
        greenOn += px.g == 1.0 ? 1 : 0;
    }
    assert(greenOn == 2);  // thresholds 0.125 and 0.375 are below 0.4, 0.625 and 0.875 are not
}

void sizes() {
    assert(paletteSize(kMono) == 2 && paletteSize(kFour) == 4);
    assert(paletteSize(kRgb8) == 8 && paletteSize({4, false}) == 64);
    assert(paletteSize({1, false}) == 1);
    // Distinct tones per channel: (levels - 1) * n^2 + 1.
    assert(tones(Pattern::Bayer2, 2) == 5);
    assert(tones(Pattern::Bayer4, 2) == 17);
    assert(tones(Pattern::Bayer8, 2) == 65);
    assert(tones(Pattern::Bayer2, 4) == 13);
    assert(tones(Pattern::Off, 4) == 4);
    assert(tones(Pattern::Noise, 2) == -1);  // random: no fixed count
}

void hsv() {
    const Rgb red = hsvToRgb(0.0, 1.0, 1.0);
    const Rgb yellow = hsvToRgb(60.0, 1.0, 1.0);
    const Rgb green = hsvToRgb(120.0, 1.0, 1.0);
    const Rgb blue = hsvToRgb(240.0, 1.0, 1.0);
    assert(near(red.r, 1) && near(red.g, 0) && near(red.b, 0));
    assert(near(yellow.r, 1) && near(yellow.g, 1) && near(yellow.b, 0));
    assert(near(green.r, 0) && near(green.g, 1) && near(green.b, 0));
    assert(near(blue.r, 0) && near(blue.g, 0) && near(blue.b, 1));
    const Rgb gray = hsvToRgb(200.0, 0.0, 0.4);
    assert(near(gray.r, 0.4) && near(gray.g, 0.4) && near(gray.b, 0.4));
    const Rgb wrapped = hsvToRgb(360.0, 1.0, 1.0);  // full turn is red again
    assert(near(wrapped.r, 1) && near(wrapped.g, 0) && near(wrapped.b, 0));
    const Rgb negative = hsvToRgb(-120.0, 1.0, 1.0);  // and -120 is blue
    assert(near(negative.r, 0) && near(negative.g, 0) && near(negative.b, 1));
}

void testImage() {
    const Image a = makeTestImage(312, 192);
    assert(a.w == 312 && a.h == 192 && a.px.size() == 312U * 192U);
    for (const Rgb& p : a.px) {
        assert(p.r >= 0.0 && p.r <= 1.0 && p.g >= 0.0 && p.g <= 1.0 && p.b >= 0.0 && p.b <= 1.0);
    }
    // The top band is a gray ramp from black to white, never decreasing.
    assert(near(a.px[0].r, 0.0) && near(a.px[311].r, 1.0));
    for (int x = 0; x < 311; ++x) {
        const Rgb p = a.px[static_cast<std::size_t>(x)];
        const Rgb q = a.px[static_cast<std::size_t>(x + 1)];
        assert(near(p.r, p.g) && near(p.g, p.b));
        assert(q.r >= p.r);
    }
    // Deterministic.
    const Image b = makeTestImage(312, 192);
    for (std::size_t i = 0; i < a.px.size(); ++i) assert(a.px[i].r == b.px[i].r && a.px[i].g == b.px[i].g && a.px[i].b == b.px[i].b);
    // Degenerate sizes do not crash and give an empty or tiny image.
    assert(makeTestImage(0, 10).px.empty());
    assert(makeTestImage(10, -1).px.empty());
    const Image one = makeTestImage(1, 1);
    assert(one.px.size() == 1);
}

void rgbaWriteHonoursThePitch() {
    Image img = flat(3, 2, {0, 0, 0});
    img.px[5] = {1.0, 0.5, 0.0};  // bottom right
    img.px[0] = {0.0, 1.0, 1.0};
    constexpr int kPitch = 16;  // 4 bytes of padding per row
    std::vector<std::uint8_t> buf(static_cast<std::size_t>(kPitch) * 2U, 0xAB);
    toRgba(img, buf.data(), kPitch);
    // Pixel (2,1) starts at row 1 * pitch + 2 * 4.
    const std::size_t at = static_cast<std::size_t>(kPitch) + 8U;
    assert(buf[at] == 255 && buf[at + 1] == 128 && buf[at + 2] == 0 && buf[at + 3] == 255);
    assert(buf[0] == 0 && buf[1] == 255 && buf[2] == 255 && buf[3] == 255);
    // The padding after each row is untouched.
    for (int y = 0; y < 2; ++y)
        for (int i = 12; i < kPitch; ++i) assert(buf[static_cast<std::size_t>(y * kPitch + i)] == 0xAB);
    // A pitch that is too small writes nothing instead of overrunning.
    std::vector<std::uint8_t> small(64, 0xCD);
    toRgba(img, small.data(), 8);
    for (std::uint8_t b : small) assert(b == 0xCD);
}

void blockErrorCases() {
    const Image half = flat(4, 4, {0.5, 0.5, 0.5});
    // Identical images: no error.
    assert(near(blockError(half, half, 2), 0.0));
    // A checkerboard of black and white against a flat gray whose *linear* value is 0.5: every 2x2 block averages 0.5.
    Image checker = flat(4, 4, {0, 0, 0});
    for (int y = 0; y < 4; ++y)
        for (int x = 0; x < 4; ++x) {
            const double v = (x + y) % 2 == 0 ? 1.0 : 0.0;
            checker.px[static_cast<std::size_t>(y * 4 + x)] = {v, v, v};
        }
    const double gray = linearToSrgb(0.5);
    assert(near(blockError(flat(4, 4, {gray, gray, gray}), checker, 2), 0.0, 1e-9));
    // All white against that gray: every block is off by 0.5.
    assert(near(blockError(flat(4, 4, {gray, gray, gray}), flat(4, 4, {1, 1, 1}), 2), 0.5, 1e-9));
    // Block 1 compares single pixels: the checkerboard is off by 0.5 everywhere.
    assert(near(blockError(flat(4, 4, {gray, gray, gray}), checker, 1), 0.5, 1e-9));
    // Only full blocks count: a 5x5 image with block 2 uses the 4x4 corner.
    assert(near(blockError(flat(5, 5, {gray, gray, gray}), flat(5, 5, {1, 1, 1}), 2), 0.5, 1e-9));
    // Nothing to compare: sizes differ, block too big, block below 1.
    assert(near(blockError(half, flat(2, 2, {0, 0, 0}), 2), 0.0));
    assert(near(blockError(half, half, 8), 0.0));
    assert(near(blockError(half, half, 0), 0.0));
}

void errorOrdering() {
    // On the real test image the linear-light result is closer to the original (block means) than the naive one.
    const Image src = makeTestImage(312, 192);
    const double right = blockError(src, dither(src, kMono, Pattern::Bayer8, Space::Linear), 8);
    const double naive = blockError(src, dither(src, kMono, Pattern::Bayer8, Space::Srgb), 8);
    assert(right < naive);
    // And a finer matrix beats a coarser one in the correct space.
    const double coarse = blockError(src, dither(src, kMono, Pattern::Bayer2, Space::Linear), 8);
    assert(right < coarse);
    // Dithering beats plain rounding.
    const double off = blockError(src, dither(src, kMono, Pattern::Off, Space::Linear), 8);
    assert(right < off);
}

}  // namespace

int main() {
    transferCurve();
    bayerMatrices();
    bayerIsAPermutation();
    bayerWrapsAndRejects();
    thresholdsAreCentered();
    noiseIsFixedAndUniformEnough();
    quantizeChannelEdges();
    bayer2x2PlacesPixels();
    everyTileHasTheSameShare();
    linearLightKeepsTheBrightness();
    linearLightHoldsForEveryMatrixAndPalette();
    fourLevelsMixTwoNeighbours();
    offIsPlainRounding();
    grayPaletteUsesLuminance();
    colorPaletteKeepsChannelsApart();
    sizes();
    hsv();
    testImage();
    rgbaWriteHonoursThePitch();
    blockErrorCases();
    errorOrdering();
    std::puts("ordered-dithering logic: all asserts passed");
    return 0;
}
