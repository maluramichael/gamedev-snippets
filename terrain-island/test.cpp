// Tests for the island logic. Plain asserts on hand-computed cases, no framework, no SDL.
#undef NDEBUG
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>

#include "logic.hpp"

using namespace isl;

namespace {

bool near(float a, float b, float eps = 1e-5F) { return std::fabs(a - b) < eps; }

void fadeCurve() {
    assert(fade(0.0F) == 0.0F);
    assert(fade(1.0F) == 1.0F);
    assert(near(fade(0.5F), 0.5F));
    // 6/1024 - 15/256 + 10/64 = 0.103515625
    assert(near(fade(0.25F), 0.103515625F));
    assert(near(fade(0.75F), 1.0F - 0.103515625F));  // point symmetric around 0.5
    // The slope is zero at both ends, that is the whole point of 6t^5 - 15t^4 + 10t^3: no visible grid lines.
    assert(fade(0.001F) < 1e-6F);
    assert(fade(0.999F) > 1.0F - 1e-6F);
}

void gradientsAreUnit() {
    for (int ix = -5; ix <= 5; ++ix) {
        for (int iy = -5; iy <= 5; ++iy) {
            const Vec2 g = gradient(ix, iy, 3);
            assert(near(g.x * g.x + g.y * g.y, 1.0F, 1e-4F));
        }
    }
}

void noiseProperties() {
    // Gradient noise is exactly zero on every lattice point.
    for (int ix = -4; ix <= 4; ++ix) {
        for (int iy = -4; iy <= 4; ++iy) {
            assert(noise(static_cast<float>(ix), static_cast<float>(iy), 11) == 0.0F);
        }
    }
    // Hand derived: on the bottom edge of a cell (ty = 0, u = 0.5) only the two lower corners count:
    // 0.5 * (g00.x * 0.5 + g10.x * -0.5) = 0.25 * (g00.x - g10.x), scaled to [-1, 1] by sqrt(2).
    const Vec2 g00 = gradient(0, 0, 5);
    const Vec2 g10 = gradient(1, 0, 5);
    assert(near(noise(0.5F, 0.0F, 5), 0.25F * (g00.x - g10.x) * 1.41421356F, 1e-5F));
    // Same input, same output. Another seed changes the picture.
    assert(noise(2.37F, -4.81F, 9) == noise(2.37F, -4.81F, 9));
    bool differs = false;
    float maxAbs = 0.0F;
    double sum = 0.0;
    int count = 0;
    for (int i = 0; i < 64; ++i) {
        for (int j = 0; j < 64; ++j) {
            const float x = static_cast<float>(i) * 0.37F - 5.0F;
            const float y = static_cast<float>(j) * 0.37F - 5.0F;
            const float a = noise(x, y, 1);
            if (a != noise(x, y, 2)) differs = true;
            maxAbs = std::max(maxAbs, std::fabs(a));
            sum += static_cast<double>(a);
            ++count;
        }
    }
    assert(differs);
    assert(maxAbs <= 1.0F);  // never leaves [-1, 1]
    assert(maxAbs > 0.5F);   // but it is not flat either
    assert(std::fabs(sum / count) < 0.1);
    // Continuous: a tiny step in x moves the value only a tiny bit.
    for (int i = 0; i < 200; ++i) {
        const float x = static_cast<float>(i) * 0.113F;
        assert(std::fabs(noise(x + 1e-4F, 0.77F, 1) - noise(x, 0.77F, 1)) < 1e-3F);
    }
    // Huge or non finite input must not break the int conversion.
    assert(std::isfinite(noise(1e30F, -1e30F, 1)));
    assert(noise(std::nanf(""), 0.0F, 1) == 0.0F);
}

void fbmProperties() {
    // One octave is the plain noise, octave 0 has no offset.
    assert(near(fbm(1.3F, 2.9F, 1, 4), noise(1.3F, 2.9F, 4)));
    // More octaves change the value but stay inside [-1, 1] because the sum is divided by the amplitudes.
    assert(!near(fbm(1.3F, 2.9F, 5, 4), noise(1.3F, 2.9F, 4)));
    float maxAbs = 0.0F;
    for (int i = 0; i < 80; ++i) {
        for (int j = 0; j < 80; ++j) {
            maxAbs = std::max(maxAbs, std::fabs(fbm(static_cast<float>(i) * 0.21F, static_cast<float>(j) * 0.21F, 8, 6)));
        }
    }
    assert(maxAbs <= 1.0F);
    // Octave counts are clamped to 1..8.
    assert(fbm(0.4F, 0.4F, 0, 1) == fbm(0.4F, 0.4F, 1, 1));
    assert(fbm(0.4F, 0.4F, 99, 1) == fbm(0.4F, 0.4F, 8, 1));
    // The octaves are offset against each other, so the lattice zeros of octave 0 are not zero for the sum.
    assert(fbm(2.0F, 3.0F, 4, 1) != 0.0F);
}

void distances() {
    assert(distance(Shape::Circle, 0.0F, 0.0F) == 0.0F);
    assert(near(distance(Shape::Circle, 0.6F, 0.8F), 1.0F));
    assert(near(distance(Shape::Circle, -0.3F, 0.4F), 0.5F));  // 3-4-5 triangle
    assert(distance(Shape::Circle, 1.0F, 1.0F) == 1.0F);       // the corner would be 1.414, clamped
    assert(near(distance(Shape::Square, 0.5F, -0.9F), 0.9F));
    assert(distance(Shape::Square, 1.0F, 1.0F) == 1.0F);
    assert(near(distance(Shape::Square, -0.2F, 0.0F), 0.2F));
    assert(near(distance(Shape::Diamond, 0.25F, 0.25F), 0.5F));
    assert(near(distance(Shape::Diamond, -0.25F, 0.5F), 0.75F));
    assert(distance(Shape::Diamond, 0.8F, 0.8F) == 1.0F);
    // The same point is closer to the middle for a circle than for a square, and for a square than for a diamond.
    assert(distance(Shape::Circle, 0.5F, 0.5F) < distance(Shape::Diamond, 0.5F, 0.5F));
    assert(distance(Shape::Square, 0.5F, 0.5F) < distance(Shape::Circle, 0.5F, 0.5F));
}

void falloffCurve() {
    assert(falloff(Shape::Circle, 0.0F, 0.0F, 2.0F) == 0.0F);
    assert(near(falloff(Shape::Circle, 0.6F, 0.8F, 3.0F), 1.0F));
    assert(near(falloff(Shape::Square, 0.5F, 0.0F, 2.0F), 0.25F));    // 0.5^2
    assert(near(falloff(Shape::Square, 0.5F, 0.0F, 1.0F), 0.5F));     // exponent 1 is the plain distance
    assert(near(falloff(Shape::Square, 0.5F, 0.0F, 0.5F), 0.70710678F));  // a small exponent eats into the middle
    assert(falloff(Shape::Square, 0.5F, 0.0F, 4.0F) < falloff(Shape::Square, 0.5F, 0.0F, 1.0F));
    // An absurd exponent is clamped, the result stays a number in [0, 1].
    assert(near(falloff(Shape::Square, 0.5F, 0.0F, 0.0F), std::pow(0.5F, 0.1F)));
    assert(falloff(Shape::Square, 0.5F, 0.0F, 1000.0F) >= 0.0F);
    assert(falloff(Shape::Square, 0.5F, 0.0F, 1000.0F) <= 1.0F);
}

void combining() {
    // subtract: e - strength * f
    assert(near(combine(Mode::Subtract, 0.6F, 0.25F, 0.5F), 0.475F));
    assert(near(combine(Mode::Subtract, 0.6F, 0.25F, 0.0F), 0.6F));
    assert(combine(Mode::Subtract, 0.2F, 1.0F, 1.0F) == 0.0F);  // clamped, never negative
    // lerp: between the terrain and the mask 1 - f
    assert(near(combine(Mode::Lerp, 0.6F, 0.25F, 0.5F), 0.675F));  // lerp(0.6, 0.75, 0.5)
    assert(near(combine(Mode::Lerp, 0.6F, 0.25F, 0.0F), 0.6F));
    assert(near(combine(Mode::Lerp, 0.6F, 0.25F, 1.0F), 0.75F));   // strength 1: the noise is gone
    assert(near(combine(Mode::Lerp, 0.9F, 1.0F, 0.5F), 0.45F));    // at the border the mask is 0, so it pulls down
    // Strength outside 0..1 is clamped.
    assert(near(combine(Mode::Lerp, 0.6F, 0.25F, 7.0F), 0.75F));
    assert(near(combine(Mode::Lerp, 0.6F, 0.25F, -3.0F), 0.6F));
}

void coordinates() {
    assert(near(cellToNorm(0, 4), -0.75F));
    assert(near(cellToNorm(3, 4), 0.75F));
    assert(near(cellToNorm(1, 4), -0.25F));
    assert(near(cellToNorm(16, 33), 0.0F));  // odd size: the middle cell is exactly the middle
    assert(near(cellToNorm(0, 33), 1.0F / 33.0F - 1.0F));
    assert(near(cellToNorm(0, 7), -cellToNorm(6, 7)));
    assert(rowForY(40.0F, 40.0F, 480.0F, 160) == 0);
    assert(rowForY(42.99F, 40.0F, 480.0F, 160) == 0);
    assert(rowForY(43.0F, 40.0F, 480.0F, 160) == 1);
    assert(rowForY(280.0F, 40.0F, 480.0F, 160) == 80);
    assert(rowForY(519.9F, 40.0F, 480.0F, 160) == 159);
    assert(rowForY(10.0F, 40.0F, 480.0F, 160) == 0);     // above the map
    assert(rowForY(600.0F, 40.0F, 480.0F, 160) == 159);  // below the map
    assert(near(sliderValue(100.0F, 100.0F, 200.0F, 0.5F, 4.0F), 0.5F));
    assert(near(sliderValue(300.0F, 100.0F, 200.0F, 0.5F, 4.0F), 4.0F));
    assert(near(sliderValue(200.0F, 100.0F, 200.0F, 0.5F, 4.0F), 2.25F));
    assert(near(sliderValue(50.0F, 100.0F, 200.0F, 0.5F, 4.0F), 0.5F));   // left of the track
    assert(near(sliderValue(400.0F, 100.0F, 200.0F, 0.5F, 4.0F), 4.0F));  // right of the track
    assert(near(sliderValue(150.0F, 100.0F, 0.0F, 0.5F, 4.0F), 0.5F));    // a track without width
}

void heightmapShape() {
    Params p;
    p.seed = 3;
    p.shape = Shape::Circle;
    p.exponent = 1.0F;
    p.mode = Mode::Lerp;
    p.strength = 0.0F;
    const int n = 33;
    const Heightmap h = build(p, n);
    assert(h.n == n);
    assert(h.noise.size() == 33U * 33U && h.falloff.size() == h.noise.size() && h.height.size() == h.noise.size());
    for (float v : h.noise) assert(v >= 0.0F && v <= 1.0F);
    assert(h.falloff[16 * 33 + 16] == 0.0F);  // the middle cell has no falloff
    assert(near(h.falloff[16 * 33 + 0], 1.0F - 1.0F / 33.0F));  // left edge, row in the middle: d = 32/33
    // All four mirror images of the falloff are equal, for every shape.
    for (Shape s : {Shape::Circle, Shape::Square, Shape::Diamond}) {
        p.shape = s;
        const Heightmap m = build(p, n);
        for (int y = 0; y < n; ++y) {
            for (int x = 0; x < n; ++x) {
                const float f = m.falloff[static_cast<std::size_t>(y * n + x)];
                assert(near(f, m.falloff[static_cast<std::size_t>(y * n + (n - 1 - x))]));
                assert(near(f, m.falloff[static_cast<std::size_t>((n - 1 - y) * n + x)]));
                assert(f >= 0.0F && f <= 1.0F);
            }
        }
    }
    // Strength 0 leaves the plain noise.
    p.shape = Shape::Circle;
    p.mode = Mode::Subtract;
    const Heightmap plain = build(p, n);
    for (std::size_t i = 0; i < plain.height.size(); ++i) assert(plain.height[i] == plain.noise[i]);
}

void coneWhenTooStrong() {
    // Lerp with strength 1 throws the noise away: two seeds give exactly the same cone, and it is 1 - falloff.
    Params a;
    a.seed = 1;
    a.mode = Mode::Lerp;
    a.strength = 1.0F;
    a.exponent = 1.0F;
    Params b = a;
    b.seed = 2;
    const Heightmap ha = build(a, 33);
    const Heightmap hb = build(b, 33);
    assert(ha.height == hb.height);
    assert(near(ha.height[16 * 33 + 16], 1.0F));  // the peak is in the middle
    for (std::size_t i = 0; i < ha.height.size(); ++i) assert(near(ha.height[i], 1.0F - ha.falloff[i]));
    // With weaker strength the seed matters again.
    a.strength = 0.5F;
    b.strength = 0.5F;
    assert(build(a, 33).height != build(b, 33).height);
}

void islandIsInTheMiddle() {
    // Over several seeds the middle is clearly higher than the border ring once the falloff is on.
    for (std::uint32_t seed = 1; seed <= 6; ++seed) {
        Params p;
        p.seed = seed;
        p.mode = Mode::Lerp;
        p.strength = 0.7F;
        p.exponent = 1.5F;
        const int n = 40;
        const Heightmap h = build(p, n);
        double middle = 0.0;
        double ring = 0.0;
        int nm = 0;
        int nr = 0;
        for (int y = 0; y < n; ++y) {
            for (int x = 0; x < n; ++x) {
                const float v = h.height[static_cast<std::size_t>(y * n + x)];
                if (x >= n / 4 && x < n - n / 4 && y >= n / 4 && y < n - n / 4) {
                    middle += static_cast<double>(v);
                    ++nm;
                } else if (x == 0 || y == 0 || x == n - 1 || y == n - 1) {
                    ring += static_cast<double>(v);
                    ++nr;
                }
            }
        }
        assert(middle / nm > ring / nr + 0.25);
    }
}

void statistics() {
    const std::vector<float> g{0.1F, 0.5F, 0.7F, 0.2F};  // 2 x 2
    assert(near(landFraction(g, 0.3F), 0.5F));
    assert(landFraction(g, 0.0F) == 1.0F);
    assert(landFraction(g, 0.9F) == 0.0F);
    assert(landFraction({}, 0.3F) == 0.0F);  // no division by zero
    assert(isLand(0.3F, 0.3F));              // exactly at sea level counts as land (the beach)
    assert(!isLand(0.29F, 0.3F));

    std::vector<float> center(9, 0.0F);
    center[4] = 1.0F;
    assert(!touchesBorder(center, 3, 0.5F));
    for (std::size_t border : {0U, 1U, 2U, 3U, 5U, 6U, 7U, 8U}) {
        std::vector<float> one(9, 0.0F);
        one[border] = 1.0F;
        assert(touchesBorder(one, 3, 0.5F));
    }
    assert(!touchesBorder(std::vector<float>(9, 0.0F), 3, 0.5F));
    assert(!touchesBorder({}, 3, 0.5F));                         // wrong size
    assert(!touchesBorder(std::vector<float>(8, 1.0F), 3, 0.5F));  // 8 values for a 3 x 3 grid

    // Strong falloff keeps the sea at the border, no falloff does not.
    Params p;
    p.seed = 4;
    p.mode = Mode::Lerp;
    p.strength = 1.0F;
    assert(!touchesBorder(build(p, 41).height, 41, 0.05F));
    p.strength = 0.0F;
    assert(touchesBorder(build(p, 41).height, 41, 0.0F));
}

void bands() {
    const float sea = 0.3F;
    assert(classify(0.17F, sea) == Band::Deep);
    assert(classify(0.19F, sea) == Band::Shallow);
    assert(classify(0.2999F, sea) == Band::Shallow);
    assert(classify(0.3F, sea) == Band::Beach);
    assert(classify(0.32F, sea) == Band::Beach);
    assert(classify(0.34F, sea) == Band::Grass);
    assert(classify(0.45F, sea) == Band::Forest);
    assert(classify(0.65F, sea) == Band::Rock);
    assert(classify(0.9F, sea) == Band::Snow);
    // The bands move with the sea level.
    assert(classify(0.5F, 0.6F) == Band::Shallow);
    assert(classify(0.3F, 0.1F) == Band::Forest);
}

}  // namespace

int main() {
    fadeCurve();
    gradientsAreUnit();
    noiseProperties();
    fbmProperties();
    distances();
    falloffCurve();
    combining();
    coordinates();
    heightmapShape();
    coneWhenTooStrong();
    islandIsInTheMiddle();
    statistics();
    bands();
    std::puts("terrain-island logic: all tests passed");
    return 0;
}
