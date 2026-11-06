// Tests for the terrain-fbm logic. Plain asserts on hand-computed cases, no framework, no SDL.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <vector>

#include "logic.hpp"

using namespace tn;

namespace {

bool near(double a, double b, double eps = 1e-9) { return std::fabs(a - b) < eps; }

/// Tiny deterministic generator so the property tests do not depend on the platform's rand().
struct Lcg {
    std::uint64_t s = 12345;
    double next() {
        s = s * 6364136223846793005ULL + 1442695040888963407ULL;
        return static_cast<double>(s >> 11) / 9007199254740992.0;  // [0, 1)
    }
};

void fadeCurves() {
    for (Fade f : {Fade::Linear, Fade::Cubic, Fade::Quintic}) {
        assert(near(applyFade(f, 0.0), 0.0));
        assert(near(applyFade(f, 1.0), 1.0));
        assert(near(applyFade(f, 0.5), 0.5));  // all three are symmetric around the middle
    }
    // 6t^5 - 15t^4 + 10t^3 at 0.25 = 6/1024 - 15/256 + 10/64 = 0.103515625
    assert(near(applyFade(Fade::Quintic, 0.25), 0.103515625));
    // 3t^2 - 2t^3 at 0.25 = 3/16 - 2/64 = 0.15625
    assert(near(applyFade(Fade::Cubic, 0.25), 0.15625));
    assert(near(applyFade(Fade::Linear, 0.25), 0.25));
    // Quintic leaves 0 with zero slope AND zero curvature, cubic only with zero slope: near 0 it is far flatter.
    assert(applyFade(Fade::Quintic, 0.01) < 0.1 * applyFade(Fade::Cubic, 0.01));
    // The quintic is also flat at 1: it arrives with the same slope the neighbouring cell leaves with (0).
    assert(1.0 - applyFade(Fade::Quintic, 0.99) < 0.1 * (1.0 - applyFade(Fade::Cubic, 0.99)));
}

void hashing() {
    assert(latticeHash(3, 4, 7) == latticeHash(3, 4, 7));
    assert(latticeHash(3, 4, 7) != latticeHash(3, 4, 8));  // seed matters
    assert(latticeHash(3, 4, 7) != latticeHash(4, 3, 7));  // x and y are not interchangeable
    assert(latticeHash(-1, 0, 7) != latticeHash(1, 0, 7));
    double sum = 0.0;
    for (int y = -50; y < 50; ++y) {
        for (int x = -50; x < 50; ++x) {
            const double v = latticeValue(x, y, 3);
            assert(v >= 0.0 && v < 1.0);
            sum += v;
        }
    }
    assert(near(sum / 10000.0, 0.5, 0.02));  // roughly uniform
}

void valueNoiseBasics() {
    // On a lattice point the noise is exactly the lattice value, for every fade curve.
    for (Fade f : {Fade::Linear, Fade::Cubic, Fade::Quintic}) {
        assert(near(valueNoise(3.0, 4.0, 5, f), latticeValue(3, 4, 5)));
        assert(near(valueNoise(-2.0, -7.0, 5, f), latticeValue(-2, -7, 5)));
        // The middle of a cell is the average of its four corners (every fade gives 0.5 at 0.5).
        const double avg = (latticeValue(1, 2, 9) + latticeValue(2, 2, 9) + latticeValue(1, 3, 9) + latticeValue(2, 3, 9)) / 4.0;
        assert(near(valueNoise(1.5, 2.5, 9, f), avg));
    }
    // Negative coordinates use floor: -0.5 lies in the cell from -1 to 0, halfway between its two corners.
    const double mid = (latticeValue(-1, 0, 2) + latticeValue(0, 0, 2)) / 2.0;
    assert(near(valueNoise(-0.5, 0.0, 2, Fade::Quintic), mid));
    // Continuous across a cell border, also across 0.
    assert(near(valueNoise(1.0 - 1e-9, 0.37, 4, Fade::Quintic), valueNoise(1.0 + 1e-9, 0.37, 4, Fade::Quintic), 1e-6));
    assert(near(valueNoise(-1e-9, 0.37, 4, Fade::Quintic), valueNoise(1e-9, 0.37, 4, Fade::Quintic), 1e-6));
}

void valueNoiseStaysBetweenItsCorners() {
    Lcg rng;
    for (int i = 0; i < 20000; ++i) {
        const double x = rng.next() * 40.0 - 20.0;
        const double y = rng.next() * 40.0 - 20.0;
        const int ix = static_cast<int>(std::floor(x));
        const int iy = static_cast<int>(std::floor(y));
        const double c[4] = {latticeValue(ix, iy, 1), latticeValue(ix + 1, iy, 1), latticeValue(ix, iy + 1, 1), latticeValue(ix + 1, iy + 1, 1)};
        double lo = 1.0;
        double hi = 0.0;
        for (double v : c) {
            lo = std::min(lo, v);
            hi = std::max(hi, v);
        }
        const double n = valueNoise(x, y, 1, Fade::Quintic);
        assert(n >= lo - 1e-12 && n <= hi + 1e-12);
    }
}

void gradientCellByHand() {
    // Gradients (1,0) on the left corners and (-1,0) on the right corners all point towards the cell's middle.
    const Vec2 right{1.0, 0.0};
    const Vec2 left{-1.0, 0.0};
    // At (0.5, 0.5) each corner contributes 0.5 (offset 0.5 along the gradient), so the result is 0.5.
    assert(near(gradientCell(right, left, right, left, 0.5, 0.5, Fade::Linear), 0.5));
    assert(near(gradientCell(right, left, right, left, 0.5, 0.5, Fade::Quintic), 0.5));
    // At (0.25, 0.5): the left corners give 0.25, the right ones 0.75; the blend weight is fade(0.25).
    assert(near(gradientCell(right, left, right, left, 0.25, 0.5, Fade::Linear), 0.25 + 0.5 * 0.25));
    assert(near(gradientCell(right, left, right, left, 0.25, 0.5, Fade::Quintic), 0.25 + 0.5 * 0.103515625));
    // Gradients that point the same way everywhere cancel in the middle: (1,0) at 00, 01 and 10, 11 alike gives
    // dots 0.5, -0.5, 0.5, -0.5 and so 0.
    assert(near(gradientCell(right, right, right, right, 0.5, 0.5, Fade::Quintic), 0.0));
    // In the corner (0,0) the own corner has offset 0 and all other weights are 0.
    assert(near(gradientCell(right, left, right, left, 0.0, 0.0, Fade::Quintic), 0.0));
}

void gradientNoiseBasics() {
    for (int s = 0; s < 4; ++s) {
        for (int y = -3; y <= 3; ++y) {
            for (int x = -3; x <= 3; ++x) {
                assert(near(gradientNoise(x, y, s, Fade::Quintic), 0.0));  // zero on every lattice point
            }
        }
    }
    // Unit gradients bound the raw noise by sqrt(2)/2, scaled by sqrt(2) the result stays inside [-1, 1].
    Lcg rng;
    double lo = 1.0;
    double hi = -1.0;
    for (int i = 0; i < 100000; ++i) {
        const double x = rng.next() * 200.0 - 100.0;
        const double y = rng.next() * 200.0 - 100.0;
        const int seed = static_cast<int>(rng.next() * 100.0);
        for (Fade f : {Fade::Linear, Fade::Cubic, Fade::Quintic}) {
            const double n = gradientNoise(x, y, seed, f);
            assert(n >= -1.0 - 1e-12 && n <= 1.0 + 1e-12);
            lo = std::min(lo, n);
            hi = std::max(hi, n);
        }
    }
    assert(lo < -0.5 && hi > 0.5);  // it really uses the range, not a flat zero
    // Continuous across a cell border.
    assert(near(gradientNoise(2.0 - 1e-9, 0.31, 6, Fade::Quintic), gradientNoise(2.0 + 1e-9, 0.31, 6, Fade::Quintic), 1e-6));
}

void hostileInput() {
    const double inf = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (double v : {1e30, -1e30, inf, -inf, nan}) {
        const double a = valueNoise(v, v, 1, Fade::Quintic);
        const double b = gradientNoise(v, -v, 1, Fade::Quintic);
        assert(std::isfinite(a) && a >= 0.0 && a <= 1.0);
        assert(std::isfinite(b) && b >= -1.0 && b <= 1.0);
    }
}

void octaveSums() {
    assert(near(amplitudeSum(1, 0.5), 1.0));
    assert(near(amplitudeSum(3, 0.5), 1.75));  // 1 + 0.5 + 0.25
    assert(near(amplitudeSum(4, 1.0), 4.0));
    assert(near(amplitudeSum(3, 0.0), 1.0));
    assert(near(amplitudeSum(0, 0.5), 1.0));    // octave count is clamped to at least 1
    assert(near(amplitudeSum(99, 0.0), 1.0));   // and at most kMaxOctaves
    assert(near(amplitudeSum(99, 1.0), static_cast<double>(kMaxOctaves)));
    assert(near(amplitudeSum(2, -3.0), 1.0));   // persistence is clamped to [0, 1], so the sum can never be 0
    assert(near(amplitudeSum(2, std::numeric_limits<double>::quiet_NaN()), 1.0));
}

void fbmByHand() {
    Params p;
    p.octaves = 3;
    p.persistence = 0.5;
    p.lacunarity = 2.5;
    p.frequency = 4.0;
    p.normalize = true;
    // A constant source of 0.5: normalized stays 0.5 whatever the octaves, raw is 0.5 * (1 + 0.5 + 0.25).
    auto flat = [](double, double, int) { return 0.5; };
    assert(near(fbm(flat, 1.0, 1.0, p), 0.5));
    p.normalize = false;
    assert(near(fbm(flat, 1.0, 1.0, p), 0.875));
    // Frequencies per octave: 4, 4 * 2.5, 4 * 2.5^2 (position 1 makes the argument equal to the frequency).
    std::vector<double> seenX;
    std::vector<int> seenOctave;
    auto spy = [&](double x, double, int octave) {
        seenX.push_back(x);
        seenOctave.push_back(octave);
        return 0.0;
    };
    fbm(spy, 1.0, 1.0, p);
    assert(seenX.size() == 3);
    assert(near(seenX[0], 4.0) && near(seenX[1], 10.0) && near(seenX[2], 25.0));
    assert(seenOctave[0] == 0 && seenOctave[1] == 1 && seenOctave[2] == 2);
    // Amplitudes: a source that returns 1 only in octave 1 contributes exactly persistence^1.
    auto onlySecond = [](double, double, int octave) { return octave == 1 ? 1.0 : 0.0; };
    assert(near(fbm(onlySecond, 0.3, 0.3, p), 0.5));
    p.normalize = true;
    assert(near(fbm(onlySecond, 0.3, 0.3, p), 0.5 / 1.75));
}

void octavesAreDecorrelated() {
    // Every octave gets its own seed, otherwise every octave would be a compressed copy of the first (the same
    // lattice index would give the same value in each of them).
    assert(octaveSeed(5, 0) != octaveSeed(5, 1));
    assert(octaveSeed(5, 1) != octaveSeed(5, 2));
    assert(octaveSeed(5, 0) == 5);
    assert(valueNoise(3.0, 3.0, octaveSeed(5, 0), Fade::Linear) != valueNoise(3.0, 3.0, octaveSeed(5, 1), Fade::Linear));
}

void normalizedHeightsStayInUnitRange() {
    Lcg rng;
    bool valueClips = false;
    bool gradientClips = false;
    for (int i = 0; i < 5000; ++i) {
        Params p;
        p.octaves = 1 + static_cast<int>(rng.next() * kMaxOctaves);
        p.lacunarity = 1.0 + rng.next() * 3.0;
        p.persistence = rng.next();
        p.frequency = 1.0 + rng.next() * 15.0;
        p.seed = static_cast<int>(rng.next() * 100.0);
        p.fade = static_cast<Fade>(static_cast<int>(rng.next() * 3.0));
        const double x = rng.next() * 3.0 - 1.0;
        const double y = rng.next() * 3.0 - 1.0;
        for (Kind k : {Kind::Value, Kind::Gradient}) {
            p.normalize = true;
            const double h = heightAt(p, k, x, y);
            assert(h >= -1e-12 && h <= 1.0 + 1e-12);
            p.normalize = false;
            const double raw = heightAt(p, k, x, y);
            if (raw > 1.0) (k == Kind::Value ? valueClips : gradientClips) = true;
            assert(raw >= -1e-12 && raw <= amplitudeSum(p.octaves, p.persistence) + 1e-12);
        }
    }
    assert(valueClips && gradientClips);  // without normalizing, the sum really does leave [0, 1]
    Params one;
    one.octaves = 1;
    one.seed = 4;
    one.frequency = 2.0;
    // One octave is exactly the single noise, mapped from [-1, 1] to [0, 1] for gradient noise.
    assert(near(heightAt(one, Kind::Value, 0.3, 0.8), valueNoise(0.6, 1.6, 4, one.fade)));
    assert(near(heightAt(one, Kind::Gradient, 0.3, 0.8), 0.5 + 0.5 * gradientNoise(0.6, 1.6, 4, one.fade)));
}

void layers() {
    Params p;
    p.frequency = 2.0;
    p.lacunarity = 3.0;
    p.seed = 11;
    // Octave 0 is the base layer, octave 2 runs at 2 * 3 * 3 = 18 with its own seed.
    assert(near(layerAt(p, Kind::Value, 0, 0.4, 0.9), valueNoise(0.8, 1.8, 11, p.fade)));
    assert(near(layerAt(p, Kind::Value, 2, 0.4, 0.9), valueNoise(7.2, 16.2, octaveSeed(11, 2), p.fade)));
    assert(near(layerAt(p, Kind::Gradient, 1, 0.4, 0.9), 0.5 + 0.5 * gradientNoise(2.4, 5.4, octaveSeed(11, 1), p.fade)));
    // The first layer of the sum is the sum with a single octave.
    p.octaves = 1;
    assert(near(layerAt(p, Kind::Gradient, 0, 0.3, 0.6), heightAt(p, Kind::Gradient, 0.3, 0.6)));
}

void coordinates() {
    assert(near(unitCoord(0.0, 0.0, 280.0), 0.0));
    assert(near(unitCoord(280.0, 0.0, 280.0), 1.0));
    assert(near(unitCoord(0.0, 140.0, 280.0), 0.5));
    assert(near(unitCoord(70.0, -70.0, 280.0), 0.0));
    assert(near(unitCoord(10.0, 5.0, 0.0), 0.0));  // size 0 does not divide
    double px = 0.0;
    double py = 0.0;
    panBy(px, py, -30.0, 20.0);  // dragging the terrain by (dx, dy) moves the view by (-dx, -dy)
    assert(near(px, 30.0) && near(py, -20.0));
    panBy(px, py, -1e9, 1e9);
    assert(near(px, kPanLimit) && near(py, -kPanLimit));
}

void sliders() {
    assert(near(sliderValue(0.0, 1.0, 8.0, 1.0), 1.0));
    assert(near(sliderValue(1.0, 1.0, 8.0, 1.0), 8.0));
    assert(near(sliderValue(0.5, 0.0, 10.0, 1.0), 5.0));
    assert(near(sliderValue(0.5, 0.1, 0.9, 0.05), 0.5));
    assert(near(sliderValue(0.26, 0.0, 10.0, 1.0), 3.0));  // snaps to the step grid (2.6 rounds to 3)
    assert(near(sliderValue(-5.0, 1.0, 8.0, 1.0), 1.0));   // outside the track is clamped
    assert(near(sliderValue(5.0, 1.0, 8.0, 1.0), 8.0));
    assert(near(sliderValue(0.3, 2.0, 4.0, 0.0), 2.6));    // step 0 means continuous
    assert(near(tOfValue(1.0, 1.0, 8.0), 0.0));
    assert(near(tOfValue(8.0, 1.0, 8.0), 1.0));
    assert(near(tOfValue(4.5, 1.0, 8.0), 0.5));
    assert(near(tOfValue(3.0, 3.0, 3.0), 0.0));  // empty range does not divide
}

void colors() {
    const Rgb water = terrainColor(0.0);
    const Rgb snow = terrainColor(1.0);
    assert(water.b > water.r && water.b > 60);          // water is blue-ish
    assert(snow.r > 230 && snow.g > 230 && snow.b > 230);  // snow is almost white
    const Rgb grass = terrainColor(0.55);
    assert(grass.g > grass.r && grass.g > grass.b);
    // Outside [0, 1] the ramp clamps instead of reading past its ends.
    assert(terrainColor(-3.0).r == water.r && terrainColor(7.0).r == snow.r);
    assert(grayColor(0.0).r == 0 && grayColor(1.0).r == 255 && grayColor(0.5).g == 128);
    assert(grayColor(-1.0).r == 0 && grayColor(2.0).r == 255);
    // Shading: flat ground keeps its color, slopes facing the light get brighter, away darker, always bounded.
    assert(near(shadeFactor(0.0), 1.0));
    assert(shadeFactor(0.02) > 1.0 && shadeFactor(-0.02) < 1.0);
    assert(shadeFactor(1e9) <= 1.4 + 1e-12 && shadeFactor(-1e9) >= 0.6 - 1e-12);
    const Rgb lit = shade(Rgb{100, 100, 100}, 1.0);
    assert(lit.r == 100 && lit.g == 100 && lit.b == 100);
    assert(shade(Rgb{200, 200, 200}, 1.4).r == 255);  // clamped at 255, no wrap-around
    assert(shade(Rgb{100, 100, 100}, 0.5).r == 50);
}

}  // namespace

int main() {
    fadeCurves();
    hashing();
    valueNoiseBasics();
    valueNoiseStaysBetweenItsCorners();
    gradientCellByHand();
    gradientNoiseBasics();
    hostileInput();
    octaveSums();
    fbmByHand();
    octavesAreDecorrelated();
    normalizedHeightsStayInUnitRange();
    layers();
    coordinates();
    sliders();
    colors();
    std::puts("terrain-fbm logic: all asserts passed");
    return 0;
}
