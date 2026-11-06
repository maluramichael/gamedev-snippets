// Tests for the turing-patterns logic. Plain asserts on hand-computed cases, no framework, no SDL.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "logic.hpp"

using namespace tp;

namespace {

bool near(double a, double b, double eps = 1e-5) { return std::fabs(a - b) < eps; }

/// A field with a constant A and B everywhere.
Field uniform(int w, int h, float a, float b) {
    Field f(w, h);
    std::fill(f.a.begin(), f.a.end(), a);
    std::fill(f.b.begin(), f.b.end(), b);
    return f;
}

bool allFiniteAndInRange(const Field& f) {
    for (std::size_t i = 0; i < f.a.size(); ++i) {
        if (!std::isfinite(f.a[i]) || !std::isfinite(f.b[i])) return false;
        if (f.a[i] < 0.0F || f.a[i] > 1.0F || f.b[i] < 0.0F || f.b[i] > 1.0F) return false;
    }
    return true;
}

double maxDeviation(const Field& f, float center) {
    double worst = 0.0;
    for (float v : f.a) worst = std::max(worst, std::fabs(static_cast<double>(v) - center));
    return worst;
}

// The kernel is -1 in the middle, 0.2 for the four direct neighbours and 0.05 for the four diagonals: the weights
// sum to zero, so a flat field has no diffusion, and the worst (checkerboard) mode has eigenvalue -1.6.
void laplaceKernel() {
    const Field flat = uniform(6, 5, 0.7F, 0.2F);
    for (int y = 0; y < 5; ++y) {
        for (int x = 0; x < 6; ++x) {
            assert(near(laplaceA(flat, x, y), 0.0));
            assert(near(laplaceB(flat, x, y), 0.0));
        }
    }
    // One spike of 1 at (2,2) on a 6x5 grid: centre -1, direct neighbours 0.2, diagonal ones 0.05, others 0.
    Field spike = uniform(6, 5, 0.0F, 0.0F);
    spike.a[2 * 6 + 2] = 1.0F;
    assert(near(laplaceA(spike, 2, 2), -1.0));
    assert(near(laplaceA(spike, 3, 2), 0.2));
    assert(near(laplaceA(spike, 2, 1), 0.2));
    assert(near(laplaceA(spike, 3, 3), 0.05));
    assert(near(laplaceA(spike, 4, 2), 0.0));  // two cells away does not feel the spike
    // The grid wraps: a spike in the corner (0,0) reaches (5,0), (0,4) and the opposite corner (5,4) diagonally.
    Field corner = uniform(6, 5, 0.0F, 0.0F);
    corner.a[0] = 1.0F;
    assert(near(laplaceA(corner, 5, 0), 0.2));
    assert(near(laplaceA(corner, 0, 4), 0.2));
    assert(near(laplaceA(corner, 5, 4), 0.05));
    assert(near(laplaceA(corner, 1, 1), 0.05));
}

void worstEigenvalueAndStepLimit() {
    // Checkerboard of A around 0.5, no B, no feed: only diffusion acts. The checkerboard is multiplied by
    // (1 - 1.6 * dA * dt) every step, so dt = 1 shrinks it to 0.06 and dt = 1.5 grows it to 0.14.
    auto checker = [] {
        Field f = uniform(8, 8, 0.5F, 0.0F);
        for (int y = 0; y < 8; ++y) {
            for (int x = 0; x < 8; ++x) f.a[static_cast<std::size_t>(y * 8 + x)] += ((x + y) % 2 == 0 ? 0.1F : -0.1F);
        }
        return f;
    };
    Params p;
    p.feed = 0.0F;
    p.kill = 0.0F;
    Field good = checker();
    p.dt = 1.0F;
    step(good, p, Update::Buffered);
    assert(near(maxDeviation(good, 0.5F), 0.06));
    Field bad = checker();
    p.dt = 1.5F;
    step(bad, p, Update::Buffered);
    assert(near(maxDeviation(bad, 0.5F), 0.14));
    // Many steps: below the limit the checkerboard dies out, above it it grows until the clamp stops it.
    Field calm = checker();
    p.dt = 1.0F;
    for (int i = 0; i < 10; ++i) step(calm, p, Update::Buffered);
    assert(maxDeviation(calm, 0.5F) < 0.001);  // 0.1 * 0.6^10 = 0.0006
    Field wild = checker();
    p.dt = 1.5F;
    for (int i = 0; i < 10; ++i) step(wild, p, Update::Buffered);
    assert(maxDeviation(wild, 0.5F) > 0.3);
    assert(allFiniteAndInRange(wild));
    // The limit belongs to the buffered update. In place, a cell already sees the new value of its left and upper
    // neighbours, which damps the checkerboard: the same dt = 1.5 stays calm there (measured, 0.0018 after 10 steps).
    Field inPlace = checker();
    for (int i = 0; i < 10; ++i) step(inPlace, p, Update::InPlace);
    assert(maxDeviation(inPlace, 0.5F) < 0.01);
    // The limit itself: 2 / (1.6 * max(dA, dB)) = 1.25 for the default diffusion rates.
    assert(near(maxStableDt(Params{}), 1.25));
    Params fast;
    fast.dA = 0.5F;
    fast.dB = 2.0F;
    assert(near(maxStableDt(fast), 0.625));
    Params none;
    none.dA = 0.0F;
    none.dB = 0.0F;
    assert(maxStableDt(none) > 1e6F);  // no diffusion, no diffusion limit, and no division by zero
}

void reactionOnly() {
    // A flat field has no diffusion, so one step is the reaction alone. A = 0.5, B = 0.25, f = 0.04, k = 0.06:
    //   A' = 0.5 - 0.5 * 0.0625 + 0.04 * 0.5          = 0.48875
    //   B' = 0.25 + 0.5 * 0.0625 - (0.06 + 0.04) * 0.25 = 0.25625
    Field f = uniform(5, 5, 0.5F, 0.25F);
    Params p;
    p.feed = 0.04F;
    p.kill = 0.06F;
    step(f, p, Update::Buffered);
    for (std::size_t i = 0; i < f.a.size(); ++i) {
        assert(near(f.a[i], 0.48875));
        assert(near(f.b[i], 0.25625));
    }
    // The empty state (A = 1, B = 0) is a fixed point: nothing grows out of nothing.
    Field empty = uniform(5, 5, 1.0F, 0.0F);
    for (int i = 0; i < 20; ++i) step(empty, p, Update::Buffered);
    for (std::size_t i = 0; i < empty.a.size(); ++i) assert(empty.a[i] == 1.0F && empty.b[i] == 0.0F);
}

void diffusionOnly() {
    // A = 0 everywhere, so A * B^2 is 0 and f = k = 0 switches the rest off: B only diffuses. One cell of B = 1 on a
    // 7x7 grid: centre 1 + dB * (-1) = 0.5, direct neighbours 0.5 * 0.2 = 0.1, diagonal ones 0.5 * 0.05 = 0.025.
    Field f = uniform(7, 7, 0.0F, 0.0F);
    f.b[3 * 7 + 3] = 1.0F;
    Params p;
    p.feed = 0.0F;
    p.kill = 0.0F;
    step(f, p, Update::Buffered);
    assert(near(f.b[3 * 7 + 3], 0.5));
    assert(near(f.b[3 * 7 + 4], 0.1));
    assert(near(f.b[2 * 7 + 3], 0.1));
    assert(near(f.b[2 * 7 + 2], 0.025));
    assert(near(f.b[3 * 7 + 5], 0.0));
    // The total of B is kept.
    double total = 0.0;
    for (float v : f.b) total += v;
    assert(near(total, 1.0));
}

void diffusionKeepsTheTotal() {
    Field f(16, 12);
    for (int y = 0; y < 12; ++y) {
        for (int x = 0; x < 16; ++x) f.a[static_cast<std::size_t>(y * 16 + x)] = 0.2F + 0.6F * static_cast<float>((x * 7 + y * 13) % 10) / 9.0F;
    }
    auto sum = [](const Field& g) {
        double s = 0.0;
        for (float v : g.a) s += v;
        return s;
    };
    const double before = sum(f);
    Params p;
    p.feed = 0.0F;
    p.kill = 0.0F;
    for (int i = 0; i < 30; ++i) step(f, p, Update::Buffered);
    assert(near(sum(f), before, 1e-3));
}

void steadyStatesOfTheReaction() {
    // dA = -A B^2 + f (1 - A) = 0 and dB = A B^2 - (f + k) B = 0 give (f + k) B^2 - f B + f (f + k) = 0, which has a
    // real root only for k <= sqrt(f) / 2 - f.
    assert(near(killLimit(0.0625F), 0.0625));  // sqrt = 0.25
    assert(near(killLimit(0.04F), 0.06));      // sqrt = 0.2
    assert(near(killLimit(0.09F), 0.06));      // sqrt = 0.3
    assert(killLimit(0.0F) == 0.0F);
    assert(killLimit(-1.0F) == 0.0F);

    Steady s[2];
    // Exactly on the limit the two roots merge: B = f / (2 (f + k)) = 0.25, A = (f + k) / B = 0.5.
    assert(steadyStates(0.0625F, 0.0625F, s) == 1);
    assert(near(s[0].a, 0.5) && near(s[0].b, 0.25));
    // Beyond the limit there is no uniform state with B in it.
    assert(steadyStates(0.0625F, 0.07F, s) == 0);
    assert(steadyStates(0.0F, 0.05F, s) == 0);
    // Below it there are two, the one with more B first. f = 0.0625, k = 0.05: B = 0.39886 and 0.15669.
    assert(steadyStates(0.0625F, 0.05F, s) == 2);
    assert(near(s[0].b, 0.39886, 1e-4) && near(s[1].b, 0.15669, 1e-4));
    assert(s[0].b > s[1].b);
    // Both really are fixed points of the update.
    Params p;
    p.feed = 0.0625F;
    p.kill = 0.05F;
    for (const Steady& st : s) {
        Field f = uniform(4, 4, st.a, st.b);
        step(f, p, Update::Buffered);
        assert(near(f.a[5], st.a) && near(f.b[5], st.b));
    }
}

void bufferedKeepsTheSymmetry() {
    // A block of B in the middle of a 24x16 grid is mirror symmetric left-right and up-down. Reading the old values
    // only (buffered) keeps it that way, updating in place does not: later cells already see the new values of the
    // earlier ones, so the result depends on the scan direction.
    auto build = [] {
        Field f(24, 16);
        for (int y = 6; y < 10; ++y) {
            for (int x = 10; x < 14; ++x) {
                f.a[static_cast<std::size_t>(y * 24 + x)] = 0.5F;
                f.b[static_cast<std::size_t>(y * 24 + x)] = 1.0F;
            }
        }
        return f;
    };
    auto asymmetry = [](const Field& f) {
        double worst = 0.0;
        for (int y = 0; y < f.h; ++y) {
            for (int x = 0; x < f.w; ++x) {
                const int mx = f.w - 1 - x;
                const int my = f.h - 1 - y;
                const auto i = static_cast<std::size_t>(y * f.w + x);
                worst = std::max(worst, std::fabs(static_cast<double>(f.b[i]) - f.b[static_cast<std::size_t>(y * f.w + mx)]));
                worst = std::max(worst, std::fabs(static_cast<double>(f.b[i]) - f.b[static_cast<std::size_t>(my * f.w + x)]));
            }
        }
        return worst;
    };
    Params p;
    Field buffered = build();
    Field inPlace = build();
    assert(asymmetry(buffered) == 0.0);
    for (int i = 0; i < 60; ++i) {
        step(buffered, p, Update::Buffered);
        step(inPlace, p, Update::InPlace);
    }
    assert(asymmetry(buffered) < 1e-4);
    assert(asymmetry(inPlace) > 1e-2);
    assert(allFiniteAndInRange(inPlace));
}

void stayBoundedAndFinite() {
    // Every preset at the normal step, many steps from a seeded field: always inside [0, 1], never NaN.
    for (const Preset& pr : kPresets) {
        Field f(48, 32);
        seed(f, 7, 6, 4);
        Params p;
        p.feed = pr.feed;
        p.kill = pr.kill;
        for (int i = 0; i < 400; ++i) step(f, p, Update::Buffered);
        assert(allFiniteAndInRange(f));
    }
    // Far above the diffusion limit the field is garbage, but it stays inside the clamp.
    Field f(32, 32);
    seed(f, 3, 8, 4);
    Params wild;
    wild.dt = 4.0F;
    for (int i = 0; i < 200; ++i) step(f, wild, Update::Buffered);
    assert(allFiniteAndInRange(f));
    // A NaN that sneaks in (a bad value from outside) is wiped out by the clamp and does not spread.
    Field g = uniform(8, 8, 1.0F, 0.0F);
    g.b[20] = std::nanf("");
    step(g, Params{}, Update::Buffered);
    assert(allFiniteAndInRange(g));
    step(g, Params{}, Update::InPlace);
    assert(allFiniteAndInRange(g));
}

void seeding() {
    Field a(40, 30);
    Field b(40, 30);
    seed(a, 5, 6, 3);
    seed(b, 5, 6, 3);
    assert(a.a == b.a && a.b == b.b);  // same seed, same field
    Field c(40, 30);
    seed(c, 6, 6, 3);
    assert(c.b != a.b);
    int seeded = 0;
    for (std::size_t i = 0; i < a.b.size(); ++i) {
        if (a.b[i] > 0.0F) {
            ++seeded;
            assert(a.b[i] == 1.0F && a.a[i] == 0.5F);
        } else {
            assert(a.a[i] == 1.0F);
        }
    }
    assert(seeded >= 9 && seeded <= 6 * 9);  // six 3x3 blocks, overlaps only make it fewer
    // Degenerate input does not crash and leaves a clean field.
    Field d(10, 10);
    seed(d, 1, 0, 3);
    seed(d, 1, 5, 0);
    seed(d, 1, 5, -2);
    assert(coverage(d, 0.1F) == 0.0F);
    Field tiny(3, 3);
    seed(tiny, 1, 4, 50);  // a block larger than the grid is cut to the grid
    assert(allFiniteAndInRange(tiny));
}

void planting() {
    // One seed is a block in the middle: 16x16 on 160x105 starts at x = 72, y = 44 (the odd row is cut towards the top).
    Field f(160, 105);
    f.b[0] = 0.7F;  // old content is cleared
    plant(f, Preset{"x", 0.03F, 0.06F, 1, 16}, 9);
    int blocks = 0;
    for (float v : f.b) blocks += v > 0.0F ? 1 : 0;
    assert(blocks == 256);
    assert(f.b[f.index(72, 44)] == 1.0F && f.b[f.index(87, 59)] == 1.0F);
    assert(f.b[f.index(71, 44)] == 0.0F && f.b[f.index(88, 44)] == 0.0F);
    assert(f.b[f.index(72, 43)] == 0.0F && f.b[f.index(72, 60)] == 0.0F);
    assert(f.b[0] == 0.0F);
    // Several seeds go through seed(), the same seed value gives the same field.
    Field g(160, 105);
    Field h(160, 105);
    plant(g, kPresets[0], 4);
    plant(h, kPresets[0], 4);
    assert(g.b == h.b);
    assert(coverage(g, 0.5F) > 0.0F);
    // A block larger than the grid is cut to it.
    Field tiny(5, 5);
    plant(tiny, Preset{"x", 0.03F, 0.06F, 1, 40}, 1);
    assert(coverage(tiny, 0.5F) == 1.0F);
}

void painting() {
    Field f(8, 8);
    paint(f, 0, 0, 1);  // radius 1: the centre and its four direct neighbours, wrapped around the edges
    int painted = 0;
    for (std::size_t i = 0; i < f.b.size(); ++i) {
        if (f.b[i] == 1.0F) {
            ++painted;
            assert(f.a[i] == 0.5F);
        }
    }
    assert(painted == 5);
    assert(f.b[0] == 1.0F && f.b[1] == 1.0F && f.b[7] == 1.0F && f.b[8] == 1.0F && f.b[56] == 1.0F);
    assert(f.b[9] == 0.0F);  // (1,1) is a diagonal neighbour, outside the disc of radius 1
    erase(f, 0, 0, 0);       // radius 0 is just the centre cell
    assert(f.b[0] == 0.0F && f.a[0] == 1.0F && f.b[1] == 1.0F);
    Field g(8, 8);
    paint(g, -1, 3, 2);  // centre outside the field: ignored
    paint(g, 8, 3, 2);
    paint(g, 3, 3, -1);
    assert(coverage(g, 0.1F) == 0.0F);
}

void coverageAndLevel() {
    Field f(4, 2);
    f.b[0] = 0.5F;
    f.b[5] = 0.11F;
    f.b[3] = 0.1F;  // not above the threshold
    assert(near(coverage(f, 0.1F), 0.25));
    assert(coverage(Field(0, 0), 0.1F) == 0.0F);
    assert(near(level(1.0F, 0.0F), 0.0));  // empty: paper
    assert(near(level(0.0F, 1.0F), 1.0));
    assert(near(level(0.75F, 0.25F), 0.5));
    assert(near(level(0.5F, 0.5F), 1.0));
    assert(near(level(2.0F, -1.0F), 0.0));  // out of range input is clamped
    assert(level(std::nanf(""), 0.0F) == 0.0F);
}

void parameterMap() {
    // The picker maps k to x and f to y (f grows upwards). Rect: x 100..300, y 50..150.
    const MapRect r{100.0F, 50.0F, 200.0F, 100.0F};
    float f = 0.0F;
    float k = 0.0F;
    mapToParams(r, 100.0F, 150.0F, f, k);  // bottom left: smallest f, smallest k
    assert(near(f, 0.01) && near(k, 0.045));
    mapToParams(r, 300.0F, 50.0F, f, k);  // top right
    assert(near(f, 0.1) && near(k, 0.07));
    mapToParams(r, 200.0F, 100.0F, f, k);  // middle
    assert(near(f, 0.055) && near(k, 0.0575));
    mapToParams(r, -50.0F, 900.0F, f, k);  // outside is clamped to the range
    assert(near(f, 0.01) && near(k, 0.045));
    mapToParams(r, 1000.0F, -900.0F, f, k);
    assert(near(f, 0.1) && near(k, 0.07));
    float x = 0.0F;
    float y = 0.0F;
    paramsToMap(r, 0.055F, 0.0575F, x, y);
    assert(near(x, 200.0) && near(y, 100.0));
    paramsToMap(r, 0.0F, 0.2F, x, y);  // out of range values stay on the rect
    assert(near(x, 300.0) && near(y, 150.0));
    // Round trip for a point in the middle of the preset area (Karl Sims' coral values).
    paramsToMap(r, 0.0545F, 0.062F, x, y);
    mapToParams(r, x, y, f, k);
    assert(near(f, 0.0545) && near(k, 0.062));
    // A degenerate rect does not divide by zero.
    mapToParams(MapRect{0.0F, 0.0F, 0.0F, 0.0F}, 5.0F, 5.0F, f, k);
    assert(std::isfinite(f) && std::isfinite(k));
}

void presetsAreInRange() {
    for (const Preset& pr : kPresets) {
        assert(pr.feed >= kFeedMin && pr.feed <= kFeedMax);
        assert(pr.kill >= kKillMin && pr.kill <= kKillMax);
        assert(pr.name != nullptr && pr.name[0] != '\0');
        assert(pr.seeds >= 1 && pr.seedSize >= 1);
    }
}

}  // namespace

int main() {
    laplaceKernel();
    worstEigenvalueAndStepLimit();
    reactionOnly();
    diffusionOnly();
    diffusionKeepsTheTotal();
    steadyStatesOfTheReaction();
    bufferedKeepsTheSymmetry();
    stayBoundedAndFinite();
    seeding();
    planting();
    painting();
    coverageAndLevel();
    parameterMap();
    presetsAreInRange();
    std::puts("turing-patterns logic: all tests passed");
    return 0;
}
