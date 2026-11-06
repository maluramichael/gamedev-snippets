// Tests for the terrain-erosion logic. Plain asserts on hand-computed cases, no framework, no SDL.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

#include "logic.hpp"

using namespace te;

namespace {

bool near(double a, double b, double eps = 1e-9) { return std::fabs(a - b) < eps; }

Grid filled(int w, int h, double value) {
    Grid g(w, h);
    for (double& v : g.v) v = value;
    return g;
}

/// A plane: height = base + sx * x + sy * y.
Grid plane(int w, int h, double base, double sx, double sy) {
    Grid g(w, h);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) g.at(x, y) = base + sx * x + sy * y;
    }
    return g;
}

double sumOf(const Grid& g) { return totalHeight(g); }

bool allFinite(const Grid& g) {
    for (double v : g.v) {
        if (!std::isfinite(v)) return false;
    }
    return true;
}

// ---------------------------------------------------------------- grid, random numbers, noise

void gridBasics() {
    Grid g(4, 3);
    assert(g.v.size() == 12);
    g.at(3, 2) = 7.0;
    assert(g.v[2 * 4 + 3] == 7.0);  // row-major
    assert(g.inside(0, 0) && g.inside(3, 2));
    assert(!g.inside(-1, 0) && !g.inside(4, 0) && !g.inside(0, 3));
    Grid empty(0, 0);
    assert(empty.v.empty());
    Grid negative(-5, 3);  // sizes below 0 become 0, no huge allocation
    assert(negative.w == 0 && negative.v.empty());
}

void randomNumbers() {
    Rng a(5);
    Rng b(5);
    Rng c(6);
    bool differs = false;
    double sum = 0.0;
    for (int i = 0; i < 10000; ++i) {
        const double x = a.next();
        assert(x >= 0.0 && x < 1.0);
        assert(x == b.next());  // same seed, same sequence
        if (x != c.next()) differs = true;
        sum += x;
    }
    assert(differs);
    assert(near(sum / 10000.0, 0.5, 0.02));
}

void noiseBasics() {
    // Gradient noise is exactly 0 on every lattice point (the offset to all corners it uses is 0 for its own corner
    // and the dot products with the other corners vanish by the fade weights 0).
    for (int y = -3; y <= 3; ++y) {
        for (int x = -3; x <= 3; ++x) assert(near(gradientNoise(x, y, 4), 0.0));
    }
    double lo = 1.0;
    double hi = -1.0;
    for (int i = 0; i < 4000; ++i) {
        const double x = i * 0.0371 - 70.0;
        const double y = i * 0.0213 + 5.0;
        const double v = gradientNoise(x, y, 4);
        assert(v >= -1.0 && v <= 1.0);
        lo = std::min(lo, v);
        hi = std::max(hi, v);
    }
    assert(lo < -0.3 && hi > 0.3);  // not constant
    assert(gradientNoise(1.3, 2.7, 4) == gradientNoise(1.3, 2.7, 4));
    assert(gradientNoise(1.3, 2.7, 4) != gradientNoise(1.3, 2.7, 5));
    // Hostile input: NaN and huge values are clamped, nothing throws or returns NaN.
    assert(std::isfinite(gradientNoise(std::numeric_limits<double>::quiet_NaN(), 1.0, 1)));
    assert(std::isfinite(gradientNoise(1e300, -1e300, 1)));
    assert(std::isfinite(gradientNoise(std::numeric_limits<double>::infinity(), 0.5, 1)));
}

void fbmAndTerrain() {
    for (int i = 0; i < 2000; ++i) {
        const double v = fbm(i * 0.0137, i * 0.0291, 3, 6, 2.5);
        assert(v >= 0.0 && v <= 1.0);
    }
    assert(fbm(0.4, 0.4, 3, 0, 2.5) == fbm(0.4, 0.4, 3, 1, 2.5));  // fewer than one octave means one
    assert(std::isfinite(fbm(0.4, 0.4, 3, 1000, 2.5)));            // octave count is capped
    // Persistence 0 keeps only the first octave: 0.5 + 0.5 * noise at the scaled position.
    assert(near(fbm(0.37, 0.61, 3, 5, 2.5, 0.0), 0.5 + 0.5 * gradientNoise(0.37 * 2.5, 0.61 * 2.5, 3)));
    // Persistence 1 weighs all octaves the same: the plain average of the five (octave i uses seed + i * 1013, f * 2^i).
    double avg = 0.0;
    for (int i = 0; i < 5; ++i) avg += 0.5 + 0.5 * gradientNoise(0.37 * 2.5 * std::pow(2.0, i), 0.61 * 2.5 * std::pow(2.0, i), 3 + i * 1013);
    assert(near(fbm(0.37, 0.61, 3, 5, 2.5, 1.0), avg / 5.0));
    assert(fbm(0.37, 0.61, 3, 5, 2.5, 7.0) == fbm(0.37, 0.61, 3, 5, 2.5, 1.0));  // persistence is clamped to 1

    Grid a(48, 48);
    generateTerrain(a, 7);
    double lo = 1e9;
    double hi = -1e9;
    for (double v : a.v) {
        lo = std::min(lo, v);
        hi = std::max(hi, v);
    }
    assert(lo == 0.0 && hi == 1.0);  // stretched to the full range
    Grid b(48, 48);
    generateTerrain(b, 7);
    assert(a.v == b.v);  // deterministic
    generateTerrain(b, 8);
    assert(a.v != b.v);

    // A constant grid cannot be stretched: it becomes flat 0, no division by zero.
    Grid flat = filled(5, 5, 3.0);
    normalizeToUnit(flat);
    for (double v : flat.v) assert(v == 0.0);
    Grid ramp = plane(5, 1, 10.0, 2.0, 0.0);  // 10 12 14 16 18
    normalizeToUnit(ramp);
    assert(near(ramp.at(0, 0), 0.0) && near(ramp.at(2, 0), 0.5) && near(ramp.at(4, 0), 1.0));
}

// ---------------------------------------------------------------- sampling

void sampling() {
    // 2x2 plane h = x + 2y: the corners are 0, 1 / 2, 3.
    const Grid g = plane(2, 2, 0.0, 1.0, 2.0);
    Sample s = sampleAt(g, 0.0, 0.0);
    assert(near(s.h, 0.0) && near(s.gx, 1.0) && near(s.gy, 2.0));
    s = sampleAt(g, 0.5, 0.5);
    assert(near(s.h, 1.5) && near(s.gx, 1.0) && near(s.gy, 2.0));  // a plane has the same gradient everywhere
    s = sampleAt(g, 0.25, 0.75);
    assert(near(s.h, 0.25 + 1.5));

    // A bilinear patch that is not a plane: corners 0, 1 / 1, 0 (a saddle). Middle = average = 0.5.
    Grid saddle(2, 2);
    saddle.at(0, 0) = 0.0;
    saddle.at(1, 0) = 1.0;
    saddle.at(0, 1) = 1.0;
    saddle.at(1, 1) = 0.0;
    s = sampleAt(saddle, 0.5, 0.5);
    assert(near(s.h, 0.5) && near(s.gx, 0.0) && near(s.gy, 0.0));
    s = sampleAt(saddle, 0.0, 0.0);  // gx = (h10 - h00)(1 - fy) + (h11 - h01) fy = 1, gy = 1
    assert(near(s.gx, 1.0) && near(s.gy, 1.0));
}

// ---------------------------------------------------------------- one step of a drop

void capacity() {
    DropParams p;
    p.capacity = 4.0;
    p.minSlope = 0.01;
    assert(near(capacityOf(-0.5, 2.0, 1.0, p), 0.5 * 2.0 * 1.0 * 4.0));  // steep downhill: slope * speed * water * factor
    assert(near(capacityOf(0.1, 2.0, 1.0, p), 0.01 * 2.0 * 1.0 * 4.0));  // uphill: the minimum slope counts
    assert(near(capacityOf(0.0, 1.0, 0.5, p), 0.01 * 1.0 * 0.5 * 4.0));  // flat: also the minimum slope
    assert(capacityOf(-0.5, 0.0, 1.0, p) == 0.0);                          // no speed, no capacity
    assert(capacityOf(-0.5, 2.0, -3.0, p) >= 0.0);                         // negative water never gives negative capacity
}

void stepChanges() {
    DropParams p;
    p.erode = 0.3;
    p.deposit = 0.3;
    // Carrying more than it can: it drops (5 - 4) * 0.3.
    Change c = stepChange(5.0, 4.0, -0.5, p);
    assert(c.erode == 0.0 && near(c.deposit, 0.3));
    // Below its capacity: it picks up (4 - 1) * 0.3 = 0.9, but never more than the drop in height (0.5).
    c = stepChange(1.0, 4.0, -0.5, p);
    assert(c.deposit == 0.0 && near(c.erode, 0.5));
    // With a gentler step the rate is the limit: (4 - 3.9) * 0.3 = 0.03 < 0.5.
    c = stepChange(3.9, 4.0, -0.5, p);
    assert(near(c.erode, 0.03));
    // Uphill: it drops min(dh, sediment), enough to fill the step, never more than it carries.
    c = stepChange(5.0, 4.0, 0.1, p);
    assert(c.erode == 0.0 && near(c.deposit, 0.1));
    c = stepChange(0.05, 4.0, 0.1, p);
    assert(near(c.deposit, 0.05));
    c = stepChange(0.0, 4.0, 0.1, p);
    assert(c.deposit == 0.0 && c.erode == 0.0);
    // Exactly at capacity: nothing to drop, nothing to pick up.
    c = stepChange(4.0, 4.0, -0.5, p);
    assert(c.deposit == 0.0 && c.erode == 0.0);
    // A flat step cannot be dug into.
    c = stepChange(0.0, 0.04, 0.0, p);
    assert(c.erode == 0.0 && c.deposit == 0.0);
}

// ---------------------------------------------------------------- spreading material over cells

void bilinearSplat() {
    Grid g = filled(6, 6, 0.0);
    addBilinear(g, 2.25, 3.5, 8.0);  // fx 0.25, fy 0.5: weights 0.375, 0.125 / 0.375, 0.125
    assert(near(g.at(2, 3), 3.0) && near(g.at(3, 3), 1.0) && near(g.at(2, 4), 3.0) && near(g.at(3, 4), 1.0));
    assert(near(sumOf(g), 8.0));
    // At the last row and column the neighbour does not exist: everything stays on the grid and nothing is lost.
    Grid e = filled(6, 6, 0.0);
    addBilinear(e, 5.0, 5.0, 2.0);
    assert(near(e.at(5, 5), 2.0) && near(sumOf(e), 2.0));
    addBilinear(e, 99.0, -4.0, 1.0);  // far outside: clamped onto the grid, still conserved
    assert(near(sumOf(e), 3.0));
}

void brush() {
    // In the middle of the grid the weights are max(0, 1 - d / 3) for the distance d to the centre, normalized.
    // Sum over the cells: 1 + 4 * 2/3 + 4 * (1 - sqrt2/3) + 4 * 1/3 + 8 * (1 - sqrt5/3) + 4 * (1 - sqrt8/3) = 9.380298
    const double total = 1.0 + 4.0 * (2.0 / 3.0) + 4.0 * (1.0 - std::sqrt(2.0) / 3.0) + 4.0 * (1.0 / 3.0) + 8.0 * (1.0 - std::sqrt(5.0) / 3.0) +
                         4.0 * (1.0 - std::sqrt(8.0) / 3.0);
    assert(near(total, 9.380298, 1e-6));
    Grid g = filled(21, 21, 0.0);
    addBrush(g, 10.0, 10.0, 1.0);
    assert(near(g.at(10, 10), 1.0 / total, 1e-12));
    assert(near(g.at(11, 10), (2.0 / 3.0) / total, 1e-12));
    assert(near(g.at(12, 10), (1.0 / 3.0) / total, 1e-12));
    assert(g.at(13, 10) == 0.0);  // distance 3: the brush ends before it
    assert(near(sumOf(g), 1.0, 1e-12));
    // Negative amounts take material away.
    addBrush(g, 10.0, 10.0, -1.0);
    for (double v : g.v) assert(near(v, 0.0, 1e-12));
    // In a corner a part of the disc is outside: the weights are renormalized, the amount is still handed out in full.
    Grid corner = filled(21, 21, 0.0);
    addBrush(corner, 0.0, 0.0, 1.0);
    assert(near(sumOf(corner), 1.0, 1e-12));
    assert(corner.at(0, 0) > corner.at(1, 0));
    addBrush(corner, 20.0, 20.0, 3.0);
    assert(near(sumOf(corner), 4.0, 1e-12));
    // Between cells (fractional centre) the weights still add up.
    Grid frac = filled(21, 21, 0.0);
    addBrush(frac, 7.3, 12.9, 5.0);
    assert(near(sumOf(frac), 5.0, 1e-12));
}

// ---------------------------------------------------------------- whole drops

void dropOnASlope() {
    // A plane that falls to the right: 1.0 at x = 0, 0.69 at x = 31. A drop without inertia runs straight along +x.
    const Grid start = plane(32, 32, 1.0, -0.01, 0.0);
    Grid g = start;
    DropParams p;
    p.inertia = 0.0;
    Rng rng(1);
    std::vector<Vec2> path;
    const DropResult r = runDrop(g, 4.5, 10.5, rng, p, &path);
    // Positions 5.5 ... 30.5 are on the map, 31.5 is not (the last full cell ends at 31): 26 steps.
    assert(r.steps == 26);
    assert(path.size() == 27);
    assert(near(path.front().x, 4.5) && near(path.back().x, 30.5));
    for (const Vec2& v : path) assert(near(v.y, 10.5));  // never leaves its row
    // Nothing disappears: what it dug up is put down again at the latest where it leaves the map.
    assert(near(sumOf(g), sumOf(start), 1e-9));
    // It took material near the start and put some down further on.
    assert(g.at(4, 10) < start.at(4, 10));
    double gained = 0.0;
    for (std::size_t i = 0; i < g.v.size(); ++i) gained += std::max(0.0, g.v[i] - start.v[i]);
    assert(gained > 0.0);
    // Rows far away from the path are untouched (the brush reaches 3 cells).
    for (int x = 0; x < 32; ++x) assert(g.at(x, 20) == start.at(x, 20));
}

void dropWithOneStep() {
    // One step on the same plane. The capacity is max(0.01, 0.01) * 1 * 1 * 4 = 0.04, so 0.3 * 0.04 = 0.012 would be
    // allowed, but the step only goes down by 0.01: it digs 0.01 with the brush around (4.5, 10.5). The end of its
    // life puts that load down at the new position (5.5, 10.5), 0.0025 on each of the four cells around it.
    Grid g = plane(32, 32, 1.0, -0.01, 0.0);
    const Grid start = g;
    DropParams p;
    p.inertia = 0.0;
    p.maxSteps = 1;
    Rng rng(1);
    const DropResult r = runDrop(g, 4.5, 10.5, rng, p, nullptr);
    assert(r.steps == 1);
    Grid expected = start;
    addBrush(expected, 4.5, 10.5, -0.01);
    addBilinear(expected, 5.5, 10.5, 0.01);
    for (std::size_t i = 0; i < g.v.size(); ++i) assert(near(g.v[i], expected.v[i], 1e-12));
    assert(g.at(4, 10) < start.at(4, 10));  // the start cell is not among the deposit cells: pure loss
    assert(near(sumOf(g), sumOf(start), 1e-12));
}

void dropInABowl() {
    // A round bowl: a drop without inertia follows the gradient exactly, rolls to the bottom and jumps back and forth
    // around it. It cannot leave the map, so it uses all its steps. (With the default inertia of 0.3 it would shoot
    // across the bowl and out the other side: steps have a fixed length of one cell.)
    Grid g(41, 41);
    for (int y = 0; y < 41; ++y) {
        for (int x = 0; x < 41; ++x) g.at(x, y) = 0.0005 * ((x - 20) * (x - 20) + (y - 20) * (y - 20));
    }
    const Grid start = g;
    DropParams p;
    p.inertia = 0.0;
    Rng rng(9);
    for (int i = 0; i < 50; ++i) {
        const DropResult r = runDrop(g, 25.3 + i * 0.1, 20.2, rng, p, nullptr);
        assert(r.steps == p.maxSteps);
        assert(allFinite(g));
    }
    assert(near(sumOf(g), sumOf(start), 1e-9));
}

void dropOnFlatGround() {
    // No gradient: the direction comes from the random number generator. No NaN, nothing changes (no height drop,
    // so nothing to dig).
    Grid g = filled(30, 30, 0.5);
    const Grid start = g;
    DropParams p;
    Rng rng(3);
    for (int i = 0; i < 100; ++i) {
        const DropResult r = runDrop(g, 15.0, 15.0, rng, p, nullptr);
        assert(r.steps >= 0 && r.steps <= p.maxSteps);
    }
    assert(allFinite(g));
    assert(g.v == start.v);
}

void dropEdgeCases() {
    DropParams p;
    Rng rng(2);
    Grid tiny = filled(2, 2, 1.0);  // a single cell: every step of length 1 leaves it, so the drop does nothing
    const DropResult r = runDrop(tiny, 0.5, 0.5, rng, p, nullptr);
    assert(r.steps == 0);
    Grid g = plane(16, 16, 1.0, -0.02, 0.01);
    const Grid start = g;
    // Start positions outside the map are clamped onto it, NaN is treated as 0.
    runDrop(g, -50.0, 99.0, rng, p, nullptr);
    runDrop(g, std::numeric_limits<double>::quiet_NaN(), 3.0, rng, p, nullptr);
    runDrop(g, 1e300, -1e300, rng, p, nullptr);
    assert(allFinite(g));
    assert(near(sumOf(g), sumOf(start), 1e-9));
    // Extreme settings do not break the conservation either.
    p.inertia = 1.0;
    p.erode = 1.0;
    p.deposit = 1.0;
    p.evaporate = 1.0;
    p.capacity = 100.0;
    for (int i = 0; i < 200; ++i) runDrop(g, 3.0 + i * 0.05, 8.0, rng, p, nullptr);
    assert(allFinite(g));
    assert(near(sumOf(g), sumOf(start), 1e-9));
    p.inertia = -3.0;  // nonsense values are clamped
    p.evaporate = -2.0;
    p.deposit = 9.0;
    for (int i = 0; i < 200; ++i) runDrop(g, 3.0 + i * 0.05, 8.0, rng, p, nullptr);
    assert(allFinite(g));
    assert(near(sumOf(g), sumOf(start), 1e-9));
}

void naiveDropsLoseMaterial() {
    // The naive drop cuts the same amount in every step and never puts anything down. On the plane it runs 26 steps
    // and 26 * erode * kNaiveCut disappear (the brush hands out the full amount even next to the edge).
    Grid g = plane(32, 32, 1.0, -0.01, 0.0);
    const double before = sumOf(g);
    DropParams p;
    p.mode = Mode::Naive;
    p.inertia = 0.0;
    p.erode = 0.5;
    Rng rng(1);
    const DropResult r = runDrop(g, 4.5, 10.5, rng, p, nullptr);
    assert(r.steps == 26);
    assert(near(before - sumOf(g), 26.0 * p.erode * kNaiveCut, 1e-9));
}

void naiveDropsStayInTheHollow() {
    // A valley along x: the height is 0.01 * |x - 20|. The naive drop slides in from x = 10.5 and cuts a little in
    // every step. At 19.5 the next position (20.5) is now higher than the spot it has just dug out, so it stops dead:
    // stuck in the hollow after 9 steps, 31 steps unused, and it never gets to the other side.
    Grid g(41, 9);
    for (int y = 0; y < 9; ++y) {
        for (int x = 0; x < 41; ++x) g.at(x, y) = 0.01 * std::fabs(x - 20.0);
    }
    DropParams p;
    p.mode = Mode::Naive;
    p.inertia = 0.0;
    Rng rng(1);
    std::vector<Vec2> path;
    const DropResult r = runDrop(g, 10.5, 4.5, rng, p, &path);
    assert(r.steps == 9);
    assert(path.size() == 10);
    assert(near(path.back().x, 19.5));
    for (const Vec2& v : path) assert(v.x <= 19.5 + 1e-9);
}

void sedimentDropsFillTheHollow() {
    // The same valley with the real drop: the sediment it brings stays in the hollow and raises its floor.
    Grid g(41, 9);
    for (int y = 0; y < 9; ++y) {
        for (int x = 0; x < 41; ++x) g.at(x, y) = 0.01 * std::fabs(x - 20.0) + 0.002 * y;
    }
    const Grid start = g;
    DropParams p;
    p.inertia = 0.0;
    Rng rng(4);
    for (int i = 0; i < 300; ++i) runDrop(g, 8.5 + (i % 5), 3.5 + (i % 3), rng, p, nullptr);
    assert(near(sumOf(g), sumOf(start), 1e-9));
    assert(allFinite(g));
    double floorBefore = 0.0;
    double floorAfter = 0.0;
    for (int x = 18; x <= 22; ++x) {
        floorBefore += start.at(x, 4);
        floorAfter += g.at(x, 4);
    }
    assert(floorAfter > floorBefore);
}

// ---------------------------------------------------------------- thermal erosion

void thermalByHand() {
    // A 3x3 grid, centre 10, everything else 0. Relief 1 and a talus angle of 45 degrees: the allowed difference
    // is 1 per cell distance (tan 45 = 1) and sqrt2 on the diagonals.
    // Excess: 4 orthogonal neighbours 10 - 1 = 9, 4 diagonal neighbours 10 - sqrt2.
    // Moved: 0.5 * the largest excess (9) = 4.5, shared out in proportion to the excess.
    Grid g = filled(3, 3, 0.0);
    g.at(1, 1) = 10.0;
    ThermalParams p;
    p.relief = 1.0;
    p.talusDeg = 45.0;
    p.rate = 0.5;
    std::vector<double> scratch;
    const double moved = thermalPass(g, scratch, p);
    const double exOrth = 9.0;
    const double exDiag = 10.0 - std::sqrt(2.0);
    const double sumEx = 4.0 * exOrth + 4.0 * exDiag;
    assert(near(moved, 4.5, 1e-9));
    assert(near(g.at(1, 1), 5.5, 1e-9));
    assert(near(g.at(1, 0), 4.5 * exOrth / sumEx, 1e-9));
    assert(near(g.at(0, 0), 4.5 * exDiag / sumEx, 1e-9));
    assert(near(sumOf(g), 10.0, 1e-9));
}

void thermalLeavesGentleSlopes() {
    ThermalParams p;
    p.relief = 1.0;
    p.talusDeg = 45.0;
    std::vector<double> scratch;
    // 30 degrees (tan = 0.57735) is gentler than 45: nothing moves.
    Grid gentle = plane(10, 10, 0.0, std::tan(30.0 * 3.14159265358979323846 / 180.0), 0.0);
    const Grid before = gentle;
    assert(thermalPass(gentle, scratch, p) == 0.0);
    assert(gentle.v == before.v);
    // 60 degrees is steeper: material moves.
    Grid steep = plane(10, 10, 0.0, std::tan(60.0 * 3.14159265358979323846 / 180.0), 0.0);
    assert(thermalPass(steep, scratch, p) > 0.0);
    // A flat grid and an empty grid are fine.
    Grid flat = filled(8, 8, 2.0);
    assert(thermalPass(flat, scratch, p) == 0.0);
    Grid empty(0, 0);
    assert(thermalPass(empty, scratch, p) == 0.0);
    Grid line(1, 7);
    line.at(0, 3) = 5.0;
    thermalPass(line, scratch, p);  // a single column has only two neighbours per cell, no crash
    assert(near(sumOf(line), 5.0, 1e-9));
}

void thermalRelaxesASpike() {
    // A tower in the middle of flat ground. With enough passes no slope is steeper than the talus angle (plus a
    // small tolerance), and not one grain is lost.
    Grid g = filled(31, 31, 0.0);
    g.at(15, 15) = 20.0;
    ThermalParams p;
    p.relief = 1.0;
    p.talusDeg = 45.0;
    std::vector<double> scratch;
    const double before = steepestDeg(g, 1.0);
    assert(before > 80.0);
    for (int i = 0; i < 400; ++i) thermalPass(g, scratch, p);
    assert(near(sumOf(g), 20.0, 1e-9));
    assert(steepestDeg(g, 1.0) < 45.001);  // it settles at exactly the talus angle
    assert(g.at(15, 15) < 3.0);  // the tower is a heap now (a cone of 45 degrees and volume 20 is about 2.7 high)
    assert(g.at(15, 15) > g.at(15, 20));  // but still a heap, highest in the middle
    for (double v : g.v) assert(v >= -1e-12);  // it never digs holes
}

void thermalUsesRelief() {
    // The same heights are steeper when the relief is larger: height difference 0.019 per cell.
    // Relief 50: slope 0.95, below 45 degrees (tan = 1), nothing moves. Relief 100: slope 1.9, it moves.
    ThermalParams p;
    p.talusDeg = 45.0;
    std::vector<double> scratch;
    Grid a = plane(8, 8, 0.0, 0.019, 0.0);
    p.relief = 50.0;
    assert(thermalPass(a, scratch, p) == 0.0);
    Grid b = plane(8, 8, 0.0, 0.019, 0.0);
    p.relief = 100.0;
    assert(thermalPass(b, scratch, p) > 0.0);
}

// ---------------------------------------------------------------- statistics

void statistics() {
    // h = 0.5 x with relief 1: |gradient| = 0.5 everywhere, atan(0.5) = 26.565 degrees.
    const Grid g = plane(10, 10, 0.0, 0.5, 0.0);
    assert(near(meanSlopeDeg(g, 1.0), 26.565051177, 1e-6));
    assert(near(meanSlopeDeg(g, 2.0), 45.0, 1e-9));  // relief 2: the slope is 1
    assert(near(steepestDeg(g, 1.0), 26.565051177, 1e-6));
    // A diagonal neighbour is sqrt2 away: h = x + y rises by 2 over a distance of sqrt2 along the diagonal (slope
    // sqrt2 = 54.7356 degrees), but only by 1 over 1 along the axes (45 degrees).
    const Grid d = plane(10, 10, 0.0, 1.0, 1.0);
    assert(near(steepestDeg(d, 1.0), 54.735610317, 1e-6));
    assert(near(meanSlopeDeg(filled(6, 6, 3.0), 1.0), 0.0));
    assert(meanSlopeDeg(Grid(2, 2), 1.0) == 0.0);  // no interior cell
    assert(steepestDeg(Grid(1, 1), 1.0) == 0.0);
    assert(near(sumOf(filled(4, 5, 0.25)), 5.0));
}

// ---------------------------------------------------------------- colors

void colors() {
    const Rgb low = terrainColor(0.0);
    const Rgb high = terrainColor(1.0);
    assert(high.r > low.r && high.g > low.g && high.b > low.b);  // dark lowland, bright peaks
    assert(terrainColor(-5.0).r == low.r && terrainColor(7.0).r == high.r);
    const Rgb nan = terrainColor(std::numeric_limits<double>::quiet_NaN());
    assert(nan.r == low.r && nan.g == low.g);

    assert(near(shadeFactor(0.0, 0.0), 1.0));
    assert(near(shadeFactor(0.5, 0.5), 1.2));    // 1 + 0.4 * mean(0.5, 0.5)
    assert(near(shadeFactor(-0.5, -0.5), 0.8));
    assert(near(shadeFactor(1.0, 0.0), 1.2));    // mean(1, 0) = 0.5
    assert(near(shadeFactor(10.0, 10.0), 1.5));  // clamped
    assert(near(shadeFactor(-10.0, -10.0), 0.45));
    assert(near(shadeFactor(std::numeric_limits<double>::quiet_NaN(), 1.0), 1.0));
    assert(shade(Rgb{100, 100, 100}, 1.0).r == 100);
    assert(shade(Rgb{200, 200, 200}, 1.5).r == 255);  // clamped, no wrap-around
    assert(shade(Rgb{100, 100, 100}, 0.5).g == 50);

    // Contour lines: a line where floor(h / interval) changes between two neighbours.
    assert(isContour(0.49, 0.51, 0.05));     // 9.8 vs 10.2: crosses 0.5
    assert(!isContour(0.41, 0.44, 0.05));    // 8.2 and 8.8: both between 0.40 and 0.45
    assert(isContour(0.44, 0.46, 0.05));     // 8.8 vs 9.2: crosses 0.45
    assert(isContour(0.46, 0.44, 0.05));     // the order does not matter
    assert(!isContour(0.3, 0.3, 0.0));       // interval 0 means off, no division by zero
    assert(isContour(-0.01, 0.01, 0.1));     // crossing zero counts as well

    // Change tint: no change keeps the base color, the sign picks the side, size picks the strength.
    const Rgb base{100, 100, 100};
    const Rgb same = diffTint(base, 0.0, 0.02);
    assert(same.r == 100 && same.g == 100 && same.b == 100);
    const Rgb dug = diffTint(base, -0.02, 0.02);   // full strength: the pure orange
    assert(dug.r == 234 && dug.g == 88 && dug.b == 12);
    const Rgb rose = diffTint(base, 0.02, 0.02);   // full strength: the pure teal
    assert(rose.r == 13 && rose.g == 148 && rose.b == 136);
    const Rgb half = diffTint(base, -0.01, 0.02);   // halfway: (100 + 234) / 2
    assert(half.r == 167 && half.g == 94 && half.b == 56);
    const Rgb over = diffTint(base, -9.0, 0.02);    // more than full strength is clamped
    assert(over.r == 234);
    const Rgb badScale = diffTint(base, 1.0, 0.0);  // scale 0: no division by zero, full tint
    assert(badScale.g == 148);
    const Rgb nanDelta = diffTint(base, std::numeric_limits<double>::quiet_NaN(), 0.02);
    assert(nanDelta.r == 100);
}

// ---------------------------------------------------------------- numbers for the screen

void formatting() {
    assert(groupDigits(0) == "0");
    assert(groupDigits(7) == "7");
    assert(groupDigits(999) == "999");
    assert(groupDigits(1000) == "1,000");
    assert(groupDigits(12345) == "12,345");
    assert(groupDigits(1234567) == "1,234,567");
    assert(groupDigits(-1000) == "-1,000");
    assert(groupDigits(-5) == "-5");
}

void sliders() {
    assert(near(sliderValue(0.5, 0.0, 10.0, 0.0), 5.0));
    assert(near(sliderValue(0.34, 0.0, 10.0, 1.0), 3.0));   // snapped to the step
    assert(near(sliderValue(0.36, 0.0, 10.0, 1.0), 4.0));
    assert(near(sliderValue(-3.0, 2.0, 8.0, 1.0), 2.0));    // clamped
    assert(near(sliderValue(9.0, 2.0, 8.0, 1.0), 8.0));
    assert(near(sliderValue(std::numeric_limits<double>::quiet_NaN(), 2.0, 8.0, 1.0), 2.0));
    assert(near(sliderValue(0.5, 4.0, 4.0, 1.0), 4.0));     // empty range
    assert(near(tOfValue(5.0, 0.0, 10.0), 0.5));
    assert(near(tOfValue(50.0, 0.0, 10.0), 1.0));
    assert(near(tOfValue(1.0, 3.0, 3.0), 0.0));              // empty range: no division by zero
}

}  // namespace

int main() {
    gridBasics();
    randomNumbers();
    noiseBasics();
    fbmAndTerrain();
    sampling();
    capacity();
    stepChanges();
    bilinearSplat();
    brush();
    dropOnASlope();
    dropWithOneStep();
    dropInABowl();
    dropOnFlatGround();
    dropEdgeCases();
    naiveDropsLoseMaterial();
    naiveDropsStayInTheHollow();
    sedimentDropsFillTheHollow();
    thermalByHand();
    thermalLeavesGentleSlopes();
    thermalRelaxesASpike();
    thermalUsesRelief();
    statistics();
    colors();
    formatting();
    sliders();
    std::puts("terrain-erosion logic: all asserts passed");
    return 0;
}
