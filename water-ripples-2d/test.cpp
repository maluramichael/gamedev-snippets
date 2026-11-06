// Tests for the wave equation in logic.hpp. Hand-computed cases first, then the properties the demo relies on.
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "logic.hpp"

namespace {

int failures = 0;

#define CHECK(cond)                                                                    \
    do {                                                                               \
        if (!(cond)) {                                                                 \
            std::printf("FAILED %s:%d: %s\n", __FILE__, __LINE__, #cond);              \
            ++failures;                                                                \
        }                                                                              \
    } while (0)

bool near(double a, double b, double eps = 1e-6) { return std::fabs(a - b) <= eps; }

using gd::water::Field;

// 5x5 grid, a single unit impulse in the middle. Worked out by hand with next = sum4 / 2 - prev:
//   step 1: the four neighbours of the middle get 0.5, the middle itself is 0 - 0 = 0
//   step 2: the middle is 4 * 0.5 / 2 - 1 = 0, the interior corners get (0.5 + 0.5) / 2 = 0.5, edge cells 0
void handComputedSteps() {
    Field f(5, 5);
    f.impulse(2, 2, 1.0F);
    f.step(1.0F);
    CHECK(near(f.at(2, 2), 0.0));
    CHECK(near(f.at(1, 2), 0.5));
    CHECK(near(f.at(3, 2), 0.5));
    CHECK(near(f.at(2, 1), 0.5));
    CHECK(near(f.at(2, 3), 0.5));
    CHECK(near(f.at(1, 1), 0.0));
    CHECK(near(f.previous(2, 2), 1.0));  // the old height is kept as `prev`

    f.step(1.0F);
    CHECK(near(f.at(2, 2), 0.0));
    CHECK(near(f.at(1, 1), 0.5));
    CHECK(near(f.at(3, 1), 0.5));
    CHECK(near(f.at(1, 3), 0.5));
    CHECK(near(f.at(3, 3), 0.5));
    CHECK(near(f.at(1, 2), 0.0));
    CHECK(near(f.at(2, 1), 0.0));
}

void dampingScalesTheResult() {
    Field f(5, 5);
    f.impulse(2, 2, 1.0F);
    f.step(0.5F);
    CHECK(near(f.at(1, 2), 0.25));  // 0.5 * damping
    CHECK(near(f.at(2, 2), 0.0));
}

void zeroStaysZero() {
    Field f(16, 9);
    for (int i = 0; i < 50; ++i) f.step(0.99F);
    CHECK(f.maxAbs() == 0.0F);
    CHECK(f.energy() == 0.0);
}

// A centred impulse on an odd-sized square grid must spread with the symmetry of the grid.
void symmetricSpread() {
    constexpr int n = 31;
    constexpr int c = n / 2;
    Field f(n, n);
    f.impulse(c, c, 1.0F);
    for (int i = 0; i < 9; ++i) f.step(1.0F);
    bool ok = true;
    bool spread = false;
    for (int d = 1; d < 10; ++d) {
        for (int e = 0; e < 10; ++e) {
            const float v = f.at(c + d, c + e);
            ok = ok && near(v, f.at(c - d, c + e)) && near(v, f.at(c + d, c - e)) && near(v, f.at(c - d, c - e)) && near(v, f.at(c + e, c + d));
            spread = spread || std::fabs(v) > 1e-4F;
        }
    }
    CHECK(ok);
    CHECK(spread);
    // No signal travels faster than one cell per step, so after 9 steps nothing exists beyond 9 cells.
    CHECK(f.at(c + 10, c) == 0.0F);
    CHECK(f.at(c, c - 10) == 0.0F);
}

// Damping 1.0: the discrete energy is conserved, also after the wave hit the (reflecting) border.
void energyConservedWithoutDamping() {
    Field f(40, 24);
    f.drop(12.0F, 10.0F, 4.0F, 1.0F);
    const double e0 = f.energy();
    CHECK(e0 > 0.0);
    for (int i = 0; i < 300; ++i) f.step(1.0F);
    const double e1 = f.energy();
    CHECK(std::fabs(e1 - e0) / e0 < 1e-3);
    CHECK(f.maxAbs() < 2.0F);  // and the field did not blow up
}

// Damping d multiplies the field every step, so the energy falls by about d^steps (0.99^200 = 0.134).
void dampingLetsAmplitudeDecay() {
    Field f(40, 24);
    f.drop(20.0F, 12.0F, 4.0F, 1.0F);
    const double e0 = f.energy();
    double last = e0;
    for (int i = 1; i <= 200; ++i) {
        f.step(0.99F);
        if (i % 20 == 0) {
            const double e = f.energy();
            CHECK(e < last);  // not every single step (the gradient term wobbles), but every 20th
            last = e;
        }
    }
    CHECK(last / e0 > 0.08);
    CHECK(last / e0 < 0.2);
}

// A stronger damping decays faster.
void strongerDampingDecaysFaster() {
    Field a(40, 24), b(40, 24);
    a.drop(20.0F, 12.0F, 4.0F, 1.0F);
    b.drop(20.0F, 12.0F, 4.0F, 1.0F);
    for (int i = 0; i < 100; ++i) {
        a.step(0.97F);
        b.step(0.999F);
    }
    CHECK(a.energy() < b.energy());
}

// The border is pinned to 0 and drops outside the grid are harmless.
void borderStaysZeroAndNothingIsOutOfRange() {
    Field f(24, 14);
    f.drop(0.0F, 0.0F, 6.0F, 5.0F);
    f.drop(23.0F, 13.0F, 6.0F, 5.0F);
    f.drop(-50.0F, 400.0F, 10.0F, 5.0F);
    f.drop(1000.0F, -3.0F, 1000.0F, 5.0F);
    f.drop(12.0F, 7.0F, 0.0F, 5.0F);
    f.drop(12.0F, 7.0F, 1000.0F, 1.0F);
    for (int i = 0; i < 200; ++i) f.step(0.995F);
    CHECK(f.heights().size() == 24U * 14U);
    bool border = true;
    for (int x = 0; x < 24; ++x) border = border && f.at(x, 0) == 0.0F && f.at(x, 13) == 0.0F;
    for (int y = 0; y < 14; ++y) border = border && f.at(0, y) == 0.0F && f.at(23, y) == 0.0F;
    CHECK(border);
}

void tinyGrids() {
    Field one(1, 1), two(2, 2), three(3, 3), thin(30, 1);
    for (Field* f : {&one, &two, &three, &thin}) {
        f->drop(0.0F, 0.0F, 3.0F, 1.0F);
        f->impulse(1, 1, 1.0F);
        for (int i = 0; i < 10; ++i) f->step(0.99F);
    }
    CHECK(one.maxAbs() == 0.0F);
    CHECK(two.maxAbs() == 0.0F);
    CHECK(thin.maxAbs() == 0.0F);
    // 3x3: one interior cell, all four neighbours are border cells (0), so it just rings down: 1, -1 * damping, ...
    Field single(3, 3);
    single.impulse(1, 1, 1.0F);
    single.step(1.0F);
    CHECK(near(single.at(1, 1), -0.0));  // sum4 = 0, prev = 0
    single.step(1.0F);
    CHECK(near(single.at(1, 1), -1.0));  // sum4 = 0, prev = 1 (the impulse before the first step)
}

// Why damping and fixed steps matter: past the limit courant2 = 0.5 the scheme is unstable and explodes.
void courantLimit() {
    Field stable(32, 32), unstable(32, 32);
    stable.drop(16.0F, 16.0F, 3.0F, 1.0F);
    unstable.drop(16.0F, 16.0F, 3.0F, 1.0F);
    for (int i = 0; i < 80; ++i) {
        stable.step(1.0F, 0.5F);
        unstable.step(1.0F, 0.6F);
    }
    CHECK(stable.maxAbs() < 2.0F);
    CHECK(unstable.maxAbs() > 1000.0F);
}

}  // namespace

int main() {
    handComputedSteps();
    dampingScalesTheResult();
    zeroStaysZero();
    symmetricSpread();
    energyConservedWithoutDamping();
    dampingLetsAmplitudeDecay();
    strongerDampingDecaysFaster();
    borderStaysZeroAndNothingIsOutOfRange();
    tinyGrids();
    courantLimit();
    if (failures != 0) {
        std::printf("%d check(s) failed\n", failures);
        return EXIT_FAILURE;
    }
    std::printf("all checks passed\n");
    return EXIT_SUCCESS;
}
