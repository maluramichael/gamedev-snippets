// Tests for the starfield logic. Plain asserts on hand-computed cases, no framework, no SDL.
// Camera space: x right, y down (like the screen), z into the scene. The camera sits at the origin.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "logic.hpp"

using namespace sf;

namespace {

bool near(double a, double b, double eps = 1e-9) { return std::fabs(a - b) < eps; }

constexpr Vec2 kCentre{480.0, 230.0};
constexpr Vec2 kHalf{400.0, 150.0};  // half of the visible window in px
const Lens kLens{300.0, 0.05, 1.0};
const Box kBox{1.5, 0.6, 1.0};  // fits the far end of the view: 400 / 300 = 1.33 <= 1.5, 150 / 300 = 0.5 <= 0.6

// ---------------------------------------------------------------- projection

void projectionByHand() {
    // x / z * focal: 0.5 / 2 * 300 = 75 px right of the centre, 0.2 / 2 * 300 = 30 px below it.
    const auto p = project({0.5, 0.2, 2.0}, kCentre, kLens);
    assert(p && near(p->x, 480.0 + 75.0) && near(p->y, 230.0 + 30.0));
    // Straight ahead lands on the centre, whatever the depth.
    const auto c = project({0.0, 0.0, 0.7}, kCentre, kLens);
    assert(c && near(c->x, kCentre.x) && near(c->y, kCentre.y));
    // Twice as far away means half as far from the centre.
    const auto q = project({0.5, 0.2, 4.0}, kCentre, kLens);
    assert(q && near(q->x - kCentre.x, 37.5) && near(q->y - kCentre.y, 15.0));
}

void nearPlaneGuard() {
    // At the near plane and behind it the point is not drawn, it never gets divided.
    assert(!project({0.01, 0.0, 0.05}, kCentre, kLens));
    assert(!project({0.01, 0.0, 0.0}, kCentre, kLens));
    assert(!project({0.01, 0.0, -0.3}, kCentre, kLens));
    assert(project({0.01, 0.0, 0.0500001}, kCentre, kLens));
}

void unguardedDivideExplodes() {
    // The bug: x / z with no test. Just in front of the camera the star flies out of the picture,
    // at z = 0 only the floor keeps the number finite, behind the camera the picture is mirrored.
    const Vec2 a = projectUnguarded({0.01, 0.0, 0.0}, kCentre, kLens);
    assert(std::isfinite(a.x) && std::isfinite(a.y));
    assert(near(a.x - kCentre.x, 300.0 * 0.01 / kMinDepth, 1e-6));  // 30000 px
    const Vec2 near2 = projectUnguarded({0.01, 0.0, 0.001}, kCentre, kLens);
    assert(near(near2.x - kCentre.x, 3000.0, 1e-6));
    // Behind the camera x flips sign: the same star at z = -0.2 appears left of the centre.
    const Vec2 behind = projectUnguarded({0.01, 0.0, -0.2}, kCentre, kLens);
    assert(near(behind.x - kCentre.x, -15.0));
    const Vec2 front = projectUnguarded({0.01, 0.0, 0.2}, kCentre, kLens);
    assert(near(front.x - kCentre.x, 15.0));
    // Where the guard would have stopped the star, the unguarded value is far outside the window.
    assert(std::fabs(a.x - kCentre.x) > 50.0 * kHalf.x);
}

// ---------------------------------------------------------------- motion and perspective

void radialSpeedGrowsWithProximity() {
    // The screen speed of a star is focal * x * vz / z^2. Half the distance, four times the speed.
    const Lens lens{100.0, 0.01, 10.0};
    const Vec2 zero{0.0, 0.0};
    const double dt = 0.01;
    auto moved = [&](double z) {
        const auto a = project({1.0, 0.0, z}, zero, lens);
        const auto b = project({1.0, 0.0, z - 1.0 * dt}, zero, lens);  // flying at vz = 1
        assert(a && b);
        return b->x - a->x;
    };
    // By hand: 100 / 0.99 - 100 = 1.0101 and 100 / 0.49 - 200 = 4.0816, ratio 4.04.
    assert(near(moved(1.0), 100.0 / 0.99 - 100.0, 1e-9));
    assert(near(moved(0.5), 100.0 / 0.49 - 200.0, 1e-9));
    const double ratio = moved(0.5) / moved(1.0);
    assert(ratio > 3.9 && ratio < 4.2);
}

void starsFlowAwayFromTheVanishingPoint() {
    // Flying along v = (0.3, 0, 1) with focal 100: the vanishing point is 100 * 0.3 / 1 = 30 px right of the centre.
    const Vec2 zero{0.0, 0.0};
    const Vec3 v{0.3, 0.0, 1.0};
    const auto foe = vanishingPoint(v, zero, 100.0);
    assert(foe && near(foe->x, 30.0) && near(foe->y, 0.0));
    // A star at (0.5, 0.2, 2) after half a second sits at p - v / 2 = (0.35, 0.2, 1.5).
    const Lens lens{100.0, 0.01, 10.0};
    const auto a = project({0.5, 0.2, 2.0}, zero, lens);
    const auto b = project({0.35, 0.2, 1.5}, zero, lens);
    assert(a && b);
    // Both points and the vanishing point lie on one line: the cross product of the two offsets is 0.
    const double cross = (a->x - foe->x) * (b->y - foe->y) - (a->y - foe->y) * (b->x - foe->x);
    assert(near(cross, 0.0, 1e-9));
    // And the star moved away from it, not towards it.
    assert(std::hypot(b->x - foe->x, b->y - foe->y) > std::hypot(a->x - foe->x, a->y - foe->y));
}

void vanishingPointNeedsForwardSpeed() {
    // Sideways or backwards flight has no vanishing point in front: no division by zero.
    assert(!vanishingPoint({1.0, 0.0, 0.0}, kCentre, 300.0));
    assert(!vanishingPoint({1.0, 0.0, -0.5}, kCentre, 300.0));
    assert(vanishingPoint({1.0, 0.0, 0.5}, kCentre, 300.0));
}

void velocityPointsAtTheMouse() {
    // The mouse 150 px right of and 60 px below the centre, focal 300, speed 0.4: v = (0.2, 0.08, 0.4).
    const Vec3 v = velocityToward({kCentre.x + 150.0, kCentre.y + 60.0}, kCentre, 300.0, 0.4);
    assert(near(v.x, 0.2) && near(v.y, 0.08) && near(v.z, 0.4));
    const auto foe = vanishingPoint(v, kCentre, 300.0);
    assert(foe && near(foe->x, kCentre.x + 150.0) && near(foe->y, kCentre.y + 60.0));
    // Mouse in the middle: straight ahead.
    const Vec3 s = velocityToward(kCentre, kCentre, 300.0, 0.4);
    assert(near(s.x, 0.0) && near(s.y, 0.0) && near(s.z, 0.4));
    // Speed zero stays zero: no NaN from a normalisation.
    const Vec3 z = velocityToward({600.0, 100.0}, kCentre, 300.0, 0.0);
    assert(z.x == 0.0 && z.y == 0.0 && z.z == 0.0);
}

// ---------------------------------------------------------------- streaks

void streakFromOldAndNewPosition() {
    // Star at (1, 0, 2), flying at vz = 1, exposure 0.5 s: it was at z = 2.5 half a second ago.
    // focal 100: head 100 * 1 / 2 = 50, tail 100 * 1 / 2.5 = 40, streak length 10.
    const Lens lens{100.0, 0.05, 10.0};
    const Vec2 zero{0.0, 0.0};
    const auto s = streak({1.0, 0.0, 2.0}, {0.0, 0.0, 1.0}, 0.5, zero, lens);
    assert(s && near(s->head.x, 50.0) && near(s->tail.x, 40.0));
    assert(near(s->head.y, 0.0) && near(s->tail.y, 0.0));
    // With a sideways component the tail moves too: v = (0.4, 0, 1): it was at (1.2, 0, 2.5): 100 * 1.2 / 2.5 = 48.
    const auto t = streak({1.0, 0.0, 2.0}, {0.4, 0.0, 1.0}, 0.5, zero, lens);
    assert(t && near(t->tail.x, 48.0));
    // No exposure, no streak: head and tail coincide.
    const auto u = streak({1.0, 0.0, 2.0}, {0.0, 0.0, 1.0}, 0.0, zero, lens);
    assert(u && near(u->head.x, u->tail.x));
    // A star in or behind the near plane gets none (its head cannot be projected).
    assert(!streak({1.0, 0.0, 0.04}, {0.0, 0.0, 1.0}, 0.5, zero, lens));
    assert(!streak({1.0, 0.0, -1.0}, {0.0, 0.0, 1.0}, 0.5, zero, lens));
}

// ---------------------------------------------------------------- the world

void rngBasics() {
    Rng zero(0);  // a zero seed must not freeze the generator
    assert(zero.next() != 0 || zero.next() != 0);
    Rng a(42);
    Rng b(42);
    for (int i = 0; i < 10; ++i) assert(a.next() == b.next());
    Rng r(7);
    double sum = 0.0;
    for (int i = 0; i < 20000; ++i) {
        const double u = r.uniform();
        assert(u >= 0.0 && u < 1.0);
        sum += u;
    }
    assert(std::fabs(sum / 20000.0 - 0.5) < 0.01);
}

void wrappingIntoRanges() {
    assert(near(wrapInto(1.65, -1.5, 3.0), -1.35));
    assert(near(wrapInto(-1.65, -1.5, 3.0), 1.35));
    assert(near(wrapInto(0.3, -1.5, 3.0), 0.3));
    assert(near(wrapInto(-1.5, -1.5, 3.0), -1.5));  // the lower edge belongs to the range
    assert(near(wrapInto(1.5, -1.5, 3.0), -1.5));   // the upper edge does not
    assert(near(wrapInto(8.0, 0.0, 1.0), 0.0));
    // A tiny negative value must not come back as exactly the length (rounding), the range is half open.
    const double w = wrapInto(-1e-18, 0.0, 1.0);
    assert(w >= 0.0 && w < 1.0);
}

void starPassesTheCamera() {
    Rng rng(1);
    // z = 0.1 flying at 1 for 0.3 s ends at z = -0.2, which comes back around as 0.8 (the far end of the box).
    Star s{{0.2, 0.1, 0.1}, 1.0F, 0};
    const int wraps = advance(s, {0.0, 0.0, 1.0}, 0.3, kBox, Respawn::Far, rng);
    assert(wraps == 1);
    assert(near(s.p.z, 0.8));
    // The slot at the far end gets fresh x and y, uniform over the whole box slab.
    assert(std::fabs(s.p.x) <= kBox.halfX && std::fabs(s.p.y) <= kBox.halfY);
    assert(!(near(s.p.x, 0.2) && near(s.p.y, 0.1)));
    // Without a wrap nothing is rerolled: only the motion applies, x and y change by -v dt.
    Star t{{0.2, 0.1, 0.9}, 1.0F, 0};
    assert(advance(t, {0.1, -0.2, 0.5}, 0.2, kBox, Respawn::Far, rng) == 0);
    assert(near(t.p.x, 0.18) && near(t.p.y, 0.14) && near(t.p.z, 0.8));
    // A step longer than the whole box wraps several times and still lands inside it: 0.1 - 2.35 = -2.25 -> 0.75.
    Star u{{0.0, 0.0, 0.1}, 1.0F, 0};
    const int many = advance(u, {0.0, 0.0, 2.35}, 1.0, kBox, Respawn::Far, rng);
    assert(many == 3);
    assert(near(u.p.z, 0.75));
}

void starLeavesSideways() {
    Rng rng(2);
    // x = 1.45, flying left at 0.2 for 1 s: x grows by 0.2 to 1.65, which wraps to -1.35. z is untouched.
    Star s{{1.45, 0.0, 0.5}, 1.0F, 0};
    assert(advance(s, {-0.2, 0.0, 0.0}, 1.0, kBox, Respawn::Far, rng) == 0);
    assert(near(s.p.x, -1.35) && near(s.p.z, 0.5));
    // Same for y: -0.55 - 0.2 = -0.75 leaves the box [-0.6, 0.6) and comes back at 0.45.
    Star t{{0.0, -0.55, 0.5}, 1.0F, 0};
    assert(advance(t, {0.0, 0.2, 0.0}, 1.0, kBox, Respawn::Far, rng) == 0);
    assert(near(t.p.y, 0.45));
}

void respawnFarAgainstAnywhere() {
    // 2000 stars 0.01 in front of the camera, one 0.05 deep step: all of them wrap.
    // Far: the carry keeps them in one thin slab at the far end, 0.01 - 0.05 + 1 = 0.96.
    // Anywhere (the bug): z is rolled again, so stars appear in the middle of the view out of nowhere.
    Rng rng(3);
    double lo = 2.0;
    double hi = -1.0;
    double sum = 0.0;
    for (int i = 0; i < 2000; ++i) {
        Star far{{0.0, 0.0, 0.01}, 1.0F, 0};
        assert(advance(far, {0.0, 0.0, 1.0}, 0.05, kBox, Respawn::Far, rng) == 1);
        assert(near(far.p.z, 0.96));
        Star any{{0.0, 0.0, 0.01}, 1.0F, 0};
        assert(advance(any, {0.0, 0.0, 1.0}, 0.05, kBox, Respawn::Anywhere, rng) == 1);
        assert(any.p.z >= 0.0 && any.p.z < kBox.depth);
        lo = std::min(lo, any.p.z);
        hi = std::max(hi, any.p.z);
        sum += any.p.z;
    }
    assert(lo < 0.05 && hi > 0.95);
    assert(std::fabs(sum / 2000.0 - 0.5) < 0.03);
}

void rerolledPositionsAreUniform() {
    // Quartiles of the rerolled x over [-1.5, 1.5]: each holds a quarter of the stars, within 2 percent.
    Rng rng(4);
    int q[4] = {0, 0, 0, 0};
    constexpr int kN = 20000;
    for (int i = 0; i < kN; ++i) {
        Star s{{0.0, 0.0, 0.01}, 1.0F, 0};
        advance(s, {0.0, 0.0, 1.0}, 0.05, kBox, Respawn::Far, rng);
        const int k = std::min(3, static_cast<int>((s.p.x + kBox.halfX) / (2.0 * kBox.halfX) * 4.0));
        ++q[k];
    }
    for (int k = 0; k < 4; ++k) assert(std::fabs(static_cast<double>(q[k]) / kN - 0.25) < 0.02);
}

int visibleIn(const std::vector<Star>& stars, double z0, double z1) {
    int n = 0;
    for (const Star& s : stars) {
        if (s.p.z >= z0 && s.p.z < z1 && inFrustum(s.p, kHalf, kLens)) ++n;
    }
    return n;
}

void inFrustumBounds() {
    // At depth 1 the window is 400 / 300 = 1.333 wide and 150 / 300 = 0.5 high on each side.
    assert(inFrustum({1.33, 0.49, 1.0}, kHalf, kLens));
    assert(!inFrustum({1.34, 0.0, 1.0}, kHalf, kLens));
    assert(!inFrustum({0.0, 0.51, 1.0}, kHalf, kLens));
    assert(!inFrustum({0.0, 0.0, 0.05}, kHalf, kLens));   // the near plane itself is out
    assert(!inFrustum({0.0, 0.0, -1.0}, kHalf, kLens));   // behind the camera
    assert(inFrustum({0.0, 0.0, 0.5}, kHalf, kLens));
    // At depth 0.5 only half of that is inside.
    assert(inFrustum({0.66, 0.0, 0.5}, kHalf, kLens));
    assert(!inFrustum({0.68, 0.0, 0.5}, kHalf, kLens));
}

void densityIsUniformInTheViewVolume() {
    // The number of visible stars in a depth slab grows with the volume of the slab, which is z^3 for a pyramid.
    // Slab [0.4, 0.5) against [0.8, 0.9): (0.9^3 - 0.8^3) / (0.5^3 - 0.4^3) = 0.217 / 0.061 = 3.557.
    // Wrong would be 1 (equal counts per depth) or 2.25 (the area, z^2).
    Rng rng(5);
    std::vector<Star> stars = makeStars(40000, kBox, rng);
    const double expected = (0.729 - 0.512) / (0.125 - 0.064);
    {
        const double ratio = static_cast<double>(visibleIn(stars, 0.8, 0.9)) / visibleIn(stars, 0.4, 0.5);
        assert(std::fabs(ratio - expected) < 0.5);
    }
    // And it stays that way while flying, with a sideways component: the wrap does not thin out any side.
    const Vec3 v{0.25, -0.1, 0.4};
    for (int i = 0; i < 300; ++i) {
        for (Star& s : stars) advance(s, v, 1.0 / 60.0, kBox, Respawn::Far, rng);
    }
    for (const Star& s : stars) {
        assert(s.p.z >= 0.0 && s.p.z < kBox.depth);
        assert(s.p.x >= -kBox.halfX && s.p.x < kBox.halfX);
        assert(s.p.y >= -kBox.halfY && s.p.y < kBox.halfY);
    }
    const double ratio = static_cast<double>(visibleIn(stars, 0.8, 0.9)) / visibleIn(stars, 0.4, 0.5);
    assert(std::fabs(ratio - expected) < 0.5);
}

void randomDepthThinsOutTheFarField() {
    // The pop-in bug is not only ugly, it also changes the density. A star that is rerolled at a random depth
    // drifts down from there, so near the camera there are more of them than far away: the density falls like
    // (1 - z / depth). The slab ratio [0.8, 0.9) to [0.4, 0.5) drops from 3.56 to about 0.96.
    Rng rng(11);
    std::vector<Star> stars = makeStars(20000, kBox, rng);
    for (int i = 0; i < 300; ++i) {
        for (Star& s : stars) advance(s, {0.0, 0.0, 1.0}, 1.0 / 60.0, kBox, Respawn::Anywhere, rng);
    }
    const double ratio = static_cast<double>(visibleIn(stars, 0.8, 0.9)) / visibleIn(stars, 0.4, 0.5);
    assert(ratio < 1.5);
    // The right way keeps the 3.56 even under the same long flight.
    std::vector<Star> good = makeStars(20000, kBox, rng);
    for (int i = 0; i < 300; ++i) {
        for (Star& s : good) advance(s, {0.0, 0.0, 1.0}, 1.0 / 60.0, kBox, Respawn::Far, rng);
    }
    const double goodRatio = static_cast<double>(visibleIn(good, 0.8, 0.9)) / visibleIn(good, 0.4, 0.5);
    assert(std::fabs(goodRatio - 3.557) < 0.6);
}

void worldIsFilledWithinTheBox() {
    Rng rng(6);
    const auto stars = makeStars(5000, kBox, rng);
    assert(stars.size() == 5000);
    for (const Star& s : stars) {
        assert(s.p.z >= 0.0 && s.p.z < kBox.depth);
        assert(std::fabs(s.p.x) <= kBox.halfX && std::fabs(s.p.y) <= kBox.halfY);
        assert(s.mag > 0.0F && s.mag <= 1.0F);
        assert(s.tint < kTints);
    }
}

// ---------------------------------------------------------------- fading, smoothing, steering

void depthFadeRamps() {
    // Invisible at and behind the near plane, full at three times the near distance, invisible at the far plane.
    assert(near(depthFade(0.05, kLens), 0.0));
    assert(near(depthFade(0.0, kLens), 0.0));
    assert(near(depthFade(-1.0, kLens), 0.0));
    assert(near(depthFade(0.15, kLens), 1.0));
    assert(near(depthFade(0.5, kLens), 1.0));
    assert(near(depthFade(1.0, kLens), 0.0));
    assert(near(depthFade(1.5, kLens), 0.0));
    // Smoothstep halfway: near ramp from 0.05 to 0.15 has its middle at 0.10, far ramp 0.75 to 1.0 at 0.875.
    assert(near(depthFade(0.10, kLens), 0.5));
    assert(near(depthFade(0.875, kLens), 0.5));
    // A star already fades on its way out, before the near test would cut it off.
    assert(depthFade(0.07, kLens) < 0.2);
}

void starSizeIsBounded() {
    // focal 300 * 0.0012 / z: 0.36 / 0.3 = 1.2 px at z = 0.3.
    assert(near(apparentRadius(0.3, kLens), 1.2));
    // Closer is bigger, up to the cap, and the cap also holds at and behind the near plane (no divide by zero).
    assert(apparentRadius(0.2, kLens) > apparentRadius(0.3, kLens));
    assert(near(apparentRadius(0.05, kLens), kMaxStarPx));
    assert(near(apparentRadius(0.0, kLens), kMaxStarPx));
    assert(near(apparentRadius(-2.0, kLens), kMaxStarPx));
    // Far away it stays a visible dot.
    assert(near(apparentRadius(1.0, kLens), kMinStarPx));
}

void approachIgnoresFrameRate() {
    // One step of 1/30 s equals two steps of 1/60 s: 1 - exp(-lambda dt) composes.
    const double lambda = 5.0;
    const double once = approach(0.0, 10.0, lambda, 1.0 / 30.0);
    const double twice = approach(approach(0.0, 10.0, lambda, 1.0 / 60.0), 10.0, lambda, 1.0 / 60.0);
    assert(near(once, twice, 1e-12));
    assert(near(once, 10.0 * (1.0 - std::exp(-lambda / 30.0))));
    assert(near(approach(3.0, 10.0, lambda, 0.0), 3.0));  // no time, no movement
    assert(approach(3.0, 10.0, lambda, 100.0) <= 10.0);   // a long stall never overshoots
    assert(near(approach(3.0, 10.0, lambda, 100.0), 10.0, 1e-9));
}

void steeringTargetIsClamped() {
    // The mouse may leave the stage, the vanishing point stays at most 70 percent of the way to the edge.
    const Vec2 t = clampHeading({kCentre.x + 1000.0, kCentre.y - 1000.0}, kCentre, kHalf, 0.7);
    assert(near(t.x, kCentre.x + 0.7 * kHalf.x) && near(t.y, kCentre.y - 0.7 * kHalf.y));
    const Vec2 inside = clampHeading({kCentre.x + 100.0, kCentre.y + 20.0}, kCentre, kHalf, 0.7);
    assert(near(inside.x, kCentre.x + 100.0) && near(inside.y, kCentre.y + 20.0));
}

void autopilotStaysInBounds() {
    const Vec2 amp{200.0, 60.0};
    Vec2 prev = autopilot(0.0, amp);
    double maxStep = 0.0;
    for (int i = 1; i <= 60 * 600; ++i) {
        const Vec2 p = autopilot(i / 60.0, amp);
        assert(std::fabs(p.x) <= amp.x + 1e-9 && std::fabs(p.y) <= amp.y + 1e-9);
        maxStep = std::max(maxStep, std::hypot(p.x - prev.x, p.y - prev.y));
        prev = p;
    }
    assert(maxStep < 4.0);  // no jumps: a few px per frame at most
}

// ---------------------------------------------------------------- the layered (naive) version

void flatPositionByHand() {
    // Angle 0, 100 px out, steering 40 px right: layer 1 shifts by half of it (parallax 0.5), layer 2 by all of it.
    const FlatStar a{0.0, 100.0, 1, 1.0F, 0};
    const Vec2 p = flatPosition(a, kCentre, {40.0, 0.0});
    assert(near(p.x, 480.0 + 100.0 + 20.0) && near(p.y, 230.0));
    const FlatStar b{0.0, 100.0, 2, 1.0F, 0};
    assert(near(flatPosition(b, kCentre, {40.0, 0.0}).x, 480.0 + 100.0 + 40.0));
    // Angle 90 degrees points down the screen.
    const FlatStar c{std::acos(-1.0) / 2.0, 50.0, 0, 1.0F, 0};
    const Vec2 q = flatPosition(c, kCentre, {0.0, 0.0});
    assert(near(q.x, 480.0) && near(q.y, 280.0));
}

void flatSpeedIgnoresDistance() {
    // The flaw in one assert: every star of a layer moves out at the same px/s, near the centre or far from it.
    Rng rng(8);
    FlatStar a{0.0, 10.0, 2, 1.0F, 0};
    FlatStar b{0.0, 230.0, 2, 1.0F, 0};
    assert(!advanceFlat(a, 0.5, kCentre, {0, 0}, kHalf, rng));
    assert(!advanceFlat(b, 0.5, kCentre, {0, 0}, kHalf, rng));
    assert(near(a.radius - 10.0, kFlatLayers[2].speed * 0.5));
    assert(near(b.radius - 230.0, kFlatLayers[2].speed * 0.5));
    // The perspective version does the opposite, see radialSpeedGrowsWithProximity.
}

void flatStarRespawnsAtTheCentre() {
    Rng rng(9);
    FlatStar s{0.0, 395.0, 2, 1.0F, 0};  // 5 px from the right edge of the window, the fast layer
    const bool respawned = advanceFlat(s, 0.1, kCentre, {0, 0}, kHalf, rng);  // +26 px: out
    assert(respawned);
    assert(s.radius >= 0.0 && s.radius <= kFlatSpawnRadius);
    // A star that stays inside is not touched.
    FlatStar t{0.0, 100.0, 0, 1.0F, 0};
    assert(!advanceFlat(t, 0.1, kCentre, {0, 0}, kHalf, rng));
    assert(near(t.radius, 100.0 + kFlatLayers[0].speed * 0.1));
    // Respawn radii are uniform over the area of the small disc: the share inside half the radius is a quarter.
    int inner = 0;
    constexpr int kN = 20000;
    for (int i = 0; i < kN; ++i) {
        FlatStar u{0.0, 1e6, 0, 1.0F, 0};
        assert(advanceFlat(u, 0.0, kCentre, {0, 0}, kHalf, rng));
        if (u.radius <= kFlatSpawnRadius / 2.0) ++inner;
    }
    assert(std::fabs(static_cast<double>(inner) / kN - 0.25) < 0.02);
}

void flatWorldCoversTheWindow() {
    Rng rng(10);
    const auto stars = makeFlatStars(900, kHalf, rng);
    assert(stars.size() == 900);
    int outside = 0;
    int perLayer[kFlatLayerCount] = {0, 0, 0};
    for (const FlatStar& s : stars) {
        const Vec2 p = flatPosition(s, kCentre, {0.0, 0.0});
        if (std::fabs(p.x - kCentre.x) > kHalf.x || std::fabs(p.y - kCentre.y) > kHalf.y) ++outside;
        assert(s.layer >= 0 && s.layer < kFlatLayerCount);
        ++perLayer[s.layer];
    }
    // Everything starts inside the window, and every layer has stars.
    assert(outside == 0);
    for (int l = 0; l < kFlatLayerCount; ++l) assert(perLayer[l] > 100);
}

}  // namespace

int main() {
    projectionByHand();
    nearPlaneGuard();
    unguardedDivideExplodes();
    radialSpeedGrowsWithProximity();
    starsFlowAwayFromTheVanishingPoint();
    vanishingPointNeedsForwardSpeed();
    velocityPointsAtTheMouse();
    streakFromOldAndNewPosition();
    rngBasics();
    wrappingIntoRanges();
    starPassesTheCamera();
    starLeavesSideways();
    respawnFarAgainstAnywhere();
    rerolledPositionsAreUniform();
    inFrustumBounds();
    densityIsUniformInTheViewVolume();
    randomDepthThinsOutTheFarField();
    worldIsFilledWithinTheBox();
    depthFadeRamps();
    starSizeIsBounded();
    approachIgnoresFrameRate();
    steeringTargetIsClamped();
    autopilotStaysInBounds();
    flatPositionByHand();
    flatSpeedIgnoresDistance();
    flatStarRespawnsAtTheCentre();
    flatWorldCoversTheWindow();
    std::puts("starfield-parallax logic: all asserts passed");
    return 0;
}
