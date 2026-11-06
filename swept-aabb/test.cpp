// Plain-assert tests for the swept AABB logic. Every expected value is worked out by hand in the comment above it.
#include <cmath>
#include <cstdio>
#include <vector>

#include "logic.hpp"

namespace {

int failures = 0;

#define CHECK(cond)                                                      \
    do {                                                                 \
        if (!(cond)) {                                                   \
            std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);  \
            ++failures;                                                  \
        }                                                                \
    } while (false)

bool near(float a, float b, float eps = 1e-4F) { return std::fabs(a - b) <= eps; }

using namespace sw;

Aabb box(float x0, float y0, float x1, float y1) { return Aabb{{x0, y0}, {x1, y1}}; }

void overlapTests() {
    const Aabb a = box(0, 0, 10, 10);
    CHECK(overlaps(a, box(5, 5, 15, 15)));      // corner overlap
    CHECK(overlaps(a, box(2, 2, 3, 3)));        // fully inside
    CHECK(!overlaps(a, box(10, 0, 20, 10)));    // sharing an edge is touching, not overlapping
    CHECK(!overlaps(a, box(10, 10, 20, 20)));   // sharing only a corner
    CHECK(!overlaps(a, box(11, 0, 20, 10)));    // clear gap on x
    CHECK(!overlaps(a, box(0, -20, 10, -1)));   // clear gap on y
}

void sweepHeadOn() {
    const Aabb wall = box(30, -5, 40, 15);
    // Box [0,10], moving 40: its right edge reaches x = 30 after 20 units, so t = 20 / 40 = 0.5. Normal points back (-x).
    const Hit h = sweep(box(0, 0, 10, 10), {40, 0}, wall);
    CHECK(h.hit);
    CHECK(near(h.t, 0.5F));
    CHECK(near(h.normal.x, -1.0F) && near(h.normal.y, 0.0F));

    // Same wall from the other side: box [50,60] moving left 40, its left edge reaches x = 40 after 10 units, t = 0.25.
    const Hit left = sweep(box(50, 0, 60, 10), {-40, 0}, wall);
    CHECK(left.hit);
    CHECK(near(left.t, 0.25F));
    CHECK(near(left.normal.x, 1.0F) && near(left.normal.y, 0.0F));

    // Straight down onto a floor at y = 20: the x velocity is zero, which must not divide by zero. t = 10 / 20 = 0.5.
    const Hit down = sweep(box(0, 0, 10, 10), {0, 20}, box(-5, 20, 15, 30));
    CHECK(down.hit);
    CHECK(near(down.t, 0.5F));
    CHECK(near(down.normal.x, 0.0F) && near(down.normal.y, -1.0F));

    // Straight up, mirrored: ceiling [-5,15] x [-30,-20], box top at 0 reaches -20 after 20 units, t = 20 / 40 = 0.5.
    const Hit up = sweep(box(0, 0, 10, 10), {0, -40}, box(-5, -30, 15, -20));
    CHECK(up.hit);
    CHECK(near(up.t, 0.5F));
    CHECK(near(up.normal.y, 1.0F));
}

void sweepMisses() {
    const Aabb wall = box(30, -5, 40, 15);
    // Not far enough: 20 units to the wall, 5 per frame, t would be 4.
    CHECK(!sweep(box(0, 0, 10, 10), {5, 0}, wall).hit);
    // Standing still never hits, with or without a wall nearby.
    CHECK(!sweep(box(0, 0, 10, 10), {0, 0}, wall).hit);
    CHECK(!sweep(box(25, 0, 35, 10), {0, 0}, wall).hit);
    // Wall is above the path (box y [0,10], wall y [20,30]), velocity on y is zero: miss by the no-motion branch.
    CHECK(!sweep(box(0, 0, 10, 10), {40, 0}, box(30, 20, 40, 30)).hit);
    // Diagonal that passes below the wall: x slab is [0.5, 1.0], y slab is [-0.25, 0.25], they never overlap.
    CHECK(!sweep(box(0, 0, 10, 10), {40, 40}, box(30, 0, 40, 10)).hit);
    // Wall behind the box.
    CHECK(!sweep(box(50, 0, 60, 10), {40, 0}, wall).hit);
    // Grazing a corner: x slab is [0.5, 1.0], y slab is [0, 0.5], entry equals exit, the boxes meet in one point only.
    CHECK(!sweep(box(0, 0, 10, 10), {40, 40}, box(30, 10, 40, 20)).hit);
}

void sweepTouching() {
    // Box [0,10] touching a wall [10,20] on its right.
    const Aabb wall = box(10, 0, 20, 10);
    // Moving into it: stopped at once, t = 0.
    const Hit into = sweep(box(0, 0, 10, 10), {5, 0}, wall);
    CHECK(into.hit);
    CHECK(near(into.t, 0.0F));
    CHECK(near(into.normal.x, -1.0F));
    // Moving away from it: exit time is 0, not a hit.
    CHECK(!sweep(box(0, 0, 10, 10), {-5, 0}, wall).hit);
    // Sliding along it (only y velocity): the faces touch but never overlap, not a hit.
    CHECK(!sweep(box(0, 0, 10, 10), {0, 5}, wall).hit);
}

void sweepStartsOverlapped() {
    // Box already inside the wall: the entry time is negative, no sensible normal, no hit.
    CHECK(!sweep(box(0, 0, 10, 10), {10, 0}, box(5, 0, 15, 10)).hit);
}

void sweepNormals() {
    // Diagonal onto a floor: x slab is [-0.375, 1.5], y slab is [0.25, 0.75], so y enters last: t = 0.25, normal -y.
    const Hit floor = sweep(box(0, 0, 10, 10), {40, 40}, box(-5, 20, 60, 30));
    CHECK(floor.hit);
    CHECK(near(floor.t, 0.25F));
    CHECK(near(floor.normal.x, 0.0F) && near(floor.normal.y, -1.0F));

    // Clipping the top corner of a wall: x enters at 0.5, y at (16 - 10) / 10 = 0.6, y is last: t = 0.6, normal -y.
    const Hit corner = sweep(box(0, 0, 10, 10), {40, 10}, box(30, 16, 50, 50));
    CHECK(corner.hit);
    CHECK(near(corner.t, 0.6F));
    CHECK(near(corner.normal.x, 0.0F) && near(corner.normal.y, -1.0F));

    // Same wall, shallower: y enters at (12 - 10) / 10 = 0.2, x at 0.5, x is last: face hit, t = 0.5, normal -x.
    const Hit face = sweep(box(0, 0, 10, 10), {40, 10}, box(30, 12, 50, 50));
    CHECK(face.hit);
    CHECK(near(face.t, 0.5F));
    CHECK(near(face.normal.x, -1.0F) && near(face.normal.y, 0.0F));

    // Dead on a corner: both slabs open at 0.5. The tie goes to x, the normal is still one clean axis.
    const Hit tie = sweep(box(0, 0, 10, 10), {40, 40}, box(30, 30, 50, 50));
    CHECK(tie.hit);
    CHECK(near(tie.t, 0.5F));
    CHECK(near(tie.normal.x, -1.0F) && near(tie.normal.y, 0.0F));

    // Reaching the wall exactly at the end of the frame still counts: 20 units to go, 20 units of motion, t = 1.
    const Hit late = sweep(box(0, 0, 10, 10), {20, 0}, box(30, 0, 40, 10));
    CHECK(late.hit);
    CHECK(near(late.t, 1.0F));
    // One unit short of the wall is a miss for this frame.
    CHECK(!sweep(box(0, 0, 10, 10), {19, 0}, box(30, 0, 40, 10)).hit);
}

void moveStop() {
    const Aabb wall = box(30, -5, 40, 15);
    // t = 0.5 puts the box at [20,30], the skin keeps it 0.02 off the wall.
    const Aabb stopped = moveSwept(box(0, 0, 10, 10), {40, 0}, std::span<const Aabb>(&wall, 1), false);
    CHECK(near(stopped.min.x, 20.0F - kSkin, 1e-3F));
    CHECK(near(stopped.min.y, 0.0F));
    CHECK(!overlaps(stopped, wall));
    // Free motion goes the whole way.
    const Aabb free = moveSwept(box(0, 0, 10, 10), {5, 3}, std::span<const Aabb>(&wall, 1), false);
    CHECK(near(free.min.x, 5.0F) && near(free.min.y, 3.0F));
}

void moveSlide() {
    // Tall wall at x = 30: hit at t = 0.5 at (20,10), the remaining (20,10) loses its x part and slides 10 down.
    const Aabb wall = box(30, -50, 40, 50);
    const Aabb slid = moveSwept(box(0, 0, 10, 10), {40, 20}, std::span<const Aabb>(&wall, 1), true);
    CHECK(near(slid.min.x, 20.0F - kSkin, 1e-3F));
    CHECK(near(slid.min.y, 20.0F, 1e-3F));
    CHECK(!overlaps(slid, wall));

    // Clipping a top corner (wall y from 16): lands at t = 0.6 on (24,6), remaining (16,4) slides 16 along the top.
    const Aabb top = box(30, 16, 50, 50);
    const Aabb over = moveSwept(box(0, 0, 10, 10), {40, 10}, std::span<const Aabb>(&top, 1), true);
    CHECK(near(over.min.x, 40.0F, 1e-3F));
    CHECK(near(over.min.y, 6.0F - kSkin, 1e-3F));
    CHECK(!overlaps(over, top));

    // Slide must not stop where stop-mode stops: same input, different result.
    const Aabb stop = moveSwept(box(0, 0, 10, 10), {40, 10}, std::span<const Aabb>(&top, 1), false);
    CHECK(stop.min.x < 25.0F);

    // Straight into a wall, nothing to slide along: slide ends up where stop does.
    const Aabb straight = moveSwept(box(0, 0, 10, 10), {40, 0}, std::span<const Aabb>(&wall, 1), true);
    CHECK(near(straight.min.x, 20.0F - kSkin, 1e-3F) && near(straight.min.y, 0.0F));
}

void moveNearest() {
    // Two obstacles on the path, listed far one first: the near one at x = 20 must win. t = 10 / 40 = 0.25.
    const std::vector<Aabb> walls{box(30, -5, 40, 15), box(20, -5, 25, 15)};
    const Aabb stopped = moveSwept(box(0, 0, 10, 10), {40, 0}, walls, false);
    CHECK(near(stopped.min.x, 10.0F - kSkin, 1e-3F));
}

void naiveTests() {
    const Aabb wall = box(30, -5, 40, 15);
    // 45 per frame: the box lands on [45,55], clear of the wall, but the sweep says it crossed it at t = 20/45.
    const NaiveStep tunnel = naiveStep(box(0, 0, 10, 10), {45, 0}, wall, false);
    CHECK(!tunnel.blocked);
    CHECK(tunnel.missed);
    CHECK(near(tunnel.box.min.x, 45.0F));

    // 25 per frame: [25,35] overlaps, the move is refused and the box stays. The sweep would have gone 20 units
    // (t = 20/25 = 0.8) before the wall, so the naive box is 20 short.
    const NaiveStep blocked = naiveStep(box(0, 0, 10, 10), {25, 0}, wall, false);
    CHECK(blocked.blocked);
    CHECK(!blocked.missed);
    CHECK(near(blocked.box.min.x, 0.0F));
    CHECK(near(blocked.shortBy, 20.0F));

    // 5 per frame, far from the wall: plain move.
    const NaiveStep calm = naiveStep(box(0, 0, 10, 10), {5, 0}, wall, false);
    CHECK(!calm.blocked && !calm.missed);
    CHECK(near(calm.box.min.x, 5.0F));
    CHECK(near(calm.shortBy, 0.0F));

    // Slide variant on a tall wall: x refused, y accepted, so the box moves along the wall.
    const Aabb tall = box(30, -50, 40, 50);
    const NaiveStep slide = naiveStep(box(0, 0, 10, 10), {25, 10}, tall, true);
    CHECK(slide.blocked);
    CHECK(near(slide.box.min.x, 0.0F) && near(slide.box.min.y, 10.0F));
}

void velocityTests() {
    const Vec2 flat = velocity(150.0F, 0.0F);
    CHECK(near(flat.x, 150.0F) && near(flat.y, 0.0F));
    const Vec2 down = velocity(100.0F, 90.0F);
    CHECK(near(down.x, 0.0F, 1e-3F) && near(down.y, 100.0F, 1e-3F));
    // 30 degrees: cos = 0.8660254, sin = 0.5.
    const Vec2 tilt = velocity(100.0F, 30.0F);
    CHECK(near(tilt.x, 86.60254F, 1e-3F) && near(tilt.y, 50.0F, 1e-3F));
}

// ---- whole runs in the demo scene: box 36 wide starting at x = 60, wall at x = 590, 10 thick ----

Run runUntilDone(Mode mode, float speed, float thickness, bool slide, float degrees = 0.0F) {
    Run run = startRun(mode, startBox(95.0F));
    const Aabb wall = wallBox(thickness);
    const Vec2 v = velocity(speed, degrees);
    for (int i = 0; i < 200 && !run.done; ++i) step(run, v, wall, slide);
    return run;
}

void sceneTests() {
    // Naive, 20 per frame: positions 60 + 20k. k = 24 gives [540,576], k = 25 gives [560,596] which overlaps [590,600].
    // So the box rests at 540, 14 short of the wall (590 - 576).
    const Run slow = runUntilDone(Mode::Naive, 20.0F, 10.0F, false);
    CHECK(slow.done);
    CHECK(near(slow.box.min.x, 540.0F));
    CHECK(!slow.missed);
    CHECK(near(slow.shortBy, 14.0F, 1e-3F));

    // Naive, 150 per frame: boxes at 60, 210, 360, 510, 660. [510,546] and [660,696] both clear [590,600]: tunneled.
    const Run fast = runUntilDone(Mode::Naive, 150.0F, 10.0F, false);
    CHECK(fast.done);
    CHECK(fast.missed);
    CHECK(fast.box.min.x > 600.0F);
    CHECK(fast.key.valid && near(fast.key.from.min.x, 510.0F));

    // Naive, 80 per frame: 60 + 80k, [540,576] then [620,656]: tunneled as well, different phase.
    CHECK(runUntilDone(Mode::Naive, 80.0F, 10.0F, false).missed);
    // Below wall + box = 46 the naive test cannot jump: 45 per frame lands on 510, then [555,591] overlaps [590,600].
    const Run edge = runUntilDone(Mode::Naive, 45.0F, 10.0F, false);
    CHECK(!edge.missed);
    CHECK(near(edge.box.min.x, 510.0F));
    // A thick wall (60) stops 80 per frame: 540 -> [620,656] overlaps [590,650].
    CHECK(!runUntilDone(Mode::Naive, 80.0F, 60.0F, false).missed);

    // The sweep never tunnels and never overlaps, at any speed. Its box rests just left of the wall.
    for (const float speed : {20.0F, 45.0F, 46.0F, 80.0F, 150.0F, 400.0F, 5000.0F, 100000.0F}) {
        for (const float thickness : {1.0F, 10.0F, 60.0F}) {
            Run run = startRun(Mode::Sweep, startBox(95.0F));
            const Aabb wall = wallBox(thickness);
            const Vec2 v = velocity(speed, 0.0F);
            bool clean = true;
            for (int i = 0; i < 200 && !run.done; ++i) {
                step(run, v, wall, false);
                if (overlaps(run.box, wall) || run.box.max.x > wall.min.x) clean = false;
            }
            CHECK(clean);
            CHECK(run.done);
            CHECK(run.key.valid);
            CHECK(run.box.max.x > wall.min.x - 0.1F);
        }
    }

    // Sweep at 150: the first frame to hit starts at 510, the box [510,546] has 590 - 546 = 44 to go, t = 44 / 150.
    const Run sweepRun = runUntilDone(Mode::Sweep, 150.0F, 10.0F, false);
    CHECK(sweepRun.key.valid);
    CHECK(near(sweepRun.key.hit.t, 44.0F / 150.0F, 1e-4F));
    const Aabb impact = impactBox(sweepRun.key);
    CHECK(near(impact.max.x, 590.0F, 1e-3F));
}

void slideSceneTest() {
    // Sliding down a wall: 8 degrees, the box meets the wall face and slides along it until it clears the wall bottom
    // (y = 155), then moves on and leaves the lane. It never overlaps the wall.
    Run run = startRun(Mode::Sweep, startBox(70.0F));
    const Aabb wall = wallBox(10.0F);
    const Vec2 v = velocity(150.0F, 8.0F);
    bool clean = true;
    for (int i = 0; i < 200 && !run.done; ++i) {
        step(run, v, wall, true);
        if (overlaps(run.box, wall)) clean = false;
    }
    CHECK(clean);
    CHECK(run.done);
    CHECK(run.box.min.y > kLaneH || run.box.min.x > kFieldW);  // slid past the bottom end of the wall and left
    CHECK(run.key.valid && near(run.key.hit.normal.x, -1.0F));

    // Stop mode with the same input rests at the wall and is done after two still frames.
    Run stop = startRun(Mode::Sweep, startBox(70.0F));
    for (int i = 0; i < 200 && !stop.done; ++i) step(stop, v, wall, false);
    CHECK(stop.done);
    CHECK(near(stop.box.max.x, 590.0F - kSkin, 1e-2F));
}

}  // namespace

int main() {
    overlapTests();
    sweepHeadOn();
    sweepMisses();
    sweepTouching();
    sweepStartsOverlapped();
    sweepNormals();
    moveStop();
    moveSlide();
    moveNearest();
    naiveTests();
    velocityTests();
    sceneTests();
    slideSceneTest();
    if (failures == 0) std::printf("all swept-aabb tests passed\n");
    return failures == 0 ? 0 : 1;
}
