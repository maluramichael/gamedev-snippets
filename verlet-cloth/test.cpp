// Tests for the Verlet cloth logic. Plain asserts on hand-computed cases, no framework, no SDL.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "logic.hpp"

using namespace vc;

namespace {

bool near(float a, float b, float eps = 1e-4F) { return std::fabs(a - b) <= eps; }
bool near(Vec2 a, Vec2 b, float eps = 1e-4F) { return near(a.x, b.x, eps) && near(a.y, b.y, eps); }

Point at(float x, float y, float invMass = 1.0F) {
    Point p;
    p.pos = {x, y};
    p.prev = p.pos;
    p.invMass = invMass;
    return p;
}

Params calm() {
    Params prm;
    prm.gravity = 0.0F;
    prm.damping = 1.0F;
    prm.wind = 0.0F;
    return prm;
}

void freeFallVerletAndEulerAgree() {
    // dt = 0.1, a = 900: the first step moves a*dt^2 = 9 (Verlet starts with prev == pos, so velocity 0),
    // the second 9 + 9 + 9 = 27 in total from the start. Stored velocity gives 90 -> 9, then 180 -> 27.
    Params prm = calm();
    prm.verlet = true;
    Point v = at(0, 0);
    integrate(v, {0, 900}, prm, 0.1F);
    assert(near(v.pos, {0, 9}) && near(v.prev, {0, 0}));
    integrate(v, {0, 900}, prm, 0.1F);
    assert(near(v.pos, {0, 27}) && near(v.prev, {0, 9}));
    prm.verlet = false;
    Point e = at(0, 0);
    integrate(e, {0, 900}, prm, 0.1F);
    assert(near(e.pos, {0, 9}) && near(e.vel, {0, 90}));
    integrate(e, {0, 900}, prm, 0.1F);
    assert(near(e.pos, {0, 27}) && near(e.vel, {0, 180}));
}

void velocityIsPosMinusPrev() {
    Params prm = calm();
    Point p = at(10, 0);
    p.prev = {7, 0};  // moved 3 per step
    integrate(p, {0, 0}, prm, 0.01F);
    assert(near(p.pos, {13, 0}) && near(p.prev, {10, 0}));
    // Damping scales that implicit velocity: 3 * 0.5 = 1.5.
    prm.damping = 0.5F;
    Point q = at(10, 0);
    q.prev = {7, 0};
    integrate(q, {0, 0}, prm, 0.01F);
    assert(near(q.pos, {11.5F, 0}));
}

void pinnedAndHeldPointsStay() {
    Params prm = calm();
    Point p = at(5, 5);
    p.pinned = true;
    integrate(p, {0, 900}, prm, 0.1F);
    assert(near(p.pos, {5, 5}));
    Point h = at(5, 5);
    h.held = true;
    integrate(h, {0, 900}, prm, 0.1F);
    assert(near(h.pos, {5, 5}));
    assert(p.inv() == 0.0F && h.inv() == 0.0F && at(0, 0, 0.25F).inv() == 0.25F);
}

void satisfyMovesBothHalfway() {
    // d = 10, rest 6, equal masses: each point moves 2 towards the other, the new distance is 6.
    Point a = at(0, 0);
    Point b = at(10, 0);
    satisfy(a, b, 6.0F);
    assert(near(a.pos, {2, 0}) && near(b.pos, {8, 0}));
    // The same stretch with a pinned: b does the whole 4.
    Point a2 = at(0, 0);
    a2.pinned = true;
    Point b2 = at(10, 0);
    satisfy(a2, b2, 6.0F);
    assert(near(a2.pos, {0, 0}) && near(b2.pos, {6, 0}));
    // Mass ratio 1 : 0.25 (b is four times heavier): w = 1.25, a moves 4 * 1/1.25 = 3.2, b 0.8.
    Point a3 = at(0, 0);
    Point b3 = at(10, 0, 0.25F);
    satisfy(a3, b3, 6.0F);
    assert(near(a3.pos, {3.2F, 0}) && near(b3.pos, {9.2F, 0}));
    // Compression is pushed apart the same way: d = 6, rest 10, each moves 2 outwards.
    Point a4 = at(0, 0);
    Point b4 = at(6, 0);
    satisfy(a4, b4, 10.0F);
    assert(near(a4.pos, {-2, 0}) && near(b4.pos, {8, 0}));
    // Diagonal 3-4-5, rest 3: stretch 2, each moves 1 along (0.6, 0.8).
    Point a5 = at(0, 0);
    Point b5 = at(3, 4);
    satisfy(a5, b5, 3.0F);
    assert(near(a5.pos, {0.6F, 0.8F}) && near(b5.pos, {2.4F, 3.2F}));
}

void satisfyEdgeCases() {
    // Two pinned points cannot move.
    Point a = at(0, 0);
    Point b = at(10, 0);
    a.pinned = b.pinned = true;
    satisfy(a, b, 4.0F);
    assert(near(a.pos, {0, 0}) && near(b.pos, {10, 0}));
    // Coincident points have no direction: nothing happens, in particular no NaN.
    Point c = at(3, 3);
    Point d = at(3, 3);
    satisfy(c, d, 5.0F);
    assert(near(c.pos, {3, 3}) && near(d.pos, {3, 3}));
    // Already at rest length: untouched.
    Point e = at(0, 0);
    Point f = at(0, 7);
    satisfy(e, f, 7.0F);
    assert(near(e.pos, {0, 0}) && near(f.pos, {0, 7}));
}

World twoPoints(float dist, float rest) {
    World w;
    w.pts = {at(0, 0), at(dist, 0)};
    w.links = {Link{0, 1, rest, true}};
    return w;
}

void tearing() {
    Params prm = calm();
    prm.iterations = 1;
    prm.tear = 2.0F;
    // Both ends pinned, so the distance stays what we set. 25 > 2 * 10: the link goes.
    World w = twoPoints(25.0F, 10.0F);
    w.pts[0].pinned = w.pts[1].pinned = true;
    step(w, prm, 0.0, 0.01F);
    assert(!w.links[0].alive);
    // Exactly 2 * rest is not beyond the limit; 19 is well inside.
    World edge = twoPoints(20.0F, 10.0F);
    edge.pts[0].pinned = edge.pts[1].pinned = true;
    step(edge, prm, 0.0, 0.01F);
    assert(edge.links[0].alive);
    World ok = twoPoints(19.0F, 10.0F);
    ok.pts[0].pinned = ok.pts[1].pinned = true;
    step(ok, prm, 0.0, 0.01F);
    assert(ok.links[0].alive);
    // tear = 0 means never.
    prm.tear = 0.0F;
    World never = twoPoints(500.0F, 10.0F);
    never.pts[0].pinned = never.pts[1].pinned = true;
    step(never, prm, 0.0, 0.01F);
    assert(never.links[0].alive);
    // A dead link stops pulling: free points with a dead link do not move towards each other.
    World dead = twoPoints(25.0F, 10.0F);
    dead.links[0].alive = false;
    step(dead, prm, 0.0, 0.01F);
    assert(near(dead.pts[0].pos, {0, 0}) && near(dead.pts[1].pos, {25, 0}));
}

void segmentPointDistances() {
    const Vec2 a{0, 0};
    const Vec2 b{10, 0};
    assert(near(segmentPointDistance({5, 3}, a, b), 3.0F));
    assert(near(segmentPointDistance({-4, 3}, a, b), 5.0F));  // 3-4-5 to the endpoint a
    assert(near(segmentPointDistance({13, 4}, a, b), 5.0F));  // to the endpoint b
    assert(near(segmentPointDistance({5, 0}, a, b), 0.0F));
    // A segment of length 0 is a point.
    assert(near(segmentPointDistance({4, 5}, {1, 1}, {1, 1}), 5.0F));
}

void segmentSegmentDistances() {
    assert(near(segmentsDistance({0, 0}, {10, 10}, {0, 10}, {10, 0}), 0.0F));  // an X
    assert(near(segmentsDistance({0, 0}, {10, 0}, {0, 3}, {10, 3}), 3.0F));    // parallel
    assert(near(segmentsDistance({0, 0}, {4, 0}, {7, 0}, {9, 0}), 3.0F));      // collinear, apart
    assert(near(segmentsDistance({0, 0}, {10, 0}, {5, 0}, {5, 5}), 0.0F));     // T, touching
    assert(near(segmentsDistance({0, 0}, {2, 0}, {5, 4}, {5, 8}), 5.0F));      // (2,0) to (5,4)
    assert(near(segmentsDistance({0, 0}, {10, 0}, {5, 3}, {5, 3}), 3.0F));     // second one is a point
    assert(near(segmentsDistance({0, 0}, {0, 0}, {3, 4}, {3, 4}), 5.0F));      // both are points
    assert(near(segmentsDistance({0, 0}, {10, 0}, {0, 3}, {10, 3}), segmentsDistance({0, 3}, {10, 3}, {0, 0}, {10, 0})));
}

World line3() {
    World w;
    w.pts = {at(0, 0), at(10, 0), at(20, 0)};
    w.links = {Link{0, 1, 10.0F, true}, Link{1, 2, 10.0F, true}};
    return w;
}

void cutting() {
    World w = line3();
    // A swipe across the middle of link 0 (x = 5) with radius 1.
    assert(cutAlong(w, {5, -5}, {5, 5}, 1.0F) == 1);
    assert(!w.links[0].alive && w.links[1].alive);
    // The dead link is not counted again.
    assert(cutAlong(w, {5, -5}, {5, 5}, 1.0F) == 0);
    // A swipe that stays 3 away misses with radius 2.
    World m = line3();
    assert(cutAlong(m, {5, 3}, {5, 8}, 2.0F) == 0);
    assert(m.links[0].alive && m.links[1].alive);
    // A click is a swipe of length 0: 0.5 above link 1, radius 1.
    assert(cutAlong(m, {15, 0.5F}, {15, 0.5F}, 1.0F) == 1);
    assert(m.links[0].alive && !m.links[1].alive);
    // A long swipe along the line cuts both.
    World both = line3();
    assert(cutAlong(both, {-5, 1}, {25, 1}, 2.0F) == 2);
}

void picking() {
    World w = line3();
    assert(nearestPoint(w, {9, 1}, 5.0F) == 1u);
    assert(!nearestPoint(w, {9, 1}, 1.0F).has_value());  // (10,0) is sqrt(2) away, more than 1
    assert(!nearestPoint(w, {9, 20}, 5.0F).has_value());
    assert(!nearestPoint(World{}, {0, 0}, 100.0F).has_value());
    assert(nearestPoint(w, {5, 0}, 10.0F) == 0u);       // an exact tie goes to the lower index
}

void pinToggle() {
    World w = line3();
    w.pts[1].prev = {7, 0};
    w.pts[1].vel = {5, 5};
    togglePin(w, 1);
    assert(w.pts[1].pinned && near(w.pts[1].prev, w.pts[1].pos) && near(w.pts[1].vel, {0, 0}));
    togglePin(w, 1);
    assert(!w.pts[1].pinned && near(w.pts[1].prev, w.pts[1].pos));
    togglePin(w, 99);  // out of range: ignored
}

void holding() {
    // The point jumps to the target, the implicit velocity is the jump (5), not more than maxMove.
    Point p = at(0, 0);
    moveHeld(p, {3, 4}, 10.0F, 0.01F);
    assert(near(p.pos, {3, 4}) && near(p.prev, {0, 0}) && near(p.vel, {300, 400}));
    // A jump of 50 is limited to 10 per step: prev = pos - (6, 8).
    Point q = at(0, 0);
    moveHeld(q, {30, 40}, 10.0F, 0.01F);
    assert(near(q.pos, {30, 40}) && near(q.prev, {24, 32}));
    // Not moving: no velocity, no NaN, also with dt = 0.
    Point r = at(2, 2);
    moveHeld(r, {2, 2}, 10.0F, 0.0F);
    assert(near(r.prev, {2, 2}) && near(r.vel, {0, 0}));
}

void boundsAndFloor() {
    World w;
    w.minX = 0.0F;
    w.maxX = 100.0F;
    w.floorY = 100.0F;
    Point onFloor = at(5, 120);
    onFloor.prev = {1, 110};  // sliding right at 4 per step while falling
    Point left = at(-10, 50);
    left.prev = {-5, 50};
    Point right = at(130, 50);
    Point inside = at(50, 50);
    Point pinned = at(50, 500);
    pinned.pinned = true;
    w.pts = {onFloor, left, right, inside, pinned};
    collide(w);
    assert(near(w.pts[0].pos, {5, 100}));
    assert(near(w.pts[0].prev.y, 100.0F));       // falling speed is gone
    assert(near(w.pts[0].prev.x, 3.0F));         // sliding speed halved: 5 - 4 * 0.5
    assert(near(w.pts[1].pos, {0, 50}) && near(w.pts[1].prev.x, 0.0F));
    assert(near(w.pts[2].pos, {100, 50}));
    assert(near(w.pts[3].pos, {50, 50}));
    assert(near(w.pts[4].pos, {50, 500}));       // pinned points are left alone
}

void windIsPeriodicAndSafeAfterHours() {
    assert(near(windAccel(0.0, 0.0F, 0.0F), {0, 0}));
    // t = 0, x = 0: gust = 0.55 + 0.45 sin 0 + 0.2 sin 0 = 0.55, a = 0.55 * 600 = 330 to the right, lifted by 15 percent.
    assert(near(windAccel(0.0, 0.0F, 1.0F), {330.0F, -49.5F}, 1e-3F));
    assert(near(windAccel(0.0, 0.0F, 0.5F), {165.0F, -24.75F}, 1e-3F));
    // The gust repeats every 2 pi seconds. After a million periods (about 73 days) it must still be the same:
    // a float time would have a resolution of 0.5 s there.
    const double twoPi = 6.283185307179586;
    const Vec2 early = windAccel(1.25, 37.0F, 1.0F);
    const Vec2 late = windAccel(1.25 + twoPi * 1.0e6, 37.0F, 1.0F);
    assert(near(early, late, 0.05F));
    // It varies along x, otherwise the cloth would move like a board.
    assert(!near(windAccel(1.0, 0.0F, 1.0F), windAccel(1.0, 300.0F, 1.0F), 1.0F));
}

void sliderMapping() {
    assert(near(sliderValue(150.0F, 150.0F, 200.0F, 1.0F, 30.0F), 1.0F));
    assert(near(sliderValue(250.0F, 150.0F, 200.0F, 1.0F, 30.0F), 15.5F));
    assert(near(sliderValue(350.0F, 150.0F, 200.0F, 1.0F, 30.0F), 30.0F));
    assert(near(sliderValue(0.0F, 150.0F, 200.0F, 1.0F, 30.0F), 1.0F));      // left of the track
    assert(near(sliderValue(900.0F, 150.0F, 200.0F, 1.0F, 30.0F), 30.0F));   // right of the track
    assert(near(sliderValue(200.0F, 150.0F, 0.0F, 1.0F, 30.0F), 1.0F));      // zero width: no division
    assert(near(sliderFraction(15.5F, 1.0F, 30.0F), 0.5F));
    assert(near(sliderFraction(-3.0F, 1.0F, 30.0F), 0.0F));
    assert(near(sliderFraction(5.0F, 5.0F, 5.0F), 0.0F));                     // empty range
}

void strainOfLinks() {
    // Rest 10, distance 12: +20 percent. Rest 10, distance 9: 10 percent off, counted as strain too.
    World w;
    w.pts = {at(0, 0), at(12, 0), at(12, 9)};
    w.links = {Link{0, 1, 10.0F, true}, Link{1, 2, 10.0F, true}};
    const Strain s = strain(w);
    assert(s.alive == 2 && near(s.average, 0.15F) && near(s.maximum, 0.2F));
    // Dead links do not count; no live links means zeros, not NaN.
    w.links[0].alive = false;
    assert(strain(w).alive == 1 && near(strain(w).average, 0.1F));
    w.links[1].alive = false;
    const Strain none = strain(w);
    assert(none.alive == 0 && near(none.average, 0.0F) && near(none.maximum, 0.0F));
}

void gridIndices() {
    World w;
    addCloth(w, 4, 3, 10.0F, 100.0F, 50.0F, 2);
    // 4 x 3 points, 3 * 3 horizontal + 4 * 2 vertical links.
    assert(w.pts.size() == 12 && w.links.size() == 17);
    assert(w.clothCols == 4 && w.clothRows == 3);
    assert(near(w.pts[0].pos, {100, 50}) && near(w.pts[1 * 4 + 2].pos, {120, 60}));
    for (int j = 0; j < 3; ++j) {
        for (int i = 0; i < 3; ++i) {
            const Link& h = w.links[static_cast<std::size_t>(hLinkIndex(4, i, j))];
            assert(h.a == j * 4 + i && h.b == j * 4 + i + 1 && near(h.rest, 10.0F));
        }
    }
    for (int j = 0; j < 2; ++j) {
        for (int i = 0; i < 4; ++i) {
            const Link& v = w.links[static_cast<std::size_t>(vLinkIndex(4, 3, i, j))];
            assert(v.a == j * 4 + i && v.b == (j + 1) * 4 + i && near(v.rest, 10.0F));
        }
    }
    // pinEvery = 2: columns 0 and 2 of the top row, plus the last column.
    assert(w.pts[0].pinned && !w.pts[1].pinned && w.pts[2].pinned && w.pts[3].pinned);
    for (int k = 4; k < 12; ++k) assert(!w.pts[static_cast<std::size_t>(k)].pinned);
}

void ropeBuilding() {
    World w;
    addRope(w, {0, 0}, {0, 30}, 4, 1.0F, true, false, 0.1F);
    assert(w.pts.size() == 4 && w.links.size() == 3);
    assert(near(w.pts[3].pos, {0, 30}) && w.pts[0].pinned && !w.pts[3].pinned);
    assert(near(w.pts[3].invMass, 0.1F) && near(w.pts[1].invMass, 1.0F));
    assert(near(w.links[0].rest, 10.0F));
    // Slack 1.25 makes the rest length longer than the span: the rope will sag.
    World s;
    addRope(s, {0, 0}, {30, 0}, 4, 1.25F, true, true, 1.0F);
    assert(near(s.links[0].rest, 12.5F) && s.pts[0].pinned && s.pts[3].pinned);
    // The ropes added after a cloth keep the cloth's index helpers valid (cloth is first).
    World c;
    addCloth(c, 3, 3, 10.0F, 0, 0, 1);
    const std::size_t clothLinks = c.links.size();
    addRope(c, {50, 0}, {50, 20}, 3, 1.0F, true, false, 1.0F);
    assert(c.links.size() == clothLinks + 2 && c.links[clothLinks].a == 9);
}

/// A rope hanging from a pin with a heavy end.
double settledStrain(int iterations, bool verlet) {
    World w;
    addRope(w, {0, 0}, {0, 150}, 11, 1.0F, true, false, 0.1F);
    Params prm;
    prm.iterations = iterations;
    prm.verlet = verlet;
    prm.wind = 0.0F;
    prm.gravity = 900.0F;
    prm.damping = 0.995F;
    double t = 0.0;
    for (int i = 0; i < 1200; ++i) {
        step(w, prm, t, 1.0F / 120.0F);
        t += 1.0 / 120.0;
    }
    return strain(w).average;
}

void iterationsDecideRubberOrCloth() {
    const double rubber = settledStrain(1, true);
    const double stiff = settledStrain(40, true);
    std::printf("hanging rope strain: 1 iteration %.4f, 40 iterations %.4f\n", rubber, stiff);
    assert(rubber > 0.03);              // one pass: clearly stretched
    assert(stiff < 0.02);               // forty passes: close to its rest length
    assert(rubber > 5.0 * stiff);
}

void storedVelocityKeepsTheStretch() {
    // One pinned point on top, one hanging below at the rest length 10, no damping.
    for (const bool verlet : {true, false}) {
        World w;
        w.pts = {at(0, 0), at(0, 10)};
        w.pts[0].pinned = true;
        w.links = {Link{0, 1, 10.0F, true}};
        Params prm;
        prm.verlet = verlet;
        prm.iterations = 10;
        prm.gravity = 900.0F;
        prm.damping = 1.0F;
        prm.wind = 0.0F;
        const float dt = 1.0F / 120.0F;
        for (int i = 0; i < 240; ++i) step(w, prm, 0.0, dt);
        assert(near(w.pts[1].pos.y, 10.0F, 0.1F));  // either way it hangs at its rest length
        // Cut the thread and take one step.
        w.links[0].alive = false;
        const float before = w.pts[1].pos.y;
        step(w, prm, 0.0, dt);
        const float moved = w.pts[1].pos.y - before;
        if (verlet) {
            // Each correction became velocity (pos - prev) and was removed again: only gravity is left, 900 / 120^2 = 0.0625.
            assert(moved < 0.5F);
        } else {
            // The stored velocity grew by 7.5 per step for 240 steps = 1800 px/s, one more step: 1807.5 / 120 = 15.06.
            assert(near(moved, 15.0625F, 0.2F));
        }
    }
}

void sceneStaysFinite() {
    for (const int iterations : {1, 12, 30}) {
        for (const bool verlet : {true, false}) {
            World w = makeScene();
            Params prm;
            prm.iterations = iterations;
            prm.verlet = verlet;
            prm.wind = 1.0F;
            prm.tear = 3.0F;
            double t = 0.0;
            for (int i = 0; i < 1500; ++i) {
                step(w, prm, t, 1.0F / 120.0F);
                t += 1.0 / 120.0;
            }
            for (const Point& p : w.pts) {
                assert(std::isfinite(p.pos.x) && std::isfinite(p.pos.y));
                assert(p.pos.x >= w.minX - 1e-3F && p.pos.x <= w.maxX + 1e-3F && p.pos.y <= w.floorY + 1e-3F);
            }
        }
    }
}

void sceneShape() {
    const World w = makeScene();
    assert(w.clothCols * w.clothRows < static_cast<int>(w.pts.size()));
    const Strain s = strain(w);
    assert(s.alive == static_cast<int>(w.links.size()));
    // Everything starts at its rest length except the hammock, which is built with 15 percent slack on purpose:
    // its links are 1 / 1.15 - 1 = 13 percent too short for the span, so it starts compressed and sags.
    assert(near(s.maximum, 1.0F - 1.0F / 1.15F, 1e-3F));
}

}  // namespace

int main() {
    freeFallVerletAndEulerAgree();
    velocityIsPosMinusPrev();
    pinnedAndHeldPointsStay();
    satisfyMovesBothHalfway();
    satisfyEdgeCases();
    tearing();
    segmentPointDistances();
    segmentSegmentDistances();
    cutting();
    picking();
    pinToggle();
    holding();
    boundsAndFloor();
    windIsPeriodicAndSafeAfterHours();
    sliderMapping();
    strainOfLinks();
    gridIndices();
    ropeBuilding();
    iterationsDecideRubberOrCloth();
    storedVelocityKeepsTheStretch();
    sceneStaysFinite();
    sceneShape();
    std::puts("verlet-cloth logic: all asserts passed");
    return 0;
}
