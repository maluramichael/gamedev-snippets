// Plain-assert tests for the rigid body logic. Every expected value is worked out by hand in the comment above it.
#include <cmath>
#include <cstdio>
#include <vector>

#include "logic.hpp"

namespace {

int failures = 0;

#define CHECK(cond)                                                                \
    do {                                                                           \
        if (!(cond)) {                                                             \
            std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);            \
            ++failures;                                                            \
        }                                                                          \
    } while (false)

bool near(float a, float b, float eps = 1e-4F) { return std::fabs(a - b) <= eps; }

using namespace rb;

// ---- collision detection ----

void circleCircle() {
    // radii 1 and 0.5, centers 1.2 apart on x: depth = 1.5 - 1.2 = 0.3, normal from a to b is +x.
    const Manifold m = collide(Body::circle({0, 0}, 1.0F), Body::circle({1.2F, 0}, 0.5F));
    CHECK(m.count == 1);
    CHECK(near(m.normal.x, 1.0F) && near(m.normal.y, 0.0F));
    CHECK(near(m.contacts[0].separation, -0.3F));
    // Contact sits in the middle of the overlap: a's surface is at x = 1.0, b's at x = 0.7, middle 0.85.
    CHECK(near(m.contacts[0].position.x, 0.85F));

    // Touching exactly counts as a miss, 1.5 apart is a miss, diagonal 3-4-5 triangle: distance 5 vs radius sum 6.
    CHECK(collide(Body::circle({0, 0}, 1.0F), Body::circle({1.5F, 0}, 0.5F)).count == 0);
    CHECK(collide(Body::circle({0, 0}, 1.0F), Body::circle({3.0F, 4.0F}, 1.0F)).count == 0);
    const Manifold d = collide(Body::circle({0, 0}, 3.0F), Body::circle({3.0F, 4.0F}, 3.0F));
    CHECK(d.count == 1);
    CHECK(near(d.contacts[0].separation, -1.0F));
    CHECK(near(d.normal.x, 0.6F) && near(d.normal.y, 0.8F));

    // Identical centers: no division by zero, the normal is some unit vector.
    const Manifold same = collide(Body::circle({2, 2}, 1.0F), Body::circle({2, 2}, 1.0F));
    CHECK(same.count == 1);
    CHECK(near(std::sqrt(dot(same.normal, same.normal)), 1.0F));
    CHECK(near(same.contacts[0].separation, -2.0F));
}

void circleBox() {
    const Body box = Body::box({0, 0}, {1.0F, 1.0F});
    // Circle (r 0.5) right of the box, center at x = 1.4: face contact, depth 0.5 - 0.4 = 0.1.
    const Manifold face = collide(Body::circle({1.4F, 0.3F}, 0.5F), box);
    CHECK(face.count == 1);
    // Normal points from a (circle) to b (box): -x.
    CHECK(near(face.normal.x, -1.0F) && near(face.normal.y, 0.0F));
    CHECK(near(face.contacts[0].separation, -0.1F));
    CHECK(near(face.contacts[0].position.x, 1.0F) && near(face.contacts[0].position.y, 0.3F));

    // Same pair with the arguments swapped: the normal flips.
    const Manifold swapped = collide(box, Body::circle({1.4F, 0.3F}, 0.5F));
    CHECK(swapped.count == 1);
    CHECK(near(swapped.normal.x, 1.0F));
    CHECK(near(swapped.contacts[0].separation, -0.1F));

    // Corner region: circle center at (1.3, 1.4), nearest box point (1, 1), distance 0.5, radius 0.6, depth 0.1.
    const Manifold corner = collide(box, Body::circle({1.3F, 1.4F}, 0.6F));
    CHECK(corner.count == 1);
    CHECK(near(corner.normal.x, 0.6F) && near(corner.normal.y, 0.8F));
    CHECK(near(corner.contacts[0].separation, -0.1F));
    // Radius 0.49 does not reach the corner (distance 0.5): a miss.
    CHECK(collide(box, Body::circle({1.3F, 1.4F}, 0.49F)).count == 0);

    // Circle center inside the box, 0.2 from the right face: pushed out through the right face, depth r + 0.2.
    const Manifold inside = collide(box, Body::circle({0.8F, 0.1F}, 0.3F));
    CHECK(inside.count == 1);
    CHECK(near(inside.normal.x, 1.0F) && near(inside.normal.y, 0.0F));
    CHECK(near(inside.contacts[0].separation, -0.5F));

    // Rotated box (45 degrees, half 1): its right corner is at x = 1.4142. Circle r 0.5 at x = 1.8: depth 0.1142.
    const Body diamond = Body::box({0, 0}, {1.0F, 1.0F}, 0.78539816F);
    const Manifold rot = collide(diamond, Body::circle({1.8F, 0}, 0.5F));
    CHECK(rot.count == 1);
    CHECK(near(rot.contacts[0].separation, -(1.41421356F + 0.5F - 1.8F), 1e-3F));
    CHECK(near(rot.normal.x, 1.0F, 1e-3F));
}

void boxBox() {
    const Body a = Body::box({0, 0}, {1.0F, 1.0F});
    // Overlap 0.5 along x: b spans 0.5..2.5, a spans -1..1. Normal from a to b is +x.
    const Manifold m = collide(a, Body::box({1.5F, 0}, {1.0F, 1.0F}));
    CHECK(m.count == 2);
    CHECK(near(m.normal.x, 1.0F) && near(m.normal.y, 0.0F));
    CHECK(near(m.contacts[0].separation, -0.5F) && near(m.contacts[1].separation, -0.5F));
    // Both contact points lie on the reference face x = 1 at y = -1 and y = +1 (in either order).
    CHECK(near(m.contacts[0].position.x, 1.0F) && near(m.contacts[1].position.x, 1.0F));
    CHECK(near(std::fabs(m.contacts[0].position.y), 1.0F) && near(std::fabs(m.contacts[1].position.y), 1.0F));
    CHECK(m.contacts[0].position.y * m.contacts[1].position.y < 0.0F);

    // Separated on x (gap 0.5) and exactly touching are both misses.
    CHECK(collide(a, Body::box({2.5F, 0}, {1.0F, 1.0F})).count == 0);
    CHECK(collide(a, Body::box({2.0F, 0}, {1.0F, 1.0F})).count == 0);
    // Shifted on y by 1.5: the y overlap is 0.5, the x overlap is 2, so the normal is +y.
    const Manifold up = collide(a, Body::box({0.2F, 1.5F}, {1.0F, 1.0F}));
    CHECK(up.count == 2);
    CHECK(near(up.normal.x, 0.0F) && near(up.normal.y, 1.0F));
    CHECK(near(up.contacts[0].separation, -0.5F));

    // 45 degree box at x = 2.3: its left corner is at 2.3 - 1.4142 = 0.8858, a's right face is x = 1, depth 0.1142.
    const Manifold d = collide(a, Body::box({2.3F, 0}, {1.0F, 1.0F}, 0.78539816F));
    CHECK(d.count >= 1);
    CHECK(near(d.normal.x, 1.0F, 1e-3F) && near(d.normal.y, 0.0F, 1e-3F));
    CHECK(near(d.contacts[0].separation, -(1.0F - (2.3F - 1.41421356F)), 1e-3F));

    // Separated only along an axis of b: diamond at (2, 2). On the world axes it overlaps a (left corner at 0.586),
    // along its own axis (1,1)/sqrt2 the gap is 2.828 - 1 - 1.414 = 0.414. No contact.
    CHECK(collide(a, Body::box({2.0F, 2.0F}, {1.0F, 1.0F}, 0.78539816F)).count == 0);

    // A box resting on the ground: ground top at y = 0 (y points down), box bottom 0.01 below it: two contacts, depth 0.01.
    const Body ground = Body::box({0, 1.0F}, {10.0F, 1.0F}, 0.0F, true);
    const Body resting = Body::box({0.3F, -0.5F + 0.01F}, {0.5F, 0.5F});
    const Manifold g = collide(resting, ground);
    CHECK(g.count == 2);
    CHECK(near(g.normal.x, 0.0F) && near(g.normal.y, 1.0F));
    CHECK(near(g.contacts[0].separation, -0.01F) && near(g.contacts[1].separation, -0.01F));

    // A box on its corner (45 degrees, center 0.1 inside the diagonal height): one contact, depth 0.1.
    const Body corner = Body::box({0, -0.70710678F + 0.1F}, {0.5F, 0.5F}, 0.78539816F);
    const Manifold c = collide(corner, ground);
    CHECK(c.count == 1);
    CHECK(near(c.contacts[0].separation, -0.1F, 1e-3F));
    CHECK(near(c.contacts[0].position.x, 0.0F, 1e-3F));
}

void pointTest() {
    // Circle radius 1 at (2, 2): (2.9, 2) is inside, (3.1, 2) is outside.
    const Body c = Body::circle({2, 2}, 1.0F);
    CHECK(contains(c, {2.9F, 2.0F}));
    CHECK(!contains(c, {3.1F, 2.0F}));
    // Box half (2, 0.5) turned by 90 degrees becomes 1 wide and 4 tall: (0, 1.8) is inside, (1.5, 0) is outside.
    const Body b = Body::box({0, 0}, {2.0F, 0.5F}, 1.57079633F);
    CHECK(contains(b, {0.0F, 1.8F}));
    CHECK(!contains(b, {1.5F, 0.0F}));
    CHECK(contains(b, {0.4F, 0.0F}));
}

// ---- impulse resolution ----

World spaceWorld(float restitution) {
    World w;
    w.gravity = {0, 0};
    w.restitution = restitution;
    w.friction = 0.0F;
    return w;
}

void elasticHeadOn() {
    // Two equal circles, mass m each, v = +2 and 0, e = 1. j = -(1+e) v_rel / (1/m + 1/m) => velocities swap.
    World w = spaceWorld(1.0F);
    w.bodies.push_back(Body::circle({0, 0}, 1.0F));
    w.bodies.push_back(Body::circle({1.995F, 0}, 1.0F));  // 0.005 deep, below the slop: no position push
    w.bodies[0].vel = {2.0F, 0};
    w.step(1.0F / 120.0F);
    CHECK(near(w.bodies[0].vel.x, 0.0F) && near(w.bodies[1].vel.x, 2.0F));
    CHECK(near(w.bodies[0].vel.y, 0.0F) && near(w.bodies[1].vel.y, 0.0F));
}

void elasticConservation() {
    // Unequal masses (radius 1 and 0.5 => mass ratio 4:1), e = 1: momentum and kinetic energy stay.
    World w = spaceWorld(1.0F);
    w.bodies.push_back(Body::circle({0, 0}, 1.0F));
    w.bodies.push_back(Body::circle({1.495F, 0}, 0.5F));
    w.bodies[0].vel = {3.0F, 0};
    w.bodies[1].vel = {-1.0F, 0};
    const float ma = 1.0F / w.bodies[0].invMass;
    const float mb = 1.0F / w.bodies[1].invMass;
    const float p0 = ma * 3.0F + mb * -1.0F;
    const float e0 = 0.5F * ma * 9.0F + 0.5F * mb * 1.0F;
    w.step(1.0F / 120.0F);
    const float p1 = ma * w.bodies[0].vel.x + mb * w.bodies[1].vel.x;
    const float e1 = 0.5F * ma * w.bodies[0].vel.x * w.bodies[0].vel.x + 0.5F * mb * w.bodies[1].vel.x * w.bodies[1].vel.x;
    CHECK(near(p0, p1, 1e-3F));
    CHECK(near(e0, e1, 1e-3F));
    // 1D elastic formulas: v1' = ((ma-mb) v1 + 2 mb v2)/(ma+mb), v2' = ((mb-ma) v2 + 2 ma v1)/(ma+mb).
    CHECK(near(w.bodies[0].vel.x, ((ma - mb) * 3.0F + 2.0F * mb * -1.0F) / (ma + mb), 1e-3F));
    CHECK(near(w.bodies[1].vel.x, ((mb - ma) * -1.0F + 2.0F * ma * 3.0F) / (ma + mb), 1e-3F));
}

void wallReflection() {
    // A circle flying into a static wall (box): normal speed flips and scales with e. Wall at x >= 1.
    for (const float e : {1.0F, 0.5F}) {
        World w = spaceWorld(e);
        w.bodies.push_back(Body::box({2.0F, 0}, {1.0F, 5.0F}, 0.0F, true));
        w.bodies.push_back(Body::circle({0.505F, 0}, 0.5F));  // right edge at 1.005, the wall face is at x = 1: 0.005 deep
        w.bodies[1].vel = {4.0F, 1.0F};
        w.step(1.0F / 120.0F);
        CHECK(near(w.bodies[1].vel.x, -4.0F * e, 1e-3F));
        CHECK(near(w.bodies[1].vel.y, 1.0F));  // friction is off: tangential speed untouched
        CHECK(w.bodies[0].vel.x == 0.0F && w.bodies[0].vel.y == 0.0F);  // the wall does not move
    }
}

void inelasticStop() {
    // e = 0: the normal speed relative to the wall is zero afterwards.
    World w = spaceWorld(0.0F);
    w.bodies.push_back(Body::box({2.0F, 0}, {1.0F, 5.0F}, 0.0F, true));
    w.bodies.push_back(Body::circle({0.505F, 0}, 0.5F));
    w.bodies[1].vel = {4.0F, 0};
    w.step(1.0F / 120.0F);
    CHECK(near(w.bodies[1].vel.x, 0.0F));

    // Two equal circles, e = 0: both end up with the same velocity (the mean), momentum is kept.
    World v = spaceWorld(0.0F);
    v.bodies.push_back(Body::circle({0, 0}, 1.0F));
    v.bodies.push_back(Body::circle({1.995F, 0}, 1.0F));
    v.bodies[0].vel = {2.0F, 0};
    v.step(1.0F / 120.0F);
    CHECK(near(v.bodies[0].vel.x, 1.0F) && near(v.bodies[1].vel.x, 1.0F));
}

void frictionSlowsSliding() {
    // A box sliding on the ground at 3 m/s with mu 0.5 (both surfaces 0.5 => 0.5) decelerates by mu * g = 4.905 m/s^2:
    // it stops after 3 / 4.905 = 0.61 s. After 1.5 s it must be at rest.
    World w;
    w.friction = 0.5F;
    w.restitution = 0.0F;
    w.bodies.push_back(Body::box({0, 1.0F}, {50.0F, 1.0F}, 0.0F, true));
    w.bodies.push_back(Body::box({0, -0.5F}, {0.5F, 0.5F}));
    w.bodies[1].vel = {3.0F, 0};
    for (int i = 0; i < 180; ++i) w.step(1.0F / 120.0F);
    CHECK(std::fabs(w.bodies[1].vel.x) < 0.05F);
    // Distance travelled: v^2 / (2 mu g) = 9 / 9.81 = 0.917 m (a little less because the first steps see the contact late).
    CHECK(w.bodies[1].pos.x > 0.7F && w.bodies[1].pos.x < 1.05F);
}

// ---- resting ----

float lowestPoint(const Body& b) {
    if (b.shape == Shape::Circle) return b.pos.y + b.radius;
    const float c = std::cos(b.angle);
    const float s = std::sin(b.angle);
    return b.pos.y + std::fabs(s) * b.half.x + std::fabs(c) * b.half.y;
}

float speed(const Body& b) { return std::sqrt(dot(b.vel, b.vel)) + std::fabs(b.angVel) * 0.5F; }

World groundWorld(float restitution) {
    World w;
    w.restitution = restitution;
    w.bodies.push_back(Body::box({16.0F, 15.0F}, {20.0F, 1.0F}, 0.0F, true));    // floor, top at y = 14
    w.bodies.push_back(Body::box({-1.0F, 7.0F}, {1.0F, 12.0F}, 0.0F, true));     // left wall
    w.bodies.push_back(Body::box({33.0F, 7.0F}, {1.0F, 12.0F}, 0.0F, true));     // right wall
    return w;
}

void circleComesToRest() {
    World w = groundWorld(0.5F);
    w.bodies.push_back(Body::circle({10.0F, 4.0F}, 0.6F));
    float deepest = 0.0F;
    for (int i = 0; i < 1200; ++i) {
        w.step(1.0F / 120.0F);
        deepest = std::max(deepest, lowestPoint(w.bodies[3]) - 14.0F);
    }
    CHECK(speed(w.bodies[3]) < 0.05F);
    CHECK(near(lowestPoint(w.bodies[3]), 14.0F, 0.03F));
    CHECK(deepest < 0.1F);  // it never sinks through, not even in the first impact
}

void boxComesToRest() {
    World w = groundWorld(0.3F);
    w.bodies.push_back(Body::box({10.0F, 6.0F}, {0.6F, 0.4F}, 0.4F));  // tilted, tumbles, then lies flat
    float deepest = 0.0F;
    for (int i = 0; i < 1200; ++i) {
        w.step(1.0F / 120.0F);
        deepest = std::max(deepest, lowestPoint(w.bodies[3]) - 14.0F);
    }
    CHECK(speed(w.bodies[3]) < 0.05F);
    CHECK(near(lowestPoint(w.bodies[3]), 14.0F, 0.03F));
    CHECK(deepest < 0.15F);
    const float turns = w.bodies[3].angle / 1.57079633F;  // lies flat: angle is a multiple of 90 degrees
    CHECK(std::fabs(turns - std::round(turns)) < 0.05F);
}

void stackStaysStanding() {
    // Five boxes (1 x 1) on top of each other, centers exactly 1 m apart, e = 0. After 6 s the tower still stands.
    World w = groundWorld(0.0F);
    for (int i = 0; i < 5; ++i) w.bodies.push_back(Body::box({10.0F, 13.5F - static_cast<float>(i) * 1.0F}, {0.5F, 0.5F}));
    for (int i = 0; i < 720; ++i) w.step(1.0F / 120.0F);
    for (int i = 0; i < 5; ++i) {
        const Body& b = w.bodies[3 + static_cast<std::size_t>(i)];
        CHECK(std::fabs(b.pos.x - 10.0F) < 0.1F);
        CHECK(std::fabs(b.pos.y - (13.5F - static_cast<float>(i))) < 0.12F);
        CHECK(speed(b) < 0.1F);
    }
}

void tallTowerStaysStanding() {
    // Ten boxes. This one collapses completely with a single iteration or without warm starting, 8 iterations hold it.
    World w = groundWorld(0.0F);
    for (int i = 0; i < 10; ++i) w.bodies.push_back(Body::box({10.0F, 13.5F - static_cast<float>(i)}, {0.5F, 0.5F}));
    for (int i = 0; i < 1200; ++i) w.step(1.0F / 120.0F);
    const Body& top = w.bodies.back();
    CHECK(std::fabs(top.pos.x - 10.0F) < 0.1F);
    CHECK(std::fabs(top.pos.y - (13.5F - 9.0F)) < 0.1F);
    CHECK(speed(top) < 0.05F);
}

void pyramidStaysStanding() {
    // 4-3-2-1 bricks (1 x 0.6), offset by half a brick per row, like the demo start scene.
    World w = groundWorld(0.0F);
    const float bw = 1.0F;
    const float bh = 0.6F;
    for (int row = 0; row < 4; ++row) {
        const int n = 4 - row;
        for (int i = 0; i < n; ++i) {
            const float x = 10.0F + (static_cast<float>(i) - static_cast<float>(n - 1) / 2.0F) * bw;
            w.bodies.push_back(Body::box({x, 14.0F - bh / 2.0F - static_cast<float>(row) * bh}, {bw / 2.0F, bh / 2.0F}));
        }
    }
    const std::vector<Body> start(w.bodies.begin() + 3, w.bodies.end());
    for (int i = 0; i < 720; ++i) w.step(1.0F / 120.0F);
    for (std::size_t i = 0; i < start.size(); ++i) {
        const Body& b = w.bodies[3 + i];
        CHECK(std::fabs(b.pos.x - start[i].pos.x) < 0.15F);
        CHECK(std::fabs(b.pos.y - start[i].pos.y) < 0.15F);
    }
}

void noEnergyFromNothing() {
    // A circle at rest on the floor stays at rest; the solver adds no speed by itself.
    World w = groundWorld(0.9F);
    w.bodies.push_back(Body::circle({10.0F, 13.4F}, 0.6F));
    for (int i = 0; i < 600; ++i) w.step(1.0F / 120.0F);
    CHECK(speed(w.bodies[3]) < 0.05F);
    CHECK(near(w.bodies[3].pos.y, 13.4F, 0.03F));
}

void energyKeptInARoom() {
    // Four circles in a closed room, no gravity, no friction, e = 1: they bounce off each other and the walls for
    // 20 simulated seconds and the kinetic energy stays within 2 percent (it is only 3 steps of slop and float rounding).
    World w;
    w.gravity = {0, 0};
    w.restitution = 1.0F;
    w.friction = 0.0F;
    w.bodies.push_back(Body::box({5.0F, 12.0F}, {20.0F, 1.0F}, 0.0F, true));   // floor, top at y = 11
    w.bodies.push_back(Body::box({5.0F, -12.0F}, {20.0F, 1.0F}, 0.0F, true));  // ceiling, bottom at y = -11
    w.bodies.push_back(Body::box({-11.0F, 0.0F}, {1.0F, 20.0F}, 0.0F, true));  // left wall, face at x = -10
    w.bodies.push_back(Body::box({21.0F, 0.0F}, {1.0F, 20.0F}, 0.0F, true));   // right wall, face at x = 20
    const Vec2 starts[4] = {{0, 0}, {6.0F, 3.0F}, {12.0F, -4.0F}, {3.0F, -7.0F}};
    const Vec2 speeds[4] = {{4.0F, 1.5F}, {-3.0F, 2.0F}, {1.0F, -5.0F}, {-2.5F, 3.5F}};
    const float radii[4] = {0.6F, 0.8F, 0.5F, 0.7F};
    for (int i = 0; i < 4; ++i) {
        Body b = Body::circle(starts[i], radii[i]);
        b.vel = speeds[i];
        w.bodies.push_back(b);
    }
    auto kinetic = [&w] {
        float e = 0.0F;
        for (const Body& b : w.bodies) e += 0.5F * b.mass() * dot(b.vel, b.vel);
        return e;
    };
    const float before = kinetic();
    for (int i = 0; i < 2400; ++i) w.step(1.0F / 120.0F);
    CHECK(std::fabs(kinetic() - before) < 0.02F * before);
}

void pointSpring() {
    // A body without gravity pulled by the spring ends up with its anchor on the target.
    World w;
    w.gravity = {0, 0};
    w.bodies.push_back(Body::box({0, 0}, {0.5F, 0.5F}));
    for (int i = 0; i < 480; ++i) {
        applySpring(w.bodies[0], {0.0F, 0.0F}, {3.0F, 2.0F}, 1.0F / 120.0F);
        w.step(1.0F / 120.0F);
    }
    CHECK(near(w.bodies[0].pos.x, 3.0F, 0.05F) && near(w.bodies[0].pos.y, 2.0F, 0.05F));
}

}  // namespace

int main() {
    circleCircle();
    circleBox();
    boxBox();
    pointTest();
    elasticHeadOn();
    elasticConservation();
    wallReflection();
    inelasticStop();
    frictionSlowsSliding();
    circleComesToRest();
    boxComesToRest();
    stackStaysStanding();
    tallTowerStaysStanding();
    pyramidStaysStanding();
    noEnergyFromNothing();
    energyKeptInARoom();
    pointSpring();
    if (failures != 0) {
        std::printf("%d check(s) failed\n", failures);
        return 1;
    }
    std::printf("all checks passed\n");
    return 0;
}
