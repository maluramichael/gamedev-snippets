// Plain-assert tests for the spatial hash logic. Every expected value is worked out by hand in the comment next to it.
// `spatial-hash-logic --report` additionally prints the pair-test numbers of the demo's start scene (used for the post).
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <utility>
#include <vector>

#include "logic.hpp"

namespace {

int failures = 0;

#define CHECK(cond)                                                     \
    do {                                                                \
        if (!(cond)) {                                                  \
            std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); \
            ++failures;                                                 \
        }                                                               \
    } while (false)

bool near(float a, float b, float eps = 1e-4F) { return std::fabs(a - b) <= eps; }

using namespace sh;

Body circle(float x, float y, float r, float vx = 0.0F, float vy = 0.0F, float mass = 1.0F) {
    Body b;
    b.x = x;
    b.y = y;
    b.r = r;
    b.vx = vx;
    b.vy = vy;
    b.invMass = 1.0F / mass;
    return b;
}

std::uint64_t key(int a, int b) { return (static_cast<std::uint64_t>(std::min(a, b)) << 32) | static_cast<std::uint64_t>(std::max(a, b)); }

/// Independent reference: every overlapping pair by two plain loops, not by the library's own brute force.
std::vector<std::uint64_t> reference(const std::vector<Body>& bodies) {
    std::vector<std::uint64_t> out;
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        for (std::size_t j = i + 1; j < bodies.size(); ++j) {
            const float dx = bodies[j].x - bodies[i].x;
            const float dy = bodies[j].y - bodies[i].y;
            const float rr = bodies[i].r + bodies[j].r;
            if (dx * dx + dy * dy < rr * rr) out.push_back(key(static_cast<int>(i), static_cast<int>(j)));
        }
    }
    std::sort(out.begin(), out.end());
    return out;
}

std::vector<std::uint64_t> found(const World& w) {
    std::vector<std::uint64_t> out;
    for (const Pair& p : w.hits()) out.push_back(key(p.a, p.b));
    std::sort(out.begin(), out.end());
    return out;
}

std::vector<std::uint64_t> unique(std::vector<std::uint64_t> v) {
    v.erase(std::unique(v.begin(), v.end()), v.end());
    return v;
}

// ---- narrow phase ----

void narrowPhase() {
    // Radii 1 and 0.5, 1.5 apart: touching exactly is a miss, 1.4 apart is a hit.
    CHECK(!overlaps(circle(0, 0, 1.0F), circle(1.5F, 0, 0.5F)));
    CHECK(overlaps(circle(0, 0, 1.0F), circle(1.4F, 0, 0.5F)));
    // 3-4-5 triangle: distance 5 against a radius sum of 6 hits, against 5 only touches.
    CHECK(overlaps(circle(0, 0, 3.0F), circle(3, 4, 3.0F)));
    CHECK(!overlaps(circle(0, 0, 2.5F), circle(3, 4, 2.5F)));
    // Same center always overlaps, as long as there is a radius.
    CHECK(overlaps(circle(2, 2, 1.0F), circle(2, 2, 1.0F)));
    CHECK(brutePairs(0) == 0 && brutePairs(1) == 0 && brutePairs(2) == 1 && brutePairs(5) == 10);
    CHECK(brutePairs(2000) == 1999000);
}

// ---- grid ----

void gridCells() {
    // World 100 x 50, cell 10: 10 columns, 5 rows.
    const Rect world{0, 0, 100, 50};
    // A at (15,15) r 4 spans 11..19 on both axes: cell (1,1) only. B at (20,15) r 4 spans x 16..24: cells (1,1) and (2,1).
    std::vector<Body> bodies{circle(15, 15, 4), circle(20, 15, 4)};
    Grid g;
    g.build(bodies, world, 10.0F);
    CHECK(g.cols() == 10 && g.rows() == 5);
    CHECK(g.entries() == 3);
    CHECK(g.cellItems(1, 1).size() == 2);
    CHECK(g.cellItems(1, 1)[0] == 0 && g.cellItems(1, 1)[1] == 1);
    CHECK(g.cellItems(2, 1).size() == 1 && g.cellItems(2, 1)[0] == 1);
    CHECK(g.cellItems(0, 1).empty());
    CHECK(g.cellItems(1, 0).empty());
    // The pair shares only cell (1,1), so it is tested once with and without dedup.
    for (const bool dedup : {true, false}) {
        Counters c;
        int calls = 0;
        g.pairs(dedup, c, [&](int, int) { ++calls; });
        CHECK(c.tests == 1 && calls == 1 && c.skipped == 0);
    }

    // A body that pokes out of the world is clamped into the border cells, 100 + 3 would be column 10 of 10.
    bodies = {circle(0, 0, 5), circle(100, 50, 3)};
    g.build(bodies, world, 10.0F);
    CHECK(g.cellItems(0, 0).size() == 1);
    CHECK(g.cellItems(9, 4).size() == 1);
    CHECK(g.entries() == 2);

    // A cell size that does not divide the world: 100 / 30 rounds up to 4 columns, 50 / 30 to 2 rows.
    g.build(bodies, world, 30.0F);
    CHECK(g.cols() == 4 && g.rows() == 2);
    // Cell sizes below 1 are raised to 1 instead of exploding the table.
    g.build(bodies, world, 0.0F);
    CHECK(g.cols() == 100 && g.rows() == 50);

    // Mouse lookup: (35, 12) is column 3, row 1; outside the world is no cell.
    g.build(bodies, world, 10.0F);
    int cx = -1;
    int cy = -1;
    CHECK(g.cellAt(35.0F, 12.0F, cx, cy) && cx == 3 && cy == 1);
    CHECK(!g.cellAt(-1.0F, 12.0F, cx, cy));
    CHECK(!g.cellAt(35.0F, 51.0F, cx, cy));
}

void gridDuplicates() {
    // Two big circles. A (20,20) r 8 spans 12..28 -> cells 1..2 on both axes. B (22,22) r 8 spans 14..30 -> cells 1..3.
    // They share x 1..2 and y 1..2: four cells. Without dedup the pair shows up four times.
    const std::vector<Body> bodies{circle(20, 20, 8), circle(22, 22, 8)};
    Grid g;
    g.build(bodies, Rect{0, 0, 100, 100}, 10.0F);
    CHECK(g.entries() == 4 + 9);  // A: 2x2 cells, B: 3x3 cells

    Counters raw;
    g.pairs(false, raw, [](int, int) {});
    CHECK(raw.tests == 4 && raw.skipped == 0);

    // With dedup only the first common cell, (max of the lower x indices, max of the lower y indices) = (1,1), keeps it.
    Counters dedup;
    int a = -1;
    int b = -1;
    g.pairs(true, dedup, [&](int i, int j) {
        a = i;
        b = j;
    });
    CHECK(dedup.tests == 1 && dedup.skipped == 3);
    CHECK(a == 0 && b == 1);
}

// ---- quadtree ----

void quadStraddler() {
    // Four small bodies in the corners of a 100 x 100 world and one body on the crossing of the split lines.
    const std::vector<Body> bodies{circle(10, 10, 1), circle(90, 10, 1), circle(10, 90, 1), circle(90, 90, 1), circle(50, 50, 5)};
    Quadtree t;
    t.build(bodies, Rect{0, 0, 100, 100}, 1, 4);
    CHECK(t.node(0).child >= 0);
    CHECK(t.node(0).count == 1);  // only the straddler stays in the root, the corners moved down

    // The straddler's box (45..55) touches all four quadrants: 4 tests, one per corner body. Brute force would need 10.
    Counters c;
    int calls = 0;
    t.pairs(bodies, c, [&](int, int) { ++calls; });
    CHECK(c.tests == 4 && calls == 4);

    // The grid with 50 px cells sees the same: the straddler sits in all four cells, each cell holds one corner body.
    Grid g;
    g.build(bodies, Rect{0, 0, 100, 100}, 50.0F);
    Counters gc;
    g.pairs(true, gc, [](int, int) {});
    CHECK(gc.tests == 4);
}

void quadNeighbors() {
    // Two touching-range bodies in one corner and three far away: the near pair must be tested at least once.
    const std::vector<Body> bodies{circle(10, 10, 1), circle(12, 10, 1.5F), circle(90, 10, 1), circle(10, 90, 1), circle(90, 90, 1)};
    Quadtree t;
    t.build(bodies, Rect{0, 0, 100, 100}, 1, 6);
    Counters c;
    bool nearPair = false;
    t.pairs(bodies, c, [&](int a, int b) {
        if (key(a, b) == key(0, 1)) nearPair = true;
    });
    CHECK(nearPair);
    CHECK(c.tests == 1);  // no other pair is even a candidate
}

void quadDepthBound() {
    // 50 bodies on one spot that does not sit on a split line (30,30 r 1): the tree must stop at maxDepth 4.
    // Rect sizes 100, 50, 25, 12.5, 6.25: at depth 4 (6.25 wide) the bodies cannot go lower, they all stay there.
    std::vector<Body> bodies;
    for (int i = 0; i < 50; ++i) bodies.push_back(circle(30, 30, 1));
    Quadtree t;
    t.build(bodies, Rect{0, 0, 100, 100}, 8, 4);
    CHECK(t.deepest() == 4);
    Counters c;
    t.pairs(bodies, c, [](int, int) {});
    CHECK(c.tests == 50 * 49 / 2);
    // All 50 share the leaf, so the tree degrades to brute force there. The grid does the same inside one cell.
    Grid g;
    g.build(bodies, Rect{0, 0, 100, 100}, 10.0F);
    Counters gc;
    g.pairs(true, gc, [](int, int) {});
    CHECK(gc.tests == 1225);
}

void quadOutside() {
    // A sticks out of the world on the left (box -7..3), B sits next to it. A fits the upper-left quadrant by its
    // coordinates, but it is not contained in that quadrant's rectangle, so it has to stay in the root and still pair up.
    const std::vector<Body> bodies{circle(-2, 20, 5), circle(3, 20, 2)};
    Quadtree t;
    t.build(bodies, Rect{0, 0, 100, 100}, 1, 4);
    CHECK(t.node(0).child >= 0);
    CHECK(t.node(0).count == 1);
    Counters c;
    bool seen = false;
    t.pairs(bodies, c, [&](int a, int b) { seen = seen || key(a, b) == key(0, 1); });
    CHECK(seen);
}

// ---- all three engines find the same contacts ----

World denseWorld(int n, bool big, std::uint32_t seed) {
    World w;
    w.bounds = Rect{0, 0, 200, 150};
    w.populate(n, big, seed);
    return w;
}

void enginesAgree() {
    for (const std::uint32_t seed : {1U, 2U, 7U}) {
        World w = denseWorld(300, true, seed);
        const std::vector<std::uint64_t> expected = reference(w.bodies);
        CHECK(expected.size() > 50);  // the scene is dense enough to mean something

        Counters brute = w.findContacts(Mode::Brute, 16, true);
        CHECK(found(w) == expected);
        CHECK(brute.tests == 300 * 299 / 2);

        for (const int cell : {4, 8, 16, 50, 500}) {
            Counters withDedup = w.findContacts(Mode::Grid, cell, true);
            CHECK(found(w) == expected);
            CHECK(withDedup.tests <= brute.tests);
            Counters raw = w.findContacts(Mode::Grid, cell, false);
            CHECK(unique(found(w)) == expected);  // same pairs, only more often
            CHECK(raw.tests == withDedup.tests + withDedup.skipped);
            CHECK(raw.tests >= withDedup.tests);
        }
        for (const std::pair<int, int>& shape :{std::pair{8, 8}, std::pair{1, 3}, std::pair{4, 6}, std::pair{64, 2}}) {
            w.quadCapacity = shape.first;
            w.quadDepth = shape.second;
            Counters q = w.findContacts(Mode::Quad, 16, true);
            CHECK(found(w) == expected);
            CHECK(q.tests <= brute.tests);
        }
    }
}

void gridBeatsBruteForce() {
    // The reason for the whole exercise, on the demo's start scene: 2000 bodies, brute force needs 1,999,000 tests.
    World w;
    w.bounds = defaultField();
    w.populate(2000, true, kSeed);
    const Counters brute = w.findContacts(Mode::Brute, 16, true);
    CHECK(brute.tests == 1999000);
    const Counters grid = w.findContacts(Mode::Grid, 16, true);
    const Counters quad = w.findContacts(Mode::Quad, 16, true);
    CHECK(grid.tests * 50 < brute.tests);  // less than 2 percent
    CHECK(quad.tests * 20 < brute.tests);
}

// ---- physics ----

void resolveHeadOn() {
    // Equal masses, r 1, centers 1.5 apart (overlap 0.5), approaching at 1 each: elastic hit swaps the velocities.
    Body a = circle(0, 0, 1, 1, 0);
    Body b = circle(1.5F, 0, 1, -1, 0);
    CHECK(resolve(a, b));
    CHECK(near(a.vx, -1.0F) && near(b.vx, 1.0F));
    // Each moves 0.25 apart: centers end exactly 2 apart, which is touching, no longer overlapping.
    CHECK(near(a.x, -0.25F) && near(b.x, 1.75F));
    CHECK(!overlaps(a, b));
    CHECK(!resolve(a, b));  // second call: nothing left to do

    // Mass 1 hits mass 2 at rest, v = 3: 1D elastic gives (m1-m2)/(m1+m2)*3 = -1 and 2*m1/(m1+m2)*3 = 2.
    Body c = circle(0, 0, 1, 3, 0, 1.0F);
    Body d = circle(1.9F, 0, 1, 0, 0, 2.0F);
    CHECK(resolve(c, d));
    CHECK(near(c.vx, -1.0F) && near(d.vx, 2.0F));
    CHECK(near(1.0F * 3.0F, 1.0F * c.vx + 2.0F * d.vx));  // momentum 3 before, -1 + 4 after
    // The lighter body takes 2/3 of the push: overlap 0.1 -> c moves 0.0667 back, d 0.0333 forward.
    CHECK(near(c.x, -0.1F * 2.0F / 3.0F) && near(d.x, 1.9F + 0.1F / 3.0F));
}

void resolveSeparating() {
    // Overlapping but already moving apart: positions are corrected, velocities stay.
    Body a = circle(0, 0, 1, -1, 0);
    Body b = circle(1.5F, 0, 1, 1, 0);
    CHECK(resolve(a, b));
    CHECK(near(a.vx, -1.0F) && near(b.vx, 1.0F));
    CHECK(near(b.x - a.x, 2.0F));
    // Sliding past each other (relative velocity along the normal is 0): nothing changes in the velocities.
    Body c = circle(0, 0, 1, 0, 2);
    Body d = circle(1.5F, 0, 1, 0, -2);
    CHECK(resolve(c, d));
    CHECK(near(c.vy, 2.0F) && near(d.vy, -2.0F));
}

void resolveDegenerate() {
    // Same center: no division by zero, the normal falls back to +x and the pair is pushed 1 each way (r sum 2).
    Body a = circle(5, 5, 1);
    Body b = circle(5, 5, 1);
    CHECK(resolve(a, b));
    CHECK(std::isfinite(a.x) && std::isfinite(b.x));
    CHECK(near(a.x, 4.0F) && near(b.x, 6.0F) && near(a.y, 5.0F));
    // Two bodies without a mass (invMass 0) cannot be moved; the call must not divide by zero.
    Body s = circle(0, 0, 1);
    Body t = circle(1, 0, 1);
    s.invMass = 0.0F;
    t.invMass = 0.0F;
    CHECK(!resolve(s, t));
    CHECK(near(s.x, 0.0F) && near(t.x, 1.0F));
}

void walls() {
    World w;
    w.bounds = Rect{0, 0, 100, 50};
    // Left wall: r 2 at x = -1 moving left ends at x = 2 moving right. The corner flips both axes.
    w.bodies = {circle(-1, 25, 2, -5, 0), circle(1, 1, 2, -3, -4), circle(99, 49, 2, 3, 4)};
    w.integrate(0.0F);
    CHECK(near(w.bodies[0].x, 2.0F) && near(w.bodies[0].vx, 5.0F));
    CHECK(near(w.bodies[1].x, 2.0F) && near(w.bodies[1].y, 2.0F) && near(w.bodies[1].vx, 3.0F) && near(w.bodies[1].vy, 4.0F));
    CHECK(near(w.bodies[2].x, 98.0F) && near(w.bodies[2].y, 48.0F) && near(w.bodies[2].vx, -3.0F) && near(w.bodies[2].vy, -4.0F));
    // Free flight: 10 px/s for 0.5 s is 5 px.
    w.bodies = {circle(50, 25, 2, 10, 0)};
    w.integrate(0.5F);
    CHECK(near(w.bodies[0].x, 55.0F));
}

double energy(const World& w) {
    double e = 0.0;
    for (const Body& b : w.bodies) e += 0.5 * (1.0 / static_cast<double>(b.invMass)) * static_cast<double>(b.vx * b.vx + b.vy * b.vy);
    return e;
}

void simulationKeepsEnergy() {
    // Elastic hits and elastic walls: the kinetic energy stays put, no matter which engine finds the contacts.
    for (const Mode mode : {Mode::Brute, Mode::Grid, Mode::Quad}) {
        World w = denseWorld(200, true, 3);
        const double before = energy(w);
        for (int i = 0; i < 300; ++i) w.step(1.0F / 60.0F, mode, 12, true);
        const double after = energy(w);
        CHECK(std::fabs(after - before) < 1e-3 * before);
        for (const Body& b : w.bodies) CHECK(std::isfinite(b.x) && std::isfinite(b.y) && b.x > -25.0F && b.x < 225.0F && b.y > -25.0F && b.y < 175.0F);
    }
}

void populate() {
    World a;
    a.bounds = defaultField();
    a.populate(2000, true, 1);
    CHECK(a.bodies.size() == 2000);
    int big = 0;
    for (const Body& b : a.bodies) {
        if (b.r > 6.0F) ++big;
        constexpr float tol = 1e-3F;
        CHECK(b.x - b.r >= a.bounds.x0 - tol && b.x + b.r <= a.bounds.x1 + tol && b.y - b.r >= a.bounds.y0 - tol && b.y + b.r <= a.bounds.y1 + tol);
        CHECK(b.invMass > 0.0F);
    }
    CHECK(big == 25);  // 2000 / 80
    // Same seed, same scene; another seed, another scene.
    World b;
    b.bounds = a.bounds;
    b.populate(2000, true, 1);
    CHECK(a.bodies[0].x == b.bodies[0].x && a.bodies[1999].y == b.bodies[1999].y);
    b.populate(2000, true, 2);
    CHECK(a.bodies[0].x != b.bodies[0].x);
    // No big bodies on request; 500 gets 6 of them (500 / 80), 0 bodies is fine.
    a.populate(2000, false, 1);
    CHECK(std::none_of(a.bodies.begin(), a.bodies.end(), [](const Body& x) { return x.r > 6.0F; }));
    a.populate(500, true, 1);
    CHECK(std::count_if(a.bodies.begin(), a.bodies.end(), [](const Body& x) { return x.r > 6.0F; }) == 6);
    a.populate(0, true, 1);
    CHECK(a.bodies.empty());
    Counters c = a.step(1.0F / 60.0F, Mode::Quad, 16, true);
    CHECK(c.tests == 0);
}

// ---- formatting and the slider ----

void formatting() {
    CHECK(formatInt(0) == "0");
    CHECK(formatInt(999) == "999");
    CHECK(formatInt(1000) == "1,000");
    CHECK(formatInt(1999000) == "1,999,000");
    CHECK(formatInt(-12345) == "-12,345");
    CHECK(formatFixed2(0.0) == "0.00");
    CHECK(formatFixed2(25.0) == "25.00");
    CHECK(formatFixed2(0.375) == "0.38");
    CHECK(formatFixed2(100.0) == "100.00");
    CHECK(formatFixed2(1234.5) == "1234.50");
    CHECK(formatFixed2(-1.5) == "-1.50");
    CHECK(formatFixed2(0.004) == "0.00");
}

void slider() {
    CHECK(cellFromT(0.0F) == 4 && cellFromT(1.0F) == 128);
    CHECK(cellFromT(-3.0F) == 4 && cellFromT(9.0F) == 128);
    CHECK(cellFromT(std::nanf("")) == 4);
    CHECK(cellFromT(0.5F) == 23);  // 4 * sqrt(32) = 22.63
    CHECK(near(tFromCell(4), 0.0F) && near(tFromCell(128), 1.0F));
    for (int c = kMinCell; c <= kMaxCell; ++c) CHECK(cellFromT(tFromCell(c)) == c);
    CHECK(stepCell(4, -1) == 4 && stepCell(128, 1) == 128);
    CHECK(stepCell(4, 1) == 5);
    CHECK(stepCell(10, 1) == 11);   // 10 * 1.12 = 11.2
    CHECK(stepCell(16, 1) == 18);   // 17.92
    CHECK(stepCell(16, -1) == 14);  // 14.29
    CHECK(stepCell(5, -1) == 4);
    CHECK(stepCell(100, 1) == 112);
}

void report() {
    World w;
    w.bounds = defaultField();
    w.populate(2000, true, kSeed);
    const Counters brute = w.findContacts(Mode::Brute, 16, true);
    std::printf("start scene: 2000 bodies (%zu overlapping pairs), seed %u\n", w.hits().size(), kSeed);
    std::printf("brute force: %ld tests\n", brute.tests);
    for (const int cell : {4, 6, 8, 12, 16, 24, 32, 64, 128}) {
        const Counters raw = w.findContacts(Mode::Grid, cell, false);
        const Counters c = w.findContacts(Mode::Grid, cell, true);
        std::printf("grid %3d px: %8ld tests (%ld without dedup), %7zu entries, sum %8ld, cells %dx%d\n", cell, c.tests, raw.tests, w.grid().entries(),
                    c.tests + static_cast<long>(w.grid().entries()), w.grid().cols(), w.grid().rows());
    }
    const Counters q = w.findContacts(Mode::Quad, 16, true);
    std::printf("quadtree: %ld tests, %zu nodes, depth %d\n", q.tests, w.tree().nodeCount(), w.tree().deepest());
    World nobig;
    nobig.bounds = defaultField();
    nobig.populate(2000, false, kSeed);
    for (const int cell : {4, 8, 16, 32}) {
        const Counters c = nobig.findContacts(Mode::Grid, cell, true);
        std::printf("grid %3d px, no big bodies: %8ld tests, %7zu entries\n", cell, c.tests, nobig.grid().entries());
    }
}

// ---- contact glow: a contact lasts only a step or two, so a binary highlight flickers at 60 Hz ----

void contactGlow() {
    constexpr float dt = 1.0F / 60.0F;
    // The glow rises toward 1 while a circle touches: after one rise time constant it has covered 1 - exp(-1) = 0.632121.
    CHECK(near(stepHeat(0.0F, true, kHeatRiseTau), 0.632121F, 1e-4F));
    // One 60 Hz step of contact is deliberately a small move, 1 - exp(-(1/60)/0.1) = 0.153518, not a jump to full.
    CHECK(near(stepHeat(0.0F, true, dt), 0.153518F, 1e-4F));
    CHECK(near(stepHeat(1.0F, true, dt), 1.0F));
    // Without contact the glow decays: after one decay time constant it is exp(-1) = 0.367879.
    CHECK(near(stepHeat(1.0F, false, kHeatTau), 0.367879F, 1e-4F));
    // No time passes, nothing changes. A negative or NaN dt must not brighten or darken either.
    CHECK(near(stepHeat(0.5F, false, 0.0F), 0.5F));
    CHECK(near(stepHeat(0.5F, true, 0.0F), 0.5F));
    CHECK(near(stepHeat(0.5F, false, -1.0F), 0.5F));
    CHECK(near(stepHeat(0.5F, true, std::nanf("")), 0.5F));
    // Never leaves [0, 1], also for out-of-range input.
    CHECK(stepHeat(7.0F, false, dt) <= 1.0F);
    CHECK(stepHeat(7.0F, true, dt) <= 1.0F);
    CHECK(stepHeat(-3.0F, false, dt) >= 0.0F);
    CHECK(stepHeat(-3.0F, true, dt) >= 0.0F);
    CHECK(stepHeat(0.0F, false, dt) == 0.0F);
    // The point of it: a circle that touches every other step used to switch between two colors every frame.
    // Now it settles into a narrow band (hand-solved fixed point: about 0.797 after a touch, 0.760 after a gap).
    float h = 0.0F;
    for (int i = 0; i < 600; ++i) h = stepHeat(h, i % 2 == 0, dt);
    const float afterGap = h;
    h = stepHeat(h, true, dt);
    CHECK(near(h, 0.797F, 0.01F));
    CHECK(near(afterGap, 0.760F, 0.01F));
    CHECK(h - afterGap < 0.05F);
    // No single step, rising or falling, moves the color by more than a fifth of the range.
    for (const float start : {0.0F, 0.3F, 0.7F, 1.0F}) {
        CHECK(std::fabs(stepHeat(start, true, dt) - start) <= 0.2F);
        CHECK(std::fabs(stepHeat(start, false, dt) - start) <= 0.2F);
    }
    // After one second without contact it is mostly gone (exp(-1/0.35) = 0.0574).
    h = 1.0F;
    for (int i = 0; i < 60; ++i) h = stepHeat(h, false, dt);
    CHECK(h < 0.07F);
}

}  // namespace

int main(int argc, char** argv) {
    narrowPhase();
    gridCells();
    gridDuplicates();
    quadStraddler();
    quadNeighbors();
    quadDepthBound();
    quadOutside();
    enginesAgree();
    gridBeatsBruteForce();
    resolveHeadOn();
    resolveSeparating();
    resolveDegenerate();
    walls();
    simulationKeepsEnergy();
    populate();
    formatting();
    slider();
    contactGlow();
    if (argc > 1 && std::strcmp(argv[1], "--report") == 0) report();
    if (failures != 0) {
        std::printf("%d check(s) failed\n", failures);
        return 1;
    }
    std::printf("spatial-hash logic: all checks passed\n");
    return 0;
}
