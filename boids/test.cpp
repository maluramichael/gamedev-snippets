// Tests for the boids logic. Plain asserts on hand-computed cases, no framework, no SDL.
// The world is a torus, so most cases come in pairs: the same situation in the middle and across the seam.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "logic.hpp"

using namespace boids;

namespace {

bool near(float a, float b, float eps = 1e-3F) { return std::fabs(a - b) <= eps; }
bool near(Vec2 a, Vec2 b, float eps = 1e-3F) { return near(a.x, b.x, eps) && near(a.y, b.y, eps); }

// Big force limit so the limit does not hide the rule under test; speeds chosen for easy arithmetic.
Params testParams() {
    Params p;
    p.perception = 60.0F;
    p.separationRadius = 24.0F;
    p.maxSpeed = 100.0F;
    p.minSpeed = 50.0F;
    p.maxForce = 1000.0F;
    p.fearRadius = 40.0F;
    p.fearBoost = 4.0F;
    p.separation = 1.0F;
    p.alignment = 1.0F;
    p.cohesion = 1.0F;
    return p;
}

Params only(float s, float a, float c) {
    Params p = testParams();
    p.separation = s;
    p.alignment = a;
    p.cohesion = c;
    return p;
}

const World kBig{1000.0F, 1000.0F};

Sums gather(const Bird& me, const std::vector<Bird>& others, const World& w, const Params& p) {
    Sums s;
    for (const Bird& o : others) accumulate(s, me, o, w, p);
    return s;
}

Vec2 steer(const Bird& me, const std::vector<Bird>& others, const World& w, const Params& p) {
    return flockSteering(me, gather(me, others, w, p), p);
}

// ---- vector helpers -------------------------------------------------------------------------------------------

void limitedAndNormalized() {
    assert(near(limited({3, 4}, 10), {3, 4}));    // shorter than the limit: unchanged
    assert(near(limited({6, 8}, 5), {3, 4}));     // 10 long, cut to 5 in the same direction
    assert(near(limited({1, 1}, 0), {0, 0}));     // limit 0 means no force at all
    assert(near(limited({0, 0}, 5), {0, 0}));     // zero stays zero, no division
    assert(near(normalized({0, 5}), {0, 1}));
    assert(near(normalized({0, 0}), {0, 0}));     // zero vector: no NaN
    assert(near(normalized({3, 4}), {0.6F, 0.8F}));
}

void wrapping() {
    assert(near(wrapCoord(5, 10), 5));
    assert(near(wrapCoord(-1, 10), 9));
    assert(near(wrapCoord(10, 10), 0));  // the right edge itself is the left edge
    assert(near(wrapCoord(23, 10), 3));
    const float tiny = wrapCoord(-1e-9F, 10);  // 10 - 1e-9 rounds to 10.0F, which must come back as 0
    assert(tiny >= 0.0F && tiny < 10.0F);
    // shortest signed way from a to b
    assert(near(wrapDelta(1, 9, 10), -2));
    assert(near(wrapDelta(9, 1, 10), 2));
    assert(near(wrapDelta(2, 5, 10), 3));
    assert(near(wrapDelta(0, 5, 10), 5));   // exactly half: stays, both ways are equally long
    assert(near(wrapDelta(5, 0, 10), -5));
}

void clampingSpeed() {
    const Params p = testParams();  // 50 .. 100
    assert(near(clampSpeed({300, 400}, p), {60, 80}));  // 500 -> 100
    assert(near(clampSpeed({3, 4}, p), {30, 40}));      // 5 -> 50
    assert(near(clampSpeed({60, 80}, p), {60, 80}));    // exactly max: unchanged
    assert(near(clampSpeed({30, 40}, p), {30, 40}));    // exactly min: unchanged
    assert(near(clampSpeed({0, 0}, p), {50, 0}));       // standing still: picks a direction instead of dividing
}

// ---- the three rules ------------------------------------------------------------------------------------------

void separationPushesAway() {
    const Bird a{{100, 100}, {0, 0}};
    const Bird b{{104, 100}, {0, 0}};
    // d = 4 < 24: away from b, full desired speed 100, no own velocity to subtract
    assert(near(steer(a, {b}, kBig, only(1, 0, 0)), {-100, 0}));
    // seen from b the push points the other way
    assert(near(steer(b, {a}, kBig, only(1, 0, 0)), {100, 0}));
    // above and below work the same, y is not special
    const Bird c{{100, 96}, {0, 0}};
    assert(near(steer(a, {c}, kBig, only(1, 0, 0)), {0, 100}));
    // weight 0 switches the rule off
    assert(near(steer(a, {b}, kBig, only(0, 0, 0)), {0, 0}));
}

void separationRadiusBoundary() {
    const Params p = only(1, 0, 0);
    const Bird a{{100, 100}, {0, 0}};
    assert(near(steer(a, {Bird{{123.9F, 100}, {0, 0}}}, kBig, p), {-100, 0}));  // 23.9 < 24: pushes
    assert(near(steer(a, {Bird{{124.0F, 100}, {0, 0}}}, kBig, p), {0, 0}));     // exactly 24: does not
    assert(near(steer(a, {Bird{{140.0F, 100}, {0, 0}}}, kBig, p), {0, 0}));     // 40: outside, inside perception
}

void separationWeighsCloseNeighborsMore() {
    // Two neighbors on opposite sides: the nearer one (distance 4 against 8) wins, 1/4 against 1/8.
    const Bird a{{100, 100}, {0, 0}};
    const Bird near1{{104, 100}, {0, 0}};
    const Bird far1{{92, 100}, {0, 0}};
    const Vec2 s = steer(a, {near1, far1}, kBig, only(1, 0, 0));
    assert(s.x < 0.0F);  // pushed away from the near one, to the left
    assert(near(s, {-100, 0}));
}

void alignmentMatchesNeighbors() {
    const Bird a{{0, 0}, {100, 0}};
    const Bird b{{10, 0}, {0, 100}};
    // desired = heading of b at full speed (0,100), minus own velocity (100,0)
    assert(near(steer(a, {b}, kBig, only(0, 1, 0)), {-100, 100}));
    // same heading, nothing to correct for the direction; speed 100 equals max, so zero
    const Bird c{{10, 0}, {100, 0}};
    assert(near(steer(a, {c}, kBig, only(0, 1, 0)), {0, 0}));
    // a slower neighbor with the same heading still asks for full speed: (100,0) - (50,0)
    const Bird slow{{0, 0}, {50, 0}};
    assert(near(steer(slow, {c}, kBig, only(0, 1, 0)), {50, 0}));
}

void cohesionSeeksTheCenter() {
    const Bird a{{0, 0}, {0, 0}};
    const Bird b{{10, 0}, {0, 0}};
    const Bird c{{30, 0}, {0, 0}};
    // center of b and c is (20,0); slowing radius is half the perception = 30, so desired speed is 100 * 20/30
    assert(near(steer(a, {b, c}, kBig, only(0, 0, 1)), {66.6667F, 0}));
    // outside the slowing radius the desired speed is the maximum
    const Bird d{{45, 0}, {0, 0}};
    assert(near(steer(a, {d}, kBig, only(0, 0, 1)), {100, 0}));
    // sitting exactly in the middle: nothing to seek, and no NaN
    const Bird mid{{50, 0}, {0, 0}};
    const Bird l{{40, 0}, {0, 0}};
    const Bird r{{60.0F - 0.001F, 0}, {0, 0}};
    assert(near(steer(mid, {l, r}, kBig, only(0, 0, 1)), {0, 0}, 0.1F));
}

void perceptionBoundary() {
    const Params p = only(0, 1, 1);
    const Bird a{{100, 100}, {0, 0}};
    assert(near(steer(a, {Bird{{160.0F, 100}, {0, 100}}}, kBig, p), {0, 0}));     // exactly 60: not a neighbor
    assert(!near(steer(a, {Bird{{159.9F, 100}, {0, 100}}}, kBig, p), {0, 0}));    // 59.9: is one
    assert(near(steer(a, {}, kBig, p), {0, 0}));                                  // alone: nothing
}

void limitAppliesPerRule() {
    Params p = only(1, 0, 0);
    p.maxForce = 30.0F;
    const Bird a{{100, 100}, {0, 0}};
    const Bird b{{104, 100}, {0, 0}};
    assert(near(steer(a, {b}, kBig, p), {-30, 0}));  // wanted 100, gets 30
}

void rulesAddUp() {
    // Alone in the middle of two neighbors that the three rules ask different things from. The sum of the weighted rules:
    const Bird a{{0, 0}, {0, 0}};
    const Bird b{{10, 0}, {0, 0}};
    const Params all = testParams();
    const Vec2 sep = steer(a, {b}, kBig, only(1, 0, 0));
    const Vec2 coh = steer(a, {b}, kBig, only(0, 0, 1));
    assert(near(steer(a, {b}, kBig, all), {sep.x + coh.x, sep.y + coh.y}));
    // doubling a weight doubles its share
    assert(near(steer(a, {b}, kBig, only(2, 0, 0)), {sep.x * 2.0F, sep.y * 2.0F}));
}

void worldIsATorus() {
    const World w{100.0F, 100.0F};
    const Bird a{{1, 50}, {0, 0}};
    const Bird b{{99, 50}, {0, 0}};
    // across the seam b is 2 to the left: separation pushes a to the right, cohesion pulls it left
    assert(near(steer(a, {b}, w, only(1, 0, 0)), {100, 0}));
    assert(steer(a, {b}, w, only(0, 0, 1)).x < 0.0F);
    // seen from b, a is 2 to the right
    assert(near(steer(b, {a}, w, only(1, 0, 0)), {-100, 0}));
    // same in y
    const Bird c{{50, 1}, {0, 0}};
    const Bird d{{50, 99}, {0, 0}};
    assert(near(steer(c, {d}, w, only(1, 0, 0)), {0, 100}));
}

void coincidentBirdsDoNotProduceNaN() {
    const Bird a{{50, 50}, {10, 0}};
    const Bird b{{50, 50}, {0, 10}};
    const Vec2 s = steer(a, {b}, kBig, testParams());
    assert(std::isfinite(s.x) && std::isfinite(s.y));
}

// ---- predator -------------------------------------------------------------------------------------------------

void fleeFromPredator() {
    const Params p = testParams();  // fear radius 40, boost 4
    const Bird me{{140, 100}, {0, 0}};
    const Predator pr{{150, 100}, true};
    // 10 away: direction -x, desired (-100,0), strength 1 - 10/40 = 0.75
    assert(inFear(me, pr, kBig, p));
    assert(near(fleeSteering(me, pr, kBig, p), {-75, 0}));
    // exactly on the radius: not afraid, no force
    const Predator edge{{180, 100}, true};
    assert(!inFear(me, edge, kBig, p));
    assert(near(fleeSteering(me, edge, kBig, p), {0, 0}));
    // just inside: a tiny force, pointing away
    const Predator justIn{{179.9F, 100}, true};
    assert(inFear(me, justIn, kBig, p));
    const Vec2 f = fleeSteering(me, justIn, kBig, p);
    assert(f.x < 0.0F && f.x > -1.0F);
    // inactive predator does nothing, wherever it is
    const Predator off{{150, 100}, false};
    assert(!inFear(me, off, kBig, p));
    assert(near(fleeSteering(me, off, kBig, p), {0, 0}));
}

void fleeAcrossTheSeam() {
    const Params p = testParams();
    const Bird me{{5, 100}, {0, 0}};
    const Predator pr{{995, 100}, true};  // 10 to the left across the seam
    assert(near(fleeSteering(me, pr, kBig, p), {75, 0}));
}

void fleeOnTopOfThePredator() {
    const Params p = testParams();
    const Bird me{{100, 100}, {0, 50}};
    const Predator pr{{100, 100}, true};
    // no direction to flee along: keep going the way it already moves, full strength (distance 0)
    assert(near(fleeSteering(me, pr, kBig, p), {0, 50}));
    const Bird still{{100, 100}, {0, 0}};
    const Vec2 f = fleeSteering(still, pr, kBig, p);
    assert(std::isfinite(f.x) && std::isfinite(f.y));
}

void fleeForceUsesTheBoost() {
    Params p = testParams();
    p.maxForce = 10.0F;  // boost 4 -> flee may use 40
    const Bird me{{100, 100}, {0, 0}};
    const Predator pr{{101, 100}, true};  // 1 away, strength 0.975, wanted 100 -> limited to 40, scaled
    assert(near(fleeSteering(me, pr, kBig, p), {-40.0F * 0.975F, 0}));
}

// ---- grid -----------------------------------------------------------------------------------------------------

void gridCells() {
    Grid g;
    std::vector<Bird> none;
    g.build(none, World{200.0F, 100.0F}, 60.0F);
    assert(g.cols() == 3 && g.rows() == 1);  // floor(200/60) = 3, cells are 66.7 wide, never narrower than 60
    assert(g.cellX(0.0F) == 0);
    assert(g.cellX(66.0F) == 0);
    assert(g.cellX(67.0F) == 1);
    assert(g.cellX(199.9F) == 2);
    assert(g.cellX(200.0F) == 2);        // edge value clamps to the last cell
    assert(g.cellX(-1.0F) == 0);
    assert(g.cellX(std::nanf("")) == 0);  // no cast of NaN to int
    assert(g.cellY(50.0F) == 0);
    g.build(none, World{960.0F, 370.0F}, 60.0F);
    assert(g.cols() == 16 && g.rows() == 6);
    g.build(none, World{10.0F, 10.0F}, 60.0F);
    assert(g.cols() == 1 && g.rows() == 1);  // world smaller than one cell
    g.build(none, World{1e12F, 1e12F}, 1.0F);
    assert(g.cols() == 1024 && g.rows() == 1024);  // capped instead of an overflowing cast
    g.build(none, World{100.0F, 100.0F}, 0.0F);
    assert(g.cols() == 100 && g.rows() == 100);  // a cell size of 0 is treated as 1
}

// Every bird must be offered exactly once, also when the world has fewer than three cells in a direction.
void gridOffersEveryBirdOnceWhenSmall() {
    const World worlds[] = {{100.0F, 100.0F}, {130.0F, 70.0F}, {190.0F, 130.0F}, {960.0F, 370.0F}};
    for (const World& w : worlds) {
        Flock f(w);
        Params p = testParams();
        f.reset(60, 3, p);
        Grid g;
        g.build(f.birds(), w, p.perception);
        for (std::size_t i = 0; i < f.birds().size(); ++i) {
            std::vector<int> seen(f.birds().size(), 0);
            std::size_t offered = 0;
            // 3x3 block around the bird's own cell reaches everything in a world of up to 3x3 cells
            g.forEachCandidate(f.birds()[i].pos, [&](int j) {
                ++seen[static_cast<std::size_t>(j)];
                ++offered;
            });
            for (int s : seen) assert(s <= 1);  // never twice
            assert(seen[i] == 1);               // includes itself
            if (g.cols() <= 3 && g.rows() <= 3) assert(offered == f.birds().size());
        }
    }
}

// The grid may offer more birds than needed, but never miss one that the brute-force test accepts.
void gridFindsEveryNeighbor() {
    const World worlds[] = {{100.0F, 100.0F}, {130.0F, 70.0F}, {200.0F, 100.0F}, {960.0F, 370.0F}};
    for (const World& w : worlds) {
        Flock f(w);
        Params p = testParams();
        f.reset(150, 11, p);
        Grid g;
        g.build(f.birds(), w, p.perception);
        const float r2 = p.perception * p.perception;
        for (std::size_t i = 0; i < f.birds().size(); ++i) {
            int brute = 0;
            for (std::size_t j = 0; j < f.birds().size(); ++j) {
                if (j == i) continue;
                const Vec2 d{wrapDelta(f.birds()[i].pos.x, f.birds()[j].pos.x, w.width), wrapDelta(f.birds()[i].pos.y, f.birds()[j].pos.y, w.height)};
                if (lengthSq(d) < r2) ++brute;
            }
            int viaGrid = 0;
            g.forEachCandidate(f.birds()[i].pos, [&](int j) {
                if (static_cast<std::size_t>(j) == i) return;
                const Vec2 d{wrapDelta(f.birds()[i].pos.x, f.birds()[static_cast<std::size_t>(j)].pos.x, w.width),
                             wrapDelta(f.birds()[i].pos.y, f.birds()[static_cast<std::size_t>(j)].pos.y, w.height)};
                if (lengthSq(d) < r2) ++viaGrid;
            });
            assert(viaGrid == brute);
        }
    }
}

void gridAcrossTheSeam() {
    const World w{960.0F, 370.0F};
    Flock f(w);
    f.reset(0, 1, testParams());
    f.birds().push_back(Bird{{1, 50}, {0, 0}});
    f.birds().push_back(Bird{{959, 50}, {0, 0}});
    const Params p = only(1, 0, 0);
    f.computeAccelerations(p, Predator{}, Search::Grid);
    assert(near(f.accel()[0], {100, 0}));   // pushed right, away from the bird 2 px to its left across the seam
    assert(near(f.accel()[1], {-100, 0}));
}

void gridAndBruteForceAgree() {
    const World worlds[] = {{100.0F, 100.0F}, {960.0F, 370.0F}};
    for (const World& w : worlds) {
        Flock f(w);
        Params p = testParams();
        p.maxForce = 260.0F;
        f.reset(300, 5, p);
        const Predator pr{{w.width * 0.5F, w.height * 0.5F}, true};
        f.computeAccelerations(p, pr, Search::Brute);
        const std::vector<Vec2> brute = f.accel();
        const std::vector<char> scaredBrute = [&] {
            std::vector<char> v;
            for (const Bird& b : f.birds()) v.push_back(b.scared ? 1 : 0);
            return v;
        }();
        const long long bruteTests = f.lastTests();
        f.computeAccelerations(p, pr, Search::Grid);
        assert(f.accel().size() == brute.size());
        for (std::size_t i = 0; i < brute.size(); ++i) {
            assert(near(f.accel()[i], brute[i], 1e-2F));  // same neighbors, only the summation order differs
            assert(f.birds()[i].scared == (scaredBrute[i] != 0));
        }
        assert(bruteTests == 300LL * 299LL);  // every bird against every other
        assert(f.lastTests() <= bruteTests);
        if (w.width > 500.0F) assert(f.lastTests() * 4 < bruteTests);  // the whole point: far fewer tests
    }
}

// ---- flock ----------------------------------------------------------------------------------------------------

void resetBasics() {
    const World w{960.0F, 370.0F};
    const Params p = testParams();
    Flock f(w);
    f.reset(100, 7, p);
    assert(f.birds().size() == 100);
    for (const Bird& b : f.birds()) {
        assert(b.pos.x >= 0.0F && b.pos.x < w.width);
        assert(b.pos.y >= 0.0F && b.pos.y < w.height);
        const float s = length(b.vel);
        assert(s >= p.minSpeed - 1e-3F && s <= p.maxSpeed + 1e-3F);
    }
    f.reset(0, 7, p);
    assert(f.birds().empty());
    f.reset(-3, 7, p);
    assert(f.birds().empty());
    f.reset(1000000, 7, p);
    assert(f.birds().size() == Flock::kMaxBirds);  // capped
    // same seed, same flock; another seed, another flock
    Flock g(w);
    f.reset(50, 42, p);
    g.reset(50, 42, p);
    for (std::size_t i = 0; i < 50; ++i) assert(near(f.birds()[i].pos, g.birds()[i].pos, 0.0F));
    g.reset(50, 43, p);
    assert(!near(f.birds()[0].pos, g.birds()[0].pos, 1e-3F));
}

void changingTheCount() {
    const World w{960.0F, 370.0F};
    const Params p = testParams();
    Flock f(w);
    f.reset(100, 7, p);
    const std::vector<Bird> before = f.birds();
    f.setCount(60, p);
    assert(f.birds().size() == 60);
    for (std::size_t i = 0; i < 60; ++i) assert(near(f.birds()[i].pos, before[i].pos, 0.0F) && near(f.birds()[i].vel, before[i].vel, 0.0F));  // the survivors are untouched
    f.setCount(150, p);
    assert(f.birds().size() == 150);
    for (std::size_t i = 0; i < 60; ++i) assert(near(f.birds()[i].pos, before[i].pos, 0.0F));
    for (std::size_t i = 60; i < 150; ++i) {  // newcomers: inside the world, speed in range
        const Bird& b = f.birds()[i];
        assert(b.pos.x >= 0.0F && b.pos.x < w.width && b.pos.y >= 0.0F && b.pos.y < w.height);
        assert(length(b.vel) >= p.minSpeed - 1e-3F && length(b.vel) <= p.maxSpeed + 1e-3F);
    }
    assert(!near(f.birds()[60].pos, f.birds()[61].pos, 1e-3F));  // not all on the same spot
    f.setCount(-5, p);
    assert(f.birds().empty());
    f.setCount(1000000, p);
    assert(f.birds().size() == Flock::kMaxBirds);
    f.computeAccelerations(p, Predator{}, Search::Grid);  // the accelerations follow the new size
    assert(f.accel().size() == Flock::kMaxBirds);
}

void stepMovesAloneBird() {
    const World w{100.0F, 100.0F};
    const Params p = testParams();
    Flock f(w);
    f.reset(0, 1, p);
    f.birds().push_back(Bird{{10, 10}, {60, 0}});
    f.step(0.5F, p, Predator{}, Search::Grid);
    assert(near(f.birds()[0].pos, {40, 10}));  // 60 px/s * 0.5 s, no neighbors, nothing changes the velocity
    assert(near(f.birds()[0].vel, {60, 0}));
    f.step(0.5F, p, Predator{}, Search::Brute);
    f.step(0.5F, p, Predator{}, Search::Grid);
    assert(near(f.birds()[0].pos, {0, 10}));  // 40 + 30 + 30 = 100 wraps to 0
    f.step(0.5F, p, Predator{}, Search::Grid);
    assert(near(f.birds()[0].pos, {30, 10}));
}

void stepScalesWithDt() {
    // The same acceleration for twice the time changes the velocity twice as much (before the speed clamp).
    const World w{1000.0F, 1000.0F};
    Params p = only(1, 0, 0);
    p.minSpeed = 0.0F;
    Flock f(w);
    f.reset(0, 1, p);
    f.birds().push_back(Bird{{100, 100}, {0, 0}});
    f.birds().push_back(Bird{{104, 100}, {0, 0}});
    f.step(0.01F, p, Predator{}, Search::Grid);
    const float v1 = f.birds()[0].vel.x;  // wanted -100 per second
    assert(near(v1, -1.0F));
    f.reset(0, 1, p);
    f.birds().push_back(Bird{{100, 100}, {0, 0}});
    f.birds().push_back(Bird{{104, 100}, {0, 0}});
    f.step(0.02F, p, Predator{}, Search::Grid);
    assert(near(f.birds()[0].vel.x, 2.0F * v1));
}

void stepKeepsInvariants() {
    const World w{960.0F, 370.0F};
    Params p;  // the defaults the demo ships with
    Flock f(w);
    f.reset(300, 9, p);
    const Predator pr{{480.0F, 185.0F}, true};
    for (int i = 0; i < 300; ++i) f.step(1.0F / 60.0F, p, pr, i % 2 == 0 ? Search::Grid : Search::Brute);
    for (const Bird& b : f.birds()) {
        assert(std::isfinite(b.pos.x) && std::isfinite(b.pos.y) && std::isfinite(b.vel.x) && std::isfinite(b.vel.y));
        assert(b.pos.x >= 0.0F && b.pos.x < w.width);
        assert(b.pos.y >= 0.0F && b.pos.y < w.height);
        const float s = length(b.vel);
        assert(s >= p.minSpeed - 1e-2F && s <= p.maxSpeed + 1e-2F);
    }
}

void scaredFlagFollowsThePredator() {
    const World w{1000.0F, 1000.0F};
    const Params p = testParams();
    Flock f(w);
    f.reset(0, 1, p);
    f.birds().push_back(Bird{{100, 100}, {60, 0}});
    f.birds().push_back(Bird{{500, 500}, {60, 0}});
    f.computeAccelerations(p, Predator{{110, 100}, true}, Search::Grid);
    assert(f.birds()[0].scared && !f.birds()[1].scared);
    f.computeAccelerations(p, Predator{{110, 100}, false}, Search::Grid);  // predator gone: no one is scared any more
    assert(!f.birds()[0].scared && !f.birds()[1].scared);
}

// Three local rules, no leader: after ten simulated seconds the birds mostly fly the same way.
// With only separation the same flock stays as disordered as it started.
void flockingEmerges() {
    const World w{960.0F, 370.0F};
    Params p;
    Flock f(w);
    f.reset(400, 1, p);
    const float before = f.polarization();
    assert(before < 0.2F);  // random headings average out
    for (int i = 0; i < 600; ++i) f.step(1.0F / 60.0F, p, Predator{}, Search::Grid);
    const float after = f.polarization();
    assert(after > 0.5F);

    Params lonely = p;
    lonely.alignment = 0.0F;
    lonely.cohesion = 0.0F;
    Flock g(w);
    g.reset(400, 1, lonely);
    for (int i = 0; i < 600; ++i) g.step(1.0F / 60.0F, lonely, Predator{}, Search::Grid);
    assert(g.polarization() < 0.3F);
    assert(g.polarization() < after);
}

// ---- display helpers ------------------------------------------------------------------------------------------

void formatting() {
    assert(formatCount(0) == "0");
    assert(formatCount(7) == "7");
    assert(formatCount(999) == "999");
    assert(formatCount(1000) == "1,000");
    assert(formatCount(93120) == "93,120");
    assert(formatCount(999000) == "999,000");
    assert(formatCount(1234567) == "1,234,567");
    assert(formatCount(-1234) == "-1,234");
}

void sliders() {
    // track from x = 100, 200 wide, values 0..4
    assert(near(sliderValue(100, 100, 200, 0, 4), 0));
    assert(near(sliderValue(300, 100, 200, 0, 4), 4));
    assert(near(sliderValue(200, 100, 200, 0, 4), 2));
    assert(near(sliderValue(150, 100, 200, 0, 4), 1));
    assert(near(sliderValue(50, 100, 200, 0, 4), 0));    // left of the track clamps
    assert(near(sliderValue(900, 100, 200, 0, 4), 4));   // right of the track clamps
    assert(near(sliderValue(150, 100, 0, 0, 4), 0));     // zero-width track: no division
    assert(near(sliderFraction(2, 0, 4), 0.5F));
    assert(near(sliderFraction(-1, 0, 4), 0.0F));
    assert(near(sliderFraction(9, 0, 4), 1.0F));
    assert(near(sliderFraction(3, 3, 3), 0.0F));         // empty range
}

}  // namespace

int main() {
    limitedAndNormalized();
    wrapping();
    clampingSpeed();
    separationPushesAway();
    separationRadiusBoundary();
    separationWeighsCloseNeighborsMore();
    alignmentMatchesNeighbors();
    cohesionSeeksTheCenter();
    perceptionBoundary();
    limitAppliesPerRule();
    rulesAddUp();
    worldIsATorus();
    coincidentBirdsDoNotProduceNaN();
    fleeFromPredator();
    fleeAcrossTheSeam();
    fleeOnTopOfThePredator();
    fleeForceUsesTheBoost();
    gridCells();
    gridOffersEveryBirdOnceWhenSmall();
    gridFindsEveryNeighbor();
    gridAcrossTheSeam();
    gridAndBruteForceAgree();
    resetBasics();
    changingTheCount();
    stepMovesAloneBird();
    stepScalesWithDt();
    stepKeepsInvariants();
    scaredFlagFollowsThePredator();
    flockingEmerges();
    formatting();
    sliders();
    std::puts("boids-logic: all tests passed");
    return 0;
}
