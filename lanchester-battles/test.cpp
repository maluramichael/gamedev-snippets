// Plain-assert tests for the Lanchester logic. Every expected value is worked out by hand in the comment above it.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
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

bool near(double a, double b, double eps = 1e-3) { return std::fabs(a - b) <= eps; }

using namespace lb;

// ---- the two laws for one engagement ----

void squareLaw() {
    // Equal units, 100 against 70: A^2 - B^2 = 10000 - 4900 = 5100 survives, so A keeps sqrt(5100) = 71.4143.
    Outcome o = fight(Law::Square, 100, 70, 1, 1);
    CHECK(o.winner == Winner::A);
    CHECK(near(o.left, 71.4143));

    // Twice the army: 10000 - 2500 = 7500, sqrt = 86.6025, so 87 percent of the army is still standing.
    o = fight(Law::Square, 100, 50, 1, 1);
    CHECK(o.winner == Winner::A);
    CHECK(near(o.left, 86.6025));

    // Swapping the sides swaps the winner and keeps the number.
    o = fight(Law::Square, 70, 100, 1, 1);
    CHECK(o.winner == Winner::B);
    CHECK(near(o.left, 71.4143));

    // Exactly equal armies annihilate each other.
    o = fight(Law::Square, 60, 60, 1, 1);
    CHECK(o.winner == Winner::Draw);
    CHECK(o.left == 0.0);

    // Unit strength counts once, army size counts squared: 1 * 100^2 = 10000 = 4 * 50^2, so a tie.
    o = fight(Law::Square, 100, 50, 1, 4);
    CHECK(o.winner == Winner::Draw);
    o = fight(Law::Square, 50, 100, 4, 1);
    CHECK(o.winner == Winner::Draw);

    // Only 2 times as strong: 10000 - 2 * 2500 = 5000, A keeps sqrt(5000) = 70.7107.
    o = fight(Law::Square, 100, 50, 1, 2);
    CHECK(o.winner == Winner::A);
    CHECK(near(o.left, 70.7107));

    // B is 9 times as strong with a third of the army: 90^2 = 8100 = 9 * 30^2, a tie again.
    CHECK(fight(Law::Square, 90, 30, 1, 9).winner == Winner::Draw);

    // Only the ratio of the strengths matters, not their scale: 0.05 for both is the same fight as 1 for both.
    o = fight(Law::Square, 100, 70, 0.05, 0.05);
    CHECK(o.winner == Winner::A && near(o.left, 71.4143));
}

void linearLaw() {
    // Linear law: A - B survives. 100 against 70 leaves 30, against 50 leaves 50 (not 87).
    Outcome o = fight(Law::Linear, 100, 70, 1, 1);
    CHECK(o.winner == Winner::A);
    CHECK(near(o.left, 30.0));
    o = fight(Law::Linear, 100, 50, 1, 1);
    CHECK(o.winner == Winner::A);
    CHECK(near(o.left, 50.0));
    o = fight(Law::Linear, 70, 100, 1, 1);
    CHECK(o.winner == Winner::B);
    CHECK(near(o.left, 30.0));
    CHECK(fight(Law::Linear, 60, 60, 1, 1).winner == Winner::Draw);

    // Strength counts linearly: 1 * 100 = 2 * 50 is the tie (the square law needs 4 for the same armies).
    CHECK(fight(Law::Linear, 100, 50, 1, 2).winner == Winner::Draw);

    // With 4 times the strength B wins: 4 * 50 - 100 = 100 in B-units of 4 each, so 25 B units are left.
    o = fight(Law::Linear, 100, 50, 1, 4);
    CHECK(o.winner == Winner::B);
    CHECK(near(o.left, 25.0));
}

void degenerate() {
    // Nobody on one side: the other side keeps everything, in both laws.
    for (Law law : {Law::Square, Law::Linear}) {
        Outcome o = fight(law, 0, 40, 1, 1);
        CHECK(o.winner == Winner::B && near(o.left, 40.0));
        o = fight(law, 25, 0, 1, 1);
        CHECK(o.winner == Winner::A && near(o.left, 25.0));
        CHECK(fight(law, 0, 0, 1, 1).winner == Winner::Draw);
        // A negative size is nonsense and counts as empty.
        o = fight(law, -5, 10, 1, 1);
        CHECK(o.winner == Winner::B && near(o.left, 10.0));
        // Nobody can hurt anybody: no result, and in particular no division by zero.
        o = fight(law, 10, 10, 0, 0);
        CHECK(o.winner == Winner::Draw && o.left == 0.0);
        // One side harmless: the other wins with all it has.
        o = fight(law, 10, 10, 1, 0);
        CHECK(o.winner == Winner::A && near(o.left, 10.0));
    }
}

// ---- reserves, the "divide and lose" case from Game AI Pro 3 chapter 25 ----

void splitArmy() {
    // Red 500 + 500 attack one after the other, Blue has 1000. First fight: Blue keeps sqrt(1000^2 - 500^2)
    // = sqrt(750000) = 866.025. Second fight: sqrt(866.025^2 - 500^2) = sqrt(500000) = 707.107. Blue wins and has
    // lost only 29 percent (the book rounds to 30).
    Outcome o = predict(Law::Square, {500, 500, 1000, 1, 1});
    CHECK(o.winner == Winner::B);
    CHECK(near(o.left, 707.107));

    // Linear law: 500 against 1000 leaves 500, the second 500 then tie with them. Splitting costs nothing.
    o = predict(Law::Linear, {500, 500, 1000, 1, 1});
    CHECK(o.winner == Winner::Draw);

    // No reserve is the plain fight.
    o = predict(Law::Square, {100, 0, 70, 1, 1});
    CHECK(o.winner == Winner::A && near(o.left, 71.4143));

    // First group wins on its own: 71.4143 are left, plus the 50 who never had to fight.
    o = predict(Law::Square, {100, 50, 70, 1, 1});
    CHECK(o.winner == Winner::A && near(o.left, 121.4143));

    // First group and enemy wipe each other out, the reserve takes the field.
    o = predict(Law::Square, {50, 30, 50, 1, 1});
    CHECK(o.winner == Winner::A && near(o.left, 30.0));

    // Reserve too small to matter: 10 against 100 leaves sqrt(9900) = 99.4987, then 10 against that leaves
    // sqrt(9900 - 100) = sqrt(9800) = 98.9949.
    o = predict(Law::Square, {10, 10, 100, 1, 1});
    CHECK(o.winner == Winner::B && near(o.left, 98.9949));
}

// ---- the curves over time ----

const Sample* at(const std::vector<Sample>& c, double t) {
    for (const Sample& s : c) {
        if (near(s.t, t, 1e-9)) return &s;
    }
    return nullptr;
}

void squareCurve() {
    const std::vector<Sample> c = curve(Law::Square, {100, 0, 70, 1, 1}, 0.05);
    CHECK(c.size() > 10);
    CHECK(c.front().t == 0.0 && c.front().a == 100.0 && c.front().b == 70.0);

    // Closed form with k = sqrt(alpha * beta) = 1: A(t) = 100 cosh t - 70 sinh t, B(t) = 70 cosh t - 100 sinh t.
    // At t = 0.5 (cosh 0.5 = 1.127626, sinh 0.5 = 0.521095): A = 76.2859, B = 26.8243.
    const Sample* s = at(c, 0.5);
    CHECK(s != nullptr);
    if (s != nullptr) {
        CHECK(near(s->a, 76.2859, 5e-3));
        CHECK(near(s->b, 26.8243, 5e-3));
    }

    // A^2 - B^2 stays 5100 the whole way, that is the law.
    for (std::size_t i = 0; i + 1 < c.size(); ++i) CHECK(near(c[i].a * c[i].a - c[i].b * c[i].b, 5100.0, 0.1));

    // B is gone when tanh t = 70/100, so t = atanh(0.7) = 0.8673 s. The curve stops once B is down to half a unit.
    CHECK(near(c.back().t, 0.8673, 0.02));
    CHECK(c.back().b <= 0.5);
    CHECK(near(c.back().a, 71.4143, 0.05));
}

void linearCurve() {
    // With alpha = beta = 1 and A > B the loser decays like B(t) = 70 e^-t and A - B stays 30.
    // At t = 1: B = 70 / e = 25.7515, A = 55.7515.
    const std::vector<Sample> c = curve(Law::Linear, {100, 0, 70, 1, 1}, 0.05);
    const Sample* s = at(c, 1.0);
    CHECK(s != nullptr);
    if (s != nullptr) {
        CHECK(near(s->b, 25.7515, 5e-3));
        CHECK(near(s->a, 55.7515, 5e-3));
    }
    for (const Sample& p : c) CHECK(near(p.a - p.b, 30.0, 1e-3));
    CHECK(c.back().b <= 0.5);
}

void splitCurve() {
    const std::vector<Sample> c = curve(Law::Square, {50, 50, 100, 1, 1}, 0.01);
    // The total of Red starts at 100 (50 fighting, 50 waiting) and only ever goes down, Blue too.
    CHECK(c.front().a == 100.0 && c.front().b == 100.0);
    for (std::size_t i = 1; i < c.size(); ++i) {
        CHECK(c[i].a <= c[i - 1].a + 1e-9);
        CHECK(c[i].b <= c[i - 1].b + 1e-9);
        CHECK(c[i].t > c[i - 1].t);
    }
    // Red is wiped out. First fight sqrt(100^2 - 50^2) = 86.6025 left, second sqrt(86.6025^2 - 50^2) = sqrt(5000) = 70.7107.
    CHECK(c.back().a <= 0.5);
    CHECK(near(c.back().b, 70.7107, 0.05));
}

void curveEdges() {
    // Nobody can kill anybody: one sample, no endless loop.
    CHECK(curve(Law::Square, {10, 0, 10, 0, 0}).size() == 1);
    // A battle that is already over has one sample as well.
    CHECK(curve(Law::Square, {0, 0, 10, 1, 1}).size() == 1);
}

// ---- the agent simulation ----
//
// The simulation knows weapons, range, targets and walking, nothing about the laws. These tests run whole battles with
// fixed seeds and compare the averages with the formulas above. Measured on the first 40 seeds (step 1/60, then
// rechecked with 1/30): ranged 100 v 70 keeps 70.1 +- 8.7 per battle (law 71.4), duels 29.6 +- 12.7 (law 30).
// The bounds below are about three standard errors of a 16 battle average, so they hold for any platform's float math.

struct Batch {
    double left = 0.0;      // mean size of the winner
    double time = 0.0;      // mean battle length in seconds
    double engagedA = 0.0;  // mean share of time units of A had an enemy to hit
    double wasted = 0.0;    // mean share of shots that arrived at a corpse
    double aWins = 0.0;     // share of battles won by A
    double a8 = 0.0;        // mean alive A after 8 s
    double b8 = 0.0;
    bool allFinished = true;
};

Batch runBatch(Params p, int runs) {
    Batch out;
    for (int seed = 1; seed <= runs; ++seed) {
        p.seed = static_cast<std::uint64_t>(seed);
        Battle battle(p);
        int guard = 0;
        while (!battle.finished() && guard++ < 100000) battle.step(1.0 / 30.0);
        out.allFinished = out.allFinished && battle.finished() && !battle.timedOut();
        const Outcome o = battle.outcome();
        out.left += o.left;
        out.time += battle.time();
        out.engagedA += battle.engaged(0);
        out.wasted += battle.wasted();
        out.aWins += o.winner == Winner::A ? 1.0 : 0.0;
        const auto& h = battle.history();
        const Sample& at8 = h[std::min<std::size_t>(80, h.size() - 1)];
        out.a8 += at8.a;
        out.b8 += at8.b;
    }
    const double n = runs;
    out.left /= n;
    out.time /= n;
    out.engagedA /= n;
    out.wasted /= n;
    out.aWins /= n;
    out.a8 /= n;
    out.b8 /= n;
    return out;
}

Params ranged(int a, int b) {
    Params p;
    p.law = Law::Square;
    p.a1 = a;
    p.b = b;
    return p;
}

Params duels(int a, int b) {
    Params p = ranged(a, b);
    p.law = Law::Linear;
    return p;
}

void rngBasics() {
    Rng a(42);
    Rng b(42);
    Rng c(43);
    bool same = true;
    bool differs = false;
    bool inRange = true;
    double sum = 0.0;
    for (int i = 0; i < 2000; ++i) {
        const double x = a.uniform();
        same = same && x == b.uniform();
        differs = differs || x != c.uniform();
        inRange = inRange && x >= 0.0 && x < 1.0;
        sum += x;
    }
    CHECK(same);
    CHECK(differs);
    CHECK(inRange);
    CHECK(near(sum / 2000.0, 0.5, 0.03));  // 2000 uniform draws: standard error 0.0065
}

void formation() {
    // 100 units: 8 columns of at most 13 (7 * 13 = 91, plus 9 in the last one). Front column on kFrontA, spacing 15.
    const std::vector<Vec2> a = block(100, kFrontA, -1);
    CHECK(a.size() == 100);
    CHECK(columns(100) == 8);
    CHECK(near(a.front().x, kFrontA, 1e-6));
    CHECK(near(a.back().x, kFrontA - 7.0 * kSpacing, 1e-6));  // 190 - 105 = 85
    double minDist = 1e9;
    bool inside = true;
    for (std::size_t i = 0; i < a.size(); ++i) {
        inside = inside && a[i].x > 4.0F && a[i].x < kFieldW - 4.0F && a[i].y > 4.0F && a[i].y < kFieldH - 4.0F;
        for (std::size_t j = i + 1; j < a.size(); ++j) {
            minDist = std::min(minDist, std::hypot(static_cast<double>(a[i].x - a[j].x), static_cast<double>(a[i].y - a[j].y)));
        }
    }
    CHECK(inside);
    CHECK(minDist >= kSpacing - 1e-3);
    // The right side mirrors it.
    const std::vector<Vec2> b = block(100, kFrontB, +1);
    CHECK(near(b.back().x, kFrontB + 7.0 * kSpacing, 1e-6));
    // One unit stands in the middle of the field's height, nothing and less than nothing is an empty block.
    const std::vector<Vec2> one = block(1, kFrontB, +1);
    CHECK(one.size() == 1 && near(one[0].y, kFieldH / 2.0));
    CHECK(block(0, kFrontB, +1).empty());
    CHECK(block(-3, kFrontB, +1).empty());
    CHECK(columns(0) == 0 && columns(14) == 1 && columns(15) == 2);
}

void setupNumbers() {
    // Ranged: 8 percent of the shots kill, one shot per 1.6 s: 0.05 kills per second. Melee: 0.2 / 0.8 = 0.25.
    CHECK(near(alphaOf(Law::Square), 0.05, 1e-12));
    CHECK(near(alphaOf(Law::Linear), 0.25, 1e-12));
    Params p = ranged(100, 50);
    p.power = 4.0;
    const Setup s = setupOf(p);
    CHECK(s.a1 == 100.0 && s.a2 == 0.0 && s.b == 50.0);
    CHECK(near(s.beta, 4.0 * s.alpha, 1e-12));
    // The same armies are a tie under the square law with power 4 and under the linear law with power 2.
    CHECK(predict(Law::Square, s).winner == Winner::Draw);
    Params q = duels(100, 50);
    q.power = 2.0;
    CHECK(predict(Law::Linear, setupOf(q)).winner == Winner::Draw);
}

void battleBasics() {
    // Start: everybody alive, the reserve counted but not yet in the fight.
    Params p = ranged(50, 100);
    p.a2 = 50;
    Battle b(p);
    CHECK(b.alive(0) == 100 && b.alive(1) == 100);
    CHECK(!b.finished() && !b.reserveReleased() && b.time() == 0.0);
    int idle = 0;
    for (const Unit& u : b.units()) idle += u.active ? 0 : 1;
    CHECK(idle == 50);

    // A zero length step changes nothing.
    b.step(0.0);
    CHECK(b.time() == 0.0 && b.shots().empty());

    // Nobody ever comes back to life, and the clock only moves forward.
    int lastA = b.alive(0);
    int lastB = b.alive(1);
    bool monotone = true;
    for (int i = 0; i < 4000 && !b.finished(); ++i) {
        b.step(1.0 / 30.0);
        monotone = monotone && b.alive(0) <= lastA && b.alive(1) <= lastB;
        lastA = b.alive(0);
        lastB = b.alive(1);
    }
    CHECK(monotone);
    CHECK(b.finished());
    CHECK(b.reserveReleased());  // the first group of 50 lost against 100, so the reserve had to come
    CHECK(b.outcome().winner == Winner::B);

    // An army against nobody: over before it starts, A keeps everything.
    Battle empty(ranged(30, 0));
    CHECK(empty.finished());
    CHECK(empty.outcome().winner == Winner::A && empty.outcome().left == 30.0);
    Battle nobody(ranged(0, 0));
    CHECK(nobody.finished() && nobody.outcome().winner == Winner::Draw);

    // Same seed, same battle, bit for bit.
    Params s = ranged(100, 70);
    s.seed = 5;
    Battle x(s);
    Battle y(s);
    while (!x.finished()) x.step(1.0 / 30.0);
    while (!y.finished()) y.step(1.0 / 30.0);
    CHECK(x.time() == y.time() && x.outcome().left == y.outcome().left && x.shotsFired() == y.shotsFired());
    // Other seeds give other battles.
    s.seed = 6;
    Battle z(s);
    while (!z.finished()) z.step(1.0 / 30.0);
    CHECK(z.shotsFired() != x.shotsFired() || z.outcome().left != x.outcome().left);
}

void squareInTheSimulation() {
    // 100 against 70, everybody in range of everybody, shots spread over the enemies.
    // Law: sqrt(100^2 - 70^2) = 71.4 left, battle over after atanh(0.7) / 0.05 = 17.3 s.
    const Batch r = runBatch(ranged(100, 70), 16);
    CHECK(r.allFinished);
    CHECK(r.aWins == 1.0);
    CHECK(r.left > 64.0 && r.left < 79.0);
    CHECK(r.time > 15.0 && r.time < 23.0);
    CHECK(r.engagedA > 0.95);
    CHECK(r.wasted < 0.06);

    // The conserved quantity along the way: after 8 s the law has A = 79.4 and B = 34.6 (RK4 on the curve).
    const std::vector<Sample> c = curve(Law::Square, setupOf(ranged(100, 70)), 0.05);
    const Sample* s = at(c, 8.0);
    CHECK(s != nullptr);
    if (s != nullptr) {
        CHECK(near(r.a8, s->a, 5.0));
        CHECK(near(r.b8, s->b, 6.0));
    }
    // 2:1: 87 percent of the winner's army survive (86.6 in the law).
    const Batch two = runBatch(ranged(100, 50), 16);
    CHECK(two.left > 78.0 && two.left < 92.0);
}

void linearInTheSimulation() {
    // Duels: A - B = 30 survive, not 71. Walking and waiting for the next opponent changes the speed, not the count.
    const Batch r = runBatch(duels(100, 70), 16);
    CHECK(r.allFinished);
    CHECK(r.left > 20.0 && r.left < 40.0);
    // Against 50 it is 50 left (the square law would say 87).
    const Batch two = runBatch(duels(100, 50), 16);
    CHECK(two.left > 40.0 && two.left < 60.0);
    // Same armies, so the square law keeps clearly more of them.
    CHECK(runBatch(ranged(100, 70), 8).left > 1.8 * r.left);
    // Only the duels that fit side by side run: 30 units of A wait, so their time at an opponent is far below B's.
    CHECK(r.engagedA < 0.5);
}

void divideAndLose() {
    // Square law: 50 + 50 attack one after the other against 100. The law says B keeps 70.7.
    Params p = ranged(50, 100);
    p.a2 = 50;
    const Batch sq = runBatch(p, 10);
    CHECK(sq.aWins == 0.0);
    CHECK(sq.left > 55.0 && sq.left < 80.0);
    // Linear law: the same split is a tie, B keeps (almost) nothing.
    Params q = duels(50, 100);
    q.a2 = 50;
    const Batch lin = runBatch(q, 10);
    CHECK(lin.left < 30.0);
    CHECK(lin.aWins > 0.15 && lin.aWins < 0.85);
}

void balancing() {
    // 100 against 50 where a B unit is 4 times as strong: a tie in the square law (the law's draw has a spread in
    // the simulation, the winner changes from battle to battle), and nowhere near the 87 left without the bonus.
    Params p = ranged(100, 50);
    p.power = 4.0;
    const Batch sq = runBatch(p, 20);
    CHECK(sq.aWins > 0.15 && sq.aWins < 0.85);
    CHECK(sq.left < 45.0);
    // Duels: a bonus of 2 is the tie, with 4 B wins and keeps 25 of its 50.
    Params q = duels(100, 50);
    q.power = 2.0;
    const Batch lin = runBatch(q, 20);
    CHECK(lin.aWins > 0.15 && lin.aWins < 0.85);
    CHECK(lin.left < 25.0);
    q.power = 4.0;
    const Batch four = runBatch(q, 10);
    CHECK(four.aWins == 0.0);
    CHECK(four.left > 17.0 && four.left < 33.0);
}

void whereTheSimulationDeviates() {
    const Batch good = runBatch(ranged(100, 70), 12);

    // Everybody shoots the nearest enemy: shots pile up on the front line and many arrive at a corpse.
    Params n = ranged(100, 70);
    n.aim = Aim::Nearest;
    const Batch nearest = runBatch(n, 12);
    CHECK(nearest.wasted > 0.10);
    CHECK(nearest.time > 1.15 * good.time);
    CHECK(nearest.left < good.left + 3.0);

    // Short range: the back of the bigger block cannot shoot, so numbers stop counting squared.
    Params s = ranged(100, 70);
    s.range = kRangeShort;
    const Batch shortRange = runBatch(s, 12);
    CHECK(good.engagedA > 0.95);
    CHECK(shortRange.engagedA < 0.7);
    CHECK(shortRange.left < 0.85 * good.left);
    CHECK(shortRange.time > 1.5 * good.time);
}

// `lanchester-battles-logic --measure` prints the table the post quotes: 40 battles per line, seeds 1 to 40, the same
// 1/60 s step as the demo. Not part of ctest (it takes a while), run it by hand when the simulation changes.
void measure() {
    struct Row {
        const char* name;
        Params p;
    };
    std::vector<Row> rows;
    rows.push_back({"ranged 100 v 70, coordinated", ranged(100, 70)});
    Params n = ranged(100, 70);
    n.aim = Aim::Nearest;
    rows.push_back({"ranged 100 v 70, nearest", n});
    n.aim = Aim::Random;
    rows.push_back({"ranged 100 v 70, random", n});
    Params s = ranged(100, 70);
    s.range = kRangeShort;
    rows.push_back({"ranged 100 v 70, short range", s});
    rows.push_back({"ranged 100 v 50", ranged(100, 50)});
    Params tie = ranged(100, 50);
    tie.power = 4.0;
    rows.push_back({"ranged 100 v 50, B x4", tie});
    Params split = ranged(50, 100);
    split.a2 = 50;
    rows.push_back({"ranged 50+50 v 100", split});
    rows.push_back({"duels 100 v 70", duels(100, 70)});
    rows.push_back({"duels 100 v 50", duels(100, 50)});
    Params d2 = duels(100, 50);
    d2.power = 2.0;
    rows.push_back({"duels 100 v 50, B x2", d2});
    Params d4 = duels(100, 50);
    d4.power = 4.0;
    rows.push_back({"duels 100 v 50, B x4", d4});
    Params dsplit = duels(50, 100);
    dsplit.a2 = 50;
    rows.push_back({"duels 50+50 v 100", dsplit});
    std::printf("%-30s %-14s %5s %5s %7s %7s %7s %6s %6s\n", "setup", "law", "left", "sd", "A:B", "time", "waste", "engA", "engB");
    for (const Row& row : rows) {
        double sum = 0.0;
        double sum2 = 0.0;
        double time = 0.0;
        double waste = 0.0;
        double engA = 0.0;
        double engB = 0.0;
        int aWins = 0;
        constexpr int kRuns = 40;
        for (int seed = 1; seed <= kRuns; ++seed) {
            Params p = row.p;
            p.seed = static_cast<std::uint64_t>(seed);
            Battle b(p);
            while (!b.finished()) b.step(1.0 / 60.0);
            const Outcome o = b.outcome();
            sum += o.left;
            sum2 += o.left * o.left;
            time += b.time();
            waste += b.wasted();
            engA += b.engaged(0);
            engB += b.engaged(1);
            aWins += o.winner == Winner::A ? 1 : 0;
        }
        const double mean = sum / kRuns;
        const Outcome law = predict(row.p.law, setupOf(row.p));
        char lawText[32];
        std::snprintf(lawText, sizeof lawText, "%s %.1f", law.winner == Winner::A ? "A" : law.winner == Winner::B ? "B" : "tie", law.left);
        char wins[32];
        std::snprintf(wins, sizeof wins, "%d:%d", aWins, kRuns - aWins);
        std::printf("%-30s %-14s %5.1f %5.1f %7s %7.1f %6.0f%% %5.0f%% %5.0f%%\n", row.name, lawText, mean, std::sqrt(std::max(0.0, sum2 / kRuns - mean * mean)), wins,
                    time / kRuns, waste / kRuns * 100.0, engA / kRuns * 100.0, engB / kRuns * 100.0);
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc > 1 && std::strcmp(argv[1], "--measure") == 0) {
        measure();
        return 0;
    }
    squareLaw();
    linearLaw();
    degenerate();
    splitArmy();
    squareCurve();
    linearCurve();
    splitCurve();
    curveEdges();
    rngBasics();
    formation();
    setupNumbers();
    battleBasics();
    squareInTheSimulation();
    linearInTheSimulation();
    divideAndLose();
    balancing();
    whereTheSimulationDeviates();
    if (failures == 0) {
        std::printf("all lanchester-battles checks passed\n");
        return 0;
    }
    std::printf("%d check(s) failed\n", failures);
    return 1;
}
