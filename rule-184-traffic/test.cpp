// Tests for the rule 184 traffic logic. Plain asserts on hand-computed cases, no framework, no SDL.
// `test --survey` only prints the relaxation times and speeds the write-up quotes (not part of ctest).
#undef NDEBUG
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "logic.hpp"

using namespace r184;

namespace {

bool near(double a, double b) { return std::fabs(a - b) < 1e-9; }

Road road(std::size_t n, std::initializer_list<std::size_t> cars) {
    Road r(n, 0);
    for (const std::size_t c : cars) r[c] = 1;
    return r;
}

Road reflectedAndComplemented(const Road& r) {
    Road out(r.size());
    for (std::size_t i = 0; i < r.size(); ++i) out[i] = r[r.size() - 1 - i] != 0 ? 0 : 1;
    return out;
}

bool noTwoAdjacent(const Road& r) {
    for (std::size_t i = 0; i < r.size(); ++i) {
        if (r[i] != 0 && r[(i + 1) % r.size()] != 0) return false;
    }
    return true;
}

bool noTwoGapsAdjacent(const Road& r) {
    for (std::size_t i = 0; i < r.size(); ++i) {
        if (r[i] == 0 && r[(i + 1) % r.size()] == 0) return false;
    }
    return true;
}

void rngMatchesTheReferenceVector() {
    // First output of splitmix64 for seed 0, from the reference implementation.
    Rng rng(0);
    assert(rng.next() == 0xE220A8397B1DCDAFULL);
    Rng a(7);
    Rng b(7);
    for (int i = 0; i < 10; ++i) assert(a.next() == b.next());
    assert(a.below(0) == 0);
    for (int i = 0; i < 1000; ++i) assert(a.below(5) < 5);
}

void theTableIsRule184() {
    // Wolfram code 184 = 10111000: bit (4*left + 2*center + right) of 184 is the new center. A ring of three
    // cells gives cell 1 exactly the neighbors 0 and 2, so one step answers every row of the table.
    for (int idx = 0; idx < 8; ++idx) {
        Road r{static_cast<std::uint8_t>((idx >> 2) & 1), static_cast<std::uint8_t>((idx >> 1) & 1), static_cast<std::uint8_t>(idx & 1)};
        stepParallel(r);
        assert(r[1] == ((184 >> idx) & 1));
    }
}

void aLoneCarDrivesOneCell() {
    Road r = road(8, {3});
    assert(stepParallel(r) == 1);
    assert(r == road(8, {4}));
    r = road(8, {7});
    assert(stepParallel(r) == 1);
    assert(r == road(8, {0}));  // the ring closes
}

void aBlockedCarWaits() {
    Road r = road(4, {0, 1});
    assert(movingCars(r) == 1);  // only the front car has a free cell ahead
    assert(stepParallel(r) == 1);
    assert(r == road(4, {0, 2}));
}

void fullAndEmptyRoadsStandStill() {
    Road full(6, 1);
    assert(stepParallel(full) == 0);
    assert(full == Road(6, 1));
    Road empty(6, 0);
    assert(stepParallel(empty) == 0);
    assert(empty == Road(6, 0));
    Road none;
    assert(stepParallel(none) == 0);
    assert(movingCars(none) == 0);
    Road single{1};  // a one-cell ring: the car is its own neighbor, so it is blocked
    assert(stepParallel(single) == 0);
    assert(single == Road{1});
}

void aQueueDissolvesFromTheFront() {
    // Three cars in a row on a ring of 12: 3 4 5 -> 3 4 6 -> 3 5 7 -> 4 6 8 (worked out by hand).
    Road r = road(12, {3, 4, 5});
    assert(stepParallel(r) == 1);
    assert(r == road(12, {3, 4, 6}));
    assert(stepParallel(r) == 2);
    assert(r == road(12, {3, 5, 7}));
    assert(stepParallel(r) == 3);
    assert(r == road(12, {4, 6, 8}));
}

void theStartWaveRunsBackwards() {
    // Ten cars in a row on an empty ring of 60. After t steps exactly 9 - t cars still wait: the wave "you may go"
    // travels one car (one cell) per step against the driving direction.
    Road r = blockRoad(60, 10, 3);
    for (int t = 0; t <= 9; ++t) {
        assert(carCount(r) - movingCars(r) == 9 - t);
        stepParallel(r);
    }
    // After 9 steps the queue is a train of cars two cells apart (3, 5, ..., 21), all driving. One more step moves
    // every one of them a cell: 4, 6, ..., 22.
    assert(carCount(r) == movingCars(r));
    for (std::size_t i = 0; i < 10; ++i) assert(r[4 + 2 * i] != 0);
}

void carsAreConserved() {
    for (std::uint64_t seed = 1; seed <= 20; ++seed) {
        Rng rng(seed);
        for (const std::size_t cars : {1U, 17U, 90U, 143U, 179U}) {
            Road r = randomRoad(180, cars, rng);
            Road w = r;
            for (int t = 0; t < 300; ++t) {
                stepParallel(r);
                stepInPlace(w);
                assert(carCount(r) == static_cast<int>(cars));
                assert(carCount(w) == static_cast<int>(cars));
            }
        }
    }
}

void theMirrorSymmetryHolds() {
    // Reflect left and right and swap cars and gaps: the same rule comes out (Wikipedia, Rule 184, Dynamics).
    // Worked example on a ring of 6: [1 1 0 0 0 0] steps to [1 0 1 0 0 0]; the transformed start is
    // [1 1 1 1 0 0], which steps to [1 1 1 0 1 0], and that is the transformed result.
    Road a = road(6, {0, 1});
    Road b = reflectedAndComplemented(a);
    assert(b == road(6, {0, 1, 2, 3}));
    stepParallel(a);
    stepParallel(b);
    assert(a == road(6, {0, 2}));
    assert(b == road(6, {0, 1, 2, 4}));
    assert(reflectedAndComplemented(a) == b);
    for (std::uint64_t seed = 1; seed <= 50; ++seed) {
        Rng rng(seed);
        Road r = randomRoad(40, rng.below(41), rng);
        Road m = reflectedAndComplemented(r);
        for (int t = 0; t < 30; ++t) {
            stepParallel(r);
            stepParallel(m);
            assert(m == reflectedAndComplemented(r));
        }
    }
}

void inPlaceSweepMovesACarManyCells() {
    // A lone car at cell 3 of 12: the sweep moves it, then meets it again one cell on, and so on until the
    // end of the array and the wrap to cell 0. Nine hops in one step, against one for the parallel update.
    Road r = road(12, {3});
    assert(stepInPlace(r) == 9);
    assert(r == road(12, {0}));
    Road p = road(12, {3});
    assert(stepParallel(p) == 1);
    // Two cars at 2 and 3: the sweep visits cell 2 first, where the car is blocked by the one at 3. That car then runs
    // 8 hops to cell 11 and one more over the edge to cell 0. The car at 2 never gets another look.
    Road two = road(12, {2, 3});
    assert(stepInPlace(two) == 9);
    assert(two == road(12, {0, 2}));
}

void randomRoadHasTheExactCount() {
    Rng rng(3);
    for (const std::size_t cars : {0U, 1U, 90U, 117U, 180U}) {
        const Road r = randomRoad(180, cars, rng);
        assert(r.size() == 180);
        assert(carCount(r) == static_cast<int>(cars));
    }
    assert(carCount(randomRoad(10, 99, rng)) == 10);  // clamped to the road length
    assert(randomRoad(0, 5, rng).empty());
}

void blockRoadWraps() {
    const Road r = blockRoad(10, 4, 8);
    assert(r == road(10, {8, 9, 0, 1}));
    assert(carCount(blockRoad(10, 99, 3)) == 10);
}

void changingTheCountKeepsWhatItCan() {
    Rng rng(5);
    Road r = randomRoad(180, 117, rng);
    const Road before = r;
    setCarCount(r, 100, rng);
    assert(carCount(r) == 100);
    for (std::size_t i = 0; i < r.size(); ++i) assert(r[i] <= before[i]);  // only removed
    const Road mid = r;
    setCarCount(r, 150, rng);
    assert(carCount(r) == 150);
    for (std::size_t i = 0; i < r.size(); ++i) assert(r[i] >= mid[i]);  // only added
    setCarCount(r, 150, rng);
    assert(r.size() == 180 && carCount(r) == 150);
    setCarCount(r, 999, rng);
    assert(carCount(r) == 180);
    setCarCount(r, 0, rng);
    assert(carCount(r) == 0);
}

void theoryAtTheEdges() {
    assert(near(theorySpeed(0.0), 1.0));
    assert(near(theorySpeed(0.2), 1.0));
    assert(near(theorySpeed(0.5), 1.0));
    assert(near(theorySpeed(0.8), 0.25));
    assert(near(theorySpeed(0.6), 2.0 / 3.0));
    assert(near(theorySpeed(1.0), 0.0));
    assert(near(theorySpeed(1.5), 0.0));
    assert(near(theorySpeed(-0.5), 1.0));
}

void speedWindowAverages() {
    SpeedWindow w(3);
    assert(w.count() == 0 && !w.ready(1));
    assert(near(w.mean(), 0.0));
    w.push(2, 4);
    assert(w.count() == 1 && w.ready(1) && !w.ready(2));
    assert(near(w.mean(), 0.5));
    w.push(4, 4);
    w.push(0, 4);
    assert(near(w.mean(), 6.0 / 12.0));
    w.push(4, 4);  // the first entry (2 of 4) drops out: 4 + 0 + 4 of 12
    assert(w.count() == 3);
    assert(near(w.mean(), 8.0 / 12.0));
    w.clear();
    assert(w.count() == 0);
    w.push(0, 0);  // no cars at all: nothing to average, no division by zero
    assert(near(w.mean(), 0.0));
}

void freeFlowIsExactlyOne() {
    // Below half full every jam is gone after the transient: no two cars touch and every car moves, every step.
    for (std::uint64_t seed = 1; seed <= 30; ++seed) {
        Rng rng(seed);
        Road r = randomRoad(180, 36, rng);
        for (int t = 0; t < 2 * 180; ++t) stepParallel(r);
        assert(noTwoAdjacent(r));
        assert(movingCars(r) == 36);
        assert(near(measureSpeed(180, 36, seed, 2 * 180, 100), 1.0));
    }
}

void congestedFlowIsOneMinusDensityOverDensity() {
    // Above half full the gaps are what moves (one cell left per step) and no two gaps touch. Each gap lets exactly
    // one car go, so (N - K) cars move per step: speed (N - K) / K, which is min(1, (1 - d) / d) for d = K / N.
    for (std::uint64_t seed = 1; seed <= 30; ++seed) {
        Rng rng(seed);
        Road r = randomRoad(180, 144, rng);
        for (int t = 0; t < 2 * 180; ++t) stepParallel(r);
        assert(noTwoGapsAdjacent(r));
        assert(movingCars(r) == 36);
        assert(near(measureSpeed(180, 144, seed, 2 * 180, 100), 0.25));  // the "about a quarter at density 0.8"
    }
    assert(near(measureSpeed(180, 126, 4, 2 * 180, 100), theorySpeed(126.0 / 180.0)));  // 54 / 126
    assert(near(measureSpeed(180, 180, 4, 10, 10), 0.0));  // a full ring never moves
    assert(near(measureSpeed(180, 0, 4, 10, 10), 0.0));    // and an empty one has no cars to average over
}

void halfFullSettlesIntoAlternation() {
    // Exactly 50 % relaxes slowest (Wikipedia: "more slowly"), but it gets there: cars and gaps alternate.
    Rng rng(9);
    Road r = randomRoad(180, 90, rng);
    int steps = 0;
    while (movingCars(r) != 90 && steps < 20 * 180) {
        stepParallel(r);
        ++steps;
    }
    assert(movingCars(r) == 90);
    assert(noTwoAdjacent(r));
}

void inPlaceBreaksTheDiagram() {
    // Cars run up to the original cell of the car ahead in a single sweep, so on a free road the speed is the mean gap
    // (N - K) / K instead of 1: four times too fast at 20 % density. The bug hides from density 0.5 up, where the
    // gaps are already one cell wide and both updates agree.
    Rng rng(2);
    Road r = randomRoad(180, 36, rng);
    long hops = 0;
    for (int t = 0; t < 100; ++t) hops += stepInPlace(r);
    const double speed = static_cast<double>(hops) / (100.0 * 36.0);
    assert(speed > 2.0 * theorySpeed(0.2));
}

void measureIsDeterministic() {
    assert(measureSpeed(180, 100, 11, 360, 100) == measureSpeed(180, 100, 11, 360, 100));
}

void survey() {
    constexpr std::size_t kN = 180;
    std::puts("steps until every car can move (K <= N/2) or no two gaps touch (K > N/2), 200 seeds, N = 180:");
    std::puts("density  cars  min  mean  max  (never within 40*N steps)");
    for (const std::size_t k : {18U, 36U, 54U, 72U, 81U, 90U, 99U, 108U, 126U, 144U, 162U}) {
        long worst = 0;
        long best = 1L << 30;
        double sum = 0;
        int never = 0;
        for (std::uint64_t seed = 1; seed <= 200; ++seed) {
            Rng rng(seed);
            Road r = randomRoad(kN, k, rng);
            long t = 0;
            const bool low = 2 * k <= kN;
            auto settled = [&] { return low ? movingCars(r) == static_cast<int>(k) : noTwoGapsAdjacent(r); };
            while (!settled() && t < 40L * kN) {
                stepParallel(r);
                ++t;
            }
            if (!settled()) {
                ++never;
                continue;
            }
            worst = std::max(worst, t);
            best = std::min(best, t);
            sum += static_cast<double>(t);
        }
        std::printf("%.2f   %4zu  %4ld  %6.1f  %4ld  never: %d\n", static_cast<double>(k) / kN, k, best, sum / (200 - never), worst, never);
    }
    std::puts("\nmeasured speed over 100 steps after 4*N steps of relaxation (seed 1) against the formula:");
    for (std::size_t k = 18; k <= 171; k += 9) {
        std::printf("density %.2f  measured %.4f  formula %.4f\n", static_cast<double>(k) / kN, measureSpeed(kN, k, 1, 4 * kN, 100), theorySpeed(static_cast<double>(k) / kN));
    }
    std::puts("\nin place sweep, mean hops per car per step over 100 steps after 4*N steps (seed 2), against the right update:");
    for (const std::size_t k : {18U, 36U, 72U, 90U, 108U, 144U}) {
        Rng rng(2);
        Road r = randomRoad(kN, k, rng);
        for (std::size_t t = 0; t < 4 * kN; ++t) stepInPlace(r);
        long hops = 0;
        for (int t = 0; t < 100; ++t) hops += stepInPlace(r);
        std::printf("density %.2f  in place %.3f  right %.3f\n", static_cast<double>(k) / kN, static_cast<double>(hops) / (100.0 * static_cast<double>(k)), theorySpeed(static_cast<double>(k) / kN));
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc > 1 && std::strcmp(argv[1], "--survey") == 0) {
        survey();
        return 0;
    }
    rngMatchesTheReferenceVector();
    theTableIsRule184();
    aLoneCarDrivesOneCell();
    aBlockedCarWaits();
    fullAndEmptyRoadsStandStill();
    aQueueDissolvesFromTheFront();
    theStartWaveRunsBackwards();
    carsAreConserved();
    theMirrorSymmetryHolds();
    inPlaceSweepMovesACarManyCells();
    randomRoadHasTheExactCount();
    blockRoadWraps();
    changingTheCountKeepsWhatItCan();
    theoryAtTheEdges();
    speedWindowAverages();
    freeFlowIsExactlyOne();
    congestedFlowIsOneMinusDensityOverDensity();
    halfFullSettlesIntoAlternation();
    inPlaceBreaksTheDiagram();
    measureIsDeterministic();
    std::puts("rule-184-traffic logic: all asserts passed");
    return 0;
}
