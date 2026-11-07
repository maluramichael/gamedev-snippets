// Tests for the job-assignment logic. Plain asserts on hand-computed cases, no framework, no SDL.
#undef NDEBUG
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

#include "logic.hpp"

using namespace ja;

namespace {

bool near(double a, double b) { return std::fabs(a - b) < 1e-9; }

bool isPerm(const Assignment& a, std::size_t n) {
    if (a.size() != n) return false;
    std::vector<int> sorted = a;
    std::sort(sorted.begin(), sorted.end());
    for (std::size_t i = 0; i < n; ++i) {
        if (sorted[i] != static_cast<int>(i)) return false;
    }
    return true;
}

// Worker A at 0, worker B at 4, job 0 at 2.5, job 1 at -3, all on one line.
// A is 2.5 from job 0 and 3 from job 1. B is 1.5 from job 0 and 7 from job 1.
Layout lineExample() { return Layout{{{0, 0}, {4, 0}}, {{2.5, 0}, {-3, 0}}}; }

const Field kField{24.0, 48.0, 936.0, 358.0};

void distanceBasics() {
    assert(near(distance({0, 0}, {3, 4}), 5.0));
    assert(near(distance({1, 1}, {1, 1}), 0.0));
    assert(near(distance({-2, 0}, {2, 0}), 4.0));
}

void costMatrixShape() {
    const Costs c = costMatrix(lineExample().workers, lineExample().jobs);
    assert(c.size() == 2 && c[0].size() == 2);
    assert(near(c[0][0], 2.5));
    assert(near(c[0][1], 3.0));
    assert(near(c[1][0], 1.5));
    assert(near(c[1][1], 7.0));
}

void greedyDependsOnOrder() {
    const Layout l = lineExample();
    const Costs c = costMatrix(l.workers, l.jobs);
    // A goes first: A takes job 0 (2.5), B is left with job 1 (7). Total 9.5.
    const Assignment first = greedy(c, {0, 1});
    assert((first == Assignment{0, 1}));
    assert(near(totalCost(c, first), 9.5));
    // B goes first: B takes job 0 (1.5), A is left with job 1 (3). Total 4.5.
    const Assignment second = greedy(c, {1, 0});
    assert((second == Assignment{1, 0}));
    assert(near(totalCost(c, second), 4.5));
}

void greedyRejectsBadInput() {
    const Costs c = costMatrix(lineExample().workers, lineExample().jobs);
    assert(greedy(c, {0}).empty());         // too short
    assert(greedy(c, {0, 0}).empty());      // not a permutation
    assert(greedy(c, {0, 2}).empty());      // out of range
    assert(greedy(c, {-1, 0}).empty());     // negative
    assert(greedy({}, {}).empty());         // nothing to do
    assert(greedy(Costs{{1, 2}}, {0}).empty());  // not square
}

void greedyTiesTakeTheLowerJob() {
    const Costs c{{1, 1}, {1, 1}};
    assert((greedy(c, {0, 1}) == Assignment{0, 1}));
    assert((greedy(c, {1, 0}) == Assignment{1, 0}));  // worker 1 goes first and takes job 0
}

void processingOrderIsAPermutation() {
    assert((processingOrder(4, false) == std::vector<int>{0, 1, 2, 3}));
    assert((processingOrder(4, true) == std::vector<int>{3, 2, 1, 0}));
    assert(processingOrder(0, true).empty());
    assert((processingOrder(1, true) == std::vector<int>{0}));
}

void hungarianHandMatrix() {
    // Rows are workers, columns jobs. The six permutations cost 5 (1+2+2), 6, 6, 7, 9 and 11, so 5 is the minimum.
    const Costs c{{4, 1, 3}, {2, 0, 5}, {3, 2, 2}};
    const Assignment a = hungarian(c);
    assert((a == Assignment{1, 0, 2}));
    assert(near(totalCost(c, a), 5.0));
    // Greedy in order 0,1,2 happens to find the same 5, in order 1,0,2 it ends at 6 (worker 1 takes the 0, worker 0 is left with 3, worker 2 with 3).
    assert(near(totalCost(c, greedy(c, {0, 1, 2})), 5.0));
    assert(near(totalCost(c, greedy(c, {1, 0, 2})), 6.0));
}

void hungarianLineExample() {
    const Layout l = lineExample();
    const Costs c = costMatrix(l.workers, l.jobs);
    const Assignment a = hungarian(c);
    assert((a == Assignment{1, 0}));
    assert(near(totalCost(c, a), 4.5));
}

void hungarianSmallAndOddInputs() {
    assert((hungarian(Costs{{7}}) == Assignment{0}));
    assert(hungarian({}).empty());
    assert(hungarian(Costs{{1, 2}}).empty());  // not square
    assert(hungarian(Costs{{1, std::numeric_limits<double>::quiet_NaN()}, {2, 3}}).empty());
    assert(hungarian(Costs{{1, std::numeric_limits<double>::infinity()}, {2, 3}}).empty());
    // Negative costs are fine: the diagonal is -2 in total, the other way round 0.
    assert((hungarian(Costs{{-1, 0}, {0, -1}}) == Assignment{0, 1}));
    // All equal: any permutation is optimal, it must still be a valid one.
    const Assignment flat = hungarian(Costs(5, std::vector<double>(5, 0.0)));
    assert(isPerm(flat, 5));
    // Worker 0 is cheap everywhere, worker 1 only at job 0: the unique optimum is 0->1, 1->0 (1 + 2 = 3, the other way costs 101).
    const Costs c{{1, 1}, {2, 100}};
    assert((hungarian(c) == Assignment{1, 0}));
}

void totalCostGuards() {
    const Costs c{{1, 2}, {3, 4}};
    assert(near(totalCost(c, {1, 0}), 5.0));
    assert(std::isinf(totalCost(c, {0})));     // wrong length
    assert(std::isinf(totalCost(c, {0, 2})));  // out of range
    assert(std::isinf(totalCost(c, {-1, 0})));
    assert(near(totalCost({}, {}), 0.0));
}

void bruteForceMatchesByHand() {
    const Costs c{{4, 1, 3}, {2, 0, 5}, {3, 2, 2}};
    assert(near(totalCost(c, bruteForce(c)), 5.0));
    assert(bruteForce({}).empty());
    assert(bruteForce(Costs(static_cast<std::size_t>(kBruteForceMax) + 1, std::vector<double>(static_cast<std::size_t>(kBruteForceMax) + 1, 1.0))).empty());
}

void hungarianMatchesBruteForceOnRandomMatrices() {
    Lcg rng(12345);
    for (int n = 1; n <= kBruteForceMax; ++n) {
        for (int rep = 0; rep < 25; ++rep) {
            Costs c(static_cast<std::size_t>(n), std::vector<double>(static_cast<std::size_t>(n)));
            for (auto& row : c) {
                for (double& v : row) v = static_cast<double>(rng.below(21)) - 5.0;  // integers -5..15, ties on purpose
            }
            const Assignment h = hungarian(c);
            const Assignment b = bruteForce(c);
            assert(isPerm(h, static_cast<std::size_t>(n)));
            assert(near(totalCost(c, h), totalCost(c, b)));
        }
    }
}

void hungarianMatchesBruteForceOnLayouts() {
    for (int n = 2; n <= kBruteForceMax; ++n) {
        for (std::uint32_t seed = 1; seed <= 40; ++seed) {
            const Layout l = randomLayout(seed * 7 + static_cast<std::uint32_t>(n), n, kField, 30.0);
            const Costs c = costMatrix(l.workers, l.jobs);
            assert(near(totalCost(c, hungarian(c)), totalCost(c, bruteForce(c))));
        }
    }
}

void segmentCrossing() {
    // The two diagonals of a square cross in its middle.
    assert(segmentsCross({0, 0}, {2, 2}, {2, 0}, {0, 2}));
    // Parallel and apart.
    assert(!segmentsCross({0, 0}, {2, 0}, {0, 1}, {2, 1}));
    // The line through one segment would hit the other, but the segment stops short.
    assert(!segmentsCross({0, 0}, {1, 1}, {2, 0}, {3, -5}));
    // The lines y = x and x + y = 3 meet at (1.5, 1.5), which is past the end of the first segment at (1, 1).
    assert(!segmentsCross({0, 0}, {1, 1}, {3, 0}, {0, 3}));
}

void crossingPointsOfAnAssignment() {
    const std::vector<Vec2> workers{{0, 0}, {2, 0}};
    const std::vector<Vec2> jobs{{0, 2}, {2, 2}};
    // Straight up on both sides: no crossing.
    assert(crossings(workers, jobs, {0, 1}).empty());
    // Swapped: the two segments form an X with its center at (1, 1).
    const auto x = crossings(workers, jobs, {1, 0});
    assert(x.size() == 1);
    assert(near(x[0].x, 1.0) && near(x[0].y, 1.0));
    // Broken input counts as no crossings, not as a crash.
    assert(crossings(workers, jobs, {0}).empty());
    assert(crossings(workers, jobs, {0, 5}).empty());
}

void crossingsOfThreeSegmentsCountPairs() {
    // Three workers on the left (y = 0, 10, 25), three jobs on the right at the same heights.
    const std::vector<Vec2> workers{{0, 0}, {0, 10}, {0, 25}};
    const std::vector<Vec2> jobs{{10, 0}, {10, 10}, {10, 25}};
    assert(crossings(workers, jobs, {0, 1, 2}).empty());
    assert(crossings(workers, jobs, {1, 0, 2}).size() == 1);  // only the first two swap
    // 0 and 2 swap and cross at (5, 12.5). The flat middle segment at y = 10 is met by both of them, at x = 4 and x = 6.
    assert(crossings(workers, jobs, {2, 1, 0}).size() == 3);
}

void collinearOverlapIsNotACrossing() {
    // Documented limit: segments on the same line are not "proper" crossings. Greedy can produce them, the optimum never needs to.
    assert(!segmentsCross({0, 0}, {2, 0}, {1, 0}, {3, 0}));
}

void optimalNeverCrosses() {
    int checked = 0;
    for (int n = 2; n <= kBruteForceMax; ++n) {
        for (std::uint32_t seed = 1; seed <= 100; ++seed) {
            const Layout l = randomLayout(seed * 31 + static_cast<std::uint32_t>(n), n, kField, 30.0);
            const Costs c = costMatrix(l.workers, l.jobs);
            assert(crossings(l.workers, l.jobs, hungarian(c)).empty());
            ++checked;
        }
    }
    assert(checked == 700);
}

void greedyOrderRangeByHand() {
    const Layout l = lineExample();
    const OrderRange r = greedyOrderRange(costMatrix(l.workers, l.jobs));
    assert(r.orders == 2);
    assert(near(r.best, 4.5));
    assert(near(r.worst, 9.5));
    const Costs three{{4, 1, 3}, {2, 0, 5}, {3, 2, 2}};
    const OrderRange t = greedyOrderRange(three);
    assert(t.orders == 6);
    assert(near(t.best, 5.0));
    // All six orders, worked out by hand (worker 2 sees jobs 1 and 2 at the same cost and takes the lower index):
    // 0,1,2 -> 1 + 2 + 2 = 5     0,2,1 -> 1 + 2 + 2 = 5
    // 1,0,2 -> 0 + 3 + 3 = 6     1,2,0 -> 0 + 2 + 4 = 6
    // 2,0,1 -> 2 + 3 + 2 = 7     2,1,0 -> 2 + 2 + 3 = 7
    assert(near(t.worst, 7.0));
    assert(greedyOrderRange({}).orders == 0);
    assert(greedyOrderRange(Costs(static_cast<std::size_t>(kBruteForceMax) + 1, std::vector<double>(static_cast<std::size_t>(kBruteForceMax) + 1, 1.0))).orders == 0);
}

void greedyOrderRangeBoundsEveryOrder() {
    const Layout l = randomLayout(99, 6, kField, 30.0);
    const Costs c = costMatrix(l.workers, l.jobs);
    const OrderRange r = greedyOrderRange(c);
    assert(r.orders == 720);
    std::vector<int> order = processingOrder(6, false);
    do {
        const double t = totalCost(c, greedy(c, order));
        assert(t >= r.best - 1e-9 && t <= r.worst + 1e-9);
    } while (std::next_permutation(order.begin(), order.end()));
    // No order beats the optimum.
    assert(r.best >= totalCost(c, hungarian(c)) - 1e-9);
}

void layoutsAreDeterministicAndSpread() {
    const Layout a = randomLayout(5, 8, kField, 40.0);
    const Layout b = randomLayout(5, 8, kField, 40.0);
    const Layout c = randomLayout(6, 8, kField, 40.0);
    assert(a.workers.size() == 8 && a.jobs.size() == 8);
    bool same = true;
    bool other = false;
    for (std::size_t i = 0; i < 8; ++i) {
        same = same && a.workers[i].x == b.workers[i].x && a.jobs[i].y == b.jobs[i].y;
        other = other || a.workers[i].x != c.workers[i].x;
    }
    assert(same);
    assert(other);
    std::vector<Vec2> all = a.workers;
    all.insert(all.end(), a.jobs.begin(), a.jobs.end());
    for (std::size_t i = 0; i < all.size(); ++i) {
        assert(all[i].x >= kField.left && all[i].x <= kField.right && all[i].y >= kField.top && all[i].y <= kField.bottom);
        for (std::size_t k = i + 1; k < all.size(); ++k) assert(distance(all[i], all[k]) >= 40.0);
    }
    assert(randomLayout(1, 0, kField, 40.0).workers.empty());
}

void resizingKeepsWhatIsThere() {
    const Layout six = randomLayout(11, 6, kField, 30.0);
    const Layout three = resized(six, 3, 77, kField, 30.0);
    assert(three.workers.size() == 3 && three.jobs.size() == 3);
    for (std::size_t i = 0; i < 3; ++i) {
        assert(three.workers[i].x == six.workers[i].x && three.workers[i].y == six.workers[i].y);
        assert(three.jobs[i].x == six.jobs[i].x && three.jobs[i].y == six.jobs[i].y);
    }
    const Layout eight = resized(six, 8, 77, kField, 30.0);
    assert(eight.workers.size() == 8 && eight.jobs.size() == 8);
    for (std::size_t i = 0; i < 6; ++i) {
        assert(eight.workers[i].x == six.workers[i].x && eight.jobs[i].x == six.jobs[i].x);
    }
    std::vector<Vec2> all = eight.workers;
    all.insert(all.end(), eight.jobs.begin(), eight.jobs.end());
    for (std::size_t i = 0; i < all.size(); ++i) {
        for (std::size_t k = i + 1; k < all.size(); ++k) assert(distance(all[i], all[k]) >= 30.0 - 1e-9);
    }
    assert(resized(six, 6, 1, kField, 30.0).workers.size() == 6);
}

void picking() {
    const Layout l{{{100, 100}, {130, 100}}, {{300, 200}}};
    // Nearest wins when two circles overlap the cursor.
    auto p = pickEntity(l, {118, 100}, 20.0);
    assert(p && p->worker && p->index == 1);
    p = pickEntity(l, {110, 100}, 20.0);
    assert(p && p->worker && p->index == 0);
    // Jobs are picked too.
    p = pickEntity(l, {305, 195}, 20.0);
    assert(p && !p->worker && p->index == 0);
    // Outside the radius nothing is picked, exactly on the radius it still is.
    assert(!pickEntity(l, {200, 150}, 20.0));
    assert(pickEntity(l, {300, 220}, 20.0));
    assert(!pickEntity(Layout{}, {0, 0}, 20.0));
}

void clamping() {
    const Vec2 in = clampToField({100, 100}, kField);
    assert(in.x == 100 && in.y == 100);
    const Vec2 out = clampToField({-50, 9999}, kField);
    assert(out.x == kField.left && out.y == kField.bottom);
}

void initialLayoutShowsTheProblem() {
    const Layout l = initialLayout();
    assert(l.workers.size() == static_cast<std::size_t>(kInitialCount) && l.jobs.size() == l.workers.size());
    const Costs c = costMatrix(l.workers, l.jobs);
    const Assignment g = greedy(c, processingOrder(kInitialCount, false));
    const Assignment o = hungarian(c);
    assert(isPerm(g, l.workers.size()));
    assert(crossings(l.workers, l.jobs, g).size() >= 2);
    assert(crossings(l.workers, l.jobs, o).empty());
    assert(totalCost(c, g) > 1.15 * totalCost(c, o));
    // The reversed order gives a visibly different result, that is what the chip is for.
    const Assignment r = greedy(c, processingOrder(kInitialCount, true));
    assert(std::fabs(totalCost(c, r) - totalCost(c, g)) > 10.0);
}

// Measured over many random layouts, printed so the post can quote them. The asserts only pin the shape of the result.
void measureGreedyAgainstOptimal() {
    constexpr int kScenes = 2000;
    constexpr int kWorkers = 6;
    int greedyOptimal = 0;
    int greedyCrosses = 0;
    int reverseDiffers = 0;
    double ratioSum = 0.0;
    double ratioMax = 0.0;
    int bestOrderOptimal = 0;
    for (int s = 1; s <= kScenes; ++s) {
        const Layout l = randomLayout(static_cast<std::uint32_t>(s) * 977 + 13, kWorkers, kField, 30.0);
        const Costs c = costMatrix(l.workers, l.jobs);
        const double opt = totalCost(c, hungarian(c));
        const Assignment g = greedy(c, processingOrder(kWorkers, false));
        const double gt = totalCost(c, g);
        assert(gt >= opt - 1e-9);
        assert(crossings(l.workers, l.jobs, hungarian(c)).empty());
        if (gt < opt + 1e-9) ++greedyOptimal;
        if (!crossings(l.workers, l.jobs, g).empty()) ++greedyCrosses;
        if (std::fabs(totalCost(c, greedy(c, processingOrder(kWorkers, true))) - gt) > 1e-9) ++reverseDiffers;
        ratioSum += gt / opt;
        ratioMax = std::max(ratioMax, gt / opt);
        if (greedyOrderRange(c).best < opt + 1e-9) ++bestOrderOptimal;
    }
    assert(greedyOptimal > 0 && greedyOptimal < kScenes);
    assert(greedyCrosses > 0);
    assert(reverseDiffers > 0);
    std::printf("measure: %d random layouts, %d workers and jobs each\n", kScenes, kWorkers);
    std::printf("measure: greedy (order 1..n) is optimal in %.1f%%, crosses itself in %.1f%%\n", 100.0 * greedyOptimal / kScenes, 100.0 * greedyCrosses / kScenes);
    std::printf("measure: greedy length / optimal length: mean %.3f, max %.3f\n", ratioSum / kScenes, ratioMax);
    std::printf("measure: reversed order changes the greedy total in %.1f%%\n", 100.0 * reverseDiffers / kScenes);
    std::printf("measure: the best of all 720 orders is optimal in %d of %d layouts\n", bestOrderOptimal, kScenes);
}

}  // namespace

int main() {
    distanceBasics();
    costMatrixShape();
    greedyDependsOnOrder();
    greedyRejectsBadInput();
    greedyTiesTakeTheLowerJob();
    processingOrderIsAPermutation();
    hungarianHandMatrix();
    hungarianLineExample();
    hungarianSmallAndOddInputs();
    totalCostGuards();
    bruteForceMatchesByHand();
    hungarianMatchesBruteForceOnRandomMatrices();
    hungarianMatchesBruteForceOnLayouts();
    segmentCrossing();
    crossingPointsOfAnAssignment();
    crossingsOfThreeSegmentsCountPairs();
    collinearOverlapIsNotACrossing();
    optimalNeverCrosses();
    greedyOrderRangeByHand();
    greedyOrderRangeBoundsEveryOrder();
    layoutsAreDeterministicAndSpread();
    resizingKeepsWhatIsThere();
    picking();
    clamping();
    initialLayoutShowsTheProblem();
    measureGreedyAgainstOptimal();
    std::puts("job-assignment logic: all asserts passed");
    return 0;
}
