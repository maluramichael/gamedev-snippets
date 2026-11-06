// Tests for the line and circle logic. Plain asserts on hand-computed cases, no framework, no SDL.
// Cells are integer coordinates, a cell (x, y) is the unit square centered on (x, y).
#undef NDEBUG
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <set>
#include <utility>

#include "logic.hpp"

using namespace lcd;

namespace {

bool has(const Cells& cells, int x, int y) { return std::find(cells.begin(), cells.end(), Cell{x, y}) != cells.end(); }

std::set<std::pair<int, int>> asSet(const Cells& cells) {
    std::set<std::pair<int, int>> s;
    for (const Cell& c : cells) s.insert({c.x, c.y});
    return s;
}

bool sameCells(const Cells& a, const Cells& b) { return asSet(a) == asSet(b); }

bool subset(const Cells& a, const Cells& b) {
    const auto big = asSet(b);
    return std::all_of(a.begin(), a.end(), [&](const Cell& c) { return big.count({c.x, c.y}) == 1; });
}

/// Independent oracle: does the segment between the centers of a and b touch the closed unit square around c?
/// Separating axis test with all coordinates doubled, so everything stays integer.
bool segmentTouchesCell(Cell a, Cell b, Cell c) {
    const long long ax = 2LL * a.x, ay = 2LL * a.y, bx = 2LL * b.x, by = 2LL * b.y;
    const long long x0 = 2LL * c.x - 1, x1 = 2LL * c.x + 1, y0 = 2LL * c.y - 1, y1 = 2LL * c.y + 1;
    if (std::max(ax, bx) < x0 || std::min(ax, bx) > x1) return false;
    if (std::max(ay, by) < y0 || std::min(ay, by) > y1) return false;
    const long long dx = bx - ax, dy = by - ay;
    long long lo = 0, hi = 0;
    bool first = true;
    for (const long long px : {x0, x1}) {
        for (const long long py : {y0, y1}) {
            const long long side = dx * (py - ay) - dy * (px - ax);
            if (first) {
                lo = hi = side;
                first = false;
            }
            lo = std::min(lo, side);
            hi = std::max(hi, side);
        }
    }
    return lo <= 0 && hi >= 0;
}

int diagonalSteps(const Cells& path) {
    int n = 0;
    for (std::size_t i = 1; i < path.size(); ++i) {
        if (path[i].x != path[i - 1].x && path[i].y != path[i - 1].y) ++n;
    }
    return n;
}

void roundingHelpers() {
    assert(floorDiv(7, 2) == 3);
    assert(floorDiv(-7, 2) == -4);  // floor, not truncation
    assert(floorDiv(-6, 2) == -3);
    assert(isqrtFloor(0) == 0 && isqrtFloor(1) == 1 && isqrtFloor(3) == 1 && isqrtFloor(4) == 2 && isqrtFloor(24) == 4 && isqrtFloor(25) == 5);
    // radius = distance rounded: sqrt(2) = 1.41 -> 1, sqrt(3) = 1.73 -> 2, sqrt(6) = 2.45 -> 2, sqrt(7) = 2.65 -> 3
    assert(radiusOf({5, 5}, {5, 5}) == 0);
    assert(radiusOf({0, 0}, {1, 0}) == 1);
    assert(radiusOf({0, 0}, {1, 1}) == 1);
    assert(radiusOf({0, 0}, {1, -1}) == 1);
    assert(radiusOf({0, 0}, {2, 1}) == 2);  // 2.24
    assert(radiusOf({0, 0}, {-1, 2}) == 2);
    assert(radiusOf({0, 0}, {6, 8}) == 10);
    assert(radiusOf({0, 0}, {3, 4}) == 5);
}

void lerpLines() {
    assert(lerpLine({2, 3}, {2, 3}) == (Cells{{2, 3}}));  // zero length: one cell, no division by zero
    assert(lerpLine({0, 0}, {5, 0}).size() == 6);
    assert(has(lerpLine({0, 0}, {5, 0}), 5, 0));
    // diagonal: every step moves both axes
    assert(lerpLine({0, 0}, {3, 3}) == (Cells{{0, 0}, {1, 1}, {2, 2}, {3, 3}}));
    // (0,0) -> (3,1): y = step / 3 = 0, 0.33, 0.67, 1 rounds to 0, 0, 1, 1
    assert(lerpLine({0, 0}, {3, 1}) == (Cells{{0, 0}, {1, 0}, {2, 1}, {3, 1}}));
    // negative direction: y = -0.33 -> 0, -0.67 -> -1
    assert(lerpLine({0, 0}, {-3, -1}) == (Cells{{0, 0}, {-1, 0}, {-2, -1}, {-3, -1}}));
    // The tie: (0,0) -> (4,1) is at y = 0.5 in the middle. Ties round up, so the mirrored line (4,-1) rounds up as well,
    // which is toward 0, and the two lines are not mirror images of each other.
    assert(has(lerpLine({0, 0}, {4, 1}), 2, 1) && !has(lerpLine({0, 0}, {4, 1}), 2, 0));
    assert(has(lerpLine({0, 0}, {4, -1}), 2, 0) && !has(lerpLine({0, 0}, {4, -1}), 2, -1));
}

void bresenhamLines() {
    assert(bresenhamLine({2, 3}, {2, 3}) == (Cells{{2, 3}}));
    assert(bresenhamLine({0, 0}, {3, 3}) == (Cells{{0, 0}, {1, 1}, {2, 2}, {3, 3}}));
    assert(bresenhamLine({0, 0}, {3, 1}) == (Cells{{0, 0}, {1, 0}, {2, 1}, {3, 1}}));
    assert(bresenhamLine({0, 0}, {0, -3}) == (Cells{{0, 0}, {0, -1}, {0, -2}, {0, -3}}));
    // Tie again: (0,0) -> (4,1) goes through (2,1) ...
    assert(bresenhamLine({0, 0}, {4, 1}) == (Cells{{0, 0}, {1, 0}, {2, 1}, {3, 1}, {4, 1}}));
    // ... and the same line drawn from the other end goes through (2,0). Same endpoints, other cells.
    assert(bresenhamLine({4, 1}, {0, 0}) == (Cells{{4, 1}, {3, 1}, {2, 0}, {1, 0}, {0, 0}}));
    // The lerp line is the same from both ends: reversing it gives exactly the same cells.
    Cells fwd = lerpLine({0, 0}, {4, 1});
    Cells back = lerpLine({4, 1}, {0, 0});
    std::reverse(back.begin(), back.end());
    assert(fwd == back);
}

void supercoverLines() {
    // (0,0) -> (4,2) never meets a corner: a plain walk of 4 + 2 + 1 cells
    assert(supercoverLine({0, 0}, {4, 2}, Corner::Both) == (Cells{{0, 0}, {1, 0}, {1, 1}, {2, 1}, {3, 1}, {3, 2}, {4, 2}}));
    assert(supercoverLine({0, 0}, {4, 2}, Corner::Diagonal) == supercoverLine({0, 0}, {4, 2}, Corner::Both));
    // (0,0) -> (3,1) passes exactly through the corner at (1.5, 0.5)
    assert(supercoverLine({0, 0}, {3, 1}, Corner::Diagonal) == (Cells{{0, 0}, {1, 0}, {2, 1}, {3, 1}}));
    const Cells both = supercoverLine({0, 0}, {3, 1}, Corner::Both);
    assert(both.size() == 6);  // the two cells that only touch the line at the corner come with it
    assert(has(both, 2, 0) && has(both, 1, 1) && has(both, 1, 0) && has(both, 2, 1));
    // (0,0) -> (2,2): the line runs along the corners of every step
    assert(supercoverLine({0, 0}, {2, 2}, Corner::Diagonal) == (Cells{{0, 0}, {1, 1}, {2, 2}}));
    assert(supercoverLine({0, 0}, {2, 2}, Corner::Both).size() == 7);
    // degenerate and axis aligned
    assert(supercoverLine({4, 4}, {4, 4}, Corner::Both) == (Cells{{4, 4}}));
    assert(supercoverLine({0, 0}, {0, 3}, Corner::Both) == (Cells{{0, 0}, {0, 1}, {0, 2}, {0, 3}}));
    assert(supercoverLine({0, 0}, {-3, 0}, Corner::Both) == (Cells{{0, 0}, {-1, 0}, {-2, 0}, {-3, 0}}));
}

void linesAgainstTheOracle() {
    // Every line between two cells of a 7x7 patch, all eight directions and every slope in between.
    for (int ax = 0; ax < 7; ++ax) {
        for (int ay = 0; ay < 7; ++ay) {
            for (int bx = 0; bx < 7; ++bx) {
                for (int by = 0; by < 7; ++by) {
                    const Cell a{ax, ay}, b{bx, by};
                    const int n = std::max(std::abs(bx - ax), std::abs(by - ay));
                    const Cells lerp = lerpLine(a, b);
                    const Cells bres = bresenhamLine(a, b);
                    const Cells diag = supercoverLine(a, b, Corner::Diagonal);
                    const Cells both = supercoverLine(a, b, Corner::Both);
                    // lerp and Bresenham take exactly one cell per step of the long axis, starting and ending on the endpoints
                    assert(static_cast<int>(lerp.size()) == n + 1 && static_cast<int>(bres.size()) == n + 1);
                    assert(lerp.front() == a && lerp.back() == b && bres.front() == a && bres.back() == b);
                    // supercover: the cells are exactly the ones the segment touches (oracle), in both corner modes' limits
                    std::set<std::pair<int, int>> touched;
                    for (int x = -1; x < 8; ++x) {
                        for (int y = -1; y < 8; ++y) {
                            if (segmentTouchesCell(a, b, {x, y})) touched.insert({x, y});
                        }
                    }
                    assert(asSet(both) == touched);
                    assert(both.size() == touched.size());  // no cell twice
                    // the diagonal mode is the same without the two side cells at each exact corner
                    assert(subset(diag, both));
                    assert(both.size() == diag.size() + 2 * static_cast<std::size_t>(diagonalSteps(diag)));
                    // both other lines only pick cells the segment touches
                    assert(subset(lerp, both) && subset(bres, both));
                    assert(diag.front() == a && diag.back() == b);
                    // consecutive cells of the diagonal path are neighbours (4 or 8)
                    for (std::size_t i = 1; i < diag.size(); ++i) {
                        assert(std::abs(diag[i].x - diag[i - 1].x) <= 1 && std::abs(diag[i].y - diag[i - 1].y) <= 1);
                        assert(diag[i] != diag[i - 1]);
                    }
                }
            }
        }
    }
}

void lineOfSightRules() {
    Walls walls(10, 10);
    const Cell a{0, 0}, b{4, 0};
    const Cells line = bresenhamLine(a, b);
    assert(lineOfSight(line, a, b, walls));
    walls.set({2, 0}, true);
    assert(!lineOfSight(line, a, b, walls));
    walls.set({2, 0}, false);
    // a wall on the viewer's own cell or on the target does not block, the sight is about the cells in between
    walls.set(a, true);
    walls.set(b, true);
    assert(lineOfSight(line, a, b, walls));
    assert(walls.count() == 2);
    assert(!walls.at({-1, 0}) && !walls.at({0, 10}));  // outside the grid there is no wall
    walls.set({-1, 0}, true);                          // and setting outside is ignored
    assert(walls.count() == 2);
    walls.clear();
    assert(walls.count() == 0);
}

void startingPosition() {
    // The demo opens with a wall next to a corner that the line passes exactly through.
    const Cell a = linePresetA(), b = linePresetB();
    const Walls walls = lineWalls();
    assert(walls.count() >= 1);
    const Cells lerp = lerpLine(a, b);
    const Cells bres = bresenhamLine(a, b);
    const Cells diag = supercoverLine(a, b, Corner::Diagonal);
    const Cells both = supercoverLine(a, b, Corner::Both);
    assert(lineOfSight(lerp, a, b, walls));
    assert(lineOfSight(bres, a, b, walls));
    assert(lineOfSight(diag, a, b, walls));    // the diagonal step squeezes through the corner
    assert(!lineOfSight(both, a, b, walls));   // the wall touches the line at that corner
    assert(both.size() > lerp.size());
}

void circleRadius() {
    assert(discDistance({0, 0}, 0) == (Cells{{0, 0}}));
    assert(discDistance({0, 0}, 1).size() == 5);  // plus shape
    assert(discDistance({0, 0}, 2).size() == 13);
    assert(discDistance({0, 0}, 3).size() == 29);
    assert(discDistance({0, 0}, 4).size() == 49);
    assert(discDistance({0, 0}, 5).size() == 81);
    // the circle is where it is told to be
    const Cells d = discDistance({10, -4}, 2);
    assert(has(d, 12, -4) && has(d, 10, -6) && !has(d, 12, -3) && has(d, 11, -3));
    // row by row (isqrt per row) gives exactly the cells of the distance test
    for (int r = 0; r <= 40; ++r) assert(sameCells(discByRows({3, 4}, r), discDistance({3, 4}, r)));
}

void midpointCircle() {
    // octant of r = 5, worked out by hand: rows y = 0..3 have x = 5, 5, 5, 4 and the next row would be x = 3 < y = 4
    assert(midpointOctant(5) == (Cells{{5, 0}, {5, 1}, {5, 2}, {4, 3}}));
    assert(midpointOctant(0) == (Cells{{0, 0}}));
    assert(midpointOctant(1) == (Cells{{1, 0}}));
    // r = 1 is the plus without its middle
    assert(sameCells(ringMidpoint({0, 0}, 1), (Cells{{1, 0}, {-1, 0}, {0, 1}, {0, -1}})));
    // r = 2: 12 cells, the 5x5 block without its corners and without the 3x3 middle
    const Cells r2 = ringMidpoint({0, 0}, 2);
    assert(r2.size() == 12);
    assert(has(r2, 2, 1) && has(r2, 1, 2) && has(r2, -2, -1) && has(r2, 0, 2) && !has(r2, 1, 1) && !has(r2, 2, 2));
    // eight fold symmetry
    for (int r = 0; r <= 60; ++r) {
        const Cells ring = ringMidpoint({0, 0}, r);
        const auto s = asSet(ring);
        for (const Cell& c : ring) {
            assert(s.count({-c.x, c.y}) == 1 && s.count({c.x, -c.y}) == 1 && s.count({c.y, c.x}) == 1);
        }
    }
    // The octant picks, for each row, the integer closest to the exact x. Checked against floating point for many radii.
    for (int r = 0; r <= 300; ++r) {
        Cells expected;
        for (int y = 0; y <= r; ++y) {
            const int x = static_cast<int>(std::floor(std::sqrt(static_cast<double>(r) * r - static_cast<double>(y) * y) + 0.5));
            if (x >= y) expected.push_back({x, y});
        }
        assert(midpointOctant(r) == expected);
    }
    // centered somewhere else
    const Cells moved = ringMidpoint({7, 9}, 5);
    assert(has(moved, 12, 9) && has(moved, 7, 4) && has(moved, 11, 12) && !has(moved, 7, 9));
}

void ringsAndGaps() {
    // r = 1: both rings are the same four cells
    assert(sameCells(ringDistance({0, 0}, 1), ringMidpoint({0, 0}, 1)));
    // r = 2: the distance test ring keeps cells with d <= 2 that touch the outside, 8 cells; the midpoint ring has 12
    const Cells dist2 = ringDistance({0, 0}, 2);
    assert(dist2.size() == 8);
    assert(has(dist2, 1, 1) && has(dist2, 2, 0) && !has(dist2, 2, 1) && !has(dist2, 1, 0));
    // The per row version at r = 5: the top cell (0,5) and the cells in row 4 are three columns apart.
    const Cells rows5 = ringRowsOnly({0, 0}, 5);
    assert(has(rows5, 0, 5) && has(rows5, 3, 4) && has(rows5, -3, 4) && !has(rows5, 1, 4) && !has(rows5, 2, 4));
    assert(!connected8(rows5));
    assert(connected8(ringMidpoint({0, 0}, 5)));
    assert(connected8(ringDistance({0, 0}, 5)));
    assert(connected8(Cells{}) == false);  // nothing is not a ring
    assert(connected8(Cells{{3, 3}}));
    assert(!connected8(Cells{{0, 0}, {2, 0}}));
    assert(connected8(Cells{{0, 0}, {1, 1}, {2, 2}}));
    for (int r = 1; r <= 60; ++r) {
        assert(connected8(ringMidpoint({0, 0}, r)));
        assert(connected8(ringDistance({0, 0}, r)));
    }
    // a ring is always inside the disc of the same radius plus one: nothing wanders off
    for (int r = 1; r <= 30; ++r) {
        assert(subset(ringMidpoint({0, 0}, r), discDistance({0, 0}, r + 1)));
        assert(subset(ringDistance({0, 0}, r), discDistance({0, 0}, r)));
    }
}

void midpointFill() {
    // r = 5: the midpoint ring's extreme cells per row span the filled disc
    const Cells fill = discMidpoint({0, 0}, 5);
    assert(has(fill, 0, 5) && has(fill, 0, 0) && has(fill, 5, 0) && has(fill, 4, 3) && !has(fill, 5, 3) && !has(fill, 5, 5));
    assert(connected8(fill));
    // the filled disc contains its ring
    assert(subset(ringMidpoint({0, 0}, 5), fill));
    assert(discMidpoint({0, 0}, 0) == (Cells{{0, 0}}));
}

void octants() {
    assert(inFirstOctant(5, 2) && inFirstOctant(5, 5) && inFirstOctant(5, 0));
    assert(!inFirstOctant(2, 5) && !inFirstOctant(5, -2) && !inFirstOctant(-5, 2) && !inFirstOctant(0, 5));
    // the midpoint ring has exactly the cells of the octant plus their mirror images: 8 per generated cell, minus the
    // cells on the mirror axes that are counted once
    assert(ringMidpoint({0, 0}, 5).size() == 28);  // (5,0): 4, (5,1): 8, (5,2): 8, (4,3): 8
    assert(ringMidpoint({0, 0}, 4).size() == 24);  // octant (4,0), (4,1), (3,2), (3,3): 4 + 8 + 8 + 4
}

}  // namespace

int main() {
    roundingHelpers();
    lerpLines();
    bresenhamLines();
    supercoverLines();
    linesAgainstTheOracle();
    lineOfSightRules();
    startingPosition();
    circleRadius();
    midpointCircle();
    ringsAndGaps();
    midpointFill();
    octants();
    std::puts("line-circle-drawing logic: all asserts passed");
    return 0;
}
