// Tests for the hex wraparound logic. Plain asserts on hand-computed cases, no framework, no SDL.
// The lattice facts are checked against an independent oracle (a determinant), not against wrap() itself.
#undef NDEBUG
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <set>
#include <tuple>
#include <vector>

#include "logic.hpp"

using namespace hexwrap;

namespace {

bool near(double a, double b) { return std::fabs(a - b) < 1e-9; }

int coord(Cube c, int axis) { return axis == 0 ? c.q : axis == 1 ? c.r : c.s; }

/// Every cube coordinate with length <= radius, in a fixed order.
std::vector<Cube> disc(int radius) {
    std::vector<Cube> out;
    for (int q = -radius; q <= radius; ++q) {
        for (int r = -radius; r <= radius; ++r) {
            const Cube c = make(q, r);
            if (length(c) <= radius) out.push_back(c);
        }
    }
    return out;
}

/// Is v a whole-number combination of m0 and m1 = rotated m0? Cramer's rule on the axial parts, exact in integers.
/// The determinant is -(3N^2 + 3N + 1), the number of tiles of the map.
bool inLattice(int radius, Cube v) {
    const int a = 2 * radius + 1, b = -radius;          // m0 as (q, r)
    const int c = radius + 1, d = -2 * radius - 1;      // m1 as (q, r), by hand: rotateCcw(m0) = (N + 1, -(2N + 1), N)
    const int det = a * d - b * c;
    assert(det == -(3 * radius * radius + 3 * radius + 1));
    const int x = v.q * d - v.r * c;
    const int y = a * v.r - b * v.q;
    return x % det == 0 && y % det == 0;
}

void tileCounts() {
    assert(tileCount(0) == 1 && tileCount(1) == 7 && tileCount(2) == 19 && tileCount(3) == 37 && tileCount(4) == 61);
    for (int n = 0; n <= 6; ++n) assert(static_cast<int>(disc(n).size()) == tileCount(n));
}

void cubeBasics() {
    const Cube c = make(2, -5);
    assert(c.q == 2 && c.r == -5 && c.s == 3);
    assert(length(c) == 5 && length(make(0, 0)) == 0 && length(make(-3, 3)) == 3);
    assert(distance(make(1, 1), make(-1, 2)) == 2);  // cube difference (2, -1, -1), the largest part is 2
    assert(distance(make(0, 0), make(3, -3)) == 3 && distance(make(-2, 0), make(2, 0)) == 4);
    assert(distance(make(1, 1), make(1, 1)) == 0);
    assert((make(1, 2) + make(3, -1)) == make(4, 1));
    assert((make(1, 2) - make(3, -1)) == make(-2, 3));
    assert((-make(2, -3)) == make(-2, 3));
    assert(!(make(1, 2) == make(2, 1)));
}

void directionsAndRotation() {
    // The six steps in screen order E NE NW W SW SE are one 60 degree turn apart (counterclockwise on screen).
    for (int i = 0; i < 6; ++i) {
        assert(length(direction(i)) == 1);
        assert(rotateCcw(direction(i)) == direction(i + 1));
        assert(direction(i + 3) == -direction(i));
        assert(opposite(i) == (i + 3) % 6);
    }
    assert(direction(6) == direction(0) && direction(-1) == direction(5));
    assert(rotateCcw(make(1, 0)) == make(1, -1));
    // Six turns are the identity, and a turn keeps the length.
    const Cube c = make(3, -7);
    Cube t = c;
    for (int i = 0; i < 6; ++i) {
        t = rotateCcw(t);
        assert(length(t) == length(c));
    }
    assert(t == c);
}

void mirrorCentersByHand() {
    // N = 1: m0 = (2N + 1, -N, -N - 1) = (3, -1, -2), the others are 60 degree turns of it.
    const Cube expected1[6] = {{3, -1, -2}, {2, -3, 1}, {-1, -2, 3}, {-3, 1, 2}, {-2, 3, -1}, {1, 2, -3}};
    for (int k = 0; k < 6; ++k) assert(mirrorCenter(1, k) == expected1[k]);
    // N = 2: m0 = (5, -2, -3), m1 = (3, -5, 2).
    assert(mirrorCenter(2, 0) == (Cube{5, -2, -3}));
    assert(mirrorCenter(2, 1) == (Cube{3, -5, 2}));
    assert(mirrorCenter(2, 3) == (Cube{-5, 2, 3}));
    // k wraps around.
    assert(mirrorCenter(2, 6) == mirrorCenter(2, 0) && mirrorCenter(2, -1) == mirrorCenter(2, 5));
    for (int n = 0; n <= 8; ++n) {
        std::set<std::tuple<int, int, int>> seen;
        for (int k = 0; k < 6; ++k) {
            const Cube m = mirrorCenter(n, k);
            assert(m.q + m.r + m.s == 0);
            assert(length(m) == 2 * n + 1);
            assert(mirrorCenter(n, k + 3) == -m);
            seen.insert({m.q, m.r, m.s});
            assert(inLattice(n, m));
        }
        assert(seen.size() == 6);
    }
}

void wrapByHand() {
    // N = 1: (2, 0, -2) is one step east of the east corner. It lies in the copy around m0 = (3, -1, -2): (-1, 1, 0).
    assert(wrap(make(2, 0), 1) == (Cube{-1, 1, 0}));
    // (0, 2, -2) is in the copy around m5 = (1, 2, -3): (-1, 0, 1).
    assert(wrap(make(0, 2), 1) == (Cube{-1, 0, 1}));
    // N = 2: (3, 0, -3) minus m0 = (5, -2, -3) is (-2, 2, 0).
    assert(wrap(make(3, 0), 2) == (Cube{-2, 2, 0}));
    // Tiles that are already on the map stay where they are, including the edge.
    assert(wrap(make(0, 0), 2) == make(0, 0));
    assert(wrap(make(2, 0), 2) == make(2, 0));
    assert(wrap(make(-2, 2), 2) == make(-2, 2));
    // A tile that lies in no copy is returned unchanged: (5, -4, -1) for N = 1 is 3 away from m0 (3, -1, -2) and from
    // m1 (2, -3, 1), and 5 from the origin. It is in the copy of m0 + m1, which wrap() does not know.
    assert(copyIndex(make(5, -4), 1) == -1);
    assert(wrap(make(5, -4), 1) == make(5, -4));
}

void copyIndices() {
    assert(copyIndex(make(0, 0), 1) == 0);
    assert(copyIndex(mirrorCenter(1, 0), 1) == 1);
    assert(copyIndex(mirrorCenter(1, 4), 1) == 5);
    assert(copyIndex(make(2, 0), 1) == 1);
    assert(copyIndex(make(1, 0), 1) == 0);
    assert(copyIndex(make(-2, 0), 1) == 4);  // -m0 = (-3, 1, 2) = m3, and (-2, 0, 2) is next to it
    for (int n = 1; n <= 4; ++n) {
        // The seven maps do not overlap, and together they are 7 * tileCount tiles.
        int covered = 0;
        for (const Cube& h : disc(3 * n + 1)) {
            int hits = length(h) <= n ? 1 : 0;
            for (int k = 0; k < 6; ++k) hits += distance(h, mirrorCenter(n, k)) <= n ? 1 : 0;
            assert(hits <= 1);
            covered += hits;
            assert((hits == 1) == (copyIndex(h, n) >= 0));
        }
        assert(covered == 7 * tileCount(n));
    }
}

void wrapIsATranslationOntoTheMap() {
    for (int n = 1; n <= 4; ++n) {
        for (const Cube& h : disc(3 * n + 1)) {
            if (copyIndex(h, n) < 0) continue;
            const Cube w = wrap(h, n);
            assert(length(w) <= n);
            assert(inLattice(n, h - w));  // moved by a whole number of map widths, nothing else
            assert(wrap(w, n) == w);
        }
    }
}

void mapTilesAreAllDifferent() {
    // Two different tiles of the map are never the same place on the torus: their difference is no lattice vector.
    for (int n = 1; n <= 3; ++n) {
        const std::vector<Cube> tiles = disc(n);
        for (const Cube& a : tiles) {
            for (const Cube& b : tiles) {
                assert((a == b) == inLattice(n, a - b));
            }
        }
    }
}

void periodic() {
    for (int n = 1; n <= 3; ++n) {
        for (const Cube& h : disc(n)) {
            for (int k = 0; k < 6; ++k) {
                const Cube far = h + mirrorCenter(n, k);
                assert(wrap(far, n) == h);
            }
        }
    }
}

void tableMatchesFormula() {
    for (int n = 1; n <= 4; ++n) {
        const WrapTable table(n);
        int ring = 0;
        for (const Cube& h : disc(n + 1)) {
            if (length(h) != n + 1) continue;
            ++ring;
            for (int d = 0; d < 6; ++d) {
                const Cube from = h - direction(d);  // a step d from here lands on h
                if (length(from) <= n) assert(table.step(from, d) == wrap(h, n));
            }
            assert(table.lookup(h) == wrap(h, n));
            assert(length(table.lookup(h)) == n);  // the far side of a one-tile ring is always the edge
        }
        assert(ring == 6 * (n + 1));
        for (const Cube& h : disc(n)) {
            for (int d = 0; d < 6; ++d) assert(table.step(h, d) == wrap(h + direction(d), n));
        }
    }
    // By hand, N = 1: east of the east corner is the south-west corner (-1, 1, 0).
    assert(WrapTable(1).step(make(1, 0), 0) == (Cube{-1, 1, 0}));
    assert(WrapTable(1).step(make(1, 0), 1) == (Cube{-1, 0, 1}));  // north-east of the east corner: (2, -1, -1) - m0
    assert(WrapTable(1).step(make(0, 0), 3) == make(-1, 0));       // inside steps are plain additions
}

void stepBackReturns() {
    for (int n = 1; n <= 4; ++n) {
        const WrapTable table(n);
        for (const Cube& h : disc(n)) {
            for (int d = 0; d < 6; ++d) {
                const Cube there = table.step(h, d);
                assert(length(there) <= n);
                assert(table.step(there, opposite(d)) == h);
            }
        }
    }
}

void sixDifferentNeighbors() {
    for (int n = 1; n <= 4; ++n) {
        const WrapTable table(n);
        for (const Cube& h : disc(n)) {
            std::set<std::tuple<int, int, int>> seen;
            for (int d = 0; d < 6; ++d) {
                const Cube t = table.step(h, d);
                assert(!(t == h));
                seen.insert({t.q, t.r, t.s});
            }
            assert(seen.size() == 6);
            assert(distinctNeighbors(h, [&](Cube c, int d) { return table.step(c, d); }) == 6);
        }
    }
}

bool onSide(Cube c, int side, int radius) {
    // The six sides in order around the map: q = N, s = -N, r = N, q = -N, s = N, r = -N.
    constexpr int kAxis[6] = {0, 2, 1, 0, 2, 1};
    constexpr int kSign[6] = {1, -1, 1, -1, 1, -1};
    return kSign[side] * coord(c, kAxis[side]) == radius;
}

void crossingASideArrivesOnTheOppositeSide() {
    // Walk out through each side. The copies sit a bit turned against the map, so a side faces two copies: 2N + 1 of
    // the 2N + 2 steps out of a side arrive on the opposite side, the last one on a side next to the opposite one.
    // Nobody arrives on the side they left.
    for (int n = 1; n <= 6; ++n) {
        const WrapTable table(n);
        for (int side = 0; side < 6; ++side) {
            int crossings = 0;
            int opposite = 0;
            for (const Cube& h : disc(n)) {
                if (!onSide(h, side, n)) continue;
                for (int d = 0; d < 6; ++d) {
                    const Cube out = h + direction(d);
                    if (length(out) <= n) continue;
                    // A step out of the map leaves through this side if it raises the coordinate the side is made of.
                    if (!onSide(out, side, n + 1)) continue;
                    ++crossings;
                    const Cube arrived = table.step(h, d);
                    assert(length(arrived) == n);
                    assert(!onSide(arrived, side, n));
                    if (onSide(arrived, (side + 3) % 6, n)) {
                        ++opposite;
                    } else {
                        assert(onSide(arrived, (side + 2) % 6, n) || onSide(arrived, (side + 4) % 6, n));
                    }
                }
            }
            assert(crossings == 2 * (n + 1));
            assert(opposite == 2 * n + 1);
        }
    }
    // By hand, N = 1, the side s = 1 has the tiles (-1, 0, 1) and (0, -1, 1). Their four steps out land on
    // (0, 1, -1), (1, -1, 0), (1, 0, -1), (0, 1, -1): three of them on the side s = -1, one on q = 1 (and r = -1).
    const WrapTable one(1);
    assert(one.step(make(-1, 0), 2) == (Cube{0, 1, -1}));
    assert(one.step(make(-1, 0), 3) == (Cube{1, -1, 0}));
    assert(one.step(make(0, -1), 2) == (Cube{1, 0, -1}));
    assert(one.step(make(0, -1), 3) == (Cube{0, 1, -1}));
    // By hand, N = 2, the east side (q == 2): the tile (2, -1, -1) steps east to (3, -1, -2), minus m0 (5, -2, -3).
    assert(WrapTable(2).step(make(2, -1), 0) == (Cube{-2, 1, 1}));
    // The top row (r == -2): (1, -2, 1) steps NW to (1, -3, 2), which belongs to the copy around m1 = (3, -5, 2):
    // (-2, 2, 0), a tile of the bottom row (r == 2).
    assert(WrapTable(2).step(make(1, -2), 2) == (Cube{-2, 2, 0}));
}

void straightWalkVisitsEveryTile() {
    // N = 1, east from the middle, by hand: seven different tiles, then back at the start.
    const WrapTable one(1);
    const Cube path[7] = {{0, 0, 0}, {1, 0, -1}, {-1, 1, 0}, {0, 1, -1}, {0, -1, 1}, {1, -1, 0}, {-1, 0, 1}};
    Cube p = make(0, 0);
    for (int i = 0; i < 7; ++i) {
        assert(p == path[i]);
        p = one.step(p, 0);
    }
    assert(p == make(0, 0));
    // N = 1..4 have 7, 19, 37 and 61 tiles. Those are prime, so a line in any direction is one long cycle through
    // every tile of the map before it closes (the group of tiles has no smaller subgroup).
    for (int n = 1; n <= 4; ++n) {
        const WrapTable table(n);
        for (int d = 0; d < 6; ++d) {
            std::set<std::tuple<int, int, int>> seen;
            Cube here = make(0, 0);
            int steps = 0;
            do {
                seen.insert({here.q, here.r, here.s});
                here = table.step(here, d);
                ++steps;
            } while (!(here == make(0, 0)) && steps < 1000);
            assert(steps == tileCount(n));
            assert(static_cast<int>(seen.size()) == tileCount(n));
        }
    }
}

void naiveJumpToTheOppositeTile() {
    // The tempting version: leave the map, appear on the opposite tile (negate the one you stepped off).
    assert(naiveStep(make(0, 0), 0, 2) == make(1, 0));  // inside steps are the same
    assert(naiveStep(make(2, 0), 0, 2) == (Cube{-2, 0, 2}));
    assert(!(naiveStep(make(2, 0), 0, 2) == WrapTable(2).step(make(2, 0), 0)));  // (-2, 2, 0) is the right one
    // N = 2, the east corner: three steps leave the map and all three land on the same tile, so the corner has only
    // 3 inside neighbors + 1 = 4 different ones instead of 6.
    assert(distinctNeighbors(make(2, 0), [](Cube c, int d) { return naiveStep(c, d, 2); }) == 4);
    assert(distinctNeighbors(make(2, 0), [](Cube c, int d) { return WrapTable(2).step(c, d); }) == 6);
    // A tile in the middle of the map is not affected.
    assert(distinctNeighbors(make(0, 0), [](Cube c, int d) { return naiveStep(c, d, 2); }) == 6);
    // Walking east from the middle never leaves row 0: 2N + 1 tiles (N = 1: (0,0,0) (1,0,-1) (-1,0,1) and back).
    Cube p = make(0, 0);
    std::set<std::tuple<int, int, int>> seen;
    for (int i = 0; i < 40; ++i) {
        seen.insert({p.q, p.r, p.s});
        p = naiveStep(p, 0, 1);
        assert(p.r == 0);
    }
    assert(seen.size() == 3);
    // On the real thing the same walk shows all 7.
    const WrapTable table(1);
    seen.clear();
    p = make(0, 0);
    for (int i = 0; i < 40; ++i) {
        seen.insert({p.q, p.r, p.s});
        p = table.step(p, 0);
    }
    assert(seen.size() == 7);
}

void pixelMath() {
    // Pointy hexes, size 1: x = sqrt(3) (q + r / 2), y = 3/2 r.
    assert(near(center(make(0, 0)).x, 0.0) && near(center(make(0, 0)).y, 0.0));
    assert(near(center(make(1, 0)).x, kSqrt3) && near(center(make(1, 0)).y, 0.0));
    assert(near(center(make(0, 1)).x, kSqrt3 / 2.0) && near(center(make(0, 1)).y, 1.5));
    assert(near(center(make(-1, -1)).x, -1.5 * kSqrt3) && near(center(make(-1, -1)).y, -1.5));
    // Every neighbor is sqrt(3) away, whatever the direction.
    for (int d = 0; d < 6; ++d) {
        const Point p = center(direction(d));
        assert(near(std::hypot(p.x, p.y), kSqrt3));
    }
    // Direction 1 is up-right on screen (y down), direction 4 down-left.
    assert(center(direction(1)).x > 0.0 && center(direction(1)).y < 0.0);
    assert(center(direction(4)).x < 0.0 && center(direction(4)).y > 0.0);
    // Corners lie on the unit circle, corner 0 is up-right, and side k (towards direction k) is corner (6 - k) % 6 to
    // the next one: side 0 (east) is corners 0 and 1, both with x = sqrt(3)/2.
    for (int i = 0; i < 6; ++i) assert(near(std::hypot(corner(i).x, corner(i).y), 1.0));
    assert(near(corner(0).x, kSqrt3 / 2.0) && near(corner(0).y, -0.5));
    assert(near(corner(1).x, kSqrt3 / 2.0) && near(corner(1).y, 0.5));
    for (int k = 0; k < 6; ++k) {
        const int a = sideCorner(k);
        const Point mid{(corner(a).x + corner((a + 1) % 6).x) / 2.0, (corner(a).y + corner((a + 1) % 6).y) / 2.0};
        const Point c = center(direction(k));
        assert(near(mid.x * 2.0, c.x) && near(mid.y * 2.0, c.y));  // the side midpoint is halfway to the neighbor
    }
}

void insideTheHex() {
    assert(insideHex({0.0, 0.0}));
    assert(insideHex({0.86, 0.0}) && !insideHex({0.87, 0.0}));
    assert(insideHex({-0.86, 0.0}) && !insideHex({-0.87, 0.0}));
    assert(insideHex({0.0, 0.99}) && !insideHex({0.0, 1.01}));
    assert(insideHex({0.0, -0.99}) && !insideHex({0.0, -1.01}));
    // Near the up-right side, which is 0.866 away from the center (y points down).
    assert(insideHex({0.8, -0.5}) && !insideHex({0.8, -0.6}));
    assert(insideHex({-0.8, 0.5}) && !insideHex({-0.8, 0.6}));
    // Every corner is on the edge: slightly pulled in it is inside, slightly pushed out it is not.
    for (int i = 0; i < 6; ++i) {
        assert(insideHex({corner(i).x * 0.99, corner(i).y * 0.99}));
        assert(!insideHex({corner(i).x * 1.01, corner(i).y * 1.01}));
    }
}

}  // namespace

int main() {
    tileCounts();
    cubeBasics();
    directionsAndRotation();
    mirrorCentersByHand();
    wrapByHand();
    copyIndices();
    wrapIsATranslationOntoTheMap();
    mapTilesAreAllDifferent();
    periodic();
    tableMatchesFormula();
    stepBackReturns();
    sixDifferentNeighbors();
    crossingASideArrivesOnTheOppositeSide();
    straightWalkVisitsEveryTile();
    naiveJumpToTheOppositeTile();
    pixelMath();
    insideTheHex();
    std::puts("hex-wraparound logic: all asserts passed");
    return 0;
}
