// Tests for the hex pathfinding logic. Plain asserts on hand-computed cases, no framework, no SDL.
// Offset coordinates are (col, row) of the storage grid, axial ones (q, r). The rows are "odd-r": odd rows sit half a
// cell to the right.
#undef NDEBUG
#include <algorithm>
#include <cassert>
#include <climits>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "logic.hpp"

using namespace ha;

namespace {

bool near(double a, double b, double eps = 1e-9) { return std::fabs(a - b) < eps; }

std::vector<int> sortedNeighbors(const Grid& g, Topology t, int col, int row) {
    const Neighbors n = neighborsOf(g, t, g.id(col, row));
    std::vector<int> out(n.begin(), n.end());
    std::sort(out.begin(), out.end());
    return out;
}

std::vector<int> ids(const Grid& g, std::vector<std::pair<int, int>> cells) {
    std::vector<int> out;
    for (const auto& c : cells) out.push_back(g.id(c.first, c.second));
    std::sort(out.begin(), out.end());
    return out;
}

bool contains(const std::vector<int>& v, int x) { return std::find(v.begin(), v.end(), x) != v.end(); }

/// A path is valid when it starts and ends right, every step is a neighbor, and its cost is the sum of the entered cells.
void checkPath(const Grid& g, Topology t, const SearchResult& r, int start, int goal) {
    assert(r.found);
    assert(!r.path.empty());
    assert(r.path.front() == start && r.path.back() == goal);
    int sum = 0;
    for (std::size_t i = 1; i < r.path.size(); ++i) {
        const Neighbors n = neighborsOf(g, t, r.path[i - 1]);
        assert(std::find(n.begin(), n.end(), r.path[i]) != n.end());
        sum += g.terrain[static_cast<std::size_t>(r.path[i])];
    }
    assert(sum == r.cost);
}

void hexDistances() {
    assert(hexDistance({0, 0}, {0, 0}) == 0);
    assert(hexDistance({0, 0}, {3, 0}) == 3);
    assert(hexDistance({0, 0}, {2, -1}) == 2);   // dq -2, dr 1, ds 1
    assert(hexDistance({0, 0}, {-2, 3}) == 3);   // dq 2, dr -3, ds 1
    assert(hexDistance({0, 0}, {4, -4}) == 4);   // along a diagonal axis
    assert(hexDistance({0, 0}, {3, 3}) == 6);    // q and r of the same sign add up
    assert(hexDistance({1, 1}, {-2, 4}) == 3);
    assert(hexDistance({1, 1}, {-2, 4}) == hexDistance({-2, 4}, {1, 1}));
    for (const Axial d : kHexDirections) assert(hexDistance({0, 0}, d) == 1);
}

void offsetConversion() {
    assert((offsetToAxial(0, 0) == Axial{0, 0}));
    assert((offsetToAxial(0, 1) == Axial{0, 1}));
    assert((offsetToAxial(0, 2) == Axial{-1, 2}));
    assert((offsetToAxial(4, 3) == Axial{3, 3}));
    assert(axialToCol(Axial{-1, 2}) == 0);
    assert(axialToCol(Axial{3, 3}) == 4);
    for (int row = -4; row < 8; ++row) {
        for (int col = -4; col < 8; ++col) {
            const Axial a = offsetToAxial(col, row);
            assert(axialToCol(a) == col && a.r == row);
        }
    }
}

void hexNeighborSets() {
    const Grid g(5, 5);
    // Even row: east, west, and two cells above and below that lean to the left.
    assert((sortedNeighbors(g, Topology::Hex, 2, 2) == ids(g, {{3, 2}, {1, 2}, {1, 1}, {2, 1}, {1, 3}, {2, 3}})));
    // Odd row: the two above and below lean to the right.
    assert((sortedNeighbors(g, Topology::Hex, 2, 1) == ids(g, {{3, 1}, {1, 1}, {2, 0}, {3, 0}, {2, 2}, {3, 2}})));
    // Corners and edges: the shifted rows stick out on the right, so the right edge of an odd row has less.
    assert((sortedNeighbors(g, Topology::Hex, 0, 0) == ids(g, {{1, 0}, {0, 1}})));
    assert((sortedNeighbors(g, Topology::Hex, 0, 1) == ids(g, {{1, 1}, {1, 0}, {0, 0}, {0, 2}, {1, 2}})));
    assert((sortedNeighbors(g, Topology::Hex, 4, 0) == ids(g, {{3, 0}, {3, 1}, {4, 1}})));
    assert((sortedNeighbors(g, Topology::Hex, 4, 1) == ids(g, {{4, 0}, {3, 1}, {4, 2}})));
}

void hexNeighborsAreSymmetric() {
    const Grid g(7, 6);
    for (int a = 0; a < g.width * g.height; ++a) {
        for (const int b : neighborsOf(g, Topology::Hex, a)) {
            const Neighbors back = neighborsOf(g, Topology::Hex, b);
            assert(std::find(back.begin(), back.end(), a) != back.end());
            assert(distance(g, Topology::Hex, a, b) == 1);
        }
    }
}

void squareNeighborSets() {
    Grid g(5, 4);
    assert((sortedNeighbors(g, Topology::Square, 0, 0) == ids(g, {{1, 0}, {0, 1}})));
    assert((sortedNeighbors(g, Topology::Square, 2, 0) == ids(g, {{1, 0}, {3, 0}, {2, 1}})));
    assert((sortedNeighbors(g, Topology::Square, 2, 1) == ids(g, {{1, 1}, {3, 1}, {2, 0}, {2, 2}})));
    g.terrain[static_cast<std::size_t>(g.id(1, 0))] = kWall;
    assert((sortedNeighbors(g, Topology::Square, 0, 0) == ids(g, {{0, 1}})));  // walls are not neighbors
    Grid h(5, 5);
    h.terrain[static_cast<std::size_t>(h.id(3, 2))] = kWall;
    assert(!contains(sortedNeighbors(h, Topology::Hex, 2, 2), h.id(3, 2)));
    assert(sortedNeighbors(h, Topology::Hex, 2, 2).size() == 5);
}

void distancesInTheGrid() {
    const Grid g(7, 7);
    assert(distance(g, Topology::Hex, g.id(0, 3), g.id(6, 3)) == 6);
    assert(distance(g, Topology::Hex, g.id(0, 0), g.id(0, 2)) == 2);
    assert(distance(g, Topology::Square, g.id(0, 0), g.id(6, 6)) == 12);
    assert(distance(g, Topology::Square, g.id(2, 5), g.id(2, 5)) == 0);
}

void openFieldAstar() {
    const Grid g(7, 7);
    const int a = g.id(0, 3);
    const int b = g.id(6, 3);
    const SearchResult hex = search(g, Topology::Hex, a, b, true);
    checkPath(g, Topology::Hex, hex, a, b);
    assert(hex.cost == 6 && hex.path.size() == 7);
    const SearchResult sq = search(g, Topology::Square, g.id(0, 0), g.id(6, 6), true);
    checkPath(g, Topology::Square, sq, g.id(0, 0), g.id(6, 6));
    assert(sq.cost == 12 && sq.path.size() == 13);
}

void heuristicSavesWork() {
    // Same row, so the shortest way is unique: every cell off the line has f above the optimum and is never expanded.
    const Grid g(11, 11);
    const int a = g.id(0, 5);
    const int b = g.id(10, 5);
    const SearchResult hex = search(g, Topology::Hex, a, b, true);
    assert(hex.found && hex.cost == 10 && hex.expanded == 11);
    const SearchResult sq = search(g, Topology::Square, a, b, true);
    assert(sq.found && sq.cost == 10 && sq.expanded == 11);
    // Without the heuristic (Dijkstra) the search spreads out in a disc. On the square grid every cell with distance
    // below 10 from the start comes first: 11+11+11+11+11+9+7+5+3+1 = 80.
    const SearchResult sqDijkstra = search(g, Topology::Square, a, b, false);
    assert(sqDijkstra.found && sqDijkstra.cost == 10 && sqDijkstra.expanded >= 80);
    const SearchResult hexDijkstra = search(g, Topology::Hex, a, b, false);
    assert(hexDijkstra.found && hexDijkstra.cost == 10 && hexDijkstra.expanded > 33);
}

void hexWallDetour() {
    Grid g(5, 3);
    g.terrain[static_cast<std::size_t>(g.id(2, 1))] = kWall;
    const int a = g.id(0, 1);
    const int b = g.id(4, 1);
    // Same row means a unique shortest way of 4 steps, the wall cuts it, the detour takes one more.
    const SearchResult r = search(g, Topology::Hex, a, b, true);
    checkPath(g, Topology::Hex, r, a, b);
    assert(r.cost == 5 && r.path.size() == 6);
    assert(!contains(r.path, g.id(2, 1)));
}

void terrainCosts() {
    // Entering a cell costs its terrain: forest 3 + grass 1. The start cell is free.
    Grid corridor(3, 1);
    corridor.terrain[static_cast<std::size_t>(corridor.id(1, 0))] = kForest;
    const SearchResult r = search(corridor, Topology::Square, corridor.id(0, 0), corridor.id(2, 0), true);
    checkPath(corridor, Topology::Square, r, corridor.id(0, 0), corridor.id(2, 0));
    assert(r.cost == 4);

    // Swamp (6) in the middle: straight would cost 7, around through row 1 costs 4.
    Grid sq(3, 3);
    sq.terrain[static_cast<std::size_t>(sq.id(1, 0))] = kSwamp;
    const SearchResult around = search(sq, Topology::Square, sq.id(0, 0), sq.id(2, 0), true);
    checkPath(sq, Topology::Square, around, sq.id(0, 0), sq.id(2, 0));
    assert(around.cost == 4 && around.path.size() == 5);
    assert(!contains(around.path, sq.id(1, 0)));

    // Hex, 3 x 2: straight through the swamp is 7, (0,0) (0,1) (1,1) (2,0) is 3.
    Grid hex(3, 2);
    hex.terrain[static_cast<std::size_t>(hex.id(1, 0))] = kSwamp;
    const SearchResult h = search(hex, Topology::Hex, hex.id(0, 0), hex.id(2, 0), true);
    checkPath(hex, Topology::Hex, h, hex.id(0, 0), hex.id(2, 0));
    assert(h.cost == 3 && h.path.size() == 4);
    assert(!contains(h.path, hex.id(1, 0)));
}

void unreachableAndDegenerate() {
    for (const Topology t : {Topology::Hex, Topology::Square}) {
        Grid g(5, 3);
        for (int row = 0; row < 3; ++row) g.terrain[static_cast<std::size_t>(g.id(2, row))] = kWall;
        const SearchResult r = search(g, t, g.id(0, 1), g.id(4, 1), true);
        assert(!r.found && r.path.empty());
        assert(r.expanded == 6);  // the whole left side (2 columns x 3 rows) is explored, then the queue is empty

        const Grid open(5, 3);
        const SearchResult same = search(open, t, open.id(1, 1), open.id(1, 1), true);
        assert(same.found && same.cost == 0 && same.expanded == 1);
        assert(same.path.size() == 1 && same.path[0] == open.id(1, 1));

        Grid walled(5, 3);
        walled.terrain[static_cast<std::size_t>(walled.id(4, 1))] = kWall;
        assert(!search(walled, t, walled.id(0, 1), walled.id(4, 1), true).found);  // goal is a wall
        assert(!search(walled, t, walled.id(4, 1), walled.id(0, 1), true).found);  // start is a wall
        assert(!search(walled, t, -1, walled.id(0, 1), true).found);
        assert(!search(walled, t, walled.id(0, 1), 15, true).found);  // one past the last id
    }
}

/// Independent check: relax every edge until nothing changes (Bellman-Ford). Slow, obviously correct.
int referenceCost(const Grid& g, Topology t, int start, int goal) {
    constexpr int kInf = INT_MAX / 2;
    if (g.terrain[static_cast<std::size_t>(start)] == kWall || g.terrain[static_cast<std::size_t>(goal)] == kWall) return -1;
    std::vector<int> d(static_cast<std::size_t>(g.width * g.height), kInf);
    d[static_cast<std::size_t>(start)] = 0;
    for (bool changed = true; changed;) {
        changed = false;
        for (int u = 0; u < g.width * g.height; ++u) {
            if (d[static_cast<std::size_t>(u)] >= kInf) continue;
            for (const int v : neighborsOf(g, t, u)) {
                const int via = d[static_cast<std::size_t>(u)] + g.terrain[static_cast<std::size_t>(v)];
                if (via < d[static_cast<std::size_t>(v)]) {
                    d[static_cast<std::size_t>(v)] = via;
                    changed = true;
                }
            }
        }
    }
    return d[static_cast<std::size_t>(goal)] >= kInf ? -1 : d[static_cast<std::size_t>(goal)];
}

void matchesReferenceOnRandomMaps() {
    std::uint32_t state = 12345U;
    const auto next = [&state](int bound) {
        state = state * 1664525U + 1013904223U;
        return static_cast<int>((state >> 16) % static_cast<std::uint32_t>(bound));
    };
    int foundCount = 0;
    int missingCount = 0;
    for (int seed = 0; seed < 60; ++seed) {
        Grid g(12, 9);
        for (int& cell : g.terrain) {
            const int roll = next(100);
            cell = roll < 22 ? kWall : roll < 32 ? kSwamp : roll < 48 ? kForest : kGrass;
        }
        const int a = next(g.width * g.height);
        const int b = next(g.width * g.height);
        g.terrain[static_cast<std::size_t>(a)] = kGrass;
        g.terrain[static_cast<std::size_t>(b)] = kGrass;
        for (const Topology t : {Topology::Hex, Topology::Square}) {
            const int want = referenceCost(g, t, a, b);
            for (const bool heuristic : {true, false}) {
                const SearchResult r = search(g, t, a, b, heuristic);
                assert(r.found == (want >= 0));
                if (r.found) {
                    assert(r.cost == want);
                    checkPath(g, t, r, a, b);
                    ++foundCount;
                } else {
                    ++missingCount;
                }
            }
        }
    }
    assert(foundCount > 100 && missingCount > 0);  // the sample really contains both kinds
}

void hexGeometry() {
    const Grid g(6, 5);
    const Layout lay = makeLayout(Topology::Hex, g, 960.0, 36.0);
    const double w = kSqrt3 * lay.size;
    // Row 0 starts at the origin, odd rows are shifted by half a cell.
    const Point c00 = centerOf(lay, g, g.id(0, 0));
    assert(near(c00.x, lay.originX + w / 2.0) && near(c00.y, lay.originY + lay.size));
    const Point c01 = centerOf(lay, g, g.id(0, 1));
    assert(near(c01.x, lay.originX + w) && near(c01.y, lay.originY + 2.5 * lay.size));
    // Neighbors in the logic are neighbors on the screen: exactly the cells at distance sqrt(3) * size.
    for (int a = 0; a < g.width * g.height; ++a) {
        const Point pa = centerOf(lay, g, a);
        int onScreen = 0;
        for (int b = 0; b < g.width * g.height; ++b) {
            const Point pb = centerOf(lay, g, b);
            if (a != b && near(std::hypot(pa.x - pb.x, pa.y - pb.y), w, 1e-6)) ++onScreen;
        }
        assert(onScreen == neighborsOf(g, Topology::Hex, a).count);
    }
    // Two neighbors share exactly one edge, which are two corners.
    const int a = g.id(2, 2);
    for (const int b : neighborsOf(g, Topology::Hex, a)) {
        int shared = 0;
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                const Point p = hexCorner(centerOf(lay, g, a), lay.size, i);
                const Point q = hexCorner(centerOf(lay, g, b), lay.size, j);
                if (std::hypot(p.x - q.x, p.y - q.y) < 1e-6) ++shared;
            }
        }
        assert(shared == 2);
    }
    // Pointy top: corner 4 is the top (cx, cy - size), corner 5 the upper right.
    const Point top = hexCorner({100.0, 100.0}, 20.0, 4);
    assert(near(top.x, 100.0) && near(top.y, 80.0));
    const Point right = hexCorner({100.0, 100.0}, 20.0, 5);
    assert(near(right.x, 100.0 + 20.0 * kSqrt3 / 2.0) && near(right.y, 90.0));
}

void picking() {
    for (const Topology t : {Topology::Hex, Topology::Square}) {
        const Grid g(26, 13);
        const Layout lay = makeLayout(t, g, 960.0, 36.0);
        for (int id = 0; id < g.width * g.height; ++id) {
            const Point c = centerOf(lay, g, id);
            assert(cellAt(lay, g, c.x, c.y) == id);
            // Slightly inside every corner is still this cell.
            const int corners = t == Topology::Hex ? 6 : 4;
            for (int i = 0; i < corners; ++i) {
                const double half = t == Topology::Hex ? 0.0 : lay.size / 2.0;
                const Point corner = t == Topology::Hex ? hexCorner(c, lay.size, i)
                                                        : Point{c.x + (i % 2 == 0 ? half : -half), c.y + (i < 2 ? half : -half)};
                const double px = c.x + (corner.x - c.x) * 0.97;
                const double py = c.y + (corner.y - c.y) * 0.97;
                assert(cellAt(lay, g, px, py) == id);
            }
        }
        // Outside the map: above, below, left, right, far away.
        const Point first = centerOf(lay, g, g.id(0, 0));
        const Point last = centerOf(lay, g, g.id(g.width - 1, g.height - 1));
        const double w = t == Topology::Hex ? kSqrt3 * lay.size : lay.size;
        assert(cellAt(lay, g, first.x - w, first.y) == -1);
        assert(cellAt(lay, g, first.x, first.y - lay.size - 1.0) == -1);
        assert(cellAt(lay, g, last.x, last.y + lay.size + 1.0) == -1);
        assert(cellAt(lay, g, last.x + w, last.y) == -1);
        assert(cellAt(lay, g, -50.0, -50.0) == -1);
        assert(cellAt(lay, g, 5000.0, 5000.0) == -1);
    }
    // The notch right of an even row (odd rows stick out half a cell further) belongs to no cell.
    const Grid g(26, 13);
    const Layout hex = makeLayout(Topology::Hex, g, 960.0, 36.0);
    const Point endOfRow0 = centerOf(hex, g, g.id(25, 0));
    assert(cellAt(hex, g, endOfRow0.x + kSqrt3 * hex.size * 0.75, endOfRow0.y) == -1);
}

void layoutFitsTheCanvas() {
    const Grid g(26, 13);
    for (const Topology t : {Topology::Hex, Topology::Square}) {
        const Layout lay = makeLayout(t, g, 960.0, 36.0);
        double minX = 1e9;
        double maxX = -1e9;
        double minY = 1e9;
        double maxY = -1e9;
        for (int id = 0; id < g.width * g.height; ++id) {
            const Point c = centerOf(lay, g, id);
            const int corners = t == Topology::Hex ? 6 : 4;
            for (int i = 0; i < corners; ++i) {
                const Point p = t == Topology::Hex ? hexCorner(c, lay.size, i)
                                                   : Point{c.x + (i % 2 == 0 ? 1 : -1) * lay.size / 2.0, c.y + (i < 2 ? 1 : -1) * lay.size / 2.0};
                minX = std::min(minX, p.x);
                maxX = std::max(maxX, p.x);
                minY = std::min(minY, p.y);
                maxY = std::max(maxY, p.y);
            }
        }
        assert(minX >= 0.0 && maxX <= 960.0);
        assert(minY >= 34.0 && maxY <= 440.0);
        assert(near(minX, 960.0 - maxX, 1e-6));  // centered
    }
}

void defaultMapShowsSomethingInteresting() {
    for (const Topology t : {Topology::Hex, Topology::Square}) {
        Grid g(kCols, kRows);
        applyDefaultMap(g);
        const int a = kDefaultStart;
        const int b = kDefaultGoal;
        assert(g.terrain[static_cast<std::size_t>(a)] != kWall && g.terrain[static_cast<std::size_t>(b)] != kWall);
        const SearchResult star = search(g, t, a, b, true);
        const SearchResult dijkstra = search(g, t, a, b, false);
        checkPath(g, t, star, a, b);
        assert(star.cost == dijkstra.cost);
        assert(star.cost > distance(g, t, a, b) + 10);  // walls force a real detour
        assert(star.expanded < dijkstra.expanded);       // and the heuristic still has something to save
        assert(std::count(g.terrain.begin(), g.terrain.end(), kForest) > 0);
        assert(std::count(g.terrain.begin(), g.terrain.end(), kSwamp) > 0);
        assert(std::count(g.terrain.begin(), g.terrain.end(), kWall) > 0);
    }
}

}  // namespace

int main() {
    hexDistances();
    offsetConversion();
    hexNeighborSets();
    hexNeighborsAreSymmetric();
    squareNeighborSets();
    distancesInTheGrid();
    openFieldAstar();
    heuristicSavesWork();
    hexWallDetour();
    terrainCosts();
    unreachableAndDegenerate();
    matchesReferenceOnRandomMaps();
    hexGeometry();
    picking();
    layoutFitsTheCanvas();
    defaultMapShowsSomethingInteresting();
    std::puts("hex-astar logic: all tests passed");
    return 0;
}
