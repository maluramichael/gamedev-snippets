// Tests for the flow field logic. Plain asserts on hand-computed cases, no framework, no SDL.
// Grids are written as rows of text: '#' is a wall, '.' is open. x grows to the right, y grows downwards.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "logic.hpp"

using namespace ff;

namespace {

bool near(double a, double b, double eps = 1e-9) { return std::fabs(a - b) < eps; }

Grid grid(const std::vector<std::string>& rows) { return gridFromRows(rows); }

// An open w x h grid.
Grid open(int w, int h) { return Grid(w, h); }

// ---- BFS distance map ------------------------------------------------------------------------------------------

void bfsCorridor() {
    const Grid g = grid({"....."});
    const Field f = bfs(g, {4, 0});
    const int expected[5] = {4, 3, 2, 1, 0};
    for (int x = 0; x < 5; ++x) assert(f.at(x, 0) == expected[x]);
    assert(f.expanded == 5);  // every cell is taken from the queue once
    assert(f.maxDist == 4);
}

void bfsAroundAWall() {
    //  . # .
    //  . # .
    //  . . .
    const Grid g = grid({".#.", ".#.", "..."});
    const Field f = bfs(g, {2, 0});
    assert(f.at(2, 0) == 0);
    assert(f.at(2, 1) == 1);
    assert(f.at(2, 2) == 2);
    assert(f.at(1, 2) == 3);
    assert(f.at(0, 2) == 4);
    assert(f.at(0, 1) == 5);
    assert(f.at(0, 0) == 6);
    assert(f.at(1, 0) == kUnreached);  // walls are never reached
    assert(f.at(1, 1) == kUnreached);
    assert(f.expanded == 7);
    assert(f.maxDist == 6);
}

void bfsSealedPocketAndBounds() {
    //  . # .
    //  # # .
    //  . . .   the cell (0,0) is cut off from the goal at (2,2)
    const Grid g = grid({".#.", "##.", "..."});
    const Field f = bfs(g, {2, 2});
    assert(f.at(0, 0) == kUnreached);
    assert(f.at(2, 0) == 2);
    assert(f.expanded == 5);  // (2,2) (1,2) (0,2) (2,1) (2,0), the cut off corner is never queued
}

void bfsWallGoalAndOutOfRange() {
    const Grid g = grid({".#."});
    const Field blockedGoal = bfs(g, {1, 0});
    assert(blockedGoal.expanded == 0);
    assert(blockedGoal.at(0, 0) == kUnreached);
    const Field outside = bfs(g, {7, 7});
    assert(outside.expanded == 0);
    const Field f = bfs(g, {0, 0});
    assert(f.at(-1, 0) == kUnreached);  // outside the grid
    assert(f.at(3, 0) == kUnreached);
    assert(f.at(0, 5) == kUnreached);
}

// ---- Flow direction: the negative gradient of the distance ------------------------------------------------------

void flowInACorridor() {
    const Grid g = grid({"....."});
    const Field f = bfs(g, {4, 0});
    // x = 2: left neighbour 3, right neighbour 1 -> gx = 3 - 1 > 0, points to +x. The border above and below counts as
    // "higher" (d + 1) on both sides and cancels out.
    const Vec2 d = flowDirection(g, f, 2, 0);
    assert(near(d.x, 1.0) && near(d.y, 0.0));
    const Vec2 first = flowDirection(g, f, 0, 0);  // left neighbour is outside the grid (counts as d + 1)
    assert(near(first.x, 1.0) && near(first.y, 0.0));
    const Vec2 atGoal = flowDirection(g, f, 4, 0);  // the goal has no direction
    assert(near(atGoal.x, 0.0) && near(atGoal.y, 0.0));
}

void flowAroundAWall() {
    const Grid g = grid({".#.", ".#.", "..."});
    const Field f = bfs(g, {2, 0});
    // (0,0) d=6: only the cell below (d=5) is lower, the wall on the right counts as d+1 = 7.
    const Vec2 a = flowDirection(g, f, 0, 0);
    assert(near(a.x, 0.0) && near(a.y, 1.0));
    // (0,2) d=4: the cell on the right (d=3) is lower, the one above (d=5) is higher, the border below counts as 5.
    const Vec2 b = flowDirection(g, f, 0, 2);
    assert(near(b.x, 1.0) && near(b.y, 0.0));
    // (1,2) d=3: left is 4, right is 2 -> +x; up is a wall (4), down is the border (4) -> no y part.
    const Vec2 c = flowDirection(g, f, 1, 2);
    assert(near(c.x, 1.0) && near(c.y, 0.0));
    // A wall cell and an unreached cell have no direction.
    const Vec2 w = flowDirection(g, f, 1, 0);
    assert(near(w.x, 0.0) && near(w.y, 0.0));
}

void flowDiagonal() {
    // In an open 3x3 field with the goal in the corner (2,2), the middle cell has d=2, left/up are 3, right/down are 1.
    const Grid g = open(3, 3);
    const Field f = bfs(g, {2, 2});
    const Vec2 d = flowDirection(g, f, 1, 1);
    assert(near(d.x, std::sqrt(0.5)) && near(d.y, std::sqrt(0.5)));  // gx = 3 - 1 = 2, gy = 2, normalized
}

void flowNeverPointsIntoAWall() {
    // Per axis the wall counts as d + 1 and neighbours differ by at most 1, so no component points into a wall.
    const Grid g = gridFromRows(presetRows());
    const Field f = bfs(g, presetGoal());
    for (int y = 0; y < g.h; ++y) {
        for (int x = 0; x < g.w; ++x) {
            const Vec2 d = flowDirection(g, f, x, y);
            if (d.x < -1e-9) assert(!g.blocked(x - 1, y));
            if (d.x > 1e-9) assert(!g.blocked(x + 1, y));
            if (d.y < -1e-9) assert(!g.blocked(x, y - 1));
            if (d.y > 1e-9) assert(!g.blocked(x, y + 1));
        }
    }
}

void flowRidgeFallsBackToOneNeighbour() {
    // A ring around a wall block. The goal is at the top middle, so the bottom middle cell (2,2) is exactly as far
    // along the left route as along the right route: d=5 on both sides, and the plain gradient cancels to zero.
    //  . . . . .
    //  . # # # .
    //  . . . . .
    const Grid g = grid({".....", ".###.", "....."});
    const Field f = bfs(g, {2, 0});
    assert(f.at(1, 2) == 5 && f.at(3, 2) == 5 && f.at(2, 2) == 6);
    const Vec2 d = flowDirection(g, f, 2, 2);
    assert(near(d.x, -1.0) && near(d.y, 0.0));  // ties go to the first neighbour in the order left, right, up, down
}

void directionsMatchTheSingleLookup() {
    const Grid g = grid({".#.", ".#.", "..."});
    const Field f = bfs(g, {2, 0});
    const std::vector<Vec2> dirs = buildDirections(g, f);
    assert(dirs.size() == 9);
    for (int y = 0; y < 3; ++y) {
        for (int x = 0; x < 3; ++x) {
            const Vec2 want = flowDirection(g, f, x, y);
            const Vec2 got = dirs[static_cast<std::size_t>(y * 3 + x)];
            assert(near(got.x, want.x) && near(got.y, want.y));
        }
    }
}

// ---- Walls and agents -------------------------------------------------------------------------------------------

void pushOutOfAWall() {
    Grid g = open(5, 5);
    g.wall[g.index(2, 2)] = 1;
    // Left of the wall cell: the circle (r = 0.3) reaches 0.2 into it, so it moves 0.2 to the left.
    Vec2 p = pushOutOfWalls(g, {1.9, 2.5}, 0.3);
    assert(near(p.x, 1.7) && near(p.y, 2.5));
    // Diagonal to the corner (2,2): distance 0.1414, pushed along the diagonal until it is 0.3 away.
    p = pushOutOfWalls(g, {1.9, 1.9}, 0.3);
    assert(near(p.x, 2.0 - 0.3 * std::sqrt(0.5), 1e-9) && near(p.y, p.x, 1e-12));
    // Free space is left alone, and exactly touching is not an overlap.
    p = pushOutOfWalls(g, {1.0, 1.0}, 0.3);
    assert(near(p.x, 1.0) && near(p.y, 1.0));
    p = pushOutOfWalls(g, {1.7, 2.5}, 0.3);
    assert(near(p.x, 1.7) && near(p.y, 2.5));
    // A centre inside the wall leaves through the nearest face (a tie goes to the left face).
    p = pushOutOfWalls(g, {2.5, 2.5}, 0.3);
    assert(near(p.x, 1.7) && near(p.y, 2.5));
    p = pushOutOfWalls(g, {2.5, 2.9}, 0.3);  // 0.1 from the bottom face
    assert(near(p.x, 2.5) && near(p.y, 3.3));
}

void gridBorderIsAWall() {
    const Grid g = open(5, 5);
    const Vec2 p = pushOutOfWalls(g, {0.1, 2.5}, 0.3);
    assert(near(p.x, 0.3) && near(p.y, 2.5));
    const Vec2 q = pushOutOfWalls(g, {4.95, 4.95}, 0.3);
    assert(near(q.x, 4.7) && near(q.y, 4.7));
    const Vec2 far = pushOutOfWalls(g, {-40.0, 99.0}, 0.3);  // far outside: clamped into the grid first
    assert(far.x >= 0.3 - 1e-9 && far.y <= 4.7 + 1e-9);
}

void paintingWalls() {
    Grid g = open(4, 4);
    const Cell goal{3, 3};
    assert(setWall(g, {1, 1}, true, goal));
    assert(g.blocked(1, 1));
    assert(!setWall(g, {1, 1}, true, goal));  // already a wall, nothing changed
    assert(setWall(g, {1, 1}, false, goal));
    assert(!g.blocked(1, 1));
    assert(!setWall(g, goal, true, goal));  // the goal cell stays open
    assert(!g.blocked(3, 3));
    assert(!setWall(g, {-1, 0}, true, goal));  // outside the grid
    assert(!setWall(g, {0, 4}, true, goal));
}

// ---- A* ---------------------------------------------------------------------------------------------------------

void astarStraightLine() {
    const Grid g = open(9, 9);
    const Path p = astar(g, {0, 4}, {8, 4});
    assert(p.found);
    assert(p.cells.size() == 9);
    assert(p.cells.front() == (Cell{0, 4}) && p.cells.back() == (Cell{8, 4}));
    // The heuristic pulls the search straight along the row: only the 9 cells of the path are ever expanded.
    // A flood from the goal (what the flow field does) touches all 81.
    assert(p.expanded == 9);
    assert(bfs(g, {8, 4}).expanded == 81);
}

void astarAroundAWall() {
    const Grid g = grid({".#.", ".#.", "..."});
    const Path p = astar(g, {0, 0}, {2, 0});
    assert(p.found);
    assert(p.cells.size() == 7);  // 6 steps, the same number the BFS gives for (0,0)
    const std::vector<Cell> want{{0, 0}, {0, 1}, {0, 2}, {1, 2}, {2, 2}, {2, 1}, {2, 0}};
    assert(p.cells == want);
    assert(p.expanded == 7);
}

void astarStartIsGoal() {
    const Grid g = open(3, 3);
    const Path p = astar(g, {1, 1}, {1, 1});
    assert(p.found && p.cells.size() == 1 && p.expanded == 1);
}

void astarNoPathFloodsTheRegion() {
    //  . # .
    //  . # .
    //  . # .   the left column is cut off from the right one
    const Grid g = grid({".#.", ".#.", ".#."});
    const Path p = astar(g, {0, 0}, {2, 0});
    assert(!p.found);
    assert(p.cells.empty());
    assert(p.expanded == 3);  // it had to search the whole left column before giving up
}

void astarRejectsBadEnds() {
    const Grid g = grid({".#."});
    assert(!astar(g, {1, 0}, {2, 0}).found);   // start in a wall
    assert(!astar(g, {0, 0}, {1, 0}).found);   // goal in a wall
    assert(!astar(g, {-1, 0}, {0, 0}).found);  // outside
    assert(astar(g, {1, 0}, {2, 0}).expanded == 0);
}

void astarMatchesBfsOnThePreset() {
    const Grid g = gridFromRows(presetRows());
    const Cell goal = presetGoal();
    const Field f = bfs(g, goal);
    int checked = 0;
    for (int y = 0; y < g.h; y += 3) {
        for (int x = 0; x < g.w; x += 5) {
            if (g.blocked(x, y)) continue;
            const Path p = astar(g, {x, y}, goal);
            assert(p.found);
            assert(static_cast<int>(p.cells.size()) - 1 == f.at(x, y));  // both find the shortest route
            for (std::size_t i = 1; i < p.cells.size(); ++i) {  // and every step is to an open neighbour
                assert(!g.blocked(p.cells[i].x, p.cells[i].y));
                assert(std::abs(p.cells[i].x - p.cells[i - 1].x) + std::abs(p.cells[i].y - p.cells[i - 1].y) == 1);
            }
            ++checked;
        }
    }
    assert(checked > 40);
}

void astarWorkAddsUpPerAgent() {
    const Grid g = grid({"....."});
    const std::vector<Cell> starts{{0, 0}, {0, 0}, {2, 0}};
    const AstarWork w = astarWork(g, {4, 0}, starts);
    assert(w.searches == 3);
    assert(w.expanded == 5 + 5 + 3);  // from x=0 all five cells, from x=2 the last three; two agents on one cell count twice
    const Grid walled = grid({".#..."});
    const AstarWork v = astarWork(walled, {4, 0}, {{1, 0}, {2, 0}});  // the first agent sits inside a wall
    assert(v.searches == 1);
    assert(v.expanded == 3);
    const AstarWork none = astarWork(g, {4, 0}, {});
    assert(none.searches == 0 && none.expanded == 0);
}

// ---- Following the field ------------------------------------------------------------------------------------------

// Walks an agent along the field with the same step the demo uses: dt = 1/60, speed 9 cells per second, radius 0.28.
// Returns the number of steps until it is inside the goal cell, or -1.
int stepsToGoal(const Grid& g, const std::vector<Vec2>& dirs, Cell start, Cell goal, int limit) {
    Vec2 pos{start.x + 0.5, start.y + 0.5};
    for (int i = 0; i < limit; ++i) {
        if (cellOf(pos) == goal) return i;
        const Cell c = cellOf(pos);
        const Vec2 d = dirs[static_cast<std::size_t>(c.y * g.w + c.x)];
        pos.x += d.x * 9.0 / 60.0;
        pos.y += d.y * 9.0 / 60.0;
        pos = pushOutOfWalls(g, pos, 0.28);
    }
    return -1;
}

void agentsFollowTheFieldToTheGoal() {
    const Grid g = gridFromRows(presetRows());
    const Cell goal = presetGoal();
    const Field f = bfs(g, goal);
    const std::vector<Vec2> dirs = buildDirections(g, f);
    int checked = 0;
    for (int y = 0; y < g.h; y += 2) {
        for (int x = 0; x < g.w; x += 3) {
            if (g.blocked(x, y)) continue;
            // Walking at 9 cells/s covers 0.15 cells per step, the longest route is 69 cells, so 3000 steps (450 cells) are plenty.
            const int steps = stepsToGoal(g, dirs, {x, y}, goal, 3000);
            assert(steps >= 0);
            // It can never be faster than the straight distance allows.
            const double straight = std::hypot(x - goal.x, y - goal.y);
            assert(steps >= static_cast<int>((straight - 0.71) / 0.15) - 2);
            ++checked;
        }
    }
    assert(checked > 100);
}

void agentOnTheRidgeStillMoves() {
    const Grid g = grid({".....", ".###.", "....."});
    const Field f = bfs(g, {2, 0});
    const std::vector<Vec2> dirs = buildDirections(g, f);
    // Exactly on the cell centre of the tie cell: the fallback gives it a direction, it must not stand still.
    assert(stepsToGoal(g, dirs, {2, 2}, {2, 0}, 3000) > 0);
}

// ---- Spawning, random numbers, numbers on screen -----------------------------------------------------------------------

void rngIsDeterministicAndInRange() {
    Rng a(42);
    Rng b(42);
    for (int i = 0; i < 100; ++i) assert(a.next() == b.next());
    Rng zero(0);  // a zero seed would make xorshift return 0 forever
    assert(zero.next() != 0);
    Rng r(7);
    bool seen[5] = {false, false, false, false, false};
    for (int i = 0; i < 1000; ++i) {
        const int v = r.below(5);
        assert(v >= 0 && v < 5);
        seen[v] = true;
        const double u = r.uniform();
        assert(u >= 0.0 && u < 1.0);
    }
    for (bool s : seen) assert(s);
}

void spawnPicksFarReachableCells() {
    const Grid g = grid({"...........", ".#########."});
    const Field f = bfs(g, {0, 0});
    Rng rng(3);
    for (int i = 0; i < 200; ++i) {
        const auto c = pickSpawn(f, 6, rng);
        assert(c.has_value());
        assert(!g.blocked(c->x, c->y));
        assert(f.at(c->x, c->y) >= 6);
    }
    // Asking for more than the field has falls back to the farther half, never to the goal or a wall.
    for (int i = 0; i < 200; ++i) {
        const auto c = pickSpawn(f, 1000, rng);
        assert(c.has_value());
        assert(f.at(c->x, c->y) >= (f.maxDist + 1) / 2);
        assert(f.at(c->x, c->y) > 0);
    }
}

void spawnHasNoCandidatesWithoutAField() {
    const Grid sealed = grid({".#.", "###", "..."});
    // Goal in the middle of a sealed single cell: nothing but the goal is reachable.
    const Field f = bfs(sealed, {0, 0});
    Rng rng(1);
    assert(!pickSpawn(f, 0, rng).has_value());
    const Field none = bfs(sealed, {1, 0});  // goal on a wall: empty field
    assert(!pickSpawn(none, 0, rng).has_value());
}

void thousandsSeparators() {
    assert(grouped(0) == "0");
    assert(grouped(7) == "7");
    assert(grouped(999) == "999");
    assert(grouped(1000) == "1,000");
    assert(grouped(41230) == "41,230");
    assert(grouped(100000) == "100,000");
    assert(grouped(1234567) == "1,234,567");
}

void cellOfPoints() {
    assert(cellOf({0.0, 0.0}) == (Cell{0, 0}));
    assert(cellOf({3.99, 1.01}) == (Cell{3, 1}));
    assert(cellOf({4.0, 2.0}) == (Cell{4, 2}));
    assert(cellOf({-0.01, -0.01}) == (Cell{-1, -1}));  // floor, not truncation towards zero
}

// ---- The map the demo starts with ----------------------------------------------------------------------------------

void presetIsWellFormed() {
    const std::array<std::string_view, 16> rows = presetRows();
    for (const std::string_view r : rows) assert(r.size() == 48);
    const Grid g = gridFromRows(rows);
    assert(g.w == 48 && g.h == 16);
    const Cell goal = presetGoal();
    assert(!g.blocked(goal.x, goal.y));
    // Every open cell can reach the goal: no sealed pockets in the start map.
    const Field f = bfs(g, goal);
    int open = 0;
    for (int y = 0; y < g.h; ++y) {
        for (int x = 0; x < g.w; ++x) {
            if (g.blocked(x, y)) continue;
            ++open;
            assert(f.at(x, y) != kUnreached);
        }
    }
    assert(f.expanded == open);
    assert(f.maxDist == 69);  // from the top left corner: down to the first gap, up to the second one, then to the goal
}

}  // namespace

int main() {
    bfsCorridor();
    bfsAroundAWall();
    bfsSealedPocketAndBounds();
    bfsWallGoalAndOutOfRange();
    flowInACorridor();
    flowAroundAWall();
    flowDiagonal();
    flowNeverPointsIntoAWall();
    flowRidgeFallsBackToOneNeighbour();
    directionsMatchTheSingleLookup();
    pushOutOfAWall();
    gridBorderIsAWall();
    paintingWalls();
    astarStraightLine();
    astarAroundAWall();
    astarStartIsGoal();
    astarNoPathFloodsTheRegion();
    astarRejectsBadEnds();
    astarMatchesBfsOnThePreset();
    astarWorkAddsUpPerAgent();
    agentsFollowTheFieldToTheGoal();
    agentOnTheRidgeStillMoves();
    rngIsDeterministicAndInRange();
    spawnPicksFarReachableCells();
    spawnHasNoCandidatesWithoutAField();
    thousandsSeparators();
    cellOfPoints();
    presetIsWellFormed();
    std::puts("flow-field logic: all tests passed");
    return 0;
}
