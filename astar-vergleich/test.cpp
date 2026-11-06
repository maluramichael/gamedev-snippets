// Tests for the grid pathfinding logic. Plain asserts on hand-computed cases, no framework, no SDL.
// Grids are described row by row in the helpers below: '.' grass (cost 1), '~' mud (cost 5), '#' wall.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "logic.hpp"

using namespace pf;

namespace {

bool near(double a, double b) { return std::fabs(a - b) < 1e-9; }

Grid make(const std::vector<std::string>& rows) {
    Grid g(static_cast<int>(rows[0].size()), static_cast<int>(rows.size()));
    for (int y = 0; y < g.height(); ++y) {
        for (int x = 0; x < g.width(); ++x) {
            const char c = rows[static_cast<std::size_t>(y)][static_cast<std::size_t>(x)];
            g.set(x, y, c == '#' ? kWall : c == '~' ? kMud : kGrass);
        }
    }
    return g;
}

Search run(const Grid& g, Cell start, Cell goal, Algo algo, bool diagonals = false, double weight = 1.0) {
    Search s(g, start, goal, Options{algo, diagonals, weight});
    s.runToEnd();
    return s;
}

void heuristics() {
    assert(near(heuristic({0, 0}, {3, 4}, false), 7.0));  // manhattan: 3 + 4
    assert(near(heuristic({3, 4}, {0, 0}, false), 7.0));  // symmetric
    assert(near(heuristic({0, 0}, {3, 4}, true), 1.0 + 3.0 * kSqrt2));  // octile: 3 diagonals and 1 straight
    assert(near(heuristic({2, 2}, {2, 2}, true), 0.0));
    assert(near(heuristic({0, 0}, {5, 0}, true), 5.0));  // on a row diagonals do not help
}

void gridBasics() {
    Grid g(4, 3);
    assert(g.width() == 4 && g.height() == 3);
    assert(g.passable(0, 0) && g.passable(3, 2));
    assert(!g.passable(-1, 0) && !g.passable(4, 0) && !g.passable(0, 3));  // outside is a wall
    g.set(1, 1, kWall);
    assert(!g.passable(1, 1));
    g.set(9, 9, kWall);  // out of range edits are ignored
    g.set(-1, 0, kMud);
    assert(g.at(-1, 0) == kWall);
    g.set(2, 2, kMud);
    assert(g.at(2, 2) == kMud && g.passable(2, 2));
    g.clear();
    assert(g.at(1, 1) == kGrass && g.at(2, 2) == kGrass);
}

void pickCellBounds() {
    // Origin (20, 34), 20 px cells, 46 x 19 cells.
    const auto a = pickCell(20.0, 34.0, 20.0, 34.0, 20.0, 46, 19);
    assert(a && a->x == 0 && a->y == 0);  // the top left corner belongs to cell 0
    const auto b = pickCell(39.99, 53.99, 20.0, 34.0, 20.0, 46, 19);
    assert(b && b->x == 0 && b->y == 0);
    const auto c = pickCell(40.0, 54.0, 20.0, 34.0, 20.0, 46, 19);
    assert(c && c->x == 1 && c->y == 1);  // the edge belongs to the next cell
    const auto d = pickCell(939.99, 413.99, 20.0, 34.0, 20.0, 46, 19);
    assert(d && d->x == 45 && d->y == 18);
    assert(!pickCell(940.0, 100.0, 20.0, 34.0, 20.0, 46, 19));  // one past the last column
    assert(!pickCell(100.0, 414.0, 20.0, 34.0, 20.0, 46, 19));  // one past the last row
    assert(!pickCell(19.9, 100.0, 20.0, 34.0, 20.0, 46, 19));   // left of the grid: truncation would give 0, floor gives -1
    assert(!pickCell(100.0, 33.5, 20.0, 34.0, 20.0, 46, 19));
    assert(!pickCell(1e30, 1e30, 20.0, 34.0, 20.0, 46, 19));    // far outside, must not overflow int
}

void lineOfCells() {
    // Painting follows the mouse in jumps; the cells in between come from a line.
    const auto flat = lineCells({0, 0}, {3, 0});
    assert(flat.size() == 4);
    assert(flat[0] == Cell({0, 0}) && flat[1] == Cell({1, 0}) && flat[3] == Cell({3, 0}));
    const auto diag = lineCells({0, 0}, {2, 2});
    assert(diag.size() == 3 && diag[1] == Cell({1, 1}));
    const auto back = lineCells({3, 1}, {0, 1});  // direction does not matter for the endpoints
    assert(back.size() == 4 && back.front() == Cell({3, 1}) && back.back() == Cell({0, 1}));
    assert(lineCells({5, 7}, {5, 7}).size() == 1);
    // A shallow line: 7 cells wide, 2 high. Every step moves by one cell in x and at most one in y, none is skipped.
    const auto shallow = lineCells({0, 0}, {6, 2});
    assert(shallow.size() == 7);
    assert(shallow.front() == Cell({0, 0}) && shallow.back() == Cell({6, 2}));
    for (std::size_t i = 1; i < shallow.size(); ++i) {
        assert(shallow[i].x == shallow[i - 1].x + 1);
        assert(shallow[i].y - shallow[i - 1].y == 0 || shallow[i].y - shallow[i - 1].y == 1);
    }
    // Steep and negative directions.
    const auto steep = lineCells({1, 5}, {0, 0});
    assert(steep.size() == 6 && steep.front() == Cell({1, 5}) && steep.back() == Cell({0, 0}));
}

void weightSlider() {
    assert(near(weightFromT(0.0), 0.0));
    assert(near(weightFromT(1.0), kMaxWeight));
    assert(near(weightFromT(0.1), 1.0));  // 10 * 0.1
    assert(near(weightFromT(0.12), 1.0));  // snaps to steps of 0.5
    assert(near(weightFromT(0.13), 1.5));
    assert(near(weightFromT(-3.0), 0.0));  // clamped
    assert(near(weightFromT(7.0), kMaxWeight));
    assert(near(weightFromT(std::nan("")), 0.0));
    assert(near(tFromWeight(1.0), 0.1));
    assert(near(tFromWeight(kMaxWeight), 1.0));
    assert(near(snapWeight(0.74), 0.5) && near(snapWeight(0.76), 1.0));
    assert(near(snapWeight(-1.0), 0.0) && near(snapWeight(99.0), kMaxWeight));
}

void corridor() {
    // One row, five cells: everything has to be expanded in order, the goal last.
    const Grid g = make({"....."});
    for (const Algo a : {Algo::Bfs, Algo::Dijkstra, Algo::Greedy, Algo::AStar}) {
        const Search s = run(g, {0, 0}, {4, 0}, a);
        assert(s.status() == Status::Found);
        assert(s.expanded() == 5);
        assert(s.path().size() == 5);
        assert(s.path().front().x == 0 && s.path().back().x == 4);
        assert(near(pathCost(g, s.path()), 4.0));
    }
}

void openFieldCounts() {
    // 5 x 5, corner to corner, no diagonals. The cost to a cell is x + y, so Dijkstra and BFS reach the goal (cost 8)
    // last of all 25 cells. A* with a perfect heuristic walks straight to the goal: nine cells. Greedy as well.
    const Grid g = make({".....", ".....", ".....", ".....", "....."});
    assert(run(g, {0, 0}, {4, 4}, Algo::Bfs).expanded() == 25);
    assert(run(g, {0, 0}, {4, 4}, Algo::Dijkstra).expanded() == 25);
    const Search a = run(g, {0, 0}, {4, 4}, Algo::AStar);
    assert(a.expanded() == 9);
    assert(a.path().size() == 9);
    assert(near(pathCost(g, a.path()), 8.0));
    assert(run(g, {0, 0}, {4, 4}, Algo::Greedy).expanded() == 9);
}

void frontierCounts() {
    const Grid g = make({".....", ".....", ".....", ".....", "....."});
    Search s(g, {0, 0}, {4, 4}, Options{Algo::Dijkstra, false, 1.0});
    assert(s.status() == Status::Running);
    assert(s.expanded() == 0 && s.frontier() == 1);  // the start is waiting
    assert(s.step());
    assert(s.expanded() == 1);
    assert(s.frontier() == 2);  // (1,0) and (0,1)
    assert(s.stateAt(0, 0) == CellState::Closed);
    assert(s.stateAt(1, 0) == CellState::Open && s.stateAt(0, 1) == CellState::Open);
    assert(s.stateAt(2, 0) == CellState::Unseen);
    assert(s.lastExpanded() && s.lastExpanded()->x == 0 && s.lastExpanded()->y == 0);
}

void blockedGoal() {
    // A wall column splits 5 x 3 in two halves of 2 x 3. The search floods its half and then gives up.
    const Grid g = make({"..#..", "..#..", "..#.."});
    for (const Algo a : {Algo::Bfs, Algo::Dijkstra, Algo::Greedy, Algo::AStar}) {
        const Search s = run(g, {0, 1}, {4, 1}, a);
        assert(s.status() == Status::NoPath);
        assert(s.expanded() == 6);
        assert(s.frontier() == 0);
        assert(s.path().empty());
    }
    // Finished searches do not step any more.
    Search s(g, {0, 1}, {4, 1}, Options{Algo::AStar, false, 1.0});
    s.runToEnd();
    assert(!s.step());
}

void startAndGoalProblems() {
    const Grid g = make({"...", ".#.", "..."});
    assert(run(g, {1, 1}, {2, 2}, Algo::AStar).status() == Status::NoPath);  // start in a wall
    assert(run(g, {0, 0}, {1, 1}, Algo::AStar).status() == Status::NoPath);  // goal in a wall
    assert(run(g, {0, 0}, {1, 1}, Algo::AStar).expanded() == 0);
    assert(run(g, {-1, 0}, {2, 2}, Algo::AStar).status() == Status::NoPath);  // start outside
    const Search same = run(g, {0, 0}, {0, 0}, Algo::AStar);
    assert(same.status() == Status::Found);
    assert(same.path().size() == 1);
    assert(near(pathCost(g, same.path()), 0.0));
}

void terrainChangesTheAnswer() {
    // Row 1 is mud between start (0,1) and goal (4,1). Straight through: 5 + 5 + 5 + 1 = 16.
    // Around over row 0: up, four to the right, down = 6 steps of grass = 6.
    const Grid g = make({".....", ".~~~.", "....."});
    const Search dij = run(g, {0, 1}, {4, 1}, Algo::Dijkstra);
    assert(near(pathCost(g, dij.path()), 6.0));
    const Search astar = run(g, {0, 1}, {4, 1}, Algo::AStar, false, 1.0);
    assert(near(pathCost(g, astar.path()), 6.0));
    // BFS counts steps: 4 steps through the mud beat 6 steps around it.
    const Search bfs = run(g, {0, 1}, {4, 1}, Algo::Bfs);
    assert(bfs.path().size() == 5);
    assert(near(pathCost(g, bfs.path()), 16.0));
    // Greedy only looks at the distance to the goal and walks straight into the mud.
    const Search greedy = run(g, {0, 1}, {4, 1}, Algo::Greedy);
    assert(near(pathCost(g, greedy.path()), 16.0));
    assert(greedy.expanded() == 5);
}

void overestimatingHeuristic() {
    // Same grid. Weight 10 makes the heuristic say "the goal is ten times as far as it is", A* turns greedy
    // and takes the mud. Hand check, f = g + 10 h: (1,1) 5+30=35 beats (0,0) 1+50=51, then (2,1) 10+20=30 beats
    // (1,0) 6+40=46, then (3,1) 15+10=25, then the goal with 16.
    const Grid g = make({".....", ".~~~.", "....."});
    const Search heavy = run(g, {0, 1}, {4, 1}, Algo::AStar, false, 10.0);
    assert(heavy.status() == Status::Found);
    assert(near(pathCost(g, heavy.path()), 16.0));
    assert(heavy.expanded() == 5);
    const Search exact = run(g, {0, 1}, {4, 1}, Algo::AStar, false, 1.0);
    assert(near(pathCost(g, exact.path()), 6.0));
}

void weightZeroIsDijkstra() {
    const Grid g = make({"........", ".~~~~~#.", "...#.~#.", ".#...~..", "...~~~.."});
    Search a(g, {0, 2}, {7, 4}, Options{Algo::AStar, false, 0.0});
    Search d(g, {0, 2}, {7, 4}, Options{Algo::Dijkstra, false, 7.0});  // the weight is ignored by Dijkstra
    a.runToEnd();
    d.runToEnd();
    assert(a.status() == Status::Found && d.status() == Status::Found);
    assert(a.order() == d.order());
    assert(a.path() == d.path());
}

void astarIsOptimalAndCheaper() {
    // On a map with walls and mud A* with weight 1 finds the same cost as Dijkstra, in no more expansions.
    const Grid g = make({"..........", ".######...", ".#....#.~.", ".#.##.#.~.", ".#.#..#.~.", "...#.....~"});
    for (const bool diag : {false, true}) {
        const Search dij = run(g, {0, 0}, {9, 5}, Algo::Dijkstra, diag);
        const Search ast = run(g, {0, 0}, {9, 5}, Algo::AStar, diag, 1.0);
        assert(dij.status() == Status::Found && ast.status() == Status::Found);
        assert(near(pathCost(g, dij.path()), pathCost(g, ast.path())));
        assert(ast.expanded() <= dij.expanded());
    }
}

void diagonals() {
    const Grid g = make({".....", ".....", ".....", ".....", "....."});
    const Search d = run(g, {0, 0}, {4, 4}, Algo::AStar, true);
    assert(d.path().size() == 5);
    assert(d.expanded() == 5);
    assert(near(pathCost(g, d.path()), 4.0 * kSqrt2));
    // Without diagonals the same trip costs 8.
    assert(near(pathCost(g, run(g, {0, 0}, {4, 4}, Algo::AStar, false).path()), 8.0));
    // BFS with diagonals counts steps: 4 diagonal steps.
    assert(run(g, {0, 0}, {4, 4}, Algo::Bfs, true).path().size() == 5);
}

void noCornerCutting() {
    // The diagonal from (0,0) to (1,1) squeezes between the wall (1,0) and the free cell (0,1): not allowed.
    const Grid g = make({".#", ".."});
    assert(!canStep(g, {0, 0}, 1, 1));
    assert(canStep(g, {0, 0}, 0, 1));
    assert(!canStep(g, {0, 0}, 1, 0));  // into the wall
    const Search s = run(g, {0, 0}, {1, 1}, Algo::AStar, true);
    assert(s.status() == Status::Found);
    assert(s.path().size() == 3);  // down, then right
    assert(near(pathCost(g, s.path()), 2.0));
    // Without the wall the diagonal is fine.
    const Grid open = make({"..", ".."});
    assert(canStep(open, {0, 0}, 1, 1));
    assert(near(pathCost(open, run(open, {0, 0}, {1, 1}, Algo::AStar, true).path()), kSqrt2));
    // Out of the grid is never a step.
    assert(!canStep(open, {0, 0}, -1, 0));
    assert(!canStep(open, {1, 1}, 1, 1));
}

void diagonalThroughMud() {
    // A diagonal step costs sqrt(2) times the terrain of the cell it enters.
    const Grid g = make({"..", ".~"});
    const std::vector<Cell> p{{0, 0}, {1, 1}};
    assert(near(pathCost(g, p), 5.0 * kSqrt2));
    // Two straight steps end in the same mud cell: 1 for the grass cell, 5 for the mud.
    assert(near(pathCost(g, {{0, 0}, {1, 0}, {1, 1}}), 1.0 + 5.0));
}

void pathCostEdgeCases() {
    const Grid g = make({"..."});
    assert(near(pathCost(g, {}), 0.0));
    assert(near(pathCost(g, {{1, 0}}), 0.0));
}

void preset() {
    const Preset p = presetMap();
    assert(p.grid.passable(p.start.x, p.start.y) && p.grid.passable(p.goal.x, p.goal.y));
    const Search dij = run(p.grid, p.start, p.goal, Algo::Dijkstra);
    const Search ast = run(p.grid, p.start, p.goal, Algo::AStar);
    const Search greedy = run(p.grid, p.start, p.goal, Algo::Greedy);
    assert(dij.status() == Status::Found && ast.status() == Status::Found && greedy.status() == Status::Found);
    assert(near(pathCost(p.grid, dij.path()), pathCost(p.grid, ast.path())));
    assert(ast.expanded() < dij.expanded());
    assert(greedy.expanded() < dij.expanded());
    assert(pathCost(p.grid, greedy.path()) > pathCost(p.grid, ast.path()) + 1e-9);  // greedy walks into the trap
    assert(run(p.grid, p.start, p.goal, Algo::Bfs).status() == Status::Found);
}

}  // namespace

int main() {
    heuristics();
    gridBasics();
    pickCellBounds();
    lineOfCells();
    weightSlider();
    corridor();
    openFieldCounts();
    frontierCounts();
    blockedGoal();
    startAndGoalProblems();
    terrainChangesTheAnswer();
    overestimatingHeuristic();
    weightZeroIsDijkstra();
    astarIsOptimalAndCheaper();
    diagonals();
    noCornerCutting();
    diagonalThroughMud();
    pathCostEdgeCases();
    preset();
    std::puts("astar-vergleich logic: all tests passed");
    return EXIT_SUCCESS;
}
