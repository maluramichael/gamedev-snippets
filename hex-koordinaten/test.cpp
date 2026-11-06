// Tests for the hex coordinate logic. Plain asserts on hand-computed cases, no framework, no SDL.
// Every offset convention is checked against the axial form, which is the one the math is done in.
#undef NDEBUG
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <queue>
#include <set>
#include <utility>

#include "logic.hpp"

using namespace hex;

namespace {

bool near(double a, double b) { return std::fabs(a - b) < 1e-9; }
bool same(Axial a, Axial b) { return a.q == b.q && a.r == b.r; }
bool same(Offset a, Offset b) { return a.col == b.col && a.row == b.row; }

constexpr Layout kLayouts[4] = {{Orientation::Pointy, Parity::Odd},
                                {Orientation::Pointy, Parity::Even},
                                {Orientation::Flat, Parity::Odd},
                                {Orientation::Flat, Parity::Even}};

void cubeKeepsTheSumAtZero() {
    // s is not stored information, it follows from q and r.
    const Cube c = toCube({2, 3});
    assert(c.q == 2 && c.r == 3 && c.s == -5);
    const Cube d = toCube({-4, 1});
    assert(d.q == -4 && d.r == 1 && d.s == 3);
    for (int q = -6; q <= 6; ++q) {
        for (int r = -6; r <= 6; ++r) {
            const Cube e = toCube({q, r});
            assert(e.q + e.r + e.s == 0);
            assert(same(toAxial(e), Axial{q, r}));
        }
    }
}

void offsetToAxialByHand() {
    // Pointy, odd rows pushed right (odd-r): q = col - (row - (row & 1)) / 2.
    assert(same(toAxial({Orientation::Pointy, Parity::Odd}, {3, 2}), Axial{2, 2}));
    assert(same(toAxial({Orientation::Pointy, Parity::Odd}, {3, 3}), Axial{2, 3}));
    assert(same(toAxial({Orientation::Pointy, Parity::Odd}, {0, -1}), Axial{1, -1}));  // negative rows keep the pattern
    // Pointy, even rows pushed right (even-r): q = col - (row + (row & 1)) / 2.
    assert(same(toAxial({Orientation::Pointy, Parity::Even}, {3, 3}), Axial{1, 3}));
    assert(same(toAxial({Orientation::Pointy, Parity::Even}, {3, 2}), Axial{2, 2}));
    // Flat, odd columns pushed down (odd-q): r = row - (col - (col & 1)) / 2.
    assert(same(toAxial({Orientation::Flat, Parity::Odd}, {3, 2}), Axial{3, 1}));
    assert(same(toAxial({Orientation::Flat, Parity::Odd}, {2, 2}), Axial{2, 1}));
    // Flat, even columns pushed down (even-q): r = row - (col + (col & 1)) / 2.
    assert(same(toAxial({Orientation::Flat, Parity::Even}, {3, 2}), Axial{3, 0}));
    assert(same(toAxial({Orientation::Flat, Parity::Even}, {-2, 1}), Axial{-2, 2}));
    assert(same(toAxial({Orientation::Flat, Parity::Odd}, {-1, 0}), Axial{-1, 1}));  // negative columns keep the pattern
}

void offsetRoundTrip() {
    for (const Layout& l : kLayouts) {
        for (int col = -8; col <= 8; ++col) {
            for (int row = -8; row <= 8; ++row) {
                const Axial a = toAxial(l, {col, row});
                assert(same(toOffset(l, a), Offset{col, row}));
            }
        }
        for (int q = -8; q <= 8; ++q) {
            for (int r = -8; r <= 8; ++r) {
                assert(same(toAxial(l, toOffset(l, {q, r})), Axial{q, r}));
            }
        }
    }
}

void doubledByHand() {
    // Pointy doubles the columns: col = 2q + r. Flat doubles the rows: row = 2r + q.
    const Doubled p = toDoubled(Orientation::Pointy, {2, 3});
    assert(p.col == 7 && p.row == 3);
    const Doubled f = toDoubled(Orientation::Flat, {2, 3});
    assert(f.col == 2 && f.row == 8);
    const Doubled n = toDoubled(Orientation::Pointy, {-3, 1});
    assert(n.col == -5 && n.row == 1);
    // Only cells where col + row is even exist.
    assert(isDoubled(Orientation::Pointy, {7, 3}));
    assert(!isDoubled(Orientation::Pointy, {6, 3}));
    assert(isDoubled(Orientation::Flat, {2, 8}));
    assert(!isDoubled(Orientation::Flat, {2, 7}));
    for (const Orientation o : {Orientation::Pointy, Orientation::Flat}) {
        for (int q = -7; q <= 7; ++q) {
            for (int r = -7; r <= 7; ++r) {
                const Doubled d = toDoubled(o, {q, r});
                assert(isDoubled(o, d));
                assert(same(fromDoubled(o, d), Axial{q, r}));
            }
        }
    }
}

void distanceByHand() {
    const Cube o{0, 0, 0};
    assert(cubeDistance(o, o) == 0);
    assert(cubeDistance(o, toCube({1, -1})) == 1);   // a neighbor
    assert(cubeDistance(o, toCube({2, 1})) == 3);    // (2, 1, -3): the largest component is 3
    assert(cubeDistance(o, toCube({1, 1})) == 2);    // (1, 1, -2): not 1, the s axis counts
    assert(cubeDistance(o, toCube({-3, 3})) == 3);   // straight along the q/r diagonal
    assert(cubeDistance(toCube({-3, 2}), toCube({2, -1})) == 5);  // d = (5, -3, -2)
    assert(cubeDistance(toCube({2, -1}), toCube({-3, 2})) == 5);  // symmetric
}

void distanceFormulasAgree() {
    // The cube form and the axial shortcut (|dq| + |dq + dr| + |dr|) / 2 give the same number.
    for (int q = -5; q <= 5; ++q) {
        for (int r = -5; r <= 5; ++r) {
            const Axial a{q, r};
            const Axial b{2 * r - 1, q + 3};
            assert(cubeDistance(toCube(a), toCube(b)) == axialDistance(a, b));
        }
    }
}

void distanceCountsSteps() {
    // Independent check: walk outward from the origin one neighbor step at a time. The number of steps it takes to reach
    // a cell is its distance, no formula involved.
    std::map<std::pair<int, int>, int> steps;
    std::queue<Axial> todo;
    steps[{0, 0}] = 0;
    todo.push({0, 0});
    while (!todo.empty()) {
        const Axial a = todo.front();
        todo.pop();
        const int s = steps[{a.q, a.r}];
        if (s >= 8) continue;
        for (int d = 0; d < 6; ++d) {
            const Axial n = neighbor(a, d);
            if (steps.count({n.q, n.r}) == 0) {
                steps[{n.q, n.r}] = s + 1;
                todo.push(n);
            }
        }
    }
    assert(steps.size() == 1 + 3 * 8 * 9);  // 1 + 3n(n+1) cells within n = 8 steps
    for (const auto& [key, s] : steps) {
        assert(cubeDistance({0, 0, 0}, toCube({key.first, key.second})) == s);
    }
}

void naiveOffsetDistanceIsWrong() {
    // Odd-r: (1, 0) is the NE neighbor of (0, 1), one step away. Counting offset steps says 2.
    const Layout l{Orientation::Pointy, Parity::Odd};
    const Offset a{0, 1};
    const Offset b{1, 0};
    assert(cubeDistance(toCube(toAxial(l, a)), toCube(toAxial(l, b))) == 1);
    assert(naiveOffsetDistance(a, b) == 2);
    // The mistake depends on the row: (0, 1) is a neighbor of (0, 0) and the naive count gets that one right.
    assert(naiveOffsetDistance({0, 0}, {0, 1}) == 1);
    // Far apart in a straight row both agree.
    assert(naiveOffsetDistance({0, 0}, {4, 0}) == 4);
    assert(naiveOffsetDistance({4, 0}, {0, 0}) == 4);
}

void neighborsByHand() {
    // Axial directions in order: (+1,0) (+1,-1) (0,-1) (-1,0) (-1,+1) (0,+1).
    const Axial c{2, 3};
    assert(same(neighbor(c, 0), Axial{3, 3}));
    assert(same(neighbor(c, 1), Axial{3, 2}));
    assert(same(neighbor(c, 2), Axial{2, 2}));
    assert(same(neighbor(c, 3), Axial{1, 3}));
    assert(same(neighbor(c, 4), Axial{1, 4}));
    assert(same(neighbor(c, 5), Axial{2, 4}));
    // Wraps around instead of reading outside the table.
    assert(same(neighbor(c, 6), neighbor(c, 0)));
    assert(same(neighbor(c, -1), neighbor(c, 5)));
    for (int d = 0; d < 6; ++d) assert(cubeDistance(toCube(c), toCube(neighbor(c, d))) == 1);
}

void offsetTablesMatchAxial() {
    // The two-table way (pick the table by row or column parity) must land on the same cell as converting to axial,
    // stepping, and converting back.
    for (const Layout& l : kLayouts) {
        for (int col = -6; col <= 6; ++col) {
            for (int row = -6; row <= 6; ++row) {
                for (int d = 0; d < 6; ++d) {
                    const Offset viaTable = neighborOffset(l, {col, row}, d);
                    const Offset viaAxial = toOffset(l, neighbor(toAxial(l, {col, row}), d));
                    assert(same(viaTable, viaAxial));
                }
            }
        }
    }
}

void offsetTableByHand() {
    // Odd-r, direction 1 is NE: an even row steps (0, -1), an odd row steps (+1, -1).
    const Layout l{Orientation::Pointy, Parity::Odd};
    assert(same(neighborOffset(l, {3, 2}, 1), Offset{3, 1}));
    assert(same(neighborOffset(l, {3, 3}, 1), Offset{4, 2}));
    // Even-r flips which rows are pushed right.
    const Layout e{Orientation::Pointy, Parity::Even};
    assert(same(neighborOffset(e, {3, 2}, 1), Offset{4, 1}));
    assert(same(neighborOffset(e, {3, 3}, 1), Offset{3, 2}));
    // Flat odd-q, direction 0 is SE: an even column steps (+1, 0), an odd column steps (+1, +1).
    const Layout f{Orientation::Flat, Parity::Odd};
    assert(same(neighborOffset(f, {2, 2}, 0), Offset{3, 2}));
    assert(same(neighborOffset(f, {3, 2}, 0), Offset{4, 3}));
}

void directionsOnScreen() {
    // Pointy has no up or down neighbor, flat has no left or right neighbor.
    assert(directionOnScreen(Orientation::Pointy, Screen::Right) == 0);
    assert(directionOnScreen(Orientation::Pointy, Screen::UpRight) == 1);
    assert(directionOnScreen(Orientation::Pointy, Screen::UpLeft) == 2);
    assert(directionOnScreen(Orientation::Pointy, Screen::Left) == 3);
    assert(directionOnScreen(Orientation::Pointy, Screen::DownLeft) == 4);
    assert(directionOnScreen(Orientation::Pointy, Screen::DownRight) == 5);
    assert(directionOnScreen(Orientation::Pointy, Screen::Up) == -1);
    assert(directionOnScreen(Orientation::Pointy, Screen::Down) == -1);
    assert(directionOnScreen(Orientation::Flat, Screen::DownRight) == 0);
    assert(directionOnScreen(Orientation::Flat, Screen::UpRight) == 1);
    assert(directionOnScreen(Orientation::Flat, Screen::Up) == 2);
    assert(directionOnScreen(Orientation::Flat, Screen::UpLeft) == 3);
    assert(directionOnScreen(Orientation::Flat, Screen::DownLeft) == 4);
    assert(directionOnScreen(Orientation::Flat, Screen::Down) == 5);
    assert(directionOnScreen(Orientation::Flat, Screen::Left) == -1);
    assert(directionOnScreen(Orientation::Flat, Screen::Right) == -1);
    // Whatever the name says, the neighbor's center really lies on that side of the screen (y points down).
    for (const Orientation o : {Orientation::Pointy, Orientation::Flat}) {
        for (int s = 0; s < 8; ++s) {
            const auto screen = static_cast<Screen>(s);
            const int d = directionOnScreen(o, screen);
            if (d < 0) continue;
            const Point from = center(o, {0, 0});
            const Point to = center(o, neighbor({0, 0}, d));
            const double dx = to.x - from.x;
            const double dy = to.y - from.y;
            const bool left = screen == Screen::Left || screen == Screen::UpLeft || screen == Screen::DownLeft;
            const bool right = screen == Screen::Right || screen == Screen::UpRight || screen == Screen::DownRight;
            const bool up = screen == Screen::Up || screen == Screen::UpLeft || screen == Screen::UpRight;
            const bool down = screen == Screen::Down || screen == Screen::DownLeft || screen == Screen::DownRight;
            assert(left == (dx < -0.1));
            assert(right == (dx > 0.1));
            assert(up == (dy < -0.1));
            assert(down == (dy > 0.1));
        }
    }
}

void arrowKeys() {
    // Pointy: left and right are real neighbors, up and down pick the right-hand one, Shift the left-hand one.
    assert(arrowToScreen(Orientation::Pointy, Arrow::Left, false) == Screen::Left);
    assert(arrowToScreen(Orientation::Pointy, Arrow::Right, false) == Screen::Right);
    assert(arrowToScreen(Orientation::Pointy, Arrow::Up, false) == Screen::UpRight);
    assert(arrowToScreen(Orientation::Pointy, Arrow::Up, true) == Screen::UpLeft);
    assert(arrowToScreen(Orientation::Pointy, Arrow::Down, false) == Screen::DownRight);
    assert(arrowToScreen(Orientation::Pointy, Arrow::Down, true) == Screen::DownLeft);
    // Flat: up and down are real neighbors, left and right pick the upper one, Shift the lower one.
    assert(arrowToScreen(Orientation::Flat, Arrow::Up, false) == Screen::Up);
    assert(arrowToScreen(Orientation::Flat, Arrow::Down, false) == Screen::Down);
    assert(arrowToScreen(Orientation::Flat, Arrow::Left, false) == Screen::UpLeft);
    assert(arrowToScreen(Orientation::Flat, Arrow::Left, true) == Screen::DownLeft);
    assert(arrowToScreen(Orientation::Flat, Arrow::Right, false) == Screen::UpRight);
    assert(arrowToScreen(Orientation::Flat, Arrow::Right, true) == Screen::DownRight);
    // Every arrow resolves to a direction that exists in that orientation.
    for (const Orientation o : {Orientation::Pointy, Orientation::Flat}) {
        for (int a = 0; a < 4; ++a) {
            for (const bool shift : {false, true}) {
                assert(directionOnScreen(o, arrowToScreen(o, static_cast<Arrow>(a), shift)) >= 0);
            }
        }
    }
}

void centersAndCorners() {
    const double s3 = std::sqrt(3.0);
    // Pointy: x = sqrt(3) (q + r / 2), y = 3/2 r. Flat: x = 3/2 q, y = sqrt(3) (r + q / 2).
    assert(near(center(Orientation::Pointy, {1, 0}).x, s3) && near(center(Orientation::Pointy, {1, 0}).y, 0.0));
    assert(near(center(Orientation::Pointy, {0, 1}).x, s3 / 2) && near(center(Orientation::Pointy, {0, 1}).y, 1.5));
    assert(near(center(Orientation::Flat, {1, 0}).x, 1.5) && near(center(Orientation::Flat, {1, 0}).y, s3 / 2));
    assert(near(center(Orientation::Flat, {0, 1}).x, 0.0) && near(center(Orientation::Flat, {0, 1}).y, s3));
    // Corners sit on the unit circle, the first one of a pointy hex points up-right, of a flat hex right.
    assert(near(corner(Orientation::Flat, 0).x, 1.0) && near(corner(Orientation::Flat, 0).y, 0.0));
    assert(near(corner(Orientation::Pointy, 0).x, s3 / 2) && near(corner(Orientation::Pointy, 0).y, -0.5));
    for (const Orientation o : {Orientation::Pointy, Orientation::Flat}) {
        for (int i = 0; i < 6; ++i) {
            const Point p = corner(o, i);
            assert(near(std::hypot(p.x, p.y), 1.0));
        }
        // Tiling: all six neighbors are exactly sqrt(3) away, which is the width across the flat sides.
        for (int d = 0; d < 6; ++d) {
            const Point a = center(o, {0, 0});
            const Point b = center(o, neighbor({0, 0}, d));
            assert(near(std::hypot(b.x - a.x, b.y - a.y), s3));
        }
        // Neighbors share an edge: two corners of one hex are two corners of the other.
        for (int d = 0; d < 6; ++d) {
            const Point shift = center(o, neighbor({0, 0}, d));
            int shared = 0;
            for (int i = 0; i < 6; ++i) {
                for (int k = 0; k < 6; ++k) {
                    const Point a = corner(o, i);
                    const Point b = corner(o, k);
                    if (near(a.x, b.x + shift.x) && near(a.y, b.y + shift.y)) ++shared;
                }
            }
            assert(shared == 2);
        }
    }
}

void boardShape() {
    const Board board{5, 4};
    for (const Layout& l : kLayouts) {
        const auto cells = boardCells(l, board);
        assert(cells.size() == 20);
        std::set<std::pair<int, int>> seen;
        for (const Axial& a : cells) {
            assert(inBoard(l, board, a));
            seen.insert({a.q, a.r});
        }
        assert(seen.size() == 20);  // no cell twice
        const Offset first = toOffset(l, cells.front());
        const Offset last = toOffset(l, cells.back());
        assert(first.col == 0 && first.row == 0);
        assert(last.col == 4 && last.row == 3);
    }
    const Layout odd{Orientation::Pointy, Parity::Odd};
    assert(inBoard(odd, board, toAxial(odd, {4, 3})));
    assert(!inBoard(odd, board, toAxial(odd, {5, 3})));
    assert(!inBoard(odd, board, toAxial(odd, {0, -1})));
    assert(!inBoard(odd, board, toAxial(odd, {0, 4})));
    // The same axial cell can belong to the board in one convention and not in the other.
    const Axial a = toAxial(odd, {4, 3});
    assert(inBoard(odd, board, a));
    assert(!inBoard({Orientation::Pointy, Parity::Even}, board, a));  // even-r shifts it to col 5
}

void boundsOfABoard() {
    // Pointy, 3 x 2, odd rows pushed right: the widest extent is the pushed row.
    const Layout l{Orientation::Pointy, Parity::Odd};
    const Bounds b = boardBounds(l, boardCells(l, {3, 2}));
    const double s3 = std::sqrt(3.0);
    assert(near(b.minX, -s3 / 2));
    assert(near(b.maxX, s3 * 2.5 + s3 / 2 + 0.0));  // col 2 in the odd row sits at sqrt(3) * 2.5
    assert(near(b.minY, -1.0));
    assert(near(b.maxY, 1.5 + 1.0));
}

void picking() {
    const double s3 = std::sqrt(3.0);
    for (const Layout& l : kLayouts) {
        const auto cells = boardCells(l, {6, 5});
        for (std::size_t i = 0; i < cells.size(); ++i) {
            const Point c = center(l.orientation, cells[i]);
            const auto hit = pickCell(l.orientation, cells, c);
            assert(hit && *hit == i);
            // Just inside each corner still picks the cell, just outside a side picks another one or none.
            for (int k = 0; k < 6; ++k) {
                const Point p{c.x + corner(l.orientation, k).x * 0.97, c.y + corner(l.orientation, k).y * 0.97};
                const auto h = pickCell(l.orientation, cells, p);
                assert(h && *h == i);
            }
        }
    }
    // Between two cells: a point 0.4 right of the shared edge belongs to the right-hand cell.
    const auto cells = boardCells({Orientation::Pointy, Parity::Odd}, {4, 1});
    const double edgeX = s3 / 2;  // the edge between col 0 and col 1 in row 0
    const auto left = pickCell(Orientation::Pointy, cells, Point{edgeX - 0.01, 0.0});
    const auto right = pickCell(Orientation::Pointy, cells, Point{edgeX + 0.01, 0.0});
    assert(left && *left == 0);
    assert(right && *right == 1);
    // Beyond the board there is nothing to pick.
    assert(!pickCell(Orientation::Pointy, cells, Point{-1.5, 0.0}));
    assert(!pickCell(Orientation::Pointy, cells, Point{0.0, 3.0}));
    assert(!pickCell(Orientation::Pointy, {}, Point{0.0, 0.0}));
    // A flat hex is only 0.866 high above its center but 1.0 wide: (0, 0.9) is outside, (0.9, 0) is inside.
    const auto flat = boardCells({Orientation::Flat, Parity::Odd}, {1, 1});
    assert(!pickCell(Orientation::Flat, flat, Point{0.0, 0.9}));
    assert(pickCell(Orientation::Flat, flat, Point{0.9, 0.0}));
    assert(!pickCell(Orientation::Pointy, boardCells({Orientation::Pointy, Parity::Odd}, {1, 1}), Point{0.9, 0.0}));
}

}  // namespace

int main() {
    cubeKeepsTheSumAtZero();
    offsetToAxialByHand();
    offsetRoundTrip();
    doubledByHand();
    distanceByHand();
    distanceFormulasAgree();
    distanceCountsSteps();
    naiveOffsetDistanceIsWrong();
    neighborsByHand();
    offsetTablesMatchAxial();
    offsetTableByHand();
    directionsOnScreen();
    arrowKeys();
    centersAndCorners();
    boardShape();
    boundsOfABoard();
    picking();
    std::puts("hex-koordinaten logic: all asserts passed");
    return 0;
}
