// Tests for the grid raycaster logic. Plain asserts on hand-computed cases, no framework, no SDL.
// Coordinates: x grows to the right, y grows downwards, cell (cx, cy) covers x in [cx, cx+1) and y in [cy, cy+1).
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "logic.hpp"

using namespace grc;

namespace {

bool near(double a, double b, double eps = 1e-9) { return std::fabs(a - b) < eps; }

/// 10 x 3: one corridor, row 1 is open from x = 1 to x = 8.
Map corridor() { return Map::fromRows({"##########", "#........#", "##########"}); }

/// Open room with a closed border and nothing inside.
Map room(int w, int h) {
    Map m(w, h);
    for (int x = 0; x < w; ++x) {
        m.set(x, 0, true);
        m.set(x, h - 1, true);
    }
    for (int y = 0; y < h; ++y) {
        m.set(0, y, true);
        m.set(w - 1, y, true);
    }
    return m;
}

void mapBasics() {
    const Map m = corridor();
    assert(m.width() == 10 && m.height() == 3);
    assert(m.solid(0, 1) && m.solid(9, 1) && m.solid(3, 0));
    assert(!m.solid(1, 1) && !m.solid(8, 1));
    // Everything outside counts as wall, so a ray can never leave the map.
    assert(m.solid(-1, 1) && m.solid(10, 1) && m.solid(4, -1) && m.solid(4, 3));
    Map r = room(5, 5);
    assert(!r.solid(2, 2));
    r.toggle(2, 2);
    assert(r.solid(2, 2));
    r.toggle(2, 2);
    assert(!r.solid(2, 2));
    r.set(-3, 2, true);  // outside: ignored, no crash
    r.set(2, 99, true);
}

void ddaStraightLines() {
    const Map m = corridor();
    // Right: from the middle of cell 1, the wall cell 9 starts at x = 9, so 7.5 units away. Cells 2..9 are entered.
    std::vector<Visit> visits;
    Hit h = castDda(m, {1.5, 1.5}, {1, 0}, &visits);
    assert(h.found && h.cx == 9 && h.cy == 1);
    assert(near(h.dist, 7.5));
    assert(h.side == Side::Vertical);
    assert(h.steps == 8 && visits.size() == 8);
    assert(visits.front().cx == 2 && near(visits.front().enter.x, 2.0) && near(visits.front().enter.y, 1.5));
    assert(visits.back().cx == 9);
    // Left: x = 1 is the first line, the wall is behind it.
    h = castDda(m, {1.5, 1.5}, {-1, 0});
    assert(h.found && h.cx == 0 && near(h.dist, 0.5) && h.steps == 1 && h.side == Side::Vertical);
    // Down: y = 2 is the first line, the wall row is behind it. A crossing of a horizontal line is a Horizontal hit.
    h = castDda(m, {1.5, 1.5}, {0, 1});
    assert(h.found && h.cx == 1 && h.cy == 2 && near(h.dist, 0.5) && h.side == Side::Horizontal);
    // Up
    h = castDda(m, {4.25, 1.75}, {0, -1});
    assert(h.found && h.cx == 4 && h.cy == 0 && near(h.dist, 0.75) && h.side == Side::Horizontal);
    // Only the direction counts, not its length.
    const Hit scaled = castDda(m, {1.5, 1.5}, {3, 0});
    assert(scaled.found && scaled.cx == 9 && near(scaled.dist, 7.5));
}

void ddaDiagonalAndTies() {
    Map m = room(7, 7);
    // From the middle of cell (1,1) along the diagonal the ray reaches the corner (2,2) after sqrt(2)/2 and both
    // lines are crossed at the same moment. Convention: x first, so cell (2,1) is visited before (2,2).
    m.set(2, 1, true);
    Hit h = castDda(m, {1.5, 1.5}, {1, 1});
    assert(h.found && h.cx == 2 && h.cy == 1 && h.side == Side::Vertical);
    assert(near(h.dist, std::sqrt(2.0) / 2.0));
    m.set(2, 1, false);
    m.set(2, 2, true);
    h = castDda(m, {1.5, 1.5}, {1, 1});
    assert(h.found && h.cx == 2 && h.cy == 2 && h.side == Side::Horizontal);
    assert(near(h.dist, std::sqrt(2.0) / 2.0));
    // A slanted ray from (1.5, 1.25) with direction (2, 1) crosses x = 2 after 0.25 * sqrt(5) and y = 2 after
    // 0.75 * sqrt(5), so the first cell it enters is (2,1).
    Map n = room(8, 8);
    n.set(2, 1, true);
    h = castDda(n, {1.5, 1.25}, {2, 1});
    assert(h.found && h.cx == 2 && h.cy == 1 && h.side == Side::Vertical);
    assert(near(h.dist, 0.25 * std::sqrt(5.0)));
}

void ddaDegenerate() {
    const Map m = corridor();
    Hit h = castDda(m, {1.5, 1.5}, {0, 0});
    assert(!h.found && h.steps == 0);
    // Starting inside a wall: reported right away, nothing to walk.
    h = castDda(m, {0.5, 1.5}, {1, 0});
    assert(h.found && h.cx == 0 && h.cy == 1 && near(h.dist, 0.0) && h.steps == 0);
    // Starting outside the map counts the same way.
    h = castDda(m, {-3.0, 1.5}, {1, 0});
    assert(h.found && near(h.dist, 0.0) && h.steps == 0);
    h = castDda(m, {1.5, 1.5}, {std::nan(""), 1.0});
    assert(!h.found && h.steps == 0);
}

void ddaVisitsAreConnected() {
    // A ray may only ever enter a neighbour that shares an edge. Every angle, three starts.
    const Map m = Map::fromRows({
        "##############",
        "#............#",
        "#..#.....#...#",
        "#............#",
        "#.....##.....#",
        "#............#",
        "##############",
    });
    const Vec2 starts[] = {{2.5, 3.5}, {6.37, 5.21}, {10.9, 1.1}};
    for (const Vec2 s : starts) {
        for (int a = 0; a < 720; ++a) {
            const double angle = (static_cast<double>(a) + 0.25) * kPi / 360.0;
            std::vector<Visit> visits;
            const Hit h = castDda(m, s, {std::cos(angle), std::sin(angle)}, &visits);
            assert(h.found);
            int px = static_cast<int>(std::floor(s.x));
            int py = static_cast<int>(std::floor(s.y));
            for (const Visit& v : visits) {
                assert(std::abs(v.cx - px) + std::abs(v.cy - py) == 1);
                px = v.cx;
                py = v.cy;
            }
            assert(px == h.cx && py == h.cy);
            assert(static_cast<int>(visits.size()) == h.steps);
        }
    }
}

void marchBasics() {
    const Map m = corridor();
    // Step 1.0: samples at x = 2.5, 3.5, ... 8.5, then 9.5 is inside the wall. The true wall is at 7.5, the march
    // reports 8.0, half a cell too far. That overshoot is why walls look jagged.
    Hit h = castMarch(m, {1.5, 1.5}, {1, 0}, 1.0);
    assert(h.found && h.cx == 9 && near(h.dist, 8.0) && h.steps == 8);
    // Step 0.5: x = 9.0 is a sample and floor(9.0) is 9, so the hit is exact. 15 samples.
    h = castMarch(m, {1.5, 1.5}, {1, 0}, 0.5);
    assert(h.found && near(h.dist, 7.5) && h.steps == 15);
    // Step 0.25: 30 samples, same exact hit.
    h = castMarch(m, {1.5, 1.5}, {1, 0}, 0.25);
    assert(h.found && near(h.dist, 7.5) && h.steps == 30);
    // Step 0.1: the sample near 7.5 decides, 75 or 76 depending on rounding, but never less than 75.
    h = castMarch(m, {1.5, 1.5}, {1, 0}, 0.1);
    assert(h.found && h.cx == 9 && h.steps >= 75 && h.steps <= 76);
    assert(h.dist >= 7.5 - 1e-9 && h.dist < 7.6);
    // The DDA needed 8 cells for the same ray.
    assert(castDda(m, {1.5, 1.5}, {1, 0}).steps == 8);
    // Side of a march hit: the cell changed in y.
    h = castMarch(m, {1.5, 1.5}, {0, 1}, 0.25);
    assert(h.found && h.cy == 2 && h.side == Side::Horizontal && near(h.dist, 0.5));
    h = castMarch(m, {1.5, 1.5}, {1, 0}, 0.25);
    assert(h.side == Side::Vertical);
}

void marchDegenerate() {
    const Map m = corridor();
    assert(!castMarch(m, {1.5, 1.5}, {0, 0}, 0.1).found);
    // Nonsense step sizes are clamped instead of looping forever or crashing.
    Hit h = castMarch(m, {1.5, 1.5}, {1, 0}, 0.0);
    assert(h.found && h.cx == 9);
    h = castMarch(m, {1.5, 1.5}, {1, 0}, -2.0);
    assert(h.found && h.cx == 9);
    // Too short a reach: nothing found.
    h = castMarch(m, {1.5, 1.5}, {1, 0}, 0.5, 3.0);
    assert(!h.found);
    // Starting in a wall.
    h = castMarch(m, {0.5, 1.5}, {1, 0}, 0.5);
    assert(h.found && h.cx == 0 && near(h.dist, 0.0) && h.steps == 0);
}

void marchMissesACorner() {
    // The line x + y = 7.1 runs from (3.5, 3.6) towards the lower right and clips the lower-left corner of cell
    // (5,2): inside that cell from (5, 2.1) to (5.1, 2.0), a chord of 0.14. By hand: t = 1.5 * sqrt(2) at the
    // entry, t = 1.6 * sqrt(2) at the exit.
    Map m = room(12, 8);
    m.set(5, 2, true);
    const Vec2 origin{3.5, 3.6};
    const Vec2 dir{1, -1};
    std::vector<Visit> visits;
    const Hit d = castDda(m, origin, dir, &visits);
    assert(d.found && d.cx == 5 && d.cy == 2 && d.side == Side::Vertical);
    assert(near(d.dist, 1.5 * std::sqrt(2.0)));
    assert(d.steps == 3);  // (4,3), (4,2), (5,2)
    assert(visits[0].cx == 4 && visits[0].cy == 3 && visits[1].cx == 4 && visits[1].cy == 2);
    // A step of 1.0 jumps over the corner (samples at t = 2 and 3 lie in cells (4,2) and (5,1)).
    const Hit coarse = castMarch(m, origin, dir, 1.0);
    assert(!(coarse.found && coarse.cx == 5 && coarse.cy == 2));
    // A step of 0.1 lands inside the chord at t = 2.2 (cell (5,2) from t = 2.1213 to 2.2627).
    const Hit fine = castMarch(m, origin, dir, 0.1);
    assert(fine.found && fine.cx == 5 && fine.cy == 2);
    assert(near(fine.dist, 2.2, 1e-9));
}

void marchAgainstDda() {
    // A march with a tiny step is the reference answer: the DDA must land on the same cell, at the distance the
    // march reports up to one step. Starts with awkward fractions so no ray runs exactly through a lattice point.
    const Map m = Map::fromRows({
        "####################",
        "#..................#",
        "#..###.......#.#.#.#",
        "#..#...........#...#",
        "#..#....#..........#",
        "#.......##.........#",
        "#...#..............#",
        "#...#...####.......#",
        "#.......#..#.......#",
        "#..##...#..#..#....#",
        "#..................#",
        "####################",
    });
    const double step = 1.0 / 2048.0;
    const Vec2 starts[] = {{2.3711, 5.4263}, {9.6127, 9.8731}, {16.2549, 3.1417}};
    int cells = 0;
    int differing = 0;
    for (const Vec2 s : starts) {
        assert(!m.solid(static_cast<int>(s.x), static_cast<int>(s.y)));
        for (int a = 0; a < 720; ++a) {
            const double angle = (static_cast<double>(a) + 0.37) * kPi / 360.0;
            const Vec2 dir{std::cos(angle), std::sin(angle)};
            const Hit d = castDda(m, s, dir);
            const Hit r = castMarch(m, s, dir, step);
            assert(d.found && r.found);
            ++cells;
            if (d.cx != r.cx || d.cy != r.cy) {
                ++differing;
            } else {
                assert(r.dist >= d.dist - 1e-9 && r.dist <= d.dist + step + 1e-9);
            }
        }
    }
    assert(cells == 2160);
    assert(differing == 0);
}

void cameraRays() {
    // 90 degree view, three columns, looking along +x: the plane is (0, 1), camera x = -2/3, 0, 2/3.
    const Camera cam{{5.0, 5.0}, 0.0, kPi / 2.0};
    const Vec2 mid = columnRay(cam, 1, 3);
    assert(near(mid.x, 1.0) && near(mid.y, 0.0));
    const Vec2 left = columnRay(cam, 0, 3);
    assert(near(left.x, 1.0 / std::sqrt(1.0 + 4.0 / 9.0), 1e-12) && near(left.y, -(2.0 / 3.0) / std::sqrt(1.0 + 4.0 / 9.0), 1e-12));
    const Vec2 right = columnRay(cam, 2, 3);
    assert(near(right.y, -left.y, 1e-12) && near(right.x, left.x, 1e-12));
    // All rays have length 1, also when turned.
    const Camera turned{{5.0, 5.0}, 1.234, 1.4};
    for (int i = 0; i < 40; ++i) {
        const Vec2 r = columnRay(turned, i, 40);
        assert(near(r.x * r.x + r.y * r.y, 1.0, 1e-12));
    }
    // Looking down (+y): the right-hand side of the screen is -x (y points down).
    const Camera down{{5.0, 5.0}, kPi / 2.0, kPi / 2.0};
    assert(columnRay(down, 2, 3).x < 0.0);
    const Vec2 v = viewDir(down);
    assert(near(v.x, 0.0, 1e-12) && near(v.y, 1.0));
}

void perpendicularDistance() {
    const Vec2 view{1.0, 0.0};
    assert(near(perpendicular(10.0, {1.0, 0.0}, view), 10.0));
    // 60 degrees off the view direction: cos = 0.5.
    assert(near(perpendicular(10.0, {0.5, std::sqrt(3.0) / 2.0}, view), 5.0, 1e-12));
    // 90 degrees: nothing in front.
    assert(near(perpendicular(10.0, {0.0, 1.0}, view), 0.0, 1e-12));
}

void flatWallIsFlat() {
    // The point of the correction. Looking at the straight east wall, the corrected distance is the same in every
    // column (the wall is x = 23, the eye x = 2.5). The straight-line distance grows towards the edges.
    const Map m = room(24, 40);
    const Camera cam{{2.5, 20.5}, 0.0, 80.0 * kPi / 180.0};
    const int columns = 61;
    double perpMin = 1e9, perpMax = 0.0, eucMin = 1e9, eucMax = 0.0;
    for (int i = 0; i < columns; ++i) {
        const Vec2 ray = columnRay(cam, i, columns);
        const Hit h = castDda(m, cam.pos, ray);
        assert(h.found && h.cx == 23 && h.side == Side::Vertical);
        const double z = perpendicular(h.dist, ray, viewDir(cam));
        perpMin = std::min(perpMin, z);
        perpMax = std::max(perpMax, z);
        eucMin = std::min(eucMin, h.dist);
        eucMax = std::max(eucMax, h.dist);
    }
    assert(near(perpMin, 20.5, 1e-9) && near(perpMax, 20.5, 1e-9));
    assert(near(eucMin, 20.5, 1e-9));  // the middle column looks straight at the wall
    assert(eucMax > 1.25 * eucMin);    // 40 degrees off to the side the wall is 1 / cos(40) = 1.3 times farther
}

void wallHeights() {
    // viewWidth / (2 tan(fov / 2)) is the focal length in pixels: 480 / 2 = 240 for a 90 degree view.
    assert(near(wallHeight(1.0, 480.0, kPi / 2.0), 240.0, 1e-9));
    assert(near(wallHeight(2.0, 480.0, kPi / 2.0), 120.0, 1e-9));
    assert(near(wallHeight(4.0, 480.0, kPi / 2.0), 60.0, 1e-9));
    // Right at the eye the height stays finite.
    assert(std::isfinite(wallHeight(0.0, 480.0, kPi / 2.0)));
    assert(near(wallHeight(0.0, 480.0, kPi / 2.0), wallHeight(kNearClip, 480.0, kPi / 2.0)));
    assert(wallHeight(-3.0, 480.0, kPi / 2.0) == wallHeight(0.0, 480.0, kPi / 2.0));
    // A wall two cells high is twice as tall at the same depth.
    assert(near(wallHeight(2.0, 480.0, kPi / 2.0, 2.0), 240.0, 1e-9));
}

void standing() {
    const Map m = room(6, 6);
    assert(canStand(m, {1.5, 1.5}, 0.25));
    assert(canStand(m, {1.25, 1.5}, 0.25));   // the left edge touches x = 1 exactly, still cell 1
    assert(!canStand(m, {1.2, 1.5}, 0.25));   // 0.95 is inside the wall
    assert(!canStand(m, {1.5, 4.8}, 0.25));   // bottom edge at 5.05
    assert(!canStand(m, {-1.0, 1.5}, 0.25));
    Map pillar = room(6, 6);
    pillar.set(3, 3, true);
    assert(!canStand(pillar, {2.8, 2.8}, 0.25));  // the corner (3.05, 3.05) is in the pillar
    assert(canStand(pillar, {2.7, 2.7}, 0.25));
}

void moving() {
    const Map m = room(6, 6);
    Vec2 p = tryMove(m, {1.5, 1.5}, {0.5, 0.5}, 0.25);
    assert(near(p.x, 2.0) && near(p.y, 2.0));
    // Into the west wall with a bit of y: x stays, y still moves (sliding).
    p = tryMove(m, {1.5, 1.5}, {-1.0, 0.25}, 0.25);
    assert(near(p.x, 1.5) && near(p.y, 1.75));
    // Straight into a corner: nothing moves.
    p = tryMove(m, {1.5, 1.5}, {-1.0, -1.0}, 0.25);
    assert(near(p.x, 1.5) && near(p.y, 1.5));
    assert(near(normalizeAngle(0.0), 0.0));
    assert(near(normalizeAngle(7.0), 7.0 - 2.0 * kPi));
    assert(near(normalizeAngle(-0.5), 2.0 * kPi - 0.5));
    assert(near(normalizeAngle(2.0 * kPi), 0.0));
    assert(near(normalizeAngle(std::nan("")), 0.0));
}

void scanning() {
    // One column straight along the corridor: 8 cells for the DDA, 15 samples at step 0.5, same wall cell.
    const Map c = corridor();
    Scan s = scan(c, {{1.5, 1.5}, 0.0, 1.0}, 1, 0.5);
    assert(s.columns.size() == 1);
    assert(near(s.ddaSteps, 8.0) && near(s.marchSteps, 15.0) && s.different == 0);
    // Step 1.0 reports the hit half a cell late, but in the right cell: not "different".
    s = scan(c, {{1.5, 1.5}, 0.0, 1.0}, 1, 1.0);
    assert(s.different == 0 && near(s.columns[0].march.dist, 8.0) && near(s.columns[0].dda.dist, 7.5));
    // The corner-clipping ray from marchMissesACorner as the middle column of a one-column view.
    Map m = room(12, 8);
    m.set(5, 2, true);
    const Camera cam{{3.5, 3.6}, -kPi / 4.0, 1.0};
    assert(scan(m, cam, 1, 1.0).different == 1);
    assert(scan(m, cam, 1, 0.1).different == 0);
    // No columns: empty, no division by zero.
    s = scan(m, cam, 0, 0.5);
    assert(s.columns.empty() && near(s.ddaSteps, 0.0) && near(s.marchSteps, 0.0) && s.different == 0);
    // In a wide view the fine march agrees nearly everywhere, the coarse one does not, and costs more.
    const Map world = defaultMap();
    const Camera look{startPosition(), startAngle(), 80.0 * kPi / 180.0};
    const Scan fine = scan(world, look, 120, 0.05);
    const Scan coarse = scan(world, look, 120, 1.0);
    assert(fine.marchSteps > 5.0 * fine.ddaSteps);
    assert(coarse.different > fine.different);
}

void defaultWorld() {
    const Map m = defaultMap();
    for (int x = 0; x < m.width(); ++x) assert(m.solid(x, 0) && m.solid(x, m.height() - 1));
    for (int y = 0; y < m.height(); ++y) assert(m.solid(0, y) && m.solid(m.width() - 1, y));
    const Vec2 p = startPosition();
    assert(canStand(m, p, kPlayerRadius));
    // Looking from the start along the start angle ends at a wall.
    const Vec2 d{std::cos(startAngle()), std::sin(startAngle())};
    assert(castDda(m, p, d).found);
}

}  // namespace

int main() {
    mapBasics();
    ddaStraightLines();
    ddaDiagonalAndTies();
    ddaDegenerate();
    ddaVisitsAreConnected();
    marchBasics();
    marchDegenerate();
    marchMissesACorner();
    marchAgainstDda();
    cameraRays();
    perpendicularDistance();
    flatWallIsFlat();
    wallHeights();
    standing();
    moving();
    scanning();
    defaultWorld();
    std::puts("grid-raycaster logic: all asserts passed");
    return 0;
}
