// Tests for the hex range and sight logic. Plain asserts on hand-computed cases, no framework, no SDL.
// Most cases use the "unit" layout (hex size 1, hex (0,0) at the pixel origin) so the numbers can be checked on paper:
// pointy-top hexes, corners at -30, 30, 90, 150, 210 and 270 degrees, y grows downwards.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "logic.hpp"

using namespace hx;

namespace {

bool near(double a, double b, double eps = 1e-9) { return std::fabs(a - b) < eps; }

const Layout kUnit{1.0, {0.0, 0.0}};
constexpr double kRoot3 = 1.7320508075688772;

// A 9x9 map: row 4, column 4 is the middle, which is axial (2,4). Radius 4 around it stays inside the map.
constexpr Hex kO{2, 4};

Hex at(int dq, int dr) { return {kO.q + dq, kO.r + dr}; }

Map emptyMap() { return Map(9, 9); }

Map withWalls(std::initializer_list<Hex> walls) {
    Map m = emptyMap();
    for (const Hex& w : walls) m.setWall(w, true);
    return m;
}

int count(const Map& m, const std::vector<Seen>& seen, Seen what) {
    int n = 0;
    for (std::size_t i = 0; i < m.size(); ++i) n += seen[i] == what ? 1 : 0;
    return n;
}

void distances() {
    assert(distance({0, 0}, {0, 0}) == 0);
    assert(distance({0, 0}, {2, -1}) == 2);
    assert(distance({0, 0}, {3, -3}) == 3);
    assert(distance({0, 0}, {1, 1}) == 2);  // no direction has q and r both +1, so (1,1) is two steps away
    assert(distance({0, 0}, {-2, 1}) == 2);
    assert(distance({1, 2}, {-1, 3}) == 2);
    for (int d = 0; d < 6; ++d) {
        assert(distance({0, 0}, neighbor({0, 0}, d)) == 1);
        assert(distance(neighbor({5, -3}, d), {5, -3}) == 1);
    }
    // The six neighbors are six different hexes.
    for (int a = 0; a < 6; ++a) {
        for (int b = a + 1; b < 6; ++b) assert(neighbor({0, 0}, a) != neighbor({0, 0}, b));
    }
}

void pixelsAndCorners() {
    const Vec2 e = toPixel(kUnit, {1, 0});
    assert(near(e.x, kRoot3) && near(e.y, 0.0));
    const Vec2 se = toPixel(kUnit, {0, 1});  // one row down, half a hex to the right
    assert(near(se.x, kRoot3 / 2.0) && near(se.y, 1.5));
    const Vec2 below = toPixel(kUnit, {-1, 2});
    assert(near(below.x, 0.0) && near(below.y, 3.0));
    const Vec2 shifted = toPixel(Layout{20.0, {100.0, 50.0}}, {0, 0});
    assert(near(shifted.x, 100.0) && near(shifted.y, 50.0));
    // The edge normals point at the six neighbors, in the order of kDirections.
    for (int d = 0; d < 6; ++d) {
        const Vec2 n = toPixel(kUnit, neighbor({0, 0}, d));
        assert(near(n.x, kNormals[static_cast<std::size_t>(d)].x * kRoot3));
        assert(near(n.y, kNormals[static_cast<std::size_t>(d)].y * kRoot3));
    }
    // Corners: all one size away from the center, corner 0 is at -30 degrees (right, up).
    for (int i = 0; i < 6; ++i) {
        const Vec2 c = corner(kUnit, {0, 0}, i);
        assert(near(std::hypot(c.x, c.y), 1.0));
    }
    assert(near(corner(kUnit, {0, 0}, 0).x, kRoot3 / 2.0) && near(corner(kUnit, {0, 0}, 0).y, -0.5));
    assert(near(corner(kUnit, {0, 0}, 5).x, 0.0) && near(corner(kUnit, {0, 0}, 5).y, -1.0));
    // Three hexes meet in a corner: (0,0) corner 0, (1,0) corner 4 and (1,-1) corner 2 are the same point.
    const Vec2 a = corner(kUnit, {0, 0}, 0);
    const Vec2 b = corner(kUnit, {1, 0}, 4);
    const Vec2 c = corner(kUnit, {1, -1}, 2);
    assert(near(a.x, b.x) && near(a.y, b.y) && near(a.x, c.x) && near(a.y, c.y));
}

void rounding() {
    assert(roundHex(0.1, 0.1) == Hex(0, 0));
    assert(roundHex(1.2, -0.1) == Hex(1, 0));
    assert(roundHex(0.9, 0.9) == Hex(1, 1));  // plain rounding gives (1,1), and q + r + s = 0 holds with s = -2
    assert(roundHex(-0.3, 0.6) == Hex(0, 0));  // squared cube distances: 0.54 to (0,0), 0.74 to (0,1)
    assert(roundHex(-0.3, 0.8) == Hex(0, 1));  // 0.98 to (0,0), 0.38 to (0,1)
    assert(roundHex(2.0, -1.0) == Hex(2, -1));  // exact values stay
    assert(roundHex(std::numeric_limits<double>::quiet_NaN(), 0.0) == Hex(0, 0));
    assert(roundHex(1e300, 0.0).q <= 1000000 + 1);  // clamped, no float to int overflow
}

void pixelRoundTrip() {
    const Layout layout{20.0, {279.3, 56.0}};
    const Map m = emptyMap();
    for (std::size_t i = 0; i < m.size(); ++i) {
        const Hex h = m.hexAtIndex(i);
        const Vec2 c = toPixel(layout, h);
        assert(fromPixel(layout, c) == h);
        // 0.9 apothems towards a neighbor is still inside, 1.1 apothems is already the neighbor.
        const double apothem = layout.size * kRoot3 / 2.0;
        for (int d = 0; d < 6; ++d) {
            const Vec2 n = kNormals[static_cast<std::size_t>(d)];
            assert(fromPixel(layout, {c.x + n.x * apothem * 0.9, c.y + n.y * apothem * 0.9}) == h);
            assert(fromPixel(layout, {c.x + n.x * apothem * 1.1, c.y + n.y * apothem * 1.1}) == neighbor(h, d));
        }
    }
}

void mapShape() {
    const Map m = emptyMap();
    assert(m.size() == 81);
    assert(m.contains({0, 0}) && !m.contains({-1, 0}) && m.contains({8, 0}) && !m.contains({9, 0}));
    assert(m.contains({0, 1}) && !m.contains({-1, 1}) && m.contains({8, 1}) && !m.contains({9, 1}));
    assert(m.contains({-1, 2}) && !m.contains({-2, 2}) && m.contains({7, 2}) && !m.contains({8, 2}));
    assert(m.contains({-4, 8}) && !m.contains({-5, 8}) && m.contains({4, 8}) && !m.contains({5, 8}));
    assert(!m.contains({0, 9}) && !m.contains({0, -1}));
    assert(m.hexAt(4, 4) == kO);
    assert(m.hexAt(0, 0) == Hex(0, 0));
    assert(m.hexAt(8, 8) == Hex(4, 8));
    // index and hexAtIndex are inverse and cover every cell once.
    for (std::size_t i = 0; i < m.size(); ++i) {
        assert(m.contains(m.hexAtIndex(i)));
        assert(m.index(m.hexAtIndex(i)) == i);
    }
    int inside = 0;
    for (int d = 0; d < 6; ++d) inside += m.contains(neighbor(kO, d)) ? 1 : 0;
    assert(inside == 6);
}

void walls() {
    Map m = emptyMap();
    assert(!m.wall(kO));
    m.setWall(kO, true);
    assert(m.wall(kO) && m.walls().size() == 1 && m.walls()[0] == kO);
    m.setWall({100, 100}, true);  // outside: ignored
    assert(m.walls().size() == 1 && !m.wall({100, 100}));
    m.setWall(kO, false);
    assert(m.walls().empty());
    m.setWall(kO, true);
    m.clearWalls();
    assert(m.walls().empty());
}

void rangeLoop() {
    const Map m = emptyMap();
    assert(hexesInRange(m, kO, 0).size() == 1);
    assert(hexesInRange(m, kO, 1).size() == 7);
    assert(hexesInRange(m, kO, 2).size() == 19);  // 3 n (n + 1) + 1
    assert(hexesInRange(m, kO, 4).size() == 61);
    assert(hexesInRange(m, kO, -1).empty());
    for (const Hex& h : hexesInRange(m, kO, 3)) assert(distance(h, kO) <= 3);
    // In the corner of the map the ring is cut: (0,0) keeps itself, (1,0) and (0,1).
    assert(hexesInRange(m, {0, 0}, 1).size() == 3);
}

void floodWithoutWalls() {
    const Map m = emptyMap();
    const Reach r = floodFill(m, kO, 2);
    assert(r.layers.size() == 3);
    assert(r.layers[0].size() == 1 && r.layers[1].size() == 6 && r.layers[2].size() == 12);
    assert(r.cost[m.index(kO)] == 0);
    assert(r.cost[m.index(at(1, 0))] == 1);
    assert(r.cost[m.index(at(2, -1))] == 2);
    assert(r.cost[m.index(at(3, 0))] == -1);
    assert(floodFill(m, kO, 0).layers.size() == 1);
    assert(floodFill(m, kO, -1).layers.empty());
    // Without walls the loop and the flood fill agree, from every cell and for every size, edges included.
    for (std::size_t i = 0; i < m.size(); ++i) {
        for (int n = 0; n <= 6; ++n) {
            const Reach f = floodFill(m, m.hexAtIndex(i), n);
            std::size_t reached = 0;
            for (int c : f.cost) reached += c >= 0 ? 1 : 0;
            assert(reached == hexesInRange(m, m.hexAtIndex(i), n).size());
        }
    }
}

void floodWithWalls() {
    // One wall east of the unit. (4,4) is two steps away but the shortest way around the wall takes three.
    Map m = withWalls({at(1, 0)});
    assert(floodFill(m, kO, 2).cost[m.index(at(2, 0))] == -1);
    assert(floodFill(m, kO, 3).cost[m.index(at(2, 0))] == 3);
    assert(floodFill(m, kO, 3).cost[m.index(at(1, 0))] == -1);  // a wall is never entered
    assert(distance(kO, at(2, 0)) == 2);

    // Six walls around the unit: it can move nowhere, the loop still claims 19 hexes (minus nothing).
    Map ring = emptyMap();
    for (int d = 0; d < 6; ++d) ring.setWall(neighbor(kO, d), true);
    const Reach trapped = floodFill(ring, kO, 5);
    assert(trapped.layers.size() == 1);
    assert(hexesInRange(ring, kO, 2).size() == 19);
    // Open the west side: one step out, two steps to the hex behind the gap.
    ring.setWall(neighbor(kO, 3), false);
    const Reach out = floodFill(ring, kO, 2);
    assert(out.cost[ring.index(neighbor(kO, 3))] == 1);
    assert(out.cost[ring.index(at(-2, 0))] == 2);

    // A complete wall in column 6 splits the map: nothing east of it is reachable, however many points.
    Map split = emptyMap();
    for (int row = 0; row < 9; ++row) split.setWall(split.hexAt(row, 6), true);
    const Hex west = split.hexAt(4, 2);
    const Reach west5 = floodFill(split, west, 5);
    int eastInLoop = 0;
    for (const Hex& h : hexesInRange(split, west, 5)) eastInLoop += (h.q + (h.r >> 1)) >= 7 ? 1 : 0;
    assert(eastInLoop > 0);  // the loop reaches over the wall
    for (std::size_t i = 0; i < split.size(); ++i) {
        if (west5.cost[i] >= 0) assert(split.hexAtIndex(i).q + (split.hexAtIndex(i).r >> 1) <= 5);
    }

    // Unit on a wall or outside the map: nothing to fill.
    assert(floodFill(m, at(1, 0), 3).layers.empty());
    assert(floodFill(m, {50, 50}, 3).layers.empty());
}

void lines() {
    const std::vector<Hex> east = hexLine({0, 0}, {3, 0});
    assert(east.size() == 4 && east[0] == Hex(0, 0) && east[1] == Hex(1, 0) && east[2] == Hex(2, 0) && east[3] == Hex(3, 0));
    const std::vector<Hex> north = hexLine({0, 0}, {0, -3});
    assert(north.size() == 4 && north[1] == Hex(0, -1) && north[2] == Hex(0, -2) && north[3] == Hex(0, -3));
    const std::vector<Hex> diagonal = hexLine({0, 0}, {3, -3});
    assert(diagonal.size() == 4 && diagonal[1] == Hex(1, -1) && diagonal[2] == Hex(2, -2));
    assert(hexLine({4, 4}, {4, 4}).size() == 1);
    // (0,0) to (2,-1): the middle hex is a tie between (1,0) and (1,-1). The nudge makes it (1,0), always.
    const std::vector<Hex> tie = hexLine({0, 0}, {2, -1});
    assert(tie.size() == 3 && tie[1] == Hex(1, 0));
    assert(hexLine({0, 0}, {2, -1}) == tie);
    // Every step of every line is a neighbor, in every direction.
    for (int dq = -4; dq <= 4; ++dq) {
        for (int dr = -4; dr <= 4; ++dr) {
            const Hex to{dq, dr};
            const std::vector<Hex> line = hexLine({0, 0}, to);
            assert(static_cast<int>(line.size()) == distance({0, 0}, to) + 1);
            assert(line.front() == Hex(0, 0) && line.back() == to);
            for (std::size_t i = 1; i < line.size(); ++i) assert(distance(line[i - 1], line[i]) == 1);
        }
    }
}

void segmentAgainstHex() {
    const Hex h{0, 0};
    assert(segmentCrossesHex(kUnit, h, {-3, 0}, {3, 0}));
    assert(segmentCrossesHex(kUnit, h, {3, 0}, {-3, 0}));  // direction does not matter
    assert(segmentCrossesHex(kUnit, h, {-2, -1.5}, {2, 1.5}));
    assert(segmentCrossesHex(kUnit, h, {-3, 0.9}, {3, 0.9}));    // clips the bottom tip: x from -0.173 to 0.173
    assert(!segmentCrossesHex(kUnit, h, {-3, 1.05}, {3, 1.05}));  // just past the bottom corner (0,1)
    assert(!segmentCrossesHex(kUnit, h, {-3, 1.0}, {3, 1.0}));    // touches only the bottom corner (0,1)
    assert(!segmentCrossesHex(kUnit, h, {-3, -1.0}, {3, -1.0}));  // same at the top corner (0,-1)
    // Sliding along an edge is not crossing: x = 0.866 is the right edge of (0,0) and the left edge of (1,0).
    assert(!segmentCrossesHex(kUnit, h, {kRoot3 / 2.0, -3}, {kRoot3 / 2.0, 3}));
    assert(!segmentCrossesHex(kUnit, {1, 0}, {kRoot3 / 2.0, -3}, {kRoot3 / 2.0, 3}));
    // A segment that stops at the edge does not enter, one that goes a little further does.
    assert(!segmentCrossesHex(kUnit, h, {-3, 0}, {-kRoot3 / 2.0, 0}));
    assert(segmentCrossesHex(kUnit, h, {-3, 0}, {-0.8, 0}));
    assert(!segmentCrossesHex(kUnit, h, {-3, 0}, {-1, 0}));  // stops short
    assert(segmentCrossesHex(kUnit, h, {-0.1, 0}, {0.1, 0}));  // entirely inside
    assert(!segmentCrossesHex(kUnit, {5, 5}, {-3, 0}, {3, 0}));  // another hex far away
    // Scale and offset of the layout do not change the answer.
    const Layout big{20.0, {300.0, 80.0}};
    assert(segmentCrossesHex(big, h, {280.0, 80.0}, {320.0, 80.0}));
    assert(!segmentCrossesHex(big, h, {250.0, 100.0}, {350.0, 100.0}));  // y = 80 + 20 touches the corner only
}

void sightByCenter() {
    // Open field: everything is visible.
    Map open = emptyMap();
    assert(sees(Sight::Center, open, kUnit, open.walls(), kO, at(3, 0)));
    // A wall right in front blocks the straight line behind it.
    Map wall = withWalls({at(1, 0)});
    assert(!sees(Sight::Center, wall, kUnit, wall.walls(), kO, at(3, 0)));
    assert(!sees(Sight::Center, wall, kUnit, wall.walls(), kO, at(2, 0)));
    // The wall itself is visible: you see its face. A wall behind the target does not matter.
    assert(sees(Sight::Center, wall, kUnit, wall.walls(), kO, at(1, 0)));
    assert(sees(Sight::Center, withWalls({at(4, 0)}), kUnit, withWalls({at(4, 0)}).walls(), kO, at(3, 0)));
    // A wall off to the side is no obstacle: (1,1) is at 30 degrees below, the line to (3,0) runs at 0 degrees.
    const Map side = withWalls({at(1, 1)});
    assert(sees(Sight::Center, side, kUnit, side.walls(), kO, at(3, 0)));
    // A hex sees itself and its neighbors even when all of them are walls.
    Map ring = emptyMap();
    for (int d = 0; d < 6; ++d) ring.setWall(neighbor(kO, d), true);
    assert(sees(Sight::Center, ring, kUnit, ring.walls(), kO, kO));
    assert(sees(Sight::Center, ring, kUnit, ring.walls(), kO, neighbor(kO, 2)));
    assert(!sees(Sight::Center, ring, kUnit, ring.walls(), kO, at(2, 0)));
}

void sightMethodsDisagree() {
    // A single wall at (1,0). The line to (2,-1) runs from the center of the unit through the corner it shares with
    // (1,0) and then along the edge between (1,0) and (1,-1). It never enters (1,0), so the exact line is clear.
    // The hex line has to pick (1,0) or (1,-1) for the middle step (a tie) and picks (1,0): blocked.
    const Map one = withWalls({at(1, 0)});
    assert(sees(Sight::Center, one, kUnit, one.walls(), kO, at(2, -1)));
    assert(!sees(Sight::HexLine, one, kUnit, one.walls(), kO, at(2, -1)));
    assert(sees(Sight::Corners, one, kUnit, one.walls(), kO, at(2, -1)));

    // Two walls (1,0) and (1,-1) side by side: the same line slides through the zero-width crack between them.
    // By the rule that touching is not blocking, the center line is clear. Whoever wants a solid wall has to say so.
    const Map pair = withWalls({at(1, 0), at(1, -1)});
    assert(sees(Sight::Center, pair, kUnit, pair.walls(), kO, at(2, -1)));

    // A wall at (2,0) and a target at (4,-1). The center line runs through the wall (at x of the wall's center it
    // is 0.857 above the middle, the hex reaches up to 1). The target's top corner is seen over the wall.
    const Map pillar = withWalls({at(2, 0)});
    assert(!sees(Sight::Center, pillar, kUnit, pillar.walls(), kO, at(4, -1)));
    assert(!sees(Sight::HexLine, pillar, kUnit, pillar.walls(), kO, at(4, -1)));  // the line is (0,0) (1,0) (2,0) (3,-1) (4,-1)
    assert(sees(Sight::Corners, pillar, kUnit, pillar.walls(), kO, at(4, -1)));

    // A wall right next to the unit hides the whole cone behind it, corners included: (3,0) is at most 11 degrees off
    // the axis, the wall covers plus and minus 30.
    assert(!sees(Sight::Corners, one, kUnit, one.walls(), kO, at(3, 0)));
}

void sightField() {
    // Radius 2 around the middle, one wall east of the unit.
    const Map m = withWalls({at(1, 0)});
    const SightField center = computeSight(m, kUnit, kO, 2, Sight::Center);
    const SightField line = computeSight(m, kUnit, kO, 2, Sight::HexLine);
    const SightField corners = computeSight(m, kUnit, kO, 2, Sight::Corners);
    assert(count(m, center.seen, Seen::Outside) == 81 - 19);
    assert(count(m, center.seen, Seen::Visible) + count(m, center.seen, Seen::Hidden) == 19);
    // Center: only the hex straight behind the wall is hidden.
    assert(count(m, center.seen, Seen::Hidden) == 1);
    assert(center.seen[m.index(at(2, 0))] == Seen::Hidden);
    assert(center.seen[m.index(kO)] == Seen::Visible);
    assert(center.seen[m.index(at(1, 0))] == Seen::Visible);
    // Hex line also loses (2,-1) to the tie rule, and (1,1) stays: its line goes through (0,1).
    assert(count(m, line.seen, Seen::Hidden) == 2);
    assert(line.seen[m.index(at(2, -1))] == Seen::Hidden && line.seen[m.index(at(1, 1))] == Seen::Visible);
    assert(count(m, corners.seen, Seen::Hidden) == 1);
    // "differs" marks the answers that are not the center ray's answer.
    int dc = 0;
    int dl = 0;
    int dk = 0;
    for (std::size_t i = 0; i < m.size(); ++i) {
        dc += center.differs[i] != 0 ? 1 : 0;
        dl += line.differs[i] != 0 ? 1 : 0;
        dk += corners.differs[i] != 0 ? 1 : 0;
    }
    assert(dc == 0 && dl == 1 && dk == 0);
    assert(line.differs[m.index(at(2, -1))] != 0);
    // Without walls everything inside the radius is visible in every mode.
    const Map empty = emptyMap();
    for (Sight mode : {Sight::HexLine, Sight::Center, Sight::Corners}) {
        const SightField f = computeSight(empty, kUnit, kO, 3, mode);
        assert(count(empty, f.seen, Seen::Visible) == 37 && count(empty, f.seen, Seen::Hidden) == 0);
    }
    // Radius 0 sees only the unit's own hex; a unit outside the map sees nothing.
    assert(count(empty, computeSight(empty, kUnit, kO, 0, Sight::Center).seen, Seen::Visible) == 1);
    assert(count(empty, computeSight(empty, kUnit, {50, 50}, 3, Sight::Center).seen, Seen::Visible) == 0);
}

void presetLayout() {
    Map m(kPresetCols, kPresetRows);
    const std::optional<Hex> start = loadLayout(m, presetRows());
    assert(start.has_value());
    assert(m.contains(*start) && !m.wall(*start));
    assert(!m.walls().empty());
    // The first frame must show the point of the demo: at 5 points the flood fill reaches clearly fewer hexes than the loop.
    const Reach r = floodFill(m, *start, 5);
    std::size_t reached = 0;
    for (int c : r.cost) reached += c >= 0 ? 1 : 0;
    std::size_t claimed = 0;
    for (const Hex& h : hexesInRange(m, *start, 5)) claimed += m.wall(h) ? 0 : 1;
    assert(reached + 8 <= claimed);

    // A malformed layout is rejected and leaves the map alone.
    Map other(kPresetCols, kPresetRows);
    std::vector<std::string> wrong = presetRows();
    wrong[3].pop_back();
    assert(!loadLayout(other, wrong).has_value());
    assert(other.walls().empty());
    wrong = presetRows();
    wrong.pop_back();
    assert(!loadLayout(other, wrong).has_value());
    std::vector<std::string> noUnit = presetRows();
    for (std::string& row : noUnit) {
        for (char& ch : row) ch = ch == '@' ? '.' : ch;
    }
    assert(!loadLayout(other, noUnit).has_value());
}

}  // namespace

int main() {
    distances();
    pixelsAndCorners();
    rounding();
    pixelRoundTrip();
    mapShape();
    walls();
    rangeLoop();
    floodWithoutWalls();
    floodWithWalls();
    lines();
    segmentAgainstHex();
    sightByCenter();
    sightMethodsDisagree();
    sightField();
    presetLayout();
    std::puts("hex-reichweite-sicht logic: all asserts passed");
    return 0;
}
