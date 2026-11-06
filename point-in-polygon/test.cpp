// Tests for the point-in-polygon logic. Plain asserts on hand-computed cases, no framework, no SDL.
// All coordinates are written in "math" form; the logic does not care which way y points.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <cstdio>

#include "logic.hpp"

using namespace pip;

namespace {

bool evenOdd(const Polygon& poly, Vec2 p) { return inside(poly, p, Rule::EvenOdd); }
bool nonZero(const Polygon& poly, Vec2 p) { return inside(poly, p, Rule::NonZero); }
bool near(double a, double b) { return std::fabs(a - b) < 1e-9; }

const Polygon kSquare{{0, 0}, {4, 0}, {4, 4}, {0, 4}};

Polygon mirrored(const Polygon& poly) {
    Polygon out = poly;
    for (Vec2& v : out) v.x = -v.x;
    return out;
}

void squareInsideOutside() {
    assert(evenOdd(kSquare, {2, 2}));
    assert(nonZero(kSquare, {2, 2}));
    assert(!evenOdd(kSquare, {5, 2}));  // right of the square: ray leaves immediately
    assert(!evenOdd(kSquare, {-1, 2}));  // left of the square: ray enters and leaves, two crossings
    assert(!evenOdd(kSquare, {2, 5}));
    assert(!evenOdd(kSquare, {2, -1}));
    assert(!nonZero(kSquare, {-1, 2}));
}

void crossingsOfSquare() {
    // From (2,2) the ray to the right meets exactly the edge x = 4.
    const auto c = rayCrossings(kSquare, {2, 2});
    assert(c.size() == 1);
    assert(near(c[0].x, 4.0));
    assert(c[0].edge == 1);  // edge 1 runs from vertex 1 (4,0) to vertex 2 (4,4)
    assert(c[0].dir == 1);   // y grows along that edge
    // From (-1,2) it meets x = 0 and x = 4, sorted from left to right with opposite directions.
    const auto d = rayCrossings(kSquare, {-1, 2});
    assert(d.size() == 2);
    assert(near(d[0].x, 0.0) && near(d[1].x, 4.0));
    assert(d[0].dir == -d[1].dir);
    assert(windingNumber(kSquare, {-1, 2}) == 0);
    assert(std::abs(windingNumber(kSquare, {2, 2})) == 1);
}

void crossingXOnADiagonal() {
    // Triangle with a slanted edge from (0,0) to (4,8): at y = 2 it is at x = 1.
    const Polygon tri{{0, 0}, {4, 8}, {-4, 8}};
    const auto c = rayCrossings(tri, {-10, 2});
    assert(c.size() == 2);
    assert(near(c[0].x, -1.0) && near(c[1].x, 1.0));
    assert(evenOdd(tri, {0, 5}));
    assert(!evenOdd(tri, {3, 2}));  // right of the slanted edge at that height
}

void concaveNotch() {
    // U shape. The notch between the arms (x 2 to 3, y 2 to 4) is outside.
    const Polygon u{{0, 0}, {5, 0}, {5, 4}, {3, 4}, {3, 2}, {2, 2}, {2, 4}, {0, 4}};
    assert(evenOdd(u, {1, 3}));    // left arm
    assert(evenOdd(u, {4, 3}));    // right arm
    assert(evenOdd(u, {2.5, 1}));  // below the notch
    assert(!evenOdd(u, {2.5, 3}));  // inside the notch: ray hits x = 3 and x = 5, two crossings
    assert(!evenOdd(u, {2.5, 3.9}));
    assert(nonZero(u, {1, 3}) && !nonZero(u, {2.5, 3}));  // simple polygon: both rules agree
}

void rayThroughVertices() {
    // Diamond with vertices left and right exactly at the height of the ray (y = 0).
    const Polygon d{{0, -2}, {2, 0}, {0, 2}, {-2, 0}};
    assert(evenOdd(d, {0, 0}));   // ray leaves through the right vertex: that must count once
    assert(!evenOdd(d, {-5, 0}));  // ray passes the left vertex and the right vertex: once each, even
    assert(!evenOdd(d, {5, 0}));
    assert(rayCrossings(d, {-5, 0}).size() == 2);
    assert(rayCrossings(d, {0, 0}).size() == 1);
    assert(windingNumber(d, {-5, 0}) == 0);
}

void rayGrazesATip() {
    // The tip touches the ray from above or from below. Both edges at the tip are either counted or not counted
    // together, so the parity stays even and the point outside.
    const Polygon above{{-2, 2}, {0, 0}, {2, 2}};
    const Polygon below{{-2, -2}, {0, 0}, {2, -2}};
    assert(rayCrossings(above, {-5, 0}).size() == 2);  // both edges meet at the tip, both counted
    assert(rayCrossings(below, {-5, 0}).empty());      // neither counted
    assert(!evenOdd(above, {-5, 0}) && !evenOdd(below, {-5, 0}));
    assert(!nonZero(above, {-5, 0}) && !nonZero(below, {-5, 0}));
}

void horizontalEdgeOnTheRay() {
    // L shape: a 4x2 slab with a 2x2 block on its left side. The edge from (4,2) to (2,2) is horizontal at y = 2.
    const Polygon l{{0, 0}, {4, 0}, {4, 2}, {2, 2}, {2, 4}, {0, 4}};
    assert(rayCrossings(l, {-1, 2}).size() == 2);  // the horizontal edge never counts
    assert(!evenOdd(l, {-1, 2}));
    assert(evenOdd(l, {1, 2}));   // on the seam between slab and block, inside
    assert(!evenOdd(l, {3, 2}));  // on the horizontal edge (max y of the slab): outside by the rule
    assert(!evenOdd(l, {3, 3}));  // above the slab, right of the block
    assert(evenOdd(l, {1, 3}));
    assert(evenOdd(l, {3, 1}));
}

void pointOnBoundary() {
    // Rule: edges are half open, a point on the min-x or min-y side is inside, on the max-x or max-y side outside.
    assert(evenOdd(kSquare, {0, 2}));    // left edge
    assert(!evenOdd(kSquare, {4, 2}));   // right edge
    assert(evenOdd(kSquare, {2, 0}));    // bottom edge (min y)
    assert(!evenOdd(kSquare, {2, 4}));   // top edge (max y)
    assert(evenOdd(kSquare, {0, 0}));    // min corner
    assert(!evenOdd(kSquare, {4, 0}));   // corner on the max-x side
    assert(!evenOdd(kSquare, {0, 4}));   // corner on the max-y side
    assert(!evenOdd(kSquare, {4, 4}));   // max corner
    // Same for the winding number.
    assert(nonZero(kSquare, {0, 2}) && !nonZero(kSquare, {4, 2}));
    assert(nonZero(kSquare, {2, 0}) && !nonZero(kSquare, {2, 4}));
}

void sharedEdges() {
    // Two polygons that share an edge never both claim a point on it, whatever the slant of the edge. On the inner part of the shared edge exactly one does.
    const Polygon left{{0, 0}, {4, 0}, {4, 4}, {0, 4}};
    const Polygon right{{4, 0}, {8, 0}, {8, 4}, {4, 4}};
    const Polygon lower{{0, 0}, {4, 0}, {4, 4}};  // the square cut along its diagonal
    const Polygon upper{{0, 0}, {4, 4}, {0, 4}};
    for (double t = 0.0; t < 4.0; t += 0.5) {  // t = 4 is the top corner, on the outer boundary of both
        assert(evenOdd(left, {4, t}) != evenOdd(right, {4, t}));    // exactly one of them
        assert(evenOdd(lower, {t, t}) != evenOdd(upper, {t, t}));
    }
    assert(!evenOdd(left, {4, 4}) && !evenOdd(right, {4, 4}));      // never both
}

void degeneratePolygons() {
    assert(!evenOdd({}, {0, 0}));
    assert(!evenOdd({{0, 0}, {4, 4}}, {1, 1}));
    assert(rayCrossings({{0, 0}, {4, 4}}, {-1, 1}).empty());
    assert(!nonZero({{0, 0}}, {0, 0}));
}

void pentagramCenter() {
    // The pentagram {5/2}: the middle pentagon is covered twice. Even-odd calls it outside, nonzero inside.
    const Polygon star = starPreset();
    assert(star.size() == 5);
    double cx = 0.0, cy = 0.0;
    for (const Vec2& v : star) { cx += v.x; cy += v.y; }
    cx /= 5.0; cy /= 5.0;
    assert(!evenOdd(star, {cx, cy}));
    assert(nonZero(star, {cx, cy}));
    assert(std::abs(windingNumber(star, {cx, cy})) == 2);
    assert(rayCrossings(star, {cx, cy}).size() == 2);
    // A point in the top tip is covered once, so both rules say inside. The tip is the vertex with the smallest y.
    std::size_t top = 0;
    for (std::size_t i = 1; i < star.size(); ++i) if (star[i].y < star[top].y) top = i;
    const Vec2 tip{cx, star[top].y + (cy - star[top].y) * 0.1};
    assert(evenOdd(star, tip) && nonZero(star, tip));
    assert(std::abs(windingNumber(star, tip)) == 1);
    // Far away: outside under both.
    assert(!evenOdd(star, {cx + 1000, cy}) && !nonZero(star, {cx + 1000, cy}));
}

void concavePresetNotch() {
    const Polygon poly = concavePreset();
    assert(poly.size() >= 6);
    // Both rules agree on a simple polygon, everywhere on a grid (offset so no sample lies on an edge).
    int insideCount = 0;
    for (double y = 0.37; y < 540; y += 7.0) {
        for (double x = 0.41; x < 960; x += 7.0) {
            assert(evenOdd(poly, {x, y}) == nonZero(poly, {x, y}));
            insideCount += evenOdd(poly, {x, y}) ? 1 : 0;
        }
    }
    assert(insideCount > 500);  // the preset is not empty
}

void mirrorGivesTheSameAnswer() {
    // A ray to the left is the mirror image of a ray to the right. Evaluating the mirrored polygon is a second,
    // independent way to get the answer for every sample that is not on the boundary.
    for (const Polygon& poly : {concavePreset(), starPreset()}) {
        const Polygon m = mirrored(poly);
        for (double y = 0.37; y < 540; y += 5.0) {
            for (double x = 0.41; x < 960; x += 5.0) {
                for (Rule rule : {Rule::EvenOdd, Rule::NonZero}) {
                    const bool a = inside(poly, {x, y}, rule);
                    const bool b = inside(m, {-x, y}, rule);
                    assert(a == b);
                }
            }
        }
    }
}

void scanSpansMatchPointTest() {
    // The filled area is drawn with scanline spans. A sample is inside a span exactly when the point test says so.
    for (const Polygon& poly : {concavePreset(), starPreset(), kSquare}) {
        for (Rule rule : {Rule::EvenOdd, Rule::NonZero}) {
            for (double y = 0.37; y < 540; y += 6.0) {
                const auto spans = scanSpans(poly, y, rule);
                for (double x = 0.41; x < 960; x += 6.0) {
                    bool inSpan = false;
                    for (const Span& s : spans) inSpan = inSpan || (x >= s.x0 && x < s.x1);
                    assert(inSpan == inside(poly, {x, y}, rule));
                }
            }
        }
    }
    // Hand case: the square at y = 2 is one span from 0 to 4.
    const auto s = scanSpans(kSquare, 2.0, Rule::EvenOdd);
    assert(s.size() == 1 && near(s[0].x0, 0.0) && near(s[0].x1, 4.0));
    assert(scanSpans(kSquare, 9.0, Rule::EvenOdd).empty());
}

void snapping() {
    assert(near(snapToVertexRow(kSquare, 0.3, 0.5), 0.0));
    assert(near(snapToVertexRow(kSquare, 3.8, 0.5), 4.0));
    assert(near(snapToVertexRow(kSquare, 2.0, 0.5), 2.0));  // too far from any vertex row, unchanged
    // Snapped onto the bottom row the ray runs along the bottom edge and the right edge's start: nothing odd happens.
    const Vec2 p{-1, snapToVertexRow(kSquare, 0.2, 0.5)};
    assert(!evenOdd(kSquare, p));
}

void presetHomes() {
    // The first frame must already show the interesting case.
    assert(evenOdd(concavePreset(), concaveHome()));
    assert(!evenOdd(starPreset(), starHome()) && nonZero(starPreset(), starHome()));
}

void editing() {
    Polygon poly = kSquare;
    // Nearest vertex within a radius.
    assert(nearestVertex(poly, {0.5, 0.5}, 1.0) == std::optional<std::size_t>(0));
    assert(nearestVertex(poly, {3.6, 3.8}, 1.0) == std::optional<std::size_t>(2));
    assert(!nearestVertex(poly, {2, 2}, 1.0).has_value());
    // Nearest edge: a point above the middle of the bottom edge (edge 0, from vertex 0 to vertex 1).
    const auto hit = nearestEdge(poly, {2, 0.4}, 1.0);
    assert(hit.has_value());
    assert(hit->index == 0);
    assert(near(hit->point.x, 2.0) && near(hit->point.y, 0.0) && near(hit->distance, 0.4));
    // The closing edge from the last vertex back to vertex 0 is edge 3.
    const auto closing = nearestEdge(poly, {0.3, 2}, 1.0);
    assert(closing.has_value() && closing->index == 3 && near(closing->point.y, 2.0));
    // Past the end of an edge the nearest point is clamped to the vertex.
    const auto beyond = nearestEdge(poly, {6, 0}, 3.0);
    assert(beyond.has_value() && beyond->index == 0 && near(beyond->point.x, 4.0) && near(beyond->distance, 2.0));
    assert(!nearestEdge(poly, {2, 2}, 1.0).has_value());
    // Insert after edge 0: the new vertex lands at index 1.
    const std::size_t at = insertVertex(poly, 0, {2, 0});
    assert(at == 1 && poly.size() == 5 && near(poly[1].x, 2.0) && near(poly[2].x, 4.0));
    // Insert on the closing edge: appended at the end.
    const std::size_t last = poly.size() - 1;  // the closing edge runs from the last vertex back to vertex 0
    assert(insertVertex(poly, last, {0, 2}) == last + 1);
    assert(poly.size() == 6 && near(poly.back().x, 0.0) && near(poly.back().y, 2.0));
    // Removing keeps at least three vertices.
    assert(removeVertex(poly, 0));
    assert(poly.size() == 5);
    assert(near(poly[0].x, 2.0));  // the old vertex 1 moved up
    Polygon tri{{0, 0}, {4, 0}, {0, 4}};
    assert(!removeVertex(tri, 1));
    assert(tri.size() == 3);
    assert(!removeVertex(poly, 99));
}

}  // namespace

int main() {
    squareInsideOutside();
    crossingsOfSquare();
    crossingXOnADiagonal();
    concaveNotch();
    rayThroughVertices();
    rayGrazesATip();
    horizontalEdgeOnTheRay();
    pointOnBoundary();
    sharedEdges();
    degeneratePolygons();
    pentagramCenter();
    concavePresetNotch();
    mirrorGivesTheSameAnswer();
    scanSpansMatchPointTest();
    snapping();
    presetHomes();
    editing();
    std::puts("point-in-polygon logic: all asserts passed");
    return 0;
}
