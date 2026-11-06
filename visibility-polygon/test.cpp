// Tests for the visibility polygon logic. Plain asserts on hand-computed cases, no framework, no SDL.
// Coordinates are "math" form (y up); the logic does not care which way y points on screen.
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "logic.hpp"

using namespace vis;

namespace {

bool near(double a, double b, double tol = 1e-9) { return std::fabs(a - b) < tol; }

std::vector<Segment> room() {
    std::vector<Segment> s;
    addBox(s, 0, 0, 10, 10);
    return s;
}

// Room 10 x 10 with a pillar [3,4] x [4,6]. The light at (1,5) looks straight at the pillar's near face.
std::vector<Segment> pillarRoom() {
    std::vector<Segment> s = room();
    addBox(s, 3, 4, 1, 2);
    return s;
}

constexpr Vec2 kLight{1, 5};

Options withMethod(Method m) {
    Options o;
    o.method = m;
    return o;
}

// ---- castRay ----------------------------------------------------------------------------------------------------

void castRayBasics() {
    const std::vector<Segment> wall{{{5, -3}, {5, 3}}};
    const auto hit = castRay({0, 0}, {1, 0}, wall);
    assert(hit && near(hit->t, 5.0) && near(hit->point.x, 5.0) && near(hit->point.y, 0.0));
    // t is measured in lengths of dir: a direction of length 2 reaches the wall at t = 2.5.
    const auto fast = castRay({0, 0}, {2, 0}, wall);
    assert(fast && near(fast->t, 2.5));
    assert(!castRay({0, 0}, {-1, 0}, wall));  // wall behind the ray
    assert(!castRay({0, 0}, {0, 1}, wall));   // parallel, never meets
    assert(!castRay({0, 4}, {1, 0}, wall));   // passes above the end of the wall (u > 1)
    assert(!castRay({0, -4}, {1, 0}, wall));  // passes below the start (u < 0)
    assert(!castRay({0, 0}, {0, 0}, wall));   // no direction, no division by zero
}

void castRayGrazesAnEndpoint() {
    const std::vector<Segment> wall{{{5, 0}, {5, 3}}};
    const auto hit = castRay({0, 0}, {1, 0}, wall);  // touches the lower end exactly
    assert(hit && near(hit->t, 5.0));
}

void castRayAlongAWallIsParallel() {
    const std::vector<Segment> wall{{{2, 0}, {5, 0}}};
    assert(!castRay({0, 0}, {1, 0}, wall));  // collinear: skipped, the endpoints are handled by their own rays
}

void castRayPicksTheNearest() {
    const std::vector<Segment> walls{{{8, -3}, {8, 3}}, {{5, -3}, {5, 3}}, {{9, -3}, {9, 3}}};
    const auto hit = castRay({0, 0}, {1, 0}, walls);
    assert(hit && near(hit->t, 5.0));
}

void castRayThroughAClosedCorner() {
    // A V shaped corner at (6,4): two walls that share an endpoint. A ray that hits the shared point exactly must
    // not slip between them.
    const std::vector<Segment> v{{{4, 6}, {6, 4}}, {{6, 4}, {8, 6}}};
    const auto hit = castRay({6, 0}, {0, 1}, v);
    assert(hit && near(hit->t, 4.0) && near(hit->point.y, 4.0));
    // The same through cos/sin of the angle, which is only approximately straight up.
    const double up = std::atan2(1.0, 0.0);
    const auto viaAngle = castRay({6, 0}, {std::cos(up), std::sin(up)}, v);
    assert(viaAngle && near(viaAngle->point.y, 4.0, 1e-6));
}

// ---- helpers ------------------------------------------------------------------------------------------------------

void containsAndArea() {
    const std::vector<Vec2> sq{{0, 0}, {4, 0}, {4, 4}, {0, 4}};
    assert(near(area(sq), 16.0));
    assert(contains(sq, {2, 2}));
    assert(!contains(sq, {5, 2}));
    assert(!contains(sq, {-1, 2}));
    assert(!contains(sq, {2, 5}));
    const std::vector<Vec2> two{{0, 0}, {1, 1}};
    assert(near(area(two), 0.0) && !contains(two, {0.5, 0.5}));
}

void lineOfSightBasics() {
    const auto s = pillarRoom();
    assert(lineOfSight(kLight, {2, 5}, s));        // in front of the pillar
    assert(!lineOfSight(kLight, {8, 5}, s));       // straight through the pillar
    assert(lineOfSight(kLight, {9.5, 0.25}, s));   // passes below the pillar's lower corner
    assert(!lineOfSight(kLight, {9.5, 7.0}, s));   // slope 2/8.5 = 0.235 < 0.5: behind the pillar
    assert(lineOfSight({1, 1}, {1, 1}, s));        // zero length sees itself
}

void cornerEditing() {
    std::vector<Segment> s;
    addBox(s, 10, 10, 20, 20);  // corner (10,10) is shared by two segments
    addBox(s, 100, 100, 5, 5);
    const auto c = nearestCorner(s, s.size(), {12, 11}, 5.0);
    assert(c && c->x == 10 && c->y == 10);
    assert(!nearestCorner(s, s.size(), {50, 50}, 5.0));   // nothing within reach
    assert(!nearestCorner(s, 4, {101, 101}, 5.0));        // second box is not movable
    assert(moveCorner(s, {10, 10}, {12, 14}) == 2);       // both segments follow
    assert(s[0].a.x == 12 && s[0].a.y == 14);
    assert(s[3].b.x == 12 && s[3].b.y == 14);
    assert(moveCorner(s, {10, 10}, {0, 0}) == 0);         // the old position is gone
}

// ---- the visibility polygon ---------------------------------------------------------------------------------------

void emptyRoom() {
    const auto s = room();
    Result r;
    compute({5, 5}, s, withMethod(Method::CornerRays), r);
    assert(r.rays.size() == 12);  // 4 corners, 3 rays each
    assert(r.polygon.size() == 12);
    assert(near(area(r.polygon), 100.0, 1e-6));
    compute({5, 5}, s, withMethod(Method::ExactCorners), r);
    assert(r.rays.size() == 4);
    assert(near(area(r.polygon), 100.0, 1e-6));  // nothing hides here, so nothing can go wrong
}

void rayKindsAndOrder() {
    const auto s = room();
    Result r;
    compute({5, 5}, s, withMethod(Method::CornerRays), r);
    int corner = 0;
    int before = 0;
    int after = 0;
    for (std::size_t i = 0; i < r.rays.size(); ++i) {
        if (r.rays[i].kind == RayKind::Corner) ++corner;
        if (r.rays[i].kind == RayKind::Before) ++before;
        if (r.rays[i].kind == RayKind::After) ++after;
        if (i > 0) assert(r.rays[i].angle > r.rays[i - 1].angle);  // sorted by angle, strictly
    }
    assert(corner == 4 && before == 4 && after == 4);
    // Every Before sits just below a Corner, every After just above.
    for (std::size_t i = 0; i < r.rays.size(); ++i) {
        if (r.rays[i].kind == RayKind::Corner) {
            // The neighbours on both sides are the offset rays, except at the ends of the sweep where they wrap.
            if (i > 0) assert(r.rays[i - 1].kind == RayKind::Before);
            if (i + 1 < r.rays.size()) assert(r.rays[i + 1].kind == RayKind::After);
        }
    }
}

void pillarCornerRaysAreExact() {
    // Hand calculation, light (1,5): the shadow is the wedge |y - 5| <= (x - 1) / 2 for x >= 3, clipped to the room,
    // that is a trapezoid with half heights 1 (at x = 3) and 4.5 (at x = 10), area (2 + 9) / 2 * 7 = 38.5.
    const auto s = pillarRoom();
    Result r;
    compute(kLight, s, withMethod(Method::CornerRays), r);
    assert(r.rays.size() == 24);  // 8 distinct corners
    assert(near(area(r.polygon), 100.0 - 38.5, 0.01));
    assert(contains(r.polygon, {2, 5}));
    assert(!contains(r.polygon, {8, 5}));
    assert(contains(r.polygon, {9.5, 0.25}));  // lit, just below the shadow
    assert(contains(r.polygon, {9.5, 9.75}));
}

void pillarExactCornersCutsTheLitRegion() {
    // Without the offset rays the polygon goes from the room corner (10,0) straight to the pillar corner (3,4) and
    // from (3,6) to (10,10). Shoelace on (0,0) (10,0) (3,4) (3,6) (10,10) (0,10) gives 58, two triangles of 1.75 are missing.
    const auto s = pillarRoom();
    Result r;
    compute(kLight, s, withMethod(Method::ExactCorners), r);
    assert(r.rays.size() == 8);
    assert(near(area(r.polygon), 58.0, 1e-9));
    assert(lineOfSight(kLight, {9.5, 0.5}, s));
    assert(!contains(r.polygon, {9.5, 0.5}));  // lit in reality, lost by the naive polygon: the chord (10,0)-(3,4) is at y = 0.29 there
    assert(contains(r.polygon, {9.5, 0.25}));  // below the chord it is inside
    assert(contains(r.polygon, {2, 5}));
}

void polygonAgreesWithLineOfSight() {
    // An independent answer: a point is inside the polygon exactly when the straight segment to the light is free.
    const auto s = pillarRoom();
    Result r;
    compute(kLight, s, withMethod(Method::CornerRays), r);
    int lit = 0;
    int dark = 0;
    int mismatches = 0;
    for (int i = 0; i < 16; ++i) {
        for (int j = 0; j < 18; ++j) {
            const Vec2 p{0.37 + 0.61 * i, 0.23 + 0.53 * j};  // odd steps: no sample sits on a wall or on a shadow edge
            if (p.x > 3 - 0.2 && p.x < 4 + 0.2 && p.y > 4 - 0.2 && p.y < 6 + 0.2) continue;  // in or at the pillar
            if (lineOfSight(kLight, p, s)) ++lit; else ++dark;
            if (lineOfSight(kLight, p, s) != contains(r.polygon, p)) ++mismatches;
        }
    }
    assert(lit > 50 && dark > 20);
    assert(mismatches == 0);

    compute(kLight, s, withMethod(Method::ExactCorners), r);
    int naiveMismatches = 0;
    for (int i = 0; i < 16; ++i) {
        for (int j = 0; j < 18; ++j) {
            const Vec2 p{0.37 + 0.61 * i, 0.23 + 0.53 * j};
            if (p.x > 3 - 0.2 && p.x < 4 + 0.2 && p.y > 4 - 0.2 && p.y < 6 + 0.2) continue;
            if (lineOfSight(kLight, p, s) != contains(r.polygon, p)) ++naiveMismatches;
        }
    }
    assert(naiveMismatches > 0);
}

void uniformRaysConvergeButNeverArrive() {
    const auto s = pillarRoom();
    Result r;
    Options o = withMethod(Method::Uniform);
    o.uniformCount = 36;
    compute(kLight, s, o, r);
    assert(r.rays.size() == 36);
    const double err36 = std::fabs(area(r.polygon) - 61.5);
    o.uniformCount = 720;
    compute(kLight, s, o, r);
    assert(r.rays.size() == 720);
    const double err720 = std::fabs(area(r.polygon) - 61.5);
    assert(err720 < err36);
    assert(err720 > 0.01);  // still not exact: the silhouette angle is no multiple of 0.5 degrees
    assert(err720 < 2.0);
    o.uniformCount = 0;  // clamped, never an empty or endless loop
    compute(kLight, s, o, r);
    assert(r.rays.size() >= 3);
}

void rayThroughASharedCornerDoesNotLeak() {
    // The V corner at (6,4) as part of a room: a light straight below it must see the corner, not the far wall.
    std::vector<Segment> s = room();
    s.push_back({{4, 6}, {6, 4}});
    s.push_back({{6, 4}, {8, 6}});
    Result r;
    compute({6, 0.5}, s, withMethod(Method::ExactCorners), r);
    bool found = false;
    for (const Ray& ray : r.rays) {
        if (std::fabs(ray.angle - std::atan2(1.0, 0.0)) < 1e-12) {
            found = true;
            assert(near(ray.point.y, 4.0, 1e-6));  // not 10
        }
    }
    assert(found);
}

void rayGrazingAWallEndStopsThere() {
    // The tip (1,9) of a wedge, seen from (9,9) along the row y = 9. cos/sin of the angle are only approximately
    // straight left, so the ray misses the tip by a rounding error. Without slack on the wall ends it flies on to the
    // room wall at x = 0 and the polygon has a hole in front of the tip.
    std::vector<Segment> s = room();
    s.push_back({{2, 3}, {1, 9}});
    s.push_back({{1, 9}, {3, 6}});
    Result r;
    compute({9, 9}, s, withMethod(Method::ExactCorners), r);
    bool found = false;
    for (const Ray& ray : r.rays) {
        if (std::fabs(ray.angle - kPi) < 1e-9) {
            found = true;
            assert(near(ray.dist, 8.0, 1e-6));
            assert(near(ray.point.x, 1.0, 1e-6));
        }
    }
    assert(found);
}

void lightOnACornerStaysFinite() {
    const auto s = pillarRoom();
    Result r;
    compute({3, 4}, s, withMethod(Method::CornerRays), r);  // exactly on the pillar's lower left corner
    assert(r.polygon.size() > 3);
    for (const Vec2& p : r.polygon) assert(std::isfinite(p.x) && std::isfinite(p.y));
    compute({5, 5}, std::vector<Segment>{}, withMethod(Method::CornerRays), r);  // no walls at all
    assert(r.polygon.empty() && r.rays.empty());
}

void flashlightCone() {
    const auto s = room();
    Result r;
    Options o = withMethod(Method::CornerRays);
    o.halfAngle = kPi / 4.0;
    o.aim = 0.0;  // looking right: the cone spans from corner (10,0) to corner (10,10), a triangle of 0.5 * 10 * 5 = 25
    compute({5, 5}, s, o, r);
    assert(!r.full && r.rimBegin == 1);
    assert(r.polygon[0].x == 5 && r.polygon[0].y == 5);  // the light itself is the first vertex
    assert(near(area(r.polygon), 25.0, 1e-6));
    assert(contains(r.polygon, {8, 5}));
    assert(!contains(r.polygon, {5, 8}));
    assert(!contains(r.polygon, {2, 5}));
    // The same cone aimed up and aimed left; left crosses the +-pi seam of atan2.
    o.aim = kPi / 2.0;
    compute({5, 5}, s, o, r);
    assert(near(area(r.polygon), 25.0, 1e-6));
    assert(contains(r.polygon, {5, 8}) && !contains(r.polygon, {8, 5}));
    o.aim = kPi;
    compute({5, 5}, s, o, r);
    assert(near(area(r.polygon), 25.0, 1e-6));
    assert(contains(r.polygon, {2, 5}) && !contains(r.polygon, {8, 5}));
    o.aim = -kPi + 0.3;  // an aim outside (-pi, pi] after adding up wheel ticks is still fine
    compute({5, 5}, s, o, r);
    assert(contains(r.polygon, {2, 5}) && !contains(r.polygon, {8, 5}));
    // A cone of 180 degrees or more is the full circle.
    o.halfAngle = kPi;
    compute({5, 5}, s, o, r);
    assert(r.full && r.rimBegin == 0);
    assert(near(area(r.polygon), 100.0, 1e-6));
}

void coneStopsAtAPillar() {
    const auto s = pillarRoom();
    Result r;
    Options o = withMethod(Method::CornerRays);
    o.halfAngle = 0.3;
    o.aim = 0.0;
    // The cone is narrower than the pillar's silhouette (atan(0.5) = 0.46 rad), so it only reaches the near face:
    // a triangle with its base on x = 3 (distance 2) and half width 2 * tan(0.3).
    compute(kLight, s, o, r);
    assert(r.polygon.size() == 3);
    assert(near(area(r.polygon), 4.0 * std::tan(0.3), 1e-6));
    assert(contains(r.polygon, {2.5, 5}));
    assert(!contains(r.polygon, {8, 5}));    // behind the pillar
    assert(!contains(r.polygon, {9, 0.8}));  // lit in reality (lineOfSight), but outside the cone: slope -0.525 is 0.48 rad
    assert(lineOfSight(kLight, {9, 0.8}, s));
}

}  // namespace

int main() {
    castRayBasics();
    castRayGrazesAnEndpoint();
    castRayAlongAWallIsParallel();
    castRayPicksTheNearest();
    castRayThroughAClosedCorner();
    containsAndArea();
    lineOfSightBasics();
    cornerEditing();
    emptyRoom();
    rayKindsAndOrder();
    pillarCornerRaysAreExact();
    pillarExactCornersCutsTheLitRegion();
    polygonAgreesWithLineOfSight();
    uniformRaysConvergeButNeverArrive();
    rayThroughASharedCornerDoesNotLeak();
    rayGrazingAWallEndStopsThere();
    lightOnACornerStaysFinite();
    flashlightCone();
    coneStopsAtAPillar();
    std::puts("visibility-polygon logic ok");
    return 0;
}
