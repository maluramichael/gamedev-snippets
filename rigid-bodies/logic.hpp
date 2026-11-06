// 2D rigid body physics without SDL: circles and oriented boxes, impulse based resolution.
//
// World units are meters, y points down like on the screen (gravity is +y). The step is meant to be called with a
// fixed dt (see the update-loop and fixed-timestep demos). One step is:
//   1. integrate forces (gravity) into velocities
//   2. find contacts (collide), carry impulses over from the last step (warm starting)
//   3. solve the contacts with sequential impulses, a few iterations over all of them
//   4. integrate velocities into positions
// The box-box contact generation (separating axis test, reference face, incident edge, clipping to at most two
// points) follows the approach of Box2D Lite by Erin Catto.
#pragma once

#include <algorithm>
#include <cmath>
#include <map>
#include <utility>
#include <vector>

namespace rb {

constexpr float kGravity = 9.81F;

// ---- small vector math ----

struct Vec2 {
    float x = 0.0F;
    float y = 0.0F;
};
inline Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
inline Vec2 operator-(Vec2 a) { return {-a.x, -a.y}; }
inline Vec2 operator*(Vec2 a, float s) { return {a.x * s, a.y * s}; }
inline Vec2 operator*(float s, Vec2 a) { return {a.x * s, a.y * s}; }
inline float dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
inline float cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }
inline Vec2 cross(float s, Vec2 v) { return {-s * v.y, s * v.x}; }  // angular velocity s at offset v => velocity
inline float length(Vec2 a) { return std::sqrt(dot(a, a)); }

struct Rot {
    float c;
    float s;
};
inline Rot rot(float angle) { return {std::cos(angle), std::sin(angle)}; }
inline Vec2 mul(Rot r, Vec2 v) { return {r.c * v.x - r.s * v.y, r.s * v.x + r.c * v.y}; }
inline Vec2 mulT(Rot r, Vec2 v) { return {r.c * v.x + r.s * v.y, -r.s * v.x + r.c * v.y}; }

// ---- bodies ----

enum class Shape { Circle, Box };

struct Body {
    Shape shape = Shape::Circle;
    Vec2 pos;
    Vec2 vel;
    float angle = 0.0F;
    float angVel = 0.0F;
    float radius = 0.0F;  // circle
    Vec2 half;            // box: half extents in the body frame
    float invMass = 0.0F;     // 0 = static, infinite mass
    float invInertia = 0.0F;

    bool isStatic() const { return invMass == 0.0F; }
    float mass() const { return invMass == 0.0F ? 0.0F : 1.0F / invMass; }
    /// Radius of a circle around the body that contains it, for the cheap broad phase.
    float boundRadius() const { return shape == Shape::Circle ? radius : length(half); }

    // Density is 1, so the mass is the area.
    static Body circle(Vec2 p, float r, bool isStatic = false) {
        Body b;
        b.shape = Shape::Circle;
        b.pos = p;
        b.radius = r;
        if (!isStatic) {
            const float m = 3.14159265F * r * r;
            b.invMass = 1.0F / m;
            b.invInertia = 1.0F / (0.5F * m * r * r);
        }
        return b;
    }

    static Body box(Vec2 p, Vec2 half, float angle = 0.0F, bool isStatic = false) {
        Body b;
        b.shape = Shape::Box;
        b.pos = p;
        b.half = half;
        b.angle = angle;
        if (!isStatic) {
            const float w = 2.0F * half.x;
            const float h = 2.0F * half.y;
            const float m = w * h;
            b.invMass = 1.0F / m;
            b.invInertia = 12.0F / (m * (w * w + h * h));
        }
        return b;
    }
};

/// Is the world point p inside the body?
inline bool contains(const Body& b, Vec2 p) {
    if (b.shape == Shape::Circle) return length(p - b.pos) <= b.radius;
    const Vec2 local = mulT(rot(b.angle), p - b.pos);
    return std::fabs(local.x) <= b.half.x && std::fabs(local.y) <= b.half.y;
}

// ---- collision detection ----

struct Contact {
    Vec2 position;           // world point on the surface
    float separation = 0.0F;  // negative = overlap depth
    // Solver data. pn and pt are the accumulated impulses and survive from step to step (warm starting).
    float pn = 0.0F;
    float pt = 0.0F;
    float massNormal = 0.0F;
    float massTangent = 0.0F;
    float targetSpeed = 0.0F;  // wanted speed along the normal after solving: bounce or position correction
    Vec2 r1;
    Vec2 r2;
};

/// Result of colliding body a with body b. The normal points from a to b. Up to two contact points.
struct Manifold {
    Vec2 normal;
    int count = 0;
    Contact contacts[2];
};

namespace detail {

struct CircleBox {
    bool hit = false;
    Vec2 normal;  // from the box to the circle
    float depth = 0.0F;
    Vec2 point;  // on the box surface
};

inline CircleBox circleVsBox(const Body& circle, const Body& box) {
    CircleBox out;
    const Rot r = rot(box.angle);
    const Vec2 local = mulT(r, circle.pos - box.pos);
    const Vec2 clamped{std::clamp(local.x, -box.half.x, box.half.x), std::clamp(local.y, -box.half.y, box.half.y)};
    const Vec2 d = local - clamped;
    const float dist2 = dot(d, d);
    Vec2 normalLocal;
    Vec2 pointLocal;
    if (dist2 > 0.0F) {
        // The center is outside the box: the closest box point is on a face or a corner.
        const float dist = std::sqrt(dist2);
        if (dist >= circle.radius) return out;
        normalLocal = d * (1.0F / dist);
        pointLocal = clamped;
        out.depth = circle.radius - dist;
    } else {
        // The center is inside the box: leave through the nearest face.
        const float dx = box.half.x - std::fabs(local.x);
        const float dy = box.half.y - std::fabs(local.y);
        if (dx < dy) {
            const float sign = local.x < 0.0F ? -1.0F : 1.0F;
            normalLocal = {sign, 0.0F};
            pointLocal = {sign * box.half.x, local.y};
            out.depth = circle.radius + dx;
        } else {
            const float sign = local.y < 0.0F ? -1.0F : 1.0F;
            normalLocal = {0.0F, sign};
            pointLocal = {local.x, sign * box.half.y};
            out.depth = circle.radius + dy;
        }
    }
    out.hit = true;
    out.normal = mul(r, normalLocal);
    out.point = box.pos + mul(r, pointLocal);
    return out;
}

inline Manifold circleCircle(const Body& a, const Body& b) {
    Manifold m;
    const Vec2 d = b.pos - a.pos;
    const float dist = length(d);
    const float radii = a.radius + b.radius;
    if (dist >= radii) return m;
    m.normal = dist > 1e-6F ? d * (1.0F / dist) : Vec2{0.0F, 1.0F};
    m.count = 1;
    m.contacts[0].separation = dist - radii;
    // Middle of the overlap: a's surface is at radius ra, b's surface at dist - rb.
    m.contacts[0].position = a.pos + m.normal * (0.5F * (a.radius + dist - b.radius));
    return m;
}

struct ClipPoint {
    Vec2 v;
};

/// Keeps the part of the segment in with dot(normal, v) <= offset.
inline int clipSegment(ClipPoint out[2], const ClipPoint in[2], Vec2 normal, float offset) {
    int n = 0;
    const float d0 = dot(normal, in[0].v) - offset;
    const float d1 = dot(normal, in[1].v) - offset;
    if (d0 <= 0.0F) out[n++] = in[0];
    if (d1 <= 0.0F) out[n++] = in[1];
    if (d0 * d1 < 0.0F) {
        const float t = d0 / (d0 - d1);
        out[n++] = ClipPoint{in[0].v + (in[1].v - in[0].v) * t};
    }
    return n;
}

/// The edge of `box` that faces most against `referenceNormal` (a world direction).
inline void incidentEdge(ClipPoint edge[2], const Body& box, Vec2 referenceNormal) {
    const Rot r = rot(box.angle);
    const Vec2 n = -mulT(r, referenceNormal);
    const Vec2 h = box.half;
    Vec2 c0;
    Vec2 c1;
    if (std::fabs(n.x) > std::fabs(n.y)) {
        if (n.x > 0.0F) {
            c0 = {h.x, -h.y};
            c1 = {h.x, h.y};
        } else {
            c0 = {-h.x, h.y};
            c1 = {-h.x, -h.y};
        }
    } else {
        if (n.y > 0.0F) {
            c0 = {h.x, h.y};
            c1 = {-h.x, h.y};
        } else {
            c0 = {-h.x, -h.y};
            c1 = {h.x, -h.y};
        }
    }
    edge[0].v = box.pos + mul(r, c0);
    edge[1].v = box.pos + mul(r, c1);
}

inline Manifold boxBox(const Body& a, const Body& b) {
    Manifold m;
    const Rot ra = rot(a.angle);
    const Rot rb = rot(b.angle);
    const Vec2 d = b.pos - a.pos;
    const Vec2 dA = mulT(ra, d);  // center distance seen from a's frame
    const Vec2 dB = mulT(rb, d);  // and from b's frame
    const Rot rel = rot(b.angle - a.angle);
    const float ac = std::fabs(rel.c);
    const float as = std::fabs(rel.s);
    // Half extent of one box along the axes of the other.
    const Vec2 extentBinA{ac * b.half.x + as * b.half.y, as * b.half.x + ac * b.half.y};
    const Vec2 extentAinB{ac * a.half.x + as * a.half.y, as * a.half.x + ac * a.half.y};

    // Separating axis test on the four face normals. Separation > 0 along an axis = a gap = no contact.
    const Vec2 sepA{std::fabs(dA.x) - a.half.x - extentBinA.x, std::fabs(dA.y) - a.half.y - extentBinA.y};
    if (sepA.x >= 0.0F || sepA.y >= 0.0F) return m;
    const Vec2 sepB{std::fabs(dB.x) - extentAinB.x - b.half.x, std::fabs(dB.y) - extentAinB.y - b.half.y};
    if (sepB.x >= 0.0F || sepB.y >= 0.0F) return m;

    // The axis with the least overlap (largest negative separation) is the contact normal. a's axes win near ties
    // so the choice does not flicker from step to step.
    enum class Axis { FaceAX, FaceAY, FaceBX, FaceBY };
    Axis axis = Axis::FaceAX;
    float separation = sepA.x;
    Vec2 normal = dA.x > 0.0F ? Vec2{ra.c, ra.s} : Vec2{-ra.c, -ra.s};
    constexpr float kRelativeTol = 0.95F;
    constexpr float kAbsoluteTol = 0.01F;
    if (sepA.y > kRelativeTol * separation + kAbsoluteTol * a.half.y) {
        axis = Axis::FaceAY;
        separation = sepA.y;
        normal = dA.y > 0.0F ? Vec2{-ra.s, ra.c} : Vec2{ra.s, -ra.c};
    }
    if (sepB.x > kRelativeTol * separation + kAbsoluteTol * b.half.x) {
        axis = Axis::FaceBX;
        separation = sepB.x;
        normal = dB.x > 0.0F ? Vec2{rb.c, rb.s} : Vec2{-rb.c, -rb.s};
    }
    if (sepB.y > kRelativeTol * separation + kAbsoluteTol * b.half.y) {
        axis = Axis::FaceBY;
        separation = sepB.y;
        normal = dB.y > 0.0F ? Vec2{-rb.s, rb.c} : Vec2{rb.s, -rb.c};
    }

    // Reference face: the face of the box that owns the axis. Incident edge: the edge of the other box facing it.
    Vec2 frontNormal;
    Vec2 sideNormal;
    float front;
    float negSide;
    float posSide;
    ClipPoint incident[2];
    switch (axis) {
        case Axis::FaceAX:
            frontNormal = normal;
            front = dot(a.pos, frontNormal) + a.half.x;
            sideNormal = {-ra.s, ra.c};
            negSide = -dot(a.pos, sideNormal) + a.half.y;
            posSide = dot(a.pos, sideNormal) + a.half.y;
            incidentEdge(incident, b, frontNormal);
            break;
        case Axis::FaceAY:
            frontNormal = normal;
            front = dot(a.pos, frontNormal) + a.half.y;
            sideNormal = {ra.c, ra.s};
            negSide = -dot(a.pos, sideNormal) + a.half.x;
            posSide = dot(a.pos, sideNormal) + a.half.x;
            incidentEdge(incident, b, frontNormal);
            break;
        case Axis::FaceBX:
            frontNormal = -normal;
            front = dot(b.pos, frontNormal) + b.half.x;
            sideNormal = {-rb.s, rb.c};
            negSide = -dot(b.pos, sideNormal) + b.half.y;
            posSide = dot(b.pos, sideNormal) + b.half.y;
            incidentEdge(incident, a, frontNormal);
            break;
        case Axis::FaceBY:
        default:
            frontNormal = -normal;
            front = dot(b.pos, frontNormal) + b.half.y;
            sideNormal = {rb.c, rb.s};
            negSide = -dot(b.pos, sideNormal) + b.half.x;
            posSide = dot(b.pos, sideNormal) + b.half.x;
            incidentEdge(incident, a, frontNormal);
            break;
    }

    // Clip the incident edge to the side planes of the reference face, then keep what is behind the face.
    ClipPoint clip1[2];
    ClipPoint clip2[2];
    if (clipSegment(clip1, incident, -sideNormal, negSide) < 2) return m;
    if (clipSegment(clip2, clip1, sideNormal, posSide) < 2) return m;

    m.normal = normal;
    for (const ClipPoint& p : clip2) {
        const float sep = dot(frontNormal, p.v) - front;
        if (sep <= 0.0F) {
            Contact& c = m.contacts[m.count++];
            c.separation = sep;
            c.position = p.v - frontNormal * sep;  // slide the point onto the reference face
        }
    }
    return m;
}

}  // namespace detail

/// Contacts between two bodies (normal from a to b), or an empty manifold. Touching only is not a contact.
inline Manifold collide(const Body& a, const Body& b) {
    if (a.shape == Shape::Circle && b.shape == Shape::Circle) return detail::circleCircle(a, b);
    if (a.shape == Shape::Box && b.shape == Shape::Box) return detail::boxBox(a, b);

    Manifold m;
    const bool circleFirst = a.shape == Shape::Circle;
    const detail::CircleBox hit = circleFirst ? detail::circleVsBox(a, b) : detail::circleVsBox(b, a);
    if (!hit.hit) return m;
    m.count = 1;
    m.normal = circleFirst ? -hit.normal : hit.normal;  // the result points box -> circle, we want a -> b
    m.contacts[0].separation = -hit.depth;
    m.contacts[0].position = hit.point;
    return m;
}

// ---- world and solver ----

struct Arbiter {
    int a = 0;
    int b = 0;
    Manifold manifold;
};

struct World {
    std::vector<Body> bodies;
    std::vector<Arbiter> arbiters;  // the contacts of the last step, kept for drawing and for warm starting
    Vec2 gravity{0.0F, kGravity};
    float restitution = 0.5F;
    float friction = 0.5F;
    int iterations = 8;
    float biasFactor = 0.2F;       // share of the overlap that is pushed out per step
    float slop = 0.01F;            // overlap in meters that is tolerated, keeps resting contacts from jittering
    float restThreshold = 1.0F;    // m/s: slower impacts do not bounce
    float maxCorrection = 4.0F;    // m/s: cap for the position correction speed

    void step(float dt) {
        const float invDt = 1.0F / dt;
        for (Body& b : bodies) {
            if (!b.isStatic()) b.vel = b.vel + gravity * dt;
        }

        findContacts();

        for (Arbiter& arb : arbiters) {
            Body& a = bodies[static_cast<std::size_t>(arb.a)];
            Body& b = bodies[static_cast<std::size_t>(arb.b)];
            Manifold& m = arb.manifold;
            const Vec2 tangent{m.normal.y, -m.normal.x};
            for (int i = 0; i < m.count; ++i) {
                Contact& c = m.contacts[i];
                c.r1 = c.position - a.pos;
                c.r2 = c.position - b.pos;
                // Effective mass along a direction d: 1 / (1/mA + 1/mB + (r1 x d)^2/IA + (r2 x d)^2/IB)
                const float rn1 = cross(c.r1, m.normal);
                const float rn2 = cross(c.r2, m.normal);
                c.massNormal = 1.0F / (a.invMass + b.invMass + a.invInertia * rn1 * rn1 + b.invInertia * rn2 * rn2);
                const float rt1 = cross(c.r1, tangent);
                const float rt2 = cross(c.r2, tangent);
                c.massTangent = 1.0F / (a.invMass + b.invMass + a.invInertia * rt1 * rt1 + b.invInertia * rt2 * rt2);

                const float correction = std::min(-biasFactor * invDt * std::min(0.0F, c.separation + slop), maxCorrection);
                const float approach = dot(relativeVelocity(a, b, c), m.normal);
                const float bounce = approach < -restThreshold ? -restitution * approach : 0.0F;
                c.targetSpeed = std::max(correction, bounce);

                // Warm start: apply what the last step needed, the solver then only has to fix the difference.
                applyImpulse(a, b, c, m.normal * c.pn + tangent * c.pt);
            }
        }

        for (int it = 0; it < iterations; ++it) {
            for (Arbiter& arb : arbiters) {
                Body& a = bodies[static_cast<std::size_t>(arb.a)];
                Body& b = bodies[static_cast<std::size_t>(arb.b)];
                Manifold& m = arb.manifold;
                const Vec2 tangent{m.normal.y, -m.normal.x};
                for (int i = 0; i < m.count; ++i) {
                    Contact& c = m.contacts[i];

                    // Friction first: it is limited by the normal impulse, which should be the freshest value.
                    const float vt = dot(relativeVelocity(a, b, c), tangent);
                    const float maxFriction = friction * c.pn;
                    const float oldPt = c.pt;
                    c.pt = std::clamp(oldPt + c.massTangent * -vt, -maxFriction, maxFriction);
                    applyImpulse(a, b, c, tangent * (c.pt - oldPt));

                    // Normal: j = massNormal * (target - vn). The sum of all impulses may never pull (stay >= 0).
                    const float vn = dot(relativeVelocity(a, b, c), m.normal);
                    const float oldPn = c.pn;
                    c.pn = std::max(oldPn + c.massNormal * (c.targetSpeed - vn), 0.0F);
                    applyImpulse(a, b, c, m.normal * (c.pn - oldPn));
                }
            }
        }

        for (Body& b : bodies) {
            if (b.isStatic()) continue;
            b.pos = b.pos + b.vel * dt;
            b.angle += b.angVel * dt;
        }
    }

    /// Removes every non-static body.
    void clearDynamic() {
        bodies.erase(std::remove_if(bodies.begin(), bodies.end(), [](const Body& b) { return !b.isStatic(); }), bodies.end());
        arbiters.clear();
    }

private:
    static Vec2 relativeVelocity(const Body& a, const Body& b, const Contact& c) {
        return (b.vel + cross(b.angVel, c.r2)) - (a.vel + cross(a.angVel, c.r1));
    }

    static void applyImpulse(Body& a, Body& b, const Contact& c, Vec2 p) {
        a.vel = a.vel - p * a.invMass;
        a.angVel -= a.invInertia * cross(c.r1, p);
        b.vel = b.vel + p * b.invMass;
        b.angVel += b.invInertia * cross(c.r2, p);
    }

    void findContacts() {
        std::map<std::pair<int, int>, const Arbiter*> old;
        for (const Arbiter& arb : arbiters) old[{arb.a, arb.b}] = &arb;
        std::vector<Arbiter> fresh;
        const int n = static_cast<int>(bodies.size());
        for (int i = 0; i < n; ++i) {
            for (int j = i + 1; j < n; ++j) {
                const Body& a = bodies[static_cast<std::size_t>(i)];
                const Body& b = bodies[static_cast<std::size_t>(j)];
                if (a.isStatic() && b.isStatic()) continue;
                const float reach = a.boundRadius() + b.boundRadius();
                const Vec2 d = b.pos - a.pos;
                if (dot(d, d) > reach * reach) continue;  // broad phase: bounding circles do not touch
                Manifold m = collide(a, b);
                if (m.count == 0) continue;
                // Carry the impulses over from the contact point that is nearest to the same one last step.
                const auto it = old.find({i, j});
                if (it != old.end()) {
                    for (int k = 0; k < m.count; ++k) {
                        for (int q = 0; q < it->second->manifold.count; ++q) {
                            const Contact& prev = it->second->manifold.contacts[q];
                            const Vec2 gap = prev.position - m.contacts[k].position;
                            if (dot(gap, gap) < kMatchDistance * kMatchDistance) {
                                m.contacts[k].pn = prev.pn;
                                m.contacts[k].pt = prev.pt;
                                break;
                            }
                        }
                    }
                }
                fresh.push_back(Arbiter{i, j, m});
            }
        }
        arbiters = std::move(fresh);
    }

    static constexpr float kMatchDistance = 0.1F;
};

/// Total energy in joules: kinetic (linear + rotation) plus potential above `floorY`.
inline float totalEnergy(const World& w, float floorY) {
    float e = 0.0F;
    for (const Body& b : w.bodies) {
        if (b.isStatic()) continue;
        const float m = b.mass();
        const float inertia = 1.0F / b.invInertia;
        e += 0.5F * m * dot(b.vel, b.vel) + 0.5F * inertia * b.angVel * b.angVel + m * kGravity * (floorY - b.pos.y);
    }
    return e;
}

/// Pulls the point `localAnchor` of the body (in body coordinates) towards `target` with a critically damped spring.
/// Meant for dragging with the mouse: call it once per step before World::step.
inline void applySpring(Body& b, Vec2 localAnchor, Vec2 target, float dt) {
    if (b.isStatic()) return;
    constexpr float kOmega = 12.0F;       // rad/s, how stiff
    constexpr float kMaxAccel = 80.0F;    // m/s^2, so a far target does not shoot the body away
    const Vec2 r = mul(rot(b.angle), localAnchor);
    const Vec2 anchor = b.pos + r;
    const Vec2 anchorVelocity = b.vel + cross(b.angVel, r);
    const float m = b.mass();
    Vec2 force = (target - anchor) * (m * kOmega * kOmega) - anchorVelocity * (2.0F * m * kOmega);
    const float limit = m * kMaxAccel;
    const float len = length(force);
    if (len > limit) force = force * (limit / len);
    const Vec2 impulse = force * dt;
    b.vel = b.vel + impulse * b.invMass;
    b.angVel += b.invInertia * cross(r, impulse);
}

}  // namespace rb
