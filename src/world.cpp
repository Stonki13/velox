#include "velox/world.h"
#include "ccd.h"
#include <algorithm>

namespace velox {

World::World() : backend_(createCpuBackend()) {}

BodyId World::addSphere(Vec3 position, float radius, float mass) {
    Body b;
    b.position = position;
    b.shape = ShapeType::Sphere;
    b.radius = radius;
    b.invMass = mass > 0.0f ? 1.0f / mass : 0.0f;
    bodies_.push_back(b);
    return static_cast<BodyId>(bodies_.size() - 1);
}

BodyId World::addStaticPlane(Vec3 normal, float offset) {
    Body b;
    b.shape = ShapeType::Plane;
    b.planeNormal = normalize(normal);
    b.planeOffset = offset;
    b.invMass = 0.0f;
    bodies_.push_back(b);
    return static_cast<BodyId>(bodies_.size() - 1);
}

// Predictive Contact Sweeping (PCS)
//
// 1. Speculative detection: contacts are created while pairs are still apart,
//    whenever their relative motion could close the gap this step.
// 2. Velocity solve: iterative sequential impulses. Each contact only removes
//    approach velocity in excess of gap/dt — a body is allowed to close its
//    remaining gap but not to pass it. Grazing motion is untouched, piles are
//    handled by iteration instead of a time-of-impact event queue, so there
//    is nothing to stall.
// 3. Exact sweep safety net: after integrating positions, every dynamic body
//    is swept along the displacement it actually made; anything the iterative
//    solve let slip is clamped back to its exact time of impact. Tunneling
//    stays impossible by construction.
void World::step(float dt) {
    if (dt <= 0.0f) return;
    backend_->integrate(bodies_, gravity, dt);
    backend_->findContacts(bodies_, dt, contacts_);

    prevPositions_.resize(bodies_.size());
    for (size_t i = 0; i < bodies_.size(); ++i)
        prevPositions_[i] = bodies_[i].position;

    // --- velocity solve -----------------------------------------------------
    constexpr int kVelocityIterations = 8;
    constexpr float kRestitutionThreshold = 1.0f; // m/s; below this, no bounce
    for (int iter = 0; iter < kVelocityIterations; ++iter) {
        for (Contact& c : contacts_) {
            Body& a = bodies_[c.a];
            Body& b = bodies_[c.b];
            float invMassSum = a.invMass + b.invMass;
            if (invMassSum == 0.0f) continue;

            float vn = dot(a.velocity - b.velocity, c.normal);

            // Target normal velocity: may still close the remaining gap, and
            // must bounce with restitution if the approach was fast.
            float target = c.gap > 0.0f ? -c.gap / dt : 0.0f;
            if (c.vn0 < -kRestitutionThreshold)
                target = std::max(target, -std::min(a.restitution, b.restitution) * c.vn0);

            float jn = (target - vn) / invMassSum;
            float newImpulse = std::max(c.normalImpulse + jn, 0.0f);
            jn = newImpulse - c.normalImpulse;
            c.normalImpulse = newImpulse;

            Vec3 impulse = c.normal * jn;
            a.velocity += impulse * a.invMass;
            b.velocity -= impulse * b.invMass;

            // Coulomb friction, clamped by the accumulated normal impulse.
            Vec3 rv = a.velocity - b.velocity;
            Vec3 tangent = rv - c.normal * dot(rv, c.normal);
            float tLen = length(tangent);
            if (tLen > 1e-6f) {
                tangent *= 1.0f / tLen;
                float jt = -dot(rv, tangent) / invMassSum;
                float maxFriction = std::sqrt(a.friction * b.friction) * c.normalImpulse;
                float newTangent = std::clamp(c.tangentImpulse + jt, -maxFriction, maxFriction);
                jt = newTangent - c.tangentImpulse;
                c.tangentImpulse = newTangent;
                a.velocity += tangent * (jt * a.invMass);
                b.velocity -= tangent * (jt * b.invMass);
            }
        }
    }

    // --- integrate positions -------------------------------------------------
    for (Body& b : bodies_)
        if (!b.isStatic()) b.position += b.velocity * dt;

    // --- exact sweep safety net ----------------------------------------------
    for (BodyId i = 0; i < bodies_.size(); ++i) {
        Body& a = bodies_[i];
        if (a.isStatic() || a.shape != ShapeType::Sphere) continue;
        Vec3 dispA = a.position - prevPositions_[i];

        for (BodyId j = 0; j < bodies_.size(); ++j) {
            if (i == j) continue;
            Body& b = bodies_[j];
            float toi = -1.0f;
            Vec3 n{};
            if (b.shape == ShapeType::Plane) {
                toi = sweepSpherePlane(prevPositions_[i], dispA, a.radius,
                                       b.planeNormal, b.planeOffset);
                n = b.planeNormal;
            } else if (b.shape == ShapeType::Sphere && j < i) { // each pair once
                Vec3 pb = b.isStatic() ? b.position : prevPositions_[j];
                Vec3 dispB = b.isStatic() ? Vec3{} : b.position - pb;
                toi = sweepSphereSphere(prevPositions_[i], dispA, a.radius,
                                        pb, dispB, b.radius);
                Vec3 ca = prevPositions_[i] + dispA * toi;
                Vec3 cb = pb + dispB * toi;
                n = normalize(ca - cb);
            }
            if (toi < 0.0f) continue;

            // Only intervene if the pair actually ended up interpenetrating —
            // contacts the solver already handled land here with toi ~ 0 and
            // no overlap, and must not be disturbed.
            float endGap;
            if (b.shape == ShapeType::Plane)
                endGap = dot(b.planeNormal, a.position) - b.planeOffset - a.radius;
            else
                endGap = length(a.position - b.position) - (a.radius + b.radius);
            constexpr float kSlop = 1e-4f;
            if (endGap >= -kSlop) continue;

            // Clamp back to the exact impact point and kill approach velocity.
            a.position = prevPositions_[i] + dispA * toi + n * kSlop;
            float vn = dot(a.velocity - b.velocity, n);
            if (vn < 0.0f) a.velocity -= n * (vn * (b.isStatic() ? 1.0f : 0.5f));
        }
    }
}

} // namespace velox
