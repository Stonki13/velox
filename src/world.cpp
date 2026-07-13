#include "velox/world.h"
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

void World::step(float dt) {
    backend_->integrate(bodies_, gravity, dt);

    // CCD stepping: advance all bodies only to the earliest time of impact,
    // resolve that contact, then continue with the remaining time. This is
    // what makes tunneling impossible regardless of velocity.
    float remaining = 1.0f;
    for (int iter = 0; iter < 8 && remaining > 1e-6f; ++iter) {
        float slice = dt * remaining;
        backend_->findContacts(bodies_, slice, contacts_);

        float earliest = 1.0f;
        for (const Contact& c : contacts_)
            earliest = std::min(earliest, c.toi);

        // Advance everyone to the earliest impact.
        for (Body& b : bodies_)
            if (!b.isStatic()) b.position += b.velocity * (slice * earliest);

        // Resolve impulses for contacts occurring at (or before) that time.
        for (const Contact& c : contacts_) {
            if (c.toi > earliest + 1e-6f) continue;
            Body& a = bodies_[c.a];
            Body& b = bodies_[c.b];
            float invMassSum = a.invMass + b.invMass;
            if (invMassSum == 0.0f) continue;

            Vec3 rv = a.velocity - b.velocity;
            float vn = dot(rv, c.normal);
            if (vn >= 0.0f) continue; // already separating

            float e = std::min(a.restitution, b.restitution);
            float jn = -(1.0f + e) * vn / invMassSum;
            Vec3 impulse = c.normal * jn;
            a.velocity += impulse * a.invMass;
            b.velocity -= impulse * b.invMass;

            // Coulomb friction against the tangential velocity.
            Vec3 tangent = rv - c.normal * vn;
            float tLen = length(tangent);
            if (tLen > 1e-6f) {
                tangent *= 1.0f / tLen;
                float mu = std::sqrt(a.friction * b.friction);
                float jt = std::clamp(-dot(rv, tangent) / invMassSum, -mu * jn, mu * jn);
                a.velocity += tangent * (jt * a.invMass);
                b.velocity -= tangent * (jt * b.invMass);
            }

            // Positional correction for any residual penetration.
            if (c.depth > 1e-4f) {
                Vec3 correction = c.normal * (c.depth * 0.8f / invMassSum);
                a.position += correction * a.invMass;
                b.position -= correction * b.invMass;
            }
        }

        remaining *= (1.0f - earliest);
    }
}

} // namespace velox
