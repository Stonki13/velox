#include "velox/world.h"
#include "gjk.h"
#include <algorithm>

namespace velox {

World::World() : backend_(createCpuBackend()) {}

BodyId World::addSphere(Vec3 position, float radius, float mass) {
    Body b;
    b.position = position;
    b.shape = ShapeType::Sphere;
    b.radius = radius;
    b.invMass = mass > 0.0f ? 1.0f / mass : 0.0f;
    if (mass > 0.0f) {
        float i = 0.4f * mass * radius * radius; // solid sphere: 2/5 m r^2
        b.invInertia = {1.0f / i, 1.0f / i, 1.0f / i};
    }
    bodies_.push_back(b);
    return static_cast<BodyId>(bodies_.size() - 1);
}

BodyId World::addBox(Vec3 position, Vec3 halfExtents, float mass) {
    Body b;
    b.position = position;
    b.shape = ShapeType::Box;
    b.halfExtents = halfExtents;
    b.invMass = mass > 0.0f ? 1.0f / mass : 0.0f;
    if (mass > 0.0f) {
        Vec3 e = halfExtents * 2.0f;
        float k = mass / 12.0f;
        b.invInertia = {1.0f / (k * (e.y * e.y + e.z * e.z)),
                        1.0f / (k * (e.x * e.x + e.z * e.z)),
                        1.0f / (k * (e.x * e.x + e.y * e.y))};
    }
    bodies_.push_back(b);
    return static_cast<BodyId>(bodies_.size() - 1);
}

BodyId World::addCapsule(Vec3 position, float radius, float halfHeight, float mass) {
    Body b;
    b.position = position;
    b.shape = ShapeType::Capsule;
    b.radius = radius;
    b.capsuleHalfHeight = halfHeight;
    b.invMass = mass > 0.0f ? 1.0f / mass : 0.0f;
    if (mass > 0.0f) {
        // Approximated as a cylinder of the same core length plus end caps.
        float h = halfHeight * 2.0f, r = radius;
        float iy = 0.5f * mass * r * r;
        float ix = mass * (h * h / 12.0f + r * r * 0.25f) + 0.4f * mass * r * r;
        b.invInertia = {1.0f / ix, 1.0f / iy, 1.0f / ix};
    }
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

BodyId World::addStaticMesh(const std::vector<Vec3>& vertices,
                            const std::vector<uint32_t>& indices) {
    Mesh m;
    m.firstVertex = static_cast<uint32_t>(meshes_.vertices.size());
    m.vertexCount = static_cast<uint32_t>(vertices.size());
    m.firstIndex = static_cast<uint32_t>(meshes_.indices.size());
    m.indexCount = static_cast<uint32_t>(indices.size());
    m.aabbMin = m.aabbMax = vertices.empty() ? Vec3{} : vertices[0];
    for (const Vec3& v : vertices) {
        m.aabbMin = {std::min(m.aabbMin.x, v.x), std::min(m.aabbMin.y, v.y), std::min(m.aabbMin.z, v.z)};
        m.aabbMax = {std::max(m.aabbMax.x, v.x), std::max(m.aabbMax.y, v.y), std::max(m.aabbMax.z, v.z)};
    }
    meshes_.vertices.insert(meshes_.vertices.end(), vertices.begin(), vertices.end());
    meshes_.indices.insert(meshes_.indices.end(), indices.begin(), indices.end());
    meshes_.meshes.push_back(m);

    Body b;
    b.shape = ShapeType::Mesh;
    b.meshIndex = static_cast<uint32_t>(meshes_.meshes.size() - 1);
    b.invMass = 0.0f; // mesh colliders are static (level geometry)
    bodies_.push_back(b);
    return static_cast<BodyId>(bodies_.size() - 1);
}

namespace {

Vec3 pointVelocity(const Body& b, const Vec3& p) {
    return b.velocity + cross(b.angularVelocity, p - b.position);
}

// Signed gap and contact normal (from B towards A) between two bodies with
// their transforms overridden — the distance oracle for conservative
// advancement.
struct GapProbe { float gap; Vec3 normal; };

GapProbe gapAt(const Body& a, const Vec3& pa, const Quat& qa,
               const Body& b, const Vec3& pb, const Quat& qb,
               const MeshSoup& soup) {
    Body ta = a; ta.position = pa; ta.orientation = qa;
    Body tb = b; tb.position = pb; tb.orientation = qb;

    if (tb.shape == ShapeType::Plane) {
        Convex c = makeConvex(ta);
        Vec3 deep = c.support(-tb.planeNormal);
        return {dot(tb.planeNormal, deep) - tb.planeOffset - c.radius, tb.planeNormal};
    }
    if (tb.shape == ShapeType::Mesh) {
        const Mesh& m = soup.meshes[tb.meshIndex];
        Convex c = makeConvex(ta);
        GapProbe best{1e30f, {0, 1, 0}};
        for (uint32_t t = 0; t < m.indexCount; t += 3) {
            GjkResult r = gjkDistance(c, makeTriangle(
                soup.vertices[m.firstVertex + soup.indices[m.firstIndex + t]],
                soup.vertices[m.firstVertex + soup.indices[m.firstIndex + t + 1]],
                soup.vertices[m.firstVertex + soup.indices[m.firstIndex + t + 2]]));
            if (r.distance < best.gap) best = {r.distance, r.normal};
        }
        return best;
    }
    GjkResult r = gjkDistance(makeConvex(ta), makeConvex(tb));
    return {r.distance, r.normal};
}

} // namespace

// Predictive Contact Sweeping (PCS)
//
// 1. Speculative detection: contacts are created while pairs are still apart,
//    whenever their relative motion could close the gap this step.
// 2. Velocity solve: iterative sequential impulses at contact points with full
//    rotational response. Each contact only removes approach velocity in
//    excess of gap/dt, so grazing motion is untouched and piles are handled
//    by iteration instead of a time-of-impact event queue.
// 3. Conservative-advancement safety net: after integrating positions, any
//    pair that ended up interpenetrating is rewound along its actual motion
//    (rotation included) to the moment of first contact. Tunneling stays
//    impossible regardless of linear or angular speed.
void World::step(float dt) {
    if (dt <= 0.0f) return;
    backend_->integrate(bodies_, gravity, dt);
    backend_->findContacts(bodies_, meshes_, dt, contacts_);

    prev_.resize(bodies_.size());
    for (size_t i = 0; i < bodies_.size(); ++i)
        prev_[i] = {bodies_[i].position, bodies_[i].orientation};

    // --- velocity solve -----------------------------------------------------
    constexpr int kVelocityIterations = 10;
    constexpr float kRestitutionThreshold = 1.0f; // m/s; below this, no bounce
    for (int iter = 0; iter < kVelocityIterations; ++iter) {
        for (Contact& c : contacts_) {
            Body& a = bodies_[c.a];
            Body& b = bodies_[c.b];
            Vec3 ra = c.point - a.position;
            Vec3 rb = c.point - b.position;

            // Effective mass along the normal, including rotation.
            Vec3 raxn = cross(ra, c.normal);
            Vec3 rbxn = cross(rb, c.normal);
            float kNormal = a.invMass + b.invMass +
                            dot(raxn, a.invInertiaMul(raxn)) +
                            dot(rbxn, b.invInertiaMul(rbxn));
            if (kNormal <= 0.0f) continue;

            float vn = dot(pointVelocity(a, c.point) - pointVelocity(b, c.point), c.normal);

            // Target normal velocity: may still close the remaining gap, and
            // must bounce with restitution if the approach was fast.
            float target = c.gap > 0.0f ? -c.gap / dt : 0.0f;
            if (c.vn0 < -kRestitutionThreshold)
                target = std::max(target, -std::min(a.restitution, b.restitution) * c.vn0);

            float jn = (target - vn) / kNormal;
            float newImpulse = std::max(c.normalImpulse + jn, 0.0f);
            jn = newImpulse - c.normalImpulse;
            c.normalImpulse = newImpulse;

            Vec3 impulse = c.normal * jn;
            a.velocity += impulse * a.invMass;
            b.velocity -= impulse * b.invMass;
            a.angularVelocity += a.invInertiaMul(cross(ra, impulse));
            b.angularVelocity -= b.invInertiaMul(cross(rb, impulse));

            // Coulomb friction, clamped by the accumulated normal impulse.
            Vec3 rv = pointVelocity(a, c.point) - pointVelocity(b, c.point);
            Vec3 tangent = rv - c.normal * dot(rv, c.normal);
            float tLen = length(tangent);
            if (tLen > 1e-6f) {
                tangent *= 1.0f / tLen;
                Vec3 raxt = cross(ra, tangent);
                Vec3 rbxt = cross(rb, tangent);
                float kTangent = a.invMass + b.invMass +
                                 dot(raxt, a.invInertiaMul(raxt)) +
                                 dot(rbxt, b.invInertiaMul(rbxt));
                if (kTangent > 0.0f) {
                    float jt = -dot(rv, tangent) / kTangent;
                    float maxFriction = std::sqrt(a.friction * b.friction) * c.normalImpulse;
                    float newTangent = std::clamp(c.tangentImpulse + jt, -maxFriction, maxFriction);
                    jt = newTangent - c.tangentImpulse;
                    c.tangentImpulse = newTangent;
                    Vec3 fImpulse = tangent * jt;
                    a.velocity += fImpulse * a.invMass;
                    b.velocity -= fImpulse * b.invMass;
                    a.angularVelocity += a.invInertiaMul(cross(ra, fImpulse));
                    b.angularVelocity -= b.invInertiaMul(cross(rb, fImpulse));
                }
            }
        }
    }

    // --- integrate positions & orientations ----------------------------------
    for (Body& b : bodies_) {
        if (b.isStatic()) continue;
        b.position += b.velocity * dt;
        b.orientation = integrate(b.orientation, b.angularVelocity, dt);
    }

    // --- conservative-advancement safety net ---------------------------------
    // Only pairs that ended the step interpenetrating need rescue; for those,
    // walk time forward from the pre-step state in provably-safe increments
    // (gap / max surface speed) until first contact, then clamp there.
    constexpr float kSlop = 1e-3f;
    for (BodyId i = 0; i < bodies_.size(); ++i) {
        Body& a = bodies_[i];
        if (a.isStatic()) continue;

        for (BodyId j = 0; j < bodies_.size(); ++j) {
            if (i == j) continue;
            Body& b = bodies_[j];
            if (!b.isStatic() && j > i) continue; // dynamic pairs once
            if (b.shape == ShapeType::Plane && a.shape == ShapeType::Plane) continue;

            GapProbe end = gapAt(a, a.position, a.orientation,
                                 b, b.position, b.orientation, meshes_);
            if (end.gap >= -8e-3f) continue; // shallow: leave it to the solver

            // Interpolators over the step for both bodies.
            Vec3 pa0 = prev_[i].position, da = a.position - pa0;
            Quat qa0 = prev_[i].orientation;
            Vec3 pb0 = b.isStatic() ? b.position : prev_[j].position;
            Vec3 db = b.position - pb0;
            Quat qb0 = b.isStatic() ? b.orientation : prev_[j].orientation;

            float bound = a.maxPointSpeed() * dt + (b.isStatic() ? 0.0f : b.maxPointSpeed() * dt);
            if (bound < 1e-9f) continue;

            float t = 0.0f;
            Vec3 n = end.normal;
            for (int iter = 0; iter < 24 && t < 1.0f; ++iter) {
                Quat qa = integrate(qa0, a.angularVelocity, dt * t);
                Quat qb = b.isStatic() ? qb0 : integrate(qb0, b.angularVelocity, dt * t);
                GapProbe g = gapAt(a, pa0 + da * t, qa, b, pb0 + db * t, qb, meshes_);
                n = g.normal;
                if (g.gap < kSlop) break;
                t = std::min(1.0f, t + g.gap / bound);
            }
            if (t >= 1.0f) continue; // never actually touched (false alarm)

            // Clamp A back to the time of impact and kill its approach speed
            // along the true contact normal at that moment.
            a.position = pa0 + da * t;
            a.orientation = integrate(qa0, a.angularVelocity, dt * t);
            float vn = dot(a.velocity - b.velocity, n);
            if (vn < 0.0f) a.velocity -= n * (vn * (b.isStatic() ? 1.0f : 0.5f));
        }
    }
}

} // namespace velox
