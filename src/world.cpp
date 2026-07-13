#include "velox/world.h"
#include "narrowphase.h"
#include <algorithm>
#include <stdexcept>

namespace velox {

World::World(BackendType type) {
    if (type != BackendType::Cpu) backend_.reset(createCudaBackend());
    if (!backend_) {
        if (type == BackendType::Cuda)
            throw std::runtime_error("velox: CUDA backend unavailable "
                                     "(not built with VELOX_ENABLE_CUDA or no device)");
        backend_.reset(createCpuBackend());
    }
}

const char* World::backendName() const { return backend_->name(); }

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

namespace {

// Recursive median-split BVH build. Children are allocated consecutively so
// traversal only stores one child index per inner node.
struct BvhBuilder {
    std::vector<BvhNode>& nodes;
    std::vector<uint32_t>& triRefs;   // global array; this mesh's refs start at refBase
    const std::vector<Vec3>& centroids;
    const std::vector<Vec3>& triMin;
    const std::vector<Vec3>& triMax;
    uint32_t refBase;

    void computeBounds(uint32_t nodeIdx, uint32_t first, uint32_t count) {
        BvhNode& n = nodes[nodeIdx];
        n.aabbMin = triMin[triRefs[first]];
        n.aabbMax = triMax[triRefs[first]];
        for (uint32_t i = 1; i < count; ++i) {
            n.aabbMin = vmin(n.aabbMin, triMin[triRefs[first + i]]);
            n.aabbMax = vmax(n.aabbMax, triMax[triRefs[first + i]]);
        }
    }

    void build(uint32_t nodeIdx, uint32_t first, uint32_t count) {
        computeBounds(nodeIdx, first, count);
        if (count <= 4) {
            nodes[nodeIdx].leftFirst = first;
            nodes[nodeIdx].triCount = count;
            return;
        }
        // Split at the median along the widest centroid axis.
        Vec3 lo = centroids[triRefs[first]], hi = lo;
        for (uint32_t i = 1; i < count; ++i) {
            lo = vmin(lo, centroids[triRefs[first + i]]);
            hi = vmax(hi, centroids[triRefs[first + i]]);
        }
        Vec3 ext = hi - lo;
        int axis = ext.x > ext.y ? (ext.x > ext.z ? 0 : 2) : (ext.y > ext.z ? 1 : 2);
        auto key = [&](uint32_t t) {
            const Vec3& c = centroids[t];
            return axis == 0 ? c.x : axis == 1 ? c.y : c.z;
        };
        std::nth_element(triRefs.begin() + first,
                         triRefs.begin() + first + count / 2,
                         triRefs.begin() + first + count,
                         [&](uint32_t a, uint32_t b) { return key(a) < key(b); });
        uint32_t mid = count / 2;

        uint32_t left = static_cast<uint32_t>(nodes.size());
        nodes[nodeIdx].leftFirst = left;
        nodes[nodeIdx].triCount = 0;
        nodes.emplace_back();
        nodes.emplace_back();
        build(left, first, mid);
        build(left + 1, first + mid, count - mid);
    }
};

} // namespace

BodyId World::addStaticMesh(const std::vector<Vec3>& vertices,
                            const std::vector<uint32_t>& indices) {
    Mesh m;
    m.firstVertex = static_cast<uint32_t>(meshes_.vertices.size());
    m.vertexCount = static_cast<uint32_t>(vertices.size());
    m.firstIndex = static_cast<uint32_t>(meshes_.indices.size());
    m.indexCount = static_cast<uint32_t>(indices.size());
    m.aabbMin = m.aabbMax = vertices.empty() ? Vec3{} : vertices[0];
    for (const Vec3& v : vertices) {
        m.aabbMin = vmin(m.aabbMin, v);
        m.aabbMax = vmax(m.aabbMax, v);
    }
    meshes_.vertices.insert(meshes_.vertices.end(), vertices.begin(), vertices.end());
    meshes_.indices.insert(meshes_.indices.end(), indices.begin(), indices.end());

    // Build the triangle BVH.
    uint32_t triCount = m.indexCount / 3;
    std::vector<Vec3> centroids(triCount), triMin(triCount), triMax(triCount);
    for (uint32_t t = 0; t < triCount; ++t) {
        const Vec3& a = vertices[indices[t * 3]];
        const Vec3& b = vertices[indices[t * 3 + 1]];
        const Vec3& c = vertices[indices[t * 3 + 2]];
        centroids[t] = (a + b + c) * (1.0f / 3.0f);
        triMin[t] = vmin(vmin(a, b), c);
        triMax[t] = vmax(vmax(a, b), c);
    }
    m.firstNode = static_cast<uint32_t>(meshes_.bvhNodes.size());
    m.firstTriRef = static_cast<uint32_t>(meshes_.bvhTriRefs.size());
    for (uint32_t t = 0; t < triCount; ++t) meshes_.bvhTriRefs.push_back(t);
    if (triCount > 0) {
        meshes_.bvhNodes.emplace_back();
        BvhBuilder builder{meshes_.bvhNodes, meshes_.bvhTriRefs,
                           centroids, triMin, triMax, m.firstTriRef};
        builder.build(m.firstNode, m.firstTriRef, triCount);
    }
    m.nodeCount = static_cast<uint32_t>(meshes_.bvhNodes.size()) - m.firstNode;
    meshes_.meshes.push_back(m);

    Body b;
    b.shape = ShapeType::Mesh;
    b.meshIndex = static_cast<uint32_t>(meshes_.meshes.size() - 1);
    b.invMass = 0.0f; // mesh colliders are static (level geometry)
    bodies_.push_back(b);
    return static_cast<BodyId>(bodies_.size() - 1);
}

JointId World::addBallJoint(BodyId a, BodyId b, Vec3 worldAnchor) {
    Joint j;
    j.type = JointType::Ball;
    j.a = a; j.b = b;
    j.localAnchorA = rotateInv(bodies_[a].orientation, worldAnchor - bodies_[a].position);
    j.localAnchorB = rotateInv(bodies_[b].orientation, worldAnchor - bodies_[b].position);
    joints_.push_back(j);
    return static_cast<JointId>(joints_.size() - 1);
}

JointId World::addDistanceJoint(BodyId a, BodyId b, Vec3 worldAnchorA, Vec3 worldAnchorB) {
    Joint j;
    j.type = JointType::Distance;
    j.a = a; j.b = b;
    j.localAnchorA = rotateInv(bodies_[a].orientation, worldAnchorA - bodies_[a].position);
    j.localAnchorB = rotateInv(bodies_[b].orientation, worldAnchorB - bodies_[b].position);
    j.restLength = length(worldAnchorA - worldAnchorB);
    joints_.push_back(j);
    return static_cast<JointId>(joints_.size() - 1);
}

JointId World::addHingeJoint(BodyId a, BodyId b, Vec3 worldAnchor, Vec3 worldAxis) {
    Joint j;
    j.type = JointType::Hinge;
    j.a = a; j.b = b;
    j.localAnchorA = rotateInv(bodies_[a].orientation, worldAnchor - bodies_[a].position);
    j.localAnchorB = rotateInv(bodies_[b].orientation, worldAnchor - bodies_[b].position);
    Vec3 axis = normalize(worldAxis);
    j.localAxisA = rotateInv(bodies_[a].orientation, axis);
    j.localAxisB = rotateInv(bodies_[b].orientation, axis);
    joints_.push_back(j);
    return static_cast<JointId>(joints_.size() - 1);
}

void World::wake(BodyId id) {
    bodies_[id].asleep = 0;
    bodies_[id].sleepTimer = 0.0f;
}

namespace {

// Minimal column-major 3x3 for joint effective-mass solves.
struct Mat3 {
    Vec3 c0, c1, c2;
};

Vec3 mul(const Mat3& m, const Vec3& v) {
    return m.c0 * v.x + m.c1 * v.y + m.c2 * v.z;
}

Mat3 inverse(const Mat3& m) {
    Vec3 r0 = cross(m.c1, m.c2);
    Vec3 r1 = cross(m.c2, m.c0);
    Vec3 r2 = cross(m.c0, m.c1);
    float det = dot(m.c0, r0);
    float inv = std::fabs(det) > 1e-12f ? 1.0f / det : 0.0f;
    // rows of the inverse are r0,r1,r2 scaled; transpose into columns.
    return {{r0.x * inv, r1.x * inv, r2.x * inv},
            {r0.y * inv, r1.y * inv, r2.y * inv},
            {r0.z * inv, r1.z * inv, r2.z * inv}};
}

// K(P): change of relative anchor velocity per unit impulse P at the anchors.
Mat3 pointMass(const Body& a, const Body& b, const Vec3& ra, const Vec3& rb) {
    auto col = [&](Vec3 e) {
        Vec3 k = e * (a.invMass + b.invMass);
        k += cross(a.invInertiaMul(cross(ra, e)), ra);
        k += cross(b.invInertiaMul(cross(rb, e)), rb);
        return k;
    };
    return {col({1, 0, 0}), col({0, 1, 0}), col({0, 0, 1})};
}

Vec3 anchorVelocity(const Body& x, const Vec3& r) {
    return x.velocity + cross(x.angularVelocity, r);
}

void applyImpulse(Body& x, const Vec3& r, const Vec3& P, float sign) {
    if (x.isStatic()) return;
    x.velocity += P * (sign * x.invMass);
    x.angularVelocity += x.invInertiaMul(cross(r, P)) * sign;
}

// Signed gap and contact normal (from B towards A) between two bodies with
// their transforms overridden — the distance oracle for conservative
// advancement.
struct GapProbe { float gap; Vec3 normal; };

GapProbe gapAt(const Body& a, const Vec3& pa, const Quat& qa,
               const Body& b, const Vec3& pb, const Quat& qb,
               const MeshSoupView& soup, float searchRadius) {
    Body ta = a; ta.position = pa; ta.orientation = qa;
    Body tb = b; tb.position = pb; tb.orientation = qb;

    if (tb.shape == ShapeType::Plane) {
        Convex c = makeConvex(ta);
        Vec3 deep = c.support(-tb.planeNormal);
        return {dot(tb.planeNormal, deep) - tb.planeOffset - c.radius, tb.planeNormal};
    }
    if (tb.shape == ShapeType::Mesh) {
        GapProbe best{1e30f, {0, 1, 0}};
        meshGapProbe(ta, tb, soup, searchRadius, best.gap, best.normal);
        return best;
    }
    GjkResult r = gjkDistance(makeConvex(ta), makeConvex(tb));
    return {r.distance, r.normal};
}

} // namespace

// Iterative impulse solve for all joints, with Baumgarte positional bias.
// Joints are few compared to contacts, so this runs on the CPU.
void World::solveJoints(float dt) {
    if (joints_.empty()) return;
    constexpr int kJointIterations = 8;
    constexpr float kBeta = 0.2f; // positional correction per step fraction

    for (const Joint& j : joints_) {
        // A sleeping body attached to an awake one must participate.
        if (bodies_[j.a].asleep != bodies_[j.b].asleep) { wake(j.a); wake(j.b); }
    }

    for (int iter = 0; iter < kJointIterations; ++iter) {
        for (Joint& j : joints_) {
            Body& a = bodies_[j.a];
            Body& b = bodies_[j.b];
            if (a.asleep && b.asleep) continue;
            Vec3 ra = rotate(a.orientation, j.localAnchorA);
            Vec3 rb = rotate(b.orientation, j.localAnchorB);
            Vec3 pa = a.position + ra, pb = b.position + rb;
            Vec3 vel = anchorVelocity(a, ra) - anchorVelocity(b, rb);

            switch (j.type) {
            case JointType::Ball: {
                Vec3 bias = (pa - pb) * (kBeta / dt);
                Vec3 P = mul(inverse(pointMass(a, b, ra, rb)), -(vel + bias));
                applyImpulse(a, ra, P, +1.0f);
                applyImpulse(b, rb, P, -1.0f);
                break;
            }
            case JointType::Distance: {
                Vec3 d = pa - pb;
                float len = length(d);
                if (len < 1e-8f) break;
                Vec3 n = d * (1.0f / len);
                Vec3 raxn = cross(ra, n), rbxn = cross(rb, n);
                float k = a.invMass + b.invMass +
                          dot(raxn, a.invInertiaMul(raxn)) +
                          dot(rbxn, b.invInertiaMul(rbxn));
                if (k <= 0.0f) break;
                float bias = (len - j.restLength) * (kBeta / dt);
                float lambda = -(dot(vel, n) + bias) / k;
                applyImpulse(a, ra, n * lambda, +1.0f);
                applyImpulse(b, rb, n * lambda, -1.0f);
                break;
            }
            case JointType::Hinge: {
                // Point constraint...
                Vec3 bias = (pa - pb) * (kBeta / dt);
                Vec3 P = mul(inverse(pointMass(a, b, ra, rb)), -(vel + bias));
                applyImpulse(a, ra, P, +1.0f);
                applyImpulse(b, rb, P, -1.0f);
                // ...plus two angular rows keeping the axes aligned.
                Vec3 axisA = rotate(a.orientation, j.localAxisA);
                Vec3 axisB = rotate(b.orientation, j.localAxisB);
                Vec3 err = cross(axisB, axisA); // rotate B by err to realign
                Vec3 ref = std::fabs(axisA.x) < 0.9f ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
                Vec3 b1 = normalize(cross(axisA, ref));
                Vec3 b2 = cross(axisA, b1);
                Vec3 wr = a.angularVelocity - b.angularVelocity;
                for (Vec3 t : {b1, b2}) {
                    float k = dot(t, a.invInertiaMul(t)) + dot(t, b.invInertiaMul(t));
                    if (k <= 0.0f) continue;
                    // err = cross(axisB, axisA): B must gain +err spin to
                    // realign, i.e. the relative velocity wr = wa - wb must
                    // be driven towards -err * beta/dt.
                    float lambda = -(dot(wr, t) + dot(err, t) * (kBeta / dt)) / k;
                    if (!a.isStatic()) a.angularVelocity += a.invInertiaMul(t * lambda);
                    if (!b.isStatic()) b.angularVelocity -= b.invInertiaMul(t * lambda);
                    wr = a.angularVelocity - b.angularVelocity;
                }
                break;
            }
            }
        }
    }
}

// Union-find islands over contacts and joints; whole islands fall asleep
// together once every member has been slow for long enough.
void World::updateSleeping(float dt) {
    constexpr float kMotionTol = 2.5e-3f; // |v|^2 + |w|^2 threshold
    constexpr float kTimeToSleep = 0.5f;  // seconds

    size_t n = bodies_.size();
    unionParent_.resize(n);
    for (uint32_t i = 0; i < n; ++i) unionParent_[i] = i;
    // Iterative find with path halving.
    auto find = [&](uint32_t x) {
        while (unionParent_[x] != x) {
            unionParent_[x] = unionParent_[unionParent_[x]];
            x = unionParent_[x];
        }
        return x;
    };
    auto unite = [&](uint32_t x, uint32_t y) {
        x = find(x); y = find(y);
        if (x != y) unionParent_[x] = y;
    };
    for (const Contact& c : contacts_)
        if (!bodies_[c.a].isStatic() && !bodies_[c.b].isStatic()) unite(c.a, c.b);
    for (const Joint& j : joints_)
        if (!bodies_[j.a].isStatic() && !bodies_[j.b].isStatic()) unite(j.a, j.b);

    for (Body& b : bodies_) {
        if (b.isStatic() || b.asleep) continue;
        float motion = lengthSq(b.velocity) + lengthSq(b.angularVelocity);
        b.sleepTimer = motion < kMotionTol ? b.sleepTimer + dt : 0.0f;
    }

    // Minimum timer per island root; islands where everyone is calm sleep.
    islandTimer_.assign(n, 1e30f);
    for (BodyId i = 0; i < n; ++i) {
        Body& b = bodies_[i];
        if (b.isStatic() || b.asleep) continue;
        uint32_t root = find(i);
        if (b.sleepTimer < islandTimer_[root]) islandTimer_[root] = b.sleepTimer;
    }
    for (BodyId i = 0; i < n; ++i) {
        Body& b = bodies_[i];
        if (b.isStatic() || b.asleep) continue;
        if (islandTimer_[find(i)] > kTimeToSleep) {
            b.asleep = 1;
            b.velocity = {};
            b.angularVelocity = {};
        }
    }
}

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
    const int nSub = substeps > 0 ? substeps : 1;
    const float h = dt / nSub;

    // Detect ONCE per step, with a speculative reach covering the full dt.
    // The solver substeps below re-evaluate each contact's live gap from
    // current body positions (Contact::bias0), Box2D-v3 style.
    backend_->integrate(bodies_, gravity, h); // first substep's gravity
    backend_->findContacts(bodies_, meshes_, dt, contacts_);

    // --- wake pass ------------------------------------------------------------
    // A sleeping body is woken when something awake and moving (or actually
    // striking it) shares a contact with it.
    for (const Contact& c : contacts_) {
        Body& a = bodies_[c.a];
        Body& b = bodies_[c.b];
        bool impact = c.vn0 < -0.1f;
        auto moving = [](const Body& x) {
            return lengthSq(x.velocity) + lengthSq(x.angularVelocity) > 1e-3f;
        };
        if (a.asleep && !b.isStatic() && !b.asleep && (impact || moving(b))) wake(c.a);
        if (b.asleep && !a.isStatic() && !a.asleep && (impact || moving(a))) wake(c.b);
    }

    // --- warm starting ---------------------------------------------------------
    // Carry accumulated normal impulses from last frame's matching contacts
    // (same body pair, nearby contact point).
    if (!prevContacts_.empty()) {
        auto keyOf = [](const Contact& c) { return (uint64_t)c.a << 32 | c.b; };
        for (Contact& c : contacts_) {
            uint64_t key = keyOf(c);
            auto lo = std::lower_bound(prevContacts_.begin(), prevContacts_.end(), key,
                [&](const Contact& p, uint64_t k) { return keyOf(p) < k; });
            float bestDistSq = 2.5e-3f; // 5 cm matching radius
            for (auto it = lo; it != prevContacts_.end() && keyOf(*it) == key; ++it) {
                float d = lengthSq(it->point - c.point);
                if (d < bestDistSq) { bestDistSq = d; c.normalImpulse = it->normalImpulse; }
            }
        }
    }

    prev_.resize(bodies_.size());
    for (size_t i = 0; i < bodies_.size(); ++i)
        prev_[i] = {bodies_[i].position, bodies_[i].orientation};

    // --- solver substeps -------------------------------------------------------
    // Each substep: gravity, velocity solve against live gaps, joints, and
    // position integration. Warm starting applies only on the first substep;
    // afterwards the accumulated impulses already live in the velocities.
    for (int s = 0; s < nSub; ++s) {
        if (s > 0) backend_->integrate(bodies_, gravity, h);
        backend_->solveVelocities(bodies_, contacts_, h, s == 0);
        solveJoints(h);
        for (Body& b : bodies_) {
            if (b.isStatic() || b.asleep) continue;
            b.position += b.velocity * h;
            b.orientation = integrate(b.orientation, b.angularVelocity, h);
        }
    }

    // --- conservative-advancement safety net ---------------------------------
    // Only pairs that ended the step interpenetrating need rescue. Speculative
    // detection guarantees every pair that could touch this step produced a
    // contact, so it suffices to re-check the (deduplicated) contact pairs;
    // for offenders, walk time forward from the pre-step state in provably-
    // safe increments (gap / max surface speed) until first contact.
    constexpr float kSlop = 1e-3f;
    const MeshSoupView soup = view(meshes_);
    pairKeys_.clear();
    for (const Contact& c : contacts_)
        pairKeys_.push_back((uint64_t)c.a << 32 | c.b);
    std::sort(pairKeys_.begin(), pairKeys_.end());
    pairKeys_.erase(std::unique(pairKeys_.begin(), pairKeys_.end()), pairKeys_.end());

    for (uint64_t key : pairKeys_) {
        BodyId i = (BodyId)(key >> 32), j = (BodyId)key;
        // The clamp below rewinds A only; make A the dynamic one.
        if (bodies_[i].isStatic()) { BodyId t = i; i = j; j = t; }
        Body& a = bodies_[i];
        Body& b = bodies_[j];
        if (a.isStatic()) continue;
        float search = a.maxPointSpeed() * dt + a.radius +
                       length(a.halfExtents) + a.capsuleHalfHeight + 0.1f;
        {

            GapProbe end = gapAt(a, a.position, a.orientation,
                                 b, b.position, b.orientation, soup, search);
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
                GapProbe g = gapAt(a, pa0 + da * t, qa, b, pb0 + db * t, qb, soup, search);
                n = g.normal;
                if (g.gap < kSlop) break;
                t = vmin(1.0f, t + g.gap / bound);
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

    backend_->fetchImpulses(contacts_); // GPU-resident impulses -> host contacts

    // --- positional correction (split-impulse style) --------------------------
    // Resolve residual penetration by translating bodies apart directly.
    // Velocities are untouched, so no energy is injected (Baumgarte bias in
    // the velocity solve launches stacked bodies). The current gap of each
    // contact is estimated from its detected gap plus the relative motion of
    // its bodies along the normal since detection.
    {
        constexpr int kPositionIterations = 3;
        constexpr float kSlop = 1e-3f, kResolve = 0.6f;
        for (int iter = 0; iter < kPositionIterations; ++iter) {
            for (const Contact& c : contacts_) {
                Body& a = bodies_[c.a];
                Body& b = bodies_[c.b];
                float invMassSum = a.invMass + b.invMass;
                if (invMassSum <= 0.0f) continue;
                float g = c.bias0 + dot(c.normal, a.position - b.position);
                if (g >= -kSlop) continue;
                float push = -kResolve * (g + kSlop) / invMassSum;
                if (!a.asleep) a.position += c.normal * (push * a.invMass);
                if (!b.asleep) b.position -= c.normal * (push * b.invMass);
            }
        }
    }

    // --- sleeping + persistent contacts for next frame ------------------------
    updateSleeping(dt);
    prevContacts_ = contacts_;
    std::sort(prevContacts_.begin(), prevContacts_.end(),
              [](const Contact& x, const Contact& y) {
                  return ((uint64_t)x.a << 32 | x.b) < ((uint64_t)y.a << 32 | y.b);
              });
}

} // namespace velox
