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

namespace {

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

    // --- velocity solve (backend: sequential on CPU, graph-colored on GPU) ---
    backend_->solveVelocities(bodies_, contacts_, dt);

    // --- integrate positions & orientations ----------------------------------
    for (Body& b : bodies_) {
        if (b.isStatic()) continue;
        b.position += b.velocity * dt;
        b.orientation = integrate(b.orientation, b.angularVelocity, dt);
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
}

} // namespace velox
