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

BodyId World::addConvexHull(Vec3 position, const std::vector<Vec3>& points, float mass) {
    if (points.size() < 4)
        throw std::invalid_argument("velox: convex hull requires at least four points");
    Body b;
    b.position = position;
    b.shape = ShapeType::Hull;
    b.hullFirst = static_cast<uint32_t>(meshes_.hullPoints.size());
    b.hullCount = static_cast<uint32_t>(points.size());
    meshes_.hullPoints.insert(meshes_.hullPoints.end(), points.begin(), points.end());
    float r2 = 0.0f;
    Vec3 lo = points[0], hi = lo;
    for (const Vec3& p : points) {
        r2 = vmax(r2, lengthSq(p));
        lo = vmin(lo, p);
        hi = vmax(hi, p);
    }
    b.radius = sqrtf(r2); // bounding radius (AABB + maxPointSpeed)
    b.invMass = mass > 0.0f ? 1.0f / mass : 0.0f;
    if (mass > 0.0f) {
        // Inertia approximated by the bounding box of the points.
        Vec3 e = hi - lo;
        float k = mass / 12.0f;
        b.invInertia = {1.0f / (k * (e.y * e.y + e.z * e.z) + 1e-9f),
                        1.0f / (k * (e.x * e.x + e.z * e.z) + 1e-9f),
                        1.0f / (k * (e.x * e.x + e.y * e.y) + 1e-9f)};
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
    // Shared perpendicular reference: measures the joint angle (0 now).
    Vec3 ref = std::fabs(axis.x) < 0.9f ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
    ref = normalize(cross(axis, ref));
    j.localRefA = rotateInv(bodies_[a].orientation, ref);
    j.localRefB = rotateInv(bodies_[b].orientation, ref);
    joints_.push_back(j);
    return static_cast<JointId>(joints_.size() - 1);
}

float World::hingeAngle(JointId id) const {
    const Joint& j = joints_[id];
    const Body& a = bodies_[j.a];
    const Body& b = bodies_[j.b];
    Vec3 axis = rotate(a.orientation, j.localAxisA);
    Vec3 refA = rotate(a.orientation, j.localRefA);
    Vec3 refB = rotate(b.orientation, j.localRefB);
    // Signed angle from refA to refB about the axis.
    float s = dot(cross(refA, refB), axis);
    float c = dot(refA, refB);
    return std::atan2(s, c);
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
        Convex c = makeConvex(ta, soup);
        Vec3 deep = c.support(-tb.planeNormal);
        return {dot(tb.planeNormal, deep) - tb.planeOffset - c.radius, tb.planeNormal};
    }
    if (tb.shape == ShapeType::Mesh) {
        GapProbe best{1e30f, {0, 1, 0}};
        meshGapProbe(ta, tb, soup, searchRadius, best.gap, best.normal);
        return best;
    }
    GjkResult r = gjkDistance(makeConvex(ta, soup), makeConvex(tb, soup));
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

    for (Joint& j : joints_) {
        j.motorImpulse = 0.0f;
        j.limitImpulse = 0.0f;
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

                // Motor and limit act on the rotation about the hinge axis.
                float kAxis = dot(axisA, a.invInertiaMul(axisA)) +
                              dot(axisA, b.invInertiaMul(axisA));
                if (kAxis > 0.0f) {
                    if (j.enableMotor) {
                        float wAxis = dot(a.angularVelocity - b.angularVelocity, axisA);
                        float lambda = (j.motorSpeed - wAxis) / kAxis;
                        float maxImpulse = j.maxMotorTorque * dt;
                        float oldImpulse = j.motorImpulse;
                        j.motorImpulse = vclamp(oldImpulse + lambda, -maxImpulse, maxImpulse);
                        lambda = j.motorImpulse - oldImpulse;
                        if (!a.isStatic()) a.angularVelocity += a.invInertiaMul(axisA * lambda);
                        if (!b.isStatic()) b.angularVelocity -= b.invInertiaMul(axisA * lambda);
                    }
                    if (j.enableLimit) {
                        // Current angle from the stored references.
                        Vec3 refA = rotate(a.orientation, j.localRefA);
                        Vec3 refB = rotate(b.orientation, j.localRefB);
                        float angle = std::atan2(dot(cross(refA, refB), axisA), dot(refA, refB));
                        float wAxis = dot(a.angularVelocity - b.angularVelocity, axisA);
                        // angle measures B relative... refA fixed on A: angle
                        // grows when B rotates +axis relative to A, i.e. when
                        // wAxis (wa - wb) is negative.
                        float lambda = 0.0f, oldImpulse = j.limitImpulse;
                        if (angle < j.lowerLimit) {
                            // angleDot = -wAxis. At the lower limit, only a
                            // negative axis impulse is allowed.
                            float target = (angle - j.lowerLimit) * (kBeta / dt);
                            lambda = (target - wAxis) / kAxis;
                            j.limitImpulse = vmin(0.0f, oldImpulse + lambda);
                            lambda = j.limitImpulse - oldImpulse;
                        } else if (angle > j.upperLimit) {
                            // At the upper limit, only a positive axis impulse
                            // is allowed. The bias drives the angle back in.
                            float target = (angle - j.upperLimit) * (kBeta / dt);
                            lambda = (target - wAxis) / kAxis;
                            j.limitImpulse = vmax(0.0f, oldImpulse + lambda);
                            lambda = j.limitImpulse - oldImpulse;
                        }
                        if (lambda != 0.0f) {
                            if (!a.isStatic()) a.angularVelocity += a.invInertiaMul(axisA * lambda);
                            if (!b.isStatic()) b.angularVelocity -= b.invInertiaMul(axisA * lambda);
                        }
                    }
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

    // Joint-connected bodies do not collide by default. Without this filter,
    // contacts at a hinge anchor fight the joint and CCD repeatedly rewinds the
    // pair. Set Joint::collideConnected when the linkage should self-collide.
    if (!joints_.empty() && !contacts_.empty()) {
        std::vector<uint64_t> excluded;
        excluded.reserve(joints_.size());
        for (const Joint& j : joints_) {
            if (j.collideConnected) continue;
            BodyId lo = j.a < j.b ? j.a : j.b;
            BodyId hi = j.a < j.b ? j.b : j.a;
            excluded.push_back((uint64_t)lo << 32 | hi);
        }
        std::sort(excluded.begin(), excluded.end());
        contacts_.erase(std::remove_if(contacts_.begin(), contacts_.end(),
            [&](const Contact& c) {
                BodyId lo = c.a < c.b ? c.a : c.b;
                BodyId hi = c.a < c.b ? c.b : c.a;
                return std::binary_search(excluded.begin(), excluded.end(),
                                          (uint64_t)lo << 32 | hi);
            }), contacts_.end());
    }

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
    // Carry accumulated normal impulses from last frame. Explicit manifold
    // features match exactly; generic GJK contacts fall back to proximity.
    if (!prevContacts_.empty()) {
        auto keyOf = [](const Contact& c) { return (uint64_t)c.a << 32 | c.b; };
        for (Contact& c : contacts_) {
            uint64_t key = keyOf(c);
            auto lo = std::lower_bound(prevContacts_.begin(), prevContacts_.end(), key,
                [&](const Contact& p, uint64_t k) { return keyOf(p) < k; });
            if (c.featureKey != 0) {
                bool matched = false;
                for (auto it = lo; it != prevContacts_.end() && keyOf(*it) == key; ++it)
                    if (it->featureKey == c.featureKey) {
                        c.normalImpulse = it->normalImpulse;
                        c.tangentImpulse1 = it->tangentImpulse1;
                        c.tangentImpulse2 = it->tangentImpulse2;
                        matched = true;
                        break;
                    }
                if (matched) continue;
            }
            float bestDistSq = 2.5e-3f; // 5 cm matching radius
            for (auto it = lo; it != prevContacts_.end() && keyOf(*it) == key; ++it) {
                float d = lengthSq(it->point - c.point);
                if (d < bestDistSq) {
                    bestDistSq = d;
                    c.normalImpulse = it->normalImpulse;
                    c.tangentImpulse1 = it->tangentImpulse1;
                    c.tangentImpulse2 = it->tangentImpulse2;
                }
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
        // Keep A dynamic for the static-dynamic case. Dynamic-dynamic pairs
        // are rewound symmetrically below.
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

            // Rewind both dynamic trajectories to the same impact time. This
            // preserves their shared timeline instead of arbitrarily clamping
            // one participant and leaving the other at the end of the step.
            a.position = pa0 + da * t;
            a.orientation = integrate(qa0, a.angularVelocity, dt * t);
            if (!b.isStatic()) {
                b.position = pb0 + db * t;
                b.orientation = integrate(qb0, b.angularVelocity, dt * t);
            }

            // Remove only the remaining approach velocity with an
            // inverse-mass impulse. Linear momentum is conserved for two
            // dynamic bodies; static geometry receives no velocity update.
            float vn = dot(a.velocity - b.velocity, n);
            float invMassSum = a.invMass + b.invMass;
            if (vn < 0.0f && invMassSum > 0.0f) {
                float impulse = -vn / invMassSum;
                a.velocity += n * (impulse * a.invMass);
                if (!b.isStatic()) b.velocity -= n * (impulse * b.invMass);
            }

            // Finish the frame from TOI using the corrected velocities. The
            // normal approach component is now zero, so this cannot recreate
            // the crossing, and tangential/center-of-mass motion is retained.
            float remaining = dt * (1.0f - t);
            a.position += a.velocity * remaining;
            a.orientation = integrate(a.orientation, a.angularVelocity, remaining);
            if (!b.isStatic()) {
                b.position += b.velocity * remaining;
                b.orientation = integrate(b.orientation, b.angularVelocity, remaining);
            }
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
        constexpr float kPositionSlop = 1e-3f, kResolve = 0.6f;
        for (int iter = 0; iter < kPositionIterations; ++iter) {
            for (const Contact& c : contacts_) {
                Body& a = bodies_[c.a];
                Body& b = bodies_[c.b];
                float invMassSum = a.invMass + b.invMass;
                if (invMassSum <= 0.0f) continue;
                float g = contactLiveGap(a, b, c);
                if (g >= -kPositionSlop) continue;
                float push = -kResolve * (g + kPositionSlop) / invMassSum;
                if (!a.asleep) a.position += c.normal * (push * a.invMass);
                if (!b.asleep) b.position -= c.normal * (push * b.invMass);
            }
        }
    }

    // --- contact begin events --------------------------------------------------
    // A pair "touches" when the solver applied impulse through it. An event
    // fires when a touching pair was not touching the previous step.
    {
        events_.clear();
        pairKeys_.clear();
        auto canonicalKey = [](BodyId a, BodyId b) {
            BodyId lo = a < b ? a : b;
            BodyId hi = a < b ? b : a;
            return (uint64_t)lo << 32 | hi;
        };
        for (const Contact& c : contacts_) {
            if (c.normalImpulse <= 1e-6f) continue;
            pairKeys_.push_back(canonicalKey(c.a, c.b));
        }
        std::sort(pairKeys_.begin(), pairKeys_.end());
        pairKeys_.erase(std::unique(pairKeys_.begin(), pairKeys_.end()), pairKeys_.end());
        for (uint64_t key : pairKeys_) {
            if (std::binary_search(prevPairKeys_.begin(), prevPairKeys_.end(), key))
                continue;
            ContactEvent ev{(BodyId)(key >> 32), (BodyId)key, {}, {}, 0.0f};
            for (const Contact& c : contacts_)
                if (canonicalKey(c.a, c.b) == key && c.normalImpulse > ev.impulse) {
                    ev.impulse = c.normalImpulse;
                    ev.point = c.point;
                    ev.normal = c.a == ev.a ? c.normal : -c.normal;
                }
            events_.push_back(ev);
        }
        prevPairKeys_ = pairKeys_;
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
