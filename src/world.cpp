#include "velox/world.h"
#include "narrowphase.h"
#include <algorithm>
#include <chrono>
#include <stdexcept>

namespace velox {

namespace {

bool finiteFloat(float value) { return std::isfinite(value); }
bool finiteVec(const Vec3& v) {
    return finiteFloat(v.x) && finiteFloat(v.y) && finiteFloat(v.z);
}
bool finiteQuat(const Quat& q) {
    return finiteFloat(q.x) && finiteFloat(q.y) &&
           finiteFloat(q.z) && finiteFloat(q.w);
}

bool validCombineMode(MaterialCombineMode mode) {
    return (uint8_t)mode <= (uint8_t)MaterialCombineMode::Maximum;
}

bool validJointType(JointType type) {
    return (uint8_t)type <= (uint8_t)JointType::SixDof;
}

void jointFrame(const Joint& joint, const Body& a, const Body& b,
                Vec3& xA, Vec3& yA, Vec3& zA,
                Vec3& xB, Vec3& yB, Vec3& zB) {
    xA = normalize(rotate(a.orientation, joint.localAxisA));
    yA = normalize(rotate(a.orientation, joint.localRefA));
    zA = normalize(cross(xA, yA));
    yA = cross(zA, xA);
    xB = normalize(rotate(b.orientation, joint.localAxisB));
    yB = normalize(rotate(b.orientation, joint.localRefB));
    zB = normalize(cross(xB, yB));
    yB = cross(zB, xB);
}

Quat quaternionFromRotationMatrix(float m00, float m01, float m02,
                                  float m10, float m11, float m12,
                                  float m20, float m21, float m22) {
    Quat q;
    float trace = m00 + m11 + m22;
    if (trace > 0.0f) {
        float s = std::sqrt(trace + 1.0f) * 2.0f;
        q = {(m21 - m12) / s, (m02 - m20) / s,
             (m10 - m01) / s, 0.25f * s};
    } else if (m00 > m11 && m00 > m22) {
        float s = std::sqrt(1.0f + m00 - m11 - m22) * 2.0f;
        q = {0.25f * s, (m01 + m10) / s, (m02 + m20) / s,
             (m21 - m12) / s};
    } else if (m11 > m22) {
        float s = std::sqrt(1.0f + m11 - m00 - m22) * 2.0f;
        q = {(m01 + m10) / s, 0.25f * s, (m12 + m21) / s,
             (m02 - m20) / s};
    } else {
        float s = std::sqrt(1.0f + m22 - m00 - m11) * 2.0f;
        q = {(m02 + m20) / s, (m12 + m21) / s, 0.25f * s,
             (m10 - m01) / s};
    }
    return normalize(q);
}

Vec3 sixDofRotationVector(const Joint& joint, const Body& a, const Body& b) {
    Vec3 xA, yA, zA, xB, yB, zB;
    jointFrame(joint, a, b, xA, yA, zA, xB, yB, zB);
    Quat q = quaternionFromRotationMatrix(
        dot(xA, xB), dot(xA, yB), dot(xA, zB),
        dot(yA, xB), dot(yA, yB), dot(yA, zB),
        dot(zA, xB), dot(zA, yB), dot(zA, zB));
    if (q.w < 0.0f) q = {-q.x, -q.y, -q.z, -q.w};
    Vec3 v{q.x, q.y, q.z};
    float sinHalf = length(v);
    if (sinHalf < 1e-7f) return v * 2.0f;
    float angle = 2.0f * std::atan2(sinHalf, vclamp(q.w, 0.0f, 1.0f));
    return v * (angle / sinHalf);
}

float component(const Vec3& v, int axis) {
    return axis == 0 ? v.x : (axis == 1 ? v.y : v.z);
}

float& component(Vec3& v, int axis) {
    return axis == 0 ? v.x : (axis == 1 ? v.y : v.z);
}

void requireFiniteVec(const Vec3& value, const char* message) {
    if (!finiteVec(value)) throw std::invalid_argument(message);
}

void requireMass(float mass) {
    if (!finiteFloat(mass) || mass < 0.0f)
        throw std::invalid_argument("velox: mass must be finite and non-negative");
}

void requirePositive(float value, const char* message) {
    if (!finiteFloat(value) || value <= 0.0f) throw std::invalid_argument(message);
}

void requireNonNegative(float value, const char* message) {
    if (!finiteFloat(value) || value < 0.0f) throw std::invalid_argument(message);
}

void validateRuntimeBody(const Body& body) {
    if (!finiteVec(body.position) || !finiteQuat(body.orientation) ||
        !finiteVec(body.velocity) || !finiteVec(body.angularVelocity) ||
        !finiteVec(body.force) || !finiteVec(body.torque) ||
        !finiteFloat(body.invMass) || body.invMass < 0.0f ||
        !finiteVec(body.invInertia) || body.invInertia.x < 0.0f ||
        body.invInertia.y < 0.0f || body.invInertia.z < 0.0f ||
        !finiteFloat(body.restitution) ||
        body.restitution < 0.0f || !finiteFloat(body.friction) || body.friction < 0.0f ||
        !finiteVec(body.frictionScale) || body.frictionScale.x < 0.0f ||
        body.frictionScale.y < 0.0f || body.frictionScale.z < 0.0f ||
        !finiteFloat(body.rollingFriction) || body.rollingFriction < 0.0f ||
        !finiteFloat(body.spinningFriction) || body.spinningFriction < 0.0f ||
        !validCombineMode(body.frictionCombine) ||
        !validCombineMode(body.restitutionCombine) ||
        !finiteFloat(body.linearDamping) || body.linearDamping < 0.0f ||
        !finiteFloat(body.angularDamping) || body.angularDamping < 0.0f ||
        !finiteFloat(body.gravityScale))
        throw std::invalid_argument("velox: body contains invalid or non-finite state");
    float q2 = body.orientation.x * body.orientation.x +
               body.orientation.y * body.orientation.y +
               body.orientation.z * body.orientation.z +
               body.orientation.w * body.orientation.w;
    if (!finiteFloat(q2) || q2 < 1e-12f)
        throw std::invalid_argument("velox: body orientation must be a finite non-zero quaternion");
}

} // namespace

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

bool World::isValid(BodyId id) const noexcept {
    uint32_t slot = id.slot();
    return slot < bodySlots_.size() && bodySlots_[slot].dense != UINT32_MAX &&
           bodySlots_[slot].generation == id.generation();
}

bool World::isValid(JointId id) const noexcept {
    uint32_t slot = id.slot();
    return slot < jointSlots_.size() && jointSlots_[slot].dense != UINT32_MAX &&
           jointSlots_[slot].generation == id.generation();
}

BodyIndex World::resolve(BodyId id) const {
    if (!isValid(id)) throw std::out_of_range("velox: invalid or stale body handle");
    return bodySlots_[id.slot()].dense;
}

uint32_t World::resolve(JointId id) const {
    if (!isValid(id)) throw std::out_of_range("velox: invalid or stale joint handle");
    return jointSlots_[id.slot()].dense;
}

BodyId World::bodyHandle(BodyIndex dense) const {
    uint32_t slot = bodyDenseToSlot_[dense];
    return BodyId::make(slot, bodySlots_[slot].generation);
}

BodyId World::addBody(Body bodyValue) {
    if (bodies_.size() >= UINT32_MAX)
        throw std::length_error("velox: body capacity exceeded");
    uint32_t slot;
    if (freeBodySlots_.empty()) {
        slot = static_cast<uint32_t>(bodySlots_.size());
        bodySlots_.push_back({});
    } else {
        slot = freeBodySlots_.back();
        freeBodySlots_.pop_back();
    }
    uint32_t dense = static_cast<uint32_t>(bodies_.size());
    bodies_.push_back(bodyValue);
    bodyDenseToSlot_.push_back(slot);
    bodySlots_[slot].dense = dense;
    return BodyId::make(slot, bodySlots_[slot].generation);
}

JointId World::addJoint(Joint jointValue) {
    if (joints_.size() >= UINT32_MAX)
        throw std::length_error("velox: joint capacity exceeded");
    uint32_t slot;
    if (freeJointSlots_.empty()) {
        slot = static_cast<uint32_t>(jointSlots_.size());
        jointSlots_.push_back({});
    } else {
        slot = freeJointSlots_.back();
        freeJointSlots_.pop_back();
    }
    uint32_t dense = static_cast<uint32_t>(joints_.size());
    joints_.push_back(jointValue);
    jointDenseToSlot_.push_back(slot);
    jointSlots_[slot].dense = dense;
    return JointId::make(slot, jointSlots_[slot].generation);
}

Body& World::body(BodyId id) {
    return bodies_[resolve(id)];
}

const Body& World::body(BodyId id) const {
    return bodies_[resolve(id)];
}

Joint& World::joint(JointId id) {
    return joints_[resolve(id)];
}

const Joint& World::joint(JointId id) const {
    return joints_[resolve(id)];
}

BodyId World::addSphere(Vec3 position, float radius, float mass) {
    requireFiniteVec(position, "velox: sphere position must be finite");
    requirePositive(radius, "velox: sphere radius must be finite and positive");
    requireMass(mass);
    Body b;
    b.position = position;
    b.shape = ShapeType::Sphere;
    b.radius = radius;
    b.motionType = mass > 0.0f ? MotionType::Dynamic : MotionType::Static;
    b.invMass = mass > 0.0f ? 1.0f / mass : 0.0f;
    if (mass > 0.0f) {
        float i = 0.4f * mass * radius * radius; // solid sphere: 2/5 m r^2
        b.invInertia = {1.0f / i, 1.0f / i, 1.0f / i};
    }
    return addBody(b);
}

BodyId World::addBox(Vec3 position, Vec3 halfExtents, float mass) {
    requireFiniteVec(position, "velox: box position must be finite");
    requireFiniteVec(halfExtents, "velox: box half extents must be finite");
    if (halfExtents.x <= 0.0f || halfExtents.y <= 0.0f || halfExtents.z <= 0.0f)
        throw std::invalid_argument("velox: box half extents must be positive");
    requireMass(mass);
    Body b;
    b.position = position;
    b.shape = ShapeType::Box;
    b.halfExtents = halfExtents;
    b.motionType = mass > 0.0f ? MotionType::Dynamic : MotionType::Static;
    b.invMass = mass > 0.0f ? 1.0f / mass : 0.0f;
    if (mass > 0.0f) {
        Vec3 e = halfExtents * 2.0f;
        float k = mass / 12.0f;
        b.invInertia = {1.0f / (k * (e.y * e.y + e.z * e.z)),
                        1.0f / (k * (e.x * e.x + e.z * e.z)),
                        1.0f / (k * (e.x * e.x + e.y * e.y))};
    }
    return addBody(b);
}

BodyId World::addCapsule(Vec3 position, float radius, float halfHeight, float mass) {
    requireFiniteVec(position, "velox: capsule position must be finite");
    requirePositive(radius, "velox: capsule radius must be finite and positive");
    requireNonNegative(halfHeight, "velox: capsule half height must be finite and non-negative");
    requireMass(mass);
    Body b;
    b.position = position;
    b.shape = ShapeType::Capsule;
    b.radius = radius;
    b.capsuleHalfHeight = halfHeight;
    b.motionType = mass > 0.0f ? MotionType::Dynamic : MotionType::Static;
    b.invMass = mass > 0.0f ? 1.0f / mass : 0.0f;
    if (mass > 0.0f) {
        // Uniform solid capsule: split mass by the volumes of the cylinder
        // core and the two hemispheres, then apply the parallel-axis theorem.
        float r = radius;
        float cylinderWeight = 2.0f * halfHeight;
        float sphereWeight = (4.0f / 3.0f) * r;
        float cylinderMass = mass * cylinderWeight /
                             (cylinderWeight + sphereWeight);
        float sphereMass = mass - cylinderMass;
        float iy = 0.5f * cylinderMass * r * r +
                   0.4f * sphereMass * r * r;
        float ix = cylinderMass *
                       (3.0f * r * r + 4.0f * halfHeight * halfHeight) /
                       12.0f +
                   sphereMass *
                       (0.4f * r * r + 0.75f * halfHeight * r +
                        halfHeight * halfHeight);
        b.invInertia = {1.0f / ix, 1.0f / iy, 1.0f / ix};
    }
    return addBody(b);
}

BodyId World::addCylinder(Vec3 position, float radius, float halfHeight, float mass) {
    requireFiniteVec(position, "velox: cylinder position must be finite");
    requirePositive(radius, "velox: cylinder radius must be finite and positive");
    requirePositive(halfHeight,
                    "velox: cylinder half height must be finite and positive");
    requireMass(mass);
    Body b;
    b.position = position;
    b.shape = ShapeType::Cylinder;
    b.radius = radius;
    b.capsuleHalfHeight = halfHeight;
    b.motionType = mass > 0.0f ? MotionType::Dynamic : MotionType::Static;
    b.invMass = mass > 0.0f ? 1.0f / mass : 0.0f;
    if (mass > 0.0f) {
        float height = 2.0f * halfHeight;
        float iy = 0.5f * mass * radius * radius;
        float ix = mass * (3.0f * radius * radius + height * height) / 12.0f;
        b.invInertia = {1.0f / ix, 1.0f / iy, 1.0f / ix};
    }
    return addBody(b);
}

BodyId World::addCone(Vec3 position, float radius, float height, float mass) {
    requireFiniteVec(position, "velox: cone position must be finite");
    requirePositive(radius, "velox: cone radius must be finite and positive");
    requirePositive(height, "velox: cone height must be finite and positive");
    requireMass(mass);
    Body b;
    b.position = position; // center of mass, one quarter-height above the base
    b.shape = ShapeType::Cone;
    b.radius = radius;
    b.capsuleHalfHeight = height * 0.5f;
    b.motionType = mass > 0.0f ? MotionType::Dynamic : MotionType::Static;
    b.invMass = mass > 0.0f ? 1.0f / mass : 0.0f;
    if (mass > 0.0f) {
        float iy = 0.3f * mass * radius * radius;
        float ix = 0.15f * mass * radius * radius +
                   0.0375f * mass * height * height;
        b.invInertia = {1.0f / ix, 1.0f / iy, 1.0f / ix};
    }
    return addBody(b);
}

BodyId World::addConvexHull(Vec3 position, const std::vector<Vec3>& points, float mass) {
    requireFiniteVec(position, "velox: convex hull position must be finite");
    requireMass(mass);
    if (points.size() < 4)
        throw std::invalid_argument("velox: convex hull requires at least four points");
    if (points.size() > UINT32_MAX)
        throw std::invalid_argument("velox: convex hull has too many points");
    for (const Vec3& p : points)
        requireFiniteVec(p, "velox: convex hull points must be finite");

    // Reject point, line, and plane clouds. GJK can represent them, but they
    // have zero volume and cannot produce valid 3D mass properties or EPA seeds.
    const Vec3 p0 = points[0];
    float scale2 = 0.0f;
    size_t i1 = 0;
    for (size_t i = 1; i < points.size(); ++i) {
        float d2 = lengthSq(points[i] - p0);
        if (d2 > scale2) { scale2 = d2; i1 = i; }
    }
    if (scale2 < 1e-12f)
        throw std::invalid_argument("velox: convex hull points are coincident");
    Vec3 edge = points[i1] - p0;
    float bestArea2 = 0.0f;
    size_t i2 = 0;
    for (size_t i = 1; i < points.size(); ++i) {
        float area2 = lengthSq(cross(edge, points[i] - p0));
        if (area2 > bestArea2) { bestArea2 = area2; i2 = i; }
    }
    if (bestArea2 <= scale2 * scale2 * 1e-12f)
        throw std::invalid_argument("velox: convex hull points are collinear");
    Vec3 planeNormal = cross(edge, points[i2] - p0);
    float maxPlaneDistance = 0.0f;
    for (const Vec3& p : points)
        maxPlaneDistance = vmax(maxPlaneDistance, fabsf(dot(planeNormal, p - p0)));
    float scale = sqrtf(scale2);
    if (maxPlaneDistance <= scale * scale * scale * 1e-6f)
        throw std::invalid_argument("velox: convex hull points are coplanar");
    Body b;
    b.position = position;
    b.shape = ShapeType::Hull;
    b.hullFirst = static_cast<uint32_t>(meshes_.hullPoints.size());
    b.hullCount = static_cast<uint32_t>(points.size());
    b.motionType = mass > 0.0f ? MotionType::Dynamic : MotionType::Static;
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
    return addBody(b);
}

BodyId World::addCompound(Vec3 position, const std::vector<CompoundShape>& shapes,
                          float mass) {
    requireFiniteVec(position, "velox: compound position must be finite");
    requireMass(mass);
    if (shapes.empty())
        throw std::invalid_argument("velox: compound requires at least one child shape");
    if (shapes.size() > UINT32_MAX ||
        meshes_.compoundChildren.size() > UINT32_MAX - shapes.size())
        throw std::length_error("velox: compound child capacity exceeded");

    std::vector<CompoundChild> children;
    std::vector<Vec3> hullPoints;
    children.reserve(shapes.size());
    Vec3 aggregateLo{1e30f, 1e30f, 1e30f};
    Vec3 aggregateHi{-1e30f, -1e30f, -1e30f};
    float aggregateRadius = 0.0f;

    for (const CompoundShape& shape : shapes) {
        requireFiniteVec(shape.localPosition,
                         "velox: compound child position must be finite");
        if (!finiteQuat(shape.localOrientation))
            throw std::invalid_argument("velox: compound child orientation must be finite");
        float q2 = shape.localOrientation.x * shape.localOrientation.x +
                   shape.localOrientation.y * shape.localOrientation.y +
                   shape.localOrientation.z * shape.localOrientation.z +
                   shape.localOrientation.w * shape.localOrientation.w;
        if (q2 < 1e-12f)
            throw std::invalid_argument("velox: compound child orientation must be non-zero");

        CompoundChild child;
        child.shape = shape.shape;
        child.localPosition = shape.localPosition;
        child.localOrientation = normalize(shape.localOrientation);
        float bound = 0.0f;
        switch (shape.shape) {
        case ShapeType::Sphere:
            requirePositive(shape.radius,
                            "velox: compound sphere radius must be positive");
            child.radius = shape.radius;
            bound = shape.radius;
            break;
        case ShapeType::Box:
            if (!finiteVec(shape.halfExtents) || shape.halfExtents.x <= 0.0f ||
                shape.halfExtents.y <= 0.0f || shape.halfExtents.z <= 0.0f)
                throw std::invalid_argument("velox: compound box half extents must be positive");
            child.halfExtents = shape.halfExtents;
            bound = length(shape.halfExtents);
            break;
        case ShapeType::Capsule:
            requirePositive(shape.radius,
                            "velox: compound capsule radius must be positive");
            requireNonNegative(shape.capsuleHalfHeight,
                               "velox: compound capsule half height must be non-negative");
            child.radius = shape.radius;
            child.capsuleHalfHeight = shape.capsuleHalfHeight;
            bound = shape.radius + shape.capsuleHalfHeight;
            break;
        case ShapeType::Cylinder:
            requirePositive(shape.radius,
                            "velox: compound cylinder radius must be positive");
            requirePositive(shape.capsuleHalfHeight,
                            "velox: compound cylinder half height must be positive");
            child.radius = shape.radius;
            child.capsuleHalfHeight = shape.capsuleHalfHeight;
            bound = sqrtf(shape.radius * shape.radius +
                          shape.capsuleHalfHeight * shape.capsuleHalfHeight);
            break;
        case ShapeType::Cone: {
            requirePositive(shape.radius,
                            "velox: compound cone radius must be positive");
            requirePositive(shape.capsuleHalfHeight,
                            "velox: compound cone half height must be positive");
            child.radius = shape.radius;
            child.capsuleHalfHeight = shape.capsuleHalfHeight;
            float top = 1.5f * shape.capsuleHalfHeight;
            bound = vmax(top, sqrtf(shape.radius * shape.radius +
                                    0.25f * shape.capsuleHalfHeight *
                                        shape.capsuleHalfHeight));
            break;
        }
        case ShapeType::Hull: {
            if (shape.hullPoints.size() < 4)
                throw std::invalid_argument("velox: compound hull requires at least four points");
            if (shape.hullPoints.size() > UINT32_MAX ||
                hullPoints.size() > UINT32_MAX - shape.hullPoints.size())
                throw std::length_error("velox: compound hull point capacity exceeded");
            for (const Vec3& p : shape.hullPoints) {
                requireFiniteVec(p, "velox: compound hull points must be finite");
                bound = vmax(bound, length(p));
            }
            const Vec3 p0 = shape.hullPoints[0];
            float scale2 = 0.0f;
            size_t i1 = 0;
            for (size_t i = 1; i < shape.hullPoints.size(); ++i) {
                float d2 = lengthSq(shape.hullPoints[i] - p0);
                if (d2 > scale2) { scale2 = d2; i1 = i; }
            }
            if (scale2 < 1e-12f)
                throw std::invalid_argument("velox: compound hull points are coincident");
            Vec3 edge = shape.hullPoints[i1] - p0;
            float bestArea2 = 0.0f;
            size_t i2 = 0;
            for (size_t i = 1; i < shape.hullPoints.size(); ++i) {
                float area2 = lengthSq(cross(edge, shape.hullPoints[i] - p0));
                if (area2 > bestArea2) { bestArea2 = area2; i2 = i; }
            }
            if (bestArea2 <= scale2 * scale2 * 1e-12f)
                throw std::invalid_argument("velox: compound hull points are collinear");
            Vec3 planeNormal = cross(edge, shape.hullPoints[i2] - p0);
            float maxPlaneDistance = 0.0f;
            for (const Vec3& p : shape.hullPoints)
                maxPlaneDistance = vmax(
                    maxPlaneDistance, fabsf(dot(planeNormal, p - p0)));
            float scale = sqrtf(scale2);
            if (maxPlaneDistance <= scale * scale * scale * 1e-6f)
                throw std::invalid_argument("velox: compound hull points are coplanar");
            child.hullFirst = static_cast<uint32_t>(meshes_.hullPoints.size() +
                                                    hullPoints.size());
            child.hullCount = static_cast<uint32_t>(shape.hullPoints.size());
            hullPoints.insert(hullPoints.end(), shape.hullPoints.begin(),
                              shape.hullPoints.end());
            break;
        }
        default:
            throw std::invalid_argument(
                "velox: compound children must be sphere, box, capsule, or hull");
        }
        Vec3 r{bound, bound, bound};
        aggregateLo = vmin(aggregateLo, shape.localPosition - r);
        aggregateHi = vmax(aggregateHi, shape.localPosition + r);
        aggregateRadius = vmax(aggregateRadius, length(shape.localPosition) + bound);
        children.push_back(child);
    }

    Body body;
    body.position = position;
    body.shape = ShapeType::Compound;
    body.compoundFirst = static_cast<uint32_t>(meshes_.compoundChildren.size());
    body.compoundCount = static_cast<uint32_t>(children.size());
    body.radius = aggregateRadius;
    body.motionType = mass > 0.0f ? MotionType::Dynamic : MotionType::Static;
    body.invMass = mass > 0.0f ? 1.0f / mass : 0.0f;
    if (mass > 0.0f) {
        Vec3 e = aggregateHi - aggregateLo;
        float k = mass / 12.0f;
        body.invInertia = {
            1.0f / (k * (e.y * e.y + e.z * e.z) + 1e-9f),
            1.0f / (k * (e.x * e.x + e.z * e.z) + 1e-9f),
            1.0f / (k * (e.x * e.x + e.y * e.y) + 1e-9f)};
    }
    meshes_.hullPoints.insert(meshes_.hullPoints.end(), hullPoints.begin(), hullPoints.end());
    meshes_.compoundChildren.insert(meshes_.compoundChildren.end(),
                                    children.begin(), children.end());
    return addBody(body);
}

BodyId World::addStaticPlane(Vec3 normal, float offset) {
    requireFiniteVec(normal, "velox: plane normal must be finite");
    if (lengthSq(normal) < 1e-12f)
        throw std::invalid_argument("velox: plane normal must be non-zero");
    if (!finiteFloat(offset)) throw std::invalid_argument("velox: plane offset must be finite");
    Body b;
    b.shape = ShapeType::Plane;
    b.motionType = MotionType::Static;
    b.planeNormal = normalize(normal);
    b.planeOffset = offset;
    b.invMass = 0.0f;
    return addBody(b);
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
    if (vertices.size() < 3 || indices.empty() || indices.size() % 3 != 0)
        throw std::invalid_argument("velox: mesh requires vertices and complete triangles");
    if (vertices.size() > UINT32_MAX || indices.size() > UINT32_MAX)
        throw std::invalid_argument("velox: mesh exceeds 32-bit storage limits");
    for (const Vec3& vertex : vertices)
        requireFiniteVec(vertex, "velox: mesh vertices must be finite");
    for (uint32_t index : indices)
        if ((size_t)index >= vertices.size())
            throw std::out_of_range("velox: mesh index is outside the vertex array");
    for (size_t i = 0; i < indices.size(); i += 3) {
        Vec3 e1 = vertices[indices[i + 1]] - vertices[indices[i]];
        Vec3 e2 = vertices[indices[i + 2]] - vertices[indices[i]];
        float edgeScale2 = vmax(lengthSq(e1), lengthSq(e2));
        if (edgeScale2 < 1e-16f || lengthSq(cross(e1, e2)) <= edgeScale2 * edgeScale2 * 1e-12f)
            throw std::invalid_argument("velox: mesh contains a degenerate triangle");
    }
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
    b.motionType = MotionType::Static;
    b.meshIndex = static_cast<uint32_t>(meshes_.meshes.size() - 1);
    b.invMass = 0.0f; // mesh colliders are static (level geometry)
    return addBody(b);
}

BodyId World::addStaticHeightfield(uint32_t width, uint32_t depth, float cellSize,
                                   const std::vector<float>& heights, Vec3 origin) {
    if (width < 2 || depth < 2)
        throw std::invalid_argument("velox: heightfield requires at least 2x2 samples");
    requirePositive(cellSize, "velox: heightfield cell size must be positive");
    requireFiniteVec(origin, "velox: heightfield origin must be finite");
    uint64_t sampleCount = (uint64_t)width * depth;
    if (sampleCount != heights.size() || sampleCount > UINT32_MAX)
        throw std::invalid_argument("velox: heightfield sample count does not match dimensions");
    for (float height : heights)
        if (!finiteFloat(height))
            throw std::invalid_argument("velox: heightfield samples must be finite");
    uint64_t indexCount = (uint64_t)(width - 1) * (depth - 1) * 6;
    if (indexCount > UINT32_MAX)
        throw std::length_error("velox: heightfield exceeds 32-bit index capacity");

    std::vector<Vec3> vertices;
    std::vector<uint32_t> indices;
    vertices.reserve((size_t)sampleCount);
    indices.reserve((size_t)indexCount);
    for (uint32_t z = 0; z < depth; ++z)
        for (uint32_t x = 0; x < width; ++x)
            vertices.push_back(origin + Vec3{x * cellSize, heights[z * width + x],
                                             z * cellSize});
    for (uint32_t z = 0; z + 1 < depth; ++z) {
        for (uint32_t x = 0; x + 1 < width; ++x) {
            uint32_t a = z * width + x;
            uint32_t b = a + 1;
            uint32_t c = a + width;
            uint32_t d = c + 1;
            indices.insert(indices.end(), {a, c, b, b, c, d});
        }
    }
    return addStaticMesh(vertices, indices);
}

MotionType World::motionType(BodyId id) const { return body(id).motionType; }

void World::setMotionType(BodyId id, MotionType type) {
    Body& b = body(id);
    if ((b.shape == ShapeType::Plane || b.shape == ShapeType::Mesh) &&
        type != MotionType::Static)
        throw std::invalid_argument("velox: plane and mesh colliders must remain static");
    if (type == MotionType::Dynamic && b.invMass <= 0.0f)
        throw std::invalid_argument("velox: dynamic motion requires positive mass");
    b.motionType = type;
    b.asleep = 0;
    b.sleepTimer = 0.0f;
    if (type == MotionType::Static) {
        b.velocity = {};
        b.angularVelocity = {};
    }
}

void World::setTransform(BodyId id, Vec3 position, Quat orientation) {
    requireFiniteVec(position, "velox: body position must be finite");
    if (!finiteQuat(orientation))
        throw std::invalid_argument("velox: body orientation must be finite");
    float q2 = orientation.x * orientation.x + orientation.y * orientation.y +
               orientation.z * orientation.z + orientation.w * orientation.w;
    if (q2 < 1e-12f)
        throw std::invalid_argument("velox: body orientation must be non-zero");
    Body& b = body(id);
    if (b.shape == ShapeType::Plane || b.shape == ShapeType::Mesh)
        throw std::invalid_argument("velox: plane and mesh transforms are baked into geometry");
    b.position = position;
    b.orientation = normalize(orientation);
    wake(id);
}

void World::setLinearVelocity(BodyId id, Vec3 velocity) {
    requireFiniteVec(velocity, "velox: linear velocity must be finite");
    Body& b = body(id);
    if (b.isStatic()) throw std::invalid_argument("velox: static bodies cannot have velocity");
    b.velocity = velocity;
    wake(id);
}

void World::setAngularVelocity(BodyId id, Vec3 velocity) {
    requireFiniteVec(velocity, "velox: angular velocity must be finite");
    Body& b = body(id);
    if (b.isStatic()) throw std::invalid_argument("velox: static bodies cannot have velocity");
    b.angularVelocity = velocity;
    wake(id);
}

void World::addForce(BodyId id, Vec3 force) {
    requireFiniteVec(force, "velox: force must be finite");
    Body& b = body(id);
    if (!b.isDynamic()) throw std::invalid_argument("velox: forces require a dynamic body");
    b.force += force;
    wake(id);
}

void World::addForceAtPoint(BodyId id, Vec3 force, Vec3 worldPoint) {
    requireFiniteVec(force, "velox: force must be finite");
    requireFiniteVec(worldPoint, "velox: force point must be finite");
    Body& b = body(id);
    if (!b.isDynamic()) throw std::invalid_argument("velox: forces require a dynamic body");
    b.force += force;
    b.torque += cross(worldPoint - b.position, force);
    wake(id);
}

void World::addTorque(BodyId id, Vec3 torque) {
    requireFiniteVec(torque, "velox: torque must be finite");
    Body& b = body(id);
    if (!b.isDynamic()) throw std::invalid_argument("velox: torque requires a dynamic body");
    b.torque += torque;
    wake(id);
}

void World::addLinearImpulse(BodyId id, Vec3 impulse) {
    requireFiniteVec(impulse, "velox: impulse must be finite");
    Body& b = body(id);
    if (!b.isDynamic()) throw std::invalid_argument("velox: impulses require a dynamic body");
    b.velocity += impulse * b.solverInvMass();
    wake(id);
}

void World::addImpulseAtPoint(BodyId id, Vec3 impulse, Vec3 worldPoint) {
    requireFiniteVec(impulse, "velox: impulse must be finite");
    requireFiniteVec(worldPoint, "velox: impulse point must be finite");
    Body& b = body(id);
    if (!b.isDynamic()) throw std::invalid_argument("velox: impulses require a dynamic body");
    b.velocity += impulse * b.solverInvMass();
    b.angularVelocity += b.invInertiaMul(cross(worldPoint - b.position, impulse));
    wake(id);
}

void World::clearForces(BodyId id) {
    Body& b = body(id);
    b.force = {};
    b.torque = {};
}

WorldSnapshot World::saveSnapshot() const {
    WorldSnapshot snapshot;
    snapshot.owner_ = this;
    snapshot.gravity_ = gravity;
    snapshot.substeps_ = substeps;
    snapshot.bodies_ = bodies_;
    snapshot.bodySlots_.reserve(bodySlots_.size());
    for (const HandleSlot& slot : bodySlots_)
        snapshot.bodySlots_.push_back({slot.dense, slot.generation});
    snapshot.bodyDenseToSlot_ = bodyDenseToSlot_;
    snapshot.freeBodySlots_ = freeBodySlots_;
    snapshot.contacts_ = contacts_;
    snapshot.prevContacts_ = prevContacts_;
    snapshot.joints_ = joints_;
    snapshot.jointSlots_.reserve(jointSlots_.size());
    for (const HandleSlot& slot : jointSlots_)
        snapshot.jointSlots_.push_back({slot.dense, slot.generation});
    snapshot.jointDenseToSlot_ = jointDenseToSlot_;
    snapshot.freeJointSlots_ = freeJointSlots_;
    snapshot.previous_.reserve(prev_.size());
    for (const PrevState& state : prev_)
        snapshot.previous_.push_back({state.position, state.orientation});
    snapshot.pairKeys_ = pairKeys_;
    snapshot.previousPairKeys_ = prevPairKeys_;
    snapshot.unionParent_ = unionParent_;
    snapshot.islandTimer_ = islandTimer_;
    snapshot.contactEvents_ = events_;
    snapshot.jointBreakEvents_ = jointBreakEvents_;
    snapshot.meshes_ = meshes_;
    snapshot.lastStepStats_ = lastStepStats_;
    return snapshot;
}

void World::restoreSnapshot(const WorldSnapshot& snapshot) {
    if (snapshot.owner_ != this)
        throw std::invalid_argument(
            "velox: a snapshot can only be restored to its originating world");

    // Allocate every converted private representation before changing World,
    // giving restore strong exception safety.
    WorldSnapshot staged = snapshot;
    std::vector<HandleSlot> bodySlots;
    bodySlots.reserve(staged.bodySlots_.size());
    for (const WorldSnapshot::Slot& slot : staged.bodySlots_)
        bodySlots.push_back({slot.dense, slot.generation});
    std::vector<HandleSlot> jointSlots;
    jointSlots.reserve(staged.jointSlots_.size());
    for (const WorldSnapshot::Slot& slot : staged.jointSlots_)
        jointSlots.push_back({slot.dense, slot.generation});
    std::vector<PrevState> previous;
    previous.reserve(staged.previous_.size());
    for (const WorldSnapshot::Previous& state : staged.previous_)
        previous.push_back({state.position, state.orientation});

    gravity = staged.gravity_;
    substeps = staged.substeps_;
    bodies_.swap(staged.bodies_);
    bodySlots_.swap(bodySlots);
    bodyDenseToSlot_.swap(staged.bodyDenseToSlot_);
    freeBodySlots_.swap(staged.freeBodySlots_);
    contacts_.swap(staged.contacts_);
    prevContacts_.swap(staged.prevContacts_);
    joints_.swap(staged.joints_);
    jointSlots_.swap(jointSlots);
    jointDenseToSlot_.swap(staged.jointDenseToSlot_);
    freeJointSlots_.swap(staged.freeJointSlots_);
    prev_.swap(previous);
    pairKeys_.swap(staged.pairKeys_);
    prevPairKeys_.swap(staged.previousPairKeys_);
    unionParent_.swap(staged.unionParent_);
    islandTimer_.swap(staged.islandTimer_);
    events_.swap(staged.contactEvents_);
    jointBreakEvents_.swap(staged.jointBreakEvents_);
    std::swap(meshes_, staged.meshes_);
    lastStepStats_ = staged.lastStepStats_;
    backend_->invalidateCaches();
}

JointId World::addBallJoint(BodyId a, BodyId b, Vec3 worldAnchor) {
    BodyIndex ia = resolve(a), ib = resolve(b);
    if (ia == ib) throw std::invalid_argument("velox: a joint requires two different bodies");
    requireFiniteVec(worldAnchor, "velox: ball-joint anchor must be finite");
    Joint j;
    j.type = JointType::Ball;
    j.a = ia; j.b = ib;
    j.localAnchorA = rotateInv(bodies_[ia].orientation, worldAnchor - bodies_[ia].position);
    j.localAnchorB = rotateInv(bodies_[ib].orientation, worldAnchor - bodies_[ib].position);
    return addJoint(j);
}

JointId World::addDistanceJoint(BodyId a, BodyId b, Vec3 worldAnchorA, Vec3 worldAnchorB) {
    BodyIndex ia = resolve(a), ib = resolve(b);
    if (ia == ib) throw std::invalid_argument("velox: a joint requires two different bodies");
    requireFiniteVec(worldAnchorA, "velox: distance-joint anchor must be finite");
    requireFiniteVec(worldAnchorB, "velox: distance-joint anchor must be finite");
    Joint j;
    j.type = JointType::Distance;
    j.a = ia; j.b = ib;
    j.localAnchorA = rotateInv(bodies_[ia].orientation, worldAnchorA - bodies_[ia].position);
    j.localAnchorB = rotateInv(bodies_[ib].orientation, worldAnchorB - bodies_[ib].position);
    j.restLength = length(worldAnchorA - worldAnchorB);
    return addJoint(j);
}

JointId World::addSpringJoint(BodyId a, BodyId b, Vec3 worldAnchorA,
                              Vec3 worldAnchorB, float frequencyHz,
                              float dampingRatio) {
    requirePositive(frequencyHz,
                    "velox: spring frequency must be finite and positive");
    requireNonNegative(dampingRatio,
                       "velox: spring damping ratio must be finite and non-negative");
    JointId id = addDistanceJoint(a, b, worldAnchorA, worldAnchorB);
    Joint& spring = joint(id);
    spring.enableSpring = true;
    spring.springFrequencyHz = frequencyHz;
    spring.springDampingRatio = dampingRatio;
    return id;
}

JointId World::addHingeJoint(BodyId a, BodyId b, Vec3 worldAnchor, Vec3 worldAxis) {
    BodyIndex ia = resolve(a), ib = resolve(b);
    if (ia == ib) throw std::invalid_argument("velox: a joint requires two different bodies");
    requireFiniteVec(worldAnchor, "velox: hinge anchor must be finite");
    requireFiniteVec(worldAxis, "velox: hinge axis must be finite");
    if (lengthSq(worldAxis) < 1e-12f)
        throw std::invalid_argument("velox: hinge axis must be non-zero");
    Joint j;
    j.type = JointType::Hinge;
    j.a = ia; j.b = ib;
    j.localAnchorA = rotateInv(bodies_[ia].orientation, worldAnchor - bodies_[ia].position);
    j.localAnchorB = rotateInv(bodies_[ib].orientation, worldAnchor - bodies_[ib].position);
    Vec3 axis = normalize(worldAxis);
    j.localAxisA = rotateInv(bodies_[ia].orientation, axis);
    j.localAxisB = rotateInv(bodies_[ib].orientation, axis);
    // Shared perpendicular reference: measures the joint angle (0 now).
    Vec3 ref = std::fabs(axis.x) < 0.9f ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
    ref = normalize(cross(axis, ref));
    j.localRefA = rotateInv(bodies_[ia].orientation, ref);
    j.localRefB = rotateInv(bodies_[ib].orientation, ref);
    return addJoint(j);
}

JointId World::addConeTwistJoint(BodyId a, BodyId b, Vec3 worldAnchor, Vec3 worldAxis) {
    BodyIndex ia = resolve(a), ib = resolve(b);
    if (ia == ib) throw std::invalid_argument("velox: a joint requires two different bodies");
    requireFiniteVec(worldAnchor, "velox: cone/twist anchor must be finite");
    requireFiniteVec(worldAxis, "velox: cone/twist axis must be finite");
    if (lengthSq(worldAxis) < 1e-12f)
        throw std::invalid_argument("velox: cone/twist axis must be non-zero");
    Vec3 axis = normalize(worldAxis);
    Vec3 ref = std::fabs(axis.x) < 0.9f ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
    ref = normalize(cross(axis, ref));

    Joint j;
    j.type = JointType::ConeTwist;
    j.a = ia; j.b = ib;
    j.localAnchorA = rotateInv(bodies_[ia].orientation, worldAnchor - bodies_[ia].position);
    j.localAnchorB = rotateInv(bodies_[ib].orientation, worldAnchor - bodies_[ib].position);
    j.localAxisA = rotateInv(bodies_[ia].orientation, axis);
    j.localAxisB = rotateInv(bodies_[ib].orientation, axis);
    j.localRefA = rotateInv(bodies_[ia].orientation, ref);
    j.localRefB = rotateInv(bodies_[ib].orientation, ref);
    return addJoint(j);
}

JointId World::addFixedJoint(BodyId a, BodyId b, Vec3 worldAnchor) {
    BodyIndex ia = resolve(a), ib = resolve(b);
    if (ia == ib) throw std::invalid_argument("velox: a joint requires two different bodies");
    requireFiniteVec(worldAnchor, "velox: fixed-joint anchor must be finite");
    Joint j;
    j.type = JointType::Fixed;
    j.a = ia; j.b = ib;
    j.localAnchorA = rotateInv(bodies_[ia].orientation,
                               worldAnchor - bodies_[ia].position);
    j.localAnchorB = rotateInv(bodies_[ib].orientation,
                               worldAnchor - bodies_[ib].position);
    j.localAxisA = rotateInv(bodies_[ia].orientation, {1, 0, 0});
    j.localAxisB = rotateInv(bodies_[ib].orientation, {1, 0, 0});
    j.localRefA = rotateInv(bodies_[ia].orientation, {0, 1, 0});
    j.localRefB = rotateInv(bodies_[ib].orientation, {0, 1, 0});
    return addJoint(j);
}

JointId World::addPrismaticJoint(BodyId a, BodyId b, Vec3 worldAnchor,
                                 Vec3 worldAxis) {
    BodyIndex ia = resolve(a), ib = resolve(b);
    if (ia == ib) throw std::invalid_argument("velox: a joint requires two different bodies");
    requireFiniteVec(worldAnchor, "velox: prismatic-joint anchor must be finite");
    requireFiniteVec(worldAxis, "velox: prismatic-joint axis must be finite");
    if (lengthSq(worldAxis) < 1e-12f)
        throw std::invalid_argument("velox: prismatic-joint axis must be non-zero");
    Vec3 axis = normalize(worldAxis);
    Vec3 ref = std::fabs(axis.x) < 0.9f ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
    ref = normalize(cross(axis, ref));
    Joint j;
    j.type = JointType::Prismatic;
    j.a = ia; j.b = ib;
    j.localAnchorA = rotateInv(bodies_[ia].orientation,
                               worldAnchor - bodies_[ia].position);
    j.localAnchorB = rotateInv(bodies_[ib].orientation,
                               worldAnchor - bodies_[ib].position);
    j.localAxisA = rotateInv(bodies_[ia].orientation, axis);
    j.localAxisB = rotateInv(bodies_[ib].orientation, axis);
    j.localRefA = rotateInv(bodies_[ia].orientation, ref);
    j.localRefB = rotateInv(bodies_[ib].orientation, ref);
    return addJoint(j);
}

JointId World::addSixDofJoint(BodyId a, BodyId b, Vec3 worldAnchor) {
    BodyIndex ia = resolve(a), ib = resolve(b);
    if (ia == ib) throw std::invalid_argument("velox: a joint requires two different bodies");
    requireFiniteVec(worldAnchor, "velox: 6DoF-joint anchor must be finite");
    Joint j;
    j.type = JointType::SixDof;
    j.a = ia; j.b = ib;
    j.localAnchorA = rotateInv(bodies_[ia].orientation,
                               worldAnchor - bodies_[ia].position);
    j.localAnchorB = rotateInv(bodies_[ib].orientation,
                               worldAnchor - bodies_[ib].position);
    j.localAxisA = rotateInv(bodies_[ia].orientation, {1, 0, 0});
    j.localAxisB = rotateInv(bodies_[ib].orientation, {1, 0, 0});
    j.localRefA = rotateInv(bodies_[ia].orientation, {0, 1, 0});
    j.localRefB = rotateInv(bodies_[ib].orientation, {0, 1, 0});
    return addJoint(j);
}

float World::hingeAngle(JointId id) const {
    const Joint& j = joint(id);
    if (j.type != JointType::Hinge)
        throw std::invalid_argument("velox: hingeAngle requires a hinge joint");
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

float World::coneSwingAngle(JointId id) const {
    const Joint& j = joint(id);
    if (j.type != JointType::ConeTwist)
        throw std::invalid_argument("velox: coneSwingAngle requires a cone/twist joint");
    Vec3 axisA = normalize(rotate(bodies_[j.a].orientation, j.localAxisA));
    Vec3 axisB = normalize(rotate(bodies_[j.b].orientation, j.localAxisB));
    return std::acos(vclamp(dot(axisA, axisB), -1.0f, 1.0f));
}

float World::coneTwistAngle(JointId id) const {
    const Joint& j = joint(id);
    if (j.type != JointType::ConeTwist)
        throw std::invalid_argument("velox: coneTwistAngle requires a cone/twist joint");
    Vec3 axis = normalize(rotate(bodies_[j.a].orientation, j.localAxisA));
    Vec3 refA = rotate(bodies_[j.a].orientation, j.localRefA);
    Vec3 refB = rotate(bodies_[j.b].orientation, j.localRefB);
    refA = normalize(refA - axis * dot(refA, axis));
    refB = normalize(refB - axis * dot(refB, axis));
    if (lengthSq(refA) < 1e-12f || lengthSq(refB) < 1e-12f) return 0.0f;
    return std::atan2(dot(cross(refA, refB), axis), dot(refA, refB));
}

float World::prismaticTranslation(JointId id) const {
    const Joint& j = joint(id);
    if (j.type != JointType::Prismatic)
        throw std::invalid_argument(
            "velox: prismaticTranslation requires a prismatic joint");
    const Body& a = bodies_[j.a];
    const Body& b = bodies_[j.b];
    Vec3 pa = a.position + rotate(a.orientation, j.localAnchorA);
    Vec3 pb = b.position + rotate(b.orientation, j.localAnchorB);
    Vec3 axis = normalize(rotate(a.orientation, j.localAxisA));
    return dot(pb - pa, axis);
}

Vec3 World::sixDofLinearTranslation(JointId id) const {
    const Joint& j = joint(id);
    if (j.type != JointType::SixDof)
        throw std::invalid_argument(
            "velox: sixDofLinearTranslation requires a 6DoF joint");
    const Body& a = bodies_[j.a];
    const Body& b = bodies_[j.b];
    Vec3 pa = a.position + rotate(a.orientation, j.localAnchorA);
    Vec3 pb = b.position + rotate(b.orientation, j.localAnchorB);
    Vec3 xA, yA, zA, xB, yB, zB;
    jointFrame(j, a, b, xA, yA, zA, xB, yB, zB);
    Vec3 d = pb - pa;
    return {dot(d, xA), dot(d, yA), dot(d, zA)};
}

Vec3 World::sixDofAngularRotation(JointId id) const {
    const Joint& j = joint(id);
    if (j.type != JointType::SixDof)
        throw std::invalid_argument(
            "velox: sixDofAngularRotation requires a 6DoF joint");
    return sixDofRotationVector(j, bodies_[j.a], bodies_[j.b]);
}

void World::wake(BodyId id) {
    Body& b = body(id);
    b.asleep = 0;
    b.sleepTimer = 0.0f;
}

bool World::isAwake(BodyId id) const { return !body(id).asleep; }

void World::removeJointDense(uint32_t dense) {
    uint32_t last = static_cast<uint32_t>(joints_.size() - 1);
    uint32_t removedSlot = jointDenseToSlot_[dense];
    if (dense != last) {
        joints_[dense] = joints_[last];
        uint32_t movedSlot = jointDenseToSlot_[last];
        jointDenseToSlot_[dense] = movedSlot;
        jointSlots_[movedSlot].dense = dense;
    }
    joints_.pop_back();
    jointDenseToSlot_.pop_back();

    HandleSlot& slot = jointSlots_[removedSlot];
    slot.dense = UINT32_MAX;
    if (slot.generation != UINT32_MAX) {
        ++slot.generation;
        freeJointSlots_.push_back(removedSlot);
    }
}

void World::removeJoint(JointId id) { removeJointDense(resolve(id)); }

void World::removeBody(BodyId id) {
    BodyIndex dense = resolve(id);

    // Removing an endpoint also removes every constraint that references it.
    for (uint32_t i = static_cast<uint32_t>(joints_.size()); i-- > 0;)
        if (joints_[i].a == dense || joints_[i].b == dense)
            removeJointDense(i);

    BodyIndex last = static_cast<BodyIndex>(bodies_.size() - 1);
    uint32_t removedSlot = bodyDenseToSlot_[dense];
    if (dense != last) {
        bodies_[dense] = bodies_[last];
        uint32_t movedSlot = bodyDenseToSlot_[last];
        bodyDenseToSlot_[dense] = movedSlot;
        bodySlots_[movedSlot].dense = dense;
        for (Joint& joint : joints_) {
            if (joint.a == last) joint.a = dense;
            if (joint.b == last) joint.b = dense;
        }
    }
    bodies_.pop_back();
    bodyDenseToSlot_.pop_back();

    HandleSlot& slot = bodySlots_[removedSlot];
    slot.dense = UINT32_MAX;
    if (slot.generation != UINT32_MAX) {
        ++slot.generation;
        freeBodySlots_.push_back(removedSlot);
    }

    // Dense indices changed, so no contact or sleeping cache may survive.
    contacts_.clear();
    prevContacts_.clear();
    pairKeys_.clear();
    prevPairKeys_.clear();
    events_.clear();
    prev_.clear();
    unionParent_.clear();
    islandTimer_.clear();
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
        Vec3 k = e * (a.solverInvMass() + b.solverInvMass());
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
    if (!x.isDynamic()) return;
    x.velocity += P * (sign * x.solverInvMass());
    x.angularVelocity += x.invInertiaMul(cross(r, P)) * sign;
}

void applyJointImpulse(Joint& joint, Body& a, Body& b,
                       const Vec3& ra, const Vec3& rb,
                       const Vec3& impulse, float signA) {
    applyImpulse(a, ra, impulse, signA);
    applyImpulse(b, rb, impulse, -signA);
    joint.reactionLinearImpulse += impulse * signA;
}

void applyJointAngularImpulse(Joint& joint, Body& a, Body& b,
                              const Vec3& impulse, float signA = 1.0f) {
    if (a.isDynamic()) a.angularVelocity += a.invInertiaMul(impulse) * signA;
    if (b.isDynamic()) b.angularVelocity -= b.invInertiaMul(impulse) * signA;
    joint.reactionAngularImpulse += impulse * signA;
}

Mat3 angularMass(const Body& a, const Body& b) {
    auto col = [&](Vec3 e) { return a.invInertiaMul(e) + b.invInertiaMul(e); };
    return {col({1, 0, 0}), col({0, 1, 0}), col({0, 0, 1})};
}

void solveAngularFrame(Joint& j, Body& a, Body& b, float biasScale) {
    Vec3 xA = normalize(rotate(a.orientation, j.localAxisA));
    Vec3 xB = normalize(rotate(b.orientation, j.localAxisB));
    Vec3 yA = normalize(rotate(a.orientation, j.localRefA));
    Vec3 yB = normalize(rotate(b.orientation, j.localRefB));
    Vec3 zA = cross(xA, yA);
    Vec3 zB = cross(xB, yB);
    Vec3 error = (cross(xB, xA) + cross(yB, yA) + cross(zB, zA)) * 0.5f;
    Vec3 relativeW = a.angularVelocity - b.angularVelocity;
    Vec3 impulse = mul(inverse(angularMass(a, b)),
                       -(relativeW + error * biasScale));
    applyJointAngularImpulse(j, a, b, impulse);
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

    if (ta.shape == ShapeType::Compound) {
        GapProbe best{1e30f, {0, 1, 0}};
        for (uint32_t i = 0; i < ta.compoundCount; ++i) {
            Body child = compoundChildBody(
                ta, soup.compoundChildren[ta.compoundFirst + i]);
            GapProbe gap = gapAt(child, child.position, child.orientation,
                                 tb, tb.position, tb.orientation, soup, searchRadius);
            if (gap.gap < best.gap) best = gap;
        }
        return best;
    }
    if (tb.shape == ShapeType::Compound) {
        GapProbe best{1e30f, {0, 1, 0}};
        for (uint32_t i = 0; i < tb.compoundCount; ++i) {
            Body child = compoundChildBody(
                tb, soup.compoundChildren[tb.compoundFirst + i]);
            GapProbe gap = gapAt(ta, ta.position, ta.orientation,
                                 child, child.position, child.orientation,
                                 soup, searchRadius);
            if (gap.gap < best.gap) best = gap;
        }
        return best;
    }

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
        if (bodies_[j.a].asleep != bodies_[j.b].asleep) {
            bodies_[j.a].asleep = bodies_[j.b].asleep = 0;
            bodies_[j.a].sleepTimer = bodies_[j.b].sleepTimer = 0.0f;
        }
    }

    for (Joint& j : joints_) {
        j.motorImpulse = 0.0f;
        j.limitImpulse = 0.0f;
        j.swingImpulse = 0.0f;
        j.twistImpulse = 0.0f;
        j.springImpulse = 0.0f;
        j.linearMotorImpulse = {};
        j.angularMotorImpulse = {};
        j.linearLimitImpulse = {};
        j.angularLimitImpulse = {};
        j.reactionLinearImpulse = {};
        j.reactionAngularImpulse = {};
        j.broken = false;
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
                applyJointImpulse(j, a, b, ra, rb, P, +1.0f);
                break;
            }
            case JointType::Distance: {
                Vec3 d = pa - pb;
                float len = length(d);
                if (len < 1e-8f) break;
                Vec3 n = d * (1.0f / len);
                Vec3 raxn = cross(ra, n), rbxn = cross(rb, n);
                float k = a.solverInvMass() + b.solverInvMass() +
                          dot(raxn, a.invInertiaMul(raxn)) +
                          dot(rbxn, b.invInertiaMul(rbxn));
                if (k <= 0.0f) break;
                float lambda;
                if (j.enableSpring) {
                    constexpr float kTau = 6.28318530718f;
                    float effectiveMass = 1.0f / k;
                    float omega = kTau * j.springFrequencyHz;
                    float stiffness = effectiveMass * omega * omega;
                    float damping = 2.0f * effectiveMass *
                                    j.springDampingRatio * omega;
                    float softness = 1.0f /
                        (dt * (damping + dt * stiffness));
                    float bias = (len - j.restLength) * dt * stiffness * softness;
                    lambda = -(dot(vel, n) + bias +
                               softness * j.springImpulse) / (k + softness);
                    j.springImpulse += lambda;
                } else {
                    float bias = (len - j.restLength) * (kBeta / dt);
                    lambda = -(dot(vel, n) + bias) / k;
                }
                applyJointImpulse(j, a, b, ra, rb, n * lambda, +1.0f);
                break;
            }
            case JointType::Fixed: {
                Vec3 bias = (pa - pb) * (kBeta / dt);
                Vec3 P = mul(inverse(pointMass(a, b, ra, rb)), -(vel + bias));
                applyJointImpulse(j, a, b, ra, rb, P, +1.0f);
                solveAngularFrame(j, a, b, kBeta / dt);
                break;
            }
            case JointType::Prismatic: {
                Vec3 axis = normalize(rotate(a.orientation, j.localAxisA));
                Vec3 ref = rotate(a.orientation, j.localRefA);
                Vec3 t1 = normalize(ref - axis * dot(ref, axis));
                if (lengthSq(t1) < 1e-12f) {
                    ref = std::fabs(axis.x) < 0.9f ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
                    t1 = normalize(cross(axis, ref));
                }
                Vec3 t2 = cross(axis, t1);
                Mat3 pointK = pointMass(a, b, ra, rb);
                Vec3 error = pa - pb;
                for (Vec3 t : {t1, t2}) {
                    float k = dot(t, mul(pointK, t));
                    if (k <= 0.0f) continue;
                    float lambda = -(dot(vel, t) + dot(error, t) * (kBeta / dt)) / k;
                    applyJointImpulse(j, a, b, ra, rb, t * lambda, +1.0f);
                    vel = anchorVelocity(a, ra) - anchorVelocity(b, rb);
                }
                solveAngularFrame(j, a, b, kBeta / dt);

                float kAxis = dot(axis, mul(pointK, axis));
                if (kAxis > 0.0f) {
                    float axisSpeed = dot(anchorVelocity(b, rb) -
                                          anchorVelocity(a, ra), axis);
                    if (j.enableMotor) {
                        float lambda = (j.motorSpeed - axisSpeed) / kAxis;
                        float maxImpulse = j.maxMotorForce * dt;
                        float oldImpulse = j.motorImpulse;
                        j.motorImpulse = vclamp(oldImpulse + lambda,
                                                -maxImpulse, maxImpulse);
                        lambda = j.motorImpulse - oldImpulse;
                        applyJointImpulse(j, a, b, ra, rb,
                                          axis * lambda, -1.0f);
                        axisSpeed = dot(anchorVelocity(b, rb) -
                                        anchorVelocity(a, ra), axis);
                    }
                    if (j.enableLimit) {
                        float translation = dot(pb - pa, axis);
                        float lambda = 0.0f;
                        float oldImpulse = j.limitImpulse;
                        if (translation < j.lowerLimit) {
                            float bias = (translation - j.lowerLimit) * (kBeta / dt);
                            lambda = -(axisSpeed + bias) / kAxis;
                            j.limitImpulse = vmax(0.0f, oldImpulse + lambda);
                            lambda = j.limitImpulse - oldImpulse;
                        } else if (translation > j.upperLimit) {
                            float bias = (translation - j.upperLimit) * (kBeta / dt);
                            lambda = -(axisSpeed + bias) / kAxis;
                            j.limitImpulse = vmin(0.0f, oldImpulse + lambda);
                            lambda = j.limitImpulse - oldImpulse;
                        }
                        applyJointImpulse(j, a, b, ra, rb,
                                          axis * lambda, -1.0f);
                    }
                }
                break;
            }
            case JointType::SixDof: {
                Vec3 xA, yA, zA, xB, yB, zB;
                jointFrame(j, a, b, xA, yA, zA, xB, yB, zB);
                Vec3 axes[3] = {xA, yA, zA};
                Vec3 d = pb - pa;
                Mat3 pointK = pointMass(a, b, ra, rb);

                for (int axisIndex = 0; axisIndex < 3; ++axisIndex) {
                    uint8_t bit = uint8_t(1u << axisIndex);
                    Vec3 axis = axes[axisIndex];
                    float k = dot(axis, mul(pointK, axis));
                    if (k <= 0.0f) continue;

                    float speed = dot(anchorVelocity(b, rb) -
                                      anchorVelocity(a, ra), axis);
                    if ((j.linearMotorMask & bit) != 0) {
                        float lambda = (component(j.linearMotorSpeed, axisIndex) -
                                        speed) / k;
                        float maxImpulse = component(j.maxLinearMotorForce,
                                                     axisIndex) * dt;
                        float& accumulated = component(j.linearMotorImpulse,
                                                       axisIndex);
                        float oldImpulse = accumulated;
                        accumulated = vclamp(oldImpulse + lambda,
                                             -maxImpulse, maxImpulse);
                        lambda = accumulated - oldImpulse;
                        applyJointImpulse(j, a, b, ra, rb, axis * lambda,
                                          -1.0f);
                        speed = dot(anchorVelocity(b, rb) -
                                    anchorVelocity(a, ra), axis);
                    }

                    if ((j.linearLimitMask & bit) == 0) continue;
                    float position = dot(d, axis);
                    float lower = component(j.lowerLinearLimit, axisIndex);
                    float upper = component(j.upperLinearLimit, axisIndex);
                    float lambda = 0.0f;
                    float& accumulated = component(j.linearLimitImpulse,
                                                   axisIndex);
                    float oldImpulse = accumulated;
                    if (lower == upper) {
                        float bias = (position - lower) * (kBeta / dt);
                        lambda = -(speed + bias) / k;
                        accumulated += lambda;
                    } else if (position < lower) {
                        float bias = (position - lower) * (kBeta / dt);
                        lambda = -(speed + bias) / k;
                        accumulated = vmax(0.0f, oldImpulse + lambda);
                        lambda = accumulated - oldImpulse;
                    } else if (position > upper) {
                        float bias = (position - upper) * (kBeta / dt);
                        lambda = -(speed + bias) / k;
                        accumulated = vmin(0.0f, oldImpulse + lambda);
                        lambda = accumulated - oldImpulse;
                    }
                    if (lambda != 0.0f)
                        applyJointImpulse(j, a, b, ra, rb, axis * lambda,
                                          -1.0f);
                }

                Vec3 rotation = sixDofRotationVector(j, a, b);
                for (int axisIndex = 0; axisIndex < 3; ++axisIndex) {
                    uint8_t bit = uint8_t(1u << axisIndex);
                    Vec3 axis = axes[axisIndex];
                    float k = dot(axis, a.invInertiaMul(axis)) +
                              dot(axis, b.invInertiaMul(axis));
                    if (k <= 0.0f) continue;

                    float speed = dot(b.angularVelocity - a.angularVelocity,
                                      axis);
                    if ((j.angularMotorMask & bit) != 0) {
                        float lambda = (speed -
                                        component(j.angularMotorSpeed,
                                                  axisIndex)) / k;
                        float maxImpulse = component(j.maxAngularMotorTorque,
                                                     axisIndex) * dt;
                        float& accumulated = component(j.angularMotorImpulse,
                                                       axisIndex);
                        float oldImpulse = accumulated;
                        accumulated = vclamp(oldImpulse + lambda,
                                             -maxImpulse, maxImpulse);
                        lambda = accumulated - oldImpulse;
                        applyJointAngularImpulse(j, a, b, axis * lambda);
                        speed = dot(b.angularVelocity - a.angularVelocity,
                                    axis);
                    }

                    if ((j.angularLimitMask & bit) == 0) continue;
                    float angle = component(rotation, axisIndex);
                    float lower = component(j.lowerAngularLimit, axisIndex);
                    float upper = component(j.upperAngularLimit, axisIndex);
                    float lambda = 0.0f;
                    float& accumulated = component(j.angularLimitImpulse,
                                                   axisIndex);
                    float oldImpulse = accumulated;
                    if (lower == upper) {
                        float bias = (angle - lower) * (kBeta / dt);
                        lambda = (speed + bias) / k;
                        accumulated += lambda;
                    } else if (angle < lower) {
                        float bias = (angle - lower) * (kBeta / dt);
                        lambda = (speed + bias) / k;
                        accumulated = vmin(0.0f, oldImpulse + lambda);
                        lambda = accumulated - oldImpulse;
                    } else if (angle > upper) {
                        float bias = (angle - upper) * (kBeta / dt);
                        lambda = (speed + bias) / k;
                        accumulated = vmax(0.0f, oldImpulse + lambda);
                        lambda = accumulated - oldImpulse;
                    }
                    if (lambda != 0.0f)
                        applyJointAngularImpulse(j, a, b, axis * lambda);
                }
                break;
            }
            case JointType::Hinge: {
                // Point constraint...
                Vec3 bias = (pa - pb) * (kBeta / dt);
                Vec3 P = mul(inverse(pointMass(a, b, ra, rb)), -(vel + bias));
                applyJointImpulse(j, a, b, ra, rb, P, +1.0f);
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
                    applyJointAngularImpulse(j, a, b, t * lambda);
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
                        applyJointAngularImpulse(j, a, b, axisA * lambda);
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
                            applyJointAngularImpulse(j, a, b, axisA * lambda);
                        }
                    }
                }
                break;
            }
            case JointType::ConeTwist: {
                // Ball anchor at the joint center.
                Vec3 bias = (pa - pb) * (kBeta / dt);
                Vec3 P = mul(inverse(pointMass(a, b, ra, rb)), -(vel + bias));
                applyJointImpulse(j, a, b, ra, rb, P, +1.0f);

                Vec3 axisA = normalize(rotate(a.orientation, j.localAxisA));
                Vec3 axisB = normalize(rotate(b.orientation, j.localAxisB));
                Vec3 wr = a.angularVelocity - b.angularVelocity;

                if (j.enableSwingLimit) {
                    float angle = std::acos(vclamp(dot(axisA, axisB), -1.0f, 1.0f));
                    if (angle > j.swingLimit) {
                        Vec3 n = cross(axisB, axisA);
                        if (lengthSq(n) < 1e-10f) {
                            Vec3 refA = rotate(a.orientation, j.localRefA);
                            n = cross(axisA, refA);
                        }
                        n = normalize(n);
                        float k = dot(n, a.invInertiaMul(n)) + dot(n, b.invInertiaMul(n));
                        if (k > 0.0f) {
                            float error = angle - j.swingLimit;
                            float lambda = -(dot(wr, n) + error * (kBeta / dt)) / k;
                            float oldImpulse = j.swingImpulse;
                            j.swingImpulse = vmin(0.0f, oldImpulse + lambda);
                            lambda = j.swingImpulse - oldImpulse;
                            applyJointAngularImpulse(j, a, b, n * lambda);
                            wr = a.angularVelocity - b.angularVelocity;
                        }
                    }
                }

                if (j.enableTwistLimit) {
                    Vec3 refA = rotate(a.orientation, j.localRefA);
                    Vec3 refB = rotate(b.orientation, j.localRefB);
                    refA = normalize(refA - axisA * dot(refA, axisA));
                    refB = normalize(refB - axisA * dot(refB, axisA));
                    if (lengthSq(refA) < 1e-10f || lengthSq(refB) < 1e-10f) break;
                    float angle = std::atan2(dot(cross(refA, refB), axisA),
                                             dot(refA, refB));
                    float k = dot(axisA, a.invInertiaMul(axisA)) +
                              dot(axisA, b.invInertiaMul(axisA));
                    if (k > 0.0f) {
                        float wAxis = dot(a.angularVelocity - b.angularVelocity, axisA);
                        float lambda = 0.0f;
                        float oldImpulse = j.twistImpulse;
                        if (angle < j.lowerTwistLimit) {
                            float target = (angle - j.lowerTwistLimit) * (kBeta / dt);
                            lambda = (target - wAxis) / k;
                            j.twistImpulse = vmin(0.0f, oldImpulse + lambda);
                            lambda = j.twistImpulse - oldImpulse;
                        } else if (angle > j.upperTwistLimit) {
                            float target = (angle - j.upperTwistLimit) * (kBeta / dt);
                            lambda = (target - wAxis) / k;
                            j.twistImpulse = vmax(0.0f, oldImpulse + lambda);
                            lambda = j.twistImpulse - oldImpulse;
                        }
                        if (lambda != 0.0f) {
                            applyJointAngularImpulse(j, a, b, axisA * lambda);
                        }
                    }
                }
                break;
            }
            }
        }
    }

    finishBrokenJoints(dt);
}

void World::finishBrokenJoints(float dt) {
    for (Joint& joint : joints_) {
        float force = length(joint.reactionLinearImpulse) / dt;
        float torque = length(joint.reactionAngularImpulse) / dt;
        joint.broken = joint.broken || force > joint.breakForce ||
                       torque > joint.breakTorque;
    }
    for (size_t i = joints_.size(); i-- > 0;) {
        if (!joints_[i].broken) continue;
        const Joint& joint = joints_[i];
        uint32_t slot = jointDenseToSlot_[i];
        jointBreakEvents_.push_back({
            JointId::make(slot, jointSlots_[slot].generation),
            bodyHandle(joint.a), bodyHandle(joint.b),
            length(joint.reactionLinearImpulse) / dt,
            length(joint.reactionAngularImpulse) / dt});
        removeJointDense(static_cast<uint32_t>(i));
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
        if (!bodies_[c.a].isSensor() && !bodies_[c.b].isSensor() &&
            bodies_[c.a].isDynamic() && bodies_[c.b].isDynamic()) unite(c.a, c.b);
    for (const Joint& j : joints_)
        if (bodies_[j.a].isDynamic() && bodies_[j.b].isDynamic()) unite(j.a, j.b);

    for (Body& b : bodies_) {
        if (!b.isDynamic() || b.asleep) continue;
        float motion = lengthSq(b.velocity) + lengthSq(b.angularVelocity);
        b.sleepTimer = motion < kMotionTol ? b.sleepTimer + dt : 0.0f;
    }

    // Minimum timer per island root; islands where everyone is calm sleep.
    islandTimer_.assign(n, 1e30f);
    for (BodyIndex i = 0; i < n; ++i) {
        Body& b = bodies_[i];
        if (!b.isDynamic() || b.asleep) continue;
        uint32_t root = find(i);
        if (b.sleepTimer < islandTimer_[root]) islandTimer_[root] = b.sleepTimer;
    }
    for (BodyIndex i = 0; i < n; ++i) {
        Body& b = bodies_[i];
        if (!b.isDynamic() || b.asleep) continue;
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
    using Clock = std::chrono::steady_clock;
    const auto stepStart = Clock::now();
    lastStepStats_ = {};
    lastStepStats_.dt = dt;
    lastStepStats_.bodyCount = bodies_.size();
    lastStepStats_.jointCount = joints_.size();
    if (!finiteFloat(dt) || dt < 0.0f)
        throw std::invalid_argument("velox: timestep must be finite and non-negative");
    if (dt == 0.0f) {
        for (const Body& body : bodies_)
            if (body.isDynamic() && !body.asleep)
                ++lastStepStats_.awakeDynamicBodies;
        lastStepStats_.totalMs = std::chrono::duration<double, std::milli>(
            Clock::now() - stepStart).count();
        return;
    }
    jointBreakEvents_.clear();
    if (substeps <= 0) throw std::invalid_argument("velox: substeps must be positive");
    if (!finiteVec(gravity)) throw std::invalid_argument("velox: gravity must be finite");
    for (Body& body : bodies_) {
        validateRuntimeBody(body);
        switch (body.shape) {
        case ShapeType::Sphere:
            requirePositive(body.radius, "velox: sphere body has an invalid radius");
            break;
        case ShapeType::Box:
            if (!finiteVec(body.halfExtents) || body.halfExtents.x <= 0.0f ||
                body.halfExtents.y <= 0.0f || body.halfExtents.z <= 0.0f)
                throw std::invalid_argument("velox: box body has invalid half extents");
            break;
        case ShapeType::Capsule:
            requirePositive(body.radius, "velox: capsule body has an invalid radius");
            requireNonNegative(body.capsuleHalfHeight,
                               "velox: capsule body has an invalid half height");
            break;
        case ShapeType::Cylinder:
            requirePositive(body.radius, "velox: cylinder body has an invalid radius");
            requirePositive(body.capsuleHalfHeight,
                            "velox: cylinder body has an invalid half height");
            break;
        case ShapeType::Cone:
            requirePositive(body.radius, "velox: cone body has an invalid radius");
            requirePositive(body.capsuleHalfHeight,
                            "velox: cone body has an invalid half height");
            break;
        case ShapeType::Plane:
            if (!finiteVec(body.planeNormal) || lengthSq(body.planeNormal) < 1e-12f ||
                !finiteFloat(body.planeOffset))
                throw std::invalid_argument("velox: plane body has invalid geometry");
            break;
        case ShapeType::Mesh:
            if ((size_t)body.meshIndex >= meshes_.meshes.size())
                throw std::out_of_range("velox: mesh body references an invalid mesh");
            break;
        case ShapeType::Hull:
            if (body.hullCount < 4 || (size_t)body.hullFirst + body.hullCount >
                                        meshes_.hullPoints.size())
                throw std::out_of_range("velox: hull body references invalid point storage");
            break;
        case ShapeType::Compound:
            if (body.compoundCount == 0 ||
                (size_t)body.compoundFirst + body.compoundCount >
                    meshes_.compoundChildren.size())
                throw std::out_of_range(
                    "velox: compound body references invalid child storage");
            for (uint32_t i = 0; i < body.compoundCount; ++i) {
                const CompoundChild& child =
                    meshes_.compoundChildren[body.compoundFirst + i];
                if (!finiteVec(child.localPosition) ||
                    !finiteQuat(child.localOrientation) ||
                    !isConvexVolume(child.shape))
                    throw std::invalid_argument(
                        "velox: compound child contains invalid state");
            }
            break;
        }
        body.orientation = normalize(body.orientation);
    }
    for (const Joint& joint : joints_) {
        if (!validJointType(joint.type) || joint.a >= bodies_.size() ||
            joint.b >= bodies_.size() || joint.a == joint.b ||
            !finiteVec(joint.localAnchorA) || !finiteVec(joint.localAnchorB) ||
            !finiteVec(joint.localAxisA) || !finiteVec(joint.localAxisB) ||
            !finiteVec(joint.localRefA) || !finiteVec(joint.localRefB))
            throw std::invalid_argument("velox: joint contains invalid body endpoints");
        bool framed = joint.type == JointType::Hinge ||
                      joint.type == JointType::ConeTwist ||
                      joint.type == JointType::Fixed ||
                      joint.type == JointType::Prismatic ||
                      joint.type == JointType::SixDof;
        if (framed && (lengthSq(joint.localAxisA) < 1e-12f ||
                       lengthSq(joint.localAxisB) < 1e-12f ||
                       lengthSq(joint.localRefA) < 1e-12f ||
                       lengthSq(joint.localRefB) < 1e-12f ||
                       lengthSq(cross(joint.localAxisA,
                                      joint.localRefA)) < 1e-12f ||
                       lengthSq(cross(joint.localAxisB,
                                      joint.localRefB)) < 1e-12f))
            throw std::invalid_argument("velox: joint contains an invalid local frame");
        if (!finiteFloat(joint.motorSpeed) || !finiteFloat(joint.maxMotorTorque) ||
            joint.maxMotorTorque < 0.0f || !finiteFloat(joint.maxMotorForce) ||
            joint.maxMotorForce < 0.0f || !finiteFloat(joint.lowerLimit) ||
            !finiteFloat(joint.upperLimit) || joint.lowerLimit > joint.upperLimit ||
            !finiteFloat(joint.swingLimit) || joint.swingLimit < 0.0f ||
            joint.swingLimit > 3.14159265f || !finiteFloat(joint.lowerTwistLimit) ||
            !finiteFloat(joint.upperTwistLimit) ||
            joint.lowerTwistLimit < -3.14159265f ||
            joint.upperTwistLimit > 3.14159265f ||
            joint.lowerTwistLimit > joint.upperTwistLimit)
            throw std::invalid_argument("velox: joint contains invalid motor or limit settings");
        if ((joint.linearLimitMask & ~uint8_t(0x7)) != 0 ||
            (joint.angularLimitMask & ~uint8_t(0x7)) != 0 ||
            (joint.linearMotorMask & ~uint8_t(0x7)) != 0 ||
            (joint.angularMotorMask & ~uint8_t(0x7)) != 0 ||
            !finiteVec(joint.lowerLinearLimit) ||
            !finiteVec(joint.upperLinearLimit) ||
            !finiteVec(joint.lowerAngularLimit) ||
            !finiteVec(joint.upperAngularLimit) ||
            !finiteVec(joint.linearMotorSpeed) ||
            !finiteVec(joint.angularMotorSpeed) ||
            !finiteVec(joint.maxLinearMotorForce) ||
            !finiteVec(joint.maxAngularMotorTorque) ||
            joint.lowerLinearLimit.x > joint.upperLinearLimit.x ||
            joint.lowerLinearLimit.y > joint.upperLinearLimit.y ||
            joint.lowerLinearLimit.z > joint.upperLinearLimit.z ||
            joint.lowerAngularLimit.x > joint.upperAngularLimit.x ||
            joint.lowerAngularLimit.y > joint.upperAngularLimit.y ||
            joint.lowerAngularLimit.z > joint.upperAngularLimit.z ||
            joint.lowerAngularLimit.x < -3.14159265f ||
            joint.lowerAngularLimit.y < -3.14159265f ||
            joint.lowerAngularLimit.z < -3.14159265f ||
            joint.upperAngularLimit.x > 3.14159265f ||
            joint.upperAngularLimit.y > 3.14159265f ||
            joint.upperAngularLimit.z > 3.14159265f ||
            joint.maxLinearMotorForce.x < 0.0f ||
            joint.maxLinearMotorForce.y < 0.0f ||
            joint.maxLinearMotorForce.z < 0.0f ||
            joint.maxAngularMotorTorque.x < 0.0f ||
            joint.maxAngularMotorTorque.y < 0.0f ||
            joint.maxAngularMotorTorque.z < 0.0f)
            throw std::invalid_argument(
                "velox: joint contains invalid 6DoF settings");
        if (!finiteFloat(joint.springFrequencyHz) ||
            !finiteFloat(joint.springDampingRatio) ||
            joint.springFrequencyHz < 0.0f || joint.springDampingRatio < 0.0f ||
            (joint.enableSpring && (joint.type != JointType::Distance ||
                                    joint.springFrequencyHz <= 0.0f)))
            throw std::invalid_argument("velox: joint contains invalid spring settings");
        if (!finiteFloat(joint.breakForce) || !finiteFloat(joint.breakTorque) ||
            joint.breakForce < 0.0f || joint.breakTorque < 0.0f)
            throw std::invalid_argument("velox: joint contains invalid break thresholds");
    }
    const int nSub = substeps > 0 ? substeps : 1;
    const float h = dt / nSub;

    // Detect ONCE per step, with a speculative reach covering the full dt.
    // The solver substeps below re-evaluate each contact's live gap from
    // current body positions (Contact::bias0), Box2D-v3 style.
    const auto detectionStart = Clock::now();
    backend_->integrate(bodies_, gravity, h); // first substep's gravity
    backend_->findContacts(bodies_, meshes_, dt, contacts_);
    const auto detectionEnd = Clock::now();
    lastStepStats_.generatedContacts = contacts_.size();

    // Joint-connected bodies do not collide by default. Without this filter,
    // contacts at a hinge anchor fight the joint and CCD repeatedly rewinds the
    // pair. Set Joint::collideConnected when the linkage should self-collide.
    if (!joints_.empty() && !contacts_.empty()) {
        std::vector<uint64_t> excluded;
        excluded.reserve(joints_.size());
        for (const Joint& j : joints_) {
            if (j.collideConnected) continue;
            BodyIndex lo = j.a < j.b ? j.a : j.b;
            BodyIndex hi = j.a < j.b ? j.b : j.a;
            excluded.push_back((uint64_t)lo << 32 | hi);
        }
        std::sort(excluded.begin(), excluded.end());
        contacts_.erase(std::remove_if(contacts_.begin(), contacts_.end(),
            [&](const Contact& c) {
                BodyIndex lo = c.a < c.b ? c.a : c.b;
                BodyIndex hi = c.a < c.b ? c.b : c.a;
                return std::binary_search(excluded.begin(), excluded.end(),
                                          (uint64_t)lo << 32 | hi);
            }), contacts_.end());
    }

    if (contactModifier_ && !contacts_.empty()) {
        contacts_.erase(std::remove_if(contacts_.begin(), contacts_.end(),
            [&](Contact& c) {
                ContactModifyData data;
                data.a = bodyHandle(c.a);
                data.b = bodyHandle(c.b);
                data.point = c.point;
                data.normal = c.normal;
                data.restitution = c.restitution;
                data.friction1 = c.friction1;
                data.friction2 = c.friction2;
                data.rollingFriction = c.rollingFriction;
                data.spinningFriction = c.spinningFriction;
                contactModifier_(data);
                if (!data.enabled) return true;
                if (!finiteVec(data.point) || !finiteVec(data.normal) ||
                    lengthSq(data.normal) < 1e-12f ||
                    !finiteFloat(data.restitution) || data.restitution < 0.0f ||
                    !finiteFloat(data.friction1) || data.friction1 < 0.0f ||
                    !finiteFloat(data.friction2) || data.friction2 < 0.0f ||
                    !finiteFloat(data.rollingFriction) || data.rollingFriction < 0.0f ||
                    !finiteFloat(data.spinningFriction) || data.spinningFriction < 0.0f)
                    throw std::invalid_argument(
                        "velox: contact modifier returned invalid contact state");

                c.point = data.point;
                c.normal = normalize(data.normal);
                c.localAnchorA = contactAnchorLocal(bodies_[c.a], c.point);
                c.localAnchorB = contactAnchorLocal(bodies_[c.b], c.point);
                c.bias0 = c.gap;
                c.vn0 = dot(npPointVelocity(bodies_[c.a], c.point) -
                            npPointVelocity(bodies_[c.b], c.point), c.normal);
                c.restitution = data.restitution;
                c.friction1 = data.friction1;
                c.friction2 = data.friction2;
                c.rollingFriction = data.rollingFriction;
                c.spinningFriction = data.spinningFriction;
                return false;
            }), contacts_.end());
    }

    // --- wake pass ------------------------------------------------------------
    // A sleeping body is woken when something awake and moving (or actually
    // striking it) shares a contact with it.
    for (const Contact& c : contacts_) {
        Body& a = bodies_[c.a];
        Body& b = bodies_[c.b];
        if (a.isSensor() || b.isSensor()) continue;
        bool impact = c.vn0 < -0.1f;
        auto moving = [](const Body& x) {
            return lengthSq(x.velocity) + lengthSq(x.angularVelocity) > 1e-3f;
        };
        if (a.asleep && !b.isStatic() && !b.asleep && (impact || moving(b))) {
            a.asleep = 0; a.sleepTimer = 0.0f;
        }
        if (b.asleep && !a.isStatic() && !a.asleep && (impact || moving(a))) {
            b.asleep = 0; b.sleepTimer = 0.0f;
        }
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
                        c.rollingImpulse1 = it->rollingImpulse1;
                        c.rollingImpulse2 = it->rollingImpulse2;
                        c.spinningImpulse = it->spinningImpulse;
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
                    c.rollingImpulse1 = it->rollingImpulse1;
                    c.rollingImpulse2 = it->rollingImpulse2;
                    c.spinningImpulse = it->spinningImpulse;
                }
            }
        }
    }

    prev_.resize(bodies_.size());
    for (size_t i = 0; i < bodies_.size(); ++i)
        prev_[i] = {bodies_[i].position, bodies_[i].orientation};

    for (const Contact& contact : contacts_)
        if (!bodies_[contact.a].isSensor() && !bodies_[contact.b].isSensor())
            ++lastStepStats_.solvedContacts;
    const auto solverStart = Clock::now();

    // --- solver substeps -------------------------------------------------------
    // Each substep: gravity, velocity solve against live gaps, joints, and
    // position integration. Warm starting applies only on the first substep;
    // afterwards the accumulated impulses already live in the velocities.
    bool advancedOnDevice = backend_->advanceSubsteps(
        bodies_, contacts_, joints_, gravity, h, nSub);
    lastStepStats_.deviceSubsteps = advancedOnDevice;
    if (advancedOnDevice) finishBrokenJoints(h);
    if (!advancedOnDevice) {
        for (int s = 0; s < nSub; ++s) {
            if (s > 0) backend_->integrate(bodies_, gravity, h);
            backend_->solveVelocities(bodies_, contacts_, h, s == 0);
            solveJoints(h);
            for (Body& b : bodies_) {
                if (b.isStatic() || b.asleep) continue;
                b.advanceTransform(h);
            }
        }
    }
    const auto solverEnd = Clock::now();

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
        BodyIndex i = (BodyIndex)(key >> 32), j = (BodyIndex)key;
        // Keep A dynamic for the static-dynamic case. Dynamic-dynamic pairs
        // are rewound symmetrically below.
        if (!bodies_[i].isDynamic() && bodies_[j].isDynamic()) {
            BodyIndex t = i; i = j; j = t;
        }
        Body& a = bodies_[i];
        Body& b = bodies_[j];
        if (a.isSensor() || b.isSensor()) continue;
        if (!a.isDynamic()) continue;
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

            Vec3 angularMomentumA = a.worldAngularMomentum();
            Vec3 angularMomentumB = b.isDynamic() ? b.worldAngularMomentum() : Vec3{};
            auto orientationAt = [](const Body& body, const Quat& start,
                                    const Vec3& angularMomentum, float elapsed) {
                Body sample = body;
                sample.orientation = start;
                sample.angularVelocity = sample.invInertiaMul(angularMomentum);
                sample.advanceOrientation(elapsed);
                return sample.orientation;
            };

            float t = 0.0f;
            Vec3 n = end.normal;
            for (int iter = 0; iter < 24 && t < 1.0f; ++iter) {
                Quat qa = orientationAt(a, qa0, angularMomentumA, dt * t);
                Quat qb = b.isStatic()
                    ? qb0
                    : orientationAt(b, qb0, angularMomentumB, dt * t);
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
            a.orientation = qa0;
            a.angularVelocity = a.invInertiaMul(angularMomentumA);
            a.advanceOrientation(dt * t);
            if (b.isDynamic()) {
                b.position = pb0 + db * t;
                b.orientation = qb0;
                b.angularVelocity = b.invInertiaMul(angularMomentumB);
                b.advanceOrientation(dt * t);
            }

            // Remove only the remaining approach velocity with an
            // inverse-mass impulse. Linear momentum is conserved for two
            // dynamic bodies; static geometry receives no velocity update.
            float vn = dot(a.velocity - b.velocity, n);
            float invMassSum = a.solverInvMass() + b.solverInvMass();
            if (vn < 0.0f && invMassSum > 0.0f) {
                float impulse = -vn / invMassSum;
                a.velocity += n * (impulse * a.solverInvMass());
                if (b.isDynamic()) b.velocity -= n * (impulse * b.solverInvMass());
            }

            // Finish the frame from TOI using the corrected velocities. The
            // normal approach component is now zero, so this cannot recreate
            // the crossing, and tangential/center-of-mass motion is retained.
            float remaining = dt * (1.0f - t);
            a.advanceTransform(remaining);
            if (b.isDynamic()) b.advanceTransform(remaining);
        }
    }

    backend_->fetchImpulses(contacts_); // GPU-resident impulses -> host contacts
    const auto ccdEnd = Clock::now();

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
                if (a.isSensor() || b.isSensor()) continue;
                float invMassSum = a.solverInvMass() + b.solverInvMass();
                if (invMassSum <= 0.0f) continue;
                float g = contactLiveGap(a, b, c);
                if (g >= -kPositionSlop) continue;
                float push = -kResolve * (g + kPositionSlop) / invMassSum;
                if (a.isDynamic() && !a.asleep)
                    a.position += c.normal * (push * a.solverInvMass());
                if (b.isDynamic() && !b.asleep)
                    b.position -= c.normal * (push * b.solverInvMass());
            }
        }
    }

    // --- contact and sensor events --------------------------------------------
    // Physical pairs are active when solving produced an impulse or their
    // live anchors are touching. Sensors additionally use the speculative
    // sweep condition, so a fast body crossing an entire trigger in one step
    // still produces a Begin followed by an End.
    {
        events_.clear();
        pairKeys_.clear();
        auto canonicalKey = [](BodyIndex a, BodyIndex b) {
            BodyIndex lo = a < b ? a : b;
            BodyIndex hi = a < b ? b : a;
            return (uint64_t)lo << 32 | hi;
        };
        for (const Contact& c : contacts_) {
            const Body& a = bodies_[c.a];
            const Body& b = bodies_[c.b];
            bool sensor = a.isSensor() || b.isSensor();
            float liveGap = contactLiveGap(a, b, c);
            bool sweptSensorHit = sensor && c.vn0 < 0.0f &&
                                  c.gap <= -c.vn0 * dt + 1e-3f;
            if (c.normalImpulse <= 1e-6f && liveGap > 1e-3f && !sweptSensorHit)
                continue;
            pairKeys_.push_back(canonicalKey(c.a, c.b));
        }
        std::sort(pairKeys_.begin(), pairKeys_.end());
        pairKeys_.erase(std::unique(pairKeys_.begin(), pairKeys_.end()), pairKeys_.end());
        for (uint64_t key : pairKeys_) {
            BodyIndex lo = (BodyIndex)(key >> 32);
            BodyIndex hi = (BodyIndex)key;
            bool persisted = std::binary_search(prevPairKeys_.begin(), prevPairKeys_.end(), key);
            ContactEvent ev{bodyHandle(lo), bodyHandle(hi), {}, {}, 0.0f,
                            persisted ? ContactEventType::Persist : ContactEventType::Begin,
                            bodies_[lo].isSensor() || bodies_[hi].isSensor()};
            bool representativeSet = false;
            for (const Contact& c : contacts_)
                if (canonicalKey(c.a, c.b) == key &&
                    (!representativeSet || c.normalImpulse > ev.impulse)) {
                    representativeSet = true;
                    ev.impulse = c.normalImpulse;
                    ev.point = c.point;
                    ev.normal = c.a == lo ? c.normal : -c.normal;
                }
            events_.push_back(ev);
        }
        for (uint64_t key : prevPairKeys_) {
            if (std::binary_search(pairKeys_.begin(), pairKeys_.end(), key)) continue;
            BodyIndex lo = (BodyIndex)(key >> 32);
            BodyIndex hi = (BodyIndex)key;
            if (lo >= bodies_.size() || hi >= bodies_.size()) continue;
            events_.push_back({bodyHandle(lo), bodyHandle(hi), {}, {}, 0.0f,
                               ContactEventType::End,
                               bodies_[lo].isSensor() || bodies_[hi].isSensor()});
        }
        prevPairKeys_ = pairKeys_;
    }

    // --- sleeping + persistent contacts for next frame ------------------------
    updateSleeping(dt);
    for (Body& body : bodies_) {
        body.force = {};
        body.torque = {};
    }
    prevContacts_ = contacts_;
    std::sort(prevContacts_.begin(), prevContacts_.end(),
              [](const Contact& x, const Contact& y) {
                  return ((uint64_t)x.a << 32 | x.b) < ((uint64_t)y.a << 32 | y.b);
              });

    const auto stepEnd = Clock::now();
    for (const Body& body : bodies_)
        if (body.isDynamic() && !body.asleep)
            ++lastStepStats_.awakeDynamicBodies;
    lastStepStats_.bodyCount = bodies_.size();
    lastStepStats_.jointCount = joints_.size();
    auto milliseconds = [](auto duration) {
        return std::chrono::duration<double, std::milli>(duration).count();
    };
    lastStepStats_.collisionDetectionMs = milliseconds(detectionEnd - detectionStart);
    lastStepStats_.solverMs = milliseconds(solverEnd - solverStart);
    lastStepStats_.ccdMs = milliseconds(ccdEnd - solverEnd);
    lastStepStats_.setupMs = milliseconds(detectionStart - stepStart) +
                             milliseconds(solverStart - detectionEnd);
    lastStepStats_.finalizeMs = milliseconds(stepEnd - ccdEnd);
    lastStepStats_.totalMs = milliseconds(stepEnd - stepStart);
}

} // namespace velox
