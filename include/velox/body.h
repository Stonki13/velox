#pragma once
#include "ccd.h"
#include "math.h"
#include <cstdint>
#include <vector>

namespace velox {

using BodyIndex = uint32_t;

// Stable external handle. The slot identifies a logical body while the
// generation rejects handles retained after that slot has been removed and
// reused. Simulation internals use BodyIndex for dense CPU/GPU arrays.
struct BodyId {
    uint64_t value = UINT64_MAX;

    VELOX_HD static BodyId make(uint32_t slot, uint32_t generation) {
        BodyId id;
        id.value = (uint64_t(generation) << 32) | slot;
        return id;
    }
    VELOX_HD uint32_t slot() const { return uint32_t(value); }
    VELOX_HD uint32_t generation() const { return uint32_t(value >> 32); }
};

VELOX_HD inline bool operator==(BodyId a, BodyId b) { return a.value == b.value; }
VELOX_HD inline bool operator!=(BodyId a, BodyId b) { return !(a == b); }
VELOX_HD inline bool operator<(BodyId a, BodyId b) { return a.value < b.value; }

enum class ShapeType : uint8_t {
    Sphere, Plane, Box, Capsule, Mesh, Hull, Compound, Cylinder, Cone,
    RoundedBox, Ellipsoid
};
enum class MotionType : uint8_t { Static, Kinematic, Dynamic };
// Controls the amount of numerical work spent on this body's convex geometry.
// Robust and Paranoid are intended for imported or fuzzed geometry; the value
// remains plain data so CPU and CUDA consume the same body representation.
enum class GeometryQuality : uint8_t { Normal, Robust, Paranoid };
// When two modes differ, the mode later in this enum takes precedence.
enum class MaterialCombineMode : uint8_t {
    Average, GeometricMean, Minimum, Multiply, Maximum
};

// Host-side description of one locally transformed convex child. Compound
// bodies accept spheres, boxes, capsules, cylinders, cones, and convex hulls.
struct CompoundShape {
    ShapeType shape = ShapeType::Sphere;
    Vec3 localPosition;
    Quat localOrientation;
    float radius = 0.5f;
    Vec3 halfExtents{0.5f, 0.5f, 0.5f};
    float capsuleHalfHeight = 0.5f; // Capsule/Cylinder: half height; Cone: half total height
    std::vector<Vec3> hullPoints;
};

// Flattened runtime child: pointer-free and shared unchanged with CUDA.
struct CompoundChild {
    ShapeType shape = ShapeType::Sphere;
    Vec3 localPosition;
    Quat localOrientation;
    float radius = 0.5f;
    Vec3 halfExtents{0.5f, 0.5f, 0.5f};
    float capsuleHalfHeight = 0.5f;
    uint32_t hullFirst = 0, hullCount = 0;
    uint32_t hullFaceFirst = 0, hullFaceCount = 0;
};

// Data-oriented body layout: plain structs in contiguous arrays so the same
// data can be uploaded to a GPU backend unchanged.
struct Body {
    Vec3 position;
    Quat orientation;
    Vec3 velocity;
    Vec3 angularVelocity;
    Vec3 force;
    Vec3 torque;
    float invMass = 1.0f;       // 0 => static / infinite mass
    Vec3 invInertia;            // diagonal inverse principal moments
    Quat inertiaOrientation;    // principal-inertia frame to body-local frame
    float restitution = 0.3f;
    float friction = 0.5f;
    Vec3 frictionScale{1, 1, 1}; // local-space anisotropic multiplier
    float rollingFriction = 0.0f;  // angular resistance length scale
    float spinningFriction = 0.0f; // angular resistance length scale
    MaterialCombineMode frictionCombine = MaterialCombineMode::GeometricMean;
    MaterialCombineMode restitutionCombine = MaterialCombineMode::Minimum;
    float linearDamping = 0.0f;
    float angularDamping = 0.0f;
    float gravityScale = 1.0f;
    MotionType motionType = MotionType::Dynamic;
    BodyCcdTuning ccdTuning;
    GeometryQuality geometryQuality = GeometryQuality::Normal;
    uint32_t categoryBits = 1u;
    uint32_t maskBits = UINT32_MAX;
    uint8_t sensor = 0;
    uint8_t enableSleep = 1;    // when 0, body never enters sleep state
    uint8_t fixedRotation = 0;  // when 1, angular velocity is zeroed and orientation is locked

    uint8_t asleep = 0;         // sleeping bodies skip integration and solving
    float sleepTimer = 0.0f;    // seconds below the motion threshold

    ShapeType shape = ShapeType::Sphere;
    float radius = 0.5f;        // Sphere, Capsule
    Vec3 halfExtents{0.5f, 0.5f, 0.5f}; // Box
    float capsuleHalfHeight = 0.5f; // Capsule/Cylinder: half height; Cone: half total height
    Vec3 planeNormal{0, 1, 0};  // Plane: dot(n, p) = planeOffset
    float planeOffset = 0.0f;
    uint32_t meshIndex = 0;     // Mesh: index into World's mesh storage
    uint32_t hullFirst = 0, hullCount = 0; // Hull: local-space points in soup
    uint32_t hullFaceFirst = 0, hullFaceCount = 0; // Packed QuickHull triangles
    uint32_t compoundFirst = 0, compoundCount = 0;

    VELOX_HD bool isStatic() const { return motionType == MotionType::Static; }
    VELOX_HD bool isKinematic() const { return motionType == MotionType::Kinematic; }
    VELOX_HD bool isDynamic() const { return motionType == MotionType::Dynamic; }
    VELOX_HD bool isSensor() const { return sensor != 0; }
    VELOX_HD bool isLocked() const {
        return ccdTuning.quality == MotionQuality::Locked;
    }
    VELOX_HD bool canCollideWith(const Body& other) const {
        return (maskBits & other.categoryBits) != 0 &&
               (other.maskBits & categoryBits) != 0;
    }
    VELOX_HD float solverInvMass() const {
        return isDynamic() && !isLocked() ? invMass : 0.0f;
    }

    VELOX_HD Quat inertiaFrameAt(const Quat& at) const {
        if (inertiaOrientation.x == 0.0f && inertiaOrientation.y == 0.0f &&
            inertiaOrientation.z == 0.0f && inertiaOrientation.w == 1.0f)
            return at;
        return mul(at, inertiaOrientation);
    }

    // World-space inverse-inertia multiply: I⁻¹_world * v = R (I⁻¹_body (Rᵀ v))
    VELOX_HD Vec3 invInertiaMulAt(const Vec3& v, const Quat& at) const {
        if (!isDynamic() || isLocked()) return {};
        Quat frame = inertiaFrameAt(at);
        Vec3 local = rotateInv(frame, v);
        return rotate(frame, {local.x * invInertia.x,
                              local.y * invInertia.y,
                              local.z * invInertia.z});
    }

    VELOX_HD Vec3 invInertiaMul(const Vec3& v) const {
        return invInertiaMulAt(v, orientation);
    }

    VELOX_HD Vec3 inertiaMul(const Vec3& v) const {
        if (isLocked()) return {};
        Quat frame = inertiaFrameAt(orientation);
        Vec3 local = rotateInv(frame, v);
        Vec3 weighted{
            invInertia.x > 0.0f ? local.x / invInertia.x : 0.0f,
            invInertia.y > 0.0f ? local.y / invInertia.y : 0.0f,
            invInertia.z > 0.0f ? local.z / invInertia.z : 0.0f};
        return rotate(frame, weighted);
    }

    VELOX_HD Vec3 worldAngularMomentum() const {
        return inertiaMul(angularVelocity);
    }

    VELOX_HD float rotationalKineticEnergy() const {
        return 0.5f * dot(angularVelocity, worldAngularMomentum());
    }

    // Advance orientation while preserving world angular momentum as an
    // anisotropic inertia tensor rotates. Kinematic angular velocity is a
    // prescribed trajectory and is therefore not reprojected.
    VELOX_HD void advanceOrientation(float dt) {
        if (isLocked()) return;
        bool anisotropic = invInertia.x != invInertia.y ||
                           invInertia.y != invInertia.z;
        if (isDynamic() && anisotropic) {
            Vec3 angularMomentum = worldAngularMomentum();
            Quat start = orientation;
            Vec3 startVelocity = angularVelocity;
            Quat next = integrate(start, startVelocity, dt);
            // Implicit midpoint for q' = 0.5 * omega(q,L) * q. Three fixed
            // point iterations are enough at solver-substep timesteps.
            for (int iteration = 0; iteration < 3; ++iteration) {
                Vec3 endVelocity = invInertiaMulAt(angularMomentum, next);
                Vec3 midpointVelocity = (startVelocity + endVelocity) * 0.5f;
                next = integrate(start, midpointVelocity, dt);
            }
            orientation = next;
            angularVelocity = invInertiaMul(angularMomentum);
        } else {
            orientation = integrate(orientation, angularVelocity, dt);
        }
    }

    VELOX_HD void advanceTransform(float dt) {
        if (isLocked()) return;
        position += velocity * dt;
        advanceOrientation(dt);
    }

    // Conservative bound on how far any surface point can move per second:
    // linear speed plus angular speed times the shape's bounding radius.
    // For hulls, radius holds the bounding radius (set at creation).
    VELOX_HD float maxPointSpeed() const {
        float r = radius;
        if (shape == ShapeType::Box) r = length(halfExtents);
        else if (shape == ShapeType::RoundedBox) r = length(halfExtents) + radius;
        else if (shape == ShapeType::Ellipsoid) r = vmax(halfExtents.x, vmax(halfExtents.y, halfExtents.z));
        else if (shape == ShapeType::Capsule) r = capsuleHalfHeight + radius;
        else if (shape == ShapeType::Cylinder)
            r = sqrtf(capsuleHalfHeight * capsuleHalfHeight + radius * radius);
        else if (shape == ShapeType::Cone) {
            float top = 1.5f * capsuleHalfHeight;
            r = vmax(top, sqrtf(radius * radius + 0.25f * capsuleHalfHeight * capsuleHalfHeight));
        }
        else if (shape == ShapeType::Compound) r = radius;
        return length(velocity) + length(angularVelocity) * r;
    }
};

// Static triangle mesh collider (triangle soup with an AABB for culling).
// Mesh bodies are always static — this matches how game engines use
// non-convex mesh colliders for level geometry.
struct Mesh {
    // Flat arrays: 3 floats per vertex, 3 indices per triangle.
    // Stored by World; Body references it via meshIndex.
    uint32_t firstVertex = 0, vertexCount = 0;
    uint32_t firstIndex = 0, indexCount = 0;
    uint32_t firstNode = 0, nodeCount = 0;   // triangle BVH (flat, GPU-traversable)
    uint32_t firstTriRef = 0;                // leaf triangle references
    Vec3 aabbMin, aabbMax;
};

// Flat BVH node. Leaves hold a range of triangle references; inner nodes
// store the index of their left child, and the right child is left + 1.
struct BvhNode {
    Vec3 aabbMin, aabbMax;
    uint32_t leftFirst = 0; // inner: index of left child (right = left+1); leaf: first tri ref
    uint32_t triCount = 0;  // 0 = inner node, >0 = leaf
};

} // namespace velox
