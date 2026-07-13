#pragma once
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

enum class ShapeType : uint8_t { Sphere, Plane, Box, Capsule, Mesh, Hull, Compound };
enum class MotionType : uint8_t { Static, Kinematic, Dynamic };

// Host-side description of one locally transformed convex child. Compound
// bodies accept spheres, boxes, capsules, and convex hulls.
struct CompoundShape {
    ShapeType shape = ShapeType::Sphere;
    Vec3 localPosition;
    Quat localOrientation;
    float radius = 0.5f;
    Vec3 halfExtents{0.5f, 0.5f, 0.5f};
    float capsuleHalfHeight = 0.5f;
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
    Vec3 invInertia;            // body-space diagonal inverse inertia tensor
    float restitution = 0.3f;
    float friction = 0.5f;
    float linearDamping = 0.0f;
    float angularDamping = 0.0f;
    float gravityScale = 1.0f;
    MotionType motionType = MotionType::Dynamic;
    uint32_t categoryBits = 1u;
    uint32_t maskBits = UINT32_MAX;
    uint8_t sensor = 0;

    uint8_t asleep = 0;         // sleeping bodies skip integration and solving
    float sleepTimer = 0.0f;    // seconds below the motion threshold

    ShapeType shape = ShapeType::Sphere;
    float radius = 0.5f;        // Sphere, Capsule
    Vec3 halfExtents{0.5f, 0.5f, 0.5f}; // Box
    float capsuleHalfHeight = 0.5f;     // Capsule: half length of the core segment (local Y)
    Vec3 planeNormal{0, 1, 0};  // Plane: dot(n, p) = planeOffset
    float planeOffset = 0.0f;
    uint32_t meshIndex = 0;     // Mesh: index into World's mesh storage
    uint32_t hullFirst = 0, hullCount = 0; // Hull: local-space points in soup
    uint32_t compoundFirst = 0, compoundCount = 0;

    VELOX_HD bool isStatic() const { return motionType == MotionType::Static; }
    VELOX_HD bool isKinematic() const { return motionType == MotionType::Kinematic; }
    VELOX_HD bool isDynamic() const { return motionType == MotionType::Dynamic; }
    VELOX_HD bool isSensor() const { return sensor != 0; }
    VELOX_HD bool canCollideWith(const Body& other) const {
        return (maskBits & other.categoryBits) != 0 &&
               (other.maskBits & categoryBits) != 0;
    }
    VELOX_HD float solverInvMass() const { return isDynamic() ? invMass : 0.0f; }

    // World-space inverse-inertia multiply: I⁻¹_world * v = R (I⁻¹_body (Rᵀ v))
    VELOX_HD Vec3 invInertiaMul(const Vec3& v) const {
        if (!isDynamic()) return {};
        Vec3 local = rotateInv(orientation, v);
        return rotate(orientation, {local.x * invInertia.x,
                                    local.y * invInertia.y,
                                    local.z * invInertia.z});
    }

    // Conservative bound on how far any surface point can move per second:
    // linear speed plus angular speed times the shape's bounding radius.
    // For hulls, radius holds the bounding radius (set at creation).
    VELOX_HD float maxPointSpeed() const {
        float r = radius;
        if (shape == ShapeType::Box) r = length(halfExtents);
        else if (shape == ShapeType::Capsule) r = capsuleHalfHeight + radius;
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
