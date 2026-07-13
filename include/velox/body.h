#pragma once
#include "math.h"
#include <cstdint>

namespace velox {

using BodyId = uint32_t;

enum class ShapeType : uint8_t { Sphere, Plane, Box, Capsule, Mesh };

// Data-oriented body layout: plain structs in contiguous arrays so the same
// data can be uploaded to a GPU backend unchanged.
struct Body {
    Vec3 position;
    Quat orientation;
    Vec3 velocity;
    Vec3 angularVelocity;
    float invMass = 1.0f;       // 0 => static / infinite mass
    Vec3 invInertia;            // body-space diagonal inverse inertia tensor
    float restitution = 0.3f;
    float friction = 0.5f;

    ShapeType shape = ShapeType::Sphere;
    float radius = 0.5f;        // Sphere, Capsule
    Vec3 halfExtents{0.5f, 0.5f, 0.5f}; // Box
    float capsuleHalfHeight = 0.5f;     // Capsule: half length of the core segment (local Y)
    Vec3 planeNormal{0, 1, 0};  // Plane: dot(n, p) = planeOffset
    float planeOffset = 0.0f;
    uint32_t meshIndex = 0;     // Mesh: index into World's mesh storage

    VELOX_HD bool isStatic() const { return invMass == 0.0f; }

    // World-space inverse-inertia multiply: I⁻¹_world * v = R (I⁻¹_body (Rᵀ v))
    VELOX_HD Vec3 invInertiaMul(const Vec3& v) const {
        Vec3 local = rotateInv(orientation, v);
        return rotate(orientation, {local.x * invInertia.x,
                                    local.y * invInertia.y,
                                    local.z * invInertia.z});
    }

    // Conservative bound on how far any surface point can move per second:
    // linear speed plus angular speed times the shape's bounding radius.
    VELOX_HD float maxPointSpeed() const {
        float r = radius;
        if (shape == ShapeType::Box) r = length(halfExtents);
        else if (shape == ShapeType::Capsule) r = capsuleHalfHeight + radius;
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
