#pragma once
#include "math.h"
#include <array>
#include <cstdint>
#include <vector>

namespace velox {

// Feature identifiers for stable manifold points across frames.
enum class ContactFeature : uint32_t {
    None = 0,
    Vertex = 1u << 0,       // vertex index into hull point cloud or box corner mask
    Edge   = 1u << 1,       // edge index (pair of vertex indices)
    Face   = 1u << 2,       // face index (triangle fan from QuickHull mass faces)
    Implicit = 1u << 30,    // sphere/capsule endpoint tag
    Triangle = 1u << 31,    // triangle index in mesh BVH leaf
};

// Persistent manifold point: carries feature ids so the solver can warm-start
// from previous-frame impulses on the same geometric feature.
struct ManifoldPoint {
    Vec3 normal;              // from B towards A (world space)
    Vec3 pointA, pointB;      // witness points on each surface
    Vec3 localAnchorA;        // in A's body frame
    Vec3 localAnchorB;        // in B's body frame
    float gap;                // signed distance at detection time
    uint32_t featureIdA;      // ContactFeature mask for A
    uint32_t featureIdB;      // ContactFeature mask for B
    float restitution;
    float friction1, friction2;
    float rollingFriction, spinningFriction;
};

// Persistent manifold between two bodies. Up to kMaxManifoldPoints points are
// stored; the solver iterates over them each substep using live gaps.
struct Manifold {
    static constexpr int kMaxManifoldPoints = 8;

    uint32_t a, b;
    uint64_t featureKey;      // stable pair key for warm-start lookup
    Vec3 normal;              // representative contact normal (from B to A)
    std::array<ManifoldPoint, kMaxManifoldPoints> points;
    int pointCount = 0;

    VELOX_HD bool hasFeature(uint32_t idA, uint32_t idB) const {
        return featureKey == (((uint64_t(idA)+1) << 32) | (uint64_t(idB)+1));
    }
};

// Manifold generation result: one manifold per touching pair.
struct ManifoldResult {
    std::vector<Manifold> manifolds;
    int totalPoints = 0;
};

} // namespace velox
