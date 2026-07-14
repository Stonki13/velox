// manifold_clipping.h — Design sketch for Sutherland-Hodgman convex face clipping.
// Self-contained: includes only what's needed for compilation in isolation.
// TODO: integrate with velox::Vec3, velox::Convex, velox::MeshSoupView.

#pragma once

#include <cstdint>
#include <vector>
#include <array>

// Forward declarations — replace with #include "velox/backend.h" etc. when integrating.
namespace velox {
    struct Vec3;
    struct Convex;
    struct MeshSoupView;
}

namespace velox {

// A 2D point in the clipping plane (local 2D coordinates after projection).
struct ClipPoint2D {
    float u, v;               // coordinates in the local 2D clipping plane
    uint32_t originalIndex;   // index into the original shape's vertex list
    bool isOriginalVertex;    // true if this point was an original vertex (not an edge intersection)
};

// A convex polygon in 2D clipping space, used as input/output for Sutherland-Hodgman.
struct ClipPolygon {
    std::array<ClipPoint2D, 32> points;   // max 32 vertices per polygon
    int count = 0;

    void Clear() { count = 0; }
    void Push(const ClipPoint2D& p) { if (count < 32) points[count++] = p; }
};

// Feature identifier for a clipped manifold point. Encodes which primitive
// element (vertex, edge, face) each point originated from.
struct ManifoldFeatureId {
    uint8_t type : 4;       // 0=none, 1=vertex, 2=edge, 3=face
    uint8_t index : 4;      // vertex/edge/face index
};

// A single point in a persistent contact manifold. Produced by the clipping
// algorithm and consumed by the solver for warm starting.
struct ManifoldPoint {
    Vec3 normal;              // contact normal (from B towards A, world space)
    Vec3 pointA, pointB;      // witness points on each surface
    Vec3 localAnchorA;        // in A's body frame
    Vec3 localAnchorB;        // in B's body frame
    float gap;                // signed distance at detection time
    ManifoldFeatureId featureA;   // which feature of A this point came from
    ManifoldFeatureId featureB;   // which feature of B this point came from
};

// Result of clipping two convex shapes against each other.
struct ClippedManifold {
    std::array<ManifoldPoint, 8> points;    // up to 8 manifold points
    int count = 0;                          // number of valid points
    Vec3 representativeNormal{};            // average normal for warm starting

    void Clear() { count = 0; }
};

// Sutherland-Hodgman convex polygon clipping algorithm.
// Clips the subject polygon against each edge of the clip polygon in sequence.
// Both polygons are in 2D local coordinates on the clipping plane.
//
// Algorithm (standard Sutherland-Hodgman, 4 cases per edge):
//   For each edge of the clip polygon (defined by two consecutive vertices):
//     For each edge of the subject polygon (consecutive vertex pairs):
//       Case 1: both inside  → emit second vertex
//       Case 2: first inside, second outside → emit intersection
//       Case 3: both outside → emit nothing
//       Case 4: first outside, second inside → emit intersection, then second vertex
//   The output of one edge becomes the input for the next edge.
//
// TODO: implement the 4-case edge processing logic with intersection computation.
// TODO: handle degenerate cases (zero-area polygons, collinear edges).
void SutherlandHodgmanClip(
    const ClipPolygon& subject,
    const ClipPolygon& clipEdge,     // single edge of the clip polygon
    ClipPolygon& output);             // written here; must be empty on entry

// Project a 3D convex shape's vertices onto a 2D clipping plane.
// Builds a local 2D coordinate frame from the contact normal using Gram-Schmidt.
//
// Algorithm:
//   1. Pick a world-space reference vector not parallel to the normal (e.g., {1,0,0}
//      unless normal is near-aligned, then use {0,1,0}).
//   2. Compute u-axis = normalize(cross(normal, reference)).
//   3. Compute v-axis = cross(normal, u).
//   4. For each vertex: project (vertex - origin) onto u and v to get (u, v) coords.
//
// TODO: handle the case where the shape has no vertices in the clipping half-space
//       (return empty polygon; caller should fall back to GJK witness point).
void ProjectShapeToClipPlane(
    const Convex& convex,
    const MeshSoupView& soup,
    const Vec3& normal,               // clipping plane normal (world space)
    const Vec3& origin,               // point on the clipping plane
    ClipPolygon& outVertices);        // projected 2D vertices with original indices

// Generate a persistent contact manifold between two convex shapes using
// Sutherland-Hodgman clipping. This is the main entry point for item 01.
//
// Algorithm:
//   1. Get the contact normal from GJK/EPA (already computed by the narrow phase).
//   2. Project both shapes onto the plane perpendicular to the normal.
//   3. Clip shape A's projection against shape B's edges (Sutherland-Hodgman).
//   4. For each clipped vertex, reproject back to 3D by interpolating along the
//      original edge between the two parent vertices.
//   5. Assign feature ids based on which primitive element each point came from.
//   6. Compute per-point material values (restitution, friction) from body properties.
//
// TODO: integrate with the existing narrow phase to receive the GJK/EPA normal
//       and penetration depth; this function only handles the clipping step.
// TODO: handle convex-plane pairs (project only the convex shape, clip against
//       an infinite half-space defined by the plane).
// TODO: handle convex-mesh pairs (project convex, clip against mesh triangle edges
//       within the BVH leaf that contains the contact).
ClippedManifold GeneratePersistentManifold(
    const Convex& A,
    const Convex& B,
    const MeshSoupView& soup,
    const Vec3& normal,               // from B towards A (from GJK/EPA)
    const Vec3& penetrationPoint,     // deepest point on A's surface
    float gap);                       // signed distance at detection

} // namespace velox
