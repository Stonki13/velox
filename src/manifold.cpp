#include "velox/manifold.h"
#include "velox/math.h"
#include <algorithm>
#include <cmath>
#include <vector>

namespace velox {

// 2D point for clipping in the contact plane
struct ClipVertex {
    Vec3 position;      // 3D position (projected to plane)
    uint32_t featureId; // feature identifier
    float distance;     // signed distance to clipping edge
};

// Project 3D point onto 2D plane defined by normal
inline Vec2 projectToPlane(const Vec3& p, const Vec3& normal, const Vec3& origin) {
    Vec3 u, v;
    // Build tangent frame
    if (std::abs(normal.x) < 0.9f) {
        u = normalize(cross(normal, Vec3{1, 0, 0}));
    } else {
        u = normalize(cross(normal, Vec3{0, 1, 0}));
    }
    v = cross(normal, u);

    Vec3 d = p - origin;
    return {dot(d, u), dot(d, v)};
}

// Unproject 2D point back to 3D
inline Vec3 unprojectFromPlane(const Vec2& p2d, const Vec3& normal, const Vec3& origin) {
    Vec3 u, v;
    if (std::abs(normal.x) < 0.9f) {
        u = normalize(cross(normal, Vec3{1, 0, 0}));
    } else {
        u = normalize(cross(normal, Vec3{0, 1, 0}));
    }
    v = cross(normal, u);

    return origin + u * p2d.x + v * p2d.y;
}

// Clip polygon against a single edge (Sutherland-Hodgman step)
inline void clipPolygonEdge(
    const std::vector<ClipVertex>& input,
    const Vec3& edgeStart,
    const Vec3& edgeNormal,
    std::vector<ClipVertex>& output)
{
    output.clear();
    if (input.empty()) return;

    for (size_t i = 0; i < input.size(); ++i) {
        const ClipVertex& current = input[i];
        const ClipVertex& next = input[(i + 1) % input.size()];

        float currentDist = dot(current.position - edgeStart, edgeNormal);
        float nextDist = dot(next.position - edgeStart, edgeNormal);

        bool currentInside = currentDist <= 0.0f;
        bool nextInside = nextDist <= 0.0f;

        if (currentInside) {
            output.push_back(current);
            if (!nextInside) {
                // Compute intersection
                float t = currentDist / (currentDist - nextDist);
                ClipVertex intersection;
                intersection.position = current.position + (next.position - current.position) * t;
                intersection.featureId = current.featureId; // inherit feature
                intersection.distance = 0.0f;
                output.push_back(intersection);
            }
        } else if (nextInside) {
            // Compute intersection
            float t = currentDist / (currentDist - nextDist);
            ClipVertex intersection;
            intersection.position = current.position + (next.position - current.position) * t;
            intersection.featureId = next.featureId; // inherit feature
            intersection.distance = 0.0f;
            output.push_back(intersection);
        }
    }
}

// Generate manifold from two convex polygons using Sutherland-Hodgman clipping
void generateManifold(
    const std::vector<Vec3>& polygonA,
    const std::vector<uint32_t>& featuresA,
    const std::vector<Vec3>& polygonB,
    const std::vector<uint32_t>& featuresB,
    const Vec3& contactNormal,
    const Vec3& contactPoint,
    Manifold& manifold)
{
    manifold.normal = contactNormal;
    manifold.pointCount = 0;

    if (polygonA.empty() || polygonB.empty()) return;

    // Project both polygons onto the contact plane
    std::vector<ClipVertex> polyA, polyB;
    for (size_t i = 0; i < polygonA.size(); ++i) {
        ClipVertex v;
        v.position = polygonA[i];
        v.featureId = featuresA[i];
        v.distance = dot(polygonA[i] - contactPoint, contactNormal);
        polyA.push_back(v);
    }

    for (size_t i = 0; i < polygonB.size(); ++i) {
        ClipVertex v;
        v.position = polygonB[i];
        v.featureId = featuresB[i];
        v.distance = dot(polygonB[i] - contactPoint, contactNormal);
        polyB.push_back(v);
    }

    // Clip polygon A against each edge of polygon B
    std::vector<ClipVertex> clipped = polyA;
    std::vector<ClipVertex> temp;

    for (size_t i = 0; i < polyB.size(); ++i) {
        Vec3 edgeStart = polyB[i].position;
        Vec3 edgeEnd = polyB[(i + 1) % polyB.size()].position;
        Vec3 edgeDir = normalize(edgeEnd - edgeStart);
        Vec3 edgeNormal = cross(contactNormal, edgeDir);

        clipPolygonEdge(clipped, edgeStart, edgeNormal, temp);
        clipped = temp;

        if (clipped.empty()) break;
    }

    // Generate manifold points from clipped polygon
    for (const ClipVertex& v : clipped) {
        if (manifold.pointCount >= Manifold::kMaxManifoldPoints) break;

        ManifoldPoint& mp = manifold.points[manifold.pointCount];
        mp.normal = contactNormal;
        mp.pointA = v.position;
        mp.pointB = v.position; // For now, assume coincident (can refine with GJK)
        mp.gap = v.distance;
        mp.featureIdA = v.featureId;
        mp.featureIdB = 0; // TODO: map to B's feature
        mp.restitution = 0.0f;
        mp.friction1 = 0.5f;
        mp.friction2 = 0.5f;
        mp.rollingFriction = 0.0f;
        mp.spinningFriction = 0.0f;

        manifold.pointCount++;
    }
}

// Generate box face vertices (for box-box or box-plane contacts)
void getBoxFaceVertices(
    const Vec3& center,
    const Vec3& halfExtents,
    const Quat& orientation,
    int faceIndex, // 0-5 for each face
    std::vector<Vec3>& vertices,
    std::vector<uint32_t>& features)
{
    vertices.clear();
    features.clear();

    // Box has 6 faces, each with 4 vertices
    // Face indices: 0=+X, 1=-X, 2=+Y, 3=-Y, 4=+Z, 5=-Z
    Vec3 corners[8] = {
        {-halfExtents.x, -halfExtents.y, -halfExtents.z},
        { halfExtents.x, -halfExtents.y, -halfExtents.z},
        { halfExtents.x,  halfExtents.y, -halfExtents.z},
        {-halfExtents.x,  halfExtents.y, -halfExtents.z},
        {-halfExtents.x, -halfExtents.y,  halfExtents.z},
        { halfExtents.x, -halfExtents.y,  halfExtents.z},
        { halfExtents.x,  halfExtents.y,  halfExtents.z},
        {-halfExtents.x,  halfExtents.y,  halfExtents.z}
    };

    int faceCorners[6][4] = {
        {1, 2, 6, 5}, // +X
        {0, 4, 7, 3}, // -X
        {3, 2, 6, 7}, // +Y
        {0, 1, 5, 4}, // -Y
        {4, 5, 6, 7}, // +Z
        {0, 3, 2, 1}  // -Z
    };

    for (int i = 0; i < 4; ++i) {
        Vec3 local = corners[faceCorners[faceIndex][i]];
        Vec3 world = center + rotate(orientation, local);
        vertices.push_back(world);
        features.push_back(static_cast<uint32_t>(ContactFeature::Face) | (faceIndex << 16));
    }
}

} // namespace velox
