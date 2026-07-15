#pragma once
#include "velox/backend.h"
#include "gjk.h"

// Header-only, VELOX_HD face clipping for persistent convex manifolds. The
// narrow phase owns the final Contact records; this layer only produces stable
// witness points, depths, and feature pairs.

namespace velox {

constexpr int kMaxManifoldPoints = 8;

struct ManifoldPoint {
    Vec3 normal;
    Vec3 pointA, pointB;
    Vec3 localAnchorA, localAnchorB;
    float gap;
    uint32_t featureIdA, featureIdB;
    float restitution;
    float friction1, friction2;
    float rollingFriction, spinningFriction;
};

struct Manifold {
    BodyIndex a, b;
    uint64_t featureKey;
    Vec3 normal;
    ManifoldPoint points[kMaxManifoldPoints];
    int pointCount = 0;

    VELOX_HD bool hasFeature(uint32_t idA, uint32_t idB) const {
        return featureKey == (((uint64_t(idA) + 1) << 32) | (uint64_t(idB) + 1));
    }
};

namespace manifold_detail {

constexpr uint32_t kEdgeTag = 0x20000000u;
constexpr uint32_t kFaceTag = 0x40000000u;

struct ClipVertex { Vec3 p; uint32_t id; };
struct FacePolygon {
    ClipVertex vertices[64];
    int count = 0;
    uint32_t faceId = kFaceTag;
    float plane = 0.0f;
};
struct ClipVertex2D {
    float u, v;
    uint32_t featureA, featureB;
};

VELOX_HD inline uint32_t edgeFeature(uint32_t a, uint32_t b) {
    uint32_t lo = a < b ? a : b;
    uint32_t hi = a < b ? b : a;
    // A symmetric, deterministic pair hash. The high tag makes it disjoint
    // from raw vertex ids and face ids while keeping the old 32-bit key format.
    return kEdgeTag | (((lo * 0x1f123bb5u) ^ (hi * 0x5f356495u)) & 0x1fffffffu);
}

VELOX_HD inline uint32_t faceFeature(const uint32_t* ids, int count) {
    uint32_t sorted[64];
    for (int i = 0; i < count; ++i) sorted[i] = ids[i];
    for (int i = 1; i < count; ++i) {
        uint32_t key = sorted[i];
        int j = i - 1;
        while (j >= 0 && sorted[j] > key) { sorted[j + 1] = sorted[j]; --j; }
        sorted[j + 1] = key;
    }
    uint32_t hash = 2166136261u;
    for (int i = 0; i < count; ++i) hash = (hash ^ sorted[i]) * 16777619u;
    return kFaceTag | (hash & 0x1fffffffu);
}

VELOX_HD inline void buildPlaneBasis(const Vec3& normal, Vec3& uAxis, Vec3& vAxis) {
    Vec3 ref = fabsf(normal.x) < 0.9f ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
    uAxis = normalize(cross(normal, ref));
    vAxis = cross(normal, uAxis);
}

VELOX_HD inline void orderCounterClockwise(ClipVertex* vertices, int count,
                                            const Vec3& uAxis, const Vec3& vAxis) {
    if (count < 2) return;
    Vec3 center{};
    for (int i = 0; i < count; ++i) center += vertices[i].p;
    center *= 1.0f / float(count);
    for (int i = 1; i < count; ++i) {
        ClipVertex key = vertices[i];
        float keyAngle = atan2f(dot(key.p - center, vAxis), dot(key.p - center, uAxis));
        int j = i - 1;
        while (j >= 0) {
            float angle = atan2f(dot(vertices[j].p - center, vAxis),
                                 dot(vertices[j].p - center, uAxis));
            if (angle <= keyAngle) break;
            vertices[j + 1] = vertices[j];
            --j;
        }
        vertices[j + 1] = key;
    }
}

VELOX_HD inline int boxFace(const Convex& convex, const Vec3& direction,
                            FacePolygon& out) {
    Vec3 d = rotateInv(convex.orientation, direction);
    float ax = fabsf(d.x), ay = fabsf(d.y), az = fabsf(d.z);
    int axis = ax >= ay && ax >= az ? 0 : (ay >= az ? 1 : 2);
    bool positive = (axis == 0 ? d.x : axis == 1 ? d.y : d.z) >= 0.0f;
    int other0 = axis == 0 ? 1 : 0;
    int other1 = axis == 2 ? 1 : 2;
    out.count = 0;
    for (int bits = 0; bits < 4; ++bits) {
        float values[3] = {-convex.halfExtents.x, -convex.halfExtents.y,
                           -convex.halfExtents.z};
        float extents[3] = {convex.halfExtents.x, convex.halfExtents.y,
                            convex.halfExtents.z};
        values[axis] = positive ? extents[axis] : -extents[axis];
        values[other0] = (bits & 1) ? extents[other0] : -extents[other0];
        values[other1] = (bits & 2) ? extents[other1] : -extents[other1];
        int vertex = (values[0] > 0.0f ? 1 : 0) |
                     (values[1] > 0.0f ? 2 : 0) |
                     (values[2] > 0.0f ? 4 : 0);
        out.vertices[out.count++] = {
            convex.position + rotate(convex.orientation, {values[0], values[1], values[2]}),
            (uint32_t)vertex};
    }
    out.faceId = kFaceTag | uint32_t(axis * 2 + (positive ? 1 : 0));
    return out.count;
}

VELOX_HD inline int hullFace(const Convex& convex, const Vec3& direction,
                             FacePolygon& out) {
    if (!convex.hullPts || convex.hullCount == 0) return 0;
    if (convex.hullFaceIndices && convex.hullFaceCount > 0) {
        int seed = -1;
        Vec3 seedNormal{};
        float bestAlignment = -1e30f;
        for (uint32_t face = 0; face < convex.hullFaceCount; ++face) {
            const uint32_t* indices = convex.hullFaceIndices + face * 3;
            if (indices[0] >= convex.hullCount || indices[1] >= convex.hullCount ||
                indices[2] >= convex.hullCount) return 0;
            Vec3 localNormal = cross(convex.hullPts[indices[1]] - convex.hullPts[indices[0]],
                                     convex.hullPts[indices[2]] - convex.hullPts[indices[0]]);
            if (lengthSq(localNormal) < 1e-16f) return 0;
            Vec3 worldNormal = rotate(convex.orientation, normalize(localNormal));
            float alignment = dot(direction, worldNormal);
            if (alignment > bestAlignment) {
                bestAlignment = alignment;
                seed = (int)face;
                seedNormal = worldNormal;
            }
        }
        // A sloped triangle is not a valid reference face for a vertex or
        // edge contact. Its projection would invent a contact plane; retain
        // the GJK witness fallback unless the face nearly matches the axis.
        if (seed < 0 || bestAlignment < 0.95f) return 0;

        const uint32_t* seedIndices = convex.hullFaceIndices + seed * 3;
        Vec3 seedPoint = convex.position +
                         rotate(convex.orientation, convex.hullPts[seedIndices[0]]);
        float scale = 1.0f;
        for (uint32_t i = 0; i < convex.hullCount; ++i)
            scale = vmax(scale, length(convex.hullPts[i]));
        float planeTolerance = 1e-5f + 1e-4f * scale;
        out.count = 0;
        uint32_t ids[64];
        for (uint32_t face = 0; face < convex.hullFaceCount; ++face) {
            const uint32_t* indices = convex.hullFaceIndices + face * 3;
            Vec3 localNormal = cross(convex.hullPts[indices[1]] - convex.hullPts[indices[0]],
                                     convex.hullPts[indices[2]] - convex.hullPts[indices[0]]);
            if (lengthSq(localNormal) < 1e-16f) return 0;
            Vec3 worldNormal = rotate(convex.orientation, normalize(localNormal));
            Vec3 point = convex.position + rotate(convex.orientation,
                                                   convex.hullPts[indices[0]]);
            if (dot(seedNormal, worldNormal) < 1.0f - 1e-4f ||
                fabsf(dot(seedNormal, point - seedPoint)) > planeTolerance)
                continue;
            for (int corner = 0; corner < 3; ++corner) {
                uint32_t id = indices[corner];
                bool present = false;
                for (int i = 0; i < out.count; ++i)
                    if (out.vertices[i].id == id) present = true;
                if (!present) {
                    if (out.count >= 64) return 0;
                    out.vertices[out.count] = {
                        convex.position + rotate(convex.orientation, convex.hullPts[id]), id};
                    ids[out.count++] = id;
                }
            }
        }
        if (out.count < 3) return 0;
        out.faceId = faceFeature(ids, out.count);
        return out.count;
    }

    // Compatibility fallback for Convex values assembled directly by callers
    // that have a point cloud but no retained QuickHull topology.
    float support = -1e30f, minSupport = 1e30f;
    Vec3 world[64];
    int total = convex.hullCount < 64u ? (int)convex.hullCount : 64;
    for (int i = 0; i < total; ++i) {
        world[i] = convex.position + rotate(convex.orientation, convex.hullPts[i]);
        float projection = dot(direction, world[i]);
        support = vmax(support, projection);
        minSupport = vmin(minSupport, projection);
    }
    float tolerance = 1e-5f + 1e-4f * vmax(1.0f, support - minSupport);
    out.count = 0;
    uint32_t ids[64];
    for (int i = 0; i < total; ++i)
        if (dot(direction, world[i]) >= support - tolerance) {
            out.vertices[out.count] = {world[i], (uint32_t)i};
            ids[out.count++] = (uint32_t)i;
        }
    if (out.count < 3) return 0;
    out.faceId = faceFeature(ids, out.count);
    return out.count;
}

VELOX_HD inline int supportFace(const Convex& convex, const Vec3& direction,
                                FacePolygon& out) {
    if (convex.kind == Convex::Box) return boxFace(convex, direction, out);
    if (convex.kind == Convex::Hull) return hullFace(convex, direction, out);
    return 0;
}

VELOX_HD inline int clipAgainstEdge(const ClipVertex2D* input, int inputCount,
                                    const ClipVertex2D& edgeA,
                                    const ClipVertex2D& edgeB,
                                    ClipVertex2D* output) {
    if (inputCount == 0) return 0;
    const float edgeU = edgeB.u - edgeA.u, edgeV = edgeB.v - edgeA.v;
    int outputCount = 0;
    for (int i = 0; i < inputCount; ++i) {
        const ClipVertex2D& previous = input[(i + inputCount - 1) % inputCount];
        const ClipVertex2D& current = input[i];
        float previousSide = edgeU * (previous.v - edgeA.v) -
                             edgeV * (previous.u - edgeA.u);
        float currentSide = edgeU * (current.v - edgeA.v) -
                            edgeV * (current.u - edgeA.u);
        bool previousInside = previousSide >= -1e-6f;
        bool currentInside = currentSide >= -1e-6f;
        if (previousInside != currentInside && outputCount < 64) {
            float denominator = previousSide - currentSide;
            float t = fabsf(denominator) > 1e-12f ? previousSide / denominator : 0.0f;
            t = vclamp(t, 0.0f, 1.0f);
            output[outputCount++] = {
                previous.u + (current.u - previous.u) * t,
                previous.v + (current.v - previous.v) * t,
                edgeFeature(previous.featureA, current.featureA),
                edgeFeature(edgeA.featureB, edgeB.featureB)};
        }
        if (currentInside && outputCount < 64) output[outputCount++] = current;
    }
    return outputCount;
}

VELOX_HD inline int clipFaces(const FacePolygon& subject, const FacePolygon& clip,
                              const Vec3& normal, ClipVertex2D* output,
                              Vec3& origin, Vec3& uAxis, Vec3& vAxis) {
    buildPlaneBasis(normal, uAxis, vAxis);
    origin = clip.vertices[0].p;
    ClipVertex2D current[64], scratch[64], clip2d[64];
    for (int i = 0; i < subject.count; ++i) {
        Vec3 d = subject.vertices[i].p - origin;
        current[i] = {dot(d, uAxis), dot(d, vAxis), subject.vertices[i].id, clip.faceId};
    }
    for (int i = 0; i < clip.count; ++i) {
        Vec3 d = clip.vertices[i].p - origin;
        clip2d[i] = {dot(d, uAxis), dot(d, vAxis), 0, clip.vertices[i].id};
    }
    int count = subject.count;
    for (int i = 0; i < clip.count && count > 0; ++i) {
        count = clipAgainstEdge(current, count, clip2d[i],
                                clip2d[(i + 1) % clip.count], scratch);
        for (int j = 0; j < count; ++j) current[j] = scratch[j];
    }
    for (int i = 0; i < count; ++i) output[i] = current[i];
    return count;
}

VELOX_HD inline void reducePoints(ManifoldPoint* points, int& count) {
    if (count <= kMaxManifoldPoints) return;
    ManifoldPoint selected[kMaxManifoldPoints];
    bool used[64]{};
    int deepest = 0;
    for (int i = 1; i < count; ++i)
        if (points[i].gap < points[deepest].gap ||
            (points[i].gap == points[deepest].gap &&
             (points[i].featureIdA < points[deepest].featureIdA ||
              (points[i].featureIdA == points[deepest].featureIdA &&
               points[i].featureIdB < points[deepest].featureIdB)))) deepest = i;
    selected[0] = points[deepest];
    used[deepest] = true;
    for (int out = 1; out < kMaxManifoldPoints; ++out) {
        int best = -1;
        float bestDistance = -1.0f;
        for (int i = 0; i < count; ++i) {
            if (used[i]) continue;
            float nearest = 1e30f;
            for (int j = 0; j < out; ++j)
                nearest = vmin(nearest, lengthSq(points[i].pointA - selected[j].pointA));
            if (nearest > bestDistance ||
                (nearest == bestDistance && best >= 0 &&
                 (points[i].featureIdA < points[best].featureIdA ||
                  (points[i].featureIdA == points[best].featureIdA &&
                   points[i].featureIdB < points[best].featureIdB)))) {
                best = i;
                bestDistance = nearest;
            }
        }
        if (best < 0) break;
        selected[out] = points[best];
        used[best] = true;
    }
    for (int i = 0; i < kMaxManifoldPoints; ++i) points[i] = selected[i];
    count = kMaxManifoldPoints;
}

} // namespace manifold_detail

// Normal points from B towards A. A contributes its face facing -normal and B
// its face facing +normal; their planar overlap is the contact manifold.
VELOX_HD inline int clipFaceManifold(const Convex& A, const Convex& B,
                                     const Vec3& normal, Manifold* out) {
    using namespace manifold_detail;
    FacePolygon faceA{}, faceB{};
    if (supportFace(A, -normal, faceA) < 3 || supportFace(B, normal, faceB) < 3)
        return 0;
    Vec3 uAxis, vAxis;
    buildPlaneBasis(normal, uAxis, vAxis);
    orderCounterClockwise(faceA.vertices, faceA.count, uAxis, vAxis);
    orderCounterClockwise(faceB.vertices, faceB.count, uAxis, vAxis);

    faceA.plane = dot(normal, faceA.vertices[0].p);
    faceB.plane = dot(normal, faceB.vertices[0].p);
    ClipVertex2D clipped[64];
    Vec3 origin;
    int count = clipFaces(faceA, faceB, normal, clipped, origin, uAxis, vAxis);
    if (count == 0) return 0;

    float originPlane = dot(normal, origin);
    float gap = faceA.plane - faceB.plane;
    ManifoldPoint candidates[64];
    int pointCount = count < 64 ? count : 64;
    for (int i = 0; i < pointCount; ++i) {
        Vec3 tangentPoint = origin + uAxis * clipped[i].u + vAxis * clipped[i].v;
        ManifoldPoint& point = candidates[i];
        point.normal = normal;
        point.pointA = tangentPoint + normal * (faceA.plane - originPlane);
        point.pointB = tangentPoint + normal * (faceB.plane - originPlane);
        point.localAnchorA = {};
        point.localAnchorB = {};
        point.gap = gap;
        point.featureIdA = clipped[i].featureA;
        point.featureIdB = clipped[i].featureB;
        point.restitution = point.friction1 = point.friction2 = 0.0f;
        point.rollingFriction = point.spinningFriction = 0.0f;
    }
    reducePoints(candidates, pointCount);
    for (int i = 0; i < pointCount; ++i) out->points[i] = candidates[i];
    out->a = 0;
    out->b = 0;
    out->featureKey = 0;
    out->normal = normal;
    out->pointCount = pointCount;
    return pointCount;
}

VELOX_HD inline uint64_t computeFeatureKey(uint32_t idA, uint32_t idB) {
    return ((uint64_t(idA) + 1) << 32) | (uint64_t(idB) + 1);
}

} // namespace velox
