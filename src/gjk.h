#pragma once
#include "velox/backend.h"

// Header-only, VELOX_HD: the exact same GJK runs on CPU and on the GPU.

namespace velox {

// A convex shape for GJK: a core support function plus an inflation radius.
// Spheres and capsules are points/segments with a radius, which keeps their
// GJK cores from ever overlapping in practice; boxes and triangles have
// radius 0.
struct Convex {
    enum Kind : uint8_t { Point, Segment, Box, Triangle, Hull, Cylinder, Cone, RoundedBox, Ellipsoid } kind;
    Vec3 position;      // world transform
    Quat orientation;
    Vec3 halfExtents;   // Box
    Vec3 segA, segB;    // Segment (world), Triangle uses segA/segB/triC
    Vec3 triC;
    const Vec3* hullPts = nullptr; // Hull: local-space point cloud
    uint32_t hullCount = 0;
    const uint32_t* hullFaceIndices = nullptr; // triples of local point indices
    uint32_t hullFaceCount = 0;
    float radius;       // inflation
    float geomRadius = 0.0f; // Cylinder/Cone geometric radius
    float boundingRadius = 0.0f; // local core bound used by numerical tolerances
    GeometryQuality geometryQuality = GeometryQuality::Normal;

    VELOX_HD Vec3 supportOffset(const Vec3& dir) const {
        switch (kind) {
        case Point:
            return {};
        case Segment: {
            Vec3 a = segA - position, b = segB - position;
            return dot(dir, a) > dot(dir, b) ? a : b;
        }
        case Triangle: {
            Vec3 best = segA - position;
            Vec3 b = segB - position, c = triC - position;
            if (dot(dir, b) > dot(dir, best)) best = b;
            if (dot(dir, c) > dot(dir, best)) best = c;
            return best;
        }
        case Box: {
            Vec3 d = rotateInv(orientation, dir);
            Vec3 local{d.x >= 0 ? halfExtents.x : -halfExtents.x,
                       d.y >= 0 ? halfExtents.y : -halfExtents.y,
                       d.z >= 0 ? halfExtents.z : -halfExtents.z};
            return rotate(orientation, local);
        }
        case Hull: {
            Vec3 d = rotateInv(orientation, dir);
            uint32_t best = 0;
            float bestDot = dot(d, hullPts[0]);
            for (uint32_t i = 1; i < hullCount; ++i) {
                float t = dot(d, hullPts[i]);
                if (t > bestDot) { bestDot = t; best = i; }
            }
            return rotate(orientation, hullPts[best]);
        }
        case Cylinder: {
            Vec3 d = rotateInv(orientation, dir);
            float radial = sqrtf(d.x * d.x + d.z * d.z);
            Vec3 local{radial > 1e-9f ? geomRadius * d.x / radial : 0.0f,
                       d.y >= 0.0f ? halfExtents.y : -halfExtents.y,
                       radial > 1e-9f ? geomRadius * d.z / radial : 0.0f};
            return rotate(orientation, local);
        }
        case Cone: {
            Vec3 d = rotateInv(orientation, dir);
            float radial = sqrtf(d.x * d.x + d.z * d.z);
            float halfHeight = halfExtents.y;
            Vec3 base{radial > 1e-9f ? geomRadius * d.x / radial : 0.0f,
                      -0.5f * halfHeight,
                      radial > 1e-9f ? geomRadius * d.z / radial : 0.0f};
            Vec3 tip{0, 1.5f * halfHeight, 0};
            Vec3 local = dot(d, tip) > dot(d, base) ? tip : base;
            return rotate(orientation, local);
        }
        case RoundedBox: {
            Vec3 d = rotateInv(orientation, dir);
            Vec3 core{d.x >= 0 ? halfExtents.x : -halfExtents.x,
                      d.y >= 0 ? halfExtents.y : -halfExtents.y,
                      d.z >= 0 ? halfExtents.z : -halfExtents.z};
            Vec3 n = normalize(d);
            return rotate(orientation, core + n * radius);
        }
        case Ellipsoid: {
            Vec3 d = rotateInv(orientation, dir);
            Vec3 scaled{halfExtents.x * halfExtents.x * d.x,
                        halfExtents.y * halfExtents.y * d.y,
                        halfExtents.z * halfExtents.z * d.z};
            float len = length(scaled);
            Vec3 local = len > 1e-9f ? scaled * (1.0f / len) : Vec3{};
            return rotate(orientation, local);
        }
        }
        return {};
    }

    VELOX_HD Vec3 support(const Vec3& dir) const {
        return position + supportOffset(dir);
    }
};

// soup provides hull point storage; pass {} when b cannot be a hull.
VELOX_HD inline Convex makeConvex(const Body& b, const MeshSoupView& soup) {
    Convex c{};
    c.position = b.position;
    c.orientation = b.orientation;
    c.geometryQuality = b.geometryQuality;
    switch (b.shape) {
    case ShapeType::Sphere:
        c.kind = Convex::Point;
        c.radius = b.radius;
        c.boundingRadius = 0.0f;
        break;
    case ShapeType::Capsule: {
        c.kind = Convex::Segment;
        Vec3 axis = rotate(b.orientation, {0, b.capsuleHalfHeight, 0});
        c.segA = b.position + axis;
        c.segB = b.position - axis;
        c.radius = b.radius;
        c.boundingRadius = b.capsuleHalfHeight;
        break;
    }
    case ShapeType::Box:
        c.kind = Convex::Box;
        c.halfExtents = b.halfExtents;
        c.radius = 0.0f;
        c.boundingRadius = length(b.halfExtents);
        break;
    case ShapeType::Hull:
        c.kind = Convex::Hull;
        c.hullPts = soup.hullPoints + b.hullFirst;
        c.hullCount = b.hullCount;
        c.hullFaceIndices = b.hullFaceCount
            ? soup.hullFaceIndices + b.hullFaceFirst : nullptr;
        c.hullFaceCount = b.hullFaceCount;
        c.radius = 0.0f;
        c.boundingRadius = b.radius;
        break;
    case ShapeType::Cylinder:
        c.kind = Convex::Cylinder;
        c.halfExtents.y = b.capsuleHalfHeight;
        c.geomRadius = b.radius;
        c.radius = 0.0f;
        c.boundingRadius = sqrtf(b.capsuleHalfHeight * b.capsuleHalfHeight +
                                 b.radius * b.radius);
        break;
    case ShapeType::Cone:
        c.kind = Convex::Cone;
        c.halfExtents.y = b.capsuleHalfHeight;
        c.geomRadius = b.radius;
        c.radius = 0.0f;
        c.boundingRadius = vmax(1.5f * b.capsuleHalfHeight,
                                sqrtf(b.radius * b.radius +
                                      0.25f * b.capsuleHalfHeight * b.capsuleHalfHeight));
        break;
    case ShapeType::RoundedBox:
        c.kind = Convex::RoundedBox;
        c.halfExtents = b.halfExtents;
        c.radius = b.radius;
        c.boundingRadius = length(b.halfExtents) + b.radius;
        break;
    case ShapeType::Ellipsoid:
        c.kind = Convex::Ellipsoid;
        c.halfExtents = b.halfExtents;
        c.radius = 0.0f;
        c.boundingRadius = vmax(b.halfExtents.x, vmax(b.halfExtents.y, b.halfExtents.z));
        break;
    default: // Plane/Mesh are not convex volumes; callers handle them separately
        c.kind = Convex::Point;
        c.radius = 0.0f;
        break;
    }
    return c;
}

VELOX_HD inline Convex makeTriangle(const Vec3& a, const Vec3& b, const Vec3& c) {
    Convex t{};
    t.kind = Convex::Triangle;
    t.position = a + ((b - a) + (c - a)) * (1.0f / 3.0f);
    t.segA = a;
    t.segB = b;
    t.triC = c;
    t.radius = 0.0f;
    t.boundingRadius = vmax(length(a - t.position),
                            vmax(length(b - t.position), length(c - t.position)));
    return t;
}

struct GjkResult {
    float distance;   // between inflated surfaces; <= 0 if cores overlap
    Vec3 normal;      // from B towards A
    Vec3 pointA;      // witness point on A's surface
    Vec3 pointB;      // witness point on B's surface
    bool overlapping; // cores intersect: distance/witnesses are a fallback
};

namespace gjk_detail {

VELOX_HD inline GeometryQuality pairQuality(const Convex& A, const Convex& B) {
    return (uint8_t)A.geometryQuality >= (uint8_t)B.geometryQuality
        ? A.geometryQuality : B.geometryQuality;
}

VELOX_HD inline float convexScale(const Convex& c) {
    if (c.boundingRadius > 0.0f) return c.boundingRadius + c.radius;
    if (c.kind == Convex::Box) return length(c.halfExtents) + c.radius;
    if (c.kind == Convex::Segment)
        return 0.5f * length(c.segA - c.segB) + c.radius;
    if (c.kind == Convex::Triangle)
        return vmax(length(c.segA - c.position),
                    vmax(length(c.segB - c.position), length(c.triC - c.position)));
    return c.radius;
}

VELOX_HD inline float pairScale(const Convex& A, const Convex& B) {
    return vmax(1e-8f, convexScale(A) + convexScale(B));
}

VELOX_HD inline float relativeTolerance(const Convex& A, const Convex& B) {
    GeometryQuality quality = pairQuality(A, B);
    return quality == GeometryQuality::Normal ? 1e-7f :
           quality == GeometryQuality::Robust ? 3e-7f : 1e-6f;
}

VELOX_HD inline float duplicateDistanceSq(const Convex& A, const Convex& B) {
    if (pairQuality(A, B) == GeometryQuality::Normal) return 1e-12f;
    float e = pairScale(A, B) * relativeTolerance(A, B);
    return vmax(1e-30f, e * e);
}

VELOX_HD inline float witnessResidual(const Convex& A, const Convex& B,
                                      const GjkResult& result) {
    if (result.overlapping || lengthSq(result.normal) < 0.25f) return 1e30f;
    float aPlane = dot(-result.normal, A.support(-result.normal)) - A.radius;
    float bPlane = dot(result.normal, B.support(result.normal)) + B.radius;
    return fabsf(dot(-result.normal, result.pointA) - aPlane) +
           fabsf(dot(result.normal, result.pointB) - bPlane);
}

struct SupportVert {
    Vec3 p;  // a - b, the Minkowski-difference point
    Vec3 a;  // witness on A's core
    Vec3 b;  // witness on B's core
};

VELOX_HD inline SupportVert supportVertex(const Convex& A, const Convex& B,
                                          const Vec3& dir) {
    SupportVert v;
    Vec3 offsetA = A.supportOffset(dir);
    Vec3 offsetB = B.supportOffset(-dir);
    v.a = A.position + offsetA;
    v.b = B.position + offsetB;
    v.p = (A.position - B.position) + (offsetA - offsetB);
    return v;
}

struct Simplex {
    SupportVert v[4];
    float bary[4];
    int count = 0;
};

// Closest point to the origin on the current simplex; reduces the simplex to
// the minimal feature containing that point and records barycentrics.
VELOX_HD inline void solveSegment(Simplex& s) {
    Vec3 a = s.v[0].p, b = s.v[1].p;
    Vec3 ab = b - a;
    float t = -dot(a, ab);
    if (t <= 0.0f) { s.count = 1; s.bary[0] = 1.0f; return; }
    float denom = dot(ab, ab);
    if (t >= denom) { s.v[0] = s.v[1]; s.count = 1; s.bary[0] = 1.0f; return; }
    t /= denom;
    s.count = 2; s.bary[0] = 1.0f - t; s.bary[1] = t;
}

VELOX_HD inline void solveTriangle(Simplex& s) {
    Vec3 a = s.v[0].p, b = s.v[1].p, c = s.v[2].p;
    Vec3 ab = b - a, ac = c - a, ap = -a;
    float d1 = dot(ab, ap), d2 = dot(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f) { s.count = 1; s.bary[0] = 1.0f; return; }

    Vec3 bp = -b;
    float d3 = dot(ab, bp), d4 = dot(ac, bp);
    if (d3 >= 0.0f && d4 <= d3) { s.v[0] = s.v[1]; s.count = 1; s.bary[0] = 1.0f; return; }

    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
        float t = d1 / (d1 - d3);
        s.count = 2; s.bary[0] = 1.0f - t; s.bary[1] = t; return;
    }

    Vec3 cp = -c;
    float d5 = dot(ab, cp), d6 = dot(ac, cp);
    if (d6 >= 0.0f && d5 <= d6) { s.v[0] = s.v[2]; s.count = 1; s.bary[0] = 1.0f; return; }

    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
        float t = d2 / (d2 - d6);
        s.v[1] = s.v[2];
        s.count = 2; s.bary[0] = 1.0f - t; s.bary[1] = t; return;
    }

    float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
        float t = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        s.v[0] = s.v[1]; s.v[1] = s.v[2];
        s.count = 2; s.bary[0] = 1.0f - t; s.bary[1] = t; return;
    }

    float sum = va + vb + vc;
    if (sum < 1e-20f) {
        // Degenerate (collinear/duplicate) triangle: keep the vertex closest
        // to the origin rather than producing NaN barycentrics.
        int best = 0;
        for (int i = 1; i < 3; ++i)
            if (lengthSq(s.v[i].p) < lengthSq(s.v[best].p)) best = i;
        s.v[0] = s.v[best];
        s.count = 1;
        s.bary[0] = 1.0f;
        return;
    }
    float denom = 1.0f / sum;
    s.count = 3;
    s.bary[0] = va * denom;
    s.bary[1] = vb * denom;
    s.bary[2] = vc * denom;
}

// Returns true if the origin is inside the tetrahedron.
VELOX_HD inline bool solveTetrahedron(Simplex& s, float scale, bool robust) {
    (void)robust;
    float volume = dot(s.v[1].p - s.v[0].p,
                       cross(s.v[2].p - s.v[0].p, s.v[3].p - s.v[0].p));
    float volumeFloor = 1e-10f * scale * scale * scale;
    // A sliver tetrahedron cannot certify containment at any quality level:
    // its face-plane signs drown in rounding noise. Reduce it to its nearest
    // triangle and let the iteration keep converging on the surface instead.
    if (fabsf(volume) < volumeFloor) {
        Simplex best{};
        float bestDist = 1e30f;
        const int faces[4][3] = {{0, 1, 2}, {0, 1, 3}, {0, 2, 3}, {1, 2, 3}};
        for (int f = 0; f < 4; ++f) {
            Simplex tri{};
            tri.count = 3;
            tri.v[0] = s.v[faces[f][0]];
            tri.v[1] = s.v[faces[f][1]];
            tri.v[2] = s.v[faces[f][2]];
            solveTriangle(tri);
            Vec3 closest{};
            for (int i = 0; i < tri.count; ++i) closest += tri.v[i].p * tri.bary[i];
            float distanceSq = lengthSq(closest);
            if (distanceSq < bestDist) { bestDist = distanceSq; best = tri; }
        }
        s = best;
        return false;
    }
    Simplex best{};
    float bestDist = 1e30f;
    bool inside = true;
    // Origin-side tests are normalized to plane distances and compared against
    // a scale-relative band: with support coordinates of magnitude ~scale,
    // float dot/cross noise reaches ~scale * 1e-6, and a raw sign test lets a
    // grazing contact (origin within noise of a face) report "inside", feeding
    // EPA a polytope whose nearest face is at ~zero distance and produces a
    // garbage penetration axis (found by differential testing: a sphere
    // resting on a 100 m static box teleported through it).
    const float boundary = 3e-6f * scale;
    const int faces[4][3] = {{0, 1, 2}, {0, 1, 3}, {0, 2, 3}, {1, 2, 3}};
    for (int f = 0; f < 4; ++f) {
        int i0 = faces[f][0], i1 = faces[f][1], i2 = faces[f][2];
        Vec3 a = s.v[i0].p, b = s.v[i1].p, c = s.v[i2].p;
        Vec3 n = cross(b - a, c - a);
        float nLen = length(n);
        if (nLen < 1e-20f) continue; // degenerate face; volume gate handles it
        Vec3 opposite = s.v[6 - i0 - i1 - i2].p;
        float originPlaneDist = dot(n, -a) / nLen;
        float oppositePlaneDist = dot(n, opposite - a) / nLen;
        if (originPlaneDist * oppositePlaneDist < 0.0f ||
            fabsf(originPlaneDist) <= boundary) {
            inside = false;
            Simplex tri{};
            tri.count = 3;
            tri.v[0] = s.v[i0]; tri.v[1] = s.v[i1]; tri.v[2] = s.v[i2];
            solveTriangle(tri);
            Vec3 closest{};
            for (int i = 0; i < tri.count; ++i) closest += tri.v[i].p * tri.bary[i];
            float d = lengthSq(closest);
            if (d < bestDist) { bestDist = d; best = tri; }
        }
    }
    if (!inside) s = best;
    return inside;
}

VELOX_HD inline bool originInsideTetrahedron(const SupportVert& a,
                                             const SupportVert& b,
                                             const SupportVert& c,
                                             const SupportVert& d,
                                             float scale) {
    Vec3 v[4] = {a.p, b.p, c.p, d.p};
    float volume = dot(v[1] - v[0], cross(v[2] - v[0], v[3] - v[0]));
    float volumeFloor = 1e-10f * scale * scale * scale;
    if (fabsf(volume) < volumeFloor) return false;
    const int faces[4][3] = {{0, 1, 2}, {0, 1, 3}, {0, 2, 3}, {1, 2, 3}};
    for (int f = 0; f < 4; ++f) {
        int i0 = faces[f][0], i1 = faces[f][1], i2 = faces[f][2];
        int opposite = 6 - i0 - i1 - i2;
        Vec3 n = cross(v[i1] - v[i0], v[i2] - v[i0]);
        float originSide = dot(n, -v[i0]);
        float oppositeSide = dot(n, v[opposite] - v[i0]);
        // EPA needs the origin strictly inside its seed polytope. A tetrahedron
        // with the origin on a face can make a zero-distance face look
        // converged and return an arbitrary penetration axis.
        if (originSide * oppositeSide <= volumeFloor * volumeFloor) return false;
    }
    return true;
}

// Lower-dimensional GJK simplexes are common when the origin lies exactly on
// a search segment or triangle. Sample support directions and find a
// tetrahedron containing the origin so EPA can start from a closed polytope.
VELOX_HD inline bool expandOverlapSimplex(const Convex& A, const Convex& B,
                                          Simplex& s) {
    SupportVert candidates[18];
    int count = 0;
    for (int i = 0; i < s.count; ++i) candidates[count++] = s.v[i];
    const Vec3 dirs[14] = {
        {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1},
        {1, 1, 1}, {-1, 1, 1}, {1, -1, 1}, {1, 1, -1},
        {-1, -1, 1}, {-1, 1, -1}, {1, -1, -1}, {-1, -1, -1}};
    for (int d = 0; d < 14 && count < 18; ++d) {
        SupportVert w = supportVertex(A, B, dirs[d]);
        bool duplicate = false;
        for (int i = 0; i < count; ++i)
            if (lengthSq(w.p - candidates[i].p) < duplicateDistanceSq(A, B)) duplicate = true;
        if (!duplicate) candidates[count++] = w;
    }
    for (int i = 0; i < count - 3; ++i)
        for (int j = i + 1; j < count - 2; ++j)
            for (int k = j + 1; k < count - 1; ++k)
                for (int l = k + 1; l < count; ++l)
                    if (originInsideTetrahedron(candidates[i], candidates[j],
                                                candidates[k], candidates[l], pairScale(A, B))) {
                        s.count = 4;
                        s.v[0] = candidates[i]; s.v[1] = candidates[j];
                        s.v[2] = candidates[k]; s.v[3] = candidates[l];
                        return true;
                    }
    return false;
}

struct EpaFace {
    int a, b, c;
    Vec3 normal;
    float distance;
    bool valid;
};

VELOX_HD inline bool makeEpaFace(const SupportVert* vertices, int ia, int ib, int ic,
                                 float minAreaSq, EpaFace& face) {
    Vec3 a = vertices[ia].p, b = vertices[ib].p, c = vertices[ic].p;
    Vec3 n = cross(b - a, c - a);
    float n2 = lengthSq(n);
    if (n2 < minAreaSq) return false;
    n *= 1.0f / sqrtf(n2);
    float distance = dot(n, a);
    if (distance < 0.0f) {
        int tmp = ib; ib = ic; ic = tmp;
        n = -n;
        distance = -distance;
    }
    face = {ia, ib, ic, n, distance, true};
    return true;
}

VELOX_HD inline void epaWitness(const SupportVert* vertices, const EpaFace& face,
                                Vec3& pointA, Vec3& pointB) {
    Simplex tri{};
    tri.count = 3;
    tri.v[0] = vertices[face.a];
    tri.v[1] = vertices[face.b];
    tri.v[2] = vertices[face.c];
    solveTriangle(tri);
    pointA = {};
    pointB = {};
    for (int i = 0; i < tri.count; ++i) {
        pointA += tri.v[i].a * tri.bary[i];
        pointB += tri.v[i].b * tri.bary[i];
    }
}

// Expanding Polytope Algorithm. Fixed-size storage keeps this callable from
// CUDA kernels and makes failure explicit instead of allocating unpredictably.
VELOX_HD inline bool epaPenetration(const Convex& A, const Convex& B,
                                    const Simplex& simplex, GjkResult& result) {
    constexpr int kMaxVertices = 64;
    constexpr int kMaxFaces = 128;
    constexpr int kMaxEdges = 192;
    SupportVert vertices[kMaxVertices];
    EpaFace faces[kMaxFaces];
    int vertexCount = 4, faceCount = 0;
    for (int i = 0; i < 4; ++i) vertices[i] = simplex.v[i];

    const int initialFaces[4][3] = {{0, 1, 2}, {0, 3, 1}, {0, 2, 3}, {1, 3, 2}};
    float maxInitialAreaSq = 0.0f;
    for (int i = 0; i < 4; ++i) {
        Vec3 a = vertices[initialFaces[i][0]].p;
        Vec3 b = vertices[initialFaces[i][1]].p;
        Vec3 c = vertices[initialFaces[i][2]].p;
        maxInitialAreaSq = vmax(maxInitialAreaSq, lengthSq(cross(b - a, c - a)));
    }
    float scale = pairScale(A, B);
    GeometryQuality quality = pairQuality(A, B);
    float areaFloorSq = quality == GeometryQuality::Normal ? 1e-16f :
        vmax(maxInitialAreaSq * 1e-16f, 1e-20f * scale * scale * scale * scale);
    for (int i = 0; i < 4; ++i) {
        EpaFace face;
        if (makeEpaFace(vertices, initialFaces[i][0], initialFaces[i][1],
                        initialFaces[i][2],
                        quality == GeometryQuality::Normal ? 1e-16f : 0.0f, face))
            faces[faceCount++] = face;
    }
    if (faceCount < 4) return false;

    int bestFace = 0;
    int maxIterations = quality == GeometryQuality::Normal ? 48 : 64;
    for (int iteration = 0; iteration < maxIterations; ++iteration) {
        bestFace = -1;
        float bestDistance = 1e30f;
        for (int i = 0; i < faceCount; ++i)
            if (faces[i].valid && faces[i].distance < bestDistance) {
                bestDistance = faces[i].distance;
                bestFace = i;
            }
        if (bestFace < 0) return false;

        EpaFace closest = faces[bestFace];
        SupportVert w = supportVertex(A, B, closest.normal);
        float supportDistance = dot(closest.normal, w.p);
        float tolerance = quality == GeometryQuality::Normal
            ? 1e-4f * vmax(1.0f, supportDistance)
            : 1e-4f * vmax(scale, fabsf(supportDistance));
        bool duplicate = false;
        for (int i = 0; i < vertexCount; ++i)
            if (lengthSq(w.p - vertices[i].p) < duplicateDistanceSq(A, B)) duplicate = true;
        if (supportDistance - closest.distance <= tolerance || duplicate) {
            Vec3 coreA, coreB;
            epaWitness(vertices, closest, coreA, coreB);
            result.normal = -closest.normal;
            result.distance = -(closest.distance + A.radius + B.radius);
            result.pointA = coreA - result.normal * A.radius;
            result.pointB = coreB + result.normal * B.radius;
            result.overlapping = true;
            return true;
        }
        if (vertexCount >= kMaxVertices) break;
        int newVertex = vertexCount++;
        vertices[newVertex] = w;

        struct Edge { int a, b; } edges[kMaxEdges];
        int edgeCount = 0;
        for (int i = 0; i < faceCount; ++i) {
            if (!faces[i].valid) continue;
            const EpaFace& face = faces[i];
            if (dot(face.normal, w.p - vertices[face.a].p) <= tolerance) continue;
            faces[i].valid = false;
            int faceEdges[3][2] = {{face.a, face.b}, {face.b, face.c}, {face.c, face.a}};
            for (int e = 0; e < 3; ++e) {
                int reverse = -1;
                for (int k = 0; k < edgeCount; ++k)
                    if (edges[k].a == faceEdges[e][1] && edges[k].b == faceEdges[e][0]) {
                        reverse = k;
                        break;
                    }
                if (reverse >= 0) edges[reverse] = edges[--edgeCount];
                else if (edgeCount < kMaxEdges)
                    edges[edgeCount++] = {faceEdges[e][0], faceEdges[e][1]};
            }
        }

        for (int e = 0; e < edgeCount; ++e) {
            EpaFace face;
            if (!makeEpaFace(vertices, edges[e].a, edges[e].b, newVertex,
                             areaFloorSq, face)) continue;
            int slot = -1;
            for (int i = 0; i < faceCount; ++i)
                if (!faces[i].valid) { slot = i; break; }
            if (slot >= 0) faces[slot] = face;
            else if (faceCount < kMaxFaces) faces[faceCount++] = face;
            else return false;
        }
    }

    if (bestFace >= 0 && faces[bestFace].valid) {
        Vec3 coreA, coreB;
        epaWitness(vertices, faces[bestFace], coreA, coreB);
        result.normal = -faces[bestFace].normal;
        result.distance = -(faces[bestFace].distance + A.radius + B.radius);
        result.pointA = coreA - result.normal * A.radius;
        result.pointB = coreB + result.normal * B.radius;
        result.overlapping = true;
        return true;
    }
    return false;
}

} // namespace gjk_detail

VELOX_HD inline GjkResult gjkDistanceSeeded(const Convex& A, const Convex& B,
                                            Vec3 initialDirection) {
    using namespace gjk_detail;

    Simplex s{};
    float scale = pairScale(A, B);
    float scaleSq = scale * scale;
    GeometryQuality quality = pairQuality(A, B);
    Vec3 dir = initialDirection;
    bool robust = pairQuality(A, B) != GeometryQuality::Normal;
    if (lengthSq(dir) < (robust ? scaleSq * 1e-14f : 1e-12f)) dir = A.position - B.position;
    if (lengthSq(dir) < (robust ? scaleSq * 1e-14f : 1e-12f)) dir = {1, 0, 0};
    s.v[0] = supportVertex(A, B, dir);
    s.count = 1;
    s.bary[0] = 1.0f;

    GjkResult r{};
    int maxIterations = quality == GeometryQuality::Normal ? 32 :
                        quality == GeometryQuality::Robust ? 64 : 96;
    for (int iter = 0; iter < maxIterations; ++iter) {
        Vec3 closest{};
        for (int i = 0; i < s.count; ++i) closest += s.v[i].p * s.bary[i];
        float distSq = lengthSq(closest);
        if (distSq < (robust ? scaleSq * 1e-10f : 1e-10f)) {
            if (s.count < 4) expandOverlapSimplex(A, B, s);
            r.overlapping = true;
            break;
        }

        Vec3 d = -closest;
        SupportVert w = supportVertex(A, B, d);
        // No more progress towards the origin => converged.
        float progress = dot(w.p, d) - dot(closest, d);
        float progressFloor = robust ? relativeTolerance(A, B) * scale * sqrtf(distSq) +
            relativeTolerance(A, B) * relativeTolerance(A, B) * scaleSq
            : 1e-6f * sqrtf(distSq) + 1e-9f;
        if (progress < progressFloor) break;

        // A repeated support vertex would create a degenerate simplex (this
        // happens for flat Minkowski differences, e.g. point vs triangle).
        bool duplicate = false;
        for (int i = 0; i < s.count; ++i)
            if (lengthSq(w.p - s.v[i].p) < duplicateDistanceSq(A, B)) {
                duplicate = true;
                break;
            }
        if (duplicate) break;

        s.v[s.count++] = w;
        if (s.count == 2) solveSegment(s);
        else if (s.count == 3) solveTriangle(s);
        else if (solveTetrahedron(s, scale,
                                  pairQuality(A, B) != GeometryQuality::Normal)) {
            r.overlapping = true;
            break;
        }
    }

    Vec3 pa{}, pb{};
    for (int i = 0; i < s.count; ++i) {
        pa += s.v[i].a * s.bary[i];
        pb += s.v[i].b * s.bary[i];
    }

    if (r.overlapping) {
        if (s.count == 4 && epaPenetration(A, B, s, r)) return r;
        // Core overlap (rare: cores are points/segments/boxes kept apart by
        // the speculative solver). Lower-dimensional core intersections (for
        // example coincident capsule segments) cannot seed a 3D EPA polytope,
        // so retain a conservative center-based fallback for those cases.
        Vec3 n = A.position - B.position;
        n = lengthSq(n) > (robust ? scaleSq * 1e-14f : 1e-12f) ? normalize(n) : Vec3{0, 1, 0};
        float depth = dot(n, B.support(n) - A.support(-n));
        r.normal = n;
        r.distance = -depth - A.radius - B.radius;
        r.pointA = A.support(-n);
        r.pointB = B.support(n);
        return r;
    }

    Vec3 delta = pa - pb;
    float coreDist = length(delta);
    r.normal = coreDist > (robust ? scale * 1e-7f : 1e-8f)
        ? delta * (1.0f / coreDist) : Vec3{0, 1, 0};
    r.distance = coreDist - A.radius - B.radius;
    r.pointA = pa - r.normal * A.radius;
    r.pointB = pb + r.normal * B.radius;
    return r;
}

VELOX_HD inline GjkResult gjkDistance(const Convex& A, const Convex& B) {
    GjkResult best = gjkDistanceSeeded(A, B, A.position - B.position);
    GeometryQuality quality = gjk_detail::pairQuality(A, B);
    if (quality == GeometryQuality::Normal || best.overlapping) return best;

    const Vec3 robustDirections[6] = {
        {1, 0, 0}, {-1, 0, 0}, {0, 1, 0},
        {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
    int directionCount = quality == GeometryQuality::Robust ? 6 : 14;
    const Vec3 paranoidDirections[8] = {
        {1, 1, 1}, {-1, 1, 1}, {1, -1, 1}, {1, 1, -1},
        {-1, -1, 1}, {-1, 1, -1}, {1, -1, -1}, {-1, -1, -1}};
    float bestResidual = gjk_detail::witnessResidual(A, B, best);
    for (int i = 0; i < directionCount; ++i) {
        Vec3 direction = i < 6 ? robustDirections[i] : paranoidDirections[i - 6];
        GjkResult candidate = gjkDistanceSeeded(A, B, direction);
        if (candidate.overlapping) return candidate;
        float candidateResidual = gjk_detail::witnessResidual(A, B, candidate);
        if (candidateResidual + gjk_detail::pairScale(A, B) * 1e-6f < bestResidual ||
            (candidateResidual <= bestResidual + gjk_detail::pairScale(A, B) * 1e-6f &&
             candidate.distance < best.distance)) {
            best = candidate;
            bestResidual = candidateResidual;
        }
    }
    return best;
}

} // namespace velox
