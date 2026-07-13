#pragma once
#include "velox/backend.h"
#include "gjk.h"

// Shared narrow phase, header-only and VELOX_HD: the CPU backend and the CUDA
// kernels execute this exact code. Contacts are written into a caller-provided
// fixed-size buffer (no allocation, device-friendly).

namespace velox {

constexpr int kMaxContactsPerPair = 12;

VELOX_HD inline Vec3 npPointVelocity(const Body& b, const Vec3& p) {
    return b.velocity + cross(b.angularVelocity, p - b.position);
}

// World-space AABB of a body inflated by its speculative reach over dt.
VELOX_HD inline void bodyAabb(const Body& b, float dt, Vec3& lo, Vec3& hi) {
    float ext;
    switch (b.shape) {
    case ShapeType::Box:     ext = length(b.halfExtents); break;
    case ShapeType::Capsule: ext = b.capsuleHalfHeight + b.radius; break;
    case ShapeType::Sphere:  ext = b.radius; break;
    default:                 ext = 0.0f; break; // plane/mesh handled separately
    }
    float reach = ext + b.maxPointSpeed() * dt + 1e-2f;
    lo = b.position - Vec3{reach, reach, reach};
    hi = b.position + Vec3{reach, reach, reach};
}

VELOX_HD inline bool aabbOverlap(const Vec3& lo1, const Vec3& hi1,
                                 const Vec3& lo2, const Vec3& hi2) {
    return hi1.x >= lo2.x && lo1.x <= hi2.x &&
           hi1.y >= lo2.y && lo1.y <= hi2.y &&
           hi1.z >= lo2.z && lo1.z <= hi2.z;
}

namespace np_detail {

// Appends a speculative contact if the pair could touch within the step.
VELOX_HD inline void emit(const Body& a, const Body& b, BodyId ia, BodyId ib,
                          const Vec3& normal, const Vec3& point, float gap,
                          float dt, Contact* out, int cap, int& n) {
    if (n >= cap) return;
    float vn = dot(npPointVelocity(a, point) - npPointVelocity(b, point), normal);
    constexpr float slop = 1e-3f;
    if (gap > -vn * dt + slop && gap > slop) return; // cannot touch this step
    out[n++] = {ia, ib, normal, point, gap, vn, 0.0f, 0.0f};
}

VELOX_HD inline void planeConvex(const Body& conv, const Body& plane,
                                 BodyId ic, BodyId ip, float dt,
                                 Contact* out, int cap, int& n) {
    const Vec3& pn = plane.planeNormal;
    switch (conv.shape) {
    case ShapeType::Sphere:
        emit(conv, plane, ic, ip, pn, conv.position - pn * conv.radius,
             dot(pn, conv.position) - plane.planeOffset - conv.radius, dt, out, cap, n);
        break;
    case ShapeType::Capsule: {
        Vec3 axis = rotate(conv.orientation, {0, conv.capsuleHalfHeight, 0});
        Vec3 ends[2] = {conv.position + axis, conv.position - axis};
        for (int e = 0; e < 2; ++e)
            emit(conv, plane, ic, ip, pn, ends[e] - pn * conv.radius,
                 dot(pn, ends[e]) - plane.planeOffset - conv.radius, dt, out, cap, n);
        break;
    }
    case ShapeType::Box: {
        // Up to 4 deepest vertices form the manifold (a resting box needs
        // multiple contact points or it wobbles on a single one).
        Vec3 pts[8];
        float gaps[8];
        for (int i = 0; i < 8; ++i) {
            Vec3 local{(i & 1) ? conv.halfExtents.x : -conv.halfExtents.x,
                       (i & 2) ? conv.halfExtents.y : -conv.halfExtents.y,
                       (i & 4) ? conv.halfExtents.z : -conv.halfExtents.z};
            pts[i] = conv.position + rotate(conv.orientation, local);
            gaps[i] = dot(pn, pts[i]) - plane.planeOffset;
        }
        for (int pick = 0; pick < 4; ++pick) { // selection sort, 4 deepest
            int best = pick;
            for (int i = pick + 1; i < 8; ++i)
                if (gaps[i] < gaps[best]) best = i;
            { float tg = gaps[pick]; gaps[pick] = gaps[best]; gaps[best] = tg; }
            { Vec3 tp = pts[pick]; pts[pick] = pts[best]; pts[best] = tp; }
            emit(conv, plane, ic, ip, pn, pts[pick], gaps[pick], dt, out, cap, n);
        }
        break;
    }
    default: break;
    }
}

VELOX_HD inline void convexConvex(const Body& a, const Body& b, BodyId ia, BodyId ib,
                                  float dt, Contact* out, int cap, int& n) {
    GjkResult r = gjkDistance(makeConvex(a), makeConvex(b));
    Vec3 mid = (r.pointA + r.pointB) * 0.5f;
    emit(a, b, ia, ib, r.normal, mid, r.distance, dt, out, cap, n);

    // Box-box face contact needs more than one point to rest stably: add the
    // box vertices that are nearly as close to the other body's surface.
    if (a.shape == ShapeType::Box && b.shape == ShapeType::Box && r.distance < 0.05f) {
        for (int i = 0; i < 8; ++i) {
            Vec3 local{(i & 1) ? a.halfExtents.x : -a.halfExtents.x,
                       (i & 2) ? a.halfExtents.y : -a.halfExtents.y,
                       (i & 4) ? a.halfExtents.z : -a.halfExtents.z};
            Vec3 p = a.position + rotate(a.orientation, local);
            float d = dot(r.normal, p - r.pointB);
            if (d < r.distance + 0.02f && lengthSq(p - mid) > 1e-6f)
                emit(a, b, ia, ib, r.normal, p, d, dt, out, cap, n);
        }
    }
}

VELOX_HD inline void meshConvex(const Body& conv, const Body& meshBody,
                                BodyId ic, BodyId im, const MeshSoupView& soup,
                                float dt, Contact* out, int cap, int& n) {
    const Mesh& m = soup.meshes[meshBody.meshIndex];
    Vec3 lo, hi;
    bodyAabb(conv, dt, lo, hi);
    Convex cc = makeConvex(conv);

    // Iterative BVH traversal with an explicit stack (device-friendly).
    uint32_t stack[48];
    int sp = 0;
    stack[sp++] = m.firstNode;
    while (sp > 0) {
        const BvhNode& node = soup.bvhNodes[stack[--sp]];
        if (!aabbOverlap(lo, hi, node.aabbMin, node.aabbMax)) continue;
        if (node.triCount == 0) {
            if (sp + 2 <= 48) {
                stack[sp++] = node.leftFirst;
                stack[sp++] = node.leftFirst + 1;
            }
            continue;
        }
        for (uint32_t k = 0; k < node.triCount; ++k) {
            uint32_t tri = soup.bvhTriRefs[node.leftFirst + k];
            uint32_t base = m.firstIndex + tri * 3;
            GjkResult r = gjkDistance(cc, makeTriangle(
                soup.vertices[m.firstVertex + soup.indices[base]],
                soup.vertices[m.firstVertex + soup.indices[base + 1]],
                soup.vertices[m.firstVertex + soup.indices[base + 2]]));
            emit(conv, meshBody, ic, im, r.normal, (r.pointA + r.pointB) * 0.5f,
                 r.distance, dt, out, cap, n);
        }
    }
}

} // namespace np_detail

VELOX_HD inline bool isConvexVolume(ShapeType t) {
    return t == ShapeType::Sphere || t == ShapeType::Box || t == ShapeType::Capsule;
}

// Narrow phase for one pair; returns the number of contacts written to out
// (at most kMaxContactsPerPair).
VELOX_HD inline int collidePair(const Body& a, const Body& b, BodyId ia, BodyId ib,
                                const MeshSoupView& soup, float dt,
                                Contact* out, int cap) {
    using namespace np_detail;
    int n = 0;
    if (isConvexVolume(a.shape) && isConvexVolume(b.shape))
        convexConvex(a, b, ia, ib, dt, out, cap, n);
    else if (isConvexVolume(a.shape) && b.shape == ShapeType::Plane)
        planeConvex(a, b, ia, ib, dt, out, cap, n);
    else if (a.shape == ShapeType::Plane && isConvexVolume(b.shape))
        planeConvex(b, a, ib, ia, dt, out, cap, n);
    else if (isConvexVolume(a.shape) && b.shape == ShapeType::Mesh)
        meshConvex(a, b, ia, ib, soup, dt, out, cap, n);
    else if (a.shape == ShapeType::Mesh && isConvexVolume(b.shape))
        meshConvex(b, a, ib, ia, soup, dt, out, cap, n);
    return n;
}

// Minimum gap + contact normal between a convex body and a mesh, BVH-pruned
// but exact within an expanded search box — the distance oracle used by the
// conservative-advancement safety net.
VELOX_HD inline void meshGapProbe(const Body& conv, const Body& meshBody,
                                  const MeshSoupView& soup, float searchRadius,
                                  float& bestGap, Vec3& bestNormal) {
    const Mesh& m = soup.meshes[meshBody.meshIndex];
    Vec3 r{searchRadius, searchRadius, searchRadius};
    Vec3 lo = conv.position - r, hi = conv.position + r;
    Convex cc = makeConvex(conv);

    uint32_t stack[48];
    int sp = 0;
    stack[sp++] = m.firstNode;
    while (sp > 0) {
        const BvhNode& node = soup.bvhNodes[stack[--sp]];
        if (!aabbOverlap(lo, hi, node.aabbMin, node.aabbMax)) continue;
        if (node.triCount == 0) {
            if (sp + 2 <= 48) {
                stack[sp++] = node.leftFirst;
                stack[sp++] = node.leftFirst + 1;
            }
            continue;
        }
        for (uint32_t k = 0; k < node.triCount; ++k) {
            uint32_t tri = soup.bvhTriRefs[node.leftFirst + k];
            uint32_t base = m.firstIndex + tri * 3;
            GjkResult g = gjkDistance(cc, makeTriangle(
                soup.vertices[m.firstVertex + soup.indices[base]],
                soup.vertices[m.firstVertex + soup.indices[base + 1]],
                soup.vertices[m.firstVertex + soup.indices[base + 2]]));
            if (g.distance < bestGap) { bestGap = g.distance; bestNormal = g.normal; }
        }
    }
}

} // namespace velox
