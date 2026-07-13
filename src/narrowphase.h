#pragma once
#include "velox/backend.h"
#include "gjk.h"

// Shared narrow phase, header-only and VELOX_HD: the CPU backend and the CUDA
// kernels execute this exact code. Contacts are written into a caller-provided
// fixed-size buffer (no allocation, device-friendly).

namespace velox {

constexpr int kMaxContactsPerPair = 12;
constexpr int kVelocityIterations = 8; // per substep

VELOX_HD inline Vec3 npPointVelocity(const Body& b, const Vec3& p) {
    return b.velocity + cross(b.angularVelocity, p - b.position);
}

VELOX_HD inline Vec3 contactAnchorWorld(const Body& b, const Vec3& localAnchor) {
    return b.position + rotate(b.orientation, localAnchor);
}

VELOX_HD inline float contactLiveGap(const Body& a, const Body& b, const Contact& c) {
    Vec3 pa = contactAnchorWorld(a, c.localAnchorA);
    Vec3 pb = contactAnchorWorld(b, c.localAnchorB);
    return c.bias0 + dot(c.normal, pa - pb);
}

VELOX_HD inline void contactTangents(const Vec3& normal, Vec3& tangent1, Vec3& tangent2) {
    Vec3 an{fabsf(normal.x), fabsf(normal.y), fabsf(normal.z)};
    Vec3 reference = an.x < an.y
        ? (an.x < an.z ? Vec3{1, 0, 0} : Vec3{0, 0, 1})
        : (an.y < an.z ? Vec3{0, 1, 0} : Vec3{0, 0, 1});
    tangent1 = normalize(cross(normal, reference));
    tangent2 = cross(normal, tangent1);
}

// Re-applies the accumulated normal impulse carried over from the previous
// frame (warm starting). Persistent contacts then converge in far fewer
// iterations, which is what keeps tall stacks solid.
VELOX_HD inline void warmStartContact(Body& a, Body& b, Contact& c) {
    if (c.normalImpulse <= 0.0f && c.tangentImpulse1 == 0.0f &&
        c.tangentImpulse2 == 0.0f) return;
    Vec3 pa = contactAnchorWorld(a, c.localAnchorA);
    Vec3 pb = contactAnchorWorld(b, c.localAnchorB);
    c.point = (pa + pb) * 0.5f;
    Vec3 tangent1, tangent2;
    contactTangents(c.normal, tangent1, tangent2);
    Vec3 impulse = c.normal * c.normalImpulse +
                   tangent1 * c.tangentImpulse1 +
                   tangent2 * c.tangentImpulse2;
    if (a.isDynamic()) {
        a.velocity += impulse * a.solverInvMass();
        a.angularVelocity += a.invInertiaMul(cross(c.point - a.position, impulse));
    }
    if (b.isDynamic()) {
        b.velocity -= impulse * b.solverInvMass();
        b.angularVelocity -= b.invInertiaMul(cross(c.point - b.position, impulse));
    }
}

// One sequential-impulse pass over one contact (normal + friction with
// accumulated clamping). Shared by the CPU solver (sequential order) and the
// CUDA solver (graph-colored parallel order). Static bodies are never
// written, so contacts sharing only a static body may run concurrently.
VELOX_HD inline void solveContact(Body& a, Body& b, Contact& c, float dt) {
    Vec3 pa = contactAnchorWorld(a, c.localAnchorA);
    Vec3 pb = contactAnchorWorld(b, c.localAnchorB);
    c.point = (pa + pb) * 0.5f;
    Vec3 ra = c.point - a.position;
    Vec3 rb = c.point - b.position;

    // Effective mass along the normal, including rotation.
    Vec3 raxn = cross(ra, c.normal);
    Vec3 rbxn = cross(rb, c.normal);
    float kNormal = a.solverInvMass() + b.solverInvMass() +
                    dot(raxn, a.invInertiaMul(raxn)) +
                    dot(rbxn, b.invInertiaMul(rbxn));
    if (kNormal <= 0.0f) return;

    float vn = dot(npPointVelocity(a, c.point) - npPointVelocity(b, c.point), c.normal);

    // Target normal velocity: may still close the remaining gap, and must
    // bounce with restitution if the approach was fast.
    constexpr float kRestitutionThreshold = 1.0f; // m/s; below this, no bounce
    // May close the remaining LIVE gap (re-evaluated from current positions —
    // this is what lets several substeps reuse one detection pass), no
    // further. Penetration is fixed by the positional correction pass, never
    // by velocity bias (no energy gain).
    float liveGap = c.bias0 + dot(c.normal, pa - pb);
    float target = liveGap > 0.0f ? -liveGap / dt : 0.0f;
    if (c.vn0 < -kRestitutionThreshold)
        target = vmax(target, -vmin(a.restitution, b.restitution) * c.vn0);

    float jn = (target - vn) / kNormal;
    float newImpulse = vmax(c.normalImpulse + jn, 0.0f);
    jn = newImpulse - c.normalImpulse;
    c.normalImpulse = newImpulse;

    Vec3 impulse = c.normal * jn;
    if (a.isDynamic()) {
        a.velocity += impulse * a.solverInvMass();
        a.angularVelocity += a.invInertiaMul(cross(ra, impulse));
    }
    if (b.isDynamic()) {
        b.velocity -= impulse * b.solverInvMass();
        b.angularVelocity -= b.invInertiaMul(cross(rb, impulse));
    }

    // Two-axis Coulomb friction. A deterministic tangent basis makes the two
    // accumulated rows stable across frames, and circular clamping avoids the
    // directional bias of independent box constraints.
    Vec3 rv = npPointVelocity(a, c.point) - npPointVelocity(b, c.point);
    Vec3 tangent1, tangent2;
    contactTangents(c.normal, tangent1, tangent2);
    Vec3 raxt1 = cross(ra, tangent1), rbxt1 = cross(rb, tangent1);
    Vec3 raxt2 = cross(ra, tangent2), rbxt2 = cross(rb, tangent2);
    float k1 = a.solverInvMass() + b.solverInvMass() + dot(raxt1, a.invInertiaMul(raxt1)) +
               dot(rbxt1, b.invInertiaMul(rbxt1));
    float k2 = a.solverInvMass() + b.solverInvMass() + dot(raxt2, a.invInertiaMul(raxt2)) +
               dot(rbxt2, b.invInertiaMul(rbxt2));
    if (k1 > 0.0f && k2 > 0.0f) {
        float old1 = c.tangentImpulse1, old2 = c.tangentImpulse2;
        float next1 = old1 - dot(rv, tangent1) / k1;
        float next2 = old2 - dot(rv, tangent2) / k2;
        float maxFriction = sqrtf(a.friction * b.friction) * c.normalImpulse;
        float magnitude = sqrtf(next1 * next1 + next2 * next2);
        if (magnitude > maxFriction && magnitude > 1e-9f) {
            float scale = maxFriction / magnitude;
            next1 *= scale;
            next2 *= scale;
        }
        c.tangentImpulse1 = next1;
        c.tangentImpulse2 = next2;
        Vec3 fImpulse = tangent1 * (next1 - old1) + tangent2 * (next2 - old2);
        if (a.isDynamic()) {
            a.velocity += fImpulse * a.solverInvMass();
            a.angularVelocity += a.invInertiaMul(cross(ra, fImpulse));
        }
        if (b.isDynamic()) {
            b.velocity -= fImpulse * b.solverInvMass();
            b.angularVelocity -= b.invInertiaMul(cross(rb, fImpulse));
        }
    }
}

// World-space AABB of a body inflated by its speculative reach over dt.
VELOX_HD inline void bodyAabb(const Body& b, float dt, Vec3& lo, Vec3& hi) {
    float ext;
    switch (b.shape) {
    case ShapeType::Box:     ext = length(b.halfExtents); break;
    case ShapeType::Capsule: ext = b.capsuleHalfHeight + b.radius; break;
    case ShapeType::Sphere:  ext = b.radius; break;
    case ShapeType::Hull:    ext = b.radius; break;
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

constexpr uint32_t kNoFeature = 0xffffffffu;
constexpr uint32_t kFaceFeature = 0x40000000u;
constexpr uint32_t kImplicitFeature = 0x80000000u;
constexpr uint32_t kTriangleFeature = 0xc0000000u;

VELOX_HD inline uint64_t contactFeatureKey(uint32_t featureA, uint32_t featureB) {
    uint64_t a = featureA == kNoFeature ? 0 : (uint64_t)featureA + 1;
    uint64_t b = featureB == kNoFeature ? 0 : (uint64_t)featureB + 1;
    return (a << 32) | b;
}

// Appends a speculative contact if the pair could touch within the step.
// The reach uses the sum of ABSOLUTE surface speeds, not the relative
// velocity: the velocity solve runs after detection and can stop either body
// dead (e.g. the box below lands), turning a zero-relative-velocity pair into
// a colliding one within the same step.
VELOX_HD inline void emit(const Body& a, const Body& b, BodyIndex ia, BodyIndex ib,
                          const Vec3& normal, const Vec3& point, float gap,
                          float dt, Contact* out, int cap, int& n,
                          uint64_t featureKey = 0) {
    if (n >= cap) return;
    constexpr float slop = 1e-3f;
    float reach = (a.maxPointSpeed() + b.maxPointSpeed()) * dt + slop;
    if (gap > reach) return; // cannot touch this step
    float vn = dot(npPointVelocity(a, point) - npPointVelocity(b, point), normal);
    Vec3 localA = rotateInv(a.orientation, point - a.position);
    Vec3 localB = rotateInv(b.orientation, point - b.position);
    float bias0 = gap;
    out[n++] = {ia, ib, featureKey, normal, point, localA, localB, gap, bias0,
                vn, 0.0f, 0.0f, 0.0f};
}

VELOX_HD inline void planeConvex(const Body& conv, const Body& plane,
                                 BodyIndex ic, BodyIndex ip, const MeshSoupView& soup,
                                 float dt, Contact* out, int cap, int& n) {
    const Vec3& pn = plane.planeNormal;
    switch (conv.shape) {
    case ShapeType::Hull: {
        // Up to 4 deepest hull vertices along the plane normal.
        const Vec3* pts = soup.hullPoints + conv.hullFirst;
        uint32_t selected[4]{};
        int selectedCount = 0;
        for (int pick = 0; pick < 4 && pick < (int)conv.hullCount; ++pick) {
            float bestGap = 1e30f;
            Vec3 bestP{};
            uint32_t bestIndex = 0;
            for (uint32_t i = 0; i < conv.hullCount; ++i) {
                Vec3 p = conv.position + rotate(conv.orientation, pts[i]);
                float g = dot(pn, p) - plane.planeOffset;
                bool taken = false;
                for (int k = 0; k < selectedCount; ++k)
                    if (selected[k] == i) taken = true;
                if (!taken && g < bestGap) {
                    bestGap = g; bestP = p; bestIndex = i;
                }
            }
            if (bestGap > 0.1f) break; // rest are far from the plane
            selected[selectedCount++] = bestIndex;
            emit(conv, plane, ic, ip, pn, bestP, bestGap, dt, out, cap, n,
                 contactFeatureKey(bestIndex, kFaceFeature));
        }
        break;
    }
    case ShapeType::Sphere:
        emit(conv, plane, ic, ip, pn, conv.position - pn * conv.radius,
             dot(pn, conv.position) - plane.planeOffset - conv.radius, dt, out, cap, n,
             contactFeatureKey(kImplicitFeature, kFaceFeature));
        break;
    case ShapeType::Capsule: {
        Vec3 axis = rotate(conv.orientation, {0, conv.capsuleHalfHeight, 0});
        Vec3 ends[2] = {conv.position + axis, conv.position - axis};
        for (int e = 0; e < 2; ++e)
            emit(conv, plane, ic, ip, pn, ends[e] - pn * conv.radius,
                 dot(pn, ends[e]) - plane.planeOffset - conv.radius, dt, out, cap, n,
                 contactFeatureKey(kImplicitFeature | (uint32_t)e, kFaceFeature));
        break;
    }
    case ShapeType::Box: {
        // Up to 4 deepest vertices form the manifold (a resting box needs
        // multiple contact points or it wobbles on a single one).
        Vec3 pts[8];
        float gaps[8];
        uint32_t features[8];
        for (int i = 0; i < 8; ++i) {
            Vec3 local{(i & 1) ? conv.halfExtents.x : -conv.halfExtents.x,
                       (i & 2) ? conv.halfExtents.y : -conv.halfExtents.y,
                       (i & 4) ? conv.halfExtents.z : -conv.halfExtents.z};
            pts[i] = conv.position + rotate(conv.orientation, local);
            gaps[i] = dot(pn, pts[i]) - plane.planeOffset;
            features[i] = (uint32_t)i;
        }
        for (int pick = 0; pick < 4; ++pick) { // selection sort, 4 deepest
            int best = pick;
            for (int i = pick + 1; i < 8; ++i)
                if (gaps[i] < gaps[best]) best = i;
            { float tg = gaps[pick]; gaps[pick] = gaps[best]; gaps[best] = tg; }
            { Vec3 tp = pts[pick]; pts[pick] = pts[best]; pts[best] = tp; }
            { uint32_t tf = features[pick]; features[pick] = features[best]; features[best] = tf; }
            emit(conv, plane, ic, ip, pn, pts[pick], gaps[pick], dt, out, cap, n,
                 contactFeatureKey(features[pick], kFaceFeature));
        }
        break;
    }
    default: break;
    }
}

VELOX_HD inline void boxVertex(const Body& box, int i, Vec3& out) {
    Vec3 local{(i & 1) ? box.halfExtents.x : -box.halfExtents.x,
               (i & 2) ? box.halfExtents.y : -box.halfExtents.y,
               (i & 4) ? box.halfExtents.z : -box.halfExtents.z};
    out = box.position + rotate(box.orientation, local);
}

VELOX_HD inline void convexConvex(const Body& a, const Body& b, BodyIndex ia, BodyIndex ib,
                                  const MeshSoupView& soup,
                                  float dt, Contact* out, int cap, int& n) {
    Convex ca = makeConvex(a, soup), cb = makeConvex(b, soup);
    GjkResult r = gjkDistance(ca, cb);

    // A single GJK witness is not a stable resting manifold for a hull face.
    // Emit up to four hull vertices on the near-support plane. This keeps the
    // response data-oriented (no face topology required) while giving both
    // the sequential CPU solver and graph-colored GPU solver enough torque
    // constraints to settle a face instead of repeatedly hitting one point.
    if ((a.shape == ShapeType::Hull || b.shape == ShapeType::Hull) &&
        r.distance < 0.05f) {
        const bool hullIsA = a.shape == ShapeType::Hull;
        const Body& hull = hullIsA ? a : b;
        const Vec3* pts = soup.hullPoints + hull.hullFirst;
        uint32_t selected[4]{};
        int selectedCount = 0;
        float reference = hullIsA ? dot(r.normal, cb.support(r.normal))
                                  : dot(r.normal, ca.support(-r.normal));
        for (int pick = 0; pick < 4 && pick < (int)hull.hullCount; ++pick) {
            uint32_t best = 0;
            float bestGap = 1e30f;
            Vec3 bestPoint{};
            bool found = false;
            for (uint32_t i = 0; i < hull.hullCount; ++i) {
                bool used = false;
                for (int k = 0; k < selectedCount; ++k)
                    if (selected[k] == i) used = true;
                if (used) continue;
                Vec3 p = hull.position + rotate(hull.orientation, pts[i]);
                float gap = hullIsA ? dot(r.normal, p) - reference
                                    : reference - dot(r.normal, p);
                if (gap < bestGap) {
                    best = i;
                    bestGap = gap;
                    bestPoint = p;
                    found = true;
                }
            }
            if (!found || bestGap > r.distance + 0.02f) break;
            selected[selectedCount++] = best;
            uint64_t feature = hullIsA
                ? contactFeatureKey(best, kImplicitFeature)
                : contactFeatureKey(kImplicitFeature, best);
            emit(a, b, ia, ib, r.normal, bestPoint, bestGap, dt, out, cap, n, feature);
        }
        if (n > 0) return;
    }

    if (a.shape == ShapeType::Box && b.shape == ShapeType::Box && r.distance < 0.05f) {
        // Between parallel box faces the GJK witness pair is ill-defined (any
        // opposing points are equally close), so its normal can tilt with
        // floating-point noise and slowly shove stacks sideways. Test the 6
        // face normals as separating axes; if one explains the separation,
        // snap the contact normal to it and build the manifold from vertices
        // against that face plane — deterministic geometry, no witness points.
        Vec3 nrm = r.normal;
        float best = -1e30f;
        uint32_t referenceFeature = kFaceFeature;
        for (int side = 0; side < 2; ++side) {
            const Body& box = side ? b : a;
            for (int ax = 0; ax < 3; ++ax) {
                Vec3 axis = rotate(box.orientation,
                                   {ax == 0 ? 1.0f : 0.0f, ax == 1 ? 1.0f : 0.0f,
                                    ax == 2 ? 1.0f : 0.0f});
                if (dot(axis, a.position - b.position) < 0.0f) axis = -axis;
                float sep = dot(axis, ca.support(-axis) - cb.support(axis));
                if (sep > best) {
                    best = sep;
                    nrm = axis;
                    referenceFeature = kFaceFeature | (uint32_t)(side * 3 + ax);
                }
            }
        }
        if (best < 0.05f) {
            // Select one incident face. Emitting vertices from both boxes
            // creates eight nearly duplicate constraints while speculative
            // faces are still separated, biasing the sequential solver.
            float refB = dot(nrm, cb.support(nrm));   // B surface along +nrm
            float refA = dot(nrm, ca.support(-nrm));  // A surface along -nrm
            Vec3 ptsA[8], ptsB[8];
            float gapsA[8], gapsB[8];
            uint32_t idsA[8], idsB[8];
            int countA = 0, countB = 0;
            for (int i = 0; i < 8; ++i) {
                Vec3 v;
                boxVertex(a, i, v);
                float gap = dot(nrm, v) - refB;
                if (gap < best + 0.02f) {
                    ptsA[countA] = v;
                    gapsA[countA++] = gap;
                    idsA[countA - 1] = (uint32_t)i;
                }
                boxVertex(b, i, v);
                gap = refA - dot(nrm, v);
                if (gap < best + 0.02f) {
                    ptsB[countB] = v;
                    gapsB[countB++] = gap;
                    idsB[countB - 1] = (uint32_t)i;
                }
            }
            bool useA = countA >= countB;
            int count = useA ? countA : countB;
            const int balancedOrder[4] = {0, 3, 1, 2};
            for (int k = 0; k < count && k < 4; ++k) {
                int i = count >= 4 ? balancedOrder[k] : k;
                float gap = useA ? gapsA[i] : gapsB[i];
                Vec3 p = useA ? ptsA[i] - nrm * (gap * 0.5f)
                              : ptsB[i] + nrm * (gap * 0.5f);
                uint64_t feature = useA
                    ? contactFeatureKey(idsA[i], referenceFeature)
                    : contactFeatureKey(referenceFeature, idsB[i]);
                emit(a, b, ia, ib, nrm, p, gap, dt, out, cap, n, feature);
            }
            if (count > 0) return;
        }
    }

    emit(a, b, ia, ib, r.normal, (r.pointA + r.pointB) * 0.5f, r.distance, dt, out, cap, n);
}

VELOX_HD inline void meshConvex(const Body& conv, const Body& meshBody,
                                BodyIndex ic, BodyIndex im, const MeshSoupView& soup,
                                float dt, Contact* out, int cap, int& n) {
    const Mesh& m = soup.meshes[meshBody.meshIndex];
    Vec3 lo, hi;
    bodyAabb(conv, dt, lo, hi);
    Convex cc = makeConvex(conv, soup);

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
                 r.distance, dt, out, cap, n,
                 contactFeatureKey(kImplicitFeature, kTriangleFeature | tri));
        }
    }
}

} // namespace np_detail

VELOX_HD inline bool isConvexVolume(ShapeType t) {
    return t == ShapeType::Sphere || t == ShapeType::Box ||
           t == ShapeType::Capsule || t == ShapeType::Hull;
}

// Narrow phase for one pair; returns the number of contacts written to out
// (at most kMaxContactsPerPair).
VELOX_HD inline int collidePair(const Body& a, const Body& b, BodyIndex ia, BodyIndex ib,
                                const MeshSoupView& soup, float dt,
                                Contact* out, int cap) {
    using namespace np_detail;
    int n = 0;
    if (isConvexVolume(a.shape) && isConvexVolume(b.shape))
        convexConvex(a, b, ia, ib, soup, dt, out, cap, n);
    else if (isConvexVolume(a.shape) && b.shape == ShapeType::Plane)
        planeConvex(a, b, ia, ib, soup, dt, out, cap, n);
    else if (a.shape == ShapeType::Plane && isConvexVolume(b.shape))
        planeConvex(b, a, ib, ia, soup, dt, out, cap, n);
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
    Convex cc = makeConvex(conv, soup);

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
