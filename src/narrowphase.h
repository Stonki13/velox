#pragma once
#include "velox/backend.h"
#include "gjk.h"
#include "manifold.h"

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
    return b.position + (b.shape == ShapeType::Sphere
        ? localAnchor : rotate(b.orientation, localAnchor));
}

VELOX_HD inline Vec3 contactAnchorLocal(const Body& b, const Vec3& worldAnchor) {
    Vec3 offset = worldAnchor - b.position;
    return b.shape == ShapeType::Sphere ? offset : rotateInv(b.orientation, offset);
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

VELOX_HD inline float combineMaterial(float a, float b,
                                      MaterialCombineMode modeA,
                                      MaterialCombineMode modeB) {
    MaterialCombineMode mode = (uint8_t)modeA >= (uint8_t)modeB ? modeA : modeB;
    switch (mode) {
    case MaterialCombineMode::Average: return 0.5f * a + 0.5f * b;
    case MaterialCombineMode::GeometricMean: return sqrtf(a) * sqrtf(b);
    case MaterialCombineMode::Minimum: return vmin(a, b);
    case MaterialCombineMode::Multiply:
        return b > 0.0f && a > 3.402823466e+38F / b
            ? 3.402823466e+38F : a * b;
    case MaterialCombineMode::Maximum: return vmax(a, b);
    }
    return 0.0f;
}

VELOX_HD inline float directionalFrictionScale(const Body& body,
                                                const Vec3& worldDirection) {
    Vec3 local = rotateInv(body.orientation, worldDirection);
    Vec3 scaled{local.x * body.frictionScale.x,
                local.y * body.frictionScale.y,
                local.z * body.frictionScale.z};
    return length(scaled);
}

// Re-applies the accumulated normal impulse carried over from the previous
// frame (warm starting). Persistent contacts then converge in far fewer
// iterations, which is what keeps tall stacks solid.
VELOX_HD inline void warmStartContact(Body& a, Body& b, Contact& c) {
    if (c.normalImpulse <= 0.0f && c.tangentImpulse1 == 0.0f &&
        c.tangentImpulse2 == 0.0f && c.rollingImpulse1 == 0.0f &&
        c.rollingImpulse2 == 0.0f && c.spinningImpulse == 0.0f) return;
    Vec3 pa = contactAnchorWorld(a, c.localAnchorA);
    Vec3 pb = contactAnchorWorld(b, c.localAnchorB);
    c.point = (pa + pb) * 0.5f;
    Vec3 tangent1, tangent2;
    contactTangents(c.normal, tangent1, tangent2);
    Vec3 impulse = c.normal * c.normalImpulse +
                   tangent1 * c.tangentImpulse1 +
                   tangent2 * c.tangentImpulse2;
    Vec3 angularImpulse = tangent1 * c.rollingImpulse1 +
                          tangent2 * c.rollingImpulse2 +
                          c.normal * c.spinningImpulse;
    // Per-body witness arms, matching solveContact (see the comment there).
    if (a.isDynamic()) {
        a.velocity += impulse * a.solverInvMass();
        a.angularVelocity += a.invInertiaMul(cross(pa - a.position, impulse));
        a.angularVelocity += a.invInertiaMul(angularImpulse);
    }
    if (b.isDynamic()) {
        b.velocity -= impulse * b.solverInvMass();
        b.angularVelocity -= b.invInertiaMul(cross(pb - b.position, impulse));
        b.angularVelocity -= b.invInertiaMul(angularImpulse);
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
    // Each body's torque arm comes from its OWN witness anchor, not the
    // shared midpoint. A static body's anchor is frozen at detection while
    // the dynamic body advances through the substeps, so the midpoint lags
    // the true contact tangentially and the NORMAL impulse acquires a bogus
    // torque arm (differential testing vs Jolt: a frictionless sphere
    // sliding on a plane slowly gained spin, and a rolling sphere bled ~10%
    // speed against a persistent phantom slip).
    Vec3 ra = pa - a.position;
    Vec3 rb = pb - b.position;

    // Effective mass along the normal, including rotation.
    Vec3 raxn = cross(ra, c.normal);
    Vec3 rbxn = cross(rb, c.normal);
    float kNormal = a.solverInvMass() + b.solverInvMass() +
                    dot(raxn, a.invInertiaMul(raxn)) +
                    dot(rbxn, b.invInertiaMul(rbxn));
    if (kNormal <= 0.0f) return;

    float vn = dot(npPointVelocity(a, pa) - npPointVelocity(b, pb), c.normal);

    // Target normal velocity: may still close the remaining gap, and must
    // bounce with restitution if the approach was fast.
    constexpr float kRestitutionThreshold = 1.0f; // m/s; below this, no bounce
    // May close the remaining LIVE gap (re-evaluated from current positions —
    // this is what lets several substeps reuse one detection pass), no
    // further. Penetration is fixed by the positional correction pass, never
    // by velocity bias (no energy gain).
    float liveGap = c.bias0 + dot(c.normal, pa - pb);
    float target = liveGap > 0.0f ? -liveGap / dt : 0.0f;
    // Restitution only fires once the surfaces actually meet. A speculative
    // contact can be created a full step's travel ahead of the surface;
    // bouncing from that distance reflects the body early (differential
    // testing vs Jolt: a 150 m/s sphere bounced 2 m before the wall). Once
    // the body separates again (vn > 0), the restitution floor must remain:
    // the impulse accumulated while braking across the speculative gap would
    // otherwise act as glue and cancel the exit velocity on later substeps.
    if (c.vn0 < -kRestitutionThreshold && (liveGap <= 0.0f || vn > 0.0f))
        target = vmax(target, -c.restitution * c.vn0);

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
    Vec3 rv = npPointVelocity(a, pa) - npPointVelocity(b, pb);
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
        float max1 = c.friction1 * c.normalImpulse;
        float max2 = c.friction2 * c.normalImpulse;
        if (max1 <= 0.0f) next1 = 0.0f;
        if (max2 <= 0.0f) next2 = 0.0f;
        float ellipse = (max1 > 0.0f ? next1 * next1 / (max1 * max1) : 0.0f) +
                        (max2 > 0.0f ? next2 * next2 / (max2 * max2) : 0.0f);
        if (ellipse > 1.0f) {
            float scale = 1.0f / sqrtf(ellipse);
            next1 *= scale; next2 *= scale;
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

    // Rolling and spinning resistance are angular impulses bounded by the
    // normal load. Coefficients have length units, matching common rigid-body
    // rolling-friction models (torque impulse <= coefficient * normal impulse).
    Vec3 wr = a.angularVelocity - b.angularVelocity;
    float kr1 = dot(tangent1, a.invInertiaMul(tangent1)) +
                dot(tangent1, b.invInertiaMul(tangent1));
    float kr2 = dot(tangent2, a.invInertiaMul(tangent2)) +
                dot(tangent2, b.invInertiaMul(tangent2));
    if (c.rollingFriction > 0.0f && kr1 > 0.0f && kr2 > 0.0f) {
        float old1 = c.rollingImpulse1, old2 = c.rollingImpulse2;
        float next1 = old1 - dot(wr, tangent1) / kr1;
        float next2 = old2 - dot(wr, tangent2) / kr2;
        float limit = c.rollingFriction * c.normalImpulse;
        float magnitude = sqrtf(next1 * next1 + next2 * next2);
        if (magnitude > limit && magnitude > 1e-9f) {
            float scale = limit / magnitude;
            next1 *= scale; next2 *= scale;
        }
        c.rollingImpulse1 = next1;
        c.rollingImpulse2 = next2;
        Vec3 torque = tangent1 * (next1 - old1) + tangent2 * (next2 - old2);
        if (a.isDynamic()) a.angularVelocity += a.invInertiaMul(torque);
        if (b.isDynamic()) b.angularVelocity -= b.invInertiaMul(torque);
    }
    wr = a.angularVelocity - b.angularVelocity;
    float ks = dot(c.normal, a.invInertiaMul(c.normal)) +
               dot(c.normal, b.invInertiaMul(c.normal));
    if (c.spinningFriction > 0.0f && ks > 0.0f) {
        float old = c.spinningImpulse;
        float next = old - dot(wr, c.normal) / ks;
        float limit = c.spinningFriction * c.normalImpulse;
        next = vclamp(next, -limit, limit);
        c.spinningImpulse = next;
        Vec3 torque = c.normal * (next - old);
        if (a.isDynamic()) a.angularVelocity += a.invInertiaMul(torque);
        if (b.isDynamic()) b.angularVelocity -= b.invInertiaMul(torque);
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
    case ShapeType::Compound: ext = b.radius; break;
    case ShapeType::Cylinder:
        ext = sqrtf(b.radius * b.radius +
                    b.capsuleHalfHeight * b.capsuleHalfHeight);
        break;
    case ShapeType::Cone: {
        float top = 1.5f * b.capsuleHalfHeight;
        ext = vmax(top, sqrtf(b.radius * b.radius +
                              0.25f * b.capsuleHalfHeight * b.capsuleHalfHeight));
        break;
    }
    default:                 ext = 0.0f; break; // plane/mesh handled separately
    }
    const bool continuous = b.ccdTuning.enableContinuous &&
                            b.ccdTuning.quality != MotionQuality::Low &&
                            b.ccdTuning.quality != MotionQuality::Locked;
    const float sweep = continuous ? b.maxPointSpeed() * dt : 0.0f;
    float reach = ext + sweep + b.ccdTuning.collisionMargin + 1e-2f;
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
    const bool sweepA = a.ccdTuning.enableContinuous &&
                        a.ccdTuning.quality != MotionQuality::Low &&
                        a.ccdTuning.quality != MotionQuality::Locked;
    const bool sweepB = b.ccdTuning.enableContinuous &&
                        b.ccdTuning.quality != MotionQuality::Low &&
                        b.ccdTuning.quality != MotionQuality::Locked;
    float reach = (sweepA ? a.maxPointSpeed() : 0.0f) * dt +
                  (sweepB ? b.maxPointSpeed() : 0.0f) * dt +
                  a.ccdTuning.speculativeDistance +
                  b.ccdTuning.speculativeDistance + slop;
    if (gap > reach) return; // cannot touch this step
    float vn = dot(npPointVelocity(a, point) - npPointVelocity(b, point), normal);
    Vec3 localA = contactAnchorLocal(a, point);
    Vec3 localB = contactAnchorLocal(b, point);
    float bias0 = gap;
    Vec3 tangent1, tangent2;
    contactTangents(normal, tangent1, tangent2);
    float friction = combineMaterial(a.friction, b.friction,
                                     a.frictionCombine, b.frictionCombine);
    float scale1 = sqrtf(directionalFrictionScale(a, tangent1) *
                         directionalFrictionScale(b, tangent1));
    float scale2 = sqrtf(directionalFrictionScale(a, tangent2) *
                         directionalFrictionScale(b, tangent2));
    float restitution = combineMaterial(a.restitution, b.restitution,
                                        a.restitutionCombine, b.restitutionCombine);
    float rolling = combineMaterial(a.rollingFriction, b.rollingFriction,
                                    a.frictionCombine, b.frictionCombine);
    float spinning = combineMaterial(a.spinningFriction, b.spinningFriction,
                                     a.frictionCombine, b.frictionCombine);
    out[n++] = {ia, ib, featureKey, normal, point, localA, localB, gap, bias0, vn,
                restitution, friction * scale1, friction * scale2, rolling, spinning,
                0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
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
    case ShapeType::Cylinder:
    case ShapeType::Cone: {
        Convex shape = makeConvex(conv, soup);
        Vec3 deepest = shape.support(-pn);
        emit(conv, plane, ic, ip, pn, deepest,
             dot(pn, deepest) - plane.planeOffset, dt, out, cap, n,
             contactFeatureKey(kImplicitFeature, kFaceFeature));
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

    // Face clipping replaces sampled support points for planar Box/Hull
    // contacts. For boxes, snap nearly parallel faces to a geometric axis
    // before clipping: GJK witnesses on parallel faces are underdetermined.
    if (r.distance < 0.05f &&
        ((a.shape == ShapeType::Box || a.shape == ShapeType::Hull) &&
         (b.shape == ShapeType::Box || b.shape == ShapeType::Hull))) {
        Vec3 manifoldNormal = r.normal;
        if (a.shape == ShapeType::Box && b.shape == ShapeType::Box) {
            float best = -1e30f;
            for (int side = 0; side < 2; ++side) {
                const Body& box = side ? b : a;
                for (int axis = 0; axis < 3; ++axis) {
                    Vec3 candidate = rotate(box.orientation,
                        {axis == 0 ? 1.0f : 0.0f, axis == 1 ? 1.0f : 0.0f,
                         axis == 2 ? 1.0f : 0.0f});
                    if (dot(candidate, a.position - b.position) < 0.0f)
                        candidate = -candidate;
                    float separation = dot(candidate,
                        ca.support(-candidate) - cb.support(candidate));
                    if (separation > best) {
                        best = separation;
                        manifoldNormal = candidate;
                    }
                }
            }
        }
        Manifold manifold{};
        int pointCount = clipFaceManifold(ca, cb, manifoldNormal, &manifold);
        if (pointCount > 0) {
            bool emitted = false;
            for (int i = 0; i < pointCount && n < cap; ++i) {
                const ManifoldPoint& mp = manifold.points[i];
                uint64_t featureKey = computeFeatureKey(mp.featureIdA, mp.featureIdB);
                // Anchors are regenerated from this frame's witness midpoint.
                // Only the impulses are warm-started by the stable key, never
                // a previous frame's world-space anchor or penetration depth.
                emit(a, b, ia, ib, mp.normal, (mp.pointA + mp.pointB) * 0.5f,
                     mp.gap, dt, out, cap, n, featureKey);
                emitted = true;
            }
            if (emitted) return;
        }
    }

    // Fallback: single GJK witness for non-Box/Hull pairs or when manifold generation fails.
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
           t == ShapeType::Capsule || t == ShapeType::Hull ||
           t == ShapeType::Cylinder || t == ShapeType::Cone;
}

// Narrow phase for one pair; returns the number of contacts written to out
// (at most kMaxContactsPerPair).
VELOX_HD inline int collideSimplePair(const Body& a, const Body& b,
                                      BodyIndex ia, BodyIndex ib,
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

// Compound dispatch expands locally transformed children but keeps contacts
// owned by the parent dense indices. Anchors and relative velocity are
// remapped from child frames to parent frames before the solver sees them.
VELOX_HD inline int collidePair(const Body& a, const Body& b, BodyIndex ia, BodyIndex ib,
                                const MeshSoupView& soup, float dt,
                                Contact* out, int cap) {
    if (a.shape != ShapeType::Compound && b.shape != ShapeType::Compound)
        return collideSimplePair(a, b, ia, ib, soup, dt, out, cap);

    int countA = a.shape == ShapeType::Compound ? (int)a.compoundCount : 1;
    int countB = b.shape == ShapeType::Compound ? (int)b.compoundCount : 1;
    int total = 0;
    for (int ca = 0; ca < countA && total < cap; ++ca) {
        Body geometryA = a.shape == ShapeType::Compound
            ? compoundChildBody(a, soup.compoundChildren[a.compoundFirst + ca]) : a;
        for (int cb = 0; cb < countB && total < cap; ++cb) {
            Body geometryB = b.shape == ShapeType::Compound
                ? compoundChildBody(b, soup.compoundChildren[b.compoundFirst + cb]) : b;
            int added = collideSimplePair(geometryA, geometryB, ia, ib, soup, dt,
                                          out + total, cap - total);
            for (int k = 0; k < added; ++k) {
                Contact& contact = out[total + k];
                const Body& ownerA = contact.a == ia ? a : b;
                const Body& ownerB = contact.b == ib ? b : a;
                contact.localAnchorA = rotateInv(
                    ownerA.orientation, contact.point - ownerA.position);
                contact.localAnchorB = rotateInv(
                    ownerB.orientation, contact.point - ownerB.position);
                contact.vn0 = dot(npPointVelocity(ownerA, contact.point) -
                                  npPointVelocity(ownerB, contact.point),
                                  contact.normal);
                uint64_t childTag = (uint64_t)(ca + 1) * 0x9e3779b185ebca87ull ^
                                    (uint64_t)(cb + 1) * 0xc2b2ae3d27d4eb4full;
                contact.featureKey ^= childTag;
            }
            total += added;
        }
    }
    return total;
}

// Minimum gap + contact normal between a convex body and a mesh, BVH-pruned
// but exact within an expanded search box — the distance oracle used by the
// conservative-advancement safety net.
VELOX_HD inline void meshSimpleGapProbe(const Body& conv, const Body& meshBody,
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

VELOX_HD inline void meshGapProbe(const Body& conv, const Body& meshBody,
                                  const MeshSoupView& soup, float searchRadius,
                                  float& bestGap, Vec3& bestNormal) {
    if (conv.shape != ShapeType::Compound) {
        meshSimpleGapProbe(conv, meshBody, soup, searchRadius, bestGap, bestNormal);
        return;
    }
    for (uint32_t i = 0; i < conv.compoundCount; ++i) {
        Body child = compoundChildBody(
            conv, soup.compoundChildren[conv.compoundFirst + i]);
        meshSimpleGapProbe(child, meshBody, soup, searchRadius, bestGap, bestNormal);
    }
}

} // namespace velox
