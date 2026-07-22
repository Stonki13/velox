#pragma once

#include "math.h"
#include <cstdint>
#include <cmath>

namespace velox {

// ---------------------------------------------------------------------------
// Motion quality tiers
// ---------------------------------------------------------------------------

enum class MotionQuality : uint8_t { Low = 0, Medium = 1, High = 2, Locked = 3 };

// ---------------------------------------------------------------------------
// CCD quality presets (fast / balanced / accurate)
// ---------------------------------------------------------------------------

/// @brief Pre-tuned CCD quality levels that trade accuracy for speed.
///
/// - Fast:     minimal iterations, linear-only sweep, large tolerance.
///             Best for background / low-priority bodies.
/// - Balanced: moderate iterations, rotational CCD, medium tolerance.
///             Default for most dynamic bodies.
/// - Accurate: high iteration counts, full rotational + compound CCD,
///             tight tolerance, Brent root-finding. For bullets / fast objects.
enum class CcdQuality : uint8_t { Fast = 0, Balanced = 1, Accurate = 2 };

/// @brief Tuning parameters derived from a CcdQuality preset.
struct CcdQualitySettings {
    CcdQuality quality = CcdQuality::Balanced;
    uint32_t maxAdvancementIterations = 16;  ///< Conservative-advancement loop cap.
    uint32_t maxRootFindIterations = 12;     ///< Bisection / Brent refinement cap.
    float distanceTolerance = 1e-3f;         ///< Gap threshold for contact (world units).
    float timeTolerance = 1e-5f;             ///< TOI convergence threshold (seconds).
    bool enableRotationalCcd = true;         ///< Account for angular velocity in sweep.
    bool enableCompoundCcd = true;           ///< Sweep compound children individually.
    bool useBrentRootFinder = true;          ///< Brent's method vs plain bisection.
    uint32_t meshBvhMaxDepth = 8;            ///< Max BVH traversal depth for mesh CCD.
    float speculativeMargin = 0.01f;         ///< Extra sweep reach for speculative contacts.
};

/// @brief Return the CcdQualitySettings for a given preset.
VELOX_HD inline CcdQualitySettings ccdQualityPreset(CcdQuality q) {
    CcdQualitySettings s;
    s.quality = q;
    switch (q) {
    case CcdQuality::Fast:
        s.maxAdvancementIterations = 8;
        s.maxRootFindIterations = 6;
        s.distanceTolerance = 5e-3f;
        s.timeTolerance = 1e-4f;
        s.enableRotationalCcd = false;
        s.enableCompoundCcd = false;
        s.useBrentRootFinder = false;
        s.meshBvhMaxDepth = 4;
        s.speculativeMargin = 0.02f;
        break;
    case CcdQuality::Balanced:
        s.maxAdvancementIterations = 16;
        s.maxRootFindIterations = 12;
        s.distanceTolerance = 1e-3f;
        s.timeTolerance = 1e-5f;
        s.enableRotationalCcd = true;
        s.enableCompoundCcd = true;
        s.useBrentRootFinder = true;
        s.meshBvhMaxDepth = 8;
        s.speculativeMargin = 0.01f;
        break;
    case CcdQuality::Accurate:
        s.maxAdvancementIterations = 32;
        s.maxRootFindIterations = 24;
        s.distanceTolerance = 1e-4f;
        s.timeTolerance = 1e-6f;
        s.enableRotationalCcd = true;
        s.enableCompoundCcd = true;
        s.useBrentRootFinder = true;
        s.meshBvhMaxDepth = 16;
        s.speculativeMargin = 0.005f;
        break;
    }
    return s;
}

// ---------------------------------------------------------------------------
// Per-body CCD tuning (existing API, extended)
// ---------------------------------------------------------------------------

struct BodyCcdTuning {
    MotionQuality quality = MotionQuality::Medium;
    float collisionMargin = 0.0f;
    float speculativeDistance = 0.0f;
    bool enableContinuous = true;
    float minVelocityForCCD = 1.0f;
    CcdQualitySettings ccdSettings = ccdQualityPreset(CcdQuality::Balanced);
};

struct WorldCcdDefaults {
    MotionQuality defaultQuality = MotionQuality::Medium;
    float defaultCollisionMargin = 0.0f;
    float defaultSpeculativeDistance = 0.01f;
    bool defaultEnableContinuous = true;
    float defaultMinVelocityForCCD = 1.0f;
    CcdQuality defaultCcdQuality = CcdQuality::Balanced;
};

// ---------------------------------------------------------------------------
// Multi-TOI configuration (existing API)
// ---------------------------------------------------------------------------

struct CcdConfig {
    bool enabled = true;
    uint32_t maxToiEventsPerBody = 4;
    float toiVelocityFloor = 0.1f;
    float toiPenetrationBias = 1e-3f;
};

struct WorldMultiToiSettings {
    CcdConfig defaultConfig;
    uint32_t maxTotalEventsPerStep = 256;
    bool enableSubstepSplitting = true;
};

// ---------------------------------------------------------------------------
// Swept volume computation
// ---------------------------------------------------------------------------

/// @brief Axis-aligned bounding box.
struct Aabb {
    Vec3 lo{1e30f, 1e30f, 1e30f};
    Vec3 hi{-1e30f, -1e30f, -1e30f};

    VELOX_HD void expand(const Vec3& p) {
        lo = vmin(lo, p);
        hi = vmax(hi, p);
    }
    VELOX_HD void expand(const Aabb& o) {
        lo = vmin(lo, o.lo);
        hi = vmax(hi, o.hi);
    }
    VELOX_HD void inflate(float margin) {
        Vec3 m{margin, margin, margin};
        lo = lo - m;
        hi = hi + m;
    }
    VELOX_HD bool overlaps(const Aabb& o) const {
        return lo.x <= o.hi.x && hi.x >= o.lo.x &&
               lo.y <= o.hi.y && hi.y >= o.lo.y &&
               lo.z <= o.hi.z && hi.z >= o.lo.z;
    }
    VELOX_HD Vec3 center() const { return (lo + hi) * 0.5f; }
    VELOX_HD Vec3 extents() const { return (hi - lo) * 0.5f; }
    VELOX_HD float surfaceArea() const {
        Vec3 e = hi - lo;
        return 2.0f * (e.x * e.y + e.y * e.z + e.z * e.x);
    }
};

/// @brief A swept volume: the Minkowski sum of a shape's trajectory over [0, dt].
///
/// Used for broad-phase CCD pruning. The AABB is conservative (always contains
/// the true swept volume); the capsule and OBB fields provide tighter bounds
/// for convex shapes.
struct SweptVolume {
    Aabb aabb;                    ///< Conservative axis-aligned bound of the sweep.
    Vec3 startPos, endPos;        ///< Linear trajectory endpoints.
    float boundingRadius = 0.0f;  ///< Shape's local bounding-sphere radius.
    float sweepLength = 0.0f;     ///< Linear distance travelled.
    float angularSweepRad = 0.0f; ///< Total angular displacement (radians).
    bool valid = false;
};

/// @brief Compute the swept AABB for a convex shape moving linearly.
///
/// @param startPos    Position at t=0.
/// @param endPos      Position at t=dt.
/// @param halfExtents Local half-extents (box) or bounding radius (sphere).
/// @param radius      Inflation radius (capsule/sphere).
/// @return The swept volume with a conservative AABB.
VELOX_HD inline SweptVolume computeSweptVolume(const Vec3& startPos,
                                                const Vec3& endPos,
                                                const Vec3& halfExtents,
                                                float radius) {
    SweptVolume sv;
    sv.startPos = startPos;
    sv.endPos = endPos;
    sv.boundingRadius = length(halfExtents) + radius;
    sv.sweepLength = length(endPos - startPos);
    sv.angularSweepRad = 0.0f;

    Vec3 r{sv.boundingRadius, sv.boundingRadius, sv.boundingRadius};
    sv.aabb.lo = vmin(startPos, endPos) - r;
    sv.aabb.hi = vmax(startPos, endPos) + r;
    sv.valid = true;
    return sv;
}

/// @brief Compute the swept AABB accounting for rotation.
///
/// The rotational contribution inflates the bounding sphere by the angular
/// displacement times the bounding radius (arc-length bound).
VELOX_HD inline SweptVolume computeSweptVolumeRotational(const Vec3& startPos,
                                                          const Vec3& endPos,
                                                          const Vec3& halfExtents,
                                                          float radius,
                                                          float angularSpeed,
                                                          float dt) {
    SweptVolume sv = computeSweptVolume(startPos, endPos, halfExtents, radius);
    sv.angularSweepRad = angularSpeed * dt;
    // Rotational inflation: the bounding sphere grows by the arc swept at the
    // extremal radius.  For small angles this is ~boundingRadius * angle.
    float rotInflation = sv.boundingRadius * sv.angularSweepRad;
    sv.aabb.inflate(rotInflation);
    return sv;
}

// ---------------------------------------------------------------------------
// Conservative advancement algorithm
// ---------------------------------------------------------------------------

/// @brief Result of a conservative-advancement TOI query.
struct ToiResult {
    bool hit = false;          ///< True if a collision was found within [0, dt].
    float toi = 0.0f;         ///< Time of impact (seconds from start).
    float fraction = 0.0f;    ///< toi / dt.
    Vec3 point;               ///< World-space contact point at TOI.
    Vec3 normal;              ///< Contact normal (from B toward A).
    float separation = 0.0f;  ///< Signed distance at TOI (negative = penetrating).
    uint32_t iterations = 0;  ///< Number of advancement iterations used.
};

/// @brief A distance-oracle callback: returns the signed gap between two
///        configurations at a given time, plus the contact normal.
///
/// Positive = separated, negative = penetrating. The normal points from B
/// toward A. Implementations typically wrap GJK/EPA.
using DistanceOracle = float (*)(void* ctx, float t, Vec3& normal, Vec3& point);

/// @brief Conservative advancement with configurable root-finding.
///
/// Advances time from 0 toward dt, stepping by (gap / relativeSpeed) each
/// iteration. Once the gap drops below `distanceTol`, a root-finding pass
/// (Brent's method or bisection) refines the TOI to within `timeTol`.
///
/// @param oracle          Distance query callback.
/// @param ctx             Opaque context for the oracle.
/// @param dt              Time horizon.
/// @param relativeSpeed   Upper bound on the relative surface speed.
/// @param settings        Quality settings controlling iteration caps and tolerances.
/// @return ToiResult with hit=true if a collision was found.
inline ToiResult conservativeAdvancement(DistanceOracle oracle,
                                         void* ctx,
                                         float dt,
                                         float relativeSpeed,
                                         const CcdQualitySettings& settings) {
    ToiResult result;
    if (relativeSpeed < 1e-10f || dt <= 0.0f) return result;

    const float distTol = settings.distanceTolerance;
    const float timeTol = settings.timeTolerance;
    const uint32_t maxAdv = settings.maxAdvancementIterations;
    const uint32_t maxRoot = settings.maxRootFindIterations;

    // Phase 1: conservative advancement
    float t = 0.0f;
    Vec3 normal{}, point{};
    float gap = oracle(ctx, 0.0f, normal, point);

    if (gap <= distTol) {
        // Already in contact at t=0.
        result.hit = true;
        result.toi = 0.0f;
        result.fraction = 0.0f;
        result.normal = normal;
        result.point = point;
        result.separation = gap;
        return result;
    }

    float lower = 0.0f;
    float upper = dt;
    uint32_t iter = 0;

    for (; iter < maxAdv && t < dt; ++iter) {
        float advance = (gap - distTol) / relativeSpeed;
        if (advance < timeTol) break;
        t = vmin(dt, t + advance);
        gap = oracle(ctx, t, normal, point);
        if (gap <= distTol) {
            upper = t;
            break;
        }
        lower = t;
    }

    if (gap > distTol) return result; // no collision within dt

    // Phase 2: root-finding refinement
    float a = lower, b = upper;
    float fa = gap > distTol ? gap : 0.0f; // gap at lower (positive)
    // Recompute gap at lower for Brent's method
    Vec3 nA{}, pA{};
    fa = oracle(ctx, a, nA, pA);
    Vec3 nB{}, pB{};
    float fb = oracle(ctx, b, nB, pB);

    if (settings.useBrentRootFinder && fa * fb < 0.0f) {
        // Brent's method (simplified Illinois variant)
        float c = a, fc = fa;
        float d = b - a, e = d;
        for (uint32_t i = 0; i < maxRoot; ++i) {
            if (fb * fc > 0.0f) { c = a; fc = fa; d = b - a; e = d; }
            if (fabsf(fc) < fabsf(fb)) {
                a = b; b = c; c = a;
                fa = fb; fb = fc; fc = fa;
            }
            float tol = 2.0f * 1e-7f * fabsf(b) + timeTol;
            float m = 0.5f * (c - b);
            if (fabsf(m) <= tol || fabsf(fb) < distTol * 0.01f) break;
            if (fabsf(e) >= tol && fabsf(fa) > fabsf(fb)) {
                float s = fb / fa;
                float p, q;
                if (fabsf(a - c) < 1e-12f) {
                    p = 2.0f * m * s;
                    q = 1.0f - s;
                } else {
                    q = fa / fc;
                    float r = fb / fc;
                    p = s * (2.0f * m * q * (q - r) - (b - a) * (r - 1.0f));
                    q = (q - 1.0f) * (r - 1.0f) * (s - 1.0f);
                }
                if (p > 0.0f) q = -q; else p = -p;
                if (2.0f * p < vmin(3.0f * m * q - fabsf(tol * q), fabsf(e * q))) {
                    e = d; d = p / q;
                } else { d = m; e = m; }
            } else { d = m; e = m; }
            a = b; fa = fb;
            if (fabsf(d) > tol) b += d;
            else b += (m > 0.0f ? tol : -tol);
            Vec3 nTmp{}, pTmp{};
            fb = oracle(ctx, b, nTmp, pTmp);
            if (fb <= 0.0f) { normal = nTmp; point = pTmp; }
        }
    } else {
        // Bisection fallback
        for (uint32_t i = 0; i < maxRoot; ++i) {
            float mid = (a + b) * 0.5f;
            if (b - a < timeTol) break;
            Vec3 nMid{}, pMid{};
            float fmid = oracle(ctx, mid, nMid, pMid);
            if (fmid <= distTol) {
                b = mid;
                normal = nMid;
                point = pMid;
            } else {
                a = mid;
            }
        }
    }

    result.hit = true;
    result.toi = b;
    result.fraction = (dt > 0.0f) ? b / dt : 0.0f;
    result.normal = normal;
    result.point = point;
    result.separation = fb;
    result.iterations = iter;
    return result;
}

// ---------------------------------------------------------------------------
// Rotational CCD support
// ---------------------------------------------------------------------------

/// @brief Compute an upper bound on the relative surface speed between two
///        bodies accounting for both linear and angular velocity.
///
/// The bound is: |vA - vB| + |omegaA| * rA + |omegaB| * rB
/// where rA, rB are the bounding radii of the shapes.
VELOX_HD inline float relativeSurfaceSpeed(const Vec3& velA, const Vec3& omegaA, float boundRadiusA,
                                           const Vec3& velB, const Vec3& omegaB, float boundRadiusB) {
    float linearSpeed = length(velA - velB);
    float angularA = length(omegaA) * boundRadiusA;
    float angularB = length(omegaB) * boundRadiusB;
    return linearSpeed + angularA + angularB;
}

/// @brief Compute the angular sweep angle for a body over dt.
VELOX_HD inline float angularSweepAngle(const Vec3& omega, float dt) {
    return length(omega) * dt;
}

/// @brief Interpolate orientation at time t given initial orientation and
///        constant angular velocity (slerp approximation via integrate()).
VELOX_HD inline Quat orientationAt(const Quat& q0, const Vec3& omega, float t) {
    return integrate(q0, omega, t);
}

/// @brief Interpolate position at time t given initial position and constant
///        linear velocity.
VELOX_HD inline Vec3 positionAt(const Vec3& p0, const Vec3& vel, float t) {
    return p0 + vel * t;
}

// ---------------------------------------------------------------------------
// Compound shape CCD
// ---------------------------------------------------------------------------

/// @brief Result of a compound-shape TOI query: the earliest TOI across all
///        children, plus which child index produced it.
struct CompoundToiResult {
    ToiResult toi;               ///< Best (earliest) TOI across children.
    int32_t childIndex = -1;     ///< Index of the child that produced the hit.
};

/// @brief Configuration for compound-shape CCD traversal.
struct CompoundCcdConfig {
    bool testAllChildren = true;   ///< Test every child vs every child of the other compound.
    uint32_t maxChildPairs = 64;   ///< Cap on child-pair tests per compound pair.
    bool earlyOut = true;          ///< Stop after first hit if TOI is very small.
    float earlyOutThreshold = 1e-4f; ///< TOI below which early-out triggers.
};

// ---------------------------------------------------------------------------
// Mesh CCD optimization
// ---------------------------------------------------------------------------

/// @brief BVH node for mesh CCD acceleration.
///
/// A binary tree over mesh triangles. During CCD, the swept volume is tested
/// against BVH nodes; only subtrees whose AABB overlaps the swept AABB are
/// traversed, reducing triangle-level TOI tests from O(n) to O(log n + k).
struct MeshBvhNode {
    Aabb aabb;                   ///< Conservative AABB of all triangles below.
    int32_t left = -1;           ///< Left child index (-1 = leaf).
    int32_t right = -1;          ///< Right child index (-1 = leaf).
    uint32_t triFirst = 0;       ///< First triangle index (leaf only).
    uint32_t triCount = 0;       ///< Triangle count (leaf only).
    bool isLeaf() const { return left < 0 && right < 0; }
};

/// @brief Configuration for mesh CCD traversal.
struct MeshCcdConfig {
    uint32_t maxBvhDepth = 8;         ///< Maximum BVH traversal depth.
    uint32_t maxTriangleTests = 128;  ///< Cap on triangle-level TOI tests.
    bool useSweptAabbPruning = true;  ///< Prune BVH nodes by swept-AABB overlap.
    float triangleInflation = 0.0f;   ///< Extra margin on triangle AABBs.
};

/// @brief Statistics from a mesh CCD query (for profiling / debugging).
struct MeshCcdStats {
    uint32_t nodesVisited = 0;
    uint32_t trianglesTested = 0;
    uint32_t nodesPruned = 0;
    float queryTimeMs = 0.0f;
};

// ---------------------------------------------------------------------------
// Multi-TOI event scheduling
// ---------------------------------------------------------------------------

/// @brief A scheduled TOI event for the multi-TOI event queue.
///
/// The event scheduler collects TOI hits from all high-quality body pairs,
/// sorts them chronologically, and replays them in order within a step.
struct ToiEvent {
    float time = 0.0f;          ///< Absolute time of impact within the step.
    uint32_t bodyA = UINT32_MAX; ///< Index of body A.
    uint32_t bodyB = UINT32_MAX; ///< Index of body B.
    Vec3 point;                  ///< Contact point at TOI.
    Vec3 normal;                 ///< Contact normal (B -> A).
    float separation = 0.0f;    ///< Signed distance at TOI.
    uint32_t priority = 0;      ///< Tie-breaking priority (lower = first).
};

/// @brief Comparator for chronological event ordering.
struct ToiEventOrder {
    bool operator()(const ToiEvent& a, const ToiEvent& b) const {
        if (a.time != b.time) return a.time < b.time;
        if (a.bodyA != b.bodyA) return a.bodyA < b.bodyA;
        return a.bodyB < b.bodyB;
    }
};

/// @brief Configuration for the multi-TOI event scheduler.
struct MultiToiSchedulerConfig {
    uint32_t maxEventsPerBody = 4;      ///< Per-body event cap.
    uint32_t maxTotalEvents = 256;      ///< Global event cap per step.
    float minTimeSeparation = 1e-5f;    ///< Minimum time gap between events for the same pair.
    bool mergeCloseEvents = true;       ///< Merge events closer than minTimeSeparation.
    bool enableTemporalCoherence = true; ///< Reuse previous frame's TOI as a lower bound.
};

// ---------------------------------------------------------------------------
// TOI query descriptors
// ---------------------------------------------------------------------------

/// @brief Describes a single TOI query between two moving convex shapes.
struct ToiQueryDesc {
    // Body A state
    Vec3 posA, velA, omegaA;
    Quat rotA;
    Vec3 halfExtentsA;
    float radiusA = 0.0f;
    float boundRadiusA = 0.0f;

    // Body B state
    Vec3 posB, velB, omegaB;
    Quat rotB;
    Vec3 halfExtentsB;
    float radiusB = 0.0f;
    float boundRadiusB = 0.0f;

    float dt = 1.0f / 60.0f;
    CcdQualitySettings settings;
};

/// @brief Compute the swept volume for a TOI query descriptor's body A.
VELOX_HD inline SweptVolume sweptVolumeForQuery(const ToiQueryDesc& desc, bool bodyA) {
    const Vec3& pos = bodyA ? desc.posA : desc.posB;
    const Vec3& vel = bodyA ? desc.velA : desc.velB;
    const Vec3& omega = bodyA ? desc.omegaA : desc.omegaB;
    const Vec3& he = bodyA ? desc.halfExtentsA : desc.halfExtentsB;
    float r = bodyA ? desc.radiusA : desc.radiusB;

    Vec3 endPos = pos + vel * desc.dt;
    float angSpeed = length(omega);

    if (angSpeed > 1e-8f) {
        return computeSweptVolumeRotational(pos, endPos, he, r, angSpeed, desc.dt);
    }
    return computeSweptVolume(pos, endPos, he, r);
}

/// @brief Quick rejection test: do the swept volumes of two bodies overlap?
VELOX_HD inline bool sweptVolumesOverlap(const ToiQueryDesc& desc) {
    SweptVolume svA = sweptVolumeForQuery(desc, true);
    SweptVolume svB = sweptVolumeForQuery(desc, false);
    return svA.aabb.overlaps(svB.aabb);
}

} // namespace velox
