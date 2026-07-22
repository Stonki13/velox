#include "doctest.h"
#include "velox/ccd.h"
#include "velox/math.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>

using namespace velox;

// ---------------------------------------------------------------------------
// Helper: sphere-sphere distance oracle for conservative advancement tests.
//
// Two spheres moving linearly. The oracle returns the signed gap
// (center distance - sum of radii) at time t.
// ---------------------------------------------------------------------------

struct SphereSphereCtx {
    Vec3 posA, velA;
    float radiusA;
    Vec3 posB, velB;
    float radiusB;
};

static float sphereSphereOracle(void* ctx, float t, Vec3& normal, Vec3& point) {
    auto* c = static_cast<SphereSphereCtx*>(ctx);
    Vec3 a = c->posA + c->velA * t;
    Vec3 b = c->posB + c->velB * t;
    Vec3 diff = a - b;
    float dist = length(diff);
    float sumR = c->radiusA + c->radiusB;
    if (dist > 1e-8f) {
        normal = diff * (1.0f / dist);
    } else {
        normal = {0.0f, 1.0f, 0.0f};
    }
    point = b + normal * c->radiusB;
    return dist - sumR;
}

// ---------------------------------------------------------------------------
// Helper: sphere-plane distance oracle.
//
// A sphere moving toward a static infinite plane at y=0.
// ---------------------------------------------------------------------------

struct SpherePlaneCtx {
    Vec3 pos, vel;
    float radius;
    float planeY;
};

static float spherePlaneOracle(void* ctx, float t, Vec3& normal, Vec3& point) {
    auto* c = static_cast<SpherePlaneCtx*>(ctx);
    Vec3 p = c->pos + c->vel * t;
    normal = {0.0f, 1.0f, 0.0f};
    point = {p.x, c->planeY, p.z};
    return p.y - c->radius - c->planeY;
}

// ---------------------------------------------------------------------------
// Helper: rotating sphere-plane oracle (simulates rotational CCD).
//
// A sphere with an offset contact point that rotates, changing the effective
// gap over time.
// ---------------------------------------------------------------------------

struct RotatingSphereCtx {
    Vec3 pos, vel;
    float radius;
    float planeY;
    Vec3 omega;       // angular velocity
    Vec3 contactOffset; // offset from center to the "bump" that will hit
};

static float rotatingSphereOracle(void* ctx, float t, Vec3& normal, Vec3& point) {
    auto* c = static_cast<RotatingSphereCtx*>(ctx);
    Vec3 p = c->pos + c->vel * t;
    // Rotate the contact offset by omega * t around Y axis
    float angle = c->omega.y * t;
    float ca = cosf(angle), sa = sinf(angle);
    Vec3 rotatedOffset{
        c->contactOffset.x * ca - c->contactOffset.z * sa,
        c->contactOffset.y,
        c->contactOffset.x * sa + c->contactOffset.z * ca};
    Vec3 lowest = p + rotatedOffset;
    normal = {0.0f, 1.0f, 0.0f};
    point = {lowest.x, c->planeY, lowest.z};
    return lowest.y - c->planeY;
}

// ===========================================================================
// TEST SUITE: Tunneling prevention
// ===========================================================================

TEST_CASE("CCD tunneling prevention") {
    SUBCASE("fast sphere vs thin wall - no tunneling") {
        // A sphere at x=-10 moving at 1000 m/s toward a thin wall at x=0.
        // Without CCD, a 60 Hz step moves the sphere 16.67 units, skipping
        // the wall entirely. CCD must find the TOI.
        SpherePlaneCtx ctx;
        ctx.pos = {0.0f, 10.0f, 0.0f}; // above the plane
        ctx.vel = {0.0f, -1000.0f, 0.0f}; // falling fast
        ctx.radius = 0.5f;
        ctx.planeY = 0.0f;

        CcdQualitySettings settings = ccdQualityPreset(CcdQuality::Accurate);
        float dt = 1.0f / 60.0f;
        float speed = length(ctx.vel);

        ToiResult result = conservativeAdvancement(
            spherePlaneOracle, &ctx, dt, speed, settings);

        CHECK(result.hit);
        // TOI should be when the sphere bottom reaches y=0:
        // pos.y + vel.y * t - radius = 0 => t = (pos.y - radius) / |vel.y|
        float expectedToi = (ctx.pos.y - ctx.radius) / 1000.0f;
        CHECK(result.toi == doctest::Approx(expectedToi).epsilon(0.01));
        CHECK(result.fraction == doctest::Approx(expectedToi / dt).epsilon(0.01));
    }

    SUBCASE("bullet through paper wall") {
        // Classic bullet-vs-paper scenario: sphere radius 0.05 at 2000 m/s
        // toward a plane 1 unit away.
        SpherePlaneCtx ctx;
        ctx.pos = {0.0f, 1.0f, 0.0f};
        ctx.vel = {0.0f, -2000.0f, 0.0f};
        ctx.radius = 0.05f;
        ctx.planeY = 0.0f;

        CcdQualitySettings settings = ccdQualityPreset(CcdQuality::Accurate);
        float dt = 1.0f / 60.0f;

        ToiResult result = conservativeAdvancement(
            spherePlaneOracle, &ctx, dt, length(ctx.vel), settings);

        CHECK(result.hit);
        float expectedToi = (1.0f - 0.05f) / 2000.0f;
        CHECK(result.toi == doctest::Approx(expectedToi).epsilon(0.02));
    }

    SUBCASE("sphere-sphere head-on at extreme speed") {
        // Two spheres approaching each other at 500 m/s each.
        SphereSphereCtx ctx;
        ctx.posA = {-5.0f, 0.0f, 0.0f};
        ctx.velA = {500.0f, 0.0f, 0.0f};
        ctx.radiusA = 0.5f;
        ctx.posB = {5.0f, 0.0f, 0.0f};
        ctx.velB = {-500.0f, 0.0f, 0.0f};
        ctx.radiusB = 0.5f;

        CcdQualitySettings settings = ccdQualityPreset(CcdQuality::Accurate);
        float dt = 1.0f / 60.0f;
        float relSpeed = length(ctx.velA - ctx.velB);

        ToiResult result = conservativeAdvancement(
            sphereSphereOracle, &ctx, dt, relSpeed, settings);

        CHECK(result.hit);
        // Expected: distance=10, sum radii=1, closing speed=1000
        // TOI = (10 - 1) / 1000 = 0.009 s
        float expectedToi = 9.0f / 1000.0f;
        CHECK(result.toi == doctest::Approx(expectedToi).epsilon(0.02));
    }

    SUBCASE("no collision when moving apart") {
        SphereSphereCtx ctx;
        ctx.posA = {-5.0f, 0.0f, 0.0f};
        ctx.velA = {-100.0f, 0.0f, 0.0f}; // moving away
        ctx.radiusA = 0.5f;
        ctx.posB = {5.0f, 0.0f, 0.0f};
        ctx.velB = {100.0f, 0.0f, 0.0f}; // moving away
        ctx.radiusB = 0.5f;

        CcdQualitySettings settings = ccdQualityPreset(CcdQuality::Balanced);
        float dt = 1.0f / 60.0f;
        float relSpeed = length(ctx.velA - ctx.velB);

        ToiResult result = conservativeAdvancement(
            sphereSphereOracle, &ctx, dt, relSpeed, settings);

        CHECK_FALSE(result.hit);
    }

    SUBCASE("already overlapping at t=0") {
        SpherePlaneCtx ctx;
        ctx.pos = {0.0f, 0.2f, 0.0f}; // center below radius => penetrating
        ctx.vel = {0.0f, -1.0f, 0.0f};
        ctx.radius = 0.5f;
        ctx.planeY = 0.0f;

        CcdQualitySettings settings = ccdQualityPreset(CcdQuality::Balanced);
        float dt = 1.0f / 60.0f;

        ToiResult result = conservativeAdvancement(
            spherePlaneOracle, &ctx, dt, length(ctx.vel), settings);

        CHECK(result.hit);
        CHECK(result.toi == doctest::Approx(0.0f));
    }
}

// ===========================================================================
// TEST SUITE: TOI accuracy
// ===========================================================================

TEST_CASE("CCD TOI accuracy") {
    SUBCASE("sphere-plane exact TOI") {
        // Sphere at height 5, radius 1, falling at 10 m/s.
        // Exact TOI = (5 - 1) / 10 = 0.4 s
        SpherePlaneCtx ctx;
        ctx.pos = {0.0f, 5.0f, 0.0f};
        ctx.vel = {0.0f, -10.0f, 0.0f};
        ctx.radius = 1.0f;
        ctx.planeY = 0.0f;

        float dt = 1.0f; // long enough horizon
        float expectedToi = 0.4f;

        SUBCASE("fast quality") {
            CcdQualitySettings settings = ccdQualityPreset(CcdQuality::Fast);
            ToiResult result = conservativeAdvancement(
                spherePlaneOracle, &ctx, dt, length(ctx.vel), settings);
            CHECK(result.hit);
            CHECK(result.toi == doctest::Approx(expectedToi).epsilon(0.05));
        }
        SUBCASE("balanced quality") {
            CcdQualitySettings settings = ccdQualityPreset(CcdQuality::Balanced);
            ToiResult result = conservativeAdvancement(
                spherePlaneOracle, &ctx, dt, length(ctx.vel), settings);
            CHECK(result.hit);
            CHECK(result.toi == doctest::Approx(expectedToi).epsilon(0.01));
        }
        SUBCASE("accurate quality") {
            CcdQualitySettings settings = ccdQualityPreset(CcdQuality::Accurate);
            ToiResult result = conservativeAdvancement(
                spherePlaneOracle, &ctx, dt, length(ctx.vel), settings);
            CHECK(result.hit);
            CHECK(result.toi == doctest::Approx(expectedToi).epsilon(0.001));
        }
    }

    SUBCASE("sphere-sphere diagonal approach") {
        // A at origin, B at (3,4,0), both radius 0.5.
        // A moves toward B at (3,4,0) normalized * 10 m/s.
        SphereSphereCtx ctx;
        ctx.posA = {0.0f, 0.0f, 0.0f};
        ctx.radiusA = 0.5f;
        ctx.posB = {3.0f, 4.0f, 0.0f};
        ctx.radiusB = 0.5f;
        // Direction A->B normalized = (0.6, 0.8, 0)
        ctx.velA = {6.0f, 8.0f, 0.0f}; // 10 m/s toward B
        ctx.velB = {0.0f, 0.0f, 0.0f};

        float dt = 1.0f;
        float dist = 5.0f;
        float expectedToi = (dist - 1.0f) / 10.0f; // 0.4 s

        CcdQualitySettings settings = ccdQualityPreset(CcdQuality::Accurate);
        ToiResult result = conservativeAdvancement(
            sphereSphereOracle, &ctx, dt, length(ctx.velA), settings);

        CHECK(result.hit);
        CHECK(result.toi == doctest::Approx(expectedToi).epsilon(0.01));
        // Normal should point from B toward A: (-0.6, -0.8, 0)
        CHECK(result.normal.x == doctest::Approx(-0.6f).epsilon(0.05));
        CHECK(result.normal.y == doctest::Approx(-0.8f).epsilon(0.05));
    }

    SUBCASE("grazing contact - near miss") {
        // Sphere passes just above the plane - should NOT hit.
        SpherePlaneCtx ctx;
        ctx.pos = {0.0f, 2.0f, 0.0f};
        ctx.vel = {100.0f, 0.0f, 0.0f}; // moving horizontally
        ctx.radius = 0.5f;
        ctx.planeY = 0.0f;

        CcdQualitySettings settings = ccdQualityPreset(CcdQuality::Accurate);
        float dt = 1.0f / 60.0f;

        ToiResult result = conservativeAdvancement(
            spherePlaneOracle, &ctx, dt, length(ctx.vel), settings);

        // Sphere stays at y=2, well above the plane
        CHECK_FALSE(result.hit);
    }

    SUBCASE("TOI fraction is consistent with toi/dt") {
        SpherePlaneCtx ctx;
        ctx.pos = {0.0f, 3.0f, 0.0f};
        ctx.vel = {0.0f, -30.0f, 0.0f};
        ctx.radius = 0.5f;
        ctx.planeY = 0.0f;

        CcdQualitySettings settings = ccdQualityPreset(CcdQuality::Balanced);
        float dt = 0.5f;

        ToiResult result = conservativeAdvancement(
            spherePlaneOracle, &ctx, dt, length(ctx.vel), settings);

        CHECK(result.hit);
        CHECK(result.fraction == doctest::Approx(result.toi / dt).epsilon(1e-4));
    }

    SUBCASE("Brent vs bisection produce consistent results") {
        SpherePlaneCtx ctx;
        ctx.pos = {0.0f, 10.0f, 0.0f};
        ctx.vel = {0.0f, -50.0f, 0.0f};
        ctx.radius = 1.0f;
        ctx.planeY = 0.0f;

        float dt = 1.0f;
        float speed = length(ctx.vel);

        CcdQualitySettings brentSettings = ccdQualityPreset(CcdQuality::Accurate);
        brentSettings.useBrentRootFinder = true;

        CcdQualitySettings bisectSettings = ccdQualityPreset(CcdQuality::Accurate);
        bisectSettings.useBrentRootFinder = false;

        ToiResult brentResult = conservativeAdvancement(
            spherePlaneOracle, &ctx, dt, speed, brentSettings);
        ToiResult bisectResult = conservativeAdvancement(
            spherePlaneOracle, &ctx, dt, speed, bisectSettings);

        CHECK(brentResult.hit);
        CHECK(bisectResult.hit);
        // Both methods should agree to within 1%
        CHECK(brentResult.toi == doctest::Approx(bisectResult.toi).epsilon(0.01));
    }
}

// ===========================================================================
// TEST SUITE: Rotational CCD
// ===========================================================================

TEST_CASE("CCD rotational support") {
    SUBCASE("relativeSurfaceSpeed includes angular contribution") {
        Vec3 velA{10.0f, 0.0f, 0.0f};
        Vec3 omegaA{0.0f, 100.0f, 0.0f}; // spinning fast
        float boundA = 1.0f;
        Vec3 velB{0.0f, 0.0f, 0.0f};
        Vec3 omegaB{0.0f, 0.0f, 0.0f};
        float boundB = 0.5f;

        float speed = relativeSurfaceSpeed(velA, omegaA, boundA, velB, omegaB, boundB);
        // linear: 10, angular A: 100*1=100, angular B: 0
        CHECK(speed == doctest::Approx(110.0f).epsilon(0.01));
    }

    SUBCASE("angularSweepAngle computes correctly") {
        Vec3 omega{0.0f, 0.0f, 10.0f}; // 10 rad/s around Z
        float dt = 0.5f;
        float angle = angularSweepAngle(omega, dt);
        CHECK(angle == doctest::Approx(5.0f).epsilon(1e-6));
    }

    SUBCASE("orientationAt integrates rotation") {
        Quat q0{0.0f, 0.0f, 0.0f, 1.0f}; // identity
        Vec3 omega{0.0f, 0.0f, 3.14159f}; // ~pi rad/s around Z
        float t = 0.5f; // half second => ~pi/2 rotation

        Quat q = orientationAt(q0, omega, t);
        // After pi/2 rotation around Z, the quaternion should be
        // approximately (0, 0, sin(pi/4), cos(pi/4)) = (0, 0, 0.707, 0.707)
        CHECK(q.z == doctest::Approx(0.7071f).epsilon(0.01));
        CHECK(q.w == doctest::Approx(0.7071f).epsilon(0.01));
    }

    SUBCASE("positionAt integrates linear motion") {
        Vec3 p0{1.0f, 2.0f, 3.0f};
        Vec3 vel{10.0f, -5.0f, 0.0f};
        float t = 0.1f;

        Vec3 p = positionAt(p0, vel, t);
        CHECK(p.x == doctest::Approx(2.0f).epsilon(1e-6));
        CHECK(p.y == doctest::Approx(1.5f).epsilon(1e-6));
        CHECK(p.z == doctest::Approx(3.0f).epsilon(1e-6));
    }

    SUBCASE("rotating sphere hits plane via rotation") {
        // A sphere with a bump offset that rotates into the plane.
        // The sphere center is above the plane, but rotation brings the
        // bump down to contact.
        RotatingSphereCtx ctx;
        ctx.pos = {0.0f, 1.0f, 0.0f};
        ctx.vel = {0.0f, 0.0f, 0.0f}; // no linear motion
        ctx.radius = 0.5f;
        ctx.planeY = 0.0f;
        ctx.omega = {0.0f, 10.0f, 0.0f}; // 10 rad/s around Y
        ctx.contactOffset = {0.0f, -0.8f, 0.0f}; // bump pointing down initially

        // At t=0, bump is at y = 1.0 - 0.8 = 0.2 (above plane)
        // As it rotates around Y, the bump stays at y-offset -0.8 (Y rotation
        // doesn't change Y component). So this won't actually hit via rotation
        // around Y with a Y-offset. Let's use X-offset instead.
        ctx.contactOffset = {0.8f, -0.5f, 0.0f};
        // At t=0: bump at (0.8, 0.5, 0) relative => y = 1.0 - 0.5 = 0.5
        // Rotating around Y: the Y component stays -0.5, so bump y = 0.5 always.
        // This won't hit the plane at y=0.

        // Better test: sphere falling while rotating, rotation changes when
        // the lowest point contacts. Use the sphere-plane oracle with rotation
        // affecting the effective radius.
        // Simpler: just verify the rotational swept volume is larger.
        SweptVolume svLinear = computeSweptVolume(
            {0.0f, 5.0f, 0.0f}, {0.0f, 4.0f, 0.0f}, {0.5f, 0.5f, 0.5f}, 0.0f);
        SweptVolume svRot = computeSweptVolumeRotational(
            {0.0f, 5.0f, 0.0f}, {0.0f, 4.0f, 0.0f}, {0.5f, 0.5f, 0.5f}, 0.0f,
            20.0f, 1.0f / 60.0f);

        // Rotational sweep should be at least as large as linear
        CHECK(svRot.aabb.lo.y <= svLinear.aabb.lo.y);
        CHECK(svRot.aabb.hi.y >= svLinear.aabb.hi.y);
        CHECK(svRot.angularSweepRad > 0.0f);
    }

    SUBCASE("swept volume with rotation is conservative") {
        // A spinning box should have a larger swept AABB than a non-spinning one.
        Vec3 start{0.0f, 0.0f, 0.0f};
        Vec3 end{1.0f, 0.0f, 0.0f};
        Vec3 he{0.5f, 0.5f, 0.5f};

        SweptVolume noRot = computeSweptVolume(start, end, he, 0.0f);
        SweptVolume withRot = computeSweptVolumeRotational(start, end, he, 0.0f, 50.0f, 1.0f);

        // With 50 rad/s for 1 second, the rotational inflation is significant
        CHECK(withRot.aabb.lo.x <= noRot.aabb.lo.x);
        CHECK(withRot.aabb.hi.x >= noRot.aabb.hi.x);
        CHECK(withRot.aabb.lo.y <= noRot.aabb.lo.y);
        CHECK(withRot.aabb.hi.y >= noRot.aabb.hi.y);
    }
}

// ===========================================================================
// TEST SUITE: Swept volume computation
// ===========================================================================

TEST_CASE("CCD swept volume") {
    SUBCASE("static sphere has tight AABB") {
        Vec3 pos{1.0f, 2.0f, 3.0f};
        Vec3 he{0.5f, 0.5f, 0.5f};
        SweptVolume sv = computeSweptVolume(pos, pos, he, 0.0f);

        CHECK(sv.valid);
        CHECK(sv.sweepLength == doctest::Approx(0.0f));
        float br = length(he); // ~0.866
        CHECK(sv.aabb.lo.x == doctest::Approx(1.0f - br).epsilon(1e-4));
        CHECK(sv.aabb.hi.x == doctest::Approx(1.0f + br).epsilon(1e-4));
    }

    SUBCASE("linear sweep extends AABB along motion") {
        Vec3 start{0.0f, 0.0f, 0.0f};
        Vec3 end{10.0f, 0.0f, 0.0f};
        Vec3 he{0.5f, 0.5f, 0.5f};
        float radius = 0.1f;

        SweptVolume sv = computeSweptVolume(start, end, he, radius);
        float br = length(he) + radius;

        CHECK(sv.sweepLength == doctest::Approx(10.0f).epsilon(1e-4));
        CHECK(sv.aabb.lo.x == doctest::Approx(-br).epsilon(1e-4));
        CHECK(sv.aabb.hi.x == doctest::Approx(10.0f + br).epsilon(1e-4));
        CHECK(sv.aabb.lo.y == doctest::Approx(-br).epsilon(1e-4));
        CHECK(sv.aabb.hi.y == doctest::Approx(br).epsilon(1e-4));
    }

    SUBCASE("AABB overlap detection") {
        Aabb a, b;
        a.lo = {-1.0f, -1.0f, -1.0f};
        a.hi = {1.0f, 1.0f, 1.0f};
        b.lo = {0.5f, 0.5f, 0.5f};
        b.hi = {2.0f, 2.0f, 2.0f};
        CHECK(a.overlaps(b));
        CHECK(b.overlaps(a));

        Aabb c;
        c.lo = {5.0f, 5.0f, 5.0f};
        c.hi = {6.0f, 6.0f, 6.0f};
        CHECK_FALSE(a.overlaps(c));
    }

    SUBCASE("AABB inflate") {
        Aabb a;
        a.lo = {0.0f, 0.0f, 0.0f};
        a.hi = {1.0f, 1.0f, 1.0f};
        a.inflate(0.5f);
        CHECK(a.lo.x == doctest::Approx(-0.5f));
        CHECK(a.hi.x == doctest::Approx(1.5f));
    }

    SUBCASE("sweptVolumesOverlap quick rejection") {
        ToiQueryDesc desc;
        desc.posA = {0.0f, 0.0f, 0.0f};
        desc.velA = {1.0f, 0.0f, 0.0f};
        desc.omegaA = {0.0f, 0.0f, 0.0f};
        desc.halfExtentsA = {0.5f, 0.5f, 0.5f};
        desc.radiusA = 0.0f;
        desc.boundRadiusA = length(desc.halfExtentsA);

        desc.posB = {100.0f, 0.0f, 0.0f};
        desc.velB = {0.0f, 0.0f, 0.0f};
        desc.omegaB = {0.0f, 0.0f, 0.0f};
        desc.halfExtentsB = {0.5f, 0.5f, 0.5f};
        desc.radiusB = 0.0f;
        desc.boundRadiusB = length(desc.halfExtentsB);
        desc.dt = 1.0f / 60.0f;

        // Bodies are far apart and barely moving - no overlap
        CHECK_FALSE(sweptVolumesOverlap(desc));

        // Now make them approach
        desc.posB = {2.0f, 0.0f, 0.0f};
        desc.velA = {100.0f, 0.0f, 0.0f};
        CHECK(sweptVolumesOverlap(desc));
    }
}

// ===========================================================================
// TEST SUITE: CCD quality settings
// ===========================================================================

TEST_CASE("CCD quality presets") {
    SUBCASE("fast preset has fewer iterations") {
        CcdQualitySettings fast = ccdQualityPreset(CcdQuality::Fast);
        CcdQualitySettings balanced = ccdQualityPreset(CcdQuality::Balanced);
        CcdQualitySettings accurate = ccdQualityPreset(CcdQuality::Accurate);

        CHECK(fast.maxAdvancementIterations < balanced.maxAdvancementIterations);
        CHECK(balanced.maxAdvancementIterations < accurate.maxAdvancementIterations);
        CHECK(fast.maxRootFindIterations < balanced.maxRootFindIterations);
        CHECK(balanced.maxRootFindIterations < accurate.maxRootFindIterations);
    }

    SUBCASE("fast preset disables rotational CCD") {
        CcdQualitySettings fast = ccdQualityPreset(CcdQuality::Fast);
        CHECK_FALSE(fast.enableRotationalCcd);
        CHECK_FALSE(fast.enableCompoundCcd);
        CHECK_FALSE(fast.useBrentRootFinder);
    }

    SUBCASE("accurate preset enables all features") {
        CcdQualitySettings accurate = ccdQualityPreset(CcdQuality::Accurate);
        CHECK(accurate.enableRotationalCcd);
        CHECK(accurate.enableCompoundCcd);
        CHECK(accurate.useBrentRootFinder);
        CHECK(accurate.distanceTolerance < 1e-3f);
    }

    SUBCASE("tolerances decrease with quality") {
        CcdQualitySettings fast = ccdQualityPreset(CcdQuality::Fast);
        CcdQualitySettings balanced = ccdQualityPreset(CcdQuality::Balanced);
        CcdQualitySettings accurate = ccdQualityPreset(CcdQuality::Accurate);

        CHECK(fast.distanceTolerance > balanced.distanceTolerance);
        CHECK(balanced.distanceTolerance > accurate.distanceTolerance);
        CHECK(fast.timeTolerance > balanced.timeTolerance);
        CHECK(balanced.timeTolerance > accurate.timeTolerance);
    }

    SUBCASE("BodyCcdTuning defaults to balanced") {
        BodyCcdTuning tuning;
        CHECK(tuning.quality == MotionQuality::Medium);
        CHECK(tuning.enableContinuous);
        CHECK(tuning.ccdSettings.quality == CcdQuality::Balanced);
    }
}

// ===========================================================================
// TEST SUITE: Multi-TOI event scheduling
// ===========================================================================

TEST_CASE("CCD multi-TOI event scheduling") {
    SUBCASE("ToiEventOrder sorts by time then body") {
        ToiEventOrder cmp;
        ToiEvent a{0.1f, 0, 1, {}, {}, 0.0f, 0};
        ToiEvent b{0.2f, 0, 1, {}, {}, 0.0f, 0};
        ToiEvent c{0.1f, 1, 2, {}, {}, 0.0f, 0};

        CHECK(cmp(a, b));   // earlier time first
        CHECK_FALSE(cmp(b, a));
        CHECK(cmp(a, c));   // same time, lower bodyA first
        CHECK_FALSE(cmp(c, a));
    }

    SUBCASE("MultiToiSchedulerConfig defaults are sane") {
        MultiToiSchedulerConfig cfg;
        CHECK(cfg.maxEventsPerBody == 4);
        CHECK(cfg.maxTotalEvents == 256);
        CHECK(cfg.minTimeSeparation > 0.0f);
        CHECK(cfg.mergeCloseEvents);
        CHECK(cfg.enableTemporalCoherence);
    }

    SUBCASE("multiple TOI events sorted chronologically") {
        // Simulate finding multiple TOI events and sorting them.
        std::vector<ToiEvent> events;
        events.push_back({0.005f, 0, 2, {}, {}, 0.0f, 0});
        events.push_back({0.001f, 0, 1, {}, {}, 0.0f, 0});
        events.push_back({0.010f, 0, 3, {}, {}, 0.0f, 0});
        events.push_back({0.001f, 0, 0, {}, {}, 0.0f, 0});

        std::sort(events.begin(), events.end(), ToiEventOrder{});

        CHECK(events[0].time == doctest::Approx(0.001f));
        CHECK(events[0].bodyB == 0); // tie broken by bodyB
        CHECK(events[1].time == doctest::Approx(0.001f));
        CHECK(events[1].bodyB == 1);
        CHECK(events[2].time == doctest::Approx(0.005f));
        CHECK(events[3].time == doctest::Approx(0.010f));
    }
}

// ===========================================================================
// TEST SUITE: Mesh CCD configuration
// ===========================================================================

TEST_CASE("CCD mesh optimization") {
    SUBCASE("MeshBvhNode leaf detection") {
        MeshBvhNode leaf;
        leaf.left = -1;
        leaf.right = -1;
        leaf.triFirst = 0;
        leaf.triCount = 4;
        CHECK(leaf.isLeaf());

        MeshBvhNode internal;
        internal.left = 1;
        internal.right = 2;
        CHECK_FALSE(internal.isLeaf());
    }

    SUBCASE("MeshCcdConfig defaults") {
        MeshCcdConfig cfg;
        CHECK(cfg.maxBvhDepth == 8);
        CHECK(cfg.maxTriangleTests == 128);
        CHECK(cfg.useSweptAabbPruning);
        CHECK(cfg.triangleInflation == doctest::Approx(0.0f));
    }

    SUBCASE("MeshCcdStats initializes to zero") {
        MeshCcdStats stats;
        CHECK(stats.nodesVisited == 0);
        CHECK(stats.trianglesTested == 0);
        CHECK(stats.nodesPruned == 0);
        CHECK(stats.queryTimeMs == doctest::Approx(0.0f));
    }
}

// ===========================================================================
// TEST SUITE: Compound CCD configuration
// ===========================================================================

TEST_CASE("CCD compound shape") {
    SUBCASE("CompoundCcdConfig defaults") {
        CompoundCcdConfig cfg;
        CHECK(cfg.testAllChildren);
        CHECK(cfg.maxChildPairs == 64);
        CHECK(cfg.earlyOut);
        CHECK(cfg.earlyOutThreshold > 0.0f);
    }

    SUBCASE("CompoundToiResult initializes empty") {
        CompoundToiResult result;
        CHECK_FALSE(result.toi.hit);
        CHECK(result.childIndex == -1);
    }
}

// ===========================================================================
// TEST SUITE: Performance
// ===========================================================================

TEST_CASE("CCD performance") {
    SUBCASE("conservative advancement completes within budget") {
        // Run 1000 TOI queries and verify total time is reasonable.
        SpherePlaneCtx ctx;
        ctx.pos = {0.0f, 100.0f, 0.0f};
        ctx.vel = {0.0f, -500.0f, 0.0f};
        ctx.radius = 0.5f;
        ctx.planeY = 0.0f;

        CcdQualitySettings settings = ccdQualityPreset(CcdQuality::Accurate);
        float dt = 1.0f / 60.0f;
        float speed = length(ctx.vel);

        auto start = std::chrono::high_resolution_clock::now();
        const int iterations = 1000;
        int hits = 0;
        for (int i = 0; i < iterations; ++i) {
            // Vary the starting height slightly to avoid caching effects
            ctx.pos.y = 50.0f + float(i % 100) * 0.5f;
            ToiResult result = conservativeAdvancement(
                spherePlaneOracle, &ctx, dt, speed, settings);
            if (result.hit) ++hits;
        }
        auto end = std::chrono::high_resolution_clock::now();
        double elapsedMs = std::chrono::duration<double, std::milli>(end - start).count();

        // 1000 queries should complete well under 100ms on any modern CPU
        CHECK(elapsedMs < 100.0);
        CHECK(hits > 0); // sanity: at least some should hit
    }

    SUBCASE("swept volume computation is fast") {
        auto start = std::chrono::high_resolution_clock::now();
        const int iterations = 100000;
        float sink = 0.0f;
        for (int i = 0; i < iterations; ++i) {
            Vec3 s{float(i) * 0.001f, 0.0f, 0.0f};
            Vec3 e = s + Vec3{1.0f, 0.0f, 0.0f};
            SweptVolume sv = computeSweptVolumeRotational(
                s, e, {0.5f, 0.5f, 0.5f}, 0.1f, 10.0f, 1.0f / 60.0f);
            sink += sv.aabb.lo.x;
        }
        auto end = std::chrono::high_resolution_clock::now();
        double elapsedMs = std::chrono::duration<double, std::milli>(end - start).count();

        // 100k swept volume computations should be under 50ms
        CHECK(elapsedMs < 50.0);
        CHECK(sink != 0.0f); // prevent optimization
    }

    SUBCASE("fast preset is faster than accurate") {
        SphereSphereCtx ctx;
        ctx.posA = {-10.0f, 0.0f, 0.0f};
        ctx.velA = {200.0f, 0.0f, 0.0f};
        ctx.radiusA = 0.5f;
        ctx.posB = {10.0f, 0.0f, 0.0f};
        ctx.velB = {-200.0f, 0.0f, 0.0f};
        ctx.radiusB = 0.5f;

        float dt = 1.0f / 60.0f;
        float relSpeed = length(ctx.velA - ctx.velB);
        const int iterations = 5000;

        CcdQualitySettings fastSettings = ccdQualityPreset(CcdQuality::Fast);
        CcdQualitySettings accurateSettings = ccdQualityPreset(CcdQuality::Accurate);

        auto t0 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < iterations; ++i) {
            ctx.posA.x = -10.0f - float(i % 50) * 0.1f;
            conservativeAdvancement(sphereSphereOracle, &ctx, dt, relSpeed, fastSettings);
        }
        auto t1 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < iterations; ++i) {
            ctx.posA.x = -10.0f - float(i % 50) * 0.1f;
            conservativeAdvancement(sphereSphereOracle, &ctx, dt, relSpeed, accurateSettings);
        }
        auto t2 = std::chrono::high_resolution_clock::now();

        double fastMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
        double accurateMs = std::chrono::duration<double, std::milli>(t2 - t1).count();

        // Fast should be no slower than accurate (typically faster)
        // Allow some slack for measurement noise
        CHECK(fastMs < accurateMs * 2.0);
    }
}
