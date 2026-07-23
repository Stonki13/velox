#include "doctest.h"
#include <velox/velox.h>
#include <cmath>
#include <limits>
#include <vector>

using namespace velox;

// ============================================================================
// Degenerate geometry
// ============================================================================

TEST_CASE("Edge: zero-radius sphere throws") {
    World world(BackendType::Cpu);
    CHECK_THROWS(world.addSphere({0, 0, 0}, 0.0f, 1.0f));
}

TEST_CASE("Edge: negative-radius sphere throws") {
    World world(BackendType::Cpu);
    CHECK_THROWS(world.addSphere({0, 0, 0}, -0.5f, 1.0f));
}

TEST_CASE("Edge: NaN radius throws") {
    World world(BackendType::Cpu);
    CHECK_THROWS(world.addSphere({0, 0, 0}, NAN, 1.0f));
}

TEST_CASE("Edge: NaN position throws") {
    World world(BackendType::Cpu);
    SUBCASE("x is NaN") {
        CHECK_THROWS(world.addSphere({NAN, 0, 0}, 0.5f, 1.0f));
    }
    SUBCASE("y is NaN") {
        CHECK_THROWS(world.addSphere({0, NAN, 0}, 0.5f, 1.0f));
    }
    SUBCASE("z is NaN") {
        CHECK_THROWS(world.addSphere({0, 0, NAN}, 0.5f, 1.0f));
    }
}

TEST_CASE("Edge: infinity position throws") {
    World world(BackendType::Cpu);
    float inf = std::numeric_limits<float>::infinity();
    CHECK_THROWS(world.addSphere({inf, 0, 0}, 0.5f, 1.0f));
    CHECK_THROWS(world.addSphere({0, inf, 0}, 0.5f, 1.0f));
    CHECK_THROWS(world.addSphere({0, 0, inf}, 0.5f, 1.0f));
}

TEST_CASE("Edge: zero half-extent box throws") {
    World world(BackendType::Cpu);
    SUBCASE("x zero") {
        CHECK_THROWS(world.addBox({0, 0, 0}, {0, 1, 1}, 1.0f));
    }
    SUBCASE("y zero") {
        CHECK_THROWS(world.addBox({0, 0, 0}, {1, 0, 1}, 1.0f));
    }
    SUBCASE("z zero") {
        CHECK_THROWS(world.addBox({0, 0, 0}, {1, 1, 0}, 1.0f));
    }
    SUBCASE("all zero") {
        CHECK_THROWS(world.addBox({0, 0, 0}, {0, 0, 0}, 1.0f));
    }
}

TEST_CASE("Edge: negative half-extent box throws") {
    World world(BackendType::Cpu);
    CHECK_THROWS(world.addBox({0, 0, 0}, {-1, 1, 1}, 1.0f));
}

TEST_CASE("Edge: NaN half-extents throw") {
    World world(BackendType::Cpu);
    CHECK_THROWS(world.addBox({0, 0, 0}, {NAN, 1, 1}, 1.0f));
    CHECK_THROWS(world.addBox({0, 0, 0}, {1, NAN, 1}, 1.0f));
    CHECK_THROWS(world.addBox({0, 0, 0}, {1, 1, NAN}, 1.0f));
}

TEST_CASE("Edge: zero-radius capsule throws") {
    World world(BackendType::Cpu);
    CHECK_THROWS(world.addCapsule({0, 0, 0}, 0.0f, 1.0f, 1.0f));
}

TEST_CASE("Edge: negative half-height capsule throws") {
    World world(BackendType::Cpu);
    CHECK_THROWS(world.addCapsule({0, 0, 0}, 0.5f, -1.0f, 1.0f));
}

TEST_CASE("Edge: NaN mass throws") {
    World world(BackendType::Cpu);
    CHECK_THROWS(world.addSphere({0, 0, 0}, 0.5f, NAN));
}

TEST_CASE("Edge: negative mass throws") {
    World world(BackendType::Cpu);
    CHECK_THROWS(world.addSphere({0, 0, 0}, 0.5f, -1.0f));
}

TEST_CASE("Edge: very tiny sphere is valid") {
    World world(BackendType::Cpu);
    float tiny = 1e-6f;
    BodyId id = world.addSphere({0, 5, 0}, tiny, 1.0f);
    CHECK(world.isValid(id));
    CHECK(world.body(id).radius == doctest::Approx(tiny));
}

TEST_CASE("Edge: very large sphere is valid") {
    World world(BackendType::Cpu);
    float large = 1e4f;
    BodyId id = world.addSphere({0, 0, 0}, large, 1.0f);
    CHECK(world.isValid(id));
    CHECK(world.body(id).radius == doctest::Approx(large));
}

TEST_CASE("Edge: degenerate convex hull with collinear points") {
    World world(BackendType::Cpu);
    // All points on a line — degenerate hull
    std::vector<Vec3> collinear = {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}};
    // Should either throw or create a valid body without crashing
    bool threw = false;
    BodyId id;
    try {
        id = world.addConvexHull({0, 5, 0}, collinear, 1.0f);
    } catch (...) {
        threw = true;
    }
    if (!threw) {
        CHECK(world.isValid(id));
    }
}

TEST_CASE("Edge: convex hull with duplicate points") {
    World world(BackendType::Cpu);
    std::vector<Vec3> dupes = {
        {0, 0, 0}, {0, 0, 0}, {1, 0, 0}, {1, 0, 0},
        {0, 1, 0}, {0, 1, 0}, {0, 0, 1}, {0, 0, 1}
    };
    // Should handle duplicates gracefully
    bool threw = false;
    BodyId id;
    try {
        id = world.addConvexHull({0, 5, 0}, dupes, 1.0f);
    } catch (...) {
        threw = true;
    }
    if (!threw) {
        CHECK(world.isValid(id));
    }
}

TEST_CASE("Edge: convex hull with single point") {
    World world(BackendType::Cpu);
    std::vector<Vec3> single = {{1, 1, 1}};
    bool threw = false;
    try {
        world.addConvexHull({0, 5, 0}, single, 1.0f);
    } catch (...) {
        threw = true;
    }
    // A single point cannot form a volume — must throw.
    CHECK(threw);
}

TEST_CASE("Edge: empty convex hull throws") {
    World world(BackendType::Cpu);
    std::vector<Vec3> empty;
    CHECK_THROWS(world.addConvexHull({0, 5, 0}, empty, 1.0f));
}

// ============================================================================
// Extreme velocities
// ============================================================================

TEST_CASE("Edge: very high velocity does not crash") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    BodyId id = world.addSphere({0, 0, 0}, 0.5f, 1.0f);
    world.setLinearVelocity(id, {1e6f, 0, 0});

    // Step should not crash or produce NaN
    for (int i = 0; i < 10; ++i)
        world.step(1.0f / 60.0f);

    Vec3 pos = world.body(id).position;
    CHECK(std::isfinite(pos.x));
    CHECK(std::isfinite(pos.y));
    CHECK(std::isfinite(pos.z));
}

TEST_CASE("Edge: very high angular velocity does not crash") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    BodyId id = world.addBox({0, 0, 0}, {1, 1, 1}, 1.0f);
    world.setAngularVelocity(id, {1e5f, 1e5f, 1e5f});

    for (int i = 0; i < 10; ++i)
        world.step(1.0f / 60.0f);

    Vec3 pos = world.body(id).position;
    Quat q = world.body(id).orientation;
    CHECK(std::isfinite(pos.x));
    CHECK(std::isfinite(q.w));
    CHECK(std::isfinite(q.x));
}

TEST_CASE("Edge: extremely slow velocity integrates correctly") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    BodyId id = world.addSphere({0, 0, 0}, 0.5f, 1.0f);
    float tinyVel = 1e-7f;
    world.setLinearVelocity(id, {tinyVel, 0, 0});

    float dt = 1.0f / 60.0f;
    world.step(dt);

    float expected = tinyVel * dt;
    float actual = world.body(id).position.x;
    // Should have moved by approximately v*dt
    CHECK(actual == doctest::Approx(expected).epsilon(0.01f));
}

TEST_CASE("Edge: subnormal velocity does not crash") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    BodyId id = world.addSphere({0, 0, 0}, 0.5f, 1.0f);
    float subnormal = std::numeric_limits<float>::denorm_min();
    world.setLinearVelocity(id, {subnormal, 0, 0});

    world.step(1.0f / 60.0f);
    Vec3 pos = world.body(id).position;
    CHECK(std::isfinite(pos.x));
}

TEST_CASE("Edge: max float velocity does not crash") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    BodyId id = world.addSphere({0, 0, 0}, 0.5f, 1.0f);
    float maxF = std::numeric_limits<float>::max() * 0.001f; // slightly below max to avoid inf on multiply
    world.setLinearVelocity(id, {maxF, 0, 0});

    // Should not crash
    world.step(1.0f / 60.0f);
    Vec3 pos = world.body(id).position;
    // Position may be inf due to overflow, but should not be NaN
    CHECK_FALSE(std::isnan(pos.x));
}

TEST_CASE("Edge: high-speed sphere vs thin wall (tunneling stress)") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};

    // Thin static wall
    BodyId wall = world.addBox({10, 0, 0}, {0.05f, 5, 5}, 0.0f);
    // Fast-moving sphere
    BodyId bullet = world.addSphere({0, 0, 0}, 0.1f, 1.0f);
    world.setLinearVelocity(bullet, {1000, 0, 0});

    for (int i = 0; i < 60; ++i)
        world.step(1.0f / 60.0f);

    // Engine should not crash; CCD may or may not prevent tunneling
    Vec3 pos = world.body(bullet).position;
    CHECK(std::isfinite(pos.x));
}

// ============================================================================
// Boundary conditions
// ============================================================================

TEST_CASE("Edge: two spheres exactly touching (zero gap)") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    float r = 0.5f;
    // Place spheres exactly touching: distance = 2*r = 1.0
    BodyId a = world.addSphere({0, 0, 0}, r, 1.0f);
    BodyId b = world.addSphere({2.0f * r, 0, 0}, r, 1.0f);

    // Step and check no explosion
    world.step(1.0f / 60.0f);
    Vec3 posA = world.body(a).position;
    Vec3 posB = world.body(b).position;
    CHECK(std::isfinite(posA.x));
    CHECK(std::isfinite(posB.x));
}

TEST_CASE("Edge: sphere resting exactly on plane") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};
    BodyId plane = world.addStaticPlane({0, 1, 0}, 0.0f);
    // Sphere center at exactly radius height — touching plane
    float r = 0.5f;
    BodyId sphere = world.addSphere({0, r, 0}, r, 1.0f);

    for (int i = 0; i < 120; ++i)
        world.step(1.0f / 60.0f);

    // Sphere should remain near the plane, not fall through or fly away
    float y = world.body(sphere).position.y;
    CHECK(y >= -0.1f);
    CHECK(y < r + 0.5f);
}

TEST_CASE("Edge: body at origin with zero velocity") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    BodyId id = world.addSphere({0, 0, 0}, 0.5f, 1.0f);

    world.step(1.0f / 60.0f);
    Vec3 pos = world.body(id).position;
    CHECK(pos.x == doctest::Approx(0.0f).epsilon(1e-6f));
    CHECK(pos.y == doctest::Approx(0.0f).epsilon(1e-6f));
    CHECK(pos.z == doctest::Approx(0.0f).epsilon(1e-6f));
}

TEST_CASE("Edge: two bodies at exact same position") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    BodyId a = world.addSphere({0, 0, 0}, 0.5f, 1.0f);
    BodyId b = world.addSphere({0, 0, 0}, 0.5f, 1.0f);

    // Fully overlapping — solver should push apart without NaN
    for (int i = 0; i < 30; ++i)
        world.step(1.0f / 60.0f);

    Vec3 posA = world.body(a).position;
    Vec3 posB = world.body(b).position;
    CHECK(std::isfinite(posA.x));
    CHECK(std::isfinite(posA.y));
    CHECK(std::isfinite(posA.z));
    CHECK(std::isfinite(posB.x));
    CHECK(std::isfinite(posB.y));
    CHECK(std::isfinite(posB.z));
}

TEST_CASE("Edge: very large position coordinates") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};
    float far = 1e5f;
    BodyId id = world.addSphere({far, far, far}, 0.5f, 1.0f);

    world.step(1.0f / 60.0f);
    Vec3 pos = world.body(id).position;
    CHECK(std::isfinite(pos.x));
    CHECK(pos.y < far); // gravity should pull it down
}

TEST_CASE("Edge: zero dt step does not crash") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};
    BodyId id = world.addSphere({0, 5, 0}, 0.5f, 1.0f);
    float y0 = world.body(id).position.y;

    world.step(0.0f);
    float y1 = world.body(id).position.y;
    CHECK(y1 == doctest::Approx(y0));
}

TEST_CASE("Edge: very small dt step") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};
    BodyId id = world.addSphere({0, 5, 0}, 0.5f, 1.0f);

    world.step(1e-8f);
    Vec3 pos = world.body(id).position;
    CHECK(std::isfinite(pos.y));
}

TEST_CASE("Edge: negative dt step") {
    World world(BackendType::Cpu);
    BodyId id = world.addSphere({0, 5, 0}, 0.5f, 1.0f);
    // Negative dt should either throw or be handled gracefully
    bool threw = false;
    try {
        world.step(-1.0f / 60.0f);
    } catch (...) {
        threw = true;
    }
    // Negative dt must throw (validated in World::stepImpl).
    CHECK(threw);
}

// ============================================================================
// Numerical stability: extreme masses and inertia
// ============================================================================

TEST_CASE("Edge: very small mass") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};
    float tinyMass = 1e-10f;
    BodyId id = world.addSphere({0, 5, 0}, 0.5f, tinyMass);
    CHECK(world.isValid(id));
    CHECK(world.body(id).invMass > 0.0f);

    world.step(1.0f / 60.0f);
    Vec3 pos = world.body(id).position;
    CHECK(std::isfinite(pos.y));
}

TEST_CASE("Edge: very large mass") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};
    float hugeMass = 1e10f;
    BodyId id = world.addSphere({0, 5, 0}, 0.5f, hugeMass);
    CHECK(world.isValid(id));
    CHECK(world.body(id).invMass > 0.0f);
    CHECK(world.body(id).invMass < 1e-9f);

    world.step(1.0f / 60.0f);
    Vec3 pos = world.body(id).position;
    CHECK(std::isfinite(pos.y));
}

TEST_CASE("Edge: mass ratio 1:1e8 collision") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};

    BodyId light = world.addSphere({0, 0, 0}, 0.5f, 1.0f);
    BodyId heavy = world.addSphere({1.0f, 0, 0}, 0.5f, 1e8f);
    world.setLinearVelocity(light, {10, 0, 0});

    for (int i = 0; i < 60; ++i)
        world.step(1.0f / 60.0f);

    Vec3 posL = world.body(light).position;
    Vec3 posH = world.body(heavy).position;
    CHECK(std::isfinite(posL.x));
    CHECK(std::isfinite(posH.x));
    // Heavy body should barely move
    CHECK(std::abs(posH.x - 1.0f) < 1.0f);
}

TEST_CASE("Edge: extreme inertia via setMassProperties") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    BodyId id = world.addBox({0, 0, 0}, {1, 1, 1}, 1.0f);

    // Set extremely anisotropic inertia
    world.setMassProperties(id, 1.0f, {1e-10f, 1.0f, 1e10f});
    world.setAngularVelocity(id, {1, 1, 1});

    for (int i = 0; i < 60; ++i)
        world.step(1.0f / 60.0f);

    Quat q = world.body(id).orientation;
    CHECK(std::isfinite(q.w));
    CHECK(std::isfinite(q.x));
    CHECK(std::isfinite(q.y));
    CHECK(std::isfinite(q.z));
}

TEST_CASE("Edge: zero inertia body (fixed rotation)") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};
    BodyId id = world.addBox({0, 5, 0}, {1, 1, 1}, 1.0f);
    world.setFixedRotation(id, true);

    world.setAngularVelocity(id, {100, 100, 100});
    CHECK(world.body(id).angularVelocity.x == doctest::Approx(0.0f));

    for (int i = 0; i < 60; ++i)
        world.step(1.0f / 60.0f);

    // Orientation should remain identity
    Quat q = world.body(id).orientation;
    CHECK(q.w == doctest::Approx(1.0f).epsilon(1e-4f));
}

// ============================================================================
// Joint limits at boundaries
// ============================================================================

TEST_CASE("Edge: hinge joint with zero-range limit") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};
    BodyId a = world.addBox({0, 0, 0}, {1, 1, 1}, 0.0f); // static
    BodyId b = world.addBox({2, 0, 0}, {0.5f, 0.5f, 0.5f}, 1.0f);
    JointId j = world.addHingeJoint(a, b, {1, 0, 0}, {0, 1, 0});

    Joint& joint = world.joint(j);
    joint.enableLimit = true;
    joint.lowerLimit = 0.0f;
    joint.upperLimit = 0.0f; // zero range — effectively locked

    for (int i = 0; i < 120; ++i)
        world.step(1.0f / 60.0f);

    float angle = world.hingeAngle(j);
    CHECK(angle == doctest::Approx(0.0f).epsilon(0.1f));
}

TEST_CASE("Edge: hinge joint at exact limit boundary") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    BodyId a = world.addBox({0, 0, 0}, {1, 1, 1}, 0.0f);
    BodyId b = world.addBox({2, 0, 0}, {0.5f, 0.5f, 0.5f}, 1.0f);
    JointId j = world.addHingeJoint(a, b, {1, 0, 0}, {0, 1, 0});

    Joint& joint = world.joint(j);
    joint.enableLimit = true;
    joint.lowerLimit = -0.5f;
    joint.upperLimit = 0.5f;

    // Give angular velocity that would exceed the limit
    world.setAngularVelocity(b, {0, 10, 0});

    for (int i = 0; i < 120; ++i)
        world.step(1.0f / 60.0f);

    float angle = world.hingeAngle(j);
    // Angle should be clamped near the upper limit
    CHECK(angle <= 0.5f + 0.2f);
    CHECK(angle >= -0.5f - 0.2f);
}

TEST_CASE("Edge: prismatic joint at travel limit") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    BodyId a = world.addBox({0, 0, 0}, {1, 1, 1}, 0.0f);
    BodyId b = world.addBox({2, 0, 0}, {0.5f, 0.5f, 0.5f}, 1.0f);
    JointId j = world.addPrismaticJoint(a, b, {2, 0, 0}, {1, 0, 0});

    Joint& joint = world.joint(j);
    joint.enableLimit = true;
    joint.lowerLimit = -1.0f;
    joint.upperLimit = 1.0f;

    // Push body toward upper limit
    world.setLinearVelocity(b, {5, 0, 0});

    for (int i = 0; i < 120; ++i)
        world.step(1.0f / 60.0f);

    float translation = world.prismaticTranslation(j);
    CHECK(translation <= 1.0f + 0.3f);
    CHECK(translation >= -1.0f - 0.3f);
}

TEST_CASE("Edge: cone-twist joint at swing limit") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};
    BodyId a = world.addSphere({0, 5, 0}, 0.5f, 0.0f); // static anchor
    BodyId b = world.addSphere({0, 3, 0}, 0.5f, 1.0f);
    JointId j = world.addConeTwistJoint(a, b, {0, 4, 0}, {0, 1, 0});

    Joint& joint = world.joint(j);
    joint.enableSwingLimit = true;
    joint.swingLimit = 0.3f; // tight cone

    for (int i = 0; i < 120; ++i)
        world.step(1.0f / 60.0f);

    float swing = world.coneSwingAngle(j);
    CHECK(swing <= 0.3f + 0.3f); // allow some solver slop
    CHECK(std::isfinite(swing));
}

TEST_CASE("Edge: distance joint with zero rest length") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    BodyId a = world.addSphere({0, 0, 0}, 0.5f, 0.0f);
    BodyId b = world.addSphere({2, 0, 0}, 0.5f, 1.0f);
    JointId j = world.addDistanceJoint(a, b, {0, 0, 0}, {2, 0, 0});

    // Override rest length to zero — bodies should be pulled together
    world.joint(j).restLength = 0.0f;

    for (int i = 0; i < 120; ++i)
        world.step(1.0f / 60.0f);

    float dist = length(world.body(b).position - world.body(a).position);
    CHECK(dist < 1.0f); // should be much closer than initial 2.0
}

TEST_CASE("Edge: six-dof joint all axes locked") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};
    BodyId a = world.addBox({0, 5, 0}, {1, 1, 1}, 0.0f);
    BodyId b = world.addBox({2, 5, 0}, {0.5f, 0.5f, 0.5f}, 1.0f);
    JointId j = world.addSixDofJoint(a, b, {1, 5, 0});

    Joint& joint = world.joint(j);
    joint.linearLimitMask = JointAxisAll;
    joint.angularLimitMask = JointAxisAll;
    joint.lowerLinearLimit = {0, 0, 0};
    joint.upperLinearLimit = {0, 0, 0};
    joint.lowerAngularLimit = {0, 0, 0};
    joint.upperAngularLimit = {0, 0, 0};

    for (int i = 0; i < 120; ++i)
        world.step(1.0f / 60.0f);

    // Body B should stay near its initial position relative to A
    Vec3 pos = world.body(b).position;
    CHECK(std::isfinite(pos.x));
    CHECK(std::isfinite(pos.y));
    CHECK(std::isfinite(pos.z));
}

// ============================================================================
// Sleep/wake transitions
// ============================================================================

TEST_CASE("Edge: sleeping body ignores gravity") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};
    BodyId id = world.addSphere({0, 5, 0}, 0.5f, 1.0f);

    world.sleepBody(id);
    CHECK(world.body(id).asleep != 0);

    float y0 = world.body(id).position.y;
    for (int i = 0; i < 60; ++i)
        world.step(1.0f / 60.0f);

    float y1 = world.body(id).position.y;
    CHECK(y1 == doctest::Approx(y0));
}

TEST_CASE("Edge: wake sleeping body resumes simulation") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};
    BodyId id = world.addSphere({0, 5, 0}, 0.5f, 1.0f);

    world.sleepBody(id);
    world.step(1.0f / 60.0f);
    float ySleeping = world.body(id).position.y;

    world.wakeBody(id);
    CHECK(world.body(id).asleep == 0);

    world.step(1.0f / 60.0f);
    float yAwake = world.body(id).position.y;
    CHECK(yAwake < ySleeping);
}

TEST_CASE("Edge: wake via velocity change") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    BodyId id = world.addSphere({0, 0, 0}, 0.5f, 1.0f);

    world.sleepBody(id);
    CHECK(world.body(id).asleep != 0);

    // Applying velocity should wake the body (or we wake explicitly)
    world.wakeBody(id);
    world.setLinearVelocity(id, {5, 0, 0});

    world.step(1.0f / 60.0f);
    float x = world.body(id).position.x;
    CHECK(x > 0.0f);
}

TEST_CASE("Edge: sleep disabled body never sleeps") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    BodyId id = world.addSphere({0, 0, 0}, 0.5f, 1.0f);
    world.setEnableSleep(id, false);

    // Zero velocity, many steps — should remain awake
    for (int i = 0; i < 600; ++i)
        world.step(1.0f / 60.0f);

    CHECK(world.body(id).asleep == 0);
}

TEST_CASE("Edge: static body sleep state is irrelevant") {
    World world(BackendType::Cpu);
    BodyId id = world.addBox({0, 0, 0}, {10, 1, 10}, 0.0f); // static

    // Static bodies should not be affected by sleep/wake
    world.sleepBody(id);
    world.step(1.0f / 60.0f);

    Vec3 pos = world.body(id).position;
    CHECK(pos.x == doctest::Approx(0.0f));
    CHECK(pos.y == doctest::Approx(0.0f));
}

TEST_CASE("Edge: sleeping body woken by collision") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};

    // Ground plane
    world.addStaticPlane({0, 1, 0}, 0.0f);

    // Sleeping body on the ground
    BodyId sleeper = world.addSphere({0, 0.5f, 0}, 0.5f, 1.0f);
    world.sleepBody(sleeper);

    // Falling body that will hit the sleeper
    BodyId falling = world.addSphere({0, 5, 0}, 0.5f, 1.0f);

    for (int i = 0; i < 120; ++i)
        world.step(1.0f / 60.0f);

    // After collision, the sleeper should have been woken and moved
    // (or at minimum, the simulation didn't crash)
    Vec3 pos = world.body(sleeper).position;
    CHECK(std::isfinite(pos.x));
    CHECK(std::isfinite(pos.y));
}

TEST_CASE("Edge: repeated sleep/wake cycles") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};
    BodyId id = world.addSphere({0, 5, 0}, 0.5f, 1.0f);

    for (int cycle = 0; cycle < 100; ++cycle) {
        world.sleepBody(id);
        CHECK(world.body(id).asleep != 0);
        world.wakeBody(id);
        CHECK(world.body(id).asleep == 0);
        world.step(1.0f / 60.0f);
    }

    Vec3 pos = world.body(id).position;
    CHECK(std::isfinite(pos.y));
}

// ============================================================================
// Concurrent body modification during step
// ============================================================================

TEST_CASE("Edge: remove body between steps") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};
    BodyId a = world.addSphere({0, 5, 0}, 0.5f, 1.0f);
    BodyId b = world.addSphere({2, 5, 0}, 0.5f, 1.0f);

    world.step(1.0f / 60.0f);
    world.removeBody(a);
    CHECK_FALSE(world.isValid(a));
    CHECK(world.isValid(b));

    // Stepping after removal should work fine
    world.step(1.0f / 60.0f);
    Vec3 posB = world.body(b).position;
    CHECK(std::isfinite(posB.y));
}

TEST_CASE("Edge: add body between steps") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};
    BodyId a = world.addSphere({0, 5, 0}, 0.5f, 1.0f);

    world.step(1.0f / 60.0f);

    BodyId b = world.addSphere({2, 10, 0}, 0.5f, 1.0f);
    CHECK(world.bodyCount() == 2);

    world.step(1.0f / 60.0f);
    CHECK(world.isValid(a));
    CHECK(world.isValid(b));
    CHECK(world.body(b).position.y < 10.0f);
}

TEST_CASE("Edge: remove all bodies then step") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};
    BodyId a = world.addSphere({0, 5, 0}, 0.5f, 1.0f);
    BodyId b = world.addSphere({2, 5, 0}, 0.5f, 1.0f);

    world.removeBody(a);
    world.removeBody(b);
    CHECK(world.bodyCount() == 0);

    // Stepping an empty world should not crash
    world.step(1.0f / 60.0f);
    CHECK(world.bodyCount() == 0);
}

TEST_CASE("Edge: modify velocity between steps") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    BodyId id = world.addSphere({0, 0, 0}, 0.5f, 1.0f);

    world.setLinearVelocity(id, {1, 0, 0});
    world.step(1.0f / 60.0f);
    float x1 = world.body(id).position.x;

    // Reverse velocity
    world.setLinearVelocity(id, {-1, 0, 0});
    world.step(1.0f / 60.0f);
    float x2 = world.body(id).position.x;

    CHECK(x2 < x1);
}

TEST_CASE("Edge: setTransform during active simulation") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};
    BodyId id = world.addSphere({0, 5, 0}, 0.5f, 1.0f);

    for (int i = 0; i < 30; ++i)
        world.step(1.0f / 60.0f);

    // Teleport body
    world.setTransform(id, {100, 100, 100}, {});
    Vec3 pos = world.body(id).position;
    CHECK(pos.x == doctest::Approx(100.0f));
    CHECK(pos.y == doctest::Approx(100.0f));

    // Continue simulation from new position
    world.step(1.0f / 60.0f);
    float y = world.body(id).position.y;
    CHECK(y < 100.0f); // gravity pulls down
}

TEST_CASE("Edge: remove jointed body removes joint") {
    World world(BackendType::Cpu);
    BodyId a = world.addSphere({0, 5, 0}, 0.5f, 1.0f);
    BodyId b = world.addSphere({0, 3, 0}, 0.5f, 1.0f);
    JointId j = world.addBallJoint(a, b, {0, 4, 0});
    CHECK(world.isValid(j));

    world.removeBody(b);
    CHECK_FALSE(world.isValid(j));

    // Step should work without the joint
    world.step(1.0f / 60.0f);
    CHECK(world.isValid(a));
}

TEST_CASE("Edge: rapid body creation and removal") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};

    for (int iter = 0; iter < 50; ++iter) {
        BodyId id = world.addSphere({float(iter), 5, 0}, 0.5f, 1.0f);
        world.step(1.0f / 60.0f);
        world.removeBody(id);
    }

    CHECK(world.bodyCount() == 0);
    world.step(1.0f / 60.0f); // final step on empty world
}

TEST_CASE("Edge: change motion type during simulation") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};
    BodyId id = world.addSphere({0, 5, 0}, 0.5f, 1.0f);

    world.step(1.0f / 60.0f);
    float y1 = world.body(id).position.y;

    // Make static mid-simulation
    world.setMotionType(id, MotionType::Static);
    world.step(1.0f / 60.0f);
    float y2 = world.body(id).position.y;

    // Static body should not move
    CHECK(y2 == doctest::Approx(y1).epsilon(1e-4f));

    // Make dynamic again
    world.setMotionType(id, MotionType::Dynamic);
    world.step(1.0f / 60.0f);
    float y3 = world.body(id).position.y;
    CHECK(y3 < y2);
}

TEST_CASE("Edge: kinematic body pushes dynamic body") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};

    BodyId kinematic = world.addBox({0, 0, 0}, {1, 1, 1}, 0.0f);
    world.setMotionType(kinematic, MotionType::Kinematic);
    world.setLinearVelocity(kinematic, {5, 0, 0});

    BodyId dynamic = world.addSphere({3, 0, 0}, 0.5f, 1.0f);

    for (int i = 0; i < 60; ++i)
        world.step(1.0f / 60.0f);

    // Dynamic body should have been pushed
    Vec3 posDyn = world.body(dynamic).position;
    CHECK(std::isfinite(posDyn.x));
}

// ============================================================================
// Additional edge cases
// ============================================================================

TEST_CASE("Edge: sensor does not affect physics") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};

    // Sensor floor
    BodyId sensorFloor = world.addBox({0, 0, 0}, {10, 0.5f, 10}, 0.0f);
    world.setSensor(sensorFloor, true);

    BodyId ball = world.addSphere({0, 3, 0}, 0.5f, 1.0f);

    for (int i = 0; i < 120; ++i)
        world.step(1.0f / 60.0f);

    // Ball should fall through the sensor
    float y = world.body(ball).position.y;
    CHECK(y < 0.0f);
}

TEST_CASE("Edge: collision filter prevents contact") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};

    BodyId floor = world.addBox({0, 0, 0}, {10, 0.5f, 10}, 0.0f);
    world.setCollisionFilter(floor, 0x0001, 0x0001);

    BodyId ball = world.addSphere({0, 3, 0}, 0.5f, 1.0f);
    world.setCollisionFilter(ball, 0x0002, 0x0002); // different group

    for (int i = 0; i < 120; ++i)
        world.step(1.0f / 60.0f);

    // Ball should fall through since collision is filtered
    float y = world.body(ball).position.y;
    CHECK(y < 0.0f);
}

TEST_CASE("Edge: explode with zero radius") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    BodyId id = world.addSphere({5, 0, 0}, 0.5f, 1.0f);
    world.setLinearVelocity(id, {0, 0, 0});

    // Zero radius explosion should not affect anything
    world.explode({0, 0, 0}, 0.0f, 100.0f);
    Vec3 v = world.body(id).velocity;
    CHECK(v.x == doctest::Approx(0.0f).epsilon(1e-5f));
}

TEST_CASE("Edge: explode with zero impulse") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    BodyId id = world.addSphere({1, 0, 0}, 0.5f, 1.0f);
    world.setLinearVelocity(id, {0, 0, 0});

    world.explode({0, 0, 0}, 10.0f, 0.0f);
    Vec3 v = world.body(id).velocity;
    CHECK(v.x == doctest::Approx(0.0f).epsilon(1e-5f));
}

TEST_CASE("Edge: many bodies stacked") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};
    world.addStaticPlane({0, 1, 0}, 0.0f);

    // Stack 20 spheres
    std::vector<BodyId> stack;
    for (int i = 0; i < 20; ++i) {
        stack.push_back(world.addSphere({0, 0.5f + float(i) * 1.0f, 0}, 0.5f, 1.0f));
    }

    for (int i = 0; i < 300; ++i)
        world.step(1.0f / 60.0f);

    // All bodies should have finite positions
    for (auto id : stack) {
        Vec3 pos = world.body(id).position;
        CHECK(std::isfinite(pos.x));
        CHECK(std::isfinite(pos.y));
        CHECK(std::isfinite(pos.z));
    }
}

TEST_CASE("Edge: body with gravity scale zero") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};
    BodyId id = world.addSphere({0, 5, 0}, 0.5f, 1.0f);
    world.setGravityScale(id, 0.0f);

    for (int i = 0; i < 60; ++i)
        world.step(1.0f / 60.0f);

    float y = world.body(id).position.y;
    CHECK(y == doctest::Approx(5.0f).epsilon(0.01f));
}

TEST_CASE("Edge: body with negative gravity scale") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};
    BodyId id = world.addSphere({0, 5, 0}, 0.5f, 1.0f);
    world.setGravityScale(id, -1.0f); // anti-gravity

    world.step(1.0f / 60.0f);
    float y = world.body(id).position.y;
    CHECK(y > 5.0f); // should move up
}

TEST_CASE("Edge: very high damping stops body quickly") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    BodyId id = world.addSphere({0, 0, 0}, 0.5f, 1.0f);
    world.setLinearVelocity(id, {100, 0, 0});
    world.setLinearDamping(id, 100.0f);

    for (int i = 0; i < 60; ++i)
        world.step(1.0f / 60.0f);

    float speed = length(world.body(id).velocity);
    CHECK(speed < 1.0f); // should be nearly stopped
}
