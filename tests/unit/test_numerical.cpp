#include "doctest.h"
#include <velox/velox.h>
#include <cmath>
#include <limits>
#include <vector>

using namespace velox;

// ============================================================================
// Floating point precision
// ============================================================================

TEST_CASE("Numerical: free-fall position matches analytic solution") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};
    float y0 = 100.0f;
    BodyId id = world.addSphere({0, y0, 0}, 0.5f, 1.0f);

    float dt = 1.0f / 60.0f;
    int steps = 60; // 1 second
    for (int i = 0; i < steps; ++i)
        world.step(dt);

    float t = steps * dt;
    float expected = y0 - 0.5f * 10.0f * t * t; // y0 - 0.5*g*t^2
    float actual = world.body(id).position.y;

    // Allow 5% error due to discrete integration
    CHECK(actual == doctest::Approx(expected).epsilon(0.05f));
}

TEST_CASE("Numerical: velocity after free-fall matches analytic") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};
    BodyId id = world.addSphere({0, 100, 0}, 0.5f, 1.0f);

    float dt = 1.0f / 60.0f;
    int steps = 60;
    for (int i = 0; i < steps; ++i)
        world.step(dt);

    float t = steps * dt;
    float expectedVy = -10.0f * t; // v = -g*t
    float actualVy = world.body(id).velocity.y;

    CHECK(actualVy == doctest::Approx(expectedVy).epsilon(0.05f));
}

TEST_CASE("Numerical: energy conservation in zero-g with no damping") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    BodyId id = world.addSphere({0, 0, 0}, 0.5f, 2.0f);
    world.setLinearVelocity(id, {3, 4, 0}); // speed = 5

    float mass = 2.0f;
    float speed0 = length(world.body(id).velocity);
    float ke0 = 0.5f * mass * speed0 * speed0;

    for (int i = 0; i < 120; ++i)
        world.step(1.0f / 60.0f);

    float speed1 = length(world.body(id).velocity);
    float ke1 = 0.5f * mass * speed1 * speed1;

    // In zero-g with no damping, kinetic energy should be conserved
    CHECK(ke1 == doctest::Approx(ke0).epsilon(0.01f));
}

TEST_CASE("Numerical: small dt produces proportionally small displacement") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};

    SUBCASE("dt = 1/60") {
        BodyId id = world.addSphere({0, 0, 0}, 0.5f, 1.0f);
        world.setLinearVelocity(id, {1, 0, 0});
        world.step(1.0f / 60.0f);
        float dx = world.body(id).position.x;
        CHECK(dx == doctest::Approx(1.0f / 60.0f).epsilon(0.01f));
    }

    SUBCASE("dt = 1/600") {
        BodyId id = world.addSphere({0, 0, 0}, 0.5f, 1.0f);
        world.setLinearVelocity(id, {1, 0, 0});
        world.step(1.0f / 600.0f);
        float dx = world.body(id).position.x;
        CHECK(dx == doctest::Approx(1.0f / 600.0f).epsilon(0.01f));
    }
}

TEST_CASE("Numerical: position does not drift in zero-g zero-velocity") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    BodyId id = world.addSphere({1.5f, 2.5f, 3.5f}, 0.5f, 1.0f);

    for (int i = 0; i < 1000; ++i)
        world.step(1.0f / 60.0f);

    Vec3 pos = world.body(id).position;
    CHECK(pos.x == doctest::Approx(1.5f).epsilon(1e-5f));
    CHECK(pos.y == doctest::Approx(2.5f).epsilon(1e-5f));
    CHECK(pos.z == doctest::Approx(3.5f).epsilon(1e-5f));
}

TEST_CASE("Numerical: quaternion stays normalized during rotation") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    BodyId id = world.addBox({0, 0, 0}, {1, 0.5f, 0.25f}, 1.0f);
    world.setAngularVelocity(id, {1, 2, 3});

    for (int i = 0; i < 600; ++i)
        world.step(1.0f / 60.0f);

    Quat q = world.body(id).orientation;
    float qLen = sqrtf(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    CHECK(qLen == doctest::Approx(1.0f).epsilon(1e-3f));
}

TEST_CASE("Numerical: very small force accumulates over time") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    BodyId id = world.addSphere({0, 0, 0}, 0.5f, 1.0f);

    float tinyForce = 1e-4f;
    float dt = 1.0f / 60.0f;
    int steps = 600; // 10 seconds

    for (int i = 0; i < steps; ++i) {
        world.addForce(id, {tinyForce, 0, 0});
        world.step(dt);
    }

    // v = F/m * t = 1e-4 * 10 = 1e-3
    float vx = world.body(id).velocity.x;
    float expected = tinyForce * steps * dt;
    CHECK(vx == doctest::Approx(expected).epsilon(0.1f));
}

TEST_CASE("Numerical: impulse changes velocity exactly") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    float mass = 2.0f;
    BodyId id = world.addSphere({0, 0, 0}, 0.5f, mass);

    Vec3 impulse = {4, -6, 8};
    world.addLinearImpulse(id, impulse);
    world.step(1.0f / 60.0f);

    // dv = impulse / mass
    Vec3 v = world.body(id).velocity;
    CHECK(v.x == doctest::Approx(impulse.x / mass).epsilon(0.01f));
    CHECK(v.y == doctest::Approx(impulse.y / mass).epsilon(0.01f));
    CHECK(v.z == doctest::Approx(impulse.z / mass).epsilon(0.01f));
}

TEST_CASE("Numerical: large coordinates do not lose relative precision") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};

    // Two bodies far from origin but close to each other
    float base = 1e4f;
    BodyId a = world.addSphere({base, 0, 0}, 0.5f, 1.0f);
    BodyId b = world.addSphere({base + 2.0f, 0, 0}, 0.5f, 1.0f);

    world.setLinearVelocity(a, {1, 0, 0});
    world.setLinearVelocity(b, {-1, 0, 0});

    float dt = 1.0f / 60.0f;
    for (int i = 0; i < 30; ++i)
        world.step(dt);

    // They should have moved toward each other
    float dist = length(world.body(b).position - world.body(a).position);
    CHECK(dist < 2.0f); // started at 2.0, approaching
    CHECK(std::isfinite(dist));
}

// ============================================================================
// Accumulated error over many steps
// ============================================================================

TEST_CASE("Numerical: 10000 steps of free-fall stays finite") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};
    BodyId id = world.addSphere({0, 0, 0}, 0.5f, 1.0f);

    float dt = 1.0f / 60.0f;
    for (int i = 0; i < 10000; ++i)
        world.step(dt);

    Vec3 pos = world.body(id).position;
    Vec3 vel = world.body(id).velocity;
    CHECK(std::isfinite(pos.y));
    CHECK(std::isfinite(vel.y));

    // Analytic: y = -0.5*g*t^2 where t = 10000/60 ≈ 166.7s
    float t = 10000.0f * dt;
    float expectedY = -0.5f * 10.0f * t * t;
    // With discrete integration the error grows, but should be within 10%
    CHECK(pos.y == doctest::Approx(expectedY).epsilon(0.10f));
}

TEST_CASE("Numerical: bouncing ball energy does not grow") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};
    world.addStaticPlane({0, 1, 0}, 0.0f);

    float r = 0.5f;
    float dropHeight = 5.0f;
    BodyId ball = world.addSphere({0, dropHeight, 0}, r, 1.0f);
    // Set restitution to 1.0 for perfectly elastic bounce
    world.body(ball).restitution = 1.0f;

    float initialPE = 10.0f * (dropHeight - r); // m*g*h (m=1)

    // Track max height over many bounces
    float maxHeight = 0.0f;
    for (int i = 0; i < 600; ++i) {
        world.step(1.0f / 60.0f);
        float y = world.body(ball).position.y;
        if (y > maxHeight) maxHeight = y;
    }

    // Max height should not exceed initial drop height (energy should not grow)
    // Allow small numerical overshoot
    CHECK(maxHeight <= dropHeight + 0.5f);
}

TEST_CASE("Numerical: orbiting body maintains distance (centripetal)") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};

    // Central heavy body
    BodyId center = world.addSphere({0, 0, 0}, 1.0f, 0.0f); // static

    // Orbiting body with tangential velocity
    float orbitR = 5.0f;
    BodyId orbiter = world.addSphere({orbitR, 0, 0}, 0.25f, 1.0f);

    // Attach with distance joint to simulate orbit constraint
    world.addDistanceJoint(center, orbiter, {0, 0, 0}, {orbitR, 0, 0});
    world.setLinearVelocity(orbiter, {0, 0, 5}); // tangential

    float minDist = 1e10f, maxDist = 0.0f;
    for (int i = 0; i < 600; ++i) {
        world.step(1.0f / 60.0f);
        float d = length(world.body(orbiter).position - world.body(center).position);
        if (d < minDist) minDist = d;
        if (d > maxDist) maxDist = d;
    }

    // Distance should stay near orbitR
    CHECK(minDist == doctest::Approx(orbitR).epsilon(0.3f));
    CHECK(maxDist == doctest::Approx(orbitR).epsilon(0.3f));
}

TEST_CASE("Numerical: long-running spring does not explode") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    BodyId anchor = world.addSphere({0, 0, 0}, 0.5f, 0.0f);
    BodyId mass = world.addSphere({3, 0, 0}, 0.5f, 1.0f);
    world.addSpringJoint(anchor, mass, {0, 0, 0}, {3, 0, 0}, 2.0f, 0.0f); // undamped

    world.setLinearVelocity(mass, {0, 5, 0}); // perturb

    float maxDist = 0.0f;
    for (int i = 0; i < 3600; ++i) { // 60 seconds
        world.step(1.0f / 60.0f);
        float d = length(world.body(mass).position);
        if (d > maxDist) maxDist = d;
    }

    // Undamped spring should oscillate but not grow unboundedly
    CHECK(maxDist < 20.0f);
    CHECK(std::isfinite(maxDist));
}

TEST_CASE("Numerical: resting contact does not accumulate drift") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};
    world.addStaticPlane({0, 1, 0}, 0.0f);

    float r = 0.5f;
    BodyId ball = world.addSphere({0, r, 0}, r, 1.0f);

    // Let it settle
    for (int i = 0; i < 300; ++i)
        world.step(1.0f / 60.0f);

    float ySettled = world.body(ball).position.y;

    // Run for a long time
    for (int i = 0; i < 3000; ++i)
        world.step(1.0f / 60.0f);

    float yFinal = world.body(ball).position.y;

    // Should not drift significantly
    CHECK(yFinal == doctest::Approx(ySettled).epsilon(0.1f));
}

TEST_CASE("Numerical: many substeps vs few substeps converge") {
    float dt = 1.0f / 60.0f;
    float y0 = 10.0f;

    auto runSim = [&](int substeps) -> float {
        World world(BackendType::Cpu);
        world.gravity = {0, -10, 0};
        world.substeps = substeps;
        world.addStaticPlane({0, 1, 0}, 0.0f);
        BodyId ball = world.addSphere({0, y0, 0}, 0.5f, 1.0f);

        for (int i = 0; i < 120; ++i)
            world.step(dt);

        return world.body(ball).position.y;
    };

    float y1 = runSim(1);
    float y4 = runSim(4);
    float y8 = runSim(8);

    // All should produce finite results
    CHECK(std::isfinite(y1));
    CHECK(std::isfinite(y4));
    CHECK(std::isfinite(y8));

    // More substeps should converge — y4 and y8 should be closer to each other
    // than y1 and y4 (or at least all in the same ballpark)
    CHECK(std::abs(y4 - y8) <= std::abs(y1 - y4) + 0.5f);
}

// ============================================================================
// Determinism
// ============================================================================

TEST_CASE("Numerical: same initial state produces same result") {
    auto runSim = []() -> Vec3 {
        World world(BackendType::Cpu);
        world.gravity = {0, -10, 0};
        world.addStaticPlane({0, 1, 0}, 0.0f);
        BodyId a = world.addSphere({0, 5, 0}, 0.5f, 1.0f);
        BodyId b = world.addBox({1, 8, 0}, {0.5f, 0.5f, 0.5f}, 2.0f);
        world.setLinearVelocity(a, {2, 0, 1});
        world.setAngularVelocity(b, {0, 3, 0});

        for (int i = 0; i < 120; ++i)
            world.step(1.0f / 60.0f);

        return world.body(a).position;
    };

    Vec3 pos1 = runSim();
    Vec3 pos2 = runSim();

    CHECK(pos1.x == doctest::Approx(pos2.x).epsilon(1e-5f));
    CHECK(pos1.y == doctest::Approx(pos2.y).epsilon(1e-5f));
    CHECK(pos1.z == doctest::Approx(pos2.z).epsilon(1e-5f));
}

TEST_CASE("Numerical: body creation order does not affect individual body physics") {
    // Two independent bodies in zero-g should behave the same regardless
    // of which was created first
    auto runAB = []() -> std::pair<Vec3, Vec3> {
        World world(BackendType::Cpu);
        world.gravity = {0, 0, 0};
        BodyId a = world.addSphere({0, 0, 0}, 0.5f, 1.0f);
        BodyId b = world.addSphere({10, 0, 0}, 0.5f, 2.0f);
        world.setLinearVelocity(a, {1, 0, 0});
        world.setLinearVelocity(b, {-1, 0, 0});

        for (int i = 0; i < 60; ++i)
            world.step(1.0f / 60.0f);

        return {world.body(a).position, world.body(b).position};
    };

    auto runBA = []() -> std::pair<Vec3, Vec3> {
        World world(BackendType::Cpu);
        world.gravity = {0, 0, 0};
        BodyId b = world.addSphere({10, 0, 0}, 0.5f, 2.0f);
        BodyId a = world.addSphere({0, 0, 0}, 0.5f, 1.0f);
        world.setLinearVelocity(a, {1, 0, 0});
        world.setLinearVelocity(b, {-1, 0, 0});

        for (int i = 0; i < 60; ++i)
            world.step(1.0f / 60.0f);

        return {world.body(a).position, world.body(b).position};
    };

    auto [posA1, posB1] = runAB();
    auto [posA2, posB2] = runBA();

    // Non-interacting bodies should have identical trajectories
    CHECK(posA1.x == doctest::Approx(posA2.x).epsilon(1e-4f));
    CHECK(posA1.y == doctest::Approx(posA2.y).epsilon(1e-4f));
    CHECK(posB1.x == doctest::Approx(posB2.x).epsilon(1e-4f));
    CHECK(posB1.y == doctest::Approx(posB2.y).epsilon(1e-4f));
}

TEST_CASE("Numerical: snapshot restore produces identical continuation") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};
    world.addStaticPlane({0, 1, 0}, 0.0f);
    BodyId ball = world.addSphere({0, 5, 0}, 0.5f, 1.0f);
    world.setLinearVelocity(ball, {1, 0, 0.5f});

    // Run 60 steps
    for (int i = 0; i < 60; ++i)
        world.step(1.0f / 60.0f);

    // Save snapshot
    WorldSnapshot snap = world.saveSnapshot();

    // Run 60 more steps and record position
    for (int i = 0; i < 60; ++i)
        world.step(1.0f / 60.0f);
    Vec3 pos1 = world.body(ball).position;

    // Restore and run same 60 steps
    world.restoreSnapshot(snap);
    for (int i = 0; i < 60; ++i)
        world.step(1.0f / 60.0f);
    Vec3 pos2 = world.body(ball).position;

    CHECK(pos1.x == doctest::Approx(pos2.x).epsilon(1e-5f));
    CHECK(pos1.y == doctest::Approx(pos2.y).epsilon(1e-5f));
    CHECK(pos1.z == doctest::Approx(pos2.z).epsilon(1e-5f));
}

TEST_CASE("Numerical: determinism with joints") {
    auto runSim = []() -> Vec3 {
        World world(BackendType::Cpu);
        world.gravity = {0, -10, 0};
        BodyId a = world.addBox({0, 10, 0}, {1, 1, 1}, 0.0f);
        BodyId b = world.addSphere({0, 7, 0}, 0.5f, 1.0f);
        BodyId c = world.addSphere({0, 5, 0}, 0.5f, 1.0f);
        world.addDistanceJoint(a, b, {0, 10, 0}, {0, 7, 0});
        world.addBallJoint(b, c, {0, 6, 0});
        world.setLinearVelocity(c, {3, 0, 0});

        for (int i = 0; i < 120; ++i)
            world.step(1.0f / 60.0f);

        return world.body(c).position;
    };

    Vec3 pos1 = runSim();
    Vec3 pos2 = runSim();

    CHECK(pos1.x == doctest::Approx(pos2.x).epsilon(1e-4f));
    CHECK(pos1.y == doctest::Approx(pos2.y).epsilon(1e-4f));
    CHECK(pos1.z == doctest::Approx(pos2.z).epsilon(1e-4f));
}

TEST_CASE("Numerical: single large dt vs many small dts") {
    // Compare one step of dt=1/10 vs ten steps of dt=1/100
    // They won't be identical but should be close for simple free-fall
    float totalDt = 0.1f;

    World worldA(BackendType::Cpu);
    worldA.gravity = {0, -10, 0};
    BodyId a = worldA.addSphere({0, 10, 0}, 0.5f, 1.0f);
    worldA.step(totalDt);
    float ya = worldA.body(a).position.y;

    World worldB(BackendType::Cpu);
    worldB.gravity = {0, -10, 0};
    BodyId b = worldB.addSphere({0, 10, 0}, 0.5f, 1.0f);
    for (int i = 0; i < 10; ++i)
        worldB.step(totalDt / 10.0f);
    float yb = worldB.body(b).position.y;

    // Both should be close to analytic: 10 - 0.5*10*0.01 = 9.95
    float analytic = 10.0f - 0.5f * 10.0f * totalDt * totalDt;
    CHECK(ya == doctest::Approx(analytic).epsilon(0.1f));
    CHECK(yb == doctest::Approx(analytic).epsilon(0.05f));
}

// ============================================================================
// Precision stress tests
// ============================================================================

TEST_CASE("Numerical: alternating tiny and large forces") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    BodyId id = world.addSphere({0, 0, 0}, 0.5f, 1.0f);

    float dt = 1.0f / 60.0f;
    for (int i = 0; i < 600; ++i) {
        if (i % 2 == 0)
            world.addForce(id, {1e6f, 0, 0});
        else
            world.addForce(id, {-1e6f, 0, 0});
        world.step(dt);
    }

    // Net force is zero each pair — velocity should be near zero
    Vec3 v = world.body(id).velocity;
    CHECK(std::isfinite(v.x));
    // Due to alternating forces, there may be some residual but shouldn't explode
    CHECK(std::abs(v.x) < 1e6f);
}

TEST_CASE("Numerical: very stiff spring does not explode") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    BodyId anchor = world.addSphere({0, 0, 0}, 0.5f, 0.0f);
    BodyId mass = world.addSphere({1, 0, 0}, 0.5f, 1.0f);

    // Very high frequency spring — stiff
    world.addSpringJoint(anchor, mass, {0, 0, 0}, {1, 0, 0}, 100.0f, 1.0f);
    world.setLinearVelocity(mass, {0, 1, 0});

    for (int i = 0; i < 600; ++i)
        world.step(1.0f / 60.0f);

    Vec3 pos = world.body(mass).position;
    CHECK(std::isfinite(pos.x));
    CHECK(std::isfinite(pos.y));
    CHECK(std::isfinite(pos.z));
}

TEST_CASE("Numerical: simultaneous collisions from multiple directions") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};

    // Central body
    BodyId center = world.addSphere({0, 0, 0}, 1.0f, 1.0f);

    // Four bodies approaching from cardinal directions
    BodyId n = world.addSphere({0, 5, 0}, 0.5f, 1.0f);
    BodyId s = world.addSphere({0, -5, 0}, 0.5f, 1.0f);
    BodyId e = world.addSphere({5, 0, 0}, 0.5f, 1.0f);
    BodyId w = world.addSphere({-5, 0, 0}, 0.5f, 1.0f);

    world.setLinearVelocity(n, {0, -10, 0});
    world.setLinearVelocity(s, {0, 10, 0});
    world.setLinearVelocity(e, {-10, 0, 0});
    world.setLinearVelocity(w, {10, 0, 0});

    for (int i = 0; i < 120; ++i)
        world.step(1.0f / 60.0f);

    // All positions should be finite
    for (BodyId id : {center, n, s, e, w}) {
        Vec3 pos = world.body(id).position;
        CHECK(std::isfinite(pos.x));
        CHECK(std::isfinite(pos.y));
        CHECK(std::isfinite(pos.z));
    }

    // Center body should stay near origin due to symmetric impacts
    Vec3 centerPos = world.body(center).position;
    CHECK(std::abs(centerPos.x) < 2.0f);
    CHECK(std::abs(centerPos.y) < 2.0f);
}

TEST_CASE("Numerical: momentum conservation in collision") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};

    float massA = 1.0f, massB = 3.0f;
    BodyId a = world.addSphere({-2, 0, 0}, 0.5f, massA);
    BodyId b = world.addSphere({2, 0, 0}, 0.5f, massB);

    world.setLinearVelocity(a, {10, 0, 0});
    world.setLinearVelocity(b, {-2, 0, 0});

    // Initial momentum
    float px0 = massA * 10.0f + massB * (-2.0f);

    for (int i = 0; i < 120; ++i)
        world.step(1.0f / 60.0f);

    // Final momentum
    float px1 = massA * world.body(a).velocity.x +
                massB * world.body(b).velocity.x;

    // Momentum should be conserved (no external forces)
    CHECK(px1 == doctest::Approx(px0).epsilon(0.05f));
}

TEST_CASE("Numerical: angular momentum conservation (free rotation)") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    BodyId id = world.addBox({0, 0, 0}, {2, 1, 0.5f}, 1.0f);
    world.setAngularVelocity(id, {1, 2, 3});

    Vec3 L0 = world.body(id).worldAngularMomentum();

    for (int i = 0; i < 600; ++i)
        world.step(1.0f / 60.0f);

    Vec3 L1 = world.body(id).worldAngularMomentum();

    // Angular momentum should be conserved for a free rigid body
    CHECK(L1.x == doctest::Approx(L0.x).epsilon(0.05f));
    CHECK(L1.y == doctest::Approx(L0.y).epsilon(0.05f));
    CHECK(L1.z == doctest::Approx(L0.z).epsilon(0.05f));
}

TEST_CASE("Numerical: shiftOrigin preserves relative positions") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    BodyId a = world.addSphere({100, 200, 300}, 0.5f, 1.0f);
    BodyId b = world.addSphere({105, 200, 300}, 0.5f, 1.0f);

    Vec3 relBefore = world.body(b).position - world.body(a).position;

    world.shiftOrigin({100, 200, 300});

    Vec3 relAfter = world.body(b).position - world.body(a).position;

    CHECK(relAfter.x == doctest::Approx(relBefore.x).epsilon(1e-4f));
    CHECK(relAfter.y == doctest::Approx(relBefore.y).epsilon(1e-4f));
    CHECK(relAfter.z == doctest::Approx(relBefore.z).epsilon(1e-4f));

    // Body A should now be near origin
    Vec3 posA = world.body(a).position;
    CHECK(std::abs(posA.x) < 1.0f);
    CHECK(std::abs(posA.y) < 1.0f);
    CHECK(std::abs(posA.z) < 1.0f);
}

TEST_CASE("Numerical: damping exponential decay is smooth") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    BodyId id = world.addSphere({0, 0, 0}, 0.5f, 1.0f);
    world.setLinearVelocity(id, {10, 0, 0});
    world.setLinearDamping(id, 1.0f);

    float dt = 1.0f / 60.0f;
    float prevSpeed = 10.0f;
    bool monotonic = true;

    for (int i = 0; i < 300; ++i) {
        world.step(dt);
        float speed = length(world.body(id).velocity);
        if (speed > prevSpeed + 1e-4f)
            monotonic = false;
        prevSpeed = speed;
    }

    // Speed should decrease monotonically with damping
    CHECK(monotonic);
    CHECK(prevSpeed < 1.0f); // should have decayed significantly
}

TEST_CASE("Numerical: contact normal consistency across frames") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};
    world.addStaticPlane({0, 1, 0}, 0.0f);
    BodyId ball = world.addSphere({0, 2, 0}, 0.5f, 1.0f);

    // Let ball settle on plane
    for (int i = 0; i < 120; ++i)
        world.step(1.0f / 60.0f);

    // Check contact events have consistent normals (pointing up from plane)
    bool hasContact = false;
    for (int i = 0; i < 10; ++i) {
        world.step(1.0f / 60.0f);
        for (const auto& evt : world.contactEvents()) {
            if (evt.type == ContactEventType::Persist || evt.type == ContactEventType::Begin) {
                hasContact = true;
                // Normal should be roughly upward (from plane to ball)
                CHECK(evt.normal.y > 0.5f);
            }
        }
    }
    // We expect contacts to be generated
    CHECK(hasContact);
}
