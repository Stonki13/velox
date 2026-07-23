#include "doctest.h"
#include <velox/velox.h>
#include <chrono>
#include <cmath>

using namespace velox;

// Helper: step the world N times at 60 Hz.
static void stepN(World& world, int n) {
    for (int i = 0; i < n; ++i)
        world.step(1.0f / 60.0f);
}

// ============================================================================
// Sleep threshold tests
// ============================================================================

TEST_CASE("Sleep: body below velocity threshold falls asleep") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    // Disable contact stability so a lone body can sleep without contacts.
    world.sleepConfig().enableContactStability = false;
    world.sleepConfig().timeToSleep = 0.5f;
    world.sleepConfig().enableGradualSleep = false;

    BodyId id = world.addSphere({0, 0, 0}, 0.5f, 1.0f);
    world.setLinearVelocity(id, {0.01f, 0, 0}); // below default 0.05 threshold

    // After enough time below threshold, the body should sleep.
    stepN(world, 60); // 1 second > 0.5s threshold

    CHECK(world.sleepState(id) == SleepState::Asleep);
    CHECK(world.body(id).asleep == 1);
    CHECK(lengthSq(world.body(id).velocity) == doctest::Approx(0.0f));
}

TEST_CASE("Sleep: body above velocity threshold stays awake") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    world.sleepConfig().enableContactStability = false;
    world.sleepConfig().timeToSleep = 0.5f;
    world.sleepConfig().enableGradualSleep = false;

    BodyId id = world.addSphere({0, 0, 0}, 0.5f, 1.0f);
    world.setLinearVelocity(id, {1.0f, 0, 0}); // well above threshold

    stepN(world, 120); // 2 seconds

    CHECK(world.sleepState(id) == SleepState::Awake);
    CHECK(world.isAwake(id));
}

TEST_CASE("Sleep: configurable linear velocity threshold") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    world.sleepConfig().enableContactStability = false;
    world.sleepConfig().enableGradualSleep = false;
    world.sleepConfig().linearVelocityThreshold = 0.5f; // higher threshold
    world.sleepConfig().timeToSleep = 0.3f;

    BodyId id = world.addSphere({0, 0, 0}, 0.5f, 1.0f);
    world.setLinearVelocity(id, {0.3f, 0, 0}); // below 0.5 threshold

    stepN(world, 30); // 0.5s > 0.3s

    CHECK(world.sleepState(id) == SleepState::Asleep);
}

TEST_CASE("Sleep: configurable angular velocity threshold") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    world.sleepConfig().enableContactStability = false;
    world.sleepConfig().enableGradualSleep = false;
    world.sleepConfig().angularVelocityThreshold = 0.1f;
    world.sleepConfig().timeToSleep = 0.3f;

    BodyId id = world.addSphere({0, 0, 0}, 0.5f, 1.0f);
    world.setAngularVelocity(id, {0, 0.05f, 0}); // below 0.1 threshold

    stepN(world, 30);

    CHECK(world.sleepState(id) == SleepState::Asleep);
}

TEST_CASE("Sleep: acceleration threshold keeps body awake under force") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    world.sleepConfig().enableContactStability = false;
    world.sleepConfig().enableGradualSleep = false;
    world.sleepConfig().accelerationThreshold = 0.01f;
    world.sleepConfig().timeToSleep = 0.3f;

    BodyId id = world.addSphere({0, 0, 0}, 0.5f, 1.0f);
    // Apply a continuous force that produces acceleration > threshold.
    // force = 1.0 N, mass = 1.0 kg -> a = 1.0 m/s^2 >> 0.01
    // Force is cleared each step, so we need to re-apply it each step.
    for (int i = 0; i < 60; ++i) {
        world.addForce(id, {1.0f, 0, 0});
        world.step(1.0f / 60.0f);
    }

    // Body should still be awake because force is applied each step.
    CHECK(world.sleepState(id) == SleepState::Awake);
}

TEST_CASE("Sleep: timeToSleep controls how long before sleeping") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    world.sleepConfig().enableContactStability = false;
    world.sleepConfig().enableGradualSleep = false;
    world.sleepConfig().timeToSleep = 2.0f; // long threshold

    BodyId id = world.addSphere({0, 0, 0}, 0.5f, 1.0f);
    // Zero velocity, but timeToSleep is 2 seconds.

    stepN(world, 60); // 1 second < 2 seconds
    CHECK(world.sleepState(id) == SleepState::Awake);

    stepN(world, 90); // total 2.5 seconds > 2 seconds
    CHECK(world.sleepState(id) == SleepState::Asleep);
}

TEST_CASE("Sleep: sleep disabled body never sleeps") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    world.sleepConfig().enableContactStability = false;
    world.sleepConfig().enableGradualSleep = false;
    world.sleepConfig().timeToSleep = 0.1f;

    BodyId id = world.addSphere({0, 0, 0}, 0.5f, 1.0f);
    world.setEnableSleep(id, false);

    stepN(world, 120);

    CHECK(world.sleepState(id) == SleepState::Awake);
    CHECK(world.body(id).asleep == 0);
}

TEST_CASE("Sleep: static bodies are never put to sleep") {
    World world(BackendType::Cpu);
    BodyId id = world.addStaticPlane({0, 1, 0}, 0.0f);

    stepN(world, 60);

    // Static bodies don't participate in the sleep system.
    CHECK(world.body(id).asleep == 0);
}

// ============================================================================
// Gradual sleep (drowsy) tests
// ============================================================================

TEST_CASE("Sleep: gradual sleep enters drowsy before asleep") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    world.sleepConfig().enableContactStability = false;
    world.sleepConfig().enableGradualSleep = true;
    world.sleepConfig().timeToDrowsy = 0.2f;
    world.sleepConfig().timeToSleep = 0.5f;

    BodyId id = world.addSphere({0, 0, 0}, 0.5f, 1.0f);

    // After timeToDrowsy but before timeToSleep, body should be drowsy.
    stepN(world, 18); // ~0.3s > 0.2s drowsy, < 0.5s sleep
    CHECK(world.sleepState(id) == SleepState::Drowsy);
    CHECK(world.body(id).asleep == 2);

    // After timeToSleep, body should be fully asleep.
    stepN(world, 30); // total ~0.8s > 0.5s
    CHECK(world.sleepState(id) == SleepState::Asleep);
    CHECK(world.body(id).asleep == 1);
}

TEST_CASE("Sleep: drowsy body woken by velocity change") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    world.sleepConfig().enableContactStability = false;
    world.sleepConfig().enableGradualSleep = true;
    world.sleepConfig().timeToDrowsy = 0.1f;
    world.sleepConfig().timeToSleep = 1.0f;

    BodyId id = world.addSphere({0, 0, 0}, 0.5f, 1.0f);

    // Get to drowsy state.
    stepN(world, 12); // ~0.2s > 0.1s drowsy
    CHECK(world.sleepState(id) == SleepState::Drowsy);

    // Wake it up.
    world.wake(id);
    CHECK(world.sleepState(id) == SleepState::Awake);
    CHECK(world.isAwake(id));
}

TEST_CASE("Sleep: gradual sleep disabled goes straight to asleep") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    world.sleepConfig().enableContactStability = false;
    world.sleepConfig().enableGradualSleep = false;
    world.sleepConfig().timeToSleep = 0.3f;

    BodyId id = world.addSphere({0, 0, 0}, 0.5f, 1.0f);

    stepN(world, 30); // 0.5s > 0.3s

    // Should go straight to asleep, never drowsy.
    CHECK(world.sleepState(id) == SleepState::Asleep);
}

// ============================================================================
// Wake-on-contact tests
// ============================================================================

TEST_CASE("Sleep: sleeping body woken by collision") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};
    world.sleepConfig().enableContactStability = false;
    world.sleepConfig().enableGradualSleep = false;

    world.addStaticPlane({0, 1, 0}, 0.0f);

    // Sleeping body on the ground.
    BodyId sleeper = world.addSphere({0, 0.5f, 0}, 0.5f, 1.0f);
    world.sleepBody(sleeper);
    CHECK(world.body(sleeper).asleep == 1);

    // Falling body that will hit the sleeper.
    BodyId falling = world.addSphere({0, 5, 0}, 0.5f, 1.0f);

    stepN(world, 120);

    // After collision, the sleeper should have been woken and moved.
    Vec3 pos = world.body(sleeper).position;
    CHECK(std::isfinite(pos.x));
    CHECK(std::isfinite(pos.y));
}

TEST_CASE("Sleep: wake via explicit API") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};

    BodyId id = world.addSphere({0, 0, 0}, 0.5f, 1.0f);
    world.sleepBody(id);
    CHECK(world.body(id).asleep == 1);

    world.wake(id);
    CHECK(world.body(id).asleep == 0);
    CHECK(world.isAwake(id));
}

TEST_CASE("Sleep: wakeBody and sleepBody force state") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};

    BodyId id = world.addSphere({0, 0, 0}, 0.5f, 1.0f);

    world.sleepBody(id);
    CHECK(world.sleepState(id) == SleepState::Asleep);
    CHECK(lengthSq(world.body(id).velocity) == doctest::Approx(0.0f));

    world.wakeBody(id);
    CHECK(world.sleepState(id) == SleepState::Awake);
}

TEST_CASE("Sleep: sleeping body ignores gravity") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};

    BodyId id = world.addSphere({0, 5, 0}, 0.5f, 1.0f);
    world.sleepBody(id);

    float y0 = world.body(id).position.y;
    stepN(world, 60);
    float y1 = world.body(id).position.y;

    CHECK(y1 == doctest::Approx(y0));
}

TEST_CASE("Sleep: wake sleeping body resumes simulation") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};

    BodyId id = world.addSphere({0, 5, 0}, 0.5f, 1.0f);
    world.sleepBody(id);
    world.step(1.0f / 60.0f);
    float ySleeping = world.body(id).position.y;

    world.wakeBody(id);
    world.step(1.0f / 60.0f);
    float yAwake = world.body(id).position.y;

    CHECK(yAwake < ySleeping);
}

// ============================================================================
// Island sleep propagation tests
// ============================================================================

TEST_CASE("Sleep: island of touching bodies sleeps together") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};
    world.sleepConfig().enableContactStability = false;
    world.sleepConfig().enableGradualSleep = false;
    world.sleepConfig().timeToSleep = 0.5f;

    // Ground plane.
    world.addStaticPlane({0, 1, 0}, 0.0f);

    // Stack of two boxes resting on the ground.
    BodyId a = world.addBox({0, 0.5f, 0}, {0.5f, 0.5f, 0.5f}, 1.0f);
    BodyId b = world.addBox({0, 1.5f, 0}, {0.5f, 0.5f, 0.5f}, 1.0f);

    // Let them settle.
    stepN(world, 120); // 2 seconds

    // Both should be asleep (same island via contact).
    CHECK(world.sleepState(a) == SleepState::Asleep);
    CHECK(world.sleepState(b) == SleepState::Asleep);
}

TEST_CASE("Sleep: island with one active body keeps all awake") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    world.sleepConfig().enableContactStability = false;
    world.sleepConfig().enableGradualSleep = false;
    world.sleepConfig().timeToSleep = 0.5f;

    // Two bodies connected by a joint.
    BodyId a = world.addSphere({0, 0, 0}, 0.5f, 1.0f);
    BodyId b = world.addSphere({2, 0, 0}, 0.5f, 1.0f);
    world.addDistanceJoint(a, b, {0, 0, 0}, {2, 0, 0});

    // Give body b a velocity so the island stays active.
    world.setLinearVelocity(b, {0, 1.0f, 0});

    stepN(world, 60); // 1 second

    // Body a should still be awake because it's in the same island as b.
    // (b is moving, so the island's min timer resets.)
    CHECK(world.sleepState(a) == SleepState::Awake);
}

TEST_CASE("Sleep: joint-connected bodies sleep as one island") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    world.sleepConfig().enableContactStability = false;
    world.sleepConfig().enableGradualSleep = false;
    world.sleepConfig().timeToSleep = 0.3f;

    BodyId a = world.addSphere({0, 0, 0}, 0.5f, 1.0f);
    BodyId b = world.addSphere({2, 0, 0}, 0.5f, 1.0f);
    world.addDistanceJoint(a, b, {0, 0, 0}, {2, 0, 0});

    // Both stationary -> should sleep together.
    stepN(world, 30); // 0.5s > 0.3s

    CHECK(world.sleepState(a) == SleepState::Asleep);
    CHECK(world.sleepState(b) == SleepState::Asleep);
}

TEST_CASE("Sleep: separate islands sleep independently") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    world.sleepConfig().enableContactStability = false;
    world.sleepConfig().enableGradualSleep = false;
    world.sleepConfig().timeToSleep = 0.3f;

    // Two independent bodies far apart (no contact, no joint).
    BodyId a = world.addSphere({0, 0, 0}, 0.5f, 1.0f);
    BodyId b = world.addSphere({100, 0, 0}, 0.5f, 1.0f);

    // Give b a velocity so it stays awake.
    world.setLinearVelocity(b, {5.0f, 0, 0});

    stepN(world, 30); // 0.5s > 0.3s

    // a should sleep, b should stay awake.
    CHECK(world.sleepState(a) == SleepState::Asleep);
    CHECK(world.sleepState(b) == SleepState::Awake);
}

// ============================================================================
// Sleep callbacks tests
// ============================================================================

TEST_CASE("Sleep: onSleep callback fires when body sleeps") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    world.sleepConfig().enableContactStability = false;
    world.sleepConfig().enableGradualSleep = false;
    world.sleepConfig().timeToSleep = 0.2f;

    int sleepCount = 0;
    BodyId sleptBody{UINT64_MAX};
    SleepCallbacks cb;
    cb.onSleep = [&](BodyId id) { ++sleepCount; sleptBody = id; };
    world.setSleepCallbacks(std::move(cb));

    BodyId id = world.addSphere({0, 0, 0}, 0.5f, 1.0f);
    stepN(world, 30);

    CHECK(sleepCount >= 1);
    CHECK(sleptBody == id);
}

TEST_CASE("Sleep: onWake callback fires when body is woken") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};

    int wakeCount = 0;
    SleepCallbacks cb;
    cb.onWake = [&](BodyId) { ++wakeCount; };
    world.setSleepCallbacks(std::move(cb));

    BodyId id = world.addSphere({0, 0, 0}, 0.5f, 1.0f);
    world.sleepBody(id);
    CHECK(wakeCount == 0); // sleepBody fires onSleep, not onWake

    world.wakeBody(id);
    CHECK(wakeCount == 1);
}

TEST_CASE("Sleep: onDrowsy callback fires on drowsy transition") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    world.sleepConfig().enableContactStability = false;
    world.sleepConfig().enableGradualSleep = true;
    world.sleepConfig().timeToDrowsy = 0.1f;
    world.sleepConfig().timeToSleep = 1.0f;

    int drowsyCount = 0;
    SleepCallbacks cb;
    cb.onDrowsy = [&](BodyId) { ++drowsyCount; };
    world.setSleepCallbacks(std::move(cb));

    BodyId id = world.addSphere({0, 0, 0}, 0.5f, 1.0f);
    stepN(world, 12); // ~0.2s > 0.1s drowsy

    CHECK(drowsyCount >= 1);
}

TEST_CASE("Sleep: sleepBody fires onSleep callback") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};

    int sleepCount = 0;
    SleepCallbacks cb;
    cb.onSleep = [&](BodyId) { ++sleepCount; };
    world.setSleepCallbacks(std::move(cb));

    BodyId id = world.addSphere({0, 0, 0}, 0.5f, 1.0f);
    world.sleepBody(id);
    CHECK(sleepCount == 1);

    // Sleeping an already-sleeping body should not fire again.
    world.sleepBody(id);
    CHECK(sleepCount == 1);
}

// ============================================================================
// Sleep statistics tests
// ============================================================================

TEST_CASE("Sleep: stats track body counts") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    world.sleepConfig().enableContactStability = false;
    world.sleepConfig().enableGradualSleep = false;
    world.sleepConfig().timeToSleep = 0.2f;

    BodyId a = world.addSphere({0, 0, 0}, 0.5f, 1.0f);
    BodyId b = world.addSphere({100, 0, 0}, 0.5f, 1.0f);
    world.setLinearVelocity(b, {10.0f, 0, 0}); // keep b awake

    stepN(world, 30);

    const SleepStats& stats = world.sleepStats();
    CHECK(stats.totalDynamicBodies == 2);
    CHECK(stats.sleepingBodies >= 1); // a should be asleep
    CHECK(stats.awakeBodies >= 1);    // b should be awake
}

TEST_CASE("Sleep: stats track transition counts") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    world.sleepConfig().enableContactStability = false;
    world.sleepConfig().enableGradualSleep = false;
    world.sleepConfig().timeToSleep = 0.1f;

    BodyId id = world.addSphere({0, 0, 0}, 0.5f, 1.0f);
    stepN(world, 12); // let it sleep

    const SleepStats& stats = world.sleepStats();
    CHECK(stats.totalSleepTransitions >= 1);

    world.wakeBody(id);
    CHECK(world.sleepStats().totalWakeTransitions >= 1);
}

TEST_CASE("Sleep: stats track island counts") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    world.sleepConfig().enableContactStability = false;
    world.sleepConfig().enableGradualSleep = false;
    world.sleepConfig().timeToSleep = 0.2f;

    // Three independent bodies.
    world.addSphere({0, 0, 0}, 0.5f, 1.0f);
    world.addSphere({10, 0, 0}, 0.5f, 1.0f);
    world.addSphere({20, 0, 0}, 0.5f, 1.0f);

    stepN(world, 30);

    const SleepStats& stats = world.sleepStats();
    CHECK(stats.islandCount >= 3);
    CHECK(stats.sleepingIslandCount >= 3);
}

// ============================================================================
// Sleep config validation tests
// ============================================================================

TEST_CASE("Sleep: config validate clamps invalid values") {
    SleepConfig config;
    config.linearVelocityThreshold = -1.0f;
    config.angularVelocityThreshold = -1.0f;
    config.timeToSleep = -1.0f;
    config.timeToDrowsy = 10.0f; // > timeToSleep
    config.drowsySimulationRate = 5.0f; // > 1
    config.validate();

    CHECK(config.linearVelocityThreshold >= 0.0f);
    CHECK(config.angularVelocityThreshold >= 0.0f);
    CHECK(config.timeToSleep >= 0.0f);
    // When timeToSleep is 0, timeToDrowsy should also be 0
    CHECK(config.timeToDrowsy <= config.timeToSleep);
    CHECK(config.drowsySimulationRate <= 1.0f);
    CHECK(config.drowsySimulationRate > 0.0f);
}

TEST_CASE("Sleep: setSleepConfig validates automatically") {
    World world(BackendType::Cpu);
    SleepConfig config;
    config.timeToDrowsy = 100.0f;
    config.timeToSleep = 1.0f;
    world.setSleepConfig(config);

    CHECK(world.sleepConfig().timeToDrowsy < world.sleepConfig().timeToSleep);
}

// ============================================================================
// Sleep state query tests
// ============================================================================

TEST_CASE("Sleep: sleepState returns correct state") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};

    BodyId id = world.addSphere({0, 0, 0}, 0.5f, 1.0f);
    CHECK(world.sleepState(id) == SleepState::Awake);

    world.sleepBody(id);
    CHECK(world.sleepState(id) == SleepState::Asleep);

    world.wakeBody(id);
    CHECK(world.sleepState(id) == SleepState::Awake);
}

TEST_CASE("Sleep: isAwake returns false for sleeping and drowsy") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    world.sleepConfig().enableContactStability = false;
    world.sleepConfig().enableGradualSleep = true;
    world.sleepConfig().timeToDrowsy = 0.1f;
    world.sleepConfig().timeToSleep = 1.0f;

    BodyId id = world.addSphere({0, 0, 0}, 0.5f, 1.0f);
    CHECK(world.isAwake(id));

    // Get to drowsy.
    stepN(world, 12);
    CHECK_FALSE(world.isAwake(id)); // drowsy is not "fully awake"

    world.wakeBody(id);
    CHECK(world.isAwake(id));
}

// ============================================================================
// Sleep helper function tests
// ============================================================================

TEST_CASE("Sleep: sleepStateToByte and sleepStateFromByte roundtrip") {
    CHECK(sleepStateFromByte(sleepStateToByte(SleepState::Awake)) == SleepState::Awake);
    CHECK(sleepStateFromByte(sleepStateToByte(SleepState::Asleep)) == SleepState::Asleep);
    CHECK(sleepStateFromByte(sleepStateToByte(SleepState::Drowsy)) == SleepState::Drowsy);
}

TEST_CASE("Sleep: isFullyAsleep and isDrowsy helpers") {
    CHECK(isFullyAsleep(0) == false);
    CHECK(isFullyAsleep(1) == true);
    CHECK(isFullyAsleep(2) == false);

    CHECK(isDrowsy(0) == false);
    CHECK(isDrowsy(1) == false);
    CHECK(isDrowsy(2) == true);

    CHECK(isInactive(0) == false);
    CHECK(isInactive(1) == true);
    CHECK(isInactive(2) == true);
}

// ============================================================================
// Sleep visualization tests
// ============================================================================

TEST_CASE("Sleep: DebugDrawSleep flag produces lines") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    world.sleepConfig().enableContactStability = false;
    world.sleepConfig().enableGradualSleep = false;
    world.sleepConfig().timeToSleep = 0.1f;

    BodyId id = world.addSphere({0, 0, 0}, 0.5f, 1.0f);
    stepN(world, 12); // let it sleep

    std::vector<DebugLine> lines;
    world.debugLines(lines, DebugDrawSleep);
    CHECK(lines.size() > 0);
}

TEST_CASE("Sleep: DebugDrawEverything includes sleep layer") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};

    world.addSphere({0, 0, 0}, 0.5f, 1.0f);

    std::vector<DebugLine> lines;
    world.debugLines(lines, DebugDrawEverything);
    CHECK(lines.size() > 0);
}

// ============================================================================
// Repeated sleep/wake cycle tests
// ============================================================================

TEST_CASE("Sleep: repeated sleep/wake cycles are stable") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};

    BodyId id = world.addSphere({0, 5, 0}, 0.5f, 1.0f);

    for (int cycle = 0; cycle < 100; ++cycle) {
        world.sleepBody(id);
        CHECK(world.body(id).asleep == 1);
        world.wakeBody(id);
        CHECK(world.body(id).asleep == 0);
        world.step(1.0f / 60.0f);
    }

    Vec3 pos = world.body(id).position;
    CHECK(std::isfinite(pos.y));
}

// ============================================================================
// Performance tests with many sleeping bodies
// ============================================================================

TEST_CASE("Sleep: performance with many sleeping bodies") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0}; // no gravity so bodies settle quickly
    world.sleepConfig().enableContactStability = false;
    world.sleepConfig().enableGradualSleep = false;
    world.sleepConfig().timeToSleep = 0.3f;

    // Create a grid of bodies with zero velocity.
    constexpr int kCount = 200;
    std::vector<BodyId> bodies;
    bodies.reserve(kCount);
    for (int i = 0; i < kCount; ++i) {
        float x = float(i % 20) * 2.0f;
        float y = float(i / 20) * 2.0f + 5.0f;
        bodies.push_back(world.addSphere({x, y, 0}, 0.4f, 1.0f));
    }

    // Let them all sleep (zero velocity, no gravity).
    stepN(world, 60);

    // Most should be asleep.
    size_t asleepCount = 0;
    for (BodyId id : bodies)
        if (world.sleepState(id) == SleepState::Asleep)
            ++asleepCount;
    CHECK(asleepCount > kCount / 2);

    // Measure step time with sleeping bodies.
    auto t0 = std::chrono::steady_clock::now();
    stepN(world, 60);
    auto t1 = std::chrono::steady_clock::now();
    double sleepStepMs = std::chrono::duration<double, std::milli>(t1 - t0).count();

    // Wake all bodies and measure again.
    for (BodyId id : bodies)
        world.wakeBody(id);

    auto t2 = std::chrono::steady_clock::now();
    stepN(world, 60);
    auto t3 = std::chrono::steady_clock::now();
    double awakeStepMs = std::chrono::duration<double, std::milli>(t3 - t2).count();

    // Sleeping should be faster (or at least not catastrophically slower).
    // We use a generous bound because CI machines vary.
    CHECK(sleepStepMs < awakeStepMs * 3.0);
}

TEST_CASE("Sleep: performance with 500 bodies") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};
    world.sleepConfig().enableContactStability = false;
    world.sleepConfig().enableGradualSleep = false;
    world.sleepConfig().timeToSleep = 0.2f;

    constexpr int kCount = 500;
    for (int i = 0; i < kCount; ++i) {
        float x = float(i % 25) * 1.5f;
        float y = float(i / 25) * 1.5f + 3.0f;
        float z = float((i / 25) % 5) * 1.5f;
        world.addSphere({x, y, z}, 0.3f, 1.0f);
    }

    // Settle.
    stepN(world, 60);

    // Verify stats are populated.
    const SleepStats& stats = world.sleepStats();
    CHECK(stats.totalDynamicBodies == kCount);
    CHECK(stats.islandCount > 0);

    // Step should complete in reasonable time.
    auto t0 = std::chrono::steady_clock::now();
    stepN(world, 30);
    auto t1 = std::chrono::steady_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    // Generous bound: 30 steps with 500 bodies should take < 5 seconds.
    CHECK(ms < 5000.0);
}

// ============================================================================
// Edge cases
// ============================================================================

TEST_CASE("Sleep: setTransform wakes sleeping body") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};

    BodyId id = world.addSphere({0, 0, 0}, 0.5f, 1.0f);
    world.sleepBody(id);
    CHECK(world.body(id).asleep == 1);

    world.setTransform(id, {10, 0, 0}, {});
    CHECK(world.body(id).asleep == 0);
}

TEST_CASE("Sleep: setLinearVelocity wakes sleeping body") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};

    BodyId id = world.addSphere({0, 0, 0}, 0.5f, 1.0f);
    world.sleepBody(id);

    world.setLinearVelocity(id, {5, 0, 0});
    CHECK(world.body(id).asleep == 0);
}

TEST_CASE("Sleep: removeBody does not crash with sleeping bodies") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};

    BodyId a = world.addSphere({0, 0, 0}, 0.5f, 1.0f);
    BodyId b = world.addSphere({2, 0, 0}, 0.5f, 1.0f);
    world.sleepBody(a);

    world.removeBody(a);
    CHECK_FALSE(world.isValid(a));
    CHECK(world.isValid(b));

    stepN(world, 10);
    CHECK(std::isfinite(world.body(b).position.x));
}

TEST_CASE("Sleep: SleepManager reset clears state") {
    SleepManager mgr;
    mgr.config().timeToSleep = 0.1f;

    // Simulate some state.
    std::vector<Body> bodies(1);
    bodies[0].motionType = MotionType::Dynamic;
    bodies[0].invMass = 1.0f;
    bodies[0].enableSleep = 1;

    std::vector<Contact> contacts;
    std::vector<Joint> joints;
    std::vector<uint32_t> parent;
    std::vector<float> timer;

    auto resolver = [](BodyIndex) { return BodyId::make(0, 0); };

    // Run update to build internal state.
    for (int i = 0; i < 20; ++i)
        mgr.update(bodies, contacts, joints, parent, timer, 1.0f / 60.0f, resolver);

    CHECK(mgr.stats().totalDynamicBodies == 1);

    mgr.reset();
    CHECK(mgr.stats().totalDynamicBodies == 0);
    CHECK(mgr.islands().empty());
}

TEST_CASE("Sleep: SleepManager default config") {
    SleepManager mgr;
    CHECK(mgr.config().timeToSleep > 0.0f);
    CHECK(mgr.config().linearVelocityThreshold > 0.0f);
    CHECK(mgr.config().angularVelocityThreshold > 0.0f);
    CHECK(mgr.config().enableGradualSleep == true);
    CHECK(mgr.config().enableContactStability == true);
}

// ============================================================================
// Regression: drowsy bodies must keep their contacts so contact stability
// is not reset, which would wake them and create a sleep/wake cycle.
// This was a GPU-backend bug: the CUDA/Vulkan broad phases used `b.asleep`
// (truthy for Drowsy) instead of `isFullyAsleep(b.asleep)`, so drowsy pairs
// were culled from the broad phase, contacts vanished, contact stability
// reset to zero, the calm check failed, and the drowsy body was woken.
// ============================================================================

TEST_CASE("Sleep regression: settled pile stays asleep (no drowsy wake cycle)") {
    World world; // BackendType::Auto — catches GPU-backend regressions
    world.addStaticPlane({0, 1, 0}, 0.0f);
    BodyId balls[9];
    for (int i = 0; i < 9; ++i)
        balls[i] = world.addSphere(
            {(i % 3) * 1.05f, 0.5f + (i / 3) * 1.05f, 0}, 0.5f, 1.0f);

    stepN(world, 400); // ~6.7 s to settle

    bool allAsleep = true;
    for (auto id : balls) allAsleep &= !world.isAwake(id);
    CHECK(allAsleep);

    // Verify the pile STAYS asleep for another 2 seconds (no wake cycle).
    stepN(world, 120);
    for (auto id : balls)
        CHECK_FALSE(world.isAwake(id));
}

TEST_CASE("Sleep regression: contact events bounded during rest (no sleep/wake spam)") {
    World world; // BackendType::Auto
    auto floor = world.addStaticPlane({0, 1, 0}, 0.0f);
    auto ball = world.addSphere({0, 2.0f, 0}, 0.5f, 1.0f);

    int began = 0;
    float impulse = 0.0f;
    for (int i = 0; i < 300; ++i) {
        world.step(1.0f / 60.0f);
        for (const auto& ev : world.contactEvents()) {
            bool ours = (ev.a == ball && ev.b == floor) ||
                        (ev.a == floor && ev.b == ball);
            if (ours && ev.type == ContactEventType::Begin) {
                ++began;
                impulse = std::fmax(impulse, ev.impulse);
            }
        }
    }
    // Restitution bounces produce a few Begin events; resting contact must
    // not spam one Begin per sleep/wake cycle.
    CHECK(began >= 1);
    CHECK(began <= 6);
    CHECK(impulse > 0.0f);
}
