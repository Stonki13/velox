#include "doctest.h"
#include <velox/velox.h>
#include <chrono>
#include <cmath>
#include <vector>

using namespace velox;

// ---------------------------------------------------------------------------
// Helper: step the world N times at 60 Hz
// ---------------------------------------------------------------------------
static void stepN(World& world, int n) {
    for (int i = 0; i < n; ++i)
        world.step(1.0f / 60.0f);
}

// ===========================================================================
//  Layer interaction tests
// ===========================================================================

TEST_CASE("CollisionFilter: same layer collides") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};

    BodyId floor = world.addStaticPlane({0, 1, 0}, 0.0f);
    world.setCollisionFilter(floor, CollisionLayers::Static, CollisionLayers::Dynamic);

    BodyId ball = world.addSphere({0, 2, 0}, 0.5f, 1.0f);
    world.setCollisionFilter(ball, CollisionLayers::Dynamic, CollisionLayers::Static);

    stepN(world, 120);

    // Ball should rest on the floor (not fall through)
    float y = world.body(ball).position.y;
    CHECK(y > doctest::Approx(0.0f));
}

TEST_CASE("CollisionFilter: different layers do not collide") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};

    BodyId floor = world.addStaticPlane({0, 1, 0}, 0.0f);
    world.setCollisionFilter(floor, CollisionLayers::Static, CollisionLayers::Static);

    BodyId ball = world.addSphere({0, 2, 0}, 0.5f, 1.0f);
    world.setCollisionFilter(ball, CollisionLayers::Dynamic, CollisionLayers::Dynamic);

    stepN(world, 120);

    // Ball should fall through since layers don't match
    float y = world.body(ball).position.y;
    CHECK(y < doctest::Approx(0.0f));
}

TEST_CASE("CollisionFilter: bidirectional mask required") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};

    // Floor accepts Dynamic, but ball's mask doesn't include Static
    BodyId floor = world.addStaticPlane({0, 1, 0}, 0.0f);
    world.setCollisionFilter(floor, CollisionLayers::Static, CollisionLayers::Dynamic);

    BodyId ball = world.addSphere({0, 2, 0}, 0.5f, 1.0f);
    world.setCollisionFilter(ball, CollisionLayers::Dynamic, CollisionLayers::Dynamic); // no Static in mask

    stepN(world, 120);
    CHECK(world.body(ball).position.y < doctest::Approx(0.0f));
}

TEST_CASE("CollisionFilter: multi-layer mask") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};

    uint32_t floorMask = CollisionLayers::Dynamic | CollisionLayers::Character | CollisionLayers::Vehicle;
    BodyId floor = world.addStaticPlane({0, 1, 0}, 0.0f);
    world.setCollisionFilter(floor, CollisionLayers::Static, floorMask);

    // Ball on Dynamic layer — should collide
    BodyId ball = world.addSphere({0, 2, 0}, 0.5f, 1.0f);
    world.setCollisionFilter(ball, CollisionLayers::Dynamic, CollisionLayers::Static);

    // Ghost on User0 layer — should NOT collide (not in floor mask)
    BodyId ghost = world.addSphere({2, 2, 0}, 0.5f, 1.0f);
    world.setCollisionFilter(ghost, CollisionLayers::User0, CollisionLayers::Static);

    stepN(world, 120);

    CHECK(world.body(ball).position.y > doctest::Approx(0.0f));
    CHECK(world.body(ghost).position.y < doctest::Approx(0.0f));
}

TEST_CASE("CollisionFilter: makeLayerMask helper") {
    uint32_t mask2 = makeLayerMask(CollisionLayers::Dynamic, CollisionLayers::Projectile);
    CHECK((mask2 & CollisionLayers::Dynamic) != 0);
    CHECK((mask2 & CollisionLayers::Projectile) != 0);
    CHECK((mask2 & CollisionLayers::Character) == 0);

    uint32_t mask3 = makeLayerMask(CollisionLayers::Static, CollisionLayers::Dynamic, CollisionLayers::Trigger);
    CHECK((mask3 & CollisionLayers::Static) != 0);
    CHECK((mask3 & CollisionLayers::Dynamic) != 0);
    CHECK((mask3 & CollisionLayers::Trigger) != 0);
    CHECK((mask3 & CollisionLayers::Vehicle) == 0);
}

TEST_CASE("CollisionFilter: bodyHasLayer and bodyAcceptsLayer") {
    World world(BackendType::Cpu);
    BodyId id = world.addSphere({0, 0, 0}, 0.5f, 1.0f);
    world.setCollisionFilter(id, CollisionLayers::Dynamic | CollisionLayers::Projectile,
                                 CollisionLayers::Static | CollisionLayers::Dynamic);

    const Body& b = world.body(id);
    CHECK(bodyHasLayer(b, CollisionLayers::Dynamic));
    CHECK(bodyHasLayer(b, CollisionLayers::Projectile));
    CHECK_FALSE(bodyHasLayer(b, CollisionLayers::Static));

    CHECK(bodyAcceptsLayer(b, CollisionLayers::Static));
    CHECK(bodyAcceptsLayer(b, CollisionLayers::Dynamic));
    CHECK_FALSE(bodyAcceptsLayer(b, CollisionLayers::Projectile));
}

TEST_CASE("CollisionFilter: CollisionFilterData two-arg constructor") {
    CollisionFilterData fd(CollisionLayers::Dynamic, CollisionLayers::Static);
    CHECK(fd.categoryBits == CollisionLayers::Dynamic);
    CHECK(fd.maskBits == CollisionLayers::Static);
    CHECK(fd.groupIndex == 0);
}

TEST_CASE("CollisionFilter: CollisionFilterData three-arg constructor") {
    CollisionFilterData fd(CollisionLayers::Dynamic, CollisionLayers::Static, -3);
    CHECK(fd.categoryBits == CollisionLayers::Dynamic);
    CHECK(fd.maskBits == CollisionLayers::Static);
    CHECK(fd.groupIndex == -3);
}

// ===========================================================================
//  Group exclusion tests
// ===========================================================================

TEST_CASE("CollisionFilter: negative group prevents collision") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};

    BodyId floor = world.addStaticPlane({0, 1, 0}, 0.0f);
    // Floor has group 0, so group logic doesn't apply

    // Two balls in the same negative group: they should NOT collide with each other
    BodyId b1 = world.addSphere({0, 2, 0}, 0.5f, 1.0f);
    world.setCollisionFilter(b1, CollisionFilterData{CollisionLayers::Dynamic, CollisionLayers::All, -1});

    BodyId b2 = world.addSphere({0.3f, 2, 0}, 0.5f, 1.0f); // overlapping
    world.setCollisionFilter(b2, CollisionFilterData{CollisionLayers::Dynamic, CollisionLayers::All, -1});

    stepN(world, 60);

    // Both should fall (they pass through each other due to negative group)
    // They should still collide with the floor (group 0 vs -1 → fall through to layers)
    float y1 = world.body(b1).position.y;
    float y2 = world.body(b2).position.y;
    // They should be near the floor, not pushed apart by mutual collision
    CHECK(y1 > doctest::Approx(0.0f));
    CHECK(y2 > doctest::Approx(0.0f));
    // Key check: they should be close together (not repelled)
    float dx = world.body(b1).position.x - world.body(b2).position.x;
    CHECK(std::abs(dx) < doctest::Approx(0.5f));
}

TEST_CASE("CollisionFilter: positive group forces collision") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0}; // no gravity

    // Two bodies with incompatible layers but same positive group → must collide
    BodyId a = world.addSphere({0, 0, 0}, 0.5f, 1.0f);
    world.setCollisionFilter(a, CollisionFilterData{CollisionLayers::Static, CollisionLayers::None, 5});

    BodyId b = world.addSphere({0.8f, 0, 0}, 0.5f, 1.0f); // overlapping
    world.setCollisionFilter(b, CollisionFilterData{CollisionLayers::Dynamic, CollisionLayers::None, 5});

    // Give them velocities toward each other
    world.setLinearVelocity(a, {1, 0, 0});
    world.setLinearVelocity(b, {-1, 0, 0});

    stepN(world, 30);

    // Despite layers saying "no collision", the positive group forces it.
    // The bodies should have bounced / been pushed apart.
    float xa = world.body(a).position.x;
    float xb = world.body(b).position.x;
    // They should have separated (collision response occurred)
    CHECK(xb - xa > doctest::Approx(0.5f));
}

TEST_CASE("CollisionFilter: different groups fall through to layers") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};

    BodyId floor = world.addStaticPlane({0, 1, 0}, 0.0f);
    world.setCollisionFilter(floor, CollisionFilterData{CollisionLayers::Static, CollisionLayers::All, 0});

    // Ball in group +1, floor in group 0 → different groups → layer test
    BodyId ball = world.addSphere({0, 2, 0}, 0.5f, 1.0f);
    world.setCollisionFilter(ball, CollisionFilterData{CollisionLayers::Dynamic, CollisionLayers::Static, 1});

    stepN(world, 120);
    // Layers match (Dynamic vs Static), so ball should rest on floor
    CHECK(world.body(ball).position.y > doctest::Approx(0.0f));
}

TEST_CASE("CollisionFilter: group zero always uses layers") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};

    BodyId floor = world.addStaticPlane({0, 1, 0}, 0.0f);
    // Both group 0 → layer test
    BodyId ball = world.addSphere({0, 2, 0}, 0.5f, 1.0f);

    // Default filter: category=1, mask=ALL, group=0
    stepN(world, 120);
    CHECK(world.body(ball).position.y > doctest::Approx(0.0f));
}

TEST_CASE("CollisionFilter: setCollisionFilter with CollisionFilterData") {
    World world(BackendType::Cpu);
    BodyId id = world.addSphere({0, 0, 0}, 0.5f, 1.0f);

    CollisionFilterData fd{CollisionLayers::Projectile, CollisionLayers::Static, -7};
    world.setCollisionFilter(id, fd);

    CollisionFilterData readBack = world.collisionFilter(id);
    CHECK(readBack.categoryBits == CollisionLayers::Projectile);
    CHECK(readBack.maskBits == CollisionLayers::Static);
    CHECK(readBack.groupIndex == -7);
}

TEST_CASE("CollisionFilter: filter change purges contacts") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};

    BodyId floor = world.addStaticPlane({0, 1, 0}, 0.0f);
    BodyId ball = world.addSphere({0, 1.5f, 0}, 0.5f, 1.0f);

    // Let them collide first
    stepN(world, 60);
    CHECK(world.body(ball).position.y > doctest::Approx(0.0f));

    // Now change filter to prevent collision
    world.setCollisionFilter(ball, CollisionFilterData{CollisionLayers::Dynamic, CollisionLayers::None, 0});
    stepN(world, 120);

    // Ball should fall through
    CHECK(world.body(ball).position.y < doctest::Approx(0.0f));
}

// ===========================================================================
//  Custom callback tests
// ===========================================================================

TEST_CASE("CollisionFilter: custom callback rejects all") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};

    BodyId floor = world.addStaticPlane({0, 1, 0}, 0.0f);
    BodyId ball = world.addSphere({0, 2, 0}, 0.5f, 1.0f);

    world.setCollisionFilterCallback([](const Body&, const Body&) {
        return FilterResult::Reject;
    });

    stepN(world, 120);
    CHECK(world.body(ball).position.y < doctest::Approx(0.0f));
}

TEST_CASE("CollisionFilter: custom callback accepts all") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};

    BodyId floor = world.addStaticPlane({0, 1, 0}, 0.0f);
    // Incompatible layers
    world.setCollisionFilter(floor, CollisionLayers::Static, CollisionLayers::None);

    BodyId ball = world.addSphere({0, 2, 0}, 0.5f, 1.0f);
    world.setCollisionFilter(ball, CollisionLayers::Dynamic, CollisionLayers::None);

    // Callback forces collision despite incompatible layers
    world.setCollisionFilterCallback([](const Body&, const Body&) {
        return FilterResult::Accept;
    });

    stepN(world, 120);
    CHECK(world.body(ball).position.y > doctest::Approx(0.0f));
}

TEST_CASE("CollisionFilter: custom callback default defers to built-in") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};

    BodyId floor = world.addStaticPlane({0, 1, 0}, 0.0f);
    world.setCollisionFilter(floor, CollisionLayers::Static, CollisionLayers::Dynamic);

    BodyId ball = world.addSphere({0, 2, 0}, 0.5f, 1.0f);
    world.setCollisionFilter(ball, CollisionLayers::Dynamic, CollisionLayers::Static);

    // Callback returns Default → built-in rules apply → collision happens
    world.setCollisionFilterCallback([](const Body&, const Body&) {
        return FilterResult::Default;
    });

    stepN(world, 120);
    CHECK(world.body(ball).position.y > doctest::Approx(0.0f));
}

TEST_CASE("CollisionFilter: custom callback selective rejection") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};

    BodyId floor = world.addStaticPlane({0, 1, 0}, 0.0f);
    world.setCollisionFilter(floor, CollisionLayers::Static, CollisionLayers::All);

    BodyId ball = world.addSphere({0, 2, 0}, 0.5f, 1.0f);
    world.setCollisionFilter(ball, CollisionLayers::Dynamic, CollisionLayers::Static);

    BodyId ghost = world.addSphere({2, 2, 0}, 0.5f, 1.0f);
    world.setCollisionFilter(ghost, CollisionLayers::Trigger, CollisionLayers::Static);

    // Reject only Trigger-layer bodies
    world.setCollisionFilterCallback([](const Body& a, const Body& b) {
        if ((a.categoryBits & CollisionLayers::Trigger) ||
            (b.categoryBits & CollisionLayers::Trigger))
            return FilterResult::Reject;
        return FilterResult::Default;
    });

    stepN(world, 120);

    // Ball collides with floor
    CHECK(world.body(ball).position.y > doctest::Approx(0.0f));
    // Ghost falls through
    CHECK(world.body(ghost).position.y < doctest::Approx(0.0f));
}

TEST_CASE("CollisionFilter: removing callback restores built-in behavior") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};

    BodyId floor = world.addStaticPlane({0, 1, 0}, 0.0f);
    BodyId ball = world.addSphere({0, 2, 0}, 0.5f, 1.0f);

    // Install a reject-all callback
    world.setCollisionFilterCallback([](const Body&, const Body&) {
        return FilterResult::Reject;
    });
    stepN(world, 30);
    CHECK(world.body(ball).position.y < doctest::Approx(1.0f)); // falling

    // Remove the callback
    world.setCollisionFilterCallback(nullptr);
    // Teleport ball back up
    world.setTransform(ball, {0, 2, 0}, {});
    world.setLinearVelocity(ball, {0, 0, 0});
    stepN(world, 120);

    // Now built-in rules apply → ball rests on floor
    CHECK(world.body(ball).position.y > doctest::Approx(0.0f));
}

TEST_CASE("CollisionFilter: callback receives correct body data") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};

    BodyId a = world.addSphere({0, 0, 0}, 0.5f, 1.0f);
    world.setCollisionFilter(a, CollisionFilterData{CollisionLayers::Dynamic, CollisionLayers::All, 42});

    BodyId b = world.addSphere({0.8f, 0, 0}, 0.5f, 1.0f);
    world.setCollisionFilter(b, CollisionFilterData{CollisionLayers::Projectile, CollisionLayers::All, -7});

    bool callbackInvoked = false;
    uint32_t seenCatA = 0, seenCatB = 0;
    int32_t seenGroupA = 0, seenGroupB = 0;

    world.setCollisionFilterCallback([&](const Body& ba, const Body& bb) {
        callbackInvoked = true;
        seenCatA = ba.categoryBits;
        seenCatB = bb.categoryBits;
        seenGroupA = ba.groupIndex;
        seenGroupB = bb.groupIndex;
        return FilterResult::Default;
    });

    world.setLinearVelocity(a, {1, 0, 0});
    stepN(world, 5);

    CHECK(callbackInvoked);
    // The callback should have seen the filter data we set
    CHECK(((seenCatA == CollisionLayers::Dynamic && seenCatB == CollisionLayers::Projectile) ||
           (seenCatA == CollisionLayers::Projectile && seenCatB == CollisionLayers::Dynamic)));
}

// ===========================================================================
//  evaluateCollisionFilter unit tests (no World needed)
// ===========================================================================

TEST_CASE("CollisionFilter: evaluateCollisionFilter group priority") {
    Body a, b;
    a.categoryBits = CollisionLayers::Dynamic;
    a.maskBits = CollisionLayers::None; // layers say NO
    a.groupIndex = 3;

    b.categoryBits = CollisionLayers::Static;
    b.maskBits = CollisionLayers::None; // layers say NO
    b.groupIndex = 3;

    // Same positive group → collide despite layers
    CHECK(evaluateCollisionFilter(a, b));

    // Same negative group → don't collide
    a.groupIndex = -3;
    b.groupIndex = -3;
    CHECK_FALSE(evaluateCollisionFilter(a, b));

    // Different groups → fall through to layers (which say NO)
    a.groupIndex = 3;
    b.groupIndex = -3;
    CHECK_FALSE(evaluateCollisionFilter(a, b));

    // One group zero → fall through to layers
    a.groupIndex = 0;
    b.groupIndex = 3;
    CHECK_FALSE(evaluateCollisionFilter(a, b));
}

TEST_CASE("CollisionFilter: evaluateCollisionFilter layer test") {
    Body a, b;
    a.groupIndex = 0;
    b.groupIndex = 0;

    // Compatible layers
    a.categoryBits = CollisionLayers::Dynamic;
    a.maskBits = CollisionLayers::Static;
    b.categoryBits = CollisionLayers::Static;
    b.maskBits = CollisionLayers::Dynamic;
    CHECK(evaluateCollisionFilter(a, b));

    // One-way mask failure
    b.maskBits = CollisionLayers::None;
    CHECK_FALSE(evaluateCollisionFilter(a, b));
}

TEST_CASE("CollisionFilter: evaluateCollisionFilterWithCallback") {
    Body a, b;
    a.categoryBits = CollisionLayers::Dynamic;
    a.maskBits = CollisionLayers::Static;
    a.groupIndex = 0;
    b.categoryBits = CollisionLayers::Static;
    b.maskBits = CollisionLayers::Dynamic;
    b.groupIndex = 0;

    // No callback → built-in
    CollisionFilterCallback empty;
    CHECK(evaluateCollisionFilterWithCallback(a, b, empty));

    // Callback rejects
    CollisionFilterCallback reject = [](const Body&, const Body&) { return FilterResult::Reject; };
    CHECK_FALSE(evaluateCollisionFilterWithCallback(a, b, reject));

    // Callback accepts despite incompatible layers
    a.maskBits = CollisionLayers::None;
    CollisionFilterCallback accept = [](const Body&, const Body&) { return FilterResult::Accept; };
    CHECK(evaluateCollisionFilterWithCallback(a, b, accept));

    // Callback default → built-in (which now says no due to maskBits=None)
    CollisionFilterCallback def = [](const Body&, const Body&) { return FilterResult::Default; };
    CHECK_FALSE(evaluateCollisionFilterWithCallback(a, b, def));
}

// ===========================================================================
//  Performance tests with many bodies
// ===========================================================================

TEST_CASE("CollisionFilter: performance - 500 bodies with layer filtering") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};

    BodyId floor = world.addStaticPlane({0, 1, 0}, 0.0f);
    world.setCollisionFilter(floor, CollisionLayers::Static, CollisionLayers::All);

    const int N = 500;
    std::vector<BodyId> balls;
    balls.reserve(N);

    // Create a grid of spheres with alternating layers
    for (int i = 0; i < N; ++i) {
        float x = float((i % 25) - 12) * 1.2f;
        float y = 2.0f + float(i / 25) * 1.2f;
        float z = float((i % 5) - 2) * 1.2f;
        BodyId id = world.addSphere({x, y, z}, 0.4f, 1.0f);
        uint32_t layer = (i % 2 == 0) ? CollisionLayers::Dynamic : CollisionLayers::Debris;
        world.setCollisionFilter(id, layer, CollisionLayers::Static | CollisionLayers::Dynamic);
        balls.push_back(id);
    }

    auto start = std::chrono::high_resolution_clock::now();
    stepN(world, 60);
    auto end = std::chrono::high_resolution_clock::now();

    double ms = std::chrono::duration<double, std::milli>(end - start).count();
    MESSAGE("500 bodies, 60 steps with layer filtering: " << ms << " ms");

    // Sanity: simulation ran and bodies settled
    CHECK(world.bodyCount() == N + 1);
    // Performance guard: should complete in reasonable time (< 10 seconds)
    CHECK(ms < 10000.0);
}

TEST_CASE("CollisionFilter: performance - 500 bodies with group filtering") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};

    BodyId floor = world.addStaticPlane({0, 1, 0}, 0.0f);

    const int N = 500;
    std::vector<BodyId> balls;
    balls.reserve(N);

    for (int i = 0; i < N; ++i) {
        float x = float((i % 25) - 12) * 1.2f;
        float y = 2.0f + float(i / 25) * 1.2f;
        float z = float((i % 5) - 2) * 1.2f;
        BodyId id = world.addSphere({x, y, z}, 0.4f, 1.0f);
        // Assign to one of 10 negative groups (debris clusters)
        int32_t group = -(i % 10 + 1);
        world.setCollisionFilter(id, CollisionFilterData{CollisionLayers::Dynamic, CollisionLayers::All, group});
        balls.push_back(id);
    }

    auto start = std::chrono::high_resolution_clock::now();
    stepN(world, 60);
    auto end = std::chrono::high_resolution_clock::now();

    double ms = std::chrono::duration<double, std::milli>(end - start).count();
    MESSAGE("500 bodies, 60 steps with group filtering: " << ms << " ms");

    CHECK(world.bodyCount() == N + 1);
    CHECK(ms < 10000.0);
}

TEST_CASE("CollisionFilter: performance - 500 bodies with custom callback") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};

    BodyId floor = world.addStaticPlane({0, 1, 0}, 0.0f);

    const int N = 500;
    for (int i = 0; i < N; ++i) {
        float x = float((i % 25) - 12) * 1.2f;
        float y = 2.0f + float(i / 25) * 1.2f;
        float z = float((i % 5) - 2) * 1.2f;
        BodyId id = world.addSphere({x, y, z}, 0.4f, 1.0f);
        uint32_t layer = (i % 3 == 0) ? CollisionLayers::Trigger : CollisionLayers::Dynamic;
        world.setCollisionFilter(id, layer, CollisionLayers::All);
    }

    // Lightweight callback: reject trigger bodies
    world.setCollisionFilterCallback([](const Body& a, const Body& b) {
        if ((a.categoryBits & CollisionLayers::Trigger) ||
            (b.categoryBits & CollisionLayers::Trigger))
            return FilterResult::Reject;
        return FilterResult::Default;
    });

    auto start = std::chrono::high_resolution_clock::now();
    stepN(world, 60);
    auto end = std::chrono::high_resolution_clock::now();

    double ms = std::chrono::duration<double, std::milli>(end - start).count();
    MESSAGE("500 bodies, 60 steps with custom callback: " << ms << " ms");

    CHECK(world.bodyCount() == N + 1);
    CHECK(ms < 10000.0);
}

TEST_CASE("CollisionFilter: performance - 1000 bodies mixed filtering") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};

    BodyId floor = world.addStaticPlane({0, 1, 0}, 0.0f);
    world.setCollisionFilter(floor, CollisionLayers::Static, CollisionLayers::All);

    const int N = 1000;
    for (int i = 0; i < N; ++i) {
        float x = float((i % 32) - 16) * 1.1f;
        float y = 2.0f + float(i / 32) * 1.1f;
        float z = float((i % 7) - 3) * 1.1f;
        BodyId id = world.addSphere({x, y, z}, 0.35f, 1.0f);

        // Mix of filtering strategies
        if (i % 4 == 0) {
            // Layer-only
            world.setCollisionFilter(id, CollisionLayers::Dynamic, CollisionLayers::Static | CollisionLayers::Dynamic);
        } else if (i % 4 == 1) {
            // Negative group (debris cluster)
            world.setCollisionFilter(id, CollisionFilterData{CollisionLayers::Debris, CollisionLayers::All, -(i % 5 + 1)});
        } else if (i % 4 == 2) {
            // Positive group (ragdoll-like)
            world.setCollisionFilter(id, CollisionFilterData{CollisionLayers::Ragdoll, CollisionLayers::All, (i % 3 + 1)});
        } else {
            // Trigger layer
            world.setCollisionFilter(id, CollisionLayers::Trigger, CollisionLayers::None);
        }
    }

    // Callback rejects triggers
    world.setCollisionFilterCallback([](const Body& a, const Body& b) {
        if ((a.categoryBits & CollisionLayers::Trigger) ||
            (b.categoryBits & CollisionLayers::Trigger))
            return FilterResult::Reject;
        return FilterResult::Default;
    });

    auto start = std::chrono::high_resolution_clock::now();
    stepN(world, 30);
    auto end = std::chrono::high_resolution_clock::now();

    double ms = std::chrono::duration<double, std::milli>(end - start).count();
    MESSAGE("1000 bodies, 30 steps mixed filtering: " << ms << " ms");

    CHECK(world.bodyCount() == N + 1);
    CHECK(ms < 15000.0);
}

// ===========================================================================
//  Integration: combined group + layer + callback
// ===========================================================================

TEST_CASE("CollisionFilter: group overrides callback default") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};

    // Same negative group → never collide, even if callback says Default
    BodyId a = world.addSphere({0, 0, 0}, 0.5f, 1.0f);
    world.setCollisionFilter(a, CollisionFilterData{CollisionLayers::Dynamic, CollisionLayers::All, -2});

    BodyId b = world.addSphere({0.8f, 0, 0}, 0.5f, 1.0f);
    world.setCollisionFilter(b, CollisionFilterData{CollisionLayers::Dynamic, CollisionLayers::All, -2});

    world.setCollisionFilterCallback([](const Body&, const Body&) {
        return FilterResult::Default; // defer to built-in → group says NO
    });

    world.setLinearVelocity(a, {1, 0, 0});
    world.setLinearVelocity(b, {-1, 0, 0});
    stepN(world, 30);

    // They should pass through each other
    float xa = world.body(a).position.x;
    float xb = world.body(b).position.x;
    CHECK(xa > doctest::Approx(xb)); // a passed through b
}

TEST_CASE("CollisionFilter: callback accept overrides negative group") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};

    BodyId a = world.addSphere({0, 0, 0}, 0.5f, 1.0f);
    world.setCollisionFilter(a, CollisionFilterData{CollisionLayers::Dynamic, CollisionLayers::All, -2});

    BodyId b = world.addSphere({0.8f, 0, 0}, 0.5f, 1.0f);
    world.setCollisionFilter(b, CollisionFilterData{CollisionLayers::Dynamic, CollisionLayers::All, -2});

    // Callback forces collision despite negative group
    world.setCollisionFilterCallback([](const Body&, const Body&) {
        return FilterResult::Accept;
    });

    world.setLinearVelocity(a, {1, 0, 0});
    world.setLinearVelocity(b, {-1, 0, 0});
    stepN(world, 30);

    // They should have collided and bounced
    float xa = world.body(a).position.x;
    float xb = world.body(b).position.x;
    CHECK(xb - xa > doctest::Approx(0.5f));
}

TEST_CASE("CollisionFilter: default body filter values") {
    World world(BackendType::Cpu);
    BodyId id = world.addSphere({0, 0, 0}, 0.5f, 1.0f);

    CollisionFilterData fd = world.collisionFilter(id);
    CHECK(fd.categoryBits == 1u);
    CHECK(fd.maskBits == UINT32_MAX);
    CHECK(fd.groupIndex == 0);
}
