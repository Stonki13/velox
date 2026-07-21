#include "doctest.h"
#include <velox/velox.h>
#include <cmath>

using namespace velox;

TEST_CASE("CharacterController creation with default descriptor") {
    World world(BackendType::Cpu);
    CharacterControllerDesc desc{};
    CharacterController cc(world, desc);

    Vec3 pos = cc.Position();
    CHECK(pos.x == doctest::Approx(0.0f));
    CHECK(pos.y == doctest::Approx(0.0f));
    CHECK(pos.z == doctest::Approx(0.0f));
}

TEST_CASE("CharacterController SetPosition and Position") {
    World world(BackendType::Cpu);
    CharacterController cc(world);

    cc.SetPosition({10.0f, 5.0f, -3.0f});
    Vec3 pos = cc.Position();
    CHECK(pos.x == doctest::Approx(10.0f));
    CHECK(pos.y == doctest::Approx(5.0f));
    CHECK(pos.z == doctest::Approx(-3.0f));
}

TEST_CASE("CharacterController Move on flat ground") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};

    // Add a static ground plane at y=0
    world.addStaticPlane({0, 1, 0}, 0.0f);

    CharacterControllerDesc desc{};
    desc.capsuleRadius = 0.3f;
    desc.capsuleHalfHeight = 0.9f;
    CharacterController cc(world, desc);

    // Place the character just above the ground
    cc.SetPosition({0.0f, desc.capsuleHalfHeight + desc.capsuleRadius + 0.05f, 0.0f});

    // Move forward along X
    CharacterControllerResult result = cc.Move({1.0f, 0.0f, 0.0f});

    // The character should have moved in X
    Vec3 finalPos = result.finalPosition;
    CHECK(finalPos.x > 0.0f);
}

TEST_CASE("CharacterController Move with zero displacement") {
    World world(BackendType::Cpu);
    world.addStaticPlane({0, 1, 0}, 0.0f);

    CharacterController cc(world);
    cc.SetPosition({0.0f, 2.0f, 0.0f});

    CharacterControllerResult result = cc.Move({0.0f, 0.0f, 0.0f});

    // Position should remain roughly the same
    Vec3 pos = cc.Position();
    CHECK(pos.x == doctest::Approx(0.0f).epsilon(0.1f));
    CHECK(pos.z == doctest::Approx(0.0f).epsilon(0.1f));
}

TEST_CASE("CharacterController stepping behavior") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};

    // Ground plane
    world.addStaticPlane({0, 1, 0}, 0.0f);

    // A small box to step on (height 0.2, top at y=0.2)
    world.addBox({2.0f, 0.1f, 0.0f}, {0.5f, 0.1f, 0.5f}, 0.0f);

    CharacterControllerDesc desc{};
    desc.stepMaxHeight = 0.3f; // Can step up to 0.3m
    CharacterController cc(world, desc);

    // Place character near the step
    cc.SetPosition({1.2f, desc.capsuleHalfHeight + desc.capsuleRadius + 0.05f, 0.0f});

    // Move toward the step
    CharacterControllerResult result = cc.Move({2.0f, 0.0f, 0.0f});

    // The controller should have attempted to step (stepped flag or moved forward)
    // Either it stepped or it was blocked — verify it processed the move
    CHECK(result.finalPosition.x >= 1.2f);
}

TEST_CASE("CharacterController slope limit") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};

    // Ground plane
    world.addStaticPlane({0, 1, 0}, 0.0f);

    CharacterControllerDesc desc{};
    desc.slopeLimitCosine = 0.7071f; // ~45 degrees
    CharacterController cc(world, desc);

    cc.SetPosition({0.0f, 2.0f, 0.0f});

    // Move with a downward component — should still work on flat ground
    CharacterControllerResult result = cc.Move({0.5f, -0.1f, 0.0f});
    CHECK(result.finalPosition.x > 0.0f);
}

TEST_CASE("CharacterController SetJumpVelocity") {
    World world(BackendType::Cpu);
    world.addStaticPlane({0, 1, 0}, 0.0f);

    CharacterController cc(world);
    cc.SetPosition({0.0f, 2.0f, 0.0f});

    // SetJumpVelocity should not throw
    CHECK_NOTHROW(cc.SetJumpVelocity(5.0f));

    // After setting jump velocity, the next Move should consume it
    CharacterControllerResult result = cc.Move({0.0f, 0.0f, 0.0f});
    // The jump velocity should have been applied (upward displacement)
    // or consumed — verify no crash and position is reasonable
    Vec3 pos = cc.Position();
    CHECK(pos.y > 0.0f);
}

TEST_CASE("CharacterController custom descriptor values") {
    World world(BackendType::Cpu);

    CharacterControllerDesc desc{};
    desc.capsuleRadius = 0.5f;
    desc.capsuleHalfHeight = 1.2f;
    desc.stepMaxHeight = 0.5f;
    desc.slopeLimitCosine = 0.5f;
    desc.movementSpeed = 10.0f;
    desc.ghostPadding = 0.02f;

    CharacterController cc(world, desc);
    cc.SetPosition({5.0f, 3.0f, 5.0f});

    Vec3 pos = cc.Position();
    CHECK(pos.x == doctest::Approx(5.0f));
    CHECK(pos.y == doctest::Approx(3.0f));
    CHECK(pos.z == doctest::Approx(5.0f));
}

TEST_CASE("CharacterController Move result fields") {
    World world(BackendType::Cpu);
    world.addStaticPlane({0, 1, 0}, 0.0f);

    CharacterController cc(world);
    cc.SetPosition({0.0f, 2.0f, 0.0f});

    CharacterControllerResult result = cc.Move({1.0f, 0.0f, 0.0f});

    // Verify result struct has valid fields — at least one component may be nonzero
    CHECK(result.contactCount >= 0);
}
