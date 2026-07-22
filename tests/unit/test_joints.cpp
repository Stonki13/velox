#include "doctest.h"
#include <velox/velox.h>
#include <cmath>

using namespace velox;

TEST_CASE("Ball joint connects two bodies") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};
    BodyId a = world.addSphere({0, 5, 0}, 0.5f, 1.0f);
    BodyId b = world.addSphere({0, 3, 0}, 0.5f, 1.0f);
    JointId j = world.addBallJoint(a, b, {0, 4, 0});
    CHECK(world.isValid(j));
}

TEST_CASE("Hinge joint angle query") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    BodyId a = world.addBox({0, 0, 0}, {1, 1, 1}, 0.0f);
    BodyId b = world.addBox({3, 0, 0}, {1, 1, 1}, 1.0f);
    JointId j = world.addHingeJoint(a, b, {1.5f, 0, 0}, {0, 1, 0});

    float angle = world.hingeAngle(j);
    CHECK(angle == doctest::Approx(0.0f).epsilon(1e-4f));
}

TEST_CASE("Distance joint constrains bodies") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    BodyId a = world.addSphere({0, 0, 0}, 0.5f, 0.0f);
    BodyId b = world.addSphere({5, 0, 0}, 0.5f, 1.0f);
    world.addDistanceJoint(a, b, {0, 0, 0}, {5, 0, 0});

    for (int i = 0; i < 60; ++i)
        world.step(1.0f / 60.0f);

    float dist = length(world.body(b).position - world.body(a).position);
    CHECK(dist == doctest::Approx(5.0f).epsilon(0.5f));
}

TEST_CASE("Spring joint oscillates") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    BodyId a = world.addSphere({0, 0, 0}, 0.5f, 0.0f);
    BodyId b = world.addSphere({3, 0, 0}, 0.5f, 1.0f);
    world.addSpringJoint(a, b, {0, 0, 0}, {3, 0, 0}, 1.0f, 0.5f);

    world.setLinearVelocity(b, {2, 0, 0});
    for (int i = 0; i < 120; ++i)
        world.step(1.0f / 60.0f);

    Vec3 v = world.body(b).velocity;
    CHECK(length(v) > 0.0f);
}

TEST_CASE("Joint removal with body") {
    World world(BackendType::Cpu);
    BodyId a = world.addSphere({0, 5, 0}, 0.5f, 1.0f);
    BodyId b = world.addSphere({0, 3, 0}, 0.5f, 1.0f);
    JointId j = world.addBallJoint(a, b, {0, 4, 0});
    CHECK(world.isValid(j));

    world.removeBody(a);
    CHECK_FALSE(world.isValid(j));
}

TEST_CASE("Joint break event fires") {
    World world(BackendType::Cpu);
    world.gravity = {0, -50, 0};
    BodyId a = world.addSphere({0, 10, 0}, 0.5f, 0.0f);
    BodyId b = world.addSphere({0, 5, 0}, 0.5f, 10.0f);
    JointId j = world.addDistanceJoint(a, b, {0, 10, 0}, {0, 5, 0});
    world.joint(j).breakForce = 10.0f;

    bool broke = false;
    for (int i = 0; i < 300; ++i) {
        world.step(1.0f / 60.0f);
        for (const auto& evt : world.jointBreakEvents()) {
            if (evt.joint == j) broke = true;
        }
    }

    CHECK(broke);
}

TEST_CASE("Prismatic joint constrains translation") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    BodyId a = world.addBox({0, 0, 0}, {1, 1, 1}, 0.0f);
    BodyId b = world.addBox({3, 0, 0}, {0.5f, 0.5f, 0.5f}, 1.0f);
    world.addPrismaticJoint(a, b, {3, 0, 0}, {1, 0, 0});

    for (int i = 0; i < 60; ++i)
        world.step(1.0f / 60.0f);

    Vec3 pos = world.body(b).position;
    CHECK(pos.y == doctest::Approx(0.0f).epsilon(0.1f));
    CHECK(pos.z == doctest::Approx(0.0f).epsilon(0.1f));
}
