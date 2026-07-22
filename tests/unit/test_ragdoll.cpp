#include "doctest.h"
#include <velox/velox.h>
#include <cmath>

using namespace velox;

TEST_CASE("RagdollBuilder Build with simple two-bone ragdoll") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};

    BodyId torso = world.addBox({0, 5, 0}, {0.3f, 0.5f, 0.2f}, 10.0f);
    BodyId head = world.addSphere({0, 6.2f, 0}, 0.2f, 3.0f);

    std::vector<RagdollBone> bones = {
        {torso, {0, 0, 0}, 10.0f},
        {head, {0, 0, 0}, 3.0f}
    };

    std::vector<RagdollJoint> joints = {
        {torso, head, {0, 5.7f, 0}, {0, 0, 1}, 1.0f, 0.5f, false, 0.0f, 50.0f}
    };

    BodyId root = RagdollBuilder::Build(world, bones, joints);
    CHECK(world.isValid(root));
    CHECK(root == torso);
}

TEST_CASE("RagdollBuilder Build creates joints") {
    World world(BackendType::Cpu);

    BodyId torso = world.addBox({0, 5, 0}, {0.3f, 0.5f, 0.2f}, 10.0f);
    BodyId head = world.addSphere({0, 6.2f, 0}, 0.2f, 3.0f);
    BodyId pelvis = world.addBox({0, 4, 0}, {0.25f, 0.2f, 0.15f}, 5.0f);

    std::vector<RagdollBone> bones = {
        {pelvis, {0, 0, 0}, 5.0f},
        {torso, {0, 0, 0}, 10.0f},
        {head, {0, 0, 0}, 3.0f}
    };

    std::vector<RagdollJoint> joints = {
        {pelvis, torso, {0, 4.5f, 0}, {0, 0, 1}, 1.2f, 0.4f, false, 0.0f, 50.0f},
        {torso, head, {0, 5.7f, 0}, {0, 0, 1}, 1.0f, 0.5f, false, 0.0f, 50.0f}
    };

    BodyId root = RagdollBuilder::Build(world, bones, joints);

    std::vector<JointId> createdJoints = RagdollBuilder::Joints(world, root);
    CHECK(createdJoints.size() == 2);
}

TEST_CASE("RagdollBuilder Joints returns correct count") {
    World world(BackendType::Cpu);

    // Build a 4-bone chain: root -> a -> b -> c
    BodyId root = world.addBox({0, 8, 0}, {0.2f, 0.2f, 0.2f}, 2.0f);
    BodyId a = world.addBox({0, 6, 0}, {0.2f, 0.2f, 0.2f}, 2.0f);
    BodyId b = world.addBox({0, 4, 0}, {0.2f, 0.2f, 0.2f}, 2.0f);
    BodyId c = world.addBox({0, 2, 0}, {0.2f, 0.2f, 0.2f}, 2.0f);

    std::vector<RagdollBone> bones = {
        {root, {0, 0, 0}, 2.0f},
        {a, {0, 0, 0}, 2.0f},
        {b, {0, 0, 0}, 2.0f},
        {c, {0, 0, 0}, 2.0f}
    };

    std::vector<RagdollJoint> joints = {
        {root, a, {0, 7, 0}, {0, 0, 1}, 1.5f, 0.5f, false, 0.0f, 50.0f},
        {a, b, {0, 5, 0}, {0, 0, 1}, 1.5f, 0.5f, false, 0.0f, 50.0f},
        {b, c, {0, 3, 0}, {0, 0, 1}, 1.5f, 0.5f, false, 0.0f, 50.0f}
    };

    BodyId ragdollRoot = RagdollBuilder::Build(world, bones, joints);

    std::vector<JointId> createdJoints = RagdollBuilder::Joints(world, ragdollRoot);
    CHECK(createdJoints.size() == 3);
}

TEST_CASE("RagdollBuilder Build with motorized joint") {
    World world(BackendType::Cpu);

    BodyId upper = world.addBox({0, 5, 0}, {0.2f, 0.4f, 0.2f}, 5.0f);
    BodyId lower = world.addBox({0, 3, 0}, {0.2f, 0.4f, 0.2f}, 5.0f);

    std::vector<RagdollBone> bones = {
        {upper, {0, 0, 0}, 5.0f},
        {lower, {0, 0, 0}, 5.0f}
    };

    std::vector<RagdollJoint> joints = {
        {upper, lower, {0, 4, 0}, {1, 0, 0}, 1.0f, 0.5f, true, 2.0f, 100.0f}
    };

    BodyId root = RagdollBuilder::Build(world, bones, joints);
    CHECK(world.isValid(root));

    std::vector<JointId> createdJoints = RagdollBuilder::Joints(world, root);
    CHECK(createdJoints.size() == 1);
}

TEST_CASE("RagdollBuilder SetMotorTorque") {
    World world(BackendType::Cpu);

    BodyId upper = world.addBox({0, 5, 0}, {0.2f, 0.4f, 0.2f}, 5.0f);
    BodyId lower = world.addBox({0, 3, 0}, {0.2f, 0.4f, 0.2f}, 5.0f);

    std::vector<RagdollBone> bones = {
        {upper, {0, 0, 0}, 5.0f},
        {lower, {0, 0, 0}, 5.0f}
    };

    std::vector<RagdollJoint> joints = {
        {upper, lower, {0, 4, 0}, {1, 0, 0}, 1.0f, 0.5f, true, 2.0f, 50.0f}
    };

    BodyId root = RagdollBuilder::Build(world, bones, joints);
    std::vector<JointId> createdJoints = RagdollBuilder::Joints(world, root);
    REQUIRE(createdJoints.size() == 1);

    // SetMotorTorque should not throw
    CHECK_NOTHROW(RagdollBuilder::SetMotorTorque(world, createdJoints[0], 75.0f));
}

TEST_CASE("RagdollBuilder WakeAll") {
    World world(BackendType::Cpu);

    BodyId torso = world.addBox({0, 5, 0}, {0.3f, 0.5f, 0.2f}, 10.0f);
    BodyId head = world.addSphere({0, 6.2f, 0}, 0.2f, 3.0f);

    std::vector<RagdollBone> bones = {
        {torso, {0, 0, 0}, 10.0f},
        {head, {0, 0, 0}, 3.0f}
    };

    std::vector<RagdollJoint> joints = {
        {torso, head, {0, 5.7f, 0}, {0, 0, 1}, 1.0f, 0.5f, false, 0.0f, 50.0f}
    };

    BodyId root = RagdollBuilder::Build(world, bones, joints);

    // WakeAll should not throw
    CHECK_NOTHROW(RagdollBuilder::WakeAll(world, root));
}

TEST_CASE("RagdollBuilder Build single bone no joints") {
    World world(BackendType::Cpu);

    BodyId single = world.addSphere({0, 5, 0}, 0.3f, 5.0f);

    std::vector<RagdollBone> bones = {
        {single, {0, 0, 0}, 5.0f}
    };

    std::vector<RagdollJoint> joints = {};

    BodyId root = RagdollBuilder::Build(world, bones, joints);
    CHECK(world.isValid(root));
    CHECK(root == single);

    std::vector<JointId> createdJoints = RagdollBuilder::Joints(world, root);
    CHECK(createdJoints.size() == 0);
}

TEST_CASE("RagdollBuilder ragdoll falls under gravity") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};

    BodyId torso = world.addBox({0, 10, 0}, {0.3f, 0.5f, 0.2f}, 10.0f);
    BodyId head = world.addSphere({0, 11.2f, 0}, 0.2f, 3.0f);

    std::vector<RagdollBone> bones = {
        {torso, {0, 0, 0}, 10.0f},
        {head, {0, 0, 0}, 3.0f}
    };

    std::vector<RagdollJoint> joints = {
        {torso, head, {0, 10.7f, 0}, {0, 0, 1}, 1.0f, 0.5f, false, 0.0f, 50.0f}
    };

    BodyId root = RagdollBuilder::Build(world, bones, joints);

    float y0 = world.body(root).position.y;
    for (int i = 0; i < 60; ++i) {
        world.step(1.0f / 60.0f);
    }
    float y1 = world.body(root).position.y;

    // Ragdoll should have fallen
    CHECK(y1 < y0);
}
