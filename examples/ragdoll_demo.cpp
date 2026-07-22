#include <velox/velox.h>

#include <cmath>
#include <cstdio>
#include <functional>
#include <stdexcept>
#include <vector>

namespace {

using namespace velox;

bool finite(Vec3 value) {
    return std::isfinite(value.x) && std::isfinite(value.y) &&
           std::isfinite(value.z);
}

bool throwsInvalidArgument(const std::function<void()>& function) {
    try {
        function();
    } catch (const VeloxInvalidArgument&) {
        return true;
    }
    return false;
}

} // namespace

int main() {
    int failures = 0;
    const auto check = [&](bool condition, const char* label) {
        std::printf("%s: %s\n", condition ? "PASS" : "FAIL", label);
        if (!condition) ++failures;
    };

    World world(BackendType::Cpu);
    world.addStaticPlane({0, 1, 0}, 0.0f);
    const BodyId torso = world.addBox({0, 4, 0}, {0.45f, 0.7f, 0.25f}, 0.0f);
    const BodyId head = world.addSphere({0, 5.05f, 0}, 0.28f, 1.0f);
    const BodyId upperArmL = world.addCapsule({-0.8f, 4.45f, 0}, 0.14f, 0.38f, 0.7f);
    const BodyId upperArmR = world.addCapsule({0.8f, 4.45f, 0}, 0.14f, 0.38f, 0.7f);
    const BodyId forearmL = world.addCapsule({-1.45f, 4.2f, 0}, 0.12f, 0.32f, 0.5f);
    const BodyId forearmR = world.addCapsule({1.45f, 4.2f, 0}, 0.12f, 0.32f, 0.5f);
    const BodyId thighL = world.addCapsule({-0.28f, 2.75f, 0}, 0.16f, 0.48f, 1.2f);
    const BodyId thighR = world.addCapsule({0.28f, 2.75f, 0}, 0.16f, 0.48f, 1.2f);

    const std::vector<RagdollBone> bones{
        {torso, {}, 8.0f}, {head, {}, 1.0f}, {upperArmL, {}, 0.7f},
        {upperArmR, {}, 0.7f}, {forearmL, {}, 0.5f}, {forearmR, {}, 0.5f},
        {thighL, {}, 1.2f}, {thighR, {}, 1.2f},
    };
    const std::vector<RagdollJoint> links{
        {torso, head, {0, 4.75f, 0}, {0, 1, 0}, 0.5f, 0.4f},
        {torso, upperArmL, {-0.45f, 4.5f, 0}, {1, 0, 0}, 0.7f, 0.4f},
        {torso, upperArmR, {0.45f, 4.5f, 0}, {1, 0, 0}, 0.7f, 0.4f},
        {upperArmL, forearmL, {-1.1f, 4.3f, 0}, {0, 0, 1}, 0.5f, 0.4f},
        {upperArmR, forearmR, {1.1f, 4.3f, 0}, {0, 0, 1}, 0.5f, 0.4f},
        {torso, thighL, {-0.25f, 3.35f, 0}, {0, 1, 0}, 0.6f, 0.4f},
        {torso, thighR, {0.25f, 3.35f, 0}, {0, 1, 0}, 0.6f, 0.4f},
    };
    const BodyId root = RagdollBuilder::Build(world, bones, links);
    const std::vector<JointId> joints = RagdollBuilder::Joints(world, root);
    for (int frame = 0; frame < 300; ++frame)
        world.step(1.0f / 60.0f);
    bool stable = root == torso && joints.size() == links.size();
    for (const RagdollBone& bone : bones)
        stable = stable && finite(world.body(bone.body).position);
    for (JointId joint : joints)
        stable = stable && world.isValid(joint);
    check(stable, "eight-bone ragdoll remains finite with valid constraints");

    RagdollBuilder::WakeAll(world, root);
    bool allAwake = true;
    for (const RagdollBone& bone : bones)
        allAwake = allAwake && world.isAwake(bone.body);
    check(allAwake, "WakeAll wakes every registered ragdoll body");

    World motorWorld(BackendType::Cpu);
    motorWorld.gravity = {};
    const BodyId shoulder = motorWorld.addBox({}, {0.2f, 0.2f, 0.2f}, 0.0f);
    const BodyId forearm = motorWorld.addBox({1, 0, 0}, {0.5f, 0.15f, 0.15f}, 1.0f);
    const BodyId motorRoot = RagdollBuilder::Build(
        motorWorld, {{shoulder, {}, 1.0f}, {forearm, {}, 1.0f}},
        {{shoulder, forearm, {}, {0, 0, 1}, 0.0f, 0.8f, true, 4.0f, 100.0f}});
    const JointId elbow = RagdollBuilder::Joints(motorWorld, motorRoot).front();
    RagdollBuilder::SetMotorTorque(motorWorld, elbow, 100.0f);
    for (int frame = 0; frame < 30; ++frame)
        motorWorld.step(1.0f / 60.0f);
    check(std::fabs(motorWorld.hingeAngle(elbow)) > 0.2f,
          "motorized ragdoll elbow rotates toward its target speed");

    const bool cycleRejected = throwsInvalidArgument([&] {
        RagdollBuilder::Build(motorWorld, {{shoulder, {}, 1.0f}, {forearm, {}, 1.0f}},
                              {{shoulder, forearm, {}, {0, 0, 1}},
                               {forearm, shoulder, {}, {0, 0, 1}}});
    });
    const bool disconnectedRejected = throwsInvalidArgument([&] {
        const BodyId spare = motorWorld.addSphere({3, 0, 0}, 0.2f, 1.0f);
        RagdollBuilder::Build(motorWorld,
                              {{shoulder, {}, 1.0f}, {forearm, {}, 1.0f},
                               {spare, {}, 1.0f}},
                              {{shoulder, forearm, {}, {0, 0, 1}},
                               {forearm, shoulder, {}, {0, 0, 1}}});
    });
    check(cycleRejected && disconnectedRejected,
          "ragdoll builder rejects cycles and disconnected graphs");

    if (failures) {
        std::fprintf(stderr, "ragdoll_demo: %d failure(s)\n", failures);
        return 1;
    }
    std::puts("ragdoll_demo: all checks passed");
    return 0;
}
