// A compact catalogue of Velox joint types. Each independent assembly uses
// a static anchor so the example remains stable and easy to inspect.
#include <velox/velox.h>

#include <cmath>
#include <cstdio>

namespace {

bool finite(velox::Vec3 value) {
    return std::isfinite(value.x) && std::isfinite(value.y) &&
           std::isfinite(value.z);
}

} // namespace

int main() {
    using namespace velox;

    World world(BackendType::Cpu);
    world.gravity = {0.0f, -9.81f, 0.0f};

    const BodyId ballAnchor = world.addSphere({-12, 5, 0}, 0.2f, 0.0f);
    const BodyId ball = world.addSphere({-12, 3.5f, 0}, 0.3f, 1.0f);
    const JointId ballJoint = world.addBallJoint(ballAnchor, ball, {-12, 5, 0});

    const BodyId distanceAnchor = world.addSphere({-8, 5, 0}, 0.2f, 0.0f);
    const BodyId distance = world.addSphere({-8, 3, 0}, 0.3f, 1.0f);
    const JointId distanceJoint = world.addDistanceJoint(
        distanceAnchor, distance, {-8, 5, 0}, {-8, 3, 0});

    const BodyId springAnchor = world.addSphere({-4, 5, 0}, 0.2f, 0.0f);
    const BodyId spring = world.addSphere({-4, 3, 0}, 0.3f, 1.0f);
    const JointId springJoint = world.addSpringJoint(
        springAnchor, spring, {-4, 5, 0}, {-4, 3, 0}, 3.0f, 0.8f);

    const BodyId hingeAnchor = world.addBox({0, 5, 0}, {0.2f, 0.2f, 0.2f}, 0.0f);
    const BodyId door = world.addBox({1, 4, 0}, {1.0f, 0.15f, 0.5f}, 2.0f);
    const JointId hingeJoint = world.addHingeJoint(
        hingeAnchor, door, {0, 4, 0}, {0, 0, 1});
    world.joint(hingeJoint).enableMotor = true;
    world.joint(hingeJoint).motorSpeed = 1.5f;
    world.joint(hingeJoint).maxMotorTorque = 12.0f;

    const BodyId coneAnchor = world.addBox({5, 5, 0}, {0.2f, 0.2f, 0.2f}, 0.0f);
    const BodyId coneBody = world.addBox({5, 3.8f, 0}, {0.3f, 0.6f, 0.3f}, 1.0f);
    const JointId coneJoint = world.addConeTwistJoint(
        coneAnchor, coneBody, {5, 5, 0}, {0, 1, 0});
    world.joint(coneJoint).enableSwingLimit = true;
    world.joint(coneJoint).swingLimit = 0.6f;

    const BodyId fixedAnchor = world.addBox({9, 4, 0}, {0.2f, 0.2f, 0.2f}, 0.0f);
    const BodyId fixedBody = world.addBox({10, 4, 0}, {0.4f, 0.3f, 0.3f}, 1.0f);
    const JointId fixedJoint = world.addFixedJoint(fixedAnchor, fixedBody, {9.5f, 4, 0});

    const BodyId rail = world.addBox({13, 4, 0}, {0.2f, 0.2f, 0.2f}, 0.0f);
    const BodyId carriage = world.addBox({13, 4, 0}, {0.3f, 0.3f, 0.3f}, 1.0f);
    const JointId prismaticJoint = world.addPrismaticJoint(rail, carriage, {13, 4, 0}, {1, 0, 0});
    world.joint(prismaticJoint).enableMotor = true;
    world.joint(prismaticJoint).motorSpeed = 1.0f;
    world.joint(prismaticJoint).maxMotorForce = 20.0f;

    const BodyId sixDofAnchor = world.addBox({17, 4, 0}, {0.2f, 0.2f, 0.2f}, 0.0f);
    const BodyId sixDofBody = world.addBox({18, 4, 0}, {0.4f, 0.3f, 0.3f}, 1.0f);
    const JointId sixDofJoint = world.addSixDofJoint(sixDofAnchor, sixDofBody, {17.5f, 4, 0});

    for (int frame = 0; frame < 120; ++frame)
        world.step(1.0f / 60.0f);

    const bool valid = world.isValid(ballJoint) && world.isValid(distanceJoint) &&
                       world.isValid(springJoint) && world.isValid(hingeJoint) &&
                       world.isValid(coneJoint) && world.isValid(fixedJoint) &&
                       world.isValid(prismaticJoint) && world.isValid(sixDofJoint);
    const bool stateIsFinite = finite(world.body(ball).position) &&
                               finite(world.body(distance).position) &&
                               finite(world.body(spring).position) &&
                               finite(world.body(door).position) &&
                               finite(world.body(coneBody).position) &&
                               finite(world.body(fixedBody).position) &&
                               finite(world.body(carriage).position) &&
                               finite(world.body(sixDofBody).position);
    const bool queriesAreFinite = std::isfinite(world.hingeAngle(hingeJoint)) &&
                                 std::isfinite(world.coneSwingAngle(coneJoint)) &&
                                 std::isfinite(world.coneTwistAngle(coneJoint)) &&
                                 std::isfinite(world.prismaticTranslation(prismaticJoint)) &&
                                 finite(world.sixDofLinearTranslation(sixDofJoint)) &&
                                 finite(world.sixDofAngularRotation(sixDofJoint));

    const bool ok = valid && stateIsFinite && queriesAreFinite;
    std::printf("joints_demo: %s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
