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

    // --- Ball joint ---------------------------------------------------------
    const BodyId ballAnchor = world.addSphere({-24, 5, 0}, 0.2f, 0.0f);
    const BodyId ball = world.addSphere({-24, 3.5f, 0}, 0.3f, 1.0f);
    const JointId ballJoint = world.addBallJoint(ballAnchor, ball, {-24, 5, 0});

    // --- Distance joint -----------------------------------------------------
    const BodyId distanceAnchor = world.addSphere({-20, 5, 0}, 0.2f, 0.0f);
    const BodyId distance = world.addSphere({-20, 3, 0}, 0.3f, 1.0f);
    const JointId distanceJoint = world.addDistanceJoint(
        distanceAnchor, distance, {-20, 5, 0}, {-20, 3, 0});

    // --- Spring joint -------------------------------------------------------
    const BodyId springAnchor = world.addSphere({-16, 5, 0}, 0.2f, 0.0f);
    const BodyId spring = world.addSphere({-16, 3, 0}, 0.3f, 1.0f);
    const JointId springJoint = world.addSpringJoint(
        springAnchor, spring, {-16, 5, 0}, {-16, 3, 0}, 3.0f, 0.8f);

    // --- Hinge joint (with motor) -------------------------------------------
    const BodyId hingeAnchor = world.addBox({-12, 5, 0}, {0.2f, 0.2f, 0.2f}, 0.0f);
    const BodyId door = world.addBox({-11, 4, 0}, {1.0f, 0.15f, 0.5f}, 2.0f);
    const JointId hingeJoint = world.addHingeJoint(
        hingeAnchor, door, {-12, 4, 0}, {0, 0, 1});
    world.joint(hingeJoint).enableMotor = true;
    world.joint(hingeJoint).motorSpeed = 1.5f;
    world.joint(hingeJoint).maxMotorTorque = 12.0f;

    // --- ConeTwist joint (with swing limit) ---------------------------------
    const BodyId coneAnchor = world.addBox({-7, 5, 0}, {0.2f, 0.2f, 0.2f}, 0.0f);
    const BodyId coneBody = world.addBox({-7, 3.8f, 0}, {0.3f, 0.6f, 0.3f}, 1.0f);
    const JointId coneJoint = world.addConeTwistJoint(
        coneAnchor, coneBody, {-7, 5, 0}, {0, 1, 0});
    world.joint(coneJoint).enableSwingLimit = true;
    world.joint(coneJoint).swingLimit = 0.6f;

    // --- Fixed joint --------------------------------------------------------
    const BodyId fixedAnchor = world.addBox({-3, 4, 0}, {0.2f, 0.2f, 0.2f}, 0.0f);
    const BodyId fixedBody = world.addBox({-2, 4, 0}, {0.4f, 0.3f, 0.3f}, 1.0f);
    const JointId fixedJoint = world.addFixedJoint(fixedAnchor, fixedBody, {-2.5f, 4, 0});

    // --- Prismatic joint (with motor) ---------------------------------------
    const BodyId rail = world.addBox({1, 4, 0}, {0.2f, 0.2f, 0.2f}, 0.0f);
    const BodyId carriage = world.addBox({1, 4, 0}, {0.3f, 0.3f, 0.3f}, 1.0f);
    const JointId prismaticJoint = world.addPrismaticJoint(rail, carriage, {1, 4, 0}, {1, 0, 0});
    world.joint(prismaticJoint).enableMotor = true;
    world.joint(prismaticJoint).motorSpeed = 1.0f;
    world.joint(prismaticJoint).maxMotorForce = 20.0f;

    // --- SixDof joint -------------------------------------------------------
    const BodyId sixDofAnchor = world.addBox({5, 4, 0}, {0.2f, 0.2f, 0.2f}, 0.0f);
    const BodyId sixDofBody = world.addBox({6, 4, 0}, {0.4f, 0.3f, 0.3f}, 1.0f);
    const JointId sixDofJoint = world.addSixDofJoint(sixDofAnchor, sixDofBody, {5.5f, 4, 0});

    // --- Weld joint (breakable) ---------------------------------------------
    const BodyId weldAnchor = world.addBox({9, 4, 0}, {0.2f, 0.2f, 0.2f}, 0.0f);
    const BodyId weldBody = world.addBox({10, 4, 0}, {0.4f, 0.3f, 0.3f}, 1.0f);
    const JointId weldJoint = world.addWeldJoint(weldAnchor, weldBody, {9.5f, 4, 0},
                                                  500.0f, 500.0f);

    // --- Wheel joint (suspension + motor) -----------------------------------
    const BodyId chassis = world.addBox({13, 4, 0}, {1.0f, 0.3f, 0.5f}, 0.0f);
    const BodyId wheel = world.addSphere({13, 3, 0}, 0.4f, 1.0f);
    const JointId wheelJoint = world.addWheelJoint(chassis, wheel, {13, 3.5f, 0}, {0, 1, 0});
    world.joint(wheelJoint).suspensionFrequencyHz = 4.0f;
    world.joint(wheelJoint).suspensionDampingRatio = 0.6f;
    world.joint(wheelJoint).enableMotor = true;
    world.joint(wheelJoint).motorSpeed = 8.0f;
    world.joint(wheelJoint).maxMotorTorque = 5.0f;

    // --- Rope joint (slack allowed) -----------------------------------------
    const BodyId ropeAnchor = world.addSphere({17, 6, 0}, 0.2f, 0.0f);
    const BodyId ropeBody = world.addSphere({17, 3, 0}, 0.3f, 1.0f);
    const JointId ropeJoint = world.addRopeJoint(
        ropeAnchor, ropeBody, {17, 6, 0}, {17, 3, 0}, 4.0f);

    // --- Pulley joint (mechanical advantage) --------------------------------
    const BodyId pulleyA = world.addSphere({21, 4, 0}, 0.3f, 2.0f);
    const BodyId pulleyB = world.addSphere({24, 4, 0}, 0.3f, 1.0f);
    const JointId pulleyJoint = world.addPulleyJoint(
        pulleyA, pulleyB,
        {21, 4, 0}, {24, 4, 0},
        {21, 8, 0}, {24, 8, 0},
        2.0f);

    // --- Gear joint (coupled rotation) --------------------------------------
    const BodyId gearA = world.addSphere({28, 4, 0}, 0.5f, 1.0f);
    const BodyId gearB = world.addSphere({30, 4, 0}, 0.5f, 1.0f);
    const JointId gearJoint = world.addGearJoint(gearA, gearB, {0, 1, 0}, {0, 1, 0}, 1.5f);
    world.setAngularVelocity(gearA, {0, 4, 0});

    // --- Simulate -----------------------------------------------------------
    for (int frame = 0; frame < 120; ++frame)
        world.step(1.0f / 60.0f);

    // --- Validate -----------------------------------------------------------
    const bool valid =
        world.isValid(ballJoint) && world.isValid(distanceJoint) &&
        world.isValid(springJoint) && world.isValid(hingeJoint) &&
        world.isValid(coneJoint) && world.isValid(fixedJoint) &&
        world.isValid(prismaticJoint) && world.isValid(sixDofJoint) &&
        world.isValid(weldJoint) && world.isValid(wheelJoint) &&
        world.isValid(ropeJoint) && world.isValid(pulleyJoint) &&
        world.isValid(gearJoint);

    const bool stateIsFinite =
        finite(world.body(ball).position) &&
        finite(world.body(distance).position) &&
        finite(world.body(spring).position) &&
        finite(world.body(door).position) &&
        finite(world.body(coneBody).position) &&
        finite(world.body(fixedBody).position) &&
        finite(world.body(carriage).position) &&
        finite(world.body(sixDofBody).position) &&
        finite(world.body(weldBody).position) &&
        finite(world.body(wheel).position) &&
        finite(world.body(ropeBody).position) &&
        finite(world.body(pulleyA).position) &&
        finite(world.body(pulleyB).position) &&
        finite(world.body(gearA).position) &&
        finite(world.body(gearB).position);

    const bool queriesAreFinite =
        std::isfinite(world.hingeAngle(hingeJoint)) &&
        std::isfinite(world.coneSwingAngle(coneJoint)) &&
        std::isfinite(world.coneTwistAngle(coneJoint)) &&
        std::isfinite(world.prismaticTranslation(prismaticJoint)) &&
        finite(world.sixDofLinearTranslation(sixDofJoint)) &&
        finite(world.sixDofAngularRotation(sixDofJoint));

    const bool ok = valid && stateIsFinite && queriesAreFinite;
    std::printf("joints_demo: %s (%zu bodies, %zu joints)\n",
                ok ? "PASS" : "FAIL",
                world.bodyCount(),
                static_cast<size_t>(13));
    return ok ? 0 : 1;
}
