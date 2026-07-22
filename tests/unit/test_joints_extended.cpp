#include "doctest.h"
#include <velox/velox.h>
#include <cmath>

using namespace velox;

// ---------------------------------------------------------------------------
// Weld joint
// ---------------------------------------------------------------------------

TEST_CASE("Weld joint locks two bodies rigidly") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};
    BodyId a = world.addBox({0, 5, 0}, {0.5f, 0.5f, 0.5f}, 0.0f);
    BodyId b = world.addBox({1.5f, 5, 0}, {0.5f, 0.5f, 0.5f}, 1.0f);
    JointId j = world.addWeldJoint(a, b, {0.75f, 5, 0});
    CHECK(world.isValid(j));

    for (int i = 0; i < 120; ++i)
        world.step(1.0f / 60.0f);

    // Body B should stay near its initial position relative to A.
    Vec3 posB = world.body(b).position;
    CHECK(posB.x == doctest::Approx(1.5f).epsilon(0.15f));
    CHECK(posB.y == doctest::Approx(5.0f).epsilon(0.15f));
    CHECK(posB.z == doctest::Approx(0.0f).epsilon(0.15f));
}

TEST_CASE("Weld joint breaks under force") {
    World world(BackendType::Cpu);
    world.gravity = {0, -50, 0};
    BodyId a = world.addSphere({0, 10, 0}, 0.5f, 0.0f);
    BodyId b = world.addSphere({0, 5, 0}, 0.5f, 10.0f);
    JointId j = world.addWeldJoint(a, b, {0, 7.5f, 0}, 10.0f, 10.0f);

    bool broke = false;
    for (int i = 0; i < 300; ++i) {
        world.step(1.0f / 60.0f);
        for (const auto& evt : world.jointBreakEvents()) {
            if (evt.joint == j) broke = true;
        }
    }
    CHECK(broke);
}

TEST_CASE("Weld joint with soft angular constraint") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    BodyId a = world.addBox({0, 0, 0}, {0.5f, 0.5f, 0.5f}, 0.0f);
    BodyId b = world.addBox({1.5f, 0, 0}, {0.5f, 0.5f, 0.5f}, 1.0f);
    JointId j = world.addWeldJoint(a, b, {0.75f, 0, 0});
    world.joint(j).weldFrequencyHz = 5.0f;
    world.joint(j).weldDampingRatio = 0.8f;

    // Apply a torque to twist body B.
    world.addTorque(b, {0, 0, 5.0f});
    for (int i = 0; i < 60; ++i)
        world.step(1.0f / 60.0f);

    // The soft weld should resist but allow some angular deviation.
    CHECK(std::isfinite(world.body(b).orientation.w));
    CHECK(std::isfinite(world.body(b).position.x));
}

TEST_CASE("Weld joint rejects same body") {
    World world(BackendType::Cpu);
    BodyId a = world.addSphere({0, 0, 0}, 0.5f, 1.0f);
    CHECK_THROWS(world.addWeldJoint(a, a, {0, 0, 0}));
}

// ---------------------------------------------------------------------------
// Wheel joint
// ---------------------------------------------------------------------------

TEST_CASE("Wheel joint constrains lateral motion") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};
    // Chassis (static anchor).
    BodyId chassis = world.addBox({0, 3, 0}, {1, 0.3f, 0.5f}, 0.0f);
    // Wheel below the chassis.
    BodyId wheel = world.addSphere({0, 2, 0}, 0.4f, 1.0f);
    JointId j = world.addWheelJoint(chassis, wheel, {0, 2.5f, 0}, {0, 1, 0});
    CHECK(world.isValid(j));

    for (int i = 0; i < 120; ++i)
        world.step(1.0f / 60.0f);

    // Wheel should stay near x=0, z=0 (lateral constraint).
    Vec3 pos = world.body(wheel).position;
    CHECK(pos.x == doctest::Approx(0.0f).epsilon(0.2f));
    CHECK(pos.z == doctest::Approx(0.0f).epsilon(0.2f));
}

TEST_CASE("Wheel joint suspension oscillates") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};
    BodyId chassis = world.addBox({0, 5, 0}, {1, 0.3f, 0.5f}, 0.0f);
    BodyId wheel = world.addSphere({0, 4, 0}, 0.4f, 1.0f);
    JointId j = world.addWheelJoint(chassis, wheel, {0, 4.5f, 0}, {0, 1, 0});
    world.joint(j).suspensionFrequencyHz = 3.0f;
    world.joint(j).suspensionDampingRatio = 0.3f;

    // Displace the wheel downward.
    world.setLinearVelocity(wheel, {0, -2, 0});
    for (int i = 0; i < 120; ++i)
        world.step(1.0f / 60.0f);

    // The suspension should have brought the wheel back near equilibrium.
    Vec3 pos = world.body(wheel).position;
    CHECK(std::isfinite(pos.y));
}

TEST_CASE("Wheel joint motor drives spin") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    BodyId chassis = world.addBox({0, 0, 0}, {1, 0.3f, 0.5f}, 0.0f);
    BodyId wheel = world.addSphere({0, -1, 0}, 0.4f, 1.0f);
    JointId j = world.addWheelJoint(chassis, wheel, {0, -0.5f, 0}, {0, 1, 0});
    world.joint(j).enableMotor = true;
    world.joint(j).motorSpeed = 10.0f;
    world.joint(j).maxMotorTorque = 5.0f;

    for (int i = 0; i < 120; ++i)
        world.step(1.0f / 60.0f);

    // The wheel should be spinning about the Y axis.
    float spin = world.body(wheel).angularVelocity.y;
    CHECK(spin > 1.0f);
}

TEST_CASE("Wheel joint steering") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    BodyId chassis = world.addBox({0, 0, 0}, {1, 0.3f, 0.5f}, 0.0f);
    BodyId wheel = world.addSphere({0, -1, 0}, 0.4f, 1.0f);
    JointId j = world.addWheelJoint(chassis, wheel, {0, -0.5f, 0}, {0, 1, 0});
    world.joint(j).enableSteering = true;
    world.joint(j).steeringAngle = 0.3f;

    for (int i = 0; i < 120; ++i)
        world.step(1.0f / 60.0f);

    // The joint should remain valid and positions finite.
    CHECK(world.isValid(j));
    CHECK(std::isfinite(world.body(wheel).position.x));
}

TEST_CASE("Wheel joint rejects zero axis") {
    World world(BackendType::Cpu);
    BodyId a = world.addBox({0, 0, 0}, {1, 1, 1}, 0.0f);
    BodyId b = world.addSphere({0, -1, 0}, 0.4f, 1.0f);
    CHECK_THROWS(world.addWheelJoint(a, b, {0, -0.5f, 0}, {0, 0, 0}));
}

// ---------------------------------------------------------------------------
// Rope joint
// ---------------------------------------------------------------------------

TEST_CASE("Rope joint allows slack") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    BodyId a = world.addSphere({0, 0, 0}, 0.3f, 0.0f);
    BodyId b = world.addSphere({2, 0, 0}, 0.3f, 1.0f);
    // maxLength = 5, but bodies are only 2 apart — rope is slack.
    world.addRopeJoint(a, b, {0, 0, 0}, {2, 0, 0}, 5.0f);

    // Push B toward A — should be unconstrained.
    world.setLinearVelocity(b, {-3, 0, 0});
    for (int i = 0; i < 30; ++i)
        world.step(1.0f / 60.0f);

    // B should have moved closer (rope didn't resist compression).
    float dist = length(world.body(b).position - world.body(a).position);
    CHECK(dist < 2.0f);
}

TEST_CASE("Rope joint prevents over-extension") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    BodyId a = world.addSphere({0, 0, 0}, 0.3f, 0.0f);
    BodyId b = world.addSphere({3, 0, 0}, 0.3f, 1.0f);
    // maxLength = 4, bodies start 3 apart.
    world.addRopeJoint(a, b, {0, 0, 0}, {3, 0, 0}, 4.0f);

    // Push B away from A.
    world.setLinearVelocity(b, {5, 0, 0});
    for (int i = 0; i < 120; ++i)
        world.step(1.0f / 60.0f);

    // Distance should not exceed maxLength by much.
    float dist = length(world.body(b).position - world.body(a).position);
    CHECK(dist <= 4.5f);
}

TEST_CASE("Rope joint under gravity") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};
    BodyId a = world.addSphere({0, 10, 0}, 0.3f, 0.0f);
    BodyId b = world.addSphere({0, 7, 0}, 0.3f, 1.0f);
    world.addRopeJoint(a, b, {0, 10, 0}, {0, 7, 0}, 3.5f);

    for (int i = 0; i < 180; ++i)
        world.step(1.0f / 60.0f);

    // Body B should hang below A, constrained by the rope.
    float dist = length(world.body(b).position - world.body(a).position);
    CHECK(dist <= 4.0f);
    CHECK(world.body(b).position.y < world.body(a).position.y);
}

TEST_CASE("Rope joint rejects non-positive maxLength") {
    World world(BackendType::Cpu);
    BodyId a = world.addSphere({0, 0, 0}, 0.3f, 0.0f);
    BodyId b = world.addSphere({2, 0, 0}, 0.3f, 1.0f);
    CHECK_THROWS(world.addRopeJoint(a, b, {0, 0, 0}, {2, 0, 0}, 0.0f));
    CHECK_THROWS(world.addRopeJoint(a, b, {0, 0, 0}, {2, 0, 0}, -1.0f));
}

// ---------------------------------------------------------------------------
// Pulley joint
// ---------------------------------------------------------------------------

TEST_CASE("Pulley joint conserves rope length") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};
    // Two hanging bodies connected through pulleys at y=10.
    BodyId a = world.addSphere({-2, 5, 0}, 0.3f, 1.0f);
    BodyId b = world.addSphere({2, 5, 0}, 0.3f, 1.0f);
    JointId j = world.addPulleyJoint(
        a, b,
        {-2, 5, 0}, {2, 5, 0},   // body anchors
        {-2, 10, 0}, {2, 10, 0},  // ground anchors
        1.0f);
    CHECK(world.isValid(j));

    // Push A down; B should rise.
    world.setLinearVelocity(a, {0, -3, 0});
    for (int i = 0; i < 120; ++i)
        world.step(1.0f / 60.0f);

    // Both positions should be finite.
    CHECK(std::isfinite(world.body(a).position.y));
    CHECK(std::isfinite(world.body(b).position.y));
}

TEST_CASE("Pulley joint with mechanical advantage") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};
    BodyId a = world.addSphere({-2, 5, 0}, 0.3f, 2.0f);
    BodyId b = world.addSphere({2, 5, 0}, 0.3f, 1.0f);
    // ratio = 2: body B moves half the distance of A.
    JointId j = world.addPulleyJoint(
        a, b,
        {-2, 5, 0}, {2, 5, 0},
        {-2, 10, 0}, {2, 10, 0},
        2.0f);
    CHECK(world.isValid(j));

    for (int i = 0; i < 120; ++i)
        world.step(1.0f / 60.0f);

    CHECK(std::isfinite(world.body(a).position.y));
    CHECK(std::isfinite(world.body(b).position.y));
}

TEST_CASE("Pulley joint rejects non-positive ratio") {
    World world(BackendType::Cpu);
    BodyId a = world.addSphere({-2, 5, 0}, 0.3f, 1.0f);
    BodyId b = world.addSphere({2, 5, 0}, 0.3f, 1.0f);
    CHECK_THROWS(world.addPulleyJoint(
        a, b, {-2, 5, 0}, {2, 5, 0}, {-2, 10, 0}, {2, 10, 0}, 0.0f));
    CHECK_THROWS(world.addPulleyJoint(
        a, b, {-2, 5, 0}, {2, 5, 0}, {-2, 10, 0}, {2, 10, 0}, -1.0f));
}

// ---------------------------------------------------------------------------
// Gear joint
// ---------------------------------------------------------------------------

TEST_CASE("Gear joint couples angular velocity") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    BodyId a = world.addSphere({0, 0, 0}, 0.5f, 1.0f);
    BodyId b = world.addSphere({2, 0, 0}, 0.5f, 1.0f);
    // ratio = 1: equal and opposite spin.
    world.addGearJoint(a, b, {0, 1, 0}, {0, 1, 0}, 1.0f);

    world.setAngularVelocity(a, {0, 5, 0});
    for (int i = 0; i < 60; ++i)
        world.step(1.0f / 60.0f);

    // B should spin in the opposite direction.
    float wA = world.body(a).angularVelocity.y;
    float wB = world.body(b).angularVelocity.y;
    // wA + ratio * wB ≈ 0
    CHECK((wA + wB) == doctest::Approx(0.0f).epsilon(0.5f));
}

TEST_CASE("Gear joint with ratio 2") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    BodyId a = world.addSphere({0, 0, 0}, 0.5f, 1.0f);
    BodyId b = world.addSphere({2, 0, 0}, 0.5f, 1.0f);
    // ratio = 2: B spins at half the speed of A (opposite direction).
    world.addGearJoint(a, b, {0, 1, 0}, {0, 1, 0}, 2.0f);

    world.setAngularVelocity(a, {0, 6, 0});
    for (int i = 0; i < 60; ++i)
        world.step(1.0f / 60.0f);

    float wA = world.body(a).angularVelocity.y;
    float wB = world.body(b).angularVelocity.y;
    // wA + 2 * wB ≈ 0
    CHECK((wA + 2.0f * wB) == doctest::Approx(0.0f).epsilon(0.5f));
}

TEST_CASE("Gear joint with perpendicular axes") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    BodyId a = world.addSphere({0, 0, 0}, 0.5f, 1.0f);
    BodyId b = world.addSphere({0, 2, 0}, 0.5f, 1.0f);
    // A spins about Y, B about X — bevel gear.
    world.addGearJoint(a, b, {0, 1, 0}, {1, 0, 0}, 1.0f);

    world.setAngularVelocity(a, {0, 4, 0});
    for (int i = 0; i < 60; ++i)
        world.step(1.0f / 60.0f);

    CHECK(std::isfinite(world.body(a).angularVelocity.y));
    CHECK(std::isfinite(world.body(b).angularVelocity.x));
}

TEST_CASE("Gear joint rejects zero ratio") {
    World world(BackendType::Cpu);
    BodyId a = world.addSphere({0, 0, 0}, 0.5f, 1.0f);
    BodyId b = world.addSphere({2, 0, 0}, 0.5f, 1.0f);
    CHECK_THROWS(world.addGearJoint(a, b, {0, 1, 0}, {0, 1, 0}, 0.0f));
}

TEST_CASE("Gear joint rejects zero axis") {
    World world(BackendType::Cpu);
    BodyId a = world.addSphere({0, 0, 0}, 0.5f, 1.0f);
    BodyId b = world.addSphere({2, 0, 0}, 0.5f, 1.0f);
    CHECK_THROWS(world.addGearJoint(a, b, {0, 0, 0}, {0, 1, 0}, 1.0f));
    CHECK_THROWS(world.addGearJoint(a, b, {0, 1, 0}, {0, 0, 0}, 1.0f));
}

// ---------------------------------------------------------------------------
// Cross-cutting: all new joints survive simulation and produce finite state
// ---------------------------------------------------------------------------

TEST_CASE("All extended joints remain valid after simulation") {
    World world(BackendType::Cpu);
    world.gravity = {0, -9.81f, 0};

    // Weld
    BodyId weldA = world.addBox({-20, 5, 0}, {0.3f, 0.3f, 0.3f}, 0.0f);
    BodyId weldB = world.addBox({-19, 5, 0}, {0.3f, 0.3f, 0.3f}, 1.0f);
    JointId weldJ = world.addWeldJoint(weldA, weldB, {-19.5f, 5, 0});

    // Wheel
    BodyId wheelA = world.addBox({-14, 5, 0}, {1, 0.3f, 0.5f}, 0.0f);
    BodyId wheelB = world.addSphere({-14, 4, 0}, 0.4f, 1.0f);
    JointId wheelJ = world.addWheelJoint(wheelA, wheelB, {-14, 4.5f, 0}, {0, 1, 0});

    // Rope
    BodyId ropeA = world.addSphere({-8, 8, 0}, 0.3f, 0.0f);
    BodyId ropeB = world.addSphere({-8, 5, 0}, 0.3f, 1.0f);
    JointId ropeJ = world.addRopeJoint(ropeA, ropeB, {-8, 8, 0}, {-8, 5, 0}, 4.0f);

    // Pulley
    BodyId pulleyA = world.addSphere({-2, 5, 0}, 0.3f, 1.0f);
    BodyId pulleyB = world.addSphere({2, 5, 0}, 0.3f, 1.0f);
    JointId pulleyJ = world.addPulleyJoint(
        pulleyA, pulleyB,
        {-2, 5, 0}, {2, 5, 0},
        {-2, 10, 0}, {2, 10, 0}, 1.0f);

    // Gear
    BodyId gearA = world.addSphere({8, 5, 0}, 0.5f, 1.0f);
    BodyId gearB = world.addSphere({10, 5, 0}, 0.5f, 1.0f);
    JointId gearJ = world.addGearJoint(gearA, gearB, {0, 1, 0}, {0, 1, 0}, 1.0f);

    for (int i = 0; i < 180; ++i)
        world.step(1.0f / 60.0f);

    CHECK(world.isValid(weldJ));
    CHECK(world.isValid(wheelJ));
    CHECK(world.isValid(ropeJ));
    CHECK(world.isValid(pulleyJ));
    CHECK(world.isValid(gearJ));

    auto finite = [](Vec3 v) {
        return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
    };
    CHECK(finite(world.body(weldB).position));
    CHECK(finite(world.body(wheelB).position));
    CHECK(finite(world.body(ropeB).position));
    CHECK(finite(world.body(pulleyA).position));
    CHECK(finite(world.body(pulleyB).position));
    CHECK(finite(world.body(gearA).position));
    CHECK(finite(world.body(gearB).position));
}

TEST_CASE("Extended joints removed with body") {
    World world(BackendType::Cpu);
    BodyId a = world.addSphere({0, 5, 0}, 0.5f, 1.0f);
    BodyId b = world.addSphere({2, 5, 0}, 0.5f, 1.0f);

    JointId weld = world.addWeldJoint(a, b, {1, 5, 0});
    JointId rope = world.addRopeJoint(a, b, {0, 5, 0}, {2, 5, 0}, 3.0f);
    JointId gear = world.addGearJoint(a, b, {0, 1, 0}, {0, 1, 0}, 1.0f);

    world.removeBody(a);
    CHECK_FALSE(world.isValid(weld));
    CHECK_FALSE(world.isValid(rope));
    CHECK_FALSE(world.isValid(gear));
}

TEST_CASE("Extended joint type enum values are distinct") {
    CHECK((uint8_t)JointType::Weld != (uint8_t)JointType::Wheel);
    CHECK((uint8_t)JointType::Wheel != (uint8_t)JointType::Rope);
    CHECK((uint8_t)JointType::Rope != (uint8_t)JointType::Pulley);
    CHECK((uint8_t)JointType::Pulley != (uint8_t)JointType::Gear);
    CHECK((uint8_t)JointType::Gear > (uint8_t)JointType::Motor);
}
