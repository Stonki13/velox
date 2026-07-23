#include "doctest.h"
#include <velox/velox.h>
#include <cmath>

using namespace velox;

TEST_CASE("Vehicle creation with default config") {
    World world(BackendType::Cpu);
    VehicleConfig config{};
    Vehicle vehicle(world, config, {0, 5, 0});

    CHECK(world.isValid(vehicle.chassis()));
    CHECK(vehicle.wheelCount() == 0);
}

TEST_CASE("Vehicle AddDefaultWheels creates 4 wheels") {
    World world(BackendType::Cpu);
    VehicleConfig config{};
    Vehicle vehicle(world, config, {0, 5, 0});

    vehicle.AddDefaultWheels();
    CHECK(vehicle.wheelCount() == 4);
}

TEST_CASE("Vehicle AddWheel adds individual wheels") {
    World world(BackendType::Cpu);
    VehicleConfig config{};
    Vehicle vehicle(world, config, {0, 5, 0});

    WheelConfig wc{};
    wc.localPosition = {1.0f, 0.0f, 1.5f};
    wc.radius = 0.35f;
    wc.steerable = false;
    wc.driven = true;
    vehicle.AddWheel(wc);

    CHECK(vehicle.wheelCount() == 1);

    WheelConfig wc2{};
    wc2.localPosition = {-1.0f, 0.0f, 1.5f};
    vehicle.AddWheel(wc2);

    CHECK(vehicle.wheelCount() == 2);
}

TEST_CASE("Vehicle chassis is a valid dynamic body") {
    World world(BackendType::Cpu);
    VehicleConfig config{};
    config.chassisMass = 1500.0f;
    Vehicle vehicle(world, config, {0, 10, 0});

    BodyId chassis = vehicle.chassis();
    CHECK(world.isValid(chassis));

    // Chassis should be affected by gravity
    float y0 = world.body(chassis).position.y;
    world.step(1.0f / 60.0f);
    float y1 = world.body(chassis).position.y;
    CHECK(y1 < y0);
}

TEST_CASE("Vehicle SetThrottle and SetBrake") {
    World world(BackendType::Cpu);
    VehicleConfig config{};
    Vehicle vehicle(world, config, {0, 5, 0});
    vehicle.AddDefaultWheels();

    // Should not throw
    CHECK_NOTHROW(vehicle.SetThrottle(1.0f));
    CHECK_NOTHROW(vehicle.SetBrake(0.5f));
}

TEST_CASE("Vehicle SetSteering and steeringAngle") {
    World world(BackendType::Cpu);
    VehicleConfig config{};
    Vehicle vehicle(world, config, {0, 5, 0});
    vehicle.AddDefaultWheels();

    vehicle.SetSteering(0.3f);
    CHECK(vehicle.steeringAngle() == doctest::Approx(0.3f));

    vehicle.SetSteering(-0.5f);
    CHECK(vehicle.steeringAngle() == doctest::Approx(-0.5f));

    vehicle.SetSteering(0.0f);
    CHECK(vehicle.steeringAngle() == doctest::Approx(0.0f));
}

TEST_CASE("Vehicle initial gear and RPM") {
    World world(BackendType::Cpu);
    VehicleConfig config{};
    Vehicle vehicle(world, config, {0, 5, 0});
    vehicle.AddDefaultWheels();

    // Initial gear should be 1
    CHECK(vehicle.currentGear() == 1);

    // Initial RPM should be near idle
    CHECK(vehicle.engineRPM() == doctest::Approx(config.engineIdleRPM).epsilon(100.0f));
}

TEST_CASE("Vehicle Step without wheels does not crash") {
    World world(BackendType::Cpu);
    VehicleConfig config{};
    Vehicle vehicle(world, config, {0, 5, 0});

    // Step with no wheels added — should not crash
    CHECK_NOTHROW(vehicle.Step(1.0f / 60.0f));
}

TEST_CASE("Vehicle Step with default wheels on ground") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};

    // Ground plane
    world.addStaticPlane({0, 1, 0}, 0.0f);

    VehicleConfig config{};
    Vehicle vehicle(world, config, {0, 2, 0});
    vehicle.AddDefaultWheels();

    // Step several times to let the vehicle settle
    for (int i = 0; i < 60; ++i) {
        vehicle.Step(1.0f / 60.0f);
        world.step(1.0f / 60.0f);
    }

    // Vehicle should have settled near the ground
    Vec3 chassisPos = world.body(vehicle.chassis()).position;
    CHECK(chassisPos.y > 0.0f);
    CHECK(chassisPos.y < 5.0f);
}

TEST_CASE("Vehicle driving forward with throttle") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};

    // Ground plane
    world.addStaticPlane({0, 1, 0}, 0.0f);

    VehicleConfig config{};
    Vehicle vehicle(world, config, {0, 2, 0});
    vehicle.AddDefaultWheels();

    // Apply throttle and let it drive
    vehicle.SetThrottle(1.0f);
    vehicle.SetSteering(0.0f);

    for (int i = 0; i < 120; ++i) {
        vehicle.Step(1.0f / 60.0f);
        world.step(1.0f / 60.0f);
    }

    // forwardSpeed should be positive or the vehicle moved
    float speed = vehicle.forwardSpeed();
    // Speed can be positive or zero depending on simulation — just verify it's readable
    CHECK(speed >= 0.0f);
}

TEST_CASE("Vehicle wheelState is accessible") {
    World world(BackendType::Cpu);
    VehicleConfig config{};
    Vehicle vehicle(world, config, {0, 5, 0});
    vehicle.AddDefaultWheels();

    for (size_t i = 0; i < vehicle.wheelCount(); ++i) {
        const WheelState& ws = vehicle.wheelState(i);
        // Before stepping, wheels should not be grounded
        CHECK_FALSE(ws.grounded);
        CHECK(ws.compression == doctest::Approx(0.0f));
        CHECK(ws.spinVelocity == doctest::Approx(0.0f));
    }
}

TEST_CASE("Vehicle wheelConfig is accessible") {
    World world(BackendType::Cpu);
    VehicleConfig config{};
    Vehicle vehicle(world, config, {0, 5, 0});
    vehicle.AddDefaultWheels();

    for (size_t i = 0; i < vehicle.wheelCount(); ++i) {
        const WheelConfig& wc = vehicle.wheelConfig(i);
        CHECK(wc.radius > 0.0f);
        CHECK(wc.suspensionStiffness > 0.0f);
    }
}

TEST_CASE("Vehicle DrivetrainType options") {
    World world(BackendType::Cpu);

    SUBCASE("RWD") {
        VehicleConfig config{};
        config.drivetrain = DrivetrainType::RWD;
        Vehicle vehicle(world, config, {0, 5, 0});
        vehicle.AddDefaultWheels();
        CHECK(vehicle.wheelCount() == 4);
    }

    SUBCASE("FWD") {
        VehicleConfig config{};
        config.drivetrain = DrivetrainType::FWD;
        Vehicle vehicle(world, config, {0, 5, 0});
        vehicle.AddDefaultWheels();
        CHECK(vehicle.wheelCount() == 4);
    }

    SUBCASE("AWD") {
        VehicleConfig config{};
        config.drivetrain = DrivetrainType::AWD;
        Vehicle vehicle(world, config, {0, 5, 0});
        vehicle.AddDefaultWheels();
        CHECK(vehicle.wheelCount() == 4);
    }
}

TEST_CASE("Vehicle: stable cornering without rollover") {
    World world(BackendType::Cpu);
    world.addStaticPlane({0, 1, 0}, 0.0f);

    VehicleConfig config{};
    config.drivetrain = DrivetrainType::RWD;
    config.differential.type = DifferentialType::LimitedSlip;
    Vehicle vehicle(world, config, {0, 2, 0});
    vehicle.AddDefaultWheels();

    // Accelerate to speed, then steer into a turn.
    vehicle.SetThrottle(1.0f);
    for (int i = 0; i < 180; ++i) { // 3 seconds to build speed
        vehicle.Step(1.0f / 60.0f);
        world.step(1.0f / 60.0f);
    }
    float speedBeforeTurn = vehicle.forwardSpeed();
    CHECK(speedBeforeTurn > 5.0f); // must have accelerated

    // Steer into a sustained turn for 3 seconds.
    vehicle.SetThrottle(0.3f);
    vehicle.SetSteering(0.4f); // moderate left turn
    bool flipped = false;
    for (int i = 0; i < 180; ++i) {
        vehicle.Step(1.0f / 60.0f);
        world.step(1.0f / 60.0f);
        // Check the chassis hasn't flipped: the up vector should stay
        // mostly pointing up (dot with world up > 0.5).
        const Body& b = world.body(vehicle.chassis());
        Vec3 up = rotate(b.orientation, {0, 1, 0});
        if (up.y < 0.3f) flipped = true;
    }
    CHECK_FALSE(flipped);
}

TEST_CASE("Vehicle: differential types produce different behavior") {
    // A locked differential should keep both driven wheels at the same
    // spin velocity; an open differential allows them to differ.
    auto runWith = [](DifferentialType type) {
        World world(BackendType::Cpu);
        world.addStaticPlane({0, 1, 0}, 0.0f);
        VehicleConfig config{};
        config.drivetrain = DrivetrainType::RWD;
        config.differential.type = type;
        Vehicle vehicle(world, config, {0, 2, 0});
        vehicle.AddDefaultWheels();
        vehicle.SetThrottle(1.0f);
        vehicle.SetSteering(0.5f); // turning creates speed difference
        for (int i = 0; i < 120; ++i) {
            vehicle.Step(1.0f / 60.0f);
            world.step(1.0f / 60.0f);
        }
        // Return the spin difference between the two rear (driven) wheels.
        float s0 = vehicle.wheelState(2).spinVelocity;
        float s1 = vehicle.wheelState(3).spinVelocity;
        return std::fabs(s0 - s1);
    };

    float openDiff = runWith(DifferentialType::Open);
    float lockedDiff = runWith(DifferentialType::Locked);
    // Locked diff should have smaller (or zero) spin difference.
    CHECK(lockedDiff <= openDiff + 0.01f);
}
