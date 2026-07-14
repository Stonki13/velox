// vehicle.h — Design sketch for raycast vehicle model with suspension and drivetrain.
// Self-contained: includes only what's needed for compilation in isolation.
// TODO: integrate with velox::World, velox::BodyId, velox::Vec3, velox::Quat.

#pragma once

#include <cstdint>
#include <vector>
#include <array>
#include <functional>

// Forward declarations — replace with #include "velox/world.h" etc. when integrating.
namespace velox {
    struct Vec3;
    struct Quat;
    class World;
    using BodyId = uint64_t;
}

namespace velox {

// Tire friction model: maps slip ratio and slip angle to lateral/longitudinal force.
// The default is a simplified Pacejka "magic formula" approximation. Users can
// override with a custom function for realistic tire data.
using TireFrictionCurve = std::function<float(float slipRatio, float slipAngle, float normalLoad)>;

// Suspension definition for one wheel.
struct WheelSuspension {
    float restLength = 0.3f;          // meters: distance from hub to ground at rest
    float maxCompression = 0.1f;      // meters: max travel downward
    float maxExtension = 0.2f;        // meters: max travel upward (extension)
    float springFrequencyHz = 5.0f;   // Hz: natural frequency of the suspension
    float dampingRatio = 0.7f;        // critical damping ratio (0=undamped, 1=critical)
    float antiRollStiffness = 0.0f;   // N*m/rad: roll bar stiffness (0 = no anti-roll)
};

// Wheel definition: position relative to chassis, suspension config, tire params.
struct WheelDesc {
    Vec3 localPosition{};             // hub position in chassis body frame
    Vec3 wheelDirection{-0.1f, -1.0f, 0.0f};  // direction the wheel points (normalized)
    Vec3 wheelAxle{0.0f, 0.0f, -1.0f};       // axle axis (rotation axis for steering)
    float wheelRadius = 0.3f;         // meters
    float wheelHalfWidth = 0.15f;     // meters: tire width / 2
    float springFrequencyHz = 5.0f;   // override per-wheel suspension frequency
    float dampingRatio = 0.7f;        // override per-wheel damping
    TireFrictionCurve frictionCurve;  // custom friction model (null = default Pacejka)
};

// Drivetrain configuration.
struct DrivetrainConfig {
    enum class Type { RWD, FWD, AWD } type = Type::RWD;
    float engineMaxTorque = 500.0f;       // N*m: peak engine torque
    float engineMaxRPM = 8000.0f;         // RPM: redline
    std::array<float, 6> gearRatios = {3.5f, 2.2f, 1.6f, 1.2f, 0.9f, 0.75f};
    float finalDriveRatio = 3.5f;         // differential final drive ratio
    float clutchTorqueLimit = 1000.0f;    // N*m: max torque through clutch
    bool enableAntiRollBar = true;        // apply anti-roll forces on cornering
};

// Per-wheel raycast result from the suspension query.
struct WheelRaycastResult {
    bool hit = false;                     // ground detected below the wheel hub
    float distance = 0.0f;                // distance from hub to ground along wheelDirection
    Vec3 contactPoint{};                  // world-space point where ray hits ground
    Vec3 groundNormal{};                  // normal at the contact point
    float suspensionCompression = 0.0f;   // how much the spring is compressed (meters)
    float normalForce = 0.0f;             // upward force from the ground (Newtons)
    float longitudinalSlip = 0.0f;        // slip ratio (-1 to 1, negative = braking)
    float lateralSlip = 0.0f;             // slip angle in radians
    Vec3 longitudinalForce{};             // force along wheel direction from tire
    Vec3 lateralForce{};                  // force perpendicular to wheel direction
};

// Vehicle state: updated each frame by the vehicle solver.
struct VehicleState {
    float engineRPM = 0.0f;               // current engine RPM
    int currentGear = 0;                  // 0=reverse, 1-6=fwd gears
    float throttle = 0.0f;                // 0 to 1: pedal position
    float brakeTorque = 0.0f;             // N*m: braking torque applied (0 = no brake)
    float steeringAngle = 0.0f;           // radians: front wheel steering angle
    std::array<WheelRaycastResult, 4> wheels{};  // one per wheel slot
};

// Raycast vehicle: chassis body + suspended wheels with tire physics.
// The vehicle owns the chassis BodyId and manages wheel raycasts internally.
class Vehicle {
public:
    // Construct a vehicle attached to a chassis body in the given world.
    // The chassis must be a dynamic body; the vehicle adds suspension forces each step.
    //
    // TODO: validate that the chassis body exists and is dynamic; throw on invalid.
    Vehicle(World& world, BodyId chassisBody, const DrivetrainConfig& drivetrain);

    // Add a wheel at the given local position with the given suspension config.
    // Returns the wheel index (0-based). Max 4 wheels supported.
    //
    // TODO: validate wheel count ≤ 4; throw on overflow.
    int AddWheel(const WheelDesc& desc);

    // Step the vehicle simulation: raycast each wheel, compute suspension forces,
    // apply tire forces, update engine/gear state, apply anti-roll if enabled.
    // Must be called once per world step(), after velox::World::step() returns.
    //
    // TODO: implement the full vehicle solver loop:
    //   1. Raycast each wheel along its direction to find ground contact
    //   2. Compute suspension compression/extension from ray distance
    //   3. Apply spring + damper forces at each hub
    //   4. Compute tire slip ratios/angles from wheel velocity vs ground velocity
    //   5. Look up lateral/longitudinal forces from friction curve
    //   6. Apply tire forces at contact points on the chassis
    //   7. Update engine RPM based on average wheel speed and gear ratio
    //   8. Auto-shift gears based on RPM thresholds
    //   9. If anti-roll enabled, compute roll moment and apply counter-torque
    void Step(VehicleState& outState);

    // Set throttle input (0-1). Called by the user each frame before Step().
    void SetThrottle(float value) { throttle_ = vclamp(value, 0.0f, 1.0f); }

    // Set brake torque (N*m). 0 = no braking. Max depends on brake config.
    void SetBrake(float torque) { brakeTorque_ = vclamp(torque, 0.0f, 5000.0f); }

    // Set steering angle in radians (-π to π). Front wheels only.
    void SetSteering(float angle) { steeringAngle_ = vclamp(angle, -1.57f, 1.57f); }

    // Manually set the gear (0=reverse, 1-6=fwd). Auto-shift overrides this.
    void SetGear(int gear) { currentGear_ = vclamp(gear, -1, 6); }

    // Disable auto-shift and let the user control gears manually.
    void EnableManualTransmission(bool enable) { manualMode_ = enable; }

    // Get the current chassis velocity in world space (useful for speedometer).
    Vec3 GetChassisVelocity() const;

    // Get the current engine RPM.
    float GetEngineRPM() const;

private:
    World& world_;
    BodyId chassisBody_;
    DrivetrainConfig drivetrain_;
    std::array<WheelDesc, 4> wheels_{};
    int wheelCount_ = 0;

    // User inputs (updated each frame).
    float throttle_ = 0.0f;
    float brakeTorque_ = 0.0f;
    float steeringAngle_ = 0.0f;
    int currentGear_ = 0;
    bool manualMode_ = false;

    // Internal state.
    float engineRPM_ = 0.0f;
};

} // namespace velox
