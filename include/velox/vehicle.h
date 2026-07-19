#pragma once
// Raycast vehicle (roadmap 12): the chassis is a regular dynamic Body; each
// wheel is a virtual suspension that raycasts to the ground, applies
// spring/damper force at the hub, and slip-based tire forces at the contact.
// Call Step(dt) each frame BEFORE World::step so the forces enter that step.
#include "world.h"

#include <array>
#include <vector>

namespace velox {

enum class DrivetrainType : uint8_t { RWD, FWD, AWD };

struct VehicleConfig {
    float chassisMass = 1500.0f;
    Vec3 chassisHalfExtents{1.0f, 0.5f, 2.2f}; // x width, y height, z length
    DrivetrainType drivetrain = DrivetrainType::RWD;
    float engineMaxTorque = 500.0f;   // N*m at the crank
    float engineMaxRPM = 8000.0f;
    float engineIdleRPM = 900.0f;
    std::array<float, 6> gearRatios{3.5f, 2.2f, 1.6f, 1.2f, 0.9f, 0.75f};
    float finalDriveRatio = 3.5f;
    float shiftUpRPM = 7500.0f;
    float shiftDownRPM = 2500.0f;
    bool enableAntiRoll = true;
    float antiRollStiffness = 8000.0f; // N per meter of compression difference

    // Real-car suspension character, used by AddDefaultWheels to derive each
    // wheel's spring stiffness and damping from the sprung corner mass:
    //   k = m_corner * (2*pi*f)^2,   c = 2 * zeta * sqrt(k * m_corner)
    // Road cars ride around 1.1-1.6 Hz with damping ratios near 0.4-0.5;
    // higher values feel sportier/stiffer.
    float suspensionFrequencyHz = 1.35f;
    float suspensionDampingRatio = 0.45f;

    // Suspension roll-center model: lateral tire force is applied this
    // fraction of the way up from the contact patch toward the chassis
    // center. 0 = at the ground (maximum body roll and easy rollover for a
    // tall CG), 1 = at the CG (no roll at all). Real suspension geometry
    // places the roll center between the two; ~0.6 feels like a road car.
    float lateralRollCenterFactor = 0.6f;

    // Sporty street-tire grip for AddDefaultWheels (WheelConfig's own
    // default 1.8 is racing slick territory).
    float defaultTireFriction = 1.4f;
};

struct WheelConfig {
    Vec3 localPosition{};                 // hub position in chassis frame
    Vec3 direction{0.0f, -1.0f, 0.0f};    // suspension travel, chassis frame
    float radius = 0.35f;
    float suspensionRestLength = 0.35f;
    float suspensionStiffness = 45000.0f; // N/m
    float suspensionDamping = 4500.0f;    // N*s/m
    float suspensionCompression = 0.12f;  // max travel above rest (m)
    float suspensionExtension = 0.25f;    // max travel below rest (m)
    float lateralFriction = 1.8f;
    float longitudinalFriction = 1.8f;
    bool steerable = false;
    bool driven = false;  // overridden by VehicleConfig::drivetrain in Step
    bool handbrake = false;
};

// Per-wheel state readable after Step (telemetry / rendering).
struct WheelState {
    bool grounded = false;
    float compression = 0.0f;     // meters above full extension
    float suspensionForce = 0.0f; // N along the suspension axis
    float spinVelocity = 0.0f;    // rad/s about the axle
    float rotation = 0.0f;        // accumulated angle for rendering
    Vec3 contactPoint{};
    Vec3 contactNormal{};
    BodyId contactBody{};
};

class Vehicle {
public:
    // Creates the chassis body at `position`. The chassis is a normal
    // dynamic body: it collides, sleeps (Step wakes it while driving), and
    // can be removed with World::removeBody(chassis()).
    Vehicle(World& world, const VehicleConfig& config, Vec3 position);

    // Standard 4-wheel arrangement helper (front pair steerable; driven
    // flags follow the configured drivetrain).
    void AddDefaultWheels();
    void AddWheel(const WheelConfig& config);

    BodyId chassis() const { return chassis_; }
    size_t wheelCount() const { return wheels_.size(); }
    const WheelState& wheelState(size_t index) const { return states_[index]; }
    const WheelConfig& wheelConfig(size_t index) const { return wheels_[index]; }

    void SetThrottle(float value) { throttle_ = value; }   // 0..1
    void SetBrake(float value) { brake_ = value; }         // 0..1
    void SetSteering(float radians) { steering_ = radians; } // + = left
    float steeringAngle() const { return steering_; }
    int currentGear() const { return gear_; }
    float engineRPM() const { return rpm_; }
    float forwardSpeed() const;

    // Computes suspension, tire, drivetrain, and anti-roll forces from the
    // current world state and accumulates them on the chassis. Call once per
    // frame before World::step(dt).
    void Step(float dt);

private:
    World& world_;
    VehicleConfig config_;
    BodyId chassis_;
    std::vector<WheelConfig> wheels_;
    std::vector<WheelState> states_;
    float throttle_ = 0.0f, brake_ = 0.0f, steering_ = 0.0f;
    int gear_ = 1;      // 1-based; 1..gearRatios.size()
    float rpm_ = 900.0f;
    float shiftCooldown_ = 0.0f;
};

} // namespace velox
