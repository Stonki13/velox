# 12 — Vehicle Model

## Goal

Implement a raycast vehicle model: the chassis is a Velox Body, and each wheel is a simulated suspension that raycasts to the ground each frame, applies spring/damper forces at the hub, and computes tire forces from slip ratios using a Pacejka-style friction curve. Supports RWD/FWD/AWD drivetrains with gear shifting and anti-roll bars.

## Public API

```cpp
namespace velox {

struct VehicleConfig {
    float chassisMass = 1500.0f;
    Vec3 chassisHalfExtents{1.0f, 0.5f, 2.2f};
    DrivetrainType drivetrain = DrivetrainType::RWD;
    float engineMaxTorque = 500.0f;
    float engineMaxRPM = 8000.0f;
    std::array<float, 6> gearRatios{3.5f, 2.2f, 1.6f, 1.2f, 0.9f, 0.75f};
    float finalDriveRatio = 3.5f;
    bool enableAntiRoll = true;
};

struct WheelConfig {
    Vec3 localPosition{};       // hub position in chassis frame
    Vec3 direction{-0.1f, -1.0f, 0.0f};  // wheel pointing direction
    float radius = 0.3f;
    float width = 0.2f;
    float suspensionRestLength = 0.3f;
    float suspensionStiffness = 50000.0f;
    float suspensionDamping = 5000.0f;
    float suspensionCompression = 0.1f;
    float suspensionExtension = 0.2f;
    float lateralFriction = 1.8f;
    float longitudinalFriction = 1.8f;
};

class Vehicle {
public:
    Vehicle(World& world, const VehicleConfig& config);
    void AddWheel(const WheelConfig& config, BodyId chassisBody);
    void SetThrottle(float t) { throttle_ = t; }
    void SetBrake(float b) { brakeTorque_ = b; }
    void SetSteering(float angle) { steeringAngle_ = angle; }
    void Step(World& world, float dt);  // call after world.step()

private:
    World& world_;
    VehicleConfig config_;
    std::vector<WheelConfig> wheels_;
    float throttle_ = 0.0f, brakeTorque_ = 0.0f, steeringAngle_ = 0.0f;
};

} // namespace velox
```

## Data structures

- `VehicleConfig`, `WheelConfig` — new file `include/velox/vehicle.h`.
- `Vehicle` class — new file `include/velox/vehicle.h`, impl in `src/vehicle.cpp`.

## Algorithm

1. **Raycast each wheel:** cast a ray from the hub along `-direction` for `suspensionRestLength + suspensionCompression` meters. Record hit distance, contact point, ground normal.
2. **Compute suspension compression:** `compression = restLength - hitDistance`. Clamp to [-suspensionExtension, suspensionCompression].
3. **Apply spring/damper force:** `F = -stiffness * compression - damping * velocity`. Apply at the hub as a force on the chassis body.
4. **Compute tire slip:** longitudinal slip = `(wheelSurfaceSpeed - groundSpeed) / max(wheelSurfaceSpeed, 1.0)`. Lateral slip from侧向 velocity component.
5. **Look up friction forces:** `F_long = frictionCurve(longitudinalSlip) * normalLoad`, same for lateral. Apply at the contact point.
6. **Drivetrain:** engine torque × gear ratio × final drive × clutch efficiency = wheel torque. Distribute to driven wheels based on DrivetrainType.
7. **Anti-roll:** if `enableAntiRoll`, compute roll moment from lateral acceleration and apply counter-torque to chassis.

## Files

- `include/velox/vehicle.h` — new header
- `src/vehicle.cpp` — new source file

## Tests

1. **Static hang:** Vehicle on flat ground, no throttle. All 4 wheels report correct normal force (~375 kg each for 1500 kg chassis).
2. **Acceleration:** Throttle=1.0 in 1st gear for 5 seconds. Vehicle accelerates; RPM rises to redline and shifts to 2nd gear.
3. **Braking:** Vehicle at 30 m/s, brakeTorque=2000 N*m. Stops within 50 m on dry asphalt (friction=1.8).
4. **Cornering:** Vehicle at 20 m/s turning left. Anti-roll bar reduces body roll by ≥ 30% compared to no anti-roll.

## Acceptance

- [ ] Raycast suspension computes correct compression and normal force on flat ground
- [ ] Drivetrain applies torque to driven wheels only (RWD: rear; FWD: front)
- [ ] Gear shifting occurs at engineMaxRPM
- [ ] Anti-roll bar reduces body roll in cornering

## Size: M

## Risks

- Pacejka friction curve parameters are game-tunable; default values may not feel realistic. Must document that users should tune per tire type.
- Raycast each frame is O(wheels) — cheap, but compound chassis shapes require expanded raycasts. Document limitation to convex hull / primitive chassis.
