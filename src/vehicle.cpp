#include "velox/vehicle.h"

#include <algorithm>
#include <cmath>

namespace velox {
namespace {

constexpr float kGravity = 9.81f;

// Simplified Pacejka-style curve: rises to a peak near `peakSlip`, then eases
// to a slightly lower sliding value. Stable, tunable, and free of the real
// Pacejka's parameter sensitivity (documented as game-tunable in the spec).
float slipCurve(float slip, float peakSlip) {
    const float s = std::fabs(slip) / peakSlip;
    const float shape = s < 1.0f ? s * (2.0f - s)             // rise to 1.0
                                 : 1.0f - 0.15f * std::min(1.0f, (s - 1.0f) * 0.5f);
    return slip < 0.0f ? -shape : shape;
}

} // namespace

Vehicle::Vehicle(World& world, const VehicleConfig& config, Vec3 position)
    : world_(world), config_(config) {
    chassis_ = world_.addBox(position, config_.chassisHalfExtents, config_.chassisMass);
    Body& body = world_.body(chassis_);
    body.friction = 0.5f;
    body.restitution = 0.0f;
    body.angularDamping = 0.25f; // a car body is aerodynamically damped
    rpm_ = config_.engineIdleRPM;
}

void Vehicle::AddWheel(const WheelConfig& config) {
    wheels_.push_back(config);
    states_.push_back({});
}

void Vehicle::AddDefaultWheels() {
    const Vec3 half = config_.chassisHalfExtents;
    const float x = half.x * 0.9f;
    const float z = half.z * 0.72f;
    const float y = -half.y * 0.4f;
    // Quarter-car tuning: spring rate and damping derived from the sprung
    // corner mass and the configured ride frequency / damping ratio.
    const float cornerMass = config_.chassisMass * 0.25f;
    const float omega = 2.0f * 3.14159265f * config_.suspensionFrequencyHz;
    const float stiffness = cornerMass * omega * omega;
    const float damping =
        2.0f * config_.suspensionDampingRatio * std::sqrt(stiffness * cornerMass);
    for (int i = 0; i < 4; ++i) {
        WheelConfig wheel;
        const bool front = i < 2;
        wheel.localPosition = {i % 2 == 0 ? -x : x, y, front ? z : -z};
        wheel.suspensionStiffness = stiffness;
        wheel.suspensionDamping = damping;
        wheel.suspensionRestLength = 0.42f;
        wheel.suspensionCompression = 0.16f;
        wheel.suspensionExtension = 0.28f;
        wheel.lateralFriction = config_.defaultTireFriction;
        wheel.longitudinalFriction = config_.defaultTireFriction;
        wheel.steerable = front;
        wheel.driven = config_.drivetrain == DrivetrainType::AWD ||
                       (config_.drivetrain == DrivetrainType::FWD) == front;
        AddWheel(wheel);
    }
}

float Vehicle::forwardSpeed() const {
    const Body& body = world_.body(chassis_);
    const Vec3 forward = rotate(body.orientation, {0.0f, 0.0f, 1.0f});
    return dot(body.velocity, forward);
}

void Vehicle::Step(float dt) {
    if (wheels_.empty() || dt <= 0.0f) return;
    Body& body = world_.body(chassis_);
    if (throttle_ > 0.0f || brake_ > 0.0f || std::fabs(steering_) > 1e-3f)
        world_.wake(chassis_);

    const Quat orientation = body.orientation;
    const Vec3 chassisForward = rotate(orientation, {0.0f, 0.0f, 1.0f});

    // Drivetrain: engine torque through the current gear to driven wheels.
    int drivenCount = 0;
    for (const WheelConfig& wheel : wheels_)
        if (wheel.driven) ++drivenCount;
    const float gearRatio =
        config_.gearRatios[static_cast<size_t>(std::clamp(gear_, 1, (int)config_.gearRatios.size()) - 1)];
    const float totalRatio = gearRatio * config_.finalDriveRatio;
    // Torque tapers to zero at redline so the car cannot exceed it in-gear.
    const float rpmNorm = std::clamp(rpm_ / config_.engineMaxRPM, 0.0f, 1.0f);
    const float engineTorque = throttle_ * config_.engineMaxTorque *
                               (1.0f - 0.75f * rpmNorm * rpmNorm);
    const float wheelDriveTorque =
        drivenCount > 0 ? engineTorque * totalRatio / static_cast<float>(drivenCount) : 0.0f;

    float drivenSpinSum = 0.0f;
    QueryFilter filter;
    filter.ignoredBody = chassis_;

    for (size_t index = 0; index < wheels_.size(); ++index) {
        const WheelConfig& wheel = wheels_[index];
        WheelState& state = states_[index];

        // Steered wheel basis (yaw about the suspension axis).
        const float steer = wheel.steerable ? steering_ : 0.0f;
        const Quat steerRotation = fromAxisAngle({0.0f, 1.0f, 0.0f}, steer);
        const Vec3 wheelForward = rotate(orientation, rotate(steerRotation, {0.0f, 0.0f, 1.0f}));

        const Vec3 hub = body.position + rotate(orientation, wheel.localPosition);
        const Vec3 down = normalize(rotate(orientation, wheel.direction));
        const float maxRayLength =
            wheel.suspensionRestLength + wheel.suspensionExtension + wheel.radius;

        const RayHit hit = world_.rayCast(hub, down, maxRayLength, filter);
        state.grounded = hit.hit;
        if (!hit.hit) {
            // Airborne: relax compression; wheel spins from drive torque only.
            state.compression = 0.0f;
            state.suspensionForce = 0.0f;
            const float inertia = 0.5f * 20.0f * wheel.radius * wheel.radius;
            if (wheel.driven)
                state.spinVelocity += wheelDriveTorque / inertia * dt;
            state.spinVelocity *= 1.0f / (1.0f + 0.5f * dt);
            state.rotation += state.spinVelocity * dt;
            continue;
        }

        state.contactPoint = hit.point;
        state.contactNormal = hit.normal;
        state.contactBody = hit.body;

        // Suspension: spring + damper along the travel axis, applied at the
        // hub (upward on the chassis).
        const float targetLength = wheel.suspensionRestLength + wheel.radius;
        float compression = targetLength - hit.t;
        compression = std::clamp(compression, -wheel.suspensionExtension,
                                 wheel.suspensionCompression);
        const Vec3 hubVelocity =
            body.velocity + cross(body.angularVelocity, hub - body.position);
        // Hub moving along the travel axis (down) closes on the ground and
        // increases compression, so the rate is +dot(v, down).
        const float compressionRate = dot(hubVelocity, down);
        float suspensionForce = wheel.suspensionStiffness * compression +
                                wheel.suspensionDamping * compressionRate;
        // Progressive bump stop over the last 25% of compression travel:
        // real dampers stiffen sharply near the end instead of clanging into
        // a hard limit (and it keeps soft ride rates from bottoming out).
        const float bumpStart = 0.75f * wheel.suspensionCompression;
        if (compression > bumpStart) {
            const float excess = compression - bumpStart;
            suspensionForce += 8.0f * wheel.suspensionStiffness * excess *
                               (1.0f + excess / (wheel.suspensionCompression - bumpStart));
        }
        suspensionForce = std::max(0.0f, suspensionForce);
        state.compression = compression;
        state.suspensionForce = suspensionForce;
        world_.addForceAtPoint(chassis_, down * -suspensionForce, hub);

        // Tire frame on the ground plane.
        Vec3 forward = wheelForward - hit.normal * dot(wheelForward, hit.normal);
        const float forwardLength = length(forward);
        if (forwardLength < 1e-4f) continue;
        forward *= 1.0f / forwardLength;
        const Vec3 side = cross(hit.normal, forward);

        const Vec3 contactVelocity =
            body.velocity + cross(body.angularVelocity, hit.point - body.position);
        const float vForward = dot(contactVelocity, forward);
        const float vSide = dot(contactVelocity, side);

        // Longitudinal: slip ratio between wheel surface speed and ground.
        const float surfaceSpeed = state.spinVelocity * wheel.radius;
        const float slipDenominator = std::max(std::fabs(vForward), 1.0f);
        const float slipRatio = (surfaceSpeed - vForward) / slipDenominator;
        const float load = suspensionForce;
        float longitudinalForce =
            slipCurve(slipRatio, 0.12f) * wheel.longitudinalFriction * load;

        // Lateral: slip angle approximation (side velocity over speed).
        const float slipAngle = std::atan2(vSide, slipDenominator);
        float lateralForce = -slipCurve(slipAngle, 0.20f) * wheel.lateralFriction * load;

        // Friction circle: combined demand cannot exceed the load budget.
        const float budget =
            load * std::max(wheel.longitudinalFriction, wheel.lateralFriction);
        const float demand = std::sqrt(longitudinalForce * longitudinalForce +
                                       lateralForce * lateralForce);
        if (demand > budget && demand > 1e-6f) {
            const float scale = budget / demand;
            longitudinalForce *= scale;
            lateralForce *= scale;
        }
        // Longitudinal force acts at the contact patch (weight transfer under
        // throttle/braking); lateral force acts at the suspension roll center
        // so a tall chassis leans like a road car instead of tripping over
        // its own grip.
        world_.addForceAtPoint(chassis_, forward * longitudinalForce, hit.point);
        const Vec3 rollCenter =
            hit.point + (body.position - hit.point) * config_.lateralRollCenterFactor;
        world_.addForceAtPoint(chassis_, side * lateralForce, rollCenter);

        // Wheel spin integration: drive torque, brake torque, and the ground
        // reaction from the tire force.
        const float wheelMass = 20.0f;
        const float inertia = 0.5f * wheelMass * wheel.radius * wheel.radius;
        float spinTorque = -longitudinalForce * wheel.radius;
        if (wheel.driven) spinTorque += wheelDriveTorque;
        const float brakeTorqueMax = 3000.0f * brake_;
        state.spinVelocity += spinTorque / inertia * dt;
        if (brakeTorqueMax > 0.0f) {
            const float brakeDelta = brakeTorqueMax / inertia * dt;
            if (std::fabs(state.spinVelocity) <= brakeDelta) state.spinVelocity = 0.0f;
            else state.spinVelocity -= brakeDelta * (state.spinVelocity > 0.0f ? 1.0f : -1.0f);
        }
        // Grounded free-rolling relaxation toward ground speed — but NOT
        // while braking: it would erase the very slip the brake torque
        // creates and cap deceleration far below the tire's grip.
        if (brakeTorqueMax <= 0.0f) {
            const float targetSpin = vForward / wheel.radius;
            state.spinVelocity += (targetSpin - state.spinVelocity) *
                                  std::min(1.0f, 8.0f * dt);
        }
        state.rotation += state.spinVelocity * dt;
        if (wheel.driven) drivenSpinSum += std::fabs(state.spinVelocity);
    }

    // Anti-roll bars: per axle pair (wheels are added in left/right pairs),
    // transfer force from the compressed side to the extended side.
    if (config_.enableAntiRoll) {
        for (size_t pair = 0; pair + 1 < wheels_.size(); pair += 2) {
            const WheelState& left = states_[pair];
            const WheelState& right = states_[pair + 1];
            if (!left.grounded || !right.grounded) continue;
            const float difference = left.compression - right.compression;
            const float force = config_.antiRollStiffness * difference;
            const Vec3 hubLeft =
                body.position + rotate(orientation, wheels_[pair].localPosition);
            const Vec3 hubRight =
                body.position + rotate(orientation, wheels_[pair + 1].localPosition);
            // The twisted bar adds suspension force on the MORE compressed
            // side (up on the chassis corner) and unloads the other side,
            // producing a torque that rights the body.
            const Vec3 up = -normalize(rotate(orientation, wheels_[pair].direction));
            world_.addForceAtPoint(chassis_, up * force, hubLeft);
            world_.addForceAtPoint(chassis_, up * -force, hubRight);
        }
    }

    // Engine RPM follows the driven wheels through the gearing; automatic
    // shifting with a short cooldown to avoid hunting.
    if (drivenCount > 0) {
        const float averageSpin = drivenSpinSum / static_cast<float>(drivenCount);
        const float wheelRPM = averageSpin * 60.0f / (2.0f * 3.14159265f);
        rpm_ = std::max(config_.engineIdleRPM, wheelRPM * totalRatio);
    }
    shiftCooldown_ = std::max(0.0f, shiftCooldown_ - dt);
    if (shiftCooldown_ <= 0.0f) {
        if (rpm_ >= config_.shiftUpRPM && gear_ < (int)config_.gearRatios.size()) {
            ++gear_;
            shiftCooldown_ = 0.5f;
        } else if (rpm_ <= config_.shiftDownRPM && gear_ > 1) {
            --gear_;
            shiftCooldown_ = 0.5f;
        }
    }
}

} // namespace velox
