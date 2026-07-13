#pragma once
#include "body.h"

namespace velox {

struct JointId {
    uint64_t value = UINT64_MAX;

    static JointId make(uint32_t slot, uint32_t generation) {
        JointId id;
        id.value = (uint64_t(generation) << 32) | slot;
        return id;
    }
    uint32_t slot() const { return uint32_t(value); }
    uint32_t generation() const { return uint32_t(value >> 32); }
};

inline bool operator==(JointId a, JointId b) { return a.value == b.value; }
inline bool operator!=(JointId a, JointId b) { return !(a == b); }

enum class JointType : uint8_t {
    Ball, Distance, Hinge, ConeTwist, Fixed, Prismatic
};

// Joints connect two bodies (b may be a static body). Anchors and axes are
// stored in each body's local frame and solved with iterative impulses plus
// Baumgarte positional correction.
struct Joint {
    JointType type;
    BodyIndex a, b;
    Vec3 localAnchorA, localAnchorB;
    Vec3 localAxisA{0, 1, 0}, localAxisB{0, 1, 0}; // Hinge/slider/frame axis
    Vec3 localRefA{1, 0, 0}, localRefB{1, 0, 0};   // Perpendicular/frame reference
    float restLength = 0.0f;                       // Distance
    bool collideConnected = false;                 // Ignore contacts between linked bodies

    // Optional compliant distance constraint. Frequency is mass-independent;
    // dampingRatio 1 is critical damping, 0 is undamped.
    bool enableSpring = false;
    float springFrequencyHz = 0.0f;
    float springDampingRatio = 1.0f;

    // Joint is removed after a solver pass whose reaction exceeds either
    // threshold. Defaults effectively disable breaking.
    float breakForce = 3.402823466e+38F;
    float breakTorque = 3.402823466e+38F;

    // Hinge motor: drives the relative angular velocity about the axis.
    bool enableMotor = false;
    float motorSpeed = 0.0f;      // Hinge: rad/s A vs B; prismatic: m/s B vs A
    float maxMotorTorque = 0.0f;  // N*m budget
    float maxMotorForce = 0.0f;   // Prismatic motor N budget

    // Hinge angle (radians) or prismatic translation (meters), both zero at creation.
    bool enableLimit = false;
    float lowerLimit = 0.0f, upperLimit = 0.0f;

    // Cone/twist limits: swing is the angle between the two primary axes;
    // twist is B's signed rotation relative to A around the primary axis.
    bool enableSwingLimit = false;
    float swingLimit = 3.14159265f;
    bool enableTwistLimit = false;
    float lowerTwistLimit = -3.14159265f;
    float upperTwistLimit = 3.14159265f;

    // Solver scratch, reset for every substep.
    float motorImpulse = 0.0f;
    float limitImpulse = 0.0f;
    float swingImpulse = 0.0f;
    float twistImpulse = 0.0f;
    float springImpulse = 0.0f;
    Vec3 reactionLinearImpulse;
    Vec3 reactionAngularImpulse;
    bool broken = false;
};

} // namespace velox
