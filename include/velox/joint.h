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

enum class JointType : uint8_t { Ball, Distance, Hinge, ConeTwist };

// Joints connect two bodies (b may be a static body). Anchors and axes are
// stored in each body's local frame and solved with iterative impulses plus
// Baumgarte positional correction.
struct Joint {
    JointType type;
    BodyIndex a, b;
    Vec3 localAnchorA, localAnchorB;
    Vec3 localAxisA{0, 1, 0}, localAxisB{0, 1, 0}; // Hinge
    Vec3 localRefA{1, 0, 0}, localRefB{1, 0, 0};   // Hinge angle reference (perp to axis)
    float restLength = 0.0f;                       // Distance
    bool collideConnected = false;                 // Ignore contacts between linked bodies

    // Hinge motor: drives the relative angular velocity about the axis.
    bool enableMotor = false;
    float motorSpeed = 0.0f;      // rad/s, positive = A relative to B about the axis
    float maxMotorTorque = 0.0f;  // N*m budget

    // Hinge limit: clamps the joint angle (radians, measured at creation = 0).
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
};

} // namespace velox
