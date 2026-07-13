#pragma once
#include "body.h"

namespace velox {

using JointId = uint32_t;

enum class JointType : uint8_t { Ball, Distance, Hinge };

// Joints connect two bodies (b may be a static body). Anchors and axes are
// stored in each body's local frame and solved with iterative impulses plus
// Baumgarte positional correction.
struct Joint {
    JointType type;
    BodyId a, b;
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

    // Solver scratch, reset for every substep.
    float motorImpulse = 0.0f;
    float limitImpulse = 0.0f;
};

} // namespace velox
