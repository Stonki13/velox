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
    float restLength = 0.0f;                       // Distance
};

} // namespace velox
