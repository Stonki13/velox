#pragma once

#include "world.h"

#include <vector>

namespace velox {

// A ragdoll bone refers to an existing body. Velox bodies are authored around
// their center of mass, so localCenterOfMass is retained as source-asset
// metadata and validated but does not move the collider.
struct RagdollBone {
    BodyId body;
    Vec3 localCenterOfMass{};
    float mass = 1.0f;
};

// Passive ragdoll links are cone-twist joints. A motorized link becomes a
// hinge about axis so it can use the existing deterministic hinge motor.
struct RagdollJoint {
    BodyId parent, child;
    Vec3 anchor{};
    Vec3 axis{0, 1, 0};
    float swingLimit = 1.57f;
    float twistLimit = 0.5f;
    bool enableMotor = false;
    float motorSpeed = 0.0f;
    float maxMotorTorque = 50.0f;
};

class RagdollBuilder {
public:
    // Validates a directed, connected tree, applies authored dynamic masses,
    // and creates the requested cone-twist or motorized hinge constraints.
    // Returns the unique tree root. All input bodies must already belong to
    // world; no hidden Body ownership is introduced by the builder.
    static BodyId Build(World& world, const std::vector<RagdollBone>& bones,
                        const std::vector<RagdollJoint>& joints);

    // Configures the torque budget of a motorized hinge link.
    static void SetMotorTorque(World& world, JointId joint, float torque);

    // Wakes every still-valid body that was registered under this root.
    static void WakeAll(World& world, BodyId ragdollRoot);

    // Returns the constraint handles created for a root. Handles can become
    // stale after ordinary World body/joint removal and are filtered on read.
    static std::vector<JointId> Joints(World& world, BodyId ragdollRoot);
};

} // namespace velox
