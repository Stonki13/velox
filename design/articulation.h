// articulation.h — Design sketch for reduced-coordinate articulated bodies.
// Self-contained: includes only what's needed for compilation in isolation.
// TODO: integrate with velox::World, velox::BodyId, velox::JointId, velox::Vec3, velox::Quat.

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
    using JointId = uint64_t;
}

namespace velox {

// A single degree of freedom in an articulated body. Each DOF has a coordinate
// (position), velocity, and optionally a motor force/torque applied to it.
struct ArticulationDOF {
    enum class Type : uint8_t {
        Revolute,     // hinge: rotation about one axis
        Prismatic,    // slider: translation along one axis
        BallJoint     // spherical: 3 rotational DOFs (treated as 3 revolute)
    };

    Type type = Type::Revolute;
    Vec3 localAxis{};             // axis of motion in the parent body frame
    float coordinate = 0.0f;      // current position (angle or translation)
    float velocity = 0.0f;        // current velocity
    float lowerLimit = -3.14159f; // joint limit lower bound
    float upperLimit = 3.14159f;  // joint limit upper bound
    bool enableLimit = false;     // whether limits are active
    bool enableMotor = false;     // whether a motor is driving this DOF
    float motorTarget = 0.0f;     // target position (limit) or velocity (motor)
    float maxMotorForce = 0.0f;   // N*m for revolute, N for prismatic
    float stiffness = 0.0f;       // spring stiffness (N*m/rad or N/m)
    float damping = 0.0f;         // damping coefficient
};

// Reduced-coordinate articulation: a set of bodies connected by joints that
// are solved as a single constrained system with fewer degrees of freedom than
// the full body+joint representation. This improves stability for ragdolls and
// complex chains by eliminating redundant constraint forces.
//
// Unlike the existing joint-based approach (where each joint is an independent
// iterative constraint), reduced-coordinate articulation solves all DOFs
// simultaneously in a block solve, which converges faster and handles closed
// kinematic chains without drift.
class Articulation {
public:
    // Construct an articulation from a set of bodies and joints that form a
    // tree structure (no closed loops). The root body is the base of the chain.
    //
    // TODO: validate that the body+joint graph forms a tree; throw on cycles.
    // TODO: build the reduced coordinate basis by walking the tree and assigning
    //       DOFs to each joint's degrees of freedom.
    Articulation(World& world, BodyId rootBody,
                 const std::vector<BodyId>& bodies,
                 const std::vector<std::pair<JointId, JointId>>& joints);

    // Update the articulation: sync body positions/velocities from the DOF
    // coordinates, recompute the mass matrix and Coriolis terms. Must be called
    // before Solve(), after any external forces are applied to the bodies.
    //
    // TODO: implement the forward kinematics pass that computes each body's
    //       transform from the current DOF coordinates. This walks the tree from
    //       root to leaves, accumulating transforms along each joint's axis.
    void Update();

    // Solve the articulation for one timestep. Applies motor forces, limit
    // constraints, and spring/damping forces to the DOFs, then integrates.
    //
    // Algorithm (standard reduced-coordinate dynamics):
    //   1. Compute the mass matrix M(q) from body inertias and Jacobian J(q).
    //   2. Compute the Coriolis/centrifugal vector C(q, qdot).
    //   3. Compute external forces: gravity, applied forces, spring/damping.
    //   4. Solve M(q) * qddot = F_external - C(q, qdot) for accelerations.
    //   5. Integrate: qdot += qddot * dt; q += qdot * dt.
    //   6. Apply limits: clamp q to [lowerLimit, upperLimit], reflect velocity.
    //   7. Apply motors: add motor force/torque based on error from target.
    //   8. Sync body transforms back from updated DOF coordinates.
    //
    // TODO: the mass matrix computation is O(n²) in DOF count; for large ragdolls
    //       (>20 bodies), consider sparse solvers or iterative methods.
    void Solve(float dt);

    // Set the target coordinate for a specific DOF (e.g., drive a hinge to 90°).
    void SetDOFTarget(int dofIndex, float target);

    // Get the current coordinate value for a DOF.
    float GetDOFCoordinate(int dofIndex) const;

    // Get the number of degrees of freedom in this articulation.
    int GetDOFCount() const;

    // Add an external force to a specific body in the articulation. The force
    // is mapped to DOF space via the Jacobian before solving.
    void ApplyBodyForce(BodyId body, Vec3 force);
    void ApplyBodyTorque(BodyId body, Vec3 torque);

    // Enable/disable the entire articulation. Disabled articulations are
    // treated as a set of independent bodies by the solver.
    void SetEnabled(bool enabled) { enabled_ = enabled; }
    bool IsEnabled() const { return enabled_; }

private:
    World& world_;
    BodyId rootBody_;
    std::vector<ArticulationDOF> dofs_;
    std::vector<BodyId> bodyOrder_;       // tree traversal order (root first)
    std::vector<int> parentDOF_;          // DOF index of the joint connecting to parent
    bool enabled_ = true;

    // TODO: store the Jacobian matrix J[dof][body][axis] for mapping forces.
    // The Jacobian relates DOF velocities to body spatial velocities:
    //   v_body = J * qdot
    // For a revolute joint about axis a at anchor p:
    //   J = [cross(a, p - bodyCOM); a]  (6x1 column: linear + angular)
};

// Ragdoll authoring helper: constructs an Articulation from a set of body parts
// and joint anchors, with sensible defaults for human-like ragdolls.
class RagdollBuilder {
public:
    // Define a bone: a body part with its local geometry and mass.
    struct BoneDesc {
        std::string name;                 // e.g., "upper_arm_l"
        BodyId body;                      // existing body in the world
        Vec3 localCenterOfMass{};         // COM offset from bone origin
        float mass = 1.0f;
        float radius = 0.05f;             // cylinder radius for visual/debug
    };

    // Define a joint connecting two bones.
    struct JointDesc {
        std::string name;                 // e.g., "shoulder_l"
        BodyId parentBone;                // proximal bone (closer to torso)
        BodyId childBone;                 // distal bone (farther from torso)
        Vec3 anchor{};                    // world-space joint anchor
        Vec3 axis{0, 1, 0};               // primary rotation axis
        float swingLimit = 1.57f;         // max swing angle (radians)
        float twistLimit = 0.5f;          // max twist angle (radians)
    };

    // Build an articulation from bone and joint descriptions. Returns nullptr
    // if the graph is not a valid tree.
    //
    // TODO: implement tree validation (each bone has exactly one parent, root has none).
    // TODO: auto-compute default DOF limits based on human anatomy ranges.
    static std::unique_ptr<Articulation> Build(World& world,
                                                BodyId spineBone,
                                                const std::vector<BoneDesc>& bones,
                                                const std::vector<JointDesc>& joints);
};

} // namespace velox
