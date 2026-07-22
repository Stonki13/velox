#pragma once
#include "body.h"

namespace velox {

/**
 * @file joint.h
 * @brief Joint handle, joint types, and the configurable Joint constraint.
 *
 * Joints connect two bodies (body B may be a static body to anchor the joint
 * to the world). Anchors and axes are stored in each body's local frame and
 * solved with iterative impulses plus Baumgarte positional correction.
 * Create joints through the `World::add*Joint` methods and configure motors,
 * limits, and breaking thresholds via the returned Joint reference.
 */

/**
 * @brief Stable external handle to a joint.
 *
 * The slot identifies a logical joint while the generation rejects handles
 * retained after that slot has been removed and reused. Validate a retained
 * handle with `World::isValid(JointId)` before use.
 */
struct JointId {
    uint64_t value = UINT64_MAX; ///< Packed `(generation << 32) | slot`.

    /**
     * @brief Pack a slot and generation into a handle.
     * @param slot       Dense slot index.
     * @param generation Reuse generation counter for the slot.
     * @return The combined handle.
     */
    static JointId make(uint32_t slot, uint32_t generation) {
        JointId id;
        id.value = (uint64_t(generation) << 32) | slot;
        return id;
    }
    uint32_t slot() const { return uint32_t(value); }       ///< Slot component.
    uint32_t generation() const { return uint32_t(value >> 32); } ///< Generation component.
};

inline bool operator==(JointId a, JointId b) { return a.value == b.value; }
inline bool operator!=(JointId a, JointId b) { return !(a == b); }

/** @brief The family of constraint a @ref Joint implements. */
enum class JointType : uint8_t {
    Ball,      ///< Point-to-point ball-and-socket; no angular limits.
    Distance,  ///< Holds two anchors a fixed (optionally springy) distance apart.
    Hinge,     ///< Rotation about a single axis; supports motor and angle limits.
    ConeTwist, ///< Ball joint with cone swing and twist limits.
    Fixed,     ///< Rigidly locks the relative transform of both bodies.
    Prismatic, ///< Sliding along a single axis; supports motor and travel limits.
    SixDof,    ///< Fully configurable per-axis linear/angular limits and motors.
    Motor,     ///< Drives body B towards a target transform relative to body A.
    Weld,      ///< Rigid connection with explicit breakable threshold.
    Wheel,     ///< Suspension with steering; spring along axis, free rotation.
    Rope,      ///< Maximum-distance constraint with slack (no minimum).
    Pulley,    ///< Two bodies linked through ground anchors with a ratio.
    Gear       ///< Couples angular velocity of two bodies at a fixed ratio.
};

/** @brief Per-axis selection bits used by the six-degree-of-freedom joint. */
enum JointAxisBits : uint8_t {
    JointAxisX = 1u << 0,                              ///< Joint-frame X axis.
    JointAxisY = 1u << 1,                              ///< Joint-frame Y axis.
    JointAxisZ = 1u << 2,                              ///< Joint-frame Z axis.
    JointAxisAll = JointAxisX | JointAxisY | JointAxisZ ///< All three axes.
};

/**
 * @brief A configurable two-body constraint.
 *
 * Joints connect two bodies (b may be a static body). Anchors and axes are
 * stored in each body's local frame and solved with iterative impulses plus
 * Baumgarte positional correction.
 *
 * Only the fields relevant to @ref type are interpreted by the solver; the
 * rest are ignored. The trailing impulse members are solver scratch and are
 * reset every substep — do not rely on their values across steps.
 *
 * @code
 * velox::JointId hinge = world.addHingeJoint(door, frame, anchor, axis);
 * velox::Joint& j = world.joint(hinge);
 * j.enableLimit = true;
 * j.lowerLimit = 0.0f;
 * j.upperLimit = 1.5f;          // radians
 * j.enableMotor = true;
 * j.motorSpeed = 0.5f;          // rad/s
 * j.maxMotorTorque = 10.0f;     // N*m budget
 * @endcode
 */
struct Joint {
    JointType type;                 ///< Constraint family (set at creation).
    BodyIndex a, b;                 ///< Dense indices of the connected bodies.
    Vec3 localAnchorA, localAnchorB; ///< Attachment points in each body's local frame.
    Vec3 localAxisA{0, 1, 0}, localAxisB{0, 1, 0}; ///< Hinge/slider/frame axis.
    Vec3 localRefA{1, 0, 0}, localRefB{1, 0, 0};   ///< Perpendicular/frame reference.
    float restLength = 0.0f;        ///< Target distance for `Distance` joints.
    bool collideConnected = false;  ///< Ignore contacts between linked bodies.

    // Optional compliant distance constraint. Frequency is mass-independent;
    // dampingRatio 1 is critical damping, 0 is undamped.
    bool enableSpring = false;      ///< Enable the compliant distance spring.
    float springFrequencyHz = 0.0f; ///< Oscillation frequency (mass-independent).
    float springDampingRatio = 1.0f;///< 1 = critical damping, 0 = undamped.

    // Joint is removed after a solver pass whose reaction exceeds either
    // threshold. Defaults effectively disable breaking.
    float breakForce = 3.402823466e+38F;  ///< Reaction force that breaks the joint.
    float breakTorque = 3.402823466e+38F; ///< Reaction torque that breaks the joint.

    // Hinge motor: drives the relative angular velocity about the axis.
    bool enableMotor = false;   ///< Drive relative motion about/along the axis.
    float motorSpeed = 0.0f;    ///< Hinge: rad/s A vs B; prismatic: m/s B vs A.
    float maxMotorTorque = 0.0f;///< Hinge motor torque budget (N*m).
    float maxMotorForce = 0.0f; ///< Prismatic motor force budget (N).

    // Hinge angle (radians) or prismatic translation (meters), both zero at creation.
    bool enableLimit = false;       ///< Enable hinge angle / prismatic travel limits.
    float lowerLimit = 0.0f, upperLimit = 0.0f; ///< Limit bounds (rad or m).

    // Cone/twist limits: swing is the angle between the two primary axes;
    // twist is B's signed rotation relative to A around the primary axis.
    bool enableSwingLimit = false;  ///< Enable the cone swing limit.
    float swingLimit = 3.14159265f; ///< Maximum swing angle (radians).
    bool enableTwistLimit = false;  ///< Enable the twist limit.
    float lowerTwistLimit = -3.14159265f; ///< Lower twist bound (radians).
    float upperTwistLimit = 3.14159265f;  ///< Upper twist bound (radians).

    // Six-degree-of-freedom joint configuration. Bit 0/1/2 selects the
    // joint-frame X/Y/Z axis. A limit bit with equal bounds locks that axis;
    // clearing it leaves the axis free. Motors drive B relative to A.
    uint8_t linearLimitMask = JointAxisAll;  ///< Axes with linear limits enabled.
    uint8_t angularLimitMask = JointAxisAll; ///< Axes with angular limits enabled.
    uint8_t linearMotorMask = 0;             ///< Axes with linear motors enabled.
    uint8_t angularMotorMask = 0;            ///< Axes with angular motors enabled.
    Vec3 lowerLinearLimit;   ///< Per-axis lower linear bounds.
    Vec3 upperLinearLimit;   ///< Per-axis upper linear bounds.
    Vec3 lowerAngularLimit;  ///< Per-axis lower angular bounds (radians).
    Vec3 upperAngularLimit;  ///< Per-axis upper angular bounds (radians).
    Vec3 linearMotorSpeed;   ///< Per-axis linear motor target speed.
    Vec3 angularMotorSpeed;  ///< Per-axis angular motor target speed.
    Vec3 maxLinearMotorForce;  ///< Per-axis linear motor force budget.
    Vec3 maxAngularMotorTorque;///< Per-axis angular motor torque budget.

    // Weld joint: rigid 6-DOF lock with explicit break thresholds. Uses the
    // same breakForce/breakTorque fields above. The weldFrequencyHz and
    // weldDampingRatio control angular softness (0 = perfectly rigid).
    float weldFrequencyHz = 0.0f;   ///< Angular softness frequency (0 = rigid).
    float weldDampingRatio = 1.0f;  ///< Angular softness damping (1 = critical).

    // Wheel joint: suspension spring along localAxisA, free rotation about
    // localAxisA (the axle), and optional steering about localRefA.
    float suspensionFrequencyHz = 2.0f;  ///< Suspension spring frequency.
    float suspensionDampingRatio = 0.7f; ///< Suspension spring damping ratio.
    float wheelRadius = 0.5f;            ///< Wheel radius for ground contact.
    bool enableSteering = false;         ///< Enable steering about the up axis.
    float steeringAngle = 0.0f;          ///< Current steering angle (radians).
    float maxSteeringAngle = 0.6f;       ///< Steering limit (radians).

    // Rope joint: enforces a maximum distance; allows slack (shorter distances
    // are unconstrained). maxLength is the taut length.
    float maxLength = 0.0f;  ///< Maximum allowed distance between anchors.

    // Pulley joint: two bodies connected through fixed ground anchor points
    // with a mechanical advantage ratio. lengthA + ratio * lengthB = restTotal.
    Vec3 pulleyAnchorA;      ///< Ground anchor for body A (world space at creation).
    Vec3 pulleyAnchorB;      ///< Ground anchor for body B (world space at creation).
    float pulleyRatio = 1.0f;///< Mechanical advantage (force multiplier for B).

    // Gear joint: couples angular velocity of body B to body A at a fixed
    // ratio. ratio = angularSpeedB / angularSpeedA about their respective axes.
    float gearRatio = 1.0f;  ///< Angular velocity ratio (B = ratio * A).

    // Solver scratch, reset for every substep.
    float motorImpulse = 0.0f;      ///< @privatesection Accumulated motor impulse.
    float limitImpulse = 0.0f;      ///< Accumulated limit impulse.
    float swingImpulse = 0.0f;      ///< Accumulated swing-limit impulse.
    float twistImpulse = 0.0f;      ///< Accumulated twist-limit impulse.
    float springImpulse = 0.0f;     ///< Accumulated spring impulse.
    Vec3 linearMotorImpulse;        ///< Accumulated linear motor impulse.
    Vec3 angularMotorImpulse;       ///< Accumulated angular motor impulse.
    Vec3 linearLimitImpulse;        ///< Accumulated linear limit impulse.
    Vec3 angularLimitImpulse;       ///< Accumulated angular limit impulse.
    Vec3 reactionLinearImpulse;     ///< Last-step reaction linear impulse (for breaking).
    Vec3 reactionAngularImpulse;    ///< Last-step reaction angular impulse (for breaking).
    bool broken = false;            ///< Set when the joint exceeded a break threshold.
};

} // namespace velox
