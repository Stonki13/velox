#ifndef VELOX_C_H
#define VELOX_C_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file velox_c.h
 * @brief C language binding for the Velox physics engine.
 *
 * This header exposes a flat, opaque-handle API for embedding Velox in C
 * programs or foreign-language runtimes (C#, Python, Rust, ...). It wraps a
 * subset of the C++ `velox::World` surface. All state lives behind the opaque
 * @ref velox_World handle; bodies and joints are referenced by integer ids.
 *
 * @note The C API is not thread-safe. Serialize all calls to a given world
 *       from a single thread, mirroring the C++ `ThreadSafetyPolicy::Strict`
 *       default.
 *
 * @code{.c}
 * velox_World* world = velox_World_Create(VELOX_BACKEND_CPU);
 * velox_World_SetGravity(world, (velox_Vec3){0.0f, -9.81f, 0.0f});
 * velox_World_AddStaticPlane(world, (velox_Vec3){0, 1, 0}, 0.0f);
 * velox_BodyId ball = velox_World_AddSphere(world, (velox_Vec3){0, 5, 0}, 0.5f, 1.0f);
 * for (int i = 0; i < 600; ++i) {
 *     velox_World_Step(world, 1.0f / 60.0f);
 *     velox_Vec3 p = velox_Body_GetPosition(world, ball);
 * }
 * velox_World_Destroy(world);
 * @endcode
 */

/** @brief Opaque physics world. Create with @ref velox_World_Create. */
typedef struct velox_World velox_World;
/** @brief Stable handle to a body (packed slot + generation). */
typedef uint64_t velox_BodyId;
/** @brief Stable handle to a joint (packed slot + generation). */
typedef uint64_t velox_JointId;

/** @brief Three-component single-precision vector. */
typedef struct { float x, y, z; } velox_Vec3;
/** @brief Quaternion orientation `(x, y, z, w)`. */
typedef struct { float x, y, z, w; } velox_Quat;

/** @brief Solver backend selection. */
typedef enum {
    VELOX_BACKEND_AUTO = 0,  ///< CUDA when available, otherwise CPU.
    VELOX_BACKEND_CPU = 1,   ///< Portable CPU reference backend.
    VELOX_BACKEND_CUDA = 2,  ///< NVIDIA CUDA backend (creation fails if unavailable).
    VELOX_BACKEND_VULKAN = 3 ///< Cross-vendor Vulkan compute backend (creation fails if unavailable).
} velox_BackendType;

/** @brief Collider primitive family. */
typedef enum {
    VELOX_SHAPE_SPHERE = 0,
    VELOX_SHAPE_BOX = 1,
    VELOX_SHAPE_CAPSULE = 2,
    VELOX_SHAPE_CYLINDER = 3,
    VELOX_SHAPE_CONE = 4,
    VELOX_SHAPE_HULL = 5,
    VELOX_SHAPE_MESH = 6,
    VELOX_SHAPE_PLANE = 7,
    VELOX_SHAPE_COMPOUND = 8,
    VELOX_SHAPE_ROUNDED_BOX = 9,
    VELOX_SHAPE_ELLIPSOID = 10
} velox_ShapeType;

/** @brief Result of a raycast query. */
typedef struct {
    bool hit;             ///< True when the ray intersected a body.
    velox_BodyId body;    ///< Handle of the body that was hit.
    float distance;       ///< Distance along the ray to the hit point.
    velox_Vec3 point;     ///< World-space hit point.
    velox_Vec3 normal;    ///< Outward surface normal at the hit point.
} velox_RayHit;

/** @name World management
 *  @{
 */

/**
 * @brief Create a physics world.
 * @param backend Solver backend to use (@ref velox_BackendType).
 * @return Owning pointer to the new world; release with @ref velox_World_Destroy.
 */
velox_World* velox_World_Create(velox_BackendType backend);

/**
 * @brief Destroy a world and free all of its resources.
 * @param world World created by @ref velox_World_Create.
 */
void velox_World_Destroy(velox_World* world);

/**
 * @brief Advance the simulation by one fixed timestep.
 * @param world Target world.
 * @param dt    Timestep in seconds (use a fixed value, e.g. `1/60`).
 */
void velox_World_Step(velox_World* world, float dt);

/**
 * @brief Set the world gravity vector.
 * @param world   Target world.
 * @param gravity Gravity acceleration (m/s²), e.g. `{0, -9.81, 0}`.
 */
void velox_World_SetGravity(velox_World* world, velox_Vec3 gravity);

/**
 * @brief Set the number of solver substeps per @ref velox_World_Step call.
 * @param world    Target world.
 * @param substeps Substep count; more substeps yield stiffer stacks.
 */
void velox_World_SetSubsteps(velox_World* world, int substeps);
/** @} */

/** @name Body creation
 *  @{
 */

/**
 * @brief Add a dynamic sphere.
 * @param world    Target world.
 * @param position Initial world-space center.
 * @param radius   Sphere radius.
 * @param mass     Mass in kilograms (positive for a dynamic body).
 * @return Handle to the new body.
 */
velox_BodyId velox_World_AddSphere(velox_World* world, velox_Vec3 position, float radius, float mass);

/**
 * @brief Add a dynamic box.
 * @param world       Target world.
 * @param position    Initial world-space center.
 * @param halfExtents Half extents along each local axis.
 * @param mass        Mass in kilograms.
 * @return Handle to the new body.
 */
velox_BodyId velox_World_AddBox(velox_World* world, velox_Vec3 position, velox_Vec3 halfExtents, float mass);

/**
 * @brief Add a dynamic capsule.
 * @param world      Target world.
 * @param position   Initial world-space center.
 * @param radius     Capsule radius.
 * @param halfHeight Half height of the cylindrical section.
 * @param mass       Mass in kilograms.
 * @return Handle to the new body.
 */
velox_BodyId velox_World_AddCapsule(velox_World* world, velox_Vec3 position, float radius, float halfHeight, float mass);

/**
 * @brief Add a dynamic cylinder.
 * @param world      Target world.
 * @param position   Initial world-space center.
 * @param radius     Cylinder radius.
 * @param halfHeight Half height along the cylinder axis.
 * @param mass       Mass in kilograms.
 * @return Handle to the new body.
 */
velox_BodyId velox_World_AddCylinder(velox_World* world, velox_Vec3 position, float radius, float halfHeight, float mass);

/**
 * @brief Add a dynamic cone.
 * @param world    Target world.
 * @param position Initial world-space center.
 * @param radius   Base radius.
 * @param height   Total height.
 * @param mass     Mass in kilograms.
 * @return Handle to the new body.
 */
velox_BodyId velox_World_AddCone(velox_World* world, velox_Vec3 position, float radius, float height, float mass);

/**
 * @brief Add a static infinite plane.
 * @param world  Target world.
 * @param normal Plane normal (need not be normalized).
 * @param offset Signed distance from the origin along @p normal.
 * @return Handle to the new static body.
 */
velox_BodyId velox_World_AddStaticPlane(velox_World* world, velox_Vec3 normal, float offset);
/** @} */

/**
 * @brief Remove a body and any joints attached to it.
 * @param world Target world.
 * @param body  Handle of the body to remove.
 */
void velox_World_RemoveBody(velox_World* world, velox_BodyId body);

/** @name Body property access
 *  @{
 */

/**
 * @brief Get a body's world-space position.
 * @param world Target world.
 * @param body  Body handle.
 * @return Current center-of-mass position.
 */
velox_Vec3 velox_Body_GetPosition(velox_World* world, velox_BodyId body);

/**
 * @brief Get a body's world-space orientation.
 * @param world Target world.
 * @param body  Body handle.
 * @return Current orientation quaternion.
 */
velox_Quat velox_Body_GetOrientation(velox_World* world, velox_BodyId body);

/**
 * @brief Get a body's linear velocity.
 * @param world Target world.
 * @param body  Body handle.
 * @return Linear velocity (m/s).
 */
velox_Vec3 velox_Body_GetVelocity(velox_World* world, velox_BodyId body);

/**
 * @brief Get a body's angular velocity.
 * @param world Target world.
 * @param body  Body handle.
 * @return Angular velocity (rad/s).
 */
velox_Vec3 velox_Body_GetAngularVelocity(velox_World* world, velox_BodyId body);

/**
 * @brief Teleport a body to a new position.
 * @param world    Target world.
 * @param body     Body handle.
 * @param position New world-space center.
 */
void velox_Body_SetPosition(velox_World* world, velox_BodyId body, velox_Vec3 position);

/**
 * @brief Set a body's orientation.
 * @param world       Target world.
 * @param body        Body handle.
 * @param orientation New orientation quaternion.
 */
void velox_Body_SetOrientation(velox_World* world, velox_BodyId body, velox_Quat orientation);

/**
 * @brief Set a body's linear velocity.
 * @param world    Target world.
 * @param body     Body handle.
 * @param velocity New linear velocity (m/s).
 */
void velox_Body_SetVelocity(velox_World* world, velox_BodyId body, velox_Vec3 velocity);

/**
 * @brief Set a body's angular velocity.
 * @param world           Target world.
 * @param body            Body handle.
 * @param angularVelocity New angular velocity (rad/s).
 */
void velox_Body_SetAngularVelocity(velox_World* world, velox_BodyId body, velox_Vec3 angularVelocity);

/**
 * @brief Accumulate a force applied at the center of mass.
 * @param world Target world.
 * @param body  Body handle.
 * @param force Force vector (N); integrated over the next step.
 */
void velox_Body_ApplyForce(velox_World* world, velox_BodyId body, velox_Vec3 force);

/**
 * @brief Accumulate a torque about the center of mass.
 * @param world  Target world.
 * @param body   Body handle.
 * @param torque Torque vector (N*m).
 */
void velox_Body_ApplyTorque(velox_World* world, velox_BodyId body, velox_Vec3 torque);

/**
 * @brief Apply an instantaneous linear impulse at the center of mass.
 * @param world   Target world.
 * @param body    Body handle.
 * @param impulse Impulse vector (N*s).
 */
void velox_Body_ApplyImpulse(velox_World* world, velox_BodyId body, velox_Vec3 impulse);

/**
 * @brief Get a body's mass.
 * @param world Target world.
 * @param body  Body handle.
 * @return Mass in kilograms (`0` for a static body).
 */
float velox_Body_GetMass(velox_World* world, velox_BodyId body);

/**
 * @brief Set a body's mass.
 * @param world Target world.
 * @param body  Body handle.
 * @param mass  New mass in kilograms.
 */
void velox_Body_SetMass(velox_World* world, velox_BodyId body, float mass);

/**
 * @brief Get a body's collider shape type.
 * @param world Target world.
 * @param body  Body handle.
 * @return The @ref velox_ShapeType of the body's collider.
 */
velox_ShapeType velox_Body_GetShapeType(velox_World* world, velox_BodyId body);
/** @} */

/** @name Queries
 *  @{
 */

/**
 * @brief Cast a ray and return the nearest hit.
 * @param world       Target world.
 * @param origin      Ray origin (world space).
 * @param direction   Ray direction (need not be normalized).
 * @param maxDistance Maximum ray length.
 * @return A @ref velox_RayHit; check its `hit` field before use.
 */
velox_RayHit velox_World_Raycast(velox_World* world, velox_Vec3 origin, velox_Vec3 direction, float maxDistance);
/** @} */

/** @name Joints
 *  @{
 */

/**
 * @brief Connect two bodies with a ball-and-socket joint.
 * @param world  Target world.
 * @param bodyA  First body handle.
 * @param bodyB  Second body handle (may be static).
 * @param anchor World-space anchor point.
 * @return Handle to the new joint.
 */
velox_JointId velox_World_AddBallJoint(velox_World* world, velox_BodyId bodyA, velox_BodyId bodyB, velox_Vec3 anchor);

/**
 * @brief Connect two bodies with a hinge (revolute) joint.
 * @param world  Target world.
 * @param bodyA  First body handle.
 * @param bodyB  Second body handle (may be static).
 * @param anchor World-space anchor point.
 * @param axis   World-space hinge axis.
 * @return Handle to the new joint.
 */
velox_JointId velox_World_AddHingeJoint(velox_World* world, velox_BodyId bodyA, velox_BodyId bodyB, velox_Vec3 anchor, velox_Vec3 axis);

/**
 * @brief Connect two bodies with a fixed-distance joint.
 * @param world   Target world.
 * @param bodyA   First body handle.
 * @param bodyB   Second body handle (may be static).
 * @param anchorA World-space anchor on body A.
 * @param anchorB World-space anchor on body B.
 * @return Handle to the new joint.
 */
velox_JointId velox_World_AddDistanceJoint(velox_World* world, velox_BodyId bodyA, velox_BodyId bodyB, velox_Vec3 anchorA, velox_Vec3 anchorB);

/**
 * @brief Connect two bodies with a motor joint.
 *
 * Drives body B towards a target transform relative to body A, clamping the
 * corrective impulses each step.
 * @param world     Target world.
 * @param bodyA     First body handle.
 * @param bodyB     Second body handle.
 * @param anchorA   World-space anchor on body A.
 * @param anchorB   World-space anchor on body B.
 * @param maxForce  Maximum corrective linear force.
 * @param maxTorque Maximum corrective angular torque.
 * @return Handle to the new joint.
 */
velox_JointId velox_World_AddMotorJoint(velox_World* world, velox_BodyId bodyA, velox_BodyId bodyB, velox_Vec3 anchorA, velox_Vec3 anchorB, float maxForce, float maxTorque);

/**
 * @brief Remove a joint.
 * @param world Target world.
 * @param joint Handle of the joint to remove.
 */
void velox_World_RemoveJoint(velox_World* world, velox_JointId joint);
/** @} */

/**
 * @brief Apply a radial impulse to all dynamic bodies within a radius.
 *
 * Falloff is linear: full impulse at the center, zero at the radius boundary.
 * @param world   Target world.
 * @param origin  Explosion center (world space).
 * @param radius  Effect radius.
 * @param impulse Impulse magnitude at the center.
 */
void velox_World_Explode(velox_World* world, velox_Vec3 origin, float radius, float impulse);

/** @name Recording/Replay
 *  @{
 */

/** @brief Opaque deterministic-replay recording. */
typedef struct velox_ReplayRecording velox_ReplayRecording;

/**
 * @brief Create a replay recording.
 * @return Owning pointer; release with @ref velox_ReplayRecording_Destroy.
 */
velox_ReplayRecording* velox_ReplayRecording_Create(void);

/**
 * @brief Destroy a replay recording and free its resources.
 * @param recording Recording created by @ref velox_ReplayRecording_Create.
 */
void velox_ReplayRecording_Destroy(velox_ReplayRecording* recording);

/**
 * @brief Begin recording a world's trajectory.
 * @param recording Target recording.
 * @param world     World whose state is captured.
 * @param dt        Fixed timestep that will be used for each recorded frame.
 */
void velox_ReplayRecording_Begin(velox_ReplayRecording* recording, velox_World* world, float dt);

/**
 * @brief Capture one frame of the world into the recording.
 * @param recording Target recording (must have been started with @ref velox_ReplayRecording_Begin).
 * @param world     World to capture.
 */
void velox_ReplayRecording_RecordFrame(velox_ReplayRecording* recording, velox_World* world);

/**
 * @brief Replay the recording and count frames that diverge beyond tolerance.
 * @param recording          Target recording.
 * @param positionTolerance  Allowed per-body position error.
 * @param velocityTolerance  Allowed per-body velocity error.
 * @return Number of frames that exceeded either tolerance (`0` = exact replay).
 */
uint64_t velox_ReplayRecording_Verify(velox_ReplayRecording* recording, float positionTolerance, float velocityTolerance);
/** @} */

#ifdef __cplusplus
}
#endif

#endif // VELOX_C_H
