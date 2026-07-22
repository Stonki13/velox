#pragma once
#include <cfloat>
#include "ccd.h"
#include "math.h"
#include <cstdint>
#include <vector>

namespace velox {

/**
 * @file body.h
 * @brief Rigid-body representation: handles, shape/motion/material enums,
 *        compound shapes, and the data-oriented Body struct.
 *
 * The Body layout is plain data in contiguous arrays so the same
 * representation can be uploaded to a GPU backend unchanged. Most fields are
 * configured through `World` mutators (`World::setLinearVelocity`,
 * `World::setCollisionFilter`, ...) rather than written directly.
 */

using BodyIndex = uint32_t; ///< Dense index into the World's body arrays.

/**
 * @brief Stable external handle to a body.
 *
 * The slot identifies a logical body while the generation rejects handles
 * retained after that slot has been removed and reused. Simulation internals
 * use @ref BodyIndex for dense CPU/GPU arrays; application code should keep
 * `BodyId` handles and validate them with `World::isValid`.
 */
struct BodyId {
    uint64_t value = UINT64_MAX; ///< Packed `(generation << 32) | slot`.

    /**
     * @brief Pack a slot and generation into a handle.
     * @param slot       Dense slot index.
     * @param generation Reuse generation counter for the slot.
     * @return The combined handle.
     */
    VELOX_HD static BodyId make(uint32_t slot, uint32_t generation) {
        BodyId id;
        id.value = (uint64_t(generation) << 32) | slot;
        return id;
    }
    VELOX_HD uint32_t slot() const { return uint32_t(value); }       ///< Slot component.
    VELOX_HD uint32_t generation() const { return uint32_t(value >> 32); } ///< Generation component.
};

VELOX_HD inline bool operator==(BodyId a, BodyId b) { return a.value == b.value; }
VELOX_HD inline bool operator!=(BodyId a, BodyId b) { return !(a == b); }
VELOX_HD inline bool operator<(BodyId a, BodyId b) { return a.value < b.value; }

/** @brief Collider primitive family owned by a @ref Body. */
enum class ShapeType : uint8_t {
    Sphere, Plane, Box, Capsule, Mesh, Hull, Compound, Cylinder, Cone,
    RoundedBox, Ellipsoid
};

/** @brief How the solver treats a body's motion. */
enum class MotionType : uint8_t {
    Static,    ///< Infinite mass; never moves (level geometry).
    Kinematic, ///< Moved by the user via velocity/transform; ignores forces.
    Dynamic    ///< Fully simulated; integrates forces, impulses, and contacts.
};

/**
 * @brief Amount of numerical work spent on a body's convex geometry.
 *
 * Robust and Paranoid are intended for imported or fuzzed geometry; the value
 * remains plain data so CPU and CUDA consume the same body representation.
 */
enum class GeometryQuality : uint8_t { Normal, Robust, Paranoid };

/**
 * @brief Rule used to combine a material property from two touching bodies.
 *
 * When two modes differ, the mode later in this enum takes precedence.
 */
enum class MaterialCombineMode : uint8_t {
    Average, GeometricMean, Minimum, Multiply, Maximum
};

/**
 * @brief Host-side description of one locally transformed convex child.
 *
 * Compound bodies accept spheres, boxes, capsules, cylinders, cones, and
 * convex hulls. Pass a vector of these to `World::addCompound`.
 */
struct CompoundShape {
    ShapeType shape = ShapeType::Sphere; ///< Primitive type of this child.
    Vec3 localPosition;                  ///< Child offset in the compound frame.
    Quat localOrientation;               ///< Child rotation in the compound frame.
    float radius = 0.5f;                 ///< Sphere/capsule radius.
    Vec3 halfExtents{0.5f, 0.5f, 0.5f};  ///< Box half extents.
    float capsuleHalfHeight = 0.5f; ///< Capsule/Cylinder: half height; Cone: half total height.
    std::vector<Vec3> hullPoints;   ///< Local-space points for a convex hull child.
};

/**
 * @brief Flattened runtime child: pointer-free and shared unchanged with CUDA.
 *
 * This is the GPU-friendly form of @ref CompoundShape; hull geometry is
 * referenced by index ranges into the World's mesh soup rather than by pointer.
 */
struct CompoundChild {
    ShapeType shape = ShapeType::Sphere; ///< Primitive type of this child.
    Vec3 localPosition;                  ///< Child offset in the compound frame.
    Quat localOrientation;               ///< Child rotation in the compound frame.
    float radius = 0.5f;                 ///< Sphere/capsule radius.
    Vec3 halfExtents{0.5f, 0.5f, 0.5f};  ///< Box half extents.
    float capsuleHalfHeight = 0.5f;      ///< Capsule/Cylinder/Cone half height.
    uint32_t hullFirst = 0, hullCount = 0;       ///< Hull point range in the soup.
    uint32_t hullFaceFirst = 0, hullFaceCount = 0; ///< Hull face range in the soup.
};

/**
 * @brief Data-oriented rigid body.
 *
 * Plain struct in a contiguous array so the same data can be uploaded to a
 * GPU backend unchanged. Application code normally reads a copy via
 * `World::bodyState` and mutates through `World` setters; the @ref VELOX_HD
 * helpers below are used by both the CPU and CUDA narrow/solver paths.
 *
 * Mass is stored as @ref invMass (`0` means static / infinite mass) and the
 * inertia as the diagonal inverse principal moments @ref invInertia plus the
 * principal frame @ref inertiaOrientation.
 */
struct alignas(64) Body {
    Vec3 position;            ///< World-space center of mass.
    Quat orientation;         ///< World-space orientation.
    Vec3 velocity;            ///< Linear velocity (m/s).
    Vec3 angularVelocity;     ///< Angular velocity (rad/s).
    Vec3 force;               ///< Accumulated force, cleared each step.
    Vec3 torque;              ///< Accumulated torque, cleared each step.
    float invMass = 1.0f;     ///< Inverse mass; 0 => static / infinite mass.
    Vec3 invInertia;          ///< Diagonal inverse principal moments.
    Quat inertiaOrientation;  ///< Principal-inertia frame to body-local frame.
    float restitution = 0.3f; ///< Bounciness, combined per @ref restitutionCombine.
    float friction = 0.5f;    ///< Coulomb friction, combined per @ref frictionCombine.
    Vec3 frictionScale{1, 1, 1}; ///< Local-space anisotropic friction multiplier.
    float rollingFriction = 0.0f;  ///< Angular resistance length scale.
    float spinningFriction = 0.0f; ///< Angular resistance length scale.
    MaterialCombineMode frictionCombine = MaterialCombineMode::GeometricMean;    ///< Friction combine rule.
    MaterialCombineMode restitutionCombine = MaterialCombineMode::Minimum;       ///< Restitution combine rule.
    float linearDamping = 0.0f;  ///< Exponential linear velocity damping.
    float angularDamping = 0.0f; ///< Exponential angular velocity damping.
    float gravityScale = 1.0f;   ///< Multiplier applied to world gravity.
    MotionType motionType = MotionType::Dynamic; ///< Static/Kinematic/Dynamic.
    BodyCcdTuning ccdTuning;     ///< Continuous collision detection tuning.
    GeometryQuality geometryQuality = GeometryQuality::Normal; ///< Convex geometry robustness.
    uint32_t categoryBits = 1u;  ///< Collision categories this body belongs to.
    uint32_t maskBits = UINT32_MAX; ///< Collision categories this body hits.
    int32_t groupIndex = 0;      ///< Same-group override: +N always collide, -N never, 0 = use layers.
    uint8_t sensor = 0;          ///< When non-zero, reports but does not resolve contacts.
    uint8_t enableSleep = 1;     ///< When 0, body never enters sleep state.
    uint8_t fixedRotation = 0;   ///< When 1, angular velocity is zeroed and orientation is locked.

    uint8_t asleep = 0;          ///< Sleeping bodies skip integration and solving.
    float sleepTimer = 0.0f;     ///< Seconds below the motion threshold.

    ShapeType shape = ShapeType::Sphere; ///< Active collider primitive.
    float radius = 0.5f;         ///< Sphere, Capsule (and hull bounding radius).
    Vec3 halfExtents{0.5f, 0.5f, 0.5f}; ///< Box half extents.
    float capsuleHalfHeight = 0.5f; ///< Capsule/Cylinder: half height; Cone: half total height.
    Vec3 planeNormal{0, 1, 0};   ///< Plane: dot(n, p) = planeOffset.
    float planeOffset = 0.0f;    ///< Plane distance from the origin along @ref planeNormal.
    uint32_t meshIndex = 0;      ///< Mesh: index into World's mesh storage.
    uint32_t hullFirst = 0, hullCount = 0; ///< Hull: local-space points in soup.
    uint32_t hullFaceFirst = 0, hullFaceCount = 0; ///< Packed QuickHull triangles.
    uint32_t compoundFirst = 0, compoundCount = 0; ///< Compound child range in the soup.

    /// @name State predicates
    /// @{
    VELOX_HD bool isStatic() const { return motionType == MotionType::Static; }
    VELOX_HD bool isKinematic() const { return motionType == MotionType::Kinematic; }
    VELOX_HD bool isDynamic() const { return motionType == MotionType::Dynamic; }
    VELOX_HD bool isSensor() const { return sensor != 0; }
    /// True when CCD quality is `Locked` (body is treated as immovable by the solver).
    VELOX_HD bool isLocked() const {
        return ccdTuning.quality == MotionQuality::Locked;
    }
    /// True when orientation and angular motion are constrained.
    VELOX_HD bool isRotationLocked() const {
        return isLocked() || fixedRotation != 0;
    }
    /**
     * @brief Test the group + category/mask collision filter against another body.
     *
     * Priority: when both bodies share a non-zero @ref groupIndex the sign
     * decides immediately (positive = collide, negative = skip). Otherwise
     * the bidirectional category/mask layer test applies.
     *
     * @param other The body to test against.
     * @return True when the two bodies may collide.
     */
    VELOX_HD bool canCollideWith(const Body& other) const {
        // Group override takes priority over layer bits.
        if (groupIndex != 0 && groupIndex == other.groupIndex)
            return groupIndex > 0;
        return (maskBits & other.categoryBits) != 0 &&
               (other.maskBits & categoryBits) != 0;
    }
    /// @}

    /// @name Inertia helpers
    /// @{
    /// Solver-facing inverse mass: `0` for static or locked bodies.
    VELOX_HD float solverInvMass() const {
        return isDynamic() && !isLocked() ? invMass : 0.0f;
    }

    /// Resolve the principal-inertia frame at an arbitrary orientation.
    VELOX_HD Quat inertiaFrameAt(const Quat& at) const {
        if (inertiaOrientation.x == 0.0f && inertiaOrientation.y == 0.0f &&
            inertiaOrientation.z == 0.0f && inertiaOrientation.w == 1.0f)
            return at;
        return mul(at, inertiaOrientation);
    }

    /// World-space inverse-inertia multiply: I⁻¹_world * v = R (I⁻¹_body (Rᵀ v)).
    VELOX_HD Vec3 invInertiaMulAt(const Vec3& v, const Quat& at) const {
        if (!isDynamic() || isRotationLocked()) return {};
        Quat frame = inertiaFrameAt(at);
        Vec3 local = rotateInv(frame, v);
        return rotate(frame, {local.x * invInertia.x,
                              local.y * invInertia.y,
                              local.z * invInertia.z});
    }

    /// Inverse-inertia multiply at the body's current orientation.
    VELOX_HD Vec3 invInertiaMul(const Vec3& v) const {
        return invInertiaMulAt(v, orientation);
    }

    /// Inertia (not inverse) multiply at the body's current orientation.
    VELOX_HD Vec3 inertiaMul(const Vec3& v) const {
        if (isRotationLocked()) return {};
        Quat frame = inertiaFrameAt(orientation);
        Vec3 local = rotateInv(frame, v);
        Vec3 weighted{
            invInertia.x > 0.0f ? local.x / invInertia.x : 0.0f,
            invInertia.y > 0.0f ? local.y / invInertia.y : 0.0f,
            invInertia.z > 0.0f ? local.z / invInertia.z : 0.0f};
        return rotate(frame, weighted);
    }

    /// World-space angular momentum `I_world * angularVelocity`.
    VELOX_HD Vec3 worldAngularMomentum() const {
        return inertiaMul(angularVelocity);
    }

    /// Rotational kinetic energy `0.5 * ω · L`.
    VELOX_HD float rotationalKineticEnergy() const {
        return 0.5f * dot(angularVelocity, worldAngularMomentum());
    }
    /// @}

    /**
     * @brief Advance orientation while preserving world angular momentum as an
     *        anisotropic inertia tensor rotates.
     *
     * Kinematic angular velocity is a prescribed trajectory and is therefore
     * not reprojected. Locked bodies are left unchanged.
     * @param dt Timestep in seconds.
     */
    VELOX_HD void advanceOrientation(float dt) {
        if (isRotationLocked()) {
            angularVelocity = {};
            return;
        }
        bool anisotropic = invInertia.x != invInertia.y ||
                           invInertia.y != invInertia.z;
        if (isDynamic() && anisotropic) {
            Vec3 angularMomentum = worldAngularMomentum();
            Quat start = orientation;
            Vec3 startVelocity = angularVelocity;
            Quat next = integrate(start, startVelocity, dt);
            // Implicit midpoint for q' = 0.5 * omega(q,L) * q. Three fixed
            // point iterations are enough at solver-substep timesteps.
            for (int iteration = 0; iteration < 3; ++iteration) {
                Vec3 endVelocity = invInertiaMulAt(angularMomentum, next);
                Vec3 midpointVelocity = (startVelocity + endVelocity) * 0.5f;
                next = integrate(start, midpointVelocity, dt);
            }
            orientation = next;
            angularVelocity = invInertiaMul(angularMomentum);
        } else {
            orientation = integrate(orientation, angularVelocity, dt);
        }
    }

    /**
     * @brief Integrate position and orientation forward by `dt`.
     * @param dt Timestep in seconds.
     */
    VELOX_HD void advanceTransform(float dt) {
        if (isLocked()) return;
        const Vec3 previous = position;
        const Vec3 delta = velocity * dt;
        position += delta;
        // At large world coordinates a valid substep displacement can be
        // smaller than one float ULP. Preserve forward progress instead of
        // silently freezing a moving body until an origin shift occurs.
        if (delta.x != 0.0f && position.x == previous.x)
            position.x = nextafterf(previous.x, delta.x > 0.0f ? FLT_MAX : -FLT_MAX);
        if (delta.y != 0.0f && position.y == previous.y)
            position.y = nextafterf(previous.y, delta.y > 0.0f ? FLT_MAX : -FLT_MAX);
        if (delta.z != 0.0f && position.z == previous.z)
            position.z = nextafterf(previous.z, delta.z > 0.0f ? FLT_MAX : -FLT_MAX);
        advanceOrientation(dt);
    }

    /**
     * @brief Conservative bound on how far any surface point can move per second.
     *
     * Linear speed plus angular speed times the shape's bounding radius. For
     * hulls, @ref radius holds the bounding radius (set at creation).
     * @return Maximum surface-point speed (world units / second).
     */
    VELOX_HD float maxPointSpeed() const {
        float r = radius;
        if (shape == ShapeType::Box) r = length(halfExtents);
        else if (shape == ShapeType::RoundedBox) r = length(halfExtents) + radius;
        else if (shape == ShapeType::Ellipsoid) r = vmax(halfExtents.x, vmax(halfExtents.y, halfExtents.z));
        else if (shape == ShapeType::Capsule) r = capsuleHalfHeight + radius;
        else if (shape == ShapeType::Cylinder)
            r = sqrtf(capsuleHalfHeight * capsuleHalfHeight + radius * radius);
        else if (shape == ShapeType::Cone) {
            float top = 1.5f * capsuleHalfHeight;
            r = vmax(top, sqrtf(radius * radius + 0.25f * capsuleHalfHeight * capsuleHalfHeight));
        }
        else if (shape == ShapeType::Compound) r = radius;
        return length(velocity) + length(angularVelocity) * r;
    }
};

/**
 * @brief Static triangle mesh collider (triangle soup with an AABB for culling).
 *
 * Mesh bodies are always static — this matches how game engines use non-convex
 * mesh colliders for level geometry. The flat arrays are stored by the World;
 * a @ref Body references them via `meshIndex`.
 */
struct Mesh {
    // Flat arrays: 3 floats per vertex, 3 indices per triangle.
    // Stored by World; Body references it via meshIndex.
    uint32_t firstVertex = 0, vertexCount = 0; ///< Vertex range in the soup.
    uint32_t firstIndex = 0, indexCount = 0;   ///< Index range in the soup.
    uint32_t firstNode = 0, nodeCount = 0;     ///< Triangle BVH (flat, GPU-traversable).
    uint32_t firstTriRef = 0;                  ///< Leaf triangle references.
    Vec3 aabbMin, aabbMax;                     ///< World-space bounds for culling.
};

/**
 * @brief Flat BVH node over a triangle mesh.
 *
 * Leaves hold a range of triangle references; inner nodes store the index of
 * their left child, and the right child is `left + 1`.
 */
struct BvhNode {
    Vec3 aabbMin, aabbMax;   ///< Node bounds.
    uint32_t leftFirst = 0;  ///< Inner: index of left child (right = left+1); leaf: first tri ref.
    uint32_t triCount = 0;   ///< 0 = inner node, >0 = leaf.
};

} // namespace velox
