#pragma once
#include "backend.h"
#include "collision_filter.h"
#include "joint.h"
#include "memory_pool.h"
#include "queries.h"
#include "sleep.h"
#include "softbody.h"
#include "solver.h"
#include "task_system.h"
#include "thread_safety.h"
#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace velox {

/**
 * @file world.h
 * @brief The World simulation container and its supporting event,
 *        configuration, and snapshot types.
 *
 * A World owns every body, joint, collision cache, and snapshot created
 * through it. Keep the generation-checked handles it returns in your gameplay
 * objects; do not cache references returned from `body()`/`joint()` across
 * structural changes or `step()` calls.
 *
 * @code
 * velox::World world(velox::BackendType::Cpu);
 * world.setGravity({0.0f, -9.81f, 0.0f});
 * world.addStaticPlane({0, 1, 0}, 0.0f);
 * velox::BodyId ball = world.addSphere({0, 5, 0}, 0.5f, 1.0f);
 * for (int i = 0; i < 600; ++i)
 *     world.step(1.0f / 60.0f);
 * @endcode
 */

/** @brief Lifecycle phase of a contact between two bodies. */
enum class ContactEventType : uint8_t { Begin, Persist, End };

// Stable geometric primitive tags used to form persistent contact keys.
/** @brief Geometric feature tags used to form persistent contact keys. */
enum class ContactFeature : uint32_t {
    None = 0,
    Vertex = 1u << 0,
    Edge = 1u << 1,
    Face = 1u << 2,
    Implicit = 1u << 30,
    Triangle = 1u << 31,
};

/**
 * @brief A body pair touching this step.
 *
 * End events carry handles and zero impulse; point/normal retain their default
 * values because the pair no longer meets.
 */
struct ContactEvent {
    BodyId a, b;          ///< The two bodies in contact.
    Vec3 point, normal;   ///< Representative contact; normal points from b to a.
    float impulse;        ///< Largest accumulated normal impulse in the manifold.
    ContactEventType type = ContactEventType::Begin; ///< Lifecycle phase.
    bool sensor = false;  ///< True when the contact involves a sensor body.
};

/** @brief Emitted when a joint exceeds its break force/torque threshold. */
struct JointBreakEvent {
    JointId joint; ///< Stale immediately after the break, identifies the removed joint.
    BodyId a, b;   ///< Bodies that were connected.
    float force = 0.0f;  ///< Reaction force magnitude at the break.
    float torque = 0.0f; ///< Reaction torque magnitude at the break.
};

/** @brief Lifecycle phase of a body reported by @ref BodyEvent. */
enum class BodyEventType : uint8_t { Created, Destroyed, Moved };

/** @brief A body lifecycle event from the most recent `step()`. */
struct BodyEvent {
    BodyEventType type;  ///< What happened to the body.
    BodyId body;         ///< Affected body handle.
    Vec3 position;       ///< Body position at the time of the event.
    Quat orientation;    ///< Body orientation at the time of the event.
};

/**
 * @brief Closest points between two bodies' surfaces.
 *
 * `distance` is negative when the bodies overlap. When either body is an
 * unbounded static (plane/mesh) paired with another unbounded static,
 * `distance` is a large sentinel (`1e30f`).
 */
struct ClosestPointResult {
    float distance = 0.0f; ///< Signed gap; negative means overlap.
    Vec3 pointA, pointB;   ///< Witness points on each body's surface.
    Vec3 normal;           ///< From B towards A.
};

/**
 * @brief Quality diagnostics for a body's convex geometry.
 *
 * Computed from the body's current local geometry. Querying by `BodyId` keeps
 * diagnostics tied to geometry owned by this World and validates stale handles.
 */
struct GeometryDiagnostics {
    float minEdgeLength = 0.0f;      ///< Shortest hull edge.
    float maxEdgeLength = 0.0f;      ///< Longest hull edge.
    float aspectRatio = 1.0f;        ///< Worst triangle aspect ratio.
    float volume = 0.0f;             ///< Enclosed volume.
    bool isDegenerate = false;       ///< True when the geometry is unusable.
    int nearCoplanarFaceCount = 0;   ///< Faces nearly coplanar with a neighbor.
};

/** @brief One hit produced by a multi-TOI query. */
struct MultiToiHit {
    float toi = 0.0f;    ///< Time of impact within the step.
    BodyId body;         ///< Body that was hit.
    Vec3 point;          ///< World-space contact point.
    Vec3 normal;         ///< From the hit body toward the queried body.
    float fraction = 0.0f; ///< Fraction of the step at impact.
};

/** @brief A single renderer-independent debug line segment. */
struct DebugLine {
    Vec3 a, b;                   ///< Segment endpoints (world space).
    uint32_t color = 0xffffffffu; ///< Packed `0xRRGGBBAA` color.
};

// Independently selectable debug-visualization layers. The line lists produced
// by World::debugLines() (and the DebugDraw interface in debug_draw.h) honor
// any combination of these bits.
/** @brief Bit flags selecting which layers `debugLines()` emits. */
enum DebugDrawFlags : uint32_t {
    DebugDrawShapes = 1u << 0,          ///< Collider wireframes (compound/mesh aware).
    DebugDrawAabbs = 1u << 1,           ///< Broad-phase axis-aligned bounding boxes.
    DebugDrawContacts = 1u << 2,        ///< Contact points + normals.
    DebugDrawJoints = 1u << 3,          ///< Joint anchors, links, and axes.
    DebugDrawVelocityVectors = 1u << 4, ///< Linear + angular velocity arrows.
    DebugDrawForceVectors = 1u << 5,    ///< Accumulated force + torque (pre-step).
    DebugDrawCenterOfMass = 1u << 6,    ///< Per-body center-of-mass frame axes.
    DebugDrawIslands = 1u << 7,         ///< Tint shapes by solver island.
    DebugDrawSleep = 1u << 8,           ///< Tint shapes by sleep state (awake/drowsy/asleep).
    // The original four-layer set; kept stable so existing callers are unchanged.
    DebugDrawAll = DebugDrawShapes | DebugDrawAabbs |
                   DebugDrawContacts | DebugDrawJoints,
    // Every available visualization layer.
    DebugDrawEverything = DebugDrawAll | DebugDrawVelocityVectors |
                          DebugDrawForceVectors | DebugDrawCenterOfMass |
                          DebugDrawIslands | DebugDrawSleep
};

/** @brief Mutable per-contact data handed to a @ref ContactModifier. */
struct ContactModifyData {
    BodyId a, b;          ///< The two bodies in contact.
    Vec3 point, normal;   ///< Contact point and normal (editable).
    float restitution = 0.0f; ///< Resolved restitution (editable).
    float friction1 = 0.0f, friction2 = 0.0f; ///< Resolved friction (editable).
    float rollingFriction = 0.0f, spinningFriction = 0.0f; ///< Angular friction (editable).
    bool enabled = true;  ///< Set false to disable this contact.
};

/**
 * @brief Callback invoked once per generated contact before solving.
 * @see World::setContactModifier
 */
using ContactModifier = std::function<void(ContactModifyData&)>;

/** @brief Timing and counting breakdown of the most recent `step()`. */
struct StepStats {
    float dt = 0.0f;                 ///< Timestep that was simulated.
    size_t bodyCount = 0;            ///< Total bodies in the world.
    size_t awakeDynamicBodies = 0;   ///< Dynamic bodies that were awake.
    size_t generatedContacts = 0;    ///< Contacts generated by narrow phase.
    size_t solvedContacts = 0;       ///< Contacts handed to the solver.
    uint32_t velocityIterations = 0; ///< Solver velocity iterations used.
    size_t jointCount = 0;           ///< Active joints.
    size_t multiToiEvents = 0;       ///< Conservative-advancement recoveries.
    bool deviceSubsteps = false;     ///< True when substeps ran on the GPU.
    double setupMs = 0.0;            ///< Step setup time (ms).
    double collisionDetectionMs = 0.0; ///< Total collision detection time (ms).
    double solverMs = 0.0;           ///< Total solver time (ms).
    double ccdMs = 0.0;              ///< CCD time (ms).
    double finalizeMs = 0.0;         ///< Finalization time (ms).
    double totalMs = 0.0;            ///< Whole-step wall time (ms).
    size_t broadPhaseProxies = 0;    ///< Broad-phase proxy count.
    size_t narrowPhaseTests = 0;     ///< Narrow-phase pair tests.
    size_t islandCount = 0;          ///< Solver island count.
    double broadPhaseMs = 0.0;       ///< Broad-phase time (ms).
    double narrowPhaseMs = 0.0;      ///< Narrow-phase time (ms).
    double contactSolverMs = 0.0;    ///< Contact solver time (ms).
    double jointSolverMs = 0.0;      ///< Joint solver time (ms).
    double ccdRecoveryMs = 0.0;      ///< CCD recovery time (ms).
    // Memory-pool accounting for the stepped world. These let callers observe
    // allocation overhead (and its absence) without a separate query.
    size_t memoryReservedBytes = 0;  ///< Bytes the world's pools hold from the OS.
    size_t memoryUsedBytes = 0;      ///< Bytes currently handed out by the pools.
    size_t memoryPeakBytes = 0;      ///< High-water mark of pool usage.
    double memoryFragmentation = 0.0;///< Internal fragmentation ratio `[0, 1]`.
    size_t bodyCapacity = 0;         ///< Pre-reserved body slots.
    size_t contactCapacity = 0;      ///< Pre-reserved contact slots.
    size_t jointCapacity = 0;        ///< Pre-reserved joint slots.
};

class World;

/**
 * @brief Copyable rollback point owned by the World that created it.
 *
 * Its contents are intentionally opaque so internal dense indices and cache
 * layouts can evolve. Create with `World::saveSnapshot()` and apply with
 * `World::restoreSnapshot()`.
 */
class WorldSnapshot {
public:
    WorldSnapshot() = default;

private:
    friend class World;
    friend struct SerializationAccess; // serialization.cpp: snapshot <-> bytes
    struct Slot { uint32_t dense = UINT32_MAX, generation = 0; };
    struct Previous { Vec3 position; Quat orientation; };

    const World* owner_ = nullptr;
    Vec3 gravity_;
    int substeps_ = 4;
    std::vector<Body> bodies_;
    std::vector<Slot> bodySlots_;
    std::vector<uint32_t> bodyDenseToSlot_, freeBodySlots_;
    std::vector<Contact> contacts_, prevContacts_;
    std::vector<Joint> joints_;
    std::vector<Slot> jointSlots_;
    std::vector<uint32_t> jointDenseToSlot_, freeJointSlots_;
    std::vector<Previous> previous_;
    std::vector<uint64_t> pairKeys_, previousPairKeys_;
    std::vector<uint32_t> unionParent_;
    std::vector<float> islandTimer_;
    std::vector<ContactEvent> contactEvents_;
    std::vector<JointBreakEvent> jointBreakEvents_;
    MeshSoup meshes_;
    StepStats lastStepStats_;
    WorldCcdDefaults ccdDefaults_{};
    WorldMultiToiSettings multiToiSettings_{};
    SolverOptions solverOptions_{};
};

struct BroadPhaseData;

/**
 * @brief CPU contact solving strategy.
 *
 * Parallel solves independent contact islands concurrently on the worker pool;
 * results are bitwise identical to Sequential because islands share no dynamic
 * bodies. The GPU backend uses graph coloring and ignores this setting.
 */
enum class IslandSolvingMode : uint8_t { Sequential = 0, Parallel = 1 };

/**
 * @brief Floating-point determinism policy.
 *
 * Relaxed mode favors throughput and may use platform-specific floating-point
 * contraction or the CUDA graph-colored solver. Strict mode is available only
 * in a `VELOX_STRICT_FLOATING_POINT` build and selects the ordered CPU
 * reference backend, making replay comparison meaningful across supported CPU
 * platforms.
 */
enum class DeterminismMode : uint8_t { Relaxed = 0, Strict = 1 };

/**
 * @brief Response to a recoverable CUDA allocation or device failure.
 *
 * CPU fallback reruns the failed frame from its pre-step snapshot; because the
 * CPU and CUDA solvers use different execution orders, the switch is not a
 * lockstep-deterministic transition.
 */
enum class DeviceLossPolicy : uint8_t { FallbackToCPU = 0, ThrowException = 1 };

/**
 * @brief Whether the CUDA backend keeps data resident on the GPU across steps.
 *
 * Disabled is the default and matches the original per-substep upload path.
 * Resident activates the fast path when the backend supports it; the world
 * silently falls back to the non-resident path when constraints prevent it
 * (unsupported joint types, adaptive iteration, shape mutations mid-step).
 */
enum class GPUResidentMode : uint8_t { Disabled = 0, Resident = 1 };

/**
 * @brief Controls which threads may enter World methods.
 *
 * Strict is the default and confines all access to the creating thread.
 * Relaxed permits cross-thread query calls, but mutations and `step()` remain
 * creator-thread only. Concurrent serializes the supported method API with an
 * internal lock; it is intended for tools and gameplay code that cannot
 * maintain an external world lock.
 */
enum class ThreadSafetyPolicy : uint8_t {
    Strict = 0,
    Relaxed = 1,
    Concurrent = 2,
};

/** @brief Counters describing how a world has been accessed across threads. */
struct ThreadSafetyReport {
    uint64_t queryCallsFromNonMainThread = 0;  ///< Cross-thread query calls.
    uint64_t mutationCallsDuringStep = 0;      ///< Rejected mutations during step.
    uint64_t stepInvocationsOnNonMainThread = 0; ///< Rejected foreign step calls.
};

/**
 * @brief Runtime replacement for convex primitive collider geometry.
 *
 * Hull and compound payloads are value-owned so callers can safely retain
 * their input after mutation returns. Apply with `World::mutateShape()`.
 */
struct ShapeMutation {
    /** @brief Primitive type to replace the body's collider with. */
    enum class Type : uint8_t { Sphere, Box, Capsule, Cylinder, Cone, Hull, Compound, RoundedBox, Ellipsoid };
    Type type = Type::Sphere;            ///< New collider type.
    float radius = 0.0f;                 ///< Sphere/capsule/rounded-box radius.
    Vec3 halfExtents{0.5f, 0.5f, 0.5f};  ///< Box/ellipsoid half extents.
    float capsuleHalfHeight = 0.0f;      ///< Capsule/cylinder half height.
    std::vector<Vec3> hullPoints;        ///< Local-space points for a hull.
    std::vector<CompoundShape> compoundShapes; ///< Children for a compound.
    bool preserveMassProperties = false; ///< Keep existing mass/inertia.
};

/** @brief Non-uniform scale applied to a body's collider via `World::scaleShape()`. */
struct ShapeScale {
    Vec3 factor{1.0f, 1.0f, 1.0f}; ///< Per-axis scale factor.
    bool updateMassProperties = true; ///< Recompute mass/inertia after scaling.
};

/**
 * @brief The physics simulation container.
 *
 * A World owns every body, joint, collision cache, and snapshot created
 * through it. It is created with `ThreadSafetyPolicy::Strict` (owner-thread
 * only); select another policy with `setThreadSafetyPolicy()` before sharing
 * it across threads.
 *
 * See the [API guide](../../docs/api-guide.md) for common patterns, threading
 * guidelines, and performance tips.
 */
class World {
public:
    /**
     * @brief Construct a world.
     *
     * Auto picks the NVIDIA CUDA backend when built with `VELOX_ENABLE_CUDA`
     * and a device is present, otherwise the portable CPU backend. Cuda throws
     * if unavailable.
     * @param type Backend to use (default `BackendType::Auto`).
     */
    explicit World(BackendType type = BackendType::Auto);
    ~World();

    /// @name Backend & determinism
    /// @{
    DeterminismMode determinismMode() const;
    /**
     * @brief Select the floating-point determinism policy.
     *
     * Strict mode requires a build configured with `VELOX_STRICT_FLOATING_POINT`.
     * It uses the CPU reference backend; CUDA strict parity is not yet supported.
     * @param mode Desired policy.
     */
    void setDeterminismMode(DeterminismMode mode);

    DeviceLossPolicy deviceLossPolicy() const;
    void setDeviceLossPolicy(DeviceLossPolicy policy); ///< Set the CUDA device-loss response.
    GPUResidentMode gpuResidentMode() const;
    void setGPUResidentMode(GPUResidentMode mode); ///< Enable/disable GPU-resident data.
    /**
     * @brief Whether the active backend is the portable CPU implementation.
     *
     * True including an Auto/CUDA world that fell back after a recoverable
     * failure.
     */
    bool isOnCPUBackend() const;
    /**
     * @brief Recreate the CUDA backend after CPU fallback.
     * @return False when no usable CUDA device is available; call during a
     *         non-critical frame.
     */
    bool resetCUDABackend();
    /// @}

    /// @name Solver & CCD configuration
    /// @{
    WorldCcdDefaults ccdDefaults() const;
    void setCcdDefaults(WorldCcdDefaults defaults); ///< Set world-wide CCD defaults.
    SolverOptions solverOptions() const;
    void setSolverOptions(SolverOptions options); ///< Set solver iteration options.
    WorldMultiToiSettings multiToiSettings() const;
    void setMultiToiSettings(WorldMultiToiSettings settings); ///< Set multi-TOI settings.
    void setCcdTuning(BodyId id, BodyCcdTuning tuning); ///< Override CCD tuning for one body.
    BodyCcdTuning ccdTuning(BodyId id) const;          ///< Get a body's CCD tuning.
    /// Predicted time-of-impact hits for a body over the next `dt`.
    std::vector<MultiToiHit> queryMultiToi(BodyId id, float dt) const;

    IslandSolvingMode islandSolvingMode() const;
    void setIslandSolvingMode(IslandSolvingMode mode); ///< Select Sequential/Parallel islands.
    /// @}

    /// @name Thread safety
    /// @{
    ThreadSafetyPolicy threadSafetyPolicy() const;
    void setThreadSafetyPolicy(ThreadSafetyPolicy policy); ///< Set the cross-thread access policy.
    ThreadSafetyReport threadSafetyReport() const; ///< Access-pattern counters.
    /// @}

    /// @name Workers & task system
    /// @{
    const char* backendName() const; ///< Human-readable active backend name.
    void setWorkerCount(uint32_t count); ///< Set the internal worker-pool size.
    uint32_t workerCount() const;        ///< Current worker-pool size.
    /**
     * @brief Inject an external task system for parallel work distribution.
     *
     * Pass `nullptr` to revert to the internal worker pool. The TaskSystem must
     * remain alive for the lifetime of the World (or until replaced).
     * @param system External task system, or `nullptr`.
     */
    void setTaskSystem(TaskSystem* system);
    TaskSystem* taskSystem() const; ///< The active external task system, if any.
    /// @}

    Vec3 gravity{0, -9.81f, 0}; ///< World gravity (m/s²). Configure before sharing.

    /// Solver substeps per step() call. More substeps = stiffer stacks and
    /// less friction drift for the same iteration budget (Box2D v3 approach).
    int substeps = 4;

    /// @name Thread-safe configuration accessors
    /// Safe configuration alternatives for use after publishing a World to
    /// another thread. Direct writes to gravity/substeps must be externally
    /// synchronized for compatibility with the pre-1.0 public fields.
    /// @{
    void setGravity(Vec3 value);   ///< Thread-safe gravity setter.
    Vec3 gravityValue() const;     ///< Thread-safe gravity getter.
    void setSubsteps(int value);   ///< Thread-safe substep setter.
    int substepCount() const;      ///< Thread-safe substep getter.
    /// @}

    /// @name Body creation
    /// Each returns a generation-checked `BodyId`. Pass `mass > 0` for a
    /// dynamic body; static geometry uses the dedicated `addStatic*` methods.
    /// @{
    BodyId addSphere(Vec3 position, float radius, float mass);
    BodyId addBox(Vec3 position, Vec3 halfExtents, float mass);
    BodyId addCapsule(Vec3 position, float radius, float halfHeight, float mass);
    BodyId addCylinder(Vec3 position, float radius, float halfHeight, float mass);
    BodyId addCone(Vec3 position, float radius, float height, float mass);
    BodyId addRoundedBox(Vec3 position, Vec3 halfExtents, float radius, float mass);
    BodyId addEllipsoid(Vec3 position, Vec3 radii, float mass);
    /**
     * @brief Add a convex hull from a local-space point cloud.
     *
     * Interior points are excluded from mass integration but still cost
     * support-function time.
     */
    BodyId addConvexHull(Vec3 position, const std::vector<Vec3>& points, float mass);
    /**
     * @brief Add a compound rigid body from convex children.
     *
     * Dynamic compounds use uniform density across child volumes. Geometry is
     * recentered without moving it in world space, so `Body::position` is the
     * computed center of mass rather than the authored compound origin.
     */
    BodyId addCompound(Vec3 position, const std::vector<CompoundShape>& shapes,
                       float mass);
    BodyId addStaticPlane(Vec3 normal, float offset);
    /**
     * @brief Add a static triangle mesh (level geometry).
     * @param vertices xyz triples.
     * @param indices  3 per triangle.
     */
    BodyId addStaticMesh(const std::vector<Vec3>& vertices,
                         const std::vector<uint32_t>& indices);
    /**
     * @brief Add a static heightfield.
     *
     * Row-major heights over `width x depth` samples. The grid lies in local
     * X/Z; `origin` is the world position of sample (0, 0).
     */
    BodyId addStaticHeightfield(uint32_t width, uint32_t depth, float cellSize,
                                const std::vector<float>& heights, Vec3 origin = {});
    /// @}

    /// @name Body access & mutation
    /// @{
    Body& body(BodyId id);             ///< Borrowed reference; do not retain across step/mutation.
    const Body& body(BodyId id) const; ///< Borrowed reference; do not retain across step/mutation.
    /// Thread-safe copies. body()/joint() return borrowed references and must
    /// not outlive external synchronization with step() or mutation.
    Body bodyState(BodyId id) const;   ///< Thread-safe value copy of a body.
    GeometryDiagnostics queryGeometryDiagnostics(BodyId id) const; ///< Convex geometry diagnostics.
    size_t bodyCount() const;          ///< Number of bodies currently alive.
    bool isValid(BodyId id) const;     ///< True when the handle still refers to a live body.
    void removeBody(BodyId id);        ///< Also removes joints attached to the body.
    MotionType motionType(BodyId id) const;
    void setMotionType(BodyId id, MotionType type); ///< Switch Static/Kinematic/Dynamic.
    /**
     * @brief Replace collider geometry without changing the handle or joints.
     *
     * Hull and compound payloads are built and appended transactionally.
     */
    void mutateShape(BodyId id, const ShapeMutation& mutation);
    void scaleShape(BodyId id, const ShapeScale& scale); ///< Non-uniformly scale a collider.
    void setCollisionMargin(BodyId id, float margin);    ///< Set the collision margin.
    /**
     * @brief Override mass and principal moments/axes without changing motion type.
     * @param id                   Target body handle.
     * @param mass                 New mass (kg).
     * @param principalInertia     Diagonal principal moments of inertia.
     * @param principalOrientation Rotates the principal frame into body-local space.
     */
    void setMassProperties(BodyId id, float mass, Vec3 principalInertia,
                           Quat principalOrientation = {});
    void setTransform(BodyId id, Vec3 position, Quat orientation); ///< Teleport a body.
    void setLinearVelocity(BodyId id, Vec3 velocity);
    void setAngularVelocity(BodyId id, Vec3 velocity);
    void addForce(BodyId id, Vec3 force);                 ///< Accumulate a center-of-mass force.
    void addForceAtPoint(BodyId id, Vec3 force, Vec3 worldPoint); ///< Force at a world point (adds torque).
    void addTorque(BodyId id, Vec3 torque);               ///< Accumulate a torque.
    void addLinearImpulse(BodyId id, Vec3 impulse);       ///< Instantaneous center-of-mass impulse.
    void addImpulseAtPoint(BodyId id, Vec3 impulse, Vec3 worldPoint); ///< Impulse at a world point.
    void clearForces(BodyId id);                          ///< Zero accumulated force/torque.
    void setSensor(BodyId id, bool enabled);              ///< Toggle sensor mode.
    bool isSensor(BodyId id) const;
    void setGravityScale(BodyId id, float scale);         ///< Per-body gravity multiplier.
    float gravityScale(BodyId id) const;
    void setLinearDamping(BodyId id, float damping);
    void setAngularDamping(BodyId id, float damping);
    void setCollisionFilter(BodyId id, uint32_t categoryBits, uint32_t maskBits); ///< Set category/mask bits.
    void setCollisionFilter(BodyId id, const CollisionFilterData& filter); ///< Set category/mask bits and group index.
    CollisionFilterData collisionFilter(BodyId id) const; ///< Get a body's complete filter configuration.
    void setEnableSleep(BodyId id, bool enabled);         ///< Allow/forbid sleeping.
    bool isSleepEnabled(BodyId id) const;
    void setFixedRotation(BodyId id, bool enabled);       ///< Lock/unlock rotation.
    bool isFixedRotation(BodyId id) const;
    void wakeBody(BodyId id);                             ///< Force a body awake.
    void sleepBody(BodyId id);                            ///< Force a body to sleep.
    /**
     * @brief Apply a radial impulse from origin to all dynamic bodies within radius.
     *
     * Falloff is linear: full impulse at center, zero at radius boundary.
     */
    void explode(Vec3 origin, float radius, float impulse);
    /**
     * @brief Subtract offset from every world-space coordinate.
     *
     * Preserves relative geometry and dynamics. Use this to keep large worlds
     * near the floating-point origin without rebuilding bodies, joints, or meshes.
     */
    void shiftOrigin(Vec3 offset);
    WorldSnapshot saveSnapshot() const;              ///< Capture a rollback point.
    void restoreSnapshot(const WorldSnapshot& snapshot); ///< Roll back to a snapshot.
    const StepStats& lastStepStats() const { return lastStepStats_; } ///< Borrowed timing stats.
    StepStats lastStepStatsCopy() const;             ///< Thread-safe copy of timing stats.
    /// @}

    /// @name Soft bodies
    /// @{
    SoftBodyId addSoftBody(const SoftBodyDesc& desc);
    SoftBody& softBody(SoftBodyId id);
    const SoftBody& softBody(SoftBodyId id) const;
    void removeSoftBody(SoftBodyId id);
    size_t softBodyCount() const;
    bool isValid(SoftBodyId id) const;
    /// @}

    /// @name Memory pools
    /// The world pre-reserves body/contact/joint capacity and routes scratch
    /// allocations through pooled allocators so a running simulation does not
    /// touch the system allocator on the hot path. These accessors expose the
    /// pool accounting; the most recent snapshot is also carried in StepStats.
    /// @{
    /**
     * @brief Aggregate pool accounting: reserved/used bytes, peak, and
     *        internal fragmentation across the world's pools and pre-reserved
     *        body/contact/joint storage.
     */
    MemoryPoolStats memoryStats() const;
    /// The general-purpose pool backing query-result and scratch buffers.
    MemoryPool& memoryPool() { return memoryPool_; }
    const MemoryPool& memoryPool() const { return memoryPool_; }
    /**
     * @brief Pre-reserve capacity so subsequent body/contact/joint creation
     *        and stepping do not reallocate.
     * @param bodies   Minimum body slots to keep warm.
     * @param contacts Minimum contact slots to keep warm.
     * @param joints   Minimum joint slots to keep warm.
     */
    void reserveCapacity(size_t bodies, size_t contacts, size_t joints);
    /**
     * @brief Acquire a pool-backed scratch buffer for query results.
     *
     * Reuses warm slab blocks instead of malloc/free per query. Release with
     * `releaseQueryBuffer(ptr, bytes)` passing the same `bytes`.
     * @param bytes Requested size in bytes.
     * @return A non-null buffer of at least `bytes` bytes.
     */
    void* acquireQueryBuffer(size_t bytes);
    /// Release a buffer obtained from `acquireQueryBuffer`.
    void releaseQueryBuffer(void* ptr, size_t bytes);
    /// @}

    /// @name Joints
    /// @{
    JointId addBallJoint(BodyId a, BodyId b, Vec3 worldAnchor);
    JointId addDistanceJoint(BodyId a, BodyId b, Vec3 worldAnchorA, Vec3 worldAnchorB);
    JointId addSpringJoint(BodyId a, BodyId b, Vec3 worldAnchorA, Vec3 worldAnchorB,
                           float frequencyHz, float dampingRatio);
    JointId addHingeJoint(BodyId a, BodyId b, Vec3 worldAnchor, Vec3 worldAxis);
    JointId addConeTwistJoint(BodyId a, BodyId b, Vec3 worldAnchor, Vec3 worldAxis);
    JointId addFixedJoint(BodyId a, BodyId b, Vec3 worldAnchor);
    JointId addPrismaticJoint(BodyId a, BodyId b, Vec3 worldAnchor, Vec3 worldAxis);
    JointId addSixDofJoint(BodyId a, BodyId b, Vec3 worldAnchor);
    /**
     * @brief Add a motor joint driving body B towards a target transform.
     *
     * maxForce/maxTorque clamp the corrective impulses each step.
     */
    JointId addMotorJoint(BodyId a, BodyId b, Vec3 worldAnchorA, Vec3 worldAnchorB,
                          float maxForce, float maxTorque);
    /**
     * @brief Add a weld joint: rigid 6-DOF lock with explicit break thresholds.
     *
     * weldFrequencyHz > 0 enables a soft angular constraint (spring-damper).
     */
    JointId addWeldJoint(BodyId a, BodyId b, Vec3 worldAnchor,
                         float breakForce = 3.402823466e+38F,
                         float breakTorque = 3.402823466e+38F);
    /**
     * @brief Add a wheel joint: suspension spring along worldAxis, free spin.
     *
     * Body A is the chassis, body B is the wheel. Configure suspension
     * frequency/damping and steering via the returned Joint reference.
     */
    JointId addWheelJoint(BodyId a, BodyId b, Vec3 worldAnchor, Vec3 worldAxis);
    /**
     * @brief Add a rope joint: maximum-distance constraint with slack.
     *
     * Bodies closer than maxLength are unconstrained; the rope only resists
     * stretching beyond maxLength.
     */
    JointId addRopeJoint(BodyId a, BodyId b, Vec3 worldAnchorA, Vec3 worldAnchorB,
                         float maxLength);
    /**
     * @brief Add a pulley joint: two bodies linked through ground anchors.
     *
     * lengthA + ratio * lengthB = constant. ratio > 1 gives mechanical
     * advantage to body B.
     */
    JointId addPulleyJoint(BodyId a, BodyId b, Vec3 worldAnchorA, Vec3 worldAnchorB,
                           Vec3 groundAnchorA, Vec3 groundAnchorB, float ratio);
    /**
     * @brief Add a gear joint: couples angular velocity at a fixed ratio.
     *
     * wB about worldAxisB = -ratio * wA about worldAxisA.
     */
    JointId addGearJoint(BodyId a, BodyId b, Vec3 worldAxisA, Vec3 worldAxisB,
                         float ratio);
    Joint& joint(JointId id);              ///< Configure motors/limits (borrowed reference).
    const Joint& joint(JointId id) const;  ///< Borrowed reference.
    Joint jointState(JointId id) const;    ///< Thread-safe value copy of a joint.
    bool isValid(JointId id) const;        ///< True when the joint handle is live.
    void removeJoint(JointId id);
    float hingeAngle(JointId id) const;    ///< Radians, 0 at creation.
    float coneSwingAngle(JointId id) const;
    float coneTwistAngle(JointId id) const;
    float prismaticTranslation(JointId id) const;
    Vec3 sixDofLinearTranslation(JointId id) const;
    Vec3 sixDofAngularRotation(JointId id) const;
    /// Joint break events from the most recent step().
    const std::vector<JointBreakEvent>& jointBreakEvents() const {
        return jointBreakEvents_;
    }
    /// @}

    /// Body lifecycle events from the most recent step().
    const std::vector<BodyEvent>& bodyEvents() const { return bodyEvents_; }

    /// Contact and sensor Begin/Persist/End events from the most recent step().
    const std::vector<ContactEvent>& contactEvents() const { return events_; }
    /**
     * @brief Install a per-contact modifier callback.
     *
     * Runs once per generated contact before waking, warm starting, and solving.
     * The callback may change the point, normal, resolved material values, or
     * disable the contact. Throwing aborts step() before body transforms advance.
     */
    void setContactModifier(ContactModifier modifier);

    /**
     * @brief Install a custom collision filter callback.
     *
     * The callback is invoked once per broadphase candidate pair, after the
     * built-in group + layer test. It can override the result with
     * @ref FilterResult::Accept or @ref FilterResult::Reject, or defer to
     * the built-in rules with @ref FilterResult::Default.
     *
     * Pass an empty `std::function` (or `nullptr`) to remove the callback.
     *
     * @note CPU broadphase only. GPU-resident stepping uses built-in rules.
     * @see CollisionFilterCallback, FilterResult
     */
    void setCollisionFilterCallback(CollisionFilterCallback callback);

    /// @name Sleeping
    /// Bodies whose island stays below configurable motion thresholds for a
    /// while are put to sleep (zero cost until something touches them). An
    /// optional drowsy intermediate state reduces the simulation rate before
    /// full sleep. Call wake() after manually changing a sleeping body's
    /// velocity or position.
    ///
    /// Configure thresholds via `sleepConfig()`, read diagnostics via
    /// `sleepStats()` and `sleepIslands()`, and install transition callbacks
    /// via `setSleepCallbacks()`.
    /// @{
    void wake(BodyId id);       ///< Wake a sleeping or drowsy body.
    bool isAwake(BodyId id) const; ///< True when the body is fully awake.

    /// @brief Mutable access to the sleep configuration.
    ///
    /// Call `sleepConfig().validate()` after modifying fields directly, or
    /// use `setSleepConfig()` which validates automatically.
    SleepConfig& sleepConfig() { return sleepManager_.config(); }

    /// @brief Read-only access to the sleep configuration.
    const SleepConfig& sleepConfig() const { return sleepManager_.config(); }

    /// @brief Replace the entire sleep configuration (validated).
    void setSleepConfig(SleepConfig config) { sleepManager_.setConfig(std::move(config)); }

    /// @brief Statistics from the most recent sleep update.
    const SleepStats& sleepStats() const { return sleepManager_.stats(); }

    /// @brief Island snapshots from the most recent sleep update.
    const std::vector<SleepIsland>& sleepIslands() const { return sleepManager_.islands(); }

    /// @brief Install sleep/wake/drowsy transition callbacks.
    void setSleepCallbacks(SleepCallbacks callbacks) { sleepManager_.setCallbacks(std::move(callbacks)); }

    /// @brief Query a body's current sleep state.
    SleepState sleepState(BodyId id) const;
    /// @}

    /// @name Queries
    /// @{
    /**
     * @brief Cast a ray and return the nearest hit.
     * @param origin  Ray origin (world space).
     * @param dir     Ray direction.
     * @param maxDist Maximum ray length.
     * @param filter  Category/mask and sensor filtering.
     * @return A @ref RayHit; check `hit` before use.
     */
    RayHit rayCast(Vec3 origin, Vec3 dir, float maxDist,
                   const QueryFilter& filter = {}) const;
    /**
     * @brief Cast a ray and collect every hit within maxDist, nearest-first.
     *
     * Each body appears at most once (its nearest intersection).
     */
    void rayCastAll(Vec3 origin, Vec3 dir, float maxDist,
                    std::vector<RayHit>& out,
                    const QueryFilter& filter = {}) const;
    ClosestPointResult closestPoints(BodyId a, BodyId b) const; ///< Closest points between two bodies.
    void overlapSphere(Vec3 center, float radius, std::vector<BodyId>& out,
                       const QueryFilter& filter = {}) const;
    void overlapBox(Vec3 center, Vec3 halfExtents, Quat orientation,
                    std::vector<BodyId>& out,
                    const QueryFilter& filter = {}) const;
    void overlapCapsule(Vec3 center, float radius, float halfHeight,
                        Quat orientation, std::vector<BodyId>& out,
                        const QueryFilter& filter = {}) const;
    void overlapConvexHull(Vec3 center, const std::vector<Vec3>& points,
                           Quat orientation, std::vector<BodyId>& out,
                           const QueryFilter& filter = {}) const;
    ShapeCastHit sphereCast(Vec3 center, float radius, Vec3 direction,
                            float maxDist, const QueryFilter& filter = {}) const;
    ShapeCastHit boxCast(Vec3 center, Vec3 halfExtents, Quat orientation,
                         Vec3 direction, float maxDist,
                         const QueryFilter& filter = {}) const;
    ShapeCastHit capsuleCast(Vec3 center, float radius, float halfHeight,
                             Quat orientation, Vec3 direction, float maxDist,
                             const QueryFilter& filter = {}) const;
    ShapeCastHit convexHullCast(Vec3 center, const std::vector<Vec3>& points,
                                Quat orientation, Vec3 direction, float maxDist,
                                const QueryFilter& filter = {}) const;
    /**
     * @brief Synchronously resolve value-owned requests against one consistent state.
     *
     * Invalid individual requests produce `success=false` and an error; they do
     * not abort the rest of the batch.
     */
    void batchQueries(const std::vector<QueryDesc>& queries,
                      std::vector<QueryResult>& outResults) const;
    /**
     * @brief Thread-safe, non-blocking async query submission.
     *
     * Requests are resolved at the deterministic beginning of the next
     * owner-thread step(), before any simulation state changes. This is
     * intentionally allowed even in Strict thread-safety mode because it copies
     * no World state at submission time.
     */
    AsyncQueryHandle submitAsyncQuery(const QueryDesc& query);
    /**
     * @brief Block until an async query has been resolved.
     *
     * A handle may be consumed once; callers must externally synchronize World
     * destruction.
     */
    QueryResult getAsyncResult(AsyncQueryHandle handle);
    /// Export renderer-independent debug lines for shapes/contacts/AABBs/joints.
    void debugLines(std::vector<DebugLine>& out,
                    uint32_t flags = DebugDrawAll) const;
    /// @}

    /**
     * @brief Advance the simulation by one fixed timestep.
     *
     * Advances the simulation using Predictive Contact Sweeping: speculative
     * contacts solved iteratively, backed by a conservative-advancement sweep
     * safety net, so no velocity can tunnel through geometry and grazing
     * contact stays smooth.
     * @param dt Timestep in seconds (use a fixed value).
     */
    void step(float dt);

private:
    friend struct SerializationAccess; // serialization.cpp: dense state access

    struct HandleSlot {
        uint32_t dense = UINT32_MAX;
        uint32_t generation = 0;
    };

    BodyIndex resolve(BodyId id) const;
    uint32_t resolve(JointId id) const;
    BodyId addBody(Body body);
    JointId addJoint(Joint joint);
    BodyId bodyHandle(BodyIndex dense) const;
    bool queryAllows(BodyIndex dense, const QueryFilter& filter) const;
    void overlapShape(const Body& shape, std::vector<BodyId>& out,
                      const QueryFilter& filter) const;
    void overlapShapeWithSoup(const Body& shape, std::vector<BodyId>& out,
                              const QueryFilter& filter,
                              const MeshSoupView& soup) const;
    ShapeCastHit castShape(Body shape, Vec3 direction, float maxDist,
                           const QueryFilter& filter) const;
    ShapeCastHit castShapeWithSoup(Body shape, Vec3 direction, float maxDist,
                                   const QueryFilter& filter,
                                   const MeshSoupView& soup) const;
    QueryResult executeQuery(const QueryDesc& query) const;
    bool asyncQueryReady(uint64_t id) const VELOX_REQUIRES(asyncQueryMutex_);
    void waitForAsyncQuery(ScopedLock<ThreadSafeMutex>& lock, uint64_t id)
        VELOX_REQUIRES(asyncQueryMutex_) VELOX_NO_THREAD_SAFETY_ANALYSIS;
    void drainAsyncQueries();
    void removeJointDense(uint32_t dense);
    // Incremental broad phase: rebuild/refit the AABB tree as needed.
    // `refit` forces a proxy refresh even without recorded mutations (used by
    // step(), where integration always moves bodies).
    void ensureBroadPhase(float dt, bool refit) const;
    void buildCandidatePairs(float dt);
    void solveJoints(float dt);
    void finishBrokenJoints(float dt);
    void updateSleeping(float dt);
    struct StepRollback;
    StepRollback saveStepRollback() const;
    void restoreStepRollback(StepRollback&& rollback);
    void stepImpl(float dt);
    void resetBackend(BackendType type);
    void activateCpuFallback(const BackendFailure& failure);
    enum class AccessKind : uint8_t { Query, Mutation, Step };
    class AccessGuard {
    public:
        AccessGuard(const World& world, AccessKind kind, const char* method);

    private:
        const World& world_;
        ScopedLock<ThreadSafeRecursiveMutex> lock_;
    };

    std::vector<Body> bodies_;
    std::vector<HandleSlot> bodySlots_;
    std::vector<uint32_t> bodyDenseToSlot_;
    std::vector<uint32_t> freeBodySlots_;
    std::vector<Contact> contacts_;
    std::vector<Joint> joints_;
    std::vector<HandleSlot> jointSlots_;
    std::vector<uint32_t> jointDenseToSlot_;
    std::vector<uint32_t> freeJointSlots_;
    struct PrevState { Vec3 position; Quat orientation; };
    std::vector<PrevState> prev_;
    std::vector<uint64_t> pairKeys_;
    std::vector<Contact> prevContacts_;  // sorted by pair key (warm starting)
    std::vector<uint32_t> unionParent_;
    std::vector<float> islandTimer_;
    std::vector<ContactEvent> events_;
    std::vector<JointBreakEvent> jointBreakEvents_;
    std::vector<BodyEvent> bodyEvents_;
    std::vector<BodyEvent> pendingBodyEvents_;
    std::vector<uint64_t> prevPairKeys_;
    ContactModifier contactModifier_;
    CollisionFilterCallback collisionFilterCallback_;
    IslandSolvingMode islandSolvingMode_ = IslandSolvingMode::Parallel;
    DeterminismMode determinismMode_ = DeterminismMode::Relaxed;
    DeviceLossPolicy deviceLossPolicy_ = DeviceLossPolicy::FallbackToCPU;
    GPUResidentMode gpuResidentMode_ = GPUResidentMode::Disabled;
    TaskSystem* taskSystem_ = nullptr;
    bool fallbackToCPU_ = false;
    WorldCcdDefaults ccdDefaults_;
    WorldMultiToiSettings multiToiSettings_;
    SolverOptions solverOptions_;
    BackendType requestedBackend_ = BackendType::Auto;
    StepStats lastStepStats_{};
    // General-purpose pool for query-result and scratch buffers. Pre-reserved
    // body/contact/joint vector capacity (see reserveCapacity) keeps the dense
    // arrays from reallocating during stepping; this pool handles the irregular
    // sizes that do not fit the fixed records.
    MemoryPool memoryPool_;
    MeshSoup meshes_;
    std::vector<uint64_t> candidatePairs_;
    mutable std::unique_ptr<BroadPhaseData> broadPhase_;
    std::unique_ptr<Backend> backend_;
    SleepManager sleepManager_; ///< Island-based sleep system with gradual sleep.
    std::vector<SoftBody> softBodies_;
    std::vector<HandleSlot> softBodySlots_;
    std::vector<uint32_t> softBodyDenseToSlot_;
    std::vector<uint32_t> freeSoftBodySlots_;
    // Reentrant world lock serializing every supported entry point under the
    // Relaxed/Concurrent thread-safety policies. Marked as a Clang Thread
    // Safety Analysis capability so the members it protects can be annotated.
    mutable ThreadSafeRecursiveMutex accessMutex_;
    struct PendingAsyncQuery { uint64_t id; QueryDesc query; };
    struct AsyncQueryResult { bool ready = false; QueryResult result; };
    // Owns the async-query bookkeeping below. Distinct from accessMutex_ so a
    // foreign thread can submit a query without taking the world lock.
    mutable ThreadSafeMutex asyncQueryMutex_;
    std::condition_variable_any asyncQueryReady_;
    std::atomic<bool> hasPendingAsyncQueries_{false};
    uint64_t nextAsyncQueryId_ VELOX_GUARDED_BY(asyncQueryMutex_) = 1;
    std::deque<PendingAsyncQuery> pendingAsyncQueries_ VELOX_GUARDED_BY(asyncQueryMutex_);
    std::unordered_map<uint64_t, AsyncQueryResult> asyncQueryResults_ VELOX_GUARDED_BY(asyncQueryMutex_);
    std::thread::id ownerThread_;
    ThreadSafetyPolicy threadSafetyPolicy_ = ThreadSafetyPolicy::Strict;
    mutable ThreadSafetyReport threadSafetyReport_;
};

} // namespace velox
