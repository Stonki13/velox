#pragma once
#include "backend.h"
#include "joint.h"
#include "queries.h"
#include "solver.h"
#include "task_system.h"
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

enum class ContactEventType : uint8_t { Begin, Persist, End };

// Stable geometric primitive tags used to form persistent contact keys.
enum class ContactFeature : uint32_t {
    None = 0,
    Vertex = 1u << 0,
    Edge = 1u << 1,
    Face = 1u << 2,
    Implicit = 1u << 30,
    Triangle = 1u << 31,
};

// A body pair touching this step. End events carry handles and zero impulse;
// point/normal retain their default values because the pair no longer meets.
struct ContactEvent {
    BodyId a, b;
    Vec3 point, normal;   // representative contact; normal points from b to a
    float impulse;        // largest accumulated normal impulse in the manifold
    ContactEventType type = ContactEventType::Begin;
    bool sensor = false;
};

struct JointBreakEvent {
    JointId joint; // stale immediately after the break, identifies the removed joint
    BodyId a, b;
    float force = 0.0f;
    float torque = 0.0f;
};

enum class BodyEventType : uint8_t { Created, Destroyed, Moved };

struct BodyEvent {
    BodyEventType type;
    BodyId body;
    Vec3 position;
    Quat orientation;
};

// Closest points between two bodies' surfaces. distance is negative when the
// bodies overlap. When either body is an unbounded static (plane/mesh) paired
// with another unbounded static, distance is a large sentinel (1e30f).
struct ClosestPointResult {
    float distance = 0.0f;
    Vec3 pointA, pointB; // witness points on each body's surface
    Vec3 normal;         // from B towards A
};

// Computed from the body's current local geometry. Querying by BodyId keeps
// diagnostics tied to geometry owned by this World and validates stale handles.
struct GeometryDiagnostics {
    float minEdgeLength = 0.0f;
    float maxEdgeLength = 0.0f;
    float aspectRatio = 1.0f;
    float volume = 0.0f;
    bool isDegenerate = false;
    int nearCoplanarFaceCount = 0;
};

struct MultiToiHit {
    float toi = 0.0f;
    BodyId body;
    Vec3 point;
    Vec3 normal; // from the hit body toward the queried body
    float fraction = 0.0f;
};

struct DebugLine {
    Vec3 a, b;
    uint32_t color = 0xffffffffu; // 0xRRGGBBAA
};

enum DebugDrawFlags : uint32_t {
    DebugDrawShapes = 1u << 0,
    DebugDrawAabbs = 1u << 1,
    DebugDrawContacts = 1u << 2,
    DebugDrawJoints = 1u << 3,
    DebugDrawAll = DebugDrawShapes | DebugDrawAabbs |
                   DebugDrawContacts | DebugDrawJoints
};

struct ContactModifyData {
    BodyId a, b;
    Vec3 point, normal;
    float restitution = 0.0f;
    float friction1 = 0.0f, friction2 = 0.0f;
    float rollingFriction = 0.0f, spinningFriction = 0.0f;
    bool enabled = true;
};

using ContactModifier = std::function<void(ContactModifyData&)>;

struct StepStats {
    float dt = 0.0f;
    size_t bodyCount = 0;
    size_t awakeDynamicBodies = 0;
    size_t generatedContacts = 0;
    size_t solvedContacts = 0;
    uint32_t velocityIterations = 0;
    size_t jointCount = 0;
    size_t multiToiEvents = 0;
    bool deviceSubsteps = false;
    double setupMs = 0.0;
    double collisionDetectionMs = 0.0;
    double solverMs = 0.0;
    double ccdMs = 0.0;
    double finalizeMs = 0.0;
    double totalMs = 0.0;
    size_t broadPhaseProxies = 0;
    size_t narrowPhaseTests = 0;
    size_t islandCount = 0;
    double broadPhaseMs = 0.0;
    double narrowPhaseMs = 0.0;
    double contactSolverMs = 0.0;
    double jointSolverMs = 0.0;
    double ccdRecoveryMs = 0.0;
};

class World;

// Copyable rollback point owned by the World that created it. Its contents are
// intentionally opaque so internal dense indices and cache layouts can evolve.
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
    WorldCcdDefaults ccdDefaults_;
    WorldMultiToiSettings multiToiSettings_;
    SolverOptions solverOptions_;
};

struct BroadPhaseData;

// CPU contact solving strategy. Parallel solves independent contact islands
// concurrently on the worker pool; results are bitwise identical to
// Sequential because islands share no dynamic bodies. The GPU backend uses
// graph coloring and ignores this setting.
enum class IslandSolvingMode : uint8_t { Sequential = 0, Parallel = 1 };

// Relaxed mode favors throughput and may use platform-specific floating-point
// contraction or the CUDA graph-colored solver. Strict mode is available only
// in a VELOX_STRICT_FLOATING_POINT build and selects the ordered CPU reference
// backend, making replay comparison meaningful across supported CPU platforms.
enum class DeterminismMode : uint8_t { Relaxed = 0, Strict = 1 };

// Controls the response to a recoverable CUDA allocation or device failure.
// CPU fallback reruns the failed frame from its pre-step snapshot; because the
// CPU and CUDA solvers use different execution orders, the switch is not a
// lockstep-deterministic transition.
enum class DeviceLossPolicy : uint8_t { FallbackToCPU = 0, ThrowException = 1 };

// Controls whether the CUDA backend keeps body/contact/joint data resident on
// the GPU across step() calls, eliminating per-substep PCIe transfers.
// Disabled is the default and matches the original per-substep upload path.
// Resident activates the fast path when the backend supports it; the world
// silently falls back to the non-resident path when constraints prevent it
// (unsupported joint types, adaptive iteration, shape mutations mid-step).
enum class GPUResidentMode : uint8_t { Disabled = 0, Resident = 1 };

// Controls which threads may enter World methods. Strict is the default and
// confines all access to the creating thread. Relaxed permits cross-thread
// query calls, but mutations and step() remain creator-thread only. Concurrent
// serializes the supported method API with an internal lock; it is intended
// for tools and gameplay code that cannot maintain an external world lock.
enum class ThreadSafetyPolicy : uint8_t {
    Strict = 0,
    Relaxed = 1,
    Concurrent = 2,
};

struct ThreadSafetyReport {
    uint64_t queryCallsFromNonMainThread = 0;
    uint64_t mutationCallsDuringStep = 0;
    uint64_t stepInvocationsOnNonMainThread = 0;
};

// Runtime replacement for convex primitive collider geometry. Hull and
// compound payloads are value-owned so callers can safely retain their input
// after mutation returns.
struct ShapeMutation {
    enum class Type : uint8_t { Sphere, Box, Capsule, Cylinder, Cone, Hull, Compound, RoundedBox, Ellipsoid };
    Type type = Type::Sphere;
    float radius = 0.0f;
    Vec3 halfExtents{0.5f, 0.5f, 0.5f};
    float capsuleHalfHeight = 0.0f;
    std::vector<Vec3> hullPoints;
    std::vector<CompoundShape> compoundShapes;
    bool preserveMassProperties = false;
};

struct ShapeScale {
    Vec3 factor{1.0f, 1.0f, 1.0f};
    bool updateMassProperties = true;
};

class World {
public:
    // Auto picks the NVIDIA CUDA backend when built with VELOX_ENABLE_CUDA and
    // a device is present, otherwise the portable CPU backend. Cuda throws if
    // unavailable.
    explicit World(BackendType type = BackendType::Auto);
    ~World();

    DeterminismMode determinismMode() const;
    // Strict mode requires a build configured with VELOX_STRICT_FLOATING_POINT.
    // It uses the CPU reference backend; CUDA strict parity is not yet supported.
    void setDeterminismMode(DeterminismMode mode);

    DeviceLossPolicy deviceLossPolicy() const;
    void setDeviceLossPolicy(DeviceLossPolicy policy);
    GPUResidentMode gpuResidentMode() const;
    void setGPUResidentMode(GPUResidentMode mode);
    // True whenever the active backend is the portable CPU implementation,
    // including an Auto/CUDA world that fell back after a recoverable failure.
    bool isOnCPUBackend() const;
    // Recreate the CUDA backend after CPU fallback. Returns false when no
    // usable CUDA device is available; call during a non-critical frame.
    bool resetCUDABackend();

    WorldCcdDefaults ccdDefaults() const;
    void setCcdDefaults(WorldCcdDefaults defaults);
    SolverOptions solverOptions() const;
    void setSolverOptions(SolverOptions options);
    WorldMultiToiSettings multiToiSettings() const;
    void setMultiToiSettings(WorldMultiToiSettings settings);
    void setCcdTuning(BodyId id, BodyCcdTuning tuning);
    BodyCcdTuning ccdTuning(BodyId id) const;
    std::vector<MultiToiHit> queryMultiToi(BodyId id, float dt) const;

    IslandSolvingMode islandSolvingMode() const;
    void setIslandSolvingMode(IslandSolvingMode mode);

    ThreadSafetyPolicy threadSafetyPolicy() const;
    void setThreadSafetyPolicy(ThreadSafetyPolicy policy);
    ThreadSafetyReport threadSafetyReport() const;

    const char* backendName() const;
    void setWorkerCount(uint32_t count);
    uint32_t workerCount() const;
    // Inject an external task system for parallel work distribution.
    // Pass nullptr to revert to the internal worker pool. The TaskSystem
    // must remain alive for the lifetime of the World (or until replaced).
    void setTaskSystem(TaskSystem* system);
    TaskSystem* taskSystem() const;

    Vec3 gravity{0, -9.81f, 0};

    // Solver substeps per step() call. More substeps = stiffer stacks and
    // less friction drift for the same iteration budget (Box2D v3 approach).
    int substeps = 4;

    // Safe configuration alternatives for use after publishing a World to
    // another thread. Direct writes to gravity/substeps must be externally
    // synchronized for compatibility with the pre-1.0 public fields.
    void setGravity(Vec3 value);
    Vec3 gravityValue() const;
    void setSubsteps(int value);
    int substepCount() const;

    BodyId addSphere(Vec3 position, float radius, float mass);
    BodyId addBox(Vec3 position, Vec3 halfExtents, float mass);
    BodyId addCapsule(Vec3 position, float radius, float halfHeight, float mass);
    BodyId addCylinder(Vec3 position, float radius, float halfHeight, float mass);
    BodyId addCone(Vec3 position, float radius, float height, float mass);
    BodyId addRoundedBox(Vec3 position, Vec3 halfExtents, float radius, float mass);
    BodyId addEllipsoid(Vec3 position, Vec3 radii, float mass);
    // Convex hull from a local-space point cloud. Interior points are excluded
    // from mass integration but still cost support-function time.
    BodyId addConvexHull(Vec3 position, const std::vector<Vec3>& points, float mass);
    // Dynamic compounds use uniform density across child volumes. Geometry is
    // recentered without moving it in world space, so Body::position is the
    // computed center of mass rather than the authored compound origin.
    BodyId addCompound(Vec3 position, const std::vector<CompoundShape>& shapes,
                       float mass);
    BodyId addStaticPlane(Vec3 normal, float offset);
    // Static triangle mesh (level geometry). vertices: xyz triples,
    // indices: 3 per triangle.
    BodyId addStaticMesh(const std::vector<Vec3>& vertices,
                         const std::vector<uint32_t>& indices);
    // Row-major heights over width x depth samples. Grid lies in local X/Z;
    // origin is the world position of sample (0, 0).
    BodyId addStaticHeightfield(uint32_t width, uint32_t depth, float cellSize,
                                const std::vector<float>& heights, Vec3 origin = {});

    Body& body(BodyId id);
    const Body& body(BodyId id) const;
    // Thread-safe copies. body()/joint() return borrowed references and must
    // not outlive external synchronization with step() or mutation.
    Body bodyState(BodyId id) const;
    GeometryDiagnostics queryGeometryDiagnostics(BodyId id) const;
    size_t bodyCount() const;
    bool isValid(BodyId id) const;
    void removeBody(BodyId id); // also removes joints attached to the body
    MotionType motionType(BodyId id) const;
    void setMotionType(BodyId id, MotionType type);
    // Replace collider geometry without changing the body's handle or joints.
    // Hull and compound payloads are built and appended transactionally.
    void mutateShape(BodyId id, const ShapeMutation& mutation);
    void scaleShape(BodyId id, const ShapeScale& scale);
    void setCollisionMargin(BodyId id, float margin);
    // Overrides mass and principal moments/axes without changing motion type.
    // principalOrientation rotates the principal frame into body-local space.
    void setMassProperties(BodyId id, float mass, Vec3 principalInertia,
                           Quat principalOrientation = {});
    void setTransform(BodyId id, Vec3 position, Quat orientation);
    void setLinearVelocity(BodyId id, Vec3 velocity);
    void setAngularVelocity(BodyId id, Vec3 velocity);
    void addForce(BodyId id, Vec3 force);
    void addForceAtPoint(BodyId id, Vec3 force, Vec3 worldPoint);
    void addTorque(BodyId id, Vec3 torque);
    void addLinearImpulse(BodyId id, Vec3 impulse);
    void addImpulseAtPoint(BodyId id, Vec3 impulse, Vec3 worldPoint);
    void clearForces(BodyId id);
    void setSensor(BodyId id, bool enabled);
    bool isSensor(BodyId id) const;
    void setGravityScale(BodyId id, float scale);
    float gravityScale(BodyId id) const;
    void setLinearDamping(BodyId id, float damping);
    void setAngularDamping(BodyId id, float damping);
    void setCollisionFilter(BodyId id, uint32_t categoryBits, uint32_t maskBits);
    void setEnableSleep(BodyId id, bool enabled);
    bool isSleepEnabled(BodyId id) const;
    void setFixedRotation(BodyId id, bool enabled);
    bool isFixedRotation(BodyId id) const;
    void wakeBody(BodyId id);
    void sleepBody(BodyId id);
    // Apply a radial impulse from origin to all dynamic bodies within radius.
    // Falloff is linear: full impulse at center, zero at radius boundary.
    void explode(Vec3 origin, float radius, float impulse);
    // Subtracts offset from every world-space coordinate while preserving
    // relative geometry and dynamics. Use this to keep large worlds near the
    // floating-point origin without rebuilding bodies, joints, or meshes.
    void shiftOrigin(Vec3 offset);
    WorldSnapshot saveSnapshot() const;
    void restoreSnapshot(const WorldSnapshot& snapshot);
    const StepStats& lastStepStats() const { return lastStepStats_; }
    StepStats lastStepStatsCopy() const;

    // --- joints -------------------------------------------------------------
    JointId addBallJoint(BodyId a, BodyId b, Vec3 worldAnchor);
    JointId addDistanceJoint(BodyId a, BodyId b, Vec3 worldAnchorA, Vec3 worldAnchorB);
    JointId addSpringJoint(BodyId a, BodyId b, Vec3 worldAnchorA, Vec3 worldAnchorB,
                           float frequencyHz, float dampingRatio);
    JointId addHingeJoint(BodyId a, BodyId b, Vec3 worldAnchor, Vec3 worldAxis);
    JointId addConeTwistJoint(BodyId a, BodyId b, Vec3 worldAnchor, Vec3 worldAxis);
    JointId addFixedJoint(BodyId a, BodyId b, Vec3 worldAnchor);
    JointId addPrismaticJoint(BodyId a, BodyId b, Vec3 worldAnchor, Vec3 worldAxis);
    JointId addSixDofJoint(BodyId a, BodyId b, Vec3 worldAnchor);
    Joint& joint(JointId id);                              // configure motors/limits
    const Joint& joint(JointId id) const;
    Joint jointState(JointId id) const;
    bool isValid(JointId id) const;
    void removeJoint(JointId id);
    float hingeAngle(JointId id) const;                     // radians, 0 at creation
    float coneSwingAngle(JointId id) const;
    float coneTwistAngle(JointId id) const;
    float prismaticTranslation(JointId id) const;
    Vec3 sixDofLinearTranslation(JointId id) const;
    Vec3 sixDofAngularRotation(JointId id) const;
    const std::vector<JointBreakEvent>& jointBreakEvents() const {
        return jointBreakEvents_;
    }

    // Body lifecycle events from the most recent step().
    const std::vector<BodyEvent>& bodyEvents() const { return bodyEvents_; }

    // Contact and sensor Begin/Persist/End events from the most recent step().
    const std::vector<ContactEvent>& contactEvents() const { return events_; }
    // Runs once per generated contact before waking, warm starting, and solving.
    // The callback may change the point, normal, resolved material values, or
    // disable the contact. Throwing aborts step() before body transforms advance.
    void setContactModifier(ContactModifier modifier);

    // --- sleeping -----------------------------------------------------------
    // Bodies whose island stays below the motion threshold for a while are
    // put to sleep (zero cost until something touches them). Call wake() after
    // manually changing a sleeping body's velocity or position.
    void wake(BodyId id);
    bool isAwake(BodyId id) const;

    // --- queries ------------------------------------------------------------
    RayHit rayCast(Vec3 origin, Vec3 dir, float maxDist,
                   const QueryFilter& filter = {}) const;
    // All bodies hit within maxDist, sorted nearest-first. Each body appears
    // at most once (its nearest intersection).
    void rayCastAll(Vec3 origin, Vec3 dir, float maxDist,
                    std::vector<RayHit>& out,
                    const QueryFilter& filter = {}) const;
    ClosestPointResult closestPoints(BodyId a, BodyId b) const;
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
    // Synchronously resolve value-owned requests against one consistent World
    // state. Invalid individual requests produce success=false and an error;
    // they do not abort the rest of the batch.
    void batchQueries(const std::vector<QueryDesc>& queries,
                      std::vector<QueryResult>& outResults) const;
    // Thread-safe, non-blocking submission. Requests are resolved at the
    // deterministic beginning of the next owner-thread step(), before any
    // simulation state changes. This is intentionally allowed even in Strict
    // thread-safety mode because it copies no World state at submission time.
    AsyncQueryHandle submitAsyncQuery(const QueryDesc& query);
    // Blocks until the request has been resolved. A handle may be consumed
    // once; callers must externally synchronize World destruction.
    QueryResult getAsyncResult(AsyncQueryHandle handle);
    void debugLines(std::vector<DebugLine>& out,
                    uint32_t flags = DebugDrawAll) const;

    // Advances the simulation using Predictive Contact Sweeping: speculative
    // contacts solved iteratively, backed by a conservative-advancement sweep
    // safety net, so no velocity can tunnel through geometry and grazing
    // contact stays smooth.
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
        std::unique_lock<std::recursive_mutex> lock_;
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
    std::vector<uint64_t> prevPairKeys_;
    ContactModifier contactModifier_;
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
    StepStats lastStepStats_;
    MeshSoup meshes_;
    std::vector<uint64_t> candidatePairs_;
    mutable std::unique_ptr<BroadPhaseData> broadPhase_;
    std::unique_ptr<Backend> backend_;
    mutable std::recursive_mutex accessMutex_;
    struct PendingAsyncQuery { uint64_t id; QueryDesc query; };
    struct AsyncQueryResult { bool ready = false; QueryResult result; };
    mutable std::mutex asyncQueryMutex_;
    std::condition_variable asyncQueryReady_;
    std::atomic<bool> hasPendingAsyncQueries_{false};
    uint64_t nextAsyncQueryId_ = 1;
    std::deque<PendingAsyncQuery> pendingAsyncQueries_;
    std::unordered_map<uint64_t, AsyncQueryResult> asyncQueryResults_;
    std::thread::id ownerThread_;
    ThreadSafetyPolicy threadSafetyPolicy_ = ThreadSafetyPolicy::Strict;
    mutable ThreadSafetyReport threadSafetyReport_;
};

} // namespace velox
