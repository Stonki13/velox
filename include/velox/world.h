#pragma once
#include "backend.h"
#include "joint.h"
#include <functional>
#include <memory>
#include <utility>
#include <vector>

namespace velox {

enum class ContactEventType : uint8_t { Begin, Persist, End };

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

struct RayHit {
    bool hit = false;
    BodyId body;
    float t = 0.0f;   // distance along the ray direction
    Vec3 point, normal;
};

struct QueryFilter {
    uint32_t categoryBits = UINT32_MAX;
    uint32_t maskBits = UINT32_MAX;
    bool includeSensors = true;
    BodyId ignoredBody;
};

struct ShapeCastHit {
    bool hit = false;
    BodyId body;
    float distance = 0.0f;
    float fraction = 0.0f;
    Vec3 point, normal; // normal points from the hit body toward the cast shape
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
    size_t jointCount = 0;
    double setupMs = 0.0;
    double collisionDetectionMs = 0.0;
    double solverMs = 0.0;
    double ccdMs = 0.0;
    double finalizeMs = 0.0;
    double totalMs = 0.0;
};

class World;

// Copyable rollback point owned by the World that created it. Its contents are
// intentionally opaque so internal dense indices and cache layouts can evolve.
class WorldSnapshot {
public:
    WorldSnapshot() = default;

private:
    friend class World;
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
};

class World {
public:
    // Auto picks the CUDA backend when built with VELOX_ENABLE_CUDA and a
    // device is present, otherwise the CPU backend. Cuda throws if unavailable.
    explicit World(BackendType type = BackendType::Auto);

    const char* backendName() const;
    void setWorkerCount(uint32_t count) { backend_->setWorkerCount(count); }
    uint32_t workerCount() const { return backend_->workerCount(); }

    Vec3 gravity{0, -9.81f, 0};

    // Solver substeps per step() call. More substeps = stiffer stacks and
    // less friction drift for the same iteration budget (Box2D v3 approach).
    int substeps = 4;

    BodyId addSphere(Vec3 position, float radius, float mass);
    BodyId addBox(Vec3 position, Vec3 halfExtents, float mass);
    BodyId addCapsule(Vec3 position, float radius, float halfHeight, float mass);
    BodyId addCylinder(Vec3 position, float radius, float halfHeight, float mass);
    BodyId addCone(Vec3 position, float radius, float height, float mass);
    // Convex hull from a local-space point cloud (points should already be on
    // the hull; interior points only cost support-function time).
    BodyId addConvexHull(Vec3 position, const std::vector<Vec3>& points, float mass);
    BodyId addCompound(Vec3 position, const std::vector<CompoundShape>& shapes, float mass);
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
    size_t bodyCount() const { return bodies_.size(); }
    bool isValid(BodyId id) const noexcept;
    void removeBody(BodyId id); // also removes joints attached to the body
    MotionType motionType(BodyId id) const;
    void setMotionType(BodyId id, MotionType type);
    void setTransform(BodyId id, Vec3 position, Quat orientation);
    void setLinearVelocity(BodyId id, Vec3 velocity);
    void setAngularVelocity(BodyId id, Vec3 velocity);
    void addForce(BodyId id, Vec3 force);
    void addForceAtPoint(BodyId id, Vec3 force, Vec3 worldPoint);
    void addTorque(BodyId id, Vec3 torque);
    void addLinearImpulse(BodyId id, Vec3 impulse);
    void addImpulseAtPoint(BodyId id, Vec3 impulse, Vec3 worldPoint);
    void clearForces(BodyId id);
    WorldSnapshot saveSnapshot() const;
    void restoreSnapshot(const WorldSnapshot& snapshot);
    const StepStats& lastStepStats() const { return lastStepStats_; }

    // --- joints -------------------------------------------------------------
    JointId addBallJoint(BodyId a, BodyId b, Vec3 worldAnchor);
    JointId addDistanceJoint(BodyId a, BodyId b, Vec3 worldAnchorA, Vec3 worldAnchorB);
    JointId addSpringJoint(BodyId a, BodyId b, Vec3 worldAnchorA, Vec3 worldAnchorB,
                           float frequencyHz, float dampingRatio);
    JointId addHingeJoint(BodyId a, BodyId b, Vec3 worldAnchor, Vec3 worldAxis);
    JointId addConeTwistJoint(BodyId a, BodyId b, Vec3 worldAnchor, Vec3 worldAxis);
    JointId addFixedJoint(BodyId a, BodyId b, Vec3 worldAnchor);
    JointId addPrismaticJoint(BodyId a, BodyId b, Vec3 worldAnchor, Vec3 worldAxis);
    Joint& joint(JointId id);                              // configure motors/limits
    const Joint& joint(JointId id) const;
    bool isValid(JointId id) const noexcept;
    void removeJoint(JointId id);
    float hingeAngle(JointId id) const;                     // radians, 0 at creation
    float coneSwingAngle(JointId id) const;
    float coneTwistAngle(JointId id) const;
    float prismaticTranslation(JointId id) const;
    const std::vector<JointBreakEvent>& jointBreakEvents() const {
        return jointBreakEvents_;
    }

    // Contact and sensor Begin/Persist/End events from the most recent step().
    const std::vector<ContactEvent>& contactEvents() const { return events_; }
    // Runs once per generated contact before waking, warm starting, and solving.
    // The callback may change the point, normal, resolved material values, or
    // disable the contact. Throwing aborts step() before body transforms advance.
    void setContactModifier(ContactModifier modifier) {
        contactModifier_ = std::move(modifier);
    }

    // --- sleeping -----------------------------------------------------------
    // Bodies whose island stays below the motion threshold for a while are
    // put to sleep (zero cost until something touches them). Call wake() after
    // manually changing a sleeping body's velocity or position.
    void wake(BodyId id);
    bool isAwake(BodyId id) const;

    // --- queries ------------------------------------------------------------
    RayHit rayCast(Vec3 origin, Vec3 dir, float maxDist,
                   const QueryFilter& filter = {}) const;
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

    // Advances the simulation using Predictive Contact Sweeping: speculative
    // contacts solved iteratively, backed by a conservative-advancement sweep
    // safety net, so no velocity can tunnel through geometry and grazing
    // contact stays smooth.
    void step(float dt);

private:
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
    void removeJointDense(uint32_t dense);
    void solveJoints(float dt);
    void updateSleeping(float dt);

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
    std::vector<uint64_t> prevPairKeys_;
    ContactModifier contactModifier_;
    StepStats lastStepStats_;
    MeshSoup meshes_;
    std::unique_ptr<Backend> backend_;
};

} // namespace velox
