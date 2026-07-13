#pragma once
#include "backend.h"
#include "joint.h"
#include <memory>
#include <vector>

namespace velox {

// A body pair that began touching this step (fires once per pair; fires again
// only after the pair separates and re-touches).
struct ContactEvent {
    BodyId a, b;
    Vec3 point, normal;   // representative contact; normal points from b to a
    float impulse;        // largest accumulated normal impulse in the manifold
};

struct RayHit {
    bool hit = false;
    BodyId body = 0;
    float t = 0.0f;   // distance along the ray direction
    Vec3 point, normal;
};

class World {
public:
    // Auto picks the CUDA backend when built with VELOX_ENABLE_CUDA and a
    // device is present, otherwise the CPU backend. Cuda throws if unavailable.
    explicit World(BackendType type = BackendType::Auto);

    const char* backendName() const;

    Vec3 gravity{0, -9.81f, 0};

    // Solver substeps per step() call. More substeps = stiffer stacks and
    // less friction drift for the same iteration budget (Box2D v3 approach).
    int substeps = 4;

    BodyId addSphere(Vec3 position, float radius, float mass);
    BodyId addBox(Vec3 position, Vec3 halfExtents, float mass);
    BodyId addCapsule(Vec3 position, float radius, float halfHeight, float mass);
    // Convex hull from a local-space point cloud (points should already be on
    // the hull; interior points only cost support-function time).
    BodyId addConvexHull(Vec3 position, const std::vector<Vec3>& points, float mass);
    BodyId addStaticPlane(Vec3 normal, float offset);
    // Static triangle mesh (level geometry). vertices: xyz triples,
    // indices: 3 per triangle.
    BodyId addStaticMesh(const std::vector<Vec3>& vertices,
                         const std::vector<uint32_t>& indices);

    Body& body(BodyId id) { return bodies_[id]; }
    const Body& body(BodyId id) const { return bodies_[id]; }
    size_t bodyCount() const { return bodies_.size(); }

    // --- joints -------------------------------------------------------------
    JointId addBallJoint(BodyId a, BodyId b, Vec3 worldAnchor);
    JointId addDistanceJoint(BodyId a, BodyId b, Vec3 worldAnchorA, Vec3 worldAnchorB);
    JointId addHingeJoint(BodyId a, BodyId b, Vec3 worldAnchor, Vec3 worldAxis);
    Joint& joint(JointId id) { return joints_[id]; }        // configure motors/limits
    float hingeAngle(JointId id) const;                     // radians, 0 at creation

    // Contact begin events from the most recent step().
    const std::vector<ContactEvent>& contactEvents() const { return events_; }

    // --- sleeping -----------------------------------------------------------
    // Bodies whose island stays below the motion threshold for a while are
    // put to sleep (zero cost until something touches them). Call wake() after
    // manually changing a sleeping body's velocity or position.
    void wake(BodyId id);
    bool isAwake(BodyId id) const { return !bodies_[id].asleep; }

    // --- queries ------------------------------------------------------------
    RayHit rayCast(Vec3 origin, Vec3 dir, float maxDist) const;
    void overlapSphere(Vec3 center, float radius, std::vector<BodyId>& out) const;

    // Advances the simulation using Predictive Contact Sweeping: speculative
    // contacts solved iteratively, backed by a conservative-advancement sweep
    // safety net, so no velocity can tunnel through geometry and grazing
    // contact stays smooth.
    void step(float dt);

private:
    void solveJoints(float dt);
    void updateSleeping(float dt);

    std::vector<Body> bodies_;
    std::vector<Contact> contacts_;
    std::vector<Joint> joints_;
    struct PrevState { Vec3 position; Quat orientation; };
    std::vector<PrevState> prev_;
    std::vector<uint64_t> pairKeys_;
    std::vector<Contact> prevContacts_;  // sorted by pair key (warm starting)
    std::vector<uint32_t> unionParent_;
    std::vector<float> islandTimer_;
    std::vector<ContactEvent> events_;
    std::vector<uint64_t> prevPairKeys_;
    MeshSoup meshes_;
    std::unique_ptr<Backend> backend_;
};

} // namespace velox
