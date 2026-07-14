#pragma once
#include "joint.h"
#include <vector>

namespace velox {

// All static mesh geometry, flattened into contiguous arrays (GPU-uploadable).
struct MeshSoup {
    std::vector<Vec3> vertices;
    std::vector<uint32_t> indices;
    std::vector<Mesh> meshes;
    std::vector<BvhNode> bvhNodes;   // all meshes' BVHs, concatenated
    std::vector<uint32_t> bvhTriRefs; // leaf -> triangle number within its mesh
    std::vector<Vec3> hullPoints;    // convex hull local-space points
    std::vector<CompoundChild> compoundChildren;
};

// Raw-pointer view of MeshSoup, usable identically from host and device code.
struct MeshSoupView {
    const Vec3* vertices;
    const uint32_t* indices;
    const Mesh* meshes;
    const BvhNode* bvhNodes;
    const uint32_t* bvhTriRefs;
    const Vec3* hullPoints;
    const CompoundChild* compoundChildren;
};

inline MeshSoupView view(const MeshSoup& s) {
    return {s.vertices.data(), s.indices.data(), s.meshes.data(),
            s.bvhNodes.data(), s.bvhTriRefs.data(), s.hullPoints.data(),
            s.compoundChildren.data()};
}

VELOX_HD inline Body compoundChildBody(const Body& parent,
                                       const CompoundChild& child) {
    Body result = parent;
    result.shape = child.shape;
    result.position = parent.position + rotate(parent.orientation, child.localPosition);
    result.orientation = mul(parent.orientation, child.localOrientation);
    result.radius = child.radius;
    result.halfExtents = child.halfExtents;
    result.capsuleHalfHeight = child.capsuleHalfHeight;
    result.hullFirst = child.hullFirst;
    result.hullCount = child.hullCount;
    result.compoundFirst = result.compoundCount = 0;
    return result;
}

// A speculative contact: generated while the pair is still separated, whenever
// the relative motion could close the gap within the step. gap > 0 means
// "not yet touching"; the solver only removes approach velocity in excess of
// gap/dt, so grazing bodies are never artificially stopped.
struct Contact {
    BodyIndex a, b;
    uint64_t featureKey;  // stable shape-feature pair; 0 when unavailable
    Vec3 normal;          // from b towards a
    Vec3 point;           // world-space contact point (torque arm for rotation)
    Vec3 localAnchorA;    // contact anchor in A's local frame
    Vec3 localAnchorB;    // contact anchor in B's local frame
    float gap;            // signed distance between surfaces at detection
    float bias0;          // gap - dot(normal, anchorA - anchorB) at detection;
                          // lets the solver evaluate the live translational and
                          // rotational gap over several solver substeps
    float vn0;            // normal approach velocity at detection (for restitution)
    float restitution;    // resolved material values (modifiable before solve)
    float friction1, friction2;
    float rollingFriction, spinningFriction;
    float normalImpulse;  // accumulated by the solver
    float tangentImpulse1;
    float tangentImpulse2;
    float rollingImpulse1, rollingImpulse2;
    float spinningImpulse;
};

enum class BackendType { Auto, Cpu, Cuda };

// Compute backend interface. The CPU reference backend lives in solver.cpp;
// the CUDA backend (backend_cuda.cu) runs the same integration and narrow
// phase — shared VELOX_HD code — over device buffers.
class Backend {
public:
    virtual ~Backend() = default;
    virtual const char* name() const = 0;
    virtual void integrate(std::vector<Body>& bodies, const Vec3& gravity, float dt) = 0;
    // True when the backend consumes host-built broad-phase candidate pairs
    // (from the World's incremental AABB tree). GPU backends run their own
    // broad phase on-device and return false.
    virtual bool wantsHostPairs() const { return false; }
    // Finds all pairs whose gap could close within dt (speculative detection).
    // hostPairs, when non-null, holds candidate pairs encoded (min<<32)|max
    // over dense indices; backends that report wantsHostPairs() use it instead
    // of running a broad phase.
    virtual void findContacts(const std::vector<Body>& bodies, const MeshSoup& meshes,
                              float dt, const std::vector<uint64_t>* hostPairs,
                              std::vector<Contact>& out) = 0;
    // Iterative velocity solve over the contacts. CPU: sequential impulses.
    // CUDA: graph-colored parallel impulses (same math, conflict-free order).
    // warmStart is true only for the first substep of a step: accumulated
    // impulses already live in the velocities on later substeps.
    virtual void solveVelocities(std::vector<Body>& bodies,
                                 std::vector<Contact>& contacts, float dt,
                                 bool warmStart) = 0;
    // Optional fast path after the first velocity integration/contact find.
    // Returns true when all solver substeps and transform integration were
    // completed. CPU and backends requiring host-side constraints return false.
    virtual bool advanceSubsteps(std::vector<Body>&, std::vector<Contact>&,
                                 std::vector<Joint>&, const Vec3&, float, int) {
        return false;
    }
    // Called once after the last substep: backends holding impulses in
    // device memory write the accumulated values back into `contacts` (used
    // for next frame's warm starting). CPU backend solves in place: no-op.
    virtual void fetchImpulses(std::vector<Contact>&) {}
    // Host state was restored or replaced; discard any backend-side mirrors.
    virtual void invalidateCaches() {}
    // CPU backend: 0 selects hardware concurrency. GPU backends report 0.
    virtual void setWorkerCount(uint32_t) {}
    virtual uint32_t workerCount() const { return 0; }
};

Backend* createCpuBackend();
Backend* createCudaBackend(); // returns nullptr if unavailable (no build/device)

} // namespace velox
