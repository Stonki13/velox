#pragma once
#include "body.h"
#include <vector>

namespace velox {

// All static mesh geometry, flattened into contiguous arrays (GPU-uploadable).
struct MeshSoup {
    std::vector<Vec3> vertices;
    std::vector<uint32_t> indices;
    std::vector<Mesh> meshes;
    std::vector<BvhNode> bvhNodes;   // all meshes' BVHs, concatenated
    std::vector<uint32_t> bvhTriRefs; // leaf -> triangle number within its mesh
};

// Raw-pointer view of MeshSoup, usable identically from host and device code.
struct MeshSoupView {
    const Vec3* vertices;
    const uint32_t* indices;
    const Mesh* meshes;
    const BvhNode* bvhNodes;
    const uint32_t* bvhTriRefs;
};

inline MeshSoupView view(const MeshSoup& s) {
    return {s.vertices.data(), s.indices.data(), s.meshes.data(),
            s.bvhNodes.data(), s.bvhTriRefs.data()};
}

// A speculative contact: generated while the pair is still separated, whenever
// the relative motion could close the gap within the step. gap > 0 means
// "not yet touching"; the solver only removes approach velocity in excess of
// gap/dt, so grazing bodies are never artificially stopped.
struct Contact {
    BodyId a, b;
    Vec3 normal;          // from b towards a
    Vec3 point;           // world-space contact point (torque arm for rotation)
    float gap;            // signed distance between surfaces (negative = penetrating)
    float vn0;            // normal approach velocity at detection (for restitution)
    float normalImpulse;  // accumulated by the solver
    float tangentImpulse;
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
    // Finds all pairs whose gap could close within dt (speculative detection).
    virtual void findContacts(const std::vector<Body>& bodies, const MeshSoup& meshes,
                              float dt, std::vector<Contact>& out) = 0;
};

Backend* createCpuBackend();
Backend* createCudaBackend(); // returns nullptr if unavailable (no build/device)

} // namespace velox
