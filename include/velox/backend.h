#pragma once
#include "body.h"
#include <vector>

namespace velox {

// A speculative contact: generated while the pair is still separated, whenever
// the relative motion could close the gap within the step. gap > 0 means
// "not yet touching"; the solver only removes approach velocity in excess of
// gap/dt, so grazing bodies are never artificially stopped.
struct Contact {
    BodyId a, b;
    Vec3 normal;          // from b towards a
    float gap;            // signed distance between surfaces (negative = penetrating)
    float vn0;            // normal approach velocity at detection (for restitution)
    float normalImpulse;  // accumulated by the solver
    float tangentImpulse;
};

// Compute backend interface. The CPU reference backend lives in solver.cpp;
// a CUDA backend implements the same interface over device buffers, which is
// possible because Body is a POD laid out for bulk upload.
class Backend {
public:
    virtual ~Backend() = default;
    virtual void integrate(std::vector<Body>& bodies, const Vec3& gravity, float dt) = 0;
    // Finds all pairs whose gap could close within dt (speculative detection).
    virtual void findContacts(const std::vector<Body>& bodies, float dt,
                              std::vector<Contact>& out) = 0;
};

Backend* createCpuBackend();
#if VELOX_HAS_CUDA
Backend* createCudaBackend();
#endif

} // namespace velox
