#pragma once
#include "body.h"
#include <vector>

namespace velox {

struct Contact {
    BodyId a, b;
    Vec3 normal;      // from b towards a
    float depth;
    float toi;        // time of impact in [0, 1] of the step; 1 = discrete contact
};

// Compute backend interface. The CPU reference backend lives in solver.cpp;
// a CUDA backend implements the same interface over device buffers, which is
// possible because Body is a POD laid out for bulk upload.
class Backend {
public:
    virtual ~Backend() = default;
    virtual void integrate(std::vector<Body>& bodies, const Vec3& gravity, float dt) = 0;
    virtual void findContacts(const std::vector<Body>& bodies, float dt,
                              std::vector<Contact>& out) = 0;
};

Backend* createCpuBackend();
#if VELOX_HAS_CUDA
Backend* createCudaBackend();
#endif

} // namespace velox
