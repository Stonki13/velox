#include "narrowphase.h"
#include <algorithm>

namespace velox {

// CPU reference backend. The narrow phase is the shared VELOX_HD code in
// narrowphase.h — identical to what the CUDA kernels run.
class CpuBackend final : public Backend {
public:
    const char* name() const override { return "cpu"; }

    void integrate(std::vector<Body>& bodies, const Vec3& gravity, float dt) override {
        for (Body& b : bodies) {
            if (b.isStatic()) continue;
            b.velocity += gravity * dt; // semi-implicit Euler
        }
    }

    void findContacts(const std::vector<Body>& bodies, const MeshSoup& meshes,
                      float dt, std::vector<Contact>& out) override {
        out.clear();
        const MeshSoupView soup = view(meshes);
        const size_t n = bodies.size();

        // Planes and meshes are unbounded/large: every dynamic body is tested
        // against them directly. Bounded bodies go through sweep-and-prune.
        boundless_.clear();
        sorted_.clear();
        aabbs_.resize(n);
        for (BodyId i = 0; i < n; ++i) {
            const Body& b = bodies[i];
            if (b.shape == ShapeType::Plane || b.shape == ShapeType::Mesh)
                boundless_.push_back(i);
            else {
                bodyAabb(b, dt, aabbs_[i].lo, aabbs_[i].hi);
                sorted_.push_back(i);
            }
        }

        Contact buf[kMaxContactsPerPair];
        auto flush = [&](int count) {
            for (int c = 0; c < count; ++c) out.push_back(buf[c]);
        };

        for (BodyId i : sorted_)
            if (!bodies[i].isStatic())
                for (BodyId j : boundless_)
                    flush(collidePair(bodies[i], bodies[j], i, j, soup, dt,
                                      buf, kMaxContactsPerPair));

        // Sweep-and-prune along X: sort by AABB min, only test while overlapping.
        std::sort(sorted_.begin(), sorted_.end(), [&](BodyId a, BodyId b) {
            return aabbs_[a].lo.x < aabbs_[b].lo.x;
        });
        for (size_t si = 0; si < sorted_.size(); ++si) {
            BodyId i = sorted_[si];
            for (size_t sj = si + 1; sj < sorted_.size(); ++sj) {
                BodyId j = sorted_[sj];
                if (aabbs_[j].lo.x > aabbs_[i].hi.x) break; // pruned: sorted axis
                if (bodies[i].isStatic() && bodies[j].isStatic()) continue;
                if (!aabbOverlap(aabbs_[i].lo, aabbs_[i].hi, aabbs_[j].lo, aabbs_[j].hi))
                    continue;
                flush(collidePair(bodies[i], bodies[j], i, j, soup, dt,
                                  buf, kMaxContactsPerPair));
            }
        }
    }

private:
    struct Aabb { Vec3 lo, hi; };
    std::vector<Aabb> aabbs_;
    std::vector<BodyId> sorted_;
    std::vector<BodyId> boundless_;
};

Backend* createCpuBackend() { return new CpuBackend(); }

#if !VELOX_HAS_CUDA
Backend* createCudaBackend() { return nullptr; }
#endif

} // namespace velox
