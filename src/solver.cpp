#include "velox/backend.h"

namespace velox {

// CPU reference backend. Loops here are written branch-light over flat arrays
// so the CUDA backend can mirror them kernel-for-kernel.
class CpuBackend final : public Backend {
public:
    void integrate(std::vector<Body>& bodies, const Vec3& gravity, float dt) override {
        for (Body& b : bodies) {
            if (b.isStatic()) continue;
            b.velocity += gravity * dt; // semi-implicit Euler
        }
    }

    void findContacts(const std::vector<Body>& bodies, float dt,
                      std::vector<Contact>& out) override {
        out.clear();
        // O(n^2) all-pairs for now; sweep-and-prune broad phase is next.
        for (BodyId i = 0; i < bodies.size(); ++i) {
            for (BodyId j = i + 1; j < bodies.size(); ++j) {
                const Body& a = bodies[i];
                const Body& b = bodies[j];
                if (a.isStatic() && b.isStatic()) continue;
                collide(a, b, i, j, dt, out);
            }
        }
    }

private:
    // A pair becomes a speculative contact when the gap could close within the
    // step: gap < closing_speed * dt + slop.
    static void emit(const Body& a, const Body& b, BodyId ia, BodyId ib,
                     Vec3 normal, float gap, float dt, std::vector<Contact>& out) {
        float vn = dot(a.velocity - b.velocity, normal);
        constexpr float slop = 1e-3f;
        if (gap > -vn * dt + slop && gap > slop) return; // cannot touch this step
        out.push_back({ia, ib, normal, gap, vn, 0.0f, 0.0f});
    }

    static void collide(const Body& a, const Body& b, BodyId ia, BodyId ib,
                        float dt, std::vector<Contact>& out) {
        if (a.shape == ShapeType::Sphere && b.shape == ShapeType::Plane) {
            spherePlane(a, b, ia, ib, dt, out);
        } else if (a.shape == ShapeType::Plane && b.shape == ShapeType::Sphere) {
            spherePlane(b, a, ib, ia, dt, out);
        } else if (a.shape == ShapeType::Sphere && b.shape == ShapeType::Sphere) {
            Vec3 d = a.position - b.position;
            float dist = length(d);
            Vec3 n = dist > 1e-8f ? d * (1.0f / dist) : Vec3{0, 1, 0};
            emit(a, b, ia, ib, n, dist - (a.radius + b.radius), dt, out);
        }
    }

    static void spherePlane(const Body& s, const Body& p, BodyId is, BodyId ip,
                            float dt, std::vector<Contact>& out) {
        float gap = dot(p.planeNormal, s.position) - p.planeOffset - s.radius;
        emit(s, p, is, ip, p.planeNormal, gap, dt, out);
    }
};

Backend* createCpuBackend() { return new CpuBackend(); }

} // namespace velox
