#include "velox/backend.h"
#include "ccd.h"

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
    static void collide(const Body& a, const Body& b, BodyId ia, BodyId ib,
                        float dt, std::vector<Contact>& out) {
        if (a.shape == ShapeType::Sphere && b.shape == ShapeType::Plane) {
            spherePlane(a, b, ia, ib, dt, out);
        } else if (a.shape == ShapeType::Plane && b.shape == ShapeType::Sphere) {
            spherePlane(b, a, ib, ia, dt, out);
        } else if (a.shape == ShapeType::Sphere && b.shape == ShapeType::Sphere) {
            float toi = sweepSphereSphere(a, b, dt);
            if (toi < 0.0f) return;
            Vec3 pa = a.position + a.velocity * (dt * toi);
            Vec3 pb = b.position + b.velocity * (dt * toi);
            Vec3 n = normalize(pa - pb);
            float depth = (a.radius + b.radius) - length(pa - pb);
            out.push_back({ia, ib, n, depth > 0 ? depth : 0, toi});
        }
    }

    static void spherePlane(const Body& s, const Body& p, BodyId is, BodyId ip,
                            float dt, std::vector<Contact>& out) {
        float toi = sweepSpherePlane(s, p, dt);
        if (toi < 0.0f) return;
        Vec3 pos = s.position + s.velocity * (dt * toi);
        float depth = s.radius - (dot(p.planeNormal, pos) - p.planeOffset);
        out.push_back({is, ip, p.planeNormal, depth > 0 ? depth : 0, toi});
    }
};

Backend* createCpuBackend() { return new CpuBackend(); }

} // namespace velox
