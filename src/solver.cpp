#include "velox/backend.h"
#include "gjk.h"
#include <algorithm>

namespace velox {

namespace {

bool isConvexVolume(ShapeType t) {
    return t == ShapeType::Sphere || t == ShapeType::Box || t == ShapeType::Capsule;
}

// Velocity of a body's surface at a world point (linear + angular).
Vec3 pointVelocity(const Body& b, const Vec3& p) {
    return b.velocity + cross(b.angularVelocity, p - b.position);
}

// A pair becomes a speculative contact when the gap could close within the
// step. Point velocities (not just center velocities) decide, so spin counts.
void emit(const Body& a, const Body& b, BodyId ia, BodyId ib,
          const Vec3& normal, const Vec3& point, float gap, float dt,
          std::vector<Contact>& out) {
    float vn = dot(pointVelocity(a, point) - pointVelocity(b, point), normal);
    constexpr float slop = 1e-3f;
    if (gap > -vn * dt + slop && gap > slop) return; // cannot touch this step
    out.push_back({ia, ib, normal, point, gap, vn, 0.0f, 0.0f});
}

// --- plane vs convex: manifold from the deepest core features ---------------
void planeConvex(const Body& conv, const Body& plane, BodyId ic, BodyId ip,
                 float dt, std::vector<Contact>& out) {
    const Vec3& n = plane.planeNormal;
    auto planeGap = [&](const Vec3& p, float r) {
        return dot(n, p) - plane.planeOffset - r;
    };

    switch (conv.shape) {
    case ShapeType::Sphere:
        emit(conv, plane, ic, ip, n, conv.position - n * conv.radius,
             planeGap(conv.position, conv.radius), dt, out);
        break;
    case ShapeType::Capsule: {
        Vec3 axis = rotate(conv.orientation, {0, conv.capsuleHalfHeight, 0});
        for (Vec3 end : {conv.position + axis, conv.position - axis})
            emit(conv, plane, ic, ip, n, end - n * conv.radius,
                 planeGap(end, conv.radius), dt, out);
        break;
    }
    case ShapeType::Box: {
        // Up to 4 deepest vertices form the manifold (a resting box needs
        // multiple contact points or it wobbles on a single one).
        struct V { Vec3 p; float gap; };
        V verts[8];
        int k = 0;
        for (int i = 0; i < 8; ++i) {
            Vec3 local{(i & 1) ? conv.halfExtents.x : -conv.halfExtents.x,
                       (i & 2) ? conv.halfExtents.y : -conv.halfExtents.y,
                       (i & 4) ? conv.halfExtents.z : -conv.halfExtents.z};
            Vec3 p = conv.position + rotate(conv.orientation, local);
            verts[k++] = {p, planeGap(p, 0.0f)};
        }
        std::sort(verts, verts + 8, [](const V& a, const V& b) { return a.gap < b.gap; });
        for (int i = 0; i < 4; ++i)
            emit(conv, plane, ic, ip, n, verts[i].p, verts[i].gap, dt, out);
        break;
    }
    default: break;
    }
}

// --- convex vs convex via GJK ------------------------------------------------
void convexConvex(const Body& a, const Body& b, BodyId ia, BodyId ib,
                  float dt, std::vector<Contact>& out) {
    GjkResult r = gjkDistance(makeConvex(a), makeConvex(b));
    Vec3 mid = (r.pointA + r.pointB) * 0.5f;
    emit(a, b, ia, ib, r.normal, mid, r.distance, dt, out);

    // Box-box face contact needs more than one point to rest stably: add the
    // box vertices that are nearly as close to the other body's surface.
    if (a.shape == ShapeType::Box && b.shape == ShapeType::Box && r.distance < 0.05f) {
        Convex cb = makeConvex(b);
        for (int i = 0; i < 8; ++i) {
            Vec3 local{(i & 1) ? a.halfExtents.x : -a.halfExtents.x,
                       (i & 2) ? a.halfExtents.y : -a.halfExtents.y,
                       (i & 4) ? a.halfExtents.z : -a.halfExtents.z};
            Vec3 p = a.position + rotate(a.orientation, local);
            float d = dot(r.normal, p - r.pointB);
            if (d < r.distance + 0.02f && lengthSq(p - mid) > 1e-6f)
                emit(a, b, ia, ib, r.normal, p, d, dt, out);
        }
        (void)cb;
    }
}

// --- static mesh vs convex: per-triangle GJK with AABB culling ---------------
void meshConvex(const Body& conv, const Body& meshBody, BodyId ic, BodyId im,
                const MeshSoup& soup, float dt, std::vector<Contact>& out) {
    const Mesh& m = soup.meshes[meshBody.meshIndex];

    // Cull triangles against the convex body's swept AABB.
    float reach = conv.maxPointSpeed() * dt +
                  (conv.shape == ShapeType::Box ? length(conv.halfExtents)
                                                : conv.radius + conv.capsuleHalfHeight) +
                  1e-2f;
    Vec3 lo = conv.position - Vec3{reach, reach, reach};
    Vec3 hi = conv.position + Vec3{reach, reach, reach};
    if (hi.x < m.aabbMin.x || lo.x > m.aabbMax.x ||
        hi.y < m.aabbMin.y || lo.y > m.aabbMax.y ||
        hi.z < m.aabbMin.z || lo.z > m.aabbMax.z) return;

    Convex cc = makeConvex(conv);
    for (uint32_t t = 0; t < m.indexCount; t += 3) {
        const Vec3& v0 = soup.vertices[m.firstVertex + soup.indices[m.firstIndex + t]];
        const Vec3& v1 = soup.vertices[m.firstVertex + soup.indices[m.firstIndex + t + 1]];
        const Vec3& v2 = soup.vertices[m.firstVertex + soup.indices[m.firstIndex + t + 2]];

        Vec3 tlo{std::min({v0.x, v1.x, v2.x}), std::min({v0.y, v1.y, v2.y}), std::min({v0.z, v1.z, v2.z})};
        Vec3 thi{std::max({v0.x, v1.x, v2.x}), std::max({v0.y, v1.y, v2.y}), std::max({v0.z, v1.z, v2.z})};
        if (hi.x < tlo.x || lo.x > thi.x || hi.y < tlo.y || lo.y > thi.y ||
            hi.z < tlo.z || lo.z > thi.z) continue;

        GjkResult r = gjkDistance(cc, makeTriangle(v0, v1, v2));
        emit(conv, meshBody, ic, im, r.normal, (r.pointA + r.pointB) * 0.5f,
             r.distance, dt, out);
    }
}

} // namespace

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

    void findContacts(const std::vector<Body>& bodies, const MeshSoup& meshes,
                      float dt, std::vector<Contact>& out) override {
        out.clear();
        // O(n^2) all-pairs for now; sweep-and-prune broad phase is next.
        for (BodyId i = 0; i < bodies.size(); ++i) {
            for (BodyId j = i + 1; j < bodies.size(); ++j) {
                const Body& a = bodies[i];
                const Body& b = bodies[j];
                if (a.isStatic() && b.isStatic()) continue;

                if (isConvexVolume(a.shape) && isConvexVolume(b.shape))
                    convexConvex(a, b, i, j, dt, out);
                else if (isConvexVolume(a.shape) && b.shape == ShapeType::Plane)
                    planeConvex(a, b, i, j, dt, out);
                else if (a.shape == ShapeType::Plane && isConvexVolume(b.shape))
                    planeConvex(b, a, j, i, dt, out);
                else if (isConvexVolume(a.shape) && b.shape == ShapeType::Mesh)
                    meshConvex(a, b, i, j, meshes, dt, out);
                else if (a.shape == ShapeType::Mesh && isConvexVolume(b.shape))
                    meshConvex(b, a, j, i, meshes, dt, out);
            }
        }
    }
};

Backend* createCpuBackend() { return new CpuBackend(); }

} // namespace velox
