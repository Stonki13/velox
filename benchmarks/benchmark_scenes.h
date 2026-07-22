#pragma once
// Shared scene builders for the Velox benchmark suite.
//
// Each helper populates a freshly constructed velox::World with a reproducible
// layout so the individual benchmark_*.cpp programs stay focused on what they
// measure. The layouts mirror the scenes in examples/benchmark.cpp so results
// are comparable with the historical CPU/CUDA benchmark.

#include <velox/velox.h>

#include <cmath>
#include <cstdint>
#include <vector>

namespace veloxbench {
namespace scenes {

/// A walled floor that keeps falling bodies contained. Used by the rain scenes.
inline void addWalledFloor(velox::World& w) {
    w.addStaticPlane({0, 1, 0}, 0.0f);
    w.addStaticPlane({1, 0.2f, 0}, -25.0f);
    w.addStaticPlane({-1, 0.2f, 0}, -25.0f);
    w.addStaticPlane({0, 0.2f, 1}, -25.0f);
    w.addStaticPlane({0, 0.2f, -1}, -25.0f);
}

/// Drop `n` unit spheres in a cubic lattice above a walled floor. Produces a
/// dense dynamic/dynamic contact workload that stresses broadphase pair
/// generation, narrowphase contact generation, and the contact solver together.
inline void sphereRain(velox::World& w, int n) {
    addWalledFloor(w);
    int side = (int)std::ceil(std::cbrt((float)n));
    for (int i = 0; i < n; ++i) {
        int x = i % side, y = (i / side) % side, z = i / (side * side);
        w.addSphere({(x - side * 0.5f) * 1.1f, 2.0f + y * 1.1f,
                     (z - side * 0.5f) * 1.1f},
                    0.5f, 1.0f);
    }
}

/// A grid of `n` static boxes plus a single dynamic sphere resting on them.
/// The boxes are spaced so the sphere only touches a few at a time, giving a
/// large body count with a controlled, modest contact count - ideal for
/// isolating broadphase cost from narrowphase/solver cost.
inline void staticField(velox::World& w, int n) {
    w.gravity = {0, 0, 0};
    int side = (int)std::ceil(std::sqrt((float)n));
    for (int i = 0; i < n; ++i) {
        int x = i % side, z = i / side;
        w.addBox({(x - side * 0.5f) * 3.0f, 0.0f, (z - side * 0.5f) * 3.0f},
                 {0.5f, 0.5f, 0.5f}, 0.0f);
    }
    // One dynamic body that we can move around to drive query/overlap work.
    w.addSphere({0.0f, 0.6f, 0.0f}, 0.5f, 1.0f);
}

/// `n` independent distance joints fanning out from a shared anchor, with no
/// gravity. Each joint is its own solver island, so this measures joint-solver
/// throughput and island dispatch rather than contact resolution.
inline void jointFan(velox::World& w, int n) {
    w.gravity = {0, 0, 0};
    auto anchor = w.addSphere({}, 0.1f, 0.0f);
    w.body(anchor).maskBits = 0;
    for (int i = 0; i < n; ++i) {
        float x = 2.0f + i * 0.5f;
        auto body = w.addSphere({x, 0, 0}, 0.1f, 1.0f);
        w.body(body).maskBits = 0;
        w.addDistanceJoint(anchor, body, {}, {x, 0, 0});
        w.setLinearVelocity(body, {0, 1, 0});
    }
}

/// A bumpy triangle-mesh terrain (grid x grid quads => 2*grid^2 triangles)
/// with `n` spheres dropped onto it. Exercises BVH-backed mesh narrowphase.
inline void meshTerrain(velox::World& w, int n, int grid) {
    std::vector<velox::Vec3> verts;
    std::vector<uint32_t> idx;
    float size = 60.0f, half = size * 0.5f;
    for (int z = 0; z <= grid; ++z)
        for (int x = 0; x <= grid; ++x) {
            float fx = x * size / grid - half, fz = z * size / grid - half;
            float fy = std::sin(fx * 0.4f) * std::cos(fz * 0.4f) * 1.5f;
            verts.push_back({fx, fy, fz});
        }
    for (int z = 0; z < grid; ++z)
        for (int x = 0; x < grid; ++x) {
            uint32_t a = z * (grid + 1) + x, b = a + 1, c = a + grid + 1,
                     d = c + 1;
            idx.insert(idx.end(), {a, c, b, b, c, d});
        }
    w.addStaticMesh(verts, idx);

    int side = (int)std::ceil(std::cbrt((float)n));
    for (int i = 0; i < n; ++i) {
        int x = i % side, y = (i / side) % side, z = i / (side * side);
        w.addSphere({(x - side * 0.5f) * 1.2f, 6.0f + y * 1.2f,
                     (z - side * 0.5f) * 1.2f},
                    0.5f, 1.0f);
    }
}

} // namespace scenes
} // namespace veloxbench
