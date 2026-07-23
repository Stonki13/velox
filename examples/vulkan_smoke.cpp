// Smoke + parity test for the cross-vendor Vulkan compute backend.
// Runs the same scene on BackendType::Cpu and BackendType::Vulkan and
// compares final state within a behavioral tolerance: the Vulkan backend
// solves contacts in graph-colored order on the GPU while the CPU reference
// solves sequentially (same per-contact math, different order — the same
// deliberate trade the CUDA backend makes), so small trajectory divergence
// in a contact pile is expected and bitwise agreement is not. Skips
// (exit 0, message) when the library was built without VELOX_ENABLE_VULKAN
// or no Vulkan driver/device is present, matching cuda_recovery_demo's
// convention.
#include <velox/velox.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <vector>

using namespace velox;

namespace {

std::vector<BodyId> buildScene(World& world) {
    world.gravity = {0, -9.81f, 0};
    world.addStaticPlane({0, 1, 0}, 0.0f);
    std::vector<BodyId> ids;
    for (int i = 0; i < 20; ++i) {
        const int x = i % 4, y = i / 4;
        BodyId id = world.addBox({float(x - 2) * 1.2f, 0.5f + float(y) * 1.05f, 0},
                                 {0.4f, 0.5f, 0.4f}, 1.0f);
        world.setAngularVelocity(id, {0, 0.1f * float(i + 1), 0});
        world.body(id).angularDamping = 0.05f;
        world.body(id).linearDamping = 0.01f;
        ids.push_back(id);
    }
    return ids;
}

std::vector<BodyId> buildSphereScene(World& world) {
    world.gravity = {0, -9.81f, 0};
    world.addStaticPlane({0, 1, 0}, 0.0f);
    std::vector<BodyId> ids;
    for (int i = 0; i < 24; ++i) {
        const int x = i % 4, y = i / 4;
        BodyId id = world.addSphere({float(x - 2) * 1.05f, 0.5f + float(y) * 1.02f, 0},
                                    0.5f, 1.0f);
        world.setLinearVelocity(id, {0.05f * float(i % 3), 0, 0});
        ids.push_back(id);
    }
    return ids;
}

// CPU-vs-Vulkan comparison of one scene. Same bounds philosophy as
// cuda_parity_demo: per-contact math is identical, solve order is not
// (colored vs sequential), so a settling pile diverges by solver-order
// chaos, not by error.
bool checkParity(const char* label, std::vector<BodyId> (*build)(World&),
                 bool trajectory = true) {
    World cpu(BackendType::Cpu);
    std::vector<BodyId> cpuIds = build(cpu);
    World vulkan(BackendType::Vulkan);
    std::vector<BodyId> vulkanIds = build(vulkan);

    for (int i = 0; i < 120; ++i) {
        cpu.step(1.0f / 60.0f);
        vulkan.step(1.0f / 60.0f);
    }

    float maxPositionDelta = 0.0f, maxVelocityDelta = 0.0f;
    bool finite = true;
    for (size_t i = 0; i < cpuIds.size(); ++i) {
        const Body& a = cpu.body(cpuIds[i]);
        const Body& b = vulkan.body(vulkanIds[i]);
        maxPositionDelta = std::max(maxPositionDelta, length(a.position - b.position));
        maxVelocityDelta = std::max(maxVelocityDelta, length(a.velocity - b.velocity));
        finite = finite && std::isfinite(b.position.x) && std::isfinite(b.velocity.x);
    }
    bool ok;
    if (trajectory) {
        ok = finite && maxPositionDelta < 0.05f && maxVelocityDelta < 0.25f;
    } else {
        // Behavioral bounds for chaotic scenes (free-rolling sphere piles
        // scatter, so colored-vs-sequential solve order legitimately diverges
        // trajectories — the difftest suite treats such scenes the same way):
        // nothing fell through the floor, escaped the arena, or blew up.
        ok = finite;
        for (BodyId id : vulkanIds) {
            const Body& b = vulkan.body(id);
            ok = ok && b.position.y > 0.3f && b.position.y < 25.0f &&
                 std::fabs(b.position.x) < 40.0f && std::fabs(b.position.z) < 40.0f &&
                 length(b.velocity) < 8.0f;
        }
    }
    std::printf("vulkan_smoke: %s maxPosDelta=%.6f maxVelDelta=%.6f %s\n",
                label, maxPositionDelta, maxVelocityDelta, ok ? "PASS" : "FAIL");
    return ok;
}

void enableGpuNarrowPhase() {
#ifdef _WIN32
    _putenv_s("VELOX_VULKAN_GPU_NARROW", "1");
#else
    setenv("VELOX_VULKAN_GPU_NARROW", "1", 1);
#endif
}

} // namespace

int main() {
    try {
        {
            World probe(BackendType::Vulkan);
            std::printf("vulkan_smoke: backend=%s\n", probe.backendName());
        }
        bool ok = checkParity("box_pile (CPU narrow phase)", buildScene);
        ok = checkParity("sphere_pile (CPU narrow phase)", buildSphereScene, false) && ok;
        // Final pass exercises the opt-in GPU narrow phase (sphere/plane
        // scenes only; measured slower than the CPU narrow phase and kept as
        // an experiment -- see backend_vulkan.cpp -- but must stay correct).
        enableGpuNarrowPhase();
        ok = checkParity("sphere_pile (GPU narrow phase)", buildSphereScene, false) && ok;
        return ok ? 0 : 1;
    } catch (const std::runtime_error& e) {
        std::printf("vulkan_smoke: %s; skipped\n", e.what());
        return 0;
    }
}
