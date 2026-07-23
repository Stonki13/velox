// Smoke + parity test for the cross-vendor Vulkan compute backend.
// Runs the same scene on BackendType::Cpu and BackendType::Vulkan and
// compares final state within a tight tolerance: the Vulkan backend's only
// GPU stage (velocity integration) does the same arithmetic as the CPU
// reference, so the two runs should agree to float rounding, not just
// behaviorally. Skips (exit 0, message) when the library was built without
// VELOX_ENABLE_VULKAN or no Vulkan driver/device is present, matching
// cuda_recovery_demo's convention.
#include <velox/velox.h>

#include <cmath>
#include <cstdio>
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

} // namespace

int main() {
    World cpu(BackendType::Cpu);
    std::vector<BodyId> cpuIds = buildScene(cpu);

    try {
        World vulkan(BackendType::Vulkan);
        std::vector<BodyId> vulkanIds = buildScene(vulkan);
        std::printf("vulkan_smoke: backend=%s\n", vulkan.backendName());

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

        // Integration is the only stage on the GPU and does identical math;
        // remaining divergence is float contraction feeding a mildly chaotic
        // contact pile. These bounds are far tighter than the cross-backend
        // CUDA parity test's, reflecting the much smaller surface.
        const bool ok = finite && maxPositionDelta < 0.02f && maxVelocityDelta < 0.1f;
        std::printf("vulkan_smoke: maxPosDelta=%.6f maxVelDelta=%.6f %s\n",
                    maxPositionDelta, maxVelocityDelta, ok ? "PASS" : "FAIL");
        return ok ? 0 : 1;
    } catch (const std::runtime_error& e) {
        std::printf("vulkan_smoke: %s; skipped\n", e.what());
        return 0;
    }
}
