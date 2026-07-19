#include <velox/velox.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace {

void setCudaFailure(const char* value) {
#ifdef _WIN32
    _putenv_s("VELOX_CUDA_TEST_FAIL", value ? value : "");
#else
    if (value) setenv("VELOX_CUDA_TEST_FAIL", value, 1);
    else unsetenv("VELOX_CUDA_TEST_FAIL");
#endif
}

void addScene(velox::World& world, velox::BodyId& ball) {
    world.gravity = {0, -9.81f, 0};
    world.addStaticPlane({0, 1, 0}, 0.0f);
    ball = world.addSphere({0, 2, 0}, 0.25f, 1.0f);
}

bool finiteBody(const velox::Body& body) {
    return std::isfinite(body.position.x) && std::isfinite(body.position.y) &&
           std::isfinite(body.position.z) && std::isfinite(body.velocity.x) &&
           std::isfinite(body.velocity.y) && std::isfinite(body.velocity.z);
}

} // namespace

int main() {
    velox::World probe(velox::BackendType::Auto);
    if (std::string(probe.backendName()) != "cuda") {
        std::puts("cuda_recovery_demo: CUDA unavailable; skipped");
        return 0;
    }

    bool ok = true;
    velox::BodyId ball;
    addScene(probe, ball);
    probe.setDeviceLossPolicy(velox::DeviceLossPolicy::FallbackToCPU);
    setCudaFailure("oom");
    probe.step(1.0f / 60.0f);
    setCudaFailure(nullptr);
    ok &= probe.isOnCPUBackend() && finiteBody(probe.bodyState(ball));

    const bool restoredCuda = probe.resetCUDABackend();
    ok &= restoredCuda && !probe.isOnCPUBackend();
    probe.step(1.0f / 60.0f);
    ok &= finiteBody(probe.bodyState(ball));

    velox::World deviceLoss(velox::BackendType::Auto);
    velox::BodyId deviceLossBall;
    addScene(deviceLoss, deviceLossBall);
    deviceLoss.setDeviceLossPolicy(velox::DeviceLossPolicy::FallbackToCPU);
    setCudaFailure("device-loss");
    deviceLoss.step(1.0f / 60.0f);
    setCudaFailure(nullptr);
    ok &= deviceLoss.isOnCPUBackend() && finiteBody(deviceLoss.bodyState(deviceLossBall));
    ok &= deviceLoss.resetCUDABackend() && !deviceLoss.isOnCPUBackend();

    velox::World throwing(velox::BackendType::Auto);
    velox::BodyId throwingBall;
    addScene(throwing, throwingBall);
    throwing.setDeviceLossPolicy(velox::DeviceLossPolicy::ThrowException);
    bool propagated = false;
    setCudaFailure("device-loss");
    try {
        throwing.step(1.0f / 60.0f);
    } catch (const velox::BackendFailure& failure) {
        propagated = failure.kind() == velox::BackendFailureKind::DeviceLost;
    }
    setCudaFailure(nullptr);
    ok &= propagated && !throwing.isOnCPUBackend();

    std::printf("cuda_recovery_demo: %s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
