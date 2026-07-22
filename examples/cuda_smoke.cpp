#include <velox/velox.h>

#include <cmath>
#include <cstdio>
#include <cstring>

namespace {

bool finiteBody(const velox::Body& body) {
    return std::isfinite(body.position.x) && std::isfinite(body.position.y) &&
           std::isfinite(body.position.z) && std::isfinite(body.velocity.x) &&
           std::isfinite(body.velocity.y) && std::isfinite(body.velocity.z);
}

void addScene(velox::World& world, velox::BodyId& tracked) {
    world.gravity = {0.0f, -9.81f, 0.0f};
    world.substeps = 4;
    world.addStaticPlane({0.0f, 1.0f, 0.0f}, 0.0f);
    world.addBox({3.0f, 0.5f, 0.0f}, {0.25f, 0.5f, 2.0f}, 0.0f);
    tracked = world.addSphere({0.0f, 4.0f, 0.0f}, 0.4f, 1.0f);
    world.addBox({-0.7f, 6.0f, 0.0f}, {0.35f, 0.35f, 0.35f}, 1.0f);
}

} // namespace

int main() {
#if !VELOX_HAS_CUDA
    std::fputs("cuda_smoke was built without CUDA support\n", stderr);
    return 1;
#else
    velox::World cpu(velox::BackendType::Cpu);
    velox::World cuda(velox::BackendType::Cuda);
    if (std::strcmp(cuda.backendName(), "cuda") != 0) {
        std::fputs("cuda_smoke: CUDA backend did not initialize\n", stderr);
        return 1;
    }

    velox::BodyId cpuTracked;
    velox::BodyId cudaTracked;
    addScene(cpu, cpuTracked);
    addScene(cuda, cudaTracked);

    for (int frame = 0; frame < 240; ++frame) {
        cpu.step(1.0f / 60.0f);
        cuda.step(1.0f / 60.0f);
    }

    const velox::Body cpuBody = cpu.bodyState(cpuTracked);
    const velox::Body cudaBody = cuda.bodyState(cudaTracked);
    const float positionError = velox::length(cpuBody.position - cudaBody.position);
    const float velocityError = velox::length(cpuBody.velocity - cudaBody.velocity);
    const bool ok = finiteBody(cudaBody) && cuda.lastStepStats().deviceSubsteps &&
                    positionError < 2.0e-2f && velocityError < 5.0e-2f;
    std::printf("cuda_smoke: %s (position error %.6f, velocity error %.6f)\n",
                ok ? "PASS" : "FAIL", positionError, velocityError);
    return ok ? 0 : 1;
#endif
}
