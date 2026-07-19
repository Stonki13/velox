#include <velox/velox.h>

#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace {

struct State {
    velox::Vec3 position;
    velox::Quat orientation;
    velox::Vec3 velocity;
    velox::Vec3 angularVelocity;
};

std::vector<State> runScene(uint32_t workers) {
    velox::World world(velox::BackendType::Auto);
    world.setDeterminismMode(velox::DeterminismMode::Strict);
    world.setWorkerCount(workers);
    world.gravity = {0.0f, -9.81f, 0.0f};
    world.addStaticPlane({0.0f, 1.0f, 0.0f}, 0.0f);

    std::vector<velox::BodyId> bodies;
    for (int i = 0; i < 12; ++i) {
        const int x = i % 3;
        const int y = i / 3;
        const velox::BodyId id = world.addBox(
            {float(x - 1) * 0.75f, 0.5f + float(y) * 1.02f, 0.0f},
            {0.35f, 0.5f, 0.35f}, 1.0f);
        world.setAngularVelocity(id, {0.0f, 0.15f * float(i + 1), 0.0f});
        bodies.push_back(id);
    }
    for (int frame = 0; frame < 360; ++frame) world.step(1.0f / 60.0f);

    std::vector<State> result;
    result.reserve(bodies.size());
    for (velox::BodyId id : bodies) {
        const velox::Body& body = world.body(id);
        result.push_back({body.position, body.orientation, body.velocity,
                          body.angularVelocity});
    }
    return result;
}

} // namespace

int main() {
#if VELOX_STRICT_FLOATING_POINT
    velox::World backendSelection(velox::BackendType::Auto);
    backendSelection.setDeterminismMode(velox::DeterminismMode::Strict);
    const bool selectedCpu = std::strcmp(backendSelection.backendName(), "cpu") == 0;
    bool rejectedParallel = false;
    try {
        backendSelection.setIslandSolvingMode(velox::IslandSolvingMode::Parallel);
    } catch (const std::logic_error&) {
        rejectedParallel = true;
    }

    const std::vector<State> serial = runScene(1);
    const std::vector<State> parallel = runScene(0);
    const bool identical = serial.size() == parallel.size() &&
        std::memcmp(serial.data(), parallel.data(), serial.size() * sizeof(State)) == 0;
    if (!selectedCpu || !rejectedParallel || !identical) {
        std::fprintf(stderr, "determinism_demo: strict mode contract failed\n");
        return 1;
    }
    std::puts("determinism_demo: strict CPU replay is bitwise identical");
#else
    velox::World world(velox::BackendType::Cpu);
    bool rejected = false;
    try {
        world.setDeterminismMode(velox::DeterminismMode::Strict);
    } catch (const std::logic_error&) {
        rejected = true;
    }
    if (!rejected) {
        std::fprintf(stderr, "determinism_demo: relaxed build accepted strict mode\n");
        return 1;
    }
    std::puts("determinism_demo: strict mode correctly requires a strict build");
#endif
    return 0;
}
