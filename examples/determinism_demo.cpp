#include <velox/velox.h>

#include <cstdio>
#include <cstdint>
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

void hashFloat(uint64_t& hash, float value) {
    uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    for (int shift = 0; shift < 32; shift += 8) {
        hash ^= (bits >> shift) & 0xffu;
        hash *= 1099511628211ull;
    }
}

void hashState(uint64_t& hash, const velox::Body& body) {
    const float values[] = {
        body.position.x, body.position.y, body.position.z,
        body.orientation.x, body.orientation.y, body.orientation.z, body.orientation.w,
        body.velocity.x, body.velocity.y, body.velocity.z,
        body.angularVelocity.x, body.angularVelocity.y, body.angularVelocity.z,
    };
    for (float value : values) hashFloat(hash, value);
}

std::vector<State> runScene(uint32_t workers, int frames, uint64_t* trace = nullptr) {
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
    if (trace) *trace = 1469598103934665603ull;
    for (int frame = 0; frame < frames; ++frame) {
        world.step(1.0f / 60.0f);
        if (trace)
            for (velox::BodyId id : bodies) hashState(*trace, world.body(id));
    }

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

int main(int argc, char** argv) {
#if VELOX_STRICT_FLOATING_POINT
    const bool traceOnly = argc == 2 && std::strcmp(argv[1], "--trace") == 0;
    velox::World backendSelection(velox::BackendType::Auto);
    backendSelection.setDeterminismMode(velox::DeterminismMode::Strict);
    const bool selectedCpu = std::strcmp(backendSelection.backendName(), "cpu") == 0;
    bool rejectedParallel = false;
    try {
        backendSelection.setIslandSolvingMode(velox::IslandSolvingMode::Parallel);
    } catch (const std::logic_error&) {
        rejectedParallel = true;
    }

    uint64_t serialTrace = 0, parallelTrace = 0;
    const std::vector<State> serial = runScene(1, 360, &serialTrace);
    const std::vector<State> parallel = runScene(0, 360, &parallelTrace);
    const bool identical = serial.size() == parallel.size() &&
        serialTrace == parallelTrace &&
        std::memcmp(serial.data(), parallel.data(), serial.size() * sizeof(State)) == 0;
    if (!selectedCpu || !rejectedParallel || !identical) {
        std::fprintf(stderr, "determinism_demo: strict mode contract failed\n");
        return 1;
    }
    if (traceOnly) {
        uint64_t trace = 0;
        runScene(0, 1000, &trace);
        std::printf("VELOX_STRICT_TRACE %016llx\n",
                    static_cast<unsigned long long>(trace));
        return 0;
    }
    std::puts("determinism_demo: strict CPU replay is bitwise identical");
#else
    (void)argc;
    (void)argv;
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
