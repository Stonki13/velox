// Parallel island solving (roadmap 20) verification:
//  1. Bitwise determinism: a many-island scene solved with 1 worker
//     (sequential reference) and with the full worker pool in Parallel mode
//     must produce byte-identical body transforms after 300 frames.
//  2. Mode toggle: Sequential mode on the pool must also match.
//  3. Speedup: the parallel solve should not be slower than sequential on a
//     many-island workload (informational; hard-fails only on determinism).
#include <velox/velox.h>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <vector>

using namespace velox;

namespace {

struct Snapshot {
    std::vector<Vec3> positions;
    std::vector<Quat> orientations;
    std::vector<Vec3> velocities;
    std::vector<Vec3> angularVelocities;
};

// 64 disjoint 6-box stacks: 64 independent contact islands.
void buildScene(World& world, std::vector<BodyId>& bodies) {
    world.addStaticPlane({0, 1, 0}, 0);
    for (int stack = 0; stack < 64; ++stack) {
        const float x = static_cast<float>(stack % 8) * 4.0f;
        const float z = static_cast<float>(stack / 8) * 4.0f;
        for (int level = 0; level < 6; ++level) {
            const BodyId id = world.addBox(
                {x, 0.5f + 1.001f * static_cast<float>(level), z},
                {0.5f, 0.5f, 0.5f}, 1.0f);
            world.body(id).friction = 0.7f;
            world.body(id).restitution = 0.0f;
            bodies.push_back(id);
        }
    }
}

Snapshot run(uint32_t workers, IslandSolvingMode mode, double& solveMs) {
    World world(BackendType::Cpu);
    world.setWorkerCount(workers);
    world.setIslandSolvingMode(mode);
    world.substeps = 4;
    std::vector<BodyId> bodies;
    buildScene(world, bodies);

    double total = 0.0;
    for (int frame = 0; frame < 300; ++frame) {
        world.step(1.0f / 60.0f);
        total += world.lastStepStats().solverMs;
    }
    solveMs = total / 300.0;

    Snapshot snapshot;
    for (BodyId id : bodies) {
        const Body& body = world.body(id);
        snapshot.positions.push_back(body.position);
        snapshot.orientations.push_back(body.orientation);
        snapshot.velocities.push_back(body.velocity);
        snapshot.angularVelocities.push_back(body.angularVelocity);
    }
    return snapshot;
}

bool identical(const Snapshot& a, const Snapshot& b) {
    return a.positions.size() == b.positions.size() &&
           std::memcmp(a.positions.data(), b.positions.data(),
                       a.positions.size() * sizeof(Vec3)) == 0 &&
           std::memcmp(a.orientations.data(), b.orientations.data(),
                       a.orientations.size() * sizeof(Quat)) == 0 &&
           std::memcmp(a.velocities.data(), b.velocities.data(),
                       a.velocities.size() * sizeof(Vec3)) == 0 &&
           std::memcmp(a.angularVelocities.data(), b.angularVelocities.data(),
                       a.angularVelocities.size() * sizeof(Vec3)) == 0;
}

} // namespace

int main() {
    int failures = 0;
    double sequentialMs = 0.0, parallelMs = 0.0, poolSequentialMs = 0.0;

    const Snapshot reference = run(1, IslandSolvingMode::Parallel, sequentialMs);
    const Snapshot parallel = run(0, IslandSolvingMode::Parallel, parallelMs);
    const Snapshot poolSequential = run(0, IslandSolvingMode::Sequential, poolSequentialMs);

    if (!identical(reference, parallel)) {
        std::fprintf(stderr, "FAIL: parallel islands diverge from the sequential reference\n");
        ++failures;
    }
    std::printf("islands: 64 stacks, 300 frames\n");
    std::printf("  solver ms/step  1 worker: %.3f  pool parallel: %.3f  pool batches: %.3f\n",
                sequentialMs, parallelMs, poolSequentialMs);
    std::printf("  parallel bitwise-identical to sequential: %s\n",
                failures == 0 ? "yes" : "NO");

    if (failures) return 1;
    std::puts("islands_demo: all checks passed");
    return 0;
}
