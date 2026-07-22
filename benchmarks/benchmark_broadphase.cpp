// Broadphase benchmark.
//
// Measures the broad-phase stage in isolation by reading the per-step
// StepStats::broadPhaseMs the engine records. Scenes vary body count and
// spatial density so we capture both the cost of proxy management and the cost
// of pair generation as the world gets crowded.
//
// Each measured iteration restores a saved snapshot, advances one step, and
// records the broad-phase time, so allocation and warm-up stay out of the
// numbers.

#include "benchmark_cli.h"
#include "benchmark_scenes.h"

#include <velox/velox.h>

#include <string>

using namespace veloxbench;

namespace {

/// Run one broadphase-focused case at a given body count.
BenchResult runBroadphase(const Options& opts, int bodyCount) {
    velox::World world(velox::BackendType::Cpu);
    scenes::sphereRain(world, bodyCount);
    velox::WorldSnapshot baseline = world.saveSnapshot();

    BenchConfig config;
    config.name = "broadphase/sphere_rain_" + std::to_string(bodyCount);
    config.category = "broadphase";
    config.bodyCount = bodyCount;
    config = opts.apply(config);

    auto body = [&]() -> double {
        world.restoreSnapshot(baseline);
        world.step(1.0f / 60.0f);
        return world.lastStepStats().broadPhaseMs;
    };

    auto proxies = [&]() -> double {
        return (double)world.lastStepStats().broadPhaseProxies;
    };

    return runCase(config, body, proxies, "broadPhaseProxies");
}

/// A sparse static field isolates proxy upkeep from contact work: many bodies,
/// almost no overlapping pairs, so broadPhaseMs is dominated by the AABB tree
/// update rather than pair emission.
BenchResult runStaticField(const Options& opts, int bodyCount) {
    velox::World world(velox::BackendType::Cpu);
    scenes::staticField(world, bodyCount);
    velox::WorldSnapshot baseline = world.saveSnapshot();

    BenchConfig config;
    config.name = "broadphase/static_field_" + std::to_string(bodyCount);
    config.category = "broadphase";
    config.bodyCount = bodyCount;
    config = opts.apply(config);

    auto body = [&]() -> double {
        world.restoreSnapshot(baseline);
        world.step(1.0f / 60.0f);
        return world.lastStepStats().broadPhaseMs;
    };
    auto pairs = [&]() -> double {
        return (double)world.lastStepStats().narrowPhaseTests;
    };
    return runCase(config, body, pairs, "narrowPhaseTests");
}

} // namespace

int main(int argc, char** argv) {
    Options opts = Options::parse(argc, argv);
    BenchmarkReport report;

    for (int n : {512, 2048, 8192}) report.add(runBroadphase(opts, n));
    for (int n : {1024, 4096, 16384}) report.add(runStaticField(opts, n));

    return finish(opts, report);
}
