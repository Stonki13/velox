// Narrowphase benchmark.
//
// Measures contact-generation cost via StepStats::narrowPhaseMs and reports the
// number of contacts produced as a secondary metric. Two workloads are
// covered:
//
//   * sphere/sphere rain  - dense convex-vs-convex contact generation.
//   * spheres on a mesh   - BVH-backed convex-vs-triangle contact generation.
//
// Reading the engine's own narrow-phase timing keeps the measurement aligned
// with what the profiler reports and avoids re-implementing the dispatch.

#include "benchmark_cli.h"
#include "benchmark_scenes.h"

#include <velox/velox.h>

#include <string>

using namespace veloxbench;

namespace {

BenchResult runSphereContacts(const Options& opts, int bodyCount) {
    velox::World world(velox::BackendType::Cpu);
    scenes::sphereRain(world, bodyCount);
    velox::WorldSnapshot baseline = world.saveSnapshot();

    BenchConfig config;
    config.name = "narrowphase/sphere_contacts_" + std::to_string(bodyCount);
    config.category = "narrowphase";
    config.bodyCount = bodyCount;
    config = opts.apply(config);

    auto body = [&]() -> double {
        world.restoreSnapshot(baseline);
        world.step(1.0f / 60.0f);
        return world.lastStepStats().narrowPhaseMs;
    };
    auto contacts = [&]() -> double {
        return (double)world.lastStepStats().generatedContacts;
    };
    return runCase(config, body, contacts, "generatedContacts");
}

BenchResult runMeshContacts(const Options& opts, int bodyCount, int grid) {
    velox::World world(velox::BackendType::Cpu);
    scenes::meshTerrain(world, bodyCount, grid);
    velox::WorldSnapshot baseline = world.saveSnapshot();

    BenchConfig config;
    config.name = "narrowphase/mesh_terrain_" + std::to_string(bodyCount);
    config.category = "narrowphase";
    config.bodyCount = bodyCount;
    config = opts.apply(config);

    auto body = [&]() -> double {
        world.restoreSnapshot(baseline);
        world.step(1.0f / 60.0f);
        return world.lastStepStats().narrowPhaseMs;
    };
    auto tests = [&]() -> double {
        return (double)world.lastStepStats().narrowPhaseTests;
    };
    return runCase(config, body, tests, "narrowPhaseTests");
}

} // namespace

int main(int argc, char** argv) {
    Options opts = Options::parse(argc, argv);
    BenchmarkReport report;

    for (int n : {512, 2048, 8192}) report.add(runSphereContacts(opts, n));
    for (int n : {1024, 4096}) report.add(runMeshContacts(opts, n, 100));

    return finish(opts, report);
}
