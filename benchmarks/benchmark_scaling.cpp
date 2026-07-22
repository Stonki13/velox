// Scaling benchmark.
//
// Measures whole-step wall time (StepStats::totalMs) as the body count grows,
// producing the data behind the scaling curves in docs/performance.md. The
// secondary metric is the per-body cost (totalMs / bodyCount) so the report
// makes sub-linear vs linear vs super-linear scaling obvious at a glance.
//
// A single dense sphere-rain scene is rebuilt at each size; the snapshot
// restore keeps every measured step starting from the same configuration.

#include "benchmark_cli.h"
#include "benchmark_scenes.h"

#include <velox/velox.h>

#include <string>

using namespace veloxbench;

namespace {

BenchResult runScaling(const Options& opts, int bodyCount) {
    velox::World world(velox::BackendType::Cpu);
    scenes::sphereRain(world, bodyCount);
    velox::WorldSnapshot baseline = world.saveSnapshot();

    BenchConfig config;
    config.name = "scaling/total_step_" + std::to_string(bodyCount);
    config.category = "scaling";
    config.bodyCount = bodyCount;
    config = opts.apply(config);

    auto body = [&]() -> double {
        world.restoreSnapshot(baseline);
        world.step(1.0f / 60.0f);
        return world.lastStepStats().totalMs;
    };
    // Per-body cost: highlights how marginal cost changes with scale.
    auto perBody = [&]() -> double {
        return world.lastStepStats().totalMs / (double)bodyCount;
    };
    return runCase(config, body, perBody, "msPerBody");
}

} // namespace

int main(int argc, char** argv) {
    Options opts = Options::parse(argc, argv);
    BenchmarkReport report;

    // Geometric body-count sweep: enough points to fit a scaling curve.
    for (int n : {128, 256, 512, 1024, 2048, 4096, 8192})
        report.add(runScaling(opts, n));

    return finish(opts, report);
}
