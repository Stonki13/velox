// Constraint solver benchmark.
//
// Measures the two solver stages the engine times separately:
//
//   * contact solver - StepStats::contactSolverMs over a dense sphere stack.
//   * joint solver   - StepStats::jointSolverMs over many independent
//                      distance-joint islands.
//
// The contact case is also run across a sweep of velocity-iteration counts so
// the report shows how solver cost scales with iteration budget - the main
// knob users trade quality against.

#include "benchmark_cli.h"
#include "benchmark_scenes.h"

#include <velox/velox.h>

#include <string>

using namespace veloxbench;

namespace {

BenchResult runContactSolver(const Options& opts, int bodyCount,
                             int velocityIterations) {
    velox::World world(velox::BackendType::Cpu);
    scenes::sphereRain(world, bodyCount);

    velox::SolverOptions solver = world.solverOptions();
    solver.velocityIterations = velocityIterations;
    world.setSolverOptions(solver);

    velox::WorldSnapshot baseline = world.saveSnapshot();

    BenchConfig config;
    config.name = "solver/contact_" + std::to_string(bodyCount) + "_it" +
                  std::to_string(velocityIterations);
    config.category = "solver";
    config.bodyCount = bodyCount;
    config = opts.apply(config);

    auto body = [&]() -> double {
        world.restoreSnapshot(baseline);
        world.step(1.0f / 60.0f);
        return world.lastStepStats().contactSolverMs;
    };
    auto solved = [&]() -> double {
        return (double)world.lastStepStats().solvedContacts;
    };
    return runCase(config, body, solved, "solvedContacts");
}

BenchResult runJointSolver(const Options& opts, int jointCount) {
    velox::World world(velox::BackendType::Cpu);
    scenes::jointFan(world, jointCount);
    velox::WorldSnapshot baseline = world.saveSnapshot();

    BenchConfig config;
    config.name = "solver/joints_" + std::to_string(jointCount);
    config.category = "solver";
    config.bodyCount = jointCount;
    config = opts.apply(config);

    auto body = [&]() -> double {
        world.restoreSnapshot(baseline);
        world.step(1.0f / 60.0f);
        return world.lastStepStats().jointSolverMs;
    };
    auto islands = [&]() -> double {
        return (double)world.lastStepStats().islandCount;
    };
    return runCase(config, body, islands, "islandCount");
}

} // namespace

int main(int argc, char** argv) {
    Options opts = Options::parse(argc, argv);
    BenchmarkReport report;

    // Contact solver vs iteration budget at a fixed dense workload.
    for (int it : {2, 4, 8, 16}) report.add(runContactSolver(opts, 2048, it));
    // Contact solver vs problem size at the default iteration budget.
    for (int n : {512, 2048, 8192}) report.add(runContactSolver(opts, n, 4));
    // Joint solver throughput across island counts.
    for (int n : {256, 1024, 4096}) report.add(runJointSolver(opts, n));

    return finish(opts, report);
}
