// Scene-query benchmark.
//
// Queries (raycasts, overlaps, shape casts) run outside step(), so they are
// timed directly rather than read from StepStats. Each measured iteration
// issues a fixed batch of queries against a static field of boxes and reports
// the batch time; the secondary metric is the batch size so results can be
// normalised to a per-query cost.
//
// The field is rebuilt once and reused read-only across iterations - queries
// never mutate world state, so no snapshot restore is needed between runs.

#include "benchmark_cli.h"
#include "benchmark_scenes.h"

#include <velox/velox.h>

#include <string>
#include <vector>

using namespace veloxbench;

namespace {

constexpr int kBatch = 1024;

/// Deterministic pseudo-random unit in [0, 1) so query origins/directions are
/// reproducible across runs without pulling in a RNG dependency.
float rand01(uint32_t& state) {
    state = state * 1664525u + 1013904223u;
    return (float)(state >> 8) / (float)(1u << 24);
}

BenchResult runRaycast(const Options& opts, int bodyCount) {
    velox::World world(velox::BackendType::Cpu);
    scenes::staticField(world, bodyCount);

    BenchConfig config;
    config.name = "queries/raycast_" + std::to_string(bodyCount);
    config.category = "queries";
    config.bodyCount = bodyCount;
    config = opts.apply(config);

    auto body = [&]() -> double {
        uint32_t state = 12345u;
        return timeMs([&]() {
            for (int i = 0; i < kBatch; ++i) {
                velox::Vec3 origin{(rand01(state) - 0.5f) * 60.0f,
                                   5.0f + rand01(state) * 5.0f,
                                   (rand01(state) - 0.5f) * 60.0f};
                velox::Vec3 dir{(rand01(state) - 0.5f), -1.0f,
                                (rand01(state) - 0.5f)};
                (void)world.rayCast(origin, dir, 100.0f);
            }
        });
    };
    auto batch = [&]() -> double { return (double)kBatch; };
    return runCase(config, body, batch, "queriesPerBatch");
}

BenchResult runOverlap(const Options& opts, int bodyCount) {
    velox::World world(velox::BackendType::Cpu);
    scenes::staticField(world, bodyCount);
    std::vector<velox::BodyId> hits;

    BenchConfig config;
    config.name = "queries/overlap_sphere_" + std::to_string(bodyCount);
    config.category = "queries";
    config.bodyCount = bodyCount;
    config = opts.apply(config);

    auto body = [&]() -> double {
        uint32_t state = 98765u;
        return timeMs([&]() {
            for (int i = 0; i < kBatch; ++i) {
                velox::Vec3 center{(rand01(state) - 0.5f) * 60.0f, 0.0f,
                                   (rand01(state) - 0.5f) * 60.0f};
                hits.clear();
                world.overlapSphere(center, 3.0f, hits);
            }
        });
    };
    auto batch = [&]() -> double { return (double)kBatch; };
    return runCase(config, body, batch, "queriesPerBatch");
}

BenchResult runSphereCast(const Options& opts, int bodyCount) {
    velox::World world(velox::BackendType::Cpu);
    scenes::staticField(world, bodyCount);

    BenchConfig config;
    config.name = "queries/sphere_cast_" + std::to_string(bodyCount);
    config.category = "queries";
    config.bodyCount = bodyCount;
    config = opts.apply(config);

    auto body = [&]() -> double {
        uint32_t state = 55555u;
        return timeMs([&]() {
            for (int i = 0; i < kBatch; ++i) {
                velox::Vec3 center{(rand01(state) - 0.5f) * 60.0f, 1.0f,
                                   (rand01(state) - 0.5f) * 60.0f};
                velox::Vec3 dir{(rand01(state) - 0.5f), 0.0f,
                                (rand01(state) - 0.5f)};
                (void)world.sphereCast(center, 0.5f, dir, 50.0f);
            }
        });
    };
    auto batch = [&]() -> double { return (double)kBatch; };
    return runCase(config, body, batch, "queriesPerBatch");
}

} // namespace

int main(int argc, char** argv) {
    Options opts = Options::parse(argc, argv);
    BenchmarkReport report;

    for (int n : {1024, 4096, 16384}) {
        report.add(runRaycast(opts, n));
        report.add(runOverlap(opts, n));
        report.add(runSphereCast(opts, n));
    }

    return finish(opts, report);
}
