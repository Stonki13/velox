#include "velox/benchmark.h"
#include <algorithm>
#include <numeric>

namespace velox {

BenchmarkResult runBenchmark(const BenchmarkConfig& config) {
    World world(BackendType::Auto);
    world.gravity = config.gravity;
    world.substeps = config.substeps;
    if (config.setup) config.setup(world);

    for (int i = 0; i < config.warmupFrames; ++i)
        world.step(1.0f / 60.0f);

    std::vector<double> frameTimes;
    frameTimes.reserve(config.measureFrames);
    size_t totalContacts = 0;
    bool anyDevice = false;
    for (int i = 0; i < config.measureFrames; ++i) {
        world.step(1.0f / 60.0f);
        auto stats = world.lastStepStats();
        frameTimes.push_back(stats.totalMs);
        totalContacts += stats.generatedContacts;
        if (stats.deviceSubsteps) anyDevice = true;
    }

    std::sort(frameTimes.begin(), frameTimes.end());
    BenchmarkResult result;
    result.name = config.name;
    result.meanMs = std::accumulate(frameTimes.begin(), frameTimes.end(), 0.0) /
                    frameTimes.size();
    result.minMs = frameTimes.front();
    result.maxMs = frameTimes.back();
    size_t p95Idx = std::min(frameTimes.size() - 1,
                             size_t(frameTimes.size() * 0.95));
    result.p95Ms = frameTimes[p95Idx];
    result.bodyCount = static_cast<int>(world.bodyCount());
    result.contactCount = static_cast<int>(totalContacts / size_t(config.measureFrames));
    result.deviceSubsteps = anyDevice;
    return result;
}

std::vector<BenchmarkResult> runBenchmarkSuite(
    const std::vector<BenchmarkConfig>& configs) {
    std::vector<BenchmarkResult> results;
    results.reserve(configs.size());
    for (const auto& config : configs)
        results.push_back(runBenchmark(config));
    return results;
}

} // namespace velox
