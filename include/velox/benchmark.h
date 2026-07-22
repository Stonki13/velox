#pragma once
#include "world.h"
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace velox {

struct BenchmarkConfig {
    std::string name;
    int warmupFrames = 60;
    int measureFrames = 300;
    int substeps = 4;
    Vec3 gravity{0, -9.81f, 0};
    std::function<void(World&)> setup;
};

struct BenchmarkResult {
    std::string name;
    double meanMs = 0.0;
    double minMs = 0.0;
    double maxMs = 0.0;
    double p95Ms = 0.0;
    int bodyCount = 0;
    int contactCount = 0;
    bool deviceSubsteps = false;
};

BenchmarkResult runBenchmark(const BenchmarkConfig& config);

std::vector<BenchmarkResult> runBenchmarkSuite(
    const std::vector<BenchmarkConfig>& configs);

} // namespace velox
