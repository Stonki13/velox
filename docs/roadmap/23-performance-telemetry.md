# 23 — Performance Telemetry

## Goal

Add memory usage and PCIe transfer counters to `StepStats`, implement a benchmark regression tracking system in CI, and provide per-subsystem timing breakdowns (broad phase, narrow phase, solver, CCD) that help identify performance bottlenecks as the engine evolves.

## Public API

```cpp
namespace velox {

struct StepStats {
    float dt = 0.0f;
    size_t bodyCount = 0;
    size_t awakeDynamicBodies = 0;
    size_t generatedContacts = 0;
    size_t solvedContacts = 0;
    size_t jointCount = 0;

    // Existing timing fields:
    double setupMs = 0.0;
    double collisionDetectionMs = 0.0;
    double solverMs = 0.0;
    double ccdMs = 0.0;
    double finalizeMs = 0.0;
    double totalMs = 0.0;

    // New telemetry fields:
    size_t broadPhaseProxies = 0;         // number of AABB tree proxies
    size_t narrowPhaseTests = 0;          // GJK/EPA tests performed
    size_t solverIterations = 0;          // total velocity solve iterations
    size_t islandCount = 0;               // independent solving islands
    uint64_t deviceMemoryUsed = 0;        // bytes allocated on GPU
    uint64_t hostToDeviceTransfers = 0;   // number of H2D transfers this step
    uint64_t deviceToHostTransfers = 0;   // number of D2H transfers this step
    uint64_t transferBytesTotal = 0;      // total bytes moved over PCIe

    // Per-subsystem timing (more granular than existing fields):
    double broadPhaseMs = 0.0;
    double narrowPhaseMs = 0.0;
    double contactSolverMs = 0.0;
    double jointSolverMs = 0.0;
    double ccdRecoveryMs = 0.0;
};

// Benchmark configuration: a named scene to run for regression tracking.
struct BenchmarkConfig {
    std::string name;
    int frames = 600;                     // 10 seconds at 60 Hz
    float dt = 1.0f / 60.0f;
    int substeps = 4;
};

// Benchmark result: aggregate stats over all frames.
struct BenchmarkResult {
    double meanTotalMs = 0.0;
    double p95TotalMs = 0.0;              // 95th percentile step time
    double maxTotalMs = 0.0;
    double meanSolverMs = 0.0;
    double meanCCDMs = 0.0;
    uint64_t totalTransferBytes = 0;
};

// Run a benchmark scene and return aggregate statistics.
BenchmarkResult runBenchmark(const World& world, const BenchmarkConfig& config);

} // namespace velox
```

## Data structures

- New fields on `StepStats` — added to `include/velox/world.h`.
- `BenchmarkConfig`, `BenchmarkResult` — new file `include/velox/benchmark.h`.
- `runBenchmark()` — new function in `src/benchmark.cpp`.

## Algorithm

**Telemetry collection:**

1. **Broad phase proxies:** Count the number of active proxies in the incremental AABB tree (accessible via `broadPhase_->proxyCount()`).
2. **Narrow phase tests:** Increment a counter each time `gjkDistance()` or `epaPenetration()` is called. Reset at step start.
3. **Solver iterations:** Sum the iteration counts across all contact islands. The existing `kVelocityIterations` constant gives the per-substep count; multiply by substeps and island count.
4. **Device memory:** Query `cudaMemGetInfo()` before and after each step; the difference is the net allocation change. Accumulate total allocated (not just delta) for reporting.
5. **Transfer counts:** Increment counters in the CUDA backend's `uploadToGPU()` and `downloadFromGPU()` wrappers.

**Benchmark regression tracking:**

1. Define a set of canonical benchmark scenes (e.g., 512-sphere rain, 2048-sphere rain, 20k-triangle terrain with 2048 bodies).
2. Run each scene for `frames` steps, recording `StepStats::totalMs` per frame.
3. Compute mean, p95, and max step times. Also track solver/CCD breakdown.
4. Store results in a JSON file (`benchmarks/results.json`) with git commit hash and hardware specs.
5. In CI: compare current p95 against the baseline (stored in repo). If p95 increases by > 10%, fail the build with a regression report.

## Files

- `include/velox/world.h` — add telemetry fields to StepStats
- `include/velox/benchmark.h` — new header with BenchmarkConfig, BenchmarkResult
- `src/benchmark.cpp` — benchmark execution and statistics
- `benchmarks/scenes.json` — canonical benchmark scene definitions (new file)
- `benchmarks/results_baseline.json` — baseline results for regression comparison

## Tests

1. **Telemetry accuracy:** Run a scene with 100 bodies; verify `StepStats::narrowPhaseTests` matches the expected GJK call count (within 5% tolerance for parallel execution variance).
2. **Benchmark reproducibility:** Run the same benchmark scene 10 times on the same hardware. Coefficient of variation (stddev/mean) for p95 step time must be < 5%.
3. **Regression detection:** Intentionally slow down the solver by a factor of 2; verify that CI flags the regression (p95 increase > 10%).

## Acceptance

- [ ] `StepStats` includes broad phase proxy count, narrow phase test count, solver iterations, island count
- [ ] Device memory usage and transfer counts are reported in StepStats
- [ ] Benchmark system runs canonical scenes and reports mean/p95/max step times
- [ ] CI regression check fails when p95 increases by > 10% compared to baseline

## Size: M

## Risks

- Telemetry counters add overhead to the hot path (counter increments, memory queries). Must ensure the overhead is < 1% of total step time; benchmark with counters enabled vs disabled.
- Baseline results are hardware-dependent. The CI regression threshold must be adjusted per runner; document the reference hardware specs for each baseline.
- `cudaMemGetInfo()` can be expensive if called frequently. Batch the query to once per step rather than per allocation.
