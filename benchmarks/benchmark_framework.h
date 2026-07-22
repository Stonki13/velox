#pragma once
// Velox benchmark framework (header-only).
//
// A small, dependency-free harness shared by every benchmark_*.cpp program in
// this directory. It provides:
//
//   * Statistical analysis  - mean, median, standard deviation, min/max and
//                             arbitrary percentiles (p50/p95/p99 by default).
//   * Warmup + iteration    - a configurable number of discarded warmup runs
//     control                  followed by timed, measured iterations.
//   * Memory tracking       - peak working-set sampling across a run, with
//                             platform backends for Windows and POSIX.
//   * Baseline comparison   - load a previously saved JSON baseline and flag
//                             metrics that regressed beyond a threshold.
//   * Reporting             - emit results as JSON (machine readable, matches
//                             the schema scripts/run_benchmarks.py expects) or
//                             as a human-readable table / CSV.
//
// The framework deliberately does not depend on Velox; each benchmark supplies
// its own scene setup and a callable that performs one measured iteration and
// returns a duration in milliseconds. This keeps the statistics portable and
// lets the same harness time whole steps, individual phases, or scene queries.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

#if defined(_WIN32)
#define VELOX_BENCH_WINDOWS 1
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
// Keep windows.h from defining min/max macros, which would break std::max and
// numeric_limits<>::max() used inside the Velox headers.
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <psapi.h>
#else
#define VELOX_BENCH_POSIX 1
#include <sys/resource.h>
#include <sys/time.h>
#endif

namespace veloxbench {

using Clock = std::chrono::high_resolution_clock;

// ---------------------------------------------------------------------------
// Statistics
// ---------------------------------------------------------------------------

/// Descriptive statistics over a sample of millisecond measurements.
struct Stats {
    int count = 0;
    double mean = 0.0;
    double median = 0.0;
    double stddev = 0.0;
    double min = 0.0;
    double max = 0.0;
    double p50 = 0.0;
    double p95 = 0.0;
    double p99 = 0.0;

    /// Compute statistics from raw samples. The input is copied and sorted;
    /// an empty input yields an all-zero result with count == 0.
    static Stats compute(std::vector<double> samples) {
        Stats s;
        s.count = (int)samples.size();
        if (samples.empty()) return s;
        std::sort(samples.begin(), samples.end());
        s.min = samples.front();
        s.max = samples.back();
        double sum = 0.0;
        for (double v : samples) sum += v;
        s.mean = sum / samples.size();
        s.median = percentileSorted(samples, 0.50);
        s.p50 = s.median;
        s.p95 = percentileSorted(samples, 0.95);
        s.p99 = percentileSorted(samples, 0.99);
        double acc = 0.0;
        for (double v : samples) {
            double d = v - s.mean;
            acc += d * d;
        }
        // Population stddev is fine for benchmarking; we are summarising the
        // measured population, not estimating a wider distribution.
        s.stddev = std::sqrt(acc / samples.size());
        return s;
    }

    /// Linear-interpolated percentile over an already-sorted range.
    /// `p` is in [0, 1]. Exposed so callers can request custom percentiles.
    static double percentileSorted(const std::vector<double>& sorted, double p) {
        if (sorted.empty()) return 0.0;
        if (sorted.size() == 1) return sorted.front();
        double rank = p * (double)(sorted.size() - 1);
        size_t lo = (size_t)std::floor(rank);
        size_t hi = (size_t)std::ceil(rank);
        if (lo == hi) return sorted[lo];
        double frac = rank - (double)lo;
        return sorted[lo] * (1.0 - frac) + sorted[hi] * frac;
    }

    /// Coefficient of variation (stddev / mean). A low value indicates a
    /// stable measurement; values above ~0.2 suggest a noisy benchmark.
    double coefficientOfVariation() const {
        return mean > 0.0 ? stddev / mean : 0.0;
    }
};

// ---------------------------------------------------------------------------
// Memory tracking
// ---------------------------------------------------------------------------

/// Peak resident/working-set memory observed during a benchmark, in bytes.
///
/// On Windows this reads the process peak working set; on POSIX it reads the
/// peak resident set size from getrusage (kilobytes on Linux, bytes on macOS,
/// normalised to bytes here). Values are best-effort: when the platform call
/// fails the field is left at zero.
struct MemoryUsage {
    uint64_t peakBytes = 0;      ///< Peak working/resident set size.
    uint64_t currentBytes = 0;   ///< Current working/resident set size.

    static MemoryUsage sample() {
        MemoryUsage m;
#if defined(VELOX_BENCH_WINDOWS)
        PROCESS_MEMORY_COUNTERS_EX counters{};
        if (GetProcessMemoryInfo(GetCurrentProcess(),
                                 reinterpret_cast<PROCESS_MEMORY_COUNTERS*>(&counters),
                                 sizeof(counters))) {
            m.peakBytes = counters.PeakWorkingSetSize;
            m.currentBytes = counters.WorkingSetSize;
        }
#else
        struct rusage usage {};
        if (getrusage(RUSAGE_SELF, &usage) == 0) {
            uint64_t peak = (uint64_t)usage.ru_maxrss;
#if defined(__APPLE__)
            // macOS reports bytes; Linux reports kilobytes.
            m.peakBytes = peak;
#else
            m.peakBytes = peak * 1024ull;
#endif
            m.currentBytes = m.peakBytes;
        }
#endif
        return m;
    }

    double peakMiB() const { return (double)peakBytes / (1024.0 * 1024.0); }
};

// ---------------------------------------------------------------------------
// Benchmark case
// ---------------------------------------------------------------------------

/// Configuration for one benchmark case.
struct BenchConfig {
    std::string name;        ///< Stable identifier used as the JSON key.
    int warmupIterations = 5;   ///< Discarded runs that prime caches/JIT.
    int measureIterations = 30; ///< Timed runs that feed the statistics.
    int bodyCount = 0;          ///< Optional context: bodies in the scene.
    std::string category;       ///< Optional grouping tag (e.g. "broadphase").
};

/// The outcome of running one benchmark case.
struct BenchResult {
    BenchConfig config;
    Stats stats;                 ///< Timing statistics in milliseconds.
    MemoryUsage memory;          ///< Peak memory observed during the run.
    double extraMetric = 0.0;    ///< Optional secondary metric (e.g. contacts).
    std::string extraLabel;      ///< Name for extraMetric in reports.

    /// Serialise to a single JSON object. The field names match the schema
    /// consumed by scripts/run_benchmarks.py and the existing
    /// scripts/benchmark_regression.py baseline format.
    std::string toJson() const {
        char buf[1024];
        std::snprintf(buf, sizeof(buf),
                      "    {\n"
                      "      \"name\": \"%s\",\n"
                      "      \"category\": \"%s\",\n"
                      "      \"bodyCount\": %d,\n"
                      "      \"iterations\": %d,\n"
                      "      \"meanMs\": %.6f,\n"
                      "      \"medianMs\": %.6f,\n"
                      "      \"stddevMs\": %.6f,\n"
                      "      \"minMs\": %.6f,\n"
                      "      \"maxMs\": %.6f,\n"
                      "      \"p50Ms\": %.6f,\n"
                      "      \"p95Ms\": %.6f,\n"
                      "      \"p99Ms\": %.6f,\n"
                      "      \"cv\": %.6f,\n"
                      "      \"peakMemoryMiB\": %.3f,\n"
                      "      \"extraLabel\": \"%s\",\n"
                      "      \"extraMetric\": %.6f\n"
                      "    }",
                      config.name.c_str(), config.category.c_str(),
                      config.bodyCount, stats.count, stats.mean, stats.median,
                      stats.stddev, stats.min, stats.max, stats.p50, stats.p95,
                      stats.p99, stats.coefficientOfVariation(),
                      memory.peakMiB(), extraLabel.c_str(), extraMetric);
        return std::string(buf);
    }
};

/// Run one benchmark case.
///
/// `body` performs a single measured iteration and returns its duration in
/// milliseconds. It is invoked `warmupIterations` times (results discarded)
/// and then `measureIterations` times (results collected). Memory is sampled
/// after the measured runs so the peak reflects the whole case.
///
/// The optional `extraMetric` producer is called once after timing completes
/// (e.g. to report the contact count from the last step); leave it empty to
/// skip. A default-constructed std::function is empty and tests false.
inline BenchResult runCase(
    const BenchConfig& config,
    const std::function<double()>& body,
    const std::function<double()>& extraMetric = {},
    const char* extraLabel = "") {
    BenchResult result;
    result.config = config;

    for (int i = 0; i < config.warmupIterations; ++i) (void)body();

    std::vector<double> samples;
    samples.reserve(config.measureIterations);
    for (int i = 0; i < config.measureIterations; ++i) samples.push_back(body());

    result.stats = Stats::compute(std::move(samples));
    result.memory = MemoryUsage::sample();
    if (extraMetric) {
        result.extraMetric = extraMetric();
        result.extraLabel = extraLabel;
    }
    return result;
}

/// Time a callable in milliseconds using the high-resolution clock.
template <typename Fn>
inline double timeMs(Fn&& fn) {
    auto t0 = Clock::now();
    fn();
    auto t1 = Clock::now();
    return std::chrono::duration<double, std::milli>(t1 - t0).count();
}

// ---------------------------------------------------------------------------
// Report
// ---------------------------------------------------------------------------

/// Collects benchmark results and renders them as JSON, CSV, or a table.
class BenchmarkReport {
public:
    void add(BenchResult result) { results_.push_back(std::move(result)); }

    const std::vector<BenchResult>& results() const { return results_; }

    /// Emit a JSON array of result objects. Matches the schema expected by
    /// scripts/run_benchmarks.py and scripts/benchmark_regression.py.
    std::string toJson() const {
        std::string out = "[\n";
        for (size_t i = 0; i < results_.size(); ++i) {
            out += results_[i].toJson();
            out += (i + 1 < results_.size()) ? ",\n" : "\n";
        }
        out += "]";
        return out;
    }

    /// Emit a CSV with a header row, one line per result.
    std::string toCsv() const {
        std::string out =
            "name,category,bodyCount,iterations,meanMs,medianMs,stddevMs,"
            "minMs,maxMs,p50Ms,p95Ms,p99Ms,cv,peakMemoryMiB,extraLabel,"
            "extraMetric\n";
        char buf[1024];
        for (const auto& r : results_) {
            std::snprintf(buf, sizeof(buf),
                          "%s,%s,%d,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
                          "%.6f,%.6f,%.3f,%s,%.6f\n",
                          r.config.name.c_str(), r.config.category.c_str(),
                          r.config.bodyCount, r.stats.count, r.stats.mean,
                          r.stats.median, r.stats.stddev, r.stats.min,
                          r.stats.max, r.stats.p50, r.stats.p95, r.stats.p99,
                          r.stats.coefficientOfVariation(), r.memory.peakMiB(),
                          r.extraLabel.c_str(), r.extraMetric);
            out += buf;
        }
        return out;
    }

    /// Print a human-readable table to stdout.
    void printTable() const {
        std::printf("%-32s %8s %8s %8s %8s %8s %10s\n", "benchmark", "mean",
                    "median", "p95", "stddev", "cv", "peakMiB");
        std::printf(
            "--------------------------------------------------------------"
            "----------------\n");
        for (const auto& r : results_) {
            std::printf("%-32s %8.3f %8.3f %8.3f %8.3f %8.3f %10.2f\n",
                        r.config.name.c_str(), r.stats.mean, r.stats.median,
                        r.stats.p95, r.stats.stddev,
                        r.stats.coefficientOfVariation(), r.memory.peakMiB());
        }
    }

private:
    std::vector<BenchResult> results_;
};

// ---------------------------------------------------------------------------
// Baseline comparison
// ---------------------------------------------------------------------------

/// One baseline entry: the recorded mean for a named benchmark.
struct BaselineEntry {
    std::string name;
    double meanMs = 0.0;
};

/// A single detected regression.
struct Regression {
    std::string name;
    double baselineMs = 0.0;
    double currentMs = 0.0;
    double ratio = 0.0; ///< current / baseline; > 1.0 means slower.
};

/// Parse the minimal subset of the JSON baseline format we need: an array of
/// objects each carrying "name" and "meanMs". This is intentionally a small
/// hand-rolled scanner so the framework stays dependency-free; it tolerates
/// whitespace and field ordering but expects well-formed numeric values.
inline std::vector<BaselineEntry> parseBaseline(const std::string& json) {
    std::vector<BaselineEntry> entries;
    size_t pos = 0;
    auto findKey = [&](const char* key, size_t from) -> size_t {
        std::string needle = std::string("\"") + key + "\"";
        return json.find(needle, from);
    };
    while (true) {
        size_t nameKey = findKey("name", pos);
        if (nameKey == std::string::npos) break;
        size_t colon = json.find(':', nameKey);
        size_t q1 = json.find('"', colon + 1);
        size_t q2 = json.find('"', q1 + 1);
        if (q1 == std::string::npos || q2 == std::string::npos) break;
        std::string name = json.substr(q1 + 1, q2 - q1 - 1);

        size_t meanKey = findKey("meanMs", q2);
        double mean = 0.0;
        if (meanKey != std::string::npos) {
            size_t mcolon = json.find(':', meanKey);
            mean = std::strtod(json.c_str() + mcolon + 1, nullptr);
        }
        entries.push_back({name, mean});
        pos = q2 + 1;
    }
    return entries;
}

/// Compare results against a baseline. Any benchmark whose mean exceeds
/// `baseline * threshold` is reported as a regression. Benchmarks absent from
/// the baseline are ignored (they are new, not regressed).
inline std::vector<Regression> detectRegressions(
    const std::vector<BenchResult>& results,
    const std::vector<BaselineEntry>& baseline, double threshold = 1.10) {
    std::vector<Regression> regressions;
    for (const auto& r : results) {
        for (const auto& b : baseline) {
            if (b.name != r.config.name) continue;
            double ratio = b.meanMs > 0.0 ? r.stats.mean / b.meanMs : 0.0;
            if (ratio > threshold)
                regressions.push_back({r.config.name, b.meanMs, r.stats.mean,
                                       ratio});
            break;
        }
    }
    return regressions;
}

} // namespace veloxbench
