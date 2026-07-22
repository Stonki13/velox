#pragma once
// Shared command-line handling and output dispatch for the benchmark suite.
//
// Every benchmark_*.cpp program parses its argv through Options::parse so the
// runner (scripts/run_benchmarks.py) can drive them uniformly:
//
//   benchmark_x [--json] [--csv] [--warmup N] [--iters N]
//               [--baseline file.json] [--threshold 1.10] [--quiet]
//
// Output precedence: --json and --csv write machine-readable text to stdout
// (and nothing else, so the stream is parseable). With neither flag a
// human-readable table is printed. When --baseline is supplied, detected
// regressions are reported to stderr and the process exits non-zero.

#include "benchmark_framework.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>

namespace veloxbench {

struct Options {
    bool json = false;
    bool csv = false;
    bool quiet = false;
    int warmup = -1;   ///< -1 = use each case's default.
    int iters = -1;    ///< -1 = use each case's default.
    std::string baselinePath;
    double threshold = 1.10;

    static Options parse(int argc, char** argv) {
        Options o;
        for (int i = 1; i < argc; ++i) {
            std::string a = argv[i];
            if (a == "--json") o.json = true;
            else if (a == "--csv") o.csv = true;
            else if (a == "--quiet") o.quiet = true;
            else if (a == "--warmup" && i + 1 < argc)
                o.warmup = std::atoi(argv[++i]);
            else if (a == "--iters" && i + 1 < argc)
                o.iters = std::atoi(argv[++i]);
            else if (a == "--baseline" && i + 1 < argc)
                o.baselinePath = argv[++i];
            else if (a == "--threshold" && i + 1 < argc)
                o.threshold = std::atof(argv[++i]);
        }
        return o;
    }

    /// Apply the global warmup/iteration overrides to a case config.
    BenchConfig apply(BenchConfig config) const {
        if (warmup >= 0) config.warmupIterations = warmup;
        if (iters >= 0) config.measureIterations = iters;
        return config;
    }
};

/// Read a whole file into a string. Returns an empty string when the file
/// cannot be opened.
inline std::string readFile(const std::string& path) {
    std::ifstream in(path);
    if (!in) return {};
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

/// Emit the report according to the parsed options and run the baseline
/// regression check when a baseline path was given. Returns the process exit
/// code (0 = ok, 1 = regressions detected).
inline int finish(const Options& opts, const BenchmarkReport& report) {
    if (opts.json) {
        std::printf("%s\n", report.toJson().c_str());
    } else if (opts.csv) {
        std::printf("%s", report.toCsv().c_str());
    } else {
        report.printTable();
    }

    if (!opts.baselinePath.empty()) {
        std::string baselineJson = readFile(opts.baselinePath);
        if (baselineJson.empty()) {
            std::fprintf(stderr, "warning: could not read baseline '%s'\n",
                         opts.baselinePath.c_str());
            return 0;
        }
        auto baseline = parseBaseline(baselineJson);
        auto regressions =
            detectRegressions(report.results(), baseline, opts.threshold);
        if (!regressions.empty()) {
            std::fprintf(stderr, "PERFORMANCE REGRESSIONS DETECTED:\n");
            for (const auto& r : regressions) {
                std::fprintf(stderr,
                             "  %s: %.3fms -> %.3fms (%.2fx)\n", r.name.c_str(),
                             r.baselineMs, r.currentMs, r.ratio);
            }
            return 1;
        }
        if (!opts.quiet && !opts.json && !opts.csv)
            std::fprintf(stderr, "No performance regressions detected.\n");
    }
    return 0;
}

} // namespace veloxbench
