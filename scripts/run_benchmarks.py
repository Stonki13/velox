#!/usr/bin/env python3
"""Run the Velox benchmark suite and produce performance reports.

This driver discovers the benchmark executables produced by the
``benchmarks/`` CMake target, runs each one with ``--json``, aggregates the
results, and can:

  * print a human-readable performance report (text or Markdown),
  * export the combined results to JSON and/or CSV,
  * detect regressions against a saved baseline,
  * (re)write the baseline from the current run.

The benchmark executables share a common CLI (see benchmarks/benchmark_cli.h):
``--json`` makes each one emit a JSON array of result objects on stdout, which
this script parses and merges.

Examples
--------
Run everything and print a report::

    python scripts/run_benchmarks.py --build-dir build

Export combined results and check against a baseline::

    python scripts/run_benchmarks.py --build-dir build \
        --json-out results.json --csv-out results.csv \
        --baseline benchmark_baseline.json --threshold 1.10

Record a fresh baseline::

    python scripts/run_benchmarks.py --build-dir build \
        --update-baseline benchmark_baseline.json
"""

import argparse
import csv
import json
import os
import platform
import subprocess
import sys

# Benchmark executables produced by benchmarks/CMakeLists.txt.
BENCHMARKS = [
    "benchmark_broadphase",
    "benchmark_narrowphase",
    "benchmark_solver",
    "benchmark_queries",
    "benchmark_scaling",
]

# Multi-config generators (Visual Studio, Xcode) nest binaries under a
# configuration folder; single-config generators (Ninja, Makefiles) do not.
CONFIG_SUBDIRS = ["Release", "RelWithDebInfo", "MinSizeRel", "Debug", ""]


def find_executable(build_dir, name):
    """Locate a benchmark executable under the build tree.

    Searches ``<build>/benchmarks[/<config>]/<name>[.exe]`` for the common
    CMake configurations. Returns the first match or None.
    """
    exe = name + (".exe" if platform.system() == "Windows" else "")
    candidates = [
        os.path.join(build_dir, "benchmarks", sub, exe) for sub in CONFIG_SUBDIRS
    ]
    # Also tolerate a flat layout where everything lands in the build root.
    candidates += [
        os.path.join(build_dir, sub, exe) for sub in CONFIG_SUBDIRS
    ]
    for path in candidates:
        if os.path.isfile(path):
            return path
    return None


def run_one(exe, warmup, iters, timeout):
    """Run a single benchmark executable and parse its JSON output."""
    cmd = [exe, "--json"]
    if warmup is not None:
        cmd += ["--warmup", str(warmup)]
    if iters is not None:
        cmd += ["--iters", str(iters)]
    try:
        proc = subprocess.run(
            cmd, capture_output=True, text=True, timeout=timeout
        )
    except subprocess.TimeoutExpired:
        print(f"  TIMEOUT after {timeout}s: {exe}", file=sys.stderr)
        return []
    if proc.returncode != 0:
        print(f"  FAILED ({proc.returncode}): {exe}", file=sys.stderr)
        if proc.stderr:
            print(proc.stderr, file=sys.stderr)
        return []
    try:
        return json.loads(proc.stdout)
    except json.JSONDecodeError as exc:
        print(f"  BAD JSON from {exe}: {exc}", file=sys.stderr)
        return []


def collect(build_dir, warmup, iters, timeout):
    """Run every discoverable benchmark and merge their results."""
    results = []
    found = 0
    for name in BENCHMARKS:
        exe = find_executable(build_dir, name)
        if exe is None:
            print(f"  not built, skipping: {name}", file=sys.stderr)
            continue
        found += 1
        print(f"  running {name} ...", file=sys.stderr)
        results.extend(run_one(exe, warmup, iters, timeout))
    if found == 0:
        print(
            f"No benchmark executables found under '{build_dir}'. "
            "Build them first (cmake --build <build> --target "
            "benchmark_broadphase ...) or pass --build-dir.",
            file=sys.stderr,
        )
    return results


def detect_regressions(results, baseline, threshold):
    """Return a list of regressions where mean exceeds baseline * threshold."""
    baseline_by_name = {entry["name"]: entry for entry in baseline}
    regressions = []
    for result in results:
        base = baseline_by_name.get(result["name"])
        if base is None:
            continue
        base_ms = base.get("meanMs", 0.0)
        cur_ms = result.get("meanMs", 0.0)
        if base_ms > 0 and cur_ms > base_ms * threshold:
            regressions.append(
                {
                    "name": result["name"],
                    "baseline_ms": base_ms,
                    "current_ms": cur_ms,
                    "ratio": cur_ms / base_ms,
                }
            )
    return regressions


def write_json(results, path):
    with open(path, "w") as f:
        json.dump(results, f, indent=2)
    print(f"Wrote JSON: {path}", file=sys.stderr)


def write_csv(results, path):
    if not results:
        return
    # Union of keys across results, with a stable preferred ordering first.
    preferred = [
        "name", "category", "bodyCount", "iterations", "meanMs", "medianMs",
        "stddevMs", "minMs", "maxMs", "p50Ms", "p95Ms", "p99Ms", "cv",
        "peakMemoryMiB", "extraLabel", "extraMetric",
        "gitCommit", "cpuName", "gpuName", "hostname",
    ]
    keys = [k for k in preferred if any(k in r for r in results)]
    keys += sorted({k for r in results for k in r} - set(keys))
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        for r in results:
            writer.writerow(r)
    print(f"Wrote CSV: {path}", file=sys.stderr)


def print_report(results, regressions):
    """Print a human-readable performance report grouped by category."""
    if not results:
        print("No benchmark results collected.")
        return

    by_category = {}
    for r in results:
        by_category.setdefault(r.get("category", "misc"), []).append(r)

    header = (
        f"{'benchmark':40s} {'mean':>9s} {'median':>9s} {'p95':>9s} "
        f"{'stddev':>9s} {'cv':>6s} {'peakMiB':>8s}"
    )
    print("Velox performance report")
    print("=" * len(header))
    for category in sorted(by_category):
        print(f"\n[{category}]")
        print(header)
        print("-" * len(header))
        for r in by_category[category]:
            print(
                f"{r['name']:40s} {r.get('meanMs', 0):9.3f} "
                f"{r.get('medianMs', 0):9.3f} {r.get('p95Ms', 0):9.3f} "
                f"{r.get('stddevMs', 0):9.3f} {r.get('cv', 0):6.3f} "
                f"{r.get('peakMemoryMiB', 0):8.2f}"
            )

    if regressions:
        print("\nPERFORMANCE REGRESSIONS DETECTED:")
        for reg in regressions:
            print(
                f"  {reg['name']}: {reg['baseline_ms']:.3f}ms -> "
                f"{reg['current_ms']:.3f}ms ({reg['ratio']:.2f}x)"
            )
    else:
        print("\nNo performance regressions detected.")


def main():
    parser = argparse.ArgumentParser(
        description="Run the Velox benchmark suite and report results."
    )
    parser.add_argument(
        "--build-dir", default="build",
        help="CMake build directory containing the benchmark executables.",
    )
    parser.add_argument("--warmup", type=int, default=None,
                        help="Override warmup iterations for every benchmark.")
    parser.add_argument("--iters", type=int, default=None,
                        help="Override measured iterations for every benchmark.")
    parser.add_argument("--timeout", type=int, default=600,
                        help="Per-benchmark timeout in seconds.")
    parser.add_argument("--json-out", default=None,
                        help="Write combined results to this JSON file.")
    parser.add_argument("--csv-out", default=None,
                        help="Write combined results to this CSV file.")
    parser.add_argument("--baseline", default=None,
                        help="Baseline JSON to compare against for regressions.")
    parser.add_argument("--threshold", type=float, default=1.10,
                        help="Regression threshold (1.10 = 10%% slower fails).")
    parser.add_argument("--update-baseline", default=None, metavar="PATH",
                        help="Write the current results as the new baseline.")
    args = parser.parse_args()

    print("Collecting benchmark results ...", file=sys.stderr)
    results = collect(args.build_dir, args.warmup, args.iters, args.timeout)
    if not results:
        sys.exit(1)

    regressions = []
    if args.baseline:
        if not os.path.exists(args.baseline):
            print(f"No baseline at {args.baseline}; use --update-baseline to "
                  "create one.", file=sys.stderr)
        else:
            with open(args.baseline) as f:
                baseline = json.load(f)
            regressions = detect_regressions(results, baseline, args.threshold)

    print_report(results, regressions)

    if args.json_out:
        write_json(results, args.json_out)
    if args.csv_out:
        write_csv(results, args.csv_out)
    if args.update_baseline:
        write_json(results, args.update_baseline)

    if regressions:
        sys.exit(1)


if __name__ == "__main__":
    main()
