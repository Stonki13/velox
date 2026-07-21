#!/usr/bin/env python3
"""Benchmark regression checker.

Runs the Velox benchmark suite and compares against a baseline JSON file.
Exits with code 1 if any metric exceeds the regression threshold.

Usage:
    python benchmark_regression.py [--baseline baseline.json] [--threshold 1.10]
"""

import argparse
import json
import os
import subprocess
import sys


def run_benchmark(benchmark_exe):
    """Run the benchmark executable and parse JSON output."""
    result = subprocess.run(
        [benchmark_exe, "--json"],
        capture_output=True,
        text=True,
        timeout=300,
    )
    if result.returncode != 0:
        print(f"Benchmark failed:\n{result.stderr}", file=sys.stderr)
        sys.exit(1)
    return json.loads(result.stdout)


def check_regression(current, baseline, threshold):
    """Compare current results against baseline. Returns list of regressions."""
    regressions = []
    baseline_by_name = {r["name"]: r for r in baseline}

    for result in current:
        name = result["name"]
        if name not in baseline_by_name:
            continue
        base = baseline_by_name[name]
        if result["meanMs"] > base["meanMs"] * threshold:
            regressions.append({
                "name": name,
                "baseline_ms": base["meanMs"],
                "current_ms": result["meanMs"],
                "ratio": result["meanMs"] / base["meanMs"],
            })
    return regressions


def main():
    parser = argparse.ArgumentParser(description="Check benchmark regressions")
    parser.add_argument(
        "--benchmark",
        default=os.path.join("build", "examples", "Release", "benchmark.exe"),
        help="Path to benchmark executable",
    )
    parser.add_argument(
        "--baseline",
        default="benchmark_baseline.json",
        help="Path to baseline JSON file",
    )
    parser.add_argument(
        "--threshold",
        type=float,
        default=1.10,
        help="Regression threshold (1.10 = 10%% slower triggers failure)",
    )
    parser.add_argument(
        "--update-baseline",
        action="store_true",
        help="Update the baseline file with current results",
    )
    args = parser.parse_args()

    current = run_benchmark(args.benchmark)

    if args.update_baseline:
        with open(args.baseline, "w") as f:
            json.dump(current, f, indent=2)
        print(f"Baseline updated: {args.baseline}")
        return

    if not os.path.exists(args.baseline):
        print(f"No baseline found at {args.baseline}. Use --update-baseline to create one.")
        sys.exit(1)

    with open(args.baseline) as f:
        baseline = json.load(f)

    regressions = check_regression(current, baseline, args.threshold)

    if regressions:
        print("PERFORMANCE REGRESSIONS DETECTED:")
        for r in regressions:
            print(
                f"  {r['name']}: {r['baseline_ms']:.3f}ms -> {r['current_ms']:.3f}ms "
                f"({r['ratio']:.2f}x)"
            )
        sys.exit(1)
    else:
        print("No performance regressions detected.")


if __name__ == "__main__":
    main()
