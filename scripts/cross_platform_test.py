#!/usr/bin/env python3
"""Cross-platform test runner and determinism comparator for Velox.

Runs the cross-platform test suite on the local machine, collects
determinism traces, and compares results across platforms when multiple
trace files are available.

Usage:
    python scripts/cross_platform_test.py [options]

Options:
    --build-dir DIR       Build directory (default: build)
    --trace-dir DIR       Directory containing trace files from other platforms
    --output-dir DIR      Directory for reports (default: reports)
    --run-tests           Build and run the cross-platform tests locally
    --compare-only        Skip local tests, only compare existing traces
    --verbose             Verbose output

The script produces:
    - A JSON compatibility report in the output directory
    - A human-readable summary printed to stdout
    - Exit code 0 if all checks pass, 1 otherwise
"""

import argparse
import hashlib
import json
import os
import platform
import re
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path


# ---------------------------------------------------------------------------
# Platform identification
# ---------------------------------------------------------------------------

def identify_platform():
    """Return a dict describing the current platform."""
    system = platform.system()
    machine = platform.machine().lower()

    os_name = {
        "Windows": "windows",
        "Linux": "linux",
        "Darwin": "macos",
    }.get(system, system.lower())

    arch = "x86_64"
    if machine in ("arm64", "aarch64"):
        arch = "arm64"
    elif machine in ("x86_64", "amd64", "x64"):
        arch = "x86_64"

    return {
        "os": os_name,
        "arch": arch,
        "platform_id": f"{os_name}-{arch}",
        "python_version": platform.python_version(),
        "system": system,
        "machine": platform.machine(),
        "processor": platform.processor(),
    }


# ---------------------------------------------------------------------------
# Build and test execution
# ---------------------------------------------------------------------------

def run_command(cmd, cwd=None, verbose=False):
    """Run a command and return (returncode, stdout, stderr)."""
    if verbose:
        print(f"  > {' '.join(cmd)}")
    result = subprocess.run(
        cmd, cwd=cwd, capture_output=True, text=True, timeout=300
    )
    if verbose and result.stdout:
        print(result.stdout)
    if verbose and result.stderr:
        print(result.stderr, file=sys.stderr)
    return result.returncode, result.stdout, result.stderr


def configure_build(build_dir, strict_fp=True, verbose=False):
    """Configure the CMake build with cross-platform tests enabled."""
    cmd = [
        "cmake", "-S", ".", "-B", str(build_dir),
        "-DCMAKE_BUILD_TYPE=Release",
        "-DVELOX_ENABLE_CUDA=OFF",
        "-DVELOX_BUILD_EXAMPLES=OFF",
        "-DBUILD_TESTING=ON",
    ]
    if strict_fp:
        cmd.append("-DVELOX_STRICT_FLOATING_POINT=ON")

    print(f"[configure] {' '.join(cmd)}")
    rc, out, err = run_command(cmd, verbose=verbose)
    if rc != 0:
        print(f"[ERROR] CMake configure failed:\n{err}")
        return False
    return True


def build_tests(build_dir, verbose=False):
    """Build the cross-platform test targets."""
    targets = [
        "test_determinism",
        "test_floating_point",
        "test_endianness",
        "test_alignment",
    ]
    for target in targets:
        cmd = ["cmake", "--build", str(build_dir), "--config", "Release",
               "--target", target]
        print(f"[build] {target}")
        rc, out, err = run_command(cmd, verbose=verbose)
        if rc != 0:
            print(f"[ERROR] Build failed for {target}:\n{err}")
            return False
    return True


def run_ctest(build_dir, verbose=False):
    """Run the cross-platform tests via ctest."""
    cmd = [
        "ctest", "--test-dir", str(build_dir),
        "-C", "Release",
        "-R", "velox\\.xplatform\\.",
        "--output-on-failure",
    ]
    print(f"[test] {' '.join(cmd)}")
    rc, out, err = run_command(cmd, verbose=verbose)
    if rc != 0:
        print(f"[ERROR] Tests failed:\n{out}\n{err}")
        return False, out
    return True, out


def extract_trace(build_dir, verbose=False):
    """Run the determinism test and extract the trace output."""
    # Find the test executable.
    test_exe = None
    for pattern in [
        build_dir / "tests" / "cross_platform" / "Release" / "test_determinism.exe",
        build_dir / "tests" / "cross_platform" / "test_determinism.exe",
        build_dir / "tests" / "cross_platform" / "test_determinism",
    ]:
        if pattern.exists():
            test_exe = pattern
            break

    if test_exe is None:
        print("[WARN] Could not find test_determinism executable")
        return None

    print(f"[trace] Running {test_exe}")
    rc, out, err = run_command([str(test_exe)], verbose=verbose)

    # Extract the trace line.
    for line in out.splitlines():
        if line.startswith("VELOX_XPLATFORM_TRACE "):
            trace_hex = line[len("VELOX_XPLATFORM_TRACE "):].strip()
            return trace_hex

    print("[WARN] No trace found in test output")
    return None


# ---------------------------------------------------------------------------
# Trace comparison
# ---------------------------------------------------------------------------

def load_traces(trace_dir):
    """Load trace files from a directory.

    Expected file naming: strict-trace-<os>.txt or trace-<platform_id>.txt
    Each file contains lines starting with VELOX_XPLATFORM_TRACE or
    VELOX_STRICT_TRACE followed by hex data.
    """
    traces = {}
    trace_path = Path(trace_dir)
    if not trace_path.exists():
        return traces

    for f in sorted(trace_path.iterdir()):
        if not f.is_file():
            continue
        content = f.read_text(errors="replace")
        for line in content.splitlines():
            line = line.strip()
            for prefix in ("VELOX_XPLATFORM_TRACE ", "VELOX_STRICT_TRACE "):
                if line.startswith(prefix):
                    trace_hex = line[len(prefix):].strip()
                    # Derive platform name from filename.
                    name = f.stem
                    for p in ("strict-trace-", "trace-"):
                        if name.startswith(p):
                            name = name[len(p):]
                            break
                    traces[name] = trace_hex
                    break

    return traces


def compare_traces(traces):
    """Compare all traces for bitwise equality.

    Returns (all_match, details) where details is a list of comparison results.
    """
    if len(traces) < 2:
        return True, [{"status": "skipped", "reason": "fewer than 2 traces"}]

    names = sorted(traces.keys())
    reference_name = names[0]
    reference = traces[reference_name]
    ref_hash = hashlib.sha256(reference.encode()).hexdigest()[:16]

    details = []
    all_match = True

    for name in names[1:]:
        trace = traces[name]
        trace_hash = hashlib.sha256(trace.encode()).hexdigest()[:16]
        match = trace == reference

        if not match:
            all_match = False
            # Find first differing position.
            first_diff = -1
            for i, (a, b) in enumerate(zip(reference, trace)):
                if a != b:
                    first_diff = i
                    break
            if first_diff == -1:
                first_diff = min(len(reference), len(trace))

            details.append({
                "platform": name,
                "status": "MISMATCH",
                "sha256_prefix": trace_hash,
                "first_diff_char": first_diff,
                "ref_length": len(reference),
                "trace_length": len(trace),
            })
        else:
            details.append({
                "platform": name,
                "status": "MATCH",
                "sha256_prefix": trace_hash,
            })

    return all_match, details


# ---------------------------------------------------------------------------
# Report generation
# ---------------------------------------------------------------------------

def generate_report(platform_info, test_passed, trace, traces, all_match,
                    details, output_dir):
    """Generate a JSON compatibility report."""
    report = {
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "local_platform": platform_info,
        "test_suite_passed": test_passed,
        "local_trace_sha256": (
            hashlib.sha256(trace.encode()).hexdigest() if trace else None
        ),
        "local_trace_length": len(trace) if trace else 0,
        "cross_platform_comparison": {
            "traces_compared": len(traces),
            "all_match": all_match,
            "details": details,
        },
        "platforms_tested": sorted(traces.keys()) if traces else [],
        "verdict": "PASS" if (test_passed and all_match) else "FAIL",
    }

    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    report_file = output_path / "cross_platform_report.json"
    report_file.write_text(json.dumps(report, indent=2) + "\n")
    print(f"\n[report] Written to {report_file}")

    # Also write a human-readable summary.
    summary_file = output_path / "cross_platform_summary.txt"
    lines = [
        "Velox Cross-Platform Compatibility Report",
        "=" * 50,
        f"Generated: {report['generated_at']}",
        f"Platform:  {platform_info['platform_id']}",
        f"OS:        {platform_info['system']} ({platform_info['machine']})",
        "",
        f"Test suite: {'PASS' if test_passed else 'FAIL'}",
        f"Trace hash: {report['local_trace_sha256'] or 'N/A'}",
        "",
        f"Cross-platform traces compared: {len(traces)}",
        f"All traces match: {'YES' if all_match else 'NO'}",
        "",
    ]
    for d in details:
        if d.get("status") == "skipped":
            lines.append(f"  [SKIP] {d.get('reason', 'unknown')}")
        elif d["status"] == "MATCH":
            lines.append(f"  [MATCH] {d['platform']} (sha256:{d['sha256_prefix']})")
        else:
            lines.append(
                f"  [MISMATCH] {d['platform']} "
                f"(first diff at char {d['first_diff_char']}, "
                f"sha256:{d['sha256_prefix']})"
            )
    lines.append("")
    lines.append(f"VERDICT: {report['verdict']}")
    lines.append("")

    summary_file.write_text("\n".join(lines))
    print(f"[report] Summary written to {summary_file}")

    return report


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Velox cross-platform test runner and determinism comparator"
    )
    parser.add_argument("--build-dir", default="build",
                        help="Build directory (default: build)")
    parser.add_argument("--trace-dir", default=None,
                        help="Directory with trace files from other platforms")
    parser.add_argument("--output-dir", default="reports",
                        help="Output directory for reports (default: reports)")
    parser.add_argument("--run-tests", action="store_true",
                        help="Build and run cross-platform tests locally")
    parser.add_argument("--compare-only", action="store_true",
                        help="Only compare existing traces, skip local tests")
    parser.add_argument("--verbose", action="store_true",
                        help="Verbose output")
    args = parser.parse_args()

    platform_info = identify_platform()
    print(f"[info] Platform: {platform_info['platform_id']}")
    print(f"[info] System:   {platform_info['system']} {platform_info['machine']}")

    build_dir = Path(args.build_dir)
    test_passed = True
    trace = None
    traces = {}

    if not args.compare_only:
        if args.run_tests:
            # Configure, build, and run tests.
            if not configure_build(build_dir, strict_fp=True, verbose=args.verbose):
                test_passed = False
            elif not build_tests(build_dir, verbose=args.verbose):
                test_passed = False
            else:
                test_passed, _ = run_ctest(build_dir, verbose=args.verbose)
                trace = extract_trace(build_dir, verbose=args.verbose)
        else:
            # Just try to extract a trace from an existing build.
            trace = extract_trace(build_dir, verbose=args.verbose)

    # Load and compare traces.
    if args.trace_dir:
        traces = load_traces(args.trace_dir)
        print(f"[info] Loaded {len(traces)} trace(s) from {args.trace_dir}")

    # Add local trace to the comparison set.
    if trace:
        traces[platform_info["platform_id"]] = trace

    all_match, details = compare_traces(traces)

    # Generate report.
    report = generate_report(
        platform_info, test_passed, trace, traces, all_match, details,
        args.output_dir
    )

    # Print verdict.
    print(f"\n{'=' * 50}")
    print(f"VERDICT: {report['verdict']}")
    print(f"{'=' * 50}")

    return 0 if report["verdict"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
