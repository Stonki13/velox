# Production Readiness

Velox 1.x is released only after the repository gates below are green. These
gates are intentionally split between portable hosted CI and hardware-backed
validation; a hosted runner cannot prove that a CUDA kernel executed on a real
GPU.

## Required Pull Request Gates

- Windows, Linux, and macOS build and run the CPU suite.
- Strict floating-point traces match across Windows, Linux x86_64, macOS
  Intel, and Apple Silicon. Linux ARM64 runs the cross-platform suite.
- Linux full integration builds the optional sandbox and Jolt differential
  test, then validates a shared installed package through
  `tests/package_consumer`.
- AddressSanitizer, UndefinedBehaviorSanitizer, and ThreadSanitizer jobs run
  the relevant stress, fuzz, soak, and concurrent-access coverage.
- The documentation job builds the API reference.

## GPU Hardware Gate

`.github/workflows/cuda.yml` is intentionally dispatched on a self-hosted
runner labelled `self-hosted`, `windows`, `x64`, and `gpu`. It requires a CUDA
toolkit and a real NVIDIA device. The workflow records `nvidia-smi`, compiles
the CUDA backend, and runs `velox.cuda_smoke`, `stress_demo`, and
`proto_manifold`. It is manually dispatched and capped at 90 minutes because
the clean `sm_120` CUDA build is substantially more expensive than CPU builds.
CUDA 13+ builds use the compiler's `min` fast-compile mode by default; set
`VELOX_CUDA_FAST_COMPILE=0` to restore every device optimization when an
offline release build can absorb the longer compilation time.

`velox.cuda_smoke` fails if `BackendType::Cuda` falls back to CPU, if GPU
substeps were not used, or if a 240-frame CPU/CUDA scene diverges beyond its
documented tolerance. Dispatch it with `cuda_architectures=120` for an RTX
50-series runner, or `native` to let a recent CMake select the installed GPU.

### Validated Hardware

On Windows 11 with CUDA 13.2 and an RTX 5080 (`sm_120`), the clean Release
build completed in 75 seconds with `VELOX_CUDA_FAST_COMPILE=min`. The CUDA
smoke/parity test, stress demo, and persistent-manifold prototype passed in
the same hardware workflow.

## Release Checklist

1. All pull request gates are green for the release commit.
2. Run the CUDA hardware workflow on each supported driver/toolkit baseline.
3. Run the scheduled extended fuzz and 60,000-frame soak jobs successfully.
4. Review benchmark results on the maintained CPU and GPU reference machines;
   explain or fix regressions above 10 percent.
5. Verify the version header, `vcpkg.json`, and `CHANGELOG.md` agree.
6. Create the signed `vX.Y.Z` tag. The release workflow packages CPU-only
   Windows, Linux, and macOS install trees and publishes SHA-256 manifests.

The CPU backend is portable across Intel and AMD processors. CUDA acceleration
remains NVIDIA-only; AMD and Intel GPU deployments use the CPU backend.
