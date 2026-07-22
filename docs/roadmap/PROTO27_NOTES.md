# Roadmap 27: Platform Coverage

## Delivered

- Extended the public platform header with explicit macOS Intel/Apple Silicon
  and Linux x86_64/ARM64 identifiers. Every platform and architecture macro is
  always defined as `0` or `1`.
- Added `cmake/platform.cmake`, invoked for the Velox target, to centralize
  platform configuration without forcing a host-specific CPU architecture
  flag into redistributable packages.
- Added `platform_demo` and CTest `velox.platform`; it verifies exactly one
  platform and one architecture at compile time.
- Made macOS coverage explicit in CI job names: `macos-13` is the Intel lane
  and `macos-14` is the Apple Silicon ARM64 lane. Both CPU and strict replay
  matrices configure with `VELOX_ENABLE_CUDA=OFF`, so CUDA is skipped
  deliberately on unsupported platforms.

## Verification

- CPU-only Release `platform_demo`: passed on Windows x86_64.
- CUDA-enabled Release `platform_demo`: passed on Windows x86_64.
- The existing CI CPU and strict matrices now provide the macOS build and full
  non-CUDA test gates; GitHub Actions is the authoritative verification for
  macOS and ARM runners.

## Merge Recommendation

Ready to merge. The macOS ARM execution result is necessarily pending the
first hosted CI run, not substituted by a local Windows result.
