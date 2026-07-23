# 27 — Platform Coverage

## Goal

Add macOS and ARM-based CI lanes to Velox's existing Windows + Linux x86_64 CI. This ensures the engine builds and passes all tests on Apple Silicon (M1/M2/M3), which is increasingly common for developer machines and target platforms for mobile/VR applications.

## Public API

```cpp
// No new public symbols. This item adds CI infrastructure and platform-specific
// build configurations. All changes are in cmake/, .github/, and build scripts.

// Platform detection macro (for conditional compilation if needed):
#if defined(__APPLE__) && defined(__arm64__)
#define VELOX_PLATFORM_MACOS_ARM 1
#elif defined(__APPLE__)
#define VELOX_PLATFORM_MACOS_X86 1
#elif defined(_WIN32)
#define VELOX_PLATFORM_WINDOWS 1
#elif defined(__linux__) && defined(__aarch64__)
#define VELOX_PLATFORM_LINUX_ARM 1
#elif defined(__linux__)
#define VELOX_PLATFORM_LINUX_X86 64 1
#endif
```

## Data structures

- No new library data structures. Platform detection macros are added to `include/velox/platform.h` (new header).

## Algorithm

**CI infrastructure:**

1. **GitHub Actions workflows:** Add macOS runners (`macos-15` for Apple
   Silicon, `macos-15-intel` for Intel Mac) alongside existing Windows and
   Linux runners.
2. **Build configuration:** CMake must detect the platform and select appropriate compiler flags:
   - macOS: use Clang (default on Xcode); link against `-framework Cocoa` for any windowing examples.
   - ARM: ensure SIMD intrinsics are compiled with `-mcpu=apple-m1` or equivalent; verify that all GJK/EPA math works correctly on ARM NEON.
3. **Test execution:** Run the full 48-test stress suite + fuzzing + soak tests on each platform. Some tests may be platform-specific (e.g., CUDA tests skipped on macOS without an NVIDIA GPU).

**Platform-specific considerations:**

- **Apple Silicon:** The CUDA backend is not available on macOS. CI must skip CUDA-specific tests on ARM macOS runners.
- **ARM NEON:** Velox's math functions use portable C++ (no SIMD intrinsics currently), so ARM should work out of the box. Verify that `VELOX_HD` macros compile correctly with Clang on ARM.
- **File system case sensitivity:** macOS defaults to case-insensitive file systems; ensure that test fixtures don't rely on case-sensitive file names.

## Files

- `include/velox/platform.h` — new header with platform detection macros
- `.github/workflows/ci.yml` — add macOS and ARM job definitions (modify existing workflow)
- `cmake/platform.cmake` — new file with platform-specific build configuration

## Tests

1. **macOS build:** Velox compiles and links on macOS 14+ (Apple Silicon) with Clang. All non-CUDA tests pass.
2. **ARM NEON math verification:** Run the GJK/EPA test suite on ARM; verify that all distance calculations match x86_64 results within 1e-6.
3. **Cross-platform determinism:** Run the same scene on Windows, Linux x86_64, and macOS ARM in Strict determinism mode; verify bitwise identical output (requires item 24).

## Acceptance

- [ ] CI includes macOS ARM runner (Apple Silicon)
- [ ] Velox builds successfully on macOS with Clang
- [ ] All non-CUDA tests pass on macOS ARM
- [ ] Platform detection macros correctly identify macOS and ARM platforms
- [ ] CUDA tests are skipped gracefully on platforms without NVIDIA GPUs

## Size: S

## Risks

- Apple Silicon CI runners may have different performance characteristics than x86_64; benchmark baselines must be platform-specific. A test that passes timing-wise on x86 may fail on ARM due to different cache sizes or branch prediction.
- CUDA is not available on macOS. Any CUDA-specific tests must be conditional; document this limitation clearly in the CI configuration.
- Xcode's Clang version may lag behind upstream LLVM; ensure that the minimum supported Clang version (for Velox's C++17 features) is available on the target macOS versions.
