# Cross-Platform Support

Velox targets consistent, deterministic behavior across all major desktop
platforms and both dominant CPU architectures. This document describes the
support matrix, known platform differences, and guidelines for porting.

## Platform Support Matrix

| Platform | Architecture | Compiler | Status | Deterministic |
|----------|-------------|----------|--------|---------------|
| Windows 10+ | x86_64 | MSVC 2022 (v143) | ✅ Supported | ✅ With strict FP |
| Windows 10+ | ARM64 | MSVC 2022 (v143) | ⚠️ Experimental | ✅ With strict FP |
| Ubuntu 22.04+ | x86_64 | GCC 12+ / Clang 15+ | ✅ Supported | ✅ With strict FP |
| Ubuntu 22.04+ | ARM64 | GCC 12+ / Clang 15+ | ✅ Supported | ✅ With strict FP |
| macOS 13 (Ventura) | x86_64 | AppleClang 14+ | ✅ Supported | ✅ With strict FP |
| macOS 14 (Sonoma)+ | ARM64 (Apple Silicon) | AppleClang 15+ | ✅ Supported | ✅ With strict FP |

### Architecture Notes

- **x86_64**: Intel and AMD processors. No AVX requirement — the CPU backend
  uses portable C++17 with no architecture-specific intrinsics.
- **ARM64**: Apple Silicon (M1/M2/M3/M4) and Linux ARM64 (Graviton, Ampere).
  No NEON requirement for the core library.

### Compiler Requirements

- C++17 support (mandatory)
- IEEE 754 single-precision floating-point (mandatory)
- `std::thread` and `std::mutex` (mandatory)
- No external dependencies for the core library

## Determinism Guarantees

### Strict Floating-Point Mode

When built with `-DVELOX_STRICT_FLOATING_POINT=ON`, Velox guarantees
bitwise-identical simulation results across all supported platforms:

| Compiler | Flags Applied |
|----------|--------------|
| MSVC | `/fp:precise` (no FMA contraction) |
| GCC/Clang | `-ffp-contract=off -fno-fast-math -fno-math-errno` |
| NVCC | `--fmad=false` (disable FMA on GPU) |

### What Is Deterministic

- Rigid body integration (position, orientation, velocity)
- Collision detection (GJK/EPA, contact manifolds)
- Constraint solver (sequential impulses)
- Serialization output (V1 and V2 formats)
- Replay recording and verification

### What Is NOT Guaranteed Deterministic

- Thread scheduling order (use single-threaded mode for determinism)
- GPU backend results (CUDA floating-point may differ from CPU)
- Wall-clock timing measurements
- Memory allocation addresses

## Known Platform Differences

### Floating-Point Behavior

| Behavior | x86_64 | ARM64 | Notes |
|----------|--------|-------|-------|
| FMA contraction | Available (FMA3) | Available (NEON) | Disabled in strict mode |
| Denormal handling | Full IEEE 754 | May flush-to-zero | Engine avoids denormal reliance |
| Extended precision | SSE2 (64-bit) | N/A | x87 80-bit not used (C++17) |
| `sqrtf` accuracy | ≤1 ULP | ≤1 ULP | IEEE 754 requirement |
| `sinf`/`cosf` | libm-dependent | libm-dependent | Avoided in hot paths |

### Serialization

- All multi-byte integers are stored in **little-endian** byte order.
- Floats are stored as IEEE 754 binary32 in little-endian.
- The V2 format includes CRC-32 checksums for corruption detection.
- Archives are portable across all supported platforms.

### Struct Alignment

| Struct | Size | Alignment | Notes |
|--------|------|-----------|-------|
| `Vec2` | 8 bytes | 4 | 2 × float |
| `Vec3` | 12 bytes | 4 | 3 × float, tightly packed |
| `Quat` | 16 bytes | 4 | 4 × float |
| `BodyId` | 8 bytes | 8 | Packed slot + generation |
| `Body` | ≥64 bytes | 64 | `alignas(64)` for cache lines |

### Threading

- Thread scheduling is OS-dependent and non-deterministic.
- For reproducible results, use `World` in single-threaded mode or
  ensure deterministic task ordering.
- The `TaskSystem` uses `std::thread` — no platform-specific threading APIs.

## Porting Guide

### Adding a New Platform

1. **Verify compiler support**: Ensure your compiler supports C++17 and
   IEEE 754 single-precision arithmetic.

2. **Add platform macros**: Update `include/velox/platform.h` with
   detection logic for the new platform/architecture combination.

3. **Update CMake**: Add platform-specific flags in `cmake/platform.cmake`
   and `cmake/determinism.cmake` if needed.

4. **Run the test suite**:
   ```bash
   cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
     -DVELOX_ENABLE_CUDA=OFF -DVELOX_STRICT_FLOATING_POINT=ON
   cmake --build build --parallel
   ctest --test-dir build -C Release --output-on-failure
   ```

5. **Verify determinism**: Run the cross-platform trace test and compare
   against a reference platform:
   ```bash
   python scripts/cross_platform_test.py --run-tests --trace-dir traces/
   ```

6. **Update CI**: Add the platform to the matrix in
   `.github/workflows/ci.yml` under the `cross-platform` job.

### Adding a New Architecture

1. Ensure `sizeof(float) == 4`, `sizeof(double) == 8`, and
   `sizeof(void*) == 8` (or update assumptions).

2. Verify that `alignas(64)` is respected by the compiler for the
   `Body` struct.

3. Check that `std::atomic` operations on `uint64_t` are lock-free
   (required for `BodyId` handle validation in concurrent contexts).

4. Run `tests/cross_platform/test_alignment` to verify struct layouts.

### Common Porting Issues

| Issue | Cause | Solution |
|-------|-------|----------|
| FMA differences | Compiler contracts `a*b+c` to `fma(a,b,c)` | Use strict FP mode |
| Denormal crash | ARM FTZ mode enabled | Avoid denormal-dependent code |
| Alignment fault | Unaligned SIMD access | Ensure `alignas(64)` on Body arrays |
| Serialization mismatch | Endianness assumption | Use explicit LE encode/decode |
| NaN propagation | Platform-specific NaN boxing | Use IEEE 754 quiet NaN only |

## Testing

### Local Testing

```bash
# Build and run cross-platform tests
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DVELOX_ENABLE_CUDA=OFF -DVELOX_STRICT_FLOATING_POINT=ON
cmake --build build --parallel
ctest --test-dir build -C Release -R "velox\.xplatform\." --output-on-failure
```

### Cross-Platform Comparison

```bash
# Run tests and generate a report
python scripts/cross_platform_test.py --run-tests --output-dir reports/

# Compare traces from multiple platforms
python scripts/cross_platform_test.py --compare-only --trace-dir traces/
```

### CI Pipeline

The CI pipeline runs cross-platform tests on every push and PR:

1. **cross-platform** job: Builds and tests on 5 platform configurations
   (Linux x86_64, Linux ARM64, Windows x86_64, macOS x86_64, macOS ARM64).

2. **cross-platform-parity** job: Downloads determinism traces from all
   platforms and verifies they are bitwise identical.

3. **strict-cpu** + **strict-parity** jobs: Additional 1000-frame trace
   comparison using the determinism demo.

## References

- `include/velox/platform.h` — Compile-time platform detection
- `cmake/determinism.cmake` — Floating-point compiler flags
- `cmake/platform.cmake` — Platform-specific build policy
- `tests/cross_platform/` — Cross-platform test suite
- `scripts/cross_platform_test.py` — Test runner and trace comparator
- `docs/PORTABILITY.md` — CPU/GPU backend portability
