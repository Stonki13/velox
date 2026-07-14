# 24 — Cross-Platform Determinism

## Goal

Ensure that Velox produces bit-identical simulation results on Windows (MSVC), Linux (GCC, Clang), and any future platform. This is required for lockstep multiplayer where multiple clients must simulate the same scene in sync. Achieved by enforcing fixed-order floating-point operations, disabling FMA instructions, and using deterministic data structures.

## Public API

```cpp
namespace velox {

enum class DeterminismMode : uint8_t {
    Relaxed = 0,        // default: best performance, platform-dependent results
    Strict = 1          // bit-identical across platforms; requires compiler flags
};

class World {
public:
    DeterminismMode determinismMode() const;
    void setDeterminismMode(DeterminismMode mode);
};

} // namespace velox
```

## Data structures

- `DeterminismMode` enum — new, lives in `include/velox/world.h`.
- `World::determinismMode_` member — added to World.

## Algorithm

**Fixed-order math:**

1. **Disable FMA instructions.** FMA (fused multiply-add) can produce different results than separate multiply + add due to rounding differences. Compile with `-ffp-contract=off` (GCC/Clang) and `/fp:precise` (MSVC). Provide a `VELOX_DISABLE_FMA` macro that inserts explicit intermediate variables to prevent FMA contraction.
2. **Enforce operation order.** The solver's sequential-impulse loop iterates over contacts in a fixed order (sorted by pair key). The CUDA backend uses graph coloring which may process contacts in a different order; in Strict mode, the CUDA backend falls back to sequential solving on-device.
3. **Deterministic sorting.** Any sort operation in the pipeline (e.g., sorting contacts by pair key for warm starting) must use a stable, comparison-based sort with a total ordering that is platform-independent. Avoid `std::sort` on floating-point values; use integer keys where possible.

**Platform-specific compiler flags:**

- **MSVC:** `/fp:precise` (disable fast-math), `/arch:AVX2` (fixed SIMD width), no `/fp:fast`.
- **GCC/Clang:** `-ffp-contract=off -fno-fast-math -march=haswell` (or a portable baseline).
- **CUDA:** `--fmad=false` in nvcc flags.

**Verification:**

1. Run the same scene on Windows + Linux + CUDA; compare body positions frame-by-frame.
2. In Strict mode, all three must produce bitwise identical output.
3. In Relaxed mode, results may differ by < 1e-6 due to floating-point ordering.

## Files

- `include/velox/world.h` — add DeterminismMode enum and accessor
- `cmake/determinism.cmake` — new file with compiler flag configuration for strict determinism
- `src/math.h` — add VELOX_FMA_DISABLE macro that prevents FMA contraction

## Tests

1. **Cross-platform parity:** Run 5 canonical scenes on Windows (MSVC), Linux (GCC), and CUDA. In Strict mode, all body positions must match bitwise across all three platforms for 1000 frames.
2. **FMA disable verification:** Compile a test that performs `a * b + c`; verify the assembly output does not contain `vfmadd` instructions when `VELOX_DISABLE_FMA` is active.
3. **Deterministic sorting:** Generate 1000 random pair keys; sort them on Windows and Linux; verify the sorted arrays are identical.

## Acceptance

- [ ] Strict determinism mode produces bitwise identical results across MSVC, GCC, Clang, and CUDA
- [ ] FMA instructions are disabled in Strict mode (verified via assembly inspection)
- [ ] All sorts use stable, platform-independent comparison
- [ ] Cross-platform test passes for all 5 canonical scenes

## Size: M

## Risks

- Disabling FMA and fast-math can reduce performance by 5-15% on modern CPUs. Must document the tradeoff and make Strict mode opt-in.
- CUDA's graph-colored solver inherently processes contacts in a different order than the CPU sequential solver. Achieving bitwise parity requires the CUDA backend to use the same ordering, which may negate the parallelism benefit. Consider accepting < 1e-6 tolerance for GPU vs CPU comparison rather than strict bitwise equality.
- Some floating-point operations (e.g., `sqrt`, `sin`) can produce platform-dependent results at the last few ULPs. Use `std::sqrt` consistently and avoid platform-specific math intrinsics.
