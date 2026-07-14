# 25 — Large Worlds (Double Precision)

## Goal

Add an optional double-precision mode for worlds with coordinates exceeding ~10 km from the origin, where single-precision floating-point noise causes jitter, tunneling, and instability. Double precision provides ~15 decimal digits of accuracy vs ~7 for float; at 10 km scale, float loses centimeter-level precision while double retains millimeter precision.

## Public API

```cpp
namespace velox {

enum class PrecisionMode : uint8_t {
    Single = 0,       // default: float32 everywhere (current behavior)
    Double = 1        // double64 for positions, orientations, velocities
};

class World {
public:
    PrecisionMode precisionMode() const;
    void setPrecisionMode(PrecisionMode mode);

    // Scale-aware tolerances: automatically adjust GJK/EPA/solver tolerances
    // based on the world's coordinate scale. Call after shifting origin or
    // adding large bodies.
    void updateScaleAwareTolerances();
};

// Compute the recommended precision mode for a given world extent.
inline PrecisionMode recommendPrecisionMode(float maxCoordinate) {
    return maxCoordinate > 10000.0f ? PrecisionMode::Double : PrecisionMode::Single;
}

} // namespace velox
```

## Data structures

- `PrecisionMode` enum — new, lives in `include/velox/world.h`.
- `World::precisionMode_` member — added to World.
- Scale-aware tolerance parameters: `gjkTolerance`, `epaTolerance`, `solverBias`, scaled by `worldScale = max(|position|) + shapeRadius`.

## Algorithm

**Double-precision mode:**

1. When `PrecisionMode::Double` is active, all position/orientation/velocity fields use `double` instead of `float`. The `Body` struct has a compile-time branch (`#ifdef VELOX_DOUBLE_PRECISION`) that selects the appropriate type.
2. All math functions in `math.h` have double-precision overloads (or are compiled with `-DVELOX_DOUBLE=1` to use `double` variants).
3. GJK/EPA tolerances scale with the world extent: `tolerance = 1e-6 * worldScale` instead of a fixed `1e-4`. This prevents the solver from treating legitimate double-precision noise as penetration.

**Scale-aware tolerances:**

1. Track the maximum coordinate magnitude in the world (updated on `shiftOrigin()`, body creation, and origin shift).
2. Compute `worldScale = maxCoordinate + maxShapeRadius`.
3. Adjust GJK/EPA tolerance: `gjkTolerance = vmax(1e-6f, 1e-8f * worldScale)`.
4. Adjust solver bias: `biasFactor = vmin(0.1f, 1e-4f * worldScale)`.
5. These tolerances are recomputed on `updateScaleAwareTolerances()` or automatically after origin shifts.

**Performance considerations:**

- Double precision is ~2× slower than single on most CPUs (wider registers, more memory bandwidth). On GPU, double precision throughput is typically 32× slower than single on consumer GPUs; must document this and recommend double only for worlds > 10 km extent.
- The CUDA backend's double-precision path requires `__double2float_rd` conversions at the host-device boundary if mixing modes. In pure double mode, all device code uses `double`.

## Files

- `include/velox/world.h` — add PrecisionMode enum, accessor, tolerance update method
- `include/velox/math.h` — add double-precision overloads (or compile-time type selection)
- `src/gjk.h` — scale tolerances based on world extent
- `cmake/precision.cmake` — new file with VELOX_DOUBLE_PRECISION build flag

## Tests

1. **Large world stability:** 100 bodies in a world centered at (10000, 10000, 10000). In single precision, tower drifts > 1 m over 10 seconds. In double precision, drift < 0.01 m.
2. **Tolerance scaling:** World with max coordinate 50000 m; GJK tolerance automatically set to ≥ 5e-5 (vs 1e-6 for a world at origin). Verify via debug query.
3. **Origin shift preserves precision:** Shift a large world by (-10000, -10000, -10000); coordinates near origin should have full single-precision accuracy after the shift.

## Acceptance

- [ ] `PrecisionMode::Double` produces stable simulations for worlds with coordinates > 10 km
- [ ] Scale-aware tolerances adjust automatically based on world extent
- [ ] Double precision mode has ≤ 3× performance overhead vs single on CPU
- [ ] CUDA double-precision path compiles and runs correctly

## Size: M

## Risks

- Mixing single and double precision in the same world (e.g., a large static terrain with small dynamic bodies near the origin) requires careful tolerance management. The solver must use the appropriate precision for each body based on its coordinate magnitude.
- Double precision on GPU is significantly slower than single. For worlds that only need double precision in one region, consider hybrid mode: double for bodies beyond 10 km, single for others. This adds complexity to the backend interface.
- `shiftOrigin()` must handle double-precision worlds correctly; subtracting a large offset from double coordinates and storing back as float would lose precision. In double mode, all internal storage remains double regardless of origin shift.
