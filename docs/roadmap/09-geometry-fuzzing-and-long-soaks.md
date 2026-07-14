# 09 — Geometry Fuzzing and Long Soaks

## Goal

Add a dedicated geometry fuzzer that generates random degenerate shapes (near-coplanar hulls, needle bodies, duplicate vertices, zero-volume meshes) and runs them through GJK/EPA/CCD to catch numerical edge cases. Pair this with multi-hour soak tests that track energy/momentum drift over thousands of simulation steps to surface slow accumulations that unit tests miss.

## Public API

```cpp
namespace velox {
namespace fuzz {

// Fuzz configuration: controls the distribution and constraints of generated shapes.
struct FuzzConfig {
    int maxVertices = 32;           // max vertices per random hull
    float minScale = 1e-4f;         // minimum shape scale
    float maxScale = 1e3f;          // maximum shape scale
    float degenerateProbability = 0.3f;  // chance of generating a degenerate shape
    int maxPairsPerRun = 10000;     // pairs to test per fuzz invocation
    bool enableSoakTest = true;     // run long-duration soak after fuzzing
    int soakFrames = 60000;         // frames for soak test (≈17 min at 60 Hz)
    float driftTolerance = 0.05f;   // max allowed energy drift as fraction of initial
};

// Fuzz test result: summary statistics after a fuzz run.
struct FuzzResult {
    int pairsTested = 0;
    int failures = 0;               // GJK/EPA returned NaN/Inf or crashed
    int symmetryViolations = 0;     // gjkDistance(A,B).distance != gjkDistance(B,A)
    int witnessFailures = 0;        // witness point not on actual surface
    float maxDriftInSoak = 0.0f;    // max energy drift during soak test
    bool passed = false;            // true if failures == 0 and drift < tolerance
};

// Run the geometry fuzzer with the given configuration.
FuzzResult runGeometryFuzzer(const FuzzConfig& config);

// Generate a random convex hull from a Gaussian point cloud.
std::vector<Vec3> generateRandomHull(int vertexCount, float scale,
                                      bool allowDegenerate = false);

// Generate a random scene for soak testing: N bodies dropped onto a plane.
struct SoakScene {
    int bodyCount;
    float planeY;
    float gravityY;
    std::vector<Vec3> initialPositions;
    std::vector<float> masses;
};
SoakScene generateSoakScene(int bodyCount, float scale);

} // namespace fuzz
} // namespace velox
```

## Data structures

- `FuzzConfig` struct — new file `tests/fuzz.h`. Controls fuzz generation parameters.
- `FuzzResult` struct — new file `tests/fuzz.h`. Aggregated test results.
- `SoakScene` struct — new file `tests/fuzz.h`. Scene description for soak tests.
- `fuzz::runGeometryFuzzer()` — new function in `tests/fuzz.cpp`.
- `fuzz::generateRandomHull()` — new function in `tests/fuzz.cpp`.
- `fuzz::generateSoakScene()` — new function in `tests/fuzz.cpp`.

## Algorithm

**Geometry fuzzer:**

1. **Shape generation.** For each pair to test:
   a. With probability `degenerateProbability`, generate a degenerate shape: all vertices coplanar, or all vertices collinear, or duplicate vertices within 1e-8.
   b. Otherwise, sample N vertices from a 3D Gaussian (σ = scale), compute the convex hull via QuickHull, scale by a random factor in [minScale, maxScale].
   c. Pair each generated hull with a random primitive (sphere, box, capsule) at a random offset.
2. **GJK/EPA validation.** For each pair:
   a. Call `gjkDistance(A, B)` and `gjkDistance(B, A)`; verify symmetry within 1e-6.
   b. If overlapping, call `epaPenetration()` and verify the witness points lie on the actual surfaces (re-support in the normal direction and check distance < 1e-4).
   c. If not overlapping, verify the distance equals the analytical minimum for simple shapes (sphere-sphere, sphere-box) when applicable.
   d. Track NaN/Inf outputs; any occurrence is a failure.
3. **Metamorphic properties.** Randomly select 10% of pairs and verify:
   a. Translation invariance: translate both shapes by the same vector; distance unchanged.
   b. Rotation invariance: rotate both shapes by the same quaternion; distance unchanged.
   c. Scale consistency: scale both shapes by the same factor; distance scales proportionally.

**Soak test:**

1. **Scene generation.** Drop N bodies (random sizes, random initial heights) onto a static plane. Bodies have random masses and zero initial velocity.
2. **Run simulation.** Step the world for `soakFrames` frames at 60 Hz. At every 600th frame (every 10 seconds), record: total kinetic energy, total potential energy, total momentum, number of sleeping bodies.
3. **Drift analysis.** Compute:
   a. Energy drift = |E_final - E_initial| / |E_initial|. Must be < `driftTolerance` (default 5%).
   b. Momentum drift = |p_final - p_initial| / |p_initial|. Must be < 1% (external forces from gravity should conserve horizontal momentum).
   c. Sleeping ratio = sleeping bodies / total bodies at end. Must be > 90% (most bodies should have settled).
4. **Failure conditions.** Soak test fails if: energy drift > tolerance, any body tunnels through the plane, or any body achieves velocity > 100 m/s (indicates instability).

## Files

**New files:**
- `tests/fuzz.h` — FuzzConfig, FuzzResult, SoakScene definitions and function declarations
- `tests/fuzz.cpp` — fuzzer implementation, random shape generation, validation logic
- `tests/soak_test.cpp` — soak test harness, drift tracking, failure detection
- `cmake/fuzz_targets.cmake` — CMake target definitions for fuzz/soak tests (new file)

**Modified files:**
- None to core engine; all changes in tests/

## Tests

1. **Fuzzer 10⁶ pairs:** Run with FuzzConfig{maxPairsPerRun=1000000}. Must have zero symmetry violations, zero NaN outputs, zero witness failures.
2. **Soak test 60000 frames (17 min):** 50 bodies dropped on a plane. Energy drift < 5%, all bodies sleeping at end, no tunneling through plane.
3. **Degenerate shape stress:** Generate 1000 coplanar hulls (all z=0); GJK must not crash or return NaN for any pair. EPA should fall back to a valid 2D normal.
4. **Needle shape test:** Generate 100 needle shapes (aspect ratio > 1000:1); all GJK/EPA calls must complete in < 1 ms and return finite values.

## Acceptance

- [ ] `runGeometryFuzzer()` runs 10⁶ random pairs with zero failures
- [ ] Soak test runs 60000 frames without tunneling or energy drift > 5%
- [ ] Degenerate shapes (coplanar, collinear, duplicate vertices) handled gracefully
- [ ] Fuzzer integrated into CI: runs nightly for 30 minutes with failure alerting
- [ ] `generateRandomHull()` produces valid convex hulls that pass QuickHull validation

## Size: M

## Risks

- Soak test duration (17 min per run) is long for CI. Must be optional in PR builds; required only in nightly runs. Consider reducing soakFrames to 6000 for PR validation with relaxed tolerances.
- The degeneracy probability (30%) generates many truly degenerate shapes that may not exercise meaningful code paths. Focus on "near-degenerate" shapes (slightly coplanar, slightly collinear) which are more likely to trigger numerical issues in production.
- Energy drift tolerance of 5% is generous; some scenes (e.g., many resting contacts) may naturally drift due to solver approximation. Must distinguish between expected drift (from splitting impulses) and unexpected drift (from bugs).
