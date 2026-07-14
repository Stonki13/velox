# 02 — GJK/EPA Hardening

## Goal

Make the GJK/EPA distance/penetration solver robust against degenerate inputs: near-coplanar hull faces, needle shapes (aspect ratio > 1000:1), shapes with volumes below machine epsilon, and duplicate/coincident vertices. Add a dedicated geometry fuzzer that exercises these edge cases continuously so regressions are caught before they reach the solver.

## Public API

```cpp
namespace velox {

// Geometry quality hint for GJK/EPA. Lets callers opt into extra robustness
// passes at the cost of a few extra support queries.
enum class GeometryQuality : uint8_t {
    Normal = 0,       // standard GJK/EPA path
    Robust = 1,       // extra degeneracy checks, larger tolerances
    Paranoid = 2      // full metamorphic verification per call (testing only)
};

struct Body;  // forward declaration

// Runtime geometry diagnostics. Populated when GeometryQuality >= Robust.
struct GeometryDiagnostics {
    float minEdgeLength = 0.0f;       // shortest edge in the shape
    float maxEdgeLength = 0.0f;       // longest edge
    float aspectRatio = 1.0f;         // maxEdge / minEdge
    float volume = 0.0f;              // computed via divergence theorem
    bool isDegenerate = false;        // volume < 1e-12 * maxEdge^3
    int nearCoplanarFaceCount = 0;    // faces within 1e-3 rad of coplanar
};

// Query: compute diagnostics for a body's shape. O(n) in hull vertex count.
GeometryDiagnostics queryGeometryDiagnostics(const Body& body) const;

} // namespace velox
```

## Data structures

- `GeometryQuality` enum — new, lives in `include/velox/body.h`. Per-body quality hint stored as a uint8_t field on `Body`.
- `GeometryDiagnostics` struct — new file `src/geometry_diagnostics.h`. Computed lazily and cached per body.
- `Body::geometryQuality` field — added to `include/velox/body.h`, default `Normal`.
- `Body::diagnosticsCache` — optional cached diagnostics, lives in `src/world.cpp` as a parallel vector keyed by dense index.

## Algorithm

**GJK hardening steps:**

1. **Pre-flight degeneracy check.** Before running GJK, compute the bounding box of the shape's support function samples (8 axis-aligned directions). If max-edge / min-edge > 1000:0, flag as needle and use anisotropic scaling in the support direction computation to prevent loss of precision.
2. **Duplicate vertex rejection.** When building a hull from points, reject vertices within 1e-8 of any existing vertex. For GJK input, collapse coincident support directions.
3. **Near-coplanar face detection.** After EPA convergence, walk all faces and compute the dihedral angle between adjacent faces. If < 1e-3 rad, merge them into a single super-face for manifold generation (prevents jitter from two nearly-parallel contact normals).
4. **Tetrahedron volume floor.** In `expandOverlapSimplex`, require the seed tetrahedron to have volume > 1e-10 * supportDistance³. If below, perturb vertices along random directions until the floor is met or give up and fall back to a center-based overlap normal.
5. **EPA face-area filter.** During EPA iteration, skip faces whose area < 1e-8 * maxFaceArea. These are numerical artifacts that cause spurious penetration normals.

**Geometry fuzzer algorithm:**

1. Generate random convex hulls: 4–64 vertices sampled from a Gaussian (σ = 1), scaled by a random log-uniform factor (1e-4 to 1e3).
2. Run GJK/EPA on all pairs of generated shapes against each other and against primitives (sphere, box, capsule).
3. Check metamorphic properties: (a) `gjkDistance(A,B).distance == gjkDistance(B,A).distance` (symmetry), (b) translation invariance (translate both by same vector, distance unchanged), (c) witness points lie on actual surfaces (re-support and verify).
4. Track failure rate; abort the fuzz run if any property fails more than 0.1% of the time.

## Files

**New files:**
- `src/geometry_diagnostics.h` — GeometryDiagnostics computation, degeneracy detection
- `tests/geometry_fuzz.cpp` — geometry fuzzer harness
- `cmake/fuzz_targets.cmake` — CMake target for fuzz build

**Modified files:**
- `include/velox/body.h` — add `GeometryQuality` field to Body struct
- `src/gjk.h` — add degeneracy guards in `gjkDistance`, `expandOverlapSimplex`, `epaPenetration`
- `CMakeLists.txt` — add fuzz test target (note: per hard rules we do NOT modify CMakeLists; instead document the addition in Files section)

## Tests

1. **Needle shape GJK:** Box with half-extents {500, 0.001, 0.001} vs sphere at distance 0.5 must return correct distance and normal without NaN/Inf.
2. **Coplanar hull faces:** Hull with 100 vertices all lying on z=0 plane (degenerate 2D) must not crash GJK; should fall back to a valid 2D normal.
3. **Tiny shape pair:** Two shapes with volume < 1e-15 must still produce a well-defined contact normal, not arbitrary NaN.
4. **Fuzzer soak (1 hour):** Run geometry fuzzer for 10⁶ random pairs; zero symmetry violations, zero NaN outputs, witness points within 1e-6 of actual surface.
5. **EPA near-coplanar merge:** Two hulls resting on nearly-parallel faces (dihedral < 0.001 rad) must produce a single stable contact normal, not two conflicting ones that cause jitter.

## Acceptance

- [ ] GJK returns valid results for all shapes with aspect ratio up to 10⁶:1
- [ ] EPA handles degenerate (coplanar/duplicate) input without crashing or returning NaN
- [ ] Geometry fuzzer runs in CI for 30 minutes nightly with zero metamorphic violations
- [ ] `queryGeometryDiagnostics` returns correct aspect ratio and volume for all shape types
- [ ] Near-coplanar face merging reduces resting jitter by ≥ 50% on hull stacks

## Size: M

## Risks

- The anisotropic scaling in GJK for needle shapes can distort the Minkowski difference, causing false negatives on close calls. Must validate against analytical needle-vs-needle cases.
- EPA face-area filtering threshold (1e-8) is heuristic; too aggressive and you lose real contacts, too lax and you keep artifacts. Needs tuning per scene type.
- Geometry fuzzer generates random hulls which may not cover the worst-case distributions (e.g., all vertices on a great circle). Seeded regression cases are essential.
