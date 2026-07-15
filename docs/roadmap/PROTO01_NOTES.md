# PROTO01 - Persistent Contact Manifolds: Review Status

## Status

Reviewed and integrated on `proto/manifolds`. The CPU and CUDA-enabled Release
builds pass the full existing suite. `proto_manifold` now verifies the roadmap
acceptance behavior directly: a resting box produces four points with stable
feature keys and remains stable with regenerated anchors, warm starting, and
three solver iterations.

## Audit Findings and Fixes

1. Face extraction used an arbitrary depth slab instead of the support faces.
   `src/manifold.h` now selects the actual Box face and the co-planar Hull
   support polygon for the contact normal.
2. The Sutherland-Hodgman clipper used an invalid intersection denominator and
   could read its ping-pong buffers in the wrong order. It now clips in a
   deterministic 2D basis with the standard signed-edge interpolation.
3. Clipped points lost their witness-plane separation: `pointA` and `pointB`
   could coincide and every clipped contact had a zero gap. Witnesses are now
   reconstructed on their respective face planes and retain the per-manifold
   signed gap.
4. Feature keys were not stable for clipped intersections. Vertex, edge, and
   face identifiers are now deterministic; reduction preserves the deepest
   point and selects the remaining points by deterministic farthest spacing.
5. Narrow phase integration used an unrelated plane-distance expression and
   emitted GJK depth for every manifold point. It now emits each clipped
   witness midpoint with its own normal, gap, and feature key.
6. Warm starting no longer reuses world anchors. Contacts are regenerated from
   current witnesses every frame; only accumulated impulses are copied when a
   feature key matches.
7. Hull-vs-plane speculative contacts stopped after the first distant support
   candidate, which allowed deterministic high-speed hull tunneling. The
   premature cutoff was removed.
8. The extended 80-scene fuzzer exposed an unbounded hinge positional bias.
   Hinge correction is now capped at 10 m/s in both the CPU world solver and
   the shared `VELOX_HD` joint solver.
9. `ContactFeature` is now exposed through `include/velox/world.h` as required
   by the roadmap.
10. Runtime hulls now retain packed QuickHull triangle indices in `MeshSoup`.
    Clipping selects and merges coplanar support triangles into one stable
    support polygon; the same immutable topology is uploaded to CUDA.
11. Topology clipping now requires the selected face to be within 0.95 of the
    contact-normal alignment. Vertex and edge contacts, such as an octahedron
    landing on its tip, retain the GJK witness fallback instead of inventing a
    projected face plane.

## Scope and Fallbacks

- Face clipping is used for Box/Hull pairs while their GJK distance is within
  the existing speculative-contact threshold. Other pairs, degenerate support
  faces, and empty clips retain the single GJK witness fallback.
- Hull support faces are selected from retained QuickHull triangles and merged
  by coplanarity before clipping. Directly assembled `Convex` values without
  topology retain the co-planar-point fallback for test and tool compatibility.
- The device implementation shares the same header-only clipping code. It has
  a fixed 64-vertex working limit and falls back when a support face is
  degenerate.

## Verification

Release build, CUDA enabled:

- `cmake -S . -B build && cmake --build build --config Release -j 8`: passed.
- `ctest --test-dir build -C Release --output-on-failure`: passed
  `velox.stress`, `velox.fuzz`, and `velox.soak`.
- `build/examples/Release/proto_manifold.exe`: passed all eight checks,
  including CPU/CUDA clipped-manifold parity.
- `build/examples/Release/stress_demo.exe`: passed, including the 10-box tower.
- `build/examples/Release/fuzz_demo.exe 80`: passed twice consecutively.

`proto_manifold` checks a four-point Box-on-Box manifold, exact witness-plane
gap, partial overlap, hull contact, bitwise deterministic output, feature-key
persistence through a tangential slide, topology-backed hull clipping, 180
frames of a warm-started box resting with exactly three solver iterations, and
CPU/CUDA agreement on the active four-contact manifold and its 60-frame motion.

## Benchmark

Two-step Release benchmark, milliseconds per step. The pre-review run and this
run use the same command: `build/examples/Release/benchmark.exe 2`.

| Scene | CPU mode | Before | After | Delta |
| --- | --- | ---: | ---: | ---: |
| Rain, 512 spheres | cpu-1 | 0.831 | 0.816 | -1.8% |
| Rain, 512 spheres | cpu-auto | 0.590 | 0.607 | +2.9% |
| Rain, 2048 spheres | cpu-1 | 4.251 | 4.190 | -1.4% |
| Rain, 2048 spheres | cpu-auto | 3.186 | 3.058 | -4.0% |
| Rain, 8192 spheres | cpu-1 | 17.574 | 16.873 | -4.0% |
| Rain, 8192 spheres | cpu-auto | 11.704 | 11.331 | -3.2% |
| Terrain, 2048 spheres | cpu-1 | 0.969 | 0.701 | -27.7% |
| Terrain, 2048 spheres | cpu-auto | 0.844 | 0.738 | -12.6% |

No CPU regression exceeded 10% in the initial comparison.

The benchmark executable now restores an identical snapshot for every sample
and reports the median. Its default is 10 steps and 3 samples (30 timed steps
per configuration, versus the former two-step smoke run); larger values remain
available through `benchmark.exe <steps> <samples>`. The current 10x3 median
results are 0.804/0.720 ms for 512-sphere CPU rain, 4.078/2.921 ms for
2048-sphere CPU rain, 17.041/11.367 ms for 8192-sphere CPU rain, and
0.834/0.901 ms for CPU terrain (single-worker/auto-worker respectively).

## Merge Recommendation

Ready to merge after normal review. The required behavior and regression gates
pass, CUDA compilation succeeds, and runtime hull face topology now backs the
stable clipping identifiers.
