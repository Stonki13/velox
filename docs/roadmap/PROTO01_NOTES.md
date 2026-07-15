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

## Scope and Fallbacks

- Face clipping is used for Box/Hull pairs while their GJK distance is within
  the existing speculative-contact threshold. Other pairs, degenerate support
  faces, and empty clips retain the single GJK witness fallback.
- Hull faces are derived from the co-planar support vertices already retained
  in the runtime hull point cloud. The current runtime body format does not
  retain QuickHull triangle topology; a future public manifold API can store
  those face indices if callers need to inspect them.
- The device implementation shares the same header-only clipping code. It has
  a fixed 64-vertex working limit and falls back when a support face is
  degenerate.

## Verification

Release build, CUDA enabled:

- `cmake -S . -B build && cmake --build build --config Release -j 8`: passed.
- `ctest --test-dir build -C Release --output-on-failure`: passed
  `velox.stress`, `velox.fuzz`, and `velox.soak`.
- `build/examples/Release/proto_manifold.exe`: passed all seven checks.
- `build/examples/Release/stress_demo.exe`: passed, including the 10-box tower.
- `build/examples/Release/fuzz_demo.exe 80`: passed twice consecutively.

`proto_manifold` checks a four-point Box-on-Box manifold, exact witness-plane
gap, partial overlap, hull contact, bitwise deterministic output, feature-key
persistence through a tangential slide, and 180 frames of a warm-started box
resting with exactly three solver iterations.

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

No CPU regression exceeded 10%. The two-step measurement is intended as a
smoke comparison, not a statistically stable performance benchmark.

## Merge Recommendation

Ready to merge after normal review. The required behavior and regression gates
pass, CUDA compilation succeeds, and the remaining Hull topology limitation is
documented rather than hidden behind unstable identifiers.
