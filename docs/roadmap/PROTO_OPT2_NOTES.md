# PROTO_OPT2 - CPU Narrowphase Optimization: Review Status

## Status

Implemented on `proto/optimize2`. The CPU backend now bypasses generic GJK
for ordinary sphere-sphere candidate pairs. The optimization is deliberately
host-only: shared `VELOX_HD` narrowphase code and the CUDA backend remain
unchanged.

## Profile First

Profiling used a disposable `tests/difftest/diag.cpp` that constructs the
8192-sphere rain scene, warms it for five 60 Hz frames, then averages ten
frames from `World::lastStepStats()`. At that point the scene has no generated
contacts, which isolates candidate rejection rather than solver cost.

| CPU mode | Total ms | Setup ms | Detection ms | Solver ms | Contacts |
| --- | ---: | ---: | ---: | ---: | ---: |
| Before, 1 worker | 22.766 | 0.783 | 20.768 | 1.165 | 0 |
| Before, 16 workers | 8.391 | 0.575 | 6.912 | 0.855 | 0 |
| After, 1 worker (median of 3) | 14.453 | 0.605 | 12.820 | 0.984 | 0 |
| After, 16 workers (median of 3) | 7.118 | 0.611 | 5.608 | 0.839 | 0 |

Collision detection was the measured bottleneck (82% of the first parallel
sample). The solver was not changed. The scratch diagnostic was restored
before this commit.

## Change

`src/solver.cpp` adds a CPU-only sphere-pair path inside
`CpuBackend::findContacts`:

- Fat AABB leaves produce many separated sphere candidates. A conservative
  squared-distance check rejects pairs beyond speculative reach without a GJK
  call. A round-off band leaves threshold-adjacent pairs on the exact path.
- Pairs inside that band use the analytic point-core result that generic GJK
  already computes for non-coincident spheres, then call the same
  `np_detail::emit` function for anchors, material combination, feature keys,
  and speculative-contact filtering.
- Near-coincident sphere cores retain generic GJK, preserving its existing
  deterministic fallback normal and behavior.

A scratch comparison covered 4,913 deterministic sphere configurations and
found byte-identical generated `Contact` records between the analytic path and
generic GJK outside that preserved near-coincident fallback.

## Benchmark

`examples/benchmark.exe 15 5`, median ms/step. These are before/after runs on
the same RTX 5080 / 16-core Windows host. Timing noise is visible on the small
and CUDA-bound scenes; no CPU scene regressed by 10% or more.

| Scene | CPU-1 before | CPU-1 after | CPU-auto before | CPU-auto after | CUDA before | CUDA after |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Rain, 512 spheres | 0.914 | 0.908 | 0.633 | 0.661 | 2.316 | 2.278 |
| Rain, 2048 spheres | 5.032 | 4.793 | 2.196 | 2.066 | 3.997 | 2.955 |
| Rain, 8192 spheres | 21.662 | 19.709 | 8.279 | 8.217 | 9.166 | 9.189 |
| Terrain, 2048 spheres | 1.217 | 1.194 | 0.912 | 0.835 | 2.601 | 2.648 |
| Joints, 64 | 0.145 | 0.136 | 0.150 | 0.142 | 4.225 | 4.027 |
| Joints, 256 | 0.583 | 0.551 | 0.593 | 0.534 | 3.308 | 4.481 |
| Joints, 1024 | 2.271 | 2.247 | 2.486 | 2.348 | 4.586 | 4.174 |
| Joints, 4096 | 9.199 | 9.266 | 9.230 | 8.917 | 8.826 | 8.819 |
| Mesh archipelago | 1.030 | 1.025 | 0.870 | 0.863 | 11.206 | 11.362 |

The meaningful affected result is 8192-sphere rain on one worker:
`21.662 -> 19.709 ms/step` (-9.0%). The auto-worker result is approximately
neutral in this short benchmark (`8.279 -> 8.217 ms/step`), while the
phase profile shows collision detection improving from 6.912 to 5.608 ms.
Terrain also improves on both CPU modes. CUDA results are unchanged apart from
normal run-to-run variance because the CUDA source path was not modified.

## Verification

- Configured with `VELOX_BUILD_SANDBOX=ON` and `VELOX_BUILD_DIFFTEST=ON` and
  built Release with CUDA enabled.
- All 10 CTest suites passed: stress, fuzz, soak, islands (bitwise
  determinism), vehicle, serialize, geometry fuzz, character, sandbox
  self-test, and Jolt differential testing.
- `fuzz_demo.exe 80` passed twice consecutively (zero failures).
- `proto_manifold.exe` passed all eight checks, including CPU/CUDA parity.
- CUDA compilation remained enabled and completed in the Release build.

## Deviations

None. CUDA contact generation remains on the common shared implementation;
the optimization is intentionally limited to CPU candidate processing because
the profile identified that path as the immediate bottleneck.

## Merge Recommendation

Ready to merge after normal review. The change is small, preserves contact
records for its optimized domain, leaves CUDA untouched, and passes the full
determinism, fuzz, manifold, and differential gates.
