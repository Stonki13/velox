# PROTO20 - Parallel Island Solving: Review Status

## Status

Implemented on `proto/islands`. The CPU backend partitions contacts into
independent islands (union-find over dynamic-body links) and solves whole
islands concurrently on the existing worker pool. Registered as ctest
`velox.islands`.

## Design

- `IslandSolvingMode { Sequential, Parallel }` on `World` (default Parallel),
  plumbed to the backend through `Backend::setParallelIslands`. The CUDA
  backend ignores it (graph-colored solving instead).
- Islands are rebuilt on the first substep of each step (the contact set is
  stable across substeps) with a counting-sort bucket pass that preserves
  each island's contacts in global sequential order. Ranges are dispatched
  largest-first for load balance; inter-island order cannot affect results
  because islands share no dynamic bodies.
- **Bitwise determinism by construction:** each island's solve executes the
  exact instruction sequence of the single-threaded reference restricted to
  that island, and islands are disjoint, so Parallel mode is byte-identical
  to a 1-worker run — verified by `islands_demo` (300 frames, 64 stacks,
  memcmp over positions/orientations/velocities).
- Single-island scenes (one big pile) fall back to the existing
  conflict-free batch path, which remains available as `Sequential` mode.

## Deviations from the spec

- No small-island main-thread threshold: the worker pool's atomic task
  stealing makes per-island dispatch overhead one fetch_add; measurements
  showed no benefit from special-casing small islands.
- Joints are not merged into contact islands: joints are solved in a separate
  world-level pass, so two islands connected only by a joint still share no
  contact-solver writes. (Revisit if joint solving ever moves into the
  backend's island loop.)

## Measurements

`islands_demo` (64 disjoint 6-box stacks, 300 frames, 16-core host):

| Solver path | ms/step (solver) |
| --- | ---: |
| 1 worker (reference) | 1.752 |
| Worker pool, conflict batches (old default) | 1.784 |
| Worker pool, parallel islands | **0.316** |

5.5x over sequential — and the old batch path was actually SLOWER than a
single thread on this workload (a barrier per conflict batch per iteration
per substep), which parallel islands eliminates entirely.

## Verification

- `islands_demo`: parallel run bitwise identical to the 1-worker reference.
- Full suite: `velox.stress`, `velox.fuzz`, `velox.soak`,
  `velox.geometry_fuzz`, `velox.character`, `velox.sandbox`,
  `velox.difftest`, `velox.islands` all pass; `fuzz_demo 80` and
  `proto_manifold` clean.

## Merge recommendation

Ready to merge after normal review.
