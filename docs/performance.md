# Performance Guide

How Velox spends its time, how that cost scales with scene size, and how to
measure and tune it. This guide pairs with the benchmark suite in
[`benchmarks/`](../benchmarks) and the runner in
[`scripts/run_benchmarks.py`](../scripts/run_benchmarks.py); together they make
performance measurable and trackable across changes.

For the solver design behind these numbers, see [concepts](concepts.md); for
threading behaviour, see [threading](threading.md).

---

## Anatomy of a step

Every `World::step(dt)` runs the same pipeline. The engine times each stage and
exposes the breakdown through `World::lastStepStats()` (`StepStats`), which is
the single source of truth the benchmarks read from:

| Stage             | `StepStats` field        | What it does                                                        |
| ----------------- | ------------------------ | ------------------------------------------------------------------- |
| Setup             | `setupMs`                | Integrate forces, predict motion, rebuild island bookkeeping.       |
| Broad-phase       | `broadPhaseMs`           | Update AABB proxies and emit candidate overlapping pairs.           |
| Narrow-phase      | `narrowPhaseMs`          | Test candidate pairs and generate contact manifolds.                |
| Contact solver    | `contactSolverMs`        | Resolve contact constraints (velocity + position).                  |
| Joint solver      | `jointSolverMs`          | Resolve joint constraints.                                          |
| CCD               | `ccdMs`                  | Continuous-collision recovery for fast bodies.                      |
| Finalize          | `finalizeMs`             | Integrate positions, emit events, publish results.                  |
| **Total**         | `totalMs`                | Whole-step wall time.                                               |

Counters that explain the timing: `broadPhaseProxies`, `narrowPhaseTests`,
`generatedContacts`, `solvedContacts`, `islandCount`, `awakeDynamicBodies`.

> **Tip:** when a step is slow, read `lastStepStats()` first. The dominant
> field tells you which stage to optimise - do not guess.

---

## Performance characteristics

### Broad-phase
Cost is driven by **proxy count** and **spatial density**, not directly by body
count. A sparse field of thousands of static boxes is cheap because almost no
pairs overlap; the same count packed into a small volume is expensive because
the candidate-pair count grows. Measured by
[`benchmark_broadphase.cpp`](../benchmarks/benchmark_broadphase.cpp).

- Sparse, well-separated bodies: near-linear in body count, small constant.
- Dense piles: pair generation dominates and grows faster than linearly with
  local density.

### Narrow-phase
Cost scales with the number of **candidate pairs** the broad-phase hands it and
the **primitive pair type**. Convex-vs-convex (sphere/sphere, box/box) is the
fast path; convex-vs-triangle mesh goes through a BVH and costs more per pair.
Measured by [`benchmark_narrowphase.cpp`](../benchmarks/benchmark_narrowphase.cpp).

### Constraint solver
The solver is usually the largest single cost in a contact-heavy scene. It
scales with `solvedContacts × velocityIterations` for contacts and with
`jointCount × iterations` for joints. Independent contact **islands** solve in
parallel on the worker pool, so wall time grows sub-linearly with core count
when the scene fragments into many islands. Measured by
[`benchmark_solver.cpp`](../benchmarks/benchmark_solver.cpp).

### Scene queries
Raycasts, overlaps, and shape casts run against the broad-phase acceleration
structure and are read-only (never mutate the world). Cost is logarithmic in
proxy count for a ray, and proportional to the number of proxies a volume
overlaps. Measured by [`benchmark_queries.cpp`](../benchmarks/benchmark_queries.cpp).

---

## Scaling behavior

[`benchmark_scaling.cpp`](../benchmarks/benchmark_scaling.cpp) sweeps body count
and reports `totalMs` plus the per-body cost (`msPerBody`). Interpret the curve
through the per-body metric:

- **Flat `msPerBody`** → linear scaling; the workload grows proportionally.
- **Falling `msPerBody`** → sub-linear; fixed overhead is amortised (or parallel
  island solving is winning as the scene fragments).
- **Rising `msPerBody`** → super-linear; pair density or solver coupling is
  growing faster than the body count - a signal to reduce contact density or
  enable sleeping.

Dense sphere piles tend toward super-linear in the contact stages because each
added body creates several new contacts; sparse scenes and joint fans stay
close to linear.

---

## Optimization tips

Ordered roughly by impact-per-effort:

1. **Let bodies sleep.** Sleeping bodies leave the awake set and skip collision
   and solving entirely. Keep `setEnableSleep(id, true)` (the default) and avoid
   needlessly waking bodies. Watch `awakeDynamicBodies`, not `bodyCount`.
2. **Reduce contact density.** Solver cost tracks `solvedContacts`. Use simpler
   colliders where detail is not needed, add spacing in piles, and prefer fewer
   larger shapes over many tiny ones.
3. **Tune the iteration budget.** `SolverOptions::velocityIterations` is the main
   quality/cost knob (default 8). The solver benchmark sweeps 2/4/8/16 so you can
   pick the lowest count that still looks stable for your scene.
4. **Use substeps for stiffness, not iterations.** `World::substeps` (default 4)
   gives stiffer stacks with less friction drift for the same iteration budget;
   raising it is often cheaper than raising iterations to fix sag.
5. **Prefer the parallel island solver.** `IslandSolvingMode::Parallel` solves
   independent islands concurrently; it is bitwise-identical to sequential and
   helps any scene that fragments into many islands (ragdolls, debris, joints).
6. **Match worker count to cores.** `setWorkerCount(0)` auto-sizes the pool.
   Pinning to 1 is only useful for determinism experiments or profiling.
7. **Keep queries batched.** When issuing many rays/overlaps per frame, use the
   batched query API (`batchQueries`) so they share one consistent world state
   and amortise traversal setup.
8. **Consider the GPU backend for very large dense scenes.** `BackendType::Auto`
   selects CUDA when available; it pays off for large dynamic/dynamic workloads
   and falls back to CPU automatically on failure. See
   [CUDA recovery](cuda-recovery.md).
9. **Keep the world near the origin.** Large coordinates erode floating-point
   precision and can widen AABBs; call `shiftOrigin()` to recenter big worlds.

---

## Running the benchmarks

Build the suite (on by default via `VELOX_BUILD_BENCHMARKS`) and run the
driver:

```bash
cmake -B build -DVELOX_BUILD_BENCHMARKS=ON
cmake --build build --config Release

# Human-readable report
python scripts/run_benchmarks.py --build-dir build

# Machine-readable export
python scripts/run_benchmarks.py --build-dir build \
    --json-out results.json --csv-out results.csv
```

Each benchmark is also a standalone executable that accepts `--json`, `--csv`,
`--warmup N`, `--iters N`, and `--baseline <file>`:

```bash
./build/benchmarks/Release/benchmark_solver --json
```

### Tracking regressions

Record a baseline on a known-good build, then compare subsequent runs against
it. Both the driver and the standalone executables exit non-zero when any
metric exceeds the threshold (default 10% slower):

```bash
# Record once
python scripts/run_benchmarks.py --build-dir build \
    --update-baseline benchmark_baseline.json

# Compare on every change (CI-friendly)
python scripts/run_benchmarks.py --build-dir build \
    --baseline benchmark_baseline.json --threshold 1.10
```

The legacy single-binary checker
[`scripts/benchmark_regression.py`](../scripts/benchmark_regression.py) remains
available for the older `examples/benchmark` executable and shares the same
baseline JSON schema.

### Reading the results

Each result carries `meanMs`, `medianMs`, `stddevMs`, `p95Ms`, `p99Ms`, and a
coefficient of variation `cv`. Prefer the **median** for comparisons (robust to
outliers) and watch `cv`: a value above ~0.2 means the measurement is noisy -
increase `--iters`, close background load, or pin CPU frequency before trusting
a regression call. `peakMemoryMiB` tracks the working-set cost of each scene.
