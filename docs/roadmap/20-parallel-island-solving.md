# 20 — Parallel Island Solving

## Goal

Solve independent contact islands on the CPU worker pool in parallel. Currently, the CPU backend solves contacts sequentially (even with deterministic batching). By identifying islands that share no dynamic bodies, we can solve multiple islands concurrently without changing the mathematical result — each island is an independent Gauss-Seidel system.

## Public API

```cpp
namespace velox {

enum class IslandSolvingMode : uint8_t {
    Sequential = 0,       // current behavior: one thread solves all islands
    Parallel = 1          // solve independent islands on the worker pool
};

class World {
public:
    IslandSolvingMode islandSolvingMode() const;
    void setIslandSolvingMode(IslandSolvingMode mode);
};

} // namespace velox
```

## Data structures

- `IslandSolvingMode` enum — new, lives in `include/velox/world.h`.
- `World::islandSolvingMode_` member — added to World.

## Algorithm

1. **Island identification:** After building candidate pairs and contacts, the existing union-find island labeling assigns each dynamic body to an island ID. Islands are independent if they share no bodies.
2. **Work distribution:** Collect all islands into a list. Each island is a self-contained set of bodies and contacts that can be solved independently.
3. **Worker pool dispatch:** Submit each island as a task to the existing CPU worker pool. The worker solves the island's contacts using the standard sequential-impulse solver (no changes to the solver math).
4. **Synchronization:** After all islands are solved, the main thread merges results (body velocities, contact impulses) back into the global arrays. Since islands share no bodies, no race conditions occur.

**Task granularity:**

- Small islands (≤ 4 bodies) have less work than the overhead of task dispatch. Threshold: islands with ≤ 8 contacts are solved sequentially on the main thread; larger islands are dispatched to workers.
- This avoids worker pool contention for scenes with many small, independent contacts (e.g., scattered debris).

## Files

- `include/velox/world.h` — add IslandSolvingMode enum and accessor methods
- `src/solver.cpp` — modify the solve loop to dispatch islands to the worker pool
- `tests/parallel_islands.cpp` — test file

## Tests

1. **Determinism:** Run a scene with 100 independent islands (no shared bodies) in Sequential vs Parallel mode. Body positions after 1000 frames must match exactly (bitwise identical).
2. **Speedup measurement:** 50 islands of 20 bodies each, solved in parallel on an 8-core machine. Wall-clock solver time should be ≤ 1/4 of sequential time (limited by overhead and small island sizes).
3. **Mixed workload:** Scene with 10 large islands (50 bodies each) and 90 small islands (2 bodies each). Large islands run in parallel; small islands run sequentially. Total time < sequential total.

## Acceptance

- [ ] Parallel mode produces bit-identical results to sequential mode
- [ ] Speedup ≥ 2× on an 8-core machine for scenes with ≥ 10 independent islands
- [ ] Small islands (≤ 8 contacts) are not dispatched to the worker pool (verified via profiling)
- [ ] `setIslandSolvingMode()` can be changed at runtime between steps

## Size: M

## Risks

- The CPU worker pool is currently designed for deterministic narrow-phase pair batches. Adding solver tasks requires ensuring that task submission and completion don't interfere with the broad-phase refit that runs concurrently.
- Island identification uses union-find over contacts + joints. If a joint connects two bodies that are in different contact islands, they must be merged into one island. Must validate this invariant before dispatching.
- Debugging parallel solver bugs is harder than sequential. Must provide a `VELOX_DEBUG_SERIAL_SOLVER` compile flag that forces sequential solving for debugging.
