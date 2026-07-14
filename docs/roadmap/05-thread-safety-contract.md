# 05 — Thread-Safety Contract

## Goal

Document and enforce the threading rules for Velox so that users can safely run queries from worker threads while the main thread is stepping, without data races or undefined behavior. The contract must be simple enough to remember, enforced by assertions in debug builds, and performant enough that release builds pay zero overhead.

## Public API

```cpp
namespace velox {

// Threading policy for World access. Determines which methods are safe from
// which threads at what times.
enum class ThreadSafetyPolicy : uint8_t {
    Strict = 0,       // only main thread touches World; queries blocked elsewhere
    Relaxed = 1,      // const methods (queries) safe from any thread during step()
    Concurrent = 2    // full concurrent access; mutation serialized via internal lock
};

// Runtime threading diagnostics. Populated in debug builds when policy != Strict.
struct ThreadSafetyReport {
    uint64_t queryCallsFromNonMainThread = 0;
    uint64_t mutationCallsDuringStep = 0;     // assertion failures caught
    uint64_t stepInvocationsOnNonMainThread = 0;
};

// World threading configuration:
ThreadSafetyPolicy threadSafetyPolicy() const;
void setThreadSafetyPolicy(ThreadSafetyPolicy policy);
ThreadSafetyReport threadSafetyReport() const;  // debug builds only

} // namespace velox
```

## Data structures

- `ThreadSafetyPolicy` enum — new, lives in `include/velox/world.h`.
- `ThreadSafetyReport` struct — new, lives in `include/velox/world.h`, behind `#ifdef VELOX_DEBUG` guard.
- `World::threadSafetyPolicy_` member — private field on World.
- `World::stepGeneration_` atomic counter — tracks the current step's generation; queries check this to verify they're not mid-step when mutating.
- `World::mutationLock_` — `std::mutex` (Concurrent policy only); no-op in Strict/Relaxed.

## Algorithm

**Threading rules (documented contract):**

1. **`step(float dt)` must be called from exactly one thread.** No concurrent step invocations. The World serializes internally via an atomic step-generation counter; a second thread calling `step()` during an in-progress step will block or assert-fail depending on policy.
2. **Const methods are thread-safe from any thread** when the World is not mid-step: `rayCast`, `rayCastAll`, `overlapSphere`, `overlapBox`, `overlapCapsule`, `overlapConvexHull`, `sphereCast`, `boxCast`, `capsuleCast`, `convexHullCast`, `closestPoints`, `debugLines`, `body() const`, `joint() const`. These only read body/joint state and broad-phase structure, which are stable between step invocations.
3. **Mutation methods require the calling thread to be the step thread** (Strict policy) or acquire an internal mutex (Concurrent policy). Mutation methods: `addSphere`, `addBox`, `removeBody`, `setTransform`, `setLinearVelocity`, `addForce`, etc.
4. **Queries during step() are safe from other threads** in Relaxed/Concurrent policies because the broad phase and body arrays are read-only during the solver pass (the CPU backend uses deterministic batch ordering; CUDA runs on device memory).

**Enforcement mechanism:**

1. At the start of `step()`, increment `stepGeneration_` atomically and set a flag `isStepping_ = true`.
2. Every mutation method checks: if `isStepping_ && policy == Strict`, assert-fail with source location. If `policy == Concurrent`, acquire `mutationLock_` (which blocks until step completes).
3. Every const query method checks: if `policy == Strict && std::this_thread::get_id() != mainThreadId_`, assert-fail. In Relaxed/Concurrent, no check needed (zero overhead in release builds via compile-time constant folding).
4. At the end of `step()`, set `isStepping_ = false`.

**Debug-only instrumentation:**

1. Each policy violation increments a counter in `ThreadSafetyReport` and logs to `VELOX_ASSERT` with the calling thread ID and method name.
2. `threadSafetyReport()` returns accumulated counts since last reset; useful for CI validation that no policy violations occur during stress tests.

## Files

**New files:**
- None (all changes are additions to existing headers)

**Modified files:**
- `include/velox/world.h` — add `ThreadSafetyPolicy`, `ThreadSafetyReport`, accessors, internal members
- `src/world.cpp` — implement threading checks in mutation methods; wrap step() with generation counter
- `cmake/threads.cmake` — document threading requirements (new cmake snippet, not modifying CMakeLists.txt directly)

## Tests

1. **Concurrent query during step:** Spawn 8 worker threads that each call `rayCast` 1000 times against a World that is simultaneously being stepped by the main thread. Must complete without data races (verify with ASan ThreadSanitizer).
2. **Mutation during step rejected:** Call `addSphere()` from a non-main thread while `step()` is in progress with Strict policy. Must assert-fail (debug build) or block until step completes (Concurrent policy).
3. **Policy switch at runtime:** Start with Strict, call `setThreadSafetyPolicy(Relaxed)` before any queries; subsequent cross-thread queries must succeed without assertion.
4. **No release overhead:** Compile in Release mode; verify that const method calls compile to the same assembly as if no threading checks existed (zero branch instructions for policy == Relaxed).

## Acceptance

- [ ] `ThreadSafetyPolicy` enum with Strict/Relaxed/Concurrent documented
- [ ] Const query methods are safe from any thread when World is not mid-step
- [ ] Mutation during step() is rejected or serialized per policy
- [ ] ThreadSanitizer passes on the 8-thread concurrent query test
- [ ] Release builds have zero overhead for const method calls (verified via assembly diff)
- [ ] `ThreadSafetyReport` accumulates violations in debug builds

## Size: S

## Risks

- The "const methods are safe during step" claim depends on the CPU backend's deterministic batch ordering not being invalidated by concurrent reads. If a worker thread reads body state while the solver is writing velocities, data races occur. Must verify that the solver writes to a shadow array and swaps at the end of each substep.
- `std::mutex` in Concurrent policy adds latency to every mutation call. For high-frequency mutation scenarios (thousands of bodies added/removed per frame), this becomes a bottleneck. Consider lock-free queues for batched mutations.
- ThreadSanitizer has known false positives on atomic counters; the concurrent query test may need `__attribute__((no_sanitize("thread")))` annotations on specific methods.
