# Thread-Safety Contract Notes

## Design

Implemented roadmap item 05 as a correctness-first synchronization boundary.
`World` has an owner thread and a reentrant internal lock. Strict is the
default; Relaxed permits cross-thread query calls; Concurrent permits both
queries and supported mutations. `step()` remains owner-thread-only in every
policy. Calls that overlap a step serialize behind the step rather than reading
solver-owned arrays while they are being written.

The original sketch's claim that queries can read during a solver pass was not
adopted: the current CPU solver updates body state in place, so that design
would be a C++ data race. A nonblocking query snapshot API remains future
work (roadmap item 16).

## Compatibility Boundary

`body()`, `joint()`, event accessors, and `lastStepStats()` retain their
reference-returning API for compatibility but cannot be made safe after their
short-lived method lock is released. New `bodyState()`, `jointState()`, and
`lastStepStatsCopy()` return safe copies. Direct writes to the historical
public `gravity` and `substeps` fields are likewise externally synchronized;
new guarded setters/getters cover shared worlds.

## Verification

- Added `velox.thread_safety`: strict rejection, relaxed query access,
  concurrent eight-worker query traffic during 240 owner-thread steps,
  concurrent mutation serialization, report counters, and copied body state.
- CUDA-enabled Release build completed successfully.
- Full CUDA-enabled Release CTest gate passed: all 13 suites, including
  stress, fuzz, soak, geometry fuzz, determinism, thread safety, the real
  workload, character controller, sandbox self-test, and Jolt differential
  testing (35.45 seconds).

The CI workflow now configures a CPU-only Clang ThreadSanitizer build and runs
the eight-worker regression. It remains pending hosted-CI evidence; MSVC does
not provide a supported ThreadSanitizer configuration for this build.

## Merge Recommendation

Ready for review. The serialized policy is appropriate for production
correctness today; high-throughput nonblocking async query snapshots should be
delivered separately.
