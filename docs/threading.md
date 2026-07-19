# Threading Contract

`World` is created with `ThreadSafetyPolicy::Strict`. In this mode every
method must be called from the thread that constructed the world. This is the
lowest-overhead policy for a conventional game loop.

## Policies

| Policy | Queries from another thread | Mutations from another thread | `step()` |
| --- | --- | --- | --- |
| `Strict` | rejected with `std::logic_error` | rejected with `std::logic_error` | owner thread only |
| `Relaxed` | allowed and serialized | rejected with `std::logic_error` | owner thread only |
| `Concurrent` | allowed and serialized | allowed and serialized | owner thread only |

The implementation uses one reentrant world lock. Calls that overlap a step
wait for the step to finish, then observe a complete state. It intentionally
does not expose partially-solved transforms or broad-phase state. This is a
correctness contract for direct reads.

`submitAsyncQuery()` is the exception: it copies a value-owned request under
its own queue mutex and returns immediately from any thread, even under
`Strict`. The owner resolves it at the start of a later `step()`; use
`getAsyncResult()` to wait for and consume the value-owned result. This never
exposes partially solved state. See [batched and async queries](batched-queries.md)
for the frame-boundary contract.

Use `ThreadSafetyPolicy::Concurrent` before handing a world to worker code:

```cpp
velox::World world;
world.setThreadSafetyPolicy(velox::ThreadSafetyPolicy::Concurrent);

// A worker may now call rayCast(), overlap*, *Cast(), bodyState(), and the
// supported mutation methods while the owner thread calls step().
```

## Borrowed References and Public Fields

`body()`, `joint()`, `lastStepStats()`, `contactEvents()`, and
`jointBreakEvents()` return borrowed references for source compatibility. A
lock cannot remain held after such a method returns, so callers must provide
external synchronization before retaining or dereferencing those references.
Prefer `bodyState()`, `jointState()`, and `lastStepStatsCopy()` for
cross-thread inspection.

For the same compatibility reason, the public `gravity` and `substeps` fields
must be configured before the world is shared. Use `setGravity()`,
`gravityValue()`, `setSubsteps()`, and `substepCount()` after it is shared.

`threadSafetyReport()` provides counters for rejected mutations, rejected
foreign step calls, and all cross-thread query calls. It is useful for
asserting the intended access pattern in tests; it is not a profiler.

The eight-worker contract regression runs under Clang ThreadSanitizer in CI.
CUDA is disabled for that job because NVIDIA's CUDA toolchain is not supported
by ThreadSanitizer.

World destruction and direct access through any borrowed reference always
require external synchronization.
