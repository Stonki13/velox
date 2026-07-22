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

## Thread-Safety Utilities (`<velox/thread_safety.h>`)

Velox ships a small, header-only concurrency toolkit for code that shares data
with a `World` or runs its own workers. It depends only on the C++17 standard
library and is included by the `<velox/velox.h>` umbrella header.

### Thread-safe wrapper types

| Type | Purpose |
| --- | --- |
| `SpinLock` | A `BasicLockable` test-and-set spinlock for critical sections that are a handful of instructions long. Works with `std::lock_guard`. Never hold it across a blocking call; use `std::mutex` for anything that can stall. |
| `Atomic<T>` | Ergonomic wrapper over `std::atomic<T>` with explicit memory orders and integral-only arithmetic (`fetchAdd`, `fetchOr`, …). |
| `AtomicFlag` | A one-shot latch. `testAndSet()` returns the previous value, so exactly one caller observes `false` — a "claim once" primitive. |
| `AtomicCounter<T>` | A contention-friendly counter with `increment()`/`decrement()` returning the new value. |
| `MutexGuarded<T>` | Bundles a value with the mutex that protects it. The value is reachable only through `lock()` (an RAII `LockedPtr`) or `withLock(fn)`, so the data and its lock cannot drift apart. |

```cpp
velox::MutexGuarded<std::vector<velox::BodyId>> pending;
pending.withLock([](auto& v) { v.push_back(id); });
{
    auto locked = pending.lock(); // holds the lock for this scope
    for (auto id : *locked) { /* ... */ }
} // unlocked here
```

### Lock-free structures for hot paths

| Type | Guarantees | Use when |
| --- | --- | --- |
| `SpscQueue<T, Capacity>` | Wait-free push and pop | Exactly one producer and one consumer (e.g. simulation thread → render thread). The fastest option. |
| `MpmcQueue<T, Capacity>` | Lock-free, ABA-safe (Vyukov sequence counters); `Capacity` must be a power of two | Any number of producers and consumers. |

Both are bounded fixed-capacity ring buffers; `tryPush`/`tryPop` return `false`
when full/empty rather than blocking. Pair them with `velox::Backoff` to avoid
spinning a core at 100% while waiting:

```cpp
velox::SpscQueue<velox::BodyId, 1024> queue;
// producer
while (!queue.tryPush(id)) velox::Backoff::cpuRelax();
// consumer
velox::BodyId id;
while (!queue.tryPop(id)) velox::Backoff::cpuRelax();
```

### Thread-local storage helpers

| Type | Purpose |
| --- | --- |
| `ThreadLocal<T>` | Per-instance, per-thread storage. Each thread gets its own `T` via `getOrCreate()`; `forEach(fn)` visits every thread's value at a frame boundary. |
| `ThreadLocalAccumulator<T>` | A per-thread counter threads bump with zero contention; `total()` sums every contribution. Ideal for statistics that would otherwise serialize on one shared counter. |

`ThreadLocal<T>` favors portability over raw speed: a single-slot lock-free
cache serves the steady-state "same thread as last time" case, but the storage
is intended for per-thread scratch touched a modest number of times per frame —
not the innermost solver loop. For that, capture a reference once per frame and
reuse it.

### Compile-time annotations

The header also defines Clang Thread Safety Analysis macros
(`VELOX_GUARDED_BY`, `VELOX_CAPABILITY`, `VELOX_REQUIRES`, …). They expand to
nothing unless a translation unit is compiled with `-Wthread-safety`, so they
never change codegen or break a build on MSVC/GCC. Velox uses them internally
to document which lock guards which member (for example, the async-query
bookkeeping in `World` is `VELOX_GUARDED_BY(asyncQueryMutex_)`). Enable
`-Wthread-safety` in your own build to have the compiler statically verify that
guarded data is only touched while its lock is held.

## Best Practices

1. **Pick the cheapest policy that is correct.** `Strict` (the default) has
   zero synchronization overhead and is right for a conventional single-thread
   game loop. Only move to `Relaxed` (cross-thread queries) or `Concurrent`
   (cross-thread queries *and* mutations) when you actually share the world.

2. **`step()` is owner-thread only, always.** No policy permits `step()` from a
   foreign thread. Run it on the thread that constructed the world; let workers
   issue queries and (under `Concurrent`) supported mutations around it.

3. **Prefer value copies across threads.** `bodyState()`, `jointState()`, and
   `lastStepStatsCopy()` return owned snapshots that stay valid after the world
   advances. The borrowed references from `body()`, `joint()`,
   `lastStepStats()`, `contactEvents()`, and `jointBreakEvents()` require
   external synchronization to retain or dereference.

4. **Configure shared fields through the setters.** Set `gravity` and
   `substeps` via `setGravity()`/`setSubsteps()` once a world is published to
   another thread; the public fields are not internally synchronized.

5. **Use `submitAsyncQuery()` for fire-and-forget reads from any thread.** It
   copies a value-owned request under its own queue mutex and never exposes
   partially-solved state — it is safe even under `Strict`. Resolve results
   with `getAsyncResult()`.

6. **Do not hold a Velox lock across your own blocking calls.** The world lock
   is reentrant only on the owner thread; a worker that blocks while holding
   query serialization can stall the owner's `step()`.

7. **Match the queue to the producer/consumer shape.** One producer + one
   consumer → `SpscQueue`. Many of either → `MpmcQueue` (power-of-two
   capacity). Always back off with `velox::Backoff` instead of a bare spin.

8. **Keep per-thread counters off the shared cacheline.** Use
   `ThreadLocalAccumulator` for statistics each worker bumps every frame, and
   read `total()` once at the frame boundary.

9. **Verify with ThreadSanitizer.** The concurrent-access contract regression
   runs under Clang ThreadSanitizer in CI (CUDA disabled, as NVIDIA's toolchain
   is unsupported by TSan). Run your own concurrency tests under TSan to catch
   races that functional tests may not surface.
