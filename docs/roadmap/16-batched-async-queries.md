# 16 — Batched Async Queries

## Goal

Provide a batched query API that can submit multiple raycasts/overlaps/shapecasts in a single call, and an async contract that lets worker threads issue queries against the World without blocking the main simulation thread. Queries execute on the broad-phase's AABB tree and are inherently read-only, making them safe from any thread when the World is not mid-step (see item 05).

## Public API

```cpp
namespace velox {

// A single query in a batch. Type determines which parameters are valid.
struct QueryDesc {
    enum class Type { Raycast, RaycastAll, OverlapSphere, OverlapBox,
                      SphereCast, BoxCast, CapsuleCast };
    Type type;
    QueryFilter filter{};

    // Parameters depend on type (union-like):
    Vec3 origin{}, direction{};       // raycast, casts
    float maxDist = 1e10f;            // raycast, casts
    Vec3 center{};                    // overlap sphere/box/capsule
    Vec3 halfExtents{};               // overlap box
    Quat orientation{};               // overlap box/capsule, cast orientation
    float radius = 0.0f;              // overlap sphere, sphere cast
    float capsuleHalfHeight = 0.0f;   // overlap capsule

    void* userData = nullptr;         // echoed back in the result
};

// Result of a single query in a batch.
struct QueryResult {
    bool success = false;             // query executed without error
    QueryDesc::Type type;
    void* userData = nullptr;

    union {
        RayHit rayHit;                // for Raycast, SphereCast, etc.
        std::vector<BodyId> overlaps; // for OverlapSphere, OverlapBox, etc.
    };
};

// Submit a batch of queries. All execute synchronously and return when complete.
// Safe to call from any thread (see ThreadSafetyPolicy).
void batchQueries(const std::vector<QueryDesc>& queries,
                  std::vector<QueryResult>& outResults);

// Async query handle: returned immediately, resolved later via getAsyncResult().
struct AsyncQueryHandle {
    uint64_t id;
};

// Submit a single query asynchronously. Returns a handle for later resolution.
AsyncQueryHandle submitAsyncQuery(const QueryDesc& query);

// Block until the async query completes and return its result.
QueryResult getAsyncResult(World& world, AsyncQueryHandle handle);

} // namespace velox
```

## Data structures

- `QueryDesc`, `QueryResult` — new file `include/velox/queries.h`.
- `AsyncQueryHandle` — new file `include/velox/queries.h`.
- `World::batchQueries()` — new method on World.
- `World::submitAsyncQuery()`, `getAsyncResult()` — new methods on World.

## Algorithm

**Batched queries:**

1. Group queries by type (raycast, overlap, cast) to maximize reuse of broad-phase traversal.
2. For each group, run a single broad-phase traversal that collects all candidate bodies, then execute narrow-phase tests in order.
3. This avoids rebuilding the candidate set for each individual query — a batch of 100 raycasts from similar origins shares most of the AABB tree traversal.

**Async contract:**

1. `submitAsyncQuery()` copies the QueryDesc into an internal queue and returns a handle immediately (zero wait).
2. The main simulation thread drains the async queue at the start of each step(), executing queries before stepping.
3. `getAsyncResult()` blocks the calling thread until the query is processed. If called from the main thread after step() returns, the result is already available (no blocking).

## Files

- `include/velox/queries.h` — new header with batched/async API
- `src/queries.cpp` — implement batch execution and async queue
- `tests/batched_queries.cpp` — test file

## Tests

1. **Batch vs individual parity:** Submit 50 raycasts as a batch; verify results match calling `rayCast()` 50 times individually (same order, same hits).
2. **Async non-blocking:** Call `submitAsyncQuery()` from a worker thread while main thread is in `step()`. The worker thread must not block; result available after step returns.
3. **Batch performance:** 100 sphere overlaps against 10,000 bodies. Batched call takes < 20% of the time of 100 individual calls (measured on reference hardware).

## Acceptance

- [ ] `batchQueries()` executes all queries and returns results in the same order as input
- [ ] Async query handle is returned immediately without blocking
- [ ] `getAsyncResult()` blocks only if the query hasn't been processed yet
- [ ] Batch performance improvement ≥ 5× for 100+ similar queries

## Size: S

## Risks

- The async queue must be drained at a deterministic point in the step loop; if drained mid-solver, it could interfere with warm starting. Must drain before collision detection starts each step.
- Batched query ordering must match individual query ordering exactly. Any reordering (e.g., grouping by type) must preserve the output order via the `userData` echo or result indexing.
