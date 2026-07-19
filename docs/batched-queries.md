# Batched and Async Queries

`World::batchQueries()` evaluates a list of value-owned `QueryDesc` requests
against one locked World state and returns results in the same order. A bad
request only fails its own `QueryResult`; inspect `success` and `error` rather
than losing the rest of the batch.

`submitAsyncQuery()` is safe from any thread, including a worker used while
the owner is stepping the simulation. Submission copies no World state and
does not wait for the world lock. Velox resolves queued requests at the start
of the next successful owner-thread `step()`, before simulation state changes.
`getAsyncResult()` blocks until then and consumes the handle.

This is a deterministic frame-boundary API, not a live read of a partially
solved World. Destroying a World while another thread waits for a result still
requires external synchronization.
