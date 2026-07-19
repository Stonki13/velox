# Batched Async Query Notes

## Design

Added value-owned `QueryDesc`, `QueryResult`, and `AsyncQueryHandle` types in
`include/velox/queries.h`. Synchronous batches preserve input order and use a
single World access boundary. Individual malformed descriptors report their
own error rather than aborting unrelated requests.

Async submission is intentionally separate from the World access mutex. It
copies a descriptor under a small queue mutex, so workers can enqueue while a
solver step owns the World lock. The owner drains the queue at the beginning
of `stepImpl()`, before integration and collision detection. Results are
value-owned, signalled by a condition variable, and consumed once.

The first pass shares the broad-phase refresh for a batch but deliberately
uses the existing, well-tested individual primitive query implementations for
narrow phase. It does not claim the roadmap's speculative five-times
throughput target yet; traversal coalescing needs separate profiling before
changing query ordering.

## Verification

`batched_queries_demo` checks direct/batch ray, overlap, and shape-cast
parity, independent invalid-request failure, strict-policy worker submission,
and deterministic next-frame async resolution.

The CPU-only Release suite passed 13/13. The CUDA-enabled Release suite passed
15/15, including the sandbox self-test and Jolt differential comparison.
`fuzz_demo 80` passed twice and `proto_manifold` passed all eight checks.

The normal benchmark was re-run after the empty-queue step check. Normal
Worlds now avoid the queue mutex entirely through an acquire/release pending
flag; the queue mutex is taken only when work is actually submitted.

Latest local warm-path medians were 11.522 ms (CPU-1), 6.771 ms (CPU-auto),
and 5.066 ms (CUDA) for 8192-sphere rain; the 2048-sphere terrain scene was
0.777 ms, 0.688 ms, and 1.883 ms respectively. Interactive desktop load made
the parallel CPU sample vary by roughly six percent across repeated runs, so
these are monitoring data rather than a new performance baseline.

## Merge Recommendation

Ready for normal review after the CPU and CUDA regression gates. The API is a
correctness-first nonblocking submission boundary; query traversal batching is
documented as follow-up performance work.
