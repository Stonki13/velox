# CUDA Error Recovery Notes

## Design

Added typed `BackendFailure` errors for recoverable CUDA allocation and device
context failures. `World::step()` saves the step-mutable observable state only
when the active backend is CUDA and `DeviceLossPolicy::FallbackToCPU` is
selected. On a typed failure it replaces the CUDA backend with CPU, restores
that rollback state, and retries the same frame. Invalid scene input and
unclassified CUDA errors still throw normally.

`resetCUDABackend()` creates a replacement CUDA backend before discarding the
working CPU backend. It returns false if CUDA cannot be recreated. The active
backend is observable through `isOnCPUBackend()`.

CUDA allocations use `cudaMemGetInfo()` as a best-effort early OOM guard, then
leave `cudaMalloc` authoritative. This is required on Windows WDDM, where the
reported free value can be zero despite a valid allocation.

## Verification

`cuda_recovery_demo` is registered as `velox.cuda_recovery`. It skips on a
CPU-only system. With CUDA it injects simulated allocation failure and device
loss independently, checks the world retries on CPU with finite body state,
restores CUDA and steps again, then injects simulated device loss under
`ThrowException` and checks that the typed error propagates without switching
backend.

The CUDA-enabled Release build, separate CPU-only Release build, and all 14
CTest suites passed. `fuzz_demo 80` passed twice and `proto_manifold` passed
all eight checks. The normal CUDA path does not activate the rollback
allocation or CPU fallback logic unless a recoverable CUDA error occurs.

`benchmark.exe` after the change measured 8192-sphere rain at 11.596 ms
(CPU-1), 6.150 ms (CPU-auto), and 6.540 ms (CUDA); terrain measured 0.841,
0.638, and 1.676 ms respectively. The new memory check runs only on a device
allocation or buffer growth, not in steady-state stepping.

## Merge Recommendation

Ready for normal review after the full regression gate. CPU fallback is
best-effort by design and is documented as non-lockstep deterministic.
