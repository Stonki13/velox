# CUDA Recovery

`World(BackendType::Auto)` uses CUDA when it is available. A CUDA allocation
failure or a failed device context can be handled without losing the simulation
frame:

```cpp
world.setDeviceLossPolicy(velox::DeviceLossPolicy::FallbackToCPU);
world.step(dt); // retries this frame on CPU if CUDA reports a recoverable failure

if (world.isOnCPUBackend()) {
    // Schedule this outside the frame-critical path: CUDA buffer recreation
    // can take noticeable time for a large world.
    const bool restored = world.resetCUDABackend();
}
```

`FallbackToCPU` is the default. Before dispatching CUDA work, Velox snapshots
the complete observable World state. If CUDA reports out-of-memory, unavailable
devices, an invalid context, a launch failure, a timeout, or an unknown runtime
failure, it discards the partial host state, switches to CPU, and re-runs the
same step. `ThrowException` instead propagates `BackendFailure` so applications
can own their recovery policy.

The fallback is a continuity mechanism, not lockstep synchronization: the CPU
and CUDA solvers have different valid impulse orders, so a transition can
produce a slightly different trajectory. `resetCUDABackend()` does not retry a
failed frame; it only recreates the CUDA backend for later steps.

CUDA allocations perform a best-effort free-memory check. On Windows WDDM a
zero `cudaMemGetInfo()` value is advisory, so `cudaMalloc` remains the final
allocation authority. This avoids false CPU fallback while still classifying a
real allocation failure as recoverable.
