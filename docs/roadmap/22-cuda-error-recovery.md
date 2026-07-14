# 22 — CUDA Error Recovery

## Goal

Handle CUDA allocation failures and device loss gracefully by falling back to the CPU backend. When a CUDA operation fails (out-of-memory, driver reset, device disconnect), Velox should not crash but instead switch to CPU execution for the remainder of the simulation, logging the failure and allowing the user to recover.

## Public API

```cpp
namespace velox {

enum class DeviceLossPolicy : uint8_t {
    FallbackToCPU = 0,      // switch to CPU backend on device loss
    ThrowException = 1,     // current behavior: throw on CUDA errors
};

class World {
public:
    DeviceLossPolicy deviceLossPolicy() const;
    void setDeviceLossPolicy(DeviceLossPolicy policy);

    // Check if the world is currently running on CPU (fallback activated).
    bool isOnCPUBackend() const;

    // Reset the CUDA backend: re-initialize device, reallocate buffers.
    // Call after a device loss event if FallbackToCPU was active.
    bool resetCUDABackend();
};

} // namespace velox
```

## Data structures

- `DeviceLossPolicy` enum — new, lives in `include/velox/world.h`.
- `World::deviceLossPolicy_` member — added to World.
- `World::fallbackToCPU_` flag — tracks whether CPU fallback is active.

## Algorithm

**Allocation failure handling:**

1. Before each CUDA allocation (body array, contact array, etc.), check available device memory via `cudaMemGetInfo()`.
2. If the requested size exceeds available memory by > 10%, log a warning and fall back to CPU for that allocation (or for the entire step if the scene is too large for GPU).
3. If a CUDA kernel launch returns an error, check the error code:
   - `cudaErrorMemoryAllocation`: fall back to CPU backend.
   - `cudaErrorNotSupported`: log and continue (feature not available on this device).
   - Other errors: throw (unexpected).

**Device loss handling:**

1. After any CUDA API call, check for `cudaErrorDevicesRemoved` or `cudaErrorNotReady`.
2. If device loss is detected:
   a. Log the error with device ID and timestamp.
   b. Set `fallbackToCPU_ = true`.
   c. Switch the active backend to the CPU backend.
   d. Continue simulation on CPU; the user can call `resetCUDABackend()` later to attempt recovery.

**Recovery:**

1. `World::resetCUDABackend()` attempts to re-initialize the CUDA device:
   a. Call `cudaSetDevice()` and verify the device is still present.
   b. Reallocate all GPU buffers at their current required sizes.
   c. If successful, switch back to CUDA backend and clear `fallbackToCPU_`.
   d. If allocation fails again, remain on CPU and return false.

## Files

- `include/velox/world.h` — add DeviceLossPolicy enum, accessors, fallback flag
- `src/backend_cuda.cu` — add error checking wrappers, fallback logic
- `src/backend.cpp` — implement `isOnCPUBackend()`, `resetCUDABackend()`

## Tests

1. **Simulated OOM:** Force a CUDA allocation failure by setting `cudaMemGetInfo` to report 0 free bytes (via test hook). World must fall back to CPU without crashing; simulation continues with correct results.
2. **Device loss simulation:** Mock `cudaErrorDevicesRemoved` after a kernel launch. World switches to CPU backend; subsequent steps run on CPU.
3. **Recovery:** After device loss fallback, call `resetCUDABackend()` with a mock that reports the device is present. World switches back to CUDA; simulation continues on GPU.

## Acceptance

- [ ] CUDA allocation failure triggers automatic fallback to CPU backend
- [ ] Device loss is detected and handled without crashing
- [ ] `isOnCPUBackend()` returns true after fallback activation
- [ ] `resetCUDABackend()` successfully restores GPU execution when the device is available
- [ ] All existing CUDA tests pass with error recovery enabled

## Size: S

## Risks

- Falling back to CPU mid-simulation changes the simulation results (CPU and GPU solvers may differ slightly due to floating-point ordering). Must document that fallback is a best-effort recovery, not a deterministic switch.
- Device loss can occur at any time (e.g., GPU driver crash, thermal shutdown). The fallback must be fast enough to avoid visible stutter in real-time applications. Consider pre-warming the CPU backend so it's ready to take over immediately.
- `resetCUDABackend()` requires reallocating all GPU buffers, which can take hundreds of milliseconds for large scenes. Must document this latency and suggest calling it during a loading screen or pause.
