# 21 — GPU Resident Stepping

## Goal

Eliminate per-substep host-device data transfers by keeping body state, contact arrays, and joint data resident on the GPU across all solver substeps. The CUDA backend currently copies body state to device once per step; with resident stepping, the data stays on device and only the final state is downloaded after all substeps complete. This reduces PCIe transfer overhead from O(substeps) to O(1).

## Public API

```cpp
namespace velox {

enum class GPUResidentMode : uint8_t {
    Disabled = 0,       // current behavior: copy per substep
    Resident = 1        // keep state on device across all substeps
};

class World {
public:
    GPUResidentMode gpuResidentMode() const;
    void setGPUResidentMode(GPUResidentMode mode);
};

} // namespace velox
```

## Data structures

- `GPUResidentMode` enum — new, lives in `include/velox/world.h`.
- `World::gpuResidentMode_` member — added to World.
- New CUDA buffer objects: device-side body array, contact array, joint array that persist across substeps.

## Algorithm

**Current flow (per-substep copy):**

1. Copy host body array → device.
2. Run integration on device.
3. Run broad phase + narrow phase on device.
4. Copy contacts → host.
5. Solve velocities on device (or host for small scenes).
6. Copy body state back to host.
7. Repeat for each substep.

**Resident flow:**

1. Copy host body array → device once at step start.
2. For each substep:
   a. Run integration on device (reads/writes resident body array).
   b. Run broad phase + narrow phase on device (reads/writes resident contact array).
   c. Solve velocities on device (reads/writes resident body and contact arrays).
   d. No host transfers between substeps.
3. After all substeps: copy final body array → host once.

**Device-side coloring and event prep:**

1. Contact graph coloring runs once per step (not per substep) since the contact set is stable across substeps.
2. Color assignments are stored in device memory and reused for each substep's solver kernel.
3. Contact events (Begin/Persist/End) are computed on device after the final substep, avoiding host-side event detection loops.

**Fallback conditions:**

- If a joint breaks mid-step, the resident path must fall back to the per-substep copy path for that substep (joint removal requires host-side array mutation).
- If `World::mutateShape()` or other mutation APIs are called during a step, the resident buffers are invalidated and the next step uses the per-substep path.

## Files

- `include/velox/world.h` — add GPUResidentMode enum and accessor methods
- `src/backend_cuda.cu` — implement resident buffer management, multi-substep kernel launches
- `cmake/cuda.cmake` — document CUDA residency requirements (new file)

## Tests

1. **Numerical parity:** Resident mode vs per-substep mode on the same scene. Body positions after 1000 frames must match within 1e-5 (floating-point ordering differences expected).
2. **Transfer reduction:** Measure PCIe transfers per step. Per-substep: 4 transfers × substeps. Resident: 2 transfers total (upload at start, download at end). Verify transfer count matches.
3. **Joint break fallback:** A distance joint set to break at 100 N force breaks at substep 2 of 4. Resident mode must detect the break and fall back to per-substep copying for that step without crashing.

## Acceptance

- [ ] Resident mode reduces PCIe transfers from O(substeps) to O(1) per step
- [ ] Numerical results match per-substep mode within floating-point tolerance
- [ ] Joint breaks during a step trigger safe fallback to per-substep path
- [ ] Shape mutations during a step invalidate resident buffers correctly

## Size: L

## Risks

- Resident buffers consume GPU memory proportional to the maximum scene size. For scenes with highly variable body counts (e.g., particle effects), this can waste VRAM. Must document the memory overhead and provide a max-body-count configuration.
- CUDA Graph capture has limitations: certain kernel launches (e.g., those with dynamic memory allocation) cannot be captured. The resident path must avoid dynamic allocations during the graph-captured region.
- Debugging GPU-resident simulations is harder because state inspection requires device-to-host downloads at arbitrary points. Must provide a `VELOX_DEBUG_RESIDENT` flag that forces per-substep copies for debugging.
