# Known Limitations

Honest, current gaps and caveats — collected here so they don't have to be
rediscovered per issue. If something you hit isn't listed, it's either not
known or not yet documented; file it (see the issue templates under
`.github/ISSUE_TEMPLATE/`).

## Compared to Jolt Physics

Velox does not claim to be generally better than Jolt. See
[`docs/roadmap/PROTO_COMPETITIVE_NOTES.md`](roadmap/PROTO_COMPETITIVE_NOTES.md)
for the full comparison; the headline gaps:

- **No production shipping history.** Jolt has shipped in real titles
  (Horizon Forbidden West) and is Godot 4's default. Velox has not shipped
  in a released game.
- **Soft-body solver is minimal.** XPBD distance-constraint solver covers
  cloth and deformable spheres with collision against static rigid bodies
  (planes, spheres, boxes). No self-collision, no tetrahedral FEM, no
  coupling back into the rigid-body solver (soft bodies do not push rigid
  bodies). See `include/velox/softbody.h`.
- **One vehicle model (extended).** A raycast vehicle with suspension,
  anti-roll bars, and a configurable differential (open/limited-slip/
  locked). Still a single model vs. Jolt's wheeled/tracked/motorcycle
  controller family — no tracked-vehicle or motorcycle-specific
  dynamics.
- **Full GPU acceleration is NVIDIA-only; the cross-vendor Vulkan backend
  covers integration + contact solving.** The CUDA backend additionally
  runs the broad and narrow phases on-device. The Vulkan backend
  (`BackendType::Vulkan`, NVIDIA/AMD/Intel) runs velocity integration and
  the graph-colored contact solve on the GPU, with collision detection on
  the CPU — measured ~25% faster than the best CPU configuration on a
  dense 8192-body contact scene, a wash at 2048 bodies, and slower on
  small or joint-dominated scenes (per-step dispatch overhead; joints and
  the `ConeBlockSolver`/`Adaptive` solver configurations delegate to the
  CPU path). Its crossover point is therefore higher than CUDA's. Neither
  GPU backend is part of the strict bitwise determinism guarantee; use
  `BackendType::Cpu` for lockstep replay.
- **CUDA has a crossover point.** Measured (see
  `PROTO_COMPETITIVE_NOTES.md`'s Phase 2 section): the CUDA backend wins
  clearly on dense dynamic-dynamic contact scenes at roughly 2000+ bodies.
  Below that, or for joint-dominated/sparse-contact scenes, the CPU
  backend is currently faster. `BackendType::Auto` does not (yet) choose
  based on scene shape — it is a static choice made at `World`
  construction time.
- **Cross-engine character-controller comparison is behavioral only.**
  The difftest corpus (`tests/difftest/`) now includes 3 character-slope
  scenes (flat, gentle, steep) comparing Velox's `CharacterController`
  against Jolt's `CharacterVirtual`. The comparison checks climb/slide
  agreement and approximate final position — not identical trajectories,
  since the two controllers use fundamentally different sweep and slide
  algorithms. Grounded-state reporting may differ (Velox reports grounded
  on flat/gentle slopes where Jolt does not, due to different ground
  detection thresholds).

## Resolved test issues

- `velox.stress` sub-checks `sleeping pile (sleeps, wakes on impact)` and
  `contact events (begin fires once per touch)` failed on GPU-backend
  builds (CUDA/Vulkan) due to drowsy-state handling in the device broad
  phase and integration kernels. Fixed in Phase A (see
  `PROTO_COMPETITIVE_NOTES.md`); regression tests added to
  `tests/unit/test_sleep.cpp`.

## Deterministic multiplayer toolkit

- `RollbackBuffer`/`WorldSnapshot`-based rollback only works within the
  **same `World` instance** that produced the snapshot (this is
  `WorldSnapshot`'s existing, deliberate contract). To adopt a different
  `World`'s state — e.g. a client adopting the server's authoritative
  snapshot over the network — use `serializeWorld`/`deserializeWorld`
  (`serialization.h`) instead. See `include/velox/rollback.h`'s header
  comment for the full division of responsibility.
- The canonical hash/delta format (`kRollbackToolkitVersion`) is versioned
  but has exactly one version so far; no migration path has been
  exercised yet.

## Attaching a reproducible scene to a bug report

The fastest way to get a bug fixed is a minimal, deterministic repro:

1. Build the scene with the CPU backend (`BackendType::Cpu`) so the repro
   is bitwise reproducible across machines.
2. Serialize it with `serializeWorld()` (`include/velox/serialization.h`)
   and attach the resulting bytes, or paste the short source that builds
   the scene from scratch — whichever is smaller.
3. If the bug is a divergence between two runs (e.g. client vs. server,
   or two platforms), record both as `ReplayRecording`s and save them with
   `examples/replay_diff_cli.cpp`'s trace format; running
   `replay_diff_cli expected.trace actual.trace` reports the exact frame,
   body, and field where they first disagree — paste that output directly
   into the issue.
4. Include the timestep, substep count, and any nondefault CCD/solver/
   thread-safety/determinism-mode settings — see the bug report template
   (`.github/ISSUE_TEMPLATE/bug_report.md`) for the full checklist.

## Telemetry

None. Velox has no telemetry, analytics, or phone-home code path anywhere
in the core library, examples, or tests, and none is planned without an
explicit, documented opt-in.
