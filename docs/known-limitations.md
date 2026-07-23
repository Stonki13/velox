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
- **No soft-body solver.** Not planned until real user demand exists.
- **One vehicle model.** A single raycast vehicle vs. Jolt's wheeled/
  tracked/motorcycle controller family.
- **GPU acceleration is NVIDIA-only.** The CUDA backend has no AMD/Intel
  compute path; on non-NVIDIA hardware, compare Velox's CPU backend to
  Jolt's CPU backend, not the GPU numbers.
- **CUDA has a crossover point.** Measured (see
  `PROTO_COMPETITIVE_NOTES.md`'s Phase 2 section): the CUDA backend wins
  clearly on dense dynamic-dynamic contact scenes at roughly 2000+ bodies.
  Below that, or for joint-dominated/sparse-contact scenes, the CPU
  backend is currently faster. `BackendType::Auto` does not (yet) choose
  based on scene shape — it is a static choice made at `World`
  construction time.
- **No cross-engine character-controller comparison.** The Jolt
  differential-test corpus (`tests/difftest/`) does not include a
  character-slopes scene: a meaningful comparison needs both engines'
  character controllers running side by side, which is a materially
  different (and larger) undertaking than the harness's rigid-body scenes.
  Velox's own character controller is still covered by
  `tests/unit/test_character.cpp` and `examples/character_demo.cpp`.

## Known pre-existing test issues

- `velox.stress` and its repeat (`velox.stress_repeat`) currently fail two
  named checks: `sleeping pile (sleeps, wakes on impact)` and
  `contact events (begin fires once per touch)`. These are pre-existing
  sleep-state bookkeeping and contact-event-counting defects, not
  correctness issues in the state-hashing/rollback/replay/GPU-transfer
  work described in `PROTO_COMPETITIVE_NOTES.md`. Tracked, not silently
  ignored — see that file's Phase 0 baseline section for exact evidence.

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
