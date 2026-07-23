# Competitive Positioning: Velox vs Jolt Physics

## Status

Phase 0 audit on `feature/competitive-velox`, branched from `main` at commit
`9dc8f8e`. This document is the working plan for phases 1-5; it is updated
with exact evidence after every phase's gate.

## Method

Read, in order: `README.md`, `CHANGELOG.md`, `docs/roadmap/` (29 completed
roadmap items + their `PROTO*_NOTES.md`), `docs/production-readiness.md`,
`docs/api-stability.md`, `docs/serialization.md`, `docs/threading.md`,
`docs/performance.md`, `.github/workflows/{ci,cuda,release}.yml`,
`tests/difftest/` (the existing Jolt differential suite), the full public
API surface under `include/velox/`, and `benchmarks/`. Jolt's public
documentation and header comments (from prior general knowledge, not a fresh
clone) inform the gap analysis; no Jolt source was modified or vendored.

## Velox strengths already implemented (verified by reading code, not marketing copy)

- **Predictive Contact Sweeping CCD**: speculative contacts + iterative
  velocity solve + a conservative-advancement TOI rewind safety net for any
  pair that still tunnels, with a momentum-preserving impulse at the shared
  TOI (`README.md`, `src/world.cpp`). Validated against Jolt's `LinearCast`
  behavior in `tests/difftest` (`ccd_wall` scene).
- **Persistent contact manifolds** with Sutherland-Hodgman face clipping and
  stable feature-keyed warm starting (roadmap 01), plus a hardened GJK/EPA
  path with a `GeometryQuality` escalation knob and a dedicated geometry
  fuzzer (roadmap 02).
- **CUDA backend**: graph-colored parallel contact solving, a uniform-grid
  broad phase with cached-scratch thrust sorting, and parallel CPU island
  solving as a portable fallback path. Jolt is CPU-only; it has no GPU
  compute backend at all.
- **Deterministic replay foundation already exists**: `WorldSnapshot`
  (`World::saveSnapshot/restoreSnapshot`), a versioned binary
  `SerializedScene` format (`include/velox/serialization.h`), and a
  `ReplayRecording`/`verifyReplay` pair that re-simulates and reports the
  first mismatching **frame**. Serialization V2
  (`include/velox/serialization_v2.h`) adds CRC-32 section integrity,
  chunked/incremental serialization for large worlds, JSON/scene-graph
  export, and a V1 migration path.
- **Cross-platform bitwise determinism is already CI-gated**: strict
  floating-point traces are compared across Windows, Linux x86_64, macOS
  Intel, and Apple Silicon (`docs/production-readiness.md`,
  `tests/cross_platform/`). This is the hard precondition rollback netcode
  needs, and it is done, not aspirational.
- **Differential correctness vs Jolt** already covers six scenes
  (`sphere_drop`, `box_stack`, `pendulum`, `sphere_roll`, `ccd_wall`,
  `gyro_spin`) with behavioral invariants and statistical tolerances, not
  brittle identical-trajectory checks, plus a hard bitwise Velox-vs-Velox
  determinism gate. It already found and fixed four real Velox defects
  (GJK false overlap, early speculative restitution, CCD losing its
  bounce, rolling-contact anchor lag) — the mechanism this project extends
  in Phase 4 has already paid for itself once.
- **Benchmark framework** (`benchmarks/benchmark_framework.h`) already
  computes mean/median/stddev/min/max plus p50/p95/p99 and exports
  JSON/CSV/table with baseline-regression comparison. It does **not** yet
  stamp hardware identity or a commit hash into that JSON (Phase 2 gap).
- Character controller, raycast vehicle, and a ragdoll authoring builder are
  shipped in the core library with dedicated unit tests.
- Error handling is a structured `ErrorCode` hierarchy with actionable
  recovery suggestions (`include/velox/error.h`), a documented ABI-stability
  policy (`docs/api-stability.md`), and a C API (`velox_c.h`) plus a
  versioned stable vtable (`stable_api.h`) for FFI/dynamic-loading
  consumers.

## Real gaps vs Jolt (honest, not defensive)

- **Production history.** Jolt has shipped in a real title (Horizon Forbidden
  West) and is Godot 4's default physics engine. Velox has shipped in
  nothing. No amount of internal testing substitutes for that.
- **Soft bodies.** Jolt has a soft-body solver. Velox does not, and this plan
  does not build one — soft bodies are explicitly deferred to Phase 5 until
  real user demand exists.
- **Vehicle depth.** Jolt ships `WheeledVehicleController`,
  `TrackedVehicleController`, and `MotorcycleController` with differential
  and anti-roll-bar tuning. Velox has one raycast vehicle model
  (`include/velox/vehicle.h`). This is a real, acknowledged gap, not a
  target for this plan.
- **Ecosystem maturity.** Jolt has years of public issue history, third-party
  bindings, and battle-tested edge cases from real shipped games. Velox's
  hardening today comes from its own fuzzers, soaks, and a six-scene Jolt
  differential suite — thorough for its age, but not the same as years of
  production fire.
- **No deterministic-multiplayer toolkit in either engine.** This is the
  gap this plan targets. Jolt does not ship a canonical state hash, a
  bounded rollback ring buffer, delta-snapshot encoding, or first-divergence
  diagnostics either — engines generally leave this to the game's netcode
  layer. Velox is not behind Jolt here; it is choosing to build the missing
  piece explicitly rather than leaving it to every integrator to reinvent.
- **GPU advantage is NVIDIA-only.** CUDA has no AMD/Intel compute path.
  Framed honestly: Velox's GPU-scale claim only applies to users with an
  NVIDIA GPU; everyone else compares CPU thread pool to CPU thread pool,
  where the case is far less clear-cut.

## Chosen niche

**Deterministic multiplayer/replay physics on a CPU thread pool, plus
GPU-scale rigid-body throughput on NVIDIA hardware, with a debugging
workflow built for finding the first frame where two runs disagree.**

This is defensible because (a) Velox already has the precondition — CI-gated
bitwise cross-platform determinism — that Jolt integrators would have to
verify themselves before trusting rollback netcode on it; (b) Jolt has zero
GPU compute path, so any CUDA rigid-body throughput number is an
apples-to-nothing comparison rather than an apples-to-apples one Velox could
lose; and (c) neither engine ships the actual rollback/delta/hash tooling a
netcode integrator needs, so building it is additive, not a re-litigated
feature race.

## Phase acceptance criteria (measurable, checked at each gate)

| Phase | Deliverable | Acceptance test |
|---|---|---|
| 1 | Canonical versioned state hash | Hash is stable for an unmodified world across two independent runs; changes when any body field changes; documented format version. |
| 1 | Delta snapshot encode/decode | Delta of an unchanged world is near-zero bytes; delta replayed onto the reference snapshot reproduces the target snapshot bitwise. |
| 1 | Bounded rollback ring buffer | Configured capacity is a hard cap; pushing past capacity evicts the oldest frame; memory use verified by test, not just documented. |
| 1 | First-divergence diagnostics | Given two recordings that diverge at a known frame/body/field, the tool reports that exact frame, body index, and field name. |
| 1 | Prediction-correction sample | Headless CTest demonstrates predict -> diverge -> rollback -> resimulate -> match, with an assertion on the corrected state. |
| 1 | Determinism preserved | `velox.difftest`-style bitwise Velox-vs-Velox check across 1, 2, and N worker counts in strict mode still passes. |
| 2 | Benchmark provenance | JSON output includes hardware identity (CPU name, GPU name if CUDA), git commit hash, scene name, median, p95. |
| 2 | No unexplained regression | Any CPU scene regressing >10% vs the pre-phase baseline has a written reason in this document or the change is reverted. |
| 2 | CPU/CUDA parity | Existing `velox.cuda_smoke`-style tolerance check still passes; a new fallback test confirms CPU behavior is unchanged when CUDA is unavailable. |
| 3 | Samples run headlessly | Every new example has a CTest entry that runs without a display and exits 0/1 on success/failure. |
| 3 | No new mandatory dependency | Core library CMake target list is unchanged; new tools are optional targets. |
| 4 | Regression-per-defect | Every Velox bug the extended differential corpus finds gets one minimized scene added to `tests/difftest/scene_library.cpp`. |
| 4 | Invariants, not trajectories | New Jolt-comparison scenes assert behavioral bounds/statistical tolerances, matching the existing `tests/difftest/main.cpp` pattern. |
| 5 | Feedback scaffolding | Issue templates and a known-limitations doc exist; no telemetry code path executes without an explicit opt-in flag. |

## Baseline verification (this phase, on `main` before any new code)

Command: `cmake --build build_verify --config Release -j 16` then
`ctest --test-dir build_verify -C Release --output-on-failure` (Windows 11,
RTX 5080, CUDA 13.2, MSVC 19.44, 16 logical cores).

Result: clean Release+CUDA build (0 compiler errors). **51/53 CTest suites
pass (96%).** Two named checks inside `velox.stress` fail reproducibly (same
failure on both `velox.stress` and its `velox.stress_repeat` rerun):

- `sleeping pile (sleeps, wakes on impact)` — FAIL
- `contact events (begin fires once per touch)` — FAIL

These are pre-existing defects on `main`, unrelated to this plan's scope
(sleep-state bookkeeping and contact-event counting, not state
hashing/rollback/replay). The checks this plan's Phase 1 work depends on
already pass on this baseline: `world snapshot (topology, caches,
deterministic replay)` and `CPU workers (parallel integration/narrow phase,
serial replay)`. Recorded here rather than silently ignored; not fixed by
this plan since it is orthogonal to competitive positioning and the user is
independently addressing recent regressions on the same subsystems.

## Phase 1: deterministic multiplayer toolkit — evidence

Added, all in the isolated worktree on `feature/competitive-velox`:

- `include/velox/rollback.h` / `src/rollback.cpp` — a network-facing layer
  above the existing `WorldSnapshot` (same-instance rollback) and
  `SerializedScene` (cross-instance/cross-machine transfer):
  - `CanonicalHash computeCanonicalHash(const World&)` / `hashCanonicalBodyState(...)`:
    a versioned 64-bit FNV-1a hash over the exact bytes
    `captureCanonicalBodyState` produces — the same dense per-body
    position/orientation/velocity/angularVelocity capture `ReplayRecording`
    already treats as authoritative, so hashing never drifts from what
    replay verification considers "the state that matters."
  - `SnapshotDelta encodeDelta`/`applyDelta`/`changedBodyCount`: a
    bitmap-plus-changed-records diff between two same-shape canonical
    body-state buffers. Throws `VeloxInvalidArgument` on a body-count
    mismatch (encode) and `VeloxRuntimeError` on a corrupted/truncated
    buffer (apply) rather than silently producing wrong bytes.
  - `RollbackBuffer`: a capacity-bounded `std::deque<Entry>` ring keyed by
    an application frame number, built on `World::saveSnapshot`/
    `restoreSnapshot`. Evicts the oldest frame once `size() > capacity()`.
  - `findFirstDivergence`: compares two `ReplayRecording`s frame-by-frame,
    field-by-field, and reports the first (frame, bodyIndex, field,
    magnitude) that exceeds tolerance — or a `bodyCount`/`frameCount`
    mismatch if the recordings differ in shape before any field does.
  - `include/velox/serialization.h` gained one new public function,
    `captureCanonicalBodyState`, a thin wrapper over the existing private
    `SerializationAccess::captureBodies` used internally by
    `recordReplayFrame`/`verifyReplay` — no second definition of the
    canonical byte format was introduced.
- `examples/rollback_demo.cpp` (registered as headless CTest
  `velox.rollback_demo`): a server/client prediction-correction scenario —
  client predicts every frame, server applies a late impulse the client
  didn't predict, client detects the resulting canonical-hash mismatch,
  adopts the server's `SerializedScene` via `deserializeWorld`, and both
  sides reconverge to an identical hash. Also exercises
  `findFirstDivergence` against a predicted-vs-authoritative recording pair
  and asserts it locates the exact frame/body/field the late input first
  affects.
- `tests/unit/test_rollback.cpp` (registered as CTest `velox.unit.test_rollback`,
  16 `TEST_CASE`s): hash stability across identical replays, hash divergence
  once a body's state differs, delta round-trip (single step and a 20-frame
  delta chain), delta rejection on mismatched body counts, `applyDelta`
  rejection on a truncated or shape-mismatched buffer (simulated
  corruption), `RollbackBuffer` exact restore, capacity-bound eviction
  (oldest-frame/newest-frame/contains after overflow), zero-capacity
  rejection, `findFirstDivergence` on identical/corrupted/body-count-mismatched
  recordings, and a bitwise-strict-replay check across worker counts
  (1 worker vs. 0/auto) using `DeterminismMode::Strict`.

### Gate results (Windows 11, MSVC 19.44, CPU-only build — `build_phase1`,
`-DVELOX_ENABLE_CUDA=OFF`, isolated from the CUDA baseline build used above)

- `cmake --build build_phase1 --config Release -j 16`: clean build, 0
  compiler errors.
- `ctest --test-dir build_phase1 -C Release --output-on-failure`:
  **52/52 CTest suites pass (100%)**, including the two new
  `velox.rollback_demo` and `velox.unit.test_rollback` entries. (This is a
  CPU-only configuration; it does not include the CUDA-only suites counted
  in the 53-suite baseline above, and the two `velox.stress` sub-check
  failures noted in that baseline did not reproduce as separate CTest
  failures in this run — `velox.stress` passed here.)
- `fuzz_demo 40` run twice: `fuzz: 40 scenes, 0 failures` both times.
- `proto_manifold`: `=== Results: 0 failures ===` (8/8 manifold checks pass,
  including CPU/CUDA parity checked via CPU fallback since this build has
  no CUDA toolkit enabled).

No CPU regression is claimed or measured in this phase (no scene/timestep
changed); Phase 2 is where benchmark evidence with hardware identity and
commit hash gets attached.
