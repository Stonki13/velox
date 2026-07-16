# PROTO08 - Differential Testing vs Jolt: Review Status

## Status

Implemented on `proto/difftest`. Velox runs identical scenes side by side with
Jolt Physics 5.2 and compares trajectories statistically. Gated behind
`VELOX_BUILD_DIFFTEST=OFF`; the default build fetches nothing and is
unaffected. Registered as ctest `velox.difftest` when enabled.

## Architecture

- `tests/difftest/diff_test.h` — engine-agnostic scene descriptors (bodies,
  materials, point joints, CCD flags, tracked indices), per-frame state
  samples, and statistical result types.
- `scene_library.cpp` — five canonical scenes: `sphere_drop` (restitution),
  `box_stack` (10-box resting stability), `pendulum` (joint + energy over
  6 s), `sphere_roll` (friction + rolling transition), `ccd_wall` (150 m/s
  bullet vs thin wall — anti-tunneling head-to-head).
- `velox_runner.cpp` / `jolt_runner.cpp` — the only engine-specific code.
  Velox runs the CPU backend (deterministic reference). Jolt uses two broad
  phase layers, `CalculateInertia` mass override, and `LinearCast` motion
  quality for the CCD bullet. Ground planes are large static boxes so both
  engines see identical geometry.
- `compare.cpp` — max position/velocity deltas, per-engine energy drift
  (flags only GAINED energy; dissipation is expected), and per-axis Pearson
  correlation with a resting-noise floor.
- `main.cpp` — two gates: (1) Velox vs Velox self-comparison must be bitwise
  identical (catches solver non-determinism); (2) Velox vs Jolt within
  per-scene tolerances plus behavioral invariants that hold in any correct
  engine: the tower must stand, the bullet must never cross the wall plane,
  the pendulum arm stays 2 m, friction actually slows the rolling sphere.

## Cross-engine modeling notes

- Jolt defaults linear/angular damping to 0.05; the runner zeroes both to
  match Velox.
- Velox combines restitution with `min`, Jolt with `max`; scenes assign equal
  restitution to both partners so the effective coefficient matches.
- Tolerances are per-scene: chaotic scenes (stack) rely on behavioral bounds,
  and velocity deltas at impacts are bounded by frame quantization (a bounce
  landing one frame apart between engines briefly shows approach+exit speed).

## Engine bugs found and fixed (the point of the exercise)

Differential testing found three real Velox defects on its first run, all
invisible to the existing stress/fuzz/soak suites:

1. **GJK false overlap on large geometry** (`src/gjk.h`). At default
   `GeometryQuality::Normal`, `solveTetrahedron` used raw sign tests on
   dot/cross products of support coordinates; for a sphere resting on a
   100 m static box the float noise (~1e-3) exceeded the real 1e-3 gap and a
   sliver tetrahedron reported "origin inside", feeding EPA a garbage
   penetration axis. Symptom: the sphere lost energy, sank, and finally
   teleported through 2 m of solid static box at 1.6 m/s. Fixed by making
   the sliver-volume gate unconditional and comparing normalized plane
   distances against a scale-relative boundary band (3e-6 x pair scale).
2. **Speculative restitution fired at a distance** (`src/narrowphase.h`).
   The solver applied the restitution target as soon as a fast approach was
   detected, even on a speculative contact created a full step's travel from
   the surface: a 150 m/s bullet bounced 2 m before the wall. Restitution now
   waits for the live gap to close, and the restitution floor persists
   through the separating substeps so the impulse accumulated while braking
   across the gap cannot act as glue and cancel the exit velocity.
3. **Bounce lost across frame boundaries** (`src/world.cpp`). A fast approach
   braked across the speculative gap spans frames; the next frame's contact
   re-measured `vn0` after braking and reflected only the residual speed
   (~4 m/s instead of ~15 m/s). Warm-start matching now carries the original
   approach speed per body pair while the pair is still closing fast. Also:
   the conservative-advancement CCD rescue now applies combined restitution
   instead of removing all approach velocity inelastically.

After the fixes: a rolling sphere behaves identically on planes and static
boxes of any size, matches Jolt's textbook 5/7 rolling transition shape, and
the CCD bullet bounces at the wall face at -14.4 m/s vs Jolt's -15.0 m/s.

## Results

```
determinism: velox self-comparison bitwise identical on 5 scenes
scene             maxPosD    maxVelD  eDriftVx  eDriftJl   corr  verdict
sphere_drop        0.0796      6.563    0.0000    0.0000  1.000  PASS
box_stack          0.0141      0.102    0.0002    0.0000  1.000  PASS
pendulum           0.3082      0.890    0.0015    0.0000  0.995  PASS
sphere_roll        1.3952      0.553    0.0000    0.0000  1.000  PASS
ccd_wall           1.8064     57.353    0.0000    0.0000  0.929  PASS
```

## Deviations from the roadmap spec

- Jolt only for now; the Bullet adapter can reuse the same scene descriptors.
- Scenes are C++ descriptors rather than JSON files (no third-party JSON
  dependency; the descriptors are trivially serializable later).
- Pointwise millimeter tolerances from the spec are unrealistic across
  different solver architectures; per-scene statistical bounds plus
  engine-agnostic behavioral invariants catch regressions without false
  alarms.

## Follow-up fix: rolling-contact energy bleed (4th bug)

The "known residual" rolling-speed bleed turned out to be a fourth solver
defect, also found by this suite: `solveContact`/`warmStartContact` applied
impulses at the midpoint of the two witness anchors, and the static body's
anchor is frozen at detection while the dynamic body advances through the
substeps — so the midpoint lagged the true contact tangentially and the
NORMAL impulse acquired a spurious torque arm. Symptoms: a frictionless
sphere sliding on a plane slowly gained spin, and a rolling sphere held a
permanent ~3 mm/s phantom slip that friction dissipated (~10%/5 s). Fixed by
computing each body's torque arm from its OWN witness anchor. After the fix,
Velox reproduces the analytical 5/7 rolling transition exactly
(vx = 4.285714 for a 6 m/s launch) and `sphere_roll` matches Jolt with
maxPositionDelta 0.0000; `ccd_wall` correlation rose to 0.999.

## Notes

- `velox_difftest_diag` (EXCLUDE_FROM_ALL) is a scratch target for trajectory
  comparisons when investigating future differences.

## Merge recommendation

Ready to merge after normal review: full test suite passes, and the three
engine fixes are regression-gated by the new differential suite itself.
