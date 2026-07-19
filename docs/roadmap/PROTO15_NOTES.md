# Roadmap 15: Solver Options

## Delivered

- Added the public `solver.h` API: `SolverOptions`, `FrictionModel`,
  `IterationPolicy`, and High Quality, Fast, and Stacking presets.
- Added `World::solverOptions()` and `World::setSolverOptions()`. Options are
  validated, can be changed between steps, and are included in rollback
  snapshots and serialized scenes.
- The CPU and CUDA contact paths consume the same POD option payload. The
  cone path solves a coupled 3x3 normal/tangent effective-mass block before
  projecting its accumulated impulse into the Coulomb cone.
- Fixed iteration counts now use the configured velocity and position pass
  counts. Adaptive CPU solving stops after an ordered full contact pass whose
  largest impulse change is below `impulseThreshold`; `StepStats` reports the
  number of velocity sweeps used.
- The opt-in stack preset adds additional split-impulse correction only for
  nearly stationary contacts, avoiding velocity-energy injection.

## CUDA Behavior

CUDA supports fixed-count two-axis and cone-block solver options. Adaptive
iteration needs a deterministic global reduction across colored GPU contacts,
which is not implemented. Setting `IterationPolicy::Adaptive` on a CUDA world
therefore switches it to the ordered CPU backend rather than silently using a
fixed GPU count. Switching back to Fixed restores the requested CUDA/Auto
backend when no device-loss fallback is active.

## Compatibility Decision

The existing engine used three split-impulse position passes. The roadmap
draft proposed four, but changing the default moved a sensitive hull-resting
regression. `SolverOptions` therefore preserves the existing default of three;
the High Quality and Stacking presets select six explicitly.

## Verification

`solver_options_demo` covers adaptive sweep reduction, cone solver execution
through the selected backend, stack-preset stability, validation, rollback,
and serialization. Its resting-contact sample performs 4 adaptive velocity
sweeps versus 32 fixed sweeps (four substeps x eight), with a 0.21 mm position
difference after the frame.

- CPU-only Release CTest: 20/20 passed.
- CUDA-enabled Release CTest: 22/22 passed.
- `fuzz_demo 80`: passed twice.
- `proto_manifold`: all eight checks passed.

## Merge Recommendation

Ready to merge. The only deliberate limitation is that Adaptive uses the CPU
reference solver when a CUDA backend was active, documented above.
