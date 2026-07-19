# Multi-TOI And CCD Quality Notes

## Implemented Foundation

Added the CPU/CUDA-shared `MotionQuality`, `BodyCcdTuning`, world defaults,
and multi-TOI settings. The controls are validated, saved in snapshots and
serialized worlds, and exposed through guarded `World` accessors.

- `Low` removes swept AABB expansion and the existing conservative recovery.
- `Medium` preserves existing PCS behavior.
- `Locked` has zero solver inverse mass and inertia, cannot advance, and
  rejects accumulated motion from forces, impulses, contacts, and joints.
- collision margin affects only broad-phase AABBs; speculative distance affects
  only contact reach.
- `queryMultiToi()` performs conservative advancement plus 16 bisection steps,
  sorts hits by time/handle, and enforces per-body and world caps.

## Verification

`stress_demo` covers a 1 km/s sphere against three planes: ordered candidate
TOIs near 1.95, 3.95, and 5.95 ms, cap enforcement, default inheritance,
locked-body immobility under force and velocity requests, and CCD snapshot
restore. The CUDA-enabled stress suite remained green.

## Remaining Work

This does **not** complete roadmap item 03. The current step path still uses
the pre-existing one-shot post-solve recovery; `MotionQuality::High` does not
yet split a frame and re-query after each actual impact. Dynamic-dynamic
chronological resolution, event-budget reporting, and the bullet/staircase
integration acceptance tests remain required before claiming multi-TOI CCD.
