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
- High-quality dynamic bodies replay against static level geometry in
  chronological TOI order after the ordinary solve; each event applies a
  contact-point restitution impulse and advances only remaining time.
  `StepStats::multiToiEvents` reports the bounded event count.

## Verification

`stress_demo` covers a 1 km/s sphere against three planes: ordered candidate
TOIs near 1.95, 3.95, and 5.95 ms, cap enforcement, default inheritance,
locked-body immobility under force and velocity requests, and CCD snapshot
restore. An elastic ricochet reaches three X walls and one Z wall within 10 ms
and reports exactly four High-quality static TOI events. The CUDA-enabled
stress suite remained green.

After the in-step static pass, the full CUDA-enabled CTest suite passed all 13
tests, `fuzz_demo 80` passed twice, and `proto_manifold` passed all eight
checks. A short benchmark sample showed no default-scene regression because
the High path is inactive unless a body explicitly selects it.

## Remaining Work

This does **not** complete roadmap item 03. High-quality static geometry is
handled chronologically, but dynamic-dynamic chronological resolution still
needs both participants to be rescheduled together. The bullet/staircase
integration acceptance tests and a GPU-native multi-TOI implementation remain
required before claiming complete multi-TOI CCD.
