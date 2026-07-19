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
- High-quality bodies replay on one shared chronological TOI timeline after
  the ordinary solve. Static geometry and a pair where both dynamic bodies
  select `High` participate; each event applies a contact-point restitution
  impulse and advances every participating body through the remaining time.
  A mixed High/Medium or High/Low moving pair remains on the regular PCS
  recovery path, preventing a one-sided rewind from violating momentum.
  `StepStats::multiToiEvents` reports the bounded event count.

## Verification

`stress_demo` covers a 1 km/s sphere against three planes: ordered candidate
TOIs near 1.95, 3.95, and 5.95 ms, cap enforcement, default inheritance,
locked-body immobility under force and velocity requests, and CCD snapshot
restore. An elastic ricochet reaches three X walls and one Z wall within 10 ms,
reports exactly four High-quality static TOI events, and conserves speed within
1e-3 m/s. A two-event cap stops that same replay after exactly two events. A
2,000 m/s, equal-mass High/High sphere collision reports one event, reverses
both bodies, and preserves the center of mass. The CUDA-enabled stress suite
remained green.

After the shared timeline pass, the full CUDA-enabled CTest suite passed all 13
tests, `fuzz_demo 80` passed twice, and `proto_manifold` passed all eight
checks. `benchmark.exe` measured the default Medium-quality 8192-sphere rain
scene at 11.574 ms (CPU-1), 6.077 ms (CPU-auto), and 6.821 ms (CUDA); the
2048-sphere terrain scene measured 2.351, 3.619, and 2.038 ms respectively.
The High path is inactive unless a body explicitly selects it, so it adds no
work to those default workloads.

## Remaining Work

This does **not** complete roadmap item 03. Static and opt-in High/High dynamic
pairs are handled chronologically, but mixed-quality dynamic pairs deliberately
fall back to PCS. The bullet/staircase integration acceptance tests and a
GPU-native multi-TOI implementation remain required before claiming complete
multi-TOI CCD.
