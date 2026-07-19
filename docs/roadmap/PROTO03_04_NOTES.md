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
both bodies, and preserves the center of mass. A three-body High chain reports
two events: the first transfers momentum to the middle body and the scheduler
then finds and resolves its newly possible collision with the third body. The
same chain also passes through the CUDA Auto backend, exercising the required
host fallback after GPU substeps. A High box grazing a ten-step, 0.1 m-rise/run
staircase records at least eight events and remains above the final step rather
than tunneling through it. The CUDA-enabled stress suite remained green.

After the shared timeline pass, the full CUDA-enabled CTest suite passed all 13
tests, `fuzz_demo 80` passed twice, and `proto_manifold` passed all eight
checks. `benchmark.exe` measured the default Medium-quality 8192-sphere rain
scene at 11.574 ms (CPU-1), 6.077 ms (CPU-auto), and 6.821 ms (CUDA); the
2048-sphere terrain scene measured 2.351, 3.619, and 2.038 ms respectively.
The High path is inactive unless a body explicitly selects it, so it adds no
work to those default workloads.

## Scope And Follow-Up

Roadmap item 03 is complete for the portable CPU event scheduler and the CUDA
host fallback: it has sequential static and High/High dynamic impacts, strict
global event selection, restitution/momentum checks, event caps, a staircase
regression, and CUDA fallback coverage. Static geometry and opt-in High/High
dynamic pairs are handled chronologically; mixed-quality dynamic pairs
deliberately retain PCS so that the scheduler never rewinds one side of a
shared momentum interaction.

A GPU-native multi-TOI event scheduler remains a performance follow-up, not a
correctness limitation. It would avoid invalidating the CUDA caches after a
High-quality replay, but it is not required for the documented CPU fallback.

## Merge Recommendation

Ready to merge after normal review. The implementation is bounded by per-body
and global caps, is opt-in through `MotionQuality::High`, and the default
Medium-quality workload path remains unchanged.
