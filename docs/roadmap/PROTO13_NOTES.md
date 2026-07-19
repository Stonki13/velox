# Roadmap 13: Ragdoll Articulation

## Delivered

- Added `include/velox/ragdoll.h` and `src/ragdoll.cpp` with
  `RagdollBone`, `RagdollJoint`, and `RagdollBuilder`.
- `Build` accepts caller-owned bodies, validates that the directed link graph
  is one connected tree, normalizes dynamic bone masses while preserving each
  body's inertia shape, creates the constraints, and returns the tree root.
- Passive links use the existing cone-twist solver with swing and twist
  limits. Motorized links use the existing hinge motor around the authored
  axis; `SetMotorTorque` explicitly rejects non-hinge links rather than
  pretending a cone-twist motor exists.
- `WakeAll` and `Joints` manage the valid body and joint handles associated
  with a built root. Stale handles are filtered when queried after ordinary
  World removal.
- Added `ragdoll_demo` and CTest `velox.ragdoll`.

## Design Decisions

`RagdollBone` already contains a `BodyId`, so the builder deliberately does
not create hidden bodies or take ownership of user geometry. Its mass is
applied to dynamic bodies; `localCenterOfMass` is validated source metadata
because Velox collider bodies are already center-of-mass based.

The reduced-coordinate articulation draft remains deferred, exactly as the
roadmap specifies. This feature is an authoring layer over the production
joint solver, not a second solver path.

## Verification

`ragdoll_demo` verifies an eight-bone constrained figure over five seconds,
root/joint registration, `WakeAll`, a motorized elbow, and cycle/disconnected
graph rejection. It passed in both CPU-only and CUDA-enabled Release builds.

## Full Gate

- CPU-only Release CTest: 21/21 passed.
- CUDA-enabled Release CTest: 23/23 passed.
- `fuzz_demo 80`: passed.
- `proto_manifold`: all eight checks passed.

## Merge Recommendation

Ready to merge.
