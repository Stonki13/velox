# PROTO11 - Capsule Character Controller: Review Status

## Status

Implemented on `proto/character`. The controller is a CPU-only query layer:
it owns capsule position state but never creates, modifies, or steps a World
body. The normal CUDA-enabled Release build, full regression suite, extended
fuzzer, and persistent-manifold prototype pass.

## Design Decisions and Audit Findings

1. The roadmap's `Move(Vec3)` API has no position input despite returning a
   final position. `SetPosition(Vec3)` and `Position()` were added so callers
   can spawn or teleport the query capsule. This is controller-local state,
   not World state, and is required for repeated `Move()` calls to be useful.
2. `Move()` uses the existing public `World::capsuleCast` API only. No body is
   created, no overlap cache is retained, and no World mutation is performed.
3. Grounding uses a downward capsule cast from one `ghostPadding` above the
   physical capsule. A hit is walkable only when `normal.y >=
   slopeLimitCosine`; grounded input is projected onto that ground plane.
4. Forward movement performs at most four deterministic sweep-and-slide
   iterations. Each contact removes only the inward normal component from the
   residual. Any residual left after the fourth iteration is discarded, which
   prevents corner jitter and energy gain.
5. Resting floor contacts appear at cast time zero. Forward and upward capsule
   casts start at the ghost-skin offset, while downward ground/landing casts
   use the physical capsule. This keeps flat walking stable without a visible
   float above the floor.
6. A step is accepted only after an unobstructed upward lift, a full elevated
   horizontal recast, and a downward cast landing on a walkable slope. Tall
   walls and a ceiling over the capsule reject the step.
7. The controller exposed a public query bug: plane shape casts measured a
   sphere/capsule core against the plane but omitted its inflation radius.
   `src/queries.cpp` now measures the actual inflated surface, matching the
   narrow phase and `closestPoints` behavior.
8. `SetJumpVelocity()` retains the roadmap name. Because the query-only API
   has no timestep, its positive value is an upward displacement contribution
   on the next grounded `Move()`, documented in the public header.

## Coverage

`examples/character_demo.cpp` is registered as `velox.character` and covers:

- Flat-ground walking and stable zero-displacement grounding.
- Climbing a 0.25 m step.
- Sliding along a 45-degree wall while staying grounded.
- Sliding backward down a 50-degree slope with a 45-degree limit.
- A two-wall corner with no blocked-motion energy gain and bitwise replay from
  the same World snapshot.
- Rejection of a tall wall as a step and rejection of a valid-height step when
  a ceiling blocks the lift.

## Verification

CUDA-enabled Release build:

- `cmake -S . -B build && cmake --build build --config Release`: passed.
- `ctest --test-dir build -C Release --output-on-failure`: passed
  `velox.stress`, `velox.fuzz`, `velox.soak`, `velox.geometry_fuzz`, and
  `velox.character`.
- `build/examples/Release/fuzz_demo.exe 80`: `80 scenes, 0 failures`.
- `build/examples/Release/proto_manifold.exe`: passed all eight manifold
  checks, including CPU/CUDA parity.

## Merge Recommendation

Ready to merge after normal review. The only intentional API addition beyond
the roadmap sketch is explicit controller position access, required by the
specified position-less `Move(Vec3)` signature. No CUDA device code changed;
the existing CUDA build continues to compile.
