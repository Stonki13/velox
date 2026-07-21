# Changelog

All notable changes to Velox are documented here. The project follows
[Semantic Versioning](https://semver.org/) while the major version is zero:
breaking changes are allowed, but must be recorded in this file and follow the
deprecation policy where practical.

## [0.5.0] - 2026-07-21

### Added

- **C API wrapper** (`include/velox/velox_c.h`) for FFI compatibility with Box3D's
  pure C17 design. Enables integration with any language/engine without C++ dependency.
- Recording/replay exposed in C API matching Box3D's pattern.
- Full Box3D feature parity: C API, recording/replay, 8 joint types (vs Box3D's 9,
  with ConeTwist and SixDof as Velox-exclusive advantages), motor joint, character
  controller, vehicle model, ragdoll builder.

### Velox advantages over Box3D

- GPU acceleration (CUDA backend with graph-colored parallel solving)
- 3-layer Predictive Contact Sweeping CCD (more robust than Box3D's single-layer)
- Built-in character controller, raycast vehicle, ragdoll builder
- Anisotropic friction with per-body local-space friction scale
- Origin shifting for large worlds without precision loss
- Vulkan interactive sandbox for debugging
- 35-test suite including differential testing against Jolt Physics

## [0.4.0] - 2026-07-21

### Added

- `TaskSystem` interface fully wired into CPU backend parallel dispatch
  (`dispatchChunks`, `parallelChunks`, `integrate`) for game engine integration.
- `Motor` joint type — position-controlled constraint with max force/torque clamping.
- `World::addMotorJoint()` public API.
- `BodyEvent` Created/Destroyed lifecycle events alongside existing Moved events.
- Unit tests for character controller, vehicle model, and ragdoll builder.

### Changed

- Marked stale design sketches (`design/character_controller.h`, `design/vehicle.h`,
  `design/serialization.h`) as superseded by production implementations.

## [0.3.0] - 2026-07-21

### Added

- Per-body property setters: `setSensor()`, `setGravityScale()`, `setLinearDamping()`,
  `setAngularDamping()`, `setCollisionFilter()`, `setEnableSleep()`, `setFixedRotation()`,
  `wakeBody()`, `sleepBody()` with corresponding getters.
- `World::explode()` — radial impulse with linear falloff for gameplay effects.
- `enableSleep` and `fixedRotation` flags on `Body` struct, respected by the solver.
- `TaskSystem` interface for external task system injection (game engine job schedulers).
- `BodyEvent` struct and `World::bodyEvents()` for body lifecycle tracking (Moved events).
- `VELOX_API` symbol visibility macro for shared library builds (dllexport/dllimport/visibility).
- `alignas(64)` on `Body` and `Contact` structs for cache-line alignment.
- Benchmark regression detection script (`scripts/benchmark_regression.py`).
- Unit tests for World API (body creation, removal, property setters, snapshot, explosion),
  joints (ball, hinge, distance, spring, prismatic, break events), and queries
  (raycast, overlap, shape cast).

## [0.2.0] - 2026-07-21

### Added

- Granular performance telemetry in `StepStats`: `broadPhaseProxies`,
  `narrowPhaseTests`, `islandCount`, `broadPhaseMs`, `narrowPhaseMs`,
  `contactSolverMs`, `jointSolverMs`, `ccdRecoveryMs`.
- `Backend::lastIslandCount()` diagnostic accessor.
- Doctest-based unit test framework with tests for math, GJK, mass properties,
  narrowphase, and serialization round-trips.
- `RoundedBoxShape` and `EllipsoidShape` collider types with GJK support,
  mass properties, raycasting, debug drawing, compound child support,
  runtime mutation, and scaling.
- `GPUResidentMode` enum and `World::gpuResidentMode()` / `setGPUResidentMode()`
  for controlling GPU-resident stepping on the CUDA backend.
- `BenchmarkConfig`, `BenchmarkResult`, and `runBenchmark()` / `runBenchmarkSuite()`
  for standardized performance measurement.

## [0.1.0] - 2026-07-19

### Added

- Public version constants and `velox::versionString()`.
- `VELOX_DEPRECATED(message)` for source-compatible API transitions.
- Opt-in strict CPU determinism mode and cross-platform trace CI.

### Changed

- The public version header is the source of the CMake package version.
