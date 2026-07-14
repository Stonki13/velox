# Velox Roadmap — Master Index

Production-readiness layout for Velox, a C++17 rigid-body physics engine with GPU acceleration. Each item below has a detail file describing the public API, data structures, algorithm, affected files, tests, acceptance criteria, size estimate, and risks.

## Tier 1 — Blockers (must ship before 1.0)

| # | Item | Goal | Detail File | Dependencies |
|---|------|------|-------------|--------------|
| 01 | Persistent Contact Manifolds | Replace sampled/support-plane manifolds with general convex face clipping for stable stacking and warm starting | [01-persistent-contact-manifolds.md](01-persistent-contact-manifolds.md) | — |
| 02 | GJK/EPA Hardening | Robust degenerate geometry: near-coplanar hulls, tiny/huge/needle shapes; dedicated fuzzer harness | [02-gjk-epa-hardening.md](02-gjk-epa-hardening.md) | 01 |
| 03 | Multi-TOI CCD | Sequential time-of-impact processing per step for bullet-through-several-walls scenarios | [03-multi-toi-ccd.md](03-multi-toi-ccd.md) | 01, 02 |
| 04 | CCD Quality Controls | Per-body motion quality flags, collision margins, speculative distance tuning knobs | [04-ccd-quality-controls.md](04-ccd-quality-controls.md) | 03 |
| 05 | Thread-Safety Contract | Documented and enforced rules for cross-thread queries and mutation during step | [05-thread-safety-contract.md](05-thread-safety-contract.md) | — |
| 06 | API Versioning | Semver, CHANGELOG, deprecation policy, ABI stability guarantees | [06-api-versioning.md](06-api-versioning.md) | — |
| 07 | Documentation | Doxygen API reference, getting-started guide, concepts document | [07-documentation.md](07-documentation.md) | 06 |
| 08 | Differential Testing | Same scenes run in Jolt/Bullet; statistical trajectory/energy comparison harness | [08-differential-testing.md](08-differential-testing.md) | 01, 03 |
| 09 | Geometry Fuzzing & Long Soaks | Degenerate-shape fuzzer with energy/momentum drift tracking over multi-hour runs | [09-geometry-fuzzing-and-long-soaks.md](09-geometry-fuzzing-and-long-soaks.md) | 02, 08 |
| 10 | Real-Workload Scene | Game-like level as a release gate — must pass all Tier 1 checks | [10-real-workload-scene.md](10-real-workload-scene.md) | 01–09 |

**Tier 1 dependency graph:** `02 → 03 → 04`; `01, 03 → 08`; `06 → 07`; `01..09 → 10`. Items 05, 06 are independent blockers.

## Tier 2 — Expected Features (ship in 1.x)

| # | Item | Goal | Detail File | Dependencies |
|---|------|------|-------------|--------------|
| 11 | Character Controller | Capsule sweep-and-slide with step climbing, slope limits, grounded state | [11-character-controller.md](11-character-controller.md) | — |
| 12 | Vehicle Model | Raycast suspension, tire friction curves, drivetrain, anti-roll bar | [12-vehicle-model.md](12-vehicle-model.md) | — |
| 13 | Ragdoll Articulation | Authoring helpers over existing joints; reduced-coordinate articulation later | [13-ragdoll-articulation.md](13-ragdoll-articulation.md) | — |
| 14 | Joint Completeness | Soft constraints/compliance, position drives on all joint types; gear/pulley | [14-joint-completeness.md](14-joint-completeness.md) | — |
| 15 | Solver Options | Friction-cone block solving, iteration policies, stack-stabilization presets | [15-solver-options.md](15-solver-options.md) | 01 |
| 16 | Batched Async Queries | Batched query API with async contract for worker-thread dispatch | [16-batched-async-queries.md](16-batched-async-queries.md) | 05 |
| 17 | Runtime Collider Mutation | Safe shape replacement/scaling without remove/re-add | [17-runtime-collider-mutation.md](17-runtime-collider-mutation.md) | — |
| 18 | Serialization | Versioned scene save/load, replay recording, network snapshot compression | [18-serialization.md](18-serialization.md) | — |
| 19 | Extra Shapes | Rounded box, ellipsoid; SDF/voxel collision and mutable terrain (design only) | [19-extra-shapes.md](19-extra-shapes.md) | — |
| 20 | Parallel Island Solving | Solve independent islands on the CPU worker pool | [20-parallel-island-solving.md](20-parallel-island-solving.md) | 05 |
| 21 | GPU Resident Stepping | Eliminate per-substep host copies; device-side coloring and event prep | [21-gpu-resident-stepping.md](21-gpu-resident-stepping.md) | — |
| 22 | CUDA Error Recovery | Allocation-failure fallback to CPU, device-loss handling | [22-cuda-error-recovery.md](22-cuda-error-recovery.md) | 21 |
| 23 | Performance Telemetry | Memory/transfer counters in StepStats; benchmark regression tracking in CI | [23-performance-telemetry.md](23-performance-telemetry.md) | — |

**Tier 2 dependency graph:** `05 → 16, 20`; `01 → 15`. Most items are independent.

## Tier 3 — Maturity (post-1.0 polish)

| # | Item | Goal | Detail File | Dependencies |
|---|------|------|-------------|--------------|
| 24 | Cross-Platform Determinism | Fixed-order math and FMA control for lockstep multiplayer | [24-cross-platform-determinism.md](24-cross-platform-determinism.md) | — |
| 25 | Large Worlds (Double Precision) | Double-precision option, scale-aware tolerances | [25-large-worlds-double-precision.md](25-large-worlds-double-precision.md) | — |
| 26 | Interactive Sandbox | Windowed demo + contact/constraint inspector on debug-line API | [26-interactive-sandbox.md](26-interactive-sandbox.md) | — |
| 27 | Platform Coverage | macOS/ARM CI lanes | [27-platform-coverage.md](27-platform-coverage.md) | — |
| 28 | Release Automation | Prebuilt binaries, vcpkg/Conan packaging | [28-release-automation.md](28-release-automation.md) | 06 |
| 29 | Community Infrastructure | Issue templates with repro-scene requests, contribution guide, polished examples | [29-community-infrastructure.md](29-community-infrastructure.md) | — |

**Tier 3 dependency graph:** `06 → 28`. All others independent.

## Design Headers (draft sketches)

Self-contained compilable-in-isolation C++ headers for the five biggest systems:

| File | System |
|------|--------|
| [design/character_controller.h](../design/character_controller.h) | Capsule sweep-and-slide character controller |
| [design/vehicle.h](../design/vehicle.h) | Raycast vehicle model with suspension and drivetrain |
| [design/manifold_clipping.h](../design/manifold_clipping.h) | Sutherland-Hodgman convex face clipping for persistent manifolds |
| [design/serialization.h](../design/serialization.h) | Versioned scene serialization and replay |
| [design/articulation.h](../design/articulation.h) | Reduced-coordinate articulation over existing joints |

## How to Read This Roadmap

1. **Start with Tier 1, item 01.** Every other Tier 1 item either depends on it or is a parallel blocker (thread safety, versioning).
2. **Each detail file** has concrete C++ signatures in the `Public API` section — an engineer can start implementing from those alone.
3. **Size estimates** use S/M/L/XL relative to the existing codebase scale (~5k LOC core).
4. **Risks** call out what is most likely to go wrong; read them before starting implementation.
