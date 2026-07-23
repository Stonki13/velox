# Velox

**C++17 3D rigid-body physics with CCD, persistent contact manifolds, strict CPU replay, and optional NVIDIA CUDA acceleration.**

Velox 1.0 is a released, MIT-licensed physics library for games and realtime
simulation. It provides a portable CPU backend, an optional CUDA backend, C and
C++ APIs, deterministic CPU replay mode, a raycast vehicle, a query-only capsule
character controller, and a renderer-independent debug-line interface.

## Capabilities

- Predictive Contact Sweeping combines speculative contacts with conservative
  advancement for supported continuous-collision paths.
- Clipped persistent manifolds retain feature keys and warm-start impulses for
  resting contact and friction constraints.
- Strict mode provides cross-platform CPU trace regression coverage across the
  supported build matrix.
- The CPU backend uses stable ordering and worker parallelism for eligible work;
  the CUDA backend targets NVIDIA GPUs.

> **Support boundary:** Strict determinism is a CPU reference mode. CPU and CUDA
> use different constraint execution orders and are not lockstep-identical.
> CUDA is NVIDIA-only; Intel and AMD systems use the portable CPU backend.

## Predictive Contact Sweeping (PCS)

Classic CCD picks one of two designs and inherits its flaws: event-driven
time-of-impact stepping (exact, but stalls on piles and makes grazing contact
sticky) or speculative contacts alone (smooth, but an iterative solver can
still let extreme cases slip). Velox layers them:

1. **Speculative detection** — contacts are created while pairs are still
   apart, whenever relative motion could close the gap this step.
2. **Iterative velocity solve** — each contact removes only approach velocity
   in excess of `gap/dt`. Grazing bodies keep their speed; piles are resolved
   by solver iterations, so there is no event queue to stall.
3. **Conservative-advancement safety net** — after integration, any pair that
   ended the step interpenetrating is rewound along its actual motion
   (rotation included) to the moment of first contact, using GJK distance as
   the oracle. Dynamic pairs rewind to the same TOI, receive a
   momentum-preserving impulse, and finish the remaining frame together.
   Supported collider paths are protected without a velocity threshold, for
   both linear and angular motion.

`examples/stress_demo` locks all of this in: 2 km/s bullets, grazing skims,
a 125-sphere pile, 3 km/s head-on impacts, a 1.5 km/s spinning box, resting
stability for every shape, and a ball settling inside a triangle-mesh trough.

## CUDA acceleration

The narrow phase (GJK, manifolds, and mesh BVH traversal) uses device-compatible
code, and the CUDA backend executes against device buffers. Sequential impulses
are inherently serial, so the CUDA solver uses **graph coloring**:
contacts are partitioned so no two contacts in a color share a dynamic body,
and each color is solved as one fully parallel kernel. CMake detects a CUDA
toolkit when available; `World(BackendType::Auto)` selects CUDA only when Velox
was built with CUDA and a usable NVIDIA device is present. The different solver
ordering means CUDA is validated for physical tolerance, not CPU lockstep replay.

Per-color kernel sweeps are batched with CUDA Graphs, and the coloring is greedy
first-free-bit, which keeps the color count near the max constraints per body.
Contact graphs are captured once per step; joint graphs are reused while body
storage, joint storage, timestep, and constraint topology remain unchanged.
Worlds above the measured joint-workload crossover keep velocity integration,
contact and joint solving, and transform integration on the device across all
solver substeps, then download body state once. Smaller joint sets use the CPU
joint path to avoid graph overhead.

Performance is workload- and hardware-dependent. Run `examples/benchmark` on
your target hardware and use [the performance guide](docs/performance.md) for
the measurement protocol, tuning controls, and regression workflow.

## Solver

Box2D-v3-style TGS: detection runs **once per step**, then several solver
substeps re-evaluate every contact's live gap from persistent local anchors and
the bodies' current transforms. Sequential impulses with warm starting
(normal, two-axis Coulomb friction, rolling, and spinning impulses persist
through stable contact feature keys, with proximity fallback for topology-free
GJK contacts),
face-snapped box and convex-hull manifolds for deterministic stacking,
split-impulse positional correction (penetration is resolved by translation,
never by velocity bias — no energy injection), and whole-island sleeping. A
10-box tower remains standing in the regression suite on both CPU and CUDA.

Torque-free anisotropic bodies use an implicit-midpoint gyroscopic update.
As the body-space inertia tensor rotates, the integrator preserves world-space
angular momentum and bounds rotational-energy drift instead of treating angular
velocity as constant. The same device-compatible path runs on CPU and CUDA.

Deep convex-core overlaps use a fixed-capacity **EPA** solver shared by CPU and
CUDA. EPA returns the minimum translation normal, penetration depth, and core
witness points; lower-dimensional cores retain a conservative fallback when a
closed 3D polytope cannot exist. Minkowski support arithmetic is centered on
each convex transform before subtraction, avoiding translation-dependent loss
of distance precision; deterministic randomized metamorphic coverage checks
operand symmetry, common-translation invariance, and witness consistency.

## Features

- Full rigid body dynamics: linear + rotational (quaternions, world-space
  inverse inertia, analytic primitive and convex-polyhedron mass properties,
  automatic hull center-of-mass recentering, gyroscopic angular-momentum
  conservation, and contact torques)
- Colliders: **sphere, box, capsule, cylinder, center-of-mass-correct cone,
  dynamic convex hull, static plane, static triangle mesh, and validated static
  heightfield**, plus locally transformed convex compound bodies under one
  rigid-body handle. Dynamic hulls and uniform-density compounds compute and
  recenter their exact center of mass and full inertia tensor before reducing
  it to principal moments/axes. Incremental 3D QuickHull extracts mass faces
  from arbitrary point clouds without charging interior points combinatorially.
- **Joints**: ball, distance, hinge, cone/twist, fixed, prismatic, motor, and full
  six-degree-of-freedom constraints. Hinges support torque motors and angle
  limits; prismatic joints support force motors and signed translation limits;
  motor joints provide position-controlled constraints with max force/torque clamping;
  6DoF joints independently free, lock, limit, or velocity-motor all three
  linear and angular axes, with per-axis force/torque budgets and
  exponential-map rotation state. A 6DoF joint locks all axes by default; clear
  a `JointAxisX/Y/Z` bit in `linearLimitMask` or `angularLimitMask` to free that
  joint-frame axis, or set unequal lower/upper bounds to limit it. Ragdolls
  have independent swing/twist limits and distance constraints support mass-independent
  frequency/damping springs (iterative impulses, 3x3 block solves, Baumgarte
  stabilization). Connected bodies ignore each other by default;
  `Joint::collideConnected` opts back in.
  Per-joint force/torque thresholds support deferred breaking with observable
  break events and generation-safe stale handles.
- **Ragdoll authoring**: `RagdollBuilder` validates connected bone trees over
  existing bodies, applies per-bone mass tuning, creates limited cone-twist
   links or motorized hinge links, and can wake/query the resulting rig.
- **Gameplay helpers**: a query-only capsule `CharacterController` for
  sweep-and-slide movement and a raycast `Vehicle` with suspension, steering,
  drivetrain, and braking controls.
- **Body control**: static/kinematic/dynamic motion types, accumulated forces
  and torques, point impulses, per-body damping and gravity scaling, custom
  mass properties with independently oriented principal-inertia axes, sensor flags,
  sleep control, fixed rotation, and collision filter masks
- **Body lifecycle events**: Created/Destroyed/Moved events for gameplay logic
- **Explosion API**: radial impulse with linear falloff for gameplay effects
- **C API wrapper**: `include/velox/velox_c.h` provides FFI-compatible bindings
  for integration with any language/engine without C++ dependency
- **Materials**: average/geometric/minimum/multiply/maximum combine modes,
  body-local anisotropic friction, restitution, and load-bounded rolling and
  spinning resistance; all coefficients are shared by the CPU and CUDA solvers
- **Safe object lifetime**: generation-checked body/joint handles, dense
  swap-removal, stale-handle rejection, and automatic attached-joint cleanup
- **Rollback snapshots**: copyable same-world checkpoints restore bodies,
  joints, geometry, stable-handle generations, warm starts, sleeping state,
   and event phase while invalidating CPU/CUDA backend caches for exact replay
- **Serialization**: versioned scene archives and replay recording APIs for
  persistent simulation data and compatibility-checked loading.
- **Large-world origin shifting**: one transactional call rebases bodies,
  planes, static mesh/BVH geometry, CCD history, warm starts, and event points
  while preserving relative dynamics and stable handles
- **Frame diagnostics**: body/awake/contact/joint workload counters plus wall
  timings for setup, collision detection, solving, CCD recovery, and
  finalization; `deviceSubsteps` reports whether the complete solver substep
  loop stayed on the GPU
- **Debug drawing**: renderer-agnostic colored line lists for collider
  wireframes, compound children, mesh triangles, AABBs, contact normals, and
  joint anchors/axes with independently selectable flags
- **Sleeping & islands**: union-find islands over contacts + joints; settled
  islands cost nothing and wake on impact
- **Queries**: symmetrically filtered raycasts, sphere/box/capsule/convex-hull
  overlaps, and conservative-advancement casts for the same shapes against
  every collider (BVH-accelerated for meshes), with sensor and ignored-body
  policy
- **Collision policy**: symmetric category/mask filtering and non-responsive
  sensors, with pair-level Begin/Persist/End events, stable body handles,
  representative contact point/normal, and solved normal impulse. A validated
  host contact modifier can alter contact geometry/materials or disable
  individual manifold points before either backend solves them.
- GJK narrow phase over support functions (one code path for all convex
  pairs and mesh triangles); **triangle BVH** per mesh (flat, GPU-traversable)
- **CUDA backend**: integration, narrow phase, graph-colored contacts, and
  high-throughput joint sets stay on the GPU through every solver substep
  (CUDA Graphs batching and topology reuse), including per-substep breaking.
  Contact and broad-phase candidate buffers count-and-grow on overflow instead
  of dropping dense-scene pairs. Operational CUDA failures propagate as
  exceptions with the failing API expression and source location; cleanup
  remains non-throwing. Small joint sets retain the lower-latency CPU joint path.
- Broad phase: **incremental dynamic AABB tree** (CPU) — fat proxies with
  escape-check refits, SAH-guided insertion, AVL-style rotations; only bodies
  whose swept bounds leave their fat proxy reinsert, so mostly-static and
  mostly-sleeping scenes skip per-frame broad-phase rebuilds entirely. The
  same tree accelerates raycasts, overlaps, and shape casts (previously
  linear scans). Structural mutations (removal, snapshot restore, origin
  shift) rebuild; direct `body()` mutation re-fits lazily before the next
  query. GPU: hybrid all-pairs / compacted sweep-and-prune (CUDA). Static
  meshes participate by root AABB rather than as unbounded geometry
- PCS collision pipeline (above) with restitution and accumulated-impulse friction

Mesh colliders are static-only (level geometry), matching how game engines
treat non-convex meshes.

## Build

```bash
cmake -S . -B build -DVELOX_ENABLE_CUDA=OFF
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

This builds the portable CPU backend. Set `-DVELOX_ENABLE_CUDA=ON` on an NVIDIA
system with a CUDA toolkit to build CUDA acceleration. On Visual Studio, run
`build\\examples\\Release\\bullet_demo.exe` for the CCD example; on a
single-config generator, run `./build/examples/bullet_demo`.

## Documentation

- [Getting started](docs/getting-started.md) covers the supported integration
  flow, queries, events, debugging, and determinism modes.
- [Debugging and debug visualization](docs/debugging.md) explains the
  renderer-agnostic debug-draw layers, the `DebugDraw` interface, and the
  Dear ImGui interactive overlay.
- [Concepts](docs/concepts.md) describes Predictive Contact Sweeping, TGS,
  manifolds, islands, and backend tradeoffs.
- [Performance](docs/performance.md) explains `StepStats`, benchmark workflow,
  and the CPU/CUDA tuning tradeoffs.
- [Portability and determinism](docs/PORTABILITY.md) and
  [cross-platform replay](docs/cross-platform.md) document supported CPU
  targets and the strict replay contract.
- [Threading contract](docs/threading.md) specifies safe cross-thread world
  access and the legacy borrowed-reference limits.
- [Batched and async queries](docs/batched-queries.md) describes ordered
  batched reads and nonblocking worker submission at frame boundaries.
- [CCD quality controls](docs/ccd-controls.md) explains per-body continuous
  collision tuning, locked bodies, and the current multi-TOI query boundary.
- [CUDA recovery](docs/cuda-recovery.md) documents automatic CPU fallback,
  explicit throw policy, and controlled CUDA backend restoration.
- [Packaging and releases](docs/packaging.md) covers `find_package(Velox)`,
  Conan source packages, and tagged release artifacts.
- [Serialization](docs/serialization.md) documents scene archives, replay
  recordings, and compatibility handling.
- [Contributing](docs/CONTRIBUTING.md) describes development gates, CUDA
  compatibility requirements, and how to report a reproducible physics issue.
- [Real-workload release gate](docs/release-gate.md) defines the game-like
  CTest workload used to catch cross-subsystem regressions.
- [Production readiness](docs/production-readiness.md) lists the required
  hosted CI, CUDA hardware, packaging, performance, and release gates.
- [Known limitations](docs/known-limitations.md) lists honest gaps versus
  Jolt Physics, known pre-existing test issues, and how to attach a
  reproducible scene or replay trace to a bug report.
- [C API reference](include/velox/velox_c.h) provides FFI-compatible bindings
  for integration with C, Rust, Python, and other languages.
- `doxygen docs/Doxyfile` generates API reference HTML in `docs/api-reference`.

## Quick taste

```cpp
velox::World world;
world.gravity = {0, -9.81f, 0};

auto ground = world.addStaticPlane({0, 1, 0}, 0.0f);
auto bullet = world.addSphere({0, 1, 0}, 0.05f, 0.01f);
world.body(bullet).velocity = {0, -2000, 0};   // 2 km/s straight down

world.step(1.0f / 60.0f);   // CCD catches the impact mid-step. No tunneling.
```

## License

MIT
