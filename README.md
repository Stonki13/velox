# Velox

**A fast, tunneling-resistant 3D physics engine for games. C++17, GPU-accelerated.**

Velox (Latin: *swift*) is built around two core promises:

1. **High-speed tunneling prevention.** Continuous collision detection (CCD)
   is a first-class citizen, not a bolted-on flag. Fast-moving bodies are swept
   through time, so bullets don't pass through walls and cars don't fall
   through the floor.
2. **Blazingly fast.** The solver is designed data-oriented from day one so the
   hot loops can run on the GPU (CUDA / compute-shader backend) as well as
   wide-SIMD CPU.

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

## GPU acceleration

The narrow phase (GJK, manifolds, mesh BVH traversal) and the contact solver
are header-only `__host__ __device__` code: the CUDA backend runs **the exact
same physics code as the CPU backend**, over device buffers. Sequential
impulses are inherently serial, so the GPU solver uses **graph coloring** —
contacts are partitioned so no two contacts in a color share a dynamic body,
and each color is solved as one fully parallel kernel. CMake auto-detects a
CUDA toolkit; `World(BackendType::Auto)` picks the GPU when present.

Per-color kernel sweeps are batched with CUDA Graphs, and the coloring is greedy
first-free-bit, which keeps the color count near the max constraints per body.
Contact graphs are captured once per step; joint graphs are reused while body
storage, joint storage, timestep, and constraint topology remain unchanged.
Worlds above the measured joint-workload crossover keep velocity integration,
contact and joint solving, and transform integration on the device across all
solver substeps, then download body state once. Smaller joint sets use the CPU
joint path to avoid graph overhead.

`examples/benchmark` on an RTX 5080 vs the single-threaded CPU reference
(60 Hz steps, full pipeline, 4 solver substeps):

| Scene | CPU | CUDA |
|---|---|---|
| 512-sphere rain | 3.92 ms | **1.60 ms** |
| 2048-sphere rain | 18.07 ms | **5.11 ms** |
| 8192-sphere rain | 120.30 ms | **52.50 ms** |
| 2048 spheres on 20k-triangle terrain | 34.83 ms | **10.53 ms** |

The independent-distance-joint scene added to the same benchmark measures the
new resident constraint path (10-step local Release sample):

| Joints | CPU | CUDA |
|---|---|---|
| 1024 | 2.02 ms | **0.69 ms** |
| 4096 | 8.07 ms | **1.47 ms** |

These are a current local Release run on an RTX 5080. The GPU broad phase is
hybrid: compact all-pairs culling keeps small scenes highly occupied, while
large scenes use GPU sweep-and-prune, overflow-safe candidate compaction, and
a fully parallel narrow-phase pass. The measured crossover is 4096 bodies;
the 8192-body all-pairs reference path takes 183.94 ms instead of 51.86 ms.

The CPU backend uses a persistent worker pool for independent body integration
and deterministic narrow-phase pair batches. `World::setWorkerCount(0)` selects
hardware concurrency; `1` provides a serial reference. Contact batches are
merged in original broad-phase order, so worker count does not change solver
ordering or simulation results. The CPU impulse solver forms contiguous
order-preserving batches that stop at the first shared dynamic body; contacts
inside each batch execute in parallel without changing Gauss-Seidel ordering.
CUDA uses broader conflict-free graph coloring for maximum throughput.
Dense interconnected piles can create short CPU batches, so this deterministic
mode is intentionally conservative; the CUDA backend remains the high-throughput
solver for those scenes.

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
closed 3D polytope cannot exist.

## Features

- Full rigid body dynamics: linear + rotational (quaternions, world-space
  inverse inertia, gyroscopic angular-momentum conservation, contact torques)
- Colliders: **sphere, box, capsule, cylinder, center-of-mass-correct cone,
  dynamic convex hull, static plane, static triangle mesh, and validated static
  heightfield**, plus locally transformed convex compound bodies under one
  rigid-body handle
- **Joints**: ball, distance, hinge, cone/twist, fixed, prismatic, and full
  six-degree-of-freedom constraints. Hinges support torque motors and angle
  limits; prismatic joints support force motors and signed translation limits;
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
- **Body control**: static/kinematic/dynamic motion types, accumulated forces
  and torques, point impulses, per-body damping and gravity scaling
- **Materials**: average/geometric/minimum/multiply/maximum combine modes,
  body-local anisotropic friction, restitution, and load-bounded rolling and
  spinning resistance; all coefficients are shared by the CPU and CUDA solvers
- **Safe object lifetime**: generation-checked body/joint handles, dense
  swap-removal, stale-handle rejection, and automatic attached-joint cleanup
- **Rollback snapshots**: copyable same-world checkpoints restore bodies,
  joints, geometry, stable-handle generations, warm starts, sleeping state,
  and event phase while invalidating CPU/CUDA backend caches for exact replay
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
  of dropping dense-scene pairs. Small joint sets retain the lower-latency CPU
  joint path.
- Broad phase: sweep-and-prune (CPU), hybrid all-pairs / compacted GPU
  sweep-and-prune (CUDA)
- PCS collision pipeline (above) with restitution and accumulated-impulse friction

Mesh colliders are static-only (level geometry), matching how game engines
treat non-convex meshes.

## Build

```bash
cmake -B build
cmake --build build
ctest --test-dir build -C Release --output-on-failure
./build/examples/bullet_demo   # runs the high-speed CCD example
```

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
