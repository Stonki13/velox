# Velox

**A fast, tunneling-proof 3D physics engine for games. C++17, GPU-ready.**

Velox (Latin: *swift*) is built around two core promises:

1. **No clipping. Ever.** Continuous collision detection (CCD) is a first-class
   citizen, not a bolted-on flag. Fast-moving bodies are swept through time, so
   bullets don't pass through walls and cars don't fall through the floor.
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
   the oracle. Tunneling is impossible by construction, at any linear or
   angular speed.

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

Per-color kernel sweeps are batched with CUDA Graphs (captured once per step,
replayed per substep), and the coloring is greedy first-free-bit, which keeps
the color count near the max contacts per body.

`examples/benchmark` on an RTX 5080 vs the single-threaded CPU reference
(60 Hz steps, full pipeline, 4 solver substeps):

| Scene | CPU | CUDA |
|---|---|---|
| 512-sphere rain | 1.44 ms | 1.88 ms |
| 2048-sphere rain | 6.53 ms | 2.95 ms |
| 8192-sphere rain | 35.1 ms | **21.5 ms** |
| 2048 spheres on 20k-triangle terrain | 16.8 ms | **7.6 ms** |

## Solver

Box2D-v3-style TGS: detection runs **once per step**, then several solver
substeps re-evaluate every contact's live gap from current body positions.
Sequential impulses with warm starting (accumulated impulses persist across
frames via persistent contact matching), face-snapped box manifolds for
deterministic stacking, split-impulse positional correction (penetration is
resolved by translation, never by velocity bias — no energy injection), and
whole-island sleeping. A 10-box tower stands still to sub-millimeter drift.

## Features

- Full rigid body dynamics: linear + rotational (quaternions, world-space
  inverse inertia, contact torques)
- Colliders: **sphere, box, capsule, static plane, static triangle mesh**
- **Joints**: ball, distance, hinge (iterative impulses, 3x3 block solve for
  anchors, Baumgarte stabilization)
- **Sleeping & islands**: union-find islands over contacts + joints; settled
  islands cost nothing and wake on impact
- **Queries**: `rayCast` against every collider (BVH-accelerated for meshes),
  `overlapSphere`
- GJK narrow phase over support functions (one code path for all convex
  pairs and mesh triangles); **triangle BVH** per mesh (flat, GPU-traversable)
- **CUDA backend**: integration, narrow phase, and graph-colored contact
  solver all on the GPU (CUDA Graphs batching)
- Broad phase: sweep-and-prune (CPU), compact-AABB pair culling (GPU)
- PCS collision pipeline (above) with restitution and accumulated-impulse friction

Mesh colliders are static-only (level geometry), matching how game engines
treat non-convex meshes.

Roadmap:

- [ ] Convex hull collider (the GJK path already supports it — needs the shape)
- [ ] EPA for exact deep-penetration recovery (rarely hit thanks to PCS)
- [ ] Joint motors and limits; cone/twist for ragdolls
- [ ] Device-resident stepping (skip per-substep transfers when no joints)
- [ ] Collision events/callbacks and filtering layers

## Build

```bash
cmake -B build
cmake --build build
./build/examples/bullet_demo   # proves the no-tunneling claim
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
