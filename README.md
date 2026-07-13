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

The narrow phase (GJK, manifolds, mesh BVH traversal) is header-only
`__host__ __device__` code: the CUDA backend runs **the exact same collision
code as the CPU backend**, over device buffers. CMake auto-detects a CUDA
toolkit; `World(BackendType::Auto)` picks the GPU when present. The contact
solver currently stays on the CPU (sequential impulses are serial by nature);
a graph-colored GPU solver is the next milestone.

`examples/benchmark` on an RTX 5080 vs the CPU reference (Ryzen host, 60 Hz
steps, contact solver included — it is shared and dominates at scale):

| Scene | CPU | CUDA |
|---|---|---|
| 512-sphere rain | 0.58 ms | 0.77 ms |
| 2048-sphere rain | 2.86 ms | 2.02 ms |
| 8192-sphere rain | 17.4 ms | 10.0 ms |
| 2048 spheres on 20k-triangle terrain | 5.97 ms | 5.19 ms |

## Status

Early development. Working today:

- Full rigid body dynamics: linear + rotational (quaternions, world-space
  inverse inertia, contact torques)
- Colliders: **sphere, box, capsule, static plane, static triangle mesh**
- GJK narrow phase over support functions (one code path for all convex
  pairs and mesh triangles), with contact manifolds for resting boxes
- **Triangle BVH** per mesh (median split, flat GPU-traversable layout)
- **CUDA backend**: integration + full narrow phase on the GPU
- Broad phase: sweep-and-prune (CPU), compact-AABB pair culling (GPU)
- PCS collision pipeline (above) with restitution and accumulated-impulse friction

Mesh colliders are static-only (level geometry), matching how game engines
treat non-convex meshes.

Roadmap:

- [ ] GPU contact solver (graph coloring / Jacobi) — the current bottleneck
- [ ] Convex hull collider (the GJK path already supports it — needs the shape)
- [ ] EPA for exact deep-penetration recovery (rarely hit thanks to PCS)
- [ ] Persistent contact manifolds + warm starting
- [ ] Constraint solver (joints), islands, sleeping

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
