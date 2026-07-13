# Velox

**A fast, tunneling-proof 3D physics engine for games. C++17, GPU-ready.**

Velox (Latin: *swift*) is built around two core promises:

1. **No clipping. Ever.** Continuous collision detection (CCD) is a first-class
   citizen, not a bolted-on flag. Fast-moving bodies are swept through time, so
   bullets don't pass through walls and cars don't fall through the floor.
2. **Blazingly fast.** The solver is designed data-oriented from day one so the
   hot loops can run on the GPU (CUDA / compute-shader backend) as well as
   wide-SIMD CPU.

## Status

Early development. Working today:

- Rigid body dynamics (semi-implicit Euler integrator)
- Sphere and static-plane colliders
- **Swept-sphere CCD** — a sphere moving at any speed cannot tunnel through a plane
- Impulse-based collision response with restitution and friction
- Clean backend interface (`velox::Backend`) so a GPU solver can drop in

Roadmap:

- [ ] Boxes, capsules, convex hulls (GJK/EPA)
- [ ] Conservative-advancement CCD for rotating bodies
- [ ] Broad phase: sweep-and-prune → GPU BVH
- [ ] CUDA backend for integration + narrow phase
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
