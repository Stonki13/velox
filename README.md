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
3. **Exact sweep safety net** — after integration, every body is swept along
   the displacement it actually made; anything the solver let slip is clamped
   to its exact time of impact. Tunneling is impossible by construction.

`examples/stress_demo` locks all of this in: a 2 km/s bullet, a grazing skim,
a 125-sphere pile, a 3 km/s head-on impact, and long-run resting stability.

## Status

Early development. Working today:

- Rigid body dynamics (semi-implicit Euler integrator)
- Sphere and static-plane colliders
- PCS collision pipeline (above) with restitution and accumulated-impulse friction
- Clean backend interface (`velox::Backend`) so a GPU solver can drop in

Roadmap:

- [ ] Boxes, capsules, convex hulls (GJK/EPA)
- [ ] Rotational dynamics + conservative-advancement sweeps for spinning bodies
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
