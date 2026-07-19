# Getting Started

Velox is a C++17 rigid-body physics library. A `World` owns every body,
joint, collision cache, and snapshot created through it. Keep its handles in
your gameplay objects; do not cache references returned from `World::body()`
across structural changes.

## Build

```powershell
cmake -S . -B build -DVELOX_ENABLE_CUDA=ON
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

`World(BackendType::Auto)` uses CUDA when an NVIDIA backend is available and
otherwise uses the portable CPU backend. The CPU path is supported on Intel
and AMD processors. Use `BackendType::Cpu` for a predictable reference path.

## First Simulation

```cpp
#include <velox/velox.h>

velox::World world(velox::BackendType::Cpu);
world.gravity = {0.0f, -9.81f, 0.0f};
world.addStaticPlane({0.0f, 1.0f, 0.0f}, 0.0f);

const velox::BodyId ball = world.addSphere({0.0f, 5.0f, 0.0f}, 0.5f, 1.0f);
world.body(ball).restitution = 0.25f;

constexpr float dt = 1.0f / 60.0f;
for (int frame = 0; frame < 600; ++frame) {
    world.step(dt);
    const velox::Vec3 position = world.body(ball).position;
    // Synchronize the rendering object from position here.
}
```

Use a fixed timestep. Accumulate render time in your application and call
`step(1.0f / 60.0f)` zero or more times per render frame; do not feed an
unbounded frame delta directly to the solver.

## Bodies and Gameplay Control

The add methods return generation-checked `BodyId` handles. A handle becomes
invalid after its body is removed. Create static level geometry with mass zero
and dynamic bodies with positive mass.

```cpp
const velox::BodyId crate = world.addBox({2.0f, 2.0f, 0.0f},
                                         {0.5f, 0.5f, 0.5f}, 8.0f);
world.addForce(crate, {0.0f, 0.0f, 120.0f});
world.addImpulseAtPoint(crate, {0.0f, 2.0f, 0.0f},
                        world.body(crate).position);
```

Velox supports spheres, boxes, capsules, cylinders, cones, convex hulls,
static planes, static triangle meshes, heightfields, and compound rigid
bodies. Static meshes are level geometry only; dynamic objects must use a
convex primitive, hull, or compound.

## Queries and Events

```cpp
velox::RayHit hit = world.rayCast({0, 3, 0}, {0, -1, 0}, 20.0f);
if (hit.hit) {
    // hit.body, hit.point, hit.normal, and hit.t are valid.
}

for (const velox::ContactEvent& event : world.contactEvents()) {
    if (event.type == velox::ContactEventType::Begin) {
        // Start sound, particles, or gameplay response.
    }
}
```

`rayCastAll`, overlap queries, shape casts, and closest-point queries share
the `QueryFilter` category/mask and sensor policy. Events are valid until the
next `step()` call.

## Debugging and Replay

`World::debugLines()` exports renderer-independent lines for shapes, contacts,
AABBs, and joints. `saveSnapshot()` and `restoreSnapshot()` provide an
in-memory rollback point for one World; serialization APIs persist versioned
scene data and replay recordings.

## Threading and Determinism

Until the thread-safety contract is completed, serialize all access to a
`World` externally: do not query or mutate it concurrently with `step()`.

For deterministic CPU replay across supported compilers, configure with
`-DVELOX_STRICT_FLOATING_POINT=ON`, then select
`world.setDeterminismMode(velox::DeterminismMode::Strict)`. Strict mode uses
the sequential CPU reference backend. CUDA remains a relaxed, high-throughput
backend until ordered device solving has cross-platform parity coverage.
