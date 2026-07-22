# Velox API Guide

A task-oriented companion to the generated [API reference](api-reference/html/index.html).
This guide shows *how* to use the public API effectively; the Doxygen reference
documents *what* every type and method is. For background on the solver design,
see [concepts](concepts.md); for the build and first simulation, see
[getting started](getting-started.md).

All symbols live in the `velox` namespace and are exposed through the umbrella
header:

```cpp
#include <velox/velox.h>
```

A C binding for embedding and foreign-language interop is available in
[`<velox/velox_c.h>`](../include/velox/velox_c.h).

---

## Quick Start

A `World` owns every body, joint, collision cache, and snapshot created through
it. You drive it with a fixed timestep and read results back through
generation-checked handles.

```cpp
#include <velox/velox.h>

int main() {
    // 1. Create a world. Auto uses CUDA when available, else the CPU backend.
    velox::World world(velox::BackendType::Auto);
    world.setGravity({0.0f, -9.81f, 0.0f});

    // 2. Add static level geometry (mass is implicit for the addStatic* calls).
    world.addStaticPlane({0.0f, 1.0f, 0.0f}, 0.0f);

    // 3. Add a dynamic body. mass > 0 makes it dynamic.
    velox::BodyId ball = world.addSphere({0.0f, 5.0f, 0.0f}, 0.5f, 1.0f);
    world.body(ball).restitution = 0.25f; // borrowed reference, see below

    // 4. Step with a fixed timestep and read the result.
    constexpr float dt = 1.0f / 60.0f;
    for (int frame = 0; frame < 600; ++frame) {
        world.step(dt);
        velox::Vec3 p = world.body(ball).position;
        // ...synchronize your render object from p...
    }
}
```

### The fixed-timestep loop

Do not feed an unbounded render-frame delta to the solver. Accumulate elapsed
time and consume it in fixed increments:

```cpp
constexpr float dt = 1.0f / 60.0f;
float accumulator = 0.0f;

void onFrame(float frameSeconds) {
    accumulator += frameSeconds;
    while (accumulator >= dt) {
        world.step(dt);
        accumulator -= dt;
    }
    // Optionally interpolate rendering between the last two states using
    // accumulator / dt as the blend factor.
}
```

### Handles, not pointers

`addSphere`, `addBox`, `addJoint...`, etc. return lightweight `BodyId` /
`JointId` handles that pack a slot and a generation counter. A handle becomes
invalid when its body is removed and its slot reused; validate retained handles
before use:

```cpp
if (world.isValid(id)) {
    world.addForce(id, {0, 10, 0});
}
```

Store handles in your gameplay objects. **Do not** store the references returned
by `world.body(id)` / `world.joint(id)` — they are borrowed and may be
invalidated by the next `step()` or structural change.

---

## Common Patterns and Best Practices

### Creating bodies

| Goal | Call |
| --- | --- |
| Dynamic sphere / box / capsule | `addSphere`, `addBox`, `addCapsule` (mass > 0) |
| Cylinder / cone / rounded box / ellipsoid | `addCylinder`, `addCone`, `addRoundedBox`, `addEllipsoid` |
| Arbitrary convex shape | `addConvexHull(position, points, mass)` |
| Multi-part rigid body | `addCompound(position, shapes, mass)` |
| Infinite ground plane | `addStaticPlane(normal, offset)` |
| Level geometry | `addStaticMesh(vertices, indices)` |
| Terrain | `addStaticHeightfield(width, depth, cellSize, heights, origin)` |

Dynamic colliders must be convex (primitive, hull, or compound). Static triangle
meshes and heightfields are for level geometry only — they are never simulated
as moving bodies.

```cpp
// A compound "capsule + box" rigid body.
std::vector<velox::CompoundShape> parts(2);
parts[0].shape = velox::ShapeType::Capsule;
parts[0].radius = 0.3f;
parts[0].capsuleHalfHeight = 0.6f;
parts[1].shape = velox::ShapeType::Box;
parts[1].halfExtents = {0.4f, 0.1f, 0.4f};
parts[1].localPosition = {0.0f, 0.9f, 0.0f};
velox::BodyId character = world.addCompound({0, 2, 0}, parts, 70.0f);
```

> **Note:** A dynamic compound is recentered on its computed center of mass, so
> `Body::position` is the center of mass, not the authored origin.

### Applying forces and impulses

Forces accumulate over the step; impulses apply instantly. The `...AtPoint`
variants also induce torque.

```cpp
world.addForce(id, {0, 0, 120});                 // sustained thrust (N)
world.addTorque(id, {0, 5, 0});                  // sustained torque (N*m)
world.addLinearImpulse(id, {0, 8, 0});           // instantaneous kick (N*s)
world.addImpulseAtPoint(id, {2, 0, 0}, worldPos);// off-center kick → spin
world.clearForces(id);                           // cancel accumulated force/torque
```

### Controlling motion

```cpp
world.setMotionType(id, velox::MotionType::Kinematic); // script-driven body
world.setLinearVelocity(id, {1, 0, 0});
world.setFixedRotation(id, true);   // e.g. a character that must stay upright
world.setGravityScale(id, 0.0f);    // float in place
world.setLinearDamping(id, 0.1f);   // gentle air drag
```

### Collision filtering

Each body carries `categoryBits` (what it *is*) and `maskBits` (what it
*collides with*). Two bodies collide only when each accepts the other's
category. Queries share the same model through `QueryFilter`.

```cpp
// Player is category 2, collides with world (1) and enemies (4), not pickups (8).
world.setCollisionFilter(player, /*category*/ 0x2, /*mask*/ 0x1 | 0x4);

velox::QueryFilter filter;
filter.maskBits = 0x1;            // only hit the "world" category
filter.includeSensors = false;    // skip sensor volumes
velox::RayHit hit = world.rayCast(from, dir, 100.0f, filter);
```

### Sensors

A sensor reports contacts but does not resolve them — ideal for trigger
volumes. Toggle with `setSensor(id, true)` and watch for `ContactEvent`s whose
`sensor` flag is set.

### Reading events

Contact, body-lifecycle, and joint-break events are produced by `step()` and
remain valid until the next `step()`:

```cpp
world.step(dt);

for (const velox::ContactEvent& e : world.contactEvents()) {
    if (e.type == velox::ContactEventType::Begin && !e.sensor) {
        // e.a, e.b, e.point, e.normal, e.impulse
    }
}
for (const velox::JointBreakEvent& e : world.jointBreakEvents()) {
    // a joint exceeded its breakForce / breakTorque
}
```

### Joints

Create a joint, then configure motors and limits through the returned reference:

```cpp
velox::JointId hinge = world.addHingeJoint(door, frame, anchor, axis);
velox::Joint& j = world.joint(hinge); // borrowed reference
j.enableLimit = true;
j.lowerLimit = 0.0f;
j.upperLimit = 1.5f;        // radians
j.enableMotor = true;
j.motorSpeed = 0.5f;        // rad/s
j.maxMotorTorque = 10.0f;   // N*m budget
j.breakForce = 5000.0f;     // optional: snap under load

float angle = world.hingeAngle(hinge); // read back, radians, 0 at creation
```

Available joints: ball, distance, spring, hinge, cone-twist, fixed, prismatic,
six-DOF, and motor. Read joint state with `hingeAngle`, `coneSwingAngle`,
`prismaticTranslation`, `sixDofLinearTranslation`, etc.

### Scene queries

```cpp
// Nearest ray hit.
velox::RayHit hit = world.rayCast(origin, dir, maxDist);

// Every hit, nearest first.
std::vector<velox::RayHit> hits;
world.rayCastAll(origin, dir, maxDist, hits);

// Overlap tests.
std::vector<velox::BodyId> inside;
world.overlapSphere(center, radius, inside);
world.overlapBox(center, halfExtents, orientation, inside);

// Swept shapes (continuous).
velox::ShapeCastHit sweep = world.sphereCast(center, radius, dir, maxDist);

// Gap between two bodies (negative ⇒ overlapping).
velox::ClosestPointResult gap = world.closestPoints(a, b);
```

For issuing many queries at once, or from another thread, use the batched and
async APIs — see [batched and async queries](batched-queries.md).

### Snapshots and rollback

```cpp
velox::WorldSnapshot checkpoint = world.saveSnapshot();
// ...simulate speculatively...
world.restoreSnapshot(checkpoint); // roll back to the checkpoint
```

A snapshot is owned by the world that created it. For persisting scenes to disk,
use the serialization APIs (see [serialization](serialization.md)).

### Customizing contacts

Install a callback to adjust or disable individual contacts before solving:

```cpp
world.setContactModifier([](velox::ContactModifyData& c) {
    if (c.a == slippery || c.b == slippery) {
        c.friction1 = c.friction2 = 0.0f; // ice
    }
    if (c.a == ghost || c.b == ghost) {
        c.enabled = false; // pass through
    }
});
```

> **Warning:** Throwing from the modifier aborts `step()` before any transforms
> advance.

---

## Thread Safety Guidelines

`World` is created with `ThreadSafetyPolicy::Strict`: every call must come from
the thread that constructed it. This is the lowest-overhead mode for a normal
game loop. Choose a more permissive policy *before* sharing the world. See the
full [threading contract](threading.md).

| Policy | Queries from other threads | Mutations from other threads | `step()` |
| --- | --- | --- | --- |
| `Strict` (default) | rejected | rejected | owner thread only |
| `Relaxed` | allowed, serialized | rejected | owner thread only |
| `Concurrent` | allowed, serialized | allowed, serialized | owner thread only |

```cpp
velox::World world;
world.setThreadSafetyPolicy(velox::ThreadSafetyPolicy::Concurrent);
// Workers may now call rayCast/overlap*/*Cast/bodyState and the supported
// mutations while the owner thread calls step().
```

Rules of thumb:

- **Prefer value copies across threads.** `bodyState()`, `jointState()`, and
  `lastStepStatsCopy()` return owned values that stay valid after the world
  advances. The borrowed references from `body()`, `joint()`, `lastStepStats()`,
  `contactEvents()`, and `jointBreakEvents()` require external synchronization
  and must not be retained across `step()`.
- **Configure before sharing.** The public `gravity` and `substeps` fields are
  not internally synchronized. After the world is shared, use `setGravity()`,
  `gravityValue()`, `setSubsteps()`, and `substepCount()`.
- **Async queries are always allowed.** `submitAsyncQuery()` copies a
  value-owned request and returns immediately from any thread, even under
  `Strict`. The owner resolves it at the start of the next `step()`; consume the
  result with `getAsyncResult()`. This never exposes partially solved state.
- **`step()` is owner-thread only** under every policy.
- **World destruction** and any access through a borrowed reference always
  require external synchronization.
- Use `threadSafetyReport()` in tests to assert the intended access pattern
  (it is a correctness counter, not a profiler).

---

## Performance Tips

- **Use a fixed timestep** and keep `dt` stable. Large or variable steps hurt
  solver convergence and CCD quality.
- **Tune substeps for stiffness.** `world.substeps` (or `setSubsteps`) controls
  solver substeps per `step()`. More substeps give stiffer stacks and less
  friction drift for the same iteration budget; fewer are cheaper. The default
  is 4.
- **Let bodies sleep.** Sleeping islands cost nothing. Keep `enableSleep` on for
  ordinary dynamic bodies, and call `wake(id)` after you manually move a
  sleeping body. Use `setEnableSleep(id, false)` only for bodies that must
  always simulate.
- **Prefer cheap colliders.** Spheres are the fastest, then capsules and boxes.
  Use convex hulls and compounds only when the shape truly needs them, and keep
  hull point clouds small (interior points still cost support-function time).
- **Filter early.** Set `categoryBits`/`maskBits` so unrelated bodies never
  generate narrow-phase tests, and pass a `QueryFilter` to skip categories and
  sensors you do not need.
- **Batch queries.** `batchQueries()` resolves many requests against one
  consistent world state in a single call, avoiding repeated locking and
  broad-phase lookups.
- **Parallelize islands.** `setIslandSolvingMode(IslandSolvingMode::Parallel)`
  (the default) solves independent contact islands concurrently on the worker
  pool with bitwise-identical results. Adjust the pool with `setWorkerCount()`,
  or inject your own scheduler with `setTaskSystem()`.
- **GPU backend.** Build with `VELOX_ENABLE_CUDA` and use `BackendType::Auto` or
  `Cuda` for large scenes. `setGPUResidentMode(GPUResidentMode::Resident)` keeps
  data on the device across steps, eliminating per-substep PCIe transfers when
  the workload allows it.
- **Keep large worlds near the origin.** Floating-point precision degrades far
  from `(0,0,0)`. Call `world.shiftOrigin(offset)` periodically to recenter the
  world without rebuilding bodies, joints, or meshes.
- **Profile with the built-in stats.** `lastStepStats()` / `lastStepStatsCopy()`
  break down broad-phase, narrow-phase, solver, and CCD time so you can target
  the real bottleneck. `debugLines()` exports shapes, AABBs, contacts, and
  joints for visual debugging.
- **Raise geometry quality only when needed.** `GeometryQuality::Robust` /
  `Paranoid` protect against imported or fuzzed meshes at extra cost; leave
  authored content at `Normal`.

---

## Migration Guide from Other Engines

Velox's API is intentionally small: one `World`, generation-checked handles, and
plain-data bodies. The table and notes below map common concepts from other
engines.

### Concept mapping

| Concept | Box2D (v2/v3) | Bullet | PhysX | Velox |
| --- | --- | --- | --- | --- |
| World | `b2World` | `btDiscreteDynamicsWorld` | `PxScene` | `velox::World` |
| Body handle | `b2BodyId` / `b2Body*` | `btRigidBody*` | `PxRigidActor*` | `velox::BodyId` |
| Body types | static/kinematic/dynamic | static/kinematic/dynamic | static/kinematic/dynamic | `MotionType::Static/Kinematic/Dynamic` |
| Shape | `b2Shape` | `btCollisionShape` | `PxShape` | `ShapeType` + `Body` fields, `CompoundShape` |
| Step | `b2World::Step` | `stepSimulation` | `PxScene::simulate/fetchResults` | `World::step(dt)` |
| Joint | `b2*Joint` | `btTypedConstraint` | `PxJoint` | `World::add*Joint`, `velox::Joint` |
| Raycast | `b2World::CastRay` | `rayTest` | `PxScene::raycast` | `World::rayCast` / `rayCastAll` |
| Contact events | contact listeners | contact callbacks | contact callbacks | `World::contactEvents()` |
| Filtering | filter bits | collision groups | filter data | `categoryBits` / `maskBits` |

### From Box2D

- Velox follows the Box2D v3 handle model: `BodyId`/`JointId` are
  generation-checked values, not pointers. `world.isValid(id)` replaces
  dangling-pointer checks.
- `World::substeps` mirrors Box2D v3's substep approach. A Box2D
  `Step(dt, subStepCount, iterations)` maps to setting `world.substeps` (and
  solver options) then calling `world.step(dt)`.
- Box2D's `b2BodyDef::type` maps to `MotionType`. Velox infers dynamic vs
  static from mass at creation (`mass > 0` is dynamic) and via the dedicated
  `addStatic*` methods.
- Contact listening: instead of registering a listener object, drain
  `world.contactEvents()` after each `step()`. `Begin/Persist/End` correspond to
  Box2D's begin/pre-solve/post-solve/end semantics; per-contact mutation is done
  with `setContactModifier`.

### From Bullet

- Bullet's separate `btCollisionShape` objects are replaced by per-body shape
  fields plus `CompoundShape` vectors. There is no shape-sharing object to
  manage; geometry is value-owned by the world.
- `btDiscreteDynamicsWorld::stepSimulation(dt, maxSubSteps, fixedDt)` maps to
  the fixed-timestep loop above with `world.substeps`.
- `btTypedConstraint` motors/limits map to fields on `velox::Joint` obtained via
  `world.joint(id)` (e.g. `enableMotor`, `motorSpeed`, `enableLimit`,
  `lowerLimit`, `upperLimit`).
- `rayTest` closest results map to `rayCast`; `rayTestAll` to `rayCastAll`.

### From PhysX

- `PxScene::simulate` + `fetchResults` collapses into a single synchronous
  `world.step(dt)`; results (events, transforms) are read immediately afterward.
- `PxRigidStatic`/`PxRigidDynamic`/`PxRigidKinematic` map to `MotionType` and the
  `addStatic*` vs dynamic `add*` creation calls.
- `PxFilterData` word masks map to `categoryBits`/`maskBits` and `QueryFilter`.
- `PxJoint` types map one-to-one onto the `World::add*Joint` methods.
- PhysX `PxScene` requires a CPU dispatcher; Velox manages its own worker pool
  (`setWorkerCount`) or accepts an external `TaskSystem` (`setTaskSystem`).

### General porting checklist

1. Replace pointer body/joint storage with `BodyId`/`JointId` handles and guard
   with `isValid`.
2. Move to a fixed-timestep loop; set `world.substeps` for stiffness.
3. Recreate static geometry with `addStaticPlane` / `addStaticMesh` /
   `addStaticHeightfield`; dynamic colliders must be convex.
4. Convert material setup to per-body fields (`restitution`, `friction`,
   damping, `gravityScale`) and combine modes.
5. Convert constraints to `add*Joint` + `world.joint(id)` configuration.
6. Replace event-listener objects with post-`step()` drains of `contactEvents()`,
   `bodyEvents()`, and `jointBreakEvents()`.
7. If you need cross-thread queries or determinism, pick a
   `ThreadSafetyPolicy` and, for reproducible CPU replay, build with
   `VELOX_STRICT_FLOATING_POINT` and select `DeterminismMode::Strict`.

---

## See Also

- [Getting started](getting-started.md) — build and first simulation
- [Concepts](concepts.md) — how the solver, broad/narrow phase, and CCD work
- [Threading contract](threading.md) — full policy semantics
- [Batched and async queries](batched-queries.md) — frame-boundary query contract
- [CCD controls](ccd-controls.md) — continuous collision detection tuning
- [Generated API reference](api-reference/html/index.html) — every type and method
