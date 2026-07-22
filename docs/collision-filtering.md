# Collision Filtering

Velox provides a flexible, three-tier collision filtering system that controls which body pairs generate contacts. Filtering is evaluated in a fixed priority order:

1. **Group index** — same-group override (highest priority)
2. **Layer bits** — 32-bit category/mask test
3. **Custom callback** — user-supplied override (CPU broadphase only)

## Quick Start

```cpp
#include <velox/velox.h>

velox::World world(velox::BackendType::Cpu);
world.setGravity({0, -9.81f, 0});

// Static floor on the Static layer, accepting Dynamic bodies
velox::BodyId floor = world.addStaticPlane({0, 1, 0}, 0.0f);
world.setCollisionFilter(floor,
    velox::CollisionLayers::Static,                              // category
    velox::CollisionLayers::Dynamic | velox::CollisionLayers::Character); // mask

// Dynamic ball on the Dynamic layer, colliding with Static geometry
velox::BodyId ball = world.addSphere({0, 5, 0}, 0.5f, 1.0f);
world.setCollisionFilter(ball,
    velox::CollisionLayers::Dynamic,
    velox::CollisionLayers::Static);
```

## Layer-Based Filtering

Each body has two 32-bit fields:

| Field | Meaning |
|-------|---------|
| `categoryBits` | Layers this body **belongs to** |
| `maskBits` | Layers this body **collides with** |

Two bodies collide only when the test is bidirectional:

```
(a.maskBits & b.categoryBits) != 0  AND  (b.maskBits & a.categoryBits) != 0
```

### Named Layers

Velox provides named constants for common layers in `velox::CollisionLayers`:

| Constant | Bit | Typical Use |
|----------|-----|-------------|
| `Default` | 0 | General-purpose |
| `Static` | 1 | Non-moving level geometry |
| `Dynamic` | 2 | Fully simulated rigid bodies |
| `Kinematic` | 3 | Scripted / user-driven bodies |
| `Debris` | 4 | Small cosmetic fragments |
| `Projectile` | 5 | Bullets, arrows, thrown objects |
| `Character` | 6 | Player / NPC controllers |
| `Vehicle` | 7 | Wheeled / tracked vehicles |
| `Trigger` | 8 | Sensor-only trigger volumes |
| `Ragdoll` | 9 | Ragdoll articulation segments |
| `Water` | 10 | Fluid / buoyancy volumes |
| `User0`–`User20` | 11–31 | User-defined layers |

### Combining Layers

Use bitwise OR or the `makeLayerMask` helper:

```cpp
uint32_t mask = velox::CollisionLayers::Static |
                velox::CollisionLayers::Dynamic |
                velox::CollisionLayers::Character;

// Or with the helper (up to 4 arguments):
uint32_t mask = velox::makeLayerMask(
    velox::CollisionLayers::Static,
    velox::CollisionLayers::Dynamic,
    velox::CollisionLayers::Character);
```

### Layer Introspection

```cpp
const velox::Body& b = world.body(id);
bool isDynamic = velox::bodyHasLayer(b, velox::CollisionLayers::Dynamic);
bool hitsStatic = velox::bodyAcceptsLayer(b, velox::CollisionLayers::Static);
```

## Group-Based Filtering

The `groupIndex` field provides a fast same-group override that takes priority over layer bits:

| Group A | Group B | Same? | Result |
|--------:|--------:|:-----:|--------|
| +N | +N | yes | **Always collide** (ignores layers) |
| −N | −N | yes | **Never collide** (ignores layers) |
| 0 | any | — | Fall through to layer bits |
| +N | −M | no | Fall through to layer bits |
| +N | 0 | no | Fall through to layer bits |

### Use Cases

**Negative groups** prevent self-collision among related bodies:

```cpp
// Debris pieces from the same object should not collide with each other
velox::BodyId d1 = world.addSphere({0, 2, 0}, 0.2f, 0.5f);
velox::BodyId d2 = world.addSphere({0.3f, 2, 0}, 0.2f, 0.5f);
world.setCollisionFilter(d1, velox::CollisionFilterData{1, UINT32_MAX, -1});
world.setCollisionFilter(d2, velox::CollisionFilterData{1, UINT32_MAX, -1});
// d1 and d2 never collide, but both collide with the floor (group 0)
```

**Positive groups** force collision between bodies that would otherwise be filtered:

```cpp
// Ragdoll segments that must always interact
velox::BodyId upperArm = world.addCapsule({0, 1.5f, 0}, 0.1f, 0.3f, 2.0f);
velox::BodyId lowerArm = world.addCapsule({0, 1.0f, 0}, 0.1f, 0.3f, 2.0f);
world.setCollisionFilter(upperArm, velox::CollisionFilterData{
    velox::CollisionLayers::Ragdoll, velox::CollisionLayers::All, 1});
world.setCollisionFilter(lowerArm, velox::CollisionFilterData{
    velox::CollisionLayers::Ragdoll, velox::CollisionLayers::All, 1});
```

## Custom Filter Callbacks

For filtering logic that cannot be expressed with layers and groups, install a custom callback:

```cpp
world.setCollisionFilterCallback([](const velox::Body& a, const velox::Body& b) {
    // Reject all contacts involving trigger volumes
    if ((a.categoryBits & velox::CollisionLayers::Trigger) ||
        (b.categoryBits & velox::CollisionLayers::Trigger))
        return velox::FilterResult::Reject;

    // Accept all contacts between vehicles and characters
    bool aVehicle = (a.categoryBits & velox::CollisionLayers::Vehicle) != 0;
    bool bVehicle = (b.categoryBits & velox::CollisionLayers::Vehicle) != 0;
    bool aChar    = (a.categoryBits & velox::CollisionLayers::Character) != 0;
    bool bChar    = (b.categoryBits & velox::CollisionLayers::Character) != 0;
    if ((aVehicle && bChar) || (bVehicle && aChar))
        return velox::FilterResult::Accept;

    // Everything else: use the built-in group + layer rules
    return velox::FilterResult::Default;
});
```

### FilterResult Values

| Value | Meaning |
|-------|---------|
| `Default` | Keep the built-in group + layer result |
| `Accept` | Force collision regardless of layers/groups |
| `Reject` | Force no collision regardless of layers/groups |

### Removing the Callback

```cpp
world.setCollisionFilterCallback(nullptr);
```

### Performance Considerations

- The callback is invoked for **every broadphase candidate pair** — potentially thousands of times per step.
- Keep it lightweight: no heap allocations, locks, or I/O.
- The callback is **CPU-only**. GPU-resident stepping (`GPUResidentMode::Resident`) uses the built-in group + layer rules and does not invoke the callback.

## Setting Filters

### Per-Body Configuration

```cpp
// Category/mask only (group defaults to 0):
world.setCollisionFilter(id, categoryBits, maskBits);

// Full configuration with group index:
world.setCollisionFilter(id, velox::CollisionFilterData{categoryBits, maskBits, groupIndex});

// Read back:
velox::CollisionFilterData fd = world.collisionFilter(id);
```

Changing a body's filter immediately purges all existing contacts involving that body and marks the broadphase for re-evaluation.

### Default Values

Newly created bodies have:
- `categoryBits = 1` (Default layer)
- `maskBits = UINT32_MAX` (all layers)
- `groupIndex = 0` (no group)

This means all bodies collide with all other bodies by default.

## Filter Evaluation Order

```
┌─────────────────────────────────────────────────┐
│ 1. Group index check                            │
│    Both non-zero and equal?                     │
│    ├─ Yes, positive → COLLIDE (skip layers)     │
│    ├─ Yes, negative → NO COLLISION (skip layers)│
│    └─ No → continue to layer check              │
├─────────────────────────────────────────────────┤
│ 2. Layer bits (bidirectional category/mask)     │
│    (a.mask & b.category) && (b.mask & a.category)│
│    ├─ Pass → built-in result = COLLIDE          │
│    └─ Fail → built-in result = NO COLLISION     │
├─────────────────────────────────────────────────┤
│ 3. Custom callback (if set)                     │
│    ├─ Accept → COLLIDE (overrides built-in)     │
│    ├─ Reject → NO COLLISION (overrides built-in)│
│    └─ Default → use built-in result             │
└─────────────────────────────────────────────────┘
```

## Where Filtering Applies

| Pipeline Stage | Filter Used |
|---------------|-------------|
| Broadphase (AABB tree) | Group + Layer + Callback |
| Narrowphase fallback (sweep-and-prune) | Group + Layer (via `Body::canCollideWith`) |
| CCD / Multi-TOI | Group + Layer + Callback |
| Scene queries (raycast, overlap, shape cast) | Layer only (via `QueryFilter`) |
| GPU-resident stepping | Group + Layer only |

## Common Patterns

### One-Way Platforms

```cpp
// Platform only collides with bodies approaching from above
world.setCollisionFilterCallback([](const velox::Body& a, const velox::Body& b) {
    // Identify the platform (e.g., User0 layer)
    // This is a simplified example; real implementations check relative velocity
    return velox::FilterResult::Default;
});
```

### Player vs. Enemy Teams

```cpp
// Team A: group +1, Team B: group +2
// Same-team members don't collide (use negative sub-groups if needed)
world.setCollisionFilter(playerA1, velox::CollisionFilterData{
    velox::CollisionLayers::Character, velox::CollisionLayers::All, -10});
world.setCollisionFilter(playerA2, velox::CollisionFilterData{
    velox::CollisionLayers::Character, velox::CollisionLayers::All, -10});
// A1 and A2 never collide (same negative group)
```

### Ghost / Spectator Mode

```cpp
// Set a character to non-colliding temporarily
world.setCollisionFilter(characterId,
    velox::CollisionFilterData{velox::CollisionLayers::None, velox::CollisionLayers::None, 0});
```

## API Reference

| Function | Description |
|----------|-------------|
| `World::setCollisionFilter(BodyId, uint32_t, uint32_t)` | Set category/mask bits |
| `World::setCollisionFilter(BodyId, CollisionFilterData)` | Set category/mask/group |
| `World::collisionFilter(BodyId)` | Read back filter configuration |
| `World::setCollisionFilterCallback(CollisionFilterCallback)` | Install custom callback |
| `evaluateCollisionFilter(a, b)` | Built-in group + layer test (GPU-safe) |
| `evaluateCollisionFilterWithCallback(a, b, cb)` | Full test with callback |
| `bodyHasLayer(body, layer)` | Check body's category bits |
| `bodyAcceptsLayer(body, layer)` | Check body's mask bits |
| `makeLayerMask(...)` | Combine layer constants |
