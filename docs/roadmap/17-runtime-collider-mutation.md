# 17 — Runtime Collider Mutation

## Goal

Allow replacing or scaling a body's shape at runtime without removing and re-adding the body. This avoids the cost of broad-phase rebuilds, joint detachment/reattachment, and warm-start loss that currently force users to `removeBody()` + `add*()` cycles when shapes change (e.g., power-ups that change collision geometry, destructible objects that lose parts).

## Public API

```cpp
namespace velox {

// Shape replacement descriptor: new shape parameters for an existing body.
struct ShapeMutation {
    enum class Type { Sphere, Box, Capsule, Cylinder, Cone, Hull, Compound };
    Type type;

    float radius = 0.0f;
    Vec3 halfExtents{0.5f, 0.5f, 0.5f};
    float capsuleHalfHeight = 0.0f;
    std::vector<Vec3> hullPoints;
    std::vector<CompoundShape> compoundShapes;

    // If true, preserve the body's current mass properties (invMass, invInertia).
    // If false, recompute from the new shape geometry and density.
    bool preserveMassProperties = false;
};

// Scale the body's shape uniformly or anisotropically.
struct ShapeScale {
    Vec3 factor{1.0f, 1.0f, 1.0f};   // per-axis scale (uniform if all equal)
    bool updateMassProperties = true;  // recompute inertia for new extents
};

class World {
public:
    // Replace the shape of an existing body in-place. Broad-phase is updated
    // incrementally; joints attached to the body remain valid.
    void mutateShape(BodyId id, const ShapeMutation& mutation);

    // Scale the body's shape. For boxes/capsules, this modifies halfExtents/
    // radius directly. For hulls, all points are multiplied by the scale factor.
    void scaleShape(BodyId id, const ShapeScale& scale);

    // Replace only the collision margin (speculative reach) without changing
    // the actual geometry. Useful for tuning CCD behavior per-body.
    void setCollisionMargin(BodyId id, float margin);
};

} // namespace velox
```

## Data structures

- `ShapeMutation`, `ShapeScale` — new structs in `include/velox/world.h`.
- New World methods: `mutateShape()`, `scaleShape()`, `setCollisionMargin()`.

## Algorithm

**Shape replacement:**

1. Validate the new shape type is compatible with the body's current motion type (static bodies can have any shape; dynamic bodies must have valid mass properties).
2. If `preserveMassProperties` is false, recompute mass properties from the new geometry using existing QuickHull/analytical routines.
3. Update `Body::shape`, `Body::radius`, `Body::halfExtents`, etc. with the new parameters.
4. Mark the body's AABB as dirty; the incremental broad-phase will refit on the next `ensureBroadPhase()` call.
5. Joints attached to the body remain valid — their anchors are in local space and don't depend on shape geometry.

**Shape scaling:**

1. For boxes: multiply `halfExtents` by the scale factor. Recompute inertia if `updateMassProperties`.
2. For capsules: multiply `radius` and `capsuleHalfHeight` by the uniform scale.
3. For hulls: multiply all points in `hullPoints` by the scale factor (requires access to the MeshSoup; store a flag to rebuild hull data on next step).
4. Invalidate the broad-phase proxy for this body.

## Files

- `include/velox/world.h` — add ShapeMutation, ShapeScale, new methods
- `src/world.cpp` — implement mutation logic
- `tests/runtime_mutation.cpp` — test file

## Tests

1. **Sphere to box:** Dynamic sphere (radius 0.5, mass 1.0) mutated to box (halfExtents {0.3, 0.3, 0.3}). Body falls under gravity; trajectory matches a new box body with same mass within 1 mm after 1 second.
2. **Scale during fall:** Box (halfExtents {0.5, 0.5, 0.5}) scaled by 2× mid-fall. Inertia updates correctly; landing behavior unchanged vs. a pre-scaled box.
3. **Joint preservation:** Hinge joint connecting two bodies; mutate one body's shape. Joint remains valid, hinge angle query returns correct value post-mutation.
4. **Broad-phase update:** After mutation, the body's AABB in the broad-phase is updated within 1 frame (no stale proxies).

## Acceptance

- [ ] `mutateShape()` changes the body's geometry without removing/re-adding
- [ ] Mass properties recompute correctly for non-preserved mutations
- [ ] Attached joints remain valid after shape mutation
- [ ] Broad-phase AABB updates within 1 frame of mutation

## Size: M

## Risks

- Hull shape mutation requires modifying the shared `MeshSoup::hullPoints` array, which is also referenced by other bodies. Must copy-on-write or validate that no other body shares the same hull data before mutating.
- Scaling a hull non-uniformly changes its convexity; the resulting shape may no longer be convex, breaking GJK assumptions. Must validate convexity after non-uniform scale and reject if violated.
