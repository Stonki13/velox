# 01 — Persistent Contact Manifolds

## Goal

Replace the current sampled/support-plane manifold generation with general convex face clipping (Sutherland-Hodgman) so that resting contacts form stable, feature-stable manifolds. Each contact carries a persistent feature id (face/edge/vertex index) that survives across frames, enabling true warm starting of the iterative solver and eliminating the jitter/wobble that plagues single-point hull manifolds on flat surfaces.

## Public API

```cpp
namespace velox {

// Feature identifiers for stable manifold points across frames.
enum class ContactFeature : uint32_t {
    None = 0,
    Vertex = 1u << 0,       // vertex index into hull point cloud or box corner mask
    Edge   = 1u << 1,       // edge index (pair of vertex indices)
    Face   = 1u << 2,       // face index (triangle fan from QuickHull mass faces)
    Implicit = 1u << 30,    // sphere/capsule endpoint tag
    Triangle = 1u << 31,    // triangle index in mesh BVH leaf
};

// Persistent manifold point: carries feature ids so the solver can warm-start
// from previous-frame impulses on the same geometric feature.
struct ManifoldPoint {
    Vec3 normal;              // from B towards A (world space)
    Vec3 pointA, pointB;      // witness points on each surface
    Vec3 localAnchorA;        // in A's body frame
    Vec3 localAnchorB;        // in B's body frame
    float gap;                // signed distance at detection time
    uint32_t featureIdA;      // ContactFeature mask for A
    uint32_t featureIdB;      // ContactFeature mask for B
    float restitution;
    float friction1, friction2;
    float rollingFriction, spinningFriction;
};

// Persistent manifold between two bodies. Up to kMaxManifoldPoints points are
// stored; the solver iterates over them each substep using live gaps.
struct Manifold {
    BodyIndex a, b;
    uint64_t featureKey;      // stable pair key for warm-start lookup
    Vec3 normal;              // representative contact normal (from B to A)
    std::array<ManifoldPoint, 8> points;
    int pointCount = 0;

    VELOX_HD bool hasFeature(uint32_t idA, uint32_t idB) const {
        return featureKey == ((uint64_t(idA)+1) << 32) | (uint64_t(idB)+1);
    }
};

// Manifold generation result: one manifold per touching pair.
struct ManifoldResult {
    std::vector<Manifold> manifolds;
    int totalPoints = 0;
};

} // namespace velox
```

## Data structures

- `ContactFeature` enum — new, lives in `include/velox/world.h` alongside `ContactEventType`.
- `ManifoldPoint` struct — new file `include/velox/manifold.h`. Carries feature ids and per-point material values.
- `Manifold` struct — new file `include/velox/manifold.h`. Holds up to 8 points with a stable feature key for warm-start lookup against the previous frame's contact array.
- `Body::featureData` field (optional) — stores QuickHull mass-face indices computed at creation, used as persistent face ids during clipping. Lives in `include/velox/body.h`.

## Algorithm

**Sutherland-Hodgman convex polygon clipping** for manifold generation:

1. **Pick reference plane.** For convex-convex pairs, the GJK/EPA normal defines the splitting plane through the deepest penetration point. For convex-plane, use the plane's normal.
2. **Project both shapes onto the reference plane.** Walk each shape's face/edge/vertex set and compute signed distances to the plane. Keep vertices within the clipping half-space (distance ≤ 0).
3. **Clip polygon A against each edge of polygon B** (Sutherland-Hodgman iterative clip): for each edge of B, split the current output polygon by that edge line, keeping only the interior side. Output is a convex polygon in 2D plane coordinates.
4. **Reproject clipped vertices back to 3D.** Each clipped vertex lies on an edge of one shape; interpolate along that edge between its two endpoints using the ratio of distances to the clipping plane.
5. **Generate up to kMaxManifoldPoints (8) witness points** from the clipped polygon vertices, assigning feature ids based on which primitive element each point originated from (vertex index, edge pair, face index).
6. **Compute per-point material values** using the existing `combineMaterial` logic with body-local friction scales.

For convex-hull pairs specifically: use QuickHull mass faces as the polygon vertex sets. Each hull face is a triangle fan; project all face vertices onto the contact plane and clip. This gives deterministic, feature-stable manifolds for resting contacts on flat-ish regions.

## Files

**New files:**
- `include/velox/manifold.h` — ManifoldPoint, Manifold, ContactFeature definitions
- `src/manifold.cpp` — Sutherland-Hodgman clipping implementation
- `src/manifold.h` — inline manifold generation helpers (VELOX_HD)

**Modified files:**
- `include/velox/body.h` — add optional feature data fields for hull face indices
- `include/velox/world.h` — add ContactFeature enum, ManifoldResult type alias
- `src/narrowphase.h` — replace `planeConvex` / `convexConvex` emit logic with manifold generation; keep GJK/EPA as the penetration oracle that feeds the clipping plane
- `src/solver.cpp` — consume Manifold arrays instead of flat Contact arrays for warm starting

## Tests

1. **Box-on-plane resting stack (10 boxes):** Each box must generate a 4-point manifold on its bottom face. Tower must remain stable for 10 seconds at 60 Hz with zero drift > 1 mm.
2. **Hull-on-hull face contact:** Two identical convex hulls placed face-to-face must produce manifolds with feature ids matching the shared face index across frames (warm start stability).
3. **Feature id persistence:** Run a 100-frame simulation of two resting boxes; verify that `Manifold::featureKey` is identical for each point across all frames where contact persists.
4. **Sutherland-Hodgman edge case — partial overlap:** Two boxes sliding past each other must produce manifolds that smoothly transition from 1-point to 4-point as overlap grows, with no discontinuous jumps in total manifold area.
5. **GPU parity:** CUDA backend must produce identical manifold points (within 1e-4) to CPU for the same input configuration.

## Acceptance

- [ ] Sutherland-Hodgman clipping implemented and VELOX_HD-compatible
- [ ] ManifoldPoint carries feature ids that survive across frames
- [ ] Warm starting from previous-frame manifold impulses converges in ≤ 3 solver iterations for resting stacks (vs. ≥ 6 with current sampled approach)
- [ ] Box-on-plane generates ≥ 3 contact points when resting (not just 1 support point)
- [ ] All existing tests pass without regression
- [ ] CUDA backend produces numerically identical manifolds to CPU

## Size: L

## Risks

- Sutherland-Hodgman requires both shapes to be projected as convex polygons onto the clipping plane. Non-convex projections (e.g., a thin needle hull viewed edge-on) can produce degenerate polygons — need fallback to GJK witness for those cases.
- Feature id assignment must be deterministic across CPU and CUDA; any non-deterministic sort in the clipping output will break warm starting.
- The current `Contact::featureKey` uses implicit 32-bit tags; ManifoldPoint needs richer feature encoding without breaking the existing key format.
