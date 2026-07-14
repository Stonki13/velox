# 19 — Extra Shapes

## Goal

Add rounded box and ellipsoid collision shapes to Velox's existing primitive set. Additionally, design the interface for SDF/voxel collision and mutable terrain (implementation deferred to post-1.0). These shapes expand the range of supported geometries without requiring users to approximate them with convex hulls.

## Public API

```cpp
namespace velox {

// Rounded box: a box with filleted edges. Defined by half-extents and corner radius.
struct RoundedBoxShape {
    Vec3 halfExtents{0.5f, 0.5f, 0.5f};
    float cornerRadius = 0.05f;     // fillet radius; 0 = sharp box
    uint32_t roundingSegments = 8;  // tessellation quality for debug drawing
};

// Ellipsoid: axis-aligned or rotated ellipsoid defined by semi-axes.
struct EllipsoidShape {
    Vec3 semiAxes{0.5f, 0.5f, 0.5f};  // half-lengths along each principal axis
    bool isSphere() const { return semiAxes.x == semiAxes.y && semiAxes.y == semiAxes.z; }
};

// SDF voxel terrain: a 3D grid of signed distance values. Positive = outside,
// negative = inside. Mutable: individual voxels can be changed at runtime.
struct VoxelTerrain {
    uint32_t width, depth, height;     // grid dimensions
    float voxelSize = 0.1f;            // meters per voxel edge
    Vec3 origin{};                     // world-space origin of (0,0,0) voxel
    std::vector<int16_t> distances;    // signed distances in Q4.12 fixed point

    VELOX_HD int16_t getDistance(int x, int y, int z) const {
        if (x < 0 || x >= (int)width || y < 0 || y >= (int)depth || z < 0 || z >= (int)height)
            return 32767;  // outside world bounds
        return distances[y * width * depth + z * width + x];
    }

    void setDistance(int x, int y, int z, int16_t value) {
        if (x >= 0 && x < (int)width && y >= 0 && y < (int)depth && z >= 0 && z < (int)height)
            distances[y * width * depth + z * width + x] = value;
    }
};

// World API additions:
BodyId addRoundedBox(Vec3 position, const RoundedBoxShape& shape, float mass);
BodyId addEllipsoid(Vec3 position, const EllipsoidShape& shape, float mass);
BodyId addVoxelTerrain(const VoxelTerrain& terrain);  // static only

// Query support for new shapes:
void overlapRoundedBox(Vec3 center, const RoundedBoxShape& shape, Quat orientation,
                       std::vector<BodyId>& out, const QueryFilter& filter = {}) const;
void overlapEllipsoid(Vec3 center, const EllipsoidShape& shape, Quat orientation,
                      std::vector<BodyId>& out, const QueryFilter& filter = {}) const;
ShapeCastHit roundedBoxCast(Vec3 center, const RoundedBoxShape& shape, Quat orientation,
                            Vec3 direction, float maxDist,
                            const QueryFilter& filter = {}) const;
ShapeCastHit ellipsoidCast(Vec3 center, const EllipsoidShape& shape, Quat orientation,
                           Vec3 direction, float maxDist,
                           const QueryFilter& filter = {}) const;

} // namespace velox
```

## Data structures

- `RoundedBoxShape`, `EllipsoidShape` — new structs in `include/velox/shapes.h`.
- `VoxelTerrain` — new struct in `include/velox/shapes.h`.
- New ShapeType enum values: `ShapeType::RoundedBox`, `ShapeType::Ellipsoid`, `ShapeType::VoxelTerrain`.
- New World methods: `addRoundedBox()`, `addEllipsoid()`, `addVoxelTerrain()`, overlap/cast variants.

## Algorithm

**Rounded box GJK support function:**

1. Compute the closest point on the sharp box to the query direction (standard box support).
2. If the closest point is on a face (not edge/vertex), return it directly.
3. If on an edge, round the edge: project the direction onto the edge's perpendicular plane, normalize, scale by cornerRadius, and offset from the edge midpoint.
4. If on a vertex, round the vertex: same projection approach but onto the corner sphere centered at the vertex.

**Ellipsoid GJK support function:**

1. For direction `d`, the support point is `(semiAxes.x * d.x/|d|, semiAxes.y * d.y/|d|, semiAxes.z * d.z/|d|)` scaled by the corresponding semi-axis length.
2. This is exact for axis-aligned ellipsoids; for rotated ellipsoids, transform `d` into the ellipsoid's local frame first.

**Voxel terrain collision:**

1. **Broad phase:** voxel grid is divided into chunks (e.g., 16³ voxels); each chunk gets an AABB. Only collide with chunks whose AABB overlaps the query shape.
2. **Narrow phase:** for each overlapping voxel, trilinearly interpolate the signed distance field to find the exact surface intersection. Use a marching cubes or ray-marching approach for the contact point.
3. **Mutable terrain:** `VoxelTerrain::setDistance()` updates individual voxels; the broad-phase AABBs are invalidated and refitted on the next step.

## Files

- `include/velox/shapes.h` — new header with RoundedBoxShape, EllipsoidShape, VoxelTerrain
- `src/rounded_box.cpp` — rounded box support function, GJK integration
- `src/ellipsoid.cpp` — ellipsoid support function, GJK integration
- `src/voxel_terrain.cpp` — voxel collision detection (design only for mutable terrain)

## Tests

1. **Rounded box vs sphere:** Sphere resting on a rounded box must settle at the bottom with no jitter. Contact normal points vertically upward.
2. **Ellipsoid stack:** 5 ellipsoids (semi-axes {1, 0.5, 0.5}) stacked vertically. Tower remains stable for 5 seconds.
3. **Voxel terrain raycast:** Ray from above into a voxel sphere (radius 10 voxels) must hit the surface at the correct distance (within 1 voxel size).
4. **Mutable terrain update:** Modify 100 voxels in a 64³ terrain; next step's collision detection reflects the changes without rebuilding the entire grid.

## Acceptance

- [ ] `addRoundedBox()` creates a body with valid GJK support function
- [ ] `addEllipsoid()` creates a body with correct support function for axis-aligned and rotated cases
- [ ] Voxel terrain collision detects contacts within 1 voxel size accuracy
- [ ] Mutable terrain updates are reflected in collision detection within 1 frame

## Size: L

## Risks

- Rounded box support function has a discontinuity at the transition between face/edge/vertex rounding. Must smooth the transition to avoid jitter in the solver.
- Ellipsoid GJK is exact but the EPA penetration depth calculation requires iterative surface sampling. Must validate against analytical ellipsoid-ellipsoid overlap cases.
- Voxel terrain collision is O(voxels in query AABB) in the worst case. Must implement spatial hashing or chunk-based culling for large terrains.
