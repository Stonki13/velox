# Continuous Collision Detection (CCD)

Velox implements continuous collision detection to prevent **tunneling** — the
failure mode where a fast-moving body passes entirely through a thin obstacle
between discrete simulation steps. CCD computes the exact **time of impact
(TOI)** within a timestep and schedules collision response at that instant.

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                        Broad Phase CCD                          │
│  Swept AABB computation → overlap pruning → candidate pairs     │
└──────────────────────────────┬──────────────────────────────────┘
                               │
┌──────────────────────────────▼──────────────────────────────────┐
│                    Conservative Advancement                      │
│  Iterative gap/speed stepping → bracket [lower, upper]          │
└──────────────────────────────┬──────────────────────────────────┘
                               │
┌──────────────────────────────▼──────────────────────────────────┐
│                       Root Finding                               │
│  Brent's method (default) or bisection → refined TOI            │
└──────────────────────────────┬──────────────────────────────────┘
                               │
┌──────────────────────────────▼──────────────────────────────────┐
│                   Multi-TOI Event Scheduler                      │
│  Chronological sort → per-body caps → substep replay            │
└─────────────────────────────────────────────────────────────────┘
```

## Swept Volume Computation

The first CCD stage computes a **swept volume** — the region of space a shape
occupies during its entire trajectory over `[0, dt]`. Velox uses conservative
axis-aligned bounding boxes (AABBs) for broad-phase pruning.

### Linear Sweep

For a convex shape with bounding radius `r` moving from `p0` to `p1`:

```
sweptAABB.lo = min(p0, p1) - (r, r, r)
sweptAABB.hi = max(p0, p1) + (r, r, r)
```

The bounding radius is `length(halfExtents) + inflationRadius`, which
conservatively contains any convex shape.

### Rotational Sweep

When angular velocity is present, the bounding sphere sweeps an arc. The
rotational inflation is:

```
rotInflation = boundingRadius × angularSpeed × dt
```

This is added to all six AABB faces. For small angles this is tight; for large
angles (> π) it becomes overly conservative but remains correct.

### Quick Rejection

Before running expensive TOI queries, `sweptVolumesOverlap()` tests whether
the swept AABBs of two bodies intersect. Non-overlapping pairs are rejected in
O(1) with no GJK calls.

## Conservative Advancement Algorithm

The core TOI solver uses **conservative advancement** (also called "advancing
by the gap"):

1. **Initialize** `t = 0`, compute the signed gap `g(t)` via GJK/EPA.
2. **Advance** by `Δt = (g - tolerance) / relativeSpeed`.
3. **Repeat** until `g ≤ tolerance` (contact found) or `t ≥ dt` (no collision).
4. **Refine** the bracket `[lower, upper]` with a root-finding pass.

The `relativeSpeed` bound is critical for correctness:

```
relativeSpeed = |vA - vB| + |ωA| × rA + |ωB| × rB
```

This accounts for both linear and angular contributions to the maximum surface
speed. Underestimating this bound causes the algorithm to overshoot and miss
contacts; overestimating it only slows convergence.

### Root Finding

Once the advancement loop brackets the TOI in `[lower, upper]`, a root-finding
pass refines it:

- **Brent's method** (default for Balanced/Accurate): superlinear convergence,
  typically 3–5 iterations to reach `timeTolerance`. Combines inverse
  quadratic interpolation with bisection fallback.
- **Bisection** (Fast preset, or fallback): guaranteed linear convergence,
  halves the bracket each iteration.

The root finder solves `g(t) = distanceTolerance` — the time at which the gap
equals the contact threshold.

## Rotational CCD

Standard CCD only considers linear motion, which fails for spinning objects
whose extremal points sweep arcs. Velox's rotational CCD:

1. Computes `angularSweepAngle = |ω| × dt` for each body.
2. Inflates the swept AABB by `boundingRadius × angularSweepAngle`.
3. Uses `orientationAt(q0, ω, t)` to interpolate orientation at any time `t`
   during the conservative advancement loop.
4. The distance oracle evaluates GJK with the interpolated orientation,
   capturing rotation-induced contacts.

This is essential for:
- Spinning wheels contacting ground
- Rotating fan blades
- Tumbling debris in explosions

## Compound Shape CCD

Compound bodies (multiple convex children) require special handling:

1. **Per-child sweep**: each child's local transform is composed with the
   parent's interpolated transform at time `t`.
2. **Child-pair testing**: for two compound bodies, all `(childA, childB)`
   pairs are tested. The earliest TOI across all pairs is the compound TOI.
3. **Early-out**: if a TOI is found below `earlyOutThreshold`, remaining pairs
   are skipped.
4. **Pair cap**: `maxChildPairs` limits the combinatorial explosion for
   compounds with many children.

Configuration via `CompoundCcdConfig`:
```cpp
CompoundCcdConfig cfg;
cfg.testAllChildren = true;    // test every child pair
cfg.maxChildPairs = 64;        // cap on pair tests
cfg.earlyOut = true;           // stop on very early TOI
cfg.earlyOutThreshold = 1e-4f; // seconds
```

## Mesh CCD Optimization

Triangle meshes use a **BVH (Bounding Volume Hierarchy)** to accelerate CCD:

1. **Build**: triangles are partitioned into a binary tree of AABBs.
2. **Query**: the swept volume is tested against BVH nodes top-down.
3. **Prune**: subtrees whose AABB doesn't overlap the swept AABB are skipped.
4. **Leaf test**: only triangles in overlapping leaves undergo TOI queries.

This reduces triangle-level tests from O(n) to O(log n + k) where k is the
number of triangles whose AABBs overlap the sweep.

Configuration via `MeshCcdConfig`:
```cpp
MeshCcdConfig cfg;
cfg.maxBvhDepth = 8;           // traversal depth limit
cfg.maxTriangleTests = 128;    // hard cap on triangle tests
cfg.useSweptAabbPruning = true; // enable AABB pruning
cfg.triangleInflation = 0.0f;  // extra margin on triangle AABBs
```

## CCD Quality Settings

Three presets trade accuracy for performance:

| Setting | Fast | Balanced | Accurate |
|---------|------|----------|----------|
| Advancement iterations | 8 | 16 | 32 |
| Root-find iterations | 6 | 12 | 24 |
| Distance tolerance | 5×10⁻³ | 1×10⁻³ | 1×10⁻⁴ |
| Time tolerance | 1×10⁻⁴ | 1×10⁻⁵ | 1×10⁻⁶ |
| Rotational CCD | ✗ | ✓ | ✓ |
| Compound CCD | ✗ | ✓ | ✓ |
| Brent root-finder | ✗ | ✓ | ✓ |
| BVH max depth | 4 | 8 | 16 |
| Speculative margin | 0.02 | 0.01 | 0.005 |

### Choosing a Quality Level

- **Fast**: background objects, low-priority debris, objects unlikely to
  tunnel due to scene geometry. ~2× faster than Balanced.
- **Balanced**: default for most dynamic bodies. Handles typical game speeds
  (up to ~100 m/s at 60 Hz) without tunneling.
- **Accurate**: bullets, projectiles, racing vehicles, any object exceeding
  ~200 m/s or interacting with thin geometry. ~2× slower than Balanced.

### Per-Body Override

Each body carries a `BodyCcdTuning` with an embedded `CcdQualitySettings`:

```cpp
BodyCcdTuning tuning;
tuning.quality = MotionQuality::High;
tuning.ccdSettings = ccdQualityPreset(CcdQuality::Accurate);
tuning.minVelocityForCCD = 5.0f; // skip CCD below 5 m/s
world.setCcdTuning(bodyId, tuning);
```

## Multi-TOI Event Scheduling

When multiple collisions occur within a single timestep (e.g., a ball bouncing
between two walls), Velox uses a **multi-TOI event scheduler**:

1. **Collect**: find TOI for all eligible high-quality body pairs.
2. **Sort**: order events chronologically (ties broken by body index).
3. **Replay**: advance the simulation to each TOI in order, resolve the
   collision, then continue from that state.
4. **Cap**: per-body (`maxEventsPerBody`) and global (`maxTotalEvents`) limits
   prevent infinite loops in degenerate configurations.

### Event Merging

Events closer than `minTimeSeparation` for the same body pair are merged to
avoid redundant solver invocations. Temporal coherence reuses the previous
frame's TOI as a lower bound, skipping already-resolved intervals.

### Interaction with MotionQuality

- `MotionQuality::High` bodies participate in multi-TOI replay.
- `MotionQuality::Medium` bodies use Predictive Contact Sweeping (PCS) only.
- A High/Medium pair stays on the PCS path — rewinding only one side would
  violate its shared momentum timeline.

## Integration with the Simulation Loop

```
World::step(dt):
  1. Integrate velocities (gravity, forces)
  2. Broad-phase: compute swept AABBs, find overlapping pairs
  3. Narrow-phase: speculative contacts (PCS) for Medium bodies
  4. Multi-TOI: conservative advancement for High bodies
  5. Solve contacts (sequential impulses)
  6. Integrate positions
  7. Conservative recovery: if any High body still penetrates,
     rewind to TOI and re-resolve
```

## Numerical Robustness

- **Distance tolerance** prevents infinite loops when shapes are nearly
  touching. The gap must drop below `distanceTolerance` before a hit is
  declared.
- **Time tolerance** stops root-finding once the TOI is precise enough.
  Further refinement is wasted work at float32 precision.
- **Speed bound inflation**: the relative speed bound includes a small epsilon
  to prevent division-by-zero and ensure the advancement step is always
  positive.
- **Iteration caps**: hard limits on advancement and root-finding iterations
  guarantee termination even for pathological geometry.

## API Reference

### Core Types

| Type | Purpose |
|------|---------|
| `CcdQuality` | Enum: Fast, Balanced, Accurate |
| `CcdQualitySettings` | Tuning parameters for a quality level |
| `SweptVolume` | Swept AABB + trajectory metadata |
| `ToiResult` | Result of a TOI query |
| `ToiEvent` | Scheduled event in the multi-TOI queue |
| `CompoundToiResult` | TOI result with child index |
| `MeshBvhNode` | BVH node for mesh CCD |
| `MeshCcdConfig` | Mesh CCD traversal configuration |
| `MeshCcdStats` | Profiling statistics for mesh CCD |
| `MultiToiSchedulerConfig` | Event scheduler configuration |
| `ToiQueryDesc` | Descriptor for a pairwise TOI query |

### Key Functions

| Function | Purpose |
|----------|---------|
| `ccdQualityPreset(CcdQuality)` | Get settings for a quality level |
| `computeSweptVolume(...)` | Linear swept AABB |
| `computeSweptVolumeRotational(...)` | Rotational swept AABB |
| `conservativeAdvancement(...)` | Core TOI solver |
| `relativeSurfaceSpeed(...)` | Speed bound with angular terms |
| `angularSweepAngle(...)` | Angular displacement over dt |
| `orientationAt(...)` | Interpolated orientation at time t |
| `positionAt(...)` | Interpolated position at time t |
| `sweptVolumeForQuery(...)` | Swept volume from a ToiQueryDesc |
| `sweptVolumesOverlap(...)` | Quick AABB rejection test |

## References

- Zhang et al., "Conservative Advancement for Continuous Collision Detection
  of Convex Polyhedra" (2006)
- Tang et al., "Fast Continuous Collision Detection using Deformable Models"
- van den Bergen, "Efficient Collision Detection of Complex Deformable Models
  using AABB Trees" (1997)
- Brent, "Algorithms for Minimization without Derivatives" (1973) — root-finding
