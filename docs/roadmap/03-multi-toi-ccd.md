# 03 — Multi-TOI CCD

## Goal

Extend the current conservative-advancement CCD safety net (which handles one impact per body per step) to process multiple sequential time-of-impact events within a single simulation step. This enables bullets to pass through several walls, cars to bounce off multiple objects, and general high-speed scenarios where a body intersects more than one collider before the step ends — without resorting to substep proliferation.

## Public API

```cpp
namespace velox {

// Result of a multi-TOI sweep for one body: ordered list of impact events
// sorted by increasing time of impact within [0, dt].
struct MultiToiHit {
    float toi;              // time of impact in [0, dt]
    BodyId body;            // the other body at impact
    Vec3 point;             // world-space contact point
    Vec3 normal;            // from other body towards this body
    float fraction;         // toi / dt, useful for interpolation
};

// Per-body CCD configuration. Stored on Body or as a World-level default.
struct CcdConfig {
    bool enabled = true;                    // toggle multi-TOI processing
    float maxToiEventsPerBody = 4;          // cap to bound CPU cost
    float toiVelocityFloor = 0.1f;          // m/s; ignore TOIs below this
    float toiPenetrationBias = 1e-3f;       // allow slight penetration at TOI
};

// World-level multi-TOI settings.
struct WorldMultiToiSettings {
    CcdConfig defaultConfig;
    int maxTotalEventsPerStep = 256;        // global cap
    bool enableSubstepSplitting = true;     // split step at each TOI
};

// Query: return all TOIs for a body within the current step's motion.
std::vector<MultiToiHit> queryMultiToi(BodyId id, float dt) const;

} // namespace velox
```

## Data structures

- `MultiToiHit` struct — new file `include/velox/ccd.h`. One impact event per body per TOI.
- `CcdConfig` struct — new file `include/velox/ccd.h`. Per-body or default CCD tuning.
- `WorldMultiToiSettings` struct — lives in `include/velox/world.h`, new member on World.
- `Body::ccdConfig` field — added to `include/velox/body.h`, overrides world defaults.

## Algorithm

**Sequential TOI processing (event-driven within a step):**

1. **Initial speculative detection.** Run the existing broad-phase + narrow-phase contact detection at t=0. Collect all pairs with gap ≤ reach.
2. **For each dynamic body with velocity above `toiVelocityFloor`:**
   a. Sweep the body's AABB tree path against all static/dynamic bodies using conservative advancement (GJK as distance oracle).
   b. Record the earliest TOI event (smallest t where gap ≤ 0) up to `maxToiEventsPerBody`.
   c. If substep splitting is enabled, split the remaining time [toi, dt] and repeat from step 2a with the post-impact state.
3. **Process events in chronological order.** For each TOI event:
   a. Rewind both bodies to the TOI using inverse conservative advancement (rotate back along angular velocity, translate back along linear velocity).
   b. Apply a momentum-preserving impulse at the contact point (same as current PCS safety net).
   c. Advance both bodies forward from TOI to the next event time (or dt if last event).
4. **Resolve penetrations.** After all events are processed, run the standard positional correction pass on any remaining interpenetrations.

**Conservative advancement for multi-TOI:**

The existing `meshSimpleGapProbe` / GJK-based sweep is reused. For each candidate pair, binary-search for the exact TOI within [t_start, t_end] using bisection on the gap function (monotone for convex shapes under constant velocity). Each bisection step requires one GJK distance query — cap at 16 iterations for O(log₂(1/ε)) ≈ 17 steps to reach 1e-5 precision.

## Files

**New files:**
- `include/velox/ccd.h` — MultiToiHit, CcdConfig, WorldMultiToiSettings definitions
- `src/multi_toi.cpp` — TOI event queue management, chronological processing
- `src/multi_toi.h` — inline conservative advancement helpers (VELOX_HD)

**Modified files:**
- `include/velox/world.h` — add `WorldMultiToiSettings`, `queryMultiToi()`, `multiToiSettings_` member
- `include/velox/body.h` — add `CcdConfig ccdConfig` field
- `src/solver.cpp` — integrate multi-TOI processing into the step loop between speculative detection and velocity solve
- `src/backend.h` — Backend interface: add optional `findMultiToi()` virtual for GPU acceleration

## Tests

1. **Bullet through 3 walls:** Sphere at 2 km/s along z-axis intersects 3 parallel planes spaced 0.5 m apart. Must produce exactly 3 TOI events, each with correct point/normal, and exit the last wall with velocity reduced by restitution at each impact.
2. **TOI ordering correctness:** Two walls at z=1 and z=2; sphere starts at z=0 moving +z at 1 km/s. First event must be at z=1 (toi ≈ 0.001), second at z=2 (toi ≈ 0.002). No out-of-order processing.
3. **Energy conservation at elastic impacts:** Sphere vs static wall, restitution=1.0. Post-impact speed must equal pre-impact speed within 1e-4 (no energy loss from TOI processing itself).
4. **Max event cap:** Configure `maxToiEventsPerBody = 2` with a body that would intersect 5 walls. Must process exactly 2 events and then continue to dt with the state at the second impact.
5. **Grazing multi-TOI:** Box sliding along a staircase (10 steps, each 0.1 m rise/run). Must produce ≥ 8 TOI events (some grazes may not register as penetration) without tunneling through any step.

## Acceptance

- [ ] Multi-TOI processes ≥ 3 sequential impacts per body per step in the bullet-through-walls test
- [ ] TOI events are processed in strict chronological order
- [ ] Momentum is conserved at each elastic impact (within numerical tolerance)
- [ ] `maxToiEventsPerBody` cap is enforced (no more events than configured)
- [ ] No regression in existing CCD test scenarios (bullet_demo still passes)
- [ ] GPU backend supports multi-TOI via Backend::findMultiToi() or falls back to CPU path

## Size: L

## Risks

- Binary search for TOI assumes the gap function is monotone, which holds for convex-vs-convex under constant velocity but can fail for compound shapes with non-convex effective swept volume. May need fallback to continuous broad-phase for compounds.
- Rewinding bodies to TOI and re-applying impulses can introduce energy drift if the impulse calculation doesn't account for the partial time already spent before impact. Must use exact fractional timestep in impulse application.
- The event queue grows O(n²) in worst case (every body intersects every other). The `maxTotalEventsPerStep` cap prevents runaway cost but may silently drop valid events — needs a warning log or assertion in debug builds.
