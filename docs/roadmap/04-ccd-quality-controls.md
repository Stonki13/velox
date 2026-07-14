# 04 — CCD Quality Controls

## Goal

Expose per-body motion quality flags and tuning knobs so that performance-critical bodies (projectiles, fast-moving vehicles) can use aggressive CCD settings while static/kinematic level geometry uses conservative ones. Collision margins and speculative distance tuning prevent ghost collisions on fast bodies and unnecessary broad-phase work on slow bodies.

## Public API

```cpp
namespace velox {

// Per-body motion quality: controls how aggressively CCD and broad-phase
// expansion are applied. Higher quality = more expensive but safer.
enum class MotionQuality : uint8_t {
    Low = 0,        // no CCD, minimal broad-phase expansion (fast movers OK)
    Medium = 1,     // standard CCS + speculative contacts (default)
    High = 2,       // multi-TOI CCD + full conservative advancement
    Locked = 3      // disable all motion; body is effectively static after creation
};

// Per-body CCD tuning. Overrides World-level defaults when set explicitly.
struct BodyCcdTuning {
    MotionQuality quality = MotionQuality::Medium;
    float collisionMargin = 0.0f;       // inflated shape radius for broad phase
    float speculativeDistance = 0.0f;   // extra reach beyond maxPointSpeed*dt
    bool enableContinuous = true;       // toggle conservative advancement
    float minVelocityForCCD = 1.0f;     // m/s; below this, skip CCD entirely
};

// World-level defaults applied to bodies that don't override them.
struct WorldCcdDefaults {
    MotionQuality defaultQuality = MotionQuality::Medium;
    float defaultCollisionMargin = 0.0f;
    float defaultSpeculativeDistance = 0.01f;
    bool defaultEnableContinuous = true;
    float defaultMinVelocityForCCD = 1.0f;
};

// Accessors on World:
WorldCcdDefaults ccdDefaults() const;
void setCcdDefaults(WorldCcdDefaults defaults);

// Accessors on Body (via body(id)):
BodyCcdTuning ccdTuning() const;
void setCcdTuning(BodyCcdTuning tuning);

} // namespace velox
```

## Data structures

- `MotionQuality` enum — new, lives in `include/velox/body.h`. Stored as uint8_t on Body.
- `BodyCcdTuning` struct — new file `include/velox/ccd.h`. Per-body CCD configuration.
- `WorldCcdDefaults` struct — new file `include/velox/ccd.h`. World-level defaults.
- `Body::motionQuality` field — added to `include/velox/body.h`, default `Medium`.
- `Body::collisionMargin` field — added to `include/velox/body.h`, default 0.0f (uses shape radius).
- `World::ccdDefaults_` member — lives in `include/velox/world.h`, private.

## Algorithm

**Collision margin application:**

1. When computing the swept AABB for broad-phase expansion, inflate the shape's bounding box by `collisionMargin` in addition to `maxPointSpeed() * dt`. This catches pairs that are just outside the pure kinematic reach but within the material penetration threshold.
2. The margin is applied symmetrically: `sweptAABB = bodyAABB ± collisionMargin`. It does not affect narrow-phase distance calculations (GJK still uses the true shape).

**Speculative distance tuning:**

1. The existing speculative contact emission (`np_detail::emit`) uses `reach = (speedA + speedB) * dt + slop`. Add `speculativeDistance` as an additional additive term: `reach += bodyA.ccdTuning.speculativeDistance + bodyB.ccdTuning.speculativeDistance`.
2. This lets fast bodies "see" further ahead, creating speculative contacts earlier and giving the solver more substeps to resolve them — critical for high-speed scenarios where a single substep's integration might otherwise tunnel.

**Motion quality dispatch:**

1. `Low`: skip conservative advancement entirely; rely on speculative contacts + solver substeps only. Use standard AABB expansion (no margin).
2. `Medium`: current behavior — speculative contacts + one-shot conservative advancement safety net.
3. `High`: enable multi-TOI processing (item 03); full conservative advancement with binary-search TOI resolution.
4. `Locked`: body is treated as static for CCD purposes; no swept volume computed, no TOI events generated.

**Velocity floor filtering:**

1. Before running CCD sweep for a body, check `|velocity| > minVelocityForCCD`. If below threshold, skip the expensive conservative advancement and rely on speculative contacts alone (which are cheap).
2. This avoids wasting CPU cycles on slowly drifting bodies that would never tunnel.

## Files

**New files:**
- `include/velox/ccd.h` — MotionQuality, BodyCcdTuning, WorldCcdDefaults definitions

**Modified files:**
- `include/velox/body.h` — add `motionQuality`, `collisionMargin`, `ccdTuning` fields to Body struct
- `include/velox/world.h` — add `WorldCcdDefaults ccdDefaults_` member and accessors
- `src/narrowphase.h` — use `collisionMargin` in `bodyAabb()` expansion; use `speculativeDistance` in `emit()` reach calculation
- `src/solver.cpp` — dispatch CCD path based on `Body::motionQuality`; skip conservative advancement for Low quality

## Tests

1. **High-quality bullet through thin wall:** Sphere radius 0.05 m, velocity 3 km/s, wall thickness 0.01 m. With `MotionQuality::High`, must detect and resolve the impact within one step. With `MotionQuality::Low`, may tunnel (documented behavior).
2. **Collision margin prevents ghost collisions:** Two boxes 0.001 m apart, relative velocity 0 m/s. With `collisionMargin = 0.01`, no contact should be generated (they're not actually touching). Without margin, verify they don't falsely collide.
3. **Speculative distance extends reach:** Body A at rest, body B 0.5 m away moving at 100 m/s with `speculativeDistance = 0.1`. Must generate speculative contact (reach = 100*dt + 0.1 > 0.5 for dt=1/60).
4. **Velocity floor skips slow bodies:** Body with velocity 0.01 m/s and `minVelocityForCCD = 1.0` must not trigger conservative advancement sweep (verify via StepStats::ccdMs ≈ 0 for that body's contribution).
5. **Locked body is static:** Set a dynamic body to `MotionQuality::Locked`; it must not move under gravity or forces, and no CCD events should be generated for it.

## Acceptance

- [ ] `MotionQuality` enum exists with all four levels documented
- [ ] Per-body `collisionMargin` inflates broad-phase AABB without affecting narrow-phase geometry
- [ ] `speculativeDistance` adds to the reach calculation in speculative contact emission
- [ ] `MotionQuality::High` enables multi-TOI processing (requires item 03)
- [ ] `MotionQuality::Locked` prevents all motion and CCD for that body
- [ ] Velocity floor filtering skips CCD sweep for slow bodies
- [ ] All existing tests pass; no regression in CCD behavior at Medium quality

## Size: S

## Risks

- Collision margin is a heuristic; setting it too large creates phantom contacts with nearby static geometry, increasing solver load. Must document the tradeoff and provide sensible defaults per shape type.
- `MotionQuality::Locked` conflicts with `MotionType::Dynamic`; if both are set, Locked should win but this needs explicit documentation to avoid confusion.
- The velocity floor threshold is body-specific; a body at 0.99 m/s (below default 1.0 floor) might still tunnel through a thin wall. Users must tune per scenario.
