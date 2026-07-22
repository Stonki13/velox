# Sleeping

Velox uses **island-based sleeping** to eliminate simulation cost for bodies
that have come to rest. When every dynamic body in a connected island has been
calm for long enough, the whole island is put to sleep: integration, solving,
and broad-phase participation are skipped entirely until something wakes it.

## How it works

### Island formation

Each step, the sleep system builds **islands** using union-find over:

- **Contacts** between two dynamic, non-sensor bodies.
- **Joints** connecting two dynamic bodies.

Bodies in the same island share constraints, so they must sleep and wake
together. Static and kinematic bodies do not participate in islands but can
serve as "ground" that supports a sleeping island.

### Motion evaluation

A dynamic body is considered **calm** when all of the following hold:

| Criterion | Default threshold | Config field |
|-----------|-------------------|--------------|
| Linear speed `\|v\|` | 0.05 m/s | `linearVelocityThreshold` |
| Angular speed `\|ω\|` | 0.05 rad/s | `angularVelocityThreshold` |
| Acceleration `\|F/m\|` | 0.01 m/s² | `accelerationThreshold` |

The acceleration check prevents bodies with significant applied forces from
sleeping even when their instantaneous velocity is low (e.g., a body being
pushed by a strong spring).

### Contact stability

When `enableContactStability` is true (the default), a body must also have
**stable contacts** — contacts where the relative velocity at the contact
point stays below `contactStabilityThreshold` for at least
`contactStabilityFrames` consecutive frames. This prevents bodies resting on
vibrating or jittering surfaces from sleeping prematurely.

### Sleep timer

Each calm body accumulates `sleepTimer += dt`. When a body is no longer calm,
its timer resets to zero. An island sleeps when its **minimum** timer across
all members exceeds `timeToSleep` (default 0.5 s). This ensures the entire
island has been stable before any member sleeps.

### Gradual sleep (drowsy state)

When `enableGradualSleep` is true (the default), bodies pass through an
intermediate **drowsy** state before falling fully asleep:

1. **Awake** → fully simulated.
2. **Drowsy** (after `timeToDrowsy` seconds of calmness) → the body is still
   simulated but flagged for reduced-rate processing. This provides a smooth
   visual transition and avoids the "pop" of a body freezing mid-motion.
3. **Asleep** (after `timeToSleep` seconds) → the body skips integration and
   solving entirely. Its velocity is zeroed.

The `Body::asleep` field encodes the state as a `uint8_t`:

| Value | State | Meaning |
|-------|-------|---------|
| 0 | Awake | Fully simulated |
| 1 | Asleep | Skips integration and solving |
| 2 | Drowsy | Reduced-rate simulation (transitional) |

This encoding is backward compatible: existing code that checks
`body.asleep != 0` to mean "not fully simulated" continues to work.

### Wake conditions

A sleeping or drowsy body is woken when:

- **Collision**: an awake (or drowsy) non-static body touches it with
  sufficient impact or relative motion.
- **Explicit API**: `World::wake()`, `World::wakeBody()`, or
  `World::setLinearVelocity()` / `World::setTransform()` /
  `World::setMotionType()`.
- **Joint mismatch**: a joint connects a sleeping body to an awake one —
  both are woken.
- **Explosion**: `World::explode()` wakes drowsy bodies in range (fully
  asleep bodies are skipped).

## Configuration

All thresholds are configurable per-world through `World::sleepConfig()`:

```cpp
velox::World world(velox::BackendType::Cpu);

// Access the mutable config directly:
world.sleepConfig().timeToSleep = 1.0f;
world.sleepConfig().linearVelocityThreshold = 0.1f;
world.sleepConfig().enableGradualSleep = true;
world.sleepConfig().validate(); // clamp invalid values

// Or replace the whole config (validated automatically):
velox::SleepConfig config;
config.timeToSleep = 2.0f;
config.enableContactStability = false;
world.setSleepConfig(config);
```

### SleepConfig reference

| Field | Default | Description |
|-------|---------|-------------|
| `linearVelocityThreshold` | 0.05 | Linear speed below which a body is calm (m/s) |
| `angularVelocityThreshold` | 0.05 | Angular speed below which a body is calm (rad/s) |
| `accelerationThreshold` | 0.01 | Acceleration below which a body is calm (m/s²) |
| `timeToSleep` | 0.5 | Seconds of calmness before an island sleeps |
| `timeToDrowsy` | 0.25 | Seconds of calmness before entering drowsy state |
| `drowsySimulationRate` | 0.25 | Fraction of substeps a drowsy body runs (0, 1] |
| `enableGradualSleep` | true | Enable the drowsy intermediate state |
| `enableContactStability` | true | Require stable contacts before sleeping |
| `contactStabilityThreshold` | 0.02 | Max relative velocity for a stable contact |
| `contactStabilityFrames` | 10 | Consecutive stable frames required |

Call `config.validate()` after modifying fields directly to clamp invalid
values (negative thresholds, `timeToDrowsy >= timeToSleep`, etc.).

## Callbacks

Install transition callbacks to react to sleep state changes:

```cpp
velox::SleepCallbacks callbacks;
callbacks.onSleep = [](velox::BodyId id) {
    // Body fell asleep — disable its render animation, play a sound, etc.
};
callbacks.onWake = [](velox::BodyId id) {
    // Body woke up — re-enable animation, trigger gameplay logic, etc.
};
callbacks.onDrowsy = [](velox::BodyId id) {
    // Body entered drowsy state — start a "settling" visual effect.
};
world.setSleepCallbacks(std::move(callbacks));
```

> **Warning:** Do not call `World::step()` or mutate the World from inside a
> callback. Doing so throws or deadlocks.

## Statistics

Query sleep diagnostics after each step:

```cpp
const velox::SleepStats& stats = world.sleepStats();
printf("Awake: %zu  Drowsy: %zu  Asleep: %zu  Islands: %zu\n",
       stats.awakeBodies, stats.drowsyBodies, stats.sleepingBodies,
       stats.islandCount);
```

`SleepStats` fields:

| Field | Description |
|-------|-------------|
| `totalDynamicBodies` | Total dynamic bodies in the world |
| `awakeBodies` | Dynamic bodies fully simulated |
| `drowsyBodies` | Dynamic bodies in reduced-rate mode |
| `sleepingBodies` | Dynamic bodies fully asleep |
| `islandCount` | Total islands formed this step |
| `sleepingIslandCount` | Islands fully asleep |
| `drowsyIslandCount` | Islands in drowsy transition |
| `awakeIslandCount` | Islands fully awake |
| `totalSleepTransitions` | Cumulative awake→asleep transitions |
| `totalWakeTransitions` | Cumulative asleep→awake transitions |
| `totalDrowsyTransitions` | Cumulative awake→drowsy transitions |
| `lastUpdateMs` | Wall time of the last sleep update (ms) |

Island-level snapshots are available via `world.sleepIslands()`.

## Debug visualization

Enable the sleep visualization layer to tint shapes by sleep state:

```cpp
std::vector<velox::DebugLine> lines;
world.debugLines(lines, velox::DebugDrawSleep);
// or include it in the full set:
world.debugLines(lines, velox::DebugDrawEverything);
```

Color coding:

| State | Color | Hex |
|-------|-------|-----|
| Awake | Cyan | `0x58c4dd` |
| Drowsy | Amber | `0xffd43b` |
| Asleep | Muted blue | `0x4950ce` |

Sleeping bodies also get a small "Z" marker drawn above them.

If you use the `DebugDraw` interface:

```cpp
MyDebugDraw draw;
draw.setFlags(velox::DebugDrawSleep | velox::DebugDrawShapes);
velox::drawWorld(draw, world);
```

## Best practices

### Do

- **Use a fixed timestep.** Sleep timers accumulate `dt` each step; variable
  timesteps make sleep timing unpredictable.

- **Disable sleep for gameplay-critical bodies.** Use
  `world.setEnableSleep(id, false)` for bodies that must always simulate
  (player characters, vehicles, triggered objects).

- **Wake bodies before applying forces or impulses.** A sleeping body ignores
  forces. Call `world.wake(id)` before `world.addForce(id, ...)` if the body
  might be asleep.

- **Tune thresholds for your scale.** The defaults assume meter-scale worlds.
  For centimeter-scale worlds, lower `linearVelocityThreshold` proportionally.

- **Use `sleepStats()` for profiling.** The `awakeBodies` count tells you how
  many bodies are actually being simulated. If it's close to
  `totalDynamicBodies`, sleeping isn't helping and your thresholds may be too
  conservative.

- **Prefer `sleepConfig()` over per-body hacks.** The config is validated and
  applies uniformly. Per-body `enableSleep` is the only per-body control.

### Don't

- **Don't set `Body::asleep` directly.** Use `world.sleepBody()` and
  `world.wakeBody()` so callbacks fire and internal state stays consistent.

- **Don't expect sleeping bodies to respond to forces.** Sleeping bodies skip
  integration entirely. Wake them first.

- **Don't use very low thresholds in noisy scenes.** If contacts jitter
  (e.g., many bodies in a pile), low thresholds cause bodies to never sleep.
  Enable `enableContactStability` and increase `contactStabilityFrames`
  instead of lowering velocity thresholds.

- **Don't forget that islands sleep together.** A single vibrating body in an
  island keeps the entire island awake. Isolate noisy bodies with joints or
  disable their sleep individually.

### Performance tips

- Sleeping is the single biggest performance win for scenes with many resting
  bodies. A scene with 1000 bodies where 900 are sleeping runs ~10× faster
  than one where all are awake.

- The sleep update itself is O(n + c + j) where n = bodies, c = contacts,
  j = joints. It runs once per step after solving.

- Contact stability tracking adds a small per-contact cost. Disable it with
  `enableContactStability = false` if you don't need it and are
  CPU-constrained.

- The drowsy state adds negligible cost (it's just a state flag). The
  `drowsySimulationRate` field is reserved for future substep-skipping
  optimization.

## API reference

| Method | Description |
|--------|-------------|
| `World::sleepConfig()` | Mutable access to `SleepConfig` |
| `World::setSleepConfig(config)` | Replace config (validated) |
| `World::sleepStats()` | Read `SleepStats` from last step |
| `World::sleepIslands()` | Read `SleepIsland` snapshots |
| `World::setSleepCallbacks(cb)` | Install transition callbacks |
| `World::sleepState(id)` | Query a body's `SleepState` |
| `World::wake(id)` | Wake a sleeping/drowsy body |
| `World::isAwake(id)` | True when fully awake |
| `World::wakeBody(id)` | Force awake (fires callback) |
| `World::sleepBody(id)` | Force asleep (fires callback) |
| `World::setEnableSleep(id, bool)` | Allow/forbid sleeping per body |
| `World::isSleepEnabled(id)` | Query per-body sleep permission |

## See also

- `include/velox/sleep.h` — full type and API documentation.
- `tests/unit/test_sleep.cpp` — comprehensive test suite.
- `docs/debugging.md` — debug visualization overview.
