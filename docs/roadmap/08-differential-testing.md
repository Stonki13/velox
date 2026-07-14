# 08 — Differential Testing

## Goal

Run identical simulation scenes in Velox, Jolt Physics, and Bullet Physics simultaneously and compare trajectories, energies, and contact forces statistically. This catches regressions that unit tests miss (e.g., a solver change that subtly alters stacking behavior) by providing a ground-truth reference from battle-tested engines.

## Public API

```cpp
namespace velox {
namespace test {

// Scene description for differential testing: a sequence of world mutations
// followed by stepping, with observable state captured at each frame.
struct DiffTestScene {
    std::string name;
    Vec3 gravity{0, -9.81f, 0};
    int substeps = 4;
    float dt = 1.0f / 60.0f;

    // Bodies to add before stepping begins.
    struct BodyDesc {
        enum class Shape { Sphere, Box, Capsule, Plane };
        Shape shape;
        Vec3 position;
        Quat orientation{};
        float radius = 0.5f;
        Vec3 halfExtents{0.5f, 0.5f, 0.5f};
        float mass = 1.0f;
        Vec3 initialVelocity{};
    };
    std::vector<BodyDesc> bodies;

    // Forces/impulses to apply at specific frames (zero-indexed).
    struct ForceEvent {
        int frame;
        BodyId body;
        Vec3 force;           // accumulated force for this frame
        Vec3 impulse;         // instantaneous impulse
    };
    std::vector<ForceEvent> forces;

    // Bodies whose state is recorded each frame for comparison.
    std::vector<BodyId> trackedBodies;
};

// Statistical comparison result between two engines' outputs.
struct DiffTestResult {
    float maxPositionDelta = 0.0f;      // meters, over all frames and bodies
    float maxVelocityDelta = 0.0f;      // m/s
    float maxEnergyDrift = 0.0f;        // joules, from initial total energy
    float trajectoryCorrelation = 1.0f; // Pearson r on position time series
    bool passed = false;                 // true if all deltas within tolerance
};

// Run a scene through Velox and return the per-frame state log.
std::vector<FrameState> runVeloxScene(const DiffTestScene& scene);

// Compare two engine outputs; returns statistical diff result.
DiffTestResult compareEngines(
    const std::string& nameA,
    const std::vector<FrameState>& statesA,
    const std::string& nameB,
    const std::vector<FrameState>& statesB,
    float positionTolerance = 1e-3f,
    float velocityTolerance = 0.1f,
    float energyTolerance = 0.01f);

} // namespace test
} // namespace velox

// Per-frame captured state for comparison.
struct FrameState {
    int frame;
    float time;
    struct BodySample {
        BodyId id;
        Vec3 position;
        Quat orientation;
        Vec3 velocity;
        Vec3 angularVelocity;
    };
    std::vector<BodySample> bodies;
};

```

## Data structures

- `DiffTestScene` struct — new file `tests/diff_test.h`. Describes a reproducible scene with deterministic body placement and force application.
- `DiffTestResult` struct — new file `tests/diff_test.h`. Statistical comparison output.
- `FrameState` struct — new file `tests/diff_test.h`. Per-frame snapshot of tracked bodies.
- `test::runVeloxScene()` function — new, lives in `tests/diff_test.cpp`.
- `test::compareEngines()` function — new, lives in `tests/diff_test.cpp`.

## Algorithm

**Differential testing workflow:**

1. **Define scenes as data.** Each scene is a plain struct (DiffTestScene) that can be serialized to JSON for sharing between CI runs and human review. Scenes include: gravity, substeps, dt, body descriptions with deterministic random seeds, force events at specific frames, and which bodies to track.
2. **Run Velox deterministically.** Execute the scene through `World::step()` with fixed dt and substeps. At each frame, sample tracked bodies' positions, orientations, velocities, angular velocities into a FrameState.
3. **Run reference engine (Jolt/Bullet).** A separate test binary links against Jolt or Bullet, runs the same DiffTestScene description (parsed from JSON), and produces FrameStates in the same format. The scene parser is engine-agnostic; only the execution layer differs.
4. **Compare statistically.** For each tracked body, compute:
   - Max absolute position delta across all frames
   - Max absolute velocity delta across all frames
   - Total energy at each frame (KE + PE); max drift from frame 0
   - Pearson correlation coefficient on position time series (catches phase shifts)
5. **Pass/fail criteria.** A scene passes if: max position delta < 1 mm, max velocity delta < 0.1 m/s, energy drift < 1% of initial energy, trajectory correlation > 0.99. These tolerances account for solver differences (Box2D vs Bullet use different integration schemes) while catching regressions.

**Scene library.** Ship 5 canonical scenes:
- `stack_of_boxes.json` — 10-box tower, tests resting stability
- `pendulum.json` — constrained pendulum, tests energy conservation
- `rain.json` — 100 falling spheres, tests broad-phase + solver throughput
- `bullet_through_walls.json` — high-speed sphere through 3 planes, tests CCD
- `ragdoll.json` — 8-body ragdoll, tests joint solving

## Files

**New files:**
- `tests/diff_test.h` — DiffTestScene, DiffTestResult, FrameState definitions
- `tests/diff_test.cpp` — Velox execution and comparison logic
- `tests/scenes/stack_of_boxes.json` — canonical test scene data
- `tests/scenes/pendulum.json`
- `tests/scenes/rain.json`
- `tests/scenes/bullet_through_walls.json`
- `tests/scenes/ragdoll.json`
- `tests/diff_jolt.cpp` — Jolt backend adapter (separate test binary)
- `tests/diff_bullet.cpp` — Bullet backend adapter (separate test binary)

**Modified files:**
- None to core engine; all changes are in tests/

## Tests

1. **Velox vs Velox self-comparison:** Run the same scene twice through Velox with identical seeds. Max delta must be < 1e-8 (machine epsilon). Catches non-determinism in the solver.
2. **Stack of boxes: Velox vs Jolt.** Both engines simulate a 10-box tower for 10 seconds. Position deltas < 5 mm, energy drift < 2%. Catches stacking regressions.
3. **Pendulum energy conservation.** 60-second pendulum swing; energy drift must be < 0.5% in both engines. If Velox drifts at 2% while Jolt drifts at 0.3%, investigate Velox's integration scheme.
4. **Bullet-through-walls CCD.** Sphere at 2 km/s through 3 walls; both engines must register all 3 impacts within ±1 frame. Catches TOI calculation regressions.

## Acceptance

- [ ] DiffTestScene struct can describe all 5 canonical scenes as JSON
- [ ] `runVeloxScene()` produces FrameStates matching the scene description exactly
- [ ] `compareEngines()` computes position/velocity/energy deltas and correlation correctly (verified against hand-computed example)
- [ ] Self-comparison test passes with delta < 1e-8
- [ ] Stack-of-boxes scene runs in Jolt and produces comparable output (integration verified)

## Size: M

## Risks

- Different engines use different integration schemes (explicit Euler vs symplectic Euler vs RK4); even with identical initial conditions, trajectories will diverge over time. The tolerance must be generous enough to allow this natural divergence while still catching regressions.
- Jolt and Bullet may not be available in all CI environments. Must make the differential test optional (skip if reference engine not linked) rather than a hard requirement.
- Scene JSON format must be engine-agnostic; if Velox adds a new shape type, the JSON parser for Jolt/Bullet must be updated too. Consider a shared schema definition file.
