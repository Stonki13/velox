# 10 — Real-Workload Scene

## Goal

Define a game-like level scene that serves as the final release gate for Velox 1.0. The scene must exercise every major subsystem (broad phase, narrow phase, solver, CCD, joints, sleeping, queries) under realistic conditions. Passing this scene with all Tier 1 checks green is required before any 1.0 tag.

## Public API

```cpp
namespace velox {
namespace test {

// Release gate configuration: which Tier 1 items must pass for the scene to be valid.
struct ReleaseGateConfig {
    bool requirePersistentManifolds = true;   // item 01
    bool requireGjkHardening = true;          // item 02
    bool requireMultiToiCcd = true;           // item 03
    bool requireCcdQualityControls = true;    // item 04
    bool requireThreadSafety = true;          // item 05
    bool requireApiVersioning = true;         // item 06
    bool requireDocumentation = true;         // item 07
    bool requireDifferentialTesting = true;   // item 08
    bool requireFuzzing = true;               // item 09
};

// Real-workload scene definition: a game-like level with bodies, joints, and queries.
struct RealWorkloadScene {
    std::string name = "game_level";

    // Level geometry: static planes, meshes, heightfields.
    struct StaticGeometry {
        Vec3 planeNormal{0, 1, 0};
        float planeOffset = 0.0f;
        std::vector<Vec3> meshVertices;
        std::vector<uint32_t> meshIndices;
        uint32_t heightfieldWidth = 0;
        uint32_t heightfieldDepth = 0;
        float heightfieldCellSize = 1.0f;
        std::vector<float> heightfieldHeights;
    };
    StaticGeometry levelGeometry;

    // Dynamic bodies: boxes, spheres, capsules with realistic masses.
    struct DynamicBody {
        enum class Shape { Sphere, Box, Capsule, Hull };
        Shape shape;
        Vec3 position;
        float radius = 0.5f;
        Vec3 halfExtents{0.5f, 0.5f, 0.5f};
        float capsuleHalfHeight = 0.5f;
        std::vector<Vec3> hullPoints;
        float mass = 1.0f;
        Vec3 initialVelocity{};
    };
    std::vector<DynamicBody> bodies;

    // Joints connecting dynamic bodies (ragdoll, vehicle suspension, etc.).
    struct JointDesc {
        enum class Type { Hinge, Distance, Spring, ConeTwist, Fixed, Prismatic, SixDof };
        Type type;
        BodyId bodyA, bodyB;
        Vec3 anchorA, anchorB;
        Vec3 axis{0, 1, 0};
        float limitLower = 0.0f, limitUpper = 0.0f;
        float springFrequency = 0.0f, springDamping = 1.0f;
    };
    std::vector<JointDesc> joints;

    // Queries to run at specific frames for validation.
    struct QueryEvent {
        int frame;
        enum class Type { Raycast, OverlapSphere, ShapeCast } type;
        // ... query parameters depending on type
    };
    std::vector<QueryEvent> queries;

    // Simulation parameters.
    float dt = 1.0f / 60.0f;
    int substeps = 4;
    Vec3 gravity{0, -9.81f, 0};
    int totalFrames = 3600;  // 1 minute at 60 Hz
};

// Run the release gate: execute the scene, run all Tier 1 checks, report results.
struct ReleaseGateResult {
    bool passed = false;
    std::vector<std::string> failures;  // list of failed checks
    float totalSimulationTimeMs = 0.0f;
    float maxEnergyDrift = 0.0f;
    int bodiesSleepingAtEnd = 0;
};
ReleaseGateResult runReleaseGate(const RealWorkloadScene& scene,
                                  const ReleaseGateConfig& config);

} // namespace test
} // namespace velox
```

## Data structures

- `ReleaseGateConfig` struct — new file `tests/release_gate.h`. Which Tier 1 items are required.
- `RealWorkloadScene` struct — new file `tests/release_gate.h`. Full scene definition with geometry, bodies, joints, queries.
- `ReleaseGateResult` struct — new file `tests/release_gate.h`. Aggregated pass/fail report.
- `test::runReleaseGate()` — new function in `tests/release_gate.cpp`.

## Algorithm

**Scene composition (the "game level"):**

1. **Terrain.** A static plane at y=0 plus a heightfield representing rolling hills (width=64, depth=64, cellSize=2.0, heights from a procedural noise function). Tests broad-phase acceleration over large static worlds.
2. **Structures.** A box-built tower (5 stories, each 4 boxes stacked), a ramp (kinematic box at 30° angle), and a triangle-mesh bridge spanning a gap. Tests resting stability, slope interaction, and mesh collision.
3. **Dynamic objects.** 50 random bodies (mix of spheres, boxes, capsules) dropped from various heights. Masses range from 0.1 kg to 100 kg. Some given initial velocities (thrown objects). Tests CCD, sleeping/waking, and solver throughput.
4. **Joints.** A 6-body ragdoll (ball joints at shoulders/elbows/waist/hips/knees) standing on the ramp. A simple pendulum (hinge joint, 2 m arm). A spring-mounted platform (spring joint between static ceiling and dynamic platform). Tests joint solving across all types.
5. **Queries.** At frame 1800 (30 seconds in), run a raycast from above to count bodies intersected; an overlap sphere query around the ragdoll; a shape cast of a sphere through the bridge gap. Validates that queries return consistent results mid-simulation.

**Release gate checks:**

1. **Tier 1 item verification.** For each required item in `ReleaseGateConfig`, verify the corresponding feature is compiled and functional:
   - Item 01: Manifold point count ≥ 3 for resting box-on-box contacts (check via debug lines or internal counter).
   - Item 02: GJK/EPA handles a degenerate test shape without crash (run a quick degenerate pair before the main scene).
   - Item 03: Multi-TOI processes ≥ 2 events for a high-speed body in the scene.
   - Item 04: Per-body CCD quality flags are accessible and affect behavior (verify by toggling and observing step time changes).
   - Item 05: Thread safety policy can be set to Relaxed; cross-thread query succeeds.
   - Item 06: Version constants are defined; deprecation macro compiles.
   - Item 07: Doxygen builds without errors from the public headers used in the scene.
   - Item 08: Differential test runs for the stack-of-boxes sub-scene with tolerance check.
   - Item 09: Fuzzer runs 10⁴ pairs with zero failures before the main scene starts.
2. **Simulation stability.** Run the full 3600 frames. Check:
   - Energy drift < 5% from initial total energy.
   - No body tunnels through the terrain (verify min y-position ≥ -0.01 m for all bodies).
   - At least 80% of bodies are sleeping by frame 3600.
   - Step time < 10 ms on reference hardware (CPU backend).
3. **Query consistency.** At frame 1800, run the defined queries and verify:
   - Raycast returns ≥ 1 hit (the tower should be visible from above).
   - Overlap sphere around ragdoll returns ≥ 6 bodies (the ragdoll itself).
   - Shape cast through bridge gap returns a hit (the bridge geometry).

## Files

**New files:**
- `tests/release_gate.h` — ReleaseGateConfig, RealWorkloadScene, ReleaseGateResult definitions
- `tests/release_gate.cpp` — scene execution and check logic
- `tests/scenes/game_level.json` — serialized scene definition for reproducibility
- `docs/release-gate.md` — documentation of the release gate process

**Modified files:**
- None to core engine; all changes in tests/ and docs/

## Tests

1. **Full release gate pass.** Run `runReleaseGate(gameLevel, allRequired)`. Must return `passed = true` with zero failures.
2. **Regression detection.** After any Tier 1 change, re-run the release gate. If a previously-passing check now fails, the change is flagged for review.
3. **Performance regression.** Step time must remain < 10 ms (CPU) / < 3 ms (CUDA) after any solver optimization. Measured at frame 3600 (steady state).

## Acceptance

- [ ] `RealWorkloadScene` defines a level with terrain, structures, 50+ dynamic bodies, and joints
- [ ] Release gate runs all required Tier 1 checks before declaring success
- [ ] Simulation completes 3600 frames without tunneling or energy drift > 5%
- [ ] ≥ 80% of bodies sleeping by end of simulation
- [ ] All three query events return valid results at frame 1800
- [ ] Step time < 10 ms on reference CPU hardware

## Size: M

## Risks

- The scene is complex; debugging a failure requires isolating which subsystem caused the issue. Must log per-subsystem metrics (broad-phase time, solver time, CCD time) to the release gate report.
- "Game-like" is subjective. The scene must be documented thoroughly enough that any engineer can reproduce it exactly from the JSON definition. Include seed values for procedural generation.
- Performance benchmarks are hardware-dependent. The 10 ms CPU / 3 ms CUDA thresholds must be adjusted per CI machine; document the reference hardware specs.
