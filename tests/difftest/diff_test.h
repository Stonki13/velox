#pragma once
// Differential testing: run identical scenes through Velox and a reference
// engine (Jolt) and compare trajectories statistically. Scenes are plain
// engine-agnostic descriptors so each runner only implements execution.
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace difftest {

struct Vec3f {
    float x = 0.0f, y = 0.0f, z = 0.0f;
};

struct Quatf {
    float x = 0.0f, y = 0.0f, z = 0.0f, w = 1.0f;
};

// One body in a scene. GroundBox is a large static box used instead of an
// infinite plane so both engines see identical geometry. Mesh is a static
// triangle soup (terrain); meshVertices/meshIndices are only populated for it.
struct BodyDesc {
    enum class Shape { Sphere, Box, Capsule, GroundBox, Mesh };
    Shape shape = Shape::Sphere;
    Vec3f position{};
    Quatf orientation{};
    float radius = 0.5f;             // Sphere/Capsule
    float capsuleHalfHeight = 0.5f;  // Capsule
    Vec3f halfExtents{0.5f, 0.5f, 0.5f}; // Box/GroundBox
    std::vector<Vec3f> meshVertices; // Mesh
    std::vector<uint32_t> meshIndices; // Mesh, 3 per triangle
    float mass = 1.0f;               // 0 = static
    float restitution = 0.0f;
    float friction = 0.5f;
    Vec3f initialVelocity{};
    Vec3f initialAngularVelocity{};
    bool highSpeedCcd = false;       // request continuous collision handling
    bool gyroscopic = false;         // Jolt: enable gyroscopic force
    // Both engines' default sleep heuristics can put a body with a
    // motorized joint to sleep before the motor has finished driving it
    // (neither engine automatically exempts motor-driven bodies from
    // sleeping). Set false for bodies attached to a HingeMotor joint.
    bool allowSleep = true;
};

// A constraint between two bodies by scene index. Ball is a point (ball-
// socket) joint; HingeMotor additionally drives relative rotation about
// worldAxis at motorSpeed, budgeted by maxMotorTorque -- exercises each
// engine's motorized-joint solver rather than just a passive constraint.
struct JointDesc {
    enum class Type { Ball, HingeMotor };
    Type type = Type::Ball;
    int bodyA = -1;
    int bodyB = -1;
    Vec3f worldAnchor{};
    Vec3f worldAxis{0.0f, 1.0f, 0.0f}; // HingeMotor only
    float motorSpeed = 0.0f;           // rad/s, HingeMotor only
    float maxMotorTorque = 0.0f;       // N*m, HingeMotor only
};

struct SceneDesc {
    std::string name;
    Vec3f gravity{0.0f, -9.81f, 0.0f};
    float dt = 1.0f / 60.0f;
    int substeps = 4;
    int frames = 180;
    std::vector<BodyDesc> bodies;
    std::vector<JointDesc> joints;
    std::vector<int> tracked; // indices into bodies
};

struct BodySample {
    Vec3f position{};
    Quatf orientation{};
    Vec3f velocity{};
    Vec3f angularVelocity{};
};

struct FrameState {
    int frame = 0;
    float time = 0.0f;
    std::vector<BodySample> bodies; // one per tracked body, scene order
};

using Trajectory = std::vector<FrameState>;

// Statistical comparison between two engines' trajectories of one scene.
struct DiffResult {
    float maxPositionDelta = 0.0f;   // meters, over all frames and bodies
    float maxVelocityDelta = 0.0f;   // m/s
    float maxAngularDelta = 0.0f;    // rad/s
    float energyDriftA = 0.0f;       // relative KE+PE drift of engine A
    float energyDriftB = 0.0f;       // relative KE+PE drift of engine B
    float trajectoryCorrelation = 1.0f; // min Pearson r across axes/bodies
    bool passed = false;
};

// Per-scene tolerances: cross-engine solver differences make one universal
// millimeter bound unrealistic (chaotic scenes diverge exponentially), so
// every scene picks bounds that catch regressions without false alarms.
struct Tolerances {
    float maxPositionDelta = 5e-3f;
    float maxVelocityDelta = 0.25f;
    float maxAngularDelta = 1e30f;   // opt-in per scene
    float maxEnergyDrift = 0.05f;
    float minCorrelation = 0.99f;
    bool correlateAngular = false;   // Pearson over angular velocity series
};

inline float length(const Vec3f& v) {
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

// Total mechanical energy (kinetic + gravitational potential) of the tracked
// bodies at one frame. Rotational KE is deliberately included only through
// the angular speed magnitude with a rough inertia bound; the drift metric
// compares an engine against itself over time, so an exact tensor is not
// required for regression detection.
float totalEnergy(const SceneDesc& scene, const FrameState& frame);

DiffResult compare(const SceneDesc& scene, const Trajectory& a, const Trajectory& b,
                   const Tolerances& tolerances);

// Runners (each in its own translation unit; jolt_runner links Jolt).
Trajectory runVelox(const SceneDesc& scene);
Trajectory runJolt(const SceneDesc& scene);

// Canonical scene library.
std::vector<SceneDesc> canonicalScenes();

// ---------------------------------------------------------------------------
// Character-controller cross-engine comparison
// ---------------------------------------------------------------------------

/// Describes a character-on-slope scenario for cross-engine validation.
struct CharacterSceneDesc {
    std::string name;
    float slopeAngleDeg = 0.0f;      ///< Inclination of the ramp (degrees).
    Vec3f targetVelocity{0, 0, 3};   ///< Desired horizontal walk velocity.
    int frames = 120;                ///< Simulation frames at dt.
    float dt = 1.0f / 60.0f;
    float capsuleRadius = 0.3f;
    float capsuleHalfHeight = 0.9f;
    float slopeLimitCosine = 0.7071f; ///< ~45 degrees.
};

/// Outcome of one engine's character run.
struct CharacterResult {
    Vec3f finalPosition{};
    bool grounded = false;
    float heightGained = 0.0f;       ///< Y difference from start.
    float horizontalDistance = 0.0f; ///< XZ distance from start.
};

/// Behavioral comparison between two engines' character results.
struct CharacterDiffResult {
    bool bothGrounded = false;
    bool agreeOnClimb = false;       ///< Both gained height (or both didn't).
    float positionDelta = 0.0f;      ///< Distance between final positions.
    bool passed = false;
};

CharacterResult runVeloxCharacter(const CharacterSceneDesc& scene);
CharacterResult runJoltCharacter(const CharacterSceneDesc& scene);
CharacterDiffResult compareCharacter(const CharacterSceneDesc& scene,
                                     const CharacterResult& a,
                                     const CharacterResult& b);
std::vector<CharacterSceneDesc> characterScenes();

} // namespace difftest
