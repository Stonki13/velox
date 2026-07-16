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
// infinite plane so both engines see identical geometry.
struct BodyDesc {
    enum class Shape { Sphere, Box, Capsule, GroundBox };
    Shape shape = Shape::Sphere;
    Vec3f position{};
    Quatf orientation{};
    float radius = 0.5f;             // Sphere/Capsule
    float capsuleHalfHeight = 0.5f;  // Capsule
    Vec3f halfExtents{0.5f, 0.5f, 0.5f}; // Box/GroundBox
    float mass = 1.0f;               // 0 = static
    float restitution = 0.0f;
    float friction = 0.5f;
    Vec3f initialVelocity{};
    bool highSpeedCcd = false;       // request continuous collision handling
};

// Point (ball-socket) joint between two bodies by scene index.
struct JointDesc {
    int bodyA = -1;
    int bodyB = -1;
    Vec3f worldAnchor{};
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
    float maxEnergyDrift = 0.05f;
    float minCorrelation = 0.99f;
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

} // namespace difftest
