#include "diff_test.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

// Differential test driver. Gates:
//  1. Velox vs Velox self-comparison must be bitwise identical (determinism).
//  2. Each canonical scene runs in Velox and Jolt; statistical comparison
//     against per-scene tolerances, plus behavioral checks for scenes where
//     pointwise cross-engine comparison is meaningless (chaotic stacks, CCD).

namespace {

using namespace difftest;

bool selfDeterminism(const SceneDesc& scene) {
    const Trajectory a = runVelox(scene);
    const Trajectory b = runVelox(scene);
    if (a.size() != b.size()) return false;
    for (size_t frame = 0; frame < a.size(); ++frame)
        for (size_t body = 0; body < a[frame].bodies.size(); ++body) {
            const BodySample& sa = a[frame].bodies[body];
            const BodySample& sb = b[frame].bodies[body];
            if (std::memcmp(&sa, &sb, sizeof(BodySample)) != 0) return false;
        }
    return true;
}

Tolerances tolerancesFor(const std::string& name) {
    Tolerances t;
    if (name == "sphere_drop") {
        // Restitution models differ slightly between engines; bounce apexes
        // drift a few cm late in the run. Velocity is compared at discrete
        // frames, so an impact landing one frame apart between engines shows
        // a momentary delta up to twice the impact speed — bound accordingly.
        t.maxPositionDelta = 0.35f;
        t.maxVelocityDelta = 8.0f;
        t.minCorrelation = 0.97f;
        t.maxEnergyDrift = 0.02f;
    } else if (name == "box_stack") {
        // Chaotic in detail: rely on behavioral bounds + no energy gain.
        t.maxPositionDelta = 1.5f;
        t.maxVelocityDelta = 8.0f;
        t.minCorrelation = 0.0f;
        t.maxEnergyDrift = 0.02f;
    } else if (name == "pendulum") {
        // Joint solvers differ (Velox TGS vs Jolt); phase drifts over 6 s.
        t.maxPositionDelta = 1.0f;
        t.maxVelocityDelta = 3.0f;
        t.minCorrelation = 0.90f;
        t.maxEnergyDrift = 0.05f;
    } else if (name == "sphere_roll") {
        t.maxPositionDelta = 1.5f;
        t.maxVelocityDelta = 2.0f;
        t.minCorrelation = 0.97f;
        t.maxEnergyDrift = 0.02f;
    } else if (name == "ccd_wall") {
        // The wall must stop the bullet in both engines (behavioral check is
        // the real gate). A one-frame difference in the bounce instant shows
        // a momentary velocity delta up to the full approach+exit speed.
        t.maxPositionDelta = 6.0f;
        t.maxVelocityDelta = 170.0f;
        t.minCorrelation = 0.90f;
        t.maxEnergyDrift = 0.02f;
    }
    return t;
}

// Scene-specific behavioral invariants that hold in ANY correct engine.
bool behavioralChecks(const SceneDesc& scene, const char* engine,
                      const Trajectory& trajectory) {
    if (trajectory.empty()) return false;
    const FrameState& last = trajectory.back();

    if (scene.name == "box_stack") {
        // The tower must still stand: every box near the axis, none launched.
        for (const BodySample& body : last.bodies) {
            const float horizontal =
                std::sqrt(body.position.x * body.position.x +
                          body.position.z * body.position.z);
            if (horizontal > 0.35f || body.position.y < 0.0f) {
                std::fprintf(stderr, "  [%s] box_stack: tower collapsed (r=%.3f y=%.3f)\n",
                             engine, horizontal, body.position.y);
                return false;
            }
        }
    }
    if (scene.name == "ccd_wall") {
        // Wall face is at x = 9.75; the bullet must never pass it.
        for (const FrameState& frame : trajectory)
            if (frame.bodies[0].position.x > 9.8f) {
                std::fprintf(stderr, "  [%s] ccd_wall: bullet tunneled (x=%.2f at frame %d)\n",
                             engine, frame.bodies[0].position.x, frame.frame);
                return false;
            }
    }
    if (scene.name == "pendulum") {
        // The bob must stay on the 2 m arm (within solver slack).
        for (const FrameState& frame : trajectory) {
            const Vec3f p = frame.bodies[0].position;
            const float arm = std::sqrt((p.x - 0.0f) * (p.x - 0.0f) +
                                        (p.y - 5.0f) * (p.y - 5.0f) + p.z * p.z);
            if (std::fabs(arm - 2.0f) > 0.15f) {
                std::fprintf(stderr, "  [%s] pendulum: arm length %.3f at frame %d\n",
                             engine, arm, frame.frame);
                return false;
            }
        }
    }
    if (scene.name == "sphere_roll") {
        // Friction must slow the ball: it travels forward but not forever.
        const float x = last.bodies[0].position.x;
        if (x < -6.0f || x > 30.0f) {
            std::fprintf(stderr, "  [%s] sphere_roll: final x=%.2f out of range\n", engine, x);
            return false;
        }
    }
    return true;
}

} // namespace

int main() {
    int failures = 0;
    const std::vector<SceneDesc> scenes = canonicalScenes();

    // Gate 1: determinism.
    for (const SceneDesc& scene : scenes) {
        if (!selfDeterminism(scene)) {
            std::fprintf(stderr, "FAIL determinism: %s (velox vs velox differs)\n",
                         scene.name.c_str());
            ++failures;
        }
    }
    std::printf("determinism: velox self-comparison bitwise identical on %zu scenes\n",
                scenes.size());

    // Gate 2: velox vs jolt.
    std::printf("%-14s %10s %10s %9s %9s %6s  %s\n", "scene", "maxPosD", "maxVelD",
                "eDriftVx", "eDriftJl", "corr", "verdict");
    for (const SceneDesc& scene : scenes) {
        const Trajectory velox = runVelox(scene);
        const Trajectory jolt = runJolt(scene);
        const DiffResult result = compare(scene, velox, jolt, tolerancesFor(scene.name));
        const bool behaviorVelox = behavioralChecks(scene, "velox", velox);
        const bool behaviorJolt = behavioralChecks(scene, "jolt", jolt);
        const bool pass = result.passed && behaviorVelox && behaviorJolt;
        std::printf("%-14s %10.4f %10.3f %9.4f %9.4f %6.3f  %s\n", scene.name.c_str(),
                    result.maxPositionDelta, result.maxVelocityDelta, result.energyDriftA,
                    result.energyDriftB, result.trajectoryCorrelation,
                    pass ? "PASS" : "FAIL");
        if (!pass) ++failures;
    }

    if (failures) {
        std::fprintf(stderr, "difftest: %d failure(s)\n", failures);
        return 1;
    }
    std::puts("difftest: all scenes passed");
    return 0;
}
