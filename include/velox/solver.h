#pragma once

#include <cstdint>
#include <string>

namespace velox {

// The two-axis solver is the established low-cost path. ConeBlockSolver uses
// the coupled normal/tangent effective-mass block and projects its accumulated
// impulse into the Coulomb cone.
enum class FrictionModel : uint8_t {
    TwoAxisCoulomb = 0,
    ConeBlockSolver = 1,
};

enum class IterationPolicy : uint8_t {
    Fixed = 0,
    Adaptive = 1,
};

// Kept trivially-copyable because CPU and CUDA backends consume the exact same
// settings and rollback snapshots serialize the complete per-world state.
struct SolverOptions {
    float impulseThreshold = 1e-5f;
    float stackStiffness = 1000.0f;
    float stackDamping = 10.0f;
    int velocityIterations = 8;
    // Three is the established Velox default. Higher values are available
    // through presets without perturbing existing worlds at upgrade time.
    int positionIterations = 3;
    FrictionModel frictionModel = FrictionModel::TwoAxisCoulomb;
    IterationPolicy iterationPolicy = IterationPolicy::Fixed;
    bool enableStackStabilization = false;
    uint8_t reserved = 0;
};

struct SolverPreset {
    std::string name;
    SolverOptions options;
};

inline SolverPreset PresetHighQuality() {
    SolverPreset preset{"High Quality", {}};
    preset.options.frictionModel = FrictionModel::ConeBlockSolver;
    preset.options.velocityIterations = 16;
    preset.options.positionIterations = 6;
    return preset;
}

inline SolverPreset PresetFast() {
    SolverPreset preset{"Fast", {}};
    preset.options.velocityIterations = 4;
    preset.options.positionIterations = 2;
    return preset;
}

inline SolverPreset PresetStacking() {
    SolverPreset preset{"Stacking", {}};
    preset.options.frictionModel = FrictionModel::ConeBlockSolver;
    preset.options.velocityIterations = 12;
    preset.options.positionIterations = 6;
    preset.options.enableStackStabilization = true;
    return preset;
}

} // namespace velox
