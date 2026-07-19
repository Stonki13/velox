#pragma once

#include "math.h"
#include <cstdint>

namespace velox {

enum class MotionQuality : uint8_t { Low = 0, Medium = 1, High = 2, Locked = 3 };

struct BodyCcdTuning {
    MotionQuality quality = MotionQuality::Medium;
    float collisionMargin = 0.0f;
    float speculativeDistance = 0.0f;
    bool enableContinuous = true;
    float minVelocityForCCD = 1.0f;
};

struct WorldCcdDefaults {
    MotionQuality defaultQuality = MotionQuality::Medium;
    float defaultCollisionMargin = 0.0f;
    float defaultSpeculativeDistance = 0.01f;
    bool defaultEnableContinuous = true;
    float defaultMinVelocityForCCD = 1.0f;
};

struct CcdConfig {
    bool enabled = true;
    uint32_t maxToiEventsPerBody = 4;
    float toiVelocityFloor = 0.1f;
    float toiPenetrationBias = 1e-3f;
};

struct WorldMultiToiSettings {
    CcdConfig defaultConfig;
    uint32_t maxTotalEventsPerStep = 256;
    bool enableSubstepSplitting = true;
};

} // namespace velox
