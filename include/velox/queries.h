#pragma once

#include "body.h"
#include "math.h"
#include <cstdint>
#include <string>
#include <vector>

namespace velox {

struct RayHit {
    bool hit = false;
    BodyId body;
    float t = 0.0f;
    Vec3 point, normal;
};

struct QueryFilter {
    uint32_t categoryBits = UINT32_MAX;
    uint32_t maskBits = UINT32_MAX;
    bool includeSensors = true;
    BodyId ignoredBody;
};

struct ShapeCastHit {
    bool hit = false;
    BodyId body;
    float distance = 0.0f;
    float fraction = 0.0f;
    Vec3 point, normal;
};

// A value-type request for one scene query. Batch and async queries accept
// only primitives so the request can be copied safely across worker threads.
struct QueryDesc {
    enum class Type : uint8_t {
        Raycast,
        RaycastAll,
        OverlapSphere,
        OverlapBox,
        OverlapCapsule,
        SphereCast,
        BoxCast,
        CapsuleCast,
    };

    Type type = Type::Raycast;
    QueryFilter filter{};
    Vec3 origin{}, direction{};
    float maxDist = 1e10f;
    Vec3 center{};
    Vec3 halfExtents{};
    Quat orientation{};
    float radius = 0.0f;
    float capsuleHalfHeight = 0.0f;
    void* userData = nullptr;
};

// Every field is value-owned so a result remains valid after the World has
// advanced. Only the field matching type is populated on successful queries.
struct QueryResult {
    bool success = false;
    QueryDesc::Type type = QueryDesc::Type::Raycast;
    void* userData = nullptr;
    RayHit rayHit{};
    std::vector<RayHit> rayHits;
    ShapeCastHit shapeCastHit{};
    std::vector<BodyId> overlaps;
    std::string error;
};

struct AsyncQueryHandle {
    uint64_t id = 0;
};

} // namespace velox
