#pragma once
#include "math.h"
#include <cstdint>

namespace velox {

using BodyId = uint32_t;

enum class ShapeType : uint8_t { Sphere, Plane };

// Data-oriented body layout: plain structs in contiguous arrays so the same
// data can be uploaded to a GPU backend unchanged.
struct Body {
    Vec3 position;
    Vec3 velocity;
    float invMass = 1.0f;       // 0 => static / infinite mass
    float restitution = 0.3f;
    float friction = 0.5f;

    ShapeType shape = ShapeType::Sphere;
    float radius = 0.5f;        // Sphere
    Vec3 planeNormal{0, 1, 0};  // Plane: dot(n, p) = planeOffset
    float planeOffset = 0.0f;

    bool isStatic() const { return invMass == 0.0f; }
};

} // namespace velox
