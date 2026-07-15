#pragma once

#include "world.h"

namespace velox {

struct CharacterControllerDesc {
    float capsuleRadius = 0.3f;
    float capsuleHalfHeight = 0.9f;
    float stepMaxHeight = 0.3f;
    float slopeLimitCosine = 0.7071f;
    // Move() takes an already-scaled displacement; applications may use this
    // value when converting input and dt into that displacement.
    float movementSpeed = 5.0f;
    float ghostPadding = 0.01f;
};

struct CharacterControllerResult {
    Vec3 finalPosition{};
    Vec3 slideVelocity{};
    bool grounded = false;
    bool stepped = false;
    bool hitWall = false;
    int contactCount = 0;
    Vec3 groundNormal{};
};

// Query-based capsule controller. It owns position state only and never adds,
// removes, or mutates a World body.
class CharacterController {
public:
    explicit CharacterController(World& world, const CharacterControllerDesc& desc = {});

    // Set the query capsule's center. Call after spawning or teleporting.
    void SetPosition(Vec3 position);
    Vec3 Position() const { return position_; }

    // Resolve a desired displacement with up to four deterministic sweeps.
    CharacterControllerResult Move(Vec3 displacement);

    // With no timestep in the query-only API, v is an upward displacement
    // applied by the next Move when the controller is grounded.
    void SetJumpVelocity(float v) { jumpVel_ = v; }

private:
    bool probeGround(Vec3 position, ShapeCastHit& hit) const;
    bool tryStep(Vec3& position, Vec3 remaining, CharacterControllerResult& result) const;

    World& world_;
    CharacterControllerDesc desc_;
    Vec3 position_{};
    float jumpVel_ = 0.0f;
};

} // namespace velox
