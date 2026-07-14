// character_controller.h — Design sketch for capsule sweep-and-slide character controller.
// Self-contained: includes only what's needed for compilation in isolation.
// TODO: integrate with velox::World, velox::BodyId, velox::Vec3, velox::Quat.

#pragma once

#include <cstdint>
#include <vector>
#include <array>

// Forward declarations — replace with #include "velox/world.h" etc. when integrating.
namespace velox {
    struct Vec3;
    struct Quat;
    class World;
    using BodyId = uint64_t;
}

namespace velox {

// Character controller configuration. Tunable per-character.
struct CharacterControllerConfig {
    float capsuleRadius = 0.3f;           // meters
    float capsuleHalfHeight = 0.9f;       // half-height of the cylindrical portion
    float stepHeight = 0.3f;              // max height of obstacle that can be climbed
    float slopeLimitDegrees = 45.0f;      // max slope angle before sliding
    float minSlopeCosine = 0.7071f;       // cos(45°); skip trig at runtime
    float movementSpeed = 5.0f;           // max horizontal movement speed (m/s)
    float jumpVelocity = 8.0f;            // upward velocity applied on jump (m/s)
    bool enableAutoWalkOnSlope = true;    // apply downward force to stay on slopes
    float ghostPadding = 0.01f;           // extra margin for sweep queries (meters)
};

// Per-frame character controller state. Populated by Step().
struct CharacterControllerState {
    bool grounded = false;                // touching the ground this frame
    bool stepped = false;                 // climbed an obstacle this frame
    bool hitWall = false;                 // horizontal movement blocked
    int contactCount = 0;                 // number of ground contacts (for normal averaging)

    Vec3 groundNormal{};                  // averaged normal from all ground contacts
    float slopeAngleDegrees = 0.0f;       // angle of the ground plane (degrees)
    Vec3 slideVelocity{};                 // velocity after wall/slope resolution
};

// Character controller: capsule-shaped kinematic body with sweep-and-slide.
// Owns no velox::Body — instead, the user provides position/velocity and the
// controller computes the resolved motion via queries against the World.
class CharacterController {
public:
    // Construct a controller bound to a specific world. The config defines the
    // capsule geometry and physical parameters.
    explicit CharacterController(World& world, const CharacterControllerConfig& config);

    // Step the controller: given a desired displacement this frame, compute the
    // resolved motion accounting for collisions, step climbing, and slope limits.
    // Returns true if the character is grounded at the end of the step.
    //
    // TODO: implement sweep-and-slide loop with up to 4 iterations to handle
    //       cascading wall contacts (e.g., sliding along two perpendicular walls).
    bool Step(Vec3 desiredDisplacement, CharacterControllerState& outState);

    // Request a jump. The controller applies an upward impulse on the next Step()
    // call if the character is grounded. Must be called before Step().
    //
    // TODO: integrate with velox::addLinearImpulse or direct velocity modification.
    void Jump();

    // Set the desired movement direction (normalized). Call before Step().
    // The controller scales this by movementSpeed to produce the displacement.
    //
    // TODO: add support for variable speed (sprint, crouch) via a multiplier.
    void SetMovementDirection(Vec3 direction);

    // Query the current ground slope angle in degrees. Useful for game logic
    // (e.g., slowing down on steep slopes, triggering slide animations).
    float GetSlopeAngle() const;

    // Check if the character is currently grounded.
    bool IsGrounded() const;

    // Reset internal state (clear cached contacts, ground normal, etc.).
    // Call after teleporting the character or loading a saved position.
    //
    // TODO: decide whether to also reset the velocity or leave it to the caller.
    void Reset();

private:
    World& world_;
    CharacterControllerConfig config_;

    // Internal state between frames.
    Vec3 currentPosition_{};
    Vec3 currentVelocity_{};
    Quat currentOrientation_{};
    bool grounded_ = false;
    float slopeAngle_ = 0.0f;
    bool jumpRequested_ = false;

    // TODO: replace with velox::QueryFilter for category/mask filtering.
    // Currently uses default filter (all bodies).
};

} // namespace velox
