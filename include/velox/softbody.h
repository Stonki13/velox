#pragma once
/**
 * @file softbody.h
 * @brief XPBD soft-body solver: cloth and deformable volumetric primitives.
 *
 * Soft bodies are collections of particles connected by distance constraints,
 * solved with Extended Position-Based Dynamics (XPBD). They collide with
 * rigid bodies (planes, spheres, boxes) but do not self-collide.
 *
 * Two factory helpers are provided:
 * - @ref makeClothSoftBody — a rectangular cloth grid (structural + shear).
 * - @ref makeSoftSphereSoftBody — a deformable sphere (surface + core).
 *
 * Soft bodies are owned by @ref World and stepped inside @ref World::step
 * after the rigid-body solve. Access particle state through
 * @ref World::softBody.
 */

#include "body.h"
#include "math.h"
#include <cstdint>
#include <vector>

namespace velox {

// ---------------------------------------------------------------------------
// Handle
// ---------------------------------------------------------------------------

/// Stable handle to a soft body owned by a World.
struct SoftBodyId {
    uint64_t value = UINT64_MAX;
    static SoftBodyId make(uint32_t slot, uint32_t generation) {
        SoftBodyId id;
        id.value = (uint64_t(generation) << 32) | slot;
        return id;
    }
    uint32_t slot() const { return uint32_t(value); }
    uint32_t generation() const { return uint32_t(value >> 32); }
};

inline bool operator==(SoftBodyId a, SoftBodyId b) { return a.value == b.value; }
inline bool operator!=(SoftBodyId a, SoftBodyId b) { return !(a == b); }

// ---------------------------------------------------------------------------
// Constraint
// ---------------------------------------------------------------------------

/// XPBD distance constraint between two particles.
struct SoftDistanceConstraint {
    uint32_t a = 0;          ///< First particle index.
    uint32_t b = 0;          ///< Second particle index.
    float restLength = 0.0f; ///< Rest length.
    float compliance = 0.0f; ///< XPBD compliance (inverse stiffness, s²/kg·m).
};

// ---------------------------------------------------------------------------
// Soft body state
// ---------------------------------------------------------------------------

/// Runtime state of a soft body.
struct SoftBody {
    std::vector<Vec3> positions;     ///< Current particle positions.
    std::vector<Vec3> prevPositions; ///< Positions at the start of the step.
    std::vector<Vec3> velocities;    ///< Particle velocities.
    std::vector<float> invMasses;    ///< Per-particle inverse mass (0 = pinned).
    std::vector<SoftDistanceConstraint> constraints;

    float gravityScale = 1.0f;       ///< Scales world gravity for this body.
    float linearDamping = 0.05f;     ///< Velocity damping per step.
    int solverIterations = 8;        ///< XPBD constraint iterations per substep.

    Vec3 aabbMin{0, 0, 0};          ///< Axis-aligned bounding box (updated each step).
    Vec3 aabbMax{0, 0, 0};

    size_t particleCount() const { return positions.size(); }
    size_t constraintCount() const { return constraints.size(); }
};

// ---------------------------------------------------------------------------
// Creation descriptor
// ---------------------------------------------------------------------------

/// Describes a soft body to be added to a World.
struct SoftBodyDesc {
    std::vector<Vec3> positions;     ///< Initial particle positions.
    std::vector<float> invMasses;    ///< Per-particle inverse mass.
    std::vector<SoftDistanceConstraint> constraints;

    float gravityScale = 1.0f;
    float linearDamping = 0.05f;
    int solverIterations = 8;
};

// ---------------------------------------------------------------------------
// Factory helpers
// ---------------------------------------------------------------------------

/**
 * @brief Create a rectangular cloth grid.
 *
 * Particles are laid out in the XZ plane at y=0, with structural
 * (horizontal + vertical) and shear (diagonal) distance constraints.
 *
 * @param cols      Number of columns (X direction), >= 2.
 * @param rows      Number of rows (Z direction), >= 2.
 * @param spacing   Distance between adjacent particles.
 * @param invMass   Inverse mass per particle (0 pins the particle).
 * @param compliance Constraint compliance (0 = rigid, higher = softer).
 * @param pinCorners If true, the four corner particles are pinned (invMass=0).
 */
SoftBodyDesc makeClothSoftBody(int cols, int rows, float spacing,
                               float invMass = 1.0f,
                               float compliance = 1e-5f,
                               bool pinCorners = false);

/**
 * @brief Create a deformable sphere.
 *
 * Surface particles are placed on a sphere using a Fibonacci lattice;
 * a central core particle is connected to every surface particle to
 * provide volume preservation.
 *
 * @param radius       Sphere radius.
 * @param surfaceCount Number of surface particles (>= 12).
 * @param invMass      Inverse mass per particle.
 * @param compliance   Constraint compliance.
 */
SoftBodyDesc makeSoftSphereSoftBody(float radius, int surfaceCount = 42,
                                    float invMass = 1.0f,
                                    float compliance = 1e-5f);

// ---------------------------------------------------------------------------
// Internal solver (called by World::stepImpl)
// ---------------------------------------------------------------------------

namespace softbody_detail {

/// Advance one soft body by dt using XPBD, colliding against static rigid bodies.
void stepSoftBody(SoftBody& sb, const Vec3& gravity, float dt,
                  const std::vector<Body>& bodies);

} // namespace softbody_detail

} // namespace velox
