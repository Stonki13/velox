# 14 — Joint Completeness

## Goal

Extend Velox's joint system with soft constraints (compliance), position drives on all joint types, and new gear/pulley constraint types. Every existing joint type (ball, distance, spring, hinge, cone-twist, fixed, prismatic, 6DoF) gains optional position-drive capability; new joint types enable mechanical linkages between rotating/translating bodies.

## Public API

```cpp
namespace velox {

// Soft constraint compliance: allows a joint to "stretch" slightly under load,
// simulating elastic materials (rubber mounts, loose connections).
struct JointCompliance {
    float linearCompliance = 0.0f;      // m/N: linear stretch per Newton
    float angularCompliance = 0.0f;     // rad/N*m: angular twist per N*m
    bool enabled = false;
};

// Position drive: targets a specific joint coordinate (angle, translation, etc.)
// and applies impulses to reach it within a timestep.
struct JointPositionDrive {
    float target = 0.0f;                // target position
    float stiffness = 100.0f;           // spring stiffness (N*m/rad or N/m)
    float damping = 10.0f;              // damping coefficient
    float maxForce = 1e10f;             // force/torque budget cap
    bool enabled = false;
};

// Gear constraint: links two rotating bodies via a gear ratio.
struct GearConstraint {
    JointId jointA, jointB;             // hinge or prismatic joints to link
    float ratio = 1.0f;                 // angular/translation ratio (A:B)
    bool enableBacklash = false;        // add play in the gear teeth
    float backlash = 0.0f;              // radians/meters of play
};

// Pulley constraint: links two bodies via a pulley ratio (inverse of gear).
struct PulleyConstraint {
    BodyId bodyA, bodyB;                // bodies attached to pulley ends
    Vec3 anchorA, anchorB;              // pulley anchor points (static)
    float ratio = 1.0f;                 // length ratio (A:B)
    float maxLengthA = 10.0f;           // max extended length for A
    float maxLengthB = 10.0f;           // max extended length for B
};

// Additions to Joint struct (in include/velox/joint.h):
//   JointCompliance compliance;
//   JointPositionDrive positionDrive;
//   bool enableGear = false; GearConstraint gear;
//   bool enablePulley = false; PulleyConstraint pulley;

} // namespace velox
```

## Data structures

- `JointCompliance` — added to `include/velox/joint.h`.
- `JointPositionDrive` — added to `include/velox/joint.h`.
- `GearConstraint`, `PulleyConstraint` — new structs in `include/velox/joint.h`.
- New joint types `JointType::Gear`, `JointType::Pulley` — added to enum.

## Algorithm

**Position drive (on all joint types):**

1. Compute the current joint coordinate error: `error = target - currentValue` (angle for hinge, translation for prismatic, etc.).
2. Apply a spring-damper impulse: `impulse = stiffness * error + damping * velocityError`.
3. Clamp impulse to `maxForce * dt`.
4. Apply the impulse to both bodies via existing solver infrastructure.

**Compliance:**

1. During velocity solve, add a parallel spring element to each joint constraint.
2. The compliance acts as an additional degree of freedom: `stretch = force * compliance`.
3. This is implemented by modifying the effective mass matrix in `solveContact` to include the compliance term.

**Gear constraint:**

1. Create a virtual joint that enforces `angleA * ratio = angleB` (or `transA * ratio = transB`).
2. Solve as a standard distance constraint on the linked coordinates.
3. Backlash: add a dead zone where no force is transmitted until the gap is closed.

**Pulley constraint:**

1. Enforce `lengthA / lengthB = ratio` via a variable-length distance constraint.
2. Track the total cable length `L = lengthA + ratio * lengthB`; clamp each segment to its max.
3. Solve as two coupled distance constraints with a shared Lagrange multiplier.

## Files

- `include/velox/joint.h` — add compliance, position drive, gear/pulley fields
- `src/joint_solver.cpp` — implement new constraint solving paths
- `tests/joint_compliance.cpp` — test file for soft constraints

## Tests

1. **Position drive on hinge:** Hinge joint drives to 90° with stiffness=200, damping=10. Reaches target within 0.3 seconds, overshoot < 5°.
2. **Compliance stretch:** Distance joint with linearCompliance=0.001 m/N under 100 N load stretches 0.1 m (within tolerance).
3. **Gear ratio:** Two hinges linked 2:1 gear. Rotating one by 90° moves the other by 45°.
4. **Pulley length conservation:** Pulley with ratio=2.0; pulling 1 m on side A extends side B by 0.5 m. Total cable length constant.

## Acceptance

- [ ] Position drive works on hinge, prismatic, and 6DoF linear axes
- [ ] Compliance adds elastic behavior without solver instability
- [ ] Gear constraint enforces correct ratio with optional backlash
- [ ] Pulley constraint conserves total cable length

## Size: M

## Risks

- Position drives with high stiffness can destabilize the solver if the timestep is too large. Must document the stability condition: `stiffness * dt² / mass < 4`.
- Gear/pulley constraints create coupled systems that may require block-solving (item 15). Implementing them as independent distance constraints may cause drift in closed loops.
