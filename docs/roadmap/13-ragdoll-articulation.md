# 13 — Ragdoll Articulation

## Goal

Provide authoring helpers that simplify building ragdolls from Velox's existing joint types (ball, hinge, cone-twist), and lay the groundwork for reduced-coordinate articulation (item 13 in design/articulation.h) as a future optimization. The immediate deliverable is a builder API; the full reduced-coordinate solver is deferred.

## Public API

```cpp
namespace velox {

struct RagdollBone {
    BodyId body;
    Vec3 localCenterOfMass{};
    float mass = 1.0f;
};

struct RagdollJoint {
    BodyId parent, child;
    Vec3 anchor{};
    Vec3 axis{0, 1, 0};
    float swingLimit = 1.57f;
    float twistLimit = 0.5f;
    bool enableMotor = false;
    float motorSpeed = 0.0f;
    float maxMotorTorque = 50.0f;
};

class RagdollBuilder {
public:
    // Build a ragdoll from bone/joint descriptions. Creates bodies and joints
    // in the world, returns a handle for later reference.
    static BodyId Build(World& world, const std::vector<RagdollBone>& bones,
                        const std::vector<RagdollJoint>& joints);

    // Apply motor torque to a specific joint by index.
    static void SetMotorTorque(World& world, JointId joint, float torque);

    // Wake all ragdoll bodies (useful after loading from save).
    static void WakeAll(World& world, BodyId ragdollRoot);
};

} // namespace velox
```

## Data structures

- `RagdollBone`, `RagdollJoint` — new file `include/velox/ragdoll.h`.
- `RagdollBuilder` — new file `include/velox/ragdoll.h`, impl in `src/ragdoll.cpp`.

## Algorithm

1. **Authoring:** User defines bones (bodies) and joints (connections). Builder validates the graph is a tree, then creates bodies with appropriate masses and inertias, and adds joints between parent/child pairs using Velox's existing joint types (cone-twist for shoulder/hip, hinge for elbow/knee).
2. **Motor application:** Motors are applied via existing `Joint::enableMotor` / `motorSpeed` / `maxMotorTorque` fields. No new solver path needed.
3. **Reduced-coordinate future work:** When item 13's articulation system is implemented, the ragdoll builder can optionally construct an Articulation object instead of individual joints, gaining better stability for complex chains.

## Files

- `include/velox/ragdoll.h` — new header
- `src/ragdoll.cpp` — new source file

## Tests

1. **Standing ragdoll:** 8-bone human ragdoll (head, torso, 2 upper arms, 2 forearms, 2 thighs) standing on flat ground. Remains upright for 5 seconds without falling over (initial pose: T-pose).
2. **Falling ragdoll:** Same ragdoll pushed sideways. Falls realistically with limb separation, no interpenetration between bones.
3. **Motor control:** Apply 10 N*m motor to elbow hinge. Forearm rotates toward target angle within 0.5 seconds.

## Acceptance

- [ ] RagdollBuilder creates valid body+joint configuration from bone/joint descriptions
- [ ] Graph validation rejects cycles and disconnected components
- [ ] Existing joint solver handles ragdoll constraints without instability
- [ ] Motors drive joints to target angles as configured

## Size: S

## Risks

- Cone-twist joints have known stability issues at extreme limits; ragdolls with wide swing limits may exhibit jitter. Must document recommended limit ranges.
- Reduced-coordinate articulation (deferred) would significantly improve stability but requires a new solver path. The authoring helpers must not assume this optimization is available.
