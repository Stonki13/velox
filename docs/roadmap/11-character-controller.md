# 11 — Character Controller

## Goal

Provide a capsule-based character controller that handles sweep-and-slide movement, step climbing up to a configurable height, slope limiting, and grounded state detection. The controller queries the World for collisions but owns no Body — it is a pure computation layer on top of Velox's existing shape casts and overlaps.

## Public API

```cpp
namespace velox {

struct CharacterControllerDesc {
    float capsuleRadius = 0.3f;
    float capsuleHalfHeight = 0.9f;
    float stepMaxHeight = 0.3f;
    float slopeLimitCosine = 0.7071f;   // cos(45°)
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

// Query-based character controller. Does not own a Body; the user manages
// position/velocity and calls Move() each frame with a desired displacement.
class CharacterController {
public:
    explicit CharacterController(World& world, const CharacterControllerDesc& desc);

    // Move the character by the given displacement. Returns resolved result.
    // Uses up to 4 sweep-and-slide iterations to handle cascading wall contacts.
    CharacterControllerResult Move(Vec3 displacement);

    // Jump: set upward velocity on next move if grounded.
    void SetJumpVelocity(float v) { jumpVel_ = v; }

private:
    World& world_;
    CharacterControllerDesc desc_;
    float jumpVel_ = 0.0f;
};

} // namespace velox
```

## Data structures

- `CharacterControllerDesc` — new file `include/velox/character.h`. Configuration.
- `CharacterControllerResult` — new file `include/velox/character.h`. Per-frame output.
- `CharacterController` class — new file `include/velox/character.h`, impl in `src/character.cpp`.

## Algorithm

**Sweep-and-slide (4 iterations max):**

1. Cast the capsule downward to detect ground: raycast from capsule bottom along -Y, record hit normal and distance.
2. If ground detected and slope angle < limit, mark grounded; compute slide velocity by projecting desired displacement onto the ground plane.
3. For each of up to 4 iterations: cast the capsule forward along the residual displacement. If a wall hit is found, reflect the velocity tangent to the wall normal and continue with the remaining displacement.
4. Step climbing: if the capsule's bottom would pass through a step face (vertical obstacle within stepMaxHeight), lift the capsule by stepMaxHeight and re-cast from the elevated position.

## Files

- `include/velox/character.h` — new header
- `src/character.cpp` — new source file

## Tests

1. **Flat ground walk:** Character moves 10 m on a plane; final position = start + displacement, grounded=true throughout.
2. **Step climbing:** 0.25 m step in path; character climbs it without stopping, grounded=true after stepping.
3. **Wall slide:** Character walks into a wall at 45°; slides along the wall rather than stopping dead.
4. **Slope limit:** 50° slope with cos-limit=0.707 (45°); character slips backward, does not stand.

## Acceptance

- [ ] Move() returns correct final position after wall/slope/step resolution
- [ ] Grounded flag is true only when capsule bottom contacts ground within ghostPadding
- [ ] Step climbing handles steps up to stepMaxHeight without tunneling
- [ ] Slope limit prevents standing on angles steeper than configured

## Size: S

## Risks

- Ghost padding too small causes jitter on flat ground; too large causes false ground detection on slopes. Must tune per game.
- 4 iteration cap may not resolve all cascading contacts in complex geometries (e.g., corner between two walls). Document as a known limitation.
