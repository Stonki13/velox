#include "diff_test.h"

// Canonical differential scenes. Each is deterministic and engine-agnostic;
// see docs/roadmap/08-differential-testing.md. Ground planes are modeled as
// large static boxes so Velox and Jolt see identical geometry.

namespace difftest {
namespace {

BodyDesc ground() {
    BodyDesc body;
    body.shape = BodyDesc::Shape::GroundBox;
    body.halfExtents = {100.0f, 1.0f, 100.0f};
    body.position = {0.0f, -1.0f, 0.0f}; // top surface at y = 0
    body.mass = 0.0f;
    body.friction = 0.8f;
    body.restitution = 0.0f;
    return body;
}

// 1. Sphere drop with restitution: simple, non-chaotic bounce trajectory.
SceneDesc sphereDrop() {
    SceneDesc scene;
    scene.name = "sphere_drop";
    scene.frames = 240;
    // Both bodies share the restitution value so Velox (min combine) and
    // Jolt (max combine) resolve to the same effective bounce coefficient.
    BodyDesc floor = ground();
    floor.restitution = 0.6f;
    scene.bodies.push_back(floor);
    BodyDesc ball;
    ball.shape = BodyDesc::Shape::Sphere;
    ball.position = {0.0f, 3.0f, 0.0f};
    ball.radius = 0.5f;
    ball.mass = 1.0f;
    ball.restitution = 0.6f;
    ball.friction = 0.4f;
    scene.bodies.push_back(ball);
    scene.tracked = {1};
    return scene;
}

// 2. Ten-box stack: resting stability. Chaotic in detail, so the comparison
// relies on behavioral bounds (both towers must stand) plus energy decay.
SceneDesc boxStack() {
    SceneDesc scene;
    scene.name = "box_stack";
    scene.frames = 300;
    scene.substeps = 8;
    scene.bodies.push_back(ground());
    for (int i = 0; i < 10; ++i) {
        BodyDesc box;
        box.shape = BodyDesc::Shape::Box;
        box.halfExtents = {0.5f, 0.5f, 0.5f};
        box.position = {0.0f, 0.5f + 1.0f * static_cast<float>(i), 0.0f};
        box.mass = 1.0f;
        box.friction = 0.8f;
        box.restitution = 0.0f;
        scene.bodies.push_back(box);
        scene.tracked.push_back(i + 1);
    }
    return scene;
}

// 3. Pendulum: point-jointed bob swinging from a static anchor. Tests joint
// solving and energy behavior over several periods.
SceneDesc pendulum() {
    SceneDesc scene;
    scene.name = "pendulum";
    scene.frames = 360;
    BodyDesc anchor;
    anchor.shape = BodyDesc::Shape::Box;
    anchor.halfExtents = {0.1f, 0.1f, 0.1f};
    anchor.position = {0.0f, 5.0f, 0.0f};
    anchor.mass = 0.0f;
    scene.bodies.push_back(anchor);
    BodyDesc bob;
    bob.shape = BodyDesc::Shape::Sphere;
    bob.radius = 0.25f;
    bob.position = {2.0f, 5.0f, 0.0f}; // horizontal release, 2 m arm
    bob.mass = 2.0f;
    bob.friction = 0.0f;
    bob.restitution = 0.0f;
    scene.bodies.push_back(bob);
    JointDesc joint;
    joint.bodyA = 0;
    joint.bodyB = 1;
    joint.worldAnchor = {0.0f, 5.0f, 0.0f};
    scene.joints.push_back(joint);
    scene.tracked = {1};
    return scene;
}

// 4. Rolling sphere with friction: a sphere launched horizontally must slow
// and roll a comparable distance in both engines.
SceneDesc sphereRoll() {
    SceneDesc scene;
    scene.name = "sphere_roll";
    scene.frames = 300;
    scene.bodies.push_back(ground());
    BodyDesc ball;
    ball.shape = BodyDesc::Shape::Sphere;
    ball.radius = 0.5f;
    ball.position = {-8.0f, 0.5f, 0.0f};
    ball.mass = 1.0f;
    ball.friction = 0.5f;
    ball.restitution = 0.0f;
    ball.initialVelocity = {6.0f, 0.0f, 0.0f};
    scene.bodies.push_back(ball);
    scene.tracked = {1};
    return scene;
}

// 5. High-speed sphere vs wall: continuous collision. Both engines must stop
// the sphere at the wall instead of tunneling through it.
SceneDesc ccdWall() {
    SceneDesc scene;
    scene.name = "ccd_wall";
    scene.frames = 120;
    scene.bodies.push_back(ground());
    BodyDesc wall;
    wall.shape = BodyDesc::Shape::Box;
    wall.halfExtents = {0.25f, 4.0f, 4.0f};
    wall.position = {10.0f, 4.0f, 0.0f};
    wall.mass = 0.0f;
    wall.friction = 0.5f;
    // Match the bullet so Velox (min combine) and Jolt (max combine) agree.
    wall.restitution = 0.1f;
    scene.bodies.push_back(wall);
    BodyDesc bullet;
    bullet.shape = BodyDesc::Shape::Sphere;
    bullet.radius = 0.2f;
    bullet.position = {-20.0f, 4.0f, 0.0f};
    bullet.mass = 0.5f;
    bullet.restitution = 0.1f;
    bullet.friction = 0.3f;
    bullet.initialVelocity = {150.0f, 0.0f, 0.0f};
    bullet.highSpeedCcd = true;
    scene.bodies.push_back(bullet);
    scene.tracked = {2};
    return scene;
}

// 6. Gyroscopic precession: a box spinning about its minor inertia axis with
// a transverse component precesses regularly (non-chaotic, unlike the
// intermediate-axis tumble, so cross-engine comparison stays meaningful).
// No gravity, no contacts: pure rotational dynamics.
SceneDesc gyroSpin() {
    SceneDesc scene;
    scene.name = "gyro_spin";
    scene.gravity = {0.0f, 0.0f, 0.0f};
    scene.frames = 600;
    BodyDesc box;
    box.shape = BodyDesc::Shape::Box;
    box.halfExtents = {0.1f, 0.4f, 0.05f};
    box.position = {0.0f, 2.0f, 0.0f};
    box.mass = 1.0f;
    box.initialAngularVelocity = {0.5f, 8.0f, 0.0f}; // minor axis + transverse
    box.gyroscopic = true;
    scene.bodies.push_back(box);
    scene.tracked = {0};
    return scene;
}

// 7. Offset/leaning stack: each box shifted from the one below by a small
// horizontal offset -- harder than the centered box_stack because friction
// must resist an active toppling torque, not just hold a balanced load.
SceneDesc leaningStack() {
    SceneDesc scene;
    scene.name = "leaning_stack";
    scene.frames = 300;
    scene.substeps = 8;
    scene.bodies.push_back(ground());
    constexpr float offsetPerBox = 0.06f; // < half box width, so it settles rather than falls
    for (int i = 0; i < 8; ++i) {
        BodyDesc box;
        box.shape = BodyDesc::Shape::Box;
        box.halfExtents = {0.5f, 0.5f, 0.5f};
        box.position = {offsetPerBox * static_cast<float>(i),
                        0.5f + 1.0f * static_cast<float>(i), 0.0f};
        box.mass = 1.0f;
        box.friction = 0.9f;
        box.restitution = 0.0f;
        scene.bodies.push_back(box);
        scene.tracked.push_back(i + 1);
    }
    return scene;
}

// 8. Hinge motor: a paddle driven at a constant target angular velocity
// about a vertical axis with no gravity torque about that axis, isolating
// the motor solver (Velox's Joint::enableMotor/motorSpeed/maxMotorTorque vs
// Jolt's HingeConstraint EMotorState::Velocity) from gravity/contact noise.
SceneDesc hingeMotor() {
    SceneDesc scene;
    scene.name = "hinge_motor";
    scene.gravity = {0.0f, 0.0f, 0.0f};
    scene.frames = 60;
    // Offset away from the paddle's swept volume (paddle spans x=[0,2]) so
    // the anchor's own collision box never overlaps and fights the motor
    // with contact forces -- the joint pivot (worldAnchor below) does not
    // need to coincide with the anchor body's own geometry.
    BodyDesc anchor;
    anchor.shape = BodyDesc::Shape::Box;
    anchor.halfExtents = {0.1f, 0.1f, 0.1f};
    anchor.position = {-0.5f, 0.0f, 0.0f};
    anchor.mass = 0.0f;
    scene.bodies.push_back(anchor);
    BodyDesc paddle;
    paddle.shape = BodyDesc::Shape::Box;
    paddle.halfExtents = {1.0f, 0.1f, 0.1f};
    paddle.position = {1.0f, 0.0f, 0.0f};
    paddle.mass = 1.0f;
    paddle.allowSleep = false; // must keep spinning at the motor's target for the whole run
    scene.bodies.push_back(paddle);
    JointDesc joint;
    joint.type = JointDesc::Type::HingeMotor;
    joint.bodyA = 0;
    joint.bodyB = 1;
    joint.worldAnchor = {0.0f, 0.0f, 0.0f};
    joint.worldAxis = {0.0f, 1.0f, 0.0f};
    joint.motorSpeed = 2.0f;       // rad/s target
    joint.maxMotorTorque = 5000.0f; // ample budget to reach target quickly
    scene.joints.push_back(joint);
    scene.tracked = {1};
    return scene;
}

// 9. Terrain/mesh: a bumpy static triangle mesh (matches the shape used by
// benchmarks/benchmark_scenes.h::meshTerrain) with a sphere dropped onto it,
// exercising each engine's mesh/BVH narrow phase rather than analytic shapes.
SceneDesc terrainMesh() {
    SceneDesc scene;
    scene.name = "terrain_mesh";
    scene.frames = 240;
    BodyDesc terrain;
    terrain.shape = BodyDesc::Shape::Mesh;
    terrain.mass = 0.0f;
    terrain.friction = 0.6f;
    terrain.restitution = 0.0f;
    constexpr int grid = 12;
    constexpr float size = 20.0f, half = size * 0.5f;
    for (int z = 0; z <= grid; ++z)
        for (int x = 0; x <= grid; ++x) {
            float fx = static_cast<float>(x) * size / grid - half;
            float fz = static_cast<float>(z) * size / grid - half;
            float fy = std::sin(fx * 0.4f) * std::cos(fz * 0.4f) * 1.0f;
            terrain.meshVertices.push_back({fx, fy, fz});
        }
    for (int z = 0; z < grid; ++z)
        for (int x = 0; x < grid; ++x) {
            uint32_t a = static_cast<uint32_t>(z * (grid + 1) + x), b = a + 1,
                     c = a + grid + 1, d = c + 1;
            terrain.meshIndices.insert(terrain.meshIndices.end(), {a, c, b, b, c, d});
        }
    scene.bodies.push_back(terrain);
    BodyDesc ball;
    ball.shape = BodyDesc::Shape::Sphere;
    ball.radius = 0.5f;
    ball.position = {0.0f, 4.0f, 0.0f};
    ball.mass = 1.0f;
    ball.friction = 0.6f;
    ball.restitution = 0.1f;
    scene.bodies.push_back(ball);
    scene.tracked = {1};
    return scene;
}

// 10. High-speed CCD, grazing: the bullet crosses the wall at a shallow
// angle (mostly-tangential velocity with a small closing component) rather
// than head-on -- a harder case for continuous collision because the swept
// volume barely overlaps the wall for a much shorter window than ccd_wall's
// perpendicular impact.
SceneDesc ccdGrazing() {
    SceneDesc scene;
    scene.name = "ccd_grazing";
    scene.frames = 120;
    scene.bodies.push_back(ground());
    BodyDesc wall;
    wall.shape = BodyDesc::Shape::Box;
    wall.halfExtents = {0.15f, 4.0f, 6.0f};
    wall.position = {10.0f, 4.0f, 0.0f};
    wall.mass = 0.0f;
    wall.friction = 0.5f;
    wall.restitution = 0.1f;
    scene.bodies.push_back(wall);
    BodyDesc bullet;
    bullet.shape = BodyDesc::Shape::Sphere;
    bullet.radius = 0.15f;
    bullet.position = {-5.0f, 4.0f, -3.0f};
    bullet.mass = 0.3f;
    bullet.restitution = 0.1f;
    bullet.friction = 0.3f;
    // A meaningful tangential (z) component relative to the closing (x)
    // component (~23 degrees off the wall normal) while staying within the
    // wall's z-span [-6, 6] for the whole x-crossing -- shallower than
    // ccd_wall's head-on impact, but still an actual hit rather than a miss.
    bullet.initialVelocity = {60.0f, 0.0f, 25.0f};
    bullet.highSpeedCcd = true;
    scene.bodies.push_back(bullet);
    scene.tracked = {2};
    return scene;
}

// 11. Sleep/wake: a box settles to rest on the ground (should go to sleep
// once velocity decays) then a second box is dropped onto it later in the
// run (should wake it). Checked behaviorally (position/velocity pattern
// over the settle/impact windows), not via an engine-specific sleep flag,
// so the same scene and check work unmodified for both engines.
SceneDesc sleepWake() {
    SceneDesc scene;
    scene.name = "sleep_wake";
    scene.frames = 240;
    scene.bodies.push_back(ground());
    BodyDesc resting;
    resting.shape = BodyDesc::Shape::Box;
    resting.halfExtents = {0.5f, 0.5f, 0.5f};
    resting.position = {0.0f, 0.55f, 0.0f}; // starts just above the floor, settles fast
    resting.mass = 1.0f;
    resting.friction = 0.8f;
    resting.restitution = 0.0f;
    scene.bodies.push_back(resting);
    BodyDesc dropped;
    dropped.shape = BodyDesc::Shape::Box;
    dropped.halfExtents = {0.5f, 0.5f, 0.5f};
    // Released well above so it lands on `resting` partway through the run,
    // long after `resting` has settled.
    dropped.position = {0.0f, 8.0f, 0.0f};
    dropped.mass = 1.0f;
    dropped.friction = 0.8f;
    dropped.restitution = 0.0f;
    scene.bodies.push_back(dropped);
    scene.tracked = {1, 2};
    return scene;
}

} // namespace

std::vector<SceneDesc> canonicalScenes() {
    return {sphereDrop(),   boxStack(),  pendulum(),   sphereRoll(),
            ccdWall(),      gyroSpin(),  leaningStack(), hingeMotor(),
            terrainMesh(),  ccdGrazing(), sleepWake()};
}

std::vector<CharacterSceneDesc> characterScenes() {
    CharacterSceneDesc flat;
    flat.name = "character_flat_walk";
    flat.slopeAngleDeg = 0.0f;
    flat.targetVelocity = {0, 0, 3};
    flat.frames = 120;

    CharacterSceneDesc gentle;
    gentle.name = "character_gentle_slope";
    gentle.slopeAngleDeg = 20.0f; // below the 45-degree slope limit
    gentle.targetVelocity = {0, 0, 3};
    gentle.frames = 120;

    CharacterSceneDesc steep;
    steep.name = "character_steep_slope";
    steep.slopeAngleDeg = 55.0f; // above the 45-degree slope limit
    steep.targetVelocity = {0, 0, 3};
    steep.frames = 120;

    return {flat, gentle, steep};
}

} // namespace difftest
