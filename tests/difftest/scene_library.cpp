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

} // namespace

std::vector<SceneDesc> canonicalScenes() {
    return {sphereDrop(), boxStack(), pendulum(), sphereRoll(), ccdWall(),
            gyroSpin()};
}

} // namespace difftest
