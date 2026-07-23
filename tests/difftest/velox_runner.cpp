#include "diff_test.h"

#include <velox/velox.h>

#include <stdexcept>

namespace difftest {
namespace {

velox::Vec3 toVelox(const Vec3f& v) { return {v.x, v.y, v.z}; }
Vec3f fromVelox(const velox::Vec3& v) { return {v.x, v.y, v.z}; }
Quatf fromVelox(const velox::Quat& q) { return {q.x, q.y, q.z, q.w}; }

} // namespace

Trajectory runVelox(const SceneDesc& scene) {
    // Differential runs use the CPU backend: bitwise deterministic and the
    // reference path the CUDA backend is itself validated against.
    velox::World world(velox::BackendType::Cpu);
    world.gravity = toVelox(scene.gravity);
    world.substeps = scene.substeps;

    std::vector<velox::BodyId> ids;
    ids.reserve(scene.bodies.size());
    for (const BodyDesc& desc : scene.bodies) {
        velox::BodyId id;
        switch (desc.shape) {
        case BodyDesc::Shape::Sphere:
            id = world.addSphere(toVelox(desc.position), desc.radius, desc.mass);
            break;
        case BodyDesc::Shape::Capsule:
            id = world.addCapsule(toVelox(desc.position), desc.radius,
                                  desc.capsuleHalfHeight, desc.mass);
            break;
        case BodyDesc::Shape::Box:
        case BodyDesc::Shape::GroundBox:
            id = world.addBox(toVelox(desc.position), toVelox(desc.halfExtents), desc.mass);
            break;
        case BodyDesc::Shape::Mesh: {
            std::vector<velox::Vec3> verts;
            verts.reserve(desc.meshVertices.size());
            for (const Vec3f& v : desc.meshVertices) verts.push_back(toVelox(v));
            id = world.addStaticMesh(verts, desc.meshIndices);
            break;
        }
        default:
            throw std::runtime_error("unsupported shape");
        }
        velox::Body& body = world.body(id);
        body.restitution = desc.restitution;
        body.friction = desc.friction;
        body.enableSleep = desc.allowSleep;
        body.orientation = {desc.orientation.x, desc.orientation.y,
                            desc.orientation.z, desc.orientation.w};
        if (desc.mass > 0.0f) {
            world.setLinearVelocity(id, toVelox(desc.initialVelocity));
            world.setAngularVelocity(id, toVelox(desc.initialAngularVelocity));
        }
        ids.push_back(id);
    }

    for (const JointDesc& joint : scene.joints) {
        if (joint.type == JointDesc::Type::HingeMotor) {
            velox::JointId hinge = world.addHingeJoint(
                ids[static_cast<size_t>(joint.bodyA)], ids[static_cast<size_t>(joint.bodyB)],
                toVelox(joint.worldAnchor), toVelox(joint.worldAxis));
            velox::Joint& j = world.joint(hinge);
            j.enableMotor = true;
            j.motorSpeed = joint.motorSpeed;
            j.maxMotorTorque = joint.maxMotorTorque;
        } else {
            world.addBallJoint(ids[static_cast<size_t>(joint.bodyA)],
                               ids[static_cast<size_t>(joint.bodyB)],
                               toVelox(joint.worldAnchor));
        }
    }

    Trajectory trajectory;
    trajectory.reserve(static_cast<size_t>(scene.frames));
    for (int frame = 0; frame < scene.frames; ++frame) {
        world.step(scene.dt);
        FrameState state;
        state.frame = frame;
        state.time = scene.dt * static_cast<float>(frame + 1);
        state.bodies.reserve(scene.tracked.size());
        for (int index : scene.tracked) {
            const velox::Body& body = world.body(ids[static_cast<size_t>(index)]);
            BodySample sample;
            sample.position = fromVelox(body.position);
            sample.orientation = fromVelox(body.orientation);
            sample.velocity = fromVelox(body.velocity);
            sample.angularVelocity = fromVelox(body.angularVelocity);
            state.bodies.push_back(sample);
        }
        trajectory.push_back(std::move(state));
    }
    return trajectory;
}

CharacterResult runVeloxCharacter(const CharacterSceneDesc& scene) {
    velox::World world(velox::BackendType::Cpu);

    // Build a slope: a large static box rotated by slopeAngleDeg around X.
    float angleRad = scene.slopeAngleDeg * 3.14159265f / 180.0f;
    auto slope = world.addBox({0, -0.5f, 0}, {25.0f, 0.5f, 25.0f}, 0.0f);
    velox::Body& slopeBody = world.body(slope);
    slopeBody.orientation = velox::fromAxisAngle({1, 0, 0}, -angleRad);
    slopeBody.friction = 0.8f;

    velox::CharacterControllerDesc desc;
    desc.capsuleRadius = scene.capsuleRadius;
    desc.capsuleHalfHeight = scene.capsuleHalfHeight;
    desc.slopeLimitCosine = scene.slopeLimitCosine;
    desc.movementSpeed = velox::length(toVelox(scene.targetVelocity));

    velox::CharacterController controller(world, desc);
    // Start at the low end of the slope.
    float startY = scene.capsuleHalfHeight + scene.capsuleRadius + 0.5f;
    controller.SetPosition({0, startY, -5.0f});

    velox::Vec3 vel = toVelox(scene.targetVelocity);
    float verticalVel = 0.0f;
    CharacterResult result;
    for (int i = 0; i < scene.frames; ++i) {
        world.step(scene.dt);
        // Apply gravity to the vertical velocity, then combine with
        // the desired horizontal movement.
        verticalVel -= 9.81f * scene.dt;
        velox::Vec3 displacement = vel * scene.dt;
        displacement.y = verticalVel * scene.dt;
        auto moveResult = controller.Move(displacement);
        if (moveResult.grounded) verticalVel = 0.0f;
        result.grounded = moveResult.grounded;
    }
    velox::Vec3 finalPos = controller.Position();
    result.finalPosition = fromVelox(finalPos);
    result.heightGained = finalPos.y - startY;
    result.horizontalDistance = std::sqrt(
        finalPos.x * finalPos.x + (finalPos.z + 5.0f) * (finalPos.z + 5.0f));
    return result;
}

} // namespace difftest
