#include "diff_test.h"

// Jolt execution of the shared scene descriptors. Boilerplate follows Jolt's
// HelloWorld sample: two broad-phase layers (static / moving).

#include <Jolt/Jolt.h>

#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Constraints/PointConstraint.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Character/CharacterVirtual.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace difftest {
namespace {

namespace Layers {
constexpr JPH::ObjectLayer NON_MOVING = 0;
constexpr JPH::ObjectLayer MOVING = 1;
constexpr JPH::ObjectLayer NUM_LAYERS = 2;
} // namespace Layers

namespace BroadPhaseLayers {
constexpr JPH::BroadPhaseLayer NON_MOVING(0);
constexpr JPH::BroadPhaseLayer MOVING(1);
constexpr JPH::uint NUM_LAYERS(2);
} // namespace BroadPhaseLayers

class BPLayerInterfaceImpl final : public JPH::BroadPhaseLayerInterface {
public:
    JPH::uint GetNumBroadPhaseLayers() const override { return BroadPhaseLayers::NUM_LAYERS; }
    JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer layer) const override {
        return layer == Layers::NON_MOVING ? BroadPhaseLayers::NON_MOVING
                                           : BroadPhaseLayers::MOVING;
    }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer layer) const override {
        return layer == BroadPhaseLayers::NON_MOVING ? "NON_MOVING" : "MOVING";
    }
#endif
};

class ObjectVsBroadPhaseLayerFilterImpl final : public JPH::ObjectVsBroadPhaseLayerFilter {
public:
    bool ShouldCollide(JPH::ObjectLayer layer1, JPH::BroadPhaseLayer layer2) const override {
        if (layer1 == Layers::NON_MOVING) return layer2 == BroadPhaseLayers::MOVING;
        return true;
    }
};

class ObjectLayerPairFilterImpl final : public JPH::ObjectLayerPairFilter {
public:
    bool ShouldCollide(JPH::ObjectLayer layer1, JPH::ObjectLayer layer2) const override {
        if (layer1 == Layers::NON_MOVING) return layer2 == Layers::MOVING;
        return true;
    }
};

struct JoltRuntime {
    JoltRuntime() {
        JPH::RegisterDefaultAllocator();
        JPH::Factory::sInstance = new JPH::Factory();
        JPH::RegisterTypes();
    }
    ~JoltRuntime() {
        JPH::UnregisterTypes();
        delete JPH::Factory::sInstance;
        JPH::Factory::sInstance = nullptr;
    }
};

JPH::Vec3 toJolt(const Vec3f& v) { return {v.x, v.y, v.z}; }
Vec3f fromJolt(JPH::Vec3Arg v) { return {v.GetX(), v.GetY(), v.GetZ()}; }
Quatf fromJolt(JPH::QuatArg q) { return {q.GetX(), q.GetY(), q.GetZ(), q.GetW()}; }

void ensureJoltInit() {
    static JoltRuntime runtime; // one-time global Jolt registration
}

} // namespace

Trajectory runJolt(const SceneDesc& scene) {
    ensureJoltInit();

    BPLayerInterfaceImpl broadPhaseLayers;
    ObjectVsBroadPhaseLayerFilterImpl objectVsBroadPhase;
    ObjectLayerPairFilterImpl objectPairs;

    JPH::PhysicsSystem physics;
    physics.Init(4096, 0, 4096, 4096, broadPhaseLayers, objectVsBroadPhase, objectPairs);
    physics.SetGravity(toJolt(scene.gravity));

    JPH::TempAllocatorImpl tempAllocator(16 * 1024 * 1024);
    JPH::JobSystemThreadPool jobSystem(JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers, 1);

    JPH::BodyInterface& bodies = physics.GetBodyInterface();
    std::vector<JPH::Body*> created;
    created.reserve(scene.bodies.size());

    for (const BodyDesc& desc : scene.bodies) {
        JPH::RefConst<JPH::Shape> shape;
        switch (desc.shape) {
        case BodyDesc::Shape::Sphere:
            shape = new JPH::SphereShape(desc.radius);
            break;
        case BodyDesc::Shape::Capsule:
            shape = new JPH::CapsuleShape(desc.capsuleHalfHeight, desc.radius);
            break;
        case BodyDesc::Shape::Box:
        case BodyDesc::Shape::GroundBox:
            shape = new JPH::BoxShape(toJolt(desc.halfExtents));
            break;
        case BodyDesc::Shape::Mesh: {
            JPH::VertexList verts;
            verts.reserve(desc.meshVertices.size());
            for (const Vec3f& v : desc.meshVertices)
                verts.push_back(JPH::Float3(v.x, v.y, v.z));
            JPH::IndexedTriangleList tris;
            tris.reserve(desc.meshIndices.size() / 3);
            for (size_t i = 0; i + 2 < desc.meshIndices.size(); i += 3)
                tris.push_back(JPH::IndexedTriangle(desc.meshIndices[i],
                                                    desc.meshIndices[i + 1],
                                                    desc.meshIndices[i + 2]));
            JPH::MeshShapeSettings meshSettings(verts, tris);
            JPH::Shape::ShapeResult result = meshSettings.Create();
            if (result.HasError())
                throw std::runtime_error("Jolt mesh shape creation failed: " +
                                         std::string(result.GetError().c_str()));
            shape = result.Get();
            break;
        }
        default:
            throw std::runtime_error("unsupported shape");
        }

        const bool isStatic = desc.mass <= 0.0f;
        JPH::BodyCreationSettings settings(
            shape, JPH::RVec3(desc.position.x, desc.position.y, desc.position.z),
            JPH::Quat(desc.orientation.x, desc.orientation.y, desc.orientation.z,
                      desc.orientation.w),
            isStatic ? JPH::EMotionType::Static : JPH::EMotionType::Dynamic,
            isStatic ? Layers::NON_MOVING : Layers::MOVING);
        settings.mRestitution = desc.restitution;
        settings.mFriction = desc.friction;
        // Match Velox: no implicit velocity damping (Jolt defaults to 0.05).
        settings.mLinearDamping = 0.0f;
        settings.mAngularDamping = 0.0f;
        settings.mAllowSleeping = desc.allowSleep;
        if (!isStatic) {
            settings.mOverrideMassProperties =
                JPH::EOverrideMassProperties::CalculateInertia;
            settings.mMassPropertiesOverride.mMass = desc.mass;
            settings.mLinearVelocity = toJolt(desc.initialVelocity);
            settings.mAngularVelocity = toJolt(desc.initialAngularVelocity);
            // Velox always integrates gyroscopic precession (momentum-
            // preserving orientation advance); Jolt makes it opt-in.
            settings.mApplyGyroscopicForce = desc.gyroscopic;
            if (desc.gyroscopic) settings.mMaxAngularVelocity = 100.0f;
            if (desc.highSpeedCcd)
                settings.mMotionQuality = JPH::EMotionQuality::LinearCast;
        }

        JPH::Body* body = bodies.CreateBody(settings);
        if (!body) throw std::runtime_error("Jolt body creation failed");
        bodies.AddBody(body->GetID(), JPH::EActivation::Activate);
        created.push_back(body);
    }

    for (const JointDesc& joint : scene.joints) {
        JPH::Body& bodyA = *created[static_cast<size_t>(joint.bodyA)];
        JPH::Body& bodyB = *created[static_cast<size_t>(joint.bodyB)];
        if (joint.type == JointDesc::Type::HingeMotor) {
            JPH::HingeConstraintSettings settings;
            settings.mSpace = JPH::EConstraintSpace::WorldSpace;
            settings.mPoint1 = settings.mPoint2 =
                JPH::RVec3(joint.worldAnchor.x, joint.worldAnchor.y, joint.worldAnchor.z);
            settings.mHingeAxis1 = settings.mHingeAxis2 = toJolt(joint.worldAxis);
            settings.mNormalAxis1 = settings.mNormalAxis2 =
                settings.mHingeAxis1.GetNormalizedPerpendicular();
            JPH::Constraint* constraint = settings.Create(bodyA, bodyB);
            physics.AddConstraint(constraint);
            auto* hinge = static_cast<JPH::HingeConstraint*>(constraint);
            hinge->SetMotorState(JPH::EMotorState::Velocity);
            // Jolt's HingeConstraint internally negates the target velocity
            // relative to its hinge-axis sign convention (see
            // HingeConstraint.cpp's CalculateConstraintProperties call),
            // opposite to Velox's convention where motorSpeed is the
            // relative angular velocity of B about A directly along
            // worldAxis. Compensate here so both engines spin the same way.
            hinge->SetTargetAngularVelocity(-joint.motorSpeed);
            JPH::MotorSettings& motor = hinge->GetMotorSettings();
            motor.mMaxTorqueLimit = joint.maxMotorTorque;
            motor.mMinTorqueLimit = -joint.maxMotorTorque;
        } else {
            JPH::PointConstraintSettings settings;
            settings.mSpace = JPH::EConstraintSpace::WorldSpace;
            settings.mPoint1 = settings.mPoint2 =
                JPH::RVec3(joint.worldAnchor.x, joint.worldAnchor.y, joint.worldAnchor.z);
            physics.AddConstraint(settings.Create(bodyA, bodyB));
        }
    }

    physics.OptimizeBroadPhase();

    Trajectory trajectory;
    trajectory.reserve(static_cast<size_t>(scene.frames));
    for (int frame = 0; frame < scene.frames; ++frame) {
        physics.Update(scene.dt, scene.substeps, &tempAllocator, &jobSystem);
        FrameState state;
        state.frame = frame;
        state.time = scene.dt * static_cast<float>(frame + 1);
        state.bodies.reserve(scene.tracked.size());
        for (int index : scene.tracked) {
            const JPH::BodyID id = created[static_cast<size_t>(index)]->GetID();
            BodySample sample;
            sample.position = fromJolt(JPH::Vec3(bodies.GetPosition(id)));
            sample.orientation = fromJolt(bodies.GetRotation(id));
            sample.velocity = fromJolt(bodies.GetLinearVelocity(id));
            sample.angularVelocity = fromJolt(bodies.GetAngularVelocity(id));
            state.bodies.push_back(sample);
        }
        trajectory.push_back(std::move(state));
    }
    return trajectory;
}

CharacterResult runJoltCharacter(const CharacterSceneDesc& scene) {
    ensureJoltInit();

    BPLayerInterfaceImpl broadPhaseLayers;
    ObjectVsBroadPhaseLayerFilterImpl objectVsBroadPhase;
    ObjectLayerPairFilterImpl objectPairs;

    JPH::PhysicsSystem physics;
    physics.Init(4096, 0, 4096, 4096, broadPhaseLayers, objectVsBroadPhase, objectPairs);
    physics.SetGravity(JPH::Vec3(0, -9.81f, 0));

    JPH::TempAllocatorImpl tempAllocator(16 * 1024 * 1024);
    JPH::JobSystemThreadPool jobSystem(JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers, 1);

    // Slope: a large static box rotated by slopeAngleDeg around X.
    float angleRad = scene.slopeAngleDeg * 3.14159265f / 180.0f;
    JPH::BodyInterface& bodyInterface = physics.GetBodyInterface();
    JPH::RefConst<JPH::Shape> slopeShape = new JPH::BoxShape(JPH::Vec3(25, 0.5f, 25));
    JPH::Quat slopeRot = JPH::Quat::sRotation(JPH::Vec3(1, 0, 0), -angleRad);
    JPH::BodyCreationSettings slopeSettings(
        slopeShape, JPH::RVec3(0, -0.5f, 0), slopeRot,
        JPH::EMotionType::Static, Layers::NON_MOVING);
    slopeSettings.mFriction = 0.8f;
    bodyInterface.CreateAndAddBody(slopeSettings, JPH::EActivation::DontActivate);

    // Character capsule.
    JPH::RefConst<JPH::Shape> capsuleShape =
        new JPH::CapsuleShape(scene.capsuleHalfHeight, scene.capsuleRadius);

    JPH::CharacterVirtualSettings settings;
    settings.mShape = capsuleShape;
    settings.mMaxSlopeAngle = std::acos(scene.slopeLimitCosine);
    settings.mBackFaceMode = JPH::EBackFaceMode::IgnoreBackFaces;

    float startY = scene.capsuleHalfHeight + scene.capsuleRadius + 0.5f;
    JPH::CharacterVirtual character(&settings,
        JPH::RVec3(0, startY, -5.0f), JPH::Quat::sIdentity(), &physics);

    JPH::Vec3 desiredVel = toJolt(scene.targetVelocity);

    JPH::ObjectLayerFilter objectLayerFilter;
    JPH::BroadPhaseLayerFilter broadPhaseLayerFilter;
    JPH::BodyFilter bodyFilter;
    JPH::ShapeFilter shapeFilter;

    CharacterResult result;
    for (int i = 0; i < scene.frames; ++i) {
        physics.Update(scene.dt, 1, &tempAllocator, &jobSystem);

        character.SetLinearVelocity(desiredVel);
        character.Update(scene.dt, JPH::Vec3(0, -9.81f, 0),
                         broadPhaseLayerFilter, objectLayerFilter,
                         bodyFilter, shapeFilter, tempAllocator);

        result.grounded = character.GetGroundState() ==
                          JPH::CharacterVirtual::EGroundState::OnGround;
    }

    JPH::RVec3 finalPos = character.GetPosition();
    result.finalPosition = {float(finalPos.GetX()), float(finalPos.GetY()),
                            float(finalPos.GetZ())};
    result.heightGained = float(finalPos.GetY()) - startY;
    float dx = float(finalPos.GetX());
    float dz = float(finalPos.GetZ()) + 5.0f;
    result.horizontalDistance = std::sqrt(dx * dx + dz * dz);
    return result;
}

} // namespace difftest
