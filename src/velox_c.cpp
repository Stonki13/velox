#include "velox/velox_c.h"
#include "velox/velox.h"
#include <new>

using namespace velox;

// Helper conversions
static Vec3 toVec3(velox_Vec3 v) { return {v.x, v.y, v.z}; }
static velox_Vec3 fromVec3(Vec3 v) { return {v.x, v.y, v.z}; }
static Quat toQuat(velox_Quat q) { return {q.x, q.y, q.z, q.w}; }
static velox_Quat fromQuat(Quat q) { return {q.x, q.y, q.z, q.w}; }
static BodyId toBodyId(velox_BodyId id) { BodyId b; b.value = id; return b; }
static velox_BodyId fromBodyId(BodyId id) { return id.value; }
static JointId toJointId(velox_JointId id) { JointId j; j.value = id; return j; }
static velox_JointId fromJointId(JointId id) { return id.value; }

// World management
velox_World* velox_World_Create(velox_BackendType backend) {
    BackendType b = backend == VELOX_BACKEND_CUDA ? BackendType::Cuda :
                    backend == VELOX_BACKEND_CPU ? BackendType::Cpu : BackendType::Auto;
    try {
        World* w = new World(b);
        return reinterpret_cast<velox_World*>(w);
    } catch (...) {
        return nullptr;
    }
}

void velox_World_Destroy(velox_World* world) {
    delete reinterpret_cast<World*>(world);
}

void velox_World_Step(velox_World* world, float dt) {
    reinterpret_cast<World*>(world)->step(dt);
}

void velox_World_SetGravity(velox_World* world, velox_Vec3 gravity) {
    reinterpret_cast<World*>(world)->gravity = toVec3(gravity);
}

void velox_World_SetSubsteps(velox_World* world, int substeps) {
    reinterpret_cast<World*>(world)->substeps = substeps;
}

// Body creation
velox_BodyId velox_World_AddSphere(velox_World* world, velox_Vec3 position, float radius, float mass) {
    return fromBodyId(reinterpret_cast<World*>(world)->addSphere(toVec3(position), radius, mass));
}

velox_BodyId velox_World_AddBox(velox_World* world, velox_Vec3 position, velox_Vec3 halfExtents, float mass) {
    return fromBodyId(reinterpret_cast<World*>(world)->addBox(toVec3(position), toVec3(halfExtents), mass));
}

velox_BodyId velox_World_AddCapsule(velox_World* world, velox_Vec3 position, float radius, float halfHeight, float mass) {
    return fromBodyId(reinterpret_cast<World*>(world)->addCapsule(toVec3(position), radius, halfHeight, mass));
}

velox_BodyId velox_World_AddCylinder(velox_World* world, velox_Vec3 position, float radius, float halfHeight, float mass) {
    return fromBodyId(reinterpret_cast<World*>(world)->addCylinder(toVec3(position), radius, halfHeight, mass));
}

velox_BodyId velox_World_AddCone(velox_World* world, velox_Vec3 position, float radius, float height, float mass) {
    return fromBodyId(reinterpret_cast<World*>(world)->addCone(toVec3(position), radius, height, mass));
}

velox_BodyId velox_World_AddStaticPlane(velox_World* world, velox_Vec3 normal, float offset) {
    return fromBodyId(reinterpret_cast<World*>(world)->addStaticPlane(toVec3(normal), offset));
}

void velox_World_RemoveBody(velox_World* world, velox_BodyId body) {
    reinterpret_cast<World*>(world)->removeBody(toBodyId(body));
}

// Body property access
velox_Vec3 velox_Body_GetPosition(velox_World* world, velox_BodyId body) {
    return fromVec3(reinterpret_cast<World*>(world)->body(toBodyId(body)).position);
}

velox_Quat velox_Body_GetOrientation(velox_World* world, velox_BodyId body) {
    return fromQuat(reinterpret_cast<World*>(world)->body(toBodyId(body)).orientation);
}

velox_Vec3 velox_Body_GetVelocity(velox_World* world, velox_BodyId body) {
    return fromVec3(reinterpret_cast<World*>(world)->body(toBodyId(body)).velocity);
}

velox_Vec3 velox_Body_GetAngularVelocity(velox_World* world, velox_BodyId body) {
    return fromVec3(reinterpret_cast<World*>(world)->body(toBodyId(body)).angularVelocity);
}

void velox_Body_SetPosition(velox_World* world, velox_BodyId body, velox_Vec3 position) {
    reinterpret_cast<World*>(world)->body(toBodyId(body)).position = toVec3(position);
}

void velox_Body_SetOrientation(velox_World* world, velox_BodyId body, velox_Quat orientation) {
    reinterpret_cast<World*>(world)->body(toBodyId(body)).orientation = toQuat(orientation);
}

void velox_Body_SetVelocity(velox_World* world, velox_BodyId body, velox_Vec3 velocity) {
    reinterpret_cast<World*>(world)->body(toBodyId(body)).velocity = toVec3(velocity);
}

void velox_Body_SetAngularVelocity(velox_World* world, velox_BodyId body, velox_Vec3 angularVelocity) {
    reinterpret_cast<World*>(world)->body(toBodyId(body)).angularVelocity = toVec3(angularVelocity);
}

void velox_Body_ApplyForce(velox_World* world, velox_BodyId body, velox_Vec3 force) {
    reinterpret_cast<World*>(world)->body(toBodyId(body)).force += toVec3(force);
}

void velox_Body_ApplyTorque(velox_World* world, velox_BodyId body, velox_Vec3 torque) {
    reinterpret_cast<World*>(world)->body(toBodyId(body)).torque += toVec3(torque);
}

void velox_Body_ApplyImpulse(velox_World* world, velox_BodyId body, velox_Vec3 impulse) {
    Body& b = reinterpret_cast<World*>(world)->body(toBodyId(body));
    b.velocity += toVec3(impulse) * b.invMass;
}

float velox_Body_GetMass(velox_World* world, velox_BodyId body) {
    float invMass = reinterpret_cast<World*>(world)->body(toBodyId(body)).invMass;
    return invMass > 0.0f ? 1.0f / invMass : 0.0f;
}

void velox_Body_SetMass(velox_World* world, velox_BodyId body, float mass) {
    Body& b = reinterpret_cast<World*>(world)->body(toBodyId(body));
    b.invMass = mass > 0.0f ? 1.0f / mass : 0.0f;
}

velox_ShapeType velox_Body_GetShapeType(velox_World* world, velox_BodyId body) {
    return static_cast<velox_ShapeType>(reinterpret_cast<World*>(world)->body(toBodyId(body)).shape);
}

// Queries
velox_RayHit velox_World_Raycast(velox_World* world, velox_Vec3 origin, velox_Vec3 direction, float maxDistance) {
    RayHit hit = reinterpret_cast<World*>(world)->rayCast(toVec3(origin), toVec3(direction), maxDistance);
    velox_RayHit result;
    result.hit = hit.hit;
    result.body = fromBodyId(hit.body);
    result.distance = hit.t;
    result.point = fromVec3(hit.point);
    result.normal = fromVec3(hit.normal);
    return result;
}

// Joints
velox_JointId velox_World_AddBallJoint(velox_World* world, velox_BodyId bodyA, velox_BodyId bodyB, velox_Vec3 anchor) {
    return fromJointId(reinterpret_cast<World*>(world)->addBallJoint(toBodyId(bodyA), toBodyId(bodyB), toVec3(anchor)));
}

velox_JointId velox_World_AddHingeJoint(velox_World* world, velox_BodyId bodyA, velox_BodyId bodyB, velox_Vec3 anchor, velox_Vec3 axis) {
    return fromJointId(reinterpret_cast<World*>(world)->addHingeJoint(toBodyId(bodyA), toBodyId(bodyB), toVec3(anchor), toVec3(axis)));
}

velox_JointId velox_World_AddDistanceJoint(velox_World* world, velox_BodyId bodyA, velox_BodyId bodyB, velox_Vec3 anchorA, velox_Vec3 anchorB) {
    return fromJointId(reinterpret_cast<World*>(world)->addDistanceJoint(toBodyId(bodyA), toBodyId(bodyB), toVec3(anchorA), toVec3(anchorB)));
}

velox_JointId velox_World_AddMotorJoint(velox_World* world, velox_BodyId bodyA, velox_BodyId bodyB, velox_Vec3 anchorA, velox_Vec3 anchorB, float maxForce, float maxTorque) {
    return fromJointId(reinterpret_cast<World*>(world)->addMotorJoint(toBodyId(bodyA), toBodyId(bodyB), toVec3(anchorA), toVec3(anchorB), maxForce, maxTorque));
}

void velox_World_RemoveJoint(velox_World* world, velox_JointId joint) {
    reinterpret_cast<World*>(world)->removeJoint(toJointId(joint));
}

// Explosion effect
void velox_World_Explode(velox_World* world, velox_Vec3 origin, float radius, float impulse) {
    reinterpret_cast<World*>(world)->explode(toVec3(origin), radius, impulse);
}

// Recording/Replay
velox_ReplayRecording* velox_ReplayRecording_Create(void) {
    try {
        ReplayRecording* r = new ReplayRecording();
        return reinterpret_cast<velox_ReplayRecording*>(r);
    } catch (...) {
        return nullptr;
    }
}

void velox_ReplayRecording_Destroy(velox_ReplayRecording* recording) {
    delete reinterpret_cast<ReplayRecording*>(recording);
}

void velox_ReplayRecording_Begin(velox_ReplayRecording* recording, velox_World* world, float dt) {
    beginReplay(*reinterpret_cast<ReplayRecording*>(recording), *reinterpret_cast<World*>(world), dt);
}

void velox_ReplayRecording_RecordFrame(velox_ReplayRecording* recording, velox_World* world) {
    recordReplayFrame(*reinterpret_cast<ReplayRecording*>(recording), *reinterpret_cast<World*>(world));
}

uint64_t velox_ReplayRecording_Verify(velox_ReplayRecording* recording, float positionTolerance, float velocityTolerance) {
    return verifyReplay(*reinterpret_cast<ReplayRecording*>(recording), positionTolerance, velocityTolerance);
}
