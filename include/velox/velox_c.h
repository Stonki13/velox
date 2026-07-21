#ifndef VELOX_C_H
#define VELOX_C_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Opaque handle types
typedef struct velox_World velox_World;
typedef uint64_t velox_BodyId;
typedef uint64_t velox_JointId;

// Vector types
typedef struct { float x, y, z; } velox_Vec3;
typedef struct { float x, y, z, w; } velox_Quat;

// Backend type
typedef enum {
    VELOX_BACKEND_AUTO = 0,
    VELOX_BACKEND_CPU = 1,
    VELOX_BACKEND_CUDA = 2
} velox_BackendType;

// Shape type
typedef enum {
    VELOX_SHAPE_SPHERE = 0,
    VELOX_SHAPE_BOX = 1,
    VELOX_SHAPE_CAPSULE = 2,
    VELOX_SHAPE_CYLINDER = 3,
    VELOX_SHAPE_CONE = 4,
    VELOX_SHAPE_HULL = 5,
    VELOX_SHAPE_MESH = 6,
    VELOX_SHAPE_PLANE = 7,
    VELOX_SHAPE_COMPOUND = 8,
    VELOX_SHAPE_ROUNDED_BOX = 9,
    VELOX_SHAPE_ELLIPSOID = 10
} velox_ShapeType;

// Raycast hit result
typedef struct {
    bool hit;
    velox_BodyId body;
    float distance;
    velox_Vec3 point;
    velox_Vec3 normal;
} velox_RayHit;

// World management
velox_World* velox_World_Create(velox_BackendType backend);
void velox_World_Destroy(velox_World* world);
void velox_World_Step(velox_World* world, float dt);
void velox_World_SetGravity(velox_World* world, velox_Vec3 gravity);
void velox_World_SetSubsteps(velox_World* world, int substeps);

// Body creation
velox_BodyId velox_World_AddSphere(velox_World* world, velox_Vec3 position, float radius, float mass);
velox_BodyId velox_World_AddBox(velox_World* world, velox_Vec3 position, velox_Vec3 halfExtents, float mass);
velox_BodyId velox_World_AddCapsule(velox_World* world, velox_Vec3 position, float radius, float halfHeight, float mass);
velox_BodyId velox_World_AddCylinder(velox_World* world, velox_Vec3 position, float radius, float halfHeight, float mass);
velox_BodyId velox_World_AddCone(velox_World* world, velox_Vec3 position, float radius, float height, float mass);
velox_BodyId velox_World_AddStaticPlane(velox_World* world, velox_Vec3 normal, float offset);

// Body removal
void velox_World_RemoveBody(velox_World* world, velox_BodyId body);

// Body property access
velox_Vec3 velox_Body_GetPosition(velox_World* world, velox_BodyId body);
velox_Quat velox_Body_GetOrientation(velox_World* world, velox_BodyId body);
velox_Vec3 velox_Body_GetVelocity(velox_World* world, velox_BodyId body);
velox_Vec3 velox_Body_GetAngularVelocity(velox_World* world, velox_BodyId body);

void velox_Body_SetPosition(velox_World* world, velox_BodyId body, velox_Vec3 position);
void velox_Body_SetOrientation(velox_World* world, velox_BodyId body, velox_Quat orientation);
void velox_Body_SetVelocity(velox_World* world, velox_BodyId body, velox_Vec3 velocity);
void velox_Body_SetAngularVelocity(velox_World* world, velox_BodyId body, velox_Vec3 angularVelocity);

void velox_Body_ApplyForce(velox_World* world, velox_BodyId body, velox_Vec3 force);
void velox_Body_ApplyTorque(velox_World* world, velox_BodyId body, velox_Vec3 torque);
void velox_Body_ApplyImpulse(velox_World* world, velox_BodyId body, velox_Vec3 impulse);

float velox_Body_GetMass(velox_World* world, velox_BodyId body);
void velox_Body_SetMass(velox_World* world, velox_BodyId body, float mass);

velox_ShapeType velox_Body_GetShapeType(velox_World* world, velox_BodyId body);

// Queries
velox_RayHit velox_World_Raycast(velox_World* world, velox_Vec3 origin, velox_Vec3 direction, float maxDistance);

// Joints
velox_JointId velox_World_AddBallJoint(velox_World* world, velox_BodyId bodyA, velox_BodyId bodyB, velox_Vec3 anchor);
velox_JointId velox_World_AddHingeJoint(velox_World* world, velox_BodyId bodyA, velox_BodyId bodyB, velox_Vec3 anchor, velox_Vec3 axis);
velox_JointId velox_World_AddDistanceJoint(velox_World* world, velox_BodyId bodyA, velox_BodyId bodyB, velox_Vec3 anchorA, velox_Vec3 anchorB);
velox_JointId velox_World_AddMotorJoint(velox_World* world, velox_BodyId bodyA, velox_BodyId bodyB, velox_Vec3 anchorA, velox_Vec3 anchorB, float maxForce, float maxTorque);

void velox_World_RemoveJoint(velox_World* world, velox_JointId joint);

// Explosion effect
void velox_World_Explode(velox_World* world, velox_Vec3 origin, float radius, float impulse);

#ifdef __cplusplus
}
#endif

#endif // VELOX_C_H
