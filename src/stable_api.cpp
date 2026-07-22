/**
 * @file stable_api.cpp
 * @brief Implementation of the versioned stable C API (velox_Api_Init, etc.).
 */

#include <velox/stable_api.h>
#include <cstring>

extern "C" {

int velox_Api_Init(velox_Api* api) {
    if (!api) return -1;

    // Zero the entire struct so any trailing fields unknown to this header
    // version are NULL / 0 rather than garbage.
    std::memset(api, 0, sizeof(velox_Api));

    api->version_major = VELOX_VERSION_MAJOR;
    api->version_minor = VELOX_VERSION_MINOR;
    api->version_patch = VELOX_VERSION_PATCH;

    /* World lifecycle */
    api->World_Create      = velox_World_Create;
    api->World_Destroy     = velox_World_Destroy;
    api->World_Step        = velox_World_Step;
    api->World_SetGravity  = velox_World_SetGravity;
    api->World_SetSubsteps = velox_World_SetSubsteps;

    /* Body creation */
    api->World_AddSphere      = velox_World_AddSphere;
    api->World_AddBox         = velox_World_AddBox;
    api->World_AddCapsule     = velox_World_AddCapsule;
    api->World_AddCylinder    = velox_World_AddCylinder;
    api->World_AddCone        = velox_World_AddCone;
    api->World_AddStaticPlane = velox_World_AddStaticPlane;
    api->World_RemoveBody     = velox_World_RemoveBody;

    /* Body accessors */
    api->Body_GetPosition       = velox_Body_GetPosition;
    api->Body_GetOrientation    = velox_Body_GetOrientation;
    api->Body_GetVelocity       = velox_Body_GetVelocity;
    api->Body_GetAngularVelocity = velox_Body_GetAngularVelocity;
    api->Body_SetPosition       = velox_Body_SetPosition;
    api->Body_SetOrientation    = velox_Body_SetOrientation;
    api->Body_SetVelocity       = velox_Body_SetVelocity;
    api->Body_SetAngularVelocity = velox_Body_SetAngularVelocity;
    api->Body_ApplyForce        = velox_Body_ApplyForce;
    api->Body_ApplyTorque       = velox_Body_ApplyTorque;
    api->Body_ApplyImpulse      = velox_Body_ApplyImpulse;
    api->Body_GetMass           = velox_Body_GetMass;
    api->Body_SetMass           = velox_Body_SetMass;
    api->Body_GetShapeType      = velox_Body_GetShapeType;

    /* Queries */
    api->World_Raycast = velox_World_Raycast;

    /* Joints */
    api->World_AddBallJoint     = velox_World_AddBallJoint;
    api->World_AddHingeJoint    = velox_World_AddHingeJoint;
    api->World_AddDistanceJoint = velox_World_AddDistanceJoint;
    api->World_AddMotorJoint    = velox_World_AddMotorJoint;
    api->World_RemoveJoint      = velox_World_RemoveJoint;

    /* Effects */
    api->World_Explode = velox_World_Explode;

    /* Replay */
    api->ReplayRecording_Create     = velox_ReplayRecording_Create;
    api->ReplayRecording_Destroy    = velox_ReplayRecording_Destroy;
    api->ReplayRecording_Begin      = velox_ReplayRecording_Begin;
    api->ReplayRecording_RecordFrame = velox_ReplayRecording_RecordFrame;
    api->ReplayRecording_Verify     = velox_ReplayRecording_Verify;

    return 0;
}

bool velox_Api_IsCompatible(const velox_Api* api, uint32_t requiredEncoded) {
    if (!api) return false;

    uint32_t reqMajor = requiredEncoded / 10000u;
    uint32_t reqMinor = (requiredEncoded / 100u) % 100u;

    // Same major, runtime minor >= required minor.
    if (api->version_major != reqMajor) return false;
    if (api->version_minor < reqMinor) return false;
    return true;
}

uint32_t velox_Api_Version(const velox_Api* api) {
    if (!api) return 0;
    return VELOX_VERSION_ENCODE(api->version_major, api->version_minor, api->version_patch);
}

const char* velox_Api_VersionString(const velox_Api* /*api*/) {
    return VELOX_VERSION_STRING;
}

} // extern "C"
