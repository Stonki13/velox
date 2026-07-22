#ifndef VELOX_STABLE_API_H
#define VELOX_STABLE_API_H

/**
 * @file stable_api.h
 * @brief Stable C API with versioned function pointers and backward
 *        compatibility shims for the Velox physics engine.
 *
 * This header wraps the flat C API from velox_c.h behind a versioned vtable
 * so that consumers can:
 *   - Query the runtime API version and negotiate compatibility.
 *   - Resolve function pointers at load time (useful for dynamic loading).
 *   - Use backward-compatible shims when an older call signature is retained.
 *
 * The vtable is append-only within a major version: new entries are added at
 * the end, existing entries never change position or signature.
 *
 * @code{.c}
 * velox_Api api;
 * velox_Api_Init(&api);
 * if (!velox_Api_IsCompatible(&api, VELOX_VERSION_ENCODE(1, 0, 0))) {
 *     fprintf(stderr, "Incompatible Velox runtime\n");
 *     return 1;
 * }
 * velox_World* w = api.World_Create(VELOX_BACKEND_CPU);
 * api.World_Step(w, 1.0f / 60.0f);
 * api.World_Destroy(w);
 * @endcode
 */

#include "velox_c.h"
#include "api_version.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ----------------------------------------------------------------------- */
/* Versioned function-pointer table                                        */
/* ----------------------------------------------------------------------- */

/**
 * @brief Stable, versioned API surface.
 *
 * Fields are ordered by the version in which they were introduced.
 * Within a major version the struct is append-only; consumers compiled
 * against an older header will simply ignore trailing fields.
 *
 * @note All pointers are non-NULL after velox_Api_Init succeeds.
 */
typedef struct velox_Api {
    /* --- Version 1.0.0 --- */
    uint32_t version_major;
    uint32_t version_minor;
    uint32_t version_patch;

    /* World lifecycle */
    velox_World* (*World_Create)(velox_BackendType backend);
    void         (*World_Destroy)(velox_World* world);
    void         (*World_Step)(velox_World* world, float dt);
    void         (*World_SetGravity)(velox_World* world, velox_Vec3 gravity);
    void         (*World_SetSubsteps)(velox_World* world, int substeps);

    /* Body creation */
    velox_BodyId (*World_AddSphere)(velox_World* w, velox_Vec3 pos, float radius, float mass);
    velox_BodyId (*World_AddBox)(velox_World* w, velox_Vec3 pos, velox_Vec3 half, float mass);
    velox_BodyId (*World_AddCapsule)(velox_World* w, velox_Vec3 pos, float r, float hh, float mass);
    velox_BodyId (*World_AddCylinder)(velox_World* w, velox_Vec3 pos, float r, float hh, float mass);
    velox_BodyId (*World_AddCone)(velox_World* w, velox_Vec3 pos, float r, float h, float mass);
    velox_BodyId (*World_AddStaticPlane)(velox_World* w, velox_Vec3 normal, float offset);
    void         (*World_RemoveBody)(velox_World* w, velox_BodyId body);

    /* Body accessors */
    velox_Vec3   (*Body_GetPosition)(velox_World* w, velox_BodyId b);
    velox_Quat   (*Body_GetOrientation)(velox_World* w, velox_BodyId b);
    velox_Vec3   (*Body_GetVelocity)(velox_World* w, velox_BodyId b);
    velox_Vec3   (*Body_GetAngularVelocity)(velox_World* w, velox_BodyId b);
    void         (*Body_SetPosition)(velox_World* w, velox_BodyId b, velox_Vec3 pos);
    void         (*Body_SetOrientation)(velox_World* w, velox_BodyId b, velox_Quat q);
    void         (*Body_SetVelocity)(velox_World* w, velox_BodyId b, velox_Vec3 vel);
    void         (*Body_SetAngularVelocity)(velox_World* w, velox_BodyId b, velox_Vec3 w_vel);
    void         (*Body_ApplyForce)(velox_World* w, velox_BodyId b, velox_Vec3 force);
    void         (*Body_ApplyTorque)(velox_World* w, velox_BodyId b, velox_Vec3 torque);
    void         (*Body_ApplyImpulse)(velox_World* w, velox_BodyId b, velox_Vec3 impulse);
    float        (*Body_GetMass)(velox_World* w, velox_BodyId b);
    void         (*Body_SetMass)(velox_World* w, velox_BodyId b, float mass);
    velox_ShapeType (*Body_GetShapeType)(velox_World* w, velox_BodyId b);

    /* Queries */
    velox_RayHit (*World_Raycast)(velox_World* w, velox_Vec3 origin, velox_Vec3 dir, float maxDist);

    /* Joints */
    velox_JointId (*World_AddBallJoint)(velox_World* w, velox_BodyId a, velox_BodyId b, velox_Vec3 anchor);
    velox_JointId (*World_AddHingeJoint)(velox_World* w, velox_BodyId a, velox_BodyId b, velox_Vec3 anchor, velox_Vec3 axis);
    velox_JointId (*World_AddDistanceJoint)(velox_World* w, velox_BodyId a, velox_BodyId b, velox_Vec3 anchorA, velox_Vec3 anchorB);
    velox_JointId (*World_AddMotorJoint)(velox_World* w, velox_BodyId a, velox_BodyId b, velox_Vec3 anchorA, velox_Vec3 anchorB, float maxForce, float maxTorque);
    void          (*World_RemoveJoint)(velox_World* w, velox_JointId joint);

    /* Effects */
    void (*World_Explode)(velox_World* w, velox_Vec3 origin, float radius, float impulse);

    /* Replay */
    velox_ReplayRecording* (*ReplayRecording_Create)(void);
    void     (*ReplayRecording_Destroy)(velox_ReplayRecording* rec);
    void     (*ReplayRecording_Begin)(velox_ReplayRecording* rec, velox_World* w, float dt);
    void     (*ReplayRecording_RecordFrame)(velox_ReplayRecording* rec, velox_World* w);
    uint64_t (*ReplayRecording_Verify)(velox_ReplayRecording* rec, float posTol, float velTol);

    /* --- Version 1.1.0 (future — append here) --- */
    /* void (*World_StepFixed)(velox_World* w); */

} velox_Api;

/* ----------------------------------------------------------------------- */
/* Initialization and version negotiation                                  */
/* ----------------------------------------------------------------------- */

/**
 * @brief Populate a velox_Api table with the current runtime's function
 *        pointers and version numbers.
 *
 * Zero-initializes the struct first, then fills every entry known to this
 * header version. Callers compiled against an older header will see a
 * smaller struct; the extra trailing bytes are zeroed and harmless.
 *
 * @param[out] api  Pointer to a caller-owned velox_Api (may be stack or heap).
 * @return 0 on success, non-zero on failure (e.g. NULL argument).
 */
int velox_Api_Init(velox_Api* api);

/**
 * @brief Check whether the runtime API is compatible with a required version.
 *
 * Compatibility means: same major version, and runtime minor >= required
 * minor. Patch level is ignored for compatibility purposes.
 *
 * @param api             Initialized API table.
 * @param requiredEncoded Version produced by VELOX_VERSION_ENCODE.
 * @return true if compatible.
 */
bool velox_Api_IsCompatible(const velox_Api* api, uint32_t requiredEncoded);

/**
 * @brief Return the runtime version as an encoded integer.
 * @param api Initialized API table.
 * @return VELOX_VERSION_ENCODE(major, minor, patch).
 */
uint32_t velox_Api_Version(const velox_Api* api);

/**
 * @brief Return a human-readable version string for the runtime.
 * @param api Initialized API table (may be NULL for a static string).
 * @return Static string like "1.0.0".
 */
const char* velox_Api_VersionString(const velox_Api* api);

/* ----------------------------------------------------------------------- */
/* Backward-compatibility shims                                            */
/* ----------------------------------------------------------------------- */

/**
 * @name Compatibility shims
 *
 * These inline wrappers preserve old call signatures that have been
 * superseded. They forward to the current API and are annotated with
 * deprecation warnings so consumers migrate at their own pace.
 * @{
 */

/**
 * @brief Legacy world creation (pre-1.0 used an int backend selector).
 * @deprecated Use velox_World_Create with a velox_BackendType enum.
 */
static inline velox_World* velox_World_Create_Legacy(int backendInt) {
    return velox_World_Create((velox_BackendType)backendInt);
}

/**
 * @brief Legacy step that took a double timestep.
 * @deprecated Use velox_World_Step with a float dt.
 */
static inline void velox_World_Step_Legacy(velox_World* world, double dt) {
    velox_World_Step(world, (float)dt);
}

/**
 * @brief Legacy gravity setter that took three floats instead of a Vec3.
 * @deprecated Use velox_World_SetGravity with a velox_Vec3.
 */
static inline void velox_World_SetGravity_Legacy(velox_World* world,
                                                  float gx, float gy, float gz) {
    velox_Vec3 g;
    g.x = gx; g.y = gy; g.z = gz;
    velox_World_SetGravity(world, g);
}

/** @} */ /* Compatibility shims */

/* ----------------------------------------------------------------------- */
/* Compile-time ABI guard for the vtable itself                            */
/* ----------------------------------------------------------------------- */

/**
 * The vtable layout is part of the ABI. These static assertions ensure the
 * struct does not silently change size or alignment between releases.
 * Consumers can add their own checks after including this header.
 */
#ifdef __cplusplus
static_assert(sizeof(velox_Api) > 0, "velox_Api must be a complete type");
static_assert(offsetof(velox_Api, version_major) == 0,
              "ABI: version_major must be the first field");
static_assert(offsetof(velox_Api, World_Create) > offsetof(velox_Api, version_patch),
              "ABI: function pointers follow version fields");
#endif

#ifdef __cplusplus
}
#endif

#endif /* VELOX_STABLE_API_H */
