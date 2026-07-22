/**
 * @file test_api_stability.cpp
 * @brief Unit tests for API version macros, ABI layout guarantees, the
 *        stable C API vtable, and deprecation infrastructure.
 */

#include "doctest.h"

#include <velox/api_version.h>
#include <velox/stable_api.h>
#include <velox/velox_c.h>
#include <velox/version.h>

#include <cstddef>
#include <cstring>
#include <type_traits>

// =========================================================================
// Compile-time version assertions (fail the build if headers are wrong)
// =========================================================================

VELOX_REQUIRE_VERSION(1, 0, 0);

// The encoded version must match the individual components.
static_assert(VELOX_VERSION == VELOX_VERSION_ENCODE(1, 0, 0),
              "VELOX_VERSION encoding mismatch");

// ABI-compatible-since must not be in the future.
static_assert(VELOX_ABI_COMPATIBLE_SINCE <= VELOX_VERSION,
              "ABI compatible-since is ahead of current version");

// =========================================================================
// Compile-time ABI layout checks (the core guarantee)
// =========================================================================

// Vec3: 3 floats, no padding.
VELOX_ABI_CHECK_SIZE(velox_Vec3, 12);
VELOX_ABI_CHECK_ALIGN(velox_Vec3, 4);
VELOX_ABI_CHECK_OFFSET(velox_Vec3, x, 0);
VELOX_ABI_CHECK_OFFSET(velox_Vec3, y, 4);
VELOX_ABI_CHECK_OFFSET(velox_Vec3, z, 8);

// Quat: 4 floats, no padding.
VELOX_ABI_CHECK_SIZE(velox_Quat, 16);
VELOX_ABI_CHECK_ALIGN(velox_Quat, 4);
VELOX_ABI_CHECK_OFFSET(velox_Quat, x, 0);
VELOX_ABI_CHECK_OFFSET(velox_Quat, y, 4);
VELOX_ABI_CHECK_OFFSET(velox_Quat, z, 8);
VELOX_ABI_CHECK_OFFSET(velox_Quat, w, 12);

// BodyId / JointId are uint64_t.
static_assert(sizeof(velox_BodyId) == 8, "velox_BodyId must be 8 bytes");
static_assert(sizeof(velox_JointId) == 8, "velox_JointId must be 8 bytes");

// Enum underlying sizes.
static_assert(sizeof(velox_BackendType) == sizeof(int), "BackendType size");
static_assert(sizeof(velox_ShapeType) == sizeof(int), "ShapeType size");

// =========================================================================
// Version compatibility tests
// =========================================================================

TEST_CASE("Version macros are consistent") {
    CHECK(VELOX_VERSION_MAJOR == 1);
    CHECK(VELOX_VERSION_MINOR == 0);
    CHECK(VELOX_VERSION_PATCH == 0);
    CHECK(VELOX_VERSION == 10000u);
    CHECK(std::strcmp(VELOX_VERSION_STRING, "1.0.0") == 0);
}

TEST_CASE("VELOX_VERSION_ENCODE produces correct values") {
    CHECK(VELOX_VERSION_ENCODE(0, 0, 0) == 0u);
    CHECK(VELOX_VERSION_ENCODE(1, 0, 0) == 10000u);
    CHECK(VELOX_VERSION_ENCODE(1, 2, 3) == 10203u);
    CHECK(VELOX_VERSION_ENCODE(2, 0, 0) == 20000u);
    CHECK(VELOX_VERSION_ENCODE(0, 9, 15) == 915u);
}

TEST_CASE("C++ api namespace helpers match macros") {
    CHECK(velox::api::versionMajor() == VELOX_VERSION_MAJOR);
    CHECK(velox::api::versionMinor() == VELOX_VERSION_MINOR);
    CHECK(velox::api::versionPatch() == VELOX_VERSION_PATCH);
    CHECK(velox::api::versionEncoded() == VELOX_VERSION);
    CHECK(std::strcmp(velox::api::versionStr(), VELOX_VERSION_STRING) == 0);
}

TEST_CASE("C++ version constants from version.h match api_version.h") {
    CHECK(velox::kVersionMajor == VELOX_VERSION_MAJOR);
    CHECK(velox::kVersionMinor == VELOX_VERSION_MINOR);
    CHECK(velox::kVersionPatch == VELOX_VERSION_PATCH);
    CHECK(std::strcmp(velox::kVersionString, VELOX_VERSION_STRING) == 0);
}

TEST_CASE("isAbiCompatible accepts versions in range") {
    using velox::api::isAbiCompatible;

    // Current version is always compatible.
    CHECK(isAbiCompatible(VELOX_VERSION));

    // The minimum compatible version is compatible.
    CHECK(isAbiCompatible(VELOX_ABI_COMPATIBLE_SINCE));

    // A version below the minimum is not compatible.
    CHECK_FALSE(isAbiCompatible(VELOX_ABI_COMPATIBLE_SINCE - 1));

    // A future version beyond current is not compatible.
    CHECK_FALSE(isAbiCompatible(VELOX_VERSION + 1));
}

// =========================================================================
// ABI layout tests (runtime checks that mirror the static_asserts)
// =========================================================================

TEST_CASE("velox_Vec3 layout") {
    CHECK(sizeof(velox_Vec3) == 12);
    CHECK(alignof(velox_Vec3) == 4);
    CHECK(offsetof(velox_Vec3, x) == 0);
    CHECK(offsetof(velox_Vec3, y) == 4);
    CHECK(offsetof(velox_Vec3, z) == 8);
}

TEST_CASE("velox_Quat layout") {
    CHECK(sizeof(velox_Quat) == 16);
    CHECK(alignof(velox_Quat) == 4);
    CHECK(offsetof(velox_Quat, x) == 0);
    CHECK(offsetof(velox_Quat, y) == 4);
    CHECK(offsetof(velox_Quat, z) == 8);
    CHECK(offsetof(velox_Quat, w) == 12);
}

TEST_CASE("velox_RayHit layout stability") {
    // The exact size depends on platform padding rules, but the field
    // offsets that consumers rely on must be stable.
    CHECK(offsetof(velox_RayHit, hit) == 0);
    // body is a uint64_t; its offset must be stable.
    CHECK(offsetof(velox_RayHit, body) == offsetof(velox_RayHit, hit) + 8);
    // distance follows body.
    CHECK(offsetof(velox_RayHit, distance) == offsetof(velox_RayHit, body) + 8);
}

TEST_CASE("Enum values are stable") {
    CHECK(static_cast<int>(VELOX_BACKEND_AUTO) == 0);
    CHECK(static_cast<int>(VELOX_BACKEND_CPU) == 1);
    CHECK(static_cast<int>(VELOX_BACKEND_CUDA) == 2);

    CHECK(static_cast<int>(VELOX_SHAPE_SPHERE) == 0);
    CHECK(static_cast<int>(VELOX_SHAPE_BOX) == 1);
    CHECK(static_cast<int>(VELOX_SHAPE_CAPSULE) == 2);
    CHECK(static_cast<int>(VELOX_SHAPE_CYLINDER) == 3);
    CHECK(static_cast<int>(VELOX_SHAPE_CONE) == 4);
    CHECK(static_cast<int>(VELOX_SHAPE_HULL) == 5);
    CHECK(static_cast<int>(VELOX_SHAPE_MESH) == 6);
    CHECK(static_cast<int>(VELOX_SHAPE_PLANE) == 7);
    CHECK(static_cast<int>(VELOX_SHAPE_COMPOUND) == 8);
    CHECK(static_cast<int>(VELOX_SHAPE_ROUNDED_BOX) == 9);
    CHECK(static_cast<int>(VELOX_SHAPE_ELLIPSOID) == 10);
}

// =========================================================================
// Stable C API vtable tests
// =========================================================================

TEST_CASE("velox_Api_Init populates version fields") {
    velox_Api api;
    int rc = velox_Api_Init(&api);
    CHECK(rc == 0);
    CHECK(api.version_major == 1);
    CHECK(api.version_minor == 0);
    CHECK(api.version_patch == 0);
}

TEST_CASE("velox_Api_Init populates all function pointers") {
    velox_Api api;
    velox_Api_Init(&api);

    // World lifecycle
    CHECK(api.World_Create != nullptr);
    CHECK(api.World_Destroy != nullptr);
    CHECK(api.World_Step != nullptr);
    CHECK(api.World_SetGravity != nullptr);
    CHECK(api.World_SetSubsteps != nullptr);

    // Body creation
    CHECK(api.World_AddSphere != nullptr);
    CHECK(api.World_AddBox != nullptr);
    CHECK(api.World_AddCapsule != nullptr);
    CHECK(api.World_AddCylinder != nullptr);
    CHECK(api.World_AddCone != nullptr);
    CHECK(api.World_AddStaticPlane != nullptr);
    CHECK(api.World_RemoveBody != nullptr);

    // Body accessors
    CHECK(api.Body_GetPosition != nullptr);
    CHECK(api.Body_GetOrientation != nullptr);
    CHECK(api.Body_GetVelocity != nullptr);
    CHECK(api.Body_GetAngularVelocity != nullptr);
    CHECK(api.Body_SetPosition != nullptr);
    CHECK(api.Body_SetOrientation != nullptr);
    CHECK(api.Body_SetVelocity != nullptr);
    CHECK(api.Body_SetAngularVelocity != nullptr);
    CHECK(api.Body_ApplyForce != nullptr);
    CHECK(api.Body_ApplyTorque != nullptr);
    CHECK(api.Body_ApplyImpulse != nullptr);
    CHECK(api.Body_GetMass != nullptr);
    CHECK(api.Body_SetMass != nullptr);
    CHECK(api.Body_GetShapeType != nullptr);

    // Queries
    CHECK(api.World_Raycast != nullptr);

    // Joints
    CHECK(api.World_AddBallJoint != nullptr);
    CHECK(api.World_AddHingeJoint != nullptr);
    CHECK(api.World_AddDistanceJoint != nullptr);
    CHECK(api.World_AddMotorJoint != nullptr);
    CHECK(api.World_RemoveJoint != nullptr);

    // Effects
    CHECK(api.World_Explode != nullptr);

    // Replay
    CHECK(api.ReplayRecording_Create != nullptr);
    CHECK(api.ReplayRecording_Destroy != nullptr);
    CHECK(api.ReplayRecording_Begin != nullptr);
    CHECK(api.ReplayRecording_RecordFrame != nullptr);
    CHECK(api.ReplayRecording_Verify != nullptr);
}

TEST_CASE("velox_Api_Init rejects NULL") {
    CHECK(velox_Api_Init(nullptr) != 0);
}

TEST_CASE("velox_Api_IsCompatible") {
    velox_Api api;
    velox_Api_Init(&api);

    // Same major, same minor: compatible.
    CHECK(velox_Api_IsCompatible(&api, VELOX_VERSION_ENCODE(1, 0, 0)));

    // Same major, lower minor: compatible.
    CHECK(velox_Api_IsCompatible(&api, VELOX_VERSION_ENCODE(1, 0, 99)));

    // Different major: incompatible.
    CHECK_FALSE(velox_Api_IsCompatible(&api, VELOX_VERSION_ENCODE(2, 0, 0)));
    CHECK_FALSE(velox_Api_IsCompatible(&api, VELOX_VERSION_ENCODE(0, 9, 0)));

    // Higher minor than runtime: incompatible.
    CHECK_FALSE(velox_Api_IsCompatible(&api, VELOX_VERSION_ENCODE(1, 1, 0)));

    // NULL api: incompatible.
    CHECK_FALSE(velox_Api_IsCompatible(nullptr, VELOX_VERSION_ENCODE(1, 0, 0)));
}

TEST_CASE("velox_Api_Version returns encoded version") {
    velox_Api api;
    velox_Api_Init(&api);
    CHECK(velox_Api_Version(&api) == VELOX_VERSION);
    CHECK(velox_Api_Version(nullptr) == 0);
}

TEST_CASE("velox_Api_VersionString returns correct string") {
    velox_Api api;
    velox_Api_Init(&api);
    CHECK(std::strcmp(velox_Api_VersionString(&api), "1.0.0") == 0);
    CHECK(std::strcmp(velox_Api_VersionString(nullptr), "1.0.0") == 0);
}

TEST_CASE("velox_Api vtable function pointers match direct C API") {
    velox_Api api;
    velox_Api_Init(&api);

    // Static builds expose the direct implementation addresses. On Windows,
    // a shared-library import uses executable-local thunks, so pointer
    // identity is not part of the DLL ABI; the following end-to-end test
    // verifies those imported vtable entries are callable.
#if defined(VELOX_USING_SHARED) && defined(_WIN32)
    CHECK(api.World_Create != nullptr);
    CHECK(api.World_Destroy != nullptr);
    CHECK(api.World_Step != nullptr);
    CHECK(api.Body_GetPosition != nullptr);
    CHECK(api.World_Raycast != nullptr);
#else
    CHECK(reinterpret_cast<void*>(api.World_Create) ==
          reinterpret_cast<void*>(velox_World_Create));
    CHECK(reinterpret_cast<void*>(api.World_Destroy) ==
          reinterpret_cast<void*>(velox_World_Destroy));
    CHECK(reinterpret_cast<void*>(api.World_Step) ==
          reinterpret_cast<void*>(velox_World_Step));
    CHECK(reinterpret_cast<void*>(api.Body_GetPosition) ==
          reinterpret_cast<void*>(velox_Body_GetPosition));
    CHECK(reinterpret_cast<void*>(api.World_Raycast) ==
          reinterpret_cast<void*>(velox_World_Raycast));
#endif
}

TEST_CASE("velox_Api vtable is functional end-to-end") {
    velox_Api api;
    velox_Api_Init(&api);

    velox_World* w = api.World_Create(VELOX_BACKEND_CPU);
    REQUIRE(w != nullptr);

    velox_Vec3 gravity = {0.0f, -9.81f, 0.0f};
    api.World_SetGravity(w, gravity);

    velox_Vec3 pos = {0.0f, 5.0f, 0.0f};
    velox_BodyId ball = api.World_AddSphere(w, pos, 0.5f, 1.0f);
    CHECK(ball != UINT64_MAX);

    // Step a few frames; the ball should fall.
    for (int i = 0; i < 60; ++i) {
        api.World_Step(w, 1.0f / 60.0f);
    }

    velox_Vec3 newPos = api.Body_GetPosition(w, ball);
    CHECK(newPos.y < 5.0f); // gravity pulled it down

    api.World_Destroy(w);
}

// =========================================================================
// Backward-compatibility shim tests
// =========================================================================

TEST_CASE("Legacy shims forward correctly") {
    // velox_World_Create_Legacy with int backend
    velox_World* w = velox_World_Create_Legacy(1); // 1 == VELOX_BACKEND_CPU
    REQUIRE(w != nullptr);

    // velox_World_SetGravity_Legacy with three floats
    velox_World_SetGravity_Legacy(w, 0.0f, -9.81f, 0.0f);

    // velox_World_Step_Legacy with double dt
    velox_World_Step_Legacy(w, 1.0 / 60.0);

    velox_World_Destroy(w);
}

// =========================================================================
// Feature detection macro tests
// =========================================================================

TEST_CASE("Feature detection macros are defined") {
    // These must be 0 or 1.
    CHECK((VELOX_HAS_CUDA == 0 || VELOX_HAS_CUDA == 1));
    CHECK((VELOX_HAS_REPLAY == 0 || VELOX_HAS_REPLAY == 1));
    CHECK((VELOX_HAS_CHARACTER == 0 || VELOX_HAS_CHARACTER == 1));
    CHECK((VELOX_HAS_VEHICLE == 0 || VELOX_HAS_VEHICLE == 1));
    CHECK((VELOX_HAS_RAGDOLL == 0 || VELOX_HAS_RAGDOLL == 1));
    CHECK((VELOX_HAS_SERIALIZATION_V2 == 0 || VELOX_HAS_SERIALIZATION_V2 == 1));
    CHECK((VELOX_HAS_STABLE_C_API == 0 || VELOX_HAS_STABLE_C_API == 1));
    CHECK((VELOX_HAS_CCD == 0 || VELOX_HAS_CCD == 1));
    CHECK((VELOX_HAS_DEBUG_DRAW == 0 || VELOX_HAS_DEBUG_DRAW == 1));
}

TEST_CASE("VELOX_FEATURE macro expands correctly") {
    // VELOX_FEATURE(REPLAY) should equal VELOX_HAS_REPLAY.
    CHECK(VELOX_FEATURE(REPLAY) == VELOX_HAS_REPLAY);
    CHECK(VELOX_FEATURE(CUDA) == VELOX_HAS_CUDA);
    CHECK(VELOX_FEATURE(STABLE_C_API) == VELOX_HAS_STABLE_C_API);
}

// =========================================================================
// Deprecation infrastructure tests
// =========================================================================

TEST_CASE("VELOX_DEPRECATED macro is defined and usable") {
    // We cannot easily test that a warning is emitted portably, but we can
    // verify the macro expands to valid syntax by using it on a local struct.
    struct VELOX_DEPRECATED("test deprecation") DeprecatedTestType {
        int value;
    };

    // The type is still usable (deprecation is a warning, not an error).
    DeprecatedTestType t;
    t.value = 42;
    CHECK(t.value == 42);
}

TEST_CASE("VELOX_DEPRECATED_SINCE macro expands correctly") {
    struct VELOX_DEPRECATED_SINCE(1, 1, "use NewType instead") OldType {
        int x;
    };
    OldType o;
    o.x = 7;
    CHECK(o.x == 7);
}

TEST_CASE("VELOX_REMOVAL_IN macro expands correctly") {
    struct VELOX_REMOVAL_IN(2, 0, "migrate to v2") LegacyType {
        float f;
    };
    LegacyType l;
    l.f = 3.14f;
    CHECK(l.f == doctest::Approx(3.14f));
}

// =========================================================================
// C++ type-trait sanity checks for public types
// =========================================================================

TEST_CASE("Public C types are trivially copyable") {
    CHECK(std::is_trivially_copyable<velox_Vec3>::value);
    CHECK(std::is_trivially_copyable<velox_Quat>::value);
    CHECK(std::is_trivially_copyable<velox_RayHit>::value);
}

TEST_CASE("velox_Api is trivially copyable (POD-like vtable)") {
    CHECK(std::is_trivially_copyable<velox_Api>::value);
}
