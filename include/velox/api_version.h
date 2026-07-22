#pragma once

/**
 * @file api_version.h
 * @brief API stability guarantees for the Velox physics engine.
 *
 * This header provides:
 *   - Semantic versioning macros (compile-time and preprocessor)
 *   - ABI compatibility checks (struct sizes, alignments, offsets)
 *   - Deprecation warning infrastructure
 *   - Feature detection macros
 *
 * Include this header in any translation unit that needs to verify it was
 * compiled against a compatible version of Velox.
 */

#include "version.h"
#include "platform.h"

#include <cstddef>
#include <cstdint>

// ---------------------------------------------------------------------------
// Semantic versioning macros
// ---------------------------------------------------------------------------

/// Encode a semantic version triple into a single comparable integer.
#define VELOX_VERSION_ENCODE(major, minor, patch) \
    (((major) * 10000u) + ((minor) * 100u) + (patch))

/// The version of the headers being compiled against.
#define VELOX_VERSION \
    VELOX_VERSION_ENCODE(VELOX_VERSION_MAJOR, VELOX_VERSION_MINOR, VELOX_VERSION_PATCH)

// Mirror the C++ constants as preprocessor macros so #if guards work.
#define VELOX_VERSION_MAJOR 1
#define VELOX_VERSION_MINOR 0
#define VELOX_VERSION_PATCH 0

/// Human-readable version string.
#define VELOX_VERSION_STRING "1.0.0"

/// Minimum C++ standard required (encoded as YYYYMM, e.g. 201703 for C++17).
#define VELOX_MIN_CXX_STANDARD 201703L

/// The minimum API version this header set is backward-compatible with.
/// Consumers compiled against any version >= this and <= VELOX_VERSION
/// will link and run correctly.
#define VELOX_ABI_COMPATIBLE_SINCE VELOX_VERSION_ENCODE(1, 0, 0)

// ---------------------------------------------------------------------------
// Compile-time version assertion
// ---------------------------------------------------------------------------

/**
 * @brief Static-assert that the headers match an expected version.
 *
 * Place this at file scope in a consumer translation unit to fail the build
 * immediately if the installed headers are older than required:
 *
 * @code
 * VELOX_REQUIRE_VERSION(1, 0, 0);
 * @endcode
 */
#define VELOX_REQUIRE_VERSION(major, minor, patch)                            \
    static_assert(                                                            \
        VELOX_VERSION >= VELOX_VERSION_ENCODE(major, minor, patch),           \
        "Velox headers are older than required (" #major "." #minor "." #patch ")")

/**
 * @brief Static-assert that the headers are not newer than a maximum version.
 *
 * Useful for pinning a build to a known-good API surface.
 */
#define VELOX_MAX_VERSION(major, minor, patch)                                \
    static_assert(                                                            \
        VELOX_VERSION <= VELOX_VERSION_ENCODE(major, minor, patch),           \
        "Velox headers are newer than the maximum supported version")

// ---------------------------------------------------------------------------
// ABI compatibility checks
// ---------------------------------------------------------------------------

/**
 * @brief Compile-time ABI layout verification.
 *
 * Each VELOX_ABI_CHECK_* macro static-asserts a property of a public struct.
 * If any assertion fires, the ABI has changed in a breaking way and the
 * library must be recompiled (or the consumer must update).
 */

/// Assert the size of a type in bytes.
#define VELOX_ABI_CHECK_SIZE(Type, expected_bytes)                            \
    static_assert(sizeof(Type) == (expected_bytes),                           \
        "ABI break: sizeof(" #Type ") changed")

/// Assert the alignment of a type in bytes.
#define VELOX_ABI_CHECK_ALIGN(Type, expected_align)                           \
    static_assert(alignof(Type) == (expected_align),                          \
        "ABI break: alignof(" #Type ") changed")

/// Assert the byte offset of a member within a struct.
#define VELOX_ABI_CHECK_OFFSET(Type, member, expected_offset)                 \
    static_assert(offsetof(Type, member) == (expected_offset),                \
        "ABI break: offset of " #Type "::" #member " changed")

/// Combined size + alignment check.
#define VELOX_ABI_CHECK(Type, expected_bytes, expected_align)                 \
    VELOX_ABI_CHECK_SIZE(Type, expected_bytes);                               \
    VELOX_ABI_CHECK_ALIGN(Type, expected_align)

// ---------------------------------------------------------------------------
// Deprecation infrastructure
// ---------------------------------------------------------------------------

// VELOX_DEPRECATED is defined in version.h; re-export guard here for
// consumers that include only api_version.h.
#ifndef VELOX_DEPRECATED
  #if defined(_MSC_VER)
    #define VELOX_DEPRECATED(message) __declspec(deprecated(message))
  #elif defined(__GNUC__) || defined(__clang__)
    #define VELOX_DEPRECATED(message) __attribute__((deprecated(message)))
  #else
    #define VELOX_DEPRECATED(message)
  #endif
#endif

/**
 * @brief Mark a symbol as deprecated since a specific version.
 *
 * Usage:
 * @code
 * VELOX_DEPRECATED_SINCE(1, 1, "use World::stepFixed() instead")
 * void step(float dt);
 * @endcode
 */
#define VELOX_DEPRECATED_SINCE(major, minor, message)                         \
    VELOX_DEPRECATED("Deprecated since " #major "." #minor ": " message)

/**
 * @brief Mark a symbol for removal in a future version.
 *
 * Usage:
 * @code
 * VELOX_REMOVAL_IN(2, 0, "migrate to the v2 serialization API")
 * void serializeLegacy(Buffer& buf);
 * @endcode
 */
#define VELOX_REMOVAL_IN(major, minor, message)                               \
    VELOX_DEPRECATED("Will be removed in " #major "." #minor ": " message)

/**
 * @brief Conditionally deprecate based on the current version.
 *
 * When VELOX_VERSION >= the deprecation version the symbol is marked
 * deprecated; otherwise it compiles cleanly. This allows headers to carry
 * future deprecation annotations that activate only when the version bumps.
 */
#if VELOX_VERSION >= VELOX_VERSION_ENCODE(1, 1, 0)
  #define VELOX_DEPRECATED_IF_1_1(message) VELOX_DEPRECATED(message)
#else
  #define VELOX_DEPRECATED_IF_1_1(message)
#endif

#if VELOX_VERSION >= VELOX_VERSION_ENCODE(2, 0, 0)
  #define VELOX_DEPRECATED_IF_2_0(message) VELOX_DEPRECATED(message)
#else
  #define VELOX_DEPRECATED_IF_2_0(message)
#endif

// ---------------------------------------------------------------------------
// Feature detection macros
// ---------------------------------------------------------------------------

/// Defined to 1 when the CUDA backend is compiled in.
#ifndef VELOX_HAS_CUDA
#define VELOX_HAS_CUDA 0
#endif

/// Defined to 1 when the deterministic replay system is available.
#ifndef VELOX_HAS_REPLAY
#define VELOX_HAS_REPLAY 1
#endif

/// Defined to 1 when the character controller module is available.
#ifndef VELOX_HAS_CHARACTER
#define VELOX_HAS_CHARACTER 1
#endif

/// Defined to 1 when the vehicle module is available.
#ifndef VELOX_HAS_VEHICLE
#define VELOX_HAS_VEHICLE 1
#endif

/// Defined to 1 when the ragdoll module is available.
#ifndef VELOX_HAS_RAGDOLL
#define VELOX_HAS_RAGDOLL 1
#endif

/// Defined to 1 when the v2 serialization format is available.
#ifndef VELOX_HAS_SERIALIZATION_V2
#define VELOX_HAS_SERIALIZATION_V2 1
#endif

/// Defined to 1 when the stable C API (stable_api.h) is available.
#ifndef VELOX_HAS_STABLE_C_API
#define VELOX_HAS_STABLE_C_API 1
#endif

/// Defined to 1 when continuous collision detection is available.
#ifndef VELOX_HAS_CCD
#define VELOX_HAS_CCD 1
#endif

/// Defined to 1 when the debug draw interface is available.
#ifndef VELOX_HAS_DEBUG_DRAW
#define VELOX_HAS_DEBUG_DRAW 1
#endif

/**
 * @brief Query a feature at compile time.
 *
 * @code
 * #if VELOX_FEATURE(CUDA)
 *     // use CUDA path
 * #endif
 * @endcode
 */
#define VELOX_FEATURE(name) VELOX_HAS_##name

// ---------------------------------------------------------------------------
// C++ namespace-level helpers
// ---------------------------------------------------------------------------

#ifdef __cplusplus
namespace velox {
namespace api {

/// Runtime version query (mirrors the preprocessor macros).
inline constexpr uint32_t versionMajor() { return VELOX_VERSION_MAJOR; }
inline constexpr uint32_t versionMinor() { return VELOX_VERSION_MINOR; }
inline constexpr uint32_t versionPatch() { return VELOX_VERSION_PATCH; }
inline constexpr uint32_t versionEncoded() { return VELOX_VERSION; }
inline constexpr const char* versionStr() { return VELOX_VERSION_STRING; }

/// The minimum version this build is ABI-compatible back to.
inline constexpr uint32_t abiCompatibleSince() { return VELOX_ABI_COMPATIBLE_SINCE; }

/**
 * @brief Check whether a target version is ABI-compatible with this build.
 * @param targetEncoded A version produced by VELOX_VERSION_ENCODE.
 * @return true if the target is within the compatible range.
 */
inline constexpr bool isAbiCompatible(uint32_t targetEncoded) {
    return targetEncoded >= abiCompatibleSince() && targetEncoded <= versionEncoded();
}

} // namespace api
} // namespace velox
#endif // __cplusplus
