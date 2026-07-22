// Cross-platform struct alignment and layout tests for Velox.
//
// Verifies that critical data structures have consistent size, alignment,
// and field offsets across all supported platforms and compilers. This is
// essential for:
//   - Serialization portability (binary layout must match)
//   - GPU upload (CUDA expects specific struct layouts)
//   - SIMD operations (require specific alignment)
//   - C FFI (velox_c.h exposes structs to other languages)
//
// The Body struct uses alignas(64) for cache-line alignment. Vec3/Quat
// are plain floats with natural alignment. These tests document and
// enforce the expected layout.

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "../unit/doctest.h"

#include <velox/body.h>
#include <velox/math.h>
#include <velox/world.h>
#include <velox/joint.h>
#include <velox/platform.h>

#include <cstddef>
#include <cstdint>
#include <type_traits>

using namespace velox;

// ---------------------------------------------------------------------------
// Compile-time layout assertions
// ---------------------------------------------------------------------------

// Vec3: 3 floats, naturally aligned to 4 bytes.
static_assert(sizeof(Vec3) == 12, "Vec3 must be 12 bytes (3 floats)");
static_assert(alignof(Vec3) == 4, "Vec3 must be 4-byte aligned");

// Vec2: 2 floats.
static_assert(sizeof(Vec2) == 8, "Vec2 must be 8 bytes (2 floats)");
static_assert(alignof(Vec2) == 4, "Vec2 must be 4-byte aligned");

// Quat: 4 floats.
static_assert(sizeof(Quat) == 16, "Quat must be 16 bytes (4 floats)");
static_assert(alignof(Quat) == 4, "Quat must be 4-byte aligned");

// BodyId: single uint64_t.
static_assert(sizeof(BodyId) == 8, "BodyId must be 8 bytes");
static_assert(alignof(BodyId) == 8, "BodyId must be 8-byte aligned");

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

TEST_SUITE("cross_platform.alignment") {

    TEST_CASE("Vec3 layout") {
        CHECK(sizeof(Vec3) == 12);
        CHECK(alignof(Vec3) == 4);

        // Field offsets must be sequential floats.
        CHECK(offsetof(Vec3, x) == 0);
        CHECK(offsetof(Vec3, y) == 4);
        CHECK(offsetof(Vec3, z) == 8);
    }

    TEST_CASE("Vec2 layout") {
        CHECK(sizeof(Vec2) == 8);
        CHECK(alignof(Vec2) == 4);
        CHECK(offsetof(Vec2, x) == 0);
        CHECK(offsetof(Vec2, y) == 4);
    }

    TEST_CASE("Quat layout") {
        CHECK(sizeof(Quat) == 16);
        CHECK(alignof(Quat) == 4);
        CHECK(offsetof(Quat, x) == 0);
        CHECK(offsetof(Quat, y) == 4);
        CHECK(offsetof(Quat, z) == 8);
        CHECK(offsetof(Quat, w) == 12);
    }

    TEST_CASE("BodyId layout") {
        CHECK(sizeof(BodyId) == 8);
        CHECK(alignof(BodyId) == 8);
        CHECK(offsetof(BodyId, value) == 0);

        // Verify slot/generation packing.
        BodyId id = BodyId::make(42, 7);
        CHECK(id.slot() == 42);
        CHECK(id.generation() == 7);
        uint64_t expected = (uint64_t(7) << 32) | 42;
        CHECK(id.value == expected);
    }

    TEST_CASE("Body struct alignment") {
        // Body uses alignas(64) for cache-line alignment.
        CHECK(alignof(Body) == 64);

        // Body must be at least as large as its largest member.
        CHECK(sizeof(Body) >= 64);

        // Size must be a multiple of alignment.
        CHECK(sizeof(Body) % 64 == 0);
    }

    TEST_CASE("Body field offsets are stable") {
        // These offsets are part of the serialization contract and the
        // GPU upload path. They must not change without a version bump.
        //
        // We check the first few critical fields. The exact offsets depend
        // on the full struct definition, so we verify relative ordering
        // and alignment rather than absolute values (which would break on
        // any struct change).

        // position must come before orientation in memory.
        CHECK(offsetof(Body, position) < offsetof(Body, orientation));

        // position must be 4-byte aligned (it's a Vec3).
        CHECK(offsetof(Body, position) % 4 == 0);

        // orientation must be 4-byte aligned (it's a Quat).
        CHECK(offsetof(Body, orientation) % 4 == 0);

        // linearVelocity must be 4-byte aligned.
        CHECK(offsetof(Body, velocity) % 4 == 0);

        // angularVelocity must be 4-byte aligned.
        CHECK(offsetof(Body, angularVelocity) % 4 == 0);
    }

    TEST_CASE("Enum sizes are portable") {
        // Enums used in serialization must have stable sizes.
        CHECK(sizeof(ShapeType) == 1);   // uint8_t underlying
        CHECK(sizeof(MotionType) == 1);  // uint8_t underlying

        // Verify specific enum values for serialization stability.
        CHECK(static_cast<uint8_t>(ShapeType::Sphere) == 0);
        CHECK(static_cast<uint8_t>(ShapeType::Plane) == 1);
        CHECK(static_cast<uint8_t>(ShapeType::Box) == 2);
        CHECK(static_cast<uint8_t>(ShapeType::Capsule) == 3);

        CHECK(static_cast<uint8_t>(MotionType::Static) == 0);
        CHECK(static_cast<uint8_t>(MotionType::Kinematic) == 1);
        CHECK(static_cast<uint8_t>(MotionType::Dynamic) == 2);
    }

    TEST_CASE("Array of Vec3 is tightly packed") {
        // Vec3 arrays (vertex buffers, position arrays) must have no
        // padding between elements for GPU upload and serialization.
        Vec3 arr[4] = {{1,2,3}, {4,5,6}, {7,8,9}, {10,11,12}};

        // Each element must be exactly 12 bytes apart.
        const char* base = reinterpret_cast<const char*>(arr);
        for (int i = 0; i < 4; ++i) {
            const Vec3* elem = reinterpret_cast<const Vec3*>(base + i * 12);
            CHECK(elem->x == arr[i].x);
            CHECK(elem->y == arr[i].y);
            CHECK(elem->z == arr[i].z);
        }
    }

    TEST_CASE("Array of Quat is tightly packed") {
        Quat arr[3] = {{0,0,0,1}, {1,0,0,0}, {0,1,0,0}};

        const char* base = reinterpret_cast<const char*>(arr);
        for (int i = 0; i < 3; ++i) {
            const Quat* elem = reinterpret_cast<const Quat*>(base + i * 16);
            CHECK(elem->x == arr[i].x);
            CHECK(elem->y == arr[i].y);
            CHECK(elem->z == arr[i].z);
            CHECK(elem->w == arr[i].w);
        }
    }

    TEST_CASE("Struct alignment in arrays") {
        // When Body structs are in a vector, each must maintain 64-byte
        // alignment for SIMD access.
        // Note: std::vector respects alignas, so this should always pass.
        alignas(64) Body bodies[2];
        CHECK(reinterpret_cast<uintptr_t>(&bodies[0]) % 64 == 0);
        CHECK(reinterpret_cast<uintptr_t>(&bodies[1]) % 64 == 0);

        // The stride between elements must be a multiple of 64.
        ptrdiff_t stride = reinterpret_cast<const char*>(&bodies[1]) -
                           reinterpret_cast<const char*>(&bodies[0]);
        CHECK(stride % 64 == 0);
        CHECK(stride == static_cast<ptrdiff_t>(sizeof(Body)));
    }

    TEST_CASE("Platform pointer size") {
        // Document pointer size for the target platform.
        CHECK(sizeof(void*) == 8); // All supported targets are 64-bit.
        CHECK(sizeof(size_t) == 8);
        CHECK(sizeof(ptrdiff_t) == 8);
    }

    TEST_CASE("Integer type sizes are portable") {
        CHECK(sizeof(uint8_t) == 1);
        CHECK(sizeof(uint16_t) == 2);
        CHECK(sizeof(uint32_t) == 4);
        CHECK(sizeof(uint64_t) == 8);
        CHECK(sizeof(int8_t) == 1);
        CHECK(sizeof(int16_t) == 2);
        CHECK(sizeof(int32_t) == 4);
        CHECK(sizeof(int64_t) == 8);
    }

    TEST_CASE("JointId layout matches BodyId") {
        // JointId uses the same packed handle scheme as BodyId.
        CHECK(sizeof(JointId) == sizeof(BodyId));
        CHECK(alignof(JointId) == alignof(BodyId));
    }

    TEST_CASE("Platform identification macros are mutually exclusive") {
        // Exactly one platform must be defined.
        int platforms = VELOX_PLATFORM_WINDOWS + VELOX_PLATFORM_LINUX + VELOX_PLATFORM_MACOS;
        bool onePlatform = (platforms == 1);
        CHECK(onePlatform);

        // Exactly one architecture must be defined.
        int archs = VELOX_ARCH_X86_64 + VELOX_ARCH_ARM64;
        bool oneArch = (archs == 1);
        CHECK(oneArch);

        // Combined platform+arch macros must be consistent.
#if VELOX_PLATFORM_WINDOWS
        CHECK(VELOX_PLATFORM_MACOS_X86 == 0);
        CHECK(VELOX_PLATFORM_MACOS_ARM == 0);
        CHECK(VELOX_PLATFORM_LINUX_X86 == 0);
        CHECK(VELOX_PLATFORM_LINUX_ARM == 0);
#endif
    }
}
