// Cross-platform determinism tests for Velox.
//
// Verifies that the CPU backend produces bitwise-identical simulation results
// across Windows, Linux, and macOS on both x86_64 and ARM64. These tests
// exercise the full simulation pipeline — integration, collision detection,
// constraint solving — and compare results at the bit level.
//
// When VELOX_STRICT_FLOATING_POINT is enabled the engine uses /fp:precise
// (MSVC) or -ffp-contract=off -fno-fast-math (GCC/Clang), which guarantees
// IEEE 754 compliance and eliminates FMA contraction differences between
// architectures.

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "../unit/doctest.h"

#include <velox/world.h>
#include <velox/math.h>
#include <velox/serialization.h>
#include <velox/platform.h>
#include <velox/simd.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

using namespace velox;

#if VELOX_STRICT_FLOATING_POINT
static_assert(!VELOX_SIMD_AVAILABLE,
              "strict replay must use the canonical scalar math path");
#endif

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Extract the raw bit pattern of a float for bitwise comparison.
static uint32_t floatBits(float f) {
    uint32_t bits;
    std::memcpy(&bits, &f, sizeof(bits));
    return bits;
}

/// Build a deterministic test scene: a stack of boxes on a plane.
static void buildStackScene(World& world, int boxCount, float spacing) {
#if VELOX_STRICT_FLOATING_POINT
    world.setDeterminismMode(DeterminismMode::Strict);
#endif
    world.setGravity({0.0f, -9.81f, 0.0f});
    world.addStaticPlane({0.0f, 1.0f, 0.0f}, 0.0f);

    for (int i = 0; i < boxCount; ++i) {
        Vec3 pos{0.0f, 0.5f + float(i) * spacing, 0.0f};
        world.addBox(pos, {0.4f, 0.4f, 0.4f}, 1.0f);
    }
}

/// Run N steps and collect all body positions into a flat vector.
static std::vector<float> simulateAndCollect(World& world, int steps, float dt) {
    for (int i = 0; i < steps; ++i)
        world.step(dt);

    std::vector<float> result;
    for (uint32_t i = 0; i < world.bodyCount(); ++i) {
        const Body& b = world.body(BodyId::make(i, 0));
        result.push_back(b.position.x);
        result.push_back(b.position.y);
        result.push_back(b.position.z);
        result.push_back(b.orientation.x);
        result.push_back(b.orientation.y);
        result.push_back(b.orientation.z);
        result.push_back(b.orientation.w);
    }
    return result;
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

TEST_SUITE("cross_platform.determinism") {

    TEST_CASE("Identical scene produces identical results on repeated runs") {
        // Run the same scene twice in the same process — must be bitwise equal.
        constexpr int kSteps = 300;
        constexpr float kDt = 1.0f / 60.0f;

        World w1(BackendType::Cpu);
        buildStackScene(w1, 8, 1.1f);
        auto r1 = simulateAndCollect(w1, kSteps, kDt);

        World w2(BackendType::Cpu);
        buildStackScene(w2, 8, 1.1f);
        auto r2 = simulateAndCollect(w2, kSteps, kDt);

        REQUIRE(r1.size() == r2.size());
        for (size_t i = 0; i < r1.size(); ++i) {
            CHECK(floatBits(r1[i]) == floatBits(r2[i]));
        }
    }

    TEST_CASE("Serialization round-trip preserves bitwise state") {
        World world(BackendType::Cpu);
        buildStackScene(world, 5, 1.2f);
        constexpr float kDt = 1.0f / 60.0f;

        // Simulate a few frames to populate warm-start data.
        for (int i = 0; i < 60; ++i)
            world.step(kDt);

        // Serialize and deserialize into a fresh world.
        SerializedScene scene = serializeWorld(world);
        World restored(BackendType::Cpu);
#if VELOX_STRICT_FLOATING_POINT
        restored.setDeterminismMode(DeterminismMode::Strict);
#endif
        deserializeWorld(restored, scene);

        // Continue both for another 120 frames — must remain bitwise equal.
        auto r1 = simulateAndCollect(world, 120, kDt);
        auto r2 = simulateAndCollect(restored, 120, kDt);

        REQUIRE(r1.size() == r2.size());
        for (size_t i = 0; i < r1.size(); ++i) {
            CHECK(floatBits(r1[i]) == floatBits(r2[i]));
        }
    }

    TEST_CASE("Replay verification passes on same platform") {
        World world(BackendType::Cpu);
        buildStackScene(world, 6, 1.0f);
        constexpr float kDt = 1.0f / 60.0f;

        ReplayRecording rec;
        beginReplay(rec, world, kDt);
        for (int i = 0; i < 200; ++i) {
            world.step(kDt);
            recordReplayFrame(rec, world);
        }

        uint64_t mismatch = verifyReplay(rec);
        CHECK(mismatch == 0);
    }

    TEST_CASE("Deterministic trace output for cross-platform comparison") {
        // This test produces a canonical trace that the CI parity job compares
        // across platforms. The trace is a sequence of position hashes.
        World world(BackendType::Cpu);
        buildStackScene(world, 10, 1.05f);
        constexpr float kDt = 1.0f / 60.0f;
        constexpr int kFrames = 1000;

        std::vector<uint32_t> trace;
        trace.reserve(kFrames);

        for (int frame = 0; frame < kFrames; ++frame) {
            world.step(kDt);

            // FNV-1a hash of all body positions this frame.
            uint32_t hash = 2166136261u;
            for (uint32_t i = 0; i < world.bodyCount(); ++i) {
                const Body& b = world.body(BodyId::make(i, 0));
                const float vals[] = {
                    b.position.x, b.position.y, b.position.z,
                    b.orientation.x, b.orientation.y, b.orientation.z, b.orientation.w
                };
                for (float v : vals) {
                    uint32_t bits = floatBits(v);
                    for (int byte = 0; byte < 4; ++byte) {
                        hash ^= (bits >> (byte * 8)) & 0xFF;
                        hash *= 16777619u;
                    }
                }
            }
            trace.push_back(hash);
        }

        // The trace must be non-trivial (not all zeros or all same).
        bool allSame = true;
        for (size_t i = 1; i < trace.size(); ++i) {
            if (trace[i] != trace[0]) { allSame = false; break; }
        }
        CHECK_FALSE(allSame);

        // Print the trace for CI artifact collection.
        printf("VELOX_XPLATFORM_TRACE ");
        for (uint32_t h : trace)
            printf("%08x", h);
        printf("\n");
    }

    TEST_CASE("Platform identification is consistent") {
        // Exactly one platform and one architecture must be active.
        int platformCount = VELOX_PLATFORM_WINDOWS + VELOX_PLATFORM_LINUX + VELOX_PLATFORM_MACOS;
        CHECK(platformCount == 1);

        int archCount = VELOX_ARCH_X86_64 + VELOX_ARCH_ARM64;
        CHECK(archCount == 1);
    }

    TEST_CASE("Math operations are deterministic across invocations") {
        // Verify that basic math ops produce identical results when called
        // multiple times (no hidden state, no platform-specific fast paths
        // that vary between calls).
        Vec3 a{1.0f, 2.0f, 3.0f};
        Vec3 b{4.0f, 5.0f, 6.0f};

        Vec3 c1 = cross(a, b);
        Vec3 c2 = cross(a, b);
        CHECK(floatBits(c1.x) == floatBits(c2.x));
        CHECK(floatBits(c1.y) == floatBits(c2.y));
        CHECK(floatBits(c1.z) == floatBits(c2.z));

        float d1 = dot(a, b);
        float d2 = dot(a, b);
        CHECK(floatBits(d1) == floatBits(d2));

        Quat q = fromAxisAngle({0.0f, 1.0f, 0.0f}, 0.785398f);
        Quat q2 = fromAxisAngle({0.0f, 1.0f, 0.0f}, 0.785398f);
        CHECK(floatBits(q.x) == floatBits(q2.x));
        CHECK(floatBits(q.y) == floatBits(q2.y));
        CHECK(floatBits(q.z) == floatBits(q2.z));
        CHECK(floatBits(q.w) == floatBits(q2.w));

        Vec3 r1 = rotate(q, a);
        Vec3 r2 = rotate(q2, a);
        CHECK(floatBits(r1.x) == floatBits(r2.x));
        CHECK(floatBits(r1.y) == floatBits(r2.y));
        CHECK(floatBits(r1.z) == floatBits(r2.z));
    }
}
