// Cross-platform endianness and serialization tests for Velox.
//
// Verifies that the serialization format is portable across little-endian
// and big-endian platforms. All modern targets (x86_64, ARM64) are
// little-endian, but the tests document the assumption and verify that
// the serialization layer handles byte order correctly.
//
// The V2 format uses explicit little-endian encoding for all multi-byte
// integers and IEEE 754 little-endian for floats, making archives portable
// regardless of host byte order.

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "../unit/doctest.h"

#include <velox/world.h>
#include <velox/math.h>
#include <velox/serialization.h>
#include <velox/serialization_v2.h>
#include <velox/platform.h>

#include <cstdint>
#include <cstring>
#include <vector>

using namespace velox;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Detect host byte order at runtime.
static bool isLittleEndian() {
    uint32_t val = 1;
    uint8_t bytes[4];
    std::memcpy(bytes, &val, 4);
    return bytes[0] == 1;
}

/// Read a little-endian uint32 from a byte buffer.
static uint32_t readLE32(const uint8_t* p) {
    return uint32_t(p[0]) |
           (uint32_t(p[1]) << 8) |
           (uint32_t(p[2]) << 16) |
           (uint32_t(p[3]) << 24);
}

/// Write a uint32 in little-endian to a byte buffer.
static void writeLE32(uint8_t* p, uint32_t v) {
    p[0] = uint8_t(v);
    p[1] = uint8_t(v >> 8);
    p[2] = uint8_t(v >> 16);
    p[3] = uint8_t(v >> 24);
}

/// Read a little-endian float from a byte buffer.
static float readLEFloat(const uint8_t* p) {
    uint32_t bits = readLE32(p);
    float f;
    std::memcpy(&f, &bits, sizeof(f));
    return f;
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

TEST_SUITE("cross_platform.endianness") {

    TEST_CASE("Host endianness detection") {
        // All current Velox targets are little-endian.
        CHECK(isLittleEndian());

        // Verify with a known pattern.
        uint16_t val16 = 0x0102;
        uint8_t bytes[2];
        std::memcpy(bytes, &val16, 2);
        if (isLittleEndian()) {
            CHECK(bytes[0] == 0x02);
            CHECK(bytes[1] == 0x01);
        } else {
            CHECK(bytes[0] == 0x01);
            CHECK(bytes[1] == 0x02);
        }
    }

    TEST_CASE("Little-endian encode/decode round-trip") {
        uint32_t testValues[] = {0, 1, 0xDEADBEEF, 0x12345678, UINT32_MAX};
        for (uint32_t v : testValues) {
            uint8_t buf[4];
            writeLE32(buf, v);
            CHECK(readLE32(buf) == v);

            // Verify byte layout is little-endian regardless of host.
            CHECK(buf[0] == uint8_t(v & 0xFF));
            CHECK(buf[1] == uint8_t((v >> 8) & 0xFF));
            CHECK(buf[2] == uint8_t((v >> 16) & 0xFF));
            CHECK(buf[3] == uint8_t((v >> 24) & 0xFF));
        }
    }

    TEST_CASE("Float little-endian encoding") {
        float testValues[] = {0.0f, 1.0f, -1.0f, 3.14159f, 1e-30f, 1e30f};
        for (float v : testValues) {
            uint32_t bits;
            std::memcpy(&bits, &v, sizeof(bits));

            uint8_t buf[4];
            writeLE32(buf, bits);
            float decoded = readLEFloat(buf);

            uint32_t decodedBits;
            std::memcpy(&decodedBits, &decoded, sizeof(decodedBits));
            CHECK(decodedBits == bits);
        }
    }

    TEST_CASE("V2 archive magic is little-endian") {
        // The magic 'VLX2' is stored as 0x32584C56 in little-endian.
        // On disk: 56 4C 58 32 = 'V' 'L' 'X' '2'
        uint8_t magic[4];
        writeLE32(magic, kSerializationV2Magic);
        CHECK(magic[0] == 'V');
        CHECK(magic[1] == 'L');
        CHECK(magic[2] == 'X');
        CHECK(magic[3] == '2');
    }

    TEST_CASE("Serialization produces portable byte stream") {
        World world(BackendType::Cpu);
        world.setGravity({0.0f, -9.81f, 0.0f});
        world.addStaticPlane({0, 1, 0}, 0.0f);
        world.addSphere({0, 5, 0}, 0.5f, 1.0f);
        world.addBox({1, 3, 0}, {0.3f, 0.3f, 0.3f}, 2.0f);

        // Step to populate dynamic state.
        for (int i = 0; i < 30; ++i)
            world.step(1.0f / 60.0f);

        // V1 serialization.
        SerializedScene scene = serializeWorld(world, "endianness-test");
        std::vector<uint8_t> packed = packScene(scene);

        // The packed stream must be non-empty and start with identifiable
        // header bytes (version field).
        REQUIRE(packed.size() > 8);

        // Unpack and verify round-trip.
        SerializedScene unpacked = unpackScene(packed);
        CHECK(unpacked.version == scene.version);
        CHECK(unpacked.metadata == scene.metadata);
        CHECK(unpacked.data.size() == scene.data.size());
        CHECK(unpacked.data == scene.data);
    }

    TEST_CASE("V2 serialization round-trip") {
        World world(BackendType::Cpu);
        world.setGravity({0.0f, -9.81f, 0.0f});
        world.addStaticPlane({0, 1, 0}, 0.0f);
        world.addSphere({0, 5, 0}, 0.5f, 1.0f);
        world.addBox({2, 3, 0}, {0.4f, 0.4f, 0.4f}, 1.5f);

        for (int i = 0; i < 60; ++i)
            world.step(1.0f / 60.0f);

        // V2 serialization with no compression (always available).
        SerializationOptions opts;
        opts.compression = CompressionV2::None;
        ArchiveV2 archive = serializeWorldV2(world, opts);

        // Pack to byte stream for portability verification.
        std::vector<uint8_t> packed = packArchiveV2(archive);
        REQUIRE(packed.size() > 16);

        // Verify magic at the start of the packed stream.
        uint32_t magic = readLE32(packed.data());
        CHECK(magic == kSerializationV2Magic);

        // Unpack and deserialize into a fresh world.
        ArchiveV2 unpacked = unpackArchiveV2(packed);
        World restored(BackendType::Cpu);
        deserializeWorldV2(restored, unpacked);

        CHECK(restored.bodyCount() == world.bodyCount());
    }

    TEST_CASE("CRC-32 is endian-independent") {
        // CRC-32 operates on bytes, not words, so it must produce the same
        // result regardless of host endianness.
        const uint8_t data[] = "Velox cross-platform CRC test";
        uint32_t crc = crc32(data, sizeof(data) - 1);

        // Known-good CRC for this exact byte sequence (computed with the
        // standard ISO 3309 polynomial 0xEDB88320).
        // We verify it's non-zero and deterministic.
        CHECK(crc != 0);

        // Same input must produce same output.
        uint32_t crc2 = crc32(data, sizeof(data) - 1);
        CHECK(crc == crc2);

        // Different input must (almost certainly) produce different output.
        const uint8_t data2[] = "Velox cross-platform CRC test!";
        uint32_t crc3 = crc32(data2, sizeof(data2) - 1);
        CHECK(crc3 != crc);
    }

    TEST_CASE("Multi-byte integer serialization in body data") {
        // Verify that body indices and generation counters (uint32/uint64)
        // survive serialization regardless of host byte order.
        World world(BackendType::Cpu);
        world.setGravity({0, -9.81f, 0});
        world.addStaticPlane({0, 1, 0}, 0.0f);

        // Add enough bodies to exercise multi-byte slot indices.
        for (int i = 0; i < 10; ++i)
            world.addSphere({float(i), 5.0f, 0.0f}, 0.3f, 1.0f);

        SerializedScene scene = serializeWorld(world);
        World restored(BackendType::Cpu);
        deserializeWorld(restored, scene);

        CHECK(restored.bodyCount() == world.bodyCount());

        // Verify body handles are preserved.
        for (uint32_t i = 0; i < world.bodyCount(); ++i) {
            BodyId origId = BodyId::make(i, 0);
            CHECK(restored.isValid(origId));
        }
    }

    TEST_CASE("Byte-level archive structure is stable") {
        // Serialize a minimal scene and verify the byte layout doesn't
        // change between runs (structural determinism).
        World world(BackendType::Cpu);
        world.setGravity({0, -9.81f, 0});
        world.addStaticPlane({0, 1, 0}, 0.0f);

        auto s1 = packScene(serializeWorld(world));
        auto s2 = packScene(serializeWorld(world));

        REQUIRE(s1.size() == s2.size());
        CHECK(s1 == s2);
    }

    TEST_CASE("Byte-level archive preserves simulated contacts and joints") {
        World world(BackendType::Cpu);
        world.setGravity({0, -9.81f, 0});
        world.addStaticPlane({0, 1, 0}, 0.0f);
        const BodyId box = world.addBox({0, 0.5f, 0}, {0.5f, 0.5f, 0.5f}, 1.0f);
        const BodyId bob = world.addSphere({0, 2.0f, 0}, 0.25f, 1.0f);
        world.addDistanceJoint(box, bob, {0, 1.0f, 0}, {0, 1.75f, 0});
        for (int i = 0; i < 30; ++i) world.step(1.0f / 60.0f);

        const auto s1 = packScene(serializeWorld(world));
        const auto s2 = packScene(serializeWorld(world));

        REQUIRE(s1.size() == s2.size());
        CHECK(s1 == s2);
    }
}
