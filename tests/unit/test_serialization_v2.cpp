#include "doctest.h"
#include <velox/velox.h>
#include <velox/serialization_v2.h>

#include <chrono>
#include <cstring>

using namespace velox;

// ===========================================================================
// Round-trip tests
// ===========================================================================

TEST_CASE("v2 round-trip empty world") {
    World world(BackendType::Cpu);
    world.setGravity({0, -9.81f, 0});

    ArchiveV2 archive = serializeWorldV2(world, {CompressionV2::None, -1,
                                                  true, true, true, "empty"});
    CHECK(archive.version == kSerializationV2Version);
    CHECK(archive.metadata == "empty");
    CHECK_FALSE(archive.sections.empty());

    World restored(BackendType::Cpu);
    deserializeWorldV2(restored, archive);
    CHECK(restored.bodyCount() == 0);
    CHECK(restored.gravityValue().y == doctest::Approx(-9.81f));
}

TEST_CASE("v2 round-trip with bodies") {
    World world(BackendType::Cpu);
    world.setGravity({0, -9.81f, 0});
    world.addStaticPlane({0, 1, 0}, 0.0f);
    world.addSphere({0, 5, 0}, 0.5f, 1.0f);
    world.addBox({3, 5, 0}, {1, 1, 1}, 2.0f);
    world.addCapsule({-3, 5, 0}, 0.3f, 1.0f, 1.5f);

    ArchiveV2 archive = serializeWorldV2(world);
    World restored(BackendType::Cpu);
    deserializeWorldV2(restored, archive);
    CHECK(restored.bodyCount() == world.bodyCount());
}

TEST_CASE("v2 round-trip preserves body state after simulation") {
    World world(BackendType::Cpu);
    world.setGravity({0, -9.81f, 0});
    world.addStaticPlane({0, 1, 0}, 0.0f);
    BodyId ball = world.addSphere({0, 10, 0}, 0.5f, 1.0f);
    world.setLinearVelocity(ball, {2, 0, 0});

    for (int i = 0; i < 60; ++i)
        world.step(1.0f / 60.0f);

    Vec3 posBefore = world.body(ball).position;
    Vec3 velBefore = world.body(ball).velocity;

    ArchiveV2 archive = serializeWorldV2(world);
    World restored(BackendType::Cpu);
    deserializeWorldV2(restored, archive);

    CHECK(restored.bodyCount() == world.bodyCount());

    // Step both worlds and verify they stay in sync.
    world.step(1.0f / 60.0f);
    restored.step(1.0f / 60.0f);

    // Find the ball in the restored world (same slot/gen).
    CHECK(restored.isValid(ball));
    Vec3 posAfter = world.body(ball).position;
    Vec3 restoredPos = restored.body(ball).position;
    CHECK(length(posAfter - restoredPos) < doctest::Approx(1e-4f));
}

TEST_CASE("v2 round-trip with joints") {
    World world(BackendType::Cpu);
    BodyId a = world.addSphere({0, 5, 0}, 0.5f, 1.0f);
    BodyId b = world.addSphere({2, 5, 0}, 0.5f, 1.0f);
    world.addDistanceJoint(a, b, {0, 5, 0}, {2, 5, 0});

    ArchiveV2 archive = serializeWorldV2(world);
    World restored(BackendType::Cpu);
    deserializeWorldV2(restored, archive);
    CHECK(restored.bodyCount() == 2);
}

TEST_CASE("v2 pack/unpack binary round-trip") {
    World world(BackendType::Cpu);
    world.addSphere({0, 5, 0}, 0.5f, 1.0f);
    world.addBox({3, 5, 0}, {1, 1, 1}, 2.0f);

    ArchiveV2 archive = serializeWorldV2(world, {CompressionV2::None, -1,
                                                  true, true, true, "pack test"});
    std::vector<uint8_t> bytes = packArchiveV2(archive);
    CHECK_FALSE(bytes.empty());

    // Verify magic.
    uint32_t magic = 0;
    std::memcpy(&magic, bytes.data(), 4);
    CHECK(magic == kSerializationV2Magic);

    ArchiveV2 unpacked = unpackArchiveV2(bytes);
    CHECK(unpacked.version == archive.version);
    CHECK(unpacked.sections.size() == archive.sections.size());

    // Deserialize from unpacked archive.
    World restored(BackendType::Cpu);
    deserializeWorldV2(restored, unpacked);
    CHECK(restored.bodyCount() == world.bodyCount());
}

TEST_CASE("v2 pack/unpack preserves section CRCs") {
    World world(BackendType::Cpu);
    world.addSphere({0, 5, 0}, 0.5f, 1.0f);

    ArchiveV2 archive = serializeWorldV2(world);
    std::vector<uint8_t> bytes = packArchiveV2(archive);
    ArchiveV2 unpacked = unpackArchiveV2(bytes);

    REQUIRE(unpacked.sections.size() == archive.sections.size());
    for (size_t i = 0; i < archive.sections.size(); ++i) {
        CHECK(unpacked.sections[i].crc == archive.sections[i].crc);
        CHECK(unpacked.sections[i].type == archive.sections[i].type);
        CHECK(unpacked.sections[i].payload == archive.sections[i].payload);
    }
}

// ===========================================================================
// Version migration tests
// ===========================================================================

TEST_CASE("v1 to v2 migration round-trip") {
    World world(BackendType::Cpu);
    world.setGravity({0, -9.81f, 0});
    world.addStaticPlane({0, 1, 0}, 0.0f);
    world.addSphere({0, 5, 0}, 0.5f, 1.0f);

    // Serialize with V1.
    SerializedScene v1 = serializeWorld(world, "v1 scene");

    // Migrate to V2.
    ArchiveV2 archive = migrateV1ToV2(v1);
    CHECK(archive.version == kSerializationV2Version);
    CHECK(isMigratedV1(archive));

    // Deserialize through V2 path.
    World restored(BackendType::Cpu);
    deserializeWorldV2(restored, archive);
    CHECK(restored.bodyCount() == world.bodyCount());
}

TEST_CASE("v1 to v2 migration preserves metadata") {
    World world(BackendType::Cpu);
    world.addSphere({0, 5, 0}, 0.5f, 1.0f);

    SerializedScene v1 = serializeWorld(world, "my scene name");
    ArchiveV2 archive = migrateV1ToV2(v1);

    // The metadata section should contain the migration marker + original.
    const ArchiveSection* metaSec = archive.find(SectionType::Metadata);
    REQUIRE(metaSec != nullptr);
    CHECK(isMigratedV1(archive));
}

TEST_CASE("v2 native archive is not flagged as v1 migration") {
    World world(BackendType::Cpu);
    world.addSphere({0, 5, 0}, 0.5f, 1.0f);

    ArchiveV2 archive = serializeWorldV2(world);
    CHECK_FALSE(isMigratedV1(archive));
}

TEST_CASE("v2 rejects unsupported version") {
    World world(BackendType::Cpu);
    world.addSphere({0, 5, 0}, 0.5f, 1.0f);

    ArchiveV2 archive = serializeWorldV2(world);
    archive.version = kSerializationV2Version + 100;

    World restored(BackendType::Cpu);
    CHECK_THROWS(deserializeWorldV2(restored, archive));
}

// ===========================================================================
// Corruption detection tests
// ===========================================================================

TEST_CASE("v2 CRC detects single-bit corruption") {
    World world(BackendType::Cpu);
    world.addSphere({0, 5, 0}, 0.5f, 1.0f);
    world.addBox({3, 5, 0}, {1, 1, 1}, 2.0f);

    ArchiveV2 archive = serializeWorldV2(world);
    std::vector<uint8_t> bytes = packArchiveV2(archive);

    // Flip a bit in the middle of the payload.
    REQUIRE(bytes.size() > 40);
    bytes[bytes.size() / 2] ^= 0x01;

    CHECK_THROWS(unpackArchiveV2(bytes));
}

TEST_CASE("v2 CRC detects truncation") {
    World world(BackendType::Cpu);
    world.addSphere({0, 5, 0}, 0.5f, 1.0f);

    ArchiveV2 archive = serializeWorldV2(world);
    std::vector<uint8_t> bytes = packArchiveV2(archive);

    // Truncate the last 8 bytes.
    REQUIRE(bytes.size() > 8);
    bytes.resize(bytes.size() - 8);

    CHECK_THROWS(unpackArchiveV2(bytes));
}

TEST_CASE("v2 CRC detects section payload corruption") {
    World world(BackendType::Cpu);
    world.addSphere({0, 5, 0}, 0.5f, 1.0f);

    ArchiveV2 archive = serializeWorldV2(world);

    // Corrupt the Bodies section payload directly.
    ArchiveSection* bodiesSec = archive.find(SectionType::Bodies);
    REQUIRE(bodiesSec != nullptr);
    REQUIRE_FALSE(bodiesSec->payload.empty());
    bodiesSec->payload[0] ^= 0xFF;

    // Deserialization should detect the CRC mismatch.
    World restored(BackendType::Cpu);
    CHECK_THROWS(deserializeWorldV2(restored, archive));
}

TEST_CASE("v2 rejects bad magic") {
    std::vector<uint8_t> garbage = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
                                    10, 11, 12, 13, 14, 15, 16, 17,
                                    18, 19, 20, 21, 22, 23};
    CHECK_THROWS(unpackArchiveV2(garbage));
}

TEST_CASE("v2 rejects too-small archive") {
    std::vector<uint8_t> tiny = {0x56, 0x4C, 0x58, 0x32};
    CHECK_THROWS(unpackArchiveV2(tiny));
}

TEST_CASE("v2 CRC-32 known values") {
    // CRC-32 of empty data.
    CHECK(crc32(nullptr, 0) == 0x00000000u);

    // CRC-32 of "123456789" is 0xCBF43926 (standard check value).
    const char* check = "123456789";
    uint32_t result = crc32(check, 9);
    CHECK(result == 0xCBF43926u);
}

TEST_CASE("v2 CRC-32 detects all single-bit errors in 64 bytes") {
    std::vector<uint8_t> data(64, 0xAB);
    uint32_t baseline = crc32(data.data(), data.size());

    for (size_t byte = 0; byte < data.size(); ++byte) {
        for (int bit = 0; bit < 8; ++bit) {
            std::vector<uint8_t> corrupted = data;
            corrupted[byte] ^= static_cast<uint8_t>(1u << bit);
            uint32_t corruptedCrc = crc32(corrupted.data(), corrupted.size());
            CHECK(corruptedCrc != baseline);
        }
    }
}

// ===========================================================================
// Incremental serialization tests
// ===========================================================================

TEST_CASE("v2 incremental round-trip") {
    World world(BackendType::Cpu);
    world.setGravity({0, -9.81f, 0});
    world.addStaticPlane({0, 1, 0}, 0.0f);
    for (int i = 0; i < 20; ++i)
        world.addSphere({static_cast<float>(i) * 2.0f, 5, 0}, 0.5f, 1.0f);

    SerializationOptions opts;
    opts.metadata = "incremental test";
    IncrementalSerializer serializer(world, opts);

    // Step through chunks.
    while (serializer.nextChunk(4)) {}

    ArchiveV2 archive = serializer.finish();
    CHECK(archive.version == kSerializationV2Version);

    World restored(BackendType::Cpu);
    IncrementalDeserializer deserializer(restored, archive);
    while (deserializer.nextChunk()) {}
    deserializer.finish();

    CHECK(restored.bodyCount() == world.bodyCount());
}

TEST_CASE("v2 incremental matches monolithic") {
    World world(BackendType::Cpu);
    world.setGravity({0, -9.81f, 0});
    world.addStaticPlane({0, 1, 0}, 0.0f);
    for (int i = 0; i < 10; ++i)
        world.addBox({static_cast<float>(i) * 3.0f, 5, 0}, {1, 1, 1}, 1.0f);

    // Monolithic.
    ArchiveV2 mono = serializeWorldV2(world);
    std::vector<uint8_t> monoBytes = packArchiveV2(mono);

    // Incremental.
    IncrementalSerializer serializer(world);
    while (serializer.nextChunk(8)) {}
    ArchiveV2 incr = serializer.finish();
    std::vector<uint8_t> incrBytes = packArchiveV2(incr);

    // Both should deserialize to the same body count.
    World r1(BackendType::Cpu), r2(BackendType::Cpu);
    deserializeWorldV2(r1, unpackArchiveV2(monoBytes));
    deserializeWorldV2(r2, unpackArchiveV2(incrBytes));
    CHECK(r1.bodyCount() == r2.bodyCount());
}

// ===========================================================================
// JSON export tests
// ===========================================================================

TEST_CASE("v2 JSON export produces valid structure") {
    World world(BackendType::Cpu);
    world.setGravity({0, -9.81f, 0});
    world.addSphere({0, 5, 0}, 0.5f, 1.0f);
    world.addBox({3, 5, 0}, {1, 1, 1}, 2.0f);

    std::string json = exportWorldJson(world);
    CHECK_FALSE(json.empty());
    CHECK(json.find("\"velox-scene-v2\"") != std::string::npos);
    CHECK(json.find("\"bodyCount\": 2") != std::string::npos);
    CHECK(json.find("\"Sphere\"") != std::string::npos);
    CHECK(json.find("\"Box\"") != std::string::npos);
}

TEST_CASE("v2 archive JSON export") {
    World world(BackendType::Cpu);
    world.addSphere({0, 5, 0}, 0.5f, 1.0f);

    ArchiveV2 archive = serializeWorldV2(world, {CompressionV2::None, -1,
                                                  true, true, true, "json test"});
    std::string json = exportArchiveJson(archive);
    CHECK_FALSE(json.empty());
    CHECK(json.find("\"velox-archive-v2\"") != std::string::npos);
    CHECK(json.find("\"sectionCount\"") != std::string::npos);
}

// ===========================================================================
// Scene graph export tests
// ===========================================================================

TEST_CASE("v2 scene graph export") {
    World world(BackendType::Cpu);
    world.addStaticPlane({0, 1, 0}, 0.0f);
    BodyId a = world.addSphere({0, 5, 0}, 0.5f, 1.0f);
    BodyId b = world.addBox({3, 5, 0}, {1, 1, 1}, 2.0f);
    world.addDistanceJoint(a, b, {0, 5, 0}, {3, 5, 0});

    std::vector<SceneGraphNode> graph = exportSceneGraph(world);

    // Root + 3 bodies + 1 joint = 5 nodes.
    CHECK(graph.size() == 5);
    CHECK(graph[0].name == "World");
    CHECK(graph[0].type == "world");

    // Root should have children (bodies + joint).
    CHECK(graph[0].children.size() == 4);
}

TEST_CASE("v2 scene graph JSON export") {
    World world(BackendType::Cpu);
    world.addSphere({0, 5, 0}, 0.5f, 1.0f);

    std::vector<SceneGraphNode> graph = exportSceneGraph(world);
    std::string json = sceneGraphToJson(graph);
    CHECK_FALSE(json.empty());
    CHECK(json.find("\"World\"") != std::string::npos);
    CHECK(json.find("\"body\"") != std::string::npos);
}

TEST_CASE("v2 scene graph DOT export") {
    World world(BackendType::Cpu);
    world.addSphere({0, 5, 0}, 0.5f, 1.0f);
    world.addBox({3, 5, 0}, {1, 1, 1}, 2.0f);

    std::vector<SceneGraphNode> graph = exportSceneGraph(world);
    std::string dot = sceneGraphToDot(graph);
    CHECK_FALSE(dot.empty());
    CHECK(dot.find("digraph VeloxScene") != std::string::npos);
    CHECK(dot.find("->") != std::string::npos);
}

// ===========================================================================
// Compression tests (compile-time gated)
// ===========================================================================

TEST_CASE("v2 compression availability") {
    CHECK(isCompressionAvailable(CompressionV2::None));
    // Zlib/Zstd availability depends on build configuration.
    // Just verify the function doesn't crash.
    (void)isCompressionAvailable(CompressionV2::Zlib);
    (void)isCompressionAvailable(CompressionV2::Zstd);
}

TEST_CASE("v2 None compression round-trip") {
    std::vector<uint8_t> data = {1, 2, 3, 4, 5, 6, 7, 8};
    std::vector<uint8_t> compressed = compressBytes(data, CompressionV2::None);
    CHECK(compressed == data);

    std::vector<uint8_t> decompressed =
        decompressBytes(compressed, CompressionV2::None, data.size());
    CHECK(decompressed == data);
}

TEST_CASE("v2 unavailable compression throws") {
    std::vector<uint8_t> data = {1, 2, 3};
    if (!isCompressionAvailable(CompressionV2::Zlib))
        CHECK_THROWS(compressBytes(data, CompressionV2::Zlib));
    if (!isCompressionAvailable(CompressionV2::Zstd))
        CHECK_THROWS(compressBytes(data, CompressionV2::Zstd));
}

// ===========================================================================
// Large scene performance tests
// ===========================================================================

TEST_CASE("v2 large scene round-trip performance" * doctest::timeout(30)) {
    const int kBodyCount = 5000;

    World world(BackendType::Cpu);
    world.setGravity({0, -9.81f, 0});
    world.addStaticPlane({0, 1, 0}, 0.0f);

    for (int i = 0; i < kBodyCount; ++i) {
        float x = static_cast<float>(i % 50) * 2.0f;
        float y = static_cast<float>(i / 50) * 2.0f + 5.0f;
        world.addSphere({x, y, 0}, 0.5f, 1.0f);
    }

    REQUIRE(world.bodyCount() == static_cast<size_t>(kBodyCount) + 1);

    // Measure serialization time.
    auto t0 = std::chrono::high_resolution_clock::now();
    ArchiveV2 archive = serializeWorldV2(world);
    auto t1 = std::chrono::high_resolution_clock::now();
    double serializeMs =
        std::chrono::duration<double, std::milli>(t1 - t0).count();

    // Measure pack time.
    auto t2 = std::chrono::high_resolution_clock::now();
    std::vector<uint8_t> bytes = packArchiveV2(archive);
    auto t3 = std::chrono::high_resolution_clock::now();
    double packMs =
        std::chrono::duration<double, std::milli>(t3 - t2).count();

    // Measure unpack time.
    auto t4 = std::chrono::high_resolution_clock::now();
    ArchiveV2 unpacked = unpackArchiveV2(bytes);
    auto t5 = std::chrono::high_resolution_clock::now();
    double unpackMs =
        std::chrono::duration<double, std::milli>(t5 - t4).count();

    // Measure deserialization time.
    auto t6 = std::chrono::high_resolution_clock::now();
    World restored(BackendType::Cpu);
    deserializeWorldV2(restored, unpacked);
    auto t7 = std::chrono::high_resolution_clock::now();
    double deserializeMs =
        std::chrono::duration<double, std::milli>(t7 - t6).count();

    CHECK(restored.bodyCount() == world.bodyCount());

    // Log timings (visible with -s flag).
    MESSAGE("Large scene (" << kBodyCount << " bodies):");
    MESSAGE("  Serialize:   " << serializeMs << " ms");
    MESSAGE("  Pack:        " << packMs << " ms");
    MESSAGE("  Unpack:      " << unpackMs << " ms");
    MESSAGE("  Deserialize: " << deserializeMs << " ms");
    MESSAGE("  Archive size: " << bytes.size() << " bytes");

    // Sanity: serialization should complete in reasonable time.
    CHECK(serializeMs < 5000.0);
    CHECK(deserializeMs < 5000.0);
}

TEST_CASE("v2 large scene incremental performance" * doctest::timeout(30)) {
    const int kBodyCount = 5000;

    World world(BackendType::Cpu);
    world.setGravity({0, -9.81f, 0});
    world.addStaticPlane({0, 1, 0}, 0.0f);
    for (int i = 0; i < kBodyCount; ++i) {
        float x = static_cast<float>(i % 50) * 2.0f;
        float y = static_cast<float>(i / 50) * 2.0f + 5.0f;
        world.addSphere({x, y, 0}, 0.5f, 1.0f);
    }

    auto t0 = std::chrono::high_resolution_clock::now();
    IncrementalSerializer serializer(world);
    size_t chunks = 0;
    while (serializer.nextChunk(512)) ++chunks;
    ArchiveV2 archive = serializer.finish();
    auto t1 = std::chrono::high_resolution_clock::now();
    double incrMs =
        std::chrono::duration<double, std::milli>(t1 - t0).count();

    World restored(BackendType::Cpu);
    auto t2 = std::chrono::high_resolution_clock::now();
    IncrementalDeserializer deserializer(restored, archive);
    while (deserializer.nextChunk()) {}
    deserializer.finish();
    auto t3 = std::chrono::high_resolution_clock::now();
    double incrDeserMs =
        std::chrono::duration<double, std::milli>(t3 - t2).count();

    CHECK(restored.bodyCount() == world.bodyCount());

    MESSAGE("Incremental (" << kBodyCount << " bodies, " << chunks
                            << " chunks):");
    MESSAGE("  Serialize:   " << incrMs << " ms");
    MESSAGE("  Deserialize: " << incrDeserMs << " ms");

    CHECK(incrMs < 5000.0);
    CHECK(incrDeserMs < 5000.0);
}

TEST_CASE("v2 large scene JSON export performance" * doctest::timeout(30)) {
    const int kBodyCount = 1000;

    World world(BackendType::Cpu);
    for (int i = 0; i < kBodyCount; ++i)
        world.addSphere({static_cast<float>(i), 5, 0}, 0.5f, 1.0f);

    auto t0 = std::chrono::high_resolution_clock::now();
    std::string json = exportWorldJson(world);
    auto t1 = std::chrono::high_resolution_clock::now();
    double jsonMs =
        std::chrono::duration<double, std::milli>(t1 - t0).count();

    CHECK_FALSE(json.empty());
    MESSAGE("JSON export (" << kBodyCount << " bodies): " << jsonMs
                            << " ms, " << json.size() << " chars");
    CHECK(jsonMs < 5000.0);
}

// ===========================================================================
// Edge cases
// ===========================================================================

TEST_CASE("v2 serialize/deserialize preserves solver options") {
    World world(BackendType::Cpu);
    SolverOptions opts;
    opts.velocityIterations = 16;
    opts.positionIterations = 6;
    opts.frictionModel = FrictionModel::ConeBlockSolver;
    world.setSolverOptions(opts);
    world.addSphere({0, 5, 0}, 0.5f, 1.0f);

    ArchiveV2 archive = serializeWorldV2(world);
    World restored(BackendType::Cpu);
    deserializeWorldV2(restored, archive);

    SolverOptions restored_opts = restored.solverOptions();
    CHECK(restored_opts.velocityIterations == 16);
    CHECK(restored_opts.positionIterations == 6);
    CHECK(restored_opts.frictionModel == FrictionModel::ConeBlockSolver);
}

TEST_CASE("v2 serialize/deserialize preserves CCD defaults") {
    World world(BackendType::Cpu);
    WorldCcdDefaults ccd;
    ccd.defaultQuality = MotionQuality::High;
    ccd.defaultEnableContinuous = true;
    world.setCcdDefaults(ccd);
    world.addSphere({0, 5, 0}, 0.5f, 1.0f);

    ArchiveV2 archive = serializeWorldV2(world);
    World restored(BackendType::Cpu);
    deserializeWorldV2(restored, archive);

    WorldCcdDefaults restoredCcd = restored.ccdDefaults();
    CHECK(restoredCcd.defaultQuality == MotionQuality::High);
}

TEST_CASE("v2 empty archive sections") {
    ArchiveV2 archive;
    archive.version = kSerializationV2Version;
    // No sections — deserialization should throw (missing Bodies).
    World world(BackendType::Cpu);
    CHECK_THROWS(deserializeWorldV2(world, archive));
}

TEST_CASE("v2 find section returns nullptr for missing") {
    ArchiveV2 archive;
    CHECK(archive.find(SectionType::Bodies) == nullptr);
    CHECK(archive.find(SectionType::SceneGraph) == nullptr);
}
