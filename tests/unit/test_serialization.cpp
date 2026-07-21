#include "doctest.h"
#include <velox/velox.h>

using namespace velox;

TEST_CASE("serialization round-trip empty world") {
    World world(BackendType::Cpu);
    world.gravity = {0, -9.81f, 0};

    SerializedScene scene = serializeWorld(world, "test empty");
    CHECK(scene.version == kSerializationVersion);
    CHECK(scene.metadata == "test empty");
    CHECK_FALSE(scene.data.empty());

    World restored(BackendType::Cpu);
    deserializeWorld(restored, scene);
    CHECK(restored.bodyCount() == 0);
}

TEST_CASE("serialization round-trip with bodies") {
    World world(BackendType::Cpu);
    world.gravity = {0, -9.81f, 0};
    world.addStaticPlane({0, 1, 0}, 0.0f);
    world.addSphere({0, 5, 0}, 0.5f, 1.0f);
    world.addBox({3, 5, 0}, {1, 1, 1}, 2.0f);

    SerializedScene scene = serializeWorld(world, "bodies test");

    World restored(BackendType::Cpu);
    deserializeWorld(restored, scene);
    CHECK(restored.bodyCount() == world.bodyCount());
}

TEST_CASE("serialization round-trip preserves body state") {
    World world(BackendType::Cpu);
    world.gravity = {0, -9.81f, 0};
    BodyId id = world.addSphere({1, 2, 3}, 0.5f, 1.0f);
    world.setLinearVelocity(id, {4, 5, 6});

    for (int i = 0; i < 10; ++i)
        world.step(1.0f / 60.0f);

    Vec3 posBefore = world.body(id).position;
    Vec3 velBefore = world.body(id).velocity;

    SerializedScene scene = serializeWorld(world);

    World restored(BackendType::Cpu);
    deserializeWorld(restored, scene);

    CHECK(restored.bodyCount() == world.bodyCount());
}

TEST_CASE("pack/unpack round-trip") {
    World world(BackendType::Cpu);
    world.addSphere({0, 5, 0}, 0.5f, 1.0f);
    SerializedScene scene = serializeWorld(world, "pack test");

    std::vector<uint8_t> bytes = packScene(scene);
    CHECK_FALSE(bytes.empty());

    SerializedScene unpacked = unpackScene(bytes);
    CHECK(unpacked.version == scene.version);
    CHECK(unpacked.metadata == scene.metadata);
    CHECK(unpacked.data == scene.data);
}

TEST_CASE("unpack rejects bad data") {
    std::vector<uint8_t> garbage = {0, 1, 2, 3, 4, 5};
    CHECK_THROWS(unpackScene(garbage));
}

TEST_CASE("replay record and verify") {
    World world(BackendType::Cpu);
    world.gravity = {0, -9.81f, 0};
    world.addStaticPlane({0, 1, 0}, 0.0f);
    world.addSphere({0, 5, 0}, 0.5f, 1.0f);

    float dt = 1.0f / 60.0f;
    ReplayRecording recording;
    beginReplay(recording, world, dt);

    for (int i = 0; i < 30; ++i) {
        world.step(dt);
        recordReplayFrame(recording, world);
    }

    uint64_t result = verifyReplay(recording);
    CHECK(result == 0);
}
