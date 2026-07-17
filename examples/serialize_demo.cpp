// Serialization (roadmap 18) verification: bitwise round trip, resume
// determinism, container pack/unpack, version rejection, and replay
// verification incl. tamper detection.
#include <velox/velox.h>

#include <cstdio>
#include <cstring>
#include <vector>

using namespace velox;

namespace {

int failures = 0;

void check(bool condition, const char* label) {
    std::printf("%s: %s\n", condition ? "PASS" : "FAIL", label);
    if (!condition) ++failures;
}

std::vector<BodyId> buildScene(World& world) {
    std::vector<BodyId> bodies;
    world.addStaticPlane({0, 1, 0}, 0);
    for (int i = 0; i < 40; ++i)
        bodies.push_back(world.addBox({static_cast<float>(i % 5), 0.6f + 1.05f * (i / 5),
                                       static_cast<float>(i % 3)},
                                      {0.4f, 0.4f, 0.4f}, 1.0f));
    for (int i = 0; i < 20; ++i)
        bodies.push_back(world.addSphere({-4.0f + 0.4f * i, 3.0f, 2.0f}, 0.3f, 1.0f));
    bodies.push_back(world.addConvexHull({0, 6, -3},
        {{0.4f, 0, 0}, {-0.4f, 0, 0}, {0, 0.5f, 0}, {0, -0.5f, 0},
         {0, 0, 0.45f}, {0, 0, -0.45f}, {0.3f, 0.3f, 0.3f}}, 1.2f));
    bodies.push_back(world.addCapsule({2, 6, -3}, 0.2f, 0.3f, 1.0f));
    world.addBallJoint(bodies[0], bodies[1], {0.0f, 1.1f, 0.0f});
    world.addHingeJoint(bodies[5], bodies[6], {0.0f, 2.0f, 0.0f}, {0, 0, 1});
    return bodies;
}

bool sameBodyState(const World& a, const World& b, const std::vector<BodyId>& ids) {
    for (BodyId id : ids) {
        const Body& x = a.body(id);
        const Body& y = b.body(id);
        if (std::memcmp(&x.position, &y.position, sizeof(Vec3)) != 0 ||
            std::memcmp(&x.orientation, &y.orientation, sizeof(Quat)) != 0 ||
            std::memcmp(&x.velocity, &y.velocity, sizeof(Vec3)) != 0 ||
            std::memcmp(&x.angularVelocity, &y.angularVelocity, sizeof(Vec3)) != 0)
            return false;
    }
    return true;
}

void testRoundTripAndResume() {
    World original(BackendType::Cpu);
    std::vector<BodyId> ids = buildScene(original);
    for (int frame = 0; frame < 90; ++frame) original.step(1.0f / 60.0f);

    const SerializedScene scene = serializeWorld(original, "roundtrip test");
    World restored(BackendType::Cpu);
    deserializeWorld(restored, scene);
    check(sameBodyState(original, restored, ids),
          "round trip: restored state bitwise identical");

    // Resume determinism: both worlds must continue identically (warm-start
    // contacts and sleep state are part of the serialized state).
    for (int frame = 0; frame < 120; ++frame) {
        original.step(1.0f / 60.0f);
        restored.step(1.0f / 60.0f);
    }
    check(sameBodyState(original, restored, ids),
          "resume: 120 further frames stay bitwise identical");

    // Container round trip.
    const std::vector<uint8_t> packed = packScene(scene);
    const SerializedScene unpacked = unpackScene(packed);
    check(unpacked.metadata == scene.metadata && unpacked.data == scene.data,
          "container: pack/unpack preserves scene");
    std::printf("  scene: %zu bodies, blob %zu KB\n", ids.size(),
                packed.size() / 1024);
}

void testVersionRejection() {
    World world(BackendType::Cpu);
    buildScene(world);
    SerializedScene scene = serializeWorld(world);
    scene.version = kSerializationVersion + 1;
    bool threw = false;
    try {
        World target(BackendType::Cpu);
        deserializeWorld(target, scene);
    } catch (const std::exception&) {
        threw = true;
    }
    check(threw, "version: future version rejected");

    std::vector<uint8_t> packed = packScene(serializeWorld(world));
    packed[0] ^= 0xff; // corrupt magic
    threw = false;
    try {
        unpackScene(packed);
    } catch (const std::exception&) {
        threw = true;
    }
    check(threw, "container: corrupted magic rejected");
}

void testReplay() {
    World world(BackendType::Cpu);
    buildScene(world);
    ReplayRecording recording;
    beginReplay(recording, world, 1.0f / 60.0f);
    for (int frame = 0; frame < 60; ++frame) {
        world.step(1.0f / 60.0f);
        recordReplayFrame(recording, world);
    }
    check(verifyReplay(recording) == 0, "replay: clean recording verifies");

    // Tamper with frame 30: flip an exponent bit of a recorded position
    // component (a low mantissa bit would stay under the tolerance, which
    // is correct behavior for a tolerance-based comparison).
    ReplayRecording tampered = recording;
    tampered.frames[30][7] ^= 0x40;
    const uint64_t mismatch = verifyReplay(tampered);
    std::printf("  tampered replay first mismatch: frame %llu\n",
                static_cast<unsigned long long>(mismatch));
    check(mismatch == 31, "replay: tampering detected at the exact frame");
}

} // namespace

int main() {
    testRoundTripAndResume();
    testVersionRejection();
    testReplay();
    if (failures) {
        std::fprintf(stderr, "serialize_demo: %d failure(s)\n", failures);
        return 1;
    }
    std::puts("serialize_demo: all checks passed");
    return 0;
}
