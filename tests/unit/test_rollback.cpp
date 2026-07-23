#include "doctest.h"
#include <velox/velox.h>
#include <velox/rollback.h>

#include <cstring>

using namespace velox;

namespace {

// World is neither copyable nor movable (owns a unique_ptr backend and an
// std::atomic), so scenes are built in place on a caller-owned World rather
// than returned by value.
void populateFallingStack(World& world, BodyId* firstBody = nullptr) {
    world.gravity = {0, -9.81f, 0};
    world.addStaticPlane({0, 1, 0}, 0.0f);
    for (int i = 0; i < 4; ++i) {
        BodyId id = world.addBox({0, 2.0f + float(i) * 1.05f, 0}, {0.5f, 0.5f, 0.5f}, 1.0f);
        if (i == 0 && firstBody) *firstBody = id;
    }
}

} // namespace

TEST_CASE("canonical hash is stable across identical replays") {
    World a(BackendType::Cpu);
    World b(BackendType::Cpu);
    populateFallingStack(a);
    populateFallingStack(b);
    for (int i = 0; i < 30; ++i) { a.step(1.0f / 60.0f); b.step(1.0f / 60.0f); }
    CHECK(computeCanonicalHash(a) == computeCanonicalHash(b));
}

TEST_CASE("canonical hash changes once bodies diverge") {
    World a(BackendType::Cpu);
    World b(BackendType::Cpu);
    BodyId aFirst{}, bFirst{};
    populateFallingStack(a, &aFirst);
    populateFallingStack(b, &bFirst);
    for (int i = 0; i < 5; ++i) { a.step(1.0f / 60.0f); b.step(1.0f / 60.0f); }
    b.setLinearVelocity(bFirst, {1.0f, 0.0f, 0.0f});
    for (int i = 0; i < 5; ++i) { a.step(1.0f / 60.0f); b.step(1.0f / 60.0f); }
    CHECK(computeCanonicalHash(a) != computeCanonicalHash(b));
}

TEST_CASE("delta encode/decode round-trip reconstructs target exactly") {
    World world(BackendType::Cpu);
    populateFallingStack(world);
    std::vector<uint8_t> base = captureCanonicalBodyState(world);
    for (int i = 0; i < 10; ++i) world.step(1.0f / 60.0f);
    std::vector<uint8_t> target = captureCanonicalBodyState(world);

    SnapshotDelta delta = encodeDelta(base, target);
    std::vector<uint8_t> reconstructed = applyDelta(base, delta);
    CHECK(reconstructed == target);
}

TEST_CASE("delta of identical states has no changed bodies") {
    World world(BackendType::Cpu);
    populateFallingStack(world);
    std::vector<uint8_t> state = captureCanonicalBodyState(world);
    SnapshotDelta delta = encodeDelta(state, state);
    CHECK(changedBodyCount(delta) == 0);
    CHECK(applyDelta(state, delta) == state);
}

TEST_CASE("delta chain across multiple frames reconstructs each frame") {
    World world(BackendType::Cpu);
    populateFallingStack(world);
    std::vector<std::vector<uint8_t>> frames;
    frames.push_back(captureCanonicalBodyState(world));
    for (int i = 0; i < 20; ++i) {
        world.step(1.0f / 60.0f);
        frames.push_back(captureCanonicalBodyState(world));
    }

    std::vector<uint8_t> reconstructed = frames.front();
    for (size_t i = 1; i < frames.size(); ++i) {
        SnapshotDelta delta = encodeDelta(frames[i - 1], frames[i]);
        reconstructed = applyDelta(reconstructed, delta);
        CHECK(reconstructed == frames[i]);
    }
}

TEST_CASE("encodeDelta rejects mismatched body counts") {
    World small(BackendType::Cpu);
    small.addSphere({0, 5, 0}, 0.5f, 1.0f);
    World large(BackendType::Cpu);
    populateFallingStack(large);

    std::vector<uint8_t> a = captureCanonicalBodyState(small);
    std::vector<uint8_t> b = captureCanonicalBodyState(large);
    CHECK_THROWS(encodeDelta(a, b));
}

TEST_CASE("applyDelta rejects truncated changedRecords") {
    World world(BackendType::Cpu);
    populateFallingStack(world);
    std::vector<uint8_t> base = captureCanonicalBodyState(world);
    world.step(1.0f / 60.0f);
    std::vector<uint8_t> target = captureCanonicalBodyState(world);

    SnapshotDelta delta = encodeDelta(base, target);
    REQUIRE(changedBodyCount(delta) > 0);
    delta.changedRecords.resize(delta.changedRecords.size() - 1); // corrupt: truncate
    CHECK_THROWS(applyDelta(base, delta));
}

TEST_CASE("applyDelta rejects base/delta body count mismatch") {
    World world(BackendType::Cpu);
    populateFallingStack(world);
    std::vector<uint8_t> base = captureCanonicalBodyState(world);
    world.step(1.0f / 60.0f);
    std::vector<uint8_t> target = captureCanonicalBodyState(world);
    SnapshotDelta delta = encodeDelta(base, target);

    std::vector<uint8_t> wrongSizedBase = base;
    wrongSizedBase.resize(wrongSizedBase.size() / 2);
    CHECK_THROWS(applyDelta(wrongSizedBase, delta));
}

TEST_CASE("RollbackBuffer restores an earlier frame bitwise exactly") {
    World world(BackendType::Cpu);
    populateFallingStack(world);
    RollbackBuffer buffer(8);

    buffer.push(0, world);
    std::vector<uint8_t> frame0 = captureCanonicalBodyState(world);

    for (uint64_t frame = 1; frame <= 5; ++frame) {
        world.step(1.0f / 60.0f);
        buffer.push(frame, world);
    }

    REQUIRE(buffer.restore(0, world));
    CHECK(captureCanonicalBodyState(world) == frame0);
}

TEST_CASE("RollbackBuffer evicts oldest frames once capacity is exceeded") {
    World world(BackendType::Cpu);
    populateFallingStack(world);
    RollbackBuffer buffer(3);

    for (uint64_t frame = 0; frame < 10; ++frame) {
        world.step(1.0f / 60.0f);
        buffer.push(frame, world);
    }

    CHECK(buffer.size() == 3);
    CHECK(buffer.oldestFrame() == 7);
    CHECK(buffer.newestFrame() == 9);
    CHECK_FALSE(buffer.contains(6));
    CHECK(buffer.contains(7));

    World other(BackendType::Cpu);
    populateFallingStack(other);
    CHECK_FALSE(buffer.restore(6, other));
}

TEST_CASE("RollbackBuffer rejects zero capacity") {
    CHECK_THROWS(RollbackBuffer(0));
}

TEST_CASE("findFirstDivergence reports no divergence for identical recordings") {
    World world(BackendType::Cpu);
    populateFallingStack(world);
    ReplayRecording recording;
    beginReplay(recording, world, 1.0f / 60.0f);
    for (int i = 0; i < 20; ++i) {
        world.step(1.0f / 60.0f);
        recordReplayFrame(recording, world);
    }
    DivergenceReport report = findFirstDivergence(recording, recording);
    CHECK_FALSE(report.diverged);
}

TEST_CASE("findFirstDivergence pinpoints frame, body, and field") {
    World world(BackendType::Cpu);
    populateFallingStack(world);
    ReplayRecording expected;
    beginReplay(expected, world, 1.0f / 60.0f);
    for (int i = 0; i < 20; ++i) {
        world.step(1.0f / 60.0f);
        recordReplayFrame(expected, world);
    }

    ReplayRecording actual = expected;
    REQUIRE(actual.frames.size() > 10);
    // Corrupt one float (the first field of body 2's record) at frame index 10.
    std::memset(actual.frames[10].data() + 2 * 52, 0x7f, sizeof(float));

    DivergenceReport report = findFirstDivergence(expected, actual);
    CHECK(report.diverged);
    CHECK(report.frame == 11);
    CHECK(report.bodyIndex == 2);
    CHECK(report.field == "position");
}

TEST_CASE("findFirstDivergence reports a body-count mismatch") {
    World world(BackendType::Cpu);
    populateFallingStack(world);
    ReplayRecording expected;
    beginReplay(expected, world, 1.0f / 60.0f);
    world.step(1.0f / 60.0f);
    recordReplayFrame(expected, world);

    ReplayRecording actual = expected;
    actual.frames[0].resize(actual.frames[0].size() - 52); // drop one body's record

    DivergenceReport report = findFirstDivergence(expected, actual);
    CHECK(report.diverged);
    CHECK(report.field == "bodyCount");
}

TEST_CASE("bitwise strict replay is identical across worker counts") {
    World serial(BackendType::Cpu);
    serial.setWorkerCount(1);
    World parallel(BackendType::Cpu);
    parallel.setWorkerCount(0); // auto/all workers

    for (World* w : {&serial, &parallel}) {
        w->gravity = {0, -9.81f, 0};
        w->addStaticPlane({0, 1, 0}, 0.0f);
        for (int i = 0; i < 12; ++i) {
            const int x = i % 3, y = i / 3;
            BodyId id = w->addBox({float(x - 1) * 0.75f, 0.5f + float(y) * 1.02f, 0}, {0.35f, 0.5f, 0.35f}, 1.0f);
            w->setAngularVelocity(id, {0, 0.15f * float(i + 1), 0});
        }
    }

    for (int i = 0; i < 60; ++i) { serial.step(1.0f / 60.0f); parallel.step(1.0f / 60.0f); }

    CHECK(computeCanonicalHash(serial) == computeCanonicalHash(parallel));
}
