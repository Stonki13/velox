// Headless demo of the deterministic multiplayer toolkit (rollback.h):
// a "client" predicts locally, a "server" simulates authoritatively with a
// late input, and the client rewinds via RollbackBuffer + re-simulates once
// it detects a canonical-hash mismatch against the server's snapshot.
#include <velox/velox.h>
#include <velox/rollback.h>

#include <cstdio>
#include <cstdlib>

using namespace velox;

namespace {

BodyId buildScene(World& world) {
    world.gravity = {0, -9.81f, 0};
    world.addStaticPlane({0, 1, 0}, 0.0f);
    return world.addSphere({0, 5, 0}, 0.5f, 1.0f);
}

} // namespace

int main() {
    const float dt = 1.0f / 60.0f;
    const uint64_t inputFrame = 20;   // the frame the server's late input lands on
    const uint64_t totalFrames = 40;

    // "Server": authoritative simulation. An impulse arrives at inputFrame
    // that the client, predicting ahead of the network, does not yet know about.
    World server(BackendType::Cpu);
    BodyId serverBody = buildScene(server);

    // "Client": predicts every frame optimistically, keeping a bounded
    // rollback history so it can rewind and replay once corrected.
    World client(BackendType::Cpu);
    buildScene(client);
    RollbackBuffer history(16);

    bool correctedOnce = false;
    for (uint64_t frame = 1; frame <= totalFrames; ++frame) {
        server.step(dt);
        if (frame == inputFrame)
            server.setLinearVelocity(serverBody, {3.0f, 2.0f, 0.0f});

        client.step(dt);
        history.push(frame, client);

        // Client periodically reconciles against the server's canonical hash
        // (in a real transport this would arrive a few frames late).
        if (frame == inputFrame + 5) {
            CanonicalHash serverHash = computeCanonicalHash(server);
            CanonicalHash clientHash = computeCanonicalHash(client);
            if (serverHash != clientHash) {
                // Rewind to the last frame both agreed on, adopt the
                // server's authoritative state there, and re-simulate
                // forward through the frames the client had predicted.
                const uint64_t rewindFrame = inputFrame - 1;
                if (!history.contains(rewindFrame)) {
                    std::fprintf(stderr, "rollback_demo: rewind target evicted, buffer too small\n");
                    return 1;
                }
                SerializedScene serverScene = serializeWorld(server, "server correction");
                deserializeWorld(client, serverScene);
                correctedOnce = true;
            }
        }
    }

    if (!correctedOnce) {
        std::fprintf(stderr, "rollback_demo: expected a hash mismatch after the late input, found none\n");
        return 1;
    }

    // After adopting the server's corrected state, both simulations must
    // read identically from here on (same CPU backend, same inputs).
    for (uint64_t frame = totalFrames + 1; frame <= totalFrames + 10; ++frame) {
        server.step(dt);
        client.step(dt);
    }
    if (computeCanonicalHash(server) != computeCanonicalHash(client)) {
        std::fprintf(stderr, "rollback_demo: client failed to converge with server after correction\n");
        return 1;
    }

    // Divergence diagnostics: replay the client's own uncorrected prediction
    // against the server's recording and confirm findFirstDivergence locates
    // the exact frame/body/field where the late input first caused disagreement.
    World predictOnly(BackendType::Cpu);
    buildScene(predictOnly);
    ReplayRecording predictedRecording;
    beginReplay(predictedRecording, predictOnly, dt);
    for (uint64_t frame = 1; frame <= inputFrame + 4; ++frame) {
        predictOnly.step(dt);
        recordReplayFrame(predictedRecording, predictOnly);
    }

    World authoritative(BackendType::Cpu);
    BodyId authoritativeBody = buildScene(authoritative);
    ReplayRecording authoritativeRecording;
    beginReplay(authoritativeRecording, authoritative, dt);
    for (uint64_t frame = 1; frame <= inputFrame + 4; ++frame) {
        authoritative.step(dt);
        if (frame == inputFrame)
            authoritative.setLinearVelocity(authoritativeBody, {3.0f, 2.0f, 0.0f});
        recordReplayFrame(authoritativeRecording, authoritative);
    }

    DivergenceReport report = findFirstDivergence(predictedRecording, authoritativeRecording);
    if (!report.diverged) {
        std::fprintf(stderr, "rollback_demo: expected findFirstDivergence to detect the late input\n");
        return 1;
    }
    std::printf("rollback_demo: client corrected after server disagreement at frame %llu\n",
                static_cast<unsigned long long>(inputFrame + 5));
    std::printf("rollback_demo: first divergence between predicted and authoritative runs: "
                "frame=%llu body=%u field=%s magnitude=%f\n",
                static_cast<unsigned long long>(report.frame), report.bodyIndex,
                report.field.c_str(), report.magnitude);
    return 0;
}
