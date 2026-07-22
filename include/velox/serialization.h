#pragma once
// Versioned binary scene save/load and deterministic replay (roadmap 18).
//
// A serialized scene is the complete simulation state — bodies, joints,
// meshes/hulls, sleeping state, and persistent-contact warm-start data — so
// a deserialized world continues bitwise identically to the original (CPU
// backend). The format is platform-native binary with a magic/version/layout
// header; mismatches are rejected with an exception, never partial state.
#include "world.h"

#include <cstdint>
#include <string>
#include <vector>

namespace velox {

inline constexpr uint32_t kSerializationVersion = 2;

// LZ4/ZSTD are reserved for builds that link the external codecs; the core
// library ships dependency-free and always supports None.
enum class CompressionMode : uint8_t { None = 0, LZ4 = 1, ZSTD = 2 };

struct SerializedScene {
    uint32_t version = kSerializationVersion;
    std::string metadata;      // free-form UTF-8 (scene name, tool info...)
    std::vector<uint8_t> data; // opaque versioned state blob
};

// Complete state capture / restore. Deserializing replaces the target
// world's entire contents (bodies, joints, meshes, events, warm start).
SerializedScene serializeWorld(const World& world, std::string metadata = {});
void deserializeWorld(World& world, const SerializedScene& scene);

// Byte-stream container for files/network. Throws on unknown compression,
// bad magic, or version mismatch.
std::vector<uint8_t> packScene(const SerializedScene& scene,
                               CompressionMode mode = CompressionMode::None);
SerializedScene unpackScene(const std::vector<uint8_t>& bytes);

// Deterministic replay: an initial full scene plus per-frame dense body
// transforms/velocities. Record on the CPU backend for bitwise verification.
struct ReplayRecording {
    SerializedScene initialScene;
    float dt = 1.0f / 60.0f;
    std::vector<std::vector<uint8_t>> frames;
};

void beginReplay(ReplayRecording& recording, const World& world, float dt);
void recordReplayFrame(ReplayRecording& recording, const World& world);

// Re-simulates the recording on a fresh CPU world and compares every frame.
// Returns 0 when the recording verifies, otherwise the 1-based index of the
// first mismatching frame.
uint64_t verifyReplay(const ReplayRecording& recording,
                      float positionTolerance = 1e-5f,
                      float velocityTolerance = 1e-4f);

} // namespace velox
