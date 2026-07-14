// serialization.h — Design sketch for versioned scene serialization and replay.
// Self-contained: includes only what's needed for compilation in isolation.
// TODO: integrate with velox::World, velox::BodyId, velox::JointId, velox::Vec3, velox::Quat.

#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <memory>

// Forward declarations — replace with #include "velox/world.h" etc. when integrating.
namespace velox {
    struct Vec3;
    struct Quat;
    class World;
    using BodyId = uint64_t;
    using JointId = uint64_t;
}

namespace velox {

// Serialization format version. Increment when the binary layout changes in
// an incompatible way. The reader rejects files with mismatched versions.
inline constexpr uint32_t kSerializationVersion = 1;

// Compression mode for network snapshots. Trade-off between file size and
// decode speed.
enum class CompressionMode : uint8_t {
    None = 0,           // raw binary, fastest decode
    LZ4 = 1,            // fast compression, good for network
    ZSTD = 2            // better ratio, slightly slower decode
};

// Serialized body state: compact representation of a Body's relevant fields.
struct SerializedBody {
    uint64_t id;                    // stable BodyId (slot | generation)
    Vec3 position{};
    Quat orientation{};
    Vec3 velocity{};
    Vec3 angularVelocity{};
    float invMass = 0.0f;
    Vec3 invInertia{};
    Quat inertiaOrientation{};
    uint8_t motionType;             // enum class MotionType as uint8_t
    uint8_t shapeType;              // enum class ShapeType as uint8_t
    float radius = 0.0f;
    Vec3 halfExtents{};
    float capsuleHalfHeight = 0.0f;
    uint32_t meshIndex = UINT32_MAX;
    uint32_t hullFirst = 0;
    uint32_t hullCount = 0;
    uint32_t compoundFirst = 0;
    uint32_t compoundCount = 0;
};

// Serialized joint state.
struct SerializedJoint {
    uint64_t id;                    // stable JointId
    uint8_t type;                   // enum class JointType as uint8_t
    uint64_t bodyA;                 // BodyId of anchor A
    uint64_t bodyB;                 // BodyId of anchor B (may be static)
    Vec3 localAnchorA{};
    Vec3 localAnchorB{};
    Vec3 localAxisA{};
    Vec3 localAxisB{};
    float restLength = 0.0f;
    bool collideConnected = false;
    // Motor/limit/spring fields omitted for brevity — include if needed.
};

// Serialized scene: a complete snapshot of a World's state at a point in time.
struct SerializedScene {
    uint32_t version = kSerializationVersion;
    Vec3 gravity{};
    int substeps = 4;
    float dt = 0.0f;                // timestep used when this was captured
    std::vector<SerializedBody> bodies;
    std::vector<SerializedJoint> joints;
    std::string metadata;           // arbitrary user data (e.g., scene name)
};

// Replay recording: captures per-frame state for deterministic replay.
struct ReplayFrame {
    uint64_t frameNumber;
    float simulationTime;
    std::vector<SerializedBody> bodyStates;  // only tracked bodies
};

struct ReplayRecording {
    std::vector<ReplayFrame> frames;
    Vec3 gravity{};
    int substeps = 4;
    float dt = 0.0f;
};

// Serialize a World's state to a SerializedScene.
//
// TODO: implement by iterating over all bodies and joints, copying relevant
//       fields into the serialized structs. Handle compound shapes by expanding
//       children into separate SerializedBody entries (with parent reference).
SerializedScene SerializeWorld(const World& world);

// Deserialize a SerializedScene back into a World. Removes all existing bodies
// and joints before populating from the scene.
//
// TODO: implement by clearing the world, then adding bodies/joints from the
//       serialized data. Validate version match; throw on mismatch.
void DeserializeWorld(World& world, const SerializedScene& scene);

// Compress a serialized scene for network transmission or disk storage.
// Returns the compressed bytes; size depends on CompressionMode.
//
// TODO: integrate with LZ4 and ZSTD libraries. For None mode, just return
//       a header + raw scene bytes.
std::vector<uint8_t> CompressScene(const SerializedScene& scene, CompressionMode mode);

// Decompress bytes back into a SerializedScene.
std::vector<SerializedScene> DecompressScene(const std::vector<uint8_t>& data);

// Record the current world state as a replay frame. Call after each step().
void RecordReplayFrame(ReplayRecording& recording, const World& world, uint64_t frameNumber);

// Replay a recorded scene deterministically. Runs the simulation with the
// recorded gravity/substeps/dt and compares each frame's body states against
// the recording. Returns true if all frames match within tolerance.
//
// TODO: implement by creating a new World, deserializing frame 0, then stepping
//       and comparing each subsequent frame. Tolerance: position < 1e-5, velocity < 1e-4.
bool ReplayRecording(const ReplayRecording& recording, const World& world);

} // namespace velox
