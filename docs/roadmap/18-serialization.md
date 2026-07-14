# 18 — Serialization

## Goal

Provide versioned scene save/load, deterministic replay recording, and network snapshot compression. Scenes are serialized as binary blobs with a version header; mismatched versions are rejected. Replay recordings capture per-frame body states for debugging and demo sharing. Network snapshots use LZ4/ZSTD compression for bandwidth efficiency.

## Public API

```cpp
namespace velox {

inline constexpr uint32_t kSerializationVersion = 1;

enum class CompressionMode : uint8_t { None = 0, LZ4 = 1, ZSTD = 2 };

struct SerializedScene {
    uint32_t version = kSerializationVersion;
    Vec3 gravity{};
    int substeps = 4;
    float dt = 0.0f;
    std::vector<uint8_t> bodyData;      // flat binary: SerializedBody structs
    std::vector<uint8_t> jointData;     // flat binary: SerializedJoint structs
    std::string metadata;
};

struct ReplayFrame {
    uint64_t frameNumber;
    float simulationTime;
    std::vector<uint8_t> bodyStates;    // compressed per-frame snapshot
};

struct ReplayRecording {
    std::vector<ReplayFrame> frames;
    Vec3 gravity{};
    int substeps = 4;
    float dt = 0.0f;
};

// Save/Load
SerializedScene serializeWorld(const World& world);
void deserializeWorld(World& world, const SerializedScene& scene);

// Compression
std::vector<uint8_t> compressScene(const SerializedScene& scene, CompressionMode mode);
SerializedScene decompressScene(const std::vector<uint8_t>& data);

// Replay
void recordReplayFrame(ReplayRecording& recording, const World& world, uint64_t frame);
bool verifyReplay(const ReplayRecording& recording, const World& world);

} // namespace velox
```

## Data structures

- `SerializedScene`, `ReplayFrame`, `ReplayRecording` — new file `include/velox/serialization.h`.
- `CompressionMode` enum — new file `include/velox/serialization.h`.
- New World methods: `serializeWorld()`, `deserializeWorld()`, `compressScene()`, `decompressScene()`, `recordReplayFrame()`, `verifyReplay()`.

## Algorithm

**Serialization format:**

1. **Header (32 bytes):** magic number (`VELOX`), version uint32, gravity (12 bytes), substeps (4 bytes), dt (4 bytes), body data length (8 bytes), joint data length (8 bytes).
2. **Body data:** contiguous array of `SerializedBody` structs (POD, no pointers). Each struct contains: BodyId, position (12B), orientation (16B), velocity (12B), angularVelocity (12B), invMass (4B), invInertia (12B), inertiaOrientation (16B), motionType (1B), shapeType (1B), radius (4B), halfExtents (12B), capsuleHalfHeight (4B), meshIndex (4B), hullFirst/Count (8B), compoundFirst/Count (8B).
3. **Joint data:** contiguous array of `SerializedJoint` structs. Each struct contains: JointId, type (1B), bodyA/B (16B each), localAnchorA/B (24B total), localAxisA/B (24B total), restLength (4B), collideConnected (1B).
4. **Metadata:** null-terminated UTF-8 string after joint data.

**Compression:**

1. `None`: header + raw body/joint data + metadata. No transformation.
2. `LZ4`: LZ4 frame format around the serialized scene bytes. Fast encode/decode, ~3-5× compression for typical scenes.
3. `ZSTD`: ZSTD medium compression around the serialized scene bytes. ~5-10× compression, slightly slower decode.

**Replay verification:**

1. Create a fresh World with the recording's gravity/substeps/dt.
2. Deserialize frame 0 into the world.
3. For each subsequent frame: step the world by `dt`, compare body positions/velocities against the recorded frame (tolerance: position < 1e-5 m, velocity < 1e-4 m/s).
4. If all frames match, return true; otherwise return false with the first mismatching frame number.

## Files

- `include/velox/serialization.h` — new header
- `src/serialization.cpp` — implementation (serialize, deserialize, compress, decompress)
- `tests/serialization.cpp` — test file
- Third-party: LZ4 and ZSTD libraries (document as optional dependencies)

## Tests

1. **Round-trip fidelity:** Serialize a world with 100 bodies and 50 joints; deserialize into a new world; verify all body positions, orientations, velocities match within 1e-10 (exact binary equality for POD data).
2. **Version mismatch rejection:** Attempt to deserialize a version 2 scene into a version 1 reader. Must throw or return error, not corrupt state.
3. **Compression ratio:** 100-body scene serializes to ~50 KB raw; LZ4 compresses to ~12 KB; ZSTD to ~6 KB. Measured on reference hardware.
4. **Replay verification:** Record 60 frames of a falling-box demo; verify replay returns true. Modify frame 30's gravity; verify replay returns false at frame 31.

## Acceptance

- [ ] `serializeWorld()` produces a valid SerializedScene with correct version header
- [ ] `deserializeWorld()` reconstructs the world exactly (binary equality)
- [ ] Version mismatch is detected and rejected safely
- [ ] LZ4 compression achieves ≥ 3× ratio on typical scenes
- [ ] Replay verification correctly detects modified recordings

## Size: M

## Risks

- Binary serialization is fragile: adding/removing fields breaks older saves. Must enforce strict versioning and provide migration tools for major version bumps.
- LZ4/ZSTD are external dependencies. Must document build requirements and provide fallback to `None` mode if libraries are unavailable.
- Replay verification assumes deterministic simulation; any non-determinism (e.g., multi-threaded solver ordering) will cause false mismatches. Must ensure the serialized world uses a single-threaded solver path for replay.
