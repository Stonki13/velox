#pragma once
// Serialization V2 — production-grade scene persistence for Velox.
//
// Improvements over the V1 format (serialization.h):
//   • Versioned container with explicit migration path (V1 scenes load
//     through a compatibility adapter).
//   • CRC-32 integrity checksums on every section and the whole archive.
//   • Section-based layout enabling incremental (chunked) serialization
//     for large worlds without a full monolithic snapshot.
//   • Optional zlib / zstd compression per section (compile-time gated;
//     the core library ships dependency-free with None always available).
//   • JSON export for debugging and tooling interop.
//   • Scene-graph export for visual debugging pipelines.
//
// The V2 format is self-describing: each section carries a type tag,
// element-size validation, and a byte count so unknown sections can be
// safely skipped by older readers.

#include "world.h"

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace velox {

// ---------------------------------------------------------------------------
// Format constants
// ---------------------------------------------------------------------------

/// Magic for V2 archives: 'VLX2' in little-endian.
inline constexpr uint32_t kSerializationV2Magic = 0x32584C56u;

/// Current V2 format version. Bumped on any breaking layout change.
inline constexpr uint32_t kSerializationV2Version = 1;

/// Minimum V2 version this build can read (for forward-compat checks).
inline constexpr uint32_t kSerializationV2MinVersion = 1;

// ---------------------------------------------------------------------------
// Compression
// ---------------------------------------------------------------------------

/// Compression codec applied to individual sections.
enum class CompressionV2 : uint8_t {
    None = 0, ///< Raw bytes (always available).
    Zlib = 1, ///< DEFLATE via zlib (requires VELOX_WITH_ZLIB).
    Zstd = 2, ///< Zstandard (requires VELOX_WITH_ZSTD).
};

/// Returns true when the codec is compiled into this build.
bool isCompressionAvailable(CompressionV2 mode);

/// Compress / decompress a byte buffer. Throws when the codec is unavailable.
std::vector<uint8_t> compressBytes(const std::vector<uint8_t>& input,
                                   CompressionV2 mode, int level = -1);
std::vector<uint8_t> decompressBytes(const std::vector<uint8_t>& input,
                                     CompressionV2 mode,
                                     size_t uncompressedSize);

// ---------------------------------------------------------------------------
// CRC-32 integrity
// ---------------------------------------------------------------------------

/// CRC-32 (ISO 3309 / ITU-T V.42) used for corruption detection.
uint32_t crc32(const void* data, size_t size, uint32_t seed = 0);

// ---------------------------------------------------------------------------
// Section types
// ---------------------------------------------------------------------------

/// Well-known section identifiers. Unknown tags are skipped on read so
/// newer writers can add sections without breaking older readers.
enum class SectionType : uint32_t {
    Header      = 0x00000001u,
    Bodies      = 0x00000002u,
    Joints      = 0x00000003u,
    Contacts    = 0x00000004u,
    Meshes      = 0x00000005u,
    Settings    = 0x00000006u,
    Metadata    = 0x00000007u,
    SceneGraph  = 0x00000008u,
    Custom      = 0x80000000u, ///< First user-defined section tag.
};

// ---------------------------------------------------------------------------
// Archive container
// ---------------------------------------------------------------------------

/// A single named section inside a V2 archive.
struct ArchiveSection {
    SectionType type = SectionType::Custom;
    CompressionV2 compression = CompressionV2::None;
    uint32_t crc = 0;                  ///< CRC-32 of the *uncompressed* payload.
    std::vector<uint8_t> payload;      ///< Possibly compressed bytes.
    size_t uncompressedSize = 0;       ///< Original size before compression.
};

/// Complete V2 archive: header + ordered sections.
struct ArchiveV2 {
    uint32_t version = kSerializationV2Version;
    std::string metadata;              ///< Free-form UTF-8 (scene name, tool…).
    std::vector<ArchiveSection> sections;

    /// Find the first section with the given type, or nullptr.
    const ArchiveSection* find(SectionType type) const;
    ArchiveSection* find(SectionType type);
};

// ---------------------------------------------------------------------------
// Serialization options
// ---------------------------------------------------------------------------

/// Controls what and how to serialize.
struct SerializationOptions {
    CompressionV2 compression = CompressionV2::None;
    int compressionLevel = -1;         ///< Codec default when -1.
    bool includeContacts = true;       ///< Warm-start contact manifolds.
    bool includeMeshes = true;         ///< Triangle-soup / hull geometry.
    bool computeCrc = true;            ///< Per-section + archive CRC.
    std::string metadata;              ///< Free-form scene metadata.
};

// ---------------------------------------------------------------------------
// Core serialize / deserialize
// ---------------------------------------------------------------------------

/// Serialize the complete world state into a V2 archive.
ArchiveV2 serializeWorldV2(const World& world,
                           const SerializationOptions& opts = {});

/// Restore a world from a V2 archive. Replaces the world's entire contents.
/// Throws on version mismatch, CRC failure, or truncated data.
void deserializeWorldV2(World& world, const ArchiveV2& archive);

// ---------------------------------------------------------------------------
// Binary pack / unpack (byte-stream container)
// ---------------------------------------------------------------------------

/// Pack an archive into a flat byte stream suitable for files / network.
/// Layout: magic | version | flags | sectionCount | sections… | archiveCRC
std::vector<uint8_t> packArchiveV2(const ArchiveV2& archive);

/// Unpack a byte stream produced by packArchiveV2. Validates magic, version,
/// and CRC. Throws on any integrity failure.
ArchiveV2 unpackArchiveV2(const std::vector<uint8_t>& bytes);

// ---------------------------------------------------------------------------
// V1 backward compatibility
// ---------------------------------------------------------------------------

/// Load a V1 SerializedScene (from serialization.h) into a V2 archive.
/// The V1 blob is stored in a Bodies section with a migration marker so
/// deserializeWorldV2 can route it through the legacy decoder.
ArchiveV2 migrateV1ToV2(const struct SerializedScene& v1);

/// True when the archive was produced by the V1 migration path.
bool isMigratedV1(const ArchiveV2& archive);

// ---------------------------------------------------------------------------
// Incremental serialization (large worlds)
// ---------------------------------------------------------------------------

/// Chunked writer: serializes bodies in batches so a 100 k-body world never
/// needs a single monolithic snapshot allocation.
class IncrementalSerializer {
public:
    explicit IncrementalSerializer(const World& world,
                                   const SerializationOptions& opts = {});

    /// Serialize the next batch of up to `maxBodies` bodies.
    /// Returns false when all bodies have been emitted.
    bool nextChunk(size_t maxBodies = 4096);

    /// Finalize: writes remaining sections (joints, meshes, settings) and
    /// returns the complete archive.
    ArchiveV2 finish();

    /// Number of bodies serialized so far.
    size_t bodiesSerialized() const { return bodiesDone_; }

private:
    const World& world_;
    SerializationOptions opts_;
    std::vector<ArchiveSection> bodyChunks_;
    size_t bodiesDone_ = 0;
    bool finished_ = false;
};

/// Chunked reader: restores bodies incrementally from a V2 archive.
class IncrementalDeserializer {
public:
    explicit IncrementalDeserializer(World& world, const ArchiveV2& archive);

    /// Restore the next batch of bodies. Returns false when done.
    bool nextChunk();

    /// Finalize: restores joints, meshes, settings, and validates CRC.
    void finish();

private:
    World& world_;
    const ArchiveV2& archive_;
    size_t chunkIndex_ = 0;
    bool finished_ = false;
};

// ---------------------------------------------------------------------------
// JSON export (debugging / tooling)
// ---------------------------------------------------------------------------

/// Export the world state as a human-readable JSON string.
/// Intended for debugging, diffing, and external tooling — not for
/// round-trip fidelity (floats are printed with limited precision).
std::string exportWorldJson(const World& world, int indent = 2);

/// Export a V2 archive's structure (section table, sizes, CRCs) as JSON.
std::string exportArchiveJson(const ArchiveV2& archive, int indent = 2);

// ---------------------------------------------------------------------------
// Scene-graph export (debugging)
// ---------------------------------------------------------------------------

/// A node in the exported scene graph. Bodies are leaves; joints define
/// parent-child edges. The graph is rooted at a synthetic "World" node.
struct SceneGraphNode {
    std::string name;                  ///< e.g. "Body_42 (Sphere, dynamic)"
    std::string type;                  ///< "body", "joint", "mesh", "world"
    Vec3 position;
    Quat orientation;
    std::vector<size_t> children;      ///< Indices into the flat node array.
    // Body-specific
    uint64_t bodyId = UINT64_MAX;
    std::string shapeName;
    float mass = 0.0f;
    bool isStatic = false;
    bool isSleeping = false;
    // Joint-specific
    uint64_t jointId = UINT64_MAX;
    std::string jointType;
};

/// Build a flat scene-graph representation of the world.
/// Node 0 is always the synthetic root ("World").
std::vector<SceneGraphNode> exportSceneGraph(const World& world);

/// Serialize the scene graph to a JSON string for debugging pipelines.
std::string sceneGraphToJson(const std::vector<SceneGraphNode>& graph,
                             int indent = 2);

/// Serialize the scene graph to a DOT (Graphviz) string.
std::string sceneGraphToDot(const std::vector<SceneGraphNode>& graph);

} // namespace velox
