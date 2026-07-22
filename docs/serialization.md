# Velox Serialization Format Specification

**Version:** 2 (V2)
**Status:** Production
**Header:** `include/velox/serialization_v2.h`
**Implementation:** `src/serialization_v2.cpp`

---

## Overview

Velox provides two serialization generations:

| Feature | V1 (`serialization.h`) | V2 (`serialization_v2.h`) |
|---|---|---|
| Format version | `kSerializationVersion = 2` | `kSerializationV2Version = 1` |
| Container | Flat blob (magic + version + data) | Section-based archive |
| Integrity | None | CRC-32 per section + archive |
| Compression | Reserved (None only) | None / zlib / zstd (compile-gated) |
| Incremental | No | Yes (chunked serializer) |
| JSON export | No | Yes |
| Scene graph | No | Yes (JSON + Graphviz DOT) |
| V1 compat | — | `migrateV1ToV2()` adapter |

V2 wraps the proven V1 encoder as its core payload and adds a production
container layer. V1 scenes load through `migrateV1ToV2()`.

---

## V2 Binary Container Layout

All integers are **little-endian**. Floats are IEEE 754 binary32/binary64
in the platform's native byte order (x86/ARM little-endian).

```
Offset  Size  Field
──────  ────  ─────────────────────────────────────────────
0       4     magic          = 0x32584C56 ('VLX2')
4       4     version        = kSerializationV2Version (currently 1)
8       4     flags          bit 0: archive CRC present
12      8     sectionCount   number of sections (uint64)
20      …     sections[]     sectionCount section records
…       4     archiveCRC     CRC-32 of all preceding bytes
```

### Section Record

```
Offset  Size  Field
──────  ────  ─────────────────────────────────────────────
0       4     type           SectionType tag (uint32)
4       1     compression    CompressionV2 codec (uint8)
5       4     crc            CRC-32 of the *uncompressed* payload
9       8     uncompressedSize  original payload size in bytes
17      8     payloadSize    stored (possibly compressed) size
25      N     payload        payloadSize bytes
```

### Section Types

| Tag | Name | Description |
|---|---|---|
| `0x00000001` | Header | Reserved for future use |
| `0x00000002` | Bodies | Core simulation state (V1 blob or chunk thereof) |
| `0x00000003` | Joints | Reserved (joints are inside the V1 blob) |
| `0x00000004` | Contacts | Reserved (contacts are inside the V1 blob) |
| `0x00000005` | Meshes | Reserved (meshes are inside the V1 blob) |
| `0x00000006` | Settings | Gravity, substeps, CCD, solver options |
| `0x00000007` | Metadata | Free-form UTF-8 string |
| `0x00000008` | SceneGraph | Reserved for scene-graph payload |
| `0x80000000+` | Custom | First user-defined section tag |

Unknown section tags are **skipped** on read, so newer writers can add
sections without breaking older readers.

### Compression Codes

| Value | Codec | Build gate |
|---|---|---|
| `0` | None | Always available |
| `1` | Zlib (DEFLATE) | `VELOX_WITH_ZLIB` |
| `2` | Zstandard | `VELOX_WITH_ZSTD` |

When a codec is not compiled in, `compressBytes()` / `decompressBytes()`
throw `std::runtime_error`. `isCompressionAvailable()` queries at runtime.

---

## CRC-32 Integrity

- Polynomial: `0xEDB88320` (reflected ISO 3309 / ITU-T V.42)
- Initial value: `0xFFFFFFFF`, final XOR: `0xFFFFFFFF`
- Check value for `"123456789"`: `0xCBF43926`

Two levels of protection:

1. **Per-section CRC** — computed over the *uncompressed* payload. Verified
   during `decodeSection()` before the payload is handed to the V1 decoder.
2. **Archive CRC** — computed over the entire packed byte stream (excluding
   the trailing CRC field itself). Verified during `unpackArchiveV2()`.

---

## Settings Section Payload

The Settings section stores world configuration as a fixed sequence of
trivially-copyable PODs:

```
Field                   Type
─────                   ────
gravity                 Vec3 (3 × float32)
substeps                int32
ccdDefaults             WorldCcdDefaults (POD)
multiToiSettings        WorldMultiToiSettings (POD)
solverOptions           SolverOptions (POD)
```

The V1 Bodies blob also carries these values; the explicit Settings section
lets tools patch configuration without re-encoding the full simulation state.

---

## V1 Backward Compatibility

`migrateV1ToV2(const SerializedScene& v1)` wraps a V1 scene in a V2
archive:

- The V1 `data` blob is stored in a **Bodies** section with CRC.
- A **Metadata** section is added containing the marker string
  `"velox-v1-migrated"` followed by the original V1 metadata.
- `isMigratedV1()` detects this marker.

`deserializeWorldV2()` extracts the Bodies section, reconstructs a
`SerializedScene`, and delegates to the V1 `deserializeWorld()`.

---

## Incremental Serialization

For large worlds (10 k+ bodies), `IncrementalSerializer` splits the V1
blob into fixed-size chunks stored as separate Bodies sections:

```cpp
IncrementalSerializer serializer(world, opts);
while (serializer.nextChunk(4096)) { /* stream chunk */ }
ArchiveV2 archive = serializer.finish();
```

`IncrementalDeserializer` reassembles the chunks and restores the world:

```cpp
IncrementalDeserializer deserializer(world, archive);
while (deserializer.nextChunk()) { /* process chunk */ }
deserializer.finish();
```

Each chunk carries its own CRC, enabling per-chunk integrity verification
during streaming.

---

## JSON Export

### World JSON (`exportWorldJson`)

Produces a human-readable JSON object:

```json
{
  "format": "velox-scene-v2",
  "version": 1,
  "gravity": [0, -9.81, 0],
  "substeps": 4,
  "backend": "CPU",
  "bodyCount": 2,
  "bodies": [
    {
      "id": 0,
      "shape": "Sphere",
      "motionType": "dynamic",
      "position": [0, 5, 0],
      "orientation": [0, 0, 0, 1],
      "velocity": [0, 0, 0],
      "angularVelocity": [0, 0, 0],
      "invMass": 1,
      "restitution": 0.3,
      "friction": 0.5,
      "sensor": false,
      "sleeping": false
    }
  ]
}
```

Floats are printed with 6 significant digits. This format is intended for
debugging and tooling — **not** for round-trip fidelity.

### Archive JSON (`exportArchiveJson`)

Describes the archive structure (section table, sizes, CRCs) without
dumping payload bytes.

---

## Scene Graph Export

`exportSceneGraph()` builds a flat node array representing the world's
body/joint topology:

- **Node 0** is always the synthetic root (`"World"`).
- **Body nodes** carry shape, mass, position, sleep state.
- **Joint nodes** carry joint type and connected dense body indices.

Two output formats:

- `sceneGraphToJson()` — JSON array of node objects with `children` index
  arrays.
- `sceneGraphToDot()` — Graphviz DOT digraph for visual debugging.

---

## API Quick Reference

```cpp
// Core serialize / deserialize.
ArchiveV2 serializeWorldV2(const World&, const SerializationOptions& = {});
void deserializeWorldV2(World&, const ArchiveV2&);

// Binary pack / unpack.
std::vector<uint8_t> packArchiveV2(const ArchiveV2&);
ArchiveV2 unpackArchiveV2(const std::vector<uint8_t>&);

// V1 migration.
ArchiveV2 migrateV1ToV2(const SerializedScene&);
bool isMigratedV1(const ArchiveV2&);

// Incremental.
class IncrementalSerializer { /* nextChunk(), finish() */ };
class IncrementalDeserializer { /* nextChunk(), finish() */ };

// JSON export.
std::string exportWorldJson(const World&, int indent = 2);
std::string exportArchiveJson(const ArchiveV2&, int indent = 2);

// Scene graph.
std::vector<SceneGraphNode> exportSceneGraph(const World&);
std::string sceneGraphToJson(const std::vector<SceneGraphNode>&, int indent = 2);
std::string sceneGraphToDot(const std::vector<SceneGraphNode>&);

// Utilities.
uint32_t crc32(const void*, size_t, uint32_t seed = 0);
bool isCompressionAvailable(CompressionV2);
std::vector<uint8_t> compressBytes(const std::vector<uint8_t>&, CompressionV2, int level = -1);
std::vector<uint8_t> decompressBytes(const std::vector<uint8_t>&, CompressionV2, size_t uncompressedSize);
```

---

## Error Handling

All failure paths throw `std::runtime_error` with a `"velox v2: …"` prefix:

| Condition | Exception message |
|---|---|
| Bad magic | `"velox v2: bad magic (not a VLX2 archive)"` |
| Unsupported version | `"velox v2: unsupported archive version N"` |
| Truncated data | `"velox v2: truncated archive"` |
| CRC mismatch (section) | `"velox v2: CRC mismatch in section N"` |
| CRC mismatch (archive) | `"velox v2: archive CRC mismatch (corrupted data)"` |
| Missing Bodies section | `"velox v2: archive missing Bodies section"` |
| Unavailable codec | `"velox: compression codec not compiled into this build"` |

---

## Thread Safety

Serialization functions are **read-only** against the `World` and safe to
call from any thread provided no concurrent mutation or `step()` is in
progress (same contract as `World::saveSnapshot()`). Deserialization
**replaces** the world's contents and must be externally synchronized.

---

## Future Work

- **Logical body-by-body format**: the current V2 Bodies section wraps the
  V1 dense-array blob. A future revision may serialize each body through the
  public API for true layout-independent version migration.
- **Streaming pack/unpack**: `packArchiveV2` / `unpackArchiveV2` operate on
  in-memory buffers. A streaming variant could write/read sections
  incrementally to/from a file or socket.
- **zlib/zstd CI**: the compression paths are compile-gated but not yet
  exercised in CI (no external codec dependencies in the default build).
