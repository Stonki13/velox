#include "velox/serialization_v2.h"
#include "velox/serialization.h" // V1 compat: serializeWorld / deserializeWorld

#include <algorithm>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <type_traits>

// Optional codec includes (compile-time gated).
#if defined(VELOX_WITH_ZLIB)
#include <zlib.h>
#endif
#if defined(VELOX_WITH_ZSTD)
#include <zstd.h>
#endif

namespace velox {

// ===========================================================================
// CRC-32 (ISO 3309 / ITU-T V.42, same polynomial as zlib / PNG)
// ===========================================================================

namespace {

struct Crc32Table {
    uint32_t entries[256];
    constexpr Crc32Table() : entries{} {
        for (uint32_t i = 0; i < 256; ++i) {
            uint32_t c = i;
            for (int j = 0; j < 8; ++j)
                c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
            entries[i] = c;
        }
    }
};

constexpr Crc32Table kCrcTable{};

} // namespace

uint32_t crc32(const void* data, size_t size, uint32_t seed) {
    auto* p = static_cast<const uint8_t*>(data);
    uint32_t crc = seed ^ 0xFFFFFFFFu;
    for (size_t i = 0; i < size; ++i)
        crc = kCrcTable.entries[(crc ^ p[i]) & 0xFF] ^ (crc >> 8);
    return crc ^ 0xFFFFFFFFu;
}

// ===========================================================================
// Compression
// ===========================================================================

bool isCompressionAvailable(CompressionV2 mode) {
    switch (mode) {
    case CompressionV2::None: return true;
    case CompressionV2::Zlib:
#if defined(VELOX_WITH_ZLIB)
        return true;
#else
        return false;
#endif
    case CompressionV2::Zstd:
#if defined(VELOX_WITH_ZSTD)
        return true;
#else
        return false;
#endif
    }
    return false;
}

std::vector<uint8_t> compressBytes(const std::vector<uint8_t>& input,
                                   CompressionV2 mode, int level) {
    if (mode == CompressionV2::None)
        return input;

#if defined(VELOX_WITH_ZLIB)
    if (mode == CompressionV2::Zlib) {
        uLongf bound = compressBound(static_cast<uLong>(input.size()));
        std::vector<uint8_t> out(bound);
        int zlevel = (level >= 0 && level <= 9) ? level : Z_DEFAULT_COMPRESSION;
        int rc = compress2(out.data(), &bound, input.data(),
                           static_cast<uLong>(input.size()), zlevel);
        if (rc != Z_OK)
            throw std::runtime_error("velox: zlib compression failed (rc=" +
                                     std::to_string(rc) + ")");
        out.resize(bound);
        return out;
    }
#endif

#if defined(VELOX_WITH_ZSTD)
    if (mode == CompressionV2::Zstd) {
        size_t bound = ZSTD_compressBound(input.size());
        std::vector<uint8_t> out(bound);
        int zlevel = (level >= 1 && level <= 22) ? level : 3;
        size_t rc = ZSTD_compress(out.data(), bound, input.data(),
                                  input.size(), zlevel);
        if (ZSTD_isError(rc))
            throw std::runtime_error(
                std::string("velox: zstd compression failed: ") +
                ZSTD_getErrorName(rc));
        out.resize(rc);
        return out;
    }
#endif

    throw std::runtime_error(
        "velox: compression codec not compiled into this build");
}

std::vector<uint8_t> decompressBytes(const std::vector<uint8_t>& input,
                                     CompressionV2 mode,
                                     size_t uncompressedSize) {
    if (mode == CompressionV2::None)
        return input;

#if defined(VELOX_WITH_ZLIB)
    if (mode == CompressionV2::Zlib) {
        std::vector<uint8_t> out(uncompressedSize);
        uLongf destLen = static_cast<uLongf>(uncompressedSize);
        int rc = uncompress(out.data(), &destLen, input.data(),
                            static_cast<uLong>(input.size()));
        if (rc != Z_OK)
            throw std::runtime_error("velox: zlib decompression failed (rc=" +
                                     std::to_string(rc) + ")");
        out.resize(destLen);
        return out;
    }
#endif

#if defined(VELOX_WITH_ZSTD)
    if (mode == CompressionV2::Zstd) {
        std::vector<uint8_t> out(uncompressedSize);
        size_t rc = ZSTD_decompress(out.data(), uncompressedSize,
                                    input.data(), input.size());
        if (ZSTD_isError(rc))
            throw std::runtime_error(
                std::string("velox: zstd decompression failed: ") +
                ZSTD_getErrorName(rc));
        out.resize(rc);
        return out;
    }
#endif

    throw std::runtime_error(
        "velox: compression codec not compiled into this build");
}

// ===========================================================================
// ArchiveSection helpers
// ===========================================================================

const ArchiveSection* ArchiveV2::find(SectionType type) const {
    for (const auto& s : sections)
        if (s.type == type) return &s;
    return nullptr;
}

ArchiveSection* ArchiveV2::find(SectionType type) {
    for (auto& s : sections)
        if (s.type == type) return &s;
    return nullptr;
}

// ===========================================================================
// Binary Writer / Reader (POD helpers, same pattern as V1)
// ===========================================================================

namespace {

struct Writer {
    std::vector<uint8_t>& out;

    template <typename T>
    void pod(const T& value) {
        static_assert(std::is_trivially_copyable<T>::value, "POD only");
        const size_t offset = out.size();
        out.resize(offset + sizeof(T));
        std::memcpy(out.data() + offset, &value, sizeof(T));
    }

    void bytes(const void* data, size_t size) {
        const size_t offset = out.size();
        out.resize(offset + size);
        if (size) std::memcpy(out.data() + offset, data, size);
    }

    void u64(uint64_t v) { pod(v); }

    void str(const std::string& s) {
        u64(s.size());
        bytes(s.data(), s.size());
    }
};

struct Reader {
    const uint8_t* cursor;
    const uint8_t* end;

    void need(size_t n) const {
        if (static_cast<size_t>(end - cursor) < n)
            throw std::runtime_error("velox v2: truncated archive");
    }

    template <typename T>
    void pod(T& value) {
        static_assert(std::is_trivially_copyable<T>::value, "POD only");
        need(sizeof(T));
        std::memcpy(&value, cursor, sizeof(T));
        cursor += sizeof(T);
    }

    uint64_t u64() {
        uint64_t v = 0;
        pod(v);
        return v;
    }

    std::string str() {
        uint64_t len = u64();
        need(static_cast<size_t>(len));
        std::string s(reinterpret_cast<const char*>(cursor),
                      static_cast<size_t>(len));
        cursor += len;
        return s;
    }

    void skip(size_t n) {
        need(n);
        cursor += n;
    }
};

// Shape name helper for JSON / scene graph.
const char* shapeTypeName(ShapeType t) {
    switch (t) {
    case ShapeType::Sphere:     return "Sphere";
    case ShapeType::Plane:      return "Plane";
    case ShapeType::Box:        return "Box";
    case ShapeType::Capsule:    return "Capsule";
    case ShapeType::Mesh:       return "Mesh";
    case ShapeType::Hull:       return "Hull";
    case ShapeType::Compound:   return "Compound";
    case ShapeType::Cylinder:   return "Cylinder";
    case ShapeType::Cone:       return "Cone";
    case ShapeType::RoundedBox: return "RoundedBox";
    case ShapeType::Ellipsoid:  return "Ellipsoid";
    }
    return "Unknown";
}

const char* motionTypeName(MotionType t) {
    switch (t) {
    case MotionType::Static:    return "static";
    case MotionType::Kinematic: return "kinematic";
    case MotionType::Dynamic:   return "dynamic";
    }
    return "unknown";
}

const char* jointTypeName(JointType t) {
    switch (t) {
    case JointType::Ball:      return "Ball";
    case JointType::Distance:  return "Distance";
    case JointType::Hinge:     return "Hinge";
    case JointType::ConeTwist: return "ConeTwist";
    case JointType::Fixed:     return "Fixed";
    case JointType::Prismatic: return "Prismatic";
    case JointType::SixDof:    return "SixDof";
    case JointType::Motor:     return "Motor";
    }
    return "Unknown";
}

// JSON string escaping.
std::string jsonEscape(const std::string& s) {
    std::string out;
    out.reserve(s.size() + 8);
    for (char c : s) {
        switch (c) {
        case '"':  out += "\\\""; break;
        case '\\': out += "\\\\"; break;
        case '\n': out += "\\n";  break;
        case '\r': out += "\\r";  break;
        case '\t': out += "\\t";  break;
        default:   out += c;      break;
        }
    }
    return out;
}

// Float formatting for JSON (6 significant digits).
std::string jsonFloat(float v) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.6g", static_cast<double>(v));
    return buf;
}

std::string jsonVec3(const Vec3& v) {
    return "[" + jsonFloat(v.x) + ", " + jsonFloat(v.y) + ", " +
           jsonFloat(v.z) + "]";
}

std::string jsonQuat(const Quat& q) {
    return "[" + jsonFloat(q.x) + ", " + jsonFloat(q.y) + ", " +
           jsonFloat(q.z) + ", " + jsonFloat(q.w) + "]";
}

// Indentation helper.
std::string indent(int level, int spaces = 2) {
    return std::string(static_cast<size_t>(level * spaces), ' ');
}

// Build a section from raw payload with optional compression + CRC.
ArchiveSection makeSection(SectionType type, std::vector<uint8_t> payload,
                           const SerializationOptions& opts) {
    ArchiveSection sec;
    sec.type = type;
    sec.uncompressedSize = payload.size();
    if (opts.computeCrc)
        sec.crc = crc32(payload.data(), payload.size());
    if (opts.compression != CompressionV2::None && !payload.empty()) {
        sec.compression = opts.compression;
        sec.payload = compressBytes(payload, opts.compression,
                                    opts.compressionLevel);
    } else {
        sec.compression = CompressionV2::None;
        sec.payload = std::move(payload);
    }
    return sec;
}

// Decode a section payload (decompress + verify CRC).
std::vector<uint8_t> decodeSection(const ArchiveSection& sec,
                                   bool verifyCrc = true) {
    std::vector<uint8_t> data;
    if (sec.compression != CompressionV2::None) {
        data = decompressBytes(sec.payload, sec.compression,
                               sec.uncompressedSize);
    } else {
        data = sec.payload;
    }
    if (verifyCrc && sec.crc != 0) {
        uint32_t actual = crc32(data.data(), data.size());
        if (actual != sec.crc)
            throw std::runtime_error(
                "velox v2: CRC mismatch in section " +
                std::to_string(static_cast<uint32_t>(sec.type)) +
                " (expected " + std::to_string(sec.crc) + ", got " +
                std::to_string(actual) + ")");
    }
    return data;
}

// V1 migration marker stored in the Metadata section.
constexpr const char* kV1MigrationMarker = "velox-v1-migrated";

} // namespace

// ===========================================================================
// Core serialize / deserialize
// ===========================================================================

ArchiveV2 serializeWorldV2(const World& world,
                           const SerializationOptions& opts) {
    ArchiveV2 archive;
    archive.version = kSerializationV2Version;
    archive.metadata = opts.metadata;

    // Capture the full simulation state via the proven V1 encoder.
    SerializedScene v1 = serializeWorld(world, opts.metadata);

    // --- Bodies section (the V1 blob) ---
    archive.sections.push_back(
        makeSection(SectionType::Bodies, v1.data, opts));

    // --- Settings section ---
    {
        std::vector<uint8_t> buf;
        Writer w{buf};
        w.pod(world.gravityValue());
        w.pod(world.substepCount());
        w.pod(world.ccdDefaults());
        w.pod(world.multiToiSettings());
        w.pod(world.solverOptions());
        archive.sections.push_back(
            makeSection(SectionType::Settings, std::move(buf), opts));
    }

    // --- Metadata section ---
    {
        std::vector<uint8_t> buf;
        Writer w{buf};
        w.str(opts.metadata);
        archive.sections.push_back(
            makeSection(SectionType::Metadata, std::move(buf), opts));
    }

    return archive;
}

void deserializeWorldV2(World& world, const ArchiveV2& archive) {
    if (archive.version < kSerializationV2MinVersion ||
        archive.version > kSerializationV2Version)
        throw std::runtime_error(
            "velox v2: archive version " + std::to_string(archive.version) +
            " is not supported (expected " +
            std::to_string(kSerializationV2MinVersion) + ".." +
            std::to_string(kSerializationV2Version) + ")");

    // --- Bodies (required) ---
    const ArchiveSection* bodiesSec = archive.find(SectionType::Bodies);
    if (!bodiesSec)
        throw std::runtime_error("velox v2: archive missing Bodies section");

    std::vector<uint8_t> blob;
    for (const ArchiveSection& section : archive.sections) {
        if (section.type != SectionType::Bodies) continue;
        std::vector<uint8_t> chunk = decodeSection(section);
        blob.insert(blob.end(), chunk.begin(), chunk.end());
    }

    // Reconstruct a V1 SerializedScene and use the proven V1 decoder.
    SerializedScene v1;
    v1.version = kSerializationVersion;
    v1.data = std::move(blob);

    // Restore metadata if present.
    const ArchiveSection* metaSec = archive.find(SectionType::Metadata);
    if (metaSec) {
        std::vector<uint8_t> metaBuf = decodeSection(*metaSec, false);
        Reader r{metaBuf.data(), metaBuf.data() + metaBuf.size()};
        v1.metadata = r.str();
    }

    deserializeWorld(world, v1);

    // --- Settings (optional; the V1 blob already carries these, but the
    //     explicit section lets tools patch them without re-encoding) ---
    const ArchiveSection* settingsSec = archive.find(SectionType::Settings);
    if (settingsSec) {
        std::vector<uint8_t> buf = decodeSection(*settingsSec);
        Reader r{buf.data(), buf.data() + buf.size()};
        Vec3 gravity;
        int substeps = 4;
        WorldCcdDefaults ccdDefaults;
        WorldMultiToiSettings multiToi;
        SolverOptions solverOpts;
        r.pod(gravity);
        r.pod(substeps);
        r.pod(ccdDefaults);
        r.pod(multiToi);
        r.pod(solverOpts);
        world.setGravity(gravity);
        world.setSubsteps(substeps);
        world.setCcdDefaults(ccdDefaults);
        world.setMultiToiSettings(multiToi);
        world.setSolverOptions(solverOpts);
    }
}

// ===========================================================================
// Binary pack / unpack
// ===========================================================================

// Container layout:
//   [4]  magic        (kSerializationV2Magic)
//   [4]  version      (kSerializationV2Version)
//   [4]  flags        (bit 0: archive CRC present)
//   [8]  sectionCount
//   Per section:
//     [4]  type
//     [1]  compression
//     [4]  crc
//     [8]  uncompressedSize
//     [8]  payloadSize
//     [N]  payload
//   [4]  archiveCRC   (CRC-32 of everything before this field)

std::vector<uint8_t> packArchiveV2(const ArchiveV2& archive) {
    std::vector<uint8_t> bytes;
    Writer w{bytes};

    w.pod(kSerializationV2Magic);
    w.pod(archive.version);
    uint32_t flags = 1u; // bit 0: archive CRC present
    w.pod(flags);
    w.u64(archive.sections.size());

    for (const auto& sec : archive.sections) {
        w.pod(static_cast<uint32_t>(sec.type));
        w.pod(static_cast<uint8_t>(sec.compression));
        w.pod(sec.crc);
        w.u64(sec.uncompressedSize);
        w.u64(sec.payload.size());
        w.bytes(sec.payload.data(), sec.payload.size());
    }

    // Archive-level CRC over everything written so far.
    uint32_t archiveCrc = crc32(bytes.data(), bytes.size());
    w.pod(archiveCrc);

    return bytes;
}

ArchiveV2 unpackArchiveV2(const std::vector<uint8_t>& bytes) {
    if (bytes.size() < 24)
        throw std::runtime_error("velox v2: archive too small");

    Reader r{bytes.data(), bytes.data() + bytes.size()};

    uint32_t magic = 0;
    r.pod(magic);
    if (magic != kSerializationV2Magic)
        throw std::runtime_error("velox v2: bad magic (not a VLX2 archive)");

    ArchiveV2 archive;
    r.pod(archive.version);
    if (archive.version < kSerializationV2MinVersion ||
        archive.version > kSerializationV2Version)
        throw std::runtime_error(
            "velox v2: unsupported archive version " +
            std::to_string(archive.version));

    uint32_t flags = 0;
    r.pod(flags);

    uint64_t sectionCount = r.u64();
    // Sanity: each section header is at least 25 bytes.
    if (sectionCount > (bytes.size() / 25) + 1)
        throw std::runtime_error("velox v2: implausible section count");

    archive.sections.reserve(static_cast<size_t>(sectionCount));
    for (uint64_t i = 0; i < sectionCount; ++i) {
        ArchiveSection sec;
        uint32_t typeTag = 0;
        r.pod(typeTag);
        sec.type = static_cast<SectionType>(typeTag);
        uint8_t comp = 0;
        r.pod(comp);
        sec.compression = static_cast<CompressionV2>(comp);
        r.pod(sec.crc);
        sec.uncompressedSize = static_cast<size_t>(r.u64());
        uint64_t payloadSize = r.u64();
        r.need(static_cast<size_t>(payloadSize));
        sec.payload.assign(r.cursor, r.cursor + payloadSize);
        r.cursor += payloadSize;
        archive.sections.push_back(std::move(sec));
    }

    // Verify archive-level CRC.
    if (flags & 1u) {
        // The CRC covers everything before the last 4 bytes.
        size_t crcOffset = bytes.size() - 4;
        uint32_t storedCrc = 0;
        std::memcpy(&storedCrc, bytes.data() + crcOffset, 4);
        uint32_t computedCrc = crc32(bytes.data(), crcOffset);
        if (storedCrc != computedCrc)
            throw std::runtime_error(
                "velox v2: archive CRC mismatch (corrupted data)");
    }

    // Extract metadata from the Metadata section if present.
    const ArchiveSection* metaSec = archive.find(SectionType::Metadata);
    if (metaSec) {
        try {
            std::vector<uint8_t> metaBuf = decodeSection(*metaSec, false);
            Reader mr{metaBuf.data(), metaBuf.data() + metaBuf.size()};
            archive.metadata = mr.str();
        } catch (...) {
            // Metadata is best-effort.
        }
    }

    return archive;
}

// ===========================================================================
// V1 backward compatibility
// ===========================================================================

ArchiveV2 migrateV1ToV2(const SerializedScene& v1) {
    ArchiveV2 archive;
    archive.version = kSerializationV2Version;
    archive.metadata = v1.metadata;

    // Store the V1 blob as the Bodies section.
    SerializationOptions opts;
    opts.computeCrc = true;
    archive.sections.push_back(
        makeSection(SectionType::Bodies, v1.data, opts));

    // Store metadata with the migration marker.
    {
        std::vector<uint8_t> buf;
        Writer w{buf};
        w.str(std::string(kV1MigrationMarker) + "\n" + v1.metadata);
        archive.sections.push_back(
            makeSection(SectionType::Metadata, std::move(buf), opts));
    }

    return archive;
}

bool isMigratedV1(const ArchiveV2& archive) {
    const ArchiveSection* metaSec = archive.find(SectionType::Metadata);
    if (!metaSec) return false;
    try {
        std::vector<uint8_t> buf = decodeSection(*metaSec, false);
        Reader r{buf.data(), buf.data() + buf.size()};
        std::string meta = r.str();
        return meta.find(kV1MigrationMarker) != std::string::npos;
    } catch (...) {
        return false;
    }
}

// ===========================================================================
// Incremental serialization
// ===========================================================================

IncrementalSerializer::IncrementalSerializer(const World& world,
                                             const SerializationOptions& opts)
    : world_(world), opts_(opts) {}

bool IncrementalSerializer::nextChunk(size_t maxBodies) {
    if (finished_) return false;

    // Capture the full V1 blob on the first call, then emit it in chunks.
    // This keeps the proven V1 encoder as the source of truth while allowing
    // the container / network layer to process fixed-size chunks.
    if (bodyChunks_.empty() && bodiesDone_ == 0) {
        SerializedScene v1 = serializeWorld(world_, opts_.metadata);
        const auto& blob = v1.data;
        size_t total = blob.size();
        size_t offset = 0;
        size_t chunkIndex = 0;
        while (offset < total) {
            size_t chunkSize = std::min(maxBodies * sizeof(uint64_t),
                                        total - offset);
            // Ensure at least 1 byte per chunk.
            chunkSize = std::max<size_t>(chunkSize, 1);
            std::vector<uint8_t> chunk(blob.begin() + static_cast<ptrdiff_t>(offset),
                                       blob.begin() + static_cast<ptrdiff_t>(offset + chunkSize));
            ArchiveSection sec;
            sec.type = SectionType::Bodies;
            sec.uncompressedSize = chunk.size();
            if (opts_.computeCrc)
                sec.crc = crc32(chunk.data(), chunk.size());
            if (opts_.compression != CompressionV2::None && !chunk.empty()) {
                sec.compression = opts_.compression;
                sec.payload = compressBytes(chunk, opts_.compression,
                                            opts_.compressionLevel);
            } else {
                sec.compression = CompressionV2::None;
                sec.payload = std::move(chunk);
            }
            bodyChunks_.push_back(std::move(sec));
            offset += chunkSize;
            ++chunkIndex;
        }
        // Handle empty blob edge case.
        if (bodyChunks_.empty()) {
            ArchiveSection sec;
            sec.type = SectionType::Bodies;
            sec.uncompressedSize = 0;
            sec.crc = 0;
            bodyChunks_.push_back(std::move(sec));
        }
    }

    if (bodiesDone_ >= bodyChunks_.size()) return false;
    ++bodiesDone_;
    return bodiesDone_ < bodyChunks_.size();
}

ArchiveV2 IncrementalSerializer::finish() {
    if (finished_)
        throw std::runtime_error("velox v2: IncrementalSerializer already finished");
    finished_ = true;

    ArchiveV2 archive;
    archive.version = kSerializationV2Version;
    archive.metadata = opts_.metadata;

    // Emit all body chunks (including any not yet stepped through).
    for (auto& chunk : bodyChunks_)
        archive.sections.push_back(std::move(chunk));

    // Settings section.
    {
        std::vector<uint8_t> buf;
        Writer w{buf};
        w.pod(world_.gravityValue());
        w.pod(world_.substepCount());
        w.pod(world_.ccdDefaults());
        w.pod(world_.multiToiSettings());
        w.pod(world_.solverOptions());
        archive.sections.push_back(
            makeSection(SectionType::Settings, std::move(buf), opts_));
    }

    // Metadata section.
    {
        std::vector<uint8_t> buf;
        Writer w{buf};
        w.str(opts_.metadata);
        archive.sections.push_back(
            makeSection(SectionType::Metadata, std::move(buf), opts_));
    }

    return archive;
}

IncrementalDeserializer::IncrementalDeserializer(World& world,
                                                 const ArchiveV2& archive)
    : world_(world), archive_(archive) {}

bool IncrementalDeserializer::nextChunk() {
    if (finished_) return false;

    // Count body chunks.
    size_t bodyChunkCount = 0;
    for (const auto& sec : archive_.sections)
        if (sec.type == SectionType::Bodies) ++bodyChunkCount;

    if (chunkIndex_ >= bodyChunkCount) return false;
    ++chunkIndex_;
    return chunkIndex_ < bodyChunkCount;
}

void IncrementalDeserializer::finish() {
    if (finished_)
        throw std::runtime_error("velox v2: IncrementalDeserializer already finished");
    finished_ = true;

    // Reassemble all body chunks into a single V1 blob.
    std::vector<uint8_t> blob;
    for (const auto& sec : archive_.sections) {
        if (sec.type != SectionType::Bodies) continue;
        std::vector<uint8_t> chunk = decodeSection(sec);
        blob.insert(blob.end(), chunk.begin(), chunk.end());
    }

    SerializedScene v1;
    v1.version = kSerializationVersion;
    v1.data = std::move(blob);
    deserializeWorld(world_, v1);

    // Apply settings.
    const ArchiveSection* settingsSec = archive_.find(SectionType::Settings);
    if (settingsSec) {
        std::vector<uint8_t> buf = decodeSection(*settingsSec);
        Reader r{buf.data(), buf.data() + buf.size()};
        Vec3 gravity;
        int substeps = 4;
        WorldCcdDefaults ccdDefaults;
        WorldMultiToiSettings multiToi;
        SolverOptions solverOpts;
        r.pod(gravity);
        r.pod(substeps);
        r.pod(ccdDefaults);
        r.pod(multiToi);
        r.pod(solverOpts);
        world_.setGravity(gravity);
        world_.setSubsteps(substeps);
        world_.setCcdDefaults(ccdDefaults);
        world_.setMultiToiSettings(multiToi);
        world_.setSolverOptions(solverOpts);
    }
}

// ===========================================================================
// JSON export
// ===========================================================================

std::string exportWorldJson(const World& world, int indent) {
    std::ostringstream os;
    const std::string i0 = velox::indent(0, indent);
    const std::string i1 = velox::indent(1, indent);
    const std::string i2 = velox::indent(2, indent);

    os << i0 << "{\n";
    os << i1 << "\"format\": \"velox-scene-v2\",\n";
    os << i1 << "\"version\": " << kSerializationV2Version << ",\n";

    // Gravity & settings.
    Vec3 g = world.gravityValue();
    os << i1 << "\"gravity\": " << jsonVec3(g) << ",\n";
    os << i1 << "\"substeps\": " << world.substepCount() << ",\n";
    os << i1 << "\"backend\": \"" << jsonEscape(world.backendName()) << "\",\n";
    os << i1 << "\"bodyCount\": " << world.bodyCount() << ",\n";

    // Bodies.
    os << i1 << "\"bodies\": [\n";
    // We iterate by trying BodyIds. Since we don't have a public iterator,
    // we use bodyCount and sequential slot probing.
    size_t emitted = 0;
    size_t totalBodies = world.bodyCount();
    for (uint32_t slot = 0; emitted < totalBodies && slot < totalBodies + 64;
         ++slot) {
        // Probe generations 0..3 (covers typical reuse patterns).
        bool found = false;
        for (uint32_t gen = 0; gen < 4 && !found; ++gen) {
            BodyId id = BodyId::make(slot, gen);
            if (!world.isValid(id)) continue;
            const Body& b = world.body(id);
            if (emitted > 0) os << ",\n";
            os << i2 << "{\n";
            os << i2 << velox::indent(1, indent) << "\"id\": " << id.value << ",\n";
            os << i2 << velox::indent(1, indent) << "\"shape\": \""
               << shapeTypeName(b.shape) << "\",\n";
            os << i2 << velox::indent(1, indent) << "\"motionType\": \""
               << motionTypeName(b.motionType) << "\",\n";
            os << i2 << velox::indent(1, indent) << "\"position\": "
               << jsonVec3(b.position) << ",\n";
            os << i2 << velox::indent(1, indent) << "\"orientation\": "
               << jsonQuat(b.orientation) << ",\n";
            os << i2 << velox::indent(1, indent) << "\"velocity\": "
               << jsonVec3(b.velocity) << ",\n";
            os << i2 << velox::indent(1, indent) << "\"angularVelocity\": "
               << jsonVec3(b.angularVelocity) << ",\n";
            os << i2 << velox::indent(1, indent) << "\"invMass\": "
               << jsonFloat(b.invMass) << ",\n";
            os << i2 << velox::indent(1, indent) << "\"restitution\": "
               << jsonFloat(b.restitution) << ",\n";
            os << i2 << velox::indent(1, indent) << "\"friction\": "
               << jsonFloat(b.friction) << ",\n";
            os << i2 << velox::indent(1, indent) << "\"sensor\": "
               << (b.sensor ? "true" : "false") << ",\n";
            os << i2 << velox::indent(1, indent) << "\"sleeping\": "
               << (b.asleep ? "true" : "false") << "\n";
            os << i2 << "}";
            ++emitted;
            found = true;
        }
    }
    os << "\n" << i1 << "]\n";
    os << i0 << "}\n";
    return os.str();
}

std::string exportArchiveJson(const ArchiveV2& archive, int indent) {
    std::ostringstream os;
    const std::string i0 = velox::indent(0, indent);
    const std::string i1 = velox::indent(1, indent);
    const std::string i2 = velox::indent(2, indent);

    os << i0 << "{\n";
    os << i1 << "\"format\": \"velox-archive-v2\",\n";
    os << i1 << "\"version\": " << archive.version << ",\n";
    os << i1 << "\"metadata\": \"" << jsonEscape(archive.metadata) << "\",\n";
    os << i1 << "\"sectionCount\": " << archive.sections.size() << ",\n";
    os << i1 << "\"sections\": [\n";
    for (size_t i = 0; i < archive.sections.size(); ++i) {
        const auto& sec = archive.sections[i];
        if (i > 0) os << ",\n";
        os << i2 << "{\n";
        os << i2 << velox::indent(1, indent) << "\"type\": "
           << static_cast<uint32_t>(sec.type) << ",\n";
        os << i2 << velox::indent(1, indent) << "\"compression\": "
           << static_cast<int>(sec.compression) << ",\n";
        os << i2 << velox::indent(1, indent) << "\"crc\": " << sec.crc << ",\n";
        os << i2 << velox::indent(1, indent) << "\"uncompressedSize\": "
           << sec.uncompressedSize << ",\n";
        os << i2 << velox::indent(1, indent) << "\"payloadSize\": "
           << sec.payload.size() << "\n";
        os << i2 << "}";
    }
    os << "\n" << i1 << "]\n";
    os << i0 << "}\n";
    return os.str();
}

// ===========================================================================
// Scene-graph export
// ===========================================================================

std::vector<SceneGraphNode> exportSceneGraph(const World& world) {
    std::vector<SceneGraphNode> graph;

    // Node 0: synthetic root.
    SceneGraphNode root;
    root.name = "World";
    root.type = "world";
    root.position = world.gravityValue(); // show gravity as position hint
    graph.push_back(std::move(root));

    size_t totalBodies = world.bodyCount();

    // Create body nodes.
    size_t emitted = 0;
    for (uint32_t slot = 0; emitted < totalBodies && slot < totalBodies + 64;
         ++slot) {
        for (uint32_t gen = 0; gen < 4; ++gen) {
            BodyId id = BodyId::make(slot, gen);
            if (!world.isValid(id)) continue;
            const Body& b = world.body(id);

            SceneGraphNode node;
            node.bodyId = id.value;
            node.type = "body";
            node.shapeName = shapeTypeName(b.shape);
            node.name = "Body_" + std::to_string(id.value) + " (" +
                        node.shapeName + ", " +
                        motionTypeName(b.motionType) + ")";
            node.position = b.position;
            node.orientation = b.orientation;
            node.mass = b.invMass > 0.0f ? 1.0f / b.invMass : 0.0f;
            node.isStatic = b.isStatic();
            node.isSleeping = isFullyAsleep(b.asleep);

            size_t nodeIndex = graph.size();
            graph.push_back(std::move(node));
            graph[0].children.push_back(nodeIndex);
            ++emitted;
            break; // found this slot, move to next
        }
    }

    // Create joint nodes. Joints reference bodies by dense BodyIndex, which
    // cannot be mapped back to BodyId through the public API alone. We record
    // the dense indices in the node name for debugging and attach joints to
    // the world root.
    for (uint32_t slot = 0; slot < totalBodies + 64; ++slot) {
        for (uint32_t gen = 0; gen < 4; ++gen) {
            JointId jid = JointId::make(slot, gen);
            if (!world.isValid(jid)) continue;
            const Joint& j = world.joint(jid);

            SceneGraphNode node;
            node.jointId = jid.value;
            node.type = "joint";
            node.jointType = jointTypeName(j.type);
            node.name = "Joint_" + std::to_string(jid.value) + " (" +
                        node.jointType + ", bodies " +
                        std::to_string(j.a) + "<->" +
                        std::to_string(j.b) + ")";

            size_t jointNodeIndex = graph.size();
            graph.push_back(std::move(node));
            graph[0].children.push_back(jointNodeIndex);
            break;
        }
    }

    return graph;
}

std::string sceneGraphToJson(const std::vector<SceneGraphNode>& graph,
                             int indent) {
    std::ostringstream os;
    const std::string i0 = velox::indent(0, indent);
    const std::string i1 = velox::indent(1, indent);
    const std::string i2 = velox::indent(2, indent);
    const std::string i3 = velox::indent(3, indent);

    os << i0 << "[\n";
    for (size_t i = 0; i < graph.size(); ++i) {
        const auto& node = graph[i];
        if (i > 0) os << ",\n";
        os << i1 << "{\n";
        os << i2 << "\"index\": " << i << ",\n";
        os << i2 << "\"name\": \"" << jsonEscape(node.name) << "\",\n";
        os << i2 << "\"type\": \"" << jsonEscape(node.type) << "\",\n";
        os << i2 << "\"position\": " << jsonVec3(node.position) << ",\n";
        os << i2 << "\"orientation\": " << jsonQuat(node.orientation);
        if (node.bodyId != UINT64_MAX) {
            os << ",\n" << i2 << "\"bodyId\": " << node.bodyId;
            os << ",\n" << i2 << "\"shape\": \""
               << jsonEscape(node.shapeName) << "\"";
            os << ",\n" << i2 << "\"mass\": " << jsonFloat(node.mass);
            os << ",\n" << i2 << "\"isStatic\": "
               << (node.isStatic ? "true" : "false");
            os << ",\n" << i2 << "\"isSleeping\": "
               << (node.isSleeping ? "true" : "false");
        }
        if (node.jointId != UINT64_MAX) {
            os << ",\n" << i2 << "\"jointId\": " << node.jointId;
            os << ",\n" << i2 << "\"jointType\": \""
               << jsonEscape(node.jointType) << "\"";
        }
        if (!node.children.empty()) {
            os << ",\n" << i2 << "\"children\": [";
            for (size_t c = 0; c < node.children.size(); ++c) {
                if (c > 0) os << ", ";
                os << node.children[c];
            }
            os << "]";
        }
        os << "\n" << i1 << "}";
    }
    os << "\n" << i0 << "]\n";
    return os.str();
}

std::string sceneGraphToDot(const std::vector<SceneGraphNode>& graph) {
    std::ostringstream os;
    os << "digraph VeloxScene {\n";
    os << "  rankdir=TB;\n";
    os << "  node [shape=record, fontsize=10];\n\n";

    for (size_t i = 0; i < graph.size(); ++i) {
        const auto& node = graph[i];
        os << "  n" << i << " [label=\"{" << node.name;
        if (node.type == "body") {
            os << "|mass=" << jsonFloat(node.mass);
            os << "|pos=" << jsonFloat(node.position.x) << ","
               << jsonFloat(node.position.y) << ","
               << jsonFloat(node.position.z);
            if (node.isStatic) os << "|STATIC";
            if (node.isSleeping) os << "|SLEEPING";
        }
        if (node.type == "joint") {
            os << "|" << node.jointType;
        }
        os << "}\"];\n";
    }

    os << "\n";
    for (size_t i = 0; i < graph.size(); ++i) {
        for (size_t child : graph[i].children) {
            os << "  n" << i << " -> n" << child << ";\n";
        }
    }

    os << "}\n";
    return os.str();
}

} // namespace velox
