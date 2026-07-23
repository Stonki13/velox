#include "velox/rollback.h"
#include "velox/error.h"

#include <cstring>

namespace velox {

namespace {

constexpr size_t kRecordSize = sizeof(Vec3) + sizeof(Quat) + sizeof(Vec3) + sizeof(Vec3); // 52

// 64-bit FNV-1a: simple, dependency-free, stable across platforms for a
// fixed byte sequence (no reliance on struct padding — the input is always
// the tightly-packed captureCanonicalBodyState buffer, never a raw struct).
uint64_t fnv1a(const void* data, size_t size, uint64_t seed) {
    constexpr uint64_t kPrime = 1099511628211ull;
    uint64_t hash = seed;
    const uint8_t* bytes = static_cast<const uint8_t*>(data);
    for (size_t i = 0; i < size; ++i) {
        hash ^= bytes[i];
        hash *= kPrime;
    }
    return hash;
}

uint64_t mixU64(uint64_t hash, uint64_t value) {
    return fnv1a(&value, sizeof(value), hash);
}

// One 52-byte record's four fields, matching ReplayBodyState's layout
// exactly (see serialization.cpp) without depending on that private type.
struct RecordView {
    Vec3 position;
    Quat orientation;
    Vec3 velocity;
    Vec3 angularVelocity;
};

RecordView readRecord(const uint8_t* p) {
    RecordView r;
    std::memcpy(&r.position, p, sizeof(Vec3)); p += sizeof(Vec3);
    std::memcpy(&r.orientation, p, sizeof(Quat)); p += sizeof(Quat);
    std::memcpy(&r.velocity, p, sizeof(Vec3)); p += sizeof(Vec3);
    std::memcpy(&r.angularVelocity, p, sizeof(Vec3));
    return r;
}

void setBit(std::vector<uint8_t>& mask, uint32_t index) {
    mask[index / 8] |= static_cast<uint8_t>(1u << (index % 8));
}

bool getBit(const std::vector<uint8_t>& mask, uint32_t index) {
    if (index / 8 >= mask.size()) return false;
    return (mask[index / 8] & static_cast<uint8_t>(1u << (index % 8))) != 0;
}

} // namespace

CanonicalHash hashCanonicalBodyState(const std::vector<uint8_t>& bodyState) {
    CanonicalHash result;
    result.formatVersion = kRollbackToolkitVersion;
    result.bodyCount = static_cast<uint32_t>(bodyState.size() / kRecordSize);
    uint64_t hash = fnv1a(nullptr, 0, 1469598103934665603ull); // FNV offset basis
    hash = mixU64(hash, static_cast<uint64_t>(result.formatVersion));
    hash = mixU64(hash, static_cast<uint64_t>(result.bodyCount));
    hash = fnv1a(bodyState.data(), bodyState.size(), hash);
    result.hash = hash;
    return result;
}

CanonicalHash computeCanonicalHash(const World& world) {
    return hashCanonicalBodyState(captureCanonicalBodyState(world));
}

SnapshotDelta encodeDelta(const std::vector<uint8_t>& base,
                          const std::vector<uint8_t>& target) {
    if (base.size() % kRecordSize != 0 || target.size() % kRecordSize != 0)
        VELOX_THROW(VeloxInvalidArgument, ErrorCode::InvalidConfiguration,
                    "canonical body-state buffer size is not a multiple of the record size");
    if (base.size() != target.size())
        VELOX_THROW(VeloxInvalidArgument, ErrorCode::InvalidConfiguration,
                    "encodeDelta requires base and target to have the same body count; "
                    "send a full snapshot when bodies are added or removed");

    SnapshotDelta delta;
    delta.bodyCount = static_cast<uint32_t>(target.size() / kRecordSize);
    delta.changedMask.assign((delta.bodyCount + 7) / 8, 0);
    delta.changedRecords.reserve(target.size());

    for (uint32_t i = 0; i < delta.bodyCount; ++i) {
        const uint8_t* baseRecord = base.data() + static_cast<size_t>(i) * kRecordSize;
        const uint8_t* targetRecord = target.data() + static_cast<size_t>(i) * kRecordSize;
        if (std::memcmp(baseRecord, targetRecord, kRecordSize) != 0) {
            setBit(delta.changedMask, i);
            delta.changedRecords.insert(delta.changedRecords.end(), targetRecord,
                                        targetRecord + kRecordSize);
        }
    }
    return delta;
}

std::vector<uint8_t> applyDelta(const std::vector<uint8_t>& base,
                                const SnapshotDelta& delta) {
    if (base.size() != static_cast<size_t>(delta.bodyCount) * kRecordSize)
        VELOX_THROW(VeloxRuntimeError, ErrorCode::SerializedLayoutMismatch,
                    "applyDelta: base body count does not match the delta's body count");
    if (delta.changedMask.size() != (static_cast<size_t>(delta.bodyCount) + 7) / 8)
        VELOX_THROW(VeloxRuntimeError, ErrorCode::SerializedTruncated,
                    "applyDelta: changedMask size does not match bodyCount");

    std::vector<uint8_t> result = base;
    size_t cursor = 0;
    for (uint32_t i = 0; i < delta.bodyCount; ++i) {
        if (!getBit(delta.changedMask, i)) continue;
        if (cursor + kRecordSize > delta.changedRecords.size())
            VELOX_THROW(VeloxRuntimeError, ErrorCode::SerializedTruncated,
                        "applyDelta: changedRecords is truncated relative to changedMask "
                        "(corrupted or malicious delta)");
        std::memcpy(result.data() + static_cast<size_t>(i) * kRecordSize,
                    delta.changedRecords.data() + cursor, kRecordSize);
        cursor += kRecordSize;
    }
    if (cursor != delta.changedRecords.size())
        VELOX_THROW(VeloxRuntimeError, ErrorCode::SerializedTrailingBytes,
                    "applyDelta: changedRecords has more bytes than changedMask accounts for");
    return result;
}

uint32_t changedBodyCount(const SnapshotDelta& delta) {
    uint32_t count = 0;
    for (uint32_t i = 0; i < delta.bodyCount; ++i)
        if (getBit(delta.changedMask, i)) ++count;
    return count;
}

// --- RollbackBuffer ---------------------------------------------------------

RollbackBuffer::RollbackBuffer(size_t capacity) : capacity_(capacity) {
    if (capacity == 0)
        VELOX_THROW(VeloxInvalidArgument, ErrorCode::NonPositiveValue,
                    "RollbackBuffer capacity must be at least 1");
}

void RollbackBuffer::push(uint64_t frame, const World& world) {
    for (Entry& entry : entries_) {
        if (entry.frame == frame) {
            entry.snapshot = world.saveSnapshot();
            return;
        }
    }
    entries_.push_back({frame, world.saveSnapshot()});
    while (entries_.size() > capacity_)
        entries_.pop_front();
}

bool RollbackBuffer::contains(uint64_t frame) const {
    for (const Entry& entry : entries_)
        if (entry.frame == frame) return true;
    return false;
}

bool RollbackBuffer::restore(uint64_t frame, World& world) const {
    for (const Entry& entry : entries_) {
        if (entry.frame == frame) {
            world.restoreSnapshot(entry.snapshot);
            return true;
        }
    }
    return false;
}

uint64_t RollbackBuffer::oldestFrame() const {
    if (entries_.empty())
        VELOX_THROW(VeloxLogicError, ErrorCode::InvalidConfiguration,
                    "RollbackBuffer::oldestFrame called on an empty buffer");
    return entries_.front().frame;
}

uint64_t RollbackBuffer::newestFrame() const {
    if (entries_.empty())
        VELOX_THROW(VeloxLogicError, ErrorCode::InvalidConfiguration,
                    "RollbackBuffer::newestFrame called on an empty buffer");
    return entries_.back().frame;
}

// --- Divergence diagnostics --------------------------------------------------

DivergenceReport findFirstDivergence(const ReplayRecording& expected,
                                     const ReplayRecording& actual,
                                     float positionTolerance,
                                     float velocityTolerance,
                                     float orientationTolerance) {
    DivergenceReport report;
    const size_t frameCount = expected.frames.size() < actual.frames.size()
        ? expected.frames.size() : actual.frames.size();

    for (size_t frame = 0; frame < frameCount; ++frame) {
        const std::vector<uint8_t>& e = expected.frames[frame];
        const std::vector<uint8_t>& a = actual.frames[frame];
        const uint32_t expectedCount = static_cast<uint32_t>(e.size() / kRecordSize);
        const uint32_t actualCount = static_cast<uint32_t>(a.size() / kRecordSize);
        if (expectedCount != actualCount) {
            report.diverged = true;
            report.frame = frame + 1;
            report.bodyIndex = 0;
            report.field = "bodyCount";
            report.magnitude = static_cast<float>(actualCount) -
                               static_cast<float>(expectedCount);
            return report;
        }

        for (uint32_t body = 0; body < expectedCount; ++body) {
            RecordView er = readRecord(e.data() + static_cast<size_t>(body) * kRecordSize);
            RecordView ar = readRecord(a.data() + static_cast<size_t>(body) * kRecordSize);

            const float positionDelta = length(er.position - ar.position);
            if (positionDelta > positionTolerance) {
                report.diverged = true; report.frame = frame + 1; report.bodyIndex = body;
                report.field = "position"; report.magnitude = positionDelta;
                return report;
            }
            const Vec3 orientationDiff{er.orientation.x - ar.orientation.x,
                                       er.orientation.y - ar.orientation.y,
                                       er.orientation.z - ar.orientation.z};
            const float orientationDelta =
                length(orientationDiff) + std::fabs(er.orientation.w - ar.orientation.w);
            if (orientationDelta > orientationTolerance) {
                report.diverged = true; report.frame = frame + 1; report.bodyIndex = body;
                report.field = "orientation"; report.magnitude = orientationDelta;
                return report;
            }
            const float velocityDelta = length(er.velocity - ar.velocity);
            if (velocityDelta > velocityTolerance) {
                report.diverged = true; report.frame = frame + 1; report.bodyIndex = body;
                report.field = "velocity"; report.magnitude = velocityDelta;
                return report;
            }
            const float angularDelta = length(er.angularVelocity - ar.angularVelocity);
            if (angularDelta > velocityTolerance) {
                report.diverged = true; report.frame = frame + 1; report.bodyIndex = body;
                report.field = "angularVelocity"; report.magnitude = angularDelta;
                return report;
            }
        }
    }

    if (expected.frames.size() != actual.frames.size()) {
        report.diverged = true;
        report.frame = frameCount + 1;
        report.bodyIndex = 0;
        report.field = "frameCount";
        report.magnitude = static_cast<float>(actual.frames.size()) -
                           static_cast<float>(expected.frames.size());
    }
    return report;
}

} // namespace velox
