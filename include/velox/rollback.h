#pragma once
// Deterministic multiplayer toolkit: canonical state hashing, compact delta
// snapshots, a bounded local rollback buffer, and first-divergence
// diagnostics — the missing pieces for client-side prediction and
// server-reconciliation netcode on top of Velox's existing deterministic
// replay foundation (serialization.h) and cross-platform bitwise CPU
// determinism guarantee.
//
// Division of responsibility, deliberately not duplicated elsewhere:
//   - WorldSnapshot (world.h)      : cheapest local rollback, SAME World
//                                    instance only (restoreSnapshot enforces
//                                    this). Used by RollbackBuffer below for
//                                    a client's own predicted history.
//   - SerializedScene (serialization.h): cross-instance/cross-machine full
//                                    state transfer (no owner restriction).
//                                    Use this to adopt a server's
//                                    authoritative snapshot into a
//                                    different World (the client's).
//   - This header                 : the network-facing layer above both —
//                                    a cheap comparable hash, a compact
//                                    delta between two hashes' underlying
//                                    states, a bounded local history ring
//                                    buffer, and a diagnostic that pinpoints
//                                    exactly which body/field first
//                                    disagreed between two recordings.
#include "serialization.h"
#include "world.h"

#include <cstdint>
#include <deque>
#include <string>
#include <vector>

namespace velox {

inline constexpr uint32_t kRollbackToolkitVersion = 1;

// ---------------------------------------------------------------------------
// Canonical state hash
// ---------------------------------------------------------------------------

// A versioned, deterministic hash of a world's per-body dynamic state
// (position, orientation, velocity, angularVelocity, dense body order).
// Two worlds with the same hash and the same formatVersion are, with
// overwhelming probability, in the same observable state — this is the
// cheap value peers exchange every frame; the full SerializedScene is only
// sent when hashes disagree.
struct CanonicalHash {
    uint32_t formatVersion = kRollbackToolkitVersion;
    uint32_t bodyCount = 0;
    uint64_t hash = 0;

    bool operator==(const CanonicalHash& other) const {
        return formatVersion == other.formatVersion &&
               bodyCount == other.bodyCount && hash == other.hash;
    }
    bool operator!=(const CanonicalHash& other) const { return !(*this == other); }
};

// Hashes the world's current dynamic body state (via
// captureCanonicalBodyState) with a 64-bit FNV-1a mix over each field's raw
// bit pattern. No RNG, no wall-clock, no pointer/address dependence: two
// processes that reach the same physical state produce the same hash.
CanonicalHash computeCanonicalHash(const World& world);

// Hashes an already-captured canonical body-state buffer (from
// captureCanonicalBodyState or SnapshotDelta::apply below), so callers that
// received bytes over the network do not need a World to compute a hash for
// comparison against a locally computed one.
CanonicalHash hashCanonicalBodyState(const std::vector<uint8_t>& bodyState);

// ---------------------------------------------------------------------------
// Compact snapshot delta
// ---------------------------------------------------------------------------

// A delta between two canonical body-state buffers of identical shape
// (same body count, same dense order — i.e., no bodies added/removed
// between base and target). Only bodies whose record differs are stored,
// alongside a dense bitmap identifying which indices changed. Encoding an
// unchanged world costs the bitmap only (bodyCount/8 bytes rounded up);
// nothing else scales with body count until something actually moved.
struct SnapshotDelta {
    uint32_t bodyCount = 0;
    std::vector<uint8_t> changedMask;   // ceil(bodyCount / 8) bytes, bit i = body i changed
    std::vector<uint8_t> changedRecords; // packed ReplayBodyState-sized records, changed bodies only, in index order
};

// Encodes the difference from `base` to `target` (both in
// captureCanonicalBodyState format, same body count). Throws
// VeloxInvalidArgument if the shapes differ — a body was added or removed,
// which this compact form cannot represent; send a full snapshot instead.
SnapshotDelta encodeDelta(const std::vector<uint8_t>& base,
                          const std::vector<uint8_t>& target);

// Reconstructs the target canonical body-state buffer by applying `delta`
// to `base`. Bitwise-reproduces the original `target` bytes passed to
// encodeDelta. Throws on a base/delta body-count mismatch or a truncated
// changedRecords buffer (defends against corrupted/malicious network input).
std::vector<uint8_t> applyDelta(const std::vector<uint8_t>& base,
                                const SnapshotDelta& delta);

// Number of bodies whose record differs (popcount of changedMask).
uint32_t changedBodyCount(const SnapshotDelta& delta);

// ---------------------------------------------------------------------------
// Bounded local rollback buffer
// ---------------------------------------------------------------------------

// A capacity-bounded ring buffer of WorldSnapshot, keyed by an
// application-defined frame number, for a client's own predicted history.
// Pushing past capacity evicts the oldest retained frame; memory is bounded
// by capacity * (one WorldSnapshot), never by how long the session has run.
//
// Snapshots are only valid to restore into the SAME World instance that
// produced them (WorldSnapshot's existing contract). To adopt a DIFFERENT
// World's authoritative state (e.g. the server's, over the network), use
// serializeWorld/deserializeWorld instead — see the header comment above.
class RollbackBuffer {
public:
    explicit RollbackBuffer(size_t capacity);

    // Captures `world`'s current snapshot under `frame`. If `frame` already
    // exists it is overwritten in place (does not consume capacity twice).
    // Frames must be pushed in non-decreasing order; evicts the oldest
    // retained frame once size() would exceed capacity().
    void push(uint64_t frame, const World& world);

    // Restores `world` (which MUST be the same instance every push() in
    // this buffer captured from) to the snapshot stored for `frame`.
    // Returns false, leaving `world` untouched, if `frame` is not retained
    // (evicted or never pushed).
    bool restore(uint64_t frame, World& world) const;

    bool contains(uint64_t frame) const;
    size_t size() const { return entries_.size(); }
    size_t capacity() const { return capacity_; }
    bool empty() const { return entries_.empty(); }
    uint64_t oldestFrame() const; // precondition: !empty()
    uint64_t newestFrame() const; // precondition: !empty()

private:
    struct Entry {
        uint64_t frame;
        WorldSnapshot snapshot;
    };
    std::deque<Entry> entries_;
    size_t capacity_;
};

// ---------------------------------------------------------------------------
// First-divergence diagnostics
// ---------------------------------------------------------------------------

// Identifies exactly where two recordings of the same nominal simulation
// first disagree: the 1-based frame, the body's dense index at capture
// time, which field (position/orientation/velocity/angularVelocity),
// and the magnitude of the disagreement on that field. `diverged` is false
// when both recordings match within tolerance for their entire common
// length (a length mismatch alone, with no field mismatch, is reported as
// diverged at the first frame beyond the shorter recording).
struct DivergenceReport {
    bool diverged = false;
    uint64_t frame = 0;       // 1-based, matching verifyReplay's convention
    uint32_t bodyIndex = 0;
    std::string field;        // "position" | "orientation" | "velocity" | "angularVelocity" | "bodyCount"
    float magnitude = 0.0f;   // Euclidean distance (or bodyCount delta) that exceeded tolerance
};

// Compares two ReplayRecording frame sequences (as produced by
// beginReplay/recordReplayFrame on two independently stepped Worlds) and
// reports the first field, on the first body, at the first frame where
// they disagree beyond the given tolerances.
DivergenceReport findFirstDivergence(const ReplayRecording& expected,
                                     const ReplayRecording& actual,
                                     float positionTolerance = 1e-5f,
                                     float velocityTolerance = 1e-4f,
                                     float orientationTolerance = 1e-5f);

} // namespace velox
