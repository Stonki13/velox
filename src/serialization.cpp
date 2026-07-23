#include "velox/serialization.h"

#include <cstring>
#include "velox/error.h"
#include <type_traits>

namespace velox {
namespace {

constexpr uint32_t kMagic = 0x584f4c56u; // 'VLOX' little-endian

// The blob is the WorldSnapshot's vectors dumped in a fixed order, each as
// (element size, count, raw bytes). Element sizes are validated on load so a
// struct layout change cannot silently misread an old file.
struct Writer {
    std::vector<uint8_t>& out;

    template <typename T>
    void writeRaw(const T& value) {
        // The archive format deliberately stores raw records for compactness.
        const size_t offset = out.size();
        out.resize(offset + sizeof(T));
        std::memcpy(out.data() + offset, &value, sizeof(T));
    }

    template <typename T>
    void pod(const T& value) {
        static_assert(std::is_trivially_copyable<T>::value, "POD only");
        // Most serialized records have no padding. Keep the existing compact
        // native layout, but start from a known representation for records
        // that do carry padding (Body has its own field-wise overload below).
        T canonical{};
        canonical = value;
        writeRaw(canonical);
    }

    void pod(const Body& value) {
        Body canonical{};
        std::memset(&canonical, 0, sizeof(canonical));
        canonical.position = value.position;
        canonical.orientation = value.orientation;
        canonical.velocity = value.velocity;
        canonical.angularVelocity = value.angularVelocity;
        canonical.force = value.force;
        canonical.torque = value.torque;
        canonical.invMass = value.invMass;
        canonical.invInertia = value.invInertia;
        canonical.inertiaOrientation = value.inertiaOrientation;
        canonical.restitution = value.restitution;
        canonical.friction = value.friction;
        canonical.frictionScale = value.frictionScale;
        canonical.rollingFriction = value.rollingFriction;
        canonical.spinningFriction = value.spinningFriction;
        canonical.frictionCombine = value.frictionCombine;
        canonical.restitutionCombine = value.restitutionCombine;
        canonical.linearDamping = value.linearDamping;
        canonical.angularDamping = value.angularDamping;
        canonical.gravityScale = value.gravityScale;
        canonical.motionType = value.motionType;
        canonical.ccdTuning.quality = value.ccdTuning.quality;
        canonical.ccdTuning.collisionMargin = value.ccdTuning.collisionMargin;
        canonical.ccdTuning.speculativeDistance = value.ccdTuning.speculativeDistance;
        canonical.ccdTuning.enableContinuous = value.ccdTuning.enableContinuous;
        canonical.ccdTuning.minVelocityForCCD = value.ccdTuning.minVelocityForCCD;
        canonical.ccdTuning.ccdSettings.quality = value.ccdTuning.ccdSettings.quality;
        canonical.ccdTuning.ccdSettings.maxAdvancementIterations =
            value.ccdTuning.ccdSettings.maxAdvancementIterations;
        canonical.ccdTuning.ccdSettings.maxRootFindIterations =
            value.ccdTuning.ccdSettings.maxRootFindIterations;
        canonical.ccdTuning.ccdSettings.distanceTolerance =
            value.ccdTuning.ccdSettings.distanceTolerance;
        canonical.ccdTuning.ccdSettings.timeTolerance = value.ccdTuning.ccdSettings.timeTolerance;
        canonical.ccdTuning.ccdSettings.enableRotationalCcd =
            value.ccdTuning.ccdSettings.enableRotationalCcd;
        canonical.ccdTuning.ccdSettings.enableCompoundCcd =
            value.ccdTuning.ccdSettings.enableCompoundCcd;
        canonical.ccdTuning.ccdSettings.useBrentRootFinder =
            value.ccdTuning.ccdSettings.useBrentRootFinder;
        canonical.ccdTuning.ccdSettings.meshBvhMaxDepth =
            value.ccdTuning.ccdSettings.meshBvhMaxDepth;
        canonical.ccdTuning.ccdSettings.speculativeMargin =
            value.ccdTuning.ccdSettings.speculativeMargin;
        canonical.geometryQuality = value.geometryQuality;
        canonical.categoryBits = value.categoryBits;
        canonical.maskBits = value.maskBits;
        canonical.groupIndex = value.groupIndex;
        canonical.sensor = value.sensor;
        canonical.enableSleep = value.enableSleep;
        canonical.fixedRotation = value.fixedRotation;
        canonical.asleep = value.asleep;
        canonical.sleepTimer = value.sleepTimer;
        canonical.shape = value.shape;
        canonical.radius = value.radius;
        canonical.halfExtents = value.halfExtents;
        canonical.capsuleHalfHeight = value.capsuleHalfHeight;
        canonical.planeNormal = value.planeNormal;
        canonical.planeOffset = value.planeOffset;
        canonical.meshIndex = value.meshIndex;
        canonical.hullFirst = value.hullFirst;
        canonical.hullCount = value.hullCount;
        canonical.hullFaceFirst = value.hullFaceFirst;
        canonical.hullFaceCount = value.hullFaceCount;
        canonical.compoundFirst = value.compoundFirst;
        canonical.compoundCount = value.compoundCount;
        writeRaw(canonical);
    }

    void pod(const Contact& value) {
        Contact canonical{};
        std::memset(&canonical, 0, sizeof(canonical));
        canonical.a = value.a;
        canonical.b = value.b;
        canonical.featureKey = value.featureKey;
        canonical.normal = value.normal;
        canonical.point = value.point;
        canonical.localAnchorA = value.localAnchorA;
        canonical.localAnchorB = value.localAnchorB;
        canonical.gap = value.gap;
        canonical.bias0 = value.bias0;
        canonical.vn0 = value.vn0;
        canonical.restitution = value.restitution;
        canonical.friction1 = value.friction1;
        canonical.friction2 = value.friction2;
        canonical.rollingFriction = value.rollingFriction;
        canonical.spinningFriction = value.spinningFriction;
        canonical.normalImpulse = value.normalImpulse;
        canonical.tangentImpulse1 = value.tangentImpulse1;
        canonical.tangentImpulse2 = value.tangentImpulse2;
        canonical.rollingImpulse1 = value.rollingImpulse1;
        canonical.rollingImpulse2 = value.rollingImpulse2;
        canonical.spinningImpulse = value.spinningImpulse;
        writeRaw(canonical);
    }

    void pod(const Joint& value) {
        Joint canonical{};
        std::memset(&canonical, 0, sizeof(canonical));
        canonical.type = value.type;
        canonical.a = value.a;
        canonical.b = value.b;
        canonical.localAnchorA = value.localAnchorA;
        canonical.localAnchorB = value.localAnchorB;
        canonical.localAxisA = value.localAxisA;
        canonical.localAxisB = value.localAxisB;
        canonical.localRefA = value.localRefA;
        canonical.localRefB = value.localRefB;
        canonical.restLength = value.restLength;
        canonical.collideConnected = value.collideConnected;
        canonical.enableSpring = value.enableSpring;
        canonical.springFrequencyHz = value.springFrequencyHz;
        canonical.springDampingRatio = value.springDampingRatio;
        canonical.breakForce = value.breakForce;
        canonical.breakTorque = value.breakTorque;
        canonical.enableMotor = value.enableMotor;
        canonical.motorSpeed = value.motorSpeed;
        canonical.maxMotorTorque = value.maxMotorTorque;
        canonical.maxMotorForce = value.maxMotorForce;
        canonical.enableLimit = value.enableLimit;
        canonical.lowerLimit = value.lowerLimit;
        canonical.upperLimit = value.upperLimit;
        canonical.enableSwingLimit = value.enableSwingLimit;
        canonical.swingLimit = value.swingLimit;
        canonical.enableTwistLimit = value.enableTwistLimit;
        canonical.lowerTwistLimit = value.lowerTwistLimit;
        canonical.upperTwistLimit = value.upperTwistLimit;
        canonical.linearLimitMask = value.linearLimitMask;
        canonical.angularLimitMask = value.angularLimitMask;
        canonical.linearMotorMask = value.linearMotorMask;
        canonical.angularMotorMask = value.angularMotorMask;
        canonical.lowerLinearLimit = value.lowerLinearLimit;
        canonical.upperLinearLimit = value.upperLinearLimit;
        canonical.lowerAngularLimit = value.lowerAngularLimit;
        canonical.upperAngularLimit = value.upperAngularLimit;
        canonical.linearMotorSpeed = value.linearMotorSpeed;
        canonical.angularMotorSpeed = value.angularMotorSpeed;
        canonical.maxLinearMotorForce = value.maxLinearMotorForce;
        canonical.maxAngularMotorTorque = value.maxAngularMotorTorque;
        canonical.weldFrequencyHz = value.weldFrequencyHz;
        canonical.weldDampingRatio = value.weldDampingRatio;
        canonical.suspensionFrequencyHz = value.suspensionFrequencyHz;
        canonical.suspensionDampingRatio = value.suspensionDampingRatio;
        canonical.wheelRadius = value.wheelRadius;
        canonical.enableSteering = value.enableSteering;
        canonical.steeringAngle = value.steeringAngle;
        canonical.maxSteeringAngle = value.maxSteeringAngle;
        canonical.maxLength = value.maxLength;
        canonical.pulleyAnchorA = value.pulleyAnchorA;
        canonical.pulleyAnchorB = value.pulleyAnchorB;
        canonical.pulleyRatio = value.pulleyRatio;
        canonical.gearRatio = value.gearRatio;
        canonical.motorImpulse = value.motorImpulse;
        canonical.limitImpulse = value.limitImpulse;
        canonical.swingImpulse = value.swingImpulse;
        canonical.twistImpulse = value.twistImpulse;
        canonical.springImpulse = value.springImpulse;
        canonical.linearMotorImpulse = value.linearMotorImpulse;
        canonical.angularMotorImpulse = value.angularMotorImpulse;
        canonical.linearLimitImpulse = value.linearLimitImpulse;
        canonical.angularLimitImpulse = value.angularLimitImpulse;
        canonical.reactionLinearImpulse = value.reactionLinearImpulse;
        canonical.reactionAngularImpulse = value.reactionAngularImpulse;
        canonical.broken = value.broken;
        writeRaw(canonical);
    }

    template <typename T>
    void vec(const std::vector<T>& values) {
        static_assert(std::is_trivially_copyable<T>::value, "POD only");
        pod(static_cast<uint64_t>(sizeof(T)));
        pod(static_cast<uint64_t>(values.size()));
        out.reserve(out.size() + values.size() * sizeof(T));
        for (const T& value : values) pod(value);
    }
};

struct Reader {
    const uint8_t* cursor;
    const uint8_t* end;

    void need(size_t bytes) const {
        if (static_cast<size_t>(end - cursor) < bytes)
            VELOX_THROW(VeloxRuntimeError, ErrorCode::SerializedTruncated,
                        "truncated serialized scene");
    }

    template <typename T>
    void pod(T& value) {
        static_assert(std::is_trivially_copyable<T>::value, "POD only");
        need(sizeof(T));
        std::memcpy(&value, cursor, sizeof(T));
        cursor += sizeof(T);
    }

    template <typename T>
    void vec(std::vector<T>& values) {
        static_assert(std::is_trivially_copyable<T>::value, "POD only");
        uint64_t elementSize = 0, count = 0;
        pod(elementSize);
        pod(count);
        if (elementSize != sizeof(T))
            VELOX_THROW(VeloxRuntimeError, ErrorCode::SerializedLayoutMismatch,
                        "serialized scene layout does not match this build");
        const size_t bytes = static_cast<size_t>(count) * sizeof(T);
        need(bytes);
        values.resize(static_cast<size_t>(count));
        if (bytes) std::memcpy(values.data(), cursor, bytes);
        cursor += bytes;
    }
};

// Per-frame replay sample for one body (dense order).
struct ReplayBodyState {
    Vec3 position;
    Quat orientation;
    Vec3 velocity;
    Vec3 angularVelocity;
};

} // namespace

// Friend of World and WorldSnapshot: converts between snapshots and bytes and
// reads dense body state for replay capture.
struct SerializationAccess {
    static std::vector<uint8_t> encode(const WorldSnapshot& snapshot) {
        std::vector<uint8_t> bytes;
        Writer writer{bytes};
        writer.pod(snapshot.gravity_);
        writer.pod(snapshot.substeps_);
        writer.pod(snapshot.ccdDefaults_);
        writer.pod(snapshot.multiToiSettings_);
        writer.pod(snapshot.solverOptions_);
        writer.vec(snapshot.bodies_);
        writer.vec(snapshot.bodySlots_);
        writer.vec(snapshot.bodyDenseToSlot_);
        writer.vec(snapshot.freeBodySlots_);
        writer.vec(snapshot.contacts_);
        writer.vec(snapshot.prevContacts_);
        writer.vec(snapshot.joints_);
        writer.vec(snapshot.jointSlots_);
        writer.vec(snapshot.jointDenseToSlot_);
        writer.vec(snapshot.freeJointSlots_);
        writer.vec(snapshot.previous_);
        writer.vec(snapshot.pairKeys_);
        writer.vec(snapshot.previousPairKeys_);
        writer.vec(snapshot.unionParent_);
        writer.vec(snapshot.islandTimer_);
        writer.vec(snapshot.contactEvents_);
        writer.vec(snapshot.jointBreakEvents_);
        writer.vec(snapshot.meshes_.vertices);
        writer.vec(snapshot.meshes_.indices);
        writer.vec(snapshot.meshes_.meshes);
        writer.vec(snapshot.meshes_.bvhNodes);
        writer.vec(snapshot.meshes_.bvhTriRefs);
        writer.vec(snapshot.meshes_.hullPoints);
        writer.vec(snapshot.meshes_.hullFaceIndices);
        writer.vec(snapshot.meshes_.compoundChildren);
        writer.pod(snapshot.lastStepStats_);
        return bytes;
    }

    static WorldSnapshot decode(const std::vector<uint8_t>& bytes, World& owner) {
        WorldSnapshot snapshot;
        Reader reader{bytes.data(), bytes.data() + bytes.size()};
        reader.pod(snapshot.gravity_);
        reader.pod(snapshot.substeps_);
        reader.pod(snapshot.ccdDefaults_);
        reader.pod(snapshot.multiToiSettings_);
        reader.pod(snapshot.solverOptions_);
        reader.vec(snapshot.bodies_);
        reader.vec(snapshot.bodySlots_);
        reader.vec(snapshot.bodyDenseToSlot_);
        reader.vec(snapshot.freeBodySlots_);
        reader.vec(snapshot.contacts_);
        reader.vec(snapshot.prevContacts_);
        reader.vec(snapshot.joints_);
        reader.vec(snapshot.jointSlots_);
        reader.vec(snapshot.jointDenseToSlot_);
        reader.vec(snapshot.freeJointSlots_);
        reader.vec(snapshot.previous_);
        reader.vec(snapshot.pairKeys_);
        reader.vec(snapshot.previousPairKeys_);
        reader.vec(snapshot.unionParent_);
        reader.vec(snapshot.islandTimer_);
        reader.vec(snapshot.contactEvents_);
        reader.vec(snapshot.jointBreakEvents_);
        reader.vec(snapshot.meshes_.vertices);
        reader.vec(snapshot.meshes_.indices);
        reader.vec(snapshot.meshes_.meshes);
        reader.vec(snapshot.meshes_.bvhNodes);
        reader.vec(snapshot.meshes_.bvhTriRefs);
        reader.vec(snapshot.meshes_.hullPoints);
        reader.vec(snapshot.meshes_.hullFaceIndices);
        reader.vec(snapshot.meshes_.compoundChildren);
        reader.pod(snapshot.lastStepStats_);
        if (reader.cursor != reader.end)
            VELOX_THROW(VeloxRuntimeError, ErrorCode::SerializedTrailingBytes,
                        "trailing bytes in serialized scene");
        // Snapshots bind to their originating world; a deserialized snapshot
        // adopts the target so restoreSnapshot accepts it.
        snapshot.owner_ = &owner;
        return snapshot;
    }

    static std::vector<uint8_t> captureBodies(const World& world) {
        std::vector<uint8_t> bytes;
        bytes.reserve(world.bodies_.size() * sizeof(ReplayBodyState));
        Writer writer{bytes};
        for (const Body& body : world.bodies_)
            writer.pod(ReplayBodyState{body.position, body.orientation,
                                       body.velocity, body.angularVelocity});
        return bytes;
    }

    static bool compareBodies(const World& world, const std::vector<uint8_t>& bytes,
                              float positionTolerance, float velocityTolerance) {
        if (bytes.size() != world.bodies_.size() * sizeof(ReplayBodyState))
            return false;
        const uint8_t* cursor = bytes.data();
        for (const Body& body : world.bodies_) {
            ReplayBodyState recorded;
            std::memcpy(&recorded, cursor, sizeof(recorded));
            cursor += sizeof(recorded);
            if (length(body.position - recorded.position) > positionTolerance ||
                length(body.velocity - recorded.velocity) > velocityTolerance ||
                length(body.angularVelocity - recorded.angularVelocity) >
                    velocityTolerance)
                return false;
        }
        return true;
    }
};

SerializedScene serializeWorld(const World& world, std::string metadata) {
    SerializedScene scene;
    scene.metadata = std::move(metadata);
    scene.data = SerializationAccess::encode(world.saveSnapshot());
    return scene;
}

void deserializeWorld(World& world, const SerializedScene& scene) {
    if (scene.version != kSerializationVersion)
        VELOX_THROW(VeloxRuntimeError, ErrorCode::SerializedVersionMismatch,
                    "serialized scene version " +
                    std::to_string(scene.version) +
                    " is not supported by this build (expected " +
                    std::to_string(kSerializationVersion) + ")");
    world.restoreSnapshot(SerializationAccess::decode(scene.data, world));
}

std::vector<uint8_t> packScene(const SerializedScene& scene, CompressionMode mode) {
    if (mode != CompressionMode::None)
        VELOX_THROW(VeloxRuntimeError, ErrorCode::Unknown,
                    "this build was compiled without LZ4/ZSTD support");
    std::vector<uint8_t> bytes;
    Writer writer{bytes};
    writer.pod(kMagic);
    writer.pod(scene.version);
    writer.pod(static_cast<uint8_t>(mode));
    writer.pod(static_cast<uint64_t>(scene.metadata.size()));
    bytes.insert(bytes.end(), scene.metadata.begin(), scene.metadata.end());
    writer.pod(static_cast<uint64_t>(scene.data.size()));
    bytes.insert(bytes.end(), scene.data.begin(), scene.data.end());
    return bytes;
}

SerializedScene unpackScene(const std::vector<uint8_t>& bytes) {
    Reader reader{bytes.data(), bytes.data() + bytes.size()};
    uint32_t magic = 0;
    reader.pod(magic);
    if (magic != kMagic)
        VELOX_THROW(VeloxRuntimeError, ErrorCode::SerializedBadMagic,
                    "not a serialized velox scene");
    SerializedScene scene;
    reader.pod(scene.version);
    if (scene.version != kSerializationVersion)
        VELOX_THROW(VeloxRuntimeError, ErrorCode::SerializedVersionMismatch,
                    "serialized scene version " +
                    std::to_string(scene.version) +
                    " is not supported by this build (expected " +
                    std::to_string(kSerializationVersion) + ")");
    uint8_t mode = 0;
    reader.pod(mode);
    if (mode != static_cast<uint8_t>(CompressionMode::None))
        VELOX_THROW(VeloxRuntimeError, ErrorCode::Unknown,
                    "this build was compiled without LZ4/ZSTD support");
    uint64_t metadataSize = 0;
    reader.pod(metadataSize);
    reader.need(static_cast<size_t>(metadataSize));
    scene.metadata.assign(reinterpret_cast<const char*>(reader.cursor),
                          static_cast<size_t>(metadataSize));
    reader.cursor += metadataSize;
    uint64_t dataSize = 0;
    reader.pod(dataSize);
    reader.need(static_cast<size_t>(dataSize));
    scene.data.assign(reader.cursor, reader.cursor + dataSize);
    return scene;
}

void beginReplay(ReplayRecording& recording, const World& world, float dt) {
    recording.initialScene = serializeWorld(world, "velox replay");
    recording.dt = dt;
    recording.frames.clear();
}

void recordReplayFrame(ReplayRecording& recording, const World& world) {
    recording.frames.push_back(SerializationAccess::captureBodies(world));
}

uint64_t verifyReplay(const ReplayRecording& recording, float positionTolerance,
                      float velocityTolerance) {
    World world(BackendType::Cpu);
    deserializeWorld(world, recording.initialScene);
    for (size_t frame = 0; frame < recording.frames.size(); ++frame) {
        world.step(recording.dt);
        if (!SerializationAccess::compareBodies(world, recording.frames[frame],
                                                positionTolerance, velocityTolerance))
            return frame + 1;
    }
    return 0;
}

std::vector<uint8_t> captureCanonicalBodyState(const World& world) {
    return SerializationAccess::captureBodies(world);
}

} // namespace velox
