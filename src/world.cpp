#include "velox/world.h"
#include "velox/profiler.h"
#include "broadphase.h"
#include "geometry_diagnostics.h"
#include "mass_properties.h"
#include "narrowphase.h"
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <stdexcept>
#include <string>

namespace velox {

namespace {

bool finiteFloat(float value) { return std::isfinite(value); }
bool finiteVec(const Vec3& v) {
    return finiteFloat(v.x) && finiteFloat(v.y) && finiteFloat(v.z);
}
bool finiteQuat(const Quat& q) {
    return finiteFloat(q.x) && finiteFloat(q.y) &&
           finiteFloat(q.z) && finiteFloat(q.w);
}

void rejectDuplicateHullPoints(const std::vector<Vec3>& points, const char* message) {
    constexpr float kDuplicateDistanceSq = 1e-16f;
    for (size_t i = 0; i < points.size(); ++i)
        for (size_t j = 0; j < i; ++j)
            if (lengthSq(points[i] - points[j]) <= kDuplicateDistanceSq)
                throw std::invalid_argument(message);
}

bool validCombineMode(MaterialCombineMode mode) {
    return (uint8_t)mode <= (uint8_t)MaterialCombineMode::Maximum;
}

bool validMotionQuality(MotionQuality quality) {
    return (uint8_t)quality <= (uint8_t)MotionQuality::Locked;
}

void validateCcdTuning(const BodyCcdTuning& tuning) {
    if (!validMotionQuality(tuning.quality) ||
        !finiteFloat(tuning.collisionMargin) || tuning.collisionMargin < 0.0f ||
        !finiteFloat(tuning.speculativeDistance) || tuning.speculativeDistance < 0.0f ||
        !finiteFloat(tuning.minVelocityForCCD) || tuning.minVelocityForCCD < 0.0f)
        throw std::invalid_argument("velox: body CCD tuning is invalid");
}

void validateCcdDefaults(const WorldCcdDefaults& defaults) {
    BodyCcdTuning tuning;
    tuning.quality = defaults.defaultQuality;
    tuning.collisionMargin = defaults.defaultCollisionMargin;
    tuning.speculativeDistance = defaults.defaultSpeculativeDistance;
    tuning.enableContinuous = defaults.defaultEnableContinuous;
    tuning.minVelocityForCCD = defaults.defaultMinVelocityForCCD;
    validateCcdTuning(tuning);
}

void validateMultiToiSettings(const WorldMultiToiSettings& settings) {
    const CcdConfig& config = settings.defaultConfig;
    if (config.maxToiEventsPerBody == 0 ||
        !finiteFloat(config.toiVelocityFloor) || config.toiVelocityFloor < 0.0f ||
        !finiteFloat(config.toiPenetrationBias) || config.toiPenetrationBias < 0.0f ||
        settings.maxTotalEventsPerStep == 0)
        throw std::invalid_argument("velox: multi-TOI settings are invalid");
}

void validateSolverOptions(const SolverOptions& options) {
    if ((options.frictionModel != FrictionModel::TwoAxisCoulomb &&
         options.frictionModel != FrictionModel::ConeBlockSolver) ||
        (options.iterationPolicy != IterationPolicy::Fixed &&
         options.iterationPolicy != IterationPolicy::Adaptive) ||
        options.velocityIterations <= 0 || options.positionIterations <= 0 ||
        !finiteFloat(options.impulseThreshold) || options.impulseThreshold <= 0.0f ||
        !finiteFloat(options.stackStiffness) || options.stackStiffness < 0.0f ||
        !finiteFloat(options.stackDamping) || options.stackDamping < 0.0f)
        throw std::invalid_argument("velox: solver options are invalid");
}

bool validJointType(JointType type) {
    return (uint8_t)type <= (uint8_t)JointType::Gear;
}

void jointFrame(const Joint& joint, const Body& a, const Body& b,
                Vec3& xA, Vec3& yA, Vec3& zA,
                Vec3& xB, Vec3& yB, Vec3& zB) {
    xA = normalize(rotate(a.orientation, joint.localAxisA));
    yA = normalize(rotate(a.orientation, joint.localRefA));
    zA = normalize(cross(xA, yA));
    yA = cross(zA, xA);
    xB = normalize(rotate(b.orientation, joint.localAxisB));
    yB = normalize(rotate(b.orientation, joint.localRefB));
    zB = normalize(cross(xB, yB));
    yB = cross(zB, xB);
}

Quat quaternionFromRotationMatrix(float m00, float m01, float m02,
                                  float m10, float m11, float m12,
                                  float m20, float m21, float m22) {
    Quat q;
    float trace = m00 + m11 + m22;
    if (trace > 0.0f) {
        float s = std::sqrt(trace + 1.0f) * 2.0f;
        q = {(m21 - m12) / s, (m02 - m20) / s,
             (m10 - m01) / s, 0.25f * s};
    } else if (m00 > m11 && m00 > m22) {
        float s = std::sqrt(1.0f + m00 - m11 - m22) * 2.0f;
        q = {0.25f * s, (m01 + m10) / s, (m02 + m20) / s,
             (m21 - m12) / s};
    } else if (m11 > m22) {
        float s = std::sqrt(1.0f + m11 - m00 - m22) * 2.0f;
        q = {(m01 + m10) / s, 0.25f * s, (m12 + m21) / s,
             (m02 - m20) / s};
    } else {
        float s = std::sqrt(1.0f + m22 - m00 - m11) * 2.0f;
        q = {(m02 + m20) / s, (m12 + m21) / s, 0.25f * s,
             (m10 - m01) / s};
    }
    return normalize(q);
}

Vec3 sixDofRotationVector(const Joint& joint, const Body& a, const Body& b) {
    Vec3 xA, yA, zA, xB, yB, zB;
    jointFrame(joint, a, b, xA, yA, zA, xB, yB, zB);
    Quat q = quaternionFromRotationMatrix(
        dot(xA, xB), dot(xA, yB), dot(xA, zB),
        dot(yA, xB), dot(yA, yB), dot(yA, zB),
        dot(zA, xB), dot(zA, yB), dot(zA, zB));
    if (q.w < 0.0f) q = {-q.x, -q.y, -q.z, -q.w};
    Vec3 v{q.x, q.y, q.z};
    float sinHalf = length(v);
    if (sinHalf < 1e-7f) return v * 2.0f;
    float angle = 2.0f * std::atan2(sinHalf, vclamp(q.w, 0.0f, 1.0f));
    return v * (angle / sinHalf);
}

float& component(Vec3& v, int axis) {
    return axis == 0 ? v.x : (axis == 1 ? v.y : v.z);
}

void requireFiniteVec(const Vec3& value, const char* message) {
    if (!finiteVec(value)) throw std::invalid_argument(message);
}

void requireMass(float mass) {
    if (!finiteFloat(mass) || mass < 0.0f)
        throw std::invalid_argument("velox: mass must be finite and non-negative");
}

void requirePositive(float value, const char* message) {
    if (!finiteFloat(value) || value <= 0.0f) throw std::invalid_argument(message);
}

void requireNonNegative(float value, const char* message) {
    if (!finiteFloat(value) || value < 0.0f) throw std::invalid_argument(message);
}

void refreshPrimitiveInertia(Body& body) {
    if (body.invMass <= 0.0f) return;
    const float mass = 1.0f / body.invMass;
    float ix = 0.0f, iy = 0.0f, iz = 0.0f;
    switch (body.shape) {
    case ShapeType::Sphere:
        ix = iy = iz = 0.4f * mass * body.radius * body.radius;
        break;
    case ShapeType::Box: {
        Vec3 e = body.halfExtents * 2.0f;
        float k = mass / 12.0f;
        ix = k * (e.y * e.y + e.z * e.z);
        iy = k * (e.x * e.x + e.z * e.z);
        iz = k * (e.x * e.x + e.y * e.y);
        break;
    }
    case ShapeType::Cylinder: {
        float height = 2.0f * body.capsuleHalfHeight;
        iy = 0.5f * mass * body.radius * body.radius;
        ix = iz = mass * (3.0f * body.radius * body.radius + height * height) / 12.0f;
        break;
    }
    case ShapeType::Cone: {
        float height = 2.0f * body.capsuleHalfHeight;
        iy = 0.3f * mass * body.radius * body.radius;
        ix = iz = 0.15f * mass * body.radius * body.radius + 0.0375f * mass * height * height;
        break;
    }
    case ShapeType::Capsule: {
        float r = body.radius, h = body.capsuleHalfHeight;
        float cylinderWeight = 2.0f * h, sphereWeight = (4.0f / 3.0f) * r;
        float cylinderMass = mass * cylinderWeight / (cylinderWeight + sphereWeight);
        float sphereMass = mass - cylinderMass;
        iy = 0.5f * cylinderMass * r * r + 0.4f * sphereMass * r * r;
        ix = iz = cylinderMass * (3.0f * r * r + 4.0f * h * h) / 12.0f +
                  sphereMass * (0.4f * r * r + 0.75f * h * r + h * h);
        break;
    }
    case ShapeType::RoundedBox: {
        Vec3 e = body.halfExtents * 2.0f;
        float k = mass / 12.0f;
        ix = k * (e.y * e.y + e.z * e.z);
        iy = k * (e.x * e.x + e.z * e.z);
        iz = k * (e.x * e.x + e.y * e.y);
        break;
    }
    case ShapeType::Ellipsoid: {
        float a = body.halfExtents.x, b = body.halfExtents.y, c = body.halfExtents.z;
        ix = 0.2f * mass * (b * b + c * c);
        iy = 0.2f * mass * (a * a + c * c);
        iz = 0.2f * mass * (a * a + b * b);
        break;
    }
    default:
        return;
    }
    body.invInertia = {1.0f / ix, 1.0f / iy, 1.0f / iz};
    body.inertiaOrientation = {};
}

void validateRuntimeBody(const Body& body) {
    if (!finiteVec(body.position) || !finiteQuat(body.orientation) ||
        !finiteQuat(body.inertiaOrientation) ||
        !finiteVec(body.velocity) || !finiteVec(body.angularVelocity) ||
        !finiteVec(body.force) || !finiteVec(body.torque) ||
        !finiteFloat(body.invMass) || body.invMass < 0.0f ||
        !finiteVec(body.invInertia) || body.invInertia.x < 0.0f ||
        body.invInertia.y < 0.0f || body.invInertia.z < 0.0f ||
        !finiteFloat(body.restitution) ||
        body.restitution < 0.0f || !finiteFloat(body.friction) || body.friction < 0.0f ||
        !finiteVec(body.frictionScale) || body.frictionScale.x < 0.0f ||
        body.frictionScale.y < 0.0f || body.frictionScale.z < 0.0f ||
        !finiteFloat(body.rollingFriction) || body.rollingFriction < 0.0f ||
        !finiteFloat(body.spinningFriction) || body.spinningFriction < 0.0f ||
        !validCombineMode(body.frictionCombine) ||
        !validCombineMode(body.restitutionCombine) ||
        !finiteFloat(body.linearDamping) || body.linearDamping < 0.0f ||
        !finiteFloat(body.angularDamping) || body.angularDamping < 0.0f ||
        !finiteFloat(body.gravityScale))
        throw std::invalid_argument("velox: body contains invalid or non-finite state");
    validateCcdTuning(body.ccdTuning);
    float q2 = body.orientation.x * body.orientation.x +
               body.orientation.y * body.orientation.y +
               body.orientation.z * body.orientation.z +
               body.orientation.w * body.orientation.w;
    if (!finiteFloat(q2) || q2 < 1e-12f)
        throw std::invalid_argument("velox: body orientation must be a finite non-zero quaternion");
    float iq2 = body.inertiaOrientation.x * body.inertiaOrientation.x +
                body.inertiaOrientation.y * body.inertiaOrientation.y +
                body.inertiaOrientation.z * body.inertiaOrientation.z +
                body.inertiaOrientation.w * body.inertiaOrientation.w;
    if (!finiteFloat(iq2) || iq2 < 1e-12f)
        throw std::invalid_argument(
            "velox: inertia orientation must be a finite non-zero quaternion");
}

} // namespace

struct World::StepRollback {
    std::vector<Body> bodies;
    std::vector<Contact> contacts, previousContacts;
    std::vector<Joint> joints;
    std::vector<PrevState> previous;
    std::vector<uint64_t> pairKeys, previousPairKeys;
    std::vector<uint32_t> unionParent;
    std::vector<float> islandTimer;
    std::vector<ContactEvent> events;
    std::vector<JointBreakEvent> jointBreakEvents;
    StepStats lastStepStats;
};

World::~World() = default;

World::StepRollback World::saveStepRollback() const {
    StepRollback rollback;
    rollback.bodies = bodies_;
    rollback.contacts = contacts_;
    rollback.previousContacts = prevContacts_;
    rollback.joints = joints_;
    rollback.previous = prev_;
    rollback.pairKeys = pairKeys_;
    rollback.previousPairKeys = prevPairKeys_;
    rollback.unionParent = unionParent_;
    rollback.islandTimer = islandTimer_;
    rollback.events = events_;
    rollback.jointBreakEvents = jointBreakEvents_;
    rollback.lastStepStats = lastStepStats_;
    return rollback;
}

void World::restoreStepRollback(StepRollback&& rollback) {
    bodies_ = std::move(rollback.bodies);
    contacts_ = std::move(rollback.contacts);
    prevContacts_ = std::move(rollback.previousContacts);
    joints_ = std::move(rollback.joints);
    prev_ = std::move(rollback.previous);
    pairKeys_ = std::move(rollback.pairKeys);
    prevPairKeys_ = std::move(rollback.previousPairKeys);
    unionParent_ = std::move(rollback.unionParent);
    islandTimer_ = std::move(rollback.islandTimer);
    events_ = std::move(rollback.events);
    jointBreakEvents_ = std::move(rollback.jointBreakEvents);
    lastStepStats_ = rollback.lastStepStats;
    broadPhase_->structureDirty = true;
    broadPhase_->touched = true;
    backend_->invalidateCaches();
}

World::AccessGuard::AccessGuard(const World& world, AccessKind kind,
                                const char* method)
    : world_(world), lock_(world.accessMutex_) {
    const bool owner = std::this_thread::get_id() == world_.ownerThread_;
    if (!owner && kind == AccessKind::Query)
        ++world_.threadSafetyReport_.queryCallsFromNonMainThread;
    if (!owner && kind == AccessKind::Step) {
        ++world_.threadSafetyReport_.stepInvocationsOnNonMainThread;
        throw std::logic_error(std::string("velox: step() must be called by the World owner thread; ") +
                               method + " was called from another thread");
    }
    if (!owner && kind == AccessKind::Mutation &&
        world_.threadSafetyPolicy_ != ThreadSafetyPolicy::Concurrent) {
        ++world_.threadSafetyReport_.mutationCallsDuringStep;
        throw std::logic_error(std::string("velox: mutation from another thread requires ") +
                               "ThreadSafetyPolicy::Concurrent (" + method + ")");
    }
    if (!owner && kind == AccessKind::Query &&
        world_.threadSafetyPolicy_ == ThreadSafetyPolicy::Strict)
        throw std::logic_error(std::string("velox: cross-thread query requires ") +
                               "ThreadSafetyPolicy::Relaxed or Concurrent (" + method + ")");
}

void World::resetBackend(BackendType type) {
    std::unique_ptr<Backend> next;
    if (type == BackendType::Vulkan) {
        next.reset(createVulkanBackend());
        if (!next)
            throw std::runtime_error("velox: Vulkan backend unavailable "
                                     "(not built with VELOX_ENABLE_VULKAN, no "
                                     "Vulkan driver, or no compute-capable device)");
    } else if (type != BackendType::Cpu) {
        next.reset(createCudaBackend());
    }
    if (!next) {
        if (type == BackendType::Cuda)
            throw std::runtime_error("velox: CUDA backend unavailable "
                                     "(not built with VELOX_ENABLE_CUDA or no device)");
        next.reset(createCpuBackend());
    }
    next->setParallelIslands(determinismMode_ == DeterminismMode::Relaxed &&
                             islandSolvingMode_ == IslandSolvingMode::Parallel);
    backend_ = std::move(next);
}

void World::activateCpuFallback(const BackendFailure& failure) {
    std::fprintf(stderr, "velox: CUDA backend failure (%s); retrying frame on CPU\n",
                 failure.what());
    resetBackend(BackendType::Cpu);
    fallbackToCPU_ = true;
}

World::World(BackendType type)
    : requestedBackend_(type), broadPhase_(new BroadPhaseData),
      ownerThread_(std::this_thread::get_id()) {
    resetBackend(type);
    // Pre-reserve the dense arrays so a typical scene steps without ever
    // reallocating bodies, contacts, or joints. The cost is paid once here,
    // off the hot path, instead of in amortized vector growth during step().
    reserveCapacity(256, 4096, 128);
}

void World::reserveCapacity(size_t bodies, size_t contacts, size_t joints) {
    AccessGuard guard(*this, AccessKind::Mutation, "reserveCapacity");
    if (bodies_.capacity() < bodies) bodies_.reserve(bodies);
    if (bodySlots_.capacity() < bodies) bodySlots_.reserve(bodies);
    if (bodyDenseToSlot_.capacity() < bodies) bodyDenseToSlot_.reserve(bodies);
    if (contacts_.capacity() < contacts) contacts_.reserve(contacts);
    if (prevContacts_.capacity() < contacts) prevContacts_.reserve(contacts);
    if (joints_.capacity() < joints) joints_.reserve(joints);
    if (jointSlots_.capacity() < joints) jointSlots_.reserve(joints);
    if (jointDenseToSlot_.capacity() < joints) jointDenseToSlot_.reserve(joints);
}

MemoryPoolStats World::memoryStats() const {
    AccessGuard guard(*this, AccessKind::Query, "memoryStats");
    MemoryPoolStats stats = memoryPool_.stats();
    // Account the pre-reserved dense arrays. These are not pool-allocated, but
    // they are the dominant per-record memory and the thing the pools exist to
    // keep from reallocating, so report them alongside the slab accounting.
    auto addVector = [&stats](size_t capacity, size_t size, size_t elemSize) {
        stats.reservedBytes += capacity * elemSize;
        stats.usedBytes += size * elemSize;
        stats.requestedBytes += size * elemSize;
        stats.peakUsedBytes += size * elemSize;
    };
    addVector(bodies_.capacity(), bodies_.size(), sizeof(Body));
    addVector(contacts_.capacity(), contacts_.size(), sizeof(Contact));
    addVector(prevContacts_.capacity(), prevContacts_.size(), sizeof(Contact));
    addVector(joints_.capacity(), joints_.size(), sizeof(Joint));
    stats.fragmentation = stats.usedBytes > 0
        ? 1.0 - double(stats.requestedBytes) / double(stats.usedBytes)
        : 0.0;
    return stats;
}

void* World::acquireQueryBuffer(size_t bytes) {
    // The pool is self-synchronized; no world lock so query threads can acquire
    // scratch concurrently regardless of the world thread-safety policy.
    return memoryPool_.allocate(bytes ? bytes : 1);
}

void World::releaseQueryBuffer(void* ptr, size_t bytes) {
    memoryPool_.deallocate(ptr, bytes ? bytes : 1);
}

ThreadSafetyPolicy World::threadSafetyPolicy() const {
    AccessGuard guard(*this, AccessKind::Query, "threadSafetyPolicy");
    return threadSafetyPolicy_;
}

void World::setThreadSafetyPolicy(ThreadSafetyPolicy policy) {
    AccessGuard guard(*this, AccessKind::Mutation, "setThreadSafetyPolicy");
    if (policy != ThreadSafetyPolicy::Strict &&
        policy != ThreadSafetyPolicy::Relaxed &&
        policy != ThreadSafetyPolicy::Concurrent)
        throw std::invalid_argument("velox: invalid thread safety policy");
    threadSafetyPolicy_ = policy;
}

ThreadSafetyReport World::threadSafetyReport() const {
    AccessGuard guard(*this, AccessKind::Query, "threadSafetyReport");
    return threadSafetyReport_;
}

const char* World::backendName() const {
    AccessGuard guard(*this, AccessKind::Query, "backendName");
    return backend_->name();
}

DeterminismMode World::determinismMode() const {
    AccessGuard guard(*this, AccessKind::Query, "determinismMode");
    return determinismMode_;
}

DeviceLossPolicy World::deviceLossPolicy() const {
    AccessGuard guard(*this, AccessKind::Query, "deviceLossPolicy");
    return deviceLossPolicy_;
}

void World::setDeviceLossPolicy(DeviceLossPolicy policy) {
    AccessGuard guard(*this, AccessKind::Mutation, "setDeviceLossPolicy");
    if (policy != DeviceLossPolicy::FallbackToCPU &&
        policy != DeviceLossPolicy::ThrowException)
        throw std::invalid_argument("velox: invalid device loss policy");
    deviceLossPolicy_ = policy;
}

GPUResidentMode World::gpuResidentMode() const {
    AccessGuard guard(*this, AccessKind::Query, "gpuResidentMode");
    return gpuResidentMode_;
}

void World::setGPUResidentMode(GPUResidentMode mode) {
    AccessGuard guard(*this, AccessKind::Mutation, "setGPUResidentMode");
    if (mode != GPUResidentMode::Disabled &&
        mode != GPUResidentMode::Resident)
        throw std::invalid_argument("velox: invalid GPU resident mode");
    gpuResidentMode_ = mode;
}

bool World::isOnCPUBackend() const {
    AccessGuard guard(*this, AccessKind::Query, "isOnCPUBackend");
    return fallbackToCPU_ || std::string(backend_->name()) == "cpu";
}

bool World::resetCUDABackend() {
    AccessGuard guard(*this, AccessKind::Mutation, "resetCUDABackend");
    if (determinismMode_ == DeterminismMode::Strict ||
        solverOptions_.iterationPolicy == IterationPolicy::Adaptive)
        return false;
    try {
        resetBackend(BackendType::Cuda);
        fallbackToCPU_ = false;
        return true;
    } catch (const std::runtime_error&) {
        return false;
    }
}

IslandSolvingMode World::islandSolvingMode() const {
    AccessGuard guard(*this, AccessKind::Query, "islandSolvingMode");
    return islandSolvingMode_;
}

void World::setWorkerCount(uint32_t count) {
    AccessGuard guard(*this, AccessKind::Mutation, "setWorkerCount");
    backend_->setWorkerCount(count);
}

uint32_t World::workerCount() const {
    AccessGuard guard(*this, AccessKind::Query, "workerCount");
    return backend_->workerCount();
}

void World::setTaskSystem(TaskSystem* system) {
    AccessGuard guard(*this, AccessKind::Mutation, "setTaskSystem");
    taskSystem_ = system;
    backend_->setTaskSystem(system);
}

TaskSystem* World::taskSystem() const {
    AccessGuard guard(*this, AccessKind::Query, "taskSystem");
    return taskSystem_;
}

void World::setDeterminismMode(DeterminismMode mode) {
    AccessGuard guard(*this, AccessKind::Mutation, "setDeterminismMode");
    if (mode != DeterminismMode::Relaxed && mode != DeterminismMode::Strict)
        throw std::invalid_argument("velox: invalid determinism mode");
    if (mode == determinismMode_) return;

    if (mode == DeterminismMode::Strict) {
#if !VELOX_STRICT_FLOATING_POINT
        throw std::logic_error(
            "velox: Strict determinism requires VELOX_STRICT_FLOATING_POINT=ON at configure time");
#else
        // The CUDA graph-colored solver intentionally changes impulse order.
        // Recreate the portable CPU reference backend instead of advertising
        // bitwise parity that the current device solver cannot provide.
        determinismMode_ = DeterminismMode::Strict;
        islandSolvingMode_ = IslandSolvingMode::Sequential;
        resetBackend(BackendType::Cpu);
        // Candidate generation and constraint solving must use the same
        // scalar, ordered traversal on every host. The strict backend is a
        // reproducibility reference, so it deliberately does not inherit an
        // automatic worker count from the relaxed runtime.
        backend_->setWorkerCount(1);
#endif
        return;
    }

    determinismMode_ = DeterminismMode::Relaxed;
    resetBackend(requestedBackend_);
}

WorldCcdDefaults World::ccdDefaults() const {
    AccessGuard guard(*this, AccessKind::Query, "ccdDefaults");
    return ccdDefaults_;
}

void World::setCcdDefaults(WorldCcdDefaults defaults) {
    AccessGuard guard(*this, AccessKind::Mutation, "setCcdDefaults");
    validateCcdDefaults(defaults);
    ccdDefaults_ = defaults;
}

SolverOptions World::solverOptions() const {
    AccessGuard guard(*this, AccessKind::Query, "solverOptions");
    return solverOptions_;
}

void World::setSolverOptions(SolverOptions options) {
    AccessGuard guard(*this, AccessKind::Mutation, "setSolverOptions");
    validateSolverOptions(options);
    solverOptions_ = options;
    const bool cudaActive = std::string(backend_->name()) == "cuda";
    if (options.iterationPolicy == IterationPolicy::Adaptive && cudaActive) {
        // CUDA's colored graph does not have a deterministic global impulse
        // reduction for early-out. Use the ordered CPU implementation rather
        // than silently treating Adaptive as a fixed iteration count.
        resetBackend(BackendType::Cpu);
    } else if (options.iterationPolicy == IterationPolicy::Fixed &&
               !fallbackToCPU_ && !cudaActive && requestedBackend_ != BackendType::Cpu) {
        resetBackend(requestedBackend_);
    }
}

WorldMultiToiSettings World::multiToiSettings() const {
    AccessGuard guard(*this, AccessKind::Query, "multiToiSettings");
    return multiToiSettings_;
}

void World::setMultiToiSettings(WorldMultiToiSettings settings) {
    AccessGuard guard(*this, AccessKind::Mutation, "setMultiToiSettings");
    validateMultiToiSettings(settings);
    multiToiSettings_ = settings;
}

void World::setCcdTuning(BodyId id, BodyCcdTuning tuning) {
    AccessGuard guard(*this, AccessKind::Mutation, "setCcdTuning");
    validateCcdTuning(tuning);
    Body& value = bodies_[resolve(id)];
    value.ccdTuning = tuning;
    if (value.isLocked()) {
        value.velocity = {};
        value.angularVelocity = {};
        value.force = {};
        value.torque = {};
    }
    broadPhase_->touched = true;
}

BodyCcdTuning World::ccdTuning(BodyId id) const {
    AccessGuard guard(*this, AccessKind::Query, "ccdTuning");
    return bodies_[resolve(id)].ccdTuning;
}

void World::setIslandSolvingMode(IslandSolvingMode mode) {
    AccessGuard guard(*this, AccessKind::Mutation, "setIslandSolvingMode");
    if (mode != IslandSolvingMode::Sequential && mode != IslandSolvingMode::Parallel)
        throw std::invalid_argument("velox: invalid island solving mode");
    if (determinismMode_ == DeterminismMode::Strict &&
        mode != IslandSolvingMode::Sequential)
        throw std::logic_error(
            "velox: Strict determinism requires sequential island solving");
    islandSolvingMode_ = mode;
    backend_->setParallelIslands(mode == IslandSolvingMode::Parallel);
}

bool World::isValid(BodyId id) const {
    AccessGuard guard(*this, AccessKind::Query, "isValid");
    uint32_t slot = id.slot();
    return slot < bodySlots_.size() && bodySlots_[slot].dense != UINT32_MAX &&
           bodySlots_[slot].generation == id.generation();
}

bool World::isValid(JointId id) const {
    AccessGuard guard(*this, AccessKind::Query, "isValid");
    uint32_t slot = id.slot();
    return slot < jointSlots_.size() && jointSlots_[slot].dense != UINT32_MAX &&
           jointSlots_[slot].generation == id.generation();
}

BodyIndex World::resolve(BodyId id) const {
    if (!isValid(id)) throw std::out_of_range("velox: invalid or stale body handle");
    return bodySlots_[id.slot()].dense;
}

uint32_t World::resolve(JointId id) const {
    if (!isValid(id)) throw std::out_of_range("velox: invalid or stale joint handle");
    return jointSlots_[id.slot()].dense;
}

BodyId World::bodyHandle(BodyIndex dense) const {
    uint32_t slot = bodyDenseToSlot_[dense];
    return BodyId::make(slot, bodySlots_[slot].generation);
}

BodyId World::addBody(Body bodyValue) {
    if (bodies_.size() >= UINT32_MAX)
        throw std::length_error("velox: body capacity exceeded");
    bodyValue.ccdTuning.quality = ccdDefaults_.defaultQuality;
    bodyValue.ccdTuning.collisionMargin = ccdDefaults_.defaultCollisionMargin;
    bodyValue.ccdTuning.speculativeDistance = ccdDefaults_.defaultSpeculativeDistance;
    bodyValue.ccdTuning.enableContinuous = ccdDefaults_.defaultEnableContinuous;
    bodyValue.ccdTuning.minVelocityForCCD = ccdDefaults_.defaultMinVelocityForCCD;
    uint32_t slot;
    if (freeBodySlots_.empty()) {
        slot = static_cast<uint32_t>(bodySlots_.size());
        bodySlots_.push_back({});
    } else {
        slot = freeBodySlots_.back();
        freeBodySlots_.pop_back();
    }
    uint32_t dense = static_cast<uint32_t>(bodies_.size());
    bodies_.push_back(bodyValue);
    bodyDenseToSlot_.push_back(slot);
    bodySlots_[slot].dense = dense;
    BodyId id = BodyId::make(slot, bodySlots_[slot].generation);
    pendingBodyEvents_.push_back({BodyEventType::Created, id, bodyValue.position, bodyValue.orientation});
    return id;
}

JointId World::addJoint(Joint jointValue) {
    if (joints_.size() >= UINT32_MAX)
        throw std::length_error("velox: joint capacity exceeded");
    uint32_t slot;
    if (freeJointSlots_.empty()) {
        slot = static_cast<uint32_t>(jointSlots_.size());
        jointSlots_.push_back({});
    } else {
        slot = freeJointSlots_.back();
        freeJointSlots_.pop_back();
    }
    uint32_t dense = static_cast<uint32_t>(joints_.size());
    joints_.push_back(jointValue);
    jointDenseToSlot_.push_back(slot);
    jointSlots_[slot].dense = dense;
    return JointId::make(slot, jointSlots_[slot].generation);
}

Body& World::body(BodyId id) {
    AccessGuard guard(*this, AccessKind::Mutation, "body");
    // Mutable access can change transforms, velocities, or shape parameters:
    // the broad-phase proxies must re-fit before the tree is trusted again.
    broadPhase_->touched = true;
    return bodies_[resolve(id)];
}

const Body& World::body(BodyId id) const {
    AccessGuard guard(*this, AccessKind::Query, "body");
    return bodies_[resolve(id)];
}

GeometryDiagnostics World::queryGeometryDiagnostics(BodyId id) const {
    AccessGuard guard(*this, AccessKind::Query, "queryGeometryDiagnostics");
    return geometry_detail::diagnostics(bodies_[resolve(id)], meshes_);
}

Body World::bodyState(BodyId id) const {
    AccessGuard guard(*this, AccessKind::Query, "bodyState");
    return bodies_[resolve(id)];
}

size_t World::bodyCount() const {
    AccessGuard guard(*this, AccessKind::Query, "bodyCount");
    return bodies_.size();
}

StepStats World::lastStepStatsCopy() const {
    AccessGuard guard(*this, AccessKind::Query, "lastStepStatsCopy");
    return lastStepStats_;
}

Joint& World::joint(JointId id) {
    AccessGuard guard(*this, AccessKind::Mutation, "joint");
    return joints_[resolve(id)];
}

const Joint& World::joint(JointId id) const {
    AccessGuard guard(*this, AccessKind::Query, "joint");
    return joints_[resolve(id)];
}

Joint World::jointState(JointId id) const {
    AccessGuard guard(*this, AccessKind::Query, "jointState");
    return joints_[resolve(id)];
}

BodyId World::addSphere(Vec3 position, float radius, float mass) {
    AccessGuard guard(*this, AccessKind::Mutation, "addSphere");
    requireFiniteVec(position, "velox: sphere position must be finite");
    requirePositive(radius, "velox: sphere radius must be finite and positive");
    requireMass(mass);
    Body b{};
    b.position = position;
    b.shape = ShapeType::Sphere;
    b.radius = radius;
    b.motionType = mass > 0.0f ? MotionType::Dynamic : MotionType::Static;
    b.invMass = mass > 0.0f ? 1.0f / mass : 0.0f;
    if (mass > 0.0f) {
        float i = 0.4f * mass * radius * radius; // solid sphere: 2/5 m r^2
        b.invInertia = {1.0f / i, 1.0f / i, 1.0f / i};
    }
    return addBody(b);
}

BodyId World::addBox(Vec3 position, Vec3 halfExtents, float mass) {
    AccessGuard guard(*this, AccessKind::Mutation, "addBox");
    requireFiniteVec(position, "velox: box position must be finite");
    requireFiniteVec(halfExtents, "velox: box half extents must be finite");
    if (halfExtents.x <= 0.0f || halfExtents.y <= 0.0f || halfExtents.z <= 0.0f)
        throw std::invalid_argument("velox: box half extents must be positive");
    requireMass(mass);
    Body b{};
    b.position = position;
    b.shape = ShapeType::Box;
    b.halfExtents = halfExtents;
    b.motionType = mass > 0.0f ? MotionType::Dynamic : MotionType::Static;
    b.invMass = mass > 0.0f ? 1.0f / mass : 0.0f;
    if (mass > 0.0f) {
        Vec3 e = halfExtents * 2.0f;
        float k = mass / 12.0f;
        b.invInertia = {1.0f / (k * (e.y * e.y + e.z * e.z)),
                        1.0f / (k * (e.x * e.x + e.z * e.z)),
                        1.0f / (k * (e.x * e.x + e.y * e.y))};
    }
    return addBody(b);
}

BodyId World::addCapsule(Vec3 position, float radius, float halfHeight, float mass) {
    AccessGuard guard(*this, AccessKind::Mutation, "addCapsule");
    requireFiniteVec(position, "velox: capsule position must be finite");
    requirePositive(radius, "velox: capsule radius must be finite and positive");
    requireNonNegative(halfHeight, "velox: capsule half height must be finite and non-negative");
    requireMass(mass);
    Body b{};
    b.position = position;
    b.shape = ShapeType::Capsule;
    b.radius = radius;
    b.capsuleHalfHeight = halfHeight;
    b.motionType = mass > 0.0f ? MotionType::Dynamic : MotionType::Static;
    b.invMass = mass > 0.0f ? 1.0f / mass : 0.0f;
    if (mass > 0.0f) {
        // Uniform solid capsule: split mass by the volumes of the cylinder
        // core and the two hemispheres, then apply the parallel-axis theorem.
        float r = radius;
        float cylinderWeight = 2.0f * halfHeight;
        float sphereWeight = (4.0f / 3.0f) * r;
        float cylinderMass = mass * cylinderWeight /
                             (cylinderWeight + sphereWeight);
        float sphereMass = mass - cylinderMass;
        float iy = 0.5f * cylinderMass * r * r +
                   0.4f * sphereMass * r * r;
        float ix = cylinderMass *
                       (3.0f * r * r + 4.0f * halfHeight * halfHeight) /
                       12.0f +
                   sphereMass *
                       (0.4f * r * r + 0.75f * halfHeight * r +
                        halfHeight * halfHeight);
        b.invInertia = {1.0f / ix, 1.0f / iy, 1.0f / ix};
    }
    return addBody(b);
}

BodyId World::addCylinder(Vec3 position, float radius, float halfHeight, float mass) {
    AccessGuard guard(*this, AccessKind::Mutation, "addCylinder");
    requireFiniteVec(position, "velox: cylinder position must be finite");
    requirePositive(radius, "velox: cylinder radius must be finite and positive");
    requirePositive(halfHeight,
                    "velox: cylinder half height must be finite and positive");
    requireMass(mass);
    Body b{};
    b.position = position;
    b.shape = ShapeType::Cylinder;
    b.radius = radius;
    b.capsuleHalfHeight = halfHeight;
    b.motionType = mass > 0.0f ? MotionType::Dynamic : MotionType::Static;
    b.invMass = mass > 0.0f ? 1.0f / mass : 0.0f;
    if (mass > 0.0f) {
        float height = 2.0f * halfHeight;
        float iy = 0.5f * mass * radius * radius;
        float ix = mass * (3.0f * radius * radius + height * height) / 12.0f;
        b.invInertia = {1.0f / ix, 1.0f / iy, 1.0f / ix};
    }
    return addBody(b);
}

BodyId World::addCone(Vec3 position, float radius, float height, float mass) {
    AccessGuard guard(*this, AccessKind::Mutation, "addCone");
    requireFiniteVec(position, "velox: cone position must be finite");
    requirePositive(radius, "velox: cone radius must be finite and positive");
    requirePositive(height, "velox: cone height must be finite and positive");
    requireMass(mass);
    Body b{};
    b.position = position; // center of mass, one quarter-height above the base
    b.shape = ShapeType::Cone;
    b.radius = radius;
    b.capsuleHalfHeight = height * 0.5f;
    b.motionType = mass > 0.0f ? MotionType::Dynamic : MotionType::Static;
    b.invMass = mass > 0.0f ? 1.0f / mass : 0.0f;
    if (mass > 0.0f) {
        float iy = 0.3f * mass * radius * radius;
        float ix = 0.15f * mass * radius * radius +
                   0.0375f * mass * height * height;
        b.invInertia = {1.0f / ix, 1.0f / iy, 1.0f / ix};
    }
    return addBody(b);
}

BodyId World::addRoundedBox(Vec3 position, Vec3 halfExtents, float radius, float mass) {
    AccessGuard guard(*this, AccessKind::Mutation, "addRoundedBox");
    requireFiniteVec(position, "velox: rounded box position must be finite");
    requireFiniteVec(halfExtents, "velox: rounded box half extents must be finite");
    if (halfExtents.x <= 0.0f || halfExtents.y <= 0.0f || halfExtents.z <= 0.0f)
        throw std::invalid_argument("velox: rounded box half extents must be positive");
    requirePositive(radius, "velox: rounded box radius must be finite and positive");
    requireMass(mass);
    Body b{};
    b.position = position;
    b.shape = ShapeType::RoundedBox;
    b.halfExtents = halfExtents;
    b.radius = radius;
    b.motionType = mass > 0.0f ? MotionType::Dynamic : MotionType::Static;
    b.invMass = mass > 0.0f ? 1.0f / mass : 0.0f;
    if (mass > 0.0f) {
        Vec3 e = halfExtents * 2.0f;
        float k = mass / 12.0f;
        b.invInertia = {1.0f / (k * (e.y * e.y + e.z * e.z)),
                        1.0f / (k * (e.x * e.x + e.z * e.z)),
                        1.0f / (k * (e.x * e.x + e.y * e.y))};
    }
    return addBody(b);
}

BodyId World::addEllipsoid(Vec3 position, Vec3 radii, float mass) {
    AccessGuard guard(*this, AccessKind::Mutation, "addEllipsoid");
    requireFiniteVec(position, "velox: ellipsoid position must be finite");
    requireFiniteVec(radii, "velox: ellipsoid radii must be finite");
    if (radii.x <= 0.0f || radii.y <= 0.0f || radii.z <= 0.0f)
        throw std::invalid_argument("velox: ellipsoid radii must be positive");
    requireMass(mass);
    Body b{};
    b.position = position;
    b.shape = ShapeType::Ellipsoid;
    b.halfExtents = radii;
    b.motionType = mass > 0.0f ? MotionType::Dynamic : MotionType::Static;
    b.invMass = mass > 0.0f ? 1.0f / mass : 0.0f;
    if (mass > 0.0f) {
        float a = radii.x, bv = radii.y, c = radii.z;
        b.invInertia = {1.0f / (0.2f * mass * (bv * bv + c * c)),
                        1.0f / (0.2f * mass * (a * a + c * c)),
                        1.0f / (0.2f * mass * (a * a + bv * bv))};
    }
    return addBody(b);
}

BodyId World::addConvexHull(Vec3 position, const std::vector<Vec3>& points, float mass) {
    AccessGuard guard(*this, AccessKind::Mutation, "addConvexHull");
    requireFiniteVec(position, "velox: convex hull position must be finite");
    requireMass(mass);
    if (points.size() < 4)
        throw std::invalid_argument("velox: convex hull requires at least four points");
    if (points.size() > UINT32_MAX)
        throw std::invalid_argument("velox: convex hull has too many points");
    for (const Vec3& p : points)
        requireFiniteVec(p, "velox: convex hull points must be finite");
    rejectDuplicateHullPoints(points,
                              "velox: convex hull contains duplicate points");

    // Reject point, line, and plane clouds. GJK can represent them, but they
    // have zero volume and cannot produce valid 3D mass properties or EPA seeds.
    const Vec3 p0 = points[0];
    float scale2 = 0.0f;
    size_t i1 = 0;
    for (size_t i = 1; i < points.size(); ++i) {
        float d2 = lengthSq(points[i] - p0);
        if (d2 > scale2) { scale2 = d2; i1 = i; }
    }
    if (scale2 < 1e-12f)
        throw std::invalid_argument("velox: convex hull points are coincident");
    Vec3 edge = points[i1] - p0;
    float bestArea2 = 0.0f;
    size_t i2 = 0;
    for (size_t i = 1; i < points.size(); ++i) {
        float area2 = lengthSq(cross(edge, points[i] - p0));
        if (area2 > bestArea2) { bestArea2 = area2; i2 = i; }
    }
    if (bestArea2 <= scale2 * scale2 * 1e-12f)
        throw std::invalid_argument("velox: convex hull points are collinear");
    Vec3 planeNormal = cross(edge, points[i2] - p0);
    float maxPlaneDistance = 0.0f;
    for (const Vec3& p : points)
        maxPlaneDistance = vmax(maxPlaneDistance, fabsf(dot(planeNormal, p - p0)));
    float scale = sqrtf(scale2);
    if (maxPlaneDistance <= scale * scale * scale * 1e-6f)
        throw std::invalid_argument("velox: convex hull points are coplanar");
    std::vector<Vec3> storedPoints = points;
    mass_properties::ConvexMassProperties properties = mass_properties::convex(points);
    if (mass > 0.0f) {
        for (Vec3& point : storedPoints) point -= properties.center;
    }
    Body b{};
    b.position = position + (mass > 0.0f ? properties.center : Vec3{});
    b.shape = ShapeType::Hull;
    b.hullFirst = static_cast<uint32_t>(meshes_.hullPoints.size());
    b.hullCount = static_cast<uint32_t>(storedPoints.size());
    if (properties.triangles.size() > UINT32_MAX / 3 ||
        meshes_.hullFaceIndices.size() >
            UINT32_MAX - properties.triangles.size() * 3)
        throw std::length_error("velox: convex hull face capacity exceeded");
    b.hullFaceFirst = static_cast<uint32_t>(meshes_.hullFaceIndices.size());
    b.hullFaceCount = static_cast<uint32_t>(properties.triangles.size());
    b.motionType = mass > 0.0f ? MotionType::Dynamic : MotionType::Static;
    meshes_.hullPoints.insert(meshes_.hullPoints.end(), storedPoints.begin(),
                              storedPoints.end());
    for (const auto& triangle : properties.triangles)
        meshes_.hullFaceIndices.insert(meshes_.hullFaceIndices.end(),
                                       triangle.begin(), triangle.end());
    float r2 = 0.0f;
    for (const Vec3& p : storedPoints) {
        r2 = vmax(r2, lengthSq(p));
    }
    b.radius = sqrtf(r2); // bounding radius (AABB + maxPointSpeed)
    b.invMass = mass > 0.0f ? 1.0f / mass : 0.0f;
    if (mass > 0.0f) {
        float density = mass / float(properties.volume);
        Vec3 inertia = properties.principalInertia * density;
        b.invInertia = {1.0f / inertia.x, 1.0f / inertia.y,
                        1.0f / inertia.z};
        b.inertiaOrientation = properties.principalOrientation;
    }
    return addBody(b);
}

BodyId World::addCompound(Vec3 position, const std::vector<CompoundShape>& shapes,
                           float mass) {
    AccessGuard guard(*this, AccessKind::Mutation, "addCompound");
    requireFiniteVec(position, "velox: compound position must be finite");
    requireMass(mass);
    if (shapes.empty())
        throw std::invalid_argument("velox: compound requires at least one child shape");
    if (shapes.size() > UINT32_MAX ||
        meshes_.compoundChildren.size() > UINT32_MAX - shapes.size())
        throw std::length_error("velox: compound child capacity exceeded");

    std::vector<CompoundChild> children;
    std::vector<Vec3> hullPoints;
    std::vector<uint32_t> hullFaceIndices;
    struct ChildMassData {
        double volume = 0.0;
        Vec3 principalInertia;
        Quat principalOrientation;
    };
    std::vector<ChildMassData> childMass;
    std::vector<float> childBounds;
    children.reserve(shapes.size());
    childMass.reserve(shapes.size());
    childBounds.reserve(shapes.size());

    for (const CompoundShape& shape : shapes) {
        requireFiniteVec(shape.localPosition,
                         "velox: compound child position must be finite");
        if (!finiteQuat(shape.localOrientation))
            throw std::invalid_argument("velox: compound child orientation must be finite");
        float q2 = shape.localOrientation.x * shape.localOrientation.x +
                   shape.localOrientation.y * shape.localOrientation.y +
                   shape.localOrientation.z * shape.localOrientation.z +
                   shape.localOrientation.w * shape.localOrientation.w;
        if (q2 < 1e-12f)
            throw std::invalid_argument("velox: compound child orientation must be non-zero");

        CompoundChild child;
        child.shape = shape.shape;
        child.localPosition = shape.localPosition;
        child.localOrientation = normalize(shape.localOrientation);
        ChildMassData massData;
        massData.principalOrientation = child.localOrientation;
        float bound = 0.0f;
        switch (shape.shape) {
        case ShapeType::Sphere:
            requirePositive(shape.radius,
                            "velox: compound sphere radius must be positive");
            child.radius = shape.radius;
            bound = shape.radius;
            if (mass > 0.0f) {
                massData.volume = (4.0 / 3.0) * 3.141592653589793 *
                                  shape.radius * shape.radius * shape.radius;
                float moment = 0.4f * float(massData.volume) *
                               shape.radius * shape.radius;
                massData.principalInertia = {moment, moment, moment};
            }
            break;
        case ShapeType::Box:
            if (!finiteVec(shape.halfExtents) || shape.halfExtents.x <= 0.0f ||
                shape.halfExtents.y <= 0.0f || shape.halfExtents.z <= 0.0f)
                throw std::invalid_argument("velox: compound box half extents must be positive");
            child.halfExtents = shape.halfExtents;
            bound = length(shape.halfExtents);
            if (mass > 0.0f) {
                massData.volume = 8.0 * shape.halfExtents.x *
                                  shape.halfExtents.y * shape.halfExtents.z;
                float childMassValue = float(massData.volume) / 3.0f;
                massData.principalInertia = {
                    childMassValue * (shape.halfExtents.y * shape.halfExtents.y +
                                      shape.halfExtents.z * shape.halfExtents.z),
                    childMassValue * (shape.halfExtents.x * shape.halfExtents.x +
                                      shape.halfExtents.z * shape.halfExtents.z),
                    childMassValue * (shape.halfExtents.x * shape.halfExtents.x +
                                      shape.halfExtents.y * shape.halfExtents.y)};
            }
            break;
        case ShapeType::Capsule:
            requirePositive(shape.radius,
                            "velox: compound capsule radius must be positive");
            requireNonNegative(shape.capsuleHalfHeight,
                               "velox: compound capsule half height must be non-negative");
            child.radius = shape.radius;
            child.capsuleHalfHeight = shape.capsuleHalfHeight;
            bound = shape.radius + shape.capsuleHalfHeight;
            if (mass > 0.0f) {
                float r = shape.radius, h = shape.capsuleHalfHeight;
                double cylinderVolume = 2.0 * 3.141592653589793 * r * r * h;
                double sphereVolume = (4.0 / 3.0) * 3.141592653589793 * r * r * r;
                massData.volume = cylinderVolume + sphereVolume;
                float cylinderMass = float(cylinderVolume);
                float sphereMass = float(sphereVolume);
                float iy = 0.5f * cylinderMass * r * r +
                           0.4f * sphereMass * r * r;
                float ix = cylinderMass * (3.0f * r * r + 4.0f * h * h) /
                               12.0f +
                           sphereMass * (0.4f * r * r + 0.75f * h * r + h * h);
                massData.principalInertia = {ix, iy, ix};
            }
            break;
        case ShapeType::Cylinder:
            requirePositive(shape.radius,
                            "velox: compound cylinder radius must be positive");
            requirePositive(shape.capsuleHalfHeight,
                            "velox: compound cylinder half height must be positive");
            child.radius = shape.radius;
            child.capsuleHalfHeight = shape.capsuleHalfHeight;
            bound = sqrtf(shape.radius * shape.radius +
                          shape.capsuleHalfHeight * shape.capsuleHalfHeight);
            if (mass > 0.0f) {
                float r = shape.radius, h = shape.capsuleHalfHeight;
                massData.volume = 2.0 * 3.141592653589793 * r * r * h;
                float childMassValue = float(massData.volume);
                float iy = 0.5f * childMassValue * r * r;
                float ix = childMassValue * (3.0f * r * r + 4.0f * h * h) /
                           12.0f;
                massData.principalInertia = {ix, iy, ix};
            }
            break;
        case ShapeType::Cone: {
            requirePositive(shape.radius,
                            "velox: compound cone radius must be positive");
            requirePositive(shape.capsuleHalfHeight,
                            "velox: compound cone half height must be positive");
            child.radius = shape.radius;
            child.capsuleHalfHeight = shape.capsuleHalfHeight;
            float top = 1.5f * shape.capsuleHalfHeight;
            bound = vmax(top, sqrtf(shape.radius * shape.radius +
                                    0.25f * shape.capsuleHalfHeight *
                                        shape.capsuleHalfHeight));
            if (mass > 0.0f) {
                float r = shape.radius, height = 2.0f * shape.capsuleHalfHeight;
                massData.volume = 3.141592653589793 * r * r * height / 3.0;
                float childMassValue = float(massData.volume);
                float iy = 0.3f * childMassValue * r * r;
                float ix = 0.15f * childMassValue * r * r +
                           0.0375f * childMassValue * height * height;
                massData.principalInertia = {ix, iy, ix};
            }
            break;
        }
        case ShapeType::RoundedBox:
            requireFiniteVec(shape.halfExtents,
                             "velox: compound rounded box extents must be finite");
            if (shape.halfExtents.x <= 0.0f || shape.halfExtents.y <= 0.0f ||
                shape.halfExtents.z <= 0.0f)
                throw std::invalid_argument(
                    "velox: compound rounded box extents must be positive");
            requirePositive(shape.radius,
                            "velox: compound rounded box radius must be positive");
            child.halfExtents = shape.halfExtents;
            child.radius = shape.radius;
            bound = length(shape.halfExtents) + shape.radius;
            if (mass > 0.0f) {
                Vec3 e = shape.halfExtents * 2.0f;
                massData.volume = 8.0 * shape.halfExtents.x * shape.halfExtents.y * shape.halfExtents.z;
                float childMassValue = float(massData.volume);
                float k = childMassValue / 12.0f;
                massData.principalInertia = {k * (e.y * e.y + e.z * e.z),
                                             k * (e.x * e.x + e.z * e.z),
                                             k * (e.x * e.x + e.y * e.y)};
            }
            break;
        case ShapeType::Ellipsoid:
            requireFiniteVec(shape.halfExtents,
                             "velox: compound ellipsoid radii must be finite");
            if (shape.halfExtents.x <= 0.0f || shape.halfExtents.y <= 0.0f ||
                shape.halfExtents.z <= 0.0f)
                throw std::invalid_argument(
                    "velox: compound ellipsoid radii must be positive");
            child.halfExtents = shape.halfExtents;
            bound = vmax(shape.halfExtents.x, vmax(shape.halfExtents.y, shape.halfExtents.z));
            if (mass > 0.0f) {
                float a = shape.halfExtents.x, b = shape.halfExtents.y, c = shape.halfExtents.z;
                massData.volume = 4.0 * 3.141592653589793 * a * b * c / 3.0;
                float childMassValue = float(massData.volume);
                massData.principalInertia = {0.2f * childMassValue * (b * b + c * c),
                                             0.2f * childMassValue * (a * a + c * c),
                                             0.2f * childMassValue * (a * a + b * b)};
            }
            break;
        case ShapeType::Hull: {
            if (shape.hullPoints.size() < 4)
                throw std::invalid_argument("velox: compound hull requires at least four points");
            if (shape.hullPoints.size() > UINT32_MAX ||
                hullPoints.size() > UINT32_MAX - shape.hullPoints.size())
                throw std::length_error("velox: compound hull point capacity exceeded");
            for (const Vec3& p : shape.hullPoints) {
                requireFiniteVec(p, "velox: compound hull points must be finite");
            }
            rejectDuplicateHullPoints(
                shape.hullPoints, "velox: compound hull contains duplicate points");
            const Vec3 p0 = shape.hullPoints[0];
            float scale2 = 0.0f;
            size_t i1 = 0;
            for (size_t i = 1; i < shape.hullPoints.size(); ++i) {
                float d2 = lengthSq(shape.hullPoints[i] - p0);
                if (d2 > scale2) { scale2 = d2; i1 = i; }
            }
            if (scale2 < 1e-12f)
                throw std::invalid_argument("velox: compound hull points are coincident");
            Vec3 edge = shape.hullPoints[i1] - p0;
            float bestArea2 = 0.0f;
            size_t i2 = 0;
            for (size_t i = 1; i < shape.hullPoints.size(); ++i) {
                float area2 = lengthSq(cross(edge, shape.hullPoints[i] - p0));
                if (area2 > bestArea2) { bestArea2 = area2; i2 = i; }
            }
            if (bestArea2 <= scale2 * scale2 * 1e-12f)
                throw std::invalid_argument("velox: compound hull points are collinear");
            Vec3 planeNormal = cross(edge, shape.hullPoints[i2] - p0);
            float maxPlaneDistance = 0.0f;
            for (const Vec3& p : shape.hullPoints)
                maxPlaneDistance = vmax(
                    maxPlaneDistance, fabsf(dot(planeNormal, p - p0)));
            float scale = sqrtf(scale2);
            if (maxPlaneDistance <= scale * scale * scale * 1e-6f)
                throw std::invalid_argument("velox: compound hull points are coplanar");
            child.hullFirst = static_cast<uint32_t>(meshes_.hullPoints.size() +
                                                    hullPoints.size());
            child.hullCount = static_cast<uint32_t>(shape.hullPoints.size());
            mass_properties::ConvexMassProperties hullProperties =
                mass_properties::convex(shape.hullPoints);
            if (hullProperties.triangles.size() > UINT32_MAX / 3 ||
                meshes_.hullFaceIndices.size() + hullFaceIndices.size() >
                    UINT32_MAX - hullProperties.triangles.size() * 3)
                throw std::length_error("velox: compound hull face capacity exceeded");
            child.hullFaceFirst = static_cast<uint32_t>(meshes_.hullFaceIndices.size() +
                                                        hullFaceIndices.size());
            child.hullFaceCount = static_cast<uint32_t>(hullProperties.triangles.size());
            for (const auto& triangle : hullProperties.triangles)
                hullFaceIndices.insert(hullFaceIndices.end(), triangle.begin(), triangle.end());
            if (mass > 0.0f) {
                child.localPosition += rotate(child.localOrientation,
                                              hullProperties.center);
                massData.volume = hullProperties.volume;
                massData.principalInertia = hullProperties.principalInertia;
                massData.principalOrientation = mul(
                    child.localOrientation,
                    hullProperties.principalOrientation);
                for (const Vec3& point : shape.hullPoints) {
                    Vec3 centered = point - hullProperties.center;
                    hullPoints.push_back(centered);
                    bound = vmax(bound, length(centered));
                }
            } else {
                hullPoints.insert(hullPoints.end(), shape.hullPoints.begin(),
                                  shape.hullPoints.end());
                for (const Vec3& point : shape.hullPoints)
                    bound = vmax(bound, length(point));
            }
            break;
        }
        default:
            throw std::invalid_argument(
                "velox: compound children must be sphere, box, capsule, or hull");
        }
        children.push_back(child);
        childMass.push_back(massData);
        childBounds.push_back(bound);
    }

    Body body{};
    Vec3 center;
    mass_properties::Matrix3 centeredTensor;
    double totalVolume = 0.0;
    if (mass > 0.0f) {
        double first[3]{};
        mass_properties::Matrix3 originTensor;
        for (size_t i = 0; i < children.size(); ++i) {
            totalVolume += childMass[i].volume;
            first[0] += childMass[i].volume * children[i].localPosition.x;
            first[1] += childMass[i].volume * children[i].localPosition.y;
            first[2] += childMass[i].volume * children[i].localPosition.z;
            mass_properties::Matrix3 tensor =
                mass_properties::rotatedPrincipalTensor(
                    childMass[i].principalInertia,
                    childMass[i].principalOrientation);
            mass_properties::addAtOffset(originTensor, tensor,
                                         childMass[i].volume,
                                         children[i].localPosition);
        }
        if (totalVolume <= 1e-12)
            throw std::invalid_argument("velox: compound has no positive volume");
        center = {float(first[0] / totalVolume), float(first[1] / totalVolume),
                  float(first[2] / totalVolume)};
        centeredTensor = mass_properties::shiftedToCenter(
            originTensor, totalVolume, center);
        for (CompoundChild& child : children) child.localPosition -= center;
    }
    float aggregateRadius = 0.0f;
    for (size_t i = 0; i < children.size(); ++i)
        aggregateRadius = vmax(aggregateRadius,
                               length(children[i].localPosition) + childBounds[i]);
    body.position = position + center;
    body.shape = ShapeType::Compound;
    body.compoundFirst = static_cast<uint32_t>(meshes_.compoundChildren.size());
    body.compoundCount = static_cast<uint32_t>(children.size());
    body.radius = aggregateRadius;
    body.motionType = mass > 0.0f ? MotionType::Dynamic : MotionType::Static;
    body.invMass = mass > 0.0f ? 1.0f / mass : 0.0f;
    if (mass > 0.0f) {
        Vec3 unitMoments;
        mass_properties::jacobiDiagonalize(centeredTensor, unitMoments,
                                           body.inertiaOrientation);
        float density = mass / float(totalVolume);
        Vec3 inertia = unitMoments * density;
        body.invInertia = {1.0f / inertia.x, 1.0f / inertia.y,
                           1.0f / inertia.z};
    }
    meshes_.hullPoints.insert(meshes_.hullPoints.end(), hullPoints.begin(), hullPoints.end());
    meshes_.hullFaceIndices.insert(meshes_.hullFaceIndices.end(),
                                   hullFaceIndices.begin(), hullFaceIndices.end());
    meshes_.compoundChildren.insert(meshes_.compoundChildren.end(),
                                    children.begin(), children.end());
    return addBody(body);
}

BodyId World::addStaticPlane(Vec3 normal, float offset) {
    AccessGuard guard(*this, AccessKind::Mutation, "addStaticPlane");
    requireFiniteVec(normal, "velox: plane normal must be finite");
    if (lengthSq(normal) < 1e-12f)
        throw std::invalid_argument("velox: plane normal must be non-zero");
    if (!finiteFloat(offset)) throw std::invalid_argument("velox: plane offset must be finite");
    Body b{};
    b.shape = ShapeType::Plane;
    b.motionType = MotionType::Static;
    b.planeNormal = normalize(normal);
    b.planeOffset = offset;
    b.invMass = 0.0f;
    return addBody(b);
}

namespace {

// Recursive median-split BVH build. Children are allocated consecutively so
// traversal only stores one child index per inner node.
struct BvhBuilder {
    std::vector<BvhNode>& nodes;
    std::vector<uint32_t>& triRefs;   // global array; this mesh's refs start at refBase
    const std::vector<Vec3>& centroids;
    const std::vector<Vec3>& triMin;
    const std::vector<Vec3>& triMax;
    uint32_t refBase;

    void computeBounds(uint32_t nodeIdx, uint32_t first, uint32_t count) {
        BvhNode& n = nodes[nodeIdx];
        n.aabbMin = triMin[triRefs[first]];
        n.aabbMax = triMax[triRefs[first]];
        for (uint32_t i = 1; i < count; ++i) {
            n.aabbMin = vmin(n.aabbMin, triMin[triRefs[first + i]]);
            n.aabbMax = vmax(n.aabbMax, triMax[triRefs[first + i]]);
        }
    }

    void build(uint32_t nodeIdx, uint32_t first, uint32_t count) {
        computeBounds(nodeIdx, first, count);
        if (count <= 4) {
            nodes[nodeIdx].leftFirst = first;
            nodes[nodeIdx].triCount = count;
            return;
        }
        // Split at the median along the widest centroid axis.
        Vec3 lo = centroids[triRefs[first]], hi = lo;
        for (uint32_t i = 1; i < count; ++i) {
            lo = vmin(lo, centroids[triRefs[first + i]]);
            hi = vmax(hi, centroids[triRefs[first + i]]);
        }
        Vec3 ext = hi - lo;
        int axis = ext.x > ext.y ? (ext.x > ext.z ? 0 : 2) : (ext.y > ext.z ? 1 : 2);
        auto key = [&](uint32_t t) {
            const Vec3& c = centroids[t];
            return axis == 0 ? c.x : axis == 1 ? c.y : c.z;
        };
        std::nth_element(triRefs.begin() + first,
                         triRefs.begin() + first + count / 2,
                         triRefs.begin() + first + count,
                         [&](uint32_t a, uint32_t b) { return key(a) < key(b); });
        uint32_t mid = count / 2;

        uint32_t left = static_cast<uint32_t>(nodes.size());
        nodes[nodeIdx].leftFirst = left;
        nodes[nodeIdx].triCount = 0;
        nodes.emplace_back();
        nodes.emplace_back();
        build(left, first, mid);
        build(left + 1, first + mid, count - mid);
    }
};

} // namespace

BodyId World::addStaticMesh(const std::vector<Vec3>& vertices,
                            const std::vector<uint32_t>& indices) {
    AccessGuard guard(*this, AccessKind::Mutation, "addStaticMesh");
    if (vertices.size() < 3 || indices.empty() || indices.size() % 3 != 0)
        throw std::invalid_argument("velox: mesh requires vertices and complete triangles");
    if (vertices.size() > UINT32_MAX || indices.size() > UINT32_MAX)
        throw std::invalid_argument("velox: mesh exceeds 32-bit storage limits");
    for (const Vec3& vertex : vertices)
        requireFiniteVec(vertex, "velox: mesh vertices must be finite");
    for (uint32_t index : indices)
        if ((size_t)index >= vertices.size())
            throw std::out_of_range("velox: mesh index is outside the vertex array");
    for (size_t i = 0; i < indices.size(); i += 3) {
        Vec3 e1 = vertices[indices[i + 1]] - vertices[indices[i]];
        Vec3 e2 = vertices[indices[i + 2]] - vertices[indices[i]];
        float edgeScale2 = vmax(lengthSq(e1), lengthSq(e2));
        if (edgeScale2 < 1e-16f || lengthSq(cross(e1, e2)) <= edgeScale2 * edgeScale2 * 1e-12f)
            throw std::invalid_argument("velox: mesh contains a degenerate triangle");
    }
    Mesh m;
    m.firstVertex = static_cast<uint32_t>(meshes_.vertices.size());
    m.vertexCount = static_cast<uint32_t>(vertices.size());
    m.firstIndex = static_cast<uint32_t>(meshes_.indices.size());
    m.indexCount = static_cast<uint32_t>(indices.size());
    m.aabbMin = m.aabbMax = vertices.empty() ? Vec3{} : vertices[0];
    for (const Vec3& v : vertices) {
        m.aabbMin = vmin(m.aabbMin, v);
        m.aabbMax = vmax(m.aabbMax, v);
    }
    meshes_.vertices.insert(meshes_.vertices.end(), vertices.begin(), vertices.end());
    meshes_.indices.insert(meshes_.indices.end(), indices.begin(), indices.end());

    // Build the triangle BVH.
    uint32_t triCount = m.indexCount / 3;
    std::vector<Vec3> centroids(triCount), triMin(triCount), triMax(triCount);
    for (uint32_t t = 0; t < triCount; ++t) {
        const Vec3& a = vertices[indices[t * 3]];
        const Vec3& b = vertices[indices[t * 3 + 1]];
        const Vec3& c = vertices[indices[t * 3 + 2]];
        centroids[t] = (a + b + c) * (1.0f / 3.0f);
        triMin[t] = vmin(vmin(a, b), c);
        triMax[t] = vmax(vmax(a, b), c);
    }
    m.firstNode = static_cast<uint32_t>(meshes_.bvhNodes.size());
    m.firstTriRef = static_cast<uint32_t>(meshes_.bvhTriRefs.size());
    for (uint32_t t = 0; t < triCount; ++t) meshes_.bvhTriRefs.push_back(t);
    if (triCount > 0) {
        meshes_.bvhNodes.emplace_back();
        BvhBuilder builder{meshes_.bvhNodes, meshes_.bvhTriRefs,
                           centroids, triMin, triMax, m.firstTriRef};
        builder.build(m.firstNode, m.firstTriRef, triCount);
    }
    m.nodeCount = static_cast<uint32_t>(meshes_.bvhNodes.size()) - m.firstNode;
    meshes_.meshes.push_back(m);

    Body b{};
    b.shape = ShapeType::Mesh;
    b.motionType = MotionType::Static;
    b.meshIndex = static_cast<uint32_t>(meshes_.meshes.size() - 1);
    b.invMass = 0.0f; // mesh colliders are static (level geometry)
    return addBody(b);
}

BodyId World::addStaticHeightfield(uint32_t width, uint32_t depth, float cellSize,
                                   const std::vector<float>& heights, Vec3 origin) {
    AccessGuard guard(*this, AccessKind::Mutation, "addStaticHeightfield");
    if (width < 2 || depth < 2)
        throw std::invalid_argument("velox: heightfield requires at least 2x2 samples");
    requirePositive(cellSize, "velox: heightfield cell size must be positive");
    requireFiniteVec(origin, "velox: heightfield origin must be finite");
    uint64_t sampleCount = (uint64_t)width * depth;
    if (sampleCount != heights.size() || sampleCount > UINT32_MAX)
        throw std::invalid_argument("velox: heightfield sample count does not match dimensions");
    for (float height : heights)
        if (!finiteFloat(height))
            throw std::invalid_argument("velox: heightfield samples must be finite");
    uint64_t indexCount = (uint64_t)(width - 1) * (depth - 1) * 6;
    if (indexCount > UINT32_MAX)
        throw std::length_error("velox: heightfield exceeds 32-bit index capacity");

    std::vector<Vec3> vertices;
    std::vector<uint32_t> indices;
    vertices.reserve((size_t)sampleCount);
    indices.reserve((size_t)indexCount);
    for (uint32_t z = 0; z < depth; ++z)
        for (uint32_t x = 0; x < width; ++x)
            vertices.push_back(origin + Vec3{x * cellSize, heights[z * width + x],
                                             z * cellSize});
    for (uint32_t z = 0; z + 1 < depth; ++z) {
        for (uint32_t x = 0; x + 1 < width; ++x) {
            uint32_t a = z * width + x;
            uint32_t b = a + 1;
            uint32_t c = a + width;
            uint32_t d = c + 1;
            indices.insert(indices.end(), {a, c, b, b, c, d});
        }
    }
    return addStaticMesh(vertices, indices);
}

void World::setGravity(Vec3 value) {
    AccessGuard guard(*this, AccessKind::Mutation, "setGravity");
    requireFiniteVec(value, "velox: gravity must be finite");
    gravity = value;
}

Vec3 World::gravityValue() const {
    AccessGuard guard(*this, AccessKind::Query, "gravityValue");
    return gravity;
}

void World::setSubsteps(int value) {
    AccessGuard guard(*this, AccessKind::Mutation, "setSubsteps");
    if (value <= 0) throw std::invalid_argument("velox: substeps must be positive");
    substeps = value;
}

int World::substepCount() const {
    AccessGuard guard(*this, AccessKind::Query, "substepCount");
    return substeps;
}

MotionType World::motionType(BodyId id) const {
    AccessGuard guard(*this, AccessKind::Query, "motionType");
    return bodies_[resolve(id)].motionType;
}

void World::setMotionType(BodyId id, MotionType type) {
    AccessGuard guard(*this, AccessKind::Mutation, "setMotionType");
    Body& b = body(id);
    if ((b.shape == ShapeType::Plane || b.shape == ShapeType::Mesh) &&
        type != MotionType::Static)
        throw std::invalid_argument("velox: plane and mesh colliders must remain static");
    if (type == MotionType::Dynamic && b.invMass <= 0.0f)
        throw std::invalid_argument("velox: dynamic motion requires positive mass");
    b.motionType = type;
    b.asleep = 0;
    b.sleepTimer = 0.0f;
    if (type == MotionType::Static) {
        b.velocity = {};
        b.angularVelocity = {};
    }
}

void World::mutateShape(BodyId id, const ShapeMutation& mutation) {
    AccessGuard guard(*this, AccessKind::Mutation, "mutateShape");
    BodyIndex dense = resolve(id);
    Body candidate = bodies_[dense];
    switch (mutation.type) {
    case ShapeMutation::Type::Sphere:
        requirePositive(mutation.radius, "velox: mutated sphere radius must be positive");
        candidate.shape = ShapeType::Sphere;
        candidate.radius = mutation.radius;
        break;
    case ShapeMutation::Type::Box:
        requireFiniteVec(mutation.halfExtents,
                         "velox: mutated box extents must be finite");
        if (mutation.halfExtents.x <= 0.0f || mutation.halfExtents.y <= 0.0f ||
            mutation.halfExtents.z <= 0.0f)
            throw std::invalid_argument("velox: mutated box extents must be positive");
        candidate.shape = ShapeType::Box;
        candidate.halfExtents = mutation.halfExtents;
        break;
    case ShapeMutation::Type::Capsule:
    case ShapeMutation::Type::Cylinder:
    case ShapeMutation::Type::Cone:
        requirePositive(mutation.radius, "velox: mutated primitive radius must be positive");
        if (mutation.type == ShapeMutation::Type::Capsule)
            requireNonNegative(mutation.capsuleHalfHeight,
                               "velox: mutated capsule half height must be non-negative");
        else
            requirePositive(mutation.capsuleHalfHeight,
                            "velox: mutated primitive half height must be positive");
        candidate.shape = mutation.type == ShapeMutation::Type::Capsule ? ShapeType::Capsule :
                          mutation.type == ShapeMutation::Type::Cylinder ? ShapeType::Cylinder :
                                                                           ShapeType::Cone;
        candidate.radius = mutation.radius;
        candidate.capsuleHalfHeight = mutation.capsuleHalfHeight;
        break;
    case ShapeMutation::Type::RoundedBox:
        requireFiniteVec(mutation.halfExtents,
                         "velox: mutated rounded box extents must be finite");
        if (mutation.halfExtents.x <= 0.0f || mutation.halfExtents.y <= 0.0f ||
            mutation.halfExtents.z <= 0.0f)
            throw std::invalid_argument("velox: mutated rounded box extents must be positive");
        requirePositive(mutation.radius, "velox: mutated rounded box radius must be positive");
        candidate.shape = ShapeType::RoundedBox;
        candidate.halfExtents = mutation.halfExtents;
        candidate.radius = mutation.radius;
        break;
    case ShapeMutation::Type::Ellipsoid:
        requireFiniteVec(mutation.halfExtents,
                         "velox: mutated ellipsoid radii must be finite");
        if (mutation.halfExtents.x <= 0.0f || mutation.halfExtents.y <= 0.0f ||
            mutation.halfExtents.z <= 0.0f)
            throw std::invalid_argument("velox: mutated ellipsoid radii must be positive");
        candidate.shape = ShapeType::Ellipsoid;
        candidate.halfExtents = mutation.halfExtents;
        break;
    case ShapeMutation::Type::Hull:
    case ShapeMutation::Type::Compound: {
        const float mass = candidate.invMass > 0.0f ? 1.0f / candidate.invMass : 0.0f;
        World generated(BackendType::Cpu);
        if (mutation.type == ShapeMutation::Type::Hull)
            generated.addConvexHull({}, mutation.hullPoints, mass);
        else
            generated.addCompound({}, mutation.compoundShapes, mass);
        const Body& geometry = generated.bodies_.front();
        MeshSoup updated = meshes_; // Strong guarantee: append before commit.
        const uint32_t hullBase = static_cast<uint32_t>(updated.hullPoints.size());
        const uint32_t faceBase = static_cast<uint32_t>(updated.hullFaceIndices.size());
        const uint32_t childBase = static_cast<uint32_t>(updated.compoundChildren.size());
        updated.hullPoints.insert(updated.hullPoints.end(),
                                  generated.meshes_.hullPoints.begin(),
                                  generated.meshes_.hullPoints.end());
        updated.hullFaceIndices.insert(updated.hullFaceIndices.end(),
                                       generated.meshes_.hullFaceIndices.begin(),
                                       generated.meshes_.hullFaceIndices.end());
        std::vector<CompoundChild> children = generated.meshes_.compoundChildren;
        for (CompoundChild& child : children) {
            child.hullFirst += hullBase;
            child.hullFaceFirst += faceBase;
        }
        updated.compoundChildren.insert(updated.compoundChildren.end(),
                                        children.begin(), children.end());
        candidate.shape = geometry.shape;
        candidate.position += geometry.position;
        candidate.radius = geometry.radius;
        candidate.halfExtents = geometry.halfExtents;
        candidate.capsuleHalfHeight = geometry.capsuleHalfHeight;
        candidate.hullFirst = geometry.hullFirst + hullBase;
        candidate.hullCount = geometry.hullCount;
        candidate.hullFaceFirst = geometry.hullFaceFirst + faceBase;
        candidate.hullFaceCount = geometry.hullFaceCount;
        candidate.compoundFirst = geometry.compoundFirst + childBase;
        candidate.compoundCount = geometry.compoundCount;
        if (!mutation.preserveMassProperties) {
            candidate.invMass = geometry.invMass;
            candidate.invInertia = geometry.invInertia;
            candidate.inertiaOrientation = geometry.inertiaOrientation;
        }
        meshes_ = std::move(updated);
        break;
    }
    default:
        throw std::invalid_argument("velox: invalid shape mutation type");
    }
    if (!mutation.preserveMassProperties) refreshPrimitiveInertia(candidate);
    bodies_[dense] = candidate;
    bodies_[dense].asleep = 0;
    bodies_[dense].sleepTimer = 0.0f;
    contacts_.erase(std::remove_if(contacts_.begin(), contacts_.end(),
        [&](const Contact& c) { return c.a == dense || c.b == dense; }), contacts_.end());
    prevContacts_.erase(std::remove_if(prevContacts_.begin(), prevContacts_.end(),
        [&](const Contact& c) { return c.a == dense || c.b == dense; }), prevContacts_.end());
    broadPhase_->touched = true;
    backend_->invalidateCaches();
}

void World::scaleShape(BodyId id, const ShapeScale& scale) {
    AccessGuard guard(*this, AccessKind::Mutation, "scaleShape");
    requireFiniteVec(scale.factor, "velox: shape scale must be finite");
    if (scale.factor.x <= 0.0f || scale.factor.y <= 0.0f || scale.factor.z <= 0.0f)
        throw std::invalid_argument("velox: shape scale must be positive");
    BodyIndex dense = resolve(id);
    const Body& body = bodies_[dense];
    ShapeMutation mutation;
    mutation.preserveMassProperties = !scale.updateMassProperties;
    switch (body.shape) {
    case ShapeType::Box:
        mutation.type = ShapeMutation::Type::Box;
        mutation.halfExtents = {body.halfExtents.x * scale.factor.x,
                                body.halfExtents.y * scale.factor.y,
                                body.halfExtents.z * scale.factor.z};
        break;
    case ShapeType::RoundedBox:
        if (scale.factor.x != scale.factor.y || scale.factor.y != scale.factor.z)
            throw std::invalid_argument("velox: rounded box scaling must be uniform");
        mutation.type = ShapeMutation::Type::RoundedBox;
        mutation.halfExtents = {body.halfExtents.x * scale.factor.x,
                                body.halfExtents.y * scale.factor.y,
                                body.halfExtents.z * scale.factor.z};
        mutation.radius = body.radius * scale.factor.x;
        break;
    case ShapeType::Ellipsoid:
        mutation.type = ShapeMutation::Type::Ellipsoid;
        mutation.halfExtents = {body.halfExtents.x * scale.factor.x,
                                body.halfExtents.y * scale.factor.y,
                                body.halfExtents.z * scale.factor.z};
        break;
    case ShapeType::Sphere:
    case ShapeType::Capsule:
    case ShapeType::Cylinder:
    case ShapeType::Cone:
        if (scale.factor.x != scale.factor.y || scale.factor.y != scale.factor.z)
            throw std::invalid_argument("velox: round primitive scaling must be uniform");
        mutation.type = body.shape == ShapeType::Sphere ? ShapeMutation::Type::Sphere :
                        body.shape == ShapeType::Capsule ? ShapeMutation::Type::Capsule :
                        body.shape == ShapeType::Cylinder ? ShapeMutation::Type::Cylinder :
                                                            ShapeMutation::Type::Cone;
        mutation.radius = body.radius * scale.factor.x;
        mutation.capsuleHalfHeight = body.capsuleHalfHeight * scale.factor.x;
        break;
    default:
        throw std::invalid_argument("velox: this collider cannot yet be scaled at runtime");
    }
    mutateShape(id, mutation);
}

void World::setCollisionMargin(BodyId id, float margin) {
    AccessGuard guard(*this, AccessKind::Mutation, "setCollisionMargin");
    requireNonNegative(margin, "velox: collision margin must be non-negative");
    Body& body = bodies_[resolve(id)];
    body.ccdTuning.collisionMargin = margin;
    body.asleep = 0;
    body.sleepTimer = 0.0f;
    broadPhase_->touched = true;
    backend_->invalidateCaches();
}

void World::setMassProperties(BodyId id, float mass, Vec3 principalInertia,
                              Quat principalOrientation) {
    AccessGuard guard(*this, AccessKind::Mutation, "setMassProperties");
    requirePositive(mass, "velox: body mass must be finite and positive");
    requireFiniteVec(principalInertia,
                     "velox: principal inertia must be finite");
    if (principalInertia.x <= 0.0f || principalInertia.y <= 0.0f ||
        principalInertia.z <= 0.0f)
        throw std::invalid_argument(
            "velox: principal inertia must be positive on every axis");
    if (!finiteQuat(principalOrientation))
        throw std::invalid_argument(
            "velox: principal inertia orientation must be finite");
    float q2 = principalOrientation.x * principalOrientation.x +
               principalOrientation.y * principalOrientation.y +
               principalOrientation.z * principalOrientation.z +
               principalOrientation.w * principalOrientation.w;
    if (q2 < 1e-12f)
        throw std::invalid_argument(
            "velox: principal inertia orientation must be non-zero");
    BodyIndex dense = resolve(id);
    Body& b = bodies_[dense];
    if (b.shape == ShapeType::Plane || b.shape == ShapeType::Mesh)
        throw std::invalid_argument(
            "velox: static-only geometry cannot have mass properties");
    b.invMass = 1.0f / mass;
    b.invInertia = {1.0f / principalInertia.x,
                    1.0f / principalInertia.y,
                    1.0f / principalInertia.z};
    b.inertiaOrientation = normalize(principalOrientation);
    b.asleep = 0;
    b.sleepTimer = 0.0f;
    prevContacts_.erase(
        std::remove_if(prevContacts_.begin(), prevContacts_.end(),
                       [&](const Contact& contact) {
                           return contact.a == dense || contact.b == dense;
                       }),
        prevContacts_.end());
    backend_->invalidateCaches();
}

void World::setTransform(BodyId id, Vec3 position, Quat orientation) {
    AccessGuard guard(*this, AccessKind::Mutation, "setTransform");
    requireFiniteVec(position, "velox: body position must be finite");
    if (!finiteQuat(orientation))
        throw std::invalid_argument("velox: body orientation must be finite");
    float q2 = orientation.x * orientation.x + orientation.y * orientation.y +
               orientation.z * orientation.z + orientation.w * orientation.w;
    if (q2 < 1e-12f)
        throw std::invalid_argument("velox: body orientation must be non-zero");
    Body& b = body(id);
    if (b.shape == ShapeType::Plane || b.shape == ShapeType::Mesh)
        throw std::invalid_argument("velox: plane and mesh transforms are baked into geometry");
    b.position = position;
    b.orientation = normalize(orientation);
    wake(id);
}

void World::setLinearVelocity(BodyId id, Vec3 velocity) {
    AccessGuard guard(*this, AccessKind::Mutation, "setLinearVelocity");
    requireFiniteVec(velocity, "velox: linear velocity must be finite");
    Body& b = body(id);
    if (b.isStatic()) throw std::invalid_argument("velox: static bodies cannot have velocity");
    if (b.isLocked()) {
        b.velocity = {};
        return;
    }
    b.velocity = velocity;
    wake(id);
}

void World::setAngularVelocity(BodyId id, Vec3 velocity) {
    AccessGuard guard(*this, AccessKind::Mutation, "setAngularVelocity");
    requireFiniteVec(velocity, "velox: angular velocity must be finite");
    Body& b = body(id);
    if (b.isStatic()) throw std::invalid_argument("velox: static bodies cannot have velocity");
    if (b.isRotationLocked()) {
        b.angularVelocity = {};
        return;
    }
    b.angularVelocity = velocity;
    wake(id);
}

void World::addForce(BodyId id, Vec3 force) {
    AccessGuard guard(*this, AccessKind::Mutation, "addForce");
    requireFiniteVec(force, "velox: force must be finite");
    Body& b = body(id);
    if (!b.isDynamic()) throw std::invalid_argument("velox: forces require a dynamic body");
    if (b.isLocked()) return;
    b.force += force;
    wake(id);
}

void World::addForceAtPoint(BodyId id, Vec3 force, Vec3 worldPoint) {
    AccessGuard guard(*this, AccessKind::Mutation, "addForceAtPoint");
    requireFiniteVec(force, "velox: force must be finite");
    requireFiniteVec(worldPoint, "velox: force point must be finite");
    Body& b = body(id);
    if (!b.isDynamic()) throw std::invalid_argument("velox: forces require a dynamic body");
    if (b.isLocked()) return;
    b.force += force;
    b.torque += cross(worldPoint - b.position, force);
    wake(id);
}

void World::addTorque(BodyId id, Vec3 torque) {
    AccessGuard guard(*this, AccessKind::Mutation, "addTorque");
    requireFiniteVec(torque, "velox: torque must be finite");
    Body& b = body(id);
    if (!b.isDynamic()) throw std::invalid_argument("velox: torque requires a dynamic body");
    if (b.isLocked()) return;
    b.torque += torque;
    wake(id);
}

void World::addLinearImpulse(BodyId id, Vec3 impulse) {
    AccessGuard guard(*this, AccessKind::Mutation, "addLinearImpulse");
    requireFiniteVec(impulse, "velox: impulse must be finite");
    Body& b = body(id);
    if (!b.isDynamic()) throw std::invalid_argument("velox: impulses require a dynamic body");
    if (b.isLocked()) return;
    b.velocity += impulse * b.solverInvMass();
    wake(id);
}

void World::addImpulseAtPoint(BodyId id, Vec3 impulse, Vec3 worldPoint) {
    AccessGuard guard(*this, AccessKind::Mutation, "addImpulseAtPoint");
    requireFiniteVec(impulse, "velox: impulse must be finite");
    requireFiniteVec(worldPoint, "velox: impulse point must be finite");
    Body& b = body(id);
    if (!b.isDynamic()) throw std::invalid_argument("velox: impulses require a dynamic body");
    if (b.isLocked()) return;
    b.velocity += impulse * b.solverInvMass();
    b.angularVelocity += b.invInertiaMul(cross(worldPoint - b.position, impulse));
    wake(id);
}

void World::clearForces(BodyId id) {
    AccessGuard guard(*this, AccessKind::Mutation, "clearForces");
    Body& b = body(id);
    b.force = {};
    b.torque = {};
}

void World::setSensor(BodyId id, bool enabled) {
    AccessGuard guard(*this, AccessKind::Mutation, "setSensor");
    Body& b = body(id);
    b.sensor = enabled ? 1 : 0;
    contacts_.erase(std::remove_if(contacts_.begin(), contacts_.end(),
        [&](const Contact& c) { return c.a == resolve(id) || c.b == resolve(id); }),
        contacts_.end());
    broadPhase_->touched = true;
}

bool World::isSensor(BodyId id) const {
    AccessGuard guard(*this, AccessKind::Query, "isSensor");
    return bodies_[resolve(id)].sensor != 0;
}

void World::setGravityScale(BodyId id, float scale) {
    AccessGuard guard(*this, AccessKind::Mutation, "setGravityScale");
    if (!finiteFloat(scale))
        throw std::invalid_argument("velox: gravity scale must be finite");
    body(id).gravityScale = scale;
}

float World::gravityScale(BodyId id) const {
    AccessGuard guard(*this, AccessKind::Query, "gravityScale");
    return bodies_[resolve(id)].gravityScale;
}

void World::setLinearDamping(BodyId id, float damping) {
    AccessGuard guard(*this, AccessKind::Mutation, "setLinearDamping");
    if (!finiteFloat(damping) || damping < 0.0f)
        throw std::invalid_argument("velox: linear damping must be finite and non-negative");
    body(id).linearDamping = damping;
}

void World::setAngularDamping(BodyId id, float damping) {
    AccessGuard guard(*this, AccessKind::Mutation, "setAngularDamping");
    if (!finiteFloat(damping) || damping < 0.0f)
        throw std::invalid_argument("velox: angular damping must be finite and non-negative");
    body(id).angularDamping = damping;
}

void World::setCollisionFilter(BodyId id, uint32_t category, uint32_t mask) {
    AccessGuard guard(*this, AccessKind::Mutation, "setCollisionFilter");
    Body& b = body(id);
    b.categoryBits = category;
    b.maskBits = mask;
    contacts_.erase(std::remove_if(contacts_.begin(), contacts_.end(),
        [&](const Contact& c) { return c.a == resolve(id) || c.b == resolve(id); }),
        contacts_.end());
    broadPhase_->touched = true;
}

void World::setCollisionFilter(BodyId id, const CollisionFilterData& filter) {
    AccessGuard guard(*this, AccessKind::Mutation, "setCollisionFilter");
    Body& b = body(id);
    b.categoryBits = filter.categoryBits;
    b.maskBits = filter.maskBits;
    b.groupIndex = filter.groupIndex;
    contacts_.erase(std::remove_if(contacts_.begin(), contacts_.end(),
        [&](const Contact& c) { return c.a == resolve(id) || c.b == resolve(id); }),
        contacts_.end());
    broadPhase_->touched = true;
}

CollisionFilterData World::collisionFilter(BodyId id) const {
    AccessGuard guard(*this, AccessKind::Query, "collisionFilter");
    const Body& b = body(id);
    return CollisionFilterData{b.categoryBits, b.maskBits, b.groupIndex};
}

void World::setEnableSleep(BodyId id, bool enabled) {
    AccessGuard guard(*this, AccessKind::Mutation, "setEnableSleep");
    Body& b = body(id);
    b.enableSleep = enabled ? 1 : 0;
    if (!enabled) {
        b.asleep = 0;
        b.sleepTimer = 0.0f;
    }
}

bool World::isSleepEnabled(BodyId id) const {
    AccessGuard guard(*this, AccessKind::Query, "isSleepEnabled");
    return bodies_[resolve(id)].enableSleep != 0;
}

void World::setFixedRotation(BodyId id, bool enabled) {
    AccessGuard guard(*this, AccessKind::Mutation, "setFixedRotation");
    Body& b = body(id);
    b.fixedRotation = enabled ? 1 : 0;
    if (enabled) {
        b.angularVelocity = {};
        b.torque = {};
    }
}

bool World::isFixedRotation(BodyId id) const {
    AccessGuard guard(*this, AccessKind::Query, "isFixedRotation");
    return bodies_[resolve(id)].fixedRotation != 0;
}

void World::wakeBody(BodyId id) {
    AccessGuard guard(*this, AccessKind::Mutation, "wakeBody");
    Body& b = body(id);
    sleepManager_.wakeBody(b, id);
}

void World::sleepBody(BodyId id) {
    AccessGuard guard(*this, AccessKind::Mutation, "sleepBody");
    Body& b = body(id);
    sleepManager_.sleepBody(b, id);
}

void World::explode(Vec3 origin, float radius, float impulse) {
    AccessGuard guard(*this, AccessKind::Mutation, "explode");
    requireFiniteVec(origin, "velox: explosion origin must be finite");
    if (!finiteFloat(radius))
        throw std::invalid_argument("velox: explosion radius must be finite");
    if (radius < 0.0f)
        throw std::invalid_argument("velox: explosion radius must be non-negative");
    if (radius <= 0.0f || impulse <= 0.0f) return;
    float invRadius = 1.0f / radius;
    for (Body& b : bodies_) {
        if (!b.isDynamic() || isFullyAsleep(b.asleep)) continue;
        Vec3 delta = b.position - origin;
        float dist = length(delta);
        if (dist >= radius || dist < 1e-8f) continue;
        float falloff = 1.0f - dist * invRadius;
        Vec3 dir = delta * (1.0f / dist);
        b.velocity += dir * (impulse * falloff * b.invMass);
        b.asleep = 0;
        b.sleepTimer = 0.0f;
    }
}

void World::shiftOrigin(Vec3 offset) {
    AccessGuard guard(*this, AccessKind::Mutation, "shiftOrigin");
    requireFiniteVec(offset, "velox: origin shift must be finite");
    broadPhase_->structureDirty = true; // all stored bounds move wholesale

    // Validate every subtraction before mutating anything. Opposite, finite
    // FLT_MAX-scale operands can still overflow, so this preserves strong
    // exception safety for malformed large-world inputs.
    auto shifted = [&](const Vec3& value) {
        Vec3 result = value - offset;
        if (!finiteVec(result))
            throw std::overflow_error("velox: origin shift overflows world coordinates");
        return result;
    };
    for (const Body& body : bodies_) {
        shifted(body.position);
        if (body.shape == ShapeType::Plane) {
            float planeOffset = body.planeOffset - dot(body.planeNormal, offset);
            if (!finiteFloat(planeOffset))
                throw std::overflow_error("velox: origin shift overflows plane offset");
        }
    }
    for (const Vec3& vertex : meshes_.vertices) shifted(vertex);
    for (const Mesh& mesh : meshes_.meshes) {
        shifted(mesh.aabbMin);
        shifted(mesh.aabbMax);
    }
    for (const BvhNode& node : meshes_.bvhNodes) {
        shifted(node.aabbMin);
        shifted(node.aabbMax);
    }
    for (const Contact& contact : contacts_) shifted(contact.point);
    for (const Contact& contact : prevContacts_) shifted(contact.point);
    for (const PrevState& state : prev_) shifted(state.position);
    for (const ContactEvent& event : events_)
        if (event.type != ContactEventType::End) shifted(event.point);

    for (Body& body : bodies_) {
        body.position -= offset;
        if (body.shape == ShapeType::Plane)
            body.planeOffset -= dot(body.planeNormal, offset);
    }
    for (Vec3& vertex : meshes_.vertices) vertex -= offset;
    for (Mesh& mesh : meshes_.meshes) {
        mesh.aabbMin -= offset;
        mesh.aabbMax -= offset;
    }
    for (BvhNode& node : meshes_.bvhNodes) {
        node.aabbMin -= offset;
        node.aabbMax -= offset;
    }
    for (Contact& contact : contacts_) contact.point -= offset;
    for (Contact& contact : prevContacts_) contact.point -= offset;
    for (PrevState& state : prev_) state.position -= offset;
    for (ContactEvent& event : events_)
        if (event.type != ContactEventType::End) event.point -= offset;

    backend_->invalidateCaches();
}

WorldSnapshot World::saveSnapshot() const {
    AccessGuard guard(*this, AccessKind::Query, "saveSnapshot");
    WorldSnapshot snapshot;
    snapshot.owner_ = this;
    snapshot.gravity_ = gravity;
    snapshot.substeps_ = substeps;
    snapshot.ccdDefaults_ = ccdDefaults_;
    snapshot.multiToiSettings_ = multiToiSettings_;
    snapshot.solverOptions_ = solverOptions_;
    snapshot.bodies_ = bodies_;
    snapshot.bodySlots_.reserve(bodySlots_.size());
    for (const HandleSlot& slot : bodySlots_)
        snapshot.bodySlots_.push_back({slot.dense, slot.generation});
    snapshot.bodyDenseToSlot_ = bodyDenseToSlot_;
    snapshot.freeBodySlots_ = freeBodySlots_;
    snapshot.contacts_ = contacts_;
    snapshot.prevContacts_ = prevContacts_;
    snapshot.joints_ = joints_;
    snapshot.jointSlots_.reserve(jointSlots_.size());
    for (const HandleSlot& slot : jointSlots_)
        snapshot.jointSlots_.push_back({slot.dense, slot.generation});
    snapshot.jointDenseToSlot_ = jointDenseToSlot_;
    snapshot.freeJointSlots_ = freeJointSlots_;
    snapshot.previous_.reserve(prev_.size());
    for (const PrevState& state : prev_)
        snapshot.previous_.push_back({state.position, state.orientation});
    snapshot.pairKeys_ = pairKeys_;
    snapshot.previousPairKeys_ = prevPairKeys_;
    snapshot.unionParent_ = unionParent_;
    snapshot.islandTimer_ = islandTimer_;
    snapshot.contactEvents_ = events_;
    snapshot.jointBreakEvents_ = jointBreakEvents_;
    snapshot.meshes_ = meshes_;
    snapshot.lastStepStats_ = lastStepStats_;
    return snapshot;
}

void World::restoreSnapshot(const WorldSnapshot& snapshot) {
    AccessGuard guard(*this, AccessKind::Mutation, "restoreSnapshot");
    if (snapshot.owner_ != this)
        throw std::invalid_argument(
            "velox: a snapshot can only be restored to its originating world");
    broadPhase_->structureDirty = true; // bodies and geometry replaced wholesale

    // Allocate every converted private representation before changing World,
    // giving restore strong exception safety.
    WorldSnapshot staged = snapshot;
    validateSolverOptions(staged.solverOptions_);
    std::vector<HandleSlot> bodySlots;
    bodySlots.reserve(staged.bodySlots_.size());
    for (const WorldSnapshot::Slot& slot : staged.bodySlots_)
        bodySlots.push_back({slot.dense, slot.generation});
    std::vector<HandleSlot> jointSlots;
    jointSlots.reserve(staged.jointSlots_.size());
    for (const WorldSnapshot::Slot& slot : staged.jointSlots_)
        jointSlots.push_back({slot.dense, slot.generation});
    std::vector<PrevState> previous;
    previous.reserve(staged.previous_.size());
    for (const WorldSnapshot::Previous& state : staged.previous_)
        previous.push_back({state.position, state.orientation});

    gravity = staged.gravity_;
    substeps = staged.substeps_;
    ccdDefaults_ = staged.ccdDefaults_;
    multiToiSettings_ = staged.multiToiSettings_;
    solverOptions_ = staged.solverOptions_;
    if (solverOptions_.iterationPolicy == IterationPolicy::Adaptive &&
        std::string(backend_->name()) == "cuda")
        resetBackend(BackendType::Cpu);
    bodies_.swap(staged.bodies_);
    bodySlots_.swap(bodySlots);
    bodyDenseToSlot_.swap(staged.bodyDenseToSlot_);
    freeBodySlots_.swap(staged.freeBodySlots_);
    contacts_.swap(staged.contacts_);
    prevContacts_.swap(staged.prevContacts_);
    joints_.swap(staged.joints_);
    jointSlots_.swap(jointSlots);
    jointDenseToSlot_.swap(staged.jointDenseToSlot_);
    freeJointSlots_.swap(staged.freeJointSlots_);
    prev_.swap(previous);
    pairKeys_.swap(staged.pairKeys_);
    prevPairKeys_.swap(staged.previousPairKeys_);
    unionParent_.swap(staged.unionParent_);
    islandTimer_.swap(staged.islandTimer_);
    events_.swap(staged.contactEvents_);
    jointBreakEvents_.swap(staged.jointBreakEvents_);
    std::swap(meshes_, staged.meshes_);
    lastStepStats_ = staged.lastStepStats_;
    backend_->invalidateCaches();
}

JointId World::addBallJoint(BodyId a, BodyId b, Vec3 worldAnchor) {
    AccessGuard guard(*this, AccessKind::Mutation, "addBallJoint");
    BodyIndex ia = resolve(a), ib = resolve(b);
    if (ia == ib) throw std::invalid_argument("velox: a joint requires two different bodies");
    requireFiniteVec(worldAnchor, "velox: ball-joint anchor must be finite");
    Joint j{};
    j.type = JointType::Ball;
    j.a = ia; j.b = ib;
    j.localAnchorA = rotateInv(bodies_[ia].orientation, worldAnchor - bodies_[ia].position);
    j.localAnchorB = rotateInv(bodies_[ib].orientation, worldAnchor - bodies_[ib].position);
    return addJoint(j);
}

JointId World::addDistanceJoint(BodyId a, BodyId b, Vec3 worldAnchorA, Vec3 worldAnchorB) {
    AccessGuard guard(*this, AccessKind::Mutation, "addDistanceJoint");
    BodyIndex ia = resolve(a), ib = resolve(b);
    if (ia == ib) throw std::invalid_argument("velox: a joint requires two different bodies");
    requireFiniteVec(worldAnchorA, "velox: distance-joint anchor must be finite");
    requireFiniteVec(worldAnchorB, "velox: distance-joint anchor must be finite");
    Joint j{};
    j.type = JointType::Distance;
    j.a = ia; j.b = ib;
    j.localAnchorA = rotateInv(bodies_[ia].orientation, worldAnchorA - bodies_[ia].position);
    j.localAnchorB = rotateInv(bodies_[ib].orientation, worldAnchorB - bodies_[ib].position);
    j.restLength = length(worldAnchorA - worldAnchorB);
    return addJoint(j);
}

JointId World::addSpringJoint(BodyId a, BodyId b, Vec3 worldAnchorA,
                              Vec3 worldAnchorB, float frequencyHz,
                              float dampingRatio) {
    AccessGuard guard(*this, AccessKind::Mutation, "addSpringJoint");
    requirePositive(frequencyHz,
                    "velox: spring frequency must be finite and positive");
    requireNonNegative(dampingRatio,
                       "velox: spring damping ratio must be finite and non-negative");
    JointId id = addDistanceJoint(a, b, worldAnchorA, worldAnchorB);
    Joint& spring = joint(id);
    spring.enableSpring = true;
    spring.springFrequencyHz = frequencyHz;
    spring.springDampingRatio = dampingRatio;
    return id;
}

JointId World::addHingeJoint(BodyId a, BodyId b, Vec3 worldAnchor, Vec3 worldAxis) {
    AccessGuard guard(*this, AccessKind::Mutation, "addHingeJoint");
    BodyIndex ia = resolve(a), ib = resolve(b);
    if (ia == ib) throw std::invalid_argument("velox: a joint requires two different bodies");
    requireFiniteVec(worldAnchor, "velox: hinge anchor must be finite");
    requireFiniteVec(worldAxis, "velox: hinge axis must be finite");
    if (lengthSq(worldAxis) < 1e-12f)
        throw std::invalid_argument("velox: hinge axis must be non-zero");
    Joint j{};
    j.type = JointType::Hinge;
    j.a = ia; j.b = ib;
    j.localAnchorA = rotateInv(bodies_[ia].orientation, worldAnchor - bodies_[ia].position);
    j.localAnchorB = rotateInv(bodies_[ib].orientation, worldAnchor - bodies_[ib].position);
    Vec3 axis = normalize(worldAxis);
    j.localAxisA = rotateInv(bodies_[ia].orientation, axis);
    j.localAxisB = rotateInv(bodies_[ib].orientation, axis);
    // Shared perpendicular reference: measures the joint angle (0 now).
    Vec3 ref = std::fabs(axis.x) < 0.9f ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
    ref = normalize(cross(axis, ref));
    j.localRefA = rotateInv(bodies_[ia].orientation, ref);
    j.localRefB = rotateInv(bodies_[ib].orientation, ref);
    return addJoint(j);
}

JointId World::addConeTwistJoint(BodyId a, BodyId b, Vec3 worldAnchor, Vec3 worldAxis) {
    AccessGuard guard(*this, AccessKind::Mutation, "addConeTwistJoint");
    BodyIndex ia = resolve(a), ib = resolve(b);
    if (ia == ib) throw std::invalid_argument("velox: a joint requires two different bodies");
    requireFiniteVec(worldAnchor, "velox: cone/twist anchor must be finite");
    requireFiniteVec(worldAxis, "velox: cone/twist axis must be finite");
    if (lengthSq(worldAxis) < 1e-12f)
        throw std::invalid_argument("velox: cone/twist axis must be non-zero");
    Vec3 axis = normalize(worldAxis);
    Vec3 ref = std::fabs(axis.x) < 0.9f ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
    ref = normalize(cross(axis, ref));

    Joint j{};
    j.type = JointType::ConeTwist;
    j.a = ia; j.b = ib;
    j.localAnchorA = rotateInv(bodies_[ia].orientation, worldAnchor - bodies_[ia].position);
    j.localAnchorB = rotateInv(bodies_[ib].orientation, worldAnchor - bodies_[ib].position);
    j.localAxisA = rotateInv(bodies_[ia].orientation, axis);
    j.localAxisB = rotateInv(bodies_[ib].orientation, axis);
    j.localRefA = rotateInv(bodies_[ia].orientation, ref);
    j.localRefB = rotateInv(bodies_[ib].orientation, ref);
    return addJoint(j);
}

JointId World::addFixedJoint(BodyId a, BodyId b, Vec3 worldAnchor) {
    AccessGuard guard(*this, AccessKind::Mutation, "addFixedJoint");
    BodyIndex ia = resolve(a), ib = resolve(b);
    if (ia == ib) throw std::invalid_argument("velox: a joint requires two different bodies");
    requireFiniteVec(worldAnchor, "velox: fixed-joint anchor must be finite");
    Joint j{};
    j.type = JointType::Fixed;
    j.a = ia; j.b = ib;
    j.localAnchorA = rotateInv(bodies_[ia].orientation,
                               worldAnchor - bodies_[ia].position);
    j.localAnchorB = rotateInv(bodies_[ib].orientation,
                               worldAnchor - bodies_[ib].position);
    j.localAxisA = rotateInv(bodies_[ia].orientation, {1, 0, 0});
    j.localAxisB = rotateInv(bodies_[ib].orientation, {1, 0, 0});
    j.localRefA = rotateInv(bodies_[ia].orientation, {0, 1, 0});
    j.localRefB = rotateInv(bodies_[ib].orientation, {0, 1, 0});
    return addJoint(j);
}

JointId World::addPrismaticJoint(BodyId a, BodyId b, Vec3 worldAnchor,
                                 Vec3 worldAxis) {
    AccessGuard guard(*this, AccessKind::Mutation, "addPrismaticJoint");
    BodyIndex ia = resolve(a), ib = resolve(b);
    if (ia == ib) throw std::invalid_argument("velox: a joint requires two different bodies");
    requireFiniteVec(worldAnchor, "velox: prismatic-joint anchor must be finite");
    requireFiniteVec(worldAxis, "velox: prismatic-joint axis must be finite");
    if (lengthSq(worldAxis) < 1e-12f)
        throw std::invalid_argument("velox: prismatic-joint axis must be non-zero");
    Vec3 axis = normalize(worldAxis);
    Vec3 ref = std::fabs(axis.x) < 0.9f ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
    ref = normalize(cross(axis, ref));
    Joint j{};
    j.type = JointType::Prismatic;
    j.a = ia; j.b = ib;
    j.localAnchorA = rotateInv(bodies_[ia].orientation,
                               worldAnchor - bodies_[ia].position);
    j.localAnchorB = rotateInv(bodies_[ib].orientation,
                               worldAnchor - bodies_[ib].position);
    j.localAxisA = rotateInv(bodies_[ia].orientation, axis);
    j.localAxisB = rotateInv(bodies_[ib].orientation, axis);
    j.localRefA = rotateInv(bodies_[ia].orientation, ref);
    j.localRefB = rotateInv(bodies_[ib].orientation, ref);
    return addJoint(j);
}

JointId World::addSixDofJoint(BodyId a, BodyId b, Vec3 worldAnchor) {
    AccessGuard guard(*this, AccessKind::Mutation, "addSixDofJoint");
    BodyIndex ia = resolve(a), ib = resolve(b);
    if (ia == ib) throw std::invalid_argument("velox: a joint requires two different bodies");
    requireFiniteVec(worldAnchor, "velox: 6DoF-joint anchor must be finite");
    Joint j{};
    j.type = JointType::SixDof;
    j.a = ia; j.b = ib;
    j.localAnchorA = rotateInv(bodies_[ia].orientation,
                               worldAnchor - bodies_[ia].position);
    j.localAnchorB = rotateInv(bodies_[ib].orientation,
                               worldAnchor - bodies_[ib].position);
    j.localAxisA = rotateInv(bodies_[ia].orientation, {1, 0, 0});
    j.localAxisB = rotateInv(bodies_[ib].orientation, {1, 0, 0});
    j.localRefA = rotateInv(bodies_[ia].orientation, {0, 1, 0});
    j.localRefB = rotateInv(bodies_[ib].orientation, {0, 1, 0});
    return addJoint(j);
}

JointId World::addMotorJoint(BodyId a, BodyId b, Vec3 worldAnchorA, Vec3 worldAnchorB,
                             float maxForce, float maxTorque) {
    AccessGuard guard(*this, AccessKind::Mutation, "addMotorJoint");
    BodyIndex ia = resolve(a), ib = resolve(b);
    if (ia == ib) throw std::invalid_argument("velox: a joint requires two different bodies");
    requireFiniteVec(worldAnchorA, "velox: motor joint anchor A must be finite");
    requireFiniteVec(worldAnchorB, "velox: motor joint anchor B must be finite");
    if (maxForce < 0.0f || maxTorque < 0.0f)
        throw std::invalid_argument("velox: motor joint limits must be non-negative");
    Joint j{};
    j.type = JointType::Motor;
    j.a = ia; j.b = ib;
    j.localAnchorA = rotateInv(bodies_[ia].orientation,
                               worldAnchorA - bodies_[ia].position);
    j.localAnchorB = rotateInv(bodies_[ib].orientation,
                               worldAnchorB - bodies_[ib].position);
    j.localAxisA = rotateInv(bodies_[ia].orientation, {1, 0, 0});
    j.localAxisB = rotateInv(bodies_[ib].orientation, {1, 0, 0});
    j.localRefA = rotateInv(bodies_[ia].orientation, {0, 1, 0});
    j.localRefB = rotateInv(bodies_[ib].orientation, {0, 1, 0});
    j.maxMotorForce = maxForce;
    j.maxMotorTorque = maxTorque;
    return addJoint(j);
}

JointId World::addWeldJoint(BodyId a, BodyId b, Vec3 worldAnchor,
                            float breakForce, float breakTorque) {
    AccessGuard guard(*this, AccessKind::Mutation, "addWeldJoint");
    BodyIndex ia = resolve(a), ib = resolve(b);
    if (ia == ib) throw std::invalid_argument("velox: a joint requires two different bodies");
    requireFiniteVec(worldAnchor, "velox: weld-joint anchor must be finite");
    if (!finiteFloat(breakForce) || breakForce < 0.0f)
        throw std::invalid_argument("velox: weld break force must be finite and non-negative");
    if (!finiteFloat(breakTorque) || breakTorque < 0.0f)
        throw std::invalid_argument("velox: weld break torque must be finite and non-negative");
    Joint j{};
    j.type = JointType::Weld;
    j.a = ia; j.b = ib;
    j.localAnchorA = rotateInv(bodies_[ia].orientation,
                               worldAnchor - bodies_[ia].position);
    j.localAnchorB = rotateInv(bodies_[ib].orientation,
                               worldAnchor - bodies_[ib].position);
    j.localAxisA = rotateInv(bodies_[ia].orientation, {1, 0, 0});
    j.localAxisB = rotateInv(bodies_[ib].orientation, {1, 0, 0});
    j.localRefA = rotateInv(bodies_[ia].orientation, {0, 1, 0});
    j.localRefB = rotateInv(bodies_[ib].orientation, {0, 1, 0});
    j.breakForce = breakForce;
    j.breakTorque = breakTorque;
    return addJoint(j);
}

JointId World::addWheelJoint(BodyId a, BodyId b, Vec3 worldAnchor, Vec3 worldAxis) {
    AccessGuard guard(*this, AccessKind::Mutation, "addWheelJoint");
    BodyIndex ia = resolve(a), ib = resolve(b);
    if (ia == ib) throw std::invalid_argument("velox: a joint requires two different bodies");
    requireFiniteVec(worldAnchor, "velox: wheel-joint anchor must be finite");
    requireFiniteVec(worldAxis, "velox: wheel-joint axis must be finite");
    if (lengthSq(worldAxis) < 1e-12f)
        throw std::invalid_argument("velox: wheel-joint axis must be non-zero");
    Vec3 axis = normalize(worldAxis);
    Vec3 ref = std::fabs(axis.x) < 0.9f ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
    ref = normalize(cross(axis, ref));
    Joint j{};
    j.type = JointType::Wheel;
    j.a = ia; j.b = ib;
    j.localAnchorA = rotateInv(bodies_[ia].orientation,
                               worldAnchor - bodies_[ia].position);
    j.localAnchorB = rotateInv(bodies_[ib].orientation,
                               worldAnchor - bodies_[ib].position);
    j.localAxisA = rotateInv(bodies_[ia].orientation, axis);
    j.localAxisB = rotateInv(bodies_[ib].orientation, axis);
    j.localRefA = rotateInv(bodies_[ia].orientation, ref);
    j.localRefB = rotateInv(bodies_[ib].orientation, ref);
    return addJoint(j);
}

JointId World::addRopeJoint(BodyId a, BodyId b, Vec3 worldAnchorA, Vec3 worldAnchorB,
                            float maxLength) {
    AccessGuard guard(*this, AccessKind::Mutation, "addRopeJoint");
    BodyIndex ia = resolve(a), ib = resolve(b);
    if (ia == ib) throw std::invalid_argument("velox: a joint requires two different bodies");
    requireFiniteVec(worldAnchorA, "velox: rope-joint anchor A must be finite");
    requireFiniteVec(worldAnchorB, "velox: rope-joint anchor B must be finite");
    requirePositive(maxLength, "velox: rope max length must be finite and positive");
    Joint j{};
    j.type = JointType::Rope;
    j.a = ia; j.b = ib;
    j.localAnchorA = rotateInv(bodies_[ia].orientation,
                               worldAnchorA - bodies_[ia].position);
    j.localAnchorB = rotateInv(bodies_[ib].orientation,
                               worldAnchorB - bodies_[ib].position);
    j.maxLength = maxLength;
    return addJoint(j);
}

JointId World::addPulleyJoint(BodyId a, BodyId b, Vec3 worldAnchorA, Vec3 worldAnchorB,
                              Vec3 groundAnchorA, Vec3 groundAnchorB, float ratio) {
    AccessGuard guard(*this, AccessKind::Mutation, "addPulleyJoint");
    BodyIndex ia = resolve(a), ib = resolve(b);
    if (ia == ib) throw std::invalid_argument("velox: a joint requires two different bodies");
    requireFiniteVec(worldAnchorA, "velox: pulley-joint anchor A must be finite");
    requireFiniteVec(worldAnchorB, "velox: pulley-joint anchor B must be finite");
    requireFiniteVec(groundAnchorA, "velox: pulley ground anchor A must be finite");
    requireFiniteVec(groundAnchorB, "velox: pulley ground anchor B must be finite");
    requirePositive(ratio, "velox: pulley ratio must be finite and positive");
    Joint j{};
    j.type = JointType::Pulley;
    j.a = ia; j.b = ib;
    j.localAnchorA = rotateInv(bodies_[ia].orientation,
                               worldAnchorA - bodies_[ia].position);
    j.localAnchorB = rotateInv(bodies_[ib].orientation,
                               worldAnchorB - bodies_[ib].position);
    j.pulleyAnchorA = groundAnchorA;
    j.pulleyAnchorB = groundAnchorB;
    j.pulleyRatio = ratio;
    // Store the rest total length: lenA + ratio * lenB at creation.
    float lenA = length(worldAnchorA - groundAnchorA);
    float lenB = length(worldAnchorB - groundAnchorB);
    j.restLength = lenA + ratio * lenB;
    return addJoint(j);
}

JointId World::addGearJoint(BodyId a, BodyId b, Vec3 worldAxisA, Vec3 worldAxisB,
                            float ratio) {
    AccessGuard guard(*this, AccessKind::Mutation, "addGearJoint");
    BodyIndex ia = resolve(a), ib = resolve(b);
    if (ia == ib) throw std::invalid_argument("velox: a joint requires two different bodies");
    requireFiniteVec(worldAxisA, "velox: gear-joint axis A must be finite");
    requireFiniteVec(worldAxisB, "velox: gear-joint axis B must be finite");
    if (lengthSq(worldAxisA) < 1e-12f)
        throw std::invalid_argument("velox: gear-joint axis A must be non-zero");
    if (lengthSq(worldAxisB) < 1e-12f)
        throw std::invalid_argument("velox: gear-joint axis B must be non-zero");
    if (!finiteFloat(ratio) || ratio == 0.0f)
        throw std::invalid_argument("velox: gear ratio must be finite and non-zero");
    Joint j{};
    j.type = JointType::Gear;
    j.a = ia; j.b = ib;
    // Gear joint anchors are at the body centers (no linear constraint).
    j.localAnchorA = {};
    j.localAnchorB = {};
    j.localAxisA = rotateInv(bodies_[ia].orientation, normalize(worldAxisA));
    j.localAxisB = rotateInv(bodies_[ib].orientation, normalize(worldAxisB));
    j.gearRatio = ratio;
    return addJoint(j);
}

float World::hingeAngle(JointId id) const {
    AccessGuard guard(*this, AccessKind::Query, "hingeAngle");
    const Joint& j = joint(id);
    if (j.type != JointType::Hinge)
        throw std::invalid_argument("velox: hingeAngle requires a hinge joint");
    const Body& a = bodies_[j.a];
    const Body& b = bodies_[j.b];
    Vec3 axis = rotate(a.orientation, j.localAxisA);
    Vec3 refA = rotate(a.orientation, j.localRefA);
    Vec3 refB = rotate(b.orientation, j.localRefB);
    // Signed angle from refA to refB about the axis.
    float s = dot(cross(refA, refB), axis);
    float c = dot(refA, refB);
    return std::atan2(s, c);
}

float World::coneSwingAngle(JointId id) const {
    AccessGuard guard(*this, AccessKind::Query, "coneSwingAngle");
    const Joint& j = joint(id);
    if (j.type != JointType::ConeTwist)
        throw std::invalid_argument("velox: coneSwingAngle requires a cone/twist joint");
    Vec3 axisA = normalize(rotate(bodies_[j.a].orientation, j.localAxisA));
    Vec3 axisB = normalize(rotate(bodies_[j.b].orientation, j.localAxisB));
    return std::acos(vclamp(dot(axisA, axisB), -1.0f, 1.0f));
}

float World::coneTwistAngle(JointId id) const {
    AccessGuard guard(*this, AccessKind::Query, "coneTwistAngle");
    const Joint& j = joint(id);
    if (j.type != JointType::ConeTwist)
        throw std::invalid_argument("velox: coneTwistAngle requires a cone/twist joint");
    Vec3 axis = normalize(rotate(bodies_[j.a].orientation, j.localAxisA));
    Vec3 refA = rotate(bodies_[j.a].orientation, j.localRefA);
    Vec3 refB = rotate(bodies_[j.b].orientation, j.localRefB);
    refA = normalize(refA - axis * dot(refA, axis));
    refB = normalize(refB - axis * dot(refB, axis));
    if (lengthSq(refA) < 1e-12f || lengthSq(refB) < 1e-12f) return 0.0f;
    return std::atan2(dot(cross(refA, refB), axis), dot(refA, refB));
}

float World::prismaticTranslation(JointId id) const {
    AccessGuard guard(*this, AccessKind::Query, "prismaticTranslation");
    const Joint& j = joint(id);
    if (j.type != JointType::Prismatic)
        throw std::invalid_argument(
            "velox: prismaticTranslation requires a prismatic joint");
    const Body& a = bodies_[j.a];
    const Body& b = bodies_[j.b];
    Vec3 pa = a.position + rotate(a.orientation, j.localAnchorA);
    Vec3 pb = b.position + rotate(b.orientation, j.localAnchorB);
    Vec3 axis = normalize(rotate(a.orientation, j.localAxisA));
    return dot(pb - pa, axis);
}

Vec3 World::sixDofLinearTranslation(JointId id) const {
    AccessGuard guard(*this, AccessKind::Query, "sixDofLinearTranslation");
    const Joint& j = joint(id);
    if (j.type != JointType::SixDof)
        throw std::invalid_argument(
            "velox: sixDofLinearTranslation requires a 6DoF joint");
    const Body& a = bodies_[j.a];
    const Body& b = bodies_[j.b];
    Vec3 pa = a.position + rotate(a.orientation, j.localAnchorA);
    Vec3 pb = b.position + rotate(b.orientation, j.localAnchorB);
    Vec3 xA, yA, zA, xB, yB, zB;
    jointFrame(j, a, b, xA, yA, zA, xB, yB, zB);
    Vec3 d = pb - pa;
    return {dot(d, xA), dot(d, yA), dot(d, zA)};
}

Vec3 World::sixDofAngularRotation(JointId id) const {
    AccessGuard guard(*this, AccessKind::Query, "sixDofAngularRotation");
    const Joint& j = joint(id);
    if (j.type != JointType::SixDof)
        throw std::invalid_argument(
            "velox: sixDofAngularRotation requires a 6DoF joint");
    return sixDofRotationVector(j, bodies_[j.a], bodies_[j.b]);
}

void World::wake(BodyId id) {
    AccessGuard guard(*this, AccessKind::Mutation, "wake");
    Body& b = body(id);
    sleepManager_.wakeBody(b, id);
}

namespace {

// Tight bounds fed to the broad-phase tree: swept bounds for finite shapes
// (bodyAabb already includes the speculative reach), root bounds for meshes.
void proxyBounds(const Body& b, const MeshSoup& meshes, float dt,
                 Vec3& lo, Vec3& hi) {
    if (b.shape == ShapeType::Mesh) {
        const Mesh& mesh = meshes.meshes[b.meshIndex];
        lo = mesh.aabbMin;
        hi = mesh.aabbMax;
    } else {
        bodyAabb(b, dt, lo, hi);
    }
}

} // namespace

void World::ensureBroadPhase(float dt, bool refit) const {
    BroadPhaseData& bp = *broadPhase_;
    if (bp.structureDirty || bp.proxies.size() != bodies_.size()) {
        bp.tree.clear();
        bp.planes.clear();
        bp.proxies.assign(bodies_.size(), -1);
        for (BodyIndex i = 0; i < bodies_.size(); ++i) {
            const Body& b = bodies_[i];
            if (b.shape == ShapeType::Plane) {
                bp.planes.push_back(i);
                continue;
            }
            Vec3 lo, hi;
            proxyBounds(b, meshes_, dt, lo, hi);
            bp.proxies[i] = bp.tree.insert(lo, hi, i);
        }
        bp.structureDirty = false;
        bp.touched = false;
        bp.lastDt = dt;
        return;
    }
    if (!refit && !bp.touched) return;
    for (BodyIndex i = 0; i < bodies_.size(); ++i) {
        if (bp.proxies[i] < 0) continue;
        const Body& b = bodies_[i];
        if (b.isStatic() && !bp.touched) continue; // statics only move via API
        Vec3 lo, hi;
        proxyBounds(b, meshes_, dt, lo, hi);
        bp.tree.move(bp.proxies[i], lo, hi);
    }
    bp.touched = false;
    bp.lastDt = dt;
}

void World::buildCandidatePairs(float dt) {
    VELOX_PROFILE_CATEGORY_SCOPE("Broadphase", velox::profile::Category::Broadphase);
    ensureBroadPhase(dt, /*refit=*/true);
    const BroadPhaseData& bp = *broadPhase_;
    candidatePairs_.clear();

    auto inert = [&](BodyIndex k) {
        return bodies_[k].isStatic() || isFullyAsleep(bodies_[k].asleep);
    };
    // Capture the custom filter callback by reference for the parallel
    // section. The callback is read-only here; thread safety of the
    // callable itself is the user's responsibility.
    const CollisionFilterCallback& filterCb = collisionFilterCallback_;
    // Tree queries are read-only after the refit above, so awake bodies fan
    // out across the backend's worker pool (this was the dominant serial
    // cost of a large-scene CPU step: one tree walk per awake body). Chunk
    // results merge in chunk order; the final sort restores the canonical
    // deterministic ordering regardless of how the work was split.
    std::vector<std::vector<uint64_t>> chunkPairs;
    const size_t bodyCount = bodies_.size();
    size_t chunksUsed = 0;
    chunkPairs.resize(64);
    backend_->parallelChunks(bodyCount, 512,
        [&](size_t chunk, size_t begin, size_t end) {
            std::vector<uint64_t>& out = chunkPairs[chunk];
            out.clear();
            for (BodyIndex i = (BodyIndex)begin; i < end; ++i) {
                if (bp.proxies[i] < 0 || inert(i)) continue;
                const Body& a = bodies_[i];
                Vec3 lo, hi;
                proxyBounds(a, meshes_, dt, lo, hi);
                bp.tree.query(lo, hi, [&](uint32_t j) {
                    if (j == i) return;
                    // Active-active pairs are discovered from both endpoints:
                    // keep the lower index's discovery. Inert bodies never
                    // run a query, so their pairs survive unconditionally.
                    if (!inert(j) && j < i) return;
                    if (!evaluateCollisionFilterWithCallback(a, bodies_[j], filterCb)) return;
                    BodyIndex lo32 = i < j ? i : (BodyIndex)j;
                    BodyIndex hi32 = i < j ? (BodyIndex)j : i;
                    out.push_back((uint64_t)lo32 << 32 | hi32);
                });
                for (BodyIndex p : bp.planes)
                    if (evaluateCollisionFilterWithCallback(a, bodies_[p], filterCb))
                        out.push_back(i < p ? ((uint64_t)i << 32 | p)
                                            : ((uint64_t)p << 32 | i));
            }
        },
        &chunksUsed);
    for (size_t chunk = 0; chunk < chunksUsed && chunk < chunkPairs.size(); ++chunk)
        candidatePairs_.insert(candidatePairs_.end(), chunkPairs[chunk].begin(),
                               chunkPairs[chunk].end());
    // Deterministic order for the narrow phase and solver regardless of tree
    // shape history; unique guards against duplicate discoveries.
    std::sort(candidatePairs_.begin(), candidatePairs_.end());
    candidatePairs_.erase(
        std::unique(candidatePairs_.begin(), candidatePairs_.end()),
        candidatePairs_.end());
}

bool World::isAwake(BodyId id) const {
    AccessGuard guard(*this, AccessKind::Query, "isAwake");
    return bodies_[resolve(id)].asleep == 0;
}

SleepState World::sleepState(BodyId id) const {
    AccessGuard guard(*this, AccessKind::Query, "sleepState");
    return sleepStateFromByte(bodies_[resolve(id)].asleep);
}

void World::setContactModifier(ContactModifier modifier) {
    AccessGuard guard(*this, AccessKind::Mutation, "setContactModifier");
    contactModifier_ = std::move(modifier);
}

void World::setCollisionFilterCallback(CollisionFilterCallback callback) {
    AccessGuard guard(*this, AccessKind::Mutation, "setCollisionFilterCallback");
    collisionFilterCallback_ = std::move(callback);
    // Purge existing contacts so the new filter takes effect immediately.
    if (!contacts_.empty()) {
        contacts_.clear();
        broadPhase_->touched = true;
    }
}

void World::removeJointDense(uint32_t dense) {
    uint32_t last = static_cast<uint32_t>(joints_.size() - 1);
    uint32_t removedSlot = jointDenseToSlot_[dense];
    if (dense != last) {
        joints_[dense] = joints_[last];
        uint32_t movedSlot = jointDenseToSlot_[last];
        jointDenseToSlot_[dense] = movedSlot;
        jointSlots_[movedSlot].dense = dense;
    }
    joints_.pop_back();
    jointDenseToSlot_.pop_back();

    HandleSlot& slot = jointSlots_[removedSlot];
    slot.dense = UINT32_MAX;
    if (slot.generation != UINT32_MAX) {
        ++slot.generation;
        freeJointSlots_.push_back(removedSlot);
    }
}

void World::removeJoint(JointId id) {
    AccessGuard guard(*this, AccessKind::Mutation, "removeJoint");
    removeJointDense(resolve(id));
}

void World::removeBody(BodyId id) {
    AccessGuard guard(*this, AccessKind::Mutation, "removeBody");
    BodyIndex dense = resolve(id);
    pendingBodyEvents_.push_back({BodyEventType::Destroyed, id, bodies_[dense].position, bodies_[dense].orientation});
    broadPhase_->structureDirty = true; // dense indices reshuffle on removal

    // Removing an endpoint also removes every constraint that references it.
    for (uint32_t i = static_cast<uint32_t>(joints_.size()); i-- > 0;)
        if (joints_[i].a == dense || joints_[i].b == dense)
            removeJointDense(i);

    BodyIndex last = static_cast<BodyIndex>(bodies_.size() - 1);
    uint32_t removedSlot = bodyDenseToSlot_[dense];
    if (dense != last) {
        bodies_[dense] = bodies_[last];
        uint32_t movedSlot = bodyDenseToSlot_[last];
        bodyDenseToSlot_[dense] = movedSlot;
        bodySlots_[movedSlot].dense = dense;
        for (Joint& joint : joints_) {
            if (joint.a == last) joint.a = dense;
            if (joint.b == last) joint.b = dense;
        }
    }
    bodies_.pop_back();
    bodyDenseToSlot_.pop_back();

    HandleSlot& slot = bodySlots_[removedSlot];
    slot.dense = UINT32_MAX;
    if (slot.generation != UINT32_MAX) {
        ++slot.generation;
        freeBodySlots_.push_back(removedSlot);
    }

    // Dense indices changed, so no contact or sleeping cache may survive.
    contacts_.clear();
    prevContacts_.clear();
    pairKeys_.clear();
    prevPairKeys_.clear();
    events_.clear();
    prev_.clear();
    unionParent_.clear();
    islandTimer_.clear();
}

// ---------------------------------------------------------------------------
// Soft bodies
// ---------------------------------------------------------------------------

SoftBodyId World::addSoftBody(const SoftBodyDesc& desc) {
    AccessGuard guard(*this, AccessKind::Mutation, "addSoftBody");
    SoftBody sb;
    sb.positions = desc.positions;
    sb.prevPositions = desc.positions;
    sb.velocities.resize(desc.positions.size(), {});
    sb.invMasses = desc.invMasses;
    sb.constraints = desc.constraints;
    sb.gravityScale = desc.gravityScale;
    sb.linearDamping = desc.linearDamping;
    sb.solverIterations = desc.solverIterations;
    if (!sb.positions.empty()) {
        sb.aabbMin = sb.aabbMax = sb.positions[0];
        for (const Vec3& p : sb.positions) {
            sb.aabbMin = vmin(sb.aabbMin, p);
            sb.aabbMax = vmax(sb.aabbMax, p);
        }
    }

    uint32_t slot;
    if (!freeSoftBodySlots_.empty()) {
        slot = freeSoftBodySlots_.back();
        freeSoftBodySlots_.pop_back();
    } else {
        slot = static_cast<uint32_t>(softBodySlots_.size());
        softBodySlots_.push_back({});
    }
    uint32_t dense = static_cast<uint32_t>(softBodies_.size());
    softBodies_.push_back(std::move(sb));
    softBodyDenseToSlot_.push_back(slot);
    softBodySlots_[slot] = {dense, softBodySlots_[slot].generation};
    return SoftBodyId::make(slot, softBodySlots_[slot].generation);
}

SoftBody& World::softBody(SoftBodyId id) {
    AccessGuard guard(*this, AccessKind::Query, "softBody");
    uint32_t slot = id.slot();
    if (slot >= softBodySlots_.size() ||
        softBodySlots_[slot].generation != id.generation() ||
        softBodySlots_[slot].dense == UINT32_MAX)
        throw std::out_of_range("velox: invalid SoftBodyId");
    return softBodies_[softBodySlots_[slot].dense];
}

const SoftBody& World::softBody(SoftBodyId id) const {
    AccessGuard guard(*this, AccessKind::Query, "softBody");
    uint32_t slot = id.slot();
    if (slot >= softBodySlots_.size() ||
        softBodySlots_[slot].generation != id.generation() ||
        softBodySlots_[slot].dense == UINT32_MAX)
        throw std::out_of_range("velox: invalid SoftBodyId");
    return softBodies_[softBodySlots_[slot].dense];
}

void World::removeSoftBody(SoftBodyId id) {
    AccessGuard guard(*this, AccessKind::Mutation, "removeSoftBody");
    uint32_t slot = id.slot();
    if (slot >= softBodySlots_.size() ||
        softBodySlots_[slot].generation != id.generation() ||
        softBodySlots_[slot].dense == UINT32_MAX)
        throw std::out_of_range("velox: invalid SoftBodyId");
    uint32_t dense = softBodySlots_[slot].dense;
    uint32_t last = static_cast<uint32_t>(softBodies_.size() - 1);
    if (dense != last) {
        softBodies_[dense] = std::move(softBodies_[last]);
        uint32_t movedSlot = softBodyDenseToSlot_[last];
        softBodyDenseToSlot_[dense] = movedSlot;
        softBodySlots_[movedSlot].dense = dense;
    }
    softBodies_.pop_back();
    softBodyDenseToSlot_.pop_back();
    HandleSlot& hs = softBodySlots_[slot];
    hs.dense = UINT32_MAX;
    if (hs.generation != UINT32_MAX) {
        ++hs.generation;
        freeSoftBodySlots_.push_back(slot);
    }
}

size_t World::softBodyCount() const {
    AccessGuard guard(*this, AccessKind::Query, "softBodyCount");
    return softBodies_.size();
}

bool World::isValid(SoftBodyId id) const {
    AccessGuard guard(*this, AccessKind::Query, "isValid(SoftBodyId)");
    uint32_t slot = id.slot();
    return slot < softBodySlots_.size() &&
           softBodySlots_[slot].generation == id.generation() &&
           softBodySlots_[slot].dense != UINT32_MAX;
}

namespace {

// Minimal column-major 3x3 for joint effective-mass solves.
struct Mat3 {
    Vec3 c0, c1, c2;
};

Vec3 mul(const Mat3& m, const Vec3& v) {
    return m.c0 * v.x + m.c1 * v.y + m.c2 * v.z;
}

Mat3 inverse(const Mat3& m) {
    Vec3 r0 = cross(m.c1, m.c2);
    Vec3 r1 = cross(m.c2, m.c0);
    Vec3 r2 = cross(m.c0, m.c1);
    float det = dot(m.c0, r0);
    float inv = std::fabs(det) > 1e-12f ? 1.0f / det : 0.0f;
    // rows of the inverse are r0,r1,r2 scaled; transpose into columns.
    return {{r0.x * inv, r1.x * inv, r2.x * inv},
            {r0.y * inv, r1.y * inv, r2.y * inv},
            {r0.z * inv, r1.z * inv, r2.z * inv}};
}

// K(P): change of relative anchor velocity per unit impulse P at the anchors.
Mat3 pointMass(const Body& a, const Body& b, const Vec3& ra, const Vec3& rb) {
    auto col = [&](Vec3 e) {
        Vec3 k = e * (a.solverInvMass() + b.solverInvMass());
        k += cross(a.invInertiaMul(cross(ra, e)), ra);
        k += cross(b.invInertiaMul(cross(rb, e)), rb);
        return k;
    };
    return {col({1, 0, 0}), col({0, 1, 0}), col({0, 0, 1})};
}

Vec3 anchorVelocity(const Body& x, const Vec3& r) {
    return x.velocity + cross(x.angularVelocity, r);
}

Vec3 clampJointPositionBias(const Vec3& bias) {
    constexpr float kMaxPositionBias = 10.0f;
    float magnitudeSq = lengthSq(bias);
    if (magnitudeSq <= kMaxPositionBias * kMaxPositionBias) return bias;
    return bias * (kMaxPositionBias / std::sqrt(magnitudeSq));
}

void applyImpulse(Body& x, const Vec3& r, const Vec3& P, float sign) {
    if (!x.isDynamic()) return;
    x.velocity += P * (sign * x.solverInvMass());
    x.angularVelocity += x.invInertiaMul(cross(r, P)) * sign;
}

void applyJointImpulse(Joint& joint, Body& a, Body& b,
                       const Vec3& ra, const Vec3& rb,
                       const Vec3& impulse, float signA) {
    applyImpulse(a, ra, impulse, signA);
    applyImpulse(b, rb, impulse, -signA);
    joint.reactionLinearImpulse += impulse * signA;
}

void applyJointAngularImpulse(Joint& joint, Body& a, Body& b,
                              const Vec3& impulse, float signA = 1.0f) {
    if (a.isDynamic()) a.angularVelocity += a.invInertiaMul(impulse) * signA;
    if (b.isDynamic()) b.angularVelocity -= b.invInertiaMul(impulse) * signA;
    joint.reactionAngularImpulse += impulse * signA;
}

Mat3 angularMass(const Body& a, const Body& b) {
    auto col = [&](Vec3 e) { return a.invInertiaMul(e) + b.invInertiaMul(e); };
    return {col({1, 0, 0}), col({0, 1, 0}), col({0, 0, 1})};
}

void solveAngularFrame(Joint& j, Body& a, Body& b, float biasScale) {
    Vec3 xA = normalize(rotate(a.orientation, j.localAxisA));
    Vec3 xB = normalize(rotate(b.orientation, j.localAxisB));
    Vec3 yA = normalize(rotate(a.orientation, j.localRefA));
    Vec3 yB = normalize(rotate(b.orientation, j.localRefB));
    Vec3 zA = cross(xA, yA);
    Vec3 zB = cross(xB, yB);
    Vec3 error = (cross(xB, xA) + cross(yB, yA) + cross(zB, zA)) * 0.5f;
    Vec3 relativeW = a.angularVelocity - b.angularVelocity;
    Vec3 impulse = mul(inverse(angularMass(a, b)),
                       -(relativeW + error * biasScale));
    applyJointAngularImpulse(j, a, b, impulse);
}

// Signed gap and contact normal (from B towards A) between two bodies with
// their transforms overridden — the distance oracle for conservative
// advancement.
struct GapProbe { float gap; Vec3 normal; };

GapProbe gapAt(const Body& a, const Vec3& pa, const Quat& qa,
               const Body& b, const Vec3& pb, const Quat& qb,
               const MeshSoupView& soup, float searchRadius) {
    Body ta = a; ta.position = pa; ta.orientation = qa;
    Body tb = b; tb.position = pb; tb.orientation = qb;

    if (ta.shape == ShapeType::Compound) {
        GapProbe best{1e30f, {0, 1, 0}};
        for (uint32_t i = 0; i < ta.compoundCount; ++i) {
            Body child = compoundChildBody(
                ta, soup.compoundChildren[ta.compoundFirst + i]);
            GapProbe gap = gapAt(child, child.position, child.orientation,
                                 tb, tb.position, tb.orientation, soup, searchRadius);
            if (gap.gap < best.gap) best = gap;
        }
        return best;
    }
    if (tb.shape == ShapeType::Compound) {
        GapProbe best{1e30f, {0, 1, 0}};
        for (uint32_t i = 0; i < tb.compoundCount; ++i) {
            Body child = compoundChildBody(
                tb, soup.compoundChildren[tb.compoundFirst + i]);
            GapProbe gap = gapAt(ta, ta.position, ta.orientation,
                                 child, child.position, child.orientation,
                                 soup, searchRadius);
            if (gap.gap < best.gap) best = gap;
        }
        return best;
    }

    if (tb.shape == ShapeType::Plane) {
        Convex c = makeConvex(ta, soup);
        Vec3 deep = c.support(-tb.planeNormal);
        return {dot(tb.planeNormal, deep) - tb.planeOffset - c.radius, tb.planeNormal};
    }
    if (tb.shape == ShapeType::Mesh) {
        GapProbe best{1e30f, {0, 1, 0}};
        meshGapProbe(ta, tb, soup, searchRadius, best.gap, best.normal);
        return best;
    }
    GjkResult r = gjkDistance(makeConvex(ta, soup), makeConvex(tb, soup));
    return {r.distance, r.normal};
}

Body sweepStateAt(const Body& source, float elapsed) {
    Body result = source;
    result.advanceTransform(elapsed);
    return result;
}

bool findToi(const Body& a0, const Body& b0, float dt, float bias,
             const MeshSoupView& soup, float& toi, Vec3& normal, Vec3& point) {
    const float speedBound = a0.maxPointSpeed() + b0.maxPointSpeed();
    if (speedBound <= 1e-8f) return false;
    const float search = speedBound * dt + a0.radius + length(a0.halfExtents) +
                         a0.capsuleHalfHeight + 0.1f;
    float lower = 0.0f;
    float upper = 0.0f;
    GapProbe sample = gapAt(a0, a0.position, a0.orientation,
                            b0, b0.position, b0.orientation, soup, search);
    if (sample.gap <= bias) {
        toi = 0.0f;
        normal = sample.normal;
    } else {
        bool hit = false;
        for (int iteration = 0; iteration < 24 && lower < dt; ++iteration) {
            const float increment = std::max(1e-6f, (sample.gap - bias) / speedBound);
            upper = std::min(dt, lower + increment);
            Body a = sweepStateAt(a0, upper);
            Body b = sweepStateAt(b0, upper);
            sample = gapAt(a, a.position, a.orientation,
                           b, b.position, b.orientation, soup, search);
            if (sample.gap <= bias) {
                hit = true;
                break;
            }
            lower = upper;
            if (upper >= dt) break;
        }
        if (!hit) return false;
        for (int iteration = 0; iteration < 16; ++iteration) {
            const float middle = (lower + upper) * 0.5f;
            Body a = sweepStateAt(a0, middle);
            Body b = sweepStateAt(b0, middle);
            const GapProbe middleGap = gapAt(a, a.position, a.orientation,
                                              b, b.position, b.orientation,
                                              soup, search);
            if (middleGap.gap <= bias) {
                upper = middle;
                sample = middleGap;
            } else {
                lower = middle;
            }
        }
        toi = upper;
        normal = sample.normal;
    }
    Body a = sweepStateAt(a0, toi);
    Convex convex = makeConvex(a, soup);
    point = convex.support(-normal) - normal * convex.radius;
    return true;
}

uint32_t processHighToi(std::vector<Body>& bodies, const std::vector<Body>& starts,
                        const std::vector<Contact>& contacts, float dt,
                        const MeshSoupView& soup,
                        const WorldMultiToiSettings& settings,
                        const CollisionFilterCallback& filterCallback,
                        std::vector<uint8_t>& processedBodies) {
    processedBodies.assign(bodies.size(), 0);
    if (!settings.defaultConfig.enabled || !settings.enableSubstepSplitting)
        return 0;
    std::vector<uint8_t> eligible(bodies.size(), 0);
    for (BodyIndex i = 0; i < bodies.size(); ++i) {
        const Body& start = starts[i];
        const BodyCcdTuning& tuning = start.ccdTuning;
        eligible[i] = start.isDynamic() && !isFullyAsleep(bodies[i].asleep) &&
                      tuning.quality == MotionQuality::High &&
                      tuning.enableContinuous &&
                      start.maxPointSpeed() >= std::max(
                          tuning.minVelocityForCCD,
                          settings.defaultConfig.toiVelocityFloor);
    }
    // A medium-quality moving counterpart was already advanced by the normal
    // solver. Do not rewind just one side of that interaction; it retains the
    // regular PCS path until both bodies opt into the event scheduler.
    for (const Contact& contact : contacts) {
        const BodyIndex a = contact.a, b = contact.b;
        if (eligible[a] && bodies[b].isDynamic() && !eligible[b]) eligible[a] = 0;
        if (eligible[b] && bodies[a].isDynamic() && !eligible[a]) eligible[b] = 0;
    }
    bool anyEligible = false;
    for (BodyIndex i = 0; i < bodies.size(); ++i) {
        if (!eligible[i]) continue;
        Body& body = bodies[i];
        const Body& start = starts[i];
        body.position = start.position;
        body.orientation = start.orientation;
        body.velocity = start.velocity;
        body.angularVelocity = start.angularVelocity;
        anyEligible = true;
    }
    if (!anyEligible) return 0;
    processedBodies = eligible;

    uint32_t totalEvents = 0;
    std::vector<uint32_t> bodyEvents(bodies.size(), 0);
    float remaining = dt;
    BodyIndex previousA = UINT32_MAX, previousB = UINT32_MAX;
    while (remaining > 1e-6f && totalEvents < settings.maxTotalEventsPerStep) {
        float bestToi = remaining + 1.0f;
        BodyIndex bestA = UINT32_MAX, bestB = UINT32_MAX;
        Vec3 bestNormal{}, bestPoint{};
        for (BodyIndex a = 0; a < bodies.size(); ++a) {
            if (!eligible[a] || bodyEvents[a] >= settings.defaultConfig.maxToiEventsPerBody)
                continue;
            for (BodyIndex b = 0; b < bodies.size(); ++b) {
                if (a == b) continue;
                const bool dynamicPair = eligible[b];
                if (dynamicPair && b < a) continue;
                if (!dynamicPair && !bodies[b].isStatic()) continue;
                if (bodies[b].isSensor() || !evaluateCollisionFilterWithCallback(bodies[a], bodies[b], filterCallback)) continue;
                if (dynamicPair && bodyEvents[b] >= settings.defaultConfig.maxToiEventsPerBody)
                    continue;
                float toi = 0.0f;
                Vec3 normal{}, point{};
                if (!findToi(bodies[a], bodies[b], remaining,
                             settings.defaultConfig.toiPenetrationBias,
                             soup, toi, normal, point))
                    continue;
                if (a == previousA && b == previousB && toi <= 1e-6f)
                    continue;
                if (toi < bestToi ||
                    (toi == bestToi && (a < bestA || (a == bestA && b < bestB)))) {
                    bestToi = toi;
                    bestA = a;
                    bestB = b;
                    bestNormal = normal;
                    bestPoint = point;
                }
            }
        }
        if (bestA == UINT32_MAX) break;

        for (BodyIndex i = 0; i < bodies.size(); ++i)
            if (eligible[i]) bodies[i].advanceTransform(bestToi);
        Body& a = bodies[bestA];
        Body& b = bodies[bestB];
        const Vec3 ra = bestPoint - a.position;
        const Vec3 rb = bestPoint - b.position;
        const float vn = dot(a.velocity + cross(a.angularVelocity, ra) -
                             b.velocity - cross(b.angularVelocity, rb), bestNormal);
        const float angularMassA = dot(cross(a.invInertiaMul(cross(ra, bestNormal)), ra),
                                       bestNormal);
        const float angularMassB = dot(cross(b.invInertiaMul(cross(rb, bestNormal)), rb),
                                       bestNormal);
        const float mass = a.solverInvMass() + b.solverInvMass() + angularMassA + angularMassB;
        if (vn < 0.0f && mass > 1e-8f) {
            constexpr float kRestitutionThreshold = 1.0f;
            const float restitution = vn < -kRestitutionThreshold
                ? combineMaterial(a.restitution, b.restitution,
                                  a.restitutionCombine, b.restitutionCombine)
                : 0.0f;
            const Vec3 impulse = bestNormal * (-(1.0f + restitution) * vn / mass);
            a.velocity += impulse * a.solverInvMass();
            a.angularVelocity += a.invInertiaMul(cross(ra, impulse));
            b.velocity -= impulse * b.solverInvMass();
            b.angularVelocity -= b.invInertiaMul(cross(rb, impulse));
        }
        const float correction = std::max(2.0f * settings.defaultConfig.toiPenetrationBias,
                                          1e-4f);
        const float inverseMass = a.solverInvMass() + b.solverInvMass();
        if (inverseMass > 0.0f) {
            a.position += bestNormal * (correction * a.solverInvMass() / inverseMass);
            b.position -= bestNormal * (correction * b.solverInvMass() / inverseMass);
        }
        remaining -= bestToi;
        previousA = bestA;
        previousB = bestB;
        ++bodyEvents[bestA];
        if (eligible[bestB]) ++bodyEvents[bestB];
        ++totalEvents;
    }
    for (BodyIndex i = 0; i < bodies.size(); ++i)
        if (eligible[i]) bodies[i].advanceTransform(remaining);
    return totalEvents;
}

} // namespace

std::vector<MultiToiHit> World::queryMultiToi(BodyId id, float dt) const {
    AccessGuard guard(*this, AccessKind::Query, "queryMultiToi");
    if (!finiteFloat(dt) || dt <= 0.0f)
        throw std::invalid_argument("velox: multi-TOI timestep must be finite and positive");
    const BodyIndex sourceIndex = resolve(id);
    const Body& source = bodies_[sourceIndex];
    const BodyCcdTuning& tuning = source.ccdTuning;
    if (!source.isDynamic() || source.isLocked() || !tuning.enableContinuous ||
        tuning.quality == MotionQuality::Low ||
        source.maxPointSpeed() < std::max(tuning.minVelocityForCCD,
                                          multiToiSettings_.defaultConfig.toiVelocityFloor))
        return {};

    const MeshSoupView soup = view(meshes_);
    std::vector<MultiToiHit> hits;
    hits.reserve(std::min<size_t>(bodies_.size(),
                                  multiToiSettings_.defaultConfig.maxToiEventsPerBody));
    for (BodyIndex otherIndex = 0; otherIndex < bodies_.size(); ++otherIndex) {
        if (otherIndex == sourceIndex) continue;
        const Body& other = bodies_[otherIndex];
        if (!evaluateCollisionFilterWithCallback(source, other, collisionFilterCallback_) || (other.isStatic() && source.isStatic()))
            continue;
        float toi = 0.0f;
        Vec3 normal{}, point{};
        if (!findToi(source, other, dt,
                     multiToiSettings_.defaultConfig.toiPenetrationBias,
                     soup, toi, normal, point))
            continue;
        hits.push_back({toi, bodyHandle(otherIndex), point, normal, toi / dt});
    }
    std::sort(hits.begin(), hits.end(), [](const MultiToiHit& lhs,
                                           const MultiToiHit& rhs) {
        if (lhs.toi != rhs.toi) return lhs.toi < rhs.toi;
        return lhs.body.value < rhs.body.value;
    });
    const size_t cap = std::min<size_t>(multiToiSettings_.defaultConfig.maxToiEventsPerBody,
                                        multiToiSettings_.maxTotalEventsPerStep);
    if (hits.size() > cap) hits.resize(cap);
    return hits;
}

// Iterative impulse solve for all joints, with Baumgarte positional bias.
// Joints are few compared to contacts, so this runs on the CPU.
void World::solveJoints(float dt) {
    if (joints_.empty()) return;
    constexpr int kJointIterations = 8;
    constexpr float kBeta = 0.2f; // positional correction per step fraction

    for (const Joint& j : joints_) {
        // A sleeping body attached to an awake one must participate.
        if (bodies_[j.a].asleep != bodies_[j.b].asleep) {
            bodies_[j.a].asleep = bodies_[j.b].asleep = 0;
            bodies_[j.a].sleepTimer = bodies_[j.b].sleepTimer = 0.0f;
        }
    }

    for (Joint& j : joints_) {
        j.motorImpulse = 0.0f;
        j.limitImpulse = 0.0f;
        j.swingImpulse = 0.0f;
        j.twistImpulse = 0.0f;
        j.springImpulse = 0.0f;
        j.linearMotorImpulse = {};
        j.angularMotorImpulse = {};
        j.linearLimitImpulse = {};
        j.angularLimitImpulse = {};
        j.reactionLinearImpulse = {};
        j.reactionAngularImpulse = {};
        j.broken = false;
    }

    for (int iter = 0; iter < kJointIterations; ++iter) {
        for (Joint& j : joints_) {
            Body& a = bodies_[j.a];
            Body& b = bodies_[j.b];
            if (isFullyAsleep(a.asleep) && isFullyAsleep(b.asleep)) continue;
            Vec3 ra = rotate(a.orientation, j.localAnchorA);
            Vec3 rb = rotate(b.orientation, j.localAnchorB);
            Vec3 pa = a.position + ra, pb = b.position + rb;
            Vec3 vel = anchorVelocity(a, ra) - anchorVelocity(b, rb);

            switch (j.type) {
            case JointType::Ball: {
                Vec3 bias = (pa - pb) * (kBeta / dt);
                Vec3 P = mul(inverse(pointMass(a, b, ra, rb)), -(vel + bias));
                applyJointImpulse(j, a, b, ra, rb, P, +1.0f);
                break;
            }
            case JointType::Distance: {
                Vec3 d = pa - pb;
                float len = length(d);
                if (len < 1e-8f) break;
                Vec3 n = d * (1.0f / len);
                Vec3 raxn = cross(ra, n), rbxn = cross(rb, n);
                float k = a.solverInvMass() + b.solverInvMass() +
                          dot(raxn, a.invInertiaMul(raxn)) +
                          dot(rbxn, b.invInertiaMul(rbxn));
                if (k <= 0.0f) break;
                float lambda;
                if (j.enableSpring) {
                    constexpr float kTau = 6.28318530718f;
                    float effectiveMass = 1.0f / k;
                    float omega = kTau * j.springFrequencyHz;
                    float stiffness = effectiveMass * omega * omega;
                    float damping = 2.0f * effectiveMass *
                                    j.springDampingRatio * omega;
                    float softness = 1.0f /
                        (dt * (damping + dt * stiffness));
                    float bias = (len - j.restLength) * dt * stiffness * softness;
                    lambda = -(dot(vel, n) + bias +
                               softness * j.springImpulse) / (k + softness);
                    j.springImpulse += lambda;
                } else {
                    float bias = (len - j.restLength) * (kBeta / dt);
                    lambda = -(dot(vel, n) + bias) / k;
                }
                applyJointImpulse(j, a, b, ra, rb, n * lambda, +1.0f);
                break;
            }
            case JointType::Fixed: {
                Vec3 bias = (pa - pb) * (kBeta / dt);
                Vec3 P = mul(inverse(pointMass(a, b, ra, rb)), -(vel + bias));
                applyJointImpulse(j, a, b, ra, rb, P, +1.0f);
                solveAngularFrame(j, a, b, kBeta / dt);
                break;
            }
            case JointType::Prismatic: {
                Vec3 axis = normalize(rotate(a.orientation, j.localAxisA));
                Vec3 ref = rotate(a.orientation, j.localRefA);
                Vec3 t1 = normalize(ref - axis * dot(ref, axis));
                if (lengthSq(t1) < 1e-12f) {
                    ref = std::fabs(axis.x) < 0.9f ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
                    t1 = normalize(cross(axis, ref));
                }
                Vec3 t2 = cross(axis, t1);
                Mat3 pointK = pointMass(a, b, ra, rb);
                Vec3 error = pa - pb;
                for (Vec3 t : {t1, t2}) {
                    float k = dot(t, mul(pointK, t));
                    if (k <= 0.0f) continue;
                    float lambda = -(dot(vel, t) + dot(error, t) * (kBeta / dt)) / k;
                    applyJointImpulse(j, a, b, ra, rb, t * lambda, +1.0f);
                    vel = anchorVelocity(a, ra) - anchorVelocity(b, rb);
                }
                solveAngularFrame(j, a, b, kBeta / dt);

                float kAxis = dot(axis, mul(pointK, axis));
                if (kAxis > 0.0f) {
                    float axisSpeed = dot(anchorVelocity(b, rb) -
                                          anchorVelocity(a, ra), axis);
                    if (j.enableMotor) {
                        float lambda = (j.motorSpeed - axisSpeed) / kAxis;
                        float maxImpulse = j.maxMotorForce * dt;
                        float oldImpulse = j.motorImpulse;
                        j.motorImpulse = vclamp(oldImpulse + lambda,
                                                -maxImpulse, maxImpulse);
                        lambda = j.motorImpulse - oldImpulse;
                        applyJointImpulse(j, a, b, ra, rb,
                                          axis * lambda, -1.0f);
                        axisSpeed = dot(anchorVelocity(b, rb) -
                                        anchorVelocity(a, ra), axis);
                    }
                    if (j.enableLimit) {
                        float translation = dot(pb - pa, axis);
                        float lambda = 0.0f;
                        float oldImpulse = j.limitImpulse;
                        if (translation < j.lowerLimit) {
                            float bias = (translation - j.lowerLimit) * (kBeta / dt);
                            lambda = -(axisSpeed + bias) / kAxis;
                            j.limitImpulse = vmax(0.0f, oldImpulse + lambda);
                            lambda = j.limitImpulse - oldImpulse;
                        } else if (translation > j.upperLimit) {
                            float bias = (translation - j.upperLimit) * (kBeta / dt);
                            lambda = -(axisSpeed + bias) / kAxis;
                            j.limitImpulse = vmin(0.0f, oldImpulse + lambda);
                            lambda = j.limitImpulse - oldImpulse;
                        }
                        applyJointImpulse(j, a, b, ra, rb,
                                          axis * lambda, -1.0f);
                    }
                }
                break;
            }
            case JointType::SixDof: {
                Vec3 xA, yA, zA, xB, yB, zB;
                jointFrame(j, a, b, xA, yA, zA, xB, yB, zB);
                Vec3 axes[3] = {xA, yA, zA};
                Vec3 d = pb - pa;
                Mat3 pointK = pointMass(a, b, ra, rb);

                for (int axisIndex = 0; axisIndex < 3; ++axisIndex) {
                    uint8_t bit = uint8_t(1u << axisIndex);
                    Vec3 axis = axes[axisIndex];
                    float k = dot(axis, mul(pointK, axis));
                    if (k <= 0.0f) continue;

                    float speed = dot(anchorVelocity(b, rb) -
                                      anchorVelocity(a, ra), axis);
                    if ((j.linearMotorMask & bit) != 0) {
                        float lambda = (component(j.linearMotorSpeed, axisIndex) -
                                        speed) / k;
                        float maxImpulse = component(j.maxLinearMotorForce,
                                                     axisIndex) * dt;
                        float& accumulated = component(j.linearMotorImpulse,
                                                       axisIndex);
                        float oldImpulse = accumulated;
                        accumulated = vclamp(oldImpulse + lambda,
                                             -maxImpulse, maxImpulse);
                        lambda = accumulated - oldImpulse;
                        applyJointImpulse(j, a, b, ra, rb, axis * lambda,
                                          -1.0f);
                        speed = dot(anchorVelocity(b, rb) -
                                    anchorVelocity(a, ra), axis);
                    }

                    if ((j.linearLimitMask & bit) == 0) continue;
                    float position = dot(d, axis);
                    float lower = component(j.lowerLinearLimit, axisIndex);
                    float upper = component(j.upperLinearLimit, axisIndex);
                    float lambda = 0.0f;
                    float& accumulated = component(j.linearLimitImpulse,
                                                   axisIndex);
                    float oldImpulse = accumulated;
                    if (lower == upper) {
                        float bias = (position - lower) * (kBeta / dt);
                        lambda = -(speed + bias) / k;
                        accumulated += lambda;
                    } else if (position < lower) {
                        float bias = (position - lower) * (kBeta / dt);
                        lambda = -(speed + bias) / k;
                        accumulated = vmax(0.0f, oldImpulse + lambda);
                        lambda = accumulated - oldImpulse;
                    } else if (position > upper) {
                        float bias = (position - upper) * (kBeta / dt);
                        lambda = -(speed + bias) / k;
                        accumulated = vmin(0.0f, oldImpulse + lambda);
                        lambda = accumulated - oldImpulse;
                    }
                    if (lambda != 0.0f)
                        applyJointImpulse(j, a, b, ra, rb, axis * lambda,
                                          -1.0f);
                }

                Vec3 rotation = sixDofRotationVector(j, a, b);
                for (int axisIndex = 0; axisIndex < 3; ++axisIndex) {
                    uint8_t bit = uint8_t(1u << axisIndex);
                    Vec3 axis = axes[axisIndex];
                    float k = dot(axis, a.invInertiaMul(axis)) +
                              dot(axis, b.invInertiaMul(axis));
                    if (k <= 0.0f) continue;

                    float speed = dot(b.angularVelocity - a.angularVelocity,
                                      axis);
                    if ((j.angularMotorMask & bit) != 0) {
                        float lambda = (speed -
                                        component(j.angularMotorSpeed,
                                                  axisIndex)) / k;
                        float maxImpulse = component(j.maxAngularMotorTorque,
                                                     axisIndex) * dt;
                        float& accumulated = component(j.angularMotorImpulse,
                                                       axisIndex);
                        float oldImpulse = accumulated;
                        accumulated = vclamp(oldImpulse + lambda,
                                             -maxImpulse, maxImpulse);
                        lambda = accumulated - oldImpulse;
                        applyJointAngularImpulse(j, a, b, axis * lambda);
                        speed = dot(b.angularVelocity - a.angularVelocity,
                                    axis);
                    }

                    if ((j.angularLimitMask & bit) == 0) continue;
                    float angle = component(rotation, axisIndex);
                    float lower = component(j.lowerAngularLimit, axisIndex);
                    float upper = component(j.upperAngularLimit, axisIndex);
                    float lambda = 0.0f;
                    float& accumulated = component(j.angularLimitImpulse,
                                                   axisIndex);
                    float oldImpulse = accumulated;
                    if (lower == upper) {
                        float bias = (angle - lower) * (kBeta / dt);
                        lambda = (speed + bias) / k;
                        accumulated += lambda;
                    } else if (angle < lower) {
                        float bias = (angle - lower) * (kBeta / dt);
                        lambda = (speed + bias) / k;
                        accumulated = vmin(0.0f, oldImpulse + lambda);
                        lambda = accumulated - oldImpulse;
                    } else if (angle > upper) {
                        float bias = (angle - upper) * (kBeta / dt);
                        lambda = (speed + bias) / k;
                        accumulated = vmax(0.0f, oldImpulse + lambda);
                        lambda = accumulated - oldImpulse;
                    }
                    if (lambda != 0.0f)
                        applyJointAngularImpulse(j, a, b, axis * lambda);
                }
                break;
            }
            case JointType::Hinge: {
                // Point constraint...
                Vec3 bias = clampJointPositionBias((pa - pb) * (kBeta / dt));
                Vec3 P = mul(inverse(pointMass(a, b, ra, rb)), -(vel + bias));
                applyJointImpulse(j, a, b, ra, rb, P, +1.0f);
                // ...plus two angular rows keeping the axes aligned.
                Vec3 axisA = rotate(a.orientation, j.localAxisA);
                Vec3 axisB = rotate(b.orientation, j.localAxisB);
                Vec3 err = cross(axisB, axisA); // rotate B by err to realign
                Vec3 ref = std::fabs(axisA.x) < 0.9f ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
                Vec3 b1 = normalize(cross(axisA, ref));
                Vec3 b2 = cross(axisA, b1);
                Vec3 wr = a.angularVelocity - b.angularVelocity;
                for (Vec3 t : {b1, b2}) {
                    float k = dot(t, a.invInertiaMul(t)) + dot(t, b.invInertiaMul(t));
                    if (k <= 0.0f) continue;
                    // err = cross(axisB, axisA): B must gain +err spin to
                    // realign, i.e. the relative velocity wr = wa - wb must
                    // be driven towards -err * beta/dt.
                    float lambda = -(dot(wr, t) + dot(err, t) * (kBeta / dt)) / k;
                    applyJointAngularImpulse(j, a, b, t * lambda);
                    wr = a.angularVelocity - b.angularVelocity;
                }

                // Motor and limit act on the rotation about the hinge axis.
                float kAxis = dot(axisA, a.invInertiaMul(axisA)) +
                              dot(axisA, b.invInertiaMul(axisA));
                if (kAxis > 0.0f) {
                    if (j.enableMotor) {
                        float wAxis = dot(a.angularVelocity - b.angularVelocity, axisA);
                        float lambda = (j.motorSpeed - wAxis) / kAxis;
                        float maxImpulse = j.maxMotorTorque * dt;
                        float oldImpulse = j.motorImpulse;
                        j.motorImpulse = vclamp(oldImpulse + lambda, -maxImpulse, maxImpulse);
                        lambda = j.motorImpulse - oldImpulse;
                        applyJointAngularImpulse(j, a, b, axisA * lambda);
                    }
                    if (j.enableLimit) {
                        // Current angle from the stored references.
                        Vec3 refA = rotate(a.orientation, j.localRefA);
                        Vec3 refB = rotate(b.orientation, j.localRefB);
                        float angle = std::atan2(dot(cross(refA, refB), axisA), dot(refA, refB));
                        float wAxis = dot(a.angularVelocity - b.angularVelocity, axisA);
                        // angle measures B relative... refA fixed on A: angle
                        // grows when B rotates +axis relative to A, i.e. when
                        // wAxis (wa - wb) is negative.
                        float lambda = 0.0f, oldImpulse = j.limitImpulse;
                        if (angle < j.lowerLimit) {
                            // angleDot = -wAxis. At the lower limit, only a
                            // negative axis impulse is allowed.
                            float target = (angle - j.lowerLimit) * (kBeta / dt);
                            lambda = (target - wAxis) / kAxis;
                            j.limitImpulse = vmin(0.0f, oldImpulse + lambda);
                            lambda = j.limitImpulse - oldImpulse;
                        } else if (angle > j.upperLimit) {
                            // At the upper limit, only a positive axis impulse
                            // is allowed. The bias drives the angle back in.
                            float target = (angle - j.upperLimit) * (kBeta / dt);
                            lambda = (target - wAxis) / kAxis;
                            j.limitImpulse = vmax(0.0f, oldImpulse + lambda);
                            lambda = j.limitImpulse - oldImpulse;
                        }
                        if (lambda != 0.0f) {
                            applyJointAngularImpulse(j, a, b, axisA * lambda);
                        }
                    }
                }
                break;
            }
            case JointType::ConeTwist: {
                // Ball anchor at the joint center.
                Vec3 bias = (pa - pb) * (kBeta / dt);
                Vec3 P = mul(inverse(pointMass(a, b, ra, rb)), -(vel + bias));
                applyJointImpulse(j, a, b, ra, rb, P, +1.0f);

                Vec3 axisA = normalize(rotate(a.orientation, j.localAxisA));
                Vec3 axisB = normalize(rotate(b.orientation, j.localAxisB));
                Vec3 wr = a.angularVelocity - b.angularVelocity;

                if (j.enableSwingLimit) {
                    float angle = std::acos(vclamp(dot(axisA, axisB), -1.0f, 1.0f));
                    if (angle > j.swingLimit) {
                        Vec3 n = cross(axisB, axisA);
                        if (lengthSq(n) < 1e-10f) {
                            Vec3 refA = rotate(a.orientation, j.localRefA);
                            n = cross(axisA, refA);
                        }
                        n = normalize(n);
                        float k = dot(n, a.invInertiaMul(n)) + dot(n, b.invInertiaMul(n));
                        if (k > 0.0f) {
                            float error = angle - j.swingLimit;
                            float lambda = -(dot(wr, n) + error * (kBeta / dt)) / k;
                            float oldImpulse = j.swingImpulse;
                            j.swingImpulse = vmin(0.0f, oldImpulse + lambda);
                            lambda = j.swingImpulse - oldImpulse;
                            applyJointAngularImpulse(j, a, b, n * lambda);
                            wr = a.angularVelocity - b.angularVelocity;
                        }
                    }
                }

                if (j.enableTwistLimit) {
                    Vec3 refA = rotate(a.orientation, j.localRefA);
                    Vec3 refB = rotate(b.orientation, j.localRefB);
                    refA = normalize(refA - axisA * dot(refA, axisA));
                    refB = normalize(refB - axisA * dot(refB, axisA));
                    if (lengthSq(refA) < 1e-10f || lengthSq(refB) < 1e-10f) break;
                    float angle = std::atan2(dot(cross(refA, refB), axisA),
                                             dot(refA, refB));
                    float k = dot(axisA, a.invInertiaMul(axisA)) +
                              dot(axisA, b.invInertiaMul(axisA));
                    if (k > 0.0f) {
                        float wAxis = dot(a.angularVelocity - b.angularVelocity, axisA);
                        float lambda = 0.0f;
                        float oldImpulse = j.twistImpulse;
                        if (angle < j.lowerTwistLimit) {
                            float target = (angle - j.lowerTwistLimit) * (kBeta / dt);
                            lambda = (target - wAxis) / k;
                            j.twistImpulse = vmin(0.0f, oldImpulse + lambda);
                            lambda = j.twistImpulse - oldImpulse;
                        } else if (angle > j.upperTwistLimit) {
                            float target = (angle - j.upperTwistLimit) * (kBeta / dt);
                            lambda = (target - wAxis) / k;
                            j.twistImpulse = vmax(0.0f, oldImpulse + lambda);
                            lambda = j.twistImpulse - oldImpulse;
                        }
                        if (lambda != 0.0f) {
                            applyJointAngularImpulse(j, a, b, axisA * lambda);
                        }
                    }
                }
                break;
            }
            case JointType::Motor: {
                Vec3 bias = (pa - pb) * (kBeta / dt);
                Vec3 P = mul(inverse(pointMass(a, b, ra, rb)), -(vel + bias));
                float maxImpulse = j.maxMotorForce * dt;
                float mag = length(P);
                if (mag > maxImpulse && mag > 1e-9f)
                    P = P * (maxImpulse / mag);
                applyJointImpulse(j, a, b, ra, rb, P, +1.0f);
                Vec3 angVel = a.angularVelocity - b.angularVelocity;
                float maxAngImpulse = j.maxMotorTorque * dt;
                float angMag = length(angVel);
                if (angMag > maxAngImpulse && angMag > 1e-9f)
                    angVel = angVel * (maxAngImpulse / angMag);
                applyJointAngularImpulse(j, a, b, -angVel);
                break;
            }
            case JointType::Weld: {
                // Linear: lock anchor points together.
                Vec3 bias = clampJointPositionBias((pa - pb) * (kBeta / dt));
                Vec3 P = mul(inverse(pointMass(a, b, ra, rb)), -(vel + bias));
                applyJointImpulse(j, a, b, ra, rb, P, +1.0f);
                // Angular: lock relative orientation.
                solveAngularFrame(j, a, b, kBeta / dt);
                break;
            }
            case JointType::Wheel: {
                // Constrain lateral motion (perpendicular to suspension axis).
                Vec3 axisA = normalize(rotate(a.orientation, j.localAxisA));
                Vec3 ref = rotate(a.orientation, j.localRefA);
                Vec3 t1 = normalize(ref - axisA * dot(ref, axisA));
                if (lengthSq(t1) < 1e-12f) {
                    ref = std::fabs(axisA.x) < 0.9f ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
                    t1 = normalize(cross(axisA, ref));
                }
                Vec3 t2 = cross(axisA, t1);
                Mat3 pointK = pointMass(a, b, ra, rb);
                Vec3 error = pa - pb;
                for (Vec3 t : {t1, t2}) {
                    float k = dot(t, mul(pointK, t));
                    if (k <= 0.0f) continue;
                    float lambda = -(dot(vel, t) + dot(error, t) * (kBeta / dt)) / k;
                    applyJointImpulse(j, a, b, ra, rb, t * lambda, +1.0f);
                    vel = anchorVelocity(a, ra) - anchorVelocity(b, rb);
                }
                // Suspension spring along the axis.
                float kAxis = dot(axisA, mul(pointK, axisA));
                if (kAxis > 0.0f) {
                    float axisSpeed = dot(anchorVelocity(b, rb) -
                                          anchorVelocity(a, ra), axisA);
                    float displacement = dot(pb - pa, axisA);
                    constexpr float kTau = 6.28318530718f;
                    float effectiveMass = 1.0f / kAxis;
                    float omega = kTau * j.suspensionFrequencyHz;
                    float stiffness = effectiveMass * omega * omega;
                    float damping = 2.0f * effectiveMass *
                                    j.suspensionDampingRatio * omega;
                    float softness = 1.0f /
                        (dt * (damping + dt * stiffness));
                    float bias = displacement * dt * stiffness * softness;
                    float lambda = -(axisSpeed + bias +
                                     softness * j.springImpulse) /
                                   (kAxis + softness);
                    j.springImpulse += lambda;
                    applyJointImpulse(j, a, b, ra, rb, axisA * lambda, -1.0f);
                }
                // Lock angular DOF except rotation about the axle.
                Vec3 wr = a.angularVelocity - b.angularVelocity;
                Vec3 refA = rotate(a.orientation, j.localRefA);
                Vec3 refB = rotate(b.orientation, j.localRefB);
                Vec3 frameErr = cross(refB, refA);
                for (Vec3 t : {t1, t2}) {
                    float k = dot(t, a.invInertiaMul(t)) +
                              dot(t, b.invInertiaMul(t));
                    if (k <= 0.0f) continue;
                    float lambda = -(dot(wr, t) +
                                     dot(frameErr, t) * (kBeta / dt)) / k;
                    applyJointAngularImpulse(j, a, b, t * lambda);
                    wr = a.angularVelocity - b.angularVelocity;
                }
                // Axle motor.
                if (j.enableMotor) {
                    float kAxle = dot(axisA, a.invInertiaMul(axisA)) +
                                  dot(axisA, b.invInertiaMul(axisA));
                    if (kAxle > 0.0f) {
                        float wAxle = dot(b.angularVelocity -
                                          a.angularVelocity, axisA);
                        float lambda = (wAxle - j.motorSpeed) / kAxle;
                        float maxImpulse = j.maxMotorTorque * dt;
                        float oldImpulse = j.motorImpulse;
                        j.motorImpulse = vclamp(oldImpulse + lambda,
                                                -maxImpulse, maxImpulse);
                        lambda = j.motorImpulse - oldImpulse;
                        applyJointAngularImpulse(j, a, b, axisA * lambda);
                    }
                }
                break;
            }
            case JointType::Rope: {
                // Only resists stretching beyond maxLength.
                Vec3 d = pa - pb;
                float len = length(d);
                if (len < 1e-8f || len <= j.maxLength) break;
                Vec3 n = d * (1.0f / len);
                Vec3 raxn = cross(ra, n), rbxn = cross(rb, n);
                float k = a.solverInvMass() + b.solverInvMass() +
                          dot(raxn, a.invInertiaMul(raxn)) +
                          dot(rbxn, b.invInertiaMul(rbxn));
                if (k <= 0.0f) break;
                float overshoot = len - j.maxLength;
                float bias = overshoot * (kBeta / dt);
                float lambda = -(dot(vel, n) + bias) / k;
                float oldImpulse = j.limitImpulse;
                j.limitImpulse = vmin(0.0f, oldImpulse + lambda);
                lambda = j.limitImpulse - oldImpulse;
                applyJointImpulse(j, a, b, ra, rb, n * lambda, +1.0f);
                break;
            }
            case JointType::Pulley: {
                // lengthA + ratio * lengthB = restTotal.
                Vec3 dA = pa - j.pulleyAnchorA;
                Vec3 dB = pb - j.pulleyAnchorB;
                float lenA = length(dA);
                float lenB = length(dB);
                if (lenA < 1e-8f || lenB < 1e-8f) break;
                Vec3 nA = dA * (1.0f / lenA);
                Vec3 nB = dB * (1.0f / lenB);
                Vec3 raxnA = cross(ra, nA);
                Vec3 rbxnB = cross(rb, nB);
                float kA = a.solverInvMass() +
                           dot(raxnA, a.invInertiaMul(raxnA));
                float kB = b.solverInvMass() +
                           dot(rbxnB, b.invInertiaMul(rbxnB));
                float ratio = j.pulleyRatio;
                float k = kA + ratio * ratio * kB;
                if (k <= 0.0f) break;
                float C = lenA + ratio * lenB - j.restLength;
                float bias = C * (kBeta / dt);
                float speedA = dot(anchorVelocity(a, ra), nA);
                float speedB = dot(anchorVelocity(b, rb), nB);
                float cdot = speedA + ratio * speedB;
                float lambda = -(cdot + bias) / k;
                float oldImpulse = j.limitImpulse;
                j.limitImpulse += lambda;
                lambda = j.limitImpulse - oldImpulse;
                if (a.isDynamic() && !a.isLocked()) {
                    a.velocity += nA * (lambda * a.solverInvMass());
                    a.angularVelocity +=
                        a.invInertiaMul(cross(ra, nA * lambda));
                }
                if (b.isDynamic() && !b.isLocked()) {
                    b.velocity += nB * (lambda * ratio * b.solverInvMass());
                    b.angularVelocity +=
                        b.invInertiaMul(cross(rb, nB * (lambda * ratio)));
                }
                j.reactionLinearImpulse += nA * lambda;
                break;
            }
            case JointType::Gear: {
                // Couples angular velocity: wA + ratio * wB = 0.
                Vec3 axisA = normalize(rotate(a.orientation, j.localAxisA));
                Vec3 axisB = normalize(rotate(b.orientation, j.localAxisB));
                float ratio = j.gearRatio;
                float kA = dot(axisA, a.invInertiaMul(axisA));
                float kB = dot(axisB, b.invInertiaMul(axisB));
                float k = kA + ratio * ratio * kB;
                if (k <= 0.0f) break;
                float wA = dot(a.angularVelocity, axisA);
                float wB = dot(b.angularVelocity, axisB);
                float cdot = wA + ratio * wB;
                float lambda = -cdot / k;
                float oldImpulse = j.motorImpulse;
                j.motorImpulse += lambda;
                lambda = j.motorImpulse - oldImpulse;
                if (a.isDynamic() && !a.isLocked())
                    a.angularVelocity += a.invInertiaMul(axisA * lambda);
                if (b.isDynamic() && !b.isLocked())
                    b.angularVelocity +=
                        b.invInertiaMul(axisB * (lambda * ratio));
                j.reactionAngularImpulse += axisA * lambda;
                break;
            }
            }
        }
    }

    finishBrokenJoints(dt);
}

void World::finishBrokenJoints(float dt) {
    for (Joint& joint : joints_) {
        float force = length(joint.reactionLinearImpulse) / dt;
        float torque = length(joint.reactionAngularImpulse) / dt;
        joint.broken = joint.broken || force > joint.breakForce ||
                       torque > joint.breakTorque;
    }
    for (size_t i = joints_.size(); i-- > 0;) {
        if (!joints_[i].broken) continue;
        const Joint& joint = joints_[i];
        uint32_t slot = jointDenseToSlot_[i];
        jointBreakEvents_.push_back({
            JointId::make(slot, jointSlots_[slot].generation),
            bodyHandle(joint.a), bodyHandle(joint.b),
            length(joint.reactionLinearImpulse) / dt,
            length(joint.reactionAngularImpulse) / dt});
        removeJointDense(static_cast<uint32_t>(i));
    }
}

// Island-based sleeping with configurable thresholds, gradual sleep (drowsy
// intermediate state), contact stability tracking, and sleep/wake callbacks.
// Delegates to the SleepManager for the improved algorithm.
void World::updateSleeping(float dt) {
    sleepManager_.update(
        bodies_, contacts_, joints_,
        unionParent_, islandTimer_, dt,
        [this](BodyIndex dense) { return bodyHandle(dense); });
}

// Predictive Contact Sweeping (PCS)
//
// 1. Speculative detection: contacts are created while pairs are still apart,
//    whenever their relative motion could close the gap this step.
// 2. Velocity solve: iterative sequential impulses at contact points with full
//    rotational response. Each contact only removes approach velocity in
//    excess of gap/dt, so grazing motion is untouched and piles are handled
//    by iteration instead of a time-of-impact event queue.
// 3. Conservative-advancement safety net: after integrating positions, any
//    pair that ended up interpenetrating is rewound along its actual motion
//    (rotation included) to the moment of first contact. Tunneling stays
//    impossible regardless of linear or angular speed.
void World::step(float dt) {
    AccessGuard guard(*this, AccessKind::Step, "step");
    const bool canFallback = deviceLossPolicy_ == DeviceLossPolicy::FallbackToCPU &&
                             std::string(backend_->name()) == "cuda";
    if (!canFallback) {
        stepImpl(dt);
        return;
    }

    // CUDA functions may have copied a partially advanced host body array
    // before reporting a recoverable error. Save the complete observable World
    // state first, then restart the same frame on CPU instead of continuing
    // from an inconsistent half-step.
    StepRollback rollback = saveStepRollback();
    try {
        stepImpl(dt);
    } catch (const BackendFailure& failure) {
        activateCpuFallback(failure);
        restoreStepRollback(std::move(rollback));
        stepImpl(dt);
    }
}

void World::stepImpl(float dt) {
    using Clock = std::chrono::steady_clock;
    VELOX_PROFILE_FRAME();
    VELOX_PROFILE_CATEGORY_SCOPE("World::step", velox::profile::Category::Setup);
    const auto stepStart = Clock::now();
    lastStepStats_ = {};
    lastStepStats_.dt = dt;
    lastStepStats_.bodyCount = bodies_.size();
    lastStepStats_.jointCount = joints_.size();
    if (!finiteFloat(dt) || dt < 0.0f)
        throw std::invalid_argument("velox: timestep must be finite and non-negative");
    drainAsyncQueries();
    if (dt == 0.0f) {
        for (const Body& body : bodies_)
            if (body.isDynamic() && !isFullyAsleep(body.asleep))
                ++lastStepStats_.awakeDynamicBodies;
        lastStepStats_.totalMs = std::chrono::duration<double, std::milli>(
            Clock::now() - stepStart).count();
        return;
    }
    jointBreakEvents_.clear();
    bodyEvents_.clear();
    bodyEvents_.swap(pendingBodyEvents_);
    pendingBodyEvents_.clear();
    if (substeps <= 0) throw std::invalid_argument("velox: substeps must be positive");
    if (!finiteVec(gravity)) throw std::invalid_argument("velox: gravity must be finite");
    // Validate mutable public state on every build. Bodies and joints are
    // intentionally editable through the public API, so malformed values
    // must never enter the solver in a Release build.
    for (Body& body : bodies_) {
        validateRuntimeBody(body);
        switch (body.shape) {
        case ShapeType::Sphere:
            requirePositive(body.radius, "velox: sphere body has an invalid radius");
            break;
        case ShapeType::Box:
            if (!finiteVec(body.halfExtents) || body.halfExtents.x <= 0.0f ||
                body.halfExtents.y <= 0.0f || body.halfExtents.z <= 0.0f)
                throw std::invalid_argument("velox: box body has invalid half extents");
            break;
        case ShapeType::Capsule:
            requirePositive(body.radius, "velox: capsule body has an invalid radius");
            requireNonNegative(body.capsuleHalfHeight,
                               "velox: capsule body has an invalid half height");
            break;
        case ShapeType::Cylinder:
            requirePositive(body.radius, "velox: cylinder body has an invalid radius");
            requirePositive(body.capsuleHalfHeight,
                            "velox: cylinder body has an invalid half height");
            break;
        case ShapeType::Cone:
            requirePositive(body.radius, "velox: cone body has an invalid radius");
            requirePositive(body.capsuleHalfHeight,
                            "velox: cone body has an invalid half height");
            break;
        case ShapeType::RoundedBox:
            if (!finiteVec(body.halfExtents) || body.halfExtents.x <= 0.0f ||
                body.halfExtents.y <= 0.0f || body.halfExtents.z <= 0.0f)
                throw std::invalid_argument("velox: rounded box body has invalid half extents");
            requirePositive(body.radius, "velox: rounded box body has an invalid radius");
            break;
        case ShapeType::Ellipsoid:
            if (!finiteVec(body.halfExtents) || body.halfExtents.x <= 0.0f ||
                body.halfExtents.y <= 0.0f || body.halfExtents.z <= 0.0f)
                throw std::invalid_argument("velox: ellipsoid body has invalid radii");
            break;
        case ShapeType::Plane:
            if (!finiteVec(body.planeNormal) || lengthSq(body.planeNormal) < 1e-12f ||
                !finiteFloat(body.planeOffset))
                throw std::invalid_argument("velox: plane body has invalid geometry");
            break;
        case ShapeType::Mesh:
            if ((size_t)body.meshIndex >= meshes_.meshes.size())
                throw std::out_of_range("velox: mesh body references an invalid mesh");
            break;
        case ShapeType::Hull:
            if (body.hullCount < 4 || (size_t)body.hullFirst + body.hullCount >
                                        meshes_.hullPoints.size() ||
                body.hullFaceCount == 0 ||
                (size_t)body.hullFaceFirst + size_t(body.hullFaceCount) * 3 >
                    meshes_.hullFaceIndices.size())
                throw std::out_of_range("velox: hull body references invalid point storage");
            break;
        case ShapeType::Compound:
            if (body.compoundCount == 0 ||
                (size_t)body.compoundFirst + body.compoundCount >
                    meshes_.compoundChildren.size())
                throw std::out_of_range(
                    "velox: compound body references invalid child storage");
            for (uint32_t i = 0; i < body.compoundCount; ++i) {
                const CompoundChild& child =
                    meshes_.compoundChildren[body.compoundFirst + i];
                if (!finiteVec(child.localPosition) ||
                    !finiteQuat(child.localOrientation) ||
                    !isConvexVolume(child.shape))
                    throw std::invalid_argument(
                        "velox: compound child contains invalid state");
            }
            break;
        }
        body.orientation = normalize(body.orientation);
    }
    for (const Joint& joint : joints_) {
        if (!validJointType(joint.type) || joint.a >= bodies_.size() ||
            joint.b >= bodies_.size() || joint.a == joint.b ||
            !finiteVec(joint.localAnchorA) || !finiteVec(joint.localAnchorB) ||
            !finiteVec(joint.localAxisA) || !finiteVec(joint.localAxisB) ||
            !finiteVec(joint.localRefA) || !finiteVec(joint.localRefB))
            throw std::invalid_argument("velox: joint contains invalid body endpoints");
        bool framed = joint.type == JointType::Hinge ||
                      joint.type == JointType::ConeTwist ||
                      joint.type == JointType::Fixed ||
                      joint.type == JointType::Prismatic ||
                      joint.type == JointType::SixDof ||
                      joint.type == JointType::Motor;
        if (framed && (lengthSq(joint.localAxisA) < 1e-12f ||
                       lengthSq(joint.localAxisB) < 1e-12f ||
                       lengthSq(joint.localRefA) < 1e-12f ||
                       lengthSq(joint.localRefB) < 1e-12f ||
                       lengthSq(cross(joint.localAxisA,
                                      joint.localRefA)) < 1e-12f ||
                       lengthSq(cross(joint.localAxisB,
                                      joint.localRefB)) < 1e-12f))
            throw std::invalid_argument("velox: joint contains an invalid local frame");
        if (!finiteFloat(joint.motorSpeed) || !finiteFloat(joint.maxMotorTorque) ||
            joint.maxMotorTorque < 0.0f || !finiteFloat(joint.maxMotorForce) ||
            joint.maxMotorForce < 0.0f || !finiteFloat(joint.lowerLimit) ||
            !finiteFloat(joint.upperLimit) || joint.lowerLimit > joint.upperLimit ||
            !finiteFloat(joint.swingLimit) || joint.swingLimit < 0.0f ||
            joint.swingLimit > 3.14159265f || !finiteFloat(joint.lowerTwistLimit) ||
            !finiteFloat(joint.upperTwistLimit) ||
            joint.lowerTwistLimit < -3.14159265f ||
            joint.upperTwistLimit > 3.14159265f ||
            joint.lowerTwistLimit > joint.upperTwistLimit)
            throw std::invalid_argument("velox: joint contains invalid motor or limit settings");
        if ((joint.linearLimitMask & ~uint8_t(0x7)) != 0 ||
            (joint.angularLimitMask & ~uint8_t(0x7)) != 0 ||
            (joint.linearMotorMask & ~uint8_t(0x7)) != 0 ||
            (joint.angularMotorMask & ~uint8_t(0x7)) != 0 ||
            !finiteVec(joint.lowerLinearLimit) ||
            !finiteVec(joint.upperLinearLimit) ||
            !finiteVec(joint.lowerAngularLimit) ||
            !finiteVec(joint.upperAngularLimit) ||
            !finiteVec(joint.linearMotorSpeed) ||
            !finiteVec(joint.angularMotorSpeed) ||
            !finiteVec(joint.maxLinearMotorForce) ||
            !finiteVec(joint.maxAngularMotorTorque) ||
            joint.lowerLinearLimit.x > joint.upperLinearLimit.x ||
            joint.lowerLinearLimit.y > joint.upperLinearLimit.y ||
            joint.lowerLinearLimit.z > joint.upperLinearLimit.z ||
            joint.lowerAngularLimit.x > joint.upperAngularLimit.x ||
            joint.lowerAngularLimit.y > joint.upperAngularLimit.y ||
            joint.lowerAngularLimit.z > joint.upperAngularLimit.z ||
            joint.lowerAngularLimit.x < -3.14159265f ||
            joint.lowerAngularLimit.y < -3.14159265f ||
            joint.lowerAngularLimit.z < -3.14159265f ||
            joint.upperAngularLimit.x > 3.14159265f ||
            joint.upperAngularLimit.y > 3.14159265f ||
            joint.upperAngularLimit.z > 3.14159265f ||
            joint.maxLinearMotorForce.x < 0.0f ||
            joint.maxLinearMotorForce.y < 0.0f ||
            joint.maxLinearMotorForce.z < 0.0f ||
            joint.maxAngularMotorTorque.x < 0.0f ||
            joint.maxAngularMotorTorque.y < 0.0f ||
            joint.maxAngularMotorTorque.z < 0.0f)
            throw std::invalid_argument(
                "velox: joint contains invalid 6DoF settings");
        if (!finiteFloat(joint.springFrequencyHz) ||
            !finiteFloat(joint.springDampingRatio) ||
            joint.springFrequencyHz < 0.0f || joint.springDampingRatio < 0.0f ||
            (joint.enableSpring && (joint.type != JointType::Distance ||
                                    joint.springFrequencyHz <= 0.0f)))
            throw std::invalid_argument("velox: joint contains invalid spring settings");
        if (!finiteFloat(joint.breakForce) || !finiteFloat(joint.breakTorque) ||
            joint.breakForce < 0.0f || joint.breakTorque < 0.0f)
            throw std::invalid_argument("velox: joint contains invalid break thresholds");
    }
    const int nSub = substeps > 0 ? substeps : 1;
    const float h = dt / nSub;

    // Detect ONCE per step, with a speculative reach covering the full dt.
    // The solver substeps below re-evaluate each contact's live gap from
    // current body positions (Contact::bias0), Box2D-v3 style.
    const auto detectionStart = Clock::now();
    backend_->integrate(bodies_, gravity, h); // first substep's gravity
    bool hasHighStaticToi = false;
    for (const Body& body : bodies_) {
        if (body.isDynamic() && !isFullyAsleep(body.asleep) &&
            body.ccdTuning.quality == MotionQuality::High &&
            body.ccdTuning.enableContinuous) {
            hasHighStaticToi = true;
            break;
        }
    }
    std::vector<Body> highToiStarts;
    if (hasHighStaticToi) highToiStarts = bodies_;
    const std::vector<uint64_t>* hostPairs = nullptr;
    const auto broadPhaseStart = Clock::now();
    if (backend_->wantsHostPairs()) {
        buildCandidatePairs(dt); // incremental AABB-tree broad phase
        hostPairs = &candidatePairs_;
    }
    const auto broadPhaseEnd = Clock::now();
    // Narrowphase contact generation is instrumented at its implementation
    // site (Backend::findContacts) so the zone sits on the actual hot loop.
    backend_->findContacts(bodies_, meshes_, dt, hostPairs, contacts_);
    const auto detectionEnd = Clock::now();
    lastStepStats_.generatedContacts = contacts_.size();
    lastStepStats_.narrowPhaseTests = candidatePairs_.size();
    {
        size_t proxyCount = 0;
        for (int32_t p : broadPhase_->proxies)
            if (p >= 0) ++proxyCount;
        lastStepStats_.broadPhaseProxies = proxyCount;
    }

    // Joint-connected bodies do not collide by default. Without this filter,
    // contacts at a hinge anchor fight the joint and CCD repeatedly rewinds the
    // pair. Set Joint::collideConnected when the linkage should self-collide.
    if (!joints_.empty() && !contacts_.empty()) {
        std::vector<uint64_t> excluded;
        excluded.reserve(joints_.size());
        for (const Joint& j : joints_) {
            if (j.collideConnected) continue;
            BodyIndex lo = j.a < j.b ? j.a : j.b;
            BodyIndex hi = j.a < j.b ? j.b : j.a;
            excluded.push_back((uint64_t)lo << 32 | hi);
        }
        std::sort(excluded.begin(), excluded.end());
        contacts_.erase(std::remove_if(contacts_.begin(), contacts_.end(),
            [&](const Contact& c) {
                BodyIndex lo = c.a < c.b ? c.a : c.b;
                BodyIndex hi = c.a < c.b ? c.b : c.a;
                return std::binary_search(excluded.begin(), excluded.end(),
                                          (uint64_t)lo << 32 | hi);
            }), contacts_.end());
    }

    if (contactModifier_ && !contacts_.empty()) {
        contacts_.erase(std::remove_if(contacts_.begin(), contacts_.end(),
            [&](Contact& c) {
                ContactModifyData data;
                data.a = bodyHandle(c.a);
                data.b = bodyHandle(c.b);
                data.point = c.point;
                data.normal = c.normal;
                data.restitution = c.restitution;
                data.friction1 = c.friction1;
                data.friction2 = c.friction2;
                data.rollingFriction = c.rollingFriction;
                data.spinningFriction = c.spinningFriction;
                contactModifier_(data);
                if (!data.enabled) return true;
                if (!finiteVec(data.point) || !finiteVec(data.normal) ||
                    lengthSq(data.normal) < 1e-12f ||
                    !finiteFloat(data.restitution) || data.restitution < 0.0f ||
                    !finiteFloat(data.friction1) || data.friction1 < 0.0f ||
                    !finiteFloat(data.friction2) || data.friction2 < 0.0f ||
                    !finiteFloat(data.rollingFriction) || data.rollingFriction < 0.0f ||
                    !finiteFloat(data.spinningFriction) || data.spinningFriction < 0.0f)
                    throw std::invalid_argument(
                        "velox: contact modifier returned invalid contact state");

                c.point = data.point;
                c.normal = normalize(data.normal);
                c.localAnchorA = contactAnchorLocal(bodies_[c.a], c.point);
                c.localAnchorB = contactAnchorLocal(bodies_[c.b], c.point);
                c.bias0 = c.gap;
                c.vn0 = dot(npPointVelocity(bodies_[c.a], c.point) -
                            npPointVelocity(bodies_[c.b], c.point), c.normal);
                c.restitution = data.restitution;
                c.friction1 = data.friction1;
                c.friction2 = data.friction2;
                c.rollingFriction = data.rollingFriction;
                c.spinningFriction = data.spinningFriction;
                return false;
            }), contacts_.end());
    }

    // --- wake pass ------------------------------------------------------------
    // A sleeping body is woken when something awake and moving (or actually
    // striking it) shares a contact with it.
    for (const Contact& c : contacts_) {
        Body& a = bodies_[c.a];
        Body& b = bodies_[c.b];
        if (a.isSensor() || b.isSensor()) continue;
        bool impact = c.vn0 < -0.1f;
        auto moving = [](const Body& x) {
            return lengthSq(x.velocity) + lengthSq(x.angularVelocity) > 1e-3f;
        };
        if (isFullyAsleep(a.asleep) && !b.isStatic() && !isFullyAsleep(b.asleep) &&
            (impact || moving(b))) {
            a.asleep = 0; a.sleepTimer = 0.0f;
        }
        if (isFullyAsleep(b.asleep) && !a.isStatic() && !isFullyAsleep(a.asleep) &&
            (impact || moving(a))) {
            b.asleep = 0; b.sleepTimer = 0.0f;
        }
    }

    // --- warm starting ---------------------------------------------------------
    // Carry accumulated normal impulses from last frame. Explicit manifold
    // features match exactly; generic GJK contacts fall back to proximity.
    if (!prevContacts_.empty()) {
        auto keyOf = [](const Contact& c) { return (uint64_t)c.a << 32 | c.b; };
        for (Contact& c : contacts_) {
            uint64_t key = keyOf(c);
            auto lo = std::lower_bound(prevContacts_.begin(), prevContacts_.end(), key,
                [&](const Contact& p, uint64_t k) { return keyOf(p) < k; });
            if (c.featureKey != 0) {
                bool matched = false;
                for (auto it = lo; it != prevContacts_.end() && keyOf(*it) == key; ++it)
                    if (it->featureKey == c.featureKey) {
                        c.normalImpulse = it->normalImpulse;
                        c.tangentImpulse1 = it->tangentImpulse1;
                        c.tangentImpulse2 = it->tangentImpulse2;
                        c.rollingImpulse1 = it->rollingImpulse1;
                        c.rollingImpulse2 = it->rollingImpulse2;
                        c.spinningImpulse = it->spinningImpulse;
                        // A high-speed approach braked across the speculative
                        // gap can span several frames before touchdown; keep
                        // the ORIGINAL approach speed as the restitution
                        // reference while the pair is still closing fast, or
                        // the bounce reflects only the residual speed of the
                        // final frame. Once the body is slow or separating,
                        // the fresh vn0 takes over (no perpetual bouncing).
                        if (c.vn0 < -1.0f && it->vn0 < c.vn0) c.vn0 = it->vn0;
                        matched = true;
                        break;
                    }
                if (matched) continue;
            }
            // Restitution reference carry (see the featureKey branch): a fast
            // approach braked across the speculative gap spans frames, and a
            // fast mover travels far beyond any positional matching radius,
            // so the carry matches on the body PAIR alone. All contacts of
            // one pair share the rigid approach, making this exact enough.
            if (c.vn0 < -1.0f)
                for (auto it = lo; it != prevContacts_.end() && keyOf(*it) == key; ++it)
                    if (it->vn0 < c.vn0) c.vn0 = it->vn0;
            float bestDistSq = 2.5e-3f; // 5 cm matching radius
            for (auto it = lo; it != prevContacts_.end() && keyOf(*it) == key; ++it) {
                float d = lengthSq(it->point - c.point);
                if (d < bestDistSq) {
                    bestDistSq = d;
                    c.normalImpulse = it->normalImpulse;
                    c.tangentImpulse1 = it->tangentImpulse1;
                    c.tangentImpulse2 = it->tangentImpulse2;
                    c.rollingImpulse1 = it->rollingImpulse1;
                    c.rollingImpulse2 = it->rollingImpulse2;
                    c.spinningImpulse = it->spinningImpulse;
                }
            }
        }
    }

    prev_.resize(bodies_.size());
    for (size_t i = 0; i < bodies_.size(); ++i)
        prev_[i] = {bodies_[i].position, bodies_[i].orientation};

    for (const Contact& contact : contacts_)
        if (!bodies_[contact.a].isSensor() && !bodies_[contact.b].isSensor())
            ++lastStepStats_.solvedContacts;
    const auto solverStart = Clock::now();
    double contactSolverAccum = 0.0, jointSolverAccum = 0.0;

    // --- solver substeps -------------------------------------------------------
    // Each substep: gravity, velocity solve against live gaps, joints, and
    // position integration. Warm starting applies only on the first substep;
    // afterwards the accumulated impulses already live in the velocities.
    VELOX_PROFILE_GPU_SYNC("advanceSubsteps");
    bool advancedOnDevice = backend_->advanceSubsteps(
        bodies_, contacts_, joints_, gravity, h, nSub, solverOptions_);
    lastStepStats_.deviceSubsteps = advancedOnDevice;
    if (advancedOnDevice)
        lastStepStats_.velocityIterations = backend_->lastVelocityIterations();
    if (advancedOnDevice) finishBrokenJoints(h);
    if (!advancedOnDevice) {
        for (int s = 0; s < nSub; ++s) {
            if (s > 0) backend_->integrate(bodies_, gravity, h);
            const auto csStart = Clock::now();
            backend_->solveVelocities(bodies_, contacts_, h, s == 0, solverOptions_);
            const auto csEnd = Clock::now();
            lastStepStats_.velocityIterations += backend_->lastVelocityIterations();
            const auto jsStart = Clock::now();
            solveJoints(h);
            const auto jsEnd = Clock::now();
            contactSolverAccum += std::chrono::duration<double, std::milli>(csEnd - csStart).count();
            jointSolverAccum += std::chrono::duration<double, std::milli>(jsEnd - jsStart).count();
            for (Body& b : bodies_) {
                if (b.isStatic() || isFullyAsleep(b.asleep)) continue;
                b.advanceTransform(h);
            }
        }
    }
    const auto solverEnd = Clock::now();

    // --- conservative-advancement safety net ---------------------------------
    // Only pairs that ended the step interpenetrating need rescue. Speculative
    // detection guarantees every pair that could touch this step produced a
    // contact, so it suffices to re-check the (deduplicated) contact pairs;
    // for offenders, walk time forward from the pre-step state in provably-
    // safe increments (gap / max surface speed) until first contact.
    constexpr float kSlop = 1e-3f;
    const MeshSoupView soup = view(meshes_);
    pairKeys_.clear();
    for (const Contact& c : contacts_)
        pairKeys_.push_back((uint64_t)c.a << 32 | c.b);
    std::sort(pairKeys_.begin(), pairKeys_.end());
    pairKeys_.erase(std::unique(pairKeys_.begin(), pairKeys_.end()), pairKeys_.end());

    // High-quality bodies that only interact with static or other High bodies
    // replay on one shared timeline. Mixed-quality dynamic pairs remain on the
    // regular PCS path so no one-sided rewind can violate momentum.
    std::vector<uint8_t> highToiProcessedBodies;
    if (!highToiStarts.empty()) {
        lastStepStats_.multiToiEvents = processHighToi(
            bodies_, highToiStarts, contacts_, dt, view(meshes_), multiToiSettings_,
            collisionFilterCallback_, highToiProcessedBodies);
    }
    const bool highToiProcessed = std::any_of(highToiProcessedBodies.begin(),
                                               highToiProcessedBodies.end(),
                                               [](uint8_t value) { return value != 0; });

    for (uint64_t key : pairKeys_) {
        BodyIndex i = (BodyIndex)(key >> 32), j = (BodyIndex)key;
        // Keep A dynamic for the static-dynamic case. Dynamic-dynamic pairs
        // are rewound symmetrically below.
        if (!bodies_[i].isDynamic() && bodies_[j].isDynamic()) {
            BodyIndex t = i; i = j; j = t;
        }
        Body& a = bodies_[i];
        Body& b = bodies_[j];
        if (a.isSensor() || b.isSensor()) continue;
        if (!a.isDynamic()) continue;
        // Only skip bodies that the shared high-quality replay actually
        // rewound. A High body paired with Medium/Low motion intentionally
        // remains on this established PCS recovery path.
        if (highToiProcessed &&
            (highToiProcessedBodies[i] || highToiProcessedBodies[j])) continue;
        if (!a.ccdTuning.enableContinuous ||
            a.ccdTuning.quality == MotionQuality::Low || a.isLocked() ||
            a.maxPointSpeed() < a.ccdTuning.minVelocityForCCD)
            continue;
        float search = a.maxPointSpeed() * dt + a.radius +
                       length(a.halfExtents) + a.capsuleHalfHeight + 0.1f;
        {

            GapProbe end = gapAt(a, a.position, a.orientation,
                                 b, b.position, b.orientation, soup, search);
            if (end.gap >= -8e-3f) continue; // shallow: leave it to the solver

            // Interpolators over the step for both bodies.
            Vec3 pa0 = prev_[i].position, da = a.position - pa0;
            Quat qa0 = prev_[i].orientation;
            Vec3 pb0 = b.isStatic() ? b.position : prev_[j].position;
            Vec3 db = b.position - pb0;
            Quat qb0 = b.isStatic() ? b.orientation : prev_[j].orientation;

            float bound = a.maxPointSpeed() * dt + (b.isStatic() ? 0.0f : b.maxPointSpeed() * dt);
            if (bound < 1e-9f) continue;

            Vec3 angularMomentumA = a.worldAngularMomentum();
            Vec3 angularMomentumB = b.isDynamic() ? b.worldAngularMomentum() : Vec3{};
            auto orientationAt = [](const Body& body, const Quat& start,
                                    const Vec3& angularMomentum, float elapsed) {
                Body sample = body;
                sample.orientation = start;
                sample.angularVelocity = sample.invInertiaMul(angularMomentum);
                sample.advanceOrientation(elapsed);
                return sample.orientation;
            };

            float t = 0.0f;
            Vec3 n = end.normal;
            for (int iter = 0; iter < 24 && t < 1.0f; ++iter) {
                Quat qa = orientationAt(a, qa0, angularMomentumA, dt * t);
                Quat qb = b.isStatic()
                    ? qb0
                    : orientationAt(b, qb0, angularMomentumB, dt * t);
                GapProbe g = gapAt(a, pa0 + da * t, qa, b, pb0 + db * t, qb, soup, search);
                n = g.normal;
                if (g.gap < kSlop) break;
                t = vmin(1.0f, t + g.gap / bound);
            }
            if (t >= 1.0f) continue; // never actually touched (false alarm)

            // Rewind both dynamic trajectories to the same impact time. This
            // preserves their shared timeline instead of arbitrarily clamping
            // one participant and leaving the other at the end of the step.
            a.position = pa0 + da * t;
            a.orientation = qa0;
            a.angularVelocity = a.invInertiaMul(angularMomentumA);
            a.advanceOrientation(dt * t);
            if (b.isDynamic()) {
                b.position = pb0 + db * t;
                b.orientation = qb0;
                b.angularVelocity = b.invInertiaMul(angularMomentumB);
                b.advanceOrientation(dt * t);
            }

            // Remove the remaining approach velocity with an inverse-mass
            // impulse, bouncing with the pair's combined restitution just as
            // the contact solver would have (differential testing vs Jolt:
            // an inelastic rescue silently ate the bounce of every impact
            // fast enough to need CCD). Linear momentum is conserved for two
            // dynamic bodies; static geometry receives no velocity update.
            float vn = dot(a.velocity - b.velocity, n);
            float invMassSum = a.solverInvMass() + b.solverInvMass();
            if (vn < 0.0f && invMassSum > 0.0f) {
                constexpr float kRestitutionThreshold = 1.0f; // matches solver
                float restitution = vn < -kRestitutionThreshold
                    ? combineMaterial(a.restitution, b.restitution,
                                      a.restitutionCombine, b.restitutionCombine)
                    : 0.0f;
                float impulse = -(1.0f + restitution) * vn / invMassSum;
                a.velocity += n * (impulse * a.solverInvMass());
                if (b.isDynamic()) b.velocity -= n * (impulse * b.solverInvMass());
            }

            // Finish the frame from TOI using the corrected velocities. The
            // normal approach component is now zero, so this cannot recreate
            // the crossing, and tangential/center-of-mass motion is retained.
            float remaining = dt * (1.0f - t);
            a.advanceTransform(remaining);
            if (b.isDynamic()) b.advanceTransform(remaining);
        }
    }

    VELOX_PROFILE_GPU_SYNC("fetchImpulses");
    backend_->fetchImpulses(contacts_); // GPU-resident impulses -> host contacts
    if (highToiProcessed) backend_->invalidateCaches();
    const auto ccdEnd = Clock::now();

    // --- positional correction (split-impulse style) --------------------------
    // Resolve residual penetration by translating bodies apart directly.
    // Velocities are untouched, so no energy is injected (Baumgarte bias in
    // the velocity solve launches stacked bodies). The current gap of each
    // contact is estimated from its detected gap plus the relative motion of
    // its bodies along the normal since detection.
    {
        constexpr float kPositionSlop = 1e-3f, kResolve = 0.6f;
        for (int iter = 0; iter < solverOptions_.positionIterations; ++iter) {
            for (const Contact& c : contacts_) {
                Body& a = bodies_[c.a];
                Body& b = bodies_[c.b];
                if (a.isSensor() || b.isSensor()) continue;
                float invMassSum = a.solverInvMass() + b.solverInvMass();
                if (invMassSum <= 0.0f) continue;
                float g = contactLiveGap(a, b, c);
                if (g >= -kPositionSlop) continue;
                float resolve = kResolve;
                if (solverOptions_.enableStackStabilization) {
                    const Vec3 relativeVelocity = a.velocity - b.velocity;
                    if (lengthSq(relativeVelocity) < 1e-4f) {
                        const float stiffness = solverOptions_.stackStiffness / 1000.0f;
                        const float damping = 1.0f + solverOptions_.stackDamping * h;
                        resolve += 0.2f * stiffness / damping;
                    }
                }
                float push = -resolve * (g + kPositionSlop) / invMassSum;
                if (a.isDynamic() && !isFullyAsleep(a.asleep))
                    a.position += c.normal * (push * a.solverInvMass());
                if (b.isDynamic() && !isFullyAsleep(b.asleep))
                    b.position -= c.normal * (push * b.solverInvMass());
            }
        }
    }

    // --- contact and sensor events --------------------------------------------
    // Physical pairs are active when solving produced an impulse or their
    // live anchors are touching. Sensors additionally use the speculative
    // sweep condition, so a fast body crossing an entire trigger in one step
    // still produces a Begin followed by an End.
    {
        events_.clear();
        pairKeys_.clear();
        auto canonicalKey = [](BodyIndex a, BodyIndex b) {
            BodyIndex lo = a < b ? a : b;
            BodyIndex hi = a < b ? b : a;
            return (uint64_t)lo << 32 | hi;
        };
        // One pass over the contacts: collect (key, contact index) for every
        // active contact, then pick each pair's representative from its sorted
        // run. The previous per-key rescan of the whole contact array was
        // O(pairs x contacts) and dominated the entire step at ~10k contacts
        // (~60 ms of a ~80 ms step on an 8192-body pile).
        std::vector<std::pair<uint64_t, uint32_t>> active;
        active.reserve(contacts_.size());
        for (uint32_t index = 0; index < contacts_.size(); ++index) {
            const Contact& c = contacts_[index];
            const Body& a = bodies_[c.a];
            const Body& b = bodies_[c.b];
            bool sensor = a.isSensor() || b.isSensor();
            float liveGap = contactLiveGap(a, b, c);
            bool sweptSensorHit = sensor && c.vn0 < 0.0f &&
                                  c.gap <= -c.vn0 * dt + 1e-3f;
            if (c.normalImpulse <= 1e-6f && liveGap > 1e-3f && !sweptSensorHit)
                continue;
            active.emplace_back(canonicalKey(c.a, c.b), index);
        }
        std::sort(active.begin(), active.end());
        for (size_t i = 0; i < active.size();) {
            const uint64_t key = active[i].first;
            BodyIndex lo = (BodyIndex)(key >> 32);
            BodyIndex hi = (BodyIndex)key;
            bool persisted = std::binary_search(prevPairKeys_.begin(), prevPairKeys_.end(), key);
            ContactEvent ev{bodyHandle(lo), bodyHandle(hi), {}, {}, 0.0f,
                            persisted ? ContactEventType::Persist : ContactEventType::Begin,
                            bodies_[lo].isSensor() || bodies_[hi].isSensor()};
            bool representativeSet = false;
            for (; i < active.size() && active[i].first == key; ++i) {
                const Contact& c = contacts_[active[i].second];
                if (!representativeSet || c.normalImpulse > ev.impulse) {
                    representativeSet = true;
                    ev.impulse = c.normalImpulse;
                    ev.point = c.point;
                    // Narrow phase normals point from B toward A; events use
                    // the canonical low-handle to high-handle direction.
                    ev.normal = c.a == lo ? -c.normal : c.normal;
                }
            }
            pairKeys_.push_back(key);
            events_.push_back(ev);
        }
        for (uint64_t key : prevPairKeys_) {
            if (std::binary_search(pairKeys_.begin(), pairKeys_.end(), key)) continue;
            BodyIndex lo = (BodyIndex)(key >> 32);
            BodyIndex hi = (BodyIndex)key;
            if (lo >= bodies_.size() || hi >= bodies_.size()) continue;
            const Body& a = bodies_[lo];
            const Body& b = bodies_[hi];
            if ((a.isStatic() || isFullyAsleep(a.asleep)) &&
                (b.isStatic() || isFullyAsleep(b.asleep))) {
                ContactEvent persisted{bodyHandle(lo), bodyHandle(hi), {}, {}, 0.0f,
                                       ContactEventType::Persist,
                                       a.isSensor() || b.isSensor()};
                for (const Contact& c : prevContacts_) {
                    if (canonicalKey(c.a, c.b) != key) continue;
                    persisted.point = c.point;
                    persisted.normal = c.a == lo ? -c.normal : c.normal;
                    persisted.impulse = c.normalImpulse;
                    break;
                }
                pairKeys_.push_back(key);
                events_.push_back(persisted);
                continue;
            }
            events_.push_back({bodyHandle(lo), bodyHandle(hi), {}, {}, 0.0f,
                               ContactEventType::End,
                               bodies_[lo].isSensor() || bodies_[hi].isSensor()});
        }
        std::sort(pairKeys_.begin(), pairKeys_.end());
        pairKeys_.erase(std::unique(pairKeys_.begin(), pairKeys_.end()), pairKeys_.end());
        prevPairKeys_ = pairKeys_;
    }

    // --- soft bodies (XPBD, after rigid-body solve) --------------------------
    for (SoftBody& sb : softBodies_)
        softbody_detail::stepSoftBody(sb, gravity, h, bodies_);

    // --- sleeping + persistent contacts for next frame ------------------------
    updateSleeping(dt);
    for (Body& body : bodies_) {
        body.force = {};
        body.torque = {};
    }
    std::vector<Contact> nextPreviousContacts = contacts_;
    for (const Contact& previousContact : prevContacts_) {
        const uint64_t key = (uint64_t)(previousContact.a < previousContact.b
                                            ? previousContact.a : previousContact.b)
                           << 32 |
                             (previousContact.a < previousContact.b
                                  ? previousContact.b : previousContact.a);
        if (!std::binary_search(pairKeys_.begin(), pairKeys_.end(), key)) continue;
        const Body& a = bodies_[previousContact.a];
        const Body& b = bodies_[previousContact.b];
        if (!((a.isStatic() || isFullyAsleep(a.asleep)) &&
              (b.isStatic() || isFullyAsleep(b.asleep)))) continue;
        const bool alreadyCached = std::any_of(
            nextPreviousContacts.begin(), nextPreviousContacts.end(),
            [&](const Contact& contact) {
                return ((uint64_t)(contact.a < contact.b ? contact.a : contact.b) << 32 |
                        (contact.a < contact.b ? contact.b : contact.a)) == key;
            });
        if (!alreadyCached) nextPreviousContacts.push_back(previousContact);
    }
    prevContacts_ = std::move(nextPreviousContacts);
    std::sort(prevContacts_.begin(), prevContacts_.end(),
              [](const Contact& x, const Contact& y) {
                  return ((uint64_t)x.a << 32 | x.b) < ((uint64_t)y.a << 32 | y.b);
              });

    const auto stepEnd = Clock::now();
    for (const Body& body : bodies_)
        if (body.isDynamic() && !isFullyAsleep(body.asleep))
            ++lastStepStats_.awakeDynamicBodies;
    lastStepStats_.bodyCount = bodies_.size();
    lastStepStats_.jointCount = joints_.size();
    auto milliseconds = [](auto duration) {
        return std::chrono::duration<double, std::milli>(duration).count();
    };
    lastStepStats_.collisionDetectionMs = milliseconds(detectionEnd - detectionStart);
    lastStepStats_.solverMs = milliseconds(solverEnd - solverStart);
    lastStepStats_.ccdMs = milliseconds(ccdEnd - solverEnd);
    lastStepStats_.setupMs = milliseconds(detectionStart - stepStart) +
                             milliseconds(solverStart - detectionEnd);
    lastStepStats_.finalizeMs = milliseconds(stepEnd - ccdEnd);
    lastStepStats_.totalMs = milliseconds(stepEnd - stepStart);
    lastStepStats_.broadPhaseMs = milliseconds(broadPhaseEnd - broadPhaseStart);
    lastStepStats_.narrowPhaseMs = milliseconds(detectionEnd - broadPhaseEnd);
    lastStepStats_.contactSolverMs = contactSolverAccum;
    lastStepStats_.jointSolverMs = jointSolverAccum;
    lastStepStats_.ccdRecoveryMs = lastStepStats_.ccdMs;
    lastStepStats_.islandCount = backend_->lastIslandCount();

    // Publish pool accounting for this step so callers can observe allocation
    // overhead (and its absence) without a separate query.
    {
        MemoryPoolStats memStats = memoryPool_.stats();
        size_t reserved = memStats.reservedBytes;
        size_t used = memStats.usedBytes;
        size_t requested = memStats.requestedBytes;
        size_t peak = memStats.peakUsedBytes;
        reserved += bodies_.capacity() * sizeof(Body);
        reserved += contacts_.capacity() * sizeof(Contact);
        reserved += prevContacts_.capacity() * sizeof(Contact);
        reserved += joints_.capacity() * sizeof(Joint);
        used += bodies_.size() * sizeof(Body);
        used += contacts_.size() * sizeof(Contact);
        used += prevContacts_.size() * sizeof(Contact);
        used += joints_.size() * sizeof(Joint);
        requested += bodies_.size() * sizeof(Body);
        requested += contacts_.size() * sizeof(Contact);
        requested += prevContacts_.size() * sizeof(Contact);
        requested += joints_.size() * sizeof(Joint);
        peak += bodies_.size() * sizeof(Body);
        lastStepStats_.memoryReservedBytes = reserved;
        lastStepStats_.memoryUsedBytes = used;
        lastStepStats_.memoryPeakBytes = peak;
        lastStepStats_.memoryFragmentation =
            used > 0 ? 1.0 - double(requested) / double(used) : 0.0;
        lastStepStats_.bodyCapacity = bodies_.capacity();
        lastStepStats_.contactCapacity = contacts_.capacity();
        lastStepStats_.jointCapacity = joints_.capacity();
    }

    for (size_t i = 0; i < bodies_.size(); ++i) {
        const Body& b = bodies_[i];
        if (!b.isDynamic()) continue;
        uint32_t slot = bodyDenseToSlot_[i];
        BodyId id = BodyId::make(slot, bodySlots_[slot].generation);
        bodyEvents_.push_back({BodyEventType::Moved, id, b.position, b.orientation});
    }
}

} // namespace velox
