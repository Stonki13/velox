#include "narrowphase.h"
#include "velox/arena.h"
#include "velox/profiler.h"
#include "velox/simd.h"
#include "velox/sleep.h"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

namespace velox {

namespace {

// The dynamic tree deliberately uses fat leaves, so separated spheres can be
// broad-phase candidates. For non-coincident sphere cores, generic GJK takes
// a single point-simplex iteration and produces this analytic result exactly.
// Preserve the shared GJK fallback for the near-coincident core case, where
// its deterministic fallback normal is part of the engine's behavior.
inline bool collideSpherePair(const Body& a, const Body& b, BodyIndex ia,
                              BodyIndex ib, float dt, Contact* out, int cap,
                              int& count) {
    Vec3 delta = a.position - b.position;
    float distanceSq = dot(delta, delta);
    if (distanceSq <= 1e-10f) return false;

    float travel = (a.maxPointSpeed() + b.maxPointSpeed()) * dt + 1e-3f;
    float maximumDistance = a.radius + b.radius + travel;
    // Keep a round-off band on the slow path: its generic GJK result is the
    // authoritative decision for pairs close to speculative-contact reach.
    float safeDistance = maximumDistance + 1e-5f * vmax(1.0f, maximumDistance);
    if (distanceSq > safeDistance * safeDistance) {
        count = 0;
        return true;
    }

    float distance = sqrtf(distanceSq);
    Vec3 normal = delta * (1.0f / distance);
    float gap = distance - a.radius - b.radius;
    Vec3 pointA = a.position - normal * a.radius;
    Vec3 pointB = b.position + normal * b.radius;
    count = 0;
    np_detail::emit(a, b, ia, ib, normal, (pointA + pointB) * 0.5f, gap,
                     dt, out, cap, count);
    return true;
}

class WorkerPool {
public:
    WorkerPool() : desiredWorkers_(resolveCount(0)) {}
    ~WorkerPool() { stop(); }

    void configure(uint32_t requested) {
        uint32_t total = resolveCount(requested);
        if (total == desiredWorkers_) return;
        stop();
        desiredWorkers_ = total;
        stopping_ = false;
    }

    uint32_t workerCount() const { return desiredWorkers_; }

    void parallelFor(size_t taskCount, std::function<void(size_t)> task) {
        if (taskCount == 0) return;
        if (activePool_ == this || desiredWorkers_ == 1 || taskCount == 1) {
            for (size_t i = 0; i < taskCount; ++i) task(i);
            return;
        }
        ensureStarted();
        auto job = std::make_shared<Job>(std::move(task), taskCount,
                                         static_cast<uint32_t>(threads_.size() + 1));
        {
            std::lock_guard<std::mutex> lock(mutex_);
            job_ = job;
            ++generation_;
        }
        wake_.notify_all();
        runTasks(*job);
        finishParticipant(*job);
        std::unique_lock<std::mutex> lock(job->doneMutex);
        while (job->remaining.load() != 0) {
            if (job->done.wait_for(lock, std::chrono::milliseconds(100)) ==
                std::cv_status::timeout) {
                // A worker can begin waiting just as a dispatch is published
                // on some platforms. Re-notify rather than turning that rare
                // rendezvous miss into a permanently stalled simulation.
                lock.unlock();
                wake_.notify_all();
                lock.lock();
            }
        }
        std::exception_ptr exception;
        {
            std::lock_guard<std::mutex> exceptionLock(job->exceptionMutex);
            exception = job->exception;
        }
        if (exception) std::rethrow_exception(exception);
    }

private:
    struct Job {
        Job(std::function<void(size_t)> taskIn, size_t count, uint32_t participants)
            : task(std::move(taskIn)), taskCount(count), remaining(participants) {}

        std::function<void(size_t)> task;
        const size_t taskCount;
        std::atomic<size_t> nextTask{0};
        std::atomic<uint32_t> remaining;
        std::mutex exceptionMutex;
        std::exception_ptr exception;
        std::mutex doneMutex;
        std::condition_variable done;
    };

    class ActivePoolScope {
    public:
        explicit ActivePoolScope(WorkerPool* pool) : previous_(activePool_) {
            activePool_ = pool;
        }
        ~ActivePoolScope() { activePool_ = previous_; }

    private:
        WorkerPool* previous_;
    };

    static uint32_t resolveCount(uint32_t requested) {
        uint32_t total = requested;
        if (total == 0) total = std::thread::hardware_concurrency();
        if (total == 0) total = 1;
        return std::min(total, 64u);
    }

    void ensureStarted() {
        if (threads_.size() + 1 == desiredWorkers_) return;
        stopping_ = false;
        const uint64_t initialGeneration = generation_;
        threads_.reserve(desiredWorkers_ - 1);
        for (uint32_t i = 1; i < desiredWorkers_; ++i)
            threads_.emplace_back([this, initialGeneration] {
                workerLoop(initialGeneration);
            });
    }

    void stop() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
            ++generation_;
        }
        wake_.notify_all();
        for (std::thread& thread : threads_)
            if (thread.joinable()) thread.join();
        threads_.clear();
    }

    void workerLoop(uint64_t seenGeneration) {
        for (;;) {
            std::shared_ptr<Job> job;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                wake_.wait(lock, [&] {
                    return stopping_ || generation_ != seenGeneration;
                });
                if (stopping_) return;
                seenGeneration = generation_;
                job = job_;
            }
            runTasks(*job);
            finishParticipant(*job);
        }
    }

    void runTasks(Job& job) {
        ActivePoolScope scope(this);
        for (;;) {
            size_t index = job.nextTask.fetch_add(1);
            if (index >= job.taskCount) return;
            try {
                job.task(index);
            } catch (...) {
                std::lock_guard<std::mutex> lock(job.exceptionMutex);
                if (!job.exception) job.exception = std::current_exception();
                job.nextTask.store(job.taskCount);
                return;
            }
        }
    }

    void finishParticipant(Job& job) {
        if (job.remaining.fetch_sub(1) == 1) job.done.notify_one();
    }

    std::vector<std::thread> threads_;
    std::mutex mutex_;
    std::condition_variable wake_;
    std::shared_ptr<Job> job_;
    uint64_t generation_ = 0;
    bool stopping_ = false;
    uint32_t desiredWorkers_ = 1;
    static thread_local WorkerPool* activePool_;
};

thread_local WorkerPool* WorkerPool::activePool_ = nullptr;

} // namespace

// CPU reference backend. The narrow phase is the shared VELOX_HD code in
// narrowphase.h — identical to what the CUDA kernels run.
class CpuBackend final : public Backend {
public:
    const char* name() const override { return "cpu"; }

    void setWorkerCount(uint32_t count) override { workers_.configure(count); }
    uint32_t workerCount() const override { return workers_.workerCount(); }
    uint32_t lastVelocityIterations() const override { return lastVelocityIterations_; }
    size_t lastIslandCount() const override { return lastIslandCount_; }

    void integrate(std::vector<Body>& bodies, const Vec3& gravity, float dt) override {
        dispatchChunks(bodies.size(), 512, [&](size_t begin, size_t end) {
#if VELOX_SIMD_AVAILABLE
            // SIMD-accelerated integration: vector math uses 4-wide ops and
            // the inverse-inertia multiply reuses a precomputed rotation matrix
            // instead of two quaternion rotations per body.
            const simd::Vec4 grav = simd::v4from3(gravity);
            const simd::Vec4 vdt  = simd::v4set(dt);
            for (size_t i = begin; i < end; ++i) {
                Body& b = bodies[i];
                if (!b.isDynamic() || b.isLocked() || isFullyAsleep(b.asleep)) continue;

                // velocity += (gravity * gravityScale + force * invMass) * dt
                simd::Vec4 vel   = simd::v4from3(b.velocity);
                simd::Vec4 gScaled = simd::v4scale(grav, b.gravityScale);
                simd::Vec4 fScaled = simd::v4scale(simd::v4from3(b.force), b.solverInvMass());
                vel = simd::v4add(vel, simd::v4mul(simd::v4add(gScaled, fScaled), vdt));

                if (!b.fixedRotation) {
                    // angularVelocity += I⁻¹ * torque * dt
                    simd::InertiaTensor<Body> tensor;
                    tensor.init(b);
                    simd::Vec4 angAccel = tensor.mul(b.torque);
                    simd::Vec4 angVel = simd::v4from3(b.angularVelocity);
                    angVel = simd::v4add(angVel, simd::v4scale(angAccel, dt));
                    float angDamp = 1.0f / (1.0f + b.angularDamping * dt);
                    angVel = simd::v4scale(angVel, angDamp);
                    b.angularVelocity = {angVel.x, angVel.y, angVel.z};
                }

                float linDamp = 1.0f / (1.0f + b.linearDamping * dt);
                vel = simd::v4scale(vel, linDamp);
                b.velocity = {vel.x, vel.y, vel.z};
            }
#else
            for (size_t i = begin; i < end; ++i) {
                Body& b = bodies[i];
                if (!b.isDynamic() || b.isLocked() || isFullyAsleep(b.asleep)) continue;
                b.velocity += (gravity * b.gravityScale +
                               b.force * b.solverInvMass()) * dt;
                if (!b.fixedRotation) {
                    b.angularVelocity += b.invInertiaMul(b.torque) * dt;
                    b.angularVelocity *= 1.0f / (1.0f + b.angularDamping * dt);
                }
                b.velocity *= 1.0f / (1.0f + b.linearDamping * dt);
            }
#endif
        });
    }

    void setParallelIslands(bool enabled) override { parallelIslands_ = enabled; }

    void setTaskSystem(TaskSystem* system) override { externalTasks_ = system; }

    void parallelChunks(size_t items, size_t minPerChunk,
                        const std::function<void(size_t, size_t, size_t)>& fn,
                        size_t* chunkCountOut) override {
        size_t chunks = chunkCount(items, minPerChunk);
        if (chunkCountOut) *chunkCountOut = chunks;
        if (chunks == 0) return;
        if (externalTasks_) {
            externalTasks_->parallelFor(0, chunks, 1, [&](size_t begin, size_t end) {
                for (size_t chunk = begin; chunk < end; ++chunk)
                    fn(chunk, items * chunk / chunks, items * (chunk + 1) / chunks);
            });
            return;
        }
        workers_.parallelFor(chunks, [&](size_t chunk) {
            fn(chunk, items * chunk / chunks, items * (chunk + 1) / chunks);
        });
    }

    void solveVelocities(std::vector<Body>& bodies,
                         std::vector<Contact>& contacts, float dt,
                          bool warmStart,
                          const SolverOptions& options) override {
        VELOX_PROFILE_CATEGORY_SCOPE("ConstraintSolve",
                                     velox::profile::Category::ConstraintSolve);
        lastVelocityIterations_ = 0;
        lastIslandCount_ = 0;
        if (contacts.empty()) return;
        // Reclaim per-frame scratch (island arrays, solver batches) on the
        // first substep. Subsequent substeps reuse the same arena data.
        if (warmStart) arena_.reset();
        lastIslandCount_ = 1;
        // Elliptical anisotropic limits couple the two tangent rows. The GPU
        // path already runs two sweeps per base iteration for graph coloring;
        // match that convergence budget on CPU only when the directional
        // limits differ, without taxing ordinary isotropic contacts.
        const bool hasAnisotropicContact = std::any_of(
            contacts.begin(), contacts.end(), [](const Contact& contact) {
                return std::fabs(contact.friction1 - contact.friction2) > 1e-6f;
            });
        const int velocityIterations = hasAnisotropicContact &&
                options.frictionModel == FrictionModel::TwoAxisCoulomb
            ? 2 * options.velocityIterations : options.velocityIterations;
        // Adaptive convergence is measured after each complete ordered pass.
        // Parallel conflict batches deliberately change that order, so use the
        // reference pass rather than reporting a nondeterministic early-out.
        if (workers_.workerCount() == 1 ||
            options.iterationPolicy == IterationPolicy::Adaptive) {
            if (warmStart)
                for (Contact& c : contacts)
                    if (!bodies[c.a].isSensor() && !bodies[c.b].isSensor())
                        warmStartContact(bodies[c.a], bodies[c.b], c);
            for (int iter = 0; iter < velocityIterations; ++iter) {
                float maxImpulseChange = 0.0f;
                for (Contact& c : contacts)
                    if (!bodies[c.a].isSensor() && !bodies[c.b].isSensor())
                        maxImpulseChange = vmax(maxImpulseChange,
                            solveContact(bodies[c.a], bodies[c.b], c, dt, options));
                ++lastVelocityIterations_;
                if (options.iterationPolicy == IterationPolicy::Adaptive &&
                    maxImpulseChange < options.impulseThreshold)
                    break;
            }
            return;
        }

        // Independent contact islands solve concurrently with each island's
        // contacts in their global sequential order. Islands share no dynamic
        // bodies, so this is BITWISE identical to single-threaded solving —
        // unlike the conflict-batch fallback below, which reorders impulses.
        // Islands are rebuilt on the first substep (contacts are stable
        // within a step) and reused afterwards.
        if (parallelIslands_) {
            if (warmStart || islandContactCount_ != contacts.size())
                buildIslands(bodies, contacts);
            lastIslandCount_ = islandCount_;
            if (islandCount_ >= 2) {
                lastVelocityIterations_ = static_cast<uint32_t>(velocityIterations);
                workers_.parallelFor(islandCount_, [&](size_t island) {
                    const IslandRange range = islandRanges_[island];
                    if (warmStart)
                        for (size_t k = range.begin; k < range.end; ++k) {
                            Contact& c = contacts[islandContacts_[k]];
                            if (!bodies[c.a].isSensor() && !bodies[c.b].isSensor())
                                warmStartContact(bodies[c.a], bodies[c.b], c);
                        }
                    for (int iter = 0; iter < velocityIterations; ++iter)
                        for (size_t k = range.begin; k < range.end; ++k) {
                            Contact& c = contacts[islandContacts_[k]];
                            if (!bodies[c.a].isSensor() && !bodies[c.b].isSensor())
                                solveContact(bodies[c.a], bodies[c.b], c, dt, options);
                        }
                });
                return;
            }
            // A single island (one big pile) has no island-level parallelism;
            // fall through to conflict-free batches.
        }

        // Contacts are stable across a step's substeps; rebuild the batches
        // only on the first substep instead of once per substep.
        if (warmStart || solverBatchContactCount_ != contacts.size()) {
            buildSolverBatches(bodies, contacts);
            solverBatchContactCount_ = contacts.size();
        }
        auto runBatches = [&](auto&& solve) {
            for (size_t bi = 0; bi < solverBatchCount_; ++bi) {
                const SolverBatch& batch = solverBatches_[bi];
                size_t count = batch.end - batch.begin;
                dispatchChunks(count, 64, [&](size_t begin, size_t end) {
                    for (size_t offset = begin; offset < end; ++offset) {
                        Contact& contact = contacts[batch.begin + offset];
                        if (!bodies[contact.a].isSensor() &&
                            !bodies[contact.b].isSensor())
                            solve(contact);
                    }
                });
            }
        };
        if (warmStart)
            runBatches([&](Contact& contact) {
                warmStartContact(bodies[contact.a], bodies[contact.b], contact);
            });
        lastVelocityIterations_ = static_cast<uint32_t>(velocityIterations);
        for (int iteration = 0; iteration < velocityIterations; ++iteration)
            runBatches([&](Contact& contact) {
                solveContact(bodies[contact.a], bodies[contact.b], contact, dt, options);
            });
    }

    bool wantsHostPairs() const override { return true; }

    void findContacts(const std::vector<Body>& bodies, const MeshSoup& meshes,
                      float dt, const std::vector<uint64_t>* hostPairs,
                      std::vector<Contact>& out) override {
        VELOX_PROFILE_CATEGORY_SCOPE("Narrowphase",
                                     velox::profile::Category::Narrowphase);
        out.clear();
        const MeshSoupView soup = view(meshes);
        const size_t n = bodies.size();

        if (hostPairs) {
            // The World's incremental AABB tree already produced candidates.
            pairs_ = *hostPairs;
        } else {
            // Fallback broad phase: full sweep-and-prune rebuild. Planes are
            // truly unbounded and tested directly; meshes use root bounds.
            boundless_.clear();
            sorted_.clear();
            aabbs_.resize(n);
            for (BodyIndex i = 0; i < n; ++i) {
                const Body& b = bodies[i];
                if (b.shape == ShapeType::Plane)
                    boundless_.push_back(i);
                else {
                    if (b.shape == ShapeType::Mesh) {
                        const Mesh& mesh = meshes.meshes[b.meshIndex];
                        aabbs_[i] = {mesh.aabbMin, mesh.aabbMax};
                    } else {
                        bodyAabb(b, dt, aabbs_[i].lo, aabbs_[i].hi);
                    }
                    sorted_.push_back(i);
                }
            }

            auto inert = [&](BodyIndex k) { return bodies[k].isStatic() || isFullyAsleep(bodies[k].asleep); };

            pairs_.clear();
            for (BodyIndex i : sorted_)
                if (!inert(i))
                    for (BodyIndex j : boundless_)
                        if (bodies[i].canCollideWith(bodies[j]))
                            pairs_.push_back((uint64_t)i << 32 | j);

            // Sweep-and-prune along X: sort by min, only test while overlapping.
            std::sort(sorted_.begin(), sorted_.end(), [&](BodyIndex a, BodyIndex b) {
                return aabbs_[a].lo.x < aabbs_[b].lo.x;
            });
            for (size_t si = 0; si < sorted_.size(); ++si) {
                BodyIndex i = sorted_[si];
                for (size_t sj = si + 1; sj < sorted_.size(); ++sj) {
                    BodyIndex j = sorted_[sj];
                    if (aabbs_[j].lo.x > aabbs_[i].hi.x) break; // pruned: sorted axis
                    if (inert(i) && inert(j)) continue;
                    if (!bodies[i].canCollideWith(bodies[j])) continue;
                    if (!aabbOverlap(aabbs_[i].lo, aabbs_[i].hi, aabbs_[j].lo, aabbs_[j].hi))
                        continue;
                    pairs_.push_back((uint64_t)i << 32 | j);
                }
            }
        }

        size_t chunks = chunkCount(pairs_.size(), 256);
        chunkContacts_.resize(chunks);
        workers_.parallelFor(chunks, [&](size_t chunk) {
            size_t begin = pairs_.size() * chunk / chunks;
            size_t end = pairs_.size() * (chunk + 1) / chunks;
            std::vector<Contact>& contacts = chunkContacts_[chunk];
            contacts.clear();
            Contact buffer[kMaxContactsPerPair];
            for (size_t p = begin; p < end; ++p) {
                BodyIndex i = BodyIndex(pairs_[p] >> 32);
                BodyIndex j = BodyIndex(pairs_[p]);
                int count = 0;
                if (bodies[i].shape != ShapeType::Sphere ||
                    bodies[j].shape != ShapeType::Sphere ||
                    !collideSpherePair(bodies[i], bodies[j], i, j, dt, buffer,
                                       kMaxContactsPerPair, count))
                    count = collidePair(bodies[i], bodies[j], i, j, soup, dt,
                                        buffer, kMaxContactsPerPair);
                contacts.insert(contacts.end(), buffer, buffer + count);
            }
        });
        size_t contactCount = 0;
        for (const auto& contacts : chunkContacts_) contactCount += contacts.size();
        out.reserve(contactCount);
        for (const auto& contacts : chunkContacts_)
            out.insert(out.end(), contacts.begin(), contacts.end());
    }

private:
    struct SolverBatch { size_t begin, end; };
    struct IslandRange { size_t begin, end; };

    // Union-find over dynamic bodies connected by contacts, then bucket the
    // contact indices by island preserving global contact order (so each
    // island's sequential solve matches the single-threaded reference).
    // Islands are emitted largest-first for worker load balance; ordering
    // between islands cannot affect results because they are disjoint.
    // All scratch arrays are bump-allocated from the per-frame arena.
    void buildIslands(const std::vector<Body>& bodies,
                      const std::vector<Contact>& contacts) {
        VELOX_PROFILE_CATEGORY_SCOPE("IslandBuild",
                                     velox::profile::Category::Island);
        const size_t bodyCount = bodies.size();
        const size_t contactCount = contacts.size();

        // Carve scratch arrays out of the arena (already reset at frame start).
        islandParent_ = arena_.allocateArray<uint32_t>(bodyCount);
        islandIdOfRoot_ = arena_.allocateArray<uint32_t>(bodyCount);
        islandOfContact_ = arena_.allocateArray<uint32_t>(contactCount);
        islandSizes_ = arena_.allocateArray<size_t>(bodyCount);
        islandContacts_ = arena_.allocateArray<uint32_t>(contactCount);
        islandRanges_ = arena_.allocateArray<IslandRange>(bodyCount);
        
        // Check for allocation failure (arena exhausted)
        if (!islandParent_ || !islandIdOfRoot_ || !islandOfContact_ ||
            !islandSizes_ || !islandContacts_ || !islandRanges_) {
            // Fall back to single-island solving
            lastIslandCount_ = 1;
            return;
        }
        islandContactCount_ = contactCount;

        for (uint32_t i = 0; i < bodyCount; ++i) islandParent_[i] = i;
        auto find = [&](uint32_t x) {
            while (islandParent_[x] != x) {
                islandParent_[x] = islandParent_[islandParent_[x]];
                x = islandParent_[x];
            }
            return x;
        };
        for (const Contact& c : contacts)
            if (bodies[c.a].isDynamic() && bodies[c.b].isDynamic()) {
                uint32_t ra = find(c.a), rb = find(c.b);
                if (ra != rb) islandParent_[ra] = rb;
            }

        // Map island roots to dense ids in first-appearance order.
        std::memset(islandIdOfRoot_, 0xFF, bodyCount * sizeof(uint32_t));
        islandCount_ = 0;
        std::memset(islandSizes_, 0, bodyCount * sizeof(size_t));
        for (size_t i = 0; i < contactCount; ++i) {
            const Contact& c = contacts[i];
            uint32_t representative = bodies[c.a].isDynamic() ? c.a : c.b;
            uint32_t root = find(representative);
            uint32_t id = islandIdOfRoot_[root];
            if (id == UINT32_MAX) {
                id = static_cast<uint32_t>(islandCount_);
                islandIdOfRoot_[root] = id;
                ++islandCount_;
            }
            islandOfContact_[i] = id;
            ++islandSizes_[id];
        }

        // Bucket contact indices by island (counting sort keeps global order).
        size_t offset = 0;
        for (size_t island = 0; island < islandCount_; ++island) {
            islandRanges_[island] = {offset, offset};
            offset += islandSizes_[island];
        }
        for (size_t i = 0; i < contactCount; ++i) {
            IslandRange& range = islandRanges_[islandOfContact_[i]];
            islandContacts_[range.end++] = static_cast<uint32_t>(i);
        }
        std::sort(islandRanges_, islandRanges_ + islandCount_,
                  [](const IslandRange& x, const IslandRange& y) {
                      size_t sx = x.end - x.begin, sy = y.end - y.begin;
                      return sx != sy ? sx > sy : x.begin < y.begin;
                  });
    }

    // Conflict-free graph-coloring batches, bump-allocated from the arena.
    void buildSolverBatches(const std::vector<Body>& bodies,
                            const std::vector<Contact>& contacts) {
        const size_t bodyCount = bodies.size();
        const size_t contactCount = contacts.size();

        solverBatches_ = arena_.allocateArray<SolverBatch>(contactCount + 1);
        solverBodyStamp_ = arena_.allocateArray<uint32_t>(bodyCount);
        
        // Check for allocation failure (arena exhausted)
        if (!solverBatches_ || !solverBodyStamp_) {
            // Fall back to sequential solving
            return;
        }
        solverBatchCount_ = 0;

        std::memset(solverBodyStamp_, 0, bodyCount * sizeof(uint32_t));
        uint32_t stamp = 1;
        size_t begin = 0;
        for (size_t i = 0; i < contactCount; ++i) {
            const Contact& contact = contacts[i];
            bool active = !bodies[contact.a].isSensor() &&
                          !bodies[contact.b].isSensor();
            bool useA = active && bodies[contact.a].isDynamic();
            bool useB = active && bodies[contact.b].isDynamic();
            bool conflict = (useA && solverBodyStamp_[contact.a] == stamp) ||
                            (useB && solverBodyStamp_[contact.b] == stamp);
            if (conflict) {
                solverBatches_[solverBatchCount_++] = {begin, i};
                begin = i;
                ++stamp;
                if (stamp == 0) {
                    std::memset(solverBodyStamp_, 0,
                                bodyCount * sizeof(uint32_t));
                    stamp = 1;
                }
            }
            if (useA) solverBodyStamp_[contact.a] = stamp;
            if (useB) solverBodyStamp_[contact.b] = stamp;
        }
        solverBatches_[solverBatchCount_++] = {begin, contactCount};
    }

    size_t chunkCount(size_t items, size_t parallelThreshold) const {
        if (items == 0 || items < parallelThreshold || workers_.workerCount() == 1)
            return items == 0 ? 0 : 1;
        // Tiny chunks cost more in worker synchronization than they save. Cap
        // the fan-out by useful work as well as by available CPU workers.
        constexpr size_t kMinItemsPerChunk = 128;
        size_t usefulChunks = (items + kMinItemsPerChunk - 1) / kMinItemsPerChunk;
        return std::min({items, size_t(workers_.workerCount()), usefulChunks});
    }

    template <typename F>
    void dispatchChunks(size_t items, size_t parallelThreshold, F&& function) {
        size_t chunks = chunkCount(items, parallelThreshold);
        if (externalTasks_) {
            externalTasks_->parallelFor(0, chunks, 1, [&](size_t begin, size_t end) {
                for (size_t chunk = begin; chunk < end; ++chunk)
                    function(items * chunk / chunks, items * (chunk + 1) / chunks);
            });
            return;
        }
        workers_.parallelFor(chunks, [&](size_t chunk) {
            function(items * chunk / chunks, items * (chunk + 1) / chunks);
        });
    }

    struct Aabb { Vec3 lo, hi; };
    WorkerPool workers_;
    TaskSystem* externalTasks_ = nullptr;
    bool parallelIslands_ = true;

    // Per-frame scratch arena: reset at the start of each step, bump-allocated
    // by buildIslands / buildSolverBatches. Eliminates malloc/free churn.
    ArenaAllocator arena_;

    // Island scratch (arena-allocated, valid until next arena reset).
    uint32_t* islandParent_ = nullptr;
    uint32_t* islandIdOfRoot_ = nullptr;
    uint32_t* islandOfContact_ = nullptr;
    size_t* islandSizes_ = nullptr;
    uint32_t* islandContacts_ = nullptr;
    IslandRange* islandRanges_ = nullptr;
    size_t islandCount_ = 0;
    size_t islandContactCount_ = SIZE_MAX;

    std::vector<Aabb> aabbs_;
    std::vector<BodyIndex> sorted_;
    std::vector<BodyIndex> boundless_;
    std::vector<uint64_t> pairs_;
    std::vector<std::vector<Contact>> chunkContacts_;

    // Solver batch scratch (arena-allocated).
    SolverBatch* solverBatches_ = nullptr;
    uint32_t* solverBodyStamp_ = nullptr;
    size_t solverBatchCount_ = 0;
    size_t solverBatchContactCount_ = SIZE_MAX;
    uint32_t lastVelocityIterations_ = 0;
    size_t lastIslandCount_ = 0;
};

Backend* createCpuBackend() { return new CpuBackend(); }

#if !VELOX_HAS_CUDA
Backend* createCudaBackend() { return nullptr; }
#endif

} // namespace velox
