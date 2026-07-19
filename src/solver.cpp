#include "narrowphase.h"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <exception>
#include <functional>
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
        if (desiredWorkers_ == 1 || taskCount == 1) {
            for (size_t i = 0; i < taskCount; ++i) task(i);
            return;
        }
        ensureStarted();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            task_ = std::move(task);
            taskCount_ = taskCount;
            nextTask_.store(0);
            remaining_.store(static_cast<uint32_t>(threads_.size() + 1));
            exception_ = nullptr;
            ++generation_;
        }
        wake_.notify_all();
        runTasks();
        finishParticipant();
        std::unique_lock<std::mutex> lock(mutex_);
        done_.wait(lock, [&] { return remaining_.load() == 0; });
        std::exception_ptr exception = exception_;
        task_ = {};
        lock.unlock();
        if (exception) std::rethrow_exception(exception);
    }

private:
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
            {
                std::unique_lock<std::mutex> lock(mutex_);
                wake_.wait(lock, [&] {
                    return stopping_ || generation_ != seenGeneration;
                });
                if (stopping_) return;
                seenGeneration = generation_;
            }
            runTasks();
            finishParticipant();
        }
    }

    void runTasks() {
        for (;;) {
            size_t index = nextTask_.fetch_add(1);
            if (index >= taskCount_) return;
            try {
                task_(index);
            } catch (...) {
                std::lock_guard<std::mutex> lock(mutex_);
                if (!exception_) exception_ = std::current_exception();
                nextTask_.store(taskCount_);
                return;
            }
        }
    }

    void finishParticipant() {
        if (remaining_.fetch_sub(1) == 1) done_.notify_one();
    }

    std::vector<std::thread> threads_;
    std::mutex mutex_;
    std::condition_variable wake_, done_;
    std::function<void(size_t)> task_;
    std::exception_ptr exception_;
    std::atomic<size_t> nextTask_{0};
    std::atomic<uint32_t> remaining_{0};
    size_t taskCount_ = 0;
    uint64_t generation_ = 0;
    bool stopping_ = false;
    uint32_t desiredWorkers_ = 1;
};

} // namespace

// CPU reference backend. The narrow phase is the shared VELOX_HD code in
// narrowphase.h — identical to what the CUDA kernels run.
class CpuBackend final : public Backend {
public:
    const char* name() const override { return "cpu"; }

    void setWorkerCount(uint32_t count) override { workers_.configure(count); }
    uint32_t workerCount() const override { return workers_.workerCount(); }

    void integrate(std::vector<Body>& bodies, const Vec3& gravity, float dt) override {
        dispatchChunks(bodies.size(), 512, [&](size_t begin, size_t end) {
            for (size_t i = begin; i < end; ++i) {
                Body& b = bodies[i];
                if (!b.isDynamic() || b.isLocked() || b.asleep) continue;
                b.velocity += (gravity * b.gravityScale +
                               b.force * b.solverInvMass()) * dt;
                b.angularVelocity += b.invInertiaMul(b.torque) * dt;
                b.velocity *= 1.0f / (1.0f + b.linearDamping * dt);
                b.angularVelocity *= 1.0f / (1.0f + b.angularDamping * dt);
            }
        });
    }

    void setParallelIslands(bool enabled) override { parallelIslands_ = enabled; }

    void parallelChunks(size_t items, size_t minPerChunk,
                        const std::function<void(size_t, size_t, size_t)>& fn,
                        size_t* chunkCountOut) override {
        size_t chunks = chunkCount(items, minPerChunk);
        if (chunkCountOut) *chunkCountOut = chunks;
        if (chunks == 0) return;
        workers_.parallelFor(chunks, [&](size_t chunk) {
            fn(chunk, items * chunk / chunks, items * (chunk + 1) / chunks);
        });
    }

    void solveVelocities(std::vector<Body>& bodies,
                         std::vector<Contact>& contacts, float dt,
                         bool warmStart) override {
        if (contacts.empty()) return;
        // Elliptical anisotropic limits couple the two tangent rows. The GPU
        // path already runs two sweeps per base iteration for graph coloring;
        // match that convergence budget on CPU only when the directional
        // limits differ, without taxing ordinary isotropic contacts.
        const bool hasAnisotropicContact = std::any_of(
            contacts.begin(), contacts.end(), [](const Contact& contact) {
                return std::fabs(contact.friction1 - contact.friction2) > 1e-6f;
            });
        const int velocityIterations = hasAnisotropicContact
            ? 2 * kVelocityIterations : kVelocityIterations;
        if (workers_.workerCount() == 1) {
            if (warmStart)
                for (Contact& c : contacts)
                    if (!bodies[c.a].isSensor() && !bodies[c.b].isSensor())
                        warmStartContact(bodies[c.a], bodies[c.b], c);
            for (int iter = 0; iter < velocityIterations; ++iter)
                for (Contact& c : contacts)
                    if (!bodies[c.a].isSensor() && !bodies[c.b].isSensor())
                        solveContact(bodies[c.a], bodies[c.b], c, dt);
            return;
        }

        // Independent contact islands solve concurrently with each island's
        // contacts in their global sequential order. Islands share no dynamic
        // bodies, so this is BITWISE identical to single-threaded solving —
        // unlike the conflict-batch fallback below, which reorders impulses.
        // Islands are rebuilt on the first substep (contacts are stable
        // within a step) and reused afterwards.
        if (parallelIslands_) {
            if (warmStart || islandOfContact_.size() != contacts.size())
                buildIslands(bodies, contacts);
            if (islandRanges_.size() >= 2) {
                workers_.parallelFor(islandRanges_.size(), [&](size_t island) {
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
                                solveContact(bodies[c.a], bodies[c.b], c, dt);
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
            for (const SolverBatch& batch : solverBatches_) {
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
        for (int iteration = 0; iteration < velocityIterations; ++iteration)
            runBatches([&](Contact& contact) {
                solveContact(bodies[contact.a], bodies[contact.b], contact, dt);
            });
    }

    bool wantsHostPairs() const override { return true; }

    void findContacts(const std::vector<Body>& bodies, const MeshSoup& meshes,
                      float dt, const std::vector<uint64_t>* hostPairs,
                      std::vector<Contact>& out) override {
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

            auto inert = [&](BodyIndex k) { return bodies[k].isStatic() || bodies[k].asleep; };

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
    void buildIslands(const std::vector<Body>& bodies,
                      const std::vector<Contact>& contacts) {
        islandParent_.resize(bodies.size());
        for (uint32_t i = 0; i < islandParent_.size(); ++i) islandParent_[i] = i;
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
        islandIdOfRoot_.assign(bodies.size(), UINT32_MAX);
        islandSizes_.clear();
        islandOfContact_.resize(contacts.size());
        for (size_t i = 0; i < contacts.size(); ++i) {
            const Contact& c = contacts[i];
            uint32_t representative = bodies[c.a].isDynamic() ? c.a : c.b;
            uint32_t root = find(representative);
            uint32_t id = islandIdOfRoot_[root];
            if (id == UINT32_MAX) {
                id = (uint32_t)islandSizes_.size();
                islandIdOfRoot_[root] = id;
                islandSizes_.push_back(0);
            }
            islandOfContact_[i] = id;
            ++islandSizes_[id];
        }

        // Bucket contact indices by island (counting sort keeps global order).
        const size_t islandCount = islandSizes_.size();
        islandRanges_.resize(islandCount);
        size_t offset = 0;
        for (size_t island = 0; island < islandCount; ++island) {
            islandRanges_[island] = {offset, offset};
            offset += islandSizes_[island];
        }
        islandContacts_.resize(contacts.size());
        for (size_t i = 0; i < contacts.size(); ++i) {
            IslandRange& range = islandRanges_[islandOfContact_[i]];
            islandContacts_[range.end++] = (uint32_t)i;
        }
        std::sort(islandRanges_.begin(), islandRanges_.end(),
                  [](const IslandRange& x, const IslandRange& y) {
                      size_t sx = x.end - x.begin, sy = y.end - y.begin;
                      return sx != sy ? sx > sy : x.begin < y.begin;
                  });
    }

    void buildSolverBatches(const std::vector<Body>& bodies,
                            const std::vector<Contact>& contacts) {
        solverBatches_.clear();
        solverBodyStamp_.assign(bodies.size(), 0);
        uint32_t stamp = 1;
        size_t begin = 0;
        for (size_t i = 0; i < contacts.size(); ++i) {
            const Contact& contact = contacts[i];
            bool active = !bodies[contact.a].isSensor() &&
                          !bodies[contact.b].isSensor();
            bool useA = active && bodies[contact.a].isDynamic();
            bool useB = active && bodies[contact.b].isDynamic();
            bool conflict = (useA && solverBodyStamp_[contact.a] == stamp) ||
                            (useB && solverBodyStamp_[contact.b] == stamp);
            if (conflict) {
                solverBatches_.push_back({begin, i});
                begin = i;
                ++stamp;
                if (stamp == 0) {
                    std::fill(solverBodyStamp_.begin(), solverBodyStamp_.end(), 0);
                    stamp = 1;
                }
            }
            if (useA) solverBodyStamp_[contact.a] = stamp;
            if (useB) solverBodyStamp_[contact.b] = stamp;
        }
        solverBatches_.push_back({begin, contacts.size()});
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
        workers_.parallelFor(chunks, [&](size_t chunk) {
            function(items * chunk / chunks, items * (chunk + 1) / chunks);
        });
    }

    struct Aabb { Vec3 lo, hi; };
    WorkerPool workers_;
    bool parallelIslands_ = true;
    std::vector<uint32_t> islandParent_;
    std::vector<uint32_t> islandIdOfRoot_;
    std::vector<uint32_t> islandOfContact_;
    std::vector<size_t> islandSizes_;
    std::vector<uint32_t> islandContacts_;
    std::vector<IslandRange> islandRanges_;
    std::vector<Aabb> aabbs_;
    std::vector<BodyIndex> sorted_;
    std::vector<BodyIndex> boundless_;
    std::vector<uint64_t> pairs_;
    std::vector<std::vector<Contact>> chunkContacts_;
    std::vector<SolverBatch> solverBatches_;
    std::vector<uint32_t> solverBodyStamp_;
    size_t solverBatchContactCount_ = SIZE_MAX;
};

Backend* createCpuBackend() { return new CpuBackend(); }

#if !VELOX_HAS_CUDA
Backend* createCudaBackend() { return nullptr; }
#endif

} // namespace velox
