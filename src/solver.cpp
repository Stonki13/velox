#include "narrowphase.h"
#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <exception>
#include <functional>
#include <mutex>
#include <thread>

namespace velox {

namespace {

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
                if (!b.isDynamic() || b.asleep) continue;
                b.velocity += (gravity * b.gravityScale +
                               b.force * b.solverInvMass()) * dt;
                b.angularVelocity += b.invInertiaMul(b.torque) * dt;
                b.velocity *= 1.0f / (1.0f + b.linearDamping * dt);
                b.angularVelocity *= 1.0f / (1.0f + b.angularDamping * dt);
            }
        });
    }

    void solveVelocities(std::vector<Body>& bodies,
                         std::vector<Contact>& contacts, float dt,
                         bool warmStart) override {
        if (warmStart)
            for (Contact& c : contacts)
                if (!bodies[c.a].isSensor() && !bodies[c.b].isSensor())
                    warmStartContact(bodies[c.a], bodies[c.b], c);
        for (int iter = 0; iter < kVelocityIterations; ++iter)
            for (Contact& c : contacts)
                if (!bodies[c.a].isSensor() && !bodies[c.b].isSensor())
                    solveContact(bodies[c.a], bodies[c.b], c, dt);
    }

    void findContacts(const std::vector<Body>& bodies, const MeshSoup& meshes,
                      float dt, std::vector<Contact>& out) override {
        out.clear();
        const MeshSoupView soup = view(meshes);
        const size_t n = bodies.size();

        // Planes and meshes are unbounded/large: every dynamic body is tested
        // against them directly. Bounded bodies go through sweep-and-prune.
        boundless_.clear();
        sorted_.clear();
        aabbs_.resize(n);
        for (BodyIndex i = 0; i < n; ++i) {
            const Body& b = bodies[i];
            if (b.shape == ShapeType::Plane || b.shape == ShapeType::Mesh)
                boundless_.push_back(i);
            else {
                bodyAabb(b, dt, aabbs_[i].lo, aabbs_[i].hi);
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

        // Sweep-and-prune along X: sort by AABB min, only test while overlapping.
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
                int count = collidePair(bodies[i], bodies[j], i, j, soup, dt,
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
    size_t chunkCount(size_t items, size_t parallelThreshold) const {
        if (items == 0 || items < parallelThreshold || workers_.workerCount() == 1)
            return items == 0 ? 0 : 1;
        return std::min(items, size_t(workers_.workerCount()) * 4);
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
    std::vector<Aabb> aabbs_;
    std::vector<BodyIndex> sorted_;
    std::vector<BodyIndex> boundless_;
    std::vector<uint64_t> pairs_;
    std::vector<std::vector<Contact>> chunkContacts_;
};

Backend* createCpuBackend() { return new CpuBackend(); }

#if !VELOX_HAS_CUDA
Backend* createCudaBackend() { return nullptr; }
#endif

} // namespace velox
