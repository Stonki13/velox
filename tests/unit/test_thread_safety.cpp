#include "doctest.h"
#include <velox/thread_safety.h>
#include <velox/velox.h>

#include <atomic>
#include <thread>
#include <vector>

using namespace velox;

namespace {

// Small RAII helper: join every thread in the vector on scope exit so a
// failing CHECK cannot leak a still-running worker into the next test. Wrap a
// batch of workers in a nested scope; the joins complete when the scope ends,
// before any CHECK that depends on their results.
struct Joiner {
    std::vector<std::thread> threads;
    ~Joiner() {
        for (auto& t : threads) {
            if (t.joinable()) t.join();
        }
    }
};

unsigned hardwareConcurrency() {
    unsigned n = std::thread::hardware_concurrency();
    return n == 0 ? 4 : n;
}

} // namespace

// ===========================================================================
// Atomic / lock-free primitive tests.
//
// These hammer the utilities in thread_safety.h from many threads at once.
// Under Clang ThreadSanitizer (the CI contract regression) any data race in
// the primitives is reported here; in an ordinary build they verify functional
// correctness under contention, which is the observable symptom of a race
// (lost updates, duplicated/lost queue items, torn reads).
// ===========================================================================

TEST_CASE("AtomicCounter has no lost updates under contention") {
    constexpr int kThreads = 8;
    constexpr int kPerThread = 10000;
    AtomicCounter<uint64_t> counter;

    {
        Joiner joiner;
        for (int t = 0; t < kThreads; ++t) {
            joiner.threads.emplace_back([&counter, kPerThread] {
                for (int i = 0; i < kPerThread; ++i) counter.increment();
            });
        }
    } // joins complete here

    CHECK(counter.load() == uint64_t(kThreads) * uint64_t(kPerThread));
}

TEST_CASE("AtomicFlag claim-once returns true for exactly one thread") {
    AtomicFlag flag;
    constexpr int kThreads = 16;
    AtomicCounter<int> winners;
    std::atomic<bool> start{false};

    {
        Joiner joiner;
        for (int t = 0; t < kThreads; ++t) {
            joiner.threads.emplace_back([&] {
                while (!start.load(std::memory_order_acquire)) {
                    Backoff::cpuRelax();
                }
                // testAndSet returns the PREVIOUS value; the first caller sees false.
                if (!flag.testAndSet()) winners.increment();
            });
        }
        start.store(true, std::memory_order_release);
    }

    CHECK(winners.load() == 1);
}

TEST_CASE("SpinLock provides mutual exclusion") {
    SpinLock lock;
    constexpr int kThreads = 8;
    constexpr int kPerThread = 5000;
    uint64_t sharedCounter = 0; // deliberately non-atomic: the lock must protect it

    {
        Joiner joiner;
        for (int t = 0; t < kThreads; ++t) {
            joiner.threads.emplace_back([&] {
                for (int i = 0; i < kPerThread; ++i) {
                    std::lock_guard<SpinLock> guard(lock);
                    ++sharedCounter; // would race without the lock
                }
            });
        }
    }

    CHECK(sharedCounter == uint64_t(kThreads) * uint64_t(kPerThread));
}

TEST_CASE("SpinLock tryLock is non-blocking") {
    SpinLock lock;
    CHECK(lock.tryLock());       // free -> acquired
    CHECK_FALSE(lock.tryLock()); // already held -> fails immediately, no block
    lock.unlock();
    CHECK(lock.tryLock()); // free again
    lock.unlock();
}

TEST_CASE("MutexGuarded serializes concurrent mutation") {
    MutexGuarded<std::vector<int>> guarded;
    constexpr int kThreads = 8;
    constexpr int kPerThread = 1000;

    {
        Joiner joiner;
        for (int t = 0; t < kThreads; ++t) {
            joiner.threads.emplace_back([&guarded, t, kPerThread] {
                for (int i = 0; i < kPerThread; ++i) {
                    guarded.withLock([t, i, kPerThread](std::vector<int>& v) {
                        v.push_back(t * kPerThread + i);
                    });
                }
            });
        }
    }

    auto locked = guarded.lock();
    CHECK(locked->size() == size_t(kThreads) * size_t(kPerThread));
}

TEST_CASE("SpscQueue transfers every item in order") {
    constexpr size_t kCapacity = 64;
    constexpr int kItems = 100000;
    SpscQueue<int, kCapacity> queue;

    std::thread producer([&] {
        for (int i = 0; i < kItems; ++i) {
            while (!queue.tryPush(i)) {
                Backoff::cpuRelax();
            }
        }
    });

    long long sum = 0;
    int received = 0;
    int expected = 0;
    bool ordered = true;
    std::thread consumer([&] {
        int value;
        while (received < kItems) {
            if (queue.tryPop(value)) {
                if (value != expected) ordered = false;
                expected++;
                sum += value;
                received++;
            } else {
                Backoff::cpuRelax();
            }
        }
    });

    producer.join();
    consumer.join();

    CHECK(received == kItems);
    CHECK(ordered);
    CHECK(sum == (long long)kItems * (kItems - 1) / 2);
}

TEST_CASE("MpmcQueue loses no items across many producers and consumers") {
    constexpr size_t kCapacity = 256; // power of two
    constexpr int kProducers = 4;
    constexpr int kConsumers = 4;
    constexpr int kPerProducer = 5000;
    constexpr int kTotal = kProducers * kPerProducer;

    MpmcQueue<int, kCapacity> queue;
    AtomicCounter<int> consumed;
    AtomicCounter<long long> sum;

    {
        Joiner joiner;
        // Producers push the sequence 1..kTotal (partitioned, no duplicates).
        for (int p = 0; p < kProducers; ++p) {
            joiner.threads.emplace_back([&queue, p, kPerProducer] {
                for (int i = 0; i < kPerProducer; ++i) {
                    int value = p * kPerProducer + i + 1;
                    while (!queue.tryPush(value)) {
                        Backoff::cpuRelax();
                    }
                }
            });
        }
        // Consumers stop once exactly kTotal items have been drained in aggregate.
        for (int c = 0; c < kConsumers; ++c) {
            joiner.threads.emplace_back([&] {
                int value;
                while (consumed.load(std::memory_order_relaxed) < kTotal) {
                    if (queue.tryPop(value)) {
                        sum.fetchAdd(value);
                        consumed.increment();
                    } else {
                        Backoff::cpuRelax();
                    }
                }
            });
        }
    }

    CHECK(consumed.load() == kTotal);
    CHECK(sum.load() == (long long)kTotal * (kTotal + 1) / 2);
}

TEST_CASE("ThreadLocal gives each thread its own value") {
    ThreadLocal<int> tls(0);
    constexpr int kThreads = 8;
    constexpr int kPerThread = 1000;
    AtomicCounter<int> threadsSeeingNonZeroAtStart;
    std::atomic<bool> start{false};

    {
        Joiner joiner;
        for (int t = 0; t < kThreads; ++t) {
            joiner.threads.emplace_back([&] {
                // Each thread starts from the initial value 0, independent of others.
                if (tls.getOrCreate() != 0) threadsSeeingNonZeroAtStart.increment();
                while (!start.load(std::memory_order_acquire)) Backoff::cpuRelax();
                for (int i = 0; i < kPerThread; ++i) tls.getOrCreate()++;
            });
        }
        start.store(true, std::memory_order_release);
    }

    CHECK(threadsSeeingNonZeroAtStart.load() == 0); // no cross-thread bleed
    CHECK(tls.threadCount() == size_t(kThreads));

    long long total = 0;
    tls.forEach([&total, kPerThread](int& v) {
        CHECK(v == kPerThread); // each thread accumulated independently
        total += v;
    });
    CHECK(total == (long long)kThreads * kPerThread);
}

TEST_CASE("ThreadLocalAccumulator sums per-thread contributions") {
    ThreadLocalAccumulator<uint64_t> acc;
    constexpr int kThreads = 8;
    constexpr int kPerThread = 10000;

    {
        Joiner joiner;
        for (int t = 0; t < kThreads; ++t) {
            joiner.threads.emplace_back([&acc, kPerThread] {
                for (int i = 0; i < kPerThread; ++i) acc.increment();
            });
        }
    }

    // No contention on a shared counter, yet the total is exact.
    CHECK(acc.total() == uint64_t(kThreads) * uint64_t(kPerThread));
}

// ===========================================================================
// World-level concurrency tests.
// ===========================================================================

TEST_CASE("Concurrent body creation and deletion from many threads") {
    World world(BackendType::Cpu);
    world.setThreadSafetyPolicy(ThreadSafetyPolicy::Concurrent);

    const int threadCount = static_cast<int>(hardwareConcurrency());
    constexpr int kPerThread = 50;

    // Per-thread ID storage avoids any contention on a shared container; the
    // World itself serializes the mutations under the Concurrent policy.
    // (static_cast avoids the most-vexing-parse: a functional cast here would
    // make MSVC read this line as a function declaration.)
    std::vector<std::vector<BodyId>> created(static_cast<size_t>(threadCount));

    // Phase 1: every thread creates bodies concurrently.
    {
        Joiner joiner;
        for (int t = 0; t < threadCount; ++t) {
            joiner.threads.emplace_back([&world, &created, t, kPerThread] {
                for (int i = 0; i < kPerThread; ++i) {
                    BodyId id = world.addSphere(
                        {float(t), float(i), 0.0f}, 0.5f, 1.0f);
                    created[size_t(t)].push_back(id);
                }
            });
        }
    }

    CHECK(world.bodyCount() == size_t(threadCount) * size_t(kPerThread));
    for (auto& ids : created) {
        for (BodyId id : ids) CHECK(world.isValid(id));
    }

    // Phase 2: every thread deletes its own bodies concurrently.
    {
        Joiner joiner;
        for (int t = 0; t < threadCount; ++t) {
            joiner.threads.emplace_back([&world, &created, t] {
                for (BodyId id : created[size_t(t)]) {
                    world.removeBody(id);
                }
            });
        }
    }

    CHECK(world.bodyCount() == 0);
    for (auto& ids : created) {
        for (BodyId id : ids) CHECK_FALSE(world.isValid(id));
    }
}

TEST_CASE("Parallel query execution returns consistent results") {
    World world(BackendType::Cpu);
    world.setThreadSafetyPolicy(ThreadSafetyPolicy::Concurrent);

    // A target body sitting on the +X axis that every ray aims at, plus a cloud
    // of distractors so the broad phase has real work to do.
    BodyId target = world.addSphere({10.0f, 0.0f, 0.0f}, 1.0f, 1.0f);
    for (int i = 0; i < 200; ++i) {
        world.addSphere({float(i % 20) - 10.0f, float(i / 20) - 5.0f, -20.0f},
                        0.5f, 1.0f);
    }

    constexpr int kThreads = 8;
    constexpr int kIters = 500;
    AtomicCounter<int> targetHits;
    AtomicCounter<int> overlapCounts;

    {
        Joiner joiner;
        for (int t = 0; t < kThreads; ++t) {
            joiner.threads.emplace_back([&] {
                for (int i = 0; i < kIters; ++i) {
                    // A ray straight down +X must always hit the target body.
                    RayHit hit = world.rayCast({0.0f, 0.0f, 0.0f},
                                               {1.0f, 0.0f, 0.0f}, 100.0f);
                    if (hit.hit && hit.body == target) targetHits.increment();

                    // An overlap around the target must always find it.
                    std::vector<BodyId> out;
                    world.overlapSphere({10.0f, 0.0f, 0.0f}, 1.5f, out);
                    bool found = false;
                    for (BodyId id : out) {
                        if (id == target) found = true;
                    }
                    if (found) overlapCounts.increment();

                    // A value copy of a body must be readable concurrently too.
                    Body state = world.bodyState(target);
                    CHECK(state.position.x == doctest::Approx(10.0f));
                }
            });
        }
    }

    // Every query from every thread observed the same, complete world state.
    CHECK(targetHits.load() == kThreads * kIters);
    CHECK(overlapCounts.load() == kThreads * kIters);
}

TEST_CASE("Queries do not deadlock against a stepping owner thread") {
    World world(BackendType::Cpu);
    world.setThreadSafetyPolicy(ThreadSafetyPolicy::Concurrent);
    world.setSubsteps(2);

    for (int i = 0; i < 50; ++i) {
        world.addSphere({float(i) * 0.5f, 5.0f, 0.0f}, 0.4f, 1.0f);
    }
    world.addStaticPlane({0.0f, 1.0f, 0.0f}, 0.0f);

    constexpr int kQueryThreads = 4;
    constexpr int kSteps = 120;
    std::atomic<bool> done{false};
    std::atomic<bool> beginQueries{false};
    AtomicCounter<int> queriesCompleted;
    AtomicCounter<int> workersReady;
    AtomicCounter<int> queriesStarted;

    Joiner joiner;
    for (int t = 0; t < kQueryThreads; ++t) {
        joiner.threads.emplace_back([&] {
            workersReady.increment();
            while (!beginQueries.load(std::memory_order_acquire)) {
                Backoff::cpuRelax();
            }
            while (!done.load(std::memory_order_acquire)) {
                queriesStarted.increment();
                // These calls overlap step() and must wait for it rather than
                // deadlock; the world lock is reentrant on the owner side only.
                std::vector<BodyId> out;
                world.overlapSphere({0.0f, 5.0f, 0.0f}, 100.0f, out);
                world.rayCast({0.0f, 20.0f, 0.0f}, {0.0f, -1.0f, 0.0f}, 100.0f);
                queriesCompleted.increment();
            }
        });
    }

    // Owner thread steps; workers query concurrently the whole time.
    while (workersReady.load() != kQueryThreads) Backoff::cpuRelax();
    beginQueries.store(true, std::memory_order_release);
    while (queriesStarted.load() < kQueryThreads) Backoff::cpuRelax();
    for (int s = 0; s < kSteps; ++s) {
        world.step(1.0f / 60.0f);
    }
    done.store(true, std::memory_order_release);
    // Joiner destructor joins the workers here at scope end.

    CHECK(queriesCompleted.load() > 0);
    CHECK(world.bodyCount() == 51); // 50 spheres + plane, none lost
}

TEST_CASE("Async queries from many threads resolve without deadlock") {
    World world(BackendType::Cpu);
    // Async submission is allowed even under Strict, but Concurrent exercises
    // the fuller path where the owner may also be queried/mutated.
    world.setThreadSafetyPolicy(ThreadSafetyPolicy::Concurrent);

    // The test exercises async handoff, so the query target must remain valid
    // while worker scheduling and owner-thread stepping interleave.
    BodyId target = world.addSphere({5.0f, 0.0f, 0.0f}, 1.0f, 1.0f);
    world.setMotionType(target, MotionType::Static);

    constexpr int kThreads = 8;
    std::atomic<int> resolved{0};
    AtomicCounter<int> correctHits;

    Joiner joiner;
    for (int t = 0; t < kThreads; ++t) {
        joiner.threads.emplace_back([&] {
            QueryDesc desc;
            desc.type = QueryDesc::Type::Raycast;
            desc.origin = {0.0f, 0.0f, 0.0f};
            desc.direction = {1.0f, 0.0f, 0.0f};
            desc.maxDist = 100.0f;

            AsyncQueryHandle handle = world.submitAsyncQuery(desc);
            // Blocks until the owner thread resolves it during a later step().
            QueryResult result = world.getAsyncResult(handle);
            if (result.success && result.rayHit.hit && result.rayHit.body == target) {
                correctHits.increment();
            }
            resolved.fetch_add(1, std::memory_order_release);
        });
    }

    // Owner drives resolution. Step until every worker has its answer (or a
    // generous bound is hit, which would indicate a deadlock/lost wakeup).
    int guard = 0;
    while (resolved.load(std::memory_order_acquire) < kThreads && guard < 10000) {
        world.step(1.0f / 60.0f);
        ++guard;
    }
    // Joiner destructor joins the workers here at scope end.

    CHECK(resolved.load() == kThreads);
    CHECK(correctHits.load() == kThreads);
}

TEST_CASE("threadSafetyReport counts cross-thread query calls") {
    World world(BackendType::Cpu);
    world.setThreadSafetyPolicy(ThreadSafetyPolicy::Relaxed);
    world.addSphere({0.0f, 0.0f, 0.0f}, 0.5f, 1.0f);

    constexpr int kThreads = 4;
    constexpr int kIters = 25;
    {
        Joiner joiner;
        for (int t = 0; t < kThreads; ++t) {
            joiner.threads.emplace_back([&] {
                for (int i = 0; i < kIters; ++i) {
                    world.rayCast({0.0f, 5.0f, 0.0f}, {0.0f, -1.0f, 0.0f}, 20.0f);
                }
            });
        }
    }

    // Relaxed permits and serializes foreign queries, and reports how many
    // happened. The count is at least the number we issued.
    ThreadSafetyReport report = world.threadSafetyReport();
    CHECK(report.queryCallsFromNonMainThread >=
          uint64_t(kThreads) * uint64_t(kIters));
}
