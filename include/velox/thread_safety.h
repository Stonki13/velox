#pragma once

// Thread-safety utilities for the Velox public API.
//
// This header is host-only (it is not part of the CUDA narrow-phase path) and
// provides four groups of building blocks that gameplay and tooling code can
// use to share data with a World safely:
//
//   1. Compile-time thread-safety annotations (Clang Thread Safety Analysis),
//      which degrade to no-ops on compilers that do not support them.
//   2. Thread-safe wrapper types: SpinLock, Atomic<T>, AtomicFlag,
//      AtomicCounter<T>, and MutexGuarded<T> (a value coupled with its lock).
//   3. Lock-free data structures for hot paths: a wait-free single-producer /
//      single-consumer queue and a lock-free bounded multi-producer /
//      multi-consumer queue (Vyukov's sequence-counter algorithm, ABA-safe).
//   4. Thread-local storage helpers: ThreadLocal<T> and ThreadLocalAccumulator.
//
// Everything here is header-only and depends only on the C++17 standard
// library, so it can be included from any translation unit without linking
// additional Velox objects.

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <utility>

#if defined(_MSC_VER) && (defined(_M_X64) || defined(_M_IX86))
#include <intrin.h>
#endif

// ---------------------------------------------------------------------------
// 1. Thread-safety annotations
// ---------------------------------------------------------------------------
//
// These map onto Clang's Thread Safety Analysis attributes. They are inert
// unless the translation unit is compiled with -Wthread-safety (or
// -Wthread-safety-analysis), so adding them never changes codegen and never
// breaks a build on MSVC/GCC/Clang-without-the-flag. When the analysis is
// enabled, the compiler statically verifies that data marked GUARDED_BY is only
// touched while the named lock is held.

#if defined(__clang__) && !defined(__CUDACC__) && !defined(SWIG)
#define VELOX_THREAD_SAFETY_ANALYSIS 1
#define VELOX_CAPABILITY(x) __attribute__((capability(x)))
#define VELOX_SCOPED_CAPABILITY __attribute__((scoped_lockable))
#define VELOX_GUARDED_BY(x) __attribute__((guarded_by(x)))
#define VELOX_PT_GUARDED_BY(x) __attribute__((pt_guarded_by(x)))
#define VELOX_ACQUIRE(...) __attribute__((acquire_capability(__VA_ARGS__)))
#define VELOX_ACQUIRE_SHARED(...) \
    __attribute__((acquire_shared_capability(__VA_ARGS__)))
#define VELOX_RELEASE(...) __attribute__((release_capability(__VA_ARGS__)))
#define VELOX_RELEASE_SHARED(...) \
    __attribute__((release_shared_capability(__VA_ARGS__)))
#define VELOX_REQUIRES(...) __attribute__((requires_capability(__VA_ARGS__)))
#define VELOX_REQUIRES_SHARED(...) \
    __attribute__((requires_shared_capability(__VA_ARGS__)))
#define VELOX_ACQUIRED_BEFORE(...) __attribute__((acquired_before(__VA_ARGS__)))
#define VELOX_ACQUIRED_AFTER(...) __attribute__((acquired_after(__VA_ARGS__)))
#define VELOX_EXCLUDES(...) __attribute__((locks_excluded(__VA_ARGS__)))
#define VELOX_ASSERT_CAPABILITY(x) __attribute__((assert_capability(x)))
#define VELOX_NO_THREAD_SAFETY_ANALYSIS \
    __attribute__((no_thread_safety_analysis))
#else
#define VELOX_THREAD_SAFETY_ANALYSIS 0
#define VELOX_CAPABILITY(x)
#define VELOX_SCOPED_CAPABILITY
#define VELOX_GUARDED_BY(x)
#define VELOX_PT_GUARDED_BY(x)
#define VELOX_ACQUIRE(...)
#define VELOX_ACQUIRE_SHARED(...)
#define VELOX_RELEASE(...)
#define VELOX_RELEASE_SHARED(...)
#define VELOX_REQUIRES(...)
#define VELOX_REQUIRES_SHARED(...)
#define VELOX_ACQUIRED_BEFORE(...)
#define VELOX_ACQUIRED_AFTER(...)
#define VELOX_EXCLUDES(...)
#define VELOX_ASSERT_CAPABILITY(x)
#define VELOX_NO_THREAD_SAFETY_ANALYSIS
#endif

namespace velox {

// ---------------------------------------------------------------------------
// 2a. Backoff — progressive yielding for spin loops
// ---------------------------------------------------------------------------

// Progressive spin backoff. A spinning thread first executes pause/yield
// instructions (cheap, keeps the core's pipeline warm for the contended
// cacheline), then yields the timeslice, and finally sleeps briefly. Use it to
// avoid burning a core at 100% while waiting on a lock-free structure or a
// SpinLock under contention.
class Backoff {
public:
    // Reset to the initial spin count. Call before each new wait.
    void reset() noexcept { spins_ = 0; }

    // Advance one backoff step. Returns nothing; loop while your condition is
    // unmet and call pause() on each iteration.
    void pause() noexcept {
        if (spins_ < kSpinThreshold) {
            for (uint32_t i = 0; i < (1u << spins_); ++i) {
                cpuRelax();
            }
            ++spins_;
        } else if (spins_ < kYieldThreshold) {
            std::this_thread::yield();
            ++spins_;
        } else {
            // Cap the sleep so a lost wakeup still resolves quickly.
            std::this_thread::sleep_for(std::chrono::microseconds(50));
        }
    }

    // A single CPU relax instruction (PAUSE on x86, YIELD on ARM). Exposed so
    // callers writing their own spin loops can use the same primitive.
    static void cpuRelax() noexcept {
#if defined(_MSC_VER) && (defined(_M_X64) || defined(_M_IX86))
        _mm_pause();
#elif defined(__x86_64__) || defined(__i386__)
        __builtin_ia32_pause();
#elif defined(__aarch64__) || defined(__arm__)
        __asm__ __volatile__("yield" ::: "memory");
#else
        std::atomic_thread_fence(std::memory_order_seq_cst);
#endif
    }

private:
    static constexpr uint32_t kSpinThreshold = 4;
    static constexpr uint32_t kYieldThreshold = 12;
    uint32_t spins_ = 0;
};

// ---------------------------------------------------------------------------
// 2b. SpinLock — a tiny BasicLockable for very short critical sections
// ---------------------------------------------------------------------------

// A test-and-set spinlock satisfying the BasicLockable concept, so it works
// with std::lock_guard / std::unique_lock. Intended only for critical sections
// that are a handful of instructions long (e.g. bumping a shared counter,
// pushing one item onto a queue). It is NOT recursive and must never be held
// across a blocking call. For anything that can stall, use std::mutex.
class VELOX_CAPABILITY("mutex") SpinLock {
public:
    SpinLock() = default;
    SpinLock(const SpinLock&) = delete;
    SpinLock& operator=(const SpinLock&) = delete;

    void lock() noexcept VELOX_ACQUIRE() {
        Backoff backoff;
        while (flag_.test_and_set(std::memory_order_acquire)) {
            backoff.pause();
        }
    }

    bool tryLock() noexcept VELOX_ACQUIRE() {
        return !flag_.test_and_set(std::memory_order_acquire);
    }

    void unlock() noexcept VELOX_RELEASE() {
        flag_.clear(std::memory_order_release);
    }

private:
    std::atomic_flag flag_ = ATOMIC_FLAG_INIT;
};

// ---------------------------------------------------------------------------
// 2c. Atomic<T> — a thin, ergonomic wrapper over std::atomic
// ---------------------------------------------------------------------------

// Wraps std::atomic<T> with sensibly-named operations and explicit memory
// orders, so call sites document their ordering intent instead of relying on
// the sequential-consistency default. Arithmetic operations are only enabled
// for integral and pointer types.
template <typename T>
class Atomic {
public:
    Atomic() noexcept : value_{} {}
    Atomic(T desired) noexcept : value_(desired) {}
    Atomic(const Atomic& other) noexcept : value_(other.load()) {}
    Atomic& operator=(const Atomic& other) noexcept {
        store(other.load());
        return *this;
    }
    Atomic& operator=(T desired) noexcept {
        store(desired);
        return *this;
    }

    T load(std::memory_order order = std::memory_order_seq_cst) const noexcept {
        return value_.load(order);
    }
    void store(T desired,
               std::memory_order order = std::memory_order_seq_cst) noexcept {
        value_.store(desired, order);
    }
    T exchange(T desired,
               std::memory_order order = std::memory_order_seq_cst) noexcept {
        return value_.exchange(desired, order);
    }
    bool compareExchangeWeak(T& expected, T desired,
                             std::memory_order success = std::memory_order_seq_cst,
                             std::memory_order failure = std::memory_order_seq_cst) noexcept {
        return value_.compare_exchange_weak(expected, desired, success, failure);
    }
    bool compareExchangeStrong(T& expected, T desired,
                               std::memory_order success = std::memory_order_seq_cst,
                               std::memory_order failure = std::memory_order_seq_cst) noexcept {
        return value_.compare_exchange_strong(expected, desired, success, failure);
    }

    // Arithmetic — integral/pointer types only.
    template <typename U = T,
              typename = std::enable_if_t<std::is_integral<U>::value>>
    T fetchAdd(T arg,
               std::memory_order order = std::memory_order_seq_cst) noexcept {
        return value_.fetch_add(arg, order);
    }
    template <typename U = T,
              typename = std::enable_if_t<std::is_integral<U>::value>>
    T fetchSub(T arg,
               std::memory_order order = std::memory_order_seq_cst) noexcept {
        return value_.fetch_sub(arg, order);
    }
    template <typename U = T,
              typename = std::enable_if_t<std::is_integral<U>::value>>
    T fetchOr(T arg,
              std::memory_order order = std::memory_order_seq_cst) noexcept {
        return value_.fetch_or(arg, order);
    }
    template <typename U = T,
              typename = std::enable_if_t<std::is_integral<U>::value>>
    T fetchAnd(T arg,
               std::memory_order order = std::memory_order_seq_cst) noexcept {
        return value_.fetch_and(arg, order);
    }

    operator T() const noexcept { return load(); }

    std::atomic<T>& raw() noexcept { return value_; }
    const std::atomic<T>& raw() const noexcept { return value_; }

private:
    std::atomic<T> value_;
};

// ---------------------------------------------------------------------------
// 2d. AtomicFlag — a one-shot, wait-free latch
// ---------------------------------------------------------------------------

// A boolean latch that can be set from any thread and tested from any thread.
// testAndSet() returns the previous value, so it doubles as a "claim once"
// primitive: exactly one caller observes a false return. Backed by
// std::atomic<bool> (rather than std::atomic_flag) so test() can read the
// state without mutating it under C++17.
class AtomicFlag {
public:
    AtomicFlag() noexcept = default;
    AtomicFlag(const AtomicFlag&) = delete;
    AtomicFlag& operator=(const AtomicFlag&) = delete;

    // Set the flag, returning its previous value.
    bool testAndSet(std::memory_order order = std::memory_order_acq_rel) noexcept {
        return flag_.exchange(true, order);
    }
    // Reset to the unset state.
    void clear(std::memory_order order = std::memory_order_release) noexcept {
        flag_.store(false, order);
    }
    bool test(std::memory_order order = std::memory_order_acquire) const noexcept {
        return flag_.load(order);
    }

private:
    std::atomic<bool> flag_{false};
};

// ---------------------------------------------------------------------------
// 2e. AtomicCounter<T> — a contention-friendly monotonic counter
// ---------------------------------------------------------------------------

// A small wrapper for the common "shared counter" case: reference counts,
// pending-task tallies, generation numbers. All mutations are relaxed-friendly
// but default to seq_cst for callers that do not want to reason about ordering.
template <typename T = uint64_t>
class AtomicCounter {
    static_assert(std::is_integral<T>::value, "AtomicCounter requires an integral type");

public:
    AtomicCounter() noexcept = default;
    explicit AtomicCounter(T initial) noexcept : value_(initial) {}
    AtomicCounter(const AtomicCounter&) = delete;
    AtomicCounter& operator=(const AtomicCounter&) = delete;

    T load(std::memory_order order = std::memory_order_acquire) const noexcept {
        return value_.load(order);
    }
    void store(T desired,
               std::memory_order order = std::memory_order_release) noexcept {
        value_.store(desired, order);
    }
    // Returns the value BEFORE the increment.
    T fetchAdd(T delta,
               std::memory_order order = std::memory_order_acq_rel) noexcept {
        return value_.fetch_add(delta, order);
    }
    T fetchSub(T delta,
               std::memory_order order = std::memory_order_acq_rel) noexcept {
        return value_.fetch_sub(delta, order);
    }
    // Convenience: increment and return the NEW value.
    T increment(std::memory_order order = std::memory_order_acq_rel) noexcept {
        return value_.fetch_add(T{1}, order) + T{1};
    }
    // Convenience: decrement and return the NEW value.
    T decrement(std::memory_order order = std::memory_order_acq_rel) noexcept {
        return value_.fetch_sub(T{1}, order) - T{1};
    }
    operator T() const noexcept { return load(); }

private:
    std::atomic<T> value_{T{0}};
};

// ---------------------------------------------------------------------------
// 2f. MutexGuarded<T> — a value coupled with the lock that protects it
// ---------------------------------------------------------------------------

namespace detail {

// RAII handle returned by MutexGuarded::lock(). Holds the lock for its
// lifetime and forwards -> and * to the protected value, so the only way to
// reach the value is while the lock is held.
template <typename T, typename Mutex>
class LockedPtr {
public:
    LockedPtr(T* value, std::unique_lock<Mutex> lock)
        : value_(value), lock_(std::move(lock)) {}

    T* get() const noexcept { return value_; }
    T* operator->() const noexcept { return value_; }
    T& operator*() const noexcept { return *value_; }
    explicit operator bool() const noexcept { return value_ != nullptr; }

private:
    T* value_;
    std::unique_lock<Mutex> lock_;
};

} // namespace detail

// Bundles a value with the mutex that protects it, making the "which lock
// guards this data?" question answerable at the type level. The value can only
// be reached through lock() (which returns an RAII LockedPtr) or withLock()
// (which runs a callable while holding the lock). This eliminates the classic
// bug of a mutex and its data drifting apart in a struct.
//
//   MutexGuarded<std::vector<BodyId>> pending;
//   pending.withLock([](auto& v) { v.push_back(id); });
//   {
//       auto locked = pending.lock();
//       for (auto id : *locked) { ... }
//   } // unlocked here
template <typename T, typename Mutex = std::mutex>
class MutexGuarded {
public:
    MutexGuarded() = default;
    explicit MutexGuarded(T value) : value_(std::move(value)) {}
    MutexGuarded(const MutexGuarded&) = delete;
    MutexGuarded& operator=(const MutexGuarded&) = delete;

    // Acquire the lock and return an RAII handle to the value.
    detail::LockedPtr<T, Mutex> lock() {
        return detail::LockedPtr<T, Mutex>(&value_, std::unique_lock<Mutex>(mutex_));
    }

    // Run fn(T&) while holding the lock; returns fn's result.
    template <typename F>
    auto withLock(F&& fn) -> decltype(fn(std::declval<T&>())) {
        std::unique_lock<Mutex> guard(mutex_);
        return fn(value_);
    }

    // Copy the value out under the lock. Requires T to be copyable.
    T copy() const {
        std::unique_lock<Mutex> guard(mutex_);
        return value_;
    }

private:
    mutable Mutex mutex_;
    T value_{};
};

// ---------------------------------------------------------------------------
// 3a. SpscQueue — wait-free single-producer / single-consumer ring buffer
// ---------------------------------------------------------------------------

// A bounded FIFO for exactly one producer thread and exactly one consumer
// thread. Both tryPush() and tryPop() are wait-free: each performs a bounded,
// constant number of operations regardless of the other thread's progress.
// This is the fastest queue here and is ideal for a dedicated producer feeding
// a dedicated consumer (e.g. the simulation thread handing completed query
// results to a render thread). Capacity is fixed at compile time; using a
// power of two lets the index wrap with a mask instead of a division.
template <typename T, size_t Capacity>
class SpscQueue {
    static_assert(Capacity > 0, "SpscQueue capacity must be at least 1");

public:
    SpscQueue() = default;
    SpscQueue(const SpscQueue&) = delete;
    SpscQueue& operator=(const SpscQueue&) = delete;

    // Producer side. Returns false when full (the item is not enqueued).
    bool tryPush(const T& item) { return emplaceImpl(item); }
    bool tryPush(T&& item) { return emplaceImpl(std::move(item)); }

    template <typename... Args>
    bool tryEmplace(Args&&... args) {
        const size_t tail = tail_.load(std::memory_order_relaxed);
        const size_t next = tail + 1;
        if (next - head_.load(std::memory_order_acquire) > Capacity) {
            return false; // full
        }
        new (&storage_[slot(tail)]) T(std::forward<Args>(args)...);
        tail_.store(next, std::memory_order_release);
        return true;
    }

    // Consumer side. Returns false when empty (out is untouched).
    bool tryPop(T& out) {
        const size_t head = head_.load(std::memory_order_relaxed);
        if (head == tail_.load(std::memory_order_acquire)) {
            return false; // empty
        }
        T* item = reinterpret_cast<T*>(&storage_[slot(head)]);
        out = std::move(*item);
        item->~T();
        head_.store(head + 1, std::memory_order_release);
        return true;
    }

    // Approximate size; only meaningful as a heuristic because producer and
    // consumer indices are read non-atomically with respect to each other.
    size_t sizeApprox() const noexcept {
        const size_t tail = tail_.load(std::memory_order_relaxed);
        const size_t head = head_.load(std::memory_order_relaxed);
        return tail >= head ? tail - head : 0;
    }
    bool empty() const noexcept { return sizeApprox() == 0; }
    static constexpr size_t capacity() noexcept { return Capacity; }

private:
    template <typename U>
    bool emplaceImpl(U&& item) {
        return tryEmplace(std::forward<U>(item));
    }

    static size_t slot(size_t index) noexcept {
        // Power-of-two capacities wrap with a mask; otherwise fall back to mod.
        return (Capacity & (Capacity - 1)) == 0 ? (index & (Capacity - 1))
                                                : (index % Capacity);
    }

    // Pad the two hot indices onto separate cache lines to avoid false sharing
    // between the producer's and consumer's writes.
    alignas(64) std::atomic<size_t> head_{0};
    alignas(64) std::atomic<size_t> tail_{0};
    alignas(64) std::array<typename std::aligned_storage<sizeof(T), alignof(T)>::type,
                           Capacity>
        storage_;
};

// ---------------------------------------------------------------------------
// 3b. MpmcQueue — lock-free bounded multi-producer / multi-consumer queue
// ---------------------------------------------------------------------------

// A bounded FIFO safe for any number of concurrent producers and consumers.
// Implements Dmitry Vyukov's bounded MPMC queue: each slot carries a sequence
// counter, so producers and consumers coordinate with compare-and-swap on the
// slot sequence rather than on a shared head/tail. The sequence counters make
// it immune to the ABA problem that plagues naive lock-free stacks. Capacity
// must be a power of two. tryPush/tryPop are lock-free (they may spin briefly
// on a CAS retry but never block on another thread's scheduling).
template <typename T, size_t Capacity>
class MpmcQueue {
    static_assert(Capacity >= 2, "MpmcQueue capacity must be at least 2");
    static_assert((Capacity & (Capacity - 1)) == 0,
                  "MpmcQueue capacity must be a power of two");

    struct Cell {
        std::atomic<size_t> sequence;
        typename std::aligned_storage<sizeof(T), alignof(T)>::type storage;
    };

public:
    MpmcQueue() {
        for (size_t i = 0; i < Capacity; ++i) {
            cells_[i].sequence.store(i, std::memory_order_relaxed);
        }
    }
    MpmcQueue(const MpmcQueue&) = delete;
    MpmcQueue& operator=(const MpmcQueue&) = delete;

    template <typename... Args>
    bool tryEmplace(Args&&... args) {
        size_t pos = enqueuePos_.load(std::memory_order_relaxed);
        for (;;) {
            Cell& cell = cells_[pos & kMask];
            const size_t seq = cell.sequence.load(std::memory_order_acquire);
            const intptr_t diff = static_cast<intptr_t>(seq) - static_cast<intptr_t>(pos);
            if (diff == 0) {
                if (enqueuePos_.compare_exchange_weak(pos, pos + 1,
                                                      std::memory_order_relaxed)) {
                    new (&cell.storage) T(std::forward<Args>(args)...);
                    cell.sequence.store(pos + 1, std::memory_order_release);
                    return true;
                }
            } else if (diff < 0) {
                return false; // full
            } else {
                pos = enqueuePos_.load(std::memory_order_relaxed);
            }
        }
    }

    bool tryPush(const T& item) { return tryEmplace(item); }
    bool tryPush(T&& item) { return tryEmplace(std::move(item)); }

    bool tryPop(T& out) {
        size_t pos = dequeuePos_.load(std::memory_order_relaxed);
        for (;;) {
            Cell& cell = cells_[pos & kMask];
            const size_t seq = cell.sequence.load(std::memory_order_acquire);
            const intptr_t diff = static_cast<intptr_t>(seq) - static_cast<intptr_t>(pos + 1);
            if (diff == 0) {
                if (dequeuePos_.compare_exchange_weak(pos, pos + 1,
                                                      std::memory_order_relaxed)) {
                    T* item = reinterpret_cast<T*>(&cell.storage);
                    out = std::move(*item);
                    item->~T();
                    cell.sequence.store(pos + Capacity, std::memory_order_release);
                    return true;
                }
            } else if (diff < 0) {
                return false; // empty
            } else {
                pos = dequeuePos_.load(std::memory_order_relaxed);
            }
        }
    }

    static constexpr size_t capacity() noexcept { return Capacity; }

private:
    static constexpr size_t kMask = Capacity - 1;

    alignas(64) std::array<Cell, Capacity> cells_;
    alignas(64) std::atomic<size_t> enqueuePos_{0};
    alignas(64) std::atomic<size_t> dequeuePos_{0};
};

// ---------------------------------------------------------------------------
// 4a. ThreadLocal<T> — per-instance, per-thread storage
// ---------------------------------------------------------------------------

// Per-thread storage keyed by both the ThreadLocal instance and the accessing
// thread. Unlike a bare `thread_local` variable (which is one value per type
// per thread), each ThreadLocal<T> object hands every thread its own T, so you
// can have many independent per-thread scratch buffers.
//
// Implementation note: portability is favored over raw speed. A mutex guards a
// map from thread id to value; the common "same thread as last time" case is
// served from a lock-free cache so the steady-state cost is a single atomic
// load. This is well suited to per-thread scratch that is touched a modest
// number of times per frame; it is NOT intended for the innermost loop of a
// solver. For that, capture a reference from getOrCreate() once per frame and
// reuse it.
template <typename T>
class ThreadLocal {
public:
    ThreadLocal() = default;
    explicit ThreadLocal(T initial) : initial_(std::move(initial)), hasInitial_(true) {}
    ThreadLocal(const ThreadLocal&) = delete;
    ThreadLocal& operator=(const ThreadLocal&) = delete;

    // Get this thread's value, default-constructing (or copying the initial
    // value) on first access. Never returns null.
    T& getOrCreate() {
        const std::thread::id id = std::this_thread::get_id();
        // Fast path: same thread as the previous access.
        if (cacheId_.load(std::memory_order_relaxed) == id) {
            T* cached = cachePtr_.load(std::memory_order_acquire);
            if (cached) return *cached;
        }
        return resolveSlow(id);
    }

    // Get this thread's value, or nullptr if it has never been created.
    T* get() {
        const std::thread::id id = std::this_thread::get_id();
        if (cacheId_.load(std::memory_order_relaxed) == id) {
            return cachePtr_.load(std::memory_order_acquire);
        }
        std::lock_guard<std::mutex> guard(mutex_);
        auto it = values_.find(id);
        return it == values_.end() ? nullptr : it->second.get();
    }

    // Set this thread's value, creating the slot if needed.
    void set(T value) { getOrCreate() = std::move(value); }

    // Run fn(T&) for every thread's value that has been created. Useful for
    // aggregating per-thread counters at a frame boundary. The map is locked
    // for the duration, so fn must not call back into this ThreadLocal.
    template <typename F>
    void forEach(F&& fn) {
        std::lock_guard<std::mutex> guard(mutex_);
        for (auto& kv : values_) {
            fn(*kv.second);
        }
    }

    size_t threadCount() const {
        std::lock_guard<std::mutex> guard(mutex_);
        return values_.size();
    }

private:
    T& resolveSlow(const std::thread::id& id) {
        std::lock_guard<std::mutex> guard(mutex_);
        auto it = values_.find(id);
        if (it == values_.end()) {
            std::unique_ptr<T> created =
                hasInitial_ ? std::make_unique<T>(initial_)
                            : std::make_unique<T>();
            T* raw = created.get();
            values_.emplace(id, std::move(created));
            cachePtr_.store(raw, std::memory_order_release);
            cacheId_.store(id, std::memory_order_relaxed);
            return *raw;
        }
        cachePtr_.store(it->second.get(), std::memory_order_release);
        cacheId_.store(id, std::memory_order_relaxed);
        return *it->second;
    }

    T initial_{};
    bool hasInitial_ = false;
    mutable std::mutex mutex_;
    std::unordered_map<std::thread::id, std::unique_ptr<T>> values_;
    // Single-slot cache of the most recent (thread id -> value) resolution.
    std::atomic<std::thread::id> cacheId_{};
    std::atomic<T*> cachePtr_{nullptr};
};

// ---------------------------------------------------------------------------
// 4b. ThreadLocalAccumulator<T> — contention-free per-thread counters
// ---------------------------------------------------------------------------

// A per-thread counter that threads can bump without any contention, plus a
// total() that sums every thread's contribution. Use it for statistics that
// would otherwise serialize on a single AtomicCounter (e.g. "contacts tested
// this frame" accumulated by every worker). Reads via total() take the lock
// briefly and are intended for frame boundaries, not the hot loop.
template <typename T = uint64_t>
class ThreadLocalAccumulator {
    static_assert(std::is_integral<T>::value,
                  "ThreadLocalAccumulator requires an integral type");

public:
    // Add delta to this thread's private counter. Lock-free in steady state.
    void add(T delta) { storage_.getOrCreate() += delta; }
    void increment() { add(T{1}); }

    // This thread's current contribution.
    T local() const {
        T* p = const_cast<ThreadLocalAccumulator*>(this)->storage_.get();
        return p ? *p : T{0};
    }

    // Sum of every thread's contribution so far.
    T total() const {
        T sum = T{0};
        const_cast<ThreadLocalAccumulator*>(this)->storage_.forEach(
            [&sum](T& v) { sum += v; });
        return sum;
    }

    // Reset every thread's contribution to zero.
    void reset() {
        storage_.forEach([](T& v) { v = T{0}; });
    }

private:
    ThreadLocal<T> storage_{T{0}};
};

} // namespace velox
