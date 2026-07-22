#pragma once
#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <new>
#include <stdexcept>
#include <utility>
#include <vector>

namespace velox {

/**
 * @file memory_pool.h
 * @brief Pool allocators used to remove malloc/free churn from the simulation
 *        hot path.
 *
 * Three cooperating allocators live here:
 *
 *  - @ref FixedBlockPool / @ref FixedPool<T> — a free-list pool of equal-size
 *    blocks. Ideal for the engine's large, fixed-size records (@ref Body,
 *    @ref Contact, @ref Joint): allocation and deallocation are O(1) and never
 *    fragment because every block is the same size.
 *
 *  - @ref SlabAllocator — a variable-size allocator that rounds requests up to
 *    a power-of-two size class, each backed by a @ref FixedBlockPool. Requests
 *    larger than the largest class fall back to the system allocator. Used for
 *    query-result buffers and other irregularly sized scratch data.
 *
 *  - @ref MemoryPool — a thin facade over a @ref SlabAllocator that also offers
 *    typed `create`/`destroy` helpers and aggregate @ref MemoryPoolStats.
 *
 * All three are thread-safe: every public mutating entry point is serialized
 * with an internal mutex, so worker threads may allocate and free concurrently.
 * The mutex keeps the implementation correct and simple; the win comes from
 * avoiding the global system allocator and from reusing warm blocks, not from
 * lock-free synchronization.
 *
 * @code
 * velox::FixedPool<velox::Body> bodies(1024);   // 1024 blocks per chunk
 * velox::Body* b = bodies.create();             // placement-new in a pooled block
 * bodies.destroy(b);                            // O(1) recycle, no free()
 *
 * velox::MemoryPool pool;
 * void* scratch = pool.allocate(2048);          // slab class 2048
 * pool.deallocate(scratch, 2048);
 * velox::MemoryPoolStats stats = pool.stats();
 * @endcode
 */

/**
 * @brief Aggregate accounting for a pool or a group of pools.
 *
 * `fragmentation` is the internal fragmentation ratio in `[0, 1]`: the fraction
 * of handed-out bytes that the caller did not ask for (size-class rounding and
 * alignment padding). A pool of exact-size blocks reports `0`.
 */
struct MemoryPoolStats {
    size_t reservedBytes = 0;      ///< Bytes obtained from the system.
    size_t usedBytes = 0;          ///< Bytes currently handed out (rounded).
    size_t requestedBytes = 0;     ///< Bytes currently requested by callers.
    size_t peakUsedBytes = 0;      ///< High-water mark of @ref usedBytes.
    size_t activeAllocations = 0;  ///< Live allocation count.
    uint64_t allocationCount = 0;  ///< Lifetime allocations.
    uint64_t deallocationCount = 0;///< Lifetime deallocations.
    double fragmentation = 0.0;    ///< Internal fragmentation ratio `[0, 1]`.

    /** @brief Sum another snapshot into this one (fragmentation recomputed). */
    void add(const MemoryPoolStats& other) {
        reservedBytes += other.reservedBytes;
        usedBytes += other.usedBytes;
        requestedBytes += other.requestedBytes;
        peakUsedBytes += other.peakUsedBytes;
        activeAllocations += other.activeAllocations;
        allocationCount += other.allocationCount;
        deallocationCount += other.deallocationCount;
        fragmentation = usedBytes > 0
            ? 1.0 - double(requestedBytes) / double(usedBytes)
            : 0.0;
    }
};

namespace detail {

// Aligned raw allocation. Falls back to plain operator new when the requested
// alignment does not exceed the default max_align_t (avoids the aligned-new
// code path for the common case).
inline void* poolAllocAligned(size_t bytes, size_t alignment) {
    if (alignment <= alignof(std::max_align_t))
        return ::operator new(bytes);
    return ::operator new(bytes, std::align_val_t(alignment));
}

inline void poolFreeAligned(void* ptr, size_t alignment) {
    if (!ptr) return;
    if (alignment <= alignof(std::max_align_t))
        ::operator delete(ptr);
    else
        ::operator delete(ptr, std::align_val_t(alignment));
}

inline size_t roundUp(size_t value, size_t multiple) {
    return (value + multiple - 1) & ~(multiple - 1);
}

} // namespace detail

/**
 * @brief Free-list pool of fixed-size, fixed-alignment blocks.
 *
 * Blocks are carved from large chunks obtained from the system; freed blocks
 * are threaded onto an intrusive free list (the next-pointer lives inside the
 * free block) and handed back on the next allocation. Chunks are released only
 * at destruction, so a pool that has grown to its peak stays warm for the rest
 * of its lifetime — exactly what a per-frame contact buffer wants.
 *
 * Thread-safe: all mutating operations take an internal mutex.
 */
class FixedBlockPool {
public:
    /**
     * @brief Construct a pool.
     * @param blockSize      Minimum usable bytes per block (rounded up to
     *                       `alignment` and to at least `sizeof(void*)`).
     * @param blocksPerChunk Number of blocks carved per system allocation.
     * @param alignment      Alignment of every returned block (power of two).
     */
    explicit FixedBlockPool(size_t blockSize,
                            size_t blocksPerChunk = 256,
                            size_t alignment = alignof(std::max_align_t))
        : alignment_(alignment ? alignment : alignof(std::max_align_t)),
          blocksPerChunk_(blocksPerChunk ? blocksPerChunk : 1) {
        if (blockSize == 0)
            throw std::invalid_argument("velox: pool block size must be > 0");
        // A free block must hold the intrusive next-pointer.
        size_t minSize = std::max(blockSize, sizeof(void*));
        stride_ = detail::roundUp(minSize, alignment_);
    }

    ~FixedBlockPool() { releaseChunks(); }

    FixedBlockPool(const FixedBlockPool&) = delete;
    FixedBlockPool& operator=(const FixedBlockPool&) = delete;

    FixedBlockPool(FixedBlockPool&& other) noexcept { moveFrom(std::move(other)); }
    FixedBlockPool& operator=(FixedBlockPool&& other) noexcept {
        if (this != &other) {
            releaseChunks();
            moveFrom(std::move(other));
        }
        return *this;
    }

    /**
     * @brief Reserve capacity for at least `count` blocks without handing any
     *        out.
     *
     * Useful at world construction to pay the growth cost once, up front, so
     * the simulation step never touches the system allocator.
     */
    void reserve(size_t count) {
        std::lock_guard<std::mutex> lock(mutex_);
        while (capacityBlocks() < count)
            growChunk();
    }

    /**
     * @brief Allocate one block, growing the pool by a chunk if needed.
     * @return An aligned block of at least `blockSize()` usable bytes; never
     *         null (growth throws on out-of-memory).
     */
    void* allocate() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!freeList_)
            growChunk();
        void* block = freeList_;
        freeList_ = *reinterpret_cast<void**>(freeList_);
        ++inUse_;
        ++allocCount_;
        if (inUse_ > peakInUse_) peakInUse_ = inUse_;
        return block;
    }

    /**
     * @brief Return a block previously obtained from `allocate()`.
     *
     * Passing a null pointer is a no-op. Passing a pointer not owned by this
     * pool is undefined behavior.
     */
    void deallocate(void* block) {
        if (!block) return;
        std::lock_guard<std::mutex> lock(mutex_);
        *reinterpret_cast<void**>(block) = freeList_;
        freeList_ = block;
        if (inUse_ > 0) --inUse_;
        ++deallocCount_;
    }

    /// @name Queries
    /// @{
    size_t blockSize() const { return stride_; }      ///< Usable bytes per block.
    size_t alignment() const { return alignment_; }   ///< Block alignment.
    size_t blocksPerChunk() const { return blocksPerChunk_; }
    size_t capacityBlocks() const { return chunks_.size() * blocksPerChunk_; }
    size_t inUseBlocks() const { return inUse_; }
    size_t freeBlocks() const { return capacityBlocks() - inUse_; }
    size_t peakInUseBlocks() const { return peakInUse_; }
    size_t chunkCount() const { return chunks_.size(); }
    uint64_t allocationCount() const { return allocCount_; }
    uint64_t deallocationCount() const { return deallocCount_; }

    /** @brief Snapshot the pool's accounting. */
    MemoryPoolStats stats() const {
        std::lock_guard<std::mutex> lock(mutex_);
        MemoryPoolStats s;
        s.reservedBytes = capacityBlocks() * stride_;
        s.usedBytes = inUse_ * stride_;
        s.requestedBytes = inUse_ * stride_; // exact-size blocks: no rounding
        s.peakUsedBytes = peakInUse_ * stride_;
        s.activeAllocations = inUse_;
        s.allocationCount = allocCount_;
        s.deallocationCount = deallocCount_;
        s.fragmentation = 0.0; // fixed-size blocks do not internally fragment
        return s;
    }
    /// @}

private:
    void growChunk() {
        size_t bytes = blocksPerChunk_ * stride_;
        uint8_t* chunk = static_cast<uint8_t*>(
            detail::poolAllocAligned(bytes, alignment_));
        chunks_.push_back(chunk);
        // Thread the new blocks onto the free list. Order is irrelevant.
        for (size_t i = 0; i < blocksPerChunk_; ++i) {
            void* block = chunk + i * stride_;
            *reinterpret_cast<void**>(block) = freeList_;
            freeList_ = block;
        }
    }

    void releaseChunks() {
        for (void* chunk : chunks_)
            detail::poolFreeAligned(chunk, alignment_);
        chunks_.clear();
        freeList_ = nullptr;
        inUse_ = 0;
    }

    void moveFrom(FixedBlockPool&& other) {
        std::lock_guard<std::mutex> a(mutex_);
        std::lock_guard<std::mutex> b(other.mutex_);
        alignment_ = other.alignment_;
        stride_ = other.stride_;
        blocksPerChunk_ = other.blocksPerChunk_;
        chunks_ = std::move(other.chunks_);
        freeList_ = other.freeList_;
        inUse_ = other.inUse_;
        peakInUse_ = other.peakInUse_;
        allocCount_ = other.allocCount_;
        deallocCount_ = other.deallocCount_;
        other.freeList_ = nullptr;
        other.inUse_ = 0;
        other.peakInUse_ = 0;
        other.allocCount_ = 0;
        other.deallocCount_ = 0;
    }

    size_t alignment_;
    size_t stride_;
    size_t blocksPerChunk_;
    std::vector<void*> chunks_;
    void* freeList_ = nullptr;
    size_t inUse_ = 0;
    size_t peakInUse_ = 0;
    uint64_t allocCount_ = 0;
    uint64_t deallocCount_ = 0;
    mutable std::mutex mutex_;
};

/**
 * @brief Typed wrapper over @ref FixedBlockPool.
 *
 * `create()` constructs a `T` in a pooled block with placement new; `destroy()`
 * runs the destructor and recycles the block. `allocate()`/`deallocate()` expose
 * the raw block for callers (such as the World's dense arrays) that manage
 * construction themselves.
 */
template <typename T>
class FixedPool {
public:
    explicit FixedPool(size_t blocksPerChunk = 256)
        : pool_(sizeof(T), blocksPerChunk, alignof(T)) {}

    /// Construct a `T` in a pooled block. Returns null only if `T`'s
    /// constructor throws after allocation (the block is recycled).
    template <typename... Args>
    T* create(Args&&... args) {
        void* mem = pool_.allocate();
        try {
            return new (mem) T(std::forward<Args>(args)...);
        } catch (...) {
            pool_.deallocate(mem);
            throw;
        }
    }

    /// Destroy a `T` and recycle its block. Null is a no-op.
    void destroy(T* ptr) {
        if (!ptr) return;
        ptr->~T();
        pool_.deallocate(ptr);
    }

    /// Raw block allocation without construction.
    T* allocate() { return static_cast<T*>(pool_.allocate()); }
    /// Recycle a raw block without running a destructor.
    void deallocate(T* ptr) { pool_.deallocate(ptr); }

    void reserve(size_t count) { pool_.reserve(count); }

    const FixedBlockPool& blockPool() const { return pool_; }
    FixedBlockPool& blockPool() { return pool_; }

    size_t capacity() const { return pool_.capacityBlocks(); }
    size_t inUse() const { return pool_.inUseBlocks(); }
    size_t free() const { return pool_.freeBlocks(); }
    MemoryPoolStats stats() const { return pool_.stats(); }

private:
    FixedBlockPool pool_;
};

/**
 * @brief Variable-size allocator over power-of-two size classes.
 *
 * Requests up to `maxSize` are rounded up to the nearest power-of-two class
 * (starting at `minSize`) and served from that class's @ref FixedBlockPool.
 * Larger requests go straight to the system allocator. Deallocation requires
 * the original requested size so the correct class (or the system) can reclaim
 * the block.
 *
 * Thread-safe.
 */
class SlabAllocator {
public:
    static constexpr size_t kDefaultMinSize = 16;
    static constexpr size_t kDefaultMaxSize = 4096;

    explicit SlabAllocator(size_t minSize = kDefaultMinSize,
                           size_t maxSize = kDefaultMaxSize,
                           size_t blocksPerChunk = 256)
        : minSize_(minSize ? minSize : kDefaultMinSize),
          maxSize_(maxSize ? maxSize : kDefaultMaxSize) {
        if (maxSize_ < minSize_)
            throw std::invalid_argument(
                "velox: slab maxSize must be >= minSize");
        // Build one pool per power-of-two class in [minSize_, maxSize_].
        for (size_t classSize = minSize_; classSize <= maxSize_; classSize <<= 1) {
            classes_.emplace_back(
                std::make_unique<FixedBlockPool>(classSize, blocksPerChunk));
            if (classSize > maxSize_ / 2) break; // avoid overflow past maxSize
        }
    }

    /**
     * @brief Allocate at least `size` bytes.
     * @param size Requested bytes; `0` is treated as `1`.
     * @param alignment Requested alignment (power of two). Sizes that fit a
     *        class use the class's natural alignment, which is always a power
     *        of two >= `minSize_`; oversized requests honor `alignment`.
     * @return A non-null pointer; recycle with `deallocate(ptr, size)`.
     */
    void* allocate(size_t size, size_t alignment = alignof(std::max_align_t)) {
        size = size ? size : 1;
        if (size <= maxSize_) {
            size_t idx = classIndex(size);
            void* ptr = classes_[idx]->allocate();
            std::lock_guard<std::mutex> lock(statsMutex_);
            requestedBytes_ += size;
            usedBytes_ += classes_[idx]->blockSize();
            ++activeAllocations_;
            ++allocationCount_;
            if (usedBytes_ > peakUsedBytes_) peakUsedBytes_ = usedBytes_;
            return ptr;
        }
        // Oversized: system allocation, tracked separately.
        void* ptr = detail::poolAllocAligned(size, alignment);
        std::lock_guard<std::mutex> lock(statsMutex_);
        requestedBytes_ += size;
        usedBytes_ += size;
        ++oversizedActive_;
        ++activeAllocations_;
        ++allocationCount_;
        if (usedBytes_ > peakUsedBytes_) peakUsedBytes_ = usedBytes_;
        return ptr;
    }

    /**
     * @brief Recycle a pointer obtained from `allocate()`.
     * @param ptr  Pointer to free; null is a no-op.
     * @param size The same size passed to `allocate()`.
     */
    void deallocate(void* ptr, size_t size) {
        if (!ptr) return;
        size = size ? size : 1;
        if (size <= maxSize_) {
            size_t idx = classIndex(size);
            classes_[idx]->deallocate(ptr);
            std::lock_guard<std::mutex> lock(statsMutex_);
            requestedBytes_ -= size;
            usedBytes_ -= classes_[idx]->blockSize();
            if (activeAllocations_ > 0) --activeAllocations_;
            ++deallocationCount_;
            return;
        }
        detail::poolFreeAligned(ptr, alignof(std::max_align_t));
        std::lock_guard<std::mutex> lock(statsMutex_);
        requestedBytes_ -= size;
        usedBytes_ -= size;
        if (oversizedActive_ > 0) --oversizedActive_;
        if (activeAllocations_ > 0) --activeAllocations_;
        ++deallocationCount_;
    }

    /// @name Queries
    /// @{
    size_t minSize() const { return minSize_; }
    size_t maxSize() const { return maxSize_; }
    size_t classCount() const { return classes_.size(); }
    /// Size in bytes of the class that would serve a request of `size` bytes.
    size_t classSizeFor(size_t size) const {
        size = size ? size : 1;
        if (size > maxSize_) return size;
        return classes_[classIndex(size)]->blockSize();
    }

    MemoryPoolStats stats() const {
        MemoryPoolStats aggregate;
        for (const auto& pool : classes_)
            aggregate.add(pool->stats());
        std::lock_guard<std::mutex> lock(statsMutex_);
        // Layer the live request/usage accounting (which tracks rounding and
        // oversized allocations) on top of the per-class reserved totals.
        aggregate.requestedBytes = requestedBytes_;
        aggregate.usedBytes = usedBytes_;
        aggregate.peakUsedBytes = std::max(aggregate.peakUsedBytes, peakUsedBytes_);
        aggregate.activeAllocations = activeAllocations_;
        aggregate.allocationCount = allocationCount_;
        aggregate.deallocationCount = deallocationCount_;
        aggregate.fragmentation = usedBytes_ > 0
            ? 1.0 - double(requestedBytes_) / double(usedBytes_)
            : 0.0;
        return aggregate;
    }
    /// @}

private:
    // Smallest class index whose size is >= `size`. `size` is in [1, maxSize_].
    size_t classIndex(size_t size) const {
        size_t idx = 0;
        size_t classSize = minSize_;
        while (classSize < size) {
            classSize <<= 1;
            ++idx;
        }
        if (idx >= classes_.size()) idx = classes_.size() - 1;
        return idx;
    }

    size_t minSize_;
    size_t maxSize_;
    std::vector<std::unique_ptr<FixedBlockPool>> classes_;

    mutable std::mutex statsMutex_;
    size_t requestedBytes_ = 0;
    size_t usedBytes_ = 0;
    size_t peakUsedBytes_ = 0;
    size_t activeAllocations_ = 0;
    size_t oversizedActive_ = 0;
    uint64_t allocationCount_ = 0;
    uint64_t deallocationCount_ = 0;
};

/**
 * @brief General-purpose pool facade over a @ref SlabAllocator.
 *
 * Adds typed `create`/`destroy` helpers and aggregate @ref MemoryPoolStats on
 * top of the slab. This is the allocator the World uses for irregularly sized
 * scratch such as query-result buffers.
 */
class MemoryPool {
public:
    explicit MemoryPool(size_t minSize = SlabAllocator::kDefaultMinSize,
                        size_t maxSize = SlabAllocator::kDefaultMaxSize,
                        size_t blocksPerChunk = 256)
        : slab_(minSize, maxSize, blocksPerChunk) {}

    void* allocate(size_t size, size_t alignment = alignof(std::max_align_t)) {
        return slab_.allocate(size, alignment);
    }
    void deallocate(void* ptr, size_t size) { slab_.deallocate(ptr, size); }

    /// Typed allocate (raw storage for one `T`, not constructed).
    template <typename T>
    T* allocate() {
        return static_cast<T*>(slab_.allocate(sizeof(T), alignof(T)));
    }

    /// Construct a `T` in pooled storage.
    template <typename T, typename... Args>
    T* create(Args&&... args) {
        void* mem = slab_.allocate(sizeof(T), alignof(T));
        try {
            return new (mem) T(std::forward<Args>(args)...);
        } catch (...) {
            slab_.deallocate(mem, sizeof(T));
            throw;
        }
    }

    /// Destroy a `T` and recycle its storage. Null is a no-op.
    template <typename T>
    void destroy(T* ptr) {
        if (!ptr) return;
        ptr->~T();
        slab_.deallocate(ptr, sizeof(T));
    }

    const SlabAllocator& slab() const { return slab_; }
    SlabAllocator& slab() { return slab_; }

    MemoryPoolStats stats() const { return slab_.stats(); }
    double fragmentation() const { return slab_.stats().fragmentation; }

private:
    SlabAllocator slab_;
};

} // namespace velox
