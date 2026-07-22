#pragma once
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <new>
#include <stdexcept>

namespace velox {

// Frame-scoped bump allocator for temporary per-frame data: contact buffers,
// island scratch arrays, solver temporaries. Allocate freely during a frame,
// then call reset() once at the start of the next frame to reclaim everything
// in O(1). This eliminates malloc/free churn during stepping.
//
// Thread safety: allocate() is lock-free — it advances the bump pointer with
// an atomic compare-and-swap, so multiple worker threads can carve out scratch
// space concurrently without a mutex. reset() must only be called when no
// threads are inside allocate().
//
// Usage:
//   ArenaAllocator arena(2 * 1024 * 1024);   // 2 MiB block
//   auto* parents = arena.allocateArray<uint32_t>(bodyCount);
//   auto* scratch = arena.allocateArray<Contact>(maxContacts);
//   // ... use the arrays ...
//   arena.reset();  // next frame: all pointers invalidated, memory reused
class ArenaAllocator {
public:
    static constexpr size_t kDefaultCapacity = 4 * 1024 * 1024; // 4 MiB

    explicit ArenaAllocator(size_t capacityBytes = kDefaultCapacity)
        : capacity_(capacityBytes), offset_(0) {
        if (capacityBytes == 0)
            throw std::invalid_argument("velox: arena capacity must be > 0");
        // Allocate with extra space for alignment, then align the block pointer.
        size_t allocSize = capacityBytes + 128; // extra for 64-byte alignment
        void* raw = ::operator new(allocSize);
        uintptr_t rawAddr = reinterpret_cast<uintptr_t>(raw);
        uintptr_t alignedAddr = (rawAddr + 63) & ~uintptr_t(63);
        block_ = reinterpret_cast<uint8_t*>(alignedAddr);
        rawBlock_ = raw;
    }

    ~ArenaAllocator() { ::operator delete(rawBlock_); }

    // Non-copyable — the block is uniquely owned.
    ArenaAllocator(const ArenaAllocator&) = delete;
    ArenaAllocator& operator=(const ArenaAllocator&) = delete;

    // Movable.
    ArenaAllocator(ArenaAllocator&& other) noexcept
        : block_(other.block_),
          rawBlock_(other.rawBlock_),
          capacity_(other.capacity_),
          offset_(other.offset_.load(std::memory_order_relaxed)) {
        other.block_ = nullptr;
        other.rawBlock_ = nullptr;
        other.capacity_ = 0;
        other.offset_.store(0, std::memory_order_relaxed);
    }

    ArenaAllocator& operator=(ArenaAllocator&& other) noexcept {
        if (this != &other) {
            ::operator delete(rawBlock_);
            block_ = other.block_;
            rawBlock_ = other.rawBlock_;
            capacity_ = other.capacity_;
            offset_.store(other.offset_.load(std::memory_order_relaxed),
                          std::memory_order_relaxed);
            other.block_ = nullptr;
            other.rawBlock_ = nullptr;
            other.capacity_ = 0;
            other.offset_.store(0, std::memory_order_relaxed);
        }
        return *this;
    }

    // Bump-allocate `size` bytes with the given power-of-two `alignment`.
    // Returns nullptr when the arena is exhausted. Thread-safe (lock-free).
    void* allocate(size_t size, size_t alignment = alignof(std::max_align_t)) {
        size_t current = offset_.load(std::memory_order_relaxed);
        for (;;) {
            size_t aligned = (current + alignment - 1) & ~(alignment - 1);
            size_t next = aligned + size;
            if (next > capacity_) return nullptr;
            if (offset_.compare_exchange_weak(current, next,
                                              std::memory_order_relaxed))
                return block_ + aligned;
            // CAS failed: `current` was reloaded; retry with the new value.
        }
    }

    // Typed convenience: allocate raw storage for `count` elements of T.
    // Elements are NOT constructed — suitable for trivially-constructible
    // types or placement new.
    template <typename T>
    T* allocateArray(size_t count) {
        return static_cast<T*>(allocate(count * sizeof(T), alignof(T)));
    }

    // Reclaim all allocations in O(1). Call once per frame, outside any
    // concurrent allocate() calls. All previously returned pointers are
    // invalidated.
    void reset() { offset_.store(0, std::memory_order_relaxed); }

    // Capacity and usage queries.
    size_t capacity() const { return capacity_; }
    size_t used() const {
        return offset_.load(std::memory_order_relaxed);
    }
    size_t remaining() const { return capacity_ - used(); }

private:
    uint8_t* block_;
    void* rawBlock_; // original allocation for deallocation
    size_t capacity_;
    std::atomic<size_t> offset_; // bump pointer; atomic for thread safety
};

} // namespace velox
