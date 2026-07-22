#include "doctest.h"
#include <velox/arena.h>
#include <velox/velox.h>
#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <thread>
#include <vector>

using namespace velox;

TEST_CASE("ArenaAllocator basic allocation") {
    ArenaAllocator arena(1024);

    SUBCASE("capacity reflects constructor argument") {
        CHECK(arena.capacity() == 1024);
        CHECK(arena.used() == 0);
        CHECK(arena.remaining() == 1024);
    }

    SUBCASE("single allocation") {
        void* ptr = arena.allocate(128);
        REQUIRE(ptr != nullptr);
        CHECK(arena.used() >= 128);
        CHECK(arena.remaining() <= 1024 - 128);
    }

    SUBCASE("multiple allocations do not overlap") {
        auto* a = static_cast<uint8_t*>(arena.allocate(64));
        auto* b = static_cast<uint8_t*>(arena.allocate(64));
        REQUIRE(a != nullptr);
        REQUIRE(b != nullptr);
        // The two regions must not overlap.
        bool noOverlap = (a + 64 <= b) || (b + 64 <= a);
        CHECK(noOverlap);
    }

    SUBCASE("allocation returns writable memory") {
        auto* ptr = static_cast<uint8_t*>(arena.allocate(256));
        REQUIRE(ptr != nullptr);
        std::memset(ptr, 0xAB, 256);
        for (int i = 0; i < 256; ++i)
            CHECK(ptr[i] == 0xAB);
    }

    SUBCASE("zero-size allocation succeeds") {
        void* ptr = arena.allocate(0);
        CHECK(ptr != nullptr);
    }
}

TEST_CASE("ArenaAllocator alignment") {
    ArenaAllocator arena(4096);

    SUBCASE("default alignment is max_align_t") {
        void* ptr = arena.allocate(1);
        REQUIRE(ptr != nullptr);
        CHECK(reinterpret_cast<uintptr_t>(ptr) % alignof(std::max_align_t) == 0);
    }

    SUBCASE("explicit power-of-two alignment") {
        // Burn 1 byte to misalign the bump pointer.
        arena.allocate(1, 1);
        for (size_t alignment : {1u, 2u, 4u, 8u, 16u, 32u, 64u, 128u}) {
            void* ptr = arena.allocate(16, alignment);
            REQUIRE(ptr != nullptr);
            CHECK(reinterpret_cast<uintptr_t>(ptr) % alignment == 0);
        }
    }

    SUBCASE("allocateArray respects type alignment") {
        struct alignas(64) Padded { uint8_t data[64]; };
        arena.allocate(1, 1); // misalign
        Padded* ptr = arena.allocateArray<Padded>(4);
        REQUIRE(ptr != nullptr);
        CHECK(reinterpret_cast<uintptr_t>(ptr) % 64 == 0);
    }
}

TEST_CASE("ArenaAllocator reset") {
    ArenaAllocator arena(512);

    SUBCASE("reset reclaims all memory") {
        arena.allocate(256);
        arena.allocate(128);
        CHECK(arena.used() > 0);
        arena.reset();
        CHECK(arena.used() == 0);
        CHECK(arena.remaining() == 512);
    }

    SUBCASE("memory is reusable after reset") {
        void* first = arena.allocate(256);
        REQUIRE(first != nullptr);
        arena.reset();
        void* second = arena.allocate(256);
        REQUIRE(second != nullptr);
        // After reset the bump pointer restarts, so the same address is
        // returned for the same allocation pattern.
        CHECK(first == second);
    }

    SUBCASE("repeated reset-allocate cycles") {
        for (int frame = 0; frame < 100; ++frame) {
            arena.reset();
            auto* data = arena.allocateArray<uint32_t>(64);
            REQUIRE(data != nullptr);
            for (uint32_t i = 0; i < 64; ++i) data[i] = i;
            for (uint32_t i = 0; i < 64; ++i) CHECK(data[i] == i);
        }
    }
}

TEST_CASE("ArenaAllocator exhaustion") {
    ArenaAllocator arena(256);

    SUBCASE("returns nullptr when capacity is exceeded") {
        void* ptr = arena.allocate(256);
        REQUIRE(ptr != nullptr);
        // Arena is now full (or nearly — alignment may consume a few bytes).
        void* overflow = arena.allocate(256);
        CHECK(overflow == nullptr);
    }

    SUBCASE("reset allows allocation again after exhaustion") {
        arena.allocate(256);
        CHECK(arena.allocate(1) == nullptr);
        arena.reset();
        CHECK(arena.allocate(1) != nullptr);
    }

    SUBCASE("large single allocation that exceeds capacity") {
        void* ptr = arena.allocate(1024);
        CHECK(ptr == nullptr);
        CHECK(arena.used() == 0);
    }
}

TEST_CASE("ArenaAllocator typed allocation") {
    ArenaAllocator arena(4096);

    SUBCASE("allocateArray returns correctly sized storage") {
        constexpr size_t count = 32;
        auto* arr = arena.allocateArray<float>(count);
        REQUIRE(arr != nullptr);
        for (size_t i = 0; i < count; ++i) arr[i] = static_cast<float>(i);
        for (size_t i = 0; i < count; ++i)
            CHECK(arr[i] == doctest::Approx(static_cast<float>(i)));
    }

    SUBCASE("allocateArray of struct") {
        struct Particle {
            float x, y, z;
            uint32_t id;
        };
        auto* particles = arena.allocateArray<Particle>(16);
        REQUIRE(particles != nullptr);
        for (uint32_t i = 0; i < 16; ++i) {
            particles[i].x = static_cast<float>(i);
            particles[i].id = i;
        }
        CHECK(particles[7].x == doctest::Approx(7.0f));
        CHECK(particles[15].id == 15u);
    }

    SUBCASE("zero-count allocateArray") {
        auto* ptr = arena.allocateArray<int>(0);
        // Zero-size allocation: may return non-null, must not crash.
        (void)ptr;
    }
}

TEST_CASE("ArenaAllocator move semantics") {
    SUBCASE("move constructor transfers ownership") {
        ArenaAllocator source(2048);
        auto* ptr = source.allocate(128);
        REQUIRE(ptr != nullptr);
        size_t usedBefore = source.used();

        ArenaAllocator dest(std::move(source));
        CHECK(dest.capacity() == 2048);
        CHECK(dest.used() == usedBefore);
        CHECK(source.capacity() == 0);
    }

    SUBCASE("move assignment transfers ownership") {
        ArenaAllocator source(1024);
        source.allocate(64);
        ArenaAllocator dest(512);

        dest = std::move(source);
        CHECK(dest.capacity() == 1024);
        CHECK(source.capacity() == 0);
    }
}

TEST_CASE("ArenaAllocator constructor validation") {
    CHECK_THROWS(ArenaAllocator(0));
}

TEST_CASE("ArenaAllocator thread-safe concurrent allocation") {
    // Multiple threads bump-allocate concurrently; verify no overlaps and
    // total usage is consistent.
    constexpr int kThreads = 4;
    constexpr int kAllocsPerThread = 500;
    constexpr size_t kAllocSize = 64;
    // 4 * 500 * 64 = 128000 bytes, plus alignment padding — 256 KiB is ample.
    ArenaAllocator arena(256 * 1024);

    std::vector<std::thread> threads;
    std::vector<std::vector<void*>> results(kThreads);
    std::atomic<bool> failed{false};

    for (int t = 0; t < kThreads; ++t) {
        threads.emplace_back([&, t] {
            results[t].reserve(kAllocsPerThread);
            for (int i = 0; i < kAllocsPerThread; ++i) {
                void* ptr = arena.allocate(kAllocSize, 16);
                if (!ptr) {
                    failed.store(true);
                    return;
                }
                // Write a thread-specific pattern to detect overlaps.
                std::memset(ptr, static_cast<int>(t + 1), kAllocSize);
                results[t].push_back(ptr);
            }
        });
    }
    for (auto& th : threads) th.join();

    REQUIRE_FALSE(failed.load());

    SUBCASE("no allocation returned nullptr") {
        for (int t = 0; t < kThreads; ++t)
            CHECK(results[t].size() == static_cast<size_t>(kAllocsPerThread));
    }

    SUBCASE("all pointers are unique (no double-allocation)") {
        std::vector<void*> all;
        for (auto& v : results)
            all.insert(all.end(), v.begin(), v.end());
        std::sort(all.begin(), all.end());
        auto dup = std::adjacent_find(all.begin(), all.end());
        CHECK(dup == all.end());
    }

    SUBCASE("written patterns are intact (no overlap corruption)") {
        for (int t = 0; t < kThreads; ++t) {
            uint8_t expected = static_cast<uint8_t>(t + 1);
            for (void* ptr : results[t]) {
                auto* bytes = static_cast<uint8_t*>(ptr);
                for (size_t b = 0; b < kAllocSize; ++b) {
                    if (bytes[b] != expected) {
                        FAIL("overlap detected");
                        break;
                    }
                }
            }
        }
    }

    SUBCASE("used bytes account for all allocations") {
        size_t minExpected = kThreads * kAllocsPerThread * kAllocSize;
        CHECK(arena.used() >= minExpected);
        CHECK(arena.used() <= arena.capacity());
    }
}

TEST_CASE("ArenaAllocator default capacity") {
    ArenaAllocator arena;
    CHECK(arena.capacity() == ArenaAllocator::kDefaultCapacity);
    CHECK(arena.capacity() == 4 * 1024 * 1024);
}

TEST_CASE("ArenaAllocator solver integration smoke test") {
    // Step a small scene through the public World API. The solver's island
    // and batch scratch is arena-backed; this verifies the integration path
    // does not crash and produces a stable simulation.
    World world(BackendType::Cpu);
    world.setGravity({0, -9.81f, 0});

    world.addStaticPlane({0, 1, 0}, 0.0f);
    BodyId sphere = world.addSphere({0, 3, 0}, 0.5f, 1.0f);

    const float dt = 1.0f / 60.0f;
    for (int i = 0; i < 120; ++i)
        world.step(dt);

    // After 2 seconds the sphere should have fallen and rested near y = 0.5.
    const Body& body = world.body(sphere);
    CHECK(body.position.y < 3.0f);
    CHECK(body.position.y > -1.0f);
}
