#include "velox/velox.h"
#include <atomic>
#include <cmath>
#include <cstdio>
#include <exception>
#include <thread>
#include <vector>

namespace {

bool finite(const velox::Vec3& value) {
    return std::isfinite(value.x) && std::isfinite(value.y) &&
           std::isfinite(value.z);
}

bool expect(bool condition, const char* message) {
    if (condition) return true;
    std::fprintf(stderr, "thread_safety_demo: %s\n", message);
    return false;
}

} // namespace

int main() {
    bool ok = true;
    velox::World world(velox::BackendType::Cpu);
    world.addStaticPlane({0, 1, 0}, 0.0f);
    const velox::BodyId sphere = world.addSphere({0, 3, 0}, 0.5f, 1.0f);

    std::atomic<bool> strictRejected{false};
    std::thread strictQuery([&] {
        try {
            (void)world.rayCast({0, 6, 0}, {0, -1, 0}, 12.0f);
        } catch (const std::logic_error&) {
            strictRejected = true;
        }
    });
    strictQuery.join();
    ok &= expect(strictRejected, "Strict policy must reject cross-thread queries");

    world.setThreadSafetyPolicy(velox::ThreadSafetyPolicy::Relaxed);
    std::atomic<bool> relaxedQueryHit{false};
    std::thread relaxedQuery([&] {
        relaxedQueryHit = world.rayCast({0, 6, 0}, {0, -1, 0}, 12.0f).hit;
    });
    relaxedQuery.join();
    ok &= expect(relaxedQueryHit, "Relaxed policy must permit cross-thread queries");

    std::atomic<bool> relaxedMutationRejected{false};
    std::thread relaxedMutation([&] {
        try {
            world.setLinearVelocity(sphere, {1, 0, 0});
        } catch (const std::logic_error&) {
            relaxedMutationRejected = true;
        }
    });
    relaxedMutation.join();
    ok &= expect(relaxedMutationRejected,
                 "Relaxed policy must reject cross-thread mutations");

    world.setThreadSafetyPolicy(velox::ThreadSafetyPolicy::Concurrent);
    std::atomic<bool> run{true};
    std::atomic<uint32_t> failures{0};
    std::vector<std::thread> readers;
    readers.reserve(8);
    for (int index = 0; index < 8; ++index) {
        readers.emplace_back([&] {
            try {
                while (run.load(std::memory_order_relaxed)) {
                    std::vector<velox::BodyId> overlaps;
                    (void)world.rayCast({0, 8, 0}, {0, -1, 0}, 16.0f);
                    world.overlapSphere({0, 1, 0}, 4.0f, overlaps);
                    if (!finite(world.bodyState(sphere).position))
                        ++failures;
                }
            } catch (const std::exception&) {
                ++failures;
            }
        });
    }

    for (int frame = 0; frame < 240; ++frame) {
        world.setLinearVelocity(sphere, {0.25f, 0.0f, 0.0f});
        world.step(1.0f / 120.0f);
    }
    run = false;
    for (std::thread& reader : readers) reader.join();

    const velox::ThreadSafetyReport report = world.threadSafetyReport();
    ok &= expect(failures == 0, "concurrent query traffic must not fail");
    ok &= expect(report.queryCallsFromNonMainThread > 0,
                 "thread safety report must record cross-thread queries");
    ok &= expect(finite(world.bodyState(sphere).position),
                 "thread-safe body-state copy must remain finite");

    if (!ok) return 1;
    std::puts("thread_safety_demo: PASS");
    return 0;
}
