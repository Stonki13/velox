#include "velox/velox.h"
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>
#include <vector>

namespace {

bool sameHit(const velox::RayHit& a, const velox::RayHit& b) {
    return a.hit == b.hit && a.body == b.body &&
           std::fabs(a.t - b.t) < 1e-5f;
}

bool fail(const char* message) {
    std::printf("batched_queries_demo: FAIL: %s\n", message);
    return false;
}

} // namespace

int main() {
    using namespace velox;
    World world(BackendType::Cpu);
    const BodyId ground = world.addStaticPlane({0, 1, 0}, 0.0f);
    const BodyId box = world.addBox({0, 1, 0}, {0.5f, 0.5f, 0.5f}, 1.0f);
    world.step(0.0f); // Establish the initial broad-phase snapshot.

    QueryDesc ray;
    ray.type = QueryDesc::Type::Raycast;
    ray.origin = {0, 4, 0};
    ray.direction = {0, -1, 0};
    ray.maxDist = 10.0f;
    ray.userData = reinterpret_cast<void*>(1);

    QueryDesc overlap;
    overlap.type = QueryDesc::Type::OverlapSphere;
    overlap.center = {0, 1, 0};
    overlap.radius = 0.75f;
    overlap.userData = reinterpret_cast<void*>(2);

    QueryDesc cast;
    cast.type = QueryDesc::Type::SphereCast;
    cast.center = {0, 4, 0};
    cast.radius = 0.25f;
    cast.direction = {0, -1, 0};
    cast.maxDist = 10.0f;
    cast.userData = reinterpret_cast<void*>(3);

    QueryDesc invalid = ray;
    invalid.direction = {};
    invalid.userData = reinterpret_cast<void*>(4);

    const RayHit directRay = world.rayCast(ray.origin, ray.direction, ray.maxDist);
    std::vector<BodyId> directOverlap;
    world.overlapSphere(overlap.center, overlap.radius, directOverlap);
    const ShapeCastHit directCast = world.sphereCast(
        cast.center, cast.radius, cast.direction, cast.maxDist);

    std::vector<QueryResult> batch;
    world.batchQueries({ray, overlap, cast, invalid}, batch);
    if (batch.size() != 4) return fail("result count changed");
    if (!batch[0].success || batch[0].userData != ray.userData ||
        !sameHit(batch[0].rayHit, directRay))
        return fail("batched raycast differs from direct raycast");
    if (!batch[1].success || batch[1].overlaps != directOverlap ||
        batch[1].overlaps.empty() || batch[1].overlaps.front() != box)
        return fail("batched overlap differs from direct overlap");
    if (!batch[2].success || batch[2].shapeCastHit.hit != directCast.hit ||
        batch[2].shapeCastHit.body != directCast.body ||
        std::fabs(batch[2].shapeCastHit.distance - directCast.distance) > 1e-5f)
        return fail("batched shape cast differs from direct shape cast");
    if (batch[3].success || batch[3].error.empty() || batch[3].userData != invalid.userData)
        return fail("invalid query did not fail independently");

    QueryDesc asyncRay = ray;
    asyncRay.filter.ignoredBody = box;
    const RayHit directAsyncRay = world.rayCast(asyncRay.origin, asyncRay.direction,
                                                asyncRay.maxDist, asyncRay.filter);
    world.setTransform(box, {0, 0.5f, 0}, {});

    AsyncQueryHandle handle;
    std::atomic<double> submissionMs{1000.0};
    std::atomic<bool> contactModifierEntered{false};
    std::atomic<bool> submitted{false};
    std::atomic<bool> modifierTimedOut{false};
    std::atomic<bool> workerTimedOut{false};
    world.setContactModifier([&](ContactModifyData&) {
        contactModifierEntered.store(true, std::memory_order_release);
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::milliseconds(500);
        while (!submitted.load(std::memory_order_acquire) &&
               std::chrono::steady_clock::now() < deadline)
            std::this_thread::yield();
        modifierTimedOut.store(!submitted.load(std::memory_order_acquire),
                               std::memory_order_release);
    });
    std::thread worker([&] {
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::seconds(1);
        while (!contactModifierEntered.load(std::memory_order_acquire) &&
               std::chrono::steady_clock::now() < deadline)
            std::this_thread::yield();
        if (!contactModifierEntered.load(std::memory_order_acquire)) {
            workerTimedOut.store(true, std::memory_order_release);
            return;
        }
        const auto start = std::chrono::steady_clock::now();
        handle = world.submitAsyncQuery(asyncRay);
        const auto end = std::chrono::steady_clock::now();
        submissionMs.store(std::chrono::duration<double, std::milli>(end - start).count());
        submitted.store(true, std::memory_order_release);
    });
    // The modifier is invoked under the World step lock. Its handshake proves
    // that submission completes while the owner thread is actively stepping.
    world.step(1.0f / 60.0f);
    worker.join();
    world.setContactModifier({});
    if (workerTimedOut.load()) return fail("contact modifier was not reached during step");
    // The request was enqueued after this step's queue drain, so the following
    // zero-dt frame resolves it without changing simulation state.
    world.step(0.0f);
    const QueryResult async = world.getAsyncResult(handle);
    if (modifierTimedOut.load() || submissionMs.load() > 100.0 || !async.success ||
        !sameHit(async.rayHit, directAsyncRay))
        return fail("async query was not non-blocking or did not match direct raycast");

    std::printf("batched_queries_demo: PASS (ground=%llu, box=%llu)\n",
                static_cast<unsigned long long>(ground.value),
                static_cast<unsigned long long>(box.value));
    return 0;
}
