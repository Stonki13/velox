#include <velox/velox.h>

#include <cstdio>
#include <thread>

int main() {
    velox::World world(velox::BackendType::Cpu);
    world.setThreadSafetyPolicy(velox::ThreadSafetyPolicy::Relaxed);
    world.addStaticPlane({0, 1, 0}, 0.0f);

    bool hit = false;
    std::thread queryThread([&] {
        hit = world.rayCast({0, 2, 0}, {0, -1, 0}, 4.0f).hit;
    });
    queryThread.join();

    std::printf("multi_threaded: %s\n", hit ? "PASS" : "FAIL");
    return hit ? 0 : 1;
}
