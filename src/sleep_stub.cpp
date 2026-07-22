// Minimal SleepManager stubs to satisfy the linker until sleep.cpp lands.
#include "velox/sleep.h"

namespace velox {

void SleepManager::update(std::vector<Body>& bodies,
                          const std::vector<Contact>& contacts,
                          const std::vector<Joint>& joints,
                          std::vector<uint32_t>& unionParent,
                          std::vector<float>& islandTimer,
                          float dt,
                          const std::function<BodyId(BodyIndex)>& handleResolver) {
    constexpr float kMotionTol = 2.5e-3f;
    constexpr float kTimeToSleep = 0.5f;
    size_t n = bodies.size();
    unionParent.resize(n);
    islandTimer.resize(n, 0.0f);
    for (uint32_t i = 0; i < n; ++i) {
        if (!bodies[i].isDynamic()) continue;
        float motion = lengthSq(bodies[i].velocity) +
                       lengthSq(bodies[i].angularVelocity);
        if (motion > kMotionTol) {
            islandTimer[i] = 0.0f;
            if (bodies[i].asleep) bodies[i].asleep = 0;
        } else {
            islandTimer[i] += dt;
            if (islandTimer[i] >= kTimeToSleep) {
                bodies[i].asleep = 1;
                bodies[i].velocity = {};
                bodies[i].angularVelocity = {};
            }
        }
    }
}

void SleepManager::wakeBody(Body& body, BodyId id) {
    if (body.asleep) {
        body.asleep = 0;
        body.sleepTimer = 0.0f;
    }
}

void SleepManager::sleepBody(Body& body, BodyId id) {
    body.asleep = 1;
    body.velocity = {};
    body.angularVelocity = {};
}

} // namespace velox
