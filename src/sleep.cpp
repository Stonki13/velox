#include "velox/sleep.h"
#include "velox/backend.h"
#include "velox/joint.h"
#include <algorithm>
#include <chrono>
#include <cmath>

namespace velox {

// ---------------------------------------------------------------------------
// Forced state transitions
// ---------------------------------------------------------------------------

void SleepManager::wakeBody(Body& body, BodyId id) {
    SleepState prev = sleepStateFromByte(body.asleep);
    if (prev == SleepState::Awake) return;
    body.asleep = sleepStateToByte(SleepState::Awake);
    body.sleepTimer = 0.0f;
    ++stats_.totalWakeTransitions;
    if (callbacks_.onWake) callbacks_.onWake(id);
}

void SleepManager::sleepBody(Body& body, BodyId id) {
    SleepState prev = sleepStateFromByte(body.asleep);
    if (prev == SleepState::Asleep) return;
    body.asleep = sleepStateToByte(SleepState::Asleep);
    body.velocity = {};
    body.angularVelocity = {};
    body.sleepTimer = 0.0f;
    ++stats_.totalSleepTransitions;
    if (callbacks_.onSleep) callbacks_.onSleep(id);
}

// ---------------------------------------------------------------------------
// Reset
// ---------------------------------------------------------------------------

void SleepManager::reset() {
    contactStability_.clear();
    drowsyPhase_.clear();
    islands_.clear();
    stepCounter_ = 0;
    stats_ = {};
}

// ---------------------------------------------------------------------------
// Main update
// ---------------------------------------------------------------------------

void SleepManager::update(std::vector<Body>& bodies,
                          const std::vector<Contact>& contacts,
                          const std::vector<Joint>& joints,
                          std::vector<uint32_t>& unionParent,
                          std::vector<float>& islandTimer,
                          float dt,
                          const std::function<BodyId(BodyIndex)>& handleResolver) {
    using Clock = std::chrono::steady_clock;
    const auto t0 = Clock::now();

    const size_t n = bodies.size();
    ++stepCounter_;

    // Ensure per-body auxiliary arrays are sized.
    contactStability_.resize(n, 0);
    drowsyPhase_.resize(n, 0);

    // -----------------------------------------------------------------------
    // 1. Union-find island formation over dynamic-dynamic contacts and joints.
    // -----------------------------------------------------------------------
    unionParent.resize(n);
    for (uint32_t i = 0; i < n; ++i) unionParent[i] = i;

    auto find = [&](uint32_t x) {
        while (unionParent[x] != x) {
            unionParent[x] = unionParent[unionParent[x]]; // path halving
            x = unionParent[x];
        }
        return x;
    };
    auto unite = [&](uint32_t x, uint32_t y) {
        x = find(x); y = find(y);
        if (x != y) unionParent[x] = y;
    };

    for (const Contact& c : contacts) {
        if (c.a >= n || c.b >= n) continue;
        const Body& ba = bodies[c.a];
        const Body& bb = bodies[c.b];
        if (ba.isSensor() || bb.isSensor()) continue;
        if (ba.isDynamic() && bb.isDynamic()) unite(c.a, c.b);
    }
    for (const Joint& j : joints) {
        if (j.a >= n || j.b >= n) continue;
        if (bodies[j.a].isDynamic() && bodies[j.b].isDynamic()) unite(j.a, j.b);
    }

    // -----------------------------------------------------------------------
    // 2. Contact stability tracking.
    //
    // For each dynamic body, count how many of its contacts are "stable"
    // (relative velocity at the contact point is below the threshold). A body
    // with at least one stable contact and no unstable contacts increments its
    // stability counter; otherwise the counter resets.
    // -----------------------------------------------------------------------
    if (config_.enableContactStability) {
        // Per-body: has any contact, all contacts stable.
        std::vector<uint8_t> hasContact(n, 0);
        std::vector<uint8_t> allStable(n, 1);

        for (const Contact& c : contacts) {
            if (c.a >= n || c.b >= n) continue;
            const Body& ba = bodies[c.a];
            const Body& bb = bodies[c.b];
            if (ba.isSensor() || bb.isSensor()) continue;

            // Relative velocity at the contact point.
            Vec3 va = ba.velocity + cross(ba.angularVelocity, c.point - ba.position);
            Vec3 vb = bb.velocity + cross(bb.angularVelocity, c.point - bb.position);
            float relSpeed = length(va - vb);

            if (ba.isDynamic()) {
                hasContact[c.a] = 1;
                if (relSpeed > config_.contactStabilityThreshold) allStable[c.a] = 0;
            }
            if (bb.isDynamic()) {
                hasContact[c.b] = 1;
                if (relSpeed > config_.contactStabilityThreshold) allStable[c.b] = 0;
            }
        }

        for (uint32_t i = 0; i < n; ++i) {
            if (!bodies[i].isDynamic()) continue;
            if (hasContact[i] && allStable[i]) {
                if (contactStability_[i] < UINT16_MAX)
                    ++contactStability_[i];
            } else {
                contactStability_[i] = 0;
            }
        }
    }

    // -----------------------------------------------------------------------
    // 3. Per-body motion evaluation and sleep timer accumulation.
    //
    // A body is "calm" when:
    //   |v|² < linearThreshold²  AND
    //   |ω|² < angularThreshold² AND
    //   |a|² < accelThreshold²   (a = force * invMass)
    // AND (if contact stability is enabled) the body has enough stable frames.
    // -----------------------------------------------------------------------
    const float linTolSq = config_.linearVelocityThreshold * config_.linearVelocityThreshold;
    const float angTolSq = config_.angularVelocityThreshold * config_.angularVelocityThreshold;
    const float accTolSq = config_.accelerationThreshold * config_.accelerationThreshold;

    for (uint32_t i = 0; i < n; ++i) {
        Body& b = bodies[i];
        if (!b.isDynamic() || velox::isFullyAsleep(b.asleep) || !b.enableSleep) continue;

        float linSq = lengthSq(b.velocity);
        float angSq = lengthSq(b.angularVelocity);
        // Acceleration from accumulated forces (force is cleared each step,
        // so this reflects the current step's applied force).
        float accSq = b.invMass > 0.0f
            ? lengthSq(b.force) * b.invMass * b.invMass
            : 0.0f;

        bool calm = linSq < linTolSq && angSq < angTolSq && accSq < accTolSq;

        // Contact stability gate: require enough consecutive stable frames.
        if (calm && config_.enableContactStability) {
            calm = contactStability_[i] >= uint16_t(config_.contactStabilityFrames);
        }

        if (calm) {
            b.sleepTimer += dt;
        } else {
            b.sleepTimer = 0.0f;
            // Wake drowsy bodies that are no longer calm.
            if (velox::isDrowsy(b.asleep)) {
                b.asleep = sleepStateToByte(SleepState::Awake);
                ++stats_.totalWakeTransitions;
                if (handleResolver && callbacks_.onWake)
                    callbacks_.onWake(handleResolver(i));
            }
        }
    }

    // -----------------------------------------------------------------------
    // 4. Island-level aggregation: minimum timer and maximum motion per island.
    // -----------------------------------------------------------------------
    islandTimer.assign(n, 1e30f);
    std::vector<float> islandMaxMotion(n, 0.0f);
    std::vector<uint32_t> islandBodyCount(n, 0);

    for (uint32_t i = 0; i < n; ++i) {
        Body& b = bodies[i];
        if (!b.isDynamic() || velox::isFullyAsleep(b.asleep)) continue;
        uint32_t root = find(i);
        if (!b.enableSleep) {
            // A non-sleepable dynamic body forces its entire island awake:
            // reset the island timer so connected sleepable bodies cannot
            // transition while this body is still part of the island.
            islandTimer[root] = 0.0f;
            ++islandBodyCount[root];
            continue;
        }
        if (b.sleepTimer < islandTimer[root]) islandTimer[root] = b.sleepTimer;
        float motion = lengthSq(b.velocity) + lengthSq(b.angularVelocity);
        if (motion > islandMaxMotion[root]) islandMaxMotion[root] = motion;
        ++islandBodyCount[root];
    }

    // -----------------------------------------------------------------------
    // 5. State transitions: drowsy and sleep.
    //
    // An island transitions when its minimum timer crosses the threshold:
    //   - timeToDrowsy → Drowsy (if gradual sleep enabled)
    //   - timeToSleep  → Asleep
    //
    // All members of the island transition together.
    // -----------------------------------------------------------------------
    const bool gradual = config_.enableGradualSleep;
    const float tDrowsy = gradual ? config_.timeToDrowsy : config_.timeToSleep;
    const float tSleep = config_.timeToSleep;

    for (uint32_t i = 0; i < n; ++i) {
        Body& b = bodies[i];
        if (!b.isDynamic() || velox::isFullyAsleep(b.asleep) || !b.enableSleep) continue;

        uint32_t root = find(i);
        float minTimer = islandTimer[root];

        if (minTimer >= tSleep) {
            // Full sleep.
            SleepState prev = sleepStateFromByte(b.asleep);
            b.asleep = sleepStateToByte(SleepState::Asleep);
            b.velocity = {};
            b.angularVelocity = {};
            b.sleepTimer = 0.0f;
            if (prev != SleepState::Asleep) {
                ++stats_.totalSleepTransitions;
                if (handleResolver && callbacks_.onSleep)
                    callbacks_.onSleep(handleResolver(i));
            }
        } else if (gradual && minTimer >= tDrowsy && !velox::isDrowsy(b.asleep)) {
            // Enter drowsy state.
            b.asleep = sleepStateToByte(SleepState::Drowsy);
            drowsyPhase_[i] = 0;
            ++stats_.totalDrowsyTransitions;
            if (handleResolver && callbacks_.onDrowsy)
                callbacks_.onDrowsy(handleResolver(i));
        }
    }

    // -----------------------------------------------------------------------
    // 6. Build island snapshots and compute statistics.
    // -----------------------------------------------------------------------
    islands_.clear();
    stats_.totalDynamicBodies = 0;
    stats_.awakeBodies = 0;
    stats_.drowsyBodies = 0;
    stats_.sleepingBodies = 0;
    stats_.islandCount = 0;
    stats_.sleepingIslandCount = 0;
    stats_.drowsyIslandCount = 0;
    stats_.awakeIslandCount = 0;

    // Count per-body states.
    for (uint32_t i = 0; i < n; ++i) {
        const Body& b = bodies[i];
        if (!b.isDynamic()) continue;
        ++stats_.totalDynamicBodies;
        SleepState s = sleepStateFromByte(b.asleep);
        switch (s) {
        case SleepState::Awake:  ++stats_.awakeBodies; break;
        case SleepState::Drowsy: ++stats_.drowsyBodies; break;
        case SleepState::Asleep: ++stats_.sleepingBodies; break;
        }
    }

    // Build island list from roots that have dynamic bodies.
    // Single-pass aggregation: accumulate state flags per root, then build
    // islands from the accumulated data (O(n) instead of the previous O(n²)).
    std::vector<uint8_t> rootHasAwake(n, 0), rootHasDrowsy(n, 0);
    std::vector<uint32_t> rootTotalBodies(n, 0);
    for (uint32_t i = 0; i < n; ++i) {
        if (!bodies[i].isDynamic()) continue;
        uint32_t root = find(i);
        ++rootTotalBodies[root];
        SleepState s = sleepStateFromByte(bodies[i].asleep);
        if (s == SleepState::Awake) rootHasAwake[root] = 1;
        else if (s == SleepState::Drowsy) rootHasDrowsy[root] = 1;
    }

    std::vector<uint8_t> rootSeen(n, 0);
    for (uint32_t i = 0; i < n; ++i) {
        const Body& b = bodies[i];
        if (!b.isDynamic()) continue;
        uint32_t root = find(i);
        if (rootSeen[root]) continue;
        rootSeen[root] = 1;

        SleepIsland island;
        island.root = root;
        island.bodyCount = rootTotalBodies[root];
        island.minSleepTimer = islandTimer[root];
        island.maxMotion = islandMaxMotion[root];

        if (rootHasAwake[root]) island.state = SleepState::Awake;
        else if (rootHasDrowsy[root]) island.state = SleepState::Drowsy;
        else island.state = SleepState::Asleep;

        ++stats_.islandCount;
        if (island.state == SleepState::Asleep) ++stats_.sleepingIslandCount;
        else if (island.state == SleepState::Drowsy) ++stats_.drowsyIslandCount;
        else ++stats_.awakeIslandCount;

        islands_.push_back(island);
    }

    stats_.lastUpdateMs = std::chrono::duration<double, std::milli>(
        Clock::now() - t0).count();
}

} // namespace velox
