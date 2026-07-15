// Randomized property fuzzing and long-duration soak testing.
//
//   fuzz_demo [scenes] [seed]   — randomized scenes, invariant checks
//   fuzz_demo soak [steps]      — one dense scene stepped for a long time
//
// Every scene is generated from a deterministic LCG, so a failure report's
// (scene, seed) pair reproduces exactly. Checked invariants:
//   1. every body state stays finite (positions, orientations, velocities)
//   2. nothing tunnels through the ground plane (the engine's core promise)
//   3. kinetic energy stays bounded (no solver energy injection)
//   4. identical (seed, scene) runs produce identical trajectories on the
//      serial CPU backend (local determinism)
//   5. soak: piles eventually sleep and stay asleep, with zero drift after
#include <velox/velox.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

namespace {

struct Rng {
    uint32_t state;
    explicit Rng(uint32_t seed) : state(seed ? seed : 1u) {}
    float next() { // [0, 1)
        state = state * 1664525u + 1013904223u;
        return (state >> 8) * (1.0f / 16777216.0f);
    }
    float range(float lo, float hi) { return lo + (hi - lo) * next(); }
    int pick(int count) { return (int)(next() * count) % count; }
};

velox::Vec3 randomUnit(Rng& rng) {
    velox::Vec3 v{rng.range(-1, 1), rng.range(-1, 1), rng.range(-1, 1)};
    float len = velox::length(v);
    return len > 1e-4f ? v * (1.0f / len) : velox::Vec3{0, 1, 0};
}

struct BodyRecord {
    velox::BodyId id;
    float extent; // conservative shape bounding radius
};

// Builds a random scene; returns the max initial speed for the energy bound.
float buildScene(velox::World& w, Rng& rng, std::vector<BodyRecord>& bodies) {
    w.gravity = {0, rng.range(-25.0f, -1.0f), 0};
    w.addStaticPlane({0, 1, 0}, 0.0f);

    int count = 8 + rng.pick(28);
    float maxSpeed = 0.0f;
    for (int i = 0; i < count; ++i) {
        velox::Vec3 pos{rng.range(-12, 12), rng.range(1.5f, 25.0f), rng.range(-12, 12)};
        float mass = rng.range(0.05f, 20.0f);
        velox::BodyId id;
        float extent;
        switch (rng.pick(5)) {
        case 0: {
            float r = rng.range(0.1f, 1.2f);
            id = w.addSphere(pos, r, mass);
            extent = r;
            break;
        }
        case 1: {
            velox::Vec3 h{rng.range(0.1f, 1.0f), rng.range(0.1f, 1.0f),
                          rng.range(0.1f, 1.0f)};
            id = w.addBox(pos, h, mass);
            extent = velox::length(h);
            break;
        }
        case 2: {
            float r = rng.range(0.1f, 0.6f), hh = rng.range(0.1f, 1.0f);
            id = w.addCapsule(pos, r, hh, mass);
            extent = r + hh;
            break;
        }
        case 3: {
            float r = rng.range(0.15f, 0.8f), hh = rng.range(0.15f, 0.8f);
            id = w.addCylinder(pos, r, hh, mass);
            extent = std::sqrt(r * r + hh * hh);
            break;
        }
        default: {
            // Random tetrahedron-based hull, jittered.
            float s = rng.range(0.3f, 1.0f);
            std::vector<velox::Vec3> pts = {
                {s, s, s}, {s, -s, -s}, {-s, s, -s}, {-s, -s, s}};
            for (velox::Vec3& p : pts) {
                p.x += rng.range(-0.1f, 0.1f) * s;
                p.y += rng.range(-0.1f, 0.1f) * s;
                p.z += rng.range(-0.1f, 0.1f) * s;
            }
            id = w.addConvexHull(pos, pts, mass);
            extent = 2.0f * s;
            break;
        }
        }
        // Mostly moderate speeds; every 6th body is a high-speed CCD probe.
        float speed = (i % 6 == 5) ? rng.range(50.0f, 400.0f)
                                   : rng.range(0.0f, 12.0f);
        w.setLinearVelocity(id, randomUnit(rng) * speed);
        w.setAngularVelocity(id, randomUnit(rng) * rng.range(0.0f, 20.0f));
        if (speed > maxSpeed) maxSpeed = speed;
        bodies.push_back({id, extent});
    }

    // A few random joints between random pairs.
    int jointCount = rng.pick(5);
    for (int j = 0; j < jointCount && bodies.size() >= 2; ++j) {
        int ia = rng.pick((int)bodies.size());
        int ib = rng.pick((int)bodies.size());
        if (ia == ib) continue;
        velox::Vec3 mid = (w.body(bodies[ia].id).position +
                           w.body(bodies[ib].id).position) * 0.5f;
        switch (rng.pick(3)) {
        case 0: w.addBallJoint(bodies[ia].id, bodies[ib].id, mid); break;
        case 1:
            w.addDistanceJoint(bodies[ia].id, bodies[ib].id,
                               w.body(bodies[ia].id).position,
                               w.body(bodies[ib].id).position);
            break;
        default:
            w.addHingeJoint(bodies[ia].id, bodies[ib].id, mid, randomUnit(rng));
            break;
        }
    }
    return maxSpeed;
}

bool finite(const velox::Vec3& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

struct SceneResult {
    bool ok = true;
    const char* failure = nullptr;
    int failStep = -1;
    int failBody = -1;
    std::vector<velox::Vec3> finalPositions; // determinism fingerprint
};

SceneResult runScene(uint32_t seed, int steps) {
    SceneResult result;
    velox::World w(velox::BackendType::Cpu);
    w.setWorkerCount(1);
    Rng rng(seed);
    std::vector<BodyRecord> bodies;
    float maxSpeed = buildScene(w, rng, bodies);

    // Energy bound: initial kinetic + everything gravity can add by falling
    // from the highest spawn, with generous slack for solver transients.
    float gMag = velox::length(w.gravity);
    float speedBound = (maxSpeed + std::sqrt(2.0f * gMag * 40.0f)) * 3.0f + 50.0f;

    for (int s = 0; s < steps; ++s) {
        w.step(1.0f / 60.0f);
        for (size_t index = 0; index < bodies.size(); ++index) {
            const BodyRecord& record = bodies[index];
            const velox::Body& b = w.body(record.id);
            if (!finite(b.position) || !finite(b.velocity) ||
                !finite(b.angularVelocity) || !std::isfinite(b.orientation.w)) {
                result.ok = false;
                result.failure = "non-finite body state";
                result.failStep = s;
                result.failBody = (int)index;
                return result;
            }
            // Tunneling: a body's center below the plane by more than its
            // bounding extent means it passed through the floor.
            if (b.position.y < -(record.extent + 0.05f)) {
                result.ok = false;
                result.failure = "tunneled through ground plane";
                result.failStep = s;
                result.failBody = (int)index;
                std::printf("  body shape=%u p=(%.3f %.3f %.3f) v=(%.3f %.3f %.3f)\n",
                            (unsigned)b.shape, b.position.x, b.position.y, b.position.z,
                            b.velocity.x, b.velocity.y, b.velocity.z);
                return result;
            }
            if (velox::length(b.velocity) > speedBound) {
                result.ok = false;
                result.failure = "energy explosion (speed bound exceeded)";
                result.failStep = s;
                result.failBody = (int)index;
                std::printf("  body shape=%u p=(%.3f %.3f %.3f) v=(%.3f %.3f %.3f)\n",
                            (unsigned)b.shape, b.position.x, b.position.y, b.position.z,
                            b.velocity.x, b.velocity.y, b.velocity.z);
                return result;
            }
        }
    }
    result.finalPositions.reserve(bodies.size());
    for (const BodyRecord& record : bodies)
        result.finalPositions.push_back(w.body(record.id).position);
    return result;
}

int runFuzz(int scenes, uint32_t baseSeed) {
    int failures = 0;
    for (int scene = 0; scene < scenes; ++scene) {
        uint32_t seed = baseSeed + (uint32_t)scene * 2654435761u;
        SceneResult a = runScene(seed, 150);
        if (!a.ok) {
            std::printf("FUZZ FAIL scene=%d seed=%u step=%d body=%d: %s\n",
                        scene, seed, a.failStep, a.failBody, a.failure);
            ++failures;
            continue;
        }
        // Determinism: bitwise-identical replay on the serial CPU backend.
        SceneResult b = runScene(seed, 150);
        bool identical = b.ok && a.finalPositions.size() == b.finalPositions.size();
        for (size_t i = 0; identical && i < a.finalPositions.size(); ++i)
            identical = std::memcmp(&a.finalPositions[i], &b.finalPositions[i],
                                    sizeof(velox::Vec3)) == 0;
        if (!identical) {
            std::printf("FUZZ FAIL scene=%d seed=%u: non-deterministic replay\n",
                        scene, seed);
            ++failures;
        }
    }
    std::printf("fuzz: %d scenes, %d failures\n", scenes, failures);
    return failures;
}

int runSoak(int steps) {
    velox::World w(velox::BackendType::Cpu);
    w.setWorkerCount(1);
    w.addStaticPlane({0, 1, 0}, 0.0f);
    velox::Vec3 walls[4] = {{1, 0.3f, 0}, {-1, 0.3f, 0}, {0, 0.3f, 1}, {0, 0.3f, -1}};
    for (const velox::Vec3& n : walls) w.addStaticPlane(n, -8.0f);

    std::vector<velox::BodyId> bodies;
    Rng rng(0xC0FFEE);
    for (int i = 0; i < 60; ++i) {
        velox::Vec3 pos{rng.range(-5, 5), 1.0f + i * 0.6f, rng.range(-5, 5)};
        if (i % 3 == 0) {
            bodies.push_back(w.addBox(pos, {0.4f, 0.4f, 0.4f}, 1.0f));
        } else {
            // Without rolling friction a sphere on a flat floor rolls forever
            // (physically correct — pure rolling dissipates nothing) and the
            // pile can never sleep. Real spheres have rolling resistance.
            velox::BodyId id = w.addSphere(pos, 0.45f, 1.0f);
            w.body(id).rollingFriction = 0.05f;
            bodies.push_back(id);
        }
        // Game-typical damping. A freely rolling sphere dissipates almost
        // nothing (v = wr, friction does no work), so lone spheres coast for
        // sim-minutes and keep themselves awake; games damp them out.
        w.body(bodies.back()).linearDamping = 0.15f;
        w.body(bodies.back()).angularDamping = 0.15f;
    }

    int firstAllAsleep = -1;
    std::vector<velox::Vec3> sleepPositions;
    for (int s = 0; s < steps; ++s) {
        w.step(1.0f / 60.0f);
        bool allAsleep = true;
        for (velox::BodyId id : bodies) {
            const velox::Body& b = w.body(id);
            if (!finite(b.position) || !finite(b.velocity)) {
                std::printf("SOAK FAIL step=%d: non-finite state\n", s);
                return 1;
            }
            if (b.position.y < -0.6f) {
                std::printf("SOAK FAIL step=%d: body sank through floor (y=%f)\n",
                            s, b.position.y);
                return 1;
            }
            if (w.isAwake(id)) allAsleep = false;
        }
        if (s % 1000 == 999 && firstAllAsleep < 0) {
            int awake = 0;
            float maxV = 0, maxW = 0, minY = 1e9f;
            for (velox::BodyId id : bodies) {
                const velox::Body& b = w.body(id);
                if (w.isAwake(id)) ++awake;
                maxV = std::fmax(maxV, velox::length(b.velocity));
                maxW = std::fmax(maxW, velox::length(b.angularVelocity));
                minY = std::fmin(minY, b.position.y);
            }
            std::printf("  soak step %d: awake=%d maxV=%.4f maxW=%.4f minY=%.3f\n",
                        s, awake, maxV, maxW, minY);
        }
        if (allAsleep && firstAllAsleep < 0) {
            firstAllAsleep = s;
            for (velox::BodyId id : bodies)
                sleepPositions.push_back(w.body(id).position);
        }
        // Once asleep, nothing may drift: sleeping must be genuinely stable.
        if (firstAllAsleep >= 0 && allAsleep) {
            for (size_t i = 0; i < bodies.size(); ++i) {
                velox::Vec3 d = w.body(bodies[i]).position - sleepPositions[i];
                if (velox::length(d) > 1e-3f) {
                    std::printf("SOAK FAIL step=%d: drift while asleep (%f)\n",
                                s, velox::length(d));
                    return 1;
                }
            }
        }
    }
    if (firstAllAsleep < 0) {
        std::printf("SOAK FAIL: pile never fell asleep in %d steps\n", steps);
        for (size_t i = 0; i < bodies.size(); ++i) {
            const velox::Body& b = w.body(bodies[i]);
            if (!w.isAwake(bodies[i])) continue;
            std::printf("  awake[%zu] %s p=(%.2f %.2f %.2f) |v|=%.4f |w|=%.4f\n",
                        i, b.shape == velox::ShapeType::Box ? "box" : "sphere",
                        b.position.x, b.position.y, b.position.z,
                        velox::length(b.velocity), velox::length(b.angularVelocity));
        }
        return 1;
    }
    std::printf("soak: %d steps, pile slept at step %d, no drift after\n",
                steps, firstAllAsleep);
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    if (argc > 1 && std::strcmp(argv[1], "soak") == 0) {
        int steps = argc > 2 ? std::atoi(argv[2]) : 6000;
        return runSoak(steps > 0 ? steps : 6000);
    }
    int scenes = argc > 1 ? std::atoi(argv[1]) : 40;
    uint32_t seed = argc > 2 ? (uint32_t)std::strtoul(argv[2], nullptr, 10)
                             : 0x51CE5EEDu;
    return runFuzz(scenes > 0 ? scenes : 40, seed);
}
