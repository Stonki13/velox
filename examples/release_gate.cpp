#include <velox/velox.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>

namespace {

using velox::BodyId;
using velox::Vec3;
using velox::World;

bool finite(Vec3 value) {
    return std::isfinite(value.x) && std::isfinite(value.y) &&
           std::isfinite(value.z);
}

void addTerrain(World& world) {
    world.addStaticPlane({0, 1, 0}, 0.0f);
    constexpr uint32_t width = 33, depth = 33;
    std::vector<float> heights(width * depth);
    for (uint32_t z = 0; z < depth; ++z)
        for (uint32_t x = 0; x < width; ++x)
            heights[z * width + x] = 0.15f * std::sin(float(x) * 0.35f) *
                                     std::cos(float(z) * 0.27f);
    world.addStaticHeightfield(width, depth, 1.5f, heights, {-24, 0, -24});

    // A raised mesh bridge exercises BVH-backed mesh contact and casts.
    const std::vector<Vec3> vertices = {
        {-5, 2.0f, -2}, {5, 2.0f, -2}, {-5, 2.0f, 2}, {5, 2.0f, 2}};
    const std::vector<uint32_t> indices = {0, 2, 1, 1, 2, 3};
    world.addStaticMesh(vertices, indices);
}

void addStructures(World& world) {
    for (int level = 0; level < 5; ++level)
        for (int column = 0; column < 4; ++column)
            world.addBox({8.0f + float(column) * 0.72f, 0.35f + level * 0.72f, 0},
                         {0.32f, 0.35f, 0.32f}, 1.2f);

    const BodyId ramp = world.addBox({-8, 1.5f, 0}, {4.0f, 0.25f, 2.0f}, 0.0f);
    world.body(ramp).orientation = velox::fromAxisAngle({0, 0, 1}, -0.35f);
}

void addJoints(World& world, std::vector<BodyId>& tracked) {
    const BodyId anchor = world.addBox({-2, 7, 0}, {0.2f, 0.2f, 0.2f}, 0.0f);
    BodyId previous = anchor;
    for (int i = 0; i < 6; ++i) {
        BodyId limb = world.addCapsule({-2, 6.2f - float(i) * 0.8f, 0},
                                       0.16f, 0.28f, 2.0f);
        world.addBallJoint(previous, limb, {-2, 6.6f - float(i) * 0.8f, 0});
        previous = limb;
        tracked.push_back(limb);
    }

    const BodyId pendulumAnchor = world.addBox({3, 7, 0}, {0.2f, 0.2f, 0.2f}, 0.0f);
    const BodyId bob = world.addSphere({4.8f, 5.8f, 0}, 0.35f, 3.0f);
    world.addHingeJoint(pendulumAnchor, bob, {3, 7, 0}, {0, 0, 1});
    tracked.push_back(bob);

    const BodyId platformAnchor = world.addBox({0, 6, 7}, {0.2f, 0.2f, 0.2f}, 0.0f);
    const BodyId platform = world.addBox({0, 4.5f, 7}, {1.5f, 0.15f, 1.5f}, 20.0f);
    world.addSpringJoint(platformAnchor, platform, {0, 6, 7}, {0, 4.5f, 7}, 2.0f, 0.8f);
    tracked.push_back(platform);
}

} // namespace

int main() {
    World world(velox::BackendType::Cpu);
    world.setWorkerCount(0);
    world.gravity = {0, -9.81f, 0};
    addTerrain(world);
    addStructures(world);

    std::vector<BodyId> dynamic;
    dynamic.reserve(64);
    for (int i = 0; i < 50; ++i) {
        const float x = -12.0f + float(i % 10) * 2.3f;
        const float y = 4.0f + float(i / 10) * 1.35f;
        const float z = -8.0f + float((i * 7) % 9) * 1.8f;
        BodyId id;
        if (i % 3 == 0) id = world.addSphere({x, y, z}, 0.25f + 0.04f * (i % 4), 0.5f + i);
        else if (i % 3 == 1) id = world.addBox({x, y, z}, {0.25f, 0.35f, 0.3f}, 1.0f + i);
        else id = world.addCapsule({x, y, z}, 0.2f, 0.35f, 1.0f + i);
        if (i % 11 == 0) world.setLinearVelocity(id, {35.0f, 0.0f, 0.0f});
        dynamic.push_back(id);
    }
    addJoints(world, dynamic);

    bool queryRay = false, queryOverlap = false, queryCast = false;
    bool valid = true;
    double totalMs = 0.0;
    constexpr int frames = 3600;
    for (int frame = 0; frame < frames; ++frame) {
        world.step(1.0f / 60.0f);
        totalMs += world.lastStepStats().totalMs;
        for (BodyId id : dynamic) {
            const Vec3 position = world.body(id).position;
            valid = valid && finite(position) && position.y > -1.0f;
        }
        if (frame == 1800) {
            queryRay = world.rayCast({8.5f, 20, 0}, {0, -1, 0}, 30.0f).hit;
            std::vector<BodyId> overlaps;
            world.overlapSphere(world.body(dynamic.back()).position, 5.0f, overlaps);
            queryOverlap = !overlaps.empty();
            queryCast = world.sphereCast({-15, 2.0f, 0}, 0.25f, {1, 0, 0}, 30.0f).hit;
        }
    }

    size_t asleep = 0;
    for (BodyId id : dynamic) asleep += !world.isAwake(id);
    const double meanMs = totalMs / frames;
    const bool stable = valid && queryRay && queryOverlap && queryCast;
    const bool performance = meanMs < 10.0;
    std::printf("release_gate: bodies=%zu asleep=%zu mean=%.3fms ray=%s overlap=%s cast=%s\n",
                dynamic.size(), asleep, meanMs, queryRay ? "yes" : "no",
                queryOverlap ? "yes" : "no", queryCast ? "yes" : "no");
    if (!stable || !performance) {
        std::fprintf(stderr, "release_gate: %s%s\n", stable ? "" : "stability/query failure ",
                     performance ? "" : "performance budget exceeded");
        return 1;
    }
    std::puts("release_gate: PASS");
    return 0;
}
