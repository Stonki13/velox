#include "doctest.h"
#include <velox/velox.h>
#include <cmath>

using namespace velox;

static void stepN(World& w, int n) {
    for (int i = 0; i < n; ++i) w.step(1.0f / 60.0f);
}

TEST_CASE("SoftBody: cloth drapes over a sphere") {
    World w(BackendType::Cpu);
    auto obstacle = w.addSphere({0, 0, 0}, 1.0f, 0.0f); // static sphere
    w.setMotionType(obstacle, MotionType::Static);

    auto desc = makeClothSoftBody(10, 10, 0.3f, 1.0f, 1e-5f, false);
    // Start the cloth above the sphere.
    for (auto& p : desc.positions) p.y += 2.0f;
    auto cloth = w.addSoftBody(desc);

    stepN(w, 300); // 5 seconds to drape

    const SoftBody& sb = w.softBody(cloth);
    // Behavioral invariant: the cloth's lowest particle should be near or
    // below the sphere's equator (it draped down the sides).
    float minY = sb.positions[0].y;
    for (const auto& p : sb.positions) minY = vmin(minY, p.y);
    CHECK(minY < 0.5f);

    // No particle should be inside the sphere (collision works).
    for (const auto& p : sb.positions) {
        float d = length(p);
        CHECK(d >= 0.95f); // small tolerance for XPBD penetration
    }
}

TEST_CASE("SoftBody: soft sphere bounces and settles on a plane") {
    World w(BackendType::Cpu);
    w.addStaticPlane({0, 1, 0}, 0.0f);

    auto desc = makeSoftSphereSoftBody(0.5f, 42, 1.0f, 1e-5f);
    for (auto& p : desc.positions) p.y += 3.0f;
    auto ball = w.addSoftBody(desc);

    stepN(w, 900); // 15 seconds to bounce and settle

    const SoftBody& sb = w.softBody(ball);
    // All particles should be above the plane.
    for (const auto& p : sb.positions)
        CHECK(p.y >= -0.05f);

    // The soft sphere should have settled: max velocity near zero.
    float maxVel = 0.0f;
    for (const auto& v : sb.velocities)
        maxVel = vmax(maxVel, length(v));
    CHECK(maxVel < 1.0f);
}

TEST_CASE("SoftBody: pinned cloth stays pinned") {
    World w(BackendType::Cpu);
    auto desc = makeClothSoftBody(5, 5, 0.5f, 1.0f, 1e-5f, true);
    for (auto& p : desc.positions) p.y += 3.0f;
    auto cloth = w.addSoftBody(desc);

    Vec3 corner0 = w.softBody(cloth).positions[0];
    stepN(w, 120);

    // Pinned corners must not move.
    CHECK(length(w.softBody(cloth).positions[0] - corner0) < 1e-4f);
}

TEST_CASE("SoftBody: handle lifecycle") {
    World w(BackendType::Cpu);
    auto desc = makeClothSoftBody(3, 3, 1.0f);
    auto id = w.addSoftBody(desc);
    CHECK(w.isValid(id));
    CHECK(w.softBodyCount() == 1);

    w.removeSoftBody(id);
    CHECK_FALSE(w.isValid(id));
    CHECK(w.softBodyCount() == 0);
}

TEST_CASE("SoftBody: factory produces valid constraints") {
    auto cloth = makeClothSoftBody(4, 4, 1.0f);
    CHECK(cloth.positions.size() == 16);
    CHECK(cloth.invMasses.size() == 16);
    CHECK(cloth.constraints.size() > 0);
    for (const auto& c : cloth.constraints) {
        CHECK(c.a < cloth.positions.size());
        CHECK(c.b < cloth.positions.size());
        CHECK(c.restLength > 0.0f);
    }

    auto sphere = makeSoftSphereSoftBody(1.0f, 42);
    CHECK(sphere.positions.size() == 43); // 42 surface + 1 core
    CHECK(sphere.constraints.size() > 42); // at least core-to-surface
}
