#include "doctest.h"
#include <velox/velox.h>
#include <cmath>

using namespace velox;

TEST_CASE("Raycast hits sphere") {
    World world(BackendType::Cpu);
    world.addSphere({0, 0, 0}, 1.0f, 1.0f);
    RayHit hit = world.rayCast({-5, 0, 0}, {1, 0, 0}, 20.0f);
    CHECK(hit.hit);
    CHECK(hit.point.x == doctest::Approx(-1.0f).epsilon(0.1f));
}

TEST_CASE("Raycast misses sphere") {
    World world(BackendType::Cpu);
    world.addSphere({0, 0, 0}, 1.0f, 1.0f);
    RayHit hit = world.rayCast({-5, 5, 0}, {1, 0, 0}, 20.0f);
    CHECK_FALSE(hit.hit);
}

TEST_CASE("Raycast hits plane") {
    World world(BackendType::Cpu);
    world.addStaticPlane({0, 1, 0}, 0.0f);
    RayHit hit = world.rayCast({0, 5, 0}, {0, -1, 0}, 20.0f);
    CHECK(hit.hit);
    CHECK(hit.point.y == doctest::Approx(0.0f).epsilon(0.1f));
}

TEST_CASE("Raycast hits box") {
    World world(BackendType::Cpu);
    world.addBox({0, 0, 0}, {1, 1, 1}, 1.0f);
    RayHit hit = world.rayCast({-5, 0, 0}, {1, 0, 0}, 20.0f);
    CHECK(hit.hit);
    CHECK(hit.point.x == doctest::Approx(-1.0f).epsilon(0.1f));
}

TEST_CASE("Overlap sphere finds bodies") {
    World world(BackendType::Cpu);
    world.addSphere({0, 0, 0}, 1.0f, 1.0f);
    world.addSphere({10, 0, 0}, 1.0f, 1.0f);

    std::vector<BodyId> overlaps;
    world.overlapSphere({0, 0, 0}, 3.0f, overlaps);
    CHECK(overlaps.size() >= 1);
}

TEST_CASE("Overlap sphere finds nothing far away") {
    World world(BackendType::Cpu);
    world.addSphere({0, 0, 0}, 1.0f, 1.0f);

    std::vector<BodyId> overlaps;
    world.overlapSphere({100, 0, 0}, 1.0f, overlaps);
    CHECK(overlaps.empty());
}

TEST_CASE("Sphere cast hits box") {
    World world(BackendType::Cpu);
    world.addBox({5, 0, 0}, {1, 1, 1}, 1.0f);
    ShapeCastHit hit = world.sphereCast({0, 0, 0}, 0.5f, {1, 0, 0}, 20.0f);
    CHECK(hit.hit);
    CHECK(hit.point.x < 5.0f);
}

TEST_CASE("Sphere cast misses when too far") {
    World world(BackendType::Cpu);
    world.addBox({5, 10, 0}, {1, 1, 1}, 1.0f);
    ShapeCastHit hit = world.sphereCast({0, 0, 0}, 0.5f, {1, 0, 0}, 20.0f);
    CHECK_FALSE(hit.hit);
}

TEST_CASE("RaycastAll returns multiple hits") {
    World world(BackendType::Cpu);
    world.addSphere({2, 0, 0}, 0.5f, 1.0f);
    world.addSphere({5, 0, 0}, 0.5f, 1.0f);
    world.addSphere({8, 0, 0}, 0.5f, 1.0f);

    std::vector<RayHit> hits;
    world.rayCastAll({0, 0, 0}, {1, 0, 0}, 20.0f, hits);
    CHECK(hits.size() >= 3);
}

TEST_CASE("Raycast respects max distance") {
    World world(BackendType::Cpu);
    world.addSphere({10, 0, 0}, 1.0f, 1.0f);
    RayHit hit = world.rayCast({0, 0, 0}, {1, 0, 0}, 5.0f);
    CHECK_FALSE(hit.hit);
}

TEST_CASE("Overlap box finds bodies") {
    World world(BackendType::Cpu);
    world.addSphere({0, 0, 0}, 0.5f, 1.0f);
    world.addSphere({10, 0, 0}, 0.5f, 1.0f);

    std::vector<BodyId> overlaps;
    world.overlapBox({0, 0, 0}, {2, 2, 2}, {}, overlaps);
    CHECK(overlaps.size() >= 1);
}
