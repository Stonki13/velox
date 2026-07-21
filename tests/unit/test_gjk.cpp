#include "doctest.h"
#include "gjk.h"
#include <cmath>

using namespace velox;

static Convex makeSphere(Vec3 pos, float r) {
    Convex c{};
    c.kind = Convex::Point;
    c.position = pos;
    c.radius = r;
    c.boundingRadius = 0.0f;
    return c;
}

static Convex makeBox(Vec3 pos, Vec3 half) {
    Convex c{};
    c.kind = Convex::Box;
    c.position = pos;
    c.halfExtents = half;
    c.radius = 0.0f;
    c.boundingRadius = length(half);
    return c;
}

static Convex makeSegment(Vec3 a, Vec3 b, float r) {
    Convex c{};
    c.kind = Convex::Segment;
    c.segA = a;
    c.segB = b;
    c.position = (a + b) * 0.5f;
    c.radius = r;
    c.boundingRadius = 0.5f * length(b - a);
    return c;
}

TEST_CASE("GJK sphere-sphere separated") {
    Convex a = makeSphere({0, 0, 0}, 1.0f);
    Convex b = makeSphere({5, 0, 0}, 1.0f);
    GjkResult r = gjkDistance(a, b);
    CHECK_FALSE(r.overlapping);
    CHECK(r.distance == doctest::Approx(3.0f).epsilon(1e-4f));
    CHECK(r.normal.x == doctest::Approx(-1.0f).epsilon(1e-3f));
}

TEST_CASE("GJK sphere-sphere overlapping") {
    Convex a = makeSphere({0, 0, 0}, 1.0f);
    Convex b = makeSphere({0.5f, 0, 0}, 1.0f);
    GjkResult r = gjkDistance(a, b);
    CHECK(r.distance < 0.0f);
}

TEST_CASE("GJK box-box separated") {
    Convex a = makeBox({0, 0, 0}, {1, 1, 1});
    Convex b = makeBox({5, 0, 0}, {1, 1, 1});
    GjkResult r = gjkDistance(a, b);
    CHECK_FALSE(r.overlapping);
    CHECK(r.distance == doctest::Approx(3.0f).epsilon(1e-3f));
}

TEST_CASE("GJK box-box touching") {
    Convex a = makeBox({0, 0, 0}, {1, 1, 1});
    Convex b = makeBox({2, 0, 0}, {1, 1, 1});
    GjkResult r = gjkDistance(a, b);
    CHECK(r.distance == doctest::Approx(0.0f).epsilon(1e-2f));
}

TEST_CASE("GJK sphere-box separated") {
    Convex a = makeSphere({0, 0, 0}, 0.5f);
    Convex b = makeBox({5, 0, 0}, {1, 1, 1});
    GjkResult r = gjkDistance(a, b);
    CHECK_FALSE(r.overlapping);
    CHECK(r.distance == doctest::Approx(3.5f).epsilon(1e-3f));
}

TEST_CASE("GJK capsule-capsule separated") {
    Convex a = makeSegment({0, -1, 0}, {0, 1, 0}, 0.5f);
    Convex b = makeSegment({5, -1, 0}, {5, 1, 0}, 0.5f);
    GjkResult r = gjkDistance(a, b);
    CHECK_FALSE(r.overlapping);
    CHECK(r.distance == doctest::Approx(4.0f).epsilon(1e-3f));
}

TEST_CASE("GJK identical positions overlap") {
    Convex a = makeBox({0, 0, 0}, {1, 1, 1});
    Convex b = makeBox({0, 0, 0}, {1, 1, 1});
    GjkResult r = gjkDistance(a, b);
    CHECK(r.overlapping);
}

TEST_CASE("Convex support function box") {
    Convex box = makeBox({0, 0, 0}, {1, 2, 3});
    Vec3 s = box.supportOffset({1, 0, 0});
    CHECK(s.x == doctest::Approx(1.0f));

    Vec3 s2 = box.supportOffset({-1, -1, -1});
    CHECK(s2.x == doctest::Approx(-1.0f));
    CHECK(s2.y == doctest::Approx(-2.0f));
    CHECK(s2.z == doctest::Approx(-3.0f));
}

TEST_CASE("Convex support function sphere") {
    Convex sphere = makeSphere({3, 0, 0}, 2.0f);
    Vec3 s = sphere.support({1, 0, 0});
    CHECK(s.x == doctest::Approx(3.0f));
    CHECK(s.y == doctest::Approx(0.0f));

    Vec3 s2 = sphere.support({-1, 0, 0});
    CHECK(s2.x == doctest::Approx(3.0f));
}

TEST_CASE("GJK seeded matches unseeded") {
    Convex a = makeBox({0, 0, 0}, {1, 1, 1});
    Convex b = makeBox({4, 2, 0}, {0.5f, 0.5f, 0.5f});
    GjkResult r1 = gjkDistance(a, b);
    GjkResult r2 = gjkDistanceSeeded(a, b, {1, 0, 0});
    CHECK(r1.distance == doctest::Approx(r2.distance).epsilon(1e-4f));
    CHECK(r1.overlapping == r2.overlapping);
}
