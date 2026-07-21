#include "doctest.h"
#include "narrowphase.h"
#include <cmath>

using namespace velox;

TEST_CASE("isConvexVolume classifies shapes correctly") {
    CHECK(isConvexVolume(ShapeType::Sphere));
    CHECK(isConvexVolume(ShapeType::Box));
    CHECK(isConvexVolume(ShapeType::Capsule));
    CHECK(isConvexVolume(ShapeType::Hull));
    CHECK(isConvexVolume(ShapeType::Cylinder));
    CHECK(isConvexVolume(ShapeType::Cone));
    CHECK_FALSE(isConvexVolume(ShapeType::Plane));
    CHECK_FALSE(isConvexVolume(ShapeType::Mesh));
    CHECK_FALSE(isConvexVolume(ShapeType::Compound));
}

TEST_CASE("aabbOverlap basic cases") {
    Vec3 lo1{-1, -1, -1}, hi1{1, 1, 1};
    Vec3 lo2{0, 0, 0}, hi2{2, 2, 2};
    CHECK(aabbOverlap(lo1, hi1, lo2, hi2));

    Vec3 lo3{5, 5, 5}, hi3{6, 6, 6};
    CHECK_FALSE(aabbOverlap(lo1, hi1, lo3, hi3));
}

TEST_CASE("aabbOverlap touching faces") {
    Vec3 lo1{-1, -1, -1}, hi1{1, 1, 1};
    Vec3 lo2{1, 0, 0}, hi2{2, 1, 1};
    CHECK(aabbOverlap(lo1, hi1, lo2, hi2));
}

TEST_CASE("aabbOverlap is symmetric") {
    Vec3 lo1{-1, -1, -1}, hi1{1, 1, 1};
    Vec3 lo2{0.5f, 0.5f, 0.5f}, hi2{3, 3, 3};
    CHECK(aabbOverlap(lo1, hi1, lo2, hi2) == aabbOverlap(lo2, hi2, lo1, hi1));
}

TEST_CASE("contactTangents produces orthonormal frame") {
    Vec3 n{0, 1, 0};
    Vec3 t1, t2;
    contactTangents(n, t1, t2);

    CHECK(dot(t1, n) == doctest::Approx(0.0f).epsilon(1e-6f));
    CHECK(dot(t2, n) == doctest::Approx(0.0f).epsilon(1e-6f));
    CHECK(dot(t1, t2) == doctest::Approx(0.0f).epsilon(1e-6f));
    CHECK(length(t1) == doctest::Approx(1.0f).epsilon(1e-6f));
    CHECK(length(t2) == doctest::Approx(1.0f).epsilon(1e-6f));
}

TEST_CASE("contactTangents for each axis") {
    for (Vec3 n : {Vec3{1, 0, 0}, Vec3{0, 1, 0}, Vec3{0, 0, 1}}) {
        Vec3 t1, t2;
        contactTangents(n, t1, t2);
        CHECK(dot(t1, n) == doctest::Approx(0.0f).epsilon(1e-6f));
        CHECK(dot(t2, n) == doctest::Approx(0.0f).epsilon(1e-6f));
        CHECK(length(t1) == doctest::Approx(1.0f).epsilon(1e-5f));
        CHECK(length(t2) == doctest::Approx(1.0f).epsilon(1e-5f));
    }
}

TEST_CASE("combineMaterial modes") {
    SUBCASE("Average") {
        float r = combineMaterial(0.2f, 0.8f,
            MaterialCombineMode::Average, MaterialCombineMode::Average);
        CHECK(r == doctest::Approx(0.5f));
    }

    SUBCASE("GeometricMean") {
        float r = combineMaterial(4.0f, 9.0f,
            MaterialCombineMode::GeometricMean, MaterialCombineMode::GeometricMean);
        CHECK(r == doctest::Approx(6.0f).epsilon(1e-5f));
    }

    SUBCASE("Minimum") {
        float r = combineMaterial(0.3f, 0.7f,
            MaterialCombineMode::Minimum, MaterialCombineMode::Minimum);
        CHECK(r == doctest::Approx(0.3f));
    }

    SUBCASE("Maximum") {
        float r = combineMaterial(0.3f, 0.7f,
            MaterialCombineMode::Maximum, MaterialCombineMode::Maximum);
        CHECK(r == doctest::Approx(0.7f));
    }

    SUBCASE("Multiply") {
        float r = combineMaterial(0.5f, 0.4f,
            MaterialCombineMode::Multiply, MaterialCombineMode::Multiply);
        CHECK(r == doctest::Approx(0.2f));
    }

    SUBCASE("higher mode wins") {
        float r = combineMaterial(0.2f, 0.8f,
            MaterialCombineMode::Average, MaterialCombineMode::Maximum);
        CHECK(r == doctest::Approx(0.8f));
    }
}

TEST_CASE("bodyAabb for sphere") {
    Body b{};
    b.shape = ShapeType::Sphere;
    b.position = {0, 5, 0};
    b.radius = 1.0f;
    b.ccdTuning.enableContinuous = false;
    b.ccdTuning.collisionMargin = 0.0f;

    Vec3 lo, hi;
    bodyAabb(b, 0.0f, lo, hi);
    CHECK(lo.x == doctest::Approx(-1.01f).epsilon(1e-2f));
    CHECK(hi.y == doctest::Approx(6.01f).epsilon(1e-2f));
}

TEST_CASE("bodyAabb for box") {
    Body b{};
    b.shape = ShapeType::Box;
    b.position = {0, 0, 0};
    b.halfExtents = {2, 3, 4};
    b.ccdTuning.enableContinuous = false;
    b.ccdTuning.collisionMargin = 0.0f;

    Vec3 lo, hi;
    bodyAabb(b, 0.0f, lo, hi);
    float ext = length(b.halfExtents);
    CHECK(lo.x == doctest::Approx(-ext - 0.01f).epsilon(1e-2f));
    CHECK(hi.y == doctest::Approx(ext + 0.01f).epsilon(1e-2f));
}

TEST_CASE("npPointVelocity static body") {
    Body b{};
    b.position = {0, 0, 0};
    b.velocity = {1, 0, 0};
    b.angularVelocity = {0, 0, 0};

    Vec3 v = npPointVelocity(b, {5, 0, 0});
    CHECK(v.x == doctest::Approx(1.0f));
    CHECK(v.y == doctest::Approx(0.0f));
}

TEST_CASE("npPointVelocity with angular velocity") {
    Body b{};
    b.position = {0, 0, 0};
    b.velocity = {};
    b.angularVelocity = {0, 0, 1};

    Vec3 v = npPointVelocity(b, {1, 0, 0});
    CHECK(v.x == doctest::Approx(0.0f).epsilon(1e-6f));
    CHECK(v.y == doctest::Approx(1.0f).epsilon(1e-6f));
}
