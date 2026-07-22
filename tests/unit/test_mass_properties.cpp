#include "doctest.h"
#include "mass_properties.h"
#include "velox/error.h"
#include <cmath>

using namespace velox;
using namespace velox::mass_properties;

TEST_CASE("convex hull of a tetrahedron") {
    std::vector<Vec3> pts = {
        {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}
    };
    auto tris = convexTriangles(pts);
    CHECK(tris.size() == 4);
}

TEST_CASE("convex mass properties of a unit cube") {
    std::vector<Vec3> pts = {
        {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0},
        {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}
    };
    ConvexMassProperties mp = convex(pts);

    CHECK(mp.volume == doctest::Approx(1.0).epsilon(1e-6));
    CHECK(mp.center.x == doctest::Approx(0.5).epsilon(1e-4));
    CHECK(mp.center.y == doctest::Approx(0.5).epsilon(1e-4));
    CHECK(mp.center.z == doctest::Approx(0.5).epsilon(1e-4));

    float I = 1.0f / 6.0f;
    CHECK(mp.principalInertia.x == doctest::Approx(I).epsilon(1e-3));
    CHECK(mp.principalInertia.y == doctest::Approx(I).epsilon(1e-3));
    CHECK(mp.principalInertia.z == doctest::Approx(I).epsilon(1e-3));
}

TEST_CASE("Jacobi diagonalize diagonal matrix") {
    Matrix3 m{};
    m.m[0][0] = 3.0;
    m.m[1][1] = 1.0;
    m.m[2][2] = 2.0;
    Vec3 moments;
    Quat orient;
    jacobiDiagonalize(m, moments, orient);

    CHECK(moments.x == doctest::Approx(3.0f).epsilon(1e-4f));
    CHECK(moments.y == doctest::Approx(1.0f).epsilon(1e-4f));
    CHECK(moments.z == doctest::Approx(2.0f).epsilon(1e-4f));
}

TEST_CASE("Jacobi diagonalize symmetric matrix") {
    Matrix3 m{};
    m.m[0][0] = 2.0; m.m[0][1] = 1.0; m.m[0][2] = 0.0;
    m.m[1][0] = 1.0; m.m[1][1] = 3.0; m.m[1][2] = 0.0;
    m.m[2][0] = 0.0; m.m[2][1] = 0.0; m.m[2][2] = 1.0;

    Vec3 moments;
    Quat orient;
    jacobiDiagonalize(m, moments, orient);

    float trace = m.m[0][0] + m.m[1][1] + m.m[2][2];
    float sumMoments = moments.x + moments.y + moments.z;
    CHECK(sumMoments == doctest::Approx(trace).epsilon(1e-4f));
}

TEST_CASE("parallel axis theorem addAtOffset") {
    Matrix3 agg{};
    Matrix3 centered{};
    centered.m[0][0] = 1.0;
    centered.m[1][1] = 1.0;
    centered.m[2][2] = 1.0;

    addAtOffset(agg, centered, 2.0, {3, 0, 0});

    // I_xx unchanged (offset along x): I_xx += m*(y^2+z^2) = 0
    CHECK(agg.m[0][0] == doctest::Approx(1.0).epsilon(1e-10));
    // I_yy += m*(x^2+z^2) = 2*9 = 18
    CHECK(agg.m[1][1] == doctest::Approx(19.0).epsilon(1e-10));
    // I_zz += m*(x^2+y^2) = 2*9 = 18
    CHECK(agg.m[2][2] == doctest::Approx(19.0).epsilon(1e-10));
}

TEST_CASE("shiftedToCenter reverses addAtOffset for center shift") {
    Matrix3 centered{};
    centered.m[0][0] = 2.0;
    centered.m[1][1] = 3.0;
    centered.m[2][2] = 4.0;

    Matrix3 agg{};
    Vec3 offset{1, 2, 3};
    addAtOffset(agg, centered, 5.0, offset);

    Matrix3 back = shiftedToCenter(agg, 5.0, offset);
    CHECK(back.m[0][0] == doctest::Approx(2.0).epsilon(1e-8));
    CHECK(back.m[1][1] == doctest::Approx(3.0).epsilon(1e-8));
    CHECK(back.m[2][2] == doctest::Approx(4.0).epsilon(1e-8));
}

TEST_CASE("collinear points throw") {
    std::vector<Vec3> pts = {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}};
    CHECK_THROWS_AS(convexTriangles(pts), VeloxInvalidArgument);
}

TEST_CASE("coplanar points throw") {
    std::vector<Vec3> pts = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0}};
    CHECK_THROWS_AS(convexTriangles(pts), VeloxInvalidArgument);
}
