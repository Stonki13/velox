#include "doctest.h"
#include <velox/math.h>
#include <cmath>

using namespace velox;

TEST_CASE("Vec3 arithmetic") {
    Vec3 a{1, 2, 3}, b{4, 5, 6};

    SUBCASE("addition") {
        Vec3 c = a + b;
        CHECK(c.x == 5.0f);
        CHECK(c.y == 7.0f);
        CHECK(c.z == 9.0f);
    }

    SUBCASE("subtraction") {
        Vec3 c = a - b;
        CHECK(c.x == -3.0f);
        CHECK(c.y == -3.0f);
        CHECK(c.z == -3.0f);
    }

    SUBCASE("scalar multiply") {
        Vec3 c = a * 2.0f;
        CHECK(c.x == 2.0f);
        CHECK(c.y == 4.0f);
        CHECK(c.z == 6.0f);
    }

    SUBCASE("negation") {
        Vec3 c = -a;
        CHECK(c.x == -1.0f);
        CHECK(c.y == -2.0f);
        CHECK(c.z == -3.0f);
    }

    SUBCASE("compound assignment") {
        Vec3 c = a;
        c += b;
        CHECK(c.x == 5.0f);
        c -= Vec3{1, 1, 1};
        CHECK(c.x == 4.0f);
        c *= 0.5f;
        CHECK(c.x == 2.0f);
    }
}

TEST_CASE("Vec3 dot and cross") {
    Vec3 a{1, 0, 0}, b{0, 1, 0};

    CHECK(dot(a, b) == 0.0f);
    CHECK(dot(a, a) == 1.0f);

    Vec3 c = cross(a, b);
    CHECK(c.x == 0.0f);
    CHECK(c.y == 0.0f);
    CHECK(c.z == 1.0f);

    Vec3 d = cross(b, a);
    CHECK(d.z == -1.0f);
}

TEST_CASE("Vec3 length and normalize") {
    Vec3 v{3, 4, 0};
    CHECK(lengthSq(v) == 25.0f);
    CHECK(length(v) == 5.0f);

    Vec3 n = normalize(v);
    CHECK(n.x == doctest::Approx(0.6f));
    CHECK(n.y == doctest::Approx(0.8f));
    CHECK(n.z == 0.0f);

    Vec3 zero{};
    Vec3 nz = normalize(zero);
    CHECK(nz.x == 0.0f);
    CHECK(nz.y == 0.0f);
    CHECK(nz.z == 0.0f);
}

TEST_CASE("Vec3 min/max/clamp") {
    CHECK(vmin(1.0f, 2.0f) == 1.0f);
    CHECK(vmax(1.0f, 2.0f) == 2.0f);
    CHECK(vclamp(5.0f, 0.0f, 3.0f) == 3.0f);
    CHECK(vclamp(-1.0f, 0.0f, 3.0f) == 0.0f);
    CHECK(vclamp(1.5f, 0.0f, 3.0f) == 1.5f);

    Vec3 a{1, 5, 3}, b{4, 2, 6};
    Vec3 lo = vmin(a, b);
    CHECK(lo.x == 1.0f);
    CHECK(lo.y == 2.0f);
    CHECK(lo.z == 3.0f);

    Vec3 hi = vmax(a, b);
    CHECK(hi.x == 4.0f);
    CHECK(hi.y == 5.0f);
    CHECK(hi.z == 6.0f);
}

TEST_CASE("Quat identity and multiplication") {
    Quat id{};
    CHECK(id.w == 1.0f);
    CHECK(id.x == 0.0f);

    Quat r = mul(id, id);
    CHECK(r.w == 1.0f);
    CHECK(r.x == 0.0f);
}

TEST_CASE("Quat rotate identity") {
    Quat id{};
    Vec3 v{1, 2, 3};
    Vec3 r = rotate(id, v);
    CHECK(r.x == doctest::Approx(v.x));
    CHECK(r.y == doctest::Approx(v.y));
    CHECK(r.z == doctest::Approx(v.z));
}

TEST_CASE("Quat fromAxisAngle 90 degrees around Z") {
    Quat q = fromAxisAngle({0, 0, 1}, 3.14159265f * 0.5f);
    Vec3 v{1, 0, 0};
    Vec3 r = rotate(q, v);
    CHECK(r.x == doctest::Approx(0.0f).epsilon(1e-5f));
    CHECK(r.y == doctest::Approx(1.0f).epsilon(1e-5f));
    CHECK(r.z == doctest::Approx(0.0f).epsilon(1e-5f));
}

TEST_CASE("Quat rotateInv is inverse of rotate") {
    Quat q = fromAxisAngle({0, 1, 0}, 1.2f);
    Vec3 v{3, 4, 5};
    Vec3 rotated = rotate(q, v);
    Vec3 back = rotateInv(q, rotated);
    CHECK(back.x == doctest::Approx(v.x).epsilon(1e-5f));
    CHECK(back.y == doctest::Approx(v.y).epsilon(1e-5f));
    CHECK(back.z == doctest::Approx(v.z).epsilon(1e-5f));
}

TEST_CASE("Quat normalize") {
    Quat q{1, 1, 1, 1};
    Quat n = normalize(q);
    float len = std::sqrt(n.x * n.x + n.y * n.y + n.z * n.z + n.w * n.w);
    CHECK(len == doctest::Approx(1.0f).epsilon(1e-6f));
}

TEST_CASE("Quat integrate preserves norm") {
    Quat q = fromAxisAngle({0, 1, 0}, 0.5f);
    Vec3 omega{0, 2, 0};
    Quat q2 = integrate(q, omega, 0.1f);
    float len = std::sqrt(q2.x * q2.x + q2.y * q2.y + q2.z * q2.z + q2.w * q2.w);
    CHECK(len == doctest::Approx(1.0f).epsilon(1e-5f));
}

TEST_CASE("Quat multiplication is non-commutative") {
    Quat qx = fromAxisAngle({1, 0, 0}, 1.0f);
    Quat qy = fromAxisAngle({0, 1, 0}, 1.0f);
    Quat xy = mul(qx, qy);
    Quat yx = mul(qy, qx);
    CHECK((xy.x != doctest::Approx(yx.x).epsilon(1e-3f) ||
           xy.y != doctest::Approx(yx.y).epsilon(1e-3f) ||
           xy.z != doctest::Approx(yx.z).epsilon(1e-3f)));
}
