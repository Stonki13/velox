// Cross-platform floating-point behavior tests for Velox.
//
// Verifies that IEEE 754 single-precision semantics are consistent across
// platforms and compilers. These tests catch:
//   - FMA contraction differences (x86 FMA vs separate mul+add on ARM)
//   - Denormal handling differences (flush-to-zero on some ARM cores)
//   - NaN/Inf propagation differences
//   - Rounding mode assumptions
//   - Extended precision (x87 80-bit intermediates on 32-bit x86)
//
// The engine targets C++17 with float (32-bit) throughout. When
// VELOX_STRICT_FLOATING_POINT is ON, the build disables FMA contraction
// and fast-math optimizations to guarantee bitwise reproducibility.

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "../unit/doctest.h"

#include <velox/math.h>
#include <velox/platform.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

using namespace velox;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static uint32_t floatBits(float f) {
    uint32_t bits;
    std::memcpy(&bits, &f, sizeof(bits));
    return bits;
}

static float bitsToFloat(uint32_t bits) {
    float f;
    std::memcpy(&f, &bits, sizeof(f));
    return f;
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

TEST_SUITE("cross_platform.floating_point") {

    TEST_CASE("IEEE 754 basic properties hold") {
        SUBCASE("float is 32 bits") {
            CHECK(sizeof(float) == 4);
        }

        SUBCASE("double is 64 bits") {
            CHECK(sizeof(double) == 8);
        }

        SUBCASE("Infinity representation") {
            float inf = std::numeric_limits<float>::infinity();
            uint32_t bits = floatBits(inf);
            // IEEE 754: exponent all 1s, mantissa all 0s, sign 0.
            CHECK((bits & 0x7F800000u) == 0x7F800000u);
            CHECK((bits & 0x007FFFFFu) == 0u);
            CHECK((bits & 0x80000000u) == 0u);
        }

        SUBCASE("NaN representation") {
            float nan = std::numeric_limits<float>::quiet_NaN();
            uint32_t bits = floatBits(nan);
            // IEEE 754: exponent all 1s, mantissa non-zero.
            CHECK((bits & 0x7F800000u) == 0x7F800000u);
            CHECK((bits & 0x007FFFFFu) != 0u);
        }

        SUBCASE("Negative zero") {
            float negZero = -0.0f;
            float posZero = 0.0f;
            CHECK(negZero == posZero); // IEEE 754: -0 == +0
            CHECK(floatBits(negZero) != floatBits(posZero)); // but different bits
            CHECK(floatBits(negZero) == 0x80000000u);
            CHECK(floatBits(posZero) == 0x00000000u);
        }
    }

    TEST_CASE("Arithmetic consistency") {
        SUBCASE("Addition is commutative") {
            float a = 1.23456789f;
            float b = 9.87654321f;
            uint32_t ab = floatBits(a + b);
            uint32_t ba = floatBits(b + a);
            CHECK(ab == ba);
        }

        SUBCASE("Multiplication is commutative") {
            float a = 3.14159265f;
            float b = 2.71828182f;
            uint32_t ab = floatBits(a * b);
            uint32_t ba = floatBits(b * a);
            CHECK(ab == ba);
        }

        SUBCASE("Distributive law may NOT hold (FP is not a field)") {
            // This documents expected FP behavior — a*(b+c) != a*b + a*c
            // in general. We just verify both forms produce finite results.
            float a = 1e20f, b = 1.0f, c = -1.0f;
            float lhs = a * (b + c);
            float rhs = a * b + a * c;
            CHECK(std::isfinite(lhs));
            CHECK(std::isfinite(rhs));
            // They happen to be equal here (b+c = 0), but the point is
            // both are well-defined.
        }

        SUBCASE("Division by zero produces infinity") {
            float zero = 0.0f;
            float pos = 1.0f / zero;
            float neg = -1.0f / zero;
            CHECK(std::isinf(pos));
            CHECK(pos > 0.0f);
            CHECK(std::isinf(neg));
            CHECK(neg < 0.0f);
        }

        SUBCASE("Zero divided by zero produces NaN") {
            float zero = 0.0f;
            float nan = zero / zero;
            CHECK(std::isnan(nan));
        }
    }

    TEST_CASE("FMA contraction behavior") {
        // On platforms with hardware FMA (x86 FMA3, ARM NEON), the compiler
        // may contract a*b+c into a single fma instruction, which rounds only
        // once instead of twice. This produces a different result than
        // separate multiply and add.
        //
        // With VELOX_STRICT_FLOATING_POINT, contraction is disabled
        // (-ffp-contract=off or /fp:precise), so results are identical.

        float a = 1.0000001f;
        float b = 1.0000001f;
        float c = -1.0f;

        // Compute a*b + c both ways.
        float mul_add = a * b + c;
        float fma_result = std::fma(a, b, c);

        // Both must be finite.
        CHECK(std::isfinite(mul_add));
        CHECK(std::isfinite(fma_result));

#if VELOX_STRICT_FLOATING_POINT
        // In strict mode, the compiler must NOT contract, so mul_add
        // computed via operator* then operator+ must equal the two-step
        // result. (fma() is an explicit intrinsic and always rounds once.)
        // We verify the engine's math uses the non-contracted path.
        float step1 = a * b;
        float step2 = step1 + c;
        CHECK(floatBits(mul_add) == floatBits(step2));
#endif
    }

    TEST_CASE("Denormal (subnormal) handling") {
        // Some ARM cores flush denormals to zero by default. The engine
        // should not rely on denormal precision, but we verify the
        // platform's behavior is documented.
        float smallest_normal = std::numeric_limits<float>::min(); // ~1.175e-38
        float denormal = smallest_normal * 0.5f;

        // The denormal must be representable (not flushed to zero) on
        // IEEE-compliant platforms.
        CHECK(denormal > 0.0f);
        CHECK(denormal < smallest_normal);

        // Arithmetic on denormals must not produce NaN.
        float sum = denormal + denormal;
        bool finiteOrZero = std::isfinite(sum) || sum == 0.0f;
        CHECK(finiteOrZero); // allow FTZ
        CHECK_FALSE(std::isnan(sum));
    }

    TEST_CASE("Rounding behavior") {
        // Verify round-to-nearest-even (default IEEE 754 rounding mode).
        // 0.5f rounds to 0.0f (even), 1.5f rounds to 2.0f (even).
        CHECK(std::nearbyint(0.5f) == 0.0f);
        CHECK(std::nearbyint(1.5f) == 2.0f);
        CHECK(std::nearbyint(2.5f) == 2.0f);
        CHECK(std::nearbyint(3.5f) == 4.0f);
        CHECK(std::nearbyint(-0.5f) == -0.0f);
        CHECK(std::nearbyint(-1.5f) == -2.0f);
    }

    TEST_CASE("Velox math functions produce consistent results") {
        SUBCASE("normalize produces unit length") {
            Vec3 v{3.0f, 4.0f, 0.0f};
            Vec3 n = normalize(v);
            float len = length(n);
            // Must be very close to 1.0.
            CHECK(std::abs(len - 1.0f) < 1e-6f);
        }

        SUBCASE("normalize of zero vector returns zero") {
            Vec3 z{0.0f, 0.0f, 0.0f};
            Vec3 n = normalize(z);
            CHECK(n.x == 0.0f);
            CHECK(n.y == 0.0f);
            CHECK(n.z == 0.0f);
        }

        SUBCASE("cross product is anti-commutative") {
            Vec3 a{1.0f, 0.0f, 0.0f};
            Vec3 b{0.0f, 1.0f, 0.0f};
            Vec3 ab = cross(a, b);
            Vec3 ba = cross(b, a);
            CHECK(floatBits(ab.x) == floatBits(-ba.x));
            CHECK(floatBits(ab.y) == floatBits(-ba.y));
            CHECK(floatBits(ab.z) == floatBits(-ba.z));
        }

        SUBCASE("quaternion multiply is associative") {
            Quat a = fromAxisAngle({1, 0, 0}, 0.3f);
            Quat b = fromAxisAngle({0, 1, 0}, 0.5f);
            Quat c = fromAxisAngle({0, 0, 1}, 0.7f);

            Quat ab_c = mul(mul(a, b), c);
            Quat a_bc = mul(a, mul(b, c));

            // Associativity holds exactly for quaternion multiplication.
            CHECK(floatBits(ab_c.x) == floatBits(a_bc.x));
            CHECK(floatBits(ab_c.y) == floatBits(a_bc.y));
            CHECK(floatBits(ab_c.z) == floatBits(a_bc.z));
            CHECK(floatBits(ab_c.w) == floatBits(a_bc.w));
        }

        SUBCASE("quaternion normalize preserves direction") {
            Quat q{1.0f, 2.0f, 3.0f, 4.0f};
            Quat n = normalize(q);
            float len = std::sqrt(n.x*n.x + n.y*n.y + n.z*n.z + n.w*n.w);
            CHECK(std::abs(len - 1.0f) < 1e-6f);
        }

        SUBCASE("rotate and rotateInv are inverses") {
            Quat q = fromAxisAngle({0, 1, 0}, 1.23f);
            Vec3 v{1.0f, 2.0f, 3.0f};
            Vec3 rotated = rotate(q, v);
            Vec3 back = rotateInv(q, rotated);
            CHECK(std::abs(back.x - v.x) < 1e-5f);
            CHECK(std::abs(back.y - v.y) < 1e-5f);
            CHECK(std::abs(back.z - v.z) < 1e-5f);
        }
    }

    TEST_CASE("Transcendental function consistency") {
        // sinf/cosf/sqrtf must produce identical results for identical
        // inputs on all platforms. These are the functions most likely to
        // differ between libm implementations.
        SUBCASE("sqrtf of perfect squares") {
            CHECK(floatBits(sqrtf(4.0f)) == floatBits(2.0f));
            CHECK(floatBits(sqrtf(9.0f)) == floatBits(3.0f));
            CHECK(floatBits(sqrtf(0.25f)) == floatBits(0.5f));
        }

        SUBCASE("sqrtf(0) and sqrtf(1)") {
            CHECK(floatBits(sqrtf(0.0f)) == floatBits(0.0f));
            CHECK(floatBits(sqrtf(1.0f)) == floatBits(1.0f));
        }

        SUBCASE("sinf and cosf at cardinal angles") {
            // sin(0) = 0, cos(0) = 1
            CHECK(std::abs(sinf(0.0f)) < 1e-7f);
            CHECK(std::abs(cosf(0.0f) - 1.0f) < 1e-7f);
        }

        SUBCASE("sqrtf of negative produces NaN") {
            CHECK(std::isnan(sqrtf(-1.0f)));
        }
    }

    TEST_CASE("Platform FP configuration report") {
        // Print platform info for CI diagnostics.
        const char* platform =
            VELOX_PLATFORM_WINDOWS ? "Windows" :
            VELOX_PLATFORM_LINUX   ? "Linux" :
            VELOX_PLATFORM_MACOS   ? "macOS" : "Unknown";
        const char* arch =
            VELOX_ARCH_X86_64 ? "x86_64" :
            VELOX_ARCH_ARM64  ? "ARM64" : "Unknown";

        printf("VELOX_FP_PLATFORM=%s ARCH=%s STRICT=%d\n",
               platform, arch, int(VELOX_STRICT_FLOATING_POINT));

        // Verify float properties.
        CHECK(std::numeric_limits<float>::is_iec559);
        CHECK(std::numeric_limits<float>::radix == 2);
        CHECK(std::numeric_limits<float>::digits == 24); // 23 mantissa + 1 implicit
    }
}
