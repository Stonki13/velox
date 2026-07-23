#pragma once

// Velox SIMD acceleration layer.
//
// Provides SSE2/AVX2 (x86), NEON (AArch64), and portable scalar
// implementations of the vector/matrix primitives used by the physics
// hot paths. Every intrinsic path is guarded so the header compiles
// cleanly under CUDA (nvcc) where only the scalar fallback is active.
//
// Compile-time detection decides which ISA the compiler may emit.
// Runtime detection (CPUID / HWCAP) enables dynamic dispatch for
// optional extensions such as AVX2 that are not part of the x86-64
// baseline.

#include <cstdint>
#include <cstring>

// ================================================================
// 1. Compile-time ISA detection
// ================================================================

#define VELOX_SIMD_SSE2  0
#define VELOX_SIMD_NEON  0
#define VELOX_SIMD_AVX2_COMPILE 0

#if !defined(__CUDACC__)
#  if defined(_M_X64) || defined(__x86_64__) || defined(_M_IX86) || defined(__i386__)
#    undef  VELOX_SIMD_SSE2
#    define VELOX_SIMD_SSE2 1
#    if defined(__AVX2__)
#      undef  VELOX_SIMD_AVX2_COMPILE
#      define VELOX_SIMD_AVX2_COMPILE 1
#    endif
#    if defined(_MSC_VER)
#      include <immintrin.h>
#      include <intrin.h>
#    else
#      include <x86intrin.h>
#      include <cpuid.h>
#    endif
#  elif defined(_M_ARM64) || defined(__aarch64__)
#    undef  VELOX_SIMD_NEON
#    define VELOX_SIMD_NEON 1
#    include <arm_neon.h>
#  endif
#endif

// Strict replay is a portability contract, not a throughput mode. The SSE2
// and NEON reductions use different instruction sequences, so even with FMA
// contraction disabled they can round differently at contact boundaries.
// Keep strict builds on the common scalar evaluation order; relaxed builds
// retain all host SIMD acceleration.
#if defined(VELOX_STRICT_FLOATING_POINT) && VELOX_STRICT_FLOATING_POINT
#  define VELOX_SIMD_AVAILABLE 0
#else
#  define VELOX_SIMD_AVAILABLE (VELOX_SIMD_SSE2 || VELOX_SIMD_NEON)
#endif

// GCC/Clang allow per-function ISA selection via target attributes,
// enabling AVX2 code paths even when the translation unit targets SSE2.
#if VELOX_SIMD_SSE2 && !VELOX_SIMD_AVX2_COMPILE && \
    (defined(__GNUC__) || defined(__clang__)) && !defined(__CUDACC__)
#  define VELOX_SIMD_AVX2_TARGET __attribute__((target("avx2,fma")))
#  define VELOX_SIMD_AVX2_DISPATCH 1
#else
#  define VELOX_SIMD_AVX2_TARGET
#  define VELOX_SIMD_AVX2_DISPATCH 0
#endif

namespace velox {
namespace simd {

// ================================================================
// 2. Runtime CPU feature detection
// ================================================================

enum CpuFeatureBits : uint32_t {
    kCpuNone  = 0,
    kCpuSSE2  = 1u << 0,
    kCpuAVX2  = 1u << 1,
    kCpuFMA   = 1u << 2,
    kCpuNEON  = 1u << 3,
};

namespace detail {

#if VELOX_SIMD_SSE2

inline void cpuid(int out[4], int leaf) {
#if defined(_MSC_VER)
    __cpuid(out, leaf);
#else
    unsigned a, b, c, d;
    __get_cpuid((unsigned)leaf, &a, &b, &c, &d);
    out[0] = (int)a; out[1] = (int)b; out[2] = (int)c; out[3] = (int)d;
#endif
}

inline void cpuidex(int out[4], int leaf, int subleaf) {
#if defined(_MSC_VER)
    __cpuidex(out, leaf, subleaf);
#else
    unsigned a, b, c, d;
    __get_cpuid_count((unsigned)leaf, (unsigned)subleaf, &a, &b, &c, &d);
    out[0] = (int)a; out[1] = (int)b; out[2] = (int)c; out[3] = (int)d;
#endif
}

inline uint64_t xgetbv0() {
#if defined(_MSC_VER)
    return _xgetbv(0);
#else
    unsigned lo, hi;
    __asm__ __volatile__("xgetbv" : "=a"(lo), "=d"(hi) : "c"(0));
    return ((uint64_t)hi << 32) | lo;
#endif
}

#endif // VELOX_SIMD_SSE2

inline uint32_t detectFeatures() {
    uint32_t f = kCpuNone;
#if VELOX_SIMD_SSE2
    f |= kCpuSSE2; // baseline on x86-64
    int regs[4] = {};
    cpuid(regs, 1);
    bool osxsave = (regs[2] & (1 << 27)) != 0;
    bool osAvx = false;
    if (osxsave) {
        uint64_t xcr0 = xgetbv0();
        osAvx = (xcr0 & 0x6u) == 0x6u; // XMM + YMM state enabled by OS
    }
    cpuidex(regs, 7, 0);
    if (osAvx && (regs[1] & (1 << 5)))  f |= kCpuAVX2;
    if (osAvx && (regs[2] & (1 << 12))) f |= kCpuFMA;
#elif VELOX_SIMD_NEON
    f |= kCpuNEON; // mandatory on AArch64
#endif
    return f;
}

} // namespace detail

// Cached result of runtime detection; computed once on first call.
inline uint32_t cpuFeatures() {
    static const uint32_t cached = detail::detectFeatures();
    return cached;
}

inline bool hasSSE2()  { return (cpuFeatures() & kCpuSSE2)  != 0; }
inline bool hasAVX2()  { return (cpuFeatures() & kCpuAVX2)  != 0; }
inline bool hasFMA()   { return (cpuFeatures() & kCpuFMA)   != 0; }
inline bool hasNEON()  { return (cpuFeatures() & kCpuNEON)  != 0; }

// Human-readable name of the active SIMD level (for diagnostics / UI).
inline const char* simdLevelName() {
#if VELOX_SIMD_NEON
    return "NEON";
#elif VELOX_SIMD_AVX2_COMPILE
    return "AVX2";
#elif VELOX_SIMD_SSE2
    if (hasAVX2()) return "SSE2+AVX2(dispatch)";
    return "SSE2";
#else
    return "scalar";
#endif
}

// ================================================================
// 3. Vec4 – 4-component float vector, 16-byte aligned
// ================================================================

struct alignas(16) Vec4 {
    float x = 0, y = 0, z = 0, w = 0;
};

// ---- constructors / conversions ----

inline Vec4 v4(float x, float y, float z, float w) {
    Vec4 r; r.x = x; r.y = y; r.z = z; r.w = w; return r;
}
inline Vec4 v4set(float s) { return v4(s, s, s, s); }

// Load a Vec3 (x,y,z) with w = 0.
template <typename Vec3T>
inline Vec4 v4from3(const Vec3T& v) { return v4(v.x, v.y, v.z, 0.0f); }

// Load a Vec3 (x,y,z) with explicit w.
template <typename Vec3T>
inline Vec4 v4from3w(const Vec3T& v, float w) { return v4(v.x, v.y, v.z, w); }

// ---- arithmetic ----

inline Vec4 v4add(const Vec4& a, const Vec4& b) {
#if VELOX_SIMD_SSE2
    Vec4 r; _mm_store_ps(&r.x, _mm_add_ps(_mm_load_ps(&a.x), _mm_load_ps(&b.x))); return r;
#elif VELOX_SIMD_NEON
    Vec4 r; vst1q_f32(&r.x, vaddq_f32(vld1q_f32(&a.x), vld1q_f32(&b.x))); return r;
#else
    return v4(a.x+b.x, a.y+b.y, a.z+b.z, a.w+b.w);
#endif
}

inline Vec4 v4sub(const Vec4& a, const Vec4& b) {
#if VELOX_SIMD_SSE2
    Vec4 r; _mm_store_ps(&r.x, _mm_sub_ps(_mm_load_ps(&a.x), _mm_load_ps(&b.x))); return r;
#elif VELOX_SIMD_NEON
    Vec4 r; vst1q_f32(&r.x, vsubq_f32(vld1q_f32(&a.x), vld1q_f32(&b.x))); return r;
#else
    return v4(a.x-b.x, a.y-b.y, a.z-b.z, a.w-b.w);
#endif
}

inline Vec4 v4mul(const Vec4& a, const Vec4& b) {
#if VELOX_SIMD_SSE2
    Vec4 r; _mm_store_ps(&r.x, _mm_mul_ps(_mm_load_ps(&a.x), _mm_load_ps(&b.x))); return r;
#elif VELOX_SIMD_NEON
    Vec4 r; vst1q_f32(&r.x, vmulq_f32(vld1q_f32(&a.x), vld1q_f32(&b.x))); return r;
#else
    return v4(a.x*b.x, a.y*b.y, a.z*b.z, a.w*b.w);
#endif
}

inline Vec4 v4scale(const Vec4& a, float s) {
    return v4mul(a, v4set(s));
}

// Fused multiply-add: a + b * c  (per-component).
inline Vec4 v4madd(const Vec4& a, const Vec4& b, const Vec4& c) {
#if VELOX_SIMD_SSE2 && VELOX_SIMD_AVX2_COMPILE
    Vec4 r; _mm_store_ps(&r.x, _mm_fmadd_ps(_mm_load_ps(&b.x), _mm_load_ps(&c.x), _mm_load_ps(&a.x))); return r;
#elif VELOX_SIMD_NEON && defined(__aarch64__)
    Vec4 r; vst1q_f32(&r.x, vfmaq_f32(vld1q_f32(&a.x), vld1q_f32(&b.x), vld1q_f32(&c.x))); return r;
#else
    return v4add(a, v4mul(b, c));
#endif
}

// ---- dot products ----

// 3-component dot (ignores w).
inline float v4dot3(const Vec4& a, const Vec4& b) {
#if VELOX_SIMD_SSE2
    __m128 m = _mm_mul_ps(_mm_load_ps(&a.x), _mm_load_ps(&b.x));
    // m = [x, y, z, w].  Sum x+y+z (w is 0*0 = 0 for Vec3-loaded values).
    __m128 t = _mm_add_ps(m, _mm_shuffle_ps(m, m, _MM_SHUFFLE(1, 0, 3, 2)));
    t = _mm_add_ss(t, _mm_shuffle_ps(t, t, _MM_SHUFFLE(0, 0, 0, 1)));
    return _mm_cvtss_f32(t);
#elif VELOX_SIMD_NEON && defined(__aarch64__)
    return vaddvq_f32(vmulq_f32(vld1q_f32(&a.x), vld1q_f32(&b.x)));
#elif VELOX_SIMD_NEON
    float32x4_t m = vmulq_f32(vld1q_f32(&a.x), vld1q_f32(&b.x));
    float32x2_t lo = vget_low_f32(m), hi = vget_high_f32(m);
    float32x2_t s = vpadd_f32(lo, hi);
    s = vpadd_f32(s, s);
    return vget_lane_f32(s, 0);
#else
    return a.x*b.x + a.y*b.y + a.z*b.z;
#endif
}

// 4-component dot.
inline float v4dot4(const Vec4& a, const Vec4& b) {
#if VELOX_SIMD_SSE2
    __m128 m = _mm_mul_ps(_mm_load_ps(&a.x), _mm_load_ps(&b.x));
    __m128 t = _mm_add_ps(m, _mm_shuffle_ps(m, m, _MM_SHUFFLE(1, 0, 3, 2)));
    t = _mm_add_ss(t, _mm_shuffle_ps(t, t, _MM_SHUFFLE(0, 0, 0, 1)));
    return _mm_cvtss_f32(t);
#elif VELOX_SIMD_NEON && defined(__aarch64__)
    return vaddvq_f32(vmulq_f32(vld1q_f32(&a.x), vld1q_f32(&b.x)));
#else
    return a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w;
#endif
}

// ---- cross product (3-component, result w = 0) ----

inline Vec4 v4cross3(const Vec4& a, const Vec4& b) {
#if VELOX_SIMD_SSE2
    __m128 va = _mm_load_ps(&a.x);
    __m128 vb = _mm_load_ps(&b.x);
    // a_yzx = [ay, az, ax, aw],  b_zxy = [bz, bw, bx, by]
    __m128 a_yzx = _mm_shuffle_ps(va, va, _MM_SHUFFLE(3, 0, 2, 1));
    __m128 b_zxy = _mm_shuffle_ps(vb, vb, _MM_SHUFFLE(3, 1, 0, 2));
    __m128 a_zxy = _mm_shuffle_ps(va, va, _MM_SHUFFLE(3, 1, 0, 2));
    __m128 b_yzx = _mm_shuffle_ps(vb, vb, _MM_SHUFFLE(3, 0, 2, 1));
    __m128 c = _mm_sub_ps(_mm_mul_ps(a_yzx, b_zxy), _mm_mul_ps(a_zxy, b_yzx));
    // Zero the w lane.
    c = _mm_and_ps(c, _mm_castsi128_ps(_mm_set_epi32(0, -1, -1, -1)));
    Vec4 r; _mm_store_ps(&r.x, c); return r;
#elif VELOX_SIMD_NEON
    float ax = vgetq_lane_f32(vld1q_f32(&a.x), 0);
    float ay = vgetq_lane_f32(vld1q_f32(&a.x), 1);
    float az = vgetq_lane_f32(vld1q_f32(&a.x), 2);
    float bx = vgetq_lane_f32(vld1q_f32(&b.x), 0);
    float by = vgetq_lane_f32(vld1q_f32(&b.x), 1);
    float bz = vgetq_lane_f32(vld1q_f32(&b.x), 2);
    return v4(ay*bz - az*by, az*bx - ax*bz, ax*by - ay*bx, 0.0f);
#else
    return v4(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x, 0.0f);
#endif
}

// ---- length / normalize (3-component) ----

inline float v4lenSq3(const Vec4& a) { return v4dot3(a, a); }

inline float v4len3(const Vec4& a) {
#if VELOX_SIMD_SSE2
    return _mm_cvtss_f32(_mm_sqrt_ss(_mm_set_ss(v4dot3(a, a))));
#elif VELOX_SIMD_NEON
    float sq = v4dot3(a, a);
    return vget_lane_f32(vsqrt_f32(vdup_n_f32(sq)), 0);
#else
    float sq = v4dot3(a, a);
    return sq > 0.0f ? sqrtf(sq) : 0.0f;
#endif
}

inline Vec4 v4normalize3(const Vec4& a) {
    float sq = v4dot3(a, a);
    if (sq < 1e-16f) return v4(0, 0, 0, 0);
#if VELOX_SIMD_SSE2
    __m128 v = _mm_load_ps(&a.x);
    __m128 len = _mm_sqrt_ps(_mm_set1_ps(sq));
    __m128 inv = _mm_div_ps(_mm_set1_ps(1.0f), len);
    Vec4 r; _mm_store_ps(&r.x, _mm_mul_ps(v, inv)); return r;
#elif VELOX_SIMD_NEON
    float inv = 1.0f / sqrtf(sq);
    return v4scale(a, inv);
#else
    float inv = 1.0f / sqrtf(sq);
    return v4(a.x*inv, a.y*inv, a.z*inv, 0.0f);
#endif
}

// ================================================================
// 4. Quaternion rotation (SIMD-accelerated)
// ================================================================

// rotate(q, v) = v + 2 * cross(q.xyz, cross(q.xyz, v) + q.w * v)
// q is loaded as (x, y, z, w).
template <typename QuatT, typename Vec3T>
inline Vec4 v4rotateQuat(const QuatT& q, const Vec3T& v) {
    Vec4 qv = v4(q.x, q.y, q.z, q.w);
    Vec4 vv = v4from3(v);
    Vec4 t  = v4cross3(qv, vv);
    t = v4add(t, v4scale(vv, q.w));
    t = v4cross3(qv, t);
    return v4add(vv, v4scale(t, 2.0f));
}

template <typename QuatT, typename Vec3T>
inline Vec4 v4rotateInvQuat(const QuatT& q, const Vec3T& v) {
    Vec4 qv = v4(-q.x, -q.y, -q.z, q.w);
    Vec4 vv = v4from3(v);
    Vec4 t  = v4cross3(qv, vv);
    t = v4add(t, v4scale(vv, q.w));
    t = v4cross3(qv, t);
    return v4add(vv, v4scale(t, 2.0f));
}

// ================================================================
// 5. Mat4 – 4×4 column-major matrix (rotation-focused)
// ================================================================

struct alignas(16) Mat4 {
    Vec4 c0, c1, c2, c3; // columns
};

inline Mat4 m4identity() {
    Mat4 m;
    m.c0 = v4(1, 0, 0, 0);
    m.c1 = v4(0, 1, 0, 0);
    m.c2 = v4(0, 0, 1, 0);
    m.c3 = v4(0, 0, 0, 1);
    return m;
}

// Build a rotation matrix from a unit quaternion (x, y, z, w).
template <typename QuatT>
inline Mat4 m4fromQuat(const QuatT& q) {
    float x = q.x, y = q.y, z = q.z, w = q.w;
    float x2 = x+x, y2 = y+y, z2 = z+z;
    float xx = x*x2, xy = x*y2, xz = x*z2;
    float yy = y*y2, yz = y*z2, zz = z*z2;
    float wx = w*x2, wy = w*y2, wz = w*z2;
    Mat4 m;
    m.c0 = v4(1.0f-(yy+zz), xy+wz,        xz-wy,        0.0f);
    m.c1 = v4(xy-wz,        1.0f-(xx+zz),  yz+wx,        0.0f);
    m.c2 = v4(xz+wy,        yz-wx,         1.0f-(xx+yy), 0.0f);
    m.c3 = v4(0, 0, 0, 1);
    return m;
}

inline Mat4 m4transpose(const Mat4& m) {
#if VELOX_SIMD_SSE2
    __m128 r0 = _mm_load_ps(&m.c0.x);
    __m128 r1 = _mm_load_ps(&m.c1.x);
    __m128 r2 = _mm_load_ps(&m.c2.x);
    __m128 r3 = _mm_load_ps(&m.c3.x);
    _MM_TRANSPOSE4_PS(r0, r1, r2, r3);
    Mat4 t;
    _mm_store_ps(&t.c0.x, r0);
    _mm_store_ps(&t.c1.x, r1);
    _mm_store_ps(&t.c2.x, r2);
    _mm_store_ps(&t.c3.x, r3);
    return t;
#else
    Mat4 t;
    t.c0 = v4(m.c0.x, m.c1.x, m.c2.x, m.c3.x);
    t.c1 = v4(m.c0.y, m.c1.y, m.c2.y, m.c3.y);
    t.c2 = v4(m.c0.z, m.c1.z, m.c2.z, m.c3.z);
    t.c3 = v4(m.c0.w, m.c1.w, m.c2.w, m.c3.w);
    return t;
#endif
}

// Multiply a 4×4 matrix by a Vec4 (column vector).
inline Vec4 m4mulV4(const Mat4& m, const Vec4& v) {
#if VELOX_SIMD_SSE2
    __m128 vx = _mm_shuffle_ps(_mm_load_ps(&v.x), _mm_load_ps(&v.x), _MM_SHUFFLE(0,0,0,0));
    __m128 vy = _mm_shuffle_ps(_mm_load_ps(&v.x), _mm_load_ps(&v.x), _MM_SHUFFLE(1,1,1,1));
    __m128 vz = _mm_shuffle_ps(_mm_load_ps(&v.x), _mm_load_ps(&v.x), _MM_SHUFFLE(2,2,2,2));
    __m128 vw = _mm_shuffle_ps(_mm_load_ps(&v.x), _mm_load_ps(&v.x), _MM_SHUFFLE(3,3,3,3));
    __m128 r = _mm_add_ps(
        _mm_add_ps(_mm_mul_ps(_mm_load_ps(&m.c0.x), vx), _mm_mul_ps(_mm_load_ps(&m.c1.x), vy)),
        _mm_add_ps(_mm_mul_ps(_mm_load_ps(&m.c2.x), vz), _mm_mul_ps(_mm_load_ps(&m.c3.x), vw)));
    Vec4 out; _mm_store_ps(&out.x, r); return out;
#elif VELOX_SIMD_NEON
    float32x4_t vv = vld1q_f32(&v.x);
    float32x4_t r = vmulq_laneq_f32(vld1q_f32(&m.c0.x), vv, 0);
    r = vfmaq_laneq_f32(r, vld1q_f32(&m.c1.x), vv, 1);
    r = vfmaq_laneq_f32(r, vld1q_f32(&m.c2.x), vv, 2);
    r = vfmaq_laneq_f32(r, vld1q_f32(&m.c3.x), vv, 3);
    Vec4 out; vst1q_f32(&out.x, r); return out;
#else
    return v4(
        m.c0.x*v.x + m.c1.x*v.y + m.c2.x*v.z + m.c3.x*v.w,
        m.c0.y*v.x + m.c1.y*v.y + m.c2.y*v.z + m.c3.y*v.w,
        m.c0.z*v.x + m.c1.z*v.y + m.c2.z*v.z + m.c3.z*v.w,
        m.c0.w*v.x + m.c1.w*v.y + m.c2.w*v.z + m.c3.w*v.w);
#endif
}

// Transform a direction (w = 0) by the upper-left 3×3.
template <typename Vec3T>
inline Vec4 m4mulDir(const Mat4& m, const Vec3T& v) {
    return m4mulV4(m, v4from3(v));
}

// Matrix × matrix.
inline Mat4 m4mul(const Mat4& a, const Mat4& b) {
    Mat4 r;
    r.c0 = m4mulV4(a, b.c0);
    r.c1 = m4mulV4(a, b.c1);
    r.c2 = m4mulV4(a, b.c2);
    r.c3 = m4mulV4(a, b.c3);
    return r;
}

// ================================================================
// 6. InertiaTensor – cached world-space inverse-inertia multiply
// ================================================================
// Replaces two quaternion rotations per invInertiaMul call with two
// matrix-vector multiplies.  The matrix is built once per body per
// solver iteration and reused for every torque-arm cross product.

template <typename BodyT>
struct InertiaTensor {
    Mat4 R;      // world-from-body rotation
    Mat4 Rt;     // body-from-world (transpose)
    Vec4 invI;   // diagonal inverse principal moments
    bool active; // false for static / locked bodies

    void init(const BodyT& b) {
        active = b.isDynamic() && !b.isLocked();
        if (!active) { invI = v4set(0); R = Rt = m4identity(); return; }
        // Reproduce Body::inertiaFrameAt logic.
        auto frame = b.orientation;
        if (!(b.inertiaOrientation.x == 0.0f && b.inertiaOrientation.y == 0.0f &&
              b.inertiaOrientation.z == 0.0f && b.inertiaOrientation.w == 1.0f)) {
            // frame = mul(orientation, inertiaOrientation)
            auto& a = b.orientation;
            auto& c = b.inertiaOrientation;
            frame.x = a.w*c.x + a.x*c.w + a.y*c.z - a.z*c.y;
            frame.y = a.w*c.y - a.x*c.z + a.y*c.w + a.z*c.x;
            frame.z = a.w*c.z + a.x*c.y - a.y*c.x + a.z*c.w;
            frame.w = a.w*c.w - a.x*c.x - a.y*c.y - a.z*c.z;
        }
        R  = m4fromQuat(frame);
        Rt = m4transpose(R);
        invI = v4from3(b.invInertia);
    }

    // I⁻¹_world * v  =  R · diag(invI) · Rᵀ · v
    template <typename Vec3T>
    Vec4 mul(const Vec3T& v) const {
        if (!active) return v4(0, 0, 0, 0);
        Vec4 local = m4mulDir(Rt, v);
        local = v4mul(local, invI);
        return m4mulDir(R, local);
    }
};

// ================================================================
// 7. Batch hull-support (the single biggest GJK win)
// ================================================================
// Returns the index of the point with the maximum dot product with
// `dir`.  Points are Vec3 (12-byte stride).  The SSE2 path processes
// 4 candidates per iteration; the AVX2 path processes 8.

namespace hull_detail {

inline uint32_t scalar(const float* dx, const float* dy, const float* dz,
                       const void* pts, uint32_t count) {
    struct P { float x, y, z; };
    const P* p = static_cast<const P*>(pts);
    uint32_t best = 0;
    float bestDot = p[0].x * *dx + p[0].y * *dy + p[0].z * *dz;
    for (uint32_t i = 1; i < count; ++i) {
        float t = p[i].x * *dx + p[i].y * *dy + p[i].z * *dz;
        if (t > bestDot) { bestDot = t; best = i; }
    }
    return best;
}

#if VELOX_SIMD_SSE2

inline uint32_t sse2(float dx, float dy, float dz,
                     const void* pts, uint32_t count) {
    struct P { float x, y, z; };
    const P* p = static_cast<const P*>(pts);
    __m128 vdx = _mm_set1_ps(dx);
    __m128 vdy = _mm_set1_ps(dy);
    __m128 vdz = _mm_set1_ps(dz);

    uint32_t best = 0;
    float bestDot = p[0].x*dx + p[0].y*dy + p[0].z*dz;

    uint32_t i = 1;
    for (; i + 3 < count; i += 4) {
        __m128 px = _mm_set_ps(p[i+3].x, p[i+2].x, p[i+1].x, p[i+0].x);
        __m128 py = _mm_set_ps(p[i+3].y, p[i+2].y, p[i+1].y, p[i+0].y);
        __m128 pz = _mm_set_ps(p[i+3].z, p[i+2].z, p[i+1].z, p[i+0].z);
        __m128 dots = _mm_add_ps(_mm_add_ps(_mm_mul_ps(px, vdx),
                                            _mm_mul_ps(py, vdy)),
                                 _mm_mul_ps(pz, vdz));
        int mask = _mm_movemask_ps(_mm_cmpgt_ps(dots, _mm_set1_ps(bestDot)));
        if (mask) {
            alignas(16) float vals[4];
            _mm_store_ps(vals, dots);
            for (int k = 0; k < 4; ++k)
                if (vals[k] > bestDot) { bestDot = vals[k]; best = i + (uint32_t)k; }
        }
    }
    for (; i < count; ++i) {
        float t = p[i].x*dx + p[i].y*dy + p[i].z*dz;
        if (t > bestDot) { bestDot = t; best = i; }
    }
    return best;
}

#if VELOX_SIMD_AVX2_COMPILE || VELOX_SIMD_AVX2_DISPATCH

VELOX_SIMD_AVX2_TARGET
inline uint32_t avx2(float dx, float dy, float dz,
                     const void* pts, uint32_t count) {
    struct P { float x, y, z; };
    const P* p = static_cast<const P*>(pts);
    __m256 vdx = _mm256_set1_ps(dx);
    __m256 vdy = _mm256_set1_ps(dy);
    __m256 vdz = _mm256_set1_ps(dz);

    uint32_t best = 0;
    float bestDot = p[0].x*dx + p[0].y*dy + p[0].z*dz;

    uint32_t i = 1;
    for (; i + 7 < count; i += 8) {
        __m256 px = _mm256_set_ps(p[i+7].x, p[i+6].x, p[i+5].x, p[i+4].x,
                                   p[i+3].x, p[i+2].x, p[i+1].x, p[i+0].x);
        __m256 py = _mm256_set_ps(p[i+7].y, p[i+6].y, p[i+5].y, p[i+4].y,
                                   p[i+3].y, p[i+2].y, p[i+1].y, p[i+0].y);
        __m256 pz = _mm256_set_ps(p[i+7].z, p[i+6].z, p[i+5].z, p[i+4].z,
                                   p[i+3].z, p[i+2].z, p[i+1].z, p[i+0].z);
        __m256 dots = _mm256_add_ps(_mm256_add_ps(_mm256_mul_ps(px, vdx),
                                                   _mm256_mul_ps(py, vdy)),
                                    _mm256_mul_ps(pz, vdz));
        int mask = _mm256_movemask_ps(_mm256_cmp_ps(dots, _mm256_set1_ps(bestDot), _CMP_GT_OQ));
        if (mask) {
            alignas(32) float vals[8];
            _mm256_store_ps(vals, dots);
            for (int k = 0; k < 8; ++k)
                if (vals[k] > bestDot) { bestDot = vals[k]; best = i + (uint32_t)k; }
        }
    }
    // Scalar tail (also handles the SSE2 remainder).
    for (; i < count; ++i) {
        float t = p[i].x*dx + p[i].y*dy + p[i].z*dz;
        if (t > bestDot) { bestDot = t; best = i; }
    }
    return best;
}

#endif // AVX2
#endif // SSE2

#if VELOX_SIMD_NEON

inline uint32_t neon(float dx, float dy, float dz,
                     const void* pts, uint32_t count) {
    struct P { float x, y, z; };
    const P* p = static_cast<const P*>(pts);
    float32x4_t vdx = vdupq_n_f32(dx);
    float32x4_t vdy = vdupq_n_f32(dy);
    float32x4_t vdz = vdupq_n_f32(dz);

    uint32_t best = 0;
    float bestDot = p[0].x*dx + p[0].y*dy + p[0].z*dz;

    uint32_t i = 1;
    for (; i + 3 < count; i += 4) {
        float32x4_t px = {p[i].x, p[i+1].x, p[i+2].x, p[i+3].x};
        float32x4_t py = {p[i].y, p[i+1].y, p[i+2].y, p[i+3].y};
        float32x4_t pz = {p[i].z, p[i+1].z, p[i+2].z, p[i+3].z};
        float32x4_t dots = vaddq_f32(vaddq_f32(vmulq_f32(px, vdx),
                                               vmulq_f32(py, vdy)),
                                     vmulq_f32(pz, vdz));
        // Check if any lane beats bestDot.
        uint32x4_t gt = vcgtq_f32(dots, vdupq_n_f32(bestDot));
        if (vmaxvq_u32(gt)) {
            alignas(16) float vals[4];
            vst1q_f32(vals, dots);
            for (int k = 0; k < 4; ++k)
                if (vals[k] > bestDot) { bestDot = vals[k]; best = i + (uint32_t)k; }
        }
    }
    for (; i < count; ++i) {
        float t = p[i].x*dx + p[i].y*dy + p[i].z*dz;
        if (t > bestDot) { bestDot = t; best = i; }
    }
    return best;
}

#endif // NEON

} // namespace hull_detail

// Public entry: pick the best available implementation at runtime.
template <typename Vec3T>
inline uint32_t hullSupportIndex(const Vec3T& dir, const Vec3T* points,
                                 uint32_t count) {
    if (count == 0) return 0;
    if (count == 1) return 0;
#if VELOX_SIMD_SSE2
#  if VELOX_SIMD_AVX2_COMPILE
    if (hasAVX2())
        return hull_detail::avx2(dir.x, dir.y, dir.z, points, count);
    return hull_detail::sse2(dir.x, dir.y, dir.z, points, count);
#  elif VELOX_SIMD_AVX2_DISPATCH
    if (hasAVX2())
        return hull_detail::avx2(dir.x, dir.y, dir.z, points, count);
    return hull_detail::sse2(dir.x, dir.y, dir.z, points, count);
#  else
    return hull_detail::sse2(dir.x, dir.y, dir.z, points, count);
#  endif
#elif VELOX_SIMD_NEON
    return hull_detail::neon(dir.x, dir.y, dir.z, points, count);
#else
    return hull_detail::scalar(&dir.x, &dir.y, &dir.z, points, count);
#endif
}

// ================================================================
// 8. Batch dot-product (SoA helper for solver / narrow-phase)
// ================================================================
// Computes out[i] = dot3(a[i], b[i]) for i in [0, count).

template <typename Vec3T>
inline void batchDot3(const Vec3T* a, const Vec3T* b, float* out,
                      uint32_t count) {
#if VELOX_SIMD_SSE2
    uint32_t i = 0;
    for (; i + 3 < count; i += 4) {
        __m128 ax = _mm_set_ps(a[i+3].x, a[i+2].x, a[i+1].x, a[i+0].x);
        __m128 ay = _mm_set_ps(a[i+3].y, a[i+2].y, a[i+1].y, a[i+0].y);
        __m128 az = _mm_set_ps(a[i+3].z, a[i+2].z, a[i+1].z, a[i+0].z);
        __m128 bx = _mm_set_ps(b[i+3].x, b[i+2].x, b[i+1].x, b[i+0].x);
        __m128 by = _mm_set_ps(b[i+3].y, b[i+2].y, b[i+1].y, b[i+0].y);
        __m128 bz = _mm_set_ps(b[i+3].z, b[i+2].z, b[i+1].z, b[i+0].z);
        __m128 d = _mm_add_ps(_mm_add_ps(_mm_mul_ps(ax, bx),
                                         _mm_mul_ps(ay, by)),
                              _mm_mul_ps(az, bz));
        _mm_storeu_ps(out + i, d);
    }
    for (; i < count; ++i)
        out[i] = a[i].x*b[i].x + a[i].y*b[i].y + a[i].z*b[i].z;
#elif VELOX_SIMD_NEON
    uint32_t i = 0;
    for (; i + 3 < count; i += 4) {
        float32x4_t ax = {a[i].x, a[i+1].x, a[i+2].x, a[i+3].x};
        float32x4_t ay = {a[i].y, a[i+1].y, a[i+2].y, a[i+3].y};
        float32x4_t az = {a[i].z, a[i+1].z, a[i+2].z, a[i+3].z};
        float32x4_t bx = {b[i].x, b[i+1].x, b[i+2].x, b[i+3].x};
        float32x4_t by = {b[i].y, b[i+1].y, b[i+2].y, b[i+3].y};
        float32x4_t bz = {b[i].z, b[i+1].z, b[i+2].z, b[i+3].z};
        float32x4_t d = vaddq_f32(vaddq_f32(vmulq_f32(ax, bx),
                                            vmulq_f32(ay, by)),
                                  vmulq_f32(az, bz));
        vst1q_f32(out + i, d);
    }
    for (; i < count; ++i)
        out[i] = a[i].x*b[i].x + a[i].y*b[i].y + a[i].z*b[i].z;
#else
    for (uint32_t i = 0; i < count; ++i)
        out[i] = a[i].x*b[i].x + a[i].y*b[i].y + a[i].z*b[i].z;
#endif
}

// ================================================================
// 9. SIMD-accelerated quaternion rotate returning Vec3-compatible
// ================================================================
// Convenience wrappers that accept and return the engine's Vec3 type
// directly, for drop-in use in hot paths.

template <typename QuatT, typename Vec3T>
inline Vec3T simdRotate(const QuatT& q, const Vec3T& v) {
    Vec4 r = v4rotateQuat(q, v);
    return Vec3T{r.x, r.y, r.z};
}

template <typename QuatT, typename Vec3T>
inline Vec3T simdRotateInv(const QuatT& q, const Vec3T& v) {
    Vec4 r = v4rotateInvQuat(q, v);
    return Vec3T{r.x, r.y, r.z};
}

// SIMD dot / cross / normalize that accept the engine's Vec3.
template <typename Vec3T>
inline float simdDot(const Vec3T& a, const Vec3T& b) {
    return v4dot3(v4from3(a), v4from3(b));
}

template <typename Vec3T>
inline Vec3T simdCross(const Vec3T& a, const Vec3T& b) {
    Vec4 r = v4cross3(v4from3(a), v4from3(b));
    return Vec3T{r.x, r.y, r.z};
}

template <typename Vec3T>
inline Vec3T simdNormalize(const Vec3T& v) {
    Vec4 r = v4normalize3(v4from3(v));
    return Vec3T{r.x, r.y, r.z};
}

} // namespace simd
} // namespace velox
