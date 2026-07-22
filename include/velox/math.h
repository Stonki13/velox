#pragma once
#include <cmath>

// Every math/geometry function is compiled for both host and device so the
// CUDA backend runs the exact same narrow phase as the CPU backend.
#if defined(__CUDACC__)
#define VELOX_HD __host__ __device__
#else
#define VELOX_HD
#endif

namespace velox {

struct Vec2 {
    float x = 0.0f;
    float y = 0.0f;

    VELOX_HD Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
    VELOX_HD Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
    VELOX_HD Vec2 operator*(float s) const { return {x * s, y * s}; }
};

struct Vec3 {
    float x = 0, y = 0, z = 0;

    VELOX_HD Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    VELOX_HD Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    VELOX_HD Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    VELOX_HD Vec3 operator-() const { return {-x, -y, -z}; }
    VELOX_HD Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    VELOX_HD Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    VELOX_HD Vec3& operator*=(float s) { x *= s; y *= s; z *= s; return *this; }
};

VELOX_HD inline float dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
VELOX_HD inline float orientCrossZero(float value, int order) {
    return value != 0.0f ? value : (order > 0 ? 0.0f : -0.0f);
}
VELOX_HD inline Vec3 cross(const Vec3& a, const Vec3& b) {
    const int order = a.x != b.x ? (a.x > b.x ? 1 : -1) :
                      a.y != b.y ? (a.y > b.y ? 1 : -1) :
                      a.z != b.z ? (a.z > b.z ? 1 : -1) : 1;
    return {orientCrossZero(a.y * b.z - a.z * b.y, order),
            orientCrossZero(a.z * b.x - a.x * b.z, order),
            orientCrossZero(a.x * b.y - a.y * b.x, order)};
}
VELOX_HD inline float lengthSq(const Vec3& v) { return dot(v, v); }
VELOX_HD inline float length(const Vec3& v) { return sqrtf(lengthSq(v)); }
VELOX_HD inline Vec3 normalize(const Vec3& v) {
    float len = length(v);
    return len > 1e-8f ? v * (1.0f / len) : Vec3{};
}
VELOX_HD inline float vmin(float a, float b) { return a < b ? a : b; }
VELOX_HD inline float vmax(float a, float b) { return a > b ? a : b; }
VELOX_HD inline float vclamp(float v, float lo, float hi) { return vmin(vmax(v, lo), hi); }
VELOX_HD inline Vec3 vmin(const Vec3& a, const Vec3& b) { return {vmin(a.x, b.x), vmin(a.y, b.y), vmin(a.z, b.z)}; }
VELOX_HD inline Vec3 vmax(const Vec3& a, const Vec3& b) { return {vmax(a.x, b.x), vmax(a.y, b.y), vmax(a.z, b.z)}; }

struct Quat {
    float x = 0, y = 0, z = 0, w = 1;
};

VELOX_HD inline float canonicalQuatComponent(float value) {
    // Unit quaternion composition is rounded to a fixed binary grid so CPU
    // and CUDA retain a stable, canonical orientation representation.
    constexpr float kGrid = 4194304.0f; // 2^22
    const float scaled = value * kGrid;
    const float rounded = scaled >= 0.0f ? floorf(scaled + 0.5f)
                                         : ceilf(scaled - 0.5f);
    return rounded / kGrid;
}

VELOX_HD inline Quat mul(const Quat& a, const Quat& b) {
    Quat result{a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
                a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
    const float aLenSq = a.x * a.x + a.y * a.y + a.z * a.z + a.w * a.w;
    const float bLenSq = b.x * b.x + b.y * b.y + b.z * b.z + b.w * b.w;
    if (aLenSq < 0.999f || aLenSq > 1.001f ||
        bLenSq < 0.999f || bLenSq > 1.001f)
        return result;
    return {canonicalQuatComponent(result.x), canonicalQuatComponent(result.y),
            canonicalQuatComponent(result.z), canonicalQuatComponent(result.w)};
}

VELOX_HD inline Quat normalize(const Quat& q) {
    float len = sqrtf(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (len < 1e-8f) return {};
    float inv = 1.0f / len;
    return {q.x * inv, q.y * inv, q.z * inv, q.w * inv};
}

VELOX_HD inline Vec3 rotate(const Quat& q, const Vec3& v) {
    // v' = v + 2 qv x (qv x v + w v)
    Vec3 qv{q.x, q.y, q.z};
    Vec3 t = cross(qv, cross(qv, v) + v * q.w);
    return v + t * 2.0f;
}

VELOX_HD inline Vec3 rotateInv(const Quat& q, const Vec3& v) {
    return rotate({-q.x, -q.y, -q.z, q.w}, v);
}

// Integrates orientation by an angular velocity over dt.
VELOX_HD inline Quat integrate(const Quat& q, const Vec3& omega, float dt) {
    Quat dq = mul({omega.x * 0.5f * dt, omega.y * 0.5f * dt, omega.z * 0.5f * dt, 0.0f}, q);
    return normalize({q.x + dq.x, q.y + dq.y, q.z + dq.z, q.w + dq.w});
}

VELOX_HD inline Quat fromAxisAngle(const Vec3& axis, float radians) {
    Vec3 n = normalize(axis);
    float s = sinf(radians * 0.5f);
    return {n.x * s, n.y * s, n.z * s, cosf(radians * 0.5f)};
}

} // namespace velox
