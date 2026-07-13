#pragma once
#include <cmath>

namespace velox {

struct Vec3 {
    float x = 0, y = 0, z = 0;

    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    Vec3& operator*=(float s) { x *= s; y *= s; z *= s; return *this; }
};

inline float dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline float lengthSq(const Vec3& v) { return dot(v, v); }
inline float length(const Vec3& v) { return std::sqrt(lengthSq(v)); }
inline Vec3 normalize(const Vec3& v) {
    float len = length(v);
    return len > 1e-8f ? v * (1.0f / len) : Vec3{};
}

struct Quat {
    float x = 0, y = 0, z = 0, w = 1;
};

inline Quat mul(const Quat& a, const Quat& b) {
    return {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

inline Quat normalize(const Quat& q) {
    float len = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (len < 1e-8f) return {};
    float inv = 1.0f / len;
    return {q.x * inv, q.y * inv, q.z * inv, q.w * inv};
}

inline Vec3 rotate(const Quat& q, const Vec3& v) {
    // v' = v + 2 qv x (qv x v + w v)
    Vec3 qv{q.x, q.y, q.z};
    Vec3 t = cross(qv, cross(qv, v) + v * q.w);
    return v + t * 2.0f;
}

inline Vec3 rotateInv(const Quat& q, const Vec3& v) {
    return rotate({-q.x, -q.y, -q.z, q.w}, v);
}

// Integrates orientation by an angular velocity over dt.
inline Quat integrate(const Quat& q, const Vec3& omega, float dt) {
    Quat dq = mul({omega.x * 0.5f * dt, omega.y * 0.5f * dt, omega.z * 0.5f * dt, 0.0f}, q);
    return normalize({q.x + dq.x, q.y + dq.y, q.z + dq.z, q.w + dq.w});
}

inline Quat fromAxisAngle(const Vec3& axis, float radians) {
    Vec3 n = normalize(axis);
    float s = std::sin(radians * 0.5f);
    return {n.x * s, n.y * s, n.z * s, std::cos(radians * 0.5f)};
}

} // namespace velox
