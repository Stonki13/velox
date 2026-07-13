#include "ccd.h"
#include <cmath>

namespace velox {

float sweepSpherePlane(const Vec3& p0, const Vec3& disp, float radius,
                       const Vec3& planeNormal, float planeOffset) {
    float dist = dot(planeNormal, p0) - planeOffset - radius;
    float motion = dot(planeNormal, disp);
    if (dist <= 0.0f) return 0.0f;        // already touching/penetrating
    if (motion >= -1e-9f) return -1.0f;   // moving away or parallel
    float toi = dist / -motion;
    return toi <= 1.0f ? toi : -1.0f;
}

float sweepSphereSphere(const Vec3& pa, const Vec3& dispA, float ra,
                        const Vec3& pb, const Vec3& dispB, float rb) {
    // Relative motion: solve |p + v t| = r as a quadratic in t, t in [0, 1].
    Vec3 p = pa - pb;
    Vec3 v = dispA - dispB;
    float r = ra + rb;

    float c = lengthSq(p) - r * r;
    if (c <= 0.0f) return 0.0f;          // already overlapping
    float qa = lengthSq(v);
    if (qa < 1e-12f) return -1.0f;       // no relative motion
    float qb = dot(v, p);
    if (qb >= 0.0f) return -1.0f;        // separating
    float disc = qb * qb - qa * c;
    if (disc < 0.0f) return -1.0f;       // misses
    float toi = (-qb - std::sqrt(disc)) / qa;
    return toi <= 1.0f ? toi : -1.0f;
}

} // namespace velox
