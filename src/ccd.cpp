#include "ccd.h"

namespace velox {

float sweepSpherePlane(const Body& s, const Body& p, float dt) {
    // Signed distance of the sphere surface from the plane along its motion.
    float dist = dot(p.planeNormal, s.position) - p.planeOffset - s.radius;
    float speed = dot(p.planeNormal, s.velocity) * dt; // motion over the step
    if (dist <= 0.0f) return 0.0f;      // already touching/penetrating
    if (speed >= -1e-9f) return -1.0f;  // moving away or parallel
    float toi = dist / -speed;
    return toi <= 1.0f ? toi : -1.0f;
}

float sweepSphereSphere(const Body& a, const Body& b, float dt) {
    // Relative motion: solve |(p + v t)| = r as a quadratic in t, t in [0, dt].
    Vec3 p = a.position - b.position;
    Vec3 v = (a.velocity - b.velocity) * dt;
    float r = a.radius + b.radius;

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
