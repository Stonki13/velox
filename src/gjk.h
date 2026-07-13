#pragma once
#include "velox/body.h"

namespace velox {

// A convex shape for GJK: a core support function plus an inflation radius.
// Spheres and capsules are points/segments with a radius, which keeps their
// GJK cores from ever overlapping in practice; boxes and triangles have
// radius 0.
struct Convex {
    enum Kind : uint8_t { Point, Segment, Box, Triangle } kind;
    Vec3 position;      // world transform
    Quat orientation;
    Vec3 halfExtents;   // Box
    Vec3 segA, segB;    // Segment (world), Triangle uses segA/segB/segC
    Vec3 triC;
    float radius;       // inflation

    Vec3 support(const Vec3& dir) const;
};

Convex makeConvex(const Body& b);                       // Sphere/Box/Capsule
Convex makeTriangle(const Vec3& a, const Vec3& b, const Vec3& c);

struct GjkResult {
    float distance;   // between inflated surfaces; <= 0 if cores overlap
    Vec3 normal;      // from B towards A
    Vec3 pointA;      // witness point on A's surface
    Vec3 pointB;      // witness point on B's surface
    bool overlapping; // cores intersect: distance/witnesses are a fallback
};

GjkResult gjkDistance(const Convex& a, const Convex& b);

} // namespace velox
