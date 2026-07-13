#include "gjk.h"
#include <cmath>

namespace velox {

Vec3 Convex::support(const Vec3& dir) const {
    switch (kind) {
    case Point:
        return position;
    case Segment:
        return dot(dir, segA) > dot(dir, segB) ? segA : segB;
    case Triangle: {
        Vec3 best = segA;
        if (dot(dir, segB) > dot(dir, best)) best = segB;
        if (dot(dir, triC) > dot(dir, best)) best = triC;
        return best;
    }
    case Box: {
        Vec3 d = rotateInv(orientation, dir);
        Vec3 local{d.x >= 0 ? halfExtents.x : -halfExtents.x,
                   d.y >= 0 ? halfExtents.y : -halfExtents.y,
                   d.z >= 0 ? halfExtents.z : -halfExtents.z};
        return position + rotate(orientation, local);
    }
    }
    return position;
}

Convex makeConvex(const Body& b) {
    Convex c{};
    c.position = b.position;
    c.orientation = b.orientation;
    switch (b.shape) {
    case ShapeType::Sphere:
        c.kind = Convex::Point;
        c.radius = b.radius;
        break;
    case ShapeType::Capsule: {
        c.kind = Convex::Segment;
        Vec3 axis = rotate(b.orientation, {0, b.capsuleHalfHeight, 0});
        c.segA = b.position + axis;
        c.segB = b.position - axis;
        c.radius = b.radius;
        break;
    }
    case ShapeType::Box:
        c.kind = Convex::Box;
        c.halfExtents = b.halfExtents;
        c.radius = 0.0f;
        break;
    default: // Plane/Mesh are not convex volumes; callers handle them separately
        c.kind = Convex::Point;
        c.radius = 0.0f;
        break;
    }
    return c;
}

Convex makeTriangle(const Vec3& a, const Vec3& b, const Vec3& c) {
    Convex t{};
    t.kind = Convex::Triangle;
    t.segA = a;
    t.segB = b;
    t.triC = c;
    t.radius = 0.0f;
    return t;
}

namespace {

struct SupportVert {
    Vec3 p;  // a - b, the Minkowski-difference point
    Vec3 a;  // witness on A's core
    Vec3 b;  // witness on B's core
};

struct Simplex {
    SupportVert v[4];
    float bary[4];
    int count = 0;
};

// Closest point to the origin on the current simplex; reduces the simplex to
// the minimal feature containing that point and records barycentrics.
void solveSegment(Simplex& s) {
    Vec3 a = s.v[0].p, b = s.v[1].p;
    Vec3 ab = b - a;
    float t = -dot(a, ab);
    if (t <= 0.0f) { s.count = 1; s.bary[0] = 1.0f; return; }
    float denom = dot(ab, ab);
    if (t >= denom) { s.v[0] = s.v[1]; s.count = 1; s.bary[0] = 1.0f; return; }
    t /= denom;
    s.count = 2; s.bary[0] = 1.0f - t; s.bary[1] = t;
}

void solveTriangle(Simplex& s) {
    Vec3 a = s.v[0].p, b = s.v[1].p, c = s.v[2].p;
    Vec3 ab = b - a, ac = c - a, ap = -a;
    float d1 = dot(ab, ap), d2 = dot(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f) { s.count = 1; s.bary[0] = 1.0f; return; }

    Vec3 bp = -b;
    float d3 = dot(ab, bp), d4 = dot(ac, bp);
    if (d3 >= 0.0f && d4 <= d3) { s.v[0] = s.v[1]; s.count = 1; s.bary[0] = 1.0f; return; }

    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
        float t = d1 / (d1 - d3);
        s.count = 2; s.bary[0] = 1.0f - t; s.bary[1] = t; return;
    }

    Vec3 cp = -c;
    float d5 = dot(ab, cp), d6 = dot(ac, cp);
    if (d6 >= 0.0f && d5 <= d6) { s.v[0] = s.v[2]; s.count = 1; s.bary[0] = 1.0f; return; }

    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
        float t = d2 / (d2 - d6);
        s.v[1] = s.v[2];
        s.count = 2; s.bary[0] = 1.0f - t; s.bary[1] = t; return;
    }

    float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
        float t = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        s.v[0] = s.v[1]; s.v[1] = s.v[2];
        s.count = 2; s.bary[0] = 1.0f - t; s.bary[1] = t; return;
    }

    float denom = 1.0f / (va + vb + vc);
    s.count = 3;
    s.bary[0] = va * denom;
    s.bary[1] = vb * denom;
    s.bary[2] = vc * denom;
}

// Returns true if the origin is inside the tetrahedron.
bool solveTetrahedron(Simplex& s) {
    // Test the origin against each face; recurse into the closest.
    Simplex best{};
    float bestDist = 1e30f;
    bool inside = true;
    const int faces[4][3] = {{0, 1, 2}, {0, 1, 3}, {0, 2, 3}, {1, 2, 3}};
    for (const auto& f : faces) {
        Vec3 a = s.v[f[0]].p, b = s.v[f[1]].p, c = s.v[f[2]].p;
        Vec3 n = cross(b - a, c - a);
        Vec3 opposite = s.v[6 - f[0] - f[1] - f[2]].p;
        float signOrigin = dot(n, -a);
        float signOpp = dot(n, opposite - a);
        if (signOrigin * signOpp < 0.0f) {
            inside = false;
            Simplex tri{};
            tri.count = 3;
            tri.v[0] = s.v[f[0]]; tri.v[1] = s.v[f[1]]; tri.v[2] = s.v[f[2]];
            solveTriangle(tri);
            Vec3 closest{};
            for (int i = 0; i < tri.count; ++i) closest += tri.v[i].p * tri.bary[i];
            float d = lengthSq(closest);
            if (d < bestDist) { bestDist = d; best = tri; }
        }
    }
    if (!inside) s = best;
    return inside;
}

} // namespace

GjkResult gjkDistance(const Convex& A, const Convex& B) {
    auto supportMD = [&](const Vec3& dir) {
        SupportVert v;
        v.a = A.support(dir);
        v.b = B.support(-dir);
        v.p = v.a - v.b;
        return v;
    };

    Simplex s{};
    Vec3 dir = A.position - B.position;
    if (A.kind == Convex::Triangle) dir = A.segA - B.position;
    if (lengthSq(dir) < 1e-12f) dir = {1, 0, 0};
    s.v[0] = supportMD(dir);
    s.count = 1;
    s.bary[0] = 1.0f;

    GjkResult r{};
    for (int iter = 0; iter < 32; ++iter) {
        Vec3 closest{};
        for (int i = 0; i < s.count; ++i) closest += s.v[i].p * s.bary[i];
        float distSq = lengthSq(closest);
        if (distSq < 1e-10f) { r.overlapping = true; break; }

        Vec3 d = -closest;
        SupportVert w = supportMD(d);
        // No more progress towards the origin => converged.
        float progress = dot(w.p, d) - dot(closest, d);
        if (progress < 1e-6f * std::sqrt(distSq) + 1e-9f) break;

        s.v[s.count++] = w;
        if (s.count == 2) solveSegment(s);
        else if (s.count == 3) solveTriangle(s);
        else if (solveTetrahedron(s)) { r.overlapping = true; break; }
    }

    Vec3 pa{}, pb{};
    for (int i = 0; i < s.count; ++i) {
        pa += s.v[i].a * s.bary[i];
        pb += s.v[i].b * s.bary[i];
    }

    if (r.overlapping) {
        // Core overlap (rare: cores are points/segments/boxes kept apart by
        // the speculative solver). Fall back to a center-based normal; depth
        // is estimated from the support extents along it, which is
        // conservative but keeps the response pointing the right way.
        Vec3 n = A.position - B.position;
        if (A.kind == Convex::Triangle)
            n = (A.segA + A.segB + A.triC) * (1.0f / 3.0f) - B.position;
        if (B.kind == Convex::Triangle)
            n = A.position - (B.segA + B.segB + B.triC) * (1.0f / 3.0f);
        n = lengthSq(n) > 1e-12f ? normalize(n) : Vec3{0, 1, 0};
        float depth = dot(n, B.support(n) - A.support(-n));
        r.normal = n;
        r.distance = -depth - A.radius - B.radius;
        r.pointA = A.support(-n);
        r.pointB = B.support(n);
        return r;
    }

    Vec3 delta = pa - pb;
    float coreDist = length(delta);
    r.normal = coreDist > 1e-8f ? delta * (1.0f / coreDist) : Vec3{0, 1, 0};
    r.distance = coreDist - A.radius - B.radius;
    r.pointA = pa - r.normal * A.radius;
    r.pointB = pb + r.normal * B.radius;
    return r;
}

} // namespace velox
