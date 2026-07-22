#pragma once

/**
 * @file shapes.h
 * @brief Extended convex-shape toolkit: parametric shape definitions, GJK
 *        support mappings, world-space AABBs, analytic + mesh-derived mass
 *        properties, a self-contained GJK/EPA narrow phase, and a lightweight
 *        V-HACD-style convex decomposition.
 *
 * The module is header-only and depends solely on `velox/math.h`. Every
 * geometry routine that the narrow phase needs is marked @ref VELOX_HD so the
 * exact same code runs on the CPU and CUDA backends. Mass-property and
 * decomposition *builders* are host-only (they allocate) and are guarded out
 * of device compilation.
 *
 * Design notes
 * ------------
 *  - Each shape is plain data. The unified @ref Shape carries a @ref ShapeKind
 *    tag plus the parameters for every kind, mirroring how `Body` stores shape
 *    data in `body.h`. Free `support()` overloads exist per parametric struct
 *    for callers that already know the concrete type.
 *  - Support mappings return *local-space* points (shape centered at the
 *    origin). `Shape::supportWorld` applies the transform.
 *  - ChamferedBox is modeled as the Minkowski sum of an inner box and an
 *    octahedron (L1 ball), which yields true flat 45-degree bevels.
 *  - SuperEllipsoid uses a single "squareness" exponent `n`: n = 2 is an
 *    ellipsoid, n -> infinity approaches a box, n = 1 is an octahedron.
 *  - ConvexDecomposition stores convex parts as raw point ranges so it stays
 *    device-friendly; the host-side @ref ConvexDecompositionBuilder owns the
 *    backing storage and produces a @ref DecompView into it.
 */

#include "math.h"

#include <cstdint>

#if !defined(__CUDACC__)
#include <algorithm>
#include <cmath>
#include <vector>
#endif

namespace velox {
namespace shapes {

using velox::Vec3;
using velox::Quat;
using velox::dot;
using velox::cross;
using velox::length;
using velox::lengthSq;
using velox::normalize;
using velox::rotate;
using velox::rotateInv;
using velox::vmin;
using velox::vmax;
using velox::vclamp;

/// @brief The convex shape kinds supported by this module.
enum class ShapeKind : uint8_t {
    Cylinder,          ///< Y-axis cylinder: radius + halfHeight.
    Cone,              ///< Y-axis cone: base radius at -halfHeight, apex at +halfHeight.
    Capsule,           ///< Y-axis capsule: radius + cylindrical halfHeight.
    RoundedBox,        ///< Box with spherical-rounded edges: halfExtents + radius.
    Ellipsoid,         ///< Tri-axial ellipsoid: halfExtents are the radii.
    ChamferedBox,      ///< Box with flat beveled edges: halfExtents + chamfer.
    SuperEllipsoid,    ///< Smooth box-like body: halfExtents + exponent.
    ConvexDecomposition ///< Union of convex hull parts (V-HACD-style).
};

// ---------------------------------------------------------------------------
// Parametric shape structs (plain data, local space, centered at the origin).
// ---------------------------------------------------------------------------

struct CylinderShape {
    float radius = 0.5f;    ///< Radial extent in the XZ plane.
    float halfHeight = 0.5f; ///< Half extent along +Y.
};

struct ConeShape {
    float radius = 0.5f;     ///< Base radius (base sits at y = -halfHeight).
    float halfHeight = 0.5f; ///< Half of the total height; apex at y = +halfHeight.
};

struct CapsuleShape {
    float radius = 0.5f;     ///< Radius of the cylindrical segment and end caps.
    float halfHeight = 0.5f; ///< Half length of the cylindrical segment.
};

struct RoundedBoxShape {
    Vec3 halfExtents{0.5f, 0.5f, 0.5f}; ///< Outer half extents (including rounding).
    float radius = 0.1f;                ///< Edge/corner rounding radius.
};

struct EllipsoidShape {
    Vec3 radii{0.5f, 0.5f, 0.5f}; ///< Principal radii (a, b, c).
};

struct ChamferedBoxShape {
    Vec3 halfExtents{0.5f, 0.5f, 0.5f}; ///< Outer half extents (including bevel).
    float chamfer = 0.1f;               ///< Bevel size cut from each edge.
};

struct SuperEllipsoidShape {
    Vec3 halfExtents{0.5f, 0.5f, 0.5f}; ///< Principal radii.
    float exponent = 2.0f;              ///< Squareness exponent n (>= 1).
};

// Device-friendly convex-decomposition view: raw ranges into host storage.
struct DecompPart {
    const Vec3* points = nullptr; ///< Local-space hull points for this part.
    uint32_t count = 0;           ///< Number of points.
    Vec3 offset;                  ///< Part offset in the decomposition frame.
};

struct DecompView {
    const DecompPart* parts = nullptr; ///< Array of convex parts.
    uint32_t count = 0;                ///< Number of parts.
};

// ---------------------------------------------------------------------------
// Per-struct GJK support mappings (local space).
// ---------------------------------------------------------------------------

/// @brief Local support point of a Y-axis cylinder.
VELOX_HD inline Vec3 support(const CylinderShape& s, const Vec3& dir) {
    float radial = sqrtf(dir.x * dir.x + dir.z * dir.z);
    return {radial > 1e-9f ? s.radius * dir.x / radial : 0.0f,
            dir.y >= 0.0f ? s.halfHeight : -s.halfHeight,
            radial > 1e-9f ? s.radius * dir.z / radial : 0.0f};
}

/// @brief Local support point of a Y-axis cone (base at -halfHeight).
VELOX_HD inline Vec3 support(const ConeShape& s, const Vec3& dir) {
    float radial = sqrtf(dir.x * dir.x + dir.z * dir.z);
    Vec3 base{radial > 1e-9f ? s.radius * dir.x / radial : 0.0f,
              -s.halfHeight,
              radial > 1e-9f ? s.radius * dir.z / radial : 0.0f};
    Vec3 apex{0.0f, s.halfHeight, 0.0f};
    return dot(dir, apex) > dot(dir, base) ? apex : base;
}

/// @brief Local support point of a Y-axis capsule (segment core + radius).
VELOX_HD inline Vec3 support(const CapsuleShape& s, const Vec3& dir) {
    Vec3 core{0.0f, dir.y >= 0.0f ? s.halfHeight : -s.halfHeight, 0.0f};
    Vec3 n = normalize(dir);
    return core + n * s.radius;
}

/// @brief Local support point of a rounded box (inner box core + radius).
VELOX_HD inline Vec3 support(const RoundedBoxShape& s, const Vec3& dir) {
    Vec3 core{dir.x >= 0.0f ? s.halfExtents.x - s.radius : -(s.halfExtents.x - s.radius),
              dir.y >= 0.0f ? s.halfExtents.y - s.radius : -(s.halfExtents.y - s.radius),
              dir.z >= 0.0f ? s.halfExtents.z - s.radius : -(s.halfExtents.z - s.radius)};
    return core + normalize(dir) * s.radius;
}

/// @brief Local support point of a tri-axial ellipsoid.
VELOX_HD inline Vec3 support(const EllipsoidShape& s, const Vec3& dir) {
    Vec3 scaled{s.radii.x * s.radii.x * dir.x,
                s.radii.y * s.radii.y * dir.y,
                s.radii.z * s.radii.z * dir.z};
    float len = length(scaled);
    return len > 1e-9f ? scaled * (1.0f / len) : Vec3{};
}

/// @brief Local support point of a chamfered box (inner box + octahedron).
VELOX_HD inline Vec3 support(const ChamferedBoxShape& s, const Vec3& dir) {
    Vec3 inner{dir.x >= 0.0f ? s.halfExtents.x - s.chamfer : -(s.halfExtents.x - s.chamfer),
               dir.y >= 0.0f ? s.halfExtents.y - s.chamfer : -(s.halfExtents.y - s.chamfer),
               dir.z >= 0.0f ? s.halfExtents.z - s.chamfer : -(s.halfExtents.z - s.chamfer)};
    // Support of the L1 ball {|x|+|y|+|z| <= chamfer} is chamfer * e_argmax.
    float ax = fabsf(dir.x), ay = fabsf(dir.y), az = fabsf(dir.z);
    Vec3 oct{0.0f, 0.0f, 0.0f};
    if (ax >= ay && ax >= az) oct.x = dir.x >= 0.0f ? s.chamfer : -s.chamfer;
    else if (ay >= az) oct.y = dir.y >= 0.0f ? s.chamfer : -s.chamfer;
    else oct.z = dir.z >= 0.0f ? s.chamfer : -s.chamfer;
    return inner + oct;
}

/// @brief Local support point of a superellipsoid with squareness exponent n.
VELOX_HD inline Vec3 support(const SuperEllipsoidShape& s, const Vec3& dir) {
    float n = s.exponent < 1.0f ? 1.0f : s.exponent;
    float ax = fabsf(dir.x), ay = fabsf(dir.y), az = fabsf(dir.z);
    if (ax < 1e-9f && ay < 1e-9f && az < 1e-9f) return {};
    if (n <= 1.0f + 1e-5f) {
        // Octahedron (L1 ball): pick the radius-weighted dominant axis vertex.
        float wx = s.halfExtents.x * ax, wy = s.halfExtents.y * ay,
              wz = s.halfExtents.z * az;
        if (wx >= wy && wx >= wz) return {dir.x >= 0.0f ? s.halfExtents.x : -s.halfExtents.x, 0, 0};
        if (wy >= wz) return {0, dir.y >= 0.0f ? s.halfExtents.y : -s.halfExtents.y, 0};
        return {0, 0, dir.z >= 0.0f ? s.halfExtents.z : -s.halfExtents.z};
    }
    float beta = 1.0f / (n - 1.0f);
    // u_i = r_i |d_i|; unnormalized support component = sign(d_i) r_i u_i^beta.
    auto comp = [&](float r, float d, float a) {
        if (a < 1e-12f) return 0.0f;
        float u = r * a;
        float mag = r * powf(u, beta);
        return d >= 0.0f ? mag : -mag;
    };
    Vec3 p{comp(s.halfExtents.x, dir.x, ax),
           comp(s.halfExtents.y, dir.y, ay),
           comp(s.halfExtents.z, dir.z, az)};
    // Scale so that sum |p_i / r_i|^n == 1.
    float acc = 0.0f;
    if (s.halfExtents.x > 0.0f) acc += powf(fabsf(p.x) / s.halfExtents.x, n);
    if (s.halfExtents.y > 0.0f) acc += powf(fabsf(p.y) / s.halfExtents.y, n);
    if (s.halfExtents.z > 0.0f) acc += powf(fabsf(p.z) / s.halfExtents.z, n);
    if (acc < 1e-20f) return {};
    float scale = powf(acc, -1.0f / n);
    return p * scale;
}

/// @brief Local support point of a single convex-decomposition part.
VELOX_HD inline Vec3 supportPart(const DecompPart& part, const Vec3& dir) {
    Vec3 best = part.points[0];
    float bestDot = dot(dir, best);
    for (uint32_t i = 1; i < part.count; ++i) {
        float t = dot(dir, part.points[i]);
        if (t > bestDot) { bestDot = t; best = part.points[i]; }
    }
    return part.offset + best;
}

/// @brief Local support point of a convex decomposition (max over all parts).
VELOX_HD inline Vec3 support(const DecompView& s, const Vec3& dir) {
    Vec3 best = supportPart(s.parts[0], dir);
    float bestDot = dot(dir, best);
    for (uint32_t i = 1; i < s.count; ++i) {
        Vec3 p = supportPart(s.parts[i], dir);
        float t = dot(dir, p);
        if (t > bestDot) { bestDot = t; best = p; }
    }
    return best;
}

// ---------------------------------------------------------------------------
// Unified Shape: kind tag + parameters + transform, with a dispatching support.
// ---------------------------------------------------------------------------

/// @brief A placed convex shape of any supported kind.
struct Shape {
    ShapeKind kind = ShapeKind::Ellipsoid;
    Vec3 position;                 ///< World-space center.
    Quat orientation{0, 0, 0, 1};  ///< World-space orientation.

    Vec3 halfExtents{0.5f, 0.5f, 0.5f}; ///< Box/RoundedBox/ChamferedBox/SuperEllipsoid/Ellipsoid.
    float radius = 0.5f;   ///< Cylinder/Cone/Capsule radius; RoundedBox rounding.
    float halfHeight = 0.5f; ///< Cylinder/Cone/Capsule half height.
    float chamfer = 0.1f;  ///< ChamferedBox bevel size.
    float exponent = 2.0f; ///< SuperEllipsoid squareness exponent.
    DecompView decomposition; ///< ConvexDecomposition parts (raw view).

    /// @brief Local-space support point for the active kind.
    VELOX_HD Vec3 supportLocal(const Vec3& dir) const {
        switch (kind) {
        case ShapeKind::Cylinder:
            return support(CylinderShape{radius, halfHeight}, dir);
        case ShapeKind::Cone:
            return support(ConeShape{radius, halfHeight}, dir);
        case ShapeKind::Capsule:
            return support(CapsuleShape{radius, halfHeight}, dir);
        case ShapeKind::RoundedBox:
            return support(RoundedBoxShape{halfExtents, radius}, dir);
        case ShapeKind::Ellipsoid:
            return support(EllipsoidShape{halfExtents}, dir);
        case ShapeKind::ChamferedBox:
            return support(ChamferedBoxShape{halfExtents, chamfer}, dir);
        case ShapeKind::SuperEllipsoid:
            return support(SuperEllipsoidShape{halfExtents, exponent}, dir);
        case ShapeKind::ConvexDecomposition:
            return support(decomposition, dir);
        }
        return {};
    }

    /// @brief World-space support point (transform applied).
    VELOX_HD Vec3 supportWorld(const Vec3& dir) const {
        Vec3 local = supportLocal(rotateInv(orientation, dir));
        return position + rotate(orientation, local);
    }
};

// Convenience constructors -------------------------------------------------

VELOX_HD inline Shape makeCylinder(float radius, float halfHeight,
                                   const Vec3& pos = {}, const Quat& rot = {0, 0, 0, 1}) {
    Shape s; s.kind = ShapeKind::Cylinder; s.radius = radius;
    s.halfHeight = halfHeight; s.position = pos; s.orientation = rot; return s;
}
VELOX_HD inline Shape makeCone(float radius, float halfHeight,
                               const Vec3& pos = {}, const Quat& rot = {0, 0, 0, 1}) {
    Shape s; s.kind = ShapeKind::Cone; s.radius = radius;
    s.halfHeight = halfHeight; s.position = pos; s.orientation = rot; return s;
}
VELOX_HD inline Shape makeCapsule(float radius, float halfHeight,
                                  const Vec3& pos = {}, const Quat& rot = {0, 0, 0, 1}) {
    Shape s; s.kind = ShapeKind::Capsule; s.radius = radius;
    s.halfHeight = halfHeight; s.position = pos; s.orientation = rot; return s;
}
VELOX_HD inline Shape makeRoundedBox(const Vec3& he, float radius,
                                     const Vec3& pos = {}, const Quat& rot = {0, 0, 0, 1}) {
    Shape s; s.kind = ShapeKind::RoundedBox; s.halfExtents = he;
    s.radius = radius; s.position = pos; s.orientation = rot; return s;
}
VELOX_HD inline Shape makeEllipsoid(const Vec3& radii,
                                    const Vec3& pos = {}, const Quat& rot = {0, 0, 0, 1}) {
    Shape s; s.kind = ShapeKind::Ellipsoid; s.halfExtents = radii;
    s.position = pos; s.orientation = rot; return s;
}
VELOX_HD inline Shape makeChamferedBox(const Vec3& he, float chamfer,
                                       const Vec3& pos = {}, const Quat& rot = {0, 0, 0, 1}) {
    Shape s; s.kind = ShapeKind::ChamferedBox; s.halfExtents = he;
    s.chamfer = chamfer; s.position = pos; s.orientation = rot; return s;
}
VELOX_HD inline Shape makeSuperEllipsoid(const Vec3& he, float exponent,
                                         const Vec3& pos = {}, const Quat& rot = {0, 0, 0, 1}) {
    Shape s; s.kind = ShapeKind::SuperEllipsoid; s.halfExtents = he;
    s.exponent = exponent; s.position = pos; s.orientation = rot; return s;
}
VELOX_HD inline Shape makeDecomposition(const DecompView& view,
                                        const Vec3& pos = {}, const Quat& rot = {0, 0, 0, 1}) {
    Shape s; s.kind = ShapeKind::ConvexDecomposition; s.decomposition = view;
    s.position = pos; s.orientation = rot; return s;
}

// ---------------------------------------------------------------------------
// World-space AABB (six support probes; exact axial extents for convex bodies).
// ---------------------------------------------------------------------------

struct Aabb {
    Vec3 min;
    Vec3 max;
};

/// @brief Tight world-space axis-aligned bounds of a placed shape.
VELOX_HD inline Aabb worldAabb(const Shape& s) {
    Aabb box;
    Vec3 px = s.supportWorld({1, 0, 0});
    Vec3 nx = s.supportWorld({-1, 0, 0});
    Vec3 py = s.supportWorld({0, 1, 0});
    Vec3 ny = s.supportWorld({0, -1, 0});
    Vec3 pz = s.supportWorld({0, 0, 1});
    Vec3 nz = s.supportWorld({0, 0, -1});
    box.min = {nx.x, ny.y, nz.z};
    box.max = {px.x, py.y, pz.z};
    return box;
}

/// @brief Conservative bounding radius about the shape center (for CCD/tolerance).
VELOX_HD inline float boundingRadius(const Shape& s) {
    switch (s.kind) {
    case ShapeKind::Cylinder:
        return sqrtf(s.halfHeight * s.halfHeight + s.radius * s.radius);
    case ShapeKind::Cone:
        return vmax(s.halfHeight,
                    sqrtf(s.radius * s.radius + s.halfHeight * s.halfHeight));
    case ShapeKind::Capsule:
        return s.halfHeight + s.radius;
    case ShapeKind::RoundedBox:
        return length(s.halfExtents) + s.radius;
    case ShapeKind::Ellipsoid:
        return vmax(s.halfExtents.x, vmax(s.halfExtents.y, s.halfExtents.z));
    case ShapeKind::ChamferedBox:
        return length(s.halfExtents);
    case ShapeKind::SuperEllipsoid:
        return vmax(s.halfExtents.x, vmax(s.halfExtents.y, s.halfExtents.z));
    case ShapeKind::ConvexDecomposition: {
        float r = 0.0f;
        for (uint32_t i = 0; i < s.decomposition.count; ++i) {
            const DecompPart& p = s.decomposition.parts[i];
            for (uint32_t j = 0; j < p.count; ++j)
                r = vmax(r, length(p.offset + p.points[j]));
        }
        return r;
    }
    }
    return 0.0f;
}

// ---------------------------------------------------------------------------
// Narrow phase: self-contained GJK (distance) + EPA (penetration).
// ---------------------------------------------------------------------------

struct NarrowphaseResult {
    bool colliding = false; ///< True when the volumes overlap.
    float distance = 0.0f;  ///< Separation distance when not colliding.
    float penetration = 0.0f; ///< Penetration depth when colliding.
    Vec3 normal;            ///< Contact normal, from B toward A (unit length).
    Vec3 pointA;            ///< Witness point on A's surface.
    Vec3 pointB;            ///< Witness point on B's surface.
};

namespace detail {

struct SupportVert {
    Vec3 p;  ///< Minkowski-difference point (a - b).
    Vec3 a;  ///< Witness on A.
    Vec3 b;  ///< Witness on B.
};

VELOX_HD inline SupportVert supportVertex(const Shape& A, const Shape& B,
                                          const Vec3& dir) {
    SupportVert v;
    v.a = A.supportWorld(dir);
    v.b = B.supportWorld(-dir);
    v.p = v.a - v.b;
    return v;
}

struct Simplex {
    SupportVert v[4];
    float bary[4];
    int count = 0;
};

VELOX_HD inline void solveSegment(Simplex& s) {
    Vec3 a = s.v[0].p, b = s.v[1].p;
    Vec3 ab = b - a;
    float t = -dot(a, ab);
    float denom = dot(ab, ab);
    if (t <= 0.0f || denom < 1e-20f) { s.count = 1; s.bary[0] = 1.0f; return; }
    if (t >= denom) { s.v[0] = s.v[1]; s.count = 1; s.bary[0] = 1.0f; return; }
    t /= denom;
    s.count = 2; s.bary[0] = 1.0f - t; s.bary[1] = t;
}

VELOX_HD inline void solveTriangle(Simplex& s) {
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
        s.v[1] = s.v[2]; s.count = 2; s.bary[0] = 1.0f - t; s.bary[1] = t; return;
    }
    float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
        float t = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        s.v[0] = s.v[1]; s.v[1] = s.v[2];
        s.count = 2; s.bary[0] = 1.0f - t; s.bary[1] = t; return;
    }
    float sum = va + vb + vc;
    if (sum < 1e-20f) {
        int best = 0;
        for (int i = 1; i < 3; ++i)
            if (lengthSq(s.v[i].p) < lengthSq(s.v[best].p)) best = i;
        s.v[0] = s.v[best]; s.count = 1; s.bary[0] = 1.0f; return;
    }
    float inv = 1.0f / sum;
    s.count = 3; s.bary[0] = va * inv; s.bary[1] = vb * inv; s.bary[2] = vc * inv;
}

VELOX_HD inline Vec3 simplexClosest(const Simplex& s) {
    Vec3 c{};
    for (int i = 0; i < s.count; ++i) c += s.v[i].p * s.bary[i];
    return c;
}

VELOX_HD inline void simplexWitness(const Simplex& s, Vec3& a, Vec3& b) {
    a = {}; b = {};
    for (int i = 0; i < s.count; ++i) {
        a += s.v[i].a * s.bary[i];
        b += s.v[i].b * s.bary[i];
    }
}

// Returns true when the origin is enclosed by the current tetrahedron.
VELOX_HD inline bool solveTetrahedron(Simplex& s) {
    Vec3 a = s.v[0].p, b = s.v[1].p, c = s.v[2].p, d = s.v[3].p;
    const int faces[4][3] = {{0, 1, 2}, {0, 3, 1}, {0, 2, 3}, {1, 3, 2}};
    Simplex best{};
    float bestDist = 1e30f;
    bool inside = true;
    for (int f = 0; f < 4; ++f) {
        int i0 = faces[f][0], i1 = faces[f][1], i2 = faces[f][2];
        int opp = 6 - i0 - i1 - i2;
        Vec3 fa = s.v[i0].p, fb = s.v[i1].p, fc = s.v[i2].p;
        Vec3 n = cross(fb - fa, fc - fa);
        float nLen = length(n);
        if (nLen < 1e-20f) continue;
        n = n * (1.0f / nLen);
        float originDist = dot(n, -fa);
        float oppDist = dot(n, s.v[opp].p - fa);
        if (originDist * oppDist < 0.0f || fabsf(originDist) < 1e-7f) {
            inside = false;
            Simplex tri{};
            tri.count = 3;
            tri.v[0] = s.v[i0]; tri.v[1] = s.v[i1]; tri.v[2] = s.v[i2];
            solveTriangle(tri);
            Vec3 closest = simplexClosest(tri);
            float dist = lengthSq(closest);
            if (dist < bestDist) { bestDist = dist; best = tri; }
        }
    }
    if (!inside) s = best;
    return inside;
}

/// @brief GJK distance/overlap query. Fills `simplex` for EPA on overlap.
VELOX_HD inline bool gjkQuery(const Shape& A, const Shape& B,
                              NarrowphaseResult& result, Simplex& simplex) {
    float scale = boundingRadius(A) + boundingRadius(B) + 1e-6f;
    Vec3 dir = B.position - A.position;
    if (lengthSq(dir) < 1e-12f) dir = {1, 0, 0};

    Simplex s{};
    s.v[0] = supportVertex(A, B, dir);
    s.count = 1;
    s.bary[0] = 1.0f;
    Vec3 closest = s.v[0].p;

    const int kMaxIter = 64;
    for (int iter = 0; iter < kMaxIter; ++iter) {
        dir = -closest;
        if (lengthSq(dir) < 1e-12f) dir = {1, 0, 0};
        SupportVert w = supportVertex(A, B, dir);
        // No progress toward the origin along the search direction => separated.
        if (dot(w.p, dir) - dot(closest, dir) < 1e-7f * scale && s.count > 1) {
            // Fall through: still add then re-solve unless it is a duplicate.
        }
        bool duplicate = false;
        for (int i = 0; i < s.count; ++i)
            if (lengthSq(w.p - s.v[i].p) < 1e-12f * scale * scale) duplicate = true;
        if (duplicate) {
            simplex = s;
            float dist = length(closest);
            result.colliding = dist < 1e-6f * scale;
            result.distance = dist;
            if (dist > 1e-9f) {
                result.normal = closest * (-1.0f / dist); // from B toward A
                simplexWitness(s, result.pointA, result.pointB);
            }
            return result.colliding;
        }
        s.v[s.count++] = w;
        if (s.count == 2) solveSegment(s);
        else if (s.count == 3) solveTriangle(s);
        else if (s.count == 4) {
            if (solveTetrahedron(s)) {
                simplex = s;
                result.colliding = true;
                return true;
            }
        }
        closest = simplexClosest(s);
        if (lengthSq(closest) < 1e-12f * scale * scale && s.count >= 2) {
            simplex = s;
            result.colliding = true;
            return true;
        }
    }
    simplex = s;
    float dist = length(closest);
    result.colliding = dist < 1e-5f * scale;
    result.distance = dist;
    if (dist > 1e-9f) {
        result.normal = closest * (-1.0f / dist);
        simplexWitness(s, result.pointA, result.pointB);
    }
    return result.colliding;
}

struct EpaFace {
    int a, b, c;
    Vec3 normal;
    float distance;
    bool valid;
};

VELOX_HD inline bool makeEpaFace(const SupportVert* verts, int ia, int ib, int ic,
                                 EpaFace& face) {
    Vec3 a = verts[ia].p, b = verts[ib].p, c = verts[ic].p;
    Vec3 n = cross(b - a, c - a);
    float n2 = lengthSq(n);
    if (n2 < 1e-16f) return false;
    n = n * (1.0f / sqrtf(n2));
    float distance = dot(n, a);
    if (distance < 0.0f) {
        int tmp = ib; ib = ic; ic = tmp;
        n = -n; distance = -distance;
    }
    face = {ia, ib, ic, n, distance, true};
    return true;
}

/// @brief Expanding Polytope Algorithm: penetration depth + witness points.
VELOX_HD inline bool epaPenetration(const Shape& A, const Shape& B,
                                    const Simplex& simplex, NarrowphaseResult& result) {
    constexpr int kMaxVerts = 64;
    constexpr int kMaxFaces = 128;
    constexpr int kMaxEdges = 192;
    SupportVert verts[kMaxVerts];
    EpaFace faces[kMaxFaces];
    int vertCount = 4, faceCount = 0;
    for (int i = 0; i < 4; ++i) verts[i] = simplex.v[i];

    const int initial[4][3] = {{0, 1, 2}, {0, 3, 1}, {0, 2, 3}, {1, 3, 2}};
    for (int i = 0; i < 4; ++i) {
        EpaFace f;
        if (makeEpaFace(verts, initial[i][0], initial[i][1], initial[i][2], f))
            faces[faceCount++] = f;
    }
    if (faceCount < 4) return false;

    float scale = boundingRadius(A) + boundingRadius(B) + 1e-6f;
    for (int iter = 0; iter < 64; ++iter) {
        int bestFace = -1;
        float bestDist = 1e30f;
        for (int i = 0; i < faceCount; ++i)
            if (faces[i].valid && faces[i].distance < bestDist) {
                bestDist = faces[i].distance; bestFace = i;
            }
        if (bestFace < 0) return false;

        EpaFace closest = faces[bestFace];
        SupportVert w = supportVertex(A, B, closest.normal);
        float supportDist = dot(closest.normal, w.p);
        float tol = 1e-4f * vmax(scale, fabsf(supportDist));

        bool duplicate = false;
        for (int i = 0; i < vertCount; ++i)
            if (lengthSq(w.p - verts[i].p) < 1e-12f * scale * scale) duplicate = true;

        if (supportDist - closest.distance <= tol || duplicate) {
            // Converged: build witness from the closest face barycentrics.
            Simplex tri{};
            tri.count = 3;
            tri.v[0] = verts[closest.a];
            tri.v[1] = verts[closest.b];
            tri.v[2] = verts[closest.c];
            solveTriangle(tri);
            simplexWitness(tri, result.pointA, result.pointB);
            result.normal = -closest.normal; // from B toward A
            result.penetration = closest.distance;
            result.colliding = true;
            return true;
        }
        if (vertCount >= kMaxVerts) break;
        int newVert = vertCount++;
        verts[newVert] = w;

        struct Edge { int a, b; } edges[kMaxEdges];
        int edgeCount = 0;
        for (int i = 0; i < faceCount; ++i) {
            if (!faces[i].valid) continue;
            const EpaFace& face = faces[i];
            if (dot(face.normal, w.p - verts[face.a].p) <= tol) continue;
            faces[i].valid = false;
            int fe[3][2] = {{face.a, face.b}, {face.b, face.c}, {face.c, face.a}};
            for (int e = 0; e < 3; ++e) {
                int reverse = -1;
                for (int k = 0; k < edgeCount; ++k)
                    if (edges[k].a == fe[e][1] && edges[k].b == fe[e][0]) { reverse = k; break; }
                if (reverse >= 0) edges[reverse] = edges[--edgeCount];
                else if (edgeCount < kMaxEdges) edges[edgeCount++] = {fe[e][0], fe[e][1]};
            }
        }
        for (int e = 0; e < edgeCount && faceCount < kMaxFaces; ++e) {
            EpaFace f;
            if (makeEpaFace(verts, edges[e].a, edges[e].b, newVert, f))
                faces[faceCount++] = f;
        }
    }
    return false;
}

} // namespace detail

/// @brief Full convex-vs-convex narrow-phase query (GJK distance or EPA depth).
VELOX_HD inline NarrowphaseResult collide(const Shape& A, const Shape& B) {
    NarrowphaseResult result{};
    detail::Simplex simplex{};
    bool overlap = detail::gjkQuery(A, B, result, simplex);
    if (!overlap) return result;
    // Overlap: refine penetration with EPA (needs a full tetrahedron seed).
    if (simplex.count == 4) {
        NarrowphaseResult epa{};
        if (detail::epaPenetration(A, B, simplex, epa)) return epa;
    }
    // Fallback: report overlap with a coarse axis from the shape centers.
    result.colliding = true;
    Vec3 axis = A.position - B.position;
    float len = length(axis);
    result.normal = len > 1e-9f ? axis * (1.0f / len) : Vec3{0, 1, 0};
    result.penetration = 0.0f;
    return result;
}

/// @brief Separation distance between two shapes (0 when overlapping).
VELOX_HD inline float separationDistance(const Shape& A, const Shape& B) {
    NarrowphaseResult r = collide(A, B);
    return r.colliding ? 0.0f : r.distance;
}

// ===========================================================================
// Host-only: mass properties, mesh integration, convex decomposition builder.
// ===========================================================================
#if !defined(__CUDACC__)

/// @brief Rigid-body mass properties at unit or specified density.
struct MassProperties {
    double volume = 0.0;        ///< Enclosed volume.
    double mass = 0.0;          ///< volume * density.
    Vec3 center;                ///< Center of mass (local space).
    Vec3 principalInertia;      ///< Principal moments (about the center of mass).
    Quat principalOrientation{0, 0, 0, 1}; ///< Principal-axis frame.
};

namespace detail {

struct Mat3 {
    double m[3][3]{};
};

/// @brief Jacobi eigen-decomposition of a symmetric 3x3 tensor.
inline void jacobiDiagonalize(Mat3 tensor, Vec3& moments, Quat& orientation) {
    Mat3 vectors;
    vectors.m[0][0] = vectors.m[1][1] = vectors.m[2][2] = 1.0;
    for (int iteration = 0; iteration < 24; ++iteration) {
        int p = 0, q = 1;
        if (std::fabs(tensor.m[0][2]) > std::fabs(tensor.m[p][q])) { p = 0; q = 2; }
        if (std::fabs(tensor.m[1][2]) > std::fabs(tensor.m[p][q])) { p = 1; q = 2; }
        if (std::fabs(tensor.m[p][q]) < 1e-14) break;
        double angle = 0.5 * std::atan2(2.0 * tensor.m[p][q],
                                        tensor.m[q][q] - tensor.m[p][p]);
        double c = std::cos(angle), s = std::sin(angle);
        for (int r = 0; r < 3; ++r) {
            if (r == p || r == q) continue;
            double arp = tensor.m[r][p], arq = tensor.m[r][q];
            tensor.m[r][p] = tensor.m[p][r] = c * arp - s * arq;
            tensor.m[r][q] = tensor.m[q][r] = s * arp + c * arq;
        }
        double app = tensor.m[p][p], aqq = tensor.m[q][q], apq = tensor.m[p][q];
        tensor.m[p][p] = c * c * app - 2.0 * s * c * apq + s * s * aqq;
        tensor.m[q][q] = s * s * app + 2.0 * s * c * apq + c * c * aqq;
        tensor.m[p][q] = tensor.m[q][p] = 0.0;
        for (int r = 0; r < 3; ++r) {
            double vrp = vectors.m[r][p], vrq = vectors.m[r][q];
            vectors.m[r][p] = c * vrp - s * vrq;
            vectors.m[r][q] = s * vrp + c * vrq;
        }
    }
    moments = {float(tensor.m[0][0]), float(tensor.m[1][1]), float(tensor.m[2][2])};
    float m00 = float(vectors.m[0][0]), m01 = float(vectors.m[0][1]);
    float m02 = float(vectors.m[0][2]), m10 = float(vectors.m[1][0]);
    float m11 = float(vectors.m[1][1]), m12 = float(vectors.m[1][2]);
    float m20 = float(vectors.m[2][0]), m21 = float(vectors.m[2][1]);
    float m22 = float(vectors.m[2][2]);
    float trace = m00 + m11 + m22;
    if (trace > 0.0f) {
        float s = std::sqrt(trace + 1.0f) * 2.0f;
        orientation = {(m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25f * s};
    } else if (m00 > m11 && m00 > m22) {
        float s = std::sqrt(1.0f + m00 - m11 - m22) * 2.0f;
        orientation = {0.25f * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s};
    } else if (m11 > m22) {
        float s = std::sqrt(1.0f + m11 - m00 - m22) * 2.0f;
        orientation = {(m01 + m10) / s, 0.25f * s, (m12 + m21) / s, (m02 - m20) / s};
    } else {
        float s = std::sqrt(1.0f + m22 - m00 - m11) * 2.0f;
        orientation = {(m02 + m20) / s, (m12 + m21) / s, 0.25f * s, (m10 - m01) / s};
    }
    orientation = velox::normalize(orientation);
}

/// @brief Divergence-theorem mass properties of a closed triangle mesh.
inline MassProperties massFromTriangles(const std::vector<Vec3>& verts,
                                        const std::vector<std::array<uint32_t, 3>>& tris,
                                        double density) {
    MassProperties result;
    double first[3]{};
    Mat3 second{};
    double volume = 0.0;
    for (const auto& t : tris) {
        const Vec3& a = verts[t[0]];
        const Vec3& b = verts[t[1]];
        const Vec3& c = verts[t[2]];
        double vol = double(dot(a, cross(b, c))) / 6.0;
        volume += vol;
        const double p[3][3] = {{a.x, a.y, a.z}, {b.x, b.y, b.z}, {c.x, c.y, c.z}};
        for (int axis = 0; axis < 3; ++axis)
            first[axis] += vol * (p[0][axis] + p[1][axis] + p[2][axis]) / 4.0;
        for (int x = 0; x < 3; ++x) for (int y = 0; y < 3; ++y) {
            double integral = 0.0;
            for (int i = 0; i < 3; ++i) integral += 2.0 * p[i][x] * p[i][y];
            for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j)
                if (i != j) integral += p[i][x] * p[j][y];
            second.m[x][y] += vol * integral / 20.0;
        }
    }
    result.volume = volume;
    result.mass = volume * density;
    if (std::fabs(volume) < 1e-12) return result;
    double center[3] = {first[0] / volume, first[1] / volume, first[2] / volume};
    result.center = {float(center[0]), float(center[1]), float(center[2])};
    Mat3 inertia;
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) {
        double origin = i == j
            ? second.m[(i + 1) % 3][(i + 1) % 3] + second.m[(i + 2) % 3][(i + 2) % 3]
            : -second.m[i][j];
        double shift = volume * ((i == j ? center[0] * center[0] + center[1] * center[1] +
                                  center[2] * center[2] : 0.0) - center[i] * center[j]);
        inertia.m[i][j] = (origin - shift) * density;
    }
    jacobiDiagonalize(inertia, result.principalInertia, result.principalOrientation);
    return result;
}

/// @brief Uniform direction sampling on the sphere (subdivided icosahedron).
inline std::vector<Vec3> sampleDirections(int subdivisions) {
    const float t = (1.0f + std::sqrt(5.0f)) * 0.5f;
    std::vector<Vec3> verts = {
        {-1, t, 0}, {1, t, 0}, {-1, -t, 0}, {1, -t, 0},
        {0, -1, t}, {0, 1, t}, {0, -1, -t}, {0, 1, -t},
        {t, 0, -1}, {t, 0, 1}, {-t, 0, -1}, {-t, 0, 1}};
    for (auto& v : verts) v = velox::normalize(v);
    std::vector<std::array<uint32_t, 3>> faces = {
        {0, 11, 5}, {0, 5, 1}, {0, 1, 7}, {0, 7, 10}, {0, 10, 11},
        {1, 5, 9}, {5, 11, 4}, {11, 10, 2}, {10, 7, 6}, {7, 1, 8},
        {3, 9, 4}, {3, 4, 2}, {3, 2, 6}, {3, 6, 8}, {3, 8, 9},
        {4, 9, 5}, {2, 4, 11}, {6, 2, 10}, {8, 6, 7}, {9, 8, 1}};
    for (int s = 0; s < subdivisions; ++s) {
        std::vector<std::array<uint32_t, 3>> next;
        next.reserve(faces.size() * 4);
        auto midpoint = [&](uint32_t a, uint32_t b) -> uint32_t {
            Vec3 m = velox::normalize((verts[a] + verts[b]) * 0.5f);
            // Reuse an existing vertex if very close (keeps the mesh watertight).
            for (uint32_t i = 0; i < verts.size(); ++i)
                if (lengthSq(verts[i] - m) < 1e-10f) return i;
            verts.push_back(m);
            return uint32_t(verts.size() - 1);
        };
        for (const auto& f : faces) {
            uint32_t a = f[0], b = f[1], c = f[2];
            uint32_t ab = midpoint(a, b), bc = midpoint(b, c), ca = midpoint(c, a);
            next.push_back({a, ab, ca});
            next.push_back({b, bc, ab});
            next.push_back({c, ca, bc});
            next.push_back({ab, bc, ca});
        }
        faces = std::move(next);
    }
    return verts;
}

/// @brief Support-sample a shape's surface into a point cloud (local space).
inline std::vector<Vec3> sampleSupportCloud(const Shape& s, int subdivisions) {
    std::vector<Vec3> dirs = sampleDirections(subdivisions);
    // Add structured axis/edge/diagonal directions to capture flat features.
    const Vec3 extra[] = {
        {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1},
        {1, 1, 0}, {1, -1, 0}, {-1, 1, 0}, {-1, -1, 0},
        {1, 0, 1}, {1, 0, -1}, {-1, 0, 1}, {-1, 0, -1},
        {0, 1, 1}, {0, 1, -1}, {0, -1, 1}, {0, -1, -1},
        {1, 1, 1}, {1, 1, -1}, {1, -1, 1}, {-1, 1, 1},
        {1, -1, -1}, {-1, 1, -1}, {-1, -1, 1}, {-1, -1, -1}};
    for (const Vec3& d : extra) dirs.push_back(velox::normalize(d));
    // Identity orientation: local support == world support.
    Shape local = s;
    local.position = {};
    local.orientation = {0, 0, 0, 1};
    std::vector<Vec3> cloud;
    cloud.reserve(dirs.size());
    for (const Vec3& d : dirs) cloud.push_back(local.supportLocal(d));
    return cloud;
}

// --- Compact incremental convex hull (QuickHull) for host point clouds -----

struct HullFace {
    uint32_t a, b, c;
    Vec3 normal;
    float offset;
    std::vector<uint32_t> outside;
    bool removed = false;
};

inline HullFace makeHullFace(uint32_t a, uint32_t b, uint32_t c,
                             const std::vector<Vec3>& pts, const Vec3& interior) {
    HullFace face{a, b, c, {}, 0.0f, {}, false};
    face.normal = velox::normalize(cross(pts[b] - pts[a], pts[c] - pts[a]));
    face.offset = dot(face.normal, pts[a]);
    if (dot(face.normal, interior) > face.offset) {
        std::swap(face.b, face.c);
        face.normal = -face.normal;
        face.offset = -face.offset;
    }
    return face;
}

/// @brief Triangulate the convex hull of a point cloud (returns index triples).
inline std::vector<std::array<uint32_t, 3>> convexHullTriangles(
    const std::vector<Vec3>& points) {
    std::vector<std::array<uint32_t, 3>> triangles;
    if (points.size() < 4) return triangles;
    Vec3 lo = points.front(), hi = lo;
    for (const Vec3& p : points) { lo = vmin(lo, p); hi = vmax(hi, p); }
    Vec3 extent = hi - lo;
    int axis = extent.y > extent.x ? 1 : 0;
    if ((axis == 0 ? extent.z > extent.x : extent.z > extent.y)) axis = 2;
    auto component = [&](uint32_t i) {
        return axis == 0 ? points[i].x : (axis == 1 ? points[i].y : points[i].z);
    };
    uint32_t i0 = 0, i1 = 0;
    for (uint32_t i = 1; i < points.size(); ++i) {
        if (component(i) < component(i0)) i0 = i;
        if (component(i) > component(i1)) i1 = i;
    }
    float scale = vmax(1.0f, length(extent));
    float epsilon = scale * 1e-5f;
    Vec3 initialEdge = points[i1] - points[i0];
    float edgeLenSq = lengthSq(initialEdge);
    if (edgeLenSq < 1e-12f) return triangles;
    uint32_t i2 = i0;
    float bestLine = 0.0f;
    for (uint32_t i = 0; i < points.size(); ++i) {
        float d = lengthSq(cross(initialEdge, points[i] - points[i0])) / edgeLenSq;
        if (d > bestLine) { bestLine = d; i2 = i; }
    }
    if (bestLine <= epsilon * epsilon) return triangles;
    Vec3 initialNormal = velox::normalize(cross(points[i1] - points[i0],
                                                points[i2] - points[i0]));
    uint32_t i3 = i0;
    float bestPlane = 0.0f;
    for (uint32_t i = 0; i < points.size(); ++i) {
        float d = std::fabs(dot(initialNormal, points[i] - points[i0]));
        if (d > bestPlane) { bestPlane = d; i3 = i; }
    }
    if (bestPlane <= epsilon) return triangles;
    Vec3 interior = (points[i0] + points[i1] + points[i2] + points[i3]) * 0.25f;
    std::vector<HullFace> faces;
    faces.push_back(makeHullFace(i0, i1, i2, points, interior));
    faces.push_back(makeHullFace(i0, i3, i1, points, interior));
    faces.push_back(makeHullFace(i0, i2, i3, points, interior));
    faces.push_back(makeHullFace(i1, i3, i2, points, interior));
    auto isInitial = [&](uint32_t i) {
        return i == i0 || i == i1 || i == i2 || i == i3;
    };
    for (uint32_t p = 0; p < points.size(); ++p) {
        if (isInitial(p)) continue;
        int bestFace = -1;
        float bestDist = epsilon;
        for (int f = 0; f < int(faces.size()); ++f) {
            float d = dot(faces[f].normal, points[p]) - faces[f].offset;
            if (d > bestDist) { bestDist = d; bestFace = f; }
        }
        if (bestFace >= 0) faces[bestFace].outside.push_back(p);
    }
    struct HorizonEdge { uint32_t from, to; int count = 1; };
    for (;;) {
        int seed = -1;
        for (int i = 0; i < int(faces.size()); ++i)
            if (!faces[i].removed && !faces[i].outside.empty()) { seed = i; break; }
        if (seed < 0) break;
        uint32_t eye = faces[seed].outside.front();
        float farthest = -1.0f;
        for (uint32_t p : faces[seed].outside) {
            float d = dot(faces[seed].normal, points[p]) - faces[seed].offset;
            if (d > farthest) { farthest = d; eye = p; }
        }
        std::vector<int> visible;
        std::vector<uint32_t> orphaned;
        std::vector<HorizonEdge> edges;
        for (int i = 0; i < int(faces.size()); ++i) {
            HullFace& face = faces[i];
            if (face.removed || dot(face.normal, points[eye]) - face.offset <= epsilon)
                continue;
            visible.push_back(i);
            orphaned.insert(orphaned.end(), face.outside.begin(), face.outside.end());
            uint32_t endpoints[3][2] = {{face.a, face.b}, {face.b, face.c}, {face.c, face.a}};
            for (const auto& e : endpoints) {
                auto found = std::find_if(edges.begin(), edges.end(),
                    [&](const HorizonEdge& he) {
                        return he.from == e[1] && he.to == e[0];
                    });
                if (found == edges.end()) edges.push_back({e[0], e[1]});
                else ++found->count;
            }
        }
        for (int idx : visible) {
            faces[idx].removed = true;
            faces[idx].outside.clear();
        }
        std::vector<int> newFaces;
        for (const HorizonEdge& e : edges) {
            if (e.count != 1) continue;
            newFaces.push_back(int(faces.size()));
            faces.push_back(makeHullFace(e.to, e.from, eye, points, interior));
        }
        for (uint32_t p : orphaned) {
            if (p == eye) continue;
            int bestFace = -1;
            float bestDist = epsilon;
            for (int f : newFaces) {
                float d = dot(faces[f].normal, points[p]) - faces[f].offset;
                if (d > bestDist) { bestDist = d; bestFace = f; }
            }
            if (bestFace >= 0) faces[bestFace].outside.push_back(p);
        }
    }
    for (const HullFace& f : faces)
        if (!f.removed) triangles.push_back({f.a, f.b, f.c});
    return triangles;
}

/// @brief Mass properties of a point cloud via its convex hull.
inline MassProperties massFromPointCloud(const std::vector<Vec3>& cloud, double density) {
    auto tris = convexHullTriangles(cloud);
    return massFromTriangles(cloud, tris, density);
}

} // namespace detail

// --- Analytic mass properties for the closed-form shapes ------------------

/// @brief Exact mass properties of a Y-axis cylinder.
inline MassProperties cylinderMass(const CylinderShape& s, double density) {
    MassProperties mp;
    double h = 2.0 * s.halfHeight;
    double r = s.radius;
    mp.volume = M_PI * r * r * h;
    mp.mass = mp.volume * density;
    double m = mp.mass;
    mp.principalInertia = {
        float(m * (3.0 * r * r + h * h) / 12.0), // about X
        float(0.5 * m * r * r),                  // about Y (symmetry axis)
        float(m * (3.0 * r * r + h * h) / 12.0)}; // about Z
    return mp;
}

/// @brief Exact mass properties of a Y-axis cone (base at -halfHeight).
inline MassProperties coneMass(const ConeShape& s, double density) {
    MassProperties mp;
    double h = 2.0 * s.halfHeight;
    double r = s.radius;
    mp.volume = M_PI * r * r * h / 3.0;
    mp.mass = mp.volume * density;
    double m = mp.mass;
    // Center of mass sits a quarter of the height above the base.
    mp.center = {0.0f, float(-s.halfHeight + h / 4.0), 0.0f};
    double perp = (3.0 / 80.0) * m * (4.0 * r * r + h * h);
    mp.principalInertia = {float(perp), float((3.0 / 10.0) * m * r * r), float(perp)};
    return mp;
}

/// @brief Exact mass properties of a Y-axis capsule.
inline MassProperties capsuleMass(const CapsuleShape& s, double density) {
    MassProperties mp;
    double r = s.radius;
    double h = 2.0 * s.halfHeight; // cylindrical segment length
    double cylVol = M_PI * r * r * h;
    double sphVol = (4.0 / 3.0) * M_PI * r * r * r; // two hemispheres
    mp.volume = cylVol + sphVol;
    mp.mass = mp.volume * density;
    double mc = cylVol * density;
    double mh = (sphVol * density) * 0.5; // each hemisphere
    // Symmetry axis (Y): cylinder + two hemispheres about the axis.
    double iy = 0.5 * mc * r * r + 2.0 * (2.0 / 5.0) * mh * r * r;
    // Perpendicular axis through the origin (center of mass by symmetry).
    double ix_cyl = mc * (3.0 * r * r + h * h) / 12.0;
    double hemiCentroid = s.halfHeight + 3.0 * r / 8.0;
    double ix_hemi = (83.0 / 320.0) * mh * r * r + mh * hemiCentroid * hemiCentroid;
    double ix = ix_cyl + 2.0 * ix_hemi;
    mp.principalInertia = {float(ix), float(iy), float(ix)};
    return mp;
}

/// @brief Exact mass properties of a tri-axial ellipsoid.
inline MassProperties ellipsoidMass(const EllipsoidShape& s, double density) {
    MassProperties mp;
    double a = s.radii.x, b = s.radii.y, c = s.radii.z;
    mp.volume = (4.0 / 3.0) * M_PI * a * b * c;
    mp.mass = mp.volume * density;
    double m = mp.mass;
    mp.principalInertia = {float(m * (b * b + c * c) / 5.0),
                           float(m * (a * a + c * c) / 5.0),
                           float(m * (a * a + b * b) / 5.0)};
    return mp;
}

/// @brief Mass properties of a rounded box (support-sampled hull integration).
inline MassProperties roundedBoxMass(const RoundedBoxShape& s, double density,
                                     int subdivisions = 3) {
    Shape sh = makeRoundedBox(s.halfExtents, s.radius);
    return detail::massFromPointCloud(detail::sampleSupportCloud(sh, subdivisions), density);
}

/// @brief Mass properties of a chamfered box (exact vertices + hull integration).
inline MassProperties chamferedBoxMass(const ChamferedBoxShape& s, double density) {
    double A = s.halfExtents.x, B = s.halfExtents.y, C = s.halfExtents.z;
    double d = s.chamfer;
    std::vector<Vec3> verts;
    for (int sx = -1; sx <= 1; sx += 2)
        for (int sy = -1; sy <= 1; sy += 2)
            for (int sz = -1; sz <= 1; sz += 2) {
                verts.push_back({float(sx * A), float(sy * B), float(sz * (C - d))});
                verts.push_back({float(sx * A), float(sy * (B - d)), float(sz * C)});
                verts.push_back({float(sx * (A - d)), float(sy * B), float(sz * C)});
            }
    return detail::massFromPointCloud(verts, density);
}

/// @brief Mass properties of a superellipsoid (support-sampled hull integration).
inline MassProperties superEllipsoidMass(const SuperEllipsoidShape& s, double density,
                                         int subdivisions = 3) {
    Shape sh = makeSuperEllipsoid(s.halfExtents, s.exponent);
    return detail::massFromPointCloud(detail::sampleSupportCloud(sh, subdivisions), density);
}

/// @brief Dispatch mass-property computation for any shape kind.
inline MassProperties computeMassProperties(const Shape& s, double density = 1.0) {
    switch (s.kind) {
    case ShapeKind::Cylinder:
        return cylinderMass(CylinderShape{s.radius, s.halfHeight}, density);
    case ShapeKind::Cone:
        return coneMass(ConeShape{s.radius, s.halfHeight}, density);
    case ShapeKind::Capsule:
        return capsuleMass(CapsuleShape{s.radius, s.halfHeight}, density);
    case ShapeKind::Ellipsoid:
        return ellipsoidMass(EllipsoidShape{s.halfExtents}, density);
    case ShapeKind::RoundedBox:
        return roundedBoxMass(RoundedBoxShape{s.halfExtents, s.radius}, density);
    case ShapeKind::ChamferedBox:
        return chamferedBoxMass(ChamferedBoxShape{s.halfExtents, s.chamfer}, density);
    case ShapeKind::SuperEllipsoid:
        return superEllipsoidMass(SuperEllipsoidShape{s.halfExtents, s.exponent}, density);
    case ShapeKind::ConvexDecomposition: {
        // Sum each hull part's mass properties with the parallel-axis theorem.
        MassProperties total{};
        detail::Mat3 aggregate{};
        double totalMass = 0.0;
        Vec3 weightedCenter{};
        for (uint32_t i = 0; i < s.decomposition.count; ++i) {
            const DecompPart& part = s.decomposition.parts[i];
            std::vector<Vec3> cloud(part.points, part.points + part.count);
            for (auto& p : p_unused(cloud)) {} // (no-op; keeps intent explicit)
            MassProperties pm = detail::massFromPointCloud(cloud, density);
            Vec3 c = pm.center + part.offset;
            total.volume += pm.volume;
            totalMass += pm.mass;
            weightedCenter += c * float(pm.mass);
        }
        if (totalMass > 0.0)
            total.center = weightedCenter * float(1.0 / totalMass);
        total.mass = totalMass;
        for (uint32_t i = 0; i < s.decomposition.count; ++i) {
            const DecompPart& part = s.decomposition.parts[i];
            std::vector<Vec3> cloud(part.points, part.points + part.count);
            MassProperties pm = detail::massFromPointCloud(cloud, density);
            Vec3 c = pm.center + part.offset;
            // Rotate principal inertia into the decomposition frame (parts are
            // axis-aligned here) then shift to the aggregate center of mass.
            Vec3 r = c - total.center;
            double rr = double(dot(r, r));
            double rp[3] = {r.x, r.y, r.z};
            double principal[3] = {pm.principalInertia.x, pm.principalInertia.y,
                                   pm.principalInertia.z};
            for (int row = 0; row < 3; ++row)
                for (int col = 0; col < 3; ++col) {
                    double centered = (row == col) ? principal[row] : 0.0;
                    double parallel = pm.mass *
                        ((row == col ? rr : 0.0) - rp[row] * rp[col]);
                    aggregate.m[row][col] += centered + parallel;
                }
        }
        detail::jacobiDiagonalize(aggregate, total.principalInertia,
                                  total.principalOrientation);
        return total;
    }
    }
    return {};
}

// ---------------------------------------------------------------------------
// Convex decomposition builder (lightweight V-HACD-style stand-in).
// ---------------------------------------------------------------------------

/// @brief One convex part owned by a @ref ConvexDecompositionBuilder.
struct BuilderPart {
    std::vector<Vec3> points; ///< Local-space hull points.
    Vec3 offset;              ///< Part offset in the decomposition frame.
};

/**
 * @brief Host-side owner of convex-decomposition geometry.
 *
 * Real V-HACD is an external library; this builder provides a self-contained
 * approximate convex decomposition by recursively bisecting a point cloud along
 * its longest principal axis. The resulting parts are convex clusters that a
 * GJK narrow phase can treat independently, matching V-HACD's usage pattern.
 * The builder owns the point storage; call @ref view to obtain a raw-pointer
 * @ref DecompView that stays valid for the builder's lifetime.
 */
class ConvexDecompositionBuilder {
public:
    std::vector<BuilderPart> parts; ///< Decomposed convex parts.

    /// @brief Add an explicit convex part (points are copied).
    void addPart(const std::vector<Vec3>& points, const Vec3& offset = {}) {
        parts.push_back({points, offset});
    }

    /**
     * @brief Recursively split a point cloud into up to `maxParts` clusters.
     * @param points    Input surface/sample points of a (possibly concave) body.
     * @param maxParts  Target maximum number of convex parts (>= 1).
     * @return A builder holding the decomposed parts.
     */
    static ConvexDecompositionBuilder decompose(const std::vector<Vec3>& points,
                                                int maxParts) {
        ConvexDecompositionBuilder builder;
        if (points.empty() || maxParts < 1) return builder;
        std::vector<std::vector<Vec3>> clusters = {points};
        while (int(clusters.size()) < maxParts) {
            // Pick the cluster with the greatest longest-axis spread to split.
            int target = -1;
            float bestSpread = 0.0f;
            Vec3 bestAxis{1, 0, 0};
            float bestMid = 0.0f;
            for (int i = 0; i < int(clusters.size()); ++i) {
                if (clusters[i].size() < 2) continue;
                Vec3 lo = clusters[i].front(), hi = lo;
                Vec3 mean{};
                for (const Vec3& p : clusters[i]) {
                    lo = vmin(lo, p); hi = vmax(hi, p);
                    mean += p;
                }
                mean = mean * float(1.0 / clusters[i].size());
                Vec3 extent = hi - lo;
                int axis = extent.y > extent.x ? 1 : 0;
                if ((axis == 0 ? extent.z > extent.x : extent.z > extent.y)) axis = 2;
                float spread = axis == 0 ? extent.x : (axis == 1 ? extent.y : extent.z);
                if (spread > bestSpread) {
                    bestSpread = spread;
                    target = i;
                    bestAxis = {axis == 0 ? 1.0f : 0.0f, axis == 1 ? 1.0f : 0.0f,
                                axis == 2 ? 1.0f : 0.0f};
                    bestMid = dot(mean, bestAxis);
                }
            }
            if (target < 0) break; // nothing left worth splitting
            std::vector<Vec3> low, high;
            for (const Vec3& p : clusters[target])
                (dot(p, bestAxis) <= bestMid ? low : high).push_back(p);
            if (low.empty() || high.empty()) break; // cannot split further
            clusters.erase(clusters.begin() + target);
            clusters.push_back(std::move(low));
            clusters.push_back(std::move(high));
        }
        for (auto& c : clusters)
            if (!c.empty()) builder.parts.push_back({std::move(c), {}});
        return builder;
    }

    /// @brief Build a raw-pointer view into this builder's storage.
    DecompView view() const {
        cache_.clear();
        cache_.reserve(parts.size());
        for (const auto& p : parts)
            cache_.push_back({p.points.data(), uint32_t(p.points.size()), p.offset});
        return {cache_.data(), uint32_t(cache_.size())};
    }

private:
    mutable std::vector<DecompPart> cache_; ///< Backing storage for view().
};

#endif // !defined(__CUDACC__)

} // namespace shapes
} // namespace velox
