// Scene queries: raycasts and overlap tests against every collider type.
#include "velox/world.h"
#include "narrowphase.h"
#include <stdexcept>

namespace velox {

namespace {

bool finiteQueryFloat(float value) { return std::isfinite(value); }
bool finiteQueryVec(const Vec3& v) {
    return finiteQueryFloat(v.x) && finiteQueryFloat(v.y) && finiteQueryFloat(v.z);
}
bool finiteQueryQuat(const Quat& q) {
    return finiteQueryFloat(q.x) && finiteQueryFloat(q.y) &&
           finiteQueryFloat(q.z) && finiteQueryFloat(q.w);
}

float validateQueryHull(const std::vector<Vec3>& points, Quat orientation) {
    if (points.size() < 4 || points.size() > UINT32_MAX)
        throw std::invalid_argument(
            "velox: convex hull query requires at least four points");
    if (!finiteQueryQuat(orientation))
        throw std::invalid_argument("velox: convex hull query orientation must be finite");
    float q2 = orientation.x * orientation.x + orientation.y * orientation.y +
               orientation.z * orientation.z + orientation.w * orientation.w;
    if (q2 < 1e-12f)
        throw std::invalid_argument("velox: convex hull query orientation must be non-zero");
    for (const Vec3& point : points)
        if (!finiteQueryVec(point))
            throw std::invalid_argument("velox: convex hull query points must be finite");

    const Vec3 p0 = points[0];
    float scale2 = 0.0f;
    size_t i1 = 0;
    for (size_t i = 1; i < points.size(); ++i) {
        float d2 = lengthSq(points[i] - p0);
        if (d2 > scale2) { scale2 = d2; i1 = i; }
    }
    if (scale2 < 1e-12f)
        throw std::invalid_argument("velox: convex hull query points are coincident");
    Vec3 edge = points[i1] - p0;
    float bestArea2 = 0.0f;
    size_t i2 = 0;
    for (size_t i = 1; i < points.size(); ++i) {
        float area2 = lengthSq(cross(edge, points[i] - p0));
        if (area2 > bestArea2) { bestArea2 = area2; i2 = i; }
    }
    if (bestArea2 <= scale2 * scale2 * 1e-12f)
        throw std::invalid_argument("velox: convex hull query points are collinear");
    Vec3 normal = cross(edge, points[i2] - p0);
    float planeDistance = 0.0f;
    float radius2 = 0.0f;
    for (const Vec3& point : points) {
        planeDistance = vmax(planeDistance, fabsf(dot(normal, point - p0)));
        radius2 = vmax(radius2, lengthSq(point));
    }
    float scale = sqrtf(scale2);
    if (planeDistance <= scale * scale * scale * 1e-6f)
        throw std::invalid_argument("velox: convex hull query points are coplanar");
    return sqrtf(radius2);
}

struct LocalHit { bool hit; float t; Vec3 normal; };

LocalHit raySphere(const Vec3& o, const Vec3& d, const Vec3& center, float r) {
    Vec3 m = o - center;
    float b = dot(m, d);
    float c = dot(m, m) - r * r;
    if (c > 0.0f && b > 0.0f) return {false};
    float disc = b * b - c;
    if (disc < 0.0f) return {false};
    float t = -b - sqrtf(disc);
    if (t < 0.0f) t = 0.0f;
    Vec3 p = o + d * t;
    return {true, t, normalize(p - center)};
}

LocalHit rayPlane(const Vec3& o, const Vec3& d, const Vec3& n, float offset) {
    float denom = dot(n, d);
    float dist = dot(n, o) - offset;
    if (denom > -1e-8f) return {false};        // parallel or exiting
    float t = -dist / denom;
    if (t < 0.0f) return {false};
    return {true, t, n};
}

// Slab test in the box's local frame.
LocalHit rayBox(const Vec3& o, const Vec3& d, const Body& box) {
    Vec3 lo = rotateInv(box.orientation, o - box.position);
    Vec3 ld = rotateInv(box.orientation, d);
    const Vec3& h = box.halfExtents;
    float tmin = 0.0f, tmax = 1e30f;
    Vec3 nLocal{};
    const float lov[3] = {lo.x, lo.y, lo.z}, ldv[3] = {ld.x, ld.y, ld.z};
    const float hv[3] = {h.x, h.y, h.z};
    for (int i = 0; i < 3; ++i) {
        if (std::fabs(ldv[i]) < 1e-9f) {
            if (std::fabs(lov[i]) > hv[i]) return {false};
            continue;
        }
        float inv = 1.0f / ldv[i];
        float t1 = (-hv[i] - lov[i]) * inv;
        float t2 = (hv[i] - lov[i]) * inv;
        float sign = -1.0f;
        if (t1 > t2) { float tt = t1; t1 = t2; t2 = tt; sign = 1.0f; }
        if (t1 > tmin) {
            tmin = t1;
            nLocal = {};
            (&nLocal.x)[i] = sign;
        }
        if (t2 < tmax) tmax = t2;
        if (tmin > tmax) return {false};
    }
    return {true, tmin, rotate(box.orientation, nLocal)};
}

LocalHit rayCapsule(const Vec3& o, const Vec3& d, const Body& cap) {
    // Local frame: core segment on the Y axis, radius r.
    Vec3 lo = rotateInv(cap.orientation, o - cap.position);
    Vec3 ld = rotateInv(cap.orientation, d);
    float r = cap.radius, h = cap.capsuleHalfHeight;

    // Infinite cylinder x^2 + z^2 = r^2.
    float a = ld.x * ld.x + ld.z * ld.z;
    LocalHit best{false, 1e30f, {}};
    if (a > 1e-12f) {
        float b = lo.x * ld.x + lo.z * ld.z;
        float c = lo.x * lo.x + lo.z * lo.z - r * r;
        float disc = b * b - a * c;
        if (disc >= 0.0f) {
            float t = (-b - sqrtf(disc)) / a;
            if (t >= 0.0f) {
                float y = lo.y + ld.y * t;
                if (y >= -h && y <= h)
                    best = {true, t, normalize(Vec3{lo.x + ld.x * t, 0, lo.z + ld.z * t})};
            }
        }
    }
    // End cap spheres.
    for (float s : {h, -h}) {
        LocalHit hc = raySphere(lo, ld, {0, s, 0}, r);
        if (hc.hit && hc.t < best.t) best = hc;
    }
    if (!best.hit) return best;
    best.normal = rotate(cap.orientation, best.normal);
    return best;
}

// Möller–Trumbore, front and back faces.
LocalHit rayTriangle(const Vec3& o, const Vec3& d,
                     const Vec3& v0, const Vec3& v1, const Vec3& v2) {
    Vec3 e1 = v1 - v0, e2 = v2 - v0;
    Vec3 p = cross(d, e2);
    float det = dot(e1, p);
    if (std::fabs(det) < 1e-12f) return {false};
    float inv = 1.0f / det;
    Vec3 s = o - v0;
    float u = dot(s, p) * inv;
    if (u < 0.0f || u > 1.0f) return {false};
    Vec3 q = cross(s, e1);
    float v = dot(d, q) * inv;
    if (v < 0.0f || u + v > 1.0f) return {false};
    float t = dot(e2, q) * inv;
    if (t < 0.0f) return {false};
    Vec3 n = normalize(cross(e1, e2));
    if (dot(n, d) > 0.0f) n = -n;
    return {true, t, n};
}

bool rayAabb(const Vec3& o, const Vec3& d, const Vec3& lo, const Vec3& hi, float tmax) {
    float t0 = 0.0f, t1 = tmax;
    const float ov[3] = {o.x, o.y, o.z}, dv[3] = {d.x, d.y, d.z};
    const float lov[3] = {lo.x, lo.y, lo.z}, hiv[3] = {hi.x, hi.y, hi.z};
    for (int i = 0; i < 3; ++i) {
        if (std::fabs(dv[i]) < 1e-9f) {
            if (ov[i] < lov[i] || ov[i] > hiv[i]) return false;
            continue;
        }
        float inv = 1.0f / dv[i];
        float a = (lov[i] - ov[i]) * inv, b = (hiv[i] - ov[i]) * inv;
        if (a > b) { float tt = a; a = b; b = tt; }
        if (a > t0) t0 = a;
        if (b < t1) t1 = b;
        if (t0 > t1) return false;
    }
    return true;
}

LocalHit rayMesh(const Vec3& o, const Vec3& d, const Mesh& m,
                 const MeshSoupView& soup, float tmax) {
    LocalHit best{false, tmax, {}};
    uint32_t stack[48];
    int sp = 0;
    stack[sp++] = m.firstNode;
    while (sp > 0) {
        const BvhNode& node = soup.bvhNodes[stack[--sp]];
        if (!rayAabb(o, d, node.aabbMin, node.aabbMax, best.t)) continue;
        if (node.triCount == 0) {
            if (sp + 2 <= 48) {
                stack[sp++] = node.leftFirst;
                stack[sp++] = node.leftFirst + 1;
            }
            continue;
        }
        for (uint32_t k = 0; k < node.triCount; ++k) {
            uint32_t tri = soup.bvhTriRefs[node.leftFirst + k];
            uint32_t base = m.firstIndex + tri * 3;
            LocalHit h = rayTriangle(o, d,
                soup.vertices[m.firstVertex + soup.indices[base]],
                soup.vertices[m.firstVertex + soup.indices[base + 1]],
                soup.vertices[m.firstVertex + soup.indices[base + 2]]);
            if (h.hit && h.t < best.t) { best = h; best.hit = true; }
        }
    }
    return best;
}

// Conservative-advancement raycast against any convex: march the ray point
// forward by the GJK distance to the hull until touching (or past maxDist).
LocalHit rayConvex(const Vec3& o, const Vec3& d, const Body& body,
                   const MeshSoupView& soup, float maxDist) {
    Convex hull = makeConvex(body, soup);
    Convex probe{};
    probe.kind = Convex::Point;
    float t = 0.0f;
    Vec3 lastNormal{};
    for (int iter = 0; iter < 64; ++iter) {
        probe.position = o + d * t;
        GjkResult r = gjkDistance(probe, hull);
        lastNormal = r.normal;
        if (r.distance < 1e-4f) return {true, t, lastNormal};
        float advance = dot(r.normal, -d);   // closing rate along the ray
        if (advance < 1e-6f) return {false}; // moving away or parallel
        t += r.distance / advance;
        if (t > maxDist) return {false};
    }
    return {false};
}

LocalHit rayBody(const Vec3& origin, const Vec3& dir, const Body& body,
                 const MeshSoupView& soup, float maxDist) {
    if (body.shape == ShapeType::Compound) {
        LocalHit best{false, maxDist, {}};
        for (uint32_t i = 0; i < body.compoundCount; ++i) {
            Body child = compoundChildBody(
                body, soup.compoundChildren[body.compoundFirst + i]);
            LocalHit hit = rayBody(origin, dir, child, soup, best.t);
            if (hit.hit && hit.t < best.t) best = hit;
        }
        return best;
    }
    switch (body.shape) {
    case ShapeType::Sphere:
        return raySphere(origin, dir, body.position, body.radius);
    case ShapeType::Box:
        return rayBox(origin, dir, body);
    case ShapeType::Capsule:
        return rayCapsule(origin, dir, body);
    case ShapeType::Plane:
        return rayPlane(origin, dir, body.planeNormal, body.planeOffset);
    case ShapeType::Mesh:
        return rayMesh(origin, dir, soup.meshes[body.meshIndex], soup, maxDist);
    case ShapeType::Hull:
    case ShapeType::Cylinder:
    case ShapeType::Cone:
        return rayConvex(origin, dir, body, soup, maxDist);
    default:
        return {false};
    }
}

struct QueryGap { float gap; Vec3 normal, point; };

QueryGap queryGap(const Body& probe, const Body& target,
                  const MeshSoupView& soup, float searchRadius) {
    if (target.shape == ShapeType::Compound) {
        QueryGap best{1e30f, {0, 1, 0}, {}};
        for (uint32_t i = 0; i < target.compoundCount; ++i) {
            Body child = compoundChildBody(
                target, soup.compoundChildren[target.compoundFirst + i]);
            QueryGap gap = queryGap(probe, child, soup, searchRadius);
            if (gap.gap < best.gap) best = gap;
        }
        return best;
    }
    Convex query = makeConvex(probe, soup);
    if (target.shape == ShapeType::Plane) {
        Vec3 point = query.support(-target.planeNormal);
        float gap = dot(target.planeNormal, point) - target.planeOffset;
        return {gap, target.planeNormal, point - target.planeNormal * (0.5f * gap)};
    }
    if (target.shape == ShapeType::Mesh) {
        float gap = 1e30f;
        Vec3 normal{0, 1, 0};
        meshGapProbe(probe, target, soup, searchRadius, gap, normal);
        Vec3 point = query.support(-normal) - normal * (0.5f * gap);
        return {gap, normal, point};
    }
    GjkResult result = gjkDistance(query, makeConvex(target, soup));
    return {result.distance, result.normal,
            (result.pointA + result.pointB) * 0.5f};
}

} // namespace

bool World::queryAllows(BodyIndex dense, const QueryFilter& filter) const {
    const Body& body = bodies_[dense];
    BodyId handle = bodyHandle(dense);
    if (handle == filter.ignoredBody) return false;
    if (!filter.includeSensors && body.isSensor()) return false;
    return (filter.maskBits & body.categoryBits) != 0 &&
           (body.maskBits & filter.categoryBits) != 0;
}

RayHit World::rayCast(Vec3 origin, Vec3 dir, float maxDist,
                      const QueryFilter& filter) const {
    if (!finiteQueryVec(origin) || !finiteQueryVec(dir) ||
        !finiteQueryFloat(maxDist) || maxDist < 0.0f)
        throw std::invalid_argument("velox: ray cast inputs must be finite and maxDist non-negative");
    if (lengthSq(dir) < 1e-12f)
        throw std::invalid_argument("velox: ray direction must be non-zero");
    dir = normalize(dir);
    RayHit best;
    best.t = maxDist;
    const MeshSoupView soup = view(meshes_);

    for (BodyIndex i = 0; i < bodies_.size(); ++i) {
        if (!queryAllows(i, filter)) continue;
        const Body& b = bodies_[i];
        LocalHit h = rayBody(origin, dir, b, soup, best.t);
        if (h.hit && h.t < best.t) {
            best.hit = true;
            best.body = bodyHandle(i);
            best.t = h.t;
            best.normal = h.normal;
        }
    }
    if (best.hit) best.point = origin + dir * best.t;
    return best;
}

void World::overlapShape(const Body& shape, std::vector<BodyId>& out,
                         const QueryFilter& filter) const {
    const MeshSoupView soup = view(meshes_);
    overlapShapeWithSoup(shape, out, filter, soup);
}

void World::overlapShapeWithSoup(const Body& shape, std::vector<BodyId>& out,
                                 const QueryFilter& filter,
                                 const MeshSoupView& soup) const {
    out.clear();
    float searchRadius = shape.radius + length(shape.halfExtents) +
                         shape.capsuleHalfHeight + 0.1f;
    for (BodyIndex i = 0; i < bodies_.size(); ++i) {
        if (!queryAllows(i, filter)) continue;
        QueryGap gap = queryGap(shape, bodies_[i], soup, searchRadius);
        if (gap.gap <= 0.0f) out.push_back(bodyHandle(i));
    }
}

void World::overlapConvexHull(Vec3 center, const std::vector<Vec3>& points,
                              Quat orientation, std::vector<BodyId>& out,
                              const QueryFilter& filter) const {
    if (!finiteQueryVec(center))
        throw std::invalid_argument("velox: convex hull query center must be finite");
    Body probe;
    probe.shape = ShapeType::Hull;
    probe.position = center;
    probe.orientation = normalize(orientation);
    probe.radius = validateQueryHull(points, orientation);
    if (meshes_.hullPoints.size() > UINT32_MAX - points.size())
        throw std::length_error("velox: convex hull query point capacity exceeded");
    probe.hullFirst = static_cast<uint32_t>(meshes_.hullPoints.size());
    probe.hullCount = static_cast<uint32_t>(points.size());
    std::vector<Vec3> hullPoints = meshes_.hullPoints;
    hullPoints.insert(hullPoints.end(), points.begin(), points.end());
    MeshSoupView soup = view(meshes_);
    soup.hullPoints = hullPoints.data();
    overlapShapeWithSoup(probe, out, filter, soup);
}

void World::overlapSphere(Vec3 center, float radius, std::vector<BodyId>& out,
                          const QueryFilter& filter) const {
    if (!finiteQueryVec(center) || !finiteQueryFloat(radius) || radius <= 0.0f)
        throw std::invalid_argument("velox: overlap sphere requires a finite center and positive radius");
    Body probe;
    probe.shape = ShapeType::Sphere;
    probe.position = center;
    probe.radius = radius;
    overlapShape(probe, out, filter);
}

void World::overlapBox(Vec3 center, Vec3 halfExtents, Quat orientation,
                       std::vector<BodyId>& out, const QueryFilter& filter) const {
    if (!finiteQueryVec(center) || !finiteQueryVec(halfExtents) ||
        halfExtents.x <= 0.0f || halfExtents.y <= 0.0f || halfExtents.z <= 0.0f ||
        !finiteQueryQuat(orientation))
        throw std::invalid_argument("velox: overlap box requires finite positive geometry");
    float q2 = orientation.x * orientation.x + orientation.y * orientation.y +
               orientation.z * orientation.z + orientation.w * orientation.w;
    if (q2 < 1e-12f)
        throw std::invalid_argument("velox: overlap box orientation must be non-zero");
    Body probe;
    probe.shape = ShapeType::Box;
    probe.position = center;
    probe.orientation = normalize(orientation);
    probe.halfExtents = halfExtents;
    overlapShape(probe, out, filter);
}

void World::overlapCapsule(Vec3 center, float radius, float halfHeight,
                           Quat orientation, std::vector<BodyId>& out,
                           const QueryFilter& filter) const {
    if (!finiteQueryVec(center) || !finiteQueryFloat(radius) || radius <= 0.0f ||
        !finiteQueryFloat(halfHeight) || halfHeight < 0.0f ||
        !finiteQueryQuat(orientation))
        throw std::invalid_argument("velox: overlap capsule requires finite positive geometry");
    float q2 = orientation.x * orientation.x + orientation.y * orientation.y +
               orientation.z * orientation.z + orientation.w * orientation.w;
    if (q2 < 1e-12f)
        throw std::invalid_argument("velox: overlap capsule orientation must be non-zero");
    Body probe;
    probe.shape = ShapeType::Capsule;
    probe.position = center;
    probe.orientation = normalize(orientation);
    probe.radius = radius;
    probe.capsuleHalfHeight = halfHeight;
    overlapShape(probe, out, filter);
}

ShapeCastHit World::castShape(Body shape, Vec3 direction, float maxDist,
                              const QueryFilter& filter) const {
    const MeshSoupView soup = view(meshes_);
    return castShapeWithSoup(shape, direction, maxDist, filter, soup);
}

ShapeCastHit World::castShapeWithSoup(Body shape, Vec3 direction, float maxDist,
                                      const QueryFilter& filter,
                                      const MeshSoupView& soup) const {
    if (!finiteQueryVec(direction) || lengthSq(direction) < 1e-12f ||
        !finiteQueryFloat(maxDist) || maxDist < 0.0f)
        throw std::invalid_argument(
            "velox: shape cast requires a non-zero finite direction and non-negative distance");
    direction = normalize(direction);
    Vec3 start = shape.position;
    float bound = shape.radius + length(shape.halfExtents) +
                  shape.capsuleHalfHeight + 0.1f;
    float searchRadius = maxDist + bound;
    ShapeCastHit best;
    best.distance = maxDist;

    for (BodyIndex i = 0; i < bodies_.size(); ++i) {
        if (!queryAllows(i, filter)) continue;
        float distance = 0.0f;
        QueryGap last{1e30f, {0, 1, 0}, {}};
        bool hit = false;
        for (int iteration = 0; iteration < 64; ++iteration) {
            shape.position = start + direction * distance;
            last = queryGap(shape, bodies_[i], soup, searchRadius);
            if (last.gap <= 1e-4f) { hit = true; break; }
            float closingSpeed = -dot(last.normal, direction);
            if (closingSpeed <= 1e-6f) break;
            distance += last.gap / closingSpeed;
            if (distance > best.distance || distance > maxDist) break;
        }
        if (hit && distance <= best.distance) {
            best.hit = true;
            best.body = bodyHandle(i);
            best.distance = distance;
            best.fraction = maxDist > 0.0f ? distance / maxDist : 0.0f;
            best.point = last.point;
            best.normal = last.normal;
        }
    }
    return best;
}

ShapeCastHit World::convexHullCast(Vec3 center,
                                   const std::vector<Vec3>& points,
                                   Quat orientation, Vec3 direction,
                                   float maxDist,
                                   const QueryFilter& filter) const {
    if (!finiteQueryVec(center))
        throw std::invalid_argument("velox: convex hull cast center must be finite");
    Body shape;
    shape.shape = ShapeType::Hull;
    shape.position = center;
    shape.orientation = normalize(orientation);
    shape.radius = validateQueryHull(points, orientation);
    if (meshes_.hullPoints.size() > UINT32_MAX - points.size())
        throw std::length_error("velox: convex hull cast point capacity exceeded");
    shape.hullFirst = static_cast<uint32_t>(meshes_.hullPoints.size());
    shape.hullCount = static_cast<uint32_t>(points.size());
    std::vector<Vec3> hullPoints = meshes_.hullPoints;
    hullPoints.insert(hullPoints.end(), points.begin(), points.end());
    MeshSoupView soup = view(meshes_);
    soup.hullPoints = hullPoints.data();
    return castShapeWithSoup(shape, direction, maxDist, filter, soup);
}

ShapeCastHit World::sphereCast(Vec3 center, float radius, Vec3 direction,
                               float maxDist, const QueryFilter& filter) const {
    if (!finiteQueryVec(center) || !finiteQueryFloat(radius) || radius <= 0.0f)
        throw std::invalid_argument("velox: sphere cast requires a finite center and radius");
    Body shape;
    shape.shape = ShapeType::Sphere;
    shape.position = center;
    shape.radius = radius;
    return castShape(shape, direction, maxDist, filter);
}

ShapeCastHit World::boxCast(Vec3 center, Vec3 halfExtents, Quat orientation,
                            Vec3 direction, float maxDist,
                            const QueryFilter& filter) const {
    if (!finiteQueryVec(center) || !finiteQueryVec(halfExtents) ||
        halfExtents.x <= 0.0f || halfExtents.y <= 0.0f || halfExtents.z <= 0.0f ||
        !finiteQueryQuat(orientation))
        throw std::invalid_argument("velox: box cast requires finite positive geometry");
    float q2 = orientation.x * orientation.x + orientation.y * orientation.y +
               orientation.z * orientation.z + orientation.w * orientation.w;
    if (q2 < 1e-12f)
        throw std::invalid_argument("velox: box cast orientation must be non-zero");
    Body shape;
    shape.shape = ShapeType::Box;
    shape.position = center;
    shape.orientation = normalize(orientation);
    shape.halfExtents = halfExtents;
    return castShape(shape, direction, maxDist, filter);
}

ShapeCastHit World::capsuleCast(Vec3 center, float radius, float halfHeight,
                                Quat orientation, Vec3 direction, float maxDist,
                                const QueryFilter& filter) const {
    if (!finiteQueryVec(center) || !finiteQueryFloat(radius) || radius <= 0.0f ||
        !finiteQueryFloat(halfHeight) || halfHeight < 0.0f ||
        !finiteQueryQuat(orientation))
        throw std::invalid_argument("velox: capsule cast requires finite positive geometry");
    float q2 = orientation.x * orientation.x + orientation.y * orientation.y +
               orientation.z * orientation.z + orientation.w * orientation.w;
    if (q2 < 1e-12f)
        throw std::invalid_argument("velox: capsule cast orientation must be non-zero");
    Body shape;
    shape.shape = ShapeType::Capsule;
    shape.position = center;
    shape.orientation = normalize(orientation);
    shape.radius = radius;
    shape.capsuleHalfHeight = halfHeight;
    return castShape(shape, direction, maxDist, filter);
}

} // namespace velox
