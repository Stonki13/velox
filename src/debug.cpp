#include "velox/world.h"
#include "narrowphase.h"
#include <cmath>

namespace velox {
namespace {

constexpr uint32_t kShapeColor = 0x58c4ddffu;
constexpr uint32_t kStaticColor = 0x8b95a5ffu;
constexpr uint32_t kAabbColor = 0xf2c14effu;
constexpr uint32_t kContactColor = 0xf45b69ffu;
constexpr uint32_t kJointColor = 0x7bd389ffu;
constexpr float kTau = 6.28318530718f;

void addLine(std::vector<DebugLine>& out, Vec3 a, Vec3 b, uint32_t color) {
    out.push_back({a, b, color});
}

void addCircle(std::vector<DebugLine>& out, Vec3 center, Vec3 u, Vec3 v,
               float radius, uint32_t color) {
    constexpr int segments = 16;
    Vec3 previous = center + u * radius;
    for (int i = 1; i <= segments; ++i) {
        float angle = kTau * float(i) / float(segments);
        Vec3 point = center + (u * cosf(angle) + v * sinf(angle)) * radius;
        addLine(out, previous, point, color);
        previous = point;
    }
}

void addBox(std::vector<DebugLine>& out, Vec3 center, Quat orientation,
            Vec3 half, uint32_t color) {
    Vec3 corners[8];
    for (int i = 0; i < 8; ++i) {
        Vec3 local{(i & 1) ? half.x : -half.x,
                   (i & 2) ? half.y : -half.y,
                   (i & 4) ? half.z : -half.z};
        corners[i] = center + rotate(orientation, local);
    }
    constexpr int edges[12][2] = {
        {0,1},{2,3},{4,5},{6,7}, {0,2},{1,3},{4,6},{5,7},
        {0,4},{1,5},{2,6},{3,7}};
    for (const auto& edge : edges)
        addLine(out, corners[edge[0]], corners[edge[1]], color);
}

void addAabb(std::vector<DebugLine>& out, Vec3 lo, Vec3 hi) {
    addBox(out, (lo + hi) * 0.5f, {}, (hi - lo) * 0.5f, kAabbColor);
}

void addPointCross(std::vector<DebugLine>& out, Vec3 point, uint32_t color) {
    constexpr float s = 0.04f;
    addLine(out, point - Vec3{s,0,0}, point + Vec3{s,0,0}, color);
    addLine(out, point - Vec3{0,s,0}, point + Vec3{0,s,0}, color);
    addLine(out, point - Vec3{0,0,s}, point + Vec3{0,0,s}, color);
}

void addShape(std::vector<DebugLine>& out, const Body& body,
              const MeshSoupView& soup, uint32_t color) {
    if (body.shape == ShapeType::Compound) {
        for (uint32_t i = 0; i < body.compoundCount; ++i)
            addShape(out, compoundChildBody(
                body, soup.compoundChildren[body.compoundFirst + i]), soup, color);
        return;
    }
    Vec3 x = rotate(body.orientation, {1,0,0});
    Vec3 y = rotate(body.orientation, {0,1,0});
    Vec3 z = rotate(body.orientation, {0,0,1});
    switch (body.shape) {
    case ShapeType::Sphere:
        addCircle(out, body.position, x, y, body.radius, color);
        addCircle(out, body.position, x, z, body.radius, color);
        addCircle(out, body.position, y, z, body.radius, color);
        break;
    case ShapeType::Box:
        addBox(out, body.position, body.orientation, body.halfExtents, color);
        break;
    case ShapeType::Capsule: {
        Vec3 a = body.position - y * body.capsuleHalfHeight;
        Vec3 b = body.position + y * body.capsuleHalfHeight;
        addCircle(out, a, x, z, body.radius, color);
        addCircle(out, b, x, z, body.radius, color);
        for (Vec3 radial : {x, -x, z, -z})
            addLine(out, a + radial * body.radius, b + radial * body.radius, color);
        break;
    }
    case ShapeType::Cylinder: {
        Vec3 a = body.position - y * body.capsuleHalfHeight;
        Vec3 b = body.position + y * body.capsuleHalfHeight;
        addCircle(out, a, x, z, body.radius, color);
        addCircle(out, b, x, z, body.radius, color);
        for (Vec3 radial : {x, -x, z, -z})
            addLine(out, a + radial * body.radius, b + radial * body.radius, color);
        break;
    }
    case ShapeType::Cone: {
        Vec3 base = body.position - y * (0.5f * body.capsuleHalfHeight);
        Vec3 tip = body.position + y * (1.5f * body.capsuleHalfHeight);
        addCircle(out, base, x, z, body.radius, color);
        for (Vec3 radial : {x, -x, z, -z})
            addLine(out, base + radial * body.radius, tip, color);
        break;
    }
    case ShapeType::Hull:
        for (uint32_t i = 0; i < body.hullCount; ++i)
            addPointCross(out, body.position + rotate(
                body.orientation, soup.hullPoints[body.hullFirst + i]), color);
        break;
    case ShapeType::Plane: {
        Vec3 n = body.planeNormal;
        Vec3 ref = std::fabs(n.x) < 0.9f ? Vec3{1,0,0} : Vec3{0,1,0};
        Vec3 u = normalize(cross(n, ref)), v = cross(n, u);
        Vec3 center = n * body.planeOffset;
        addLine(out, center - u * 5.0f, center + u * 5.0f, color);
        addLine(out, center - v * 5.0f, center + v * 5.0f, color);
        addLine(out, center, center + n, color);
        break;
    }
    case ShapeType::Mesh: {
        const Mesh& mesh = soup.meshes[body.meshIndex];
        for (uint32_t i = 0; i < mesh.indexCount; i += 3) {
            uint32_t base = mesh.firstIndex + i;
            Vec3 a = soup.vertices[mesh.firstVertex + soup.indices[base]];
            Vec3 b = soup.vertices[mesh.firstVertex + soup.indices[base + 1]];
            Vec3 c = soup.vertices[mesh.firstVertex + soup.indices[base + 2]];
            addLine(out, a, b, color); addLine(out, b, c, color);
            addLine(out, c, a, color);
        }
        break;
    }
    default: break;
    }
}

} // namespace

void World::debugLines(std::vector<DebugLine>& out, uint32_t flags) const {
    out.clear();
    MeshSoupView soup = view(meshes_);
    if (flags & DebugDrawShapes)
        for (const Body& body : bodies_)
            addShape(out, body, soup,
                     body.isStatic() ? kStaticColor : kShapeColor);
    if (flags & DebugDrawAabbs) {
        for (const Body& body : bodies_) {
            if (body.shape == ShapeType::Plane) continue;
            if (body.shape == ShapeType::Mesh) {
                const Mesh& mesh = meshes_.meshes[body.meshIndex];
                addAabb(out, mesh.aabbMin, mesh.aabbMax);
            } else {
                Vec3 lo, hi;
                bodyAabb(body, 0.0f, lo, hi);
                addAabb(out, lo, hi);
            }
        }
    }
    if (flags & DebugDrawContacts)
        for (const Contact& contact : contacts_) {
            addPointCross(out, contact.point, kContactColor);
            addLine(out, contact.point,
                    contact.point + contact.normal * 0.35f, kContactColor);
        }
    if (flags & DebugDrawJoints)
        for (const Joint& joint : joints_) {
            const Body& a = bodies_[joint.a];
            const Body& b = bodies_[joint.b];
            Vec3 pa = a.position + rotate(a.orientation, joint.localAnchorA);
            Vec3 pb = b.position + rotate(b.orientation, joint.localAnchorB);
            addLine(out, pa, pb, kJointColor);
            addPointCross(out, pa, kJointColor);
            addPointCross(out, pb, kJointColor);
            if (joint.type == JointType::Hinge ||
                joint.type == JointType::ConeTwist ||
                joint.type == JointType::Prismatic) {
                Vec3 axis = normalize(rotate(a.orientation, joint.localAxisA));
                addLine(out, pa - axis * 0.5f, pa + axis * 0.5f, kJointColor);
            }
        }
}

} // namespace velox
