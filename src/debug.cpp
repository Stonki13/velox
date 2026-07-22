#include "velox/world.h"
#include "velox/sleep.h"
#include "narrowphase.h"
#include <cmath>

namespace velox {
namespace {

constexpr uint32_t kShapeColor = 0x58c4ddffu;
constexpr uint32_t kStaticColor = 0x8b95a5ffu;
constexpr uint32_t kAabbColor = 0xf2c14effu;
constexpr uint32_t kContactColor = 0xf45b69ffu;
constexpr uint32_t kJointColor = 0x7bd389ffu;
constexpr uint32_t kAxisXColor = 0xf45b69ffu;
constexpr uint32_t kAxisYColor = 0x7bd389ffu;
constexpr uint32_t kAxisZColor = 0x58c4ddffu;
constexpr uint32_t kVelocityColor = 0x35d0baffu;  // linear velocity
constexpr uint32_t kAngularVelocityColor = 0xffa94du; // angular velocity
constexpr uint32_t kForceColor = 0xff8787u;       // accumulated force
constexpr uint32_t kTorqueColor = 0xcc5de8u;      // accumulated torque
constexpr uint32_t kSleepAwakeColor = 0x58c4ddffu;  // awake: default shape color
constexpr uint32_t kSleepDrowsyColor = 0xffd43bu;   // drowsy: amber/yellow
constexpr uint32_t kSleepAsleepColor = 0x4950ceffu;  // asleep: muted blue
constexpr float kTau = 6.28318530718f;
constexpr float kVelocityScale = 0.10f; // seconds of motion drawn as an arrow
constexpr float kForceScale = 1e-3f;    // arrow length per newton (visual only)

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

// A line capped with a small four-spoke arrowhead so direction reads at a glance.
void addArrow(std::vector<DebugLine>& out, Vec3 from, Vec3 to, uint32_t color) {
    addLine(out, from, to, color);
    Vec3 dir = to - from;
    float len = length(dir);
    if (len < 1e-6f) return;
    dir = dir * (1.0f / len);
    Vec3 ref = std::fabs(dir.x) < 0.9f ? Vec3{1,0,0} : Vec3{0,1,0};
    Vec3 u = normalize(cross(dir, ref));
    Vec3 v = cross(dir, u);
    float head = vclamp(len * 0.25f, 0.03f, 0.2f);
    Vec3 base = to - dir * head;
    addLine(out, to, base + u * head * 0.5f, color);
    addLine(out, to, base - u * head * 0.5f, color);
    addLine(out, to, base + v * head * 0.5f, color);
    addLine(out, to, base - v * head * 0.5f, color);
}

// Body-frame axes (X red, Y green, Z blue); used for centers of mass.
void addAxes(std::vector<DebugLine>& out, Vec3 center, Quat orientation, float size) {
    addLine(out, center, center + rotate(orientation, {size, 0, 0}), kAxisXColor);
    addLine(out, center, center + rotate(orientation, {0, size, 0}), kAxisYColor);
    addLine(out, center, center + rotate(orientation, {0, 0, size}), kAxisZColor);
}

// Angular velocity drawn as an axis arrow plus a partial circle whose sweep
// grows with spin rate, so the sense of rotation is visible.
void addAngularVelocity(std::vector<DebugLine>& out, Vec3 center, Vec3 omega) {
    float rate = length(omega);
    if (rate < 1e-5f) return;
    Vec3 axis = omega * (1.0f / rate);
    addArrow(out, center, center + axis * (rate * kVelocityScale), kAngularVelocityColor);
    Vec3 ref = std::fabs(axis.x) < 0.9f ? Vec3{1,0,0} : Vec3{0,1,0};
    Vec3 u = normalize(cross(axis, ref));
    Vec3 v = cross(axis, u);
    constexpr float radius = 0.2f;
    constexpr int segments = 12;
    float sweep = vclamp(rate * kVelocityScale, 0.4f, kTau * 0.75f);
    Vec3 previous = center + u * radius;
    for (int i = 1; i <= segments; ++i) {
        float angle = sweep * float(i) / float(segments);
        Vec3 point = center + (u * cosf(angle) + v * sinf(angle)) * radius;
        addLine(out, previous, point, kAngularVelocityColor);
        previous = point;
    }
}

// Golden-ratio hue spacing yields visually distinct, stable island colors.
uint32_t islandPalette(uint32_t index) {
    float hue = float(index) * 0.61803398875f;
    hue -= floorf(hue);
    float h = hue * 6.0f;
    int sector = int(h);
    float f = h - float(sector);
    float r, g, b;
    switch (sector % 6) {
    case 0: r = 1.0f; g = f;     b = 0.0f;   break;
    case 1: r = 1.0f - f; g = 1.0f; b = 0.0f; break;
    case 2: r = 0.0f; g = 1.0f;  b = f;       break;
    case 3: r = 0.0f; g = 1.0f - f; b = 1.0f; break;
    case 4: r = f;     g = 0.0f;  b = 1.0f;   break;
    default: r = 1.0f; g = 0.0f;  b = 1.0f - f; break;
    }
    auto toByte = [](float value) { return uint32_t(value * 255.0f + 0.5f); };
    return (toByte(r) << 24) | (toByte(g) << 16) | (toByte(b) << 8) | 0xffu;
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
    case ShapeType::RoundedBox:
        addBox(out, body.position, body.orientation, body.halfExtents, color);
        break;
    case ShapeType::Ellipsoid: {
        addCircle(out, body.position, x, y, vmax(body.halfExtents.x, body.halfExtents.y), color);
        addCircle(out, body.position, x, z, vmax(body.halfExtents.x, body.halfExtents.z), color);
        addCircle(out, body.position, y, z, vmax(body.halfExtents.y, body.halfExtents.z), color);
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
    AccessGuard guard(*this, AccessKind::Query, "debugLines");
    out.clear();
    MeshSoupView soup = view(meshes_);

    // Optional island coloring: union-find over dynamic-dynamic contacts and
    // joints, mirroring the solver's island construction. Recomputed here so
    // the visualization is valid even before the first step() populates the
    // internal sleeping islands. islandColor[i] == 0 means "no island tint".
    std::vector<uint32_t> islandColor;
    if (flags & DebugDrawIslands) {
        const uint32_t n = uint32_t(bodies_.size());
        std::vector<uint32_t> parent(n);
        for (uint32_t i = 0; i < n; ++i) parent[i] = i;
        auto find = [&](uint32_t x) {
            while (parent[x] != x) {
                parent[x] = parent[parent[x]];
                x = parent[x];
            }
            return x;
        };
        auto unite = [&](uint32_t x, uint32_t y) {
            x = find(x); y = find(y);
            if (x != y) parent[x] = y;
        };
        for (const Contact& c : contacts_)
            if (bodies_[c.a].isDynamic() && bodies_[c.b].isDynamic()) unite(c.a, c.b);
        for (const Joint& j : joints_)
            if (bodies_[j.a].isDynamic() && bodies_[j.b].isDynamic()) unite(j.a, j.b);
        std::vector<uint32_t> rootColor(n, UINT32_MAX);
        uint32_t nextColor = 0;
        islandColor.assign(n, 0);
        for (uint32_t i = 0; i < n; ++i) {
            if (!bodies_[i].isDynamic()) continue;
            uint32_t root = find(i);
            if (rootColor[root] == UINT32_MAX) rootColor[root] = nextColor++;
            islandColor[i] = islandPalette(rootColor[root]);
        }
    }

    if (flags & DebugDrawShapes)
        for (BodyIndex i = 0; i < bodies_.size(); ++i) {
            const Body& body = bodies_[i];
            uint32_t color = body.isStatic() ? kStaticColor : kShapeColor;
            if ((flags & DebugDrawIslands) && body.isDynamic() && islandColor[i])
                color = islandColor[i];
            addShape(out, body, soup, color);
        }
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
            if (joint.type == JointType::SixDof) {
                Vec3 x = normalize(rotate(a.orientation, joint.localAxisA));
                Vec3 y = normalize(rotate(a.orientation, joint.localRefA));
                Vec3 z = normalize(cross(x, y));
                addLine(out, pa, pa + x * 0.5f, kAxisXColor);
                addLine(out, pa, pa + y * 0.5f, kAxisYColor);
                addLine(out, pa, pa + z * 0.5f, kAxisZColor);
            } else if (joint.type == JointType::Hinge ||
                joint.type == JointType::ConeTwist ||
                joint.type == JointType::Prismatic) {
                Vec3 axis = normalize(rotate(a.orientation, joint.localAxisA));
                addLine(out, pa - axis * 0.5f, pa + axis * 0.5f, kJointColor);
            }
        }
    if (flags & DebugDrawVelocityVectors)
        for (const Body& body : bodies_) {
            if (body.isStatic()) continue;
            if (lengthSq(body.velocity) > 1e-8f)
                addArrow(out, body.position,
                         body.position + body.velocity * kVelocityScale,
                         kVelocityColor);
            addAngularVelocity(out, body.position, body.angularVelocity);
        }
    // Accumulated force/torque are cleared inside step(); draw these between
    // addForce()/addTorque() and the next step() to see pending inputs.
    if (flags & DebugDrawForceVectors)
        for (const Body& body : bodies_) {
            if (body.isStatic()) continue;
            if (lengthSq(body.force) > 1e-8f)
                addArrow(out, body.position,
                         body.position + body.force * kForceScale, kForceColor);
            if (lengthSq(body.torque) > 1e-8f)
                addArrow(out, body.position,
                         body.position + body.torque * kForceScale, kTorqueColor);
        }
    if (flags & DebugDrawCenterOfMass)
        for (const Body& body : bodies_) {
            // Unbounded statics (plane/mesh) have no meaningful center of mass.
            if (body.shape == ShapeType::Plane || body.shape == ShapeType::Mesh)
                continue;
            addAxes(out, body.position, body.orientation, 0.15f);
        }
    // Sleep state visualization: tint shapes by sleep state so sleeping and
    // drowsy bodies are immediately distinguishable from awake ones.
    if (flags & DebugDrawSleep)
        for (BodyIndex i = 0; i < bodies_.size(); ++i) {
            const Body& body = bodies_[i];
            if (!body.isDynamic()) continue;
            uint32_t color;
            SleepState state = sleepStateFromByte(body.asleep);
            switch (state) {
            case SleepState::Asleep: color = kSleepAsleepColor; break;
            case SleepState::Drowsy: color = kSleepDrowsyColor; break;
            default:                 color = kSleepAwakeColor;  break;
            }
            addShape(out, body, soup, color);
            // Draw a small "Z" marker above sleeping bodies for clarity.
            if (state == SleepState::Asleep) {
                Vec3 top = body.position + Vec3{0, body.radius + 0.15f, 0};
                constexpr float s = 0.06f;
                addLine(out, top + Vec3{-s, s, 0}, top + Vec3{s, s, 0}, color);
                addLine(out, top + Vec3{s, s, 0}, top + Vec3{-s, -s, 0}, color);
                addLine(out, top + Vec3{-s, -s, 0}, top + Vec3{s, -s, 0}, color);
            }
        }
}

} // namespace velox
