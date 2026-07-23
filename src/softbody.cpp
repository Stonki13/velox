#include "velox/softbody.h"
#include "velox/body.h"
#include <cmath>
#include <algorithm>

namespace velox {

// ---------------------------------------------------------------------------
// Factory: cloth grid
// ---------------------------------------------------------------------------

SoftBodyDesc makeClothSoftBody(int cols, int rows, float spacing,
                               float invMass, float compliance,
                               bool pinCorners) {
    SoftBodyDesc desc;

    int n = cols * rows;
    desc.positions.resize(n);
    desc.invMasses.resize(n, invMass);

    float halfW = (cols - 1) * spacing * 0.5f;
    float halfH = (rows - 1) * spacing * 0.5f;
    for (int r = 0; r < rows; ++r)
        for (int c = 0; c < cols; ++c) {
            int i = r * cols + c;
            desc.positions[i] = {c * spacing - halfW, 0.0f, r * spacing - halfH};
        }

    if (pinCorners) {
        desc.invMasses[0] = 0.0f;
        desc.invMasses[cols - 1] = 0.0f;
        desc.invMasses[(rows - 1) * cols] = 0.0f;
        desc.invMasses[(rows - 1) * cols + cols - 1] = 0.0f;
    }

    auto addConstraint = [&](int a, int b) {
        float len = length(desc.positions[a] - desc.positions[b]);
        desc.constraints.push_back({uint32_t(a), uint32_t(b), len, compliance});
    };

    for (int r = 0; r < rows; ++r)
        for (int c = 0; c < cols; ++c) {
            int i = r * cols + c;
            if (c + 1 < cols) addConstraint(i, i + 1);           // structural X
            if (r + 1 < rows) addConstraint(i, i + cols);         // structural Z
            if (c + 1 < cols && r + 1 < rows) {
                addConstraint(i, i + cols + 1);                   // shear
                addConstraint(i + 1, i + cols);                   // shear
            }
        }

    return desc;
}

// ---------------------------------------------------------------------------
// Factory: soft sphere (Fibonacci lattice + core)
// ---------------------------------------------------------------------------

SoftBodyDesc makeSoftSphereSoftBody(float radius, int surfaceCount,
                                    float invMass, float compliance) {
    SoftBodyDesc desc;

    int n = surfaceCount + 1; // +1 for core
    desc.positions.resize(n);
    desc.invMasses.resize(n, invMass);

    // Core particle at the center.
    int core = surfaceCount;
    desc.positions[core] = {0, 0, 0};

    // Fibonacci lattice on the sphere surface.
    const float goldenAngle = 3.14159265f * (3.0f - std::sqrt(5.0f));
    for (int i = 0; i < surfaceCount; ++i) {
        float y = 1.0f - 2.0f * (i + 0.5f) / surfaceCount;
        float r = std::sqrt(std::fmax(0.0f, 1.0f - y * y));
        float theta = goldenAngle * i;
        desc.positions[i] = {radius * r * std::cos(theta),
                             radius * y,
                             radius * r * std::sin(theta)};
    }

    // Core-to-surface constraints (volume preservation).
    for (int i = 0; i < surfaceCount; ++i)
        desc.constraints.push_back({uint32_t(i), uint32_t(core), radius, compliance});

    // Surface-to-surface constraints: connect each particle to its nearest
    // neighbors. A simple O(n²) approach is fine for the small counts used.
    float connectDist = radius * 4.5f / std::sqrt(float(surfaceCount));
    for (int i = 0; i < surfaceCount; ++i)
        for (int j = i + 1; j < surfaceCount; ++j) {
            float d = length(desc.positions[i] - desc.positions[j]);
            if (d < connectDist)
                desc.constraints.push_back({uint32_t(i), uint32_t(j), d, compliance});
        }

    return desc;
}

// ---------------------------------------------------------------------------
// XPBD solver
// ---------------------------------------------------------------------------

namespace softbody_detail {

void solveDistanceConstraint(SoftBody& sb, const SoftDistanceConstraint& c,
                             float dt) {
    float wA = sb.invMasses[c.a];
    float wB = sb.invMasses[c.b];
    float wSum = wA + wB;
    if (wSum <= 0.0f) return;

    Vec3 delta = sb.positions[c.b] - sb.positions[c.a];
    float dist = length(delta);
    if (dist < 1e-10f) return;

    Vec3 dir = delta * (1.0f / dist);
    float C = dist - c.restLength;
    float alphaTilde = c.compliance / (dt * dt);
    float dLambda = -C / (wSum + alphaTilde);

    sb.positions[c.a] -= dir * (dLambda * wA);
    sb.positions[c.b] += dir * (dLambda * wB);
}

void collideParticleWithBody(Vec3& pos, float invMass, const Body& body) {
    if (invMass <= 0.0f) return;

    switch (body.shape) {
    case ShapeType::Plane: {
        float d = dot(pos, body.planeNormal) - body.planeOffset;
        if (d < 0.0f)
            pos -= body.planeNormal * d;
        break;
    }
    case ShapeType::Sphere: {
        Vec3 delta = pos - body.position;
        float dist = length(delta);
        float minDist = body.radius;
        if (dist < minDist && dist > 1e-10f) {
            pos = body.position + delta * (minDist / dist);
        }
        break;
    }
    case ShapeType::Box: {
        // Transform particle into box-local space.
        Vec3 local = rotateInv(body.orientation, pos - body.position);
        Vec3 he = body.halfExtents;
        // Clamp to the box surface if inside.
        bool inside = std::fabs(local.x) < he.x &&
                      std::fabs(local.y) < he.y &&
                      std::fabs(local.z) < he.z;
        if (inside) {
            // Push out along the axis of least penetration.
            float px = he.x - std::fabs(local.x);
            float py = he.y - std::fabs(local.y);
            float pz = he.z - std::fabs(local.z);
            if (px <= py && px <= pz)
                local.x = local.x > 0 ? he.x : -he.x;
            else if (py <= pz)
                local.y = local.y > 0 ? he.y : -he.y;
            else
                local.z = local.z > 0 ? he.z : -he.z;
            pos = body.position + rotate(body.orientation, local);
        }
        break;
    }
    default:
        break;
    }
}

void stepSoftBody(SoftBody& sb, const Vec3& gravity, float dt,
                  const std::vector<Body>& bodies) {
    size_t n = sb.positions.size();
    if (n == 0) return;

    // 1. Save previous positions and predict.
    sb.prevPositions = sb.positions;
    Vec3 g = gravity * sb.gravityScale;
    for (size_t i = 0; i < n; ++i) {
        if (sb.invMasses[i] <= 0.0f) continue;
        sb.velocities[i] *= 1.0f / (1.0f + sb.linearDamping * dt);
        sb.positions[i] += sb.velocities[i] * dt + g * (dt * dt);
    }

    // 2. XPBD constraint iterations.
    for (int iter = 0; iter < sb.solverIterations; ++iter) {
        for (const auto& c : sb.constraints)
            solveDistanceConstraint(sb, c, dt);

        // 3. Collision with rigid bodies.
        for (size_t i = 0; i < n; ++i) {
            if (sb.invMasses[i] <= 0.0f) continue;
            for (const Body& body : bodies) {
                if (body.isDynamic()) continue;
                collideParticleWithBody(sb.positions[i], sb.invMasses[i], body);
            }
        }
    }

    // 4. Update velocities from position change.
    float invDt = 1.0f / dt;
    for (size_t i = 0; i < n; ++i) {
        if (sb.invMasses[i] <= 0.0f) {
            sb.velocities[i] = {};
            continue;
        }
        sb.velocities[i] = (sb.positions[i] - sb.prevPositions[i]) * invDt;
    }

    // 5. Update AABB.
    sb.aabbMin = sb.positions[0];
    sb.aabbMax = sb.positions[0];
    for (size_t i = 1; i < n; ++i) {
        sb.aabbMin = vmin(sb.aabbMin, sb.positions[i]);
        sb.aabbMax = vmax(sb.aabbMax, sb.positions[i]);
    }
}

} // namespace softbody_detail
} // namespace velox
