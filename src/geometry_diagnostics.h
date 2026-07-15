#pragma once

#include "velox/world.h"
#include <cmath>
#include <unordered_map>

namespace velox::geometry_detail {

inline void includeEdge(GeometryDiagnostics& out, float length) {
    if (!std::isfinite(length) || length <= 0.0f) return;
    if (out.minEdgeLength == 0.0f || length < out.minEdgeLength)
        out.minEdgeLength = length;
    out.maxEdgeLength = vmax(out.maxEdgeLength, length);
}

inline uint64_t edgeKey(uint32_t a, uint32_t b) {
    uint32_t lo = a < b ? a : b;
    uint32_t hi = a < b ? b : a;
    return (uint64_t(lo) << 32) | hi;
}

inline GeometryDiagnostics diagnostics(const Body& body, const MeshSoup& soup) {
    constexpr float kPi = 3.14159265358979323846f;
    GeometryDiagnostics out{};
    auto finish = [&]() {
        if (out.minEdgeLength > 0.0f)
            out.aspectRatio = out.maxEdgeLength / out.minEdgeLength;
        else
            out.aspectRatio = 1.0f;
        float scale = out.maxEdgeLength;
        out.isDegenerate = scale <= 0.0f ||
            out.volume < 1e-12f * scale * scale * scale;
    };

    switch (body.shape) {
    case ShapeType::Sphere:
        includeEdge(out, body.radius * 2.0f);
        out.volume = 4.0f * kPi * body.radius * body.radius * body.radius / 3.0f;
        break;
    case ShapeType::Box:
        includeEdge(out, body.halfExtents.x * 2.0f);
        includeEdge(out, body.halfExtents.y * 2.0f);
        includeEdge(out, body.halfExtents.z * 2.0f);
        out.volume = 8.0f * body.halfExtents.x * body.halfExtents.y * body.halfExtents.z;
        break;
    case ShapeType::Capsule: {
        float h = body.capsuleHalfHeight * 2.0f;
        includeEdge(out, body.radius * 2.0f);
        includeEdge(out, h + body.radius * 2.0f);
        out.volume = kPi * body.radius * body.radius * h +
            4.0f * kPi * body.radius * body.radius * body.radius / 3.0f;
        break;
    }
    case ShapeType::Cylinder:
        includeEdge(out, body.radius * 2.0f);
        includeEdge(out, body.capsuleHalfHeight * 2.0f);
        out.volume = kPi * body.radius * body.radius * body.capsuleHalfHeight * 2.0f;
        break;
    case ShapeType::Cone:
        includeEdge(out, body.radius * 2.0f);
        includeEdge(out, body.capsuleHalfHeight * 2.0f);
        out.volume = kPi * body.radius * body.radius * body.capsuleHalfHeight * 2.0f / 3.0f;
        break;
    case ShapeType::Hull: {
        if (body.hullFaceCount == 0 || body.hullFirst + body.hullCount > soup.hullPoints.size() ||
            body.hullFaceFirst + body.hullFaceCount * 3 > soup.hullFaceIndices.size())
            break;
        const Vec3* points = soup.hullPoints.data() + body.hullFirst;
        const uint32_t* indices = soup.hullFaceIndices.data() + body.hullFaceFirst;
        std::unordered_map<uint64_t, Vec3> adjacentNormals;
        double signedVolume = 0.0;
        for (uint32_t face = 0; face < body.hullFaceCount; ++face) {
            uint32_t ia = indices[face * 3], ib = indices[face * 3 + 1], ic = indices[face * 3 + 2];
            if (ia >= body.hullCount || ib >= body.hullCount || ic >= body.hullCount) continue;
            Vec3 a = points[ia], b = points[ib], c = points[ic];
            includeEdge(out, length(b - a));
            includeEdge(out, length(c - b));
            includeEdge(out, length(a - c));
            Vec3 crossProduct = cross(b - a, c - a);
            signedVolume += double(dot(a, crossProduct)) / 6.0;
            float normalLength = length(crossProduct);
            if (normalLength <= 1e-20f) continue;
            Vec3 normal = crossProduct * (1.0f / normalLength);
            const uint32_t edge[3][2] = {{ia, ib}, {ib, ic}, {ic, ia}};
            for (const auto& e : edge) {
                uint64_t key = edgeKey(e[0], e[1]);
                auto found = adjacentNormals.find(key);
                if (found == adjacentNormals.end()) adjacentNormals.emplace(key, normal);
                else if (dot(found->second, normal) >= 0.9999995f)
                    ++out.nearCoplanarFaceCount;
            }
        }
        out.volume = float(std::fabs(signedVolume));
        break;
    }
    case ShapeType::Compound:
        includeEdge(out, body.radius * 2.0f);
        // Compound children can overlap. The bounding sphere is the only
        // meaningful whole-body volume without performing a boolean union.
        out.volume = 4.0f * kPi * body.radius * body.radius * body.radius / 3.0f;
        break;
    case ShapeType::Plane:
    case ShapeType::Mesh:
        break;
    }
    finish();
    return out;
}

} // namespace velox::geometry_detail
