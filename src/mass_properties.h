#pragma once

#include "velox/math.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace velox::mass_properties {

struct Matrix3 {
    double m[3][3]{};
};

struct ConvexMassProperties {
    double volume = 0.0;
    Vec3 center;
    Vec3 principalInertia; // Unit-density principal moments.
    Quat principalOrientation;
    std::vector<std::array<uint32_t, 3>> triangles;
};

struct Plane {
    Vec3 normal;
    float offset = 0.0f;
};

struct ProjectedPoint {
    float x, y;
    uint32_t index;
};

inline float cross2(const ProjectedPoint& a, const ProjectedPoint& b,
                    const ProjectedPoint& c) {
    return (b.x - a.x) * (c.y - a.y) -
           (b.y - a.y) * (c.x - a.x);
}

inline std::vector<uint32_t> planeHull(const std::vector<Vec3>& points,
                                       const Plane& plane, float epsilon) {
    Vec3 reference = std::fabs(plane.normal.x) < 0.8f
        ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
    Vec3 u = normalize(cross(plane.normal, reference));
    Vec3 v = cross(plane.normal, u);
    std::vector<ProjectedPoint> projected;
    for (uint32_t i = 0; i < points.size(); ++i) {
        if (std::fabs(dot(plane.normal, points[i]) - plane.offset) <= epsilon)
            projected.push_back({dot(points[i], u), dot(points[i], v), i});
    }
    std::sort(projected.begin(), projected.end(),
              [](const ProjectedPoint& a, const ProjectedPoint& b) {
                  if (a.x != b.x) return a.x < b.x;
                  if (a.y != b.y) return a.y < b.y;
                  return a.index < b.index;
              });
    std::vector<ProjectedPoint> hull;
    for (const ProjectedPoint& point : projected) {
        while (hull.size() >= 2 &&
               cross2(hull[hull.size() - 2], hull.back(), point) <= epsilon)
            hull.pop_back();
        hull.push_back(point);
    }
    size_t lower = hull.size();
    for (size_t i = projected.size(); i-- > 0;) {
        const ProjectedPoint& point = projected[i];
        while (hull.size() > lower &&
               cross2(hull[hull.size() - 2], hull.back(), point) <= epsilon)
            hull.pop_back();
        hull.push_back(point);
    }
    if (!hull.empty()) hull.pop_back();
    std::vector<uint32_t> result;
    result.reserve(hull.size());
    for (const ProjectedPoint& point : hull) result.push_back(point.index);
    return result;
}

inline std::vector<std::array<uint32_t, 3>> convexTriangles(
    const std::vector<Vec3>& points) {
    Vec3 lo = points.front(), hi = lo;
    for (const Vec3& point : points) {
        lo = vmin(lo, point);
        hi = vmax(hi, point);
    }
    float scale = vmax(1.0f, length(hi - lo));
    float epsilon = scale * 1e-5f;
    std::vector<Plane> planes;
    for (uint32_t i = 0; i + 2 < points.size(); ++i) {
        for (uint32_t j = i + 1; j + 1 < points.size(); ++j) {
            for (uint32_t k = j + 1; k < points.size(); ++k) {
                Vec3 n = cross(points[j] - points[i], points[k] - points[i]);
                if (lengthSq(n) <= epsilon * epsilon) continue;
                n = normalize(n);
                float d = dot(n, points[i]);
                bool front = false, back = false;
                for (const Vec3& point : points) {
                    float distance = dot(n, point) - d;
                    front |= distance > epsilon;
                    back |= distance < -epsilon;
                }
                if (front && back) continue;
                if (front) { n = -n; d = -d; }
                bool duplicate = false;
                for (const Plane& plane : planes) {
                    if (dot(n, plane.normal) > 1.0f - 1e-4f &&
                        std::fabs(d - plane.offset) <= 2.0f * epsilon) {
                        duplicate = true;
                        break;
                    }
                }
                if (!duplicate) planes.push_back({n, d});
            }
        }
    }
    std::vector<std::array<uint32_t, 3>> triangles;
    for (const Plane& plane : planes) {
        std::vector<uint32_t> polygon = planeHull(points, plane, epsilon * 2.0f);
        for (size_t i = 1; i + 1 < polygon.size(); ++i)
            triangles.push_back({polygon[0], polygon[i], polygon[i + 1]});
    }
    return triangles;
}

inline void jacobiDiagonalize(Matrix3 tensor, Vec3& moments, Quat& orientation) {
    Matrix3 vectors;
    vectors.m[0][0] = vectors.m[1][1] = vectors.m[2][2] = 1.0;
    for (int iteration = 0; iteration < 16; ++iteration) {
        int p = 0, q = 1;
        if (std::fabs(tensor.m[0][2]) > std::fabs(tensor.m[p][q])) { p = 0; q = 2; }
        if (std::fabs(tensor.m[1][2]) > std::fabs(tensor.m[p][q])) { p = 1; q = 2; }
        if (std::fabs(tensor.m[p][q]) < 1e-12) break;
        double angle = 0.5 * std::atan2(2.0 * tensor.m[p][q],
                                       tensor.m[q][q] - tensor.m[p][p]);
        double c = std::cos(angle), s = std::sin(angle);
        for (int r = 0; r < 3; ++r) {
            if (r == p || r == q) continue;
            double arp = tensor.m[r][p], arq = tensor.m[r][q];
            tensor.m[r][p] = tensor.m[p][r] = c * arp - s * arq;
            tensor.m[r][q] = tensor.m[q][r] = s * arp + c * arq;
        }
        double app = tensor.m[p][p], aqq = tensor.m[q][q];
        double apq = tensor.m[p][q];
        tensor.m[p][p] = c * c * app - 2.0 * s * c * apq + s * s * aqq;
        tensor.m[q][q] = s * s * app + 2.0 * s * c * apq + c * c * aqq;
        tensor.m[p][q] = tensor.m[q][p] = 0.0;
        for (int r = 0; r < 3; ++r) {
            double vrp = vectors.m[r][p], vrq = vectors.m[r][q];
            vectors.m[r][p] = c * vrp - s * vrq;
            vectors.m[r][q] = s * vrp + c * vrq;
        }
    }
    moments = {float(tensor.m[0][0]), float(tensor.m[1][1]),
               float(tensor.m[2][2])};
    float m00 = float(vectors.m[0][0]), m01 = float(vectors.m[0][1]);
    float m02 = float(vectors.m[0][2]), m10 = float(vectors.m[1][0]);
    float m11 = float(vectors.m[1][1]), m12 = float(vectors.m[1][2]);
    float m20 = float(vectors.m[2][0]), m21 = float(vectors.m[2][1]);
    float m22 = float(vectors.m[2][2]);
    float trace = m00 + m11 + m22;
    if (trace > 0.0f) {
        float s = std::sqrt(trace + 1.0f) * 2.0f;
        orientation = {(m21 - m12) / s, (m02 - m20) / s,
                       (m10 - m01) / s, 0.25f * s};
    } else if (m00 > m11 && m00 > m22) {
        float s = std::sqrt(1.0f + m00 - m11 - m22) * 2.0f;
        orientation = {0.25f * s, (m01 + m10) / s, (m02 + m20) / s,
                       (m21 - m12) / s};
    } else if (m11 > m22) {
        float s = std::sqrt(1.0f + m11 - m00 - m22) * 2.0f;
        orientation = {(m01 + m10) / s, 0.25f * s, (m12 + m21) / s,
                       (m02 - m20) / s};
    } else {
        float s = std::sqrt(1.0f + m22 - m00 - m11) * 2.0f;
        orientation = {(m02 + m20) / s, (m12 + m21) / s, 0.25f * s,
                       (m10 - m01) / s};
    }
    orientation = normalize(orientation);
}

inline Matrix3 rotatedPrincipalTensor(const Vec3& moments,
                                      const Quat& orientation) {
    Vec3 axes[3] = {rotate(orientation, {1, 0, 0}),
                    rotate(orientation, {0, 1, 0}),
                    rotate(orientation, {0, 0, 1})};
    double components[3][3] = {
        {axes[0].x, axes[0].y, axes[0].z},
        {axes[1].x, axes[1].y, axes[1].z},
        {axes[2].x, axes[2].y, axes[2].z}};
    double values[3] = {moments.x, moments.y, moments.z};
    Matrix3 result;
    for (int row = 0; row < 3; ++row) for (int column = 0; column < 3; ++column) {
        double value = 0.0;
        for (int axis = 0; axis < 3; ++axis)
            value += values[axis] * components[axis][row] *
                     components[axis][column];
        result.m[row][column] = value;
    }
    return result;
}

inline void addAtOffset(Matrix3& aggregate, const Matrix3& centered,
                        double childMass, const Vec3& offset) {
    double p[3] = {offset.x, offset.y, offset.z};
    double lengthSquared = p[0] * p[0] + p[1] * p[1] + p[2] * p[2];
    for (int row = 0; row < 3; ++row) for (int column = 0; column < 3; ++column) {
        double parallel = childMass *
            ((row == column ? lengthSquared : 0.0) - p[row] * p[column]);
        aggregate.m[row][column] += centered.m[row][column] + parallel;
    }
}

inline Matrix3 shiftedToCenter(const Matrix3& origin, double totalMass,
                               const Vec3& center) {
    Matrix3 result = origin;
    double p[3] = {center.x, center.y, center.z};
    double lengthSquared = p[0] * p[0] + p[1] * p[1] + p[2] * p[2];
    for (int row = 0; row < 3; ++row) for (int column = 0; column < 3; ++column)
        result.m[row][column] -= totalMass *
            ((row == column ? lengthSquared : 0.0) - p[row] * p[column]);
    return result;
}

inline ConvexMassProperties convex(const std::vector<Vec3>& points) {
    ConvexMassProperties result;
    result.triangles = convexTriangles(points);
    Matrix3 second{};
    double first[3]{};
    for (const auto& triangle : result.triangles) {
        const Vec3& a = points[triangle[0]];
        const Vec3& b = points[triangle[1]];
        const Vec3& c = points[triangle[2]];
        double volume = double(dot(a, cross(b, c))) / 6.0;
        result.volume += volume;
        const double p[3][3] = {{a.x, a.y, a.z}, {b.x, b.y, b.z},
                                {c.x, c.y, c.z}};
        for (int axis = 0; axis < 3; ++axis)
            first[axis] += volume * (p[0][axis] + p[1][axis] + p[2][axis]) / 4.0;
        for (int x = 0; x < 3; ++x) for (int y = 0; y < 3; ++y) {
            double integral = 0.0;
            for (int i = 0; i < 3; ++i) integral += 2.0 * p[i][x] * p[i][y];
            for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j)
                if (i != j) integral += p[i][x] * p[j][y];
            second.m[x][y] += volume * integral / 20.0;
        }
    }
    if (result.volume <= 1e-12)
        throw std::invalid_argument("velox: convex hull has no positive volume");
    double center[3] = {first[0] / result.volume, first[1] / result.volume,
                        first[2] / result.volume};
    result.center = {float(center[0]), float(center[1]), float(center[2])};
    Matrix3 inertia;
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) {
        double origin = i == j
            ? second.m[(i + 1) % 3][(i + 1) % 3] +
              second.m[(i + 2) % 3][(i + 2) % 3]
            : -second.m[i][j];
        double shift = result.volume *
            ((i == j ? center[0] * center[0] + center[1] * center[1] +
                            center[2] * center[2] : 0.0) - center[i] * center[j]);
        inertia.m[i][j] = origin - shift;
    }
    jacobiDiagonalize(inertia, result.principalInertia,
                      result.principalOrientation);
    return result;
}

} // namespace velox::mass_properties
