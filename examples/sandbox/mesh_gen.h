#pragma once
// Procedural unit meshes for the solid render pass. All generators produce
// positions+normals in local space; instances scale them per body.
#include <array>
#include <cmath>
#include <cstdint>
#include <map>
#include <vector>

namespace sandbox {

struct MeshVertex {
    float px, py, pz;
    float nx, ny, nz;
};

struct CpuMesh {
    std::vector<MeshVertex> vertices;
    std::vector<uint32_t> indices;
};

inline void addTriangle(CpuMesh& mesh, uint32_t a, uint32_t b, uint32_t c) {
    mesh.indices.push_back(a);
    mesh.indices.push_back(b);
    mesh.indices.push_back(c);
}

// Unit cube: half extent 1 on every axis (instances scale by body half extents).
inline CpuMesh makeCube() {
    CpuMesh mesh;
    const float n[6][3] = {{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
    const int axisU[6] = {2, 2, 0, 0, 0, 0};
    const int axisV[6] = {1, 1, 2, 2, 1, 1};
    for (int face = 0; face < 6; ++face) {
        const uint32_t base = static_cast<uint32_t>(mesh.vertices.size());
        for (int corner = 0; corner < 4; ++corner) {
            const float u = (corner == 1 || corner == 2) ? 1.0f : -1.0f;
            const float v = (corner >= 2) ? 1.0f : -1.0f;
            float p[3] = {n[face][0], n[face][1], n[face][2]};
            p[axisU[face]] = u;
            p[axisV[face]] = v;
            mesh.vertices.push_back({p[0], p[1], p[2],
                                     n[face][0], n[face][1], n[face][2]});
        }
        const bool flip = (n[face][0] + n[face][1] + n[face][2]) < 0.0f;
        if (!flip) {
            addTriangle(mesh, base, base + 1, base + 2);
            addTriangle(mesh, base, base + 2, base + 3);
        } else {
            addTriangle(mesh, base, base + 2, base + 1);
            addTriangle(mesh, base, base + 3, base + 2);
        }
    }
    return mesh;
}

// Unit icosphere, 2 subdivisions (radius 1).
inline CpuMesh makeIcosphere(int subdivisions = 2) {
    const float t = (1.0f + std::sqrt(5.0f)) * 0.5f;
    std::vector<std::array<float, 3>> positions = {
        {-1,  t,  0}, { 1,  t,  0}, {-1, -t,  0}, { 1, -t,  0},
        { 0, -1,  t}, { 0,  1,  t}, { 0, -1, -t}, { 0,  1, -t},
        { t,  0, -1}, { t,  0,  1}, {-t,  0, -1}, {-t,  0,  1}};
    for (auto& p : positions) {
        const float inv = 1.0f / std::sqrt(p[0]*p[0] + p[1]*p[1] + p[2]*p[2]);
        p = {p[0]*inv, p[1]*inv, p[2]*inv};
    }
    std::vector<std::array<uint32_t, 3>> faces = {
        {0,11,5},{0,5,1},{0,1,7},{0,7,10},{0,10,11},
        {1,5,9},{5,11,4},{11,10,2},{10,7,6},{7,1,8},
        {3,9,4},{3,4,2},{3,2,6},{3,6,8},{3,8,9},
        {4,9,5},{2,4,11},{6,2,10},{8,6,7},{9,8,1}};

    for (int pass = 0; pass < subdivisions; ++pass) {
        std::map<uint64_t, uint32_t> midpointCache;
        auto midpoint = [&](uint32_t a, uint32_t b) {
            const uint64_t key = a < b ? (uint64_t(a) << 32) | b : (uint64_t(b) << 32) | a;
            auto found = midpointCache.find(key);
            if (found != midpointCache.end()) return found->second;
            std::array<float, 3> m = {
                (positions[a][0] + positions[b][0]) * 0.5f,
                (positions[a][1] + positions[b][1]) * 0.5f,
                (positions[a][2] + positions[b][2]) * 0.5f};
            const float inv = 1.0f / std::sqrt(m[0]*m[0] + m[1]*m[1] + m[2]*m[2]);
            m = {m[0]*inv, m[1]*inv, m[2]*inv};
            positions.push_back(m);
            const uint32_t index = static_cast<uint32_t>(positions.size() - 1);
            midpointCache[key] = index;
            return index;
        };
        std::vector<std::array<uint32_t, 3>> next;
        next.reserve(faces.size() * 4);
        for (const auto& f : faces) {
            const uint32_t ab = midpoint(f[0], f[1]);
            const uint32_t bc = midpoint(f[1], f[2]);
            const uint32_t ca = midpoint(f[2], f[0]);
            next.push_back({f[0], ab, ca});
            next.push_back({f[1], bc, ab});
            next.push_back({f[2], ca, bc});
            next.push_back({ab, bc, ca});
        }
        faces.swap(next);
    }

    CpuMesh mesh;
    mesh.vertices.reserve(positions.size());
    for (const auto& p : positions)
        mesh.vertices.push_back({p[0], p[1], p[2], p[0], p[1], p[2]});
    for (const auto& f : faces) addTriangle(mesh, f[0], f[1], f[2]);
    return mesh;
}

// Capsule parameterized by radius/halfHeight ratio: a single affine instance
// scale cannot keep hemisphere caps spherical while stretching the cylinder,
// so the renderer caches one mesh per ratio (rounded), scaled by halfHeight.
inline CpuMesh makeCapsule(float radiusOverHalfHeight, int segments = 24, int rings = 8) {
    // Local space: total capsule = cylinder half height 1 plus caps of radius
    // radiusOverHalfHeight (r expressed in units of the cylinder half height).
    const float r = radiusOverHalfHeight;
    CpuMesh mesh;
    const float pi = 3.14159265358979323846f;
    // Side rings: two rings (y = -1 and y = +1) with outward normals.
    for (int ring = 0; ring <= 1; ++ring) {
        const float y = ring == 0 ? -1.0f : 1.0f;
        for (int s = 0; s <= segments; ++s) {
            const float a = 2.0f * pi * static_cast<float>(s) / segments;
            const float cx = std::cos(a), cz = std::sin(a);
            mesh.vertices.push_back({cx * r, y, cz * r, cx, 0.0f, cz});
        }
    }
    for (int s = 0; s < segments; ++s) {
        const uint32_t a = static_cast<uint32_t>(s);
        const uint32_t b = static_cast<uint32_t>(s + 1);
        const uint32_t c = static_cast<uint32_t>(segments + 1 + s);
        const uint32_t d = static_cast<uint32_t>(segments + 1 + s + 1);
        addTriangle(mesh, a, c, b);
        addTriangle(mesh, b, c, d);
    }
    // Hemisphere caps.
    for (int cap = 0; cap < 2; ++cap) {
        const float sign = cap == 0 ? 1.0f : -1.0f;
        const uint32_t base = static_cast<uint32_t>(mesh.vertices.size());
        for (int ring = 0; ring <= rings; ++ring) {
            const float phi = 0.5f * pi * static_cast<float>(ring) / rings;
            const float y = std::sin(phi), radial = std::cos(phi);
            for (int s = 0; s <= segments; ++s) {
                const float a = 2.0f * pi * static_cast<float>(s) / segments;
                const float cx = std::cos(a) * radial, cz = std::sin(a) * radial;
                mesh.vertices.push_back({cx * r, sign * (1.0f + y * r), cz * r,
                                         cx, sign * y, cz});
            }
        }
        for (int ring = 0; ring < rings; ++ring)
            for (int s = 0; s < segments; ++s) {
                const uint32_t a = base + ring * (segments + 1) + s;
                const uint32_t b = a + 1;
                const uint32_t c = a + (segments + 1);
                const uint32_t d = c + 1;
                if (sign > 0.0f) {
                    addTriangle(mesh, a, b, c);
                    addTriangle(mesh, b, d, c);
                } else {
                    addTriangle(mesh, a, c, b);
                    addTriangle(mesh, b, c, d);
                }
            }
    }
    return mesh;
}

// Unit cylinder: radius 1, half height 1.
inline CpuMesh makeCylinder(int segments = 28) {
    CpuMesh mesh;
    const float pi = 3.14159265358979323846f;
    for (int ring = 0; ring <= 1; ++ring) {
        const float y = ring == 0 ? -1.0f : 1.0f;
        for (int s = 0; s <= segments; ++s) {
            const float a = 2.0f * pi * static_cast<float>(s) / segments;
            const float cx = std::cos(a), cz = std::sin(a);
            mesh.vertices.push_back({cx, y, cz, cx, 0.0f, cz});
        }
    }
    for (int s = 0; s < segments; ++s) {
        const uint32_t a = static_cast<uint32_t>(s);
        const uint32_t b = static_cast<uint32_t>(s + 1);
        const uint32_t c = static_cast<uint32_t>(segments + 1 + s);
        const uint32_t d = static_cast<uint32_t>(segments + 1 + s + 1);
        addTriangle(mesh, a, c, b);
        addTriangle(mesh, b, c, d);
    }
    for (int cap = 0; cap < 2; ++cap) {
        const float y = cap == 0 ? 1.0f : -1.0f;
        const uint32_t center = static_cast<uint32_t>(mesh.vertices.size());
        mesh.vertices.push_back({0.0f, y, 0.0f, 0.0f, y, 0.0f});
        const uint32_t base = static_cast<uint32_t>(mesh.vertices.size());
        for (int s = 0; s <= segments; ++s) {
            const float a = 2.0f * pi * static_cast<float>(s) / segments;
            mesh.vertices.push_back({std::cos(a), y, std::sin(a), 0.0f, y, 0.0f});
        }
        for (int s = 0; s < segments; ++s) {
            if (y > 0.0f) addTriangle(mesh, center, base + s + 1, base + s);
            else addTriangle(mesh, center, base + s, base + s + 1);
        }
    }
    return mesh;
}

// Unit cone matching Velox's centroid-origin convention: for half total
// height h the base plane sits at y = -0.5 and the apex at y = +1.5 (in units
// of h). Instances scale y by capsuleHalfHeight, xz by the base radius.
inline CpuMesh makeCone(int segments = 28) {
    CpuMesh mesh;
    const float pi = 3.14159265358979323846f;
    const float baseY = -0.5f, apexY = 1.5f;
    // Slant normal for r=1, height 2: (cos, 0.5, sin) normalized.
    const float ny = 0.4472136f, nr = 0.8944272f;
    for (int s = 0; s <= segments; ++s) {
        const float a = 2.0f * pi * static_cast<float>(s) / segments;
        const float cx = std::cos(a), cz = std::sin(a);
        mesh.vertices.push_back({cx, baseY, cz, cx * nr, ny, cz * nr});
        mesh.vertices.push_back({0.0f, apexY, 0.0f, cx * nr, ny, cz * nr});
    }
    for (int s = 0; s < segments; ++s) {
        const uint32_t a = static_cast<uint32_t>(2 * s);
        addTriangle(mesh, a, a + 1, a + 2);
    }
    const uint32_t center = static_cast<uint32_t>(mesh.vertices.size());
    mesh.vertices.push_back({0.0f, baseY, 0.0f, 0.0f, -1.0f, 0.0f});
    const uint32_t base = static_cast<uint32_t>(mesh.vertices.size());
    for (int s = 0; s <= segments; ++s) {
        const float a = 2.0f * pi * static_cast<float>(s) / segments;
        mesh.vertices.push_back({std::cos(a), baseY, std::sin(a), 0.0f, -1.0f, 0.0f});
    }
    for (int s = 0; s < segments; ++s)
        addTriangle(mesh, center, base + s, base + s + 1);
    return mesh;
}

// Incremental convex hull over a small point cloud, emitted flat-shaded
// (duplicated vertices with per-face normals). Good for the tens-of-points
// hulls the sandbox spawns; not meant for large inputs.
inline CpuMesh makeHullMesh(const std::vector<std::array<float, 3>>& points) {
    CpuMesh mesh;
    const size_t n = points.size();
    if (n < 4) return mesh;
    auto sub = [](const std::array<float, 3>& a, const std::array<float, 3>& b) {
        return std::array<float, 3>{a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    };
    auto crossv = [](const std::array<float, 3>& a, const std::array<float, 3>& b) {
        return std::array<float, 3>{a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2],
                                    a[0]*b[1] - a[1]*b[0]};
    };
    auto dotv = [](const std::array<float, 3>& a, const std::array<float, 3>& b) {
        return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
    };

    // Seed tetrahedron: two most distant on x, then farthest from line/plane.
    size_t i0 = 0, i1 = 1;
    float best = -1.0f;
    for (size_t i = 0; i < n; ++i)
        for (size_t j = i + 1; j < n; ++j) {
            const auto d = sub(points[i], points[j]);
            const float len = dotv(d, d);
            if (len > best) { best = len; i0 = i; i1 = j; }
        }
    size_t i2 = SIZE_MAX;
    best = 1e-12f;
    for (size_t i = 0; i < n; ++i) {
        if (i == i0 || i == i1) continue;
        const auto c = crossv(sub(points[i1], points[i0]), sub(points[i], points[i0]));
        const float area = dotv(c, c);
        if (area > best) { best = area; i2 = i; }
    }
    if (i2 == SIZE_MAX) return mesh;
    size_t i3 = SIZE_MAX;
    best = 1e-12f;
    const auto seedNormal = crossv(sub(points[i1], points[i0]), sub(points[i2], points[i0]));
    for (size_t i = 0; i < n; ++i) {
        if (i == i0 || i == i1 || i == i2) continue;
        const float volume = std::fabs(dotv(seedNormal, sub(points[i], points[i0])));
        if (volume > best) { best = volume; i3 = i; }
    }
    if (i3 == SIZE_MAX) return mesh;

    struct Face { uint32_t a, b, c; };
    std::vector<Face> faces;
    auto faceNormal = [&](const Face& f) {
        return crossv(sub(points[f.b], points[f.a]), sub(points[f.c], points[f.a]));
    };
    auto addFace = [&](uint32_t a, uint32_t b, uint32_t c, uint32_t opposite) {
        Face f{a, b, c};
        // Orient outward: the opposite seed point must be behind the face.
        if (dotv(faceNormal(f), sub(points[opposite], points[a])) > 0.0f)
            std::swap(f.b, f.c);
        faces.push_back(f);
    };
    addFace((uint32_t)i0, (uint32_t)i1, (uint32_t)i2, (uint32_t)i3);
    addFace((uint32_t)i0, (uint32_t)i1, (uint32_t)i3, (uint32_t)i2);
    addFace((uint32_t)i0, (uint32_t)i2, (uint32_t)i3, (uint32_t)i1);
    addFace((uint32_t)i1, (uint32_t)i2, (uint32_t)i3, (uint32_t)i0);

    for (size_t p = 0; p < n; ++p) {
        if (p == i0 || p == i1 || p == i2 || p == i3) continue;
        std::vector<char> visible(faces.size(), 0);
        bool any = false;
        for (size_t f = 0; f < faces.size(); ++f) {
            const auto normal = faceNormal(faces[f]);
            if (dotv(normal, sub(points[p], points[faces[f].a])) > 1e-7f) {
                visible[f] = 1;
                any = true;
            }
        }
        if (!any) continue;
        // Horizon: directed edges of visible faces whose reverse edge is not
        // shared with another visible face.
        std::map<std::pair<uint32_t, uint32_t>, int> edges;
        for (size_t f = 0; f < faces.size(); ++f) {
            if (!visible[f]) continue;
            const uint32_t e[3][2] = {{faces[f].a, faces[f].b},
                                      {faces[f].b, faces[f].c},
                                      {faces[f].c, faces[f].a}};
            for (const auto& edge : e) ++edges[{edge[0], edge[1]}];
        }
        std::vector<std::pair<uint32_t, uint32_t>> horizon;
        for (const auto& entry : edges)
            if (edges.find({entry.first.second, entry.first.first}) == edges.end())
                horizon.push_back(entry.first);
        std::vector<Face> next;
        for (size_t f = 0; f < faces.size(); ++f)
            if (!visible[f]) next.push_back(faces[f]);
        for (const auto& edge : horizon)
            next.push_back({edge.first, edge.second, (uint32_t)p});
        faces.swap(next);
    }

    for (const Face& f : faces) {
        auto normal = faceNormal(f);
        const float len = std::sqrt(dotv(normal, normal));
        if (len < 1e-12f) continue;
        normal = {normal[0]/len, normal[1]/len, normal[2]/len};
        const uint32_t base = static_cast<uint32_t>(mesh.vertices.size());
        for (uint32_t index : {f.a, f.b, f.c})
            mesh.vertices.push_back({points[index][0], points[index][1], points[index][2],
                                     normal[0], normal[1], normal[2]});
        addTriangle(mesh, base, base + 1, base + 2);
    }
    return mesh;
}

// Ground quad: unit square in xz at y = 0, up normal (instances scale it big).
inline CpuMesh makeGroundQuad() {
    CpuMesh mesh;
    mesh.vertices = {
        {-1, 0, -1, 0, 1, 0}, {1, 0, -1, 0, 1, 0},
        {1, 0, 1, 0, 1, 0}, {-1, 0, 1, 0, 1, 0}};
    addTriangle(mesh, 0, 2, 1);
    addTriangle(mesh, 0, 3, 2);
    return mesh;
}

} // namespace sandbox
