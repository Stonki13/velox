#include <velox/velox.h>
#include "../src/gjk.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <vector>

namespace {

using namespace velox;

struct Rng {
    uint32_t state;
    explicit Rng(uint32_t seed) : state(seed ? seed : 1u) {}
    float next() {
        state = state * 1664525u + 1013904223u;
        return (state >> 8) * (1.0f / 16777216.0f);
    }
    float range(float lo, float hi) { return lo + (hi - lo) * next(); }
};

bool finiteVec(const Vec3& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

Convex makeBox(Vec3 position, Vec3 halfExtents) {
    Convex result{};
    result.kind = Convex::Box;
    result.position = position;
    result.halfExtents = halfExtents;
    result.boundingRadius = length(halfExtents);
    result.geometryQuality = GeometryQuality::Robust;
    return result;
}

Convex makeSphere(Vec3 position, float radius) {
    Convex result{};
    result.kind = Convex::Point;
    result.position = position;
    result.radius = radius;
    result.geometryQuality = GeometryQuality::Robust;
    return result;
}

Convex makeHull(Vec3 position, const std::vector<Vec3>& points) {
    Convex result{};
    result.kind = Convex::Hull;
    result.position = position;
    result.hullPts = points.data();
    result.hullCount = static_cast<uint32_t>(points.size());
    for (const Vec3& point : points)
        result.boundingRadius = vmax(result.boundingRadius, length(point));
    result.geometryQuality = GeometryQuality::Robust;
    return result;
}

bool validResult(const GjkResult& result) {
    return std::isfinite(result.distance) && finiteVec(result.normal) &&
           finiteVec(result.pointA) && finiteVec(result.pointB) &&
           lengthSq(result.normal) > 0.25f && lengthSq(result.normal) < 2.0f;
}

bool expectFinite(const char* name, const GjkResult& result) {
    if (validResult(result)) return true;
    std::printf("geometry_fuzz: %s returned a non-finite result\n", name);
    return false;
}

bool regressionCases() {
    Convex needle = makeBox({}, {500.0f, 0.001f, 0.001f});
    Convex sphere = makeSphere({500.75f, 0.0f, 0.0f}, 0.25f);
    GjkResult needleResult = gjkDistance(needle, sphere);
    if (!expectFinite("needle", needleResult) ||
        std::fabs(needleResult.distance - 0.5f) > 1e-3f ||
        needleResult.normal.x > -0.99f) {
        std::printf("geometry_fuzz: needle regression failed (distance %.9g normal %.9g %.9g %.9g)\n",
                    needleResult.distance, needleResult.normal.x,
                    needleResult.normal.y, needleResult.normal.z);
        return false;
    }

    std::vector<Vec3> coplanar = {
        {-1.0f, -1.0f, 0.0f}, {1.0f, -1.0f, 0.0f},
        {1.0f, 1.0f, 0.0f}, {-1.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 0.0f}, {0.2f, -0.4f, 0.0f}};
    if (!expectFinite("coplanar hull", gjkDistance(makeHull({}, coplanar),
                                                     makeSphere({0, 0, 0.01f}, 0.01f))))
        return false;

    Convex tinyA = makeBox({}, {1e-6f, 1e-6f, 1e-6f});
    Convex tinyB = makeBox({2.5e-6f, 0, 0}, {1e-6f, 1e-6f, 1e-6f});
    if (!expectFinite("tiny shapes", gjkDistance(tinyA, tinyB))) return false;
    return true;
}

bool diagnosticsCases() {
    World world(BackendType::Cpu);
    BodyId box = world.addBox({}, {4.0f, 2.0f, 1.0f}, 1.0f);
    GeometryDiagnostics boxInfo = world.queryGeometryDiagnostics(box);
    if (std::fabs(boxInfo.minEdgeLength - 2.0f) > 1e-6f ||
        std::fabs(boxInfo.maxEdgeLength - 8.0f) > 1e-6f ||
        std::fabs(boxInfo.aspectRatio - 4.0f) > 1e-6f ||
        std::fabs(boxInfo.volume - 64.0f) > 1e-5f || boxInfo.isDegenerate) {
        std::printf("geometry_fuzz: box diagnostics regression failed\n");
        return false;
    }
    std::vector<Vec3> tetra = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    BodyId hull = world.addConvexHull({}, tetra, 1.0f);
    GeometryDiagnostics hullInfo = world.queryGeometryDiagnostics(hull);
    if (std::fabs(hullInfo.volume - 1.0f / 6.0f) > 1e-5f || hullInfo.isDegenerate) {
        std::printf("geometry_fuzz: hull diagnostics regression failed\n");
        return false;
    }
    tetra.push_back({0, 0, 0});
    try {
        world.addConvexHull({}, tetra, 1.0f);
        std::printf("geometry_fuzz: duplicate hull point was accepted\n");
        return false;
    } catch (const std::invalid_argument&) {
    }
    return true;
}

bool checkMetamorphic(const Convex& a, const Convex& b, int iteration) {
    GjkResult ab = gjkDistance(a, b);
    GjkResult ba = gjkDistance(b, a);
    if (!expectFinite("random pair", ab) || !expectFinite("swapped random pair", ba)) return false;
    float scale = vmax(1.0f, gjk_detail::pairScale(a, b));
    // Single-precision support points span scales from 1e-4 to 1e3 here.
    // Use a scale-relative surface tolerance instead of an absolute epsilon.
    float tolerance = 5e-3f * scale;
    if (!ab.overlapping && !ba.overlapping &&
        std::fabs(ab.distance - ba.distance) > tolerance) {
        std::printf("geometry_fuzz: symmetry failed at iteration %d (%.9g vs %.9g)\n",
                    iteration, ab.distance, ba.distance);
        return false;
    }
    if (!ab.overlapping && !ba.overlapping && std::fabs(ab.distance) > tolerance &&
        dot(ab.normal, ba.normal) > -0.95f) {
        std::printf("geometry_fuzz: normal symmetry failed at iteration %d\n", iteration);
        return false;
    }

    Convex translatedA = a, translatedB = b;
    float translationScale = vmax(1e-3f,
                                  vmin(gjk_detail::convexScale(a),
                                       gjk_detail::convexScale(b)));
    Vec3 translation{translationScale * 1.25f, -translationScale * 0.5f,
                     translationScale * 0.75f};
    translatedA.position += translation;
    translatedB.position += translation;
    GjkResult translated = gjkDistance(translatedA, translatedB);
    if (!expectFinite("translated random pair", translated) ||
        std::fabs(ab.distance - translated.distance) > tolerance) {
        std::printf("geometry_fuzz: translation invariance failed at iteration %d "
                    "(%.9g vs %.9g scale %.9g tolerance %.9g)\n", iteration,
                    ab.distance, translated.distance, scale, tolerance);
        std::printf("  overlap original=%d translated=%d\n",
                    int(ab.overlapping), int(translated.overlapping));
        std::printf("  box p=(%.9g %.9g %.9g) h=(%.9g %.9g %.9g), "
                    "hull p=(%.9g %.9g %.9g) bound=%.9g count=%u\n",
                    a.position.x, a.position.y, a.position.z,
                    a.halfExtents.x, a.halfExtents.y, a.halfExtents.z,
                    b.position.x, b.position.y, b.position.z,
                    b.boundingRadius, b.hullCount);
        return false;
    }

    if (!ab.overlapping && ab.distance > tolerance) {
        float expectedA = dot(-ab.normal, a.support(-ab.normal)) - a.radius;
        float expectedB = dot(ab.normal, b.support(ab.normal)) + b.radius;
        if (std::fabs(dot(-ab.normal, ab.pointA) - expectedA) > tolerance ||
            std::fabs(dot(ab.normal, ab.pointB) - expectedB) > tolerance) {
            std::printf("geometry_fuzz: witness support failed at iteration %d "
                        "(distance %.9g residuals %.9g %.9g tolerance %.9g)\n",
                        iteration, ab.distance,
                        dot(-ab.normal, ab.pointA) - expectedA,
                        dot(ab.normal, ab.pointB) - expectedB, tolerance);
            std::printf("  box p=(%.9g %.9g %.9g) h=(%.9g %.9g %.9g), "
                        "hull p=(%.9g %.9g %.9g) bound=%.9g count=%u\n",
                        a.position.x, a.position.y, a.position.z,
                        a.halfExtents.x, a.halfExtents.y, a.halfExtents.z,
                        b.position.x, b.position.y, b.position.z,
                        b.boundingRadius, b.hullCount);
            return false;
        }
    }
    return true;
}

} // namespace

int main(int argc, char** argv) {
    int iterations = argc > 1 ? std::atoi(argv[1]) : 1000;
    if (iterations < 1) iterations = 1;
    if (!regressionCases() || !diagnosticsCases()) return 1;

    Rng rng(0x6a09e667u);
    for (int i = 0; i < iterations; ++i) {
        float scaleA = std::pow(10.0f, rng.range(-4.0f, 3.0f));
        float scaleB = std::pow(10.0f, rng.range(-4.0f, 3.0f));
        Convex a = makeBox({rng.range(-5.0f, 5.0f), rng.range(-5.0f, 5.0f), rng.range(-5.0f, 5.0f)},
                           {scaleA * rng.range(0.001f, 1.0f), scaleA * rng.range(0.001f, 1.0f),
                            scaleA * rng.range(0.001f, 1.0f)});
        std::vector<Vec3> points;
        int pointCount = 4 + int(rng.next() * 61.0f);
        points.reserve(pointCount);
        for (int point = 0; point < pointCount; ++point)
            points.push_back({scaleB * rng.range(-1.0f, 1.0f),
                              scaleB * rng.range(-1.0f, 1.0f),
                              scaleB * rng.range(-1.0f, 1.0f)});
        Convex b = makeHull({rng.range(-5.0f, 5.0f), rng.range(-5.0f, 5.0f), rng.range(-5.0f, 5.0f)},
                            points);
        if (!checkMetamorphic(a, b, i)) return 1;
    }
    std::printf("geometry_fuzz: PASS (%d deterministic pairs)\n", iterations);
    return 0;
}
