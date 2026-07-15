// Prototype test for persistent contact manifolds with Sutherland-Hodgman clipping.
// Tests face extraction, polygon clipping, and feature ID persistence.

#include "../src/narrowphase.h"
#include "../include/velox/world.h"
#include <cstdio>
#include <cmath>
#include <cstring>

using namespace velox;

static int failures = 0;
#define check(cond) do { \
    if (!(cond)) { \
        printf("FAIL: %s (line %d)\n", #cond, __LINE__); \
        failures++; \
    } else { \
        printf("PASS: %s\n", #cond); \
    } \
} while(0)

// Helper to create a Box Convex shape
static Convex makeBox(Vec3 pos, Vec3 extents, Quat rot = {}) {
    Convex c{};
    c.kind = Convex::Box;
    c.position = pos;
    c.orientation = rot;
    c.halfExtents = extents;
    return c;
}

// Helper to create a Hull Convex shape (unit cube as hull)
static Convex makeHull(Vec3 pos, Vec3 extents, Quat rot = {}) {
    Convex c{};
    c.kind = Convex::Hull;
    c.position = pos;
    c.orientation = rot;
    // 8 vertices of a unit cube centered at origin
    static const Vec3 pts[8] = {
        {-0.5f, -0.5f, -0.5f}, {0.5f, -0.5f, -0.5f},
        {-0.5f, 0.5f, -0.5f}, {0.5f, 0.5f, -0.5f},
        {-0.5f, -0.5f, 0.5f}, {0.5f, -0.5f, 0.5f},
        {-0.5f, 0.5f, 0.5f}, {0.5f, 0.5f, 0.5f}
    };
    static const uint32_t faces[] = {
        0, 1, 5, 0, 5, 4, 2, 6, 7, 2, 7, 3,
        0, 2, 3, 0, 3, 1, 4, 5, 7, 4, 7, 6,
        0, 4, 6, 0, 6, 2, 1, 3, 7, 1, 7, 5,
    };
    c.hullPts = pts;
    c.hullCount = 8;
    c.hullFaceIndices = faces;
    c.hullFaceCount = 12;
    return c;
}

static bool sameFeatureSet(const Manifold& a, const Manifold& b) {
    if (a.pointCount != b.pointCount) return false;
    for (int i = 0; i < a.pointCount; ++i) {
        bool found = false;
        for (int j = 0; j < b.pointCount; ++j)
            if (a.points[i].featureIdA == b.points[j].featureIdA &&
                a.points[i].featureIdB == b.points[j].featureIdB) found = true;
        if (!found) return false;
    }
    return true;
}

static void copyWarmStart(Contact* current, int currentCount,
                          const Contact* previous, int previousCount) {
    for (int i = 0; i < currentCount; ++i)
        for (int j = 0; j < previousCount; ++j)
            if (current[i].featureKey == previous[j].featureKey) {
                current[i].normalImpulse = previous[j].normalImpulse;
                current[i].tangentImpulse1 = previous[j].tangentImpulse1;
                current[i].tangentImpulse2 = previous[j].tangentImpulse2;
                break;
            }
}

int main() {
    printf("=== Proto Manifold Tests ===\n\n");

    // Test 1: Box-on-box face stack (4-point manifold)
    {
        printf("Test 1: Box-on-box face stack\n");
        Convex A = makeBox({0, 0.5f, 0}, {1, 1, 1});
        Convex B = makeBox({0, -0.5f, 0}, {4, 1, 4});
        Vec3 normal{0, 1, 0};

        Manifold manifold;
        int count = clipFaceManifold(A, B, normal, &manifold);
        check(count == 4); // Should get 4 points on the face
        bool correctDepth = true;
        for (int i = 0; i < count; ++i)
            correctDepth &= std::fabs(manifold.points[i].gap + 1.0f) < 1e-5f &&
                            std::fabs(manifold.points[i].pointA.y + 0.5f) < 1e-5f &&
                            std::fabs(manifold.points[i].pointB.y - 0.5f) < 1e-5f;
        check(correctDepth);
    }

    // Test 2: Edge-on-face contact (2-point manifold)
    {
        printf("\nTest 2: Edge-on-face contact\n");
        Convex A = makeBox({0, 0.5f, 0}, {1, 1, 1});
        Convex B = makeBox({0, -0.5f, 0}, {1, 1, 1});
        Vec3 normal{0, 1, 0};

        Manifold manifold;
        int count = clipFaceManifold(A, B, normal, &manifold);
        check(count >= 2); // Should get at least 2 points
    }

    // Test 3: Partial overlap area (boxes sliding past each other)
    {
        printf("\nTest 3: Partial overlap\n");
        // Two boxes overlapping in x, with contact normal along x
        Convex A = makeBox({0.5f, 0, 0}, {1, 1, 1});
        Convex B = makeBox({-0.5f, 0, 0}, {1, 1, 1});
        Vec3 normal{1, 0, 0};

        Manifold manifold;
        int count = clipFaceManifold(A, B, normal, &manifold);
        check(count >= 2); // Should get some overlap points
    }

    // Test 4: Hull-vs-box face contact
    {
        printf("\nTest 4: Hull-vs-box face contact\n");
        Convex A = makeHull({0, 0.5f, 0}, {1, 1, 1});
        Convex B = makeBox({0, -0.5f, 0}, {1, 1, 1});
        Vec3 normal{0, 1, 0};

        Manifold manifold;
        int count = clipFaceManifold(A, B, normal, &manifold);
        check(count >= 2); // Should get multiple points
    }

    // Test 5: Determinism — run 3 times and verify bitwise identical output
    {
        printf("\nTest 5: Determinism (3 runs)\n");
        Convex A = makeBox({0, 0.5f, 0}, {1, 1, 1});
        Convex B = makeBox({0, -0.5f, 0}, {1, 1, 1});
        Vec3 normal{0, 1, 0};

        Manifold m1{}, m2{}, m3{};
        clipFaceManifold(A, B, normal, &m1);
        clipFaceManifold(A, B, normal, &m2);
        clipFaceManifold(A, B, normal, &m3);

        check(memcmp(&m1, &m2, sizeof(Manifold)) == 0);
        check(memcmp(&m2, &m3, sizeof(Manifold)) == 0);
    }

    // Test 6: Feature ID persistence across frames
    {
        printf("\nTest 6: Feature ID persistence\n");
        Convex A = makeBox({0, 0.5f, 0}, {1, 1, 1});
        Convex B = makeBox({0, -0.5f, 0}, {4, 1, 4});
        Vec3 normal{0, 1, 0};

        Manifold m1{}, m2{};
        clipFaceManifold(A, B, normal, &m1);
        clipFaceManifold(A, B, normal, &m2);

        // Feature IDs should be identical across frames for the same geometry
        check(sameFeatureSet(m1, m2));

        // A small tangential slide must retain the same bottom-face vertex
        // ids while the overlap remains fully inside B's supporting face.
        A.position.x += 0.05f;
        clipFaceManifold(A, B, normal, &m2);
        check(sameFeatureSet(m1, m2));
    }

    // Test 7: regenerate contacts each frame, warm-start only by feature id,
    // and solve exactly three iterations. This verifies that no stale world
    // anchors are reused and that a resting box retains a four-point manifold.
    {
        printf("\nTest 7: 3-iteration warm-started resting box\n");
        Body box{}, ground{};
        box.shape = ShapeType::Box;
        box.motionType = MotionType::Dynamic;
        box.position = {0, 0.5f, 0};
        box.halfExtents = {0.5f, 0.5f, 0.5f};
        box.invMass = 1.0f;
        box.invInertia = {6.0f, 6.0f, 6.0f};
        ground.shape = ShapeType::Box;
        ground.motionType = MotionType::Static;
        ground.position = {0, -0.5f, 0};
        ground.halfExtents = {4.0f, 0.5f, 4.0f};
        MeshSoupView soup{};
        Contact previous[kMaxContactsPerPair]{};
        int previousCount = 0;
        bool stable = true;
        for (int frame = 0; frame < 180; ++frame) {
            box.velocity.y -= 9.81f / 60.0f;
            Contact contacts[kMaxContactsPerPair]{};
            int count = collideSimplePair(box, ground, 0, 1, soup, 1.0f / 60.0f,
                                          contacts, kMaxContactsPerPair);
            if (count != 4) {
                printf("  frame %d: contact count %d\n", frame, count);
                stable = false;
            }
            copyWarmStart(contacts, count, previous, previousCount);
            for (int i = 0; i < count; ++i) warmStartContact(box, ground, contacts[i]);
            for (int iteration = 0; iteration < 3; ++iteration)
                for (int i = 0; i < count; ++i) solveContact(box, ground, contacts[i], 1.0f / 60.0f);
            box.advanceTransform(1.0f / 60.0f);
            if (frame >= 10 && (std::fabs(box.position.y - 0.5f) >= 1e-3f ||
                                std::fabs(box.velocity.y) >= 1e-3f)) {
                printf("  frame %d: y=%g vy=%g\n", frame,
                       box.position.y, box.velocity.y);
                stable = false;
            }
            previousCount = count;
            for (int i = 0; i < count; ++i) previous[i] = contacts[i];
        }
        check(stable);
    }

    // Test 8: the CUDA backend executes the same clipped manifold code. The
    // public API intentionally hides contacts, so verify the externally
    // observable contract: an active four-contact manifold and matching motion.
    {
        printf("\nTest 8: CPU/CUDA clipped-manifold parity\n");
        World cpu(BackendType::Cpu), accelerated(BackendType::Auto);
        cpu.gravity = accelerated.gravity = {0, -9.81f, 0};
        auto setup = [](World& world) {
            world.addBox({0, -0.5f, 0}, {4, 0.5f, 4}, 0.0f);
            return world.addBox({0, 0.5f, 0}, {0.5f, 0.5f, 0.5f}, 1.0f);
        };
        BodyId cpuBox = setup(cpu);
        BodyId acceleratedBox = setup(accelerated);
        bool initialManifold = false, trajectoryMatch = true;
        for (int frame = 0; frame < 60; ++frame) {
            cpu.step(1.0f / 60.0f);
            accelerated.step(1.0f / 60.0f);
            if (frame == 0)
                initialManifold = cpu.lastStepStats().generatedContacts == 4 &&
                                  accelerated.lastStepStats().generatedContacts == 4;
            const Body& expected = cpu.body(cpuBox);
            const Body& actual = accelerated.body(acceleratedBox);
            trajectoryMatch &= length(expected.position - actual.position) < 1e-2f &&
                               length(expected.velocity - actual.velocity) < 1e-2f;
        }
        bool cuda = std::strcmp(accelerated.backendName(), "cuda") == 0;
        if (!cuda) printf("  CUDA unavailable; checked CPU fallback\n");
        check(initialManifold && (!cuda || trajectoryMatch));
    }

    printf("\n=== Results: %d failures ===\n", failures);
    return failures > 0 ? 1 : 0;
}
