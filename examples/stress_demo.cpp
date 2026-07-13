// Stress tests for Predictive Contact Sweeping — each case targets a classic
// CCD failure mode. Exits nonzero if any invariant is violated.
#include <velox/velox.h>
#include <cmath>
#include <cstdio>
#include <vector>

static int failures = 0;
static void check(bool ok, const char* name) {
    std::printf("%-44s %s\n", name, ok ? "PASS" : "FAIL");
    if (!ok) ++failures;
}

// 1. Tunneling: tiny sphere into the floor at 2 km/s.
static void testBullet() {
    velox::World w;
    w.addStaticPlane({0, 1, 0}, 0.0f);
    auto id = w.addSphere({0, 1.0f, 0}, 0.05f, 0.01f);
    w.body(id).velocity = {0, -2000.0f, 0};
    bool ok = true;
    for (int i = 0; i < 120; ++i) {
        w.step(1.0f / 60.0f);
        ok &= w.body(id).position.y >= -1e-3f;
    }
    check(ok, "bullet vs floor (no tunneling)");
}

// 2. Sticky grazing: a sphere skimming just above the floor must not be
// slowed or snagged by speculative contacts it never actually touches.
static void testGrazing() {
    velox::World w;
    w.gravity = {0, 0, 0};
    w.addStaticPlane({0, 1, 0}, 0.0f);
    auto id = w.addSphere({0, 0.5f + 0.01f, 0}, 0.5f, 1.0f); // 1 cm clearance
    w.body(id).velocity = {100.0f, 0, 0};                    // fast horizontal skim
    for (int i = 0; i < 60; ++i) w.step(1.0f / 60.0f);
    const auto& b = w.body(id);
    bool ok = std::fabs(b.velocity.x - 100.0f) < 1e-3f && std::fabs(b.velocity.y) < 1e-3f;
    check(ok, "grazing skim (no sticky contacts)");
}

// 3. TOI stall: a pile of touching spheres generates endless zero-time
// impacts in an event-driven CCD loop. PCS handles it with solver iterations;
// this passing at all (in bounded time) plus nobody sinking is the test.
static void testPile() {
    velox::World w;
    w.addStaticPlane({0, 1, 0}, 0.0f);
    const int N = 125; // 5x5x5 pile
    for (int x = 0; x < 5; ++x)
        for (int y = 0; y < 5; ++y)
            for (int z = 0; z < 5; ++z)
                w.addSphere({x * 1.01f, 0.5f + y * 1.01f, z * 1.01f}, 0.5f, 1.0f);
    for (int i = 0; i < 300; ++i) w.step(1.0f / 60.0f); // 5 s to settle
    bool ok = true;
    for (velox::BodyId id = 1; id <= N; ++id)
        ok &= w.body(id).position.y >= 0.5f - 5e-2f; // nobody sank into the floor
    check(ok, "125-sphere pile (no TOI stall, no sinking)");
}

// 4. High-speed sphere-vs-sphere: two bullets meeting head-on between frames.
static void testHeadOn() {
    velox::World w;
    w.gravity = {0, 0, 0};
    auto a = w.addSphere({-50, 0, 0}, 0.1f, 1.0f);
    auto b = w.addSphere({+50, 0, 0}, 0.1f, 1.0f);
    w.body(a).velocity = {+3000, 0, 0};
    w.body(b).velocity = {-3000, 0, 0};
    bool ok = true;
    for (int i = 0; i < 60; ++i) {
        w.step(1.0f / 60.0f);
        // If they tunneled, they'd swap sides and keep flying apart while
        // still approaching in "crossed" positions. Overlap must never persist.
        float dist = velox::length(w.body(a).position - w.body(b).position);
        ok &= dist >= 0.2f - 1e-2f;
    }
    check(ok, "3 km/s head-on spheres (no pass-through)");
}

// 5. Resting stability: a ball sitting on the floor must not jitter or sink.
static void testResting() {
    velox::World w;
    w.addStaticPlane({0, 1, 0}, 0.0f);
    auto id = w.addSphere({0, 0.5f, 0}, 0.5f, 1.0f);
    for (int i = 0; i < 600; ++i) w.step(1.0f / 60.0f); // 10 s
    const auto& b = w.body(id);
    bool ok = std::fabs(b.position.y - 0.5f) < 5e-3f && std::fabs(b.velocity.y) < 0.2f;
    check(ok, "resting sphere (stable, no jitter/sink)");
}

// 6. Resting box: needs a proper contact manifold + rotational solver or it
// wobbles on one point and falls over.
static void testRestingBox() {
    velox::World w;
    w.addStaticPlane({0, 1, 0}, 0.0f);
    auto id = w.addBox({0, 0.5f, 0}, {0.5f, 0.5f, 0.5f}, 1.0f);
    for (int i = 0; i < 600; ++i) w.step(1.0f / 60.0f); // 10 s
    const auto& b = w.body(id);
    bool ok = std::fabs(b.position.y - 0.5f) < 2e-2f &&
              velox::length(b.angularVelocity) < 0.5f;
    check(ok, "resting box (manifold keeps it flat)");
}

// 7. Fast box vs floor: tunneling test for a rotated non-sphere shape.
static void testFastBox() {
    velox::World w;
    w.addStaticPlane({0, 1, 0}, 0.0f);
    auto id = w.addBox({0, 2.0f, 0}, {0.2f, 0.2f, 0.2f}, 1.0f);
    w.body(id).orientation = velox::fromAxisAngle({1, 0, 1}, 0.7f);
    w.body(id).velocity = {0, -1500.0f, 0};
    w.body(id).angularVelocity = {30.0f, 0, 10.0f}; // spinning hard, too
    bool ok = true;
    for (int i = 0; i < 120; ++i) {
        w.step(1.0f / 60.0f);
        ok &= w.body(id).position.y >= -0.4f; // center may dip to ~vertex depth only
    }
    check(ok, "1.5 km/s spinning box vs floor (no tunneling)");
}

// 8. Capsule resting and rolling on a plane.
static void testCapsule() {
    velox::World w;
    w.addStaticPlane({0, 1, 0}, 0.0f);
    auto id = w.addCapsule({0, 2.0f, 0}, 0.3f, 0.5f, 1.0f);
    w.body(id).orientation = velox::fromAxisAngle({0, 0, 1}, 1.5707963f); // lying down
    for (int i = 0; i < 600; ++i) w.step(1.0f / 60.0f);
    const auto& b = w.body(id);
    bool ok = std::fabs(b.position.y - 0.3f) < 2e-2f;
    check(ok, "capsule settles lying on the floor");
}

// 9. Mesh collider: a ball dropped onto a V-shaped triangle-mesh ramp must
// come to rest inside the valley, and a bullet must not pierce the mesh.
static void testMesh() {
    velox::World w;
    // V-trough: two rectangles meeting at y=0 along the z axis.
    std::vector<velox::Vec3> verts = {
        {-4, 4, -4}, {0, 0, -4}, {0, 0, 4}, {-4, 4, 4},   // left wall
        {4, 4, -4},  {0, 0, -4}, {0, 0, 4}, {4, 4, 4},    // right wall
    };
    std::vector<uint32_t> idx = {0, 1, 2, 0, 2, 3, 4, 6, 5, 4, 7, 6};
    w.addStaticMesh(verts, idx);

    auto ball = w.addSphere({-2.0f, 3.5f, 0}, 0.4f, 1.0f);
    auto bullet = w.addSphere({2.0f, 30.0f, 0}, 0.05f, 0.01f);
    w.body(bullet).velocity = {0, -1200.0f, 0};

    bool ok = true;
    for (int i = 0; i < 600; ++i) {
        w.step(1.0f / 60.0f);
        // Neither body may end up below the trough surface (y >= |x| - slack).
        for (auto id : {ball, bullet}) {
            const auto& b = w.body(id);
            if (std::fabs(b.position.x) < 3.5f && std::fabs(b.position.z) < 3.5f)
                ok &= b.position.y >= std::fabs(b.position.x) - 0.5f;
        }
    }
    ok &= velox::length(w.body(ball).velocity) < 1.0f; // ball settled in the valley
    check(ok, "triangle-mesh trough (rest + no tunneling)");
}

int main() {
    testBullet();
    testGrazing();
    testPile();
    testHeadOn();
    testResting();
    testRestingBox();
    testFastBox();
    testCapsule();
    testMesh();
    std::printf("\n%s\n", failures == 0 ? "All stress tests passed."
                                        : "STRESS TESTS FAILED");
    return failures;
}
