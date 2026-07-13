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

// 10. Ball-joint pendulum: anchor distance must stay constant while it swings.
static void testPendulum() {
    velox::World w;
    auto anchor = w.addBox({0, 5, 0}, {0.1f, 0.1f, 0.1f}, 0.0f); // static
    auto bob = w.addSphere({2, 5, 0}, 0.3f, 1.0f);
    w.addBallJoint(anchor, bob, {0, 5, 0});
    bool ok = true;
    bool swung = false;
    for (int i = 0; i < 600; ++i) {
        w.step(1.0f / 60.0f);
        float len = velox::length(w.body(bob).position - velox::Vec3{0, 5, 0});
        ok &= std::fabs(len - 2.0f) < 0.1f;   // joint holds
        if (w.body(bob).position.x < -1.0f) swung = true; // it actually swings
    }
    check(ok && swung, "ball-joint pendulum (constraint holds, swings)");
}

// 11. Hinge: a door pushed sideways may only rotate about the hinge axis.
static void testHinge() {
    velox::World w;
    w.gravity = {0, 0, 0};
    auto frame = w.addBox({0, 2, 0}, {0.05f, 1.0f, 0.05f}, 0.0f); // static post
    auto door = w.addBox({0.55f, 2, 0}, {0.5f, 0.9f, 0.05f}, 2.0f);
    w.addHingeJoint(frame, door, {0.05f, 2, 0}, {0, 1, 0});
    w.body(door).velocity = {0, 0, 3.0f}; // shove the door
    bool ok = true;
    for (int i = 0; i < 300; ++i) {
        w.step(1.0f / 60.0f);
        const auto& d = w.body(door);
        // Angular velocity must stay along the hinge axis (Y).
        ok &= std::fabs(d.angularVelocity.x) < 0.5f && std::fabs(d.angularVelocity.z) < 0.5f;
        ok &= std::fabs(d.position.y - 2.0f) < 0.05f; // no vertical drift
    }
    check(ok, "hinge door (rotation locked to axis)");
}

// 12. Distance-joint chain hanging under gravity keeps its link lengths.
static void testChain() {
    velox::World w;
    auto top = w.addSphere({0, 6, 0}, 0.1f, 0.0f); // static anchor
    velox::BodyId prev = top;
    velox::BodyId links[4];
    for (int i = 0; i < 4; ++i) {
        links[i] = w.addSphere({0.5f * (i + 1), 6, 0}, 0.1f, 0.5f);
        w.addDistanceJoint(prev, links[i], w.body(prev).position, w.body(links[i]).position);
        prev = links[i];
    }
    bool ok = true;
    float minEndY = 6.0f;
    for (int s = 0; s < 600; ++s) {
        w.step(1.0f / 60.0f);
        velox::Vec3 last{0, 6, 0};
        for (int i = 0; i < 4; ++i) {
            float len = velox::length(w.body(links[i]).position - last);
            ok &= std::fabs(len - 0.5f) < 0.08f; // links never stretch/break
            last = w.body(links[i]).position;
        }
        if (w.body(links[3]).position.y < minEndY) minEndY = w.body(links[3]).position.y;
    }
    ok &= minEndY < 4.8f; // the chain actually swung down under gravity
    check(ok, "distance-joint chain (links hold, hangs)");
}

// 13. Sleeping: a settled pile must fall asleep, stay put, and wake on impact.
static void testSleeping() {
    velox::World w;
    w.addStaticPlane({0, 1, 0}, 0.0f);
    velox::BodyId balls[9];
    for (int i = 0; i < 9; ++i)
        balls[i] = w.addSphere({(i % 3) * 1.05f, 0.5f + (i / 3) * 1.05f, 0}, 0.5f, 1.0f);
    for (int i = 0; i < 400; ++i) w.step(1.0f / 60.0f); // settle ~6.7 s
    bool allAsleep = true;
    for (auto id : balls) allAsleep &= !w.isAwake(id);

    // Wake on impact: drop a ball onto the pile.
    auto intruder = w.addSphere({1.05f, 6, 0}, 0.4f, 1.0f);
    (void)intruder;
    bool wokeUp = false;
    for (int i = 0; i < 120; ++i) {
        w.step(1.0f / 60.0f);
        if (w.isAwake(balls[7])) wokeUp = true; // ball under the impact point
    }
    check(allAsleep && wokeUp, "sleeping pile (sleeps, wakes on impact)");
}

// 14. Raycasts against every collider type.
static void testRaycast() {
    velox::World w;
    w.gravity = {0, 0, 0};
    w.addStaticPlane({0, 1, 0}, 0.0f);
    auto ball = w.addSphere({5, 1, 0}, 0.5f, 1.0f);
    auto box = w.addBox({-5, 1, 0}, {0.5f, 0.5f, 0.5f}, 1.0f);
    std::vector<velox::Vec3> verts = {{-1, 0, 8}, {1, 0, 8}, {0, 2, 8}};
    std::vector<uint32_t> idx = {0, 1, 2};
    auto mesh = w.addStaticMesh(verts, idx);

    auto h1 = w.rayCast({5, 5, 0}, {0, -1, 0}, 10.0f);
    auto h2 = w.rayCast({-5, 1, -5}, {0, 0, 1}, 10.0f);
    auto h3 = w.rayCast({0, 1, 0}, {0, 0, 1}, 20.0f);
    auto h4 = w.rayCast({9, 5, 9}, {0, -1, 0}, 10.0f); // only the plane below
    bool ok = h1.hit && h1.body == ball && std::fabs(h1.t - 3.5f) < 1e-3f &&
              h2.hit && h2.body == box && std::fabs(h2.t - 4.5f) < 1e-3f &&
              h3.hit && h3.body == mesh && std::fabs(h3.t - 8.0f) < 1e-3f &&
              h4.hit && std::fabs(h4.t - 5.0f) < 1e-3f;

    std::vector<velox::BodyId> found;
    w.overlapSphere({5, 1.2f, 0}, 0.9f, found); // clear of the floor plane
    bool overlapOk = found.size() == 1 && found[0] == ball;
    check(ok && overlapOk, "raycast + overlap (sphere/box/mesh/plane)");
}

// 15. Warm starting: a 10-box tower must survive 10 seconds without toppling.
static void testStack() {
    velox::World w;
    w.addStaticPlane({0, 1, 0}, 0.0f);
    velox::BodyId boxes[10];
    for (int i = 0; i < 10; ++i)
        boxes[i] = w.addBox({0, 0.5f + i * 1.001f, 0}, {0.5f, 0.5f, 0.5f}, 1.0f);
    for (int i = 0; i < 600; ++i) w.step(1.0f / 60.0f);
    bool ok = true;
    for (int i = 0; i < 10; ++i) {
        const auto& b = w.body(boxes[i]);
        ok &= std::fabs(b.position.x) < 0.25f && std::fabs(b.position.z) < 0.25f;
        ok &= std::fabs(b.position.y - (0.5f + i * 1.0f)) < 0.2f;
    }
    check(ok, "10-box tower (warm-started stack stays up)");
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
    testPendulum();
    testHinge();
    testChain();
    testSleeping();
    testRaycast();
    testStack();
    std::printf("\n%s\n", failures == 0 ? "All stress tests passed."
                                        : "STRESS TESTS FAILED");
    return failures;
}
