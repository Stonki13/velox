// Stress tests for Predictive Contact Sweeping — each case targets a classic
// CCD failure mode. Exits nonzero if any invariant is violated.
#include <velox/velox.h>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

static int failures = 0;
static void check(bool ok, const char* name) {
    std::printf("%-44s %s\n", name, ok ? "PASS" : "FAIL");
    if (!ok) ++failures;
}

template <typename F>
static bool throwsException(F&& action) {
    try { action(); }
    catch (const std::exception&) { return true; }
    return false;
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
    std::vector<velox::BodyId> bodies;
    bodies.reserve(N);
    for (int x = 0; x < 5; ++x)
        for (int y = 0; y < 5; ++y)
            for (int z = 0; z < 5; ++z)
                bodies.push_back(w.addSphere(
                    {x * 1.01f, 0.5f + y * 1.01f, z * 1.01f}, 0.5f, 1.0f));
    for (int i = 0; i < 300; ++i) w.step(1.0f / 60.0f); // 5 s to settle
    bool ok = true;
    for (velox::BodyId id : bodies)
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

// 16. Convex hull: an octahedron dropped onto a bounded box must come to rest
// on a face without sinking, and a raycast must hit it. The bounded pedestal
// also catches hull-AABB regressions that a boundless plane cannot expose.
static void testHull() {
    velox::World w;
    bool rejectedEmpty = false;
    try { w.addConvexHull({}, {}, 1.0f); }
    catch (const std::invalid_argument&) { rejectedEmpty = true; }
    w.addStaticPlane({0, 1, 0}, 0.0f);
    w.addBox({0, 0.5f, 0}, {5.0f, 0.5f, 5.0f}, 0.0f);
    std::vector<velox::Vec3> octa = {
        {0.6f, 0, 0}, {-0.6f, 0, 0}, {0, 0.6f, 0}, {0, -0.6f, 0}, {0, 0, 0.6f}, {0, 0, -0.6f}};
    auto hull = w.addConvexHull({0, 2.0f, 0}, octa, 1.0f);
    for (int i = 0; i < 600; ++i) w.step(1.0f / 60.0f);
    const auto& b = w.body(hull);
    // Pedestal top is y=1. A face/edge rest puts the center in (1.3, 1.6).
    bool ok = rejectedEmpty && b.position.y > 1.25f && b.position.y < 1.65f &&
              velox::length(b.velocity) < 0.1f;
    auto hit = w.rayCast({b.position.x, 5, b.position.z}, {0, -1, 0}, 10.0f);
    ok &= hit.hit && hit.body == hull;
    check(ok, "convex hull vs box (broad phase, rest, raycast)");
}

// 17. Hinge motor: drives the door to its target angular speed.
static void testMotor() {
    velox::World w;
    w.gravity = {0, 0, 0};
    auto post = w.addBox({0, 2, 0}, {0.05f, 1.0f, 0.05f}, 0.0f);
    auto door = w.addBox({0.55f, 2, 0}, {0.5f, 0.9f, 0.05f}, 2.0f);
    auto j = w.addHingeJoint(post, door, {0.05f, 2, 0}, {0, 1, 0});
    w.joint(j).enableMotor = true;
    w.joint(j).motorSpeed = -3.0f; // door spins about +Y relative to post
    w.joint(j).maxMotorTorque = 50.0f;
    for (int i = 0; i < 240; ++i) w.step(1.0f / 60.0f);
    float wy = w.body(door).angularVelocity.y;
    check(std::fabs(wy - 3.0f) < 0.2f, "hinge motor (reaches target speed)");
}

// 18. Hinge limit: a motored door must stop at the limit angle.
static void testLimit() {
    velox::World w;
    w.gravity = {0, 0, 0};
    auto post = w.addBox({0, 2, 0}, {0.05f, 1.0f, 0.05f}, 0.0f);
    auto door = w.addBox({0.55f, 2, 0}, {0.5f, 0.9f, 0.05f}, 2.0f);
    auto j = w.addHingeJoint(post, door, {0.05f, 2, 0}, {0, 1, 0});
    w.joint(j).enableMotor = true;
    w.joint(j).motorSpeed = -2.0f;
    w.joint(j).maxMotorTorque = 20.0f;
    w.joint(j).enableLimit = true;
    w.joint(j).lowerLimit = -2.0f;
    w.joint(j).upperLimit = 1.0f; // ~57 degrees
    float maxAngle = 0.0f;
    for (int i = 0; i < 360; ++i) {
        w.step(1.0f / 60.0f);
        maxAngle = std::fmax(maxAngle, std::fabs(w.hingeAngle(j)));
    }
    // The motor pushes into the limit; the angle must never blow past it.
    check(maxAngle < 1.15f && maxAngle > 0.85f, "hinge limit (motor stops at limit)");
}

// 19. Contact events: exactly one begin event when a ball lands, none while
// it rests, and the event carries a sane impulse.
static void testEvents() {
    velox::World w;
    auto floor = w.addStaticPlane({0, 1, 0}, 0.0f);
    auto ball = w.addSphere({0, 2.0f, 0}, 0.5f, 1.0f);
    int began = 0;
    float impulse = 0.0f;
    for (int i = 0; i < 300; ++i) {
        w.step(1.0f / 60.0f);
        for (const auto& ev : w.contactEvents()) {
            bool ours = (ev.a == ball && ev.b == floor) || (ev.a == floor && ev.b == ball);
            if (ours && ev.type == velox::ContactEventType::Begin) {
                ++began;
                impulse = std::fmax(impulse, ev.impulse);
            }
        }
    }
    // Restitution can cause a couple of touch-bounce-touch cycles, but resting
    // contact must not spam events every frame.
    check(began >= 1 && began <= 6 && impulse > 0.0f,
          "contact events (begin fires once per touch)");
}

// 20. Deep hull overlap: the minimum translation is vertical (1.2 m) while
// the center delta is diagonal. EPA must recover the face normal instead of
// using the old center-based fallback, which would incorrectly add X drift.
static void testDeepHullPenetration() {
    velox::World w;
    w.gravity = {0, 0, 0};
    std::vector<velox::Vec3> cube = {
        {-1, -1, -1}, {1, -1, -1}, {-1, 1, -1}, {1, 1, -1},
        {-1, -1, 1},  {1, -1, 1},  {-1, 1, 1},  {1, 1, 1}};
    w.addConvexHull({0, 0, 0}, cube, 0.0f);
    auto moving = w.addConvexHull({0.2f, 0.8f, 0}, cube, 1.0f);
    for (int i = 0; i < 10; ++i) w.step(1.0f / 60.0f);
    const auto& b = w.body(moving);
    bool ok = std::fabs(b.position.x - 0.2f) < 0.05f &&
              b.position.y > 1.95f && std::fabs(b.position.z) < 0.05f;
    check(ok, "EPA deep hull overlap (exact minimum translation)");
}

// 21. Unequal-mass dynamic CCD must preserve linear momentum and the
// center-of-mass trajectory while preventing the pair from crossing.
static void testDynamicCcdMomentum() {
    velox::World w;
    w.gravity = {0, 0, 0};
    w.substeps = 1;
    auto a = w.addSphere({-5, 0, 0}, 0.5f, 1.0f);
    auto b = w.addSphere({5, 0, 0}, 0.5f, 3.0f);
    w.body(a).velocity = {1000, 0, 0};
    w.body(b).velocity = {-500, 0, 0};
    w.body(a).restitution = w.body(b).restitution = 0.0f;
    const float initialMomentum = -500.0f;
    const float initialCom = 2.5f;
    const float dt = 1.0f / 60.0f;
    w.step(dt);
    const auto& ba = w.body(a);
    const auto& bb = w.body(b);
    float momentum = ba.velocity.x * 1.0f + bb.velocity.x * 3.0f;
    float com = (ba.position.x + bb.position.x * 3.0f) * 0.25f;
    float expectedCom = initialCom + (initialMomentum * 0.25f) * dt;
    bool ok = ba.position.x <= bb.position.x &&
              std::fabs(momentum - initialMomentum) < 0.05f &&
              std::fabs(com - expectedCom) < 0.02f;
    check(ok, "dynamic CCD (symmetric, momentum + COM preserved)");
}

// 22. Friction must be isotropic in the contact plane. A diagonal slide on a
// flat plane should lose both tangent components equally and settle without
// veering toward either solver basis axis.
static void testTwoAxisFriction() {
    velox::World w;
    auto floor = w.addStaticPlane({0, 1, 0}, 0.0f);
    auto box = w.addBox({0, 0.5f, 0}, {0.5f, 0.5f, 0.5f}, 1.0f);
    w.body(floor).friction = 0.8f;
    w.body(box).friction = 0.8f;
    w.body(box).restitution = 0.0f;
    w.body(box).velocity = {4.0f, 0, 4.0f};
    for (int i = 0; i < 180; ++i) w.step(1.0f / 60.0f);
    const auto& b = w.body(box);
    float horizontalSpeed = std::sqrt(b.velocity.x * b.velocity.x +
                                      b.velocity.z * b.velocity.z);
    bool ok = horizontalSpeed < 0.15f &&
              std::fabs(b.position.x - b.position.z) < 0.05f;
    check(ok, "two-axis friction (isotropic diagonal slide)");
}

// 23. Public inputs and mutable runtime state must fail before unsafe geometry
// reaches GJK/BVH/CUDA code, and failed additions must not change bodyCount().
static void testValidation() {
    velox::World w;
    size_t initialCount = w.bodyCount();
    bool ok = true;
    ok &= throwsException([&] { w.addSphere({}, -1.0f, 1.0f); });
    ok &= throwsException([&] { w.addBox({}, {1, 0, 1}, 1.0f); });
    ok &= throwsException([&] { w.addCapsule({}, 0.5f, -0.1f, 1.0f); });
    ok &= throwsException([&] { w.addStaticPlane({}, 0.0f); });
    std::vector<velox::Vec3> flat = {{0, 0, 0}, {1, 0, 0}, {0, 0, 1}, {1, 0, 1}};
    ok &= throwsException([&] { w.addConvexHull({}, flat, 1.0f); });
    ok &= throwsException([&] {
        w.addStaticMesh({{0, 0, 0}, {1, 0, 0}, {0, 1, 0}}, {0, 1, 9});
    });
    ok &= throwsException([&] {
        w.addStaticMesh({{0, 0, 0}, {1, 0, 0}, {2, 0, 0}}, {0, 1, 2});
    });
    ok &= w.bodyCount() == initialCount;

    constexpr float r = 0.5f, halfHeight = 1.0f, mass = 3.0f;
    auto capsule = w.addCapsule({}, r, halfHeight, mass);
    float cylinderWeight = 2.0f * halfHeight;
    float sphereWeight = (4.0f / 3.0f) * r;
    float cylinderMass = mass * cylinderWeight /
                         (cylinderWeight + sphereWeight);
    float sphereMass = mass - cylinderMass;
    float expectedIy = 0.5f * cylinderMass * r * r +
                       0.4f * sphereMass * r * r;
    float expectedIx = cylinderMass *
                           (3.0f * r * r +
                            4.0f * halfHeight * halfHeight) /
                           12.0f +
                       sphereMass *
                           (0.4f * r * r + 0.75f * halfHeight * r +
                            halfHeight * halfHeight);
    const velox::Body& capsuleBody = w.body(capsule);
    ok &= std::fabs(capsuleBody.invInertia.x - 1.0f / expectedIx) < 1e-5f;
    ok &= std::fabs(capsuleBody.invInertia.y - 1.0f / expectedIy) < 1e-5f;

    auto zeroCoreCapsule = w.addCapsule({}, r, 0.0f, mass);
    auto equalSphere = w.addSphere({}, r, mass);
    ok &= velox::length(w.body(zeroCoreCapsule).invInertia -
                        w.body(equalSphere).invInertia) < 1e-6f;

    auto ground = w.addStaticPlane({0, 1, 0}, 0.0f);
    auto body = w.addSphere({0, 2, 0}, 0.5f, 1.0f);
    ok &= throwsException([&] { (void)w.body(velox::BodyId::make(9999, 0)); });
    ok &= throwsException([&] { (void)w.joint(velox::JointId::make(9999, 0)); });
    ok &= throwsException([&] { w.addBallJoint(body, body, {}); });
    ok &= throwsException([&] { w.addHingeJoint(ground, body, {}, {}); });
    ok &= throwsException([&] { (void)w.rayCast({}, {}, 1.0f); });
    std::vector<velox::BodyId> overlaps;
    ok &= throwsException([&] { w.overlapSphere({}, 0.0f, overlaps); });
    ok &= throwsException([&] { w.step(-1.0f); });
    ok &= throwsException([&] { w.step(std::numeric_limits<float>::quiet_NaN()); });
    w.substeps = 0;
    ok &= throwsException([&] { w.step(1.0f / 60.0f); });
    w.substeps = 4;
    w.body(body).velocity.x = std::numeric_limits<float>::infinity();
    ok &= throwsException([&] { w.step(1.0f / 60.0f); });
    w.body(body).velocity = {};
    w.step(0.0f); // explicitly supported no-op
    check(ok, "public API validation (geometry, IDs, state, timestep)");
}

// 24. Kinematic bodies follow prescribed velocities, ignore gravity and
// impulses, but still wake and push dynamic bodies. Motion-type transitions
// retain the body's original mass/inertia for a later return to Dynamic.
static void testMotionTypes() {
    velox::World w;
    w.addStaticPlane({0, 1, 0}, 0.0f);
    auto mover = w.addBox({-2, 0.5f, 0}, {0.5f, 0.5f, 0.5f}, 2.0f);
    auto ball = w.addSphere({0, 0.5f, 0}, 0.5f, 1.0f);
    w.setMotionType(mover, velox::MotionType::Kinematic);
    w.body(mover).velocity = {4, 0, 0};
    for (int i = 0; i < 60; ++i) w.step(1.0f / 60.0f);
    const auto movingPosition = w.body(mover).position;
    bool ok = w.motionType(mover) == velox::MotionType::Kinematic &&
              std::fabs(movingPosition.x - 2.0f) < 0.05f &&
              std::fabs(movingPosition.y - 0.5f) < 0.02f &&
              std::fabs(w.body(mover).velocity.x - 4.0f) < 1e-4f &&
              w.body(ball).position.x > 1.0f;

    w.setMotionType(mover, velox::MotionType::Static);
    velox::Vec3 staticPosition = w.body(mover).position;
    w.step(1.0f / 60.0f);
    ok &= velox::length(w.body(mover).position - staticPosition) < 1e-6f;
    w.body(mover).position.y = 2.0f;
    w.setMotionType(mover, velox::MotionType::Dynamic);
    w.step(1.0f / 60.0f);
    ok &= w.body(mover).velocity.y < 0.0f;

    auto massless = w.addBox({10, 1, 0}, {0.5f, 0.5f, 0.5f}, 0.0f);
    w.setMotionType(massless, velox::MotionType::Kinematic);
    ok &= throwsException([&] { w.setMotionType(massless, velox::MotionType::Dynamic); });
    check(ok, "motion types (static, kinematic, dynamic transitions)");
}

// 25. Controlled state changes must preserve the dynamic-body invariants:
// forces accumulate for one public step, impulses are immediate, off-center
// application produces angular motion, and per-body integration controls work.
static void testForcesAndStateApi() {
    velox::World w;
    w.gravity = {0, 0, 0};
    auto body = w.addBox({0, 5, 0}, {0.5f, 0.5f, 0.5f}, 2.0f);

    w.addForce(body, {12, 0, 0});
    w.step(0.5f);
    bool ok = std::fabs(w.body(body).velocity.x - 3.0f) < 1e-4f;
    w.step(0.5f);
    ok &= std::fabs(w.body(body).velocity.x - 3.0f) < 1e-4f;

    w.addLinearImpulse(body, {2, 0, 0});
    ok &= std::fabs(w.body(body).velocity.x - 4.0f) < 1e-4f;
    velox::Vec3 point = w.body(body).position + velox::Vec3{1, 0, 0};
    w.addImpulseAtPoint(body, {0, 2, 0}, point);
    ok &= w.body(body).angularVelocity.z > 0.1f;

    w.setLinearVelocity(body, {10, 0, 0});
    w.setAngularVelocity(body, {});
    w.body(body).linearDamping = 1.0f;
    w.step(1.0f);
    ok &= std::fabs(w.body(body).velocity.x - 4.096f) < 1e-3f;

    w.gravity = {0, -9.81f, 0};
    w.body(body).linearDamping = 0.0f;
    w.body(body).gravityScale = 0.0f;
    w.setTransform(body, {2, 10, 3}, {0, 0, 0, 2});
    w.setLinearVelocity(body, {});
    w.step(0.25f);
    ok &= std::fabs(w.body(body).position.y - 10.0f) < 1e-5f;
    ok &= std::fabs(w.body(body).orientation.w - 1.0f) < 1e-5f;

    auto fixed = w.addBox({20, 0, 0}, {1, 1, 1}, 0.0f);
    ok &= throwsException([&] { w.addForce(fixed, {1, 0, 0}); });
    ok &= throwsException([&] { w.addLinearImpulse(fixed, {1, 0, 0}); });
    ok &= throwsException([&] { w.setLinearVelocity(fixed, {1, 0, 0}); });
    ok &= throwsException([&] {
        w.setTransform(body, {}, {0, 0, 0, 0});
    });
    check(ok, "forces and state API (impulses, damping, gravity scale)");
}

// 26. Public handles survive dense-array swaps, while removed handles are
// rejected even after their slots are reused. Body removal also owns the
// lifetime of attached joints so no constraint can retain a dead endpoint.
static void testStableHandlesAndRemoval() {
    velox::World w;
    w.gravity = {0, 0, 0};
    auto a = w.addSphere({-4, 0, 0}, 0.5f, 1.0f);
    auto removed = w.addSphere({0, 0, 0}, 0.5f, 1.0f);
    auto moved = w.addSphere({4, 0, 0}, 0.5f, 1.0f);
    auto retainedJoint = w.addDistanceJoint(a, moved, {-4, 0, 0}, {4, 0, 0});

    w.removeBody(removed);
    bool ok = w.bodyCount() == 2 && !w.isValid(removed) && w.isValid(moved) &&
              w.isValid(retainedJoint);
    ok &= std::fabs(w.body(moved).position.x - 4.0f) < 1e-6f;
    ok &= throwsException([&] { (void)w.body(removed); });
    w.step(1.0f / 60.0f); // remapped joint endpoints remain usable

    auto replacement = w.addSphere({0, 0, 0}, 0.5f, 1.0f);
    ok &= replacement != removed && w.isValid(replacement) && !w.isValid(removed);
    ok &= throwsException([&] { w.setLinearVelocity(removed, {1, 0, 0}); });

    auto disposableJoint = w.addBallJoint(moved, replacement, {2, 0, 0});
    w.removeJoint(disposableJoint);
    ok &= !w.isValid(disposableJoint);
    ok &= throwsException([&] { (void)w.joint(disposableJoint); });
    auto reusedJoint = w.addBallJoint(moved, replacement, {2, 0, 0});
    ok &= reusedJoint != disposableJoint && w.isValid(reusedJoint);

    w.removeBody(a);
    ok &= !w.isValid(a) && !w.isValid(retainedJoint) && w.isValid(reusedJoint);

    auto hit = w.rayCast({10, 0, 0}, {-1, 0, 0}, 20.0f);
    ok &= hit.hit && w.isValid(hit.body);
    std::vector<velox::BodyId> overlaps;
    w.overlapSphere({0, 0, 0}, 1.0f, overlaps);
    ok &= !overlaps.empty() && w.isValid(overlaps.front());
    check(ok, "stable handles (removal, reuse, joint and query remap)");
}

// 27. Category/mask tests are symmetric and sensors report a complete event
// lifecycle without changing velocity, position, sleeping, or CCD response.
static void testFiltersSensorsAndEventPhases() {
    velox::World filtered;
    filtered.gravity = {0, 0, 0};
    auto wall = filtered.addBox({0, 0, 0}, {0.5f, 2, 2}, 0.0f);
    auto ghost = filtered.addSphere({-3, 0, 0}, 0.25f, 1.0f);
    filtered.body(wall).categoryBits = 2u;
    filtered.body(wall).maskBits = 2u;
    filtered.body(ghost).categoryBits = 1u;
    filtered.body(ghost).maskBits = 1u;
    filtered.setLinearVelocity(ghost, {12, 0, 0});
    filtered.step(0.5f);
    bool ok = filtered.body(ghost).position.x > 2.9f &&
              std::fabs(filtered.body(ghost).velocity.x - 12.0f) < 1e-4f &&
              filtered.contactEvents().empty();

    velox::World triggers;
    triggers.gravity = {0, 0, 0};
    auto sensor = triggers.addBox({0, 0, 0}, {0.5f, 1, 1}, 0.0f);
    auto visitor = triggers.addSphere({-3, 0, 0}, 0.25f, 1.0f);
    triggers.body(sensor).sensor = 1;
    triggers.setLinearVelocity(visitor, {8, 0, 0});
    int began = 0, persisted = 0, ended = 0;
    for (int i = 0; i < 90; ++i) {
        triggers.step(1.0f / 60.0f);
        for (const auto& ev : triggers.contactEvents()) {
            bool ours = (ev.a == sensor && ev.b == visitor) ||
                        (ev.a == visitor && ev.b == sensor);
            if (!ours || !ev.sensor) continue;
            if (ev.type == velox::ContactEventType::Begin) ++began;
            else if (ev.type == velox::ContactEventType::Persist) ++persisted;
            else if (ev.type == velox::ContactEventType::End) ++ended;
        }
    }
    ok &= began == 1 && persisted > 0 && ended == 1;
    ok &= std::fabs(triggers.body(visitor).velocity.x - 8.0f) < 1e-4f;
    ok &= triggers.body(visitor).position.x > 8.9f;
    check(ok, "filters and sensors (begin, persist, end without response)");
}

// 28. Cone/twist joints provide a ball anchor plus independent ragdoll-style
// swing and axial limits. Start outside each envelope and require recovery.
static void testConeTwistJoint() {
    velox::World swingWorld;
    swingWorld.gravity = {0, 0, 0};
    auto swingBase = swingWorld.addBox({}, {0.5f, 0.5f, 0.5f}, 0.0f);
    auto swingBody = swingWorld.addBox({}, {0.4f, 0.7f, 0.3f}, 1.0f);
    auto swingJoint = swingWorld.addConeTwistJoint(
        swingBase, swingBody, {}, {0, 1, 0});
    swingWorld.joint(swingJoint).enableSwingLimit = true;
    swingWorld.joint(swingJoint).swingLimit = 0.35f;
    swingWorld.setTransform(swingBody, {}, velox::fromAxisAngle({1, 0, 0}, 1.0f));
    for (int i = 0; i < 120; ++i) swingWorld.step(1.0f / 60.0f);

    velox::World twistWorld;
    twistWorld.gravity = {0, 0, 0};
    auto twistBase = twistWorld.addBox({}, {0.5f, 0.5f, 0.5f}, 0.0f);
    auto twistBody = twistWorld.addBox({}, {0.4f, 0.7f, 0.3f}, 1.0f);
    auto twistJoint = twistWorld.addConeTwistJoint(
        twistBase, twistBody, {}, {0, 1, 0});
    twistWorld.joint(twistJoint).enableTwistLimit = true;
    twistWorld.joint(twistJoint).lowerTwistLimit = -0.3f;
    twistWorld.joint(twistJoint).upperTwistLimit = 0.3f;
    twistWorld.setTransform(twistBody, {}, velox::fromAxisAngle({0, 1, 0}, 1.0f));
    for (int i = 0; i < 120; ++i) twistWorld.step(1.0f / 60.0f);

    bool ok = swingWorld.coneSwingAngle(swingJoint) < 0.38f &&
              std::fabs(twistWorld.coneTwistAngle(twistJoint)) < 0.33f;
    check(ok, "cone/twist joint (swing cone and axial limits)");
}

// 29. Compound bodies keep several locally transformed convex children under
// one dynamic body and one stable public handle. Contacts must solve against
// the parent frame, while queries and CCD still inspect every child.
static void testCompoundBody() {
    velox::CompoundShape left;
    left.shape = velox::ShapeType::Sphere;
    left.localPosition = {-1, 0, 0};
    left.radius = 0.5f;
    velox::CompoundShape right = left;
    right.localPosition = {1, 0, 0};

    velox::World w;
    w.addStaticPlane({0, 1, 0}, 0.0f);
    auto compound = w.addCompound({0, 3, 0}, {left, right}, 2.0f);
    for (int i = 0; i < 360; ++i) w.step(1.0f / 60.0f);
    bool ok = w.body(compound).position.y > 0.42f;

    velox::Vec3 childCenter = w.body(compound).position +
        velox::rotate(w.body(compound).orientation, right.localPosition);
    auto hit = w.rayCast(childCenter + velox::Vec3{0, 3, 0},
                         {0, -1, 0}, 10.0f);
    ok &= hit.hit && hit.body == compound;
    std::vector<velox::BodyId> overlaps;
    w.overlapSphere(childCenter, 0.2f, overlaps);
    ok &= overlaps.size() == 1 && overlaps[0] == compound;

    size_t count = w.bodyCount();
    velox::CompoundShape invalid;
    invalid.shape = velox::ShapeType::Plane;
    ok &= throwsException([&] { w.addCompound({}, {invalid}, 1.0f); });
    ok &= w.bodyCount() == count;

    velox::World bullet;
    bullet.gravity = {0, 0, 0};
    bullet.addStaticPlane({0, 1, 0}, 0.0f);
    velox::CompoundShape fastBox;
    fastBox.shape = velox::ShapeType::Box;
    fastBox.localPosition = {1, 0, 0};
    fastBox.localOrientation = velox::fromAxisAngle({0, 0, 1}, 0.4f);
    fastBox.halfExtents = {0.2f, 0.2f, 0.2f};
    auto projectile = bullet.addCompound({0, 2, 0}, {fastBox}, 1.0f);
    bullet.setLinearVelocity(projectile, {0, -1500, 0});
    bullet.setAngularVelocity(projectile, {20, 30, 10});
    for (int i = 0; i < 10; ++i) {
        bullet.step(1.0f / 60.0f);
        velox::Vec3 center = bullet.body(projectile).position +
            velox::rotate(bullet.body(projectile).orientation,
                          fastBox.localPosition);
        ok &= center.y > -0.5f;
    }
    check(ok, "compound body (local children, parent queries, CCD)");
}

// 30. Cylinder and center-of-mass-centered cone support runs through the same
// GJK/EPA/CCD path as other convexes. Heightfields build validated terrain
// triangles into the mesh BVH rather than requiring hand-authored indices.
static void testCylinderConeAndHeightfield() {
    velox::World primitives;
    primitives.addStaticPlane({0, 1, 0}, 0.0f);
    auto cylinder = primitives.addCylinder({-2, 4, 0}, 0.6f, 1.0f, 1.0f);
    auto cone = primitives.addCone({2, 4, 0}, 0.8f, 2.0f, 1.0f);
    for (int i = 0; i < 300; ++i) primitives.step(1.0f / 60.0f);
    bool ok = primitives.body(cylinder).position.y > 0.85f &&
              primitives.body(cone).position.y > 0.35f;
    auto cylinderHit = primitives.rayCast({-2, 5, 0}, {0, -1, 0}, 10.0f);
    auto coneHit = primitives.rayCast({2, 5, 0}, {0, -1, 0}, 10.0f);
    ok &= cylinderHit.hit && cylinderHit.body == cylinder;
    ok &= coneHit.hit && coneHit.body == cone;

    velox::World terrain;
    std::vector<float> heights = {
        1.0f, 1.0f, 1.0f, 1.0f,
        1.0f, 0.0f, 0.0f, 1.0f,
        1.0f, 0.0f, 0.0f, 1.0f,
        1.0f, 1.0f, 1.0f, 1.0f};
    auto heightfield = terrain.addStaticHeightfield(
        4, 4, 1.0f, heights, {-1.5f, 0, -1.5f});
    auto ball = terrain.addSphere({0, 3, 0}, 0.35f, 1.0f);
    for (int i = 0; i < 300; ++i) terrain.step(1.0f / 60.0f);
    ok &= terrain.body(ball).position.y > 0.3f;
    auto terrainHit = terrain.rayCast({0, 5, 0}, {0, -1, 0}, 10.0f);
    ok &= terrainHit.hit &&
          (terrainHit.body == ball || terrainHit.body == heightfield);
    size_t count = terrain.bodyCount();
    ok &= throwsException([&] {
        terrain.addStaticHeightfield(3, 3, 1.0f, {0, 0, 0});
    });
    ok &= terrain.bodyCount() == count;

    velox::World fast;
    fast.gravity = {0, 0, 0};
    fast.addStaticPlane({0, 1, 0}, 0.0f);
    auto projectile = fast.addCone({0, 2, 0}, 0.2f, 0.5f, 1.0f);
    fast.setLinearVelocity(projectile, {0, -1500, 0});
    fast.setAngularVelocity(projectile, {30, 10, 20});
    for (int i = 0; i < 10; ++i) {
        fast.step(1.0f / 60.0f);
        ok &= fast.body(projectile).position.y > -0.5f;
    }
    check(ok, "cylinder, cone, and heightfield (GJK, BVH, CCD)");
}

// 31. Scene queries share symmetric category/mask filtering, sensor policy,
// and ignored stable handles. Convex casts use conservative advancement and
// return distance/fraction independent of the simulation timestep.
static void testFilteredQueriesAndShapeCasts() {
    velox::World w;
    w.gravity = {0, 0, 0};
    auto sensor = w.addSphere({2, 0, 0}, 0.5f, 0.0f);
    w.body(sensor).sensor = 1;
    auto target = w.addBox({5, 0, 0}, {0.5f, 1, 1}, 0.0f);
    w.body(target).categoryBits = 2u;
    w.body(target).maskBits = 4u;

    auto defaultRay = w.rayCast({0, 0, 0}, {1, 0, 0}, 10.0f);
    velox::QueryFilter filter;
    filter.categoryBits = 4u;
    filter.maskBits = 2u;
    filter.includeSensors = false;
    auto filteredRay = w.rayCast({0, 0, 0}, {1, 0, 0}, 10.0f, filter);
    bool ok = defaultRay.hit && defaultRay.body == sensor &&
              filteredRay.hit && filteredRay.body == target;

    std::vector<velox::BodyId> overlaps;
    w.overlapBox({5, 0, 0}, {0.75f, 0.75f, 0.75f}, {}, overlaps, filter);
    ok &= overlaps.size() == 1 && overlaps[0] == target;
    w.overlapCapsule({5, 0, 0}, 0.3f, 0.8f, {}, overlaps, filter);
    ok &= overlaps.size() == 1 && overlaps[0] == target;
    std::vector<velox::Vec3> queryHull = {
        {-0.25f, -0.25f, -0.25f}, {0.25f, -0.25f, -0.25f},
        {-0.25f,  0.25f, -0.25f}, {0.25f,  0.25f, -0.25f},
        {-0.25f, -0.25f,  0.25f}, {0.25f, -0.25f,  0.25f},
        {-0.25f,  0.25f,  0.25f}, {0.25f,  0.25f,  0.25f}};
    w.overlapConvexHull({5, 0, 0}, queryHull, {}, overlaps, filter);
    ok &= overlaps.size() == 1 && overlaps[0] == target;

    auto sphereHit = w.sphereCast({0, 0, 0}, 0.25f, {1, 0, 0}, 10.0f, filter);
    auto boxHit = w.boxCast({0, 0, 0}, {0.25f, 0.25f, 0.25f}, {},
                            {1, 0, 0}, 10.0f, filter);
    auto capsuleHit = w.capsuleCast({0, 0, 0}, 0.25f, 0.5f, {},
                                    {1, 0, 0}, 10.0f, filter);
    auto hullHit = w.convexHullCast({0, 0, 0}, queryHull, {},
                                    {1, 0, 0}, 10.0f, filter);
    ok &= sphereHit.hit && sphereHit.body == target &&
          std::fabs(sphereHit.distance - 4.25f) < 0.02f;
    ok &= boxHit.hit && boxHit.body == target &&
          std::fabs(boxHit.distance - 4.25f) < 0.02f;
    ok &= capsuleHit.hit && capsuleHit.body == target &&
          std::fabs(capsuleHit.distance - 4.25f) < 0.02f;
    ok &= hullHit.hit && hullHit.body == target &&
          std::fabs(hullHit.distance - 4.25f) < 0.02f;
    ok &= std::fabs(sphereHit.fraction - 0.425f) < 0.003f;

    filter.ignoredBody = target;
    ok &= !w.sphereCast({0, 0, 0}, 0.25f, {1, 0, 0}, 10.0f, filter).hit;

    velox::World terrain;
    auto ground = terrain.addStaticHeightfield(
        2, 2, 5.0f, {0, 0, 0, 0}, {-2.5f, 0, -2.5f});
    auto groundHit = terrain.sphereCast({0, 5, 0}, 0.5f, {0, -1, 0}, 10.0f);
    auto hullGroundHit = terrain.convexHullCast(
        {0, 5, 0}, queryHull, {}, {0, -1, 0}, 10.0f);
    ok &= groundHit.hit && groundHit.body == ground &&
          std::fabs(groundHit.distance - 4.5f) < 0.02f;
    ok &= hullGroundHit.hit && hullGroundHit.body == ground &&
          std::fabs(hullGroundHit.distance - 4.75f) < 0.02f;
    ok &= throwsException([&] {
        (void)terrain.boxCast({}, {1, 1, 1}, {}, {}, 10.0f);
    });
    ok &= throwsException([&] {
        terrain.overlapConvexHull({}, {{0, 0, 0}, {1, 0, 0},
                                      {0, 1, 0}, {1, 1, 0}}, {}, overlaps);
    });
    check(ok, "filtered queries and casts (sphere, box, capsule, hull)");
}

// 32. Material coefficients are resolved once per contact for both backends.
// Combine modes, body-local anisotropy, angular resistance, and host contact
// modification must all affect the same shared solver rows.
static void testMaterialsAndContactModification() {
    auto resolvedFriction = [](velox::MaterialCombineMode mode) {
        velox::World w;
        w.gravity = {0, 0, 0};
        auto floor = w.addStaticPlane({0, 1, 0}, 0.0f);
        auto ball = w.addSphere({0, 0.5f, 0}, 0.5f, 1.0f);
        w.body(floor).friction = 0.25f;
        w.body(ball).friction = 0.5f;
        w.body(floor).frictionCombine = w.body(ball).frictionCombine = mode;
        float resolved = -1.0f;
        w.setContactModifier([&](velox::ContactModifyData& contact) {
            resolved = contact.friction1;
        });
        w.step(1.0f / 60.0f);
        return resolved;
    };
    bool allCombinesOk =
        std::fabs(resolvedFriction(velox::MaterialCombineMode::Average) - 0.375f) < 1e-5f &&
        std::fabs(resolvedFriction(velox::MaterialCombineMode::GeometricMean) -
                  std::sqrt(0.125f)) < 1e-5f &&
        std::fabs(resolvedFriction(velox::MaterialCombineMode::Minimum) - 0.25f) < 1e-5f &&
        std::fabs(resolvedFriction(velox::MaterialCombineMode::Multiply) - 0.125f) < 1e-5f &&
        std::fabs(resolvedFriction(velox::MaterialCombineMode::Maximum) - 0.5f) < 1e-5f;

    auto bounceVelocity = [](velox::MaterialCombineMode sphereMode) {
        velox::World w;
        w.gravity = {0, 0, 0};
        auto floor = w.addStaticPlane({0, 1, 0}, 0.0f);
        auto ball = w.addSphere({0, 1, 0}, 0.5f, 1.0f);
        w.body(floor).restitution = 0.0f;
        w.body(ball).restitution = 1.0f;
        w.body(ball).restitutionCombine = sphereMode;
        w.setLinearVelocity(ball, {0, -5, 0});
        for (int i = 0; i < 15; ++i) w.step(1.0f / 60.0f);
        return w.body(ball).velocity.y;
    };
    float minimumBounce = bounceVelocity(velox::MaterialCombineMode::Minimum);
    float maximumBounce = bounceVelocity(velox::MaterialCombineMode::Maximum);
    bool combineOk = allCombinesOk && minimumBounce < 0.1f && maximumBounce > 4.0f;

    velox::World directional;
    auto floor = directional.addStaticPlane({0, 1, 0}, 0.0f);
    directional.body(floor).friction = 1.0f;
    auto alongX = directional.addBox({0, 0.5f, -50}, {0.5f, 0.5f, 0.5f}, 1.0f);
    auto alongZ = directional.addBox({-50, 0.5f, 0}, {0.5f, 0.5f, 0.5f}, 1.0f);
    for (auto id : {alongX, alongZ}) {
        directional.body(id).friction = 1.0f;
        directional.body(id).frictionScale = {0, 1, 4};
    }
    directional.setLinearVelocity(alongX, {10, 0, 0});
    directional.setLinearVelocity(alongZ, {0, 0, 10});
    for (int i = 0; i < 120; ++i) directional.step(1.0f / 60.0f);
    bool anisotropyOk = directional.body(alongX).velocity.x > 9.0f &&
                        std::fabs(directional.body(alongZ).velocity.z) < 0.5f;

    auto angularSpeed = [](bool rolling, float coefficient) {
        velox::World w;
        auto ground = w.addStaticPlane({0, 1, 0}, 0.0f);
        auto sphere = w.addSphere({0, 0.5f, 0}, 0.5f, 1.0f);
        w.body(ground).friction = w.body(sphere).friction = 0.0f;
        w.body(ground).rollingFriction = w.body(sphere).rollingFriction = coefficient;
        w.body(ground).spinningFriction = w.body(sphere).spinningFriction = coefficient;
        w.setAngularVelocity(sphere, rolling ? velox::Vec3{20, 0, 0}
                                             : velox::Vec3{0, 20, 0});
        for (int i = 0; i < 60; ++i) w.step(1.0f / 60.0f);
        return rolling ? std::fabs(w.body(sphere).angularVelocity.x)
                       : std::fabs(w.body(sphere).angularVelocity.y);
    };
    float rollingFree = angularSpeed(true, 0.0f);
    float rollingResisted = angularSpeed(true, 0.5f);
    float spinningFree = angularSpeed(false, 0.0f);
    float spinningResisted = angularSpeed(false, 0.5f);
    bool angularOk = rollingFree > 19.0f && rollingResisted < 0.5f &&
                     spinningFree > 19.0f && spinningResisted < 0.5f;

    velox::World disabled;
    auto disabledFloor = disabled.addStaticPlane({0, 1, 0}, 0.0f);
    auto falling = disabled.addSphere({0, 1, 0}, 0.5f, 1.0f);
    int callbackCount = 0;
    bool handlesOk = true;
    disabled.setContactModifier([&](velox::ContactModifyData& contact) {
        ++callbackCount;
        handlesOk &= (contact.a == falling && contact.b == disabledFloor) ||
                     (contact.a == disabledFloor && contact.b == falling);
        contact.enabled = false;
    });
    for (int i = 0; i < 40; ++i) disabled.step(1.0f / 60.0f);
    bool disableOk = callbackCount > 0 && handlesOk &&
                     disabled.body(falling).position.y < -0.5f;

    velox::World modified;
    modified.gravity = {0, 0, 0};
    modified.addStaticPlane({0, 1, 0}, 0.0f);
    auto modifiedBall = modified.addSphere({0, 1, 0}, 0.5f, 1.0f);
    modified.setLinearVelocity(modifiedBall, {0, -5, 0});
    modified.setContactModifier([](velox::ContactModifyData& contact) {
        contact.restitution = 1.0f;
        contact.friction1 = contact.friction2 = 0.0f;
    });
    for (int i = 0; i < 15; ++i) modified.step(1.0f / 60.0f);
    bool overrideOk = modified.body(modifiedBall).velocity.y > 4.0f;
    modified.setContactModifier([](velox::ContactModifyData& contact) {
        contact.friction1 = std::numeric_limits<float>::quiet_NaN();
    });
    modified.setLinearVelocity(modifiedBall, {0, -20, 0});
    bool validationOk = throwsException([&] {
        for (int i = 0; i < 30; ++i) modified.step(1.0f / 60.0f);
    });

    velox::World invalidMaterial;
    invalidMaterial.gravity = {0, 0, 0};
    auto invalidBody = invalidMaterial.addSphere({}, 0.5f, 1.0f);
    invalidMaterial.body(invalidBody).frictionScale.x = -1.0f;
    validationOk &= throwsException([&] { invalidMaterial.step(1.0f / 60.0f); });
    invalidMaterial.body(invalidBody).frictionScale = {1, 1, 1};
    invalidMaterial.body(invalidBody).frictionCombine =
        static_cast<velox::MaterialCombineMode>(255);
    validationOk &= throwsException([&] { invalidMaterial.step(1.0f / 60.0f); });

    check(combineOk, "material combine modes (all rules and restitution)");
    check(anisotropyOk, "anisotropic friction (body-local directional scales)");
    check(angularOk, "rolling and spinning friction (warm-started torque rows)");
    check(disableOk && overrideOk && validationOk,
          "contact modification (handles, override, disable, validation)");
}

// 33. Fixed joints preserve a complete relative frame. Prismatic joints keep
// two linear and all three angular degrees constrained while leaving signed
// translation along the configured axis available to limits and a force motor.
static void testFixedAndPrismaticJoints() {
    velox::World fixed;
    auto fixedBase = fixed.addBox({0, 2, 0}, {0.25f, 0.25f, 0.25f}, 0.0f);
    auto welded = fixed.addBox({1, 2, 0}, {0.4f, 0.2f, 0.3f}, 1.0f);
    fixed.setTransform(welded, {1, 2, 0}, velox::fromAxisAngle({0, 0, 1}, 0.35f));
    velox::Vec3 initialDirection = velox::rotate(fixed.body(welded).orientation,
                                                  {1, 0, 0});
    auto fixedJoint = fixed.addFixedJoint(fixedBase, welded, {0.5f, 2, 0});
    fixed.setLinearVelocity(welded, {5, -3, 2});
    fixed.setAngularVelocity(welded, {4, 5, 6});
    for (int i = 0; i < 180; ++i) fixed.step(1.0f / 60.0f);
    velox::Vec3 finalDirection = velox::rotate(fixed.body(welded).orientation,
                                                {1, 0, 0});
    bool fixedOk = velox::length(fixed.body(welded).position -
                                 velox::Vec3{1, 2, 0}) < 0.04f &&
                   velox::length(finalDirection - initialDirection) < 0.04f &&
                   fixed.isValid(fixedJoint);

    velox::World freeSlider;
    freeSlider.gravity = {0, 0, 0};
    auto rail = freeSlider.addBox({}, {0.2f, 0.2f, 0.2f}, 0.0f);
    auto carriage = freeSlider.addBox({}, {0.3f, 0.3f, 0.3f}, 1.0f);
    auto freeJoint = freeSlider.addPrismaticJoint(rail, carriage, {}, {1, 0, 0});
    freeSlider.setLinearVelocity(carriage, {3, 5, 2});
    freeSlider.setAngularVelocity(carriage, {3, 4, 5});
    for (int i = 0; i < 60; ++i) freeSlider.step(1.0f / 60.0f);
    float freeTravel = freeSlider.prismaticTranslation(freeJoint);
    velox::Vec3 freePosition = freeSlider.body(carriage).position;
    velox::Vec3 freeAxis = velox::rotate(freeSlider.body(carriage).orientation,
                                         {1, 0, 0});
    bool freeOk = freeTravel > 2.5f && std::fabs(freePosition.y) < 0.03f &&
                  std::fabs(freePosition.z) < 0.03f &&
                  velox::length(freeAxis - velox::Vec3{1, 0, 0}) < 0.03f;

    velox::World powered;
    powered.gravity = {0, 0, 0};
    auto poweredRail = powered.addBox({}, {0.2f, 0.2f, 0.2f}, 0.0f);
    auto poweredCarriage = powered.addBox({}, {0.3f, 0.3f, 0.3f}, 1.0f);
    auto poweredJoint = powered.addPrismaticJoint(
        poweredRail, poweredCarriage, {}, {1, 0, 0});
    auto& slider = powered.joint(poweredJoint);
    slider.enableMotor = true;
    slider.motorSpeed = 4.0f;
    slider.maxMotorForce = 100.0f;
    slider.enableLimit = true;
    slider.lowerLimit = -1.0f;
    slider.upperLimit = 2.0f;
    for (int i = 0; i < 12; ++i) powered.step(1.0f / 60.0f);
    bool motorOk = powered.body(poweredCarriage).velocity.x > 3.5f;
    for (int i = 0; i < 120; ++i) powered.step(1.0f / 60.0f);
    float limitedTravel = powered.prismaticTranslation(poweredJoint);
    bool limitOk = limitedTravel >= -1.03f && limitedTravel <= 2.03f;

    bool validationOk = throwsException([&] {
        freeSlider.addPrismaticJoint(rail, carriage, {}, {});
    });
    validationOk &= throwsException([&] {
        (void)fixed.prismaticTranslation(fixedJoint);
    });
    freeSlider.joint(freeJoint).localAxisA = {};
    validationOk &= throwsException([&] { freeSlider.step(1.0f / 60.0f); });
    check(fixedOk, "fixed joint (anchor and complete relative frame)");
    check(freeOk && motorOk && limitOk && validationOk,
          "prismatic joint (free axis, motor, limits, validation)");
}

// 34. A 6DoF joint independently locks, limits, frees, and motors each axis in
// its creation frame. Translation and exponential-map rotation queries expose
// B relative to A without Euler-angle singularities.
static void testSixDofJoint() {
    velox::World locked;
    locked.gravity = {0, 0, 0};
    auto base = locked.addBox({}, {0.2f, 0.2f, 0.2f}, 0.0f);
    auto payload = locked.addBox({1, 0, 0}, {0.3f, 0.2f, 0.25f}, 1.0f);
    locked.setTransform(payload, {1, 0, 0},
                        velox::fromAxisAngle({0, 0, 1}, 0.35f));
    velox::Vec3 initialDirection = velox::rotate(
        locked.body(payload).orientation, {1, 0, 0});
    auto lockedJoint = locked.addSixDofJoint(base, payload, {0.5f, 0, 0});
    locked.setLinearVelocity(payload, {4, -3, 2});
    locked.setAngularVelocity(payload, {3, 4, -5});
    for (int i = 0; i < 180; ++i) locked.step(1.0f / 60.0f);
    velox::Vec3 lockedTranslation = locked.sixDofLinearTranslation(lockedJoint);
    velox::Vec3 lockedRotation = locked.sixDofAngularRotation(lockedJoint);
    velox::Vec3 finalDirection = velox::rotate(
        locked.body(payload).orientation, {1, 0, 0});
    bool lockOk = velox::length(lockedTranslation) < 0.03f &&
                  velox::length(lockedRotation) < 0.03f &&
                  velox::length(finalDirection - initialDirection) < 0.03f &&
                  !locked.lastStepStats().deviceSubsteps;

    velox::World free;
    free.gravity = {0, 0, 0};
    auto freeBase = free.addBox({}, {0.2f, 0.2f, 0.2f}, 0.0f);
    auto freeBody = free.addBox({}, {0.3f, 0.3f, 0.3f}, 1.0f);
    auto freeJoint = free.addSixDofJoint(freeBase, freeBody, {});
    free.joint(freeJoint).linearLimitMask =
        velox::JointAxisY | velox::JointAxisZ; // free X, lock Y/Z
    free.setLinearVelocity(freeBody, {3, 5, 2});
    free.setAngularVelocity(freeBody, {2, 3, 4});
    for (int i = 0; i < 60; ++i) free.step(1.0f / 60.0f);
    velox::Vec3 freeTranslation = free.sixDofLinearTranslation(freeJoint);
    velox::Vec3 freeRotation = free.sixDofAngularRotation(freeJoint);
    bool freeOk = freeTranslation.x > 2.5f &&
                  std::fabs(freeTranslation.y) < 0.03f &&
                  std::fabs(freeTranslation.z) < 0.03f &&
                  velox::length(freeRotation) < 0.03f;

    velox::World linearMotor;
    linearMotor.gravity = {0, 0, 0};
    auto linearBase = linearMotor.addBox({}, {0.2f, 0.2f, 0.2f}, 0.0f);
    auto linearBody = linearMotor.addBox({}, {0.3f, 0.3f, 0.3f}, 1.0f);
    auto linearJoint = linearMotor.addSixDofJoint(linearBase, linearBody, {});
    auto& linear = linearMotor.joint(linearJoint);
    linear.lowerLinearLimit = {-1, 0, 0};
    linear.upperLinearLimit = {2, 0, 0};
    linear.linearMotorMask = velox::JointAxisX;
    linear.linearMotorSpeed = {4, 0, 0};
    linear.maxLinearMotorForce = {100, 0, 0};
    for (int i = 0; i < 12; ++i) linearMotor.step(1.0f / 60.0f);
    bool linearMotorOk = linearMotor.body(linearBody).velocity.x > 3.5f;
    for (int i = 0; i < 120; ++i) linearMotor.step(1.0f / 60.0f);
    velox::Vec3 limitedTranslation =
        linearMotor.sixDofLinearTranslation(linearJoint);
    bool linearLimitOk = limitedTranslation.x >= -1.03f &&
                         limitedTranslation.x <= 2.03f;

    velox::World angularMotor;
    angularMotor.gravity = {0, 0, 0};
    auto angularBase = angularMotor.addBox({}, {0.2f, 0.2f, 0.2f}, 0.0f);
    auto angularBody = angularMotor.addBox({}, {0.5f, 0.2f, 0.2f}, 1.0f);
    auto angularJoint = angularMotor.addSixDofJoint(
        angularBase, angularBody, {});
    auto& angular = angularMotor.joint(angularJoint);
    angular.lowerAngularLimit = {0, 0, -0.4f};
    angular.upperAngularLimit = {0, 0, 0.4f};
    angular.angularMotorMask = velox::JointAxisZ;
    angular.angularMotorSpeed = {0, 0, 3};
    angular.maxAngularMotorTorque = {0, 0, 100};
    for (int i = 0; i < 8; ++i) angularMotor.step(1.0f / 60.0f);
    bool angularMotorOk =
        angularMotor.sixDofAngularRotation(angularJoint).z > 0.1f;
    for (int i = 0; i < 120; ++i) angularMotor.step(1.0f / 60.0f);
    velox::Vec3 limitedRotation =
        angularMotor.sixDofAngularRotation(angularJoint);
    bool angularLimitOk = limitedRotation.z >= -0.43f &&
                          limitedRotation.z <= 0.43f &&
                          std::fabs(limitedRotation.x) < 0.03f &&
                          std::fabs(limitedRotation.y) < 0.03f;

    bool validationOk = throwsException([&] {
        locked.addSixDofJoint(payload, payload, {});
    });
    validationOk &= throwsException([&] {
        (void)locked.sixDofLinearTranslation(locked.addFixedJoint(
            base, payload, {}));
    });
    free.joint(freeJoint).linearMotorMask = 0x8;
    validationOk &= throwsException([&] { free.step(1.0f / 60.0f); });
    free.joint(freeJoint).linearMotorMask = 0;
    free.joint(freeJoint).lowerAngularLimit.x = 1.0f;
    free.joint(freeJoint).upperAngularLimit.x = -1.0f;
    validationOk &= throwsException([&] { free.step(1.0f / 60.0f); });

    check(lockOk && freeOk,
          "6DoF joint (default lock and independently free axes)");
    check(linearMotorOk && linearLimitOk && angularMotorOk &&
              angularLimitOk && validationOk,
          "6DoF joint (per-axis motors, limits, queries, validation)");
}

// 35. Soft distance constraints use frequency and damping ratio rather than a
// mass-specific stiffness. Equal tuning should therefore produce the same
// motion for light and heavy bodies, and critical damping should settle cleanly.
static void testDistanceSpring() {
    auto sampleSpring = [](float mass, int steps, float dampingRatio) {
        velox::World w;
        w.gravity = {0, 0, 0};
        auto base = w.addSphere({}, 0.2f, 0.0f);
        auto bob = w.addSphere({2, 0, 0}, 0.2f, mass);
        auto spring = w.addSpringJoint(base, bob, {}, {2, 0, 0},
                                       2.0f, dampingRatio);
        w.joint(spring).restLength = 1.0f;
        for (int i = 0; i < steps; ++i) w.step(1.0f / 60.0f);
        return w.body(bob).position.x;
    };

    float light = sampleSpring(1.0f, 15, 0.25f);
    float heavy = sampleSpring(10.0f, 15, 0.25f);
    bool massIndependent = std::fabs(light - heavy) < 0.02f;

    velox::World settled;
    settled.gravity = {0, 0, 0};
    auto base = settled.addSphere({}, 0.2f, 0.0f);
    auto bob = settled.addSphere({2, 0, 0}, 0.2f, 1.0f);
    auto spring = settled.addSpringJoint(base, bob, {}, {2, 0, 0}, 2.0f, 1.0f);
    settled.joint(spring).restLength = 1.0f;
    settled.step(1.0f / 60.0f);
    bool compliant = settled.body(bob).position.x > 1.5f;
    for (int i = 0; i < 179; ++i) settled.step(1.0f / 60.0f);
    bool settledOk = std::fabs(settled.body(bob).position.x - 1.0f) < 0.02f &&
                     std::fabs(settled.body(bob).velocity.x) < 0.05f;

    bool validationOk = throwsException([&] {
        settled.addSpringJoint(base, bob, {}, {}, 0.0f, 1.0f);
    });
    validationOk &= throwsException([&] {
        settled.addSpringJoint(base, bob, {}, {}, 2.0f, -0.1f);
    });
    settled.joint(spring).springFrequencyHz =
        std::numeric_limits<float>::quiet_NaN();
    validationOk &= throwsException([&] { settled.step(1.0f / 60.0f); });

    check(massIndependent && compliant && settledOk && validationOk,
          "distance spring (mass-independent frequency and damping)");
}

// 36. Joint reactions are measured as impulse/dt. Breaking is deferred until
// the solver pass completes, then emits stable body handles and the now-stale
// joint handle before generation-safe slot reuse.
static void testBreakableJoints() {
    velox::World forceWorld;
    forceWorld.gravity = {0, 0, 0};
    auto base = forceWorld.addSphere({}, 0.2f, 0.0f);
    auto payload = forceWorld.addSphere({1, 0, 0}, 0.2f, 1.0f);
    auto rope = forceWorld.addDistanceJoint(base, payload, {}, {1, 0, 0});
    forceWorld.joint(rope).breakForce = 10.0f;
    forceWorld.setLinearVelocity(payload, {100, 0, 0});
    forceWorld.step(1.0f / 60.0f);
    bool forceOk = !forceWorld.isValid(rope) &&
                   forceWorld.jointBreakEvents().size() == 1;
    if (forceOk) {
        const auto& event = forceWorld.jointBreakEvents()[0];
        forceOk &= event.joint == rope && event.a == base && event.b == payload &&
                   event.force > 10.0f;
    }
    auto replacement = forceWorld.addDistanceJoint(base, payload, {}, {1, 0, 0});
    forceOk &= replacement != rope && forceWorld.isValid(replacement);
    forceWorld.step(1.0f / 60.0f);
    forceOk &= forceWorld.jointBreakEvents().empty();

    velox::World torqueWorld;
    torqueWorld.gravity = {0, 0, 0};
    auto torqueBase = torqueWorld.addBox({}, {0.2f, 0.2f, 0.2f}, 0.0f);
    auto rotor = torqueWorld.addBox({}, {0.3f, 0.3f, 0.3f}, 1.0f);
    auto weld = torqueWorld.addFixedJoint(torqueBase, rotor, {});
    torqueWorld.joint(weld).breakTorque = 1.0f;
    torqueWorld.setAngularVelocity(rotor, {0, 100, 0});
    torqueWorld.step(1.0f / 60.0f);
    bool torqueOk = !torqueWorld.isValid(weld) &&
                    torqueWorld.jointBreakEvents().size() == 1 &&
                    torqueWorld.jointBreakEvents()[0].torque > 1.0f;

    auto invalid = torqueWorld.addFixedJoint(torqueBase, rotor, {});
    torqueWorld.joint(invalid).breakForce = -1.0f;
    bool validationOk = throwsException([&] {
        torqueWorld.step(1.0f / 60.0f);
    });
    check(forceOk && torqueOk && validationOk,
          "breakable joints (force, torque, events, handle reuse)");
}

// 37. A rollback point restores dynamic state, topology/generations, joints,
// geometry, solver caches, sleeping history, and event phase. Replaying across
// an impact must reproduce the abandoned timeline on both CPU and CUDA.
static void testWorldSnapshots() {
    velox::World w;
    auto ground = w.addStaticPlane({0, 1, 0}, 0.0f);
    auto falling = w.addSphere({0, 3, 0}, 0.5f, 1.0f);
    auto anchor = w.addSphere({3, 4, 0}, 0.2f, 0.0f);
    auto bob = w.addSphere({3, 2, 0}, 0.3f, 1.0f);
    auto tether = w.addDistanceJoint(anchor, bob, {3, 4, 0}, {3, 2, 0});
    for (int i = 0; i < 20; ++i) w.step(1.0f / 60.0f);
    velox::Vec3 savedPosition = w.body(falling).position;
    size_t savedCount = w.bodyCount();
    auto snapshot = w.saveSnapshot();

    for (int i = 0; i < 120; ++i) w.step(1.0f / 60.0f);
    velox::Vec3 expectedFallingPosition = w.body(falling).position;
    velox::Vec3 expectedFallingVelocity = w.body(falling).velocity;
    velox::Vec3 expectedBobPosition = w.body(bob).position;
    std::vector<velox::ContactEvent> expectedEvents = w.contactEvents();

    w.removeBody(bob);
    std::vector<velox::Vec3> hullPoints = {
        {-0.5f, -0.5f, -0.5f}, {0.5f, -0.5f, -0.5f},
        {0, 0.5f, -0.5f}, {0, 0, 0.5f}};
    auto replacement = w.addConvexHull({10, 10, 10}, hullPoints, 1.0f);
    w.gravity = {1, 2, 3};
    w.substeps = 1;
    w.step(1.0f / 60.0f); // upload the divergent topology/geometry on CUDA

    w.restoreSnapshot(snapshot);
    bool topologyOk = w.bodyCount() == savedCount && w.isValid(falling) &&
                      w.isValid(bob) && w.isValid(tether) &&
                      !w.isValid(replacement) &&
                      velox::length(w.body(falling).position - savedPosition) < 1e-7f &&
                      velox::length(w.gravity - velox::Vec3{0, -9.81f, 0}) < 1e-7f &&
                      w.substeps == 4;
    for (int i = 0; i < 120; ++i) w.step(1.0f / 60.0f);
    bool replayOk = velox::length(w.body(falling).position -
                                  expectedFallingPosition) < 1e-5f &&
                    velox::length(w.body(falling).velocity -
                                  expectedFallingVelocity) < 1e-5f &&
                    velox::length(w.body(bob).position - expectedBobPosition) < 1e-5f &&
                    w.contactEvents().size() == expectedEvents.size();
    if (replayOk) {
        for (size_t i = 0; i < expectedEvents.size(); ++i) {
            const auto& a = w.contactEvents()[i];
            const auto& b = expectedEvents[i];
            replayOk &= a.a == b.a && a.b == b.b && a.type == b.type &&
                        a.sensor == b.sensor;
        }
    }
    velox::World other;
    bool validationOk = throwsException([&] { other.restoreSnapshot(snapshot); });
    velox::WorldSnapshot empty;
    validationOk &= throwsException([&] { w.restoreSnapshot(empty); });
    auto groundHit = w.rayCast({0, 5, 0}, {0, -1, 0}, 10.0f);
    bool geometryOk = groundHit.hit &&
                      (groundHit.body == falling || groundHit.body == ground);
    check(topologyOk && replayOk && validationOk && geometryOk,
          "world snapshot (topology, caches, deterministic replay)");
}

// 38. Built-in frame diagnostics expose enough phase and workload data to
// profile collision/solver/CCD regressions without an external timing harness.
static void testStepStats() {
    velox::World w;
    auto ground = w.addStaticPlane({0, 1, 0}, 0.0f);
    auto ball = w.addSphere({0, 0.5f, 0}, 0.5f, 1.0f);
    auto anchor = w.addSphere({2, 1, 0}, 0.2f, 0.0f);
    w.addDistanceJoint(anchor, ball, {2, 1, 0}, {0, 0.5f, 0});
    w.step(1.0f / 60.0f);
    const velox::StepStats stats = w.lastStepStats();
    double phases = stats.setupMs + stats.collisionDetectionMs + stats.solverMs +
                    stats.ccdMs + stats.finalizeMs;
    bool ok = stats.dt == 1.0f / 60.0f && stats.bodyCount == 3 &&
              stats.awakeDynamicBodies == 1 && stats.generatedContacts >= 1 &&
              stats.solvedContacts >= 1 && stats.jointCount == 1 &&
              stats.setupMs >= 0.0 && stats.collisionDetectionMs >= 0.0 &&
              stats.solverMs >= 0.0 && stats.ccdMs >= 0.0 &&
              stats.finalizeMs >= 0.0 && stats.totalMs > 0.0 &&
              std::fabs(stats.totalMs - phases) < 0.1;
    auto snapshot = w.saveSnapshot();
    w.step(0.0f);
    ok &= w.lastStepStats().dt == 0.0f && w.lastStepStats().bodyCount == 3;
    w.restoreSnapshot(snapshot);
    ok &= w.lastStepStats().dt == stats.dt &&
          w.lastStepStats().generatedContacts == stats.generatedContacts;
    auto hit = w.rayCast({0, 3, 0}, {0, -1, 0}, 10.0f);
    ok &= hit.hit && (hit.body == ball || hit.body == ground);
    check(ok, "step diagnostics (workload counters and phase timings)");
}

// 39. Torque-free asymmetric bodies conserve world angular momentum while
// their body-space inertia rotates, including CPU/CUDA trajectory parity.
static void testGyroscopicIntegration() {
    velox::World cpu(velox::BackendType::Cpu);
    velox::World accelerated;
    cpu.gravity = accelerated.gravity = {0, 0, 0};
    auto cpuBox = cpu.addBox({}, {0.25f, 0.75f, 1.25f}, 2.0f);
    auto acceleratedBox = accelerated.addBox(
        {}, {0.25f, 0.75f, 1.25f}, 2.0f);
    cpu.setAngularVelocity(cpuBox, {3, 5, 7});
    accelerated.setAngularVelocity(acceleratedBox, {3, 5, 7});
    velox::Vec3 initialMomentum = cpu.body(cpuBox).worldAngularMomentum();
    float initialEnergy = cpu.body(cpuBox).rotationalKineticEnergy();
    for (int i = 0; i < 600; ++i) {
        cpu.step(1.0f / 240.0f);
        accelerated.step(1.0f / 240.0f);
    }
    const velox::Body& expected = cpu.body(cpuBox);
    const velox::Body& actual = accelerated.body(acceleratedBox);
    float momentumError = velox::length(
        expected.worldAngularMomentum() - initialMomentum) /
        velox::length(initialMomentum);
    float energyError = std::fabs(expected.rotationalKineticEnergy() -
                                  initialEnergy) / initialEnergy;
    velox::Vec3 expectedAxis = velox::rotate(expected.orientation, {1, 0, 0});
    velox::Vec3 actualAxis = velox::rotate(actual.orientation, {1, 0, 0});
    bool parity = velox::length(expected.angularVelocity -
                                actual.angularVelocity) < 3e-3f &&
                  velox::length(expectedAxis - actualAxis) < 3e-3f;
    bool ok = momentumError < 1e-3f && energyError < 5e-3f && parity;
    if (!ok)
        std::printf("  momentum error %.6g, energy error %.6g, parity %s\n",
                    momentumError, energyError, parity ? "yes" : "no");
    check(ok,
          "gyroscopic integration (momentum, energy, CPU/CUDA parity)");
}

// 40. Dense CUDA narrow phases replay with an exact larger output allocation
// rather than silently truncating contacts at the initial capacity estimate.
static void testDenseContactCapacity() {
    constexpr int kBodyCount = 80;
    constexpr size_t kExpectedContacts =
        size_t(kBodyCount) * size_t(kBodyCount - 1) / 2;
    velox::World cpu(velox::BackendType::Cpu);
    velox::World accelerated;
    cpu.gravity = accelerated.gravity = {};
    for (int i = 0; i < kBodyCount; ++i) {
        auto cpuBody = cpu.addSphere({}, 0.5f, 1.0f);
        auto acceleratedBody = accelerated.addSphere({}, 0.5f, 1.0f);
        cpu.body(cpuBody).sensor = 1;
        accelerated.body(acceleratedBody).sensor = 1;
    }
    cpu.step(1.0f / 60.0f);
    accelerated.step(1.0f / 60.0f);
    bool ok = cpu.lastStepStats().generatedContacts == kExpectedContacts &&
              accelerated.lastStepStats().generatedContacts ==
                  kExpectedContacts &&
              cpu.contactEvents().size() == kExpectedContacts &&
              accelerated.contactEvents().size() == kExpectedContacts;
    accelerated.step(1.0f / 60.0f);
    ok &= accelerated.lastStepStats().generatedContacts == kExpectedContacts &&
          accelerated.contactEvents().size() == kExpectedContacts;

    check(ok, "dense contact capacity (lossless CUDA buffer growth)");
}

// 41. Large joint sets stay resident through CUDA solver substeps; mixed
// constraint types and per-substep breaking must track CPU behavior.
static void testDeviceJointSubsteps() {
    velox::World cpu(velox::BackendType::Cpu);
    velox::World accelerated;
    cpu.gravity = accelerated.gravity = {0, 0, 0};
    std::vector<velox::BodyId> cpuBodies, acceleratedBodies;
    auto populate = [](velox::World& world,
                       std::vector<velox::BodyId>& bodies) {
        auto anchor = world.addSphere({}, 0.1f, 0.0f);
        world.body(anchor).maskBits = 0;
        for (int i = 0; i < 64; ++i) {
            float x = 2.0f + 1.0f * i;
            velox::Vec3 position{x, 0, 0};
            auto body = world.addSphere(position, 0.1f, 1.0f);
            world.body(body).maskBits = 0;
            switch (i % 7) {
            case 0:
                world.addBallJoint(anchor, body, position);
                break;
            case 1:
                world.addSpringJoint(anchor, body, {}, position, 2.0f, 0.7f);
                break;
            case 2:
                world.addFixedJoint(anchor, body, position);
                break;
            case 3: {
                auto joint = world.addSixDofJoint(anchor, body, position);
                world.joint(joint).linearLimitMask =
                    velox::JointAxisY | velox::JointAxisZ;
                break;
            }
            case 4: {
                auto id = world.addPrismaticJoint(anchor, body, position,
                                                  {1, 0, 0});
                auto& joint = world.joint(id);
                joint.enableMotor = joint.enableLimit = true;
                joint.motorSpeed = 0.5f;
                joint.maxMotorForce = 10.0f;
                joint.lowerLimit = -0.2f;
                joint.upperLimit = 0.2f;
                break;
            }
            case 5: {
                auto id = world.addHingeJoint(anchor, body, position,
                                              {0, 1, 0});
                auto& joint = world.joint(id);
                joint.enableMotor = joint.enableLimit = true;
                joint.motorSpeed = 0.3f;
                joint.maxMotorTorque = 10.0f;
                joint.lowerLimit = -0.2f;
                joint.upperLimit = 0.2f;
                break;
            }
            default: {
                auto id = world.addConeTwistJoint(anchor, body, position,
                                                  {0, 1, 0});
                auto& joint = world.joint(id);
                joint.enableSwingLimit = joint.enableTwistLimit = true;
                joint.swingLimit = 0.2f;
                joint.lowerTwistLimit = -0.2f;
                joint.upperTwistLimit = 0.2f;
                break;
            }
            }
            world.setLinearVelocity(body, {0.4f, 1.0f, -0.3f});
            world.setAngularVelocity(body, {0.2f, -0.1f, 0.3f});
            bodies.push_back(body);
        }
    };
    populate(cpu, cpuBodies);
    populate(accelerated, acceleratedBodies);
    for (int step = 0; step < 4; ++step) {
        cpu.step(1.0f / 60.0f);
        accelerated.step(1.0f / 60.0f);
    }

    bool parity = true;
    for (size_t i = 0; i < cpuBodies.size(); ++i) {
        const velox::Body& expected = cpu.body(cpuBodies[i]);
        const velox::Body& actual = accelerated.body(acceleratedBodies[i]);
        parity &= velox::length(expected.position - actual.position) < 2e-3f &&
                  velox::length(expected.velocity - actual.velocity) < 2e-3f &&
                  velox::length(expected.angularVelocity -
                                actual.angularVelocity) < 2e-3f;
    }
    bool cuda = std::string(accelerated.backendName()) == "cuda";
    bool expectedPath = cuda ? accelerated.lastStepStats().deviceSubsteps :
                               !accelerated.lastStepStats().deviceSubsteps;
    velox::World breakable;
    breakable.gravity = {0, 0, 0};
    auto rail = breakable.addSphere({}, 0.1f, 0.0f);
    breakable.body(rail).maskBits = 0;
    velox::JointId brokenJoint, torqueJoint;
    for (int i = 0; i < 64; ++i) {
        velox::Vec3 position{2.0f + i, 0, 0};
        auto body = breakable.addSphere(position, 0.1f, 1.0f);
        breakable.body(body).maskBits = 0;
        auto joint = i == 1 ? breakable.addFixedJoint(rail, body, position) :
                              breakable.addBallJoint(rail, body, position);
        if (i == 0) {
            brokenJoint = joint;
            breakable.joint(joint).breakForce = 0.1f;
            breakable.setLinearVelocity(body, {10, 0, 0});
        } else if (i == 1) {
            torqueJoint = joint;
            breakable.joint(joint).breakTorque = 0.1f;
            breakable.setAngularVelocity(body, {10, 0, 0});
        }
    }
    breakable.step(1.0f / 60.0f);
    bool breakPath = cuda ? breakable.lastStepStats().deviceSubsteps :
                            !breakable.lastStepStats().deviceSubsteps;
    bool forceEvent = false, torqueEvent = false;
    for (const auto& event : breakable.jointBreakEvents()) {
        forceEvent |= event.joint == brokenJoint && event.force > 0.1f;
        torqueEvent |= event.joint == torqueJoint && event.torque > 0.1f;
    }
    bool breakOk = breakPath && !breakable.isValid(brokenJoint) &&
                   !breakable.isValid(torqueJoint) &&
                   breakable.jointBreakEvents().size() == 2 &&
                   forceEvent && torqueEvent;

    check(parity && expectedPath && !cpu.lastStepStats().deviceSubsteps &&
              breakOk,
          "CUDA joint substeps (resident solve and CPU trajectory parity)");
}

// 42. CPU integration and narrow phase may execute on persistent workers, but
// pair-range merging preserves the exact serial contact order and trajectory.
static void testDeterministicCpuWorkers() {
    velox::World serial(velox::BackendType::Cpu);
    velox::World parallel(velox::BackendType::Cpu);
    serial.setWorkerCount(1);
    parallel.setWorkerCount(4);
    std::vector<velox::BodyId> serialBodies, parallelBodies;
    serial.addStaticPlane({0, 1, 0}, 0.0f);
    parallel.addStaticPlane({0, 1, 0}, 0.0f);
    for (int i = 0; i < 512; ++i) {
        int x = i % 16, z = i / 16;
        velox::Vec3 position{(x - 8) * 2.0f, 0.5f, (z - 16) * 2.0f};
        serialBodies.push_back(serial.addSphere(position, 0.5f, 1.0f));
        parallelBodies.push_back(parallel.addSphere(position, 0.5f, 1.0f));
    }
    for (int i = 0; i < 3; ++i) {
        serial.step(1.0f / 60.0f);
        parallel.step(1.0f / 60.0f);
    }
    bool ok = serial.workerCount() == 1 && parallel.workerCount() == 4 &&
              serial.lastStepStats().generatedContacts ==
                  parallel.lastStepStats().generatedContacts;
    for (size_t i = 0; i < serialBodies.size() && ok; ++i) {
        const auto& a = serial.body(serialBodies[i]);
        const auto& b = parallel.body(parallelBodies[i]);
        ok &= velox::length(a.position - b.position) < 1e-7f &&
              velox::length(a.velocity - b.velocity) < 1e-7f &&
              velox::length(a.angularVelocity - b.angularVelocity) < 1e-7f &&
              std::fabs(a.orientation.x - b.orientation.x) < 1e-7f &&
              std::fabs(a.orientation.y - b.orientation.y) < 1e-7f &&
              std::fabs(a.orientation.z - b.orientation.z) < 1e-7f &&
              std::fabs(a.orientation.w - b.orientation.w) < 1e-7f;
    }
    parallel.setWorkerCount(2);
    ok &= parallel.workerCount() == 2;
    check(ok, "CPU workers (parallel integration/narrow phase, serial replay)");
}

// 43. Debug geometry is renderer-agnostic and must safely traverse every
// internal storage kind while allowing shapes, AABBs, contacts, and joints to
// be requested independently.
static void testDebugLines() {
    velox::World w;
    auto plane = w.addStaticPlane({0, 1, 0}, 0.0f);
    auto box = w.addBox({0, 0.5f, 0}, {0.5f, 0.5f, 0.5f}, 1.0f);
    auto anchor = w.addSphere({3, 3, 0}, 0.2f, 0.0f);
    auto capsule = w.addCapsule({3, 2, 0}, 0.3f, 0.6f, 1.0f);
    w.addDistanceJoint(anchor, capsule, {3, 3, 0}, {3, 2, 0});
    w.addCylinder({-3, 2, 0}, 0.4f, 0.6f, 1.0f);
    w.addCone({-5, 2, 0}, 0.5f, 1.2f, 1.0f);
    std::vector<velox::Vec3> hull = {
        {-0.4f,-0.4f,-0.4f}, {0.4f,-0.4f,-0.4f},
        {0,0.4f,-0.4f}, {0,0,0.4f}};
    w.addConvexHull({5, 2, 0}, hull, 1.0f);
    velox::CompoundShape child;
    child.shape = velox::ShapeType::Sphere;
    child.localPosition = {0.5f, 0, 0};
    child.radius = 0.25f;
    w.addCompound({7, 2, 0}, {child}, 1.0f);
    w.addStaticMesh({{-1,0,3}, {1,0,3}, {0,1,3}}, {0,1,2});
    w.step(1.0f / 60.0f);

    std::vector<velox::DebugLine> lines;
    w.debugLines(lines);
    bool ok = lines.size() > 100;
    for (const auto& line : lines)
        ok &= std::isfinite(line.a.x) && std::isfinite(line.a.y) &&
              std::isfinite(line.a.z) && std::isfinite(line.b.x) &&
              std::isfinite(line.b.y) && std::isfinite(line.b.z);
    w.debugLines(lines, velox::DebugDrawContacts);
    ok &= !lines.empty();
    w.debugLines(lines, velox::DebugDrawJoints);
    ok &= !lines.empty();
    w.debugLines(lines, velox::DebugDrawAabbs);
    ok &= lines.size() >= 12;
    lines.push_back({});
    w.debugLines(lines, 0);
    ok &= lines.empty() && w.isValid(plane) && w.isValid(box);
    check(ok, "debug lines (shapes, AABBs, contacts, joints, flags)");
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
    testHull();
    testMotor();
    testLimit();
    testEvents();
    testDeepHullPenetration();
    testDynamicCcdMomentum();
    testTwoAxisFriction();
    testValidation();
    testMotionTypes();
    testForcesAndStateApi();
    testStableHandlesAndRemoval();
    testFiltersSensorsAndEventPhases();
    testConeTwistJoint();
    testCompoundBody();
    testCylinderConeAndHeightfield();
    testFilteredQueriesAndShapeCasts();
    testMaterialsAndContactModification();
    testFixedAndPrismaticJoints();
    testSixDofJoint();
    testDistanceSpring();
    testBreakableJoints();
    testWorldSnapshots();
    testStepStats();
    testGyroscopicIntegration();
    testDenseContactCapacity();
    testDeviceJointSubsteps();
    testDeterministicCpuWorkers();
    testDebugLines();
    std::printf("\n%s\n", failures == 0 ? "All stress tests passed."
                                        : "STRESS TESTS FAILED");
    return failures;
}
