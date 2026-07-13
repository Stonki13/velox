// Stress tests for Predictive Contact Sweeping — each case targets a classic
// CCD failure mode. Exits nonzero if any invariant is violated.
#include <velox/velox.h>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
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
    std::printf("\n%s\n", failures == 0 ? "All stress tests passed."
                                        : "STRESS TESTS FAILED");
    return failures;
}
