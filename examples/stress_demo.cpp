// Stress tests for Predictive Contact Sweeping — each case targets a classic
// CCD failure mode. Exits nonzero if any invariant is violated.
#include <velox/velox.h>
#include <cmath>
#include <cstdio>

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

int main() {
    testBullet();
    testGrazing();
    testPile();
    testHeadOn();
    testResting();
    std::printf("\n%s\n", failures == 0 ? "All stress tests passed."
                                        : "STRESS TESTS FAILED");
    return failures;
}
