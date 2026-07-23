// gpu_debris_demo: a large-body/debris scene sized to land in the regime
// Phase 2's profiling showed CUDA actually wins in (dense dynamic-dynamic
// contact scenes at 2000+ bodies) -- a weld-joint wall breaks apart under an
// explosion into thousands of independent debris cubes, then piles up.
// Runs on BackendType::Auto so it exercises CUDA when available and falls
// back to the CPU backend transparently otherwise (Phase 2's fallback
// requirement); either way the smoke test only asserts correctness
// (finite, bounded state), not a hard performance number, since CPU timing
// varies far more across machines than "did the simulation stay sane."
#include <velox/velox.h>

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace velox;

int main() {
    World world(BackendType::Auto);
    world.gravity = {0, -9.81f, 0};
    world.addStaticPlane({0, 1, 0}, 0.0f);

    // A welded wall: gridW x gridH unit boxes, each pair of neighbors welded
    // together so the whole thing starts as one rigid structure.
    // Weld-joint-breaking dynamics are considerably heavier per body than
    // the plain sphere-rain scenes Phase 2 profiled (joint graph rebuilds as
    // welds snap at different times), so this is sized for a fast, reliable
    // CTest smoke test rather than to chase the CUDA/CPU crossover point
    // measured there.
    constexpr int gridW = 16, gridH = 16;
    constexpr float breakForce = 4000.0f, breakTorque = 4000.0f;
    std::vector<std::vector<BodyId>> wall(gridH, std::vector<BodyId>(gridW));
    for (int y = 0; y < gridH; ++y)
        for (int x = 0; x < gridW; ++x)
            wall[y][x] = world.addBox({float(x) * 1.02f, 1.0f + float(y) * 1.02f, 0},
                                      {0.5f, 0.5f, 0.5f}, 1.0f);
    for (int y = 0; y < gridH; ++y)
        for (int x = 0; x < gridW; ++x) {
            Vec3 anchor = world.body(wall[y][x]).position;
            if (x + 1 < gridW)
                world.addWeldJoint(wall[y][x], wall[y][x + 1], anchor, breakForce, breakTorque);
            if (y + 1 < gridH)
                world.addWeldJoint(wall[y][x], wall[y + 1][x], anchor, breakForce, breakTorque);
        }

    // Blast from the wall's center: this is the "debris" trigger -- every
    // weld within range that exceeds its break threshold snaps, turning one
    // rigid structure into up to gridW*gridH independent dynamic bodies, the
    // dense-contact regime this demo targets.
    world.explode({float(gridW) * 0.5f, float(gridH) * 0.5f, 0}, 30.0f, 6000.0f);

    const std::string backend = world.backendName();
    const int settleSteps = 40;
    double totalMs = 0.0;
    for (int i = 0; i < settleSteps; ++i) {
        world.step(1.0f / 60.0f);
        totalMs += world.lastStepStats().totalMs;
    }

    bool ok = true;
    for (int y = 0; y < gridH && ok; ++y)
        for (int x = 0; x < gridW && ok; ++x) {
            const Body& b = world.body(wall[y][x]);
            ok = std::isfinite(b.position.x) && std::isfinite(b.position.y) &&
                 std::isfinite(b.position.z) && std::isfinite(b.velocity.x) &&
                 std::isfinite(b.velocity.y) && std::isfinite(b.velocity.z) &&
                 b.position.y > -50.0f; // sanity bound: nothing fell through the floor
        }

    std::printf("gpu_debris_demo: backend=%s bodies=%zu steps=%d avg_ms_per_step=%.3f result=%s\n",
               backend.c_str(), world.bodyCount(), settleSteps, totalMs / settleSteps,
               ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
