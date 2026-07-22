// debug_visualizer — exercises every Velox debug-visualization layer headlessly.
//
// There is no window here: the example implements the renderer-agnostic
// velox::DebugDraw interface, feeds it each World debug layer, and reports how
// many line primitives each layer produced. It doubles as a regression check —
// it returns non-zero if any layer that should have geometry comes back empty.
//
// The same DebugDraw subclass pattern is what you implement to forward lines to
// OpenGL/Vulkan/ImGui; see docs/debugging.md and examples/imgui_debug_draw.h.

#include <velox/debug_draw.h>
#include <velox/velox.h>

#include <cstdio>
#include <set>
#include <string>
#include <vector>

using namespace velox;

namespace {

// A DebugDraw that counts primitives and remembers every distinct color it saw.
// Stands in for a real renderer; the only thing a renderer must implement is
// drawLine(), which this overrides.
class StatsDebugDraw : public DebugDraw {
public:
    size_t lineCount = 0;
    std::set<uint32_t> colors;

    void drawLine(Vec3, Vec3, uint32_t color) override {
        ++lineCount;
        colors.insert(color);
    }

    void reset() {
        lineCount = 0;
        colors.clear();
    }
};

// Render exactly one layer and return how many lines it produced.
size_t countLayer(const World& world, uint32_t flag) {
    LineBatchDebugDraw batch;
    drawWorld(batch, world, flag);
    return batch.size();
}

// Build a small scene that exercises every layer:
//   - a static ground plane (shapes/AABB skip it; it anchors contacts)
//   - a 3-box stack resting on the plane (one contact island)
//   - a lone floating box (a second island, once it lands)
//   - two boxes linked by a hinge joint (joint layer)
//   - a fast sphere that slams into the stack (lots of contacts + velocity)
void buildScene(World& world, BodyId& fastSphere, BodyId& pushedBox) {
    world.addStaticPlane({0, 1, 0}, 0.0f);

    // Resting stack: three boxes, one island once they settle on each other.
    for (int level = 0; level < 3; ++level) {
        BodyId box = world.addBox({0.0f, 0.5f + 1.001f * float(level), 0.0f},
                                  {0.5f, 0.5f, 0.5f}, 1.0f);
        world.body(box).friction = 0.7f;
        world.body(box).restitution = 0.0f;
    }

    // Lone box off to the side: its own island.
    world.addBox({4.0f, 0.5f, 0.0f}, {0.5f, 0.5f, 0.5f}, 1.0f);

    // Hinge joint between two suspended boxes (joint anchor/axis layer).
    BodyId hingeA = world.addBox({-4.0f, 3.0f, 0.0f}, {0.4f, 0.4f, 0.4f}, 1.0f);
    BodyId hingeB = world.addBox({-3.0f, 3.0f, 0.0f}, {0.4f, 0.4f, 0.4f}, 1.0f);
    world.addHingeJoint(hingeA, hingeB, {-3.5f, 3.0f, 0.0f}, {0, 1, 0});

    // A spinning, fast sphere: gives the velocity layer something to show and
    // generates contacts when it reaches the stack.
    fastSphere = world.addSphere({0.0f, 2.0f, -6.0f}, 0.4f, 1.0f);
    world.body(fastSphere).velocity = {0.0f, 0.0f, 12.0f};
    world.body(fastSphere).angularVelocity = {3.0f, 1.0f, 0.0f};

    // A box we will push with a force so the force layer has a non-zero arrow.
    pushedBox = world.addBox({2.0f, 0.5f, 3.0f}, {0.5f, 0.5f, 0.5f}, 1.0f);
}

} // namespace

int main() {
    World world(BackendType::Cpu);
    BodyId fastSphere, pushedBox;
    buildScene(world, fastSphere, pushedBox);

    // Settle the scene so resting contacts and islands exist.
    for (int frame = 0; frame < 90; ++frame)
        world.step(1.0f / 60.0f);

    int failures = 0;
    auto check = [&](const char* name, size_t lines, size_t minimum) {
        bool ok = lines >= minimum;
        std::printf("  %-22s %6zu lines  %s\n", name, lines,
                    ok ? "ok" : "EMPTY (expected geometry)");
        if (!ok) ++failures;
    };

    std::printf("debug_visualizer: per-layer line counts\n");
    check("shapes", countLayer(world, DebugDrawShapes), 1);
    check("aabbs", countLayer(world, DebugDrawAabbs), 1);
    check("contacts", countLayer(world, DebugDrawContacts), 1);
    check("joints", countLayer(world, DebugDrawJoints), 1);
    check("velocity vectors", countLayer(world, DebugDrawVelocityVectors), 1);
    check("centers of mass", countLayer(world, DebugDrawCenterOfMass), 1);

    // Force/torque are cleared inside step(); apply them and draw before stepping.
    world.addForce(pushedBox, {500.0f, 0.0f, 0.0f});
    world.addTorque(pushedBox, {0.0f, 250.0f, 0.0f});
    check("force vectors", countLayer(world, DebugDrawForceVectors), 1);
    world.step(1.0f / 60.0f); // consume the pending force

    // Island coloring: enabling the flag must introduce colors that the plain
    // shape pass does not use (one distinct tint per solver island).
    StatsDebugDraw plain, islanded;
    drawWorld(plain, world, DebugDrawShapes);
    drawWorld(islanded, world, DebugDrawShapes | DebugDrawIslands);
    bool islandColorsChanged = islanded.colors != plain.colors &&
                               islanded.colors.size() >= 2;
    std::printf("  %-22s %6zu colors  %s\n", "island coloring",
                islanded.colors.size(),
                islandColorsChanged ? "ok" : "no per-island tint");
    if (!islandColorsChanged) ++failures;

    // Show the whole kitchen-sink pass through the stats renderer, and use the
    // line-batch bounds helper to frame the scene (as a real camera would).
    StatsDebugDraw everything;
    everything.setFlags(DebugDrawEverything);
    drawWorld(everything, world); // uses the flags stored on the instance
    LineBatchDebugDraw batch;
    drawWorld(batch, world, DebugDrawEverything);
    Vec3 lo, hi;
    batch.bounds(lo, hi);
    std::printf("\n  DebugDrawEverything: %zu lines, %zu distinct colors\n",
                everything.lineCount, everything.colors.size());
    std::printf("  scene bounds: (%.2f, %.2f, %.2f) .. (%.2f, %.2f, %.2f)\n",
                lo.x, lo.y, lo.z, hi.x, hi.y, hi.z);

    if (failures) {
        std::printf("\ndebug_visualizer: %d layer(s) failed\n", failures);
        return 1;
    }
    std::printf("\ndebug_visualizer: all layers produced geometry\n");
    return 0;
}
