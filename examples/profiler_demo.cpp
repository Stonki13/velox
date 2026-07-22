// Profiler demo — shows how to instrument a Velox simulation, read the
// per-frame time breakdown, track memory, mark GPU/CPU sync points, and export
// a Chrome Trace Event Format file for interactive visualization.
//
// Run it, then open the produced "velox_profiler_trace.json" in
// chrome://tracing or https://ui.perfetto.dev to see the zone hierarchy.

#include "velox/velox.h"
#include "velox/profiler.h"

#include <cstdio>
#include <fstream>
#include <vector>

using namespace velox;
using velox::profile::Category;
using velox::profile::Profiler;

namespace {

// A small helper that pretends to allocate a render/upload buffer so the
// memory-tracking hooks have something concrete to report.
struct TrackedBuffer {
    std::vector<float> storage;
    TrackedBuffer(size_t floats) {
        storage.resize(floats);
        VELOX_PROFILE_ALLOC(floats * sizeof(float));
    }
    ~TrackedBuffer() {
        VELOX_PROFILE_FREE(storage.size() * sizeof(float));
    }
};

// Builds a stacked pile of boxes plus a few spheres resting on a ground plane.
// This produces real broadphase pairs, narrowphase contacts, solver islands,
// and constraint iterations for the profiler to measure. (World owns a mutex
// and is non-copyable/non-movable, so the scene is built in place.)
void buildScene(World& world) {
    world.addStaticPlane({0, 1, 0}, 0.0f);

    // A grid of boxes dropped from increasing heights.
    constexpr int kPerSide = 8;
    for (int x = 0; x < kPerSide; ++x) {
        for (int z = 0; z < kPerSide; ++z) {
            const float px = (x - kPerSide * 0.5f + 0.5f) * 1.1f;
            const float pz = (z - kPerSide * 0.5f + 0.5f) * 1.1f;
            const float py = 1.0f + 0.6f * ((x + z) % 5);
            world.addBox({px, py, pz}, {0.5f, 0.5f, 0.5f}, 1.0f);
        }
    }
    // A handful of spheres rolling through the pile.
    for (int i = 0; i < 12; ++i)
        world.addSphere({-4.0f + i * 0.7f, 5.0f + i * 0.2f, 0.0f}, 0.4f, 1.0f);
}

} // namespace

int main() {
    Profiler& profiler = Profiler::instance();
    profiler.setEnabled(true);
    profiler.nameCurrentThread("MainThread");

    std::printf("profiler_demo: building scene...\n");
    World world(BackendType::Cpu);
    buildScene(world);

    // Simulate a per-frame render/upload buffer to exercise memory tracking.
    TrackedBuffer uploadBuffer(64 * 1024);

    std::printf("profiler_demo: running 90 profiled frames...\n");
    constexpr int kFrames = 90;
    for (int frame = 0; frame < kFrames; ++frame) {
        // World::step already opens a frame and scopes the broadphase,
        // narrowphase, constraint solver, and island management internally.
        // Here we wrap the whole update in a Custom "Simulate" zone and add a
        // fake "Render" zone to show how application code layers on top.
        {
            VELOX_PROFILE_SCOPE("Simulate");
            world.step(1.0f / 60.0f);
        }
        {
            VELOX_PROFILE_CATEGORY_SCOPE("Render", Category::Render);
            // A stand-in for presenting the frame; marks a CPU/GPU sync point
            // the way a real renderer would at swap-chain present.
            volatile float spin = 0.0f;
            for (int i = 0; i < 2000; ++i) spin += 0.001f;
            (void)spin;
            VELOX_PROFILE_GPU_SYNC("Present");
        }
    }

    // The most recent frame's category breakdown, captured by endFrame().
    const auto breakdown = profiler.lastBreakdown();
    std::printf("profiler_demo: last frame (#%llu) took %.3f ms\n",
                static_cast<unsigned long long>(breakdown.frameIndex),
                breakdown.frameMs);

    // Full text report: frame breakdown, zone table, memory, sync points.
    profiler.printReport();

    // Export the Chrome Trace Event Format file.
    const char* tracePath = "velox_profiler_trace.json";
    const bool exported = profiler.exportChromeTrace(tracePath);

    // Verify the trace file landed on disk and is non-empty.
    std::streamsize traceBytes = 0;
    if (exported) {
        std::ifstream check(tracePath, std::ios::binary | std::ios::ate);
        if (check) traceBytes = check.tellg();
    }

    const auto report = profiler.report();
    const bool hasZones = !report.empty();
    const auto memory = profiler.memoryStats();

    std::printf("profiler_demo: zones recorded      = %zu\n", report.size());
    std::printf("profiler_demo: sync points         = %llu\n",
                static_cast<unsigned long long>(profiler.syncPointCount()));
    std::printf("profiler_demo: peak memory tracked = %llu bytes\n",
                static_cast<unsigned long long>(memory.peakBytes));
    std::printf("profiler_demo: chrome trace '%s' %s (%lld bytes)\n",
                tracePath, exported ? "written" : "FAILED",
                static_cast<long long>(traceBytes));

    const bool ok = exported && traceBytes > 0 && hasZones && memory.peakBytes > 0;
    std::printf("profiler_demo: %s\n", ok ? "OK" : "FAILED");
    return ok ? 0 : 1;
}
