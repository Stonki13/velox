// Velox benchmark: CPU vs CUDA backend on two scenes.
//   A) N-sphere rain into a walled floor (dense dynamic-dynamic contact)
//   B) sphere rain onto a procedurally bumpy triangle-mesh terrain (BVH)
#include <velox/velox.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

using Clock = std::chrono::high_resolution_clock;

static float runScene(velox::World& w, int steps) {
    // Warm-up step excluded from timing (GPU buffer allocation, BVH upload).
    w.step(1.0f / 60.0f);
    auto t0 = Clock::now();
    for (int i = 0; i < steps; ++i) w.step(1.0f / 60.0f);
    auto t1 = Clock::now();
    return std::chrono::duration<float, std::milli>(t1 - t0).count() / steps;
}

static void sceneRain(velox::World& w, int n) {
    w.addStaticPlane({0, 1, 0}, 0.0f);
    w.addStaticPlane({1, 0.2f, 0}, -25.0f);
    w.addStaticPlane({-1, 0.2f, 0}, -25.0f);
    w.addStaticPlane({0, 0.2f, 1}, -25.0f);
    w.addStaticPlane({0, 0.2f, -1}, -25.0f);
    int side = (int)std::ceil(std::cbrt((float)n));
    for (int i = 0; i < n; ++i) {
        int x = i % side, y = (i / side) % side, z = i / (side * side);
        w.addSphere({(x - side * 0.5f) * 1.1f, 2.0f + y * 1.1f, (z - side * 0.5f) * 1.1f},
                    0.5f, 1.0f);
    }
}

static void sceneMesh(velox::World& w, int n, int grid) {
    // Bumpy terrain: grid x grid quads => 2*grid^2 triangles.
    std::vector<velox::Vec3> verts;
    std::vector<uint32_t> idx;
    float size = 60.0f, half = size * 0.5f;
    for (int z = 0; z <= grid; ++z)
        for (int x = 0; x <= grid; ++x) {
            float fx = x * size / grid - half, fz = z * size / grid - half;
            float fy = std::sin(fx * 0.4f) * std::cos(fz * 0.4f) * 1.5f;
            verts.push_back({fx, fy, fz});
        }
    for (int z = 0; z < grid; ++z)
        for (int x = 0; x < grid; ++x) {
            uint32_t a = z * (grid + 1) + x, b = a + 1, c = a + grid + 1, d = c + 1;
            idx.insert(idx.end(), {a, c, b, b, c, d});
        }
    w.addStaticMesh(verts, idx);

    int side = (int)std::ceil(std::cbrt((float)n));
    for (int i = 0; i < n; ++i) {
        int x = i % side, y = (i / side) % side, z = i / (side * side);
        w.addSphere({(x - side * 0.5f) * 1.2f, 6.0f + y * 1.2f, (z - side * 0.5f) * 1.2f},
                    0.5f, 1.0f);
    }
    std::printf("  (terrain: %d triangles)\n", (int)idx.size() / 3);
}

int main(int argc, char** argv) {
    const int steps = argc > 1 ? std::max(1, std::atoi(argv[1])) : 120;
    std::printf("Velox benchmark — ms per step, %d steps, 60 Hz\n\n", steps);

    struct Configuration {
        velox::BackendType backend;
        uint32_t workers;
        const char* label;
    };
    const Configuration configurations[] = {
        {velox::BackendType::Cpu, 1, "cpu-1"},
        {velox::BackendType::Cpu, 0, "cpu-auto"},
        {velox::BackendType::Auto, 0, "auto"}};

    for (int n : {512, 2048, 8192}) {
        std::printf("Scene A: %d-sphere rain\n", n);
        for (const Configuration& configuration : configurations) {
            try {
                velox::World w(configuration.backend);
                if (configuration.backend == velox::BackendType::Cpu)
                    w.setWorkerCount(configuration.workers);
                sceneRain(w, n);
                float ms = runScene(w, steps);
                std::printf("  %-8s (%s, %u workers) %8.3f ms/step\n",
                            configuration.label, w.backendName(),
                            w.workerCount(), ms);
            } catch (const std::exception& e) {
                std::printf("  (%s)\n", e.what());
            }
        }
    }

    for (int n : {2048}) {
        std::printf("Scene B: %d spheres on mesh terrain\n", n);
        for (const Configuration& configuration : configurations) {
            try {
                velox::World w(configuration.backend);
                if (configuration.backend == velox::BackendType::Cpu)
                    w.setWorkerCount(configuration.workers);
                sceneMesh(w, n, 100);
                float ms = runScene(w, steps);
                std::printf("  %-8s (%s, %u workers) %8.3f ms/step\n",
                            configuration.label, w.backendName(),
                            w.workerCount(), ms);
            } catch (const std::exception& e) {
                std::printf("  (%s)\n", e.what());
            }
        }
    }
    return 0;
}
