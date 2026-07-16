#pragma once
// Vulkan 1.2 renderer: procedural sky, instanced lit shape meshes, debug
// lines, and a 2D overlay. The loader is resolved dynamically so no Vulkan
// SDK is needed to run; only a driver.
#include "font.h"
#include "mesh_gen.h"

#include <cstdint>
#include <memory>
#include <vector>

namespace sandbox {

class Window;

// Row-major 3x4 world transform plus color. color.w selects the fragment
// path: 1 = lit body, 2 = ground grid.
struct Instance {
    float row0[4];
    float row1[4];
    float row2[4];
    float color[4];
};

struct LineVertex {
    float x, y, z;
    float r, g, b, a;
};

struct FrameGlobals {
    float viewProj[16];    // column-major, GL conventions
    float invViewProj[16];
    float cameraPos[4];
    float sunDir[4];
};

struct DrawBatch {
    uint32_t meshId = 0;
    std::vector<Instance> instances;
};

class Renderer {
public:
    explicit Renderer(Window& window);
    ~Renderer();
    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;

    // Uploads a mesh once; returns a stable id usable in DrawBatch.
    uint32_t registerMesh(const CpuMesh& mesh);

    void render(const FrameGlobals& globals,
                const std::vector<DrawBatch>& batches,
                const std::vector<LineVertex>& lines,
                const std::vector<UiVertex>& ui,
                int width, int height);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace sandbox
