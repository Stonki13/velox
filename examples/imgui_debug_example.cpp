// imgui_debug_example — interactive physics debugging with Dear ImGui.
//
// Demonstrates wiring velox::ImGuiDebugDraw (see imgui_debug_draw.h) into an
// ImGui frame so you can orbit a scene and toggle physics debug layers live.
//
// This file is dependency-safe: when <imgui.h> is not on the include path it
// compiles to a small stub that explains how to build the real thing, so adding
// it to a project never breaks the build. To run the interactive version, add
// ImGui to your include path (and link imgui.cpp / imgui_draw.cpp / imgui_widgets.cpp
// / imgui_tables.cpp), e.g.:
//
//   c++ -std=c++17 imgui_debug_example.cpp -I<path-to>/imgui \
//       <path-to>/imgui/imgui.cpp <path-to>/imgui/imgui_draw.cpp \
//       <path-to>/imgui/imgui_widgets.cpp <path-to>/imgui/imgui_tables.cpp \
//       -I<velox>/include -L<velox>/build -lvelox
//
// In a real windowed app you would also create an ImGui backend (imgui_impl_glfw
// + imgui_impl_opengl3, etc.). The headless path below renders one frame without
// any backend so the projection and layer wiring can be verified on their own.

#include "imgui_debug_draw.h"

#include <velox/velox.h>

#include <cstdio>

#ifdef VELOX_HAVE_IMGUI

#include <imgui.h>

using namespace velox;

namespace {

// A compact scene with contacts, a joint, and motion so every layer is visible.
World makeScene() {
    World world(BackendType::Cpu);
    world.addStaticPlane({0, 1, 0}, 0.0f);
    for (int level = 0; level < 4; ++level)
        world.addBox({0.0f, 0.5f + 1.001f * float(level), 0.0f},
                     {0.5f, 0.5f, 0.5f}, 1.0f);
    BodyId a = world.addBox({-3.0f, 2.5f, 0.0f}, {0.4f, 0.4f, 0.4f}, 1.0f);
    BodyId b = world.addBox({-2.0f, 2.5f, 0.0f}, {0.4f, 0.4f, 0.4f}, 1.0f);
    world.addHingeJoint(a, b, {-2.5f, 2.5f, 0.0f}, {0, 1, 0});
    BodyId ball = world.addSphere({2.0f, 3.0f, -4.0f}, 0.4f, 1.0f);
    world.body(ball).velocity = {0.0f, 0.0f, 8.0f};
    world.body(ball).angularVelocity = {2.0f, 0.5f, 0.0f};
    return world;
}

} // namespace

int main() {
    // --- one-time ImGui setup (headless: no windowing backend) --------------
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.DisplaySize = ImVec2(1280, 720);
    io.IniFilename = nullptr; // do not write imgui.ini
    ImGui::StyleColorsDark();
    io.Fonts->AddFontDefault();
    io.Fonts->Build();

    World world = makeScene();
    ImGuiDebugDraw draw;
    draw.camera.target = {0, 1, 0};
    draw.camera.distance = 12.0f;
    draw.setFlags(DebugDrawEverything);

    // --- per-frame loop (here: a fixed number of headless frames) -----------
    const int frames = 120;
    for (int frame = 0; frame < frames; ++frame) {
        world.step(1.0f / 60.0f);

        // In a windowed app, drive the camera from input here, e.g.:
        //   draw.camera.yaw   += io.MouseDelta.x * 0.005f;
        //   draw.camera.pitch += io.MouseDelta.y * 0.005f;
        draw.camera.yaw += 0.01f; // slow auto-orbit so the projection is exercised

        ImGui::NewFrame();
        draw.begin(ImGui::GetBackgroundDrawList(), io.DisplaySize);
        drawWorld(draw, world); // forwards every enabled layer
        draw.end();
        ImGui::Render();

        if (frame == frames - 1) {
            ImDrawData* data = ImGui::GetDrawData();
            std::printf("imgui_debug_example: final frame\n");
            std::printf("  debug line segments rasterized: %d\n", draw.drawnLines());
            std::printf("  ImGui draw lists: %d, total vertices: %d, indices: %d\n",
                        data ? data->CmdListsCount : 0,
                        data ? data->TotalVtxCount : 0,
                        data ? data->TotalIdxCount : 0);
        }
    }

    ImGui::DestroyContext();
    std::printf("imgui_debug_example: headless render complete\n");
    std::printf("  (link a windowing backend such as GLFW+OpenGL3 for an interactive window)\n");
    return 0;
}

#else // !VELOX_HAVE_IMGUI

int main() {
    std::printf("imgui_debug_example: Dear ImGui was not found on the include path.\n\n");
    std::printf("This example is optional. To build the interactive version:\n");
    std::printf("  1. Get Dear ImGui (https://github.com/ocornut/imgui).\n");
    std::printf("  2. Add its folder to the include path and compile imgui.cpp,\n");
    std::printf("     imgui_draw.cpp, imgui_widgets.cpp, imgui_tables.cpp.\n");
    std::printf("  3. Add a windowing backend (e.g. backends/imgui_impl_glfw.cpp +\n");
    std::printf("     imgui_impl_opengl3.cpp) for a real interactive window.\n\n");
    std::printf("The headless debug-line pipeline is covered by the debug_visualizer\n");
    std::printf("example, which needs no third-party dependencies.\n");
    return 0;
}

#endif // VELOX_HAVE_IMGUI
