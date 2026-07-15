# PROTO26B - Interactive Sandbox v2: Review Status

## Status

Implemented on `proto/sandbox2`. Sandbox v2 remains an optional standalone
example behind `VELOX_BUILD_SANDBOX=OFF`; it adds no Velox public symbols and
does not affect library consumers or the default build graph.

## Renderer Architecture

1. The OpenGL renderer has been replaced by a Vulkan 1.2 renderer. GLFW still
   creates the window and Vulkan surface, while `vk_loader.h` opens
   `vulkan-1.dll` on Windows or `libvulkan` on Linux and resolves every entry
   point at runtime. No Vulkan SDK import library is linked.
2. `examples/sandbox/CMakeLists.txt` fetches GLFW 3.4 and Vulkan-Headers
   `v1.3.290` only when the sandbox option is enabled. Default sandbox-off
   configuration never enters that subdirectory, so it does not fetch either
   dependency.
3. GLSL sources live under `examples/sandbox/shaders/*.vert` and `*.frag`.
   Their precompiled SPIR-V words are committed in matching `*_vert.h` and
   `*_frag.h` headers. Runtime and consumer builds therefore need no shader
   compiler, shaderc, or glslang installation.
4. The renderer creates a present-mode FIFO swapchain, depth image, and x4
   MSAA target when supported (one sample otherwise). It recreates swapchain
   resources after resize/out-of-date presentation and skips rendering while
   GLFW reports a minimized zero-size framebuffer.
5. Bodies are batched by procedural unit mesh and rendered as instanced solid
   geometry. The cache provides cube, icosphere (two subdivisions), capsule,
   cylinder, cone, ground, and generated convex-hull meshes. Hull meshes cache
   by immutable point data rather than a reusable body handle.
6. The mesh fragment shader provides directional Lambert light, ambient fill,
   rim light, distance haze, stable palette colors, and desaturation for
   sleeping bodies. The sky shader uses a procedural blue-to-warm-horizon
   gradient, a sun disc/halo matching the light direction, and ground haze.
   The ground grid is shader-generated and distance-faded.

## Interaction

- `W/A/S/D`: fly camera movement. `Q/E`: turn. Hold right mouse: look.
- Left mouse selects a dynamic body by raycast and drags its clicked local
  anchor. The target is the camera-facing ray point at the original grab
  distance; wheel input adjusts that distance.
- The drag force is `m * (k * error - 2 * sqrt(k) * anchorVelocity)`, with
  `k = 90` and an acceleration cap. Applying it at the world-space anchor
  preserves natural torque, wakes the body every frame, and avoids teleports
  or solver explosions when a dragged body meets a stack.
- `1` through `7` select sphere, box, capsule, cylinder, cone, random hull,
  and rotating complex presets (ring compound, L compound, blob hull).
  `Space` spawns the selection at the cursor ray hit and remains rate-limited
  to ten spawns per second while held.
- `R`, `P`, `+/-`, `F1`-`F4`, and `L` reset, pause, adjust substeps, select
  stack/rain/ragdoll/contraption, and toggle debug lines. The overlay shows
  the selected shape, drag state, stats, and controls.

## Verification

- CPU sandbox-enabled Release build:
  `cmake -S . -B build-sandbox2-cpu -DVELOX_BUILD_SANDBOX=ON -DVELOX_ENABLE_CUDA=OFF`
  and `cmake --build build-sandbox2-cpu --config Release --parallel 4` passed.
- `ctest --test-dir build-sandbox2-cpu -C Release --output-on-failure` passed
  `velox.stress`, `velox.fuzz`, `velox.soak`, `velox.geometry_fuzz`,
  `velox.character`, and `velox.sandbox`.
- `fuzz_demo 80` passed with `80 scenes, 0 failures`.
- The headless `velox_sandbox --selftest` advances all presets and all seven
  spawn categories for 120 frames, verifies finite dynamic positions,
  contacts, and debug-line generation, and never creates a Vulkan instance.
- CUDA 13.2 sandbox-enabled Release build completed; the default-Auto
  `velox_sandbox --selftest` passed.
- Fresh sandbox-off CUDA Release configuration and full build completed with
  no `build-sandbox2-off/_deps` directory created.
- Visual native-window checks confirmed the sky, colored lit solid meshes,
  grid, stable ten-box tower, overlay, and active left-mouse drag state.

## Merge Recommendation

Ready to merge after normal review. No API changes, tests were weakened, or
build artifacts/downloaded dependencies were added.
