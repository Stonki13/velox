# 26 — Interactive Sandbox

## Goal

Provide a windowed demo application that renders Velox simulations with interactive controls: add bodies, apply forces, adjust parameters in real-time, and inspect contacts/constraints via the debug-line API. This serves as a development tool for tuning simulation parameters and a showcase for potential users.

## Public API

```cpp
// The sandbox is a standalone executable (examples/sandbox/), not part of the
// library API. It links against Velox and a minimal windowing/rendering layer.
// No new public symbols are added to the velox namespace.

// Configuration for the sandbox application:
struct SandboxConfig {
    int windowWidth = 1920;
    int windowHeight = 1080;
    float defaultGravityY = -9.81f;
    int defaultSubsteps = 4;
    bool enableDebugLines = true;
    bool enableStatsOverlay = true;
};

// Keyboard controls (documented, not part of the library API):
//   W/A/S/D — move camera
//   Q/E — rotate camera
//   Space — add sphere at cursor
//   R — reset scene
//   P — pause/unpause
//   +/- — increase/decrease substeps
//   F1-F4 — load preset scenes (stack, rain, ragdoll, vehicle)

```

## Data structures

- No new library symbols. The sandbox is an example application in `examples/sandbox/`.
- Debug-line rendering integration: the sandbox calls `World::debugLines()` each frame and renders the returned `std::vector<DebugLine>` as colored line segments.

## Algorithm

**Sandbox architecture:**

1. **Windowing:** Use a minimal GLFW or Win32 window for rendering. The sandbox is intentionally dependency-light; users can swap in their preferred windowing library.
2. **Rendering:** Debug lines are rendered as OpenGL/Vulkan/DirectX line primitives. Each `DebugLine` has start/end positions and an RGBA color.
3. **Interaction:** Mouse clicks add bodies at the cursor position (raycast from camera to determine world-space coordinates). Keyboard shortcuts control simulation parameters.
4. **Stats overlay:** Render `StepStats` values (body count, contact count, step time) as text in the corner of the window using a bitmap font.

**Preset scenes:**

- `stack.json`: 10-box tower (tests resting stability).
- `rain.json`: 200 falling spheres (tests broad-phase + solver throughput).
- `ragdoll.json`: 8-body ragdoll on a ramp (tests joints).
- `vehicle.json`: simple raycast vehicle on terrain (tests suspension).

## Files

- `examples/sandbox/main.cpp` — sandbox entry point
- `examples/sandbox/window.h/cpp` — minimal windowing abstraction
- `examples/sandbox/renderer.h/cpp` — debug-line rendering
- `examples/sandbox/input.h/cpp` — keyboard/mouse input handling
- `examples/sandbox/scenes.json` — preset scene definitions

## Tests

1. **Sandbox launches:** The sandbox executable starts, renders a default scene (empty world with a ground plane), and responds to keyboard input without crashing.
2. **Body addition:** Clicking in the window adds a sphere at the raycast hit point; the sphere falls under gravity and rests on the ground plane.
3. **Stats overlay:** Step time, body count, and contact count are displayed correctly and update each frame.
4. **Preset loading:** Pressing F1-F4 loads the corresponding preset scene; all bodies appear and simulate correctly.

## Acceptance

- [ ] Sandbox executable builds and launches on Windows and Linux
- [ ] Keyboard controls respond correctly (add body, reset, pause, adjust substeps)
- [ ] Debug lines render accurately (shapes, contacts, joints)
- [ ] Stats overlay displays correct per-frame metrics
- [ ] All 4 preset scenes load and simulate without errors

## Size: M

## Risks

- The sandbox adds a rendering dependency (OpenGL/Vulkan/DirectX) that may not be available on all CI machines. Must make the sandbox optional in the build (`VELOX_BUILD_SANDBOX` flag).
- Debug-line rendering can become a bottleneck with thousands of contacts. Must implement instanced line rendering or frustum culling for large scenes.
- The sandbox is a development tool, not a production feature. Document that it may change without notice and is not part of the public API contract.
