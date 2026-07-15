# PROTO26 - Interactive Sandbox: Review Status

## Status

Implemented on `proto/sandbox`. The sandbox is an optional standalone GLFW
and OpenGL 3.3 executable. It does not add symbols to the `velox` namespace,
does not change the library target's public API, and is excluded unless
`VELOX_BUILD_SANDBOX=ON` is selected.

## Design Decisions

1. `examples/sandbox/CMakeLists.txt` fetches GLFW 3.4 with `FetchContent` only
   for the optional target. OpenGL is found through CMake. Both the fetched
   source and build products remain beneath ignored build directories.
2. The renderer has a small in-tree OpenGL 3.3 loader and shader pair. Every
   `World::debugLines()` segment is packed into one dynamic VBO and rendered
   in one `GL_LINES` draw call per frame. A separate VBO draws the stats/help
   overlay using an embedded 5x7 bitmap font, avoiding a font dependency.
3. `main.cpp` owns the scene builders shared by windowed mode and
   `--selftest`. `scenes.json` supplies the preset counts, sizes, and ramp
   angles through a deliberately small, field-specific JSON reader; no JSON
   library is linked.
4. Simulation advances with a fixed 60 Hz accumulator, capped render delta,
   and independent render loop. Pausing clears accumulated elapsed time so an
   unpause cannot produce a large catch-up step.
5. Sphere placement raycasts from the camera through the cursor, offsets the
   spawn point along the hit normal, and rate-limits held Space to ten spheres
   per second. This avoids unbounded per-frame additions while preserving the
   requested held-key behavior.
6. F4 is intentionally a `contraption`, not a raycast vehicle: a box chassis
   with four hinged spherical wheels rolls down a terrain ramp. Roadmap item
   13 is not present, so implementing its vehicle here would create a hidden
   duplicate subsystem.

## Controls

- `W/A/S/D`: move the fly camera.
- `Q/E`: turn the camera.
- Hold right mouse button: mouse-look.
- `Space`: add a sphere at the cursor raycast hit.
- `R`: reset the current preset. `P`: pause/unpause.
- `+/-`: increase/decrease solver substeps from 1 through 16.
- `F1/F2/F3/F4`: stack, rain, ragdoll, and contraption presets.

## Verification

CPU-only sandbox-enabled Release build:

- `cmake -S . -B build-sandbox-cpu -DVELOX_BUILD_SANDBOX=ON -DVELOX_ENABLE_CUDA=OFF`
  and `cmake --build build-sandbox-cpu --config Release --parallel 4`: passed.
- `ctest --test-dir build-sandbox-cpu -C Release --output-on-failure`: passed
  `velox.stress`, `velox.fuzz`, `velox.soak`, `velox.geometry_fuzz`,
  `velox.character`, and `velox.sandbox`.
- `fuzz_demo 80`: `80 scenes, 0 failures`.
- `proto_manifold`: all eight manifold checks passed.

CUDA-enabled sandbox build:

- `cmake -S . -B build-sandbox -DVELOX_BUILD_SANDBOX=ON` detected CUDA 13.2.
- `cmake --build build-sandbox --config Release --target velox_sandbox --parallel 2`: passed.
- The default-Auto `velox_sandbox --selftest` passed all four presets.
- A visible windowed launch remained alive for five seconds before being
  closed, confirming GLFW/OpenGL context startup. Automated pixel inspection
  is not available in this native desktop check.

Default sandbox-off build:

- `cmake -S . -B build-default-sandbox-off -DVELOX_BUILD_SANDBOX=OFF` and
  `cmake --build build-default-sandbox-off --config Release --parallel 2`:
  passed with CUDA enabled. No GLFW dependency was fetched or built.

`velox_sandbox --selftest` constructs each preset without a window, advances
120 frames, checks dynamic positions are finite, requires contacts and debug
lines, and exits nonzero on failure. It is registered as `velox.sandbox` only
when the option is enabled.

## Merge Recommendation

Ready to merge after normal review. The only roadmap deviation is the
documented F4 contraption substitution for the unavailable vehicle system.
