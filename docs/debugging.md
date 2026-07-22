# Debugging and Debug Visualization

Velox ships a renderer-agnostic debug-visualization pipeline so you can see
exactly what the solver sees: collider wireframes, broad-phase AABBs, contact
points and normals, joint anchors and axes, velocity and force vectors,
per-body centers of mass, and solver-island coloring. Nothing here depends on a
graphics API — Velox emits colored line segments, and you forward them to
whatever renderer you already have (OpenGL, Vulkan, Dear ImGui, a software
rasterizer, or a headless test harness).

There are two layers:

- **`World::debugLines(std::vector<DebugLine>&, uint32_t flags)`** — fills a
  vector of `{a, b, color}` line segments for the requested layers. This is the
  low-level data source and needs no extra header.
- **`#include <velox/debug_draw.h>`** — a small `DebugDraw` interface plus a
  `drawWorld(draw, world)` bridge that feeds those line segments to your
  renderer. This is the recommended entry point.

## Quick start

```cpp
#include <velox/debug_draw.h>
#include <velox/velox.h>

struct MyDraw : velox::DebugDraw {
    void drawLine(velox::Vec3 a, velox::Vec3 b, uint32_t color) override {
        // Forward (a, b) and the 0xRRGGBBAA color to your renderer.
        // velox::unpackDebugColor(color) splits it into r/g/b/a bytes.
    }
};

velox::World world;
// ... add bodies, joints ...

MyDraw draw;
draw.setFlags(velox::DebugDrawEverything);

while (running) {
    world.step(1.0f / 60.0f);
    drawWorld(draw, world);   // call once per frame, after step()
    // present frame
}
```

`drawWorld` asks the world for its line list and forwards each segment to
`drawLine()`. Implementing just `drawLine()` is enough to render every layer;
`DebugDraw` also offers optional `drawPoint`, `drawTriangle`, `drawArrow`, and
`drawText` hooks with line-based defaults you can override for faster batched
rendering.

If you only want the raw data (e.g. to serialize or post-process), call
`World::debugLines` directly:

```cpp
std::vector<velox::DebugLine> lines;
world.debugLines(lines, velox::DebugDrawShapes | velox::DebugDrawContacts);
```

## Layer reference

Each layer is an independent bit in `DebugDrawFlags` (`include/velox/world.h`).
Combine freely with bitwise OR.

| Flag | What it draws | Color |
|---|---|---|
| `DebugDrawShapes` | Collider wireframes; compound children and mesh triangles included | dynamic = cyan, static = grey (or island tint) |
| `DebugDrawAabbs` | Broad-phase axis-aligned bounding boxes | amber |
| `DebugDrawContacts` | Contact point cross + normal arrow (length 0.35 m) | red |
| `DebugDrawJoints` | Anchor link line, anchor crosses, and joint axes | green (axes: X red / Y green / Z blue) |
| `DebugDrawVelocityVectors` | Linear velocity arrow + angular-velocity axis and spin arc | teal (linear), orange (angular) |
| `DebugDrawForceVectors` | Pending accumulated force and torque arrows | pink (force), purple (torque) |
| `DebugDrawCenterOfMass` | Body-frame axes at each center of mass (X/Y/Z = R/G/B) | axis colors |
| `DebugDrawIslands` | Recolors dynamic shapes by solver island | golden-ratio palette |

Convenience combinations:

- `DebugDrawAll` — the original four layers (shapes, AABBs, contacts, joints).
  Kept stable for backward compatibility.
- `DebugDrawEverything` — all eight layers.

## Reading the layers

**Shapes.** Wireframes are drawn from the live transforms, so a shape that looks
wrong here is genuinely mis-modeled. Static planes draw as a small cross with a
normal stub; static meshes draw every triangle (can be dense for large level
geometry — toggle this layer off when not needed).

**AABBs.** These are the *broad-phase* bounds used to cull pair tests. If two
bodies' AABBs never overlap, they can never generate contacts. A body whose AABB
looks far too large is usually a fast-spinning or fast-moving body whose swept
bounds grew — expected, but worth knowing when diagnosing "why is the broad phase
doing so many tests?".

**Contacts.** A cross marks the contact point; the arrow points along the contact
normal (from body B toward body A). Velox uses *speculative* contacts, so you may
see contacts slightly before bodies visibly touch — that is Predictive Contact
Sweeping creating the constraint early, not a bug. The number of crosses matches
`lastStepStats().generatedContacts`.

**Joints.** The green line connects the two world-space anchors; when the anchors
drift apart the constraint is being stretched (stiffness/iteration budget). Hinge,
cone-twist, and prismatic joints draw their axis; 6DoF joints draw the full
joint-frame triad.

**Velocity vectors.** The teal arrow is `velocity * 0.10` (a tenth of a second of
motion), so its length is directly comparable across bodies. The orange axis plus
arc show angular velocity; the arc sweep grows with spin rate. A body that should
be at rest but shows a velocity arrow is jittering — check solver iterations,
friction, or whether something is still injecting energy.

**Force vectors.** These show *pending* `addForce`/`addTorque` inputs. **The solver
clears accumulated forces inside `step()`**, so draw this layer *between* applying
forces and the next `step()` — immediately after `step()` the arrows are zero.

**Centers of mass.** The triad marks each body's center of mass and orientation
(body X = red, Y = green, Z = blue). For compound and hull bodies the center of
mass is recomputed and recentered at creation, so this is the authoritative point
the solver rotates about.

**Islands.** An island is a set of dynamic bodies connected by contacts and/or
joints that the solver can put to sleep as a unit. Enabling this layer tints each
island with a distinct color (static bodies stay grey). Two touching dynamic
bodies share a color; a body that changes color when it touches another has just
merged islands. Use it to confirm sleeping boundaries and to spot unexpectedly
large islands that keep more of the scene awake than intended.

## Headless debugging

Not every bug needs a window. `examples/debug_visualizer` implements `DebugDraw`
without any renderer, renders each layer, and reports how many line segments each
produced — returning non-zero if a layer that should have geometry comes back
empty. It is wired into CTest as `velox.debug_visualizer`:

```bash
cmake --build build --target debug_visualizer
./build/examples/debug_visualizer
```

`LineBatchDebugDraw` (in `debug_draw.h`) is a ready-made accumulator for the same
pattern: it stores every line and can report the axis-aligned `bounds()` of the
scene, which is handy for auto-framing a camera.

## Interactive debugging with Dear ImGui

`examples/imgui_debug_draw.h` provides `velox::ImGuiDebugDraw`, a `DebugDraw`
implementation that rasterizes the line lists into an `ImDrawList` through a small
software orbit camera — no GPU backend of its own. It is header-only and guarded
by `__has_include(<imgui.h>)`, so it is inert in projects without ImGui.

Inside your frame, after `world.step()`:

```cpp
#include "imgui_debug_draw.h"

static velox::ImGuiDebugDraw draw;
draw.camera.target   = {0, 1, 0};
draw.camera.distance = 12.0f;
draw.setFlags(velox::DebugDrawEverything);

// Drive the camera from input however you like:
draw.camera.yaw   += io.MouseDelta.x * 0.005f;
draw.camera.pitch += io.MouseDelta.y * 0.005f;

draw.begin(ImGui::GetBackgroundDrawList(), io.DisplaySize);
velox::drawWorld(draw, world);
draw.end();
```

`examples/imgui_debug_example.cpp` shows a complete frame loop. The ImGui example
is **off by default** so the core build never depends on ImGui; enable it with:

```bash
cmake -S . -B build -DVELOX_BUILD_IMGUI_EXAMPLE=ON -DCMAKE_PREFIX_PATH=<imgui-install>
```

If ImGui is not found, the example still builds as a dependency-free stub that
prints setup instructions.

## Tips and troubleshooting

- **Call after `step()`.** Contacts, islands, and velocities reflect the most
  recent step. Drawing before the first step shows shapes/AABBs/joints but no
  contacts.
- **Force arrows are empty right after `step()`.** Apply forces, draw the
  `DebugDrawForceVectors` layer, *then* step.
- **Too many lines?** Static meshes draw every triangle. Combine
  `DebugDrawShapes` selectively, or drop `DebugDrawAabbs`/mesh-heavy scenes when
  profiling renderer throughput rather than physics.
- **Colors.** All colors are packed `0xRRGGBBAA`. Use `velox::unpackDebugColor`
  (bytes) or `velox::unpackDebugColorF` (normalized floats) to map them to your
  vertex format; `velox::debugColor(r, g, b, a)` packs your own.
- **Thread safety.** `debugLines` takes the world's query lock, so it is safe to
  call from a render thread under the `Relaxed`/`Concurrent` thread-safety
  policies, but not concurrently with `step()` under the default `Strict` policy.
  See [threading.md](threading.md).
- **Determinism is unaffected.** Debug drawing is a read-only query; it never
  mutates simulation state and does not change solver results.
