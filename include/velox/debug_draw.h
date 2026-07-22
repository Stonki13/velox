#pragma once
// Renderer-agnostic debug visualization for Velox.
//
// World::debugLines() turns the simulation state into colored line lists for a
// set of independently selectable layers (see DebugDrawFlags in world.h):
// collider wireframes, AABBs, contact points/normals, joint anchors/axes,
// velocity and force vectors, centers of mass, and per-island coloring.
//
// This header layers a small interface on top of those line lists so any
// renderer — OpenGL, Vulkan, Dear ImGui, a software rasterizer, or a headless
// test harness — can consume the same data:
//
//     struct MyDraw : velox::DebugDraw {
//         void drawLine(velox::Vec3 a, velox::Vec3 b, uint32_t color) override {
//             // forward (a, b, color) to your renderer of choice
//         }
//     };
//
//     MyDraw draw;
//     draw.setFlags(velox::DebugDrawEverything);
//     velox::drawWorld(draw, world);   // call once per frame after world.step()
//
// The interface is intentionally tiny: implement the single pure drawLine();
// drawPoint/drawTriangle/drawArrow have line-based defaults you can override
// when your renderer has faster batched primitives.

#include "world.h"

#include <cmath>
#include <cstdint>
#include <vector>

namespace velox {

// --- color helpers ----------------------------------------------------------
// DebugLine stores colors packed as 0xRRGGBBAA. These helpers convert to and
// from that layout so renderers can map colors onto their own vertex format.

constexpr uint32_t debugColor(uint8_t r, uint8_t g, uint8_t b, uint8_t a = 255) {
    return (uint32_t(r) << 24) | (uint32_t(g) << 16) | (uint32_t(b) << 8) | uint32_t(a);
}

struct DebugRgba {
    uint8_t r = 255, g = 255, b = 255, a = 255;
};

inline DebugRgba unpackDebugColor(uint32_t color) {
    return DebugRgba{uint8_t(color >> 24), uint8_t(color >> 16),
                     uint8_t(color >> 8), uint8_t(color)};
}

// Normalized-float variant for renderers that prefer [0,1] channels.
inline void unpackDebugColorF(uint32_t color, float& r, float& g, float& b, float& a) {
    constexpr float inv = 1.0f / 255.0f;
    r = float(color >> 24) * inv;
    g = float((color >> 16) & 0xffu) * inv;
    b = float((color >> 8) & 0xffu) * inv;
    a = float(color & 0xffu) * inv;
}

// --- the interface ----------------------------------------------------------
// Sink for debug primitives in world space. Subclass and implement drawLine();
// everything else has a working default.
class DebugDraw {
public:
    virtual ~DebugDraw() = default;

    // Required: one colored line segment. All World layers decompose to lines,
    // so implementing just this method is enough to render every flag.
    virtual void drawLine(Vec3 a, Vec3 b, uint32_t color) = 0;

    // Optional richer primitives. The defaults decompose into drawLine();
    // override them when your renderer can draw points/triangles/arrows more
    // efficiently (e.g. instanced quads or a triangle batch).
    virtual void drawPoint(Vec3 p, float size, uint32_t color) {
        drawLine(p - Vec3{size, 0, 0}, p + Vec3{size, 0, 0}, color);
        drawLine(p - Vec3{0, size, 0}, p + Vec3{0, size, 0}, color);
        drawLine(p - Vec3{0, 0, size}, p + Vec3{0, 0, size}, color);
    }

    virtual void drawTriangle(Vec3 a, Vec3 b, Vec3 c, uint32_t color) {
        drawLine(a, b, color);
        drawLine(b, c, color);
        drawLine(c, a, color);
    }

    virtual void drawArrow(Vec3 from, Vec3 to, uint32_t color) {
        drawLine(from, to, color);
        Vec3 dir = to - from;
        float len = std::sqrt(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z);
        if (len < 1e-6f) return;
        dir = dir * (1.0f / len);
        Vec3 ref = std::fabs(dir.x) < 0.9f ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
        Vec3 u = normalize(cross(dir, ref));
        Vec3 v = cross(dir, u);
        float head = len * 0.25f;
        head = head < 0.03f ? 0.03f : (head > 0.2f ? 0.2f : head);
        Vec3 base = to - dir * head;
        drawLine(to, base + u * head * 0.5f, color);
        drawLine(to, base - u * head * 0.5f, color);
        drawLine(to, base + v * head * 0.5f, color);
        drawLine(to, base - v * head * 0.5f, color);
    }

    // Optional 3D-anchored text label; no-op by default.
    virtual void drawText(Vec3 /*position*/, const char* /*text*/, uint32_t /*color*/) {}

    // --- flag management (Box2D-style) -------------------------------------
    // drawWorld(draw, world) renders whichever layers these flags select.
    void setFlags(uint32_t flags) { flags_ = flags; }
    uint32_t flags() const { return flags_; }
    void appendFlags(uint32_t flags) { flags_ |= flags; }
    void clearFlags(uint32_t flags) { flags_ &= ~flags; }

private:
    uint32_t flags_ = DebugDrawAll;
};

// --- bridge from a World to a DebugDraw -------------------------------------
// Renders the requested layers by asking the World for its line list and
// forwarding each segment to the interface. Call once per frame, after step().
inline void drawWorld(DebugDraw& draw, const World& world, uint32_t flags) {
    std::vector<DebugLine> lines;
    world.debugLines(lines, flags);
    for (const DebugLine& line : lines)
        draw.drawLine(line.a, line.b, line.color);
}

// Convenience overload that uses the flags stored on the DebugDraw instance.
inline void drawWorld(DebugDraw& draw, const World& world) {
    drawWorld(draw, world, draw.flags());
}

// --- reference implementation ----------------------------------------------
// Accumulates every primitive into a DebugLine vector. Useful for headless
// tools and tests, and as a minimal starting point for a real renderer.
class LineBatchDebugDraw : public DebugDraw {
public:
    std::vector<DebugLine> lines;

    void drawLine(Vec3 a, Vec3 b, uint32_t color) override {
        lines.push_back(DebugLine{a, b, color});
    }

    void clear() { lines.clear(); }
    std::size_t size() const { return lines.size(); }
    bool empty() const { return lines.empty(); }

    // Axis-aligned bounds of everything drawn so far; handy for framing a camera.
    void bounds(Vec3& outMin, Vec3& outMax) const {
        if (lines.empty()) {
            outMin = outMax = Vec3{};
            return;
        }
        outMin = outMax = lines.front().a;
        auto eat = [&](Vec3 p) {
            outMin = vmin(outMin, p);
            outMax = vmax(outMax, p);
        };
        for (const DebugLine& line : lines) {
            eat(line.a);
            eat(line.b);
        }
    }
};

} // namespace velox
