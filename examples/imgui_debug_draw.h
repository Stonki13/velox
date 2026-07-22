#pragma once
// Dear ImGui backend for velox::DebugDraw.
//
// Header-only and optional: it only compiles real code when <imgui.h> is
// reachable, so including it in a project without ImGui is harmless. It renders
// the World's debug line lists into an ImDrawList through a tiny software
// orbit-camera projection — no GPU backend of its own is required, which makes
// it easy to drop on top of any existing ImGui frame.
//
// Usage inside your frame, after world.step():
//
//     static velox::ImGuiDebugDraw draw;
//     draw.camera.target  = {0, 1, 0};
//     draw.camera.distance = 12.0f;
//     draw.setFlags(velox::DebugDrawEverything);
//     draw.begin(ImGui::GetBackgroundDrawList(), io.DisplaySize);
//     velox::drawWorld(draw, world);   // forwards every enabled layer
//     draw.end();
//
// Wire the camera to your input however you like, e.g.:
//     draw.camera.yaw   += io.MouseDelta.x * 0.005f;
//     draw.camera.pitch += io.MouseDelta.y * 0.005f;

#include <velox/debug_draw.h>

#if defined(__has_include)
#  if __has_include(<imgui.h>)
#    define VELOX_HAVE_IMGUI 1
#  endif
#endif

#ifdef VELOX_HAVE_IMGUI

#include <imgui.h>

#include <cmath>

namespace velox {

// Minimal orbit camera: looks at `target` from `distance` away, oriented by
// yaw/pitch, with a vertical field of view. Projects world-space points into
// ImGui pixel coordinates (top-left origin).
struct DebugCamera {
    Vec3 target{0, 0, 0};
    float yaw = 0.6f;        // radians, around world +Y
    float pitch = 0.4f;      // radians, above the ground plane
    float distance = 10.0f;
    float fovRadians = 1.0f; // vertical field of view
    float nearPlane = 0.05f;

    Vec3 eye() const {
        float cp = std::cos(pitch), sp = std::sin(pitch);
        float cy = std::cos(yaw), sy = std::sin(yaw);
        Vec3 dir{cp * sy, sp, cp * cy};
        return target - dir * distance;
    }

    // Camera-space basis. forward points from the eye toward the target.
    void basis(Vec3& forward, Vec3& right, Vec3& up) const {
        Vec3 e = eye();
        forward = normalize(target - e);
        Vec3 worldUp{0, 1, 0};
        right = normalize(cross(forward, worldUp));
        up = cross(right, forward);
    }

    // Project a world point. Returns false when the point is behind the near
    // plane; otherwise writes pixel coordinates into outX/outY.
    bool project(Vec3 p, float screenWidth, float screenHeight,
                 float& outX, float& outY, float& outDepth) const {
        Vec3 forward, right, up;
        basis(forward, right, up);
        Vec3 d = p - eye();
        float z = dot(d, forward);
        outDepth = z;
        if (z <= nearPlane) return false;
        float x = dot(d, right);
        float y = dot(d, up);
        float f = 1.0f / std::tan(fovRadians * 0.5f);
        float aspect = screenHeight > 0.0f ? screenWidth / screenHeight : 1.0f;
        float ndcX = (f / aspect) * (x / z);
        float ndcY = f * (y / z);
        outX = (ndcX * 0.5f + 0.5f) * screenWidth;
        outY = (1.0f - (ndcY * 0.5f + 0.5f)) * screenHeight;
        return true;
    }
};

// velox::DebugDraw implementation that rasterizes lines into an ImDrawList.
class ImGuiDebugDraw : public DebugDraw {
public:
    DebugCamera camera;
    float thickness = 1.0f;

    // Point the renderer at the draw list and viewport for this frame.
    void begin(ImDrawList* list, ImVec2 displaySize) {
        list_ = list;
        width_ = displaySize.x;
        height_ = displaySize.y;
        drawnLines_ = 0;
    }

    void end() { list_ = nullptr; }

    // Number of screen-space line segments emitted during the last frame.
    int drawnLines() const { return drawnLines_; }

    void drawLine(Vec3 a, Vec3 b, uint32_t color) override {
        if (!list_) return;
        float ax, ay, az, bx, by, bz;
        bool va = camera.project(a, width_, height_, ax, ay, az);
        bool vb = camera.project(b, width_, height_, bx, by, bz);
        if (!va && !vb) return; // wholly behind the camera

        // Clip the segment against the near plane so lines that straddle the
        // camera do not produce wild full-screen artifacts.
        if (!va || !vb) {
            Vec3 near_, far_;
            float nx, ny, nz, fx, fy, fz;
            if (va) { near_ = a; far_ = b; nx = ax; ny = ay; nz = az; fx = bx; fy = by; fz = bz; }
            else    { near_ = b; far_ = a; nx = bx; ny = by; nz = bz; fx = ax; fy = ay; fz = az; }
            // Interpolate to the point where depth == nearPlane.
            Vec3 forward, right, up;
            camera.basis(forward, right, up);
            float dn = dot(near_ - camera.eye(), forward);
            float df = dot(far_ - camera.eye(), forward);
            float t = (camera.nearPlane - dn) / (df - dn);
            Vec3 clipped = near_ + (far_ - near_) * t;
            float cx, cy, cz;
            if (!camera.project(clipped, width_, height_, cx, cy, cz)) return;
            if (va) { bx = cx; by = cy; } else { ax = cx; ay = cy; }
            (void)nx; (void)ny; (void)nz; (void)fx; (void)fy; (void)fz;
        }

        list_->AddLine(ImVec2(ax, ay), ImVec2(bx, by), toImU32(color), thickness);
        ++drawnLines_;
    }

    void drawPoint(Vec3 p, float size, uint32_t color) override {
        if (!list_) return;
        float x, y, depth;
        if (!camera.project(p, width_, height_, x, y, depth)) return;
        // Scale the marker down with distance so it reads as a fixed world size.
        float r = size * (height_ * 0.5f) / (depth > camera.nearPlane ? depth : camera.nearPlane);
        if (r < 1.0f) r = 1.0f;
        list_->AddCircleFilled(ImVec2(x, y), r, toImU32(color));
    }

private:
    // DebugLine colors are 0xRRGGBBAA; ImGui wants 0xAABBGGRR.
    static ImU32 toImU32(uint32_t color) {
        uint32_t r = (color >> 24) & 0xffu;
        uint32_t g = (color >> 16) & 0xffu;
        uint32_t b = (color >> 8) & 0xffu;
        uint32_t a = color & 0xffu;
        return IM_COL32(r, g, b, a);
    }

    ImDrawList* list_ = nullptr;
    float width_ = 0.0f, height_ = 0.0f;
    int drawnLines_ = 0;
};

} // namespace velox

#endif // VELOX_HAVE_IMGUI
