// cloth.cpp — a cloth sheet that drapes over a sphere with collision.
//
// Physics demonstrated:
//   * Cloth is a grid of point masses joined by spring-damper constraints
//     (World::addSpringJoint). Structural springs along the weave resist
//     stretching; shear springs across the diagonals stop the grid shearing
//     into a rhombus, so the sheet keeps its area while staying flexible.
//   * The border of the sheet is pinned (static bodies). Gravity sags the
//     free interior until it contacts a static sphere, and the collision
//     solver conforms the cloth to the curved surface — a classic drape.
//   * The result is a tent-like depression wrapping the sphere: the centre
//     rests on the sphere's crown while the pinned border stays high.
//
// Fixed 60 FPS timestep; a side-view cross-section (with the sphere drawn as
// a circle) is printed every second.
#include <velox/velox.h>

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace {

using namespace velox;

bool finite(Vec3 v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

struct SideView {
    int w, h;
    float minX, maxX, minY, maxY;
    std::vector<char> grid;
    SideView(int w, int h, float minX, float maxX, float minY, float maxY)
        : w(w), h(h), minX(minX), maxX(maxX), minY(minY), maxY(maxY),
          grid(static_cast<size_t>(w) * h, ' ') {}
    void put(float x, float y, char c) {
        int cx = int((x - minX) / (maxX - minX) * (w - 1));
        int cy = int((y - minY) / (maxY - minY) * (h - 1));
        if (cx >= 0 && cx < w && cy >= 0 && cy < h)
            grid[static_cast<size_t>(h - 1 - cy) * w + cx] = c;
    }
    std::string render() const {
        std::string s;
        s.reserve(static_cast<size_t>(w + 1) * h);
        for (int r = 0; r < h; ++r) {
            s.append(grid.begin() + static_cast<size_t>(r) * w,
                     grid.begin() + static_cast<size_t>(r + 1) * w);
            s.push_back('\n');
        }
        return s;
    }
};

} // namespace

int main() {
    World world(BackendType::Cpu);
    world.setGravity({0, -9.81f, 0});
    world.substeps = 8; // small sub-steps keep the spring cloth stable

    // --- The collision obstacle: a static sphere --------------------------
    const Vec3 sphereCenter{0.0f, 1.5f, 0.0f};
    const float sphereRadius = 1.0f;
    const BodyId obstacle = world.addSphere(sphereCenter, sphereRadius, 0.0f);

    // --- Build the cloth grid in the X/Z plane ----------------------------
    constexpr int kCols = 9, kRows = 9;
    constexpr float kSpacing = 0.35f;
    constexpr float kClothY = 3.0f;
    const float x0 = -(kCols - 1) * kSpacing * 0.5f;

    std::vector<std::vector<BodyId>> p(kCols, std::vector<BodyId>(kRows));
    for (int c = 0; c < kCols; ++c)
        for (int r = 0; r < kRows; ++r) {
            const bool border =
                (c == 0 || c == kCols - 1 || r == 0 || r == kRows - 1);
            const Vec3 pos{x0 + c * kSpacing, kClothY, x0 + r * kSpacing};
            // Border nodes are pinned (static); the interior is free to drape.
            p[c][r] = world.addSphere(pos, 0.06f, border ? 0.0f : 0.05f);
            if (!border) world.setLinearDamping(p[c][r], 0.05f);
        }

    // Weave the springs: structural (along the grid) + shear (diagonals).
    std::vector<JointId> springs;
    const auto link = [&](BodyId a, BodyId b, float hz) {
        const Vec3 pa = world.body(a).position, pb = world.body(b).position;
        springs.push_back(world.addSpringJoint(a, b, pa, pb, hz, 0.5f));
    };
    for (int c = 0; c < kCols; ++c)
        for (int r = 0; r < kRows; ++r) {
            if (c + 1 < kCols) link(p[c][r], p[c + 1][r], 6.0f);
            if (r + 1 < kRows) link(p[c][r], p[c][r + 1], 6.0f);
            if (c + 1 < kCols && r + 1 < kRows) {
                link(p[c][r], p[c + 1][r + 1], 5.0f);
                link(p[c + 1][r], p[c][r + 1], 5.0f);
            }
        }

    const int mid = kCols / 2;
    const int frames = 600; // 10 s at 60 FPS
    for (int frame = 0; frame < frames; ++frame) {
        world.step(1.0f / 60.0f);

        if (frame % 60 == 0) {
            // Cross-section through the centre row (z = 0).
            SideView view(56, 16, -2.0f, 2.0f, 0.5f, 3.6f);
            for (int a = 0; a <= 48; ++a) { // draw the sphere outline
                const float t = a / 48.0f * 6.2831853f;
                view.put(sphereCenter.x + sphereRadius * cosf(t),
                         sphereCenter.y + sphereRadius * sinf(t), '.');
            }
            for (int c = 0; c < kCols; ++c) // cloth centre-row profile
                view.put(world.body(p[c][mid]).position.x,
                         world.body(p[c][mid]).position.y, 'o');
            std::printf("t=%4.1fs  centre y=%.2f\n%s", frame / 60.0f,
                        world.body(p[mid][mid]).position.y,
                        view.render().c_str());
        }
    }

    // --- Validate ---------------------------------------------------------
    int failures = 0;
    const auto check = [&](bool cond, const char* label) {
        std::printf("%s: %s\n", cond ? "PASS" : "FAIL", label);
        if (!cond) ++failures;
    };

    bool allFinite = true;
    float maxInteriorSpeed = 0.0f;
    for (int c = 0; c < kCols; ++c)
        for (int r = 0; r < kRows; ++r) {
            const Body b = world.bodyState(p[c][r]);
            allFinite = allFinite && finite(b.position);
            const bool border =
                (c == 0 || c == kCols - 1 || r == 0 || r == kRows - 1);
            if (!border) maxInteriorSpeed = vmax(maxInteriorSpeed, length(b.velocity));
        }

    const Vec3 corner = world.bodyState(p[0][0]).position;
    const Vec3 centre = world.bodyState(p[mid][mid]).position;
    const float centreDist = length(centre - sphereCenter);
    bool springsValid = true;
    for (JointId s : springs) springsValid = springsValid && world.isValid(s);

    check(allFinite, "every cloth node stays finite");
    check(springsValid, "all weave springs remain intact");
    check(std::fabs(corner.y - kClothY) < 0.1f, "pinned border stays in place");
    check(centre.y < corner.y - 0.2f, "the interior sags below the border");
    check(centre.y > sphereCenter.y && centre.y < kClothY,
          "the centre rests on the sphere crown, not through it");
    check(centreDist > sphereRadius - 0.2f,
          "collision keeps the cloth on the sphere surface");
    check(maxInteriorSpeed < 0.6f, "the cloth settles into a steady drape");
    check(world.isValid(obstacle), "obstacle remains valid");

    if (failures) {
        std::fprintf(stderr, "cloth: %d failure(s)\n", failures);
        return 1;
    }
    std::puts("cloth: all checks passed");
    return 0;
}
