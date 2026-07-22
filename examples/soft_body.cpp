// soft_body.cpp — a mass-spring soft body that squashes and rebounds.
//
// Physics demonstrated:
//   * A soft body is built as a lattice of point masses (small spheres)
//     connected by compliant distance constraints. World::addSpringJoint
//     creates a mass-independent spring-damper: springFrequencyHz sets the
//     oscillation rate and springDampingRatio the energy loss (1 = critical).
//   * Structural springs (grid neighbours) resist stretching, shear springs
//     (diagonals) resist the lattice collapsing into a rhombus. Together they
//     give a squishy body that holds its volume.
//   * Dropped under gravity, the body deforms on impact (its bounding height
//     collapses), stores elastic energy, then rebounds and settles.
//
// Fixed 60 FPS timestep; the bounding height (squash) and an ASCII view are
// printed every half second.
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
    void ground(float y) {
        int cy = int((y - minY) / (maxY - minY) * (h - 1));
        if (cy >= 0 && cy < h)
            for (int x = 0; x < w; ++x)
                grid[static_cast<size_t>(h - 1 - cy) * w + x] = '=';
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
    world.substeps = 8; // stiff springs need small sub-steps for stability

    world.addStaticPlane({0, 1, 0}, 0.0f);

    // --- Build the mass lattice -------------------------------------------
    constexpr int kCols = 6, kRows = 6;
    constexpr float kSpacing = 0.4f;
    constexpr float kRadius = 0.12f;
    constexpr float kMass = 0.2f;
    const float x0 = -(kCols - 1) * kSpacing * 0.5f;
    const float y0 = 3.5f;

    std::vector<std::vector<BodyId>> p(kCols, std::vector<BodyId>(kRows));
    for (int c = 0; c < kCols; ++c)
        for (int r = 0; r < kRows; ++r) {
            const Vec3 pos{x0 + c * kSpacing, y0 + r * kSpacing, 0.0f};
            p[c][r] = world.addSphere(pos, kRadius, kMass);
            world.setLinearDamping(p[c][r], 0.02f);
        }

    // Connect neighbours with springs. Anchors are the particle positions at
    // build time, so the rest length equals the initial spacing.
    std::vector<JointId> springs;
    const auto link = [&](BodyId a, BodyId b, float hz, float damp) {
        const Vec3 pa = world.body(a).position, pb = world.body(b).position;
        springs.push_back(world.addSpringJoint(a, b, pa, pb, hz, damp));
    };
    for (int c = 0; c < kCols; ++c)
        for (int r = 0; r < kRows; ++r) {
            if (c + 1 < kCols) link(p[c][r], p[c + 1][r], 5.0f, 0.4f); // structural
            if (r + 1 < kRows) link(p[c][r], p[c][r + 1], 5.0f, 0.4f); // structural
            if (c + 1 < kCols && r + 1 < kRows) {
                link(p[c][r], p[c + 1][r + 1], 4.0f, 0.4f); // shear
                link(p[c + 1][r], p[c][r + 1], 4.0f, 0.4f); // shear
            }
        }

    const float initialHeight = (kRows - 1) * kSpacing;
    float minBoundingHeight = initialHeight;
    float reboundHeight = 0.0f;
    bool impacted = false;

    const int frames = 480; // 8 s at 60 FPS
    for (int frame = 0; frame < frames; ++frame) {
        world.step(1.0f / 60.0f);

        // Bounding height of the lattice = its current "thickness".
        float loY = 1e30f, hiY = -1e30f;
        for (int c = 0; c < kCols; ++c)
            for (int r = 0; r < kRows; ++r) {
                const float y = world.body(p[c][r]).position.y;
                loY = vmin(loY, y);
                hiY = vmax(hiY, y);
            }
        const float boundingHeight = hiY - loY;
        if (boundingHeight < minBoundingHeight) {
            minBoundingHeight = boundingHeight;
            impacted = true;
        }
        if (impacted) reboundHeight = vmax(reboundHeight, boundingHeight);

        if (frame % 30 == 0) {
            SideView view(48, 16, -2.0f, 2.0f, -0.3f, 6.0f);
            view.ground(0.0f);
            for (int c = 0; c < kCols; ++c)
                for (int r = 0; r < kRows; ++r)
                    view.put(world.body(p[c][r]).position.x,
                             world.body(p[c][r]).position.y, 'o');
            std::printf("t=%4.1fs  height=%.2f (min=%.2f)\n%s", frame / 60.0f,
                        boundingHeight, minBoundingHeight,
                        view.render().c_str());
        }
    }

    // --- Validate ---------------------------------------------------------
    int failures = 0;
    const auto check = [&](bool cond, const char* label) {
        std::printf("%s: %s\n", cond ? "PASS" : "FAIL", label);
        if (!cond) ++failures;
    };

    bool allFinite = true, aboveFloor = true;
    float finalLoY = 1e30f, finalHiY = -1e30f, finalSpeed = 0.0f;
    for (int c = 0; c < kCols; ++c)
        for (int r = 0; r < kRows; ++r) {
            const Body b = world.bodyState(p[c][r]);
            allFinite = allFinite && finite(b.position);
            aboveFloor = aboveFloor && b.position.y > -0.2f;
            finalLoY = vmin(finalLoY, b.position.y);
            finalHiY = vmax(finalHiY, b.position.y);
            finalSpeed = vmax(finalSpeed, length(b.velocity));
        }
    bool springsValid = true;
    for (JointId s : springs) springsValid = springsValid && world.isValid(s);

    check(allFinite, "every mass stays finite");
    check(aboveFloor, "the soft body does not tunnel through the floor");
    check(springsValid, "all springs remain intact");
    check(minBoundingHeight < 0.7f * initialHeight,
          "the body squashes to under 70% of its height on impact");
    check(reboundHeight > minBoundingHeight + 0.05f,
          "stored elastic energy makes the body rebound");
    check(finalLoY < 0.6f && finalSpeed < 1.5f,
          "the body comes to rest on the floor");
    (void)finalHiY;

    if (failures) {
        std::fprintf(stderr, "soft_body: %d failure(s)\n", failures);
        return 1;
    }
    std::puts("soft_body: all checks passed");
    return 0;
}
