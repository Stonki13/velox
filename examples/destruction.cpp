// destruction.cpp — breakable objects that shatter into debris.
//
// Physics demonstrated:
//   * Weld joints hold a tower of blocks together as a single rigid body.
//     Each weld carries an explicit breakForce / breakTorque threshold; when
//     the solver's reaction across the weld exceeds it, the joint is removed
//     and a JointBreakEvent is emitted (World::jointBreakEvents).
//   * World::explode applies a radially falloff impulse to every dynamic body
//     in range — a convenient blast source. The shock overloads the lower
//     welds first, so the tower fractures and topples.
//   * On every break event we spawn small debris fragments at the fracture
//     point and fling them outward, turning one rigid tower into a scattering
//     pile of independent rigid bodies.
//
// Fixed 60 FPS timestep; an ASCII side view is printed every half second.
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
    world.substeps = 6; // stiff welds before they break

    world.addStaticPlane({0, 1, 0}, 0.0f);

    // --- Build a tower of blocks welded into one rigid structure ----------
    constexpr int kBlocks = 8;
    const Vec3 half{0.5f, 0.25f, 0.5f};
    std::vector<BodyId> blocks;
    for (int i = 0; i < kBlocks; ++i)
        blocks.push_back(world.addBox({0.0f, 0.25f + i * 0.5f, 0.0f}, half, 2.0f));

    // Breakable welds between consecutive blocks. The thresholds are chosen so
    // a strong blast overloads the lower joints but the tower holds at rest.
    std::vector<JointId> welds;
    for (int i = 0; i + 1 < kBlocks; ++i) {
        const Vec3 anchor{0.0f, 0.5f + i * 0.5f, 0.0f}; // shared face
        welds.push_back(world.addWeldJoint(blocks[i], blocks[i + 1], anchor,
                                           /*breakForce=*/700.0f,
                                           /*breakTorque=*/350.0f));
    }
    const int initialWelds = int(welds.size());

    // Let the tower settle so the welds start un-stressed.
    for (int i = 0; i < 60; ++i) world.step(1.0f / 60.0f);

    // --- Detonate a blast at the base of the tower ------------------------
    world.explode({1.2f, 0.4f, 0.0f}, /*radius=*/3.5f, /*impulse=*/45.0f);

    std::vector<BodyId> debris;
    int brokenCount = 0;
    float maxScatter = 0.0f;

    const int frames = 480; // 8 s at 60 FPS
    for (int frame = 0; frame < frames; ++frame) {
        world.step(1.0f / 60.0f);

        // Fracture handling: spawn debris at each freshly broken weld.
        for (const JointBreakEvent& ev : world.jointBreakEvents()) {
            ++brokenCount;
            if (!world.isValid(ev.a) || !world.isValid(ev.b)) continue;
            const Vec3 mid = (world.body(ev.a).position +
                              world.body(ev.b).position) * 0.5f;
            // Two fragments per fracture, flung away from the tower axis.
            for (int k = 0; k < 2; ++k) {
                const float dir = (k == 0) ? 1.0f : -1.0f;
                const BodyId frag = world.addBox(
                    mid + Vec3{0.2f * dir, 0.1f, 0.0f}, {0.12f, 0.12f, 0.12f},
                    0.3f);
                world.setLinearVelocity(frag, {4.0f * dir, 3.0f, 1.5f * dir});
                world.setAngularVelocity(frag, {6.0f * dir, 4.0f, -5.0f * dir});
                debris.push_back(frag);
            }
        }

        // Track how far the blocks have scattered from the tower axis (x=0).
        for (BodyId b : blocks)
            maxScatter = vmax(maxScatter,
                              std::fabs(world.body(b).position.x));

        if (frame % 30 == 0) {
            SideView view(56, 16, -5.0f, 6.0f, 0.0f, 6.0f);
            view.ground(0.0f);
            for (int i = 0; i < kBlocks; ++i)
                view.put(world.body(blocks[i]).position.x,
                         world.body(blocks[i]).position.y,
                         char('0' + i));
            for (BodyId d : debris)
                view.put(world.body(d).position.x, world.body(d).position.y,
                         '*');
            std::printf("t=%4.1fs  welds broken=%d/%d  debris=%zu\n%s",
                        frame / 60.0f, brokenCount, initialWelds, debris.size(),
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
    for (BodyId b : blocks) allFinite = allFinite && finite(world.body(b).position);
    for (BodyId d : debris) allFinite = allFinite && finite(world.body(d).position);

    int stillValid = 0;
    for (JointId w : welds)
        if (world.isValid(w)) ++stillValid;

    check(allFinite, "all blocks and debris stay finite");
    check(brokenCount > 0, "the blast breaks at least one weld");
    check(brokenCount == initialWelds - stillValid,
          "broken events match the joints that were removed");
    check(debris.size() == size_t(brokenCount) * 2,
          "two debris fragments spawn per fracture");
    check(maxScatter > 1.0f, "fragments scatter away from the tower");

    if (failures) {
        std::fprintf(stderr, "destruction: %d failure(s)\n", failures);
        return 1;
    }
    std::puts("destruction: all checks passed");
    return 0;
}
