// ragdoll_chain.cpp — a chain of ragdolls linked together by constraints.
//
// Physics demonstrated:
//   * RagdollBuilder assembles each humanoid from rigid bones (capsules /
//     boxes / spheres) joined by cone-twist constraints with anatomically
//     limited swing + twist, so elbows/knees/necks bend like joints rather
//     than freely rotating ball sockets.
//   * Ragdolls are then chained hand-to-hand with rope joints (maximum
//     distance constraints with slack) and the lead ragdoll is hung from a
//     static ceiling anchor, producing a dangling, swinging chain.
//   * An initial impulse sets the chain swinging; gravity + joint limits
//     dissipate the motion into a stable hang.
//
// The simulation runs on a fixed 60 FPS timestep and prints an ASCII side
// view every half second so the swinging chain is visible in the console.
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

// A compact 6-bone humanoid: torso (root), head, two arms, two legs.
// `origin` is the torso center. Hands/feet are the limb endpoints used to
// chain ragdolls together.
struct Humanoid {
    BodyId torso, head, armL, armR, legL, legR;
    Vec3 handL, handR; // world-space grip points at build time
};

Humanoid makeHumanoid(World& world, Vec3 o) {
    Humanoid h;
    h.torso = world.addBox(o, {0.18f, 0.32f, 0.12f}, 6.0f);
    h.head = world.addSphere(o + Vec3{0, 0.55f, 0}, 0.16f, 1.0f);
    h.armL = world.addCapsule(o + Vec3{-0.30f, -0.05f, 0}, 0.07f, 0.28f, 0.7f);
    h.armR = world.addCapsule(o + Vec3{0.30f, -0.05f, 0}, 0.07f, 0.28f, 0.7f);
    h.legL = world.addCapsule(o + Vec3{-0.12f, -0.75f, 0}, 0.09f, 0.32f, 1.2f);
    h.legR = world.addCapsule(o + Vec3{0.12f, -0.75f, 0}, 0.09f, 0.32f, 1.2f);

    // Cone-twist links rooted at the torso form a directed tree. The swing
    // limit caps how far each limb can fan out; the twist limit caps spin.
    const std::vector<RagdollBone> bones{
        {h.torso, {}, 6.0f}, {h.head, {}, 1.0f}, {h.armL, {}, 0.7f},
        {h.armR, {}, 0.7f},  {h.legL, {}, 1.2f}, {h.legR, {}, 1.2f},
    };
    const std::vector<RagdollJoint> links{
        {h.torso, h.head, o + Vec3{0, 0.40f, 0}, {0, 1, 0}, 0.5f, 0.4f},
        {h.torso, h.armL, o + Vec3{-0.22f, 0.25f, 0}, {1, 0, 0}, 1.2f, 0.4f},
        {h.torso, h.armR, o + Vec3{0.22f, 0.25f, 0}, {1, 0, 0}, 1.2f, 0.4f},
        {h.torso, h.legL, o + Vec3{-0.12f, -0.35f, 0}, {0, 1, 0}, 0.7f, 0.4f},
        {h.torso, h.legR, o + Vec3{0.12f, -0.35f, 0}, {0, 1, 0}, 0.7f, 0.4f},
    };
    RagdollBuilder::Build(world, bones, links);

    // Grip points sit just past the shoulder/hip so the rope has a little
    // slack between neighbours.
    h.handL = o + Vec3{-0.30f, -0.40f, 0};
    h.handR = o + Vec3{0.30f, -0.40f, 0};
    return h;
}

// Minimal ASCII side view (X right, Y up) for console visualization.
struct SideView {
    int w, h;
    float minX, maxX, minY, maxY;
    std::vector<char> grid;
    SideView(int w, int h, float minX, float maxX, float minY, float maxY)
        : w(w), h(h), minX(minX), maxX(maxX), minY(minY), maxY(maxY),
          grid(static_cast<size_t>(w) * h, ' ') {}
    void clear() { std::fill(grid.begin(), grid.end(), ' '); }
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
    world.substeps = 6; // stiffer joints for a clean hang

    constexpr int kCount = 3;          // number of chained ragdolls
    constexpr float kSpacing = 1.1f;   // horizontal gap between ragdolls
    const float ceilingY = 6.0f;

    // Static ceiling anchor for the lead ragdoll.
    const BodyId ceiling = world.addSphere({0, ceilingY, 0}, 0.15f, 0.0f);

    std::vector<Humanoid> ragdolls;
    for (int i = 0; i < kCount; ++i)
        ragdolls.push_back(makeHumanoid(world, {i * kSpacing, ceilingY - 1.5f, 0}));

    // Hang the lead ragdoll's torso from the ceiling with a rope (allows swing).
    {
        const Vec3 torsoPos = world.body(ragdolls[0].torso).position;
        const JointId hang = world.addRopeJoint(
            ceiling, ragdolls[0].torso, {0, ceilingY, 0}, torsoPos,
            length(Vec3{0, ceilingY, 0} - torsoPos) + 0.05f);
        world.joint(hang).collideConnected = false;
    }

    // Chain neighbours: right hand of ragdoll i -> left hand of ragdoll i+1.
    std::vector<JointId> chainJoints;
    for (int i = 0; i + 1 < kCount; ++i) {
        const float gap = length(ragdolls[i].handR - ragdolls[i + 1].handL);
        const JointId link = world.addRopeJoint(
            ragdolls[i].armR, ragdolls[i + 1].armL,
            ragdolls[i].handR, ragdolls[i + 1].handL, gap + 0.15f);
        world.joint(link).collideConnected = false;
        chainJoints.push_back(link);
    }

    // Kick the tail ragdoll sideways so the whole chain swings.
    world.addLinearImpulse(ragdolls.back().torso, {6.0f, 0.0f, 0.0f});
    world.addLinearImpulse(ragdolls.back().legL, {3.0f, 0.0f, 0.0f});

    const float restX = (kCount - 1) * kSpacing * 0.5f; // chain centroid at rest
    float maxSwing = 0.0f;

    const int frames = 600; // 10 s at 60 FPS
    for (int frame = 0; frame < frames; ++frame) {
        world.step(1.0f / 60.0f);

        // Track the swing amplitude of the chain centroid about its rest X.
        Vec3 centroid{};
        int n = 0;
        for (const Humanoid& h : ragdolls) {
            centroid += world.body(h.torso).position;
            ++n;
        }
        centroid = centroid * (1.0f / float(n));
        maxSwing = vmax(maxSwing, std::fabs(centroid.x - restX));

        if (frame % 30 == 0) {
            SideView view(60, 18, -2.0f, kCount * kSpacing + 1.0f, -1.0f,
                          ceilingY + 0.5f);
            view.put(0, ceilingY, '#'); // ceiling anchor
            for (const Humanoid& h : ragdolls) {
                view.put(world.body(h.torso).position.x,
                         world.body(h.torso).position.y, 'O');
                view.put(world.body(h.head).position.x,
                         world.body(h.head).position.y, 'o');
                view.put(world.body(h.armL).position.x,
                         world.body(h.armL).position.y, '-');
                view.put(world.body(h.armR).position.x,
                         world.body(h.armR).position.y, '-');
                view.put(world.body(h.legL).position.x,
                         world.body(h.legL).position.y, '/');
                view.put(world.body(h.legR).position.x,
                         world.body(h.legR).position.y, '\\');
            }
            std::printf("t=%4.1fs  swing=%.2f\n%s", frame / 60.0f, maxSwing,
                        view.render().c_str());
        }
    }

    // --- Validate ---------------------------------------------------------
    int failures = 0;
    const auto check = [&](bool cond, const char* label) {
        std::printf("%s: %s\n", cond ? "PASS" : "FAIL", label);
        if (!cond) ++failures;
    };

    bool allFinite = true, allBonesValid = true;
    for (const Humanoid& h : ragdolls) {
        const BodyId bones[] = {h.torso, h.head, h.armL, h.armR, h.legL, h.legR};
        for (BodyId b : bones) {
            allBonesValid = allBonesValid && world.isValid(b);
            allFinite = allFinite && finite(world.body(b).position);
        }
    }
    bool chainValid = true;
    for (JointId j : chainJoints) chainValid = chainValid && world.isValid(j);

    check(allBonesValid && allFinite,
          "all ragdoll bones stay valid and finite while chained");
    check(chainValid, "hand-to-hand chain joints remain intact");
    check(maxSwing > 0.2f, "the impulse makes the chain swing");
    // After 10 s the hang should have settled near the rest position.
    Vec3 finalCentroid{};
    for (const Humanoid& h : ragdolls)
        finalCentroid += world.body(h.torso).position;
    finalCentroid = finalCentroid * (1.0f / float(kCount));
    check(std::fabs(finalCentroid.x - restX) < 1.0f,
          "the chain settles back near its rest position");

    if (failures) {
        std::fprintf(stderr, "ragdoll_chain: %d failure(s)\n", failures);
        return 1;
    }
    std::puts("ragdoll_chain: all checks passed");
    return 0;
}
