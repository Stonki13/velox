// CPU/CUDA parity + fallback check for the CUDA backend (Phase 2: profile
// before optimizing, then verify any transfer-path change did not change
// results). Two scenes exercise the two paths advanceSubsteps takes through
// solveVelocities: one with real contacts (device stays resident across the
// joint kernels, no re-upload needed) and one with joints only and zero
// contacts (solveVelocities never touches the device, so a re-upload before
// the joint kernels is required). Both must agree with the CPU backend
// within a loose behavioral tolerance -- this is a cross-backend numerical
// comparison, not a bitwise determinism check (see tests/difftest for that
// convention against Jolt).
#include <velox/velox.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

using namespace velox;

namespace {

// Both worlds see the identical sequence of add*() calls below, so BodyId
// slot/generation assignment (purely a function of insertion order) is
// identical across the two -- the returned ids are valid on either world.
std::vector<BodyId> buildBoxStack(World& world) {
    world.gravity = {0, -9.81f, 0};
    world.addStaticPlane({0, 1, 0}, 0.0f);
    std::vector<BodyId> ids;
    for (int i = 0; i < 6; ++i)
        ids.push_back(world.addBox({0, 1.0f + float(i) * 1.02f, 0}, {0.5f, 0.5f, 0.5f}, 1.0f));
    return ids;
}

std::vector<BodyId> buildJointFan(World& world) {
    world.gravity = {0, 0, 0};
    std::vector<BodyId> ids;
    BodyId anchor = world.addSphere({}, 0.1f, 0.0f);
    world.body(anchor).maskBits = 0;
    for (int i = 0; i < 80; ++i) {
        float x = 2.0f + float(i) * 0.5f;
        BodyId body = world.addSphere({x, 0, 0}, 0.1f, 1.0f);
        world.body(body).maskBits = 0;
        world.addDistanceJoint(anchor, body, {}, {x, 0, 0});
        world.setLinearVelocity(body, {0, 1, 0});
        ids.push_back(body);
    }
    return ids;
}

bool statesAgree(const World& a, const World& b, const std::vector<BodyId>& ids,
                 float positionTolerance, float velocityTolerance) {
    for (BodyId id : ids) {
        const Body& ba = a.body(id);
        const Body& bb = b.body(id);
        if (length(ba.position - bb.position) > positionTolerance) return false;
        if (length(ba.velocity - bb.velocity) > velocityTolerance) return false;
        if (!std::isfinite(ba.position.x) || !std::isfinite(bb.position.x)) return false;
    }
    return true;
}

// Runs `build` identically on CPU and CUDA (Auto) worlds for `steps` steps
// and checks the results agree. Returns true (skipping, not failing) when
// this machine has no CUDA device, matching cuda_recovery_demo's convention.
bool checkParity(const char* label, std::vector<BodyId> (*build)(World&), int steps,
                 float positionTolerance, float velocityTolerance) {
    World probe(BackendType::Auto);
    if (std::string(probe.backendName()) != "cuda") {
        std::printf("cuda_parity_demo: %s: CUDA unavailable; skipped\n", label);
        return true;
    }

    World cpu(BackendType::Cpu);
    World cuda(BackendType::Auto);
    build(cpu);
    std::vector<BodyId> ids = build(cuda);
    if (std::string(cuda.backendName()) != "cuda") {
        std::printf("cuda_parity_demo: %s: CUDA unavailable after scene build; skipped\n", label);
        return true;
    }

    for (int i = 0; i < steps; ++i) {
        cpu.step(1.0f / 60.0f);
        cuda.step(1.0f / 60.0f);
    }

    bool ok = statesAgree(cpu, cuda, ids, positionTolerance, velocityTolerance);
    if (std::getenv("VELOX_PARITY_DEBUG")) {
        float maxPos = 0.0f, maxVel = 0.0f;
        for (BodyId id : ids) {
            const Body& ba = cpu.body(id);
            const Body& bb = cuda.body(id);
            maxPos = std::max(maxPos, length(ba.position - bb.position));
            maxVel = std::max(maxVel, length(ba.velocity - bb.velocity));
        }
        std::printf("  max position delta=%.6f  max velocity delta=%.6f\n", maxPos, maxVel);
    }
    std::printf("cuda_parity_demo: %s: %s\n", label, ok ? "PASS" : "FAIL");
    return ok;
}

} // namespace

int main() {
    bool ok = true;
    // Dense contact scene: exercises the advanceSubsteps path where
    // solveVelocities already left dBodies_ current (contacts non-empty),
    // so the redundant re-upload this phase removed must stay a no-op.
    // Tolerance is loose (not a bitwise check): CPU and CUDA use different
    // contact orderings/solver code paths, and a settling stack is
    // chaotically sensitive to sub-ULP differences over many steps --
    // measured max divergence here is ~0.08 units after 90 steps.
    ok &= checkParity("box_stack (contacts, no joints)", buildBoxStack, 90,
                      0.15f, 0.5f);
    // Joint-only scene: contacts is always empty (masked out), so
    // solveVelocities takes its early return and advanceSubsteps must still
    // upload before the joint kernels read dBodies_.
    ok &= checkParity("joint_fan (joints, no contacts)", buildJointFan, 60,
                      0.05f, 0.25f);
    return ok ? 0 : 1;
}
