#include "velox/velox.h"
#include <cmath>
#include <cstdio>
#include <vector>

namespace {
bool check(bool value, const char* message) {
    if (!value) std::printf("runtime_mutation_demo: FAIL: %s\n", message);
    return value;
}
}

int main() {
    using namespace velox;
    World world(BackendType::Cpu);
    world.addStaticPlane({0, 1, 0}, 0.0f);
    const BodyId body = world.addSphere({0, 3, 0}, 0.5f, 1.0f);
    const BodyId anchor = world.addBox({0, 6, 0}, {0.25f, 0.25f, 0.25f}, 0.0f);
    const JointId hinge = world.addHingeJoint(anchor, body, {0, 4.5f, 0}, {0, 0, 1});

    ShapeMutation box;
    box.type = ShapeMutation::Type::Box;
    box.halfExtents = {0.3f, 0.6f, 0.4f};
    world.mutateShape(body, box);
    Body changed = world.bodyState(body);
    bool ok = check(changed.shape == ShapeType::Box, "sphere did not become box") &&
              check(std::fabs(changed.halfExtents.y - 0.6f) < 1e-6f,
                    "box extents were not applied") &&
              check(world.isValid(hinge), "mutation invalidated attached hinge") &&
              check(std::isfinite(world.hingeAngle(hinge)), "hinge query became invalid");

    const float oldInvInertia = changed.invInertia.x;
    world.scaleShape(body, {{2, 1, 0.5f}, true});
    changed = world.bodyState(body);
    ok &= check(std::fabs(changed.halfExtents.x - 0.6f) < 1e-6f &&
                std::fabs(changed.halfExtents.z - 0.2f) < 1e-6f,
                "anisotropic box scale failed");
    ok &= check(std::fabs(changed.invInertia.x - oldInvInertia) > 1e-4f,
                "scale did not refresh inertia");

    world.setCollisionMargin(body, 0.05f);
    ok &= check(std::fabs(world.bodyState(body).ccdTuning.collisionMargin - 0.05f) < 1e-6f,
                "collision margin update failed");
    world.step(1.0f / 60.0f);
    std::vector<BodyId> overlaps;
    world.overlapBox(world.bodyState(body).position, changed.halfExtents,
                     changed.orientation, overlaps);
    bool found = false;
    for (BodyId id : overlaps) found |= id == body;
    ok &= check(found, "broad phase retained stale shape proxy after mutation");

    std::printf("runtime_mutation_demo: %s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
