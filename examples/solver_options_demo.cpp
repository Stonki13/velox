#include <velox/velox.h>

#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>

namespace {

using namespace velox;

bool finite(Vec3 value) {
    return std::isfinite(value.x) && std::isfinite(value.y) &&
           std::isfinite(value.z);
}

BodyId makeRestingWorld(World& world, const SolverOptions& options) {
    world.setWorkerCount(1);
    world.setSolverOptions(options);
    world.addStaticPlane({0, 1, 0}, 0.0f);
    return world.addBox({0, 0.5f, 0}, {0.5f, 0.5f, 0.5f}, 1.0f);
}

bool sameOptions(const SolverOptions& a, const SolverOptions& b) {
    return a.frictionModel == b.frictionModel &&
           a.iterationPolicy == b.iterationPolicy &&
           a.velocityIterations == b.velocityIterations &&
           a.positionIterations == b.positionIterations &&
           a.impulseThreshold == b.impulseThreshold &&
           a.stackStiffness == b.stackStiffness &&
           a.stackDamping == b.stackDamping &&
           a.enableStackStabilization == b.enableStackStabilization;
}

} // namespace

int main() {
    int failures = 0;
    const auto check = [&](bool condition, const char* label) {
        std::printf("%s: %s\n", condition ? "PASS" : "FAIL", label);
        if (!condition) ++failures;
    };

    SolverOptions fixed;
    fixed.velocityIterations = 8;
    fixed.positionIterations = 4;
    World fixedWorld(BackendType::Cpu);
    const BodyId fixedBox = makeRestingWorld(fixedWorld, fixed);
    fixedWorld.step(1.0f / 60.0f);

    SolverOptions adaptive = fixed;
    adaptive.iterationPolicy = IterationPolicy::Adaptive;
    adaptive.impulseThreshold = 0.1f;
    World adaptiveWorld(BackendType::Cpu);
    const BodyId adaptiveBox = makeRestingWorld(adaptiveWorld, adaptive);
    adaptiveWorld.step(1.0f / 60.0f);

    const float positionDelta = length(fixedWorld.body(fixedBox).position -
                                       adaptiveWorld.body(adaptiveBox).position);
    std::printf("  fixed sweeps=%u adaptive sweeps=%u rest delta=%.7f\n",
                fixedWorld.lastStepStats().velocityIterations,
                adaptiveWorld.lastStepStats().velocityIterations, positionDelta);
    check(adaptiveWorld.lastStepStats().velocityIterations <
              fixedWorld.lastStepStats().velocityIterations &&
              positionDelta < 2e-3f,
          "adaptive solver reduces ordered sweeps without changing rest state");

    SolverOptions cone = PresetHighQuality().options;
    cone.frictionModel = FrictionModel::ConeBlockSolver;
    World coneWorld(BackendType::Cpu);
    const BodyId coneBox = makeRestingWorld(coneWorld, cone);
    for (int frame = 0; frame < 120; ++frame)
        coneWorld.step(1.0f / 60.0f);
    check(finite(coneWorld.body(coneBox).position),
          "cone block friction solver stays finite");

    SolverOptions stacking = PresetStacking().options;
    World stackWorld(BackendType::Cpu);
    stackWorld.setSolverOptions(stacking);
    stackWorld.addStaticPlane({0, 1, 0}, 0.0f);
    BodyId stackTop;
    for (int i = 0; i < 12; ++i)
        stackTop = stackWorld.addBox({0, 0.5f + i * 1.0f, 0},
                                     {0.5f, 0.5f, 0.5f}, 1.0f);
    for (int frame = 0; frame < 240; ++frame)
        stackWorld.step(1.0f / 60.0f);
    check(finite(stackWorld.body(stackTop).position),
          "stack stabilization preset remains finite");

    const WorldSnapshot snapshot = adaptiveWorld.saveSnapshot();
    adaptiveWorld.setSolverOptions(PresetFast().options);
    adaptiveWorld.restoreSnapshot(snapshot);
    check(sameOptions(adaptiveWorld.solverOptions(), adaptive),
          "rollback snapshot restores solver options");

    const SerializedScene scene = serializeWorld(adaptiveWorld);
    World restored(BackendType::Cpu);
    deserializeWorld(restored, scene);
    check(sameOptions(restored.solverOptions(), adaptive),
          "serialized scene preserves solver options");

    bool rejected = false;
    try {
        SolverOptions invalid;
        invalid.velocityIterations = 0;
        restored.setSolverOptions(invalid);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    check(rejected, "invalid solver options are rejected");

    World autoWorld(BackendType::Auto);
    autoWorld.setSolverOptions(adaptive);
    check(std::string(autoWorld.backendName()) == "cpu",
          "adaptive policy uses the ordered CPU backend");
    autoWorld.setSolverOptions(cone);
    autoWorld.addStaticPlane({0, 1, 0}, 0.0f);
    const BodyId autoBox =
        autoWorld.addBox({0, 0.5f, 0}, {0.5f, 0.5f, 0.5f}, 1.0f);
    for (int frame = 0; frame < 30; ++frame)
        autoWorld.step(1.0f / 60.0f);
    check(finite(autoWorld.body(autoBox).position),
          "fixed cone solver executes through the selected backend");

    if (failures) {
        std::fprintf(stderr, "solver_options_demo: %d failure(s)\n", failures);
        return 1;
    }
    std::puts("solver_options_demo: all checks passed");
    return 0;
}
