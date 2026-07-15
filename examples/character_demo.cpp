#include <velox/character.h>

#include <cmath>
#include <cstdio>
#include <cstring>

namespace {

using velox::CharacterController;
using velox::CharacterControllerDesc;
using velox::CharacterControllerResult;
using velox::Vec3;
using velox::World;

constexpr float kRadius = 0.3f;
constexpr float kHalfHeight = 0.9f;
constexpr float kStandingCenterY = kRadius + kHalfHeight;

bool expect(bool condition, const char* message) {
    if (!condition) std::fprintf(stderr, "character_demo: %s\n", message);
    return condition;
}

bool near(float actual, float expected, float tolerance = 0.03f) {
    return std::fabs(actual - expected) <= tolerance;
}

bool sameBits(Vec3 a, Vec3 b) {
    return std::memcmp(&a, &b, sizeof(Vec3)) == 0;
}

CharacterControllerDesc controllerDesc() {
    CharacterControllerDesc desc;
    desc.capsuleRadius = kRadius;
    desc.capsuleHalfHeight = kHalfHeight;
    desc.stepMaxHeight = 0.3f;
    desc.slopeLimitCosine = 0.7071f;
    desc.ghostPadding = 0.01f;
    return desc;
}

bool testFlatWalk() {
    World world(velox::BackendType::Cpu);
    world.addStaticPlane({0, 1, 0}, 0.0f);
    CharacterController controller(world, controllerDesc());
    controller.SetPosition({0, kStandingCenterY, 0});
    CharacterControllerResult result = controller.Move({10, 0, 0});
    return expect(result.grounded, "flat walk must report grounded") &&
           expect(near(result.finalPosition.x, 10.0f), "flat walk must cover displacement") &&
           expect(near(result.finalPosition.y, kStandingCenterY), "flat walk must retain height");
}

bool testStepClimb() {
    World world(velox::BackendType::Cpu);
    world.addStaticPlane({0, 1, 0}, 0.0f);
    world.addBox({2.0f, 0.125f, 0}, {0.4f, 0.125f, 2.0f}, 0.0f);
    CharacterController controller(world, controllerDesc());
    controller.SetPosition({0, kStandingCenterY, 0});
    CharacterControllerResult result = controller.Move({4, 0, 0});
    return expect(result.stepped, "0.25 m obstacle must be climbed") &&
           expect(result.grounded, "step climb must finish grounded") &&
           expect(result.finalPosition.x > 3.8f, "step climb must cross obstacle");
}

bool testWallSlide() {
    World world(velox::BackendType::Cpu);
    world.addStaticPlane({0, 1, 0}, 0.0f);
    world.addBox({2.0f, 2.0f, 0}, {0.1f, 2.0f, 5.0f}, 0.0f);
    CharacterController controller(world, controllerDesc());
    controller.SetPosition({0, kStandingCenterY, 0});
    CharacterControllerResult result = controller.Move({3, 0, 3});
    return expect(result.hitWall, "diagonal wall impact must report wall") &&
           expect(result.grounded, "wall slide must remain grounded") &&
           expect(result.finalPosition.x < 1.65f, "wall slide must not pass through wall") &&
           expect(result.finalPosition.z > 1.5f, "wall slide must retain tangential movement");
}

bool testSteepSlopeSlip() {
    World world(velox::BackendType::Cpu);
    const float radians = 50.0f * 3.14159265358979323846f / 180.0f;
    world.addStaticPlane({-std::sin(radians), std::cos(radians), 0}, 0.0f);
    CharacterController controller(world, controllerDesc());
    controller.SetPosition({0, kHalfHeight + kRadius / std::cos(radians) + 0.01f, 0});
    CharacterControllerResult result = controller.Move({0, -0.5f, 0});
    return expect(!result.grounded, "50 degree slope must not be grounded") &&
           expect(result.finalPosition.x < -0.02f, "steep slope must slide backward");
}

bool testCornerNoEnergyAndDeterminism() {
    World world(velox::BackendType::Cpu);
    world.addStaticPlane({0, 1, 0}, 0.0f);
    world.addBox({2.0f, 2.0f, 0}, {0.1f, 2.0f, 5.0f}, 0.0f);
    world.addBox({0, 2.0f, 2.0f}, {5.0f, 2.0f, 0.1f}, 0.0f);
    const velox::WorldSnapshot snapshot = world.saveSnapshot();

    CharacterController first(world, controllerDesc());
    first.SetPosition({0, kStandingCenterY, 0});
    CharacterControllerResult a = first.Move({5, 0, 5});
    world.restoreSnapshot(snapshot);
    CharacterController second(world, controllerDesc());
    second.SetPosition({0, kStandingCenterY, 0});
    CharacterControllerResult b = second.Move({5, 0, 5});

    return expect(a.finalPosition.x < 1.65f && a.finalPosition.z < 1.65f,
                  "corner must block both normal components") &&
           expect(velox::length(a.slideVelocity) < 2.5f,
                  "corner must not preserve blocked movement as energy") &&
           expect(sameBits(a.finalPosition, b.finalPosition) &&
                  sameBits(a.slideVelocity, b.slideVelocity) &&
                  a.grounded == b.grounded && a.stepped == b.stepped && a.hitWall == b.hitWall,
                  "same snapshot and displacement must be bitwise deterministic");
}

bool testTallWallAndCeilingRejectSteps() {
    World tallWall(velox::BackendType::Cpu);
    tallWall.addStaticPlane({0, 1, 0}, 0.0f);
    tallWall.addBox({2.0f, 2.0f, 0}, {0.1f, 2.0f, 2.0f}, 0.0f);
    CharacterController wallController(tallWall, controllerDesc());
    wallController.SetPosition({0, kStandingCenterY, 0});
    CharacterControllerResult wall = wallController.Move({4, 0, 0});

    World ceilingWorld(velox::BackendType::Cpu);
    ceilingWorld.addStaticPlane({0, 1, 0}, 0.0f);
    ceilingWorld.addBox({2.0f, 0.125f, 0}, {0.4f, 0.125f, 2.0f}, 0.0f);
    ceilingWorld.addBox({2.0f, 2.65f, 0}, {1.0f, 0.10f, 2.0f}, 0.0f);
    CharacterController ceilingController(ceilingWorld, controllerDesc());
    ceilingController.SetPosition({0, kStandingCenterY, 0});
    CharacterControllerResult ceiling = ceilingController.Move({4, 0, 0});

    return expect(!wall.stepped && wall.finalPosition.x < 1.65f,
                  "tall wall must not be treated as a step") &&
           expect(!ceiling.stepped && ceiling.finalPosition.x < 1.65f,
                  "ceiling must block step lift");
}

bool testZeroMotionGrounding() {
    World world(velox::BackendType::Cpu);
    world.addStaticPlane({0, 1, 0}, 0.0f);
    CharacterController controller(world, controllerDesc());
    const Vec3 start{1.25f, kStandingCenterY, -3.5f};
    controller.SetPosition(start);
    CharacterControllerResult result = controller.Move({});
    return expect(result.grounded, "zero motion must still probe ground") &&
           expect(sameBits(result.finalPosition, start), "zero motion must not shift position") &&
           expect(sameBits(result.slideVelocity, Vec3{}), "zero motion must have zero slide velocity");
}

} // namespace

int main() {
    bool ok = testFlatWalk();
    ok &= testStepClimb();
    ok &= testWallSlide();
    ok &= testSteepSlopeSlip();
    ok &= testCornerNoEnergyAndDeterminism();
    ok &= testTallWallAndCeilingRejectSteps();
    ok &= testZeroMotionGrounding();
    if (ok) std::printf("character_demo: all character controller checks passed\n");
    return ok ? 0 : 1;
}
