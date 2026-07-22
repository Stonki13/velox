#include <velox/velox.h>

#include <cmath>
#include <cstdio>
#include <vector>

int main() {
    velox::World world(velox::BackendType::Cpu);
    world.addStaticPlane({0.0f, 1.0f, 0.0f}, 0.0f);
    const velox::BodyId wall = world.addBox({3.0f, 1.0f, 0.0f},
                                            {0.25f, 1.0f, 1.0f}, 0.0f);
    const velox::BodyId sphere = world.addSphere({0.0f, 2.0f, 0.0f}, 0.5f, 1.0f);
    world.setLinearVelocity(sphere, {2.0f, 0.0f, 0.0f});

    velox::CharacterController controller(world);
    controller.SetPosition({-1.0f, 1.21f, 0.0f});
    const velox::CharacterControllerResult character = controller.Move({0.25f, 0.0f, 0.0f});

    for (int frame = 0; frame < 180; ++frame) {
        world.step(1.0f / 60.0f);
    }

    const velox::WorldSnapshot snapshot = world.saveSnapshot();
    const velox::Body beforeRestore = world.bodyState(sphere);
    world.step(1.0f / 60.0f);
    world.restoreSnapshot(snapshot);
    const velox::Body afterRestore = world.bodyState(sphere);

    const velox::RayHit hit = world.rayCast({0.0f, 1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, 10.0f);
    std::vector<velox::BodyId> nearby;
    world.overlapSphere({0.0f, 1.0f, 0.0f}, 5.0f, nearby);
    const bool finite = std::isfinite(afterRestore.position.x) &&
                        std::isfinite(afterRestore.position.y) &&
                        std::isfinite(character.finalPosition.x);
    const bool restored = velox::length(beforeRestore.position - afterRestore.position) < 1.0e-6f;

    if (!world.isValid(sphere) || !world.isValid(wall) || !finite || !restored ||
        !hit.hit || nearby.empty() || velox::versionString()[0] == '\0') {
        std::fputs("installed Velox package consumer failed\n", stderr);
        return 1;
    }
    std::puts("installed Velox package consumer passed");
    return 0;
}
