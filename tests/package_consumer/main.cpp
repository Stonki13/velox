#include <velox/velox.h>

#include <cstdio>

int main() {
    velox::World world(velox::BackendType::Cpu);
    world.addStaticPlane({0.0f, 1.0f, 0.0f}, 0.0f);
    const velox::BodyId sphere = world.addSphere({0.0f, 2.0f, 0.0f}, 0.5f, 1.0f);
    world.step(1.0f / 60.0f);

    if (!world.isValid(sphere) || velox::versionString()[0] == '\0') {
        std::fputs("installed Velox package consumer failed\n", stderr);
        return 1;
    }
    std::puts("installed Velox package consumer passed");
    return 0;
}
