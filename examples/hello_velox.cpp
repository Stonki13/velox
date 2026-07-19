#include "velox/velox.h"
#include <cstdio>

int main() {
    velox::World world(velox::BackendType::Cpu);
    world.addStaticPlane({0, 1, 0}, 0.0f);
    const velox::BodyId sphere = world.addSphere({0, 3, 0}, 0.5f, 1.0f);
    for (int frame = 0; frame < 120; ++frame) world.step(1.0f / 60.0f);
    const velox::Body state = world.bodyState(sphere);
    std::printf("hello_velox: sphere y=%.3f\n", state.position.y);
    return state.position.y >= 0.45f && state.position.y <= 0.55f ? 0 : 1;
}
