#include "velox/velox.h"
#include <cstdio>

int main() {
    velox::World world(velox::BackendType::Cpu);
    world.addStaticPlane({0, 1, 0}, 0.0f);
    const velox::BodyId sensor = world.addBox({0, 1, 0}, {1, 0.1f, 1}, 0.0f);
    world.body(sensor).sensor = 1;
    world.addSphere({0, 3, 0}, 0.5f, 1.0f);
    int begins = 0, persists = 0;
    for (int i = 0; i < 180; ++i) {
        world.step(1.0f / 60.0f);
        for (const velox::ContactEvent& event : world.contactEvents()) {
            begins += event.type == velox::ContactEventType::Begin && event.sensor;
            persists += event.type == velox::ContactEventType::Persist && event.sensor;
        }
    }
    std::printf("contact_events: begin=%d persist=%d\n", begins, persists);
    return begins > 0 && persists > 0 ? 0 : 1;
}
