// Proof of the no-tunneling claim: fire a tiny sphere at 2 km/s at the ground.
// A discrete engine at 60 Hz would move it ~33 m per step and skip straight
// through. Velox's CCD catches the impact mid-step and it bounces.
#include <velox/velox.h>
#include <cstdio>

int main() {
    velox::World world;
    world.addStaticPlane({0, 1, 0}, 0.0f);

    auto bullet = world.addSphere({0, 1.0f, 0}, 0.05f, 0.01f);
    world.body(bullet).velocity = {0, -2000.0f, 0};
    world.body(bullet).restitution = 0.5f;

    for (int i = 0; i < 10; ++i) {
        world.step(1.0f / 60.0f);
        const auto& b = world.body(bullet);
        std::printf("step %2d  y = %10.4f  vy = %10.2f\n", i, b.position.y, b.velocity.y);
    }

    bool ok = world.body(bullet).position.y >= 0.0f;
    std::printf(ok ? "\nOK: bullet stayed above the ground. No tunneling.\n"
                   : "\nFAIL: bullet tunneled through the ground!\n");
    return ok ? 0 : 1;
}
