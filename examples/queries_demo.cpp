#include <velox/velox.h>

#include <cstdio>
#include <vector>

int main() {
    velox::World world(velox::BackendType::Cpu);
    world.addStaticPlane({0, 1, 0}, 0.0f);
    world.addBox({0, 1, 0}, {0.5f, 0.5f, 0.5f}, 1.0f);
    world.step(0.0f);

    const velox::RayHit ray = world.rayCast({0, 4, 0}, {0, -1, 0}, 10.0f);
    std::vector<velox::BodyId> overlaps;
    world.overlapSphere({0, 1, 0}, 1.0f, overlaps);
    const velox::ShapeCastHit cast =
        world.sphereCast({0, 4, 0}, 0.25f, {0, -1, 0}, 10.0f);

    const bool ok = ray.hit && cast.hit && !overlaps.empty();
    std::printf("queries_demo: %s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
