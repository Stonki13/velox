#include "velox/velox.h"
#include <cstdio>
#include <vector>
int main() {
    velox::World w(velox::BackendType::Cpu);
    w.addStaticPlane({0, 1, 0}, 0); auto b = w.addBox({0, 1, 0}, {0.5f,0.5f,0.5f}, 1);
    w.step(0); auto ray = w.rayCast({0,4,0},{0,-1,0},10); std::vector<velox::BodyId> hits;
    w.overlapSphere({0,1,0},1,hits); auto cast = w.sphereCast({0,4,0},.25f,{0,-1,0},10);
    bool ok = ray.hit && cast.hit && !hits.empty(); std::printf("queries_demo: %s\n", ok?"PASS":"FAIL"); return ok?0:1;
}
