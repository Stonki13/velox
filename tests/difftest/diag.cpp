#include "diff_test.h"
#include <cstdio>
using namespace difftest;
int main(){
  for (const SceneDesc& s : canonicalScenes()) {
    if (s.name != "ccd_wall") continue;
    Trajectory v = runVelox(s);
    for (size_t f = 8; f < 24; ++f)
      printf("f%3zu velox x=%8.4f y=%7.4f vx=%9.4f vy=%8.4f\n",
        f, v[f].bodies[0].position.x, v[f].bodies[0].position.y,
           v[f].bodies[0].velocity.x, v[f].bodies[0].velocity.y);
  }
  return 0;
}
