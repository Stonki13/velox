#include <iostream>
#include <cmath>
#include <iomanip>
#include "velox/ccd.h"
#include "velox/math.h"

using namespace velox;

struct SpherePlaneCtx {
    Vec3 pos, vel;
    float radius;
    float planeY;
};

static float spherePlaneOracle(void* ctx, float t, Vec3& normal, Vec3& point) {
    auto* c = static_cast<SpherePlaneCtx*>(ctx);
    Vec3 p = c->pos + c->vel * t;
    normal = {0.0f, 1.0f, 0.0f};
    point = {p.x, c->planeY, p.z};
    return p.y - c->radius - c->planeY;
}

int main() {
    SpherePlaneCtx ctx;
    ctx.pos = {0.0f, 10.0f, 0.0f};
    ctx.vel = {0.0f, -1000.0f, 0.0f};
    ctx.radius = 0.5f;
    ctx.planeY = 0.0f;

    CcdQualitySettings settings = ccdQualityPreset(CcdQuality::Accurate);
    float dt = 1.0f / 60.0f;
    float speed = length(ctx.vel);

    std::cout << std::setprecision(10);
    Vec3 normal, point;
    float gap = spherePlaneOracle(&ctx, 0.0f, normal, point);
    std::cout << "Gap at t=0: " << gap << std::endl;
    std::cout << "distTol: " << settings.distanceTolerance << std::endl;
    std::cout << "Speed: " << speed << std::endl;
    std::cout << "dt: " << dt << std::endl;

    float advance = (gap - settings.distanceTolerance) / speed;
    std::cout << "First advance: " << advance << std::endl;

    float t = advance;
    gap = spherePlaneOracle(&ctx, t, normal, point);
    std::cout << "Gap at t=" << t << ": " << gap << std::endl;
    std::cout << "gap <= distTol: " << (gap <= settings.distanceTolerance) << std::endl;

    ToiResult result = conservativeAdvancement(spherePlaneOracle, &ctx, dt, speed, settings);
    std::cout << "Result hit: " << result.hit << std::endl;
    std::cout << "Result toi: " << result.toi << std::endl;
    std::cout << "Result fraction: " << result.fraction << std::endl;

    return 0;
}
