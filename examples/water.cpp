// water.cpp — buoyancy, drag, and a surface current.
//
// Physics demonstrated:
//   * Velox has no built-in fluid, so water is modeled with per-body forces
//     applied before each step. For every body we estimate the submerged
//     fraction from how far its centre sits below the water plane (y = 0).
//   * Buoyancy pushes up with Archimedes' force: waterDensity * submergedVolume
//     * g. A body floats when that equals its weight, i.e. submergedFraction
//     = bodyDensity / waterDensity — so light bodies ride high and dense ones
//     sink to the tank floor.
//   * Linear drag pulls each submerged body toward the water velocity. Giving
//     the water a horizontal velocity models a surface current that carries
//     floating objects downstream.
//
// Fixed 60 FPS timestep; a side view (water line + tank floor + bodies) is
// printed every second.
#include <velox/velox.h>

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace {

using namespace velox;

constexpr float kWaterDensity = 1.0f;
constexpr float kGravity = 9.81f;
constexpr float kFloorY = -2.0f;
constexpr float kCurrentSpeed = 1.0f; // water drifts in +X
constexpr float kDragPerMass = 6.0f;  // drag strength relative to body mass

bool finite(Vec3 v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

struct Floater {
    BodyId id;
    float volume;   // for Archimedes' force
    float halfY;    // vertical half-extent (radius for spheres)
    float mass;
    float density;
    char marker;
    const char* name;
};

// Submerged fraction in [0,1] from the centre height (linear approximation of
// the spherical/box cap; exact at the float equilibrium used by the checks).
float submergedFraction(const Floater& f, float centerY) {
    return vclamp((f.halfY - centerY) / (2.0f * f.halfY), 0.0f, 1.0f);
}

void applyWaterForces(World& world, const Floater& f) {
    const Body b = world.bodyState(f.id);
    const float sub = submergedFraction(f, b.position.y);
    if (sub <= 0.0f) return; // fully above water: no fluid interaction

    // Archimedes buoyancy (up) + drag toward the moving water (current).
    const float buoyancy = kWaterDensity * f.volume * sub * kGravity;
    const Vec3 waterVel{kCurrentSpeed, 0.0f, 0.0f};
    const Vec3 drag = (waterVel - b.velocity) * (kDragPerMass * f.mass * sub);
    world.addForce(f.id, {0.0f, buoyancy, 0.0f});
    world.addForce(f.id, drag);
    // Angular drag damps spinning so floaters settle flat.
    world.addTorque(f.id, b.angularVelocity * (-3.0f * f.mass * sub));
}

struct SideView {
    int w, h;
    float minX, maxX, minY, maxY;
    std::vector<char> grid;
    SideView(int w, int h, float minX, float maxX, float minY, float maxY)
        : w(w), h(h), minX(minX), maxX(maxX), minY(minY), maxY(maxY),
          grid(static_cast<size_t>(w) * h, ' ') {}
    void put(float x, float y, char c) {
        int cx = int((x - minX) / (maxX - minX) * (w - 1));
        int cy = int((y - minY) / (maxY - minY) * (h - 1));
        if (cx >= 0 && cx < w && cy >= 0 && cy < h)
            grid[static_cast<size_t>(h - 1 - cy) * w + cx] = c;
    }
    void hline(float y, char c) {
        int cy = int((y - minY) / (maxY - minY) * (h - 1));
        if (cy >= 0 && cy < h)
            for (int x = 0; x < w; ++x)
                grid[static_cast<size_t>(h - 1 - cy) * w + x] = c;
    }
    std::string render() const {
        std::string s;
        s.reserve(static_cast<size_t>(w + 1) * h);
        for (int r = 0; r < h; ++r) {
            s.append(grid.begin() + static_cast<size_t>(r) * w,
                     grid.begin() + static_cast<size_t>(r + 1) * w);
            s.push_back('\n');
        }
        return s;
    }
};

} // namespace

int main() {
    World world(BackendType::Cpu);
    world.setGravity({0, -kGravity, 0});
    world.substeps = 6;

    world.addStaticPlane({0, 1, 0}, kFloorY); // tank floor at y = -2

    std::vector<Floater> floaters;
    const auto addSphereFloater = [&](Vec3 pos, float r, float density,
                                      char marker, const char* name) {
        const float volume = 4.0f / 3.0f * 3.14159265f * r * r * r;
        const float mass = density * volume;
        const BodyId id = world.addSphere(pos, r, mass);
        world.setEnableSleep(id, false); // buoyancy must act every frame
        world.setLinearDamping(id, 0.01f);
        floaters.push_back({id, volume, r, mass, density, marker, name});
    };
    const auto addBoxFloater = [&](Vec3 pos, Vec3 he, float density, char marker,
                                   const char* name) {
        const float volume = 8.0f * he.x * he.y * he.z;
        const float mass = density * volume;
        const BodyId id = world.addBox(pos, he, mass);
        world.setEnableSleep(id, false);
        world.setLinearDamping(id, 0.01f);
        floaters.push_back({id, volume, he.y, mass, density, marker, name});
    };

    // A range of densities: cork and wood float, the barge rides the surface,
    // the near-neutral buoy hovers just under the water, and stone sinks.
    addSphereFloater({-2.5f, 1.0f, 0.0f}, 0.40f, 0.20f, 'c', "cork");
    addSphereFloater({-1.0f, 1.0f, 0.0f}, 0.50f, 0.40f, 'w', "wood");
    addBoxFloater({0.7f, 1.0f, 0.0f}, {0.6f, 0.2f, 0.6f}, 0.50f, 'b', "barge");
    addSphereFloater({2.0f, 1.0f, 0.0f}, 0.40f, 0.95f, 'n', "buoy");
    addSphereFloater({3.2f, 1.0f, 0.0f}, 0.40f, 2.50f, 's', "stone");

    const std::vector<float> startX = [] {
        return std::vector<float>{};
    }();
    (void)startX;
    std::vector<float> x0;
    for (const Floater& f : floaters) x0.push_back(world.body(f.id).position.x);

    const int frames = 600; // 10 s at 60 FPS
    for (int frame = 0; frame < frames; ++frame) {
        for (const Floater& f : floaters) applyWaterForces(world, f);
        world.step(1.0f / 60.0f);

        if (frame % 60 == 0) {
            SideView view(60, 16, -4.0f, 5.0f, kFloorY - 0.4f, 1.3f);
            view.hline(0.0f, '~');      // water surface
            view.hline(kFloorY, '=');   // tank floor
            for (const Floater& f : floaters)
                view.put(world.body(f.id).position.x,
                         world.body(f.id).position.y, f.marker);
            std::string status;
            for (const Floater& f : floaters) {
                char buf[64];
                std::snprintf(buf, sizeof(buf), " %s:%.0f%%", f.name,
                              submergedFraction(f, world.body(f.id).position.y) * 100.0f);
                status += buf;
            }
            std::printf("t=%4.1fs submerged:%s\n%s", frame / 60.0f,
                        status.c_str(), view.render().c_str());
        }
    }

    // --- Validate ---------------------------------------------------------
    int failures = 0;
    const auto check = [&](bool cond, const char* label) {
        std::printf("%s: %s\n", cond ? "PASS" : "FAIL", label);
        if (!cond) ++failures;
    };

    const auto findY = [&](char marker) {
        for (const Floater& f : floaters)
            if (f.marker == marker) return world.bodyState(f.id).position.y;
        return -999.0f;
    };
    const auto findX = [&](char marker) {
        for (const Floater& f : floaters)
            if (f.marker == marker) return world.bodyState(f.id).position.x;
        return -999.0f;
    };

    bool allFinite = true;
    for (const Floater& f : floaters)
        allFinite = allFinite && finite(world.bodyState(f.id).position);

    const float corkY = findY('c'), woodY = findY('w'), bargeY = findY('b');
    const float buoyY = findY('n'), stoneY = findY('s');

    check(allFinite, "every floating body stays finite");
    check(woodY > 0.0f && woodY < 0.45f, "wood floats partly above the surface");
    check(corkY > woodY, "less-dense cork rides higher than wood");
    check(bargeY > -0.3f && bargeY < 0.3f, "the barge rides near the waterline");
    check(buoyY < -0.05f && buoyY > -1.2f, "the near-neutral buoy hovers below the surface");
    check(stoneY < -1.2f && stoneY > kFloorY - 0.1f, "dense stone sinks to the tank floor");
    check(findX('w') > x0[1] + 0.3f, "the current carries the floating wood downstream");

    if (failures) {
        std::fprintf(stderr, "water: %d failure(s)\n", failures);
        return 1;
    }
    std::puts("water: all checks passed");
    return 0;
}
