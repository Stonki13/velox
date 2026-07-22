// vehicle_terrain.cpp — a raycast vehicle driving over heightfield terrain.
//
// Physics demonstrated:
//   * World::addStaticHeightfield builds a triangle-mesh heightfield from a
//     row-major grid of samples. Sample (x, z) lands at world
//     origin + {x*cellSize, height, z*cellSize}, so a smooth analytic height
//     function gives rolling hills.
//   * The raycast Vehicle (vehicle.h) drops a virtual suspension ray from
//     each wheel hub every frame, applies a spring/damper along the ray and
//     slip-based tire forces at the contact, then a drivetrain integrates
//     engine torque through the gears. Calling Vehicle::Step BEFORE
//     World::step feeds those forces into the same step.
//   * As the chassis climbs and descends the hills the suspension compresses
//     and extends, the wheels stay grounded, and body roll follows the slope.
//
// Fixed 60 FPS timestep; an ASCII side profile (along the direction of
// travel) is printed every half second.
#include <velox/velox.h>

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace {

using namespace velox;

// Smooth rolling-hills height function used both to fill the heightfield and
// to place / visualize the vehicle. Amplitudes and frequencies are chosen so
// the slopes stay gentle enough for the car to climb in a low gear.
float terrainHeight(float wx, float wz) {
    return 1.5f * sinf(0.12f * wx) + 1.2f * sinf(0.18f * wz + 0.6f) +
           0.6f * cosf(0.08f * (wx + wz));
}

bool finite(Vec3 v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

// ASCII side view: horizontal axis is the travel direction (Z), vertical is Y.
struct SideView {
    int w, h;
    float minZ, maxZ, minY, maxY;
    std::vector<char> grid;
    SideView(int w, int h, float minZ, float maxZ, float minY, float maxY)
        : w(w), h(h), minZ(minZ), maxZ(maxZ), minY(minY), maxY(maxY),
          grid(static_cast<size_t>(w) * h, ' ') {}
    void put(float z, float y, char c) {
        int cx = int((z - minZ) / (maxZ - minZ) * (w - 1));
        int cy = int((y - minY) / (maxY - minY) * (h - 1));
        if (cx >= 0 && cx < w && cy >= 0 && cy < h)
            grid[static_cast<size_t>(h - 1 - cy) * w + cx] = c;
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
    world.setGravity({0, -9.81f, 0});

    // --- Build the heightfield terrain ------------------------------------
    constexpr uint32_t kSize = 200;     // 200 x 200 samples
    constexpr float kCell = 1.0f;       // 1 m per cell -> 200 m extent
    const Vec3 origin{-100.0f, 0.0f, -100.0f};
    std::vector<float> heights(static_cast<size_t>(kSize) * kSize);
    for (uint32_t z = 0; z < kSize; ++z)
        for (uint32_t x = 0; x < kSize; ++x) {
            const float wx = origin.x + x * kCell;
            const float wz = origin.z + z * kCell;
            heights[static_cast<size_t>(z) * kSize + x] = terrainHeight(wx, wz);
        }
    world.addStaticHeightfield(kSize, kSize, kCell, heights, origin);

    // --- Spawn the vehicle above the terrain ------------------------------
    const float startX = 0.0f, startZ = -70.0f;
    const float startY = terrainHeight(startX, startZ) + 1.0f; // clearance
    VehicleConfig config;                // default road-car tune
    Vehicle vehicle(world, config, {startX, startY, startZ});
    vehicle.AddDefaultWheels();
    vehicle.SetThrottle(0.7f);           // cruise forward over the hills

    float maxSpeed = 0.0f, minY = 1e30f, maxY = -1e30f;
    int groundedFrames = 0;
    const int frames = 420; // 7 s at 60 FPS

    for (int frame = 0; frame < frames; ++frame) {
        vehicle.Step(1.0f / 60.0f); // suspension/tire/drivetrain forces
        world.step(1.0f / 60.0f);

        const Body chassis = world.bodyState(vehicle.chassis());
        maxSpeed = vmax(maxSpeed, vehicle.forwardSpeed());
        minY = vmin(minY, chassis.position.y);
        maxY = vmax(maxY, chassis.position.y);

        int grounded = 0;
        for (size_t i = 0; i < vehicle.wheelCount(); ++i)
            if (vehicle.wheelState(i).grounded) ++grounded;
        if (grounded >= 2) ++groundedFrames;

        if (frame % 30 == 0) {
            const float cz = chassis.position.z, cy = chassis.position.y;
            SideView view(64, 14, cz - 22.0f, cz + 22.0f, cy - 5.0f, cy + 5.0f);
            // Terrain profile sampled along the car's X at successive Z.
            for (int col = 0; col < 64; ++col) {
                const float z = cz - 22.0f + (col / 63.0f) * 44.0f;
                view.put(z, terrainHeight(startX, z), '.');
            }
            view.put(cz, cy, 'C'); // chassis
            std::printf(
                "t=%4.1fs  speed=%5.1f m/s  gear=%d  rpm=%5.0f  y=%5.2f  "
                "grounded=%d/%zu\n%s",
                frame / 60.0f, vehicle.forwardSpeed(), vehicle.currentGear(),
                vehicle.engineRPM(), cy, grounded, vehicle.wheelCount(),
                view.render().c_str());
        }
    }

    // --- Validate ---------------------------------------------------------
    int failures = 0;
    const auto check = [&](bool cond, const char* label) {
        std::printf("%s: %s\n", cond ? "PASS" : "FAIL", label);
        if (!cond) ++failures;
    };

    const Body final = world.bodyState(vehicle.chassis());
    const Vec3 up = rotate(final.orientation, {0, 1, 0});
    const float travel = final.position.z - startZ;

    check(finite(final.position), "chassis position stays finite on terrain");
    check(travel > 25.0f, "vehicle drives forward over the terrain");
    check(maxSpeed > 6.0f, "vehicle builds up speed on the hills");
    check(up.y > 0.3f, "vehicle stays upright (does not roll over)");
    check(maxY - minY > 1.0f, "suspension follows the undulating terrain");
    check(groundedFrames > frames / 2, "wheels stay grounded most of the run");

    if (failures) {
        std::fprintf(stderr, "vehicle_terrain: %d failure(s)\n", failures);
        return 1;
    }
    std::puts("vehicle_terrain: all checks passed");
    return 0;
}
