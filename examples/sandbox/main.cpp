#include "input.h"
#include "renderer.h"
#include "window.h"

#include <velox/velox.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

namespace {

using velox::BodyId;
using velox::DebugLine;
using velox::Quat;
using velox::RayHit;
using velox::Vec3;
using velox::World;

constexpr float Pi = 3.14159265358979323846f;
constexpr float FixedDt = 1.0f / 60.0f;

enum class Preset { Stack, Rain, Ragdoll, Contraption };

struct SceneParameters {
    int stackBoxes = 10;
    float boxHalfExtent = 0.5f;
    int rainSpheres = 200;
    float sphereRadius = 0.2f;
    int ragdollBodies = 8;
    float ragdollRampDegrees = 20.0f;
    int contraptionWheels = 4;
    float contraptionRampDegrees = 12.0f;
};

float sceneNumber(const std::string& json, const char* section, const char* field, float fallback) {
    const size_t sectionOffset = json.find(std::string("\"") + section + "\"");
    if (sectionOffset == std::string::npos) return fallback;
    const size_t fieldOffset = json.find(std::string("\"") + field + "\"", sectionOffset);
    if (fieldOffset == std::string::npos) return fallback;
    const size_t colon = json.find(':', fieldOffset);
    if (colon == std::string::npos) return fallback;
    char* end = nullptr;
    const float value = std::strtof(json.c_str() + colon + 1, &end);
    return end == json.c_str() + colon + 1 || !std::isfinite(value) ? fallback : value;
}

SceneParameters loadSceneParameters() {
    std::ifstream file("scenes.json");
    if (!file) return {};
    const std::string json((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    SceneParameters parameters;
    parameters.stackBoxes = static_cast<int>(sceneNumber(json, "stack", "boxes", parameters.stackBoxes));
    parameters.boxHalfExtent = sceneNumber(json, "stack", "boxHalfExtent", parameters.boxHalfExtent);
    parameters.rainSpheres = static_cast<int>(sceneNumber(json, "rain", "spheres", parameters.rainSpheres));
    parameters.sphereRadius = sceneNumber(json, "rain", "sphereRadius", parameters.sphereRadius);
    parameters.ragdollBodies = static_cast<int>(sceneNumber(json, "ragdoll", "bodies", parameters.ragdollBodies));
    parameters.ragdollRampDegrees = sceneNumber(json, "ragdoll", "rampDegrees", parameters.ragdollRampDegrees);
    parameters.contraptionWheels = static_cast<int>(sceneNumber(json, "contraption", "wheels", parameters.contraptionWheels));
    parameters.contraptionRampDegrees = sceneNumber(json, "contraption", "rampDegrees", parameters.contraptionRampDegrees);
    return parameters;
}

const char* presetName(Preset preset) {
    switch (preset) {
    case Preset::Stack: return "STACK";
    case Preset::Rain: return "RAIN";
    case Preset::Ragdoll: return "RAGDOLL";
    case Preset::Contraption: return "CONTRAPTION";
    }
    return "UNKNOWN";
}

bool finite(Vec3 value) {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

class SandboxScene {
public:
    explicit SandboxScene(SceneParameters parameters) : parameters_(parameters) {}

    void reset(Preset preset) {
        preset_ = preset;
        world_ = std::make_unique<World>();
        dynamicBodies_.clear();
        switch (preset) {
        case Preset::Stack: buildStack(); break;
        case Preset::Rain: buildRain(); break;
        case Preset::Ragdoll: buildRagdoll(); break;
        case Preset::Contraption: buildContraption(); break;
        }
    }

    World& world() { return *world_; }
    const World& world() const { return *world_; }
    Preset preset() const { return preset_; }
    const std::vector<BodyId>& dynamicBodies() const { return dynamicBodies_; }

    void addSphereAt(Vec3 position) {
        const BodyId sphere = world_->addSphere(position, 0.25f, 1.0f);
        world_->body(sphere).friction = 0.6f;
        dynamicBodies_.push_back(sphere);
    }

private:
    BodyId sphere(Vec3 position, float radius = 0.2f, float mass = 1.0f) {
        const BodyId id = world_->addSphere(position, radius, mass);
        world_->body(id).friction = 0.65f;
        world_->body(id).restitution = 0.05f;
        if (mass > 0.0f) dynamicBodies_.push_back(id);
        return id;
    }

    BodyId box(Vec3 position, Vec3 half, float mass = 1.0f) {
        const BodyId id = world_->addBox(position, half, mass);
        world_->body(id).friction = 0.75f;
        world_->body(id).restitution = 0.02f;
        if (mass > 0.0f) dynamicBodies_.push_back(id);
        return id;
    }

    BodyId capsule(Vec3 position, float radius, float halfHeight, float mass = 1.0f) {
        const BodyId id = world_->addCapsule(position, radius, halfHeight, mass);
        world_->body(id).friction = 0.7f;
        if (mass > 0.0f) dynamicBodies_.push_back(id);
        return id;
    }

    void ground() { world_->addStaticPlane({0.0f, 1.0f, 0.0f}, 0.0f); }

    void buildStack() {
        ground();
        const float extent = parameters_.boxHalfExtent;
        for (int i = 0; i < parameters_.stackBoxes; ++i)
            box({0.0f, extent + (extent * 2.02f) * static_cast<float>(i), 0.0f},
                {extent, extent, extent});
    }

    void buildRain() {
        ground();
        for (int i = 0; i < parameters_.rainSpheres; ++i) {
            const int x = i % 20;
            const int layer = i / 20;
            sphere({-5.7f + 0.6f * static_cast<float>(x),
                    1.5f + 0.55f * static_cast<float>(layer),
                    static_cast<float>((x * 7 + layer * 3) % 9) * 0.18f - 0.7f},
                   parameters_.sphereRadius);
        }
    }

    void buildRagdoll() {
        ground();
        const BodyId ramp = box({0.0f, 0.45f, 0.0f}, {6.0f, 0.25f, 3.0f}, 0.0f);
        world_->setTransform(ramp, {0.0f, 0.45f, 0.0f},
                             velox::fromAxisAngle({0.0f, 0.0f, 1.0f},
                                                  -parameters_.ragdollRampDegrees * Pi / 180.0f));

        std::vector<BodyId> bodies(static_cast<size_t>(parameters_.ragdollBodies));
        for (int i = 0; i < static_cast<int>(bodies.size()); ++i)
            bodies[i] = capsule({-2.0f, 5.5f + 0.58f * static_cast<float>(i), 0.0f}, 0.18f, 0.28f);
        for (size_t i = 1; i < bodies.size(); ++i)
            world_->addBallJoint(bodies[i - 1], bodies[i],
                                 {-2.0f, 5.5f + 0.58f * static_cast<float>(i) - 0.29f, 0.0f});
    }

    void buildContraption() {
        ground();
        const BodyId ramp = box({0.5f, 0.35f, 0.0f}, {7.0f, 0.25f, 3.5f}, 0.0f);
        world_->setTransform(ramp, {0.5f, 0.35f, 0.0f},
                             velox::fromAxisAngle({0.0f, 0.0f, 1.0f},
                                                  -parameters_.contraptionRampDegrees * Pi / 180.0f));
        const BodyId chassis = box({-4.0f, 2.5f, 0.0f}, {1.1f, 0.28f, 0.65f}, 4.0f);
        const std::array<Vec3, 4> offsets{{
            {-0.75f, -0.48f, -0.72f}, {-0.75f, -0.48f, 0.72f},
            {0.75f, -0.48f, -0.72f}, {0.75f, -0.48f, 0.72f}}};
        for (int i = 0; i < parameters_.contraptionWheels; ++i) {
            const Vec3 offset = offsets[static_cast<size_t>(i) % offsets.size()];
            const Vec3 wheelPosition{-4.0f + offset.x, 2.5f + offset.y, offset.z};
            const BodyId wheel = sphere(wheelPosition, 0.38f, 0.8f);
            world_->addHingeJoint(chassis, wheel, wheelPosition, {0.0f, 0.0f, 1.0f});
        }
    }

    std::unique_ptr<World> world_;
    std::vector<BodyId> dynamicBodies_;
    SceneParameters parameters_;
    Preset preset_ = Preset::Stack;
};

std::array<float, 16> multiply(const std::array<float, 16>& a,
                               const std::array<float, 16>& b) {
    std::array<float, 16> result{};
    for (int column = 0; column < 4; ++column)
        for (int row = 0; row < 4; ++row)
            for (int i = 0; i < 4; ++i)
                result[column * 4 + row] += a[i * 4 + row] * b[column * 4 + i];
    return result;
}

std::array<float, 16> perspective(float verticalFovRadians, float aspect, float nearPlane, float farPlane) {
    const float f = 1.0f / std::tan(verticalFovRadians * 0.5f);
    return {f / aspect, 0.0f, 0.0f, 0.0f,
            0.0f, f, 0.0f, 0.0f,
            0.0f, 0.0f, (farPlane + nearPlane) / (nearPlane - farPlane), -1.0f,
            0.0f, 0.0f, (2.0f * farPlane * nearPlane) / (nearPlane - farPlane), 0.0f};
}

std::array<float, 16> lookAt(Vec3 eye, Vec3 forward) {
    const Vec3 f = velox::normalize(forward);
    const Vec3 s = velox::normalize(velox::cross(f, {0.0f, 1.0f, 0.0f}));
    const Vec3 u = velox::cross(s, f);
    return {s.x, u.x, -f.x, 0.0f,
            s.y, u.y, -f.y, 0.0f,
            s.z, u.z, -f.z, 0.0f,
            -velox::dot(s, eye), -velox::dot(u, eye), velox::dot(f, eye), 1.0f};
}

struct Camera {
    Vec3 position{8.0f, 6.0f, 10.0f};
    float yaw = -2.25f;
    float pitch = -0.28f;

    Vec3 forward() const {
        return velox::normalize(Vec3{std::cos(pitch) * std::cos(yaw), std::sin(pitch),
                                     std::cos(pitch) * std::sin(yaw)});
    }

    Vec3 right() const { return velox::normalize(velox::cross(forward(), {0.0f, 1.0f, 0.0f})); }
};

void spawnAtCursor(SandboxScene& scene, const Camera& camera, const sandbox::Input& input,
                   int width, int height) {
    const float aspect = static_cast<float>(width) / static_cast<float>(height);
    const float x = static_cast<float>(2.0 * input.cursorX() / static_cast<double>(width) - 1.0);
    const float y = static_cast<float>(1.0 - 2.0 * input.cursorY() / static_cast<double>(height));
    const float tangent = std::tan(60.0f * Pi / 360.0f);
    const Vec3 direction = velox::normalize(camera.forward() + camera.right() * (x * tangent * aspect) +
                                            velox::cross(camera.right(), camera.forward()) * (y * tangent));
    const RayHit hit = scene.world().rayCast(camera.position, direction, 200.0f);
    if (hit.hit) scene.addSphereAt(hit.point + hit.normal * 0.32f);
}

bool selfTest(const SceneParameters& parameters) {
    constexpr std::array<Preset, 4> presets{
        Preset::Stack, Preset::Rain, Preset::Ragdoll, Preset::Contraption};
    for (Preset preset : presets) {
        SandboxScene scene(parameters);
        scene.reset(preset);
        bool sawContact = false;
        for (int step = 0; step < 120; ++step) {
            scene.world().step(FixedDt);
            sawContact = sawContact || scene.world().lastStepStats().generatedContacts > 0;
            for (BodyId id : scene.dynamicBodies()) {
                if (!finite(scene.world().body(id).position)) {
                    std::fprintf(stderr, "sandbox self-test: %s produced a non-finite position\n", presetName(preset));
                    return false;
                }
            }
        }
        std::vector<DebugLine> lines;
        scene.world().debugLines(lines);
        if (!sawContact || lines.empty()) {
            std::fprintf(stderr, "sandbox self-test: %s did not produce expected contacts/debug lines\n", presetName(preset));
            return false;
        }
    }
    std::puts("sandbox self-test passed");
    return true;
}

std::vector<std::string> overlay(const SandboxScene& scene, int substeps, bool paused, float fps) {
    const velox::StepStats& stats = scene.world().lastStepStats();
    char line[128];
    std::vector<std::string> result;
    std::snprintf(line, sizeof(line), "PRESET: %s", presetName(scene.preset()));
    result.emplace_back(line);
    std::snprintf(line, sizeof(line), "BODIES: %zu ACTIVE: %zu", stats.bodyCount, stats.awakeDynamicBodies);
    result.emplace_back(line);
    std::snprintf(line, sizeof(line), "CONTACTS: %zu STEP: %.2F MS", stats.generatedContacts, stats.totalMs);
    result.emplace_back(line);
    std::snprintf(line, sizeof(line), "SUBSTEPS: %d %s FPS: %.0F", substeps, paused ? "PAUSED" : "RUNNING", fps);
    result.emplace_back(line);
    result.emplace_back("WASD MOVE  QE TURN  RMB LOOK");
    result.emplace_back("SPACE SPAWN  R RESET  P PAUSE  +/- SUBSTEPS");
    result.emplace_back("F1 STACK  F2 RAIN  F3 RAGDOLL  F4 CONTRAPTION");
    return result;
}

int runInteractive(const SceneParameters& parameters) {
    sandbox::Window window(1280, 720, "Velox Interactive Sandbox");
    sandbox::Renderer renderer;
    sandbox::Input input;
    SandboxScene scene(parameters);
    scene.reset(Preset::Stack);
    Camera camera;
    bool paused = false;
    int substeps = 4;
    float accumulator = 0.0f;
    float spawnCooldown = 0.0f;
    float fps = 0.0f;
    int frameCount = 0;
    auto last = std::chrono::steady_clock::now();
    auto fpsStart = last;

    while (!window.shouldClose()) {
        window.pollEvents();
        input.update(window);
        const auto now = std::chrono::steady_clock::now();
        const float frameSeconds = std::min(0.25f,
            std::chrono::duration<float>(now - last).count());
        last = now;
        accumulator += frameSeconds;
        spawnCooldown += frameSeconds;

        if (input.pressed(sandbox::Action::Reset)) scene.reset(scene.preset());
        if (input.pressed(sandbox::Action::Pause)) paused = !paused;
        if (input.pressed(sandbox::Action::IncreaseSubsteps)) substeps = std::min(substeps + 1, 16);
        if (input.pressed(sandbox::Action::DecreaseSubsteps)) substeps = std::max(substeps - 1, 1);
        if (input.pressed(sandbox::Action::Stack)) scene.reset(Preset::Stack);
        if (input.pressed(sandbox::Action::Rain)) scene.reset(Preset::Rain);
        if (input.pressed(sandbox::Action::Ragdoll)) scene.reset(Preset::Ragdoll);
        if (input.pressed(sandbox::Action::Contraption)) scene.reset(Preset::Contraption);

        if (input.mouseLook()) {
            camera.yaw += input.mouseDeltaX() * 0.004f;
            camera.pitch = std::clamp(camera.pitch - input.mouseDeltaY() * 0.004f, -1.45f, 1.45f);
        }
        if (input.down(sandbox::Action::TurnLeft)) camera.yaw -= frameSeconds * 1.7f;
        if (input.down(sandbox::Action::TurnRight)) camera.yaw += frameSeconds * 1.7f;
        Vec3 move{};
        const Vec3 forward = camera.forward();
        const Vec3 flatForward = velox::normalize(Vec3{forward.x, 0.0f, forward.z});
        if (input.down(sandbox::Action::Forward)) move += flatForward;
        if (input.down(sandbox::Action::Backward)) move -= flatForward;
        if (input.down(sandbox::Action::Right)) move += camera.right();
        if (input.down(sandbox::Action::Left)) move -= camera.right();
        if (velox::lengthSq(move) > 0.0f) camera.position += velox::normalize(move) * (7.0f * frameSeconds);

        if (input.pressed(sandbox::Action::Spawn) ||
            (input.down(sandbox::Action::Spawn) && spawnCooldown >= 0.1f)) {
            spawnAtCursor(scene, camera, input, window.width(), window.height());
            spawnCooldown = 0.0f;
        }

        scene.world().substeps = substeps;
        if (!paused) {
            while (accumulator >= FixedDt) {
                scene.world().step(FixedDt);
                accumulator -= FixedDt;
            }
        } else {
            accumulator = 0.0f;
        }

        std::vector<DebugLine> lines;
        scene.world().debugLines(lines);
        const auto viewProjection = multiply(perspective(60.0f * Pi / 180.0f,
                                                          static_cast<float>(window.width()) / window.height(),
                                                          0.05f, 300.0f),
                                             lookAt(camera.position, camera.forward()));
        ++frameCount;
        const float fpsSeconds = std::chrono::duration<float>(now - fpsStart).count();
        if (fpsSeconds >= 0.5f) {
            fps = static_cast<float>(frameCount) / fpsSeconds;
            frameCount = 0;
            fpsStart = now;
        }
        renderer.render(lines, viewProjection, overlay(scene, substeps, paused, fps),
                        window.width(), window.height());
        window.swapBuffers();
    }
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    const SceneParameters parameters = loadSceneParameters();
    if (argc == 2 && std::string(argv[1]) == "--selftest") return selfTest(parameters) ? 0 : 1;
    try {
        return runInteractive(parameters);
    } catch (const std::exception& error) {
        std::fprintf(stderr, "velox_sandbox: %s\n", error.what());
        return 1;
    }
}
