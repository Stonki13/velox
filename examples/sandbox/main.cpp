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
#include <cstring>
#include <fstream>
#include <map>
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

enum class Preset { Stack, Rain, Ragdoll, Contraption, Gyro };

enum class SpawnShape { Sphere, Box, Capsule, Cylinder, Cone, RandomHull, Complex };

const char* spawnShapeName(SpawnShape shape) {
    switch (shape) {
    case SpawnShape::Sphere: return "SPHERE";
    case SpawnShape::Box: return "BOX";
    case SpawnShape::Capsule: return "CAPSULE";
    case SpawnShape::Cylinder: return "CYLINDER";
    case SpawnShape::Cone: return "CONE";
    case SpawnShape::RandomHull: return "HULL";
    case SpawnShape::Complex: return "COMPLEX";
    }
    return "UNKNOWN";
}

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
    case Preset::Contraption: return "VEHICLE";
    case Preset::Gyro: return "GYRO";
    }
    return "UNKNOWN";
}

bool finite(Vec3 value) {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

// Curated palette; assignment is stable per body slot so colors never flicker.
constexpr float kPalette[][3] = {
    {0.91f, 0.34f, 0.32f}, // coral red
    {0.98f, 0.69f, 0.25f}, // amber
    {0.96f, 0.87f, 0.44f}, // sun yellow
    {0.48f, 0.78f, 0.46f}, // leaf green
    {0.30f, 0.70f, 0.67f}, // teal
    {0.36f, 0.57f, 0.89f}, // sky blue
    {0.55f, 0.45f, 0.86f}, // violet
    {0.89f, 0.50f, 0.74f}, // pink
    {0.79f, 0.60f, 0.42f}, // tan
    {0.62f, 0.79f, 0.88f}, // ice
};
constexpr int kPaletteCount = static_cast<int>(sizeof(kPalette) / sizeof(kPalette[0]));

// One renderable piece of a body: a mesh id (assigned lazily by the render
// layer) plus a local transform and scale. Compounds contribute several.
struct RenderPiece {
    enum class Kind { Sphere, Box, Capsule, Cylinder, Cone, Hull, Ground };
    Kind kind = Kind::Sphere;
    Vec3 localPosition{};
    Quat localOrientation{};
    Vec3 scale{1.0f, 1.0f, 1.0f};
    float capsuleRatio = 1.0f; // Capsule only: radius / halfHeight
    std::vector<std::array<float, 3>> hullPoints; // Hull only
};

struct RenderBody {
    BodyId id;
    bool isStatic = false;
    std::vector<RenderPiece> pieces;
};

class SandboxScene {
public:
    explicit SandboxScene(SceneParameters parameters) : parameters_(parameters) {}

    void reset(Preset preset) {
        preset_ = preset;
        vehicle_.reset(); // references the old world; must die first
        world_ = std::make_unique<World>();
        dynamicBodies_.clear();
        renderBodies_.clear();
        complexCycle_ = 0;
        spawnSeed_ = 0x9e3779b9u;
        switch (preset) {
        case Preset::Stack: buildStack(); break;
        case Preset::Rain: buildRain(); break;
        case Preset::Ragdoll: buildRagdoll(); break;
        case Preset::Contraption: buildVehicle(); break;
        case Preset::Gyro: buildGyro(); break;
        }
    }

    World& world() { return *world_; }
    const World& world() const { return *world_; }
    Preset preset() const { return preset_; }
    const std::vector<BodyId>& dynamicBodies() const { return dynamicBodies_; }
    const std::vector<RenderBody>& renderBodies() const { return renderBodies_; }
    velox::Vehicle* vehicle() { return vehicle_.get(); }
    const velox::Vehicle* vehicle() const { return vehicle_.get(); }

    void spawnAt(SpawnShape shape, Vec3 position) {
        switch (shape) {
        case SpawnShape::Sphere: {
            const BodyId id = sphere(position, 0.25f, 1.0f);
            pieceSphere(id, 0.25f);
            break;
        }
        case SpawnShape::Box: {
            const Vec3 half{0.24f, 0.24f, 0.24f};
            const BodyId id = box(position, half, 1.0f);
            pieceBox(id, half);
            break;
        }
        case SpawnShape::Capsule: {
            const BodyId id = capsule(position, 0.16f, 0.26f, 1.0f);
            pieceCapsule(id, 0.16f, 0.26f);
            break;
        }
        case SpawnShape::Cylinder: {
            const BodyId id = world_->addCylinder(position, 0.22f, 0.26f, 1.0f);
            world_->body(id).friction = 0.7f;
            dynamicBodies_.push_back(id);
            RenderBody body{id, false, {}};
            RenderPiece piece;
            piece.kind = RenderPiece::Kind::Cylinder;
            piece.scale = {0.22f, 0.26f, 0.22f};
            body.pieces.push_back(std::move(piece));
            renderBodies_.push_back(std::move(body));
            break;
        }
        case SpawnShape::Cone: {
            const BodyId id = world_->addCone(position, 0.26f, 0.5f, 1.0f);
            world_->body(id).friction = 0.7f;
            dynamicBodies_.push_back(id);
            RenderBody body{id, false, {}};
            RenderPiece piece;
            piece.kind = RenderPiece::Kind::Cone;
            piece.scale = {0.26f, 0.25f, 0.26f}; // y scale = half total height
            body.pieces.push_back(std::move(piece));
            renderBodies_.push_back(std::move(body));
            break;
        }
        case SpawnShape::RandomHull: {
            spawnRandomHull(position);
            break;
        }
        case SpawnShape::Complex: {
            switch (complexCycle_++ % 3) {
            case 0: spawnRingCompound(position); break;
            case 1: spawnLCompound(position); break;
            default: spawnBlobHull(position); break;
            }
            break;
        }
        }
    }

    void spawnDefault(Vec3 position) { spawnAt(SpawnShape::Sphere, position); }

private:
    uint32_t nextRandom() {
        spawnSeed_ = spawnSeed_ * 1664525u + 1013904223u;
        return spawnSeed_;
    }
    float random01() { return static_cast<float>(nextRandom() >> 8) / 16777216.0f; }

    void pieceSphere(BodyId id, float radius, bool isStatic = false) {
        RenderBody body{id, isStatic, {}};
        RenderPiece piece;
        piece.kind = RenderPiece::Kind::Sphere;
        piece.scale = {radius, radius, radius};
        body.pieces.push_back(std::move(piece));
        renderBodies_.push_back(std::move(body));
    }

    void pieceBox(BodyId id, Vec3 half, bool isStatic = false) {
        RenderBody body{id, isStatic, {}};
        RenderPiece piece;
        piece.kind = RenderPiece::Kind::Box;
        piece.scale = half;
        body.pieces.push_back(std::move(piece));
        renderBodies_.push_back(std::move(body));
    }

    void pieceCapsule(BodyId id, float radius, float halfHeight, bool isStatic = false) {
        RenderBody body{id, isStatic, {}};
        RenderPiece piece;
        piece.kind = RenderPiece::Kind::Capsule;
        piece.scale = {halfHeight, halfHeight, halfHeight};
        piece.capsuleRatio = radius / halfHeight;
        body.pieces.push_back(std::move(piece));
        renderBodies_.push_back(std::move(body));
    }

    void spawnRandomHull(Vec3 position) {
        std::vector<Vec3> points;
        std::vector<std::array<float, 3>> renderPoints;
        const int count = 8 + static_cast<int>(random01() * 8.0f);
        const Vec3 radii{0.20f + random01() * 0.18f, 0.20f + random01() * 0.18f,
                         0.20f + random01() * 0.18f};
        for (int i = 0; i < count; ++i) {
            const float z = random01() * 2.0f - 1.0f;
            const float a = random01() * 2.0f * Pi;
            const float r = std::sqrt(std::max(0.0f, 1.0f - z * z));
            const Vec3 p{r * std::cos(a) * radii.x, z * radii.y, r * std::sin(a) * radii.z};
            points.push_back(p);
            renderPoints.push_back({p.x, p.y, p.z});
        }
        const BodyId id = world_->addConvexHull(position, points, 1.0f);
        world_->body(id).friction = 0.7f;
        dynamicBodies_.push_back(id);
        RenderBody body{id, false, {}};
        RenderPiece piece;
        piece.kind = RenderPiece::Kind::Hull;
        piece.hullPoints = std::move(renderPoints);
        body.pieces.push_back(std::move(piece));
        renderBodies_.push_back(std::move(body));
    }

    void spawnBlobHull(Vec3 position) {
        // Low-poly blob: jittered icosahedron directions.
        const float t = (1.0f + std::sqrt(5.0f)) * 0.5f;
        const Vec3 base[12] = {
            {-1, t, 0}, {1, t, 0}, {-1, -t, 0}, {1, -t, 0},
            {0, -1, t}, {0, 1, t}, {0, -1, -t}, {0, 1, -t},
            {t, 0, -1}, {t, 0, 1}, {-t, 0, -1}, {-t, 0, 1}};
        std::vector<Vec3> points;
        std::vector<std::array<float, 3>> renderPoints;
        for (const Vec3& direction : base) {
            const Vec3 d = velox::normalize(direction);
            const float radius = 0.28f + random01() * 0.14f;
            const Vec3 p = d * radius;
            points.push_back(p);
            renderPoints.push_back({p.x, p.y, p.z});
        }
        const BodyId id = world_->addConvexHull(position, points, 1.2f);
        world_->body(id).friction = 0.75f;
        dynamicBodies_.push_back(id);
        RenderBody body{id, false, {}};
        RenderPiece piece;
        piece.kind = RenderPiece::Kind::Hull;
        piece.hullPoints = std::move(renderPoints);
        body.pieces.push_back(std::move(piece));
        renderBodies_.push_back(std::move(body));
    }

    void spawnRingCompound(Vec3 position) {
        // Eight boxes arranged as a ring standing in the XY plane.
        std::vector<velox::CompoundShape> shapes;
        const int segments = 8;
        const float ringRadius = 0.42f;
        for (int i = 0; i < segments; ++i) {
            const float a = 2.0f * Pi * static_cast<float>(i) / segments;
            velox::CompoundShape shape;
            shape.shape = velox::ShapeType::Box;
            shape.localPosition = {std::cos(a) * ringRadius, std::sin(a) * ringRadius, 0.0f};
            shape.localOrientation = velox::fromAxisAngle({0, 0, 1}, a);
            shape.halfExtents = {0.20f, 0.09f, 0.09f};
            shapes.push_back(shape);
        }
        addCompoundWithRender(position, shapes, 1.5f);
    }

    void spawnLCompound(Vec3 position) {
        std::vector<velox::CompoundShape> shapes;
        velox::CompoundShape a;
        a.shape = velox::ShapeType::Box;
        a.localPosition = {0.0f, 0.0f, 0.0f};
        a.halfExtents = {0.42f, 0.12f, 0.12f};
        shapes.push_back(a);
        velox::CompoundShape b;
        b.shape = velox::ShapeType::Box;
        b.localPosition = {-0.30f, 0.28f, 0.0f};
        b.halfExtents = {0.12f, 0.28f, 0.12f};
        shapes.push_back(b);
        addCompoundWithRender(position, shapes, 1.4f);
    }

    BodyId addCompoundWithRender(Vec3 position,
                                 const std::vector<velox::CompoundShape>& shapes,
                                 float mass) {
        const BodyId id = world_->addCompound(position, shapes, mass);
        world_->body(id).friction = 0.7f;
        dynamicBodies_.push_back(id);
        // addCompound recenters geometry on the center of mass; authored child
        // locals must shift by (authored origin - computed COM) to line up.
        const Vec3 shift = position - world_->body(id).position;
        RenderBody body{id, false, {}};
        for (const velox::CompoundShape& shape : shapes) {
            RenderPiece piece;
            piece.localPosition = shape.localPosition + shift;
            piece.localOrientation = shape.localOrientation;
            switch (shape.shape) {
            case velox::ShapeType::Box:
                piece.kind = RenderPiece::Kind::Box;
                piece.scale = shape.halfExtents;
                break;
            case velox::ShapeType::Sphere:
                piece.kind = RenderPiece::Kind::Sphere;
                piece.scale = {shape.radius, shape.radius, shape.radius};
                break;
            case velox::ShapeType::Capsule:
                piece.kind = RenderPiece::Kind::Capsule;
                piece.scale = {shape.capsuleHalfHeight, shape.capsuleHalfHeight,
                               shape.capsuleHalfHeight};
                piece.capsuleRatio = shape.radius / shape.capsuleHalfHeight;
                break;
            case velox::ShapeType::Cylinder:
                piece.kind = RenderPiece::Kind::Cylinder;
                piece.scale = {shape.radius, shape.capsuleHalfHeight, shape.radius};
                break;
            case velox::ShapeType::Cone:
                piece.kind = RenderPiece::Kind::Cone;
                piece.scale = {shape.radius, shape.capsuleHalfHeight, shape.radius};
                break;
            default:
                piece.kind = RenderPiece::Kind::Box;
                piece.scale = {0.1f, 0.1f, 0.1f};
                break;
            }
            body.pieces.push_back(std::move(piece));
        }
        renderBodies_.push_back(std::move(body));
        return id;
    }

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

    void ground() {
        const BodyId id = world_->addStaticPlane({0.0f, 1.0f, 0.0f}, 0.0f);
        RenderBody body{id, true, {}};
        RenderPiece piece;
        piece.kind = RenderPiece::Kind::Ground;
        piece.scale = {400.0f, 1.0f, 400.0f};
        body.pieces.push_back(std::move(piece));
        renderBodies_.push_back(std::move(body));
    }

    void buildStack() {
        ground();
        const float extent = parameters_.boxHalfExtent;
        for (int i = 0; i < parameters_.stackBoxes; ++i) {
            const BodyId id = box({0.0f, extent + (extent * 2.0f) * static_cast<float>(i), 0.0f},
                                  {extent, extent, extent});
            pieceBox(id, {extent, extent, extent});
        }
    }

    void buildRain() {
        ground();
        for (int i = 0; i < parameters_.rainSpheres; ++i) {
            const int x = i % 20;
            const int layer = i / 20;
            const BodyId id = sphere({-5.7f + 0.6f * static_cast<float>(x),
                                      1.5f + 0.55f * static_cast<float>(layer),
                                      static_cast<float>((x * 7 + layer * 3) % 9) * 0.18f - 0.7f},
                                     parameters_.sphereRadius);
            pieceSphere(id, parameters_.sphereRadius);
        }
    }

    void buildRagdoll() {
        ground();
        const BodyId ramp = box({0.0f, 0.45f, 0.0f}, {6.0f, 0.25f, 3.0f}, 0.0f);
        world_->setTransform(ramp, {0.0f, 0.45f, 0.0f},
                             velox::fromAxisAngle({0.0f, 0.0f, 1.0f},
                                                  -parameters_.ragdollRampDegrees * Pi / 180.0f));
        pieceBox(ramp, {6.0f, 0.25f, 3.0f}, true);

        std::vector<BodyId> bodies(static_cast<size_t>(parameters_.ragdollBodies));
        for (int i = 0; i < static_cast<int>(bodies.size()); ++i) {
            bodies[i] = capsule({-2.0f, 5.5f + 0.58f * static_cast<float>(i), 0.0f}, 0.18f, 0.28f);
            pieceCapsule(bodies[i], 0.18f, 0.28f);
        }
        for (size_t i = 1; i < bodies.size(); ++i)
            world_->addBallJoint(bodies[i - 1], bodies[i],
                                 {-2.0f, 5.5f + 0.58f * static_cast<float>(i) - 0.29f, 0.0f});
    }

    // F4: the real raycast Vehicle (roadmap 12), driven with the arrow keys.
    // A ramp and a wall of light boxes give it something to jump and smash.
    void buildVehicle() {
        ground();
        const BodyId ramp = box({0.0f, 0.4f, 14.0f}, {2.5f, 0.25f, 3.0f}, 0.0f);
        world_->setTransform(ramp, {0.0f, 0.4f, 14.0f},
                             velox::fromAxisAngle({1.0f, 0.0f, 0.0f},
                                                  -10.0f * Pi / 180.0f));
        pieceBox(ramp, {2.5f, 0.25f, 3.0f}, true);
        for (int level = 0; level < 4; ++level)
            for (int column = 0; column < 5; ++column) {
                const BodyId brick = box({-2.0f + 1.0f * static_cast<float>(column),
                                          0.4f + 0.81f * static_cast<float>(level),
                                          26.0f},
                                         {0.45f, 0.4f, 0.3f}, 0.4f);
                pieceBox(brick, {0.45f, 0.4f, 0.3f});
            }
        velox::VehicleConfig config;
        config.chassisMass = 900.0f;
        config.chassisHalfExtents = {0.9f, 0.35f, 1.9f};
        config.drivetrain = velox::DrivetrainType::RWD;
        vehicle_ = std::make_unique<velox::Vehicle>(*world_, config,
                                                    Vec3{0.0f, 1.2f, -6.0f});
        vehicle_->AddDefaultWheels();
        dynamicBodies_.push_back(vehicle_->chassis());
        // No pieceBox: the car renders as a composite (body, cabin, bumpers,
        // detailed wheels) in buildBatches instead of one flat box.
    }

    // Gyroscopic showcase: three torque-free T-handles floating in zero
    // gravity, each spun about a different body axis.
    void buildGyro() {
        world_->gravity = {0.0f, 0.0f, 0.0f};
        ground(); // visual reference; nothing falls onto it
        std::vector<velox::CompoundShape> tHandle;
        velox::CompoundShape grip;
        grip.shape = velox::ShapeType::Box;
        grip.localPosition = {0.0f, -0.2f, 0.0f};
        grip.halfExtents = {0.12f, 0.6f, 0.12f};
        tHandle.push_back(grip);
        velox::CompoundShape crossbar;
        crossbar.shape = velox::ShapeType::Box;
        crossbar.localPosition = {0.0f, 0.5f, 0.0f};
        crossbar.halfExtents = {0.75f, 0.1f, 0.16f};
        tHandle.push_back(crossbar);
        const Vec3 spins[3] = {
            {0.0f, 8.0f, 0.05f},
            {5.0f, 0.05f, 0.0f},
            {0.05f, 0.0f, 4.5f},
        };
        for (int i = 0; i < 3; ++i) {
            const Vec3 position{-3.5f + 3.5f * static_cast<float>(i), 3.5f, 0.0f};
            const BodyId id = addCompoundWithRender(position, tHandle, 1.0f);
            world_->setAngularVelocity(id, spins[i]);
        }
    }

    std::unique_ptr<World> world_;
    std::unique_ptr<velox::Vehicle> vehicle_;
    std::vector<BodyId> dynamicBodies_;
    std::vector<RenderBody> renderBodies_;
    SceneParameters parameters_;
    Preset preset_ = Preset::Stack;
    int complexCycle_ = 0;
    uint32_t spawnSeed_ = 0x9e3779b9u;
};

// --- matrices ----------------------------------------------------------------

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

// General 4x4 inverse (column-major); used for the sky's ray reconstruction.
std::array<float, 16> inverse(const std::array<float, 16>& m) {
    std::array<float, 16> inv;
    inv[0] = m[5]*m[10]*m[15] - m[5]*m[11]*m[14] - m[9]*m[6]*m[15] +
             m[9]*m[7]*m[14] + m[13]*m[6]*m[11] - m[13]*m[7]*m[10];
    inv[4] = -m[4]*m[10]*m[15] + m[4]*m[11]*m[14] + m[8]*m[6]*m[15] -
             m[8]*m[7]*m[14] - m[12]*m[6]*m[11] + m[12]*m[7]*m[10];
    inv[8] = m[4]*m[9]*m[15] - m[4]*m[11]*m[13] - m[8]*m[5]*m[15] +
             m[8]*m[7]*m[13] + m[12]*m[5]*m[11] - m[12]*m[7]*m[9];
    inv[12] = -m[4]*m[9]*m[14] + m[4]*m[10]*m[13] + m[8]*m[5]*m[14] -
              m[8]*m[6]*m[13] - m[12]*m[5]*m[10] + m[12]*m[6]*m[9];
    inv[1] = -m[1]*m[10]*m[15] + m[1]*m[11]*m[14] + m[9]*m[2]*m[15] -
             m[9]*m[3]*m[14] - m[13]*m[2]*m[11] + m[13]*m[3]*m[10];
    inv[5] = m[0]*m[10]*m[15] - m[0]*m[11]*m[14] - m[8]*m[2]*m[15] +
             m[8]*m[3]*m[14] + m[12]*m[2]*m[11] - m[12]*m[3]*m[10];
    inv[9] = -m[0]*m[9]*m[15] + m[0]*m[11]*m[13] + m[8]*m[1]*m[15] -
             m[8]*m[3]*m[13] - m[12]*m[1]*m[11] + m[12]*m[3]*m[9];
    inv[13] = m[0]*m[9]*m[14] - m[0]*m[10]*m[13] - m[8]*m[1]*m[14] +
              m[8]*m[2]*m[13] + m[12]*m[1]*m[10] - m[12]*m[2]*m[9];
    inv[2] = m[1]*m[6]*m[15] - m[1]*m[7]*m[14] - m[5]*m[2]*m[15] +
             m[5]*m[3]*m[14] + m[13]*m[2]*m[7] - m[13]*m[3]*m[6];
    inv[6] = -m[0]*m[6]*m[15] + m[0]*m[7]*m[14] + m[4]*m[2]*m[15] -
             m[4]*m[3]*m[14] - m[12]*m[2]*m[7] + m[12]*m[3]*m[6];
    inv[10] = m[0]*m[5]*m[15] - m[0]*m[7]*m[13] - m[4]*m[1]*m[15] +
              m[4]*m[3]*m[13] + m[12]*m[1]*m[7] - m[12]*m[3]*m[5];
    inv[14] = -m[0]*m[5]*m[14] + m[0]*m[6]*m[13] + m[4]*m[1]*m[14] -
              m[4]*m[2]*m[13] - m[12]*m[1]*m[6] + m[12]*m[2]*m[5];
    inv[3] = -m[1]*m[6]*m[11] + m[1]*m[7]*m[10] + m[5]*m[2]*m[11] -
             m[5]*m[3]*m[10] - m[9]*m[2]*m[7] + m[9]*m[3]*m[6];
    inv[7] = m[0]*m[6]*m[11] - m[0]*m[7]*m[10] - m[4]*m[2]*m[11] +
             m[4]*m[3]*m[10] + m[8]*m[2]*m[7] - m[8]*m[3]*m[6];
    inv[11] = -m[0]*m[5]*m[11] + m[0]*m[7]*m[9] + m[4]*m[1]*m[11] -
              m[4]*m[3]*m[9] - m[8]*m[1]*m[7] + m[8]*m[3]*m[5];
    inv[15] = m[0]*m[5]*m[10] - m[0]*m[6]*m[9] - m[4]*m[1]*m[10] +
              m[4]*m[2]*m[9] + m[8]*m[1]*m[6] - m[8]*m[2]*m[5];
    float det = m[0]*inv[0] + m[1]*inv[4] + m[2]*inv[8] + m[3]*inv[12];
    if (std::fabs(det) < 1e-20f) return inv;
    det = 1.0f / det;
    for (float& v : inv) v *= det;
    return inv;
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

Vec3 cursorRay(const Camera& camera, const sandbox::Input& input, int width, int height) {
    const float aspect = static_cast<float>(width) / static_cast<float>(height);
    const float x = static_cast<float>(2.0 * input.cursorX() / static_cast<double>(width) - 1.0);
    const float y = static_cast<float>(1.0 - 2.0 * input.cursorY() / static_cast<double>(height));
    const float tangent = std::tan(60.0f * Pi / 360.0f);
    return velox::normalize(camera.forward() + camera.right() * (x * tangent * aspect) +
                            velox::cross(camera.right(), camera.forward()) * (y * tangent));
}

// --- dragging ----------------------------------------------------------------

struct DragState {
    bool active = false;
    BodyId body;
    Vec3 localAnchor{};
    float distance = 0.0f;
    Vec3 target{}; // cursor-plane target, refreshed once per render frame
};

void updateDrag(DragState& drag, SandboxScene& scene, const Camera& camera,
                const sandbox::Input& input, int width, int height) {
    World& world = scene.world();
    if (input.dragPressed() && !input.mouseLook()) {
        const Vec3 direction = cursorRay(camera, input, width, height);
        const RayHit hit = world.rayCast(camera.position, direction, 200.0f);
        if (hit.hit && world.isValid(hit.body) &&
            world.motionType(hit.body) == velox::MotionType::Dynamic) {
            drag.active = true;
            drag.body = hit.body;
            drag.distance = hit.t;
            const velox::Body& body = world.body(hit.body);
            drag.localAnchor = velox::rotateInv(body.orientation, hit.point - body.position);
        }
    }
    if (drag.active && (!input.dragDown() || !world.isValid(drag.body))) drag.active = false;
    if (!drag.active) return;

    // Scroll pulls the grabbed body nearer or pushes it farther.
    drag.distance *= 1.0f + input.scrollDelta() * 0.09f;
    drag.distance = std::clamp(drag.distance, 0.6f, 120.0f);
    drag.target = camera.position + cursorRay(camera, input, width, height) * drag.distance;
}

// The spring force is applied exactly once per PHYSICS step (inside the
// fixed-step loop), never per render frame: forces accumulate until a step
// consumes them, so per-frame application at high FPS stacked several spring
// impulses into one step (or none into the next) and made dragging jitter.
void applyDragForce(const DragState& drag, World& world) {
    if (!drag.active || !world.isValid(drag.body)) return;
    const velox::Body& body = world.body(drag.body);
    const Vec3 anchorWorld = body.position + velox::rotate(body.orientation, drag.localAnchor);
    const Vec3 armVector = anchorWorld - body.position;
    const Vec3 pointVelocity = body.velocity + velox::cross(body.angularVelocity, armVector);

    // Critically damped spring at the grabbed point, applied as a force so
    // the solver can still push back (no teleporting, no stack explosions).
    const float mass = body.invMass > 0.0f ? 1.0f / body.invMass : 0.0f;
    if (mass <= 0.0f) return;
    const float stiffness = 140.0f;
    const float damping = 2.0f * std::sqrt(stiffness); // critical
    Vec3 spring = (drag.target - anchorWorld) * stiffness - pointVelocity * damping;
    const float maxAcceleration = 300.0f;
    const float springLength = velox::length(spring);
    if (springLength > maxAcceleration) spring *= maxAcceleration / springLength;
    world.wake(drag.body);
    world.addForceAtPoint(drag.body, spring * mass, anchorWorld);
}

// --- rendering helpers ---------------------------------------------------------

class RenderCache {
public:
    explicit RenderCache(sandbox::Renderer& renderer) : renderer_(renderer) {
        cube_ = renderer.registerMesh(sandbox::makeCube());
        sphere_ = renderer.registerMesh(sandbox::makeIcosphere(2));
        cylinder_ = renderer.registerMesh(sandbox::makeCylinder());
        cone_ = renderer.registerMesh(sandbox::makeCone());
        ground_ = renderer.registerMesh(sandbox::makeGroundQuad());
    }

    uint32_t cube() const { return cube_; }
    uint32_t sphere() const { return sphere_; }
    uint32_t cylinder() const { return cylinder_; }
    uint32_t cone() const { return cone_; }
    uint32_t ground() const { return ground_; }

    uint32_t capsule(float ratio) {
        const int key = static_cast<int>(ratio * 1000.0f + 0.5f);
        auto found = capsules_.find(key);
        if (found != capsules_.end()) return found->second;
        const uint32_t id = renderer_.registerMesh(sandbox::makeCapsule(ratio));
        capsules_[key] = id;
        return id;
    }

    // Cache by geometry, not body handle: world resets reuse body slots while
    // a freshly generated random hull needs its own triangle mesh.
    uint32_t hull(const std::vector<std::array<float, 3>>& points) {
        uint64_t key = 1469598103934665603ull;
        for (const auto& point : points) {
            for (float component : point) {
                uint32_t bits = 0;
                std::memcpy(&bits, &component, sizeof(bits));
                key ^= bits;
                key *= 1099511628211ull;
            }
        }
        auto found = hulls_.find(key);
        if (found != hulls_.end()) return found->second;
        const uint32_t id = renderer_.registerMesh(sandbox::makeHullMesh(points));
        hulls_[key] = id;
        return id;
    }

private:
    sandbox::Renderer& renderer_;
    uint32_t cube_ = 0, sphere_ = 0, cylinder_ = 0, cone_ = 0, ground_ = 0;
    std::map<int, uint32_t> capsules_;
    std::map<uint64_t, uint32_t> hulls_;
};

sandbox::Instance makeInstance(Vec3 position, Quat orientation, Vec3 scale,
                               const float color[3], float colorW) {
    const Vec3 cx = velox::rotate(orientation, {1.0f, 0.0f, 0.0f}) * scale.x;
    const Vec3 cy = velox::rotate(orientation, {0.0f, 1.0f, 0.0f}) * scale.y;
    const Vec3 cz = velox::rotate(orientation, {0.0f, 0.0f, 1.0f}) * scale.z;
    sandbox::Instance instance;
    instance.row0[0] = cx.x; instance.row0[1] = cy.x; instance.row0[2] = cz.x; instance.row0[3] = position.x;
    instance.row1[0] = cx.y; instance.row1[1] = cy.y; instance.row1[2] = cz.y; instance.row1[3] = position.y;
    instance.row2[0] = cx.z; instance.row2[1] = cy.z; instance.row2[2] = cz.z; instance.row2[3] = position.z;
    instance.color[0] = color[0];
    instance.color[1] = color[1];
    instance.color[2] = color[2];
    instance.color[3] = colorW;
    return instance;
}

void buildBatches(const SandboxScene& scene, RenderCache& cache,
                  std::vector<sandbox::DrawBatch>& batches) {
    std::map<uint32_t, size_t> batchIndex;
    auto batchFor = [&](uint32_t meshId) -> sandbox::DrawBatch& {
        auto found = batchIndex.find(meshId);
        if (found != batchIndex.end()) return batches[found->second];
        batchIndex[meshId] = batches.size();
        batches.push_back({meshId, {}});
        return batches.back();
    };

    const World& world = scene.world();

    // The car renders as a composite: body + cabin + bumpers on the chassis
    // pose, and per-wheel tire/hub/spokes placed from suspension telemetry
    // (wheels are virtual raycasts, not bodies). Spokes make spin visible.
    if (const velox::Vehicle* vehicle = scene.vehicle()) {
        const velox::Body& chassis = world.body(vehicle->chassis());
        const Quat pose = chassis.orientation;
        auto atLocal = [&](const Vec3& local) {
            return chassis.position + velox::rotate(pose, local);
        };
        const float paint[3] = {0.82f, 0.16f, 0.14f};      // sporty red
        const float glass[3] = {0.16f, 0.22f, 0.30f};      // dark cabin
        const float trim[3] = {0.20f, 0.20f, 0.22f};       // bumpers/skirt
        const float silver[3] = {0.78f, 0.78f, 0.82f};     // hubs
        const float darkRubber[3] = {0.13f, 0.13f, 0.15f}; // tires
        // Lower body: full footprint, slightly shallower than the collider.
        batchFor(cache.cube()).instances.push_back(makeInstance(
            atLocal({0.0f, -0.06f, 0.0f}), pose, {0.90f, 0.24f, 1.90f}, paint, 1.0f));
        // Hood/trunk shoulder line.
        batchFor(cache.cube()).instances.push_back(makeInstance(
            atLocal({0.0f, 0.16f, 0.15f}), pose, {0.84f, 0.10f, 1.55f}, paint, 1.0f));
        // Cabin, rear-biased with "glass" color.
        batchFor(cache.cube()).instances.push_back(makeInstance(
            atLocal({0.0f, 0.38f, -0.30f}), pose, {0.70f, 0.16f, 0.85f}, glass, 1.0f));
        // Bumpers and side skirts.
        batchFor(cache.cube()).instances.push_back(makeInstance(
            atLocal({0.0f, -0.18f, 1.86f}), pose, {0.86f, 0.10f, 0.10f}, trim, 1.0f));
        batchFor(cache.cube()).instances.push_back(makeInstance(
            atLocal({0.0f, -0.18f, -1.86f}), pose, {0.86f, 0.10f, 0.10f}, trim, 1.0f));

        const Quat alignAxle = velox::fromAxisAngle({0.0f, 0.0f, 1.0f}, Pi * 0.5f);
        for (size_t i = 0; i < vehicle->wheelCount(); ++i) {
            const velox::WheelConfig& wheel = vehicle->wheelConfig(i);
            const velox::WheelState& state = vehicle->wheelState(i);
            const Vec3 hub = chassis.position +
                             velox::rotate(pose, wheel.localPosition);
            const Vec3 down = velox::normalize(velox::rotate(pose, wheel.direction));
            const float travel = state.grounded
                ? wheel.suspensionRestLength - state.compression
                : wheel.suspensionRestLength;
            const Vec3 center = hub + down * travel;
            Quat wheelPose = pose;
            if (wheel.steerable)
                wheelPose = velox::mul(wheelPose,
                    velox::fromAxisAngle({0.0f, 1.0f, 0.0f}, vehicle->steeringAngle()));
            const Quat rolling = velox::mul(wheelPose,
                velox::fromAxisAngle({1.0f, 0.0f, 0.0f}, state.rotation));
            const Quat tirePose = velox::mul(rolling, alignAxle);
            // Tire, bright hub, and two crossing spokes (visible rotation).
            batchFor(cache.cylinder()).instances.push_back(makeInstance(
                center, tirePose, {wheel.radius, 0.14f, wheel.radius}, darkRubber, 1.0f));
            batchFor(cache.cylinder()).instances.push_back(makeInstance(
                center, tirePose, {wheel.radius * 0.55f, 0.15f, wheel.radius * 0.55f},
                silver, 1.0f));
            batchFor(cache.cube()).instances.push_back(makeInstance(
                center, rolling, {0.152f, wheel.radius * 0.82f, 0.045f}, silver, 1.0f));
            batchFor(cache.cube()).instances.push_back(makeInstance(
                center, rolling, {0.152f, 0.045f, wheel.radius * 0.82f}, silver, 1.0f));
        }
    }
    for (const RenderBody& renderBody : scene.renderBodies()) {
        if (!world.isValid(renderBody.id)) continue;
        const velox::Body& body = world.body(renderBody.id);

        float color[3];
        float colorW = 1.0f;
        if (renderBody.isStatic) {
            color[0] = 0.62f; color[1] = 0.63f; color[2] = 0.58f;
        } else {
            const float* palette = kPalette[renderBody.id.slot() % kPaletteCount];
            color[0] = palette[0]; color[1] = palette[1]; color[2] = palette[2];
            if (body.asleep) {
                // Desaturate sleeping bodies so islands visibly settle.
                const float gray = (color[0] + color[1] + color[2]) / 3.0f;
                color[0] = color[0] * 0.35f + gray * 0.65f;
                color[1] = color[1] * 0.35f + gray * 0.65f;
                color[2] = color[2] * 0.35f + gray * 0.65f;
            }
        }

        for (const RenderPiece& piece : renderBody.pieces) {
            const Vec3 world_position =
                body.position + velox::rotate(body.orientation, piece.localPosition);
            const Quat world_orientation = velox::mul(body.orientation, piece.localOrientation);
            uint32_t meshId = 0;
            switch (piece.kind) {
            case RenderPiece::Kind::Sphere: meshId = cache.sphere(); break;
            case RenderPiece::Kind::Box: meshId = cache.cube(); break;
            case RenderPiece::Kind::Capsule: meshId = cache.capsule(piece.capsuleRatio); break;
            case RenderPiece::Kind::Cylinder: meshId = cache.cylinder(); break;
            case RenderPiece::Kind::Cone: meshId = cache.cone(); break;
            case RenderPiece::Kind::Hull:
                meshId = cache.hull(piece.hullPoints);
                break;
            case RenderPiece::Kind::Ground: {
                meshId = cache.ground();
                const float groundColor[3] = {0.62f, 0.68f, 0.60f};
                batchFor(meshId).instances.push_back(makeInstance(
                    {0.0f, 0.0f, 0.0f}, {}, piece.scale, groundColor, 2.0f));
                continue;
            }
            }
            batchFor(meshId).instances.push_back(
                makeInstance(world_position, world_orientation, piece.scale, color, colorW));
        }
    }
}

// --- selftest ------------------------------------------------------------------

bool selfTest(const SceneParameters& parameters) {
    constexpr std::array<Preset, 5> presets{
        Preset::Stack, Preset::Rain, Preset::Ragdoll, Preset::Contraption,
        Preset::Gyro};
    for (Preset preset : presets) {
        SandboxScene scene(parameters);
        scene.reset(preset);
        const Vec3 vehicleStart = scene.vehicle()
            ? scene.world().body(scene.vehicle()->chassis()).position : Vec3{};
        bool sawContact = false;
        for (int step = 0; step < 120; ++step) {
            if (velox::Vehicle* vehicle = scene.vehicle()) {
                vehicle->SetThrottle(1.0f);
                vehicle->Step(FixedDt);
            }
            scene.world().step(FixedDt);
            sawContact = sawContact || scene.world().lastStepStats().generatedContacts > 0;
            for (BodyId id : scene.dynamicBodies()) {
                if (!finite(scene.world().body(id).position)) {
                    std::fprintf(stderr, "sandbox self-test: %s produced a non-finite position\n", presetName(preset));
                    return false;
                }
            }
        }
        if (scene.vehicle()) {
            const Vec3 travelled =
                scene.world().body(scene.vehicle()->chassis()).position - vehicleStart;
            if (travelled.z < 1.0f) {
                std::fprintf(stderr, "sandbox self-test: vehicle did not drive (dz=%.2f)\n",
                             travelled.z);
                return false;
            }
        }
        std::vector<DebugLine> lines;
        scene.world().debugLines(lines);
        // The zero-gravity gyro preset is contact-free by design; its gate is
        // that the tumbling boxes keep spinning (no artificial damping).
        if (preset == Preset::Gyro) {
            bool spinning = true;
            for (BodyId id : scene.dynamicBodies())
                spinning = spinning &&
                    velox::lengthSq(scene.world().body(id).angularVelocity) > 1.0f;
            if (!spinning || lines.empty()) {
                std::fprintf(stderr, "sandbox self-test: gyro preset lost its spin\n");
                return false;
            }
            continue;
        }
        if (!sawContact || lines.empty()) {
            std::fprintf(stderr, "sandbox self-test: %s did not produce expected contacts/debug lines\n", presetName(preset));
            return false;
        }
    }

    // Every spawnable palette shape must simulate stably too.
    constexpr std::array<SpawnShape, 7> shapes{
        SpawnShape::Sphere, SpawnShape::Box, SpawnShape::Capsule, SpawnShape::Cylinder,
        SpawnShape::Cone, SpawnShape::RandomHull, SpawnShape::Complex};
    SandboxScene scene(parameters);
    scene.reset(Preset::Stack);
    float x = -6.0f;
    for (SpawnShape shape : shapes) {
        scene.spawnAt(shape, {x, 2.0f, 3.0f});
        scene.spawnAt(shape, {x, 3.2f, 3.0f}); // Complex preset cycles variants
        x += 2.0f;
    }
    bool sawContact = false;
    for (int step = 0; step < 120; ++step) {
        scene.world().step(FixedDt);
        sawContact = sawContact || scene.world().lastStepStats().generatedContacts > 0;
        for (BodyId id : scene.dynamicBodies()) {
            if (!finite(scene.world().body(id).position)) {
                std::fprintf(stderr, "sandbox self-test: spawn palette produced a non-finite position\n");
                return false;
            }
        }
    }
    if (!sawContact) {
        std::fprintf(stderr, "sandbox self-test: spawn palette produced no contacts\n");
        return false;
    }
    std::puts("sandbox self-test passed");
    return true;
}

// --- overlay -------------------------------------------------------------------

std::vector<std::string> overlay(const SandboxScene& scene, int substeps, bool paused,
                                 float fps, SpawnShape spawnShape, bool showLines,
                                 bool dragging) {
    const velox::StepStats& stats = scene.world().lastStepStats();
    char line[160];
    std::vector<std::string> result;
    std::snprintf(line, sizeof(line), "PRESET: %s   SHAPE: %s%s", presetName(scene.preset()),
                  spawnShapeName(spawnShape), dragging ? "   DRAGGING" : "");
    result.emplace_back(line);
    std::snprintf(line, sizeof(line), "BODIES: %zu ACTIVE: %zu", stats.bodyCount, stats.awakeDynamicBodies);
    result.emplace_back(line);
    std::snprintf(line, sizeof(line), "CONTACTS: %zu STEP: %.2F MS", stats.generatedContacts, stats.totalMs);
    result.emplace_back(line);
    std::snprintf(line, sizeof(line), "SUBSTEPS: %d %s FPS: %.0F%s", substeps,
                  paused ? "PAUSED" : "RUNNING", fps, showLines ? "  LINES ON" : "");
    result.emplace_back(line);
    result.emplace_back("WASD MOVE  QE TURN  RMB LOOK  LMB DRAG  WHEEL DEPTH");
    result.emplace_back("1-7 SHAPE  SPACE SPAWN  R RESET  P PAUSE  +/- SUBSTEPS  L LINES");
    result.emplace_back("F1 STACK  F2 RAIN  F3 RAGDOLL  F4 VEHICLE  F5 GYRO");
    if (scene.preset() == Preset::Gyro)
        result.emplace_back("ZERO-G: LEFT STABLE SPIN  MIDDLE DZHANIBEKOV FLIPS  RIGHT STABLE");
    if (const velox::Vehicle* vehicle = scene.vehicle()) {
        std::snprintf(line, sizeof(line),
                      "ARROWS DRIVE   SPEED: %.0F KM/H  GEAR: %d  RPM: %.0F",
                      vehicle->forwardSpeed() * 3.6f, vehicle->currentGear(),
                      vehicle->engineRPM());
        result.emplace_back(line);
    }
    return result;
}

// --- interactive loop ------------------------------------------------------------

int runInteractive(const SceneParameters& parameters) {
    sandbox::Window window(1280, 720, "Velox Interactive Sandbox");
    sandbox::Renderer renderer(window);
    RenderCache cache(renderer);
    sandbox::Input input;
    SandboxScene scene(parameters);
    scene.reset(Preset::Stack);
    Camera camera;
    DragState drag;
    bool paused = false;
    bool showLines = false;
    // Interactive demos let users pile bodies deep; more substeps keeps tall
    // stacks solid under real-time (non-uniform) frame pacing. Cheap here: the
    // step budget is well under 1 ms even for the CUDA backend.
    int substeps = 8;
    float steering = 0.0f;
    float accumulator = 0.0f;
    float spawnCooldown = 0.0f;
    float fps = 0.0f;
    int frameCount = 0;
    SpawnShape spawnShape = SpawnShape::Sphere;
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
        const bool drawable = window.width() > 0 && window.height() > 0;

        if (input.pressed(sandbox::Action::Reset)) { scene.reset(scene.preset()); drag.active = false; }
        if (input.pressed(sandbox::Action::Pause)) paused = !paused;
        if (input.pressed(sandbox::Action::ToggleLines)) showLines = !showLines;
        if (input.pressed(sandbox::Action::IncreaseSubsteps)) substeps = std::min(substeps + 1, 16);
        if (input.pressed(sandbox::Action::DecreaseSubsteps)) substeps = std::max(substeps - 1, 1);
        if (input.pressed(sandbox::Action::Stack)) { scene.reset(Preset::Stack); drag.active = false; }
        if (input.pressed(sandbox::Action::Rain)) { scene.reset(Preset::Rain); drag.active = false; }
        if (input.pressed(sandbox::Action::Ragdoll)) { scene.reset(Preset::Ragdoll); drag.active = false; }
        if (input.pressed(sandbox::Action::Contraption)) {
            scene.reset(Preset::Contraption);
            drag.active = false;
            camera.position = {5.5f, 3.0f, -11.0f};
            camera.yaw = 1.95f; // over-the-shoulder view toward the course
            camera.pitch = -0.15f;
        }
        if (input.pressed(sandbox::Action::Gyro)) {
            scene.reset(Preset::Gyro);
            drag.active = false;
            // Front-row seat for the Dzhanibekov showcase.
            camera.position = {0.0f, 3.6f, 9.0f};
            camera.yaw = -1.5708f; // look toward -z (the boxes)
            camera.pitch = -0.05f;
        }
        if (input.pressed(sandbox::Action::Shape1)) spawnShape = SpawnShape::Sphere;
        if (input.pressed(sandbox::Action::Shape2)) spawnShape = SpawnShape::Box;
        if (input.pressed(sandbox::Action::Shape3)) spawnShape = SpawnShape::Capsule;
        if (input.pressed(sandbox::Action::Shape4)) spawnShape = SpawnShape::Cylinder;
        if (input.pressed(sandbox::Action::Shape5)) spawnShape = SpawnShape::Cone;
        if (input.pressed(sandbox::Action::Shape6)) spawnShape = SpawnShape::RandomHull;
        if (input.pressed(sandbox::Action::Shape7)) spawnShape = SpawnShape::Complex;

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

        if (drawable && (input.pressed(sandbox::Action::Spawn) ||
            (input.down(sandbox::Action::Spawn) && spawnCooldown >= 0.1f))) {
            const Vec3 direction = cursorRay(camera, input, window.width(), window.height());
            const RayHit hit = scene.world().rayCast(camera.position, direction, 200.0f);
            if (hit.hit) scene.spawnAt(spawnShape, hit.point + hit.normal * 0.45f);
            spawnCooldown = 0.0f;
        }

        if (drawable) updateDrag(drag, scene, camera, input, window.width(), window.height());

        // Vehicle controls: smoothed steering, throttle/brake on the arrows.
        if (velox::Vehicle* vehicle = scene.vehicle()) {
            const float steerTarget =
                (input.down(sandbox::Action::DriveLeft) ? 0.45f : 0.0f) +
                (input.down(sandbox::Action::DriveRight) ? -0.45f : 0.0f);
            steering += (steerTarget - steering) * std::min(1.0f, 10.0f * frameSeconds);
            vehicle->SetSteering(steering);
            vehicle->SetThrottle(input.down(sandbox::Action::DriveForward) ? 1.0f : 0.0f);
            vehicle->SetBrake(input.down(sandbox::Action::DriveBrake) ? 1.0f : 0.0f);
        }

        scene.world().substeps = substeps;
        if (!paused) {
            while (accumulator >= FixedDt) {
                applyDragForce(drag, scene.world());
                if (velox::Vehicle* vehicle = scene.vehicle()) vehicle->Step(FixedDt);
                scene.world().step(FixedDt);
                accumulator -= FixedDt;
            }
        } else {
            accumulator = 0.0f;
        }

        if (!drawable) continue;

        // Camera matrices and globals.
        const auto projection = perspective(60.0f * Pi / 180.0f,
                                            static_cast<float>(window.width()) / window.height(),
                                            0.05f, 500.0f);
        const auto view = lookAt(camera.position, camera.forward());
        const auto viewProjection = multiply(projection, view);
        const auto invViewProjection = inverse(viewProjection);
        sandbox::FrameGlobals globals{};
        std::copy(viewProjection.begin(), viewProjection.end(), globals.viewProj);
        std::copy(invViewProjection.begin(), invViewProjection.end(), globals.invViewProj);
        globals.cameraPos[0] = camera.position.x;
        globals.cameraPos[1] = camera.position.y;
        globals.cameraPos[2] = camera.position.z;
        globals.cameraPos[3] = 1.0f;
        const Vec3 sun = velox::normalize(Vec3{0.35f, 0.75f, 0.30f});
        globals.sunDir[0] = sun.x;
        globals.sunDir[1] = sun.y;
        globals.sunDir[2] = sun.z;
        globals.sunDir[3] = 0.0f;

        std::vector<sandbox::DrawBatch> batches;
        buildBatches(scene, cache, batches);

        std::vector<sandbox::LineVertex> lineVertices;
        if (showLines) {
            std::vector<DebugLine> lines;
            scene.world().debugLines(lines);
            lineVertices.reserve(lines.size() * 2);
            for (const DebugLine& line : lines) {
                const float r = static_cast<float>((line.color >> 24) & 0xff) / 255.0f;
                const float g = static_cast<float>((line.color >> 16) & 0xff) / 255.0f;
                const float b = static_cast<float>((line.color >> 8) & 0xff) / 255.0f;
                const float a = static_cast<float>(line.color & 0xff) / 255.0f;
                lineVertices.push_back({line.a.x, line.a.y, line.a.z, r, g, b, a});
                lineVertices.push_back({line.b.x, line.b.y, line.b.z, r, g, b, a});
            }
        }

        ++frameCount;
        const float fpsSeconds = std::chrono::duration<float>(now - fpsStart).count();
        if (fpsSeconds >= 0.5f) {
            fps = static_cast<float>(frameCount) / fpsSeconds;
            frameCount = 0;
            fpsStart = now;
        }

        std::vector<sandbox::UiVertex> uiVertices;
        sandbox::appendText(uiVertices,
                            overlay(scene, substeps, paused, fps, spawnShape, showLines,
                                    drag.active),
                            window.width(), window.height());

        renderer.render(globals, batches, lineVertices, uiVertices,
                        window.width(), window.height());
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
