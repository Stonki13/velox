#pragma once
#include "backend.h"
#include <memory>
#include <vector>

namespace velox {

class World {
public:
    World();

    Vec3 gravity{0, -9.81f, 0};

    BodyId addSphere(Vec3 position, float radius, float mass);
    BodyId addBox(Vec3 position, Vec3 halfExtents, float mass);
    BodyId addCapsule(Vec3 position, float radius, float halfHeight, float mass);
    BodyId addStaticPlane(Vec3 normal, float offset);
    // Static triangle mesh (level geometry). vertices: xyz triples,
    // indices: 3 per triangle.
    BodyId addStaticMesh(const std::vector<Vec3>& vertices,
                         const std::vector<uint32_t>& indices);

    Body& body(BodyId id) { return bodies_[id]; }
    const Body& body(BodyId id) const { return bodies_[id]; }
    size_t bodyCount() const { return bodies_.size(); }

    // Advances the simulation using Predictive Contact Sweeping: speculative
    // contacts solved iteratively, backed by a conservative-advancement sweep
    // safety net, so no velocity can tunnel through geometry and grazing
    // contact stays smooth.
    void step(float dt);

private:
    std::vector<Body> bodies_;
    std::vector<Contact> contacts_;
    struct PrevState { Vec3 position; Quat orientation; };
    std::vector<PrevState> prev_;
    MeshSoup meshes_;
    std::unique_ptr<Backend> backend_;
};

} // namespace velox
