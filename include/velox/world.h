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
    BodyId addStaticPlane(Vec3 normal, float offset);

    Body& body(BodyId id) { return bodies_[id]; }
    const Body& body(BodyId id) const { return bodies_[id]; }
    size_t bodyCount() const { return bodies_.size(); }

    // Advances the simulation using Predictive Contact Sweeping: speculative
    // contacts solved iteratively, backed by an exact sweep safety net, so no
    // velocity can tunnel through geometry and grazing contact stays smooth.
    void step(float dt);

private:
    std::vector<Body> bodies_;
    std::vector<Contact> contacts_;
    std::vector<Vec3> prevPositions_;
    std::unique_ptr<Backend> backend_;
};

} // namespace velox
