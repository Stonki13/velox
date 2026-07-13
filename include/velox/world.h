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

    // Advances the simulation. CCD is always on: bodies are swept along their
    // motion for the full substep, so no velocity can tunnel through geometry.
    void step(float dt);

private:
    std::vector<Body> bodies_;
    std::vector<Contact> contacts_;
    std::unique_ptr<Backend> backend_;
};

} // namespace velox
