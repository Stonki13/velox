#include "doctest.h"
#include <velox/velox.h>
#include <cmath>

using namespace velox;

TEST_CASE("World body creation and counting") {
    World world(BackendType::Cpu);
    CHECK(world.bodyCount() == 0);

    BodyId s = world.addSphere({0, 5, 0}, 0.5f, 1.0f);
    CHECK(world.bodyCount() == 1);
    CHECK(world.isValid(s));

    BodyId b = world.addBox({3, 5, 0}, {1, 1, 1}, 2.0f);
    CHECK(world.bodyCount() == 2);
    CHECK(world.isValid(b));

    BodyId p = world.addStaticPlane({0, 1, 0}, 0.0f);
    CHECK(world.bodyCount() == 3);
}

TEST_CASE("World body removal") {
    World world(BackendType::Cpu);
    BodyId a = world.addSphere({0, 5, 0}, 0.5f, 1.0f);
    BodyId b = world.addSphere({3, 5, 0}, 0.5f, 1.0f);
    CHECK(world.bodyCount() == 2);

    world.removeBody(a);
    CHECK(world.bodyCount() == 1);
    CHECK_FALSE(world.isValid(a));
    CHECK(world.isValid(b));
}

TEST_CASE("World setTransform") {
    World world(BackendType::Cpu);
    BodyId id = world.addSphere({0, 0, 0}, 0.5f, 1.0f);
    world.setTransform(id, {10, 20, 30}, {});
    Vec3 pos = world.body(id).position;
    CHECK(pos.x == doctest::Approx(10.0f));
    CHECK(pos.y == doctest::Approx(20.0f));
    CHECK(pos.z == doctest::Approx(30.0f));
}

TEST_CASE("World velocity setters") {
    World world(BackendType::Cpu);
    BodyId id = world.addSphere({0, 5, 0}, 0.5f, 1.0f);

    world.setLinearVelocity(id, {1, 2, 3});
    Vec3 v = world.body(id).velocity;
    CHECK(v.x == doctest::Approx(1.0f));
    CHECK(v.y == doctest::Approx(2.0f));
    CHECK(v.z == doctest::Approx(3.0f));

    world.setAngularVelocity(id, {0, 1, 0});
    Vec3 w = world.body(id).angularVelocity;
    CHECK(w.y == doctest::Approx(1.0f));
}

TEST_CASE("World sensor flag") {
    World world(BackendType::Cpu);
    BodyId id = world.addSphere({0, 5, 0}, 0.5f, 1.0f);
    CHECK_FALSE(world.isSensor(id));

    world.setSensor(id, true);
    CHECK(world.isSensor(id));

    world.setSensor(id, false);
    CHECK_FALSE(world.isSensor(id));
}

TEST_CASE("World gravity scale") {
    World world(BackendType::Cpu);
    BodyId id = world.addSphere({0, 5, 0}, 0.5f, 1.0f);
    CHECK(world.gravityScale(id) == doctest::Approx(1.0f));

    world.setGravityScale(id, 0.0f);
    CHECK(world.gravityScale(id) == doctest::Approx(0.0f));

    world.setGravityScale(id, 2.5f);
    CHECK(world.gravityScale(id) == doctest::Approx(2.5f));
}

TEST_CASE("World damping setters") {
    World world(BackendType::Cpu);
    BodyId id = world.addSphere({0, 5, 0}, 0.5f, 1.0f);

    world.setLinearDamping(id, 0.5f);
    CHECK(world.body(id).linearDamping == doctest::Approx(0.5f));

    world.setAngularDamping(id, 0.3f);
    CHECK(world.body(id).angularDamping == doctest::Approx(0.3f));

    CHECK_THROWS(world.setLinearDamping(id, -1.0f));
    CHECK_THROWS(world.setAngularDamping(id, -1.0f));
}

TEST_CASE("World collision filter") {
    World world(BackendType::Cpu);
    BodyId id = world.addSphere({0, 5, 0}, 0.5f, 1.0f);

    world.setCollisionFilter(id, 0x0002, 0x0004);
    CHECK(world.body(id).categoryBits == 0x0002);
    CHECK(world.body(id).maskBits == 0x0004);
}

TEST_CASE("World sleep control") {
    World world(BackendType::Cpu);
    BodyId id = world.addSphere({0, 5, 0}, 0.5f, 1.0f);
    CHECK(world.isSleepEnabled(id));

    world.setEnableSleep(id, false);
    CHECK_FALSE(world.isSleepEnabled(id));

    world.setEnableSleep(id, true);
    CHECK(world.isSleepEnabled(id));

    world.sleepBody(id);
    CHECK(world.body(id).asleep != 0);

    world.wakeBody(id);
    CHECK(world.body(id).asleep == 0);
}

TEST_CASE("World fixed rotation") {
    World world(BackendType::Cpu);
    BodyId id = world.addSphere({0, 5, 0}, 0.5f, 1.0f);
    CHECK_FALSE(world.isFixedRotation(id));

    world.setAngularVelocity(id, {0, 5, 0});
    world.setFixedRotation(id, true);
    CHECK(world.isFixedRotation(id));
    CHECK(world.body(id).angularVelocity.y == doctest::Approx(0.0f));
}

TEST_CASE("World explode applies radial impulse") {
    World world(BackendType::Cpu);
    world.gravity = {0, 0, 0};
    BodyId id = world.addSphere({5, 0, 0}, 0.5f, 1.0f);
    world.setLinearVelocity(id, {0, 0, 0});

    world.explode({0, 0, 0}, 10.0f, 10.0f);

    Vec3 v = world.body(id).velocity;
    CHECK(v.x > 0.0f);
    CHECK(v.y == doctest::Approx(0.0f).epsilon(1e-5f));
    CHECK(v.z == doctest::Approx(0.0f).epsilon(1e-5f));
}

TEST_CASE("World addRoundedBox and addEllipsoid") {
    World world(BackendType::Cpu);
    BodyId rb = world.addRoundedBox({0, 5, 0}, {1, 1, 1}, 0.1f, 1.0f);
    CHECK(world.isValid(rb));
    CHECK(world.body(rb).shape == ShapeType::RoundedBox);

    BodyId el = world.addEllipsoid({3, 5, 0}, {1, 2, 3}, 2.0f);
    CHECK(world.isValid(el));
    CHECK(world.body(el).shape == ShapeType::Ellipsoid);
}

TEST_CASE("World step advances simulation") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};
    BodyId id = world.addSphere({0, 10, 0}, 0.5f, 1.0f);
    float y0 = world.body(id).position.y;

    world.step(1.0f / 60.0f);
    float y1 = world.body(id).position.y;
    CHECK(y1 < y0);
}

TEST_CASE("World snapshot save/restore") {
    World world(BackendType::Cpu);
    world.gravity = {0, -10, 0};
    BodyId id = world.addSphere({0, 5, 0}, 0.5f, 1.0f);
    world.step(1.0f / 60.0f);

    WorldSnapshot snap = world.saveSnapshot();
    float yBefore = world.body(id).position.y;

    world.step(1.0f / 60.0f);
    float yAfter = world.body(id).position.y;
    CHECK(yAfter < yBefore);

    world.restoreSnapshot(snap);
    float yRestored = world.body(id).position.y;
    CHECK(yRestored == doctest::Approx(yBefore));
}

TEST_CASE("World invalid input throws") {
    World world(BackendType::Cpu);
    CHECK_THROWS(world.addSphere({NAN, 0, 0}, 0.5f, 1.0f));
    CHECK_THROWS(world.addSphere({0, 0, 0}, -1.0f, 1.0f));
    CHECK_THROWS(world.addBox({0, 0, 0}, {0, 1, 1}, 1.0f));
}
