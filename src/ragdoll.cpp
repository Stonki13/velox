#include "velox/ragdoll.h"

#include <cmath>
#include <mutex>
#include <stdexcept>
#include <unordered_map>

namespace velox {
namespace {

struct RagdollRecord {
    const World* world = nullptr;
    BodyId root;
    std::vector<BodyId> bodies;
    std::vector<JointId> joints;
};

std::mutex recordsMutex;
std::vector<RagdollRecord> records;

bool finite(float value) { return std::isfinite(value); }
bool finite(Vec3 value) {
    return finite(value.x) && finite(value.y) && finite(value.z);
}

void remember(RagdollRecord record) {
    std::lock_guard<std::mutex> lock(recordsMutex);
    for (RagdollRecord& existing : records) {
        if (existing.world == record.world && existing.root == record.root) {
            existing = std::move(record);
            return;
        }
    }
    records.push_back(std::move(record));
}

RagdollRecord lookup(const World& world, BodyId root) {
    std::lock_guard<std::mutex> lock(recordsMutex);
    for (const RagdollRecord& record : records)
        if (record.world == &world && record.root == root)
            return record;
    return {};
}

void applyAuthoredMass(World& world, const RagdollBone& bone) {
    const Body state = world.bodyState(bone.body);
    if (!state.isDynamic()) return;
    if (state.invMass <= 0.0f || state.invInertia.x <= 0.0f ||
        state.invInertia.y <= 0.0f || state.invInertia.z <= 0.0f)
        throw std::invalid_argument("velox: ragdoll dynamic bone has invalid mass properties");
    const float sourceMass = 1.0f / state.invMass;
    const float scale = bone.mass / sourceMass;
    const Vec3 sourceInertia{1.0f / state.invInertia.x,
                              1.0f / state.invInertia.y,
                              1.0f / state.invInertia.z};
    world.setMassProperties(bone.body, bone.mass, sourceInertia * scale,
                            state.inertiaOrientation);
}

} // namespace

BodyId RagdollBuilder::Build(World& world, const std::vector<RagdollBone>& bones,
                             const std::vector<RagdollJoint>& links) {
    if (bones.empty())
        throw std::invalid_argument("velox: ragdoll requires at least one bone");
    if (links.size() + 1 != bones.size())
        throw std::invalid_argument("velox: ragdoll graph must contain bones - 1 links");

    std::unordered_map<uint64_t, size_t> boneIndex;
    boneIndex.reserve(bones.size());
    std::vector<std::vector<size_t>> children(bones.size());
    std::vector<uint32_t> parentCount(bones.size(), 0);
    for (size_t index = 0; index < bones.size(); ++index) {
        const RagdollBone& bone = bones[index];
        if (!world.isValid(bone.body) || !finite(bone.localCenterOfMass) ||
            !finite(bone.mass) || bone.mass <= 0.0f ||
            !boneIndex.emplace(bone.body.value, index).second)
            throw std::invalid_argument("velox: ragdoll contains an invalid or duplicate bone");
    }
    for (const RagdollJoint& link : links) {
        const auto parent = boneIndex.find(link.parent.value);
        const auto child = boneIndex.find(link.child.value);
        if (parent == boneIndex.end() || child == boneIndex.end() ||
            parent->second == child->second || !finite(link.anchor) ||
            !finite(link.axis) || lengthSq(link.axis) < 1e-12f ||
            !finite(link.swingLimit) || link.swingLimit < 0.0f ||
            link.swingLimit > 3.14159265f || !finite(link.twistLimit) ||
            link.twistLimit < 0.0f || link.twistLimit > 3.14159265f ||
            !finite(link.motorSpeed) || !finite(link.maxMotorTorque) ||
            link.maxMotorTorque < 0.0f)
            throw std::invalid_argument("velox: ragdoll contains an invalid link");
        if (++parentCount[child->second] != 1)
            throw std::invalid_argument("velox: ragdoll child has multiple parents");
        children[parent->second].push_back(child->second);
    }

    size_t rootIndex = bones.size();
    for (size_t index = 0; index < parentCount.size(); ++index)
        if (parentCount[index] == 0) {
            if (rootIndex != bones.size())
                throw std::invalid_argument("velox: ragdoll graph is disconnected");
            rootIndex = index;
        }
    if (rootIndex == bones.size())
        throw std::invalid_argument("velox: ragdoll graph contains a cycle");

    std::vector<uint8_t> visited(bones.size(), 0);
    std::vector<size_t> queue{rootIndex};
    for (size_t cursor = 0; cursor < queue.size(); ++cursor) {
        const size_t parent = queue[cursor];
        if (visited[parent])
            throw std::invalid_argument("velox: ragdoll graph contains a cycle");
        visited[parent] = 1;
        queue.insert(queue.end(), children[parent].begin(), children[parent].end());
    }
    for (uint8_t reached : visited)
        if (!reached)
            throw std::invalid_argument("velox: ragdoll graph is disconnected");

    for (const RagdollBone& bone : bones)
        applyAuthoredMass(world, bone);

    RagdollRecord record;
    record.world = &world;
    record.root = bones[rootIndex].body;
    record.bodies.reserve(bones.size());
    for (const RagdollBone& bone : bones) record.bodies.push_back(bone.body);
    record.joints.reserve(links.size());
    for (const RagdollJoint& link : links) {
        JointId id;
        if (link.enableMotor) {
            id = world.addHingeJoint(link.parent, link.child, link.anchor, link.axis);
            Joint& joint = world.joint(id);
            joint.enableMotor = true;
            joint.motorSpeed = link.motorSpeed;
            joint.maxMotorTorque = link.maxMotorTorque;
            joint.enableLimit = true;
            joint.lowerLimit = -link.twistLimit;
            joint.upperLimit = link.twistLimit;
        } else {
            id = world.addConeTwistJoint(link.parent, link.child, link.anchor, link.axis);
            Joint& joint = world.joint(id);
            joint.enableSwingLimit = true;
            joint.swingLimit = link.swingLimit;
            joint.enableTwistLimit = true;
            joint.lowerTwistLimit = -link.twistLimit;
            joint.upperTwistLimit = link.twistLimit;
        }
        record.joints.push_back(id);
    }
    remember(std::move(record));
    return bones[rootIndex].body;
}

void RagdollBuilder::SetMotorTorque(World& world, JointId id, float torque) {
    if (!finite(torque) || torque < 0.0f)
        throw std::invalid_argument("velox: ragdoll motor torque must be finite and non-negative");
    Joint& joint = world.joint(id);
    if (joint.type != JointType::Hinge)
        throw std::invalid_argument("velox: ragdoll motor requires a hinge link");
    joint.enableMotor = true;
    joint.maxMotorTorque = torque;
}

void RagdollBuilder::WakeAll(World& world, BodyId root) {
    const RagdollRecord record = lookup(world, root);
    if (record.world == nullptr)
        throw std::invalid_argument("velox: ragdoll root is not registered with this world");
    for (BodyId body : record.bodies)
        if (world.isValid(body)) world.wake(body);
}

std::vector<JointId> RagdollBuilder::Joints(World& world, BodyId root) {
    const RagdollRecord record = lookup(world, root);
    if (record.world == nullptr)
        throw std::invalid_argument("velox: ragdoll root is not registered with this world");
    std::vector<JointId> valid;
    valid.reserve(record.joints.size());
    for (JointId joint : record.joints)
        if (world.isValid(joint)) valid.push_back(joint);
    return valid;
}

} // namespace velox
