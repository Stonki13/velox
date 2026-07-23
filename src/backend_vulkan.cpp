// Vulkan compute backend: cross-vendor GPU acceleration (NVIDIA, AMD, Intel —
// any driver exposing a Vulkan 1.1 compute queue), complementing the
// NVIDIA-only CUDA backend.
//
// Stage 2 scope: velocity integration AND the graph-colored contact solve run
// on the GPU (shaders/integrate.comp, shaders/solve_contacts.comp — the solve
// is a faithful port of narrowphase.h's warmStartContact/solveContact
// TwoAxisCoulomb path, using the same host-side deterministic sort + greedy
// coloring the CUDA backend uses). Broad phase and narrow phase still
// delegate to the owned CPU backend, as do configurations the GPU path does
// not cover yet (ConeBlockSolver friction, Adaptive iteration policy).
//
// Stage 4 (advanceSubsteps below) additionally fuses the ENTIRE joint-free
// substep loop -- velocity solve, position/orientation integration
// (shaders/integrate_transforms.comp, isotropic inertia only) -- into a
// single command-buffer submission per step, keeping bodies and colored
// contacts GPU-resident the whole time instead of one submission per
// backend call. Measured (docs/roadmap/PROTO_COMPETITIVE_NOTES.md): a wash
// against the unfused stage-2/3 path, not a win -- the dispatch/barrier
// count per step is unchanged by fusion (same number of colored solve
// passes either way), and that appears to be the actual bottleneck, not
// the number of CPU-GPU fence waits fusion removes. Kept enabled (it is
// not a regression and the architecture is the right foundation for a
// future win, e.g. a real multi-substep pipeline barrier restructuring or
// moving coloring itself on-device), but the crossover point from stage 2
// stands.
//
// Like the CUDA backend, this backend is NOT part of the strict bitwise
// cross-platform determinism guarantee: GPU float contraction differs from
// the CPU reference path. Use BackendType::Cpu for lockstep replay.
#include "velox/backend.h"

#include <vulkan/vulkan.h>

#include "integrate_spv.h"
#include "solve_contacts_spv.h"
#include "narrow_sphere_spv.h"
#include "integrate_transforms_spv.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <vector>

namespace velox {

namespace {

// Must match the GpuBody struct in the shaders (std430: eight vec4s).
struct GpuBody {
    float velocity[3];        float invMass;
    float angularVelocity[3]; float gravityScale;
    float force[3];           float linearDamping;
    float torque[3];          float angularDamping;
    float invInertia[3];      float flagsBits;
    float orientation[4];
    float inertiaOrientation[4];
    float position[3];        float speculativeDistance;
    float shapeData[4];       // sphere: (radius,0,0,0); plane: (nx,ny,nz,offset)
    float material[4];        // restitution, friction, rollingFriction, spinningFriction
    float frictionScale[3];   float maxPointSpeed;
};
static_assert(sizeof(GpuBody) == 176, "GpuBody must match the std430 layout");

// Must match GpuContact in the shaders (eight vec4-sized rows).
struct GpuContact {
    float normal[3];       float bias0;
    float anchorA[3];      float friction1;
    float anchorB[3];      float friction2;
    float vn0;             float restitution;
    float rollingFriction; float spinningFriction;
    float normalImpulse;   float tangentImpulse1;
    float tangentImpulse2; float spinningImpulse;
    float rollingImpulse1; float rollingImpulse2;
    uint32_t featureLo, featureHi;
    uint32_t a, b, skip, pad;
    float point[3];        float gap;
};
static_assert(sizeof(GpuContact) == 128, "GpuContact must match the std430 layout");

struct IntegratePush {
    float gravity[3];
    float dt;
    uint32_t count;
};

struct SolvePush {
    uint32_t mode;
    uint32_t first;
    uint32_t count;
    float dt;
};

struct NarrowPush {
    uint32_t pairCount;
    uint32_t capacity;
    float dt;
};

struct TransformPush {
    float dt;
    uint32_t count;
};

constexpr uint32_t kFlagIntegrate = 1u;
constexpr uint32_t kFlagTorque = 2u;
constexpr uint32_t kFlagDynamic = 4u;
constexpr uint32_t kFlagSolveAngular = 8u;
constexpr uint32_t kFlagSphereAnchor = 16u;
constexpr uint32_t kFlagPlane = 32u;
constexpr uint32_t kFlagSweep = 64u;
// Transform-integration eligibility (!isStatic() && !isLocked() && !asleep)
// is deliberately distinct from kFlagIntegrate (velocity-integration
// eligibility, which additionally requires isDynamic()): a kinematic body
// advances its transform from an externally set velocity without ever being
// gravity/force-integrated. Bits 8-13 are reserved by frictionCombine/
// restitutionCombine below (3 bits each).
constexpr uint32_t kFlagTransformIntegrate = 128u;
constexpr uint32_t kFlagRotationLocked = 1u << 14;
constexpr uint32_t kPushRange = 32; // covers every shader's push block

class VulkanBackend final : public Backend {
public:
    static VulkanBackend* create() {
        std::unique_ptr<VulkanBackend> backend(new VulkanBackend());
        if (!backend->initialize()) return nullptr;
        return backend.release();
    }

    ~VulkanBackend() override {
        if (device_) {
            vkDeviceWaitIdle(device_);
            destroyBuffer(bodyBuffer_, bodyMemory_, mappedBodies_);
            destroyBuffer(contactBuffer_, contactMemory_, mappedContacts_);
            destroyBuffer(pairBuffer_, pairMemory_, mappedPairs_);
            destroyBuffer(counterBuffer_, counterMemory_, mappedCounter_);
            if (fence_) vkDestroyFence(device_, fence_, nullptr);
            if (commandPool_) vkDestroyCommandPool(device_, commandPool_, nullptr);
            if (descriptorPool_) vkDestroyDescriptorPool(device_, descriptorPool_, nullptr);
            if (integratePipeline_) vkDestroyPipeline(device_, integratePipeline_, nullptr);
            if (solvePipeline_) vkDestroyPipeline(device_, solvePipeline_, nullptr);
            if (narrowPipeline_) vkDestroyPipeline(device_, narrowPipeline_, nullptr);
            if (transformPipeline_) vkDestroyPipeline(device_, transformPipeline_, nullptr);
            if (pipelineLayout_) vkDestroyPipelineLayout(device_, pipelineLayout_, nullptr);
            if (descriptorLayout_) vkDestroyDescriptorSetLayout(device_, descriptorLayout_, nullptr);
            if (integrateModule_) vkDestroyShaderModule(device_, integrateModule_, nullptr);
            if (solveModule_) vkDestroyShaderModule(device_, solveModule_, nullptr);
            if (narrowModule_) vkDestroyShaderModule(device_, narrowModule_, nullptr);
            if (transformModule_) vkDestroyShaderModule(device_, transformModule_, nullptr);
            vkDestroyDevice(device_, nullptr);
        }
        if (instance_) vkDestroyInstance(instance_, nullptr);
    }

    const char* name() const override { return "vulkan"; }

    void integrate(std::vector<Body>& bodies, const Vec3& gravity, float dt) override {
        const uint32_t n = static_cast<uint32_t>(bodies.size());
        if (n == 0) return;
        if (!ensureBodyCapacity(n)) {
            cpu_->integrate(bodies, gravity, dt);
            return;
        }
        packBodies(bodies);
        if (!beginCommands()) { cpu_->integrate(bodies, gravity, dt); return; }
        recordIntegrate(gravity, dt, n);
        if (!submitCommands()) { cpu_->integrate(bodies, gravity, dt); return; }
        unpackVelocities(bodies);
    }

    void solveVelocities(std::vector<Body>& bodies, std::vector<Contact>& contacts,
                         float dt, bool warmStart, const SolverOptions& options) override {
        // Scope guard: the GPU shader implements the default TwoAxisCoulomb
        // scalar-row path with fixed iterations. The 3x3 cone block solver
        // and the adaptive early-out (an ordered reduction) stay on the CPU
        // reference path rather than silently approximating them.
        if (options.frictionModel == FrictionModel::ConeBlockSolver ||
            options.iterationPolicy == IterationPolicy::Adaptive) {
            cpu_->solveVelocities(bodies, contacts, dt, warmStart, options);
            gpuSolveActive_ = false;
            return;
        }
        lastVelocityIterations_ = 0;
        const uint32_t n = static_cast<uint32_t>(bodies.size());
        const uint32_t m = static_cast<uint32_t>(contacts.size());
        if (m == 0) return;
        if (!ensureBodyCapacity(n) ||
            (warmStart && !ensureContactCapacity(m))) {
            cpu_->solveVelocities(bodies, contacts, dt, warmStart, options);
            gpuSolveActive_ = false;
            return;
        }

        if (warmStart && !colorContacts(bodies, contacts)) {
            cpu_->solveVelocities(bodies, contacts, dt, warmStart, options);
            gpuSolveActive_ = false;
            return;
        }
        if (!gpuSolveActive_ || solveCount_ != m) {
            // A non-warm-start call without a matching device-resident set
            // (e.g. the warm-start substep ran on the CPU path): stay on CPU.
            cpu_->solveVelocities(bodies, contacts, dt, warmStart, options);
            return;
        }

        packBodies(bodies);

        // Colored order converges slower per sweep than sequential order, so
        // run twice the sweeps — the CUDA backend's measured trade-off.
        const int gpuIterations = 2 * options.velocityIterations;
        if (!beginCommands()) {
            cpu_->solveVelocities(bodies, contacts, dt, warmStart, options);
            gpuSolveActive_ = false;
            return;
        }
        if (warmStart) recordSolvePass(0, dt);
        for (int iteration = 0; iteration < gpuIterations; ++iteration)
            recordSolvePass(1, dt);
        if (!submitCommands()) {
            cpu_->solveVelocities(bodies, contacts, dt, warmStart, options);
            gpuSolveActive_ = false;
            return;
        }
        lastVelocityIterations_ = static_cast<uint32_t>(gpuIterations);
        unpackVelocities(bodies);
    }

    // Stage 4: fuses the ENTIRE substep loop (velocity solve, joint-free,
    // plus position/orientation integration) into ONE submission, eliminating
    // the per-substep fence wait that stages 2/3 pay via World's default
    // per-call loop (integrate + solveVelocities as separate submissions,
    // repeated per substep). Bodies and colored contacts stay device-resident
    // for the whole step; only one upload and one download happen.
    //
    // Scope guards, deliberate and narrow:
    //  - Any joint present: this backend has no GPU joint solver at all
    //    (unlike CUDA's hinge/distance kernels), so bail entirely and let
    //    World's default per-call loop run joints on the CPU path as usual.
    //  - ConeBlockSolver / Adaptive: same reasons solveVelocities excludes them.
    //  - Any DYNAMIC body with anisotropic inverse inertia: shaders/
    //    integrate_transforms.comp only ports Body::advanceOrientation's
    //    ISOTROPIC branch (the common case: spheres, cubes, any symmetric
    //    shape). The anisotropic branch is an iterative fixed-point gyroscopic
    //    solve that is not worth hand-porting to GLSL for this pass; bodies
    //    that need it fall back to the CPU path for the whole step, not a
    //    silent approximation.
    // World already called integrate() for substep 0 before invoking this
    // (see world.cpp's "first substep's gravity" comment); this method only
    // needs to redo it for s > 0, matching the CUDA backend's own contract.
    bool advanceSubsteps(std::vector<Body>& bodies, std::vector<Contact>& contacts,
                         std::vector<Joint>& joints, const Vec3& gravity, float dt,
                         int substeps, const SolverOptions& options) override {
        if (!joints.empty()) return false;
        if (options.frictionModel == FrictionModel::ConeBlockSolver ||
            options.iterationPolicy == IterationPolicy::Adaptive)
            return false;
        const uint32_t n = static_cast<uint32_t>(bodies.size());
        if (n == 0) return false;
        for (const Body& b : bodies) {
            if (!b.isDynamic()) continue;
            if (b.invInertia.x != b.invInertia.y || b.invInertia.y != b.invInertia.z)
                return false; // needs the CPU's iterative gyroscopic solve
        }
        const uint32_t m = static_cast<uint32_t>(contacts.size());
        const bool hasContacts = m > 0;
        if (!ensureBodyCapacity(n) || (hasContacts && !ensureContactCapacity(m)))
            return false;
        if (hasContacts && !colorContacts(bodies, contacts)) return false;

        packBodies(bodies);
        if (!beginCommands()) { gpuSolveActive_ = false; return false; }

        const int gpuIterations = 2 * options.velocityIterations;
        if (hasContacts) recordSolvePass(0, dt); // warm start substep 0's contacts
        for (int i = 0; i < gpuIterations; ++i)
            if (hasContacts) recordSolvePass(1, dt);
        recordIntegrateTransforms(n, dt);

        for (int s = 1; s < substeps; ++s) {
            recordIntegrate(gravity, dt, n);
            for (int i = 0; i < gpuIterations; ++i)
                if (hasContacts) recordSolvePass(1, dt); // reuse resident impulses, no recolor
            recordIntegrateTransforms(n, dt);
        }

        if (!submitCommands()) { gpuSolveActive_ = false; return false; }

        unpackFull(bodies);
        lastVelocityIterations_ = static_cast<uint32_t>(substeps * gpuIterations);
        return true;
    }

    void fetchImpulses(std::vector<Contact>& contacts) override {
        if (!gpuSolveActive_ || solveCount_ == 0 ||
            contacts.size() != static_cast<size_t>(solveCount_)) {
            cpu_->fetchImpulses(contacts);
            return;
        }
        const GpuContact* mapped = static_cast<const GpuContact*>(mappedContacts_);
        for (uint32_t k = 0; k < solveCount_; ++k) {
            contacts[k].normalImpulse = mapped[k].normalImpulse;
            contacts[k].tangentImpulse1 = mapped[k].tangentImpulse1;
            contacts[k].tangentImpulse2 = mapped[k].tangentImpulse2;
            contacts[k].rollingImpulse1 = mapped[k].rollingImpulse1;
            contacts[k].rollingImpulse2 = mapped[k].rollingImpulse2;
            contacts[k].spinningImpulse = mapped[k].spinningImpulse;
        }
    }

    // Broad phase stays on the host (the World's incremental AABB tree).
    // Narrow phase runs on the GPU for scenes made entirely of spheres and
    // planes — the analytic pairs narrow_sphere.comp ports exactly; any
    // other shape in the scene routes the whole call to the CPU reference
    // path (GJK/EPA/manifold clipping are not ported).
    bool wantsHostPairs() const override { return cpu_->wantsHostPairs(); }
    void findContacts(const std::vector<Body>& bodies, const MeshSoup& meshes,
                      float dt, const std::vector<uint64_t>* hostPairs,
                      std::vector<Contact>& out) override {
        const uint32_t n = static_cast<uint32_t>(bodies.size());
        // Measured on the canonical sphere-rain scenes (RTX 5080): the extra
        // synchronous upload/dispatch/readback per step made whole-step time
        // WORSE than the stage-2 CPU narrow phase (8192 bodies: 159 ms vs
        // 88 ms). Kept as an explicit opt-in experiment rather than shipped
        // as a default regression; making it win needs the narrow phase and
        // solve fused into one submission with device-resident bodies.
        bool gpuEligible = gpuNarrowEnabled_ && hostPairs != nullptr &&
                           !hostPairs->empty() && n > 0;
        for (uint32_t i = 0; gpuEligible && i < n; ++i)
            gpuEligible = bodies[i].shape == ShapeType::Sphere ||
                          bodies[i].shape == ShapeType::Plane;
        const uint32_t pairCount =
            gpuEligible ? static_cast<uint32_t>(hostPairs->size()) : 0;
        if (!gpuEligible || !ensureBodyCapacity(n) ||
            !ensurePairCapacity(pairCount) || !ensureContactCapacity(pairCount)) {
            cpu_->findContacts(bodies, meshes, dt, hostPairs, out);
            return;
        }

        packBodies(bodies);
        uint32_t* pairWords = static_cast<uint32_t*>(mappedPairs_);
        for (uint32_t p = 0; p < pairCount; ++p) {
            pairWords[p * 2] = static_cast<uint32_t>((*hostPairs)[p] >> 32);
            pairWords[p * 2 + 1] = static_cast<uint32_t>((*hostPairs)[p]);
        }
        *static_cast<uint32_t*>(mappedCounter_) = 0;

        NarrowPush push{};
        push.pairCount = pairCount;
        push.capacity = pairCount; // sphere pairs emit at most one contact each
        push.dt = dt;
        if (!beginCommands()) {
            cpu_->findContacts(bodies, meshes, dt, hostPairs, out);
            return;
        }
        vkCmdBindPipeline(commandBuffer_, VK_PIPELINE_BIND_POINT_COMPUTE, narrowPipeline_);
        vkCmdBindDescriptorSets(commandBuffer_, VK_PIPELINE_BIND_POINT_COMPUTE,
                                pipelineLayout_, 0, 1, &descriptorSet_, 0, nullptr);
        vkCmdPushConstants(commandBuffer_, pipelineLayout_, VK_SHADER_STAGE_COMPUTE_BIT,
                           0, sizeof(push), &push);
        vkCmdDispatch(commandBuffer_, (pairCount + 127) / 128, 1, 1);
        if (!submitCommands()) {
            cpu_->findContacts(bodies, meshes, dt, hostPairs, out);
            return;
        }

        uint32_t count = *static_cast<uint32_t*>(mappedCounter_);
        if (count > pairCount) count = pairCount; // atomic overshoot past capacity
        const GpuContact* mapped = static_cast<const GpuContact*>(mappedContacts_);
        out.clear();
        out.resize(count);
        for (uint32_t k = 0; k < count; ++k) {
            const GpuContact& g = mapped[k];
            Contact& c = out[k];
            std::memset(&c, 0, sizeof(Contact));
            c.a = g.a;
            c.b = g.b;
            c.featureKey = (uint64_t)g.featureHi << 32 | g.featureLo;
            std::memcpy(&c.normal, g.normal, sizeof(float) * 3);
            std::memcpy(&c.point, g.point, sizeof(float) * 3);
            std::memcpy(&c.localAnchorA, g.anchorA, sizeof(float) * 3);
            std::memcpy(&c.localAnchorB, g.anchorB, sizeof(float) * 3);
            c.gap = g.gap;
            c.bias0 = g.bias0;
            c.vn0 = g.vn0;
            c.restitution = g.restitution;
            c.friction1 = g.friction1;
            c.friction2 = g.friction2;
            c.rollingFriction = g.rollingFriction;
            c.spinningFriction = g.spinningFriction;
        }
    }
    void invalidateCaches() override {
        gpuSolveActive_ = false;
        solveCount_ = 0;
        cpu_->invalidateCaches();
    }
    void setWorkerCount(uint32_t count) override { cpu_->setWorkerCount(count); }
    uint32_t workerCount() const override { return cpu_->workerCount(); }
    void setParallelIslands(bool enabled) override { cpu_->setParallelIslands(enabled); }
    void setTaskSystem(TaskSystem* system) override { cpu_->setTaskSystem(system); }
    uint32_t lastVelocityIterations() const override {
        return lastVelocityIterations_ ? lastVelocityIterations_
                                       : cpu_->lastVelocityIterations();
    }
    size_t lastIslandCount() const override { return cpu_->lastIslandCount(); }
    void parallelChunks(size_t items, size_t minPerChunk,
                        const std::function<void(size_t, size_t, size_t)>& fn,
                        size_t* chunkCountOut) override {
        cpu_->parallelChunks(items, minPerChunk, fn, chunkCountOut);
    }

private:
    VulkanBackend() : cpu_(createCpuBackend()) {}

    bool initialize() {
        VkApplicationInfo appInfo{VK_STRUCTURE_TYPE_APPLICATION_INFO};
        appInfo.pApplicationName = "velox";
        appInfo.apiVersion = VK_API_VERSION_1_1;
        VkInstanceCreateInfo instanceInfo{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
        instanceInfo.pApplicationInfo = &appInfo;
        if (vkCreateInstance(&instanceInfo, nullptr, &instance_) != VK_SUCCESS)
            return false;

        uint32_t deviceCount = 0;
        vkEnumeratePhysicalDevices(instance_, &deviceCount, nullptr);
        if (deviceCount == 0) return false;
        std::vector<VkPhysicalDevice> devices(deviceCount);
        vkEnumeratePhysicalDevices(instance_, &deviceCount, devices.data());

        for (int pass = 0; pass < 2 && !physicalDevice_; ++pass) {
            for (VkPhysicalDevice candidate : devices) {
                VkPhysicalDeviceProperties props;
                vkGetPhysicalDeviceProperties(candidate, &props);
                if (pass == 0 &&
                    props.deviceType != VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU)
                    continue;
                uint32_t family = 0;
                if (findComputeQueueFamily(candidate, family)) {
                    physicalDevice_ = candidate;
                    queueFamily_ = family;
                    break;
                }
            }
        }
        if (!physicalDevice_) return false;

        float priority = 1.0f;
        VkDeviceQueueCreateInfo queueInfo{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
        queueInfo.queueFamilyIndex = queueFamily_;
        queueInfo.queueCount = 1;
        queueInfo.pQueuePriorities = &priority;
        VkDeviceCreateInfo deviceInfo{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
        deviceInfo.queueCreateInfoCount = 1;
        deviceInfo.pQueueCreateInfos = &queueInfo;
        if (vkCreateDevice(physicalDevice_, &deviceInfo, nullptr, &device_) != VK_SUCCESS)
            return false;
        vkGetDeviceQueue(device_, queueFamily_, 0, &queue_);

        if (!createShaderModule(velox_integrate_spv, velox_integrate_spv_size,
                                integrateModule_) ||
            !createShaderModule(velox_solve_contacts_spv, velox_solve_contacts_spv_size,
                                solveModule_) ||
            !createShaderModule(velox_narrow_sphere_spv, velox_narrow_sphere_spv_size,
                                narrowModule_) ||
            !createShaderModule(velox_integrate_transforms_spv,
                                velox_integrate_transforms_spv_size, transformModule_))
            return false;

        VkDescriptorSetLayoutBinding bindings[4]{};
        for (uint32_t i = 0; i < 4; ++i) {
            bindings[i].binding = i;
            bindings[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            bindings[i].descriptorCount = 1;
            bindings[i].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
        }
        VkDescriptorSetLayoutCreateInfo layoutInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
        layoutInfo.bindingCount = 4;
        layoutInfo.pBindings = bindings;
        if (vkCreateDescriptorSetLayout(device_, &layoutInfo, nullptr, &descriptorLayout_) != VK_SUCCESS)
            return false;

        VkPushConstantRange pushRange{};
        pushRange.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
        pushRange.size = kPushRange;
        VkPipelineLayoutCreateInfo pipelineLayoutInfo{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
        pipelineLayoutInfo.setLayoutCount = 1;
        pipelineLayoutInfo.pSetLayouts = &descriptorLayout_;
        pipelineLayoutInfo.pushConstantRangeCount = 1;
        pipelineLayoutInfo.pPushConstantRanges = &pushRange;
        if (vkCreatePipelineLayout(device_, &pipelineLayoutInfo, nullptr, &pipelineLayout_) != VK_SUCCESS)
            return false;

        if (!createComputePipeline(integrateModule_, integratePipeline_) ||
            !createComputePipeline(solveModule_, solvePipeline_) ||
            !createComputePipeline(narrowModule_, narrowPipeline_) ||
            !createComputePipeline(transformModule_, transformPipeline_))
            return false;

        VkDescriptorPoolSize poolSize{VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 4};
        VkDescriptorPoolCreateInfo poolInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
        poolInfo.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
        poolInfo.maxSets = 1;
        poolInfo.poolSizeCount = 1;
        poolInfo.pPoolSizes = &poolSize;
        if (vkCreateDescriptorPool(device_, &poolInfo, nullptr, &descriptorPool_) != VK_SUCCESS)
            return false;
        VkDescriptorSetAllocateInfo setInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        setInfo.descriptorPool = descriptorPool_;
        setInfo.descriptorSetCount = 1;
        setInfo.pSetLayouts = &descriptorLayout_;
        if (vkAllocateDescriptorSets(device_, &setInfo, &descriptorSet_) != VK_SUCCESS)
            return false;

        VkCommandPoolCreateInfo commandPoolInfo{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
        commandPoolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
        commandPoolInfo.queueFamilyIndex = queueFamily_;
        if (vkCreateCommandPool(device_, &commandPoolInfo, nullptr, &commandPool_) != VK_SUCCESS)
            return false;
        VkCommandBufferAllocateInfo commandInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
        commandInfo.commandPool = commandPool_;
        commandInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        commandInfo.commandBufferCount = 1;
        if (vkAllocateCommandBuffers(device_, &commandInfo, &commandBuffer_) != VK_SUCCESS)
            return false;

        VkFenceCreateInfo fenceInfo{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
        if (vkCreateFence(device_, &fenceInfo, nullptr, &fence_) != VK_SUCCESS)
            return false;

        // Every descriptor binding must always reference a valid buffer, so
        // allocate minimal placeholders up front; real workloads grow them.
        return ensureBodyCapacity(1) && ensureContactCapacity(1) &&
               ensurePairCapacity(1) &&
               allocateBuffer(sizeof(uint32_t), counterBuffer_, counterMemory_,
                              mappedCounter_) &&
               (updateDescriptors(), true);
    }

    bool createShaderModule(const unsigned char* bytes, size_t size, VkShaderModule& out) {
        std::vector<uint32_t> code(size / sizeof(uint32_t));
        std::memcpy(code.data(), bytes, size);
        VkShaderModuleCreateInfo info{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
        info.codeSize = size;
        info.pCode = code.data();
        return vkCreateShaderModule(device_, &info, nullptr, &out) == VK_SUCCESS;
    }

    bool createComputePipeline(VkShaderModule module, VkPipeline& out) {
        VkComputePipelineCreateInfo info{VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};
        info.stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        info.stage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
        info.stage.module = module;
        info.stage.pName = "main";
        info.layout = pipelineLayout_;
        return vkCreateComputePipelines(device_, VK_NULL_HANDLE, 1, &info,
                                        nullptr, &out) == VK_SUCCESS;
    }

    static bool findComputeQueueFamily(VkPhysicalDevice device, uint32_t& family) {
        uint32_t count = 0;
        vkGetPhysicalDeviceQueueFamilyProperties(device, &count, nullptr);
        std::vector<VkQueueFamilyProperties> families(count);
        vkGetPhysicalDeviceQueueFamilyProperties(device, &count, families.data());
        for (uint32_t i = 0; i < count; ++i)
            if (families[i].queueFlags & VK_QUEUE_COMPUTE_BIT) {
                family = i;
                return true;
            }
        return false;
    }

    void destroyBuffer(VkBuffer& buffer, VkDeviceMemory& memory, void*& mapped) {
        if (mapped) {
            vkUnmapMemory(device_, memory);
            mapped = nullptr;
        }
        if (buffer) {
            vkDestroyBuffer(device_, buffer, nullptr);
            buffer = VK_NULL_HANDLE;
        }
        if (memory) {
            vkFreeMemory(device_, memory, nullptr);
            memory = VK_NULL_HANDLE;
        }
    }

    bool allocateBuffer(VkDeviceSize size, VkBuffer& buffer, VkDeviceMemory& memory,
                        void*& mapped) {
        VkBufferCreateInfo bufferInfo{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
        bufferInfo.size = size;
        bufferInfo.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
        bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        if (vkCreateBuffer(device_, &bufferInfo, nullptr, &buffer) != VK_SUCCESS)
            return false;

        VkMemoryRequirements requirements;
        vkGetBufferMemoryRequirements(device_, buffer, &requirements);
        VkPhysicalDeviceMemoryProperties memoryProps;
        vkGetPhysicalDeviceMemoryProperties(physicalDevice_, &memoryProps);
        constexpr VkMemoryPropertyFlags wanted =
            VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
        uint32_t typeIndex = UINT32_MAX;
        for (uint32_t i = 0; i < memoryProps.memoryTypeCount; ++i)
            if ((requirements.memoryTypeBits & (1u << i)) &&
                (memoryProps.memoryTypes[i].propertyFlags & wanted) == wanted) {
                typeIndex = i;
                break;
            }
        if (typeIndex == UINT32_MAX) return false;

        VkMemoryAllocateInfo allocInfo{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
        allocInfo.allocationSize = requirements.size;
        allocInfo.memoryTypeIndex = typeIndex;
        if (vkAllocateMemory(device_, &allocInfo, nullptr, &memory) != VK_SUCCESS)
            return false;
        if (vkBindBufferMemory(device_, buffer, memory, 0) != VK_SUCCESS) return false;
        return vkMapMemory(device_, memory, 0, VK_WHOLE_SIZE, 0, &mapped) == VK_SUCCESS;
    }

    void updateDescriptors() {
        if (!bodyBuffer_ || !contactBuffer_ || !pairBuffer_ || !counterBuffer_)
            return; // still initializing; the last allocation triggers the write
        VkDescriptorBufferInfo bufferInfos[4] = {
            {bodyBuffer_, 0, VK_WHOLE_SIZE},
            {contactBuffer_, 0, VK_WHOLE_SIZE},
            {pairBuffer_, 0, VK_WHOLE_SIZE},
            {counterBuffer_, 0, VK_WHOLE_SIZE},
        };
        VkWriteDescriptorSet writes[4]{};
        for (uint32_t i = 0; i < 4; ++i) {
            writes[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            writes[i].dstSet = descriptorSet_;
            writes[i].dstBinding = i;
            writes[i].descriptorCount = 1;
            writes[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            writes[i].pBufferInfo = &bufferInfos[i];
        }
        vkUpdateDescriptorSets(device_, 4, writes, 0, nullptr);
    }

    bool ensurePairCapacity(uint32_t pairCount) {
        const VkDeviceSize needed = VkDeviceSize(pairCount) * 2 * sizeof(uint32_t);
        if (needed <= pairCapacity_ && pairBuffer_) return true;
        vkDeviceWaitIdle(device_);
        destroyBuffer(pairBuffer_, pairMemory_, mappedPairs_);
        pairCapacity_ = 0;
        if (!allocateBuffer(needed, pairBuffer_, pairMemory_, mappedPairs_)) return false;
        pairCapacity_ = needed;
        updateDescriptors();
        return true;
    }

    bool ensureBodyCapacity(uint32_t count) {
        const VkDeviceSize needed = VkDeviceSize(count) * sizeof(GpuBody);
        if (needed <= bodyCapacity_ && bodyBuffer_) return true;
        vkDeviceWaitIdle(device_);
        destroyBuffer(bodyBuffer_, bodyMemory_, mappedBodies_);
        bodyCapacity_ = 0;
        if (!allocateBuffer(needed, bodyBuffer_, bodyMemory_, mappedBodies_)) return false;
        bodyCapacity_ = needed;
        if (contactBuffer_) updateDescriptors();
        return true;
    }

    bool ensureContactCapacity(uint32_t count) {
        const VkDeviceSize needed = VkDeviceSize(count) * sizeof(GpuContact);
        if (needed <= contactCapacity_ && contactBuffer_) return true;
        vkDeviceWaitIdle(device_);
        destroyBuffer(contactBuffer_, contactMemory_, mappedContacts_);
        contactCapacity_ = 0;
        if (!allocateBuffer(needed, contactBuffer_, contactMemory_, mappedContacts_))
            return false;
        contactCapacity_ = needed;
        if (bodyBuffer_) updateDescriptors();
        return true;
    }

    void packBodies(const std::vector<Body>& bodies) {
        GpuBody* mapped = static_cast<GpuBody*>(mappedBodies_);
        for (size_t i = 0; i < bodies.size(); ++i) {
            const Body& b = bodies[i];
            GpuBody& g = mapped[i];
            std::memcpy(g.velocity, &b.velocity, sizeof(float) * 3);
            g.invMass = b.solverInvMass();
            std::memcpy(g.angularVelocity, &b.angularVelocity, sizeof(float) * 3);
            g.gravityScale = b.gravityScale;
            std::memcpy(g.force, &b.force, sizeof(float) * 3);
            g.linearDamping = b.linearDamping;
            std::memcpy(g.torque, &b.torque, sizeof(float) * 3);
            g.angularDamping = b.angularDamping;
            std::memcpy(g.invInertia, &b.invInertia, sizeof(float) * 3);
            uint32_t flags = 0;
            if (b.isDynamic() && !b.isLocked() && !b.asleep) {
                flags |= kFlagIntegrate;
                if (!b.isRotationLocked()) flags |= kFlagTorque;
            }
            if (b.isDynamic()) flags |= kFlagDynamic;
            if (b.isDynamic() && !b.isRotationLocked()) flags |= kFlagSolveAngular;
            if (b.shape == ShapeType::Sphere) flags |= kFlagSphereAnchor;
            if (b.shape == ShapeType::Plane) flags |= kFlagPlane;
            if (b.ccdTuning.enableContinuous &&
                b.ccdTuning.quality != MotionQuality::Low &&
                b.ccdTuning.quality != MotionQuality::Locked)
                flags |= kFlagSweep;
            if (!b.isStatic() && !b.isLocked() && !b.asleep) flags |= kFlagTransformIntegrate;
            if (b.isRotationLocked()) flags |= kFlagRotationLocked;
            flags |= (uint32_t)b.frictionCombine << 8;
            flags |= (uint32_t)b.restitutionCombine << 11;
            std::memcpy(&g.flagsBits, &flags, sizeof(flags));
            std::memcpy(g.orientation, &b.orientation, sizeof(float) * 4);
            std::memcpy(g.inertiaOrientation, &b.inertiaOrientation, sizeof(float) * 4);
            std::memcpy(g.position, &b.position, sizeof(float) * 3);
            g.speculativeDistance = b.ccdTuning.speculativeDistance;
            if (b.shape == ShapeType::Plane) {
                std::memcpy(g.shapeData, &b.planeNormal, sizeof(float) * 3);
                g.shapeData[3] = b.planeOffset;
            } else {
                g.shapeData[0] = b.radius;
                g.shapeData[1] = g.shapeData[2] = g.shapeData[3] = 0.0f;
            }
            g.material[0] = b.restitution;
            g.material[1] = b.friction;
            g.material[2] = b.rollingFriction;
            g.material[3] = b.spinningFriction;
            std::memcpy(g.frictionScale, &b.frictionScale, sizeof(float) * 3);
            g.maxPointSpeed = b.maxPointSpeed();
        }
    }

    void unpackVelocities(std::vector<Body>& bodies) {
        const GpuBody* mapped = static_cast<const GpuBody*>(mappedBodies_);
        for (size_t i = 0; i < bodies.size(); ++i) {
            Body& b = bodies[i];
            if (!b.isDynamic()) continue;
            std::memcpy(&b.velocity, mapped[i].velocity, sizeof(float) * 3);
            std::memcpy(&b.angularVelocity, mapped[i].angularVelocity, sizeof(float) * 3);
        }
    }

    // advanceSubsteps integrates position/orientation on the GPU too (see
    // shaders/integrate_transforms.comp), so its readback needs the full
    // dynamic state, not just velocity.
    void unpackFull(std::vector<Body>& bodies) {
        const GpuBody* mapped = static_cast<const GpuBody*>(mappedBodies_);
        for (size_t i = 0; i < bodies.size(); ++i) {
            Body& b = bodies[i];
            if (b.isStatic()) continue;
            std::memcpy(&b.velocity, mapped[i].velocity, sizeof(float) * 3);
            std::memcpy(&b.angularVelocity, mapped[i].angularVelocity, sizeof(float) * 3);
            std::memcpy(&b.position, mapped[i].position, sizeof(float) * 3);
            std::memcpy(&b.orientation, mapped[i].orientation, sizeof(float) * 4);
        }
    }

    // Deterministic sort + greedy graph coloring, identical to the CUDA
    // backend's host pass: contact arrival order is nondeterministic, and
    // unstable solve order is enough to walk a tall stack off balance.
    // Reorders `contacts` in place (color order) and uploads the colored
    // contacts to the device buffer. Returns false on a capacity failure.
    bool colorContacts(const std::vector<Body>& bodies, std::vector<Contact>& contacts) {
        const uint32_t n = static_cast<uint32_t>(bodies.size());
        const uint32_t m = static_cast<uint32_t>(contacts.size());
        std::sort(contacts.begin(), contacts.end(),
                  [](const Contact& x, const Contact& y) {
                      uint64_t kx = (uint64_t)x.a << 32 | x.b;
                      uint64_t ky = (uint64_t)y.a << 32 | y.b;
                      if (kx != ky) return kx < ky;
                      if (x.point.x != y.point.x) return x.point.x < y.point.x;
                      if (x.point.y != y.point.y) return x.point.y < y.point.y;
                      return x.point.z < y.point.z;
                  });

        colorMask_.assign(n, 0);
        nextColor_.assign(n, 64);
        colorOf_.resize(m);
        numColors_ = 0;
        for (uint32_t k = 0; k < m; ++k) {
            const Contact& c = contacts[k];
            bool sensor = bodies[c.a].isSensor() || bodies[c.b].isSensor();
            bool sa = sensor || !bodies[c.a].isDynamic();
            bool sb = sensor || !bodies[c.b].isDynamic();
            uint64_t used = (sa ? 0 : colorMask_[c.a]) | (sb ? 0 : colorMask_[c.b]);
            int color;
            if (~used) {
                color = 0;
                while (used & (1ull << color)) ++color;
            } else {
                int na = sa ? 64 : nextColor_[c.a];
                int nb = sb ? 64 : nextColor_[c.b];
                color = na > nb ? na : nb;
                if (!sa) nextColor_[c.a] = color + 1;
                if (!sb) nextColor_[c.b] = color + 1;
            }
            if (color < 64) {
                if (!sa) colorMask_[c.a] |= 1ull << color;
                if (!sb) colorMask_[c.b] |= 1ull << color;
            }
            colorOf_[k] = color;
            if (color + 1 > numColors_) numColors_ = color + 1;
        }

        colorStart_.assign(numColors_ + 1, 0);
        for (uint32_t k = 0; k < m; ++k) ++colorStart_[colorOf_[k] + 1];
        for (int c = 0; c < numColors_; ++c) colorStart_[c + 1] += colorStart_[c];
        sorted_.resize(m);
        fill_ = colorStart_;
        for (uint32_t k = 0; k < m; ++k) sorted_[fill_[colorOf_[k]]++] = contacts[k];
        contacts = sorted_;

        GpuContact* mapped = static_cast<GpuContact*>(mappedContacts_);
        for (uint32_t k = 0; k < m; ++k) {
            const Contact& c = contacts[k];
            GpuContact& g = mapped[k];
            std::memcpy(g.normal, &c.normal, sizeof(float) * 3);
            g.bias0 = c.bias0;
            std::memcpy(g.anchorA, &c.localAnchorA, sizeof(float) * 3);
            g.friction1 = c.friction1;
            std::memcpy(g.anchorB, &c.localAnchorB, sizeof(float) * 3);
            g.friction2 = c.friction2;
            g.vn0 = c.vn0;
            g.restitution = c.restitution;
            g.rollingFriction = c.rollingFriction;
            g.spinningFriction = c.spinningFriction;
            g.normalImpulse = c.normalImpulse;
            g.tangentImpulse1 = c.tangentImpulse1;
            g.tangentImpulse2 = c.tangentImpulse2;
            g.spinningImpulse = c.spinningImpulse;
            g.rollingImpulse1 = c.rollingImpulse1;
            g.rollingImpulse2 = c.rollingImpulse2;
            g.a = c.a;
            g.b = c.b;
            g.skip = (bodies[c.a].isSensor() || bodies[c.b].isSensor()) ? 1u : 0u;
            g.pad = 0;
        }
        solveCount_ = m;
        gpuSolveActive_ = true;
        return true;
    }

    // Records one warm-start (mode 0) or solve (mode 1) pass over every
    // color into the ALREADY-BEGUN command buffer. Caller is responsible for
    // beginCommands()/submitCommands(); this only records, never submits, so
    // advanceSubsteps can chain many passes into a single submission.
    void recordSolvePass(uint32_t mode, float dt) {
        vkCmdBindPipeline(commandBuffer_, VK_PIPELINE_BIND_POINT_COMPUTE, solvePipeline_);
        vkCmdBindDescriptorSets(commandBuffer_, VK_PIPELINE_BIND_POINT_COMPUTE,
                                pipelineLayout_, 0, 1, &descriptorSet_, 0, nullptr);
        SolvePush push{};
        push.mode = mode;
        push.dt = dt;
        for (int color = 0; color < numColors_; ++color) {
            push.first = static_cast<uint32_t>(colorStart_[color]);
            push.count = static_cast<uint32_t>(colorStart_[color + 1] - colorStart_[color]);
            if (push.count == 0) continue;
            vkCmdPushConstants(commandBuffer_, pipelineLayout_, VK_SHADER_STAGE_COMPUTE_BIT,
                               0, sizeof(push), &push);
            vkCmdDispatch(commandBuffer_, (push.count + 127) / 128, 1, 1);
            recordBarrier();
        }
    }

    // Records an integrate.comp dispatch (velocity integration) into the
    // already-begun command buffer, for advanceSubsteps' s > 0 substeps.
    void recordIntegrate(const Vec3& gravity, float dt, uint32_t n) {
        vkCmdBindPipeline(commandBuffer_, VK_PIPELINE_BIND_POINT_COMPUTE, integratePipeline_);
        vkCmdBindDescriptorSets(commandBuffer_, VK_PIPELINE_BIND_POINT_COMPUTE,
                                pipelineLayout_, 0, 1, &descriptorSet_, 0, nullptr);
        IntegratePush push{};
        push.gravity[0] = gravity.x;
        push.gravity[1] = gravity.y;
        push.gravity[2] = gravity.z;
        push.dt = dt;
        push.count = n;
        vkCmdPushConstants(commandBuffer_, pipelineLayout_, VK_SHADER_STAGE_COMPUTE_BIT,
                           0, sizeof(push), &push);
        vkCmdDispatch(commandBuffer_, (n + 255) / 256, 1, 1);
        recordBarrier();
    }

    // Records an integrate_transforms.comp dispatch (position/orientation
    // integration, the isotropic path only -- see advanceSubsteps' scope
    // guard) into the already-begun command buffer.
    void recordIntegrateTransforms(uint32_t n, float dt) {
        vkCmdBindPipeline(commandBuffer_, VK_PIPELINE_BIND_POINT_COMPUTE, transformPipeline_);
        vkCmdBindDescriptorSets(commandBuffer_, VK_PIPELINE_BIND_POINT_COMPUTE,
                                pipelineLayout_, 0, 1, &descriptorSet_, 0, nullptr);
        TransformPush push{};
        push.dt = dt;
        push.count = n;
        vkCmdPushConstants(commandBuffer_, pipelineLayout_, VK_SHADER_STAGE_COMPUTE_BIT,
                           0, sizeof(push), &push);
        vkCmdDispatch(commandBuffer_, (n + 255) / 256, 1, 1);
        recordBarrier();
    }

    void recordBarrier() {
        VkMemoryBarrier barrier{VK_STRUCTURE_TYPE_MEMORY_BARRIER};
        barrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT;
        vkCmdPipelineBarrier(commandBuffer_, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                             VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 1, &barrier,
                             0, nullptr, 0, nullptr);
    }

    bool beginCommands() {
        VkCommandBufferBeginInfo begin{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
        begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
        return vkBeginCommandBuffer(commandBuffer_, &begin) == VK_SUCCESS;
    }

    bool submitCommands() {
        if (vkEndCommandBuffer(commandBuffer_) != VK_SUCCESS) return false;
        VkSubmitInfo submit{VK_STRUCTURE_TYPE_SUBMIT_INFO};
        submit.commandBufferCount = 1;
        submit.pCommandBuffers = &commandBuffer_;
        if (vkQueueSubmit(queue_, 1, &submit, fence_) != VK_SUCCESS) return false;
        const VkResult wait = vkWaitForFences(device_, 1, &fence_, VK_TRUE, UINT64_MAX);
        vkResetFences(device_, 1, &fence_);
        return wait == VK_SUCCESS;
    }

    std::unique_ptr<Backend> cpu_;
    VkInstance instance_ = VK_NULL_HANDLE;
    VkPhysicalDevice physicalDevice_ = VK_NULL_HANDLE;
    VkDevice device_ = VK_NULL_HANDLE;
    VkQueue queue_ = VK_NULL_HANDLE;
    uint32_t queueFamily_ = 0;
    VkShaderModule integrateModule_ = VK_NULL_HANDLE;
    VkShaderModule solveModule_ = VK_NULL_HANDLE;
    VkDescriptorSetLayout descriptorLayout_ = VK_NULL_HANDLE;
    VkPipelineLayout pipelineLayout_ = VK_NULL_HANDLE;
    VkPipeline integratePipeline_ = VK_NULL_HANDLE;
    VkPipeline solvePipeline_ = VK_NULL_HANDLE;
    VkDescriptorPool descriptorPool_ = VK_NULL_HANDLE;
    VkDescriptorSet descriptorSet_ = VK_NULL_HANDLE;
    VkCommandPool commandPool_ = VK_NULL_HANDLE;
    VkCommandBuffer commandBuffer_ = VK_NULL_HANDLE;
    VkFence fence_ = VK_NULL_HANDLE;
    VkBuffer bodyBuffer_ = VK_NULL_HANDLE;
    VkDeviceMemory bodyMemory_ = VK_NULL_HANDLE;
    void* mappedBodies_ = nullptr;
    VkDeviceSize bodyCapacity_ = 0;
    VkBuffer contactBuffer_ = VK_NULL_HANDLE;
    VkDeviceMemory contactMemory_ = VK_NULL_HANDLE;
    void* mappedContacts_ = nullptr;
    VkDeviceSize contactCapacity_ = 0;
    VkBuffer pairBuffer_ = VK_NULL_HANDLE;
    VkDeviceMemory pairMemory_ = VK_NULL_HANDLE;
    void* mappedPairs_ = nullptr;
    VkDeviceSize pairCapacity_ = 0;
    VkBuffer counterBuffer_ = VK_NULL_HANDLE;
    VkDeviceMemory counterMemory_ = VK_NULL_HANDLE;
    void* mappedCounter_ = nullptr;
    VkShaderModule narrowModule_ = VK_NULL_HANDLE;
    VkPipeline narrowPipeline_ = VK_NULL_HANDLE;
    VkShaderModule transformModule_ = VK_NULL_HANDLE;
    VkPipeline transformPipeline_ = VK_NULL_HANDLE;

    // Host-side coloring state (mirrors the CUDA backend's members).
    std::vector<uint64_t> colorMask_;
    std::vector<int> nextColor_;
    std::vector<int> colorOf_;
    std::vector<int> colorStart_;
    std::vector<int> fill_;
    std::vector<Contact> sorted_;
    int numColors_ = 0;
    uint32_t solveCount_ = 0;
    bool gpuSolveActive_ = false;
    uint32_t lastVelocityIterations_ = 0;
    bool gpuNarrowEnabled_ = std::getenv("VELOX_VULKAN_GPU_NARROW") != nullptr;
};

} // namespace

Backend* createVulkanBackend() { return VulkanBackend::create(); }

} // namespace velox
