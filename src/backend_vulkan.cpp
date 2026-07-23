// Vulkan compute backend: cross-vendor GPU acceleration (NVIDIA, AMD, Intel —
// any driver exposing a Vulkan 1.1 compute queue), complementing the
// NVIDIA-only CUDA backend.
//
// Stage 1 scope, deliberate: velocity integration runs on the GPU through the
// integrate.comp shader; broad phase, narrow phase, and the contact solver
// delegate to an owned CPU backend, so simulation results stay correct by
// construction while the Vulkan infrastructure (device selection, buffer
// management, pipeline setup, dispatch/readback) is real and tested. Later
// stages can move the graph-colored contact solve on-device the way the CUDA
// backend does, without touching the World-facing interface again.
//
// Like the CUDA backend, this backend is NOT part of the strict bitwise
// cross-platform determinism guarantee: GPU float contraction differs from
// the CPU reference path. Use BackendType::Cpu for lockstep replay.
#include "velox/backend.h"

#include <vulkan/vulkan.h>

#include "integrate_spv.h"

#include <cstdio>
#include <cstring>
#include <memory>
#include <vector>

namespace velox {

namespace {

// Must match the GpuBody struct in shaders/integrate.comp (std430: seven
// vec4s, 112 bytes, no padding surprises since every member is a vec4).
struct GpuBody {
    float velocity[3];        float invMass;
    float angularVelocity[3]; float gravityScale;
    float force[3];           float linearDamping;
    float torque[3];          float angularDamping;
    float invInertia[3];      float flagsBits;
    float orientation[4];
    float inertiaOrientation[4];
};
static_assert(sizeof(GpuBody) == 112, "GpuBody must match the std430 layout");

struct PushConstants {
    float gravity[3];
    float dt;
    uint32_t count;
};

constexpr uint32_t kFlagIntegrate = 1u;
constexpr uint32_t kFlagTorque = 2u;

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
            destroyBodyBuffer();
            if (fence_) vkDestroyFence(device_, fence_, nullptr);
            if (commandPool_) vkDestroyCommandPool(device_, commandPool_, nullptr);
            if (descriptorPool_) vkDestroyDescriptorPool(device_, descriptorPool_, nullptr);
            if (pipeline_) vkDestroyPipeline(device_, pipeline_, nullptr);
            if (pipelineLayout_) vkDestroyPipelineLayout(device_, pipelineLayout_, nullptr);
            if (descriptorLayout_) vkDestroyDescriptorSetLayout(device_, descriptorLayout_, nullptr);
            if (shaderModule_) vkDestroyShaderModule(device_, shaderModule_, nullptr);
            vkDestroyDevice(device_, nullptr);
        }
        if (instance_) vkDestroyInstance(instance_, nullptr);
    }

    const char* name() const override { return "vulkan"; }

    void integrate(std::vector<Body>& bodies, const Vec3& gravity, float dt) override {
        const uint32_t n = static_cast<uint32_t>(bodies.size());
        if (n == 0) return;
        if (!ensureBodyCapacity(n)) {
            // Allocation failed (device memory pressure): this frame still
            // has to integrate, and the CPU path is bit-for-bit the reference.
            cpu_->integrate(bodies, gravity, dt);
            return;
        }

        GpuBody* mapped = static_cast<GpuBody*>(mappedBodies_);
        for (uint32_t i = 0; i < n; ++i) {
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
            std::memcpy(&g.flagsBits, &flags, sizeof(flags));
            std::memcpy(g.orientation, &b.orientation, sizeof(float) * 4);
            std::memcpy(g.inertiaOrientation, &b.inertiaOrientation, sizeof(float) * 4);
        }

        PushConstants push{};
        push.gravity[0] = gravity.x;
        push.gravity[1] = gravity.y;
        push.gravity[2] = gravity.z;
        push.dt = dt;
        push.count = n;
        if (!dispatch(push, n)) {
            cpu_->integrate(bodies, gravity, dt);
            return;
        }

        for (uint32_t i = 0; i < n; ++i) {
            Body& b = bodies[i];
            if (!b.isDynamic() || b.isLocked() || b.asleep) continue;
            std::memcpy(&b.velocity, mapped[i].velocity, sizeof(float) * 3);
            std::memcpy(&b.angularVelocity, mapped[i].angularVelocity, sizeof(float) * 3);
        }
    }

    // Broad phase, narrow phase, and the contact/joint solve run on the CPU
    // reference path in stage 1 (see file header). Delegation keeps the
    // World-facing behavior identical to BackendType::Cpu for these phases.
    bool wantsHostPairs() const override { return cpu_->wantsHostPairs(); }
    void findContacts(const std::vector<Body>& bodies, const MeshSoup& meshes,
                      float dt, const std::vector<uint64_t>* hostPairs,
                      std::vector<Contact>& out) override {
        cpu_->findContacts(bodies, meshes, dt, hostPairs, out);
    }
    void solveVelocities(std::vector<Body>& bodies, std::vector<Contact>& contacts,
                         float dt, bool warmStart, const SolverOptions& options) override {
        cpu_->solveVelocities(bodies, contacts, dt, warmStart, options);
    }
    void fetchImpulses(std::vector<Contact>& contacts) override {
        cpu_->fetchImpulses(contacts);
    }
    void invalidateCaches() override { cpu_->invalidateCaches(); }
    void setWorkerCount(uint32_t count) override { cpu_->setWorkerCount(count); }
    uint32_t workerCount() const override { return cpu_->workerCount(); }
    void setParallelIslands(bool enabled) override { cpu_->setParallelIslands(enabled); }
    void setTaskSystem(TaskSystem* system) override { cpu_->setTaskSystem(system); }
    uint32_t lastVelocityIterations() const override { return cpu_->lastVelocityIterations(); }
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

        // Prefer a discrete GPU; accept any device with a compute queue.
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

        // Shader module from the SPIR-V bytes embedded at build time.
        std::vector<uint32_t> code(velox_integrate_spv_size / sizeof(uint32_t));
        std::memcpy(code.data(), velox_integrate_spv, velox_integrate_spv_size);
        VkShaderModuleCreateInfo shaderInfo{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
        shaderInfo.codeSize = velox_integrate_spv_size;
        shaderInfo.pCode = code.data();
        if (vkCreateShaderModule(device_, &shaderInfo, nullptr, &shaderModule_) != VK_SUCCESS)
            return false;

        VkDescriptorSetLayoutBinding binding{};
        binding.binding = 0;
        binding.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        binding.descriptorCount = 1;
        binding.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
        VkDescriptorSetLayoutCreateInfo layoutInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
        layoutInfo.bindingCount = 1;
        layoutInfo.pBindings = &binding;
        if (vkCreateDescriptorSetLayout(device_, &layoutInfo, nullptr, &descriptorLayout_) != VK_SUCCESS)
            return false;

        VkPushConstantRange pushRange{};
        pushRange.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
        pushRange.size = sizeof(PushConstants);
        VkPipelineLayoutCreateInfo pipelineLayoutInfo{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
        pipelineLayoutInfo.setLayoutCount = 1;
        pipelineLayoutInfo.pSetLayouts = &descriptorLayout_;
        pipelineLayoutInfo.pushConstantRangeCount = 1;
        pipelineLayoutInfo.pPushConstantRanges = &pushRange;
        if (vkCreatePipelineLayout(device_, &pipelineLayoutInfo, nullptr, &pipelineLayout_) != VK_SUCCESS)
            return false;

        VkComputePipelineCreateInfo pipelineInfo{VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};
        pipelineInfo.stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        pipelineInfo.stage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
        pipelineInfo.stage.module = shaderModule_;
        pipelineInfo.stage.pName = "main";
        pipelineInfo.layout = pipelineLayout_;
        if (vkCreateComputePipelines(device_, VK_NULL_HANDLE, 1, &pipelineInfo,
                                     nullptr, &pipeline_) != VK_SUCCESS)
            return false;

        VkDescriptorPoolSize poolSize{VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1};
        VkDescriptorPoolCreateInfo poolInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
        poolInfo.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
        poolInfo.maxSets = 1;
        poolInfo.poolSizeCount = 1;
        poolInfo.pPoolSizes = &poolSize;
        if (vkCreateDescriptorPool(device_, &poolInfo, nullptr, &descriptorPool_) != VK_SUCCESS)
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
        return vkCreateFence(device_, &fenceInfo, nullptr, &fence_) == VK_SUCCESS;
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

    void destroyBodyBuffer() {
        if (mappedBodies_) {
            vkUnmapMemory(device_, bodyMemory_);
            mappedBodies_ = nullptr;
        }
        if (bodyBuffer_) {
            vkDestroyBuffer(device_, bodyBuffer_, nullptr);
            bodyBuffer_ = VK_NULL_HANDLE;
        }
        if (bodyMemory_) {
            vkFreeMemory(device_, bodyMemory_, nullptr);
            bodyMemory_ = VK_NULL_HANDLE;
        }
        if (descriptorSet_) {
            vkFreeDescriptorSets(device_, descriptorPool_, 1, &descriptorSet_);
            descriptorSet_ = VK_NULL_HANDLE;
        }
        bodyCapacity_ = 0;
    }

    bool ensureBodyCapacity(uint32_t count) {
        const VkDeviceSize needed = VkDeviceSize(count) * sizeof(GpuBody);
        if (needed <= bodyCapacity_ && bodyBuffer_) return true;
        vkDeviceWaitIdle(device_);
        destroyBodyBuffer();

        VkBufferCreateInfo bufferInfo{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
        bufferInfo.size = needed;
        bufferInfo.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
        bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        if (vkCreateBuffer(device_, &bufferInfo, nullptr, &bodyBuffer_) != VK_SUCCESS)
            return false;

        VkMemoryRequirements requirements;
        vkGetBufferMemoryRequirements(device_, bodyBuffer_, &requirements);
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
        if (vkAllocateMemory(device_, &allocInfo, nullptr, &bodyMemory_) != VK_SUCCESS)
            return false;
        if (vkBindBufferMemory(device_, bodyBuffer_, bodyMemory_, 0) != VK_SUCCESS)
            return false;
        if (vkMapMemory(device_, bodyMemory_, 0, VK_WHOLE_SIZE, 0, &mappedBodies_) != VK_SUCCESS)
            return false;

        VkDescriptorSetAllocateInfo setInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        setInfo.descriptorPool = descriptorPool_;
        setInfo.descriptorSetCount = 1;
        setInfo.pSetLayouts = &descriptorLayout_;
        if (vkAllocateDescriptorSets(device_, &setInfo, &descriptorSet_) != VK_SUCCESS)
            return false;
        VkDescriptorBufferInfo bufferDescriptor{bodyBuffer_, 0, VK_WHOLE_SIZE};
        VkWriteDescriptorSet write{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
        write.dstSet = descriptorSet_;
        write.dstBinding = 0;
        write.descriptorCount = 1;
        write.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        write.pBufferInfo = &bufferDescriptor;
        vkUpdateDescriptorSets(device_, 1, &write, 0, nullptr);

        bodyCapacity_ = needed;
        return true;
    }

    bool dispatch(const PushConstants& push, uint32_t count) {
        VkCommandBufferBeginInfo begin{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
        begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
        if (vkBeginCommandBuffer(commandBuffer_, &begin) != VK_SUCCESS) return false;
        vkCmdBindPipeline(commandBuffer_, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline_);
        vkCmdBindDescriptorSets(commandBuffer_, VK_PIPELINE_BIND_POINT_COMPUTE,
                                pipelineLayout_, 0, 1, &descriptorSet_, 0, nullptr);
        vkCmdPushConstants(commandBuffer_, pipelineLayout_, VK_SHADER_STAGE_COMPUTE_BIT,
                           0, sizeof(PushConstants), &push);
        vkCmdDispatch(commandBuffer_, (count + 255) / 256, 1, 1);
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
    VkShaderModule shaderModule_ = VK_NULL_HANDLE;
    VkDescriptorSetLayout descriptorLayout_ = VK_NULL_HANDLE;
    VkPipelineLayout pipelineLayout_ = VK_NULL_HANDLE;
    VkPipeline pipeline_ = VK_NULL_HANDLE;
    VkDescriptorPool descriptorPool_ = VK_NULL_HANDLE;
    VkDescriptorSet descriptorSet_ = VK_NULL_HANDLE;
    VkCommandPool commandPool_ = VK_NULL_HANDLE;
    VkCommandBuffer commandBuffer_ = VK_NULL_HANDLE;
    VkFence fence_ = VK_NULL_HANDLE;
    VkBuffer bodyBuffer_ = VK_NULL_HANDLE;
    VkDeviceMemory bodyMemory_ = VK_NULL_HANDLE;
    void* mappedBodies_ = nullptr;
    VkDeviceSize bodyCapacity_ = 0;
};

} // namespace

Backend* createVulkanBackend() { return VulkanBackend::create(); }

} // namespace velox
