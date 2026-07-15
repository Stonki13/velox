#include "renderer.h"
#include "vk_loader.h"
#include "window.h"

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#endif

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <algorithm>
#include <cstring>
#include <stdexcept>
#include <string>

#include "shaders/sky_vert.h"
#include "shaders/sky_frag.h"
#include "shaders/mesh_vert.h"
#include "shaders/mesh_frag.h"
#include "shaders/line_vert.h"
#include "shaders/line_frag.h"
#include "shaders/ui_vert.h"
#include "shaders/ui_frag.h"

#ifndef _WIN32
#include <dlfcn.h>
#endif

namespace sandbox {

// --- dynamic loader ----------------------------------------------------------

bool VkFunctions::loadLoader() {
#ifdef _WIN32
    HMODULE module = LoadLibraryA("vulkan-1.dll");
    if (!module) return false;
    vkGetInstanceProcAddr = reinterpret_cast<PFN_vkGetInstanceProcAddr>(
        reinterpret_cast<void*>(GetProcAddress(module, "vkGetInstanceProcAddr")));
#else
    void* module = dlopen("libvulkan.so.1", RTLD_NOW | RTLD_LOCAL);
    if (!module) module = dlopen("libvulkan.so", RTLD_NOW | RTLD_LOCAL);
    if (!module) return false;
    vkGetInstanceProcAddr = reinterpret_cast<PFN_vkGetInstanceProcAddr>(
        dlsym(module, "vkGetInstanceProcAddr"));
#endif
    if (!vkGetInstanceProcAddr) return false;
    vkCreateInstance = reinterpret_cast<PFN_vkCreateInstance>(
        vkGetInstanceProcAddr(nullptr, "vkCreateInstance"));
    vkEnumerateInstanceVersion = reinterpret_cast<PFN_vkEnumerateInstanceVersion>(
        vkGetInstanceProcAddr(nullptr, "vkEnumerateInstanceVersion"));
    return vkCreateInstance != nullptr;
}

void VkFunctions::loadInstance(VkInstance instance) {
#define SANDBOX_VK_LOAD(name) \
    name = reinterpret_cast<PFN_##name>(vkGetInstanceProcAddr(instance, #name));
    SANDBOX_VK_INSTANCE_FUNCS(SANDBOX_VK_LOAD)
    // Device functions resolve through the instance too; loadDevice sharpens
    // them to direct dispatch afterwards.
    SANDBOX_VK_DEVICE_FUNCS(SANDBOX_VK_LOAD)
#undef SANDBOX_VK_LOAD
}

void VkFunctions::loadDevice(VkDevice device) {
#define SANDBOX_VK_LOAD_DEVICE(name) \
    if (auto proc = vkGetDeviceProcAddr(device, #name)) \
        name = reinterpret_cast<PFN_##name>(proc);
    SANDBOX_VK_DEVICE_FUNCS(SANDBOX_VK_LOAD_DEVICE)
#undef SANDBOX_VK_LOAD_DEVICE
}

namespace {

VkFunctions vk;

void check(VkResult result, const char* what) {
    if (result != VK_SUCCESS)
        throw std::runtime_error(std::string("Vulkan error in ") + what +
                                 " (VkResult " + std::to_string(int(result)) + ")");
}

struct Buffer {
    VkBuffer buffer = VK_NULL_HANDLE;
    VkDeviceMemory memory = VK_NULL_HANDLE;
    void* mapped = nullptr;
    VkDeviceSize size = 0;
};

struct GpuMesh {
    Buffer vertices;
    Buffer indices;
    uint32_t indexCount = 0;
};

} // namespace

// --- renderer implementation -------------------------------------------------

struct Renderer::Impl {
    Window* window = nullptr;
    VkInstance instance = VK_NULL_HANDLE;
    VkSurfaceKHR surface = VK_NULL_HANDLE;
    VkPhysicalDevice physical = VK_NULL_HANDLE;
    VkPhysicalDeviceMemoryProperties memoryProperties{};
    uint32_t queueFamily = 0;
    VkDevice device = VK_NULL_HANDLE;
    VkQueue queue = VK_NULL_HANDLE;

    VkSwapchainKHR swapchain = VK_NULL_HANDLE;
    VkFormat swapchainFormat = VK_FORMAT_B8G8R8A8_UNORM;
    VkExtent2D extent{};
    std::vector<VkImage> swapchainImages;
    std::vector<VkImageView> swapchainViews;
    std::vector<VkFramebuffer> framebuffers;
    VkSampleCountFlagBits samples = VK_SAMPLE_COUNT_4_BIT;
    VkImage colorImage = VK_NULL_HANDLE;
    VkDeviceMemory colorMemory = VK_NULL_HANDLE;
    VkImageView colorView = VK_NULL_HANDLE;
    VkImage depthImage = VK_NULL_HANDLE;
    VkDeviceMemory depthMemory = VK_NULL_HANDLE;
    VkImageView depthView = VK_NULL_HANDLE;
    VkRenderPass renderPass = VK_NULL_HANDLE;

    VkDescriptorSetLayout setLayout = VK_NULL_HANDLE;
    VkDescriptorPool descriptorPool = VK_NULL_HANDLE;
    VkDescriptorSet descriptorSet = VK_NULL_HANDLE;
    VkPipelineLayout pipelineLayout = VK_NULL_HANDLE;
    VkPipeline meshPipeline = VK_NULL_HANDLE;
    VkPipeline skyPipeline = VK_NULL_HANDLE;
    VkPipeline linePipeline = VK_NULL_HANDLE;
    VkPipeline uiPipeline = VK_NULL_HANDLE;

    VkCommandPool commandPool = VK_NULL_HANDLE;
    VkCommandBuffer commandBuffer = VK_NULL_HANDLE;
    VkSemaphore imageAvailable = VK_NULL_HANDLE;
    VkSemaphore renderFinished = VK_NULL_HANDLE;
    VkFence inFlight = VK_NULL_HANDLE;

    Buffer uniform;
    Buffer instanceBuffer;
    Buffer lineBuffer;
    Buffer uiBuffer;
    std::vector<GpuMesh> meshes;

    // --- helpers -------------------------------------------------------------

    uint32_t findMemoryType(uint32_t typeBits, VkMemoryPropertyFlags properties) const {
        for (uint32_t i = 0; i < memoryProperties.memoryTypeCount; ++i)
            if ((typeBits & (1u << i)) &&
                (memoryProperties.memoryTypes[i].propertyFlags & properties) == properties)
                return i;
        throw std::runtime_error("Vulkan: no suitable memory type");
    }

    Buffer createBuffer(VkDeviceSize size, VkBufferUsageFlags usage) const {
        Buffer result;
        result.size = size;
        VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
        info.size = size;
        info.usage = usage;
        info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        check(vk.vkCreateBuffer(device, &info, nullptr, &result.buffer), "vkCreateBuffer");
        VkMemoryRequirements requirements;
        vk.vkGetBufferMemoryRequirements(device, result.buffer, &requirements);
        VkMemoryAllocateInfo alloc{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
        alloc.allocationSize = requirements.size;
        alloc.memoryTypeIndex = findMemoryType(
            requirements.memoryTypeBits,
            VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        check(vk.vkAllocateMemory(device, &alloc, nullptr, &result.memory), "vkAllocateMemory");
        check(vk.vkBindBufferMemory(device, result.buffer, result.memory, 0), "vkBindBufferMemory");
        check(vk.vkMapMemory(device, result.memory, 0, VK_WHOLE_SIZE, 0, &result.mapped),
              "vkMapMemory");
        return result;
    }

    void destroyBuffer(Buffer& buffer) const {
        if (buffer.mapped) vk.vkUnmapMemory(device, buffer.memory);
        if (buffer.buffer) vk.vkDestroyBuffer(device, buffer.buffer, nullptr);
        if (buffer.memory) vk.vkFreeMemory(device, buffer.memory, nullptr);
        buffer = {};
    }

    // Grows a dynamic buffer; safe because a single frame is in flight and the
    // caller waits on the frame fence before uploading.
    void ensureCapacity(Buffer& buffer, VkDeviceSize needed, VkBufferUsageFlags usage) {
        if (buffer.size >= needed && buffer.buffer) return;
        vk.vkDeviceWaitIdle(device);
        destroyBuffer(buffer);
        VkDeviceSize size = 65536;
        while (size < needed) size *= 2;
        buffer = createBuffer(size, usage);
    }

    VkShaderModule createShader(const uint32_t* words, size_t bytes) const {
        VkShaderModuleCreateInfo info{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
        info.codeSize = bytes;
        info.pCode = words;
        VkShaderModule module;
        check(vk.vkCreateShaderModule(device, &info, nullptr, &module), "vkCreateShaderModule");
        return module;
    }

    // --- initialization ------------------------------------------------------

    void createInstanceAndDevice() {
        if (!vk.loadLoader())
            throw std::runtime_error("Vulkan loader not found (vulkan-1.dll). "
                                     "Install a GPU driver with Vulkan support.");

        uint32_t extensionCount = 0;
        const char** extensions = glfwGetRequiredInstanceExtensions(&extensionCount);
        if (!extensions)
            throw std::runtime_error("GLFW reports no Vulkan surface extensions");

        VkApplicationInfo app{VK_STRUCTURE_TYPE_APPLICATION_INFO};
        app.pApplicationName = "velox_sandbox";
        app.apiVersion = VK_API_VERSION_1_2;
        VkInstanceCreateInfo info{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
        info.pApplicationInfo = &app;
        info.enabledExtensionCount = extensionCount;
        info.ppEnabledExtensionNames = extensions;
        check(vk.vkCreateInstance(&info, nullptr, &instance), "vkCreateInstance");
        vk.loadInstance(instance);

        check(glfwCreateWindowSurface(instance, window->native(), nullptr, &surface),
              "glfwCreateWindowSurface");

        uint32_t deviceCount = 0;
        check(vk.vkEnumeratePhysicalDevices(instance, &deviceCount, nullptr),
              "vkEnumeratePhysicalDevices");
        if (deviceCount == 0) throw std::runtime_error("Vulkan: no physical devices");
        std::vector<VkPhysicalDevice> devices(deviceCount);
        check(vk.vkEnumeratePhysicalDevices(instance, &deviceCount, devices.data()),
              "vkEnumeratePhysicalDevices");

        // Prefer a discrete GPU whose queue family supports graphics + present.
        int bestScore = -1;
        for (VkPhysicalDevice candidate : devices) {
            uint32_t familyCount = 0;
            vk.vkGetPhysicalDeviceQueueFamilyProperties(candidate, &familyCount, nullptr);
            std::vector<VkQueueFamilyProperties> families(familyCount);
            vk.vkGetPhysicalDeviceQueueFamilyProperties(candidate, &familyCount, families.data());
            for (uint32_t family = 0; family < familyCount; ++family) {
                if (!(families[family].queueFlags & VK_QUEUE_GRAPHICS_BIT)) continue;
                VkBool32 present = VK_FALSE;
                vk.vkGetPhysicalDeviceSurfaceSupportKHR(candidate, family, surface, &present);
                if (!present) continue;
                VkPhysicalDeviceProperties properties;
                vk.vkGetPhysicalDeviceProperties(candidate, &properties);
                const int score =
                    properties.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU ? 2 : 1;
                if (score > bestScore) {
                    bestScore = score;
                    physical = candidate;
                    queueFamily = family;
                }
                break;
            }
        }
        if (!physical) throw std::runtime_error("Vulkan: no graphics+present capable GPU");
        vk.vkGetPhysicalDeviceMemoryProperties(physical, &memoryProperties);

        VkPhysicalDeviceProperties properties;
        vk.vkGetPhysicalDeviceProperties(physical, &properties);
        const VkSampleCountFlags counts =
            properties.limits.framebufferColorSampleCounts &
            properties.limits.framebufferDepthSampleCounts;
        samples = (counts & VK_SAMPLE_COUNT_4_BIT) ? VK_SAMPLE_COUNT_4_BIT
                                                   : VK_SAMPLE_COUNT_1_BIT;

        const float priority = 1.0f;
        VkDeviceQueueCreateInfo queueInfo{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
        queueInfo.queueFamilyIndex = queueFamily;
        queueInfo.queueCount = 1;
        queueInfo.pQueuePriorities = &priority;
        const char* deviceExtensions[] = {VK_KHR_SWAPCHAIN_EXTENSION_NAME};
        VkDeviceCreateInfo deviceInfo{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
        deviceInfo.queueCreateInfoCount = 1;
        deviceInfo.pQueueCreateInfos = &queueInfo;
        deviceInfo.enabledExtensionCount = 1;
        deviceInfo.ppEnabledExtensionNames = deviceExtensions;
        check(vk.vkCreateDevice(physical, &deviceInfo, nullptr, &device), "vkCreateDevice");
        vk.loadDevice(device);
        vk.vkGetDeviceQueue(device, queueFamily, 0, &queue);
    }

    void createImageResource(VkFormat format, VkImageUsageFlags usage,
                             VkImageAspectFlags aspect, VkImage& image,
                             VkDeviceMemory& memory, VkImageView& view) const {
        VkImageCreateInfo info{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
        info.imageType = VK_IMAGE_TYPE_2D;
        info.format = format;
        info.extent = {extent.width, extent.height, 1};
        info.mipLevels = 1;
        info.arrayLayers = 1;
        info.samples = samples;
        info.tiling = VK_IMAGE_TILING_OPTIMAL;
        info.usage = usage;
        info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        check(vk.vkCreateImage(device, &info, nullptr, &image), "vkCreateImage");
        VkMemoryRequirements requirements;
        vk.vkGetImageMemoryRequirements(device, image, &requirements);
        VkMemoryAllocateInfo alloc{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
        alloc.allocationSize = requirements.size;
        alloc.memoryTypeIndex = findMemoryType(requirements.memoryTypeBits,
                                               VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        check(vk.vkAllocateMemory(device, &alloc, nullptr, &memory), "vkAllocateMemory");
        check(vk.vkBindImageMemory(device, image, memory, 0), "vkBindImageMemory");
        VkImageViewCreateInfo viewInfo{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
        viewInfo.image = image;
        viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
        viewInfo.format = format;
        viewInfo.subresourceRange = {aspect, 0, 1, 0, 1};
        check(vk.vkCreateImageView(device, &viewInfo, nullptr, &view), "vkCreateImageView");
    }

    void createSwapchain() {
        VkSurfaceCapabilitiesKHR capabilities;
        check(vk.vkGetPhysicalDeviceSurfaceCapabilitiesKHR(physical, surface, &capabilities),
              "surface capabilities");
        extent = capabilities.currentExtent;
        if (extent.width == UINT32_MAX) {
            extent.width = static_cast<uint32_t>(window->width());
            extent.height = static_cast<uint32_t>(window->height());
        }
        extent.width = std::max(1u, extent.width);
        extent.height = std::max(1u, extent.height);

        uint32_t formatCount = 0;
        vk.vkGetPhysicalDeviceSurfaceFormatsKHR(physical, surface, &formatCount, nullptr);
        std::vector<VkSurfaceFormatKHR> formats(formatCount);
        vk.vkGetPhysicalDeviceSurfaceFormatsKHR(physical, surface, &formatCount, formats.data());
        VkSurfaceFormatKHR chosen = formats[0];
        for (const auto& format : formats)
            if (format.format == VK_FORMAT_B8G8R8A8_UNORM) { chosen = format; break; }
        swapchainFormat = chosen.format;

        uint32_t imageCount = capabilities.minImageCount + 1;
        if (capabilities.maxImageCount > 0)
            imageCount = std::min(imageCount, capabilities.maxImageCount);

        VkSwapchainCreateInfoKHR info{VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR};
        info.surface = surface;
        info.minImageCount = imageCount;
        info.imageFormat = chosen.format;
        info.imageColorSpace = chosen.colorSpace;
        info.imageExtent = extent;
        info.imageArrayLayers = 1;
        info.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
        info.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
        info.preTransform = capabilities.currentTransform;
        info.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
        info.presentMode = VK_PRESENT_MODE_FIFO_KHR; // vsync, always available
        info.clipped = VK_TRUE;
        check(vk.vkCreateSwapchainKHR(device, &info, nullptr, &swapchain),
              "vkCreateSwapchainKHR");

        uint32_t count = 0;
        vk.vkGetSwapchainImagesKHR(device, swapchain, &count, nullptr);
        swapchainImages.resize(count);
        vk.vkGetSwapchainImagesKHR(device, swapchain, &count, swapchainImages.data());
        swapchainViews.resize(count);
        for (uint32_t i = 0; i < count; ++i) {
            VkImageViewCreateInfo viewInfo{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
            viewInfo.image = swapchainImages[i];
            viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
            viewInfo.format = swapchainFormat;
            viewInfo.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
            check(vk.vkCreateImageView(device, &viewInfo, nullptr, &swapchainViews[i]),
                  "swapchain image view");
        }

        if (samples != VK_SAMPLE_COUNT_1_BIT)
            createImageResource(swapchainFormat,
                                VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT |
                                    VK_IMAGE_USAGE_TRANSIENT_ATTACHMENT_BIT,
                                VK_IMAGE_ASPECT_COLOR_BIT, colorImage, colorMemory, colorView);
        createImageResource(VK_FORMAT_D32_SFLOAT,
                            VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT,
                            VK_IMAGE_ASPECT_DEPTH_BIT, depthImage, depthMemory, depthView);

        framebuffers.resize(count);
        for (uint32_t i = 0; i < count; ++i) {
            VkImageView attachments1x[] = {swapchainViews[i], depthView};
            VkImageView attachmentsMs[] = {colorView, depthView, swapchainViews[i]};
            VkFramebufferCreateInfo fbInfo{VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO};
            fbInfo.renderPass = renderPass;
            fbInfo.attachmentCount = samples == VK_SAMPLE_COUNT_1_BIT ? 2u : 3u;
            fbInfo.pAttachments = samples == VK_SAMPLE_COUNT_1_BIT ? attachments1x
                                                                   : attachmentsMs;
            fbInfo.width = extent.width;
            fbInfo.height = extent.height;
            fbInfo.layers = 1;
            check(vk.vkCreateFramebuffer(device, &fbInfo, nullptr, &framebuffers[i]),
                  "vkCreateFramebuffer");
        }
    }

    void destroySwapchain() {
        for (VkFramebuffer framebuffer : framebuffers)
            vk.vkDestroyFramebuffer(device, framebuffer, nullptr);
        framebuffers.clear();
        if (depthView) vk.vkDestroyImageView(device, depthView, nullptr);
        if (depthImage) vk.vkDestroyImage(device, depthImage, nullptr);
        if (depthMemory) vk.vkFreeMemory(device, depthMemory, nullptr);
        depthView = VK_NULL_HANDLE; depthImage = VK_NULL_HANDLE; depthMemory = VK_NULL_HANDLE;
        if (colorView) vk.vkDestroyImageView(device, colorView, nullptr);
        if (colorImage) vk.vkDestroyImage(device, colorImage, nullptr);
        if (colorMemory) vk.vkFreeMemory(device, colorMemory, nullptr);
        colorView = VK_NULL_HANDLE; colorImage = VK_NULL_HANDLE; colorMemory = VK_NULL_HANDLE;
        for (VkImageView view : swapchainViews)
            vk.vkDestroyImageView(device, view, nullptr);
        swapchainViews.clear();
        if (swapchain) vk.vkDestroySwapchainKHR(device, swapchain, nullptr);
        swapchain = VK_NULL_HANDLE;
    }

    void recreateSwapchain() {
        vk.vkDeviceWaitIdle(device);
        destroySwapchain();
        createSwapchain();
    }

    void createRenderPass() {
        const bool multisampled = samples != VK_SAMPLE_COUNT_1_BIT;
        VkAttachmentDescription attachments[3]{};
        // 0: color (MSAA target, or the swapchain image directly at 1x)
        attachments[0].format = swapchainFormat;
        attachments[0].samples = samples;
        attachments[0].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        attachments[0].storeOp = multisampled ? VK_ATTACHMENT_STORE_OP_DONT_CARE
                                              : VK_ATTACHMENT_STORE_OP_STORE;
        attachments[0].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        attachments[0].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        attachments[0].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        attachments[0].finalLayout = multisampled
            ? VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL
            : VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
        // 1: depth
        attachments[1].format = VK_FORMAT_D32_SFLOAT;
        attachments[1].samples = samples;
        attachments[1].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        attachments[1].storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        attachments[1].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        attachments[1].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        attachments[1].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        attachments[1].finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
        // 2: resolve target (swapchain image) when multisampled
        attachments[2].format = swapchainFormat;
        attachments[2].samples = VK_SAMPLE_COUNT_1_BIT;
        attachments[2].loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        attachments[2].storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        attachments[2].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        attachments[2].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        attachments[2].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        attachments[2].finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

        VkAttachmentReference colorRef{0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
        VkAttachmentReference depthRef{1, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL};
        VkAttachmentReference resolveRef{2, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};

        VkSubpassDescription subpass{};
        subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
        subpass.colorAttachmentCount = 1;
        subpass.pColorAttachments = &colorRef;
        subpass.pDepthStencilAttachment = &depthRef;
        subpass.pResolveAttachments = multisampled ? &resolveRef : nullptr;

        VkSubpassDependency dependency{};
        dependency.srcSubpass = VK_SUBPASS_EXTERNAL;
        dependency.dstSubpass = 0;
        dependency.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT |
                                  VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
        dependency.srcAccessMask = 0;
        dependency.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT |
                                  VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
        dependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT |
                                   VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

        VkRenderPassCreateInfo info{VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO};
        info.attachmentCount = multisampled ? 3u : 2u;
        info.pAttachments = attachments;
        info.subpassCount = 1;
        info.pSubpasses = &subpass;
        info.dependencyCount = 1;
        info.pDependencies = &dependency;
        check(vk.vkCreateRenderPass(device, &info, nullptr, &renderPass), "vkCreateRenderPass");
    }

    void createDescriptors() {
        VkDescriptorSetLayoutBinding binding{};
        binding.binding = 0;
        binding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        binding.descriptorCount = 1;
        binding.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
        VkDescriptorSetLayoutCreateInfo layoutInfo{
            VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
        layoutInfo.bindingCount = 1;
        layoutInfo.pBindings = &binding;
        check(vk.vkCreateDescriptorSetLayout(device, &layoutInfo, nullptr, &setLayout),
              "vkCreateDescriptorSetLayout");

        VkDescriptorPoolSize poolSize{VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1};
        VkDescriptorPoolCreateInfo poolInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
        poolInfo.maxSets = 1;
        poolInfo.poolSizeCount = 1;
        poolInfo.pPoolSizes = &poolSize;
        check(vk.vkCreateDescriptorPool(device, &poolInfo, nullptr, &descriptorPool),
              "vkCreateDescriptorPool");

        VkDescriptorSetAllocateInfo alloc{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        alloc.descriptorPool = descriptorPool;
        alloc.descriptorSetCount = 1;
        alloc.pSetLayouts = &setLayout;
        check(vk.vkAllocateDescriptorSets(device, &alloc, &descriptorSet),
              "vkAllocateDescriptorSets");

        uniform = createBuffer(sizeof(FrameGlobals), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT);
        VkDescriptorBufferInfo bufferInfo{uniform.buffer, 0, sizeof(FrameGlobals)};
        VkWriteDescriptorSet write{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
        write.dstSet = descriptorSet;
        write.dstBinding = 0;
        write.descriptorCount = 1;
        write.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        write.pBufferInfo = &bufferInfo;
        vk.vkUpdateDescriptorSets(device, 1, &write, 0, nullptr);

        VkPipelineLayoutCreateInfo pipelineLayoutInfo{
            VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
        pipelineLayoutInfo.setLayoutCount = 1;
        pipelineLayoutInfo.pSetLayouts = &setLayout;
        check(vk.vkCreatePipelineLayout(device, &pipelineLayoutInfo, nullptr, &pipelineLayout),
              "vkCreatePipelineLayout");
    }

    VkPipeline createPipeline(const uint32_t* vertWords, size_t vertBytes,
                              const uint32_t* fragWords, size_t fragBytes,
                              VkPrimitiveTopology topology,
                              const VkVertexInputBindingDescription* bindings,
                              uint32_t bindingCount,
                              const VkVertexInputAttributeDescription* attributes,
                              uint32_t attributeCount,
                              bool depthTest, bool depthWrite,
                              VkCompareOp depthCompare, bool blend) const {
        VkShaderModule vert = createShader(vertWords, vertBytes);
        VkShaderModule frag = createShader(fragWords, fragBytes);
        VkPipelineShaderStageCreateInfo stages[2]{};
        stages[0].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
        stages[0].module = vert;
        stages[0].pName = "main";
        stages[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
        stages[1].module = frag;
        stages[1].pName = "main";

        VkPipelineVertexInputStateCreateInfo vertexInput{
            VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};
        vertexInput.vertexBindingDescriptionCount = bindingCount;
        vertexInput.pVertexBindingDescriptions = bindings;
        vertexInput.vertexAttributeDescriptionCount = attributeCount;
        vertexInput.pVertexAttributeDescriptions = attributes;

        VkPipelineInputAssemblyStateCreateInfo assembly{
            VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
        assembly.topology = topology;

        VkPipelineViewportStateCreateInfo viewport{
            VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO};
        viewport.viewportCount = 1;
        viewport.scissorCount = 1;

        VkPipelineRasterizationStateCreateInfo raster{
            VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
        raster.polygonMode = VK_POLYGON_MODE_FILL;
        raster.cullMode = VK_CULL_MODE_NONE;
        raster.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
        raster.lineWidth = 1.0f;

        VkPipelineMultisampleStateCreateInfo multisample{
            VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO};
        multisample.rasterizationSamples = samples;

        VkPipelineDepthStencilStateCreateInfo depth{
            VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO};
        depth.depthTestEnable = depthTest ? VK_TRUE : VK_FALSE;
        depth.depthWriteEnable = depthWrite ? VK_TRUE : VK_FALSE;
        depth.depthCompareOp = depthCompare;

        VkPipelineColorBlendAttachmentState blendAttachment{};
        blendAttachment.colorWriteMask =
            VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
            VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
        blendAttachment.blendEnable = blend ? VK_TRUE : VK_FALSE;
        if (blend) {
            blendAttachment.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
            blendAttachment.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
            blendAttachment.colorBlendOp = VK_BLEND_OP_ADD;
            blendAttachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
            blendAttachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ZERO;
            blendAttachment.alphaBlendOp = VK_BLEND_OP_ADD;
        }
        VkPipelineColorBlendStateCreateInfo blendState{
            VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
        blendState.attachmentCount = 1;
        blendState.pAttachments = &blendAttachment;

        const VkDynamicState dynamicStates[] = {VK_DYNAMIC_STATE_VIEWPORT,
                                                VK_DYNAMIC_STATE_SCISSOR};
        VkPipelineDynamicStateCreateInfo dynamic{
            VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO};
        dynamic.dynamicStateCount = 2;
        dynamic.pDynamicStates = dynamicStates;

        VkGraphicsPipelineCreateInfo info{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
        info.stageCount = 2;
        info.pStages = stages;
        info.pVertexInputState = &vertexInput;
        info.pInputAssemblyState = &assembly;
        info.pViewportState = &viewport;
        info.pRasterizationState = &raster;
        info.pMultisampleState = &multisample;
        info.pDepthStencilState = &depth;
        info.pColorBlendState = &blendState;
        info.pDynamicState = &dynamic;
        info.layout = pipelineLayout;
        info.renderPass = renderPass;
        info.subpass = 0;

        VkPipeline pipeline;
        check(vk.vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &info, nullptr, &pipeline),
              "vkCreateGraphicsPipelines");
        vk.vkDestroyShaderModule(device, vert, nullptr);
        vk.vkDestroyShaderModule(device, frag, nullptr);
        return pipeline;
    }

    void createPipelines() {
        // Mesh: per-vertex position+normal, per-instance matrix rows + color.
        const VkVertexInputBindingDescription meshBindings[] = {
            {0, sizeof(MeshVertex), VK_VERTEX_INPUT_RATE_VERTEX},
            {1, sizeof(Instance), VK_VERTEX_INPUT_RATE_INSTANCE}};
        const VkVertexInputAttributeDescription meshAttributes[] = {
            {0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0},
            {1, 0, VK_FORMAT_R32G32B32_SFLOAT, 12},
            {2, 1, VK_FORMAT_R32G32B32A32_SFLOAT, 0},
            {3, 1, VK_FORMAT_R32G32B32A32_SFLOAT, 16},
            {4, 1, VK_FORMAT_R32G32B32A32_SFLOAT, 32},
            {5, 1, VK_FORMAT_R32G32B32A32_SFLOAT, 48}};
        meshPipeline = createPipeline(
            mesh_vert_spv, sizeof(mesh_vert_spv), mesh_frag_spv, sizeof(mesh_frag_spv),
            VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST, meshBindings, 2, meshAttributes, 6,
            true, true, VK_COMPARE_OP_LESS, false);

        // Sky: fullscreen triangle at far depth, drawn after opaque geometry.
        skyPipeline = createPipeline(
            sky_vert_spv, sizeof(sky_vert_spv), sky_frag_spv, sizeof(sky_frag_spv),
            VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST, nullptr, 0, nullptr, 0,
            true, false, VK_COMPARE_OP_LESS_OR_EQUAL, false);

        const VkVertexInputBindingDescription lineBinding{
            0, sizeof(LineVertex), VK_VERTEX_INPUT_RATE_VERTEX};
        const VkVertexInputAttributeDescription lineAttributes[] = {
            {0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0},
            {1, 0, VK_FORMAT_R32G32B32A32_SFLOAT, 12}};
        linePipeline = createPipeline(
            line_vert_spv, sizeof(line_vert_spv), line_frag_spv, sizeof(line_frag_spv),
            VK_PRIMITIVE_TOPOLOGY_LINE_LIST, &lineBinding, 1, lineAttributes, 2,
            true, false, VK_COMPARE_OP_LESS_OR_EQUAL, true);

        const VkVertexInputBindingDescription uiBinding{
            0, sizeof(UiVertex), VK_VERTEX_INPUT_RATE_VERTEX};
        const VkVertexInputAttributeDescription uiAttributes[] = {
            {0, 0, VK_FORMAT_R32G32_SFLOAT, 0},
            {1, 0, VK_FORMAT_R32G32B32A32_SFLOAT, 8}};
        uiPipeline = createPipeline(
            ui_vert_spv, sizeof(ui_vert_spv), ui_frag_spv, sizeof(ui_frag_spv),
            VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST, &uiBinding, 1, uiAttributes, 2,
            false, false, VK_COMPARE_OP_ALWAYS, true);
    }

    void createCommandsAndSync() {
        VkCommandPoolCreateInfo poolInfo{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
        poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
        poolInfo.queueFamilyIndex = queueFamily;
        check(vk.vkCreateCommandPool(device, &poolInfo, nullptr, &commandPool),
              "vkCreateCommandPool");
        VkCommandBufferAllocateInfo alloc{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
        alloc.commandPool = commandPool;
        alloc.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        alloc.commandBufferCount = 1;
        check(vk.vkAllocateCommandBuffers(device, &alloc, &commandBuffer),
              "vkAllocateCommandBuffers");

        VkSemaphoreCreateInfo semInfo{VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO};
        check(vk.vkCreateSemaphore(device, &semInfo, nullptr, &imageAvailable), "semaphore");
        check(vk.vkCreateSemaphore(device, &semInfo, nullptr, &renderFinished), "semaphore");
        VkFenceCreateInfo fenceInfo{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
        fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;
        check(vk.vkCreateFence(device, &fenceInfo, nullptr, &inFlight), "fence");
    }

    // --- frame ---------------------------------------------------------------

    void drawFrame(const FrameGlobals& globals, const std::vector<DrawBatch>& batches,
                   const std::vector<LineVertex>& lines, const std::vector<UiVertex>& ui) {
        check(vk.vkWaitForFences(device, 1, &inFlight, VK_TRUE, UINT64_MAX), "wait fence");

        // Skip rendering entirely while minimized.
        if (window->width() <= 0 || window->height() <= 0) return;
        if (extent.width != static_cast<uint32_t>(window->width()) ||
            extent.height != static_cast<uint32_t>(window->height()))
            recreateSwapchain();

        uint32_t imageIndex = 0;
        VkResult acquire = vk.vkAcquireNextImageKHR(device, swapchain, UINT64_MAX,
                                                    imageAvailable, VK_NULL_HANDLE, &imageIndex);
        if (acquire == VK_ERROR_OUT_OF_DATE_KHR) {
            recreateSwapchain();
            return;
        }
        if (acquire != VK_SUCCESS && acquire != VK_SUBOPTIMAL_KHR)
            check(acquire, "vkAcquireNextImageKHR");

        // Upload per-frame data (single frame in flight: fence already waited).
        std::memcpy(uniform.mapped, &globals, sizeof(FrameGlobals));

        size_t instanceCount = 0;
        for (const DrawBatch& batch : batches) instanceCount += batch.instances.size();
        if (instanceCount > 0) {
            ensureCapacity(instanceBuffer, instanceCount * sizeof(Instance),
                           VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
            size_t offset = 0;
            for (const DrawBatch& batch : batches) {
                std::memcpy(static_cast<char*>(instanceBuffer.mapped) + offset * sizeof(Instance),
                            batch.instances.data(), batch.instances.size() * sizeof(Instance));
                offset += batch.instances.size();
            }
        }
        if (!lines.empty()) {
            ensureCapacity(lineBuffer, lines.size() * sizeof(LineVertex),
                           VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
            std::memcpy(lineBuffer.mapped, lines.data(), lines.size() * sizeof(LineVertex));
        }
        if (!ui.empty()) {
            ensureCapacity(uiBuffer, ui.size() * sizeof(UiVertex),
                           VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
            std::memcpy(uiBuffer.mapped, ui.data(), ui.size() * sizeof(UiVertex));
        }

        check(vk.vkResetFences(device, 1, &inFlight), "reset fence");
        check(vk.vkResetCommandBuffer(commandBuffer, 0), "reset command buffer");
        VkCommandBufferBeginInfo begin{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
        check(vk.vkBeginCommandBuffer(commandBuffer, &begin), "begin command buffer");

        VkClearValue clears[2]{};
        clears[0].color = {{0.5f, 0.7f, 0.9f, 1.0f}};
        clears[1].depthStencil = {1.0f, 0};
        VkRenderPassBeginInfo passBegin{VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO};
        passBegin.renderPass = renderPass;
        passBegin.framebuffer = framebuffers[imageIndex];
        passBegin.renderArea = {{0, 0}, extent};
        passBegin.clearValueCount = 2;
        passBegin.pClearValues = clears;
        vk.vkCmdBeginRenderPass(commandBuffer, &passBegin, VK_SUBPASS_CONTENTS_INLINE);

        // Negative-height viewport keeps OpenGL clip conventions (y up).
        VkViewport viewport{};
        viewport.x = 0.0f;
        viewport.y = static_cast<float>(extent.height);
        viewport.width = static_cast<float>(extent.width);
        viewport.height = -static_cast<float>(extent.height);
        viewport.minDepth = 0.0f;
        viewport.maxDepth = 1.0f;
        vk.vkCmdSetViewport(commandBuffer, 0, 1, &viewport);
        VkRect2D scissor{{0, 0}, extent};
        vk.vkCmdSetScissor(commandBuffer, 0, 1, &scissor);

        vk.vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                   pipelineLayout, 0, 1, &descriptorSet, 0, nullptr);

        // Opaque shapes first.
        if (instanceCount > 0) {
            vk.vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, meshPipeline);
            VkDeviceSize instanceOffset = 0;
            for (const DrawBatch& batch : batches) {
                if (batch.instances.empty() || batch.meshId >= meshes.size()) continue;
                const GpuMesh& mesh = meshes[batch.meshId];
                const VkBuffer buffers[] = {mesh.vertices.buffer, instanceBuffer.buffer};
                const VkDeviceSize offsets[] = {0, instanceOffset * sizeof(Instance)};
                vk.vkCmdBindVertexBuffers(commandBuffer, 0, 2, buffers, offsets);
                vk.vkCmdBindIndexBuffer(commandBuffer, mesh.indices.buffer, 0,
                                        VK_INDEX_TYPE_UINT32);
                vk.vkCmdDrawIndexed(commandBuffer, mesh.indexCount,
                                    static_cast<uint32_t>(batch.instances.size()), 0, 0, 0);
                instanceOffset += batch.instances.size();
            }
        }

        // Sky fills every pixel the geometry left open (depth-tested at far).
        vk.vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, skyPipeline);
        vk.vkCmdDraw(commandBuffer, 3, 1, 0, 0);

        if (!lines.empty()) {
            vk.vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, linePipeline);
            const VkDeviceSize zero = 0;
            vk.vkCmdBindVertexBuffers(commandBuffer, 0, 1, &lineBuffer.buffer, &zero);
            vk.vkCmdDraw(commandBuffer, static_cast<uint32_t>(lines.size()), 1, 0, 0);
        }

        if (!ui.empty()) {
            vk.vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, uiPipeline);
            const VkDeviceSize zero = 0;
            vk.vkCmdBindVertexBuffers(commandBuffer, 0, 1, &uiBuffer.buffer, &zero);
            vk.vkCmdDraw(commandBuffer, static_cast<uint32_t>(ui.size()), 1, 0, 0);
        }

        vk.vkCmdEndRenderPass(commandBuffer);
        check(vk.vkEndCommandBuffer(commandBuffer), "end command buffer");

        const VkPipelineStageFlags waitStage = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        VkSubmitInfo submit{VK_STRUCTURE_TYPE_SUBMIT_INFO};
        submit.waitSemaphoreCount = 1;
        submit.pWaitSemaphores = &imageAvailable;
        submit.pWaitDstStageMask = &waitStage;
        submit.commandBufferCount = 1;
        submit.pCommandBuffers = &commandBuffer;
        submit.signalSemaphoreCount = 1;
        submit.pSignalSemaphores = &renderFinished;
        check(vk.vkQueueSubmit(queue, 1, &submit, inFlight), "vkQueueSubmit");

        VkPresentInfoKHR present{VK_STRUCTURE_TYPE_PRESENT_INFO_KHR};
        present.waitSemaphoreCount = 1;
        present.pWaitSemaphores = &renderFinished;
        present.swapchainCount = 1;
        present.pSwapchains = &swapchain;
        present.pImageIndices = &imageIndex;
        VkResult presented = vk.vkQueuePresentKHR(queue, &present);
        if (presented == VK_ERROR_OUT_OF_DATE_KHR || presented == VK_SUBOPTIMAL_KHR)
            recreateSwapchain();
        else
            check(presented, "vkQueuePresentKHR");
    }

    ~Impl() {
        if (!device) {
            if (surface) vk.vkDestroySurfaceKHR(instance, surface, nullptr);
            if (instance) vk.vkDestroyInstance(instance, nullptr);
            return;
        }
        vk.vkDeviceWaitIdle(device);
        for (GpuMesh& mesh : meshes) {
            destroyBuffer(mesh.vertices);
            destroyBuffer(mesh.indices);
        }
        destroyBuffer(uniform);
        destroyBuffer(instanceBuffer);
        destroyBuffer(lineBuffer);
        destroyBuffer(uiBuffer);
        if (inFlight) vk.vkDestroyFence(device, inFlight, nullptr);
        if (imageAvailable) vk.vkDestroySemaphore(device, imageAvailable, nullptr);
        if (renderFinished) vk.vkDestroySemaphore(device, renderFinished, nullptr);
        if (commandPool) vk.vkDestroyCommandPool(device, commandPool, nullptr);
        if (meshPipeline) vk.vkDestroyPipeline(device, meshPipeline, nullptr);
        if (skyPipeline) vk.vkDestroyPipeline(device, skyPipeline, nullptr);
        if (linePipeline) vk.vkDestroyPipeline(device, linePipeline, nullptr);
        if (uiPipeline) vk.vkDestroyPipeline(device, uiPipeline, nullptr);
        if (pipelineLayout) vk.vkDestroyPipelineLayout(device, pipelineLayout, nullptr);
        if (descriptorPool) vk.vkDestroyDescriptorPool(device, descriptorPool, nullptr);
        if (setLayout) vk.vkDestroyDescriptorSetLayout(device, setLayout, nullptr);
        destroySwapchain();
        if (renderPass) vk.vkDestroyRenderPass(device, renderPass, nullptr);
        vk.vkDestroyDevice(device, nullptr);
        if (surface) vk.vkDestroySurfaceKHR(instance, surface, nullptr);
        if (instance) vk.vkDestroyInstance(instance, nullptr);
    }
};

Renderer::Renderer(Window& window) : impl_(std::make_unique<Impl>()) {
    impl_->window = &window;
    impl_->createInstanceAndDevice();
    impl_->createRenderPass();
    impl_->createSwapchain();
    impl_->createDescriptors();
    impl_->createPipelines();
    impl_->createCommandsAndSync();
}

Renderer::~Renderer() = default;

uint32_t Renderer::registerMesh(const CpuMesh& mesh) {
    GpuMesh gpu;
    gpu.vertices = impl_->createBuffer(mesh.vertices.size() * sizeof(MeshVertex),
                                       VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
    std::memcpy(gpu.vertices.mapped, mesh.vertices.data(),
                mesh.vertices.size() * sizeof(MeshVertex));
    gpu.indices = impl_->createBuffer(mesh.indices.size() * sizeof(uint32_t),
                                      VK_BUFFER_USAGE_INDEX_BUFFER_BIT);
    std::memcpy(gpu.indices.mapped, mesh.indices.data(),
                mesh.indices.size() * sizeof(uint32_t));
    gpu.indexCount = static_cast<uint32_t>(mesh.indices.size());
    impl_->meshes.push_back(gpu);
    return static_cast<uint32_t>(impl_->meshes.size() - 1);
}

void Renderer::render(const FrameGlobals& globals, const std::vector<DrawBatch>& batches,
                      const std::vector<LineVertex>& lines, const std::vector<UiVertex>& ui,
                      int, int) {
    impl_->drawFrame(globals, batches, lines, ui);
}

} // namespace sandbox
