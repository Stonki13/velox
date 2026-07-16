#include "window.h"

#include <GLFW/glfw3.h>

#include <stdexcept>

namespace sandbox {
namespace {

void glfwError(int, const char* description) {
    (void)description;
}

} // namespace

Window::Window(int width, int height, const char* title) : width_(width), height_(height) {
    glfwSetErrorCallback(glfwError);
    if (!glfwInit()) throw std::runtime_error("GLFW initialization failed");
    if (!glfwVulkanSupported()) {
        glfwTerminate();
        throw std::runtime_error("No Vulkan loader/driver available for GLFW");
    }

    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    native_ = glfwCreateWindow(width, height, title, nullptr, nullptr);
    if (!native_) {
        glfwTerminate();
        throw std::runtime_error("GLFW window creation failed");
    }

    glfwSetWindowUserPointer(native_, this);
    glfwSetFramebufferSizeCallback(native_, [](GLFWwindow* window, int w, int h) {
        auto* owner = static_cast<Window*>(glfwGetWindowUserPointer(window));
        if (owner) {
            owner->width_ = w;
            owner->height_ = h;
        }
    });
}

Window::~Window() {
    if (native_) glfwDestroyWindow(native_);
    glfwTerminate();
}

void Window::pollEvents() const { glfwPollEvents(); }
bool Window::shouldClose() const { return glfwWindowShouldClose(native_) != 0; }

} // namespace sandbox
