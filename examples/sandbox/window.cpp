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

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
#endif
    native_ = glfwCreateWindow(width, height, title, nullptr, nullptr);
    if (!native_) {
        glfwTerminate();
        throw std::runtime_error("GLFW window creation failed");
    }

    glfwMakeContextCurrent(native_);
    glfwSwapInterval(1);
    glfwSetWindowUserPointer(native_, this);
    glfwSetFramebufferSizeCallback(native_, [](GLFWwindow* window, int w, int h) {
        auto* owner = static_cast<Window*>(glfwGetWindowUserPointer(window));
        if (owner) {
            owner->width_ = w > 0 ? w : 1;
            owner->height_ = h > 0 ? h : 1;
        }
    });
}

Window::~Window() {
    if (native_) glfwDestroyWindow(native_);
    glfwTerminate();
}

void Window::pollEvents() const { glfwPollEvents(); }
void Window::swapBuffers() const { glfwSwapBuffers(native_); }
bool Window::shouldClose() const { return glfwWindowShouldClose(native_) != 0; }

} // namespace sandbox
