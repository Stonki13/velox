#include "input.h"
#include "window.h"

#include <GLFW/glfw3.h>

namespace sandbox {
namespace {

bool keyDown(GLFWwindow* window, int key) {
    return glfwGetKey(window, key) == GLFW_PRESS;
}

} // namespace

void Input::update(const Window& window) {
    previous_ = down_;
    GLFWwindow* native = window.native();
    down_[static_cast<size_t>(Action::Forward)] = keyDown(native, GLFW_KEY_W);
    down_[static_cast<size_t>(Action::Backward)] = keyDown(native, GLFW_KEY_S);
    down_[static_cast<size_t>(Action::Left)] = keyDown(native, GLFW_KEY_A);
    down_[static_cast<size_t>(Action::Right)] = keyDown(native, GLFW_KEY_D);
    down_[static_cast<size_t>(Action::TurnLeft)] = keyDown(native, GLFW_KEY_Q);
    down_[static_cast<size_t>(Action::TurnRight)] = keyDown(native, GLFW_KEY_E);
    down_[static_cast<size_t>(Action::Spawn)] = keyDown(native, GLFW_KEY_SPACE);
    down_[static_cast<size_t>(Action::Reset)] = keyDown(native, GLFW_KEY_R);
    down_[static_cast<size_t>(Action::Pause)] = keyDown(native, GLFW_KEY_P);
    down_[static_cast<size_t>(Action::IncreaseSubsteps)] =
        keyDown(native, GLFW_KEY_EQUAL) || keyDown(native, GLFW_KEY_KP_ADD);
    down_[static_cast<size_t>(Action::DecreaseSubsteps)] =
        keyDown(native, GLFW_KEY_MINUS) || keyDown(native, GLFW_KEY_KP_SUBTRACT);
    down_[static_cast<size_t>(Action::Stack)] = keyDown(native, GLFW_KEY_F1);
    down_[static_cast<size_t>(Action::Rain)] = keyDown(native, GLFW_KEY_F2);
    down_[static_cast<size_t>(Action::Ragdoll)] = keyDown(native, GLFW_KEY_F3);
    down_[static_cast<size_t>(Action::Contraption)] = keyDown(native, GLFW_KEY_F4);

    previousMouseLook_ = mouseLook_;
    mouseLook_ = glfwGetMouseButton(native, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
    glfwGetCursorPos(native, &cursorX_, &cursorY_);
    mouseDeltaX_ = 0.0f;
    mouseDeltaY_ = 0.0f;
    if (mouseLook_ && previousMouseLook_ && haveCursor_) {
        mouseDeltaX_ = static_cast<float>(cursorX_ - previousCursorX_);
        mouseDeltaY_ = static_cast<float>(cursorY_ - previousCursorY_);
    }
    previousCursorX_ = cursorX_;
    previousCursorY_ = cursorY_;
    haveCursor_ = true;
}

bool Input::down(Action action) const { return down_[static_cast<size_t>(action)]; }

bool Input::pressed(Action action) const {
    const size_t index = static_cast<size_t>(action);
    return down_[index] && !previous_[index];
}

} // namespace sandbox
