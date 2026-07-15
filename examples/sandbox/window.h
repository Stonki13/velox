#pragma once

struct GLFWwindow;

namespace sandbox {

class Window {
public:
    Window(int width, int height, const char* title);
    ~Window();

    Window(const Window&) = delete;
    Window& operator=(const Window&) = delete;

    void pollEvents() const;
    void swapBuffers() const;
    bool shouldClose() const;
    int width() const { return width_; }
    int height() const { return height_; }
    GLFWwindow* native() const { return native_; }

private:
    GLFWwindow* native_ = nullptr;
    int width_ = 1;
    int height_ = 1;
};

} // namespace sandbox
