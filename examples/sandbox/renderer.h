#pragma once

#include <array>
#include <string>
#include <vector>

#include <velox/world.h>

namespace sandbox {

class Renderer {
public:
    Renderer();
    ~Renderer();

    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;

    void render(const std::vector<velox::DebugLine>& lines,
                const std::array<float, 16>& viewProjection,
                const std::vector<std::string>& overlay,
                int width, int height);

private:
    unsigned int program_ = 0;
    unsigned int lineVao_ = 0;
    unsigned int lineVbo_ = 0;
    unsigned int textVao_ = 0;
    unsigned int textVbo_ = 0;
    int mvpLocation_ = -1;
};

} // namespace sandbox
