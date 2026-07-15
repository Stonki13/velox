#include "renderer.h"

#include <GLFW/glfw3.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cstddef>
#include <stdexcept>
#include <string>

namespace sandbox {
namespace {

#ifndef GL_ARRAY_BUFFER
#define GL_ARRAY_BUFFER 0x8892
#endif
#ifndef GL_DYNAMIC_DRAW
#define GL_DYNAMIC_DRAW 0x88E8
#endif
#ifndef GL_VERTEX_SHADER
#define GL_VERTEX_SHADER 0x8B31
#endif
#ifndef GL_FRAGMENT_SHADER
#define GL_FRAGMENT_SHADER 0x8B30
#endif
#ifndef GL_COMPILE_STATUS
#define GL_COMPILE_STATUS 0x8B81
#endif
#ifndef GL_LINK_STATUS
#define GL_LINK_STATUS 0x8B82
#endif

struct Vertex {
    float x, y, z;
    float r, g, b, a;
};

using CreateShaderProc = GLuint (*)(GLenum);
using ShaderSourceProc = void (*)(GLuint, GLsizei, const char* const*, const GLint*);
using CompileShaderProc = void (*)(GLuint);
using GetShaderivProc = void (*)(GLuint, GLenum, GLint*);
using GetShaderInfoLogProc = void (*)(GLuint, GLsizei, GLsizei*, char*);
using CreateProgramProc = GLuint (*)();
using AttachShaderProc = void (*)(GLuint, GLuint);
using LinkProgramProc = void (*)(GLuint);
using GetProgramivProc = void (*)(GLuint, GLenum, GLint*);
using GetProgramInfoLogProc = void (*)(GLuint, GLsizei, GLsizei*, char*);
using DeleteShaderProc = void (*)(GLuint);
using DeleteProgramProc = void (*)(GLuint);
using GenVertexArraysProc = void (*)(GLsizei, GLuint*);
using BindVertexArrayProc = void (*)(GLuint);
using DeleteVertexArraysProc = void (*)(GLsizei, const GLuint*);
using GenBuffersProc = void (*)(GLsizei, GLuint*);
using BindBufferProc = void (*)(GLenum, GLuint);
using BufferDataProc = void (*)(GLenum, std::ptrdiff_t, const void*, GLenum);
using DeleteBuffersProc = void (*)(GLsizei, const GLuint*);
using VertexAttribPointerProc = void (*)(GLuint, GLint, GLenum, GLboolean, GLsizei, const void*);
using EnableVertexAttribArrayProc = void (*)(GLuint);
using UseProgramProc = void (*)(GLuint);
using GetUniformLocationProc = GLint (*)(GLuint, const char*);
using UniformMatrix4fvProc = void (*)(GLint, GLsizei, GLboolean, const GLfloat*);

struct Gl {
    CreateShaderProc createShader = nullptr;
    ShaderSourceProc shaderSource = nullptr;
    CompileShaderProc compileShader = nullptr;
    GetShaderivProc getShaderiv = nullptr;
    GetShaderInfoLogProc getShaderInfoLog = nullptr;
    CreateProgramProc createProgram = nullptr;
    AttachShaderProc attachShader = nullptr;
    LinkProgramProc linkProgram = nullptr;
    GetProgramivProc getProgramiv = nullptr;
    GetProgramInfoLogProc getProgramInfoLog = nullptr;
    DeleteShaderProc deleteShader = nullptr;
    DeleteProgramProc deleteProgram = nullptr;
    GenVertexArraysProc genVertexArrays = nullptr;
    BindVertexArrayProc bindVertexArray = nullptr;
    DeleteVertexArraysProc deleteVertexArrays = nullptr;
    GenBuffersProc genBuffers = nullptr;
    BindBufferProc bindBuffer = nullptr;
    BufferDataProc bufferData = nullptr;
    DeleteBuffersProc deleteBuffers = nullptr;
    VertexAttribPointerProc vertexAttribPointer = nullptr;
    EnableVertexAttribArrayProc enableVertexAttribArray = nullptr;
    UseProgramProc useProgram = nullptr;
    GetUniformLocationProc getUniformLocation = nullptr;
    UniformMatrix4fvProc uniformMatrix4fv = nullptr;
};

Gl gl;

template <typename T>
T load(const char* name) {
    auto proc = glfwGetProcAddress(name);
    if (!proc) throw std::runtime_error(std::string("OpenGL 3.3 function unavailable: ") + name);
    return reinterpret_cast<T>(proc);
}

void loadGl() {
    gl.createShader = load<CreateShaderProc>("glCreateShader");
    gl.shaderSource = load<ShaderSourceProc>("glShaderSource");
    gl.compileShader = load<CompileShaderProc>("glCompileShader");
    gl.getShaderiv = load<GetShaderivProc>("glGetShaderiv");
    gl.getShaderInfoLog = load<GetShaderInfoLogProc>("glGetShaderInfoLog");
    gl.createProgram = load<CreateProgramProc>("glCreateProgram");
    gl.attachShader = load<AttachShaderProc>("glAttachShader");
    gl.linkProgram = load<LinkProgramProc>("glLinkProgram");
    gl.getProgramiv = load<GetProgramivProc>("glGetProgramiv");
    gl.getProgramInfoLog = load<GetProgramInfoLogProc>("glGetProgramInfoLog");
    gl.deleteShader = load<DeleteShaderProc>("glDeleteShader");
    gl.deleteProgram = load<DeleteProgramProc>("glDeleteProgram");
    gl.genVertexArrays = load<GenVertexArraysProc>("glGenVertexArrays");
    gl.bindVertexArray = load<BindVertexArrayProc>("glBindVertexArray");
    gl.deleteVertexArrays = load<DeleteVertexArraysProc>("glDeleteVertexArrays");
    gl.genBuffers = load<GenBuffersProc>("glGenBuffers");
    gl.bindBuffer = load<BindBufferProc>("glBindBuffer");
    gl.bufferData = load<BufferDataProc>("glBufferData");
    gl.deleteBuffers = load<DeleteBuffersProc>("glDeleteBuffers");
    gl.vertexAttribPointer = load<VertexAttribPointerProc>("glVertexAttribPointer");
    gl.enableVertexAttribArray = load<EnableVertexAttribArrayProc>("glEnableVertexAttribArray");
    gl.useProgram = load<UseProgramProc>("glUseProgram");
    gl.getUniformLocation = load<GetUniformLocationProc>("glGetUniformLocation");
    gl.uniformMatrix4fv = load<UniformMatrix4fvProc>("glUniformMatrix4fv");
}

GLuint compile(GLenum type, const char* source) {
    GLuint shader = gl.createShader(type);
    gl.shaderSource(shader, 1, &source, nullptr);
    gl.compileShader(shader);
    GLint success = GL_FALSE;
    gl.getShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (success == GL_TRUE) return shader;

    std::array<char, 1024> log{};
    gl.getShaderInfoLog(shader, static_cast<GLsizei>(log.size()), nullptr, log.data());
    gl.deleteShader(shader);
    throw std::runtime_error(std::string("sandbox shader compilation failed: ") + log.data());
}

std::array<unsigned char, 7> glyph(char c) {
    switch (static_cast<char>(std::toupper(static_cast<unsigned char>(c)))) {
    case 'A': return {0x0e, 0x11, 0x11, 0x1f, 0x11, 0x11, 0x11};
    case 'B': return {0x1e, 0x11, 0x11, 0x1e, 0x11, 0x11, 0x1e};
    case 'C': return {0x0f, 0x10, 0x10, 0x10, 0x10, 0x10, 0x0f};
    case 'D': return {0x1e, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1e};
    case 'E': return {0x1f, 0x10, 0x10, 0x1e, 0x10, 0x10, 0x1f};
    case 'F': return {0x1f, 0x10, 0x10, 0x1e, 0x10, 0x10, 0x10};
    case 'G': return {0x0f, 0x10, 0x10, 0x17, 0x11, 0x11, 0x0f};
    case 'H': return {0x11, 0x11, 0x11, 0x1f, 0x11, 0x11, 0x11};
    case 'I': return {0x1f, 0x04, 0x04, 0x04, 0x04, 0x04, 0x1f};
    case 'J': return {0x01, 0x01, 0x01, 0x01, 0x11, 0x11, 0x0e};
    case 'K': return {0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11};
    case 'L': return {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1f};
    case 'M': return {0x11, 0x1b, 0x15, 0x15, 0x11, 0x11, 0x11};
    case 'N': return {0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11};
    case 'O': return {0x0e, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0e};
    case 'P': return {0x1e, 0x11, 0x11, 0x1e, 0x10, 0x10, 0x10};
    case 'Q': return {0x0e, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0d};
    case 'R': return {0x1e, 0x11, 0x11, 0x1e, 0x14, 0x12, 0x11};
    case 'S': return {0x0f, 0x10, 0x10, 0x0e, 0x01, 0x01, 0x1e};
    case 'T': return {0x1f, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04};
    case 'U': return {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0e};
    case 'V': return {0x11, 0x11, 0x11, 0x11, 0x11, 0x0a, 0x04};
    case 'W': return {0x11, 0x11, 0x11, 0x15, 0x15, 0x1b, 0x11};
    case 'X': return {0x11, 0x11, 0x0a, 0x04, 0x0a, 0x11, 0x11};
    case 'Y': return {0x11, 0x11, 0x0a, 0x04, 0x04, 0x04, 0x04};
    case 'Z': return {0x1f, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1f};
    case '0': return {0x0e, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0e};
    case '1': return {0x04, 0x0c, 0x04, 0x04, 0x04, 0x04, 0x0e};
    case '2': return {0x0e, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1f};
    case '3': return {0x1e, 0x01, 0x01, 0x0e, 0x01, 0x01, 0x1e};
    case '4': return {0x02, 0x06, 0x0a, 0x12, 0x1f, 0x02, 0x02};
    case '5': return {0x1f, 0x10, 0x10, 0x1e, 0x01, 0x01, 0x1e};
    case '6': return {0x0e, 0x10, 0x10, 0x1e, 0x11, 0x11, 0x0e};
    case '7': return {0x1f, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08};
    case '8': return {0x0e, 0x11, 0x11, 0x0e, 0x11, 0x11, 0x0e};
    case '9': return {0x0e, 0x11, 0x11, 0x0f, 0x01, 0x01, 0x0e};
    case ':': return {0x00, 0x04, 0x04, 0x00, 0x04, 0x04, 0x00};
    case '.': return {0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x06};
    case '/': return {0x01, 0x02, 0x02, 0x04, 0x08, 0x08, 0x10};
    case '+': return {0x00, 0x04, 0x04, 0x1f, 0x04, 0x04, 0x00};
    case '-': return {0x00, 0x00, 0x00, 0x1f, 0x00, 0x00, 0x00};
    case '=': return {0x00, 0x1f, 0x00, 0x1f, 0x00, 0x00, 0x00};
    default: return {};
    }
}

void appendText(std::vector<Vertex>& out, const std::vector<std::string>& lines,
                int width, int height) {
    const float pixelX = 2.0f / static_cast<float>(width);
    const float pixelY = 2.0f / static_cast<float>(height);
    const float glyphScale = 2.0f;
    const float left = -1.0f + 12.0f * pixelX;
    float top = 1.0f - 16.0f * pixelY;
    const auto addQuad = [&](float x0, float y0, float x1, float y1) {
        const Vertex a{x0, y0, 0.0f, 0.93f, 0.95f, 0.97f, 1.0f};
        const Vertex b{x1, y0, 0.0f, 0.93f, 0.95f, 0.97f, 1.0f};
        const Vertex c{x1, y1, 0.0f, 0.93f, 0.95f, 0.97f, 1.0f};
        const Vertex d{x0, y1, 0.0f, 0.93f, 0.95f, 0.97f, 1.0f};
        out.insert(out.end(), {a, b, c, a, c, d});
    };

    for (const std::string& line : lines) {
        float x = left;
        for (char character : line) {
            const auto rows = glyph(character);
            for (int row = 0; row < 7; ++row) {
                for (int column = 0; column < 5; ++column) {
                    if ((rows[row] & (1u << (4 - column))) == 0) continue;
                    const float x0 = x + static_cast<float>(column) * glyphScale * pixelX;
                    const float y0 = top - static_cast<float>(row + 1) * glyphScale * pixelY;
                    addQuad(x0, y0, x0 + glyphScale * pixelX, y0 + glyphScale * pixelY);
                }
            }
            x += 6.0f * glyphScale * pixelX;
        }
        top -= 10.0f * glyphScale * pixelY;
    }
}

constexpr std::array<float, 16> identityMatrix{
    1.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 1.0f};

} // namespace

Renderer::Renderer() {
    loadGl();
    constexpr const char* vertexSource = R"(
        #version 330 core
        layout(location = 0) in vec3 inPosition;
        layout(location = 1) in vec4 inColor;
        uniform mat4 uMvp;
        out vec4 color;
        void main() { gl_Position = uMvp * vec4(inPosition, 1.0); color = inColor; }
    )";
    constexpr const char* fragmentSource = R"(
        #version 330 core
        in vec4 color;
        out vec4 outColor;
        void main() { outColor = color; }
    )";

    const GLuint vertex = compile(GL_VERTEX_SHADER, vertexSource);
    const GLuint fragment = compile(GL_FRAGMENT_SHADER, fragmentSource);
    program_ = gl.createProgram();
    gl.attachShader(program_, vertex);
    gl.attachShader(program_, fragment);
    gl.linkProgram(program_);
    gl.deleteShader(vertex);
    gl.deleteShader(fragment);
    GLint linked = GL_FALSE;
    gl.getProgramiv(program_, GL_LINK_STATUS, &linked);
    if (linked != GL_TRUE) {
        std::array<char, 1024> log{};
        gl.getProgramInfoLog(program_, static_cast<GLsizei>(log.size()), nullptr, log.data());
        throw std::runtime_error(std::string("sandbox program link failed: ") + log.data());
    }
    mvpLocation_ = gl.getUniformLocation(program_, "uMvp");

    const auto configureBuffer = [&](GLuint& vao, GLuint& vbo) {
        gl.genVertexArrays(1, &vao);
        gl.genBuffers(1, &vbo);
        gl.bindVertexArray(vao);
        gl.bindBuffer(GL_ARRAY_BUFFER, vbo);
        gl.vertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), nullptr);
        gl.enableVertexAttribArray(0);
        gl.vertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                               reinterpret_cast<const void*>(3 * sizeof(float)));
        gl.enableVertexAttribArray(1);
    };
    configureBuffer(lineVao_, lineVbo_);
    configureBuffer(textVao_, textVbo_);
}

Renderer::~Renderer() {
    if (lineVbo_) gl.deleteBuffers(1, &lineVbo_);
    if (textVbo_) gl.deleteBuffers(1, &textVbo_);
    if (lineVao_) gl.deleteVertexArrays(1, &lineVao_);
    if (textVao_) gl.deleteVertexArrays(1, &textVao_);
    if (program_) gl.deleteProgram(program_);
}

void Renderer::render(const std::vector<velox::DebugLine>& lines,
                      const std::array<float, 16>& viewProjection,
                      const std::vector<std::string>& overlay,
                      int width, int height) {
    glViewport(0, 0, width, height);
    glClearColor(0.055f, 0.075f, 0.10f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glLineWidth(1.0f);

    std::vector<Vertex> lineVertices;
    lineVertices.reserve(lines.size() * 2);
    for (const velox::DebugLine& line : lines) {
        const float r = static_cast<float>((line.color >> 24) & 0xffu) / 255.0f;
        const float g = static_cast<float>((line.color >> 16) & 0xffu) / 255.0f;
        const float b = static_cast<float>((line.color >> 8) & 0xffu) / 255.0f;
        const float a = static_cast<float>(line.color & 0xffu) / 255.0f;
        lineVertices.push_back({line.a.x, line.a.y, line.a.z, r, g, b, a});
        lineVertices.push_back({line.b.x, line.b.y, line.b.z, r, g, b, a});
    }

    gl.useProgram(program_);
    gl.uniformMatrix4fv(mvpLocation_, 1, GL_FALSE, viewProjection.data());
    gl.bindVertexArray(lineVao_);
    gl.bindBuffer(GL_ARRAY_BUFFER, lineVbo_);
    gl.bufferData(GL_ARRAY_BUFFER, static_cast<std::ptrdiff_t>(lineVertices.size() * sizeof(Vertex)),
                  lineVertices.data(), GL_DYNAMIC_DRAW);
    if (!lineVertices.empty()) glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(lineVertices.size()));

    std::vector<Vertex> textVertices;
    textVertices.reserve(overlay.size() * 300);
    appendText(textVertices, overlay, width, height);
    glDisable(GL_DEPTH_TEST);
    gl.uniformMatrix4fv(mvpLocation_, 1, GL_FALSE, identityMatrix.data());
    gl.bindVertexArray(textVao_);
    gl.bindBuffer(GL_ARRAY_BUFFER, textVbo_);
    gl.bufferData(GL_ARRAY_BUFFER, static_cast<std::ptrdiff_t>(textVertices.size() * sizeof(Vertex)),
                  textVertices.data(), GL_DYNAMIC_DRAW);
    if (!textVertices.empty()) glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(textVertices.size()));
}

} // namespace sandbox
