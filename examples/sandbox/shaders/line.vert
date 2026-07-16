#version 450

layout(set = 0, binding = 0) uniform Globals {
    mat4 viewProj;
    mat4 invViewProj;
    vec4 cameraPos;
    vec4 sunDir;
} g;

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec4 inColor;
layout(location = 0) out vec4 vColor;

void main() {
    vColor = inColor;
    gl_Position = g.viewProj * vec4(inPosition, 1.0);
}
