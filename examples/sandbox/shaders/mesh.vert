#version 450

layout(set = 0, binding = 0) uniform Globals {
    mat4 viewProj;
    mat4 invViewProj;
    vec4 cameraPos;
    vec4 sunDir;
} g;

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
// Per-instance model matrix rows (row-major 3x4) and color.
layout(location = 2) in vec4 inRow0;
layout(location = 3) in vec4 inRow1;
layout(location = 4) in vec4 inRow2;
layout(location = 5) in vec4 inColor; // rgb color, w: 1 = lit body, 2 = ground grid

layout(location = 0) out vec3 vNormal;
layout(location = 1) out vec3 vWorldPos;
layout(location = 2) out vec4 vColor;

void main() {
    vec3 world = vec3(dot(inRow0, vec4(inPosition, 1.0)),
                      dot(inRow1, vec4(inPosition, 1.0)),
                      dot(inRow2, vec4(inPosition, 1.0)));
    vec3 normal = vec3(dot(inRow0.xyz, inNormal),
                       dot(inRow1.xyz, inNormal),
                       dot(inRow2.xyz, inNormal));
    vNormal = normal;
    vWorldPos = world;
    vColor = inColor;
    gl_Position = g.viewProj * vec4(world, 1.0);
}
