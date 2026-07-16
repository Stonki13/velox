#version 450

layout(location = 0) out vec2 vNdc;

void main() {
    // Fullscreen triangle from the vertex index alone.
    vec2 ndc = vec2((gl_VertexIndex << 1) & 2, gl_VertexIndex & 2) * 2.0 - 1.0;
    vNdc = ndc;
    gl_Position = vec4(ndc, 1.0, 1.0);
}
