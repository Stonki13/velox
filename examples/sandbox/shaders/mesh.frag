#version 450

layout(set = 0, binding = 0) uniform Globals {
    mat4 viewProj;
    mat4 invViewProj;
    vec4 cameraPos;
    vec4 sunDir;
} g;

layout(location = 0) in vec3 vNormal;
layout(location = 1) in vec3 vWorldPos;
layout(location = 2) in vec4 vColor;
layout(location = 0) out vec4 outColor;

void main() {
    vec3 n = normalize(vNormal);
    vec3 sun = normalize(g.sunDir.xyz);
    vec3 view = normalize(g.cameraPos.xyz - vWorldPos);
    vec3 albedo = vColor.rgb;

    if (vColor.w > 1.5) {
        // Ground: light gray-green with fine grid lines fading with distance.
        vec2 cell = abs(fract(vWorldPos.xz) - 0.5);
        float lineNear = 1.0 - smoothstep(0.46, 0.5, max(cell.x, cell.y));
        vec2 cell5 = abs(fract(vWorldPos.xz * 0.2) - 0.5);
        float line5 = 1.0 - smoothstep(0.475, 0.5, max(cell5.x, cell5.y));
        float dist = length(vWorldPos.xz - g.cameraPos.xz);
        float fade = exp(-dist * 0.02);
        float grid = max(lineNear * 0.55, line5 * 0.8) * fade;
        albedo = mix(albedo, albedo * 0.72, grid);
    }

    float diffuse = max(dot(n, sun), 0.0);
    float ambient = 0.34;
    float rim = pow(1.0 - max(dot(n, view), 0.0), 3.0) * 0.16;
    vec3 lit = albedo * (ambient + diffuse * 0.78) + vec3(rim);

    // Distance haze toward the sky horizon color for depth cueing.
    float dist = length(vWorldPos - g.cameraPos.xyz);
    vec3 hazeColor = vec3(0.80, 0.86, 0.91);
    float haze = 1.0 - exp(-dist * 0.006);
    lit = mix(lit, hazeColor, haze);

    outColor = vec4(lit, 1.0);
}
