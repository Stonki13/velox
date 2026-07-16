#version 450

layout(set = 0, binding = 0) uniform Globals {
    mat4 viewProj;
    mat4 invViewProj;
    vec4 cameraPos;
    vec4 sunDir; // xyz normalized, points from scene toward the sun
} g;

layout(location = 0) in vec2 vNdc;
layout(location = 0) out vec4 outColor;

void main() {
    vec4 far = g.invViewProj * vec4(vNdc, 1.0, 1.0);
    vec3 ray = normalize(far.xyz / far.w - g.cameraPos.xyz);
    float up = ray.y;

    // Natural blue gradient: deep zenith blue to warm pale horizon.
    vec3 zenith = vec3(0.145, 0.335, 0.665);
    vec3 mid = vec3(0.410, 0.610, 0.870);
    vec3 horizon = vec3(0.845, 0.895, 0.925);
    float t = clamp(up, 0.0, 1.0);
    vec3 sky = mix(horizon, mid, smoothstep(0.0, 0.18, t));
    sky = mix(sky, zenith, smoothstep(0.18, 0.85, t));

    // Sun disc with a soft halo, consistent with the scene light direction.
    float sunDot = dot(ray, normalize(g.sunDir.xyz));
    float disc = smoothstep(0.9996, 0.99985, sunDot);
    float halo = pow(clamp(sunDot, 0.0, 1.0), 350.0) * 0.55 +
                 pow(clamp(sunDot, 0.0, 1.0), 24.0) * 0.10;
    vec3 sunColor = vec3(1.0, 0.965, 0.88);
    sky += sunColor * (disc * 6.0 + halo);

    // Below the horizon: gentle ground haze so the world never shows void.
    vec3 haze = vec3(0.72, 0.77, 0.78);
    float below = smoothstep(0.0, -0.35, up);
    sky = mix(sky, haze, below);

    outColor = vec4(sky, 1.0);
}
