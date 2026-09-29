#version 450
#extension GL_GOOGLE_include_directive : require

#include "frame-globals.glsl"

layout(location = 0) in vec2 ndc;

layout(location = 0) out vec4 colour;

void main() {
  vec4 near = camera.inverseViewProj * vec4(ndc, 0.0, 1.0);
  vec4 far = camera.inverseViewProj * vec4(ndc, 1.0, 1.0);

  vec3 eye = normalize(far.xyz / far.w - near.xyz / near.w);

  float up = clamp(eye.y, -1.0, 1.0);

  vec3 sky = mix(light.skyTop.rgb,
                 light.skyHorizon.rgb,
                 clamp(pow(1.0 - up, light.skyTop.w), 0.0, 1.0));

  vec3 ground = mix(light.groundBottom.rgb,
                    light.skyHorizon.rgb,
                    clamp(pow(1.0 + up, light.skyHorizon.w), 0.0, 1.0));

  colour = vec4(mix(ground, sky, step(0.0, up)), 1.0);
}
