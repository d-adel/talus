#version 450
#extension GL_GOOGLE_include_directive : require

#include "frame-globals.glsl"
#include "grid-push.glsl"

layout(location = 0) in vec3 world;
layout(location = 1) in vec4 tint;

layout(location = 0) out vec4 colour;

void main() {
  vec3 normal = grid.normal.xyz;

  vec3 toCamera = camera.position.xyz - world;

  vec3 view = normalize(toCamera);

  float angleFade = smoothstep(0.05, 0.2, abs(dot(view, normal)));

  vec3 cameraOnPlane =
    camera.position.xyz - normal * dot(camera.position.xyz, normal);

  float distanceFade =
    1.0 - distance(world, cameraOnPlane) / max(grid.fade.x, 1e-6);

  distanceFade = smoothstep(0.02, 0.3, distanceFade);

  float alpha = tint.a * distanceFade * angleFade;

  if (alpha <= 0.002) {
    discard;
  }

  colour = vec4(tint.rgb, alpha);
}
