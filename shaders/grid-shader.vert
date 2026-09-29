#version 450
#extension GL_GOOGLE_include_directive : require

#include "frame-globals.glsl"
#include "grid-push.glsl"

layout(location = 0) in vec3 inPosition;
layout(location = 1) in float inKind;

layout(location = 0) out vec3 world;
layout(location = 1) out vec4 tint;

vec3 toLinear(vec3 colour) {
  return mix(
    pow((colour + vec3(0.055)) * (1.0 / 1.055), vec3(2.4)),
    colour * (1.0 / 12.92),
    lessThan(colour, vec3(0.04045))
  );
}

void main() {
  world = inPosition;

  float decimals = grid.normal.w;

  if (inKind > 2.5) {
    tint = grid.axisColourV;
  } else if (inKind > 1.5) {
    tint = grid.axisColourU;
  } else if (inKind > 0.5) {
    tint = mix(grid.primary, grid.secondary, decimals);
  } else {
    tint = vec4(grid.secondary.rgb, grid.secondary.a * (1.0 - decimals));
  }

  tint = vec4(toLinear(tint.rgb), tint.a);

  gl_Position = camera.viewProj * vec4(inPosition, 1.0);
}
