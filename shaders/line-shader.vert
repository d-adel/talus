#version 450
#extension GL_GOOGLE_include_directive : require

#include "frame-globals.glsl"

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec4 inColour;

layout(location = 0) out vec4 tint;

void main() {
  tint = inColour;

  gl_Position = camera.viewProj * vec4(inPosition, 1.0);
}
