#ifndef TALUS_PUSH_CONSTANTS_GLSL
#define TALUS_PUSH_CONSTANTS_GLSL

layout(push_constant) uniform PushConstants {
  mat4 model;
  uint materialIndex;
  uint recordIndex;
} push;

#endif
