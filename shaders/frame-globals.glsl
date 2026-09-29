#ifndef TALUS_FRAME_GLOBALS_GLSL
#define TALUS_FRAME_GLOBALS_GLSL

layout(std140, set = 0, binding = 0) uniform CameraUniform {
  mat4 view;
  mat4 proj;
  mat4 viewProj;
  mat4 inverseViewProj;
  vec4 position;
} camera;

layout(std140, set = 0, binding = 1) uniform MarchParams {
  float epsilon;
  float maxDistance;
  float omega;

  float gradientEpsilon;
  float shadowMaxDistance;
  float shadowSharpness;
  uint maxIterations;
} params;

layout(std140, set = 0, binding = 2) uniform SceneCounts {
  uint sdfRecords;
} counts;

layout(std140, set = 0, binding = 3) uniform SceneLight {
  vec4 direction;
  vec4 skyTop;
  vec4 skyHorizon;
  vec4 groundBottom;
  vec4 shading;
} light;

#endif
