#ifndef TALUS_GRID_PUSH_GLSL
#define TALUS_GRID_PUSH_GLSL

layout(push_constant) uniform GridPush {
  vec4 primary;
  vec4 secondary;
  vec4 axisColourU;
  vec4 axisColourV;
  vec4 normal;
  vec4 fade;
} grid;

#endif
