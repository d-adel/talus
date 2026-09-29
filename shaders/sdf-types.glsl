#ifndef TALUS_FIELD_TYPES_GLSL
#define TALUS_FIELD_TYPES_GLSL

struct Primitive {
  mat4 inverseModel;
  vec4 shape;
  uvec4 shapeConfig;
  vec4 bounds;
};

struct SdfRecordShared {
  mat4 model;
  mat4 inverseModel;
  vec4 voxelSize;
  uvec4 resolutionVolumeIndex;
  uvec4 config;
  vec4 localOrigin;
};

#endif
