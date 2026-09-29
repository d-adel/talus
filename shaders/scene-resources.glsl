#ifndef TALUS_SCENE_RESOURCES_GLSL
#define TALUS_SCENE_RESOURCES_GLSL

const int MAX_TEXTURES = TALUS_MAX_TEXTURES;
const int MAX_SAMPLERS = TALUS_MAX_SAMPLERS;
const int MAX_MATERIALS = TALUS_MAX_MATERIALS;

struct Material {
  vec4 baseColor;
  vec4 emissive;

  float metallic;
  float roughness;
  float normalScale;
  float occlusionStrength;

  float alphaCutoff;
  uint flags;
  uint baseColorTexture;
  uint baseColorSampler;

  uint metallicRoughnessTexture;
  uint metallicRoughnessSampler;
  uint normalTexture;
  uint normalSampler;

  uint occlusionTexture;
  uint occlusionSampler;
  uint emissiveTexture;
  uint emissiveSampler;
};

layout(std140, set = 1, binding = 0) uniform MaterialTable {
  Material materials[MAX_MATERIALS];
} materialTable;

layout(set = 1, binding = 1) uniform texture2D textures[MAX_TEXTURES];
layout(set = 1, binding = 2) uniform sampler samplers[MAX_SAMPLERS];

#endif
