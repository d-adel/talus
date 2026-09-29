#pragma once

#include <glm/glm.hpp>

#include <cstddef>
#include <cstdint>

namespace talus::renderer {

constexpr uint32_t MATERIAL_FLAG_DOUBLE_SIDED = 1u << 0;
constexpr uint32_t MATERIAL_FLAG_ALPHA_MASK = 1u << 1;
constexpr uint32_t MATERIAL_FLAG_ALPHA_BLEND = 1u << 2;

struct MaterialRecord {
  glm::vec4 baseColor;
  glm::vec4 emissive;

  float metallic;
  float roughness;
  float normalScale;
  float occlusionStrength;

  float alphaCutoff;
  uint32_t flags;
  uint32_t baseColorTexture;
  uint32_t baseColorSampler;

  uint32_t metallicRoughnessTexture;
  uint32_t metallicRoughnessSampler;
  uint32_t normalTexture;
  uint32_t normalSampler;

  uint32_t occlusionTexture;
  uint32_t occlusionSampler;
  uint32_t emissiveTexture;
  uint32_t emissiveSampler;
};

static_assert(
  offsetof(MaterialRecord, baseColor) == 0,
  "MaterialRecord must keep the std140 layout the shaders index by"
);
static_assert(
  offsetof(MaterialRecord, emissive) == 16,
  "MaterialRecord must keep the std140 layout the shaders index by"
);
static_assert(
  offsetof(MaterialRecord, metallic) == 32,
  "MaterialRecord must keep the std140 layout the shaders index by"
);
static_assert(
  offsetof(MaterialRecord, roughness) == 36,
  "MaterialRecord must keep the std140 layout the shaders index by"
);
static_assert(
  offsetof(MaterialRecord, normalScale) == 40,
  "MaterialRecord must keep the std140 layout the shaders index by"
);
static_assert(
  offsetof(MaterialRecord, occlusionStrength) == 44,
  "MaterialRecord must keep the std140 layout the shaders index by"
);
static_assert(
  offsetof(MaterialRecord, alphaCutoff) == 48,
  "MaterialRecord must keep the std140 layout the shaders index by"
);
static_assert(
  offsetof(MaterialRecord, flags) == 52,
  "MaterialRecord must keep the std140 layout the shaders index by"
);
static_assert(
  offsetof(MaterialRecord, baseColorTexture) == 56,
  "MaterialRecord must keep the std140 layout the shaders index by"
);
static_assert(
  offsetof(MaterialRecord, baseColorSampler) == 60,
  "MaterialRecord must keep the std140 layout the shaders index by"
);
static_assert(
  offsetof(MaterialRecord, metallicRoughnessTexture) == 64,
  "MaterialRecord must keep the std140 layout the shaders index by"
);
static_assert(
  offsetof(MaterialRecord, metallicRoughnessSampler) == 68,
  "MaterialRecord must keep the std140 layout the shaders index by"
);
static_assert(
  offsetof(MaterialRecord, normalTexture) == 72,
  "MaterialRecord must keep the std140 layout the shaders index by"
);
static_assert(
  offsetof(MaterialRecord, normalSampler) == 76,
  "MaterialRecord must keep the std140 layout the shaders index by"
);
static_assert(
  offsetof(MaterialRecord, occlusionTexture) == 80,
  "MaterialRecord must keep the std140 layout the shaders index by"
);
static_assert(
  offsetof(MaterialRecord, occlusionSampler) == 84,
  "MaterialRecord must keep the std140 layout the shaders index by"
);
static_assert(
  offsetof(MaterialRecord, emissiveTexture) == 88,
  "MaterialRecord must keep the std140 layout the shaders index by"
);
static_assert(
  offsetof(MaterialRecord, emissiveSampler) == 92,
  "MaterialRecord must keep the std140 layout the shaders index by"
);
static_assert(
  sizeof(MaterialRecord) == 96,
  "MaterialRecord must keep the std140 layout the shaders index by"
);

}
