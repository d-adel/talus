#pragma once

#include <talus/core/asset/asset.h>
#include <talus/core/config/config.h>

#include <talus/objectmodel/component/material/material.h>

#include <glm/glm.hpp>

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace talus::serialize {

inline constexpr uint32_t MATERIAL_VERSION = 1u;

struct StoredTextureSlot {
  talus::core::AssetId texture = talus::core::INVALID_ASSET;
  uint32_t sampler = talus::core::SAMPLER_LINEAR_REPEAT;
};

struct StoredMaterial {
  uint32_t version = MATERIAL_VERSION;

  glm::vec4 baseColor{1.0f};
  glm::vec4 emissive{0.0f, 0.0f, 0.0f, 1.0f};

  float metallic = 0.0f;
  float roughness = 1.0f;
  float normalScale = 1.0f;
  float occlusionStrength = 1.0f;
  float alphaCutoff = 0.5f;

  uint32_t flags = 0u;

  StoredTextureSlot baseColorTexture;
  StoredTextureSlot metallicRoughnessTexture;
  StoredTextureSlot normalTexture;
  StoredTextureSlot occlusionTexture;
  StoredTextureSlot emissiveTexture;

  float friction = 0.5f;
  float restitution = 0.0f;
  float density = 1000.0f;
  std::optional<float> strength;
  std::optional<float> youngsModulus;
  std::optional<float> poissonsRatio;
  std::optional<float> stiffnessDamping;
  std::optional<float> yieldStress;
  std::optional<float> hardeningModulus;
};

[[nodiscard]] std::string writeMaterial(const StoredMaterial& stored);

[[nodiscard]] bool readMaterial(std::string_view text, StoredMaterial& stored);

[[nodiscard]] objectmodel::PhysicalMaterial physicalMaterial(
  const StoredMaterial& stored
);

void setPhysicalMaterial(
  StoredMaterial& stored,
  const objectmodel::PhysicalMaterial& physical
);

}
