#pragma once

#include <talus/objectmodel/trait/authored/authored.h>
#include <talus/objectmodel/trait/assetbacked/assetbacked.h>

#include <talus/objectmodel/trait/selfcontained/selfcontained.h>

#include <cstdint>
#include <limits>

namespace talus::objectmodel {

struct Material {
  uint32_t index = UINT32_MAX;
};

template<>
inline constexpr bool selfContained<Material> = true;

template<>
inline constexpr bool authored<Material> = true;

template<>
inline constexpr bool assetBacked<Material> = true;

struct PhysicalMaterial {
  float friction = 0.5f;
  float restitution = 0.0f;
  float density = 1000.0f;
  float strength = std::numeric_limits<float>::infinity();
  float youngsModulus = 0.0f;
  float poissonsRatio = 0.3f;
  float stiffnessDamping = 0.0f;
  float yieldStress = std::numeric_limits<float>::infinity();
  float hardeningModulus = 0.0f;
};

}
