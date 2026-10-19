#pragma once
#include <cstdint>
#include <limits>

namespace talus {

enum class ElasticModel : uint32_t {
  Corotated,
  NeoHookean
};

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
  ElasticModel elasticModel = ElasticModel::Corotated;
  float hysteresis = 0.0f;
  float dampingRatio = std::numeric_limits<float>::infinity();
  float tearStrain = std::numeric_limits<float>::infinity();
  float plasticMax = std::numeric_limits<float>::infinity();
  float crushStress = std::numeric_limits<float>::infinity();
  float crushLimit = 1.0f;
};

}
