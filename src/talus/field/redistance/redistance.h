#pragma once

#include <talus/field/sampledsdf/sampledsdf.h>

#include <glm/glm.hpp>

#include <cstdint>
#include <span>

namespace talus::field {

float redistance(
  SampledSdf& field,
  const SampleBox& box,
  std::span<const uint8_t> damaged,
  std::span<float> values,
  uint32_t sweeps = 8u
);

float redistance(
  SampledSdf& field,
  const SampleBox& box,
  std::span<const uint8_t> damaged,
  uint32_t sweeps = 8u
);

float redistance(SampledSdf& field, const SampleBox& box);

float godunov(float a, float b, float c, const glm::vec3& h);

}
