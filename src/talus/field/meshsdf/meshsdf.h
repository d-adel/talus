#pragma once

#include <talus/field/sampledsdf/sampledsdf.h>

#include <glm/glm.hpp>

#include <cstdint>
#include <span>

namespace talus::field {

SampledSdf bakeMesh(
  std::span<const glm::vec3> vertices,
  std::span<const uint32_t> indices,
  const glm::vec3& minimum,
  const glm::vec3& maximum,
  const glm::uvec3& resolution,
  uint8_t material = 0u
);

}
