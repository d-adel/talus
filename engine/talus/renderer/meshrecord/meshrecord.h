#pragma once

#include <glm/glm.hpp>

#include <cstdint>

namespace talus::renderer {

struct MeshRecord {
  uint32_t mesh = UINT32_MAX;
  uint32_t material = UINT32_MAX;
  glm::mat4 model = glm::mat4(1.0f);
  uint32_t entity = UINT32_MAX;
};

}
