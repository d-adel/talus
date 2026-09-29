#pragma once

#include <glm/glm.hpp>

#include <cstdint>

namespace talus::view {

struct GridPlane {
  glm::vec3 normal{0.0f, 1.0f, 0.0f};
  glm::vec3 axisU{1.0f, 0.0f, 0.0f};
  glm::vec3 axisV{0.0f, 0.0f, 1.0f};
};

struct GridStyle {
  glm::vec4 primary{0.56f, 0.56f, 0.56f, 0.5f};
  glm::vec4 secondary{0.38f, 0.38f, 0.38f, 0.5f};

  glm::vec4 axisColourU{0.96f, 0.20f, 0.32f, 1.0f};
  glm::vec4 axisColourV{0.16f, 0.55f, 0.96f, 1.0f};

  int32_t size = 200;
  int32_t steps = 8;

  float levelBias = -0.2f;
  int32_t levelMin = 0;
  int32_t levelMax = 2;
};

}
