#pragma once

#include <talus/objectmodel/trait/authored/authored.h>
#include <talus/objectmodel/trait/selfcontained/selfcontained.h>

#include <glm/glm.hpp>

#include <cmath>

namespace talus::objectmodel {

struct Environment {
  glm::vec3 skyTop{0.385f, 0.454f, 0.55f};
  glm::vec3 skyHorizon{0.6463f, 0.6558f, 0.6708f};
  glm::vec3 groundBottom{0.2f, 0.169f, 0.133f};

  float skyCurve = 4.0f;
  float groundCurve = 30.0f;

  float ambientEnergy = 0.55f;
  float occlusionStrength = 1.4f;
  float exposure = 1.0f;
};

template<>
inline constexpr bool selfContained<Environment> = true;

template<>
inline constexpr bool authored<Environment> = true;

inline glm::vec3 skyColour(
  const Environment& environment,
  const glm::vec3& direction
) {
  const float up = glm::clamp(direction.y, -1.0f, 1.0f);

  const glm::vec3 above = glm::mix(
    environment.skyTop,
    environment.skyHorizon,
    glm::clamp(std::pow(1.0f - up, environment.skyCurve), 0.0f, 1.0f)
  );

  const glm::vec3 below = glm::mix(
    environment.groundBottom,
    environment.skyHorizon,
    glm::clamp(std::pow(1.0f + up, environment.groundCurve), 0.0f, 1.0f)
  );

  return up >= 0.0f ? above : below;
}

}
