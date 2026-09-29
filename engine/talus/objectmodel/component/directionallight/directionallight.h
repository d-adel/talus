#pragma once

#include <talus/objectmodel/trait/authored/authored.h>
#include <talus/objectmodel/trait/selfcontained/selfcontained.h>

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>

namespace talus::objectmodel {

struct DirectionalLight {
  glm::vec3 direction{0.4f, 1.0f, 0.6f};

  float ambient = 0.06f;
};

template<>
inline constexpr bool selfContained<DirectionalLight> = true;

template<>
inline constexpr bool authored<DirectionalLight> = true;

inline glm::vec3 toLight(const DirectionalLight& light) {
  const float length = glm::length(light.direction);

  return length > glm::epsilon<float>() ? light.direction / length
                                        : glm::vec3(0.0f, 1.0f, 0.0f);
}

}
