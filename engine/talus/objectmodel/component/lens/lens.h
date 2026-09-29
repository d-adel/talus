#pragma once

#include <talus/objectmodel/trait/authored/authored.h>
#include <talus/objectmodel/component/pose/pose.h>
#include <talus/objectmodel/trait/selfcontained/selfcontained.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace talus::objectmodel {

struct Lens {
  float verticalFov = 70.0f;
  float nearPlane = 0.1f;
  float farPlane = 10000.0f;
};

template<>
inline constexpr bool selfContained<Lens> = true;

template<>
inline constexpr bool authored<Lens> = true;

inline glm::mat4 projection(const Lens& lens, float aspect) {
  glm::mat4 matrix = glm::perspective(
    glm::radians(lens.verticalFov),
    aspect,
    lens.nearPlane,
    lens.farPlane
  );

  matrix[1][1] *= -1.0f;

  return matrix;
}

inline glm::mat4 view(const Pose& pose) {
  return glm::inverse(model(pose));
}

}
