#pragma once

#include <talus/objectmodel/component/pose/pose.h>
#include <talus/objectmodel/trait/authored/authored.h>
#include <talus/objectmodel/trait/requires/requires.h>
#include <talus/objectmodel/trait/selfcontained/selfcontained.h>

#include <glm/glm.hpp>

namespace talus::objectmodel {

struct Support {
  glm::vec3 halfExtents{0.5f};
};

template<>
inline constexpr bool selfContained<Support> = true;

template<>
inline constexpr bool authored<Support> = true;

template<>
struct Requires<Support> {
  using List = ComponentList<Pose>;
};

}
