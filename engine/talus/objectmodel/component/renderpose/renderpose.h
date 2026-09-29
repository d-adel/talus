#pragma once

#include <talus/objectmodel/trait/transient/transient.h>

#include <talus/objectmodel/component/pose/pose.h>
#include <talus/objectmodel/entity/entity.h>
#include <talus/objectmodel/trait/selfcontained/selfcontained.h>

#include <entt/entt.hpp>

namespace talus::objectmodel {

struct RenderPose {
  Pose pose;
};

template<>
inline constexpr bool selfContained<RenderPose> = true;

[[nodiscard]] const Pose& drawnPose(
  const entt::registry& registry,
  Entity entity
);

template<>
inline constexpr bool transient<RenderPose> = true;

}
