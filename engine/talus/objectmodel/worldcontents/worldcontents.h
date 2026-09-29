#pragma once

#include <talus/objectmodel/entity/entity.h>
#include <talus/objectmodel/world/world.h>

#include <entt/core/type_info.hpp>

#include <string>
#include <string_view>
#include <vector>

namespace talus::objectmodel {

[[nodiscard]] std::vector<Entity> liveEntities(const World& world);

[[nodiscard]] std::string_view componentName(const entt::type_info& info);

template<typename T>
[[nodiscard]] std::string_view componentName() {
  return componentName(entt::type_id<T>());
}

[[nodiscard]] std::vector<std::string> componentNames(
  const World& world,
  Entity entity
);

[[nodiscard]] std::vector<std::string> registeredComponentNames(
  const World& world
);

[[nodiscard]] std::vector<std::string> authoredComponentNames();

[[nodiscard]] std::vector<std::string> transientComponentNames();

}
