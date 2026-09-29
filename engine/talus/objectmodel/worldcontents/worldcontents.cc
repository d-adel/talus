#include "worldcontents.ih"

namespace talus::objectmodel {

std::vector<Entity> liveEntities(const World& world) {
  std::vector<Entity> entities;

  const auto* storage = world.registry().storage<Entity>();

  if (storage == nullptr) {
    return entities;
  }

  entities.reserve(storage->free_list());

  for (const auto [entity] : storage->each()) {
    entities.push_back(entity);
  }

  std::sort(entities.begin(), entities.end(), [](Entity left, Entity right) {
    return entt::to_entity(left) < entt::to_entity(right);
  });

  return entities;
}

std::string_view componentName(const entt::type_info& info) {
  const std::string_view full = info.name();

  const auto separator = full.rfind("::");

  return separator == std::string_view::npos ? full
                                             : full.substr(separator + 2);
}

std::vector<std::string> componentNames(const World& world, Entity entity) {
  std::vector<std::string> names;

  const auto& registry = world.registry();

  for (const auto [id, storage] : registry.storage()) {
    if (id != storage.info().hash()) {
      continue;
    }

    if (!storage.contains(entity)) {
      continue;
    }

    names.emplace_back(componentName(storage.info()));
  }

  const DynamicComponents& dynamic = world.dynamic();

  for (ComponentType type = 0u; type < dynamic.types(); ++type) {
    if (dynamic.contains(type, entity)) {
      names.emplace_back(dynamic.spec(type)->name);
    }
  }

  std::sort(names.begin(), names.end());

  return names;
}

std::vector<std::string> registeredComponentNames(const World& world) {
  std::vector<std::string> names;

  for (const auto [id, storage] : world.registry().storage()) {
    if (id != storage.info().hash()) {
      continue;
    }

    names.emplace_back(componentName(storage.info()));
  }

  const DynamicComponents& dynamic = world.dynamic();

  for (ComponentType type = 0u; type < dynamic.types(); ++type) {
    names.emplace_back(dynamic.spec(type)->name);
  }

  return names;
}

std::vector<std::string> authoredComponentNames() {
  std::vector<std::string> names;

  AuthoredComponents::each([&]<typename T>() {
    names.emplace_back(componentName<T>());
  });

  return names;
}

std::vector<std::string> transientComponentNames() {
  std::vector<std::string> names;

  TransientComponents::each([&]<typename T>() {
    names.emplace_back(componentName<T>());
  });

  return names;
}

}
