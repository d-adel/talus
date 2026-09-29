#include "instantiate.ih"

namespace talus::objectmodel {

namespace {

std::string describe(const entt::registry& registry, Entity entity) {
  if (const auto* named = registry.try_get<Name>(entity)) {
    const std::string_view text = named->text.view();

    if (!text.empty()) {
      return std::string(text);
    }
  }

  return std::to_string(static_cast<uint32_t>(entity));
}

void requireOneBody(const entt::registry& registry, Entity entity) {
  const uint32_t bodies =
    static_cast<uint32_t>(registry.all_of<StaticBody>(entity))
    + static_cast<uint32_t>(registry.all_of<RigidBody>(entity))
    + static_cast<uint32_t>(registry.all_of<DeformableBody>(entity));

  if (bodies > 1u) {
    throw std::runtime_error(
      "The entity " + describe(registry, entity)
      + " carries more than one body component"
    );
  }
}

void requireElasticLanes(const World& source, Entity entity) {
  const entt::registry& registry = source.registry();

  if (!registry.all_of<DeformableBody>(entity)) {
    return;
  }

  const auto* held = registry.try_get<SharedSdf>(entity);

  if (held == nullptr || held->field == nullptr) {
    return;
  }

  const talus::field::SampledSdf& lattice = *held->field;

  const std::span<const PhysicalMaterial> materials = source.materials();

  const size_t samples =
    std::min(lattice.distances.size(), lattice.materials.size());

  for (size_t sample = 0u; sample != samples; ++sample) {
    if (lattice.distances[sample] >= 0.0f) {
      continue;
    }

    const uint32_t lane = lattice.materials[sample];

    if (lane < materials.size() && materials[lane].youngsModulus > 0.0f) {
      continue;
    }

    throw std::runtime_error(
      "The deformable " + describe(registry, entity) + " has no Young's"
      " modulus on material lane " + std::to_string(lane)
    );
  }
}

template<Authored T>
void carry(const entt::registry& source, World& destination, Entity entity) {
  if constexpr (std::is_empty_v<T>) {
    if (source.all_of<T>(entity)) {
      destination.emplaceAuthored<T>(entity, T{});
    }
  } else {
    if (const T* value = source.try_get<T>(entity)) {
      destination.emplaceAuthored<T>(entity, *value);
    }
  }
}

}

void validateBodies(const World& source) {
  const entt::registry& registry = source.registry();

  for (const Entity entity : liveEntities(source)) {
    requireOneBody(registry, entity);
    requireElasticLanes(source, entity);
  }
}

void instantiateWorld(const World& source, World& destination) {
  const entt::registry& registry = source.registry();

  for (ComponentType type = 0u; type < source.dynamic().types(); ++type) {
    const ComponentSpec* spec = source.dynamic().spec(type);

    if (
      spec == nullptr
      || destination.dynamic().declare(*spec) == INVALID_COMPONENT_TYPE
    ) {
      throw std::runtime_error("Cannot instantiate a declared component type");
    }
  }

  const auto reproduce = [&](Entity entity) {
    if (!destination.createAuthoredAt(entity)) {
      throw std::runtime_error(
        "Cannot instantiate an entity at its authored id"
      );
    }

    AuthoredComponents::each([&]<typename T>() {
      carry<T>(registry, destination, entity);
    });

    if (const auto* held = registry.try_get<SharedSdf>(entity)) {
      destination.adoptSampledSdf(entity, *held);
    }

    if (registry.all_of<Deformed>(entity)) {
      destination.emplaceOrReplace<Deformed>(entity, Deformed{});
    }

    for (ComponentType type = 0u; type < source.dynamic().types(); ++type) {
      const std::byte* value = source.dynamic().get(type, entity);

      if (value == nullptr) {
        continue;
      }

      const std::span<const std::byte> bytes(
        value,
        source.dynamic().spec(type)->size
      );

      if (!destination.dynamic().emplace(type, entity, bytes)) {
        throw std::runtime_error("Cannot instantiate a declared component");
      }
    }
  };

  for (const Entity entity : liveEntities(source)) {
    reproduce(entity);

    if (!registry.all_of<SdfGrid>(entity)) {
      continue;
    }

    if (
      registry.all_of<SdfPrimitiveRange>(entity)
      && !destination.emplacePrimitiveRange(entity)
    ) {
      throw std::runtime_error("Cannot instantiate an object's primitives");
    }

    for (const talus::field::Primitive& primitive : source.primitives(entity)) {
      if (!destination.addPrimitive(entity, primitive)) {
        throw std::runtime_error("Cannot instantiate an object's primitives");
      }
    }
  }

  for (const Collection& held : source.collections()) {
    if (!destination.createCollection(held.name)) {
      throw std::runtime_error("Cannot instantiate a collection");
    }

    for (const Entity member : held.members) {
      if (!destination.collect(held.name, member)) {
        throw std::runtime_error("Cannot instantiate a collection member");
      }
    }
  }
}

}
