#include "worldfile.ih"

namespace talus::serialize {

namespace {

constexpr uint32_t MATERIAL_LANE = 1u;

struct WorldOptions : glz::opts {
  bool new_lines_in_arrays = false;
  uint8_t indentation_width = 2;
};

bool carriesAnything(const StoredEntity& kept) {
  if (!kept.primitives.empty() || !kept.dynamic.empty()) {
    return true;
  }

  bool any = false;

  std::apply(
    [&](auto... bound) {
      ((any = any || (kept.*(bound.slot)).has_value()), ...);
    },
    PORTABLE_COMPONENTS
  );

  return any;
}

template<talus::objectmodel::Authored T>
std::optional<StoredType<T>> componentOf(
  const entt::registry& registry,
  talus::objectmodel::Entity entity,
  const talus::core::AssetDatabase& assets
) {
  if constexpr (talus::objectmodel::AssetBacked<T>) {
    const T* value = registry.try_get<T>(entity);

    return value == nullptr
             ? std::nullopt
             : std::optional<talus::core::AssetId>(assets.id<T>(value->index));
  } else if constexpr (std::is_empty_v<T>) {
    return registry.all_of<T>(entity) ? std::optional<T>(T{}) : std::nullopt;
  } else {
    const T* value = registry.try_get<T>(entity);

    return value == nullptr ? std::nullopt : std::optional<T>(*value);
  }
}

template<talus::objectmodel::Authored T>
bool putBack(
  talus::objectmodel::World& world,
  talus::objectmodel::Entity entity,
  const std::optional<StoredType<T>>& stored,
  const talus::core::AssetDatabase& assets
) {
  if (!stored.has_value()) {
    return true;
  }

  if constexpr (talus::objectmodel::AssetBacked<T>) {
    const uint32_t index = assets.index<T>(*stored);

    if (index == talus::core::UNBOUND_ASSET) {
      return false;
    }

    world.emplaceAuthored<T>(entity, T{index});
  } else {
    world.emplaceAuthored<T>(entity, *stored);
  }

  return true;
}

}

namespace {

void readLanes(
  const std::byte* at,
  talus::objectmodel::FieldKind kind,
  std::vector<double>& into
) {
  const uint32_t lanes = talus::objectmodel::fieldLanes(kind);

  for (uint32_t lane = 0u; lane < lanes; ++lane) {
    switch (talus::objectmodel::fieldLane(kind)) {
      case talus::objectmodel::FieldLane::Int: {
        int32_t held = 0;
        std::memcpy(&held, at + lane * 4u, 4u);
        into.push_back(static_cast<double>(held));
        break;
      }
      case talus::objectmodel::FieldLane::UInt: {
        uint32_t held = 0u;
        std::memcpy(&held, at + lane * 4u, 4u);
        into.push_back(static_cast<double>(held));
        break;
      }
      case talus::objectmodel::FieldLane::Bool: {
        uint8_t held = 0u;
        std::memcpy(&held, at + lane, 1u);
        into.push_back(held == 0u ? 0.0 : 1.0);
        break;
      }
      case talus::objectmodel::FieldLane::Float: {
        float held = 0.0f;
        std::memcpy(&held, at + lane * 4u, 4u);
        into.push_back(static_cast<double>(held));
        break;
      }
      case talus::objectmodel::FieldLane::Text: {
        uint8_t held = 0u;
        std::memcpy(&held, at + lane, 1u);
        into.push_back(static_cast<double>(held));
        break;
      }
    }
  }
}

void writeLanes(
  std::byte* at,
  talus::objectmodel::FieldKind kind,
  const std::vector<double>& from,
  size_t& cursor
) {
  const uint32_t lanes = talus::objectmodel::fieldLanes(kind);

  for (uint32_t lane = 0u; lane < lanes && cursor < from.size(); ++lane) {
    const double held = from[cursor++];

    switch (talus::objectmodel::fieldLane(kind)) {
      case talus::objectmodel::FieldLane::Int: {
        const int32_t value = static_cast<int32_t>(held);
        std::memcpy(at + lane * 4u, &value, 4u);
        break;
      }
      case talus::objectmodel::FieldLane::UInt: {
        const uint32_t value = static_cast<uint32_t>(held);
        std::memcpy(at + lane * 4u, &value, 4u);
        break;
      }
      case talus::objectmodel::FieldLane::Bool: {
        const uint8_t value = held == 0.0 ? 0u : 1u;
        std::memcpy(at + lane, &value, 1u);
        break;
      }
      case talus::objectmodel::FieldLane::Float: {
        const float value = static_cast<float>(held);
        std::memcpy(at + lane * 4u, &value, 4u);
        break;
      }
      case talus::objectmodel::FieldLane::Text: {
        const uint8_t value = static_cast<uint8_t>(held);
        std::memcpy(at + lane, &value, 1u);
        break;
      }
    }
  }
}

}

StoredWorld captureWorld(

  const talus::objectmodel::World& world,
  const talus::core::AssetDatabase& assets
) {
  const entt::registry& registry = world.registry();

  StoredWorld stored{};

  const auto keep = [&](talus::objectmodel::Entity entity) {
    StoredEntity kept{};

    std::apply(
      [&](auto... bound) {
        ((kept.*(bound.slot) = componentOf<typename decltype(bound)::Component>(
            registry,
            entity,
            assets
          )),
         ...);
      },
      PORTABLE_COMPONENTS
    );

    return kept;
  };

  std::vector<talus::objectmodel::Entity> order;
  std::unordered_map<talus::objectmodel::Entity, uint32_t> indices;

  for (const talus::objectmodel::Entity entity :
       registry.view<entt::entity>()) {
    order.push_back(entity);
  }

  std::reverse(order.begin(), order.end());

  for (const talus::objectmodel::Entity entity : order) {
    StoredEntity kept = keep(entity);

    if (registry.all_of<talus::objectmodel::SdfGrid>(entity)) {
      for (const talus::field::Primitive& primitive :
           world.primitives(entity)) {
        StoredPrimitive held{};

        held.primitive = primitive;

        held.material = assets.id<talus::objectmodel::Material>(
          primitive.shapeConfig[MATERIAL_LANE]
        );

        held.primitive.shapeConfig[MATERIAL_LANE] = 0u;

        kept.primitives.push_back(held);
      }
    }

    const talus::objectmodel::DynamicComponents& dynamic = world.dynamic();

    for (talus::objectmodel::ComponentType type = 0u; type < dynamic.types();
         ++type) {
      const std::byte* at = dynamic.get(type, entity);

      if (at == nullptr) {
        continue;
      }

      const talus::objectmodel::ComponentSpec* spec = dynamic.spec(type);

      StoredDynamic held{};
      held.type = spec->name;

      for (const talus::objectmodel::FieldSpec& field : spec->fields) {
        readLanes(at + field.offset, field.kind, held.values);
      }

      kept.dynamic.push_back(std::move(held));
    }

    if (!carriesAnything(kept)) {
      continue;
    }

    stored.entities.push_back(std::move(kept));
    indices[entity] = static_cast<uint32_t>(stored.entities.size() - 1u);
  }

  for (talus::objectmodel::ComponentType type = 0u;
       type < world.dynamic().types();
       ++type) {
    const talus::objectmodel::ComponentSpec* spec = world.dynamic().spec(type);

    StoredComponentType described{};
    described.name = spec->name;
    described.size = spec->size;

    for (const talus::objectmodel::FieldSpec& field : spec->fields) {
      described.fields.push_back(
        StoredField{field.name, static_cast<uint32_t>(field.kind), field.offset}
      );
    }

    stored.componentTypes.push_back(std::move(described));
  }

  for (const talus::objectmodel::Collection& held : world.collections()) {
    StoredCollection kept{};
    kept.name = held.name;

    for (const talus::objectmodel::Entity member : held.members) {
      const auto found = indices.find(member);

      if (found != indices.end()) {
        kept.members.push_back(found->second);
      }
    }

    stored.collections.push_back(std::move(kept));
  }

  return stored;
}

bool restoreWorld(
  const StoredWorld& stored,
  talus::objectmodel::World& world,
  const talus::core::AssetDatabase& assets
) {
  if (stored.version > WORLD_VERSION) {
    return false;
  }

  for (const StoredComponentType& described : stored.componentTypes) {
    talus::objectmodel::ComponentSpec spec{};
    spec.name = described.name;
    spec.size = described.size;

    for (const StoredField& field : described.fields) {
      spec.fields.push_back(
        talus::objectmodel::FieldSpec{
          field.name,
          static_cast<talus::objectmodel::FieldKind>(field.kind),
          field.offset
        }
      );
    }

    if (
      world.dynamic().declare(spec)
      == talus::objectmodel::INVALID_COMPONENT_TYPE
    ) {
      return false;
    }
  }

  std::vector<talus::objectmodel::Entity> created;

  for (const StoredEntity& kept : stored.entities) {
    const talus::objectmodel::Entity entity = world.createAuthored();
    created.push_back(entity);

    bool placed = true;

    std::apply(
      [&](auto... bound) {
        ((placed = putBack<typename decltype(bound)::Component>(
                     world,
                     entity,
                     kept.*(bound.slot),
                     assets
                   )
                   && placed),
         ...);
      },
      PORTABLE_COMPONENTS
    );

    if (!placed) {
      return false;
    }

    for (const StoredPrimitive& held : kept.primitives) {
      talus::field::Primitive primitive = held.primitive;

      const uint32_t index =
        assets.index<talus::objectmodel::Material>(held.material);

      if (index == talus::core::UNBOUND_ASSET) {
        return false;
      }

      primitive.shapeConfig[MATERIAL_LANE] = index;

      if (!world.addPrimitive(entity, primitive)) {
        return false;
      }
    }

    for (const StoredDynamic& held : kept.dynamic) {
      const talus::objectmodel::ComponentType type =
        world.dynamic().find(held.type);

      const talus::objectmodel::ComponentSpec* spec =
        world.dynamic().spec(type);

      if (spec == nullptr) {
        return false;
      }

      std::vector<std::byte> bytes(spec->size, std::byte{0});

      size_t cursor = 0u;

      for (const talus::objectmodel::FieldSpec& field : spec->fields) {
        writeLanes(
          bytes.data() + field.offset,
          field.kind,
          held.values,
          cursor
        );
      }

      if (!world.dynamic().emplace(type, entity, bytes)) {
        return false;
      }
    }
  }

  for (const StoredCollection& kept : stored.collections) {
    if (!world.createCollection(kept.name)) {
      return false;
    }

    for (const uint32_t index : kept.members) {
      if (index >= created.size() || !world.collect(kept.name, created[index])) {
        return false;
      }
    }
  }

  return true;
}

std::string writeWorld(const StoredWorld& stored) {
  std::string text;

  if (glz::write<WorldOptions{{.prettify = true}}>(stored, text)) {
    return std::string();
  }

  return text;
}

bool readWorld(std::string_view text, StoredWorld& stored) {
  return !glz::read<glz::opts{.error_on_unknown_keys = false}>(stored, text);
}

}
