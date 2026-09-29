#include "componentschema.ih"

namespace talus::serialize {

namespace {

using talus::objectmodel::ComponentSpec;
using talus::objectmodel::ComponentType;
using talus::objectmodel::Entity;
using talus::objectmodel::FIELD_OPTIONAL;
using talus::objectmodel::FieldKind;
using talus::objectmodel::FieldLane;
using talus::objectmodel::FieldSpec;
using talus::objectmodel::INVALID_COMPONENT_TYPE;
using talus::objectmodel::World;

template<typename T>
struct KindOf;

template<>
struct KindOf<float> {
  static constexpr FieldKind kind = FieldKind::Float;
  static constexpr uint32_t flags = 0u;
};

template<>
struct KindOf<int32_t> {
  static constexpr FieldKind kind = FieldKind::Int;
  static constexpr uint32_t flags = 0u;
};

template<>
struct KindOf<uint32_t> {
  static constexpr FieldKind kind = FieldKind::UInt;
  static constexpr uint32_t flags = 0u;
};

template<>
struct KindOf<bool> {
  static constexpr FieldKind kind = FieldKind::Bool;
  static constexpr uint32_t flags = 0u;
};

template<>
struct KindOf<glm::vec2> {
  static constexpr FieldKind kind = FieldKind::Vec2;
  static constexpr uint32_t flags = 0u;
};

template<>
struct KindOf<glm::vec3> {
  static constexpr FieldKind kind = FieldKind::Vec3;
  static constexpr uint32_t flags = 0u;
};

template<>
struct KindOf<glm::vec4> {
  static constexpr FieldKind kind = FieldKind::Vec4;
  static constexpr uint32_t flags = 0u;
};

template<>
struct KindOf<glm::quat> {
  static constexpr FieldKind kind = FieldKind::Quat;
  static constexpr uint32_t flags = 0u;
};

template<>
struct KindOf<glm::uvec3> {
  static constexpr FieldKind kind = FieldKind::UVec3;
  static constexpr uint32_t flags = 0u;
};

template<>
struct KindOf<talus::objectmodel::VolumeElementType> {
  static constexpr FieldKind kind = FieldKind::UInt;
  static constexpr uint32_t flags = 0u;
};

template<>
struct KindOf<talus::objectmodel::Text> {
  static constexpr FieldKind kind = FieldKind::Text;
  static constexpr uint32_t flags = 0u;
};

template<typename U>
struct KindOf<std::optional<U>> {
  static constexpr FieldKind kind = KindOf<U>::kind;
  static constexpr uint32_t flags = FIELD_OPTIONAL;
};

void encode(const float& value, std::byte* at) {
  std::memcpy(at, &value, 4u);
}

void encode(const int32_t& value, std::byte* at) {
  std::memcpy(at, &value, 4u);
}

void encode(const uint32_t& value, std::byte* at) {
  std::memcpy(at, &value, 4u);
}

void encode(const bool& value, std::byte* at) {
  const uint8_t held = value ? 1u : 0u;
  std::memcpy(at, &held, 1u);
}

void encode(const glm::vec2& value, std::byte* at) {
  const float lanes[2]{value.x, value.y};
  std::memcpy(at, lanes, 8u);
}

void encode(const glm::vec3& value, std::byte* at) {
  const float lanes[3]{value.x, value.y, value.z};
  std::memcpy(at, lanes, 12u);
}

void encode(const glm::vec4& value, std::byte* at) {
  const float lanes[4]{value.x, value.y, value.z, value.w};
  std::memcpy(at, lanes, 16u);
}

void encode(const glm::quat& value, std::byte* at) {
  const float lanes[4]{value.x, value.y, value.z, value.w};
  std::memcpy(at, lanes, 16u);
}

void encode(const glm::uvec3& value, std::byte* at) {
  const uint32_t lanes[3]{value.x, value.y, value.z};
  std::memcpy(at, lanes, 12u);
}

void encode(const talus::objectmodel::VolumeElementType& value, std::byte* at) {
  const uint32_t held = static_cast<uint32_t>(value);
  std::memcpy(at, &held, 4u);
}

void encode(const talus::objectmodel::Text& value, std::byte* at) {
  std::memcpy(at, value.chars.data(), talus::objectmodel::TEXT_CAPACITY);
}

template<typename U>
void encode(const std::optional<U>& value, std::byte* at) {
  const uint8_t present = value.has_value() ? 1u : 0u;
  std::memcpy(at, &present, 1u);
  if (value) {
    encode(*value, at + 1);
  } else {
    std::memset(at + 1, 0, talus::objectmodel::fieldSize(KindOf<U>::kind));
  }
}

void decode(float& value, const std::byte* at) {
  std::memcpy(&value, at, 4u);
}

void decode(int32_t& value, const std::byte* at) {
  std::memcpy(&value, at, 4u);
}

void decode(uint32_t& value, const std::byte* at) {
  std::memcpy(&value, at, 4u);
}

void decode(talus::objectmodel::VolumeElementType& value, const std::byte* at) {
  uint32_t held = 0u;
  std::memcpy(&held, at, 4u);
  value = held == 0u ? talus::objectmodel::VolumeElementType::Hex
                     : talus::objectmodel::VolumeElementType::Tet;
}

void decode(bool& value, const std::byte* at) {
  uint8_t held = 0u;
  std::memcpy(&held, at, 1u);
  value = held != 0u;
}

void decode(glm::vec2& value, const std::byte* at) {
  float lanes[2];
  std::memcpy(lanes, at, 8u);
  value = glm::vec2(lanes[0], lanes[1]);
}

void decode(glm::vec3& value, const std::byte* at) {
  float lanes[3];
  std::memcpy(lanes, at, 12u);
  value = glm::vec3(lanes[0], lanes[1], lanes[2]);
}

void decode(glm::vec4& value, const std::byte* at) {
  float lanes[4];
  std::memcpy(lanes, at, 16u);
  value = glm::vec4(lanes[0], lanes[1], lanes[2], lanes[3]);
}

void decode(glm::quat& value, const std::byte* at) {
  float lanes[4];
  std::memcpy(lanes, at, 16u);
  value = glm::quat(lanes[3], lanes[0], lanes[1], lanes[2]);
}

void decode(glm::uvec3& value, const std::byte* at) {
  uint32_t lanes[3];
  std::memcpy(lanes, at, 12u);
  value = glm::uvec3(lanes[0], lanes[1], lanes[2]);
}

void decode(talus::objectmodel::Text& value, const std::byte* at) {
  std::memcpy(value.chars.data(), at, talus::objectmodel::TEXT_CAPACITY);
}

template<typename U>
void decode(std::optional<U>& value, const std::byte* at) {
  uint8_t present = 0u;
  std::memcpy(&present, at, 1u);
  if (present == 0u) {
    value.reset();
    return;
  }
  U held{};
  decode(held, at + 1);
  value = held;
}

template<typename T>
constexpr size_t memberCount() {
  if constexpr (std::is_empty_v<T>) {
    return 0u;
  } else {
    return glz::reflect<T>::size;
  }
}

template<typename T, size_t I>
using MemberType = std::remove_cvref_t<glz::refl_t<T, I>>;

template<size_t I, typename T>
decltype(auto) member(T& value) {
  if constexpr (glz::reflectable<T>) {
    return glz::get<I>(glz::to_tie(value));
  } else {
    return glz::get_member(value, glz::get<I>(glz::reflect<T>::values));
  }
}

template<typename T, typename Fn>
void forEachMember(Fn&& fn) {
  [&]<size_t... I>(std::index_sequence<I...>) {
    (fn.template operator()<I>(), ...);
  }(std::make_index_sequence<memberCount<T>()>{});
}

template<typename T>
constexpr uint32_t componentFlags() {
  if constexpr (talus::objectmodel::assetBacked<T>) {
    return COMPONENT_ASSET_BACKED;
  } else if constexpr (
    talus::objectmodel::selfContained<T> && !std::is_empty_v<T>
  ) {
    return COMPONENT_WRITABLE;
  } else {
    return 0u;
  }
}

template<typename T>
uint32_t byteSize() {
  uint32_t size = 0u;
  forEachMember<T>([&]<size_t I>() {
    using M = MemberType<T, I>;
    size += fieldByteSize(KindOf<M>::kind, KindOf<M>::flags);
  });
  return size;
}

template<typename T>
std::vector<std::byte> encodeAll(const T& value) {
  T copy = value;
  std::vector<std::byte> bytes(byteSize<T>());
  uint32_t offset = 0u;
  forEachMember<T>([&]<size_t I>() {
    using M = MemberType<T, I>;
    encode(member<I>(copy), bytes.data() + offset);
    offset += fieldByteSize(KindOf<M>::kind, KindOf<M>::flags);
  });
  return bytes;
}

template<typename T>
ComponentSchema schemaOf() {
  ComponentSchema schema;
  schema.spec.name = std::string(talus::objectmodel::componentName<T>());
  uint32_t offset = 0u;
  forEachMember<T>([&]<size_t I>() {
    using M = MemberType<T, I>;
    schema.spec.fields.push_back(
      FieldSpec{
        std::string(glz::reflect<T>::keys[I]),
        KindOf<M>::kind,
        offset,
        KindOf<M>::flags
      }
    );
    offset += fieldByteSize(KindOf<M>::kind, KindOf<M>::flags);
  });
  schema.spec.size = offset;
  schema.flags = componentFlags<T>();
  schema.defaults = encodeAll(T{});
  return schema;
}

bool wellFormed(
  FieldKind kind,
  uint32_t flags,
  std::span<const std::byte> bytes
) {
  size_t at = 0u;
  if ((flags & FIELD_OPTIONAL) != 0u) {
    uint8_t present = 0u;
    std::memcpy(&present, bytes.data(), 1u);
    if (present > 1u) {
      return false;
    }
    at = 1u;
  }
  const uint32_t lanes = talus::objectmodel::fieldLanes(kind);
  switch (talus::objectmodel::fieldLane(kind)) {
    case FieldLane::Float: {
      for (uint32_t lane = 0u; lane < lanes; ++lane) {
        float held = 0.0f;
        std::memcpy(&held, bytes.data() + at + lane * 4u, 4u);
        if (!std::isfinite(held)) {
          return false;
        }
      }
      return true;
    }
    case FieldLane::Bool: {
      uint8_t held = 0u;
      std::memcpy(&held, bytes.data() + at, 1u);
      return held <= 1u;
    }
    case FieldLane::Int:
    case FieldLane::UInt:
      return true;
    case FieldLane::Text:
      return std::memchr(bytes.data() + at, 0, lanes) != nullptr;
  }
  return false;
}

template<typename T>
WriteStatus writeAuthored(
  World& world,
  Entity entity,
  [[maybe_unused]] uint32_t field,
  [[maybe_unused]] FieldKind kind,
  [[maybe_unused]] std::span<const std::byte> bytes
) {
  if (!world.registry().all_of<T>(entity)) {
    return WriteStatus::NoComponent;
  }
  if constexpr ((componentFlags<T>() & COMPONENT_WRITABLE) == 0u) {
    return WriteStatus::ReadOnly;
  } else {
    if (field >= memberCount<T>()) {
      return WriteStatus::NoField;
    }
    T candidate = world.registry().get<T>(entity);
    WriteStatus status = WriteStatus::NoField;
    forEachMember<T>([&]<size_t I>() {
      if (I != field) {
        return;
      }
      using M = MemberType<T, I>;
      if (KindOf<M>::kind != kind) {
        status = WriteStatus::KindMismatch;
        return;
      }
      if (bytes.size() != fieldByteSize(kind, KindOf<M>::flags)) {
        status = WriteStatus::SizeMismatch;
        return;
      }
      if (!wellFormed(kind, KindOf<M>::flags, bytes)) {
        status = WriteStatus::Invalid;
        return;
      }
      decode(member<I>(candidate), bytes.data());
      status = WriteStatus::Written;
    });
    if (status != WriteStatus::Written) {
      return status;
    }
    if (!talus::objectmodel::valid(candidate)) {
      return WriteStatus::Invalid;
    }
    world.patch<T>(entity, [&](T& target) { target = candidate; });
    return WriteStatus::Written;
  }
}

WriteStatus writeDynamic(
  World& world,
  Entity entity,
  ComponentType type,
  uint32_t field,
  FieldKind kind,
  std::span<const std::byte> bytes
) {
  const ComponentSpec* spec = world.dynamic().spec(type);
  std::byte* at = world.dynamic().get(type, entity);
  if (spec == nullptr || at == nullptr) {
    return WriteStatus::NoComponent;
  }
  if (field >= spec->fields.size()) {
    return WriteStatus::NoField;
  }
  const FieldSpec& target = spec->fields[field];
  if (target.kind != kind) {
    return WriteStatus::KindMismatch;
  }
  const uint32_t size = fieldByteSize(kind, target.flags);
  if (bytes.size() != size || target.offset + size > spec->size) {
    return WriteStatus::SizeMismatch;
  }
  if (!wellFormed(kind, target.flags, bytes)) {
    return WriteStatus::Invalid;
  }
  std::memcpy(at + target.offset, bytes.data(), size);
  world.touch(entity);
  return WriteStatus::Written;
}

}

std::string_view describe(WriteStatus status) noexcept {
  switch (status) {
    case WriteStatus::Written:
      return "written";
    case WriteStatus::NoEntity:
      return "the entity is not alive";
    case WriteStatus::NoComponent:
      return "the entity has no such component";
    case WriteStatus::NoField:
      return "the component has no such field";
    case WriteStatus::KindMismatch:
      return "the value's kind does not match the field";
    case WriteStatus::SizeMismatch:
      return "the value's size does not match the field";
    case WriteStatus::ReadOnly:
      return "the component is read-only";
    case WriteStatus::Invalid:
      return "the value breaks the component's rules";
  }
  return "unknown";
}

uint32_t fieldByteSize(FieldKind kind, uint32_t flags) noexcept {
  return ((flags & FIELD_OPTIONAL) != 0u ? 1u : 0u)
         + talus::objectmodel::fieldSize(kind);
}

std::vector<ComponentSchema> authoredSchemas() {
  std::vector<ComponentSchema> schemas;
  talus::objectmodel::AuthoredComponents::each([&]<typename T>() {
    schemas.push_back(schemaOf<T>());
  });
  return schemas;
}

std::optional<ComponentSchema> componentSchema(
  const World& world,
  std::string_view component
) {
  std::optional<ComponentSchema> found;
  talus::objectmodel::AuthoredComponents::each([&]<typename T>() {
    if (!found && component == talus::objectmodel::componentName<T>()) {
      found = schemaOf<T>();
    }
  });
  if (found) {
    return found;
  }
  const ComponentType type = world.dynamic().find(component);
  if (type == INVALID_COMPONENT_TYPE) {
    return std::nullopt;
  }
  const ComponentSpec* spec = world.dynamic().spec(type);
  return ComponentSchema{
    *spec,
    COMPONENT_WRITABLE,
    std::vector<std::byte>(spec->size)
  };
}

std::optional<std::vector<std::byte>> componentBytes(
  const World& world,
  Entity entity,
  std::string_view component
) {
  if (!world.registry().valid(entity)) {
    return std::nullopt;
  }
  std::optional<std::vector<std::byte>> found;
  talus::objectmodel::AuthoredComponents::each([&]<typename T>() {
    if (found || component != talus::objectmodel::componentName<T>()) {
      return;
    }
    if (!world.registry().all_of<T>(entity)) {
      return;
    }
    if constexpr (std::is_empty_v<T>) {
      found = std::vector<std::byte>{};
    } else {
      found = encodeAll(world.registry().get<T>(entity));
    }
  });
  if (found) {
    return found;
  }
  const ComponentType type = world.dynamic().find(component);
  if (type == INVALID_COMPONENT_TYPE) {
    return std::nullopt;
  }
  const std::byte* at = world.dynamic().get(type, entity);
  if (at == nullptr) {
    return std::nullopt;
  }
  return std::vector<std::byte>(at, at + world.dynamic().spec(type)->size);
}

WriteStatus writeField(
  World& world,
  Entity entity,
  std::string_view component,
  uint32_t field,
  FieldKind kind,
  std::span<const std::byte> bytes
) {
  if (!world.registry().valid(entity)) {
    return WriteStatus::NoEntity;
  }
  std::optional<WriteStatus> status;
  talus::objectmodel::AuthoredComponents::each([&]<typename T>() {
    if (!status && component == talus::objectmodel::componentName<T>()) {
      status = writeAuthored<T>(world, entity, field, kind, bytes);
    }
  });
  if (status) {
    return *status;
  }
  const ComponentType type = world.dynamic().find(component);
  if (type == INVALID_COMPONENT_TYPE) {
    return WriteStatus::NoComponent;
  }
  return writeDynamic(world, entity, type, field, kind, bytes);
}

}

namespace {

using talus::objectmodel::ComponentType;
using talus::objectmodel::Entity;
using talus::objectmodel::INVALID_COMPONENT_TYPE;
using talus::objectmodel::World;

template<typename... Rs>
bool allPresent(
  const World& world,
  Entity entity,
  talus::objectmodel::ComponentList<Rs...>
) {
  return (world.registry().all_of<Rs>(entity) && ...);
}

template<typename... Rs>
bool anyPresent(
  const World& world,
  Entity entity,
  talus::objectmodel::ComponentList<Rs...>
) {
  return (world.registry().all_of<Rs>(entity) || ...);
}

template<typename T>
bool requirementsMet(const World& world, Entity entity) {
  return allPresent(
    world,
    entity,
    typename talus::objectmodel::Requires<T>::List{}
  );
}

template<typename T>
bool nothingExcluded(const World& world, Entity entity) {
  return !anyPresent(
    world,
    entity,
    typename talus::objectmodel::Excludes<T>::List{}
  );
}

template<typename T, typename... Rs>
constexpr bool listed(talus::objectmodel::ComponentList<Rs...>) {
  return (std::is_same_v<T, Rs> || ...);
}

template<typename T, typename... Us>
bool requiredByAny(
  const World& world,
  Entity entity,
  talus::objectmodel::ComponentList<Us...>
) {
  return (
    (world.registry().all_of<Us>(entity)
     && listed<T>(typename talus::objectmodel::Requires<Us>::List{}))
    || ...
  );
}

template<typename T>
bool requiredByAnother(const World& world, Entity entity) {
  return requiredByAny<T>(
    world,
    entity,
    talus::objectmodel::AuthoredComponents{}
  );
}

bool carries(const World& world, Entity entity, std::string_view name) {
  bool found = false;
  talus::objectmodel::AuthoredComponents::each([&]<typename T>() {
    if (!found && name == talus::objectmodel::componentName<T>()) {
      found = world.registry().all_of<T>(entity);
    }
  });
  if (found) {
    return true;
  }
  const ComponentType type = world.dynamic().find(name);
  return type != INVALID_COMPONENT_TYPE
         && world.dynamic().contains(type, entity);
}

bool namedRequirementsMet(
  const World& world,
  Entity entity,
  const talus::objectmodel::ComponentSpec& spec
) {
  for (const std::string& name : spec.requirements) {
    if (!carries(world, entity, name)) {
      return false;
    }
  }
  return true;
}

template<typename... Rs>
bool requiresNamed(
  std::string_view name,
  talus::objectmodel::ComponentList<Rs...>
) {
  return ((name == talus::objectmodel::componentName<Rs>()) || ...);
}

bool requiredByDynamic(
  const World& world,
  Entity entity,
  std::string_view name
) {
  for (ComponentType type = 0u; type < world.dynamic().types(); ++type) {
    if (!world.dynamic().contains(type, entity)) {
      continue;
    }
    for (const std::string& needed : world.dynamic().spec(type)->requirements) {
      if (needed == name) {
        return true;
      }
    }
  }
  return false;
}

}

namespace talus::serialize {

std::string removalBlocker(
  const World& world,
  Entity entity,
  std::string_view component
) {
  std::string blocker;
  talus::objectmodel::AuthoredComponents::each([&]<typename Us>() {
    if (!blocker.empty() || !world.registry().all_of<Us>(entity)) {
      return;
    }
    if (
      requiresNamed(
        component,
        typename talus::objectmodel::Requires<Us>::List{}
      )
    ) {
      blocker = std::string(talus::objectmodel::componentName<Us>());
    }
  });
  if (!blocker.empty()) {
    return blocker;
  }
  for (ComponentType type = 0u; type < world.dynamic().types(); ++type) {
    if (!world.dynamic().contains(type, entity)) {
      continue;
    }
    const talus::objectmodel::ComponentSpec* spec = world.dynamic().spec(type);
    for (const std::string& needed : spec->requirements) {
      if (needed == component) {
        return spec->name;
      }
    }
  }
  return blocker;
}

std::vector<std::string> componentNames(const World& world) {
  std::vector<std::string> names;
  talus::objectmodel::AuthoredComponents::each([&]<typename T>() {
    names.emplace_back(talus::objectmodel::componentName<T>());
  });
  for (ComponentType type = 0u; type < world.dynamic().types(); ++type) {
    names.push_back(world.dynamic().spec(type)->name);
  }
  return names;
}

bool canAddComponent(
  const World& world,
  Entity entity,
  std::string_view component
) {
  if (!world.registry().valid(entity)) {
    return false;
  }
  bool matched = false;
  bool allowed = false;
  talus::objectmodel::AuthoredComponents::each([&]<typename T>() {
    if (matched || component != talus::objectmodel::componentName<T>()) {
      return;
    }
    matched = true;
    allowed = !world.registry().all_of<T>(entity)
              && requirementsMet<T>(world, entity)
              && nothingExcluded<T>(world, entity);
  });
  if (matched) {
    return allowed;
  }
  const ComponentType type = world.dynamic().find(component);
  return type != INVALID_COMPONENT_TYPE
         && !world.dynamic().contains(type, entity)
         && namedRequirementsMet(world, entity, *world.dynamic().spec(type));
}

bool addComponent(World& world, Entity entity, std::string_view component) {
  if (!canAddComponent(world, entity, component)) {
    return false;
  }
  bool matched = false;
  bool added = false;
  talus::objectmodel::AuthoredComponents::each([&]<typename T>() {
    if (matched || component != talus::objectmodel::componentName<T>()) {
      return;
    }
    matched = true;
    if constexpr (talus::objectmodel::single<T>) {
      world.emplaceSingle<T>(entity, T{});
    } else {
      world.emplaceAuthored<T>(entity, T{});
      world.touch(entity);
    }
    added = true;
  });
  if (matched) {
    return added;
  }
  const ComponentType type = world.dynamic().find(component);
  const std::vector<std::byte> zeros(world.dynamic().spec(type)->size);
  if (!world.dynamic().emplace(type, entity, zeros)) {
    return false;
  }
  world.touch(entity);
  return true;
}

}

namespace talus::serialize {

bool removeComponent(World& world, Entity entity, std::string_view component) {
  if (!world.registry().valid(entity)) {
    return false;
  }
  bool matched = false;
  bool removed = false;
  talus::objectmodel::AuthoredComponents::each([&]<typename T>() {
    if (matched || component != talus::objectmodel::componentName<T>()) {
      return;
    }
    matched = true;
    if constexpr (
      std::is_same_v<T, talus::objectmodel::Pose>
      || std::is_same_v<T, talus::objectmodel::SdfGrid>
    ) {
      return;
    } else {
      if (
        requiredByAnother<T>(world, entity)
        || requiredByDynamic(world, entity, component)
      ) {
        return;
      }
      removed = world.removeAuthored<T>(entity);
    }
  });
  if (matched) {
    return removed;
  }
  const ComponentType type = world.dynamic().find(component);
  if (
    type == INVALID_COMPONENT_TYPE
    || requiredByDynamic(world, entity, component)
    || !world.dynamic().remove(type, entity)
  ) {
    return false;
  }
  world.touch(entity);
  return true;
}

}
