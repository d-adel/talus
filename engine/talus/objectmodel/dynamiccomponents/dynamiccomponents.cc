#include "dynamiccomponents.ih"

namespace talus::objectmodel {

uint32_t fieldSize(FieldKind kind) noexcept {
  const FieldKindInfo& info = fieldKindInfo(kind);

  return info.lanes * info.laneBytes;
}

uint32_t fieldLanes(FieldKind kind) noexcept {
  return fieldKindInfo(kind).lanes;
}

std::string_view fieldKindName(FieldKind kind) noexcept {
  return fieldKindInfo(kind).name;
}

FieldLane fieldLane(FieldKind kind) noexcept {
  return fieldKindInfo(kind).lane;
}

bool sameLayout(const ComponentSpec& a, const ComponentSpec& b) {
  if (
    a.name != b.name || a.size != b.size || a.fields.size() != b.fields.size()
  ) {
    return false;
  }

  for (size_t index = 0u; index < a.fields.size(); ++index) {
    const FieldSpec& first = a.fields[index];
    const FieldSpec& second = b.fields[index];

    if (
      first.name != second.name || first.kind != second.kind
      || first.offset != second.offset || first.flags != second.flags
    ) {
      return false;
    }
  }

  return true;
}

DynamicComponents::Store* DynamicComponents::store(
  ComponentType type
) noexcept {
  return type < m_stores.size() ? &m_stores[type] : nullptr;
}

const DynamicComponents::Store* DynamicComponents::store(
  ComponentType type
) const noexcept {
  return type < m_stores.size() ? &m_stores[type] : nullptr;
}

ComponentType DynamicComponents::declare(ComponentSpec spec) {
  if (spec.name.empty() || spec.size == 0u) {
    return INVALID_COMPONENT_TYPE;
  }

  for (const FieldSpec& field : spec.fields) {
    if (
      field.name.empty() || field.offset + fieldSize(field.kind) > spec.size
    ) {
      return INVALID_COMPONENT_TYPE;
    }
  }

  const ComponentType existing = find(spec.name);

  if (existing != INVALID_COMPONENT_TYPE) {
    if (!sameLayout(m_stores[existing].spec, spec)) {
      return INVALID_COMPONENT_TYPE;
    }

    m_stores[existing].spec.requirements = std::move(spec.requirements);

    return existing;
  }

  m_stores.push_back(Store{std::move(spec), {}, {}});

  return static_cast<ComponentType>(m_stores.size() - 1u);
}

ComponentType DynamicComponents::find(std::string_view name) const noexcept {
  for (size_t index = 0u; index < m_stores.size(); ++index) {
    if (m_stores[index].spec.name == name) {
      return static_cast<ComponentType>(index);
    }
  }

  return INVALID_COMPONENT_TYPE;
}

const ComponentSpec* DynamicComponents::spec(
  ComponentType type
) const noexcept {
  const Store* found = store(type);

  return found == nullptr ? nullptr : &found->spec;
}

size_t DynamicComponents::types() const noexcept {
  return m_stores.size();
}

bool DynamicComponents::emplace(
  ComponentType type,
  Entity entity,
  std::span<const std::byte> value
) {
  Store* found = store(type);

  if (found == nullptr || value.size() != found->spec.size) {
    return false;
  }

  if (found->set.contains(entity)) {
    const size_t at = found->set.index(entity) * found->spec.size;

    std::memcpy(found->bytes.data() + at, value.data(), value.size());

    return true;
  }

  found->set.push(entity);
  found->bytes.insert(found->bytes.end(), value.begin(), value.end());

  return true;
}

bool DynamicComponents::remove(ComponentType type, Entity entity) {
  Store* found = store(type);

  if (found == nullptr || !found->set.contains(entity)) {
    return false;
  }

  const size_t stride = found->spec.size;
  const size_t at = found->set.index(entity);
  const size_t last = found->set.size() - 1u;

  if (at != last) {
    std::memcpy(
      found->bytes.data() + at * stride,
      found->bytes.data() + last * stride,
      stride
    );
  }

  found->set.erase(entity);
  found->bytes.resize(last * stride);

  return true;
}

bool DynamicComponents::contains(
  ComponentType type,
  Entity entity
) const noexcept {
  const Store* found = store(type);

  return found != nullptr && found->set.contains(entity);
}

std::byte* DynamicComponents::get(ComponentType type, Entity entity) noexcept {
  Store* found = store(type);

  if (found == nullptr || !found->set.contains(entity)) {
    return nullptr;
  }

  return found->bytes.data() + found->set.index(entity) * found->spec.size;
}

const std::byte* DynamicComponents::get(
  ComponentType type,
  Entity entity
) const noexcept {
  const Store* found = store(type);

  if (found == nullptr || !found->set.contains(entity)) {
    return nullptr;
  }

  return found->bytes.data() + found->set.index(entity) * found->spec.size;
}

std::span<const Entity> DynamicComponents::entities(
  ComponentType type
) const noexcept {
  const Store* found = store(type);

  if (found == nullptr) {
    return {};
  }

  return std::span<const Entity>(found->set.data(), found->set.size());
}

std::span<std::byte> DynamicComponents::data(ComponentType type) noexcept {
  Store* found = store(type);

  return found == nullptr ? std::span<std::byte>() : std::span(found->bytes);
}

std::span<const std::byte> DynamicComponents::data(
  ComponentType type
) const noexcept {
  const Store* found = store(type);

  return found == nullptr ? std::span<const std::byte>()
                          : std::span<const std::byte>(found->bytes);
}

size_t DynamicComponents::count(ComponentType type) const noexcept {
  const Store* found = store(type);

  return found == nullptr ? 0u : found->set.size();
}

void DynamicComponents::clear() noexcept {
  m_stores.clear();
}

void DynamicComponents::clearEntities() noexcept {
  for (Store& held : m_stores) {
    held.set.clear();
    held.bytes.clear();
  }
}

void DynamicComponents::clear(Entity entity) {
  for (ComponentType type = 0u; type < m_stores.size(); ++type) {
    static_cast<void>(remove(type, entity));
  }
}

}
