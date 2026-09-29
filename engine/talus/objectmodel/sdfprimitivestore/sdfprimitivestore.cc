#include "sdfprimitivestore.ih"

namespace talus::objectmodel {

using talus::field::Primitive;

SdfPrimitiveStore::SdfPrimitiveStore()
    : m_primitives(talus::core::MAX_PRIMITIVE_POOL) {}

void SdfPrimitiveStore::connect(entt::registry& registry) {
  registry.on_destroy<SdfPrimitiveRange>()
    .connect<&SdfPrimitiveStore::onDestroyRange>(*this);
}

void SdfPrimitiveStore::onDestroyRange(
  entt::registry& registry,
  Entity entity
) {
  const SdfPrimitiveRange& range = registry.get<SdfPrimitiveRange>(entity);

  m_primitives.release({range.offset, range.capacity});
}

void SdfPrimitiveStore::refresh(entt::registry& registry, Entity entity) {
  const SdfPrimitiveRange& range = registry.get<SdfPrimitiveRange>(entity);

  const std::span<const Primitive> primitives =
    m_primitives.at({range.offset, range.capacity}, range.count);

  registry.patch<SdfGrid>(entity, [primitives](SdfGrid& object) {
    fitToPrimitives(object, primitives);
  });
}

bool SdfPrimitiveStore::emplaceRange(entt::registry& registry, Entity entity) {
  if (
    !registry.valid(entity) || !registry.all_of<SdfGrid>(entity)
    || registry.all_of<SdfPrimitiveRange>(entity)
  ) {
    return false;
  }

  registry.emplace<SdfPrimitiveRange>(entity);

  refresh(registry, entity);

  return true;
}

bool SdfPrimitiveStore::insert(
  entt::registry& registry,
  Entity entity,
  uint32_t index,
  const Primitive& primitive
) {
  if (!registry.valid(entity) || !registry.all_of<SdfGrid>(entity)) {
    return false;
  }

  SdfPrimitiveRange& range = registry.get_or_emplace<SdfPrimitiveRange>(entity);

  if (index > range.count || range.count >= talus::core::MAX_SDF_PRIMITIVES) {
    return false;
  }

  if (range.count == range.capacity) {
    const std::optional<RangeStore<Primitive>::Range> grown = m_primitives.grow(
      {range.offset, range.capacity},
      range.count,
      range.count + 1,
      talus::core::MAX_SDF_PRIMITIVES
    );

    if (!grown) {
      return false;
    }

    range.offset = grown->offset;
    range.capacity = grown->capacity;
  }

  const std::span<Primitive> elements =
    m_primitives.at({range.offset, range.capacity}, range.count + 1);

  std::move_backward(
    elements.begin() + index,
    elements.begin() + range.count,
    elements.begin() + range.count + 1
  );

  elements[index] = primitive;

  ++range.count;

  refresh(registry, entity);

  return true;
}

bool SdfPrimitiveStore::append(
  entt::registry& registry,
  Entity entity,
  const Primitive& primitive
) {
  if (!registry.valid(entity) || !registry.all_of<SdfGrid>(entity)) {
    return false;
  }

  const SdfPrimitiveRange* range = registry.try_get<SdfPrimitiveRange>(entity);

  return insert(
    registry,
    entity,
    range == nullptr ? 0 : range->count,
    primitive
  );
}

bool SdfPrimitiveStore::set(
  entt::registry& registry,
  Entity entity,
  uint32_t index,
  const Primitive& primitive
) {
  if (!registry.valid(entity) || !registry.all_of<SdfPrimitiveRange>(entity)) {
    return false;
  }

  const SdfPrimitiveRange& range = registry.get<SdfPrimitiveRange>(entity);

  if (index >= range.count) {
    return false;
  }

  m_primitives.at({range.offset, range.capacity}, range.count)[index] =
    primitive;

  refresh(registry, entity);

  return true;
}

bool SdfPrimitiveStore::remove(
  entt::registry& registry,
  Entity entity,
  uint32_t index
) {
  if (!registry.valid(entity) || !registry.all_of<SdfPrimitiveRange>(entity)) {
    return false;
  }

  SdfPrimitiveRange& range = registry.get<SdfPrimitiveRange>(entity);

  if (index >= range.count) {
    return false;
  }

  const std::span<Primitive> elements =
    m_primitives.at({range.offset, range.capacity}, range.count);

  std::move(
    elements.begin() + index + 1,
    elements.begin() + range.count,
    elements.begin() + index
  );

  --range.count;

  refresh(registry, entity);

  return true;
}

bool SdfPrimitiveStore::clear(entt::registry& registry, Entity entity) {
  if (!registry.valid(entity) || !registry.all_of<SdfPrimitiveRange>(entity)) {
    return false;
  }

  registry.get<SdfPrimitiveRange>(entity).count = 0;

  refresh(registry, entity);

  return true;
}

std::span<const Primitive> SdfPrimitiveStore::primitives(
  const entt::registry& registry,
  Entity entity
) const {
  const SdfPrimitiveRange* range = registry.try_get<SdfPrimitiveRange>(entity);

  if (range == nullptr) {
    return {};
  }

  return m_primitives.at({range->offset, range->capacity}, range->count);
}

uint32_t SdfPrimitiveStore::count(
  const entt::registry& registry,
  Entity entity
) const {
  const SdfPrimitiveRange* range = registry.try_get<SdfPrimitiveRange>(entity);

  return range == nullptr ? 0 : range->count;
}

std::span<const Primitive> SdfPrimitiveStore::pool() const noexcept {
  return m_primitives.pool();
}

}
