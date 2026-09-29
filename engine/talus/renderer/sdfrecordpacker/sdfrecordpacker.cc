#include "sdfrecordpacker.ih"

namespace talus::renderer {

namespace {

using talus::objectmodel::Entity;
using talus::objectmodel::SdfGrid;

bool sameGrid(const SdfGrid& first, const SdfGrid& second) {
  return first.voxelSize == second.voxelSize
         && first.resolution == second.resolution
         && first.origin == second.origin && first.margin == second.margin
         && first.shadowCullMargin == second.shadowCullMargin;
}

bool samePrimitives(
  std::span<const talus::field::Primitive> first,
  std::span<const talus::field::Primitive> second
) {
  return first.size() == second.size()
         && (first.empty() || std::memcmp(first.data(), second.data(), first.size_bytes()) == 0);
}

std::shared_ptr<talus::field::SampledSdf> latticeHandle(
  const entt::registry& registry,
  Entity entity
) {
  const auto* held = registry.try_get<talus::objectmodel::SharedSdf>(entity);

  return held == nullptr ? nullptr : held->field;
}

bool sameHandle(
  const std::weak_ptr<talus::field::SampledSdf>& held,
  const std::shared_ptr<talus::field::SampledSdf>& now
) {
  return !held.owner_before(now) && !now.owner_before(held);
}

VolumeKey primitiveKey(
  const entt::registry& registry,
  Entity entity,
  const SdfGrid& grid,
  std::span<const talus::field::Primitive> primitives
) {
  if (registry.all_of<talus::objectmodel::Deformed>(entity)) {
    return VolumeKey{};
  }

  return volumeKey(
    primitives,
    grid.resolution,
    talus::objectmodel::gridMargin(grid, primitives),
    glm::vec3(grid.origin),
    grid.voxelSize
  );
}

bool latticeOnly(const entt::registry& registry, Entity entity) {
  return !registry.all_of<talus::objectmodel::SdfPrimitiveRange>(entity);
}

SdfGrid latticeGrid(
  const SdfGrid& grid,
  const talus::field::SampledSdf& lattice
) {
  SdfGrid drawn = grid;
  drawn.resolution = lattice.resolution;
  drawn.origin = glm::vec4(lattice.origin, 1.0f);
  drawn.voxelSize = lattice.voxelSize;

  return drawn;
}

}

SdfRecordPacker::SdfRecordPacker(
  VolumePool& volumePool,
  SdfResidency& residency,
  SdfRecordTable& recordTable
)
    : m_volumePool(volumePool),
      m_residency(residency),
      m_recordTable(recordTable) {}

uint32_t SdfRecordPacker::pack(
  talus::objectmodel::World& world,
  std::span<const talus::objectmodel::Entity> entities,
  uint32_t fieldRepresentation,
  const std::function<void(talus::objectmodel::Entity)>& onFieldReplaced
) {
  const entt::registry& registry = world.registry();

  forgetRetriedDrops();

  uint32_t recordIndex = 0;

  m_recordEntities.clear();
  m_recordEntities.reserve(entities.size());

  m_stillDropped.clear();

  for (const talus::objectmodel::Entity entity : entities) {
    const talus::objectmodel::SdfGrid& grid =
      registry.get<talus::objectmodel::SdfGrid>(entity);

    const std::span<const talus::field::Primitive> primitives =
      world.primitives(entity);

    const talus::objectmodel::SharedSdf* held =
      latticeOnly(registry, entity)
        ? registry.try_get<talus::objectmodel::SharedSdf>(entity)
        : nullptr;

    const talus::field::SampledSdf* lattice =
      held == nullptr ? nullptr : held->field.get();

    if (
      lattice != nullptr
      && registry.all_of<talus::objectmodel::BakedVolume>(entity)
      && !placedWithLattice(entity, held->field)
    ) {
      world.clearBakedVolume(entity);
    }

    const bool gpuBaked =
      lattice == nullptr && world.needsBake(entity)
      && !registry.all_of<talus::objectmodel::DeformableBody>(entity);

    const bool privately =
      gpuBaked && releaseForBake(world, entity, grid, primitives);

    if (!registry.all_of<talus::objectmodel::BakedVolume>(entity)) {
      const auto dropped = m_dropped.find(entity);

      if (
        dropped != m_dropped.end() && sameGrid(dropped->second.grid, grid)
        && samePrimitives(dropped->second.primitives, primitives)
        && sameHandle(dropped->second.lattice, latticeHandle(registry, entity))
      ) {
        m_stillDropped.insert(m_dropped.extract(dropped));

        continue;
      }

      if (!placeVolume(
            world,
            entity,
            grid,
            lattice,
            privately,
            onFieldReplaced
          )) {
        if (!m_volumePoolFullReported) {
          m_volumePoolFullReported = true;
          std::cout << "Volume pool full, object not drawn\n";
        }

        m_stillDropped.insert_or_assign(
          entity,
          DroppedField{
            grid,
            std::vector<talus::field::Primitive>(
              primitives.begin(),
              primitives.end()
            ),
            latticeHandle(registry, entity)
          }
        );

        continue;
      }
    }

    const auto* range =
      registry.try_get<talus::objectmodel::SdfPrimitiveRange>(entity);

    const uint32_t primitiveOffset = range == nullptr ? 0u : range->offset;

    const uint32_t primitiveCount = range == nullptr ? 0u : range->count;

    m_recordTable.setRecord(
      recordIndex,
      primitiveOffset,
      primitiveCount,
      talus::objectmodel::drawnPose(registry, entity),
      lattice == nullptr ? grid : latticeGrid(grid, *lattice),
      registry.get<talus::objectmodel::BakedVolume>(entity),
      lattice == nullptr ? fieldRepresentation : talus::core::FIELD_SAMPLED,
      primitives
    );

    if (world.needsBake(entity)) {
      if (!gpuBaked) {
        world.markBaked(entity);
      } else {
        m_recordTable.appendToBakeList(recordIndex);

        if (
          onFieldReplaced && world.needsResample(entity)
          && registry.all_of<
             talus::objectmodel::BodyId,
             talus::objectmodel::SharedSdf>(entity)
        ) {
          const talus::field::Aabb refit = talus::objectmodel::gridBox(
            primitives,
            talus::objectmodel::gridMargin(grid, primitives)
          );

          world.setSampledSdf(
            entity,
            talus::field::bake(
              primitives,
              refit.minimum,
              refit.maximum,
              grid.resolution
            )
          );

          onFieldReplaced(entity);
        }
      }
    }

    m_recordEntities.push_back(entity);
    ++recordIndex;
  }

  m_dropped.swap(m_stillDropped);
  m_droppedVolumes = static_cast<uint32_t>(m_dropped.size());

  return recordIndex;
}

void SdfRecordPacker::forgetRetriedDrops() {
  if (m_residency.releases() != m_releasesSeen) {
    m_releasesSeen = m_residency.releases();
    m_dropped.clear();
  }

  for (const talus::objectmodel::Entity entity :
       m_residency.takeUnplacedDirty()) {
    m_dropped.erase(entity);
  }
}

bool SdfRecordPacker::placedWithLattice(
  talus::objectmodel::Entity entity,
  const std::shared_ptr<talus::field::SampledSdf>& lattice
) const {
  const size_t slot = entt::to_entity(entity);

  if (slot >= m_placedLattices.size()) {
    return false;
  }

  const PlacedLattice& placed = m_placedLattices[slot];

  return placed.entity == entity && sameHandle(placed.lattice, lattice);
}

void SdfRecordPacker::rememberLattice(
  const entt::registry& registry,
  talus::objectmodel::Entity entity
) {
  const size_t slot = entt::to_entity(entity);

  if (slot >= m_placedLattices.size()) {
    m_placedLattices.resize(slot + 1u);
  }

  m_placedLattices[slot] = PlacedLattice{
    entity,
    registry.get<talus::objectmodel::SharedSdf>(entity).field
  };
}

bool SdfRecordPacker::releaseForBake(
  talus::objectmodel::World& world,
  talus::objectmodel::Entity entity,
  const talus::objectmodel::SdfGrid& grid,
  std::span<const talus::field::Primitive> primitives
) {
  const entt::registry& registry = world.registry();

  const auto* volume =
    registry.try_get<talus::objectmodel::BakedVolume>(entity);

  if (volume == nullptr) {
    return false;
  }

  const VolumeKey& held = m_volumePool.key(volume->index);

  if (
    !held.lattice && held == primitiveKey(registry, entity, grid, primitives)
  ) {
    return false;
  }

  if (held.lattice || m_volumePool.users(volume->index) > 1u) {
    world.clearBakedVolume(entity);

    return true;
  }

  m_volumePool.forgetKey(volume->index);

  return false;
}

bool SdfRecordPacker::placeVolume(
  talus::objectmodel::World& world,
  talus::objectmodel::Entity entity,
  const talus::objectmodel::SdfGrid& grid,
  const talus::field::SampledSdf* lattice,
  bool privately,
  const std::function<void(talus::objectmodel::Entity)>& onFieldReplaced
) {
  if (lattice != nullptr) {
    return placeLattice(world, entity, *lattice, onFieldReplaced);
  }

  const entt::registry& registry = world.registry();

  std::span<const talus::field::Primitive> primitives =
    world.primitives(entity);

  const talus::field::SampledSdf* carried = world.currentSampledSdf(entity);

  const bool reusable = carried != nullptr;

  const VolumeKey key =
    privately ? VolumeKey{} : primitiveKey(registry, entity, grid, primitives);

  uint32_t index = reusable ? UINT32_MAX : m_volumePool.acquire(key);

  const talus::objectmodel::Entity donor =
    index == UINT32_MAX ? talus::objectmodel::INVALID_ENTITY
                        : m_residency.sdfOnSlot(world, index);

  if (index != UINT32_MAX && donor == talus::objectmodel::INVALID_ENTITY) {
    m_volumePool.release(index);

    index = UINT32_MAX;
  }

  talus::field::SampledSdf baked;

  if (!reusable && donor == talus::objectmodel::INVALID_ENTITY) {
    const talus::field::Aabb box = talus::objectmodel::gridBox(
      primitives,
      talus::objectmodel::gridMargin(grid, primitives)
    );

    baked =
      talus::field::bake(primitives, box.minimum, box.maximum, grid.resolution);
  }

  if (index == UINT32_MAX) {
    index = m_volumePool.allocate(reusable ? *carried : baked, key);
  }

  if (index == UINT32_MAX) {
    return false;
  }

  registerVolume(world, entity, index);

  if (donor != talus::objectmodel::INVALID_ENTITY) {
    world.shareSampledSdf(donor, entity);
  } else if (!reusable) {
    world.setSampledSdf(entity, std::move(baked));
  }

  if (registry.all_of<talus::objectmodel::DeformableBody>(entity)) {
    m_recordTable.uploadBounds(index, *world.sampledSdf(entity));
  }

  if (onFieldReplaced) {
    onFieldReplaced(entity);
  }

  return true;
}

bool SdfRecordPacker::placeLattice(
  talus::objectmodel::World& world,
  talus::objectmodel::Entity entity,
  const talus::field::SampledSdf& lattice,
  const std::function<void(talus::objectmodel::Entity)>& onFieldReplaced
) {
  VolumeKey key = world.registry().all_of<talus::objectmodel::Deformed>(entity)
                    ? VolumeKey{}
                    : latticeKey(lattice);

  uint32_t index = m_volumePool.acquire(key);

  bool shared = false;

  if (index != UINT32_MAX) {
    const talus::objectmodel::Entity donor =
      m_residency.sdfOnSlot(world, index);

    const talus::field::SampledSdf* held =
      donor == talus::objectmodel::INVALID_ENTITY
          || !latticeOnly(world.registry(), donor)
        ? nullptr
        : world.currentSampledSdf(donor);

    shared = held != nullptr && sameLattice(*held, lattice);

    if (!shared) {
      m_volumePool.release(index);

      index = UINT32_MAX;
      key = VolumeKey{};
    }
  }

  if (index == UINT32_MAX) {
    index = m_volumePool.allocate(lattice, key);
  }

  if (index == UINT32_MAX) {
    return false;
  }

  registerVolume(world, entity, index);

  rememberLattice(world.registry(), entity);

  if (!shared) {
    m_recordTable.uploadBounds(index, lattice);
  }

  world.markBaked(entity);

  if (onFieldReplaced) {
    onFieldReplaced(entity);
  }

  return true;
}

void SdfRecordPacker::registerVolume(
  talus::objectmodel::World& world,
  talus::objectmodel::Entity entity,
  uint32_t index
) {
  const VolumeImages images = m_volumePool.images(index);

  m_recordTable.registerVolume(
    images.distance.imageView(),
    images.material.imageView(),
    index
  );

  world.setBakedVolume(entity, index);
}

std::span<const talus::objectmodel::Entity> SdfRecordPacker::
  recordEntities() const noexcept {
  return m_recordEntities;
}

uint32_t SdfRecordPacker::droppedVolumes() const noexcept {
  return m_droppedVolumes;
}

}
