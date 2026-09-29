#include "sdfresidency.ih"

namespace talus::renderer {

SdfResidency::SdfResidency(
  VolumePool& pool,
  SdfRecordTable& table,
  gpu::Uploader& uploader
)
    : m_pool(pool), m_table(table), m_uploader(uploader) {}

void SdfResidency::attach(objectmodel::World& world) {
  world.onBakedVolumeReleased([this](uint32_t slot) { m_pool.release(slot); });
}

objectmodel::Entity SdfResidency::sdfOnSlot(
  const objectmodel::World& world,
  uint32_t slot
) const {
  const auto view = world.registry().view<const objectmodel::BakedVolume>();

  for (const objectmodel::Entity owner : view) {
    if (view.get<const objectmodel::BakedVolume>(owner).index != slot) {
      continue;
    }

    if (world.hasSampledSdf(owner)) {
      return owner;
    }
  }

  return objectmodel::INVALID_ENTITY;
}

void SdfResidency::releaseAll(objectmodel::World& world) {
  world.clearBakedVolumes();

  m_unplacedDirty.clear();

  ++m_releases;
}

uint64_t SdfResidency::releases() const noexcept {
  return m_releases;
}

std::vector<objectmodel::Entity> SdfResidency::takeUnplacedDirty() {
  return std::exchange(m_unplacedDirty, {});
}

void SdfResidency::upload(
  objectmodel::World& world,
  std::span<const std::pair<objectmodel::Entity, field::SampleBox>> dirty,
  core::Telemetry& telemetry
) {
  if (dirty.empty()) {
    return;
  }

  const auto dropped = telemetry.key("dentsDropped");

  const entt::registry& registry = world.registry();

  for (const auto& [entity, box] : dirty) {
    if (!registry.valid(entity)) {
      continue;
    }

    if (!registry.all_of<objectmodel::BakedVolume>(entity)) {
      m_unplacedDirty.push_back(entity);

      continue;
    }

    if (!registry.all_of<objectmodel::SharedSdf>(entity)) {
      continue;
    }

    const uint32_t shared =
      registry.get<objectmodel::BakedVolume>(entity).index;

    const field::SampledSdf& field = *world.sampledSdf(entity);

    const uint32_t slot = m_pool.makeUnique(m_uploader, shared, field);

    if (slot == UINT32_MAX) {
      telemetry.add(dropped, 1.0);

      continue;
    }

    if (slot != shared) {
      if (slot >= m_table.volumeCapacity()) {
        m_uploader.submit();
      }

      const auto images = m_pool.images(slot);

      m_table.registerVolume(
        images.distance.imageView(),
        images.material.imageView(),
        slot
      );

      world.setBakedVolume(entity, slot);
    } else {
      m_pool.forgetKey(slot);
    }

    m_pool.upload(m_uploader, slot, field, box);

    m_table.uploadBounds(m_uploader, slot, field);
  }

  m_uploader.submit();
}

void SdfResidency::flush(
  objectmodel::World& world,
  core::Telemetry& telemetry
) {
  if (world.sdfDirty().empty()) {
    return;
  }

  upload(world, world.sdfDirty(), telemetry);

  world.clearSdfDirty();
}

}
