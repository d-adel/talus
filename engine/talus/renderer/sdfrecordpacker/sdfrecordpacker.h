#pragma once

#include <talus/objectmodel/component/sdfgrid/sdfgrid.h>
#include <talus/objectmodel/entity/entity.h>
#include <talus/objectmodel/world/world.h>
#include <talus/renderer/sdfrecordtable/sdfrecordtable.h>
#include <talus/renderer/sdfresidency/sdfresidency.h>
#include <talus/renderer/volumepool/volumepool.h>

#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <unordered_map>
#include <vector>

namespace talus::renderer {

class SdfRecordPacker {
public:
  SdfRecordPacker(
    VolumePool& volumePool,
    SdfResidency& residency,
    SdfRecordTable& recordTable
  );

  SdfRecordPacker(const SdfRecordPacker&) = delete;
  SdfRecordPacker& operator=(const SdfRecordPacker&) = delete;
  SdfRecordPacker(SdfRecordPacker&&) = delete;
  SdfRecordPacker& operator=(SdfRecordPacker&&) = delete;

  [[nodiscard]] uint32_t pack(
    talus::objectmodel::World& world,
    std::span<const talus::objectmodel::Entity> entities,
    uint32_t fieldRepresentation,
    const std::function<void(talus::objectmodel::Entity)>& onFieldReplaced = {}
  );

  [[nodiscard]] std::span<const talus::objectmodel::Entity>
  recordEntities() const noexcept;

  [[nodiscard]] uint32_t droppedVolumes() const noexcept;

private:
  struct DroppedField {
    talus::objectmodel::SdfGrid grid;
    std::vector<talus::field::Primitive> primitives;
    std::weak_ptr<talus::field::SampledSdf> lattice;
  };

  struct PlacedLattice {
    talus::objectmodel::Entity entity = talus::objectmodel::INVALID_ENTITY;
    std::weak_ptr<talus::field::SampledSdf> lattice;
  };

  void forgetRetriedDrops();

  [[nodiscard]] bool placedWithLattice(
    talus::objectmodel::Entity entity,
    const std::shared_ptr<talus::field::SampledSdf>& lattice
  ) const;

  void rememberLattice(
    const entt::registry& registry,
    talus::objectmodel::Entity entity
  );

  [[nodiscard]] bool releaseForBake(
    talus::objectmodel::World& world,
    talus::objectmodel::Entity entity,
    const talus::objectmodel::SdfGrid& grid,
    std::span<const talus::field::Primitive> primitives
  );

  [[nodiscard]] bool placeVolume(
    talus::objectmodel::World& world,
    talus::objectmodel::Entity entity,
    const talus::objectmodel::SdfGrid& grid,
    const talus::field::SampledSdf* lattice,
    bool privately,
    const std::function<void(talus::objectmodel::Entity)>& onFieldReplaced
  );

  [[nodiscard]] bool placeLattice(
    talus::objectmodel::World& world,
    talus::objectmodel::Entity entity,
    const talus::field::SampledSdf& lattice,
    const std::function<void(talus::objectmodel::Entity)>& onFieldReplaced
  );

  void registerVolume(
    talus::objectmodel::World& world,
    talus::objectmodel::Entity entity,
    uint32_t index
  );

  VolumePool& m_volumePool;
  SdfResidency& m_residency;
  SdfRecordTable& m_recordTable;

  std::vector<talus::objectmodel::Entity> m_recordEntities;

  std::unordered_map<talus::objectmodel::Entity, DroppedField> m_dropped;
  std::unordered_map<talus::objectmodel::Entity, DroppedField> m_stillDropped;

  std::vector<PlacedLattice> m_placedLattices;

  uint32_t m_droppedVolumes = 0u;

  uint64_t m_releasesSeen = 0u;

  bool m_volumePoolFullReported = false;
};

}
