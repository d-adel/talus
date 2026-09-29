#pragma once

#include <talus/field/sampledsdf/sampledsdf.h>
#include <talus/core/telemetry/telemetry.h>
#include <talus/gpu/uploader/uploader.h>
#include <talus/objectmodel/entity/entity.h>
#include <talus/objectmodel/world/world.h>
#include <talus/renderer/sdfrecordtable/sdfrecordtable.h>
#include <talus/renderer/volumepool/volumepool.h>

#include <cstdint>
#include <span>
#include <utility>
#include <vector>

namespace talus::renderer {

class SdfResidency {
public:
  SdfResidency(
    VolumePool& pool,
    SdfRecordTable& table,
    gpu::Uploader& uploader
  );

  SdfResidency(const SdfResidency&) = delete;
  SdfResidency& operator=(const SdfResidency&) = delete;
  SdfResidency(SdfResidency&&) = delete;
  SdfResidency& operator=(SdfResidency&&) = delete;

  void attach(objectmodel::World& world);

  [[nodiscard]] objectmodel::Entity sdfOnSlot(
    const objectmodel::World& world,
    uint32_t slot
  ) const;

  void releaseAll(objectmodel::World& world);

  [[nodiscard]] uint64_t releases() const noexcept;

  [[nodiscard]] std::vector<objectmodel::Entity> takeUnplacedDirty();

  void upload(
    objectmodel::World& world,
    std::span<const std::pair<objectmodel::Entity, field::SampleBox>> dirty,
    core::Telemetry& telemetry
  );

  void flush(objectmodel::World& world, core::Telemetry& telemetry);

private:
  VolumePool& m_pool;
  SdfRecordTable& m_table;
  gpu::Uploader& m_uploader;

  std::vector<objectmodel::Entity> m_unplacedDirty;

  uint64_t m_releases = 0u;
};

}
