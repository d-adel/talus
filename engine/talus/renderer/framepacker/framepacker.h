#pragma once

#include <talus/objectmodel/entity/entity.h>
#include <talus/objectmodel/world/world.h>
#include <talus/renderer/sdfrecordpacker/sdfrecordpacker.h>
#include <talus/renderer/frame/frame.h>
#include <talus/renderer/meshbuffers/meshbuffers.h>
#include <talus/renderer/meshrecordpacker/meshrecordpacker.h>

#include <functional>
#include <span>

namespace talus::renderer {

class FramePacker {
public:
  FramePacker(
    VolumePool& volumePool,
    SdfResidency& residency,
    SdfRecordTable& recordTable
  );

  FramePacker(const FramePacker&) = delete;
  FramePacker& operator=(const FramePacker&) = delete;
  FramePacker(FramePacker&&) = delete;
  FramePacker& operator=(FramePacker&&) = delete;

  void pack(
    Frame& frame,
    talus::objectmodel::World& world,
    std::span<const talus::objectmodel::Entity> sdfGrids,
    std::span<const MeshBuffers> meshBuffers,
    const std::function<void(talus::objectmodel::Entity)>& onFieldReplaced = {}
  );

  void commitBakes();

  [[nodiscard]] std::span<const talus::objectmodel::Entity>
  sdfRecordEntities() const noexcept;

private:
  SdfRecordTable& m_recordTable;
  talus::objectmodel::World* m_packed = nullptr;
  SdfRecordPacker m_sdfRecords;
  MeshRecordPacker m_meshRecords;
};

}
