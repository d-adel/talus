#pragma once

#include <talus/objectmodel/entity/entity.h>
#include <talus/objectmodel/world/world.h>
#include <talus/renderer/meshrecord/meshrecord.h>

#include <span>
#include <vector>

namespace talus::renderer {

class MeshRecordPacker {
public:
  MeshRecordPacker() = default;

  MeshRecordPacker(const MeshRecordPacker&) = delete;
  MeshRecordPacker& operator=(const MeshRecordPacker&) = delete;
  MeshRecordPacker(MeshRecordPacker&&) = delete;
  MeshRecordPacker& operator=(MeshRecordPacker&&) = delete;

  [[nodiscard]] std::span<const MeshRecord> pack(
    const talus::objectmodel::World& world,
    std::span<const talus::objectmodel::Entity> entities
  );

private:
  std::vector<MeshRecord> m_records;
};

}
