#include "framepacker.ih"

namespace talus::renderer {

FramePacker::FramePacker(
  VolumePool& volumePool,
  SdfResidency& residency,
  SdfRecordTable& recordTable
)
    : m_recordTable(recordTable),
      m_sdfRecords(volumePool, residency, recordTable) {}

void FramePacker::pack(
  Frame& frame,
  talus::objectmodel::World& world,
  std::span<const talus::objectmodel::Entity> sdfGrids,
  std::span<const MeshBuffers> meshBuffers,
  const std::function<void(talus::objectmodel::Entity)>& onFieldReplaced
) {
  m_packed = &world;

  m_recordTable.newFrame();

  frame.sdfRecordCount = m_sdfRecords.pack(
    world,
    sdfGrids,
    frame.fieldRepresentation,
    onFieldReplaced
  );

  frame.droppedVolumes = m_sdfRecords.droppedVolumes();

  frame.sdfEntities = m_sdfRecords.recordEntities();

  frame.meshRecords = m_meshRecords.pack(world, world.meshes());
  frame.meshes = meshBuffers;
  frame.primitives = world.pool();

  const talus::objectmodel::Entity sun =
    talus::objectmodel::firstWith<talus::objectmodel::DirectionalLight>(world);

  if (sun != talus::objectmodel::INVALID_ENTITY) {
    frame.light =
      world.registry().get<const talus::objectmodel::DirectionalLight>(sun);
  }

  const talus::objectmodel::Entity sky =
    talus::objectmodel::firstWith<talus::objectmodel::Environment>(world);

  if (sky != talus::objectmodel::INVALID_ENTITY) {
    frame.environment =
      world.registry().get<const talus::objectmodel::Environment>(sky);
  }
}

void FramePacker::commitBakes() {
  if (m_packed == nullptr) {
    return;
  }

  const std::span<const talus::objectmodel::Entity> packed =
    m_sdfRecords.recordEntities();

  for (const uint32_t slot : m_recordTable.bakeList()) {
    m_packed->markBaked(packed[slot]);
  }

  m_packed = nullptr;
}

std::span<const talus::objectmodel::Entity> FramePacker::
  sdfRecordEntities() const noexcept {
  return m_sdfRecords.recordEntities();
}

}
