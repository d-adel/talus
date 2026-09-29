#include "meshrecordpacker.ih"

namespace talus::renderer {

std::span<const MeshRecord> MeshRecordPacker::pack(
  const talus::objectmodel::World& world,
  std::span<const talus::objectmodel::Entity> entities
) {
  const entt::registry& registry = world.registry();

  m_records.clear();

  for (const talus::objectmodel::Entity entity : entities) {
    m_records.push_back(
      {registry.get<talus::objectmodel::Mesh>(entity).index,
       registry.get<talus::objectmodel::Material>(entity).index,
       talus::objectmodel::model(
         talus::objectmodel::drawnPose(registry, entity)
       )}
    );

    m_records.back().entity = static_cast<uint32_t>(entity);
  }

  return m_records;
}

}
